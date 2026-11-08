"""CameraPipeline ownership tests with the real direct MonoIsp pixel path.

Only TagDetectLwc is a test double: it checks ISP bypass pixels and input
framing, and supplies a drain interval. These are control tests, not an
end-to-end compressor or pose-accuracy test.

Run with ``python -m unittest discover -s tests/lutcam -p test_camera_control.py``.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class _CameraControlRunner:
    paired_mode = False
    use_dsp_counters = False
    native_isp = False

    @classmethod
    def setUpClass(cls):
        cls.build_dir = tempfile.TemporaryDirectory(prefix="lutcam-control-")
        cls.addClassCleanup(cls.build_dir.cleanup)
        sources = [
            "rtl/isp/MonoIsp.sv",
            "rtl/lutcam/CameraPipeline.sv",
            "tests/lutcam/CameraProcessingStub.sv",
            "tests/lutcam/camera_control_tb.sv",
        ]
        if cls.use_dsp_counters or cls.native_isp:
            sources.append(str(cls.dsp_model))
        command = [
            "verilator", "--binary", "--timing", "-Wno-fatal",
            *(["-DSYNTHESIS"] if cls.native_isp else []),
            "--top-module", "camera_control_tb", "--Mdir", cls.build_dir.name,
            f"-GPAIRED_MODE={int(cls.paired_mode)}",
            f"-GUSE_DSP_COUNTERS={int(cls.use_dsp_counters)}",
            *(str(ROOT / source) for source in sources),
        ]
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120)
        if result.returncode:
            raise RuntimeError(f"Verilator compilation failed:\n{result.stdout}\n{result.stderr}")
        cls.binary = Path(cls.build_dir.name) / "Vcamera_control_tb"

    def run_case(self, case):
        result = subprocess.run(
            [str(self.binary), f"+CASE={case}"], cwd=self.build_dir.name,
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(f"PASS camera control case {case}", result.stdout)


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class CameraControlTests(_CameraControlRunner, unittest.TestCase):
    def test_sof_start_frame_stop_drain_ownership_and_active_isp_write_rejection(self):
        self.run_case(0)

    def test_receiver_fault_stays_latched_until_disarmed_and_capture_recovers(self):
        self.run_case(1)

    def test_early_eof_marker_latches_fault(self):
        self.run_case(2)

    def test_missing_eof_marker_latches_fault(self):
        self.run_case(3)

    def test_lwc_fault_cause_survives_child_reset_until_disarmed_clear(self):
        self.run_case(4)

    def test_solver_drop_cause_survives_child_reset_until_disarmed_clear(self):
        self.run_case(5)

    def test_candidate_protocol_diagnostic_survives_two_complete_frames_until_clear(self):
        self.run_case(6)

    def test_noncanonical_defect_table_ready_loss_faults_unbuffered_capture(self):
        self.run_case(7)


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class PairedCameraControlTests(_CameraControlRunner, unittest.TestCase):
    paired_mode = True

    def test_shared_frame_id_and_paired_clear_preserve_isp_configuration(self):
        self.run_case(8)

    def test_candidate_protocol_error_does_not_raise_paired_local_fault(self):
        self.run_case(6)


if __name__ == "__main__":
    unittest.main()
