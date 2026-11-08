"""Native camera accounting matches the original register priority and wrap."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import test_camera_control as control


ROOT = Path(__file__).resolve().parents[2]
DSP_MODEL = Path(os.environ.get("LITEX_ENV_EFINITY", "/nonexistent")) / (
    "sim_models/verilog/efx_dsp48.v"
)
NATIVE_AVAILABLE = shutil.which("verilator") is not None and DSP_MODEL.is_file()
NATIVE_REASON = "Verilator and LITEX_ENV_EFINITY vendor DSP model are required"


@unittest.skipUnless(NATIVE_AVAILABLE, NATIVE_REASON)
class CameraDspCounterTests(unittest.TestCase):
    def test_original_priority_hold_reset_and_wrapping(self):
        with tempfile.TemporaryDirectory(prefix="lutcam-camera-counters-") as build_dir:
            result = subprocess.run([
                "verilator", "--binary", "--timing", "-Wno-fatal", "-j", "4",
                "--top-module", "camera_event_counter_tb", "--Mdir", build_dir,
                str(ROOT / "tests/lutcam/camera_event_counter_tb.sv"),
                str(ROOT / "rtl/lutcam/CameraPipeline.sv"), str(DSP_MODEL),
            ], cwd=ROOT, capture_output=True, text=True, timeout=120)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([
                str(Path(build_dir) / "Vcamera_event_counter_tb"),
            ], cwd=ROOT, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS camera counters: 1000000 cycles", result.stdout)
            print(result.stdout, end="")


@unittest.skipUnless(NATIVE_AVAILABLE, NATIVE_REASON)
class CameraDspControlTests(control.CameraControlTests):
    use_dsp_counters = True
    dsp_model = DSP_MODEL


@unittest.skipUnless(NATIVE_AVAILABLE, NATIVE_REASON)
class PairedCameraDspControlTests(control.PairedCameraControlTests):
    use_dsp_counters = True
    dsp_model = DSP_MODEL


if __name__ == "__main__":
    unittest.main()
