"""Marker ownership and cancellation boundaries using the real MonoIsp.

The native profile defines SYNTHESIS so the ISP arithmetic DSP branches are
actually exercised; USE_DSP_COUNTERS alone only changes camera accounting.
"""
from pathlib import Path
import os
import shutil
import unittest

from test_camera_control import _CameraControlRunner


def native_model():
    required = os.environ.get("LUTCAM_REQUIRE_EFINIX_NATIVE") == "1"
    if not shutil.which("verilator"):
        if required:
            raise RuntimeError("Required Verilator executable is missing")
        raise unittest.SkipTest("Verilator is required for native camera RTL tests")
    efinity = os.environ.get("LITEX_ENV_EFINITY")
    if not efinity:
        if required:
            raise RuntimeError("LUTCAM_REQUIRE_EFINIX_NATIVE=1 requires LITEX_ENV_EFINITY")
        raise unittest.SkipTest("Set LITEX_ENV_EFINITY to test native camera DSP arithmetic")
    model = Path(efinity) / "sim_models/verilog/efx_dsp48.v"
    if not model.is_file():
        raise RuntimeError(f"Configured Efinity DSP48 model is missing: {model}")
    return model.resolve()


class _MarkerCases:
    def test_malformed_marker_fault_edge_and_restart(self):
        self.run_case(9)

    def test_single_cycle_reset_before_during_and_after_final_pair(self):
        self.run_case(10)

    def test_sensor_bubbles_and_blocked_wishbone_writes(self):
        self.run_case(11)

    def test_isp_write_read_and_cancellation_at_sof(self):
        self.run_case(12)


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class CameraMarkerBoundaryTests(_MarkerCases, _CameraControlRunner, unittest.TestCase):
    pass


class NativeCameraMarkerBoundaryTests(_MarkerCases, _CameraControlRunner, unittest.TestCase):
    use_dsp_counters = True
    native_isp = True

    @classmethod
    def setUpClass(cls):
        cls.dsp_model = native_model()
        super().setUpClass()


class NativePairedCameraMarkerBoundaryTests(NativeCameraMarkerBoundaryTests):
    paired_mode = True


if __name__ == "__main__":
    unittest.main()
