"""Required native ISP validation must fail rather than silently skip."""

import os
from pathlib import Path
import unittest
from unittest.mock import patch

import isp_sim


class IspNativeEnvironmentTests(unittest.TestCase):
    def test_required_verilator_cannot_skip(self):
        with patch.dict(os.environ, {"LUTCAM_REQUIRE_EFINIX_NATIVE": "1"}, clear=True):
            with patch.object(isp_sim.shutil, "which", return_value=None):
                with self.assertRaisesRegex(RuntimeError, "Verilator"):
                    isp_sim.profiles()

    def test_required_vendor_configuration_cannot_skip(self):
        with patch.dict(os.environ, {"LUTCAM_REQUIRE_EFINIX_NATIVE": "1"}, clear=True):
            with patch.object(isp_sim.shutil, "which", return_value="verilator"):
                with self.assertRaisesRegex(RuntimeError, "LITEX_ENV_EFINITY"):
                    isp_sim.profiles()

    def test_configured_missing_vendor_cannot_skip(self):
        with patch.dict(os.environ, {"LITEX_ENV_EFINITY": "/missing-efinity"}, clear=True):
            with patch.object(isp_sim.shutil, "which", return_value="verilator"):
                with patch.object(Path, "is_file", return_value=False):
                    with self.assertRaisesRegex(RuntimeError, "DSP48 model is missing"):
                        isp_sim.profiles()

    def test_optional_unconfigured_run_is_portable(self):
        with patch.dict(os.environ, {}, clear=True):
            with patch.object(isp_sim.shutil, "which", return_value="verilator"):
                self.assertEqual(isp_sim.profiles(), [("portable", None)])


if __name__ == "__main__":
    unittest.main()
