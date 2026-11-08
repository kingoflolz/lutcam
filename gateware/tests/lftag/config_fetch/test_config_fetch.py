"""Narrow configuration-fetch interface parity, stalls, and camera banking.

Run with ``python -m pytest -q tests/lftag/config_fetch/test_config_fetch.py``.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]

@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class ConfigFetchTests(unittest.TestCase):
    native_dsp = False

    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix="lutcam-config-fetch-")
        cls.addClassCleanup(cls.build.cleanup)
        sources = [line for line in (ROOT / "rtl/lftag/sources.f").read_text().splitlines()
                   if line.strip() and not line.lstrip().startswith("#")]
        sources.append("tests/lftag/config_fetch/config_fetch_tb.sv")
        native_options = []
        if cls.native_dsp:
            from litex.build.efinix.toolchain import find_efinity_path
            try:
                vendor = Path(find_efinity_path()) / "sim_models/verilog/efx_dsp48.v"
            except OSError as error:
                raise unittest.SkipTest("Efinity DSP simulation model is required") from error
            if not vendor.is_file():
                raise unittest.SkipTest("Efinity DSP simulation model is required")
            native_options = ["-DSYNTHESIS", str(vendor)]
        result = subprocess.run(
            ["verilator", "--binary", "--timing", "--assert", "-Wno-fatal",
             "-j", "2", "--top-module", "config_fetch_tb", "--Mdir", cls.build.name,
             f"-I{ROOT / 'rtl/lftag'}", f"-I{ROOT / 'tests/lftag/config_fetch'}",
             *native_options, *(str(ROOT / source) for source in sources)],
            cwd=ROOT, capture_output=True, text=True, timeout=240)
        if result.returncode:
            raise RuntimeError(f"Verilator compilation failed:\n{result.stdout}\n{result.stderr}")
        cls.binary = Path(cls.build.name) / "Vconfig_fetch_tb"

    def run_case(self, case):
        result = subprocess.run([str(self.binary), f"+CASE={case}"], cwd=ROOT,
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(f"PASS config fetch case {case}", result.stdout)
        print(result.stdout, end="")

    def test_both_camera_banks_and_stalls_match_direct_configuration(self):
        self.run_case(0)

    def test_threshold_equalities_and_extremes_match_direct_configuration(self):
        self.run_case(1)

    def test_resets_during_controller_and_homography_fetch_discard_stale_replies(self):
        self.run_case(2)

    def test_tied_candidate_accepts_with_reserved_words_zero(self):
        self.run_case(3)

    def test_normalized_exact_boundaries_and_full_width(self):
        self.run_case(4)

    def test_random_full_width_rationals_match_exact_q16_acceptance(self):
        self.run_case(5)

@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class NativeConfigFetchTests(ConfigFetchTests):
    """Exercise fabric configuration with native arithmetic through full core transactions."""
    native_dsp = True


if __name__ == "__main__":
    unittest.main()
