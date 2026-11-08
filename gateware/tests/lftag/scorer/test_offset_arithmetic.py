"""Exhaustive signed19 offset arithmetic in the actual current scorer RTL."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
DSP_MODEL = Path(os.environ.get("LITEX_ENV_EFINITY", "/nonexistent")) / "sim_models/verilog/efx_dsp48.v"


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class OffsetArithmeticTests(unittest.TestCase):
    def run_mode(self, native):
        with tempfile.TemporaryDirectory(prefix="lutcam-offset-arithmetic-") as tmp:
            command = ["verilator", "--binary", "--timing", "--assert", "-Wno-fatal",
                       "-j", "2", "--top-module", "offset_arithmetic_tb", "--Mdir", tmp,
                       str(ROOT / "rtl/lftag/LfTagHomogeneousGeometryScorer.sv"),
                       str(Path(__file__).with_name("offset_arithmetic_tb.sv"))]
            if native:
                command += ["-DSYNTHESIS", str(DSP_MODEL)]
            built = subprocess.run(command, capture_output=True, text=True, timeout=120)
            self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
            result = subprocess.run([str(Path(tmp) / "Voffset_arithmetic_tb")],
                                    capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS offset arithmetic checks=1572864 domains=3 values_per_domain=524288", result.stdout)
            print(("native" if native else "portable") + ": " + result.stdout.strip(), flush=True)

    def test_portable_full_signed_domain_and_clock_reset_independence(self):
        self.run_mode(False)

    @unittest.skipUnless(DSP_MODEL.is_file(), "LITEX_ENV_EFINITY vendor model is required")
    def test_native_full_signed_domain_and_clock_reset_independence(self):
        self.run_mode(True)


if __name__ == "__main__":
    unittest.main()
