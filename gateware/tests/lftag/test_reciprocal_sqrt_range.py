"""Compare the pose-scale RTL to an independent square-root oracle."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class ReciprocalSqrtRangeTests(unittest.TestCase):
    def test_all_normalization_bins_exponents_backpressure_and_reset(self):
        with tempfile.TemporaryDirectory(prefix="lftag-rsqrt-range-") as directory:
            result = subprocess.run(
                [
                    "verilator", "--binary", "--timing", "--assert", "-Wno-fatal", "-j", "2",
                    "--top-module", "reciprocal_sqrt_range_tb", "--Mdir", directory,
                    "+incdir+rtl/lftag", "rtl/lftag/LfTagReciprocalSqrt512.sv",
                    "tests/lftag/reciprocal_sqrt_range_tb.sv",
                ], cwd=ROOT, capture_output=True, text=True, timeout=90,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run(
                [str(Path(directory) / "Vreciprocal_sqrt_range_tb")],
                cwd=ROOT, capture_output=True, text=True, timeout=20,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS reciprocal sqrt range", result.stdout)


if __name__ == "__main__":
    unittest.main()
