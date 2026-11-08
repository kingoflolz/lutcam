"""Exact cycle-shift parity of the centroid timing boundary."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SOURCES = ["rtl/lrcca/lrcca_util_pkg.sv"] + [
    str(path.relative_to(ROOT)) for path in sorted((ROOT / "rtl/lrcca").glob("Centroid*.sv"))
] + ["tests/lutcam/centroid_registered_input_tb.sv"]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class CentroidRegisteredInputTests(unittest.TestCase):
    def test_values_bubbles_and_inflight_reset_match_one_clock_later(self):
        with tempfile.TemporaryDirectory(prefix="lutcam-centroid-input-") as directory:
            result = subprocess.run(
                ["verilator", "--binary", "--timing", "--assert", "-j", "2",
                 "-Wno-TIMESCALEMOD", "--top-module", "centroid_registered_input_tb",
                 "--Mdir", directory, *SOURCES],
                cwd=ROOT, capture_output=True, text=True, timeout=120,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run(
                [str(Path(directory) / "Vcentroid_registered_input_tb")],
                cwd=ROOT, capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS centroid input register: four arithmetic configurations", result.stdout)
            print(result.stdout.splitlines()[0])


if __name__ == "__main__":
    unittest.main()
