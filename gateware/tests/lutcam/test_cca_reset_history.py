"""An aborted frame's line RAM must not become the next frame's top row."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class CcaResetHistoryTests(unittest.TestCase):
    def test_aborted_line_memory_matches_fresh_instance_in_both_modes(self):
        with tempfile.TemporaryDirectory(prefix="lutcam-cca-reset-") as directory:
            result = subprocess.run([
                "verilator", "--binary", "--timing", "--assert", "-Wno-fatal", "-j", "2",
                "--top-module", "cca_reset_history_tb", "--Mdir", directory,
                "rtl/lrcca/lrcca_util_pkg.sv", "rtl/lrcca/TwoPPCS0Stage.sv",
                "tests/lutcam/cca_reset_history_tb.sv",
            ], cwd=ROOT, capture_output=True, text=True, timeout=90)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(Path(directory) / "Vcca_reset_history_tb")],
                cwd=ROOT, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS CCA reset history both hole-fill modes", result.stdout)


if __name__ == "__main__":
    unittest.main()
