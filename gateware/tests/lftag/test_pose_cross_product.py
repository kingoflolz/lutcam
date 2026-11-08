"""Reproduce synchronous-RAM pose cross products across consecutive candidates."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class PoseCrossProductTests(unittest.TestCase):
    def test_sequential_pose_rotation_and_translation_match_independent_math(self):
        with tempfile.TemporaryDirectory(prefix="lftag-pose-cross-") as directory:
            result = subprocess.run(
                [
                    "verilator", "--binary", "--timing", "--assert", "-Wno-fatal", "-j", "2",
                    "--top-module", "pose_cross_product_tb", "--Mdir", directory,
                    "rtl/lftag/LfTagPoseReconstruct.sv", "tests/lftag/pose_cross_product_tb.sv",
                ], cwd=ROOT, capture_output=True, text=True, timeout=90,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run(
                [str(Path(directory) / "Vpose_cross_product_tb")],
                cwd=ROOT, capture_output=True, text=True, timeout=20,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS pose cross product", result.stdout)


if __name__ == "__main__":
    unittest.main()
