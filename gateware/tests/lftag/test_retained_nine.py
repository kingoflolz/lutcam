"""Retained-nine decoding through the production FIFO, solver, and real ROMs.

Run with ``python -m pytest -q tests/lftag/test_retained_nine.py``.
No FPGA hardware or vendor libraries are required.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class RetainedNineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix="lutcam-retained-nine-")
        cls.addClassCleanup(cls.build.cleanup)
        sources = (ROOT / "rtl/lftag/sources.f").read_text().splitlines()
        sources.append("tests/lftag/retained_nine_tb.sv")
        result = subprocess.run(
            ["verilator", "--binary", "--timing", "--assert", "-Wno-fatal",
             "-j", "2", "--top-module", "retained_nine_tb",
             "--Mdir", cls.build.name, f"-I{ROOT / 'rtl/lftag'}",
             *(str(ROOT / source) for source in sources if source.strip())],
            cwd=ROOT, capture_output=True, text=True, timeout=240,
        )
        if result.returncode:
            raise RuntimeError(f"Verilator compilation failed:\n{result.stdout}\n{result.stderr}")
        cls.binary = Path(cls.build.name) / "Vretained_nine_tb"

    def run_case(self, case):
        result = subprocess.run(
            [str(self.binary), f"+CASE={case}"], cwd=ROOT,
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(f"PASS retained nine case {case}", result.stdout)

    def test_retained_nine_produces_repeatable_pose_with_reserved_bit_zero(self):
        self.run_case(0)

    def test_invalid_centroid_rejects_then_valid_nine_recovers_without_reset(self):
        self.run_case(1)

    def test_mixed_scale_and_frame_metadata_reject_then_valid_nine_recovers(self):
        self.run_case(2)


if __name__ == "__main__":
    unittest.main()
