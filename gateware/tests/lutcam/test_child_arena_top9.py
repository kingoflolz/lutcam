"""Production child selection must retain the exact top nine payloads."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class ChildArenaTopNineTests(unittest.TestCase):
    def test_literal_top_nine_and_ties_with_backpressure(self):
        for entries, width, bundled in ((108, 34, 0), (511, 38, 1)):
            with self.subTest(entries=entries, width=width, bundled=bundled):
                self.check_literal_top_nine(entries, width, bundled)

    def check_literal_top_nine(self, entries, width, bundled):
        with tempfile.TemporaryDirectory(prefix="lutcam-child-top9-") as directory:
            result = subprocess.run([
                "verilator", "--binary", "--timing", "--assert", "-Wno-fatal", "-j", "2",
                "--top-module", "child_arena_top9_tb", "--Mdir", directory,
                f"-GENTRY_COUNT={entries}", f"-GFEATURE_WIDTH={width}",
                f"-GBUNDLED_COMMANDS={bundled}",
                "rtl/lrcca/lrcca_util_pkg.sv", "rtl/lrcca/ChildArenaSyncFifo.sv",
                "rtl/lrcca/ChildArenaBundleFifo.sv",
                "rtl/lrcca/ChildArenaReturnedIdFifo.sv", "rtl/lrcca/ChildArenaFvMem.sv",
                "rtl/lrcca/ChildArena.sv", "tests/lutcam/child_arena_top9_tb.sv",
            ], cwd=ROOT, capture_output=True, text=True, timeout=90)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(Path(directory) / "Vchild_arena_top9_tb")],
                cwd=ROOT, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS ChildArena literal top-nine selection", result.stdout)


if __name__ == "__main__":
    unittest.main()
