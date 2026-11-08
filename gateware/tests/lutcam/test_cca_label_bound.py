"""The production outer CCA must retain every legal row-run label."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SOURCES = [
    "rtl/lrcca/lrcca_util_pkg.sv",
    "rtl/lrcca/TwoPPCS0Stage.sv",
    "rtl/lrcca/TwoPPCS1Stage.sv",
    "rtl/lrcca/TwoPPCS2Stage.sv",
    "rtl/lrcca/TwoPPCS6BStage.sv",
    "rtl/lrcca/TwoPPCS3Stage.sv",
    "rtl/lrcca/TwoPPCS4Stage.sv",
    "rtl/lrcca/S5HighChunkAdd.sv",
    "rtl/lrcca/TwoPPCS5Stage.sv",
    "rtl/lrcca/TwoPPC.sv",
    "tests/lutcam/cca_label_bound_tb.sv",
]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class CcaLabelBoundTests(unittest.TestCase):
    def run_capacity(self, depth):
        with tempfile.TemporaryDirectory(prefix=f"lutcam-cca-depth-{depth}-") as directory:
            result = subprocess.run(
                ["verilator", "--binary", "--timing", "--assert", "--timescale", "1ns/1ps",
                 "-j", "2", "--top-module", "cca_label_bound_tb", "--Mdir", directory,
                 f"-GTABLE_DEPTH={depth}", *SOURCES],
                cwd=ROOT, capture_output=True, text=True, timeout=180,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            # Preserve compiler diagnostics rather than concealing warnings.
            if result.stderr:
                print(result.stderr, end="")
            args = [str(Path(directory) / "Vcca_label_bound_tb")]
            if depth == 511:
                args.append("+ONLY_WORST")
            result = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=30)
            print(result.stdout, end="")
            if result.stderr:
                print(result.stderr, end="")
            return result

    def test_848_entries_cover_both_hole_fill_modes(self):
        result = self.run_capacity(848)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("PASS CCA label bound depth=848", result.stdout)
        for mode in (0, 1):
            self.assertRegex(result.stdout,
                             rf"CCA_BOUND mode={mode} depth=848 rows=336 "
                             r"peak_runs=847 peak_allocated=846 "
                             r"peak_c_reg=846 peak_p_issue=847 peak_next_read=847")

    def test_old_511_entry_limit_fails_on_legal_vertical_stripes(self):
        result = self.run_capacity(511)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("CCA_LABEL_CAPACITY depth=511", result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
