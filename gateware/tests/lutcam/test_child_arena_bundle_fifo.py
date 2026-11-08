"""Epoch-order, serialization, and admission-reserve tests for the arena FIFO."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SOURCES = (
    "rtl/lrcca/ChildArenaBundleFifo.sv",
    "tests/lutcam/child_arena_bundle_fifo_tb.sv",
)




class _BundleFifoRunner:
    synthesis = False
    depth = 128
    id_bits = 7
    feature_width = 34
    admission_reserve = 22

    @classmethod
    def setUpClass(cls):
        cls.build_dir = tempfile.TemporaryDirectory(prefix="lutcam-bundle-fifo-")
        cls.addClassCleanup(cls.build_dir.cleanup)
        result = subprocess.run(
            ["verilator", "--binary", "--timing", "--assert", "-Wno-fatal", "-j", "2",
             "--top-module", "child_arena_bundle_fifo_tb", "--Mdir", cls.build_dir.name,
             f"-GDEPTH={cls.depth}", f"-GID_BITS={cls.id_bits}",
             f"-GFEATURE_WIDTH={cls.feature_width}",
             f"-GADMISSION_RESERVE={cls.admission_reserve}",
             *(["-DSYNTHESIS"] if cls.synthesis else []), *SOURCES],
            cwd=ROOT, capture_output=True, text=True, timeout=120,
        )
        if result.returncode:
            raise RuntimeError(f"Verilator compilation failed:\n{result.stdout}\n{result.stderr}")
        cls.binary = Path(cls.build_dir.name) / "Vchild_arena_bundle_fifo_tb"

    def run_case(self, case, seed=1729275197):
        result = subprocess.run(
            [str(self.binary), f"+CASE={case}", f"+SEED={seed}"],
            cwd=self.build_dir.name, capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(
            f"PASS ChildArenaBundleFifo depth={self.depth} case={case} "
            f"id_bits={self.id_bits} feature_width={self.feature_width}", result.stdout,
        )
        print(next(line for line in result.stdout.splitlines() if line.startswith("PASS ")))

    def test_prior_merge_and_read_precede_later_child_and_all_field_combinations(self):
        self.run_case(0)

    def test_reserved_late_arrivals_full_replacement_and_wraparound(self):
        self.run_case(1)

    def test_reset_aborts_every_serializer_phase_and_full_fifo(self):
        self.run_case(2)

    def test_random_bursts_match_flat_command_scoreboard(self):
        for seed in (1729275197, 1, 2147483647):
            with self.subTest(seed=seed):
                self.run_case(3, seed)


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class ChildArenaBundleFifoTests(_BundleFifoRunner, unittest.TestCase):
    """Original 66-bit bundle shape with the unregistered centroid reserve."""


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class SmallChildArenaBundleFifoTests(_BundleFifoRunner, unittest.TestCase):
    """Force frequent pointer wraparound with the same 22-epoch reserve."""

    depth = 32


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class OddDepthChildArenaBundleFifoTests(_BundleFifoRunner, unittest.TestCase):
    """Exercise explicit last-address wrap at a non-power-of-two depth."""

    depth = 31


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class ProductionChildArenaBundleFifoTests(_BundleFifoRunner, unittest.TestCase):
    """Exact production dimensions: 512 epochs of 78-bit bundles."""

    depth = 512
    id_bits = 9
    feature_width = 38
    admission_reserve = 23


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class LaneBoundaryChildArenaBundleFifoTests(_BundleFifoRunner, unittest.TestCase):
    """An 80-bit bundle fills four 20-bit storage lanes without padding."""

    depth = 512
    id_bits = 9
    feature_width = 40
    admission_reserve = 23


@unittest.skipUnless(shutil.which("iverilog") and shutil.which("vvp"),
                     "Icarus Verilog is required for four-state RTL tests")
class FourStateChildArenaBundleFifoTests(unittest.TestCase):
    def test_uninitialized_storage_never_escapes_and_stalled_output_is_stable(self):
        with tempfile.TemporaryDirectory(prefix="lutcam-bundle-fifo-four-state-") as directory:
            binary = str(Path(directory) / "child_arena_bundle_fifo_tb")
            result = subprocess.run(
                ["iverilog", "-g2012", "-s", "child_arena_bundle_fifo_tb", "-o", binary,
                 "-Pchild_arena_bundle_fifo_tb.DEPTH=32", *SOURCES],
                cwd=ROOT, capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            for case in range(4):
                with self.subTest(case=case):
                    result = subprocess.run(
                        ["vvp", binary, f"+CASE={case}", "+SEED=1729275197"],
                        cwd=directory, capture_output=True, text=True, timeout=30,
                    )
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertIn(f"PASS ChildArenaBundleFifo depth=32 case={case}", result.stdout)


# The synthesis profile also compiles without any vendor primitive library.
# Portable Icarus tests retain complementary four-state RAM coverage.
@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class SynthesisProductionChildArenaBundleFifoTests(ProductionChildArenaBundleFifoTests):
    synthesis = True


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class SynthesisLaneBoundaryChildArenaBundleFifoTests(LaneBoundaryChildArenaBundleFifoTests):
    synthesis = True


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class SynthesisOddDepthChildArenaBundleFifoTests(OddDepthChildArenaBundleFifoTests):
    synthesis = True


if __name__ == "__main__":
    unittest.main()
