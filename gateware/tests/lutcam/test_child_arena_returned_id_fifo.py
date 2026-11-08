"""Returned IDs preserve reset prefixes, wrap, and exact first-word timing."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
PORTABLE_AVAILABLE = shutil.which("iverilog") is not None and shutil.which("vvp") is not None


class _ReturnedFifoRunner:
    width = 9
    preload = 511
    synthesis = False

    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="lutcam-returned-id-fifo-")
        cls.addClassCleanup(cls.directory.cleanup)
        build = Path(cls.directory.name)
        sources = [str(ROOT / "rtl/lrcca/ChildArenaReturnedIdFifo.sv"),
                   str(ROOT / "tests/lutcam/child_arena_returned_id_fifo_tb.sv")]
        top = "child_arena_returned_id_fifo_tb"
        if cls.synthesis:
            # The synthesis profile needs no vendor primitive. Portable
            # Icarus supplies four-state checks, including uninitialized RAM.
            command = ["verilator", "--binary", "--timing", "--assert", "-Wno-fatal",
                       "-j", "2", "-DSYNTHESIS", "--top-module", top, "--Mdir", str(build),
                       f"-GWIDTH={cls.width}", f"-GPRELOAD_COUNT={cls.preload}",
                       *sources]
            cls.run_command = [str(build / f"V{top}")]
        else:
            command = ["iverilog", "-g2012", "-s", top, "-o", str(build / "fifo.vvp"),
                       f"-P{top}.WIDTH={cls.width}", f"-P{top}.PRELOAD_COUNT={cls.preload}",
                       *sources]
            cls.run_command = ["vvp", str(build / "fifo.vvp")]
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)

    def test_prefix_wrap_refill_reset_and_randomized_scoreboard(self):
        for seed in (1729275197, 1, 2147483647):
            with self.subTest(seed=seed):
                result = subprocess.run([*self.run_command, f"+SEED={seed}"],
                    cwd=ROOT, capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn(f"PASS ReturnedId width={self.width} preload={self.preload}", result.stdout)
                print(next(line for line in result.stdout.splitlines() if line.startswith("PASS ")))


@unittest.skipUnless(PORTABLE_AVAILABLE, "Icarus and vvp are required")
class ProductionReturnedIdFifoTests(_ReturnedFifoRunner, unittest.TestCase):
    """Production ChildArena ENTRY_COUNT=511 uses9-bit returned IDs."""


@unittest.skipUnless(PORTABLE_AVAILABLE, "Icarus and vvp are required")
class SmallReturnedIdFifoTests(_ReturnedFifoRunner, unittest.TestCase):
    width = 7
    preload = 108


@unittest.skipUnless(PORTABLE_AVAILABLE, "Icarus and vvp are required")
class EmptyPrefixReturnedIdFifoTests(_ReturnedFifoRunner, unittest.TestCase):
    width = 7
    preload = 0


@unittest.skipUnless(PORTABLE_AVAILABLE, "Icarus and vvp are required")
class OnePrefixReturnedIdFifoTests(_ReturnedFifoRunner, unittest.TestCase):
    width = 7
    preload = 1


@unittest.skipUnless(PORTABLE_AVAILABLE, "Icarus and vvp are required")
class MaximumPrefixReturnedIdFifoTests(_ReturnedFifoRunner, unittest.TestCase):
    width = 11
    preload = 1024


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class SynthesisProductionReturnedIdFifoTests(_ReturnedFifoRunner, unittest.TestCase):
    synthesis = True


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class SynthesisSmallReturnedIdFifoTests(_ReturnedFifoRunner, unittest.TestCase):
    synthesis = True
    width = 7
    preload = 108


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class SynthesisMaximumPrefixReturnedIdFifoTests(_ReturnedFifoRunner, unittest.TestCase):
    synthesis = True
    width = 11
    preload = 1024


if __name__ == "__main__":
    unittest.main()
