"""Generated-RTL equivalence for the paired-capture residual timing guard."""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "fpga" / "litex"))

from migen import Cat, Constant, Module, Mux, Signal
from migen.fhdl import verilog
from migen.sim import run_simulation

from lutcam.pair import _ResidualSkewGuard


class GuardEquivalence(Module):
    def __init__(self):
        self.latency0 = Signal(21)
        self.latency1 = Signal(21)
        self.maximum = Signal(22)
        self.old_good, self.new_good, self.mismatch = Signal(), Signal(), Signal()
        residual = Signal((22, True))
        absolute = Signal(22)
        self.submodules.guard = _ResidualSkewGuard(residual, self.maximum)
        self.comb += [
            residual.eq(Cat(self.latency1, Constant(0, 1))
                        - Cat(self.latency0, Constant(0, 1))),
            absolute.eq(Mux(residual < 0, -residual, residual)),
            self.old_good.eq(absolute <= self.maximum),
            self.new_good.eq(self.guard.within),
            self.mismatch.eq(self.old_good != self.new_good),
        ]


class PairResidualGuardTests(unittest.TestCase):
    def test_signed_extremes_zero_and_large_unsigned_limits(self):
        dut = GuardEquivalence()
        latencies = (0, 1, (1 << 20) - 1, 1 << 20, (1 << 21) - 2, (1 << 21) - 1)
        limits = (0, 1, (1 << 21) - 2, (1 << 21) - 1, 1 << 21, (1 << 22) - 1)

        def stimulus():
            for l0 in latencies:
                for l1 in latencies:
                    for limit in limits:
                        yield dut.latency0.eq(l0)
                        yield dut.latency1.eq(l1)
                        yield dut.maximum.eq(limit)
                        yield
                        self.assertEqual((yield dut.new_good), int(abs(l1 - l0) <= limit),
                                         (l0, l1, limit))
                        self.assertEqual((yield dut.mismatch), 0, (l0, l1, limit))

        run_simulation(dut, stimulus())

    @unittest.skipUnless(shutil.which("yosys"), "Yosys is required for the exhaustive RTL proof")
    def test_generated_rtl_equivalent_for_all_64_input_bits(self):
        dut = GuardEquivalence()
        generated = verilog.convert(dut, name="guard_equivalence", ios={
            dut.latency0, dut.latency1, dut.maximum, dut.old_good, dut.new_good, dut.mismatch})
        with tempfile.TemporaryDirectory(prefix="lutcam-pair-guard-proof-") as temporary:
            directory = Path(temporary)
            generated.write(str(directory / "guard.v"))
            script = directory / "proof.ys"
            script.write_text(
                "read_verilog guard.v\n"
                "prep -top guard_equivalence\n"
                "flatten\nopt_clean\n"
                f"sat -verify -prove {generated.ns.get_name(dut.mismatch)} 0 -show-inputs -show-outputs\n")
            result = subprocess.run(["yosys", "-s", str(script)], cwd=directory,
                                    capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("SAT proof finished - no model found: SUCCESS!", result.stdout)


if __name__ == "__main__":
    unittest.main()
