"""The ROM mapping hint preserves LiteX's identification bytes and CSR port."""

import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from migen import ClockDomain, Module
from migen.sim import run_simulation
from litex.gen.fhdl import verilog
from litex.soc.cores.identifier import Identifier
from litex.soc.integration.soc import build_time
from litex.soc.interconnect import csr_bus

from lutcam.identifier import IDENTIFIER_ROM_LOGIC_BITS, use_logic_identifier


EXPECTED = b"LUTCAM dual RAW10 ISP camera 2026-09-26 22:44:03\0"


class IdentifierFixture(Module):
    def __init__(self, logic=False):
        with patch.dict(os.environ, SOURCE_DATE_EPOCH="1790462643"):
            self.submodules.identifier = Identifier("LUTCAM dual RAW10 ISP camera " + build_time())
        if logic:
            use_logic_identifier(self.identifier)
        self.submodules.sram = csr_bus.SRAM(self.identifier.mem, address=4,
            read_only=True, bus=csr_bus.Interface(data_width=32))
        self.bus = self.sram.bus


class IdentifierTests(unittest.TestCase):
    def test_all_bytes_through_read_only_csr_bank(self):
        # There is no length prefix in the pinned LiteX identifier protocol:
        # the existing 48-byte build string is followed by one NUL byte.
        for logic in (False, True):
            with self.subTest(logic=logic):
                dut = IdentifierFixture(logic)
                self.assertEqual(dut.identifier.get_memories(), [(True, dut.identifier.mem)])
                self.assertEqual(bytes(dut.identifier.mem.init), EXPECTED)

                def bench():
                    for index, value in enumerate(EXPECTED):
                        yield dut.bus.adr.eq(0x2000 // 4 + index)
                        yield dut.bus.re.eq(1)
                        yield
                        yield
                        self.assertEqual((yield dut.bus.dat_r), value)
                    yield dut.bus.adr.eq(0x2000 // 4)
                    yield dut.bus.dat_w.eq(0xFFFFFFFF)
                    yield dut.bus.we.eq(1)
                    yield
                    yield
                    yield dut.bus.we.eq(0)
                    yield
                    self.assertEqual((yield dut.bus.dat_r), EXPECTED[0])
                    yield dut.bus.adr.eq(0)
                    yield
                    yield
                    self.assertEqual((yield dut.bus.dat_r), 0)

                run_simulation(dut, bench())

    def test_emitted_attribute_and_guard_do_not_change_memory_contents(self):
        dut = IdentifierFixture(logic=True)
        dut.clock_domains.cd_sys = ClockDomain("sys")
        generated = verilog.convert(dut,
            ios=set(dut.bus.flatten()) | {dut.cd_sys.clk, dut.cd_sys.rst}, name="identifier_probe")
        self.assertIn('(* syn_romstyle = "logic" *)', generated.main_source)
        self.assertEqual(IDENTIFIER_ROM_LOGIC_BITS, len(EXPECTED) * 8)
        init = next(iter(generated.data_files.values()))
        self.assertEqual(bytes(int(word, 16) for word in init.split()), EXPECTED)
        with self.assertRaisesRegex(ValueError, "392-bit"):
            use_logic_identifier(Identifier("x" * 49))


if __name__ == "__main__":
    unittest.main()
