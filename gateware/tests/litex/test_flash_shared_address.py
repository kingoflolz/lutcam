"""Shared packet/SPI address profile retains public flash and SPI contracts."""

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

from migen import Signal
from migen.sim import run_simulation

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))
from lutcam.flash_packet import PacketFlashController, _PageSPITransaction
from tests.litex import test_flash_packet as packet_tests
from tests.litex.spi_nor_model import SpiNor


class SharedPacketAddressTests(packet_tests.PacketFlashTests):
    """Run the independent pin-level NOR/record suite with sharing enabled."""

    def fixture(self, *, boot_slot=1, poll_timeout_cycles=50000,
                golden_maintenance=False, **model_options):
        dut = PacketFlashController(clk_freq=200, spi_clk_freq=50,
                                    boot_slot=boot_slot,
                                    golden_maintenance=golden_maintenance,
                                    poll_timeout_cycles=poll_timeout_cycles,
                                    share_spi_address=True)
        return dut, SpiNor(dut.pads, **model_options)


class SPIAddressSnapshotTests(unittest.TestCase):
    def test_default_snapshots_address_and_shared_mode_accepts_held_address(self):
        # Standalone callers may change inputs immediately after start. They
        # must keep the original snapshot behavior unless they explicitly opt
        # into the stable-address contract used by the packet controller.
        for half_period in (2, 3, 12):
            for shared in (False, True):
                with self.subTest(half_period=half_period, shared=shared):
                    pads = SimpleNamespace(clk=Signal(), cs_n=Signal(reset=1),
                                           mosi=Signal(), miso=Signal(),
                                           wp=Signal(reset=1), hold=Signal(reset=1))
                    dut = _PageSPITransaction(pads, half_period,
                                              address_is_stable=shared)
                    flash = SpiNor(pads)

                    def bench():
                        yield dut.opcode.eq(3)
                        yield dut.address.eq(0xA1B2C3)
                        yield dut.with_address.eq(1)
                        yield dut.read_length.eq(1)
                        yield dut.start.eq(1)
                        yield
                        yield
                        yield dut.start.eq(0)
                        for cycle in range(2000):
                            if not shared:
                                yield dut.address.eq((cycle * 0x102037) & 0xFFFFFF)
                            if (yield dut.done):
                                break
                            yield
                        else:
                            self.fail("SPI transaction did not complete")
                        for _ in range(3):
                            yield
                        self.assertEqual(flash.transactions, [b"\x03\xa1\xb2\xc3\xff"])

                    run_simulation(dut, [bench(), flash.process()])


if __name__ == "__main__":
    unittest.main()
