"""Firmware loading through the real packet controller and pin-level SPI NOR."""

from pathlib import Path
import struct
import sys
import unittest
import zlib

from migen import Module
from migen.sim import run_simulation

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga/litex"))
from lutcam.boot_reader import mac_from_uid
from lutcam.flash_packet import PacketFlashController
from lutcam.supervisor import FirmwareLoader, FIRMWARE_OFFSET, FIRMWARE_MAGIC, RAM_BYTES, STACK_BYTES
from tests.litex.spi_nor_model import SpiNor


class FirmwareLoaderTests(unittest.TestCase):
    def exercise(self, length, slot):
        dut = Module()
        dut.submodules.loader = loader = FirmwareLoader()
        dut.submodules.flash = flash = PacketFlashController(
            clk_freq=200, spi_clk_freq=50, boot_slot=slot, boot_client=loader.spi)
        dut.comb += flash.inhibit.eq(loader.busy)
        nor = SpiNor(flash.pads)
        payload = bytes((i * 43 + (i >> 8) + 0x81) & 255 for i in range(length))
        checksum = zlib.crc32(payload)
        header = struct.pack("<IIII", FIRMWARE_MAGIC, length, checksum, RAM_BYTES)
        base = slot * 0x200000 + FIRMWARE_OFFSET
        nor.memory.update(enumerate(header + payload, base))
        loaded = bytearray()

        def bench():
            yield loader.slot.eq(slot)
            yield loader.identity_ready.eq(1)
            for _ in range(300_000):
                if (yield loader.write):
                    self.assertEqual((yield loader.address) * 4, len(loaded))
                    loaded.extend((yield loader.data).to_bytes(4, "little"))
                if not (yield loader.busy):
                    break
                yield
            else:
                self.fail("firmware loader did not finish")
            # The physical CS register releases on the following edge.
            yield
            yield
            yield
            self.assertEqual((yield loader.error), 0)
            self.assertEqual((yield loader.done), 1)
            self.assertEqual((yield loader.length), length)
            self.assertEqual((yield loader.expected_crc), checksum)
            self.assertEqual(loaded, payload)
            self.assertEqual(nor.wren_count, 0, "boot must never write flash")
            self.assertEqual((yield loader.mac_address), mac_from_uid(nor.uid))
            self.assertEqual((yield loader.mac_ready), 1)
            self.assertEqual(nor.transactions[1][:5], bytes([0x4b, 0, 0, 0, 0]))
            self.assertEqual(len(nor.transactions[1]), 21)
            # Opcode 03 reads continuously across page boundaries.
            self.assertEqual(len(nor.transactions), 2)
            transaction = nor.transactions[0]
            self.assertEqual(transaction[0], 3)
            self.assertEqual(int.from_bytes(transaction[1:4], "big"), base)
            self.assertEqual(len(transaction) - 4, len(header) + length)
            if length == 4:
                from tests.litex.test_flash_packet import PacketFlashTests, JEDEC, RESULT
                host = PacketFlashTests()
                yield from host.start(flash, 1, JEDEC)
                yield from host.complete(flash, 1)
                self.assertEqual((yield from host.read(flash, RESULT)), 0xc86018)

        run_simulation(dut, [bench(), nor.process()])

    def test_short_image(self): self.exercise(4, 1)
    def test_first_page_exact(self): self.exercise(240, 2)
    def test_first_page_plus_one_word(self): self.exercise(244, 1)
    def test_second_page_exact(self): self.exercise(496, 2)
    def test_second_page_plus_one_word(self): self.exercise(500, 1)
    def test_maximum_image(self): self.exercise(RAM_BYTES - STACK_BYTES, 2)

    def test_identity_only_recovery_and_blank_uid_rejection(self):
        for uid in (bytes(range(16)), bytes(reversed(range(16))), bytes(16), b"\xff" * 16):
            with self.subTest(uid=uid.hex()):
                dut = Module()
                dut.submodules.loader = loader = FirmwareLoader(with_firmware=False)
                dut.submodules.flash = flash = PacketFlashController(
                    clk_freq=200, spi_clk_freq=50, boot_client=loader.spi)
                dut.comb += flash.inhibit.eq(loader.busy)
                nor = SpiNor(flash.pads)
                nor.uid = uid

                def bench():
                    yield loader.identity_ready.eq(1)
                    for _ in range(2000):
                        if not (yield loader.busy):
                            break
                        self.assertEqual((yield loader.write), 0)
                        yield
                    else:
                        self.fail("UID read timed out")
                    valid = uid not in (bytes(16), b"\xff" * 16)
                    self.assertEqual((yield loader.done), valid)
                    self.assertEqual((yield loader.mac_ready), valid)
                    self.assertEqual((yield loader.error), 0 if valid else 8)
                    if valid:
                        self.assertEqual((yield loader.mac_address), mac_from_uid(uid))
                        self.assertEqual((yield loader.mac_address) >> 40 & 3, 2)

                run_simulation(dut, [bench(), nor.process()])


if __name__ == "__main__": unittest.main()
