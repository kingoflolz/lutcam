"""Read the board identity and resident firmware using the shared SPI shifter."""

from migen import (Cat, Case, Constant, FSM, If, Module, Mux, NextState,
                   NextValue, Record, Signal)

FIRMWARE_OFFSET = 0x180000
FIRMWARE_MAGIC = 0x3256524c
RAM_BYTES = 4096
STACK_BYTES = 128
MAILBOX_BASE = RAM_BYTES - STACK_BYTES - 56


def mac_from_uid(uid):
    """CRC-64/XZ of all 16 UID bytes, low 48 bits, local unicast address.

    This is a stable address derivation, not an authentication hash.
    The serial hardware shares this CRC register with firmware CRC-32.
    """
    if len(uid) != 16 or uid in (bytes(16), b"\xff" * 16):
        raise ValueError("flash UID must contain 16 nonblank bytes")
    crc = (1 << 64) - 1
    for byte in uid:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xc96c5795d7870f42 if crc & 1 else 0)
    return ((~crc & ((1 << 48) - 1)) & ~(3 << 40)) | (2 << 40)


class FirmwareLoader(Module):
    """Exclusive, read-only boot client of the existing flash byte engine.

    Use a continuous 03 read for the selected slot's header and payload,
    followed by GD25LQ128E UID (4B, address 000000, one dummy byte).
    SRAM is released only after the header and CRC-32 both pass. No packet
    staging, command IDs, system bus master or flash polling is needed.
    """
    def __init__(self, *, with_firmware=True, byte_timeout=128):
        self.spi = spi = Record([("start", 1), ("done", 1), ("tx", 8),
                                 ("rx", 8), ("cs_n", 1), ("busy", 1)])
        spi.cs_n.reset = Constant(1, 1)
        self.slot, self.identity_ready = Signal(2), Signal()
        self.busy, self.done, self.mac_ready = Signal(), Signal(), Signal()
        self.mac_address = Signal(48)
        self.error = Signal(4)
        self.length, self.expected_crc = Signal(12), Signal(32)
        self.write = Signal()
        self.address = Signal(max=RAM_BYTES // 4)
        self.data = Signal(32)
        self.submodules.fsm = fsm = FSM(reset_state="IDENTITY")
        uid, header = Signal(reset=int(not with_firmware)), Signal(reset=1)
        command_byte = Signal(3)
        byte = Signal(2)
        word = Signal(10)
        data = Signal(32)
        crc = Signal(64, reset=0xffffffff if with_firmware else (1 << 64) - 1)
        bit = Signal(3)
        wait = Signal(max=byte_timeout)
        any_zero, any_one = Signal(), Signal()
        self.comb += [
            self.busy.eq(~fsm.ongoing("DONE")), spi.busy.eq(self.busy),
            self.address.eq(word), self.data.eq(data), spi.tx.eq(0),
            self.mac_address.eq(((~crc[:48]) & (((1 << 48) - 1) ^ (3 << 40))) | (2 << 40)),
        ]

        def fail(code):
            return [NextValue(self.error, code), NextValue(spi.cs_n, 1), NextState("RELEASE")]

        def wait_byte(success):
            return If(spi.done, NextValue(wait, 0), *success).Elif(
                wait == byte_timeout - 1, *fail(2)).Else(NextValue(wait, wait + 1))

        fsm.act("IDENTITY", If(self.identity_ready,
            If(with_firmware & (self.slot != 1) & (self.slot != 2), *fail(3))
            .Else(NextValue(spi.cs_n, 0), NextState("PREFIX"))))
        fsm.act("PREFIX", spi.start.eq(1),
            Case(command_byte, {
                0: spi.tx.eq(Mux(uid, 0x4b, 0x03)),
                1: spi.tx.eq(Mux(uid, 0, Mux(self.slot == 1, 0x38, 0x58))),
            }), NextState("PREFIX_WAIT"))
        fsm.act("PREFIX_WAIT", wait_byte([
            If(command_byte == Mux(uid, 4, 3), NextState("BYTE"))
            .Else(NextValue(command_byte, command_byte + 1), NextState("PREFIX"))]))
        fsm.act("BYTE", spi.start.eq(1), NextState("BYTE_WAIT"))
        fsm.act("BYTE_WAIT", wait_byte([
            NextValue(data, Cat(data[8:], spi.rx)),
            If(uid, NextValue(any_zero, any_zero | (spi.rx != 255)),
                    NextValue(any_one, any_one | (spi.rx != 0))),
            If(uid | ~header, NextValue(crc, crc ^ spi.rx),
                NextValue(bit, 0), NextState("HASH")).Else(NextState("BYTE_NEXT"))]))
        fsm.act("HASH", NextValue(crc, (crc >> 1) ^ Mux(crc[0],
            Mux(uid, 0xc96c5795d7870f42, 0xedb88320), 0)),
            If(bit == 7, NextState("BYTE_NEXT")).Else(NextValue(bit, bit + 1)))
        fsm.act("BYTE_NEXT", NextValue(byte, byte + 1),
            If(byte == 3, NextState("WORD")).Else(NextState("BYTE")))
        fsm.act("WORD",
            If(uid,
                If(word == 3,
                    If(~any_zero | ~any_one, *fail(8)).Else(
                        NextValue(self.mac_ready, 1), NextValue(spi.cs_n, 1),
                        NextState("RELEASE")))
                .Else(NextValue(word, word + 1), NextState("BYTE")))
            .Elif(header, Case(word, {
                0: If(data != FIRMWARE_MAGIC, *fail(6)).Else(
                    NextValue(word, 1), NextState("BYTE")),
                1: If((data == 0) | (data > RAM_BYTES - STACK_BYTES) | (data[:2] != 0),
                    *fail(6)).Else(NextValue(self.length, data),
                        NextValue(word, 2), NextState("BYTE")),
                2: [NextValue(self.expected_crc, data), NextValue(word, 3), NextState("BYTE")],
                3: If(data != RAM_BYTES, *fail(6)).Else(NextValue(header, 0),
                    NextValue(word, 0), NextState("BYTE")),
            }))
            .Else(self.write.eq(1),
                If(word + 1 == self.length[2:], NextValue(spi.cs_n, 1), NextState("CHECK"))
                .Else(NextValue(word, word + 1), NextState("BYTE"))))
        # A full idle byte time exceeds the flash's minimum CS-high interval.
        fsm.act("GAP", If(wait == 31, NextValue(wait, 0),
            NextValue(spi.cs_n, 0), NextState("PREFIX")).Else(NextValue(wait, wait + 1)))
        fsm.act("CHECK", If((crc[:32] ^ 0xffffffff) != self.expected_crc, *fail(7))
                .Else(NextValue(word, 0), NextValue(command_byte, 0), NextValue(wait, 0),
                      NextValue(uid, 1), NextValue(crc, (1 << 64) - 1), NextState("GAP")))
        # The shared engine registers chip select; retain ownership until
        # that final deassertion has reached the physical pin.
        fsm.act("RELEASE", NextValue(self.done, self.error == 0), NextState("DONE"))
        fsm.act("DONE")
