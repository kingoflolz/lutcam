"""Small, read-only A/B image selector for the golden FPGA configuration.

The scanner owns the flash controller's Wishbone interface while ``busy`` is
asserted.  It checks the committed manifest and complete image CRC32 before
requesting a remote update.  No flash mutation is performed by this module.
The surrounding SoC keeps Etherbone available when neither image is valid.
"""

from migen import Case, FSM, If, Module, Mux, NextState, NextValue, Signal
from litex.soc.interconnect import wishbone


FLASH_CSR_BASE = 0x30025000
SLOT_A = 0x200000
SLOT_B = 0x400000
SLOT_SIZE = 0x200000
MANIFEST_SECTOR_SIZE = 4096
MANIFEST_MAGIC = b"LUTCOTA1"
MANIFEST_COMMIT = 0x434F4D54


class GoldenBootScanner(Module):
    """Validate A, then B, and hold a selected-slot request until accepted.

    ``bus`` is a classic 32-bit Wishbone master with absolute word addresses.
    The caller must give it exclusive controller ownership while ``busy`` is
    high, including the intervals between transactions. ``request_slot`` uses
    1 for A and 2 for B. A remote-update error suppresses automatic boot so a
    configuration failure cannot create an endless golden/application loop.

    CRC32 is the reflected IEEE/zlib form (initial/final XOR 0xffffffff).
    The header checksum covers manifest bytes 0..39. Image bytes are read in
    batches of at most 256 and the final partial word is not padded into CRC.
    Every READ command is one complete ABI3 record containing its command ID,
    address, operation/length, and matching commit ID. Trusted record metadata
    identifies this scanner separately from network and USB request sources.
    Both Wishbone responses and flash-command completion have finite limits.
    """

    ERROR_NONE = 0
    ERROR_REMOTE_UPDATE = 1
    ERROR_BUS = 2
    ERROR_BUS_TIMEOUT = 3
    ERROR_FLASH_TIMEOUT = 4
    ERROR_FLASH = 5
    ERROR_MANIFEST = 6
    ERROR_HEADER_CRC = 7
    ERROR_IMAGE_CRC = 8

    def __init__(self, *, startup_cycles=10_000_000, wb_timeout_cycles=4096,
                 flash_timeout_cycles=25_000_000, csr_base=FLASH_CSR_BASE):
        if startup_cycles < 0 or wb_timeout_cycles < 1 or flash_timeout_cycles < 1:
            raise ValueError("golden scanner timeouts must be positive")
        if csr_base & 3:
            raise ValueError("flash CSR base must be word aligned")

        self.bus = bus = wishbone.Interface(data_width=32, addressing="word")
        self.busy = Signal()
        self.done = Signal()
        self.selected_slot = Signal(2)
        self.valid_a = Signal()
        self.valid_b = Signal()
        self.error = Signal(4)
        self.error_a = Signal(4)
        self.error_b = Signal(4)
        self.ru_error = Signal()
        self.request_valid = Signal()
        self.request_slot = Signal(2)
        self.request_ready = Signal()
        self.record_first = Signal()
        self.record_last = Signal()
        self.record_count = Signal(9)
        self.record_source = Signal(2)
        self.record_abort = Signal()

        self.submodules.fsm = fsm = FSM(reset_state="STARTUP")
        current_slot = Signal(2, reset=1)
        slot_base = Signal(24)
        metadata = Signal(reset=1)
        wb_wait = Signal(max=max(2, wb_timeout_cycles))
        # Startup and command completion never overlap. The first command
        # resets this shared timer before it begins its completion deadline.
        flash_wait = Signal(max=max(2, startup_cycles + 1, flash_timeout_cycles))
        operation_id = Signal(32)
        command_word = Signal(2)
        command_data = Signal(32)
        read_address = Signal(32)
        offset = Signal(21)
        remaining = Signal(21)
        read_length = Signal(9, reset=64)
        buffer_word = Signal(6)
        chunk_remaining = Signal(9)
        word_data = Signal(32)
        word_remaining = Signal(3)
        crc = Signal(32, reset=0xFFFFFFFF)
        image_crc = Signal(32)
        manifest_bad = Signal()
        header_bad = Signal()

        # Eight explicitly named stages keep the generated CRC network linear.
        # One byte per clock is small enough for the golden design and prevents
        # a wide multiword CRC expression on a critical path.
        crc_stages = [Signal(32) for _ in range(9)]
        self.comb += crc_stages[0].eq(crc ^ word_data[:8])
        for previous, following in zip(crc_stages, crc_stages[1:]):
            self.comb += following.eq((previous >> 1) ^
                                     Mux(previous[0], 0xEDB88320, 0))

        self.comb += [
            slot_base.eq(Mux(current_slot == 1, SLOT_A, SLOT_B)),
            read_address.eq(slot_base +
                Mux(metadata, SLOT_SIZE - MANIFEST_SECTOR_SIZE, offset)),
            self.done.eq(fsm.ongoing("REQUEST") | fsm.ongoing("DONE")),
            self.busy.eq(~self.done),
            self.request_valid.eq(fsm.ongoing("REQUEST") & ~self.ru_error),
            self.request_slot.eq(self.selected_slot),
            self.record_count.eq(4),
            self.record_source.eq(2),
            command_data.eq(Mux(command_word == 1, read_address,
                Mux(command_word == 2, 1 | (read_length << 8), operation_id))),
            bus.sel.eq(0xF),
        ]

        def fail(code):
            return [NextValue(self.error, code),
                    If(current_slot == 1, NextValue(self.error_a, code)).Else(
                        NextValue(self.error_b, code)),
                    NextValue(wb_wait, 0), NextValue(flash_wait, 0),
                    NextState("NEXT_SLOT")]

        def transaction(address, write, data, success):
            """Hold a request through ACK/ERR; every bus state is bounded."""
            return [bus.cyc.eq(1), bus.stb.eq(1), bus.we.eq(write),
                    bus.adr.eq((csr_base >> 2) + address), bus.dat_w.eq(data),
                    If(bus.err, self.record_abort.eq(1),
                        *fail(self.ERROR_BUS)).Elif(bus.ack,
                        NextValue(wb_wait, 0), *success,
                    ).Elif(wb_wait == wb_timeout_cycles - 1,
                        self.record_abort.eq(1), *fail(self.ERROR_BUS_TIMEOUT),
                    ).Else(NextValue(wb_wait, wb_wait + 1))]

        fsm.act("STARTUP",
            If(flash_wait == startup_cycles,
                If(self.ru_error,
                    NextValue(self.error, self.ERROR_REMOTE_UPDATE),
                    NextState("DONE"),
                ).Else(NextState("CANDIDATE")),
            ).Else(NextValue(flash_wait, flash_wait + 1)))

        fsm.act("CANDIDATE",
            NextValue(metadata, 1), NextValue(read_length, 64),
            NextValue(offset, 0), NextValue(crc, 0xFFFFFFFF),
            NextValue(manifest_bad, 0), NextValue(header_bad, 0),
            NextValue(buffer_word, 0), NextState("READ_ID"))

        fsm.act("READ_ID", *transaction(0x1C // 4, 0, 0, [
            If(bus.dat_r == 0xFFFFFFFF, *fail(self.ERROR_FLASH)).Else(
                NextValue(operation_id, bus.dat_r + 1), NextValue(command_word, 0),
                NextState("RECORD_GAP"))]))
        # Each word has a separate classic transfer, but all four belong to
        # the same trusted record. Only its final matching ID commits work.
        fsm.act("RECORD_GAP", NextState("COMMAND_RECORD"))
        fsm.act("COMMAND_RECORD",
            self.record_first.eq(command_word == 0),
            self.record_last.eq(command_word == 3),
            *transaction(0x400 // 4 + command_word, 1, command_data, [
                If(command_word == 3,
                    NextValue(flash_wait, 0), NextState("WAIT_DONE"),
                ).Else(NextValue(command_word, command_word + 1),
                    NextState("RECORD_GAP"))]))

        # Completion polling has a separate deadline; an indefinitely busy
        # flash device must leave the network recovery image usable.
        fsm.act("WAIT_DONE", *transaction(0x20 // 4, 0, 0, [
            If(bus.dat_r == operation_id,
                NextState("READ_ERROR"),
            ).Elif(flash_wait == flash_timeout_cycles - 1,
                *fail(self.ERROR_FLASH_TIMEOUT),
            ).Else(NextState("POLL_GAP"))]),
            If(flash_wait != flash_timeout_cycles - 1,
                NextValue(flash_wait, flash_wait + 1)))
        fsm.act("POLL_GAP",
            If(flash_wait == flash_timeout_cycles - 1,
                *fail(self.ERROR_FLASH_TIMEOUT),
            ).Else(NextValue(flash_wait, flash_wait + 1), NextState("WAIT_DONE")))
        fsm.act("READ_ERROR", *transaction(0x24 // 4, 0, 0, [
            If(bus.dat_r != 0, *fail(self.ERROR_FLASH)).Else(
                NextValue(buffer_word, 0),
                NextValue(chunk_remaining, read_length),
                If(metadata, NextState("MANIFEST_WORD")).Else(
                    NextState("IMAGE_WORD")))]))

        # Check fixed fields as they arrive instead of retaining whole words.
        # Continue reading every manifest word even after an error, preserving
        # the bus sequence and manifest-error priority over header-CRC errors.
        def check_field(expected):
            return If(bus.dat_r != expected, NextValue(manifest_bad, 1))

        fields = {
            0: check_field(int.from_bytes(MANIFEST_MAGIC[:4], "little")),
            1: check_field(int.from_bytes(MANIFEST_MAGIC[4:], "little")),
            2: check_field(0),
            3: check_field(current_slot),
            4: [If((bus.dat_r == 0) |
                   (bus.dat_r > SLOT_SIZE - MANIFEST_SECTOR_SIZE),
                   NextValue(manifest_bad, 1)),
                # Validate all 32 bits before narrowing; malformed lengths
                # must never alias a small valid image.
                NextValue(remaining, bus.dat_r[:21])],
            5: NextValue(image_crc, bus.dat_r),
            10: NextValue(header_bad, (crc ^ 0xFFFFFFFF) != bus.dat_r),
            11: check_field(MANIFEST_COMMIT),
        }
        fsm.act("MANIFEST_WORD", *transaction(0x100 // 4 + buffer_word, 0, 0, [
            Case(buffer_word, fields),
            NextValue(word_data, bus.dat_r), NextValue(word_remaining, 4),
            If(buffer_word < 10, NextState("MANIFEST_CRC")).Elif(
                buffer_word == 15, NextState("VALIDATE_MANIFEST"),
            ).Else(NextValue(buffer_word, buffer_word + 1))]))
        fsm.act("MANIFEST_CRC",
            NextValue(crc, crc_stages[-1]), NextValue(word_data, word_data >> 8),
            If(word_remaining == 1,
                NextValue(buffer_word, buffer_word + 1), NextState("MANIFEST_WORD"),
            ).Else(NextValue(word_remaining, word_remaining - 1)))
        fsm.act("VALIDATE_MANIFEST",
            If(manifest_bad, *fail(self.ERROR_MANIFEST)).Elif(
                header_bad, *fail(self.ERROR_HEADER_CRC),
            ).Else(NextValue(metadata, 0), NextValue(crc, 0xFFFFFFFF),
                NextValue(offset, 0),
                NextState("IMAGE_CHUNK")))

        fsm.act("IMAGE_CHUNK",
            NextValue(read_length, Mux(remaining > 256, 256, remaining)),
            NextState("READ_ID"))
        fsm.act("IMAGE_WORD", *transaction(0x100 // 4 + buffer_word, 0, 0, [
            NextValue(word_data, bus.dat_r),
            NextValue(word_remaining, Mux(chunk_remaining > 4, 4, chunk_remaining)),
            NextState("IMAGE_CRC")]))
        fsm.act("IMAGE_CRC",
            NextValue(crc, crc_stages[-1]), NextValue(word_data, word_data >> 8),
            NextValue(chunk_remaining, chunk_remaining - 1),
            If(chunk_remaining == 1,
                NextValue(remaining, remaining - read_length),
                NextValue(offset, offset + read_length), NextState("CHUNK_DONE"),
            ).Elif(word_remaining == 1,
                NextValue(buffer_word, buffer_word + 1), NextState("IMAGE_WORD"),
            ).Else(NextValue(word_remaining, word_remaining - 1)))
        fsm.act("CHUNK_DONE",
            If(remaining != 0, NextState("IMAGE_CHUNK")).Elif(
                (crc ^ 0xFFFFFFFF) != image_crc, *fail(self.ERROR_IMAGE_CRC),
            ).Elif(self.ru_error,
                NextValue(self.error, self.ERROR_REMOTE_UPDATE), NextState("DONE"),
            ).Else(NextValue(self.selected_slot, current_slot),
                If(current_slot == 1, NextValue(self.valid_a, 1)).Else(
                    NextValue(self.valid_b, 1)),
                NextValue(self.error, self.ERROR_NONE), NextState("REQUEST")))
        fsm.act("NEXT_SLOT",
            If(current_slot == 1,
                NextValue(current_slot, 2), NextState("CANDIDATE"),
            ).Else(NextState("DONE")))
        fsm.act("REQUEST",
            If(self.ru_error,
                NextValue(self.error, self.ERROR_REMOTE_UPDATE),
                NextValue(self.selected_slot, 0), NextState("DONE"),
            ).Elif(self.request_ready, NextState("DONE")))
        fsm.act("DONE")
