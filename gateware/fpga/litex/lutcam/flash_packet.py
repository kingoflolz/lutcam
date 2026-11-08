"""Complete, transport-validated page records over Wishbone.

The record bridge must supply trustworthy per-word packet metadata and must
not forward an Ethernet frame before its FCS has passed. This controller does
not authenticate arbitrary raw Wishbone writes as transport records. The register at 0x04 is reserved zero; buffer/transfer registers describe
actual capacity and layout.
"""

from functools import reduce
from operator import or_
from types import SimpleNamespace

from migen import (Array, Cat, Case, ClockSignal, Constant, FSM, If, Instance, Memory, Module, Mux,
                   NextState, NextValue, Replicate, ResetSignal, Signal)
from litex.soc.interconnect import wishbone

from .flash import (_SPIByte, FLASH_BASE, FLASH_SIZE, FLASH_MAGIC, SLOT_SIZE,
                    READ, PROGRAM, ERASE4K, ERASE64K, JEDEC, STATUS,
                    ERR_NONE, ERR_RANGE, ERR_PROTECTED, ERR_LENGTH,
                    ERR_ALIGNMENT, ERR_OPCODE, ERR_WEL, ERR_TIMEOUT)

BUFFER_SIZE = 256
BUFFER_OFFSET = 0x100
COMMAND_OFFSET = 0x400
MAX_RECORD_WORDS = 68


class _PageSPITransaction(Module):
    """One command, optional address, and bounded write/read data phase."""

    def __init__(self, pads, half_period, *, address_is_stable=False, boot_client=None):
        self.start, self.done = Signal(), Signal()
        self.opcode = Signal(8)
        self.address = Signal(24)
        self.with_address = Signal()
        self.write_length, self.read_length = Signal(9), Signal(9)
        # A transaction has one payload direction. Both clients address the
        # same counter; neither consumes its index outside its own phase.
        self.write_index = self.read_index = payload_index = Signal(8)
        self.write_data, self.read_data = Signal(8), Signal(8)
        self.read_strobe = Signal()
        self.cs_n = cs_n = Signal(reset=1)
        cs_n.name_override = "flash_cs_n"
        cs_n.attr.add("no_retiming")
        self.submodules.byte = byte = _SPIByte(pads, half_period)
        start, tx = Signal(), Signal(8)
        self.comb += [byte.start.eq(start), byte.tx.eq(tx)]
        if boot_client is not None:
            self.comb += [boot_client.done.eq(byte.done), boot_client.rx.eq(byte.rx),
                If(boot_client.busy, byte.start.eq(boot_client.start), byte.tx.eq(boot_client.tx))]
            # Both owners release CS before handing over the byte engine.
            self.sync += If(boot_client.busy, cs_n.eq(boot_client.cs_n))
        # Only the packet controller can guarantee that its descriptor address
        # stays unchanged until this whole transaction finishes. Other callers
        # retain the usual snapshot-at-start behavior.
        if address_is_stable:
            command, address = Signal(8), self.address
        else:
            command, address = Signal(8), Signal(24)
        address_enabled = Signal()
        payload_is_write = Signal()
        remaining = Signal(9)
        address_index = Signal(2)
        phase = Signal(2)
        COMMAND, ADDRESS, WRITE, READ_DATA = range(4)
        gap = Signal(max=max(2, half_period * 2))
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += [
            # Drive the physical select from one FF, rather than a decode of
            # multiple FSM bits that could glitch between active states.
            pads.cs_n.eq(cs_n),
            self.read_data.eq(byte.rx),
            self.read_strobe.eq(fsm.ongoing("WAIT") & byte.done & (phase == READ_DATA)),
            Case(phase, {
                COMMAND: tx.eq(command),
                ADDRESS: tx.eq(Array([address[0:8], address[8:16], address[16:24]])[address_index]),
                WRITE: tx.eq(self.write_data),
                READ_DATA: tx.eq(0xff),
            }),
        ]

        def payload():
            return If(remaining != 0,
                NextValue(phase, Mux(payload_is_write, WRITE, READ_DATA)), NextState("PREPARE"),
            ).Else(NextValue(cs_n, 1), NextValue(gap, half_period * 2 - 1), NextState("GAP"))

        fsm.act("IDLE",
            If(self.start,
                NextValue(cs_n, 0),
                NextValue(command, self.opcode),
                *([] if address_is_stable else [NextValue(address, self.address)]),
                NextValue(address_enabled, self.with_address),
                NextValue(payload_is_write, self.write_length != 0),
                NextValue(remaining, Mux(self.write_length != 0, self.write_length, self.read_length)),
                NextValue(payload_index, 0),
                NextValue(address_index, 2), NextValue(phase, COMMAND), NextState("PREPARE"),
            ),
        )
        # This extra cycle also lets the synchronous page-buffer RAM settle
        # after changing its byte index before the next byte captures data.
        fsm.act("PREPARE", NextState("SEND"))
        fsm.act("SEND", start.eq(1), NextState("WAIT"))
        fsm.act("WAIT",
            If(byte.done,
                Case(phase, {
                    COMMAND: If(address_enabled,
                        NextValue(phase, ADDRESS), NextState("PREPARE"),
                    ).Else(payload()),
                    ADDRESS: If(address_index != 0,
                        NextValue(address_index, address_index - 1), NextState("PREPARE"),
                    ).Else(payload()),
                    WRITE: If(remaining == 1,
                        NextValue(cs_n, 1), NextValue(gap, half_period * 2 - 1), NextState("GAP"),
                    ).Else(NextValue(remaining, remaining - 1),
                        NextValue(payload_index, payload_index + 1), NextState("PREPARE")),
                    READ_DATA: If(remaining == 1,
                        NextValue(cs_n, 1), NextValue(gap, half_period * 2 - 1), NextState("GAP"),
                    ).Else(NextValue(remaining, remaining - 1),
                        NextValue(payload_index, payload_index + 1), NextState("PREPARE")),
                }),
            ),
        )
        fsm.act("GAP",
            If(gap == 0, self.done.eq(1), NextState("IDLE"))
            .Else(NextValue(gap, gap - 1)),
        )


class PacketFlashController(Module):
    """One independently validated command per 4..68-word transport record.

    At 0x400, incrementing word addresses carry request ID, address,
    ``opcode | (length << 8)``, PROGRAM data words, and the matching commit ID.
    READ/PROGRAM lengths are 1..256 and cannot cross a 256-byte page. Other
    opcodes use length zero. The record count includes the header and commit.
    Only the exact next nonzero ID can execute. Other IDs and new commands
    received while busy/inhibited are drained as harmless no-ops.
    Ignored records check their outer framing but do not decode their body.
    Malformed/incomplete records consume no ID. Completed invalid descriptors
    consume the next ID and report the usual descriptor error without SPI.

    Metadata remains stable through each Wishbone response. Source-qualified
    abort bits cancel their owner's partial record even with no bus request.
    A foreign source cannot replace an owner or corrupt its partial payload.
    The 256-byte result buffer at 0x100 is read-only to the host. All command
    writes require full byte selects. Registers at 0x10/14/18/34 read as zero
    so a contiguous completion snapshot remains possible.
    """

    def __init__(self, pads=None, *, clk_freq=120_000_000,
                 spi_clk_freq=30_000_000, boot_slot=0, poll_timeout_cycles=None,
                 max_transport_words=128, use_dsp_poll=False, use_dsp_storage=False,
                 share_spi_address=False, golden_maintenance=False, boot_client=None):
        if clk_freq <= 0 or spi_clk_freq <= 0 or spi_clk_freq > clk_freq // 4:
            raise ValueError("SPI frequency must be positive and at most sys/4")
        if isinstance(boot_slot, int) and not 0 <= boot_slot < 8:
            raise ValueError("boot_slot must be in 0..7")
        if not 68 <= max_transport_words <= 128:
            raise ValueError("packet transport must support 68..128 words")
        half_period = max(2, (int(clk_freq) + 2 * int(spi_clk_freq) - 1) // (2 * int(spi_clk_freq)))
        self.spi_half_period = half_period
        timeout = int(clk_freq) * 5 if poll_timeout_cycles is None else int(poll_timeout_cycles)
        if timeout < 1:
            raise ValueError("poll timeout must be positive")
        self.bus = bus = wishbone.Interface(data_width=32, addressing="word")
        self.inhibit, self.busy = Signal(), Signal()
        self.record_first, self.record_last, self.record_abort = Signal(), Signal(), Signal()
        self.record_count, self.record_source = Signal(9), Signal(2)
        self.record_abort_sources = Signal(4)
        self.boot_slot = slot = Signal(3)
        self.comb += slot.eq(boot_slot)
        self.pads = pads = pads if pads is not None else SimpleNamespace(
            clk=Signal(), cs_n=Signal(reset=1), mosi=Signal(), miso=Signal(),
            wp=Signal(reset=1), hold=Signal(reset=1))
        self.comb += [pads.wp.eq(1), pads.hold.eq(1)]
        self.submodules.spi = spi = _PageSPITransaction(
            pads, half_period, address_is_stable=share_spi_address, boot_client=boot_client)
        self.accepted_id, self.done_id = accepted, done = Signal(32), Signal(32)
        self.error = error = Signal(4)
        self.result = result = Signal(24)
        address, length, opcode = Signal(24), Signal(9), Signal(3)
        if share_spi_address:
            # New records are replay-only while the engine is busy. Its sole
            # address writer is therefore disabled through every SPI command,
            # including completion polls, source aborts and withdrawn bus cycles.
            self.comb += spi.address.eq(address)
        address_high = Signal()
        descriptor_error = Signal(4)
        pending_id = Signal(32)
        pending_capture = Signal()
        owner = Signal(2)
        packet_active, replay = Signal(), Signal()
        packet_count = Signal(9)
        packet_index = Signal(7)
        self.packet_active = packet_active
        start, complete = Signal(), Signal()
        operation_error = Signal(4)
        abort_bits = Array(self.record_abort_sources[index] for index in range(4))
        owner_abort = packet_active & (abort_bits[owner] | (self.inhibit & ~replay))
        word_abort = self.record_abort | abort_bits[self.record_source]
        destructive = (opcode == PROGRAM) | (opcode == ERASE4K) | (opcode == ERASE64K)

        # Validate every original header bit before narrowing stored fields.
        incoming_opcode, incoming_length = bus.dat_w[:8], bus.dat_w[8:17]
        incoming_error = Signal(4)
        commit_error = Signal(4)
        page_end = Signal(9)
        if golden_maintenance:
            # A temporary application image may replace golden while executing
            # from A or B. Every other flash slot remains read-only, including
            # both application images. There is deliberately no runtime unlock.
            protected = (address[21:24] != 0) | ((slot != 1) & (slot != 2))
        else:
            protected = (address[21:24] == 0) | ((slot != 0) & (address[21:24] == slot))
        self.comb += [
            commit_error.eq(Mux(descriptor_error != 0, descriptor_error,
                                Mux(destructive & protected, ERR_PROTECTED, 0))),
            page_end.eq(Cat(address[:8], Constant(0, 1)) + incoming_length),
            Case(incoming_opcode, {
                READ: If((incoming_length == 0) | (incoming_length > 256), incoming_error.eq(ERR_LENGTH))
                    .Elif(address_high, incoming_error.eq(ERR_RANGE))
                    .Elif(page_end > 256, incoming_error.eq(ERR_ALIGNMENT)),
                PROGRAM: If((incoming_length == 0) | (incoming_length > 256), incoming_error.eq(ERR_LENGTH))
                    .Elif(address_high, incoming_error.eq(ERR_RANGE))
                    .Elif(page_end > 256, incoming_error.eq(ERR_ALIGNMENT))
                    .Elif(protected, incoming_error.eq(ERR_PROTECTED)),
                ERASE4K: If(incoming_length != 0, incoming_error.eq(ERR_LENGTH))
                    .Elif(address_high, incoming_error.eq(ERR_RANGE))
                    .Elif(address[:12] != 0, incoming_error.eq(ERR_ALIGNMENT))
                    .Elif(protected, incoming_error.eq(ERR_PROTECTED)),
                ERASE64K: If(incoming_length != 0, incoming_error.eq(ERR_LENGTH))
                    .Elif(address_high, incoming_error.eq(ERR_RANGE))
                    .Elif(address[:16] != 0, incoming_error.eq(ERR_ALIGNMENT))
                    .Elif(protected, incoming_error.eq(ERR_PROTECTED)),
                JEDEC: If(incoming_length != 0, incoming_error.eq(ERR_LENGTH)),
                STATUS: If(incoming_length != 0, incoming_error.eq(ERR_LENGTH)),
                "default": incoming_error.eq(ERR_OPCODE),
            }),
        ]

        # Byte storage fits one RAM10; bus words are assembled over four
        # bounded cycles. SPI reservations retain their original clock timing.
        page = Memory(8, BUFFER_SIZE, init=[0] * BUFFER_SIZE)
        port = page.get_port(write_capable=True)
        self.specials += [page, port]
        cpu_request, cpu_write, cpu_grant = Signal(), Signal(), Signal()
        cpu_address, cpu_data = Signal(8), Signal(8)
        cpu_lane = Signal(2)
        cpu_base = Signal(6)
        cpu_shift = Signal(32)
        spi_read, spi_write = Signal(), Signal()
        self.submodules.engine = engine = FSM(reset_state="IDLE")
        self.comb += [
            self.busy.eq(~engine.ongoing("IDLE")),
            spi_read.eq(engine.ongoing("EXEC_WAIT") & (opcode == PROGRAM) & spi.fsm.ongoing("PREPARE")),
            spi_write.eq(engine.ongoing("EXEC_WAIT") & ~destructive & spi.read_strobe),
            spi.write_data.eq(port.dat_r),
            cpu_grant.eq(cpu_request & ~spi_read & ~spi_write),
            port.adr.eq(Mux(spi_write, spi.read_index, Mux(spi_read, spi.write_index, cpu_address))),
            port.dat_w.eq(Mux(spi_write, spi.read_data, cpu_data)),
            port.we.eq(spi_write | (cpu_grant & cpu_write)),
        ]
        last_status = Signal(8)
        poll_count = Signal(max=max(2, timeout))
        polling = Signal()
        self.comb += polling.eq(engine.ongoing("INITIAL_START") | engine.ongoing("INITIAL_WAIT") |
                                engine.ongoing("POLL_START") | engine.ongoing("POLL_WAIT"))
        if use_dsp_poll:
            if len(poll_count) > 48:
                raise ValueError("DSP poll timeout must fit 48 bits")
            poll_result = Signal(48)
            self.specials += Instance("EFX_DSP48",
                p_MODE="NORMAL", p_SIGNED=0, p_M_SEL="C", p_N_SEL="W",
                p_W_SEL="X", p_W_REG=1, p_RST_SYNC=1,
                i_A=Constant(0, 19), i_B=Constant(0, 18), i_C=Constant(1, 18),
                i_CASCIN=Constant(0, 48), i_OP=Constant(0, 2), i_SHIFT_ENA=0,
                i_CLK=ClockSignal(), i_CE=poll_count != timeout - 1,
                i_RST=ResetSignal() | ~polling, o_O=poll_result)
            self.comb += poll_count.eq(poll_result[:len(poll_count)])
        else:
            self.sync += If(~polling, poll_count.eq(0)).Elif(
                poll_count != timeout - 1, poll_count.eq(poll_count + 1))
        self.sync += [
            If(spi.read_strobe, last_status.eq(spi.read_data)),
            If(start, result.eq(0)).Elif(engine.ongoing("EXEC_WAIT") & spi.read_strobe,
                If(opcode == JEDEC, result.eq(Cat(spi.read_data, result[:16])))
                .Elif(opcode == STATUS, result.eq(spi.read_data))),
        ]

        def status_transaction():
            return [spi.opcode.eq(0x05), spi.read_length.eq(1), spi.start.eq(1)]

        engine.act("IDLE", If(start, NextState("INITIAL_START")))
        engine.act("INITIAL_START", *status_transaction(), NextState("INITIAL_WAIT"))
        engine.act("INITIAL_WAIT", If(spi.done,
            If(last_status[0],
                If(poll_count == timeout - 1, NextState("TIMEOUT")).Else(NextState("INITIAL_START")))
            .Elif(destructive, NextState("WREN_START")).Else(NextState("EXEC_START"))))
        engine.act("WREN_START", spi.opcode.eq(0x06), spi.start.eq(1), NextState("WREN_WAIT"))
        engine.act("WREN_WAIT", If(spi.done, NextState("WEL_START")))
        engine.act("WEL_START", *status_transaction(), NextState("WEL_WAIT"))
        engine.act("WEL_WAIT", If(spi.done,
            If(~last_status[1], NextState("WEL_ERROR")).Else(NextState("EXEC_START"))))
        engine.act("EXEC_START", spi.start.eq(1),
            *([] if share_spi_address else [spi.address.eq(address)]),
            Case(opcode, {
                READ: [spi.opcode.eq(0x03), spi.with_address.eq(1), spi.read_length.eq(length)],
                PROGRAM: [spi.opcode.eq(0x02), spi.with_address.eq(1), spi.write_length.eq(length)],
                ERASE4K: [spi.opcode.eq(0x20), spi.with_address.eq(1)],
                ERASE64K: [spi.opcode.eq(0xd8), spi.with_address.eq(1)],
                JEDEC: [spi.opcode.eq(0x9f), spi.read_length.eq(3)],
                STATUS: [spi.opcode.eq(0x05), spi.read_length.eq(1)],
            }), NextState("EXEC_WAIT"))
        engine.act("EXEC_WAIT", If(spi.done,
            If(destructive, NextState("POLL_START")).Else(NextState("DONE"))))
        engine.act("POLL_START", *status_transaction(), NextState("POLL_WAIT"))
        engine.act("POLL_WAIT", If(spi.done,
            If(last_status[0],
                If(poll_count == timeout - 1, NextState("TIMEOUT")).Else(NextState("POLL_START")))
            .Else(NextState("DONE"))))
        engine.act("DONE", complete.eq(1), NextState("IDLE"))
        engine.act("WEL_ERROR", complete.eq(1), operation_error.eq(ERR_WEL), NextState("IDLE"))
        engine.act("TIMEOUT", complete.eq(1), operation_error.eq(ERR_TIMEOUT), NextState("IDLE"))

        self.submodules.wb = wb = FSM(reset_state="IDLE")
        response, response_error = Signal(32), Signal()
        request = bus.cyc & bus.stb
        record_address = (bus.adr[:10] >= COMMAND_OFFSET // 4) & (bus.adr[:10] < COMMAND_OFFSET // 4 + MAX_RECORD_WORDS)
        buffer_address = (bus.adr[:10] >= BUFFER_OFFSET // 4) & (bus.adr[:10] < (BUFFER_OFFSET + BUFFER_SIZE) // 4)
        new_command, invalid_descriptor = Signal(), Signal()
        expected_count = Signal(9)
        self.comb += [
            bus.dat_r.eq(response),
            bus.ack.eq(request & wb.ongoing("RESPONSE") & ~response_error),
            bus.err.eq(request & wb.ongoing("RESPONSE") & response_error),
            expected_count.eq(4 + Mux(incoming_opcode == PROGRAM,
                                     (Cat(incoming_length, Constant(0, 1)) + 3) >> 2, 0)),
            cpu_address.eq(Cat(cpu_lane, cpu_base)),
            cpu_data.eq(cpu_shift[:8]),
        ]
        read_values = {
            0x00 // 4: FLASH_MAGIC,
            0x04 // 4: 0,
            0x08 // 4: Cat(self.busy, error != 0),
            0x0c // 4: FLASH_SIZE,
            0x10 // 4: 0, 0x14 // 4: 0, 0x18 // 4: 0, 0x34 // 4: 0,
            0x1c // 4: accepted,
            0x20 // 4: done,
            0x24 // 4: error,
            0x28 // 4: result,
            0x2c // 4: slot,
            0x30 // 4: int(clk_freq) // (2 * half_period),
            0x38 // 4: BUFFER_SIZE,
            0x3c // 4: max_transport_words,
            0x40 // 4: BUFFER_OFFSET,
        }
        # Each register address enables one value. Keep data selection separate
        # from validity and the existing response/RAM-access state machine.
        selected_read = Signal(32)
        self.comb += selected_read.eq(reduce(or_, (
            value & Replicate(bus.adr[:10] == word, 32)
            for word, value in read_values.items())))
        reads = {word: [] for word in read_values}
        reads["default"] = NextValue(response_error, 1)

        def reject(clear=True):
            actions = [NextValue(response_error, 1)]
            if clear:
                actions.append(NextValue(packet_active, 0))
            return actions

        wb.act("IDLE", If(request,
            NextValue(response, 0), NextValue(response_error, 0), NextState("RESPONSE"),
            If(~bus.we,
                If(buffer_address,
                    NextValue(cpu_base, bus.adr[:6]), NextValue(cpu_lane, 0),
                    NextValue(cpu_shift, 0), NextState("READ_REQUEST"),
                ).Else(NextValue(response, selected_read), Case(bus.adr[:10], reads)),
            ).Elif(~record_address | (bus.sel != 0xf),
                NextValue(response_error, 1),
                If(packet_active & (self.record_source == owner), NextValue(packet_active, 0)),
            ).Elif(owner_abort, *reject())
             .Elif(packet_active & (self.record_source != owner), *reject(False))
             .Elif(word_abort, *reject())
             .Elif(self.record_first,
                If((bus.adr[:10] != COMMAND_OFFSET // 4) | self.record_last |
                   (self.record_count < 4) | (self.record_count > MAX_RECORD_WORDS), *reject())
                .Elif(self.busy | self.inhibit | (accepted == 0xffffffff) |
                      (bus.dat_w != accepted + 1),
                    NextValue(packet_active, 1), NextValue(replay, 1),
                    (pending_capture.eq(1) if use_dsp_storage else NextValue(pending_id, bus.dat_w)), NextValue(packet_count, self.record_count),
                    NextValue(packet_index, 1), NextValue(owner, self.record_source),
                ).Else(NextValue(packet_active, 1), NextValue(replay, 0),
                    (pending_capture.eq(1) if use_dsp_storage else NextValue(pending_id, bus.dat_w)), NextValue(packet_count, self.record_count),
                    NextValue(packet_index, 1), NextValue(owner, self.record_source),
                    NextValue(descriptor_error, 0)),
            ).Elif(~packet_active, *reject(False))
             .Elif((self.record_count != packet_count) |
                   (bus.adr[:10] != COMMAND_OFFSET // 4 + packet_index) |
                   (self.record_last != (packet_index == packet_count - 1)), *reject())
             .Elif(self.record_last,
                NextValue(packet_active, 0),
                If(bus.dat_w != pending_id, NextValue(response_error, 1))
                .Elif(~replay,
                    new_command.eq(1),
                    If(commit_error != 0, invalid_descriptor.eq(1)).Else(start.eq(1))),
            ).Else(
                NextValue(packet_index, packet_index + 1),
                If(~replay,
                    If(packet_index == 1,
                        NextValue(address, bus.dat_w[:24]), NextValue(address_high, bus.dat_w[24:] != 0),
                    ).Elif(packet_index == 2,
                        If((bus.dat_w[17:] != 0) | (expected_count != packet_count), *reject())
                        .Else(NextValue(opcode, incoming_opcode[:3]), NextValue(length, incoming_length),
                              NextValue(descriptor_error, incoming_error)),
                    ).Else(
                        If(descriptor_error == 0,
                            NextValue(cpu_base, packet_index - 3), NextValue(cpu_lane, 0),
                            NextValue(cpu_shift, bus.dat_w), NextState("WRITE_BYTES"))),
                ),
            ),
        ))
        wb.act("WRITE_BYTES",
            If(owner_abort | word_abort, *reject(), NextState("RESPONSE"))
            .Else(cpu_request.eq(1), cpu_write.eq(1),
                If(cpu_grant,
                    NextValue(cpu_shift, cpu_shift >> 8),
                    If(cpu_lane == 3, NextState("RESPONSE"))
                    .Else(NextValue(cpu_lane, cpu_lane + 1)))),
        )
        wb.act("READ_REQUEST", cpu_request.eq(1),
               If(cpu_grant, NextState("READ_CAPTURE")))
        wb.act("READ_CAPTURE",
            NextValue(cpu_shift, Cat(cpu_shift[8:], port.dat_r)),
            If(cpu_lane == 3,
                NextValue(response, Cat(cpu_shift[8:], port.dat_r)), NextState("RESPONSE"),
            ).Else(NextValue(cpu_lane, cpu_lane + 1), NextState("READ_REQUEST")))
        wb.act("RESPONSE", NextState("IDLE"))
        if use_dsp_storage:
            for target, source, capture in (
                (pending_id, bus.dat_w, pending_capture),
                (accepted, pending_id, new_command),
                (done, Mux(complete, accepted, pending_id),
                    complete | (new_command & invalid_descriptor)),
            ):
                output = Signal(48)
                self.specials += Instance("EFX_DSP48",
                    p_MODE="NORMAL", p_SIGNED=0, p_M_SEL="P", p_N_SEL="C", p_C_EXT="ALIGN_LEFT",
                    p_W_SEL="X", p_W_REG=1, p_RST_SYNC=1,
                    i_A=source[:19], i_B=Constant(1, 18),
                    i_C=Cat(source[19:32], Constant(0, 5)),
                    i_CASCIN=Constant(0, 48), i_OP=Constant(0, 2), i_SHIFT_ENA=0,
                    i_CLK=ClockSignal(), i_CE=capture, i_RST=ResetSignal(), o_O=output)
                self.comb += target.eq(Cat(output[:19], output[30:43]))
        self.sync += [
            If(owner_abort, packet_active.eq(0)),
            If(new_command,
                *([] if use_dsp_storage else [accepted.eq(pending_id)]), error.eq(0),
                If(invalid_descriptor,
                    *([] if use_dsp_storage else [done.eq(pending_id)]), error.eq(commit_error))),
            If(complete, *([] if use_dsp_storage else [done.eq(accepted)]), error.eq(operation_error)),
        ]
