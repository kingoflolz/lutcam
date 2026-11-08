"""Transport record boundaries alongside the existing host Wishbone buses.

Etherbone metadata describes one write or read section, as exported by the
pinned LiteEth master. UART metadata describes one complete UARTBone command.
Neither boundary is inferred from CYC: UARTBone drops it between words, and
Etherbone drops it while returning each read response.
"""

from migen import Cat, Case, ClockDomainsRenamer, If, Module, Mux, NextState, NextValue
from migen import FSM, ResetInserter, Signal, Instance, ClockSignal, ResetSignal, Constant
from litex.gen import LiteXModule
from litex.gen.genlib.misc import WaitTimer
from litex.soc.cores.uart import (
    CMD_READ_BURST_FIXED, CMD_READ_BURST_INCR,
    CMD_WRITE_BURST_FIXED, CMD_WRITE_BURST_INCR,
)
from litex.soc.interconnect import stream, wishbone


def _record_signals(owner, source_id):
    if not isinstance(source_id, int) or not 0 <= source_id < 4:
        raise ValueError("record source_id must fit two bits")
    owner.record_first = Signal()
    owner.record_last = Signal()
    owner.record_count = Signal(9)
    owner.record_source = Signal(2)
    owner.record_abort = Signal()
    # Connect guard_fault to this host's WishboneGuard.fault_event, never its
    # sticky diagnostic fault bit. Reset is a stream/parser reset, not CYC.
    owner.guard_fault = Signal()
    owner.command_reset = Signal()
    owner.abort = owner.record_abort
    owner.comb += owner.record_source.eq(source_id)


class EtherboneRecordMetadata(Module):
    """Observe a LiteEthEtherboneWishboneMaster without changing its traffic.

    Instantiate beside the existing master, before its WishboneGuard. Metadata
    is valid for every held CYC/STB word and remains stable through ACK. The
    stream's count/last delimit sections, including a write followed by reads.
    A guard failure poisons the remaining section even though the guard ACKs
    failed words. Invalid count/last or changing parameters also poison it.
    The upstream receiver must validate declared section lengths against the
    complete packet length: these MMAP fields alone cannot identify a short
    physical packet whose decoder continues into the following packet.

    Route record_abort into a source-qualified abort vector, independently of
    the bus arbiter: a failure can remain asserted while no Wishbone word is
    selected. command_reset must accompany a reset of the observed transport.
    """

    def __init__(self, master, *, source_id=0):
        sink, bus = master.sink, master.bus
        if (len(sink.count) != 8 or len(sink.we) != 1
                or len(bus.dat_w) != 32 or len(bus.sel) != 4):
            raise ValueError("Etherbone master layout changed; review record metadata")
        _record_signals(self, source_id)
        self.bus = bus
        index = Signal(9)
        started = Signal()
        count = Signal(9)
        write = Signal()
        poisoned = Signal()
        active = bus.cyc & bus.stb
        accepted = sink.valid & sink.ready
        malformed = Signal()
        self.comb += [
            self.record_first.eq(active & (index == 0)),
            self.record_last.eq(active & sink.last),
            self.record_count.eq(Mux(started, count, sink.count)),
            malformed.eq(active & ((sink.count == 0)
                | (started & ((sink.count != count) | (sink.we != write)))
                | (sink.last != (index + 1 == sink.count)))),
            self.record_abort.eq(self.command_reset | self.guard_fault
                                 | poisoned | malformed),
        ]
        self.sync += [
            If(active & ~started,
                started.eq(1), count.eq(sink.count), write.eq(sink.we)),
            If(self.guard_fault | malformed, poisoned.eq(1)),
            If(accepted,
                If(sink.last,
                    index.eq(0), started.eq(0), poisoned.eq(0),
                ).Else(index.eq(index + 1)),
            ),
            If(self.command_reset,
                index.eq(0), started.eq(0), poisoned.eq(0)),
        ]


# The UART command state machine below is derived from LiteX Stream2Wishbone
# at b6ae9e0b227354aecffef5339d3e946f2395ac09, litex/soc/cores/uart.py.
# Copyright (c) 2014 Yann Sionneau <ys@m-labs.hk>
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2015 Sebastien Bourdeauducq <sb@m-labs.hk>
# Copyright (c) 2018 Tim 'mithro' Ansell <me@mith.ro>
# SPDX-License-Identifier: BSD-2-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


class _DspWaitTimer(Module):
    """Same wait/done cycle contract as WaitTimer, with an elapsed counter."""
    def __init__(self, cycles):
        self.wait, self.done = Signal(), Signal()
        count, result = Signal(max=cycles+1), Signal(48)
        if len(count) > 48:
            raise ValueError("native UART timeout must fit 48 bits")
        self.specials += Instance("EFX_DSP48",
            p_MODE="NORMAL", p_SIGNED=0, p_M_SEL="C", p_N_SEL="W",
            p_W_SEL="X", p_W_REG=1, p_RST_SYNC=1,
            i_A=Constant(0,19), i_B=Constant(0,18), i_C=Constant(1,18),
            i_CASCIN=Constant(0,48), i_OP=Constant(0,2), i_SHIFT_ENA=0,
            i_CLK=ClockSignal(), i_CE=self.wait & ~self.done,
            i_RST=ResetSignal() | ~self.wait, o_O=result)
        self.comb += count.eq(result[:len(count)])
        # Predict the same terminal edge as the elapsed counter. Keeping done
        # registered removes its wide comparison from the live bus/reset path.
        # Global reset clears this register together with the DSP accumulator.
        self.sync += If(~self.wait,
            self.done.eq(0),
        ).Else(self.done.eq(self.done | (count == cycles - 1)))


class RecordUARTBone(LiteXModule):
    """UARTBone's 32-bit protocol with command boundaries and timeout aborts.

    The existing USB CDC endpoints and byte order are unchanged. A command is
    one opcode, a byte count (zero encodes 256 words), a big-endian word address, and
    write data or read responses. Fixed/incrementing reads/writes are retained.
    The original absolute 100 ms command timer covers input, bus, and response
    stalls. The decoded count is explicitly nine bits so the zero encoding
    agrees in Migen simulation and generated Verilog.

    guard_fault poisons the whole remaining command without resetting byte
    parsing. command_reset aborts immediately and discards parser state; the
    caller must also discard stale transport bytes when resetting a link.
    record_abort is source-qualified out of band, including between words.
    timeout_cycles is an optional simulation override for the 100 ms timer.
    compact_datapath predecodes opcodes and reuses read-data and word-count
    arithmetic. use_dsp_timeout selects the Titanium native elapsed timer.
    Both are opt-in; neither changes valid stream bytes or command timing.
    """

    def __init__(self, phy=None, clk_freq=None, cd="sys", address_width=32,
                 *, source_id=1, timeout_cycles=None, compact_datapath=False, use_dsp_timeout=False):
        if address_width not in (8, 16, 32, 64):
            raise ValueError("Unsupported UARTBone address-width")
        if timeout_cycles is None:
            if clk_freq is None or clk_freq <= 0:
                raise ValueError("clk_freq must be positive")
            timeout_cycles = int(100e-3 * clk_freq)
        if not isinstance(timeout_cycles, int) or timeout_cycles < 1:
            raise ValueError("timeout_cycles must be positive")
        if cd != "sys" and phy is None:
            raise ValueError("a non-sys UART clock domain requires a PHY")
        if phy is not None:
            self.phy = phy if cd == "sys" else ClockDomainsRenamer(cd)(phy)
        if cd == "sys" and phy is not None:
            self.sink, self.source = phy.source, phy.sink
        else:
            self.sink = stream.Endpoint([("data", 8)])
            self.source = stream.Endpoint([("data", 8)])
            if cd != "sys":
                self.tx_cdc = stream.ClockDomainCrossing(
                    [("data", 8)], cd_from="sys", cd_to=cd)
                self.rx_cdc = stream.ClockDomainCrossing(
                    [("data", 8)], cd_from=cd, cd_to="sys")
                self.comb += [self.phy.source.connect(self.rx_cdc.sink),
                              self.tx_cdc.source.connect(self.phy.sink),
                              self.rx_cdc.source.connect(self.sink),
                              self.source.connect(self.tx_cdc.sink)]
        sink, source = self.sink, self.source
        self.wishbone = bus = wishbone.Interface(
            data_width=32, address_width=address_width, addressing="word")
        _record_signals(self, source_id)

        # The three opcode predicates retain all four operations and reject every
        # other byte after the same address-collection phase as the generic core.
        cmd = Signal(3 if compact_datapath else 8, reset_less=True)
        incr = Signal()
        length = Signal(9, reset_less=True)
        address = Signal(address_width, reset_less=True)
        data = Signal(32, reset_less=True)
        data_bytes_count = Signal(2, reset_less=True)
        addr_bytes_count = Signal(max=max(2, address_width // 8), reset_less=True)
        words_count = Signal(9, reset_less=True)
        poisoned = Signal()
        data_bytes_done = data_bytes_count == 3
        addr_bytes_done = addr_bytes_count == address_width // 8 - 1
        if compact_datapath:
            # Legal commands have length 1..256 and completed words 0..256.
            # Widen before incrementing, so this equality cannot wrap at 511.
            next_words_count = Signal(10)
            self.comb += next_words_count.eq(words_count + 1)
            words_done = next_words_count == length
        else:
            next_words_count = words_count + 1
            words_done = words_count == length - 1

        self.fsm = fsm = ResetInserter()(FSM(reset_state="RECEIVE-CMD"))
        self.timer = timer = (_DspWaitTimer(timeout_cycles) if use_dsp_timeout else WaitTimer(timeout_cycles))
        parser_reset = timer.done | self.command_reset
        self.comb += [
            timer.wait.eq(~fsm.ongoing("RECEIVE-CMD")),
            fsm.reset.eq(parser_reset),
            bus.adr.eq(address), bus.dat_w.eq(data), bus.sel.eq(0xf),
            self.record_first.eq(bus.cyc & bus.stb & (words_count == 0)),
            self.record_last.eq(bus.cyc & bus.stb & words_done),
            self.record_count.eq(length),
            self.record_abort.eq(parser_reset | self.guard_fault | poisoned),
        ]
        self.sync += [
            If(self.guard_fault, poisoned.eq(1)),
            If(fsm.ongoing("RECEIVE-CMD") & sink.valid & sink.ready,
                poisoned.eq(0)),
            If(parser_reset, poisoned.eq(0)),
        ]
        fsm.act("RECEIVE-CMD",
            sink.ready.eq(~parser_reset),
            NextValue(data_bytes_count, 0), NextValue(addr_bytes_count, 0),
            NextValue(words_count, 0),
            If(sink.valid,
                NextValue(cmd, Cat((sink.data == CMD_WRITE_BURST_INCR) | (sink.data == CMD_WRITE_BURST_FIXED),
                    (sink.data == CMD_READ_BURST_INCR) | (sink.data == CMD_READ_BURST_FIXED),
                    (sink.data == CMD_WRITE_BURST_INCR) | (sink.data == CMD_READ_BURST_INCR)) if compact_datapath else sink.data), NextState("RECEIVE-LENGTH")),
        )
        fsm.act("RECEIVE-LENGTH",
            sink.ready.eq(~parser_reset),
            If(sink.valid,
                NextValue(length, Mux(sink.data == 0, 256, sink.data)),
                NextState("RECEIVE-ADDRESS")),
        )
        fsm.act("RECEIVE-ADDRESS",
            sink.ready.eq(~parser_reset),
            If(sink.valid,
                NextValue(address, Cat(sink.data, address)),
                NextValue(addr_bytes_count, addr_bytes_count + 1),
                If(addr_bytes_done,
                    If(cmd[0] if compact_datapath else ((cmd == CMD_WRITE_BURST_INCR) | (cmd == CMD_WRITE_BURST_FIXED)),
                        NextValue(incr, cmd[2] if compact_datapath else cmd == CMD_WRITE_BURST_INCR),
                        NextState("RECEIVE-DATA"),
                    ).Elif(cmd[1] if compact_datapath else ((cmd == CMD_READ_BURST_INCR) | (cmd == CMD_READ_BURST_FIXED)),
                        NextValue(incr, cmd[2] if compact_datapath else cmd == CMD_READ_BURST_INCR),
                        NextState("READ-DATA"),
                    ).Else(NextState("RECEIVE-CMD")))),
        )
        fsm.act("RECEIVE-DATA",
            sink.ready.eq(~parser_reset),
            If(sink.valid,
                NextValue(data, Cat(sink.data, data)),
                NextValue(data_bytes_count, data_bytes_count + 1),
                If(data_bytes_done, NextState("WRITE-DATA"))),
        )
        fsm.act("WRITE-DATA",
            bus.stb.eq(~parser_reset), bus.we.eq(1), bus.cyc.eq(~parser_reset),
            If(bus.ack,
                NextValue(words_count, next_words_count),
                NextValue(address, address + incr),
                If(words_done, NextState("RECEIVE-CMD"))
                .Else(NextState("RECEIVE-DATA"))),
        )
        fsm.act("READ-DATA",
            bus.stb.eq(~parser_reset), bus.cyc.eq(~parser_reset),
            If(bus.ack,
                NextValue(data, bus.dat_r), NextState("SEND-DATA")),
        )
        if compact_datapath:
            self.comb += source.data.eq(data[24:32])
        else:
            self.comb += Case(data_bytes_count, {
                i: source.data.eq(data[8*n:8*(n+1)])
                for i, n in enumerate(reversed(range(4)))})
        fsm.act("SEND-DATA",
            source.valid.eq(~parser_reset),
            If(source.ready,
                # A read word is dead after byte four. A following write shifts
                # in four fresh bytes before exposing dat_w to Wishbone.
                *([NextValue(data, Cat(Constant(0, 8), data[:24]))] if compact_datapath else []),
                NextValue(data_bytes_count, data_bytes_count + 1),
                If(data_bytes_done,
                    NextValue(words_count, next_words_count),
                    NextValue(address, address + incr),
                    If(words_done, NextState("RECEIVE-CMD"))
                    .Else(NextState("READ-DATA")))),
        )
        self.comb += source.last.eq(data_bytes_done & words_done)
        if hasattr(source, "length"):
            self.comb += source.length.eq(4 * length)
