"""Byte-wide Ethernet transmit CRC using a synchronous immutable table."""
from migen import Cat, Constant, FSM, If, Memory, Mux, NextState, NextValue, Signal
from migen.genlib.misc import chooser
from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from liteeth.mac.crc import crc_calc


class ByteCrcRom(LiteXModule):
    """One accepted byte per clock, with the CRC visible after that edge.

    For LiteEth's non-reflected internal CRC representation, advancing one
    byte is ``(state << 8) ^ table[data ^ reverse(state[31:24])]``. The ROM
    returns the table term while a 24-bit register retains the shifted term.
    A seed flag supplies the initial all-one state, so neither the ROM output
    nor the unqualified payload register requires reset.
    """

    def __init__(self):
        self.data = Signal(8)
        self.ce = Signal()
        self.reset = Signal()
        self.value = Signal(32)

        table = Memory(32, 256, init=[
            crc_calc(8, 32, 0x04c11db7, 0, index) for index in range(256)
        ])
        read = table.get_port(has_re=True)
        self.specials += table, read
        seed = Signal(reset=1)
        low = Signal(24, reset_less=True)
        state = Signal(32)
        self.comb += [
            state.eq(Mux(seed, 0xffffffff, read.dat_r ^ Cat(Constant(0, 8), low))),
            read.adr.eq(self.data ^ state[24:32][::-1]),
            read.re.eq(self.ce),
            self.value.eq(state[::-1] ^ 0xffffffff),
        ]
        self.sync += [
            If(self.reset, seed.eq(1)).Elif(self.ce, seed.eq(0)),
            If(self.ce, low.eq(state[:24])),
        ]


class ByteCrcRomInserter(LiteXModule):
    """Exact-cycle replacement for LiteEth's 8-bit transmit CRC inserter.

    The final byte's synchronous ROM result is available in the existing CRC
    emission state. Reading it there removes the separate 32-bit completed
    CRC register without adding a cycle or restricting backpressure.
    Endpoint arguments permit an instance-scoped MAC replacement to preserve
    its existing input buffer and all external endpoint objects.
    """

    def __init__(self, description, *, sink=None, source=None, crc_type=ByteCrcRom):
        self.sink = sink = stream.Endpoint(description) if sink is None else sink
        self.source = source = stream.Endpoint(description) if source is None else source
        if len(sink.data) != 8 or len(source.data) != 8:
            raise ValueError("The CRC ROM inserter requires byte-wide endpoints")
        self.submodules.crc = crc = crc_type()
        crc_packet = Signal(32)
        saved_be = Signal(reset_less=True)
        # Preserve LiteEth's four zero CRC bytes when the final last_be is zero.
        self.comb += [
            crc.data.eq(sink.data),
            crc_packet.eq(Mux(saved_be, crc.value, 0)),
        ]
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            crc.reset.eq(1),
            sink.ready.eq(1),
            If(sink.valid,
                sink.ready.eq(0),
                NextState("COPY"),
            ),
        )
        fsm.act("COPY",
            crc.ce.eq(sink.valid & source.ready),
            sink.connect(source),
            source.last.eq(0),
            source.last_be.eq(0),
            If(sink.valid & sink.last & source.ready,
                NextValue(saved_be, sink.last_be),
                NextState("CRC"),
            ),
        )
        count = Signal(2, reset=3)
        done = Signal()
        self.comb += done.eq(count == 0)
        fsm.act("CRC",
            source.valid.eq(1),
            chooser(crc_packet, count, source.data, reverse=True),
            If(done,
                source.last.eq(1),
                If(source.ready, NextState("IDLE")),
            ),
        )
        self.sync += If(fsm.ongoing("IDLE"),
            count.eq(3),
        ).Elif(fsm.ongoing("CRC") & ~done,
            count.eq(count - source.ready),
        )

