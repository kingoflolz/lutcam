"""Streaming record decoding behind a complete validated Ethernet frame."""
from migen import Cat, Constant, FSM, If, Module, NextState, NextValue, Signal
from litex.soc.interconnect import stream
from liteeth.common import eth_etherbone_mmap_description, eth_etherbone_record_description


class StreamingEtherboneRecordReceiver(Module):
    """Decode one already-complete, length-checked Etherbone record."""

    def __init__(self):
        self.sink = sink = stream.Endpoint(eth_etherbone_record_description(32))
        self.source = source = stream.Endpoint(eth_etherbone_mmap_description(32))
        self.length = Signal(16)  # Record header plus record payload, in bytes.
        self.invalid = Signal()
        base = Signal(32)
        count = Signal(8)
        writes = Signal(8)
        reads = Signal(8)
        byte_enable = Signal(4)
        words = Signal(10)
        expected_length = Signal(12)
        self.comb += [
            words.eq(Cat(sink.wcount, Constant(0, 2)) +
                     Cat(sink.rcount, Constant(0, 2)) + (sink.wcount != 0) + (sink.rcount != 0)),
            expected_length.eq((words << 2) + 4),
        ]
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += [source.be.eq(byte_enable), source.data.eq(sink.data),
                      source.base_addr.eq(base), source.last_be.eq(source.last << 3)]
        malformed = (self.length != expected_length) | (words == 0) | (sink.error != 0)
        fsm.act("IDLE",
            sink.ready.eq(1),
            If(sink.valid,
                NextValue(count, 0),
                If(malformed | sink.last,
                    self.invalid.eq(1),
                    If(~sink.last, NextState("DROP")),
                ).Else(
                    NextValue(base, sink.data), NextValue(writes, sink.wcount),
                    NextValue(reads, sink.rcount), NextValue(byte_enable, sink.byte_enable),
                    If(sink.wcount != 0, NextState("WRITES")).Else(NextState("READS")),
                ),
            ),
        )
        for state_name, writing in (("WRITES", True), ("READS", False)):
            amount = writes if writing else reads
            final_operation = count == amount - 1
            expected_last = final_operation & (reads == 0) if writing else final_operation
            fsm.act(state_name,
                source.we.eq(int(writing)), source.count.eq(amount), source.last.eq(final_operation),
                source.addr.eq(base[2:] + count if writing else sink.data[2:]),
                If(sink.valid,
                    If((sink.last != expected_last) | (sink.error != 0),
                        sink.ready.eq(1), self.invalid.eq(1),
                        If(sink.last, NextState("IDLE")).Else(NextState("DROP")),
                    ).Else(
                        source.valid.eq(1), sink.ready.eq(source.ready),
                        If(source.ready,
                            NextValue(count, count + 1),
                            If(final_operation,
                                If(int(writing) & (reads != 0), NextState("READ_BASE"))
                                .Else(NextState("IDLE")),
                            ),
                        ),
                    ),
                ),
            )
        fsm.act("READ_BASE",
            sink.ready.eq(1), NextValue(count, 0),
            If(sink.valid,
                If(sink.last | (sink.error != 0),
                    self.invalid.eq(1),
                    If(sink.last, NextState("IDLE")).Else(NextState("DROP")),
                ).Else(NextValue(base, sink.data), NextState("READS")),
            ),
        )
        fsm.act("DROP", sink.ready.eq(1), If(sink.valid & sink.last, NextState("IDLE")))

