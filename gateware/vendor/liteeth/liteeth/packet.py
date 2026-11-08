"""Indexed aligned headers; unaligned headers retain LiteX's implementation.

Derived from LiteX packet.py, copyright 2015-2024 Florent Kermarrec and
copyright 2019 Vamsi K Vytla; SPDX-License-Identifier: BSD-2-Clause.
"""
from migen import Case, FSM, If, NextState, NextValue, Signal
from litex.gen import LiteXModule
from litex.soc.interconnect import stream, packet
original_packetizer_init = packet.Packetizer.__init__
original_depacketizer_init = packet.Depacketizer.__init__


class IndexedPacketizer(LiteXModule):
    def __init__(self, sink_description, source_description, header, interface=None):
        dw = dict(sink_description.payload_layout)["data"]
        if header.length % (dw // 8):
            if interface is not None:
                raise ValueError("In-place header replacement requires word alignment")
            original_packetizer_init(self, sink_description, source_description, header)
            return
        self.sink = sink = (stream.Endpoint(sink_description) if interface is None
                            else interface.sink)
        self.source = source = (stream.Endpoint(source_description) if interface is None
                                else interface.source)
        self.header = Signal(header.length * 8) if interface is None else interface.header
        words = header.length * 8 // dw
        if words == 0:
            raise ValueError("Header must contain at least one word")
        count = Signal(max=max(words, 2))
        self.comb += header.encode(sink, self.header)
        last_be_copy = []
        if hasattr(sink, "last_be") and hasattr(source, "last_be"):
            last_be_copy = [If(sink.last, source.last_be.eq(packet._last_be_normalize(sink.last_be)))]
        if hasattr(sink, "error") and hasattr(source, "error"):
            self.comb += source.error.eq(sink.error)
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            sink.ready.eq(1),
            NextValue(count, 1),
            If(sink.valid,
                sink.ready.eq(0),
                source.valid.eq(1),
                source.first.eq(1),
                source.data.eq(self.header[:dw]),
                If(source.ready,
                    NextState("ALIGNED-DATA-COPY" if words == 1 else "HEADER-SEND")
                )
            )
        )
        fsm.act("HEADER-SEND",
            source.valid.eq(1),
            Case(count, {i: source.data.eq(self.header[i*dw:(i+1)*dw]) for i in range(1, words)}),
            If(source.ready,
                NextValue(count, count + 1),
                If(count == words - 1, NextState("ALIGNED-DATA-COPY"))
            )
        )
        fsm.act("ALIGNED-DATA-COPY",
            source.valid.eq(sink.valid),
            source.last.eq(sink.last),
            source.data.eq(sink.data),
            *last_be_copy,
            If(source.valid & source.ready,
                sink.ready.eq(1),
                If(source.last, NextState("IDLE"))
            )
        )


class IndexedDepacketizer(LiteXModule):
    def __init__(self, sink_description, source_description, header, interface=None):
        dw = dict(sink_description.payload_layout)["data"]
        if header.length % (dw // 8):
            if interface is not None:
                raise ValueError("In-place header replacement requires word alignment")
            original_depacketizer_init(self, sink_description, source_description, header)
            return
        self.sink = sink = (stream.Endpoint(sink_description) if interface is None
                            else interface.sink)
        self.source = source = (stream.Endpoint(source_description) if interface is None
                                else interface.source)
        self.header = Signal(header.length * 8, reset_less=True) if interface is None else interface.header
        # The old combinational header was driven by reset-less storage. It is
        # now that storage itself; preserve the original reset behavior.
        self.header.reset_less = True
        words = header.length * 8 // dw
        if words == 0:
            raise ValueError("Header must contain at least one word")
        count = Signal(max=max(words, 2))
        first = Signal()
        capture_first = Signal()
        capture_next = Signal()
        self.sync += If(capture_first, self.header[:dw].eq(sink.data))
        for i in range(1, words):
            self.sync += If(capture_next & (count == i), self.header[i*dw:(i+1)*dw].eq(sink.data))
        self.comb += header.decode(self.header, source)
        last_be_copy = []
        if hasattr(sink, "last_be") and hasattr(source, "last_be"):
            last_be_copy = [If(sink.last, source.last_be.eq(packet._last_be_normalize(sink.last_be)))]
        if hasattr(sink, "error") and hasattr(source, "error"):
            self.comb += source.error.eq(sink.error)
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            sink.ready.eq(1),
            NextValue(count, 1),
            If(sink.valid,
                capture_first.eq(1),
                NextValue(first, 1),
                NextState("ALIGNED-DATA-COPY" if words == 1 else "HEADER-RECEIVE"),
                If(sink.last, NextState("IDLE"))
            )
        )
        fsm.act("HEADER-RECEIVE",
            sink.ready.eq(1),
            If(sink.valid,
                capture_next.eq(1),
                NextValue(count, count + 1),
                If(count == words - 1,
                    NextValue(first, 1),
                    NextState("ALIGNED-DATA-COPY")
                ),
                If(sink.last, NextState("IDLE"))
            )
        )
        fsm.act("ALIGNED-DATA-COPY",
            source.valid.eq(sink.valid),
            source.first.eq(first),
            source.last.eq(sink.last),
            sink.ready.eq(source.ready),
            source.data.eq(sink.data),
            *last_be_copy,
            If(source.valid & source.ready,
                NextValue(first, 0),
                If(source.last, NextState("IDLE"))
            )
        )

