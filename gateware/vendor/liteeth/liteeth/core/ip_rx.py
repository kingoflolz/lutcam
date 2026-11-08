"""Streaming IPv4 receive validation for the 16-bit datapath.

Derived from LiteEth ip.py, copyright 2015-2023 Florent Kermarrec.
SPDX-License-Identifier: BSD-2-Clause.
"""
from migen import Cat, FSM, If, NextState, NextValue, Signal
from litex.gen import LiteXModule
from litex.soc.interconnect import packet, stream
from liteeth.common import eth_mac_description, eth_ipv4_user_description


class StreamingIPRX(LiteXModule):
    def __init__(self, ip_address, with_broadcast=True, dw=16, interface=None):
        if dw != 16:
            raise ValueError("Streaming IPv4 RX requires 16-bit words")
        self.sink = sink = stream.Endpoint(eth_mac_description(dw)) if interface is None else interface.sink
        self.source = source = stream.Endpoint(eth_ipv4_user_description(dw)) if interface is None else interface.source
        count = Signal(max=10)
        checksum = Signal(16, reset_less=True)
        summed = Signal(17)
        self.comb += summed.eq(checksum + sink.data)
        total_length = Signal(16, reset_less=True)
        protocol = Signal(8, reset_less=True)
        sender_ip = Signal(32, reset_less=True)
        version_ihl_ok = Signal(reset_less=True)
        fragment_ok = Signal(reset_less=True)
        if not with_broadcast:
            target_ip = Signal(32, reset_less=True)
        capture_first = Signal()
        capture_next = Signal()
        swapped_word = Cat(sink.data[8:16], sink.data[:8])
        self.sync += If(capture_first,
            checksum.eq(sink.data),
            version_ihl_ok.eq(sink.data[:8] == 0x45),
        ).Elif(capture_next, checksum.eq(summed[:16] + summed[16]))
        fields = {1: total_length.eq(swapped_word),
                  3: fragment_ok.eq((swapped_word & 0x3fff) == 0),
                  4: protocol.eq(sink.data[8:16]),
                  6: sender_ip[16:].eq(swapped_word),
                  7: sender_ip[:16].eq(swapped_word)}
        if not with_broadcast:
            fields.update({8: target_ip[16:].eq(swapped_word), 9: target_ip[:16].eq(swapped_word)})
        for index, assignment in fields.items():
            self.sync += If(capture_next & (count == index), assignment)
        payload_valid = Signal()
        payload_ready = Signal()
        self.parser = parser = FSM(reset_state="IDLE")
        parser.act("IDLE", sink.ready.eq(1), NextValue(count, 1),
            If(sink.valid, capture_first.eq(1), NextState("HEADER"),
               If(sink.last, NextState("IDLE"))))
        parser.act("HEADER", sink.ready.eq(1),
            If(sink.valid, capture_next.eq(1), NextValue(count, count + 1),
               If(count == 9, NextState("DATA")),
               If(sink.last, NextState("IDLE"))))
        parser.act("DATA", payload_valid.eq(sink.valid), sink.ready.eq(payload_ready),
            source.data.eq(sink.data), source.last.eq(sink.last),
            If(sink.last, source.last_be.eq(packet._last_be_normalize(sink.last_be))),
            If(sink.valid & payload_ready & sink.last, NextState("IDLE")))
        # The old checker takes eleven consecutive payload-valid clocks. Keep
        # the original handshake latency and restart when payload valid drops.
        delay = Signal(max=12)
        self.sync += If(~payload_valid, delay.eq(0)).Elif(delay != 11, delay.eq(delay + 1))
        acceptable = version_ihl_ok & fragment_ok & (checksum == 0xffff)
        if not with_broadcast:
            # Compare at the original decision cycle, retaining dynamic-IP
            # behavior rather than capturing an early destination match.
            acceptable = acceptable & (target_ip == ip_address)
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE", If(payload_valid & (delay == 11),
            NextState("DROP"), If(acceptable, NextState("RECEIVE"))))
        fsm.act("RECEIVE", source.valid.eq(payload_valid), payload_ready.eq(source.ready),
            If(payload_valid & source.ready & sink.last, NextState("IDLE")))
        fsm.act("DROP", payload_ready.eq(1),
            If(payload_valid & sink.last, NextState("IDLE")))
        self.comb += [source.error.eq(sink.error), source.length.eq(total_length - 20),
                      source.ip_address.eq(sender_ip), source.protocol.eq(protocol)]

