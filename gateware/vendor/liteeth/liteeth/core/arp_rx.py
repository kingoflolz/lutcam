"""Compact fixed-address ARP receiver.

Derived from LiteEth arp.py, copyright 2015-2023 Florent Kermarrec.
SPDX-License-Identifier: BSD-2-Clause.
"""
import ipaddress
from migen import Case, FSM, If, NextState, NextValue, Signal
from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from liteeth.common import eth_mac_description, arp_header
from liteeth.core.arp import _arp_table_layout


def constant_ip(value):
    if isinstance(value, str):
        value = int(ipaddress.IPv4Address(value))
    if type(value) is not int or not 0 <= value <= 0xffffffff:
        raise TypeError('Compact ARP RX requires a constant IPv4 address')
    return value


class CompactARPRX(LiteXModule):
    """Validate header words as received; keep sender addresses and opcode flags.

    Retain the pinned RX's timing: one first-payload wait cycle, a one-cycle
    response independent of source.ready, then drain through packet.last.
    Unknown opcodes still produce valid with both request/reply flags zero.
    """
    def __init__(self, ip_address, dw=16, interface=None):
        ip_address = constant_ip(ip_address)
        if dw not in (8, 16, 32):
            raise ValueError('Compact ARP RX supports 8, 16 or 32 bits')
        expected_fields = {'hwtype': (0,16), 'proto': (2,16), 'hwsize': (4,8),
            'protosize': (5,8), 'opcode': (6,16), 'sender_mac': (8,48),
            'sender_ip': (14,32), 'target_mac': (18,48), 'target_ip': (24,32)}
        if (arp_header.length != 28 or not arp_header.swap_field_bytes
                or {name: (field.byte, field.width) for name, field in arp_header.fields.items()} != expected_fields
                or any(field.offset != 0 for field in arp_header.fields.values())):
            raise ValueError('ARP header changed')
        self.sink = sink = stream.Endpoint(eth_mac_description(dw)) if interface is None else interface.sink
        self.source = source = stream.Endpoint(_arp_table_layout) if interface is None else interface.source
        source.ip_address.reset_less = True
        source.mac_address.reset_less = True
        bytes_per_word = dw // 8
        words = 28 // bytes_per_word
        count = Signal(max=words)
        bad = Signal(reset_less=True)
        request_opcode = Signal(reset_less=True)
        reply_opcode = Signal(reset_less=True)
        capture_first = Signal()
        capture_next = Signal()

        validation = dict(hwtype=1, proto=0x800, hwsize=6, protosize=4, target_ip=ip_address)
        masks, expected = [0]*words, [0]*words
        for name, value in validation.items():
            field = arp_header.fields[name]
            encoded = value.to_bytes(field.width//8, 'big')
            for offset, byte in enumerate(encoded):
                address = field.byte + offset
                word, lane = divmod(address, bytes_per_word)
                masks[word] |= 255 << (lane*8)
                expected[word] |= byte << (lane*8)
        first_mismatch = (sink.data & masks[0]) != expected[0]
        updates = {i: bad.eq(bad | ((sink.data & masks[i]) != expected[i]))
                   for i in range(1, words) if masks[i]}
        self.sync += If(capture_first,
            bad.eq(first_mismatch)
        ).Elif(capture_next, Case(count, updates))

        # Capture sender fields directly in network-order output Signals.
        for name, output in [('sender_mac', source.mac_address), ('sender_ip', source.ip_address)]:
            field = arp_header.fields[name]
            for byte in range(field.width//8):
                word, lane = divmod(field.byte + byte, bytes_per_word)
                low = field.width - 8*(byte + 1)
                enable = capture_first if word == 0 else capture_next & (count == word)
                self.sync += If(enable, output[low:low+8].eq(sink.data[lane*8:(lane+1)*8]))

        opcode = arp_header.fields['opcode']
        first_word = opcode.byte // bytes_per_word
        last_word = (opcode.byte + 1) // bytes_per_word
        for word in range(first_word, last_word + 1):
            mask = 0
            request_value = 0
            reply_value = 0
            for byte, (request_byte, reply_byte) in enumerate(zip(b'\x00\x01', b'\x00\x02')):
                address = opcode.byte + byte
                index, lane = divmod(address, bytes_per_word)
                if index == word:
                    mask |= 255 << (lane*8)
                    request_value |= request_byte << (lane*8)
                    reply_value |= reply_byte << (lane*8)
            request_match = (sink.data & mask) == request_value
            reply_match = (sink.data & mask) == reply_value
            if word != first_word:
                request_match = request_opcode & request_match
                reply_match = reply_opcode & reply_match
            self.sync += If(capture_next & (count == word),
                request_opcode.eq(request_match), reply_opcode.eq(reply_match))

        self.fsm = fsm = FSM(reset_state='IDLE')
        fsm.act('IDLE',
            sink.ready.eq(1),
            NextValue(count, 1),
            If(sink.valid,
                capture_first.eq(1),
                NextState('HEADER'),
                If(sink.last, NextState('IDLE'))
            )
        )
        fsm.act('HEADER',
            sink.ready.eq(1),
            If(sink.valid,
                capture_next.eq(1),
                NextValue(count, count + 1),
                If(count == words - 1, NextState('WAIT-PAYLOAD')),
                If(sink.last, NextState('IDLE'))
            )
        )
        fsm.act('WAIT-PAYLOAD',
            If(sink.valid,
                NextState('CHECK')
            ).Else(sink.ready.eq(1))
        )
        fsm.act('CHECK',
            If(~bad,
                source.valid.eq(1),
                source.request.eq(request_opcode),
                source.reply.eq(reply_opcode)
            ),
            NextState('DRAIN')
        )
        fsm.act('DRAIN',
            sink.ready.eq(1),
            If(sink.valid & sink.last, NextState('IDLE'))
        )

