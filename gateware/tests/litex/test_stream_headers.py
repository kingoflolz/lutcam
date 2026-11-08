"""Compare indexed headers with independent upstream serialization."""

import random
import pytest
from migen import Module
from migen.sim import run_simulation
from litex.soc.interconnect import stream, packet
from liteeth.common import arp_header, ipv4_header, udp_header, mac_header, etherbone_packet_header, etherbone_record_header
from liteeth.packet import IndexedPacketizer, IndexedDepacketizer

HEADERS = [arp_header, ipv4_header, udp_header, mac_header, etherbone_packet_header, etherbone_record_header]

@pytest.mark.parametrize('direction', ['tx', 'rx'])
@pytest.mark.parametrize('dw', [8, 16, 32])
@pytest.mark.parametrize('with_be', [False, True])
@pytest.mark.parametrize('header', HEADERS, ids=['arp', 'ipv4', 'udp', 'mac', 'etherbone-packet', 'etherbone-record'])
def test_equivalent(header, dw, with_be, direction):
    payload = [('data', dw), ('error', dw // 8)]
    if with_be:
        payload += [('last_be', dw // 8)]
    raw = stream.EndpointDescription(payload_layout=payload)
    param = stream.EndpointDescription(payload_layout=payload, param_layout=header.get_layout())
    klass = packet.Packetizer if direction == 'tx' else packet.Depacketizer
    optimized = IndexedPacketizer if direction == 'tx' else IndexedDepacketizer
    descriptions = (param, raw) if direction == 'tx' else (raw, param)
    dut = Module()
    dut.submodules.old = old = klass(*descriptions, header)
    dut.submodules.new = new = optimized(*descriptions, header)
    rng = random.Random(291 + dw + header.length)
    packets = []
    for index in range(40):
        params = {name: rng.getrandbits(width) for name, width in header.get_layout()}
        size = rng.randrange(1, 25)
        if direction == 'rx':
            # Include every header truncation point and complete packets.
            size = index % (header.length * 8 // dw + 1) + 1 if index < header.length * 8 // dw else size + header.length * 8 // dw
        words = []
        for i in range(size):
            values = dict(data=rng.getrandbits(dw), error=rng.getrandbits(dw // 8), first=int(i == 0), last=int(i == size - 1))
            if with_be:
                values['last_be'] = (1 << (index % (dw // 8))) if index % 5 else 0
            if direction == 'tx':
                values.update(params)
            words.append(values)
        packets.append(words)
    transactions = [word for words in packets for word in words]
    stats = dict(accepted=0, emitted=0, stalled=0, bubbles=0)
    def bench():
        position = 0
        driving = False
        for cycle in range(30000):
            # The previous cycle's driven transaction remains until both copies accept it.
            old_ready = yield old.sink.ready
            new_ready = yield new.sink.ready
            assert old_ready == new_ready, (cycle, 'sink.ready', old_ready, new_ready)
            old_valid = yield old.source.valid
            new_valid = yield new.source.valid
            assert old_valid == new_valid, (cycle, 'source.valid', old_valid, new_valid)
            if old_valid:
                names = ['first', 'last', 'data', 'error'] + (['last_be'] if with_be else [])
                if direction == 'rx':
                    names += [name for name, _ in header.get_layout()]
                for name in names:
                    a = yield getattr(old.source, name)
                    b = yield getattr(new.source, name)
                    assert a == b, (cycle, name, a, b)
                if direction == 'rx':
                    a = yield old.header
                    b = yield new.header
                    assert a == b, (cycle, 'complete header', a, b)
                if (yield old.source.ready):
                    stats['emitted'] += 1
                else:
                    stats['stalled'] += 1
            if driving and old_ready:
                position += 1
                driving = False
                stats['accepted'] += 1
            if not driving and position < len(transactions):
                driving = rng.randrange(4) != 0
            if not driving:
                stats['bubbles'] += 1
            if position == len(transactions) and not old_valid:
                break
            ready = (cycle % 41 >= 11) and (rng.randrange(3) != 0)
            for part in [old, new]:
                yield part.source.ready.eq(ready)
                yield part.sink.valid.eq(driving)
                if position < len(transactions):
                    for name, value in transactions[position].items():
                        yield getattr(part.sink, name).eq(value)
            yield
        else:
            raise AssertionError('Simulation timed out')
        assert stats['accepted'] == len(transactions)
        assert stats['emitted'] > 100
        assert stats['stalled'] > 20
        assert stats['bubbles'] > 20
    run_simulation(dut, bench())
