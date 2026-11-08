"""Actual LiteEth Etherbone packet/record/Wishbone paths for 128-word OTA I/O.

The only modeled boundaries are a 32-bit UDP payload port and a delayed
Wishbone memory. Packet encoding/decoding uses the independent LiteX host codec;
all packet FIFOs, record counts, byte swapping, and bus sequencing are upstream
LiteEth modules used by the board.
"""

from types import SimpleNamespace

from migen import Memory, Module, Signal
from migen.sim import passive, run_simulation
from litex.soc.interconnect import stream
from litex.tools.remote.etherbone import (
    EtherbonePacket, EtherboneReads, EtherboneRecord, EtherboneWrites,
)
from liteeth.common import eth_udp_user_description
from liteeth.frontend.etherbone import LiteEthEtherbone


WORDS = 128
FIFO_DEPTH = 256
BASE = 0x30025100
HOST_IP = 0xC0A801FB
UDP_PORT = 1234


class _UDPBoundary:
    dw = 32

    def __init__(self):
        self.port = SimpleNamespace(
            sink=stream.Endpoint(eth_udp_user_description(32)),
            source=stream.Endpoint(eth_udp_user_description(32)))

    def get_port(self, port, dw, cd, **kwargs):
        assert (port, dw, cd, kwargs) == (UDP_PORT, 32, "sys", {})
        return self.port


class _Frontend(Module):
    def __init__(self):
        self.udp = _UDPBoundary()
        self.submodules.etherbone = LiteEthEtherbone(
            SimpleNamespace(crossbar=self.udp), UDP_PORT, buffer_depth=FIFO_DEPTH)


def _encode(*, writes=None, reads=None, return_address=0):
    record = EtherboneRecord()
    if writes is not None:
        record.writes = EtherboneWrites(base_addr=BASE, datas=writes)
    if reads is not None:
        record.reads = EtherboneReads(base_ret_addr=return_address, addrs=reads)
    packet = EtherbonePacket()
    packet.records = [record]
    packet.encode()
    return bytes(packet.bytes)


def test_128_word_read_write_records_with_bus_and_packet_backpressure():
    dut = _Frontend()
    frontend = dut.etherbone
    bus = frontend.wishbone.bus
    incoming, outgoing = dut.udp.port.source, dut.udp.port.sink
    memory, transactions, responses = {}, [], []
    accepted_packets, completed_requests = [], []
    receiver_items, sender_items = [], []
    flow = {"allow_tx": False, "cycle": 0, "bus_stalls": 0, "tx_stalls": 0}

    # The production helper must preserve the real buffered packet FIFO shape,
    # including space for the base-address word before the 128 data words.
    for side, width in (("receiver", 42), ("sender", 70)):
        fifo = getattr(frontend.record, side).fifo.payload_fifo.fifo.fifo
        memories = [item for item in fifo._fragment.specials if isinstance(item, Memory)]
        assert len(memories) == 1
        assert (memories[0].width, memories[0].depth) == (width, FIFO_DEPTH)
        assert ("syn_ramstyle", "block_ram") in memories[0].attr
        assert not any(port.async_read for port in memories[0].ports)

    @passive
    def wishbone_memory():
        while True:
            yield bus.ack.eq(0)
            if not ((yield bus.cyc) and (yield bus.stb)):
                yield
                continue
            request = ((yield bus.we), (yield bus.adr), (yield bus.dat_w), (yield bus.sel))
            operation = len(transactions)
            # This also stalls at address/packet boundaries and across repeated
            # FIFO pointer wraps, rather than only delaying the first request.
            delay = 23 if operation % 31 == 0 else (operation * 7 + 2) % 6
            for _ in range(delay):
                assert (yield bus.cyc) and (yield bus.stb)
                assert ((yield bus.we), (yield bus.adr), (yield bus.dat_w), (yield bus.sel)) == request
                flow["bus_stalls"] += 1
                yield
            write, address, data, select = request
            assert select == 15
            # The record receiver must have received LAST before allowing any
            # Wishbone side effects from this whole-packet-buffered record.
            assert len(accepted_packets) > operation // WORDS
            if write:
                memory[address] = data
            else:
                assert address in memory, f"unexpected read address {address << 2:#x}"
                yield bus.dat_r.eq(memory[address])
            transactions.append(request)
            yield bus.ack.eq(1)
            yield
            yield bus.ack.eq(0)
            yield

    @passive
    def response_receiver():
        payload = bytearray()
        metadata = None
        held = None
        while True:
            cycle = flow["cycle"]
            flow["cycle"] += 1
            ready = flow["allow_tx"] and cycle % 13 not in (0, 1, 2, 6, 7)
            yield outgoing.ready.eq(ready)
            yield
            if not (yield outgoing.valid):
                continue
            beat = ((yield outgoing.data), (yield outgoing.last), (yield outgoing.last_be),
                    (yield outgoing.length), (yield outgoing.src_port),
                    (yield outgoing.dst_port), (yield outgoing.ip_address))
            if held is not None:
                assert beat == held, "Etherbone TX changed a stalled beat"
            if not (yield outgoing.ready):
                held = beat
                flow["tx_stalls"] += 1
                continue
            held = None
            data, last, last_be, length, src_port, dst_port, ip_address = beat
            if metadata is None:
                metadata = (length, src_port, dst_port, ip_address)
                # TX waits until the complete 128-word read result is present;
                # response backpressure must not prevent those reads finishing.
                expected_transactions = (len(responses) + 1) * WORDS * 2
                assert len(transactions) == expected_transactions
            assert metadata == (length, src_port, dst_port, ip_address)
            payload.extend(data.to_bytes(4, "little"))
            if last:
                assert last_be == 8
                assert len(payload) == length == 528
                responses.append((bytes(payload), metadata))
                payload = bytearray()
                metadata = None

    @passive
    def record_monitor():
        while True:
            for endpoint, captured in ((frontend.record.receiver.source, receiver_items),
                                       (frontend.record.sender.sink, sender_items)):
                if (yield endpoint.valid) and (yield endpoint.ready):
                    captured.append(((yield endpoint.we), (yield endpoint.count),
                                     (yield endpoint.addr), (yield endpoint.data),
                                     (yield endpoint.last)))
            yield

    def send_packet(payload, source_port):
        assert len(payload) == 528
        for offset in range(0, len(payload), 4):
            # Input bubbles exercise packetizer/depacketizer stalls while each
            # valid word remains stable until the upstream FIFO accepts it.
            if offset // 4 % 17 == 8:
                yield incoming.valid.eq(0)
                for _ in range(3):
                    yield
            yield incoming.data.eq(int.from_bytes(payload[offset:offset + 4], "little"))
            yield incoming.valid.eq(1)
            yield incoming.first.eq(offset == 0)
            yield incoming.last.eq(offset == len(payload) - 4)
            yield incoming.last_be.eq(8 if offset == len(payload) - 4 else 0)
            yield incoming.length.eq(len(payload))
            yield incoming.ip_address.eq(HOST_IP)
            yield incoming.src_port.eq(source_port)
            yield incoming.dst_port.eq(UDP_PORT)
            yield
            for _ in range(400):
                if (yield incoming.ready):
                    break
                yield
            else:
                raise AssertionError("128-word request cannot fit in the RX packet FIFO")
        accepted_packets.append(payload)
        yield incoming.valid.eq(0)
        yield incoming.last.eq(0)
        yield

    def wait_for(predicate, message, limit=10000):
        for _ in range(limit):
            if predicate():
                return
            yield
        raise AssertionError(message)

    def bench():
        for round_number in range(2):
            # Two write/read rounds wrap both payload FIFO pointers and replace
            # all data, exposing stale entries and off-by-one count handling.
            data = [((i * 0x1020307) ^ (0x89ABCDEF + round_number * 0x1234567)) & 0xFFFFFFFF
                    for i in range(WORDS)]
            source_port = 40000 + round_number
            yield from send_packet(_encode(writes=data), source_port)
            written_count = (round_number * 2 + 1) * WORDS
            yield from wait_for(lambda: len(transactions) == written_count,
                                "128-word write record did not complete")
            assert len(responses) == round_number, "write-only record generated a reply"
            written = transactions[round_number * 2 * WORDS:written_count]
            assert written == [(1, BASE // 4 + i, value, 15) for i, value in enumerate(data)]
            order = [(i * 37 + 11) % WORDS for i in range(WORDS)]
            addresses = [BASE + 4 * index for index in order]
            return_address = 0x12345670 + 4 * round_number
            flow["allow_tx"] = False
            yield from send_packet(_encode(reads=addresses, return_address=return_address), source_port)
            complete_count = (round_number + 1) * 2 * WORDS
            yield from wait_for(lambda: len(transactions) == complete_count,
                                "128-word read deadlocked with output backpressure")
            # Keep the consumer blocked after the entire record has filled.
            for _ in range(80):
                assert len(transactions) == complete_count
                assert len(responses) == round_number
                yield
            flow["allow_tx"] = True
            yield from wait_for(lambda: len(responses) == round_number + 1,
                                "128-word response was truncated or deadlocked")
            payload, metadata = responses[-1]
            assert metadata == (528, UDP_PORT, source_port, HOST_IP)
            packet = EtherbonePacket(init=payload)
            packet.decode()
            assert (packet.magic, packet.version, packet.pf, packet.pr) == (0x4E6F, 1, 0, 0)
            assert len(packet.records) == 1
            record = packet.records[0]
            assert (record.wcount, record.rcount, record.byte_enable) == (WORDS, 0, 15)
            assert record.writes.base_addr == return_address
            assert record.writes.get_datas() == [data[index] for index in order]
            assert [request[1] * 4 for request in transactions[written_count:complete_count]] == addresses
            assert all(not request[0] for request in transactions[written_count:complete_count])
            completed_requests.append((data, order))
        for _ in range(40):
            yield
        assert len(transactions) == 4 * WORDS
        assert len(responses) == 2
        assert len(receiver_items) == 4 * WORDS
        assert len(sender_items) == 2 * WORDS
        for captured, record_count in ((receiver_items, 4), (sender_items, 2)):
            for index in range(record_count):
                record = captured[index * WORDS:(index + 1) * WORDS]
                assert all(item[1] == WORDS for item in record)
                assert [item[4] for item in record] == [0] * (WORDS - 1) + [1]
        assert flow["bus_stalls"] > 1000
        assert flow["tx_stalls"] > 100
        assert len(accepted_packets) == 4
        assert len(completed_requests) == 2

    fragment = dut.get_fragment()
    # The pinned Migen simulator assumes every memory port has a read-data
    # signal. LiteX correctly creates write-only FIFO ports without one. Add an
    # unconnected simulation read result; FIFO storage/control is unchanged.
    for special in fragment.specials:
        if isinstance(special, Memory):
            for port in special.ports:
                if port.dat_r is None:
                    port.dat_r = Signal(special.width)
    run_simulation(fragment, [bench(), wishbone_memory(), response_receiver(), record_monitor()])
