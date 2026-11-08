"""Check arithmetic, exact latency, and the actual LiteEth RX replacement."""

from pathlib import Path
import random
import sys

import pytest
from migen import Module
from migen.sim import passive, run_simulation
from liteeth.core.ip import LiteEthIPRX, LiteEthIPV4Checksum

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))
from liteeth.core.checksum import IterativeIPV4Checksum


def _oracle(header, skip_checksum=False):
    data = bytearray(header.to_bytes(20, "little"))
    if skip_checksum:
        data[10:12] = b"\0\0"
    total = sum(int.from_bytes(data[i:i + 2], "big") for i in range(0, 20, 2))
    while total >> 16:
        total = (total & 0xffff) + (total >> 16)
    return total ^ 0xffff


@pytest.mark.parametrize("skip_checksum", [False, True])
def test_checksum_oracle_original_latency_ce_and_reset(skip_checksum):
    dut = Module()
    dut.submodules.old = old = LiteEthIPV4Checksum(skip_checksum=skip_checksum)
    dut.submodules.new = new = IterativeIPV4Checksum(skip_checksum=skip_checksum)
    dut.comb += [old.header.eq(new.header), old.ce.eq(new.ce), old.reset.eq(new.reset)]
    rng = random.Random(0xa54b)
    headers = [0, (1 << 160) - 1, int.from_bytes(bytes.fromhex(
        "45000073000040004011b861c0a80001c0a800c7"), "little")]
    headers += [sum((0xffff if i != j else 1) << (16 * i) for i in range(10))
                for j in range(10)]
    headers += [rng.getrandbits(160) for _ in range(500)]
    latency = 9 if skip_checksum else 11

    def bench():
        for index, header in enumerate(headers):
            # Restart a partial sum even when CE is low.
            yield new.reset.eq(1)
            yield new.ce.eq(index % 2)
            yield new.header.eq(header)
            yield
            yield new.reset.eq(0)
            count = 0
            for _ in range(80):
                ce = rng.randrange(3) != 0
                yield new.ce.eq(ce)
                yield
                yield new.ce.eq(0)
                yield
                if ce:
                    count = min(latency, count + 1)
                assert (yield old.done) == (yield new.done) == (count == latency)
                if count == latency:
                    expected = _oracle(header, skip_checksum)
                    assert (yield old.value) == expected
                    assert (yield new.value) == expected
                    break
            else:
                raise AssertionError("Checksum did not complete")
            for _ in range(3):
                yield new.header.eq(rng.getrandbits(160))
                yield new.ce.eq(rng.randrange(2))
                yield
                yield
                assert (yield new.done) == (yield old.done) == 1
                assert (yield new.value) == (yield old.value) == expected
            yield new.reset.eq(1)
            yield new.ce.eq(0)
            yield
            yield new.reset.eq(0)
            yield new.ce.eq(1)
            for _ in range(rng.randrange(1, 8)):
                yield

    run_simulation(dut, bench())


def _receiver():
    return LiteEthIPRX(0x024c55540001, 0xc0a80132, with_broadcast=False, dw=32)


def test_replaced_receiver_accepts_valid_drops_bad_and_recovers_with_backpressure():
    rx = _receiver()
    packets, expected = [], []
    for index, (length, good) in enumerate(((12, True), (28, False), (60, True),
                                          (16, False), (36, True))):
        payload = bytes((index * 53 + i * 17) & 255 for i in range(length))
        header = bytearray.fromhex("450000000000000080110000c0a80101c0a80132")
        header[2:4] = (20 + length).to_bytes(2, "big")
        header[4:6] = (index * 113).to_bytes(2, "big")
        checksum = _oracle(int.from_bytes(header, "little"))
        header[10:12] = (checksum ^ (0 if good else 1)).to_bytes(2, "big")
        packets.append(bytes(header) + payload)
        if good:
            expected.append(payload)
    received = []
    pending = bytearray()
    stalls = [0, 0]

    def transmit():
        yield rx.sink.valid.eq(0)
        for _ in range(3):
            yield
        for packet in packets:
            for offset in range(0, len(packet), 4):
                last = offset + 4 == len(packet)
                yield rx.sink.valid.eq(1)
                yield rx.sink.data.eq(int.from_bytes(packet[offset:offset + 4], "little"))
                yield rx.sink.last.eq(last)
                yield rx.sink.last_be.eq(8 if last else 0)
                yield
                for _ in range(200):
                    if (yield rx.sink.ready):
                        break
                    stalls[0] += 1
                    yield
                else:
                    raise AssertionError("RX failed to consume the packet")
            yield rx.sink.valid.eq(0)
            for _ in range(3):
                yield
        for _ in range(100):
            yield

    @passive
    def receive():
        cycle = 0
        while True:
            if (yield rx.source.valid):
                if (yield rx.source.ready):
                    pending.extend((yield rx.source.data).to_bytes(4, "little"))
                    assert (yield rx.source.protocol) == 17
                    assert (yield rx.source.ip_address) == 0xc0a80101
                    assert (yield rx.source.error) == 0
                    if (yield rx.source.last):
                        assert (yield rx.source.last_be) == 8
                        assert (yield rx.source.length) == len(pending)
                        received.append(bytes(pending))
                        pending.clear()
                else:
                    stalls[1] += 1
            yield rx.source.ready.eq(cycle % 7 >= 3)
            cycle += 1
            yield

    run_simulation(rx, [transmit(), receive()])
    assert received == expected
    assert not pending
    assert all(stalls)
