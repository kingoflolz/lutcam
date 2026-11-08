"""Exact-cycle byte CRC compatibility with the upstream implementation.

The retained test is deliberately small. LiteEth is the cycle/sideband reference;
binascii.crc32 independently checks complete wire CRCs. Reset cases interrupt
COPY and each of the four emitted CRC bytes with a single active reset edge.
"""

import binascii
import random
import unittest

from migen import Module, ResetInserter
from migen.sim import run_simulation
from liteeth.common import eth_phy_description
from liteeth.mac.crc import LiteEthMACCRC32Inserter

from liteeth.mac.crc_rom import ByteCrcRomInserter


class EthernetCrcTests(unittest.TestCase):
    def test_exact_cycles_crc_vectors_stalls_and_dirty_reset_recovery(self):
        dut = Module()
        dut.submodules.old = old = ResetInserter()(LiteEthMACCRC32Inserter(eth_phy_description(8)))
        dut.submodules.new = new = ResetInserter()(ByteCrcRomInserter(eth_phy_description(8)))
        rng = random.Random(0xC32)
        stats = dict(cycles=0, stalls=0, bubbles=0, frames=0, resets=0)
        reset_positions = set()

        def step(*, valid=0, data=0, first=0, last=0, last_be=1, error=0, ready=1, reset=0):
            for part in (old, new):
                yield part.reset.eq(reset)
                yield part.sink.valid.eq(valid)
                yield part.sink.data.eq(data)
                yield part.sink.first.eq(first)
                yield part.sink.last.eq(last)
                yield part.sink.last_be.eq(last_be)
                yield part.sink.error.eq(error)
                yield part.source.ready.eq(ready)
            yield
            stats["cycles"] += 1
            if reset:
                stats["resets"] += 1
                return False, None
            self.assertEqual((yield old.sink.ready), (yield new.sink.ready))
            self.assertEqual((yield old.source.valid), (yield new.source.valid))
            emitted = None
            if (yield old.source.valid):
                for field in ("data", "first", "last", "last_be", "error"):
                    self.assertEqual((yield getattr(old.source, field)),
                                     (yield getattr(new.source, field)),
                                     (stats["cycles"], field))
                if ready:
                    emitted = ((yield old.source.data), (yield old.source.last))
                else:
                    stats["stalls"] += 1
            if not valid:
                stats["bubbles"] += 1
            return bool(valid and (yield old.sink.ready)), emitted

        def frame(payload, *, last_be=1):
            position = 0
            driving = False
            received = bytearray()
            for _ in range(8 * len(payload) + 200):
                if not driving and position < len(payload):
                    driving = rng.randrange(4) != 0
                ready = rng.randrange(4) != 0 and stats["cycles"] % 31 >= 4
                accepted, emitted = yield from step(valid=int(driving),
                    data=payload[position] if position < len(payload) else 0,
                    first=int(position == 0), last=int(position == len(payload) - 1),
                    last_be=last_be, error=int(position % 11 == 7), ready=ready)
                if accepted:
                    position += 1
                    driving = False
                if emitted is not None:
                    received.append(emitted[0])
                    if emitted[1]:
                        break
            else:
                self.fail("CRC frame did not finish")
            self.assertEqual(position, len(payload))
            expected_crc = binascii.crc32(payload).to_bytes(4, "little") if last_be else bytes(4)
            self.assertEqual(bytes(received), payload + expected_crc)
            stats["frames"] += 1

        def abort_at(position):
            # Every case dirties the CRC with nonzero data before aborting.
            payload = b"\x39\xa6\x51\xe7"
            count = 0
            target = 2 if position == "COPY" else len(payload)
            for _ in range(20):
                accepted, _ = yield from step(valid=1, data=payload[count],
                    first=int(count == 0), last=int(count == len(payload) - 1))
                if accepted:
                    count += 1
                if count == target:
                    break
            self.assertEqual(count, target)
            if position == "COPY":
                yield from step(valid=1, data=payload[count], ready=0)
                self.assertEqual((yield old.fsm.state), old.fsm.encoding["COPY"])
            else:
                # Hold the first CRC byte, accept the requested preceding
                # bytes, then hold exactly the byte to be interrupted.
                yield from step(ready=0)
                for _ in range(position):
                    _, emitted = yield from step(ready=1)
                    self.assertIsNotNone(emitted)
                yield from step(ready=0)
                self.assertEqual((yield old.fsm.state), old.fsm.encoding["CRC"])
            # Live input flags deliberately remain asserted on the abort edge.
            yield from step(reset=1, valid=1, data=255, first=1, last=1, error=1, ready=0)
            reset_positions.add(position)
            yield from frame(b"123456789")

        def bench():
            self.assertEqual(binascii.crc32(b"123456789"), 0xCBF43926)
            yield from step(reset=1)
            for payload in (b"\0", b"123456789", bytes(range(60)),
                            bytes(rng.randrange(256) for _ in range(1518))):
                yield from frame(payload)
            # Upstream emits four zero CRC bytes when final last_be is clear.
            yield from frame(b"\x82\x17\x93", last_be=0)
            for position in ("COPY", 0, 1, 2, 3):
                yield from abort_at(position)
            self.assertEqual(reset_positions, {"COPY", 0, 1, 2, 3})
            self.assertEqual(stats["frames"], 10)
            self.assertEqual(stats["resets"], 6)
            self.assertGreater(stats["stalls"], 100)
            self.assertGreater(stats["bubbles"], 100)
        run_simulation(dut, bench())


if __name__ == "__main__":
    unittest.main()
