"""Record metadata around the installed LiteEth Wishbone master.

The stream source models decoded, length-validated write/read sections. The
actual LiteEth master supplies all Wishbone and response-stream sequencing.
Malformed datagram framing is tested by the upstream length validator.
"""

import unittest

from migen import Module, ResetInserter
from migen.sim import passive, run_simulation
from liteeth.frontend.etherbone import LiteEthEtherboneWishboneMaster

from fpga.litex.lutcam.record_bus import EtherboneRecordMetadata


class _Fixture(Module):
    def __init__(self, source_id=2):
        self.submodules.master = ResetInserter()(LiteEthEtherboneWishboneMaster())
        self.submodules.metadata = EtherboneRecordMetadata(
            self.master, source_id=source_id)


class EtherboneRecordMetadataTests(unittest.TestCase):
    def _metadata(self, metadata):
        return ((yield metadata.record_first), (yield metadata.record_last),
                (yield metadata.record_count), (yield metadata.record_source),
                (yield metadata.record_abort))

    def _word(self, dut, *, count, index, write=True, last=None,
              expected_count=None, abort=False, guard=False, stalls=3,
              guard_at_ack=False, response_stalls=4, address=None, value=None):
        """Present one stream beat until the real master's handshake.

        ACK stalls and read-response stalls are independent: reads must retain
        their section index while SEND_DATA deasserts the Wishbone cycle.
        """
        master, metadata = dut.master, dut.metadata
        sink, bus = master.sink, master.bus
        if last is None:
            last = index == count - 1
        if expected_count is None:
            expected_count = count
        if address is None:
            address = 0x1000 + index
        if value is None:
            value = 0x12340000 + index
        yield sink.valid.eq(1)
        yield sink.we.eq(write)
        yield sink.count.eq(count)
        yield sink.last.eq(last)
        yield sink.addr.eq(address)
        yield sink.data.eq(value)
        yield sink.be.eq(0xf)
        yield sink.base_addr.eq(0x9000)
        yield master.source.ready.eq(0)
        yield bus.ack.eq(0)
        yield bus.dat_r.eq(value ^ 0xffffffff)
        yield metadata.guard_fault.eq(guard and not guard_at_ack)
        yield
        for _ in range(12):
            if (yield bus.cyc) and (yield bus.stb):
                break
            yield
        else:
            self.fail("LiteEth did not present a Wishbone request")

        expected = (int(index == 0), int(last), expected_count, 2,
                    int(abort or (guard and not guard_at_ack)))
        for _ in range(stalls):
            self.assertEqual((yield from self._metadata(metadata)), expected)
            self.assertEqual((yield bus.adr), address)
            self.assertEqual((yield bus.we), int(write))
            self.assertFalse((yield sink.ready))
            yield

        yield metadata.guard_fault.eq(guard)
        yield bus.ack.eq(1)
        yield
        yield bus.ack.eq(0)
        yield metadata.guard_fault.eq(0)
        if write:
            yield sink.valid.eq(0)
            yield
        else:
            # A read does not consume its incoming MMAP beat until the read
            # response is accepted. CYC is low throughout this backpressure.
            for _ in range(6):
                if (yield master.source.valid):
                    break
                yield
            else:
                self.fail("LiteEth did not present a read response")
            for _ in range(response_stalls):
                self.assertFalse((yield bus.cyc))
                self.assertTrue((yield master.source.valid))
                self.assertFalse((yield sink.ready))
                self.assertEqual((yield master.source.data), value ^ 0xffffffff)
                self.assertEqual((yield metadata.record_count), expected_count)
                self.assertEqual((yield metadata.record_abort), int(abort or guard))
                yield
            yield master.source.ready.eq(1)
            yield
            yield sink.valid.eq(0)
            yield master.source.ready.eq(0)
            yield

    def _run(self, dut, bench, expected_acks=None):
        acknowledgements = []

        @passive
        def monitor():
            while True:
                bus = dut.master.bus
                if (yield bus.cyc) and (yield bus.stb) and (yield bus.ack):
                    acknowledgements.append((yield from self._metadata(dut.metadata)))
                yield

        run_simulation(dut, [bench(), monitor()])
        if expected_acks is not None:
            self.assertEqual(acknowledgements, expected_acks)
        return acknowledgements

    def test_write_then_read_sections_survive_ack_and_response_stalls(self):
        dut = _Fixture()

        def bench():
            for write, count in ((True, 3), (False, 3), (True, 1), (False, 1)):
                for index in range(count):
                    yield from self._word(dut, count=count, index=index,
                                          write=write, stalls=2 + index,
                                          response_stalls=3 + index)

        self._run(dut, bench, [
            (1, 0, 3, 2, 0), (0, 0, 3, 2, 0), (0, 1, 3, 2, 0),
            (1, 0, 3, 2, 0), (0, 0, 3, 2, 0), (0, 1, 3, 2, 0),
            (1, 1, 1, 2, 0), (1, 1, 1, 2, 0),
        ])

    def test_guard_failure_poisons_remaining_section_and_read_response_gap(self):
        for write in (True, False):
            with self.subTest(write=write):
                dut = _Fixture()

                def bench():
                    yield from self._word(dut, count=3, index=0, write=write)
                    yield from self._word(dut, count=3, index=1, write=write,
                                          guard=True, guard_at_ack=True)
                    self.assertTrue((yield dut.metadata.record_abort))
                    # No new transfer is needed to keep the source-qualified
                    # abort asserted while another host could own the bus.
                    for _ in range(5):
                        self.assertTrue((yield dut.metadata.record_abort))
                        yield
                    yield from self._word(dut, count=3, index=2, write=write,
                                          abort=True)
                    self.assertFalse((yield dut.metadata.record_abort))
                    yield from self._word(dut, count=1, index=0, write=write)

                self._run(dut, bench, [
                    (1, 0, 3, 2, 0), (0, 0, 3, 2, 1),
                    (0, 1, 3, 2, 1), (1, 1, 1, 2, 0),
                ])

    def test_fault_on_last_word_is_visible_on_ack_and_clears_for_next_section(self):
        for write in (True, False):
            with self.subTest(write=write):
                dut = _Fixture()

                def bench():
                    yield from self._word(dut, count=1, index=0, write=write,
                                          guard=True, guard_at_ack=True)
                    yield from self._word(dut, count=1, index=0, write=write)

                self._run(dut, bench, [(1, 1, 1, 2, 1), (1, 1, 1, 2, 0)])

    def test_maximum_section_count_does_not_wrap_early(self):
        dut = _Fixture()

        def bench():
            for index in range(255):
                yield from self._word(dut, count=255, index=index, stalls=1)
            yield from self._word(dut, count=1, index=0)

        self._run(dut, bench, [
            (int(index == 0), int(index == 254), 255, 2, 0)
            for index in range(255)
        ] + [(1, 1, 1, 2, 0)])

    def test_count_change_preserves_original_count_and_poison(self):
        dut = _Fixture()

        def bench():
            yield from self._word(dut, count=3, index=0)
            yield from self._word(dut, count=4, index=1,
                                  expected_count=3, abort=True)
            yield from self._word(dut, count=3, index=2, abort=True)
            yield from self._word(dut, count=2, index=0)
            yield from self._word(dut, count=2, index=1)

        self._run(dut, bench, [
            (1, 0, 3, 2, 0), (0, 0, 3, 2, 1), (0, 1, 3, 2, 1),
            (1, 0, 2, 2, 0), (0, 1, 2, 2, 0),
        ])

    def test_early_last_and_zero_count_are_poisoned(self):
        for count in (0, 3):
            with self.subTest(count=count):
                dut = _Fixture()

                def bench():
                    yield from self._word(dut, count=count, index=0,
                                          last=True, abort=True)
                    yield from self._word(dut, count=1, index=0)

                self._run(dut, bench, [
                    (1, 1, count, 2, 1), (1, 1, 1, 2, 0),
                ])

    def test_missing_last_remains_poisoned_until_transport_reset(self):
        dut = _Fixture()

        def bench():
            yield from self._word(dut, count=1, index=0,
                                  last=False, abort=True)
            self.assertTrue((yield dut.metadata.record_abort))
            yield dut.metadata.command_reset.eq(1)
            yield dut.master.reset.eq(1)
            yield
            self.assertTrue((yield dut.metadata.record_abort))
            yield dut.metadata.command_reset.eq(0)
            yield dut.master.reset.eq(0)
            yield
            yield from self._word(dut, count=1, index=0)

        self._run(dut, bench, [(1, 0, 1, 2, 1), (1, 1, 1, 2, 0)])

    def test_metadata_widths_and_source_range(self):
        dut = _Fixture()
        self.assertEqual(len(dut.metadata.record_count), 9)
        self.assertEqual(len(dut.metadata.record_source), 2)
        for source_id in (-1, 4):
            with self.subTest(source_id=source_id):
                with self.assertRaises(ValueError):
                    _Fixture(source_id=source_id)


if __name__ == "__main__":
    unittest.main()
