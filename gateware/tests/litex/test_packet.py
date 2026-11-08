"""Simulation checks for the camera UDP packet transport.

These checks reconstruct datagram bytes at the actual valid/ready interface:
packet framing, ownership, and byte order are verified together with stalls.
Run with ``python -m unittest discover -s tests/litex -p test_packet.py``.
"""

import shutil
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path

from migen import Module, Signal
from migen.sim import passive, run_simulation
from migen.fhdl import verilog

from fpga.litex.lutcam.packet import (
    IMAGE_FRAGMENT_WORDS,
    ChildPacketizer,
    ImagePacketizer,
    PosePacketizer,
    PacketArbiter,
)


def header(camera, stream, frame, sequence, offset, length, status=0):
    return struct.pack(
        ">8I", 0x4C555443, 0x00000020 | (camera << 16) | (stream << 8),
        frame, sequence, offset, length, status, 0,
    )


def word_bytes(words, width):
    return b"".join(word.to_bytes(width, "big") for word in words)


class Capture:
    """Consume UDP32 beats and also check stability across backpressure."""

    def __init__(self, source, ready=lambda cycle: True):
        self.source = source
        self.ready = ready
        self.packets = []
        self.metadata = []
        self.pending = bytearray()
        self.stalled = None
        self.cycles = 0

    @passive
    def run(self):
        source = self.source
        while True:
            valid = (yield source.valid)
            ready = (yield source.ready)
            if valid:
                beat = (
                    (yield source.data), (yield source.last),
                    (yield source.last_be), (yield source.length),
                    (yield source.ip_address), (yield source.dst_port),
                )
                if self.stalled is not None:
                    assert beat == self.stalled, "UDP beat changed while stalled"
                if ready:
                    raw = beat[0].to_bytes(4, "little")
                    if beat[1]:
                        assert beat[2] in (1, 2, 4, 8), "invalid final-byte mask"
                        raw = raw[:beat[2].bit_length()]
                    self.pending.extend(raw)
                    if beat[1]:
                        assert len(self.pending) == beat[3], "UDP length mismatch"
                        self.packets.append(bytes(self.pending))
                        self.metadata.append(beat[3:])
                        self.pending.clear()
                    self.stalled = None
                else:
                    self.stalled = beat
            else:
                assert self.stalled is None, "UDP valid withdrawn while stalled"
            yield source.ready.eq(bool(self.ready(self.cycles)))
            self.cycles += 1
            yield


def configure(dut):
    yield dut.enable.storage.eq(1)
    yield dut.destination_ip.storage.eq(0xC0A80102)
    yield dut.destination_port.storage.eq(6000)
    yield


def idle(cycles=100):
    for _ in range(cycles):
        yield


def image_frame(dut, words, frame_id, padding_start=None):
    for index, word in enumerate(words):
        yield dut.sink.valid.eq(1)
        yield dut.sink.data.eq(word)
        yield dut.sink.sof.eq(index == 0)
        if padding_start is not None:
            yield dut.sink.padding.eq(index >= padding_start)
        yield dut.sink.frame_id.eq(frame_id)
        yield
    yield dut.sink.valid.eq(0)
    yield dut.sink.sof.eq(0)
    if padding_start is not None:
        yield dut.sink.padding.eq(0)


def child_group(dut, records, frame_id):
    for index, record in enumerate(records):
        yield dut.sink.valid.eq(1)
        yield dut.sink.data.eq(record)
        yield dut.sink.end.eq(index == len(records) - 1)
        yield dut.sink.frame_id.eq(frame_id)
        yield
    yield dut.sink.valid.eq(0)
    yield dut.sink.end.eq(0)


def pose_result(dut, words, frame_id):
    for index, word in enumerate(words):
        yield dut.sink.valid.eq(1)
        yield dut.sink.data.eq(word)
        yield dut.sink.last.eq(index == len(words) - 1)
        yield dut.sink.frame_id.eq(frame_id)
        yield
        stalled = 0
        while not (yield dut.sink.ready):
            stalled += 1
            assert stalled < 2000, "pose input did not resume after output drain"
            yield
    yield dut.sink.valid.eq(0)
    yield dut.sink.last.eq(0)


class PacketTests(unittest.TestCase):
    def test_image_fragments_byte_order_and_frame_ownership(self):
        dut = ImagePacketizer(camera_id=1, slots=8, frame_words=10, fragment_words=4)
        capture = Capture(dut.source, ready=lambda c: c >= 35 and c % 5 not in (0, 1))
        first = [0x1000 + index for index in range(10)]
        second = [0xE080 + index for index in range(10)]

        def stimulus():
            yield from configure(dut)
            yield from image_frame(dut, first, 0x01020304)
            yield from image_frame(dut, second, 0xABCDEF12)
            # Frame ownership must have been captured when each packet arrived.
            yield dut.sink.frame_id.eq(0xDEADBEEF)
            yield from idle(250)

        run_simulation(dut, [stimulus(), capture.run()])
        expected = []
        for frame_index, (frame, words) in enumerate(((0x01020304, first), (0xABCDEF12, second))):
            for fragment in range(3):
                payload = word_bytes(words[fragment * 4:(fragment + 1) * 4], 2)
                expected.append(header(1, 1, frame, frame_index * 3 + fragment,
                                       fragment * 8, len(payload), int(fragment == 2)) + payload)
        self.assertEqual(capture.packets, expected)
        self.assertEqual(capture.pending, b"")
        self.assertEqual(capture.metadata, [(len(packet), 0xC0A80102, 6000) for packet in expected])

    def test_image_simultaneous_packet_commit_and_drain(self):
        dut = ImagePacketizer(camera_id=0, slots=4, frame_words=4, fragment_words=4)
        capture = Capture(dut.source)

        def stimulus():
            yield from configure(dut)
            for frame in range(12):
                yield from image_frame(dut, [frame * 16 + n for n in range(4)], frame)
                yield from idle(14)
            yield from idle(100)

        run_simulation(dut, [stimulus(), capture.run()])
        expected = [header(0, 1, frame, frame, 0, 8, 1)
                    + word_bytes([frame * 16 + n for n in range(4)], 2)
                    for frame in range(12)]
        self.assertEqual(capture.packets, expected)

    def test_child_group_boundary_and_tuple_bit_32(self):
        dut = ChildPacketizer(camera_id=1, slots=8, max_records=9)
        capture = Capture(dut.source, ready=lambda c: c >= 25 and c % 7 != 2)
        groups = [[0x1ABCDEF01], [0x12345678, 0x100000000, 0x1FFFFFFFF], [0x10203040]]

        def stimulus():
            yield from configure(dut)
            for index, records in enumerate(groups):
                yield from child_group(dut, records, 17 + index)
            yield dut.sink.frame_id.eq(1000)
            yield from idle(200)

        run_simulation(dut, [stimulus(), capture.run()])
        expected = [header(1, 2, 17 + index, index, 0, len(records) * 8)
                    + word_bytes(records, 8) for index, records in enumerate(groups)]
        self.assertEqual(capture.packets, expected)

    def test_pose_rejections_suppressed_and_accepted_results_stall(self):
        dut = PosePacketizer(camera_id=0, slots=4)
        capture = Capture(dut.source, ready=lambda c: c >= 100 and c % 6 not in (2, 3, 4))
        rejection = [0x10203040 + n for n in range(8)]
        accepted = [0x80000000 + n for n in range(22)]
        accepted[8] = 0x00000001
        accepted2 = [0xCAFE0000 + n for n in range(22)]
        accepted2[8] = 0x00000001

        def stimulus():
            yield from configure(dut)
            yield from pose_result(dut, rejection, 19)
            yield from pose_result(dut, accepted, 20)
            yield from pose_result(dut, rejection, 21)
            yield from pose_result(dut, accepted2, 22)
            yield dut.sink.frame_id.eq(0xDEADBEEF)
            yield from idle(450)

        run_simulation(dut, [stimulus(), capture.run()])
        expected = [header(0, 3, frame, index, 0, 88) + word_bytes(words, 4)
                    for index, (frame, words) in enumerate(((20, accepted), (22, accepted2)))]
        self.assertEqual(capture.packets, expected)

    def test_image_full_queue_sets_fault_without_overwriting_packets(self):
        dut = ImagePacketizer(camera_id=0, slots=2, frame_words=4, fragment_words=4)
        capture = Capture(dut.source, ready=lambda c: c >= 100)
        status = {}

        def stimulus():
            yield from configure(dut)
            for frame in range(3):
                yield from image_frame(dut, [frame * 16 + n for n in range(4)], frame)
            yield from idle(20)
            status["level"] = (yield dut.fifo_level.status)
            status["high_water"] = (yield dut.high_water.status)
            status["faults"] = (yield dut.faults.status)
            status["sticky"] = (yield dut.fault_sticky)
            yield from idle(180)
            status["sent"] = (yield dut.packets_sent.status)
            status["committed"] = (yield dut.packets_committed.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(status["level"], 2)
        self.assertEqual(status["high_water"], 2)
        self.assertEqual(status["faults"] & 1, 1)
        self.assertEqual(status["sticky"], 1)
        self.assertEqual(status["sent"], 2)
        self.assertEqual(status["committed"], 2)
        expected = [header(0, 1, frame, frame, 0, 8, 1)
                    + word_bytes([frame * 16 + n for n in range(4)], 2)
                    for frame in range(2)]
        self.assertEqual(capture.packets, expected)

    def test_pose_full_queue_backpressures_and_recovers(self):
        dut = PosePacketizer(camera_id=1, slots=2)
        capture = Capture(dut.source, ready=lambda c: c >= 180)
        words = [0x12340000 + n for n in range(22)]
        words[1] |= 0x80000000
        words[8] = 0x00000001
        status = {}

        def stimulus():
            yield from configure(dut)
            for frame in range(3):
                yield from pose_result(dut, words, frame)
            yield from idle(180)
            status["faults"] = (yield dut.faults.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(status["faults"], 0)
        self.assertEqual(capture.packets, [header(1, 3, frame, frame, 0, 88)
                                         + word_bytes(words, 4) for frame in range(3)])

    def test_arbiter_rotates_only_at_complete_packet_boundaries(self):
        dut = Module()
        a = ImagePacketizer(camera_id=0, slots=4, frame_words=4, fragment_words=4)
        b = ImagePacketizer(camera_id=1, slots=4, frame_words=4, fragment_words=4)
        arbiter = PacketArbiter([a.source, b.source])
        dut.submodules.a = a
        dut.submodules.b = b
        dut.submodules.arbiter = arbiter
        capture = Capture(arbiter.source, ready=lambda c: c >= 50 and c % 5 != 2)

        def stimulus():
            yield from configure(a)
            yield from configure(b)
            for camera in (a, b):
                for frame in range(2):
                    yield from image_frame(camera, [frame * 16 + n for n in range(4)], frame)
            yield from idle(200)

        run_simulation(dut, [stimulus(), capture.run()])
        expected = [header(camera, 1, frame, frame, 0, 8, 1)
                    + word_bytes([frame * 16 + n for n in range(4)], 2)
                    for frame in range(2) for camera in range(2)]
        self.assertEqual(capture.packets, expected)

    def test_child_oversize_fault_discards_whole_group(self):
        dut = ChildPacketizer(camera_id=0, slots=4, max_records=2)
        capture = Capture(dut.source)
        status = {}

        def stimulus():
            yield from configure(dut)
            yield from child_group(dut, [0x12345678], 10)
            yield from child_group(dut, [0x100000001, 0x100000002, 0x100000003], 11)
            yield from idle(100)
            status["faults"] = (yield dut.faults.status)
            status["level"] = (yield dut.fifo_level.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(status["faults"], 2)
        self.assertEqual(status["level"], 0)
        self.assertEqual(capture.packets, [header(0, 2, 10, 0, 0, 8)
                                         + word_bytes([0x12345678], 8)])

    def test_child_frame_change_fault_discards_whole_group(self):
        dut = ChildPacketizer(camera_id=0, slots=4, max_records=9)
        capture = Capture(dut.source)
        status = {}

        def stimulus():
            yield from configure(dut)
            yield dut.sink.valid.eq(1)
            yield dut.sink.data.eq(0x100000001)
            yield dut.sink.frame_id.eq(10)
            yield dut.sink.end.eq(0)
            yield
            yield dut.sink.data.eq(0x100000002)
            yield dut.sink.frame_id.eq(11)
            yield dut.sink.end.eq(1)
            yield
            yield dut.sink.valid.eq(0)
            yield from idle(100)
            status["faults"] = (yield dut.faults.status)
            status["level"] = (yield dut.fifo_level.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(status["faults"], 2)
        self.assertEqual(status["level"], 0)
        self.assertEqual(capture.packets, [])

    def test_malformed_pose_status_fault_discards_whole_result(self):
        # Missing validity and malformed status flags discard the whole result.
        invalid_statuses = [0] + [1 | 1 << bit for bit in range(4, 28)]
        for status_word in invalid_statuses:
            with self.subTest(status_word=hex(status_word)):
                dut = PosePacketizer(camera_id=0, slots=4)
                capture = Capture(dut.source)
                words = [0x12340000 + n for n in range(22)]
                words[1] |= 0x80000000
                words[8] = status_word
                observed = {}

                def stimulus():
                    yield from configure(dut)
                    yield from pose_result(dut, words, 10)
                    yield from idle(100)
                    observed["faults"] = (yield dut.faults.status)
                    observed["level"] = (yield dut.fifo_level.status)

                run_simulation(dut, [stimulus(), capture.run()])
                self.assertEqual(observed["faults"], 2)
                self.assertEqual(observed["level"], 0)
                self.assertEqual(capture.packets, [])

    def test_destination_change_waits_for_queued_packets_to_drain(self):
        dut = ImagePacketizer(camera_id=0, slots=4, frame_words=4, fragment_words=4)
        capture = Capture(dut.source, ready=lambda c: c >= 70)
        observed = {}

        def stimulus():
            yield from configure(dut)
            yield from image_frame(dut, [1, 2, 3, 4], 10)
            yield from image_frame(dut, [5, 6, 7, 8], 11)
            yield dut.destination_ip.storage.eq(0x0A000001)
            yield dut.destination_port.storage.eq(8000)
            yield from idle(20)
            observed["pending_ip"] = (yield dut.active_destination_ip.status)
            observed["pending_port"] = (yield dut.active_destination_port.status)
            yield from idle(100)
            observed["drained_ip"] = (yield dut.active_destination_ip.status)
            observed["drained_port"] = (yield dut.active_destination_port.status)
            yield from image_frame(dut, [9, 10, 11, 12], 12)
            yield from idle(100)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(observed["pending_ip"], 0xC0A80102)
        self.assertEqual(observed["pending_port"], 6000)
        self.assertEqual(observed["drained_ip"], 0x0A000001)
        self.assertEqual(observed["drained_port"], 8000)
        self.assertEqual(capture.metadata, [(40, 0xC0A80102, 6000),
                                          (40, 0xC0A80102, 6000),
                                          (40, 0x0A000001, 8000)])

    def test_packet_and_byte_counters_wrap_modulo_32_bits(self):
        dut = ImagePacketizer(camera_id=0, slots=4, frame_words=4, fragment_words=4)
        capture = Capture(dut.source)
        observed = {}

        def stimulus():
            yield from configure(dut)
            # Seed hardware counters near wrap without billions of cycles.
            yield dut.packets_committed.status.eq(0xFFFFFFFF)
            yield dut.packets_sent.status.eq(0xFFFFFFFF)
            yield dut.bytes_sent.status.eq(0xFFFFFFF0)
            yield from image_frame(dut, [1, 2, 3, 4], 10)
            yield from idle(100)
            observed["committed"] = (yield dut.packets_committed.status)
            observed["sent"] = (yield dut.packets_sent.status)
            observed["bytes"] = (yield dut.bytes_sent.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(observed, {"committed": 0, "sent": 0, "bytes": 24})
        self.assertEqual(len(capture.packets), 1)

    def test_full_pose_queue_retires_rejection_and_holds_accepted_prefix(self):
        dut = PosePacketizer(camera_id=0, slots=2)
        release_output = [False]
        capture = Capture(dut.source, ready=lambda c: release_output[0])
        accepted = [0x80000000 + n for n in range(22)]
        accepted[8] = 0x00000001
        rejection = [0x12340000 + n for n in range(8)]
        observed = {}

        def stimulus():
            yield from configure(dut)
            yield from pose_result(dut, accepted, 10)
            yield from pose_result(dut, accepted, 11)
            start_cycle = capture.cycles
            yield from pose_result(dut, rejection, 12)
            observed["rejection_clocks"] = capture.cycles - start_cycle
            # Two prefix words determine the accepted marker before reserving
            # scarce output RAM. They must transfer even with both slots full.
            for index in range(2):
                yield dut.sink.valid.eq(1)
                yield dut.sink.data.eq(accepted[index])
                yield dut.sink.frame_id.eq(13)
                yield dut.sink.last.eq(0)
                yield
                self.assertEqual((yield dut.sink.ready), 1)
            yield dut.sink.data.eq(accepted[2])
            for _ in range(20):
                yield
                self.assertEqual((yield dut.sink.ready), 0)
            release_output[0] = True
            waited = 0
            while not (yield dut.sink.ready):
                waited += 1
                self.assertLess(waited, 2000)
                yield
            yield from pose_result(dut, accepted[3:], 13)
            yield from idle(200)
            observed["faults"] = (yield dut.faults.status)
            observed["rejections"] = (yield dut.rejections.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(observed["rejection_clocks"], 8)
        self.assertEqual(observed["faults"], 0)
        self.assertEqual(observed["rejections"], 1)
        self.assertEqual(capture.packets, [header(0, 3, frame, index, 0, 88)
                                         + word_bytes(accepted, 4)
                                         for index, frame in enumerate((10, 11, 13))])

    def test_pose_marker_must_match_native_packet_length(self):
        for length, marker in ((22, 0), (8, 1)):
            with self.subTest(length=length, accepted_marker=marker):
                dut = PosePacketizer(camera_id=0, slots=4)
                capture = Capture(dut.source)
                words = [0x12340000 + n for n in range(length)]
                words[1] |= marker << 31
                if length == 22:
                    words[8] = 0x00000001
                observed = {}

                def stimulus():
                    yield from configure(dut)
                    yield from pose_result(dut, words, 10)
                    yield from idle(100)
                    observed["faults"] = (yield dut.faults.status)
                    observed["level"] = (yield dut.fifo_level.status)

                run_simulation(dut, [stimulus(), capture.run()])
                self.assertEqual(observed["faults"], 2)
                self.assertEqual(observed["level"], 0)
                self.assertEqual(capture.packets, [])

    def test_virtual_padding_preserves_prefix_and_fragment_boundaries(self):
        for prefix_words in (0, 1, 2, 3, 4, 5, 8, 15, 16, 17):
            with self.subTest(prefix_words=prefix_words):
                dut = ImagePacketizer(camera_id=1, slots=8, frame_words=18, fragment_words=4)
                capture = Capture(dut.source, ready=lambda c: c >= 40 and c % 4 == 0)
                words = [0xE080 + n for n in range(prefix_words)] + [0] * (18 - prefix_words)
                observed = {}

                def stimulus():
                    yield from configure(dut)
                    yield from image_frame(dut, words, 0x10203040, padding_start=prefix_words)
                    yield from idle(350)
                    observed["faults"] = (yield dut.faults.status)

                run_simulation(dut, [stimulus(), capture.run()])
                expected = []
                for fragment in range(5):
                    payload = word_bytes(words[fragment * 4:(fragment + 1) * 4], 2)
                    expected.append(header(1, 1, 0x10203040, fragment, fragment * 8,
                                           len(payload), int(fragment == 4)) + payload)
                self.assertEqual(observed["faults"], 0)
                self.assertEqual(capture.packets, expected)

    def test_late_padding_violation_preserves_published_zero_tail(self):
        for nonzero_word, withdrawn_marker in ((1, False), (0, True)):
            with self.subTest(nonzero_word=nonzero_word, withdrawn_marker=withdrawn_marker):
                dut = ImagePacketizer(camera_id=0, slots=4, frame_words=8, fragment_words=4)
                capture = Capture(dut.source)
                observed = {}

                def stimulus():
                    yield from configure(dut)
                    yield dut.sink.valid.eq(1)
                    yield dut.sink.sof.eq(1)
                    yield dut.sink.data.eq(0x1234)
                    yield
                    yield dut.sink.sof.eq(0)
                    yield dut.sink.data.eq(0)
                    yield dut.sink.padding.eq(1)
                    yield
                    yield dut.sink.data.eq(nonzero_word)
                    yield dut.sink.padding.eq(not withdrawn_marker)
                    yield
                    yield dut.sink.valid.eq(0)
                    yield from idle(100)
                    observed["faults"] = (yield dut.faults.status)
                    observed["level"] = (yield dut.fifo_level.status)

                run_simulation(dut, [stimulus(), capture.run()])
                self.assertEqual(observed, {"faults": 2, "level": 0})
                words = [0x1234] + [0] * 7
                self.assertEqual(capture.packets, [header(0, 1, 0, fragment, fragment * 8, 8, fragment)
                                                 + word_bytes(words[fragment * 4:(fragment + 1) * 4], 2)
                                                 for fragment in range(2)])

    def test_padding_publishes_and_frees_slot_before_physical_tail_ends(self):
        dut = ImagePacketizer(camera_id=0, slots=2, frame_words=8, fragment_words=4)
        capture = Capture(dut.source)
        observed = {}
        next_frame = [0xAB00 + n for n in range(8)]

        def stimulus():
            yield from configure(dut)
            yield dut.sink.valid.eq(1)
            yield dut.sink.sof.eq(1)
            yield dut.sink.frame_id.eq(7)
            yield dut.sink.data.eq(0x1234)
            yield
            yield dut.sink.sof.eq(0)
            yield dut.sink.padding.eq(1)
            yield dut.sink.data.eq(0)
            yield
            yield dut.sink.valid.eq(0)
            yield from idle(100)
            observed["packets_before_tail_end"] = len(capture.packets)
            observed["level_before_tail_end"] = (yield dut.fifo_level.status)
            # Consume the six remaining physical padding words after the
            # complete logical image has already left the packet controller.
            for _ in range(6):
                yield dut.sink.valid.eq(1)
                yield
            yield dut.sink.valid.eq(0)
            yield dut.sink.padding.eq(0)
            yield from image_frame(dut, next_frame, 8)
            yield from idle(100)
            observed["faults"] = (yield dut.faults.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(observed, {"packets_before_tail_end": 2, "level_before_tail_end": 0, "faults": 0})
        expected = []
        for frame_index, (frame, words) in enumerate(((7, [0x1234] + [0] * 7), (8, next_frame))):
            for fragment in range(2):
                expected.append(header(0, 1, frame, frame_index * 2 + fragment,
                                       fragment * 8, 8, fragment)
                                + word_bytes(words[fragment * 4:(fragment + 1) * 4], 2))
        self.assertEqual(capture.packets, expected)

    def test_new_sof_before_physical_padding_end_faults_after_tail_drains(self):
        dut = ImagePacketizer(camera_id=0, slots=2, frame_words=8, fragment_words=4)
        capture = Capture(dut.source)
        observed = {}

        def stimulus():
            yield from configure(dut)
            yield dut.sink.valid.eq(1)
            yield dut.sink.sof.eq(1)
            yield dut.sink.frame_id.eq(7)
            yield dut.sink.data.eq(0x1234)
            yield
            yield dut.sink.sof.eq(0)
            yield dut.sink.padding.eq(1)
            yield dut.sink.data.eq(0)
            yield
            yield dut.sink.valid.eq(0)
            yield from idle(100)
            observed["level_before_new_sof"] = (yield dut.fifo_level.status)
            yield dut.sink.valid.eq(1)
            yield dut.sink.sof.eq(1)
            yield dut.sink.padding.eq(0)
            yield dut.sink.data.eq(0x5678)
            yield dut.sink.frame_id.eq(8)
            yield
            yield dut.sink.valid.eq(0)
            yield from idle(100)
            observed["faults"] = (yield dut.faults.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(observed, {"level_before_new_sof": 0, "faults": 2})
        words = [0x1234] + [0] * 7
        self.assertEqual(capture.packets, [header(0, 1, 7, fragment, fragment * 8, 8, fragment)
                                         + word_bytes(words[fragment * 4:(fragment + 1) * 4], 2)
                                         for fragment in range(2)])

    def test_padding_on_sof_uses_new_frame_metadata_and_consumes_when_slots_full(self):
        dut = ImagePacketizer(camera_id=0, slots=4, frame_words=8, fragment_words=4)
        capture = Capture(dut.source, ready=lambda c: c >= 80 and c % 2 == 0)
        first = [0x1200 + index for index in range(8)]
        observed = {}

        def stimulus():
            yield from configure(dut)
            yield from image_frame(dut, first, 10)
            yield from image_frame(dut, [0] * 8, 11, padding_start=0)
            yield from image_frame(dut, [0] * 8, 12, padding_start=0)
            yield from idle(300)
            observed["faults"] = (yield dut.faults.status)
            observed["level"] = (yield dut.fifo_level.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(observed, {"faults": 0, "level": 0})
        expected = []
        for frame_index, (frame, words) in enumerate(((10, first), (11, [0] * 8), (12, [0] * 8))):
            for fragment in range(2):
                expected.append(header(0, 1, frame, frame_index * 2 + fragment,
                                       fragment * 8, 8, fragment)
                                + word_bytes(words[fragment * 4:(fragment + 1) * 4], 2))
        self.assertEqual(capture.packets, expected)

    def test_disabled_padding_frame_does_not_poison_next_frame(self):
        dut = ImagePacketizer(camera_id=0, slots=4, frame_words=8, fragment_words=4)
        capture = Capture(dut.source)
        words = [0xAB00 + n for n in range(8)]
        observed = {}

        def stimulus():
            yield from image_frame(dut, [0] * 8, 10, padding_start=0)
            yield from configure(dut)
            yield from image_frame(dut, words, 11)
            yield from idle(100)
            observed["faults"] = (yield dut.faults.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(observed["faults"], 0)
        self.assertEqual(capture.packets, [header(0, 1, 11, fragment, fragment * 8, 8, fragment)
                                         + word_bytes(words[fragment * 4:(fragment + 1) * 4], 2)
                                         for fragment in range(2)])

    def test_virtual_padding_keeps_next_frame_behind_previous_tail(self):
        dut = ImagePacketizer(camera_id=0, slots=8, frame_words=18, fragment_words=4)
        capture = Capture(dut.source, ready=lambda c: c >= 60 and c % 3 == 0)
        first = [0x1122, 0x3344, 0x5566] + [0] * 15
        second = [0xABC0 + n for n in range(18)]
        observed = {}

        def stimulus():
            yield from configure(dut)
            yield from image_frame(dut, first, 10, padding_start=3)
            yield from image_frame(dut, second, 11)
            yield dut.sink.frame_id.eq(0xBADBAD)
            yield from idle(500)
            observed["faults"] = (yield dut.faults.status)
            observed["level"] = (yield dut.fifo_level.status)

        run_simulation(dut, [stimulus(), capture.run()])
        expected = []
        for frame_index, (frame, words) in enumerate(((10, first), (11, second))):
            for fragment in range(5):
                payload = word_bytes(words[fragment * 4:(fragment + 1) * 4], 2)
                expected.append(header(0, 1, frame, frame_index * 5 + fragment,
                                       fragment * 8, len(payload), int(fragment == 4)) + payload)
        self.assertEqual(observed, {"faults": 0, "level": 0})
        self.assertEqual(capture.packets, expected)

    def test_six_streams_share_udp_during_virtual_padding(self):
        dut = Module()
        cameras = [[ImagePacketizer(camera_id=c, slots=4, frame_words=64, fragment_words=16),
                    ChildPacketizer(camera_id=c, slots=4), PosePacketizer(camera_id=c, slots=4)]
                   for c in range(2)]
        sources = []
        for camera_id, controllers in enumerate(cameras):
            for stream_id, controller in enumerate(controllers):
                setattr(dut.submodules, f"camera{camera_id}_stream{stream_id}", controller)
                sources.append(controller.source)
        arbiter = PacketArbiter(sources)
        dut.submodules.arbiter = arbiter
        go = Signal()
        pace = [0, 100]

        def ready(cycle):
            if len(capture.packets) != pace[0]:
                pace[0] = len(capture.packets)
                pace[1] = cycle + 8
            return cycle >= pace[1] and cycle % 4 == 0

        capture = Capture(arbiter.source, ready=ready)
        expected = {}
        for camera_id in range(2):
            image_words = [0x1000 * (camera_id + 1) + n for n in range(5)] + [0] * 59
            for fragment in range(4):
                payload = word_bytes(image_words[fragment * 16:(fragment + 1) * 16], 2)
                expected[(camera_id, 1, fragment)] = header(camera_id, 1, 44, fragment,
                                                         fragment * 32, 32, int(fragment == 3)) + payload
            for sequence in range(2):
                children = [0x100000000 + camera_id * 0x100 + sequence * 16 + n for n in range(9)]
                pose = [0x80000000 + camera_id * 0x100 + sequence * 32 + n for n in range(22)]
                pose[8] = 0x00000001
                expected[(camera_id, 2, sequence)] = header(camera_id, 2, 44, sequence, 0, 72) + word_bytes(children, 8)
                expected[(camera_id, 3, sequence)] = header(camera_id, 3, 44, sequence, 0, 88) + word_bytes(pose, 4)

        def produce_image(camera_id):
            while not (yield go):
                yield
            words = [0x1000 * (camera_id + 1) + n for n in range(5)] + [0] * 59
            yield from image_frame(cameras[camera_id][0], words, 44, padding_start=5)

        def produce_children(camera_id):
            while not (yield go):
                yield
            for sequence in range(2):
                words = [0x100000000 + camera_id * 0x100 + sequence * 16 + n for n in range(9)]
                yield from child_group(cameras[camera_id][1], words, 44)

        def produce_poses(camera_id):
            while not (yield go):
                yield
            for sequence in range(2):
                words = [0x80000000 + camera_id * 0x100 + sequence * 32 + n for n in range(22)]
                words[8] = 0x00000001
                yield from pose_result(cameras[camera_id][2], words, 44)

        def stimulus():
            for controllers in cameras:
                for controller in controllers:
                    yield from configure(controller)
            yield go.eq(1)
            for _ in range(4000):
                if len(capture.packets) == len(expected):
                    break
                yield
            else:
                self.fail("six-stream output failed to drain")
            yield from idle(3)
            for controllers in cameras:
                for controller in controllers:
                    self.assertEqual((yield controller.faults.status), 0)
                    self.assertEqual((yield controller.fifo_level.status), 0)

        generators = [stimulus(), capture.run()]
        for camera_id in range(2):
            generators += [produce_image(camera_id), produce_children(camera_id), produce_poses(camera_id)]
        run_simulation(dut, generators)
        received = {}
        for packet in capture.packets:
            words = struct.unpack(">8I", packet[:32])
            key = ((words[1] >> 16) & 0xFF, (words[1] >> 8) & 0xFF, words[3])
            self.assertNotIn(key, received)
            received[key] = packet
        self.assertEqual(received, expected)

    @unittest.skipUnless(shutil.which("iverilog") and shutil.which("vvp"),
                         "full-frame transport simulation requires Icarus Verilog")
    def test_two_full_padding_bursts_with_bounded_gigabit_service(self):
        # Run the actual Migen-generated RTL in Icarus: interpreted Migen
        # simulation takes minutes for this ~344,000-clock stress fixture.
        # This is a synthetic zero-tail stress case, not a claim about a
        # particular image's semantic bit count. Both cameras burst at one
        # 16-bit word/clock. UDP service is slower than gigabit wire speed.
        fragments_per_frame = (64000 + IMAGE_FRAGMENT_WORDS - 1) // IMAGE_FRAGMENT_WORDS
        fragment_bytes = IMAGE_FRAGMENT_WORDS * 2
        dut = Module()
        cameras = [ImagePacketizer(camera_id=camera, slots=4) for camera in range(2)]
        dut.submodules.camera0 = cameras[0]
        dut.submodules.camera1 = cameras[1]
        arbiter = PacketArbiter([camera.source for camera in cameras])
        dut.submodules.arbiter = arbiter
        ports = {}
        for camera_id, camera in enumerate(cameras):
            dut.comb += [camera.enable.storage.eq(1),
                         camera.destination_ip.storage.eq(0xC0A80102),
                         camera.destination_port.storage.eq(6000)]
            for field in ("valid", "sof", "padding", "data", "frame_id"):
                ports[f"camera{camera_id}_{field}"] = getattr(camera.sink, field)
            ports[f"faults{camera_id}"] = camera.faults.status
            ports[f"high_water{camera_id}"] = camera.high_water.status
            ports[f"level{camera_id}"] = camera.fifo_level.status
        for field in ("valid", "ready", "data", "last", "last_be", "length"):
            ports[f"udp_{field}"] = getattr(arbiter.source, field)
        for name, signal in ports.items():
            signal.name_override = name
        rtl = verilog.convert(dut, ios=set(ports.values()), name="PacketBurstFixture")
        testbench = r"""
module PacketBurstTest;
  localparam integer EXPECTED_PACKETS = __EXPECTED_PACKETS__;
  reg sys_clk = 0;
  reg sys_rst = 1;
  always #5 sys_clk = ~sys_clk;
  reg camera_valid = 0, camera_sof = 0, camera_padding = 0;
  reg [15:0] camera0_data = 0, camera1_data = 0;
  reg udp_ready = 0;
  wire udp_valid, udp_last;
  wire [31:0] udp_data;
  wire [3:0] udp_last_be;
  wire [15:0] udp_length;
  wire [1:0] faults0, faults1;
  wire [31:0] high_water0, high_water1, level0, level1;
  integer cycle = 0, gap = 0, packets = 0, output_file, index;
  PacketBurstFixture dut (
    .sys_clk(sys_clk), .sys_rst(sys_rst),
    .camera0_valid(camera_valid), .camera1_valid(camera_valid),
    .camera0_sof(camera_sof), .camera1_sof(camera_sof),
    .camera0_padding(camera_padding), .camera1_padding(camera_padding),
    .camera0_data(camera0_data), .camera1_data(camera1_data),
    .camera0_frame_id(32'h10203040), .camera1_frame_id(32'h10203040),
    .udp_valid(udp_valid), .udp_ready(udp_ready), .udp_data(udp_data),
    .udp_last(udp_last), .udp_last_be(udp_last_be), .udp_length(udp_length),
    .faults0(faults0), .faults1(faults1),
    .high_water0(high_water0), .high_water1(high_water1),
    .level0(level0), .level1(level1)
  );
  always @(negedge sys_clk) begin
    if (sys_rst) begin
      udp_ready = 0;
      cycle = 0;
      gap = 0;
    end else begin
      udp_ready = (gap == 0) && ((cycle % 4) == 0);
      if (gap > 0) gap = gap - 1;
      cycle = cycle + 1;
    end
  end
  always @(posedge sys_clk) begin
    if (!sys_rst && udp_valid && udp_ready) begin
      $fwrite(output_file, "%08x %01x %01x %0d\n", udp_data, udp_last, udp_last_be, udp_length);
      if (udp_last) begin
        packets = packets + 1;
        gap = 100;
      end
    end
    if (!sys_rst && ((faults0 !== 0) || (faults1 !== 0)))
      $fatal(1, "packet controller fault: %x %x", faults0, faults1);
  end
  initial begin
    output_file = $fopen("udp_beats.txt", "w");
    repeat (3) @(negedge sys_clk);
    sys_rst = 0;
    repeat (3) @(negedge sys_clk);
    for (index = 0; index < 64000; index = index + 1) begin
      @(negedge sys_clk);
      camera_valid = 1;
      camera_sof = (index == 0);
      camera_padding = (index >= 233);
      camera0_data = (index < 233) ? 16'h1000 + index : 0;
      camera1_data = (index < 233) ? 16'h2000 + index : 0;
    end
    @(negedge sys_clk);
    camera_valid = 0;
    camera_sof = 0;
    camera_padding = 0;
    while (packets < EXPECTED_PACKETS) @(negedge sys_clk);
    repeat (5) @(negedge sys_clk);
    if ((packets != EXPECTED_PACKETS) || (level0 !== 0) || (level1 !== 0) ||
        (high_water0 > 4) || (high_water1 > 4))
      $fatal(1, "bad final queue state");
    $display("PASS cycles=%0d packets=%0d high_water=%0d,%0d", cycle, packets, high_water0, high_water1);
    $fclose(output_file);
    $finish;
  end
  initial begin
    repeat (450000) @(posedge sys_clk);
    $fatal(1, "full-frame packet drain timed out at %0d datagrams", packets);
  end
endmodule
"""
        with tempfile.TemporaryDirectory(prefix="lutcam-packet-test-") as temporary:
            directory = Path(temporary)
            (directory / "packet.v").write_text(rtl.main_source)
            for name, content in rtl.data_files.items():
                (directory / name).write_text(content)
            (directory / "testbench.v").write_text(testbench.replace("__EXPECTED_PACKETS__", str(2 * fragments_per_frame)))
            compilation = subprocess.run(["iverilog", "-g2012", "-s", "PacketBurstTest", "-o", "simulate", "packet.v", "testbench.v"],
                                         cwd=directory, capture_output=True, text=True, timeout=60)
            self.assertEqual(compilation.returncode, 0, compilation.stdout + compilation.stderr)
            simulation = subprocess.run(["vvp", "simulate"], cwd=directory, capture_output=True, text=True, timeout=90)
            self.assertEqual(simulation.returncode, 0, simulation.stdout + simulation.stderr)
            self.assertIn("PASS", simulation.stdout)
            packets = []
            pending = bytearray()
            for line in (directory / "udp_beats.txt").read_text().splitlines():
                data, last, mask, length = line.split()
                pending.extend(int(data, 16).to_bytes(4, "little"))
                if int(last, 16):
                    self.assertEqual(int(mask, 16), 8)
                    self.assertEqual(len(pending), int(length))
                    packets.append(bytes(pending))
                    pending.clear()
            self.assertFalse(pending)
        self.assertEqual(len(packets), 2 * fragments_per_frame)
        for camera_id in range(2):
            camera_packets = [packet for packet in packets if packet[5] == camera_id]
            expected_image = word_bytes([0x1000 * (camera_id + 1) + n for n in range(233)], 2)
            expected_image += bytes(2 * (64000 - 233))
            self.assertEqual(len(camera_packets), fragments_per_frame)
            self.assertEqual(b"".join(packet[32:] for packet in camera_packets), expected_image)
            for fragment, packet in enumerate(camera_packets):
                expected_length = min(fragment_bytes, 128000 - fragment * fragment_bytes)
                self.assertEqual(packet[:32], header(camera_id, 1, 0x10203040,
                                                    fragment, fragment * fragment_bytes,
                                                    expected_length, int(fragment == fragments_per_frame - 1)))

    def test_non_power_of_two_image_stride_wrap_is_wire_equivalent(self):
        expected = []
        for frame in range(6):
            words = [0x1000 + frame * 16 + n for n in range(12)]
            for fragment in range(2):
                expected.append(header(0, 1, frame, frame * 2 + fragment, fragment * 12, 12, fragment)
                                + word_bytes(words[fragment * 6:(fragment + 1) * 6], 2))
        for packed in (True, False):
            with self.subTest(packed_slots=packed):
                dut = ImagePacketizer(camera_id=0, slots=2, frame_words=12, fragment_words=6,
                                      packed_slots=packed)
                capture = Capture(dut.source, ready=lambda c: c % 4 != 1)

                def stimulus():
                    yield from configure(dut)
                    for frame in range(6):
                        yield from image_frame(dut, [0x1000 + frame * 16 + n for n in range(12)], frame)
                        yield from idle(80)
                    self.assertEqual((yield dut.faults.status), 0)

                run_simulation(dut, [stimulus(), capture.run()])
                self.assertEqual(capture.packets, expected)

    def test_non_power_of_two_child_stride_wrap_is_wire_equivalent(self):
        groups = [[0x100000000 + group * 16 + n for n in range(group % 3 + 1)] for group in range(9)]
        expected = [header(1, 2, group, group, 0, len(words) * 8) + word_bytes(words, 8)
                    for group, words in enumerate(groups)]
        for packed in (True, False):
            with self.subTest(packed_slots=packed):
                dut = ChildPacketizer(camera_id=1, slots=2, max_records=3, packed_slots=packed)
                capture = Capture(dut.source, ready=lambda c: c % 4 != 1)

                def stimulus():
                    yield from configure(dut)
                    for group, words in enumerate(groups):
                        yield from child_group(dut, words, group)
                        yield from idle(40)
                    self.assertEqual((yield dut.faults.status), 0)

                run_simulation(dut, [stimulus(), capture.run()])
                self.assertEqual(capture.packets, expected)

    def test_22_word_pose_stride_wrap_is_wire_equivalent(self):
        results = [[0x80000000 + frame * 32 + n for n in range(22)] for frame in range(8)]
        for words in results:
            words[8] = 0x00000001
        expected = [header(1, 3, 0x10203040 + frame, frame, 0, 88) + word_bytes(words, 4)
                    for frame, words in enumerate(results)]
        for packed in (True, False):
            with self.subTest(packed_slots=packed):
                dut = PosePacketizer(camera_id=1, slots=2, packed_slots=packed)
                capture = Capture(dut.source, ready=lambda c: c % 3 != 1)

                def stimulus():
                    yield from configure(dut)
                    for frame, words in enumerate(results):
                        yield from pose_result(dut, words, 0x10203040 + frame)
                        yield from idle(80)
                    self.assertEqual((yield dut.faults.status), 0)

                run_simulation(dut, [stimulus(), capture.run()])
                self.assertEqual(capture.packets, expected)

    def test_extra_image_word_without_sof_faults_after_committed_frame(self):
        dut = ImagePacketizer(camera_id=0, slots=4, frame_words=2, fragment_words=2)
        capture = Capture(dut.source)
        observed = {}

        def stimulus():
            yield from configure(dut)
            yield from image_frame(dut, [0x1122, 0x3344], 10)
            yield dut.sink.valid.eq(1)
            yield dut.sink.sof.eq(0)
            yield dut.sink.data.eq(0x5566)
            yield
            yield dut.sink.valid.eq(0)
            yield from idle(100)
            observed["faults"] = (yield dut.faults.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(observed["faults"], 2)
        self.assertEqual(capture.packets, [header(0, 1, 10, 0, 0, 4, 1) + bytes.fromhex("11223344")])

    def test_disabled_stream_emits_no_packets(self):
        dut = ImagePacketizer(camera_id=0, slots=4, frame_words=4, fragment_words=4)
        capture = Capture(dut.source)

        def stimulus():
            yield from image_frame(dut, [1, 2, 3, 4], 0)
            yield from idle(50)
            yield from configure(dut)
            yield dut.capture_enable.eq(0)
            yield from image_frame(dut, [5, 6, 7, 8], 1)
            yield from idle(50)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(capture.packets, [])


if __name__ == "__main__":
    unittest.main()
