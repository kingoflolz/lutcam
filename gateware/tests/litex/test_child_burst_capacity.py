"""Native child bursts must fit before a stalled UDP serializer drains them.

Both cameras emit independent nine-tuple groups concurrently. The oracle
reconstructs every group from actual UDP bytes, allowing the first batch to
have been planned before the rest of the burst arrived.
"""

import struct
import unittest

from migen import Signal
from migen.sim import passive, run_simulation

from test_packet import Capture, idle, word_bytes
from test_shared_packet import SharedHarness, batched_header, configure


FRAME = 0x1234a001


def tuples(camera, group):
    """Distinct native values also exercise both values of tuple bit 32."""
    return [(((camera + group + index) & 1) << 32)
            | (camera << 30) | (group << 16) | (index << 8)
            | ((camera + group * 17 + index * 29) & 255)
            for index in range(9)]


class ChildBurstCapacityTests(unittest.TestCase):
    def run_burst(self, *, slots, groups=64, tuple_gap=0, frames=None):
        frames = [FRAME] * groups if frames is None else frames
        self.assertEqual(len(frames), groups)
        dut = SharedHarness(cameras=2, child_slots=slots, image_slots=2,
                            pose_slots=2, frame_words=16, fragment_words=16,
                            batched_packets=True, compact_padding=True)
        transport = dut.transport
        children = [transport.channels[camera * 3 + 1] for camera in range(2)]
        go = Signal()
        finished = [Signal(), Signal()]
        ready = {"enabled": False}
        capture = Capture(transport.source,
            ready=lambda cycle: ready["enabled"] and cycle % 7 in (0, 2, 5))
        observed = {"input_tuples": [0, 0], "releases": [0, 0],
                    "full_levels": [], "final": []}
        accepted = min(groups, slots)
        overflow = int(groups > slots)

        @passive
        def monitor():
            while True:
                for camera, child in enumerate(children):
                    self.assertLessEqual((yield child.fifo_level), slots)
                    if (yield child.sink.valid):
                        observed["input_tuples"][camera] += 1
                        self.assertEqual((yield child.sink.ready), 1,
                                         "native child producer cannot be backpressured")
                    if (yield child.release):
                        observed["releases"][camera] += 1
                        self.assertEqual((yield transport.source.valid), 1)
                        self.assertEqual((yield transport.source.ready), 1)
                yield

        def producer(camera):
            child = children[camera]
            while not (yield go):
                yield
            for group, frame in enumerate(frames):
                for index, value in enumerate(tuples(camera, group)):
                    yield child.sink.valid.eq(1)
                    yield child.sink.data.eq(value)
                    yield child.sink.frame_id.eq(frame)
                    yield child.sink.end.eq(index == 8)
                    yield
                    if tuple_gap:
                        yield child.sink.valid.eq(0)
                        yield child.sink.end.eq(0)
                        yield from idle(tuple_gap)
            yield child.sink.valid.eq(0)
            yield child.sink.end.eq(0)
            yield child.sink.frame_id.eq(0xdeadbeef)
            yield finished[camera].eq(1)
            yield

        def stimulus():
            yield from configure(transport)
            yield go.eq(1)
            for _ in range(groups * 9 * (tuple_gap + 1) + 40):
                if (yield finished[0]) and (yield finished[1]):
                    break
                yield
            else:
                self.fail("concurrent native producers did not finish")
            yield from idle(5)
            self.assertEqual(capture.packets, [])
            self.assertEqual(capture.pending, b"")
            self.assertEqual(observed["input_tuples"], [groups * 9] * 2)
            self.assertEqual(observed["releases"], [0, 0])
            for camera, child in enumerate(children):
                index = camera * 3 + 1
                level = (yield child.committed_level)
                observed["full_levels"].append(level)
                self.assertEqual(level, accepted)
                self.assertEqual((yield child.fifo_level), accepted)
                self.assertEqual((yield child.high_water), accepted)
                self.assertEqual((yield child.faults), overflow)
                self.assertEqual((yield dut.overflows[index]), overflow)
                self.assertEqual((yield dut.protocol_errors[index]), 0)
            ready["enabled"] = True
            for _ in range(20_000):
                if (yield transport.drained):
                    break
                yield
            else:
                self.fail("child queues did not drain under bounded UDP backpressure")
            yield from idle(6)
            self.assertEqual(capture.pending, b"")
            self.assertEqual(observed["releases"], [accepted] * 2)
            for camera, child in enumerate(children):
                index = camera * 3 + 1
                for level in (child.fifo_level, child.committed_level,
                              child.counter_committed_level):
                    self.assertEqual((yield level), 0)
                self.assertEqual((yield child.faults), overflow,
                                 "draining must not silently clear a real overflow")
                self.assertEqual((yield dut.overflows[index]), overflow)
                self.assertEqual((yield dut.protocol_errors[index]), 0)
                observed["final"].append({
                    "committed_packets": (yield dut.committed[index]),
                    "sent_packets": (yield dut.sent[index]),
                    "bytes": (yield dut.byte_counts[index]),
                    "next_sequence": (yield dut.sequences[index]),
                })
            for index in (0, 2, 3, 5):
                self.assertEqual((yield transport.channels[index].faults), 0)
                self.assertEqual((yield transport.channels[index].fifo_level), 0)
            self.assertEqual((yield transport.drained), 1)

        run_simulation(dut, [stimulus(), producer(0), producer(1), monitor(), capture.run()])

        recovered = [[], []]
        sequences = [0, 0]
        byte_counts = [0, 0]
        packet_frames = [[], []]
        for packet, metadata in zip(capture.packets, capture.metadata):
            magic, info, frame, sequence, offset, length, status, reserved = struct.unpack(
                ">8I", packet[:32])
            camera, stream = (info >> 16) & 255, (info >> 8) & 255
            self.assertIn(camera, (0, 1))
            self.assertEqual(stream, 2)
            self.assertEqual(sequence, sequences[camera])
            self.assertEqual((magic, offset, status, reserved), (0x4c555443, 0, 0, 0))
            self.assertEqual(length, len(packet) - 32)
            self.assertEqual(length % 72, 0)
            batch_size = length // 72
            self.assertIn(batch_size, range(1, 5))
            start = len(recovered[camera])
            self.assertLessEqual(start + batch_size, accepted)
            self.assertEqual(frames[start:start + batch_size], [frame] * batch_size,
                             "a V3 child batch crossed a frame boundary")
            body = b"".join(word_bytes(tuples(camera, group), 8)
                            for group in range(start, start + batch_size))
            self.assertEqual(packet,
                batched_header(camera, 2, frame, sequence, 0, len(body)) + body)
            for base in range(32, len(packet), 72):
                recovered[camera].append(struct.unpack(">9Q", packet[base:base + 72]))
            self.assertEqual(metadata, (len(packet), 0xc0a80102, 6001 + camera * 3))
            sequences[camera] += 1
            byte_counts[camera] += len(packet)
            packet_frames[camera].append(frame)
        for camera in range(2):
            self.assertEqual(recovered[camera],
                             [tuple(tuples(camera, group)) for group in range(accepted)])
            self.assertEqual(observed["final"][camera], {
                "committed_packets": sequences[camera], "sent_packets": sequences[camera],
                "bytes": byte_counts[camera], "next_sequence": sequences[camera],
            })
        return observed, packet_frames

    def test_old_eight_slots_overflow_on_a_legitimate_64_group_burst(self):
        observed, _ = self.run_burst(slots=8, groups=64, tuple_gap=2)
        self.assertEqual(observed["full_levels"], [8, 8])

    def test_64_slots_preserve_both_cameras_contiguous_and_arena_paced_bursts(self):
        for gap in (0, 2):
            with self.subTest(tuple_gap=gap):
                observed, _ = self.run_burst(slots=64, groups=64, tuple_gap=gap)
                self.assertEqual(observed["full_levels"], [64, 64])
                self.assertEqual(observed["releases"], [64, 64])

    def test_group_65_still_overflows_without_corrupting_the_first_64(self):
        observed, _ = self.run_burst(slots=64, groups=65)
        self.assertEqual(observed["full_levels"], [64, 64])
        self.assertEqual(observed["releases"], [64, 64])

    def test_queued_next_frame_groups_never_share_a_batch_with_the_previous_frame(self):
        frames = [FRAME] * 3 + [FRAME + 1] * 5
        _, packet_frames = self.run_burst(slots=64, groups=8, frames=frames)
        for camera in range(2):
            self.assertEqual(set(packet_frames[camera]), {FRAME, FRAME + 1})
            self.assertEqual(packet_frames[camera], sorted(packet_frames[camera]))


if __name__ == "__main__":
    unittest.main()
