"""Accepted centroid queues backpressure without losing measured geometry."""

import struct
import unittest

from migen.sim import passive, run_simulation

import test_packet as helper
import test_shared_packet as shared


class AcceptedCentroidTransport(unittest.TestCase):
    def test_full_queues_backpressure_without_loss_or_frame_mix(self):
        dut = shared.SharedHarness(
            child_slots=2, pose_slots=2, frame_words=16, fragment_words=16,
            compact_padding=True, batched_packets=True, accepted_centroids=True,
        )
        transport = dut.transport
        gate = {"ready": False}
        capture = helper.Capture(
            transport.source, ready=lambda cycle: gate["ready"] and cycle % 5 != 0,
        )
        done = [False, False]
        expected = {}
        stalls = [0, 0]

        @passive
        def monitor():
            while True:
                for camera in range(2):
                    child = transport.channels[camera * 3 + 1]
                    if (yield child.sink.valid) and not (yield child.sink.ready):
                        stalls[camera] += 1
                yield

        def producer(camera):
            child, pose = transport.channels[camera * 3 + 1:camera * 3 + 3]
            yield from shared.idle(8)
            for group in range(12):
                frame = 0x12340000 + camera * 256 + group
                words = [(1 << 32) | (camera << 24) | (group << 12) | (i * 17)
                         for i in range(9)]
                expected[camera, 2, frame] = helper.word_bytes(words, 8)
                for index, word in enumerate(words):
                    yield child.sink.valid.eq(1)
                    yield child.sink.data.eq(word)
                    yield child.sink.frame_id.eq(frame)
                    yield child.sink.end.eq(index == 8)
                    yield
                    for _ in range(4000):
                        if (yield child.sink.ready):
                            break
                        yield
                    else:
                        self.fail("centroid input did not recover")
                yield child.sink.valid.eq(0)
                yield child.sink.end.eq(0)
                words = shared.accepted_pose(0x10000 + camera * 256 + group)
                expected[camera, 3, frame] = helper.word_bytes(words, 4)
                yield from shared.send_pose(pose, words, frame, budget=4000)
            done[camera] = True

        def stimulus():
            yield from shared.configure(transport)
            yield from shared.idle(300)
            for camera in range(2):
                child = transport.channels[camera * 3 + 1]
                self.assertGreater(stalls[camera], 0)
                self.assertEqual((yield child.committed_level), 2)
                self.assertEqual((yield child.faults), 0)
            gate["ready"] = True
            for _ in range(15000):
                if all(done) and len(capture.packets) == 48 and (yield transport.drained):
                    break
                yield
            else:
                self.fail(f"did not drain all data: packets={len(capture.packets)}")
            for channel in transport.channels:
                self.assertEqual((yield channel.faults), 0)
                self.assertEqual((yield channel.fifo_level), 0)
            actual = {}
            for packet in capture.packets:
                header = struct.unpack(">8I", packet[:32])
                key = ((header[1] >> 16) & 255, (header[1] >> 8) & 255, header[2])
                self.assertEqual(header[1] >> 24, 3)
                self.assertEqual(header[5], len(packet) - 32)
                self.assertNotIn(key, actual)
                actual[key] = packet[32:]
            self.assertEqual(actual, expected)

        run_simulation(dut, [stimulus(), producer(0), producer(1), monitor(), capture.run()])


if __name__ == "__main__":
    unittest.main()
