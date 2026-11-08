"""Wire-level checks for the shared camera UDP serializer and raw queues."""

import struct
import unittest

from migen import Array, If, Module, Mux, Signal
from migen.sim import passive, run_simulation

from fpga.litex.lutcam.packet import SharedPacketTransport
from test_packet import Capture, child_group, header, idle, image_frame, word_bytes


class SharedHarness(Module):
    """Minimal external sequence service and event-accounting consumer."""

    def __init__(self, **kwargs):
        self.submodules.transport = transport = SharedPacketTransport(**kwargs)
        self.sequence_gate = Signal(reset=1)
        self.sequences = Array(Signal(32) for _ in transport.channels)
        self.committed = [Signal(32) for _ in transport.channels]
        self.sent = [Signal(32) for _ in transport.channels]
        self.byte_counts = [Signal(32) for _ in transport.channels]
        self.overflows = [Signal(32) for _ in transport.channels]
        self.protocol_errors = [Signal(32) for _ in transport.channels]
        self.rejections = [Signal(32) for _ in transport.channels]
        self.double_commit = [Signal() for _ in transport.channels]
        self.comb += [
            transport.sequence_valid.eq(transport.sequence_request & self.sequence_gate),
            transport.sequence_value.eq(self.sequences[transport.sequence_stream]),
        ]
        if kwargs.get("external_destination", False):
            self.lookup_ips = Array(Signal(32) for _ in transport.channels)
            self.lookup_ports = Array(Signal(16) for _ in transport.channels)
            self.lookup_poison = Signal()
            self.comb += [
                transport.sequence_destination_ip.eq(Mux(self.lookup_poison, 0xDEAD0BAD,
                                                        self.lookup_ips[transport.sequence_stream])),
                transport.sequence_destination_port.eq(Mux(self.lookup_poison, 54321,
                                                          self.lookup_ports[transport.sequence_stream])),
            ]
        for index, channel in enumerate(transport.channels):
            self.sync += [
                self.committed[index].eq(self.committed[index] + channel.commit_event),
                If(channel.commit_event == 2, self.double_commit[index].eq(1)),
                If(channel.sent_event,
                    self.sequences[index].eq(self.sequences[index] + 1),
                    self.sent[index].eq(self.sent[index] + 1),
                    self.byte_counts[index].eq(self.byte_counts[index] + channel.bytes_event),
                ),
                self.overflows[index].eq(self.overflows[index] + channel.overflow_event),
                self.protocol_errors[index].eq(self.protocol_errors[index] + channel.protocol_event),
                self.rejections[index].eq(self.rejections[index] + channel.reject_event),
            ]


def configure(transport):
    for index, channel in enumerate(transport.channels):
        yield channel.enable.eq(1)
        yield channel.capture_enable.eq(1)
        yield channel.destination_ip.eq(0xC0A80102)
        yield channel.destination_port.eq(6000 + index)
        yield channel.destination_update.eq(1)
    yield
    yield
    for channel in transport.channels:
        yield channel.destination_update.eq(0)
    yield


def accepted_pose(seed):
    words = [(seed + index) & 0xFFFFFFFF for index in range(22)]
    words[1] |= 0x80000000
    words[8] = 0x00000001
    return words


def rejected_pose(seed):
    words = [(seed + index) & 0xFFFFFFFF for index in range(8)]
    words[1] &= 0x7FFFFFFF
    return words


def send_pose(channel, words, frame_id, budget=2000):
    """Keep each native beat stable until accepted, with a finite timeout."""
    for index, word in enumerate(words):
        yield channel.sink.valid.eq(1)
        yield channel.sink.data.eq(word)
        yield channel.sink.frame_id.eq(frame_id)
        yield channel.sink.last.eq(index == len(words) - 1)
        yield
        for _ in range(budget):
            if (yield channel.sink.ready):
                break
            yield
        else:
            raise AssertionError("shared pose input failed to become ready")
    yield channel.sink.valid.eq(0)
    yield channel.sink.last.eq(0)


def wait_packets(capture, count, budget=3000):
    for _ in range(budget):
        if len(capture.packets) == count:
            yield from idle(4)
            return
        if len(capture.packets) > count:
            raise AssertionError("shared serializer emitted an unexpected packet")
        yield
    raise AssertionError(f"shared serializer emitted {len(capture.packets)} of {count} packets")


def image_packets(camera, words, frame_id, fragment_words, sequence=0):
    packets = []
    for start in range(0, len(words), fragment_words):
        fragment = words[start:start + fragment_words]
        packets.append(header(camera, 1, frame_id, sequence + len(packets), start * 2,
                              len(fragment) * 2, int(start + len(fragment) == len(words)))
                       + word_bytes(fragment, 2))
    return packets


def packet_key(packet):
    values = struct.unpack(">8I", packet[:32])
    return ((values[1] >> 16) & 0xFF, (values[1] >> 8) & 0xFF, values[3])


def compact_header(camera, stream, frame, sequence, offset, length, status=0):
    return struct.pack(
        ">8I", 0x4C555443, 0x00000020 | (camera << 16) | (stream << 8),
        frame, sequence, offset, length, status, 0,
    )


def compact_image_packets(camera, words, frame_id, frame_words, fragment_words, sequence=0):
    """Version two sends semantic words and a final marker, without zero fill."""
    packets = []
    for start in range(0, len(words), fragment_words):
        fragment = words[start:start + fragment_words]
        final = start + len(fragment) == len(words) and (
            len(fragment) < fragment_words or len(words) == frame_words)
        packets.append(compact_header(camera, 1, frame_id, sequence + len(packets),
                                      start * 2, len(fragment) * 2, int(final))
                       + word_bytes(fragment, 2))
    if len(words) < frame_words and len(words) % fragment_words == 0:
        packets.append(compact_header(camera, 1, frame_id, sequence + len(packets),
                                      len(words) * 2, 0, 1))
    return packets


class SharedPacketTests(unittest.TestCase):
    def test_shared_payload_prioritizes_contiguous_children_and_stalls_poses_safely(self):
        # Larger logical child capacity isolates RAM arbitration from expected
        # FIFO overflow: 200 consecutive tuples form 23 groups. Production
        # remains at eight child slots; this is a parameterized stress fixture.
        dut = SharedHarness(cameras=2, child_slots=32, pose_slots=2,
                            frame_words=16, fragment_words=16,
                            external_destination=True, external_context_held=True)
        transport = dut.transport
        child, pose = transport.channels[1:3]
        go, child_done, pose_done = Signal(), Signal(), Signal()
        readiness = {"enabled": False}
        capture = Capture(transport.source, ready=lambda cycle: readiness["enabled"] and cycle % 4 == 0)
        records = [0x100000000 | (index * 0x010203 & 0xFFFFFFFF) for index in range(200)]
        groups = [records[start:start + 9] for start in range(0, len(records), 9)]
        poses = [accepted_pose(0x83000000), accepted_pose(0xCA000020)]
        observations = {"child_words": 0, "child_stalls": 0, "pose_stalls_with_free_slot": 0}

        @passive
        def monitor():
            while True:
                if (yield child.sink.valid):
                    observations["child_words"] += 1
                    if not (yield child.sink.ready):
                        observations["child_stalls"] += 1
                    if ((yield pose.sink.valid) and not (yield pose.sink.ready)
                            and (yield pose.committed_level) < 2):
                        observations["pose_stalls_with_free_slot"] += 1
                yield

        def child_producer():
            while not (yield go):
                yield
            # Every clock carries a tuple, including across group boundaries.
            for index, record in enumerate(records):
                yield child.sink.valid.eq(1)
                yield child.sink.data.eq(record)
                yield child.sink.frame_id.eq(0xABCD0000 + index // 9)
                yield child.sink.end.eq(index % 9 == 8 or index == len(records) - 1)
                yield
            yield child.sink.valid.eq(0)
            yield child.sink.end.eq(0)
            yield child_done.eq(1)
            yield

        def pose_producer():
            while not (yield go):
                yield
            for index, words in enumerate(poses):
                yield from send_pose(pose, words, 0xB0000001 + index)
            yield from idle(4)
            self.assertEqual((yield pose.committed_level), 2)
            self.assertEqual(capture.packets, [])
            before = capture.cycles
            yield from send_pose(pose, rejected_pose(0x12345600), 0xB0000003)
            self.assertLessEqual(capture.cycles - before, 12)
            yield from idle(3)
            self.assertEqual((yield dut.rejections[2]), 1)
            self.assertEqual((yield pose.committed_level), 2)
            yield pose_done.eq(1)
            yield

        def stimulus():
            yield from configure(transport)
            for index in range(6):
                yield dut.lookup_ips[index].eq(0x0A000001 + index)
                yield dut.lookup_ports[index].eq(7100 + index)
            yield go.eq(1)
            for _ in range(3000):
                if (yield child_done) and (yield pose_done):
                    break
                yield
            else:
                self.fail("shared-RAM producers did not retire while Ethernet was stalled")
            self.assertEqual(capture.packets, [])
            self.assertEqual((yield child.committed_level), len(groups))
            self.assertEqual((yield pose.committed_level), 2)
            self.assertEqual(observations["child_words"], 200)
            self.assertEqual(observations["child_stalls"], 0)
            self.assertGreater(observations["pose_stalls_with_free_slot"], 100)
            for channel in transport.channels:
                self.assertEqual((yield channel.faults), 0)
            readiness["enabled"] = True
            yield from wait_packets(capture, len(groups) + len(poses), budget=8000)
            for channel in transport.channels:
                self.assertEqual((yield channel.faults), 0)
                self.assertEqual((yield channel.fifo_level), 0)
            self.assertEqual((yield dut.sent[1]), len(groups))
            self.assertEqual((yield dut.sent[2]), len(poses))
            self.assertEqual((yield dut.committed[1]), len(groups))
            self.assertEqual((yield dut.committed[2]), len(poses))
            self.assertEqual((yield dut.byte_counts[1]), sum(32 + 8 * len(group) for group in groups))
            self.assertEqual((yield dut.byte_counts[2]), 240)

        run_simulation(dut, [stimulus(), child_producer(), pose_producer(), monitor(), capture.run()])
        expected = {}
        for index, group in enumerate(groups):
            expected[(0, 2, index)] = (header(0, 2, 0xABCD0000 + index, index, 0, 8 * len(group))
                                        + word_bytes(group, 8))
        for index, words in enumerate(poses):
            expected[(0, 3, index)] = (header(0, 3, 0xB0000001 + index, index, 0, 88)
                                        + word_bytes(words, 4))
        self.assertEqual(len(capture.packets), len(expected))
        self.assertEqual({packet_key(packet): packet for packet in capture.packets}, expected)
        for packet, metadata in zip(capture.packets, capture.metadata):
            _, kind, _ = packet_key(packet)
            self.assertEqual(metadata, (len(packet), 0x0A000001 + kind - 1, 7100 + kind - 1))

    def test_external_destinations_latch_with_delayed_lookup_and_ignore_later_changes(self):
        self._check_external_destinations(provider_held=False)

    def test_provider_held_destinations_remain_stable_through_packet_stalls(self):
        self._check_external_destinations(provider_held=True)

    def _check_external_destinations(self, provider_held):
        dut = SharedHarness(cameras=2, image_slots=2, child_slots=2, pose_slots=2,
                            frame_words=16, fragment_words=16, external_destination=True,
                            external_context_held=provider_held)
        transport = dut.transport
        go = Signal()
        readiness = {"enabled": False}
        capture = Capture(transport.source, ready=lambda cycle: readiness["enabled"] and cycle % 4 == 0)
        expected = {}
        for camera in range(2):
            pixels = [0x1200 + camera * 0x100 + index for index in range(16)]
            children = [0x100000000 + camera * 0x100 + index for index in range(3)]
            pose = accepted_pose(0x81002000 + camera * 0x100)
            expected[(camera, 1, 0)] = image_packets(camera, pixels, 0xA1000000 + camera, 16)[0]
            expected[(camera, 2, 0)] = header(camera, 2, 0xA2000000 + camera, 0, 0, 24) + word_bytes(children, 8)
            expected[(camera, 3, 0)] = header(camera, 3, 0xA3000000 + camera, 0, 0, 88) + word_bytes(pose, 4)

        def producer(camera):
            while not (yield go):
                yield
            pixels = [0x1200 + camera * 0x100 + index for index in range(16)]
            children = [0x100000000 + camera * 0x100 + index for index in range(3)]
            yield from image_frame(transport.channels[camera * 3], pixels, 0xA1000000 + camera)
            yield from child_group(transport.channels[camera * 3 + 1], children, 0xA2000000 + camera)
            yield from send_pose(transport.channels[camera * 3 + 2],
                                 accepted_pose(0x81002000 + camera * 0x100), 0xA3000000 + camera)

        def lookup_service():
            while not (yield go):
                yield
            for packet_number in range(6):
                for _ in range(500):
                    if (yield transport.sequence_request):
                        break
                    yield
                else:
                    self.fail("external packet context lookup was never requested")
                selected = (yield transport.sequence_stream)
                yield dut.lookup_poison.eq(0)
                for _ in range(7):
                    self.assertEqual((yield transport.sequence_request), 1)
                    self.assertEqual((yield transport.sequence_stream), selected)
                    self.assertEqual((yield transport.source.valid), 0)
                    yield
                yield dut.sequence_gate.eq(1)
                for _ in range(10):
                    yield
                    if (yield transport.source.valid):
                        break
                else:
                    self.fail("serializer did not accept external packet context")
                yield dut.sequence_gate.eq(0)
                if not provider_held:
                    yield dut.lookup_poison.eq(1)
                # In latched mode, lookup buses may change immediately after
                # the response. Held mode instead keeps the provider context
                # valid through every accepted beat, including the last one.
                for _ in range(6):
                    yield
                    expected_ip = 0x0A001001 + selected if provider_held else 0xDEAD0BAD
                    expected_port = 7100 + selected if provider_held else 54321
                    self.assertEqual((yield transport.sequence_destination_ip), expected_ip)
                    self.assertEqual((yield transport.sequence_destination_port), expected_port)
                    self.assertEqual((yield transport.source.valid), 1)
                    self.assertEqual((yield transport.source.ready), 0)
                    self.assertEqual((yield transport.source.ip_address), 0x0A001001 + selected)
                    self.assertEqual((yield transport.source.dst_port), 7100 + selected)
                readiness["enabled"] = True
                yield from wait_packets(capture, packet_number + 1)
                readiness["enabled"] = False
                if provider_held:
                    # The last beat has been accepted. Context can now be
                    # recycled while the next request waits for its response.
                    yield dut.lookup_poison.eq(1)
                    yield
                    self.assertEqual((yield transport.source.valid), 0)

        def stimulus():
            yield from configure(transport)
            yield dut.sequence_gate.eq(0)
            for index in range(6):
                yield dut.lookup_ips[index].eq(0x0A001001 + index)
                yield dut.lookup_ports[index].eq(7100 + index)
            yield go.eq(1)
            yield from wait_packets(capture, 6, budget=3000)
            for index, channel in enumerate(transport.channels):
                self.assertEqual((yield channel.faults), 0)
                self.assertEqual((yield channel.fifo_level), 0)
                self.assertEqual((yield dut.sent[index]), 1)

        run_simulation(dut, [stimulus(), lookup_service(), producer(0), producer(1), capture.run()])
        self.assertEqual({packet_key(packet): packet for packet in capture.packets}, expected)
        for packet, metadata in zip(capture.packets, capture.metadata):
            camera, kind, _ = packet_key(packet)
            channel = camera * 3 + kind - 1
            self.assertEqual(metadata, (len(packet), 0x0A001001 + channel, 7100 + channel))

    def test_six_streams_preserve_packet_bytes_sequences_and_event_accounting(self):
        dut = SharedHarness(cameras=2, image_slots=2, child_slots=2, pose_slots=2,
                            frame_words=64, fragment_words=16)
        transport = dut.transport
        go = Signal()
        capture = Capture(transport.source, ready=lambda cycle: cycle >= 160 and cycle % 7 not in (0, 2, 3))
        expected = {}
        for camera in range(2):
            pixels = [0x1000 * (camera + 1) + n for n in range(5)] + [0] * 59
            for packet in image_packets(camera, pixels, 0x81230070 + camera, 16):
                expected[packet_key(packet)] = packet
            for sequence in range(2):
                children = [0x100000000 + camera * 0x100 + sequence * 16 + n for n in range(9)]
                pose = accepted_pose(0x80001000 + camera * 0x100 + sequence * 32)
                expected[(camera, 2, sequence)] = header(camera, 2, 0xABCD0080 + sequence, sequence, 0, 72) + word_bytes(children, 8)
                expected[(camera, 3, sequence)] = header(camera, 3, 0xCAFE0090 + sequence, sequence, 0, 88) + word_bytes(pose, 4)

        def image_producer(camera):
            while not (yield go):
                yield
            pixels = [0x1000 * (camera + 1) + n for n in range(5)] + [0] * 59
            yield from image_frame(transport.channels[camera * 3], pixels, 0x81230070 + camera, padding_start=5)

        def child_producer(camera):
            while not (yield go):
                yield
            for sequence in range(2):
                records = [0x100000000 + camera * 0x100 + sequence * 16 + n for n in range(9)]
                yield from child_group(transport.channels[camera * 3 + 1], records, 0xABCD0080 + sequence)

        def pose_producer(camera):
            while not (yield go):
                yield
            channel = transport.channels[camera * 3 + 2]
            yield from send_pose(channel, rejected_pose(0x12340000 + camera * 16), 89)
            for sequence in range(2):
                yield from send_pose(channel, accepted_pose(0x80001000 + camera * 0x100 + sequence * 32), 0xCAFE0090 + sequence)

        def stimulus():
            yield from configure(transport)
            yield go.eq(1)
            yield from wait_packets(capture, len(expected), budget=6000)
            for index, channel in enumerate(transport.channels):
                camera, kind = divmod(index, 3)
                products = [packet for key, packet in expected.items() if key[:2] == (camera, kind + 1)]
                self.assertEqual((yield channel.faults), 0)
                self.assertEqual((yield channel.fifo_level), 0)
                self.assertEqual((yield dut.sent[index]), len(products))
                self.assertEqual((yield dut.committed[index]), len(products))
                self.assertEqual((yield dut.byte_counts[index]), sum(map(len, products)))
                self.assertEqual((yield dut.rejections[index]), int(kind == 2))
                self.assertEqual((yield dut.overflows[index]), 0)
                self.assertEqual((yield dut.protocol_errors[index]), 0)

        generators = [stimulus(), capture.run()]
        for camera in range(2):
            generators += [image_producer(camera), child_producer(camera), pose_producer(camera)]
        run_simulation(dut, generators)
        self.assertEqual(len({packet_key(packet) for packet in capture.packets}), len(expected))
        self.assertEqual({packet_key(packet): packet for packet in capture.packets}, expected)
        self.assertEqual([packet_key(packet)[:2] for packet in capture.packets[:6]],
                         [(0, 1), (0, 2), (0, 3), (1, 1), (1, 2), (1, 3)])
        for packet, metadata in zip(capture.packets, capture.metadata):
            camera, kind, _ = packet_key(packet)
            self.assertEqual(metadata, (len(packet), 0xC0A80102, 6000 + camera * 3 + kind - 1))

    def test_delayed_sequence_service_holds_header_and_wraps_per_stream(self):
        dut = SharedHarness(cameras=2, frame_words=16, fragment_words=4, child_slots=2)
        transport = dut.transport
        channel = transport.channels[1]
        capture = Capture(transport.source)

        def stimulus():
            yield from configure(transport)
            yield dut.sequence_gate.eq(0)
            yield dut.sequences[1].eq(0xFFFFFFFE)
            yield from child_group(channel, [0x100000011], 41)
            yield from child_group(channel, [0x100000022], 42)
            yield from idle(30)
            self.assertEqual((yield transport.sequence_request), 1)
            self.assertEqual((yield transport.sequence_stream), 1)
            self.assertEqual((yield transport.source.valid), 0)
            self.assertEqual(capture.packets, [])
            yield dut.sequence_gate.eq(1)
            yield from wait_packets(capture, 2)
            yield from child_group(channel, [0x100000033], 43)
            yield from wait_packets(capture, 3)
            self.assertEqual((yield dut.sequences[1]), 1)
            self.assertEqual((yield dut.sequences[0]), 0)
            self.assertEqual((yield dut.sent[1]), 3)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(capture.packets, [
            header(0, 2, frame, sequence, 0, 8) + word_bytes([word], 8)
            for frame, sequence, word in ((41, 0xFFFFFFFE, 0x100000011),
                                         (42, 0xFFFFFFFF, 0x100000022),
                                         (43, 0, 0x100000033))])

    def test_destination_changes_wait_for_that_channel_to_drain(self):
        dut = SharedHarness(cameras=2, frame_words=16, fragment_words=4, child_slots=2)
        transport = dut.transport
        channel = transport.channels[1]
        capture = Capture(transport.source, ready=lambda cycle: cycle >= 90 and cycle % 4 != 0)

        def stimulus():
            yield from configure(transport)
            yield from child_group(channel, [0x100000001], 10)
            yield channel.destination_ip.eq(0x0A000007)
            yield channel.destination_port.eq(7777)
            yield channel.destination_update.eq(1)
            yield from child_group(channel, [0x100000002], 11)
            yield from idle(20)
            self.assertEqual((yield channel.active_destination_ip), 0xC0A80102)
            self.assertEqual((yield channel.active_destination_port), 6001)
            yield from wait_packets(capture, 2)
            yield from idle(4)
            self.assertEqual((yield channel.active_destination_ip), 0x0A000007)
            self.assertEqual((yield channel.active_destination_port), 7777)
            yield channel.destination_update.eq(0)
            yield from child_group(channel, [0x100000003], 12)
            yield from wait_packets(capture, 3)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(capture.metadata, [(40, 0xC0A80102, 6001), (40, 0xC0A80102, 6001), (40, 0x0A000007, 7777)])

    def test_full_image_queue_consumes_physical_padding_and_preserves_commits(self):
        dut = SharedHarness(cameras=2, image_slots=2, frame_words=16, fragment_words=4)
        transport = dut.transport
        channel = transport.channels[0]
        capture = Capture(transport.source, ready=lambda cycle: cycle >= 120)

        def stimulus():
            yield from configure(transport)
            yield from image_frame(channel, [0] * 16, 20, padding_start=0)
            yield from image_frame(channel, [0] * 16, 21, padding_start=0)
            yield from idle(3)
            self.assertEqual((yield channel.faults), 0)
            self.assertEqual((yield channel.committed_level), 2)
            self.assertEqual((yield channel.fifo_level), 2)
            yield from image_frame(channel, [0] * 16, 22, padding_start=0)
            yield from idle(3)
            self.assertEqual((yield channel.faults) & 1, 1)
            self.assertEqual((yield channel.fault_sticky), 1)
            yield from wait_packets(capture, 8)
            self.assertEqual((yield dut.overflows[0]), 1)
            self.assertEqual((yield dut.committed[0]), 8)
            self.assertEqual((yield dut.sent[0]), 8)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(capture.packets, image_packets(0, [0] * 16, 20, 4) + image_packets(0, [0] * 16, 21, 4, sequence=4))

    def test_zero_tail_can_drain_before_physical_input_ends_then_next_frame_follows(self):
        dut = SharedHarness(cameras=2, image_slots=4, frame_words=64, fragment_words=16)
        transport = dut.transport
        channel = transport.channels[0]
        capture = Capture(transport.source)
        first = [0x1001, 0x2002, 0x3003] + [0] * 61
        second = [0x7000 + index for index in range(64)]

        def stimulus():
            yield from configure(transport)
            yield from image_frame(channel, first[:4], 55, padding_start=3)
            yield from wait_packets(capture, 4)
            self.assertEqual((yield channel.fifo_level), 0)
            self.assertEqual((yield channel.committed_level), 0)
            for _ in range(60):
                yield channel.sink.valid.eq(1)
                yield channel.sink.sof.eq(0)
                yield channel.sink.padding.eq(1)
                yield channel.sink.data.eq(0)
                yield
            yield channel.sink.valid.eq(0)
            yield channel.sink.padding.eq(0)
            yield
            yield from image_frame(channel, second, 56)
            yield from wait_packets(capture, 8)
            self.assertEqual((yield channel.faults), 0)
            self.assertEqual((yield dut.committed[0]), 8)
            self.assertEqual((yield dut.byte_counts[0]), 512)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(capture.packets, image_packets(0, first, 55, 16) + image_packets(0, second, 56, 16, sequence=4))

    def test_late_padding_fault_retains_published_promise_and_clear_recovers(self):
        dut = SharedHarness(cameras=2, image_slots=4, frame_words=16, fragment_words=4)
        transport = dut.transport
        channel = transport.channels[0]
        capture = Capture(transport.source, ready=lambda cycle: cycle % 3 != 1)
        bad_physical = [0x1234, 0, 1] + [0] * 13
        promised = [0x1234] + [0] * 15
        next_frame = [0x8000 + index for index in range(16)]

        def stimulus():
            yield from configure(transport)
            yield from image_frame(channel, bad_physical, 60, padding_start=1)
            yield from wait_packets(capture, 4)
            self.assertEqual((yield channel.faults), 2)
            self.assertEqual((yield dut.protocol_errors[0]), 1)
            yield channel.clear.eq(1)
            yield
            yield channel.clear.eq(0)
            yield
            self.assertEqual((yield channel.faults), 0)
            yield from image_frame(channel, next_frame, 61)
            yield from wait_packets(capture, 8)
            self.assertEqual((yield channel.faults), 0)
            self.assertEqual((yield dut.protocol_errors[0]), 1)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(capture.packets, image_packets(0, promised, 60, 4) + image_packets(0, next_frame, 61, 4, sequence=4))

    def test_new_descriptor_and_virtual_continuation_count_two_commits(self):
        dut = SharedHarness(cameras=2, image_slots=2, frame_words=16, fragment_words=4)
        transport = dut.transport
        channel = transport.channels[0]
        readiness = {"enabled": False}
        capture = Capture(transport.source, ready=lambda cycle: readiness["enabled"] and cycle % 4 == 0)

        def stimulus():
            yield from configure(transport)
            yield from image_frame(channel, [0] * 16, 25, padding_start=0)
            readiness["enabled"] = True
            for _ in range(200):
                if ((yield transport.source.valid) and (yield transport.source.last)
                        and not (yield transport.source.ready) and capture.cycles % 4 == 0):
                    break
                yield
            else:
                self.fail("first virtual fragment failed to reach its last beat")
            # The first word commits the next frame's promised zero tail on
            # the same edge that the current descriptor advances a fragment.
            yield from image_frame(channel, [0] * 16, 26, padding_start=0)
            yield from wait_packets(capture, 8)
            self.assertEqual((yield dut.double_commit[0]), 1)
            self.assertEqual((yield dut.committed[0]), 8)
            self.assertEqual((yield dut.sent[0]), 8)
            self.assertEqual((yield channel.faults), 0)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(capture.packets, image_packets(0, [0] * 16, 25, 4) + image_packets(0, [0] * 16, 26, 4, sequence=4))

    def test_full_pose_queue_consumes_rejection_then_stalls_accepted_result(self):
        dut = SharedHarness(cameras=2, pose_slots=2, frame_words=16, fragment_words=4)
        transport = dut.transport
        channel = transport.channels[2]
        capture = Capture(transport.source, ready=lambda cycle: cycle >= 320 and cycle % 3 != 1)
        poses = [accepted_pose(0xA0001000 + index * 32) for index in range(3)]

        def stimulus():
            yield from configure(transport)
            for index in range(2):
                yield from send_pose(channel, poses[index], 100 + index)
            yield from idle(3)
            self.assertEqual((yield channel.committed_level), 2)
            before = capture.cycles
            yield from send_pose(channel, rejected_pose(0x12340000), 102)
            self.assertLessEqual(capture.cycles - before, 12)
            yield from idle(2)
            self.assertEqual((yield dut.rejections[2]), 1)
            self.assertEqual((yield channel.committed_level), 2)
            before = capture.cycles
            yield from send_pose(channel, poses[2], 103)
            self.assertGreater(capture.cycles - before, 100)
            yield from wait_packets(capture, 3)
            self.assertEqual((yield channel.faults), 0)
            self.assertEqual((yield dut.committed[2]), 3)
            self.assertEqual((yield dut.sent[2]), 3)
            self.assertEqual((yield dut.byte_counts[2]), 360)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(capture.packets, [header(0, 3, frame, sequence, 0, 88) + word_bytes(pose, 4)
                                         for sequence, (frame, pose) in enumerate(zip((100, 101, 103), poses))])

    def test_child_fault_is_local_and_clear_preserves_sequence(self):
        for malformed in ("oversize", "frame_change"):
            with self.subTest(malformed=malformed):
                dut = SharedHarness(cameras=2, child_slots=2, frame_words=16, fragment_words=4)
                transport = dut.transport
                channel, other = transport.channels[1], transport.channels[4]
                capture = Capture(transport.source, ready=lambda cycle: cycle >= 80)

                def stimulus():
                    yield from configure(transport)
                    yield from child_group(channel, [0x100000011], 7)
                    if malformed == "oversize":
                        yield from child_group(channel, list(range(10)), 8)
                    else:
                        yield channel.sink.valid.eq(1)
                        yield channel.sink.data.eq(3)
                        yield channel.sink.frame_id.eq(8)
                        yield channel.sink.end.eq(0)
                        yield
                        yield channel.sink.frame_id.eq(9)
                        yield channel.sink.end.eq(1)
                        yield
                        yield channel.sink.valid.eq(0)
                    yield from child_group(other, [0x1ABCDEF01], 10)
                    yield from wait_packets(capture, 2)
                    self.assertEqual((yield channel.faults), 2)
                    self.assertEqual((yield other.faults), 0)
                    self.assertEqual((yield dut.protocol_errors[1]), 1)
                    yield channel.clear.eq(1)
                    yield
                    yield channel.clear.eq(0)
                    yield
                    yield from child_group(channel, [0x100000022], 11)
                    yield from wait_packets(capture, 3)
                    self.assertEqual((yield channel.faults), 0)

                run_simulation(dut, [stimulus(), capture.run()])
                expected = [header(0, 2, 7, 0, 0, 8) + word_bytes([0x100000011], 8),
                            header(1, 2, 10, 0, 0, 8) + word_bytes([0x1ABCDEF01], 8),
                            header(0, 2, 11, 1, 0, 8) + word_bytes([0x100000022], 8)]
                self.assertEqual({packet_key(packet): packet for packet in capture.packets},
                                 {packet_key(packet): packet for packet in expected})

    def test_reset_of_drained_camera_preserves_other_camera_pending_packet(self):
        dut = SharedHarness(cameras=2, frame_words=16, fragment_words=4)
        transport = dut.transport
        channel, other = transport.channels[1], transport.channels[4]
        capture = Capture(transport.source)

        def stimulus():
            yield from configure(transport)
            yield dut.sequence_gate.eq(0)
            yield from child_group(channel, list(range(10)), 1)
            yield from child_group(other, [0x1DEADBEEF], 2)
            yield from idle(4)
            self.assertEqual((yield channel.faults), 2)
            self.assertEqual((yield channel.fifo_level), 0)
            self.assertEqual((yield other.committed_level), 1)
            yield transport.camera_reset[0].eq(1)
            yield
            yield transport.camera_reset[0].eq(0)
            yield from idle(3)
            self.assertEqual((yield channel.faults), 0)
            self.assertEqual((yield other.committed_level), 1)
            yield channel.destination_update.eq(1)
            yield
            yield channel.destination_update.eq(0)
            yield dut.sequence_gate.eq(1)
            yield from wait_packets(capture, 1)
            yield from child_group(channel, [0x100000001], 3)
            yield from wait_packets(capture, 2)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(capture.packets, [header(1, 2, 2, 0, 0, 8) + word_bytes([0x1DEADBEEF], 8),
                                          header(0, 2, 3, 0, 0, 8) + word_bytes([0x100000001], 8)])


class CompactPaddingTests(unittest.TestCase):
    def test_compact_image_boundaries_preserve_exact_bytes_and_final_masks(self):
        for prefix in (0, 1, 2, 511, 512, 513, 1024):
            with self.subTest(semantic_words=prefix):
                dut = SharedHarness(cameras=2, compact_padding=True,
                                    frame_words=1024, fragment_words=512)
                transport = dut.transport
                channel = transport.channels[0]
                capture = Capture(transport.source,
                                  ready=lambda cycle: cycle >= 1200 and cycle % 7 not in (1, 4))
                semantic = [(0x31F * index + 0x1234) & 0xFFFF for index in range(prefix)]
                expected = compact_image_packets(0, semantic, 0xABC00100 + prefix, 1024, 512)
                final_masks = []

                @passive
                def monitor():
                    while True:
                        if (yield transport.source.valid) and (yield transport.source.ready):
                            if (yield transport.source.last):
                                final_masks.append((yield transport.source.last_be))
                            else:
                                self.assertEqual((yield transport.source.last_be), 0)
                        yield

                def stimulus():
                    yield from configure(transport)
                    yield from image_frame(channel, semantic + [0] * (1024 - prefix),
                                           0xABC00100 + prefix,
                                           padding_start=prefix if prefix < 1024 else None)
                    yield from wait_packets(capture, len(expected), budget=5000)
                    self.assertEqual((yield channel.faults), 0)
                    self.assertEqual((yield channel.fifo_level), 0)
                    self.assertEqual((yield channel.configuration_idle), 1)
                    self.assertEqual((yield dut.committed[0]), len(expected))
                    self.assertEqual((yield dut.sent[0]), len(expected))
                    self.assertEqual((yield dut.byte_counts[0]), sum(map(len, expected)))

                run_simulation(dut, [stimulus(), capture.run(), monitor()])
                self.assertEqual(capture.packets, expected)
                self.assertEqual(final_masks, [2 if len(packet) % 4 == 2 else 8 for packet in expected])
                self.assertEqual(capture.metadata, [(len(packet), 0xC0A80102, 6000) for packet in expected])

    def test_compact_back_to_back_frames_preserve_stream_sequences_and_accounting(self):
        dut = SharedHarness(cameras=2, image_slots=8, compact_padding=True,
                            frame_words=64, fragment_words=16)
        transport = dut.transport
        channel = transport.channels[0]
        capture = Capture(transport.source, ready=lambda cycle: cycle >= 320 and cycle % 4 == 0)
        frames = [(40, []), (41, [0x1020]), (42, list(range(16))),
                  (43, [(0x80A0 + index) & 0xFFFF for index in range(64)])]
        expected = []
        for frame, semantic in frames:
            expected += compact_image_packets(0, semantic, frame, 64, 16, len(expected))

        @passive
        def accounting():
            while True:
                self.assertEqual((yield dut.committed[0]),
                                 (yield dut.sent[0]) + (yield channel.committed_level))
                yield

        def stimulus():
            yield from configure(transport)
            for frame, semantic in frames:
                yield from image_frame(channel, semantic + [0] * (64 - len(semantic)), frame,
                                       padding_start=len(semantic) if len(semantic) < 64 else None)
            yield from wait_packets(capture, len(expected), budget=4000)
            self.assertEqual((yield channel.faults), 0)
            self.assertEqual((yield channel.fifo_level), 0)
            self.assertEqual((yield dut.committed[0]), len(expected))
            self.assertEqual((yield dut.sent[0]), len(expected))
            self.assertEqual((yield dut.byte_counts[0]), sum(map(len, expected)))
            self.assertEqual((yield dut.sequences[0]), len(expected))

        run_simulation(dut, [stimulus(), accounting(), capture.run()])
        self.assertEqual(capture.packets, expected)

    def test_compact_six_streams_use_version_two_and_export_only_accepted_poses(self):
        dut = SharedHarness(cameras=2, image_slots=2, child_slots=2, pose_slots=2,
                            compact_padding=True, frame_words=32, fragment_words=16)
        transport = dut.transport
        go = Signal()
        capture = Capture(transport.source, ready=lambda cycle: cycle >= 180 and cycle % 4 == 0)
        expected = {}
        for camera in range(2):
            semantic = [0x2000 * (camera + 1) + index for index in range(17)]
            for packet in compact_image_packets(camera, semantic, 0xA100 + camera, 32, 16):
                expected[packet_key(packet)] = packet
            children = [0x100000000 | (camera << 16) | index for index in range(9)]
            pose = accepted_pose(0x83000100 + camera * 32)
            expected[(camera, 2, 0)] = compact_header(camera, 2, 0xB100 + camera, 0, 0, 72) + word_bytes(children, 8)
            expected[(camera, 3, 0)] = compact_header(camera, 3, 0xC100 + camera, 0, 0, 88) + word_bytes(pose, 4)

        def producer(camera):
            while not (yield go):
                yield
            semantic = [0x2000 * (camera + 1) + index for index in range(17)]
            yield from image_frame(transport.channels[camera * 3], semantic + [0] * 15,
                                   0xA100 + camera, padding_start=17)
            yield from child_group(transport.channels[camera * 3 + 1],
                                   [0x100000000 | (camera << 16) | index for index in range(9)],
                                   0xB100 + camera)
            yield from send_pose(transport.channels[camera * 3 + 2], rejected_pose(0x1000), 0xC000 + camera)
            yield from send_pose(transport.channels[camera * 3 + 2],
                                accepted_pose(0x83000100 + camera * 32), 0xC100 + camera)

        def stimulus():
            yield from configure(transport)
            yield go.eq(1)
            yield from wait_packets(capture, len(expected), budget=4000)
            for index, channel in enumerate(transport.channels):
                camera, kind = divmod(index, 3)
                products = [packet for key, packet in expected.items() if key[:2] == (camera, kind + 1)]
                self.assertEqual((yield channel.faults), 0)
                self.assertEqual((yield channel.fifo_level), 0)
                self.assertEqual((yield dut.committed[index]), len(products))
                self.assertEqual((yield dut.sent[index]), len(products))
                self.assertEqual((yield dut.byte_counts[index]), sum(map(len, products)))
                self.assertEqual((yield dut.rejections[index]), int(kind == 2))

        run_simulation(dut, [stimulus(), producer(0), producer(1), capture.run()])
        self.assertEqual(len(capture.packets), len(expected))
        self.assertEqual({packet_key(packet): packet for packet in capture.packets}, expected)
        for packet, metadata in zip(capture.packets, capture.metadata):
            camera, kind, _ = packet_key(packet)
            self.assertEqual(metadata, (len(packet), 0xC0A80102, 6000 + camera * 3 + kind - 1))

    def test_compact_published_tail_still_validates_remaining_physical_words(self):
        for violation in ("nonzero_padding", "withdrawn_padding", "early_sof"):
            with self.subTest(violation=violation):
                dut = SharedHarness(cameras=2, compact_padding=True,
                                    frame_words=16, fragment_words=8)
                transport = dut.transport
                channel = transport.channels[0]
                capture = Capture(transport.source, ready=lambda cycle: cycle >= 40 and cycle % 3 != 1)

                def stimulus():
                    yield from configure(transport)
                    for index in range(16):
                        yield channel.sink.valid.eq(1)
                        yield channel.sink.data.eq(0x1234 if index == 0 else
                                                  int(violation == "nonzero_padding" and index == 2))
                        yield channel.sink.frame_id.eq(83)
                        yield channel.sink.sof.eq(index == 0 or (violation == "early_sof" and index == 2))
                        yield channel.sink.padding.eq(index >= 1 and
                                                     not (violation == "withdrawn_padding" and index == 2))
                        yield
                    yield channel.sink.valid.eq(0)
                    yield channel.sink.sof.eq(0)
                    yield channel.sink.padding.eq(0)
                    yield from wait_packets(capture, 1)
                    self.assertEqual((yield channel.faults), 2)
                    self.assertGreaterEqual((yield dut.protocol_errors[0]), 1)
                    yield channel.clear.eq(1)
                    yield
                    yield channel.clear.eq(0)
                    yield
                    yield from image_frame(channel, list(range(16)), 84)
                    yield from wait_packets(capture, 3)
                    self.assertEqual((yield channel.faults), 0)
                    self.assertEqual((yield dut.sent[0]), 3)
                    self.assertEqual((yield dut.byte_counts[0]), 130)

                run_simulation(dut, [stimulus(), capture.run()])
                self.assertEqual(capture.packets,
                                 compact_image_packets(0, [0x1234], 83, 16, 8)
                                 + compact_image_packets(0, list(range(16)), 84, 16, 8, sequence=1))


def batched_header(camera, stream, frame, sequence, offset, length):
    return struct.pack(
        ">8I", 0x4C555443, 0x00000020 | (camera << 16) | (stream << 8),
        frame, sequence, offset, length, 0, 0,
    )


def batched_image_packets(camera, words, frame, fragment_words, sequence=0):
    return [batched_header(camera, 1, frame, sequence + index, start * 2,
                           len(words[start:start + fragment_words]) * 2)
            + word_bytes(words[start:start + fragment_words], 2)
            for index, start in enumerate(range(0, len(words), fragment_words))]


class BatchedPacketTests(unittest.TestCase):
    def test_batched_ring_wrap_refill_and_stalled_pose_record_boundary_preserve_payloads(self):
        dut = SharedHarness(child_slots=4, pose_slots=2, compact_padding=True, batched_packets=True,
                            frame_words=16, fragment_words=16)
        transport = dut.transport
        child, pose = transport.channels[1:3]
        readiness = {"enabled": False}
        observation = {"child_stalled": False, "pose_stalled": False,
                       "simultaneous_commit_release": 0, "child_releases": 0, "pose_releases": 0}
        refill = Signal()
        capture = Capture(transport.source, ready=lambda cycle: readiness["enabled"])
        groups = [[0x100000000 + group * 0x1010 + index for index in range(9)] for group in range(8)]
        poses = [accepted_pose(0x86001000 + index * 32) for index in range(2)]

        @passive
        def monitor():
            child_beats = child_packets = pose_beats = pose_packets = 0
            while True:
                valid, ready = (yield transport.source.valid), (yield transport.source.ready)
                port = (yield transport.source.src_port)
                if valid and ready:
                    if port == 6001:
                        child_beats += 1
                        if child_packets == 0 and child_beats == 43:
                            readiness["enabled"] = False
                        if (yield transport.source.last):
                            child_packets += 1
                            child_beats = 0
                    if port == 6002:
                        pose_beats += 1
                        if pose_packets == 0 and pose_beats == 29:
                            readiness["enabled"] = False
                        if (yield transport.source.last):
                            pose_packets += 1
                            pose_beats = 0
                elif valid and not ready:
                    if port == 6001 and child_packets == 0 and child_beats == 43:
                        observation["child_stalled"] = True
                    if port == 6002 and pose_packets == 0 and pose_beats == 29:
                        observation["pose_stalled"] = True
                        self.assertEqual((yield pose.release), 0)
                if (yield child.release):
                    observation["child_releases"] += 1
                    if (yield child._commit):
                        observation["simultaneous_commit_release"] += 1
                if (yield pose.release):
                    observation["pose_releases"] += 1
                yield

        @passive
        def refill_children():
            while not (yield refill):
                yield
            for group in groups[5:]:
                while (yield child.committed_level) >= 4:
                    yield
                yield from child_group(child, group, 10)
                yield

        def stimulus():
            yield from configure(transport)
            yield from image_frame(transport.channels[0], list(range(16)), 900)
            for _ in range(100):
                if (yield transport.source.valid):
                    break
                yield
            else:
                self.fail("image blocker did not start")
            for group in groups[:4]:
                yield from child_group(child, group, 10)
            for words in poses:
                yield from send_pose(pose, words, 20)
            readiness["enabled"] = True
            for _ in range(1000):
                if observation["child_stalled"]:
                    break
                yield
            else:
                self.fail("second child-record boundary was not stalled")
            self.assertEqual((yield child.committed_level), 3)
            # Slot zero has been released. Reuse it and align the new record's
            # commit with the next old record's release to exercise pointer
            # wrap, RAM reuse, and simultaneous occupancy updates together.
            for index, value in enumerate(groups[4]):
                yield child.sink.valid.eq(1)
                yield child.sink.data.eq(value)
                yield child.sink.frame_id.eq(10)
                yield child.sink.end.eq(index == 8)
                if index == 8:
                    readiness["enabled"] = True
                yield
            yield child.sink.valid.eq(0)
            yield child.sink.end.eq(0)
            yield
            yield refill.eq(1)
            for _ in range(1000):
                if observation["pose_stalled"]:
                    break
                yield
            else:
                self.fail("first pose-record boundary was not stalled")
            self.assertEqual((yield pose.committed_level), 2)
            self.assertEqual((yield dut.sent[2]), 0)
            for _ in range(9):
                self.assertEqual((yield transport.source.valid), 1)
                self.assertEqual((yield transport.source.ready), 0)
                self.assertEqual((yield transport.source.last), 0)
                self.assertEqual((yield pose.release), 0)
                yield
            readiness["enabled"] = True
            yield from wait_packets(capture, 4)
            self.assertGreaterEqual(observation["simultaneous_commit_release"], 1)
            self.assertEqual(observation["child_releases"], 8)
            self.assertEqual(observation["pose_releases"], 2)
            self.assertEqual((yield child.faults), 0)
            self.assertEqual((yield pose.faults), 0)
            self.assertEqual((yield child.fifo_level), 0)
            self.assertEqual((yield pose.fifo_level), 0)
            self.assertLessEqual((yield child.high_water), 4)
            self.assertEqual((yield dut.committed[1]), 2)
            self.assertEqual((yield dut.sent[1]), 2)
            self.assertEqual((yield dut.byte_counts[1]), 640)
            self.assertEqual((yield dut.committed[2]), 1)
            self.assertEqual((yield dut.sent[2]), 1)
            self.assertEqual((yield dut.byte_counts[2]), 208)

        run_simulation(dut, [stimulus(), refill_children(), monitor(), capture.run()])
        expected = [batched_image_packets(0, list(range(16)), 900, 16)[0]]
        for sequence, offset in enumerate((0, 4)):
            body = b"".join(word_bytes(group, 8) for group in groups[offset:offset + 4])
            expected.append(batched_header(0, 2, 10, sequence, 0, len(body)) + body)
        body = b"".join(word_bytes(words, 4) for words in poses)
        expected.append(batched_header(0, 3, 20, 0, 0, len(body)) + body)
        self.assertEqual({packet_key(packet): packet for packet in capture.packets},
                         {packet_key(packet): packet for packet in expected})

    def test_batched_reset_immediately_after_native_commit_cannot_select_cleared_record(self):
        for kind in (0, 1, 2):
            with self.subTest(kind=kind):
                dut = SharedHarness(compact_padding=True, batched_packets=True,
                                    frame_words=16, fragment_words=16)
                transport = dut.transport
                capture = Capture(transport.source)
                words = list(range(16)) if kind == 0 else (
                    [0x100000000 + index for index in range(9)] if kind == 1 else accepted_pose(0x85000000))

                def produce(camera, frame):
                    channel = transport.channels[camera * 3 + kind]
                    if kind == 0:
                        yield from image_frame(channel, words, frame)
                    elif kind == 1:
                        yield from child_group(channel, words, frame)
                    else:
                        yield from send_pose(channel, words, frame)

                def stimulus():
                    yield from configure(transport)
                    yield from produce(0, 100)
                    # No idle clock between native commit and reset: the
                    # serializer must exclude this queue from its IDLE choice
                    # on the same edge that the queue is being cleared.
                    yield transport.camera_reset[0].eq(1)
                    yield
                    yield transport.camera_reset[0].eq(0)
                    yield from idle(60)
                    self.assertEqual(capture.packets, [])
                    self.assertEqual((yield transport.sequence_request), 0)
                    self.assertEqual((yield transport.channels[kind].committed_level), 0)
                    self.assertEqual((yield transport.channels[kind].faults), 0)
                    yield from produce(1, 101)
                    yield from produce(0, 102)
                    yield from wait_packets(capture, 2)
                    for camera in range(2):
                        channel = transport.channels[camera * 3 + kind]
                        self.assertEqual((yield channel.faults), 0)
                        self.assertEqual((yield channel.committed_level), 0)
                        self.assertEqual((yield dut.sent[camera * 3 + kind]), 1)

                run_simulation(dut, [stimulus(), capture.run()])
                body = word_bytes(words, (2, 8, 4)[kind])
                expected = [batched_header(camera, kind + 1, frame, 0, 0, len(body)) + body
                            for camera, frame in ((1, 101), (0, 102))]
                self.assertEqual({packet_key(packet): packet for packet in capture.packets},
                                 {packet_key(packet): packet for packet in expected})

    def test_batched_image_scenes_have_no_empty_final_packets_or_final_flag(self):
        dut = SharedHarness(compact_padding=True, batched_packets=True,
                            frame_words=64, fragment_words=16)
        transport = dut.transport
        image = transport.channels[0]
        capture = Capture(transport.source, ready=lambda cycle: cycle >= 320 and cycle % 4 == 0)
        frames = [(20, []), (21, list(range(16))),
                  (22, [0x2030 + index for index in range(17)]),
                  (23, [0x4050 + index for index in range(64)])]
        expected = []
        for frame, words in frames:
            expected += batched_image_packets(0, words, frame, 16, len(expected))

        @passive
        def accounting():
            while True:
                self.assertEqual((yield dut.committed[0]),
                                 (yield dut.sent[0]) + (yield image.counter_committed_level))
                self.assertEqual((yield image.counter_committed_level), (yield image.committed_level))
                yield

        def stimulus():
            yield from configure(transport)
            for frame, words in frames:
                yield from image_frame(image, words + [0] * (64 - len(words)), frame,
                                       padding_start=len(words) if len(words) < 64 else None)
            yield from wait_packets(capture, len(expected), budget=4000)
            self.assertEqual((yield image.faults), 0)
            self.assertEqual((yield image.fifo_level), 0)
            self.assertEqual((yield dut.committed[0]), len(expected))
            self.assertEqual((yield dut.sent[0]), len(expected))
            self.assertEqual((yield dut.sequences[0]), len(expected))
            self.assertEqual((yield dut.byte_counts[0]), sum(map(len, expected)))

        run_simulation(dut, [stimulus(), accounting(), capture.run()])
        self.assertEqual(capture.packets, expected)
        self.assertTrue(all(len(packet) > 32 for packet in capture.packets))
        self.assertTrue(all(struct.unpack_from(">I", packet, 24)[0] == 0 for packet in capture.packets))

    def test_batched_empty_padding_needs_no_slot_at_exact_boundary_or_new_sof(self):
        dut = SharedHarness(image_slots=8, compact_padding=True, batched_packets=True,
                            frame_words=144, fragment_words=16)
        transport = dut.transport
        image = transport.channels[0]
        readiness = {"enabled": False}
        capture = Capture(transport.source, ready=lambda cycle: readiness["enabled"] and cycle % 3 != 1)
        words = [0x5000 + index for index in range(128)]
        expected = batched_image_packets(0, words, 30, 16)

        def stimulus():
            yield from configure(transport)
            yield from image_frame(image, words + [0] * 16, 30, padding_start=128)
            yield from idle(4)
            self.assertEqual((yield image.committed_level), 8)
            self.assertEqual((yield image.faults), 0)
            yield from image_frame(image, [0] * 144, 31, padding_start=0)
            yield from idle(4)
            self.assertEqual((yield image.committed_level), 8)
            self.assertEqual((yield image.fifo_level), 8)
            self.assertEqual((yield image.faults), 0)
            self.assertEqual((yield dut.committed[0]), 8)
            self.assertEqual((yield dut.overflows[0]), 0)
            readiness["enabled"] = True
            yield from wait_packets(capture, 8, budget=4000)
            self.assertEqual((yield image.fifo_level), 0)
            self.assertEqual((yield image.configuration_idle), 1)
            self.assertEqual((yield dut.byte_counts[0]), sum(map(len, expected)))

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(capture.packets, expected)

    def test_batched_six_stream_batches_obey_limits_frames_and_datagram_accounting(self):
        dut = SharedHarness(compact_padding=True, batched_packets=True,
                            frame_words=16, fragment_words=16)
        transport = dut.transport
        readiness = {"enabled": False}
        capture = Capture(transport.source,
                          ready=lambda cycle: readiness["enabled"] and cycle % 5 in (0, 2))
        image_words = [[0x1000 * (camera + 1) + index for index in range(16)] for camera in range(2)]
        children = [[[(camera << 28) | (group << 16) | (0x100000000 + index)
                       for index in range(9)] for group in range(7 if camera == 0 else 2)]
                    for camera in range(2)]
        child_frames = [[10] * 5 + [11] * 2, [10] * 2]
        poses = [[accepted_pose(0x84000100 + camera * 0x100 + index * 32) for index in range(2)]
                 for camera in range(2)]
        pose_frames = [[90, 90], [91, 92]]
        expected = {}
        for camera in range(2):
            packet = batched_image_packets(camera, image_words[camera], 900 + camera, 16)[0]
            expected[packet_key(packet)] = packet
        for camera, batches in enumerate(((range(4), range(4, 5), range(5, 7)), (range(2),))):
            for sequence, indices in enumerate(batches):
                indices = list(indices)
                body = b"".join(word_bytes(children[camera][index], 8) for index in indices)
                packet = batched_header(camera, 2, child_frames[camera][indices[0]], sequence, 0, len(body)) + body
                expected[packet_key(packet)] = packet
        for camera, batches in enumerate(((range(2),), (range(1), range(1, 2)))):
            for sequence, indices in enumerate(batches):
                indices = list(indices)
                body = b"".join(word_bytes(poses[camera][index], 4) for index in indices)
                packet = batched_header(camera, 3, pose_frames[camera][indices[0]], sequence, 0, len(body)) + body
                expected[packet_key(packet)] = packet
        releases = {index: [] for index in (1, 2, 4, 5)}

        @passive
        def monitor():
            beats = {index: 0 for index in range(6)}
            packets = {index: 0 for index in range(6)}
            while True:
                accept = (yield transport.source.valid) and (yield transport.source.ready)
                selected = (yield transport.source.src_port) - 6000
                if accept:
                    beats[selected] += 1
                for index in (1, 2, 4, 5):
                    channel = transport.channels[index]
                    self.assertEqual((yield dut.committed[index]),
                                     (yield dut.sent[index]) + (yield channel.counter_committed_level))
                    self.assertIn((yield channel.counter_committed_level), (0, 1))
                    if (yield channel.release):
                        self.assertTrue(accept)
                        self.assertEqual(selected, index)
                        releases[index].append((packets[index], beats[index], (yield transport.source.last)))
                if accept and (yield transport.source.last):
                    packets[selected] += 1
                    beats[selected] = 0
                yield

        def stimulus():
            yield from configure(transport)
            yield from image_frame(transport.channels[0], image_words[0], 900)
            for _ in range(80):
                if (yield transport.source.valid):
                    break
                yield
            else:
                self.fail("image blocker did not acquire serializer ownership")
            self.assertEqual((yield transport.source.src_port), 6000)
            yield from image_frame(transport.channels[3], image_words[1], 901)
            for camera in range(2):
                for group, frame in zip(children[camera], child_frames[camera]):
                    yield from child_group(transport.channels[camera * 3 + 1], group, frame)
                for words, frame in zip(poses[camera], pose_frames[camera]):
                    yield from send_pose(transport.channels[camera * 3 + 2], words, frame)
                yield from send_pose(transport.channels[camera * 3 + 2], rejected_pose(0x1000), 99)
            yield from idle(4)
            for index, count in ((1, 7), (2, 2), (4, 2), (5, 2)):
                self.assertEqual((yield transport.channels[index].committed_level), count)
                self.assertEqual((yield dut.committed[index]), 0,
                                 "auxiliary commit counters count planned datagrams, not native records")
            readiness["enabled"] = True
            yield from wait_packets(capture, len(expected), budget=6000)
            for index, channel in enumerate(transport.channels):
                camera, kind = divmod(index, 3)
                products = [packet for key, packet in expected.items() if key[:2] == (camera, kind + 1)]
                self.assertEqual((yield channel.faults), 0)
                self.assertEqual((yield channel.fifo_level), 0)
                self.assertEqual((yield channel.counter_committed_level), 0)
                self.assertEqual((yield dut.committed[index]), len(products))
                self.assertEqual((yield dut.sent[index]), len(products))
                self.assertEqual((yield dut.byte_counts[index]), sum(map(len, products)))
                self.assertEqual((yield dut.rejections[index]), int(kind == 2))

        run_simulation(dut, [stimulus(), monitor(), capture.run()])
        self.assertEqual(len(capture.packets), len(expected))
        self.assertEqual({packet_key(packet): packet for packet in capture.packets}, expected)
        self.assertEqual(releases[1], [(0, 26, 0), (0, 44, 0), (0, 62, 0), (0, 80, 1),
                                       (1, 26, 1), (2, 26, 0), (2, 44, 1)])
        self.assertEqual(releases[4], [(0, 26, 0), (0, 44, 1)])
        self.assertEqual(releases[2], [(0, 30, 0), (0, 52, 1)])
        self.assertEqual(releases[5], [(0, 30, 1), (1, 30, 1)])
        for packet, metadata in zip(capture.packets, capture.metadata):
            camera, kind, _ = packet_key(packet)
            self.assertEqual(metadata, (len(packet), 0xC0A80102, 6000 + camera * 3 + kind - 1))

    def test_batched_batch_does_not_wait_for_or_absorb_future_child_records(self):
        dut = SharedHarness(compact_padding=True, batched_packets=True,
                            frame_words=16, fragment_words=16)
        transport = dut.transport
        child = transport.channels[1]
        capture = Capture(transport.source, ready=lambda cycle: cycle % 4 == 0)
        groups = [[0x100000000 + group * 16 + index for index in range(9)] for group in range(2)]

        def stimulus():
            yield from configure(transport)
            yield from child_group(child, groups[0], 80)
            for _ in range(80):
                if (yield transport.source.valid):
                    break
                yield
            else:
                self.fail("a single committed child record waited for a future batch member")
            self.assertEqual((yield transport.source.length), 104)
            self.assertEqual((yield transport.source.src_port), 6001)
            # The next group is committed before the first packet finishes,
            # but cannot change the already-fixed packet length or body.
            yield from child_group(child, groups[1], 80)
            self.assertEqual(len(capture.packets), 0)
            yield from wait_packets(capture, 2)
            self.assertEqual((yield child.faults), 0)
            self.assertEqual((yield dut.committed[1]), 2)
            self.assertEqual((yield dut.sent[1]), 2)
            self.assertEqual((yield dut.byte_counts[1]), 208)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(capture.packets, [batched_header(0, 2, 80, index, 0, 72) + word_bytes(group, 8)
                                          for index, group in enumerate(groups)])

    def test_batched_short_and_long_child_groups_fault_without_emitting_partial_records(self):
        for count in (8, 10):
            with self.subTest(tuple_count=count):
                dut = SharedHarness(compact_padding=True, batched_packets=True,
                                    frame_words=16, fragment_words=16)
                transport = dut.transport
                child = transport.channels[1]
                capture = Capture(transport.source)
                valid = [0x100000000 + index for index in range(9)]

                def stimulus():
                    yield from configure(transport)
                    yield from child_group(child, list(range(count)), 100)
                    yield from idle(60)
                    self.assertEqual((yield child.faults), 2)
                    self.assertGreaterEqual((yield dut.protocol_errors[1]), 1)
                    self.assertEqual((yield child.committed_level), 0)
                    self.assertEqual((yield dut.committed[1]), 0)
                    self.assertEqual((yield dut.sent[1]), 0)
                    self.assertEqual(capture.packets, [])
                    yield child.clear.eq(1)
                    yield
                    yield child.clear.eq(0)
                    yield
                    yield from child_group(child, valid, 101)
                    yield from wait_packets(capture, 1)
                    self.assertEqual((yield child.faults), 0)
                    self.assertEqual((yield dut.committed[1]), 1)
                    self.assertEqual((yield dut.sent[1]), 1)
                    self.assertEqual((yield dut.byte_counts[1]), 104)

                run_simulation(dut, [stimulus(), capture.run()])
                self.assertEqual(capture.packets, [batched_header(0, 2, 101, 0, 0, 72) + word_bytes(valid, 8)])


if __name__ == "__main__":
    unittest.main()
