"""Physical capture orchestration fault paths and strict packet accounting."""

from collections import deque
from contextlib import redirect_stdout, redirect_stderr
import io
from itertools import product
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from tools.lutcam.capture import (
    DiagnosticCapture, FrameWriter, decode_lftag_diagnostics, main, validate_capture,
)
from tools.lutcam.receive import Packet, PacketError, parse_datagram


def packet(frame=0, sequence=0, offset=0, payload=b"\x12\x34", *, camera=1, version=0):
    return Packet(camera, 1, frame, sequence, offset, 0, payload, version)


def datagram(value):
    return struct.pack(">8I", 0x4c555443, value.version << 24 | value.camera << 16 | 1 << 8 | 32,
                       value.frame_id, value.sequence, value.offset, len(value.payload), 0, 0) + value.payload


class Clock:
    def __init__(self):
        self.now = 0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds

    def wait(self, readable, writable, exceptional, seconds):
        self.sleep(seconds)
        return readable, [], []


class Receiver:
    def __init__(self):
        self.queue = deque()

    def recvfrom(self, size):
        if not self.queue:
            raise BlockingIOError()
        return self.queue.popleft()


class Bus:
    BASE, ISP, IMAGE = 0x30024100, 0x30010000, 0x30024580

    def __init__(self):
        self.values = {0: 0, self.BASE: 0, self.BASE + 0x0c: 5,
                       self.BASE + 0x14: 10, self.ISP: 0x4d495350,
                       self.ISP + 8: 4}
        self.writes = []
        self.fail_snapshot = False
        self.init_delay_reads = self.init_reads_remaining = 0

    def read(self, address, length=None):
        if length is not None:
            if self.fail_snapshot and address == self.BASE:
                raise TimeoutError("snapshot link lost")
            return [self.values.get(address + 4 * index, 0) for index in range(length)]
        if address == self.BASE + 0x14 and self.init_reads_remaining:
            self.init_reads_remaining -= 1
            return self.values.get(address, 0) & ~8
        return self.values.get(address, 0)

    def write(self, address, value):
        self.writes.append((address, value))
        self.values[address] = value
        if address == self.BASE + 8:
            self.init_reads_remaining = self.init_delay_reads
            self.values[self.BASE + 0x18] = 0
            self.values[self.BASE + 0x20] = self.values[self.BASE + 0x24] = 0
        if address == self.BASE + 0x34:
            self.values[self.BASE + 0x38] = 1


class Sensor:
    def __init__(self, bus, receiver):
        self.bus, self.receiver = bus, receiver
        self.streaming = 0
        self.writes = []
        self.fault_during = self.fault_after = 0
        self.break_post_snapshot = self.fail_stop = False
        self.packets = [packet(), packet(sequence=1, offset=2), packet(frame=1, sequence=2)]

    def read_register(self, address, register, length=1):
        return {0x300a: b"\x92\x81", 0x0100: bytes([self.streaming]),
                0x3808: bytes.fromhex("05000320")}[register]

    def write_register(self, address, register, value):
        self.writes.append((address, register, value))
        if not value and self.fail_stop:
            raise TimeoutError("sensor standby NACK")
        self.streaming = value
        if value:
            self.bus.values[Bus.BASE + 0x18] = self.fault_during
            self.bus.values[Bus.BASE + 0x20] = self.bus.values[Bus.BASE + 0x24] = 2
            self.bus.values[Bus.IMAGE + 0x24] = self.bus.values[Bus.IMAGE + 0x28] = 3
            self.receiver.queue.extend((datagram(value), ("192.168.1.50", 6003)) for value in self.packets)
        else:
            self.bus.values[Bus.BASE + 0x18] |= self.fault_after
            self.bus.fail_snapshot = self.break_post_snapshot


class FrameWriterTests(unittest.TestCase):
    def test_streams_frames_and_accepts_uint32_wrap(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = FrameWriter(directory, 1)
            writer.add(packet(frame=0xffffffff, sequence=0xffffffff))
            writer.add(packet(frame=0, sequence=0))
            writer.close()
            self.assertEqual(writer.packets, 2)
            self.assertEqual((Path(directory) / "frame-00000000.lwc").read_bytes(), b"\x12\x34")
            self.assertEqual(list(writer.frames), [str(0xffffffff), "0"])

    def test_gap_duplicate_reorder_overlap_wrong_camera_version(self):
        bad = [packet(sequence=2, offset=2), packet(sequence=0, offset=2),
               packet(sequence=1, offset=4), packet(sequence=1, offset=0),
               packet(frame=2, sequence=1), packet(frame=1, sequence=1, offset=2),
               packet(sequence=1, offset=2, camera=0), packet(sequence=1, offset=2, version=2)]
        for value in bad:
            with self.subTest(value=value), tempfile.TemporaryDirectory() as directory:
                writer = FrameWriter(directory, 1)
                writer.add(packet())
                with self.assertRaises(PacketError):
                    writer.add(value)
                writer.close()

    def test_existing_frame_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frame-00000000.lwc"
            path.write_bytes(b"keep")
            with self.assertRaises(FileExistsError):
                FrameWriter(directory, 1).add(packet())
            self.assertEqual(path.read_bytes(), b"keep")

    def test_frame_rate_excludes_frames_buffered_by_blocking_sensor_start(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = Clock()
            writer = FrameWriter(directory, 1, clock=clock)
            # Six buffered frames arrive at once, then the live 25 Hz stream.
            for index, timestamp in enumerate([0] * 6 + [0.04, 0.08, 0.12, 0.16, 0.20]):
                clock.now = timestamp
                writer.add(packet(frame=index, sequence=index))
            writer.close()
            report = writer.summary()
            self.assertAlmostEqual(report["host_frame_rate_hz"], 25)
            self.assertEqual(report["frames"]["5"]["first_packet_elapsed"], 0)
            self.assertEqual(report["frames"]["8"]["first_packet_elapsed"], 0.12)
            self.assertEqual(report["host_frame_rate_window"]["frames"], 3)
            self.assertEqual(report["host_frame_rate_window"]["start_elapsed"], 0.12)
            self.assertEqual(report["host_frame_rate_window"]["end_elapsed"], 0.20)

    def test_report_records_actual_parsed_udp_payload_lengths_and_frame_tails(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = FrameWriter(directory, 1)
            packets = [packet(payload=bytes(1024)),
                       packet(sequence=1, offset=1024, payload=bytes(1022)),
                       packet(frame=1, sequence=2, payload=bytes(8))]
            for value in packets:
                writer.add(parse_datagram(datagram(value)))
            writer.close()
            report = writer.summary()
            self.assertEqual(report["payload_length_histogram"], {"1024": 1, "1022": 1, "8": 1})
            self.assertEqual(report["frames"]["0"]["last_packet_bytes"], 1022)
            self.assertEqual(report["frames"]["1"]["last_packet_bytes"], 8)
            self.assertEqual(report["frames"]["0"]["bytes"], 2046)

    def test_short_capture_has_no_rate_without_two_frames_after_warmup(self):
        for timestamps in ([0, 0.04, 0.08], [0, 0.05, 0.12], [0, 0.12, 0.12]):
            with self.subTest(timestamps=timestamps), tempfile.TemporaryDirectory() as directory:
                clock = Clock()
                writer = FrameWriter(directory, 1, clock=clock)
                for index, timestamp in enumerate(timestamps):
                    clock.now = timestamp
                    writer.add(packet(frame=index, sequence=index))
                writer.close()
                self.assertIsNone(writer.summary()["host_frame_rate_hz"])


class CaptureRefusalTests(unittest.TestCase):
    def test_single_camera_api_refuses_before_io(self):
        with tempfile.TemporaryDirectory() as directory:
            bus, receiver, clock = Bus(), Receiver(), Clock()
            sensor = Sensor(bus, receiver)
            capture = DiagnosticCapture(bus, receiver, FrameWriter(directory, 1),
                camera=1, address=0x10, destination="192.168.1.251", port=6003,
                board_ip="192.168.1.50", seconds=1, sensor=sensor)
            with self.assertRaisesRegex(Exception, "single-camera capture is disabled"):
                capture.run()
            self.assertEqual(bus.writes, [])
            self.assertEqual(sensor.writes, [])



class SnapshotTests(unittest.TestCase):
    class ReadTrackingBus(Bus):
        def __init__(self):
            super().__init__()
            self.reads = []
            self.reject_isp = False

        def read(self, address, length=None):
            self.reads.append((address, length))
            if self.reject_isp and self.ISP <= address < self.ISP + 0x10000:
                raise AssertionError("faulted ISP must not receive a bus access")
            return super().read(address, length)

    def capture(self, bus, *, transport="etherbone"):
        return DiagnosticCapture(bus, Receiver(), None, camera=1, address=0x10,
            destination="192.168.1.251", port=6003, board_ip="192.168.1.50",
            seconds=1, transport=transport, sensor=object())

    def test_sticky_camera_fault_skips_isp_and_preserves_host_faults(self):
        for transport, fault_base in (("etherbone", 0x30024800), ("usb", 0x30024880)):
            with self.subTest(transport=transport):
                bus = self.ReadTrackingBus()
                bus.values[bus.BASE + 0x14] |= 4
                bus.values[fault_base] = 7
                bus.values[fault_base + 4] = 0x30010008
                bus.values[fault_base + 8] = 3
                bus.reject_isp = True
                state = self.capture(bus, transport=transport).snapshot()
                self.assertIsNone(state["isp"])
                self.assertTrue(state["isp_read_skipped"])
                self.assertEqual(state["host_faults"], [7, 0x30010008, 3])
                self.assertTrue(state["camera"][5] & 4)
                self.assertFalse(any(bus.ISP <= address < bus.ISP + 0x10000
                                     for address, _ in bus.reads))
                self.assertEqual(bus.writes, [])

    def test_explicit_skip_does_not_access_healthy_isp(self):
        bus = self.ReadTrackingBus()
        bus.reject_isp = True
        state = self.capture(bus).snapshot(include_isp=False)
        self.assertIsNone(state["isp"])
        self.assertTrue(state["isp_read_skipped"])
        self.assertFalse(state["camera"][5] & 4)
        self.assertFalse(any(bus.ISP <= address < bus.ISP + 0x10000
                             for address, _ in bus.reads))
        self.assertEqual(bus.writes, [])

    def test_fault_during_diagnostic_reads_skips_isp_after_status_recheck(self):
        class LateFaultBus(self.ReadTrackingBus):
            def read(self, address, length=None):
                if address == self.IMAGE:
                    self.values[self.BASE + 0x14] = self.late_status
                return super().read(address, length)

        for late_status in (12, 0xffffffff):
            with self.subTest(late_status=late_status):
                bus = LateFaultBus()
                self.assertFalse(bus.values[bus.BASE + 0x14] & 4)
                bus.late_status = late_status
                bus.reject_isp = True

                state = self.capture(bus).snapshot()

                self.assertEqual(state["camera"][5], late_status)
                self.assertIsNone(state["isp"])
                self.assertTrue(state["isp_read_skipped"])
                self.assertEqual(bus.reads[-1], (bus.BASE + 0x14, None))
                self.assertFalse(any(bus.ISP <= address < bus.ISP + 0x10000
                                     for address, _ in bus.reads))
                self.assertEqual(bus.writes, [])

    def test_healthy_snapshot_preserves_data_and_reads_host_faults_before_isp(self):
        bus = self.ReadTrackingBus()
        regions = {"camera": (bus.BASE, 16), "isp": (bus.ISP, 5),
                   "board": (0, 4), "image": (bus.IMAGE, 14),
                   "host_faults": (0x30024800, 3)}
        expected = {}
        for region_index, (name, (base, count)) in enumerate(regions.items(), 1):
            expected[name] = [region_index * 0x100 + index for index in range(count)]
            for index, value in enumerate(expected[name]):
                bus.values[base + index * 4] = value
        expected["camera"][5] = bus.values[bus.BASE + 0x14] = 2

        state = self.capture(bus).snapshot()

        for name, values in expected.items():
            self.assertEqual(state[name], values, name)
        self.assertFalse(state.get("isp_read_skipped"))
        read_addresses = [address for address, _ in bus.reads]
        self.assertEqual(read_addresses[0], bus.BASE)
        self.assertLess(read_addresses.index(0x30024800), read_addresses.index(bus.ISP))
        self.assertEqual(bus.writes, [])

    def test_snapshot_omits_unsupported_lftag_diagnostics(self):
        for magic in (0, 0xffffffff, 0x4c464403):
            with self.subTest(magic=magic):
                bus = self.ReadTrackingBus()
                bus.values[bus.BASE + 0x4c] = magic

                state = self.capture(bus).snapshot()

                self.assertNotIn("lftag", state)
                self.assertEqual(bus.reads.count((bus.BASE + 0x40, 4)), 1)
                self.assertEqual(bus.writes, [])

    def test_snapshot_reads_supported_lftag_diagnostics_when_isp_is_skipped(self):
        modes = ((False, True), (True, True), (False, False))
        for magic, (faulted, include_isp) in product((0x4c464400,), modes):
            with self.subTest(magic=magic, faulted=faulted, include_isp=include_isp):
                bus = self.ReadTrackingBus()
                if faulted:
                    bus.values[bus.BASE + 0x14] |= 4
                bus.reject_isp = faulted or not include_isp
                raw_words = [0x15, 0x1234, 0x5678, magic]
                for offset, value in zip((0x40, 0x44, 0x48, 0x4c), raw_words):
                    bus.values[bus.BASE + offset] = value

                state = self.capture(bus).snapshot(include_isp=include_isp)

                self.assertIsInstance(state["lftag"], dict)
                self.assertEqual(state["lftag"]["raw_words"], raw_words)
                self.assertEqual(state["lftag"], decode_lftag_diagnostics(raw_words))
                diagnostic_reads = [(address, length) for address, length in bus.reads
                                    if bus.BASE + 0x40 <= address < bus.BASE + 0x50]
                self.assertEqual(diagnostic_reads, [(bus.BASE + 0x40, 4)])
                self.assertEqual(bus.writes, [])
                if bus.reject_isp:
                    self.assertIsNone(state["isp"])
                    self.assertFalse(any(bus.ISP <= address < bus.ISP + 0x10000
                                         for address, _ in bus.reads))
                else:
                    self.assertEqual(state["isp"], [0x4d495350, 0, 4, 0, 0])

    @staticmethod
    def valid_report():
        def state():
            camera, image = [0] * 16, [0] * 14
            camera[5] = 2
            camera[8:10] = [1, 1]
            image[9:11] = [1, 1]
            return {"camera": camera, "image": image, "isp": [0] * 5,
                    "host_faults": [0] * 3, "board": [0] * 4}
        return {"frames": {"0": {}}, "packets": 1, "snapshots": [state()],
                "capture_complete": state(), "post_stop": state()}

    def test_missing_isp_evidence_fails_validation_without_exception(self):
        self.assertEqual(validate_capture(self.valid_report()), [])
        for location in ("snapshots", "capture_complete", "post_stop"):
            for absent in (False, True):
                with self.subTest(location=location, absent=absent):
                    report = self.valid_report()
                    state = report[location][0] if location == "snapshots" else report[location]
                    if absent:
                        del state["isp"]
                    else:
                        state["isp"] = None
                    state["isp_read_skipped"] = "ISP intentionally omitted"
                    errors = validate_capture(report)
                    self.assertTrue(any("ISP" in error for error in errors), errors)

    def test_rejected_tag_candidate_does_not_invalidate_image_capture(self):
        report = self.valid_report()
        for state in report["snapshots"] + [report["capture_complete"], report["post_stop"]]:
            state["camera"][5] |= 64
        self.assertEqual(validate_capture(report), [])
        self.assertTrue(report["capture_complete"]["camera"][5] & 64)
        for fault in (4, 32, 128):
            with self.subTest(fault=fault):
                report = self.valid_report()
                report["capture_complete"]["camera"][5] |= 64 | fault
                self.assertIn("camera capture/configuration fault", validate_capture(report))


class LFTagDiagnosticsTests(unittest.TestCase):
    MAGIC = 0x4c464400


    def test_unknown_magic_does_not_decode_words(self):
        for magic in (0, 0xffffffff, 0x4c464403):
            with self.subTest(magic=magic):
                self.assertIsNone(decode_lftag_diagnostics([0xffffffff] * 3 + [magic]))


    def test_each_protocol_reason_bit_has_its_own_name(self):
        reasons = ("bad_count", "scale_3", "scale_changed", "frame_changed")
        layouts = ((self.MAGIC, 6),)
        for (magic, valid_bit), (bit, reason) in product(layouts, enumerate(reasons)):
            with self.subTest(magic=magic, bit=bit):
                state = decode_lftag_diagnostics([1 << valid_bit, 1 << bit, 0, magic])
                self.assertEqual(state["protocol"]["reason_bits"], 1 << bit)
                self.assertEqual(state["protocol"]["reasons"], [reason])


    def test_current_six_bit_high_water_does_not_imply_context_or_seen_flags(self):
        for high_water in (32, 63):
            with self.subTest(high_water=high_water):
                raw_words = [high_water, 0xffffffff, 0xffffffff, self.MAGIC]
                state = decode_lftag_diagnostics(raw_words)
                self.assertEqual(state["raw_words"], raw_words)
                self.assertEqual(state["magic"], self.MAGIC)
                self.assertEqual(state["high_water"], high_water)
                for name in ("protocol_context_valid", "denied_admission_context_valid",
                             "protocol_group_seen", "dropped_group_seen"):
                    self.assertFalse(state[name], name)
                self.assertIsNone(state["protocol"])
                self.assertIsNone(state["denied_admission"])

    def test_current_status_flags_are_independent_of_each_other_and_high_water(self):
        flags = {"protocol_context_valid": 6, "denied_admission_context_valid": 7,
                 "protocol_group_seen": 8, "dropped_group_seen": 9}
        for selected, bit in flags.items():
            with self.subTest(selected=selected):
                state = decode_lftag_diagnostics([32 | (1 << bit), 0, 0, self.MAGIC])
                self.assertEqual(state["high_water"], 32)
                for name in flags:
                    self.assertEqual(state[name], name == selected, name)
                self.assertEqual(state["protocol"] is not None,
                                 selected == "protocol_context_valid")
                self.assertEqual(state["denied_admission"] is not None,
                                 selected == "denied_admission_context_valid")

    def test_current_six_bit_denied_levels_do_not_bleed_into_neighboring_fields(self):
        for own, peer in ((32, 63), (63, 32)):
            with self.subTest(own=own, peer=peer):
                state = decode_lftag_diagnostics(
                    [32 | (1 << 7), 0, own | (peer << 6), self.MAGIC])
                self.assertEqual(state["denied_admission"], {
                    "own_level": own, "peer_level": peer, "pop_fire": False,
                    "owner_active": False, "owner_camera": 0,
                    "core_output_valid": False, "core_output_ready": False,
                    "core_idle": False, "scale": 0, "frame": 0,
                    "group_end": False,
                })

    def test_current_denied_context_flags_preserve_shifted_scale_frame_and_levels(self):
        flags = {"pop_fire": 12, "owner_active": 13, "owner_camera": 14,
                 "core_output_valid": 15, "core_output_ready": 16,
                 "core_idle": 17, "group_end": 29}
        for selected in (None, *flags, "all"):
            with self.subTest(selected=selected):
                dropped = 63 | (32 << 6) | (2 << 18) | (0xa5 << 20)
                expected = {"own_level": 63, "peer_level": 32,
                            "scale": 2, "frame": 0xa5}
                for name, bit in flags.items():
                    enabled = selected in (name, "all")
                    expected[name] = int(enabled) if name == "owner_camera" else enabled
                    if enabled:
                        dropped |= 1 << bit
                state = decode_lftag_diagnostics([1 << 7, 0, dropped, self.MAGIC])
                self.assertEqual(state["denied_admission"], expected)

    def test_protocol_word_preserves_fields(self):
        protocol = (0b11010 | (12 << 5) | (1 << 9) | (3 << 10) | (2 << 12)
                    | (0xa5 << 14) | (0x5a << 22) | (1 << 30))
        state = decode_lftag_diagnostics([1 << 6, protocol, 0, self.MAGIC])
        self.assertEqual(state["protocol"], {
            "reason_bits": 0b01010,
            "reasons": ["scale_3", "frame_changed"],
            "stage_count_before_tuple": 12, "group_end": True,
            "first_scale": 3, "current_scale": 2,
            "first_frame": 0xa5, "current_frame": 0x5a,
        })

    def test_invalid_contexts_remain_absent_even_with_seen_flags_and_stale_words(self):
        for seen_bits in (0, 1 << 8, 1 << 9, (1 << 8) | (1 << 9)):
            with self.subTest(seen_bits=seen_bits):
                state = decode_lftag_diagnostics([seen_bits, 0xffffffff, 0xffffffff, self.MAGIC])
                self.assertEqual(state["high_water"], 0)
                self.assertFalse(state["protocol_context_valid"])
                self.assertFalse(state["denied_admission_context_valid"])
                self.assertEqual(state["protocol_group_seen"], bool(seen_bits & (1 << 8)))
                self.assertEqual(state["dropped_group_seen"], bool(seen_bits & (1 << 9)))
                self.assertIsNone(state["protocol"])
                self.assertIsNone(state["denied_admission"])

    def test_decoder_requires_exactly_four_uint32_words(self):
        for words in ([], [0] * 3, [0] * 5):
            with self.subTest(words=words), self.assertRaises(ValueError):
                decode_lftag_diagnostics(words)
        for index in range(4):
            for value in (True, False, -1, 1 << 32, 1.0, "0", None):
                with self.subTest(index=index, value=value):
                    words = [0, 0, 0, self.MAGIC]
                    words[index] = value
                    with self.assertRaises(ValueError):
                        decode_lftag_diagnostics(words)


if __name__ == "__main__":
    unittest.main()
