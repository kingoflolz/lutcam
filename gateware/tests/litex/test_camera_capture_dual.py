"""Dual sensor preflight, shared ownership, cleanup and packet accounting."""

from contextlib import redirect_stdout, redirect_stderr
from itertools import product
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.lutcam.capture import DiagnosticCapture, FrameWriter
from tools.lutcam.capture_dual import DualDiagnosticCapture, main
from tests.litex.test_camera_capture import Clock, Receiver, datagram, packet


class DualBus:
    PAIR = 0x30024200

    def __init__(self):
        self.events, self.values = [], {0: 0}
        self.fail_snapshots = set()
        self.sensors = {}
        self.lock_enabled = True
        self.snapshot_counter = 0
        self.fail_disable_generator = False
        self.drop_clear = False
        self.partial_fault = False
        self.fault_pair_status = None
        self.recovery_init_failure = False
        self.before_recovery_reset = None
        self.after_recovery_reset = None
        self.reset_count = 0
        self.faulted_isp_reads = []
        self.rejected_writes = []
        self.values.update({self.PAIR: 0x4c435031, self.PAIR + 4: 1,
            self.PAIR + 8: 0, self.PAIR + 0x14: 0, self.PAIR + 0x20: 4_000_000,
            self.PAIR + 0x24: 12_000, self.PAIR + 0x64: 994812, self.PAIR + 0x68: 1536,
            self.PAIR + 0x6c: 0, self.PAIR + 0x88: 2, self.PAIR + 0x8c: 900000})
        for camera in (0, 1):
            base, isp = 0x30024000 + camera * 0x100, 0x30000000 + camera * 0x10000
            self.values.update({base: 0, base + 0x0c: 5, base + 0x14: 10,
                                isp: 0x4d495350, isp + 8: 4})

    def locked(self):
        return self.lock_enabled and self.values[self.PAIR + 0x6c] and len(self.sensors) == 2 and all(sensor.streaming for sensor in self.sensors.values())

    def read(self, address, length=None):
        if length is not None:
            if address in self.fail_snapshots:
                raise TimeoutError("snapshot unavailable")
            return [self.read(address + 4 * index) for index in range(length)]
        if address == self.PAIR + 0x0c:
            if self.values[self.PAIR + 0x14] and self.fault_pair_status is not None:
                return self.fault_pair_status
            return (16 | 128 | (64 if self.locked() else 0)
                    | (512 if self.snapshot_counter else 0)
                    | (3 if self.values[self.PAIR + 8] else 0)
                    | (32 if self.values[self.PAIR + 0x14] else 0))
        for camera in (0, 1):
            isp = 0x30000000 + camera * 0x10000
            if isp <= address < isp + 0x10000 and self.values[0x30024014 + camera * 0x100] & 4:
                self.faulted_isp_reads.append(address)
                return 0xffffffff
        return self.values.get(address, 0)

    def write(self, address, value):
        self.events.append(("bus-write", address, value))
        if address == self.PAIR + 0x60 and self.drop_clear:
            return  # An unacknowledged management write may never reach hardware.
        if address == self.PAIR + 0x6c and not value and self.values[address] and self.fail_disable_generator:
            raise TimeoutError("FSYNC disable failed")
        if address in (0x30024000, 0x30024100):
            raise RuntimeError("individual RUN writes are forbidden")
        faulted = any(self.values[0x30024014 + camera * 0x100] & 4 for camera in (0, 1))
        if faulted and (address in (0x30024400, 0x30024580)
                        or 0x30000000 <= address < 0x30020000):
            self.rejected_writes.append((address, value))
            return
        if address == self.PAIR + 0x10:
            self.reset_count += 1
            if self.reset_count > 1 and self.before_recovery_reset is not None:
                self.before_recovery_reset()
        self.values[address] = value
        if address in (self.PAIR + 0x10, self.PAIR + 0x60):
            self.values[self.PAIR + 0x14] = 0
            self.values[self.PAIR + 0x18] = self.values[self.PAIR + 0x1c] = 0
            for camera in (0, 1):
                base = 0x30024000 + camera * 0x100
                self.values[base + 0x18] = self.values[base + 0x20] = self.values[base + 0x24] = 0
                self.values[base + 0x14] = 10
                # Explicit diagnostic CLEAR clears retained contexts. Paired
                # camera RESET is not the shared solver's global reset.
                if address == self.PAIR + 0x60:
                    for offset in (0x40, 0x44, 0x48):
                        self.values[base + offset] = 0
                for product in range(3):
                    self.values[0x30024414 + camera * 0x180 + product * 0x80] = 0
                if address == self.PAIR + 0x10:
                    self.values[0x30000004 + camera * 0x10000] = 0
                    self.values[0x30000008 + camera * 0x10000] = 4
                    for product in range(3):
                        self.values[0x30024400 + camera * 0x180 + product * 0x80] = 0
                    if self.reset_count > 1 and self.recovery_init_failure:
                        self.values[base + 0x14] = 2  # Idle but receiver never initializes.
            if address == self.PAIR + 0x10 and self.reset_count > 1 and self.after_recovery_reset is not None:
                self.after_recovery_reset()
        if address == self.PAIR + 0x28 and self.locked():
            self.snapshot_counter += 1
            anchor = (1 << 32) + self.snapshot_counter * self.values[self.PAIR + 0x64]
            sof0 = anchor + self.values.get(self.PAIR + 0x90, 0) + 1000
            sof1 = anchor + self.values.get(self.PAIR + 0x94, 0) + 1005
            raw_skew = (sof1 - sof0) & 0xffffffff
            words = [self.snapshot_counter, sof0 & 0xffffffff, sof0 >> 32,
                     sof1 & 0xffffffff, sof1 >> 32, raw_skew, raw_skew, raw_skew, 3, 3, 3, 3, 0]
            for index, word in enumerate(words):self.values[self.PAIR + 0x2c + 4 * index] = word
            for index, word in enumerate([anchor & 0xffffffff, anchor >> 32, 1000, 1005, 1000, 1005]):
                self.values[self.PAIR + 0x70 + 4 * index] = word
            for offset in (0x98, 0x9c, 0xa0):
                self.values[self.PAIR + offset] = 5
        if address == self.PAIR + 8:
            for camera in (0, 1):self.values[0x30024000 + camera * 0x100] = value
            if value:
                self.values[self.PAIR + 0x18] = self.values[self.PAIR + 0x1c] = 2
                for sensor in self.sensors.values():sensor.admit()
        for camera in (0, 1):
            base = 0x30024000 + camera * 0x100
            if address == base + 0x34:self.values[base + 0x38] = 1


class DualSensor:
    def __init__(self, bus, receiver, camera):
        from tools.lutcam.ov9281 import board_mode_writes
        self.bus, self.receiver, self.camera = bus, receiver, camera
        self.base = 0x30024000 + camera * 0x100
        self.image = 0x30024400 + camera * 0x180
        self.address = (0x60, 0x10)[camera]
        self.streaming = 0
        self.values = dict(board_mode_writes(camera, "off", fps=120))
        self.values.update({0x300a:0x92, 0x300b:0x81})
        self.fault_during = self.fault_warmup = 0
        self.fail_stop = self.fail_post_snapshot = False
        self.packets = [packet(camera=camera), packet(frame=1, sequence=1, camera=camera, payload=bytes(1022))]
        bus.sensors[camera] = self

    def read_register(self, address, register, length=1):
        if address != self.address:raise RuntimeError("wrong address strap")
        self.bus.events.append(("sensor-read", self.camera, register))
        return bytes(self.values.get(register + index, 0) for index in range(length))

    def write_register(self, address, register, value):
        if address != self.address:raise RuntimeError("wrong address strap")
        self.bus.events.append(("sensor-write", self.camera, register, value))
        if not value and self.fail_stop:raise TimeoutError("standby failed")
        self.streaming = value
        self.values[register] = value
        if value:self.bus.values[self.base + 0x18] = self.fault_warmup
        elif self.fail_post_snapshot:self.bus.fail_snapshots.add(self.base)

    def admit(self):
        self.bus.values[self.base + 0x18] = self.fault_during
        if self.fault_during:
            self.bus.values[DualBus.PAIR + 0x14] |= 1 << self.camera
            self.bus.values[self.base + 0x14] = 12  # Faulted, initialized, not idle.
            if self.bus.partial_fault:
                # A pair abort holds both processing paths in reset. Committed
                # packets drain, but each uncommitted writer retains one slot.
                for camera in (0, 1):
                    self.bus.values[0x30024014 + camera * 0x100] = 12
                    self.bus.values[0x30024414 + camera * 0x180] = 1
        self.bus.values[self.base + 0x20] = self.bus.values[self.base + 0x24] = 2
        self.bus.values[self.image + 0x24] = self.bus.values[self.image + 0x28] = 2
        self.receiver.queue.extend((datagram(value), ("192.168.1.50", 6000 + self.camera * 3)) for value in self.packets)


class DualCaptureTests(unittest.TestCase):
    def fixture(self, directory):
        bus, clock = DualBus(), Clock()
        captures, sensors = [], []
        for camera in (0, 1):
            receiver = Receiver()
            sensor = DualSensor(bus, receiver, camera)
            sensors.append(sensor)
            output = Path(directory) / f"camera{camera}"
            output.mkdir()
            captures.append(DiagnosticCapture(
                bus, receiver, FrameWriter(output, camera, clock=clock), camera=camera,
                address=sensor.address, destination="192.168.1.251", port=6000 + camera * 3,
                board_ip="192.168.1.50", seconds=0.006, sensor=sensor,
                clock=clock, sleep=clock.sleep, wait=clock.wait))
        runner = DualDiagnosticCapture(bus, captures, seconds=0.006,
                                       clock=clock, sleep=clock.sleep, wait=clock.wait)
        return runner, bus, sensors

    def run_capture(self, runner):
        with patch("tools.lutcam.capture.BootClient") as boot:
            boot.return_value.identity.return_value = {"image_slot": 1, "build_id": "fixture"}
            return runner.run()

    def assert_both_stopped(self, bus):
        for camera in (0, 1):
            self.assertIn(("sensor-write", camera, 0x0100, 0), bus.events)
            self.assertEqual(bus.values[0x30024000 + camera * 0x100], 0)
            self.assertEqual(bus.values[0x30000004 + camera * 0x10000], 0)
            self.assertEqual(bus.values[0x30024400 + camera * 0x180], 0)

    def test_pair_warmup_clear_arm_and_graceful_shutdown_order(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, sensors = self.fixture(directory)
            sensors[1].fault_warmup = 32
            report = self.run_capture(runner)
            self.assertTrue(report["ok"], report)
            self.assert_both_stopped(bus)
            self.assertGreaterEqual(report["warmup"]["warmup_seconds"], 0.1)
            arm = bus.events.index(("bus-write", DualBus.PAIR + 8, 1))
            clear = bus.events.index(("bus-write", DualBus.PAIR + 0x60, 1))
            reset = bus.events.index(("bus-write", DualBus.PAIR + 0x10, 1))
            for camera in (0,1):
                stream = bus.events.index(("sensor-write", camera, 0x0100, 1))
                self.assertLess(reset, stream)
                self.assertLess(stream, clear)
            self.assertLess(clear, arm)
            self.assertEqual(sum(event == ("bus-write", DualBus.PAIR + 0x10, 1) for event in bus.events), 1)
            stop = bus.events.index(("bus-write", DualBus.PAIR + 8, 0))
            slave_stop = bus.events.index(("sensor-write", 1, 0x0100, 0))
            master_stop = bus.events.index(("sensor-write", 0, 0x0100, 0))
            generator_off = max(index for index,event in enumerate(bus.events) if event == ("bus-write", DualBus.PAIR + 0x6c, 0))
            self.assertLess(stop, slave_stop)
            self.assertLess(slave_stop, master_stop)
            self.assertLess(master_stop, generator_off)
            for capture in runner.captures:
                self.assertTrue(capture.report["ok"], capture.report)
                self.assertEqual(capture.report["frames"]["1"]["last_packet_bytes"], 1022)
            self.assertEqual(report["pair_capture_complete"]["pairs_completed"], 2)
            self.assertEqual(sum(event == ("bus-write", 0, 0) for event in bus.events), 1)

    def test_unsupported_pair_abi_has_no_sensor_or_bus_mutations(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fixture(directory)
            bus.values[DualBus.PAIR] = 0
            report = self.run_capture(runner)
            self.assertFalse(report["ok"])
            self.assertEqual(bus.events, [])

    def test_dropped_clear_rejects_each_retained_lftag_word_before_pair_arm(self):
        for magic, camera, (word_index, value) in product(
                (0x4c464400,), (0, 1), enumerate((0x101, 0x80000000, 0x04000000))):
            with self.subTest(magic=magic, camera=camera, word_index=word_index), tempfile.TemporaryDirectory() as directory:
                runner, bus, _ = self.fixture(directory)
                bus.drop_clear = True
                retained = [0, 0, 0, magic]
                retained[word_index] = value
                capture = runner.captures[camera]
                for index, word in enumerate(retained):
                    bus.values[capture.base + 0x40 + index * 4] = word

                report = self.run_capture(runner)

                self.assertFalse(report["ok"])
                self.assertIn("LFTag diagnostic clear did not leave both retained contexts clean", report["error"])
                self.assertEqual(report["error_camera"], camera)
                self.assertEqual(capture.report["initial"]["lftag"]["raw_words"], retained)
                self.assertEqual(capture.report["armed"]["lftag"]["raw_words"], retained)
                self.assertIn(("bus-write", DualBus.PAIR + 0x60, 1), bus.events)
                self.assertNotIn(("bus-write", DualBus.PAIR + 8, 1), bus.events)
                self.assertEqual(bus.reset_count, 1)
                self.assert_both_stopped(bus)

    def test_delivered_clear_removes_old_lftag_contexts_and_allows_pair_arm(self):
        for magic in (0x4c464400,):
            with self.subTest(magic=magic), tempfile.TemporaryDirectory() as directory:
                runner, bus, _ = self.fixture(directory)
                retained = [0x1ff, 0x81234567, 0x07654321, magic]
                for capture in runner.captures:
                    for index, word in enumerate(retained):
                        bus.values[capture.base + 0x40 + index * 4] = word

                report = self.run_capture(runner)

                self.assertTrue(report["ok"], report)
                self.assertIn(("bus-write", DualBus.PAIR + 8, 1), bus.events)
                self.assertEqual(bus.reset_count, 1)
                for capture in runner.captures:
                    self.assertEqual(capture.report["initial"]["lftag"]["raw_words"], retained)
                    self.assertEqual(capture.report["armed"]["lftag"]["raw_words"],
                                     [0, 0, 0, magic])
                self.assert_both_stopped(bus)

    def test_second_sensor_mode_mismatch_causes_no_writes(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, sensors = self.fixture(directory)
            sensors[1].values[0x380f] ^= 1
            report = self.run_capture(runner)
            self.assertFalse(report["ok"])
            self.assertEqual(report["error_phase"], "preflight")
            self.assertFalse(any(event[0].endswith("write") for event in bus.events))

    def test_second_camera_pending_product_prevents_reset(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fixture(directory)
            bus.values[0x30024694] = 1
            report = self.run_capture(runner)
            self.assertFalse(report["ok"])
            self.assertIn("queued product", report["error"])
            self.assertFalse(any(event[0].endswith("write") for event in bus.events))

    def test_warmup_no_lock_never_arms_and_stops_both(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fixture(directory)
            bus.lock_enabled = False
            report = self.run_capture(runner)
            self.assertFalse(report["ok"])
            self.assertEqual(report["error_phase"], "warmup")
            self.assertIn("latched_counts", report["warmup_failure"])
            self.assertEqual(set(report["warmup_failure"]["cameras"]), {"0", "1"})
            self.assertFalse(report["warmup_failure"]["pair_live"]["status"] & 64)
            self.assertNotIn(("bus-write", DualBus.PAIR + 8, 1), bus.events)
            self.assert_both_stopped(bus)

    def test_pair_fault_preserves_error_and_drains_received_packets(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, sensors = self.fixture(directory)
            sensors[1].fault_during = 4
            report = self.run_capture(runner)
            self.assertFalse(report["ok"])
            self.assertIn("paired capture fault", report["error"])
            self.assertIsNone(report["error_camera"])
            self.assert_both_stopped(bus)
            for capture in runner.captures:
                self.assertEqual(capture.writer.packets, 2)
                self.assertIn("failure_drain", capture.report)
            self.assertFalse(any("failure drain" in error for error in report["cleanup_errors"]))

    def fault_fixture(self, directory):
        runner, bus, sensors = self.fixture(directory)
        bus.partial_fault = True
        sensors[1].fault_during = 32
        return runner, bus, sensors

    def test_faulted_partial_writers_recover_only_after_saved_standby_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, sensors = self.fault_fixture(directory)
            reset_observations = []
            standby_times = []
            original_stop = runner._stop_sensor

            def stop_sensor(camera):
                result = original_stop(camera)
                standby_times.append(runner.clock())
                return result

            runner._stop_sensor = stop_sensor

            def before_reset():
                self.assertTrue(all(sensor.streaming == 0 for sensor in sensors))
                self.assertEqual(bus.values[DualBus.PAIR + 0x6c], 0)
                self.assertEqual(bus.read(DualBus.PAIR + 0x0c) & 15, 0)
                self.assertTrue(bus.read(DualBus.PAIR + 0x0c) & 16)
                self.assertEqual(len(standby_times), 2)
                self.assertGreaterEqual(runner.clock() - max(standby_times), 0.05 - 1e-12)
                for capture in runner.captures:
                    self.assertIsNone(capture.writer.file)
                    evidence = capture.writer.output / "before-fault-reset.json"
                    saved = json.loads(evidence.read_text())
                    self.assertIsInstance(saved, dict)
                    self.assertIn('"post_stop"', evidence.read_text())
                    self.assertIn('"packets"', evidence.read_text())
                reset_observations.append(runner.clock())

            bus.before_recovery_reset = before_reset
            report = self.run_capture(runner)
            self.assertFalse(report["ok"])
            self.assertIn("paired capture fault", report["error"])
            self.assertEqual(bus.reset_count, 2)
            self.assertEqual(len(reset_observations), 1)
            self.assertEqual(report["failure_drain"]["partial_writer_cameras"], [0, 1])
            self.assertTrue(report["fault_recovery"]["attempted"])
            self.assertTrue(report["fault_recovery"]["ok"], report)
            self.assertIn("before", report["fault_recovery"])
            self.assertIn("after", report["fault_recovery"])
            self.assertEqual(bus.faulted_isp_reads, [])
            self.assertEqual(bus.rejected_writes, [])
            self.assertFalse(any("failure drain" in error for error in report["cleanup_errors"]))
            self.assert_both_stopped(bus)
            for capture in runner.captures:
                self.assertEqual(capture.report["post_stop"]["camera"][5], 12)
                self.assertEqual(capture.report["post_stop"]["image"][5], 1)
                self.assertIsNone(capture.report["post_stop"]["isp"])
                self.assertEqual(capture.report["after_cleanup"]["image"][5], 0)
            self.assertFalse(any(event[:2] in (("bus-write", 0x3002480c),
                                               ("bus-write", 0x3002488c)) for event in bus.events))

    def test_fault_recovery_guards_block_reset_and_unsafe_configuration(self):
        for failure in ("standby", "generator", "undrained", "busy", "all-ones", "post-stop"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                runner, bus, sensors = self.fault_fixture(directory)
                if failure == "standby":
                    sensors[1].fail_stop = True
                elif failure == "generator":
                    bus.fail_disable_generator = True
                elif failure == "undrained":
                    bus.fault_pair_status = 32 | 512
                elif failure == "busy":
                    bus.fault_pair_status = 4 | 16 | 32 | 512
                elif failure == "all-ones":
                    bus.fault_pair_status = 0xffffffff
                else:
                    sensors[1].fail_post_snapshot = True
                report = self.run_capture(runner)
                self.assertFalse(report["ok"])
                self.assertIn("error", report)
                self.assertEqual(bus.reset_count, 1, report)
                self.assertFalse(report.get("fault_recovery", {}).get("ok", False))
                self.assertEqual(bus.faulted_isp_reads, [])
                self.assertEqual(bus.rejected_writes, [])
                self.assertEqual(sensors[0].streaming, 0)

    def test_fault_recovery_persists_lftag_context_before_paired_reset(self):
        cases = (([[32 | (1 << 6) | (1 << 8), 1 | (8 << 5) | (1 << 9), 0, 0x4c464400],
              [63 | (1 << 7) | (1 << 9), 0, 32 | (63 << 6) | (1 << 12), 0x4c464400]],
             [32, 63], 32, 63),)
        for retained, high_water, own_level, peer_level in cases:
            with self.subTest(magic=retained[0][3]), tempfile.TemporaryDirectory() as directory:
                runner, bus, sensors = self.fault_fixture(directory)
                original_admit = sensors[1].admit

                def admit_and_retain_context():
                    original_admit()
                    for capture, words in zip(runner.captures, retained):
                        for index, word in enumerate(words):
                            bus.values[capture.base + 0x40 + index * 4] = word

                sensors[1].admit = admit_and_retain_context
                saved_before_reset = []

                def before_reset():
                    for capture, words in zip(runner.captures, retained):
                        path = capture.writer.output / "before-fault-reset.json"
                        saved = json.loads(path.read_text())
                        for phase in ("failure_drain", "post_stop"):
                            self.assertEqual(saved["camera"][phase]["lftag"]["raw_words"], words)
                        self.assertEqual(bus.read(capture.base + 0x40, length=4), words)
                        saved_before_reset.append(saved["camera"]["post_stop"]["lftag"])

                bus.before_recovery_reset = before_reset
                report = self.run_capture(runner)
                self.assertIn("paired capture fault", report["error"])
                self.assertTrue(report["fault_recovery"]["ok"], report)
                self.assertEqual(len(saved_before_reset), 2)
                self.assertEqual([state["high_water"] for state in saved_before_reset], high_water)
                self.assertEqual(saved_before_reset[1]["denied_admission"]["own_level"], own_level)
                self.assertEqual(saved_before_reset[1]["denied_admission"]["peer_level"], peer_level)
                for capture, words in zip(runner.captures, retained):
                    self.assertEqual(capture.report["post_stop"]["lftag"]["raw_words"], words)
                    self.assertEqual(capture.report["after_cleanup"]["lftag"]["raw_words"], words)
                    saved = json.loads((capture.writer.output / "before-fault-reset.json").read_text())
                    self.assertEqual(saved["camera"]["post_stop"]["lftag"]["raw_words"], words)
                self.assertEqual(bus.faulted_isp_reads, [])

    def test_existing_fault_evidence_is_preserved_and_blocks_reset(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fault_fixture(directory)
            evidence = runner.captures[0].writer.output / "before-fault-reset.json"
            evidence.write_bytes(b"existing evidence\n")
            report = self.run_capture(runner)
            self.assertIn("paired capture fault", report["error"])
            self.assertEqual(bus.reset_count, 1)
            self.assertEqual(evidence.read_bytes(), b"existing evidence\n")
            self.assertFalse(report["fault_recovery"]["ok"])
            self.assertEqual(bus.rejected_writes, [])

    def test_fault_recovery_rechecks_hardware_after_saving_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fault_fixture(directory)
            original = runner.pair.disarmed_drained

            def change_state_after_save():
                if all((capture.writer.output / "before-fault-reset.json").exists()
                       for capture in runner.captures):
                    bus.fault_pair_status = 4 | 16 | 32 | 512
                return original()

            runner.pair.disarmed_drained = change_state_after_save
            report = self.run_capture(runner)
            self.assertIn("paired capture fault", report["error"])
            self.assertEqual(bus.reset_count, 1)
            self.assertFalse(report["fault_recovery"]["ok"])
            self.assertEqual(bus.rejected_writes, [])

    def test_recovery_initialization_timeout_keeps_original_fault_and_saved_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fault_fixture(directory)
            bus.recovery_init_failure = True
            report = self.run_capture(runner)
            self.assertIn("paired capture fault", report["error"])
            self.assertEqual(bus.reset_count, 2)
            recovery = report["fault_recovery"]
            self.assertTrue(recovery["attempted"])
            self.assertFalse(recovery["ok"])
            self.assertIn("TimeoutError", recovery["error"])
            self.assertLess(runner.clock(), 4)
            for capture in runner.captures:
                self.assertTrue((capture.writer.output / "before-fault-reset.json").is_file())
                self.assertEqual(capture.report["post_stop"]["camera"][5], 12)

    def test_recovery_requires_all_product_queues_empty_and_valid_isp_access(self):
        for failure in ("children", "pose", "isp-magic", "isp-running"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                runner, bus, _ = self.fault_fixture(directory)

                def incomplete_reset():
                    camera = runner.captures[1]
                    if failure in ("children", "pose"):
                        channel = 1 if failure == "children" else 2
                        bus.values[camera.image + channel * 0x80 + 0x14] = 1
                    elif failure == "isp-magic":
                        bus.values[camera.isp] = 0xffffffff
                    else:
                        bus.values[camera.isp + 4] = 1

                bus.after_recovery_reset = incomplete_reset
                report = self.run_capture(runner)
                self.assertIn("paired capture fault", report["error"])
                self.assertEqual(bus.reset_count, 2)
                self.assertTrue(report["fault_recovery"]["attempted"])
                self.assertFalse(report["fault_recovery"]["ok"])
                self.assertIn("error", report["fault_recovery"])
                self.assertLess(runner.clock(), 4)
                self.assertTrue(all("output_cleanup_skipped" in capture.report
                                    for capture in runner.captures))

    def test_failure_drain_never_accepts_unsent_committed_packets(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fault_fixture(directory)
            bus.values[DualBus.PAIR + 0x14] = 1
            for camera, capture in enumerate(runner.captures):
                bus.values[capture.base + 0x14] = 12
                bus.values[capture.image + 0x14] = 1
                bus.values[capture.image + 0x24] = 2
                bus.values[capture.image + 0x28] = 1
            with self.assertRaisesRegex(TimeoutError, "drain timeout"):
                runner._drain(evidence_key="failure_drain")
            self.assertEqual(bus.reset_count, 0)

    def test_success_drain_rejects_partial_writer_even_when_pair_drained(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fixture(directory)
            bus.values[runner.captures[0].image + 0x14] = 1
            with self.assertRaisesRegex(TimeoutError, "drain timeout"):
                runner._drain()

    def test_packet_failure_drains_both_sockets_with_bounded_error_examples(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, sensors = self.fixture(directory)
            sensors[0].packets = [packet(camera=0)] + [packet(camera=0, sequence=2, offset=2)] * 32
            report = self.run_capture(runner)
            self.assertFalse(report["ok"])
            self.assertIn("packet sequence", report["error"])
            evidence = report["failure_drain_packet_errors"]["0"]
            self.assertEqual(evidence["count"], 31)
            self.assertEqual(len(evidence["examples"]), 8)
            self.assertEqual(runner.captures[1].writer.packets, 2)
            self.assertTrue(all(not capture.receiver.queue for capture in runner.captures))
            self.assert_both_stopped(bus)

    def test_failed_warmup_diagnostics_preserve_original_error(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fixture(directory)
            def fail_warmup():
                bus.fail_snapshots.update((0x30024000, 0x30024100))
                raise TimeoutError("original warmup failure")
            runner._warmup = fail_warmup
            report = self.run_capture(runner)
            self.assertIn("original warmup failure", report["error"])
            self.assertTrue(any("snapshot unavailable" in error for error in report["warmup_failure"]["errors"]))
            self.assertEqual(report["sensor_standby_verified"], [True, True])
            self.assertTrue(report["generator_disabled_after_standby"])
            self.assertEqual(bus.reset_count, 1)
            for capture in runner.captures:
                self.assertIn("output_cleanup_skipped", capture.report)
                self.assertEqual(bus.values[capture.isp + 4], 1)
                self.assertEqual(bus.values[capture.image], 1)

    def test_failed_standby_keeps_fsync_running_and_stops_other_sensor(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, sensors = self.fixture(directory)
            sensors[1].fail_stop = True
            report = self.run_capture(runner)
            self.assertFalse(report["ok"])
            self.assertFalse(report["generator_disabled_after_standby"])
            self.assertEqual(bus.values[DualBus.PAIR + 0x6c], 1)
            self.assertEqual(sensors[0].streaming, 0)
            self.assertEqual(bus.values[0], 0)

    def test_failed_generator_disable_is_not_reported_as_success(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fixture(directory)
            bus.fail_disable_generator = True
            report = self.run_capture(runner)
            self.assertFalse(report["ok"])
            self.assertFalse(report["generator_disabled_after_standby"])
            self.assertEqual(report["sensor_standby_verified"], [True, True])
            self.assertTrue(any("FSYNC disable failed" in error for error in report["cleanup_errors"]))

    def test_intentional_phase_offsets_validate_residual_not_raw_skew(self):
        for phases in ((0, 2072.525), (0, 4145.05), (4145.05, 0)):
            with self.subTest(phases=phases), tempfile.TemporaryDirectory() as directory:
                runner, bus, _ = self.fixture(directory)
                runner.phases_us = phases
                report = self.run_capture(runner)
                self.assertTrue(report["ok"], report)
                final = report["pair_capture_complete"]
                self.assertGreater(abs(final["last_skew_cycles"]), final["max_skew_limit_cycles"])
                self.assertEqual(final["residual_skew_cycles"], 5)
                self.assertEqual(final["last_skew_cycles"], final["phase1_cycles"] - final["phase0_cycles"] + 5)

    def test_60_fps_mode_selects_its_distinct_generator_period(self):
        from tools.lutcam.ov9281 import board_mode_writes
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, sensors = self.fixture(directory)
            runner.fps = 60
            for camera, sensor in enumerate(sensors):
                sensor.values.update(dict(board_mode_writes(camera, "off", fps=60)))
            report = self.run_capture(runner)
            self.assertTrue(report["ok"], report)
            self.assertEqual(report["generator"]["period_cycles"], 1990716)

    def test_stale_raw_evidence_cannot_validate_new_completed_pairs(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, _, _ = self.fixture(directory)
            report = self.run_capture(runner)
            self.assertTrue(report["ok"], report)
            report["pair_capture_complete"]["epoch"] = report["pair_before_arm"]["epoch"]
            report["pair_capture_complete"]["matched"] = 1
            errors = runner._validate_pair()
            self.assertTrue(any("does not cover" in error for error in errors))
            self.assertTrue(any("did not advance" in error for error in errors))

    def test_graceful_standby_settles_before_generator_disable(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fixture(directory)
            event_times = []
            original_write = bus.write
            def write(address, value):
                event_times.append((address, value, runner.clock()))
                original_write(address, value)
            bus.write = write
            report = self.run_capture(runner)
            self.assertTrue(report["ok"], report)
            stop = next(now for address, value, now in event_times if address == DualBus.PAIR + 8 and not value)
            disable = max(now for address, value, now in event_times if address == DualBus.PAIR + 0x6c and not value)
            self.assertGreaterEqual(disable - stop, 0.05)

    def test_missing_packet_does_not_claim_completed_pair(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, sensors = self.fixture(directory)
            sensors[0].packets.pop()
            report = self.run_capture(runner)
            self.assertFalse(report["ok"])
            self.assertIn("drain timeout", report["error"])
            self.assertNotIn("pair_capture_complete", report)
            self.assert_both_stopped(bus)

    def test_dual_runner_requires_one_shared_bus(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, bus, _ = self.fixture(directory)
            runner.captures[1].bus = DualBus()
            with self.assertRaises(ValueError):DualDiagnosticCapture(bus, runner.captures, seconds=1)


class DualCommandTests(unittest.TestCase):
    def arguments(self, output):
        return ["--address0", "0x60", "--address1", "0x10", "--destination", "192.168.1.251",
                "--output", str(output)]

    def test_connection_failure_preserves_both_reports(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            with patch("tools.lutcam.capture_dual.socket.socket"), patch(
                    "tools.lutcam.capture_dual.create_transport", side_effect=TimeoutError("link unavailable")), redirect_stdout(io.StringIO()):
                self.assertEqual(main(self.arguments(output)), 1)
            for path in (output / "report.json", output / "camera0/report.json", output / "camera1/report.json"):
                report = json.loads(path.read_text())
                self.assertFalse(report["ok"])
                self.assertIn("link unavailable", report["error"])

    def test_duplicate_image_ports_rejected_before_directory_or_connection(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(self.arguments(output) + ["--port1", "6000"])
            self.assertFalse(output.exists())

    def test_invalid_phases_rejected_before_directory_or_connection(self):
        for value in ("-1", "nan", "inf", "8290.1", "8290.0999"):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / "capture"
                with patch("tools.lutcam.capture_dual.create_transport") as transport, redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    main(self.arguments(output) + ["--phase0-us", value])
                transport.assert_not_called()
                self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
