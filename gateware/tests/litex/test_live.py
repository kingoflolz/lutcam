"""Live management lifecycle tests: mocks only, no board or network access."""

import io
import json
import threading
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from tools.lutcam.capture import CaptureError, DiagnosticCapture
from tools.lutcam.live import (
    CooperativeSensorIO, ExposureCommands, LiveCapture, OrderedManagementBus, StopRequested, json_event, main,
    route_destination, stdin_commands,
)
from tools.lutcam.paired import DRAINED, FAULT, LOCKED
from tools.lutcam.ov9281 import PAIR_BASE, PAIR_MAGIC
from tools.lutcam.exposure import ExposureUpdateError
from tools.lutcam.calibration_config import configure_camera, load_calibration


def exposure_command(command_id=1, lines=200, gain=32):
    return json.dumps({"command": "exposure", "id": command_id,
                       "exposure_lines": lines, "gain_code": gain})


class LiveTests(unittest.TestCase):
    def make_session(self, *, stop_on_running=True):
        self.events, self.actions = [], []
        self.now = 0
        self.stop = threading.Event()
        self.running = False
        self.fault = False
        self.registers = {0: 1, PAIR_BASE: PAIR_MAGIC, PAIR_BASE + 4: 1,
                          PAIR_BASE + 8: 0, PAIR_BASE + 0x0c: DRAINED | LOCKED,
                          PAIR_BASE + 0x14: 0}
        self.bus = Mock()
        self.bus.read.side_effect = lambda address: self.registers.get(address, 1)
        self.bus.write.side_effect = lambda address, value: self.registers.__setitem__(address, value)
        self.pair = Mock()
        self.pair.generator.side_effect = lambda enabled: self.actions.append(("generator", enabled)) or True
        self.pair.run.side_effect = self.set_run
        self.pair.read.side_effect = lambda offset: (DRAINED | LOCKED) if offset == 0x0c else int(self.running)
        self.pair.snapshot.return_value = {"epoch": 1}
        self.sensors, self.captures = [], []
        for camera in (0, 1):
            sensor = Mock()
            sensor._read_control.return_value = 5
            sensor.read_register.return_value = b"\0"
            sensor._write_register.side_effect = lambda register, value, c=camera: self.actions.append(("sensor", c, register, value))
            self.sensors.append(sensor)
            words = [0] * 16
            words[5] = 10
            base = 0x30024000 + camera * 0x100
            self.registers.update({base: 0, base + 0x14: 10, base + 0x38: 1, base + 0x3c: 0})
            capture = Mock(camera=camera, base=base, image=0x30024400 + camera * 0x180,
                           isp=0x30000000 + camera * 0x10000, port=6000 + 3 * camera,
                           destination=0xc0a801fb, bus=self.bus)
            capture.write.side_effect = lambda address, value, c=capture: DiagnosticCapture.write(c, address, value)
            capture.read_words.side_effect = lambda address, count: [
                self.registers.get(address + 4 * i, 0) for i in range(count)]
            capture.report = {"destination": "192.168.1.251"}
            capture.snapshot.return_value = {"camera": words, "image": [0] * 14, "isp": None}
            self.captures.append(capture)
        self.dual = Mock(pair=self.pair, sensors=self.sensors)
        self.dual.report = {"warmup": {"epoch": 0}}
        self.dual._pair_live.side_effect = lambda: {
            "run": int(self.running), "status": DRAINED | LOCKED | (FAULT if self.fault else 0),
            "fault": int(self.fault), "sync_enable": 0,
            "pairs_started": 4, "pairs_completed": 4}

        def emit(state, message, details=None):
            self.events.append({"state": state, "message": message, "details": details})
            if stop_on_running and state == "running":
                self.stop.set()

        self.session = LiveCapture(self.bus, self.captures, stop=self.stop, emit=emit,
                                   clock=lambda: self.now, sleep=self.sleep, dual=self.dual)
        return self.session

    def sleep(self, seconds):
        self.now += seconds

    def set_run(self, enabled):
        self.running = enabled
        self.actions.append(("run", enabled))

    def test_start_and_stop_order_and_no_isp_diagnostic_reads(self):
        session = self.make_session()
        self.assertEqual(session.run(), 0)
        self.assertEqual([e["state"] for e in self.events], ["starting", "running", "stopping", "stopped"])
        initial_exposure = self.events[1]["details"]["exposure"]
        self.assertEqual(initial_exposure, {"id": 0, "exposure_lines": 400, "gain_code": 16,
            "line_time_us": 9.1, "min_lines": 1, "max_lines": 885,
            "min_gain_code": 16, "max_gain_code": 255})
        for sensor in self.sensors:
            sensor.configure.assert_called_once()
            self.assertEqual(sensor.configure.call_args.kwargs["pattern"], "off")
            self.assertEqual(sensor.configure.call_args.kwargs["fps"], 120)
            sensor.power_on.assert_not_called()
        self.pair.configure_generator.assert_called_once_with(120, (0, 0))
        self.assertLess(self.actions.index(("run", False)), self.actions.index(("sensor", 1, 0x100, 0)))
        self.assertLess(self.actions.index(("sensor", 0, 0x100, 0)), len(self.actions) - 1)
        self.assertEqual(self.actions[-1], ("generator", False))
        for capture in self.captures:
            self.assertTrue(all(call.kwargs == {"include_isp": False} for call in capture.snapshot.call_args_list))
            capture.write.assert_any_call(capture.isp + 4, 0)
            for channel in range(3):
                capture.write.assert_any_call(capture.image + channel * 0x80, 0)

    def test_busy_pair_is_never_owned_or_stopped(self):
        session = self.make_session()
        self.pair.require_idle.side_effect = CaptureError("already active")
        self.assertEqual(session.run(), 1)
        self.assertFalse(session.owned)
        self.pair.run.assert_not_called()
        self.pair.generator.assert_not_called()
        for sensor in self.sensors:
            sensor.configure.assert_not_called()
            sensor._write_register.assert_not_called()
        self.assertEqual(self.events[-1]["state"], "error")

    def test_calibration_and_products_are_verified_after_isp_setup_before_arm(self):
        session = self.make_session()
        for camera, capture in enumerate(self.captures):
            capture.configure.side_effect = lambda c=camera: self.actions.append(("configure", c))

        def configure(capture, calibration, **kwargs):
            self.assertFalse(self.running)
            result = configure_camera(capture, calibration, **kwargs)
            self.actions.append(("calibration", capture.camera))
            return result

        with patch("tools.lutcam.live.configure_camera", side_effect=configure):
            self.assertEqual(session.run(), 0)
        self.assertLess(self.actions.index(("configure", 1)), self.actions.index(("calibration", 0)))
        self.assertLess(self.actions.index(("calibration", 1)), self.actions.index(("generator", True)))
        self.assertLess(self.actions.index(("calibration", 1)), self.actions.index(("run", True)))
        report = self.events[1]["details"]["calibration"]
        self.assertTrue(report["readback_verified"])
        self.assertEqual(report["provenance"]["kind"], "nominal_lens")
        self.assertTrue(report["centroid_stream"])
        self.assertTrue(report["pose_stream"])
        for camera, item in enumerate(report["cameras"]):
            self.assertEqual(item["camera"], camera)
            self.assertEqual(item["requested"]["fx"], 2.8 / 0.003)
            self.assertEqual(item["cx"], 639.5)
            self.assertEqual(item["cy"], 399.5)
            self.assertEqual(item["port"], self.captures[camera].port)
            self.assertEqual(item["products"], [1, 2, 3])
            self.assertTrue(item["calibration_commit_verified"])
        session.emit("running", "later status", {})
        self.assertEqual(self.events[-1]["details"]["calibration"], report)

    def test_failed_calibration_commit_cleans_up_without_enabling_fsync_or_arm(self):
        session = self.make_session()
        self.registers[self.captures[1].base + 0x3c] = 1
        self.assertEqual(session.run(), 1)
        self.assertNotIn(("run", True), self.actions)
        self.assertNotIn(("generator", True), self.actions)
        self.assertFalse(any(event["state"] == "running" for event in self.events))
        self.assertIn("calibration commit failed", self.events[-1]["message"])
        self.assertFalse(session.calibration["readback_verified"])
        for capture in self.captures:
            for channel in range(3):
                capture.write.assert_any_call(capture.image + channel * 0x80, 0)

    def test_stop_during_calibration_cleans_up_without_committing_partial_bank(self):
        session = self.make_session()
        capture = self.captures[0]
        original = capture.write.side_effect

        def write(address, value):
            original(address, value)
            if address == capture.base + 0x80:
                self.stop.set()

        capture.write.side_effect = write
        self.assertEqual(session.run(), 0)
        self.assertNotIn(("run", True), self.actions)
        self.assertNotIn(("generator", True), self.actions)
        self.assertFalse(any(call.args[0] == capture.base + 0x34 for call in self.bus.write.call_args_list))
        self.assertEqual(self.events[-1]["state"], "stopped")

    def test_streaming_sensor_is_not_stopped_even_if_pair_is_idle(self):
        session = self.make_session()
        self.sensors[1].read_register.return_value = b"\1"
        self.assertEqual(session.run(), 1)
        self.assertFalse(session.owned)
        self.assertEqual(self.actions, [])

    def test_prestart_stop_is_clean_and_has_no_mutations(self):
        session = self.make_session()
        self.stop.set()
        self.assertEqual(session.run(), 0)
        self.assertEqual(self.actions, [])
        self.assertEqual(self.events[-1]["state"], "stopped")

    def test_stop_during_configuration_cleans_partial_configuration(self):
        session = self.make_session()

        def configure(**kwargs):
            self.stop.set()
            kwargs["progress"](1, 100)

        self.sensors[0].configure.side_effect = configure
        self.assertEqual(session.run(), 0)
        self.sensors[1].configure.assert_not_called()
        self.assertNotIn(("run", True), self.actions)
        self.assertNotIn(("run", False), self.actions)  # Already stopped; do not manufacture a health fault.
        self.assertIn(("sensor", 0, 0x100, 0), self.actions)
        self.assertIn(("sensor", 1, 0x100, 0), self.actions)
        self.assertEqual(self.events[-1]["state"], "stopped")

    def test_power_on_only_for_sensor_that_needs_it(self):
        session = self.make_session()
        self.sensors[0]._read_control.side_effect = [4, 5]
        self.assertEqual(session.run(), 0)
        self.sensors[0].power_on.assert_called_once_with()
        self.sensors[1].power_on.assert_not_called()

    def test_hardware_fault_keeps_terminal_fault_after_preserved_reset(self):
        session = self.make_session(stop_on_running=False)
        self.fault = True
        self.assertEqual(session.run(), 1)
        states = [e["state"] for e in self.events]
        self.assertEqual(states[-1], "fault")
        self.assertNotIn("stopped", states)
        self.assertEqual(states.count("fault"), 2)
        self.assertTrue(self.events[-1]["details"]["cleanup"]["reset_recovered"])
        evidence_index = next(i for i, event in enumerate(self.events)
                              if event["message"] == "Retained diagnostics before paired reset")
        self.assertTrue(self.events[evidence_index]["details"]["pair"]["fault"])
        self.assertEqual(self.pair.reset.call_count, 2)

    def test_nonfatal_candidate_protocol_warning_does_not_stop_capture(self):
        session = self.make_session()
        self.captures[0].snapshot.return_value["camera"][5] |= 64
        state = session._snapshot()
        state["pair"]["run"] = 1
        self.assertFalse(session._faulted(state))
        self.assertEqual(len(state["warnings"]), 1)

    def test_product_diagnostics_survive_fault_without_reading_isp(self):
        session = self.make_session()
        expected = []
        for camera, capture in enumerate(self.captures):
            capture.snapshot.return_value["camera"][5] |= 4
            for channel, count in ((1, 14), (2, 15)):
                base = capture.image + channel * 0x80
                words = [camera * 1000 + channel * 100 + i for i in range(count)]
                for i, value in enumerate(words):
                    self.registers[base + i * 4] = value
                expected.append((camera, channel, base, count, words))
        writes_before = self.bus.write.call_count
        snapshot = session._snapshot()
        self.assertEqual(self.bus.write.call_count, writes_before)
        for camera, channel, base, count, words in expected:
            capture = self.captures[camera]
            capture.read_words.assert_any_call(base, count)
            product = snapshot["cameras"][camera]["products"][("centroids", "pose")[channel - 1]]
            self.assertEqual(product["faults"], words[7])
            self.assertEqual(product["overflows"], words[12])
            self.assertEqual(product["protocol_errors"], words[13])
            self.assertEqual(product["fifo_level"], words[5])
            self.assertEqual(product["packets_committed"], words[9])
            self.assertEqual(product["packets_sent"], words[10])
            self.assertNotIn("clear", product)
            if channel == 2:
                self.assertEqual(product["rejections"], words[14])
            else:
                self.assertNotIn("rejections", product)
        for capture in self.captures:
            capture.snapshot.assert_called_once_with(include_isp=False)
            self.assertEqual(capture.read_words.call_count, 2)
            self.assertTrue(all(capture.image + 0x80 <= call.args[0] <= capture.image + 0x100
                                for call in capture.read_words.call_args_list))

    def test_product_fault_evidence_is_retained_before_cleanup_reset(self):
        session = self.make_session(stop_on_running=False)
        self.fault = True
        child = self.captures[0].image + 0x80
        self.registers[child + 0x1c] = 2
        self.registers[child + 0x34] = 7
        self.assertEqual(session.run(), 1)
        final = self.events[-1]
        self.assertEqual(final["state"], "fault")
        evidence = final["details"]["cleanup"]["before_reset"]["cameras"][0]["products"]["centroids"]
        self.assertEqual(evidence["faults"], 2)
        self.assertEqual(evidence["protocol_errors"], 7)
        self.assertTrue(final["details"]["cleanup"]["reset_recovered"])

    def test_truncated_product_diagnostics_are_not_reported_as_clean(self):
        session = self.make_session()
        self.captures[0].read_words.return_value = [0]
        self.captures[0].read_words.side_effect = None
        with self.assertRaisesRegex(CaptureError, "centroids diagnostics were truncated"):
            session._snapshot()

    def test_stop_before_next_poll_preserves_fault_discovered_during_cleanup(self):
        session = self.make_session()
        self.fault = True  # Stop is requested by the first running event, before a poll.
        self.assertEqual(session.run(), 1)
        self.assertEqual(self.events[-1]["state"], "fault")
        self.assertTrue(self.events[-1]["details"]["cleanup"]["hardware_fault"])
        self.assertTrue(self.events[-1]["details"]["cleanup"]["reset_recovered"])

    def test_standby_line_truncation_alone_is_not_reported_as_capture_failure(self):
        session = self.make_session()
        original = session._output

        def emit(state, message, details=None):
            original(state, message, details)
            if state == "running":
                self.captures[0].snapshot.return_value["camera"][6] = 0x400

        session._output = emit
        self.assertEqual(session.run(), 0)
        self.assertEqual(self.events[-1]["state"], "stopped")
        self.assertFalse(self.events[-1]["details"]["cleanup"]["hardware_fault"])
        self.assertTrue(self.events[-1]["details"]["cleanup"]["reset_recovered"])

    def test_standby_failure_leaves_fsync_and_reset_untouched(self):
        session = self.make_session()
        normal = self.sensors[1]._write_register.side_effect

        def write(register, value):
            if value == 0:
                raise TimeoutError("standby uncertain")
            return normal(register, value)

        self.sensors[1]._write_register.side_effect = write
        self.assertEqual(session.run(), 1)
        self.assertEqual(self.pair.generator.call_args_list[-1].args, (True,))
        self.assertEqual(self.pair.reset.call_count, 1)
        self.assertEqual(self.events[-1]["state"], "error")
        for capture in self.captures:
            capture.write.assert_any_call(capture.image + 0x100, 1)  # Startup pose enable succeeded.
            self.assertFalse(any(call.args == (capture.isp + 4, 0) for call in capture.write.call_args_list))
            self.assertFalse(any(call.args == (capture.image, 0) for call in capture.write.call_args_list))

    def test_drain_timeout_prevents_reset_and_isp_access(self):
        session = self.make_session()
        self.pair.read.side_effect = lambda offset: LOCKED if offset == 0x0c else 0
        self.assertEqual(session.run(), 1)
        self.assertGreaterEqual(self.now, 1)
        self.assertLess(self.now, 1.1)
        self.assertEqual(self.pair.reset.call_count, 1)
        for capture in self.captures:
            self.assertFalse(any(call.args == (capture.isp + 4, 0) for call in capture.write.call_args_list))

    def test_uncertain_arm_completion_still_stops_owned_capture(self):
        session = self.make_session()

        def run(enabled):
            self.set_run(enabled)
            if enabled:
                raise TimeoutError("arm reply lost")

        self.pair.run.side_effect = run
        self.assertEqual(session.run(), 1)
        self.assertEqual([action for action in self.actions if action[0] == "run"],
                         [("run", True), ("run", False)])

    def test_unknown_stop_state_is_not_assumed_idle(self):
        session = self.make_session()
        self.pair.read.side_effect = [TimeoutError("stop precheck lost"), DRAINED | LOCKED, 0]
        self.assertEqual(session.run(), 0)
        self.assertIn(("run", False), self.actions)
        self.assertIn("stop_state_error", self.events[-1]["details"]["cleanup"])

    def test_closed_viewer_stdout_cannot_prevent_owned_cleanup(self):
        session = self.make_session(stop_on_running=False)
        original = session._output

        def closed_after_start(state, message, details=None):
            if state != "starting":
                raise BrokenPipeError("viewer exited")
            original(state, message, details)

        session._output = closed_after_start
        self.assertEqual(session.run(), 0)
        self.assertIn(("run", False), self.actions)
        for camera in (0, 1):
            self.assertIn(("sensor", camera, 0x100, 0), self.actions)
        self.assertEqual(self.actions[-1], ("generator", False))

    def test_disarmed_warmup_uses_hardware_packet_counters(self):
        session = self.make_session()
        self.captures[0].snapshot.return_value["image"][9] = 1
        self.assertEqual(session.run(), 1)
        self.assertNotIn(("run", True), self.actions)
        self.pair.clear.assert_not_called()
        self.assertIn("while paired capture was disarmed", self.events[-1]["message"])

    def test_exposure_ack_is_published_only_after_setter_and_retained(self):
        session = self.make_session(stop_on_running=False)
        session.commands.submit(exposure_command(7, 123, 40))
        caller = threading.get_ident()

        def apply(sensors, lines, gain, **kwargs):
            self.assertEqual(threading.get_ident(), caller)
            self.assertEqual((sensors, lines, gain), (self.sensors, 123, 40))
            self.assertEqual(session.exposure["id"], 0)
            self.assertEqual(self.events, [])

        with patch("tools.lutcam.live.apply_linked_exposure", side_effect=apply) as setter:
            session._process_exposure()
        setter.assert_called_once()
        session.emit("running", "periodic", {})
        self.assertTrue(all(e["details"]["exposure"]["id"] == 7 for e in self.events))
        self.assertTrue(all(e["details"]["exposure"]["exposure_lines"] == 123 for e in self.events))

    def test_firmware_owns_sensor_and_isp_initialization_and_shutdown(self):
        session = self.make_session()
        firmware = Mock()
        firmware.snapshot.return_value = dict(exposure_lines=400, gain_code=16,
            enabled=True, ev_compensation=0.0, p90=[180,190], heartbeat=7)
        with patch("tools.lutcam.live.CameraSupervisor.discover", return_value=firmware):
            self.assertEqual(session.run(), 0)
        self.assertEqual([call.args for call in firmware.command.call_args_list],
                         [(1,), (2,), (3,), (4,)])
        for sensor in self.sensors:
            sensor.configure.assert_not_called()
            sensor.power_on.assert_not_called()
            sensor._write_register.assert_not_called()
        for capture in self.captures:
            capture.configure.assert_called_once_with(firmware_isp=True)
        self.assertEqual(self.events[1]["details"]["firmware_exposure"]["p90"], [180,190])

    def test_firmware_auto_command_does_not_run_host_sensor_setter(self):
        session = self.make_session(stop_on_running=False)
        session.supervisor = Mock()
        session.supervisor.snapshot.return_value = dict(exposure_lines=500, gain_code=16,
            enabled=False, ev_compensation=-0.75, p90=[100,105], heartbeat=7)
        session.commands.submit(json.dumps(dict(command="auto_exposure", id=0, enabled=False, ev_quarters=-3)))
        with patch("tools.lutcam.live.apply_linked_exposure") as setter:
            session._process_exposure()
        setter.assert_not_called()
        session.supervisor.set_auto_exposure.assert_called_once_with(False, -3)
        self.assertEqual(self.events[-1]["details"]["exposure"]["exposure_lines"], 500)

    def test_latest_exposure_arriving_during_update_remains_pending(self):
        session = self.make_session(stop_on_running=False)
        session.commands.submit(exposure_command(1))

        def apply(*args, **kwargs):
            session.commands.submit(exposure_command(2, 300, 48))

        with patch("tools.lutcam.live.apply_linked_exposure", side_effect=apply):
            session._process_exposure()
        self.assertEqual(session.exposure["id"], 1)
        with patch("tools.lutcam.live.apply_linked_exposure") as setter:
            session._process_exposure()
        self.assertEqual(session.exposure["id"], 2)
        self.assertEqual(setter.call_args.args[1:], (300, 48))

    def test_uncertain_exposure_keeps_capture_and_disables_further_changes(self):
        session = self.make_session(stop_on_running=False)
        session.commands.submit(exposure_command(9))
        with patch("tools.lutcam.live.apply_linked_exposure",
                   side_effect=ExposureUpdateError("camera1 readback failed")) as setter:
            session._process_exposure()
            session.commands.submit(exposure_command(10))
            session._process_exposure()
        setter.assert_called_once()
        self.assertFalse(self.stop.is_set())
        self.pair.run.assert_not_called()
        self.assertEqual(session.exposure["id"], 0)
        self.assertEqual(self.events[-1]["state"], "running")
        self.assertEqual(self.events[-1]["details"]["exposure_error"]["id"], 10)
        self.assertTrue(self.events[-1]["details"]["exposure_error"]["disabled"])
        session.emit("running", "periodic")
        self.assertEqual(self.events[-1]["details"]["exposure_error"]["id"], 10)

    def test_invalid_exposure_is_nonfatal_and_does_not_disable_valid_updates(self):
        session = self.make_session(stop_on_running=False)
        session.commands.submit(exposure_command(11, 9999))
        with patch("tools.lutcam.live.apply_linked_exposure") as setter:
            session._process_exposure()
            setter.assert_not_called()
            self.assertFalse(session.exposure_disabled)
            self.assertEqual(self.events[-1]["details"]["exposure_error"]["id"], 11)
            session.commands.submit(exposure_command(12))
            session._process_exposure()
        self.assertEqual(session.exposure["id"], 12)
        self.assertNotIn("exposure_error", self.events[-1]["details"])

    def test_stop_cancels_pending_exposure_before_any_sensor_io(self):
        session = self.make_session(stop_on_running=False)
        session.commands.submit(exposure_command())
        self.stop.set()
        with patch("tools.lutcam.live.apply_linked_exposure") as setter:
            with self.assertRaises(StopRequested):
                session._process_exposure()
        setter.assert_not_called()

    def test_stop_during_group_payload_closes_both_groups_without_launch(self):
        session = self.make_session(stop_on_running=False)
        operations = []
        for camera, sensor in enumerate(self.sensors):
            sensor.camera, sensor.address = camera, (0x60, 0x10)[camera]

            def write(address, register, value, c=camera):
                operations.append((c, register, value))
                if c == 0 and register == 0x3500:
                    self.stop.set()

            sensor.i2c.inner.write_register.side_effect = write
        session.commands.submit(exposure_command())
        with self.assertRaises(StopRequested):
            session._process_exposure()
        self.assertEqual(operations[-2:], [(0, 0x3208, 0x10), (1, 0x3208, 0x10)])
        self.assertFalse(any(register == 0x3208 and value == 0xa0 for _, register, value in operations))
        self.assertEqual(session.exposure["id"], 0)
        self.assertTrue(session.exposure_disabled)


class LiveInterfaceTests(unittest.TestCase):
    def test_sensor_cancellation_occurs_after_whole_transaction(self):
        stop = threading.Event()
        inner = Mock()
        inner.read_register.side_effect = lambda *args: stop.set() or b"\0"

        def check():
            if stop.is_set():
                raise StopRequested()

        wrapper = CooperativeSensorIO(inner, check)
        with self.assertRaises(StopRequested):
            wrapper.read_register(0x10, 0x300a, 2)
        inner.read_register.assert_called_once_with(0x10, 0x300a, 2)
        with self.assertRaises(StopRequested):
            wrapper.write_register(0x10, 0x100, 0)
        inner.write_register.assert_not_called()

    def test_stdin_stop_and_eof(self):
        for content in ("stop\n", " STOP \n", ""):
            stop = threading.Event()
            stdin_commands(io.StringIO(content), stop)
            self.assertTrue(stop.is_set())

    def test_json_event_has_one_parseable_line(self):
        stream = io.StringIO()
        with patch("sys.stdout", stream):
            json_event("fault", "line1\nline2", {"camera": 0})
        self.assertEqual(len(stream.getvalue().splitlines()), 1)
        self.assertEqual(json.loads(stream.getvalue())["state"], "fault")

    def test_route_selection_connects_udp_without_sending(self):
        probe = Mock()
        probe.getsockname.return_value = ("192.168.1.251", 54321)
        with patch("tools.lutcam.live.socket.socket") as factory:
            factory.return_value.__enter__.return_value = probe
            self.assertEqual(route_destination("192.168.1.50"), "192.168.1.251")
        probe.connect.assert_called_once_with(("192.168.1.50", 1234))
        probe.send.assert_not_called()
        probe.sendto.assert_not_called()

    def test_bad_ports_rejected_before_transport_access(self):
        with patch("tools.lutcam.live.create_transport") as transport:
            with patch("tools.lutcam.live.json_event") as event:
                self.assertEqual(main(["--port0", "6000", "--port1", "6000"]), 1)
        transport.assert_not_called()
        self.assertEqual(event.call_args.args[0], "error")

    def test_cli_camera_addresses_and_viewer_ports_are_not_bound(self):
        with patch("tools.lutcam.live.create_transport") as transport, \
             patch("tools.lutcam.live.socket.socket") as sockets, \
             patch("tools.lutcam.live.signal.signal"), \
             patch("tools.lutcam.live.threading.Thread"), \
             patch("tools.lutcam.live.DiagnosticCapture") as capture, \
             patch("tools.lutcam.live.LiveCapture") as manager:
            manager.return_value.run.return_value = 0
            self.assertEqual(main(["--destination", "192.168.1.251"]), 0)
        self.assertEqual([call.kwargs["address"] for call in capture.call_args_list], [0x60, 0x10])
        self.assertEqual([call.kwargs["port"] for call in capture.call_args_list], [6000, 6003])
        self.assertEqual([call.kwargs["camera"] for call in capture.call_args_list], [0, 1])
        self.assertEqual(transport.call_args.args[0].transport, "etherbone")
        self.assertEqual(sockets.return_value.__enter__.return_value.bind.call_args_list[0].args,
                         (("0.0.0.0", 0),))
        self.assertEqual(sockets.return_value.__enter__.return_value.bind.call_count, 2)
        self.assertEqual(manager.call_args.kwargs["calibration"], load_calibration())

    def test_invalid_calibration_rejected_before_transport_or_stdin_thread(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.json"
            path.write_text('{"schema_version":2}')
            with patch("tools.lutcam.live.create_transport") as transport, \
                 patch("tools.lutcam.live.threading.Thread") as thread, \
                 patch("tools.lutcam.live.json_event") as event:
                self.assertEqual(main(["--calibration", str(path)]), 1)
            transport.assert_not_called()
            thread.assert_not_called()
            self.assertEqual(event.call_args.args[0], "error")
            self.assertIn("Calibration:", event.call_args.args[1])

    def test_cli_supplied_calibration_passed_to_session(self):
        value = {"schema_version": 1, "image_size": [1280, 800], "baseline_mm": 120,
                 "tag_width_mm": 88,
                 "cameras": [dict(fx=900 + c, fy=910 + c, cx=639, cy=399,
                                  distortion=[0.1, -0.1, 0, 0, 0]) for c in range(2)]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fitted.json"
            path.write_text(json.dumps(value))
            with patch("tools.lutcam.live.create_transport"), \
                 patch("tools.lutcam.live.socket.socket"), \
                 patch("tools.lutcam.live.signal.signal"), \
                 patch("tools.lutcam.live.threading.Thread"), \
                 patch("tools.lutcam.live.DiagnosticCapture"), \
                 patch("tools.lutcam.live.LiveCapture") as manager:
                manager.return_value.run.return_value = 0
                self.assertEqual(main(["--destination", "192.168.1.251", "--calibration", str(path)]), 0)
            report = manager.call_args.kwargs["calibration"]
            self.assertEqual(report["provenance"]["path"], str(path))
            self.assertEqual(report["tag_width_mm"], 88)
            self.assertEqual(report["tag_width"], 1)
            self.assertFalse(report["distortion_applied"])
            self.assertEqual([c["requested"]["fx"] for c in report["cameras"]], [900, 901])


class ExposureCommandTests(unittest.TestCase):
    def test_only_latest_valid_command_is_retained(self):
        commands = ExposureCommands()
        for command_id in range(1000):
            commands.submit(exposure_command(command_id))
        pending, error = commands.take()
        self.assertEqual(pending["id"], 999)
        self.assertIsNone(error)
        self.assertEqual(commands.take(), (None, None))

    def test_invalid_commands_preserve_valid_pending_without_sensor_access(self):
        invalid = ["not json", "[]", exposure_command(True), exposure_command(1 << 32),
                   exposure_command(1, 0), exposure_command(1, 886), exposure_command(1, True),
                   exposure_command(1, 2.0), exposure_command(1, 2, 15), exposure_command(1, 2, 256),
                   '{"command":"reset","id":9}', "[" * 1024, "x" * 1025]
        for line in invalid:
            with self.subTest(line=line[:80]):
                commands = ExposureCommands()
                commands.submit(exposure_command(17))
                commands.submit(line)
                pending, error = commands.take()
                self.assertEqual(pending["id"], 17)
                self.assertIsNotNone(error)

    def test_oversized_physical_line_is_drained_not_reinterpreted(self):
        commands, stop = ExposureCommands(), threading.Event()
        commands.submit = Mock(wraps=commands.submit)
        commands.reject = Mock(wraps=commands.reject)
        # The embedded JSON suffix is part of the oversized physical line.
        stream = io.StringIO("x" * 1025 + exposure_command(1) + "\n" + exposure_command(2) + "\nstop\n")
        stdin_commands(stream, stop, commands)
        self.assertTrue(stop.is_set())
        commands.submit.assert_called_once_with(exposure_command(2) + "\n")
        commands.reject.assert_called_once()
        self.assertEqual(commands.take(), (None, None))

    def test_broken_stdin_requests_stop_and_discards_pending(self):
        commands, stop = ExposureCommands(), threading.Event()
        commands.submit(exposure_command())
        stdin_commands(Mock(readline=Mock(side_effect=OSError("closed"))), stop, commands)
        self.assertTrue(stop.is_set())
        self.assertEqual(commands.take(), (None, None))


class OrderedManagementTests(unittest.TestCase):
    def test_configuration_reproduces_clear_then_destination_drop_and_fence_fixes_it(self):
        class CheckedFrameBus:
            """One receive bank: bare writes overlap until a read completes."""
            def __init__(self):
                self.pending = False
                self.dropped = []
                self.mixed = []
                self.values = {PAIR_BASE: PAIR_MAGIC, 0x30000000: 0x4d495350,
                               0x30000008: 4, 0x30024038: 1}

            def supports_ordered_write_read(self):
                return True

            def write(self, address, value):
                if self.pending:
                    self.dropped.append(address)
                else:
                    self.values[address] = value
                    self.pending = True

            def read(self, address, length=None):
                self.pending = False  # Completed/retried ordinary read.
                return self.values.get(address, 0)

            def write_read(self, address, value, read_address):
                self.mixed.append((address, value, read_address))
                if self.pending:
                    raise TimeoutError("receiver occupied")
                self.write(address, value)
                return self.read(read_address)

        def capture(bus):
            return DiagnosticCapture(bus, Mock(), Mock(), camera=0, address=0x60,
                destination="192.168.1.251", port=6000, board_ip="192.168.1.50", seconds=1)

        with patch("tools.lutcam.capture.require_pair_idle"):
            raw = CheckedFrameBus()
            with self.assertRaisesRegex(CaptureError, "0x30024404"):
                capture(raw).configure()
            self.assertIn(0x30024404, raw.dropped)
            raw = CheckedFrameBus()
            capture(OrderedManagementBus(raw, require_ordered=True)).configure()
        self.assertEqual(raw.dropped, [])
        self.assertEqual(raw.values[0x30024404], 0xc0a801fb)
        self.assertEqual(raw.values[0x30024408], 6000)
        self.assertEqual(raw.values[0x30024400], 1)
        self.assertTrue(all(read_address == PAIR_BASE for _, _, read_address in raw.mixed))
        self.assertTrue(all(any(address == clear for address, _, _ in raw.mixed)
                            for clear in (0x30024420, 0x300244a0, 0x30024520)))

    def test_write_only_pulse_timeout_never_replays(self):
        raw = Mock()
        raw.supports_ordered_write_read.return_value = True
        raw.write_read.side_effect = TimeoutError("uncertain clear completion")
        bus = OrderedManagementBus(raw, require_ordered=True)
        with self.assertRaises(TimeoutError):
            bus.write(PAIR_BASE + 0x60, 1)
        raw.write_read.assert_called_once_with(PAIR_BASE + 0x60, 1, PAIR_BASE)
        raw.write.assert_not_called()

    def test_invalid_fence_response_is_not_success(self):
        for value in (0, 1, 0xffffffff):
            with self.subTest(value=value):
                raw = Mock()
                raw.supports_ordered_write_read.return_value = True
                raw.write_read.return_value = value
                with self.assertRaises(CaptureError):
                    OrderedManagementBus(raw).write(0x30024404, 0xc0a801fb)
                self.assertEqual(raw.write_read.call_count, 1)

    def test_unrecognized_etherbone_bridge_rejected_before_mutation(self):
        raw = Mock()
        raw.supports_ordered_write_read.return_value = False
        with self.assertRaises(CaptureError):
            OrderedManagementBus(raw, require_ordered=True)
        raw.write.assert_not_called()
        raw.write_read.assert_not_called()

    def test_usb_ordering_and_i2c_mixed_calls_remain_unchanged(self):
        serial = Mock(spec=["read", "write"])
        OrderedManagementBus(serial).write(PAIR_BASE + 0x10, 1)
        serial.write.assert_called_once_with(PAIR_BASE + 0x10, 1)
        raw = Mock()
        raw.supports_ordered_write_read.return_value = True
        raw.write_read.return_value = 3
        bus = OrderedManagementBus(raw)
        self.assertEqual(bus.write_read(0x30020000, 3, 0x30020000), 3)
        raw.write_read.assert_called_once_with(0x30020000, 3, 0x30020000)


if __name__ == "__main__":
    unittest.main()
