"""Linked runtime exposure transactions; fake sensors only, no hardware I/O."""

from contextlib import contextmanager
import unittest

from tools.lutcam.exposure import (
    DEFAULT_GAIN_CODE, DEFAULT_LINES, LINE_TIME_US, MAX_GAIN_CODE, MAX_LINES,
    MIN_GAIN_CODE, MIN_LINES, ExposureUpdateError, apply_linked_exposure,
    validate_settings,
)


class GroupedSensor:
    """Stage bytes until END/LAUNCH, with failures before or after a write."""

    def __init__(self, camera, events):
        self.camera, self.address = camera, (0x60, 0x10)[camera]
        self.events = events
        self.i2c = self
        self.active = {0x3500: 0, 0x3501: 0x19, 0x3502: 0,
                       0x3508: 0, 0x3509: 16}
        self.staged = {}
        self.opened = self.ended = False
        self.fail_before, self.fail_after = {}, {}
        self.read_errors, self.read_values = {}, {}
        self.cancelled = False
        self.cancel_after = None
        self.cleanup_depth = 0

    @contextmanager
    def uncancellable(self):
        self.cleanup_depth += 1
        try:
            yield
        finally:
            self.cleanup_depth -= 1

    @staticmethod
    def _fail(table, key):
        error = table.get(key)
        if isinstance(error, list):
            error = error.pop(0) if error else None
        if error is not None:
            raise error

    def write_register(self, address, register, value):
        if address != self.address:
            raise AssertionError("wrong sensor address")
        if self.cancelled and not self.cleanup_depth:
            raise RuntimeError("cooperative cancellation")
        payload = bytes([value]) if type(value) is int else bytes(value)
        self.events.append(("write", self.camera, address, register, payload))
        key = (register, payload)
        self._fail(self.fail_before, key)
        if register == 0x3208:
            if payload == b"\x00":
                self.staged = {}
                self.opened, self.ended = True, False
            elif payload == b"\x10":
                self.opened, self.ended = False, True
            elif payload == b"\xa0":
                if not self.ended or self.opened:
                    raise AssertionError("launch before group end")
                self.active.update(self.staged)
                self.ended = False
            else:
                raise AssertionError("unexpected group command")
        else:
            if not self.opened:
                raise AssertionError("exposure payload outside an open group")
            self.staged.update({register + offset: byte
                                for offset, byte in enumerate(payload)})
        self._fail(self.fail_after, key)
        if self.cancel_after == key:
            self.cancelled = True
            raise RuntimeError("cooperative cancellation after completed write")

    def read_register(self, register, length=1):
        self.events.append(("read", self.camera, register, length))
        self._fail(self.read_errors, (register, length))
        return self.read_values.get((register, length), bytes(
            self.active.get(register + offset, 0) for offset in range(length)))


class LinkedExposureTests(unittest.TestCase):
    def fixture(self):
        self.events = []
        self.sensors = [GroupedSensor(camera, self.events) for camera in (0, 1)]
        return self.sensors

    def apply(self, lines=400, gain=16):
        return apply_linked_exposure(
            self.sensors, lines, gain,
            sleep=lambda seconds: self.events.append(("sleep", seconds)))

    def writes(self):
        return [event for event in self.events if event[0] == "write"]

    def group_events(self, value):
        return [event for event in self.writes()
                if event[3:] == (0x3208, bytes([value]))]

    def assert_uncertain(self, action):
        with self.assertRaises(ExposureUpdateError) as raised:
            action()
        self.assertIs(raised.exception.uncertain, True)
        self.assertIsInstance(raised.exception.cleanup_errors, tuple)
        return raised.exception

    def test_public_limits_match_fixed_120fps_mode(self):
        self.assertEqual((MIN_LINES, MAX_LINES, MIN_GAIN_CODE, MAX_GAIN_CODE),
                         (1, 885, 16, 255))
        self.assertEqual((DEFAULT_LINES, DEFAULT_GAIN_CODE), (400, 16))
        self.assertEqual(LINE_TIME_US, 9.1)

    def test_invalid_settings_are_rejected_before_any_sensor_access(self):
        invalid = [(0, 16), (886, 16), (-1, 16), (400, 15), (400, 256),
                   (True, 16), (False, 16), (400, True), (400, False),
                   (400.0, 16), (400, 16.0), ("400", 16), (400, "16"),
                   (None, 16), (400, None), (float("nan"), 16),
                   (400, float("inf")), (2**128, 16), (400, 2**128)]
        for lines, gain in invalid:
            with self.subTest(lines=lines, gain=gain):
                self.fixture()
                with self.assertRaises(ValueError):
                    validate_settings(lines, gain)
                with self.assertRaises(ValueError):
                    self.apply(lines, gain)
                self.assertEqual(self.events, [])

    def test_group_order_and_both_final_readbacks_before_success(self):
        self.fixture()
        self.assertIsNone(self.apply())
        expected = [
            ("write", 0, 0x60, 0x3208, b"\x00"),
            ("write", 1, 0x10, 0x3208, b"\x00"),
            ("write", 0, 0x60, 0x3500, b"\x00\x19\x00"),
            ("write", 0, 0x60, 0x3508, b"\x00\x10"),
            ("write", 1, 0x10, 0x3500, b"\x00\x19\x00"),
            ("write", 1, 0x10, 0x3508, b"\x00\x10"),
            ("write", 0, 0x60, 0x3208, b"\x10"),
            ("write", 1, 0x10, 0x3208, b"\x10"),
            ("write", 0, 0x60, 0x3208, b"\xa0"),
            ("write", 1, 0x10, 0x3208, b"\xa0"),
            ("sleep", 0.025),
            ("read", 0, 0x3500, 3), ("read", 0, 0x3508, 2),
            ("read", 1, 0x3500, 3), ("read", 1, 0x3508, 2),
        ]
        self.assertEqual(self.events, expected)

    def test_minimum_maximum_and_nonunity_gain_encoding(self):
        for lines, gain, packed in ((1, 16, b"\x00\x00\x10"),
                                    (885, 255, b"\x00\x37\x50"),
                                    (513, 37, b"\x00\x20\x10")):
            with self.subTest(lines=lines, gain=gain):
                self.fixture()
                self.assertIsNone(self.apply(lines, gain))
                for camera, address in ((0, 0x60), (1, 0x10)):
                    self.assertIn(("write", camera, address, 0x3500, packed), self.events)
                    self.assertIn(("write", camera, address, 0x3508, bytes([0, gain])), self.events)

    def test_payload_failure_ends_both_groups_and_never_launches(self):
        self.fixture()
        self.sensors[1].fail_before[(0x3508, b"\x00\x10")] = TimeoutError("gain write lost")
        self.assert_uncertain(self.apply)
        self.assertEqual({event[1] for event in self.group_events(0x10)}, {0, 1})
        self.assertEqual(self.group_events(0xa0), [])
        self.assertFalse(any(event[0] in ("read", "sleep") for event in self.events))

    def test_applied_open_with_lost_ack_is_still_closed(self):
        self.fixture()
        self.sensors[0].fail_after[(0x3208, b"\x00")] = TimeoutError("open acknowledgement lost")
        self.assert_uncertain(self.apply)
        self.assertEqual([event[1] for event in self.group_events(0x00)], [0])
        self.assertEqual([event[1] for event in self.group_events(0x10)], [0])
        self.assertEqual(self.group_events(0xa0), [])
        self.assertFalse(self.sensors[0].opened)

    def test_end_failure_does_not_skip_other_sensor_cleanup_or_launch(self):
        self.fixture()
        self.sensors[0].fail_before[(0x3208, b"\x10")] = TimeoutError("group end lost")
        error = self.assert_uncertain(self.apply)
        self.assertIn(1, [event[1] for event in self.group_events(0x10)])
        self.assertEqual(self.group_events(0xa0), [])
        self.assertTrue(error.cleanup_errors)
        self.assertFalse(self.sensors[1].opened)

    def test_cleanup_errors_do_not_replace_original_failure(self):
        self.fixture()
        self.sensors[1].fail_before[(0x3500, b"\x00\x19\x00")] = TimeoutError("original payload failure")
        for sensor in self.sensors:
            sensor.fail_before[(0x3208, b"\x10")] = RuntimeError("cleanup end failure")
        error = self.assert_uncertain(self.apply)
        self.assertEqual(len(error.cleanup_errors), 2)
        self.assertIn("original payload failure", str(error))
        self.assertEqual({event[1] for event in self.group_events(0x10)}, {0, 1})
        self.assertEqual(self.group_events(0xa0), [])

    def test_cleanup_end_bypasses_transaction_cancellation(self):
        self.fixture()
        self.sensors[0].cancel_after = (0x3500, b"\x00\x19\x00")
        error = self.assert_uncertain(self.apply)
        self.assertEqual(error.cleanup_errors, ())
        self.assertEqual({event[1] for event in self.group_events(0x10)}, {0, 1})
        self.assertEqual(self.group_events(0xa0), [])
        self.assertTrue(all(not sensor.opened and sensor.cleanup_depth == 0
                            for sensor in self.sensors))

    def test_launch_failure_is_uncertain_and_never_replayed(self):
        for camera in (0, 1):
            with self.subTest(camera=camera):
                self.fixture()
                self.sensors[camera].fail_after[(0x3208, b"\xa0")] = TimeoutError("launch acknowledgement lost")
                self.assert_uncertain(self.apply)
                self.assertEqual([event[1] for event in self.group_events(0xa0)],
                                 list(range(camera + 1)))
                self.assertFalse(any(event[0] == "read" for event in self.events))

    def test_readback_timeout_is_uncertain_after_both_launches(self):
        self.fixture()
        self.sensors[1].read_errors[(0x3508, 2)] = TimeoutError("readback lost")
        self.assert_uncertain(self.apply)
        self.assertEqual([event[1] for event in self.group_events(0xa0)], [0, 1])
        self.assertEqual(len([event for event in self.events if event[0] == "read"]), 4)

    def test_short_or_mismatched_readback_cannot_acknowledge_success(self):
        bad_values = ((0x3500, 3, b"\x00\x19"),
                      (0x3500, 3, b"\x00\x19\x01"),
                      (0x3508, 2, b"\x10"),
                      (0x3508, 2, b"\x01\x10"),
                      (0x3508, 2, b"\x00\x11"))
        for camera in (0, 1):
            for register, length, observed in bad_values:
                with self.subTest(camera=camera, register=register, observed=observed):
                    self.fixture()
                    self.sensors[camera].read_values[(register, length)] = observed
                    self.assert_uncertain(self.apply)
                    self.assertEqual([event[1] for event in self.group_events(0xa0)], [0, 1])


if __name__ == "__main__":
    unittest.main()
