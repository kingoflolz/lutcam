"""OV9281 host sequencing and refusal paths; no hardware connection is opened.

The fake sensor is a separate byte-addressed register map with injectable NACKs
and readback corruption. Tests check observable I/O ordering and independently
specified mode dimensions/clock divisors, rather than helper internals.
"""

from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.lutcam.control import I2CNack
from tools.lutcam.ov9281 import (
    CameraError, MODE_FINAL, MODE_NAME, MODE_WRITES, OV9281, main, mode_manifest,
    mode_writes, board_mode_writes, board_mode_manifest, PAIR_BASE, PAIR_MAGIC,
    PAIR_RUN, PAIR_STATUS, PAIR_FAULT, PAIR_STATUS_DRAINED, PAIR_STATUS_BUSY, PAIR_STATUS_FAULT,
)


class CameraBus:
    def __init__(self, events):
        self.events = events
        self.values = {}
        self.corrupt_write = None
        self.values.update({PAIR_BASE: PAIR_MAGIC, PAIR_BASE + 4: 0,
                            PAIR_BASE + PAIR_RUN: 0,
                            PAIR_BASE + PAIR_STATUS: PAIR_STATUS_DRAINED,
                            PAIR_BASE + PAIR_FAULT: 0})
        for camera in (0, 1):
            base = 0x30024000 + camera * 0x100
            self.values.update({base: 0, base + 0x0c: 5,
                                base + 0x14: 2 | 8, base + 0x18: 0})
            self.values[0x30020004 + camera * 0x1000] = 3

    def read(self, address):
        value = self.values.get(address, 0)
        self.events.append(("bus-read", address, value))
        return value

    def write(self, address, value):
        self.events.append(("bus-write", address, value))
        self.values[address] = value ^ int(address == self.corrupt_write)


class RegisterSensor:
    def __init__(self, events):
        self.events = events
        self.values = {0x300a: 0x92, 0x300b: 0x81, 0x0100: 0}
        self.nack_write = None
        self.corrupt_write = None
        self.corrupt_write_occurrence = None
        self.write_counts = {}
        self.read_hook = None

    def release(self):
        self.events.append(("release",))

    def read_register(self, device, register, length=1):
        if self.read_hook:
            self.read_hook(register, length)
        result = bytes(self.values.get(register + offset, 0)
                       for offset in range(length))
        self.events.append(("sensor-read", device, register, length, result))
        return result

    def write_register(self, device, register, data):
        payload = bytes([data]) if isinstance(data, int) else bytes(data)
        self.events.append(("sensor-write", device, register, payload))
        if register == self.nack_write:
            raise I2CNack("injected sensor NACK")
        self.write_counts[register] = self.write_counts.get(register, 0) + 1
        corrupt = (register == self.corrupt_write and
                   (self.corrupt_write_occurrence is None or
                    self.write_counts[register] == self.corrupt_write_occurrence))
        for offset, value in enumerate(payload):
            self.values[register + offset] = value ^ int(corrupt)


class OV9281Tests(unittest.TestCase):
    def fixture(self, *, camera=1, address=0x10):
        events = []
        bus = CameraBus(events)
        sensor = RegisterSensor(events)
        helper = OV9281(bus, camera=camera, address=address, i2c=sensor,
                        sleep=lambda duration: events.append(("sleep", duration)),
                        clock=lambda: 0.0)
        return helper, bus, sensor, events

    @staticmethod
    def sensor_writes(events):
        return [event for event in events if event[0] == "sensor-write"]

    @staticmethod
    def any_writes(events):
        return [event for event in events if event[0].endswith("write")]

    def test_constructor_has_no_io_and_requires_explicit_selection(self):
        _, _, _, events = self.fixture()
        self.assertEqual(events, [])
        for arguments in ({}, {"camera": 0}, {"address": 0x10}):
            with self.subTest(arguments=arguments):
                with self.assertRaises(TypeError):
                    OV9281(CameraBus(events), **arguments)
        self.assertEqual(events, [])

    def test_invalid_selection_has_no_io(self):
        for camera, address in ((-1, 0x10), (2, 0x10), (0, -1), (0, 0x80),
                                (1.0, 0x10), (True, 0x10), (0, True)):
            with self.subTest(camera=camera, address=address):
                events = []
                with self.assertRaises(ValueError):
                    OV9281(CameraBus(events), camera=camera, address=address,
                           i2c=RegisterSensor(events))
                self.assertEqual(events, [])

    def test_default_i2c_base_tracks_only_selected_camera(self):
        for camera, expected in ((0, 0x30020000), (1, 0x30021000)):
            with self.subTest(camera=camera):
                events = []
                helper = OV9281(CameraBus(events), camera=camera, address=0x10)
                self.assertEqual(helper.i2c.address, expected)
                self.assertEqual(events, [])

    def test_invalid_or_unbounded_i2c_timing_has_no_io(self):
        for option, values in (("half_period", (float("nan"), float("inf"), -1, 1.01)),
                               ("stretch_timeout", (float("nan"), float("inf"), -1, 0, 60.01))):
            for value in values:
                with self.subTest(option=option, value=value):
                    events = []
                    with self.assertRaises(ValueError):
                        OV9281(CameraBus(events), camera=1, address=0x10,
                               i2c=RegisterSensor(events), **{option: value})
                    self.assertEqual(events, [])

    def test_identification_reads_big_endian_chip_id_at_explicit_address(self):
        helper, _, _, events = self.fixture(address=0x60)
        helper.identify()
        self.assertIn(("sensor-read", 0x60, 0x300a, 2, b"\x92\x81"), events)
        self.assertEqual(self.any_writes(events), [])

    def test_invalid_read_bounds_have_no_io(self):
        helper, _, _, events = self.fixture()
        for register, length in ((-1, 1), (0x10000, 1), (0, 0),
                                 (0, 257), (0xffff, 2)):
            with self.subTest(register=register, length=length):
                with self.assertRaises(ValueError):
                    helper.read_register(register, length)
                self.assertEqual(events, [])

    def test_partial_identity_response_aborts_before_sensor_writes(self):
        helper, _, sensor, events = self.fixture()
        with patch.object(sensor, "read_register", return_value=b"\x92"):
            with self.assertRaises(CameraError):
                helper.configure()
        self.assertEqual(self.any_writes(events), [])

    def test_bounded_raw_read_keeps_byte_order_and_does_not_write(self):
        helper, _, sensor, events = self.fixture()
        sensor.values.update({0x4000 + offset: offset for offset in range(256)})
        self.assertEqual(helper.read_register(0x4000, 256), bytes(range(256)))
        self.assertEqual(self.any_writes(events), [])

    def test_power_sequence_reads_back_controls_before_starting_delays(self):
        helper, bus, _, events = self.fixture()
        base = 0x30024100
        bus.values[base + 0x0c] = 0
        helper.power_on()
        writes = [event for event in events if event[0] == "bus-write"]
        self.assertEqual(writes, [("bus-write", base + 0x0c, 4),
                                  ("bus-write", base + 0x0c, 5)])
        for value in (4, 5):
            index = events.index(("bus-write", base + 0x0c, value))
            self.assertEqual(events[index + 1:index + 3],
                             [("bus-read", base + 0x0c, value), ("sleep", 0.001)])
        self.assertLess(events.index(("release",)),
                        events.index(("sensor-read", 0x10, 0x300a, 2, b"\x92\x81")))
        self.assertEqual(self.sensor_writes(events), [])
        for event in events:
            if event[0] == "bus-write":
                self.assertTrue(base <= event[1] < base + 0x100,
                                "power-on may inspect both cameras but only alter the selected one")

    def test_power_refuses_armed_or_nonidle_camera_without_writes(self):
        for run, status in ((1, 2 | 8), (0, 8)):
            with self.subTest(run=run, status=status):
                helper, bus, _, events = self.fixture()
                bus.values[0x30024100] = run
                bus.values[0x30024114] = status
                with self.assertRaises(CameraError):
                    helper.power_on()
                self.assertEqual(self.any_writes(events), [])

    def test_failed_power_control_readback_stops_before_release_or_identity(self):
        helper, bus, _, events = self.fixture()
        bus.corrupt_write = 0x3002410c
        with self.assertRaises(CameraError):
            helper.power_on()
        self.assertEqual(self.any_writes(events), [("bus-write", 0x3002410c, 4)])
        self.assertFalse(any(event[0] in ("release", "sensor-read", "sleep")
                             for event in events))

    def test_wrong_identity_blocks_sensor_mutations(self):
        for method in ("configure",):
            with self.subTest(method=method):
                helper, _, sensor, events = self.fixture()
                sensor.values.update(MODE_FINAL)
                sensor.values[0x300b] = 0x82
                with self.assertRaises(CameraError):
                    getattr(helper, method)()
                self.assertEqual(self.any_writes(events), [])

    def test_configure_refuses_running_sensor_instead_of_implicitly_stopping(self):
        helper, _, sensor, events = self.fixture()
        sensor.values[0x0100] = 1
        with self.assertRaises(CameraError):
            helper.configure()
        self.assertEqual(self.any_writes(events), [])

    def test_configure_refuses_bad_capture_or_power_preconditions(self):
        for register, value in ((0x30024100, 1), (0x30024114, 8),
                                (0x3002410c, 0), (0x3002410c, 4)):
            with self.subTest(register=register, value=value):
                helper, bus, _, events = self.fixture()
                bus.values[register] = value
                with self.assertRaises(CameraError):
                    helper.configure()
                self.assertEqual(self.any_writes(events), [])

    def test_power_and_configure_refuse_unknown_pair_magic_without_sensor_io(self):
        for method in ("power_on", "configure"):
            for offset, value in ((0, 0), (0, 0xffffffff), (0, 0xdeadbeef)):
                with self.subTest(method=method, offset=offset, value=value):
                    helper, bus, _, events = self.fixture()
                    bus.values[PAIR_BASE + offset] = value
                    with self.assertRaises(CameraError):
                        getattr(helper, method)()
                    self.assertTrue(events)
                    self.assertTrue(all(event[0] == "bus-read" for event in events),
                                    "firmware rejection must precede I2C pin activity or sensor power writes")

    def test_sensor_configuration_ignores_reserved_pair_word_on_new_and_old_boards(self):
        for reserved in (0, 1, 2, 0xffffffff):
            with self.subTest(reserved=reserved):
                helper, bus, _, events = self.fixture()
                bus.values[PAIR_BASE + 4] = reserved
                helper.configure()
                self.assertFalse(any(event[:2] == ("bus-read", PAIR_BASE + 4)
                                     for event in events))
                self.assertTrue(self.sensor_writes(events))

    def test_power_and_configure_refuse_every_busy_or_faulted_pair_state_without_sensor_io(self):
        bad_registers = [(PAIR_RUN, 1), (PAIR_STATUS, 0),
                         (PAIR_STATUS, 0xffffffff),
                         (PAIR_STATUS, PAIR_STATUS_DRAINED | PAIR_STATUS_FAULT),
                         (PAIR_FAULT, 1), (PAIR_FAULT, 0xffffffff)]
        bad_registers += [(PAIR_STATUS, PAIR_STATUS_DRAINED | bit)
                          for bit in (1, 2, 4, 8)]
        self.assertEqual(PAIR_STATUS_BUSY, 1 | 2 | 4 | 8)
        for method in ("power_on", "configure"):
            for offset, value in bad_registers:
                with self.subTest(method=method, offset=offset, value=value):
                    helper, bus, _, events = self.fixture()
                    bus.values[PAIR_BASE + offset] = value
                    with self.assertRaises(CameraError):
                        getattr(helper, method)()
                    self.assertTrue(all(event[0] == "bus-read" for event in events))

    def test_power_and_configure_require_both_cameras_disarmed_and_idle(self):
        for selected in (0, 1):
            for busy_camera in (0, 1):
                for offset, value in ((0, 1), (0x14, 8), (0x14, 2 | 1),
                                      (0x14, 2 | 4), (0x14, 0xffffffff)):
                    for method in ("power_on", "configure"):
                        with self.subTest(selected=selected, busy_camera=busy_camera,
                                          offset=offset, method=method):
                            helper, bus, _, events = self.fixture(camera=selected)
                            bus.values[0x30024000 + busy_camera * 0x100 + offset] = value
                            with self.assertRaises(CameraError):
                                getattr(helper, method)()
                            self.assertTrue(all(event[0] == "bus-read" for event in events))

    def test_pair_status_transport_failure_cannot_reach_sensor_io(self):
        for method in ("power_on", "configure"):
            with self.subTest(method=method):
                helper, bus, _, events = self.fixture()
                read = bus.read
                def fail_status(address):
                    if address == PAIR_BASE + PAIR_STATUS:
                        raise OSError("pair status transport unavailable")
                    return read(address)
                with patch.object(bus, "read", side_effect=fail_status):
                    with self.assertRaises(OSError):
                        getattr(helper, method)()
                self.assertTrue(all(event[0] == "bus-read" for event in events))

    def test_safe_drained_pair_can_retain_readonly_diagnostic_status_bits(self):
        helper, bus, sensor, _ = self.fixture()
        # Locked, both-ready, VSYNC level and snapshot-valid do not request capture.
        bus.values[PAIR_BASE + PAIR_STATUS] = PAIR_STATUS_DRAINED | (0xf << 6)
        helper.configure()
        self.assertEqual(sensor.values[0x0100], 0)

    def test_unknown_mode_is_rejected_before_io(self):
        helper, _, _, events = self.fixture()
        with self.assertRaises((ValueError, CameraError)):
            helper.configure(mode="unreviewed-sensor-mode")
        self.assertEqual(events, [])

    def test_invalid_pattern_or_solid_value_is_rejected_before_io(self):
        for method in ("configure", "verify_mode"):
            for options in ({"pattern": "noise"}, {"solid": -1},
                            {"solid": 1024}, {"solid": 1.5}, {"solid": True}):
                with self.subTest(method=method, options=options):
                    helper, _, _, events = self.fixture()
                    with self.assertRaises(ValueError):
                        getattr(helper, method)(**options)
                    self.assertEqual(events, [])

    def test_invalid_frame_rate_preset_is_rejected_before_io(self):
        for fps in (0, 30, 121, True, 60.0):
            for method in ("configure", "verify_mode"):
                with self.subTest(method=method, fps=fps):
                    helper, _, _, events = self.fixture()
                    with self.assertRaises(ValueError):
                        getattr(helper, method)(fps=fps)
                    self.assertEqual(events, [])
            for make_mode in (mode_writes, mode_manifest):
                with self.subTest(function=make_mode.__name__, fps=fps):
                    with self.assertRaises(ValueError):
                        make_mode(fps=fps)

    def test_120fps_changes_only_vertical_total_for_each_pattern(self):
        self.assertEqual(mode_writes(), MODE_WRITES)
        self.assertEqual(mode_writes(fps=60), MODE_WRITES)
        for pattern in ("bars", "off", "solid"):
            with self.subTest(pattern=pattern):
                slower = mode_writes(pattern, 0x155, fps=60)
                faster = mode_writes(pattern, 0x155, fps=120)
                self.assertEqual([reg for reg, _ in faster], [reg for reg, _ in slower])
                differences = [(old, new) for old, new in zip(slower, faster) if old != new]
                self.assertEqual(differences,
                                 [((0x380e, 0x07), (0x380e, 0x03)),
                                  ((0x380f, 0x1e), (0x380f, 0x8e))])
                registers = dict(faster)
                self.assertEqual((registers[0x380e] << 8) | registers[0x380f], 910)
                exposure = ((registers[0x3500] << 16) |
                            (registers[0x3501] << 8) | registers[0x3502]) >> 4
                self.assertEqual(exposure, 400)

    def test_120fps_configuration_and_explicit_verification_check_timing(self):
        helper, _, sensor, events = self.fixture()
        helper.configure(pattern="solid", solid=0x155, fps=120)
        for register, value in ((0x380e, 0x03), (0x380f, 0x8e)):
            write = ("sensor-write", 0x10, register, bytes([value]))
            index = events.index(write)
            self.assertEqual(events[index + 1],
                             ("sensor-read", 0x10, register, 1, bytes([value])))
        self.assertEqual(sensor.values[0x0100], 0)
        events.clear()
        helper.verify_mode(pattern="solid", solid=0x155, fps=120)
        self.assertEqual(self.any_writes(events), [])
        self.assertIn(("sensor-read", 0x10, 0x380e, 1, b"\x03"), events)
        self.assertIn(("sensor-read", 0x10, 0x380f, 1, b"\x8e"), events)

    def test_mode_verification_rejects_a_different_fps_preset(self):
        for configured, requested in ((60, 120), (120, 60)):
            with self.subTest(configured=configured, requested=requested):
                helper, _, sensor, events = self.fixture()
                helper.configure(fps=configured)
                events.clear()
                with self.assertRaises(CameraError):
                    helper.verify_mode(fps=requested)
                self.assertEqual(self.any_writes(events), [])
                self.assertEqual(sensor.values[0x0100], 0)

    def test_explicit_mode_name_cannot_silently_select_different_frame_rate(self):
        mode120 = "raw10-1280x800-25mhz-120fps-bars"
        for name, fps in ((MODE_NAME, 120), (mode120, 60)):
            with self.subTest(name=name, fps=fps):
                helper, _, _, events = self.fixture()
                with self.assertRaises(ValueError):
                    helper.configure(mode=name, fps=fps)
                self.assertEqual(events, [])
        helper, _, sensor, _ = self.fixture()
        report = helper.configure(mode=mode120, fps=120)
        self.assertEqual(report["mode"], mode120)
        self.assertEqual((sensor.values[0x380e] << 8) | sensor.values[0x380f], 910)

    def test_manifest_reports_calculated_rate_and_programmed_vertical_total(self):
        for fps, vts in ((60, 1822), (120, 910)):
            with self.subTest(fps=fps):
                manifest = mode_manifest(fps=fps)
                registers = dict(manifest["writes"])
                self.assertEqual(manifest["fps_preset"], fps)
                self.assertEqual(manifest["vts"], vts)
                self.assertEqual((registers[0x380e] << 8) | registers[0x380f], vts)
                self.assertEqual(manifest["hts"], 728)
                self.assertAlmostEqual(manifest["derived_frame_rate_hz"], 80_000_000 / (728 * vts))
                self.assertNotEqual(manifest["derived_frame_rate_hz"], fps)

    def test_bars_and_off_explicitly_disable_a_previous_solid_pattern(self):
        for pattern, expected_bars in (("bars", 0x80), ("off", 0)):
            with self.subTest(pattern=pattern):
                helper, _, sensor, events = self.fixture()
                helper.configure(pattern="solid", solid=0x2aa)
                self.assertEqual(sensor.values[0x4320], 0x82)
                events.clear()
                helper.configure(pattern=pattern)
                self.assertEqual(sensor.values[0x4320], 0x80)
                self.assertEqual(sensor.values[0x5e00], expected_bars)
                self.assertIn(("sensor-write", 0x10, 0x4320, b"\x80"), events)
                self.assertEqual(sensor.values[0x0100], 0)
                events.clear()
                helper.verify_mode(pattern=pattern)
                self.assertEqual(self.any_writes(events), [])
                self.assertEqual(sensor.values[0x0100], 0)

    def test_solid_programs_and_checks_all_four_ten_bit_constant_pairs(self):
        for value in (0, 0x155, 0x2aa, 1023):
            with self.subTest(value=value):
                helper, _, sensor, events = self.fixture()
                helper.configure(pattern="solid", solid=value)
                self.assertEqual(sensor.values[0x4320], 0x82)
                self.assertEqual(sensor.values[0x5e00], 0)
                for register in (0x4322, 0x4324, 0x4326, 0x4328):
                    self.assertEqual((sensor.values[register] << 8) |
                                     sensor.values[register + 1], value)
                    for offset in (0, 1):
                        payload = bytes([(value >> 8) if offset == 0 else (value & 255)])
                        write = ("sensor-write", 0x10, register + offset, payload)
                        index = events.index(write)
                        self.assertEqual(events[index + 1],
                                         ("sensor-read", 0x10, register + offset, 1, payload))
                self.assertEqual(sensor.values[0x0100], 0)

    def test_mode_verification_rejects_wrong_pattern_or_solid_level(self):
        for configured, requested in (("bars", "off"), ("bars", "solid"),
                                      ("off", "bars"), ("off", "solid"),
                                      ("solid", "bars"), ("solid", "off")):
            with self.subTest(configured=configured, requested=requested):
                helper, _, sensor, events = self.fixture()
                helper.configure(pattern=configured, solid=0x155)
                events.clear()
                with self.assertRaises(CameraError):
                    helper.verify_mode(pattern=requested, solid=0x155)
                self.assertEqual(self.any_writes(events), [])
                self.assertEqual(sensor.values[0x0100], 0)
        helper, _, sensor, events = self.fixture()
        helper.configure(pattern="solid", solid=0x155)
        events.clear()
        with self.assertRaises(CameraError):
            helper.verify_mode(pattern="solid", solid=0x255)
        self.assertEqual(self.any_writes(events), [])
        self.assertEqual(sensor.values[0x0100], 0)

    def test_solid_verification_checks_each_of_the_four_pixel_constants(self):
        for register in range(0x4322, 0x432a):
            with self.subTest(register=register):
                helper, _, sensor, events = self.fixture()
                sensor.values.update(dict(board_mode_writes(1, "solid", 0x155)))
                sensor.values[register] ^= 1
                with self.assertRaises(CameraError):
                    helper.verify_mode(pattern="solid", solid=0x155)
                self.assertEqual(self.any_writes(events), [])

    def test_configuration_checks_every_write_then_entire_final_state(self):
        helper, _, sensor, events = self.fixture()
        helper.configure()
        writes = board_mode_writes(1)
        expected = [("sensor-write", 0x10, register, bytes([value]))
                    for register, value in writes]
        self.assertEqual(self.sensor_writes(events), expected)
        for index, event in enumerate(events):
            if event[0] == "sensor-write":
                self.assertEqual(events[index + 1],
                                 ("sensor-read", 0x10, event[2], 1, event[3]))
        last_write = max(index for index, event in enumerate(events)
                         if event[0] == "sensor-write")
        final_reads = {event[2] for event in events[last_write + 2:]
                       if event[0] == "sensor-read"}
        self.assertTrue(set(dict(writes)) <= final_reads)
        self.assertEqual(sensor.values[0x0100], 0)
        self.assertIn(("sleep", 0.066), events[last_write:])
        self.assertFalse(any(event[0] == "bus-write" for event in events))

    def test_board_mode_sets_both_sensors_as_slaves_after_all_base_timing_writes(self):
        defaults = ((0x0106, 0), (0x3027, 0), (0x3666, 0), (0x4f00, 0),
                    (0x4311, 4), (0x4312, 0), (0x4313, 0),
                    (0x4314, 0), (0x4315, 1), (0x4316, 0))
        for camera in (0, 1):
            for fps in (60, 120):
                with self.subTest(camera=camera, fps=fps):
                    base = mode_writes("solid", 0x155, fps=fps)
                    board = board_mode_writes(camera, "solid", 0x155, fps=fps)
                    self.assertEqual(board[0], (0x3823, 0),
                                     "disable previous slave synchronization before changing timing")
                    self.assertEqual(board[1:len(base) + 1], base)
                    self.assertCountEqual(board[len(base) + 1:-2], defaults)
                    self.assertEqual(board[-2:], ((0x3006, 0x00), (0x3823, 0x30)))
                    self.assertFalse(any(register == 0x0100 and value for register, value in board))
                    self.assertFalse(any(register == 0x3006 and value & 2 for register, value in board),
                                     "neither sensor may drive FSIN against the FPGA sync generator")
                    registers = dict(board)
                    self.assertEqual(registers[0x4f00], 0, "continuous sync requires normal sensor timing")
                    self.assertEqual(registers[0x3030] & 4, 0,
                                     "continuous sync must not enable FSIN-wakeup snapshot mode")

    def test_board_modes_require_an_explicit_valid_camera(self):
        for camera in (-1, 2, True, 1.0):
            for make_mode in (board_mode_writes, board_mode_manifest):
                with self.subTest(camera=camera, function=make_mode.__name__):
                    with self.assertRaises(ValueError):
                        make_mode(camera)

    def test_configure_verifies_role_then_settles_before_complete_final_readback(self):
        for camera in (0, 1):
            with self.subTest(camera=camera):
                helper, _, sensor, events = self.fixture(camera=camera)
                result = helper.configure()
                self.assertEqual(result["role"], "slave")
                writes = self.sensor_writes(events)
                self.assertEqual(writes[-2:], [("sensor-write", 0x10, 0x3006, b"\x00"),
                                              ("sensor-write", 0x10, 0x3823, b"\x30")])
                for write in writes[-2:]:
                    index = max(index for index, event in enumerate(events) if event == write)
                    self.assertEqual(events[index + 1],
                                     ("sensor-read", 0x10, write[2], 1, write[3]))
                settling = events.index(("sleep", 0.066))
                final_write = max(index for index, event in enumerate(events)
                                  if event[0] == "sensor-write")
                self.assertGreater(settling, final_write + 1)
                final_reads = {event[2] for event in events[settling + 1:]
                               if event[0] == "sensor-read"}
                self.assertTrue(set(dict(board_mode_writes(camera))) <= final_reads)
                self.assertFalse(self.any_writes(events[settling + 1:]))
                self.assertEqual(sensor.values[0x0100], 0)

    def test_role_readback_failure_aborts_before_settling_or_further_writes(self):
        for camera in (0, 1):
            helper, _, sensor, events = self.fixture(camera=camera)
            sensor.corrupt_write = 0x3823
            sensor.corrupt_write_occurrence = 2
            with self.subTest(camera=camera):
                with self.assertRaises(CameraError):
                    helper.configure()
                self.assertEqual(self.sensor_writes(events)[-1][2], 0x3823)
                self.assertNotIn(("sleep", 0.066), events)
                self.assertEqual(sensor.values[0x0100], 0)

    def test_verification_rejects_sensor_master_or_disabled_sync_without_mutation(self):
        for camera in (0, 1):
            for register, value in ((0x3006, 0x02), (0x3823, 0x00)):
                with self.subTest(camera=camera, register=register):
                    helper, _, sensor, events = self.fixture(camera=camera)
                    sensor.values.update(dict(board_mode_writes(camera)))
                    sensor.values[register] = value
                    with self.assertRaises(CameraError):
                        helper.verify_mode()
                    self.assertEqual(self.any_writes(events), [])

    def test_board_manifest_documents_fpga_master_and_required_gateware(self):
        for camera in (0, 1):
            with self.subTest(camera=camera):
                manifest = board_mode_manifest(camera, "off", fps=120)
                self.assertEqual(manifest["camera"], camera)
                self.assertEqual(manifest["writes"],
                                 [list(pair) for pair in board_mode_writes(camera, "off", fps=120)])
                sync = manifest["synchronization"]
                self.assertEqual(sync["role"], "slave")
                self.assertEqual(sync["master"], "fpga")
                self.assertEqual(sync["slave_cameras"], [0, 1])
                self.assertEqual(sync["fpga_fsin_direction"], "output")
                self.assertEqual(sync["sensor_fsin_direction"], "input")
                self.assertFalse(sync["sensor_vsync_output_enabled"])
                self.assertEqual(sync["required_pair_control"],
                                 {"base": PAIR_BASE, "magic": PAIR_MAGIC})
                self.assertEqual(sync["settle_after_writes_seconds"], 0.066)
                self.assertIn("synchronization_table", manifest["provenance"])
                self.assertFalse(manifest["stream_on_included"])
                for fps, period in ((60, 1_990_716), (120, 994_812)):
                    timing = board_mode_manifest(camera, "off", fps=fps)["synchronization"]
                    self.assertEqual(timing["generator_reference_clock_hz"], 120_000_000)
                    self.assertEqual(timing["generator_period_cycles"], period)
                    self.assertEqual(timing["generator_pulse_width_cycles"], 1536)
                    self.assertAlmostEqual(timing["derived_trigger_rate_hz"], 120_000_000 / period)
                    self.assertAlmostEqual(timing["generator_pulse_width_cycles"] /
                                           timing["generator_reference_clock_hz"], 12.8e-6)

    def test_nack_or_readback_mismatch_aborts_at_first_failed_write(self):
        failing_register = 0x3013  # Early Linux common register, before PLL overrides.
        writes = board_mode_writes(1)
        failure_index = next(index for index, pair in enumerate(writes)
                             if pair[0] == failing_register)
        for fault, expected_error in (("nack_write", I2CNack),
                                      ("corrupt_write", CameraError)):
            with self.subTest(fault=fault):
                helper, _, sensor, events = self.fixture()
                setattr(sensor, fault, failing_register)
                with self.assertRaises(expected_error):
                    helper.configure()
                self.assertEqual([event[2] for event in self.sensor_writes(events)],
                                 [register for register, _ in writes[:failure_index + 1]])
                self.assertEqual(sensor.values[0x0100], 0)

    def test_final_verification_catches_later_register_corruption(self):
        helper, _, sensor, events = self.fixture()
        writes = board_mode_writes(1)
        corrupted = False
        def corrupt_after_last_write(register, length):
            nonlocal corrupted
            if not corrupted and register == writes[-1][0] and len(self.sensor_writes(events)) == len(writes):
                sensor.values[0x0302] ^= 1
                corrupted = True
        sensor.read_hook = corrupt_after_last_write
        with self.assertRaises(CameraError):
            helper.configure()
        self.assertEqual(sensor.values[0x0100], 0)
        self.assertEqual(len(self.sensor_writes(events)), len(writes))

    def test_mode_verification_rejects_a_corrupt_pll_setting(self):
        helper, _, sensor, events = self.fixture()
        sensor.values.update(dict(board_mode_writes(1)))
        sensor.values[0x0302] ^= 1
        with self.assertRaises(CameraError):
            helper.verify_mode()
        self.assertEqual(self.any_writes(events), [])

    def test_individual_start_and_stop_refuse_before_any_io(self):
        for camera in (0, 1):
            for method in ("start", "stop"):
                for streaming in (0, 1):
                    with self.subTest(camera=camera, method=method, streaming=streaming):
                        helper, _, sensor, events = self.fixture(camera=camera)
                        sensor.values.update(dict(board_mode_writes(camera)))
                        sensor.values[0x0100] = streaming
                        with self.assertRaises(CameraError):
                            getattr(helper, method)()
                        self.assertEqual(events, [])

    def test_status_is_read_only_even_while_streaming(self):
        helper, _, sensor, events = self.fixture()
        sensor.values.update(MODE_FINAL)
        sensor.values[0x0100] = 1
        report = helper.status()
        self.assertIsInstance(report, dict)
        self.assertTrue(any(event[0] == "bus-read" for event in events))
        self.assertTrue(any(event[0] == "sensor-read" for event in events))
        self.assertEqual(self.any_writes(events), [])
        self.assertNotIn(("release",), events)

    def test_mode_registers_independently_describe_expected_geometry_and_clocks(self):
        self.assertEqual(MODE_NAME, "raw10-1280x800-25mhz-60fps-bars")
        self.assertFalse(any(register == 0x0103 for register, _ in MODE_WRITES),
                         "reset would invalidate ordered readback and power sequencing")
        self.assertFalse(any(register in (0x0101, 0x1000) for register, _ in MODE_WRITES),
                         "reserved or undocumented driver writes are not OV9281 mode configuration")
        self.assertFalse(any(register == 0x0100 and value != 0
                             for register, value in MODE_WRITES))
        self.assertEqual((MODE_FINAL[0x3808] << 8) | MODE_FINAL[0x3809], 1280)
        self.assertEqual((MODE_FINAL[0x380a] << 8) | MODE_FINAL[0x380b], 800)
        hts = (MODE_FINAL[0x380c] << 8) | MODE_FINAL[0x380d]
        vts = (MODE_FINAL[0x380e] << 8) | MODE_FINAL[0x380f]
        self.assertEqual((hts, vts), (728, 1822))
        expected = {0x030a: 0, 0x0300: 1, 0x0301: 0, 0x0302: 48,
                    0x0303: 0, 0x0304: 3, 0x0305: 2, 0x0306: 1,
                    0x0314: 0, 0x030b: 3, 0x030c: 0, 0x030d: 64,
                    0x030f: 3, 0x030e: 2, 0x0312: 7, 0x0313: 1,
                    0x3039: 0x32, 0x303a: 0, 0x3662: 5,
                    0x4800: 0, 0x4802: 0, 0x5e00: 0x80}
        self.assertEqual({register: MODE_FINAL[register] for register in expected}, expected)
        pll1_vco = 25_000_000 / 1.5 * MODE_FINAL[0x0302]
        pll2_vco = 25_000_000 / 2.5 * MODE_FINAL[0x030d]
        self.assertEqual(pll1_vco, 800_000_000)
        self.assertEqual(pll2_vco, 640_000_000)
        self.assertEqual(pll1_vco / 8, 100_000_000)
        self.assertAlmostEqual((pll2_vco / 4 / 2) / (hts * vts), 60.312903342541105)


class OV9281CLITests(unittest.TestCase):
    def test_cli_forwards_explicit_120fps_to_configure(self):
        for operation in ("configure",):
            with self.subTest(operation=operation):
                with patch("tools.lutcam.ov9281.create_transport"), \
                        patch("tools.lutcam.ov9281.OV9281") as helper:
                    method = getattr(helper.return_value, operation)
                    method.return_value = {}
                    with redirect_stdout(io.StringIO()):
                        result = main(["--camera", "1", "--address", "0x10",
                                       operation, "--pattern", "off", "--fps", "120"])
                    self.assertEqual(result, 0)
                    method.assert_called_once()
                    self.assertEqual(method.call_args.kwargs["fps"], 120)
                    self.assertEqual(method.call_args.kwargs["pattern"], "off")

    def test_cli_individual_start_and_stop_refuse_before_transport(self):
        for operation in ("start", "stop"):
            with self.subTest(operation=operation):
                output = io.StringIO()
                with patch("tools.lutcam.ov9281.create_transport") as connect:
                    with redirect_stdout(output):
                        result = main(["--camera", "1", "--address", "0x10", operation])
                    connect.assert_not_called()
                self.assertNotEqual(result, 0)
                self.assertFalse(json.loads(output.getvalue())["ok"])

    def test_cli_rejects_unknown_fps_before_opening_connection(self):
        for operation in ("configure", "start"):
            for fps in ("0", "30", "121", "true", "60.0"):
                with self.subTest(operation=operation, fps=fps):
                    with patch("tools.lutcam.ov9281.create_transport") as connect:
                        with redirect_stderr(io.StringIO()):
                            with self.assertRaises(SystemExit):
                                main(["--camera", "1", "--address", "0x10",
                                      operation, "--fps", fps])
                        connect.assert_not_called()

    def test_cli_export_120fps_includes_correct_timing_without_connection(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "120fps.json"
            with patch("tools.lutcam.ov9281.create_transport") as connect:
                with redirect_stdout(io.StringIO()):
                    result = main(["--camera", "1", "--address", "0x10",
                                   "export-mode", str(destination), "--fps", "120"])
                connect.assert_not_called()
            self.assertEqual(result, 0)
            manifest = json.loads(destination.read_text())
            self.assertEqual(manifest["fps_preset"], 120)
            self.assertEqual(manifest["vts"], 910)
            self.assertAlmostEqual(manifest["derived_frame_rate_hz"], 80_000_000 / (728 * 910))
            registers = dict(manifest["writes"])
            self.assertEqual((registers[0x380e] << 8) | registers[0x380f], 910)

    def test_cli_rejects_mode_and_fps_mismatch_before_connection(self):
        output = io.StringIO()
        with patch("tools.lutcam.ov9281.create_transport") as connect:
            with redirect_stdout(output):
                result = main(["--camera", "1", "--address", "0x10", "configure",
                               "--fps", "120", "--mode", MODE_NAME])
            connect.assert_not_called()
        self.assertNotEqual(result, 0)
        self.assertFalse(json.loads(output.getvalue())["ok"])

    def test_cli_requires_camera_and_unshifted_address_before_opening_connection(self):
        for arguments in (("status",), ("--camera", "1", "status"),
                          ("--address", "0x10", "status"),
                          ("--camera", "2", "--address", "0x10", "status")):
            with self.subTest(arguments=arguments):
                with patch("tools.lutcam.ov9281.create_transport") as connect:
                    with redirect_stderr(io.StringIO()):
                        with self.assertRaises(SystemExit):
                            main(list(arguments))
                    connect.assert_not_called()

    def test_cli_rejects_shifted_address_before_opening_connection(self):
        output = io.StringIO()
        with patch("tools.lutcam.ov9281.create_transport") as connect:
            with redirect_stdout(output):
                result = main(["--camera", "1", "--address", "0xc0", "status"])
            connect.assert_not_called()
        self.assertNotEqual(result, 0)
        self.assertFalse(json.loads(output.getvalue())["ok"])

    def test_cli_validates_read_bounds_before_opening_connection(self):
        for register, length in (("-1", "1"), ("0x10000", "1"), ("0", "0"),
                                 ("0", "257"), ("0xffff", "2")):
            with self.subTest(register=register, length=length):
                output = io.StringIO()
                with patch("tools.lutcam.ov9281.create_transport") as connect:
                    with redirect_stdout(output):
                        result = main(["--camera", "1", "--address", "0x10",
                                       "read", register, "--length", length])
                    connect.assert_not_called()
                self.assertNotEqual(result, 0)
                self.assertFalse(json.loads(output.getvalue())["ok"])

    def test_cli_rejects_solid_out_of_range_before_opening_connection(self):
        for operation in ("configure", "start"):
            with self.subTest(operation=operation):
                output = io.StringIO()
                with patch("tools.lutcam.ov9281.create_transport") as connect:
                    with redirect_stdout(output):
                        result = main(["--camera", "1", "--address", "0x10",
                                       operation, "--pattern", "solid", "--solid", "0x400"])
                    connect.assert_not_called()
                self.assertNotEqual(result, 0)
                self.assertFalse(json.loads(output.getvalue())["ok"])

    def test_mode_export_never_opens_transport(self):
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "sensor-mode.json"
            with patch("tools.lutcam.ov9281.create_transport") as connect:
                with redirect_stdout(output):
                    result = main(["--camera", "1", "--address", "0x10", "export-mode", str(destination)])
                connect.assert_not_called()
            self.assertEqual(result, 0)
            manifest = json.loads(destination.read_text())
        self.assertEqual(manifest["name"], MODE_NAME)
        self.assertEqual(manifest["input_clock_hz"], 25_000_000)
        self.assertEqual(manifest["writes"], [list(pair) for pair in board_mode_writes(1)])
        # The manifest is independently serializable for a reviewable offline plan.
        self.assertEqual(json.loads(json.dumps(mode_manifest()))["name"], MODE_NAME)

    def test_export_records_selected_pattern_and_exact_solid_constants(self):
        with tempfile.TemporaryDirectory() as directory:
            for pattern in ("off", "solid"):
                with self.subTest(pattern=pattern):
                    output = io.StringIO()
                    destination = Path(directory) / f"{pattern}.json"
                    with patch("tools.lutcam.ov9281.create_transport") as connect:
                        with redirect_stdout(output):
                            result = main(["--camera", "1", "--address", "0x10",
                                           "export-mode", str(destination),
                                           "--pattern", pattern, "--solid", "0x155"])
                        connect.assert_not_called()
                    self.assertEqual(result, 0)
                    manifest = json.loads(destination.read_text())
                    self.assertEqual(manifest["pattern"], pattern)
                    registers = dict(manifest["writes"])
                    self.assertEqual(registers[0x5e00], 0)
                    self.assertEqual(registers[0x4320], 0x82 if pattern == "solid" else 0x80)
                    if pattern == "solid":
                        self.assertEqual(manifest["solid_pixel"], 0x155)
                        for register in (0x4322, 0x4324, 0x4326, 0x4328):
                            self.assertEqual((registers[register] << 8) | registers[register + 1], 0x155)


if __name__ == "__main__":
    unittest.main()
