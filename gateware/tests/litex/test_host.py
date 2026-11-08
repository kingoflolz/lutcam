"""Host control/protocol checks; these tests never open a hardware connection."""

from collections import deque
import io
import json
import random
import struct
import tempfile
from pathlib import Path
import unittest
from contextlib import redirect_stdout, redirect_stderr
from unittest.mock import Mock, patch

from tools.lutcam.control import (
    BitBangI2C, I2CError, I2CNack, I2CTimeout, main as control_main,
)
from tools.lutcam import ov9281
from tools.lutcam.flash import EtherboneUDP, FLASH_BASE, FLASH_MAGIC
from tools.lutcam.receive import (
    CHILD, FRAGMENT_SIZE, IMAGE, IMAGE_SIZE, POSE,
    Packet, PacketError, SequenceTracker, StreamReceiver,
    main as receive_main, parse_datagram,
)


def datagram(payload, *, camera=0, stream=IMAGE, frame=7, sequence=0,
             offset=0, flags=0, version=0):
    return struct.pack(
        ">8I", 0x4C555443, (version << 24) | (camera << 16) | (stream << 8) | 32,
        frame, sequence, offset, len(payload), flags, 0,
    ) + payload


def image_packets(data, *, camera=0, frame=7, sequence=0, fragment_size=FRAGMENT_SIZE,
                  version=0):
    return [
        datagram(data[offset:offset + fragment_size], camera=camera, frame=frame,
                 sequence=(sequence + index) & 0xffffffff, offset=offset,
                 flags=0,
                 version=version)
        for index, offset in enumerate(range(0, len(data), fragment_size))
    ]


def child_batch(groups):
    return b"".join(struct.pack(">9Q", *group) for group in groups)


class PinBus:
    def __init__(self, samples=()):
        self.samples = deque(samples)
        self.control = 0
        self.writes = []
        self.reads = []

    def write(self, address, value):
        self.control = value
        self.writes.append((address, value))

    def read(self, address):
        self.reads.append(address)
        if address % 8 == 0:
            return self.control
        return self.samples.popleft() if self.samples else (~self.control & 3)


class MixedPinBus(PinBus):
    """A checked-frame bridge requiring one ordered record per pin change."""

    def __init__(self, samples=()):
        super().__init__(samples)
        self.combined = []

    def supports_ordered_write_read(self):
        return True

    def write_read(self, write_address, value, read_address):
        self.combined.append((write_address, value, read_address))
        self.control = value
        return value

    def write(self, address, value):
        raise AssertionError("pin change escaped the mixed record")

    def read(self, address):
        if address % 8 == 0:
            raise AssertionError("immediate separate control read would be dropped")
        return super().read(address)


class FaultMixedPinBus(MixedPinBus):
    """Drop selected requests/replies while recording actual pin transitions."""

    def __init__(self, faults=None, samples=()):
        super().__init__(samples)
        self.faults = faults or {}
        self.transitions = []
        self.timeouts = []

    def write_read(self, write_address, value, read_address):
        fault = self.faults.get(len(self.combined))
        self.combined.append((write_address, value, read_address))
        if isinstance(fault, BaseException):
            raise fault
        if fault != "request":
            if self.control != value:
                self.transitions.append((self.control, value))
            self.control = value
        if fault in ("request", "reply"):
            timeout = TimeoutError(f"lost {fault}")
            self.timeouts.append(timeout)
            raise timeout
        return value

    def read(self, address):
        return PinBus.read(self, address)


class OrderedEtherboneCapabilityTests(unittest.TestCase):
    def test_ordered_record_capability_is_cached(self):
        bus = EtherboneUDP()
        bus.read = Mock(side_effect=[[FLASH_MAGIC, 0], [256, 128, 0x100]])
        self.assertTrue(bus.supports_ordered_write_read())
        self.assertTrue(bus.supports_ordered_write_read())
        self.assertEqual(bus.read.call_count, 2)

    def test_reopened_socket_requires_fresh_firmware_capability(self):
        bus = EtherboneUDP()
        old_socket, new_socket = Mock(), Mock()
        bus.socket = old_socket
        bus.read = Mock(side_effect=[[FLASH_MAGIC, 0], [256, 128, 0x100], [0xffffffff, 0]])
        self.assertTrue(bus.supports_ordered_write_read())
        bus.close()
        old_socket.close.assert_called_once_with()
        with patch("tools.lutcam.flash.socket.socket", return_value=new_socket):
            bus.open()
        self.assertFalse(bus.supports_ordered_write_read())
        self.assertEqual(bus.read.call_count, 3)
        bus.close()

    def test_probe_failure_does_not_cache_or_enable_capability(self):
        bus = EtherboneUDP()
        bus.read = Mock(side_effect=[TimeoutError("lost discovery reply"), [0xffffffff, 0]])
        with self.assertRaises(TimeoutError):
            bus.supports_ordered_write_read()
        self.assertFalse(bus.supports_ordered_write_read())
        self.assertEqual(bus.read.call_count, 2)


class TransactionI2C(BitBangI2C):
    """Record software transaction composition above the tested pin layer."""

    def __init__(self, data=()):
        super().__init__(PinBus(), 0x1000)
        self.events = []
        self.data = iter(data)
        self.fail_byte = None

    def start(self, *, repeated=False):
        self.events.append(("start", repeated))

    def stop(self):
        self.events.append(("stop",))

    def write_byte(self, value):
        self.events.append(("write", value))
        if value == self.fail_byte:
            raise I2CNack("injected NACK")

    def read_byte(self, *, ack):
        self.events.append(("read", ack))
        return next(self.data)


class I2CTests(unittest.TestCase):
    def setUp(self):
        self.sleep = patch("tools.lutcam.control.time.sleep")
        self.sleep.start()
        self.addCleanup(self.sleep.stop)

    def test_construction_has_no_hardware_side_effect(self):
        bus = PinBus()
        BitBangI2C(bus, 0x1000)
        self.assertEqual(bus.writes, [])
        self.assertEqual(bus.reads, [])

    def test_mixed_records_preserve_pin_sequence_and_clock_stretch_reads(self):
        bus = MixedPinBus([0, 0, 1])
        i2c = BitBangI2C(bus, 0x1000)
        i2c.write_bit(0)
        self.assertEqual(bus.combined, [(0x1000, value, 0x1000) for value in (3, 2, 3)])
        self.assertEqual(bus.reads, [0x1004] * 3)

    def test_mixed_readback_is_checked_before_the_settling_delay(self):
        bus = MixedPinBus()
        bus.write_read = Mock(return_value=3)
        i2c = BitBangI2C(bus, 0x1000)
        with patch.object(i2c, "_delay") as delay:
            with self.assertRaisesRegex(I2CError, "readback mismatch"):
                i2c.release()
        bus.write_read.assert_called_once_with(0x1000, 0, 0x1000)
        delay.assert_not_called()

    def test_lost_mixed_reply_is_confirmed_without_another_write(self):
        bus = FaultMixedPinBus({0: "reply"})
        i2c = BitBangI2C(bus, 0x1000)
        i2c._lines(scl_low=True, sda_low=True)
        self.assertEqual(bus.combined, [(0x1000, 3, 0x1000)])
        self.assertEqual(bus.reads, [0x1000])
        self.assertEqual(bus.transitions, [(0, 3)])

    def test_dropped_mixed_request_retries_only_the_same_level(self):
        bus = FaultMixedPinBus({0: "request", 1: "request"})
        i2c = BitBangI2C(bus, 0x1000)
        i2c._lines(scl_low=True, sda_low=False)
        self.assertEqual(bus.combined, [(0x1000, 1, 0x1000)] * 3)
        self.assertEqual(bus.reads, [0x1000] * 2)
        self.assertEqual(bus.transitions, [(0, 1)])

    def test_lost_noop_request_needs_no_replay(self):
        bus = FaultMixedPinBus({0: "request"})
        BitBangI2C(bus, 0x1000).release()
        self.assertEqual(len(bus.combined), 1)
        self.assertEqual(bus.reads, [0x1000])
        self.assertEqual(bus.transitions, [])

    def test_pin_retry_preserves_full_byte_edges_and_clock_stretch(self):
        # Two low SCL samples require stretching; the last sample is slave ACK.
        samples = [0, 0] + [1] * 10
        baseline = FaultMixedPinBus(samples=samples)
        lossy = FaultMixedPinBus({0: "request", 4: "reply"}, samples=samples)
        for bus in (baseline, lossy):
            BitBangI2C(bus, 0x1000).write_byte(0xa6)
        self.assertEqual(lossy.transitions, baseline.transitions)
        self.assertEqual(len(lossy.combined), len(baseline.combined) + 1)
        self.assertEqual(lossy.reads.count(0x1004), baseline.reads.count(0x1004))
        self.assertFalse(lossy.samples)

    def test_pin_retry_exhaustion_preserves_first_timeout_without_next_edge(self):
        bus = FaultMixedPinBus({index: "request" for index in range(3)})
        i2c = BitBangI2C(bus, 0x1000)
        with patch.object(i2c, "_delay") as delay:
            with self.assertRaises(TimeoutError) as raised:
                i2c.write_bit(0)
        self.assertIs(raised.exception, bus.timeouts[0])
        self.assertIsInstance(raised.exception.__cause__, I2CError)
        self.assertEqual(bus.combined, [(0x1000, 3, 0x1000)] * 3)
        self.assertEqual(bus.reads, [0x1000] * 3)
        self.assertEqual(bus.transitions, [])
        delay.assert_not_called()

    def test_failed_timeout_readback_aborts_without_retry(self):
        for error in (TimeoutError("read timeout"), RuntimeError("read failed"),
                      KeyboardInterrupt("interrupt")):
            with self.subTest(error=error):
                bus = FaultMixedPinBus({0: "request"})
                bus.read = Mock(side_effect=error)
                i2c = BitBangI2C(bus, 0x1000)
                with self.assertRaises(type(error) if isinstance(error, KeyboardInterrupt)
                                       else TimeoutError) as raised:
                    i2c._lines(scl_low=True, sda_low=True)
                expected = error if isinstance(error, KeyboardInterrupt) else bus.timeouts[0]
                self.assertIs(raised.exception, expected)
                if expected is not error:
                    self.assertIs(raised.exception.__cause__, error)
                self.assertEqual(len(bus.combined), 1)
                bus.read.assert_called_once_with(0x1000)
                self.assertEqual(bus.transitions, [])

    def test_error_sentinel_cannot_confirm_both_pins_low(self):
        for lost_reply in (False, True):
            with self.subTest(lost_reply=lost_reply):
                bus = FaultMixedPinBus({0: "reply"} if lost_reply else {})
                if lost_reply:
                    bus.read = Mock(return_value=0xffffffff)
                else:
                    bus.write_read = Mock(return_value=0xffffffff)
                i2c = BitBangI2C(bus, 0x1000)
                with patch.object(i2c, "_delay") as delay:
                    with self.assertRaises(TimeoutError if lost_reply else I2CError):
                        i2c._lines(scl_low=True, sda_low=True)
                delay.assert_not_called()
                if lost_reply:
                    self.assertEqual(len(bus.combined), 1)
                else:
                    bus.write_read.assert_called_once_with(0x1000, 3, 0x1000)

    def test_non_timeout_mixed_error_does_not_verify_or_retry(self):
        for error in (RuntimeError("bus error"), KeyboardInterrupt("interrupt")):
            with self.subTest(error=error):
                bus = FaultMixedPinBus({0: error})
                i2c = BitBangI2C(bus, 0x1000)
                with self.assertRaises(type(error)) as raised:
                    i2c.release()
                self.assertIs(raised.exception, error)
                self.assertEqual(len(bus.combined), 1)
                self.assertEqual(bus.reads, [])

    def test_mixed_readback_mismatch_does_not_retry(self):
        bus = FaultMixedPinBus()
        bus.write_read = Mock(return_value=1)
        with self.assertRaises(I2CError):
            i2c = BitBangI2C(bus, 0x1000)
            i2c.release()
        bus.write_read.assert_called_once_with(0x1000, 0, 0x1000)
        self.assertEqual(bus.reads, [])

    def test_mixed_method_alone_does_not_enable_unverified_firmware(self):
        for advertised in (None, False):
            with self.subTest(advertised=advertised):
                bus = PinBus()
                bus.write_read = Mock(side_effect=AssertionError("unverified mixed record"))
                if advertised is not None:
                    bus.supports_ordered_write_read = Mock(return_value=advertised)
                BitBangI2C(bus, 0x1000).release()
                self.assertEqual(bus.writes, [(0x1000, 0)])
                self.assertEqual(bus.reads, [0x1000])
                bus.write_read.assert_not_called()

    def test_byte_bit_order_ack_and_open_drain(self):
        value = 0xa6
        bits = [(value >> shift) & 1 for shift in range(7, -1, -1)]
        bus = PinBus([1 | (bit << 1) for bit in bits] + [1, 1])
        i2c = BitBangI2C(bus, 0x1000)
        i2c.write_byte(value)
        expected = []
        for bit in bits:
            expected.extend([1 | ((1 - bit) << 1), (1 - bit) << 1,
                             1 | ((1 - bit) << 1)])
        expected.extend([1, 0, 1])  # Ninth clock: master releases SDA for ACK.
        self.assertEqual(bus.writes, [(0x1000, value) for value in expected])
        self.assertEqual(bus.reads.count(0x1004), 10)
        self.assertEqual(bus.reads.count(0x1000), len(expected))

    def test_byte_nack(self):
        i2c = BitBangI2C(PinBus(), 0)
        with self.assertRaises(I2CNack):
            i2c.write_byte(0xff)

    def test_read_byte_and_final_nack(self):
        value = 0x93
        samples = []
        for shift in range(7, -1, -1):
            samples.extend([1 | (((value >> shift) & 1) << 1)] * 2)
        bus = PinBus(samples)
        i2c = BitBangI2C(bus, 0)
        self.assertEqual(i2c.read_byte(ack=False), value)
        self.assertEqual([value for _, value in bus.writes[-3:]], [1, 0, 1])

    def test_clock_stretch_samples_actual_scl(self):
        bus = PinBus([0, 0, 1])
        i2c = BitBangI2C(bus, 0)
        i2c.write_bit(0)
        self.assertEqual(bus.reads.count(4), 3)

    def test_clock_stretch_timeout(self):
        bus = PinBus([0, 0, 0])
        i2c = BitBangI2C(bus, 0, stretch_timeout=0.1)
        with patch("tools.lutcam.control.time.monotonic", side_effect=[0, 0.05, 0.11]):
            with self.assertRaises(I2CTimeout):
                i2c.write_bit(1)

    def test_start_and_stop_pin_sequence(self):
        bus = PinBus()
        i2c = BitBangI2C(bus, 0)
        i2c.start()
        self.assertEqual([value for _, value in bus.writes], [0, 0, 2, 3])
        bus.writes.clear()
        i2c.start(repeated=True)
        self.assertEqual([value for _, value in bus.writes], [1, 0, 2, 3])
        bus.writes.clear()
        i2c.stop()
        self.assertEqual([value for _, value in bus.writes], [3, 2, 0])

    def test_register_write_is_explicit_16_bit_big_endian(self):
        i2c = TransactionI2C()
        i2c.write_register(0x60, 0x3501, b"\x12\x34")
        self.assertEqual(i2c.events, [
            ("start", False), ("write", 0xc0), ("write", 0x35),
            ("write", 0x01), ("write", 0x12), ("write", 0x34), ("stop",),
        ])

    def test_register_read_repeated_start_and_last_byte_nack(self):
        i2c = TransactionI2C([0x92, 0x81])
        self.assertEqual(i2c.read_register(0x60, 0x300a, 2), b"\x92\x81")
        self.assertEqual(i2c.events, [
            ("start", False), ("write", 0xc0), ("write", 0x30),
            ("write", 0x0a), ("start", True), ("write", 0xc1),
            ("read", True), ("read", False), ("stop",),
        ])

    def test_nack_aborts_with_stop_without_retry(self):
        i2c = TransactionI2C()
        i2c.fail_byte = 0x35
        with self.assertRaises(I2CNack):
            i2c.write_register(0x60, 0x3500, 1)
        self.assertEqual(i2c.events, [
            ("start", False), ("write", 0xc0), ("write", 0x35), ("stop",),
        ])

    def test_recovery_nine_clocks_and_stop(self):
        # SDA remains stuck for all nine pulses, then releases at STOP.
        bus = PinBus([1] * 9 + [1, 3])
        i2c = BitBangI2C(bus, 0)
        i2c.recover()
        self.assertEqual([value for _, value in bus.writes],
                         [0] + [1, 0] * 9 + [3, 2, 0, 0])
        self.assertFalse(bus.samples)

    def test_recovery_does_not_mistake_sensor_data_one_for_idle(self):
        class InterruptedReadBus(PinBus):
            def __init__(self):
                super().__init__()
                self.rises = 0
                self.active = True
                self.slave_low = False

            def write(self, address, value):
                old_scl, new_scl = not (self.control & 1), not (value & 1)
                super().write(address, value)
                if old_scl and not new_scl and self.active:
                    # Sensor sends 0x80: the first high SDA is data, not idle.
                    self.slave_low = 1 <= self.rises < 8
                if not old_scl and new_scl and self.active:
                    self.rises += 1
                    if self.rises == 9 and not (value & 2):
                        self.active = False  # Released SDA is the final NACK.
                        self.slave_low = False

            def read(self, address):
                value = super().read(address)
                return value & ~2 if address % 8 == 4 and self.slave_low else value

        bus = InterruptedReadBus()
        BitBangI2C(bus, 0x1000).recover()
        self.assertEqual(bus.rises, 9)
        self.assertFalse(bus.active)
        self.assertEqual(bus.read(0x1004), 3)
        self.assertEqual([value for _, value in bus.writes],
                         [0] + [1, 0] * 9 + [3, 2, 0, 0])

    def test_invalid_arguments_never_touch_bus(self):
        bus = PinBus()
        i2c = BitBangI2C(bus, 0)
        for device, register in [(0xc0, 0), (-1, 0), (0x60, 0x10000)]:
            with self.assertRaises(ValueError):
                i2c.write_register(device, register, 0)
        with self.assertRaises(ValueError):
            i2c.read_register(0x60, 0, 0)
        self.assertFalse(bus.writes)

    def test_nonfinite_or_unbounded_timing_rejected_without_bus_access(self):
        for option, invalid in (
            ("half_period", (-1, float("nan"), float("inf"), -float("inf"), 1.001)),
            ("stretch_timeout", (0, -1, float("nan"), float("inf"), -float("inf"), 60.001)),
        ):
            for value in invalid:
                with self.subTest(option=option, value=value):
                    bus = PinBus()
                    with self.assertRaises(ValueError):
                        BitBangI2C(bus, 0x1000, **{option: value})
                    self.assertEqual((bus.reads, bus.writes), ([], []))
        # The library stays generic; CLI address restrictions do not apply here.
        BitBangI2C(PinBus(), 0x1000, half_period=0, stretch_timeout=60)
        BitBangI2C(PinBus(), 0x1000, half_period=1, stretch_timeout=0.001)

    def test_interrupt_during_register_transactions_releases_both_pins(self):
        for operation in ("read", "write"):
            with self.subTest(operation=operation):
                bus = PinBus()
                i2c = BitBangI2C(bus, 0x1000)
                interrupt = KeyboardInterrupt("injected transaction interrupt")
                with patch.object(i2c, "_register_address", side_effect=interrupt):
                    with self.assertRaises(KeyboardInterrupt) as raised:
                        if operation == "read":
                            i2c.read_register(0x10, 0x300a, 2)
                        else:
                            i2c.write_register(0x10, 0x3500, 1)
                self.assertIs(raised.exception, interrupt)
                self.assertIn((0x1000, 3), bus.writes)
                self.assertEqual(bus.writes[-1], (0x1000, 0))
                self.assertEqual(bus.control, 0)

    def test_abort_failure_preserves_original_interrupt(self):
        for cleanup_error in (RuntimeError("transport failed"), KeyboardInterrupt("again")):
            with self.subTest(cleanup_error=cleanup_error):
                i2c = TransactionI2C()
                original = KeyboardInterrupt("original")
                with patch.object(i2c, "_register_address", side_effect=original):
                    with patch.object(i2c, "stop", side_effect=cleanup_error) as stop:
                        with self.assertRaises(KeyboardInterrupt) as raised:
                            i2c.write_register(0x10, 0x3500, 1)
                self.assertIs(raised.exception, original)
                stop.assert_called_once_with()

    def test_stop_and_recovery_release_pins_when_interrupted(self):
        for operation in ("stop", "recover"):
            for interrupt_at_first_write in (True, False):
                with self.subTest(operation=operation, first_write=interrupt_at_first_write):
                    bus = PinBus()
                    i2c = BitBangI2C(bus, 0x1000)
                    original = KeyboardInterrupt("original")
                    target = "_delay" if interrupt_at_first_write else "_clock_high"
                    with patch.object(i2c, target, side_effect=[original, None]):
                        with self.assertRaises(KeyboardInterrupt) as raised:
                            getattr(i2c, operation)()
                    self.assertIs(raised.exception, original)
                    self.assertEqual(bus.writes[-1], (0x1000, 0))
                    self.assertEqual(bus.control, 0)

    def test_cleanup_release_failure_does_not_replace_interrupt(self):
        for operation in ("stop", "recover"):
            with self.subTest(operation=operation):
                i2c = BitBangI2C(PinBus(), 0x1000)
                original = KeyboardInterrupt("original")
                releases = [None, RuntimeError("release failed")] if operation == "recover" else [RuntimeError("release failed")]
                with patch.object(i2c, "_clock_high", side_effect=original):
                    with patch.object(i2c, "release", side_effect=releases) as release:
                        with self.assertRaises(KeyboardInterrupt) as raised:
                            getattr(i2c, operation)()
                self.assertIs(raised.exception, original)
                self.assertEqual(release.call_count, len(releases))


class RawSensorCLITests(unittest.TestCase):
    """Raw diagnostics cannot bypass the paired camera state machine."""

    @staticmethod
    def pair_bus(overrides=None):
        values = {
            ov9281.PAIR_BASE: ov9281.PAIR_MAGIC,
            ov9281.PAIR_BASE + 4: 0,
            ov9281.PAIR_BASE + ov9281.PAIR_RUN: 0,
            ov9281.PAIR_BASE + ov9281.PAIR_STATUS: ov9281.PAIR_STATUS_DRAINED,
            ov9281.PAIR_BASE + ov9281.PAIR_FAULT: 0,
            ov9281.CONTROL_BASE: 0,
            ov9281.CONTROL_BASE + 0x14: 2,
            ov9281.CONTROL_BASE + 0x100: 0,
            ov9281.CONTROL_BASE + 0x114: 2,
        }
        values.update(overrides or {})
        bus = Mock()
        bus.read.side_effect = values.__getitem__
        return bus

    def test_protected_registers_rejected_before_transport(self):
        protected = (0x0100, 0x0103, 0x0106, 0x3006, 0x3027, 0x3030, 0x3666,
                     0x3823, 0x4311, 0x4312, 0x4313, 0x4314, 0x4315, 0x4316, 0x4f00)
        for base in ("0x30020000", "0x30021000"):
            for register in protected:
                # Check a direct access and an overlap from the prior byte.
                for start, data in ((register, ["1"]), (register - 1, ["0", "1"])):
                    with self.subTest(base=base, register=register, start=start):
                        stderr = io.StringIO()
                        with patch("litex.RemoteClient") as client, redirect_stderr(stderr):
                            with self.assertRaises(SystemExit) as raised:
                                control_main(["--i2c-base", base, "write", "0x10",
                                              hex(start), *data])
                        self.assertEqual(raised.exception.code, 2)
                        self.assertIn("tools.lutcam.capture_dual", stderr.getvalue())
                        client.assert_not_called()

    def test_noncanonical_bases_rejected_before_transport_for_every_action(self):
        for base in ("0x30024000", "0x30024100", "0x30020004", "0x30021001", "0x1000", "-4"):
            for arguments in (("read", "0x10", "0x300a"),
                              ("write", "0x10", "0x3500", "1"), ("recover",)):
                with self.subTest(base=base, arguments=arguments):
                    with patch("litex.RemoteClient") as client, redirect_stderr(io.StringIO()):
                        with self.assertRaises(SystemExit) as raised:
                            control_main(["--i2c-base", base, *arguments])
                    self.assertEqual(raised.exception.code, 2)
                    client.assert_not_called()

    def test_unsafe_cli_timing_rejected_before_transport_for_every_action(self):
        for option, values in (("--half-period", ("nan", "inf", "-inf", "-1", "1.001")),
                               ("--stretch-timeout", ("nan", "inf", "-inf", "0", "60.001"))):
            for value in values:
                for arguments in (("read", "0x10", "0x300a"),
                                  ("write", "0x10", "0x3500", "1"), ("recover",)):
                    with self.subTest(option=option, value=value, arguments=arguments):
                        with patch("litex.RemoteClient") as client, redirect_stderr(io.StringIO()):
                            with self.assertRaises(SystemExit) as raised:
                                control_main(["--i2c-base", "0x30021000", f"{option}={value}", *arguments])
                        self.assertEqual(raised.exception.code, 2)
                        client.assert_not_called()

    def test_invalid_write_ranges_and_bytes_rejected_before_transport(self):
        invalid = (("0x10", "0xffff", "0", "1"),
                   ("0x80", "0x3500", "1"),
                   ("0x10", "0x10000", "1"),
                   ("0x10", "0x3500", "256"),
                   ("0x10", "0x3500", "-1"))
        for arguments in invalid:
            with self.subTest(arguments=arguments):
                with patch("litex.RemoteClient") as client, redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as raised:
                        control_main(["--i2c-base", "0x30021000", "write", *arguments])
                self.assertEqual(raised.exception.code, 2)
                client.assert_not_called()

    def test_busy_or_incompatible_pair_prevents_all_i2c_activity(self):
        unavailable = (
            {ov9281.PAIR_BASE: 0xffffffff},
            {ov9281.PAIR_BASE: 0xdeadbeef},
            {ov9281.CONTROL_BASE: 1},
            {ov9281.CONTROL_BASE + 0x114: 1},
            {ov9281.PAIR_BASE + ov9281.PAIR_RUN: 1},
            {ov9281.PAIR_BASE + ov9281.PAIR_STATUS: 0},
            {ov9281.PAIR_BASE + ov9281.PAIR_STATUS: ov9281.PAIR_STATUS_DRAINED | 1},
            {ov9281.PAIR_BASE + ov9281.PAIR_FAULT: 1},
        )
        for base in ("0x30020000", "0x30021000"):
            for action in (("write", "0x10", "0x3500", "1"), ("recover",)):
                for overrides in unavailable:
                    with self.subTest(base=base, action=action, overrides=overrides):
                        bus = self.pair_bus(overrides)
                        with patch("litex.RemoteClient", return_value=bus):
                            with self.assertRaises(ov9281.CameraError):
                                control_main(["--i2c-base", base, *action])
                        bus.open.assert_called_once_with()
                        bus.close.assert_called_once_with()
                        bus.write.assert_not_called()
                        # The strict read mapping also rejects every I2C pin read.
                        self.assertTrue(bus.read.called)

    def test_idle_pair_permits_generic_write_after_guard(self):
        bus = self.pair_bus()
        events = []
        original_read = bus.read.side_effect
        bus.read.side_effect = lambda address: (events.append(("read", address)),
                                               original_read(address))[1]
        with patch("litex.RemoteClient", return_value=bus):
            with patch.object(BitBangI2C, "write_register",
                              side_effect=lambda *args: events.append(("write", *args))) as write:
                control_main(["--i2c-base", "0x30021000", "write", "0x10",
                              "0x3500", "0x00", "0x19", "0x00"])
        write.assert_called_once_with(0x10, 0x3500, [0, 0x19, 0])
        self.assertEqual(events[-1], ("write", 0x10, 0x3500, [0, 0x19, 0]))
        self.assertEqual(events[0], ("read", ov9281.PAIR_BASE))
        bus.close.assert_called_once_with()

    def test_idle_pair_permits_explicit_recovery(self):
        bus = self.pair_bus()
        with patch("litex.RemoteClient", return_value=bus):
            with patch.object(BitBangI2C, "recover") as recover:
                control_main(["--i2c-base", "0x30020000", "recover"])
        recover.assert_called_once_with()
        bus.read.assert_any_call(ov9281.PAIR_BASE + ov9281.PAIR_FAULT)
        bus.close.assert_called_once_with()

    def test_reads_remain_available_without_pair_firmware(self):
        bus = Mock()
        bus.read.side_effect = AssertionError("raw read must not require paired metadata")
        stdout = io.StringIO()
        with patch("litex.RemoteClient", return_value=bus), redirect_stdout(stdout):
            with patch.object(BitBangI2C, "read_register", return_value=b"\x92\x81") as read:
                control_main(["--i2c-base", "0x30021000", "read", "0x10",
                              "0x300a", "--length", "2"])
        read.assert_called_once_with(0x10, 0x300a, 2)
        self.assertEqual(stdout.getvalue(), "92 81\n")
        bus.read.assert_not_called()
        bus.close.assert_called_once_with()

    def test_library_still_allows_coordinator_stream_writes(self):
        i2c = TransactionI2C()
        i2c.write_register(0x10, 0x0100, 1)
        self.assertEqual(i2c.events, [
            ("start", False), ("write", 0x20), ("write", 0x01),
            ("write", 0x00), ("write", 0x01), ("stop",),
        ])


class ReceiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.image = b"".join((word & 0xffff).to_bytes(2, "big") for word in range(64000))

    def test_exact_header_and_native_image_word_order(self):
        data = datagram(self.image[:FRAGMENT_SIZE], camera=1, frame=0x12345678,
                        sequence=0x90abcdef)
        self.assertEqual(data[:32].hex(),
                         "4c555443000101201234567890abcdef00000000000004000000000000000000")
        packet = parse_datagram(data)
        self.assertEqual(packet.payload[:6], b"\x00\x00\x00\x01\x00\x02")
        self.assertEqual((packet.camera, packet.frame_id, packet.sequence),
                         (1, 0x12345678, 0x90abcdef))

    def test_child_tuple_33_bit_payload(self):
        values = tuple((1 << 32) | value for value in range(9))
        raw = datagram(struct.pack(">9Q", *values), stream=CHILD)
        self.assertEqual(parse_datagram(raw).child_tuples(), values)
        with self.assertRaises(PacketError):
            parse_datagram(datagram(struct.pack(">9Q", 1 << 33, *range(8)), stream=CHILD))


    def test_accepted_pose_only(self):
        words = list(range(22))
        for version in [0]:
            words[8] = (version << 28) | 0xf
            raw = datagram(struct.pack(">22I", *words), stream=POSE)
            self.assertEqual(parse_datagram(raw).pose_words(), tuple(words))
        for status in [0, 0x11]:
            words[8] = status
            with self.assertRaises(PacketError):
                parse_datagram(datagram(struct.pack(">22I", *words), stream=POSE))
        with self.assertRaises(PacketError):
            parse_datagram(datagram(struct.pack(">8I", *range(8)), stream=POSE))

    def test_truncation_and_header_validation(self):
        raw = datagram(self.image[:FRAGMENT_SIZE])
        invalid = [raw[:31], raw[:-1], raw + b"\x00", b"BAD!" + raw[4:]]
        for word, value in [(1, 0x00020120), (1, 0x00000420),
                            (1, 0x00000118), (6, 2), (7, 1)]:
            edited = bytearray(raw)
            struct.pack_into(">I", edited, word * 4, value)
            invalid.append(edited)
        for data in invalid:
            with self.subTest(length=len(data)):
                with self.assertRaises(PacketError):
                    parse_datagram(data)


    def test_packet_constructor_defaults_to_unversioned_current_layout(self):
        self.assertEqual(Packet(0, IMAGE, 7, 0, 0, 0, b"ab").version, 0)


    def test_current_image_fragments_go_directly_to_decoder(self):
        # Arbitrary bytes deliberately need not form valid LWC packets. The
        # receiver validates the transport header, not the image codec.
        for payload in [b"ab", b"\xff\xff", b"\x00\x00" * 512, b"\xff\xff" * 704]:
            receiver = StreamReceiver()
            raw = datagram(payload, version=0, camera=1, frame=29,
                           offset=1024, sequence=300)
            self.assertEqual(receiver.feed(raw), Packet(1, IMAGE, 29, 300, 1024, 0, payload, 0))
            self.assertIsNone(receiver.feed(raw))

    def test_current_reordered_and_gapped_fragments_return_immediately(self):
        receiver = StreamReceiver()
        packets = [datagram(b"ab", version=0),
                   datagram(b"ef", version=0, offset=4, sequence=2),
                   datagram(b"cd", version=0, offset=2, sequence=1)]
        for raw in packets:
            self.assertEqual(receiver.feed(raw), parse_datagram(raw))
        stats = receiver.statistics()
        self.assertEqual(stats["streams"]["0:1"]["missing_packets"], 0)
        self.assertEqual(stats["streams"]["0:1"]["reordered_packets"], 1)
        self.assertEqual(receiver.feed(datagram(b"ij", version=0, offset=8, sequence=4)).payload, b"ij")
        self.assertEqual(receiver.statistics()["streams"]["0:1"]["missing_packets"], 1)

    def test_current_range_placement_is_decoder_responsibility(self):
        receiver = StreamReceiver()
        packets = [datagram(b"abcdef", version=0),
                   datagram(b"ZZ", version=0, offset=2, sequence=1),
                   datagram(b"YY", version=0, offset=0, sequence=2)]
        self.assertEqual([receiver.feed(raw) for raw in packets],
                         [parse_datagram(raw) for raw in packets])


    def test_current_random_fragment_order_deduplicates_without_buffering(self):
        rng = random.Random(0x335443)
        for frame in range(32):
            data = self.image[:2 * rng.randint(1, 64000)]
            packets = image_packets(data, camera=frame % 2, frame=frame, version=0)
            order = packets + rng.sample(packets, min(len(packets), 3))
            rng.shuffle(order)
            receiver = StreamReceiver()
            delivered = set()
            for raw in order:
                packet = parse_datagram(raw)
                result = receiver.feed(raw)
                if packet.sequence in delivered:
                    self.assertIsNone(result)
                else:
                    self.assertEqual(result, packet)
                delivered.add(packet.sequence)
            self.assertEqual(len(delivered), len(packets))

    def test_current_child_batches_preserve_each_group(self):
        groups = tuple(tuple((1 << 32) | (index << 8) | i for i in range(9))
                       for index in range(4))
        for size in range(1, len(groups) + 1):
            packet = parse_datagram(datagram(child_batch(groups[:size]), version=0, stream=CHILD))
            self.assertEqual(packet.child_groups(), groups[:size])
            if size == 1:
                self.assertEqual(packet.child_tuples(), groups[0])
            else:
                with self.assertRaisesRegex(ValueError, "child_groups"):
                    packet.child_tuples()

    def test_current_child_batch_validates_every_fixed_record_and_tuple(self):
        first = child_batch([tuple(range(9))])
        for tail in [struct.pack(">I", 9), b"\x00\x00", bytes(64),
                     child_batch([(1 << 33,) + tuple(range(8))])]:
            with self.assertRaises(PacketError):
                parse_datagram(datagram(first + tail, version=0, stream=CHILD))
        for body in [b"", bytes(8)]:
            with self.assertRaises(PacketError):
                parse_datagram(datagram(body, version=0, stream=CHILD))
        with self.assertRaises(PacketError):
            parse_datagram(datagram(first, version=0, stream=CHILD, offset=2))
        # The packet contract caps batches at four groups, even when a larger
        # batch would fit beneath the generic UDP datagram ceiling.
        for count in [5, 19]:
            many = tuple(tuple(index * 9 + i for i in range(9)) for index in range(count))
            with self.assertRaisesRegex(PacketError, "one through four"):
                parse_datagram(datagram(child_batch(many), version=0, stream=CHILD))

    def test_current_pose_batches_validate_every_record(self):
        poses = []
        for index in range(2):
            words = list(range(index * 22, (index + 1) * 22))
            words[8] = 0x0000000f
            poses.append(tuple(words))
        body = b"".join(struct.pack(">22I", *words) for words in poses)
        packet = parse_datagram(datagram(body, version=0, stream=POSE))
        self.assertEqual(packet.poses(), tuple(poses))
        with self.assertRaisesRegex(ValueError, "poses"):
            packet.pose_words()
        for version in [0]:
            single = parse_datagram(datagram(body[:88], version=version, stream=POSE))
            self.assertEqual(single.poses(), (poses[0],))
            self.assertEqual(single.pose_words(), poses[0])
        for status in [0, 0x11]:
            bad = bytearray(body)
            struct.pack_into(">I", bad, 88 + 32, status)
            with self.assertRaises(PacketError):
                parse_datagram(datagram(bad, version=0, stream=POSE))
        for bad in [b"", body[:-1], body + b"ab"]:
            with self.assertRaises(PacketError):
                parse_datagram(datagram(bad, version=0, stream=POSE))
        for count in [3, 16]:
            with self.assertRaisesRegex(PacketError, "at most two"):
                parse_datagram(datagram(body[:88] * count, version=0, stream=POSE))
        with self.assertRaises(PacketError):
            parse_datagram(datagram(body, version=0, stream=POSE, offset=2))

    def test_current_cli_emits_one_json_record_per_group_or_pose(self):
        groups = (tuple(range(9)), tuple(range(9, 18)))
        pose = tuple(0x00000001 if index == 8 else index for index in range(22))
        packets = [datagram(child_batch(groups), version=0, stream=CHILD, camera=1),
                   datagram(struct.pack(">22I", *pose) * 2, version=0, stream=POSE, camera=1)]
        stdout, stderr = io.StringIO(), io.StringIO()
        with patch("tools.lutcam.receive.socket.socket") as socket_factory, \
                redirect_stdout(stdout), redirect_stderr(stderr):
            sock = socket_factory.return_value.__enter__.return_value
            sock.recvfrom.side_effect = [(raw, ("127.0.0.1", 6000)) for raw in packets] + [KeyboardInterrupt]
            receive_main(["--port", "6000"])
        records = [json.loads(line) for line in stdout.getvalue().splitlines()]
        self.assertEqual(records, [
            {"camera": 1, "frame_id": 7, "stream": "child", "tuples": list(group)}
            for group in groups
        ] + [{"camera": 1, "frame_id": 7, "stream": "pose", "words": list(pose)}] * 2)

    def test_current_cli_captures_raw_fragments_without_claiming_frame_completion(self):
        packets = [datagram(b"ab", version=0, camera=1, frame=7, offset=4, sequence=1),
                   datagram(b"cd", version=0, camera=1, frame=7, offset=0, sequence=0)]
        stdout, stderr = io.StringIO(), io.StringIO()
        with tempfile.TemporaryDirectory() as directory:
            with patch("tools.lutcam.receive.socket.socket") as socket_factory, \
                    redirect_stdout(stdout), redirect_stderr(stderr):
                sock = socket_factory.return_value.__enter__.return_value
                sock.recvfrom.side_effect = [(raw, ("127.0.0.1", 6000)) for raw in packets] + [KeyboardInterrupt]
                receive_main(["--port", "6000", "--output-dir", directory])
            records = [json.loads(line) for line in stdout.getvalue().splitlines()]
            self.assertEqual(len(records), 2)
            for record, raw in zip(records, packets):
                packet = parse_datagram(raw)
                fragment = Path(record.pop("file"))
                self.assertEqual(record, dict(camera=1, frame_id=7, stream="image_fragment",
                                              offset=packet.offset, sequence=packet.sequence, bytes=2))
                self.assertEqual(fragment.name, "camera1-frame00000007"
                                 f"-offset{packet.offset:08x}-sequence{packet.sequence:08x}.lwcpart")
                self.assertEqual(fragment.read_bytes(), packet.payload)
            self.assertFalse(list(Path(directory).glob("*.lwc")))

    def test_sequences_wrap_and_repair_reordered_gap(self):
        tracker = SequenceTracker(window=8)
        for sequence in [0xfffffffe, 1]:
            self.assertTrue(tracker.add(sequence))
        self.assertEqual(tracker.missing, 2)
        self.assertTrue(tracker.add(0xffffffff))
        self.assertTrue(tracker.add(0))
        self.assertEqual(tracker.missing, 0)
        self.assertFalse(tracker.add(0))
        self.assertTrue(tracker.add(100))
        self.assertFalse(tracker.add(2))
        self.assertLessEqual(len(tracker.holes), 7)
        self.assertEqual(tracker.too_old, 1)


if __name__ == "__main__":
    unittest.main()
