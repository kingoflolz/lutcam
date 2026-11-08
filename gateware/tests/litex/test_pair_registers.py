"""Pair control commands and stopped-state guards through the real decoder."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from migen import Module, Signal
from migen.sim import passive, run_simulation

from lutcam.counters import SharedPacketCounters
from lutcam.camera import CALIBRATION, SHARED_CALIBRATION, shared_detector_configuration
from lutcam.packet import SharedPacketTransport
from lutcam.registers import CONTROL_BASE, IntegrationRegisters
from test_integration_registers import (FakeCamera, calibration_addresses,
                                        camera_address, packet_address, transfer)
from test_packet import Capture, child_group, idle


class FakePair(Module):
    """Controllable controller states for independently exercising CSR guards."""

    def __init__(self):
        for name in ("run_request", "busy", "fault", "start", "stop", "clear", "reset", "snapshot"):
            setattr(self, name, Signal(name=name))
        for name in ("drained", "ready", "locked"):
            setattr(self, name, Signal(reset=1, name=name))
        self.status = Signal(10)
        self.watchdog_cycles = Signal(22, reset=4_000_000)
        self.max_skew_cycles = Signal(22, reset=12_000)
        self.sync_period = Signal(21, reset=994_812)
        self.sync_width = Signal(21, reset=1536)
        self.sync_enable = Signal()
        self.min_latency_cycles = Signal(21, reset=2)
        self.max_latency_cycles = Signal(21, reset=900_000)
        self.phase0_cycles, self.phase1_cycles = Signal(21), Signal(21)
        self.snap_sync = Signal(64)
        self.snap_sof0, self.snap_sof1 = Signal(64), Signal(64)
        for name in ("fault_cause", "pairs_started", "pairs_completed", "snap_epoch",
                     "snap_sof0_count", "snap_sof1_count", "snap_sync_count",
                     "snap_matched_pairs", "snap_missing_events"):
            setattr(self, name, Signal(32, name=name))
        for name in ("snap_skew", "snap_min_skew", "snap_max_skew", "snap_latency0",
                     "snap_latency1", "snap_min_latency", "snap_max_latency", "snap_residual_skew",
                     "snap_min_residual_skew", "snap_max_residual_skew"):
            setattr(self, name, Signal((32, True), name=name))


class PairRegisterHarness(Module):
    def __init__(self, shared_configuration=False):
        common = shared_detector_configuration() if shared_configuration else None
        self.cameras = [FakeCamera(common), FakeCamera(common)]
        self.submodules.camera0, self.submodules.camera1 = self.cameras
        self.submodules.pair = FakePair()
        self.submodules.transport = SharedPacketTransport(image_slots=2, child_slots=2,
            pose_slots=2, frame_words=16, fragment_words=8,
            external_destination=True, external_context_held=True)
        self.submodules.counters = SharedPacketCounters(packet_totals=True,
            with_destinations=True, fixed_packet_layout=True)
        self.submodules.registers = IntegrationRegisters(self.cameras, self.transport,
            self.counters, banked_destinations=True, pair=self.pair)
        self.events = []

def command_monitor(dut):
    @passive
    def monitor():
        while True:
            values = []
            for name in ("start", "stop", "clear", "reset", "snapshot"):
                values.append((yield getattr(dut.pair, name)))
            for camera in dut.cameras:
                values.extend(((yield camera.clear.re), (yield camera.soft_reset.re),
                               (yield camera.calibration_commit.re)))
            values.extend(((yield dut.counters.clear_camera),
                           (yield dut.counters.clear_statistics)))
            if any(values):
                dut.events.append(tuple(values))
            yield
    return monitor()


class PairRegisterTests(unittest.TestCase):
    def test_shared_detector_aliases_byte_lanes_and_pair_wide_write_guards(self):
        dut = PairRegisterHarness(shared_configuration=True)
        bus = dut.registers.bus
        addresses = [calibration_addresses(index) for index in range(2)]

        def bench():
            for index, (name, width, default) in enumerate(CALIBRATION):
                words = (width + 31) // 32
                values = []
                for part in range(words):
                    mask = (1 << min(32, width - (words - part - 1) * 32)) - 1
                    first = (0x12345678 + index * 0x1020304 + part) & mask
                    alias = part % 2
                    self.assertEqual((yield from transfer(bus, addresses[alias][name] + part * 4,
                                                         first)), (0, 0))
                    # A write via the other alias updates only selected bytes.
                    patch = 0xaabbccdd & mask
                    target = 1 - alias if name in SHARED_CALIBRATION else alias
                    yield from transfer(bus, addresses[target][name] + part * 4, patch, sel=5)
                    expected = ((first & 0xff00ff00) | (patch & 0x00ff00ff)) & mask
                    values.append(expected)
                    for camera in range(2):
                        want = expected if name in SHARED_CALIBRATION or camera == alias else default
                        self.assertEqual((yield from transfer(bus, addresses[camera][name] + part * 4)),
                                         (want, 0))
                if name in SHARED_CALIBRATION:
                    expected_full = sum(value << ((words - part - 1) * 32)
                                        for part, value in enumerate(values))
                    for camera in dut.cameras:
                        self.assertEqual((yield camera.configuration[name]), expected_full)

            # Each global condition blocks writes through either alias, even
            # when the addressed camera itself appears idle.
            for blocked, value, restore in ((dut.pair.busy, 1, 0),
                                            (dut.pair.drained, 0, 1),
                                            (dut.cameras[0].idle, 0, 1),
                                            (dut.cameras[1].idle, 0, 1)):
                yield blocked.eq(value)
                yield
                for alias in range(2):
                    address = addresses[alias]["mean_residual_limit_q16"]
                    before = yield from transfer(bus, address)
                    self.assertEqual((yield from transfer(bus, address, 0xffffffff))[1], 1)
                    self.assertEqual((yield from transfer(bus, address)), before)
                    self.assertEqual((yield dut.cameras[alias].configuration_error.status), 1)
                yield blocked.eq(restore)
                yield
            # A shared setting does not merge the per-camera commit/valid flags.
            yield from transfer(bus, camera_address(0, "calibration_commit"), 1)
            self.assertEqual((yield dut.cameras[0].calibration_valid.status), 1)
            self.assertEqual((yield dut.cameras[1].calibration_valid.status), 0)
            yield from transfer(bus, camera_address(1, "calibration_commit"), 1)
            self.assertEqual((yield dut.cameras[1].calibration_valid.status), 1)

        run_simulation(dut, bench())

    def test_retained_solver_diagnostics_are_readable_while_faulted_and_read_only(self):
        dut = PairRegisterHarness()
        names = ("solver_debug_status", "solver_debug_protocol", "solver_debug_drop", "solver_debug_magic")

        def bench():
            yield dut.pair.fault.eq(1)
            yield dut.pair.drained.eq(0)
            for camera in dut.cameras:
                yield camera.idle.eq(0)
            for index, camera in enumerate(dut.cameras):
                values = (0x1f0 | index, 0x4321abcd ^ index, 0x1234567 ^ index, 0x4c464400)
                for name, value in zip(names, values):
                    yield getattr(camera, name).status.eq(value)
                yield
                for position, (name, value) in enumerate(zip(names, values)):
                    address = index * 0x100 + 0x40 + position * 4
                    observed, error = yield from transfer(dut.registers.bus, address)
                    self.assertEqual((observed, error), (value, False))
                    _, error = yield from transfer(dut.registers.bus, address, 0)
                    self.assertTrue(error)
                    self.assertEqual((yield getattr(camera, name).status), value)
        run_simulation(dut, bench())

    def test_pair_map_snapshot_word_order_and_read_only_errors(self):
        dut = PairRegisterHarness()
        bus, pair = dut.registers.bus, dut.pair
        registers = dut.registers.regions["pair"].obj
        self.assertEqual([register.origin for register in registers],
                         list(range(CONTROL_BASE + 0x200, CONTROL_BASE + 0x2a4, 4)))

        def bench():
            yield pair.status.eq(0x2a5)
            yield pair.fault_cause.eq(0x43210)
            yield pair.snap_sof0.eq(0x123456789abcdef0)
            yield pair.snap_sof1.eq(0xfedcba9876543210)
            yield pair.snap_skew.eq(-7)
            yield pair.snap_min_skew.eq(-9)
            yield pair.snap_max_skew.eq(11)
            yield pair.snap_epoch.eq(42)
            yield pair.pairs_started.eq(8)
            yield pair.pairs_completed.eq(7)
            yield pair.snap_sync.eq(0x12345678abcdef00)
            yield pair.snap_latency0.eq(-3)
            yield pair.snap_latency1.eq(120)
            yield pair.snap_min_latency.eq(-5)
            yield pair.snap_max_latency.eq(125)
            yield pair.snap_residual_skew.eq(-13)
            yield pair.snap_min_residual_skew.eq(-15)
            yield pair.snap_max_residual_skew.eq(17)
            yield
            expected = {0x200: 0x4c435031, 0x204: 0, 0x20c: 0x2a5,
                        0x214: 0x43210, 0x218: 8, 0x21c: 7,
                        0x22c: 42, 0x230: 0x9abcdef0, 0x234: 0x12345678,
                        0x238: 0x76543210, 0x23c: 0xfedcba98,
                        0x240: 0xfffffff9, 0x244: 0xfffffff7, 0x248: 11,
                        0x270: 0xabcdef00, 0x274: 0x12345678,
                        0x278: 0xfffffffd, 0x27c: 120, 0x280: 0xfffffffb, 0x284: 125,
                        0x298: 0xfffffff3, 0x29c: 0xfffffff1, 0x2a0: 17}
            for address, value in expected.items():
                self.assertEqual((yield from transfer(bus, address)), (value, 0))
                self.assertEqual((yield from transfer(bus, address, 0))[1], 1)
                self.assertEqual((yield from transfer(bus, address)), (value, 0))
            for address in (camera_address(0, "status"), packet_address(1, 5),
                            packet_address(1, 9), packet_address(1, 3)):
                self.assertEqual((yield from transfer(bus, address, 0))[1], 1)
            yield pair.busy.eq(1)
            yield
            for index in range(2):
                self.assertEqual((yield from transfer(bus, camera_address(index, "run"))), (1, 0))
            self.assertEqual((yield from transfer(bus, 0x208)), (0, 0))

        run_simulation(dut, [bench(), command_monitor(dut)])

    def test_run_guards_idempotence_byte_lanes_and_invalid_commands(self):
        dut = PairRegisterHarness()
        bus, pair = dut.registers.bus, dut.pair

        def bench():
            self.assertEqual((yield from transfer(bus, 0x208, 1))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x26c, 1))[1], 0)
            for field, value in (("ready", 0), ("locked", 0), ("fault", 1), ("busy", 1)):
                signal = getattr(pair, field)
                yield signal.eq(value)
                yield
                self.assertEqual((yield from transfer(bus, 0x208, 1))[1], 1)
                yield signal.eq(1 - value)
                yield
            self.assertFalse(dut.events)
            for index in range(2):
                for name in ("run", "clear", "soft_reset"):
                    self.assertEqual((yield from transfer(bus, camera_address(index, name), 1))[1], 1)
            self.assertFalse(dut.events)
            self.assertEqual((yield from transfer(bus, 0x208, 1, sel=2))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x208, 1, sel=0))[1], 0)
            self.assertFalse(dut.events)
            self.assertEqual((yield from transfer(bus, 0x208, 1))[1], 0)
            self.assertEqual(sum(event[0] for event in dut.events), 1)
            yield pair.run_request.eq(1)
            yield pair.busy.eq(1)
            yield pair.ready.eq(0)
            yield pair.locked.eq(0)
            yield pair.fault.eq(1)
            yield
            self.assertEqual((yield from transfer(bus, 0x208, 1))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x208, 0))[1], 0)
            self.assertEqual(sum(event[0] for event in dut.events), 2)
            self.assertEqual(sum(event[1] for event in dut.events), 1)
            self.assertEqual((yield from transfer(bus, 0x228, 1, sel=8))[1], 0)
            self.assertEqual(sum(event[4] for event in dut.events), 1)

        run_simulation(dut, [bench(), command_monitor(dut)])

    def test_configuration_requires_both_idle_and_drained(self):
        dut = PairRegisterHarness()
        bus, pair = dut.registers.bus, dut.pair
        cfg = calibration_addresses(0)["inv_fx_q2_30"]
        writes = ((cfg, 0x12345678), (camera_address(0, "calibration_commit"), 1),
                  (camera_address(0, "sensor_control"), 5), (packet_address(0, 0), 1),
                  (packet_address(0, 1), 0xc0a80101), (packet_address(0, 2), 7000),
                  (packet_address(0, 8), 1), (0x220, 4_100_000), (0x224, 10000))

        def bench():
            for signal, value in ((pair.busy, 1), (pair.drained, 0), (dut.cameras[1].idle, 0)):
                yield signal.eq(value)
                yield
                for address, data in writes:
                    self.assertEqual((yield from transfer(bus, address, data))[1], 1)
                self.assertEqual((yield dut.cameras[0].configuration["inv_fx_q2_30"]), 0)
                self.assertEqual((yield dut.cameras[0].sensor_control.storage), 0)
                self.assertEqual((yield dut.transport.channels[0].enable), 0)
                self.assertFalse(dut.events)
                yield signal.eq(1 - value)
                yield
            for address, data in writes:
                self.assertEqual((yield from transfer(bus, address, data))[1], 0)
            self.assertEqual((yield dut.cameras[0].sensor_control.storage), 5)
            self.assertEqual((yield dut.cameras[1].sensor_control.storage), 0)
            self.assertEqual((yield from transfer(bus, camera_address(0, "sensor_control"), 7))[1], 1)
            self.assertEqual((yield from transfer(bus, camera_address(0, "sensor_control"), 7, sel=2))[1], 0)
            self.assertEqual((yield dut.cameras[0].sensor_control.storage), 5)
            self.assertEqual((yield from transfer(bus, packet_address(0, 1))), (0xc0a80101, 0))
            self.assertEqual((yield from transfer(bus, packet_address(0, 2))), (7000, 0))

        run_simulation(dut, [bench(), command_monitor(dut)])

    def test_timer_partial_writes_validate_merged_values(self):
        dut = PairRegisterHarness()
        bus = dut.registers.bus

        def bench():
            self.assertEqual((yield from transfer(bus, 0x220)), (4_000_000, 0))
            self.assertEqual((yield from transfer(bus, 0x224)), (12_000, 0))
            for address, value in ((0x220, 0), (0x220, 12_000), (0x224, 0), (0x224, 4_000_000),
                                   (0x220, 0x1000000), (0x224, 0x1000000)):
                self.assertEqual((yield from transfer(bus, address, value))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x224, 0x100))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x220, 0x200))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x224, 0xff, sel=1))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x224)), (0x1ff, 0))
            self.assertEqual((yield from transfer(bus, 0x224, 0x200, sel=2))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x220, 0x100, sel=2))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x224)), (0x1ff, 0))
            self.assertEqual((yield from transfer(bus, 0x220)), (0x200, 0))

        run_simulation(dut, bench())

    def test_shared_timing_merge_zero_extends_and_rejects_selected_upper_bytes(self):
        dut = PairRegisterHarness()
        bus = dut.registers.bus

        def bench():
            for address in (0x220, 0x224, 0x264, 0x268, 0x288, 0x28c, 0x290, 0x294):
                original, error = yield from transfer(bus, address)
                self.assertEqual(error, 0)
                width = 22 if address in (0x220, 0x224) else 21
                self.assertLess(original, 1 << width)
                self.assertEqual((yield from transfer(bus, address, 1 << width, sel=4))[1], 1)
                self.assertEqual((yield from transfer(bus, address, 0x00ffffff, sel=8))[1], 0)
                self.assertEqual((yield from transfer(bus, address, 0x01000000, sel=8))[1], 1)
                self.assertEqual((yield from transfer(bus, address, 0xff000000 | original))[1], 1)
                self.assertEqual((yield from transfer(bus, address, 0xff000000 | original, sel=1))[1], 0)
                self.assertEqual((yield from transfer(bus, address)), (original, 0))

        run_simulation(dut, bench())

    def test_master_generator_guards_byte_merges_and_recovery_preservation(self):
        dut = PairRegisterHarness()
        bus, pair = dut.registers.bus, dut.pair

        def bench():
            for address, value in ((0x264, 994812), (0x268, 1536), (0x26c, 0),
                                   (0x288, 2), (0x28c, 900000)):
                self.assertEqual((yield from transfer(bus, address)), (value, 0))
            for address, value in ((0x264, 0), (0x264, 900000), (0x268, 0),
                                   (0x268, 994812), (0x288, 0), (0x288, 900000),
                                   (0x28c, 2), (0x28c, 994812)):
                self.assertEqual((yield from transfer(bus, address, value))[1], 1)
            for address, value in ((0x264, 0x100000), (0x268, 0x100),
                                   (0x288, 0x100), (0x28c, 0x200)):
                self.assertEqual((yield from transfer(bus, address, value))[1], 0)
            for address in (0x268, 0x288):
                self.assertEqual((yield from transfer(bus, address, 0xff, sel=1))[1], 0)
                self.assertEqual((yield from transfer(bus, address)), (0x1ff, 0))
            self.assertEqual((yield from transfer(bus, 0x268, 0x100000, sel=4))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x288, 0x200, sel=2))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x28c, 0x100, sel=2))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x264, 0x200))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x264, 0x400))[1], 0)
            settings = ((0x264, 0x400), (0x268, 0x1ff), (0x288, 0x1ff), (0x28c, 0x200))
            for signal, value in ((pair.busy, 1), (pair.drained, 0), (dut.cameras[1].idle, 0)):
                yield signal.eq(value)
                yield
                for address, data in settings:
                    self.assertEqual((yield from transfer(bus, address, data))[1], 1)
                self.assertEqual((yield from transfer(bus, 0x26c, 1))[1], 1)
                yield signal.eq(1 - value)
                yield
            self.assertEqual((yield from transfer(bus, 0x26c, 1, sel=2))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x26c)), (0, 0))
            self.assertEqual((yield from transfer(bus, 0x26c, 3))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x26c)), (1, 0))
            for address, value in settings:
                self.assertEqual((yield from transfer(bus, address, value))[1], 1)
            for address, value in ((0x220, 4_100_000), (0x224, 10_000)):
                self.assertEqual((yield from transfer(bus, address, value))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x220)), (4_000_000, 0))
            self.assertEqual((yield from transfer(bus, 0x224)), (12_000, 0))
            for command in (0x260, 0x210):
                self.assertEqual((yield from transfer(bus, command, 1))[1], 0)
                for address, value in settings:
                    self.assertEqual((yield from transfer(bus, address)), (value, 0))
                self.assertEqual((yield from transfer(bus, 0x26c)), (1, 0))
            self.assertEqual((yield from transfer(bus, 0x26c, 0))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x220, 4_100_000))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x224, 10_000))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x268, 0x100))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x208, 1))[1], 1)

        run_simulation(dut, bench())

    def test_phase_guards_period_dependencies_and_21_22_bit_timing_limit(self):
        dut = PairRegisterHarness()
        bus, pair = dut.registers.bus, dut.pair

        def bench():
            for address in (0x290, 0x294):
                self.assertEqual((yield from transfer(bus, address)), (0, 0))
                self.assertEqual((yield from transfer(bus, address, 994812))[1], 1)
                self.assertEqual((yield from transfer(bus, address, 994811))[1], 0)
                self.assertEqual((yield from transfer(bus, 0x264, 994811))[1], 1)
                self.assertEqual((yield from transfer(bus, address, 0))[1], 0)
                self.assertEqual((yield from transfer(bus, address, 0x100))[1], 0)
                self.assertEqual((yield from transfer(bus, address, 0xff, sel=1))[1], 0)
                self.assertEqual((yield from transfer(bus, address)), (0x1ff, 0))
                self.assertEqual((yield from transfer(bus, address, 0x100000, sel=4))[1], 1)
                self.assertEqual((yield from transfer(bus, address)), (0x1ff, 0))
            for signal, value in ((pair.busy, 1), (pair.drained, 0), (dut.cameras[1].idle, 0)):
                yield signal.eq(value)
                yield
                for address in (0x290, 0x294):
                    self.assertEqual((yield from transfer(bus, address, 0))[1], 1)
                yield signal.eq(1 - value)
                yield
            self.assertEqual((yield from transfer(bus, 0x26c, 1))[1], 0)
            for address in (0x290, 0x294):
                self.assertEqual((yield from transfer(bus, address, 0))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x26c, 0))[1], 0)
            for address, limit in ((0x264, 0x200000), (0x220, 0x400000)):
                self.assertEqual((yield from transfer(bus, address, limit))[1], 1)
                self.assertEqual((yield from transfer(bus, address, limit, sel=4))[1], 1)
                self.assertEqual((yield from transfer(bus, address, limit - 1))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x224, 0x3ffffe))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x224, 0x1000000, sel=8))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x224)), (0x3ffffe, 0))
            self.assertEqual((yield from transfer(bus, 0x294, 0x1ffffe))[1], 0)
            self.assertEqual((yield from transfer(bus, 0x264, 0x1ffffe))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x294, 0x1fffff))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x264)), (0x1fffff, 0))

        run_simulation(dut, bench())

    def test_faulted_drain_allows_generator_disable_without_clearing_evidence(self):
        dut = PairRegisterHarness()
        bus, pair = dut.registers.bus, dut.pair

        def bench():
            self.assertEqual((yield from transfer(bus, 0x26c, 1))[1], 0)
            yield pair.fault.eq(1)
            yield pair.fault_cause.eq(0x1234)
            for camera in dut.cameras:
                yield camera.idle.eq(0)
            yield pair.busy.eq(1)
            yield
            self.assertEqual((yield from transfer(bus, 0x26c, 0))[1], 1)
            yield pair.busy.eq(0)
            yield pair.drained.eq(0)
            yield
            self.assertEqual((yield from transfer(bus, 0x26c, 0))[1], 1)
            self.assertEqual((yield pair.sync_enable), 1)
            yield pair.drained.eq(1)
            yield
            self.assertEqual((yield from transfer(bus, 0x26c, 0))[1], 0)
            self.assertEqual((yield pair.sync_enable), 0)
            self.assertEqual((yield from transfer(bus, 0x26c, 1))[1], 1)
            self.assertEqual((yield from transfer(bus, 0x26c, 1, sel=2))[1], 0)
            self.assertEqual((yield pair.sync_enable), 0)
            self.assertEqual((yield pair.fault), 1)
            self.assertEqual((yield from transfer(bus, 0x214)), (0x1234, 0))
            self.assertFalse(dut.events)

        run_simulation(dut, [bench(), command_monitor(dut)])

    def test_clear_and_reset_require_drain_but_allow_faulted_nonidle_cameras(self):
        dut = PairRegisterHarness()
        bus, pair = dut.registers.bus, dut.pair
        draining = {"enabled": False}
        capture = Capture(dut.transport.source, ready=lambda _: draining["enabled"])

        def bench():
            for index in range(6):
                yield from transfer(bus, packet_address(index, 0), 1)
                yield from transfer(bus, packet_address(index, 1), 0xc0a80101 + index)
            yield from child_group(dut.transport.channels[1], [1, 2], 42)
            yield pair.drained.eq(0)
            yield from idle(30)
            self.assertEqual((yield dut.transport.source.valid), 1)
            held_word = (yield dut.transport.source.data)
            for address in (0x210, 0x260):
                self.assertEqual((yield from transfer(bus, address, 1))[1], 1)
                self.assertEqual((yield dut.transport.source.valid), 1)
                self.assertEqual((yield dut.transport.source.data), held_word)
                self.assertEqual((yield dut.transport.channels[1].fifo_level), 1)
            self.assertFalse(dut.events)
            draining["enabled"] = True
            yield from idle(150)
            self.assertEqual(len(capture.packets), 1)
            yield pair.drained.eq(1)
            yield pair.fault.eq(1)
            for camera in dut.cameras:
                yield camera.idle.eq(0)
            yield
            self.assertEqual((yield from transfer(bus, 0x260, 1))[1], 0)
            clear = [event for event in dut.events if event[2]]
            self.assertEqual(len(clear), 1)
            self.assertEqual(clear[0][5:11], (1, 0, 0, 1, 0, 0))
            self.assertEqual(clear[0][11:], (0, 3))
            yield from idle(10)
            for index in range(6):
                self.assertEqual((yield from transfer(bus, packet_address(index, 0))), (1, 0))
                self.assertEqual((yield from transfer(bus, packet_address(index, 1))), (0xc0a80101 + index, 0))
            self.assertEqual((yield from transfer(bus, 0x210, 1))[1], 0)
            reset = [event for event in dut.events if event[3]]
            self.assertEqual(len(reset), 1)
            self.assertEqual(reset[0][5:11], (0, 1, 0, 0, 1, 0))
            self.assertEqual(reset[0][11:], (3, 0))
            yield from idle(10)
            for index in range(6):
                self.assertEqual((yield from transfer(bus, packet_address(index, 0))), (0, 0))
                self.assertEqual((yield from transfer(bus, packet_address(index, 1))), (0, 0))

        run_simulation(dut, [bench(), command_monitor(dut), capture.run()])


if __name__ == "__main__":
    unittest.main()
