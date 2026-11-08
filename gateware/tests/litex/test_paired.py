"""Host checks against an independent paired-control Wishbone register model."""

import unittest

from tools.lutcam.ov9281 import CameraError
from tools.lutcam.paired import PairError, PairedCaptureControl, quantize_phases


# These are ABI facts, deliberately independent of the helper's constants.
BASE = 0x30024200
MAGIC = 0x4c435031
DRAINED, FAULT, LOCKED, READY, SNAPSHOT_VALID = 16, 32, 64, 128, 512
DROP_SNAPSHOT_REQUEST = object()


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def __call__(self):
        return self.now

    def sleep(self, duration):
        if duration <= 0:
            raise AssertionError("polling must yield for a positive interval")
        self.sleeps.append(duration)
        self.now += duration


def coherent_snapshot(*, epoch=17, anchor=(3 << 32) - 20,
                      latency0=35, latency1=20, phase0=0, phase1=0):
    """SOFs cross a low-word rollover and have negative inter-camera skew."""
    sof0, sof1 = anchor + phase0 + latency0, anchor + phase1 + latency1
    phase_difference = phase1 - phase0
    values = {
        0x2c: epoch,
        0x30: sof0 & 0xffffffff, 0x34: sof0 >> 32,
        0x38: sof1 & 0xffffffff, 0x3c: sof1 >> 32,
        0x40: (sof1 - sof0) & 0xffffffff,
        0x44: (phase_difference - 25) & 0xffffffff,
        0x48: (phase_difference + 40) & 0xffffffff,
        0x4c: 101, 0x50: 102, 0x54: 103, 0x58: 100, 0x5c: 2,
        0x70: anchor & 0xffffffff, 0x74: anchor >> 32,
        0x78: latency0 & 0xffffffff, 0x7c: latency1 & 0xffffffff,
        0x80: 10, 0x84: 60,
        0x90: phase0, 0x94: phase1,
        0x98: (latency1 - latency0) & 0xffffffff,
        0x9c: (-25) & 0xffffffff, 0xa0: 40,
    }
    return values


class PairBus:
    """Only documented register accesses exist; writes cannot fake readiness."""

    def __init__(self, *, overrides=None, snapshot=None, snapshots=()):
        registers = {
            0x00: MAGIC, 0x04: 0, 0x08: 0,
            0x0c: DRAINED | LOCKED | READY, 0x14: 0,
            0x18: 7, 0x1c: 7, 0x24: 12_000,
            0x64: 994_812, 0x68: 1536, 0x6c: 0,
            0x88: 2, 0x8c: 900_000,
            0x90: 0, 0x94: 0,
        }
        self.registers = {BASE + offset: value for offset, value in registers.items()}
        # require_pair_idle also checks each camera's RUN and idle/fault status.
        self.registers.update({0x30024000: 0, 0x30024014: 2,
                               0x30024100: 0, 0x30024114: 2})
        self.registers.update(overrides or {})
        self.snapshot = snapshot
        self.snapshots = list(snapshots)
        self.events = []
        self.readback_overrides = {}

    def read(self, address, length=None):
        self.events.append(("read", address, length))
        if length is not None:
            if not 1 <= length <= 8:
                raise AssertionError("raw management transport supports at most eight words")
            return [self.registers[address + 4 * index] for index in range(length)]
        return self.readback_overrides.get(address, self.registers[address])

    def write(self, address, value):
        self.events.append(("write", address, value))
        if address == BASE + 0x28:
            snapshot = self.snapshots.pop(0) if self.snapshots else self.snapshot
            if snapshot is DROP_SNAPSHOT_REQUEST:
                return  # A busy latch keeps its previous valid epoch and payload.
            self.registers[BASE + 0x0c] &= ~SNAPSHOT_VALID
            if snapshot is not None:
                self.registers.update({BASE + offset: word for offset, word in snapshot.items()})
                self.registers[BASE + 0x0c] |= SNAPSHOT_VALID
        elif address in (BASE + 0x10, BASE + 0x60):
            pass  # Command strobes are not read/write storage.
        elif address in (BASE + 0x64, BASE + 0x68, BASE + 0x90, BASE + 0x94):
            if self.registers[BASE + 0x6c]:
                raise AssertionError("timing writes require a disabled generator")
            if address == BASE + 0x64:
                if not (0 < self.registers[BASE + 0x68] < value
                        and self.registers[BASE + 0x90] < value
                        and self.registers[BASE + 0x94] < value):
                    raise AssertionError("new period must contain existing width and phases")
            elif address == BASE + 0x68:
                if not 0 < value < self.registers[BASE + 0x64]:
                    raise AssertionError("pulse width must be shorter than the period")
            elif not 0 <= value < self.registers[BASE + 0x64]:
                raise AssertionError("phase must lie within the current period")
            self.registers[address] = value
        elif address in (BASE + 0x08, BASE + 0x6c):
            self.registers[address] = value
        else:
            raise AssertionError(f"undocumented paired write to {address:#x}")

    @property
    def writes(self):
        return [event for event in self.events if event[0] == "write"]


class PairedControlTests(unittest.TestCase):
    def control(self, bus):
        clock = FakeClock()
        return PairedCaptureControl(bus, clock=clock, sleep=clock.sleep), clock

    def test_identity_is_mandatory_before_every_public_mutation(self):
        operations = (
            ("reset", lambda control: control.reset()),
            ("clear", lambda control: control.clear()),
            ("configure", lambda control: control.configure_generator(120)),
            ("enable", lambda control: control.generator(True)),
            ("disable", lambda control: control.generator(False)),
            ("run", lambda control: control.run(True)),
            ("stop", lambda control: control.run(False)),
            ("snapshot", lambda control: control.snapshot()),
        )
        for register, invalid in ((BASE, 0xdeadbeef), (BASE, 0xffffffff)):
            for name, operation in operations:
                with self.subTest(register=register, operation=name):
                    bus = PairBus(overrides={register: invalid}, snapshot=coherent_snapshot())
                    control, _ = self.control(bus)
                    with self.assertRaises((PairError, CameraError)):
                        operation(control)
                    self.assertEqual(bus.writes, [])

    def test_reserved_pair_word_does_not_gate_identity_or_configuration(self):
        for reserved in (0, 1, 2, 0xffffffff):
            with self.subTest(reserved=reserved):
                bus = PairBus(overrides={BASE + 4: reserved})
                control, _ = self.control(bus)
                control.configure_generator(120)
                self.assertFalse(any(event[:2] == ("read", BASE + 4)
                                     for event in bus.events))
                self.assertTrue(bus.writes)

    def test_stopped_and_drained_required_before_configuration_or_clear(self):
        for status in (0, DRAINED | 1, DRAINED | 2, DRAINED | 4, DRAINED | 8):
            for operation in (lambda control: control.clear(),
                              lambda control: control.configure_generator(60),
                              lambda control: control.generator(True),
                              lambda control: control.generator(False)):
                with self.subTest(status=status, operation=operation):
                    bus = PairBus(overrides={BASE + 0x0c: status})
                    control, _ = self.control(bus)
                    with self.assertRaisesRegex(PairError, "disarmed, drained"):
                        operation(control)
                    self.assertEqual(bus.writes, [])
        bus = PairBus(overrides={BASE + 0x08: 1})
        control, _ = self.control(bus)
        with self.assertRaises(PairError):
            control.clear()
        self.assertEqual(bus.writes, [])

    def test_reset_accepts_stopped_drained_faults_for_recovery(self):
        bus = PairBus(overrides={BASE + 0x14: 1, BASE + 0x0c: DRAINED | FAULT,
                                 0x30024014: 2 | 4, 0x30024114: 2 | 4})
        control, _ = self.control(bus)
        control.reset()
        self.assertEqual(bus.writes, [("write", BASE + 0x10, 1)])
        self.assertEqual(bus.events[-1], ("read", BASE + 0x08, None))

    def test_idle_guard_still_checks_both_camera_paths_and_faults(self):
        for overrides in ({0x30024000: 1}, {0x30024114: 1},
                          {BASE + 0x14: 1}, {BASE + 0x0c: DRAINED | FAULT}):
            with self.subTest(overrides=overrides):
                bus = PairBus(overrides=overrides)
                control, _ = self.control(bus)
                with self.assertRaises(CameraError):
                    control.require_idle()
                self.assertEqual(bus.writes, [])

    def test_reset_refuses_busy_or_undrained_acquisition(self):
        for overrides in ({BASE + 0x08: 1}, {BASE + 0x0c: 0},
                          *({BASE + 0x0c: DRAINED | FAULT | bit} for bit in (1, 2, 4, 8))):
            with self.subTest(overrides=overrides):
                bus = PairBus(overrides=overrides)
                control, _ = self.control(bus)
                with self.assertRaisesRegex(PairError, "disarmed, drained"):
                    control.reset()
                self.assertEqual(bus.writes, [])

    def test_reset_and_clear_have_ordered_completion_reads(self):
        for operation, offset in (("reset", 0x10), ("clear", 0x60)):
            with self.subTest(operation=operation):
                bus = PairBus()
                control, _ = self.control(bus)
                getattr(control, operation)()
                self.assertEqual(bus.events[-2:], [("write", BASE + offset, 1),
                                                  ("read", BASE + 0x08, None)])

    def test_generator_presets_write_exact_period_and_pulse_width(self):
        for fps, period in ((60, 1_990_716), (120, 994_812)):
            with self.subTest(fps=fps):
                bus = PairBus()
                control, _ = self.control(bus)
                result = control.configure_generator(fps)
                self.assertEqual(bus.writes, [("write", BASE + 0x90, 0),
                                             ("write", BASE + 0x94, 0),
                                             ("write", BASE + 0x64, period),
                                             ("write", BASE + 0x68, 1536),
                                             ("write", BASE + 0x90, 0),
                                             ("write", BASE + 0x94, 0)])
                self.assertEqual(result["fps_preset"], fps)
                self.assertEqual(result["sys_hz"], 120_000_000)
                self.assertEqual(result["period_cycles"], period)
                self.assertEqual(result["width_cycles"], 1536)
                self.assertEqual(result["min_latency_limit_cycles"], 2)
                self.assertEqual(result["max_latency_limit_cycles"], 900_000)
                self.assertAlmostEqual(result["derived_trigger_rate_hz"], 120_000_000 / period)
                self.assertEqual(result["phases"], [
                    {"requested_us": 0, "cycles": 0, "actual_us": 0},
                    {"requested_us": 0, "cycles": 0, "actual_us": 0},
                ])
                for index, event in enumerate(bus.events):
                    if event[0] == "write":
                        self.assertEqual(bus.events[index + 1], ("read", event[1], None))

    def test_generator_parks_old_phases_before_shortening_period(self):
        bus = PairBus(overrides={BASE + 0x64: 1_990_716,
                                 BASE + 0x90: 1_500_000, BASE + 0x94: 1_750_000})
        control, _ = self.control(bus)
        result = control.configure_generator(120, phases_us=(1.25, 4_145.05))
        self.assertEqual(bus.writes, [
            ("write", BASE + 0x90, 0), ("write", BASE + 0x94, 0),
            ("write", BASE + 0x64, 994_812), ("write", BASE + 0x68, 1536),
            ("write", BASE + 0x90, 150), ("write", BASE + 0x94, 497_406),
        ])
        self.assertEqual([phase["cycles"] for phase in result["phases"]], [150, 497_406])

    def test_unsupported_generator_interval_fails_before_bus_access(self):
        for fps in (0, 30, 59, 61, 119, 121, 240):
            with self.subTest(fps=fps):
                bus = PairBus()
                control, _ = self.control(bus)
                with self.assertRaises(ValueError):
                    control.configure_generator(fps)
                self.assertEqual(bus.events, [])

    def test_live_generator_timing_cannot_be_changed(self):
        bus = PairBus(overrides={BASE + 0x6c: 1})
        control, _ = self.control(bus)
        with self.assertRaisesRegex(PairError, "disabled"):
            control.configure_generator(60)
        self.assertEqual(bus.writes, [])

    def test_latency_interval_must_fit_the_requested_generator_period(self):
        for fps, minimum, maximum in (
            (60, 0, 900_000), (120, 2, 2), (120, 10, 5),
            (60, 2, 1_990_716), (120, 2, 994_812),
            (120, 2, 1_000_000),
        ):
            with self.subTest(fps=fps, minimum=minimum, maximum=maximum):
                bus = PairBus(overrides={BASE + 0x88: minimum, BASE + 0x8c: maximum})
                control, _ = self.control(bus)
                with self.assertRaises(PairError):
                    control.configure_generator(fps)
                self.assertEqual(bus.writes, [])
        # The same interval is valid for the longer requested 60 fps period,
        # even though the old period register still contains the 120 fps value.
        bus = PairBus(overrides={BASE + 0x88: 2, BASE + 0x8c: 1_000_000})
        control, _ = self.control(bus)
        self.assertEqual(control.configure_generator(60)["max_latency_limit_cycles"], 1_000_000)

    def test_existing_width_must_fit_requested_period_before_any_timing_write(self):
        for width in (0, 994_812, 1_000_000, 1_990_716):
            with self.subTest(width=width):
                bus = PairBus(overrides={BASE + 0x64: 1_990_716, BASE + 0x68: width})
                control, _ = self.control(bus)
                with self.assertRaisesRegex(PairError, "existing FSYNC width"):
                    control.configure_generator(120)
                self.assertEqual(bus.writes, [])

    def test_quantize_phases_rejects_nonfinite_negative_boolean_and_period_offsets(self):
        for fps, period in ((60, 1_990_716), (120, 994_812)):
            for invalid in (float("nan"), float("inf"), float("-inf"), -0.001,
                            True, False, period / 120, (period + 1) / 120):
                for phases in ((invalid, 0), (0, invalid)):
                    with self.subTest(fps=fps, phases=phases):
                        bus = PairBus()
                        control, _ = self.control(bus)
                        with self.assertRaises(ValueError):
                            quantize_phases(fps, phases)
                        with self.assertRaises(ValueError):
                            control.configure_generator(fps, phases)
                        self.assertEqual(bus.events, [])

    def test_quantize_phases_requires_supported_preset_and_exactly_two_offsets(self):
        for fps, phases in ((30, (0, 0)), (60, ()), (120, (0,)), (120, (0, 0, 0))):
            with self.subTest(fps=fps, phases=phases):
                with self.assertRaises(ValueError):
                    quantize_phases(fps, phases)

    def test_quantize_phases_reports_actual_time_and_uses_nearest_even_rounding(self):
        for requested, cycles in ((0, 0), (1 / 120, 1), (0.0125, 2),
                                  (0.0375, 4), (1.251, 150)):
            with self.subTest(requested=requested):
                phase = quantize_phases(120, (requested, 0))[0]
                self.assertEqual(phase["requested_us"], requested)
                self.assertEqual(phase["cycles"], cycles)
                self.assertAlmostEqual(phase["actual_us"], cycles / 120)
        for fps, period in ((60, 1_990_716), (120, 994_812)):
            with self.subTest(fps=fps):
                self.assertEqual(quantize_phases(fps, ((period - 0.51) / 120, 0))[0]["cycles"],
                                 period - 1)
                with self.assertRaisesRegex(ValueError, "rounds outside"):
                    quantize_phases(fps, ((period - 0.49) / 120, 0))

    def test_generator_enable_and_disable_are_verified(self):
        bus = PairBus()
        control, _ = self.control(bus)
        for enabled in (True, False):
            control.generator(enabled)
            self.assertEqual(bus.events[-2:], [("write", BASE + 0x6c, int(enabled)),
                                               ("read", BASE + 0x6c, None)])
        bus.readback_overrides[BASE + 0x6c] = 0
        with self.assertRaisesRegex(PairError, "readback mismatch"):
            control.generator(True)

    def test_arm_rejects_missing_lock_readiness_fault_or_generator(self):
        for overrides in (
            {BASE + 0x0c: DRAINED | READY, BASE + 0x6c: 1},
            {BASE + 0x0c: DRAINED | LOCKED, BASE + 0x6c: 1},
            {BASE + 0x0c: DRAINED | LOCKED | READY | FAULT, BASE + 0x6c: 1},
            {BASE + 0x14: 1, BASE + 0x6c: 1},
            {BASE + 0x6c: 0},
        ):
            with self.subTest(overrides=overrides):
                bus = PairBus(overrides=overrides)
                control, _ = self.control(bus)
                with self.assertRaisesRegex(PairError, "ready, locked and fault-free"):
                    control.run(True)
                self.assertEqual(bus.writes, [])

    def test_arm_and_stop_use_one_verified_shared_run_register(self):
        bus = PairBus(overrides={BASE + 0x6c: 1})
        control, _ = self.control(bus)
        control.run(True)
        bus.registers[BASE + 0x0c] = 1 | 4 | READY | LOCKED
        control.run(False)
        self.assertEqual(bus.writes, [("write", BASE + 0x08, 1),
                                     ("write", BASE + 0x08, 0)])
        self.assertEqual(bus.events[-1], ("read", BASE + 0x08, None))

    def test_snapshot_reassembles_64_bit_rollover_and_signed_skew(self):
        bus = PairBus(snapshot=coherent_snapshot())
        control, _ = self.control(bus)
        result = control.snapshot()
        self.assertEqual(result["epoch"], 17)
        self.assertEqual(result["anchor_cycles"], (3 << 32) - 20)
        self.assertEqual(result["sof0_cycles"], (3 << 32) + 15)
        self.assertEqual(result["sof1_cycles"], 3 << 32)
        self.assertEqual(result["last_skew_cycles"], -15)
        self.assertEqual(result["min_skew_cycles"], -25)
        self.assertEqual(result["max_skew_cycles"], 40)
        self.assertEqual((result["phase0_cycles"], result["phase1_cycles"]), (0, 0))
        self.assertEqual(result["residual_skew_cycles"], -15)
        self.assertEqual(result["min_residual_skew_cycles"], -25)
        self.assertEqual(result["max_residual_skew_cycles"], 40)
        self.assertEqual(result["sof0_latency_cycles"], 35)
        self.assertEqual(result["sof1_latency_cycles"], 20)
        self.assertEqual(result["min_latency_cycles"], 10)
        self.assertEqual(result["max_latency_cycles"], 60)
        self.assertEqual([result[key] for key in ("raw_sof0", "raw_sof1", "vsync",
                                                "matched", "missing")],
                         [101, 102, 103, 100, 2])
        self.assertEqual((result["pairs_started"], result["pairs_completed"]), (7, 7))
        self.assertEqual(result["min_latency_limit_cycles"], 2)
        self.assertEqual(result["max_latency_limit_cycles"], 900_000)
        self.assertEqual(bus.writes, [("write", BASE + 0x28, 1)])

    def test_snapshot_rejects_incoherent_sof_or_anchor_evidence(self):
        for offset in (0x40, 0x78, 0x7c):
            with self.subTest(offset=offset):
                snapshot = coherent_snapshot()
                snapshot[offset] += 1
                bus = PairBus(snapshot=snapshot)
                control, _ = self.control(bus)
                with self.assertRaisesRegex(PairError, "disagree"):
                    control.snapshot()

    def test_half_period_phase_difference_has_large_raw_skew_and_small_residual(self):
        for phase0, phase1 in ((0, 497_406), (497_406, 0)):
            with self.subTest(phase0=phase0, phase1=phase1):
                anchor = (3 << 32) - 20
                bus = PairBus(snapshot=coherent_snapshot(anchor=anchor, phase0=phase0,
                                                         phase1=phase1))
                control, _ = self.control(bus)
                result = control.snapshot()
                phase_difference = phase1 - phase0
                self.assertEqual(result["last_skew_cycles"], phase_difference - 15)
                self.assertGreater(abs(result["last_skew_cycles"]), 497_000)
                self.assertEqual(result["min_skew_cycles"], phase_difference - 25)
                self.assertEqual(result["max_skew_cycles"], phase_difference + 40)
                self.assertEqual(result["residual_skew_cycles"], -15)
                self.assertEqual(result["min_residual_skew_cycles"], -25)
                self.assertEqual(result["max_residual_skew_cycles"], 40)
                self.assertEqual(result["sof0_cycles"], anchor + phase0 + 35)
                self.assertEqual(result["sof1_cycles"], anchor + phase1 + 20)
                self.assertEqual(result["sof0_latency_cycles"], 35)
                self.assertEqual(result["sof1_latency_cycles"], 20)
                self.assertEqual((result["phase0_cycles"], result["phase1_cycles"]),
                                 (phase0, phase1))

    def test_snapshot_rejects_inconsistent_phase_corrected_residual(self):
        snapshot = coherent_snapshot(phase0=497_406, phase1=0)
        snapshot[0x98] = (-14) & 0xffffffff
        bus = PairBus(snapshot=snapshot)
        control, _ = self.control(bus)
        with self.assertRaisesRegex(PairError, "phase-corrected skew disagrees"):
            control.snapshot()

    def test_snapshot_preserves_negative_latency_diagnostics(self):
        snapshot = coherent_snapshot(anchor=5 << 32, latency0=-1, latency1=2)
        snapshot.update({0x80: (-3) & 0xffffffff, 0x84: 5})
        bus = PairBus(snapshot=snapshot)
        control, _ = self.control(bus)
        result = control.snapshot()
        self.assertEqual(result["sof0_cycles"], (5 << 32) - 1)
        self.assertEqual(result["sof0_latency_cycles"], -1)
        self.assertEqual(result["sof1_latency_cycles"], 2)
        self.assertEqual(result["last_skew_cycles"], 3)
        self.assertEqual(result["min_latency_cycles"], -3)
        self.assertEqual(result["max_latency_cycles"], 5)

    def test_snapshot_waits_for_valid_evidence_then_reads_one_latch(self):
        bus = PairBus(snapshots=[None, None, coherent_snapshot()])
        control, clock = self.control(bus)
        self.assertEqual(control.snapshot(timeout=0.01)["epoch"], 17)
        self.assertEqual(clock.sleeps, [0.002, 0.002])
        self.assertEqual(bus.writes, [("write", BASE + 0x28, 1)] * 3)

    def test_snapshot_freshness_retries_equal_older_and_half_range_epochs(self):
        bus = PairBus(snapshots=[coherent_snapshot(epoch=17), coherent_snapshot(epoch=16),
                                 coherent_snapshot(epoch=17 + 0x80000000),
                                 coherent_snapshot(epoch=18)])
        control, clock = self.control(bus)
        result = control.snapshot(timeout=0.01, after_epoch=17)
        self.assertEqual(result["epoch"], 18)
        self.assertEqual(clock.sleeps, [0.002] * 3)
        self.assertEqual(bus.writes, [("write", BASE + 0x28, 1)] * 4)
        bulk_reads = [event for event in bus.events if event[0] == "read" and event[2] is not None]
        self.assertEqual(bulk_reads, [("read", BASE + 0x2c, 8), ("read", BASE + 0x4c, 5),
                                      ("read", BASE + 0x70, 6), ("read", BASE + 0x98, 3)])
        last_request = max(index for index, event in enumerate(bus.events) if event[0] == "write")
        self.assertTrue(all(index > last_request for index, event in enumerate(bus.events)
                            if event[0] == "read" and event[2] is not None))

    def test_stale_valid_snapshot_times_out_without_reading_stale_payload(self):
        bus = PairBus(snapshot=coherent_snapshot(epoch=17))
        control, clock = self.control(bus)
        with self.assertRaisesRegex(TimeoutError, "no fresh paired SOF"):
            control.snapshot(timeout=0.005, after_epoch=17)
        self.assertGreaterEqual(clock.now, 0.005)
        self.assertLessEqual(clock.now, 0.007)
        self.assertEqual(len(bus.writes), 4)
        self.assertFalse(any(event[2] is not None for event in bus.events if event[0] == "read"))

    def test_snapshot_retries_dropped_request_while_old_latch_remains_valid(self):
        bus = PairBus(snapshot=coherent_snapshot(epoch=17))
        control, clock = self.control(bus)
        self.assertEqual(control.snapshot()["epoch"], 17)
        bus.events.clear()
        bus.snapshots = [DROP_SNAPSHOT_REQUEST, coherent_snapshot(epoch=18)]
        self.assertEqual(control.snapshot(after_epoch=17)["epoch"], 18)
        self.assertEqual(clock.sleeps, [0.002])
        self.assertEqual(bus.writes, [("write", BASE + 0x28, 1)] * 2)
        last_request = max(index for index, event in enumerate(bus.events) if event[0] == "write")
        self.assertTrue(all(index > last_request for index, event in enumerate(bus.events)
                            if event[0] == "read" and event[2] is not None))

    def test_snapshot_freshness_accepts_uint32_epoch_wrap(self):
        for previous, current in ((0xfffffffe, 0xffffffff), (0xffffffff, 0),
                                  (0xfffffffe, 1)):
            with self.subTest(previous=previous, current=current):
                bus = PairBus(snapshot=coherent_snapshot(epoch=current))
                control, clock = self.control(bus)
                self.assertEqual(control.snapshot(after_epoch=previous)["epoch"], current)
                self.assertEqual(clock.sleeps, [])

    def test_snapshot_without_freshness_accepts_repeated_epoch(self):
        bus = PairBus(snapshot=coherent_snapshot(epoch=17))
        control, clock = self.control(bus)
        self.assertEqual(control.snapshot()["epoch"], 17)
        self.assertEqual(control.snapshot()["epoch"], 17)
        self.assertEqual(clock.sleeps, [])
        self.assertEqual(bus.writes, [("write", BASE + 0x28, 1)] * 2)

    def test_snapshot_rejects_invalid_freshness_epoch_before_bus_access(self):
        for epoch in (True, False, -1, 1 << 32, 17.0, "17"):
            with self.subTest(epoch=epoch):
                bus = PairBus(snapshot=coherent_snapshot())
                control, _ = self.control(bus)
                with self.assertRaisesRegex(ValueError, "uint32 epoch"):
                    control.snapshot(after_epoch=epoch)
                self.assertEqual(bus.events, [])

    def test_no_valid_snapshot_times_out_with_bounded_polling(self):
        bus = PairBus(snapshot=None)
        control, clock = self.control(bus)
        with self.assertRaisesRegex(TimeoutError, "no valid paired SOF"):
            control.snapshot(timeout=0.005)
        self.assertGreaterEqual(clock.now, 0.005)
        self.assertLessEqual(clock.now, 0.007)
        self.assertEqual(len(bus.writes), 4)
        self.assertFalse(any(event[2] is not None for event in bus.events if event[0] == "read"))


if __name__ == "__main__":
    unittest.main()
