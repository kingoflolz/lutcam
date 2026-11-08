"""Shared trigger, admission, stop, and synchronization evidence regressions.

The controller sees already decoded CSI start-of-frame events. These tests
exercise that contract without pretending to model sensor exposure or D-PHY.
"""

import re
import shutil
import subprocess

import pytest
from migen import ClockDomain, Module
from migen.fhdl import verilog
from migen.sim import run_simulation

from fpga.litex.lutcam.pair import PairedCaptureControl


_STATE = (
    "grants", "busy", "drained", "abort", "fault", "locked", "ready",
    "run_request", "pairs_started", "pairs_completed", "status", "fault_cause",
)


def _cycles(count):
    for _ in range(count):
        yield


def _pulse(signal):
    yield signal.eq(1)
    yield
    yield signal.eq(0)
    yield


def _configure(dut, *, enable=True, phases=(0, 0)):
    for name, value in (
        ("sync_period", 100), ("sync_width", 5),
        ("min_latency_cycles", 2), ("max_latency_cycles", 80),
        ("watchdog_cycles", 200), ("max_skew_cycles", 10),
        ("health_good", 1), ("transport_drained", 1),
        ("pipeline_ready", 3), ("sync_enable", 0),
    ):
        yield getattr(dut, name).eq(value)
    yield dut.phase0_cycles.eq(phases[0])
    yield dut.phase1_cycles.eq(phases[1])
    yield from _cycles(2)
    # Leave the first enabled edge for the caller to observe and answer.
    yield dut.sync_enable.eq(enable)


def _wait_sync(dut):
    # Always wait for a new physical output edge, even if invoked during the
    # pulse from the preceding epoch.
    for _ in range(220):
        if not (yield dut.sync_output):
            break
        yield
    else:
        raise AssertionError("FSYNC stayed high")
    for _ in range(220):
        yield
        if (yield dut.sync_output):
            return
    raise AssertionError("FSYNC did not rise")


def _epoch(dut, *, delays=(10, 12), events=None, length=45):
    """Return observations after driving one response to a generated trigger.

Migen processes the new inputs on the following clock edge, so assertions
allow one sample after each driven event before observing registered state.
"""
    yield from _wait_sync(dut)
    samples = []
    for age in range(length):
        sof = sum(1 << index for index, delay in enumerate(delays) if delay == age)
        yield dut.sof.eq(sof)
        for name, value in (events or {}).get(age, {}).items():
            yield getattr(dut, name).eq(value)
        yield
        sample = {}
        for name in _STATE:
            sample[name] = (yield getattr(dut, name))
        samples.append(sample)
    yield dut.sof.eq(0)
    yield
    return samples


def _warm(dut):
    yield from _epoch(dut)
    assert (yield dut.locked) == 0
    yield from _epoch(dut)
    assert (yield dut.locked) == 1
    assert (yield dut.busy) == 0
    assert (yield dut.pairs_started) == 0


def _snapshot(dut):
    yield from _pulse(dut.snapshot)
    yield
    result = {}
    for name in (
        "snap_epoch", "snap_sync", "snap_sof0", "snap_sof1", "snap_skew",
        "snap_min_skew", "snap_max_skew", "snap_latency0", "snap_latency1",
        "snap_residual_skew", "snap_min_residual_skew", "snap_max_residual_skew",
        "snap_min_latency", "snap_max_latency", "snap_sof0_count",
        "snap_sof1_count", "snap_sync_count", "snap_matched_pairs",
        "snap_missing_events",
    ):
        result[name] = (yield getattr(dut, name))
    return result


def test_generator_period_width_and_disabled_output():
    dut = PairedCaptureControl()
    assert dut.sync_period.reset.value == 994812
    assert dut.sync_width.reset.value == 1536
    assert dut.sync_enable.reset.value == 0
    assert dut.master_sync is dut.sync_output

    def bench():
        yield from _configure(dut, enable=False)
        for _ in range(12):
            assert (yield dut.sync_output) == 0
            yield
        yield dut.sync_enable.eq(1)
        levels = []
        for _ in range(310):
            yield
            levels.append((yield dut.sync_output))
        rises = [index for index in range(1, len(levels))
                 if levels[index] and not levels[index - 1]]
        assert len(rises) >= 3
        assert all(second - first == 100 for first, second in zip(rises, rises[1:]))
        assert all(levels[index:index + 6] == [1, 1, 1, 1, 1, 0]
                   for index in rises if index + 6 <= len(levels))
        yield dut.sync_enable.eq(0)
        yield from _cycles(3)
        assert (yield dut.sync_output) == 0

    run_simulation(dut, bench())


def test_warmup_requires_two_epochs_and_snapshot_is_coherent():
    dut = PairedCaptureControl()

    def bench():
        yield from _configure(dut)
        yield from _warm(dut)
        first = yield from _snapshot(dut)
        assert first["snap_matched_pairs"] == 2
        assert first["snap_sof0_count"] == first["snap_sof1_count"] == 2
        assert first["snap_sof1"] - first["snap_sof0"] == first["snap_skew"] == 2
        assert first["snap_latency0"] == first["snap_sof0"] - first["snap_sync"]
        assert first["snap_latency1"] == first["snap_sof1"] - first["snap_sync"]
        assert 2 <= first["snap_latency0"] < first["snap_latency1"] <= 80
        assert first["snap_min_skew"] == first["snap_max_skew"] == 2
        assert first["snap_min_latency"] == first["snap_latency0"]
        assert first["snap_max_latency"] == first["snap_latency1"]
        yield from _epoch(dut, delays=(16, 13))
        assert (yield dut.snap_epoch) == first["snap_epoch"]
        assert (yield dut.snap_sof0) == first["snap_sof0"]
        second = yield from _snapshot(dut)
        assert second["snap_matched_pairs"] == 3
        assert second["snap_epoch"] == first["snap_epoch"] + 1
        assert second["snap_skew"] == -3
        assert second["snap_min_skew"] == -3
        assert second["snap_max_skew"] == 2
        assert second["snap_min_latency"] == first["snap_min_latency"]
        assert second["snap_max_latency"] == second["snap_latency0"]

    run_simulation(dut, bench())


def test_stop_between_camera_admissions_completes_pair_and_transport_drain():
    dut = PairedCaptureControl()

    def bench():
        yield from _configure(dut)
        yield from _warm(dut)
        yield from _pulse(dut.start)
        assert (yield dut.run_request) == 1
        trace = yield from _epoch(dut, events={
            8: {"transport_drained": 0},
            10: {"admitted": 1, "active": 1},
            11: {"admitted": 0, "stop": 1},
            12: {"stop": 0, "admitted": 2, "active": 3},
            13: {"admitted": 0},
            30: {"retired": 1, "active": 2},
            31: {"retired": 0},
            32: {"retired": 2, "active": 0},
            33: {"retired": 0},
            40: {"transport_drained": 1},
        })
        assert trace[2]["grants"] == 3
        assert trace[12]["run_request"] == 0
        assert trace[12]["grants"] == 2
        assert trace[14]["grants"] == 0
        assert trace[34]["pairs_completed"] == 1
        assert trace[35]["busy"] == 1
        assert trace[35]["drained"] == 0
        assert trace[44]["busy"] == 0
        assert trace[44]["drained"] == 1
        assert (yield dut.pairs_started) == (yield dut.pairs_completed) == 1
        assert (yield dut.fault) == 0
        yield from _epoch(dut)
        assert (yield dut.grants) == 0
        assert (yield dut.pairs_started) == 1

    run_simulation(dut, bench())


@pytest.mark.parametrize("field,mask,cause", [
    ("local_fault", 1, 1 << 0), ("local_fault", 2, 1 << 1),
    ("packet_fault", 1, 1 << 2), ("receiver_fault", 2, 1 << 5),
    ("health_good", 0, 1 << 11),
])
def test_either_camera_or_shared_health_fault_revokes_both_grants(field, mask, cause):
    dut = PairedCaptureControl()

    def bench():
        yield from _configure(dut)
        yield from _warm(dut)
        yield from _pulse(dut.start)
        trace = yield from _epoch(dut, events={5: {field: mask}})
        assert trace[2]["grants"] == 3
        assert trace[8]["abort"] == trace[8]["fault"] == 1
        assert trace[8]["grants"] == trace[8]["run_request"] == 0
        assert trace[8]["fault_cause"] & cause
        assert trace[-1]["busy"] == 0
        assert trace[-1]["fault"] == 1
        assert trace[-1]["status"] & 0x0f == 0
        assert trace[-1]["drained"] == 1
        assert trace[-1]["pairs_started"] == 1
        assert (yield dut.pairs_completed) == 0
        yield getattr(dut, field).eq(1 if field == "health_good" else 0)
        yield from _cycles(3)
        assert (yield dut.fault) == 1
        yield from _pulse(dut.clear)
        assert (yield dut.fault) == (yield dut.abort) == 0
        assert (yield dut.pairs_started) == (yield dut.pairs_completed) == 0

    run_simulation(dut, bench())


@pytest.mark.parametrize("delays", [(10, None), (10, 25), (0, 2), (82, 84)])
def test_missing_skewed_early_or_late_sensor_cannot_establish_lock(delays):
    dut = PairedCaptureControl()

    def bench():
        yield from _configure(dut)
        if delays == (0, 2):
            # The bench reacts to an observed output edge two simulator
            # scheduling steps later; place that response below this bound.
            yield dut.min_latency_cycles.eq(5)
        for _ in range(3):
            yield from _epoch(dut, delays=delays, length=90)
            assert (yield dut.locked) == 0
        snap = yield from _snapshot(dut)
        assert snap["snap_matched_pairs"] == 0
        assert snap["snap_missing_events"] > 0
        assert (yield dut.fault) == 0  # Warmup failure has no active frame to abort.
        yield from _pulse(dut.start)
        assert (yield dut.run_request) == 0

    run_simulation(dut, bench())


def test_duplicate_sof_and_missing_epoch_break_existing_lock():
    dut = PairedCaptureControl()

    def bench():
        yield from _configure(dut)
        yield from _warm(dut)
        yield from _epoch(dut, events={20: {"sof": 1}})
        assert (yield dut.locked) == 0
        first = yield from _snapshot(dut)
        assert first["snap_missing_events"] >= 1
        yield from _epoch(dut)
        assert (yield dut.locked) == 0
        yield from _epoch(dut)
        assert (yield dut.locked) == 1
        yield from _epoch(dut, delays=(None, None), length=90)
        yield from _epoch(dut, delays=(None, None), length=10)
        assert (yield dut.locked) == 0
        second = yield from _snapshot(dut)
        assert second["snap_missing_events"] > first["snap_missing_events"]

    run_simulation(dut, bench())


def test_start_requires_generator_lock_and_ready_and_reset_rewarms():
    dut = PairedCaptureControl()

    def bench():
        yield from _configure(dut, enable=False)
        yield from _pulse(dut.start)
        assert (yield dut.run_request) == 0
        yield dut.sync_enable.eq(1)
        yield from _pulse(dut.start)
        assert (yield dut.run_request) == 0
        # The rejected START deliberately left the first trigger unanswered.
        # Reset its raw association before measuring two complete warm epochs.
        yield from _pulse(dut.reset)
        yield from _warm(dut)
        yield dut.pipeline_ready.eq(1)
        yield from _pulse(dut.start)
        assert (yield dut.run_request) == 0
        yield dut.pipeline_ready.eq(3)
        yield from _pulse(dut.clear)
        assert (yield dut.locked) == 1
        assert (yield dut.sync_enable) == 1
        snap = yield from _snapshot(dut)
        assert snap["snap_matched_pairs"] == 0
        assert snap["snap_sof0_count"] == snap["snap_sof1_count"] == 0
        assert not ((yield dut.status) & (1 << 9))
        yield from _pulse(dut.reset)
        assert (yield dut.locked) == 0
        assert (yield dut.sync_enable) == 1
        yield from _warm(dut)
        yield from _pulse(dut.start)
        assert (yield dut.run_request) == 1

    run_simulation(dut, bench())


@pytest.mark.parametrize("delays", [(0, 2), (82, 84)])
def test_rejected_latency_still_publishes_diagnostic_timestamps(delays):
    dut = PairedCaptureControl()

    def bench():
        yield from _configure(dut)
        yield dut.min_latency_cycles.eq(5)
        yield from _epoch(dut, delays=delays, length=90)
        evidence = yield from _snapshot(dut)
        assert (yield dut.locked) == 0
        assert (yield dut.status) & (1 << 9)
        assert evidence["snap_matched_pairs"] == 0
        assert evidence["snap_missing_events"] > 0
        assert evidence["snap_sof1"] - evidence["snap_sof0"] == 2
        assert evidence["snap_latency0"] == evidence["snap_sof0"] - evidence["snap_sync"]
        assert evidence["snap_latency1"] == evidence["snap_sof1"] - evidence["snap_sync"]
        if delays[0] == 0:
            assert evidence["snap_latency0"] < 5
        else:
            assert evidence["snap_latency0"] > 80

    run_simulation(dut, bench())


class _SlaveSensors:
    """Independent fixed-latency SOFs and finite per-camera frame lifetimes."""

    def __init__(self, dut, *, latencies=(10, 12), frame_cycles=20):
        self.dut = dut
        self.latencies = latencies
        self.frame_cycles = frame_cycles
        self.cycle = 0
        self.previous_outputs = 0
        self.sof_events = {}
        self.retire_events = {}
        self.active = 0
        self.pulses = [[], []]
        self.admissions = [[], []]
        self.retirements = [[], []]
        self.trace = []

    def run(self, cycles, *, actions=None):
        dut = self.dut
        for offset in range(cycles):
            for command in (dut.start, dut.stop, dut.clear, dut.reset, dut.snapshot):
                yield command.eq(0)
            for name, value in (actions or {}).get(offset, {}).items():
                yield getattr(dut, name).eq(value)
            levels = yield dut.sync_outputs
            rises = levels & ~self.previous_outputs
            self.previous_outputs = levels
            for camera in range(2):
                if rises & (1 << camera):
                    self.pulses[camera].append(self.cycle)
                    when = self.cycle + self.latencies[camera]
                    self.sof_events.setdefault(when, []).append(camera)
            sof, admitted, retired = 0, 0, 0
            for camera in self.sof_events.pop(self.cycle, []):
                bit = 1 << camera
                sof |= bit
                if (yield dut.grants) & bit:
                    assert not self.active & bit, "Per-camera frames overlap"
                    frame_id = yield dut.pending_frame_id[camera]
                    self.admissions[camera].append((frame_id, self.cycle))
                    admitted |= bit
                    self.active |= bit
                    when = self.cycle + self.frame_cycles
                    self.retire_events.setdefault(when, []).append((camera, frame_id))
            for camera, frame_id in self.retire_events.pop(self.cycle, []):
                retired |= 1 << camera
                self.active &= ~(1 << camera)
                self.retirements[camera].append((frame_id, self.cycle))
            yield dut.sof.eq(sof)
            yield dut.admitted.eq(admitted)
            yield dut.retired.eq(retired)
            yield dut.active.eq(self.active)
            sample = {"cycle": self.cycle, "outputs": levels}
            for name in _STATE:
                sample[name] = yield getattr(dut, name)
            self.trace.append(sample)
            self.cycle += 1
            yield


@pytest.mark.parametrize("phases", [(0, 0), (0, 50), (50, 0), (99, 0), (0, 99), (99, 99)])
def test_independent_phase_outputs_and_same_epoch_timing_snapshot(phases):
    dut = PairedCaptureControl()
    sensors = _SlaveSensors(dut)

    def bench():
        yield from _configure(dut, phases=phases)
        yield from sensors.run(425)
        assert (yield dut.locked) == 1
        assert (yield dut.pairs_started) == 0
        # Discard the intentionally unsynchronized first simulator sample;
        # subsequent events measure the physical output edges exactly.
        for camera in range(2):
            edges = sensors.pulses[camera]
            assert len(edges) >= 4
            assert all(right - left == 100 for left, right in zip(edges[1:], edges[2:]))
        assert sensors.pulses[1][2] - sensors.pulses[0][2] == phases[1] - phases[0]
        yield from sensors.run(280, actions={0: {"clear": 1}})
        assert (yield dut.locked) == 1
        yield from sensors.run(3, actions={0: {"snapshot": 1}})
        anchor = yield dut.snap_sync
        sof0, sof1 = (yield dut.snap_sof0), (yield dut.snap_sof1)
        latency0, latency1 = (yield dut.snap_latency0), (yield dut.snap_latency1)
        assert latency0 == sof0 - anchor - phases[0]
        assert latency1 == sof1 - anchor - phases[1]
        assert latency1 - latency0 == 2
        assert (yield dut.snap_skew) == sof1 - sof0 == phases[1] - phases[0] + 2
        assert (yield dut.snap_residual_skew) == 2
        assert (yield dut.snap_min_residual_skew) == 2
        assert (yield dut.snap_max_residual_skew) == 2
        assert (yield dut.snap_missing_events) == 0
        assert (yield dut.fault) == 0

    run_simulation(dut, bench())


def test_phased_epochs_overlap_with_shared_ids_and_stop_preserves_late_peer():
    dut = PairedCaptureControl()
    sensors = _SlaveSensors(dut)

    def bench():
        yield from _configure(dut, phases=(0, 99))
        yield from sensors.run(425)
        assert (yield dut.locked) == 1
        yield from sensors.run(300, actions={0: {"start": 1}})
        started = yield dut.pairs_started
        assert started >= 2
        assert len(sensors.admissions[0]) > len(sensors.admissions[1])
        stop_cycle = sensors.cycle
        yield from sensors.run(220, actions={0: {"stop": 1}})
        assert (yield dut.busy) == 0
        assert (yield dut.drained) == 1
        assert (yield dut.pairs_started) == (yield dut.pairs_completed) == started
        expected = list(range(started))
        assert [frame for frame, _cycle in sensors.admissions[0]] == expected
        assert [frame for frame, _cycle in sensors.admissions[1]] == expected
        assert sensors.admissions[0][1][1] < sensors.admissions[1][0][1]
        assert sensors.admissions[1][-1][1] > stop_cycle
        assert (yield dut.fault) == 0

    run_simulation(dut, bench())


def test_stop_before_first_phase_still_completes_both_issued_members():
    dut = PairedCaptureControl()
    sensors = _SlaveSensors(dut)

    def bench():
        yield from _configure(dut, phases=(50, 99))
        yield from sensors.run(425)
        assert (yield dut.locked) == 1
        yield from sensors.run(2, actions={0: {"start": 1}})
        for _ in range(110):
            if (yield dut.pairs_started):
                break
            yield from sensors.run(1)
        else:
            raise AssertionError("Controller did not issue a base epoch")
        assert sensors.admissions == [[], []]
        stop_cycle = sensors.cycle
        yield from sensors.run(220, actions={0: {"stop": 1}})
        assert (yield dut.pairs_started) == (yield dut.pairs_completed) == 1
        assert (yield dut.busy) == 0
        assert (yield dut.fault) == 0
        for admissions in sensors.admissions:
            assert len(admissions) == 1
            assert admissions[0][0] == 0
            assert admissions[0][1] > stop_cycle

    run_simulation(dut, bench())


class _PairPhaseRTLHarness(Module):
    def __init__(self):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.submodules.pair = pair = PairedCaptureControl()
        self.ios = {self.cd_sys.clk, self.cd_sys.rst}
        fields = (
            "sync_period", "sync_width", "sync_enable", "phase0_cycles", "phase1_cycles",
            "min_latency_cycles", "max_latency_cycles", "watchdog_cycles", "max_skew_cycles",
            "sof", "admitted", "retired", "active", "pipeline_ready", "local_fault",
            "receiver_fault", "packet_fault", "health_good", "transport_drained",
            "start", "stop", "clear", "reset", "snapshot", "sync_outputs",
            "grants", "busy", "drained", "abort", "fault", "locked", "ready",
            "run_request", "pairs_started", "pairs_completed", "fault_cause",
        )
        for name in fields:
            signal = getattr(pair, name)
            signal.name_override = name
            self.ios.add(signal)
        for index, signal in enumerate(pair.pending_frame_id):
            signal.name_override = f"pair_id{index}"
            self.ios.add(signal)


_PAIR_PHASE_RTL_BENCH = r"""`timescale 1ns/1ps
module tb;
reg sys_clk=0, sys_rst=1;
always #5 sys_clk=~sys_clk;
reg [31:0] sync_period=100, sync_width=5;
reg [31:0] phase0_cycles=0, phase1_cycles=99;
reg [31:0] min_latency_cycles=2, max_latency_cycles=80;
reg [31:0] watchdog_cycles=200, max_skew_cycles=10;
reg sync_enable=0;
reg [1:0] sof=0, admitted=0, retired=0, active=0, pipeline_ready=3;
reg [1:0] local_fault=0, receiver_fault=0, packet_fault=0;
reg health_good=1, transport_drained=1;
reg start=0, stop=0, clear=0, reset=0, snapshot=0;
wire [1:0] sync_outputs, grants;
wire busy, drained, abort, fault, locked, ready, run_request;
wire [31:0] pairs_started, pairs_completed, fault_cause, pair_id0, pair_id1;
pair_phase_probe dut(.*);
// Check combinational abort at the sampling edge. SOF is a testbench input
// held until the following negedge; post-edge combinational values can see
// updated history and that stale input before the bench deasserts it.
always @(posedge sys_clk)if(!sys_rst&&abort)
 $fatal(1,"Unexpected abort at sampling edge cause=%08x",fault_cause);
integer cycle, channel, expected0, expected1;
integer sof_due[0:1], retire_due[0:1], count_admitted[0:1], count_retired[0:1];
integer start_sent=0, stop_due=-1, second_base=-1, late_grant_seen=0, overlap_seen=0;
reg [1:0] previous_sync=0;
reg [31:0] observed_id;
initial begin
 for(channel=0; channel<2; channel=channel+1)begin
  sof_due[channel]=-1;retire_due[channel]=-1;
  count_admitted[channel]=0;count_retired[channel]=0;
 end
 repeat(4)@(negedge sys_clk);
 sys_rst=0;sync_enable=1;
 for(cycle=0;cycle<900;cycle=cycle+1)begin
  @(posedge sys_clk);#1;
  // A delayed pulse may span a common-base wrap, but there must be no
  // synthetic partial pulse at startup from the preceding nonexistent epoch.
  expected0=((cycle%100)<5);
  expected1=(cycle>=99)&&(((cycle-99)%100)<5);
  if(sync_outputs[0]!==expected0[0]||sync_outputs[1]!==expected1[0])
   $fatal(1,"Pulse width/phase/startup at %0d: got %b expected %0d%0d",
          cycle,sync_outputs,expected1,expected0);
  if(fault)$fatal(1,"Unexpected paired fault at %0d cause=%08x",cycle,fault_cause);
  for(channel=0;channel<2;channel=channel+1)begin
   if(sync_outputs[channel]&&!previous_sync[channel])
    sof_due[channel]=cycle+10+2*channel;
   if(admitted[channel])begin
    observed_id=channel?pair_id1:pair_id0;
    if(observed_id!==count_admitted[channel])
     $fatal(1,"Camera %0d admitted ID %0d expected %0d at %0d",
            channel,observed_id,count_admitted[channel],cycle);
    count_admitted[channel]=count_admitted[channel]+1;
    retire_due[channel]=cycle+8;
   end
   if(retired[channel])count_retired[channel]=count_retired[channel]+1;
  end
  previous_sync=sync_outputs;
  if(pairs_started==2&&second_base<0)begin
   second_base=cycle;stop_due=cycle+50;
  end
  // Camera 1 still belongs to the first pair after the next camera 0
  // pulse. A single shared pending-ID register would corrupt this case.
  if(second_base>=0&&cycle==second_base+1)begin
   if(pair_id0!==1||pair_id1!==0||grants!==3)
    $fatal(1,"Overlapping epochs lost IDs/grants: %0d/%0d grants=%b",pair_id0,pair_id1,grants);
   overlap_seen=1;
  end
  if(stop_due>=0&&cycle==stop_due+1)begin
   if(run_request||!busy||grants[1])
    $fatal(1,"STOP must preserve the issued epoch before its late peer pulse");
  end
  if(second_base>=0&&cycle==second_base+99)begin
   if(!grants[1]||pair_id1!==1||run_request)
    $fatal(1,"STOP suppressed the final delayed grant/ID");
   late_grant_seen=1;
  end
  if(stop_due>=0&&cycle==stop_due+180)begin
   if(pairs_started!==2||pairs_completed!==2||busy||!drained)
    $fatal(1,"Capture did not finish exactly two issued pairs: %0d/%0d busy=%b drained=%b",
           pairs_started,pairs_completed,busy,drained);
   for(channel=0;channel<2;channel=channel+1)
    if(count_admitted[channel]!=2||count_retired[channel]!=2)
     $fatal(1,"Camera %0d count admitted=%0d retired=%0d",channel,count_admitted[channel],count_retired[channel]);
   if(!overlap_seen||!late_grant_seen)$fatal(1,"Required overlap/late-grant path not covered");
   $display("PASS phase wrap, startup, overlap IDs, late-peer STOP");$finish;
  end
  @(negedge sys_clk);
  start=0;stop=(cycle+1==stop_due);sof=0;admitted=0;retired=0;
  if(locked&&!start_sent)begin start=1;start_sent=1;end
  for(channel=0;channel<2;channel=channel+1)begin
   if(cycle+1==sof_due[channel])begin
    sof[channel]=1;
    if(grants[channel])begin admitted[channel]=1;active[channel]=1;end
   end
   if(cycle+1==retire_due[channel])begin retired[channel]=1;active[channel]=0;end
  end
 end
 $fatal(1,"Timeout: locked=%b requested=%b started=%0d complete=%0d cause=%08x",
        locked,run_request,pairs_started,pairs_completed,fault_cause);
end
endmodule
"""


@pytest.mark.skipif(shutil.which("iverilog") is None or shutil.which("vvp") is None,
                    reason="Icarus Verilog required")
def test_phase_lookahead_matches_counter_through_reset_and_reconfiguration(tmp_path):
    top = _PairPhaseRTLHarness()
    rtl = verilog.convert(top, ios=top.ios, name="pair_phase_probe")
    names = {}
    for field in ("phase_counter", "phase_zero", "phase_match"):
        match = re.search(r"reg(?: \[[^\]]+\])? (\w*" + field + r") =", rtl.main_source)
        assert match, field
        names[field] = match.group(1)
    bench = r"""`timescale 1ns/1ps
module tb;
reg sys_clk=0, sys_rst=1, sync_enable=0;
always #5 sys_clk=~sys_clk;
reg [20:0] sync_period=17, phase0_cycles=0, phase1_cycles=16;
reg reset=0;
pair_phase_probe dut(
 .sys_clk(sys_clk),.sys_rst(sys_rst),.sync_enable(sync_enable),
 .sync_period(sync_period),.sync_width(21'd1),
 .phase0_cycles(phase0_cycles),.phase1_cycles(phase1_cycles),
 .min_latency_cycles(21'd0),.max_latency_cycles(21'd1),
 .watchdog_cycles(22'd100),.max_skew_cycles(22'd10),
 .health_good(1'b1),.transport_drained(1'b1),.pipeline_ready(2'b11),
 .local_fault(2'b00),.receiver_fault(2'b00),.packet_fault(2'b00),
 .sof(2'b00),.admitted(2'b00),.active(2'b00),.retired(2'b00),
 .start(1'b0),.stop(1'b0),.clear(1'b0),.reset(reset),.snapshot(1'b0));
integer samples=0, scenario;
always @(posedge sys_clk) begin
 #1;
 if(dut.COUNTER_ZERO !== (dut.COUNTER == 0) ||
    dut.COUNTER_MATCH !== {dut.COUNTER == phase1_cycles, dut.COUNTER == phase0_cycles})
  $fatal(1,"Cached phase differs at sample %0d: count=%0d phases=%0d/%0d",samples,
         dut.COUNTER,phase0_cycles,phase1_cycles);
 samples=samples+1;
end
initial begin
 repeat(3)@(negedge sys_clk);
 sys_rst=0;
 for(scenario=0; scenario<32; scenario=scenario+1)begin
  // Legal timing writes occur while disabled; one separate enable write
  // follows. Change phases and period, including equal/zero/end phases.
  sync_enable=0;
  @(negedge sys_clk);
  sync_period=2+scenario*13;
  phase0_cycles=(scenario%3==0)?0:sync_period-1;
  @(negedge sys_clk);
  phase1_cycles=(scenario%3==1)?phase0_cycles:sync_period/2;
  @(negedge sys_clk); sync_enable=1;
  repeat(2*sync_period+3)@(negedge sys_clk);
  // CSR evidence reset preserves trigger phase; global reset zeros it.
  reset=1;
  repeat(2)@(negedge sys_clk); reset=0;
  repeat(3)@(negedge sys_clk); sys_rst=1;
  repeat(3)@(negedge sys_clk); sys_rst=0;
  repeat(sync_period+2)@(negedge sys_clk);
 end
 if(samples<19000)$fatal(1,"Insufficient phase coverage");
 $display("PASS phase lookahead %0d cycles",samples);$finish;
end
endmodule
"""
    for token, field in (("COUNTER_ZERO", "phase_zero"),
                         ("COUNTER_MATCH", "phase_match"), ("COUNTER", "phase_counter")):
        bench = bench.replace(token, names[field])
    (tmp_path / "dut.v").write_text(rtl.main_source)
    (tmp_path / "tb.v").write_text(bench)
    for command in (
        ["iverilog", "-g2012", "-s", "tb", "-o", "simulate", "dut.v", "tb.v"],
        ["vvp", "simulate"],
    ):
        result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS phase lookahead" in result.stdout


@pytest.mark.skipif(shutil.which("iverilog") is None or shutil.which("vvp") is None,
                    reason="Icarus Verilog required")
def test_phase_wrap_overlap_ids_and_late_peer_stop_in_generated_rtl(tmp_path):
    top = _PairPhaseRTLHarness()
    rtl = verilog.convert(top, ios=top.ios, name="pair_phase_probe")
    (tmp_path / "dut.v").write_text(rtl.main_source)
    for name, content in rtl.data_files.items():
        (tmp_path / name).write_text(content)
    (tmp_path / "tb.v").write_text(_PAIR_PHASE_RTL_BENCH)
    for command in (
        ["iverilog", "-g2012", "-s", "tb", "-o", "simulate", "dut.v", "tb.v"],
        ["vvp", "simulate"],
    ):
        result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS phase wrap, startup, overlap IDs, late-peer STOP" in result.stdout


@pytest.mark.skipif(shutil.which("iverilog") is None or shutil.which("vvp") is None,
                    reason="Icarus Verilog required")
def test_lagging_retirement_count_across_32_bit_wrap_in_generated_rtl(tmp_path):
    top = _PairPhaseRTLHarness()
    rtl = verilog.convert(top, ios=top.ios, name="pair_phase_probe")
    # Seed the completed count near rollover; lead arithmetic remains real RTL.
    bench = r"""`timescale 1ns/1ps
module tb;
reg sys_clk=0,sys_rst=1;
always #5 sys_clk=~sys_clk;
reg [1:0]retired=0;
wire [31:0]pairs_completed;
wire fault;
pair_phase_probe dut(
 .sys_clk(sys_clk),.sys_rst(sys_rst),.sync_enable(1'b0),
 .sync_period(32'd100),.sync_width(32'd5),.phase0_cycles(32'd0),.phase1_cycles(32'd0),
 .min_latency_cycles(32'd2),.max_latency_cycles(32'd80),
 .watchdog_cycles(32'd200),.max_skew_cycles(32'd10),
 .health_good(1'b1),.transport_drained(1'b1),.pipeline_ready(2'b11),
 .local_fault(2'b00),.receiver_fault(2'b00),.packet_fault(2'b00),
 .sof(2'b00),.admitted(2'b00),.active(2'b00),.retired(retired),
 .start(1'b0),.stop(1'b0),.clear(1'b0),.reset(1'b0),.snapshot(1'b0),
 .pairs_completed(pairs_completed),.fault(fault));
initial begin
 repeat(4)@(negedge sys_clk);sys_rst=0;
 dut.pairs_completed=32'hfffffffe;
 @(negedge sys_clk);
 if(pairs_completed!==32'hfffffffe)$fatal(1,"Seed did not reach completed output");
 retired=1;
 @(negedge sys_clk);
 if(pairs_completed!==32'hfffffffe)$fatal(1,"Leading camera changed completion");
 @(negedge sys_clk);retired=0;
 if(pairs_completed!==32'hfffffffe)$fatal(1,"Leading wrap selected zero too early");
 @(negedge sys_clk);retired=2;
 @(negedge sys_clk);
 if(pairs_completed!==32'hffffffff)$fatal(1,"Lagging camera did not advance to ffffffff");
 @(negedge sys_clk);retired=0;
 if(pairs_completed!==0||fault)$fatal(1,"Completed pair count did not wrap cleanly");
 $display("PASS lagging retirement counter rollover");$finish;
end
endmodule
"""
    (tmp_path / "dut.v").write_text(rtl.main_source)
    (tmp_path / "tb.v").write_text(bench)
    for command in (
        ["iverilog", "-g2012", "-s", "tb", "-o", "simulate", "dut.v", "tb.v"],
        ["vvp", "simulate"],
    ):
        result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS lagging retirement counter rollover" in result.stdout


class _PairTimingRTLHarness(_PairPhaseRTLHarness):
    def __init__(self):
        super().__init__()
        for name in ("snap_epoch", "snap_sync", "snap_sof0", "snap_sof1", "snap_skew",
                     "snap_residual_skew", "snap_latency0", "snap_latency1",
                     "snap_matched_pairs", "snap_missing_events"):
            signal = getattr(self.pair, name)
            signal.name_override = name
            self.ios.add(signal)


@pytest.mark.skipif(shutil.which("iverilog") is None or shutil.which("vvp") is None,
                    reason="Icarus Verilog required")
@pytest.mark.parametrize("time_seed", [0x0700fffff8, 0x07fffffff5])
@pytest.mark.parametrize("phases", [(0, 0), (0, 99), (99, 0), (99, 99)])
def test_timestamp_and_epoch_rollovers_in_generated_rtl(tmp_path, time_seed, phases):
    top = _PairTimingRTLHarness()
    rtl = verilog.convert(top, ios=top.ios, name="pair_timing_probe")
    now_name = re.search(r"reg \[63:0\] (\w+_now) =", rtl.main_source)
    epoch_name = re.search(r"reg \[31:0\] (\w+_raw_epoch) =", rtl.main_source)
    assert now_name and epoch_name, "Cannot locate timestamp/epoch state for rollover seeding"
    bench = r"""`timescale 1ns/1ps
module tb;
reg sys_clk=0,sys_rst=1;
always #5 sys_clk=~sys_clk;
localparam [63:0] TIME_SEED=64'hTIME_VALUE;
localparam integer PHASE0=PHASE_ZERO,PHASE1=PHASE_ONE;
localparam integer COMPLETE_AT=(PHASE0+10>PHASE1+12)?PHASE0+10:PHASE1+12;
reg sync_enable=0,snapshot=0;
reg [1:0]sof=0;
wire [63:0]snap_sync,snap_sof0,snap_sof1;
wire [31:0]snap_epoch,snap_matched_pairs,snap_missing_events;
wire signed [31:0]snap_skew,snap_residual_skew,snap_latency0,snap_latency1;
wire fault,locked;
pair_timing_probe dut(
 .sys_clk(sys_clk),.sys_rst(sys_rst),.sync_enable(sync_enable),
 .sync_period(24'd100),.sync_width(24'd5),.phase0_cycles(PHASE0),.phase1_cycles(PHASE1),
 .min_latency_cycles(24'd2),.max_latency_cycles(24'd80),
 .watchdog_cycles(24'd200),.max_skew_cycles(24'd10),
 .health_good(1'b1),.transport_drained(1'b1),.pipeline_ready(2'b11),
 .local_fault(2'b00),.receiver_fault(2'b00),.packet_fault(2'b00),
 .sof(sof),.admitted(2'b00),.active(2'b00),.retired(2'b00),
 .start(1'b0),.stop(1'b0),.clear(1'b0),.reset(1'b0),.snapshot(snapshot),
 .snap_epoch(snap_epoch),.snap_sync(snap_sync),.snap_sof0(snap_sof0),.snap_sof1(snap_sof1),
 .snap_skew(snap_skew),.snap_residual_skew(snap_residual_skew),
 .snap_latency0(snap_latency0),.snap_latency1(snap_latency1),
 .snap_matched_pairs(snap_matched_pairs),.snap_missing_events(snap_missing_events),
 .fault(fault),.locked(locked));
integer cycle,n,snapshots=0;
reg [63:0]expected_anchor;
reg [31:0]expected_epoch;
initial begin
 repeat(4)@(negedge sys_clk);sys_rst=0;
 dut.NOW_NAME=TIME_SEED;dut.EPOCH_NAME=32'hfffffffe;sync_enable=1;
 for(cycle=0;cycle<350;cycle=cycle+1)begin
  @(posedge sys_clk);#1;
  if(snapshot)begin
   n=(cycle-COMPLETE_AT-3)/100;
   expected_anchor=TIME_SEED+100*n;
   expected_epoch=32'hffffffff+n;
   if(snap_epoch!==expected_epoch)
    $fatal(1,"Epoch rollover/current-previous association: got %08x expected %08x",snap_epoch,expected_epoch);
   if(snap_sync!==expected_anchor||snap_sof0!==expected_anchor+PHASE0+10||
      snap_sof1!==expected_anchor+PHASE1+12)
    $fatal(1,"Timestamp rollover: base=%016x expected=%016x sof0=%016x sof1=%016x",
           snap_sync,expected_anchor,snap_sof0,snap_sof1);
   if(snap_latency0!==10||snap_latency1!==12||snap_residual_skew!==2||
      snap_skew!==PHASE1-PHASE0+2)
    $fatal(1,"Truncated timestamp delta: latency=%0d/%0d skew=%0d residual=%0d",
           snap_latency0,snap_latency1,snap_skew,snap_residual_skew);
   if(snap_matched_pairs!==n+1||snap_missing_events!==0||fault)
    $fatal(1,"Good rollover pair counted incorrectly: matched=%0d missing=%0d",snap_matched_pairs,snap_missing_events);
   if(n>=1&&!locked)$fatal(1,"Rollover prevented synchronization lock");
   snapshots=snapshots+1;
   if(snapshots==3)begin
    $display("PASS timestamp low24/low32 and epoch rollover");$finish;
   end
  end
  @(negedge sys_clk);
  sof[0]=(cycle+1>=PHASE0+10)&&((cycle+1-PHASE0-10)%100==0);
  sof[1]=(cycle+1>=PHASE1+12)&&((cycle+1-PHASE1-12)%100==0);
  snapshot=(cycle+1>=COMPLETE_AT+3)&&((cycle+1-COMPLETE_AT-3)%100==0);
 end
 $fatal(1,"Rollover scenario did not produce three snapshots");
end
endmodule
"""
    replacements = {
        "TIME_VALUE": f"{time_seed:016x}", "PHASE_ZERO": str(phases[0]),
        "PHASE_ONE": str(phases[1]), "NOW_NAME": now_name.group(1),
        "EPOCH_NAME": epoch_name.group(1),
    }
    for token, value in replacements.items():
        bench = bench.replace(token, value)
    (tmp_path / "dut.v").write_text(rtl.main_source)
    (tmp_path / "tb.v").write_text(bench)
    for command in (
        ["iverilog", "-g2012", "-s", "tb", "-o", "simulate", "dut.v", "tb.v"],
        ["vvp", "simulate"],
    ):
        result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS timestamp low24/low32 and epoch rollover" in result.stdout


class _PairSnapshotStabilityHarness(Module):
    def __init__(self):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.submodules.pair = pair = PairedCaptureControl()
        self.ios = {self.cd_sys.clk, self.cd_sys.rst}
        for name in (
            "sync_period", "sync_width", "sync_enable", "phase0_cycles", "phase1_cycles",
            "min_latency_cycles", "max_latency_cycles", "watchdog_cycles", "max_skew_cycles",
            "sof", "admitted", "retired", "active", "pipeline_ready", "local_fault",
            "receiver_fault", "packet_fault", "health_good", "transport_drained",
            "start", "stop", "clear", "reset", "snapshot", "sync_outputs", "status",
            "snap_epoch", "snap_sync", "snap_sof0", "snap_sof1", "snap_skew",
            "snap_min_skew", "snap_max_skew", "snap_residual_skew", "snap_min_residual_skew",
            "snap_max_residual_skew", "snap_latency0", "snap_latency1", "snap_min_latency",
            "snap_max_latency", "snap_sof0_count", "snap_sof1_count", "snap_sync_count",
            "snap_matched_pairs", "snap_missing_events",
        ):
            signal = getattr(pair, name)
            signal.name_override = name
            self.ios.add(signal)


_PAIR_SNAPSHOT_STABILITY_BENCH = r"""`timescale 1ns/1ps
module tb;
reg sys_clk=0, sys_rst=1;
always #5 sys_clk=~sys_clk;
reg [23:0] sync_period=100, sync_width=5;
reg [23:0] phase0_cycles=0, phase1_cycles=99;
reg [23:0] min_latency_cycles=2, max_latency_cycles=80;
reg [23:0] watchdog_cycles=200, max_skew_cycles=10;
reg sync_enable=0;
reg [1:0] sof=0, admitted=0, retired=0, active=0, pipeline_ready=3;
reg [1:0] local_fault=0, receiver_fault=0, packet_fault=0;
reg health_good=1, transport_drained=1;
reg start=0, stop=0, clear=0, reset=0, snapshot=0;
wire [1:0] sync_outputs;
wire [9:0] status;
wire [63:0] snap_sync, snap_sof0, snap_sof1;
wire [31:0] snap_epoch, snap_skew, snap_min_skew, snap_max_skew;
wire [31:0] snap_residual_skew, snap_min_residual_skew, snap_max_residual_skew;
wire [31:0] snap_latency0, snap_latency1, snap_min_latency, snap_max_latency;
wire [31:0] snap_sof0_count, snap_sof1_count, snap_sync_count, snap_matched_pairs, snap_missing_events;
wire [703:0] evidence={snap_epoch,snap_sync,snap_sof0,snap_sof1,
 snap_skew,snap_min_skew,snap_max_skew,snap_residual_skew,snap_min_residual_skew,
 snap_max_residual_skew,snap_latency0,snap_latency1,snap_min_latency,snap_max_latency,
 snap_sof0_count,snap_sof1_count,snap_sync_count,snap_matched_pairs,snap_missing_events};
reg [703:0] saved_evidence;
reg [63:0] saved_sync,saved_sof0,saved_sof1;
pair_snapshot_stability_probe dut(.*);
integer cycle,channel,sof_due[0:1];
reg [1:0] previous_sync=0;
task check_snapshot;
 begin
  if(!status[9])$fatal(1,"Historical snapshot lost validity");
  if(evidence!==saved_evidence)
   $fatal(1,"Historical snapshot changed: sync %h/%h SOF0 %h/%h SOF1 %h/%h",
          snap_sync,saved_sync,snap_sof0,saved_sof0,snap_sof1,saved_sof1);
 end
endtask
initial begin
 sof_due[0]=-1;sof_due[1]=-1;
 repeat(4)@(negedge sys_clk);
 sys_rst=0;sync_enable=1;
 // Complete epoch 1 across epoch 2's base, using each sensor's own pulse.
 // Stop before epoch 2 can complete, leaving precisely one historical pair.
 for(cycle=0;cycle<125;cycle=cycle+1)begin
  @(posedge sys_clk);#1;
  for(channel=0;channel<2;channel=channel+1)
   if(sync_outputs[channel]&&!previous_sync[channel])
    sof_due[channel]=cycle+10+2*channel;
  previous_sync=sync_outputs;
  @(negedge sys_clk);
  sof=0;
  for(channel=0;channel<2;channel=channel+1)
   if(cycle+1==sof_due[channel])sof[channel]=1;
 end
 snapshot=1;
 @(posedge sys_clk);#1;
 if(!status[9]||snap_epoch!==1||snap_matched_pairs!==1||snap_missing_events!==0)
  $fatal(1,"Failed to create exactly one valid historical epoch: epoch=%0d matched=%0d missing=%0d",
         snap_epoch,snap_matched_pairs,snap_missing_events);
 if(snap_sof0-snap_sync!==64'd10||snap_sof1-snap_sync!==64'd111||
    snap_latency0!==10||snap_latency1!==12||snap_skew!==101||snap_residual_skew!==2)
  $fatal(1,"Unexpected original timing evidence");
 saved_evidence=evidence;saved_sync=snap_sync;saved_sof0=snap_sof0;saved_sof1=snap_sof1;
 @(negedge sys_clk);snapshot=0;sync_enable=0;sof=0;
 repeat(3)begin @(posedge sys_clk);#1;check_snapshot();end
 @(negedge sys_clk);
 phase0_cycles=99;phase1_cycles=0;
 // Seed the private free-running clock directly; the next two edges cross
 // another low-32 wrap after a jump of many billions of historical epochs.
 dut.__NOW__=64'h89abcdef_fffffffe;
 repeat(257)begin
  @(posedge sys_clk);#1;
  check_snapshot();
  if(sync_outputs!==0)$fatal(1,"Disabled generator emitted a trigger");
 end
 @(negedge sys_clk);snapshot=1;
 @(posedge sys_clk);#1;check_snapshot();
 @(negedge sys_clk);snapshot=0;
 repeat(8)begin @(posedge sys_clk);#1;check_snapshot();end
 $display("PASS historical snapshot survives rephase and distant clock");$finish;
end
initial begin repeat(2000)@(posedge sys_clk);$fatal(1,"Timeout");end
endmodule
"""


@pytest.mark.skipif(shutil.which("iverilog") is None or shutil.which("vvp") is None,
                    reason="Icarus Verilog required")
def test_historical_pair_snapshot_survives_rephase_and_distant_clock_in_generated_rtl(tmp_path):
    top = _PairSnapshotStabilityHarness()
    rtl = verilog.convert(top, ios=top.ios, name="pair_snapshot_stability_probe")
    # Discover the unique 64-bit free-running counter from its actual emitted
    # register declaration and increment, rather than pinning a Migen suffix.
    wide_registers = set(re.findall(r"\breg\s+\[63:0\]\s+(\w+)", rtl.main_source))
    incremented = set(re.findall(
        r"(?m)^\s*(\w+)\s*<=\s*\(\1\s*\+\s*1'd1\);", rtl.main_source))
    clocks = wide_registers & incremented
    assert len(clocks) == 1, f"Cannot identify unique free-running time counter: {clocks}"
    bench = _PAIR_SNAPSHOT_STABILITY_BENCH.replace("__NOW__", clocks.pop())
    (tmp_path / "dut.v").write_text(rtl.main_source)
    for name, content in rtl.data_files.items():
        (tmp_path / name).write_text(content)
    (tmp_path / "tb.v").write_text(bench)
    for command in (
        ["iverilog", "-g2012", "-s", "tb", "-o", "simulate", "dut.v", "tb.v"],
        ["vvp", "simulate"],
    ):
        result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS historical snapshot survives rephase and distant clock" in result.stdout


@pytest.mark.skipif(shutil.which("iverilog") is None or shutil.which("vvp") is None,
                    reason="Icarus Verilog required")
def test_full_phase_plus_latency_offset_in_generated_rtl(tmp_path):
    top = _PairTimingRTLHarness()
    rtl = verilog.convert(top, ios=top.ios, name="pair_timing_probe")
    now_name = re.search(r"reg \[63:0\] (\w+_now) =", rtl.main_source)
    counter_name = re.search(r"reg\s+\[20:0\]\s+(\w*phase_counter)\s*=",
                             rtl.main_source)
    assert now_name and counter_name, "Cannot locate trigger clock/counter for time jump"
    # Fast-forward the quiet part of a maximum-length period. Both phase
    # pulses then occur normally, and their SOFs cross the following base.
    # This specifically requires bit21 of the saved phase+latency offset.
    bench = r"""`timescale 1ns/1ps
module tb;
reg sys_clk=0,sys_rst=1;
always #5 sys_clk=~sys_clk;
localparam [63:0] TIME_SEED=64'h00000007fffffff5;
reg sync_enable=0,snapshot=0;
reg [1:0]sof=0;
wire [63:0]snap_sync,snap_sof0,snap_sof1;
wire [31:0]snap_epoch,snap_matched_pairs,snap_missing_events;
wire signed [31:0]snap_skew,snap_residual_skew,snap_latency0,snap_latency1;
pair_timing_probe dut(
 .sys_clk(sys_clk),.sys_rst(sys_rst),.sync_enable(sync_enable),
 .sync_period(21'h1fffff),.sync_width(24'd5),
 .phase0_cycles(21'h1ffffd),.phase1_cycles(21'h1ffffe),
 .min_latency_cycles(24'd2),.max_latency_cycles(24'd80),
 .watchdog_cycles(24'd200),.max_skew_cycles(24'd10),
 .health_good(1'b1),.transport_drained(1'b1),.pipeline_ready(2'b11),
 .local_fault(2'b00),.receiver_fault(2'b00),.packet_fault(2'b00),
 .sof(sof),.admitted(2'b00),.active(2'b00),.retired(2'b00),
 .start(1'b0),.stop(1'b0),.clear(1'b0),.reset(1'b0),.snapshot(snapshot),
 .snap_epoch(snap_epoch),.snap_sync(snap_sync),.snap_sof0(snap_sof0),.snap_sof1(snap_sof1),
 .snap_skew(snap_skew),.snap_residual_skew(snap_residual_skew),
 .snap_latency0(snap_latency0),.snap_latency1(snap_latency1),
 .snap_matched_pairs(snap_matched_pairs),.snap_missing_events(snap_missing_events));
integer cycle;
initial begin
 repeat(4)@(negedge sys_clk);sys_rst=0;dut.NOW_NAME=TIME_SEED;sync_enable=1;
 @(posedge sys_clk);#1;
 @(negedge sys_clk);
 // Seed one quiet clock before the phase so its registered look-ahead
 // predicate advances normally, rather than forcing inconsistent state.
 dut.COUNTER_NAME=21'h1ffffc;dut.NOW_NAME=TIME_SEED+64'h1ffffc;
 @(posedge sys_clk);#1;@(negedge sys_clk);
 for(cycle=0;cycle<20;cycle=cycle+1)begin
  @(posedge sys_clk);#1;
  if(snapshot)begin
   if(snap_sync!==TIME_SEED||snap_sof0!==TIME_SEED+64'h200007||
      snap_sof1!==TIME_SEED+64'h20000a)
    $fatal(1,"Lost offset bit21: base=%016x sof0=%016x sof1=%016x",snap_sync,snap_sof0,snap_sof1);
   if(snap_epoch!==1||snap_latency0!==10||snap_latency1!==12||
      snap_skew!==3||snap_residual_skew!==2||snap_matched_pairs!==1||snap_missing_events!==0)
    $fatal(1,"Maximum phase corrupted epoch or timing evidence");
   $display("PASS full 22-bit phase plus latency offset");$finish;
  end
  @(negedge sys_clk);
  sof[0]=(cycle+1==10);sof[1]=(cycle+1==13);snapshot=(cycle+1==16);
 end
 $fatal(1,"Maximum phase scenario did not snapshot");
end
endmodule
"""
    bench = bench.replace("NOW_NAME", now_name.group(1)).replace("COUNTER_NAME", counter_name.group(1))
    (tmp_path / "dut.v").write_text(rtl.main_source)
    (tmp_path / "tb.v").write_text(bench)
    for command in (
        ["iverilog", "-g2012", "-s", "tb", "-o", "simulate", "dut.v", "tb.v"],
        ["vvp", "simulate"],
    ):
        result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS full 22-bit phase plus latency offset" in result.stdout

@pytest.mark.skipif(shutil.which("iverilog") is None or shutil.which("vvp") is None,
                   reason="Icarus Verilog required")
def test_retirement_randomized_bounded_leads_and_wrap(tmp_path):
    top = _PairPhaseRTLHarness()
    rtl = verilog.convert(top, ios=top.ios, name="pair_phase_probe")
    bench = r'''`timescale 1ns/1ps
module tb;
reg sys_clk=0,sys_rst=1;
always #5 sys_clk=~sys_clk;
reg [1:0]retired=0;
reg clear=0;
wire [31:0]pairs_completed;
wire fault;
pair_phase_probe dut(
 .sys_clk(sys_clk),.sys_rst(sys_rst),.sync_enable(1'b0),
 .sync_period(24'd100),.sync_width(24'd5),.phase0_cycles(24'd0),.phase1_cycles(24'd0),
 .min_latency_cycles(24'd2),.max_latency_cycles(24'd80),
 .watchdog_cycles(24'd200),.max_skew_cycles(24'd10),
 .health_good(1'b1),.transport_drained(1'b1),.pipeline_ready(2'b11),
 .local_fault(2'b00),.receiver_fault(2'b00),.packet_fault(2'b00),
 .sof(2'b00),.admitted(2'b00),.active(2'b00),.retired(retired),
 .start(1'b0),.stop(1'b0),.clear(clear),.reset(1'b0),.snapshot(1'b0),
 .pairs_completed(pairs_completed),.fault(fault));
integer cycle,seed,lead,p;
reg [63:0]count0,count1,minimum;
initial begin
 seed=32'h123abcd;
 repeat(4)@(negedge sys_clk);sys_rst=0;
 dut.pairs_completed=32'hfffffffe;
 count0=64'hfffffffe;count1=64'hfffffffe;
 for(cycle=0;cycle<20000;cycle=cycle+1)begin
  p=$random(seed)&3;
  lead=count0-count1;
  if(lead>=3&&p==1)p=2;
  if(lead<=-3&&p==2)p=1;
  retired=p;
  @(negedge sys_clk);
  count0=count0+(p&1);count1=count1+((p>>1)&1);
  minimum=count0<count1?count0:count1;
  if(pairs_completed!==minimum[31:0]||fault)
   $fatal(1,"Retire mismatch cycle%0d c0%h c1%h actual%h",cycle,count0,count1,pairs_completed);
 end
 retired=0;clear=1;@(negedge sys_clk);clear=0;
 if(pairs_completed!==0)$fatal(1,"Clear failed");
 retired=1;@(negedge sys_clk);
 if(pairs_completed!==0)$fatal(1,"Clear retained stale lead");
 retired=2;@(negedge sys_clk);retired=0;
 if(pairs_completed!==1)$fatal(1,"Post-clear completion failed");
 $display("PASS 20000 randomized retirements and modular wrap");$finish;
end
endmodule
'''
    (tmp_path / 'dut.v').write_text(rtl.main_source)
    (tmp_path / 'tb.v').write_text(bench)
    subprocess.run(['iverilog','-g2012','-s','tb','-o','sim','dut.v','tb.v'],cwd=tmp_path,check=True,capture_output=True)
    result=subprocess.run(['vvp','sim'],cwd=tmp_path,check=True,capture_output=True,text=True)
    assert 'PASS 20000 randomized retirements and modular wrap' in result.stdout
