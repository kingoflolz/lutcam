"""Shared frame triggers, admission tokens, and timing evidence for two cameras."""

from migen import Cat, ClockSignal, Constant, If, Instance, Module, Mux, ResetSignal, Signal


class _ResidualSkewGuard(Module):
    """Compare signed residual bounds without negating the late data path."""

    def __init__(self, residual, maximum):
        self.within = Signal()
        extended_residual = Signal((23, True))
        positive_limit, negative_limit = Signal((23, True)), Signal((23, True))
        self.comb += [
            extended_residual.eq(Cat(residual, residual[21])),
            positive_limit.eq(Cat(maximum, Constant(0, 1))),
            negative_limit.eq(-positive_limit),
            self.within.eq((extended_residual >= negative_limit)
                           & (extended_residual <= positive_limit)),
        ]


class PairedCaptureControl(Module):
    """Issue a common pair ID with independently phased slave-camera triggers.

    Each channel has one pending admission token; its previous frame may still
    be processing when the next trigger arrives. CameraPipeline checks readiness
    at SOF. STOP finishes both members of the last issued base epoch. Raw SOFs
    are associated with their own trigger epoch using two parity slots: phase
    and trigger-to-SOF latency are each strictly less than one period.

    Timestamps describe FPGA arrival after CSI clock crossing, not exposure.
    Two consecutive good pairs establish lock. CLEAR preserves warm lock;
    RESET or disabling the generator invalidates it. Timing configuration is
    guarded by IntegrationRegisters while the generator and capture are off.
    Period, phase and latency fields use 21 bits (both 60/120 fps presets fit);
    watchdog and skew guards use 22 bits. Public IDs/counters remain 32 bits
    and absolute timestamps remain 64 bits.

    The board enables DSP-backed accumulators for seven 32-bit event counters
    and the two timestamp halves. Each uses a native feedback register with
    synchronous reset; low-bit truncation preserves modulo counter semantics.
    Generic builds retain equivalent fabric counters for simulation/portability.
    """

    def _dsp_counter(self, value, enable, clear=0):
        if not self.use_dsp_counters:
            self.sync += If(clear, value.eq(0)).Elif(enable, value.eq(value + 1))
            return
        # Only low value-width bits are observed; upper DSP bits may count on.
        result = Signal(48)
        self.specials += Instance("EFX_DSP48",
            p_MODE="NORMAL", p_SIGNED=0, p_M_SEL="C", p_N_SEL="W",
            p_W_SEL="X", p_W_REG=1, p_RST_SYNC=1,
            i_A=Constant(0, 19), i_B=Constant(0, 18), i_C=Constant(1, 18),
            i_CASCIN=Constant(0, 48), i_OP=Constant(0, 2), i_SHIFT_ENA=0,
            i_CLK=ClockSignal(), i_CE=enable,
            i_RST=ResetSignal() | clear, o_O=result)
        self.comb += value.eq(result[:len(value)])

    def _snapshot_registers(self, entries, enable, clear=0):
        """Capture coherent state in fabric; reserve DSPs for arithmetic."""
        self.sync += If(clear, *[target.eq(0) for target, _ in entries]).Elif(
            enable, *[target.eq(source) for target, source in entries])

    def __init__(self, *, use_dsp_counters=False):
        self.use_dsp_counters = use_dsp_counters
        self.sync_period = Signal(21, reset=994_812)
        self.sync_width = Signal(21, reset=1_536)
        self.sync_enable = Signal()
        self.phase0_cycles = Signal(21)
        self.phase1_cycles = Signal(21)
        self.sync_outputs = Signal(2)
        self.sync_output = self.sync_outputs[0]
        self.master_sync = self.sync_output
        self.min_latency_cycles = Signal(21, reset=2)
        self.max_latency_cycles = Signal(21, reset=900_000)
        self.sof = Signal(2)
        self.admitted = Signal(2)
        self.retired = Signal(2)
        self.active = Signal(2)
        self.pipeline_ready = Signal(2)
        self.local_fault = Signal(2)
        self.receiver_fault = Signal(2)
        self.packet_fault = Signal(2)
        self.health_good = Signal()
        self.transport_drained = Signal()
        self.pending_frame_id = [Signal(32), Signal(32)]

        self.start, self.stop, self.clear, self.reset, self.snapshot = [Signal() for _ in range(5)]
        self.watchdog_cycles = Signal(22, reset=4_000_000)
        self.max_skew_cycles = Signal(22, reset=12_000)
        self.run_request, self.abort, self.busy, self.drained = [Signal() for _ in range(4)]
        self.ready, self.locked, self.fault = [Signal() for _ in range(3)]
        self.grants = Signal(2)
        self.fault_cause = Signal(32)
        self.status = Signal(10)
        self.pairs_started, self.pairs_completed = Signal(32), Signal(32)

        self.snap_epoch = Signal(32)
        self.snap_sof0, self.snap_sof1, self.snap_sync = Signal(64), Signal(64), Signal(64)
        for name in ("skew", "min_skew", "max_skew", "residual_skew", "min_residual_skew",
                     "max_residual_skew", "latency0", "latency1", "min_latency", "max_latency"):
            setattr(self, "snap_" + name, Signal((32, True)))
        for name in ("sof0_count", "sof1_count", "sync_count", "matched_pairs", "missing_events"):
            setattr(self, "snap_" + name, Signal(32))

        self.now = now = Signal(64)
        if self.use_dsp_counters:
            self._dsp_counter(now[:32], 1)
            self._dsp_counter(now[32:], now[:32] == 0xffffffff)
        else:
            self.sync += now.eq(now + 1)
        counter, raw_epoch = Signal(21, name="phase_counter"), Signal(32)
        counter_next = Signal(21)
        self.comb += counter_next.eq(counter + 1)
        next_phase_counter = Signal(21)
        phase_zero = Signal(reset=1, reset_less=True, name="phase_zero")
        phase_match = Signal(2, reset=3, reset_less=True, name="phase_match")
        base_pulse, pulse = Signal(), Signal(2)
        pulse_epoch = Signal(32)
        phases = [self.phase0_cycles, self.phase1_cycles]
        # Cache comparisons for the counter value that will exist after this
        # edge. Pulse timing is unchanged, but the wide phase decode no longer
        # precedes latency/residual checks and immediate capture abort. Timing
        # CSRs can change only with sync disabled; their subsequent enable
        # write therefore observes refreshed predicates. Reset-less predicate
        # flops also track external phase inputs while global reset holds the
        # counter at zero, avoiding a spurious phase-zero pulse on startup.
        self.comb += next_phase_counter.eq(Mux(
            ResetSignal(allow_reset_less=True) | (self.sync_enable == 0) | (counter_next == self.sync_period),
            0, counter_next))
        self.sync += [phase_zero.eq(next_phase_counter == 0),
                      phase_match.eq(Cat(*(next_phase_counter == phase for phase in phases)))]
        self.comb += [base_pulse.eq(self.sync_enable & phase_zero),
                      pulse_epoch.eq(raw_epoch + base_pulse)]
        self.sync += If(~self.sync_enable,
            counter.eq(0), raw_epoch.eq(0),
        ).Else(
            If(counter_next == self.sync_period, counter.eq(0)).Else(counter.eq(counter_next)),
            raw_epoch.eq(pulse_epoch),
        )
        for cam in range(2):
            self.comb += pulse[cam].eq(self.sync_enable & phase_match[cam])

        cam_valid, cam_seen, cam_timed_out = Signal(2), Signal(2), Signal(2)
        cam_epoch = [Signal(2) for _ in range(2)]
        selected_epoch = [Signal(2) for _ in range(2)]
        latency, age = [Signal(21) for _ in range(2)], [Signal(21) for _ in range(2)]
        valid_sof, duplicate, missed_pulse, bad_latency, latency_timeout, clock_timeout = [Signal(2) for _ in range(6)]
        for cam in range(2):
            # One elapsed timer supplies both trigger-to-SOF latency and pulse
            # termination. Trigger priority preserves full width across the
            # shared base-period wrap; disable immediately drives FSIN low.
            self.sync += If(pulse[cam], age[cam].eq(1)).Else(age[cam].eq(age[cam] + 1))
            self.sync += If(~self.sync_enable, self.sync_outputs[cam].eq(0)
            ).Elif(pulse[cam], self.sync_outputs[cam].eq(1)
            ).Elif(age[cam] == self.sync_width, self.sync_outputs[cam].eq(0))
            self.comb += [
                selected_epoch[cam].eq(Mux(pulse[cam], pulse_epoch[:2], cam_epoch[cam])),

                latency[cam].eq(Mux(pulse[cam], 0, age[cam])),
                valid_sof[cam].eq(self.sof[cam] & (cam_valid[cam] | pulse[cam])),
                duplicate[cam].eq(self.sof[cam] & cam_valid[cam] & cam_seen[cam] & ~pulse[cam]),
                missed_pulse[cam].eq(pulse[cam] & cam_valid[cam] & ~cam_seen[cam]),
                bad_latency[cam].eq(valid_sof[cam] & ((latency[cam] < self.min_latency_cycles)
                                                    | (latency[cam] > self.max_latency_cycles))),
                latency_timeout[cam].eq(cam_valid[cam] & ~cam_seen[cam] & ~cam_timed_out[cam]
                                        & ~pulse[cam] & (age[cam] > self.max_latency_cycles)),
                clock_timeout[cam].eq(cam_valid[cam] & ~cam_seen[cam] & ~cam_timed_out[cam]
                                      & ~pulse[cam] & (age[cam] >= self.watchdog_cycles)),
            ]
            self.sync += If(self.reset | ~self.sync_enable,
                cam_valid[cam].eq(0), cam_seen[cam].eq(0), cam_timed_out[cam].eq(0),
            ).Else(
                If(pulse[cam],
                    cam_epoch[cam].eq(pulse_epoch[:2]),
                    cam_valid[cam].eq(1), cam_seen[cam].eq(self.sof[cam]), cam_timed_out[cam].eq(0),
                ).Else(
                    If(valid_sof[cam], cam_seen[cam].eq(1)),
                    If(latency_timeout[cam] | clock_timeout[cam], cam_timed_out[cam].eq(1)),
                ),
            )

        slot_complete, slot_good, slot_reuse, unmatched_sof = Signal(2), Signal(2), Signal(2), Signal(2)
        slot_latencies = [[Signal(21) for _ in range(2)] for _ in range(2)]
        slot_tags, slot_anchors = [Signal(2) for _ in range(2)], [Signal(64) for _ in range(2)]
        slot_event = [Signal(2), Signal(2)]
        complete = Signal()
        completed_anchor = Signal(64)
        completed_epoch = Signal(32)
        completed_latency = [Signal(21), Signal(21)]
        completed_offset = [Signal(22), Signal(22)]
        completed_skew, completed_residual = Signal((23, True)), Signal((22, True))
        # The residual arrives through epoch/slot selection. Keep negation
        # on the configuration side of the guard, instead of serializing a
        # data subtraction, conditional negation and magnitude comparison.
        self.submodules.residual_guard = residual_guard = _ResidualSkewGuard(
            completed_residual, self.max_skew_cycles)
        self.comb += [
            complete.eq(slot_complete != 0),
            completed_anchor.eq(Mux(slot_complete[1], slot_anchors[1], slot_anchors[0])),
            # Slot lifetime is less than two periods; the completed epoch is
            # the current one or its predecessor. Keep wide IDs only here.
            completed_epoch.eq(pulse_epoch - Mux(Mux(pulse_epoch[0], slot_complete[1], slot_complete[0]), 0, 1)),
            *[completed_latency[cam].eq(Mux(slot_complete[1], slot_latencies[1][cam], slot_latencies[0][cam]))
              for cam in range(2)],
            *[completed_offset[cam].eq(Cat(phases[cam], Constant(0, 1))
                                      + Cat(completed_latency[cam], Constant(0, 1))) for cam in range(2)],
            completed_skew.eq(Cat(completed_offset[1], Constant(0, 1))
                              - Cat(completed_offset[0], Constant(0, 1))),
            completed_residual.eq(Cat(completed_latency[1], Constant(0, 1))
                                  - Cat(completed_latency[0], Constant(0, 1))),
        ]
        for slot in range(2):
            valid, done, bad = Signal(), Signal(), Signal()
            seen = Signal(2)
            tag, anchor = Signal(2), Signal(64)
            latencies = [Signal(21), Signal(21)]
            initialize = Signal()
            seen_base, seen_next = Signal(2), Signal(2)
            event_bad = Signal()
            self.comb += [
                initialize.eq(base_pulse & (pulse_epoch[0] == slot)),
                slot_tags[slot].eq(Mux(initialize, pulse_epoch[:2], tag)),
                slot_anchors[slot].eq(Mux(initialize, now, anchor)),
                seen_base.eq(Mux(initialize, 0, seen)),
                seen_next.eq(seen_base | slot_event[slot]),
                slot_reuse[slot].eq(initialize & valid & ~done),
                event_bad.eq((slot_event[slot] & (bad_latency | duplicate)) != 0),
                slot_complete[slot].eq((valid | initialize) & ~Mux(initialize, 0, done)
                                       & (seen_next == 3)),
                slot_good[slot].eq(slot_complete[slot] & ~Mux(initialize, 0, bad) & ~event_bad
                                   & residual_guard.within),
            ]
            for cam in range(2):
                self.comb += [
                    slot_event[slot][cam].eq(valid_sof[cam] & ~duplicate[cam]
                        & (valid | initialize) & (selected_epoch[cam] == slot_tags[slot])),
                    slot_latencies[slot][cam].eq(Mux(slot_event[slot][cam], latency[cam], latencies[cam])),
                ]
            self.sync += If(self.reset | ~self.sync_enable,
                valid.eq(0), done.eq(0), bad.eq(0), seen.eq(0),
            ).Else(
                If(initialize,
                    valid.eq(1), done.eq(0), bad.eq(0), seen.eq(0), tag.eq(pulse_epoch[:2])),
                If((valid | initialize) & (slot_event[slot] != 0),
                    seen.eq(seen_next),
                    If(event_bad, bad.eq(1)),
                ),
                If(slot_complete[slot], done.eq(1)),
                *[If(slot_event[slot][cam], latencies[cam].eq(latency[cam])) for cam in range(2)],
                If(duplicate[0] & (tag == selected_epoch[0]), bad.eq(1)),
                If(duplicate[1] & (tag == selected_epoch[1]), bad.eq(1)),
            )
            self._snapshot_registers([(anchor, now)],
                initialize & self.sync_enable & ~self.reset)
        for cam in range(2):
            self.comb += unmatched_sof[cam].eq(valid_sof[cam] & ~duplicate[cam]
                                               & ~slot_event[0][cam] & ~slot_event[1][cam])
        residual_bad = Signal()
        raw_failure = Signal()
        self.comb += [
            residual_bad.eq(((slot_complete & ~slot_good) != 0)),
            raw_failure.eq((duplicate != 0) | (missed_pulse != 0) | (bad_latency != 0)
                           | (latency_timeout != 0) | (clock_timeout != 0)
                           | (slot_reuse != 0) | (unmatched_sof != 0) | residual_bad),
        ]
        lock_count = Signal(2)
        self.sync += If(self.reset | ~self.sync_enable | raw_failure,
            lock_count.eq(0), self.locked.eq(0),
        ).Elif(slot_good != 0,
            If(lock_count < 2, lock_count.eq(lock_count + 1)),
            If(lock_count >= 1, self.locked.eq(1)))

        counts = {name: Signal(32) for name in ("sof0_count", "sof1_count", "sync_count",
                                                "matched_pairs", "missing_events")}
        latest_valid, statistics_valid, snapshot_valid = Signal(), Signal(), Signal()
        latest_epoch = Signal(32)
        latest_sync = Signal(64)
        stat_widths = {"skew": (23, True), "min_skew": (23, True), "max_skew": (23, True),
                       "residual_skew": (22, True), "min_residual_skew": (22, True),
                       "max_residual_skew": (22, True), "latency0": 21, "latency1": 21,
                       "min_latency": 21, "max_latency": 21}
        stats = {name: Signal(width) for name, width in stat_widths.items()}
        latest_offset = [Signal(22), Signal(22)]
        snapshot_offset = [Signal(22), Signal(22)]
        # Share the high-word increment between both reconstructed timestamps.
        # Offsets are captured with the snapshot, so later rephasing cannot
        # change any previously published timestamp.
        snapshot_sum = [Signal(33), Signal(33)]
        snapshot_high_plus_one = Signal(32)
        self.comb += [
            snapshot_high_plus_one.eq(self.snap_sync[32:] + 1),
            *[snapshot_sum[cam].eq(Cat(self.snap_sync[:32], Constant(0, 1))
                                   + snapshot_offset[cam]) for cam in range(2)],
            self.snap_sof0.eq(Cat(snapshot_sum[0][:32], Mux(snapshot_sum[0][32],
                snapshot_high_plus_one, self.snap_sync[32:]))),
            self.snap_sof1.eq(Cat(snapshot_sum[1][:32], Mux(snapshot_sum[1][32],
                snapshot_high_plus_one, self.snap_sync[32:]))),
        ]
        skew, residual = completed_skew, completed_residual
        l0, l1 = completed_latency
        low_latency, high_latency = Signal(21), Signal(21)
        l0_is_lower = Signal()
        self.comb += [
            l0_is_lower.eq(l0 < l1),
            low_latency.eq(Mux(l0_is_lower, l0, l1)),
            high_latency.eq(Mux(l0_is_lower, l1, l0)),
        ]
        self._dsp_counter(counts["sof0_count"], self.sof[0], self.clear | self.reset)
        self._dsp_counter(counts["sof1_count"], self.sof[1], self.clear | self.reset)
        self._dsp_counter(counts["sync_count"], base_pulse, self.clear | self.reset)
        self._dsp_counter(counts["matched_pairs"], (slot_good != 0) & ~raw_failure, self.clear | self.reset)
        self._dsp_counter(counts["missing_events"], raw_failure, self.clear | self.reset)
        self.sync += If(self.clear | self.reset,
            latest_valid.eq(0), statistics_valid.eq(0), snapshot_valid.eq(0),
        ).Else(
            If(complete,
                latest_valid.eq(1), statistics_valid.eq(1),
            ),
            If(self.snapshot,
                snapshot_valid.eq(latest_valid),
            ),
        )

        self._snapshot_registers(
            [(latest_epoch, completed_epoch), (latest_sync, completed_anchor)] +
            [(latest_offset[cam], completed_offset[cam]) for cam in range(2)],
            complete & ~(self.clear | self.reset))
        self._snapshot_registers(
            [(stats["skew"], skew), (stats["residual_skew"], residual),
             (stats["latency0"], l0), (stats["latency1"], l1)],
            complete, self.clear | self.reset)
        for name, value, better in (
            ("min_skew", skew, skew < stats["min_skew"]),
            ("max_skew", skew, skew > stats["max_skew"]),
            ("min_residual_skew", residual, residual < stats["min_residual_skew"]),
            ("max_residual_skew", residual, residual > stats["max_residual_skew"]),
            ("min_latency", low_latency, low_latency < stats["min_latency"]),
            ("max_latency", high_latency, high_latency > stats["max_latency"]),
        ):
            self._snapshot_registers([(stats[name], value)],
                complete & (~statistics_valid | better), self.clear | self.reset)

        self._snapshot_registers(
            [(self.snap_epoch, latest_epoch), (self.snap_sync, latest_sync)] +
            [(snapshot_offset[cam], latest_offset[cam]) for cam in range(2)] +
            [(getattr(self, "snap_" + name), value) for name, value in stats.items()] +
            [(getattr(self, "snap_" + name), value) for name, value in counts.items()],
            self.snapshot & ~(self.clear | self.reset))

        stopping, period_issued = Signal(), Signal()
        period_pair_id = Signal(32)
        # Each camera can have one active frame plus one triggered pending
        # frame. With phase+latency<2 periods, retirement skew is at most
        # three pairs. Four signed bits leave a further guard value.
        retirement_lead = Signal((4, True), name="retirement_lead")
        complete_retirement = Signal()
        issue, fault_now, bad_admission = Signal(), Signal(), Signal()
        causes = Signal(32)
        grant_at_pulse = Signal(2)
        self.comb += [
            self.busy.eq(self.run_request | stopping | (~self.fault & (self.pairs_completed != self.pairs_started))),
            self.drained.eq((self.active == 0) & (self.grants == 0) & self.transport_drained
                            & (self.fault | (self.pairs_completed == self.pairs_started))),
            self.ready.eq((self.pipeline_ready == 3) & self.health_good & ~self.fault
                          & (self.local_fault == 0) & (self.receiver_fault == 0) & (self.packet_fault == 0)),
            complete_retirement.eq((self.retired == 3)
                | (self.retired[0] & (retirement_lead < 0))
                | (self.retired[1] & (retirement_lead > 0))),
            # Bits0..5 local/packet/receiver,6 watchdog,7 residual,
            # 8 missing/duplicate/association,9 missed admission,10 unexpected
            # admission,11 health,12 latency.
            causes.eq(Cat(self.local_fault, self.packet_fault, self.receiver_fault,
                clock_timeout != 0, residual_bad,
                (missed_pulse | duplicate | unmatched_sof | slot_reuse) != 0,
                (pulse & self.grants) != 0, bad_admission, ~self.health_good,
                (bad_latency | latency_timeout) != 0, Constant(0, 19))),
            fault_now.eq(~self.fault & self.busy & (causes != 0)),
            self.abort.eq(self.fault | fault_now),
            issue.eq(base_pulse & self.run_request & ~self.stop & self.locked
                     & self.health_good & ~raw_failure & ~self.abort),
            *[grant_at_pulse[cam].eq(pulse[cam] & Mux(base_pulse, issue, period_issued)) for cam in range(2)],
            self.status.eq(Cat(self.run_request, self.run_request & (self.grants == 0),
                ~self.fault & (self.pairs_completed != self.pairs_started), stopping, self.drained, self.fault,
                self.locked, self.ready, self.sync_output, snapshot_valid)),
        ]
        # CameraPipeline masks admitted with abort. Register the diagnostic to
        # avoid abort -> admitted -> abort combinational feedback.
        self.sync += bad_admission.eq(~(self.clear | self.reset)
                                      & ((self.admitted & ~self.grants) != 0))
        self.sync += If(self.clear | self.reset,
            self.run_request.eq(0), self.grants.eq(0), stopping.eq(0), period_issued.eq(0),
            self.fault.eq(0), self.fault_cause.eq(0),
            retirement_lead.eq(0),
            *[identity.eq(0) for identity in self.pending_frame_id],
        ).Elif(fault_now,
            self.fault.eq(1), self.fault_cause.eq(self.fault_cause | causes),
            self.run_request.eq(0), self.grants.eq(0), period_issued.eq(0), stopping.eq(1),
        ).Else(
            If(self.retired == 1, retirement_lead.eq(retirement_lead + 1))
                .Elif(self.retired == 2, retirement_lead.eq(retirement_lead - 1)),
            If(self.start & ~self.busy & self.drained & self.ready & self.locked & self.sync_enable,
                self.run_request.eq(1)),
            If(self.stop, self.run_request.eq(0), stopping.eq(1)),
            If(base_pulse, period_issued.eq(issue)),
            If(issue, period_pair_id.eq(self.pairs_started)),
            *[If(grant_at_pulse[cam] & ~self.abort,
                    self.grants[cam].eq(1),
                    self.pending_frame_id[cam].eq(Mux(base_pulse, self.pairs_started, period_pair_id)),
                ).Elif(self.admitted[cam], self.grants[cam].eq(0)) for cam in range(2)],
            If(stopping & self.drained & ~self.run_request, stopping.eq(0)),
        )

        self._dsp_counter(self.pairs_started, issue & ~fault_now, self.clear | self.reset)
        self._dsp_counter(self.pairs_completed, complete_retirement & ~fault_now, self.clear | self.reset)
