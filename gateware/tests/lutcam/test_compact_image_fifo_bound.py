"""Check the scope of the compact image FIFO argument.

The weaker byte-envelope assumptions admit an abstract nine-slot trace. The
covered fixed sensor profiles additionally provide an end-of-frame bound that
isolates consecutive frames. These checks retain that distinction; the
counterexample is not a claim that the production codec generates its trace.
"""

from collections import deque
from math import gcd
import unittest


FRAGMENT_BYTES = 1024
FRAME_BYTES = 128000
FRAME_CLOCKS = 993706
SERVICE_CLOCKS = 4000
STARTUP_CLOCKS = 1800
CREDIT_BYTES = 5632
ENVELOPE_BYTES = 5696


def boundary_counterexample():
    """Return semantic-word arrivals, padding markers, and frame timing."""
    next_clear = 3592
    old_clear = next_clear - FRAME_CLOCKS
    old_semantic_bytes = 124 * FRAGMENT_BYTES + 2
    old_prefix_bytes = old_semantic_bytes - 6148
    # Emit the earlier fragments slowly enough to drain; leave 1022 bytes in
    # the next uncommitted fragment, then wait for burst credit to replenish.
    words = [(old_clear + STARTUP_CLOCKS + index * 8, 0)
             for index in range(old_prefix_bytes // 2)]
    assert old_prefix_bytes % FRAGMENT_BYTES == FRAGMENT_BYTES - 2
    # One word completes that fragment; six more full fragments and a
    # two-byte partial tail follow. V3 emits no empty ending descriptor.
    words.extend((cycle, 0) for cycle in range(3074))

    # A new frame begins with zero credit. Quarter-byte integer units model
    # the two-bit-per-clock refill without a floating-point approximation.
    credit = 0
    emitted = 0
    for elapsed in range(1, 10000):
        credit = min(CREDIT_BYTES * 4, credit + 1)
        if elapsed >= STARTUP_CLOCKS and credit >= 8:
            credit -= 8
            words.append((next_clear + elapsed, 1))
            emitted += 2
            if emitted == FRAGMENT_BYTES + 2:
                break
    # A later marker completes frame 1 within its capture interval. This
    # extension is unnecessary to reach the peak but avoids a truncated frame.
    finals = [(3074, 0), (next_clear + 800000, 1)]
    timing = {
        0: {"clear": old_clear, "semantic_bytes": old_semantic_bytes,
            "padding_start": 3074, "physical_end": 3584},
        1: {"clear": next_clear, "semantic_bytes": emitted,
            "padding_start": next_clear + 800000,
            "physical_end": next_clear + 800000 + (FRAME_BYTES - emitted) // 2 - 1},
    }
    return sorted(words), finals, timing


def replay_slots(words, finals, service_clocks=SERVICE_CLOCKS):
    """Unbounded reference queue; arrivals precede same-clock releases.

    Every committed descriptor retires once. A partial writer already owns
    its slot. A padding marker commits a nonempty partial fragment; at an
    exact fragment boundary it allocates nothing, matching UDP v3.
    """
    events = sorted([(cycle, "word", frame) for cycle, frame in words]
                    + [(cycle, "final", frame) for cycle, frame in finals])
    committed = deque()
    partial_bytes = 0
    next_release = None
    releases = []
    high_water = 0
    first_ninth_slot = None

    def release():
        nonlocal next_release
        releases.append(next_release)
        committed.popleft()
        next_release = next_release + service_clocks if committed else None

    for cycle, kind, frame in events:
        while next_release is not None and next_release < cycle:
            release()
        if kind == "word":
            partial_bytes += 2
            if partial_bytes == FRAGMENT_BYTES:
                committed.append((frame, partial_bytes))
                partial_bytes = 0
        elif partial_bytes:
            committed.append((frame, partial_bytes))
            partial_bytes = 0
        if committed and next_release is None:
            next_release = cycle + service_clocks
        occupied = len(committed) + bool(partial_bytes)
        high_water = max(high_water, occupied)
        if occupied > 8 and first_ninth_slot is None:
            first_ninth_slot = cycle
        if next_release == cycle:
            release()
    while next_release is not None:
        release()
    return {"high_water": high_water, "first_ninth_slot": first_ninth_slot,
            "releases": releases}


class CompactImageFifoBoundTests(unittest.TestCase):
    def test_one_isolated_frame_fits_eight_slots_including_partial_tail(self):
        # Start a committed-queue busy period with one complete fragment.
        # The semantic envelope bounds all subsequent words; each retirement
        # before the final descriptor removes a full 1024-byte fragment.
        for retired in range(FRAME_BYTES // FRAGMENT_BYTES):
            elapsed = (retired + 1) * SERVICE_CLOCKS
            maximum_queued_bytes = (FRAGMENT_BYTES + ENVELOPE_BYTES + elapsed // 4
                                    - retired * FRAGMENT_BYTES)
            self.assertLess(maximum_queued_bytes, 8 * FRAGMENT_BYTES)
            # Only the final fragment can be short; v3 adds no empty ending
            # descriptor when semantic data ends at a fragment boundary.
            self.assertLessEqual((maximum_queued_bytes + FRAGMENT_BYTES - 1)
                                 // FRAGMENT_BYTES, 8)

    def test_v3_padding_at_exact_fragment_boundary_allocates_no_slot(self):
        words = [(cycle, 0) for cycle in range(FRAGMENT_BYTES // 2)]
        result = replay_slots(words, [(FRAGMENT_BYTES // 2, 0)])
        self.assertEqual(result["high_water"], 1)
        self.assertEqual(len(result["releases"]), 1)
        empty = replay_slots([], [(0, 0)])
        self.assertEqual(empty["high_water"], 0)
        self.assertEqual(empty["releases"], [])

    def test_v3_maximum_batch_wire_and_paced_round_budgets(self):
        image = 32 + 1024
        children = 32 + 4 * 9 * 8
        poses = 32 + 2 * 22 * 4
        data_udp_bytes = 2 * (image + children + poses)
        self.assertEqual((image, children, poses), (1056, 320, 208))
        self.assertEqual(data_udp_bytes, 3168)
        wire_bytes = data_udp_bytes + 80 + 7 * 66
        self.assertEqual(wire_bytes, 3710)
        # Keep the rational 120/125 clock conversion exact.
        self.assertEqual(wire_bytes * 24, 89040)
        self.assertLess(wire_bytes * 24, SERVICE_CLOCKS * 25)
        # The leaf stress fixture's 4 clocks/32-bit beat is one clock/byte.
        # Planner/lookup run during the 128 ready-low clocks after each packet.
        self.assertEqual(data_udp_bytes + 6 * 128, 3936)
        self.assertLess(data_udp_bytes + 6 * 128, SERVICE_CLOCKS)
        # Each candidate consumes two scan clocks, bounded by a snapshot of
        # one image / four child / two pose records per camera.
        scan_clocks = 2 * 2 * (1 + 4 + 2)
        self.assertEqual(scan_clocks, 28)
        self.assertLess(1 + 2 * 4 + 10, 128)

    def test_materialized_group_drain_allowance(self):
        pair_capacities = (64, 192, 176, 336)
        ram_pairs = 3 * sum(pair_capacities)
        encoder_context_pairs = 16
        replay_pipeline_pairs = 2
        materialized_groups = (ram_pairs + encoder_context_pairs + replay_pipeline_pairs + 7) // 8
        self.assertEqual(ram_pairs, 2304)
        self.assertEqual(materialized_groups, 291)
        self.assertLess(materialized_groups, 300)
        # Existing no-stall group service lemma plus separate initial/final
        # allowances. Zero-run counts serialize as one token, not 256 groups.
        self.assertEqual(300 * 11 + 32 + 32, 3364)
        self.assertLessEqual(300 * 11 + 32 + 32, 4096)

    def test_covered_phase_witnesses_leave_time_to_drain_eight_slots(self):
        profiles = (
            (833334, (942705, 882457, 907744, 882457, 882456, 882457), 14905),
            (828100, (937063, 876815, 902325), 20547),
        )
        for byte_period, witness_offsets, expected_margin in profiles:
            with self.subTest(byte_period=byte_period):
                # At this exact 6:5 clock ratio, the slow period visits five
                # phases; the faster period repeats its tested initial phase.
                phase_count = 5 // gcd(byte_period, 5)
                self.assertGreaterEqual(len(witness_offsets), phase_count + 1)
                steady_phases = {index * byte_period % 5
                                 for index in range(1, phase_count + 1)}
                self.assertEqual(len(steady_phases), phase_count)
                commit_deadline = max(witness_offsets) + 4096
                spare_after_drain = FRAME_CLOCKS - commit_deadline - 8 * SERVICE_CLOCKS
                self.assertEqual(spare_after_drain, expected_margin)
                self.assertGreater(spare_after_drain, 0)

                # The weaker counterexample explicitly violates this added
                # end-phase property, so its nine slots do not refute the
                # conditional fixed-profile result.
                _, finals, timing = boundary_counterexample()
                old_final = dict((frame, cycle) for cycle, frame in finals)[0]
                self.assertGreater(old_final - timing[0]["clear"], commit_deadline)

    def test_counterexample_satisfies_weaker_source_bounds(self):
        words, finals, timing = boundary_counterexample()
        self.assertEqual(len({cycle for cycle, _ in words}), len(words))
        self.assertEqual(timing[1]["clear"] - timing[0]["clear"], FRAME_CLOCKS)
        self.assertEqual(timing[1]["clear"] - timing[0]["physical_end"], 8)
        for frame, details in timing.items():
            emitted = 0
            credit = 0
            previous_cycle = details["clear"]
            for cycle, owner in words:
                if owner != frame:
                    continue
                self.assertGreaterEqual(cycle - details["clear"], STARTUP_CLOCKS)
                credit = min(CREDIT_BYTES * 4, credit + cycle - previous_cycle)
                self.assertGreaterEqual(credit, 8)
                credit -= 8
                emitted += 2
                self.assertLessEqual(emitted * 4, cycle - details["clear"])
                previous_cycle = cycle
            self.assertEqual(emitted, details["semantic_bytes"])
            self.assertLess(details["physical_end"], details["clear"] + FRAME_CLOCKS)
            self.assertEqual(
                emitted + 2 * (details["physical_end"] - details["padding_start"] + 1),
                FRAME_BYTES,
            )
        self.assertEqual(dict((frame, cycle) for cycle, frame in finals),
                         {frame: details["padding_start"] for frame, details in timing.items()})

        # Maximum excess over a quarter-byte/clock server checks every
        # contiguous interval, including intervals crossing the frame clear.
        cumulative_quarters = 0
        minimum_before = None
        maximum_excess = 0
        for cycle, _ in words:
            before = cumulative_quarters - cycle
            minimum_before = before if minimum_before is None else min(minimum_before, before)
            cumulative_quarters += 8
            maximum_excess = max(maximum_excess,
                                 cumulative_quarters - cycle - minimum_before)
        self.assertEqual(maximum_excess, 21519)  # 5379.75 bytes.
        self.assertLessEqual(maximum_excess, ENVELOPE_BYTES * 4)

    def test_stated_bounds_still_allow_nine_slots(self):
        words, finals, _ = boundary_counterexample()
        result = replay_slots(words, finals)
        self.assertEqual(result["high_water"], 9)
        self.assertEqual(result["first_ninth_slot"], 7696)
        self.assertEqual([cycle for cycle in result["releases"] if 0 <= cycle <= 8000],
                         [4000, 8000])


if __name__ == "__main__":
    unittest.main()
