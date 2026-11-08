//! Directed Marlin regression for the causal group encoder's exact-budget tail.
//!
//! The semantic packet stream fills all 1,024,000 frame bits, but the final
//! packet's last physical output word becomes full before the fixed eight-pair
//! service schedule completes. The packer must retain that word until packet
//! 64,000 retires instead of treating it as an early frame end.

mod rtl_support;

use std::collections::BTreeSet;
use std::io::Read;
use std::path::Path;
use std::sync::Mutex;

use flate2::read::GzDecoder;
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};

use rtl_support::{TestResult, read_u64, with_model as with_unserialized_model};

const GROUP_COUNT: usize = 64_000;
const PAIRS_PER_GROUP: usize = 8;
const OUTPUT_WORDS: usize = 64_000;
const RAW_GROUPS: usize = 7_173;
const RAW_CREDIT_GROUPS: usize = RAW_GROUPS + 1;
const EXACT_FILL_GROUP: usize = 347;
const FRAME_BITS: u64 = 1_024_000;
const INITIAL_EXTRA_ROOM: u64 = 960_000;
const RAW_PACKET_EXTRA_BITS: usize = 134;
const DIRECT_ENERGY_LIMIT: u64 = 1_992;
const MAX_INCREMENTAL_ENERGY: u64 = 384;
static RTL_MODEL_LOCK: Mutex<()> = Mutex::new(());

// The production root keeps out_ready asserted.  Before the reserved final
// word, every clock with at least one complete word removes sixteen bits
// before the next token is appended.
fn unstalled_reservoir_step(count: u8, token_bits: u8) -> u8 {
    let after_output = if count >= 16 { count - 16 } else { count };
    after_output + token_bits
}

#[test]
fn all_admissible_packet_token_prefixes_fit_the_unstalled_reservoir() {
    const RESERVOIR_BITS: u8 = 46;
    const MAX_TOKEN_BITS: u8 = 17;
    const MAX_RUN_TOKEN_BITS: u8 = 15;
    // A defensive one-bit packet completes after its three padded header
    // opportunities.  Every selected nonzero packet has three header plus
    // eight pair opportunities and totals 2..=135 bits.
    const PACKET_CLASSES: [(usize, u8, u8); 2] = [(3, 1, 1), (11, 2, 135)];

    // This is a conservative finite-state closure, not a packet corpus.  A
    // selected packet is either the padded three-opportunity, one-bit case or
    // has three header and eight pair-token opportunities, each in 0..=17
    // bits, and 2..=135 total bits.  Before it there may be no run token, or
    // one 1..=15-bit run token.  Allowing every length combination satisfying
    // only those bounds is a strict over-approximation of Huffman,
    // literal-mask, raw, and zero-run traffic.
    let mut packet_boundary_counts = BTreeSet::from([0_u8]);
    let mut reachable_peak = 0_u8;

    loop {
        let mut packet_start_counts = packet_boundary_counts.clone();
        for &count in &packet_boundary_counts {
            for run_bits in 1..=MAX_RUN_TOKEN_BITS {
                let next = unstalled_reservoir_step(count, run_bits);
                assert!(next <= RESERVOIR_BITS);
                reachable_peak = reachable_peak.max(next);
                packet_start_counts.insert(next);
            }
        }

        let mut next_boundary_counts = BTreeSet::new();
        for (token_clocks, minimum_bits, maximum_bits) in PACKET_CLASSES {
            // State is (packet bits admitted so far, reservoir occupancy).
            // Enumerate every prefix, then retain only legal packet totals at
            // the next packet boundary.
            let mut states = packet_start_counts
                .iter()
                .copied()
                .map(|count| (0_u8, count))
                .collect::<BTreeSet<_>>();
            for _ in 0..token_clocks {
                let mut next_states = BTreeSet::new();
                for &(packet_bits, count) in &states {
                    let remaining = maximum_bits - packet_bits;
                    for token_bits in 0..=MAX_TOKEN_BITS.min(remaining) {
                        let next_count = unstalled_reservoir_step(count, token_bits);
                        assert!(
                            next_count <= RESERVOIR_BITS,
                            "reachable token prefix needs {next_count} reservoir bits"
                        );
                        reachable_peak = reachable_peak.max(next_count);
                        next_states.insert((packet_bits + token_bits, next_count));
                    }
                }
                states = next_states;
            }
            next_boundary_counts.extend(
                states.into_iter().filter_map(|(packet_bits, count)| {
                    (packet_bits >= minimum_bits).then_some(count)
                }),
            );
        }
        if next_boundary_counts == packet_boundary_counts {
            break;
        }
        packet_boundary_counts = next_boundary_counts;
    }

    assert_eq!(packet_boundary_counts, (0_u8..=38).collect::<BTreeSet<_>>());
    assert_eq!(reachable_peak, 45);
}

const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("clear_frame", 0, 0, PortDirection::Input),
    ("pair_valid", 0, 0, PortDirection::Input),
    ("pair_ready", 0, 0, PortDirection::Output),
    ("pair_group_start", 0, 0, PortDirection::Input),
    ("pair_group_last", 0, 0, PortDirection::Input),
    ("pair_band", 3, 0, PortDirection::Input),
    ("pair_q0", 7, 0, PortDirection::Input),
    ("pair_q1", 7, 0, PortDirection::Input),
    ("pair_correction", 1, 0, PortDirection::Input),
    ("pair_residual_class", 1, 0, PortDirection::Input),
    ("out_sof", 0, 0, PortDirection::Output),
    ("out_valid", 0, 0, PortDirection::Output),
    ("out_padding", 0, 0, PortDirection::Output),
    ("out_ready", 0, 0, PortDirection::Input),
    ("out_data", 15, 0, PortDirection::Output),
    ("frame_done", 0, 0, PortDirection::Output),
    ("packets_done", 0, 0, PortDirection::Output),
    ("packet_bits", 20, 0, PortDirection::Output),
    ("groups_committed", 15, 0, PortDirection::Output),
    ("groups_serialized", 15, 0, PortDirection::Output),
    ("extra_room", 19, 0, PortDirection::Output),
    ("protocol_error", 0, 0, PortDirection::Output),
];

const DEBUG_PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("debug_output_count", 5, 0, PortDirection::Output),
    ("debug_token_count", 4, 0, PortDirection::Output),
    ("debug_token_valid", 0, 0, PortDirection::Output),
    ("debug_token_ready", 0, 0, PortDirection::Output),
    ("debug_token_fire", 0, 0, PortDirection::Output),
    ("debug_final_fine_bits", 7, 0, PortDirection::Output),
    ("debug_final_coarse_bits", 7, 0, PortDirection::Output),
    ("debug_final_selected_state", 1, 0, PortDirection::Output),
    ("debug_final_selected_bits", 7, 0, PortDirection::Output),
    ("debug_final_selected_huffman", 0, 0, PortDirection::Output),
    ("debug_finalizer_state", 1, 0, PortDirection::Output),
    ("debug_pair_direct_energy0", 10, 0, PortDirection::Output),
    ("debug_pair_direct_energy1", 10, 0, PortDirection::Output),
];

fn with_model<T>(
    artifact_namespace: &str,
    top_module: &str,
    top_source: &str,
    ports: &[(&str, usize, usize, PortDirection)],
    extra_sources: &[&Path],
    test: impl for<'ctx> FnOnce(&mut DynamicVerilatedModel<'ctx>) -> TestResult<T>,
) -> TestResult<T> {
    // Verilator's generated C ABI exposes process-global context symbols.
    // Loading multiple models with the same top module concurrently can
    // interpose those symbols and produce a dangling-context abort. Keep this
    // multi-model test binary deterministic while retaining normal test-harness
    // parallelism for all non-RTL work.
    let _guard = RTL_MODEL_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    with_unserialized_model(
        artifact_namespace,
        top_module,
        top_source,
        ports,
        extra_sources,
        test,
    )
}

#[derive(Default)]
struct Monitor {
    cycles: usize,
    words: usize,
    data: Vec<u16>,
    sof_pulses: usize,
    packets_done_pulses: usize,
    frame_done_pulses: usize,
    packets_done_cycle: Option<usize>,
    frame_done_cycle: Option<usize>,
    debug_enabled: bool,
    max_output_count: u64,
    committed_cycles: Vec<usize>,
    serialized_cycles: Vec<usize>,
    fired_token_counts: Vec<u64>,
    accepted_group_start_cycles: Vec<usize>,
    output_stalled: bool,
    padding_words: usize,
}

impl Monitor {
    fn clock(&mut self, model: &mut DynamicVerilatedModel<'_>) -> TestResult {
        model.pin("clk", 0_u8)?;
        model.eval();

        // out_ready is fixed high, so the value visible immediately before
        // the rising edge is the word transferred by that edge.
        let out_valid = read_u64(model, "out_valid")? != 0;
        let out_padding = read_u64(model, "out_padding")? != 0;
        assert!(!out_padding || out_valid, "padding asserted without a word");
        if out_valid && !self.output_stalled {
            if out_padding {
                assert_eq!(read_u64(model, "out_data")?, 0);
                assert_eq!(read_u64(model, "groups_serialized")?, GROUP_COUNT as u64);
                assert_eq!(
                    self.words - self.padding_words,
                    (read_u64(model, "packet_bits")? as usize).div_ceil(16),
                    "padding must begin immediately after the last semantic word"
                );
                self.padding_words += 1;
            } else {
                assert_eq!(self.padding_words, 0, "semantic word after padding");
            }
            let sof = read_u64(model, "out_sof")? != 0;
            if sof {
                self.sof_pulses += 1;
                assert_eq!(self.words, 0, "out_sof was not on the first word");
            } else {
                assert_ne!(self.words, 0, "first output word omitted out_sof");
            }
            self.words += 1;
            self.data.push(read_u64(model, "out_data")? as u16);
            assert!(
                self.words <= OUTPUT_WORDS,
                "group encoder emitted more than {OUTPUT_WORDS} words"
            );
        } else if !out_valid {
            assert_eq!(
                read_u64(model, "out_sof")?,
                0,
                "out_sof asserted without out_valid"
            );
        }

        if self.debug_enabled {
            let token_valid = read_u64(model, "debug_token_valid")? != 0;
            let token_ready = read_u64(model, "debug_token_ready")? != 0;
            assert!(
                !token_valid || token_ready,
                "packer stalled a valid token on cycle {}: count={} token_count={}",
                self.cycles + 1,
                read_u64(model, "debug_output_count")?,
                read_u64(model, "debug_token_count")?,
            );
            assert_eq!(
                read_u64(model, "debug_token_fire")? != 0,
                token_valid && token_ready,
            );
            if token_valid && token_ready {
                self.fired_token_counts
                    .push(read_u64(model, "debug_token_count")?);
            }
            self.max_output_count = self
                .max_output_count
                .max(read_u64(model, "debug_output_count")?);
        }

        model.pin("clk", 1_u8)?;
        model.eval();
        self.cycles += 1;

        assert_eq!(
            read_u64(model, "protocol_error")?,
            0,
            "causal_group_encoder asserted protocol_error on cycle {}",
            self.cycles
        );
        assert_eq!(
            read_u64(model, "packet_bits")? + read_u64(model, "extra_room")?,
            read_u64(model, "groups_committed")? + INITIAL_EXTRA_ROOM,
            "packet reservation invariant failed on cycle {}",
            self.cycles,
        );
        if read_u64(model, "packets_done")? != 0 {
            self.packets_done_pulses += 1;
            self.packets_done_cycle.get_or_insert(self.cycles);
        }
        if read_u64(model, "frame_done")? != 0 {
            self.frame_done_pulses += 1;
            self.frame_done_cycle.get_or_insert(self.cycles);
        }
        if self.debug_enabled {
            let output_count = read_u64(model, "debug_output_count")?;
            assert!(output_count <= 46, "output reservoir exceeded 46 bits");
            self.max_output_count = self.max_output_count.max(output_count);

            let committed = read_u64(model, "groups_committed")? as usize;
            assert!(committed <= self.committed_cycles.len() + 1);
            if committed == self.committed_cycles.len() + 1 {
                self.committed_cycles.push(self.cycles);
            }
            let serialized = read_u64(model, "groups_serialized")? as usize;
            assert!(serialized <= self.serialized_cycles.len() + 1);
            if serialized == self.serialized_cycles.len() + 1 {
                self.serialized_cycles.push(self.cycles);
            }
        }

        model.pin("clk", 0_u8)?;
        model.eval();
        Ok(())
    }
}

fn reset_model(model: &mut DynamicVerilatedModel<'_>, monitor: &mut Monitor) -> TestResult {
    model.pin("clk", 0_u8)?;
    model.pin("rst", 1_u8)?;
    model.pin("clear_frame", 0_u8)?;
    model.pin("out_ready", 1_u8)?;
    drive_idle(model)?;
    model.eval();
    for _ in 0..4 {
        monitor.clock(model)?;
    }
    model.pin("rst", 0_u8)?;
    Ok(())
}

fn wait_for_committed(
    model: &mut DynamicVerilatedModel<'_>,
    monitor: &mut Monitor,
    groups: u64,
) -> TestResult {
    drive_idle(model)?;
    for _ in 0..32 {
        if read_u64(model, "groups_committed")? == groups {
            return Ok(());
        }
        monitor.clock(model)?;
    }
    Err(format!("group {groups} was not committed").into())
}

fn output_bits(words: &[u16]) -> Vec<u8> {
    words
        .iter()
        .flat_map(|word| (0..16).rev().map(move |bit| ((word >> bit) & 1) as u8))
        .collect()
}

fn drive_idle(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("pair_valid", 0_u8)?;
    model.pin("pair_group_start", 0_u8)?;
    model.pin("pair_group_last", 0_u8)?;
    model.pin("pair_band", 0_u8)?;
    model.pin("pair_q0", 0_u8)?;
    model.pin("pair_q1", 0_u8)?;
    model.pin("pair_correction", 0_u8)?;
    model.pin("pair_residual_class", 0_u8)?;
    Ok(())
}

fn drive_pair(
    model: &mut DynamicVerilatedModel<'_>,
    monitor: &mut Monitor,
    band: u8,
    pair: usize,
    q0: i8,
    q1: i8,
) -> TestResult {
    drive_pair_with_metadata(model, monitor, band, pair, q0, q1, 0, 0)
}

#[allow(clippy::too_many_arguments)]
fn drive_pair_with_metadata(
    model: &mut DynamicVerilatedModel<'_>,
    monitor: &mut Monitor,
    band: u8,
    pair: usize,
    q0: i8,
    q1: i8,
    correction: u8,
    residual_class: u8,
) -> TestResult {
    model.pin("pair_valid", 1_u8)?;
    model.pin("pair_group_start", u8::from(pair == 0))?;
    model.pin("pair_group_last", u8::from(pair == PAIRS_PER_GROUP - 1))?;
    model.pin("pair_band", band)?;
    model.pin("pair_q0", q0 as u8)?;
    model.pin("pair_q1", q1 as u8)?;
    model.pin("pair_correction", correction)?;
    model.pin("pair_residual_class", residual_class)?;

    loop {
        model.eval();
        let accepted = read_u64(model, "pair_ready")? != 0;
        if accepted && pair == 0 {
            monitor.accepted_group_start_cycles.push(monitor.cycles + 1);
        }
        monitor.clock(model)?;
        if accepted {
            return Ok(());
        }
    }
}

fn drive_group(
    model: &mut DynamicVerilatedModel<'_>,
    monitor: &mut Monitor,
    band: u8,
    first_pair: (i8, i8),
) -> TestResult {
    for pair in 0..PAIRS_PER_GROUP {
        let (q0, q1) = if pair == 0 { first_pair } else { (0, 0) };
        drive_pair(model, monitor, band, pair, q0, q1)?;
    }
    Ok(())
}

fn append_control_run(bits: &mut Vec<u8>, length: usize) {
    assert!((15..=256).contains(&length));
    // coarse + Huffman + four zero-nibble codes, then the eight-bit count
    bits.extend([1, 0, 0]);
    bits.extend([0; 4]);
    let count = if length == 256 { 0 } else { length as u8 };
    bits.extend((0..8).rev().map(|bit| (count >> bit) & 1));
}

fn append_raw_delimiter(bits: &mut Vec<u8>) {
    bits.extend([1, 1, 0, 0, 0, 0, 0]);
    for value in [127u8, 127].into_iter().chain(std::iter::repeat_n(0, 14)) {
        bits.extend((0..8).rev().map(|bit| (value >> bit) & 1));
    }
}

fn decode_base64(text: &str) -> Vec<u8> {
    let mut output = Vec::with_capacity(text.len() * 3 / 4);
    let mut accumulator = 0_u32;
    let mut bits = 0_usize;
    for byte in text.bytes().filter(|byte| !byte.is_ascii_whitespace()) {
        if byte == b'=' {
            break;
        }
        let value = match byte {
            b'A'..=b'Z' => byte - b'A',
            b'a'..=b'z' => byte - b'a' + 26,
            b'0'..=b'9' => byte - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => panic!("invalid base64 byte in raw group schedule"),
        };
        accumulator = (accumulator << 6) | u32::from(value);
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            output.push((accumulator >> bits) as u8);
            accumulator &= (1_u32 << bits).wrapping_sub(1);
        }
    }
    output
}

fn canonical_group_schedule() -> TestResult<Vec<u8>> {
    let compressed = decode_base64(include_str!(
        "../../../../software/crates/lwc/src/raw_group_schedule.b64"
    ));
    let mut decoder = GzDecoder::new(compressed.as_slice());
    let mut text = String::new();
    decoder.read_to_string(&mut text)?;
    let schedule = text
        .lines()
        .map(str::parse::<u8>)
        .collect::<Result<Vec<_>, _>>()?;
    assert_eq!(schedule.len(), GROUP_COUNT);
    assert_eq!(
        schedule.last(),
        Some(&1),
        "counterexample needs final band 1"
    );
    Ok(schedule)
}

fn nominal_group_bits(band: u8) -> usize {
    const NOMINAL: [usize; 12] = [139, 135, 102, 58, 59, 43, 27, 30, 21, 10, 13, 5];
    NOMINAL[usize::from(band)]
}

fn threshold_multiplier(cutoff: u8) -> u64 {
    match cutoff {
        1 => 1,
        2 | 3 => 2,
        4 => 3,
        5 => 4,
        6 => 6,
        7 => 8,
        8 => 12,
        _ => 0,
    }
}

fn reference_gain_passes(energy: u64, extra: u64, cutoff: u8, shift: u8) -> bool {
    if cutoff == 0 {
        true
    } else if extra == 0 {
        energy != 0
    } else {
        (energy << shift) >= extra * threshold_multiplier(cutoff)
    }
}

#[test]
fn rtl_gain_admission_preserves_every_rounding_boundary() -> TestResult {
    const ADAPTER: &str = "tests/rtl_adapters/gain_cutoff_adapter.sv";
    const GAIN_PORTS: &[(&str, usize, usize, PortDirection)] = &[
        ("energy", 10, 0, PortDirection::Input),
        ("extra_bits", 7, 0, PortDirection::Input),
        ("cutoff", 3, 0, PortDirection::Input),
        ("metric_shift", 1, 0, PortDirection::Input),
        ("passes", 0, 0, PortDirection::Output),
    ];
    with_model(
        "gain_cutoff_boundaries",
        "gain_cutoff_adapter",
        ADAPTER,
        GAIN_PORTS,
        &[Path::new(ADAPTER)],
        |model| {
            // Test the actual RTL against the integer rate/quality contract.
            // Every extra-bit count and band shift visits both sides of its
            // exact admission boundary, including the zero-cost exception.
            for cutoff in 0..=8_u8 {
                model.pin("cutoff", cutoff)?;
                for extra in 0..=u8::MAX {
                    model.pin("extra_bits", extra)?;
                    for shift in 0..=3_u8 {
                        model.pin("metric_shift", shift)?;
                        let required = (u64::from(extra) * threshold_multiplier(cutoff))
                            .div_ceil(1_u64 << shift);
                        let energies = [
                            0,
                            1,
                            required.saturating_sub(1),
                            required,
                            required + 1,
                            DIRECT_ENERGY_LIMIT,
                            2047,
                        ];
                        for energy in energies.into_iter().filter(|&value| value <= 2047) {
                            model.pin("energy", energy)?;
                            model.eval();
                            assert_eq!(
                                read_u64(model, "passes")? != 0,
                                reference_gain_passes(energy, u64::from(extra), cutoff, shift),
                                "energy={energy} extra={extra} cutoff={cutoff} shift={shift}",
                            );
                        }
                    }
                }
            }
            Ok(())
        },
    )
}

fn clipped_gain_passes(energy: u64, extra: u64, cutoff: u8, shift: u8) -> bool {
    if cutoff == 0 {
        true
    } else if extra == 0 {
        energy != 0
    } else {
        let scaled = extra * threshold_multiplier(cutoff);
        let required = (scaled + (1_u64 << shift) - 1) >> shift;
        energy.min(DIRECT_ENERGY_LIMIT) >= required
    }
}

fn reference_direct_energy(value: i8, correction: bool, residual_class: bool) -> u64 {
    let magnitude = u64::from(i16::from(value).unsigned_abs());
    if magnitude >= 17 {
        DIRECT_ENERGY_LIMIT
    } else if magnitude == 0 {
        0
    } else {
        let coefficient = if magnitude & 1 == 0 {
            4
        } else if residual_class {
            6
        } else {
            2
        };
        let assists = if magnitude & 1 != 0 {
            correction
        } else {
            residual_class
        };
        let factor = if assists {
            8 * magnitude + coefficient
        } else {
            8 * magnitude - coefficient
        };
        (magnitude * factor).min(DIRECT_ENERGY_LIMIT)
    }
}

#[test]
fn bounded_direct_energy_table_matches_formula_exhaustively() -> TestResult {
    const ADAPTER: &str = "tests/rtl_adapters/causal_group_encoder_debug_adapter.sv";
    let mut ports = PORTS.to_vec();
    ports.extend_from_slice(DEBUG_PORTS);

    with_model(
        "causal_group_encoder_direct_energy",
        "causal_group_encoder_debug_adapter",
        ADAPTER,
        &ports,
        &[Path::new(ADAPTER)],
        |model| {
            let mut monitor = Monitor::default();
            reset_model(model, &mut monitor)?;
            let mut pair = 0;

            for value in i8::MIN..=i8::MAX {
                for correction in [false, true] {
                    for residual_class in [false, true] {
                        // The energy lookup is synchronous and accepts the
                        // same transfers as the collector. Check the captured
                        // values after each real accepted pair, including
                        // group-boundary stalls.
                        drive_pair_with_metadata(
                            model,
                            &mut monitor,
                            0,
                            pair,
                            value,
                            value,
                            if correction { 3 } else { 0 },
                            if residual_class { 3 } else { 0 },
                        )?;
                        pair = (pair + 1) % PAIRS_PER_GROUP;

                        let expected = reference_direct_energy(value, correction, residual_class);
                        assert_eq!(
                            read_u64(model, "debug_pair_direct_energy0")?,
                            expected,
                            "lane 0 value={value} correction={correction} \
                             residual_class={residual_class}",
                        );
                        assert_eq!(
                            read_u64(model, "debug_pair_direct_energy1")?,
                            expected,
                            "lane 1 value={value} correction={correction} \
                             residual_class={residual_class}",
                        );
                    }
                }
            }
            Ok(())
        },
    )
}

#[test]
fn saturated_energy_preserves_every_hardware_bounded_decision() {
    // Below the clip point this exhausts every energy, cutoff, packet excess,
    // and band shift.  LIMIT+1 is the only additional direct boundary needed:
    // every larger value has the same clipped value and is already above the
    // largest possible selected-packet 134*12 requirement. The 147-bit coarse
    // raw candidate is always dominated by the 135-bit fine raw candidate and
    // therefore cannot reach this admission test as the selected packet.
    for cutoff in 0..=8 {
        for extra in 0..=134 {
            for shift in 0..=3 {
                for energy in 0..=DIRECT_ENERGY_LIMIT + 1 {
                    assert_eq!(
                        clipped_gain_passes(energy, extra, cutoff, shift),
                        reference_gain_passes(energy, extra, cutoff, shift),
                        "energy={energy} extra={extra} cutoff={cutoff} shift={shift}",
                    );
                }
            }
        }
    }

    // Coarse gain subtracts incremental energy after direct energy clips.
    // The worst legal subtraction is 384, leaving 1992-384=1608 at the
    // clipped boundary.  Exhaust every controller input and subtraction at
    // the first clipped direct value; larger direct values are monotone.
    for cutoff in 0..=8 {
        for extra in 0..=134 {
            for shift in 0..=3 {
                for incremental in 0..=MAX_INCREMENTAL_ENERGY {
                    let unclipped = DIRECT_ENERGY_LIMIT + 1 - incremental;
                    let clipped = DIRECT_ENERGY_LIMIT - incremental;
                    assert_eq!(
                        reference_gain_passes(clipped, extra, cutoff, shift),
                        reference_gain_passes(unclipped, extra, cutoff, shift),
                        "incremental={incremental} extra={extra} cutoff={cutoff} \
                         shift={shift}",
                    );
                }
            }
        }
    }
}

#[test]
fn band1_zero_context_uses_retuned_k2() -> TestResult {
    with_model(
        "causal_group_encoder_band1_zero_context",
        "causal_group_encoder",
        "rtl/lwc/causal_group_encoder.sv",
        PORTS,
        &[],
        |model| {
            let mut monitor = Monitor::default();
            reset_model(model, &mut monitor)?;

            for group in 0..2 {
                for pair in 0..PAIRS_PER_GROUP {
                    // Band 1 maps fine q=2 to coarse q=1. The first lane uses
                    // reset context k=2; every following lane sees magnitude
                    // one and uses small-context k=1.
                    drive_pair(model, &mut monitor, 1, pair, 2, 2)?;
                }
                wait_for_committed(model, &mut monitor, group + 1)?;
                assert_eq!(
                    read_u64(model, "packet_bits")?,
                    64 * (group + 1),
                    "band-1 zero context did not use the retuned k=2",
                );
            }

            drive_idle(model)?;
            for _ in 0..32 {
                if monitor.data.len() >= 8 && read_u64(model, "groups_serialized")? == 2 {
                    break;
                }
                monitor.clock(model)?;
            }
            assert_eq!(read_u64(model, "groups_serialized")?, 2);

            let mut packet = vec![1, 0, 0];
            for _ in 0..4 {
                packet.extend([1, 0, 0]);
            }
            packet.extend([0, 1, 0, 0]);
            for _ in 1..2 * PAIRS_PER_GROUP {
                packet.extend([0, 1, 0]);
            }
            assert_eq!(packet.len(), 64);
            let bits = output_bits(&monitor.data);
            assert_eq!(&bits[..64], packet);
            assert_eq!(&bits[64..128], packet);
            Ok(())
        },
    )
}

#[test]
fn packet_local_rice_context_resets_and_serializes_exactly() -> TestResult {
    with_model(
        "causal_group_encoder_rice_context",
        "causal_group_encoder",
        "rtl/lwc/causal_group_encoder.sv",
        PORTS,
        &[],
        |model| {
            let mut monitor = Monitor::default();
            reset_model(model, &mut monitor)?;

            for group in 0..2 {
                for pair in 0..PAIRS_PER_GROUP {
                    let (q0, q1) = match pair {
                        0 => (6, 0),
                        7 => (0, 6),
                        _ => (0, 0),
                    };
                    drive_pair(model, &mut monitor, 3, pair, q0, q1)?;
                }
                wait_for_committed(model, &mut monitor, group + 1)?;
                assert_eq!(
                    read_u64(model, "packet_bits")?,
                    21 * (group + 1),
                    "the identical band-3 coarse packet changed length across a group reset",
                );
            }

            // Two state-zero groups remain pending until a selected nonzero
            // group supplies their causal run boundary.
            for group in 2..4 {
                for pair in 0..PAIRS_PER_GROUP {
                    drive_pair(model, &mut monitor, 3, pair, 0, 0)?;
                }
                wait_for_committed(model, &mut monitor, group + 1)?;
            }
            for pair in 0..PAIRS_PER_GROUP {
                let (q0, q1) = match pair {
                    0 => (6, 0),
                    7 => (0, 6),
                    _ => (0, 0),
                };
                drive_pair(model, &mut monitor, 3, pair, q0, q1)?;
            }
            wait_for_committed(model, &mut monitor, 5)?;
            drive_idle(model)?;
            for _ in 0..32 {
                if monitor.data.len() >= 3 && read_u64(model, "groups_serialized")? == 5 {
                    break;
                }
                monitor.clock(model)?;
            }
            assert!(monitor.data.len() >= 3, "context packets did not drain");

            // Header: nonzero/coarse/Huffman, nibble masks 1/0/0/8.
            // Both nonzero coarse magnitudes are three and follow a zero lane.
            // Band 3 therefore selects its zero-context k=1 and emits 0010.
            // The identical second packet proves that context resets at the
            // group boundary instead of leaking from a preceding packet.
            let expected = [
                1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0,
            ];
            let bits = output_bits(&monitor.data);
            assert_eq!(&bits[..21], &expected);
            assert_eq!(&bits[21..42], &expected);
            assert_eq!(read_u64(model, "groups_serialized")?, 5);
            Ok(())
        },
    )
}

#[test]
fn zero_run_boundaries_are_exact_and_single_append_bounded() -> TestResult {
    with_model(
        "causal_group_encoder_zero_runs",
        "causal_group_encoder",
        "rtl/lwc/causal_group_encoder.sv",
        PORTS,
        &[],
        |model| {
            let mut monitor = Monitor::default();
            reset_model(model, &mut monitor)?;
            let mut expected = Vec::new();
            let mut total_groups = 0usize;
            for length in [1usize, 14, 15, 16, 255, 256, 257, 1070] {
                for _ in 0..length {
                    drive_group(model, &mut monitor, 11, (0, 0))?;
                }
                drive_group(model, &mut monitor, 11, (127, 127))?;
                let mut remaining = length;
                while remaining >= 256 {
                    append_control_run(&mut expected, 256);
                    remaining -= 256;
                }
                if remaining >= 15 {
                    append_control_run(&mut expected, remaining);
                } else {
                    expected.extend(std::iter::repeat_n(0, remaining));
                }
                append_raw_delimiter(&mut expected);
                total_groups += length + 1;
            }
            drive_idle(model)?;
            for _ in 0..128 {
                monitor.clock(model)?;
                if read_u64(model, "groups_serialized")? == total_groups as u64 {
                    break;
                }
            }
            for _ in 0..4 {
                monitor.clock(model)?;
            }
            let actual = output_bits(&monitor.data);
            let complete_bits = expected.len() / 16 * 16;
            assert_eq!(actual.len(), complete_bits);
            assert_eq!(&actual[..], &expected[..complete_bits]);
            assert_eq!(read_u64(model, "groups_committed")?, total_groups as u64);
            assert_eq!(read_u64(model, "groups_serialized")?, total_groups as u64);
            assert_eq!(read_u64(model, "packet_bits")?, expected.len() as u64);
            let max_group_start_spacing = monitor
                .accepted_group_start_cycles
                .windows(2)
                .map(|cycles| cycles[1] - cycles[0])
                .max()
                .unwrap_or(0);
            assert!(
                max_group_start_spacing <= 11,
                "mixed zero-run/nonzero service took {max_group_start_spacing} clocks between accepted group starts",
            );
            Ok(())
        },
    )
}

#[test]
fn alternating_groups_never_expand_and_preserve_packet_order() -> TestResult {
    with_model(
        "causal_group_encoder_alternating",
        "causal_group_encoder",
        "rtl/lwc/causal_group_encoder.sv",
        PORTS,
        &[],
        |model| {
            const PAIRS: usize = 512;
            let mut monitor = Monitor::default();
            reset_model(model, &mut monitor)?;
            let mut expected = Vec::with_capacity(PAIRS * 136);
            for _ in 0..PAIRS {
                drive_group(model, &mut monitor, 11, (0, 0))?;
                drive_group(model, &mut monitor, 11, (127, 127))?;
                expected.push(0);
                append_raw_delimiter(&mut expected);
            }
            drive_idle(model)?;
            for _ in 0..128 {
                monitor.clock(model)?;
                if read_u64(model, "groups_serialized")? == (2 * PAIRS) as u64 {
                    break;
                }
            }
            for _ in 0..4 {
                monitor.clock(model)?;
            }
            let actual = output_bits(&monitor.data);
            assert_eq!(actual, expected);
            assert_eq!(read_u64(model, "packet_bits")?, expected.len() as u64);
            assert_eq!(read_u64(model, "groups_serialized")?, (2 * PAIRS) as u64);
            Ok(())
        },
    )
}

#[test]
fn run_and_packet_tokens_stall_cleanly_at_the_46_bit_reservoir() -> TestResult {
    with_model(
        "causal_group_encoder_run_backpressure",
        "causal_group_encoder",
        "rtl/lwc/causal_group_encoder.sv",
        PORTS,
        &[],
        |model| {
            let mut monitor = Monitor::default();
            reset_model(model, &mut monitor)?;
            model.pin("out_ready", 0_u8)?;
            monitor.output_stalled = true;
            for _ in 0..20 {
                drive_group(model, &mut monitor, 11, (0, 0))?;
            }
            drive_group(model, &mut monitor, 11, (127, 127))?;
            drive_idle(model)?;
            for _ in 0..32 {
                monitor.clock(model)?;
            }
            assert_eq!(monitor.words, 0);
            assert_eq!(read_u64(model, "groups_committed")?, 21);
            assert!(read_u64(model, "groups_serialized")? < 21);
            model.pin("out_ready", 1_u8)?;
            monitor.output_stalled = false;
            for _ in 0..128 {
                monitor.clock(model)?;
                if read_u64(model, "groups_serialized")? == 21 {
                    break;
                }
            }
            for _ in 0..4 {
                monitor.clock(model)?;
            }
            let mut expected = Vec::new();
            append_control_run(&mut expected, 20);
            append_raw_delimiter(&mut expected);
            let actual = output_bits(&monitor.data);
            let complete_bits = expected.len() / 16 * 16;
            assert_eq!(actual, expected[..complete_bits]);
            assert_eq!(read_u64(model, "packet_bits")?, expected.len() as u64);
            assert_eq!(read_u64(model, "groups_serialized")?, 21);
            Ok(())
        },
    )
}

#[test]
fn residual_long_run_flushes_at_the_exact_frame_end() -> TestResult {
    with_model(
        "causal_group_encoder_frame_end_run",
        "causal_group_encoder",
        "rtl/lwc/causal_group_encoder.sv",
        PORTS,
        &[],
        |model| {
            const FINAL_RUN: usize = 1070;
            const PREFIX: usize = GROUP_COUNT - FINAL_RUN;
            let mut monitor = Monitor::default();
            reset_model(model, &mut monitor)?;
            for group in 0..PREFIX {
                let raw = group % 33 == 0 || group + 1 == PREFIX;
                drive_group(
                    model,
                    &mut monitor,
                    11,
                    if raw { (127, 127) } else { (0, 0) },
                )?;
            }
            drive_idle(model)?;
            for _ in 0..128 {
                monitor.clock(model)?;
                if read_u64(model, "groups_serialized")? == PREFIX as u64 {
                    break;
                }
            }
            let prefix_bits = read_u64(model, "packet_bits")?;
            for _ in 0..FINAL_RUN {
                drive_group(model, &mut monitor, 11, (0, 0))?;
            }
            drive_idle(model)?;
            for _ in 0..256 {
                monitor.clock(model)?;
                if read_u64(model, "groups_serialized")? == GROUP_COUNT as u64 {
                    break;
                }
            }
            assert_eq!(read_u64(model, "groups_committed")?, GROUP_COUNT as u64);
            assert_eq!(read_u64(model, "groups_serialized")?, GROUP_COUNT as u64);
            assert_eq!(read_u64(model, "packet_bits")?, prefix_bits + 75);
            assert_eq!(monitor.packets_done_pulses, 1);
            Ok(())
        },
    )
}

#[test]
fn seventeen_bit_pairs_respect_the_stream_bound_without_stalls() -> TestResult {
    const ADAPTER: &str = "tests/rtl_adapters/causal_group_encoder_debug_adapter.sv";
    let mut ports = PORTS.to_vec();
    ports.extend_from_slice(DEBUG_PORTS);

    with_model(
        "causal_group_encoder_pair17_stream_bound",
        "causal_group_encoder_debug_adapter",
        ADAPTER,
        &ports,
        &[Path::new(ADAPTER)],
        |model| {
            // Both schedules are real k=0 band-8 Rice packets. The first
            // tail-loads six 17-bit pairs and leaves 37 queued bits. The ROM
            // setup clocks drain output words before the second packet,
            // so front-loading that packet returns to the same 37-bit peak.
            // Corrections make the fine candidates win the causal
            // incremental-gain test without changing their tokens.
            const GROUPS: [[(i8, i8); PAIRS_PER_GROUP]; 2] = [
                [
                    (0, 0),
                    (5, 5),
                    (16, 0),
                    (16, 0),
                    (16, 0),
                    (16, 0),
                    (16, 0),
                    (16, 0),
                ],
                [
                    (16, 0),
                    (16, 0),
                    (16, 0),
                    (16, 0),
                    (16, 0),
                    (16, 0),
                    (0, 0),
                    (0, 0),
                ],
            ];
            const EXPECTED_TOKENS: [u64; 22] = [
                3, 8, 8, 0, 12, 17, 17, 17, 17, 17, 17, 3, 8, 8, 17, 17, 17, 17, 17, 17, 0, 0,
            ];

            // A selected Huffman packet performs two eighteen-bit single-port
            // ROM reads after its eight input pairs. The next group's first pair
            // must still be accepted eleven clocks after the prior first
            // pair, and the two differing mask bytes must reconstruct the
            // same 21-bit packet as the software reference.
            let mut huffman_monitor = Monitor::default();
            reset_model(model, &mut huffman_monitor)?;
            let mut huffman_group_start_cycles = Vec::new();
            for _ in 0..2 {
                for pair in 0..PAIRS_PER_GROUP {
                    let (q0, q1) = match pair {
                        0 => (6, 0),
                        7 => (0, 6),
                        _ => (0, 0),
                    };
                    drive_pair(model, &mut huffman_monitor, 3, pair, q0, q1)?;
                    if pair == 0 {
                        huffman_group_start_cycles.push(huffman_monitor.cycles);
                    }
                }
                assert_eq!(read_u64(model, "debug_finalizer_state")?, 1);
                huffman_monitor.clock(model)?;
                assert_eq!(read_u64(model, "debug_finalizer_state")?, 2);
                assert_eq!(read_u64(model, "debug_final_selected_huffman")?, 1);
                assert_eq!(read_u64(model, "debug_final_selected_bits")?, 21);
            }
            assert_eq!(huffman_group_start_cycles.len(), 2);
            assert_eq!(
                huffman_group_start_cycles[1] - huffman_group_start_cycles[0],
                11,
                "serialized Huffman ROM did not sustain one group per eleven clocks",
            );
            drive_idle(model)?;
            for _ in 0..32 {
                if read_u64(model, "groups_serialized")? == 2 {
                    break;
                }
                huffman_monitor.clock(model)?;
            }
            assert_eq!(read_u64(model, "groups_committed")?, 2);
            assert_eq!(read_u64(model, "groups_serialized")?, 2);
            assert_eq!(read_u64(model, "packet_bits")?, 42);

            // Exercise the hard fallback first in the same model. Both
            // entropy candidates overflow, but fine raw is twelve bits
            // shorter and therefore dominates before the gain decision.
            let mut raw_monitor = Monitor::default();
            reset_model(model, &mut raw_monitor)?;
            drive_group(model, &mut raw_monitor, 8, (127, 127))?;
            drive_idle(model)?;
            model.eval();
            assert_eq!(read_u64(model, "debug_final_fine_bits")?, 135);
            assert_eq!(read_u64(model, "debug_final_coarse_bits")?, 147);
            raw_monitor.clock(model)?;
            assert_eq!(read_u64(model, "debug_finalizer_state")?, 2);
            assert_eq!(read_u64(model, "debug_final_selected_state")?, 2);
            assert_eq!(read_u64(model, "debug_final_selected_bits")?, 135);
            for _ in 0..32 {
                if read_u64(model, "groups_serialized")? == 1 {
                    break;
                }
                raw_monitor.clock(model)?;
            }
            for _ in 0..4 {
                raw_monitor.clock(model)?;
            }
            let mut expected = Vec::new();
            append_raw_delimiter(&mut expected);
            assert_eq!(output_bits(&raw_monitor.data), expected[..128]);
            assert_eq!(read_u64(model, "packet_bits")?, 135);
            assert_eq!(read_u64(model, "groups_serialized")?, 1);

            let mut monitor = Monitor {
                debug_enabled: true,
                ..Monitor::default()
            };
            reset_model(model, &mut monitor)?;
            let mut group_start_cycles = Vec::new();
            for pairs in GROUPS {
                for (pair, (q0, q1)) in pairs.into_iter().enumerate() {
                    let correction = u8::from(q0 != 0) | (u8::from(q1 != 0) << 1);
                    drive_pair_with_metadata(model, &mut monitor, 8, pair, q0, q1, correction, 0)?;
                    if pair == 0 {
                        group_start_cycles.push(monitor.cycles);
                    }
                }
                assert_eq!(read_u64(model, "debug_finalizer_state")?, 1);
                monitor.clock(model)?;
                assert_eq!(read_u64(model, "debug_finalizer_state")?, 2);
                assert_eq!(read_u64(model, "debug_final_selected_huffman")?, 0);
            }

            drive_idle(model)?;
            for _ in 0..32 {
                if read_u64(model, "groups_serialized")? == 2 {
                    break;
                }
                monitor.clock(model)?;
            }

            assert_eq!(read_u64(model, "groups_committed")?, 2);
            assert_eq!(read_u64(model, "groups_serialized")?, 2);
            assert_eq!(read_u64(model, "packet_bits")?, 133 + 121);
            assert_eq!(monitor.fired_token_counts, EXPECTED_TOKENS);
            assert_eq!(monitor.max_output_count, 37);
            assert_eq!(group_start_cycles.len(), 2);
            assert_eq!(
                group_start_cycles[1] - group_start_cycles[0],
                11,
                "raw-mask packet did not sustain one group per eleven clocks",
            );
            assert_eq!(monitor.committed_cycles.len(), 2);
            assert_eq!(monitor.serialized_cycles.len(), 2);
            for group in 0..2 {
                assert_eq!(
                    monitor.serialized_cycles[group] - monitor.committed_cycles[group],
                    12,
                    "nonzero packet did not use two decision, three header, and eight pair clocks",
                );
            }
            Ok(())
        },
    )
}

#[test]
fn pure_padding_marker_excludes_the_final_partial_semantic_word() -> TestResult {
    with_model(
        "causal_group_encoder_padding_marker",
        "causal_group_encoder",
        "rtl/lwc/causal_group_encoder.sv",
        PORTS,
        &[],
        |model| {
            let mut monitor = Monitor::default();
            reset_model(model, &mut monitor)?;
            for _ in 0..GROUP_COUNT {
                drive_group(model, &mut monitor, 11, (0, 0))?;
            }
            drive_idle(model)?;
            for _ in 0..OUTPUT_WORDS + 32 {
                monitor.clock(model)?;
                if monitor.frame_done_pulses != 0 {
                    break;
                }
            }
            // The all-zero stream transitions from run tokens to literal
            // one-bit packets when its negative rate debt reaches cutoff
            // zero. Its final semantic word remains partial and unmarked.
            let semantic_bits = read_u64(model, "packet_bits")? as usize;
            assert_eq!(semantic_bits, 51_517);
            assert_ne!(semantic_bits % 16, 0);
            assert_eq!(monitor.words, OUTPUT_WORDS);
            assert_eq!(
                monitor.padding_words,
                OUTPUT_WORDS - semantic_bits.div_ceil(16)
            );
            assert_eq!(monitor.frame_done_pulses, 1);
            assert_eq!(read_u64(model, "out_padding")?, 0);
            Ok(())
        },
    )
}

#[test]
fn exact_budget_tail_waits_for_packet_completion() -> TestResult {
    with_model(
        "causal_group_encoder_exact_budget",
        "causal_group_encoder",
        "rtl/lwc/causal_group_encoder.sv",
        PORTS,
        &[],
        |model| {
            let schedule = canonical_group_schedule()?;
            assert_eq!(schedule[EXACT_FILL_GROUP], 11);
            let mut monitor = Monitor::default();
            let nominal_credit = schedule[..GROUP_COUNT - 1]
                .iter()
                .map(|&band| nominal_group_bits(band) - 1)
                .sum::<usize>();
            // Leave one complete raw-packet credit in the test scheduler.
            // A directed 19-bit packet consumes the otherwise spare eighteen
            // bits, then the final 135-bit raw packet consumes the exact
            // 134-bit physical remainder beyond its one-bit reservation.
            let target_raw_credit = RAW_CREDIT_GROUPS * RAW_PACKET_EXTRA_BITS;
            let distributed_deficit = nominal_credit - target_raw_credit;
            let credit_span = GROUP_COUNT - 1;
            let mut raw_credit = 0_isize;
            let mut raw_groups = 0_usize;
            let mut zero_run_length = 0_usize;
            let mut credited_zero_groups = 0_usize;

            model.pin("clk", 0_u8)?;
            model.pin("rst", 1_u8)?;
            model.pin("clear_frame", 0_u8)?;
            model.pin("out_ready", 1_u8)?;
            drive_idle(model)?;
            model.eval();
            for _ in 0..4 {
                monitor.clock(model)?;
            }
            model.pin("rst", 0_u8)?;

            for (group, &band) in schedule.iter().enumerate() {
                // Keep rate debt close to zero while constructing the exact
                // semantic frame. The nominal profile supplies slightly more
                // raw-packet credit than the exact frame needs; spread that
                // deficit evenly over non-final groups so debt stays close
                // to zero instead of creating an artificial high-debt prefix.
                let raw_group = if group < credit_span {
                    let deficit_before = (group * distributed_deficit).div_ceil(credit_span);
                    let deficit_after = ((group + 1) * distributed_deficit).div_ceil(credit_span);
                    raw_credit += (nominal_group_bits(band) - 1) as isize
                        - (deficit_after - deficit_before) as isize;
                    if raw_credit >= RAW_PACKET_EXTRA_BITS as isize {
                        raw_credit -= RAW_PACKET_EXTRA_BITS as isize;
                        raw_groups += 1;
                        true
                    } else {
                        false
                    }
                } else {
                    false
                };
                let exact_fill_group = group == EXACT_FILL_GROUP;
                if exact_fill_group {
                    assert!(!raw_group);
                }
                if group < credit_span {
                    if raw_group || exact_fill_group {
                        zero_run_length = 0;
                    } else {
                        if zero_run_length >= 15 {
                            credited_zero_groups += 1;
                        }
                        zero_run_length += 1;
                        if zero_run_length == 256 {
                            zero_run_length = 0;
                        }
                    }
                }
                for pair in 0..PAIRS_PER_GROUP {
                    let (q0, q1, correction) = if raw_group && pair == 0 {
                        // Two maximum positive values exceed the sixteen-bit
                        // pair-token bound for both fine and coarse in every
                        // band, selecting the 135-bit raw packet.
                        (127, 127, 0)
                    } else if exact_fill_group && pair == 0 {
                        // Band 11 maps fine magnitude 16 to coarse magnitude
                        // eight. Its single-lane coarse packet is 19 bits,
                        // replacing one literal zero bit and filling the
                        // context profile's exact 18-bit semantic gap.
                        (16, 0, 0)
                    } else if group == GROUP_COUNT - 1 {
                        // One deliberately long fine pair forces the 135-bit
                        // raw escape. The coarse peer remains bounded at 118
                        // bits, while fourteen even corrected coefficients
                        // supply enough incremental gain to select fine even
                        // at the maximum causal cutoff.
                        if pair == 1 { (97, 1, 0) } else { (16, 16, 3) }
                    } else {
                        (0, 0, 0)
                    };
                    drive_pair_with_metadata(
                        model,
                        &mut monitor,
                        band,
                        pair,
                        q0,
                        q1,
                        correction,
                        0,
                    )?;
                }
            }

            assert_eq!(raw_groups, RAW_GROUPS);
            assert_eq!(raw_credit, RAW_PACKET_EXTRA_BITS as isize);

            drive_idle(model)?;
            for _ in 0..32 {
                monitor.clock(model)?;
                if monitor.frame_done_pulses != 0 {
                    break;
                }
            }
            assert_eq!(
                monitor.frame_done_pulses,
                1,
                "frame did not complete after the final group: packet_bits={}, \
                 extra_room={}, committed={}, serialized={}, words={}",
                read_u64(model, "packet_bits")?,
                read_u64(model, "extra_room")?,
                read_u64(model, "groups_committed")?,
                read_u64(model, "groups_serialized")?,
                monitor.words,
            );

            // Continue past completion to reject a repeated tail word or
            // repeated completion pulse.
            for _ in 0..16 {
                monitor.clock(model)?;
            }

            assert_eq!(monitor.words, OUTPUT_WORDS);
            assert_eq!(monitor.sof_pulses, 1);
            assert_eq!(monitor.packets_done_pulses, 1);
            assert_eq!(monitor.frame_done_pulses, 1);
            assert!(
                monitor.packets_done_cycle < monitor.frame_done_cycle,
                "physical frame completion did not follow packet completion"
            );
            assert_eq!(read_u64(model, "groups_committed")?, GROUP_COUNT as u64);
            assert_eq!(read_u64(model, "groups_serialized")?, GROUP_COUNT as u64);
            assert_eq!(read_u64(model, "packet_bits")?, FRAME_BITS);
            assert_eq!(monitor.padding_words, 0);
            assert_eq!(read_u64(model, "extra_room")?, 0);
            assert!(credited_zero_groups > 0);
            assert_eq!(read_u64(model, "out_valid")?, 0);
            Ok(())
        },
    )
}
