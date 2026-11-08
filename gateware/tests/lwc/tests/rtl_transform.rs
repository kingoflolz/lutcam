//! Marlin checks for the production 5H/3V forward wavelet transform.
//!
//! A complete raster frame is compared coefficient-for-coefficient against an
//! independent Rust CDF 5/3 model.  The same trace freezes the pair-write
//! conflict census used to size and bank the downstream coefficient queues.

mod rtl_support;

use std::path::Path;

use lwc::{FRAME_HEIGHT, FRAME_PIXELS, FRAME_WIDTH, raw_group_schedule};
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};

use rtl_support::{TestResult, clock, read_u64, sign_extend, with_model};

const ACTIVE_CLOCKS_PER_ROW: usize = FRAME_WIDTH / 2;
const HORIZONTAL_BLANK_CLOCKS: usize = 256;
const MAX_DRAIN_CLOCKS: usize = 2_000_000;
const QUIET_CLOCKS: usize = 4;

const ADAPTER: &str = "tests/rtl_adapters/forward_wavelet_transform_adapter.sv";

const TOP_PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst_n", 0, 0, PortDirection::Input),
    ("in_sof", 0, 0, PortDirection::Input),
    ("in_valid", 0, 0, PortDirection::Input),
    ("in_px0", 7, 0, PortDirection::Input),
    ("in_px1", 7, 0, PortDirection::Input),
    ("out_raw_valid", 11, 0, PortDirection::Output),
    ("out_raw_value0", 13, 0, PortDirection::Output),
    ("out_raw_value1", 13, 0, PortDirection::Output),
    ("out_raw_value2", 13, 0, PortDirection::Output),
    ("out_raw_value3", 13, 0, PortDirection::Output),
    ("out_raw_value4", 13, 0, PortDirection::Output),
    ("out_raw_value5", 13, 0, PortDirection::Output),
    ("out_raw_value6", 13, 0, PortDirection::Output),
    ("out_raw_value7", 13, 0, PortDirection::Output),
    ("out_raw_value8", 13, 0, PortDirection::Output),
    ("out_raw_value9", 13, 0, PortDirection::Output),
    ("out_raw_value10", 13, 0, PortDirection::Output),
    ("out_raw_value11", 13, 0, PortDirection::Output),
    ("frame_done", 0, 0, PortDirection::Output),
    ("protocol_error", 0, 0, PortDirection::Output),
];

const BAND_NAMES: [&str; 12] = [
    "LL5", "H5", "H4", "LH3", "HL3", "HH3", "LH2", "HL2", "HH2", "LH1", "HL1", "HH1",
];
const VALUE_PORTS: [&str; 12] = [
    "out_raw_value0",
    "out_raw_value1",
    "out_raw_value2",
    "out_raw_value3",
    "out_raw_value4",
    "out_raw_value5",
    "out_raw_value6",
    "out_raw_value7",
    "out_raw_value8",
    "out_raw_value9",
    "out_raw_value10",
    "out_raw_value11",
];

#[derive(Clone, Copy)]
struct BandRect {
    x: usize,
    y: usize,
    width: usize,
    height: usize,
}

const BANDS: [BandRect; 12] = [
    BandRect {
        x: 0,
        y: 0,
        width: FRAME_WIDTH / 32,
        height: FRAME_HEIGHT / 8,
    },
    BandRect {
        x: FRAME_WIDTH / 32,
        y: 0,
        width: FRAME_WIDTH / 32,
        height: FRAME_HEIGHT / 8,
    },
    BandRect {
        x: FRAME_WIDTH / 16,
        y: 0,
        width: FRAME_WIDTH / 16,
        height: FRAME_HEIGHT / 8,
    },
    BandRect {
        x: 0,
        y: FRAME_HEIGHT / 8,
        width: FRAME_WIDTH / 8,
        height: FRAME_HEIGHT / 8,
    },
    BandRect {
        x: FRAME_WIDTH / 8,
        y: 0,
        width: FRAME_WIDTH / 8,
        height: FRAME_HEIGHT / 8,
    },
    BandRect {
        x: FRAME_WIDTH / 8,
        y: FRAME_HEIGHT / 8,
        width: FRAME_WIDTH / 8,
        height: FRAME_HEIGHT / 8,
    },
    BandRect {
        x: 0,
        y: FRAME_HEIGHT / 4,
        width: FRAME_WIDTH / 4,
        height: FRAME_HEIGHT / 4,
    },
    BandRect {
        x: FRAME_WIDTH / 4,
        y: 0,
        width: FRAME_WIDTH / 4,
        height: FRAME_HEIGHT / 4,
    },
    BandRect {
        x: FRAME_WIDTH / 4,
        y: FRAME_HEIGHT / 4,
        width: FRAME_WIDTH / 4,
        height: FRAME_HEIGHT / 4,
    },
    BandRect {
        x: 0,
        y: FRAME_HEIGHT / 2,
        width: FRAME_WIDTH / 2,
        height: FRAME_HEIGHT / 2,
    },
    BandRect {
        x: FRAME_WIDTH / 2,
        y: 0,
        width: FRAME_WIDTH / 2,
        height: FRAME_HEIGHT / 2,
    },
    BandRect {
        x: FRAME_WIDTH / 2,
        y: FRAME_HEIGHT / 2,
        width: FRAME_WIDTH / 2,
        height: FRAME_HEIGHT / 2,
    },
];

const EXPECTED_PAIR_WRITES: [usize; 12] = [
    2_000, 2_000, 4_000, 8_000, 8_000, 8_000, 32_000, 32_000, 32_000, 128_000, 128_000, 128_000,
];

const EXPECTED_CONFLICT_MASKS: [u16; 12] = [
    0x003, 0x003, 0x004, 0xe08, 0x030, 0x030, 0x040, 0x180, 0x180, 0xe08, 0xe08, 0xe08,
];

const COMPACT_QUEUE_BANK_MASKS: [u16; 5] = [
    (1 << 6) | (1 << 9),
    (1 << 8) | (1 << 10),
    (1 << 7) | (1 << 11),
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4),
    (1 << 1) | (1 << 5),
];

const COMPACT_PAIR_CAPACITIES: [i32; 12] =
    [64, 64, 64, 192, 192, 192, 176, 176, 176, 336, 336, 336];

const CANONICAL_BACKLOG_PEAKS: [i32; 12] = [24, 24, 40, 80, 80, 80, 160, 160, 160, 318, 320, 320];

const NOMINAL_CAMERA_BACKLOG_PEAKS: [i32; 12] =
    [24, 24, 40, 68, 69, 71, 160, 146, 152, 208, 208, 208];

fn deterministic_frame() -> Vec<u8> {
    let mut state = 0x1029_3847_6a09_e667_u64;
    (0..FRAME_PIXELS)
        .map(|_| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state as u8
        })
        .collect()
}

fn forward_53_1d(values: &mut [i32]) {
    assert!(values.len() >= 2 && values.len().is_multiple_of(2));
    let half = values.len() / 2;
    let mut low = Vec::with_capacity(half);
    let mut high = Vec::with_capacity(half);

    for pair in values.chunks_exact(2) {
        low.push(pair[0]);
        high.push(pair[1]);
    }
    for i in 0..half {
        high[i] -= (low[i] + low[(i + 1).min(half - 1)]) >> 1;
    }
    for i in 0..half {
        low[i] += (high[i.saturating_sub(1)] + high[i] + 2) >> 2;
    }

    values[..half].copy_from_slice(&low);
    values[half..].copy_from_slice(&high);
}

fn reference_transform(pixels: &[u8]) -> Vec<i32> {
    assert_eq!(pixels.len(), FRAME_PIXELS);
    let mut coefficients = pixels
        .iter()
        .map(|&pixel| i32::from(pixel) - 128)
        .collect::<Vec<_>>();
    let mut active_width = FRAME_WIDTH;
    let mut active_height = FRAME_HEIGHT;

    for level in 1..=5 {
        for y in 0..active_height {
            let start = y * FRAME_WIDTH;
            forward_53_1d(&mut coefficients[start..start + active_width]);
        }

        if level <= 3 {
            for x in 0..active_width {
                let mut column = (0..active_height)
                    .map(|y| coefficients[y * FRAME_WIDTH + x])
                    .collect::<Vec<_>>();
                forward_53_1d(&mut column);
                for (y, value) in column.into_iter().enumerate() {
                    coefficients[y * FRAME_WIDTH + x] = value;
                }
            }
            active_height /= 2;
        }
        active_width /= 2;
    }

    coefficients
}

struct TraceMonitor<'a> {
    reference: &'a [i32],
    band_seen: [usize; 12],
    total_seen: usize,
    half_full: u16,
    pair_writes: [usize; 12],
    conflict_masks: [u16; 12],
    pair_arrivals: Vec<(usize, usize)>,
    group_arrivals: Vec<(usize, usize)>,
    frame_done_pulses: usize,
    cycles: usize,
}

impl<'a> TraceMonitor<'a> {
    fn new(reference: &'a [i32]) -> Self {
        Self {
            reference,
            band_seen: [0; 12],
            total_seen: 0,
            half_full: 0,
            pair_writes: [0; 12],
            conflict_masks: [0; 12],
            pair_arrivals: Vec::with_capacity(FRAME_PIXELS / 2),
            group_arrivals: Vec::with_capacity(64_000),
            frame_done_pulses: 0,
            cycles: 0,
        }
    }

    fn observe(&mut self, model: &DynamicVerilatedModel<'_>) -> TestResult {
        self.cycles += 1;
        if read_u64(model, "protocol_error")? != 0 {
            return Err(
                format!("transform asserted protocol_error at cycle {}", self.cycles).into(),
            );
        }

        let valid = read_u64(model, "out_raw_valid")? as u16;
        for (bank, mask) in COMPACT_QUEUE_BANK_MASKS.into_iter().enumerate() {
            if (valid & mask).count_ones() > 1 {
                return Err(format!(
                    "compact queue bank {bank} has multiple raw writes at cycle {}: valid={valid:#05x}",
                    self.cycles
                )
                .into());
            }
        }

        let pair_write_mask = valid & self.half_full;
        for band in 0..12 {
            let bit = 1_u16 << band;
            if pair_write_mask & bit != 0 {
                self.pair_writes[band] += 1;
                self.conflict_masks[band] |= pair_write_mask;
                self.pair_arrivals.push((self.cycles, band));
            }
        }
        self.half_full ^= valid;

        for (band, descriptor) in BANDS.into_iter().enumerate() {
            if valid & (1 << band) == 0 {
                continue;
            }
            let ordinal = self.band_seen[band];
            let band_count = descriptor.width * descriptor.height;
            if ordinal >= band_count {
                return Err(format!(
                    "extra {} coefficient at cycle {} (ordinal {ordinal})",
                    BAND_NAMES[band], self.cycles
                )
                .into());
            }
            let x = ordinal % descriptor.width;
            let y = ordinal / descriptor.width;
            let index = (descriptor.y + y) * FRAME_WIDTH + descriptor.x + x;
            let actual = sign_extend(read_u64(model, VALUE_PORTS[band])?, 14) as i32;
            let expected = self.reference[index];
            if actual != expected {
                return Err(format!(
                    "{} mismatch at ({x}, {y}), cycle {}: RTL={actual}, Rust={expected}",
                    BAND_NAMES[band], self.cycles
                )
                .into());
            }
            self.band_seen[band] += 1;
            self.total_seen += 1;
            if self.band_seen[band].is_multiple_of(16) {
                self.group_arrivals.push((self.cycles, band));
            }
        }

        if read_u64(model, "frame_done")? != 0 {
            self.frame_done_pulses += 1;
        }
        Ok(())
    }

    fn done(&self) -> bool {
        self.frame_done_pulses != 0
    }

    fn verify(
        &self,
        check_exact_conflict_census: bool,
        expected_backlog_peaks: Option<[i32; 12]>,
    ) -> TestResult {
        if self.frame_done_pulses != 1 {
            return Err(format!(
                "frame_done pulse count={}, expected 1",
                self.frame_done_pulses
            )
            .into());
        }
        if self.total_seen != FRAME_PIXELS {
            return Err(format!(
                "coefficient count={}, expected {FRAME_PIXELS}",
                self.total_seen
            )
            .into());
        }
        for (band, descriptor) in BANDS.into_iter().enumerate() {
            let expected = descriptor.width * descriptor.height;
            if self.band_seen[band] != expected {
                return Err(format!(
                    "{} count={}, expected {expected}",
                    BAND_NAMES[band], self.band_seen[band]
                )
                .into());
            }
        }
        if self.half_full != 0 {
            return Err(
                format!("unpaired coefficient lanes at end: {:#05x}", self.half_full).into(),
            );
        }
        if self.pair_writes != EXPECTED_PAIR_WRITES {
            return Err(format!(
                "pair-write counts changed: got {:?}, expected {:?}",
                self.pair_writes, EXPECTED_PAIR_WRITES
            )
            .into());
        }
        if check_exact_conflict_census && self.conflict_masks != EXPECTED_CONFLICT_MASKS {
            return Err(format!(
                "pair-write conflict masks changed: got {:03x?}, expected {:03x?}",
                self.conflict_masks, EXPECTED_CONFLICT_MASKS
            )
            .into());
        }
        if let Some(expected) = expected_backlog_peaks {
            self.verify_fixed_control_trace_backlog(expected)?;
        }
        Ok(())
    }

    fn verify_fixed_control_trace_backlog(&self, expected_peak: [i32; 12]) -> TestResult {
        const SERVICE_CLOCKS: usize = 11;
        const RELEASE_CLOCKS: usize = 11;
        let schedule = raw_group_schedule();
        if schedule.len() != 64_000 || self.group_arrivals.len() != 64_000 {
            return Err(format!(
                "group trace length mismatch: schedule={} arrivals={}",
                schedule.len(),
                self.group_arrivals.len()
            )
            .into());
        }

        let mut arrivals_by_band: [Vec<usize>; 12] = std::array::from_fn(|_| Vec::new());
        for &(cycle, band) in &self.group_arrivals {
            arrivals_by_band[band].push(cycle);
        }
        let mut arrival_index = [0_usize; 12];
        let mut group_releases = Vec::with_capacity(schedule.len());
        let mut next_start = 0_usize;
        for &band in schedule {
            let band = usize::from(band);
            let ready = arrivals_by_band[band][arrival_index[band]];
            arrival_index[band] += 1;
            // This models queue read-group starts, not encoder pair accepts.
            // A completed group cannot be visible before the clock after its
            // eighth pair write.  Once replay is running, the real queue may
            // prefetch the next pair while the encoder finalizes the previous
            // group; the proved encoder initiation interval is at most eleven
            // clocks.  Therefore this max-plus recurrence starts every
            // abstract read no earlier than the corresponding RTL read.
            let start = next_start.max(ready + 1);
            // The RTL can prefetch pair zero and then hold it through at most
            // three finalizer clocks.  Its remaining seven reads issue one
            // per clock, so its group pointer advances no later than ten
            // clocks after read-group start.  Releasing all eight abstract
            // slots at start+11 is later than every per-pair RTL release.
            group_releases.push((start + RELEASE_CLOCKS, band));
            next_start = start + SERVICE_CLOCKS;
        }

        let mut events = Vec::with_capacity(128_000);
        for &(cycle, band) in &self.pair_arrivals {
            events.push((cycle, band, 1_i8));
        }
        for &(cycle, band) in &group_releases {
            events.push((cycle, band, -8_i8));
        }
        events.sort_unstable();
        // Group-held occupancy retains all eight pair slots until the final
        // read issue advances read_group_pointer.  It upper-bounds exact
        // unread-pair occupancy by at most seven and is the state relevant to
        // the synthesized group-granular collision guard.
        let mut group_held = [0_i32; 12];
        let mut peak_group_held = [0_i32; 12];
        let mut event = 0;
        while event < events.len() {
            let cycle = events[event].0;
            let mut delta = [0_i32; 12];
            while event < events.len() && events[event].0 == cycle {
                delta[events[event].1] += i32::from(events[event].2);
                event += 1;
            }
            for band in 0..12 {
                group_held[band] += delta[band];
                if group_held[band] < 0 {
                    return Err(format!(
                        "negative abstract group-held occupancy for band {band} at cycle {cycle}"
                    )
                    .into());
                }
                peak_group_held[band] = peak_group_held[band].max(group_held[band]);
            }
        }
        eprintln!("11-clock conservative group-held pair peaks: {peak_group_held:?}");
        if peak_group_held != expected_peak {
            return Err(format!(
                "fixed-control-trace group-held peaks changed: got {peak_group_held:?}, expected {expected_peak:?}"
            )
            .into());
        }
        for band in 0..12 {
            let capacity = COMPACT_PAIR_CAPACITIES[band];
            // A first coefficient can occupy the next physical pair slot one
            // clock before that pair enters the complete-pair recurrence.
            if peak_group_held[band] + 1 >= capacity {
                return Err(format!(
                    "band {band} physical half-pair bound failed: {} + 1 !< {capacity}",
                    peak_group_held[band],
                )
                .into());
            }
            // Retain a complete-group margin beyond the virtual-service peak,
            // covering visibility at the eighth write and same-clock endpoint
            // ordering without relying on RAM read-during-write behavior.
            if peak_group_held[band] >= capacity - 8 {
                return Err(format!(
                    "band {band} completed-group margin failed: {} !< {capacity} - 8",
                    peak_group_held[band],
                )
                .into());
            }
        }
        if group_held != [0; 12] {
            return Err(format!("undrained abstract group-held state: {group_held:?}").into());
        }
        Ok(())
    }
}

fn drive_idle(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("in_sof", 0_u8)?;
    model.pin("in_valid", 0_u8)?;
    model.pin("in_px0", 0_u8)?;
    model.pin("in_px1", 0_u8)?;
    Ok(())
}

fn run_full_frame(
    model: &mut DynamicVerilatedModel<'_>,
    pixels: &[u8],
    reference: &[i32],
    camera_cadence: bool,
    horizontal_blank_clocks: usize,
    expected_backlog_peaks: Option<[i32; 12]>,
) -> TestResult {
    let mut monitor = TraceMonitor::new(reference);

    model.pin("clk", 0_u8)?;
    model.pin("rst_n", 0_u8)?;
    drive_idle(model)?;
    model.eval();
    for _ in 0..4 {
        clock(model)?;
    }
    model.pin("rst_n", 1_u8)?;

    for row in 0..FRAME_HEIGHT {
        for pair in 0..ACTIVE_CLOCKS_PER_ROW {
            let index = row * FRAME_WIDTH + 2 * pair;
            model.pin("in_sof", u8::from(index == 0))?;
            model.pin("in_valid", 1_u8)?;
            model.pin("in_px0", pixels[index])?;
            model.pin("in_px1", pixels[index + 1])?;
            clock(model)?;
            monitor.observe(model)?;
            // The CSI receiver produces 80 Mbeat/s into a 120 MHz core. Two
            // accepted pairs followed by one bubble model that exact active
            // rate while retaining the same valid-driven pixel sequence.
            if camera_cadence && pair & 1 == 1 {
                drive_idle(model)?;
                clock(model)?;
                monitor.observe(model)?;
            }
        }
        drive_idle(model)?;
        let blank_clocks = if camera_cadence {
            // 640 valid + 320 active bubbles + 290 blank = 1,250 core clocks,
            // matching the nominal 120 fps camera row average.
            290
        } else {
            horizontal_blank_clocks
        };
        for _ in 0..blank_clocks {
            clock(model)?;
            monitor.observe(model)?;
        }
    }

    drive_idle(model)?;
    let mut drain_clocks = 0;
    while !monitor.done() && drain_clocks < MAX_DRAIN_CLOCKS {
        clock(model)?;
        monitor.observe(model)?;
        drain_clocks += 1;
    }
    if !monitor.done() {
        return Err(format!(
            "transform timed out after {drain_clocks} drain clocks with {} coefficients",
            monitor.total_seen
        )
        .into());
    }

    for _ in 0..QUIET_CLOCKS {
        clock(model)?;
        monitor.observe(model)?;
    }
    monitor.verify(
        !camera_cadence && horizontal_blank_clocks == HORIZONTAL_BLANK_CLOCKS,
        expected_backlog_peaks,
    )?;
    eprintln!(
        "forward transform: {} coefficients, {} observed cycles, {} drain clocks",
        monitor.total_seen, monitor.cycles, drain_clocks
    );
    Ok(())
}

#[test]
fn forward_wavelet_transform_matches_rust_and_pair_conflict_census() -> TestResult {
    let pixels = deterministic_frame();
    let reference = reference_transform(&pixels);
    let adapter = Path::new(ADAPTER);
    with_model(
        "forward_wavelet_transform_adapter",
        "forward_wavelet_transform_adapter",
        ADAPTER,
        TOP_PORTS,
        &[adapter],
        |model| {
            run_full_frame(
                model,
                &pixels,
                &reference,
                false,
                HORIZONTAL_BLANK_CLOCKS,
                Some(CANONICAL_BACKLOG_PEAKS),
            )
        },
    )
}

#[test]
fn forward_wavelet_transform_matches_rust_at_camera_input_rate() -> TestResult {
    let pixels = deterministic_frame();
    let reference = reference_transform(&pixels);
    let adapter = Path::new(ADAPTER);
    with_model(
        "forward-wavelet-transform-camera-rate",
        "forward_wavelet_transform_adapter",
        ADAPTER,
        TOP_PORTS,
        &[adapter],
        |model| {
            run_full_frame(
                model,
                &pixels,
                &reference,
                true,
                HORIZONTAL_BLANK_CLOCKS,
                Some(NOMINAL_CAMERA_BACKLOG_PEAKS),
            )
        },
    )
}

#[test]
#[ignore = "extended-blank control-trace regression"]
fn extended_uniform_row_gaps_have_stable_backlog_traces() -> TestResult {
    let pixels = deterministic_frame();
    let reference = reference_transform(&pixels);
    let adapter = Path::new(ADAPTER);
    with_model(
        "forward_wavelet_transform_extended_row_gaps",
        "forward_wavelet_transform_adapter",
        ADAPTER,
        TOP_PORTS,
        &[adapter],
        |model| {
            for (horizontal_blank_clocks, expected_peak) in [
                (610, [24, 24, 40, 80, 80, 80, 160, 160, 160, 248, 248, 253]),
                (
                    1_024,
                    [24, 24, 40, 80, 80, 80, 160, 160, 160, 208, 208, 208],
                ),
                (
                    4_096,
                    [24, 24, 40, 80, 80, 80, 160, 160, 160, 208, 208, 208],
                ),
            ] {
                eprintln!(
                    "checking extended control trace at {horizontal_blank_clocks} blank clocks/row"
                );
                run_full_frame(
                    model,
                    &pixels,
                    &reference,
                    false,
                    horizontal_blank_clocks,
                    Some(expected_peak),
                )?;
            }
            Ok(())
        },
    )
}

// This tensor-product sign pattern excites a third-level HH coefficient
// above 900, close to the proved 11-bit limit. Random byte images rarely do.
#[test]
fn forward_wavelet_transform_retains_large_third_level_details() -> TestResult {
    const SIGNS: [i8; 21] = [
        -1, 1, 1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, 1, 1, -1,
    ];
    let adapter = Path::new(ADAPTER);
    with_model(
        "forward-wavelet-transform-large-details",
        "forward_wavelet_transform_adapter",
        ADAPTER,
        TOP_PORTS,
        &[adapter],
        |model| {
            for polarity in [-1, 1] {
                let mut pixels = vec![128_u8; FRAME_PIXELS];
                for (y, sy) in SIGNS.iter().enumerate() {
                    for (x, sx) in SIGNS.iter().enumerate() {
                        pixels[(y + 2) * FRAME_WIDTH + x + 2] =
                            if polarity * i32::from(*sx) * i32::from(*sy) > 0 {
                                255
                            } else {
                                0
                            };
                    }
                }
                let reference = reference_transform(&pixels);
                assert!(
                    reference[(FRAME_HEIGHT / 8 + 1) * FRAME_WIDTH + FRAME_WIDTH / 8 + 1].abs()
                        > 900
                );
                run_full_frame(
                    model,
                    &pixels,
                    &reference,
                    true,
                    HORIZONTAL_BLANK_CLOCKS,
                    Some(NOMINAL_CAMERA_BACKLOG_PEAKS),
                )?;
            }
            Ok(())
        },
    )
}
