//! Full-frame parity for the adaptive threshold frontend and RAM reuse.
//! Every accepted raster record is checked, including data, masks and metadata.

use std::path::{Path, PathBuf};

use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, clock, read_u64, with_model_sources};

const WIDTH: usize = 1280;
const HEIGHT: usize = 800;
const ROW_CYCLES: usize = 1092;
const ACTIVE_CYCLES: usize = 960;
const FRAME_CYCLES: usize = 994_812;
const OUTPUT_PAIRS: usize = 848;
const MAX_DRAIN: usize = 100_000;
const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;
const ADAPTER: &str = "tests/lrcca/tests/rtl_adapters/pipeline_adaptive_threshold.sv";
const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("in_vref", 0, 0, PortDirection::Input),
    ("in_href", 0, 0, PortDirection::Input),
    ("in_pair", 15, 0, PortDirection::Input),
    ("out_ready", 0, 0, PortDirection::Input),
    ("out_valid", 0, 0, PortDirection::Output),
    ("out_record", 31, 0, PortDirection::Output),
    ("status", 7, 0, PortDirection::Output),
];

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn frame(seed: u64, invert: bool) -> Vec<u8> {
    let mut state = seed;
    (0..WIDTH * HEIGHT)
        .map(|index| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let (x, y) = (index % WIDTH, index / WIDTH);
            // Broad contrasts exercise the spatial references; texture
            // exercises signed CDF and interpolation rounding.
            let level = if (x / 57 + y / 43) & 1 == 0 { 48 } else { 192 };
            let sample = (level + (state & 31) as i32 - 16).clamp(0, 255) as u8;
            if invert { 255 - sample } else { sample }
        })
        .collect()
}

fn expected_records(pixels: &[u8]) -> TestResult<Vec<u32>> {
    let composite = lrcca::adaptive_threshold_composite(pixels, WIDTH, HEIGHT)?;
    assert_eq!((composite.width, composite.height), (1696, 807));
    assert!(composite.mask.iter().flatten().any(|&pixel| pixel != 0));
    let mut records = Vec::with_capacity(OUTPUT_PAIRS * composite.height);
    for y in 0..composite.height {
        for x in (0..composite.width).step_by(2) {
            let fv0 = composite.feature_valid[y][x] != 0;
            let fv1 = composite.feature_valid[y][x + 1] != 0;
            let replicated = composite.row_replicated[y][x] != 0;
            assert_eq!(replicated, composite.row_replicated[y][x + 1] != 0);
            let source_y = match (fv0, fv1) {
                (true, true) => {
                    assert_eq!(composite.source_y[y][x], composite.source_y[y][x + 1]);
                    composite.source_y[y][x]
                }
                (true, false) => composite.source_y[y][x],
                (false, true) => composite.source_y[y][x + 1],
                (false, false) => 0,
            };
            records.push(
                u32::from(255 - composite.raw[y][x])
                    | (u32::from(255 - composite.raw[y][x + 1]) << 8)
                    | (u32::from(composite.mask[y][x] != 0) << 16)
                    | (u32::from(composite.mask[y][x + 1] != 0) << 17)
                    | (u32::from(x + 2 == composite.width) << 18)
                    | (u32::from(replicated) << 19)
                    | (u32::from(fv0) << 20)
                    | (u32::from(fv1) << 21)
                    | ((source_y as u32) << 22),
            );
        }
    }
    Ok(records)
}

fn idle(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("in_vref", 0_u8)?;
    model.pin("in_href", 0_u8)?;
    model.pin("in_pair", 0_u16)?;
    Ok(())
}

fn reset(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("rst", 1_u8)?;
    model.pin("out_ready", 1_u8)?;
    idle(model)?;
    for _ in 0..4 {
        clock(model)?;
    }
    model.pin("rst", 0_u8)?;
    Ok(())
}

struct Monitor<'a> {
    expected: &'a [u32],
    accepted: usize,
    cycles: usize,
    stalled_payload: Option<u32>,
    stalled_cycles: usize,
    frame_done: usize,
    plane_hashes: [u64; 3],
    row_start_cycle: Option<usize>,
    max_row_span: usize,
    min_row_start_interval: usize,
    max_row_start_interval: usize,
}

impl<'a> Monitor<'a> {
    fn new(expected: &'a [u32]) -> Self {
        Self {
            expected,
            accepted: 0,
            cycles: 0,
            stalled_payload: None,
            stalled_cycles: 0,
            frame_done: 0,
            plane_hashes: [FNV_OFFSET; 3],
            row_start_cycle: None,
            max_row_span: 0,
            min_row_start_interval: usize::MAX,
            max_row_start_interval: 0,
        }
    }

    fn tick(&mut self, model: &mut DynamicVerilatedModel<'_>) -> TestResult {
        // Ten stalled clocks per physical input row leave ample service time.
        // Sample the handshake before its rising edge.
        let ready = !(300..310).contains(&(self.cycles % ROW_CYCLES));
        model.pin("out_ready", u8::from(ready))?;
        model.pin("clk", 0_u8)?;
        model.eval();
        let status = read_u64(model, "status")?;
        if status & 0x1f != 0 {
            return Err(format!(
                "wavelet formatter error flags {:#04x} at cycle {} pair {}",
                status & 0x1f,
                self.cycles,
                self.accepted
            )
            .into());
        }
        self.frame_done += usize::from(status & 0x40 != 0);
        let valid = read_u64(model, "out_valid")? != 0;
        let payload = if valid || self.stalled_payload.is_some() {
            read_u64(model, "out_record")? as u32
        } else {
            0
        };
        if let Some(previous) = self.stalled_payload
            && (!valid || payload != previous)
        {
            return Err(format!("stalled output changed at cycle {}", self.cycles).into());
        }
        self.stalled_payload = if valid && !ready { Some(payload) } else { None };
        self.stalled_cycles += usize::from(valid && !ready);
        if valid && ready {
            let expected = self.expected.get(self.accepted).ok_or_else(|| {
                format!("unexpected output after {} records", self.expected.len())
            })?;
            if payload != *expected {
                return Err(format!(
                    "record mismatch at composite ({},{}), cycle {}: RTL={payload:#010x}, Rust={expected:#010x}",
                    2 * (self.accepted % OUTPUT_PAIRS),
                    self.accepted / OUTPUT_PAIRS,
                    self.cycles
                )
                .into());
            }
            let pair_x = self.accepted % OUTPUT_PAIRS;
            if pair_x == 0 {
                if let Some(previous) = self.row_start_cycle {
                    let interval = self.cycles - previous;
                    self.min_row_start_interval = self.min_row_start_interval.min(interval);
                    self.max_row_start_interval = self.max_row_start_interval.max(interval);
                }
                self.row_start_cycle = Some(self.cycles);
            }
            if pair_x + 1 == OUTPUT_PAIRS {
                let start = self.row_start_cycle.expect("row end follows a row start");
                self.max_row_span = self.max_row_span.max(self.cycles - start + 1);
            }
            let plane = if pair_x < 640 {
                0
            } else if pair_x < 804 {
                1
            } else {
                2
            };
            for byte in payload.to_le_bytes() {
                self.plane_hashes[plane] ^= u64::from(byte);
                self.plane_hashes[plane] = self.plane_hashes[plane].wrapping_mul(FNV_PRIME);
            }
            self.accepted += 1;
        }
        model.pin("clk", 1_u8)?;
        model.eval();
        self.cycles += 1;
        Ok(())
    }
}

fn drive_rows(
    model: &mut DynamicVerilatedModel<'_>,
    pixels: &[u8],
    rows: usize,
    monitor: &mut Monitor<'_>,
) -> TestResult {
    for y in 0..rows {
        model.pin("in_vref", 1_u8)?;
        let mut pair = 0;
        // The 80 MHz camera feeds two pairs per three 120 MHz system clocks:
        // 640 pairs over 960 clocks, followed by 132 clocks of line blanking.
        for active_cycle in 0..ACTIVE_CYCLES {
            let valid = active_cycle % 3 != 2;
            model.pin("in_href", u8::from(valid))?;
            if valid {
                let index = y * WIDTH + 2 * pair;
                model.pin(
                    "in_pair",
                    u16::from_le_bytes([pixels[index], pixels[index + 1]]),
                )?;
                pair += 1;
            } else {
                model.pin("in_pair", 0_u16)?;
            }
            monitor.tick(model)?;
        }
        assert_eq!(pair, WIDTH / 2);
        model.pin("in_href", 0_u8)?;
        model.pin("in_pair", 0_u16)?;
        for _ in ACTIVE_CYCLES..ROW_CYCLES {
            monitor.tick(model)?;
        }
    }
    Ok(())
}

fn complete_frame(
    model: &mut DynamicVerilatedModel<'_>,
    pixels: &[u8],
    expected: &[u32],
    label: &str,
    processing_reset_after: bool,
) -> TestResult {
    let mut monitor = Monitor::new(expected);
    drive_rows(model, pixels, HEIGHT, &mut monitor)?;
    idle(model)?;
    let mut drained = false;
    for _ in 0..MAX_DRAIN {
        monitor.tick(model)?;
        if monitor.accepted == expected.len()
            && monitor.frame_done == 1
            && read_u64(model, "status")? & 0x20 == 0
        {
            drained = true;
            break;
        }
    }
    if !drained {
        return Err(format!(
            "{label} did not retire: {}/{} records, {} done pulses, status={:#04x}",
            monitor.accepted,
            expected.len(),
            monitor.frame_done,
            read_u64(model, "status")?
        )
        .into());
    }
    let retirement_cycle = monitor.cycles;
    let reset_cycles = if processing_reset_after { 4 } else { 0 };
    assert!(
        retirement_cycle + reset_cycles < FRAME_CYCLES,
        "{label} missed the next physical frame: {retirement_cycle} clocks"
    );
    // Check the whole vertical blanking interval for duplicate output, error
    // flags or repeated completion. Keep consecutive SOFs 994812 clocks apart.
    while monitor.cycles < FRAME_CYCLES - reset_cycles {
        monitor.tick(model)?;
    }
    assert_eq!(monitor.accepted, expected.len());
    assert_eq!(monitor.frame_done, 1);
    assert!(
        monitor.stalled_cycles > 1000,
        "ready stalls did not cover active output"
    );
    assert!(
        monitor.max_row_span <= ROW_CYCLES,
        "formatter row exceeds its 1092-clock physical service budget: {}",
        monitor.max_row_span
    );
    eprintln!(
        "{label}: records={} retire_cycle={} frame_cycles={} stalled={} max_row_span={} row_start_interval={}..{} RAW/LL2/LL4 hashes={:016x?}",
        monitor.accepted,
        retirement_cycle,
        monitor.cycles + reset_cycles,
        monitor.stalled_cycles,
        monitor.max_row_span,
        monitor.min_row_start_interval,
        monitor.max_row_start_interval,
        monitor.plane_hashes
    );
    if processing_reset_after {
        // Match the board pipeline's reset between completed frames.
        assert_eq!(read_u64(model, "status")? & 0x20, 0);
        reset(model)?;
    }
    Ok(())
}

#[test]
fn adaptive_formatter_matches_same_frame_at_120fps_after_reset_and_abort() -> TestResult {
    let root = gateware_root();
    let adapter = root.join(ADAPTER);
    let sources = [
        "rtl/lwc/LwcRegisteredAddSub.sv",
        "rtl/lwc/horizontal_lift_stage.sv",
        "rtl/lwc/horizontal_lift_row_service.sv",
        "rtl/lwc/vertical_lift_scheduler.sv",
        "rtl/lwc/first_horizontal_lift.sv",
        "rtl/lwc/first_vertical_lift.sv",
        "rtl/lwc/forward_wavelet_transform.sv",
        "rtl/lrcca/Ll4CausalMean5x3.sv",
        "rtl/lrcca/Ll2CausalBoxReference.sv",
        "rtl/lrcca/ThresholdMomentWeightRom.sv",
        "rtl/lrcca/ThresholdConnectivity2PPC.sv",
        "rtl/lrcca/AdaptiveThresholdFrontend.sv",
    ];
    let sources = sources.map(Path::new);
    let flat = vec![128; WIDTH * HEIGHT];
    let first = frame(0x1857_4023_ef92_a10b, false);
    let second = frame(0x9612_7a5c_034e_b8d1, true);
    let flat_expected = expected_records(&flat)?;
    let first_expected = expected_records(&first)?;
    let second_expected = expected_records(&second)?;
    with_model_sources(
        &root,
        &sources,
        "lrcca-adaptive-full-frame",
        "PipelineAdaptiveThreshold",
        Path::new(ADAPTER),
        PORTS,
        &[],
        &[adapter.as_path()],
        |model| {
            reset(model)?;
            complete_frame(model, &flat, &flat_expected, "first-frame equality", true)?;
            complete_frame(model, &first, &first_expected, "signed texture", true)?;
            complete_frame(
                model,
                &second,
                &second_expected,
                "inverted next frame",
                true,
            )?;
            let mut aborted = Monitor::new(&first_expected);
            drive_rows(model, &first, 96, &mut aborted)?;
            assert!(aborted.accepted > OUTPUT_PAIRS * 50);
            reset(model)?;
            complete_frame(
                model,
                &second,
                &second_expected,
                "recovery after abort",
                true,
            )?;
            if let Ok(dataset) = std::env::var("LUTCAM_THRESHOLD_DATASET_DIR") {
                for index in [
                    0, 4, 6, 7, 9, 14, 17, 18, 19, 20, 29, 35, 36, 39, 46, 47, 55, 61, 64, 66, 67,
                    69, 70, 72, 73, 78, 79, 85, 91, 95, 97, 99,
                ] {
                    let path = Path::new(&dataset).join(format!("frame_{index:03}.png"));
                    let pixels = image::open(path)?.into_luma8();
                    assert_eq!(pixels.dimensions(), (WIDTH as u32, HEIGHT as u32));
                    let expected = expected_records(pixels.as_raw())?;
                    complete_frame(
                        model,
                        pixels.as_raw(),
                        &expected,
                        "natural projected tags",
                        true,
                    )?;
                }
            }
            Ok(())
        },
    )
}
