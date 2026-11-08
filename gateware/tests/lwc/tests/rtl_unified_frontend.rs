//! Full-frame regression for the adaptive-analysis taps shared with LWC.
//!
//! The normative codec coefficient trace remains covered by `rtl_transform`.
//! This test drives the same 640-active/256-blank line cadence and checks every
//! row-major LL2 and averaged-LL4 pair against the independent Rust model.

mod rtl_support;

use std::cell::Cell;
use std::path::Path;

use lwc::{FRAME_HEIGHT, FRAME_PIXELS, FRAME_WIDTH, FrontendPlane, unified_frontend_reference};
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};

use rtl_support::{TestResult, clock, read_u64, sign_extend, with_model};

const ACTIVE_CLOCKS_PER_ROW: usize = FRAME_WIDTH / 2;
const HORIZONTAL_BLANK_CLOCKS: usize = 256;
const MAX_DRAIN_CLOCKS: usize = 2_000_000;
const QUIET_CLOCKS: usize = 4;
const LL2_MAX_ROW_SERVICE_CLOCKS: usize = 640;
const LL2_MAX_PAIR_GAP: usize = 4;
const LL4_MAX_ROW_SERVICE_CLOCKS: usize = 80;
const LL4_MAX_PAIR_GAP: usize = 2;

const ADAPTER: &str = "tests/rtl_adapters/unified_wavelet_frontend_adapter.sv";

const TOP_PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst_n", 0, 0, PortDirection::Input),
    ("in_sof", 0, 0, PortDirection::Input),
    ("in_valid", 0, 0, PortDirection::Input),
    ("in_px0", 7, 0, PortDirection::Input),
    ("in_px1", 7, 0, PortDirection::Input),
    ("out_ll2_valid", 0, 0, PortDirection::Output),
    ("out_ll2_first", 0, 0, PortDirection::Output),
    ("out_ll2_last", 0, 0, PortDirection::Output),
    ("out_ll2_pair_x", 9, 0, PortDirection::Output),
    ("out_ll2_y", 9, 0, PortDirection::Output),
    ("out_ll2_value0", 13, 0, PortDirection::Output),
    ("out_ll2_value1", 13, 0, PortDirection::Output),
    ("out_ll4_valid", 0, 0, PortDirection::Output),
    ("out_ll4_first", 0, 0, PortDirection::Output),
    ("out_ll4_last", 0, 0, PortDirection::Output),
    ("out_ll4_pair_x", 9, 0, PortDirection::Output),
    ("out_ll4_y", 9, 0, PortDirection::Output),
    ("out_ll4_value0", 13, 0, PortDirection::Output),
    ("out_ll4_value1", 13, 0, PortDirection::Output),
    ("frame_done", 0, 0, PortDirection::Output),
    ("protocol_error", 0, 0, PortDirection::Output),
];

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

struct TapMonitor<'a> {
    name: &'static str,
    prefix: &'static str,
    reference: &'a FrontendPlane,
    pairs_seen: usize,
    row_start_cycle: Option<usize>,
    previous_pair_cycle: Option<usize>,
    max_row_span: usize,
    max_pair_gap: usize,
}

impl<'a> TapMonitor<'a> {
    fn new(name: &'static str, prefix: &'static str, reference: &'a FrontendPlane) -> Self {
        assert!(reference.width.is_multiple_of(2));
        Self {
            name,
            prefix,
            reference,
            pairs_seen: 0,
            row_start_cycle: None,
            previous_pair_cycle: None,
            max_row_span: 0,
            max_pair_gap: 0,
        }
    }

    fn observe(&mut self, model: &DynamicVerilatedModel<'_>, cycle: usize) -> TestResult {
        let valid_port = format!("{}_valid", self.prefix);
        if read_u64(model, &valid_port)? == 0 {
            return Ok(());
        }

        let pairs_per_row = self.reference.width / 2;
        let expected_pairs = pairs_per_row * self.reference.height;
        if self.pairs_seen >= expected_pairs {
            return Err(format!("{} emitted an extra pair at cycle {cycle}", self.name).into());
        }
        let expected_x = self.pairs_seen % pairs_per_row;
        let expected_y = self.pairs_seen / pairs_per_row;

        let pair_x = read_u64(model, &format!("{}_pair_x", self.prefix))? as usize;
        let y = read_u64(model, &format!("{}_y", self.prefix))? as usize;
        let first = read_u64(model, &format!("{}_first", self.prefix))? != 0;
        let last = read_u64(model, &format!("{}_last", self.prefix))? != 0;
        if (pair_x, y) != (expected_x, expected_y) {
            return Err(format!(
                "{} coordinate mismatch at pair {} cycle {cycle}: RTL=({pair_x},{y}), expected=({expected_x},{expected_y})",
                self.name, self.pairs_seen
            )
            .into());
        }
        if first != (expected_x == 0) || last != (expected_x + 1 == pairs_per_row) {
            return Err(format!(
                "{} boundary mismatch at ({expected_x},{expected_y}) cycle {cycle}: first={first}, last={last}",
                self.name
            )
            .into());
        }

        let scalar = 2 * self.pairs_seen;
        for lane in 0..2 {
            let actual = sign_extend(
                read_u64(model, &format!("{}_value{lane}", self.prefix))?,
                14,
            ) as i32;
            let expected = self.reference.samples[scalar + lane];
            if actual != expected {
                return Err(format!(
                    "{} mismatch at ({},{}), cycle {cycle}: RTL={actual}, Rust={expected}",
                    self.name,
                    2 * expected_x + lane,
                    expected_y
                )
                .into());
            }
        }

        if expected_x == 0 {
            self.row_start_cycle = Some(cycle);
            self.previous_pair_cycle = Some(cycle);
        } else if let Some(previous) = self.previous_pair_cycle {
            self.max_pair_gap = self.max_pair_gap.max(cycle - previous);
            self.previous_pair_cycle = Some(cycle);
        }
        if expected_x + 1 == pairs_per_row {
            let start = self.row_start_cycle.take().ok_or_else(|| {
                format!("{} ended row {expected_y} without starting it", self.name)
            })?;
            self.max_row_span = self.max_row_span.max(cycle - start + 1);
            self.previous_pair_cycle = None;
        }

        self.pairs_seen += 1;
        Ok(())
    }

    fn verify(&self, max_row_service_clocks: usize, max_pair_gap: usize) -> TestResult {
        let expected = self.reference.width * self.reference.height / 2;
        if self.pairs_seen != expected {
            return Err(format!(
                "{} pair count={}, expected={expected}",
                self.name, self.pairs_seen
            )
            .into());
        }
        if self.row_start_cycle.is_some() {
            return Err(format!("{} ended with an incomplete row", self.name).into());
        }
        if self.max_row_span > max_row_service_clocks {
            return Err(format!(
                "{} row service span={} clocks, budget={max_row_service_clocks}",
                self.name, self.max_row_span
            )
            .into());
        }
        if self.max_pair_gap > max_pair_gap {
            return Err(format!(
                "{} pair gap={} clocks, budget={max_pair_gap}",
                self.name, self.max_pair_gap
            )
            .into());
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
    ll2: &FrontendPlane,
    ll4: &FrontendPlane,
) -> TestResult {
    let mut ll2_monitor = TapMonitor::new("LL2", "out_ll2", ll2);
    let mut ll4_monitor = TapMonitor::new("LL4", "out_ll4", ll4);
    let mut cycles = 0;
    let frame_done_pulses = Cell::new(0);

    let mut observe = |model: &DynamicVerilatedModel<'_>| -> TestResult {
        cycles += 1;
        if read_u64(model, "protocol_error")? != 0 {
            return Err(format!("frontend asserted protocol_error at cycle {cycles}").into());
        }
        ll2_monitor.observe(model, cycles)?;
        ll4_monitor.observe(model, cycles)?;
        if read_u64(model, "frame_done")? != 0 {
            frame_done_pulses.set(frame_done_pulses.get() + 1);
        }
        Ok(())
    };

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
            observe(model)?;
        }
        drive_idle(model)?;
        for _ in 0..HORIZONTAL_BLANK_CLOCKS {
            clock(model)?;
            observe(model)?;
        }
    }

    drive_idle(model)?;
    let mut drain_clocks = 0;
    while frame_done_pulses.get() == 0 && drain_clocks < MAX_DRAIN_CLOCKS {
        clock(model)?;
        observe(model)?;
        drain_clocks += 1;
    }
    if frame_done_pulses.get() == 0 {
        return Err(format!("frontend timed out after {drain_clocks} drain clocks").into());
    }
    for _ in 0..QUIET_CLOCKS {
        clock(model)?;
        observe(model)?;
    }

    if frame_done_pulses.get() != 1 {
        return Err(format!(
            "frame_done pulse count={}, expected=1",
            frame_done_pulses.get()
        )
        .into());
    }
    ll2_monitor.verify(LL2_MAX_ROW_SERVICE_CLOCKS, LL2_MAX_PAIR_GAP)?;
    ll4_monitor.verify(LL4_MAX_ROW_SERVICE_CLOCKS, LL4_MAX_PAIR_GAP)?;
    eprintln!(
        "unified frontend: LL2={} pairs max_row_span={} max_pair_gap={}; LL4={} pairs max_row_span={} max_pair_gap={}; drain={drain_clocks}",
        ll2_monitor.pairs_seen,
        ll2_monitor.max_row_span,
        ll2_monitor.max_pair_gap,
        ll4_monitor.pairs_seen,
        ll4_monitor.max_row_span,
        ll4_monitor.max_pair_gap
    );
    Ok(())
}

#[test]
fn unified_frontend_taps_match_rust_at_896_clocks_per_input_row() -> TestResult {
    let pixels = deterministic_frame();
    let reference = unified_frontend_reference(&pixels, FRAME_WIDTH, FRAME_HEIGHT)
        .map_err(std::io::Error::other)?;
    let adapter = Path::new(ADAPTER);
    with_model(
        "unified_wavelet_frontend_adapter",
        "unified_wavelet_frontend_adapter",
        ADAPTER,
        TOP_PORTS,
        &[adapter],
        |model| run_full_frame(model, &pixels, &reference.ll2, &reference.ll4),
    )
}
