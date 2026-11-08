//! Cycle-by-cycle equivalence between the production ordinary and pair-fused
//! S5 centered-moment implementations.

use std::path::{Path, PathBuf};

use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, read_u64, with_model};

const WIDTH: usize = 6;
const HEIGHT: usize = 6;
const ADAPTER: &str = "tests/lrcca/tests/rtl_adapters/pair_fused_equiv.sv";

const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("datavalid", 0, 0, PortDirection::Input),
    ("d0", 0, 0, PortDirection::Input),
    ("d1", 0, 0, PortDirection::Input),
    ("feature_d0", 0, 0, PortDirection::Input),
    ("feature_d1", 0, 0, PortDirection::Input),
    ("feature_value_d0", 7, 0, PortDirection::Input),
    ("feature_value_d1", 7, 0, PortDirection::Input),
    ("feature_y", 2, 0, PortDirection::Input),
    ("sof", 0, 0, PortDirection::Input),
    ("eol", 0, 0, PortDirection::Input),
    ("ref_ready", 0, 0, PortDirection::Output),
    ("fused_ready", 0, 0, PortDirection::Output),
    ("ref_s5_control", 1, 0, PortDirection::Output),
    ("fused_s5_control", 1, 0, PortDirection::Output),
    ("ref_s5_data", 44, 0, PortDirection::Output),
    ("fused_s5_data", 44, 0, PortDirection::Output),
    ("ref_close_control", 1, 0, PortDirection::Output),
    ("fused_close_control", 1, 0, PortDirection::Output),
    ("ref_close_data", 44, 0, PortDirection::Output),
    ("fused_close_data", 44, 0, PortDirection::Output),
    ("ref_fv_valid", 0, 0, PortDirection::Output),
    ("fused_fv_valid", 0, 0, PortDirection::Output),
    ("ref_fv_data", 44, 0, PortDirection::Output),
    ("fused_fv_data", 44, 0, PortDirection::Output),
    ("ref_fv_child", 1, 0, PortDirection::Output),
    ("fused_fv_child", 1, 0, PortDirection::Output),
    ("ref_child_debug", 8, 0, PortDirection::Output),
    ("fused_child_debug", 8, 0, PortDirection::Output),
    ("ref_predicates", 7, 0, PortDirection::Output),
    ("fused_predicates", 7, 0, PortDirection::Output),
    ("ref_s5_shape", 2, 0, PortDirection::Output),
    ("fused_s5_shape", 2, 0, PortDirection::Output),
    ("ref_lane_state_meta", 6, 0, PortDirection::Output),
    ("fused_lane_state_meta", 6, 0, PortDirection::Output),
    ("ref_lane_state_data", 44, 0, PortDirection::Output),
    ("fused_lane_state_data", 44, 0, PortDirection::Output),
    ("ref_write_control", 10, 0, PortDirection::Output),
    ("fused_write_control", 10, 0, PortDirection::Output),
    ("ref_write_data0", 44, 0, PortDirection::Output),
    ("fused_write_data0", 44, 0, PortDirection::Output),
    ("ref_write_data1", 44, 0, PortDirection::Output),
    ("fused_write_data1", 44, 0, PortDirection::Output),
    ("ref_registered_state_meta", 6, 0, PortDirection::Output),
    ("fused_registered_state_meta", 6, 0, PortDirection::Output),
    ("ref_registered_state_data", 44, 0, PortDirection::Output),
    ("fused_registered_state_data", 44, 0, PortDirection::Output),
    ("ref_active_meta", 11, 0, PortDirection::Output),
    ("fused_active_meta", 11, 0, PortDirection::Output),
    ("ref_active_data", 44, 0, PortDirection::Output),
    ("fused_active_data", 44, 0, PortDirection::Output),
    ("fused_use_prev", 1, 0, PortDirection::Output),
];

#[derive(Default)]
struct Coverage {
    shapes: u8,
    predicates: u8,
    previous_lanes: u8,
    active_alias: bool,
    saturated_area: bool,
}

impl Coverage {
    fn complete(&self) -> bool {
        self.shapes == 0b1111
            && self.predicates == u8::MAX
            && self.previous_lanes == 0b11
            && self.active_alias
            && self.saturated_area
    }
}

#[derive(Clone, Copy)]
struct Location {
    frame: u16,
    y: usize,
    pair: usize,
}

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn with_pair_fused(
    test: impl for<'ctx> FnOnce(&mut DynamicVerilatedModel<'ctx>) -> TestResult,
) -> TestResult {
    let root = gateware_root();
    let adapter = root.join(ADAPTER);
    with_model(
        &root,
        Path::new("rtl/lrcca/sources.f"),
        "lrcca-pair-fused-equivalence",
        "PairFusedEquiv",
        Path::new(ADAPTER),
        PORTS,
        &[Path::new("rtl/lrcca")],
        &[adapter.as_path()],
        test,
    )
}

fn value(model: &DynamicVerilatedModel<'_>, name: &str) -> TestResult<u64> {
    read_u64(model, name)
}

fn assert_pair(
    model: &DynamicVerilatedModel<'_>,
    reference: &str,
    fused: &str,
    location: Location,
) -> TestResult<u64> {
    let reference_value = value(model, reference)?;
    let fused_value = value(model, fused)?;
    assert_eq!(
        reference_value, fused_value,
        "{reference}/{fused} mismatch at frame={:#06x}, y={}, pair={}",
        location.frame, location.y, location.pair
    );
    Ok(reference_value)
}

fn active_alias(active_meta: u64, state_meta: u64) -> bool {
    let valid = active_meta & (1 << 11) != 0;
    let mapped = active_meta & (1 << 5) != 0;
    let head = (active_meta >> 3) & 0b11;
    let state_valid = state_meta & (1 << 4) != 0;
    let state_head = (state_meta >> 2) & 0b11;
    valid && mapped && state_valid && head == state_head
}

fn check_equivalent(
    model: &DynamicVerilatedModel<'_>,
    location: Location,
    coverage: &mut Coverage,
) -> TestResult {
    assert_pair(model, "ref_ready", "fused_ready", location)?;
    let s5_control = assert_pair(model, "ref_s5_control", "fused_s5_control", location)?;
    if s5_control & 0b10 != 0 {
        assert_pair(model, "ref_s5_data", "fused_s5_data", location)?;
    }
    let close_control = assert_pair(model, "ref_close_control", "fused_close_control", location)?;
    if close_control & 0b10 != 0 {
        assert_pair(model, "ref_close_data", "fused_close_data", location)?;
    }
    let fv_valid = assert_pair(model, "ref_fv_valid", "fused_fv_valid", location)?;
    if fv_valid != 0 {
        assert_pair(model, "ref_fv_data", "fused_fv_data", location)?;
        assert_pair(model, "ref_fv_child", "fused_fv_child", location)?;
    }
    assert_pair(model, "ref_child_debug", "fused_child_debug", location)?;

    let predicates = assert_pair(model, "ref_predicates", "fused_predicates", location)?;
    let shape = assert_pair(model, "ref_s5_shape", "fused_s5_shape", location)?;
    assert_pair(
        model,
        "ref_lane_state_meta",
        "fused_lane_state_meta",
        location,
    )?;
    let lane_data = assert_pair(
        model,
        "ref_lane_state_data",
        "fused_lane_state_data",
        location,
    )?;
    let write_control = assert_pair(model, "ref_write_control", "fused_write_control", location)?;
    if write_control & (1 << 10) != 0 {
        assert_pair(model, "ref_write_data0", "fused_write_data0", location)?;
    }
    if write_control & (1 << 7) != 0 {
        assert_pair(model, "ref_write_data1", "fused_write_data1", location)?;
    }
    let registered_meta = assert_pair(
        model,
        "ref_registered_state_meta",
        "fused_registered_state_meta",
        location,
    )?;
    assert_pair(
        model,
        "ref_registered_state_data",
        "fused_registered_state_data",
        location,
    )?;
    let active_meta = assert_pair(model, "ref_active_meta", "fused_active_meta", location)?;
    let reference_alias = active_alias(active_meta, registered_meta);
    let fused_meta = value(model, "fused_active_meta")?;
    let fused_state_meta = value(model, "fused_registered_state_meta")?;
    let fused_alias = active_alias(fused_meta, fused_state_meta);
    if active_meta & (1 << 11) != 0 && !(reference_alias && fused_alias) {
        assert_pair(model, "ref_active_data", "fused_active_data", location)?;
    }
    if fused_alias {
        assert_eq!(
            value(model, "fused_active_data")?,
            value(model, "fused_registered_state_data")?,
            "fused active alias is not canonical at frame={:#06x}, y={}, pair={}",
            location.frame,
            location.y,
            location.pair
        );
    }

    let use_previous = value(model, "fused_use_prev")? as u8;
    assert_ne!(
        use_previous, 0b11,
        "both previous payloads selected at frame={:#06x}, y={}, pair={}",
        location.frame, location.y, location.pair
    );
    if shape & 0b100 != 0 {
        coverage.shapes |= 1 << (shape & 0b11);
        coverage.predicates |= predicates as u8;
        coverage.previous_lanes |= use_previous;
        coverage.active_alias |= fused_alias;
        coverage.saturated_area |= lane_data & 0b111 == 0b111;
    }
    Ok(())
}

fn tick(
    model: &mut DynamicVerilatedModel<'_>,
    location: Location,
    coverage: &mut Coverage,
) -> TestResult {
    model.pin("clk", 0_u8)?;
    model.eval();
    model.pin("clk", 1_u8)?;
    model.eval();
    check_equivalent(model, location, coverage)?;
    model.pin("clk", 0_u8)?;
    model.eval();
    Ok(())
}

fn drive_idle(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    for port in [
        "datavalid",
        "d0",
        "d1",
        "feature_d0",
        "feature_d1",
        "sof",
        "eol",
    ] {
        model.pin(port, 0_u8)?;
    }
    model.pin("feature_value_d0", 0_u8)?;
    model.pin("feature_value_d1", 0_u8)?;
    model.pin("feature_y", 0_u8)?;
    Ok(())
}

fn pixel(frame: u16, x: usize, y: usize) -> u8 {
    match (x + 3 * y + usize::from(frame)) & 7 {
        0 => 0,
        1 => 1,
        2 => 2,
        3 => 3,
        4 => 127,
        5 => 128,
        6 => 254,
        _ => 255,
    }
}

fn interior_bit(frame: u16, x: usize, y: usize) -> u8 {
    if (1..WIDTH - 1).contains(&x) && (1..HEIGHT - 1).contains(&y) {
        ((frame >> ((y - 1) * 4 + (x - 1))) & 1) as u8
    } else {
        0
    }
}

fn run_frames(model: &mut DynamicVerilatedModel<'_>, frames: &[u16]) -> TestResult<Coverage> {
    let mut coverage = Coverage::default();
    let mut location = Location {
        frame: 0,
        y: 0,
        pair: 0,
    };
    drive_idle(model)?;
    model.pin("rst", 1_u8)?;
    for _ in 0..3 {
        tick(model, location, &mut coverage)?;
    }
    model.pin("rst", 0_u8)?;
    for _ in 0..3 {
        tick(model, location, &mut coverage)?;
    }

    for &frame in frames {
        location.frame = frame;
        for y in 0..HEIGHT {
            location.y = y;
            for pair in 0..WIDTH / 2 {
                location.pair = pair;
                let mut wait_cycles = 0;
                while value(model, "ref_ready")? == 0 || value(model, "fused_ready")? == 0 {
                    drive_idle(model)?;
                    tick(model, location, &mut coverage)?;
                    wait_cycles += 1;
                    if wait_cycles > 256 {
                        return Err(format!(
                            "pair-fused cores did not become ready for frame {frame:#06x}, y={y}, pair={pair}"
                        )
                        .into());
                    }
                }
                let x0 = pair * 2;
                let d0 = interior_bit(frame, x0, y);
                let d1 = interior_bit(frame, x0 + 1, y);
                model.pin("datavalid", 1_u8)?;
                model.pin("d0", d0)?;
                model.pin("d1", d1)?;
                model.pin("feature_d0", d0)?;
                model.pin("feature_d1", d1)?;
                model.pin("feature_value_d0", pixel(frame, x0, y))?;
                model.pin("feature_value_d1", pixel(frame, x0 + 1, y))?;
                model.pin("feature_y", y as u8)?;
                model.pin("sof", u8::from(y == 0 && pair == 0))?;
                model.pin("eol", u8::from(pair + 1 == WIDTH / 2))?;
                tick(model, location, &mut coverage)?;
            }
        }
        drive_idle(model)?;
        for _ in 0..32 {
            tick(model, location, &mut coverage)?;
        }
    }
    Ok(coverage)
}

fn directed_frames() -> Vec<u16> {
    let mut frames = vec![
        0x0000, 0xffff, 0xaaaa, 0x5555, 0x6996, 0x9669, 0xf00f, 0x0ff0, 0x8421, 0x1248, 0x0001,
        0x8000, 0x0180, 0x1881, 0x7ffe, 0xe007,
    ];
    frames.extend(0_u16..=255);
    let mut state = 0x9e37_79b9_u32;
    for _ in 0..512 {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        frames.push(state as u16);
    }
    frames
}

#[test]
fn pair_fused_s5_matches_default_for_directed_4x4_interiors() -> TestResult {
    with_pair_fused(|model| {
        let coverage = run_frames(model, &directed_frames())?;
        assert!(
            coverage.complete(),
            "directed pair-fused coverage incomplete: shapes={:04b}, predicates={:08b}, previous={:02b}, alias={}, saturated={}",
            coverage.shapes,
            coverage.predicates,
            coverage.previous_lanes,
            coverage.active_alias,
            coverage.saturated_area
        );
        Ok(())
    })
}

#[test]
#[ignore = "exhaustively compares all 65,536 binary 4x4 interiors cycle by cycle"]
fn pair_fused_s5_matches_default_for_every_4x4_interior() -> TestResult {
    with_pair_fused(|model| {
        let frames = (0_u32..=u32::from(u16::MAX))
            .map(|frame| frame as u16)
            .collect::<Vec<_>>();
        let coverage = run_frames(model, &frames)?;
        assert!(
            coverage.complete(),
            "exhaustive pair-fused coverage target missing"
        );
        Ok(())
    })
}
