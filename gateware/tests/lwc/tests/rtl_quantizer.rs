//! Marlin tests for the production coefficient quantizer.
//!
//! This replaces the package-level and quantizer SystemVerilog benches with
//! an independent Rust reference over every production-reachable coefficient.

mod rtl_support;

use std::path::Path;

use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};

use rtl_support::{TestResult, clock, read_u64, sign_extend, with_model};

const BAND_COUNT: usize = 12;
const Q_BITS: usize = 8;

const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("clear_frame", 0, 0, PortDirection::Input),
    ("in_valid", 11, 0, PortDirection::Input),
    ("in_value_0", 13, 0, PortDirection::Input),
    ("in_value_1", 13, 0, PortDirection::Input),
    ("in_value_2", 13, 0, PortDirection::Input),
    ("in_value_3", 13, 0, PortDirection::Input),
    ("in_value_4", 13, 0, PortDirection::Input),
    ("in_value_5", 13, 0, PortDirection::Input),
    ("in_value_6", 13, 0, PortDirection::Input),
    ("in_value_7", 13, 0, PortDirection::Input),
    ("in_value_8", 13, 0, PortDirection::Input),
    ("in_value_9", 13, 0, PortDirection::Input),
    ("in_value_10", 13, 0, PortDirection::Input),
    ("in_value_11", 13, 0, PortDirection::Input),
    ("out_valid", 11, 0, PortDirection::Output),
    ("out_q_0", 7, 0, PortDirection::Output),
    ("out_q_1", 7, 0, PortDirection::Output),
    ("out_q_2", 7, 0, PortDirection::Output),
    ("out_q_3", 7, 0, PortDirection::Output),
    ("out_q_4", 7, 0, PortDirection::Output),
    ("out_q_5", 7, 0, PortDirection::Output),
    ("out_q_6", 7, 0, PortDirection::Output),
    ("out_q_7", 7, 0, PortDirection::Output),
    ("out_q_8", 7, 0, PortDirection::Output),
    ("out_q_9", 7, 0, PortDirection::Output),
    ("out_q_10", 7, 0, PortDirection::Output),
    ("out_q_11", 7, 0, PortDirection::Output),
    ("out_correction", 11, 0, PortDirection::Output),
    ("out_residual_class", 11, 0, PortDirection::Output),
    ("protocol_error", 0, 0, PortDirection::Output),
];

const FINE_SHIFT: [u32; BAND_COUNT] = [1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4];
const COARSE_SHIFT: [u32; BAND_COUNT] = [2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5];
const FINE_BIAS: [i32; BAND_COUNT] = [0, 0, 0, 0, 1, 3, 2, 3, 6, 6, 6, 6];
const COARSE_BIAS: [i32; BAND_COUNT] = [1, 1, 3, 3, 3, 7, 7, 7, 15, 15, 17, 15];

fn quantize_reference(source: i32, shift: u32, bias: i32) -> i32 {
    let magnitude = source.abs();
    let rounded = (magnitude + bias) >> shift;
    let signed = if source < 0 { -rounded } else { rounded };
    signed.clamp(-128, 127)
}

fn correction_reference(band: usize, source: i32) -> bool {
    let fine = quantize_reference(source, FINE_SHIFT[band], FINE_BIAS[band]);
    let independent = quantize_reference(source, COARSE_SHIFT[band], COARSE_BIAS[band]);
    fine != 0 && independent.unsigned_abs() > (fine.unsigned_abs() >> 1)
}

fn residual_class_reference(band: usize, source: i32) -> bool {
    let shift = FINE_SHIFT[band];
    let fine = quantize_reference(source, shift, FINE_BIAS[band]);
    let error = source - (fine << shift);
    if fine.unsigned_abs() & 1 != 0 {
        error.unsigned_abs() * 4 >= 1_u32 << shift
    } else {
        fine * error >= 0
    }
}

fn pin_events(model: &mut DynamicVerilatedModel<'_>, events: &[(usize, i32)]) -> TestResult<u16> {
    let mut valid = 0_u16;
    for band in 0..BAND_COUNT {
        model.pin(format!("in_value_{band}"), 0_u16)?;
    }
    for &(band, source) in events {
        assert!(band < BAND_COUNT);
        assert!((-8192..=8191).contains(&source));
        assert_eq!(valid & (1 << band), 0, "band {band} driven twice");
        valid |= 1 << band;
        model.pin(format!("in_value_{band}"), (source as i16 as u16) & 0x3fff)?;
    }

    model.pin("in_valid", valid)?;
    model.eval();
    Ok(valid)
}

fn drive_events(model: &mut DynamicVerilatedModel<'_>, events: &[(usize, i32)]) -> TestResult {
    let mut expected = [None; BAND_COUNT];
    for &(band, source) in events {
        expected[band] = Some(source);
    }
    let valid = pin_events(model, events)?;

    assert_eq!(read_u64(model, "out_valid")?, u64::from(valid));
    let corrections = read_u64(model, "out_correction")?;
    let residual_classes = read_u64(model, "out_residual_class")?;
    // Payload and metadata are meaningful only on asserted out_valid lanes.
    // The production queues use those same bits as their write enables.
    for (band, source) in expected.into_iter().enumerate() {
        let actual_q = sign_extend(read_u64(model, &format!("out_q_{band}"))?, Q_BITS);
        let actual_correction = corrections & (1 << band) != 0;
        let actual_residual_class = residual_classes & (1 << band) != 0;
        if let Some(source) = source {
            let expected_q = i64::from(quantize_reference(
                source,
                FINE_SHIFT[band],
                FINE_BIAS[band],
            ));
            assert_eq!(
                actual_q, expected_q,
                "quantized value mismatch for band {band}, source {source}"
            );
            assert_eq!(
                actual_correction,
                correction_reference(band, source),
                "correction mismatch for band {band}, source {source}"
            );
            assert_eq!(
                actual_residual_class,
                residual_class_reference(band, source),
                "residual class mismatch for band {band}, source {source}"
            );
        }
    }
    Ok(())
}

fn initialize(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("rst", 1_u8)?;
    model.pin("clear_frame", 0_u8)?;
    drive_events(model, &[])?;
    for _ in 0..3 {
        clock(model)?;
    }
    model.pin("rst", 0_u8)?;
    model.eval();
    assert_eq!(read_u64(model, "protocol_error")?, 0);
    Ok(())
}

fn check_exhaustive_values(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    // Each batch contains at most one event from every physical scalar lane.
    // Across four batches, every band sees the complete signed 14-bit domain.
    for source in -8192..=8191 {
        drive_events(model, &[6, 8, 11, 0, 1].map(|band| (band, source)))?;
        drive_events(model, &[7, 10, 2, 5].map(|band| (band, source)))?;
        drive_events(model, &[9, 3].map(|band| (band, source)))?;
        drive_events(model, &[(4, source)])?;
    }
    assert_eq!(read_u64(model, "protocol_error")?, 0);
    Ok(())
}

fn check_all_legal_lane_combinations(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    // Cartesian product of the five physical lane choices: 3*2*1*4*2 = 48.
    for lane_0 in [6, 7, 9] {
        for lane_1 in [8, 10] {
            for lane_3 in [0, 2, 3, 4] {
                for lane_4 in [1, 5] {
                    let bands = [lane_0, lane_1, 11, lane_3, lane_4];
                    let events = bands.map(|band| {
                        let source =
                            ((band * 1237 + lane_0 * 97 + lane_3 * 41) % 16384) as i32 - 8192;
                        (band, source)
                    });
                    drive_events(model, &events)?;
                }
            }
        }
    }
    assert_eq!(read_u64(model, "protocol_error")?, 0);
    Ok(())
}

fn check_illegal_conflicts_and_sticky_error(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    const ILLEGAL_PAIRS: [(usize, usize); 11] = [
        (6, 7),
        (6, 9),
        (7, 9),
        (8, 10),
        (0, 2),
        (0, 3),
        (0, 4),
        (2, 3),
        (2, 4),
        (3, 4),
        (1, 5),
    ];

    for (first, second) in ILLEGAL_PAIRS {
        model.pin("clear_frame", 1_u8)?;
        drive_events(model, &[])?;
        clock(model)?;
        assert_eq!(read_u64(model, "protocol_error")?, 0);

        model.pin("clear_frame", 0_u8)?;
        pin_events(model, &[(first, -8192), (second, 8191)])?;
        clock(model)?;
        assert_eq!(
            read_u64(model, "protocol_error")?,
            1,
            "conflicting bands {first} and {second} were not rejected"
        );

        drive_events(model, &[])?;
        clock(model)?;
        assert_eq!(
            read_u64(model, "protocol_error")?,
            1,
            "protocol_error did not remain sticky after bands {first}/{second}"
        );
    }

    model.pin("clear_frame", 1_u8)?;
    clock(model)?;
    assert_eq!(read_u64(model, "protocol_error")?, 0);
    model.pin("clear_frame", 0_u8)?;
    Ok(())
}

#[test]
fn marlin_quantizer_matches_reference_and_enforces_coloring() -> TestResult {
    with_model(
        "coefficient_quantizer",
        "coefficient_quantizer_adapter",
        "tests/rtl_adapters/coefficient_quantizer_adapter.sv",
        PORTS,
        &[Path::new(
            "tests/rtl_adapters/coefficient_quantizer_adapter.sv",
        )],
        |model| {
            initialize(model)?;
            check_exhaustive_values(model)?;
            check_all_legal_lane_combinations(model)?;
            check_illegal_conflicts_and_sticky_error(model)
        },
    )
}
