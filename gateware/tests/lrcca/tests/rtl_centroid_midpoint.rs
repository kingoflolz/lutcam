//! Marlin/Verilator parity for the production midpoint-reflection reciprocal.

use std::fs;
use std::path::{Path, PathBuf};

use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, read_u64, with_model};

const FRACTION_BITS: usize = 21;
const SEGMENT_BITS: usize = 9;
const RESIDUAL_BITS: usize = FRACTION_BITS - SEGMENT_BITS;
const DISCARD_BITS: usize = 3;
const PRODUCT_SHIFT: usize = 19;
const FRACTION_COUNT: u32 = 1 << FRACTION_BITS;
const SEGMENT_COUNT: usize = 1 << SEGMENT_BITS;
const ADAPTER: &str = "tests/lrcca/tests/rtl_adapters/centroid_midpoint_reflection.sv";
const ROM: &str = "rtl/lrcca/centroid_reciprocal_midpoint_reflection_q17.bin";

const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("normalized_fraction", 20, 0, PortDirection::Input),
    ("coefficient", 17, 0, PortDirection::Input),
    ("reciprocal", 16, 0, PortDirection::Output),
];

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn coefficients(root: &Path) -> TestResult<Vec<u32>> {
    let text = fs::read_to_string(root.join(ROM))?;
    let lines = text.lines().collect::<Vec<_>>();
    if lines.len() != SEGMENT_COUNT {
        return Err(format!(
            "midpoint coefficient ROM has {} rows, expected {SEGMENT_COUNT}",
            lines.len()
        )
        .into());
    }
    lines
        .iter()
        .enumerate()
        .map(|(index, line)| {
            if line.len() != 20 || !line.bytes().all(|byte| matches!(byte, b'0' | b'1')) {
                return Err(format!("malformed midpoint ROM row {index}: {line:?}").into());
            }
            let word = u32::from_str_radix(line, 2)?;
            if word >> 18 != 0 {
                return Err(format!("nonzero coefficient padding in ROM row {index}").into());
            }
            Ok(word)
        })
        .collect()
}

fn expected_reciprocal(value: u32, coefficient: u32) -> u32 {
    let segment = value >> RESIDUAL_BITS;
    let residual_mask = (1_u32 << RESIDUAL_BITS) - 1;
    let reflected = ((1_u64 << FRACTION_BITS)
        + (u64::from(segment) << RESIDUAL_BITS)
        + u64::from(residual_mask)
        - u64::from(value & residual_mask))
        >> DISCARD_BITS;
    ((u64::from(coefficient) * reflected) >> PRODUCT_SHIFT) as u32
}

fn check_value(
    model: &mut DynamicVerilatedModel<'_>,
    table: &[u32],
    value: u32,
) -> TestResult<u32> {
    let segment = (value >> RESIDUAL_BITS) as usize;
    let coefficient = table[segment];
    model.pin("normalized_fraction", value)?;
    model.pin("coefficient", coefficient)?;
    model.eval();
    let actual = read_u64(model, "reciprocal")? as u32;
    let expected = expected_reciprocal(value, coefficient);
    assert_eq!(
        actual, expected,
        "midpoint reflection mismatch at Q21 fraction {value}, segment {segment}"
    );
    Ok(actual)
}

fn with_midpoint(
    test: impl for<'ctx> FnOnce(&mut DynamicVerilatedModel<'ctx>, &[u32]) -> TestResult,
) -> TestResult {
    let root = gateware_root();
    let table = coefficients(&root)?;
    let adapter = root.join(ADAPTER);
    with_model(
        &root,
        Path::new("rtl/lrcca/sources.f"),
        "lrcca-centroid-midpoint-reflection",
        "CentroidMidpointReflection",
        Path::new(ADAPTER),
        PORTS,
        &[Path::new("rtl/lrcca")],
        &[adapter.as_path()],
        |model| test(model, &table),
    )
}

#[test]
fn midpoint_reflection_rtl_covers_every_table_cell_and_directed_extremes() -> TestResult {
    with_midpoint(|model, table| {
        let residual_mask = (1_u32 << RESIDUAL_BITS) - 1;
        for segment in 0..SEGMENT_COUNT as u32 {
            let base = segment << RESIDUAL_BITS;
            for residual in [0, 1, residual_mask / 2, residual_mask - 1, residual_mask] {
                check_value(model, table, base + residual)?;
            }
        }

        for value in [
            0,
            1,
            (1 << RESIDUAL_BITS) - 1,
            1 << RESIDUAL_BITS,
            2_055_270,
            2_055_271,
            2_055_272,
            FRACTION_COUNT - 2,
            FRACTION_COUNT - 1,
        ] {
            check_value(model, table, value)?;
        }
        Ok(())
    })
}

#[test]
fn midpoint_reflection_rtl_matches_every_q21_cell() -> TestResult {
    with_midpoint(|model, table| {
        let normalized_scale = 1_u64 << FRACTION_BITS;
        let target = 1_u64 << (FRACTION_BITS + 17);
        let mut maximum_error = 0_u64;
        let mut worst_case = None;
        let mut minimum_reciprocal = 1_u32 << 17;
        let mut maximum_reciprocal = 0_u32;

        for value in 0..FRACTION_COUNT {
            let reciprocal = check_value(model, table, value)?;
            minimum_reciprocal = minimum_reciprocal.min(reciprocal);
            maximum_reciprocal = maximum_reciprocal.max(reciprocal);
            for endpoint in [value, value + 1] {
                let endpoint_denominator = normalized_scale + u64::from(endpoint);
                let product = u64::from(reciprocal) * endpoint_denominator;
                let error = product.abs_diff(target);
                if error > maximum_error {
                    maximum_error = error;
                    worst_case = Some((value, endpoint, reciprocal));
                }
            }
        }

        assert_eq!(maximum_error, 4_257_008);
        assert_eq!(worst_case, Some((2_055_271, 2_055_272, 66_198)));
        assert_eq!((minimum_reciprocal, maximum_reciprocal), (65_536, 131_071));
        Ok(())
    })
}
