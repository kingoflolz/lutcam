//! Check the checked-in centroid ROMs against the Rust arithmetic models.

use std::fs;
use std::path::PathBuf;

use lrcca::{
    RECIPROCAL_MIDPOINT_COEFFICIENT_BITS, RECIPROCAL_MIDPOINT_SEGMENT_COUNT,
    RECIPROCAL_SEGMENT_COUNT, midpoint_reflection_coefficients_q18, reciprocal_segments_q17,
};

fn rtl_data_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../rtl/lrcca")
        .join(name)
}

fn binary_lines(name: &str) -> Vec<String> {
    fs::read_to_string(rtl_data_path(name))
        .unwrap()
        .lines()
        .map(str::to_owned)
        .collect()
}

#[test]
fn checked_in_segment_table_matches_exact_generator_formula() {
    let lines = binary_lines("centroid_reciprocal_segments_q17.bin");
    assert_eq!(lines.len(), RECIPROCAL_SEGMENT_COUNT);
    assert!(
        lines
            .iter()
            .all(|line| line.len() == 20 && line.bytes().all(|byte| b"01".contains(&byte)))
    );

    let decoded = lines
        .iter()
        .enumerate()
        .map(|(index, line)| {
            let base_high = if index < 37 {
                3
            } else if index < 86 {
                2
            } else if index < 154 {
                1
            } else {
                0
            };
            let drop_high = if index < 68 {
                if index < 18 {
                    7
                } else if index < 40 {
                    6
                } else {
                    5
                }
            } else if index < 107 {
                4
            } else if index < 163 {
                3
            } else {
                2
            };
            (
                (1 << 16) | (base_high << 14) | u32::from_str_radix(&line[..14], 2).unwrap(),
                (drop_high << 6) | u32::from_str_radix(&line[14..], 2).unwrap(),
            )
        })
        .collect::<Vec<_>>();
    assert_eq!(decoded, reciprocal_segments_q17());
    assert_eq!(RECIPROCAL_SEGMENT_COUNT, 256);
    assert_eq!(decoded.iter().map(|&(base, _)| base).min(), Some(65_664));
    assert_eq!(decoded.iter().map(|&(base, _)| base).max(), Some(131_071));
    assert_eq!(decoded.iter().map(|&(_, drop)| drop).min(), Some(128));
    assert_eq!(decoded.iter().map(|&(_, drop)| drop).max(), Some(509));
    assert_eq!(
        (1..RECIPROCAL_SEGMENT_COUNT)
            .filter(|&index| decoded[index].0 >> 14 != decoded[index - 1].0 >> 14)
            .collect::<Vec<_>>(),
        [37, 86, 154]
    );
    assert_eq!(
        (1..RECIPROCAL_SEGMENT_COUNT)
            .filter(|&index| decoded[index].1 >> 6 != decoded[index - 1].1 >> 6)
            .collect::<Vec<_>>(),
        [18, 40, 68, 107, 163]
    );
}

#[test]
fn checked_in_midpoint_table_matches_exact_minimax_generator() {
    let lines = binary_lines("centroid_reciprocal_midpoint_reflection_q17.bin");
    assert_eq!(lines.len(), RECIPROCAL_MIDPOINT_SEGMENT_COUNT);
    assert!(
        lines
            .iter()
            .all(|line| line.len() == 20 && line.bytes().all(|byte| b"01".contains(&byte)))
    );
    assert!(lines.iter().all(|line| line.starts_with("00")));
    let coefficients = lines
        .iter()
        .map(|line| u32::from_str_radix(line, 2).unwrap())
        .collect::<Vec<_>>();
    assert_eq!(coefficients, midpoint_reflection_coefficients_q18());
    assert_eq!(coefficients.iter().copied().min(), Some(65_601));
    assert_eq!(coefficients.iter().copied().max(), Some(261_633));
    assert!(
        coefficients
            .iter()
            .all(|&coefficient| coefficient < 1 << RECIPROCAL_MIDPOINT_COEFFICIENT_BITS)
    );
    assert_eq!(lines.len() * lines[0].len(), 10_240);
}
