//! Deterministic generation of production geometry and pose-scale tables.

use crate::geometry::{Mat3, Point2, square_to_quad, symbol_offset};

pub const POSE_RECIPROCAL_SQRT_SEED_COUNT: usize = 512;

/// Guarded reciprocal-square-root value for the compact pose-scale ROM.
///
/// The normalized radicand lies in `[2^34, 2^36)`. Each 256-entry bank
/// covers one leading-bit parity: `[2^34, 2^35)` then `[2^35, 2^36)`.
/// The table stores `round(2^34 / sqrt(midpoint))`; eight low guard bits
/// are discarded while the hardware removes the exponent serially.
pub fn pose_reciprocal_sqrt_seed(index: usize) -> u32 {
    assert!(index < POSE_RECIPROCAL_SQRT_SEED_COUNT);
    let bank = index >> 8;
    let bin = index & 0xff;
    let midpoint =
        (1_u64 << (34 + bank)) as f64 + (bin as f64 + 0.5) * (1_u64 << (26 + bank)) as f64;
    ((1_u64 << 34) as f64 / midpoint.sqrt()).round() as u32
}

pub fn pose_reciprocal_sqrt_seeds() -> [u32; POSE_RECIPROCAL_SQRT_SEED_COUNT] {
    core::array::from_fn(pose_reciprocal_sqrt_seed)
}

/// Floating unit-square-to-canonical matrix for one four-bit hypothesis.
pub fn hypothesis_matrix(hypothesis: u8) -> Mat3 {
    assert!(hypothesis < 16);
    let bottom_left_symbol = hypothesis & 0b11;
    let bottom_right_symbol = hypothesis >> 2;
    square_to_quad([
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0) + symbol_offset(bottom_right_symbol),
        Point2::new(-1.0, 1.0) + symbol_offset(bottom_left_symbol),
    ])
    .unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hypothesis_index_inserts_bottom_symbols_directly() {
        for hypothesis in 0..16u8 {
            let matrix = hypothesis_matrix(hypothesis);
            let bottom_right = matrix.transform_point(Point2::new(1.0, 1.0)).unwrap();
            let bottom_left = matrix.transform_point(Point2::new(0.0, 1.0)).unwrap();
            let expected_right = Point2::new(1.0, 1.0) + symbol_offset(hypothesis >> 2);
            let expected_left = Point2::new(-1.0, 1.0) + symbol_offset(hypothesis & 3);
            assert!((bottom_right - expected_right).norm_squared() < 1.0e-24);
            assert!((bottom_left - expected_left).norm_squared() < 1.0e-24);
        }
    }
}
