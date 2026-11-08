//! Bit-exact fixed-point centroid compressor used by TagDetect.

use std::sync::OnceLock;

pub const DEFAULT_WIDTH: usize = 1280;
pub const DEFAULT_HEIGHT: usize = 800;
pub const DEFAULT_PIXEL_BITS: usize = 8;
pub const LUT_BITS: usize = 10;
pub const LUT_SIZE: usize = 1 << LUT_BITS;
pub const RECIPROCAL_SEGMENT_BITS: usize = 8;
pub const RECIPROCAL_SEGMENT_COUNT: usize = 1 << RECIPROCAL_SEGMENT_BITS;
pub const RECIPROCAL_FRAC_BITS: usize = 17;
pub const INTERNAL_FRAC_BITS: usize = 21;
pub const RECIPROCAL_MIDPOINT_SEGMENT_BITS: usize = 9;
pub const RECIPROCAL_MIDPOINT_SEGMENT_COUNT: usize = 1 << RECIPROCAL_MIDPOINT_SEGMENT_BITS;
pub const RECIPROCAL_MIDPOINT_RESIDUAL_BITS: usize =
    INTERNAL_FRAC_BITS - RECIPROCAL_MIDPOINT_SEGMENT_BITS;
pub const RECIPROCAL_MIDPOINT_DISCARD_BITS: usize = 3;
pub const RECIPROCAL_MIDPOINT_COEFFICIENT_BITS: usize = 18;
pub const RECIPROCAL_MIDPOINT_PRODUCT_SHIFT: usize = 19;
pub const RESIDUAL_FRAC_BITS: usize = 15;
pub const COEFF_FRAC_BITS: usize = 18;
pub const OUTPUT_FRAC_BITS: usize = 5;
pub const INTERNAL_OUTPUT_FRAC_BITS: usize = 4;
pub const NORMALIZED_MOMENT_FRAC_BITS: usize = 4;
pub const LOG_M_Q5_3_BITS: usize = 8;
pub const LOG_M_Q5_3_FRAC_BITS: usize = 3;
pub const A_LOG_Q18: i64 = 93_804;
pub const A_EXP_Q18: i64 = 91_227;
const POLY_INPUT_SHIFT: usize = 2;
const POLY_INTERMEDIATE_SHIFT: usize = COEFF_FRAC_BITS;
const POLY_ONE_MINUS_SHIFT: usize = 3;
const POLY_OUTPUT_SHIFT: usize = COEFF_FRAC_BITS + INTERNAL_FRAC_BITS
    - POLY_INPUT_SHIFT
    - POLY_INTERMEDIATE_SHIFT
    - POLY_ONE_MINUS_SHIFT;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CompressedCentroid {
    pub valid: bool,
    pub x_q5: u32,
    pub y_q5: u32,
}
impl CompressedCentroid {
    pub fn x(self) -> f64 {
        self.x_q5 as f64 / (1_u32 << OUTPUT_FRAC_BITS) as f64
    }
    pub fn y(self) -> f64 {
        self.y_q5 as f64 / (1_u32 << OUTPUT_FRAC_BITS) as f64
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CentroidErrorStats {
    pub count: usize,
    pub mean_x_error: f64,
    pub mean_y_error: f64,
    pub mean_abs_x_error: f64,
    pub mean_abs_y_error: f64,
    pub rms_x_error: f64,
    pub rms_y_error: f64,
    pub max_abs_x_error: f64,
    pub max_abs_y_error: f64,
}
impl CentroidErrorStats {
    pub fn mean_abs_error(self) -> f64 {
        (self.mean_abs_x_error + self.mean_abs_y_error) / 2.0
    }
    pub fn rms_error(self) -> f64 {
        ((self.rms_x_error.powi(2) + self.rms_y_error.powi(2)) / 2.0).sqrt()
    }
    pub fn max_abs_error(self) -> f64 {
        self.max_abs_x_error.max(self.max_abs_y_error)
    }
}

fn round_to_int(value: f64) -> i32 {
    if value >= 0.0 {
        (value + 0.5).floor() as i32
    } else {
        (value - 0.5).ceil() as i32
    }
}

fn derivative_roots<D: Fn(f64) -> f64>(derivative: &D, start: f64, stop: f64) -> Vec<f64> {
    let mut roots = Vec::new();
    let (mut previous_x, mut previous_y) = (start, derivative(start));
    if previous_y == 0.0 {
        roots.push(previous_x);
    }
    for segment in 1..=16 {
        let x = start + (stop - start) * segment as f64 / 16.0;
        let y = derivative(x);
        if y == 0.0 {
            roots.push(x);
        } else if previous_y * y < 0.0 {
            let (mut low, mut high, mut low_y) = (previous_x, x, previous_y);
            for _ in 0..64 {
                let middle = (low + high) / 2.0;
                let middle_y = derivative(middle);
                if low_y * middle_y <= 0.0 {
                    high = middle;
                } else {
                    low = middle;
                    low_y = middle_y;
                }
            }
            roots.push((low + high) / 2.0);
        }
        previous_x = x;
        previous_y = y;
    }
    roots
        .into_iter()
        .filter(|root| *root >= start && *root <= stop)
        .collect()
}

fn residual_table<F, D>(function: F, derivative: D) -> Vec<i32>
where
    F: Fn(f64) -> f64,
    D: Fn(f64) -> f64,
{
    (0..LUT_SIZE)
        .map(|index| {
            let start = index as f64 / LUT_SIZE as f64;
            let stop = (index + 1) as f64 / LUT_SIZE as f64;
            let mut candidates = vec![start, stop];
            candidates.extend(derivative_roots(&derivative, start, stop));
            let values: Vec<_> = candidates.into_iter().map(&function).collect();
            let min = values.iter().copied().fold(f64::INFINITY, f64::min);
            let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            round_to_int(((min + max) / 2.0) * (1_u32 << RESIDUAL_FRAC_BITS) as f64)
        })
        .collect()
}

pub fn log_residual_table() -> &'static [i32] {
    static TABLE: OnceLock<Vec<i32>> = OnceLock::new();
    TABLE.get_or_init(|| {
        let a = A_LOG_Q18 as f64 / (1_u32 << COEFF_FRAC_BITS) as f64;
        residual_table(
            |u| (1.0 + u).log2() - u - a * u * (1.0 - u),
            |u| 1.0 / ((1.0 + u) * 2.0_f64.ln()) - 1.0 - a + 2.0 * a * u,
        )
    })
}

pub fn exp_residual_table() -> &'static [i32] {
    static TABLE: OnceLock<Vec<i32>> = OnceLock::new();
    TABLE.get_or_init(|| {
        let a = A_EXP_Q18 as f64 / (1_u32 << COEFF_FRAC_BITS) as f64;
        residual_table(
            |z| 1.0 + z - 2.0_f64.powf(z) - a * z * (1.0 - z),
            |z| 1.0 - 2.0_f64.ln() * 2.0_f64.powf(z) - a + 2.0 * a * z,
        )
    })
}

fn reciprocal_endpoint_q17(index: usize) -> u32 {
    let denominator = RECIPROCAL_SEGMENT_COUNT + index;
    let rounded = (((1_u64 << RECIPROCAL_FRAC_BITS) * RECIPROCAL_SEGMENT_COUNT as u64)
        + denominator as u64 / 2)
        / denominator as u64;
    rounded.min((1_u64 << RECIPROCAL_FRAC_BITS) - 1) as u32
}

pub fn reciprocal_segments_q17() -> &'static [(u32, u32)] {
    static TABLE: OnceLock<Vec<(u32, u32)>> = OnceLock::new();
    TABLE.get_or_init(|| {
        let endpoints: Vec<_> = (0..=RECIPROCAL_SEGMENT_COUNT)
            .map(reciprocal_endpoint_q17)
            .collect();
        (0..RECIPROCAL_SEGMENT_COUNT)
            .map(|i| (endpoints[i], endpoints[i] - endpoints[i + 1]))
            .collect()
    })
}

fn normalized_fraction_q21(value: u64) -> (u32, u32) {
    let exponent = 63 - value.leading_zeros();
    let normalized = if exponent as usize >= INTERNAL_FRAC_BITS {
        value >> (exponent as usize - INTERNAL_FRAC_BITS)
    } else {
        value << (INTERNAL_FRAC_BITS - exponent as usize)
    };
    (
        exponent,
        (normalized - (1_u64 << INTERNAL_FRAC_BITS)) as u32,
    )
}

fn truncated_poly_q21(coefficient_q18: i64, value_q21: u32, one_minus_q21: u32) -> i64 {
    let value_hi = value_q21 as i64 >> POLY_INPUT_SHIFT;
    let scaled_hi = (coefficient_q18 * value_hi) >> POLY_INTERMEDIATE_SHIFT;
    let one_minus_hi = one_minus_q21 as i64 >> POLY_ONE_MINUS_SHIFT;
    (scaled_hi * one_minus_hi) >> POLY_OUTPUT_SHIFT
}

pub fn log2_approx_q21(value: u64) -> Result<u64, &'static str> {
    if value == 0 {
        return Err("log2 input must be positive");
    }
    let (exponent, u_q21) = normalized_fraction_q21(value);
    let index = u_q21 as usize >> (INTERNAL_FRAC_BITS - LUT_BITS);
    let one_minus = (1_u32 << INTERNAL_FRAC_BITS) - u_q21;
    let poly = truncated_poly_q21(A_LOG_Q18, u_q21, one_minus);
    let residual =
        (log_residual_table()[index] as i64) << (INTERNAL_FRAC_BITS - RESIDUAL_FRAC_BITS);
    Ok((((exponent as i64) << INTERNAL_FRAC_BITS) + u_q21 as i64 + poly + residual) as u64)
}

pub fn log_m_q5_3(value: u64) -> u8 {
    if value == 0 {
        return 0;
    }
    let quantized = log2_approx_q21(value).unwrap() >> (INTERNAL_FRAC_BITS - LOG_M_Q5_3_FRAC_BITS);
    quantized.min((1_u64 << LOG_M_Q5_3_BITS) - 1) as u8
}

// This preserves the hardware's frozen, non-monotonic transition at 1_749_040/1.
#[allow(clippy::if_same_then_else)]
pub fn log_m_q5_3_threshold(value: u64) -> u8 {
    if value == 0 {
        return 0;
    }
    let (exponent, u) = normalized_fraction_q21(value);
    let fraction = if u < 868_669 {
        if u < 396_812 {
            if u < 189_812 { 0 } else { 1 }
        } else if u < 622_506 {
            2
        } else {
            3
        }
    } else if u < 1_429_837 {
        if u < 1_137_124 { 4 } else { 5 }
    } else if u < 1_749_040 {
        6
    } else if u == 1_749_041 {
        6
    } else {
        7
    };
    if exponent as usize >= 1 << (LOG_M_Q5_3_BITS - LOG_M_Q5_3_FRAC_BITS) {
        u8::MAX
    } else {
        ((exponent as u8) << LOG_M_Q5_3_FRAC_BITS) | fraction
    }
}

pub fn reciprocal_q17(normalized_fraction_q21: u32) -> Result<u32, &'static str> {
    if normalized_fraction_q21 >= 1 << INTERNAL_FRAC_BITS {
        return Err("normalized fraction must be an unsigned Q21 value");
    }
    let low_bits = INTERNAL_FRAC_BITS - RECIPROCAL_SEGMENT_BITS;
    let segment = normalized_fraction_q21 as usize >> low_bits;
    let fraction = normalized_fraction_q21 & ((1 << low_bits) - 1);
    let (base, drop) = reciprocal_segments_q17()[segment];
    let interpolated = (drop as u64 * fraction as u64 + (1_u64 << (low_bits - 1))) >> low_bits;
    Ok(base - interpolated as u32)
}

fn midpoint_reflected_mantissa(normalized_fraction_q21: u32) -> u32 {
    let segment = normalized_fraction_q21 >> RECIPROCAL_MIDPOINT_RESIDUAL_BITS;
    let fraction = normalized_fraction_q21 & ((1 << RECIPROCAL_MIDPOINT_RESIDUAL_BITS) - 1);
    ((1_u32 << INTERNAL_FRAC_BITS)
        + (segment << RECIPROCAL_MIDPOINT_RESIDUAL_BITS)
        + ((1 << RECIPROCAL_MIDPOINT_RESIDUAL_BITS) - 1)
        - fraction)
        >> RECIPROCAL_MIDPOINT_DISCARD_BITS
}

fn midpoint_reflection_coefficient(index: usize) -> u32 {
    let normalized_scale = 1_u128 << INTERNAL_FRAC_BITS;
    let segment_width = 1_u128 << RECIPROCAL_MIDPOINT_RESIDUAL_BITS;
    let left = normalized_scale + ((index as u128) << RECIPROCAL_MIDPOINT_RESIDUAL_BITS);
    let midpoint_twice = 2 * left + segment_width - 1;
    let denominator = midpoint_twice * midpoint_twice;
    let nearest = ((1_u128 << 62) + denominator / 2) / denominator;
    let target = 1_i128 << (RECIPROCAL_FRAC_BITS + INTERNAL_FRAC_BITS);
    let segment_start = (index as u32) << RECIPROCAL_MIDPOINT_RESIDUAL_BITS;
    let mut best: Option<(i128, u32)> = None;
    for coefficient in [nearest, nearest + 1, nearest + 2] {
        let mut maximum_error = 0_i128;
        let mut maximum_reciprocal = 0_u128;
        for fraction in 0..segment_width as u32 {
            let normalized = segment_start + fraction;
            let reciprocal = (coefficient * midpoint_reflected_mantissa(normalized) as u128)
                >> RECIPROCAL_MIDPOINT_PRODUCT_SHIFT;
            maximum_reciprocal = maximum_reciprocal.max(reciprocal);
            let denominator_low = normalized_scale as i128 + normalized as i128;
            let reciprocal = reciprocal as i128;
            maximum_error = maximum_error
                .max((reciprocal * denominator_low - target).abs())
                .max((reciprocal * (denominator_low + 1) - target).abs());
        }
        if maximum_reciprocal >= 1 << RECIPROCAL_FRAC_BITS {
            continue;
        }
        let candidate = (maximum_error, coefficient as u32);
        if best.is_none_or(|old| candidate < old) {
            best = Some(candidate);
        }
    }
    best.expect("every reciprocal segment has a bounded coefficient")
        .1
}

pub fn midpoint_reflection_coefficients_q18() -> &'static [u32] {
    static TABLE: OnceLock<Vec<u32>> = OnceLock::new();
    TABLE.get_or_init(|| {
        (0..RECIPROCAL_MIDPOINT_SEGMENT_COUNT)
            .map(midpoint_reflection_coefficient)
            .collect()
    })
}

pub fn midpoint_reflection_reciprocal_q17(
    normalized_fraction_q21: u32,
) -> Result<u32, &'static str> {
    if normalized_fraction_q21 >= 1 << INTERNAL_FRAC_BITS {
        return Err("normalized fraction must be an unsigned Q21 value");
    }
    let segment = normalized_fraction_q21 as usize >> RECIPROCAL_MIDPOINT_RESIDUAL_BITS;
    let coefficient = midpoint_reflection_coefficients_q18()[segment] as u64;
    Ok(
        ((coefficient * midpoint_reflected_mantissa(normalized_fraction_q21) as u64)
            >> RECIPROCAL_MIDPOINT_PRODUCT_SHIFT) as u32,
    )
}

pub fn normalize_centered_moment_q4(centered_twice_moment: i128, exponent: u32) -> i128 {
    let shift = exponent as isize - NORMALIZED_MOMENT_FRAC_BITS as isize;
    if shift <= 0 {
        centered_twice_moment << -shift
    } else {
        (centered_twice_moment + (1_i128 << (shift - 1))) >> shift
    }
}

pub fn exp2_approx_q21(log_q21: i64) -> u64 {
    let exponent = log_q21 >> INTERNAL_FRAC_BITS;
    let z = log_q21 - (exponent << INTERNAL_FRAC_BITS);
    let index = z as usize >> (INTERNAL_FRAC_BITS - LUT_BITS);
    let one_minus = (1_i64 << INTERNAL_FRAC_BITS) - z;
    let mut h = truncated_poly_q21(A_EXP_Q18, z as u32, one_minus as u32);
    h += (exp_residual_table()[index] as i64) << (INTERNAL_FRAC_BITS - RESIDUAL_FRAC_BITS);
    let factor = (1_i64 << INTERNAL_FRAC_BITS) + z - h;
    if exponent >= 0 {
        (factor as u64) << exponent as usize
    } else {
        (factor as u64) >> -exponent as usize
    }
}

fn max_weight_sum(width: usize, height: usize, pixel_bits: usize) -> u128 {
    width as u128 * height as u128 * ((1_u128 << pixel_bits) - 1)
}

/// Return deterministic legal moment tuples for error and stress tests.
pub fn iter_random_centroid_moments(
    count: usize,
    seed: u64,
    width: usize,
    height: usize,
    pixel_bits: usize,
) -> Result<Vec<(u64, u64, u64)>, String> {
    if width == 0 || height == 0 || pixel_bits == 0 {
        return Err("width, height, and pixel_bits must be positive".into());
    }
    let max_m = u64::try_from(max_weight_sum(width, height, pixel_bits))
        .map_err(|_| "configured moment range exceeds u64")?;
    // Xorshift has an absorbing all-zero state, while the retired Python
    // helper's default seed is zero and still produces a varied sequence.
    let mut state = if seed == 0 {
        0x9e37_79b9_7f4a_7c15
    } else {
        seed
    };
    let mut next = || {
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        state.wrapping_mul(0x2545_f491_4f6c_dd1d)
    };
    let mut moments = Vec::with_capacity(count);
    for _ in 0..count {
        let m = 1 + next() % max_m;
        let sx_max = (width as u128 - 1) * m as u128;
        let sy_max = (height as u128 - 1) * m as u128;
        let sx = (next() as u128 % (sx_max + 1)) as u64;
        let sy = (next() as u128 % (sy_max + 1)) as u64;
        moments.push((m, sx, sy));
    }
    Ok(moments)
}

fn validate_inputs(
    m: u64,
    sx: u64,
    sy: u64,
    width: usize,
    height: usize,
    pixel_bits: usize,
) -> Result<(), String> {
    if width == 0 || height == 0 {
        return Err("width and height must be positive".into());
    }
    if pixel_bits == 0 {
        return Err("pixel_bits must be positive".into());
    }
    if m as u128 > max_weight_sum(width, height, pixel_bits) {
        return Err("M exceeds the configured image and pixel range".into());
    }
    if m == 0 {
        return if sx == 0 && sy == 0 {
            Ok(())
        } else {
            Err("zero-M centroid must have zero first moments".into())
        };
    }
    if sx as u128 > (width - 1) as u128 * m as u128 {
        return Err("Sx is outside the configured image range".into());
    }
    if sy as u128 > (height - 1) as u128 * m as u128 {
        return Err("Sy is outside the configured image range".into());
    }
    Ok(())
}

pub fn compress_centroid(
    m: u64,
    sx: u64,
    sy: u64,
    width: usize,
    height: usize,
    pixel_bits: usize,
    midpoint_reflection_reciprocal: bool,
) -> Result<CompressedCentroid, String> {
    validate_inputs(m, sx, sy, width, height, pixel_bits)?;
    if m == 0 {
        return Ok(CompressedCentroid::default());
    }
    let x2 = ((sx as i128) << 1) - (width - 1) as i128 * m as i128;
    let y2 = ((sy as i128) << 1) - (height - 1) as i128 * m as i128;
    let (exponent, fraction) = normalized_fraction_q21(m);
    let reciprocal = if midpoint_reflection_reciprocal {
        midpoint_reflection_reciprocal_q17(fraction).unwrap()
    } else {
        reciprocal_q17(fraction).unwrap()
    } as i128;
    let centered_q4 = |centered: i128| {
        let normalized = normalize_centered_moment_q4(centered, exponent);
        let product = normalized * reciprocal;
        let shift =
            NORMALIZED_MOMENT_FRAC_BITS + RECIPROCAL_FRAC_BITS + 1 - INTERNAL_OUTPUT_FRAC_BITS;
        (product + (1_i128 << (shift - 1))) >> shift
    };
    let x_q4 = (((width - 1) as i128) << (INTERNAL_OUTPUT_FRAC_BITS - 1)) + centered_q4(x2);
    let y_q4 = (((height - 1) as i128) << (INTERNAL_OUTPUT_FRAC_BITS - 1)) + centered_q4(y2);
    Ok(CompressedCentroid {
        valid: true,
        x_q5: (x_q4 << 1) as u32,
        y_q5: (y_q4 << 1) as u32,
    })
}

pub fn compress_centroid_default(m: u64, sx: u64, sy: u64) -> Result<CompressedCentroid, String> {
    compress_centroid(
        m,
        sx,
        sy,
        DEFAULT_WIDTH,
        DEFAULT_HEIGHT,
        DEFAULT_PIXEL_BITS,
        false,
    )
}

pub fn exact_centroid(m: u64, sx: u64, sy: u64) -> Result<(f64, f64), &'static str> {
    if m == 0 {
        Err("exact centroid requires positive M")
    } else {
        Ok((sx as f64 / m as f64, sy as f64 / m as f64))
    }
}

pub fn centroid_error_stats<I>(
    moments: I,
    width: usize,
    height: usize,
    pixel_bits: usize,
) -> Result<CentroidErrorStats, String>
where
    I: IntoIterator<Item = (u64, u64, u64)>,
{
    let mut count = 0;
    let (mut sx_err, mut sy_err, mut sax, mut say, mut ssx, mut ssy, mut maxx, mut maxy) = (
        0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64,
    );
    for (m, x, y) in moments {
        let result = compress_centroid(m, x, y, width, height, pixel_bits, false)?;
        if !result.valid {
            continue;
        }
        let (exact_x, exact_y) = exact_centroid(m, x, y).unwrap();
        let (ex, ey) = (result.x() - exact_x, result.y() - exact_y);
        count += 1;
        sx_err += ex;
        sy_err += ey;
        sax += ex.abs();
        say += ey.abs();
        ssx += ex * ex;
        ssy += ey * ey;
        maxx = maxx.max(ex.abs());
        maxy = maxy.max(ey.abs());
    }
    if count == 0 {
        return Err("at least one valid centroid is required".into());
    }
    let n = count as f64;
    Ok(CentroidErrorStats {
        count,
        mean_x_error: sx_err / n,
        mean_y_error: sy_err / n,
        mean_abs_x_error: sax / n,
        mean_abs_y_error: say / n,
        rms_x_error: (ssx / n).sqrt(),
        rms_y_error: (ssy / n).sqrt(),
        max_abs_x_error: maxx,
        max_abs_y_error: maxy,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q5_3_threshold_is_exact() {
        for value in 1..100_000 {
            assert_eq!(
                log_m_q5_3(value),
                log_m_q5_3_threshold(value),
                "value={value}"
            );
        }
        assert_eq!((log2_approx_q21(1).unwrap(), log_m_q5_3(65_536)), (64, 128));
    }

    #[test]
    fn reciprocal_endpoints_and_midpoint_match() {
        for value in [0, 1, 868_669, 1_749_041, (1 << 21) - 1] {
            assert_eq!(
                reciprocal_q17(value).unwrap(),
                midpoint_reflection_reciprocal_q17(value).unwrap()
            );
        }
        assert_eq!(reciprocal_q17(0).unwrap(), 131_071);
        assert_eq!(reciprocal_q17((1 << 21) - 1).unwrap(), 65_536);
    }

    #[test]
    fn frozen_centroid_vectors() {
        assert_eq!(
            compress_centroid(1, 0, 0, 48, 36, 8, false).unwrap(),
            CompressedCentroid {
                valid: true,
                x_q5: 0,
                y_q5: 0
            }
        );
        assert_eq!(
            compress_centroid(100, 1234, 1777, 48, 36, 8, false).unwrap(),
            CompressedCentroid {
                valid: true,
                x_q5: 394,
                y_q5: 568
            }
        );
        assert_eq!(
            compress_centroid(1000, 20_000, 30_000, 48, 36, 8, true).unwrap(),
            CompressedCentroid {
                valid: true,
                x_q5: 640,
                y_q5: 960
            }
        );
        assert!(!compress_centroid(0, 0, 0, 48, 36, 8, false).unwrap().valid);
    }

    #[test]
    fn default_random_moment_seed_is_varied_and_legal() {
        let moments = iter_random_centroid_moments(32, 0, 48, 36, 8).unwrap();
        assert!(moments.windows(2).any(|pair| pair[0] != pair[1]));
        for (m, sx, sy) in moments {
            validate_inputs(m, sx, sy, 48, 36, 8).unwrap();
        }
    }
}
