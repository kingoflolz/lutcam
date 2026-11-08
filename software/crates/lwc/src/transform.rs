//! Fixed, bit-exact analysis and synthesis transform used by the codec.
//!
//! The transform is a five-level horizontal, three-level vertical reversible
//! CDF 5/3 lifting transform. Samples and coefficients remain integers; the
//! quantization is applied later by the scalar packet layer.

pub(crate) const HORIZONTAL_LEVELS: usize = 5;
pub(crate) const VERTICAL_LEVELS: usize = 3;

const SAMPLE_CENTER: i32 = 128;

/// One row-major image plane emitted by the shared wavelet/TagDetect
/// frontend reference.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FrontendPlane {
    pub width: usize,
    pub height: usize,
    pub samples: Vec<i32>,
}

impl FrontendPlane {
    pub fn row(&self, y: usize) -> &[i32] {
        let start = y * self.width;
        &self.samples[start..start + self.width]
    }
}

/// Bit-exact low-pass planes made available to TagDetect by the LWC analysis
/// frontend. `stage0` remains in the unsigned camera domain. The other planes
/// are centered signed coefficients in the same numerical domain as the q14
/// RTL datapath.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct UnifiedFrontendReference {
    pub stage0: FrontendPlane,
    pub ll2: FrontendPlane,
    pub h4_low: FrontendPlane,
    pub ll4: FrontendPlane,
}

/// Compute the TagDetect taps without changing any of the normative LWC
/// coefficient math. LL2 is the low output after H2/V2. H4-low is the low
/// output after H3/V3/H4. LL4 averages adjacent H4-low rows with signed
/// round-to-nearest/half-up arithmetic: `(a + b + 1) >> 1`.
pub fn unified_frontend_reference(
    pixels: &[u8],
    width: usize,
    height: usize,
) -> Result<UnifiedFrontendReference, String> {
    if width == 0 || !width.is_multiple_of(1 << HORIZONTAL_LEVELS) {
        return Err("frontend width must be nonzero and divisible by 32".into());
    }
    if height == 0 || !height.is_multiple_of(16) {
        return Err("frontend height must be nonzero and divisible by 16".into());
    }
    let expected = width
        .checked_mul(height)
        .ok_or("frontend dimensions overflow usize")?;
    if pixels.len() != expected {
        return Err(format!(
            "frontend pixel count mismatch: expected {expected}, got {}",
            pixels.len()
        ));
    }

    let stage0 = FrontendPlane {
        width,
        height,
        samples: pixels.iter().map(|&pixel| i32::from(pixel)).collect(),
    };
    let mut low = FrontendPlane {
        width,
        height,
        samples: pixels
            .iter()
            .map(|&pixel| i32::from(pixel) - SAMPLE_CENTER)
            .collect(),
    };

    low = low_pass_horizontal(low);
    low = low_pass_vertical(low);
    low = low_pass_horizontal(low);
    low = low_pass_vertical(low);
    let ll2 = low.clone();

    low = low_pass_horizontal(low);
    low = low_pass_vertical(low);
    let h4_low = low_pass_horizontal(low);
    let ll4 = average_adjacent_rows(&h4_low);

    Ok(UnifiedFrontendReference {
        stage0,
        ll2,
        h4_low,
        ll4,
    })
}

fn low_pass_horizontal(source: FrontendPlane) -> FrontendPlane {
    debug_assert!(source.width >= 2 && source.width.is_multiple_of(2));
    let width = source.width / 2;
    let mut samples = Vec::with_capacity(width * source.height);
    for row in source.samples.chunks_exact(source.width) {
        let mut transformed = row.to_vec();
        forward_53_1d(&mut transformed);
        samples.extend_from_slice(&transformed[..width]);
    }
    FrontendPlane {
        width,
        height: source.height,
        samples,
    }
}

fn low_pass_vertical(source: FrontendPlane) -> FrontendPlane {
    debug_assert!(source.height >= 2 && source.height.is_multiple_of(2));
    let height = source.height / 2;
    let mut samples = vec![0; source.width * height];
    for x in 0..source.width {
        let mut column = (0..source.height)
            .map(|y| source.samples[y * source.width + x])
            .collect::<Vec<_>>();
        forward_53_1d(&mut column);
        for y in 0..height {
            samples[y * source.width + x] = column[y];
        }
    }
    FrontendPlane {
        width: source.width,
        height,
        samples,
    }
}

fn average_adjacent_rows(source: &FrontendPlane) -> FrontendPlane {
    debug_assert!(source.height >= 2 && source.height.is_multiple_of(2));
    let height = source.height / 2;
    let mut samples = Vec::with_capacity(source.width * height);
    for y in 0..height {
        samples.extend(
            source
                .row(2 * y)
                .iter()
                .zip(source.row(2 * y + 1))
                .map(|(&even, &odd)| (even + odd + 1) >> 1),
        );
    }
    FrontendPlane {
        width: source.width,
        height,
        samples,
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct Rect {
    pub x: usize,
    pub y: usize,
    pub width: usize,
    pub height: usize,
}

impl Rect {
    #[cfg(test)]
    pub(crate) fn len(self) -> usize {
        self.width * self.height
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BandKind {
    Ll,
    Horizontal,
    LowHigh,
    HighLow,
    HighHigh,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct Band {
    pub kind: BandKind,
    pub level: usize,
    pub rect: Rect,
}

/// Returns the final subbands in their normative page-stream order:
/// LL5, H5, H4, LH3, HL3, HH3, LH2, HL2, HH2, LH1, HL1, HH1.
pub(crate) fn bands(width: usize, height: usize) -> Vec<Band> {
    validate_dimensions(width, height);

    let band = |kind, level, x, y, band_width, band_height| Band {
        kind,
        level,
        rect: Rect {
            x,
            y,
            width: band_width,
            height: band_height,
        },
    };

    vec![
        band(BandKind::Ll, 5, 0, 0, width / 32, height / 8),
        band(
            BandKind::Horizontal,
            5,
            width / 32,
            0,
            width / 32,
            height / 8,
        ),
        band(
            BandKind::Horizontal,
            4,
            width / 16,
            0,
            width / 16,
            height / 8,
        ),
        band(BandKind::LowHigh, 3, 0, height / 8, width / 8, height / 8),
        band(BandKind::HighLow, 3, width / 8, 0, width / 8, height / 8),
        band(
            BandKind::HighHigh,
            3,
            width / 8,
            height / 8,
            width / 8,
            height / 8,
        ),
        band(BandKind::LowHigh, 2, 0, height / 4, width / 4, height / 4),
        band(BandKind::HighLow, 2, width / 4, 0, width / 4, height / 4),
        band(
            BandKind::HighHigh,
            2,
            width / 4,
            height / 4,
            width / 4,
            height / 4,
        ),
        band(BandKind::LowHigh, 1, 0, height / 2, width / 2, height / 2),
        band(BandKind::HighLow, 1, width / 2, 0, width / 2, height / 2),
        band(
            BandKind::HighHigh,
            1,
            width / 2,
            height / 2,
            width / 2,
            height / 2,
        ),
    ]
}

/// Centers unsigned samples around zero and applies the fixed 5H/3V analysis
/// transform in place.  Coefficients use the low-first Mallat layout described
/// by [`bands`].
pub(crate) fn forward_transform(data: &mut [i32], width: usize, height: usize) {
    validate_buffer(data, width, height);

    for value in data.iter_mut() {
        *value -= SAMPLE_CENTER;
    }

    let mut active_width = width;
    let mut active_height = height;
    for level in 1..=HORIZONTAL_LEVELS {
        for y in 0..active_height {
            let start = y * width;
            forward_53_1d(&mut data[start..start + active_width]);
        }

        if level <= VERTICAL_LEVELS {
            for x in 0..active_width {
                let mut column = (0..active_height)
                    .map(|y| data[y * width + x])
                    .collect::<Vec<_>>();
                forward_53_1d(&mut column);
                for (y, value) in column.into_iter().enumerate() {
                    data[y * width + x] = value;
                }
            }
            active_height /= 2;
        }
        active_width /= 2;
    }
}

/// Applies the exact inverse lifting steps and restores the unsigned sample
/// center.  With no intervening coefficient normalization this is an exact
/// integer round trip.
pub(crate) fn inverse_transform(data: &mut [i32], width: usize, height: usize) {
    validate_buffer(data, width, height);
    // One workspace serves every level. Vertical lifting works on contiguous
    // rows instead of allocating and gathering one cache-strided column at a
    // time; the horizontal inverse reuses its first row.
    let mut workspace = vec![0; data.len()];

    for level in (1..=HORIZONTAL_LEVELS).rev() {
        let active_width = width >> (level - 1);
        let active_height = height >> VERTICAL_LEVELS.min(level - 1);

        if level <= VERTICAL_LEVELS {
            inverse_53_columns(data, width, active_width, active_height, &mut workspace);
        }

        for y in 0..active_height {
            let start = y * width;
            inverse_53_1d_with_workspace(
                &mut data[start..start + active_width],
                &mut workspace[..active_width],
            );
        }
    }

    for value in data.iter_mut() {
        *value += SAMPLE_CENTER;
    }
}

fn inverse_53_columns(
    data: &mut [i32],
    stride: usize,
    width: usize,
    height: usize,
    workspace: &mut [i32],
) {
    let half = height / 2;
    let (low, high) = data[..height * stride].split_at_mut(half * stride);
    for (y, row) in low.chunks_exact_mut(stride).enumerate() {
        let previous = &high[y.saturating_sub(1) * stride..][..width];
        let current = &high[y * stride..][..width];
        for ((value, &left), &right) in row[..width].iter_mut().zip(previous).zip(current) {
            *value -= (left + right + 2) >> 2;
        }
    }
    for y in 0..half {
        let current = &low[y * stride..][..width];
        let next = &low[(y + 1).min(half - 1) * stride..][..width];
        let detail = &high[y * stride..][..width];
        let (even, odd) = workspace[2 * y * width..][..2 * width].split_at_mut(width);
        even.copy_from_slice(current);
        for (((value, &high), &left), &right) in odd.iter_mut().zip(detail).zip(current).zip(next) {
            *value = high + ((left + right) >> 1);
        }
    }
    for (row, reconstructed) in data
        .chunks_exact_mut(stride)
        .zip(workspace[..height * width].chunks_exact(width))
    {
        row[..width].copy_from_slice(reconstructed);
    }
}

fn validate_dimensions(width: usize, height: usize) {
    assert!(
        width > 0 && width.is_multiple_of(1 << HORIZONTAL_LEVELS),
        "transform width must be nonzero and divisible by 32"
    );
    assert!(
        height > 0 && height.is_multiple_of(1 << VERTICAL_LEVELS),
        "transform height must be nonzero and divisible by 8"
    );
}

fn validate_buffer(data: &[i32], width: usize, height: usize) {
    validate_dimensions(width, height);
    assert_eq!(
        data.len(),
        width
            .checked_mul(height)
            .expect("transform dimensions overflow usize"),
        "transform buffer length does not match its dimensions"
    );
}

fn forward_53_1d(values: &mut [i32]) {
    debug_assert!(values.len() >= 2 && values.len().is_multiple_of(2));
    let half = values.len() / 2;
    let mut low = Vec::with_capacity(half);
    let mut high = Vec::with_capacity(half);

    for pair in values.chunks_exact(2) {
        low.push(pair[0]);
        high.push(pair[1]);
    }

    // Predict: odd samples use adjacent even samples. At the right edge the
    // nearest even sample is duplicated.
    for i in 0..half {
        let left = low[i];
        let right = low[(i + 1).min(half - 1)];
        high[i] -= (left + right) >> 1;
    }

    // Update: even samples use adjacent odd samples. At the left edge the
    // nearest odd sample is duplicated.
    for i in 0..half {
        let left = high[i.saturating_sub(1)];
        let right = high[i];
        low[i] += (left + right + 2) >> 2;
    }

    values[..half].copy_from_slice(&low);
    values[half..].copy_from_slice(&high);
}

fn inverse_53_1d_with_workspace(values: &mut [i32], workspace: &mut [i32]) {
    debug_assert!(values.len() >= 2 && values.len().is_multiple_of(2));
    let half = values.len() / 2;
    let (low, high) = values.split_at_mut(half);

    // Reverse the update and predict in the opposite order using the exact
    // same integer rounding as analysis.
    for i in 0..half {
        let left = high[i.saturating_sub(1)];
        let right = high[i];
        low[i] -= (left + right + 2) >> 2;
    }

    for i in 0..half {
        workspace[2 * i] = low[i];
        workspace[2 * i + 1] = high[i] + ((low[i] + low[(i + 1).min(half - 1)]) >> 1);
    }
    values.copy_from_slice(&workspace[..values.len()]);
}

#[cfg(test)]
fn inverse_53_1d(values: &mut [i32]) {
    inverse_53_1d_with_workspace(values, &mut vec![0; values.len()]);
}

#[cfg(test)]
mod tests {
    use super::*;

    // Keep the original allocation-based scalar inverse as an independent
    // oracle for the row-wise vertical and shared-workspace implementation.
    fn reference_inverse_1d(values: &mut [i32]) {
        let half = values.len() / 2;
        let mut low = values[..half].to_vec();
        let mut high = values[half..].to_vec();
        for i in 0..half {
            low[i] -= (high[i.saturating_sub(1)] + high[i] + 2) >> 2;
        }
        for i in 0..half {
            high[i] += (low[i] + low[(i + 1).min(half - 1)]) >> 1;
        }
        for i in 0..half {
            values[2 * i] = low[i];
            values[2 * i + 1] = high[i];
        }
    }

    fn reference_inverse(data: &mut [i32], width: usize, height: usize) {
        for level in (1..=HORIZONTAL_LEVELS).rev() {
            let active_width = width >> (level - 1);
            let active_height = height >> VERTICAL_LEVELS.min(level - 1);
            if level <= VERTICAL_LEVELS {
                for x in 0..active_width {
                    let mut column = (0..active_height)
                        .map(|y| data[y * width + x])
                        .collect::<Vec<_>>();
                    reference_inverse_1d(&mut column);
                    for (y, value) in column.into_iter().enumerate() {
                        data[y * width + x] = value;
                    }
                }
            }
            for y in 0..active_height {
                reference_inverse_1d(&mut data[y * width..][..active_width]);
            }
        }
        for value in data {
            *value += SAMPLE_CENTER;
        }
    }

    #[test]
    fn inverse_workspace_matches_scalar_for_every_pixel() {
        let mut state = 0xd106e4b8u32;
        for (width, height) in [(32, 8), (64, 16), (128, 40), (1280, 800)] {
            for pattern in 0..4 {
                let mut actual = (0..width * height)
                    .map(|index| {
                        state ^= state << 13;
                        state ^= state >> 17;
                        state ^= state << 5;
                        match pattern {
                            0 => ((state & 65535) as i32) - 32768,
                            1 => {
                                if index % 2 == 0 {
                                    -16384
                                } else {
                                    16383
                                }
                            }
                            2 => {
                                if index % width == 0 || index % width == width - 1 {
                                    1023
                                } else {
                                    0
                                }
                            }
                            _ => {
                                if index < width || index >= width * (height - 1) {
                                    -1023
                                } else {
                                    0
                                }
                            }
                        }
                    })
                    .collect::<Vec<_>>();
                let mut expected = actual.clone();
                reference_inverse(&mut expected, width, height);
                inverse_transform(&mut actual, width, height);
                assert_eq!(actual, expected, "{width}x{height}, pattern {pattern}");
            }
        }
    }

    fn assert_round_trip(mut samples: Vec<i32>, width: usize, height: usize) {
        let original = samples.clone();
        forward_transform(&mut samples, width, height);
        inverse_transform(&mut samples, width, height);
        assert_eq!(samples, original);
    }

    #[test]
    fn reversible_53_lifting_and_rounding_are_frozen() {
        let original = vec![0, 1, 2, 3, 4, 5, 6, 7];
        let mut values = original.clone();
        forward_53_1d(&mut values);
        assert_eq!(values, vec![0, 2, 4, 6, 0, 0, 0, 1]);
        inverse_53_1d(&mut values);
        assert_eq!(values, original);
    }

    #[test]
    fn constant_image_centers_to_zero_and_round_trips() {
        let mut samples = vec![128; 64 * 16];
        forward_transform(&mut samples, 64, 16);
        assert!(samples.iter().all(|&coefficient| coefficient == 0));
        inverse_transform(&mut samples, 64, 16);
        assert!(samples.iter().all(|&sample| sample == 128));
    }

    #[test]
    fn unified_frontend_taps_match_the_normative_mallat_transform() {
        let width = 64;
        let height = 32;
        let pixels = (0..width * height)
            .map(|index| {
                let x = index % width;
                let y = index / width;
                ((17 * x + 29 * y + 3 * x * y + (x ^ y)) & 255) as u8
            })
            .collect::<Vec<_>>();
        let taps = unified_frontend_reference(&pixels, width, height).unwrap();

        assert_eq!((taps.ll2.width, taps.ll2.height), (width / 4, height / 4));
        assert_eq!(
            (taps.h4_low.width, taps.h4_low.height),
            (width / 16, height / 8)
        );
        assert_eq!((taps.ll4.width, taps.ll4.height), (width / 16, height / 16));

        let mut coefficients = pixels
            .iter()
            .map(|&value| i32::from(value))
            .collect::<Vec<_>>();
        forward_transform(&mut coefficients, width, height);

        // Undo H5 to expose the H4-low plane retained by the full transform.
        for y in 0..height / 8 {
            let start = y * width;
            inverse_53_1d(&mut coefficients[start..start + width / 16]);
        }
        let reconstructed_h4_low = (0..height / 8)
            .flat_map(|y| {
                coefficients[y * width..y * width + width / 16]
                    .iter()
                    .copied()
            })
            .collect::<Vec<_>>();
        assert_eq!(taps.h4_low.samples, reconstructed_h4_low);

        // Undo H4, V3, and H3 to recover the complete LL2 plane.
        for y in 0..height / 8 {
            let start = y * width;
            inverse_53_1d(&mut coefficients[start..start + width / 8]);
        }
        for x in 0..width / 4 {
            let mut column = (0..height / 4)
                .map(|y| coefficients[y * width + x])
                .collect::<Vec<_>>();
            inverse_53_1d(&mut column);
            for (y, value) in column.into_iter().enumerate() {
                coefficients[y * width + x] = value;
            }
        }
        for y in 0..height / 4 {
            let start = y * width;
            inverse_53_1d(&mut coefficients[start..start + width / 4]);
        }
        let reconstructed_ll2 = (0..height / 4)
            .flat_map(|y| {
                coefficients[y * width..y * width + width / 4]
                    .iter()
                    .copied()
            })
            .collect::<Vec<_>>();
        assert_eq!(taps.ll2.samples, reconstructed_ll2);
    }

    #[test]
    fn ll4_row_average_freezes_signed_half_up_rounding() {
        let source = FrontendPlane {
            width: 4,
            height: 2,
            samples: vec![-4, -3, -2, -1, -3, -2, -1, 0],
        };
        assert_eq!(average_adjacent_rows(&source).samples, vec![-3, -2, -1, 0]);
    }

    #[test]
    fn unified_frontend_constant_frame_has_zero_wavelet_taps() {
        let taps = unified_frontend_reference(&vec![128; 64 * 32], 64, 32).unwrap();
        assert!(taps.stage0.samples.iter().all(|&sample| sample == 128));
        assert!(taps.ll2.samples.iter().all(|&sample| sample == 0));
        assert!(taps.h4_low.samples.iter().all(|&sample| sample == 0));
        assert!(taps.ll4.samples.iter().all(|&sample| sample == 0));
    }

    #[test]
    fn impulse_round_trips_exactly() {
        let width = 64;
        let height = 16;
        let mut samples = vec![128; width * height];
        samples[7 * width + 31] = 255;
        samples[8 * width + 32] = 0;
        assert_round_trip(samples, width, height);
    }

    #[test]
    fn checkerboard_round_trips_exactly() {
        let width = 64;
        let height = 16;
        let samples = (0..height)
            .flat_map(|y| (0..width).map(move |x| if (x + y) & 1 == 0 { 0 } else { 255 }))
            .collect();
        assert_round_trip(samples, width, height);
    }

    #[test]
    fn deterministic_random_image_round_trips_exactly() {
        let width = 96;
        let height = 24;
        let mut state = 0x6a09_e667_f3bc_c909_u64;
        let samples = (0..width * height)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                (state & 0xff) as i32
            })
            .collect();
        assert_round_trip(samples, width, height);
    }

    #[test]
    fn band_descriptors_cover_the_transform_once() {
        let width = 1280;
        let height = 800;
        let descriptors = bands(width, height);
        assert_eq!(descriptors.len(), 12);

        let identities = descriptors
            .iter()
            .map(|band| (band.kind, band.level))
            .collect::<Vec<_>>();
        assert_eq!(
            identities,
            vec![
                (BandKind::Ll, 5),
                (BandKind::Horizontal, 5),
                (BandKind::Horizontal, 4),
                (BandKind::LowHigh, 3),
                (BandKind::HighLow, 3),
                (BandKind::HighHigh, 3),
                (BandKind::LowHigh, 2),
                (BandKind::HighLow, 2),
                (BandKind::HighHigh, 2),
                (BandKind::LowHigh, 1),
                (BandKind::HighLow, 1),
                (BandKind::HighHigh, 1),
            ]
        );
        let mut covered = vec![false; width * height];
        let mut total_area = 0;
        for band in descriptors {
            assert!(band.rect.x + band.rect.width <= width);
            assert!(band.rect.y + band.rect.height <= height);
            total_area += band.rect.len();
            for y in band.rect.y..band.rect.y + band.rect.height {
                for x in band.rect.x..band.rect.x + band.rect.width {
                    let index = y * width + x;
                    assert!(!covered[index], "overlapping coefficient at ({x}, {y})");
                    covered[index] = true;
                }
            }
        }
        assert_eq!(total_area, width * height);
        assert!(covered.into_iter().all(|is_covered| is_covered));
    }

    #[test]
    fn hh1_extremal_pattern_fits_the_signed_ten_bit_profile() {
        let width = 256;
        let height = 64;
        let mut samples = vec![255; width * height];
        for (x, y) in [
            (1, 0),
            (2, 0),
            (4, 0),
            (5, 0),
            (0, 1),
            (3, 1),
            (6, 1),
            (0, 2),
            (3, 2),
            (6, 2),
            (1, 3),
            (2, 3),
            (4, 3),
            (5, 3),
            (0, 4),
            (3, 4),
            (6, 4),
            (0, 5),
            (3, 5),
            (6, 5),
            (1, 6),
            (2, 6),
            (4, 6),
            (5, 6),
        ] {
            samples[y * width + x] = 0;
        }

        forward_transform(&mut samples, width, height);
        let coefficient = samples[33 * width + 129];
        let hh1 = bands(width, height).last().copied().unwrap();
        assert_eq!((hh1.kind, hh1.level), (BandKind::HighHigh, 1));
        assert_eq!(coefficient, 510);
    }
}
