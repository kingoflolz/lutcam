//! LFTag direct-pose packet decoding and fixed-point pose reference.

use crate::rom::pose_reciprocal_sqrt_seed;

/// Number of 32-bit words in an accepted direct-pose packet.
pub const POSE_PACKET_WORDS: usize = 22;

/// First word of the row-major signed-Q29 rotation matrix.
pub const POSE_ROTATION_WORD: usize = 10;

/// First word of the signed-Q16 translation vector.
pub const POSE_TRANSLATION_WORD: usize = 19;

/// Errors detected by the fixed-point H-to-pose reference calculation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PoseMathError {
    InvalidSigned18Encoding,
    InvalidHomographyScale,
    InvalidLambda,
    InvalidPoseRange,
}

/// Errors detected while parsing a production direct-pose packet.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PosePacketError {
    WrongLength,
    InvalidStatus,
}

/// Fixed-point pose emitted by the production RTL.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FixedPose {
    /// Row-major object-to-camera rotation matrix in signed Q29.
    pub rotation_q29: [i32; 9],
    /// Object-to-camera translation in signed Q16. Its physical unit is the
    /// unit used for the configured printed-tag width.
    pub translation_q16: [i32; 3],
}

/// Parsed accepted LFTag result packet.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PosePacket {
    /// Q32 pooled squared error in tag-grid-pitch units.
    pub mean_decode_residual_q32: u32,
    /// The observed tag is small enough that depth conditioning is weak.
    pub weak_perspective: bool,
    /// Raw gateware pose-conditioning flag. Use [`FixedPose::conditioning_warning`]
    /// to check the transmitted pose's column norms directly.
    pub conditioning_warning: bool,
    /// One or more fixed-point geometry or pose operations saturated.
    pub saturation: bool,
    /// Direct fixed-point pose produced by the RTL.
    pub pose: FixedPose,
}

impl PosePacket {
    /// Parse an accepted direct-pose packet. Unused status bits are reserved and zero.
    pub fn from_packet(words: &[u32]) -> Result<Self, PosePacketError> {
        let Some(&status) = words.get(8) else {
            return Err(PosePacketError::WrongLength);
        };
        if words.len() != POSE_PACKET_WORDS {
            return Err(PosePacketError::WrongLength);
        }
        // Bit zero is the localization-valid bit. Accepted direct-pose packets
        // always contain a complete pose; rejected candidates stop at the
        // common eight-word header and therefore never reach this parser.
        if status & 1 == 0 || status & 0xffff_fff0 != 0 {
            return Err(PosePacketError::InvalidStatus);
        }

        Ok(Self {
            mean_decode_residual_q32: words[9],
            weak_perspective: status & (1 << 1) != 0,
            conditioning_warning: status & (1 << 2) != 0,
            saturation: status & (1 << 3) != 0,
            pose: FixedPose {
                rotation_q29: core::array::from_fn(|index| {
                    words[POSE_ROTATION_WORD + index] as i32
                }),
                translation_q16: core::array::from_fn(|index| {
                    words[POSE_TRANSLATION_WORD + index] as i32
                }),
            },
        })
    }
}

impl FixedPose {
    /// Extract only the pose from a complete accepted direct-pose packet.
    pub fn from_packet(words: &[u32]) -> Result<Self, PosePacketError> {
        Ok(PosePacket::from_packet(words)?.pose)
    }

    /// Whether the first two pose columns differ in norm by more than 1/8.
    ///
    /// Both columns use the same RMS normalization, so their norm ratio is
    /// also the homography column norm ratio used by the pose conditioning
    /// check. Compute directly from the transmitted columns.
    /// This describes pose quality, not whether the tag ID was decoded.
    pub fn conditioning_warning(self) -> bool {
        let norm_squared = |column: usize| -> u128 {
            (0..3)
                .map(|row| {
                    let value = i64::from(self.rotation_q29[row * 3 + column]);
                    (value * value) as u128
                })
                .sum()
        };
        let first = norm_squared(0);
        let second = norm_squared(1);
        // Squaring the positive 8*min(norm) < 7*max(norm) comparison avoids
        // square roots. u128 also covers arbitrary signed-i32 packet values.
        64 * first.min(second) < 49 * first.max(second)
    }
}

fn signed_18(word: u32) -> i32 {
    ((word << 14) as i32) >> 14
}

/// Derive the signed-Q10 reciprocal RMS column norm from common-scale H.
///
/// The implementation mirrors the direct-pose RTL: six squares, divide by
/// two, normalize by powers of four, then a reciprocal-square-root lookup
/// with 256 bins for each leading-bit parity. The RTL carries a positive
/// signed-18 mantissa and a separate binary exponent so small column norms
/// do not reject valid tags. This function returns their full Q10 value.
pub fn derive_lambda_q10(homography_common_q16: [i32; 9]) -> Result<i32, PoseMathError> {
    let (mantissa, left_shift) = derive_lambda_mantissa(homography_common_q16)?;
    Ok(mantissa << left_shift)
}

fn derive_lambda_mantissa(homography_common_q16: [i32; 9]) -> Result<(i32, u32), PoseMathError> {
    if homography_common_q16
        .iter()
        .any(|&value| !(-131_072..=131_071).contains(&value))
    {
        return Err(PoseMathError::InvalidSigned18Encoding);
    }
    if !(32_768..=131_071).contains(&homography_common_q16[8]) {
        return Err(PoseMathError::InvalidHomographyScale);
    }

    let column_indices = [0_usize, 3, 6, 1, 4, 7];
    let square_sum = column_indices.into_iter().fold(0_u64, |sum, index| {
        let value = i64::from(homography_common_q16[index]);
        sum + (value * value) as u64
    });
    let mut radicand = square_sum >> 1;
    let mut right_shift_count = 8_i32;

    loop {
        if radicand == 0 || radicand & (1_u64 << 36) != 0 {
            return Err(PoseMathError::InvalidLambda);
        }
        if radicand & (3_u64 << 34) != 0 {
            break;
        }
        radicand <<= 2;
        right_shift_count -= 1;
    }

    let address = if radicand & (1_u64 << 35) != 0 {
        256 | ((radicand >> 27) & 0xff) as usize
    } else {
        ((radicand >> 26) & 0xff) as usize
    };
    let lambda_q10 = pose_reciprocal_sqrt_seed(address) >> right_shift_count.max(0);
    if lambda_q10 == 0 || lambda_q10 >= (1 << 17) {
        return Err(PoseMathError::InvalidLambda);
    }
    Ok((lambda_q10 as i32, (-right_shift_count).max(0) as u32))
}

/// Reproduce the direct-pose RTL's fixed-point result from its internal H.
///
/// This intentionally uses the RTL truncation points, including the rounded
/// Q10 translation multiplier, rather than floating-point arithmetic.
pub fn pose_from_homography(
    homography_common_q16: [i32; 9],
    printed_width_q16: u32,
) -> Result<FixedPose, PoseMathError> {
    let (mantissa, left_shift) = derive_lambda_mantissa(homography_common_q16)?;
    let lambda = i64::from(mantissa);
    let h = homography_common_q16.map(i64::from);
    let r1: [i64; 3] = core::array::from_fn(|row| ((lambda * h[row * 3]) << left_shift) >> 10);
    let r2: [i64; 3] = core::array::from_fn(|row| ((lambda * h[row * 3 + 1]) << left_shift) >> 10);
    if r1
        .iter()
        .chain(&r2)
        .any(|value| !(-131_072..=131_071).contains(value))
    {
        return Err(PoseMathError::InvalidPoseRange);
    }
    let r3 = [
        (r1[1] * r2[2] - r1[2] * r2[1]) >> 16,
        (r1[2] * r2[0] - r1[0] * r2[2]) >> 16,
        (r1[0] * r2[1] - r1[1] * r2[0]) >> 16,
    ];
    if r3.iter().any(|value| !(-131_072..=131_071).contains(value)) {
        return Err(PoseMathError::InvalidPoseRange);
    }
    let rotation_q29 = core::array::from_fn(|index| {
        let row = index / 3;
        let column = index % 3;
        let value = [r1[row], r2[row], r3[row]][column] as i32;
        value.wrapping_shl(13)
    });

    let width_q16 = i64::from(signed_18(printed_width_q16));
    let translation_scale_q16 = (lambda * width_q16) >> 12;
    let translation_operand_q10 = (translation_scale_q16 + 32) >> 6;
    // Keep the multiplier operand bounded exactly as the RTL does. Apply the
    // exponent to its product, after this rounding step, not to the operand.
    let translation_wide: [i64; 3] = core::array::from_fn(|row| {
        ((translation_operand_q10 * h[row * 3 + 2]) << left_shift) >> 10
    });
    if translation_wide
        .iter()
        .any(|&value| i32::try_from(value).is_err())
    {
        return Err(PoseMathError::InvalidPoseRange);
    }
    let translation_q16 = translation_wide.map(|value| value as i32);
    Ok(FixedPose {
        rotation_q29,
        translation_q16,
    })
}
