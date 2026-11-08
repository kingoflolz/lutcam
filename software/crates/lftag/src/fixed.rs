//! Fixed-point transaction model and deterministic vectors shared with RTL tests.

use crate::corner_select::{CornerSelectStatus, select_sorted_streaming_corners};
use crate::decode::{Candidate, Decoder, Plane, RejectReason, WeightedCentroid};
use crate::geometry::{Mat3, Point2, canonical_points};

pub const SCORE_FRACTION_BITS: u32 = 32;

/// Planning watchdog for one accepted candidate. This is an interface
/// requirement, not a prediction of the software execution time.
pub const ACCEPTED_CYCLE_WATCHDOG: u16 = 1000;

/// Normal accepted-path compute target, excluding result backpressure.
pub const ACCEPTED_COMPUTE_TARGET: u16 = 900;

/// Maximum start-to-start service interval with an always-ready result sink.
pub const ACCEPTED_SERVICE_TARGET: u16 = 1000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoordinateError {
    Negative,
    FenceOrPadding,
    OutsideActivePlane,
    MixedPlane,
}

/// Saturate a signed value to the requested two's-complement width.
pub fn saturate_signed(value: i128, width: u32) -> (i64, bool) {
    assert!((1..=63).contains(&width));
    let minimum = -(1i128 << (width - 1));
    let maximum = (1i128 << (width - 1)) - 1;
    let clamped = value.clamp(minimum, maximum);
    (clamped as i64, clamped != value)
}

/// Quantize a finite floating value to signed fixed point.
pub fn quantize_signed(value: f64, fractional_bits: u32, width: u32) -> (i64, bool) {
    assert!(value.is_finite());
    let scaled = (value * (1u64 << fractional_bits) as f64).round();
    let scaled = scaled.clamp(i128::MIN as f64, i128::MAX as f64) as i128;
    saturate_signed(scaled, width)
}

/// Classify a composite TagDetect coordinate and remove the scale-plane
/// framing offset. Both inputs and the returned X coordinate are Q4.
pub fn classify_composite_q4(x_q4: i32, y_q4: i32) -> Result<(Plane, i32), CoordinateError> {
    if x_q4 < 0 || y_q4 < 0 {
        return Err(CoordinateError::Negative);
    }
    let raw_end = 1280 * 16;
    let ll2_active_start = 1284 * 16;
    let ll2_active_end = 1604 * 16;
    let ll4_active_start = 1612 * 16;
    let ll4_active_end = 1692 * 16;
    let (plane, logical_x) = if x_q4 < raw_end {
        (Plane::Raw, x_q4)
    } else if (ll2_active_start..ll2_active_end).contains(&x_q4) {
        (Plane::Ll2, x_q4 - ll2_active_start)
    } else if (ll4_active_start..ll4_active_end).contains(&x_q4) {
        (Plane::Ll4, x_q4 - ll4_active_start)
    } else {
        return Err(CoordinateError::FenceOrPadding);
    };
    let (_, height) = plane.logical_dimensions();
    if y_q4 >= height * 16 {
        return Err(CoordinateError::OutsideActivePlane);
    }
    Ok((plane, logical_x))
}

/// Convert an enriched composite-coordinate group into the standalone
/// candidate format, rejecting mixed scale planes atomically.
pub fn candidate_from_composite(
    mut centroids: [WeightedCentroid; 9],
    frame_sequence: u8,
) -> Result<Candidate, CoordinateError> {
    let mut plane = None;
    for centroid in &mut centroids {
        let (point_plane, logical_x) = classify_composite_q4(centroid.x_q4, centroid.y_q4)?;
        match plane {
            None => plane = Some(point_plane),
            Some(expected) if expected != point_plane => return Err(CoordinateError::MixedPlane),
            Some(_) => {}
        }
        centroid.x_q4 = logical_x;
    }
    Ok(Candidate {
        centroids,
        plane: plane.unwrap(),
        frame_sequence,
    })
}

/// Create a deterministic, quantized affine LFTag candidate.
pub fn ideal_q4_candidate(
    id: u16,
    plane: Plane,
    center: Point2,
    grid_pitch: f64,
    frame_sequence: u8,
) -> Candidate {
    let transform = Mat3::new([
        [grid_pitch, 0.0, center.x],
        [0.0, grid_pitch, center.y],
        [0.0, 0.0, 1.0],
    ]);
    let canonical = canonical_points(id);
    let centroids = core::array::from_fn(|index| {
        let point = transform
            .transform_point(canonical[index].position)
            .unwrap();
        let x_q4 = (point.x * 16.0).round() as i32;
        let y_q4 = (point.y * 16.0).round() as i32;
        let mass = match index {
            0 => 0x50,
            1 => 0x51,
            _ => 0x40 + (index as u8 - 2),
        };
        WeightedCentroid::new(x_q4, y_q4, mass)
    });
    Candidate {
        centroids,
        plane,
        frame_sequence,
    }
}

/// Canonical raw-plane RTL smoke vector: center `(320, 240)`, 48 pixels per
/// grid pitch, and masses `0x40..0x46, 0x50, 0x51`.
pub fn rtl_smoke_candidate(id: u16) -> Candidate {
    ideal_q4_candidate(id, Plane::Raw, Point2::new(320.0, 240.0), 48.0, 0)
}

/// Stable fixed-facing summary used by transaction-level differential tests.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FixedTransactionResult {
    pub accepted: bool,
    pub id: u16,
    pub hypothesis: u8,
    pub reject_reason: Option<RejectReason>,
    pub best_score_q32: u64,
    /// Pooled squared error in tag-grid-pitch units, floor(sum(error²)/sum(W²) * 2^32).
    /// Ranking and the compact header retain their homogeneous scores.
    pub normalized_mean_residual_q32: u32,
    pub plane: Plane,
    pub frame_sequence: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct FixedDecode {
    id: u16,
    hypothesis: u8,
    best_score_q32: u64,
    residual_sum: u64,
    scale_squared_sum: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct HypothesisScore {
    id: u16,
    public_score: u64,
    residual_sum: u64,
    scale_squared_sum: u64,
}

/// Compare the exact rational mean with a compact unsigned Q16 limit.
/// Comparing products retains the division remainder at equality boundaries.
fn normalized_residual_passes(numerator: u64, denominator: u64, limit_q16: u16) -> bool {
    denominator != 0
        && (u128::from(numerator) << 16) <= u128::from(limit_q16) * u128::from(denominator)
}

fn normalized_mean_q32(numerator: u64, denominator: u64) -> u32 {
    if denominator == 0 {
        return u32::MAX;
    }
    ((u128::from(numerator) << 32) / u128::from(denominator)).min(u128::from(u32::MAX)) as u32
}

fn signed_div_round(numerator: i128, denominator: i128) -> i64 {
    if denominator == 0 {
        return 0;
    }
    let negative = numerator.is_negative() ^ denominator.is_negative();
    let numerator = numerator.unsigned_abs();
    let denominator = denominator.unsigned_abs();
    let quotient = (numerator + (denominator >> 1)) / denominator;
    let magnitude = quotient as u64 as i64;
    if negative {
        magnitude.wrapping_neg()
    } else {
        magnitude
    }
}

fn canonical_hypothesis_matrix(hypothesis: u8) -> [i64; 9] {
    let left_x = if hypothesis & 0b0010 != 0 { 1 } else { -1 };
    let left_y = if hypothesis & 0b0001 != 0 { 1 } else { -1 };
    let right_x = if hypothesis & 0b1000 != 0 { 1 } else { -1 };
    let right_y = if hypothesis & 0b0100 != 0 { 1 } else { -1 };
    let dx1 = -right_x;
    let dx2 = -24 + left_x - right_x;
    let dx3 = right_x - left_x;
    let dy1 = -24 - right_y;
    let dy2 = left_y - right_y;
    let dy3 = right_y - left_y;
    let denominator = dx1 * dy2 - dx2 * dy1;
    let g = dx3 * dy2 - dx2 * dy3;
    let h = dx1 * dy3 - dx3 * dy1;
    [
        24 * denominator + g * 12,
        left_x * denominator + h * (-12 + left_x),
        -12 * denominator,
        -12 * g,
        (24 + left_y) * denominator + h * (12 + left_y),
        -12 * denominator,
        12 * g,
        12 * h,
        12 * denominator,
    ]
}

#[derive(Clone, Copy, Debug)]
struct HomogeneousPoint {
    u: i64,
    v: i64,
    w: i64,
}

fn qmul15(left: i64, right: i64) -> Option<i64> {
    let value = (i128::from(left) * i128::from(right)) >> 15;
    (-(1_i128 << 17)..(1_i128 << 17))
        .contains(&value)
        .then_some(value as i64)
}

fn signed_q8_coefficient(value: i64, scale: i64) -> i64 {
    signed_div_round(i128::from(value) << 8, i128::from(scale))
}

fn signed_wrap(value: i64, width: u32) -> i64 {
    let mask = (1_i64 << width) - 1;
    let wrapped = value & mask;
    if wrapped & (1_i64 << (width - 1)) != 0 {
        wrapped | !mask
    } else {
        wrapped
    }
}

fn pose_map_coefficients_fit(raw_t: i64, raw_r: i64, raw_s: i64) -> bool {
    let mut denominator = raw_t - raw_r - raw_s;
    let mut g = raw_s - raw_t;
    let mut h = raw_r - raw_t;
    let signed18 = -(1_i64 << 17)..(1_i64 << 17);
    let half = !signed18.contains(&denominator)
        || (denominator < 0
            && [denominator, g, h]
                .into_iter()
                .any(|value| value == -(1_i64 << 17)));
    if half {
        denominator >>= 1;
        g >>= 1;
        h >>= 1;
    }
    if denominator == 0 {
        return false;
    }
    if denominator < 0 {
        denominator = -denominator;
        g = -g;
        h = -h;
    }
    while denominator < (1 << 15) {
        let signed17 = -(1_i64 << 16)..(1_i64 << 16);
        if !signed17.contains(&g) || !signed17.contains(&h) {
            return false;
        }
        denominator <<= 1;
        g <<= 1;
        h <<= 1;
    }
    true
}

/// Candidate-wide Q15 homogeneous geometry used by the divide-free RTL.
/// Every point retains the exact weighted-centroid coordinate chosen by the
/// upstream corner selector; only the projective representation changes.
fn homogeneous_points_q15(
    corners: [(i64, i64); 4],
    points: [(i64, i64); 5],
    plane: Plane,
) -> Option<[HomogeneousPoint; 5]> {
    let [origin, top_right, bottom_right, bottom_left] = corners;
    let plane_shift = match plane {
        Plane::Raw => 8,
        Plane::Ll2 => 4,
        Plane::Ll4 => 0,
    };
    let cross = |left: (i64, i64), right: (i64, i64)| {
        ((left.0 - origin.0) * (right.1 - origin.1) - (left.1 - origin.1) * (right.0 - origin.0))
            >> plane_shift
    };
    let raw_t = cross(top_right, bottom_left);
    let raw_r = cross(bottom_right, bottom_left);
    let raw_s = cross(top_right, bottom_right);
    if raw_t <= 0 || raw_r <= 0 || raw_s <= 0 {
        return None;
    }
    // Pose reconstruction consumes this same raw-scale projective map.  Match
    // its signed-18 block normalization so the fixed transaction cannot
    // accept geometry that the direct-pose path has already rejected.
    if !pose_map_coefficients_fit(raw_t, raw_r, raw_s) {
        return None;
    }
    if [raw_t, raw_r, raw_s]
        .into_iter()
        .any(|value| !(-(1_i64 << 17)..(1_i64 << 17)).contains(&value))
    {
        return None;
    }

    let mut t = raw_t;
    let mut r = raw_r;
    let mut s = raw_s;
    let mut cross_shift = 0_i32;
    while t.max(r).max(s) >= (1 << 15) {
        t >>= 1;
        r >>= 1;
        s >>= 1;
        cross_shift -= 1;
    }
    while t.max(r).max(s) < (1 << 14) {
        t <<= 1;
        r <<= 1;
        s <<= 1;
        cross_shift += 1;
    }

    let d = t - r - s;
    let rs = qmul15(r, s)?;
    let k0 = qmul15(rs, t)?;
    let ka = qmul15(s, s - t)?;
    let kb = qmul15(r, r - t)?;
    let ku = qmul15(-d, s)?;
    let kv = qmul15(-d, r)?;
    if k0 <= 0 {
        return None;
    }

    let mut result = [HomogeneousPoint { u: 0, v: 0, w: 0 }; 5];
    for (slot, point) in result.iter_mut().zip(points) {
        let mut a = cross(point, bottom_left);
        let mut b = cross(top_right, point);
        if cross_shift >= 0 {
            for _ in 0..cross_shift {
                if !(-(1_i64 << 16)..(1_i64 << 16)).contains(&a)
                    || !(-(1_i64 << 16)..(1_i64 << 16)).contains(&b)
                {
                    return None;
                }
                a <<= 1;
                b <<= 1;
            }
        } else {
            a >>= -cross_shift;
            b >>= -cross_shift;
        }
        let u = qmul15(ku, a)?;
        let v = qmul15(kv, b)?;
        let w = k0 + qmul15(ka, a)? + qmul15(kb, b)?;
        if !(-(1_i64 << 17)..(1_i64 << 17)).contains(&w) {
            return None;
        }
        *slot = HomogeneousPoint { u, v, w };
    }
    Some(result)
}

fn evaluate_homogeneous_hypothesis(
    hypothesis: u8,
    points: &[HomogeneousPoint; 5],
) -> Option<HypothesisScore> {
    let matrix = canonical_hypothesis_matrix(hypothesis);
    let scale = -matrix[8];
    let coefficient = |index| -signed_q8_coefficient(matrix[index], scale);
    let [c0, c1, c3, c4, c6, c7] = [0, 1, 3, 4, 6, 7].map(coefficient);
    let mut occupancy = 0_u8;
    let mut id = (u16::from(hypothesis & 0b0011) << 8) | (u16::from(hypothesis >> 2) << 12);
    let mut residual_sum = 0_u64;
    let mut scale_squared_sum = 0_u64;
    for &point in points {
        let x = signed_wrap(((c0 * point.u + c1 * point.v) >> 8) - point.w, 24);
        let y = signed_wrap(((c3 * point.u + c4 * point.v) >> 8) - point.w, 24);
        let w = signed_wrap(((c6 * point.u + c7 * point.v) >> 8) + point.w, 24);
        if w <= 0 || 2 * x < -3 * w || 2 * x > 3 * w || 2 * y < -3 * w || 2 * y > 3 * w {
            return None;
        }
        let center_x = if 2 * x < -w {
            -w
        } else if 2 * x <= w {
            0
        } else {
            w
        };
        let center_y = if 2 * y < -w {
            -w
        } else if 2 * y <= w {
            0
        } else {
            w
        };
        let region_x = center_x.signum();
        let region_y = center_y.signum();
        let cell = match (region_x, region_y) {
            (0, -1) => 0,
            (-1, 0) => 1,
            (0, 0) => 2,
            (1, 0) => 3,
            (-1, 1) => 4,
            (0, 1) => 5,
            (1, 1) => 6,
            _ => return None,
        };
        if matches!(cell, 4 | 6) || occupancy & (1 << cell) != 0 {
            return None;
        }
        let symbol = (u8::from(x >= center_x) << 1) | u8::from(y >= center_y);
        let offset = signed_wrap(w * 21 + 128, 23) >> 8;
        let expected_x = center_x + if symbol & 2 != 0 { offset } else { -offset };
        let expected_y = center_y + if symbol & 1 != 0 { offset } else { -offset };
        let error_x = x - expected_x;
        let error_y = y - expected_y;
        if !(-(1_i64 << 17)..(1_i64 << 17)).contains(&error_x)
            || !(-(1_i64 << 17)..(1_i64 << 17)).contains(&error_y)
        {
            return None;
        }
        let square = (error_x * error_x + error_y * error_y) as u64;
        residual_sum += square;
        scale_squared_sum += (w * w) as u64;
        occupancy |= 1 << cell;
        id |= u16::from(symbol) << (2 * cell);
    }
    (occupancy == 0x2f).then_some(HypothesisScore {
        id,
        public_score: residual_sum.min((1_u64 << 34) - 1),
        residual_sum,
        scale_squared_sum,
    })
}

fn fixed_decode(decoder: &Decoder, candidate: &Candidate) -> Result<FixedDecode, RejectReason> {
    let config = decoder.config;
    if candidate.centroids.iter().any(|point| !point.valid) {
        return Err(RejectReason::InvalidCentroid);
    }
    if candidate.centroids.iter().any(|point| {
        point.x_q4 < 0 || point.x_q4 >= (1 << 15) || point.y_q4 < 0 || point.y_q4 >= (1 << 14)
    }) {
        return Err(RejectReason::NonFiniteCoordinate);
    }
    let minimum_mass = candidate
        .centroids
        .iter()
        .map(|point| point.log_mass_q5_3)
        .min()
        .unwrap();
    let maximum_mass = candidate
        .centroids
        .iter()
        .map(|point| point.log_mass_q5_3)
        .max()
        .unwrap();
    if maximum_mass - minimum_mass > config.max_log_mass_spread {
        return Err(RejectReason::MassSpread);
    }

    let points = candidate
        .centroids
        .map(|point| (i64::from(point.x_q4), i64::from(point.y_q4)));
    let mut stream_indices = core::array::from_fn::<_, 9, _>(|index| index);
    stream_indices.sort_by_key(|&index| (candidate.centroids[index].log_mass_q5_3, index));
    let stream_points = stream_indices.map(|index| candidate.centroids[index]);
    let minimum_baseline_q4 = (config.min_baseline_length * 16.0)
        .round()
        .clamp(0.0, u16::MAX as f64) as u16;
    let selection = select_sorted_streaming_corners(stream_points, minimum_baseline_q4);
    match selection.status {
        CornerSelectStatus::Valid => {}
        CornerSelectStatus::InvalidInput => return Err(RejectReason::NonFiniteCoordinate),
        CornerSelectStatus::BaselineTooShort => return Err(RejectReason::BaselineTooShort),
        CornerSelectStatus::DominantSide => return Err(RejectReason::DominantSide),
        CornerSelectStatus::CornerSelection => return Err(RejectReason::CornerSelection),
        CornerSelectStatus::InvalidQuadrilateral => {
            return Err(RejectReason::InvalidQuadrilateral);
        }
    }
    let key_indices = selection
        .corners
        .map(|stream_index| stream_indices[usize::from(stream_index)]);
    let remaining = selection
        .remaining
        .map(|stream_index| stream_indices[usize::from(stream_index)]);
    let homogeneous = homogeneous_points_q15(
        key_indices.map(|index| points[index]),
        remaining.map(|index| points[index]),
        candidate.plane,
    )
    .ok_or(RejectReason::SingularProjectiveMap)?;

    let mut best: Option<FixedDecode> = None;
    for hypothesis in 0..16u8 {
        let Some(HypothesisScore {
            id,
            public_score,
            residual_sum,
            scale_squared_sum,
        }) = evaluate_homogeneous_hypothesis(hypothesis, &homogeneous)
        else {
            continue;
        };
        match best {
            None => {
                best = Some(FixedDecode {
                    id,
                    hypothesis,
                    best_score_q32: public_score,
                    residual_sum,
                    scale_squared_sum,
                });
            }
            Some(current) if public_score < current.best_score_q32 => {
                best = Some(FixedDecode {
                    id,
                    hypothesis,
                    best_score_q32: public_score,
                    residual_sum,
                    scale_squared_sum,
                });
            }
            Some(_) => {}
        }
    }
    let best = best.ok_or(RejectReason::NoValidHypothesis)?;
    if !normalized_residual_passes(
        best.residual_sum,
        best.scale_squared_sum,
        config.mean_residual_limit_q16,
    ) {
        return Err(RejectReason::Residual);
    }
    Ok(best)
}

impl FixedTransactionResult {
    /// Serialize the compact result header independently of Rust enum layout.
    ///
    /// Word 0: valid, reserved-zero bit 62, plane, frame, hypothesis, ID. Word 1 is the
    /// Q32 best residual. Word 2 is reserved zero. Word 3 is the
    /// stable reject code (`0` for an accepted result).
    pub fn serialize_header(self) -> [u64; 4] {
        let plane = match self.plane {
            Plane::Raw => 0u64,
            Plane::Ll2 => 1,
            Plane::Ll4 => 2,
        };
        let word0 = (u64::from(self.accepted) << 63)
            | (plane << 60)
            | ((self.frame_sequence as u64) << 52)
            | ((self.hypothesis as u64) << 48)
            | self.id as u64;
        [
            word0,
            self.best_score_q32,
            0,
            self.reject_reason.map(reject_code).unwrap_or(0) as u64,
        ]
    }
}

/// Replay the integer homogeneous geometry, divide-free hypothesis
/// classification, normalized score accumulation, and tie-breaking used by
/// the RTL.
pub fn evaluate_transaction(decoder: &Decoder, candidate: &Candidate) -> FixedTransactionResult {
    match fixed_decode(decoder, candidate) {
        Ok(result) => FixedTransactionResult {
            accepted: true,
            id: result.id,
            hypothesis: result.hypothesis,
            reject_reason: None,
            best_score_q32: result.best_score_q32,
            normalized_mean_residual_q32: normalized_mean_q32(
                result.residual_sum,
                result.scale_squared_sum,
            ),
            plane: candidate.plane,
            frame_sequence: candidate.frame_sequence,
        },
        Err(reason) => FixedTransactionResult {
            accepted: false,
            id: 0,
            hypothesis: 0,
            reject_reason: Some(reason),
            best_score_q32: 0,
            normalized_mean_residual_q32: 0,
            plane: candidate.plane,
            frame_sequence: candidate.frame_sequence,
        },
    }
}

pub const fn reject_code(reason: RejectReason) -> u8 {
    match reason {
        RejectReason::InvalidCentroid => 1,
        RejectReason::NonFiniteCoordinate => 2,
        RejectReason::MassSpread => 4,
        RejectReason::BaselineTooShort => 5,
        RejectReason::DominantSide => 6,
        RejectReason::CornerSelection => 7,
        RejectReason::InvalidQuadrilateral => 8,
        RejectReason::Collinear => 9,
        RejectReason::SingularProjectiveMap => 10,
        RejectReason::NoValidHypothesis => 11,
        RejectReason::Residual => 12,
    }
}

/// Rejection classes which can appear in an RTL result packet.
///
/// Decode failures map directly to [`RejectReason`]. `Protocol` and
/// `Watchdog` are transport/engine failures, so they cannot be produced by
/// [`Decoder::decode`] from a well-formed [`Candidate`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TransportRejectReason {
    Decode(RejectReason),
    Protocol,
    Watchdog,
}

impl TransportRejectReason {
    /// Stable five-bit value carried by result-header word 6.
    pub const fn code(self) -> u8 {
        match self {
            Self::Decode(reason) => reject_code(reason),
            Self::Protocol => 14,
            Self::Watchdog => 15,
        }
    }
}

/// Parse the stable five-bit rejection field carried by RTL result packets.
///
/// Code zero denotes an accepted transaction. Unassigned values are returned
/// unchanged as an error.
pub const fn parse_transport_reject_code(code: u8) -> Result<Option<TransportRejectReason>, u8> {
    let reason = match code {
        0 => return Ok(None),
        1 => TransportRejectReason::Decode(RejectReason::InvalidCentroid),
        2 => TransportRejectReason::Decode(RejectReason::NonFiniteCoordinate),
        4 => TransportRejectReason::Decode(RejectReason::MassSpread),
        5 => TransportRejectReason::Decode(RejectReason::BaselineTooShort),
        6 => TransportRejectReason::Decode(RejectReason::DominantSide),
        7 => TransportRejectReason::Decode(RejectReason::CornerSelection),
        8 => TransportRejectReason::Decode(RejectReason::InvalidQuadrilateral),
        9 => TransportRejectReason::Decode(RejectReason::Collinear),
        10 => TransportRejectReason::Decode(RejectReason::SingularProjectiveMap),
        11 => TransportRejectReason::Decode(RejectReason::NoValidHypothesis),
        12 => TransportRejectReason::Decode(RejectReason::Residual),
        14 => TransportRejectReason::Protocol,
        15 => TransportRejectReason::Watchdog,
        _ => return Err(code),
    };
    Ok(Some(reason))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalized_error_is_scale_invariant_and_preserves_exact_boundaries() {
        for scale in [1_u64, 4, 256, 65536] {
            assert_eq!(normalized_mean_q32(3 * scale, 256 * scale), 3 << 24);
            assert!(normalized_residual_passes(3 * scale, 256 * scale, 3 << 8));
            assert!(!normalized_residual_passes(
                3 * scale,
                256 * scale,
                (3 << 8) - 1
            ));
        }
        // A nonzero remainder must not pass an equality to the floored quotient.
        let quotient_q16 = (65536 / 3) as u16;
        assert!(!normalized_residual_passes(1, 3, quotient_q16));
        assert!(normalized_residual_passes(1, 3, quotient_q16 + 1));
        assert!(!normalized_residual_passes(0, 0, u16::MAX));
        assert!(normalized_residual_passes(0, 1, 0));
        assert!(!normalized_residual_passes(1, u64::MAX, 0));
        // Full unsaturated sums and the complete configured Q16 range fit u128.
        assert!(normalized_residual_passes(
            (1 << 38) - 1,
            (1 << 39) - 1,
            u16::MAX
        ));
        assert!(!normalized_residual_passes(u64::MAX, u64::MAX, u16::MAX));
        // The default gate simplifies exactly to 64*N <= D.
        let default_limit = Decoder::default().config.mean_residual_limit_q16;
        assert_eq!(default_limit, 1024);
        assert!(normalized_residual_passes(1, 64, default_limit));
        assert!(!normalized_residual_passes(1, 63, default_limit));
        assert!(normalized_residual_passes(1, 65, default_limit));
    }

    #[test]
    fn fixed_and_floating_residual_configuration_are_independent() {
        let candidate = rtl_smoke_candidate(1234);
        let mut decoder = Decoder::default();
        let fixed = evaluate_transaction(&decoder, &candidate);
        let floating = decoder.decode(&candidate);
        assert!(fixed.accepted);
        assert!(floating.is_ok());

        decoder.config.mean_residual_limit_q16 = 0;
        assert_eq!(decoder.decode(&candidate), floating);
        decoder.config.mean_residual_limit_q16 = 1024;
        decoder.config.residual_limit = 0.0;
        assert_eq!(evaluate_transaction(&decoder, &candidate), fixed);
    }

    #[test]
    fn tied_scores_accept_deterministically_with_reserved_header() {
        // Symmetric data at the cell centers gives competing hypotheses with
        // equal fixed scores and floating scores separated only by roundoff.
        // The two larger top corners still define valid anchors.
        let positions = [
            (-1, -1),
            (1, -1),
            (0, -1),
            (-1, 0),
            (0, 0),
            (1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
        ];
        let candidate = Candidate {
            plane: Plane::Raw,
            frame_sequence: 19,
            centroids: core::array::from_fn(|i| {
                WeightedCentroid::new(
                    (320 + 16 * positions[i].0) * 16,
                    (240 + 16 * positions[i].1) * 16,
                    if i < 2 { 112 } else { 104 },
                )
            }),
        };
        let decoder = Decoder::default();
        let fixed = evaluate_transaction(&decoder, &candidate);
        assert!(fixed.accepted, "{fixed:?}");
        assert_eq!(
            (fixed.id, fixed.hypothesis, fixed.best_score_q32,),
            (3, 0, 395_136)
        );
        assert_eq!(fixed.serialize_header()[2], 0);
        assert_eq!(evaluate_transaction(&decoder, &candidate), fixed);

        // This previously failed the floating decoder's absolute 1e-8 margin.
        // Floating roundoff can choose a different winner than fixed math,
        // but both retain deterministic ranking without a score-gap gate.
        let floating = decoder.decode(&candidate).unwrap();
        assert_eq!((floating.id, floating.hypothesis), (10923, 10));
        assert_eq!(decoder.decode(&candidate).unwrap(), floating);
    }

    #[test]
    fn rtl_signed_division_rounds_nearest_with_ties_away_from_zero() {
        assert_eq!(signed_div_round(7, 2), 4);
        assert_eq!(signed_div_round(-7, 2), -4);
        assert_eq!(signed_div_round(6, 2), 3);
        assert_eq!(signed_div_round(-6, 2), -3);
        assert_eq!(signed_div_round(1, 4), 0);
        assert_eq!(signed_div_round(-1, 4), 0);
        assert_eq!(signed_div_round(1, 0), 0);
    }

    #[test]
    fn classifies_active_regions_and_rejects_fences() {
        assert_eq!(classify_composite_q4(17, 20), Ok((Plane::Raw, 17)));
        assert_eq!(
            classify_composite_q4(1284 * 16 + 17, 20),
            Ok((Plane::Ll2, 17))
        );
        assert_eq!(
            classify_composite_q4(1612 * 16 + 17, 20),
            Ok((Plane::Ll4, 17))
        );
        assert_eq!(
            classify_composite_q4(1282 * 16, 20),
            Err(CoordinateError::FenceOrPadding)
        );
    }

    #[test]
    fn smoke_vectors_decode_at_q4() {
        for id in [0, 0x1234, 0x2aaa, 0x1555, 0x3fff] {
            let candidate = rtl_smoke_candidate(id);
            let result = evaluate_transaction(&Decoder::default(), &candidate);
            assert!(result.accepted, "ID {id:#06x}: {result:?}");
            assert_eq!(result.id, id);
            assert_eq!(result.serialize_header()[0] >> 63, 1);
        }
    }

    #[test]
    fn retained_nine_validate_normally() {
        let decoder = Decoder::default();
        let mut candidate = rtl_smoke_candidate(0x1234);
        let result = evaluate_transaction(&decoder, &candidate);
        assert!(result.accepted);
        assert_eq!(result.id, 0x1234);
        assert_eq!(result.serialize_header()[0] & (1 << 62), 0);
        candidate.centroids[0].log_mass_q5_3 = 200;
        let invalid = evaluate_transaction(&decoder, &candidate);
        assert!(!invalid.accepted);
        assert_eq!(invalid.reject_reason, Some(RejectReason::MassSpread));
        assert_eq!(invalid.serialize_header()[0] & (1 << 62), 0);
    }

    #[test]
    fn fixed_scores_match_stabilized_rtl_checkpoints() {
        for (id, hypothesis, best_score_q32) in
            [(0x0000, 0, 264), (0x1234, 6, 595), (0x3fff, 15, 146)]
        {
            let result = evaluate_transaction(&Decoder::default(), &rtl_smoke_candidate(id));
            assert!(result.accepted);
            assert_eq!(
                (result.id, result.hypothesis, result.best_score_q32,),
                (id, hypothesis, best_score_q32)
            );
            assert_eq!(result.serialize_header()[2], 0);
            assert_eq!((result.plane, result.frame_sequence), (Plane::Raw, 0));
        }
    }

    #[test]
    fn transport_reject_codes_round_trip_and_cover_engine_failures() {
        assert_eq!(parse_transport_reject_code(0), Ok(None));
        for reason in [
            RejectReason::InvalidCentroid,
            RejectReason::NonFiniteCoordinate,
            RejectReason::MassSpread,
            RejectReason::BaselineTooShort,
            RejectReason::DominantSide,
            RejectReason::CornerSelection,
            RejectReason::InvalidQuadrilateral,
            RejectReason::Collinear,
            RejectReason::SingularProjectiveMap,
            RejectReason::NoValidHypothesis,
            RejectReason::Residual,
        ] {
            let transport = TransportRejectReason::Decode(reason);
            assert_eq!(
                parse_transport_reject_code(transport.code()),
                Ok(Some(transport)),
            );
        }
        assert_eq!(
            parse_transport_reject_code(14),
            Ok(Some(TransportRejectReason::Protocol)),
        );
        assert_eq!(
            parse_transport_reject_code(15),
            Ok(Some(TransportRejectReason::Watchdog)),
        );
        assert_eq!(parse_transport_reject_code(16), Err(16));
        assert_eq!(parse_transport_reject_code(u8::MAX), Err(u8::MAX));
    }
}
