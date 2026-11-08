//! Sparse LFTag3x3 decoding from nine weighted centroids.

use core::cmp::Ordering;

use crate::geometry::{DATA_CELL_CENTERS, Mat3, Point2, square_to_quad, symbol_offset};

/// TagDetect plane in which centroid coordinates are expressed.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum Plane {
    #[default]
    Raw,
    Ll2,
    Ll4,
}

impl Plane {
    pub const fn scale(self) -> i32 {
        match self {
            Self::Raw => 1,
            Self::Ll2 => 4,
            Self::Ll4 => 16,
        }
    }

    pub const fn logical_dimensions(self) -> (i32, i32) {
        match self {
            Self::Raw => (1280, 800),
            Self::Ll2 => (320, 200),
            Self::Ll4 => (80, 50),
        }
    }
}

/// One production-style centroid descriptor.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct WeightedCentroid {
    pub x_q4: i32,
    pub y_q4: i32,
    pub log_mass_q5_3: u8,
    pub valid: bool,
}

impl WeightedCentroid {
    pub const fn new(x_q4: i32, y_q4: i32, log_mass_q5_3: u8) -> Self {
        Self {
            x_q4,
            y_q4,
            log_mass_q5_3,
            valid: true,
        }
    }

    pub fn point(self) -> Point2 {
        Point2::new(self.x_q4 as f64 / 16.0, self.y_q4 as f64 / 16.0)
    }
}

/// One floating-point descriptor used by the readable algorithmic oracle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WeightedPoint {
    pub point: Point2,
    pub log_mass_q5_3: u8,
    pub valid: bool,
}

impl WeightedPoint {
    pub const fn new(point: Point2, log_mass_q5_3: u8) -> Self {
        Self {
            point,
            log_mass_q5_3,
            valid: true,
        }
    }
}

/// A complete atomic TagDetect candidate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Candidate {
    pub centroids: [WeightedCentroid; 9],
    pub plane: Plane,
    pub frame_sequence: u8,
}

impl Candidate {
    pub fn weighted_points(self) -> [WeightedPoint; 9] {
        self.centroids.map(|centroid| WeightedPoint {
            point: centroid.point(),
            log_mass_q5_3: centroid.log_mass_q5_3,
            valid: centroid.valid,
        })
    }
}

/// Tunable acceptance thresholds for the sparse decoder.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DecodeConfig {
    /// Maximum difference between weakest and strongest Q5.3 log masses.
    pub max_log_mass_spread: u8,
    /// Minimum baseline length in logical-plane pixels.
    pub min_baseline_length: f64,
    /// Minimum edge or diagonal length relative to the baseline length.
    pub min_quad_length_ratio: f64,
    /// Minimum twice-area relative to squared baseline length.
    pub min_quad_area_ratio: f64,
    /// Lower bound on covariance determinant divided by trace squared.
    pub min_covariance_ratio: f64,
    /// Maximum five-point canonical residual sum for the floating-point decoder.
    pub residual_limit: f64,
    /// Fixed-point hardware model's maximum pooled mean squared residual in
    /// tag-grid-pitch units, as an unsigned fraction of 65536. The default
    /// 1024 permits 1/64 squared error (1/8 grid pitch RMS). This does not
    /// change the floating-point decoder's `residual_limit`.
    pub mean_residual_limit_q16: u16,
    /// Homogeneous-denominator and comparison guard.
    pub epsilon: f64,
}

impl Default for DecodeConfig {
    fn default() -> Self {
        Self {
            max_log_mass_spread: 19,
            min_baseline_length: 1.0,
            min_quad_length_ratio: 0.02,
            min_quad_area_ratio: 0.02,
            min_covariance_ratio: 1.0e-4,
            residual_limit: 0.08,
            mean_residual_limit_q16: 1024,
            epsilon: 1.0e-10,
        }
    }
}

/// Hardware-counter-compatible rejection classes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RejectReason {
    InvalidCentroid,
    NonFiniteCoordinate,
    MassSpread,
    BaselineTooShort,
    DominantSide,
    CornerSelection,
    InvalidQuadrilateral,
    Collinear,
    SingularProjectiveMap,
    NoValidHypothesis,
    Residual,
}

/// One decoded canonical-to-observed point association.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Correspondence {
    pub canonical: Point2,
    pub observed: Point2,
    pub input_index: u8,
}

/// Successful sparse decode, before camera calibration and localization.
#[derive(Clone, Debug, PartialEq)]
pub struct DecodeResult {
    pub id: u16,
    pub best_score: f64,
    pub hypothesis: u8,
    /// Initial four-keypoint homography, canonical tag to logical-plane pixels.
    pub initial_homography: Mat3,
    /// Baselines followed by the seven data cells in ascending ID-field order.
    pub correspondences: [Correspondence; 9],
    pub plane: Plane,
    pub frame_sequence: u8,
}

/// Stateless decoder with explicit threshold configuration.
#[derive(Clone, Copy, Debug, Default)]
pub struct Decoder {
    pub config: DecodeConfig,
}

impl Decoder {
    pub const fn new(config: DecodeConfig) -> Self {
        Self { config }
    }

    pub fn decode(&self, candidate: &Candidate) -> Result<DecodeResult, RejectReason> {
        self.decode_weighted(
            candidate.centroids.map(|centroid| WeightedPoint {
                point: centroid.point(),
                log_mass_q5_3: centroid.log_mass_q5_3,
                valid: centroid.valid,
            }),
            candidate.plane,
            candidate.frame_sequence,
        )
    }

    pub fn decode_weighted(
        &self,
        points: [WeightedPoint; 9],
        plane: Plane,
        frame_sequence: u8,
    ) -> Result<DecodeResult, RejectReason> {
        let cfg = self.config;
        if points.iter().any(|point| !point.valid) {
            return Err(RejectReason::InvalidCentroid);
        }
        if points.iter().any(|point| !point.point.is_finite()) {
            return Err(RejectReason::NonFiniteCoordinate);
        }
        let minimum_mass = points
            .iter()
            .map(|point| point.log_mass_q5_3)
            .min()
            .unwrap();
        let maximum_mass = points
            .iter()
            .map(|point| point.log_mass_q5_3)
            .max()
            .unwrap();
        if maximum_mass - minimum_mass > cfg.max_log_mass_spread {
            return Err(RejectReason::MassSpread);
        }

        let mut ranked = core::array::from_fn::<_, 9, _>(|index| index);
        ranked.sort_by(|&left, &right| {
            points[right]
                .log_mass_q5_3
                .cmp(&points[left].log_mass_q5_3)
                .then_with(|| left.cmp(&right))
        });
        let mut left_baseline = ranked[0];
        let mut right_baseline = ranked[1];

        let mut baseline = points[right_baseline].point - points[left_baseline].point;
        let baseline_length_squared = baseline.norm_squared();
        if baseline_length_squared < cfg.min_baseline_length * cfg.min_baseline_length {
            return Err(RejectReason::BaselineTooShort);
        }

        let non_baselines: Vec<usize> = (0..9)
            .filter(|index| *index != left_baseline && *index != right_baseline)
            .collect();
        let (positive, negative) = side_counts(
            points[left_baseline].point,
            baseline,
            &non_baselines,
            &points,
            cfg.epsilon,
        );
        if positive < 6 && negative < 6 {
            return Err(RejectReason::DominantSide);
        }
        if negative > positive {
            core::mem::swap(&mut left_baseline, &mut right_baseline);
            baseline = points[right_baseline].point - points[left_baseline].point;
        }

        let interior: Vec<usize> = non_baselines
            .iter()
            .copied()
            .filter(|&index| {
                baseline.cross(points[index].point - points[left_baseline].point) > cfg.epsilon
            })
            .collect();
        if interior.len() < 6 {
            return Err(RejectReason::DominantSide);
        }

        let bottom_left = select_corner(
            points[left_baseline].point,
            baseline,
            1.0,
            &interior,
            &points,
            cfg.epsilon,
        )
        .ok_or(RejectReason::CornerSelection)?;
        let bottom_right = select_corner(
            points[right_baseline].point,
            baseline * -1.0,
            -1.0,
            &interior,
            &points,
            cfg.epsilon,
        )
        .ok_or(RejectReason::CornerSelection)?;
        if bottom_left == bottom_right {
            return Err(RejectReason::CornerSelection);
        }

        let key_indices = [left_baseline, right_baseline, bottom_right, bottom_left];
        let key_points = key_indices.map(|index| points[index].point);
        validate_quadrilateral(key_points, baseline_length_squared, cfg)?;
        validate_spread(&points, cfg)?;

        let observed_square =
            square_to_quad(key_points).ok_or(RejectReason::SingularProjectiveMap)?;
        let observed_to_square = observed_square
            .inverse()
            .ok_or(RejectReason::SingularProjectiveMap)?;

        let remaining: [usize; 5] = (0..9)
            .filter(|index| !key_indices.contains(index))
            .collect::<Vec<_>>()
            .try_into()
            .map_err(|_| RejectReason::CornerSelection)?;
        let mut rectified = [Point2::default(); 5];
        for (slot, &index) in rectified.iter_mut().zip(&remaining) {
            *slot = observed_to_square
                .transform_point(points[index].point)
                .ok_or(RejectReason::SingularProjectiveMap)?;
        }

        let mut best: Option<HypothesisResult> = None;
        for bottom_right_symbol in 0..4u8 {
            for bottom_left_symbol in 0..4u8 {
                let hypothesis = (bottom_right_symbol << 2) | bottom_left_symbol;
                let Some(result) = evaluate_hypothesis(
                    hypothesis,
                    bottom_left_symbol,
                    bottom_right_symbol,
                    rectified,
                    remaining,
                    cfg,
                ) else {
                    continue;
                };
                match &best {
                    None => best = Some(result),
                    Some(current)
                        if score_order(
                            result.score,
                            result.hypothesis,
                            current.score,
                            current.hypothesis,
                        ) == Ordering::Less =>
                    {
                        best = Some(result);
                    }
                    Some(_) => {}
                }
            }
        }
        let best = best.ok_or(RejectReason::NoValidHypothesis)?;
        if best.score > cfg.residual_limit {
            return Err(RejectReason::Residual);
        }

        let bottom_left_symbol = best.hypothesis & 0b11;
        let bottom_right_symbol = (best.hypothesis >> 2) & 0b11;
        let canonical_square = canonical_hypothesis(bottom_left_symbol, bottom_right_symbol)
            .ok_or(RejectReason::SingularProjectiveMap)?;
        let initial_homography = observed_square
            * canonical_square
                .inverse()
                .ok_or(RejectReason::SingularProjectiveMap)?;

        let mut id = ((bottom_left_symbol as u16) << 8) | ((bottom_right_symbol as u16) << 12);
        let mut correspondences = [Correspondence::default(); 9];
        correspondences[0] = Correspondence {
            canonical: Point2::new(-1.0, -1.0),
            observed: points[left_baseline].point,
            input_index: left_baseline as u8,
        };
        correspondences[1] = Correspondence {
            canonical: Point2::new(1.0, -1.0),
            observed: points[right_baseline].point,
            input_index: right_baseline as u8,
        };

        for data_index in 0..7 {
            let (symbol, input_index) = match data_index {
                4 => (bottom_left_symbol, bottom_left),
                6 => (bottom_right_symbol, bottom_right),
                _ => {
                    let assignment =
                        best.assignments[data_index].ok_or(RejectReason::NoValidHypothesis)?;
                    (assignment.symbol, assignment.input_index)
                }
            };
            id |= (symbol as u16) << (2 * data_index);
            correspondences[data_index + 2] = Correspondence {
                canonical: DATA_CELL_CENTERS[data_index] + symbol_offset(symbol),
                observed: points[input_index].point,
                input_index: input_index as u8,
            };
        }

        Ok(DecodeResult {
            id,
            best_score: best.score,
            hypothesis: best.hypothesis,
            initial_homography,
            correspondences,
            plane,
            frame_sequence,
        })
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Assignment {
    symbol: u8,
    input_index: usize,
}

#[derive(Clone, Debug)]
struct HypothesisResult {
    hypothesis: u8,
    score: f64,
    assignments: [Option<Assignment>; 7],
}

fn side_counts(
    origin: Point2,
    baseline: Point2,
    indices: &[usize],
    points: &[WeightedPoint; 9],
    epsilon: f64,
) -> (usize, usize) {
    indices.iter().fold((0, 0), |(positive, negative), &index| {
        let cross = baseline.cross(points[index].point - origin);
        (
            positive + usize::from(cross > epsilon),
            negative + usize::from(cross < -epsilon),
        )
    })
}

fn select_corner(
    origin: Point2,
    base: Point2,
    orientation: f64,
    indices: &[usize],
    points: &[WeightedPoint; 9],
    epsilon: f64,
) -> Option<usize> {
    let mut candidates: Vec<usize> = indices
        .iter()
        .copied()
        .filter(|&index| orientation * base.cross(points[index].point - origin) > epsilon)
        .collect();
    if candidates.len() < 2 {
        return None;
    }
    candidates.sort_by(|&left, &right| {
        let a = points[left].point - origin;
        let b = points[right].point - origin;
        let oriented_cross = orientation * a.cross(b);
        if oriented_cross > epsilon {
            Ordering::Less
        } else if oriented_cross < -epsilon {
            Ordering::Greater
        } else {
            // Lower input index wins an exact angular tie, so place it later.
            right.cmp(&left)
        }
    });
    let pair = &candidates[candidates.len() - 2..];
    pair.iter().copied().max_by(|&left, &right| {
        let left_distance = (points[left].point - origin).norm_squared();
        let right_distance = (points[right].point - origin).norm_squared();
        left_distance
            .total_cmp(&right_distance)
            .then_with(|| right.cmp(&left))
    })
}

fn validate_quadrilateral(
    corners: [Point2; 4],
    baseline_length_squared: f64,
    cfg: DecodeConfig,
) -> Result<(), RejectReason> {
    let minimum_length_squared =
        baseline_length_squared * cfg.min_quad_length_ratio * cfg.min_quad_length_ratio;
    let edges = [
        corners[1] - corners[0],
        corners[2] - corners[1],
        corners[3] - corners[2],
        corners[0] - corners[3],
    ];
    if edges
        .iter()
        .any(|edge| edge.norm_squared() < minimum_length_squared)
    {
        return Err(RejectReason::InvalidQuadrilateral);
    }
    let diagonal_a = corners[2] - corners[0];
    let diagonal_b = corners[3] - corners[1];
    if diagonal_a.norm_squared() < minimum_length_squared
        || diagonal_b.norm_squared() < minimum_length_squared
    {
        return Err(RejectReason::InvalidQuadrilateral);
    }

    let winding = edges.map(|_| 0.0);
    let mut winding = winding;
    for index in 0..4 {
        winding[index] = edges[index].cross(edges[(index + 1) % 4]);
    }
    if winding.iter().any(|value| *value <= cfg.epsilon) {
        return Err(RejectReason::InvalidQuadrilateral);
    }
    let twice_area = corners
        .iter()
        .enumerate()
        .map(|(index, point)| point.cross(corners[(index + 1) % 4]))
        .sum::<f64>()
        .abs();
    if twice_area < baseline_length_squared * cfg.min_quad_area_ratio {
        return Err(RejectReason::InvalidQuadrilateral);
    }
    Ok(())
}

fn validate_spread(points: &[WeightedPoint; 9], cfg: DecodeConfig) -> Result<(), RejectReason> {
    let sum_x: f64 = points.iter().map(|point| point.point.x).sum();
    let sum_y: f64 = points.iter().map(|point| point.point.y).sum();
    let sum_xx: f64 = points
        .iter()
        .map(|point| point.point.x * point.point.x)
        .sum();
    let sum_yy: f64 = points
        .iter()
        .map(|point| point.point.y * point.point.y)
        .sum();
    let sum_xy: f64 = points
        .iter()
        .map(|point| point.point.x * point.point.y)
        .sum();
    let n = 9.0;
    let cxx = n * sum_xx - sum_x * sum_x;
    let cyy = n * sum_yy - sum_y * sum_y;
    let cxy = n * sum_xy - sum_x * sum_y;
    let determinant = cxx.mul_add(cyy, -cxy * cxy);
    let trace = cxx + cyy;
    if !determinant.is_finite()
        || trace <= cfg.epsilon
        || determinant <= cfg.min_covariance_ratio * trace * trace
    {
        return Err(RejectReason::Collinear);
    }
    Ok(())
}

fn canonical_hypothesis(bottom_left_symbol: u8, bottom_right_symbol: u8) -> Option<Mat3> {
    square_to_quad([
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0) + symbol_offset(bottom_right_symbol),
        Point2::new(-1.0, 1.0) + symbol_offset(bottom_left_symbol),
    ])
}

fn evaluate_hypothesis(
    hypothesis: u8,
    bottom_left_symbol: u8,
    bottom_right_symbol: u8,
    rectified: [Point2; 5],
    remaining: [usize; 5],
    cfg: DecodeConfig,
) -> Option<HypothesisResult> {
    let canonical = canonical_hypothesis(bottom_left_symbol, bottom_right_symbol)?;
    let mut score = 0.0;
    let mut occupancy = 0u8;
    let mut assignments = [None; 7];
    for (&unit, &input_index) in rectified.iter().zip(&remaining) {
        let point = canonical.transform_point(unit)?;
        let (data_index, center) = classify_data_cell(point)?;
        if matches!(data_index, 4 | 6) {
            return None;
        }
        let bit = 1u8 << data_index;
        if occupancy & bit != 0 {
            return None;
        }
        occupancy |= bit;
        let symbol = (u8::from(point.x >= center.x) << 1) | u8::from(point.y >= center.y);
        let expected = center + symbol_offset(symbol);
        let error = point - expected;
        score += error.norm_squared();
        assignments[data_index] = Some(Assignment {
            symbol,
            input_index,
        });
    }
    const REQUIRED: u8 = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5);
    if occupancy != REQUIRED || !score.is_finite() || score > cfg.residual_limit.max(16.0) {
        return None;
    }
    Some(HypothesisResult {
        hypothesis,
        score,
        assignments,
    })
}

fn classify_data_cell(point: Point2) -> Option<(usize, Point2)> {
    const OUTER: f64 = 1.5;
    if !point.is_finite()
        || !(-OUTER..=OUTER).contains(&point.x)
        || !(-OUTER..=OUTER).contains(&point.y)
    {
        return None;
    }
    let column = if point.x < -0.5 {
        -1
    } else if point.x <= 0.5 {
        0
    } else {
        1
    };
    let row = if point.y < -0.5 {
        -1
    } else if point.y <= 0.5 {
        0
    } else {
        1
    };
    let data_index = match (column, row) {
        (0, -1) => 0,
        (-1, 0) => 1,
        (0, 0) => 2,
        (1, 0) => 3,
        (-1, 1) => 4,
        (0, 1) => 5,
        (1, 1) => 6,
        _ => return None,
    };
    Some((data_index, DATA_CELL_CENTERS[data_index]))
}

fn score_order(
    left_score: f64,
    left_hypothesis: u8,
    right_score: f64,
    right_hypothesis: u8,
) -> Ordering {
    left_score
        .total_cmp(&right_score)
        .then_with(|| left_hypothesis.cmp(&right_hypothesis))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::canonical_points;

    fn ideal_group(id: u16, transform: Mat3) -> [WeightedPoint; 9] {
        canonical_points(id).map(|canonical| {
            WeightedPoint::new(
                transform.transform_point(canonical.position).unwrap(),
                if canonical.symbol.is_none() { 112 } else { 104 },
            )
        })
    }

    #[test]
    fn decodes_every_id_without_noise() {
        let decoder = Decoder::default();
        let transform = Mat3::new([
            [48.0, 2.0, 220.0],
            [-1.0, 45.0, 130.0],
            [0.0007, -0.0004, 1.0],
        ]);
        for id in 0..(1 << 14) {
            let decoded = decoder
                .decode_weighted(ideal_group(id, transform), Plane::Raw, 7)
                .unwrap_or_else(|reason| panic!("ID {id} rejected: {reason:?}"));
            assert_eq!(decoded.id, id, "wrong decode for ID {id}");
            assert!(decoded.best_score < 1.0e-18);
        }
    }

    #[test]
    fn order_and_baseline_direction_do_not_matter() {
        let id = 0x2a55;
        let transform = Mat3::new([
            [-3.0, 55.0, 310.0],
            [-58.0, 1.0, 220.0],
            [0.0003, -0.0006, 1.0],
        ]);
        let original = ideal_group(id, transform);
        let permutation = [7, 1, 4, 8, 2, 0, 6, 3, 5];
        let shuffled = permutation.map(|index| original[index]);
        let decoded = Decoder::default()
            .decode_weighted(shuffled, Plane::Ll2, 42)
            .unwrap();
        assert_eq!(decoded.id, id);
        assert_eq!(decoded.plane, Plane::Ll2);
        assert_eq!(decoded.frame_sequence, 42);
    }

    #[test]
    fn rejects_duplicate_data_cell() {
        let mut group = ideal_group(1234, Mat3::IDENTITY.scale(40.0));
        group[4].point = group[3].point;
        assert!(
            Decoder::default()
                .decode_weighted(group, Plane::Raw, 0)
                .is_err()
        );
    }

    #[test]
    fn retained_nine_still_require_valid_centroids_and_mass_spread() {
        let group = ideal_group(
            5,
            Mat3::new([[40.0, 0.0, 100.0], [0.0, 40.0, 100.0], [0.0, 0.0, 1.0]]),
        );
        let decoder = Decoder::default();
        let decoded = decoder.decode_weighted(group, Plane::Raw, 37).unwrap();
        assert_eq!(decoded.id, 5);
        assert_eq!(decoded.frame_sequence, 37);
        let mut invalid = group;
        invalid[0].valid = false;
        assert_eq!(
            decoder.decode_weighted(invalid, Plane::Raw, 37),
            Err(RejectReason::InvalidCentroid)
        );
        let mut spread = group;
        spread[0].log_mass_q5_3 = 200;
        assert_eq!(
            decoder.decode_weighted(spread, Plane::Raw, 37),
            Err(RejectReason::MassSpread)
        );
    }
}
