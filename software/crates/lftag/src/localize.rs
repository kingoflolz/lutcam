//! Calibrated initial-homography localization and planar-pose decomposition.

use crate::decode::{DecodeResult, Plane};
use crate::geometry::{Mat3, Point2};

/// Pinhole calibration in source-image pixel-center coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraCalibration {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
}

impl CameraCalibration {
    pub const fn new(fx: f64, fy: f64, cx: f64, cy: f64) -> Self {
        Self { fx, fy, cx, cy }
    }

    pub fn is_valid(self) -> bool {
        self.fx.is_finite()
            && self.fy.is_finite()
            && self.cx.is_finite()
            && self.cy.is_finite()
            && self.fx > 0.0
            && self.fy > 0.0
    }

    pub fn normalize(self, source_pixel: Point2) -> Option<Point2> {
        self.is_valid().then(|| {
            Point2::new(
                (source_pixel.x - self.cx) / self.fx,
                (source_pixel.y - self.cy) / self.fy,
            )
        })
    }
}

/// Object-to-camera pose. Translation is in the same physical unit as the
/// printed tag width passed to [`localize`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    /// Row-major rotation matrix.
    pub rotation: [[f64; 3]; 3],
    pub translation: [f64; 3],
}

/// Hardware-oriented localization result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Localization {
    /// Homography from normalized LFTag coordinates to normalized camera
    /// coordinates.
    pub homography: Mat3,
    /// Mean canonical decode residual over the five non-key points.
    pub mean_decode_residual: f64,
    pub pose: Pose,
    pub weak_perspective: bool,
    pub conditioning_warning: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LocalizationError {
    InvalidCalibration,
    InvalidTagWidth,
    InsufficientCorrespondences,
    SingularNormalMatrix,
    WeakProjectiveDenominator,
    InvalidPose,
}

/// Convert a box-average plane coordinate to source-image pixel centers.
/// For the production CDF frontend, use [`wavelet_logical_to_source`].
pub fn logical_to_source(point: Point2, scale: i32) -> Point2 {
    let scale = scale as f64;
    Point2::new((point.x + 0.5) * scale - 0.5, (point.y + 0.5) * scale - 0.5)
}

/// Convert a production CDF frontend coordinate to source-image pixel centers.
/// CDF lifting retains the even sample at each level. The hybrid LL4 tap
/// averages adjacent H4/V3 rows, adding four source pixels only on Y.
pub fn wavelet_logical_to_source(point: Point2, plane: Plane) -> Point2 {
    let scale = f64::from(plane.scale());
    let phase_y = if plane == Plane::Ll4 { 4.0 } else { 0.0 };
    Point2::new(point.x * scale, point.y * scale + phase_y)
}

/// Fit a projective transform with `h33 = 1` by all-point linear least
/// squares. `pairs` contain `(world, image)` coordinates.
pub fn fit_homography(pairs: &[(Point2, Point2)]) -> Result<Mat3, LocalizationError> {
    if pairs.len() < 4 {
        return Err(LocalizationError::InsufficientCorrespondences);
    }
    let mut normal = [[0.0; 8]; 8];
    let mut rhs = [0.0; 8];
    for &(world, image) in pairs {
        if !world.is_finite() || !image.is_finite() {
            return Err(LocalizationError::SingularNormalMatrix);
        }
        let x_row = [
            world.x,
            world.y,
            1.0,
            0.0,
            0.0,
            0.0,
            -image.x * world.x,
            -image.x * world.y,
        ];
        let y_row = [
            0.0,
            0.0,
            0.0,
            world.x,
            world.y,
            1.0,
            -image.y * world.x,
            -image.y * world.y,
        ];
        accumulate_normal(&mut normal, &mut rhs, x_row, image.x);
        accumulate_normal(&mut normal, &mut rhs, y_row, image.y);
    }
    let solution = solve_linear(normal, rhs)?;
    let homography = Mat3::new([
        [solution[0], solution[1], solution[2]],
        [solution[3], solution[4], solution[5]],
        [solution[6], solution[7], 1.0],
    ]);
    if !homography.is_finite() || homography.determinant().abs() < 1.0e-12 {
        return Err(LocalizationError::SingularNormalMatrix);
    }
    Ok(homography)
}

fn accumulate_normal(normal: &mut [[f64; 8]; 8], rhs: &mut [f64; 8], row: [f64; 8], target: f64) {
    for i in 0..8 {
        rhs[i] = row[i].mul_add(target, rhs[i]);
        for j in i..8 {
            normal[i][j] = row[i].mul_add(row[j], normal[i][j]);
        }
    }
    let upper = *normal;
    for (row_index, values) in normal.iter_mut().enumerate() {
        for (column_index, value) in values.iter_mut().enumerate().take(row_index) {
            *value = upper[column_index][row_index];
        }
    }
}

fn solve_linear(
    mut matrix: [[f64; 8]; 8],
    mut rhs: [f64; 8],
) -> Result<[f64; 8], LocalizationError> {
    let scale = matrix
        .iter()
        .flatten()
        .map(|value| value.abs())
        .fold(0.0, f64::max);
    if !scale.is_finite() || scale == 0.0 {
        return Err(LocalizationError::SingularNormalMatrix);
    }
    let pivot_guard = scale * 1.0e-12;
    for column in 0..8 {
        let pivot = (column..8)
            .max_by(|&left, &right| {
                matrix[left][column]
                    .abs()
                    .total_cmp(&matrix[right][column].abs())
            })
            .unwrap();
        if matrix[pivot][column].abs() <= pivot_guard {
            return Err(LocalizationError::SingularNormalMatrix);
        }
        if pivot != column {
            matrix.swap(pivot, column);
            rhs.swap(pivot, column);
        }
        let divisor = matrix[column][column];
        let pivot_row = matrix[column];
        let pivot_rhs = rhs[column];
        for (row, rhs_value) in matrix.iter_mut().zip(rhs.iter_mut()).skip(column + 1) {
            let factor = row[column] / divisor;
            row[column] = 0.0;
            for (inner, value) in row.iter_mut().enumerate().skip(column + 1) {
                *value -= factor * pivot_row[inner];
            }
            *rhs_value -= factor * pivot_rhs;
        }
    }
    let mut result = [0.0; 8];
    for row in (0..8).rev() {
        let tail: f64 = ((row + 1)..8)
            .map(|column| matrix[row][column] * result[column])
            .sum();
        result[row] = (rhs[row] - tail) / matrix[row][row];
    }
    result
        .iter()
        .all(|value| value.is_finite())
        .then_some(result)
        .ok_or(LocalizationError::SingularNormalMatrix)
}

/// Calibrate a box-average decoder's four-keypoint initial homography
/// and decompose it into an object-to-camera pose. Use [`localize_wavelet`]
/// for the production CDF frontend.
///
/// Localization deliberately does not refit the homography from all nine
/// correspondences. The five non-key points have already been consumed by the
/// decoder, and their mean residual is exposed as the localization quality
/// metric without another image-space projection pass.
pub fn localize(
    decoded: &DecodeResult,
    calibration: CameraCalibration,
    printed_tag_width: f64,
) -> Result<Localization, LocalizationError> {
    let scale = f64::from(decoded.plane.scale());
    let phase = (scale - 1.0) * 0.5;
    let logical_to_source = Mat3::new([[scale, 0.0, phase], [0.0, scale, phase], [0.0, 0.0, 1.0]]);
    localize_with_source_transform(decoded, calibration, printed_tag_width, logical_to_source)
}

/// Localize the production CDF frontend's RAW, LL2 or hybrid LL4 detections.
/// Uses the same four-keypoint pose model as [`localize`] with the actual
/// wavelet sampling phase, matching the FPGA localization datapath.
pub fn localize_wavelet(
    decoded: &DecodeResult,
    calibration: CameraCalibration,
    printed_tag_width: f64,
) -> Result<Localization, LocalizationError> {
    let scale = f64::from(decoded.plane.scale());
    let origin = wavelet_logical_to_source(Point2::new(0.0, 0.0), decoded.plane);
    let logical_to_source = Mat3::new([
        [scale, 0.0, origin.x],
        [0.0, scale, origin.y],
        [0.0, 0.0, 1.0],
    ]);
    localize_with_source_transform(decoded, calibration, printed_tag_width, logical_to_source)
}

fn localize_with_source_transform(
    decoded: &DecodeResult,
    calibration: CameraCalibration,
    printed_tag_width: f64,
    logical_to_source: Mat3,
) -> Result<Localization, LocalizationError> {
    if !calibration.is_valid() {
        return Err(LocalizationError::InvalidCalibration);
    }
    if !printed_tag_width.is_finite() || printed_tag_width <= 0.0 {
        return Err(LocalizationError::InvalidTagWidth);
    }
    let camera_inverse = Mat3::new([
        [1.0 / calibration.fx, 0.0, -calibration.cx / calibration.fx],
        [0.0, 1.0 / calibration.fy, -calibration.cy / calibration.fy],
        [0.0, 0.0, 1.0],
    ]);
    let homography = (camera_inverse * logical_to_source * decoded.initial_homography)
        .normalized()
        .ok_or(LocalizationError::SingularNormalMatrix)?;
    if !homography.is_finite() || homography.determinant().abs() < 1.0e-12 {
        return Err(LocalizationError::SingularNormalMatrix);
    }
    let mean_decode_residual = decoded.best_score / 5.0;
    let (pose, weak_perspective, conditioning_warning) =
        decompose_pose(homography, printed_tag_width)?;
    Ok(Localization {
        homography,
        mean_decode_residual,
        pose,
        weak_perspective,
        conditioning_warning,
    })
}

fn decompose_pose(
    homography: Mat3,
    printed_tag_width: f64,
) -> Result<(Pose, bool, bool), LocalizationError> {
    let mut b1 = [homography.m[0][0], homography.m[1][0], homography.m[2][0]];
    let mut b2 = [homography.m[0][1], homography.m[1][1], homography.m[2][1]];
    let mut b3 = [homography.m[0][2], homography.m[1][2], homography.m[2][2]];
    if b3[2] < 0.0 {
        b1 = vec_scale(b1, -1.0);
        b2 = vec_scale(b2, -1.0);
        b3 = vec_scale(b3, -1.0);
    }
    let norm1 = vec_norm(b1);
    let norm2 = vec_norm(b2);
    if !norm1.is_finite() || !norm2.is_finite() || norm1 <= 1.0e-12 || norm2 <= 1.0e-12 {
        return Err(LocalizationError::InvalidPose);
    }
    let mean_square = 0.5 * (norm1 * norm1 + norm2 * norm2);
    let rms_norm = mean_square.sqrt();
    if !rms_norm.is_finite() || rms_norm <= 1.0e-12 {
        return Err(LocalizationError::InvalidPose);
    }
    // Match the compact hardware pose model: one RMS scale serves both
    // direction columns, and their cross product forms the third. A physical
    // planar-tag homography already has equal, orthogonal direction columns;
    // avoiding separate normalization and Gram-Schmidt saves two square roots
    // and several divider transactions.
    let lambda = 1.0 / rms_norm;
    let r1 = vec_scale(b1, lambda);
    let r2 = vec_scale(b2, lambda);
    let r3 = vec_cross(r1, r2);
    let physical_scale = printed_tag_width / 4.0;
    let translation = vec_scale(b3, lambda * physical_scale);
    if translation[2] <= 0.0 || !translation.iter().all(|value| value.is_finite()) {
        return Err(LocalizationError::InvalidPose);
    }
    let rotation = [
        [r1[0], r2[0], r3[0]],
        [r1[1], r2[1], r3[1]],
        [r1[2], r2[2], r3[2]],
    ];
    let conditioning_warning = 8.0 * norm1.min(norm2) < 7.0 * norm1.max(norm2);
    let perspective = b1[2].hypot(b2[2]) / norm1.max(norm2);
    let weak_perspective = perspective < 1.0e-3;
    Ok((
        Pose {
            rotation,
            translation,
        },
        weak_perspective,
        conditioning_warning,
    ))
}

fn vec_dot(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0].mul_add(right[0], left[1].mul_add(right[1], left[2] * right[2]))
}

fn vec_norm(value: [f64; 3]) -> f64 {
    vec_dot(value, value).sqrt()
}

fn vec_scale(value: [f64; 3], scale: f64) -> [f64; 3] {
    [value[0] * scale, value[1] * scale, value[2] * scale]
}

fn vec_cross(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_point_fit_recovers_projective_map() {
        let expected = Mat3::new([[0.31, 0.02, 0.04], [-0.01, 0.29, -0.03], [0.03, -0.02, 1.0]]);
        let world = [
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(-1.0, 1.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 0.0),
            Point2::new(0.2, -0.7),
            Point2::new(-0.8, 0.1),
            Point2::new(0.6, 0.4),
            Point2::new(0.0, 0.9),
        ];
        let pairs: Vec<_> = world
            .into_iter()
            .map(|point| (point, expected.transform_point(point).unwrap()))
            .collect();
        let actual = fit_homography(&pairs).unwrap();
        for (left, right) in actual.m.iter().flatten().zip(expected.m.iter().flatten()) {
            assert!((left - right).abs() < 1.0e-10, "{left} != {right}");
        }
    }

    #[test]
    fn logical_source_conversion_preserves_pixel_centers() {
        assert_eq!(
            logical_to_source(Point2::new(0.0, 0.0), 4),
            Point2::new(1.5, 1.5)
        );
        assert_eq!(
            logical_to_source(Point2::new(10.0, 7.0), 1),
            Point2::new(10.0, 7.0)
        );
    }
}
