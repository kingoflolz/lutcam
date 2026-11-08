//! Measured LFTag image features for offline camera calibration.
//!
//! These points come from image centroids, never from a pose or a fitted
//! homography. Coordinates retain the native, unflipped sensor orientation.

use std::collections::BTreeMap;

use lftag::fixed::{CoordinateError, candidate_from_composite, classify_composite_q4};
use lftag::localize::wavelet_logical_to_source;
use lftag::{
    DecodeConfig, DecodeResult, Decoder, Plane, Point2, RejectReason, WeightedCentroid,
    WeightedPoint,
};
use lrcca::{
    AdaptiveThresholdConfig, CentroidGroupDescriptor, CentroidGroupOutput,
    WaveletTagDetectReferenceModel,
};
use serde_json::{Value, json};

pub const IMAGE_WIDTH: usize = 1280;
pub const IMAGE_HEIGHT: usize = 800;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FeatureSource {
    ImageCentroids,
    FpgaCentroids,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FeatureDetection {
    pub id: u16,
    pub plane: Plane,
    pub source: FeatureSource,
    /// The nine observed source-image pixel centers, in canonical point order.
    pub points: [[f64; 2]; 9],
    /// Tag-plane coordinates in grid-pitch units; multiply by printed width / 4.
    pub canonical_points: [[f64; 2]; 9],
    /// Canonical decoder residuals, not image reprojection errors.
    pub best_score: f64,
}

impl FeatureDetection {
    pub fn to_json(&self) -> Value {
        let plane = match self.plane {
            Plane::Raw => 0,
            Plane::Ll2 => 1,
            Plane::Ll4 => 2,
        };
        let masses_available = self.source == FeatureSource::ImageCentroids;
        json!({
            "tag_id": self.id,
            "plane": plane,
            "scale": self.plane.scale(),
            "source_geometry": "cdf_even_ll4_y4_v1",
            "source": match self.source {
                FeatureSource::ImageCentroids => "image_centroids",
                FeatureSource::FpgaCentroids => "fpga_centroids",
            },
            "masses_available": masses_available,
            "mass_spread_checked": masses_available,
            "baseline_mass_ties_observable": masses_available,
            "points": self.points,
            "canonical_points": self.canonical_points,
            "score": self.best_score,
            "best_score": self.best_score,
        })
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct FeatureExtraction {
    /// Includes separate instances and multiscale observations of the same ID.
    /// A calibration consumer must resolve those associations explicitly.
    pub detections: Vec<FeatureDetection>,
    pub rejections: BTreeMap<String, usize>,
    /// Completed groups plus any final unterminated group.
    pub candidate_groups: usize,
}

impl FeatureExtraction {
    pub fn to_json(&self) -> Value {
        json!({
            "detections": self.detections.iter().map(FeatureDetection::to_json)
                .collect::<Vec<_>>(),
            "rejections": self.rejections,
            "candidate_groups": self.candidate_groups,
        })
    }

    fn record(&mut self, result: Result<FeatureDetection, FeatureRejection>) {
        self.candidate_groups += 1;
        match result {
            Ok(detection) => self.detections.push(detection),
            Err(reason) => *self.rejections.entry(reason.key()).or_default() += 1,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FeatureRejection {
    GroupSize(usize),
    UnterminatedGroup,
    InvalidCentroid,
    InvalidWireTuple,
    CoordinateRange,
    Coordinate(CoordinateError),
    Decode(RejectReason),
    NonFiniteScore,
}

impl FeatureRejection {
    fn key(self) -> String {
        match self {
            Self::GroupSize(_) => "group_size".into(),
            Self::UnterminatedGroup => "unterminated_group".into(),
            Self::InvalidCentroid => "invalid_centroid".into(),
            Self::InvalidWireTuple => "invalid_wire_tuple".into(),
            Self::CoordinateRange => "coordinate_range".into(),
            Self::Coordinate(error) => format!("coordinate_{error:?}"),
            Self::Decode(error) => format!("decode_{error:?}"),
            Self::NonFiniteScore => "nonfinite_score".into(),
        }
    }
}

/// Decode one measured, enriched composite-raster group without calibration.
pub fn decode_group(
    descriptors: &[CentroidGroupDescriptor],
) -> Result<FeatureDetection, FeatureRejection> {
    if descriptors.len() != 9 {
        return Err(FeatureRejection::GroupSize(descriptors.len()));
    }
    let mut centroids = [WeightedCentroid::default(); 9];
    for (centroid, descriptor) in centroids.iter_mut().zip(descriptors) {
        if !descriptor.centroid_valid {
            return Err(FeatureRejection::InvalidCentroid);
        }
        *centroid = WeightedCentroid::new(
            i32::try_from(descriptor.x_q4).map_err(|_| FeatureRejection::CoordinateRange)?,
            i32::try_from(descriptor.y_q4).map_err(|_| FeatureRejection::CoordinateRange)?,
            descriptor.log_m_q5_3,
        );
    }
    // This existing adapter checks all active plane bounds and removes the
    // 1284/1612-pixel coarse X origins. Descriptor Y is already logical Y.
    let candidate = candidate_from_composite(centroids, 0).map_err(FeatureRejection::Coordinate)?;
    let decoded = Decoder::default()
        .decode(&candidate)
        .map_err(FeatureRejection::Decode)?;
    measured_detection(decoded, FeatureSource::ImageCentroids)
}

/// Decode the measured FPGA child tuples in their transmitted ascending mass
/// order. Each zero-extended 33-bit tuple carries valid, X Q5, and Y Q5.
///
/// The wire omits masses. Rank keys recover the two final baseline
/// entries, but cannot check mass spread or ties at the baseline cutoff. This
/// is host geometric decoding, not a claim of FPGA decoder acceptance. All
/// observed coordinates retain their full Q5 precision.
pub fn decode_fpga_group(tuples: [u64; 9]) -> Result<FeatureDetection, FeatureRejection> {
    let mut points = [WeightedPoint::default(); 9];
    let mut plane = None;
    for (rank, (point, tuple)) in points.iter_mut().zip(tuples).enumerate() {
        if tuple >> 33 != 0 {
            return Err(FeatureRejection::InvalidWireTuple);
        }
        if tuple & 1 == 0 {
            return Err(FeatureRejection::InvalidCentroid);
        }
        let x_q5 = ((tuple >> 1) & 0xffff) as i32;
        let y_q5 = ((tuple >> 17) & 0xffff) as i32;
        // Active bounds are integer pixels: classifying at Q4 is exact for
        // bounds, while the measured point below retains the remaining Q5 bit.
        let (point_plane, _) =
            classify_composite_q4(x_q5 >> 1, y_q5 >> 1).map_err(FeatureRejection::Coordinate)?;
        if plane.is_some_and(|expected| expected != point_plane) {
            return Err(FeatureRejection::Coordinate(CoordinateError::MixedPlane));
        }
        plane = Some(point_plane);
        let origin = match point_plane {
            Plane::Raw => 0.0,
            Plane::Ll2 => 1284.0,
            Plane::Ll4 => 1612.0,
        };
        *point = WeightedPoint::new(
            Point2::new(x_q5 as f64 / 32.0 - origin, y_q5 as f64 / 32.0),
            rank as u8,
        );
    }
    let decoded = Decoder::new(DecodeConfig {
        max_log_mass_spread: u8::MAX,
        ..DecodeConfig::default()
    })
    .decode_weighted(points, plane.unwrap(), 0)
    .map_err(FeatureRejection::Decode)?;
    measured_detection(decoded, FeatureSource::FpgaCentroids)
}

fn measured_detection(
    decoded: DecodeResult,
    source: FeatureSource,
) -> Result<FeatureDetection, FeatureRejection> {
    if !decoded.best_score.is_finite() {
        return Err(FeatureRejection::NonFiniteScore);
    }
    Ok(FeatureDetection {
        id: decoded.id,
        plane: decoded.plane,
        source,
        points: decoded.correspondences.map(|point| {
            let measured = wavelet_logical_to_source(point.observed, decoded.plane);
            [measured.x, measured.y]
        }),
        canonical_points: decoded
            .correspondences
            .map(|point| [point.canonical.x, point.canonical.y]),
        best_score: decoded.best_score,
    })
}

/// Consume group boundaries, retaining at most nine descriptors per group.
pub fn extract_groups(events: &[CentroidGroupOutput]) -> FeatureExtraction {
    let mut result = FeatureExtraction::default();
    let mut group = Vec::with_capacity(9);
    let mut count = 0;
    for event in events {
        if let Some(descriptor) = event.descriptor {
            count += 1;
            if group.len() < 9 {
                group.push(descriptor);
            }
        }
        if event.group_end {
            result.record(if count == 9 {
                decode_group(&group)
            } else {
                Err(FeatureRejection::GroupSize(count))
            });
            group.clear();
            count = 0;
        }
    }
    if count != 0 {
        result.record(Err(FeatureRejection::UnterminatedGroup));
    }
    result
}

/// Extract features from one row-major, 8-bit native-orientation sensor image.
/// This full-image reference is intended for offline selected-frame processing.
pub fn extract_features(
    pixels: &[u8],
    width: usize,
    height: usize,
) -> Result<FeatureExtraction, String> {
    if (width, height) != (IMAGE_WIDTH, IMAGE_HEIGHT) {
        return Err("calibration extraction requires a 1280x800 source image".into());
    }
    if pixels.len() != IMAGE_WIDTH * IMAGE_HEIGHT {
        return Err("calibration image storage does not match 1280x800 dimensions".into());
    }
    let model =
        WaveletTagDetectReferenceModel::new(width, height, AdaptiveThresholdConfig::default())?;
    Ok(extract_groups(&model.centroid_group_output_stream(pixels)?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use lftag::canonical_points;

    const ID: u16 = 0x2a55;

    fn fixture(plane: Plane, center: Point2, pitch: f64) -> [CentroidGroupDescriptor; 9] {
        let origin = match plane {
            Plane::Raw => 0,
            Plane::Ll2 => 1284 * 16,
            Plane::Ll4 => 1612 * 16,
        };
        canonical_points(ID).map(|point| CentroidGroupDescriptor {
            centroid_valid: true,
            x_q4: ((center.x + pitch * point.position.x) * 16.0).round() as u32 + origin,
            y_q4: ((center.y + pitch * point.position.y) * 16.0).round() as u32,
            log_m_q5_3: if point.symbol.is_none() { 80 } else { 64 },
        })
    }

    fn raw_fixture() -> [CentroidGroupDescriptor; 9] {
        fixture(Plane::Raw, Point2::new(640.0, 400.0), 96.0)
    }

    fn events(group: &[CentroidGroupDescriptor]) -> Vec<CentroidGroupOutput> {
        group
            .iter()
            .enumerate()
            .map(|(index, descriptor)| CentroidGroupOutput {
                descriptor: Some(*descriptor),
                group_end: index + 1 == group.len(),
            })
            .collect()
    }

    fn fpga_tuples(group: [CentroidGroupDescriptor; 9]) -> [u64; 9] {
        let mut sorted = group;
        sorted.sort_by_key(|descriptor| descriptor.log_m_q5_3);
        sorted.map(|descriptor| {
            ((u64::from(descriptor.y_q4) * 2) << 17)
                | ((u64::from(descriptor.x_q4) * 2) << 1)
                | u64::from(descriptor.centroid_valid)
        })
    }

    #[test]
    fn fpga_rank_only_decode_preserves_q5_measurements_and_labels_missing_mass() {
        let original = raw_fixture();
        let mut tuples = fpga_tuples(original);
        // Canonical point4 is sorted slot2. Add one Q5 step that a Q4 adapter
        // would truncate, and retain it in the canonical observed output.
        tuples[2] += 1 << 1;
        let decoded = decode_fpga_group(tuples).unwrap();
        assert_eq!(decoded.id, ID);
        assert_eq!(decoded.source, FeatureSource::FpgaCentroids);
        for (index, point) in decoded.points.iter().enumerate() {
            assert_eq!(
                *point,
                [
                    original[index].x_q4 as f64 / 16.0 + if index == 4 { 1.0 / 32.0 } else { 0.0 },
                    original[index].y_q4 as f64 / 16.0
                ]
            );
        }
        let json = decoded.to_json();
        assert_eq!(json["source"], "fpga_centroids");
        assert_eq!(json["source_geometry"], "cdf_even_ll4_y4_v1");
        for key in [
            "masses_available",
            "mass_spread_checked",
            "baseline_mass_ties_observable",
        ] {
            assert_eq!(json[key], false);
        }
    }

    #[test]
    fn fpga_coarse_groups_project_observations_and_keep_geometric_rejections() {
        for (plane, center, pitch) in [
            (Plane::Ll2, Point2::new(160.0, 100.0), 24.0),
            (Plane::Ll4, Point2::new(40.0, 25.0), 10.0),
        ] {
            let group = fixture(plane, center, pitch);
            let expected = decode_group(&group).unwrap();
            let actual = decode_fpga_group(fpga_tuples(group)).unwrap();
            assert_eq!(
                (actual.id, actual.plane, actual.points),
                (expected.id, expected.plane, expected.points)
            );
        }
        let tuple = ((400_u64 * 32) << 17) | ((640_u64 * 32) << 1) | 1;
        assert_eq!(
            decode_fpga_group([tuple; 9]),
            Err(FeatureRejection::Decode(RejectReason::BaselineTooShort))
        );
    }

    #[test]
    fn fpga_wire_flags_and_plane_validation_are_strict() {
        let good = fpga_tuples(raw_fixture());
        let mut bad = good;
        bad[0] |= 1 << 33;
        assert_eq!(
            decode_fpga_group(bad),
            Err(FeatureRejection::InvalidWireTuple)
        );
        bad = good;
        bad[0] &= !1;
        assert_eq!(
            decode_fpga_group(bad),
            Err(FeatureRejection::InvalidCentroid)
        );
        bad = good;
        bad[0] = ((1280_u64 * 32) << 1) | 1;
        assert_eq!(
            decode_fpga_group(bad),
            Err(FeatureRejection::Coordinate(
                CoordinateError::FenceOrPadding
            ))
        );
        bad = good;
        bad[0] = ((1284_u64 * 32) << 1) | 1;
        assert_eq!(
            decode_fpga_group(bad),
            Err(FeatureRejection::Coordinate(CoordinateError::MixedPlane))
        );
    }

    #[test]
    fn shuffled_noisy_group_preserves_every_measured_point_and_correspondence() {
        let mut original = raw_fixture();
        // A non-key point must retain its measured displacement, even though
        // the four-key homography would predict its original ideal location.
        original[4].x_q4 += 3;
        original[4].y_q4 -= 2;
        let shuffled = [7, 1, 4, 8, 2, 0, 6, 3, 5].map(|index| original[index]);
        let decoded = decode_group(&shuffled).unwrap();
        assert_eq!(decoded.id, ID);
        assert_eq!(decoded.plane, Plane::Raw);
        assert!(decoded.best_score > 0.0);
        for (index, canonical) in canonical_points(ID).iter().enumerate() {
            assert_eq!(
                decoded.points[index],
                [
                    original[index].x_q4 as f64 / 16.0,
                    original[index].y_q4 as f64 / 16.0
                ]
            );
            assert_eq!(
                decoded.canonical_points[index],
                [canonical.position.x, canonical.position.y]
            );
        }
    }

    #[test]
    fn coarse_planes_remove_x_origin_and_use_actual_cdf_sample_phase() {
        for (plane, center, pitch, origin, scale, phase_y) in [
            (
                Plane::Ll2,
                Point2::new(160.0, 100.0),
                24.0,
                1284.0,
                4.0,
                0.0,
            ),
            (Plane::Ll4, Point2::new(40.0, 25.0), 10.0, 1612.0, 16.0, 4.0),
        ] {
            let group = fixture(plane, center, pitch);
            let decoded = decode_group(&group).unwrap();
            assert_eq!((decoded.id, decoded.plane), (ID, plane));
            for (observed, descriptor) in decoded.points.iter().zip(group) {
                assert_eq!(
                    *observed,
                    [
                        (descriptor.x_q4 as f64 / 16.0 - origin) * scale,
                        descriptor.y_q4 as f64 / 16.0 * scale + phase_y,
                    ]
                );
            }
        }
    }

    #[test]
    fn malformed_groups_are_rejected_before_decode() {
        let good = raw_fixture();
        assert_eq!(
            decode_group(&good[..8]),
            Err(FeatureRejection::GroupSize(8))
        );
        assert_eq!(decode_group(&[]), Err(FeatureRejection::GroupSize(0)));
        let mut bad = good;
        bad[3].centroid_valid = false;
        assert_eq!(decode_group(&bad), Err(FeatureRejection::InvalidCentroid));
        bad = good;
        bad[3].x_q4 = u32::MAX;
        assert_eq!(decode_group(&bad), Err(FeatureRejection::CoordinateRange));
        bad = good;
        bad[3].x_q4 = 1284 * 16;
        bad[3].y_q4 = 0;
        assert_eq!(
            decode_group(&bad),
            Err(FeatureRejection::Coordinate(CoordinateError::MixedPlane))
        );
    }

    #[test]
    fn retained_nine_preserve_mass_validation_without_list_metadata() {
        let mut group = raw_fixture();
        let good = decode_group(&group).unwrap();
        assert_eq!(good.id, ID);
        let json = good.to_json();
        assert!(json.get("overflow").is_none());
        assert!(json.get("overflow_known").is_none());
        assert_eq!(json["mass_spread_checked"], true);
        group[0].log_m_q5_3 = 200;
        assert_eq!(
            decode_group(&group),
            Err(FeatureRejection::Decode(RejectReason::MassSpread))
        );
    }

    #[test]
    fn fence_padding_and_exclusive_y_bounds_never_become_observations() {
        for x_q4 in [
            1280 * 16,
            1284 * 16 - 1,
            1604 * 16,
            1612 * 16 - 1,
            1692 * 16,
        ] {
            let mut group = raw_fixture();
            group[0].x_q4 = x_q4;
            group[0].y_q4 = 0;
            assert_eq!(
                decode_group(&group),
                Err(FeatureRejection::Coordinate(
                    CoordinateError::FenceOrPadding
                ))
            );
        }
        for (plane, center, pitch, height) in [
            (Plane::Raw, Point2::new(640.0, 400.0), 96.0, 800),
            (Plane::Ll2, Point2::new(160.0, 100.0), 24.0, 200),
            (Plane::Ll4, Point2::new(40.0, 25.0), 10.0, 50),
        ] {
            let mut group = fixture(plane, center, pitch);
            group[0].y_q4 = height * 16;
            assert_eq!(
                decode_group(&group),
                Err(FeatureRejection::Coordinate(
                    CoordinateError::OutsideActivePlane
                ))
            );
        }
    }

    #[test]
    fn rejected_and_unterminated_groups_do_not_contaminate_following_groups() {
        let good = raw_fixture();
        let mut stream = events(&good);
        stream.extend(events(&good[..8]));
        stream.push(CentroidGroupOutput::default());
        stream.extend(events(&good));
        stream.push(CentroidGroupOutput {
            descriptor: Some(good[0]),
            ..Default::default()
        });
        let extracted = extract_groups(&stream);
        assert_eq!(extracted.candidate_groups, 4);
        assert_eq!(extracted.detections.len(), 2);
        assert_eq!(
            extracted.rejections,
            BTreeMap::from([("group_size".into(), 1), ("unterminated_group".into(), 1),])
        );
        let mut too_long = good.to_vec();
        too_long.push(good[0]);
        let mut stream = events(&too_long);
        stream.extend(events(&good));
        let extracted = extract_groups(&stream);
        assert_eq!(extracted.detections.len(), 1);
        assert_eq!(extracted.rejections["group_size"], 1);
    }

    #[test]
    fn same_id_at_different_positions_is_not_silently_deduplicated() {
        let mut stream = events(&raw_fixture());
        stream.extend(events(&fixture(
            Plane::Raw,
            Point2::new(300.0, 200.0),
            48.0,
        )));
        let extracted = extract_groups(&stream);
        assert_eq!(extracted.detections.len(), 2);
        assert!(
            extracted
                .detections
                .iter()
                .all(|detection| detection.id == ID)
        );
        assert_ne!(
            extracted.detections[0].points,
            extracted.detections[1].points
        );
    }

    #[test]
    fn json_has_solver_keys() {
        let detection = decode_group(&raw_fixture()).unwrap();
        let value = detection.to_json();
        assert_eq!(value["tag_id"], ID);
        assert_eq!(value["plane"], 0);
        assert_eq!(value["scale"], 1);
        assert_eq!(value["score"], detection.best_score);
        assert_eq!(value["points"].as_array().unwrap().len(), 9);
        assert!(value.get("id").is_none());
    }

    #[test]
    fn invalid_dimensions_and_storage_are_errors() {
        assert!(extract_features(&[], 0, 0).is_err());
        assert!(extract_features(&[], IMAGE_WIDTH, IMAGE_HEIGHT).is_err());
        assert!(extract_features(&vec![0; IMAGE_WIDTH * IMAGE_HEIGHT], 800, 1280).is_err());
        assert_eq!(extract_groups(&[]), FeatureExtraction::default());
    }

    #[test]
    fn constant_image_does_not_invent_an_lftag() {
        let features = extract_features(
            &vec![128; IMAGE_WIDTH * IMAGE_HEIGHT],
            IMAGE_WIDTH,
            IMAGE_HEIGHT,
        )
        .unwrap();
        assert!(features.detections.is_empty());
    }
}
