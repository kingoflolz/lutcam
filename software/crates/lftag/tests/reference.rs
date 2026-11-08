use lftag::fixed::{evaluate_transaction, ideal_q4_candidate, rtl_smoke_candidate};
use lftag::{
    CameraCalibration, Candidate, Decoder, Mat3, Plane, Point2, WeightedCentroid, canonical_points,
    localize, localize_wavelet,
};

fn transformed_candidate(id: u16, transform: Mat3, permutation: [usize; 9]) -> Candidate {
    let canonical = canonical_points(id);
    let ordered: [WeightedCentroid; 9] = core::array::from_fn(|index| {
        let point = transform
            .transform_point(canonical[index].position)
            .unwrap();
        WeightedCentroid::new(
            (point.x * 16.0).round() as i32,
            (point.y * 16.0).round() as i32,
            if index < 2 {
                0x50 + index as u8
            } else {
                0x40 + index as u8 - 2
            },
        )
    });
    Candidate {
        centroids: permutation.map(|index| ordered[index]),
        plane: Plane::Raw,
        frame_sequence: 19,
    }
}

#[derive(Clone, Copy)]
struct Lcg(u64);

impl Lcg {
    fn next_u32(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 32) as u32
    }

    fn unit(&mut self) -> f64 {
        f64::from(self.next_u32()) / f64::from(u32::MAX)
    }

    fn symmetric(&mut self, magnitude: f64) -> f64 {
        (2.0 * self.unit() - 1.0) * magnitude
    }
}

fn randomized_candidate(rng: &mut Lcg, id: u16) -> Candidate {
    let angle = rng.symmetric(core::f64::consts::PI);
    let (sin, cos) = angle.sin_cos();
    let scale_x = 24.0 + 72.0 * rng.unit();
    let scale_y = scale_x * (0.65 + 0.7 * rng.unit());
    let shear = rng.symmetric(scale_x * 0.12);
    let center_x = 180.0 + 820.0 * rng.unit();
    let center_y = 140.0 + 500.0 * rng.unit();
    let projective_x = rng.symmetric(0.09);
    let projective_y = rng.symmetric(0.09);
    let affine_xx = scale_x * cos;
    let affine_xy = -scale_y * sin + shear;
    let affine_yx = scale_x * sin;
    let affine_yy = scale_y * cos;
    let transform = Mat3::new([
        [
            affine_xx + center_x * projective_x,
            affine_xy + center_x * projective_y,
            center_x,
        ],
        [
            affine_yx + center_y * projective_x,
            affine_yy + center_y * projective_y,
            center_y,
        ],
        [projective_x, projective_y, 1.0],
    ]);
    let canonical = canonical_points(id);
    let mut ordered: [WeightedCentroid; 9] = core::array::from_fn(|index| {
        let point = transform
            .transform_point(canonical[index].position)
            .unwrap();
        WeightedCentroid::new(
            ((point.x + rng.symmetric(0.12)) * 16.0).round() as i32,
            ((point.y + rng.symmetric(0.12)) * 16.0).round() as i32,
            if index < 2 {
                0x50 + index as u8
            } else {
                0x40 + index as u8 - 2
            },
        )
    });
    for upper in (1..ordered.len()).rev() {
        let selected = (rng.next_u32() as usize) % (upper + 1);
        ordered.swap(upper, selected);
    }
    Candidate {
        centroids: ordered,
        plane: Plane::Raw,
        frame_sequence: rng.next_u32() as u8,
    }
}

#[test]
fn representative_ids_survive_rotations_perspective_and_q4() {
    let decoder = Decoder::default();
    let transforms = [
        Mat3::new([
            [52.0, 3.0, 320.0],
            [-2.0, 48.0, 240.0],
            [0.0012, -0.0007, 1.0],
        ]),
        Mat3::new([
            [-4.0, 50.0, 320.0],
            [-51.0, 2.0, 240.0],
            [0.0008, 0.0010, 1.0],
        ]),
        Mat3::new([
            [-49.0, -4.0, 320.0],
            [3.0, -47.0, 240.0],
            [-0.0009, 0.0006, 1.0],
        ]),
        Mat3::new([
            [3.0, -50.0, 320.0],
            [49.0, -1.0, 240.0],
            [-0.0006, -0.0008, 1.0],
        ]),
    ];
    let permutations = [
        [0, 1, 2, 3, 4, 5, 6, 7, 8],
        [7, 1, 4, 8, 2, 0, 6, 3, 5],
        [8, 7, 6, 5, 4, 3, 2, 1, 0],
        [2, 5, 0, 7, 3, 8, 1, 4, 6],
    ];
    for id in [0, 1, 0x1234, 0x1555, 0x2aaa, 0x3fff] {
        for (&transform, &permutation) in transforms.iter().zip(&permutations) {
            let candidate = transformed_candidate(id, transform, permutation);
            let decoded = decoder
                .decode(&candidate)
                .unwrap_or_else(|reason| panic!("ID {id:#06x}: {reason:?}"));
            assert_eq!(decoded.id, id);
            assert!(decoded.best_score < 1.0e-4);
        }
    }
}

#[test]
fn randomized_projective_q4_noise_sweep_preserves_ids() {
    let decoder = Decoder::default();
    let mut rng = Lcg(0x6c66_7461_675f_6670);
    let case_count = std::env::var("LFTAG_RANDOMIZED_CASES")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(2_048u32);
    let mut fixed_accepted = 0u32;
    let mut fixed_id_mismatches = 0u32;
    let mut fixed_hypothesis_mismatches = 0u32;
    let mut fixed_residual_rejects = 0u32;
    let mut fixed_other_rejects = 0u32;
    let mut fixed_reject_codes = [0u32; 14];
    for case in 0..case_count {
        let id = (rng.next_u32() as u16) & 0x3fff;
        let candidate = randomized_candidate(&mut rng, id);
        let decoded = decoder
            .decode(&candidate)
            .unwrap_or_else(|reason| panic!("case {case}, ID {id:#06x} rejected: {reason:?}"));
        assert_eq!(
            decoded.id, id,
            "case {case}: {candidate:?}; decoded={decoded:?}"
        );
        let fixed = evaluate_transaction(&decoder, &candidate);
        if fixed.accepted {
            fixed_accepted += 1;
            fixed_id_mismatches += u32::from(fixed.id != decoded.id);
            fixed_hypothesis_mismatches += u32::from(fixed.hypothesis != decoded.hypothesis);
        } else {
            if let Some(reason) = fixed.reject_reason {
                fixed_reject_codes[usize::from(lftag::fixed::reject_code(reason))] += 1;
            }
            match fixed.reject_reason {
                Some(lftag::RejectReason::Residual) => fixed_residual_rejects += 1,
                _ => fixed_other_rejects += 1,
            }
        }
    }
    println!(
        "production homogeneous sweep: cases={case_count}, accepted={fixed_accepted}, \
         id_mismatches={fixed_id_mismatches}, hypothesis_mismatches={fixed_hypothesis_mismatches}, \
         residual_rejects={fixed_residual_rejects}, \
         other_rejects={fixed_other_rejects}"
    );
    println!("production homogeneous reject codes: {fixed_reject_codes:?}");
    assert_eq!(fixed_id_mismatches, 0);
    assert_eq!(fixed_hypothesis_mismatches, 0);
    if case_count == 2_048 {
        assert_eq!(
            fixed_accepted, 2_048,
            "the pinned production fixture must not lose accepted candidates"
        );
    }
}

fn horizontally_reflected_id(id: u16) -> u16 {
    let symbol = |cell: usize| ((id >> (2 * cell)) & 3) ^ 2;
    let reflected = [
        symbol(0),
        symbol(3),
        symbol(2),
        symbol(1),
        symbol(6),
        symbol(5),
        symbol(4),
    ];
    reflected
        .into_iter()
        .enumerate()
        .fold(0, |value, (cell, bits)| value | (bits << (2 * cell)))
}

#[test]
fn mirrored_camera_path_maps_to_the_documented_other_valid_id() {
    let transform = Mat3::new([[-48.0, 0.0, 320.0], [0.0, 48.0, 240.0], [0.0, 0.0, 1.0]]);
    for id in [0, 1, 0x1234, 0x2aaa, 0x3fff] {
        let candidate = transformed_candidate(id, transform, [6, 1, 8, 3, 0, 5, 2, 7, 4]);
        let decoded = Decoder::default().decode(&candidate).unwrap();
        assert_eq!(decoded.id, horizontally_reflected_id(id));
    }
}

#[test]
fn deterministic_rtl_vectors_have_stable_headers() {
    for id in [0, 0x1234, 0x2aaa, 0x1555, 0x3fff] {
        let result = evaluate_transaction(&Decoder::default(), &rtl_smoke_candidate(id));
        let header = result.serialize_header();
        assert!(result.accepted);
        assert_eq!(header[0] & 0x3fff, id as u64);
        assert_eq!(header[3], 0);
    }
}

#[test]
fn decode_and_localize_known_front_parallel_pose() {
    let calibration = CameraCalibration::new(600.0, 610.0, 320.0, 240.0);
    // Pose in grid-pitch units is t=(0.1,-0.05,4).  The normalized-camera
    // homography is scaled to h33=1 and then premultiplied by K here.
    let pixel_homography = Mat3::new([[150.0, 0.0, 335.0], [0.0, 152.5, 232.375], [0.0, 0.0, 1.0]]);
    let candidate = transformed_candidate(0x2d31, pixel_homography, [3, 8, 0, 4, 1, 7, 2, 6, 5]);
    let decoded = Decoder::default().decode(&candidate).unwrap();
    let localized = localize(&decoded, calibration, 0.2).unwrap();
    assert_eq!(decoded.id, 0x2d31);
    assert_eq!(localized.mean_decode_residual, decoded.best_score / 5.0);
    assert!(
        localized.mean_decode_residual < 1.0e-4,
        "unexpectedly large mean decode residual: {}",
        localized.mean_decode_residual,
    );
    let expected_translation = [0.005, -0.0025, 0.2];
    for (actual, expected) in localized
        .pose
        .translation
        .into_iter()
        .zip(expected_translation)
    {
        assert!((actual - expected).abs() < 2.0e-5, "{actual} != {expected}");
    }
    for row in 0..3 {
        for column in 0..3 {
            let expected = f64::from(row == column);
            assert!(
                (localized.pose.rotation[row][column] - expected).abs() < 5.0e-4,
                "R[{row}][{column}]={} != {expected}",
                localized.pose.rotation[row][column],
            );
        }
    }
}

#[test]
fn wavelet_sampling_phase_preserves_one_pose_across_all_planes() {
    let calibration = CameraCalibration::new(480.0, 480.0, 320.0, 240.0);
    // These independent logical centers are the inverse of the real sampling
    // locations: RAW (x,y), LL2 (4*x,4*y), LL4 (16*x,16*y+4).
    // Pitch 48 keeps all encoded symbol offsets exactly representable in Q4.
    for (plane, center, pitch) in [
        (Plane::Raw, Point2::new(320.0, 240.0), 48.0),
        (Plane::Ll2, Point2::new(80.0, 60.0), 12.0),
        (Plane::Ll4, Point2::new(20.0, 14.75), 3.0),
    ] {
        let candidate = ideal_q4_candidate(0x2a55, plane, center, pitch, 0);
        let decoded = Decoder::default().decode(&candidate).unwrap();
        let localized = localize_wavelet(&decoded, calibration, 0.2).unwrap();
        for (actual, expected) in localized.pose.translation.into_iter().zip([0.0, 0.0, 0.5]) {
            assert!(
                (actual - expected).abs() < 1.0e-10,
                "{plane:?}: {actual} != {expected}"
            );
        }
        for row in 0..3 {
            for column in 0..3 {
                let actual = localized.pose.rotation[row][column];
                let expected = f64::from(row == column);
                assert!(
                    (actual - expected).abs() < 1.0e-10,
                    "{plane:?}: R[{row}][{column}]={actual}"
                );
            }
        }
        if plane != Plane::Raw {
            let box_average = localize(&decoded, calibration, 0.2).unwrap();
            assert!(box_average.pose.translation[0] > 0.001);
            assert!(box_average.pose.translation[1] > 0.001);
        }
    }
}

#[test]
fn more_than_nine_children_decode_the_retained_tag_with_identical_pose() {
    let decoder = Decoder::default();
    let calibration = CameraCalibration::new(600.0, 610.0, 320.0, 240.0);
    let mut expected = rtl_smoke_candidate(0x1234);
    expected.centroids.sort_by_key(|point| point.log_mass_q5_3);
    let expected_decode = decoder.decode(&expected).unwrap();
    let expected_pose = localize(&expected_decode, calibration, 0.2).unwrap();

    // Small unrelated regions arrive both before and after the nine tag
    // children. Filling the retained list does not make the candidate invalid.
    let small = (0..18).map(|index| WeightedCentroid::new(index * 16, 16, 20 + index as u8));
    let mut input: Vec<_> = small.collect();
    input.splice(5..5, expected.centroids.into_iter().rev());
    let mut arena =
        lrcca::ChildArenaReferenceModel::with_sort_key(32, 9, false, |point: &WeightedCentroid| {
            point.log_mass_q5_3
        })
        .unwrap();
    let head = arena.allocate().unwrap();
    arena.init(head, input[0]).unwrap();
    for point in input.into_iter().skip(1) {
        arena.add(head, point).unwrap();
    }
    let retained: [WeightedCentroid; 9] = arena.read(head).unwrap().features.try_into().unwrap();
    assert_eq!(retained, expected.centroids);
    let actual = Candidate {
        centroids: retained,
        ..expected
    };
    let actual_decode = decoder.decode(&actual).unwrap();
    assert_eq!(actual_decode, expected_decode);
    assert_eq!(
        localize(&actual_decode, calibration, 0.2).unwrap(),
        expected_pose
    );
    assert_eq!(
        evaluate_transaction(&decoder, &actual),
        evaluate_transaction(&decoder, &expected),
    );
}

#[test]
fn plane_metadata_does_not_change_native_decode() {
    let raw = ideal_q4_candidate(0x1357, Plane::Raw, Point2::new(320.0, 240.0), 48.0, 3);
    let ll4 = ideal_q4_candidate(0x1357, Plane::Ll4, Point2::new(40.0, 25.0), 5.0, 3);
    assert_eq!(Decoder::default().decode(&raw).unwrap().id, 0x1357);
    assert_eq!(Decoder::default().decode(&ll4).unwrap().id, 0x1357);
}
