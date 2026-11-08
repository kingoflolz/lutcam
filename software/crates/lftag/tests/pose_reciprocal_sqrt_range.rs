use lftag::{PoseMathError, derive_lambda_q10, pose_from_homography};

fn integer_radicand(h: [i32; 9]) -> u64 {
    [0, 1, 3, 4, 6, 7]
        .into_iter()
        .map(|i| i64::from(h[i]).pow(2) as u64)
        .sum::<u64>()
        / 2
}

fn check_against_math(h: [i32; 9]) {
    let radicand = integer_radicand(h);
    if radicand == 0 {
        assert_eq!(derive_lambda_q10(h), Err(PoseMathError::InvalidLambda));
        return;
    }
    let ideal = 67_108_864.0 / (radicand as f64).sqrt();
    let lambda = derive_lambda_q10(h).unwrap_or_else(|error| {
        panic!("r={radicand} ideal={ideal} h={h:?}: {error:?}");
    });
    let normalization_steps = 34_u32.saturating_sub(radicand.ilog2()).div_ceil(2);
    let quantum = f64::from(1_u32 << normalization_steps.saturating_sub(8));
    assert!(
        (f64::from(lambda) - ideal).abs() <= ideal / 1024.0 + quantum,
        "r={radicand} h={h:?} got={lambda} ideal={ideal}",
    );
}

#[test]
fn power_of_four_column_norm_has_a_representable_pose_scale() {
    let h = [8192, 0, 0, 0, 8192, 0, 0, 0, 65536];
    // Independent mathematical result: Q16 columns have norm 1/8, so
    // their reciprocal is 8, or 8192 in Q10.
    let lambda = derive_lambda_q10(h).expect("representable reciprocal norm");
    assert!((f64::from(lambda) - 8192.0).abs() <= 9.0, "{lambda}");
    check_against_math(h);
}

#[test]
fn both_exponent_parities_and_all_signed18_column_magnitudes_match_math() {
    let mut leading_bits_seen = 0_u64;
    for magnitude in 1..=131_071 {
        // One, two, and six nonzero terms span every possible leading bit
        // of the integer norm, including odd exponents and the upper range.
        for h in [
            [magnitude, 0, 0, 0, 0, 0, 0, 0, 65536],
            [magnitude, 0, 0, 0, magnitude, 0, 0, 0, 65536],
            [
                -magnitude, magnitude, 0, magnitude, -magnitude, 0, -magnitude, magnitude, 65536,
            ],
        ] {
            let radicand = integer_radicand(h);
            if radicand > 0 {
                leading_bits_seen |= 1 << radicand.ilog2();
            }
            check_against_math(h);
        }
    }
    assert_eq!(leading_bits_seen, (1_u64 << 36) - 1);
    check_against_math([
        -131072, -131072, 0, -131072, -131072, 0, -131072, -131072, 65536,
    ]);
}

#[test]
fn small_norms_reconstruct_valid_poses() {
    assert_eq!(
        derive_lambda_q10([0, 0, 0, 0, 0, 0, 0, 0, 65536]),
        Err(PoseMathError::InvalidLambda),
    );
    for h in [
        [1, 0, 0, 0, 1, 0, 0, 0, 65536],
        [511, 0, 0, 0, 511, 0, 0, 0, 65536],
        [512, 0, 0, 0, 512, 0, 0, 0, 65536],
        // An odd square sum truncates to the same arithmetic boundary.
        [512, 1, 0, 0, 512, 0, 0, 0, 65536],
    ] {
        check_against_math(h);
        assert!(pose_from_homography(h, 65536).is_ok());
    }
    let minimum_valid = [512, 1, 0, 1, 512, 0, 0, 0, 65536];
    assert_eq!(integer_radicand(minimum_valid), (1 << 18) + 1);
    check_against_math(minimum_valid);

    assert_eq!(
        derive_lambda_q10([131072, 0, 0, 0, 8192, 0, 0, 0, 65536]),
        Err(PoseMathError::InvalidSigned18Encoding),
    );
    assert_eq!(
        derive_lambda_q10([8192, 0, 0, 0, 8192, 0, 0, 0, 32767]),
        Err(PoseMathError::InvalidHomographyScale),
    );
}

#[test]
fn expanded_lambda_range_still_rejects_real_translation_overflow() {
    let h = [1, 0, 0, 0, 1, 0, 0, 0, 131071];
    assert!(derive_lambda_q10(h).is_ok());
    assert_eq!(
        pose_from_homography(h, 131071),
        Err(PoseMathError::InvalidPoseRange)
    );
}
