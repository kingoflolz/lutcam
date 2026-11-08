use lrcca::{adaptive_threshold_planes, build_wavelet_composite};
use lwc::{UnifiedFrontendReference, unified_frontend_reference};

fn flat_taps() -> UnifiedFrontendReference {
    let mut frame = unified_frontend_reference(&vec![200; 64 * 64], 64, 64).unwrap();
    frame.ll2.samples.fill(100 - 128);
    frame.ll4.samples.fill(0);
    frame
}

#[test]
fn raw_rounding_cap_and_equality_are_fixed() {
    let mut frame = flat_taps();
    frame.stage0.samples[0] = 86;
    frame.stage0.samples[1] = 87;
    let result = adaptive_threshold_planes(&frame).unwrap();
    // Constant LL2=100 gives 67.1875 + 20, rounded to 87.
    assert!(result.raw_thresholds.iter().all(|&t| t == 87));
    assert_eq!(&result.raw_base_mask[..2], &[0, 1]);
    assert!(result.masks[2].iter().all(|&p| p == 1));

    frame.ll2.samples.fill(200 - 128);
    for y in 0..frame.ll2.height {
        for x in 7..=9 {
            frame.ll2.samples[y * frame.ll2.width + x] = 32 - 128;
        }
    }
    let result = adaptive_threshold_planes(&frame).unwrap();
    // A bright surrounding window is capped by the dark local three taps.
    assert_eq!(result.raw_thresholds[20 * 64 + 32], 80);
    frame.ll2.samples.fill(-343);
    assert!(
        adaptive_threshold_planes(&frame)
            .unwrap()
            .raw_thresholds
            .iter()
            .all(|&t| t == 40)
    );
    frame.ll2.samples.fill(342);
    assert!(
        adaptive_threshold_planes(&frame)
            .unwrap()
            .raw_thresholds
            .iter()
            .all(|&t| t == 128)
    );
}

#[test]
fn ll2_preserves_contrast_at_lower_brightness_with_white_equality() {
    for divisor in [1, 2, 4, 8] {
        let mut frame = flat_taps();
        frame.ll4.samples.fill(96 / divisor - 128);
        frame.ll2.samples[..5]
            .copy_from_slice(&[16, 95, 96, 97, 160].map(|sample| sample / divisor - 128));
        let planes = adaptive_threshold_planes(&frame).unwrap();
        assert_eq!(&planes.masks[1][..5], &[0, 0, 1, 1, 1]);
    }
}

#[test]
fn diagonal_repair_connects_without_adding_centroid_mass() {
    let mut frame = flat_taps();
    let center = 10 * 64 + 10;
    frame.stage0.samples[center] = 88;
    frame.stage0.samples[center - 64] = 20;
    frame.stage0.samples[center - 1] = 20;
    let planes = adaptive_threshold_planes(&frame).unwrap();
    assert_eq!(planes.raw_base_mask[center], 1);
    assert_eq!(planes.masks[0][center], 0);
    let composite = build_wavelet_composite(&frame, &Default::default()).unwrap();
    assert_eq!(composite.feature_valid[10][10], 0);
    assert_eq!(composite.feature_valid[9][10], 1);
    assert_eq!(composite.feature_valid[10][9], 1);
}

#[test]
fn malformed_taps_are_rejected_before_sampling() {
    let mut frame = flat_taps();
    frame.ll2.samples.pop();
    assert!(adaptive_threshold_planes(&frame).is_err());
    let mut frame = flat_taps();
    frame.ll4.width = 0;
    assert!(build_wavelet_composite(&frame, &Default::default()).is_err());
}
