//! Keep the checked-in pose ROM in sync with its Rust generator.

use lftag::rom::pose_reciprocal_sqrt_seeds;

#[test]
fn pose_scale_table_is_monotonic_and_matches_rtl() {
    let pose_reciprocal_sqrt = pose_reciprocal_sqrt_seeds();
    assert!(
        pose_reciprocal_sqrt
            .windows(2)
            .all(|pair| pair[0] >= pair[1])
    );
    let checked_in = include_str!("../../../rtl/lftag/lftag_rsqrt_512x18_init.svh")
        .split("18'h")
        .skip(1)
        .map(|suffix| u32::from_str_radix(&suffix[..5], 16).unwrap())
        .collect::<Vec<_>>();
    assert_eq!(checked_in.as_slice(), pose_reciprocal_sqrt.as_slice());
}
