//! Exhaustive and boundary-focused centroid arithmetic properties ported from
//! the LRCCA Python reference suite.

use lrcca::{
    INTERNAL_FRAC_BITS, INTERNAL_OUTPUT_FRAC_BITS, NORMALIZED_MOMENT_FRAC_BITS, OUTPUT_FRAC_BITS,
    RECIPROCAL_FRAC_BITS, RECIPROCAL_MIDPOINT_DISCARD_BITS, RECIPROCAL_MIDPOINT_PRODUCT_SHIFT,
    RECIPROCAL_MIDPOINT_RESIDUAL_BITS, log_m_q5_3, log_m_q5_3_threshold,
    midpoint_reflection_coefficients_q18, midpoint_reflection_reciprocal_q17,
    normalize_centered_moment_q4, reciprocal_q17, reciprocal_segments_q17,
};

fn mask(bits: usize) -> u64 {
    (1_u64 << bits) - 1
}

fn signed(value: u64, bits: usize) -> i64 {
    let value = value & mask(bits);
    if value & (1 << (bits - 1)) != 0 {
        value as i64 - (1_i64 << bits)
    } else {
        value as i64
    }
}

fn moment_lsb(center: u64, mass: u64) -> u64 {
    (center & 1) & (mass & 1)
}

fn reconstruct(quotient: u64, mass: u64, center: u64, bits: usize) -> u64 {
    ((quotient << 1) | moment_lsb(center, mass)) & mask(bits)
}

fn check_quotient_update(
    bits: usize,
    center: u64,
    mass: u64,
    quotient: u64,
    coordinate: u64,
    weight: u64,
) {
    let full = reconstruct(quotient, mass, center, bits);
    let delta_signed = (2 * coordinate as i64 - center as i64) * weight as i64;
    let delta = (delta_signed as u64) & mask(bits);
    let delta_quotient = delta >> 1;
    let carry = moment_lsb(center, mass) & moment_lsb(center, weight);
    let quotient_next = (quotient + delta_quotient + carry) & mask(bits - 1);
    let mass_next = mass + weight;
    let expected = (full + delta) & mask(bits);

    assert_eq!(signed(quotient, bits - 1), signed(full, bits) >> 1);
    assert_eq!(signed(delta_quotient, bits - 1), signed(delta, bits) >> 1);
    assert_eq!(
        reconstruct(quotient_next, mass_next, center, bits),
        expected
    );
}

fn check_quotient_merge(
    bits: usize,
    center: u64,
    mass_a: u64,
    quotient_a: u64,
    mass_b: u64,
    quotient_b: u64,
) {
    let full_a = reconstruct(quotient_a, mass_a, center, bits);
    let full_b = reconstruct(quotient_b, mass_b, center, bits);
    let carry = moment_lsb(center, mass_a) & moment_lsb(center, mass_b);
    let quotient = (quotient_a + quotient_b + carry) & mask(bits - 1);
    let mass = mass_a + mass_b;
    assert_eq!(
        reconstruct(quotient, mass, center, bits),
        (full_a + full_b) & mask(bits)
    );
}

#[test]
fn centered_moment_quotient_update_is_exhaustive_at_small_widths() {
    for bits in 3..8 {
        for center in 0..4 {
            for mass in 0..8 {
                for quotient in 0..1_u64 << (bits - 1) {
                    for coordinate in 0..4 {
                        for weight in 0..4 {
                            check_quotient_update(bits, center, mass, quotient, coordinate, weight);
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn centered_moment_quotient_merge_is_exhaustive_at_small_widths() {
    for bits in 3..8 {
        for center in 0..2 {
            for mass_a in 0..4 {
                for mass_b in 0..4 {
                    for quotient_a in 0..1_u64 << (bits - 1) {
                        for quotient_b in 0..1_u64 << (bits - 1) {
                            check_quotient_merge(
                                bits, center, mass_a, quotient_a, mass_b, quotient_b,
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn centered_moment_quotient_covers_production_edges() {
    for (bits, center) in [(39_usize, 1695_u64), (38, 799)] {
        let quotient_edges = [0, 1, (1 << (bits - 2)) - 1, 1 << (bits - 2), mask(bits - 1)];
        for mass in [0, 1, 2, 255, 256, 0x1fff_ffff] {
            for quotient in quotient_edges {
                for coordinate in [0, 1, center / 2, center.div_ceil(2), center] {
                    for weight in [0, 1, 2, 127, 128, 255] {
                        check_quotient_update(bits, center, mass, quotient, coordinate, weight);
                    }
                }
            }
        }
        for mass_a in [0, 1, 2, 255] {
            for mass_b in [0, 1, 2, 255] {
                for quotient_a in quotient_edges {
                    for quotient_b in quotient_edges {
                        check_quotient_merge(bits, center, mass_a, quotient_a, mass_b, quotient_b);
                    }
                }
            }
        }
    }
}

fn combine_quotient(
    bits: usize,
    center: u64,
    mass_a: u64,
    quotient_a: u64,
    mass_b: u64,
    quotient_b: u64,
) -> (u64, u64) {
    let carry = moment_lsb(center, mass_a) & moment_lsb(center, mass_b);
    (
        (mass_a + mass_b) & mask(bits),
        (quotient_a + quotient_b + carry) & mask(bits - 1),
    )
}

fn three_mass_parity_carry(center: u64, mass_a: u64, mass_b: u64, mass_c: u64) -> u64 {
    let (a, b, c) = (mass_a & 1, mass_b & 1, mass_c & 1);
    (center & 1) & ((a & b) | ((a ^ b) & c))
}

#[test]
fn three_mass_parity_and_modular_combine_are_exhaustive() {
    for a in 0..2 {
        for b in 0..2 {
            for c in 0..2 {
                let first_carry = a & b;
                let second_carry = (a ^ b) & c;
                let direct_carry = (a & b) | ((a ^ b) & c);
                assert!(!(first_carry != 0 && second_carry != 0));
                assert_eq!(first_carry + second_carry, direct_carry);
                assert_eq!(three_mass_parity_carry(1, a, b, c), direct_carry);
                assert_eq!(three_mass_parity_carry(0, a, b, c), 0);
            }
        }
    }

    for quotient_bits in 1..7 {
        let modulus_mask = mask(quotient_bits);
        for a in 0..2 {
            for b in 0..2 {
                for c in 0..2 {
                    let first_carry = a & b;
                    let second_carry = (a ^ b) & c;
                    let direct_carry = three_mass_parity_carry(1, a, b, c);
                    for quotient_a in 0..1_u64 << quotient_bits {
                        for quotient_b in 0..1_u64 << quotient_bits {
                            for quotient_c in 0..1_u64 << quotient_bits {
                                let after_ab =
                                    (quotient_a + quotient_b + first_carry) & modulus_mask;
                                let sequential =
                                    (after_ab + quotient_c + second_carry) & modulus_mask;
                                let direct = (quotient_a + quotient_b + quotient_c + direct_carry)
                                    & modulus_mask;
                                assert_eq!(direct, sequential);
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn three_mass_modular_combine_covers_target_width_edges() {
    for quotient_bits in [37, 38] {
        let modulus_mask = mask(quotient_bits);
        let edges = [
            0,
            1,
            (1 << (quotient_bits - 1)) - 1,
            1 << (quotient_bits - 1),
            modulus_mask,
        ];
        for a in 0..2 {
            for b in 0..2 {
                for c in 0..2 {
                    let first_carry = a & b;
                    let second_carry = (a ^ b) & c;
                    let direct_carry = three_mass_parity_carry(1, a, b, c);
                    for quotient_a in edges {
                        for quotient_b in edges {
                            for quotient_c in edges {
                                let after_ab =
                                    (quotient_a + quotient_b + first_carry) & modulus_mask;
                                let sequential =
                                    (after_ab + quotient_c + second_carry) & modulus_mask;
                                let direct = (quotient_a + quotient_b + quotient_c + direct_carry)
                                    & modulus_mask;
                                assert_eq!(direct, sequential);
                            }
                        }
                    }
                }
            }
        }
    }
}

fn pair_delta_quotient(
    bits: usize,
    center: u64,
    x0: u64,
    weight0: u64,
    weight1: u64,
) -> (u64, u64) {
    let weight_sum = weight0 + weight1;
    let delta = (2 * x0 as i64 - center as i64) * weight_sum as i64 + 2 * weight1 as i64;
    (
        weight_sum & mask(bits),
        ((delta as u64) >> 1) & mask(bits - 1),
    )
}

#[test]
fn pair_fused_quotient_matches_two_lane_order_exhaustive_small_widths() {
    for bits in 3..6 {
        for center in 0..4 {
            for x0 in 0..3 {
                for base_mass in 0..4 {
                    for base_q in 0..1_u64 << (bits - 1) {
                        for prev_mass in 0..4 {
                            for prev_q in 0..1_u64 << (bits - 1) {
                                for raw_weight0 in 0..4 {
                                    for raw_weight1 in 0..4 {
                                        for d0 in [false, true] {
                                            for d1 in [false, true] {
                                                let weight0 = if d0 { raw_weight0 } else { 0 };
                                                let weight1 = if d1 { raw_weight1 } else { 0 };
                                                for selected_prev in [-1_i8, 0, 1] {
                                                    if (selected_prev == 0 && !d0)
                                                        || (selected_prev == 1 && !d1)
                                                    {
                                                        continue;
                                                    }

                                                    let (mut seq_mass, mut seq_q) =
                                                        (base_mass, base_q);
                                                    if d0 {
                                                        let delta0 = (((2 * x0 as i64
                                                            - center as i64)
                                                            * weight0 as i64)
                                                            as u64)
                                                            & mask(bits);
                                                        (seq_mass, seq_q) = combine_quotient(
                                                            bits,
                                                            center,
                                                            seq_mass,
                                                            seq_q,
                                                            weight0,
                                                            delta0 >> 1,
                                                        );
                                                        if selected_prev == 0 {
                                                            (seq_mass, seq_q) = combine_quotient(
                                                                bits, center, seq_mass, seq_q,
                                                                prev_mass, prev_q,
                                                            );
                                                        }
                                                    }
                                                    if d1 {
                                                        let delta1 = (((2 * (x0 + 1) as i64
                                                            - center as i64)
                                                            * weight1 as i64)
                                                            as u64)
                                                            & mask(bits);
                                                        (seq_mass, seq_q) = combine_quotient(
                                                            bits,
                                                            center,
                                                            seq_mass,
                                                            seq_q,
                                                            weight1,
                                                            delta1 >> 1,
                                                        );
                                                        if selected_prev == 1 {
                                                            (seq_mass, seq_q) = combine_quotient(
                                                                bits, center, seq_mass, seq_q,
                                                                prev_mass, prev_q,
                                                            );
                                                        }
                                                    }

                                                    let (pair_mass, pair_q) = pair_delta_quotient(
                                                        bits, center, x0, weight0, weight1,
                                                    );
                                                    let (mut fused_mass, mut fused_q) =
                                                        combine_quotient(
                                                            bits, center, base_mass, base_q,
                                                            pair_mass, pair_q,
                                                        );
                                                    if selected_prev >= 0 {
                                                        (fused_mass, fused_q) = combine_quotient(
                                                            bits, center, fused_mass, fused_q,
                                                            prev_mass, prev_q,
                                                        );
                                                    }
                                                    assert_eq!(
                                                        (fused_mass, fused_q),
                                                        (seq_mass, seq_q)
                                                    );
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

fn sat_add(left: u64, right: u64, maximum: u64) -> u64 {
    maximum.min(left + right)
}

#[test]
fn pair_fused_area_keeps_lane_order_exhaustive() {
    for bits in 1..5 {
        let representable = mask(bits);
        for maximum in 0..=representable {
            for base in 0..=representable {
                for prev in 0..=representable {
                    for feature0 in 0..2 {
                        for feature1 in 0..2 {
                            for d0 in [false, true] {
                                for d1 in [false, true] {
                                    for selected_prev in [-1_i8, 0, 1] {
                                        if (selected_prev == 0 && !d0)
                                            || (selected_prev == 1 && !d1)
                                        {
                                            continue;
                                        }
                                        let mut sequential = base;
                                        if d0 {
                                            sequential = sat_add(sequential, feature0, maximum);
                                            if selected_prev == 0 {
                                                sequential = sat_add(sequential, prev, maximum);
                                            }
                                        }
                                        if d1 {
                                            sequential = sat_add(sequential, feature1, maximum);
                                            if selected_prev == 1 {
                                                sequential = sat_add(sequential, prev, maximum);
                                            }
                                        }

                                        let mut fused = base;
                                        if d0 {
                                            fused = sat_add(fused, feature0, maximum);
                                            if selected_prev == 0 {
                                                fused = sat_add(fused, prev, maximum);
                                            }
                                        }
                                        if d1 {
                                            fused = sat_add(fused, feature1, maximum);
                                            if selected_prev == 1 {
                                                fused = sat_add(fused, prev, maximum);
                                            }
                                        }
                                        assert_eq!(fused, sequential);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn pair_fused_previous_selection_is_one_hot_for_all_s5_predicates() {
    for valid in [false, true] {
        for a in [false, true] {
            for b0 in [false, true] {
                for b1 in [false, true] {
                    for c in [false, true] {
                        for d0 in [false, true] {
                            for d1 in [false, true] {
                                let o0 = valid && b0 && d0 && (!a || !c);
                                let o1 = valid && b1 && d1 && (!b0 || !d0);
                                for merge_prev0 in [false, true] {
                                    for merge_prev1 in [false, true] {
                                        assert!(!(o0 && merge_prev0 && o1 && merge_prev1));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

fn edge_sum(mass: u64, last_coordinate: u64, sign: i128) -> i128 {
    let exponent = 63 - mass.leading_zeros();
    let normalized = if exponent as usize >= INTERNAL_FRAC_BITS {
        mass >> (exponent as usize - INTERNAL_FRAC_BITS)
    } else {
        mass << (INTERNAL_FRAC_BITS - exponent as usize)
    };
    let reciprocal = reciprocal_q17((normalized - (1 << INTERNAL_FRAC_BITS)) as u32).unwrap();
    let moment =
        normalize_centered_moment_q4(sign * mass as i128 * last_coordinate as i128, exponent);
    let delta_q4 = (moment * reciprocal as i128 + (1 << 17)) >> 18;
    ((last_coordinate as i128) << (INTERNAL_OUTPUT_FRAC_BITS - 1)) + delta_q4
}

#[test]
fn endpoint_clamp_is_identical_over_proven_reachable_intervals() {
    for last_coordinate in [1695_i128, 799, 1273, 793] {
        let maximum = last_coordinate << INTERNAL_OUTPUT_FRAC_BITS;
        for value in 0..=maximum {
            let clamped = value.clamp(0, maximum);
            assert_eq!(value, clamped);
        }
    }

    for mass in [1, 39, 7837, 1 << 20] {
        for last_coordinate in [1273, 793] {
            for sign in [-1, 1] {
                let coordinate_q4 = edge_sum(mass, last_coordinate, sign);
                assert!(
                    (0..=(last_coordinate as i128) << INTERNAL_OUTPUT_FRAC_BITS)
                        .contains(&coordinate_q4)
                );
                assert_eq!((coordinate_q4 << 1) & 1, 0);
            }
        }
    }
}

#[test]
fn segment_interpolation_error_bounds_are_exhaustive() {
    let normalized_scale = 1_u64 << INTERNAL_FRAC_BITS;
    let reciprocal_scale = 1_u64 << RECIPROCAL_FRAC_BITS;
    let reciprocal_target = reciprocal_scale * normalized_scale;
    let mut worst_error = 0_u64;
    let mut worst_case = None;
    for normalized_fraction in 0..normalized_scale {
        let reciprocal = reciprocal_q17(normalized_fraction as u32).unwrap() as u64;
        for endpoint in [normalized_fraction, normalized_fraction + 1] {
            let product = reciprocal * (normalized_scale + endpoint);
            let error = product.abs_diff(reciprocal_target);
            if error > worst_error {
                worst_error = error;
                worst_case = Some((normalized_fraction, endpoint));
            }
        }
    }
    // 4_053_230 / 2^38 reduces to 2_026_615 / 2^37.
    assert_eq!(worst_error, 4_053_230);
    assert_eq!(worst_case, Some((2_015_202, 2_015_202)));

    let x_numerator = 2_026_615_u128 * 1695 + (1 << 17) + (1 << 33) + (1 << 32);
    let y_numerator = 2_026_615_u128 * 799 + (1 << 17) + (1 << 33) + (1 << 32);
    assert_eq!(x_numerator, 16_320_145_385);
    assert_eq!(y_numerator, 14_504_298_345);
    assert!(x_numerator * 100 < 9_u128 * (1 << 38));
    assert!(y_numerator * 100 < 9_u128 * (1 << 38));
}

#[test]
fn quantized_mass_log_thresholds_are_exhaustively_bit_identical() {
    let base = 1_u64 << INTERNAL_FRAC_BITS;
    for normalized_fraction in 0..1_u64 << INTERNAL_FRAC_BITS {
        let mass = base + normalized_fraction;
        assert_eq!(log_m_q5_3_threshold(mass), log_m_q5_3(mass));
    }
    for mass in [
        0,
        1,
        (1 << 31) - 1,
        1 << 31,
        (1 << 32) - 1,
        1 << 32,
        (1 << 40) + 12_345,
    ] {
        assert_eq!(log_m_q5_3_threshold(mass), log_m_q5_3(mass));
    }
}

fn floor_div_power_of_two(value: i128, shift: usize) -> i128 {
    let scale = 1_i128 << shift;
    if value >= 0 {
        value / scale
    } else {
        -((-value + scale - 1) / scale)
    }
}

fn bit_length(value: u128) -> usize {
    (u128::BITS - value.leading_zeros()) as usize
}

#[test]
fn q4_moment_normalization_widths_and_constant_product_rounding() {
    let reciprocals = [
        reciprocal_q17(0).unwrap(),
        reciprocal_q17(1).unwrap(),
        reciprocal_q17(1 << 20).unwrap(),
        reciprocal_q17((1 << 21) - 1).unwrap(),
    ];
    let centered_values = [
        -(1_i128 << 38),
        -65_537,
        -129,
        -65,
        -64,
        -1,
        0,
        1,
        63,
        64,
        65,
        65_537,
        (1_i128 << 38) - 1,
    ];
    let product_shift =
        NORMALIZED_MOMENT_FRAC_BITS + RECIPROCAL_FRAC_BITS + 1 - INTERNAL_OUTPUT_FRAC_BITS;
    for exponent in 0..29 {
        for centered in centered_values {
            let normalized = normalize_centered_moment_q4(centered, exponent);
            let scaled_error = (normalized * (1_i128 << exponent)
                - centered * (1 << NORMALIZED_MOMENT_FRAC_BITS))
                .abs();
            assert!(scaled_error * 2 <= 1_i128 << exponent);
            for reciprocal in reciprocals {
                let product = normalized * reciprocal as i128;
                let direct_q4 = (product + (1 << (product_shift - 1))) >> product_shift;
                let exact_round =
                    floor_div_power_of_two(product + (1 << (product_shift - 1)), product_shift);
                assert_eq!(direct_q4, exact_round);
            }
        }
    }

    let x_bound = 1695_u128 << (NORMALIZED_MOMENT_FRAC_BITS + 1);
    let y_bound = 799_u128 << (NORMALIZED_MOMENT_FRAC_BITS + 1);
    let x_normalized_bits = bit_length(x_bound + 1) + 1;
    let y_normalized_bits = bit_length(y_bound + 1) + 1;
    assert_eq!(NORMALIZED_MOMENT_FRAC_BITS, 4);
    assert_eq!((x_normalized_bits, y_normalized_bits), (17, 16));
    assert_eq!(
        (
            x_normalized_bits + RECIPROCAL_FRAC_BITS + 1,
            y_normalized_bits + RECIPROCAL_FRAC_BITS + 1,
        ),
        (35, 34)
    );
    assert_eq!(product_shift, 18);
    assert_eq!(OUTPUT_FRAC_BITS, INTERNAL_OUTPUT_FRAC_BITS + 1);
}

#[test]
fn midpoint_reflection_concat_identity_is_exhaustive() {
    let retained_bits = RECIPROCAL_MIDPOINT_RESIDUAL_BITS - RECIPROCAL_MIDPOINT_DISCARD_BITS;
    let retained_mask = (1_u32 << retained_bits) - 1;
    let reflection_bits =
        1 + (INTERNAL_FRAC_BITS - RECIPROCAL_MIDPOINT_RESIDUAL_BITS) + retained_bits;
    assert_eq!(reflection_bits, 19);
    for normalized_fraction in 0..1_u32 << INTERNAL_FRAC_BITS {
        let segment = normalized_fraction >> RECIPROCAL_MIDPOINT_RESIDUAL_BITS;
        let retained_fraction =
            (normalized_fraction >> RECIPROCAL_MIDPOINT_DISCARD_BITS) & retained_mask;
        let rtl_reflection = (1 << (reflection_bits - 1))
            | (segment << retained_bits)
            | ((!retained_fraction) & retained_mask);

        let fraction_mask = (1 << RECIPROCAL_MIDPOINT_RESIDUAL_BITS) - 1;
        let segment_base = segment << RECIPROCAL_MIDPOINT_RESIDUAL_BITS;
        let fraction = normalized_fraction & fraction_mask;
        let reflected_full = (1 << INTERNAL_FRAC_BITS) + segment_base + fraction_mask - fraction;
        let expected =
            (reflected_full >> RECIPROCAL_MIDPOINT_DISCARD_BITS) & ((1 << reflection_bits) - 1);
        assert_eq!(rtl_reflection, expected);
    }
}

#[test]
fn midpoint_reciprocal_error_and_product_bounds_are_exhaustive() {
    let normalized_scale = 1_u64 << INTERNAL_FRAC_BITS;
    let reciprocal_scale = 1_u64 << RECIPROCAL_FRAC_BITS;
    let target = reciprocal_scale * normalized_scale;
    let coefficients = midpoint_reflection_coefficients_q18();
    let mut worst_error = 0_u64;
    let mut worst_case = None;
    let mut minimum_reciprocal = reciprocal_scale;
    let mut maximum_reciprocal = 0_u64;
    let mut minimum_product = u64::MAX;
    let mut maximum_product = 0_u64;

    for normalized_fraction in 0..normalized_scale {
        let segment = normalized_fraction as usize >> RECIPROCAL_MIDPOINT_RESIDUAL_BITS;
        let fraction = normalized_fraction & ((1 << RECIPROCAL_MIDPOINT_RESIDUAL_BITS) - 1);
        let reflected = (normalized_scale
            + ((segment as u64) << RECIPROCAL_MIDPOINT_RESIDUAL_BITS)
            + ((1 << RECIPROCAL_MIDPOINT_RESIDUAL_BITS) - 1)
            - fraction)
            >> RECIPROCAL_MIDPOINT_DISCARD_BITS;
        let product = coefficients[segment] as u64 * reflected;
        let reciprocal = midpoint_reflection_reciprocal_q17(normalized_fraction as u32).unwrap();
        assert_eq!(
            reciprocal as u64,
            product >> RECIPROCAL_MIDPOINT_PRODUCT_SHIFT
        );
        minimum_reciprocal = minimum_reciprocal.min(reciprocal as u64);
        maximum_reciprocal = maximum_reciprocal.max(reciprocal as u64);
        minimum_product = minimum_product.min(product);
        maximum_product = maximum_product.max(product);
        for endpoint in [normalized_fraction, normalized_fraction + 1] {
            let error = (reciprocal as u64 * (normalized_scale + endpoint)).abs_diff(target);
            if error > worst_error {
                worst_error = error;
                worst_case = Some((normalized_fraction, endpoint, reciprocal));
            }
        }
    }
    // 4_257_008 / 2^38 reduces to 266_063 / 2^34.
    assert_eq!(worst_error, 4_257_008);
    assert_eq!(worst_case, Some((2_055_271, 2_055_272, 66_198)));
    assert_eq!((minimum_reciprocal, maximum_reciprocal), (65_536, 131_071));
    assert_eq!(minimum_product, 34_360_229_376);
    assert_eq!(maximum_product, 68_719_215_615);
    assert!(maximum_product < 1 << 36);

    let x_numerator = 266_063_u128 * 1695 + (1 << 14) + (1 << 30) + (1 << 29);
    let y_numerator = 266_063_u128 * 799 + (1 << 14) + (1 << 30) + (1 << 29);
    assert_eq!(x_numerator, 2_061_605_905);
    assert_eq!(y_numerator, 1_823_213_457);
    assert!(x_numerator * 100 < 9_u128 * (1 << 35));
    assert!(y_numerator * 100 < 9_u128 * (1 << 35));
}

#[test]
fn power2_normalizer_is_exhaustive_at_reduced_widths() {
    let mass_bits = 7;
    let e_max = mass_bits - 1;
    let mass_frac_bits = 4;
    let moment_frac_bits = 2;
    let value_bits = 7;
    let output_bits = 6;
    let mass_mask = mask(mass_frac_bits);
    let output_mask = mask(output_bits);
    let product_bits = value_bits + mass_bits + 1;
    let product_mask = mask(product_bits);

    for exponent in 0..=e_max {
        let scale = 1_i64 << (e_max - exponent);
        for mass in 0..1_u64 << mass_bits {
            let reference_mass = ((mass << mass_frac_bits) >> exponent) & mass_mask;
            let scaled_mass = ((mass * scale as u64) >> (e_max - mass_frac_bits)) & mass_mask;
            assert_eq!(scaled_mass, reference_mass);
        }
        for encoded in 0..1_u64 << value_bits {
            let value = signed(encoded, value_bits);
            let reference_value = if exponent <= moment_frac_bits {
                value << (moment_frac_bits - exponent)
            } else {
                let shift = exponent - moment_frac_bits;
                (value >> shift) + ((encoded >> (shift - 1)) & 1) as i64
            } as u64
                & output_mask;
            let product = value * scale;
            let product_encoded = product as u64 & product_mask;
            let fixed_shift = e_max - moment_frac_bits;
            let scaled_value =
                (product >> fixed_shift) + ((product_encoded >> (fixed_shift - 1)) & 1) as i64;
            assert_eq!(scaled_value as u64 & output_mask, reference_value);
        }
    }
}

fn encode_signed(value: i128, bits: usize) -> u128 {
    value as u128 & ((1_u128 << bits) - 1)
}

fn decode_signed(value: u128, bits: usize) -> i128 {
    let value = value & ((1_u128 << bits) - 1);
    if value & (1 << (bits - 1)) != 0 {
        value as i128 - (1_i128 << bits)
    } else {
        value as i128
    }
}

fn staged_signed_window(
    value: u128,
    value_bits: usize,
    shift_amount: usize,
    window_bits: usize,
) -> u128 {
    let mut current = value;
    let mut current_bits = value_bits;
    for stage in (0..6).rev() {
        let amount = 1 << stage;
        let stage_bits = (window_bits + amount - 1).min(current_bits);
        if shift_amount & amount != 0 {
            current = encode_signed(decode_signed(current, current_bits) >> amount, stage_bits);
        } else {
            current &= (1_u128 << stage_bits) - 1;
        }
        current_bits = stage_bits;
    }
    current & ((1_u128 << window_bits) - 1)
}

#[test]
fn staged_signed_moment_window_is_exhaustive_at_reduced_widths() {
    const FRAC_BITS: usize = 4;
    for (value_bits, output_bits) in [(7, 4), (8, 5), (10, 6)] {
        let window_bits = output_bits + 1;
        let source_bits = value_bits + 2;
        // Production's largest right shift is smaller than its signed value
        // width. Preserve that invariant: beyond it the fixed-width
        // rounding-bias addition can itself wrap, which is outside the RTL's
        // legal exponent/value configurations.
        for exponent in 0_usize..=FRAC_BITS + value_bits {
            let right_shift = exponent.saturating_sub(FRAC_BITS);
            for encoded in 0..1_u128 << value_bits {
                let value_ext = encode_signed(decode_signed(encoded, value_bits), value_bits + 1);
                let round_source = (value_ext << 1) & ((1_u128 << source_bits) - 1);
                let staged =
                    staged_signed_window(round_source, source_bits, right_shift, window_bits);
                let direct = encode_signed(
                    decode_signed(round_source, source_bits) >> right_shift,
                    window_bits,
                );
                assert_eq!(staged, direct);

                let old_result = if exponent > FRAC_BITS {
                    let old_width = value_bits + 1;
                    let bias = 1_u128 << (right_shift - 1);
                    let biased = (value_ext + bias) & ((1_u128 << old_width) - 1);
                    encode_signed(decode_signed(biased, old_width) >> right_shift, output_bits)
                } else {
                    (value_ext << (FRAC_BITS - exponent)) & ((1_u128 << output_bits) - 1)
                };
                let new_result = if exponent > FRAC_BITS {
                    let quotient = staged >> 1;
                    let guard = staged & 1;
                    (quotient + guard) & ((1_u128 << output_bits) - 1)
                } else {
                    old_result
                };
                assert_eq!(new_result, old_result);
            }
        }
    }
}

#[test]
fn split_base_interpolation_is_exhaustively_bit_identical() {
    const INTERPOLATION_BITS: usize = 13;
    const BASE_SPLIT_BITS: usize = 4;
    let segments = reciprocal_segments_q17();
    let mut minimum_term = 0_i64;
    let mut maximum_term = 0_i64;
    let mut maximum_c = 0_i64;

    for normalized_fraction in 0_u32..1 << 21 {
        let segment = normalized_fraction as usize >> INTERPOLATION_BITS;
        let fraction = normalized_fraction & ((1 << INTERPOLATION_BITS) - 1);
        let (base, drop) = segments[segment];
        let old_drop = (drop * fraction + (1 << (INTERPOLATION_BITS - 1))) >> INTERPOLATION_BITS;
        let old_reciprocal = base - old_drop;

        let base_high = base >> BASE_SPLIT_BITS;
        let base_low = base & ((1 << BASE_SPLIT_BITS) - 1);
        let split_c =
            ((base_low << INTERPOLATION_BITS) + ((1 << (INTERPOLATION_BITS - 1)) - 1)) as i64;
        let numerator = split_c - (drop * fraction) as i64;
        let split_term = numerator >> INTERPOLATION_BITS;
        let new_reciprocal = (((base_high as i64 + (split_term >> BASE_SPLIT_BITS))
            << BASE_SPLIT_BITS)
            | (split_term & ((1 << BASE_SPLIT_BITS) - 1))) as u32;

        assert_eq!(new_reciprocal, old_reciprocal);
        assert!((-(1 << 22)..1 << 22).contains(&numerator));
        assert!((-(1 << 9)..1 << 9).contains(&split_term));
        assert!(new_reciprocal < 1 << 17);
        minimum_term = minimum_term.min(split_term);
        maximum_term = maximum_term.max(split_term);
        maximum_c = maximum_c.max(split_c);
    }
    assert!((-509..=0).contains(&minimum_term));
    assert!((0..=15).contains(&maximum_term));
    assert!(maximum_c <= 126_975 && 126_975 < 1 << 17);
    assert_eq!((4095 - 509 * 8191) >> 13, -509);
    assert_eq!(((15 << 13) + 4095) >> 13, 15);
}

#[test]
fn split_base_floor_identity_covers_rounding_ties() {
    for drop in 128_i64..510 {
        for fraction in [0, 1, 4095, 4096, 4097, 8191] {
            let product = drop * fraction;
            let rounded_drop = (product + 4096) >> 13;
            let negated_drop = (4095 - product) >> 13;
            assert_eq!(negated_drop, -rounded_drop);
        }
    }
}
