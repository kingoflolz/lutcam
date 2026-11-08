//! Long-running and external-solver centroid proofs.
//!
//! These tests are ignored by the ordinary suite for the reasons recorded on
//! each test. Run them explicitly with `cargo test -p lrcca --test
//! centroid_proofs -- --ignored --test-threads=1`.

use std::io::Write;
use std::process::{Command, Stdio};

use lrcca::{
    INTERNAL_FRAC_BITS, INTERNAL_OUTPUT_FRAC_BITS, NORMALIZED_MOMENT_FRAC_BITS,
    RECIPROCAL_FRAC_BITS, midpoint_reflection_reciprocal_q17, normalize_centered_moment_q4,
};

fn run_z3(script: &str, expected_checks: usize) {
    let mut child = Command::new("z3")
        .arg("-in")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("this ignored proof requires the external `z3` executable");
    child
        .stdin
        .take()
        .unwrap()
        .write_all(script.as_bytes())
        .unwrap();
    let output = child.wait_with_output().unwrap();
    assert!(
        output.status.success(),
        "z3 failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8(output.stdout).unwrap();
    let results = stdout
        .lines()
        .filter(|line| matches!(*line, "sat" | "unsat" | "unknown"))
        .collect::<Vec<_>>();
    assert_eq!(
        results.len(),
        expected_checks,
        "unexpected z3 output:\n{stdout}"
    );
    assert!(
        results.iter().all(|result| *result == "unsat"),
        "a proof obligation failed:\n{stdout}"
    );
}

fn bv(value: u128, bits: usize) -> String {
    format!("(_ bv{value} {bits})")
}

fn extract(high: usize, low: usize, expression: &str) -> String {
    format!("((_ extract {high} {low}) {expression})")
}

fn zero_extend(extra: usize, expression: &str) -> String {
    format!("((_ zero_extend {extra}) {expression})")
}

fn sign_extend(extra: usize, expression: &str) -> String {
    format!("((_ sign_extend {extra}) {expression})")
}

fn append_inequality_check(script: &mut String, left: &str, right: &str) {
    script.push_str("(push 1)\n(assert (not (= ");
    script.push_str(left);
    script.push(' ');
    script.push_str(right);
    script.push_str(")))\n(check-sat)\n(pop 1)\n");
}

fn append_power2_mass_checks(
    script: &mut String,
    name: &str,
    mass_bits: usize,
    frac_bits: usize,
) -> usize {
    let e_max = mass_bits - 1;
    let source_bits = mass_bits + frac_bits;
    let product_bits = mass_bits * 2;
    script.push_str(&format!("(declare-fun {name} () (_ BitVec {mass_bits}))\n"));
    for exponent in 0..=e_max {
        let shifted_source = format!(
            "(bvshl {} {})",
            zero_extend(frac_bits, name),
            bv(frac_bits as u128, source_bits)
        );
        let reference = extract(
            frac_bits - 1,
            0,
            &format!(
                "(bvlshr {shifted_source} {})",
                bv(exponent as u128, source_bits)
            ),
        );
        let scale = 1_u128 << (e_max - exponent);
        let product = format!(
            "(bvmul {} {})",
            zero_extend(mass_bits, name),
            zero_extend(mass_bits, &bv(scale, mass_bits))
        );
        debug_assert_eq!(product_bits, mass_bits * 2);
        let scaled = extract(e_max - 1, e_max - frac_bits, &product);
        append_inequality_check(script, &reference, &scaled);
    }
    e_max + 1
}

fn append_power2_signed_checks(
    script: &mut String,
    name: &str,
    value_bits: usize,
    output_bits: usize,
    e_max: usize,
    frac_bits: usize,
) -> usize {
    script.push_str(&format!(
        "(declare-fun {name} () (_ BitVec {value_bits}))\n"
    ));
    let value_ext = sign_extend(1, name);
    let scale_bits = e_max + 1;
    let signed_scale_bits = scale_bits + 1;
    let product_bits = value_bits + signed_scale_bits;
    let fixed_shift = e_max - frac_bits;

    for exponent in 0..=e_max {
        let reference = if exponent > frac_bits {
            let right_amount = exponent - frac_bits;
            let round_source = format!("(concat {value_ext} {})", bv(0, 1));
            let round_window = extract(
                output_bits,
                0,
                &format!(
                    "(bvashr {round_source} {})",
                    bv(right_amount as u128, value_bits + 2)
                ),
            );
            let quotient = extract(output_bits, 1, &round_window);
            let guard = zero_extend(output_bits - 1, &extract(0, 0, &round_window));
            format!("(bvadd {quotient} {guard})")
        } else {
            extract(
                output_bits - 1,
                0,
                &format!(
                    "(bvshl {value_ext} {})",
                    bv((frac_bits - exponent) as u128, value_bits + 1)
                ),
            )
        };

        let scale = bv(1_u128 << (e_max - exponent), scale_bits);
        let signed_scale = format!("(concat {} {scale})", bv(0, 1));
        let left = sign_extend(signed_scale_bits, name);
        let right = sign_extend(value_bits, &signed_scale);
        let product = format!("(bvmul {left} {right})");
        let quotient = format!(
            "(bvashr {product} {})",
            bv(fixed_shift as u128, product_bits)
        );
        let guard = zero_extend(
            product_bits - 1,
            &extract(fixed_shift - 1, fixed_shift - 1, &product),
        );
        let rounded = format!("(bvadd {quotient} {guard})");
        let scaled = extract(output_bits - 1, 0, &rounded);
        append_inequality_check(script, &reference, &scaled);
    }
    e_max + 1
}

#[test]
#[ignore = "requires the external z3 solver to prove every production-width bit pattern"]
fn power2_normalizer_identities_are_unsat_at_production_widths() {
    let mut script = String::from("(set-logic QF_BV)\n");
    let mut checks = 0;
    checks += append_power2_mass_checks(&mut script, "mass_1280", 28, 21);
    checks += append_power2_signed_checks(&mut script, "x_1280", 38, 17, 27, 4);
    checks += append_power2_mass_checks(&mut script, "mass_mapped", 29, 21);
    checks += append_power2_signed_checks(&mut script, "x_mapped", 39, 17, 28, 4);
    checks += append_power2_signed_checks(&mut script, "y_mapped", 38, 16, 28, 4);
    checks += append_power2_signed_checks(&mut script, "y_1280", 37, 16, 27, 4);
    script.push_str("(exit)\n");
    run_z3(&script, checks);
}

fn staged_window_expression(
    mut expression: String,
    mut expression_bits: usize,
    shift_amount: usize,
    window_bits: usize,
) -> String {
    for stage in (0..6).rev() {
        let amount = 1 << stage;
        let stage_bits = (window_bits + amount - 1).min(expression_bits);
        if shift_amount & amount != 0 {
            expression = extract(
                stage_bits - 1,
                0,
                &format!(
                    "(bvashr {expression} {})",
                    bv(amount as u128, expression_bits)
                ),
            );
        } else {
            expression = extract(stage_bits - 1, 0, &expression);
        }
        expression_bits = stage_bits;
    }
    expression
}

fn append_signed_window_checks(
    script: &mut String,
    name: &str,
    value_bits: usize,
    output_bits: usize,
) -> usize {
    const FRAC_BITS: usize = 4;
    const MAX_LEGAL_EXPONENT: usize = 28;
    let window_bits = output_bits + 1;
    let source_bits = value_bits + 2;
    let value_ext = sign_extend(1, name);
    let round_source = format!("(concat {value_ext} {})", bv(0, 1));
    script.push_str(&format!(
        "(declare-fun {name} () (_ BitVec {value_bits}))\n"
    ));
    let mut checks = 0;

    for exponent in 0..=MAX_LEGAL_EXPONENT {
        let right_shift = exponent.saturating_sub(FRAC_BITS);
        let staged =
            staged_window_expression(round_source.clone(), source_bits, right_shift, window_bits);
        let direct = extract(
            window_bits - 1,
            0,
            &format!(
                "(bvashr {round_source} {})",
                bv(right_shift as u128, source_bits)
            ),
        );
        append_inequality_check(script, &staged, &direct);
        checks += 1;

        if exponent > FRAC_BITS {
            let old_width = value_bits + 1;
            let round_bias = bv(1_u128 << (right_shift - 1), old_width);
            let old_wide = format!(
                "(bvashr (bvadd {value_ext} {round_bias}) {})",
                bv(right_shift as u128, old_width)
            );
            let old_result = extract(output_bits - 1, 0, &old_wide);
            let quotient = extract(window_bits - 1, 1, &staged);
            let guard = zero_extend(output_bits - 1, &extract(0, 0, &staged));
            let new_result = format!("(bvadd {quotient} {guard})");
            append_inequality_check(script, &old_result, &new_result);
            checks += 1;
        }
    }
    checks
}

#[test]
#[ignore = "requires the external z3 solver to exhaustively cover 37- to 39-bit signed values"]
fn staged_signed_moment_window_is_exhaustively_bit_identical() {
    let mut script = String::from("(set-logic QF_BV)\n");
    let mut checks = 0;
    checks += append_signed_window_checks(&mut script, "production_x", 39, 17);
    checks += append_signed_window_checks(&mut script, "production_y", 38, 16);
    checks += append_signed_window_checks(&mut script, "verilator_x", 38, 17);
    checks += append_signed_window_checks(&mut script, "verilator_y", 37, 16);
    script.push_str("(exit)\n");
    run_z3(&script, checks);
}

#[derive(Debug)]
struct Bounds {
    minimum: i128,
    maximum: i128,
}

impl Default for Bounds {
    fn default() -> Self {
        Self {
            minimum: i128::MAX,
            maximum: i128::MIN,
        }
    }
}

impl Bounds {
    fn observe(&mut self, value: i128) {
        self.minimum = self.minimum.min(value);
        self.maximum = self.maximum.max(value);
    }
}

fn check_axis(
    mass: u64,
    exponent: u32,
    reciprocal: i128,
    last_coordinate: u64,
    bounds: &mut Bounds,
) {
    let offset = (last_coordinate as i128) << (INTERNAL_OUTPUT_FRAC_BITS - 1);
    let endpoint_moment = mass as i128 * last_coordinate as i128;
    for sign in [-1, 1] {
        let normalized = normalize_centered_moment_q4(sign * endpoint_moment, exponent);
        let shift =
            NORMALIZED_MOMENT_FRAC_BITS + RECIPROCAL_FRAC_BITS + 1 - INTERNAL_OUTPUT_FRAC_BITS;
        let delta = (normalized * reciprocal + (1 << (shift - 1))) >> shift;
        bounds.observe(offset + delta);
    }
}

#[test]
#[ignore = "sweeps 30,571,825 production mass-cell endpoints and is intentionally expensive"]
fn midpoint_reflection_production_endpoints_are_exhaustive() {
    const WIDTH: u64 = 1696;
    const HEIGHT: u64 = 800;
    const MAX_MASS: u64 = WIDTH * HEIGHT * 255;
    let mut x_bounds = Bounds::default();
    let mut y_bounds = Bounds::default();
    let mut endpoint_evaluations = 0_u64;

    {
        let mut evaluate = |mass: u64, cell: u32, exponent: u32| {
            let reciprocal = midpoint_reflection_reciprocal_q17(cell).unwrap() as i128;
            check_axis(mass, exponent, reciprocal, WIDTH - 1, &mut x_bounds);
            check_axis(mass, exponent, reciprocal, HEIGHT - 1, &mut y_bounds);
            endpoint_evaluations += 1;
        };

        for exponent in 0_u32..=21 {
            let first_mass = 1_u64 << exponent;
            let mass_count = 1_u64 << exponent;
            for offset in 0..mass_count {
                let mass = first_mass + offset;
                let cell = if exponent == 21 {
                    offset as u32
                } else {
                    (offset << (INTERNAL_FRAC_BITS - exponent as usize)) as u32
                };
                evaluate(mass, cell, exponent);
            }
        }

        for exponent in 22_u32..=28 {
            let first_mass = 1_u64 << exponent;
            let discarded_bits = exponent as usize - INTERNAL_FRAC_BITS;
            let cell_width = 1_u64 << discarded_bits;
            let exponent_maximum = (1_u64 << (exponent + 1)) - 1;
            let last_mass = MAX_MASS.min(exponent_maximum);
            let last_cell = ((last_mass - first_mass) / cell_width) as u32;
            for cell in 0..=last_cell {
                let low = first_mass + cell as u64 * cell_width;
                let high = (low + cell_width - 1).min(MAX_MASS);
                evaluate(low, cell, exponent);
                evaluate(high, cell, exponent);
            }
        }
    }

    assert_eq!(endpoint_evaluations, 30_571_825);
    assert_eq!((x_bounds.minimum, x_bounds.maximum), (0, 27_120));
    assert_eq!((y_bounds.minimum, y_bounds.maximum), (0, 12_784));
}
