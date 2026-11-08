//! Differential regression for the fused four-key localization frontend.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use lftag::fixed::ideal_q4_candidate;
use lftag::rom::hypothesis_matrix;
use lftag::{
    CameraCalibration, Candidate, Decoder, Mat3, Plane, Point2, WeightedCentroid, canonical_points,
};
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, clock, read_u64, with_model_sources};

const FUSED_MARLIN_TOP: &str = "LfTagInitialHomographyFusedMarlin";
const ADAPTER: &str = "tests/lftag/tests/rtl_adapters/initial_homography.sv";
const FUSED_TOP: &str = "LfTagInitialHomographyFused";
const FUSED_SOURCES: &[&str] = &[
    "rtl/lftag/LfTagInitialHomographyCalibrationDatapath.sv",
    "rtl/lftag/LfTagInitialHomographyNormalizeDatapath.sv",
    "rtl/lftag/LfTagInitialHomographyFused.sv",
];
const INV_F_Q2_30: u32 = 0x0022_2222;
const CX_Q16: i32 = 320 << 16;
const CY_Q16: i32 = 240 << 16;

#[derive(Clone, Debug)]
struct Vector {
    key_x_q4: [i32; 4],
    key_y_q4: [i32; 4],
    cden: i32,
    cg: i32,
    ch: i32,
    hypothesis: u8,
    scale: u8,
    inv_fx: u32,
    inv_fy: u32,
    cx: i32,
    cy: i32,
    input_saturation: bool,
    h_common_q16: [i32; 9],
    invalid: bool,
    saturation: bool,
}

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn assert_success(description: &str, output: &Output) {
    assert!(
        output.status.success(),
        "{description} failed:\n{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
}

fn plane_code(plane: Plane) -> u8 {
    match plane {
        Plane::Raw => 0,
        Plane::Ll2 => 1,
        Plane::Ll4 => 2,
    }
}

fn calibration() -> CameraCalibration {
    let inverse_focal = f64::from(INV_F_Q2_30) / f64::from(1_u32 << 30);
    CameraCalibration::new(1.0 / inverse_focal, 1.0 / inverse_focal, 320.0, 240.0)
}

fn source_camera_transform(plane: Plane) -> Mat3 {
    let camera = calibration();
    let scale = f64::from(plane.scale());
    let phase_y = if plane == Plane::Ll4 { 4.0 } else { 0.0 };
    Mat3::new([
        [scale / camera.fx, 0.0, -camera.cx / camera.fx],
        [0.0, scale / camera.fy, (phase_y - camera.cy) / camera.fy],
        [0.0, 0.0, 1.0],
    ])
}

fn build_d_q6(vector: &Vector) -> ([i64; 9], bool) {
    fn source_q6(value_q4: i64, scale: u8, y_axis: bool) -> i64 {
        match scale {
            0 => value_q4 << 2,
            1 => value_q4 << 4,
            _ => (value_q4 << 6) + if y_axis { 256 } else { 0 },
        }
    }

    let mut x = [0_i64; 4];
    let mut y = [0_i64; 4];
    let mut saturation = vector.input_saturation
        || vector.scale == 3
        || vector.cden == 0
        || (vector.inv_fx >> 26) != 0
        || (vector.inv_fy >> 26) != 0;
    let quantize_inverse = |value: u32| {
        i64::from(
            (value >> 9)
                + u32::from(value & 0x100 != 0 && (value & 0xff != 0 || value & 0x200 != 0)),
        )
    };
    let inv_fx_q21 = quantize_inverse(vector.inv_fx);
    let inv_fy_q21 = quantize_inverse(vector.inv_fy);
    for point in 0..4 {
        let dx_q6 = source_q6(i64::from(vector.key_x_q4[point]), vector.scale, false)
            - (i64::from(vector.cx) >> 10);
        let dy_q6 = source_q6(i64::from(vector.key_y_q4[point]), vector.scale, true)
            - (i64::from(vector.cy) >> 10);
        x[point] = (dx_q6 * inv_fx_q21) >> 11;
        y[point] = (dy_q6 * inv_fy_q21) >> 11;
        saturation |=
            !(-131_072..=131_071).contains(&x[point]) || !(-131_072..=131_071).contains(&y[point]);
    }
    let cden = i64::from(vector.cden);
    let cg = i64::from(vector.cg);
    let ch = i64::from(vector.ch);
    let raw = [
        (x[1] - x[0]) * cden + x[1] * cg,
        (x[3] - x[0]) * cden + x[3] * ch,
        x[0] * cden,
        (y[1] - y[0]) * cden + y[1] * cg,
        (y[3] - y[0]) * cden + y[3] * ch,
        y[0] * cden,
    ];
    let mut d = [0_i64; 9];
    for index in 0..6 {
        d[index] = raw[index] >> 18;
        saturation |= !(-131_072..=131_071).contains(&d[index]);
    }
    d[6] = cg >> 2;
    d[7] = ch >> 2;
    d[8] = cden >> 2;
    (d, saturation)
}

fn common_scale_d(d: [i64; 9], hypothesis: u8) -> Option<[i32; 9]> {
    let inverse = hypothesis_matrix(hypothesis).inverse()?.normalized()?;
    let q: [i64; 9] =
        core::array::from_fn(|index| (inverse.m[index / 3][index % 3] * 65_536.0).round() as i64);
    let p: [i128; 9] = core::array::from_fn(|index| {
        let row = index / 3;
        let column = index % 3;
        (0..3)
            .map(|inner| i128::from(d[3 * row + inner]) * i128::from(q[3 * inner + column]))
            .sum()
    });
    if p[8] <= 0 {
        return None;
    }
    let scale = |value: i128, shift: i32| {
        if shift >= 0 {
            value >> shift
        } else {
            value << -shift
        }
    };
    let mut shift = 14;
    while scale(p[8], shift) < 32_768 {
        shift -= 1;
    }
    while scale(p[8], shift) >= 131_072 {
        shift += 1;
    }
    let h: [i128; 9] = p.map(|value| scale(value, shift));
    h.iter()
        .all(|&value| (-131_072..=131_071).contains(&value))
        .then(|| h.map(|value| value as i32))
}

#[test]
fn common_scale_retains_valid_high_coefficient_homographies() {
    let d = [-935, 1624, 16287, -4613, -1951, 1883, 953, -3476, 16384];
    assert_eq!(
        common_scale_d(d, 9),
        Some([-4131, 5923, 66731, -8794, -4098, -5383, -660, -4531, 60134,]),
    );
}

fn vector_from_candidate(candidate: Candidate) -> (Vector, f64) {
    let decoded = Decoder::default()
        .decode(&candidate)
        .expect("fixture must decode");
    let canonical = hypothesis_matrix(decoded.hypothesis);
    let observed_square = decoded.initial_homography * canonical;
    let keys = [
        decoded.correspondences[0].observed,
        decoded.correspondences[1].observed,
        decoded.correspondences[8].observed,
        decoded.correspondences[6].observed,
    ];
    let mut vector = Vector {
        key_x_q4: keys.map(|point| (point.x * 16.0).round() as i32),
        key_y_q4: keys.map(|point| (point.y * 16.0).round() as i32),
        cden: 65_536,
        cg: (observed_square.m[2][0] / observed_square.m[2][2] * 65_536.0).round() as i32,
        ch: (observed_square.m[2][1] / observed_square.m[2][2] * 65_536.0).round() as i32,
        hypothesis: decoded.hypothesis,
        scale: plane_code(decoded.plane),
        inv_fx: INV_F_Q2_30,
        inv_fy: INV_F_Q2_30,
        cx: CX_Q16,
        cy: CY_Q16,
        input_saturation: false,
        h_common_q16: [0; 9],
        invalid: false,
        saturation: false,
    };
    let (compact_d, compact_saturation) = build_d_q6(&vector);
    vector.saturation = compact_saturation;
    vector.h_common_q16 =
        common_scale_d(compact_d, vector.hypothesis).expect("valid compact fixture common scaling");

    let expected = (source_camera_transform(decoded.plane) * decoded.initial_homography)
        .normalized()
        .unwrap();
    let common_scale = f64::from(vector.h_common_q16[8]) / 65_536.0;
    let maximum_error = vector
        .h_common_q16
        .into_iter()
        .zip(expected.m.iter().flatten())
        .map(|(actual, expected)| (f64::from(actual) / common_scale - *expected * 65_536.0).abs())
        .fold(0.0, f64::max);
    (vector, maximum_error)
}

fn transformed_candidate(id: u16, transform: Mat3, noisy: bool) -> Candidate {
    let mut centroids = core::array::from_fn(|index| {
        let point = transform
            .transform_point(canonical_points(id)[index].position)
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
    if noisy {
        centroids[2].x_q4 += 32;
        centroids[3].y_q4 += 24;
        centroids[4].x_q4 -= 20;
        centroids[5].y_q4 -= 28;
        centroids[7].x_q4 += 16;
    }
    Candidate {
        centroids,
        plane: Plane::Raw,
        frame_sequence: 0,
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
    let transform = Mat3::new([
        [
            scale_x * cos + center_x * projective_x,
            -scale_y * sin + shear + center_x * projective_y,
            center_x,
        ],
        [
            scale_x * sin + center_y * projective_x,
            scale_y * cos + center_y * projective_y,
            center_y,
        ],
        [projective_x, projective_y, 1.0],
    ]);
    let canonical = canonical_points(id);
    let mut centroids: [WeightedCentroid; 9] = core::array::from_fn(|index| {
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
    for upper in (1..centroids.len()).rev() {
        let selected = (rng.next_u32() as usize) % (upper + 1);
        centroids.swap(upper, selected);
    }
    Candidate {
        centroids,
        plane: Plane::Raw,
        frame_sequence: rng.next_u32() as u8,
    }
}

fn vectors() -> (Vec<Vector>, f64) {
    let projective = Mat3::new([
        [52.0, 4.0, 320.0],
        [-3.0, 48.0, 240.0],
        [0.0012, -0.0008, 1.0],
    ]);
    let mut candidates = vec![
        ideal_q4_candidate(0x1357, Plane::Raw, Point2::new(320.0, 240.0), 48.0, 0),
        ideal_q4_candidate(0x2a55, Plane::Ll2, Point2::new(80.0, 60.0), 12.0, 0),
        ideal_q4_candidate(0x3abc, Plane::Ll4, Point2::new(20.0, 14.75), 3.0, 0),
        transformed_candidate(0x2a55, projective, false),
        transformed_candidate(0x2a55, projective, true),
        transformed_candidate(
            0x1555,
            Mat3::new([
                [44.0, -11.0, 510.0],
                [9.0, 50.0, 330.0],
                [-0.0015, 0.001, 1.0],
            ]),
            false,
        ),
    ];
    let mut rng = Lcg(0x6c66_7461_675f_6670);
    for _ in 0..2_048 {
        let id = (rng.next_u32() as u16) & 0x3fff;
        candidates.push(randomized_candidate(&mut rng, id));
    }

    let mut maximum_error = 0.0_f64;
    let mut result: Vec<_> = candidates
        .into_iter()
        .map(|candidate| {
            let (vector, error) = vector_from_candidate(candidate);
            maximum_error = maximum_error.max(error);
            vector
        })
        .collect();
    let mut carried_saturation = result[0].clone();
    carried_saturation.input_saturation = true;
    carried_saturation.saturation = true;
    result.push(carried_saturation);
    result.push(Vector {
        key_x_q4: [0; 4],
        key_y_q4: [0; 4],
        cden: 0,
        cg: 0,
        ch: 0,
        hypothesis: 0,
        scale: 0,
        inv_fx: INV_F_Q2_30,
        inv_fy: INV_F_Q2_30,
        cx: CX_Q16,
        cy: CY_Q16,
        input_saturation: false,
        h_common_q16: [0; 9],
        invalid: true,
        saturation: true,
    });
    (result, maximum_error)
}

fn strict_lint_for(root: &Path, top: &str, sources: &[&str], synthesis: bool) {
    let mut command = Command::new("verilator");
    command
        .current_dir(root)
        .args(["--lint-only", "--Wall", "--Wno-DECLFILENAME"]);
    if synthesis {
        command.arg("-DSYNTHESIS");
    }
    command.args(["--top-module", top]);
    command.args(sources);
    let output = command.output().expect("launch initial-homography lint");
    assert_success(
        if synthesis {
            "SYNTHESIS initial-homography lint"
        } else {
            "normal initial-homography lint"
        },
        &output,
    );
}

const FUSED_PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("in_valid", 0, 0, PortDirection::Input),
    ("in_ready", 0, 0, PortDirection::Output),
    ("in_key_x0_q4", 17, 0, PortDirection::Input),
    ("in_key_x1_q4", 17, 0, PortDirection::Input),
    ("in_key_x2_q4", 17, 0, PortDirection::Input),
    ("in_key_x3_q4", 17, 0, PortDirection::Input),
    ("in_key_y0_q4", 17, 0, PortDirection::Input),
    ("in_key_y1_q4", 17, 0, PortDirection::Input),
    ("in_key_y2_q4", 17, 0, PortDirection::Input),
    ("in_key_y3_q4", 17, 0, PortDirection::Input),
    ("in_cden", 17, 0, PortDirection::Input),
    ("in_cg", 17, 0, PortDirection::Input),
    ("in_ch", 17, 0, PortDirection::Input),
    ("in_hypothesis", 3, 0, PortDirection::Input),
    ("in_scale", 1, 0, PortDirection::Input),
    ("in_inv_fx_q2_30", 31, 0, PortDirection::Input),
    ("in_inv_fy_q2_30", 31, 0, PortDirection::Input),
    ("in_cx_q16", 31, 0, PortDirection::Input),
    ("in_cy_q16", 31, 0, PortDirection::Input),
    ("in_saturation", 0, 0, PortDirection::Input),
    ("out_valid", 0, 0, PortDirection::Output),
    ("out_ready", 0, 0, PortDirection::Input),
    ("result_write_valid", 0, 0, PortDirection::Output),
    ("result_write_address", 3, 0, PortDirection::Output),
    ("result_write_data", 17, 0, PortDirection::Output),
    ("out_invalid", 0, 0, PortDirection::Output),
    ("out_saturation", 0, 0, PortDirection::Output),
    ("out_cycles", 6, 0, PortDirection::Output),
];

fn reset(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("in_valid", 0_u8)?;
    model.pin("out_ready", 1_u8)?;
    model.pin("rst", 1_u8)?;
    for _ in 0..4 {
        clock(model)?;
    }
    model.pin("rst", 0_u8)?;
    clock(model)?;
    Ok(())
}

fn pin_vector(model: &mut DynamicVerilatedModel<'_>, vector: &Vector) -> TestResult {
    for (axis, values) in [("x", vector.key_x_q4), ("y", vector.key_y_q4)] {
        for (index, value) in values.into_iter().enumerate() {
            model.pin(format!("in_key_{axis}{index}_q4"), (value as u32) & 0x3ffff)?;
        }
    }
    model.pin("in_cden", (vector.cden as u32) & 0x3ffff)?;
    model.pin("in_cg", (vector.cg as u32) & 0x3ffff)?;
    model.pin("in_ch", (vector.ch as u32) & 0x3ffff)?;
    model.pin("in_hypothesis", vector.hypothesis)?;
    model.pin("in_scale", vector.scale)?;
    model.pin("in_inv_fx_q2_30", vector.inv_fx)?;
    model.pin("in_inv_fy_q2_30", vector.inv_fy)?;
    model.pin("in_cx_q16", vector.cx as u32)?;
    model.pin("in_cy_q16", vector.cy as u32)?;
    model.pin("in_saturation", u8::from(vector.input_saturation))?;
    Ok(())
}

fn transfer_input(model: &mut DynamicVerilatedModel<'_>, vector: &Vector) -> TestResult {
    for _ in 0..100 {
        model.eval();
        if read_u64(model, "in_ready")? != 0 {
            pin_vector(model, vector)?;
            model.pin("in_valid", 1_u8)?;
            clock(model)?;
            model.pin("in_valid", 0_u8)?;
            return Ok(());
        }
        clock(model)?;
    }
    Err("initial-homography input did not become ready".into())
}

fn fused_sources() -> Vec<&'static Path> {
    FUSED_SOURCES
        .iter()
        .copied()
        .chain([ADAPTER])
        .map(Path::new)
        .collect()
}

fn run_fused_vectors(root: &Path, vectors: &[Vector]) -> TestResult<u64> {
    let sources = fused_sources();
    with_model_sources(
        root,
        &sources,
        "lftag-initial-homography-fused-marlin-v2",
        FUSED_MARLIN_TOP,
        Path::new(ADAPTER),
        FUSED_PORTS,
        &[Path::new("rtl/lftag")],
        &[],
        |model| {
            reset(model)?;
            let mut maximum_cycles = 0;
            for (vector_index, vector) in vectors.iter().enumerate() {
                transfer_input(model, vector)?;
                let mut actual_h = [0_i32; 9];
                let mut written = 0_u16;
                for wait in 0..400 {
                    model.eval();
                    if read_u64(model, "result_write_valid")? != 0 {
                        let address = read_u64(model, "result_write_address")? as usize;
                        assert!(address < 9, "vector {vector_index} write address {address}");
                        actual_h[address] =
                            ((read_u64(model, "result_write_data")? as u32) << 14) as i32 >> 14;
                        written |= 1 << address;
                    }
                    if read_u64(model, "out_valid")? != 0 {
                        let invalid = read_u64(model, "out_invalid")? != 0;
                        assert_eq!(
                            invalid, vector.invalid,
                            "vector {vector_index} invalid mismatch"
                        );
                        assert_eq!(
                            read_u64(model, "out_saturation")? != 0,
                            vector.saturation,
                            "vector {vector_index} saturation mismatch",
                        );
                        if !invalid {
                            assert_eq!(written, 0x1ff, "vector {vector_index} missing H writes");
                            assert_eq!(
                                actual_h, vector.h_common_q16,
                                "vector {vector_index} common H mismatch"
                            );
                        }
                        let cycles = read_u64(model, "out_cycles")?;
                        maximum_cycles = maximum_cycles.max(cycles);
                        // The public seven-bit diagnostic saturates for rare
                        // large normalization shifts; the independent wait
                        // bound still detects a stalled serial datapath.
                        assert_eq!(
                            cycles,
                            (wait as u64).min(127),
                            "vector {vector_index} cycle accounting"
                        );
                        if vector_index + 1 == vectors.len() {
                            model.pin("out_ready", 0_u8)?;
                            for _ in 0..3 {
                                clock(model)?;
                                assert_eq!(read_u64(model, "out_valid")?, 1);
                                assert_eq!(read_u64(model, "out_invalid")? != 0, invalid);
                            }
                            model.pin("out_ready", 1_u8)?;
                        }
                        clock(model)?;
                        break;
                    }
                    assert!(wait + 1 < 400, "vector {vector_index} timed out");
                    clock(model)?;
                }
            }
            Ok(maximum_cycles)
        },
    )
}

#[test]
fn fused_four_key_frontend_matches_rust_on_full_noise_corpus() -> TestResult {
    let root = gateware_root();
    strict_lint_for(&root, FUSED_TOP, FUSED_SOURCES, false);
    strict_lint_for(&root, FUSED_TOP, FUSED_SOURCES, true);

    let (vectors, maximum_error) = vectors();
    assert!(
        maximum_error <= 24.0,
        "normalized common-scale frontend error was {maximum_error} Q16 LSB"
    );
    assert_eq!(vectors.len(), 2_056);
    let fused_maximum = run_fused_vectors(&root, &vectors)?;
    assert_eq!(fused_maximum, 79);
    eprintln!(
        "fused four-key frontend: {} vectors, latency<=79 cycles, normalized common-scale error={} Q16 LSB",
        vectors.len(),
        maximum_error,
    );
    Ok(())
}
