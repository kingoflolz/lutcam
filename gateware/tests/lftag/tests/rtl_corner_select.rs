//! Differential regression for the compact, FIFO-streaming corner selector.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use lftag::corner_select::{CornerSelectStatus, CornerSelection, select_sorted_streaming_corners};
use lftag::{Candidate, Decoder, Mat3, Plane, WeightedCentroid, canonical_points};
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, clock, read_u64, with_model_sources};

const TOP: &str = "LfTagCornerSelect9";
const MARLIN_TOP: &str = "LfTagCornerSelect9Marlin";
const CORE_SOURCE: &str = "rtl/lftag/LfTagCornerSelect9.sv";
const ADAPTER: &str = "tests/lftag/tests/rtl_adapters/corner_select9.sv";
const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("in_valid", 0, 0, PortDirection::Input),
    ("in_ready", 0, 0, PortDirection::Output),
    ("in_points_valid", 0, 0, PortDirection::Input),
    ("in_min_baseline_q4", 15, 0, PortDirection::Input),
    ("desc_req_valid", 0, 0, PortDirection::Output),
    ("desc_req_index", 3, 0, PortDirection::Output),
    ("desc_rsp_valid", 0, 0, PortDirection::Input),
    ("desc_rsp_x_q4", 17, 0, PortDirection::Input),
    ("desc_rsp_y_q4", 17, 0, PortDirection::Input),
    ("desc_rsp_mass_q5_3", 7, 0, PortDirection::Input),
    ("desc_rsp_centroid_valid", 0, 0, PortDirection::Input),
    ("out_valid", 0, 0, PortDirection::Output),
    ("out_ready", 0, 0, PortDirection::Input),
    ("out_status", 2, 0, PortDirection::Output),
    ("out_input_valid", 0, 0, PortDirection::Output),
    ("out_baseline_valid", 0, 0, PortDirection::Output),
    ("out_dominant_valid", 0, 0, PortDirection::Output),
    ("out_corners_valid", 0, 0, PortDirection::Output),
    ("out_quad_valid", 0, 0, PortDirection::Output),
    ("out_corner_index_flat", 15, 0, PortDirection::Output),
    ("out_remaining_index_flat", 19, 0, PortDirection::Output),
    ("out_corner_x0_q4", 17, 0, PortDirection::Output),
    ("out_corner_x1_q4", 17, 0, PortDirection::Output),
    ("out_corner_x2_q4", 17, 0, PortDirection::Output),
    ("out_corner_x3_q4", 17, 0, PortDirection::Output),
    ("out_corner_y0_q4", 17, 0, PortDirection::Output),
    ("out_corner_y1_q4", 17, 0, PortDirection::Output),
    ("out_corner_y2_q4", 17, 0, PortDirection::Output),
    ("out_corner_y3_q4", 17, 0, PortDirection::Output),
    ("out_cycles", 7, 0, PortDirection::Output),
];

#[derive(Clone, Copy)]
struct Vector {
    points: [WeightedCentroid; 9],
    points_valid: bool,
    selection: CornerSelection,
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

fn randomized_candidate(rng: &mut Lcg, id: u16) -> Candidate {
    let angle = rng.symmetric(core::f64::consts::PI);
    let (sin, cos) = angle.sin_cos();
    let scale_x = 28.0 + 64.0 * rng.unit();
    let scale_y = scale_x * (0.7 + 0.6 * rng.unit());
    let shear = rng.symmetric(scale_x * 0.1);
    let center_x = 180.0 + 820.0 * rng.unit();
    let center_y = 140.0 + 500.0 * rng.unit();
    let projective_x = rng.symmetric(0.07);
    let projective_y = rng.symmetric(0.07);
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
    let mut centroids = core::array::from_fn(|index| {
        let point = transform
            .transform_point(canonical[index].position)
            .expect("bounded projective fixture");
        WeightedCentroid::new(
            ((point.x + rng.symmetric(0.08)) * 16.0).round() as i32,
            ((point.y + rng.symmetric(0.08)) * 16.0).round() as i32,
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

fn sorted_descriptors(candidate: &Candidate) -> ([WeightedCentroid; 9], [usize; 9]) {
    let mut descriptors = candidate
        .centroids
        .into_iter()
        .enumerate()
        .collect::<Vec<_>>();
    descriptors.sort_by_key(|(_, point)| point.log_mass_q5_3);
    let points = core::array::from_fn(|index| descriptors[index].1);
    let original_indices = core::array::from_fn(|index| descriptors[index].0);
    (points, original_indices)
}

fn packed_indices<const N: usize>(indices: [u8; N]) -> u32 {
    indices
        .into_iter()
        .enumerate()
        .fold(0, |packed, (slot, index)| {
            packed | (u32::from(index) << (4 * slot))
        })
}

fn deterministic_vectors() -> Vec<Vector> {
    let decoder = Decoder::default();
    let mut rng = Lcg(0x4c46_5461_675f_3939);
    let mut vectors = Vec::new();
    let mut corner_mismatches = 0usize;
    let mut left_mismatches = 0usize;
    let mut right_mismatches = 0usize;
    for case in 0..2_048 {
        let id = (rng.next_u32() as u16) & 0x3fff;
        let candidate = randomized_candidate(&mut rng, id);
        let decoded = decoder
            .decode(&candidate)
            .unwrap_or_else(|reason| panic!("random case {case} rejected: {reason:?}"));
        let (points, original_indices) = sorted_descriptors(&candidate);
        let selection = select_sorted_streaming_corners(points, 16);
        assert_eq!(selection.status, CornerSelectStatus::Valid);
        let hardware_corners = selection
            .corners
            .map(|index| original_indices[index as usize]);
        let decoder_corners = [
            usize::from(decoded.correspondences[0].input_index),
            usize::from(decoded.correspondences[1].input_index),
            usize::from(decoded.correspondences[8].input_index),
            usize::from(decoded.correspondences[6].input_index),
        ];
        if hardware_corners != decoder_corners {
            corner_mismatches += 1;
            left_mismatches += usize::from(hardware_corners[3] != decoder_corners[3]);
            right_mismatches += usize::from(hardware_corners[2] != decoder_corners[2]);
        }
        vectors.push(Vector {
            points,
            points_valid: true,
            selection,
        });
    }
    assert_eq!(
        corner_mismatches, 0,
        "bounded-pose differential: {corner_mismatches}/2048 groups, \
         left={left_mismatches}, right={right_mismatches}"
    );

    let mut invalid = vectors[0];
    invalid.points_valid = false;
    invalid.selection = select_sorted_streaming_corners(
        invalid.points.map(|mut point| {
            point.valid = false;
            point
        }),
        16,
    );
    vectors.push(invalid);

    let mut short = vectors[1];
    short.points[8].x_q4 = short.points[7].x_q4;
    short.points[8].y_q4 = short.points[7].y_q4;
    short.selection = select_sorted_streaming_corners(short.points, 16);
    assert_eq!(short.selection.status, CornerSelectStatus::BaselineTooShort);
    vectors.push(short);

    let mut collinear = vectors[2];
    for (index, point) in collinear.points.iter_mut().enumerate() {
        point.x_q4 = 1_000 + index as i32 * 100;
        point.y_q4 = 2_000;
    }
    collinear.selection = select_sorted_streaming_corners(collinear.points, 16);
    assert_eq!(collinear.selection.status, CornerSelectStatus::DominantSide);
    vectors.push(collinear);

    let mut tied_anchors = vectors[3];
    tied_anchors.points[6].log_mass_q5_3 = tied_anchors.points[7].log_mass_q5_3;
    tied_anchors.selection = select_sorted_streaming_corners(tied_anchors.points, 16);
    assert_eq!(
        tied_anchors.selection.status,
        CornerSelectStatus::CornerSelection
    );
    vectors.push(tied_anchors);

    let mut active_plane_edge = vectors[4];
    let maximum_x = active_plane_edge
        .points
        .iter()
        .map(|point| point.x_q4)
        .max()
        .unwrap();
    let maximum_y = active_plane_edge
        .points
        .iter()
        .map(|point| point.y_q4)
        .max()
        .unwrap();
    for point in &mut active_plane_edge.points {
        point.x_q4 += (1 << 15) - 1 - maximum_x;
        point.y_q4 += (1 << 14) - 1 - maximum_y;
    }
    active_plane_edge.selection = select_sorted_streaming_corners(active_plane_edge.points, 16);
    assert_eq!(
        active_plane_edge.selection.status,
        CornerSelectStatus::Valid
    );
    vectors.push(active_plane_edge);

    for (point_index, x_q4, y_q4) in [
        (8, -1, active_plane_edge.points[8].y_q4),
        (8, 1 << 15, active_plane_edge.points[8].y_q4),
        (7, active_plane_edge.points[7].x_q4, -1),
        (7, active_plane_edge.points[7].x_q4, 1 << 14),
    ] {
        let mut outside = active_plane_edge;
        outside.points[point_index].x_q4 = x_q4;
        outside.points[point_index].y_q4 = y_q4;
        outside.selection = select_sorted_streaming_corners(outside.points, 16);
        assert_eq!(outside.selection.status, CornerSelectStatus::InvalidInput);
        vectors.push(outside);
    }
    vectors
}

fn strict_lint(root: &Path, synthesis: bool) {
    let mut command = Command::new("verilator");
    command
        .current_dir(root)
        .args(["--lint-only", "--Wall", "--top-module", TOP]);
    if synthesis {
        command.arg("-DSYNTHESIS");
    }
    let output = command
        .arg(CORE_SOURCE)
        .output()
        .expect("failed to launch Verilator lint");
    assert_success("corner-selector lint", &output);
}

struct Driver<'model, 'ctx> {
    dut: &'model mut DynamicVerilatedModel<'ctx>,
    pending_descriptor: Option<usize>,
}

impl<'model, 'ctx> Driver<'model, 'ctx> {
    fn corner_coordinates(&self) -> TestResult<[u64; 8]> {
        Ok([
            read_u64(self.dut, "out_corner_x0_q4")?,
            read_u64(self.dut, "out_corner_x1_q4")?,
            read_u64(self.dut, "out_corner_x2_q4")?,
            read_u64(self.dut, "out_corner_x3_q4")?,
            read_u64(self.dut, "out_corner_y0_q4")?,
            read_u64(self.dut, "out_corner_y1_q4")?,
            read_u64(self.dut, "out_corner_y2_q4")?,
            read_u64(self.dut, "out_corner_y3_q4")?,
        ])
    }

    fn new(dut: &'model mut DynamicVerilatedModel<'ctx>) -> TestResult<Self> {
        let mut driver = Self {
            dut,
            pending_descriptor: None,
        };
        driver.dut.pin("in_valid", 0_u8)?;
        driver.dut.pin("in_points_valid", 0_u8)?;
        driver.dut.pin("in_min_baseline_q4", 16_u16)?;
        driver.pin_no_response()?;
        driver.dut.pin("out_ready", 1_u8)?;
        driver.dut.pin("rst", 1_u8)?;
        for _ in 0..3 {
            clock(driver.dut)?;
        }
        driver.dut.pin("rst", 0_u8)?;
        clock(driver.dut)?;
        assert_eq!(read_u64(driver.dut, "in_ready")?, 1);
        Ok(driver)
    }

    fn pin_no_response(&mut self) -> TestResult {
        self.dut.pin("desc_rsp_valid", 0_u8)?;
        self.dut.pin("desc_rsp_x_q4", 0_u32)?;
        self.dut.pin("desc_rsp_y_q4", 0_u32)?;
        self.dut.pin("desc_rsp_mass_q5_3", 0_u8)?;
        self.dut.pin("desc_rsp_centroid_valid", 0_u8)?;
        Ok(())
    }

    /// Model the one-cycle synchronous descriptor memory used by the FIFO.
    fn tick(&mut self, points: &[WeightedCentroid; 9]) -> TestResult {
        if let Some(index) = self.pending_descriptor {
            let point = points[index];
            self.dut.pin("desc_rsp_valid", 1_u8)?;
            self.dut
                .pin("desc_rsp_x_q4", (point.x_q4 as u32) & 0x3ffff)?;
            self.dut
                .pin("desc_rsp_y_q4", (point.y_q4 as u32) & 0x3ffff)?;
            self.dut.pin("desc_rsp_mass_q5_3", point.log_mass_q5_3)?;
            self.dut
                .pin("desc_rsp_centroid_valid", u8::from(point.valid))?;
        } else {
            self.pin_no_response()?;
        }
        self.dut.eval();
        let request = if read_u64(self.dut, "desc_req_valid")? != 0 {
            let index = read_u64(self.dut, "desc_req_index")? as usize;
            assert!(
                index < points.len(),
                "descriptor request {index} out of range"
            );
            Some(index)
        } else {
            None
        };
        clock(self.dut)?;
        self.pending_descriptor = request;
        Ok(())
    }

    fn check_vector(&mut self, vector_index: usize, vector: &Vector) -> TestResult<u8> {
        let mut ready_timeout = 0;
        while read_u64(self.dut, "in_ready")? == 0 {
            self.tick(&vector.points)?;
            ready_timeout += 1;
            assert!(ready_timeout < 150, "vector {vector_index} input timeout");
        }
        self.dut
            .pin("in_points_valid", u8::from(vector.points_valid))?;
        self.dut.pin("in_min_baseline_q4", 16_u16)?;
        self.dut.pin("in_valid", 1_u8)?;
        self.tick(&vector.points)?;
        self.dut.pin("in_valid", 0_u8)?;

        let mut timeout = 0;
        while read_u64(self.dut, "out_valid")? == 0 {
            self.tick(&vector.points)?;
            timeout += 1;
            assert!(timeout < 150, "vector {vector_index} output timeout");
        }

        let status = read_u64(self.dut, "out_status")?;
        assert_eq!(
            status, vector.selection.status as u64,
            "vector {vector_index}"
        );
        let flags = read_u64(self.dut, "out_input_valid")?
            | (read_u64(self.dut, "out_baseline_valid")? << 1)
            | (read_u64(self.dut, "out_dominant_valid")? << 2)
            | (read_u64(self.dut, "out_corners_valid")? << 3)
            | (read_u64(self.dut, "out_quad_valid")? << 4);
        assert_eq!(
            flags,
            u64::from(vector.selection.validity.bits()),
            "vector {vector_index} validity"
        );
        let cycles = read_u64(self.dut, "out_cycles")? as u8;
        assert!(cycles <= 47, "vector {vector_index} latency {cycles}");

        if vector.selection.status == CornerSelectStatus::Valid {
            let corner_indices = packed_indices(vector.selection.corners);
            assert_eq!(
                read_u64(self.dut, "out_corner_index_flat")?,
                u64::from(corner_indices),
                "vector {vector_index} corner indices"
            );
            assert_eq!(
                read_u64(self.dut, "out_remaining_index_flat")?,
                u64::from(packed_indices(vector.selection.remaining)),
                "vector {vector_index} remaining indices"
            );
            let coordinates = self.corner_coordinates()?;
            for slot in 0..4 {
                let point = vector.points[((corner_indices >> (4 * slot)) & 0xf) as usize];
                assert_eq!(
                    coordinates[slot],
                    u64::from((point.x_q4 as u32) & 0x3ffff),
                    "vector {vector_index} corner {slot} x"
                );
                assert_eq!(
                    coordinates[slot + 4],
                    u64::from((point.y_q4 as u32) & 0x3ffff),
                    "vector {vector_index} corner {slot} y"
                );
            }
        }

        // Periodically hold the completed transaction and prove every result
        // field is stable under backpressure.
        if vector_index.is_multiple_of(97) {
            let held = (
                read_u64(self.dut, "out_status")?,
                read_u64(self.dut, "out_corner_index_flat")?,
                read_u64(self.dut, "out_remaining_index_flat")?,
                self.corner_coordinates()?,
                read_u64(self.dut, "out_cycles")?,
            );
            self.dut.pin("out_ready", 0_u8)?;
            for _ in 0..3 {
                self.tick(&vector.points)?;
                assert_eq!(read_u64(self.dut, "out_valid")?, 1);
                assert_eq!(read_u64(self.dut, "out_status")?, held.0);
                assert_eq!(read_u64(self.dut, "out_corner_index_flat")?, held.1);
                assert_eq!(read_u64(self.dut, "out_remaining_index_flat")?, held.2);
                assert_eq!(self.corner_coordinates()?, held.3);
                assert_eq!(read_u64(self.dut, "out_cycles")?, held.4);
            }
            self.dut.pin("out_ready", 1_u8)?;
        }
        self.tick(&vector.points)?;
        Ok(cycles)
    }
}

#[test]
fn streaming_corner_selector_matches_rust_and_general_decoder() -> TestResult {
    let root = gateware_root();
    strict_lint(&root, false);
    strict_lint(&root, true);

    let vectors = deterministic_vectors();
    let adapter = root.join(ADAPTER);
    with_model_sources(
        &root,
        &[Path::new(CORE_SOURCE)],
        "lftag-corner-select-direct-v2",
        MARLIN_TOP,
        Path::new(ADAPTER),
        PORTS,
        &[],
        &[adapter.as_path()],
        |dut| {
            let mut driver = Driver::new(dut)?;
            let mut maximum_cycles = 0;
            for (index, vector) in vectors.iter().enumerate() {
                maximum_cycles = maximum_cycles.max(driver.check_vector(index, vector)?);
            }
            assert_eq!(maximum_cycles, 47);
            Ok(())
        },
    )
}
