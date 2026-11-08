//! Transaction-level Rust/RTL differential tests for the standalone LFTag
//! decoder and localizer.

use std::collections::VecDeque;
use std::path::{Path, PathBuf};

use lftag::fixed::{
    ACCEPTED_COMPUTE_TARGET, ACCEPTED_CYCLE_WATCHDOG, ACCEPTED_SERVICE_TARGET,
    TransportRejectReason, evaluate_transaction, ideal_q4_candidate, parse_transport_reject_code,
    quantize_signed, reject_code, rtl_smoke_candidate,
};
use lftag::localize::wavelet_logical_to_source;
use lftag::{
    CameraCalibration, Candidate, Decoder, Mat3, POSE_PACKET_WORDS, Plane, Point2, PosePacket,
    RejectReason, WeightedCentroid, canonical_points, fit_homography, localize_wavelet,
};
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, clock, read_u64};

const MANIFEST: &str = "rtl/lftag/sources.f";
const ADAPTER: &str = "tests/lftag/tests/rtl_adapters/core_lftag.sv";
const PACKET_WAIT_LIMIT: usize = 2_000;

const INV_F_Q2_30: u32 = 0x0022_2222;
const CX_Q16_16: u32 = 320 << 16;
const CY_Q16_16: u32 = 240 << 16;
const PRINTED_WIDTH_Q16_16: u32 = 0x0000_3333;
const MEAN_RESIDUAL_LIMIT_Q16: u16 = 1024;
const INTEGRATED_SCORE_TOLERANCE_Q32: u64 = 1 << 16;
// Conditioning now measures the normalized rotation columns, adding eleven
// serialized cycles. Shared projection priming adds sixteen more clocks
// while preserving the accepted-path compute and service budgets.
const REPRESENTATIVE_COMPUTE_CYCLES: u16 = 733;
const REPRESENTATIVE_SERVICE_CYCLES: usize = 756;

const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("clear_diagnostics", 0, 0, PortDirection::Input),
    ("in_valid", 0, 0, PortDirection::Input),
    ("in_group_end", 0, 0, PortDirection::Input),
    ("in_x_q4", 14, 0, PortDirection::Input),
    ("in_y_q4", 13, 0, PortDirection::Input),
    ("in_log_mass_q5_3", 7, 0, PortDirection::Input),
    ("in_centroid_valid", 0, 0, PortDirection::Input),
    ("in_scale", 1, 0, PortDirection::Input),
    ("in_frame_seq", 7, 0, PortDirection::Input),
    ("cfg_inv_fx_q2_30", 31, 0, PortDirection::Input),
    ("cfg_inv_fy_q2_30", 31, 0, PortDirection::Input),
    ("cfg_cx_q16_16", 31, 0, PortDirection::Input),
    ("cfg_cy_q16_16", 31, 0, PortDirection::Input),
    ("cfg_printed_width_q16_16", 31, 0, PortDirection::Input),
    ("cfg_mean_residual_limit_q16", 15, 0, PortDirection::Input),
    ("cfg_max_mass_spread", 7, 0, PortDirection::Input),
    ("cfg_min_baseline_q4", 15, 0, PortDirection::Input),
    ("out_valid", 0, 0, PortDirection::Output),
    ("out_ready", 0, 0, PortDirection::Input),
    ("out_data", 31, 0, PortDirection::Output),
    ("out_last", 0, 0, PortDirection::Output),
    ("busy", 0, 0, PortDirection::Output),
    ("fifo_level", 5, 0, PortDirection::Output),
    ("candidate_start", 0, 0, PortDirection::Output),
    ("candidate_done", 0, 0, PortDirection::Output),
    ("candidate_cycles", 9, 0, PortDirection::Output),
    ("fifo_drop_sticky", 0, 0, PortDirection::Output),
    ("protocol_error_sticky", 0, 0, PortDirection::Output),
];

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn with_core(
    test: impl for<'ctx> FnOnce(&mut DynamicVerilatedModel<'ctx>) -> TestResult,
) -> TestResult {
    let root = gateware_root();
    let adapter = root.join(ADAPTER);
    rtl_test_support::with_model(
        &root,
        Path::new(MANIFEST),
        "lftag-direct-pose-core-contract-v1",
        "CoreLfTag",
        Path::new(ADAPTER),
        PORTS,
        &[Path::new("rtl/lftag")],
        &[adapter.as_path()],
        test,
    )
}

fn plane_code(plane: Plane) -> u8 {
    match plane {
        Plane::Raw => 0,
        Plane::Ll2 => 1,
        Plane::Ll4 => 2,
    }
}

#[derive(Debug)]
struct CapturedPacket {
    words: Vec<u32>,
    candidate_cycles: u16,
    actual_cycles: u16,
}

struct Driver<'model, 'ctx> {
    dut: &'model mut DynamicVerilatedModel<'ctx>,
    cycles: usize,
    active_start: Option<usize>,
    completed_cycles: VecDeque<(u16, u16)>,
    current_packet: Vec<u32>,
    packets: VecDeque<CapturedPacket>,
    candidate_start_cycles: Vec<usize>,
    stalled_word: Option<(u32, bool)>,
    output_ready: bool,
}

impl<'model, 'ctx> Driver<'model, 'ctx> {
    fn new(dut: &'model mut DynamicVerilatedModel<'ctx>) -> TestResult<Self> {
        let mut driver = Self {
            dut,
            cycles: 0,
            active_start: None,
            completed_cycles: VecDeque::new(),
            current_packet: Vec::new(),
            packets: VecDeque::new(),
            candidate_start_cycles: Vec::new(),
            stalled_word: None,
            output_ready: true,
        };
        driver.pin_configuration()?;
        driver.pin_idle_input()?;
        driver.set_out_ready(true)?;
        driver.dut.pin("clear_diagnostics", 0_u8)?;
        driver.reset()?;
        Ok(driver)
    }

    fn pin_configuration(&mut self) -> TestResult {
        self.dut.pin("cfg_inv_fx_q2_30", INV_F_Q2_30)?;
        self.dut.pin("cfg_inv_fy_q2_30", INV_F_Q2_30)?;
        self.dut.pin("cfg_cx_q16_16", CX_Q16_16)?;
        self.dut.pin("cfg_cy_q16_16", CY_Q16_16)?;
        self.dut
            .pin("cfg_printed_width_q16_16", PRINTED_WIDTH_Q16_16)?;
        self.dut
            .pin("cfg_mean_residual_limit_q16", MEAN_RESIDUAL_LIMIT_Q16)?;
        self.dut.pin("cfg_max_mass_spread", 19_u8)?;
        self.dut.pin("cfg_min_baseline_q4", 16_u16)?;
        Ok(())
    }

    fn pin_idle_input(&mut self) -> TestResult {
        self.dut.pin("in_valid", 0_u8)?;
        self.dut.pin("in_group_end", 0_u8)?;
        self.dut.pin("in_x_q4", 0_u16)?;
        self.dut.pin("in_y_q4", 0_u16)?;
        self.dut.pin("in_log_mass_q5_3", 0_u8)?;
        self.dut.pin("in_centroid_valid", 0_u8)?;
        self.dut.pin("in_scale", 0_u8)?;
        self.dut.pin("in_frame_seq", 0_u8)?;
        Ok(())
    }

    fn reset(&mut self) -> TestResult {
        self.pin_idle_input()?;
        self.set_out_ready(true)?;
        self.dut.pin("clear_diagnostics", 0_u8)?;
        self.dut.pin("rst", 1_u8)?;
        self.active_start = None;
        self.completed_cycles.clear();
        self.current_packet.clear();
        self.packets.clear();
        self.candidate_start_cycles.clear();
        self.stalled_word = None;
        for _ in 0..4 {
            clock(self.dut)?;
            self.cycles += 1;
        }
        self.dut.pin("rst", 0_u8)?;
        clock(self.dut)?;
        self.cycles += 1;
        self.dut.eval();
        assert_eq!(read_u64(self.dut, "out_valid")?, 0);
        assert_eq!(read_u64(self.dut, "busy")?, 0);
        assert_eq!(read_u64(self.dut, "fifo_level")?, 0);
        assert_eq!(read_u64(self.dut, "fifo_drop_sticky")?, 0);
        assert_eq!(read_u64(self.dut, "protocol_error_sticky")?, 0);
        Ok(())
    }

    /// Observe the ready/valid transfer at the upcoming rising edge, then
    /// clock the model and collect registered candidate instrumentation.
    fn tick(&mut self) -> TestResult {
        self.dut.eval();
        let valid = read_u64(self.dut, "out_valid")? != 0;
        let ready = self.output_ready;
        let data = read_u64(self.dut, "out_data")? as u32;
        let last = read_u64(self.dut, "out_last")? != 0;
        if valid && !ready {
            let visible = (data, last);
            if let Some(previous) = self.stalled_word {
                assert_eq!(
                    visible, previous,
                    "result payload changed under backpressure at cycle {}",
                    self.cycles,
                );
            } else {
                self.stalled_word = Some(visible);
            }
        } else if valid {
            self.stalled_word = None;
            self.current_packet.push(data);
            if last {
                let (candidate_cycles, actual_cycles) = self
                    .completed_cycles
                    .pop_front()
                    .expect("result packet has no candidate_done event");
                self.packets.push_back(CapturedPacket {
                    words: std::mem::take(&mut self.current_packet),
                    candidate_cycles,
                    actual_cycles,
                });
            }
        } else {
            self.stalled_word = None;
            assert!(!last, "out_last asserted without out_valid");
        }

        clock(self.dut)?;
        self.cycles += 1;
        if read_u64(self.dut, "candidate_start")? != 0 {
            self.candidate_start_cycles.push(self.cycles);
            assert!(
                self.active_start.replace(self.cycles).is_none(),
                "candidate_start overlapped an active candidate",
            );
        }
        if read_u64(self.dut, "candidate_done")? != 0 {
            let started = self
                .active_start
                .take()
                .expect("candidate_done occurred without candidate_start");
            let reported = read_u64(self.dut, "candidate_cycles")? as u16;
            let actual = u16::try_from(self.cycles - started)
                .expect("candidate cycle interval does not fit instrumentation");
            assert!(reported > 0, "completed candidate reported zero clocks");
            assert_eq!(
                reported, actual,
                "candidate_cycles disagrees with candidate_start/done interval",
            );
            assert!(
                self.cycles - started <= usize::from(ACCEPTED_CYCLE_WATCHDOG),
                "candidate occupied the core for {} clocks",
                self.cycles - started,
            );
            self.completed_cycles.push_back((reported, actual));
        }
        Ok(())
    }

    fn drive_centroid(
        &mut self,
        candidate: &Candidate,
        index: usize,
        group_end: bool,
        bubble_after: bool,
    ) -> TestResult {
        let centroid = candidate.centroids[index];
        self.drive_tuple(
            centroid,
            plane_code(candidate.plane),
            candidate.frame_sequence,
            group_end,
            bubble_after,
        )
    }

    fn drive_tuple(
        &mut self,
        centroid: WeightedCentroid,
        scale_code: u8,
        frame_sequence: u8,
        group_end: bool,
        bubble_after: bool,
    ) -> TestResult {
        let x_q4 = u16::try_from(centroid.x_q4)
            .map_err(|_| format!("negative/out-of-range x_q4: {}", centroid.x_q4))?;
        let y_q4 = u16::try_from(centroid.y_q4)
            .map_err(|_| format!("negative/out-of-range y_q4: {}", centroid.y_q4))?;
        assert!(x_q4 < (1 << 15));
        assert!(y_q4 < (1 << 14));
        self.dut.pin("in_valid", 1_u8)?;
        self.dut.pin("in_group_end", u8::from(group_end))?;
        self.dut.pin("in_x_q4", x_q4)?;
        self.dut.pin("in_y_q4", y_q4)?;
        self.dut.pin("in_log_mass_q5_3", centroid.log_mass_q5_3)?;
        self.dut
            .pin("in_centroid_valid", u8::from(centroid.valid))?;
        self.dut.pin("in_scale", scale_code)?;
        self.dut.pin("in_frame_seq", frame_sequence)?;
        self.tick()?;
        self.pin_idle_input()?;
        if bubble_after {
            self.tick()?;
        }
        Ok(())
    }

    fn drive_candidate(&mut self, candidate: &Candidate) -> TestResult {
        // Production ChildArena emits retained descriptors in ascending full
        // log-mass order. Keep the original index as a deterministic tie key;
        // the compact corner selector separately rejects an ambiguous tie at
        // the two-anchor cutoff.
        let mut indices = core::array::from_fn::<_, 9, _>(|index| index);
        indices.sort_by_key(|&index| (candidate.centroids[index].log_mass_q5_3, index));
        for (slot, index) in indices.into_iter().enumerate() {
            self.drive_centroid(candidate, index, slot == 8, true)?;
        }
        Ok(())
    }

    fn wait_packet(&mut self) -> TestResult<CapturedPacket> {
        for _ in 0..PACKET_WAIT_LIMIT {
            if let Some(packet) = self.packets.pop_front() {
                return Ok(packet);
            }
            self.tick()?;
        }
        Err(format!(
            "timed out waiting for LFTag packet after {} clocks",
            PACKET_WAIT_LIMIT
        )
        .into())
    }

    fn wait_out_valid(&mut self) -> TestResult {
        for _ in 0..PACKET_WAIT_LIMIT {
            self.dut.eval();
            if read_u64(self.dut, "out_valid")? != 0 {
                return Ok(());
            }
            self.tick()?;
        }
        Err("timed out waiting for out_valid".into())
    }

    fn clear_diagnostics(&mut self) -> TestResult {
        self.dut.pin("clear_diagnostics", 1_u8)?;
        self.tick()?;
        self.dut.pin("clear_diagnostics", 0_u8)?;
        self.dut.eval();
        assert_eq!(read_u64(self.dut, "fifo_drop_sticky")?, 0);
        assert_eq!(read_u64(self.dut, "protocol_error_sticky")?, 0);
        Ok(())
    }

    fn set_out_ready(&mut self, ready: bool) -> TestResult {
        self.output_ready = ready;
        self.dut.pin("out_ready", u8::from(ready))?;
        Ok(())
    }
}

fn split_header(header: [u64; 4]) -> [u32; 8] {
    core::array::from_fn(|index| {
        let source = header[index / 2];
        if index & 1 == 0 {
            source as u32
        } else {
            (source >> 32) as u32
        }
    })
}

fn quantized_word(value: f64, fractional_bits: u32) -> u32 {
    let (quantized, saturated) = quantize_signed(value, fractional_bits, 32);
    assert!(!saturated, "expected localization value saturated");
    quantized as i32 as u32
}

fn expected_extension(candidate: &Candidate) -> Vec<u32> {
    let inv_f = INV_F_Q2_30 as f64 / (1_u64 << 30) as f64;
    let calibration = CameraCalibration::new(
        1.0 / inv_f,
        1.0 / inv_f,
        CX_Q16_16 as f64 / 65536.0,
        CY_Q16_16 as f64 / 65536.0,
    );
    let printed_width = PRINTED_WIDTH_Q16_16 as f64 / 65536.0;
    let decoded = Decoder::default()
        .decode(candidate)
        .expect("accepted RTL vector must decode in Rust");
    let fixed = evaluate_transaction(&Decoder::default(), candidate);
    assert!(fixed.accepted, "accepted RTL vector must pass fixed decode");
    let localized = localize_wavelet(&decoded, calibration, printed_width)
        .expect("accepted RTL vector must localize in Rust");
    let mut words = Vec::with_capacity(14);
    let status = u32::from(true)
        | (u32::from(localized.weak_perspective) << 1)
        | (u32::from(localized.conditioning_warning) << 2);
    words.push(status);
    words.push(fixed.normalized_mean_residual_q32);
    words.extend(
        localized
            .pose
            .rotation
            .iter()
            .flatten()
            .map(|&value| quantized_word(value, 29)),
    );
    words.extend(
        localized
            .pose
            .translation
            .iter()
            .map(|&value| quantized_word(value, 16)),
    );
    words
}

fn signed_difference(actual: u32, expected: u32) -> i64 {
    (i64::from(actual as i32) - i64::from(expected as i32)).abs()
}

fn joined_u64(words: &[u32], index: usize) -> u64 {
    u64::from(words[index]) | (u64::from(words[index + 1]) << 32)
}

fn assert_header_matches(packet: &[u32], expected: &[u32; 8], accepted: bool) {
    if !accepted {
        assert_eq!(packet.get(..8), Some(expected.as_slice()));
        return;
    }
    assert_eq!(&packet[..2], &expected[..2], "ID/metadata mismatch");
    assert_eq!(&packet[6..8], &expected[6..8], "reject/reserved mismatch");
    assert_eq!(
        &packet[4..6],
        &[0, 0],
        "reserved score-gap words must be zero"
    );
    let actual_score = joined_u64(packet, 2);
    let expected_score = joined_u64(expected, 2);
    assert!(
        actual_score.abs_diff(expected_score) <= INTEGRATED_SCORE_TOLERANCE_Q32,
        "best score differs: actual {actual_score:#018x}, expected {expected_score:#018x}",
    );
}

fn assert_packet_matches_decoder(packet: CapturedPacket, candidate: &Candidate, decoder: &Decoder) {
    let expected = evaluate_transaction(decoder, candidate);
    let expected_header = split_header(expected.serialize_header());
    assert_header_matches(&packet.words, &expected_header, expected.accepted);
    if !expected.accepted {
        assert_eq!(packet.words.len(), 8, "rejected packet length mismatch");
        return;
    }

    assert_eq!(packet.candidate_cycles, packet.actual_cycles);
    assert_eq!(
        packet.words.len(),
        POSE_PACKET_WORDS,
        "accepted packet length mismatch"
    );
    let expected_extension = expected_extension(candidate);
    assert_eq!(
        packet.words[8], expected_extension[0],
        "status/version mismatch"
    );
    let packet_mean_residual = expected.normalized_mean_residual_q32;
    assert_eq!(
        packet.words[9], packet_mean_residual,
        "packet mean decode residual is not the normalized rational error",
    );
    let direct = PosePacket::from_packet(&packet.words)
        .expect("accepted packet must have a canonical direct-pose tail");
    assert_eq!(direct.mean_decode_residual_q32, packet_mean_residual);
    for (index, (actual, &expected)) in direct
        .pose
        .rotation_q29
        .iter()
        .map(|&value| value as u32)
        .zip(&expected_extension[2..11])
        .enumerate()
    {
        // The reciprocal-norm ROM covers both exponent parities with 256
        // bins each (relative scale error <=1/1024 plus one Q10 unit).
        // Include homography/column quantization; the cross-product column
        // compounds the errors of both input columns.
        let tolerance = if index % 3 == 2 { 1 << 21 } else { 1 << 20 };
        assert!(
            signed_difference(actual, expected) <= tolerance,
            "R[{index}] differs for frame {:#04x}/{:?}: actual {:#010x}, expected {:#010x}",
            candidate.frame_sequence,
            candidate.plane,
            actual,
            expected,
        );
    }
    for (index, (actual, &expected)) in direct
        .pose
        .translation_q16
        .iter()
        .map(|&value| value as u32)
        .zip(&expected_extension[11..14])
        .enumerate()
    {
        assert!(
            signed_difference(actual, expected) <= 64,
            "t[{index}] differs: actual {:#010x}, expected {:#010x}",
            actual,
            expected,
        );
    }
    assert!(
        packet.candidate_cycles <= ACCEPTED_COMPUTE_TARGET,
        "accepted candidate took {} clocks, exceeding the normal-path target",
        packet.candidate_cycles,
    );
}

fn assert_packet_matches_reference(packet: CapturedPacket, candidate: &Candidate) {
    assert_packet_matches_decoder(packet, candidate, &Decoder::default());
}

fn assert_accepted_header_and_timing(packet: CapturedPacket, candidate: &Candidate) {
    let expected = evaluate_transaction(&Decoder::default(), candidate);
    assert!(expected.accepted, "FIFO fixture must decode");
    assert_header_matches(
        &packet.words,
        &split_header(expected.serialize_header()),
        true,
    );
    assert_eq!(packet.words.len(), POSE_PACKET_WORDS);
    assert_eq!(packet.candidate_cycles, packet.actual_cycles);
    assert!(packet.candidate_cycles <= ACCEPTED_COMPUTE_TARGET);
}

fn assert_transport_reject_packet(
    packet: CapturedPacket,
    reason: TransportRejectReason,
    scale_code: u8,
    frame_sequence: u8,
) {
    assert_eq!(packet.words.len(), 8, "transport reject must be compact");
    assert_eq!(packet.candidate_cycles, packet.actual_cycles);
    assert_eq!(packet.words[0], 0);
    assert_eq!(
        packet.words[1],
        (u32::from(scale_code) << 28) | (u32::from(frame_sequence) << 20),
        "transport reject did not retain first-tuple metadata",
    );
    assert_eq!(&packet.words[2..6], &[0; 4]);
    assert_eq!(packet.words[6], u32::from(reason.code()));
    assert_eq!(
        parse_transport_reject_code(packet.words[6] as u8),
        Ok(Some(reason)),
    );
    assert_eq!(packet.words[7], 0);
}

fn rotate_clockwise(mut candidate: Candidate) -> Candidate {
    // Rotation about the exact Q4 center (320, 240).
    for centroid in &mut candidate.centroids {
        let old_x = centroid.x_q4;
        centroid.x_q4 = 8_960 - centroid.y_q4;
        centroid.y_q4 = old_x - 1_280;
    }
    candidate
}

fn permute_candidate(mut candidate: Candidate, permutation: [usize; 9]) -> Candidate {
    let original = candidate.centroids;
    candidate.centroids = permutation.map(|index| original[index]);
    candidate
}

fn projective_noisy_candidate(id: u16) -> Candidate {
    let transform = Mat3::new([
        [52.0, 4.0, 320.0],
        [-3.0, 48.0, 240.0],
        [0.0012, -0.0008, 1.0],
    ]);
    let canonical = canonical_points(id);
    let mut centroids: [WeightedCentroid; 9] = core::array::from_fn(|index| {
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
    // Move only the five non-key cells. This leaves the initial four-key
    // homography unchanged while producing a non-zero decode residual.
    centroids[2].x_q4 += 32;
    centroids[3].y_q4 += 24;
    centroids[4].x_q4 -= 20;
    centroids[5].y_q4 -= 28;
    centroids[7].x_q4 += 16;
    Candidate {
        centroids,
        plane: Plane::Raw,
        frame_sequence: 0x81,
    }
}

fn invalid_quadrilateral_candidate() -> Candidate {
    let mut candidate = rtl_smoke_candidate(0x1234);
    for (offset, centroid) in candidate.centroids[2..].iter_mut().enumerate() {
        centroid.y_q4 = 3_073 + (offset as i32 & 1);
    }
    candidate
}

fn collinear_candidate() -> Candidate {
    let mut candidate = rtl_smoke_candidate(0x1234);
    let ys = [3_071, 3_088, 3_087, 3_088, 3_087, 3_088, 3_087];
    for (centroid, y_q4) in candidate.centroids[2..].iter_mut().zip(ys) {
        centroid.y_q4 = y_q4;
    }
    candidate.centroids[2].x_q4 = 10_000;
    candidate
}

fn assert_vector_uses_calibrated_initial_homography(candidate: &Candidate) {
    let inv_f = INV_F_Q2_30 as f64 / (1_u64 << 30) as f64;
    let calibration = CameraCalibration::new(1.0 / inv_f, 1.0 / inv_f, 320.0, 240.0);
    let decoded = Decoder::default().decode(candidate).unwrap();
    let localized =
        localize_wavelet(&decoded, calibration, PRINTED_WIDTH_Q16_16 as f64 / 65536.0).unwrap();
    let scale = f64::from(decoded.plane.scale());
    let phase_y = if decoded.plane == Plane::Ll4 {
        4.0
    } else {
        0.0
    };
    let logical_to_source_matrix =
        Mat3::new([[scale, 0.0, 0.0], [0.0, scale, phase_y], [0.0, 0.0, 1.0]]);
    let camera_inverse = Mat3::new([
        [1.0 / calibration.fx, 0.0, -calibration.cx / calibration.fx],
        [0.0, 1.0 / calibration.fy, -calibration.cy / calibration.fy],
        [0.0, 0.0, 1.0],
    ]);
    let expected = (camera_inverse * logical_to_source_matrix * decoded.initial_homography)
        .normalized()
        .unwrap();
    for (&actual, &expected) in localized
        .homography
        .m
        .iter()
        .flatten()
        .zip(expected.m.iter().flatten())
    {
        assert!(
            (actual - expected).abs() < 1.0e-12,
            "{actual} != {expected}"
        );
    }
    assert_eq!(localized.mean_decode_residual, decoded.best_score / 5.0);

    // This fixture intentionally distinguishes the selected architecture from
    // an all-nine least-squares refit.
    let normalized_pairs = decoded.correspondences.map(|correspondence| {
        let source = wavelet_logical_to_source(correspondence.observed, decoded.plane);
        (
            correspondence.canonical,
            calibration.normalize(source).unwrap(),
        )
    });
    let all_point = fit_homography(&normalized_pairs).unwrap();
    let maximum_q16_difference = localized
        .homography
        .m
        .iter()
        .flatten()
        .zip(all_point.m.iter().flatten())
        .map(|(&initial_value, &all_point_value)| {
            let initial_q16 = quantized_word(initial_value, 16);
            let all_point_q16 = quantized_word(all_point_value, 16);
            signed_difference(initial_q16, all_point_q16)
        })
        .max()
        .unwrap();
    assert!(
        maximum_q16_difference > 64,
        "test vector does not distinguish the initial and all-nine fits",
    );
}

#[test]
fn noisy_projective_fixture_uses_calibrated_initial_homography() {
    assert_vector_uses_calibrated_initial_homography(&projective_noisy_candidate(0x2a55));
}

#[test]
fn exact_q4_ids_and_rotation_match_the_rust_transaction() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;
        for (frame, id) in [0_u16, 0x1234, 0x3fff].into_iter().enumerate() {
            let mut candidate = rtl_smoke_candidate(id);
            candidate.frame_sequence = frame as u8;
            driver.drive_candidate(&candidate)?;
            let packet = driver.wait_packet()?;
            assert_packet_matches_reference(packet, &candidate);
        }

        let mut rotated = rotate_clockwise(rtl_smoke_candidate(0x1234));
        rotated.frame_sequence = 0x5a;
        driver.drive_candidate(&rotated)?;
        let packet = driver.wait_packet()?;
        assert_packet_matches_reference(packet, &rotated);
        assert_eq!(read_u64(driver.dut, "protocol_error_sticky")?, 0);
        assert_eq!(read_u64(driver.dut, "fifo_drop_sticky")?, 0);
        Ok(())
    })
}

#[test]
fn compact_mean_threshold_changes_acceptance_without_changing_quality() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;
        let candidate = projective_noisy_candidate(0x2a55);
        let mut decoder = Decoder::default();
        let baseline = evaluate_transaction(&decoder, &candidate);
        assert!(baseline.accepted);
        let floor_q16 = (baseline.normalized_mean_residual_q32 >> 16) as u16;
        assert!(floor_q16 > 0 && floor_q16 < MEAN_RESIDUAL_LIMIT_Q16);
        for limit in [0, floor_q16 - 1, floor_q16 + 1, 1024, u16::MAX] {
            decoder.config.mean_residual_limit_q16 = limit;
            let expected = evaluate_transaction(&decoder, &candidate);
            assert_eq!(expected.accepted, limit > floor_q16);
            if expected.accepted {
                assert_eq!(expected, baseline);
            }
            driver.dut.pin("cfg_mean_residual_limit_q16", limit)?;
            driver.drive_candidate(&candidate)?;
            assert_packet_matches_decoder(driver.wait_packet()?, &candidate, &decoder);
        }
        Ok(())
    })
}

fn accepted_plane_localization_matches(candidate: Candidate) -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;
        let expected = evaluate_transaction(&Decoder::default(), &candidate);
        assert!(
            expected.accepted,
            "plane {:?} fixture rejected in the fixed reference: {expected:?}",
            candidate.plane,
        );
        driver.drive_candidate(&candidate)?;
        assert_packet_matches_reference(driver.wait_packet()?, &candidate);
        Ok(())
    })
}

// The wavelet-plane centers and pitches map back to exactly the same
// source-image location and 48-pixel source pitch as the raw fixture. Running
// them independently ensures a failure at one scale cannot hide another.
#[test]
fn accepted_raw_localization_matches_the_rust_reference() -> TestResult {
    accepted_plane_localization_matches(ideal_q4_candidate(
        0x1357,
        Plane::Raw,
        Point2::new(320.0, 240.0),
        48.0,
        0x30,
    ))
}

#[test]
fn accepted_ll2_localization_and_phase_match_the_rust_reference() -> TestResult {
    accepted_plane_localization_matches(ideal_q4_candidate(
        0x2a55,
        Plane::Ll2,
        Point2::new(80.0, 60.0),
        12.0,
        0x31,
    ))
}

#[test]
fn accepted_ll4_localization_and_phase_match_the_rust_reference() -> TestResult {
    accepted_plane_localization_matches(ideal_q4_candidate(
        0x3abc,
        Plane::Ll4,
        Point2::new(20.0, 14.75),
        3.0,
        0x32,
    ))
}

#[test]
fn every_bottom_corner_hypothesis_matches_the_fixed_model() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;
        const BOTTOM_SYMBOL_MASK: u16 = (0b11 << 8) | (0b11 << 12);
        for hypothesis in 0..16_u8 {
            let id = (0x2a55 & !BOTTOM_SYMBOL_MASK)
                | (u16::from(hypothesis & 0b11) << 8)
                | (u16::from(hypothesis >> 2) << 12);
            let mut candidate = rtl_smoke_candidate(id);
            candidate.frame_sequence = hypothesis;
            let expected = evaluate_transaction(&Decoder::default(), &candidate);
            assert!(expected.accepted);
            assert_eq!(
                expected.hypothesis, hypothesis,
                "fixture does not select hypothesis {hypothesis}",
            );
            driver.drive_candidate(&candidate)?;
            assert_packet_matches_reference(driver.wait_packet()?, &candidate);
        }
        Ok(())
    })
}

#[test]
fn shuffled_corner_and_noisy_projective_fit_match_the_rust_reference() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;
        // Canonical bottom-left index 6 intentionally arrives as tuple 8.
        let shuffled = permute_candidate(rtl_smoke_candidate(0x1555), [0, 2, 3, 4, 5, 7, 8, 1, 6]);
        driver.drive_candidate(&shuffled)?;
        assert_packet_matches_reference(driver.wait_packet()?, &shuffled);

        let noisy = projective_noisy_candidate(0x2a55);
        assert_vector_uses_calibrated_initial_homography(&noisy);
        driver.drive_candidate(&noisy)?;
        assert_packet_matches_reference(driver.wait_packet()?, &noisy);
        Ok(())
    })
}

#[test]
fn rejected_completion_reset_and_protocol_diagnostics_are_atomic() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;
        let mut invalid = rtl_smoke_candidate(0x1234);
        invalid.frame_sequence = 7;
        invalid.centroids[4].valid = false;
        driver.drive_candidate(&invalid)?;
        let packet = driver.wait_packet()?;
        assert_packet_matches_reference(packet, &invalid);

        // End an eight-tuple group. It is retained as a protocol rejection,
        // but reset it before completion; reset must not leak a packet.
        let partial = rtl_smoke_candidate(0x2aaa);
        for index in 0..8 {
            driver.drive_centroid(&partial, index, index == 7, true)?;
        }
        assert_eq!(read_u64(driver.dut, "protocol_error_sticky")?, 1);
        driver.clear_diagnostics()?;
        driver.reset()?;
        for _ in 0..16 {
            driver.tick()?;
        }
        assert!(
            driver.packets.is_empty(),
            "reset leaked a partial-group packet"
        );

        let candidate = rtl_smoke_candidate(0);
        driver.drive_candidate(&candidate)?;
        let packet = driver.wait_packet()?;
        assert_packet_matches_reference(packet, &candidate);
        Ok(())
    })
}

#[test]
fn malformed_groups_emit_typed_protocol_reject_packets() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;

        // Too few tuples.
        let mut short = rtl_smoke_candidate(0x0123);
        short.frame_sequence = 0x40;
        for index in 0..8 {
            driver.drive_centroid(&short, index, index == 7, true)?;
        }
        assert_transport_reject_packet(
            driver.wait_packet()?,
            TransportRejectReason::Protocol,
            0,
            short.frame_sequence,
        );

        // Too many tuples. The tenth tuple closes the group; only the first
        // nine descriptors may occupy its atomic FIFO slot.
        let mut long = rtl_smoke_candidate(0x0456);
        long.frame_sequence = 0x41;
        for index in 0..9 {
            driver.drive_centroid(&long, index, false, true)?;
        }
        driver.drive_centroid(&long, 0, true, true)?;
        assert_transport_reject_packet(
            driver.wait_packet()?,
            TransportRejectReason::Protocol,
            0,
            long.frame_sequence,
        );

        // Scale changes within a group.
        let mut mixed_scale = rtl_smoke_candidate(0x0789);
        mixed_scale.frame_sequence = 0x42;
        for index in 0..9 {
            let scale_code = if index == 4 { 1 } else { 0 };
            driver.drive_tuple(
                mixed_scale.centroids[index],
                scale_code,
                mixed_scale.frame_sequence,
                index == 8,
                true,
            )?;
        }
        assert_transport_reject_packet(
            driver.wait_packet()?,
            TransportRejectReason::Protocol,
            0,
            mixed_scale.frame_sequence,
        );

        // Frame metadata changes within a group. The result
        // header must retain the metadata of the first tuple.
        let mut mixed_metadata = rtl_smoke_candidate(0x0abc);
        mixed_metadata.frame_sequence = 0x43;
        for index in 0..9 {
            driver.drive_tuple(
                mixed_metadata.centroids[index],
                0,
                if index >= 5 {
                    mixed_metadata.frame_sequence.wrapping_add(1)
                } else {
                    mixed_metadata.frame_sequence
                },
                index == 8,
                true,
            )?;
        }
        assert_transport_reject_packet(
            driver.wait_packet()?,
            TransportRejectReason::Protocol,
            0,
            mixed_metadata.frame_sequence,
        );

        // Scale code three is reserved and is rejected even when consistent.
        let mut reserved_scale = rtl_smoke_candidate(0x0def);
        reserved_scale.frame_sequence = 0x44;
        for index in 0..9 {
            driver.drive_tuple(
                reserved_scale.centroids[index],
                3,
                reserved_scale.frame_sequence,
                index == 8,
                true,
            )?;
        }
        assert_transport_reject_packet(
            driver.wait_packet()?,
            TransportRejectReason::Protocol,
            3,
            reserved_scale.frame_sequence,
        );

        assert_eq!(read_u64(driver.dut, "protocol_error_sticky")?, 1);
        assert_eq!(read_u64(driver.dut, "fifo_drop_sticky")?, 0);
        driver.clear_diagnostics()?;
        Ok(())
    })
}

#[test]
fn directed_geometry_and_mass_rejects_match_rust_codes() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;

        let mut baseline_short = rtl_smoke_candidate(0x1234);
        baseline_short.centroids[1].x_q4 = baseline_short.centroids[0].x_q4;
        baseline_short.centroids[1].y_q4 = baseline_short.centroids[0].y_q4;
        let mut mass_spread = rtl_smoke_candidate(0x1234);
        mass_spread.centroids[2].log_mass_q5_3 = 0;
        let invalid_quad = invalid_quadrilateral_candidate();
        let collinear = collinear_candidate();

        for (candidate, oracle_reason, model_reason, production_reason) in [
            (
                baseline_short,
                RejectReason::BaselineTooShort,
                RejectReason::BaselineTooShort,
                RejectReason::BaselineTooShort,
            ),
            (
                mass_spread,
                RejectReason::MassSpread,
                RejectReason::MassSpread,
                RejectReason::MassSpread,
            ),
            (
                invalid_quad,
                RejectReason::InvalidQuadrilateral,
                RejectReason::NoValidHypothesis,
                RejectReason::NoValidHypothesis,
            ),
            (
                collinear,
                RejectReason::Collinear,
                RejectReason::NoValidHypothesis,
                RejectReason::NoValidHypothesis,
            ),
        ] {
            let decoder = Decoder::default();
            assert_eq!(decoder.decode(&candidate), Err(oracle_reason));
            let expected = evaluate_transaction(&decoder, &candidate);
            assert_eq!(expected.reject_reason, Some(model_reason));
            driver.drive_candidate(&candidate)?;
            let packet = driver.wait_packet()?;
            if model_reason == production_reason {
                assert_packet_matches_decoder(packet, &candidate, &decoder);
            } else {
                // Keep support for deliberately different hardware-policy
                // rejection points in directed guard fixtures.
                let mut expected_header = split_header(expected.serialize_header());
                expected_header[6] = u32::from(reject_code(production_reason));
                assert_eq!(packet.words, expected_header);
                assert_eq!(packet.candidate_cycles, packet.actual_cycles);
            }
        }

        Ok(())
    })
}

#[test]
fn invalid_pose_configuration_emits_a_typed_compact_rejection() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;
        let mut candidate = rtl_smoke_candidate(0x1234);
        candidate.frame_sequence = 0x56;

        // A zero printed width cannot produce a physical translation. The
        // decoder itself still succeeds, but the direct-pose packet contract
        // requires every nominally accepted packet to contain a valid pose.
        driver.dut.pin("cfg_printed_width_q16_16", 0_u32)?;
        driver.drive_candidate(&candidate)?;
        assert_transport_reject_packet(
            driver.wait_packet()?,
            TransportRejectReason::Decode(RejectReason::SingularProjectiveMap),
            0,
            candidate.frame_sequence,
        );
        Ok(())
    })
}

#[test]
fn result_packet_is_stable_under_backpressure() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;
        let candidate = rtl_smoke_candidate(0x3fff);
        driver.set_out_ready(false)?;
        driver.drive_candidate(&candidate)?;
        driver.wait_out_valid()?;
        for _ in 0..11 {
            driver.tick()?;
        }
        assert!(driver.packets.is_empty());
        driver.set_out_ready(true)?;
        let packet = driver.wait_packet()?;
        assert_packet_matches_reference(packet, &candidate);
        Ok(())
    })
}

#[test]
fn queued_accepted_candidates_meet_the_end_to_end_service_target() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;
        let first = rtl_smoke_candidate(0x1234);
        let mut second = rtl_smoke_candidate(0x2a55);
        second.frame_sequence = 1;

        driver.drive_candidate(&first)?;
        driver.drive_candidate(&second)?;
        let first_packet = driver.wait_packet()?;
        assert_eq!(
            first_packet.candidate_cycles, REPRESENTATIVE_COMPUTE_CYCLES,
            "the timing-reference candidate changed compute latency",
        );
        assert!(
            first_packet.candidate_cycles <= ACCEPTED_COMPUTE_TARGET,
            "representative direct-pose compute latency {} exceeded the {}-cycle target",
            first_packet.candidate_cycles,
            ACCEPTED_COMPUTE_TARGET,
        );
        assert_packet_matches_reference(first_packet, &first);
        assert_packet_matches_reference(driver.wait_packet()?, &second);

        assert_eq!(driver.candidate_start_cycles.len(), 2);
        let service_cycles = driver.candidate_start_cycles[1] - driver.candidate_start_cycles[0];
        assert_eq!(
            service_cycles, REPRESENTATIVE_SERVICE_CYCLES,
            "the timing-reference queue changed start-to-start service",
        );
        assert!(
            service_cycles <= usize::from(ACCEPTED_SERVICE_TARGET),
            "queued accepted candidates were serviced {service_cycles} clocks apart, exceeding the {ACCEPTED_SERVICE_TARGET}-cycle target",
        );
        Ok(())
    })
}

#[test]
fn full_fifo_retains_exact_group_order_and_drops_only_whole_new_groups() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;

        // Hold a completed result at the output so the decoder cannot remove
        // FIFO entries while the overload pattern is assembled.
        driver.set_out_ready(false)?;
        let mut active = rtl_smoke_candidate(0x3fff);
        active.frame_sequence = 0xee;
        driver.drive_candidate(&active)?;
        driver.wait_out_valid()?;
        assert_eq!(read_u64(driver.dut, "fifo_level")?, 0);

        let retained: [Candidate; 32] = core::array::from_fn(|index| {
            let mut candidate = rtl_smoke_candidate(index as u16);
            candidate.frame_sequence = index as u8;
            candidate
        });
        for (index, candidate) in retained.iter().enumerate() {
            driver.drive_candidate(candidate)?;
            assert_eq!(read_u64(driver.dut, "fifo_level")?, (index + 1) as u64,);
        }

        for frame in 32..40_u8 {
            let mut candidate = rtl_smoke_candidate(u16::from(frame));
            candidate.frame_sequence = frame;
            driver.drive_candidate(&candidate)?;
            assert_eq!(
                read_u64(driver.dut, "fifo_level")?,
                32,
                "an overloaded group changed FIFO occupancy",
            );
        }
        assert_eq!(
            read_u64(driver.dut, "fifo_drop_sticky")?,
            1,
            "overloaded candidate FIFO did not report an atomic drop",
        );
        assert_eq!(read_u64(driver.dut, "protocol_error_sticky")?, 0);

        driver.set_out_ready(true)?;
        assert_accepted_header_and_timing(driver.wait_packet()?, &active);
        for candidate in &retained {
            assert_accepted_header_and_timing(driver.wait_packet()?, candidate);
        }

        // There must be no packet from frames 32..39 and no partially
        // retained descriptor group after all thirty-two admitted slots drain.
        for _ in 0..32 {
            driver.tick()?;
        }
        assert!(driver.packets.is_empty());
        assert_eq!(read_u64(driver.dut, "fifo_level")?, 0);
        assert_eq!(read_u64(driver.dut, "busy")?, 0);
        assert_eq!(driver.candidate_start_cycles.len(), 33);
        Ok(())
    })
}
