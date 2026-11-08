//! End-to-end Marlin/Verilator checks for the production hierarchical 2PPC
//! core, including child ordering, centroid payloads, and group terminators.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use lrcca::{PreframedHierarchicalTwoPPCReferenceModel, PreframedHierarchyConfig};
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, read_u64, with_model};

const WIDTH: usize = 48;
const HEIGHT: usize = 36;
const TRAILER_CYCLES: usize = 4096;

const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("datavalid", 0, 0, PortDirection::Input),
    ("in_ready", 0, 0, PortDirection::Output),
    ("d0", 0, 0, PortDirection::Input),
    ("d1", 0, 0, PortDirection::Input),
    ("raw_d0", 7, 0, PortDirection::Input),
    ("raw_d1", 7, 0, PortDirection::Input),
    ("feature_valid_d0", 0, 0, PortDirection::Input),
    ("feature_valid_d1", 0, 0, PortDirection::Input),
    ("feature_y", 5, 0, PortDirection::Input),
    ("sof", 0, 0, PortDirection::Input),
    ("eol", 0, 0, PortDirection::Input),
    ("inner_fv_valid", 0, 0, PortDirection::Output),
    ("inner_fv_data", 28, 0, PortDirection::Output),
    ("inner_end", 0, 0, PortDirection::Output),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Feature {
    child_sort_key: u8,
    centroid_valid: bool,
    x_q5: u32,
    y_q5: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Group {
    features: Vec<Feature>,
}

#[derive(Default)]
struct Capture {
    groups: Vec<Group>,
    pending: Option<Group>,
}

#[derive(Clone)]
struct Case {
    name: &'static str,
    raw: Vec<Vec<u8>>,
    mask: Vec<Vec<u8>>,
}

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn bit_width(value: usize) -> usize {
    let cursor = value.saturating_sub(1);
    (usize::BITS - cursor.leading_zeros()).max(1) as usize
}

fn blank(value: u8) -> Vec<Vec<u8>> {
    vec![vec![value; WIDTH]; HEIGHT]
}

fn draw_rect_border(rows: &mut [Vec<u8>], x0: usize, x1: usize, y0: usize, y1: usize) {
    rows[y0][x0..=x1].fill(1);
    rows[y1][x0..=x1].fill(1);
    for row in rows.iter_mut().take(y1 + 1).skip(y0) {
        row[x0] = 1;
        row[x1] = 1;
    }
}

fn physical_frame(logical: &[Vec<u8>]) -> Vec<Vec<u8>> {
    assert!(!logical.is_empty());
    assert!(logical.len() + 6 <= HEIGHT);
    let logical_width = logical[0].len();
    assert!(logical_width + 6 <= WIDTH);
    assert!(logical.iter().all(|row| row.len() == logical_width));
    let mut result = blank(0);
    for (y, row) in logical.iter().enumerate() {
        result[y + 3][3..3 + logical_width].copy_from_slice(row);
    }
    result
}

fn valid_rows() -> Vec<Vec<u8>> {
    blank(1)
}

fn source_y_rows() -> Vec<Vec<usize>> {
    (0..HEIGHT).map(|y| vec![y; WIDTH]).collect()
}

fn expected_groups(raw: &[Vec<u8>], mask: &[Vec<u8>]) -> TestResult<Vec<Group>> {
    let config = PreframedHierarchyConfig {
        centroid_width: Some(WIDTH),
        centroid_height: Some(HEIGHT),
        use_coarse_child_key: false,
        use_full_log_mass_child_key: true,
        midpoint_reflection_reciprocal: false,
    };
    let model = PreframedHierarchicalTwoPPCReferenceModel::new(WIDTH, HEIGHT, config)
        .map_err(|error| format!("hierarchy reference setup failed: {error}"))?;
    let groups = model
        .feature_groups(raw, mask, &valid_rows(), &source_y_rows())
        .map_err(|error| format!("hierarchy reference failed: {error}"))?;
    Ok(groups
        .into_iter()
        .filter(|group| !group.inner_features.is_empty())
        .map(|group| Group {
            features: group
                .inner_features
                .into_iter()
                .map(|feature| Feature {
                    child_sort_key: feature.child_sort_key,
                    centroid_valid: feature.centroid_valid,
                    x_q5: feature.x_q5,
                    y_q5: feature.y_q5,
                })
                .collect(),
        })
        .collect())
}

fn idle(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("datavalid", 0_u8)?;
    model.pin("d0", 0_u8)?;
    model.pin("d1", 0_u8)?;
    model.pin("raw_d0", 0_u8)?;
    model.pin("raw_d1", 0_u8)?;
    model.pin("feature_valid_d0", 1_u8)?;
    model.pin("feature_valid_d1", 1_u8)?;
    model.pin("feature_y", 0_u8)?;
    model.pin("sof", 0_u8)?;
    model.pin("eol", 0_u8)?;
    Ok(())
}

fn collect(model: &DynamicVerilatedModel<'_>, capture: &mut Capture) -> TestResult {
    let valid = read_u64(model, "inner_fv_valid")? != 0;
    let end = read_u64(model, "inner_end")? != 0;
    if end && !valid {
        return Err("hierarchy asserted inner_end without inner_fv_valid".into());
    }
    if !valid {
        return Ok(());
    }

    if capture.pending.is_none() {
        capture.pending = Some(Group {
            features: Vec::new(),
        });
    }
    let group = capture.pending.as_mut().expect("group was just created");
    let mut value = read_u64(model, "inner_fv_data")?;
    let key = (value & 0xff) as u8;
    value >>= 8;
    let centroid_valid = value & 1 != 0;
    value >>= 1;
    let x_bits = bit_width(WIDTH << 4);
    let y_bits = bit_width(HEIGHT << 4);
    let x_q4 = (value & ((1_u64 << x_bits) - 1)) as u32;
    value >>= x_bits;
    let y_q4 = (value & ((1_u64 << y_bits) - 1)) as u32;
    group.features.push(Feature {
        child_sort_key: key,
        centroid_valid,
        x_q5: x_q4 << 1,
        y_q5: y_q4 << 1,
    });
    if end {
        capture
            .groups
            .push(capture.pending.take().expect("completed group must exist"));
    }
    Ok(())
}

fn tick(model: &mut DynamicVerilatedModel<'_>, capture: &mut Capture) -> TestResult {
    model.pin("clk", 0_u8)?;
    model.eval();
    model.pin("clk", 1_u8)?;
    model.eval();
    collect(model, capture)?;
    model.pin("clk", 0_u8)?;
    model.eval();
    Ok(())
}

fn run_frame(
    model: &mut DynamicVerilatedModel<'_>,
    raw: &[Vec<u8>],
    mask: &[Vec<u8>],
    gap_cycles: usize,
) -> TestResult<Vec<Group>> {
    run_frame_inner(model, raw, mask, gap_cycles, true)
}

fn run_frame_inner(
    model: &mut DynamicVerilatedModel<'_>,
    raw: &[Vec<u8>],
    mask: &[Vec<u8>],
    gap_cycles: usize,
    reset: bool,
) -> TestResult<Vec<Group>> {
    assert_eq!(raw.len(), HEIGHT);
    assert_eq!(mask.len(), HEIGHT);
    assert!(raw.iter().all(|row| row.len() == WIDTH));
    assert!(mask.iter().all(|row| row.len() == WIDTH));

    let mut capture = Capture::default();
    if reset {
        model.pin("rst", 1_u8)?;
        idle(model)?;
        for _ in 0..3 {
            tick(model, &mut capture)?;
        }
    }
    model.pin("rst", 0_u8)?;

    for y in 0..HEIGHT {
        for pair in 0..WIDTH / 2 {
            while read_u64(model, "in_ready")? == 0 {
                idle(model)?;
                tick(model, &mut capture)?;
            }
            let x = pair * 2;
            model.pin("datavalid", 1_u8)?;
            model.pin("d0", mask[y][x])?;
            model.pin("d1", mask[y][x + 1])?;
            model.pin("raw_d0", raw[y][x])?;
            model.pin("raw_d1", raw[y][x + 1])?;
            model.pin("feature_valid_d0", 1_u8)?;
            model.pin("feature_valid_d1", 1_u8)?;
            model.pin("feature_y", y as u8)?;
            model.pin("sof", u8::from(y == 0 && pair == 0))?;
            model.pin("eol", u8::from(pair + 1 == WIDTH / 2))?;
            tick(model, &mut capture)?;
            // usize::MAX selects a nonperiodic mixture, including long gaps
            // and consecutive pairs. Gaps after EOL also exercise the flush.
            let gap = if gap_cycles == usize::MAX {
                [0, 1, 7, 2, 0, 3][(y * (WIDTH / 2) + pair) % 6]
            } else {
                gap_cycles
            };
            for _ in 0..gap {
                idle(model)?;
                tick(model, &mut capture)?;
            }
        }
    }

    idle(model)?;
    for _ in 0..TRAILER_CYCLES {
        tick(model, &mut capture)?;
    }
    if capture.pending.is_some() {
        return Err("hierarchy did not terminate its final inner-feature group".into());
    }
    Ok(capture.groups)
}

fn directed_cases() -> Vec<Case> {
    let mut ring = blank(0);
    draw_rect_border(&mut ring, 5, 12, 4, 11);

    let mut ring_with_island = blank(0);
    draw_rect_border(&mut ring_with_island, 6, 16, 5, 15);
    ring_with_island[10][11] = 1;

    let mut open_to_exterior = blank(0);
    draw_rect_border(&mut open_to_exterior, 4, 11, 4, 11);
    open_to_exterior[7][11] = 0;

    let mut two_holes = blank(0);
    for row in two_holes.iter_mut().take(15).skip(6) {
        row[20..34].fill(1);
    }
    for row in two_holes.iter_mut().take(10).skip(8) {
        row[23..25].fill(0);
    }
    for row in two_holes.iter_mut().take(13).skip(11) {
        row[29..31].fill(0);
    }

    let mut many_holes = blank(0);
    for row in many_holes.iter_mut().take(24).skip(5) {
        row[5..43].fill(1);
    }
    for (x, y) in [
        (9, 9),
        (13, 9),
        (17, 9),
        (21, 9),
        (25, 9),
        (29, 9),
        (33, 9),
        (9, 13),
        (13, 13),
        (17, 13),
        (21, 13),
    ] {
        many_holes[y][x] = 0;
    }

    [
        ("ring", ring),
        ("ring-with-island", ring_with_island),
        ("open-to-exterior", open_to_exterior),
        ("two-holes", two_holes),
        ("many-holes", many_holes),
    ]
    .into_iter()
    .map(|(name, mask)| Case {
        name,
        raw: mask.clone(),
        mask,
    })
    .collect()
}

fn saturated_weighted_pair() -> (Case, Case) {
    let mut mask = vec![vec![0; 42]; 30];
    for row in mask.iter_mut().take(19).skip(5) {
        row[6..25].fill(1);
    }
    for row in mask.iter_mut().take(14).skip(9) {
        row[11..18].fill(0);
    }
    for x in [8, 10, 12, 14, 16, 18, 20, 22] {
        mask[7][x] = 0;
    }

    let mut raw_left = vec![vec![1; 42]; 30];
    let mut raw_right = raw_left.clone();
    for y in 9..14 {
        raw_left[y][11] = 255;
        raw_right[y][17] = 255;
    }
    (
        Case {
            name: "saturated-weight-left",
            raw: physical_frame(&raw_left),
            mask: physical_frame(&mask),
        },
        Case {
            name: "saturated-weight-right",
            raw: physical_frame(&raw_right),
            mask: physical_frame(&mask),
        },
    )
}

fn hazard_cases() -> Vec<Case> {
    let (base, _) = saturated_weighted_pair();
    let mut logical_mask = base.mask[3..33]
        .iter()
        .map(|row| row[3..45].to_vec())
        .collect::<Vec<_>>();
    for row in logical_mask.iter_mut().take(14).skip(9) {
        row[11..18].fill(1);
    }
    for x in [12, 14, 16] {
        logical_mask[9][x] = 0;
    }
    for row in logical_mask.iter_mut().take(12).skip(10) {
        row[12..17].fill(0);
    }
    let raw_two_lane = (0..30)
        .map(|y| {
            (0..42)
                .map(|x| ((x * 47 + y * 61 + 3) & 0xff) as u8)
                .collect()
        })
        .collect::<Vec<Vec<u8>>>();

    let mut active_gap = base.mask[3..33]
        .iter()
        .map(|row| row[3..45].to_vec())
        .collect::<Vec<_>>();
    for row in active_gap.iter_mut().take(14).skip(9) {
        row[11..18].fill(1);
    }
    active_gap[9][11..18].fill(0);
    active_gap[11][11..18].fill(0);
    active_gap[12][11..18].fill(0);
    active_gap[10][11..13].fill(0);
    active_gap[10][16..18].fill(0);
    let raw_active_gap = (0..30)
        .map(|y| {
            (0..42)
                .map(|x| {
                    if (x + 3 * y) % 11 == 0 {
                        255
                    } else {
                        ((x + 17 * y) % 31) as u8
                    }
                })
                .collect()
        })
        .collect::<Vec<Vec<u8>>>();

    let mut bank_reuse = vec![vec![0; 42]; 30];
    for row in bank_reuse.iter_mut().take(26).skip(4) {
        row[4..38].fill(1);
    }
    for y in (7..20).step_by(2) {
        for x in ((8 + (y & 2))..32).step_by(4) {
            bank_reuse[y][x] = 0;
            bank_reuse[y + 1][x] = 0;
            bank_reuse[y + 1][x + 1] = 0;
        }
    }
    let raw_bank_reuse = (0..30)
        .map(|y| {
            (0..42)
                .map(|x| (((x * x + 19 * y + 7) % 253) + 1) as u8)
                .collect()
        })
        .collect::<Vec<Vec<u8>>>();

    vec![
        Case {
            name: "two-lane-previous-merges",
            raw: physical_frame(&raw_two_lane),
            mask: physical_frame(&logical_mask),
        },
        Case {
            name: "previous-token-active-gap",
            raw: physical_frame(&raw_active_gap),
            mask: physical_frame(&active_gap),
        },
        Case {
            name: "close-collision-bank-reuse",
            raw: physical_frame(&raw_bank_reuse),
            mask: physical_frame(&bank_reuse),
        },
    ]
}

fn random_cases() -> Vec<Case> {
    let mut state = 0x0000_c0ca_u64;
    (0..4)
        .map(|index| {
            let mut mask = blank(0);
            let mut raw = blank(0);
            for y in 4..HEIGHT - 4 {
                for x in 4..WIDTH - 4 {
                    state ^= state << 13;
                    state ^= state >> 7;
                    state ^= state << 17;
                    mask[y][x] = u8::from(!state.is_multiple_of(13));
                    raw[y][x] = (state >> 16) as u8;
                }
            }
            Case {
                name: ["random-0", "random-1", "random-2", "random-3"][index],
                raw,
                mask,
            }
        })
        .collect()
}

fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
    let mut value = *state;
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

fn stress_case(seed: u64) -> (Vec<Vec<u8>>, Vec<Vec<u8>>) {
    let logical_width = WIDTH - 6;
    let logical_height = HEIGHT - 6;
    let mut raw_state = seed ^ 0x9e37_79b9_7f4a_7c15;
    let mut mask_state = seed ^ 0xbf58_476d_1ce4_e5b9;
    let logical_raw = (0..logical_height)
        .map(|_| {
            (0..logical_width)
                .map(|_| splitmix64(&mut raw_state) as u8)
                .collect()
        })
        .collect::<Vec<Vec<u8>>>();
    let mut logical_mask = vec![vec![0; logical_width]; logical_height];
    for row in logical_mask.iter_mut().take(logical_height - 1).skip(1) {
        row[1..logical_width - 1].fill(1);
    }
    for row in logical_mask.iter_mut().take(logical_height - 2).skip(2) {
        for pixel in row.iter_mut().take(logical_width - 2).skip(2) {
            if splitmix64(&mut mask_state).is_multiple_of(9) {
                *pixel = 0;
            }
        }
    }
    (physical_frame(&logical_raw), physical_frame(&logical_mask))
}

fn sort_group_features(groups: &mut [Group]) {
    for group in groups {
        group.features.sort_by_key(|feature| {
            (
                feature.child_sort_key,
                feature.centroid_valid,
                feature.x_q5,
                feature.y_q5,
            )
        });
    }
}

#[test]
fn hierarchical_rtl_matches_reference_payloads_groups_and_end_markers() -> TestResult {
    let (weighted_left, weighted_right) = saturated_weighted_pair();
    let zero_weight = Case {
        name: "saturated-zero-weight",
        raw: blank(0),
        mask: weighted_left.mask.clone(),
    };
    let expected_left = expected_groups(&weighted_left.raw, &weighted_left.mask)?;
    let expected_right = expected_groups(&weighted_right.raw, &weighted_right.mask)?;
    assert_eq!(expected_left.len(), 1);
    assert_eq!(expected_right.len(), 1);
    assert_eq!(expected_left[0].features.len(), 9);
    assert_ne!(
        expected_left, expected_right,
        "raw pixels must move centroids"
    );
    let expected_zero = expected_groups(&zero_weight.raw, &zero_weight.mask)?;
    assert_eq!(expected_zero[0].features.len(), 9);
    assert!(
        expected_zero[0]
            .features
            .iter()
            .all(|feature| { !feature.centroid_valid && feature.x_q5 == 0 && feature.y_q5 == 0 })
    );

    let mut cases = vec![weighted_left, weighted_right, zero_weight];
    cases.extend(hazard_cases());
    cases.extend(random_cases());
    cases.extend(directed_cases());

    let root = gateware_root();
    let adapter_rel = Path::new("tests/lrcca/tests/rtl_adapters/pipeline_hierarchical.sv");
    let adapter = root.join(adapter_rel);
    with_model(
        &root,
        Path::new("rtl/lrcca/sources.f"),
        "lrcca-pipeline-hierarchical-48x36",
        "PipelineHierarchical",
        adapter_rel,
        PORTS,
        &[],
        &[adapter.as_path()],
        |model| {
            for case in &cases {
                let mut expected = expected_groups(&case.raw, &case.mask)?;
                let mut actual = run_frame(model, &case.raw, &case.mask, 0)?;
                // The original timed random stress checker treats each child
                // list as a multiset: equal mass keys can be reordered by
                // the hardware arena's merge history. Directed group tests
                // above and below retain exact stream-order comparison.
                if case.name.starts_with("random-") {
                    sort_group_features(&mut expected);
                    sort_group_features(&mut actual);
                }
                assert_eq!(
                    actual, expected,
                    "hierarchical RTL/reference mismatch for {}",
                    case.name
                );
            }
            Ok(())
        },
    )
}

#[test]
fn hierarchical_parent_alignment_survives_midrow_gaps_and_pending_reset() -> TestResult {
    let root = gateware_root();
    let adapter_rel = Path::new("tests/lrcca/tests/rtl_adapters/pipeline_hierarchical.sv");
    let adapter = root.join(adapter_rel);
    let (weighted_left, weighted_right) = saturated_weighted_pair();
    let mut cases = vec![weighted_left, weighted_right];
    cases.extend(hazard_cases());
    cases.extend(random_cases());
    with_model(
        &root,
        Path::new("rtl/lrcca/sources.f"),
        "lrcca-pipeline-hierarchical-48x36",
        "PipelineHierarchical",
        adapter_rel,
        PORTS,
        &[],
        &[adapter.as_path()],
        |model| {
            for gap in [1, 2, 7, usize::MAX] {
                for case in &cases {
                    // Reset while both CCAs hold a non-EOL pending input.
                    // run_frame then starts a complete, independent frame.
                    let mut discarded = Capture::default();
                    model.pin("rst", 1_u8)?;
                    idle(model)?;
                    tick(model, &mut discarded)?;
                    model.pin("rst", 0_u8)?;
                    model.pin("datavalid", 1_u8)?;
                    model.pin("sof", 1_u8)?;
                    tick(model, &mut discarded)?;
                    idle(model)?;
                    for _ in 0..3 {
                        tick(model, &mut discarded)?;
                    }
                    let mut expected = expected_groups(&case.raw, &case.mask)?;
                    let mut actual = run_frame(model, &case.raw, &case.mask, gap)?;
                    if case.name.starts_with("random-") {
                        sort_group_features(&mut expected);
                        sort_group_features(&mut actual);
                    }
                    assert_eq!(actual, expected, "{} with gap {gap}", case.name);
                }
            }
            Ok(())
        },
    )
}

#[test]
#[ignore = "30-second randomized hardware stress"]
fn hierarchical_rtl_30_second_stress() -> TestResult {
    let root = gateware_root();
    let adapter_rel = Path::new("tests/lrcca/tests/rtl_adapters/pipeline_hierarchical.sv");
    let adapter = root.join(adapter_rel);
    with_model(
        &root,
        Path::new("rtl/lrcca/sources.f"),
        "lrcca-pipeline-hierarchical-48x36",
        "PipelineHierarchical",
        adapter_rel,
        PORTS,
        &[],
        &[adapter.as_path()],
        |model| {
            let deadline = Instant::now() + Duration::from_secs(30);
            let mut seed = 0x0000_c0ca_u64;
            let mut frames = 0_usize;
            while Instant::now() < deadline {
                let (raw, mask) = stress_case(seed);
                let mut expected = expected_groups(&raw, &mask)?;
                let mut actual = run_frame(model, &raw, &mask, 0)?;
                sort_group_features(&mut expected);
                sort_group_features(&mut actual);
                assert_eq!(
                    actual, expected,
                    "hierarchical RTL/reference mismatch in stress frame {frames} (seed {seed:#x})"
                );
                frames += 1;
                seed = seed.wrapping_add(1);
            }
            assert!(frames > 0, "hierarchical stress did not complete a frame");
            Ok(())
        },
    )
}

/// Aborting a dirty multirow frame must clear control/valid state even when
/// invalid payload registers deliberately retain arbitrary previous contents.
#[test]
fn hierarchical_payload_reset_recovery_discards_dirty_frame_history() -> TestResult {
    let root = gateware_root();
    let adapter_rel = Path::new("tests/lrcca/tests/rtl_adapters/pipeline_hierarchical.sv");
    let adapter = root.join(adapter_rel);
    let (weighted_left, weighted_right) = saturated_weighted_pair();
    let mut cases = vec![weighted_left, weighted_right];
    cases.extend(hazard_cases());
    cases.extend(random_cases());
    with_model(
        &root,
        Path::new("rtl/lrcca/sources.f"),
        "lrcca-pipeline-hierarchical-48x36",
        "PipelineHierarchical",
        adapter_rel,
        PORTS,
        &[],
        &[adapter.as_path()],
        |model| {
            // Cross first-row, EOL, deep dirty RAM, pending ingress and all
            // downstream pipeline phases. Each reset is followed by a fully
            // independently checked frame with varying stalls.
            let mut reset_width_counts = [0_usize; 4];
            let mut prefix_reset_widths = [[false; 4]; 8];
            for trial in 0..48_usize {
                let dirty = &cases[(trial * 5 + 1) % cases.len()];
                let prefix = [1, 23, 24, 25, 47, 127, 431, 863][trial % 8];
                let mut discarded = Capture::default();
                model.pin("rst", 1_u8)?;
                idle(model)?;
                tick(model, &mut discarded)?;
                model.pin("rst", 0_u8)?;
                for pair_index in 0..prefix {
                    while read_u64(model, "in_ready")? == 0 {
                        idle(model)?;
                        tick(model, &mut discarded)?;
                    }
                    let y = pair_index / (WIDTH / 2);
                    let pair = pair_index % (WIDTH / 2);
                    let x = pair * 2;
                    model.pin("datavalid", 1_u8)?;
                    model.pin("d0", dirty.mask[y][x])?;
                    model.pin("d1", dirty.mask[y][x + 1])?;
                    model.pin("raw_d0", dirty.raw[y][x])?;
                    model.pin("raw_d1", dirty.raw[y][x + 1])?;
                    model.pin("feature_valid_d0", 1_u8)?;
                    model.pin("feature_valid_d1", 1_u8)?;
                    model.pin("feature_y", y as u8)?;
                    model.pin("sof", u8::from(pair_index == 0))?;
                    model.pin("eol", u8::from(pair + 1 == WIDTH / 2))?;
                    tick(model, &mut discarded)?;
                    if pair_index % 17 == 9 {
                        idle(model)?;
                        tick(model, &mut discarded)?;
                    }
                }
                idle(model)?;
                for _ in 0..(trial / 4) {
                    tick(model, &mut discarded)?;
                }
                // Data and feature flags deliberately remain active through
                // the abort edge; only rst makes this transaction invalid.
                model.pin("rst", 1_u8)?;
                model.pin("datavalid", 1_u8)?;
                model.pin("sof", 1_u8)?;
                model.pin("eol", 1_u8)?;
                model.pin("d0", 1_u8)?;
                model.pin("d1", 0_u8)?;
                model.pin("raw_d0", 255_u8)?;
                model.pin("raw_d1", 1_u8)?;
                // Rotate the pulse width across each set of eight prefix
                // lengths: every prefix sees all widths within these 48 trials.
                let reset_width_index = (trial + trial / 8) % 4;
                let reset_cycles = [1, 3, 4, 20][reset_width_index];
                reset_width_counts[reset_width_index] += 1;
                prefix_reset_widths[trial % 8][reset_width_index] = true;
                for _ in 0..reset_cycles {
                    tick(model, &mut Capture::default())?;
                    assert_eq!(
                        read_u64(model, "inner_fv_valid")?,
                        0,
                        "stale output during reset trial {trial}"
                    );
                }
                let clean = &cases[trial % cases.len()];
                let mut expected = expected_groups(&clean.raw, &clean.mask)?;
                let gap = [0, 1, 7, usize::MAX][trial % 4];
                let mut actual = run_frame_inner(model, &clean.raw, &clean.mask, gap, false)?;
                if clean.name.starts_with("random-") {
                    sort_group_features(&mut expected);
                    sort_group_features(&mut actual);
                }
                assert_eq!(
                    actual, expected,
                    "dirty reset trial {trial}, prefix {prefix}, reset {reset_cycles}, clean {}",
                    clean.name
                );
            }
            assert_eq!(reset_width_counts, [12; 4]);
            assert!(prefix_reset_widths.iter().flatten().all(|covered| *covered));
            Ok(())
        },
    )
}
