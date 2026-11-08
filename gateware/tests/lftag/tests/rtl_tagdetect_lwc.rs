//! End-to-end parity for the combined TagDetect, LFTag, and LWC production top.
//!
//! Repeated asymmetric full-profile frames are
//! driven at the canonical 896-clock line cadence. The LWC payload is bit-exact
//! against `Codec::encode`, while
//! TagDetect's public centroid/end stream is compared with the wavelet-tap
//! reference model used by the shared frontend.

use std::path::{Path, PathBuf};

use lftag::{POSE_PACKET_WORDS, PosePacket};
use lrcca::{TagDetectCentroid, WaveletCompositeFrame};
use lwc::{Codec, FRAME_HEIGHT, FRAME_PIXELS, FRAME_WIDTH, raw_group_schedule};
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use ov9281_csi2_rx::{
    DATA_TYPE_FRAME_END, DATA_TYPE_FRAME_START, DATA_TYPE_RAW10, OV9281_VIRTUAL_CHANNEL,
    build_long_packet, build_short_packet, pack_raw10,
};
use rtl_test_support::{TestResult, read_u64, with_model};

#[path = "support/rtl_camera.rs"]
mod rtl_camera_support;

use rtl_camera_support::{CAMERA_PORTS, CameraSimulation};

const ACTIVE_PAIR_CLOCKS: usize = FRAME_WIDTH / 2;
const HORIZONTAL_BLANK_CLOCKS: usize = 256;
const LINE_CLOCKS: usize = ACTIVE_PAIR_CLOCKS + HORIZONTAL_BLANK_CLOCKS;
const OUTPUT_WORDS: usize = FRAME_PIXELS / 16;
const CENTROID_BITS: usize = 16;
const MAX_DRAIN_CYCLES: usize = 128 * LINE_CLOCKS;
const OUTPUT_QUIET_CYCLES: usize = LINE_CLOCKS;
const FNV1A_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV1A_PRIME: u64 = 0x0000_0100_0000_01b3;
const EXPECTED_COMPOSITE_PAIRS: usize = 684_336;
const EXPECTED_LWC_GROUPS: usize = 64_000;
const EXPECTED_LWC_GROUP_SCHEDULE_HASH: u64 = 0xf073_d448_1f89_3953;
const CAMERA_LINE_BYTE_CYCLES: usize = 1042;
const CAMERA_FRAME_BYTE_CYCLES: usize = 833_334;
const LFTAG_REJECT_PACKET_WORDS: usize = 8;
const COMPACT_PAIR_CAPACITIES: [i32; 12] =
    [64, 64, 64, 192, 192, 192, 176, 176, 176, 336, 336, 336];
const COMPACT_QUEUE_BANK_MASKS: [u16; 5] = [
    (1 << 6) | (1 << 9),
    (1 << 8) | (1 << 10),
    (1 << 7) | (1 << 11),
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4),
    (1 << 1) | (1 << 5),
];

const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("in_vref", 0, 0, PortDirection::Input),
    ("in_href", 0, 0, PortDirection::Input),
    ("in_d0", 7, 0, PortDirection::Input),
    ("in_d1", 7, 0, PortDirection::Input),
    ("inner_fv_valid", 0, 0, PortDirection::Output),
    ("inner_fv_data", 32, 0, PortDirection::Output),
    ("hier_inner_end", 0, 0, PortDirection::Output),
    ("lwc_out_sof", 0, 0, PortDirection::Output),
    ("lwc_out_valid", 0, 0, PortDirection::Output),
    ("lwc_out_data", 15, 0, PortDirection::Output),
    ("lwc_protocol_error", 0, 0, PortDirection::Output),
    ("pipeline_busy", 0, 0, PortDirection::Output),
    ("lftag_result_valid", 0, 0, PortDirection::Output),
    ("lftag_result_data", 31, 0, PortDirection::Output),
    ("lftag_result_last", 0, 0, PortDirection::Output),
    ("lftag_busy", 0, 0, PortDirection::Output),
    ("lftag_fifo_level", 5, 0, PortDirection::Output),
    ("lftag_candidate_start", 0, 0, PortDirection::Output),
    ("lftag_candidate_done", 0, 0, PortDirection::Output),
    ("lftag_candidate_cycles", 9, 0, PortDirection::Output),
    ("lftag_fifo_drop_sticky", 0, 0, PortDirection::Output),
    ("lftag_protocol_error_sticky", 0, 0, PortDirection::Output),
    ("threshold_protocol_error", 0, 0, PortDirection::Output),
    ("threshold_buffer_overrun", 0, 0, PortDirection::Output),
    ("threshold_frontend_overrun", 0, 0, PortDirection::Output),
    ("hierarchy_overrun", 0, 0, PortDirection::Output),
    ("hier_child_sort_key", 7, 0, PortDirection::Output),
    ("threshold_stream_hash", 63, 0, PortDirection::Output),
    ("threshold_stream_pairs", 31, 0, PortDirection::Output),
    ("outer_table_overflows", 31, 0, PortDirection::Output),
    ("inner_table_overflows", 31, 0, PortDirection::Output),
    ("tap_overflow", 0, 0, PortDirection::Output),
    ("lwc_group_schedule_hash", 63, 0, PortDirection::Output),
    ("lwc_group_schedule_count", 31, 0, PortDirection::Output),
    ("lwc_group_schedule_valid", 0, 0, PortDirection::Output),
    ("lwc_group_schedule_band", 3, 0, PortDirection::Output),
    ("lwc_raw_valid", 11, 0, PortDirection::Output),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PublicTagEvent {
    centroid: TagDetectCentroid,
    child_sort_key: u8,
    end_inner: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PublicTagFeature {
    centroid: TagDetectCentroid,
    child_sort_key: u8,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct PublicTagGroup {
    features: Vec<PublicTagFeature>,
}

struct ExpectedTagData {
    uncapped_groups: Vec<PublicTagGroup>,
    threshold_stream_hash: u64,
    threshold_stream_pairs: usize,
}

#[derive(Default)]
struct Capture {
    tag_events: Vec<PublicTagEvent>,
    tag_event_cycles: Vec<usize>,
    lwc_words: Vec<u16>,
    lwc_word_cycles: Vec<usize>,
    lwc_group_schedule: Vec<u8>,
    lwc_raw_half_full: u16,
    lwc_raw_band_coefficients: [usize; 12],
    lwc_pair_arrivals: Vec<(usize, usize)>,
    lwc_group_arrivals: Vec<(usize, usize)>,
    lftag_packets: Vec<Vec<u32>>,
    lftag_pending_packet: Vec<u32>,
    lftag_candidate_starts: usize,
    lftag_candidate_cycles: Vec<u16>,
    clocks: usize,
}

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn load_frame() -> Vec<u8> {
    (0..FRAME_PIXELS)
        .map(|index| {
            let x = index % FRAME_WIDTH;
            let y = index / FRAME_WIDTH;
            let horizon = FRAME_HEIGHT * 3 / 7;
            let mut value = if y < horizon {
                42 + (y * 92 / horizon) as i32
            } else {
                76 + ((y - horizon) * 88 / (FRAME_HEIGHT - horizon)) as i32
            };

            let sun_x = FRAME_WIDTH * 3 / 4;
            let sun_y = FRAME_HEIGHT / 5;
            let dx = x as i32 - sun_x as i32;
            let dy = y as i32 - sun_y as i32;
            if dx * dx + dy * dy < 52 * 52 {
                value += 78;
            }
            if y > horizon && x > FRAME_WIDTH / 7 && x < FRAME_WIDTH / 7 + 180 {
                value -= 35;
            }
            if y > horizon + 100 && ((x / 96) & 1) == 0 {
                value += 12;
            }

            let texture = (x as u32)
                .wrapping_mul(0x9e37_79b1)
                .rotate_left((y & 15) as u32)
                ^ (y as u32).wrapping_mul(0x85eb_ca6b);
            value += ((texture >> 29) as i32) - 3;
            value.clamp(0, 255) as u8
        })
        .collect()
}

fn fnv1a_word(mut hash: u64, word: u32) -> u64 {
    for byte in word.to_le_bytes() {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(FNV1A_PRIME);
    }
    hash
}

fn expected_threshold_stream(composite: &WaveletCompositeFrame) -> TestResult<(usize, u64)> {
    if !composite.width.is_multiple_of(2) {
        return Err("wavelet composite width must be even".into());
    }
    let mut pairs = 0;
    let mut hash = FNV1A_OFFSET;
    for y in 0..composite.height {
        for x in (0..composite.width).step_by(2) {
            let feature_valid0 = composite.feature_valid[y][x] != 0;
            let feature_valid1 = composite.feature_valid[y][x + 1] != 0;
            let replicated0 = composite.row_replicated[y][x] != 0;
            let replicated1 = composite.row_replicated[y][x + 1] != 0;
            if replicated0 != replicated1 {
                return Err(format!(
                    "reference composite pair ({x},{y}) crosses a row-replication boundary"
                )
                .into());
            }
            let source_y = match (feature_valid0, feature_valid1) {
                (true, true) => {
                    if composite.source_y[y][x] != composite.source_y[y][x + 1] {
                        return Err(format!(
                            "reference composite pair ({x},{y}) has two source rows"
                        )
                        .into());
                    }
                    composite.source_y[y][x]
                }
                (true, false) => composite.source_y[y][x],
                (false, true) => composite.source_y[y][x + 1],
                (false, false) => 0,
            };
            if source_y >= 1 << 10 {
                return Err(format!("reference source row {source_y} exceeds ten bits").into());
            }
            let d0 = 255_u8 - composite.raw[y][x];
            let d1 = 255_u8 - composite.raw[y][x + 1];
            let word = u32::from(d0)
                | (u32::from(d1) << 8)
                | (u32::from(composite.mask[y][x] != 0) << 16)
                | (u32::from(composite.mask[y][x + 1] != 0) << 17)
                | (u32::from(x + 2 == composite.width) << 18)
                | (u32::from(replicated0) << 19)
                | (u32::from(feature_valid0) << 20)
                | (u32::from(feature_valid1) << 21)
                | ((source_y as u32) << 22);
            hash = fnv1a_word(hash, word);
            pairs += 1;
        }
    }
    Ok((pairs, hash))
}

fn expected_tag_data(pixels: &[u8]) -> TestResult<ExpectedTagData> {
    let frame = lrcca::adaptive_threshold_composite(pixels, FRAME_WIDTH, FRAME_HEIGHT)?;
    let composite = WaveletCompositeFrame {
        width: frame.width,
        height: frame.height,
        raw: frame.raw,
        mask: frame.mask,
        feature_valid: frame.feature_valid,
        source_y: frame.source_y,
        row_replicated: frame.row_replicated,
    };
    let (threshold_stream_pairs, threshold_stream_hash) = expected_threshold_stream(&composite)?;
    let hierarchy = lrcca::hierarchical::PreframedHierarchicalTwoPPCReferenceModel::new(
        composite.width,
        composite.height,
        lrcca::hierarchical::PreframedHierarchyConfig {
            centroid_width: Some(composite.width),
            centroid_height: Some(FRAME_HEIGHT),
            use_coarse_child_key: false,
            use_full_log_mass_child_key: true,
            midpoint_reflection_reciprocal: true,
        },
    )?;
    let uncapped_groups = hierarchy
        .feature_candidate_groups(
            &composite.raw,
            &composite.mask,
            &composite.feature_valid,
            &composite.source_y,
        )?
        .into_iter()
        .map(|group| {
            let mut features = group
                .inner_features
                .into_iter()
                .map(|feature| PublicTagFeature {
                    centroid: TagDetectCentroid {
                        centroid_valid: feature.centroid_valid,
                        x_q5: feature.x_q5,
                        y_q5: feature.y_q5,
                    },
                    child_sort_key: feature.child_sort_key,
                })
                .collect::<Vec<_>>();
            features.sort_by_key(|feature| {
                (
                    feature.child_sort_key,
                    feature.centroid.centroid_valid,
                    feature.centroid.x_q5,
                    feature.centroid.y_q5,
                )
            });
            PublicTagGroup { features }
        })
        .collect();
    Ok(ExpectedTagData {
        uncapped_groups,
        threshold_stream_hash,
        threshold_stream_pairs,
    })
}

#[test]
fn adaptive_integrated_pipeline_matches_centroids_and_codec() -> TestResult {
    let root = gateware_root();
    let adapter_rel = Path::new("tests/lftag/tests/rtl_adapters/pipeline_tagdetect_lwc.sv");
    let adapter = root.join(adapter_rel);
    let mut frames = vec![("texture".to_string(), load_frame())];
    if let Ok(dataset) = std::env::var("LUTCAM_THRESHOLD_DATASET_DIR") {
        for index in [
            4, 6, 7, 9, 14, 17, 18, 19, 20, 29, 35, 36, 39, 46, 47, 55, 61, 64, 66, 67, 69, 73, 78,
            79, 85, 91, 95, 97, 99,
        ] {
            let pixels = image::open(Path::new(&dataset).join(format!("frame_{index:03}.png")))?
                .into_luma8();
            assert_eq!(
                pixels.dimensions(),
                (FRAME_WIDTH as u32, FRAME_HEIGHT as u32)
            );
            frames.push((format!("dataset-{index:03}"), pixels.into_raw()));
        }
    }
    with_model(
        &root,
        Path::new("rtl/tagdetect_lwc/sources.f"),
        "tagdetect-lwc-adaptive",
        "pipeline_tagdetect_lwc",
        adapter_rel,
        PORTS,
        &[
            Path::new("rtl/lrcca"),
            Path::new("rtl/lwc"),
            Path::new("rtl/lftag"),
        ],
        &[adapter.as_path()],
        |model| {
            for (name, pixels) in &frames {
                let expected_lwc = Codec::encode(pixels, FRAME_WIDTH, FRAME_HEIGHT)?;
                let expected = expected_tag_data(pixels)?;
                let events = expected
                    .uncapped_groups
                    .iter()
                    .filter(|group| group.features.len() >= 9)
                    .count()
                    * 9;
                let (capture, drained) = run_frame(model, pixels, events)?;
                let decoder = lftag::Decoder::default();
                for (index, (group, packet)) in capture
                    .tag_events
                    .chunks_exact(9)
                    .zip(&capture.lftag_packets)
                    .enumerate()
                {
                    // Keep the actual stream order, including equal-mass ties.
                    let centroids = std::array::from_fn(|i| lftag::WeightedCentroid {
                        x_q4: (group[i].centroid.x_q5 >> 1) as i32,
                        y_q4: (group[i].centroid.y_q5 >> 1) as i32,
                        log_mass_q5_3: group[i].child_sort_key,
                        valid: group[i].centroid.centroid_valid,
                    });
                    let result = lftag::fixed::candidate_from_composite(centroids, 0)
                        .ok()
                        .map(|candidate| lftag::fixed::evaluate_transaction(&decoder, &candidate));
                    let accepted = packet[1] & 0x8000_0000 != 0;
                    assert_eq!(
                        accepted,
                        result.is_some_and(|r| r.accepted),
                        "{name}: decoder acceptance differs for candidate {index}: {result:?}"
                    );
                    if accepted {
                        assert_eq!(
                            packet[0],
                            u32::from(result.unwrap().id),
                            "{name}: decoded ID differs for candidate {index}"
                        );
                    }
                }
                validate_combined_capture(
                    model,
                    capture,
                    drained,
                    expected_lwc.bytes(),
                    &expected,
                    &format!("adaptive-{name}"),
                )?;
            }
            Ok(())
        },
    )
}

fn grouped_tag_features(
    events: &[PublicTagEvent],
    stream_name: &str,
) -> TestResult<Vec<PublicTagGroup>> {
    let mut groups = Vec::new();
    let mut pending = Vec::new();
    for event in events {
        pending.push(PublicTagFeature {
            centroid: event.centroid,
            child_sort_key: event.child_sort_key,
        });
        if event.end_inner {
            pending.sort_by_key(|feature| {
                (
                    feature.child_sort_key,
                    feature.centroid.centroid_valid,
                    feature.centroid.x_q5,
                    feature.centroid.y_q5,
                )
            });
            groups.push(PublicTagGroup {
                features: std::mem::take(&mut pending),
            });
        }
    }
    if !pending.is_empty() {
        return Err(format!(
            "{stream_name} TagDetect stream left {} centroids unterminated",
            pending.len()
        )
        .into());
    }
    Ok(groups)
}

// Every retained tuple must belong to one uncapped connected component. Ties
// at the ninth-largest full mass key may choose different allocator IDs, but
// cannot replace a strictly heavier child or move a child across components.
fn is_valid_top_nine(actual: &PublicTagGroup, candidate: &PublicTagGroup) -> bool {
    if actual.features.len() != 9 || candidate.features.len() < 9 {
        return false;
    }
    let mut mass_keys = candidate
        .features
        .iter()
        .map(|feature| feature.child_sort_key)
        .collect::<Vec<_>>();
    mass_keys.sort_unstable();
    let cutoff = mass_keys[mass_keys.len() - 9];
    let mut remaining = candidate.features.clone();
    for feature in &actual.features {
        if feature.child_sort_key < cutoff {
            return false;
        }
        let Some(index) = remaining.iter().position(|candidate| candidate == feature) else {
            return false;
        };
        // Removing each occurrence also checks multiplicity when centroids
        // happen to coincide; one source tuple cannot be reused nine times.
        remaining.swap_remove(index);
    }
    !remaining
        .iter()
        .any(|feature| feature.child_sort_key > cutoff)
}

fn assign_retained_group(
    candidate: usize,
    eligible: &[Vec<usize>],
    seen: &mut [bool],
    actual_owners: &mut [Option<usize>],
) -> bool {
    for &actual in &eligible[candidate] {
        if seen[actual] {
            continue;
        }
        seen[actual] = true;
        if actual_owners[actual]
            .is_none_or(|previous| assign_retained_group(previous, eligible, seen, actual_owners))
        {
            actual_owners[actual] = Some(candidate);
            return true;
        }
    }
    false
}

fn tag_group_mismatch(
    actual_groups: &[PublicTagGroup],
    candidate_groups: &[PublicTagGroup],
    event_cycles: &[usize],
) -> Option<String> {
    let mut public_candidates = candidate_groups
        .iter()
        .enumerate()
        .filter(|(_, group)| group.features.len() >= 9)
        .collect::<Vec<_>>();
    if actual_groups.len() != public_candidates.len() {
        return Some(format!(
            "TagDetect group count mismatch: RTL={}, public candidate reference={}",
            actual_groups.len(),
            public_candidates.len()
        ));
    }
    for (index, group) in actual_groups.iter().enumerate() {
        if group.features.len() != 9 {
            let first_event_index = actual_groups[..index]
                .iter()
                .map(|group| group.features.len())
                .sum::<usize>();
            return Some(format!(
                "TagDetect group {index} at cycle {:?} emitted {} tuples instead of 9",
                event_cycles.get(first_event_index),
                group.features.len()
            ));
        }
    }

    // Prefer exact-nine errors for diagnostics. A complete bipartite matching
    // keeps identical/equal-cutoff candidates from making greedy matching
    // order-dependent, while requiring one distinct output per component.
    public_candidates.sort_by_key(|(_, group)| group.features.len() != 9);
    let eligible = public_candidates
        .iter()
        .map(|(_, candidate)| {
            actual_groups
                .iter()
                .enumerate()
                .filter_map(|(actual, group)| is_valid_top_nine(group, candidate).then_some(actual))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let mut actual_owners = vec![None; actual_groups.len()];
    for (position, &(candidate_index, candidate)) in public_candidates.iter().enumerate() {
        if !assign_retained_group(
            position,
            &eligible,
            &mut vec![false; actual_groups.len()],
            &mut actual_owners,
        ) {
            let kind = if candidate.features.len() == 9 {
                "exact nine-child"
            } else {
                "truncated"
            };
            return Some(format!(
                "TagDetect has no distinct valid top-nine retention for {kind} candidate group {candidate_index} ({} children); compatible RTL groups {:?}",
                candidate.features.len(),
                eligible[position]
            ));
        }
    }
    None
}

#[test]
fn candidate_matching_requires_exact_nine_child_groups_without_truncation_flags() {
    let make_group = |start: u32, count: u32| PublicTagGroup {
        features: (start..start + count)
            .map(|x_q5| PublicTagFeature {
                centroid: TagDetectCentroid {
                    centroid_valid: true,
                    x_q5,
                    y_q5: 1,
                },
                child_sort_key: 1,
            })
            .collect(),
    };
    let exact = make_group(1, 9);
    let truncated = make_group(20, 16);
    let retained = PublicTagGroup {
        features: truncated.features[..9].to_vec(),
    };
    let candidates = [exact.clone(), truncated];
    assert_eq!(
        tag_group_mismatch(&[retained.clone(), exact], &candidates, &[]),
        None
    );
    let missing_exact = tag_group_mismatch(&[retained.clone(), retained], &candidates, &[]);
    assert!(
        missing_exact
            .as_deref()
            .is_some_and(|message| message.contains("exact nine-child candidate group 0"))
    );
}

#[test]
fn candidate_matching_checks_truncated_source_coverage_and_tuple_provenance() {
    let make_group = |start: u32, count: u32| PublicTagGroup {
        features: (start..start + count)
            .map(|x_q5| PublicTagFeature {
                centroid: TagDetectCentroid {
                    centroid_valid: true,
                    x_q5,
                    y_q5: 1,
                },
                child_sort_key: 1,
            })
            .collect(),
    };
    let candidates = [make_group(1, 16), make_group(20, 16)];
    let retained = candidates.each_ref().map(|group| PublicTagGroup {
        features: group.features[..9].to_vec(),
    });
    assert_eq!(tag_group_mismatch(&retained, &candidates, &[]), None);
    let mut mixed = retained.clone();
    let first = mixed[0].features[0];
    mixed[0].features[0] = mixed[1].features[0];
    mixed[1].features[0] = first;
    assert!(
        tag_group_mismatch(&mixed, &candidates, &[]).is_some(),
        "global tuple provenance must not permit cross-component mixing"
    );
    let mut reused = retained.clone();
    reused[0].features[1] = reused[0].features[0];
    assert!(
        tag_group_mismatch(&reused, &candidates, &[]).is_some(),
        "one source child cannot satisfy two retained tuples"
    );
    let missing_source = tag_group_mismatch(
        &[retained[0].clone(), retained[0].clone()],
        &candidates,
        &[],
    );
    assert!(
        missing_source
            .as_deref()
            .is_some_and(|message| message.contains("truncated candidate group 1"))
    );
    let mut unknown_tuple = retained;
    unknown_tuple[1].features[8].centroid.x_q5 = 1000;
    assert!(
        tag_group_mismatch(&unknown_tuple, &candidates, &[])
            .as_deref()
            .is_some_and(|message| message.contains("no distinct valid top-nine retention"))
    );
}

#[test]
fn candidate_matching_allows_only_mass_cutoff_ties() {
    let candidate = PublicTagGroup {
        features: (0..12)
            .map(|x_q5| PublicTagFeature {
                centroid: TagDetectCentroid {
                    centroid_valid: true,
                    x_q5,
                    y_q5: 1,
                },
                child_sort_key: [1, 2, 3, 3, 3, 4, 5, 6, 7, 8, 9, 10][x_q5 as usize],
            })
            .collect(),
    };
    // Seven children strictly exceed cutoff3; either two of the three tied
    // children are valid, independently of their allocator-dependent order.
    let mut tied = candidate.features[2..4].to_vec();
    tied.extend_from_slice(&candidate.features[5..]);
    let actual = PublicTagGroup { features: tied };
    assert!(is_valid_top_nine(&actual, &candidate));
    let mut other_tie = actual.clone();
    other_tie.features[0] = candidate.features[4];
    assert!(is_valid_top_nine(&other_tie, &candidate));
    let mut too_light = actual.clone();
    too_light.features[0] = candidate.features[1];
    assert!(!is_valid_top_nine(&too_light, &candidate));
    let mut omitted_heavier = actual.clone();
    omitted_heavier.features[8] = candidate.features[4];
    assert!(!is_valid_top_nine(&omitted_heavier, &candidate));
}

fn drive_idle(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("in_vref", 0_u8)?;
    model.pin("in_href", 0_u8)?;
    model.pin("in_d0", 0_u8)?;
    model.pin("in_d1", 0_u8)?;
    Ok(())
}

fn collect(model: &DynamicVerilatedModel<'_>, capture: &mut Capture) -> TestResult {
    if read_u64(model, "threshold_protocol_error")? != 0 {
        return Err("wavelet adaptive-threshold frontend asserted protocol_error".into());
    }
    if read_u64(model, "threshold_frontend_overrun")? != 0 {
        return Err("wavelet adaptive-threshold frontend asserted buffer overrun".into());
    }
    if read_u64(model, "hierarchy_overrun")? != 0 {
        return Err("wavelet TagDetect hierarchy backpressured a valid threshold pair".into());
    }
    if read_u64(model, "threshold_buffer_overrun")? != 0 {
        return Err("wavelet TagDetect asserted an unclassified threshold overrun".into());
    }
    if read_u64(model, "tap_overflow")? != 0 {
        return Err("shared wavelet frontend asserted protocol_error".into());
    }
    if read_u64(model, "lwc_protocol_error")? != 0 {
        return Err("LWC asserted protocol_error".into());
    }
    if read_u64(model, "lftag_fifo_drop_sticky")? != 0 {
        return Err("LFTag candidate FIFO dropped a complete group".into());
    }
    if read_u64(model, "lftag_protocol_error_sticky")? != 0 {
        return Err("LFTag candidate assembler asserted protocol_error".into());
    }

    let feature_valid = read_u64(model, "inner_fv_valid")? != 0;
    let feature_data = read_u64(model, "inner_fv_data")?;
    let group_end = read_u64(model, "hier_inner_end")? != 0;
    let child_sort_key = u8::try_from(read_u64(model, "hier_child_sort_key")?)?;
    if feature_valid {
        let mask = (1_u64 << CENTROID_BITS) - 1;
        capture.tag_events.push(PublicTagEvent {
            centroid: TagDetectCentroid {
                centroid_valid: feature_data & 1 != 0,
                x_q5: ((feature_data >> 1) & mask) as u32,
                y_q5: ((feature_data >> (1 + CENTROID_BITS)) & mask) as u32,
            },
            child_sort_key,
            end_inner: group_end,
        });
        capture.tag_event_cycles.push(capture.clocks);
    } else {
        if feature_data & 1 != 0 {
            return Err("TagDetect centroid-valid bit asserted outside a tuple".into());
        }
        if group_end {
            return Err("TagDetect asserted a group terminator without a tuple".into());
        }
    }

    let lwc_valid = read_u64(model, "lwc_out_valid")? != 0;
    let lwc_sof = read_u64(model, "lwc_out_sof")? != 0;
    let lwc_data = read_u64(model, "lwc_out_data")?;
    if lwc_valid {
        if capture.lwc_words.len() == OUTPUT_WORDS {
            return Err("LWC emitted more than 64,000 output words".into());
        }
        if lwc_sof != capture.lwc_words.is_empty() {
            return Err(format!(
                "lwc_out_sof={} on output word {}",
                u8::from(lwc_sof),
                capture.lwc_words.len()
            )
            .into());
        }
        capture.lwc_words.push(u16::try_from(lwc_data)?);
        capture.lwc_word_cycles.push(capture.clocks);
    } else if lwc_sof {
        return Err("LWC asserted out_sof without out_valid".into());
    }
    if read_u64(model, "lwc_group_schedule_valid")? != 0 {
        capture
            .lwc_group_schedule
            .push(u8::try_from(read_u64(model, "lwc_group_schedule_band")?)?);
    }

    let raw_valid = u16::try_from(read_u64(model, "lwc_raw_valid")?)?;
    for (bank, mask) in COMPACT_QUEUE_BANK_MASKS.into_iter().enumerate() {
        if (raw_valid & mask).count_ones() > 1 {
            return Err(format!(
                "camera compact queue bank {bank} has conflicting writes at clock {}: raw_valid={raw_valid:#05x}",
                capture.clocks
            )
            .into());
        }
    }
    let pair_write = raw_valid & capture.lwc_raw_half_full;
    for band in 0..12 {
        let mask = 1_u16 << band;
        if pair_write & mask != 0 {
            capture.lwc_pair_arrivals.push((capture.clocks, band));
        }
        if raw_valid & mask != 0 {
            capture.lwc_raw_band_coefficients[band] += 1;
            if capture.lwc_raw_band_coefficients[band].is_multiple_of(16) {
                capture.lwc_group_arrivals.push((capture.clocks, band));
            }
        }
    }
    capture.lwc_raw_half_full ^= raw_valid;

    if read_u64(model, "lftag_candidate_start")? != 0 {
        capture.lftag_candidate_starts += 1;
    }
    if read_u64(model, "lftag_candidate_done")? != 0 {
        let cycles = u16::try_from(read_u64(model, "lftag_candidate_cycles")?)?;
        if cycles > 1_000 {
            return Err(format!("LFTag candidate exceeded the 1,000-cycle bound: {cycles}").into());
        }
        capture.lftag_candidate_cycles.push(cycles);
    }
    let lftag_valid = read_u64(model, "lftag_result_valid")? != 0;
    let lftag_last = read_u64(model, "lftag_result_last")? != 0;
    if lftag_valid {
        capture
            .lftag_pending_packet
            .push(u32::try_from(read_u64(model, "lftag_result_data")?)?);
        if lftag_last {
            capture
                .lftag_packets
                .push(std::mem::take(&mut capture.lftag_pending_packet));
        }
    } else if lftag_last {
        return Err("LFTag asserted out_last without out_valid".into());
    }
    Ok(())
}

fn verify_fixed_camera_queue_backlog(capture: &Capture) -> TestResult<[i32; 12]> {
    const SERVICE_CLOCKS: usize = 11;
    const RELEASE_CLOCKS: usize = 11;

    let mut arrivals_by_band: [Vec<usize>; 12] = std::array::from_fn(|_| Vec::new());
    for &(cycle, band) in &capture.lwc_group_arrivals {
        arrivals_by_band[band].push(cycle);
    }

    // The virtual reader is deliberately no faster than RTL.  A completed
    // group first becomes visible on the following clock.  Thereafter the
    // encoder control proof gives an eleven-clock read-group initiation
    // bound; schedule-ROM fetch overlaps the preceding eight-pair replay.
    let mut arrival_index = [0_usize; 12];
    let mut next_start = 0_usize;
    let mut group_releases = Vec::with_capacity(raw_group_schedule().len());
    for &band in raw_group_schedule() {
        let band = usize::from(band);
        let Some(&ready) = arrivals_by_band[band].get(arrival_index[band]) else {
            return Err(format!(
                "camera control trace omitted scheduled group {} for band {band}",
                arrival_index[band]
            )
            .into());
        };
        arrival_index[band] += 1;
        let start = next_start.max(ready + 1);
        // RTL advances the group pointer by start+10 at worst: a prefetched
        // pair zero may wait through three finalizer clocks, after which the
        // remaining seven reads issue consecutively.  Releasing all eight
        // virtual slots at start+11 therefore dominates the real queue.
        group_releases.push((start + RELEASE_CLOCKS, band));
        next_start = start + SERVICE_CLOCKS;
    }

    let mut events = Vec::with_capacity(capture.lwc_pair_arrivals.len() + group_releases.len());
    for &(cycle, band) in &capture.lwc_pair_arrivals {
        events.push((cycle, band, 1_i8));
    }
    for &(cycle, band) in &group_releases {
        events.push((cycle, band, -8_i8));
    }
    events.sort_unstable();

    let mut group_held = [0_i32; 12];
    let mut peak_group_held = [0_i32; 12];
    let mut event = 0;
    while event < events.len() {
        let cycle = events[event].0;
        let mut delta = [0_i32; 12];
        while event < events.len() && events[event].0 == cycle {
            delta[events[event].1] += i32::from(events[event].2);
            event += 1;
        }
        for band in 0..12 {
            group_held[band] += delta[band];
            if group_held[band] < 0 {
                return Err(format!(
                    "negative camera group-held occupancy for band {band} at cycle {cycle}"
                )
                .into());
            }
            peak_group_held[band] = peak_group_held[band].max(group_held[band]);
        }
    }

    if group_held != [0; 12] {
        return Err(format!("camera queue model did not drain: {group_held:?}").into());
    }
    for band in 0..12 {
        let capacity = COMPACT_PAIR_CAPACITIES[band];
        if peak_group_held[band] + 1 >= capacity {
            return Err(format!(
                "camera band {band} queue bound failed: {} + one live half !< {capacity}",
                peak_group_held[band]
            )
            .into());
        }
        if peak_group_held[band] >= capacity - 8 {
            return Err(format!(
                "camera band {band} group margin failed: {} !< {capacity} - 8",
                peak_group_held[band]
            )
            .into());
        }
    }
    Ok(peak_group_held)
}

fn tick(model: &mut DynamicVerilatedModel<'_>, capture: &mut Capture) -> TestResult {
    model.pin("clk", 0_u8)?;
    model.eval();
    model.pin("clk", 1_u8)?;
    model.eval();
    capture.clocks += 1;
    collect(model, capture)?;
    model.pin("clk", 0_u8)?;
    model.eval();
    Ok(())
}

fn run_frame(
    model: &mut DynamicVerilatedModel<'_>,
    pixels: &[u8],
    expected_tag_event_count: usize,
) -> TestResult<(Capture, usize)> {
    assert_eq!(LINE_CLOCKS, 896);
    assert_eq!(ACTIVE_PAIR_CLOCKS, 640);
    assert_eq!(pixels.len(), FRAME_PIXELS);

    let mut capture = Capture::default();
    model.pin("clk", 0_u8)?;
    model.pin("rst", 1_u8)?;
    drive_idle(model)?;
    model.eval();
    for _ in 0..4 {
        tick(model, &mut capture)?;
    }
    model.pin("rst", 0_u8)?;

    for row in 0..FRAME_HEIGHT {
        for pair in 0..ACTIVE_PAIR_CLOCKS {
            let index = row * FRAME_WIDTH + 2 * pair;
            model.pin("in_vref", 1_u8)?;
            model.pin("in_href", 1_u8)?;
            model.pin("in_d0", pixels[index])?;
            model.pin("in_d1", pixels[index + 1])?;
            tick(model, &mut capture)?;
        }
        model.pin("in_vref", 1_u8)?;
        model.pin("in_href", 0_u8)?;
        model.pin("in_d0", 0_u8)?;
        model.pin("in_d1", 0_u8)?;
        for _ in 0..HORIZONTAL_BLANK_CLOCKS {
            tick(model, &mut capture)?;
        }
    }

    drive_idle(model)?;
    let mut drained = 0;
    while (capture.tag_events.len() < expected_tag_event_count
        || capture.lwc_words.len() < OUTPUT_WORDS
        || read_u64(model, "threshold_stream_pairs")? < EXPECTED_COMPOSITE_PAIRS as u64
        || read_u64(model, "pipeline_busy")? != 0
        || read_u64(model, "lftag_busy")? != 0
        || read_u64(model, "lftag_fifo_level")? != 0)
        && drained < MAX_DRAIN_CYCLES
    {
        tick(model, &mut capture)?;
        drained += 1;
        if capture.tag_events.len() > expected_tag_event_count {
            return Err(format!(
                "TagDetect emitted more than the expected {expected_tag_event_count} events"
            )
            .into());
        }
    }
    if read_u64(model, "pipeline_busy")? != 0 {
        return Err("combined pipeline did not drain within the bound".into());
    }
    if capture.tag_events.len() != expected_tag_event_count {
        let threshold_pairs = read_u64(model, "threshold_stream_pairs")?;
        let threshold_hash = read_u64(model, "threshold_stream_hash")?;
        return Err(format!(
            "TagDetect emitted {} of {expected_tag_event_count} events after {drained} drain clocks; \
             threshold pairs/hash={threshold_pairs}/{threshold_hash:#018x}",
            capture.tag_events.len(),
        )
        .into());
    }
    if capture.lwc_words.len() != OUTPUT_WORDS {
        return Err(format!(
            "LWC emitted {} of {OUTPUT_WORDS} words after {drained} drain clocks",
            capture.lwc_words.len()
        )
        .into());
    }
    let completed_tag_events = capture.tag_events.len();
    let completed_lwc_words = capture.lwc_words.len();
    for _ in 0..OUTPUT_QUIET_CYCLES {
        tick(model, &mut capture)?;
    }
    if capture.tag_events.len() != completed_tag_events {
        return Err("TagDetect emitted another event after frame completion".into());
    }
    if capture.lwc_words.len() != completed_lwc_words {
        return Err("LWC emitted another word after frame completion".into());
    }
    Ok((capture, drained))
}

fn validate_lftag_capture(capture: &Capture, expected_candidates: usize) -> TestResult {
    if !capture.lftag_pending_packet.is_empty() {
        return Err("LFTag left a serialized result packet unterminated".into());
    }
    if capture.lftag_candidate_starts != expected_candidates
        || capture.lftag_candidate_cycles.len() != expected_candidates
        || capture.lftag_packets.len() != expected_candidates
    {
        return Err(format!(
            "LFTag transaction mismatch: starts={}, done={}, packets={}, TagDetect groups={expected_candidates}",
            capture.lftag_candidate_starts,
            capture.lftag_candidate_cycles.len(),
            capture.lftag_packets.len(),
        )
        .into());
    }
    for (index, &cycles) in capture.lftag_candidate_cycles.iter().enumerate() {
        if !(1..=1_000).contains(&cycles) {
            return Err(format!(
                "LFTag candidate {index} reported {cycles} processing cycles outside 1..=1000"
            )
            .into());
        }
    }
    for (index, packet) in capture.lftag_packets.iter().enumerate() {
        if packet.len() != LFTAG_REJECT_PACKET_WORDS && packet.len() != POSE_PACKET_WORDS {
            return Err(format!(
                "LFTag packet {index} has {} words instead of {} or {}",
                packet.len(),
                LFTAG_REJECT_PACKET_WORDS,
                POSE_PACKET_WORDS,
            )
            .into());
        }
        let accepted = packet[1] & 0x8000_0000 != 0;
        let reject_word = packet[6];
        let reject_code = reject_word & 0x1f;
        if packet[7] != 0 || reject_word != reject_code {
            return Err(format!(
                "LFTag packet {index} has malformed reject header words: {reject_word:#010x}, {:#010x}",
                packet[7]
            )
            .into());
        }
        match (accepted, packet.len(), reject_code) {
            (false, LFTAG_REJECT_PACKET_WORDS, 1..=31) => {}
            (true, POSE_PACKET_WORDS, 0) => {
                let status = packet[8];
                let reserved_status = status & 0xffff_fff0;
                if status & 1 == 0 || reserved_status != 0 {
                    return Err(format!(
                        "LFTag accepted packet {index} has malformed direct-pose status {status:#010x}",
                    )
                    .into());
                }
                PosePacket::from_packet(packet).map_err(|error| {
                    format!(
                        "LFTag accepted packet {index} has an invalid direct-pose payload: {error:?}"
                    )
                })?;
            }
            _ => {
                return Err(format!(
                    "LFTag packet {index} has inconsistent accepted={accepted}, length={}, reject_code={reject_code}",
                    packet.len()
                )
                .into());
            }
        }
    }
    Ok(())
}

fn representative_direct_pose_packet() -> Vec<u32> {
    vec![
        0x0000_0000,
        0x8000_0000,
        0x0000_0008,
        0x0000_0000,
        0x047b_ceae,
        0x0000_0000,
        0x0000_0000,
        0x0000_0000,
        3,
        0x0000_0001,
        0x2001_8000,
        0xfffd_a000,
        0x0000_0000,
        0x0000_0000,
        0x1fff_4000,
        0x0000_0000,
        0x0000_0000,
        0x0000_0000,
        0x2000_a000,
        0xffff_fffb,
        0xffff_fffb,
        0x0000_8007,
    ]
}

#[test]
fn integrated_lftag_validation_uses_the_direct_pose_packet_contract() {
    let mut capture = Capture {
        lftag_packets: vec![representative_direct_pose_packet()],
        lftag_candidate_starts: 1,
        lftag_candidate_cycles: vec![750],
        ..Capture::default()
    };
    validate_lftag_capture(&capture, 1)
        .expect("the current unversioned direct-pose packet must be accepted");

    capture.lftag_packets[0].truncate(POSE_PACKET_WORDS - 1);
    assert!(
        validate_lftag_capture(&capture, 1).is_err(),
        "the unified Ti60 regression must reject a truncated direct-pose packet",
    );
}

fn validate_combined_capture(
    model: &DynamicVerilatedModel<'_>,
    capture: Capture,
    drained: usize,
    expected_lwc: &[u8],
    expected_tag: &ExpectedTagData,
    metric_prefix: &str,
) -> TestResult {
    let actual_tag_groups = grouped_tag_features(&capture.tag_events, "RTL")?;
    validate_lftag_capture(&capture, actual_tag_groups.len())?;
    let tag_mismatch = tag_group_mismatch(
        &actual_tag_groups,
        &expected_tag.uncapped_groups,
        &capture.tag_event_cycles,
    );

    let actual_threshold_hash = read_u64(model, "threshold_stream_hash")?;
    let actual_threshold_pairs = usize::try_from(read_u64(model, "threshold_stream_pairs")?)?;
    let threshold_mismatch = if (actual_threshold_pairs, actual_threshold_hash)
        != (
            expected_tag.threshold_stream_pairs,
            expected_tag.threshold_stream_hash,
        ) {
        Some(format!(
            "adaptive-threshold composite mismatch: RTL pairs/hash={actual_threshold_pairs}/\
             {actual_threshold_hash:#018x}, reference={}/{:#018x}",
            expected_tag.threshold_stream_pairs, expected_tag.threshold_stream_hash
        ))
    } else {
        None
    };
    let outer_table_overflows = read_u64(model, "outer_table_overflows")?;
    let inner_table_overflows = read_u64(model, "inner_table_overflows")?;
    let table_depth_mismatch = if outer_table_overflows != 0 || inner_table_overflows != 0 {
        Some(format!(
            "CCA provisional labels exceeded an 848-entry outer or inner table: \
             outer={outer_table_overflows}, inner={inner_table_overflows}"
        ))
    } else {
        None
    };
    let actual_lwc_group_hash = read_u64(model, "lwc_group_schedule_hash")?;
    let actual_lwc_group_count = usize::try_from(read_u64(model, "lwc_group_schedule_count")?)?;
    let expected_group_schedule = raw_group_schedule();
    let first_group_schedule_mismatch = capture
        .lwc_group_schedule
        .iter()
        .zip(expected_group_schedule)
        .position(|(actual, expected)| actual != expected);
    let lwc_group_schedule_mismatch = if (actual_lwc_group_count, actual_lwc_group_hash)
        != (EXPECTED_LWC_GROUPS, EXPECTED_LWC_GROUP_SCHEDULE_HASH)
        || capture.lwc_group_schedule != expected_group_schedule
    {
        let mismatch = first_group_schedule_mismatch.unwrap_or(
            capture
                .lwc_group_schedule
                .len()
                .min(expected_group_schedule.len()),
        );
        let diagnostic_start = mismatch.saturating_sub(16);
        let diagnostic_end = (mismatch + 17).min(capture.lwc_group_schedule.len());
        Some(format!(
            "LWC group schedule mismatch: RTL groups/hash={actual_lwc_group_count}/\
             {actual_lwc_group_hash:#018x}, reference={EXPECTED_LWC_GROUPS}/\
             {EXPECTED_LWC_GROUP_SCHEDULE_HASH:#018x}; first mismatch={mismatch}; \
             RTL bands {diagnostic_start}..{diagnostic_end}={:?}; \
             reference={:?}",
            &capture.lwc_group_schedule[diagnostic_start..diagnostic_end],
            &expected_group_schedule[diagnostic_start..diagnostic_end]
        ))
    } else {
        None
    };
    let actual_lwc = capture
        .lwc_words
        .into_iter()
        .flat_map(u16::to_be_bytes)
        .collect::<Vec<_>>();
    let lwc_mismatch = if actual_lwc != expected_lwc {
        let mismatch = actual_lwc
            .iter()
            .zip(expected_lwc)
            .position(|(actual, expected)| actual != expected)
            .unwrap_or(actual_lwc.len().min(expected_lwc.len()));
        Some(format!(
            "LWC mismatch at byte {mismatch}, word cycle {:?}: \
             RTL={:?}, reference={:?}; first bytes RTL={:02x?}, reference={:02x?}; \
             byte counts RTL={}, reference={}",
            capture.lwc_word_cycles.get(mismatch / 2),
            actual_lwc.get(mismatch),
            expected_lwc.get(mismatch),
            &actual_lwc[..actual_lwc.len().min(24)],
            &expected_lwc[..expected_lwc.len().min(24)],
            actual_lwc.len(),
            expected_lwc.len()
        ))
    } else {
        None
    };
    let lftag_accepted = capture
        .lftag_packets
        .iter()
        .filter(|packet| packet.get(1).is_some_and(|word| word & 0x8000_0000 != 0))
        .count();
    let lftag_max_cycles = capture
        .lftag_candidate_cycles
        .iter()
        .copied()
        .max()
        .unwrap_or(0);
    eprintln!(
        "{metric_prefix} metrics: composite_pairs={actual_threshold_pairs} \
         composite_hash={actual_threshold_hash:#018x} \
         lwc_groups={actual_lwc_group_count} \
         lwc_schedule_hash={actual_lwc_group_hash:#018x} lwc_words={} \
         tag_events={} tag_groups={} uncapped_candidates={} \
         lftag_candidates={} lftag_accepted={} lftag_max_cycles={} \
         truncated_reference_groups={} table_overflows={}/{} \
         drain_cycles={drained}",
        actual_lwc.len() / 2,
        capture.tag_events.len(),
        actual_tag_groups.len(),
        expected_tag
            .uncapped_groups
            .iter()
            .map(|group| group.features.len())
            .sum::<usize>(),
        capture.lftag_packets.len(),
        lftag_accepted,
        lftag_max_cycles,
        expected_tag
            .uncapped_groups
            .iter()
            .filter(|group| group.features.len() > 9)
            .count(),
        outer_table_overflows,
        inner_table_overflows
    );
    let mismatches = [
        threshold_mismatch,
        table_depth_mismatch,
        tag_mismatch,
        lwc_group_schedule_mismatch,
        lwc_mismatch,
    ]
    .into_iter()
    .flatten()
    .collect::<Vec<_>>();
    if !mismatches.is_empty() {
        return Err(mismatches.join("; ").into());
    }
    Ok(())
}

#[test]
fn adaptive_reference_fixture_has_complete_raster() -> TestResult {
    let pixels = load_frame();
    let expected = expected_tag_data(&pixels)?;
    assert_eq!(
        (
            expected.threshold_stream_pairs,
            expected.threshold_stream_hash
        ),
        (EXPECTED_COMPOSITE_PAIRS, 0xf309_9bd1_54e4_297e)
    );
    assert_eq!(
        expected
            .uncapped_groups
            .iter()
            .filter(|g| g.features.len() >= 9)
            .count(),
        6
    );
    assert_eq!(
        expected
            .uncapped_groups
            .iter()
            .map(|g| g.features.len())
            .sum::<usize>(),
        7505
    );
    eprintln!(
        "adaptive fixture: hash={:#018x} groups={} children={}",
        expected.threshold_stream_hash,
        expected
            .uncapped_groups
            .iter()
            .filter(|g| g.features.len() >= 9)
            .count(),
        expected
            .uncapped_groups
            .iter()
            .map(|g| g.features.len())
            .sum::<usize>()
    );
    Ok(())
}

#[test]
fn combined_rtl_matches_wavelet_tagdetect_and_bit_exact_lwc_at_896_clocks_per_row() -> TestResult {
    let pixels = load_frame();
    let expected_lwc = Codec::encode(&pixels, FRAME_WIDTH, FRAME_HEIGHT)?;
    let expected_first = expected_tag_data(&pixels)?;
    let expected_tag = expected_tag_data(&pixels)?;
    let expected_tag_event_count = expected_tag
        .uncapped_groups
        .iter()
        .filter(|group| group.features.len() >= 9)
        .count()
        * 9;

    let root = gateware_root();
    let adapter_rel = Path::new("tests/lftag/tests/rtl_adapters/pipeline_tagdetect_lwc.sv");
    let adapter = root.join(adapter_rel);
    with_model(
        &root,
        Path::new("rtl/tagdetect_lwc/sources.f"),
        "tagdetect-lwc-wavelet-full-frame",
        "pipeline_tagdetect_lwc",
        adapter_rel,
        PORTS,
        &[
            Path::new("rtl/lrcca"),
            Path::new("rtl/lwc"),
            Path::new("rtl/lftag"),
        ],
        &[adapter.as_path()],
        |model| {
            let (first, first_drain) = run_frame(model, &pixels, expected_tag_event_count)?;
            validate_combined_capture(
                model,
                first,
                first_drain,
                expected_lwc.bytes(),
                &expected_first,
                "combined-first",
            )?;
            // Per-frame reset must reproduce the same detections immediately.
            let (capture, drained) = run_frame(model, &pixels, expected_tag_event_count)?;
            assert!(
                drained < MAX_DRAIN_CYCLES,
                "combined outputs must finish within the bounded drain"
            );
            validate_combined_capture(
                model,
                capture,
                drained,
                expected_lwc.bytes(),
                &expected_tag,
                "combined",
            )
        },
    )
}

#[test]
fn ov9281_receiver_directly_feeds_full_frame_tagdetect_lwc_at_nominal_clocks() -> TestResult {
    let pixels = load_frame();
    let expected_lwc = Codec::encode(&pixels, FRAME_WIDTH, FRAME_HEIGHT)?;
    let expected_first = expected_tag_data(&pixels)?;
    let expected_tag = expected_tag_data(&pixels)?;
    let expected_tag_event_count = expected_tag
        .uncapped_groups
        .iter()
        .filter(|group| group.features.len() >= 9)
        .count()
        * 9;

    let root = gateware_root();
    let adapter_rel = Path::new("tests/lftag/tests/rtl_adapters/pipeline_ov9281_tagdetect_lwc.sv");
    let adapter = root.join(adapter_rel);
    let diagnostic_adapter = root.join("tests/lftag/tests/rtl_adapters/pipeline_tagdetect_lwc.sv");
    let lane_aligner = root.join("rtl/csi2_rx/Ov9281LaneAligner.sv");
    let pixel_fifo = root.join("rtl/csi2_rx/Ov9281PixelAsyncFifo.sv");
    let counter = root.join("rtl/csi2_rx/Ov9281Counter.sv");
    let receiver = root.join("rtl/csi2_rx/csi2_rx.sv");
    let mut ports = CAMERA_PORTS.to_vec();
    ports.extend([
        ("processing_reset", 0, 0, PortDirection::Input),
        ("pipeline_busy", 0, 0, PortDirection::Output),
    ]);
    with_model(
        &root,
        Path::new("rtl/tagdetect_lwc/sources.f"),
        "ov9281-direct-tagdetect-lwc-full-frame",
        "pipeline_ov9281_tagdetect_lwc",
        adapter_rel,
        &ports,
        &[
            Path::new("rtl/csi2_rx"),
            Path::new("rtl/lrcca"),
            Path::new("rtl/lwc"),
            Path::new("rtl/lftag"),
        ],
        &[
            adapter.as_path(),
            diagnostic_adapter.as_path(),
            lane_aligner.as_path(),
            pixel_fifo.as_path(),
            counter.as_path(),
            receiver.as_path(),
        ],
        |model| {
            model.pin("processing_reset", 0_u8)?;
            let mut sim = CameraSimulation::new(model)?;
            sim.reset_and_acquire_link()?;

            for frame in 0..2 {
                let (expected_tag, expected_tag_event_count) = if frame == 0 {
                    (&expected_first, expected_tag_event_count)
                } else {
                    // Keep the receiver link alive while
                    // resetting only the per-frame processing/diagnostics.
                    sim.dut.pin("processing_reset", 1_u8)?;
                    sim.run_core_edges(4)?;
                    sim.dut.pin("processing_reset", 0_u8)?;
                    sim.capture = Capture::default();
                    sim.camera_line_core_cycles.clear();
                    sim.core_row_start_cycles.clear();
                    (&expected_tag, expected_tag_event_count)
                };
                let frame_id = 0x2794 + frame;
                let frame_start =
                    build_short_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_FRAME_START, frame_id)?;
                let frame_start_byte = sim.send_packet_at(&frame_start, [0, 7], 2, 0, None)?;
                let first_line_target = frame_start_byte + 120;
                let mut line_packet_starts = Vec::with_capacity(FRAME_HEIGHT);

                for row in 0..FRAME_HEIGHT {
                    let row_start = row * FRAME_WIDTH;
                    let raw10_pixels = pixels[row_start..row_start + FRAME_WIDTH]
                        .iter()
                        .map(|pixel| u16::from(*pixel) << 2)
                        .collect::<Vec<_>>();
                    let payload = pack_raw10(&raw10_pixels)?;
                    let packet =
                        build_long_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_RAW10, &payload)?;
                    let target = first_line_target + row * CAMERA_LINE_BYTE_CYCLES;
                    let actual = sim.send_packet_at(
                        &packet,
                        [row & 7, 7 - (row & 7)],
                        row % 4,
                        row * 19,
                        Some(target),
                    )?;
                    line_packet_starts.push(actual);
                    let receiver_status = read_u64(sim.dut, "receiver_status")?;
                    if receiver_status != 0 {
                        return Err(format!(
                            "receiver status {receiver_status:#06x} after RAW10 row {row}"
                        )
                        .into());
                    }
                }

                let frame_end =
                    build_short_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_FRAME_END, frame_id)?;
                let frame_end_target = first_line_target + FRAME_HEIGHT * CAMERA_LINE_BYTE_CYCLES;
                sim.send_packet_at(&frame_end, [3, 6], 1, 7, Some(frame_end_target))?;

                let nominal_frame_end = frame_start_byte + CAMERA_FRAME_BYTE_CYCLES;
                sim.run_to_byte_edge(nominal_frame_end)?;
                let drain_start = sim.core_edges;
                sim.drain_outputs(expected_tag_event_count, OUTPUT_WORDS, MAX_DRAIN_CYCLES)?;
                // Wait for the complete pipeline to drain.
                while read_u64(sim.dut, "pipeline_busy")? != 0
                    && sim.core_edges - drain_start < MAX_DRAIN_CYCLES
                {
                    sim.run_core_edges(1)?;
                }
                assert_eq!(read_u64(sim.dut, "pipeline_busy")?, 0);
                let drained = sim.core_edges - drain_start;
                sim.run_core_edges(OUTPUT_QUIET_CYCLES)?;

                assert_eq!(read_u64(sim.dut, "receiver_status")?, 0);
                assert_eq!(read_u64(sim.dut, "receiver_protocol_error")?, 0);
                assert_eq!(read_u64(sim.dut, "receiver_marker_error")?, 0);
                assert_eq!(read_u64(sim.dut, "camera_rows_received")?, 800);
                assert_eq!(read_u64(sim.dut, "core_rows_started")?, 800);
                assert_eq!(read_u64(sim.dut, "core_rows_completed")?, 800);
                assert_eq!(line_packet_starts.len(), 800);
                assert!(
                    line_packet_starts
                        .windows(2)
                        .all(|pair| pair[1] - pair[0] == CAMERA_LINE_BYTE_CYCLES)
                );
                assert_eq!(sim.camera_line_core_cycles.len(), 800);
                assert_eq!(sim.core_row_start_cycles.len(), 800);

                let core_row_periods = sim
                    .core_row_start_cycles
                    .windows(2)
                    .map(|pair| pair[1] - pair[0])
                    .collect::<Vec<_>>();
                let min_core_row_period = *core_row_periods.iter().min().unwrap();
                let max_core_row_period = *core_row_periods.iter().max().unwrap();
                let receiver_overflow = read_u64(sim.dut, "receiver_status")? & 0x0800;
                eprintln!(
                    "camera cadence: frame_start_byte={frame_start_byte} \
                 nominal_frame_bytes={CAMERA_FRAME_BYTE_CYCLES} \
                 line_byte_cycles={CAMERA_LINE_BYTE_CYCLES} \
                 core_row_period={min_core_row_period}..{max_core_row_period} \
                 camera_lines={} core_rows={} receiver_fifo_overflow={} \
                 core_cycles_at_drain={} post_frame_drain={drained}",
                    sim.camera_line_core_cycles.len(),
                    sim.core_row_start_cycles.len(),
                    receiver_overflow != 0,
                    drain_start,
                );
                assert_eq!(receiver_overflow, 0);

                let queue_peaks = verify_fixed_camera_queue_backlog(&sim.capture)?;
                eprintln!("camera conservative group-held pair peaks: {queue_peaks:?}");

                let capture = std::mem::take(&mut sim.capture);
                validate_combined_capture(
                    sim.dut,
                    capture,
                    drained,
                    expected_lwc.bytes(),
                    expected_tag,
                    &format!("camera-direct-frame-{frame}"),
                )?;
            }
            Ok(())
        },
    )
}

struct CsiTraceConfig {
    adapter: PathBuf,
    scene: String,
    seed: u64,
    frames: usize,
}

/// Real two-lane CSI packets at 100 MHz, crossing to the 120 MHz processing
/// clock, with three frame starts separated by the caller's fixed period.
/// Dividing that period among 910 total lines reserves real vertical blanking
/// for processing drain; no frame-specific drain extension is permitted.
fn run_ov9281_monoisp_repeated_frames(
    frame_byte_cycles: usize,
    dual: bool,
    trace_config: Option<CsiTraceConfig>,
) -> TestResult {
    // Both timing profiles share the same Verilator artifact. Serialize their
    // first build as well as loading it when cargo runs tests in parallel.
    static MODEL_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    let _guard = MODEL_LOCK
        .lock()
        .map_err(|_| "CSI repeated-frame model lock poisoned")?;
    const TOTAL_LINES: usize = 910;
    let frames = trace_config.as_ref().map_or(3, |config| config.frames);
    let trace_mode = trace_config.is_some();
    assert!(
        !trace_mode || !dual,
        "trace analysis selects one complete camera"
    );
    let pixels = load_frame();
    let expected_lwc = Codec::encode(&pixels, FRAME_WIDTH, FRAME_HEIGHT)?;
    let expected_first = expected_tag_data(&pixels)?;
    let expected_tag = expected_tag_data(&pixels)?;
    let root = gateware_root();
    let (adapter_rel, top_module, artifact_namespace) = if let Some(config) = &trace_config {
        (
            config.adapter.as_path(),
            "pipeline_ov9281_trace_camera",
            "ov9281-monoisp-camera-packet-trace",
        )
    } else if dual {
        (
            Path::new("tests/lftag/tests/rtl_adapters/pipeline_ov9281_dual_camera.sv"),
            "pipeline_ov9281_dual_camera",
            "ov9281-monoisp-dual-camera-three-frames",
        )
    } else {
        (
            Path::new("tests/lftag/tests/rtl_adapters/pipeline_ov9281_camera.sv"),
            "pipeline_ov9281_camera",
            "ov9281-monoisp-camera-three-frames-120fps",
        )
    };
    let mut extra_sources = vec![
        Path::new("tests/lftag/tests/rtl_adapters/pipeline_ov9281_camera.sv"),
        Path::new("rtl/csi2_rx/Ov9281LaneAligner.sv"),
        Path::new("rtl/csi2_rx/Ov9281PixelAsyncFifo.sv"),
        Path::new("rtl/csi2_rx/Ov9281Counter.sv"),
        Path::new("rtl/csi2_rx/csi2_rx.sv"),
        Path::new("rtl/isp/MonoIsp.sv"),
        Path::new("rtl/lutcam/CameraPipeline.sv"),
    ];
    if dual || trace_mode {
        extra_sources.push(adapter_rel);
    }
    let extra_sources = extra_sources
        .into_iter()
        .map(|path| root.join(path))
        .collect::<Vec<_>>();
    let mut ports = CAMERA_PORTS.to_vec();
    ports.extend([
        ("camera_fault", 0, 0, PortDirection::Output),
        ("camera_idle", 0, 0, PortDirection::Output),
        ("frames_completed", 31, 0, PortDirection::Output),
        ("current_frame_id", 31, 0, PortDirection::Output),
        ("receiver_pairs", 31, 0, PortDirection::Output),
        ("capture_sof_clock", 31, 0, PortDirection::Output),
        ("capture_eof_clock", 31, 0, PortDirection::Output),
        ("capture_retire_clock", 31, 0, PortDirection::Output),
    ]);
    if dual {
        ports.extend([
            ("dual_compare_error", 7, 0, PortDirection::Output),
            ("dual_core_compare_cycles", 31, 0, PortDirection::Output),
            ("dual_byte_compare_cycles", 31, 0, PortDirection::Output),
            ("second_frames_completed", 31, 0, PortDirection::Output),
            ("second_camera_idle", 0, 0, PortDirection::Output),
            ("second_capture_retire_clock", 31, 0, PortDirection::Output),
        ]);
    }
    with_model(
        &root,
        Path::new("rtl/tagdetect_lwc/sources.f"),
        artifact_namespace,
        top_module,
        adapter_rel,
        &ports,
        &[
            Path::new("rtl/csi2_rx"),
            Path::new("rtl/lrcca"),
            Path::new("rtl/lwc"),
            Path::new("rtl/lftag"),
        ],
        &extra_sources
            .iter()
            .map(PathBuf::as_path)
            .collect::<Vec<_>>(),
        |model| {
            let mut sim = CameraSimulation::with_board_guards(model)?;
            sim.reset_and_acquire_link()?;
            let mut first_frame_start = None;
            let mut previous_frame_start = None;
            let mut previous_retire_clock = None;
            let mut previous_second_retire_clock = None;
            let mut random_state = trace_config.as_ref().map_or(1, |config| config.seed);
            for frame in 0..frames {
                let trace_pixels = if let Some(config) = &trace_config {
                    if config.scene == "flat-binary-late" {
                        random_state = config.seed;
                    }
                    (0..FRAME_WIDTH * FRAME_HEIGHT)
                        .map(|index| {
                            random_state ^= random_state << 13;
                            random_state ^= random_state >> 7;
                            random_state ^= random_state << 17;
                            if config.scene == "uniform" {
                                (random_state >> 56) as u8
                            } else if config.scene == "flat-binary-late"
                                && (frame == 0 || (frame == 2 && index < 560 * FRAME_WIDTH))
                            {
                                128
                            } else if random_state & 1 != 0 {
                                0
                            } else {
                                255
                            }
                        })
                        .collect::<Vec<u8>>()
                } else {
                    Vec::new()
                };
                let pixels = if trace_mode { &trace_pixels } else { &pixels };
                if let Some(config) = &trace_config {
                    let hash = pixels.iter().fold(0xcbf29ce484222325_u64, |hash, byte| {
                        (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3)
                    });
                    eprintln!(
                        "CSI input frame={frame} scene={} seed={} pixel_hash={hash:016x}",
                        config.scene, config.seed
                    );
                }
                assert_eq!(read_u64(sim.dut, "camera_idle")?, 1);
                assert_eq!(read_u64(sim.dut, "frames_completed")?, frame as u64);
                let frame_start = build_short_packet(
                    OV9281_VIRTUAL_CHANNEL,
                    DATA_TYPE_FRAME_START,
                    frame as u16,
                )?;
                let target = first_frame_start.map(|start| start + frame * frame_byte_cycles);
                let start = sim.send_packet_at(&frame_start, [0, 7], 2, 0, target)?;
                if let Some(previous) = previous_frame_start {
                    assert_eq!(start - previous, frame_byte_cycles);
                }
                first_frame_start.get_or_insert(start);
                previous_frame_start = Some(start);
                let first_line = start + 120;
                let row_clock_start = sim.core_row_start_cycles.len();
                for row in 0..FRAME_HEIGHT {
                    let raw10 = pixels[row * FRAME_WIDTH..(row + 1) * FRAME_WIDTH]
                        .iter()
                        .enumerate()
                        // Exercise every RAW10 low-bit combination as well.
                        .map(|(column, pixel)| {
                            (u16::from(*pixel) << 2) | ((column + row) & 3) as u16
                        })
                        .collect::<Vec<_>>();
                    let payload = pack_raw10(&raw10)?;
                    let packet =
                        build_long_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_RAW10, &payload)?;
                    let target = first_line + row * frame_byte_cycles / TOTAL_LINES;
                    assert_eq!(
                        sim.send_packet_at(
                            &packet,
                            [row & 7, 7 - (row & 7)],
                            row % 4,
                            row * 19,
                            Some(target)
                        )?,
                        target
                    );
                    assert_eq!(
                        read_u64(sim.dut, "receiver_status")?,
                        0,
                        "frame {frame} row {row}"
                    );
                    assert_eq!(
                        read_u64(sim.dut, "camera_fault")?,
                        0,
                        "frame {frame} row {row}"
                    );
                    if dual {
                        assert_eq!(
                            read_u64(sim.dut, "dual_compare_error")?,
                            0,
                            "concurrent camera mismatch: frame {frame} row {row}"
                        );
                    }
                }
                let frame_end =
                    build_short_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_FRAME_END, frame as u16)?;
                let end_target = first_line + FRAME_HEIGHT * frame_byte_cycles / TOTAL_LINES;
                sim.send_packet_at(&frame_end, [3, 6], 1, 7, Some(end_target))?;
                // Leave packet setup room before the *fixed* next frame start.
                // No frame-dependent drain extension is permitted.
                sim.run_to_byte_edge(start + frame_byte_cycles - 64)?;
                assert_eq!(read_u64(sim.dut, "camera_fault")?, 0);
                assert_eq!(read_u64(sim.dut, "receiver_status")?, 0);
                assert_eq!(read_u64(sim.dut, "receiver_protocol_error")?, 0);
                assert_eq!(read_u64(sim.dut, "receiver_marker_error")?, 0);
                assert_eq!(read_u64(sim.dut, "receiver_pairs")?, 512_000);
                assert_eq!(read_u64(sim.dut, "camera_rows_received")?, 800);
                assert_eq!(read_u64(sim.dut, "core_rows_started")?, 800);
                assert_eq!(read_u64(sim.dut, "core_rows_completed")?, 800);
                assert_eq!(
                    read_u64(sim.dut, "camera_idle")?,
                    1,
                    "frame {frame} did not drain within its frame period"
                );
                assert_eq!(read_u64(sim.dut, "frames_completed")?, (frame + 1) as u64);
                assert_eq!(read_u64(sim.dut, "current_frame_id")?, frame as u64);
                if dual {
                    assert_eq!(read_u64(sim.dut, "second_camera_idle")?, 1);
                    assert_eq!(read_u64(sim.dut, "dual_compare_error")?, 0);
                    assert_eq!(
                        read_u64(sim.dut, "second_frames_completed")?,
                        (frame + 1) as u64
                    );
                    let core_comparisons = read_u64(sim.dut, "dual_core_compare_cycles")?;
                    let byte_comparisons = read_u64(sim.dut, "dual_byte_compare_cycles")?;
                    assert!(core_comparisons >= ((frame + 1) * frame_byte_cycles * 6 / 5) as u64);
                    assert!(byte_comparisons >= ((frame + 1) * frame_byte_cycles) as u64);
                    eprintln!(
                        "concurrent dual CSI+MonoIsp frame={frame} period_byte={frame_byte_cycles} mismatch=0 second_frames_completed={} compared_core_edges={core_comparisons} compared_byte_edges={byte_comparisons}",
                        frame + 1
                    );
                }
                if !trace_mode {
                    let expected_groups = expected_tag
                        .uncapped_groups
                        .iter()
                        .filter(|g| g.features.len() >= 9)
                        .count();
                    assert_eq!(sim.capture.tag_events.len(), expected_groups * 9);
                    assert_eq!(sim.capture.lftag_packets.len(), expected_groups);
                }
                assert_eq!(sim.capture.lwc_words.len(), OUTPUT_WORDS);
                for packet in &sim.capture.lftag_packets {
                    assert_eq!((packet[1] >> 20) & 0xff, frame as u32);
                }
                let queue_peaks = verify_fixed_camera_queue_backlog(&sim.capture)?;
                let sof = read_u64(sim.dut, "capture_sof_clock")?;
                let eof = read_u64(sim.dut, "capture_eof_clock")?;
                let retire = read_u64(sim.dut, "capture_retire_clock")?;
                let rows = &sim.core_row_start_cycles[row_clock_start..];
                assert_eq!(rows.len(), FRAME_HEIGHT);
                let row_periods = rows
                    .windows(2)
                    .map(|pair| pair[1] - pair[0])
                    .collect::<Vec<_>>();
                eprintln!(
                    "frame={frame} admitted_sof_core={sof} received_eof_core={eof} retired_core={retire} tail_core={} next_transport_fs_byte={} row_core_min={} row_core_max={}",
                    retire - eof,
                    start + frame_byte_cycles,
                    row_periods.iter().min().unwrap(),
                    row_periods.iter().max().unwrap()
                );
                if let Some(previous) = previous_retire_clock {
                    assert!(sof > previous);
                    eprintln!(
                        "frame={} retirement_to_next_admitted_sof_margin_core={}",
                        frame - 1,
                        sof - previous
                    );
                }
                previous_retire_clock = Some(retire);
                if dual {
                    let second_retire = read_u64(sim.dut, "second_capture_retire_clock")?;
                    if let Some(previous) = previous_second_retire_clock {
                        assert!(sof > previous, "second camera missed the next admitted SOF");
                        eprintln!(
                            "frame={} second_retirement_to_next_admitted_sof_margin_core={}",
                            frame - 1,
                            sof - previous
                        );
                    }
                    previous_second_retire_clock = Some(second_retire);
                    eprintln!("frame={frame} second_retired_core={second_retire}");
                }
                eprintln!(
                    "actual CSI+MonoIsp frame={frame} start_byte={start} period_byte={frame_byte_cycles} receiver_pairs=512000 idle=1 faults=0 compact_queue_peaks={queue_peaks:?}"
                );
                let capture = std::mem::take(&mut sim.capture);
                if !trace_mode {
                    validate_combined_capture(
                        sim.dut,
                        capture,
                        0,
                        expected_lwc.bytes(),
                        // Each repeated input produces the same detections.
                        if frame == 0 {
                            &expected_first
                        } else {
                            &expected_tag
                        },
                        &format!("csi-monoisp-frame-{frame}"),
                    )?;
                } else {
                    eprintln!(
                        "CSI packet trace frame={frame} words={} children={} solver_packets={}",
                        capture.lwc_words.len(),
                        capture.tag_events.len(),
                        capture.lftag_packets.len()
                    );
                }
            }
            Ok(())
        },
    )
}

#[test]
fn ov9281_monoisp_camera_repeats_three_complete_frames_at_120fps() -> TestResult {
    run_ov9281_monoisp_repeated_frames(833_334, false, None)
}

#[test]
fn ov9281_monoisp_camera_repeats_three_frames_at_documented_910_byte_line_period() -> TestResult {
    // OV9281 example: SYS_CLK=80MHz, HTS=728, VTS=910. At 100MHz
    // receiver byte clock this is 910 byte clocks/line, 828100/frame.
    run_ov9281_monoisp_repeated_frames(828_100, false, None)
}

// Both independent receiver/ISP/processing instances receive the same CSI
// packets concurrently. The adapter checks every valid payload and all public
// status/timing against camera 0, which retains the software-reference checks.
// This deliberately does not claim asynchronous-link or shared-UDP coverage.
#[test]
fn ov9281_monoisp_dual_camera_repeats_three_complete_frames_at_120fps() -> TestResult {
    run_ov9281_monoisp_repeated_frames(833_334, true, None)
}

#[test]
fn ov9281_monoisp_dual_camera_repeats_three_frames_at_documented_910_byte_line_period() -> TestResult
{
    run_ov9281_monoisp_repeated_frames(828_100, true, None)
}

// Explicit analysis-only export: ordinary golden regressions above never consult
// these environment variables. The supplied simulation-only adapter logs all
// image/child/solver words and core-clock timestamps for exact UDP replay.
#[test]
#[ignore = "requires an explicit CSI trace adapter; exports selected analysis scenes"]
fn ov9281_monoisp_export_packet_trace() -> TestResult {
    let adapter = PathBuf::from(std::env::var("LUTCAM_CSI_TRACE_ADAPTER")?);
    let frame_byte_cycles = std::env::var("LUTCAM_CSI_TRACE_PERIOD")
        .map_or(Ok(833_334), |value| value.parse::<usize>())?;
    let scene =
        std::env::var("LUTCAM_CSI_TRACE_SCENE").unwrap_or_else(|_| "flat-binary-late".to_owned());
    assert!(["flat-binary-late", "binary", "uniform"].contains(&scene.as_str()));
    let seed = std::env::var("LUTCAM_CSI_TRACE_SEED")
        .map_or(Ok(0x243f6a8885a308d3_u64), |value| value.parse::<u64>())?;
    assert_ne!(seed, 0, "xorshift seed must be nonzero");
    let frames =
        std::env::var("LUTCAM_CSI_TRACE_FRAMES").map_or(Ok(3), |value| value.parse::<usize>())?;
    assert!((1..=256).contains(&frames));
    run_ov9281_monoisp_repeated_frames(
        frame_byte_cycles,
        false,
        Some(CsiTraceConfig {
            adapter,
            scene,
            seed,
            frames,
        }),
    )
}
