//! Structural and parameter contracts carried over from the LRCCA pytest suite.

use std::fs;
use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn source(relative: &str) -> String {
    fs::read_to_string(root().join(relative)).unwrap()
}

fn bit_width(value: usize) -> usize {
    let current = value.saturating_sub(1);
    (usize::BITS as usize - current.leading_zeros() as usize).max(1)
}

#[test]
fn canonical_hierarchy_feature_and_command_packing_widths() {
    let width = 1280usize;
    let height = 800usize;
    let hier_width = width + (width >> 2) + 8 + (width >> 4) + 8;
    let sort_key_bits = 3;
    let x_q4_bits = bit_width(hier_width << 4);
    let y_q4_bits = bit_width(height << 4);
    let generic_feature_width = sort_key_bits + 1 + x_q4_bits + y_q4_bits;
    let tagdetect_feature_width = sort_key_bits + x_q4_bits + y_q4_bits;
    let child_id_bits = bit_width(1024);
    let generic_command_width = 2 + child_id_bits + generic_feature_width.max(child_id_bits);
    let tagdetect_command_width = 2 + child_id_bits + tagdetect_feature_width.max(child_id_bits);

    assert_eq!(hier_width, 1696);
    assert_eq!((sort_key_bits, x_q4_bits, y_q4_bits), (3, 15, 14));
    assert_eq!(generic_feature_width, 33);
    assert_eq!(tagdetect_feature_width, 32);
    assert_eq!(generic_command_width, 45);
    assert_eq!(tagdetect_command_width, 44);

    let hierarchy = source("rtl/lrcca/HierarchicalTwoPPC.sv");
    let arena = source("rtl/lrcca/ChildArena.sv");
    for fragment in [
        "parameter bit FULL_LOG_MASS_CHILD_SORT = 1'b0",
        "CHILD_KEY_LOGM_BITS : CHILD_KEY_AREA_BITS",
        "CHILD_ARENA_SORT_KEY_BITS + INNER_CENTROID_VALID_BITS +",
        "parameter bit INNER_CENTROID_ALWAYS_VALID = 1'b0",
        "if (INNER_CENTROID_ALWAYS_VALID) begin : always_valid_child_fv",
        ".TAIL_KEY_LSB(FULL_LOG_MASS_CHILD_SORT ? 5 : 0)",
        ".TAIL_KEY_DENSE_AREA_RANK(!FULL_LOG_MASS_CHILD_SORT)",
        ".EQUAL_KEY_SERVICE_MIN(1'b1)",
        "child_sort_key = CHILD_ARENA_SORT_KEY_BITS'(dense_area) +",
        "log_m_q5_3[CHILD_KEY_LOGM_BITS-1]",
    ] {
        assert!(hierarchy.contains(fragment), "missing `{fragment}`");
    }
    assert!(arena.contains("equal_merge_choose_c"));
    assert!(arena.contains("CMD_WIDTH = 2 + ID_BITS + CMD_PAYLOAD_WIDTH"));
}

#[test]
fn wavelet_tagdetect_exposes_native_q4_full_mass_centroid_group_descriptors() {
    let wavelet = source("rtl/lrcca/WaveletTagDetect.sv");
    for fragment in [
        "localparam int CHILD_ARENA_SORT_KEY_BITS = 8",
        ".FULL_LOG_MASS_CHILD_SORT(1'b1)",
        "output logic centroid_group_valid",
        "output logic [CENTROID_GROUP_X_Q4_BITS-1:0] centroid_group_x_q4",
        "output logic [CENTROID_GROUP_Y_Q4_BITS-1:0] centroid_group_y_q4",
        "output logic [7:0] centroid_group_log_m_q5_3",
        "output logic centroid_group_centroid_valid",
        "assign centroid_group_centroid_valid = hier_centroid_valid",
        "assign centroid_group_x_q4 = hier_centroid_x_q4",
        "assign centroid_group_y_q4 = hier_centroid_y_q4",
        "assign centroid_group_log_m_q5_3 = hier_child_log_m_q5_3",
        "assign centroid_group_end = hier_inner_end",
    ] {
        assert!(wavelet.contains(fragment), "missing `{fragment}`");
    }

    let x_q4_bits = bit_width(1696 << 4);
    let y_q4_bits = bit_width(800 << 4);
    assert_eq!((x_q4_bits, y_q4_bits), (15, 14));
    assert_eq!(8 + y_q4_bits + x_q4_bits + 1, 38);
    assert!(!wavelet.to_ascii_lowercase().contains("lftag"));
    assert!(!wavelet.contains("frame_seq"));
}

#[test]
fn centroid_rom_parameter_reaches_litex_camera_wrapper() {
    for path in [
        "rtl/lrcca/CentroidCompressor.sv",
        "rtl/lrcca/HierarchicalTwoPPC.sv",
        "rtl/lrcca/WaveletTagDetect.sv",
        "fpga/litex/lutcam/camera.py",
    ] {
        let text = source(path);
        assert!(
            text.contains("centroid_reciprocal_segments_q17.bin"),
            "{path}"
        );
        assert!(!text.contains("CENTROID_LOG_RESIDUAL_BIN"), "{path}");
        assert!(!text.contains("CENTROID_EXP_RESIDUAL_BIN"), "{path}");
    }
}

fn saturate(value: usize, bits: usize) -> usize {
    value.min((1usize << bits) - 1)
}

#[test]
fn precompressed_child_area_key_matches_full_width_behavior() {
    for (area_bits, configured_max) in [
        (1, None),
        (2, None),
        (3, None),
        (4, Some(0)),
        (4, Some(1)),
        (4, Some(6)),
        (4, Some(7)),
        (4, Some(8)),
        (17, Some(113_778)),
        (21, Some(1_368_672)),
    ] {
        let representable_max = (1usize << area_bits) - 1;
        let area_max = configured_max.unwrap_or(representable_max);
        let mut boundaries = vec![0, 1, 6, 7, 8, area_max, representable_max];
        boundaries.push(area_max.saturating_sub(1));
        if area_max < representable_max {
            boundaries.push(area_max + 1);
        }
        for area in boundaries {
            if area <= representable_max {
                assert_eq!(area.min(area_max).min(7), area.min(area_max.min(7)));
            }
        }
    }
}

#[test]
fn inner_running_area_saturation_preserves_update_merge_and_key() {
    for area_bits in [1usize, 2, 3, 4, 21] {
        let inner_bits = area_bits.min(3);
        let representable_max = (1usize << area_bits) - 1;
        let mut boundaries = (0..=representable_max.min(8)).collect::<Vec<_>>();
        boundaries.extend([representable_max, representable_max.saturating_sub(1)]);
        for &area in &boundaries {
            let inner = saturate(area, inner_bits);
            assert_eq!(
                saturate(inner + 1, inner_bits),
                saturate(area + 1, inner_bits)
            );
            assert_eq!(inner != 0, area != 0);
        }
        for &left in &boundaries {
            for &right in &boundaries {
                assert_eq!(
                    saturate(
                        saturate(left, inner_bits) + saturate(right, inner_bits),
                        inner_bits
                    ),
                    saturate(left + right, inner_bits)
                );
            }
        }
        for configured_max in [0usize, 1, 6, 7, 8, 1_368_672] {
            if configured_max > representable_max {
                continue;
            }
            let mut areas = (0..10).collect::<Vec<_>>();
            areas.extend([
                configured_max,
                configured_max.saturating_sub(1),
                configured_max + 1,
                representable_max,
                representable_max + 1,
            ]);
            for area in areas {
                let old_key = saturate(area, area_bits).min(configured_max).min(7);
                let new_key = saturate(area, inner_bits).min(configured_max).min(7);
                assert_eq!(new_key, old_key);
            }
        }
    }
}

fn identifier_present(text: &str, needle: &str) -> bool {
    text.match_indices(needle).any(|(start, _)| {
        let before = text[..start].chars().next_back();
        let after = text[start + needle.len()..].chars().next();
        !before.is_some_and(|c| c == '_' || c.is_ascii_alphanumeric())
            && !after.is_some_and(|c| c == '_' || c.is_ascii_alphanumeric())
    })
}
