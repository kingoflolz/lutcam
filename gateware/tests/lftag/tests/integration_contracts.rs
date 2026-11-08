//! Structural contracts for the external LRCCA-to-LFTag integration.

use std::fs;
use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn source(relative: &str) -> String {
    fs::read_to_string(root().join(relative)).unwrap()
}

#[test]
fn combined_top_connects_generic_centroid_groups_to_standalone_lftag() {
    let combined = source("rtl/tagdetect_lwc/TagDetectLwc.sv");
    for fragment in [
        "LfTagCompact3x3DecoderLocalizer lftag_decoder_localizer (",
        "assign lftag_composite_x_q4 = centroid_group_x_q4",
        "assign lftag_logical_y_q4 = centroid_group_y_q4",
        "assign lftag_log_mass_q5_3 = centroid_group_log_m_q5_3",
        "LFTAG_LL2_X_START_Q4 = 15'd20544",
        "LFTAG_LL4_X_START_Q4 = 15'd25792",
        "if (lwc_in_sof)",
        "lftag_frame_seq <= lftag_frame_seq + 1'b1",
        ".centroid_group_valid",
        ".centroid_group_centroid_valid",
        ".centroid_group_x_q4",
        ".centroid_group_y_q4",
        ".centroid_group_log_m_q5_3",
        ".centroid_group_end",
        ".in_valid(centroid_group_valid)",
        ".in_group_end(centroid_group_end)",
        ".in_centroid_valid(lftag_centroid_valid && lftag_coordinate_valid)",
        ".in_frame_seq(lftag_frame_seq)",
        ".out_ready(lftag_result_ready)",
    ] {
        assert!(combined.contains(fragment), "missing `{fragment}`");
    }
    assert!(!combined.contains("`include"));
    for removed in [
        "centroid_group_overflow",
        "lftag_group_overflow",
        ".in_overflow(",
    ] {
        assert!(
            !combined.contains(removed),
            "retained child-list truncation must not enter the decoder as `{removed}`"
        );
    }

    let manifest = source("rtl/tagdetect_lwc/sources.f");
    let entries = manifest
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .collect::<Vec<_>>();
    let package = entries
        .iter()
        .position(|line| *line == "rtl/lftag/lftag_pkg.sv")
        .expect("combined manifest omits LFTag package");
    let decoder = entries
        .iter()
        .position(|line| *line == "rtl/lftag/LfTagCompact3x3DecoderLocalizer.sv")
        .expect("combined manifest omits LFTag decoder");
    let top = entries
        .iter()
        .position(|line| *line == "rtl/tagdetect_lwc/TagDetectLwc.sv")
        .expect("combined manifest omits combined top");
    assert!(package < decoder && decoder < top);
    let mut unique = entries.clone();
    unique.sort_unstable();
    unique.dedup();
    assert_eq!(
        unique.len(),
        entries.len(),
        "combined manifest has duplicates"
    );
}
