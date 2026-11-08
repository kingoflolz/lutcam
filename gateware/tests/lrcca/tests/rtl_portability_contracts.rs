//! Structural contracts for the shared, vendor-neutral centroid RTL.

use std::fs;
use std::path::{Path, PathBuf};

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn source(relative: &str) -> String {
    fs::read_to_string(gateware_root().join(relative)).unwrap()
}

fn centroid_source() -> String {
    [
        "rtl/lrcca/CentroidMantissaWindowStage.sv",
        "rtl/lrcca/CentroidMantissaWindow.sv",
        "rtl/lrcca/CentroidSignedWindowStage.sv",
        "rtl/lrcca/CentroidSignedMomentNormalizer.sv",
        "rtl/lrcca/CentroidPower2MultiplierNormalizer.sv",
        "rtl/lrcca/CentroidMidpointReflectionReciprocal.sv",
        "rtl/lrcca/CentroidCompressor.sv",
    ]
    .into_iter()
    .map(source)
    .collect::<Vec<_>>()
    .join("\n")
}

fn assert_shared_portable(source: &str) {
    for forbidden in ["ifdef", "EFX_DSP", "syn_use_dsp"] {
        assert!(
            !source.contains(forbidden),
            "vendor-specific `{forbidden}` remains"
        );
    }
}

#[test]
fn power2_normalizer_is_shared_portable_rtl_with_explicit_production_opt_in() {
    let centroid = centroid_source();
    for fragment in [
        "module CentroidPower2MultiplierNormalizer",
        "assign mass_product = mass * scale;",
        "assign x_product = x_value * signed_scale;",
        "assign y_product = y_value * signed_scale;",
        "mass_product[MASS_WINDOW_LSB +: MASS_FRAC_BITS]",
        "x_product[MOMENT_SHIFT-1]",
        "y_product[MOMENT_SHIFT-1]",
        "parameter bit POWER2_MULTIPLIER_NORMALIZER = 1'b0",
    ] {
        assert!(centroid.contains(fragment), "missing `{fragment}`");
    }
    assert_shared_portable(&centroid);

    let hierarchy = source("rtl/lrcca/HierarchicalTwoPPC.sv");
    let tagdetect = source("rtl/lrcca/WaveletTagDetect.sv");
    assert!(hierarchy.contains("parameter bit CENTROID_POWER2_MULTIPLIER_NORMALIZER = 1'b0"));
    assert!(tagdetect.contains(".CENTROID_POWER2_MULTIPLIER_NORMALIZER(1'b1)"));
}

#[test]
fn signed_moment_window_has_one_shared_ordinary_rtl_implementation() {
    let centroid = centroid_source();
    assert_eq!(
        centroid.matches("module CentroidSignedWindowStage").count(),
        1
    );
    assert_eq!(
        centroid
            .matches("module CentroidSignedMomentNormalizer")
            .count(),
        1
    );
    assert_eq!(
        centroid
            .matches("CentroidSignedMomentNormalizer #(")
            .count(),
        3
    );
    assert!(!centroid.contains("normalize_x_moment"));
    assert!(!centroid.contains("normalize_y_moment"));
    assert!(centroid.contains("right_quotient +"));
    for forbidden in ["ifdef", "EFX_", "syn_"] {
        assert!(
            !centroid.contains(forbidden),
            "vendor-specific `{forbidden}` remains"
        );
    }
}

#[test]
fn midpoint_reflection_is_default_off_portable_rtl_and_tagdetect_opts_in() {
    let centroid = centroid_source();
    for fragment in [
        "module CentroidMidpointReflectionReciprocal",
        "parameter bit MIDPOINT_REFLECTION_RECIPROCAL = 1'b0",
        "assign reflection_product = coefficient * reflected_mantissa;",
        "~normalized_fraction[RESIDUAL_BITS-1:DISCARD_BITS]",
    ] {
        assert!(centroid.contains(fragment), "missing `{fragment}`");
    }
    let midpoint = centroid
        .split_once("begin : midpoint_reflection_reciprocal")
        .expect("midpoint generate block")
        .1
        .split_once("end else begin : segmented_reciprocal")
        .expect("segmented generate block")
        .0;
    assert!(!midpoint.contains(" + "));
    assert!(!midpoint.contains(" - "));
    assert_shared_portable(&centroid);

    let expected_rom = "centroid_reciprocal_midpoint_reflection_q17.bin";
    for path in [
        "rtl/lrcca/CentroidCompressor.sv",
        "rtl/lrcca/HierarchicalTwoPPC.sv",
        "rtl/lrcca/WaveletTagDetect.sv",
        "fpga/litex/lutcam/camera.py",
    ] {
        assert!(
            source(path).contains(expected_rom),
            "{path} does not name midpoint ROM"
        );
    }
    let hierarchy = source("rtl/lrcca/HierarchicalTwoPPC.sv");
    let tagdetect = source("rtl/lrcca/WaveletTagDetect.sv");
    assert!(hierarchy.contains("parameter bit CENTROID_MIDPOINT_REFLECTION_RECIPROCAL = 1'b0"));
    assert!(tagdetect.contains(".CENTROID_MIDPOINT_REFLECTION_RECIPROCAL(1'b1)"));
}

#[test]
fn split_base_interpolation_uses_shared_portable_rtl() {
    let centroid = centroid_source();
    for fragment in [
        "RECIPROCAL_BASE_SPLIT_BITS = 4",
        "RECIPROCAL_SPLIT_C_BITS = 18",
        "reciprocal_split_c_extended - reciprocal_drop_product_signed",
        "reciprocal_split_term[RECIPROCAL_BASE_SPLIT_BITS-1:0]",
    ] {
        assert!(centroid.contains(fragment), "missing `{fragment}`");
    }
    assert!(!centroid.contains("reciprocal_interpolated_drop"));
    assert_shared_portable(&centroid);
}
