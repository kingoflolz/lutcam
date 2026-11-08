//! Shared Marlin support for production-RTL integration tests.
//!
//! Tests compile the ordered canonical source manifest, never a copied RTL
//! source list. Marlin owns source-change detection inside stable per-top
//! artifact directories.

#![allow(dead_code)]

use std::collections::HashSet;
use std::path::{Path, PathBuf};

use marlin::verilator::PortDirection;
use marlin::verilator::dynamic::DynamicVerilatedModel;

#[allow(unused_imports)]
pub use rtl_test_support::{TestResult, clock, read_u64, sign_extend};

pub const RTL_SOURCE_MANIFEST: &str = "rtl/lwc/codec_encoder_sources.f";

fn crate_root() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

fn gateware_root() -> &'static Path {
    Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../.."))
}

/// Loads and validates the ordered production RTL closure.
pub fn production_rtl_sources() -> TestResult<Vec<PathBuf>> {
    let sources =
        rtl_test_support::production_rtl_sources(gateware_root(), Path::new(RTL_SOURCE_MANIFEST))?;
    if !sources
        .iter()
        .any(|source| source == &gateware_root().join("rtl/lwc/codec_encoder_top.sv"))
    {
        return Err(format!(
            "production RTL manifest {RTL_SOURCE_MANIFEST} omits rtl/lwc/codec_encoder_top.sv"
        )
        .into());
    }

    let mut unique = HashSet::new();
    for source in &sources {
        let filename = source
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or_default();
        if source.parent() != Some(&gateware_root().join("rtl/lwc"))
            || source.extension().and_then(|extension| extension.to_str()) != Some("sv")
            || filename.contains("_tb")
            || filename.contains("evidence")
            || filename.contains("harness")
        {
            return Err(format!(
                "non-production entry in {RTL_SOURCE_MANIFEST}: {}",
                source.display()
            )
            .into());
        }
        assert!(
            unique.insert(source.clone()),
            "shared manifest loader rejected duplicates"
        );
    }
    Ok(sources)
}

/// Creates a dynamic model from the production closure and runs `test` while
/// its owning runtime remains alive.
pub fn with_model<T>(
    artifact_namespace: &str,
    top_module: &str,
    top_source: &str,
    ports: &[(&str, usize, usize, PortDirection)],
    extra_sources: &[&Path],
    test: impl for<'ctx> FnOnce(&mut DynamicVerilatedModel<'ctx>) -> TestResult<T>,
) -> TestResult<T> {
    production_rtl_sources()?;
    let top_source = if top_source.starts_with("rtl/lwc/") {
        gateware_root().join(top_source)
    } else if let Ok(relative) = Path::new(top_source).strip_prefix("rtl") {
        gateware_root().join("rtl/lwc").join(relative)
    } else {
        crate_root().join(top_source)
    };
    let extra_sources = extra_sources
        .iter()
        .map(|source| crate_root().join(source))
        .collect::<Vec<_>>();
    let extra_source_refs = extra_sources
        .iter()
        .map(PathBuf::as_path)
        .collect::<Vec<_>>();
    rtl_test_support::with_model(
        gateware_root(),
        Path::new(RTL_SOURCE_MANIFEST),
        artifact_namespace,
        top_module,
        &top_source,
        ports,
        &[Path::new("rtl/lwc")],
        &extra_source_refs,
        test,
    )
}
