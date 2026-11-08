//! Shared Marlin/Verilator support for production RTL integration tests.

use std::collections::HashSet;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use marlin::verilator::dynamic::{DynamicVerilatedModel, VerilatorValue};
use marlin::verilator::{
    AsDynamicVerilatedModel, PortDirection, VerilatedModelConfig, VerilatorRuntime,
    VerilatorRuntimeOptions,
};

pub type TestResult<T = ()> = Result<T, Box<dyn Error>>;

/// Load a line-oriented RTL manifest relative to the gateware directory.
pub fn production_rtl_sources(gateware_root: &Path, manifest: &Path) -> TestResult<Vec<PathBuf>> {
    let manifest_path = gateware_root.join(manifest);
    let contents = fs::read_to_string(&manifest_path)?;
    let mut sources = Vec::new();
    let mut unique = HashSet::new();
    for line in contents.lines().map(str::trim) {
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let source = gateware_root.join(line);
        if !unique.insert(source.clone()) {
            return Err(format!("duplicate entry in {}: {line}", manifest_path.display()).into());
        }
        if !source.is_file() {
            return Err(format!("required RTL source is missing: {}", source.display()).into());
        }
        sources.push(source);
    }
    if sources.is_empty() {
        return Err(format!("RTL manifest {} is empty", manifest_path.display()).into());
    }
    Ok(sources)
}

pub fn model_config() -> VerilatedModelConfig {
    VerilatedModelConfig {
        verilator_optimization: 1,
        ..VerilatedModelConfig::default()
    }
}

fn fingerprint_bytes(hash: &mut u64, bytes: &[u8]) {
    const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;
    for &byte in bytes {
        *hash ^= u64::from(byte);
        *hash = hash.wrapping_mul(FNV_PRIME);
    }
}

// Marlin reuses an artifact directory verbatim, so a human-readable namespace
// alone can silently run an old model after an RTL edit.  Include every direct
// compilation input and the generated wrapper contract in the directory key.
fn model_fingerprint(
    sources: &[&Path],
    include_dirs: &[&Path],
    top_module: &str,
    top_source: &Path,
    ports: &[(&str, usize, usize, PortDirection)],
) -> TestResult<u64> {
    let mut hash = 0xcbf2_9ce4_8422_2325;
    fingerprint_bytes(&mut hash, top_module.as_bytes());
    fingerprint_bytes(&mut hash, top_source.as_os_str().as_encoded_bytes());
    fingerprint_bytes(&mut hash, &fs::read(top_source)?);
    for &(name, msb, lsb, direction) in ports {
        fingerprint_bytes(&mut hash, name.as_bytes());
        fingerprint_bytes(&mut hash, &msb.to_le_bytes());
        fingerprint_bytes(&mut hash, &lsb.to_le_bytes());
        fingerprint_bytes(&mut hash, format!("{direction:?}").as_bytes());
    }
    for directory in include_dirs {
        fingerprint_bytes(&mut hash, directory.as_os_str().as_encoded_bytes());
        // Textual includes are not compilation units, but changing one must
        // still invalidate Marlin's cached model.  Hash include files in a
        // stable order alongside the direct source list.
        let mut include_files = fs::read_dir(directory)?
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .filter(|path| path.extension().is_some_and(|ext| ext == "svh"))
            .collect::<Vec<_>>();
        include_files.sort();
        for include in include_files {
            fingerprint_bytes(&mut hash, include.as_os_str().as_encoded_bytes());
            fingerprint_bytes(&mut hash, &fs::read(include)?);
        }
    }
    for source in sources {
        fingerprint_bytes(&mut hash, source.as_os_str().as_encoded_bytes());
        fingerprint_bytes(&mut hash, &fs::read(source)?);
    }
    Ok(hash)
}

/// Build a dynamic model from a production closure and keep its runtime alive
/// for the supplied test callback.
#[allow(clippy::too_many_arguments)]
pub fn with_model<T>(
    gateware_root: &Path,
    manifest: &Path,
    artifact_namespace: &str,
    top_module: &str,
    top_source: &Path,
    ports: &[(&str, usize, usize, PortDirection)],
    include_dirs: &[&Path],
    extra_sources: &[&Path],
    test: impl for<'ctx> FnOnce(&mut DynamicVerilatedModel<'ctx>) -> TestResult<T>,
) -> TestResult<T> {
    let sources = production_rtl_sources(gateware_root, manifest)?;
    let mut source_refs = Vec::with_capacity(extra_sources.len() + sources.len());
    for source in extra_sources {
        if !source.is_file() {
            return Err(
                format!("required RTL test adapter is missing: {}", source.display()).into(),
            );
        }
        source_refs.push(*source);
    }
    source_refs.extend(sources.iter().map(PathBuf::as_path));
    let includes = include_dirs
        .iter()
        .map(|path| gateware_root.join(path))
        .collect::<Vec<_>>();
    let include_refs = includes.iter().map(PathBuf::as_path).collect::<Vec<_>>();
    let top_source = gateware_root.join(top_source);
    let fingerprint =
        model_fingerprint(&source_refs, &include_refs, top_module, &top_source, ports)?;
    let artifact_directory = gateware_root
        .join("target/marlin")
        .join(format!("{artifact_namespace}-{fingerprint:016x}"));
    let runtime = VerilatorRuntime::new(
        &artifact_directory,
        &source_refs,
        &include_refs,
        [],
        VerilatorRuntimeOptions::default(),
    )?;
    let mut model = runtime.create_dyn_model(
        top_module,
        top_source
            .to_str()
            .ok_or("top source path is not valid UTF-8")?,
        ports,
        model_config(),
    )?;
    test(&mut model)
}

/// Build a dynamic model from an explicit list of RTL compilation units.
///
/// This is intended for focused leaf regressions whose compilation closure is
/// smaller than a complete production manifest. All paths in `sources` are
/// relative to the gateware directory; `extra_sources` follows [`with_model`] and
/// contains already-resolved paths such as Rust-test adapters.
#[allow(clippy::too_many_arguments)]
pub fn with_model_sources<T>(
    gateware_root: &Path,
    sources: &[&Path],
    artifact_namespace: &str,
    top_module: &str,
    top_source: &Path,
    ports: &[(&str, usize, usize, PortDirection)],
    include_dirs: &[&Path],
    extra_sources: &[&Path],
    test: impl for<'ctx> FnOnce(&mut DynamicVerilatedModel<'ctx>) -> TestResult<T>,
) -> TestResult<T> {
    let mut resolved_sources = Vec::with_capacity(sources.len());
    let mut unique = HashSet::new();
    for source in sources {
        let resolved = gateware_root.join(source);
        if !unique.insert(resolved.clone()) {
            return Err(format!("duplicate RTL source: {}", resolved.display()).into());
        }
        if !resolved.is_file() {
            return Err(format!("required RTL source is missing: {}", resolved.display()).into());
        }
        resolved_sources.push(resolved);
    }
    if resolved_sources.is_empty() {
        return Err("explicit RTL source list is empty".into());
    }

    let mut source_refs = Vec::with_capacity(extra_sources.len() + resolved_sources.len());
    for source in extra_sources {
        if !source.is_file() {
            return Err(
                format!("required RTL test adapter is missing: {}", source.display()).into(),
            );
        }
        if !unique.insert((*source).to_path_buf()) {
            return Err(format!("duplicate RTL source: {}", source.display()).into());
        }
        source_refs.push(*source);
    }
    source_refs.extend(resolved_sources.iter().map(PathBuf::as_path));

    let includes = include_dirs
        .iter()
        .map(|path| gateware_root.join(path))
        .collect::<Vec<_>>();
    let include_refs = includes.iter().map(PathBuf::as_path).collect::<Vec<_>>();
    let top_source = gateware_root.join(top_source);
    let fingerprint =
        model_fingerprint(&source_refs, &include_refs, top_module, &top_source, ports)?;
    let artifact_directory = gateware_root
        .join("target/marlin")
        .join(format!("{artifact_namespace}-{fingerprint:016x}"));
    let runtime = VerilatorRuntime::new(
        &artifact_directory,
        &source_refs,
        &include_refs,
        [],
        VerilatorRuntimeOptions::default(),
    )?;
    let mut model = runtime.create_dyn_model(
        top_module,
        top_source
            .to_str()
            .ok_or("top source path is not valid UTF-8")?,
        ports,
        model_config(),
    )?;
    test(&mut model)
}

pub fn read_u64(model: &DynamicVerilatedModel<'_>, port: &str) -> TestResult<u64> {
    Ok(match model.read(port)? {
        VerilatorValue::CData(value) => u64::from(value),
        VerilatorValue::SData(value) => u64::from(value),
        VerilatorValue::IData(value) => u64::from(value),
        VerilatorValue::QData(value) => value,
        VerilatorValue::WDataInP(_) | VerilatorValue::WDataOutP(_) => {
            return Err(format!("port {port} unexpectedly uses a wide value").into());
        }
    })
}

pub fn clock(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("clk", 0_u8)?;
    model.eval();
    model.pin("clk", 1_u8)?;
    model.eval();
    model.pin("clk", 0_u8)?;
    model.eval();
    Ok(())
}

pub fn sign_extend(value: u64, width: usize) -> i64 {
    assert!((1..=64).contains(&width));
    if width == 64 {
        value as i64
    } else {
        let shift = 64 - width;
        ((value << shift) as i64) >> shift
    }
}
