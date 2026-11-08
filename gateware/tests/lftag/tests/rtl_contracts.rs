//! Structural and arithmetic contracts for the LFTag production RTL closure.

use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::process::Command;

use rtl_test_support::production_rtl_sources;

const MANIFEST: &str = "rtl/lftag/sources.f";
const TOP: &str = "LfTagCompact3x3DecoderLocalizer";

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

#[test]
fn production_manifest_contains_every_lftag_module_exactly_once() {
    let root = gateware_root();
    let sources = production_rtl_sources(&root, Path::new(MANIFEST)).unwrap();
    let production_directory = root.join("rtl/lftag");
    let mut manifest_sources = HashSet::new();

    assert!(
        sources
            .iter()
            .any(|source| source.file_name().and_then(|name| name.to_str())
                == Some("LfTagCompact3x3DecoderLocalizer.sv")),
        "{MANIFEST} omits the standalone production top",
    );
    for source in sources {
        assert_eq!(
            source.parent(),
            Some(production_directory.as_path()),
            "non-LFTag source leaked into the standalone closure: {}",
            source.display(),
        );
        assert_eq!(
            source.extension().and_then(|extension| extension.to_str()),
            Some("sv"),
            "production source is not SystemVerilog: {}",
            source.display(),
        );
        assert!(
            manifest_sources.insert(source),
            "manifest contains a duplicate source"
        );
    }

    let directory_sources = std::fs::read_dir(&production_directory)
        .expect("read rtl/lftag")
        .map(|entry| entry.expect("read rtl/lftag entry").path())
        .filter(|path| path.extension().and_then(|extension| extension.to_str()) == Some("sv"))
        .collect::<HashSet<_>>();
    assert_eq!(
        manifest_sources, directory_sources,
        "every LFTag RTL module must belong to the production manifest",
    );
}

fn lint(synthesis: bool) {
    let root = gateware_root();
    let sources = production_rtl_sources(&root, Path::new(MANIFEST)).unwrap();
    let mut command = Command::new("verilator");
    command
        .current_dir(&root)
        .args(["--lint-only", "--Wall", "--top-module", TOP])
        .arg("-Irtl/lftag");
    if synthesis {
        command.arg("-DSYNTHESIS");
    }
    command.args(&sources);
    let output = command.output().expect("failed to launch Verilator");
    assert!(
        output.status.success(),
        "{} lint failed:\n{}{}",
        if synthesis { "SYNTHESIS" } else { "normal" },
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
}

#[test]
fn production_rtl_is_warning_free_in_normal_and_synthesis_modes() {
    lint(false);
    lint(true);
}
