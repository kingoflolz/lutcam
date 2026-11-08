use std::path::{Path, PathBuf};
use std::process::Command;
use std::{env, fs};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn rtl() -> String {
    fs::read_to_string(root().join("rtl/isp/MonoIsp.sv")).unwrap()
}

#[test]
fn isp_uses_only_the_three_budgeted_block_ram_arrays() {
    let source = rtl();
    assert_eq!(
        source.matches("ram_style").count(),
        3,
        "each storage array must have one explicit block-RAM inference attribute"
    );
    for declaration in [
        "logic [7:0] tone_lut [0:1023]",
        "logic [19:0] defect_table [0:511]",
        "logic [19:0] histogram [0:511]",
    ] {
        assert_eq!(
            source.matches(declaration).count(),
            1,
            "missing or duplicated storage array `{declaration}`"
        );
    }
    for retired in ["black_level", "DEFECT_HIGH_START", "defect_high_start"] {
        assert!(
            !source.contains(retired),
            "retired separate-stage state `{retired}` remains"
        );
    }

    // The declaration has whitespace before `[`, while every canonical RAM
    // access deliberately does not.  Lock the port-sharing topology so a
    // future convenience read/write cannot silently replicate a BRAM or add
    // another procedural owner.
    for (array, access_sites) in [("tone_lut", 3), ("defect_table", 2), ("histogram", 2)] {
        assert_eq!(
            source.matches(&format!("{array}[")).count(),
            access_sites,
            "unexpected number of canonical access sites for `{array}`"
        );
    }
    for array in ["tone_lut", "defect_table", "histogram"] {
        assert_eq!(
            source
                .split("always_ff @(posedge clk)")
                .skip(1)
                .filter(|owner| owner.contains(&format!("{array}[")))
                .count(),
            1,
            "each RAM must retain one sequential owner: {array}"
        );
    }
    assert_eq!(
        source.matches("tone_port0_write_enable").count(),
        4,
        "tone LUT software-write control must remain on port zero"
    );
    assert!(
        !source.contains("tone_port1_write_enable"),
        "tone LUT software writes must not use port one"
    );
}

fn lint(synthesis: bool) {
    let root = root();
    let manifest = fs::read_to_string(root.join("rtl/isp/sources.f")).unwrap();
    let sources = manifest
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .collect::<Vec<_>>();
    assert!(!sources.is_empty());

    let mut command = Command::new("verilator");
    command
        .current_dir(&root)
        .args(["--lint-only", "--Wall", "--top-module", "MonoIsp"])
        .arg("-Irtl/isp");
    if synthesis {
        let Some(efinity) = env::var_os("LITEX_ENV_EFINITY") else {
            assert_ne!(
                env::var("LUTCAM_REQUIRE_EFINIX_NATIVE").as_deref(),
                Ok("1"),
                "required native ISP lint needs LITEX_ENV_EFINITY"
            );
            eprintln!("native ISP lint skipped: set LITEX_ENV_EFINITY");
            return;
        };
        let model = PathBuf::from(efinity).join("sim_models/verilog/efx_dsp48.v");
        assert!(model.is_file(), "missing vendor model: {}", model.display());
        command
            .arg("-DSYNTHESIS")
            .arg(root.join("tests/lutcam/efinity_dsp_model.vlt"))
            .arg(model);
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
