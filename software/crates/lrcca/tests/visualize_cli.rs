use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../gateware")
}

#[test]
fn tag_detect_visualizer_writes_png_and_gif() {
    let output_root = std::env::temp_dir().join(format!(
        "lutcam-tag-detect-visualize-{}",
        std::process::id()
    ));
    fs::create_dir_all(&output_root).unwrap();
    let png = output_root.join("centroids.png");
    let gif = output_root.join("groups.gif");
    let status = Command::new(env!("CARGO_BIN_EXE_tag_detect_visualize"))
        .arg(gateware_root().join("fixtures/lrcca/test12.png"))
        .args(["--width", "32", "--height", "16", "--output"])
        .arg(&png)
        .arg("--gif-output")
        .arg(&gif)
        .status()
        .unwrap();
    assert!(status.success());
    assert!(fs::read(&png).unwrap().starts_with(b"\x89PNG\r\n\x1a\n"));
    assert!(fs::read(&gif).unwrap().starts_with(b"GIF89a"));
}
