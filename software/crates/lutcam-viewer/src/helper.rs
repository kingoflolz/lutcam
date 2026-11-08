//! Locate the board helpers used by capture control and discovery.

use std::{path::PathBuf, process::Command};

pub(crate) fn command(module: &str) -> Command {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../gateware");
    let python = root.join(".venv/bin/python");
    let mut command = Command::new(if python.is_file() {
        python
    } else {
        PathBuf::from("python3")
    });
    command.current_dir(root).args(["-u", "-m", module]);
    command
}
