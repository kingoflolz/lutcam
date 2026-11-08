"""Exhaust all binary windows of the maintained OV9281 B8 decoder RTL.

The bench compares the live aligner with an independent exact-onehot case
oracle. No archived RTL or build-directory baseline is required.
"""

from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[2]


def test_b8_decoder_all_binary_windows(tmp_path):
    verilator = shutil.which("verilator")
    assert verilator is not None, "Verilator is required for the exhaustive B8 RTL regression"
    command = [
        verilator, "--binary", "--timing", "--assert", "--Wall",
        "--timescale", "1ns/1ps", "-j", "2",
        "--top-module", "csi_b8_decoder_tb", "--Mdir", str(tmp_path / "obj"),
        str(ROOT / "rtl/csi2_rx/Ov9281LaneAligner.sv"),
        str(Path(__file__).with_name("csi_b8_decoder_tb.sv")),
    ]
    compiled = subprocess.run(command, capture_output=True, text=True, timeout=120)
    (tmp_path / "compile.log").write_text(compiled.stdout + compiled.stderr)
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr
    result = subprocess.run(
        [str(tmp_path / "obj/Vcsi_b8_decoder_tb")],
        capture_output=True, text=True, timeout=30,
    )
    (tmp_path / "run.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS B8 binary windows=65536 unmatched=63488 unique=2048 offsets=ff" in result.stdout
