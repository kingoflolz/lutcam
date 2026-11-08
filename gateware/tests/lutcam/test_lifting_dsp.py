"""Native lifting registers retain arithmetic, cycle, hold, and async-reset semantics."""
from pathlib import Path
import re
import shutil
import subprocess

import pytest
from litex.build.efinix.toolchain import find_efinity_path


ROOT = Path(__file__).resolve().parents[2]
FIXTURES = Path(__file__).with_name("lifting_dsp")


@pytest.mark.parametrize("simulator", ["verilator", "iverilog"])
def test_native_lifting_signed_wrap_hold_and_async_reset(tmp_path, simulator):
    if shutil.which(simulator) is None:
        pytest.skip(f"{simulator} required")
    try:
        model = Path(find_efinity_path()) / "sim_models/verilog/efx_dsp48.v"
    except OSError:
        pytest.skip("Set LITEX_ENV_EFINITY to validate the native DSP model")
    assert model.is_file()
    source = (ROOT / "rtl/lwc/horizontal_lift_stage.sv").read_text()
    source = source.replace("module horizontal_lift_stage #(",
                            "module horizontal_lift_stage_candidate #(", 1)
    candidate = tmp_path / "horizontal_lift_stage_candidate.sv"
    candidate.write_text(source)
    sources = [str(FIXTURES / "leaf_parity_tb.sv"),
               str(FIXTURES / "horizontal_lift_stage_baseline.sv"),
               str(candidate), str(ROOT / "rtl/lwc/LwcRegisteredAddSub.sv"), str(model)]
    # Explicit SYNTHESIS selects the vendor branch and disables the reference
    # stage's intentional overflow assertions: this test checks modulo wrapping.
    if simulator == "verilator":
        command = ["verilator", "--binary", "--timing", "--assert", "-DSYNTHESIS",
                   "-Wno-fatal", "--top-module", "leaf_parity_tb", "--Mdir",
                   str(tmp_path / "obj"), "-j", "4", *sources]
        run = [str(tmp_path / "obj/Vleaf_parity_tb")]
    else:
        command = ["iverilog", "-g2012", "-DSYNTHESIS", "-s", "leaf_parity_tb",
                   "-o", str(tmp_path / "parity.vvp"), *sources]
        run = ["vvp", str(tmp_path / "parity.vvp")]
    result = subprocess.run(command, capture_output=True, text=True)
    (tmp_path / "compile.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    result = subprocess.run(run, capture_output=True, text=True, timeout=60)
    (tmp_path / "run.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
    coverage = [dict((key, int(value)) for key, value in re.findall(r"(\w+)=(\d+)", line))
                for line in result.stdout.splitlines() if line.startswith("RESULT ")]
    assert sorted(row["width"] for row in coverage) == [9, 10, 14, 18]
    for row in coverage:
        assert row["rows"] == 400
        assert row["one_pair_rows"] > 0 and row["odd_pair_rows"] > 0
        assert row["high_wraps"] > 0 and row["low_wraps"] > 0
        assert row["nonzero_held_cycles"] > 0 and row["async_checks"] == 17
        assert row["active_resets"] == 8 and row["idle_resets"] == 9
    assert "PASS horizontal DSP leaf parity: all four widths" in result.stdout
