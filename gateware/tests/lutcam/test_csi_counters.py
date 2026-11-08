"""CSI counters preserve wrap, clear priority, holds, and asynchronous reset."""
from pathlib import Path
import re
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("simulator", ["verilator", "iverilog"])
@pytest.mark.parametrize("native", [False, True])
def test_csi_counter_portable_and_native(tmp_path, simulator, native):
    if shutil.which(simulator) is None:
        pytest.skip(f"{simulator} required")
    sources = [str(Path(__file__).with_name("csi_counter_tb.sv")),
               str(ROOT / "rtl/csi2_rx/Ov9281Counter.sv")]
    defines = []
    if native:
        from litex.build.efinix.toolchain import find_efinity_path
        try:
            model = Path(find_efinity_path()) / "sim_models/verilog/efx_dsp48.v"
        except OSError:
            pytest.skip("Set LITEX_ENV_EFINITY to validate the native DSP model")
        assert model.is_file()
        sources.append(str(model))
        defines = ["-DSYNTHESIS"]
    if simulator == "verilator":
        command = ["verilator", "--binary", "--timing", "--assert", "-Wno-fatal",
                   *defines, "--top-module", "csi_counter_tb", "--Mdir",
                   str(tmp_path / "obj"), "-j", "2", *sources]
        run = [str(tmp_path / "obj/Vcsi_counter_tb")]
    else:
        command = ["iverilog", "-g2012", *defines, "-s", "csi_counter_tb",
                   "-o", str(tmp_path / "test.vvp"), *sources]
        run = ["vvp", str(tmp_path / "test.vvp")]
    result = subprocess.run(command, capture_output=True, text=True, timeout=120)
    (tmp_path / "compile.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    result = subprocess.run(run, capture_output=True, text=True, timeout=120)
    (tmp_path / "run.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
    rows = [dict((key, int(value)) for key, value in re.findall(r"(\w+)=(\d+)", line))
            for line in result.stdout.splitlines() if line.startswith("RESULT ")]
    assert sorted(row["width"] for row in rows) == [7, 10, 14, 19, 20, 32]
    for row in rows:
        assert row["cycles"] > 524288
        assert row["clears"] > 0 and row["holds"] > 0
        assert row["async_checks"] == 17
        assert row["sync_clear_checks"] > 0
        if row["width"] <= 19:
            assert row["wraps"] > 0
    assert "PASS CSI counters" in result.stdout
