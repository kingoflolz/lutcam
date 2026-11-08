"""The current corner subtract helper agrees with signed integer subtraction.

The entire helper is extracted unchanged from the RTL under test and simulated
against Efinix's actual DSP48 model. The C++ oracle uses mathematical signed
integers, checks the full 48-bit DSP result as well as the truncated port, and
never clocks the DUT: every assertion therefore checks combinational behavior.
LUTCAM_CORNER_SOURCE selects a source override.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

import pytest
from litex.build.efinix.toolchain import find_efinity_path


ROOT = next(p for p in Path(__file__).resolve().parents
            if (p / "rtl/lftag/LfTagCornerSelect9.sv").is_file())
SOURCE = Path(os.environ.get("LUTCAM_CORNER_SOURCE",
                            ROOT / "rtl/lftag/LfTagCornerSelect9.sv"))
DRIVER = Path(__file__).with_name("corner_subtract_native.cpp")


@pytest.mark.skipif(shutil.which("verilator") is None, reason="Verilator required")
def test_native_corner_subtract_full_domains_and_random_pairs(tmp_path):
    try:
        model = Path(find_efinity_path()) / "sim_models/verilog/efx_dsp48.v"
    except OSError:
        pytest.skip("Efinity DSP48 model required")
    assert model.is_file()
    source_text = SOURCE.read_text()
    helpers = re.findall(r"(?ms)^module LfTagCornerSubtract\b.*?^endmodule\b", source_text)
    assert len(helpers) == 1, "The current complete subtract helper must be present exactly once"
    helper = tmp_path / "subject.sv"
    helper.write_text(helpers[0] + "\n")
    # SYNTHESIS selects the actual production DSP branch; the portable fallback
    # is not used to construct either the DUT or the independent oracle.
    top = tmp_path / "CornerSubtractProbe.sv"
    lines = ["module CornerSubtractProbe (",
             ",\n".join(f"input wire [{w-1}:0] a{w}, b{w}, "
                         f"output wire [{w-1}:0] r{w}, output wire [47:0] full{w}"
                         for w in (15, 16, 18)), ");"]
    for width in (15, 16, 18):
        lines += [f"LfTagCornerSubtract #(.WIDTH({width})) sub{width} ("
                  f".a(a{width}), .b(b{width}), .result(r{width}));",
                  f"assign full{width} = sub{width}.native.difference;"]
    lines.append("endmodule")
    top.write_text("\n".join(lines) + "\n")
    command = ["verilator", "--cc", "--exe", "--build", "--timing", "--assert",
               "-DSYNTHESIS", "-Wno-fatal", "--top-module", "CornerSubtractProbe",
               "--Mdir", str(tmp_path / "obj"), "-j", "2", str(top), str(helper),
               str(model), str(DRIVER)]
    built = subprocess.run(command, capture_output=True, text=True, timeout=180)
    (tmp_path / "compile.log").write_text(built.stdout + built.stderr)
    assert built.returncode == 0, built.stdout[-4000:] + built.stderr[-4000:]
    run = subprocess.run([str(tmp_path / "obj/VCornerSubtractProbe")],
                         capture_output=True, text=True, timeout=120)
    (tmp_path / "run.log").write_text(run.stdout + run.stderr)
    assert run.returncode == 0, run.stdout + run.stderr
    rows = [json.loads(line[len("RESULT "):]) for line in run.stdout.splitlines()
            if line.startswith("RESULT ")]
    assert [r["width"] for r in rows] == [15, 16, 18]
    for row in rows:
        assert row["exhaustive_a_values"] == 1 << row["width"]
        assert row["exhaustive_b_values"] == 1 << row["width"]
        assert row["random_pairs"] == 750_000
        assert row["directed_pairs"] > 1_000
        assert row["positive_wraps"] > 0 and row["negative_wraps"] > 0
        assert all(count > 0 for count in row["sign_quadrants"])
        assert row["clock_edges"] == 0
    report = {"source": str(SOURCE),
              "source_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              "extracted_helper_sha256": hashlib.sha256(helper.read_bytes()).hexdigest(),
              "vendor_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
              "driver_sha256": hashlib.sha256(DRIVER.read_bytes()).hexdigest(),
              "coverage": rows, "total_pairs": sum(r["pairs"] for r in rows),
              "result": "pass"}
    (tmp_path / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(run.stdout)
    print(json.dumps(report, sort_keys=True))
