"""Exhaustive token placement, including the actual combinational DSP path."""
from pathlib import Path
import hashlib
import os
import shutil
import subprocess

import pytest

ROOT = next(path for path in Path(__file__).resolve().parents
            if (path / "rtl/lwc/causal_group_encoder.sv").exists())
SOURCE = ROOT / "rtl/lwc/causal_group_encoder.sv"


@pytest.mark.parametrize("native", [False, True], ids=["portable", "native-dsp"])
def test_all_tokens_and_six_bit_offsets_match_full_width_shift(tmp_path, native):
    required = os.environ.get("LUTCAM_REQUIRE_EFINIX_NATIVE") == "1"
    if not shutil.which("verilator"):
        if required:
            pytest.fail("Required Verilator executable is missing")
        pytest.skip("Verilator unavailable")
    efinity = os.environ.get("EFINITY_HOME") or os.environ.get("LITEX_ENV_EFINITY")
    vendor = Path(efinity) / "sim_models/verilog/efx_dsp48.v" if efinity else None
    if native and (vendor is None or not vendor.is_file()):
        if required:
            pytest.fail("Required Efinity DSP48 model is missing")
        pytest.skip("Set EFINITY_HOME to test the native DSP48 model")
    source = SOURCE.read_text()
    start = source.index("    wire [15:0] append_scale")
    end = source.index("\n    // Packet tokens", start)
    block = source[start:end]
    wrapper = """module append_probe(input wire clk,input wire[16:0]token_bits,
input wire[5:0]output_count_after,output wire[45:0]actual,expected);
localparam OUTPUT_RESERVOIR_BITS=46;
""" + block + """
assign actual=append_bits;
assign expected={token_bits,29'd0}>>output_count_after;
endmodule
"""
    (tmp_path / "append_probe.sv").write_text(wrapper)
    (tmp_path / "tb.sv").write_text(TB)
    inputs = [SOURCE, tmp_path / "append_probe.sv", tmp_path / "tb.sv"]
    if native:
        inputs.append(vendor)
    hashes = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}
    command = ["verilator", "--binary", "--timing", "--assert", "--top-module", "tb",
               "-Wno-fatal", "-j", "2", "--Mdir", str(tmp_path / "obj")]
    if native:
        command += ["-DSYNTHESIS", str(vendor)]
    command += [str(tmp_path / "append_probe.sv"), str(tmp_path / "tb.sv")]
    compiled = subprocess.run(command, text=True, capture_output=True)
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr
    result = subprocess.run([str(tmp_path / "obj/Vtb")], text=True, capture_output=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS exhaustive append cases=8388608" in result.stdout
    assert all(hashlib.sha256(path.read_bytes()).hexdigest() == expected
               for path, expected in hashes.items()), "Input changed during native validation"
    print("native" if native else "portable", result.stdout.strip())


TB = r"""
module tb;
logic clk=0;
logic[16:0]token_bits=0;
logic[5:0]output_count_after=0;
wire[45:0]actual,expected;
append_probe dut(.*);
integer value,count;
longint unsigned cases=0;
initial begin
for(value=0;value<131072;value=value+1)begin
 token_bits=17'(value);
 for(count=0;count<64;count=count+1)begin
  output_count_after=6'(count);clk=~clk;#1;
  if(actual!==expected)$fatal(1,"token=%h count=%0d expected=%h actual=%h",token_bits,count,expected,actual);
  if(count>=46&&actual!==0)$fatal(1,"large offset must clear result");
  cases=cases+1;
 end
end
if(cases!=8388608)$fatal(1,"incomplete exhaustive coverage");
$display("PASS exhaustive append cases=%0d",cases);$finish;
end endmodule
"""
