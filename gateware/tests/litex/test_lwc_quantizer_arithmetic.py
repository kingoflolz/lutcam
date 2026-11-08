"""Full-domain quantizer arithmetic, including the real combinational DSP path."""
from pathlib import Path
import hashlib
import os
import shutil
import subprocess

import pytest

ROOT = next(path for path in Path(__file__).resolve().parents
            if (path / "rtl/lwc/coefficient_quantizer.sv").exists())


@pytest.mark.parametrize("native", [False, True], ids=["portable", "native-dsp"])
def test_all_signed14_values_and_lane_masks(tmp_path, native):
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
        pytest.skip("Set EFINITY_HOME or LITEX_ENV_EFINITY to test the native DSP48 model")
    (tmp_path / "tb.sv").write_text(TB)
    inputs = [ROOT / "rtl/lwc/codec_profile_pkg.sv",
              ROOT / "rtl/lwc/coefficient_quantizer.sv", tmp_path / "tb.sv"]
    if native:
        inputs.append(vendor)
    hashes = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}
    # Keep design lint strict. Only vendor-owned model diagnostics are waived;
    # no rule is suppressed on the quantizer or its package.
    lint_sources = inputs[:2]
    lint_command = ["verilator", "--lint-only", "--timing", "-Wall", "-Wpedantic",
                    "--top-module", "coefficient_quantizer"]
    if native:
        config = tmp_path / "vendor-lint.vlt"
        config.write_text("`verilator_config\n" + "".join(
            f'lint_off -rule {rule} -file "{vendor}"\n'
            for rule in ["DECLFILENAME", "GENUNNAMED", "VARHIDDEN", "DEFPARAM",
                         "WIDTHEXPAND", "UNUSEDPARAM", "PROCASSINIT", "UNUSEDSIGNAL"]))
        lint_command += ["-DSYNTHESIS", str(config)]
        lint_sources = lint_sources + [vendor]
    linted = subprocess.run(lint_command + [str(path) for path in lint_sources],
                            text=True, capture_output=True)
    assert linted.returncode == 0, linted.stdout + linted.stderr
    # Illegal masks exercise the sticky error output without the diagnostic
    # coloring assertion aborting simulation. Oracle failures use $fatal.
    command = ["verilator", "--binary", "--timing", "--top-module", "tb",
               "--timescale", "1ns/1ps", "-Wno-fatal", "-j", "2",
               "-DLUTCAM_TEST_ALLOW_QUANTIZER_CONFLICTS",
               "--Mdir", str(tmp_path / "obj")]
    if native:
        command += ["-DSYNTHESIS"]
    command += [str(path) for path in inputs]
    compiled = subprocess.run(command, text=True, capture_output=True)
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr
    result = subprocess.run([str(tmp_path / "obj/Vtb")], text=True, capture_output=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS quantizer cycles=229378" in result.stdout
    assert all(hashlib.sha256(path.read_bytes()).hexdigest() == expected
               for path, expected in hashes.items()), "Input changed during validation"
    print("native" if native else "portable", result.stdout.strip())


TB = r"""
module tb;
logic clk=0,rst=1,clear_frame=0;
logic[11:0]in_valid=0;
logic signed[11:0][13:0]in_value='0;
wire[11:0]out_valid,out_correction,out_residual_class;
wire signed[11:0][7:0]out_q;
wire protocol_error;
coefficient_quantizer dut(.*);
integer checks=0;
bit expected_error=0;
localparam integer SHIFTS[12]='{1,1,2,2,2,3,3,3,4,4,4,4};
localparam integer FINE_BIAS[12]='{0,0,0,0,1,3,2,3,6,6,6,6};
localparam integer COARSE_BIAS[12]='{1,1,3,3,3,7,7,7,15,15,17,15};
localparam integer COLORS[12]='{3,4,3,3,3,4,0,0,1,0,1,2};
function automatic integer magnitude(input integer value);
 magnitude=value<0 ? -value : value;
endfunction
function automatic integer quantize(input integer value,shift,bias);
 integer rounded;
 rounded=(magnitude(value)+bias)>>shift;
 if(value<0)rounded=-rounded;
 quantize=rounded < -128 ? -128 : rounded > 127 ? 127 : rounded;
endfunction
task automatic check;
 integer selected[5],count[5],band,value,fine,coarse,error,shift;
 bit correction,residual,conflict;
 #1;
 // The last member of each color remains observable even without valid.
 selected[0]=9;selected[1]=10;selected[2]=11;selected[3]=4;selected[4]=5;
 for(integer color=0;color<5;color++)count[color]=0;
 for(integer index=11;index>=0;index--)begin
  if(in_valid[index])begin
   selected[COLORS[index]]=index;
   count[COLORS[index]]++;
  end
 end
 if(out_valid!==in_valid)$fatal(1,"valid changed");
 if(protocol_error!==expected_error)$fatal(1,"error changed before clock");
 for(integer output_band=0;output_band<12;output_band++)begin
  band=selected[COLORS[output_band]];
  value=$signed(in_value[band]);shift=SHIFTS[band];
  fine=quantize(value,shift,FINE_BIAS[band]);
  coarse=quantize(value,shift+1,COARSE_BIAS[band]);
  correction=(fine!=0)&&(magnitude(coarse)>(magnitude(fine)>>1));
  error=value-(fine<<shift);
  residual=(magnitude(fine)&1) ? magnitude(error)*4 >= (1<<shift) : fine*error>=0;
  if($signed(out_q[output_band])!=fine || out_correction[output_band]!==correction ||
     out_residual_class[output_band]!==residual)
   $fatal(1,"oracle mismatch cycle=%0d output=%0d selected=%0d value=%0d q=%0d/%0d correction=%b/%b residual=%b/%b",
    checks,output_band,band,value,$signed(out_q[output_band]),fine,
    out_correction[output_band],correction,out_residual_class[output_band],residual);
 end
 conflict=0;
 for(integer color=0;color<5;color++)if(count[color]>1)conflict=1;
 if(rst||clear_frame)expected_error=0;
 else if(conflict)expected_error=1;
 clk=1;#1;
 if(protocol_error!==expected_error)$fatal(1,"sticky error mismatch");
 clk=0;checks++;
endtask
initial begin
 check();rst=0;check();
 for(integer value=-8192;value<8192;value++)begin
  for(integer band=0;band<12;band++)begin
   in_valid=12'b1<<band;
   for(integer lane=0;lane<12;lane++)in_value[lane]=14'(value^(lane*13));
   in_value[band]=14'(value);check();
  end
 end
 for(integer pass=0;pass<8;pass++)begin
  for(integer mask=0;mask<4096;mask++)begin
   in_valid=12'(mask);
   for(integer lane=0;lane<12;lane++)
    in_value[lane]=14'((mask*1291+lane*1399+pass*3911)^(mask>>3));
   rst=(mask%511==0);clear_frame=(mask%127==0);check();
  end
 end
 if(checks!=229378)$fatal(1,"incomplete coverage");
 $display("PASS quantizer cycles=%0d all signed14 values x12 bands, all4096masks x8 patterns",checks);
 $finish;
end
endmodule
"""
