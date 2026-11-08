"""Native DSP carry/borrow boundaries, independent of fitter fixtures."""
from pathlib import Path
import pytest
import os,subprocess,tempfile
ROOT=Path(__file__).resolve().parents[3]

def test_arithmetic():
    efinity = os.environ.get("LITEX_ENV_EFINITY") or os.environ.get("EFINITY_HOME")
    vendor = Path(efinity) / "sim_models/verilog/efx_dsp48.v" if efinity else None
    if vendor is None or not vendor.is_file():
        message = "Set EFINITY_HOME or LITEX_ENV_EFINITY for the native DSP48 model"
        if os.environ.get("LUTCAM_REQUIRE_EFINIX_NATIVE") == "1":
            pytest.fail(message)
        pytest.skip(message)
    with tempfile.TemporaryDirectory(prefix='lftag-wide-') as td:
        d=Path(td)
        (d/'tb.sv').write_text('''module tb;
logic signed[39:0] a,b;logic subtract;wire signed[39:0] r40;wire signed[26:0] r27;wire signed[20:0] r21;
LfTagWideAddSub #(.WIDTH(40)) w40(.a,.b,.subtract,.result(r40));
LfTagWideAddSub #(.WIDTH(27)) w27(.a(a[26:0]),.b(b[26:0]),.subtract,.result(r27));
LfTagWideAddSub #(.WIDTH(21)) w21(.a(a[20:0]),.b(b[20:0]),.subtract,.result(r21));
initial begin
 for(integer n=0;n<20000;n++)begin
  a={$urandom,$urandom};b={$urandom,$urandom};subtract=n[0];
  case(n%16)
   0:a=0;1:a=-1;2:b=1;3:b=-1;4:a=40'h7fffffffff;
   5:a=40'h8000000000;6:b=40'h0000ffff;7:b=40'hffff0000;
   default:begin end
  endcase
  #1;
  if(r40!==(subtract?40'(a-b):40'(a+b)))$fatal(1,"40-bit DSP arithmetic mismatch");
  if(r21!==(subtract?21'(a-b):21'(a+b)))$fatal(1,"21-bit DSP arithmetic mismatch");
  if(r27!==(subtract?27'(a-b):27'(a+b)))$fatal(1,"27-bit DSP arithmetic mismatch");
 end
 $display("PASS 20000 native wide add/subtract carry and borrow cases");$finish;
end
endmodule
''')
        r=subprocess.run(['verilator','--binary','--timing','--assert','-Wno-fatal','-DSYNTHESIS','--top-module','tb','--Mdir',str(d/'obj'),str(vendor),str(ROOT/'rtl/lftag/LfTagWideAddSub.sv'),str(d/'tb.sv')],capture_output=True,text=True,timeout=180)
        assert r.returncode==0,(r.stdout+r.stderr)[-6000:]
        r=subprocess.run([str(d/'obj/Vtb')],capture_output=True,text=True,timeout=30)
        print(r.stdout);assert r.returncode==0,r.stdout+r.stderr

if __name__=='__main__':test_arithmetic()
