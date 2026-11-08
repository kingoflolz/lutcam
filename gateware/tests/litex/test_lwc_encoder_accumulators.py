"""Modulo and reset regression for the encoder's arithmetic accumulators."""
from pathlib import Path
import os
import hashlib
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
TESTBENCH = r"""module accumulator_tb;
logic clk;
initial clk=0;
logic reset=1, enable=0;
always #5 clk<=~clk;
logic signed [17:0] increment=0;
wire [15:0] value16;
wire [20:0] value21;
logic [15:0] reference16=0;
logic [20:0] reference21=0;
logic [31:0] rng=32'h3917ea5b;
LwcEncoderAccumulator #(.WIDTH(16)) a16(.*,.value(value16));
LwcEncoderAccumulator #(.WIDTH(21)) a21(.*,.value(value21));
integer resets=0, negative_steps=0, stalls=0;
initial begin
  for (integer step=0;step<524288;step=step+1) begin
    @(negedge clk);
    rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
    increment=18'(step);
    reset=step>=262144 && (step==262144 || step%10007==0);
    enable=step<262144 || rng[0];
    if(reset) begin reference16=0; reference21=0;resets=resets+1;end
    else if(enable) begin
      reference16=16'($signed({1'b0,reference16}) + increment);
      reference21=21'($signed({1'b0,reference21}) + increment);
      if(increment<0)negative_steps=negative_steps+1;
    end else stalls=stalls+1;
    @(posedge clk);#1;
    if(value16!==reference16 || value21!==reference21)
      $fatal(1,"accumulator mismatch step=%0d inc=%0d got=%h,%h expected=%h,%h",step,increment,value16,value21,reference16,reference21);
  end
  if(resets<10||negative_steps<100000||stalls<10000)$fatal(1,"missing coverage");
  $display("PASS 524288 clocks: exhaustive signed18 increments, modulo16/21, resets=%0d negative_steps=%0d stalls=%0d",resets,negative_steps,stalls);$finish;
end
endmodule
"""


class EncoderAccumulatorTests(unittest.TestCase):
    def run_accumulator(self, native):
        required = os.environ.get("LUTCAM_REQUIRE_EFINIX_NATIVE") == "1"
        if not shutil.which("verilator"):
            if required:
                self.fail("Required Verilator executable is missing")
            self.skipTest("Verilator is required")
        source = Path(os.environ.get("LWC_ACCUMULATOR_SOURCE",
            ROOT / "rtl/lwc/LwcEncoderAccumulator.sv")).read_text()
        helper = source[source.index("module LwcEncoderAccumulator #"):]
        helper = helper.split("`default_nettype wire", 1)[0]
        vendor = None
        if native:
            efinity = os.environ.get("LITEX_ENV_EFINITY") or os.environ.get("EFINITY_HOME")
            vendor = Path(efinity) / "sim_models/verilog/efx_dsp48.v" if efinity else None
            if vendor is None or not vendor.is_file():
                if required:
                    self.fail("Required Efinity DSP48 model is missing")
                self.skipTest("Set LITEX_ENV_EFINITY for the native DSP48 model")
            vendor_hash = hashlib.sha256(vendor.read_bytes()).hexdigest()
        with tempfile.TemporaryDirectory(prefix="lutcam-encoder-accumulator-") as work:
            p = Path(work)
            (p / "LwcEncoderAccumulator.sv").write_text(helper)
            (p / "accumulator_tb.sv").write_text(TESTBENCH)
            command = ["verilator", "--binary", "--timing", "--assert",
                "--top-module", "accumulator_tb", "--Wall", "-j", "2",
                "--Mdir", str(p / "obj")]
            if native:
                config = p / "vendor.vlt"
                rules = ("DECLFILENAME", "DEFPARAM", "GENUNNAMED", "PROCASSINIT",
                         "SYNCASYNCNET", "UNUSEDPARAM", "UNUSEDSIGNAL", "VARHIDDEN", "WIDTHEXPAND")
                config.write_text("`verilator_config\n" + "".join(
                    f'lint_off -rule {rule} -file "{vendor}"\n' for rule in rules))
                command += ["-DSYNTHESIS", str(config), str(vendor)]
            command += [str(p / "LwcEncoderAccumulator.sv"), str(p / "accumulator_tb.sv")]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(p / "obj/Vaccumulator_tb")],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS 524288 clocks", result.stdout)
            if native:
                self.assertEqual(hashlib.sha256(vendor.read_bytes()).hexdigest(), vendor_hash)

    def test_portable_modulo_and_reset(self):
        self.run_accumulator(False)

    def test_native_modulo_and_reset(self):
        self.run_accumulator(True)


if __name__ == "__main__":
    unittest.main()
