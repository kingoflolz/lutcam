"""Check the maintained threshold arithmetic against scalar integer oracles."""
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p / "rtl/lrcca").is_dir())


class ThresholdArithmeticTests(unittest.TestCase):
    def simulate(self, name, sources, testbench, marker):
        if not shutil.which("verilator"):
            self.skipTest("Verilator is required")
        with tempfile.TemporaryDirectory(prefix="threshold-arithmetic-") as temporary:
            build = Path(os.environ.get("THRESHOLD_ARITHMETIC_BUILD", temporary)).resolve() / name
            build.mkdir(parents=True, exist_ok=True)
            bench = build / "tb.sv"
            bench.write_text(testbench)
            # Testbench integer coordinates, stimulus initialization and
            # multiple module names need waivers; RTL diagnostics remain fatal.
            lint = build / "lint.vlt"
            lint.write_text("`verilator_config\n" + "".join(
                f'lint_off -rule {rule} -file "{bench}"\n'
                for rule in ["DECLFILENAME", "WIDTHEXPAND", "WIDTHTRUNC",
                             "BLKSEQ", "PROCASSINIT"]
            ))
            command = ["verilator", "--binary", "--timing", "--assert", "--Wall",
                       "--Wpedantic", "--timescale", "1ns/1ps", "-j", "2",
                       "--top-module", "tb", "--Mdir", str(build / "obj"),
                       str(lint), *map(str, sources), str(bench)]
            result = subprocess.run(command, capture_output=True, text=True, timeout=180)
            (build / "compile.log").write_text(result.stdout + result.stderr)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(build / "obj/Vtb")], capture_output=True,
                                    text=True, timeout=60)
            (build / "run.log").write_text(result.stdout + result.stderr)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn(marker, result.stdout)
            print(result.stdout, end="")

    def test_ll4_mean_extrema_edges_gaps_and_reset(self):
        self.simulate("ll4_mean", [ROOT / "rtl/lrcca/Ll4CausalMean5x3.sv"],
                      (HERE / "ll4_mean_tb.sv").read_text(),
                      "PASS LL4 mean WIDTH=80 pairs=16000 samples=32000")

    def test_ll4_mean_checks_the_complete_input_protocol(self):
        bench = """
module tb;
  logic clk=0, rst=1, in_valid=0, in_first=0, in_last=0;
  logic [9:0] in_pair_x=0, in_y=0;
  logic signed [1:0][9:0] in_value=0;
  wire protocol_error;
  wire unused_valid, unused_first, unused_last, unused_busy;
  wire [9:0] unused_pair_x, unused_y;
  wire signed [1:0][9:0] unused_value;
  always #5 clk=~clk;
  Ll4CausalMean5x3 dut (
    .clk(clk), .rst(rst), .in_valid(in_valid), .in_first(in_first),
    .in_last(in_last), .in_pair_x(in_pair_x), .in_y(in_y), .in_value(in_value),
    .out_valid(unused_valid), .out_first(unused_first), .out_last(unused_last),
    .out_pair_x(unused_pair_x), .out_y(unused_y), .out_value(unused_value),
    .busy(unused_busy), .protocol_error(protocol_error)
  );
  initial begin
    for (integer fault=0; fault<7; fault++) begin
      rst=1; in_valid=0;
      repeat(3) @(negedge clk);
      if(protocol_error) $fatal(1,"protocol flag did not reset");
      rst=0; in_valid=1; in_pair_x=0; in_y=0; in_first=1; in_last=0;
      case(fault)
        0: in_pair_x=1;
        1: in_y=1;
        2: in_pair_x=64;
        3: in_y=64;
        4: in_first=0;
        5: in_last=1;
        default: begin end
      endcase
      @(negedge clk);
      if(fault==6) begin
        // A second pair cannot arrive while its predecessor's second lane
        // is still being serialized.
        in_pair_x=1; in_first=0;
        @(negedge clk);
      end
      in_valid=0;
      repeat(3) @(negedge clk);
      if(!protocol_error) $fatal(1,"missing protocol fault %0d",fault);
    end
    $display("PASS all 7 malformed LL4 input sequences");
    $finish;
  end
endmodule
"""
        self.simulate("ll4_protocol", [ROOT / "rtl/lrcca/Ll4CausalMean5x3.sv"],
                      bench, "PASS all 7 malformed LL4 input sequences")

    def test_residual_strength_complete_byte_domain(self):
        source = ROOT / "rtl/lrcca/AdaptiveThresholdFrontend.sv"
        text = source.read_text()
        start = text.index("  function automatic logic [3:0] residual_strength")
        end = text.index("  endfunction", start) + len("  endfunction")
        # Compile the exact maintained function, without instantiating unrelated
        # frame memories. The oracle uses ordinary absolute difference/division.
        function = text[start:end]
        bench = "module tb;\n" + function + """
  integer distance, expected_magnitude;
  logic [3:0] actual;
  initial begin
    for (integer p=0; p<256; p++) begin
      for (integer t=0; t<256; t++) begin
        distance = p >= t ? p-t : t-p;
        expected_magnitude = (distance+8)/16;
        if (expected_magnitude > 7) expected_magnitude = 7;
        actual = residual_strength(8'(p), 8'(t));
        if (actual !== {1'(p>=t),3'(expected_magnitude)})
          $fatal(1,"pixel=%0d threshold=%0d got=%h expected magnitude=%0d",
                 p,t,actual,expected_magnitude);
      end
    end
    $display("PASS all 65536 pixel/threshold combinations");
    $finish;
  end
endmodule
"""
        print("Residual function SHA256", hashlib.sha256(function.encode()).hexdigest())
        self.simulate("residual", [], bench, "PASS all 65536 pixel/threshold combinations")

    def test_coarse_saturation_complete_signed_domain(self):
        text = (ROOT / "rtl/lrcca/AdaptiveThresholdFrontend.sv").read_text()
        start = text.index("  function automatic logic [PIXEL_BITS-1:0] clamp_coarse")
        end = text.index("  endfunction", start) + len("  endfunction")
        bench = ("module tb;\nlocalparam int PIXEL_BITS=8, COARSE_DATA_BITS=10;\n"
                 + text[start:end] + """
  integer expected;
  logic [7:0] actual;
  initial begin
    for(integer sample_value=-512; sample_value<512; sample_value++) begin
      expected=sample_value+128;
      if(expected<0) expected=0;
      if(expected>255) expected=255;
      actual=clamp_coarse(10'(sample_value));
      if(actual!==8'(expected))
        $fatal(1,"coarse=%0d got=%0d expected=%0d",sample_value,actual,expected);
    end
    $display("PASS all 1024 coarse saturation values");
    $finish;
  end
endmodule
""")
        self.simulate("coarse", [], bench, "PASS all 1024 coarse saturation values")


if __name__ == "__main__":
    unittest.main()
