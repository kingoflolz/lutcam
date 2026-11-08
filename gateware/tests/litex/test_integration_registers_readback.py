"""Generated readback remains atomic and preserves wide/signed CSR values."""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "fpga/litex"), str(ROOT / "tests/litex")]

from migen import ClockDomain
from migen.fhdl import verilog as migen_verilog
from litex.gen.fhdl import verilog as litex_verilog
from test_integration_registers_staging import StagedRequestHarness


class RegisterReadbackTests(unittest.TestCase):
    @unittest.skipUnless(
        shutil.which("iverilog") and shutil.which("vvp"), "requires Icarus Verilog"
    )
    def test_generated_emitters_read_signed_wide_narrow_and_live_words(self):
        bench = r"""
module tb;
reg sys_clk=0; always #5 sys_clk=~sys_clk;
reg sys_rst=1,cyc=0,stb=0,we=0;
reg [29:0] adr=0;
reg [31:0] dat_w=0;
reg [3:0] sel=15;
reg signed [31:0] snap_skew=32'hf1234567;
reg [63:0] snap_sof0=64'hfedcba9876543210;
reg [31:0] live=0;
wire ack,err;
wire [31:0] dat_r;
readback_regs dut(.*);
task edge_check; begin @(posedge sys_clk); #1; end endtask
task idle; begin @(negedge sys_clk);cyc=0;stb=0;edge_check();end endtask
task write_word(input [31:0] address,input [31:0] value);
begin
@(negedge sys_clk);adr=address/4;dat_w=value;we=1;cyc=1;stb=1;
edge_check();if(ack||err)$fatal(1,"early write response");
edge_check();if(!ack||err)$fatal(1,"write response missing");idle();
end endtask
task read_word(input [31:0] address,input [31:0] expected);
begin
@(negedge sys_clk);adr=address/4;we=0;cyc=1;stb=1;
edge_check();if(ack||err)$fatal(1,"early read response");
edge_check();if(!ack||err||dat_r!==expected)
$fatal(1,"read address=%h expected=%h actual=%h ack=%b err=%b",address,expected,dat_r,ack,err);
idle();end endtask
initial begin
repeat(3) @(negedge sys_clk); sys_rst=0;
write_word(32'h88,32'h12345678);
write_word(32'h94,32'h89abcdef);
write_word(32'h98,32'h76543210);
write_word(32'hac,32'h123456a5);
read_word(32'h88,32'h12345678);
read_word(32'h94,32'hcdef);
read_word(32'h98,0);
read_word(32'hac,32'ha5);
read_word(32'h240,32'hf1234567);
read_word(32'h230,32'h76543210);
read_word(32'h234,32'hfedcba98);
read_word(32'h200,32'h4c435031);
read_word(32'h3fc,0);
read_word(32'hb4,0);
// Live data changes after address capture; all32bits must sample together.
@(negedge sys_clk);adr=32'h1c/4;we=0;cyc=1;stb=1;live=32'h11223344;
edge_check();if(ack||err)$fatal(1,"early live response");
@(negedge sys_clk);live=32'haabbccdd;
edge_check();if(!ack||err||dat_r!==32'haabbccdd)$fatal(1,"live read torn or delayed");
idle();$display("PASS banked read mux, signed/wide/narrow/live words");$finish;
end
endmodule
"""
        for emitter in (migen_verilog, litex_verilog):
            with self.subTest(emitter=emitter.__name__):
                dut = StagedRequestHarness()
                dut.clock_domains.cd_sys = ClockDomain("sys")
                bus = dut.regs.bus
                ports = {
                    name: getattr(bus, name)
                    for name in (
                        "adr",
                        "dat_w",
                        "sel",
                        "we",
                        "cyc",
                        "stb",
                        "ack",
                        "err",
                        "dat_r",
                    )
                }
                ports.update(
                    sys_clk=dut.cd_sys.clk,
                    sys_rst=dut.cd_sys.rst,
                    snap_skew=dut.pair.snap_skew,
                    snap_sof0=dut.pair.snap_sof0,
                    live=dut.cameras[0].current_frame.status,
                )
                for name, signal in ports.items():
                    signal.name_override = name
                generated = str(
                    emitter.convert(dut, ios=set(ports.values()), name="readback_regs")
                )
                with tempfile.TemporaryDirectory(
                    prefix="lutcam-csr-readback-"
                ) as directory:
                    temporary = Path(directory)
                    (temporary / "dut.v").write_text(generated)
                    (temporary / "tb.v").write_text(bench)
                    subprocess.run(
                        [
                            "iverilog",
                            "-g2012",
                            "-s",
                            "tb",
                            "-o",
                            str(temporary / "sim.vvp"),
                            str(temporary / "dut.v"),
                            str(temporary / "tb.v"),
                        ],
                        check=True,
                    )
                    subprocess.run(["vvp", str(temporary / "sim.vvp")], check=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
