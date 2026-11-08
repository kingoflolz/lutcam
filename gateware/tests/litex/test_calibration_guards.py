"""Independent generated-RTL test of all guards through RAM write commit."""

from pathlib import Path
import subprocess, sys, tempfile, unittest, shutil

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "fpga/litex"), str(ROOT / "tests/litex")]
from migen import ClockDomain
from migen.fhdl import verilog as migen_verilog
from litex.gen.fhdl import verilog as litex_verilog
from lutcam.calibration import CalibrationBank
from test_integration_registers_staging import StagedRequestHarness

PREAMBLE = r"""
module tb;
reg sys_clk=0;always #5 sys_clk=~sys_clk;
reg sys_rst=1,cyc=0,stb=0,we=0,busy=0,drained=1,idle0=1,idle1=1;
reg [29:0]adr=0;reg [31:0]dat_w=0;reg[3:0]sel=15;
wire ack,err;wire[31:0]dat_r,error0,error1;
config_guard dut(.*);
task edge_check;begin @(posedge sys_clk);#2;end endtask
task idle;begin @(negedge sys_clk);cyc=0;stb=0;edge_check();end endtask
task transfer(input bit wr,input[31:0]address,input[31:0]value,input[31:0]expected);
integer timeout;
begin
 @(negedge sys_clk);adr=address/4;dat_w=value;we=wr;sel=15;cyc=1;stb=1;
 timeout=0;
 while(!(ack||err)&&timeout<40)begin edge_check();timeout=timeout+1;end
 if(timeout==40||err||!ack||dat_r!==expected)$fatal(1,"normal transfer addr=%h data=%h expected=%h ack=%b err=%b",address,dat_r,expected,ack,err);
 idle();
end endtask
task forbidden(input integer guard);
begin case(guard)0:busy=1;1:drained=0;2:idle0=0;3:idle1=0;endcase end endtask
task guard_case(input integer guard,input integer cycles,input integer camera,
 input[31:0]address,input[31:0]mask,input[3:0]lanes);
integer n,timeout;
begin
 @(negedge sys_clk);sys_rst=1;cyc=0;stb=0;busy=0;drained=1;idle0=1;idle1=1;
 repeat(3)edge_check();@(negedge sys_clk);sys_rst=0;repeat(20)edge_check();
 transfer(1,address,32'h12345678,0);
 @(negedge sys_clk);adr=address/4;dat_w=32'h87654321;we=1;sel=lanes;cyc=1;stb=1;
 if(cycles==0)forbidden(guard);
 else begin
  for(n=0;n<cycles;n=n+1)begin edge_check();if(ack||err)$fatal(1,"unexpected early response");end
  @(negedge sys_clk);forbidden(guard);
 end
 timeout=0;
 while(!(ack||err)&&timeout<40)begin edge_check();timeout=timeout+1;end
 if(timeout==40||!err||ack||dat_r!==0)$fatal(1,"guard=%0d cycles=%0d camera=%0d address=%h lanes=%h ack=%b err=%b data=%h",guard,cycles,camera,address,lanes,ack,err,dat_r);
 idle();
 transfer(0,address,0,32'h12345678&mask);
 if((camera==0 && (error0!==1 || error1!==0)) || (camera==1 && (error1!==1 || error0!==0)))
  $fatal(1,"wrong addressed-camera sticky error guard=%0d cycle=%0d cam=%0d flags=%h,%h",guard,cycles,camera,error0,error1);
end endtask
initial begin
"""


def build_guard_bench():
    lines = [PREAMBLE]
    count = 0
    for guard in range(4):
        for cycles in range(4):
            for camera in (0, 1):
                for offset, mask in (
                    (0x80, 0xFFFFFFFF),
                    (0x94, 0xFFFF),
                    (0xAC, 0xFF),
                    (0xB0, 0xFFFF),
                ):
                    for sel in (0, 15):
                        lines.append(
                            f"guard_case({guard},{cycles},{camera},32'h{camera*0x100+offset:x},32'h{mask:x},4'h{sel:x});"
                        )
                        count += 1
    lines.append(
        f'$display("PASS {count} independent guard/commit scenarios");$finish;end endmodule'
    )
    return "\n".join(lines), count


def run_guard_checks(emitter, directory):
    bank = CalibrationBank()
    dut = StagedRequestHarness(configuration_bank=bank)
    dut.submodules.calibration_bank = bank
    dut.clock_domains.cd_sys = ClockDomain("sys")
    ports = {
        name: getattr(dut.regs.bus, name)
        for name in ("adr", "dat_w", "sel", "we", "cyc", "stb", "ack", "err", "dat_r")
    }
    ports.update(
        sys_clk=dut.cd_sys.clk,
        sys_rst=dut.cd_sys.rst,
        busy=dut.pair.busy,
        drained=dut.pair.drained,
        idle0=dut.cameras[0].idle,
        idle1=dut.cameras[1].idle,
        error0=dut.cameras[0].configuration_error.status,
        error1=dut.cameras[1].configuration_error.status,
    )
    for name, signal in ports.items():
        signal.name_override = name
    output = emitter.convert(dut, ios=set(ports.values()), name="config_guard")
    directory.mkdir(exist_ok=True)
    (directory / "dut.v").write_text(output.main_source)
    for name, value in output.data_files.items():
        (directory / name).write_text(value)
    bench, count = build_guard_bench()
    (directory / "tb.v").write_text(bench)
    subprocess.run(
        ["iverilog", "-g2012", "-s", "tb", "-o", "sim.vvp", "dut.v", "tb.v"],
        cwd=directory,
        check=True,
    )
    result = subprocess.run(
        ["vvp", "sim.vvp"], cwd=directory, text=True, capture_output=True
    )
    (directory / "simulation.log").write_text(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    print(emitter.__name__, result.stdout, flush=True)
    return dict(emitter=emitter.__name__, returncode=result.returncode, cases=count)


class CalibrationGuardTests(unittest.TestCase):
    @unittest.skipUnless(
        shutil.which("iverilog") and shutil.which("vvp"), "requires Icarus Verilog"
    )
    def test_guard_terms_remain_enforced_through_ram_commit(self):
        with tempfile.TemporaryDirectory(
            prefix="lutcam-calibration-guards-"
        ) as temporary:
            directory = Path(temporary)
            for emitter in (migen_verilog, litex_verilog):
                with self.subTest(emitter=emitter.__name__):
                    run_guard_checks(
                        emitter, directory / emitter.__name__.split(".")[0]
                    )


if __name__ == "__main__":
    unittest.main(verbosity=2)
