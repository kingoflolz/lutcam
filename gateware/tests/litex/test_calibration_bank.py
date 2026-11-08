"""Execute generated RTL against a byte-accurate CSR reference model."""

from pathlib import Path
import json
import random
import subprocess
import sys
import tempfile
import unittest
import shutil

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "fpga/litex"), str(ROOT / "tests/litex")]
from migen import ClockDomain
from migen.fhdl import verilog as migen_verilog
from litex.gen.fhdl import verilog as litex_verilog
from lutcam.calibration import CalibrationBank, CONFIG_DEFAULTS
import test_integration_registers_staging as staging

PREAMBLE = r"""
module tb;
reg sys_clk=0; always #5 sys_clk=~sys_clk;
reg sys_rst=1,cyc=0,stb=0,we=0,busy=0;
reg [29:0] adr=0;
reg [31:0] dat_w=0;
reg [3:0] sel=15;
wire ack,err;
wire [31:0] dat_r;
reg cfg_req_valid=0;
reg [3:0] cfg_req_addr=0;
wire cfg_req_ready,cfg_rsp_valid;
wire [31:0] cfg_rsp_data;
reg [31:0] model[0:15];
integer query_count=0;
integer solver_stall=0;
reg accepted;
reg [31:0] expected_data;
config_regs dut(.*);
always @(posedge sys_clk) begin
 if(cfg_req_valid&&!cfg_req_ready&&!sys_rst)solver_stall=solver_stall+1;
 else solver_stall=0;
 if(solver_stall>1)$fatal(1,"host read starved solver for consecutive cycles");
 accepted=cfg_req_valid&&cfg_req_ready&&!sys_rst;
 expected_data=model[cfg_req_addr];
 #1;
 if(cfg_rsp_valid!==accepted)
  $fatal(1,"solver response valid expected=%b actual=%b",accepted,cfg_rsp_valid);
 if(accepted) begin
  if(cfg_rsp_data!==expected_data)
   $fatal(1,"solver data expected=%h actual=%h",expected_data,cfg_rsp_data);
  query_count=query_count+1;
 end
end
task edge_check; begin @(posedge sys_clk); #2; end endtask
task idle; begin @(negedge sys_clk);cyc=0;stb=0;edge_check();end endtask
task transfer(input bit write,input [31:0] address,input [31:0] value,
 input [3:0] select,input [31:0] expected,input bit expected_error);
integer timeout;
begin
 @(negedge sys_clk);adr=address/4;dat_w=value;we=write;sel=select;cyc=1;stb=1;
 edge_check();if(ack||err)$fatal(1,"early response");
 timeout=0;
 while(!(ack||err)&&timeout<100)begin edge_check();timeout=timeout+1;end
 if(timeout==100||err!==expected_error||ack===expected_error||dat_r!==expected)
  $fatal(1,"CSR address=%h write=%b expected=%h actual=%h ack=%b err=%b",address,write,expected,dat_r,ack,err);
 idle();
end endtask
task query(input [3:0] address);
begin
 @(negedge sys_clk);cfg_req_valid=1;cfg_req_addr=address;
 edge_check();while(!cfg_rsp_valid)edge_check();
 @(negedge sys_clk);cfg_req_valid=0;edge_check();
end endtask
task cancelled_write(input [31:0] address,input integer cycles,input bit cancel_stb);
integer n;
begin
 @(negedge sys_clk);adr=address/4;dat_w=32'hbadf00d;we=1;sel=15;cyc=1;stb=1;
 for(n=0;n<cycles;n=n+1)edge_check();
 @(negedge sys_clk);if(cancel_stb)stb=0;else cyc=0;
 repeat(4)begin edge_check();if(ack||err)$fatal(1,"response after cancellation");end
 idle();
end endtask
integer i;
initial begin
 for(i=0;i<16;i=i+1)model[i]=0;
 model[1]=32'h400;model[7]=32'h1013;
 repeat(3) @(negedge sys_clk);sys_rst=0;
"""


def build_bench():
    words = list(CONFIG_DEFAULTS)
    lines = [PREAMBLE]
    operations = 0

    def mapping(camera, position):
        index = (
            (8 + camera * 4 + position)
            if position < 4
            else (0, 1, 2, 4, 3, 5, 6, 7, 7)[position - 4]
        )
        shift, mask = (
            (0, 0xFF)
            if position == 11
            else ((8, 0xFFFF) if position == 12 else
                  ((0, 0xFFFF) if position == 5 else
                   ((0, 0) if position in (6, 7, 8, 9, 10) else (0, 0xFFFFFFFF))))
        )
        return index, shift, mask

    def read(camera, position):
        nonlocal operations
        index, shift, mask = mapping(camera, position)
        address = camera * 0x100 + 0x80 + position * 4
        lines.append(
            f"transfer(0,32'h{address:x},0,15,32'h{(words[index]>>shift)&mask:08x},0);"
        )
        operations += 1

    def write(camera, position, value, select=15):
        nonlocal operations
        index, shift, mask = mapping(camera, position)
        address = camera * 0x100 + 0x80 + position * 4
        prior = (words[index] >> shift) & mask
        byte_mask = (
            sum((0xFF << (byte * 8)) for byte in range(4) if select & (1 << byte))
            & mask
        )
        result = (prior & ~byte_mask) | (value & byte_mask)
        words[index] = (words[index] & ~(mask << shift)) | (result << shift)
        lines.append(f"transfer(1,32'h{address:x},32'h{value:08x},4'h{select:x},0,0);")
        lines.append(f"model[{index}]=32'h{words[index]:08x};")
        operations += 1

    # The first request arrives before initialization completes and must stall.
    for camera in range(2):
        for position in range(13):
            read(camera, position)
    rng = random.Random(0xC0FF33)
    for iteration in range(208):
        camera, position = iteration % 2, iteration % 13
        write(camera, position, rng.getrandbits(32), (iteration // 13) % 16)
        read(camera, position)
        read(1 - camera, position)
    for index in range(16):
        lines.append(f"query(4'd{index});")
    # Both aliases retain shared fields. Intrinsics stay camera-specific.
    for camera in range(2):
        for position in range(13):
            read(camera, position)
    # Reserved words ignore writes, even while armed, and never affect numeric words.
    lines.append("@(negedge sys_clk);busy=1;")
    for camera in range(2):
        for position in (6, 7, 8, 9, 10, 13):
            address = camera * 0x100 + 0x80 + position * 4
            for select in (0, 1, 15):
                lines.append(f"transfer(1,32'h{address:x},32'hffffffff,{select},0,0);")
                lines.append(f"transfer(0,32'h{address:x},0,15,0,0);")
    for index in (2, 3, 4, 5, 6):
        lines.append(f"query(4'd{index});")
    # Busy writes, including zero-byte writes, fail without mutation.
    lines.append("@(negedge sys_clk);busy=1;")
    for address in (0x80, 0xAC, 0xB0, 0x180, 0x194, 0x1B0):
        lines.append(f"transfer(1,32'h{address:x},32'hffffffff,15,0,1);")
        lines.append(f"transfer(1,32'h{address:x},32'hffffffff,0,0,1);")
    for camera in range(2):
        for position in range(13):
            read(camera, position)
    lines.append("@(negedge sys_clk);busy=0;")
    # Abort before capture, decode, or final RAM write: no write may escape.
    for cycles in range(1, 4):
        for cancel_stb in range(2):
            lines.append(f"cancelled_write(32'h94,{cycles},{cancel_stb});")
            read(0, 5)
    # Cancel after RAM write edge preserves exactly the committed update.
    lines.append("cancelled_write(32'h94,4,0);")
    words[1] = 0xF00D
    lines.append("model[1]=32'hf00d;")
    read(0, 5)
    # Simultaneous continuous solver reads exercise host priority and response tags.
    lines.append("@(negedge sys_clk);cfg_req_valid=1;cfg_req_addr=1;")
    for camera in range(2):
        for position in range(13):
            read(camera, position)
    lines.append("@(negedge sys_clk);cfg_req_valid=0;edge_check();")
    # Holding Wishbone continuously exercises the tightest legal read cadence.
    lines.append(
        "@(negedge sys_clk);cfg_req_valid=1;cfg_req_addr=1;adr=32'h94/4;we=0;cyc=1;stb=1;"
    )
    lines.append("repeat(64)edge_check();")
    lines.append(
        "@(negedge sys_clk);cfg_req_valid=0;cyc=0;stb=0;repeat(4)edge_check();"
    )
    # Pair soft reset does not reset stored calibration. Global reset does.
    lines.append("transfer(1,32'h210,1,15,0,0);")
    for position in range(13):
        read(0, position)
    lines.append("@(negedge sys_clk);sys_rst=1;repeat(3)edge_check();")
    lines.append("@(negedge sys_clk);sys_rst=0;")
    words = list(CONFIG_DEFAULTS)
    lines.append("for(i=0;i<16;i=i+1)model[i]=0;model[1]=32'h400;model[7]=32'h1013;")
    for camera in range(2):
        for position in range(13):
            read(camera, position)
    lines.append('if(query_count<32)$fatal(1,"insufficient solver requests");')
    lines.append(
        f'$display("PASS {operations} CSR operations, byte lanes, aliases, busy guards, cancellations, reset, solver arbitration");$finish;end endmodule'
    )
    return "\n".join(lines), operations


def run_generated_checks(OUT):
    bench, count = build_bench()
    records = []
    for emitter in (migen_verilog, litex_verilog):
        bank = CalibrationBank()
        dut = staging.StagedRequestHarness(configuration_bank=bank)
        dut.submodules.calibration_bank = bank
        dut.clock_domains.cd_sys = ClockDomain("sys")
        ports = {
            name: getattr(dut.regs.bus, name)
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
        ports.update(sys_clk=dut.cd_sys.clk, sys_rst=dut.cd_sys.rst, busy=dut.pair.busy)
        ports.update(
            {
                field: getattr(bank, field)
                for field in (
                    "cfg_req_valid",
                    "cfg_req_addr",
                    "cfg_req_ready",
                    "cfg_rsp_valid",
                    "cfg_rsp_data",
                )
            }
        )
        for name, value in ports.items():
            value.name_override = name
        converted = emitter.convert(dut, ios=set(ports.values()), name="config_regs")
        directory = OUT / ("test-" + emitter.__name__.split(".")[0])
        directory.mkdir(exist_ok=True)
        (directory / "dut.v").write_text(converted.main_source)
        for name, value in converted.data_files.items():
            (directory / name).write_text(value)
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
        print(
            emitter.__name__,
            result.returncode,
            result.stdout,
            result.stderr,
            flush=True,
        )
        records.append(
            dict(
                emitter=emitter.__name__,
                returncode=result.returncode,
                csr_operations=count,
            )
        )
        if result.returncode:
            raise SystemExit(result.returncode)
    (OUT / "test-results.json").write_text(json.dumps(records, indent=2) + "\n")


class CalibrationBankTests(unittest.TestCase):
    @unittest.skipUnless(
        shutil.which("iverilog") and shutil.which("vvp"), "requires Icarus Verilog"
    )
    def test_generated_host_abi_and_solver_arbitration(self):
        with tempfile.TemporaryDirectory(
            prefix="lutcam-calibration-bank-"
        ) as directory:
            run_generated_checks(Path(directory))


if __name__ == "__main__":
    unittest.main(verbosity=2)
