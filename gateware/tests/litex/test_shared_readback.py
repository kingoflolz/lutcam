"""Shared response ownership under the supported two-host SoC topology."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = next(path for path in Path(__file__).resolve().parents
            if (path / "fpga/litex/lutcam/control.py").exists())
sys.path.insert(0, str(ROOT / "fpga/litex"))

from migen import ClockDomain, If, Module, Mux, ResetInserter, Signal
from migen.fhdl import verilog as migen_verilog
from litex.gen.fhdl import verilog as litex_verilog
from litex.soc.interconnect import csr_bus, wishbone
from lutcam.boot import BootControl
from lutcam.control import HistogramReader, WishboneGuard
from lutcam.readback import SharedReadback


class SharedReadbackFixture(Module):
    """Real CSR/Boot/Histogram leaves, guards and registered shared fabric."""
    def __init__(self, *, csr_register=False, decoder_registered=True,
                 arbiter="cycle", guard_count=2, csr_width=32,
                 guard_timeout=4096, global_timeout=1_000_000):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.clock_domains.cd_cam_ref = ClockDomain("cam_ref")
        self.submodules.readback = readback = SharedReadback()
        self.submodules.boot = BootControl(shared_readback=readback)
        self.submodules.hist = ResetInserter()(HistogramReader(shared_readback=readback))
        self.submodules.bridge = bridge = wishbone.Wishbone2CSR(
            bus_csr=csr_bus.Interface(data_width=csr_width, alignment=32), register=csr_register)
        self.guards = []
        for index in range(guard_count):
            guard = WishboneGuard(timeout_cycles=guard_timeout, external_count=True)
            setattr(self.submodules, f"guard{index}", guard)
            self.guards.append(guard)
        self.submodules.fabric = wishbone.InterconnectShared(
            [guard.master for guard in self.guards], [
                (lambda address: address[14:] == 0, bridge.wishbone),
                (lambda address: address[10:] == (0x30026000 >> 12), self.boot.bus),
                (lambda address: address[10:] == (0x30022000 >> 12), self.hist.bus),
            ], register=decoder_registered, timeout_cycles=global_timeout, arbiter=arbiter)
        for index, guard in enumerate(self.guards):
            self.comb += guard.granted.eq(self.fabric.arbiter.rr.grant == index)
        readback.validate_topology(bridge, self.fabric, self.guards,
                                  decoder_registered=decoder_registered,
                                  interconnect_timeout_cycles=global_timeout)
        self.csr_writes = Signal(16)
        self.csr_reads = Signal(16)
        self.csr_word0 = Signal(32, reset=0x12345678)
        self.csr_word1 = Signal(32, reset=0x89abcdef)
        self.sync += [
            If(bridge.csr.re,
                self.csr_reads.eq(self.csr_reads + 1),
                bridge.csr.dat_r.eq(Mux(bridge.csr.adr[0], self.csr_word1, self.csr_word0))),
            If(bridge.csr.we,
                self.csr_writes.eq(self.csr_writes + 1),
                If(bridge.csr.adr[0], self.csr_word1.eq(bridge.csr.dat_w))
                .Else(self.csr_word0.eq(bridge.csr.dat_w))),
        ]
        self.comb += self.hist.hist_dump_ready.eq(1)
        self.ports = {
            "sys_clk": self.cd_sys.clk, "sys_rst": self.cd_sys.rst,
            "cam_ref_clk": self.cd_cam_ref.clk, "cam_ref_rst": self.cd_cam_ref.rst,
            "hist_reset": self.hist.reset, "csr_writes": self.csr_writes,
            "csr_reads": self.csr_reads, "csr_word0": self.csr_word0,
            "csr_word1": self.csr_word1,
            "csr_accept": bridge.csr.re | bridge.csr.we,
        }
        # Export acceptance as a named signal rather than a derived port expression.
        accepted = Signal()
        self.comb += accepted.eq(self.ports.pop("csr_accept"))
        self.ports["csr_accept"] = accepted
        for index, guard in enumerate(self.guards):
            for field in ("adr", "dat_w", "sel", "cyc", "stb", "we", "ack", "dat_r"):
                self.ports[f"host{index}_" + field] = getattr(guard.bus, field)
            self.ports[f"host{index}_fault"] = guard.fault
        for name, signal in self.ports.items():
            signal.name_override = name


TB = r"""
module tb;
reg sys_clk=0,sys_rst=0,cam_ref_clk=0,cam_ref_rst=0,hist_reset=0;
reg [29:0] host0_adr=0,host1_adr=0;
reg [31:0] host0_dat_w=0,host1_dat_w=0;
reg [3:0] host0_sel=0,host1_sel=0;
reg host0_cyc=0,host0_stb=0,host0_we=0,host1_cyc=0,host1_stb=0,host1_we=0;
wire host0_ack,host1_ack,host0_fault,host1_fault,csr_accept;
wire [31:0] host0_dat_r,host1_dat_r,csr_word0,csr_word1;
wire [15:0] csr_writes,csr_reads;
shared_fixture dut(.*);
integer cycles=0,cases=0,owner,next_host,target,write_old,n,prior_writes;
reg [31:0] response,expected,address;
task tick;begin #2;sys_clk=1;cam_ref_clk=1;#2;sys_clk=0;cam_ref_clk=0;cycles=cycles+1;#1;end endtask
task off;begin host0_cyc=0;host0_stb=0;host1_cyc=0;host1_stb=0;end endtask
task reset_all;begin off;sys_rst=1;cam_ref_rst=1;repeat(3)tick;sys_rst=0;cam_ref_rst=0;repeat(3)tick;end endtask
task request(input integer host,input[31:0] addr,input wr,input[31:0] data);begin
 if(host==0)begin host0_adr=addr>>2;host0_dat_w=data;host0_we=wr;host0_sel=15;host0_cyc=1;host0_stb=1;end
 else begin host1_adr=addr>>2;host1_dat_w=data;host1_we=wr;host1_sel=15;host1_cyc=1;host1_stb=1;end
end endtask
task wait_response(input integer host,input integer limit);integer elapsed;begin
 elapsed=0;begin: waiting while(elapsed<limit)begin
  #1;if(host==0?host0_ack:host1_ack)begin response=host==0?host0_dat_r:host1_dat_r;disable waiting;end
  tick;elapsed=elapsed+1;end end
 if(elapsed==limit)$fatal(1,"response timeout host=%0d cycle=%0d",host,cycles);
 tick;off;repeat(3)tick;
end endtask
task read_check(input integer host,input[31:0] addr,input[31:0] value);begin
 request(host,addr,0,0);wait_response(host,40);
 if(response!==value)$fatal(1,"wrong response host=%0d addr=%h wanted=%h got=%h",host,addr,value,response);
end endtask
initial begin
 // Legal withdrawal changes old address while CYC is zero; peer may already wait.
 for(owner=0;owner<2;owner=owner+1)for(next_host=0;next_host<2;next_host=next_host+1)
 for(target=0;target<4;target=target+1)for(write_old=0;write_old<2;write_old=write_old+1)begin
  reset_all;
  read_check(owner,32'h30026000,32'h4c425331);
  read_check(owner,32'h30022004,1);
  prior_writes=csr_writes;
  case(target)
   0:begin address=0;expected=write_old?32'h76543210:32'h12345678;end
   1:begin address=4;expected=32'h89abcdef;end
   2:begin address=32'h30026000;expected=32'h4c425331;end
   3:begin address=32'h30022004;expected=1;end
  endcase
  request(owner,0,write_old,32'h76543210);tick;
  if(next_host!=owner)request(next_host,address,0,0);
  // Stop exactly after the accepted CSR side effect, before ACK is consumed.
  begin: accepted for(n=0;n<20;n=n+1)begin #1;if(csr_accept)begin tick;disable accepted;end tick;end end
  if(n==20)$fatal(1,"CSR request never accepted");
  if(owner==0)begin host0_cyc=0;host0_stb=0;host0_adr=address>>2;host0_dat_w=32'hdeadbeef;end
  else begin host1_cyc=0;host1_stb=0;host1_adr=address>>2;host1_dat_w=32'hdeadbeef;end
  if(next_host!=owner)request(next_host,address,0,0);
  tick;
  if(next_host==owner)request(next_host,address,0,0);
  wait_response(next_host,40);
  if(response!==expected)$fatal(1,"canceled CSR reached successor owner=%0d next=%0d target=%0d write=%0d expected=%h actual=%h",owner,next_host,target,write_old,expected,response);
  if(csr_writes!==prior_writes+write_old)$fatal(1,"accepted CSR write lost or duplicated");
  cases=cases+1;
 end
 // Current 4096-cycle guards cannot time out the one-cycle CSR bridge.
 // An unmapped timeout must still release ownership and admit either host.
 for(owner=0;owner<2;owner=owner+1)begin
  reset_all;request(owner,32'h31000000,0,0);wait_response(owner,4120);
  if(response!==32'hffffffff || !(owner==0?host0_fault:host1_fault))$fatal(1,"missing timeout response");
  read_check(1-owner,0,32'h12345678);
  read_check(owner,32'h30026000,32'h4c425331);
 end
 // Local histogram reset must not corrupt another leaf's acknowledged data.
 reset_all;hist_reset=1;read_check(0,32'h30026000,32'h4c425331);hist_reset=0;
 read_check(1,32'h30022004,1);
 $display("PASS shared readback ownership: %0d cancellation cases, both 4096-cycle timeouts, %0d cycles",cases,cycles);$finish;
end endmodule
"""


class SharedReadbackTests(unittest.TestCase):
    def test_unsupported_topologies_fail_closed(self):
        for options in ({"csr_register": True}, {"decoder_registered": False},
                        {"arbiter": "transaction"}, {"guard_count": 1}, {"csr_width": 8},
                        {"guard_timeout": 1}, {"guard_timeout": 4095},
                        {"global_timeout": None}, {"global_timeout": 1},
                        {"global_timeout": 999_999}):
            with self.subTest(**options), self.assertRaises(ValueError):
                SharedReadbackFixture(**options)

    @unittest.skipUnless(shutil.which("iverilog") and shutil.which("vvp"), "native Verilog tools unavailable")
    def test_native_cancellation_side_effects_timeout_and_reset(self):
        for emitter, masters, peer in ((migen_verilog,2,1), (litex_verilog,2,1),
                                       (migen_verilog,3,2), (litex_verilog,3,2)):
            with self.subTest(emitter=emitter.__name__, masters=masters), tempfile.TemporaryDirectory() as temporary:
                path = Path(temporary)
                dut = SharedReadbackFixture(guard_count=masters)
                conversion = emitter.convert(dut, ios=set(dut.ports.values()), name="shared_fixture")
                (path / "fixture.v").write_text(conversion.main_source)
                for name, data in conversion.data_files.items():
                    (path / name).write_text(data)
                bench = TB
                if peer == 2:
                    bench = bench.replace("host1_", "host2_")
                    bench = bench.replace("module tb;", """module tb;
reg [29:0] host1_adr=0;
reg [31:0] host1_dat_w=0;
reg [3:0] host1_sel=0;
reg host1_cyc=0,host1_stb=0,host1_we=0;
wire host1_ack,host1_fault;
wire [31:0] host1_dat_r;
""")
                (path / "tb.sv").write_text(bench)
                result = subprocess.run(["iverilog", "-g2012", "-s", "tb", "-o", "simulation", "fixture.v", "tb.sv"], cwd=path, text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                result = subprocess.run(["vvp", "simulation"], cwd=path, text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("PASS shared readback ownership", result.stdout)


if __name__ == "__main__":
    unittest.main()
