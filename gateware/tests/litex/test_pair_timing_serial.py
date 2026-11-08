"""Serial timing validation preserves byte merges, guards and cancellation."""
from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest

from migen import ClockDomain
from migen.fhdl import verilog as migen_verilog
from litex.gen.fhdl import verilog as litex_verilog
from test_integration_registers_staging import StagedRequestHarness


def fixture():
    dut = StagedRequestHarness()
    dut.clock_domains.cd_sys = ClockDomain("sys")
    ports = {"sys_clk": dut.cd_sys.clk, "sys_rst": dut.cd_sys.rst}
    fields = ("watchdog_cycles", "max_skew_cycles", "sync_period", "sync_width",
              "min_latency_cycles", "max_latency_cycles", "phase0_cycles", "phase1_cycles",
              "busy", "drained", "sync_enable")
    for name in fields:
        ports["pair_" + name] = getattr(dut.pair, name)
    for index, camera in enumerate(dut.cameras):
        ports[f"camera{index}_idle"] = camera.idle
    for name in ("adr", "dat_w", "sel", "cyc", "stb", "we", "ack", "err", "dat_r"):
        ports["master_" + name] = getattr(dut.regs.bus, name)
    for name, signal in ports.items():
        signal.name_override = name
    return dut, ports


TB = r"""
module tb;
reg [0:0] camera0_idle=0;
reg [0:0] camera1_idle=0;
reg [0:0] pair_busy=0;
reg [0:0] pair_drained=0;
reg [29:0] master_adr=0;
reg [31:0] master_dat_w=0;
reg [3:0] master_sel=0;
reg [0:0] master_cyc=0;
reg [0:0] master_stb=0;
reg [0:0] master_we=0;
reg [0:0] sys_clk=0;
reg [0:0] sys_rst=0;
wire [21:0] pair_watchdog_cycles;
wire [21:0] pair_max_skew_cycles;
wire [20:0] pair_sync_period;
wire [20:0] pair_sync_width;
wire [0:0] pair_sync_enable;
wire [20:0] pair_min_latency_cycles;
wire [20:0] pair_max_latency_cycles;
wire [20:0] pair_phase0_cycles;
wire [20:0] pair_phase1_cycles;
wire [31:0] master_dat_r;
wire [0:0] master_ack;
wire [0:0] master_err;
candidate dut(.camera0_idle(camera0_idle),.camera1_idle(camera1_idle),.pair_busy(pair_busy),.pair_drained(pair_drained),.pair_watchdog_cycles(pair_watchdog_cycles),.pair_max_skew_cycles(pair_max_skew_cycles),.pair_sync_period(pair_sync_period),.pair_sync_width(pair_sync_width),.pair_sync_enable(pair_sync_enable),.pair_min_latency_cycles(pair_min_latency_cycles),.pair_max_latency_cycles(pair_max_latency_cycles),.pair_phase0_cycles(pair_phase0_cycles),.pair_phase1_cycles(pair_phase1_cycles),.master_adr(master_adr),.master_dat_w(master_dat_w),.master_dat_r(master_dat_r),.master_sel(master_sel),.master_cyc(master_cyc),.master_stb(master_stb),.master_ack(master_ack),.master_we(master_we),.master_err(master_err),.sys_clk(sys_clk),.sys_rst(sys_rst));

integer cycles=0, checks=0, transactions=0, cancels=0, guards=0;
integer i,j,k,n,phase,timeout,selected,seed=32'h27419abc;
reg [31:0] expected[0:7]; reg [31:0] offsets[0:7];
reg [31:0] data,merged,last_data; reg [3:0] lanes; reg valid,last_error;
reg [31:0] before_value,after_value;
function [31:0] actual(input integer index);
begin case(index)

0:actual=pair_watchdog_cycles;
1:actual=pair_max_skew_cycles;
2:actual=pair_sync_period;
3:actual=pair_sync_width;
4:actual=pair_min_latency_cycles;
5:actual=pair_max_latency_cycles;
6:actual=pair_phase0_cycles;
7:actual=pair_phase1_cycles;
default:actual=32'hdeadbeef;endcase end endfunction
function isvalid(input integer index,input[31:0] value);
begin
isvalid=(index<2)?(value<32'h400000):(value<32'h200000);
case(index)
0:isvalid=isvalid&&(value>expected[1]);
1:isvalid=isvalid&&(value>0)&&(value<expected[0]);
2:isvalid=isvalid&&(value>expected[3])&&(value>expected[5])&&(value>expected[6])&&(value>expected[7]);
3:isvalid=isvalid&&(value>0)&&(value<expected[2]);
4:isvalid=isvalid&&(value>0)&&(value<expected[5]);
5:isvalid=isvalid&&(value>expected[4])&&(value<expected[2]);
6,7:isvalid=isvalid&&(value<expected[2]);
endcase end endfunction
task check_values;integer z;begin
checks=checks+1;
for(z=0;z<8;z=z+1)if(actual(z)!==expected[z])$fatal(1,"Value mismatch cycle %0d field %0d actual %h expected %h",cycles,z,actual(z),expected[z]);
end endtask
task tick;begin #1;sys_clk=1;#1;sys_clk=0;



#1;cycles=cycles+1;end endtask
task idle;begin master_cyc=0;master_stb=0;tick;end endtask
task drive(input[31:0] address,input wr,input[31:0] value,input[3:0] select);begin
master_adr=address>>2;master_we=wr;master_dat_w=value;master_sel=select;master_cyc=1;master_stb=1;end endtask
task response;begin timeout=0;
begin:await_response while(timeout<100)begin tick;timeout=timeout+1;
if(master_ack||master_err)begin last_data=master_dat_r;last_error=master_err;disable await_response;end end end
if(timeout==100)$fatal(1,"Timeout at cycle %0d address %h",cycles,master_adr<<2);
transactions=transactions+1;tick;idle;
end endtask
task tx(input[31:0]address,input wr,input[31:0]value,input[3:0]select);begin drive(address,wr,value,select);response;end endtask
task reset_all;begin
master_cyc=0;master_stb=0;
sys_rst=1;repeat(8)tick;
sys_rst=0;
pair_busy=0;pair_drained=1;camera0_idle=1;camera1_idle=1;
expected[0]=4000000;expected[1]=12000;expected[2]=994812;expected[3]=1536;
expected[4]=2;expected[5]=900000;expected[6]=0;expected[7]=0;
repeat(40)tick;check_values;
end endtask
task oracle_write(input integer index,input[31:0]value,input[3:0]select);integer b;begin
merged=expected[index];for(b=0;b<4;b=b+1)if(select[b])merged[b*8+:8]=value[b*8+:8];
valid=isvalid(index,merged);
tx(32'h30024000+offsets[index],1,value,select);
if(last_error!==!valid)$fatal(1,"Validity mismatch cycle %0d field %0d old %h data %h lanes %h merged %h valid %b err %b",cycles,index,expected[index],value,select,merged,valid,last_error);
if(valid)begin expected[index]=merged;if(last_data!==0)$fatal(1,"Nonzero successful write data %h",last_data);end
check_values;
tx(32'h30024000+offsets[index],0,0,15);
if(last_error||last_data!==expected[index])$fatal(1,"Readback mismatch field %0d",index);
end endtask
initial begin
offsets[0]='h220;offsets[1]='h224;offsets[2]='h264;offsets[3]='h268;
offsets[4]='h288;offsets[5]='h28c;offsets[6]='h290;offsets[7]='h294;



reset_all;
// Every byte mask and all bound equalities, adjacent values, zero and reserved bits.
for(i=0;i<8;i=i+1)for(j=0;j<16;j=j+1)begin
oracle_write(i,expected[i]+1,j);
for(k=0;k<8;k=k+1)begin
oracle_write(i,expected[k]-1,j);oracle_write(i,expected[k],j);oracle_write(i,expected[k]+1,j);
end
oracle_write(i,0,j);oracle_write(i,32'hffffffff,j);oracle_write(i,32'h200000,j);oracle_write(i,32'h400000,j);
end
for(n=0;n<2500;n=n+1)begin
selected=$unsigned($random(seed))%8;lanes=$random(seed);data=$random(seed);
if(n%3==0)data=data&32'h3fffff;
if(n%5==0)data=expected[selected]+($unsigned($random(seed))%7)-3;
oracle_write(selected,data,lanes);
end
// Drop either CYC or STB at every request/validation/ACK phase. Once
// withdrawn, no not-yet-visible write may appear later or poison a peer read.
for(i=0;i<8;i=i+1)for(j=0;j<2;j=j+1)for(phase=0;phase<17;phase=phase+1)begin
reset_all;data=expected[i]+1;
drive(32'h30024000+offsets[i],1,data,15);
repeat(phase)tick;
expected[i]=actual(i);
if(j==0)master_cyc=0;else master_stb=0;
master_adr=30'h2fffffff;master_dat_w=32'hbad0bad0;master_sel=0;
tick;idle;repeat(14)tick;check_values;cancels=cancels+1;
tx(32'h30024200,0,0,15);if(last_error||last_data!==32'h4c435031)$fatal(1,"Peer read poisoned after cancellation");
check_values;
end
// A guard falling before the commit edge prevents the delayed write.
for(i=0;i<8;i=i+1)for(j=0;j<4;j=j+1)for(phase=0;phase<16;phase=phase+1)begin
reset_all;data=expected[i]+1;drive(32'h30024000+offsets[i],1,data,15);repeat(phase)tick;
before_value=actual(i);
case(j)0:pair_busy=1;1:pair_drained=0;2:camera0_idle=0;3:camera1_idle=0;endcase
if(master_ack||master_err)begin tick;idle;end else response;
after_value=actual(i);if(after_value!==before_value)$fatal(1,"Guard failed phase %0d field %0d",phase,i);
expected[i]=before_value;check_values;guards=guards+1;
end
// Captured payload, byte lanes and target remain stable through validation.
for(i=0;i<8;i=i+1)for(phase=0;phase<8;phase=phase+1)begin
reset_all;data=expected[i]+1;drive(32'h30024000+offsets[i],1,data,15);
timeout=0;while(dut.INTERNAL_STATE!=8&&timeout<20)begin tick;timeout=timeout+1;end
if(timeout==20)$fatal(1,"Timing request failed to enter validation");
repeat(phase)tick;
master_adr=32'h30024294>>2;master_dat_w=32'hffffffff;master_sel=0;master_we=0;
response;if(last_error||last_data!==0)$fatal(1,"Captured timing payload failed");
expected[i]=data;check_values;
end
// Sync-disable is rechecked at the final edge, in addition to the pause guard.
for(i=0;i<8;i=i+1)for(phase=0;phase<16;phase=phase+1)begin
reset_all;data=expected[i]+1;drive(32'h30024000+offsets[i],1,data,15);repeat(phase)tick;
before_value=actual(i);force dut.pair_sync_enable=1;
if(master_ack||master_err)begin tick;idle;end else response;
if(actual(i)!==before_value)$fatal(1,"Sync guard failed phase %0d field %0d",phase,i);
expected[i]=before_value;check_values;guards=guards+1;release dut.pair_sync_enable;
end
// Reset during every validation cycle and check clean recovery immediately.
for(i=0;i<8;i=i+1)for(phase=0;phase<15;phase=phase+1)begin
reset_all;drive(32'h30024000+offsets[i],1,expected[i]+1,15);repeat(phase)tick;reset_all;
oracle_write(i,expected[i]+1,15);
end
$display("PASS cycles=%0d checks=%0d transactions=%0d cancellation_cases=%0d guard_cases=%0d",cycles,checks,transactions,cancels,guards);
$finish;end endmodule

"""


class SerialTimingValidationTests(unittest.TestCase):
    def test_native_bounds_byte_masks_cancel_reset_and_commit_guards(self):
        missing = [tool for tool in ("iverilog", "vvp") if not shutil.which(tool)]
        if missing:
            if os.environ.get("LUTCAM_REQUIRE_EFINIX_NATIVE") == "1":
                self.fail("Required native tools missing: " + ", ".join(missing))
            self.skipTest("Native Verilog tools unavailable")
        for emitter in (migen_verilog, litex_verilog):
            with self.subTest(emitter=emitter.__name__), tempfile.TemporaryDirectory() as temporary:
                path = Path(temporary)
                dut, ports = fixture()
                conversion = emitter.convert(dut, ios=set(ports.values()), name="candidate")
                source = conversion.main_source
                (path / "candidate.v").write_text(source)
                for name, content in conversion.data_files.items():
                    (path / name).write_text(content)
                state = re.search(r"assign master_ack = .*?\((\w+) == 3'd5\)", source)
                self.assertIsNotNone(state, "Cannot locate captured request state")
                (path / "tb.sv").write_text(TB.replace("INTERNAL_STATE", state.group(1)))
                result = subprocess.run(["iverilog", "-g2012", "-s", "tb", "-o", "simulation",
                                         "tb.sv", "candidate.v"], cwd=path, text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                result = subprocess.run(["vvp", "simulation"], cwd=path, text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("PASS cycles=", result.stdout)
                print(emitter.__name__, result.stdout.strip())


if __name__ == "__main__":
    unittest.main()
