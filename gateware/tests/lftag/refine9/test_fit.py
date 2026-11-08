"""RTL fit checked against the scheduled integer model; see test_accuracy for an independent floating solve."""
from pathlib import Path
import subprocess,tempfile
import numpy as np
from program import reference,ONES,ONE,S33,UNIT,ZERO
from model import fixtures
ROOT=Path(__file__).resolve().parents[3]

def test_fit():
    cases=list(fixtures(100));data=[]
    for tag_id,points,_ in cases:
        data += [tag_id]+[int(v) for pair in points for v in pair]+reference(tag_id,points)
    with tempfile.TemporaryDirectory(prefix='lftag-fit9-') as td:
        d=Path(td);(d/'vectors.hex').write_text(''.join(f'{v&0xffffffff:08x}\n' for v in data))
        constants=''.join(f"memory[{a}]=20'h{v&0xfffff:05x};" for a,v in {**{ONES+i:ONE for i in range(9)},S33:9<<14,UNIT:ONE,ZERO:0}.items())
        (d/'tb.sv').write_text(('''module tb;
logic clk=0,rst=1,start=0;always #5 clk=!clk;
wire ready,done,ok;wire [3:0] point_number;logic[13:0] tag_id;
logic[14:0] point_x_q4;logic[13:0] point_y_q4;
wire config_read_valid;wire [1:0] config_read_address;logic config_response_valid=0;logic [19:0] config_read_data;
wire [8:0] address_a,address_b;wire write_a,write_b;logic signed [19:0] read_data_a,read_data_b;wire signed [19:0] write_data_a,write_data_b;logic[3:0] point_world_index;
logic[19:0] job_memory[512];wire[7:0] program_address;wire program_read;logic[19:0] instruction;
logic[19:0] memory[512];logic[31:0] vectors[2800];integer testno,count;
LfTagFit9 dut(.clk,.rst,.start,.ready,.done,.done_ready(1'b0),.ok,.tag_id,.scale(2'b0),.point_number,
.program_address,.program_read,.instruction,.point_world_index,.point_x_q4,.point_y_q4,
.config_read_valid,.config_read_address,.config_read_ready(1'b1),.config_response_valid,.config_read_data,
.address_a,.address_b,.read_data_a,.read_data_b,.write_a,.write_b,.write_data_a,.write_data_b);
always @(posedge clk)begin
 instruction<=job_memory[256+int'(program_address)];
 if(write_a)memory[address_a]<=write_data_a;else read_data_a<=memory[address_a];
 if(write_b)memory[address_b]<=write_data_b;else read_data_b<=memory[address_b];
 point_world_index<=point_number;
 point_x_q4<=15'(vectors[testno*28+1+2*int'(point_number)]);
 point_y_q4<=14'(vectors[testno*28+2+2*int'(point_number)]);
 config_response_valid<=config_read_valid;
 case(config_read_address)
  0,1:config_read_data<=20'd47934;
  2:config_read_data<=20'd10240;
  3:config_read_data<=20'd6400;
 endcase
end
initial begin
`include "lftag_fit9_program.svh"
$readmemh("vectors.hex",vectors);CONSTANT_INIT repeat(4)@(negedge clk);rst=0;
for(testno=0;testno<100;testno++)begin
 tag_id=14'(vectors[testno*28]);
 @(negedge clk);start=1;@(negedge clk);start=0;
 count=0;while(!done&&count<1000)begin @(negedge clk);count++;end
 if(!done||!ok||count>=1000)$fatal(1,"failed fit %0d state=%0d cycles=%0d pc=%0d",testno,dut.state,count,dut.pc);
 for(integer i=0;i<9;i++)if($signed(memory[256+2*i])!==$signed(vectors[testno*28+19+i]))$fatal(1,"fit %0d coefficient %0d got %h expected %h",testno,i,memory[256+2*i],vectors[testno*28+19+i]);
 if(testno==0)$display("FIT_CYCLES %0d",count);
 rst=1;repeat(2)@(negedge clk);rst=0;
end
// Reset while the reciprocal divider is active, then rerun without
// resetting scratch RAM: all working state must be reconstructed.
testno=0;tag_id=14'(vectors[0]);
@(negedge clk);start=1;@(negedge clk);start=0;
count=0;while(dut.state!=dut.DIVIDE_STEP && count<1000)begin @(negedge clk);count++;end
if(dut.state!=dut.DIVIDE_STEP)$fatal(1,"no divider state reached");
rst=1;repeat(2)@(negedge clk);rst=0;
@(negedge clk);start=1;@(negedge clk);start=0;
count=0;while(!done&&count<1000)begin @(negedge clk);count++;end
if(!done||!ok||count>=1000)$fatal(1,"post-reset fit failed");
for(integer i=0;i<9;i++)if($signed(memory[256+2*i])!==$signed(vectors[19+i]))$fatal(1,"post-reset coefficient mismatch");
// A coincident nine-point group is singular and must not publish a pose.
rst=1;repeat(2)@(negedge clk);rst=0;
for(integer i=0;i<9;i++)begin vectors[1+2*i]=10240;vectors[2+2*i]=6400;end
@(negedge clk);start=1;@(negedge clk);start=0;
count=0;while(!done&&count<1000)begin @(negedge clk);count++;end
if(!done||ok||count>=1000)$fatal(1,"singular fit was accepted");
repeat(10)begin @(negedge clk);if(!done||(write_a||write_b))$fatal(1,"completed fit did not hold");end
$display("PASS 100 calibrated nine-point fits, reset during division, singular rejection");$finish;
end
endmodule
''').replace('CONSTANT_INIT',constants))
        r=subprocess.run(['verilator','--binary','--timing','--assert','-Wno-fatal','-j','2','-I'+str(ROOT/'rtl/lftag'),'--top-module','tb','--Mdir',str(d/'obj'),str(d/'tb.sv'),str(ROOT/'rtl/lftag/LfTagWideAddSub.sv'),str(ROOT/'rtl/lftag/LfTagFit9.sv')],cwd=d,capture_output=True,text=True,timeout=180)
        assert r.returncode==0,r.stdout+r.stderr
        r=subprocess.run([str(d/'obj/Vtb')],cwd=d,capture_output=True,text=True,timeout=30);print(r.stdout);assert r.returncode==0,r.stdout+r.stderr

if __name__=='__main__':test_fit()
