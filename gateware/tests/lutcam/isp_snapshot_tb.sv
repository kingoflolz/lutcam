`timescale 1ns/1ps
`default_nettype none
module isp_snapshot_tb;
logic clk=0,rst=1;
always #5 clk=!clk;
logic wb_cyc_i=0,wb_stb_i=0,wb_we_i=0;
logic [3:0] wb_sel_i=15;
logic [15:0] wb_adr_i=0;
logic [31:0] wb_dat_i=0;
wire [31:0] wb_dat_o;
wire wb_ack_o,wb_err_o;
logic in_valid=0,in_sof=0,in_eol=0;
logic [9:0] in_pixel0=200,in_pixel1=200;
wire in_ready,out_valid,out_sof,out_eol;
wire [7:0] out_pixel0,out_pixel1;
logic out_ready=1,hist_dump_req=0,hist_out_ready=0;
wire hist_dump_ready,hist_out_valid,protocol_error;
wire [31:0] hist_out_data;
wire input_position_sof,input_position_eol,input_position_eof;
MonoIsp #(.WIDTH(8),.HEIGHT(4)) isp(.*);
integer accepted=0,emitted=0;
always @(posedge clk)begin
 if(in_valid && in_ready)accepted<=accepted+1;
 if(out_valid && out_ready)emitted<=emitted+1;
 if(!rst && protocol_error)$fatal(1,"raster protocol error");
end
task wr(input logic[15:0] a,input logic[31:0] d);
 @(negedge clk);wb_cyc_i=1;wb_stb_i=1;wb_we_i=1;wb_adr_i=a;wb_dat_i=d;
 do begin @(posedge clk);#1;if(wb_err_o)$fatal(1,"ISP rejected write %h",a);end while(!wb_ack_o);
 @(negedge clk);wb_cyc_i=0;wb_stb_i=0;wb_we_i=0;
endtask
task frame(input logic[9:0] value);
 for(integer i=0;i<16;i++)begin
  @(negedge clk);in_valid=1;in_sof=(i==0);in_eol=(i%4==3);in_pixel0=value;in_pixel1=value;
  do begin @(posedge clk);end while(!in_ready);
 end
 @(negedge clk);in_valid=0;
 repeat(3)@(posedge clk);
endtask
task dump(input integer expected_bin);
 logic[31:0] held;
 integer total;
 wait(hist_dump_ready);
 @(negedge clk);hist_dump_req=1;
 @(negedge clk);hist_dump_req=0;
 total=0;
 for(integer i=0;i<512;i++)begin
  if(!hist_out_valid)$fatal(1,"missing histogram word %d",i);
  held=hist_out_data;
  repeat(2)begin @(negedge clk);if(hist_out_data!==held)$fatal(1,"histogram changed under backpressure");end
  if(hist_out_data[28:20]!=9'(i) || hist_out_data[31]!=(i==0) || hist_out_data[30]!=(i==511))$fatal(1,"histogram markers");
  if(hist_out_data[19:0] != ((i==expected_bin)?20'd16:20'd0))$fatal(1,"mixed/partial histogram at bin %d: %d",i,hist_out_data[19:0]);
  total+=int'(hist_out_data[19:0]);
  hist_out_ready=1;
  @(negedge clk);hist_out_ready=0;
 end
 if(total!=16)$fatal(1,"wrong histogram total");
endtask
initial begin
 repeat(4)@(posedge clk);@(negedge clk);rst=0;
 wr(16,1);repeat(515)@(posedge clk);wr(4,13);
 repeat(515)@(posedge clk);
 frame(200);
 fork
  begin dump(100);end
  begin for(integer n=0;n<180;n++)frame(400);end
 join
 // Clear takes 512 cycles and can finish mid-frame. Only a subsequent SOF
 // may start collection; the next frozen histogram must still total 16.
 dump(200);
 repeat(5)@(posedge clk);
 if(accepted!=181*16 || emitted!=accepted)$fatal(1,"histogram access lost video pairs");
 // Stop/restart while a snapshot is unread must discard that old exposure.
 repeat(515)@(posedge clk);frame(600);wr(4,0);wr(4,13);
 repeat(515)@(posedge clk);frame(800);dump(400);
 $display("PASS streaming histogram snapshots, full frames, backpressure and restart");$finish;
end
initial begin #2000000;$fatal(1,"snapshot test timeout");end
endmodule
