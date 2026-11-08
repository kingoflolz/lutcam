`timescale 1ns/1ps
`default_nettype none
module isp_raster_edge_case #(parameter integer WIDTH=16, HEIGHT=4)(output logic done=0);
 localparam integer ROW_PAIRS=WIDTH/2, FRAME_PAIRS=ROW_PAIRS*HEIGHT;
 logic clk=0; always #5 clk=!clk;
 logic rst=1, wb_cyc_i=0,wb_stb_i=0,wb_we_i=0;
 logic [3:0] wb_sel_i=15;
 logic [15:0] wb_adr_i=0;
 logic [31:0] wb_dat_i=0;
 wire [31:0] wb_dat_o;
 wire wb_ack_o,wb_err_o;
 logic in_valid=0,in_sof=0,in_eol=0;
 logic [9:0] in_pixel0=0,in_pixel1=0;
 wire in_ready,out_valid,out_sof,out_eol;
 logic out_ready=1;
 wire [7:0] out_pixel0,out_pixel1;
 logic hist_dump_req=0,hist_out_ready=0;
 wire hist_dump_ready,hist_out_valid,protocol_error;
 wire [31:0] hist_out_data;
  wire unused_position_sof, unused_position_eol, unused_position_eof;
 MonoIsp #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) dut(.*, .input_position_sof(unused_position_sof),
        .input_position_eol(unused_position_eol),
        .input_position_eof(unused_position_eof));
 integer seed=32'h51325e71,cycles=0;
 always @(posedge clk) cycles++;
 function automatic integer unsigned rng(input integer unsigned n);
  integer unsigned x;
  x=n^(n<<13); x=x^(x>>17); return x^(x<<5);
 endfunction
 task automatic idle(input integer n=1);
  repeat(n) begin @(negedge clk); end
 endtask
 task automatic reset;
  @(negedge clk);rst=1; in_valid=0; wb_cyc_i=0;wb_stb_i=0;hist_dump_req=0;hist_out_ready=0;out_ready=1;
  idle(3); rst=0;idle(2);
 endtask
 task automatic bus(input bit wr,input logic [15:0] address,input logic [31:0] data,input bit expecterr=0,input logic [3:0] sel=15);
  integer tries; bit complete;
  @(negedge clk);wb_cyc_i=1;wb_stb_i=1;wb_we_i=wr;wb_adr_i=address;wb_dat_i=data;wb_sel_i=sel;complete=0;
  for(tries=0;tries<8&&!complete;tries++)begin
   @(posedge clk);#1;
   if(wb_ack_o||wb_err_o)begin
    if(wb_err_o!==expecterr || wb_ack_o===expecterr) $fatal(1,"WB result %h %h expectederr%0d",address,data,expecterr);
    complete=1;
   end
  end
  if(!complete)$fatal(1,"WB timeout");
  @(negedge clk);wb_cyc_i=0;wb_stb_i=0;wb_we_i=0;wb_sel_i=15;
 endtask
 task automatic write_word(input integer index,input integer delta,input integer mask);
  bus(1,16'(16'h2000+index*4),32'((delta<<2)|mask));
 endtask
 task automatic run_frame(input bit malformed=0);
  integer addr,tries;bit accepted;
  addr=0;tries=0;
  while(addr<FRAME_PAIRS)begin
   @(negedge clk);seed=rng(seed);in_valid=(seed&7)!=0;out_ready=(seed&24)!=0;
   in_pixel0=10'(addr*17+seed);in_pixel1=10'(addr*29+17);
   in_sof=(addr==0)^malformed;in_eol=(addr%ROW_PAIRS==ROW_PAIRS-1);
   @(posedge clk); accepted=in_valid&&in_ready; #1;
   if(accepted)addr++;
   if(protocol_error) begin
    if(!malformed)$fatal(1,"unexpected frame protocol error %0d",addr);
    break;
   end
   tries++;if(tries>FRAME_PAIRS*8+32)$fatal(1,"frame timeout");
  end
  @(negedge clk);in_valid=0;out_ready=1;idle(12);
 endtask
 task automatic dump;
  integer count;logic [31:0] sum;
  if(!hist_dump_ready)$fatal(1,"dump not ready");
  @(negedge clk);hist_dump_req=1;@(negedge clk);hist_dump_req=0;
  count=0;sum=0;
  while(count<512)begin
   seed=rng(seed);hist_out_ready=(seed&3)!=0;
   @(posedge clk);
   if(hist_out_valid&&hist_out_ready)begin
    if(hist_out_data[28:20]!==9'(count))$fatal(1,"hist bin");
    if(hist_out_data[31] !== (count==0) || hist_out_data[30] !== (count==511))$fatal(1,"hist markers");
    sum+=32'(hist_out_data[19:0]);count++;
   end
   @(negedge clk);
  end
  hist_out_ready=0;idle(2);
  if(sum!==32'(FRAME_PAIRS*3))$fatal(1,"hist sample count %0d expected %0d",sum,FRAME_PAIRS*3);
 endtask
 integer i;
 initial begin
  reset();
  bus(0,0,0);if(wb_dat_o!==32'h4d495350)$fatal(1,"id mismatch");
  bus(1,16'h000c,513,1);bus(1,16'h000c,1,1,4'h3);
  bus(0,16'h0001,0,1);bus(0,16'h0800,0,1);
  bus(1,16'h0004,5,1); // Histogram requires initialization.
  bus(1,16'h2000,32'h100000,1);bus(1,16'h0004,16,1);
  for(i=0;i<1024;i++)bus(1,16'(16'h1000+i*4),32'((i*37) &255));
  for(i=0;i<32;i++)begin bus(0,16'(16'h1000+i*4),0);if(wb_dat_o!==32'((i*37)&255))$fatal(1,"tone readback");end
  for(i=0;i<8;i++)write_word(i,i, i&3);
  for(i=0;i<8;i++)begin bus(0,16'(16'h2000+i*4),0);if(wb_dat_o!==32'((i<<2)|(i&3)))$fatal(1,"defect readback");end
  // Abort a delayed RAM read before the response; subsequent read differs.
  @(negedge clk);wb_cyc_i=1;wb_stb_i=1;wb_we_i=0;wb_adr_i=16'h1000;
  @(posedge clk);#1;if(wb_ack_o||wb_err_o)$fatal(1,"early RAM ACK");
  @(negedge clk);wb_cyc_i=0;wb_stb_i=0;idle(3);
  if(wb_ack_o||wb_err_o)$fatal(1,"phantom abort ACK");bus(0,16'h1004,0);if(wb_dat_o!==37)$fatal(1,"postabort read");
  bus(1,16'h0010,1);idle(515);
  // First descriptor, optional zero-mask continuation and last descriptor.
  write_word(0,0,3);
  if(FRAME_PAIRS>1)begin
   if(FRAME_PAIRS-1>262143)begin
    write_word(1,262142,1);write_word(2,1,2);write_word(3,1,3);
    write_word(4,1,1);write_word(5,FRAME_PAIRS-1-262145,2);bus(1,16'h000c,6);
   end else begin write_word(1,FRAME_PAIRS-1,2);bus(1,16'h000c,2);end
  end else bus(1,16'h000c,1);
  bus(1,16'h0004,7);idle(12);
  bus(1,16'h000c,0,1);bus(1,16'h1000,7,1);bus(0,16'h2000,0,1);
  run_frame();run_frame();run_frame();bus(1,16'h0004,0);idle(8);dump();
  // Exact last pair valid, first out-of-range sum invalid. For full-size
  // frames two maximum deltas exercise sum carry and cumulative overflow.
  reset();
  if(FRAME_PAIRS>262143)begin write_word(0,262143,0);write_word(1,262143,1);bus(1,16'h000c,2);end
  else begin write_word(0,262143,1);bus(1,16'h000c,1);end
  bus(1,16'h0004,1);idle(20);if(!protocol_error)$fatal(1,"oversized delta not rejected");
  bus(1,16'h0004,0);bus(1,16'h0010,2);idle(3);if(protocol_error)$fatal(1,"clear failed");
  // Duplicate nonzero-mask descriptor rejected, including continuation zeros.
  write_word(0,0,1);write_word(1,0,0);write_word(2,0,2);bus(1,16'h000c,3);bus(1,16'h0004,1);idle(24);
  if(!protocol_error)$fatal(1,"duplicate descriptor not rejected");
  reset();write_word(0,0,0);bus(1,16'h000c,1);bus(1,16'h0004,1);idle(12);run_frame(1);
  if(!protocol_error)$fatal(1,"malformed SOF not rejected");
  reset();bus(1,16'h000c,0);bus(1,16'h0004,1);idle(8);
  // Stop request wins over an otherwise-ready input on the same cycle.
  @(negedge clk);in_valid=1;in_sof=1;in_eol=ROW_PAIRS==1;wb_cyc_i=1;wb_stb_i=1;wb_we_i=1;wb_adr_i=4;wb_dat_i=0;
  #1;if(in_ready)$fatal(1,"stop did not block input");
  @(posedge clk);#1;if(!wb_ack_o||wb_err_o)$fatal(1,"stop transaction");
  @(negedge clk);in_valid=0;wb_cyc_i=0;wb_stb_i=0;idle(3);
  // Reconfigure after stop and restart, then reset with a stalled output
  // and a pending defect response. These paths must clear all address state.
  write_word(0,0,3);bus(1,16'h000c,1);bus(1,16'h0004,1);idle(8);
  @(negedge clk);in_valid=1;in_sof=1;in_eol=ROW_PAIRS==1;out_ready=0;
  @(posedge clk);#1;if(!out_valid)$fatal(1,"restart output missing");
  reset();if(protocol_error||out_valid||in_ready)$fatal(1,"dirty reset failed");
  bus(1,16'h000c,0);bus(1,16'h0004,1);idle(8);
  if(!in_ready||protocol_error)$fatal(1,"post-reset restart failed");
  bus(1,16'h0004,0);idle(3);
  $display("PASS ISP raster edges %0dx%0d: %0d cycles",WIDTH,HEIGHT,cycles);done=1;
 end
endmodule
module isp_raster_edges_tb;
 wire [3:0]done;
 isp_raster_edge_case #(.WIDTH(2),.HEIGHT(1)) one(done[0]);
 isp_raster_edge_case #(.WIDTH(16),.HEIGHT(4)) pow2(done[1]);
 isp_raster_edge_case #(.WIDTH(14),.HEIGHT(3)) nonpow2(done[2]);
 isp_raster_edge_case #(.WIDTH(1280),.HEIGHT(800)) full(done[3]);
 initial begin wait(&done);$display("PASS all ISP raster edge cases");$finish;end
 initial begin #100000000;$fatal(1,"global timeout");end
endmodule
`default_nettype wire
