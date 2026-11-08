`timescale 1ns/1ps
module cca_reset_history_tb;
  logic clk=0;
  always #5 clk=!clk;
  logic rst=1, reference_rst=1, valid=0, d0=0, d1=0, eol=0;
  wire [1:0] ready, reference_ready, b0, b1, reference_b0, reference_b1;
  wire [1:0][25:0] data, reference_data;
  logic comparing=0;
  integer comparisons=0;
  integer check_mode=-1;
  for(genvar mode=0;mode<2;mode++) begin : modes
    TwoPPCS0Stage #(.WIDTH(16),.HEIGHT(8),.HOLE_FILL(mode),
      .ROW_BITS(4),.PAIR_BITS(3),.PIPE_DRAIN_CYCLES(2)) dut (
      .clk,.rst,.datavalid(valid),.d0,.d1,.feature_d0(d0),.feature_d1(d1),
      .feature_value_d0(d0),.feature_value_d1(d1),.feature_y(4'd1),.eol,
      .in_ready(ready[mode]),.b0(b0[mode]),.b1(b1[mode]),.out(data[mode]));
    TwoPPCS0Stage #(.WIDTH(16),.HEIGHT(8),.HOLE_FILL(mode),
      .ROW_BITS(4),.PAIR_BITS(3),.PIPE_DRAIN_CYCLES(2)) reference (
      .clk,.rst(reference_rst),.datavalid(valid),.d0,.d1,.feature_d0(d0),.feature_d1(d1),
      .feature_value_d0(d0),.feature_value_d1(d1),.feature_y(4'd1),.eol,
      .in_ready(reference_ready[mode]),.b0(reference_b0[mode]),.b1(reference_b1[mode]),
      .out(reference_data[mode]));
    always @(negedge clk) if(comparing && !rst && (check_mode<0 || check_mode==mode)) begin
      if(ready[mode]!==reference_ready[mode]) $fatal(1,"ready mismatch mode%0d",mode);
      if(dut.s0.valid!==reference.s0.valid) $fatal(1,"valid mismatch mode%0d",mode);
      if(dut.s0.valid) begin
        comparisons++;
        if(data[mode]!==reference_data[mode] || b0[mode]!==reference_b0[mode]
            || b1[mode]!==reference_b1[mode])
          $fatal(1,"history affected fresh frame mode%0d row%0d col%0d a/b=%b%b%b ref=%b%b%b",
            mode,dut.s0.row,dut.s0.col,dut.s0.a,b0[mode],b1[mode],
            reference.s0.a,reference_b0[mode],reference_b1[mode]);
        if(dut.s0.row==0 && (dut.s0.a || b0[mode] || b1[mode]))
          $fatal(1,"first-row predecessor exists mode%0d",mode);
      end
    end
  end
  task automatic tick; @(posedge clk);#1;@(negedge clk);#1;endtask
  task automatic pair(input logic p0,input logic p1,input logic last);
    valid=0;while(ready!=2'b11)tick();
    d0=p0;d1=p1;eol=last;valid=1;tick();valid=0;eol=0;
  endtask
  initial begin
    if($value$plusargs("CHECK_MODE=%d",check_mode)) begin end
    repeat(3)tick();
    for(int attempt=0;attempt<3;attempt++) begin
      rst=0;reference_rst=1;comparing=0;
      // Abort after several foreground rows, at different columns. Both RAM
      // banks retain live topology, unlike a normally drained frame.
      for(int row=0;row<4+attempt;row++)
        for(int col=0;col<8;col++)pair(1,1,col==7);
      for(int col=0;col<2+attempt;col++)pair(1,1,0);
      rst=1;valid=0;
      repeat(attempt==0?1:attempt==1?4:20)tick();
      rst=0;reference_rst=0;comparing=1;
      for(int row=0;row<8;row++)
        for(int col=0;col<8;col++)pair((row+col)%3==0,(row+2*col)%4==0,col==7);
      repeat(10)tick();
      comparing=0;rst=1;reference_rst=1;tick();
    end
    $display("PASS CCA reset history both hole-fill modes comparisons=%0d",comparisons);
    $finish;
  end
endmodule
