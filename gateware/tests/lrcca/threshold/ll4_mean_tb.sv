`timescale 1ns/1ps
module MeanCase #(parameter integer WIDTH=80)(output logic done=0);
  localparam integer HEIGHT=50;
  logic clk=0, rst=1, in_valid=0, in_first=0, in_last=0;
  logic [9:0] in_pair_x=0, in_y=0;
  logic signed [1:0][9:0] in_value=0;
  wire out_valid,out_first,out_last,busy,protocol_error;
  wire [9:0] out_pair_x,out_y;
  wire signed [1:0][9:0] out_value;
  integer scenario=0, observed=0, total=0, checks=0;
  always #5 clk=~clk;
  Ll4CausalMean5x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) dut(.*);
  function automatic integer sample(input integer pattern,y,x);
    integer hash_value;
    case(pattern)
      0:sample=-512;
      1:sample=511;
      2:sample=(x%2==0)?-512:511;
      3:sample=(y%2==0)?511:-512;
      4:sample=((x*17+y*29)&1023)-512;
      5:sample=((x+y)%2==0)?-1:0;
      default:begin
        hash_value=(x*73856093)^(y*19349663)^(pattern*83492791);
        sample=((hash_value^(hash_value>>11))&1023)-512;
      end
    endcase
  endfunction
  function automatic integer expected(input integer y,x);
    integer sum_value, xx, yy, rounded;
    sum_value=0;
    for(integer dy=-2;dy<=0;dy++) begin
      yy=y+dy;if(yy<0)yy=0;
      for(integer dx=-2;dx<=2;dx++) begin
        xx=x+dx;if(xx<0)xx=0;if(xx>=WIDTH)xx=WIDTH-1;
        sum_value+=sample(scenario,yy,xx);
      end
    end
    rounded=sum_value+7;
    expected=rounded>=0?rounded/15:-((-rounded+14)/15);
  endfunction
  always @(posedge clk) begin
    if(rst) observed=0;
    else begin
      if(protocol_error)$fatal(1,"protocol width=%0d scenario=%0d",WIDTH,scenario);
      if(out_valid) begin
        if(out_pair_x!==10'(observed%(WIDTH/2)) || out_y!==10'(observed/(WIDTH/2)) ||
           out_first!==(out_pair_x==0) || out_last!==(out_pair_x==WIDTH/2-1))
          $fatal(1,"coordinates width=%0d scenario=%0d pair=%0d",WIDTH,scenario,observed);
        for(integer lane=0;lane<2;lane++) begin
          if($signed(out_value[lane])!==10'(expected(int'(out_y),2*int'(out_pair_x)+lane)))
            $fatal(1,"value width=%0d scenario=%0d x=%0d y=%0d got=%0d expected=%0d",WIDTH,scenario,2*out_pair_x+lane,out_y,$signed(out_value[lane]),expected(int'(out_y),2*int'(out_pair_x)+lane));
          checks++;
        end
        observed++;
      end
    end
  end
  task automatic send_pair(input integer y,x);
    @(negedge clk);
    in_valid=1;in_first=x==0;in_last=x==WIDTH/2-1;in_y=10'(y);in_pair_x=10'(x);
    in_value[0]=10'(sample(scenario,y,2*x));in_value[1]=10'(sample(scenario,y,2*x+1));
    @(negedge clk);in_valid=0;
    repeat((x+y)%3)@(negedge clk);
  endtask
  initial begin
    repeat(3)@(negedge clk);rst=0;
    for(integer pattern=0;pattern<8;pattern++)begin
      scenario=pattern;
      if(pattern==7)begin
        // Reset while a scalar lane and the filter pipeline are live.
        send_pair(0,0);rst=1;
        repeat(2)@(negedge clk);rst=0;
      end
      for(integer y=0;y<HEIGHT;y++)begin
        for(integer x=0;x<WIDTH/2;x++)send_pair(y,x);
        repeat(12)@(negedge clk);
      end
      repeat(16)@(negedge clk);
      if(busy || observed!=WIDTH*HEIGHT/2)$fatal(1,"drain width=%0d count=%0d",WIDTH,observed);
      total+=observed;observed=0;
    end
    $display("PASS LL4 mean WIDTH=%0d pairs=%0d samples=%0d with extrema, signed rounding, bubbles, frame reuse and in-flight reset",WIDTH,total,checks);
    done=1;
  end
endmodule
module tb;
  wire done0,done1;
  MeanCase #(.WIDTH(80)) a(done0);
  MeanCase #(.WIDTH(4)) b(done1);
  initial begin wait(done0&&done1);$finish;end
  initial begin #5000000;$fatal(1,"timeout");end
endmodule
