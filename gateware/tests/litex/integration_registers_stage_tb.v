module tb;
reg sys_clk=0;always #5 sys_clk=~sys_clk;
reg sys_rst=1,cyc=0,stb=0,we=0;reg[29:0]adr=0;reg[31:0]dat_w=0;reg[3:0]sel=15;
wire ack,err,snapshot;wire[31:0]dat_r,inv_fx;
staged dut(.*);integer snaps=0;
always @(posedge sys_clk) if(!sys_rst&&snapshot)snaps=snaps+1;
task edge_check;begin @(posedge sys_clk);#1;end endtask
initial begin
repeat(3)@(negedge sys_clk);sys_rst=0;
adr=32'h80/4;dat_w=32'h12345678;we=1;cyc=1;stb=1;
edge_check();if(ack||err||inv_fx!==0)$fatal(1,"request capture had effects");
edge_check();if(!ack||err||inv_fx!==32'h12345678)$fatal(1,"decode missing write");
@(negedge sys_clk);adr=32'h228/4;dat_w=1;
edge_check();if(ack||err)$fatal(1,"ACK did not retire");
edge_check();if(ack||err)$fatal(1,"new request should be staged");
edge_check();if(!ack||err||snaps!=1)$fatal(1,"snapshot strobe count");
@(negedge sys_clk);cyc=0;stb=0;
edge_check();
@(negedge sys_clk);adr=32'h80/4;dat_w=32'habcd4321;cyc=1;stb=1;
edge_check();
@(negedge sys_clk);stb=0;
edge_check();if(ack||err||inv_fx!==32'h12345678)$fatal(1,"aborted request committed");
@(negedge sys_clk);cyc=0;
edge_check();$display("PASS generated RTL request/ACK/strobe/abort");$finish;
end endmodule
