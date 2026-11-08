module tb;
reg sys_clk=0,sys_rst=0;
reg h0_cyc=0,h0_stb=0,h0_we=0,h1_cyc=0,h1_stb=0,h1_we=0;
reg[29:0]h0_adr=0,h1_adr=0;
reg[31:0]h0_dat_w=0,h1_dat_w=0;
reg[3:0]h0_sel=0,h1_sel=0;
reg query_ready=1,query_response_valid=0,config_write_ready=1;
reg[31:0]query_data=0;
reg camera0_idle=1,camera1_idle=1,pair_busy=0,pair_drained=1;
wire mismatch,h0_ack,h1_ack,h0_fault_event,h1_fault_event;
wire[31:0]h0_dat_r,h1_dat_r;
wire query_valid,config_write_valid;
wire[5:0]query_address;
wire[3:0]config_write_address;
wire[31:0]config_write_data;
miter dut(.*);
integer cycles=0,transactions=0,timeouts=0,aborts=0,handoffs=0,writes=0;
integer latency=3,delay_count=0,saved_query=0;
reg[31:0]memory[0:63];
always @(posedge sys_clk)begin
 query_response_valid<=0;
 if(sys_rst)delay_count<=0;
 else begin
  if(delay_count>0)delay_count<=delay_count-1;
  if(delay_count==1)begin query_response_valid<=1;query_data<=memory[saved_query];end
  if(query_valid&&query_ready)begin saved_query<=query_address;delay_count<=latency;end
  if(config_write_valid&&config_write_ready)begin
   memory[36+config_write_address]<=config_write_data;writes<=writes+1;
  end
 end
end
task check;begin if(mismatch)$fatal(1,"held request mismatch cycle=%0d address=%h",cycles,h0_adr<<2);end endtask
task tick;begin #1;check;sys_clk=1;#1;check;sys_clk=0;#1;check;cycles=cycles+1;end endtask
task reset_all;begin
 h0_cyc=0;h0_stb=0;h1_cyc=0;h1_stb=0;sys_rst=1;
 query_ready=1;config_write_ready=1;latency=3;pair_busy=0;pair_drained=1;camera0_idle=1;camera1_idle=1;
 repeat(3)tick;sys_rst=0;repeat(20)tick;
end endtask
task drive(input integer host,input[31:0]address,input wr,input[31:0]data,input[3:0]lanes);begin
 if(host==0)begin h0_cyc=1;h0_stb=1;h0_adr=address>>2;h0_we=wr;h0_dat_w=data;h0_sel=lanes;end
 else begin h1_cyc=1;h1_stb=1;h1_adr=address>>2;h1_we=wr;h1_dat_w=data;h1_sel=lanes;end
end endtask
task withdraw(input integer host);begin
 if(host==0)begin h0_cyc=0;h0_stb=0;h0_adr=30'h3fffffff;h0_we=~h0_we;h0_sel=~h0_sel;h0_dat_w=~h0_dat_w;end
 else begin h1_cyc=0;h1_stb=0;h1_adr=0;h1_we=~h1_we;h1_sel=~h1_sel;h1_dat_w=~h1_dat_w;end
end endtask
reg[31:0]last;
task transfer(input integer host,input[31:0]address,input wr,input[31:0]data,input[3:0]lanes);
integer waited;begin
 drive(host,address,wr,data,lanes);
 begin:await_ack for(waited=0;waited<100;waited=waited+1)begin
  #1;if(host==0?h0_ack:h1_ack)disable await_ack;tick;
 end end
 if(!(host==0?h0_ack:h1_ack))$fatal(1,"transaction timeout");
 last=host==0?h0_dat_r:h1_dat_r;tick;withdraw(host);repeat(3)tick;transactions=transactions+1;
end endtask
integer host,word,mask,phase,mode,n,q,got0,got1,old_writes,seed=32'h325819a7;
reg[31:0]target;
initial begin
 for(n=0;n<64;n=n+1)memory[n]=32'h81230000+n;
 reset_all;
 // All calibration words, byte lanes and timing targets on both owners.
 for(host=0;host<2;host=host+1)for(word=0;word<13;word=word+1)for(mask=0;mask<16;mask=mask+1)begin
  transfer(host,32'h80+host*256+word*4,1,$random(seed),mask[3:0]);
  transfer(host,32'h80+host*256+word*4,0,0,15);
 end
 for(word=0;word<8;word=word+1)for(mask=0;mask<16;mask=mask+1)begin
  case(word)0:target='h220;1:target='h224;2:target='h264;3:target='h268;
   4:target='h288;5:target='h28c;6:target='h290;7:target='h294;endcase
  transfer(mask[0],target,1,$random(seed),mask[3:0]);
  transfer(mask[0],target,0,0,15);
 end
 // Cancel before each possible bounded calibration/timing commit and then
 // reselect from the peer. Global reset is checked at those same phases.
 for(mode=0;mode<3;mode=mode+1)for(phase=0;phase<16;phase=phase+1)begin
  drive(0,mode==1?'h220:'h80,1,32'd3000000,5);repeat(phase)tick;
  if(mode==2)reset_all;else begin withdraw(0);tick;end
  transfer(1,'h1000,0,0,15);if(last!==32'h5a69c387)$fatal(1,"peer response changed");aborts=aborts+1;
 end
 // Real4096-cycle timeout in COUNT_REQUEST, COUNT_WAIT and CONFIG_WRITE.
 for(mode=0;mode<3;mode=mode+1)begin
  reset_all;old_writes=writes;
  if(mode==0)query_ready=0;
  if(mode==1)latency=4120;
  if(mode==2)config_write_ready=0;
  drive(0,mode==2?'h404:'h424,mode==2,32'h12345678,15);repeat(8)tick;
  drive(1,'h180,0,0,15);
  begin:await_timeout for(q=0;q<4120;q=q+1)begin #1;if(h0_ack)disable await_timeout;tick;end end
  if(!h0_ack||!h0_fault_event||h0_dat_r!==32'hffffffff)$fatal(1,"timeoutACK missing");
  tick;drive(0,'h1000,0,32'h76543210,0);query_ready=1;config_write_ready=1;latency=3;
  got0=0;got1=0;
  begin:handover for(q=0;q<100;q=q+1)begin
   #1;
   if(h0_ack)begin if(h0_dat_r!==32'h5a69c387)$fatal(1,"stale response");tick;withdraw(0);got0=1;end
   else if(h1_ack)begin tick;withdraw(1);got1=1;end
   else tick;
   if(got0&&got1)disable handover;
  end end
  if(!got0||!got1||writes!=old_writes)$fatal(1,"timeout handover/late write");
  repeat(50)tick;timeouts=timeouts+1;handoffs=handoffs+2;
 end
 // Two continuous producers, changing payload immediately after sampledACK.
 reset_all;
 for(n=0;n<250;n=n+1)begin
  drive(0,'h80+(n%13)*4,n[0],$random(seed),n[3:0]);
  drive(1,'h180+(n%13)*4,~n[0],$random(seed),~n[3:0]);got0=0;got1=0;
  begin:both for(q=0;q<100;q=q+1)begin
   #1;
   if(h0_ack)begin tick;withdraw(0);got0=1;end
   else if(h1_ack)begin tick;withdraw(1);got1=1;end
   else tick;
   if(got0&&got1)disable both;
  end end
  if(!got0||!got1)$fatal(1,"concurrent hosts stalled");repeat(2)tick;handoffs=handoffs+2;
 end
 $display("PASS held request cycles=%0d transactions=%0d timeout_handoffs=%0d aborts=%0d handoffs=%0d",cycles,transactions,timeouts,aborts,handoffs);
 $finish;
end
endmodule
