`timescale 1ns/1ps
module tb;
reg clk=0,rst=1,identity_ready=0;
always #5 clk=!clk;
reg [1:0] slot=1;
wire boot_done,trap,bus_fault;
wire [3:0] boot_error;
wire record_first,record_last;
wire flash_start,flash_cs_n,flash_busy;
wire [7:0] flash_tx;
reg flash_done=0;
reg [7:0] flash_rx=0;
wire [47:0] board_mac;
wire [31:0] mac_word0={board_mac[23:16],board_mac[31:24],board_mac[39:32],board_mac[47:40]};
wire [31:0] mac_word1={16'b0,board_mac[7:0],board_mac[15:8]};
wire [8:0] record_count;
wire [29:0] p_adr;
wire [31:0] p_dat_w;
wire [3:0] p_sel;
wire p_cyc,p_stb,p_we;
reg p_ack=0,p_err=0;
reg [31:0] p_dat_r=0;
reg [29:0] host_adr=0;
reg [31:0] host_dat_w=0;
reg [3:0] host_sel=15;
reg host_cyc=0,host_stb=0,host_we=0;
wire host_ack,host_err;
wire [31:0] host_dat_r;
reg net_rx_valid=0,net_rx_last=0,net_tx_ready=1;
reg [31:0] net_rx_data=0;
reg [15:0] net_rx_length=0;
wire net_rx_ready,net_tx_valid,net_tx_last;
wire [31:0] net_tx_data,net_tx_ip_address,net_ip,net_mask,net_router;
wire [15:0] net_tx_length;
wire [2:0] net_state;
wire net_revoke;
fixture dut(.*);
reg [31:0] flash_words[0:1027];
reg [31:0] mode_words[0:130],tx_words[0:66],rx_words[0:69];
integer init_count[0:1];
integer tx_index=0,tx_packets=0,min_sp=4096,stack_bound=0;
reg reboot_requested=0,capture_revoked=0,tx_held=0;
reg [31:0] held_data;
reg held_last;
integer flash_count=0;
reg [7:0] flash_opcode=0;
reg [23:0] flash_address=0;
reg p_seen=0;
reg [31:0] i2c_id=0,i2c_result=0,i2c_rx=0,descriptor=0,write_byte=0;
reg [7:0] sensor[0:1][0:65535];
integer hist_bin[0:1],hist_value[0:1],launches=0,operations=0;
integer which=0,selected=1,cycles=0,cam,off,index,regno,lines,gain,brightness;
integer forced_brightness=-1;
reg pair_run=0,fail_stage=0,fail_end=0;
integer end_attempts=0;
reg [31:0] response;
always @(posedge clk) begin
 cycles<=cycles+1;
 if(net_revoke)capture_revoked<=1;
 if(boot_done && dut.picorv32.reg_pc>=8)begin
  if(dut.picorv32.cpuregs[2]<min_sp)min_sp=dut.picorv32.cpuregs[2];
  if(dut.picorv32.cpuregs[2]>4096 || dut.picorv32.cpuregs[2]<4096-stack_bound)$fatal(1,"stack exceeds checked call-path bound");
 end
 if(tx_held && (!net_tx_valid || net_tx_data!==held_data || net_tx_last!==held_last))$fatal(1,"DHCP TX changed under backpressure");
 tx_held<=net_tx_valid && !net_tx_ready;held_data<=net_tx_data;held_last<=net_tx_last;
 if(net_tx_valid && net_tx_ready)begin
  if(net_tx_length!=268 || tx_index>66)$fatal(1,"DHCP payload length");
  tx_words[tx_index]=net_tx_data;
  if(net_tx_last)begin
   if(tx_index!=66)$fatal(1,"DHCP truncated TX");
   if(tx_words[0]!==32'h00060101 || tx_words[59]!==32'h63538263 ||
      tx_words[7]!==mac_word0 || tx_words[8]!==mac_word1)
    $fatal(1,"DHCP BOOTP header, cookie or client identity corrupted");
   tx_index=0;tx_packets=tx_packets+1;
  end else tx_index=tx_index+1;
 end
 if(cycles>24000000)$fatal(1,"supervisor timeout, trap=%d bus_fault=%d boot=%d pc=%h",trap,bus_fault,boot_error,dut.picorv32.reg_pc);
 flash_done<=0;p_ack<=0;
 if(flash_cs_n)flash_count=0;
 if(flash_start && which!=5)begin
  if(flash_cs_n)$fatal(1,"SPI byte without chip select");
  flash_done<=1;flash_rx<=0;
  if(flash_count==0)begin
   flash_opcode=flash_tx;
   if(flash_tx!=8'h4b && flash_tx!=8'h03)$fatal(1,"boot flash write opcode");
  end else if(flash_count<=3)begin
   flash_address={flash_address[15:0],flash_tx};
   if(flash_count==3 && flash_opcode==3 && flash_address!=selected*32'h200000+32'h180000)$fatal(1,"wrong firmware slot");
  end
  if(flash_opcode==8'h4b && flash_count>=5)flash_rx<=flash_count-5;
  if(flash_opcode==8'h03 && flash_count>=4)begin
   index=(flash_count-4)/4;
   flash_rx<=flash_words[index]>>((flash_count-4)%4*8);
  end
  flash_count=flash_count+1;
 end
 if(!p_cyc || !p_stb)p_seen<=0;
 if(p_cyc && p_stb && !p_seen)begin
  if(which==10)$fatal(1,"invalid pointer escaped onto the peripheral bus");
  p_seen<=1;p_ack<=1;p_dat_r<=0;
  off=(p_adr<<2)&4095;
  if((p_adr<<2)>=32'h30020000 && (p_adr<<2)<32'h30022000)begin
   cam=((p_adr<<2)>>12)&1;
   if(p_we)begin
    if(record_first!==(off==128) || record_last!==(off==128+(record_count-1)*4))$fatal(1,"I2C record metadata");
    if(off==132)descriptor<=p_dat_w;
    if(off==136 && record_count==4)write_byte<=p_dat_w;
    if(record_last)begin
     operations<=operations+1;
     regno=(descriptor>>8)&65535;
     if((descriptor&127)!=(cam==0 ? 96 : 16))$fatal(1,"wrong strapped sensor address");
     if(!(descriptor&128) && regno==16'h3208 && write_byte==16)end_attempts<=end_attempts+1;
     i2c_id<=p_dat_w;i2c_result<=0;
     if(descriptor&128)i2c_rx<=sensor[cam][regno];
     else if(fail_stage && regno==16'h3501)begin i2c_result<=1;fail_stage<=0;end
     else if(fail_end && cam==1 && regno==16'h3208 && write_byte==16)begin i2c_result<=1;fail_end<=0;end
     else begin
      if(init_count[cam]<131)begin
       if({8'b0,regno[15:0],write_byte[7:0]}!==mode_words[init_count[cam]])$fatal(1,"compressed initialization changed register sequence");
       init_count[cam]=init_count[cam]+1;
      end
      sensor[cam][regno]<=write_byte;
      if(regno==16'h3208 && write_byte==160)begin
       launches<=launches+1;
       $display("exposure launch cam=%0d lines=%0d gain=%0d",cam,(sensor[cam][16'h3501]<<4)|(sensor[cam][16'h3502]>>4),sensor[cam][16'h3509]);
      end
     end
    end
   end else case(off)
    20:p_dat_r<=0;
    32:p_dat_r<=i2c_id;
    36:p_dat_r<=i2c_id+1;
    40:p_dat_r<=i2c_result;
    96:p_dat_r<=i2c_rx;
    default:p_dat_r<=0;
   endcase
  end else if((p_adr<<2)>=32'h30022000 && (p_adr<<2)<32'h30024000)begin
   cam=((p_adr<<2)>>12)&1;
   if(p_we && off==0)begin
    hist_bin[cam]<=0;
    lines=(sensor[cam][16'h3501]<<4)|(sensor[cam][16'h3502]>>4);
    gain=sensor[cam][16'h3509];brightness=lines*gain/128;
    if(forced_brightness>=0)brightness=forced_brightness;
    if(brightness>255)brightness=255;
    hist_value[cam]<=brightness*2;
   end
   if(!p_we && off==4)p_dat_r<=1;
   if(!p_we && off==8)begin
    p_dat_r<=((hist_bin[cam]==0)?32'h80000000:0)|((hist_bin[cam]==511)?32'h40000000:0)
       |(hist_bin[cam]<<20)|((hist_bin[cam]==hist_value[cam])?(which==8 ? 511999 : 512000):0);
    hist_bin[cam]<=hist_bin[cam]+1;
   end
  end else if((p_adr<<2)==32'h30026008)p_dat_r<=selected;
  else if((p_adr<<2)==32'h30026024)begin
   if(p_we)begin
    if(p_dat_w==9 || p_dat_w==0 || p_dat_w[1:0]!=2'(selected))$fatal(1,"invalid restart slot/token");
    if(!capture_revoked)$fatal(1,"restart did not revoke capture first");
    reboot_requested<=1;
   end else p_dat_r<=9;
  end else if((p_adr<<2)==32'h30024208)p_dat_r<=pair_run;
  else if((p_adr<<2)==32'h3002420c)p_dat_r<=16;
  else if((p_adr<<2)==32'h30000008 || (p_adr<<2)==32'h30010008)p_dat_r<=4;
 end
end
task access(input bit wr,input integer offset,input reg[31:0] data);
 integer n;
 @(negedge clk);host_adr=(32'h30028000+offset)>>2;host_we=wr;host_dat_w=data;host_cyc=1;host_stb=1;
 n=0;
 while(1)begin
  @(posedge clk);#1;n++;
  if(host_err)$fatal(1,"mailbox error at %h",offset);
  if(host_ack)begin response=host_dat_r;break;end
  if(n>5000)$fatal(1,"mailbox timeout");
 end
 @(negedge clk);host_cyc=0;host_stb=0;
 repeat(3)@(posedge clk);
endtask
task wait_ready;
 do begin access(0,8,0);end while(response!=1 && response!=4);
endtask
task command(input integer seq,input integer op,input integer a,input integer b);
 access(1,20,op);access(1,24,a);access(1,28,b);access(1,16,seq);
 do begin access(0,32,0);end while(response!=seq && !reboot_requested);
endtask
task dhcp_reply(input integer kind);
 for(integer j=0;j<70;j++)rx_words[j]=0;
 rx_words[0]=32'h00060102;rx_words[1]=tx_words[1];rx_words[4]=32'h4201a8c0;
 rx_words[7]=mac_word0;rx_words[8]=mac_word1;rx_words[59]=32'h63538263;
 rx_words[60]=32'h00000135|(kind<<16);rx_words[61]=32'h04360000;rx_words[62]=32'hfe01a8c0;
 rx_words[63]=32'h04330000;rx_words[64]=32'h08000000;
 rx_words[65]=32'h04010000;rx_words[66]=32'h00ffffff;
 rx_words[67]=32'h04030000;rx_words[68]=32'hfe01a8c0;rx_words[69]=255;
 for(integer j=0;j<70;j++)begin
  @(negedge clk);net_rx_valid=1;net_rx_data=rx_words[j];net_rx_last=(j==69);net_rx_length=280;
  do @(posedge clk);while(!net_rx_ready);
 end
 @(negedge clk);net_rx_valid=0;net_rx_last=0;
endtask
initial begin
 if($value$plusargs("CASE=%d",which))begin end
 if($value$plusargs("SLOT=%d",selected))begin end
 if($value$plusargs("STACK_BOUND=%d",stack_bound))begin end
 if(stack_bound==0 || stack_bound>160)$fatal(1,"missing/invalid stack bound");
 init_count[0]=0;init_count[1]=0;
 slot=2'(selected);
 if(which==10)$readmemh("invalid-pointer.hex",flash_words);
 else $readmemh("flash.hex",flash_words);
 $readmemh("mode.hex",mode_words);
 if(which==1)flash_words[20]=flash_words[20]^1;
 if(which==2)flash_words[0]=0;
 if(which==4)flash_words[1]=32'h10000;
 sensor[0][16'h300a]=8'h92;sensor[0][16'h300b]=8'h81;
 sensor[1][16'h300a]=8'h92;sensor[1][16'h300b]=8'h81;
 repeat(5)@(posedge clk);@(negedge clk);rst=0;identity_ready=1;
 if(which==1 || which==2 || which==4 || which==5)begin
  wait(boot_error!=0);
  repeat(100)@(posedge clk);
  if(boot_done || operations || trap || bus_fault)$fatal(1,"invalid code executed");
  if(boot_error != (which==1 ? 7 : which==5 ? 2 : 6))$fatal(1,"wrong flash rejection reason");
  $display("PASS supervisor rejects corrupt/incomplete flash case %0d",which);$finish;
 end
 if(which==10)begin
  wait(bus_fault);repeat(10)@(posedge clk);
  if(!boot_done || operations || trap)$fatal(1,"invalid pointer fault handling");
  $display("PASS supervisor rejects nonlocal pointer before system bus");$finish;
 end
 wait_ready;
 if(response!=1 || trap || bus_fault)$fatal(1,"firmware initialization failed: %h",response);
 if(sensor[0][16'h3808]!=5 || sensor[1][16'h3809]!=0 || sensor[0][16'h3823]!=48)$fatal(1,"sensor mode not installed");
 if(init_count[0]!=131 || init_count[1]!=131)$fatal(1,"incomplete sensor initialization");
 if(which==9)begin
  net_tx_ready=0;
  wait(net_tx_valid);repeat(1000)@(posedge clk);
  @(negedge clk);net_tx_ready=1;
  wait(tx_packets==1);
  if(tx_words[0]!=32'h00060101 || tx_words[7]!=mac_word0 || tx_words[8]!=mac_word1 || tx_words[60][23:16]!=1)$fatal(1,"bad DHCP discover DMA");
  dhcp_reply(2);wait(tx_packets==2);
  if(tx_words[60][23:16]!=3 || tx_words[63]!=32'h4201a8c0 || tx_words[65]!=32'hfe01a8c0)$fatal(1,"bad DHCP request");
  dhcp_reply(5);wait(net_state==3);
  if(net_ip!=32'hc0a80142 || net_mask!=32'hffffff00 || net_router!=32'hc0a801fe)$fatal(1,"bad DHCP configuration");
  wait(tx_packets==3);
  if(net_tx_ip_address!=32'hc0a801fe || tx_words[3]!=32'h4201a8c0 || tx_words[62]!=255)$fatal(1,"bad DHCP renewal");
  wait(net_ip==0);wait(net_state==1);wait(tx_packets==4);
  if(tx_words[60][23:16]!=1 || !capture_revoked)$fatal(1,"expired lease did not restart DHCP");
  $display("PASS supervisor DHCP on actual RV32EC, stack peak %0d",4096-min_sp);$finish;
 end
 command(1,3,0,0);
 if(sensor[0][16'h0100]!=1 || sensor[1][16'h0100]!=1)$fatal(1,"stream on failed");
 if(which==3 || which==7)begin
  fail_stage=(which==3);fail_end=(which==7);command(2,5,600,16);access(0,8,0);
  if(response!=4 || launches!=0)$fatal(1,"partial group launched");
  if(!reboot_requested || end_attempts!=(which==3 ? 0 : 2))$fatal(1,"failed transaction did not restart directly");
  /* Model the boot controller reloading the selected slot. The real CPU
     and flash loader must initialize successfully again, with no retry of
     the partially staged exposure transaction. */
  @(negedge clk);rst=1;init_count[0]=0;init_count[1]=0;pair_run=0;
  repeat(5)@(posedge clk);@(negedge clk);rst=0;reboot_requested=0;
  wait_ready;
  if(response!=1 || launches!=0 || init_count[0]!=131 || init_count[1]!=131)$fatal(1,"restart did not restore initialization");
  access(0,56,0);if(response!=1)$fatal(1,"restart did not restore initialized firmware data");
 end else if(which==8)begin
  pair_run=1;wait_ready;
  do begin access(0,8,0);end while(response!=4);
  access(0,12,0);if(response!=6 || launches!=0)$fatal(1,"invalid first histogram was not reported");
  wait(reboot_requested);
 end else if(which==6)begin
  command(2,5,1,16);forced_brightness=0;command(3,6,1,192);pair_run=1;
  do begin access(0,36,0);end while(response<2);
  if(response>885)$fatal(1,"exposure exceeded limit");
 end else begin
  pair_run=1;
  do begin access(0,48,0);end while(response<176 || response>210);
  if(launches<2)$fatal(1,"auto exposure did not update sensor groups");
  if(sensor[0][16'h3501]!=sensor[1][16'h3501] || sensor[0][16'h3509]!=sensor[1][16'h3509])$fatal(1,"exposure not linked");
  command(2,6,1,188); /* -1 EV: percentile level should halve. */
  do begin access(0,48,0);end while(response<88 || response>105);
  access(0,44,0);if(response!=188)$fatal(1,"EV compensation not published");
  command(3,6,1,196); /* +1 EV must double the neutral energy even when clipped. */
  do begin access(0,40,0);end while(response<48);
  repeat(500000)@(posedge clk);
  access(0,40,0);if(response<48 || response>61)$fatal(1,"positive EV cancelled or ran away");
  access(0,48,0);if(response<250)$fatal(1,"positive EV did not permit intentional clipping");
  command(4,6,0,196);access(0,56,0);if(response!=0)$fatal(1,"AE disable failed");
  command(5,5,200,16);access(0,36,0);if(response!=200)$fatal(1,"manual exposure failed");
  pair_run=0;command(6,4,0,0);
  if(sensor[0][16'h0100]!=0 || sensor[1][16'h0100]!=0)$fatal(1,"stream off failed");
 end
 if(trap || bus_fault)$fatal(1,"CPU fault");
 $display("PASS supervisor actual RV32EC firmware, slot %0d case %0d, %0d cycles, %0d I2C operations, stack peak %0d",selected,which,cycles,operations,4096-min_sp);$finish;
end
endmodule
