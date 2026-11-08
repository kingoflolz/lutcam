module shared_refine_tb;
logic clk=0;always #5 clk=!clk;
logic rst=1;logic[1:0] camera_reset=0;
logic[1:0] in_valid=0,in_group_end=0,in_centroid_valid=0;
logic[1:0][14:0] in_x_q4=0;logic[1:0][13:0] in_y_q4=0;
logic[1:0][7:0] in_log_mass_q5_3=0,in_frame_seq=0;
logic[1:0][1:0] in_scale=0;
logic[1:0][31:0] cfg_inv_fx_q2_30,cfg_inv_fy_q2_30,cfg_cx_q16_16,cfg_cy_q16_16,cfg_printed_width_q16_16;
logic[1:0][15:0] cfg_mean_residual_limit_q16='{default:16'd512};
logic[1:0][7:0] cfg_max_mass_spread='{default:8'd64};
logic[1:0][15:0] cfg_min_baseline_q4='{default:16'd16};
wire config_read_valid;wire[3:0] config_read_address;
logic config_read_ready=0,config_response_valid=0;logic[31:0] config_read_data=0;
logic[1:0] out_ready=0,centroid_ready=0,clear_diagnostics=0;
wire[1:0] out_valid,out_last,centroid_valid,centroid_last,busy,candidate_start,candidate_done;
wire[31:0] out_data;wire[32:0] centroid_data;
wire[1:0][5:0] fifo_level;wire[1:0][9:0] candidate_cycles;
wire[1:0] fifo_drop_sticky,protocol_error_sticky;
wire[1:0][31:0] debug_status,debug_protocol,debug_drop;
SharedLfTagDecoder #(.EXPORT_ACCEPTED_CENTROIDS(1'b1),.EXTERNAL_CONFIG(1'b1)) dut(.*);
logic[31:0] vectors[348];
integer which_case,cycle=0,config_delay=0;
logic[3:0] pending_address;
integer words[2],groups[2],frame[2],point[2],starts[2],finishes[2];
integer tag[2],packets[2];
logic seen[12];logic accepted_packet[2];
integer fit_count=0,fit_completed=0,overlap=0;
integer pause_word=-1,pause_point=-1;
logic blocked_out[2],blocked_point[2];logic[32:0] held_out[2];logic[33:0] held_point[2];
integer fit_begin=0,fit_cycles=0,pose_cycles=0,max_fit_cycles=0,max_pose_cycles=0;
// Mass rank does not identify a data cell. Exercise different permutations
// and both anchor orders while keeping the two large anchors last.
function automatic integer point_source(input integer f,n);
 return n<7 ? 2+((n*(f%6+1)+f)%7) : ((n-7)^(f&1));
endfunction
always @(negedge clk)begin
 out_ready[0]=(cycle%7)>1 && !(pause_word==words[0]);out_ready[1]=(cycle%11)>2;
 centroid_ready[0]=(cycle%5)>1 && !(pause_point==point[0]);centroid_ready[1]=(cycle%13)>3;
 config_read_ready=config_delay==0 && (cycle%5)!=0;
end
always @(posedge clk)begin
 cycle=cycle+1;
 config_response_valid<=0;
 if(config_read_valid&&config_read_ready)begin pending_address<=config_read_address;config_delay<=3+(cycle%4);end
 else if(config_delay>0)begin
  config_delay<=config_delay-1;
  if(config_delay==1)begin
   config_response_valid<=1;
   case(pending_address)
    0:config_read_data<=32'h3333;
    1:config_read_data<=512;
    7:config_read_data<=32'h1040;
    8,12:config_read_data<=cfg_inv_fx_q2_30[pending_address[2]];
    9,13:config_read_data<=cfg_inv_fy_q2_30[pending_address[2]];
    10,14:config_read_data<=cfg_cx_q16_16[pending_address[2]];
    11,15:config_read_data<=cfg_cy_q16_16[pending_address[2]];
    default:$fatal(1,"bad configuration address");
   endcase
  end
 end
 if(!rst)begin
  if(dut.backend.state==dut.backend.FIT_START&&dut.backend.fit_ready)begin fit_count++;fit_begin=cycle;end
  if(dut.backend.state==dut.backend.FIT_WAIT&&dut.backend.fit_done)begin
   integer f,h,expected,error;
   fit_completed++;fit_cycles=cycle-fit_begin;
   if(fit_cycles>=1000)$fatal(1,"fit latency exceeded 1000 cycles: %0d",fit_cycles);
   if(fit_cycles>max_fit_cycles)max_fit_cycles=fit_cycles;
   if(!dut.backend.fit_ok)$fatal(1,"fit rejected valid fixture");
   f=-1;for(integer k=0;k<12;k++)if(vectors[k*29][13:0]==dut.backend.tag_id)f=k;
   if(f<0)$fatal(1,"unknown tag id");
   for(integer n=0;n<9;n++)begin
    h=$signed(dut.backend.bank_a[n]);
    expected=$signed(vectors[f*29+20+n]);error=h-expected;
    if(error>2||error< -2)$fatal(1,"nine point H mismatch fixture=%0d coefficient=%0d actual=%0d expected=%0d",f,n,h,expected);
   end
  end
  if(dut.backend.state==dut.backend.POSE_WAIT&&dut.backend.pose_done)begin
   pose_cycles=cycle-fit_begin;
   if(pose_cycles>max_pose_cycles)max_pose_cycles=pose_cycles;
  end
  if(!dut.core_idle && dut.backend.state==dut.backend.FIT_WAIT)overlap++;
  for(integer c=0;c<2;c++)begin
   if(camera_reset[c])begin words[c]=0;point[c]=0;blocked_out[c]=0;blocked_point[c]=0;end
   else begin
    if(candidate_start[c])starts[c]++;
    if(candidate_done[c])finishes[c]++;
    if(blocked_out[c]&&(!out_valid[c]||held_out[c]!=={out_last[c],out_data}))$fatal(1,"stalled packet changed");
    blocked_out[c]=out_valid[c]&&!out_ready[c];held_out[c]={out_last[c],out_data};
    if(blocked_point[c]&&(!centroid_valid[c]||held_point[c]!=={centroid_last[c],centroid_data}))$fatal(1,"stalled centroid changed");
    blocked_point[c]=centroid_valid[c]&&!centroid_ready[c];held_point[c]={centroid_last[c],centroid_data};
    if(out_valid[c]&&out_ready[c])begin
     if(!busy[c])$fatal(1,"busy released before packet");
     if(words[c]==0)tag[c]=int'(out_data);
     if(words[c]==1)begin
      frame[c]=int'(out_data[27:20])-1;accepted_packet[c]=out_data[31];
      if(frame[c]<0||frame[c]>=12)$fatal(1,"bad frame");
      if(accepted_packet[c])begin
       if(tag[c]!=int'(vectors[frame[c]*29]))$fatal(1,"ID or camera ownership mismatch");
       if(out_data[29:28]!=vectors[frame[c]*29+1][1:0])$fatal(1,"scale mismatch");
      end
     end
     if(out_last[c])begin
      if(words[c]!=(accepted_packet[c]?21:7))$fatal(1,"packet length");
      words[c]=0;packets[c]++;
      if(!accepted_packet[c])seen[frame[c]]=1;
     end else words[c]++;
    end
    if(centroid_valid[c]&&centroid_ready[c])begin
     integer source,offset;logic[32:0] expected;
     source=point_source(frame[c],point[c]);
     offset=vectors[frame[c]*29+1]==1?20544:(vectors[frame[c]*29+1]==2?25792:0);
     expected={1'b0,14'(vectors[frame[c]*29+3+2*source]),1'b0,15'(vectors[frame[c]*29+2+2*source]+offset),2'b01};
     if(centroid_data!==expected)$fatal(1,"centroid mismatch frame=%0d point=%0d got=%h want=%h",frame[c],point[c],centroid_data,expected);
     if(centroid_last[c]!=(point[c]==8))$fatal(1,"centroid length");
     if(centroid_last[c])begin point[c]=0;groups[c]++;seen[frame[c]]=1;end else point[c]++;
    end
   end
  end
  if(|fifo_drop_sticky)$fatal(1,"unexpected FIFO drop");
 end
end

task automatic tick(input integer n=1);repeat(n)@(negedge clk);endtask
task automatic send(input integer f,input bit valid_points=1);
 integer c,k;c=f%2;
 for(integer n=0;n<9;n++)begin
  k=point_source(f,n);
  in_x_q4[c]=15'(vectors[f*29+2+2*k]);in_y_q4[c]=14'(vectors[f*29+3+2*k]);
  in_log_mass_q5_3[c]=n<7?8'(64+n):8'(80+n-7);
  in_frame_seq[c]=8'(f+1);in_scale[c]=vectors[f*29+1][1:0];
  in_valid[c]=1;in_group_end[c]=n==8;in_centroid_valid[c]=valid_points;tick();
 end
 in_valid[c]=0;in_group_end[c]=0;
endtask
task automatic reset_camera(input integer c);camera_reset[c]=1;tick(3);camera_reset[c]=0;tick(3);endtask
task automatic drain;
 for(integer n=0;n<3000000;n++)begin tick();if(busy==0)break;end
 if(busy!=0)$fatal(1,"drain timeout");tick(5);
endtask
initial begin
 $readmemh("shared_vectors.hex",vectors);
 for(integer c=0;c<2;c++)begin
  cfg_inv_fx_q2_30[c]=c==0?1533917:1851279;cfg_inv_fy_q2_30[c]=c==0?1470879:1731842;
  cfg_cx_q16_16[c]=(c==0?640:620)<<16;cfg_cy_q16_16[c]=(c==0?400:390)<<16;
  cfg_printed_width_q16_16[c]=32'h3333;
  words[c]=0;groups[c]=0;frame[c]=0;point[c]=0;starts[c]=0;finishes[c]=0;tag[c]=0;packets[c]=0;
  accepted_packet[c]=0;blocked_out[c]=0;blocked_point[c]=0;held_out[c]=0;held_point[c]=0;
 end
 for(integer n=0;n<12;n++)seen[n]=0;
 if(!$value$plusargs("CASE=%d",which_case))which_case=0;
 tick(5);rst=0;tick(2);
 case(which_case)
  0:begin
   for(integer f=0;f<12;f++)send(f);drain();
   for(integer f=0;f<12;f++)if(!seen[f])$fatal(1,"missing result %0d",f);
   if(fit_count!=12||overlap==0)$fatal(1,"missing fits or frontend/backend overlap");
  end
  1,2:begin
   send(0);send(1);while(dut.job_count!=2)tick();
   reset_camera(which_case-1);send(which_case+1);drain();
   if(!seen[which_case==1?1:0]||!seen[which_case+1]||seen[which_case-1])$fatal(1,"reset crossed camera boundary");
  end
  3:begin
   pause_word=0;send(0);send(1);while(!out_valid[0])tick();tick(30);
   reset_camera(0);pause_word=-1;send(2);drain();
   if(!seen[1]||!seen[2]||seen[0])$fatal(1,"stalled packet reset lost peer");
  end
  4:begin
   pause_point=4;send(0);send(1);while(!centroid_valid[0]||point[0]!=4)tick();tick(30);
   reset_camera(0);pause_point=-1;send(2);drain();
   if(!seen[1]||!seen[2]||seen[0])$fatal(1,"centroid reset lost peer");
  end
  5:begin
   send(0,0);send(1);drain();
   if(!seen[0]||!seen[1]||fit_count!=1||groups[0]!=0)$fatal(1,"reject ran refinement or was lost");
  end
  6:begin
   send(0);send(1);while(dut.job_count!=2)tick();
   for(integer n=0;n<32;n++)begin send(0);send(1);end
   tick(5);if(fifo_level[0]!=32||fifo_level[1]!=32)$fatal(1,"candidate queue capacity changed");
   drain();if(groups[0]!=33||groups[1]!=33||fit_count!=66)$fatal(1,"backlog lost candidates");
  end
  default:$fatal(1,"unknown case");
 endcase
 $display("PASS refined shared case=%0d fits=%0d max_fit_cycles=%0d max_fit_plus_pose_cycles=%0d overlap=%0d",which_case,fit_count,max_fit_cycles,max_pose_cycles,overlap);$finish;
end
initial begin #50000000;$fatal(1,"global timeout");end
endmodule
