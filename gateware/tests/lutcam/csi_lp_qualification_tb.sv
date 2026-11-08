`timescale 1ns/1ps
// Directed LP control regression, using real synchronizers, aligner, and FIFO.
// The raw model differs only in the ACTIVE LP10 diagnostic condition. Every
// control transition and all other status bits must remain cycle-identical.
// LPX/settle/heartbeat guards match the board; only the initial guard is short.
module csi_lp_qualification_tb;
  reg reset_n=0, ctrl_clk=0, byte_clk=0, byte_running=1, status_clear=0;
  always #4.166667 ctrl_clk=~ctrl_clk;
  always #5 if (byte_running) byte_clk=~byte_clk;
  reg [1:0] clock_lp=3, lp0=3, lp1=3;
  wire [15:0] status[2];
  integer cases=0, raw_skew_faults=0;
csi2_rx_raw #(.INIT_GUARD_CYCLES(4), .LPX_CYCLES(6),
    .SOT_SETTLE_CYCLES(11), .CLOCK_TIMEOUT_CYCLES(77), .PIXEL_FIFO_DEPTH(64)) raw (
    .reset_n, .ctrl_clk, .byte_clk, .pixel_clk(ctrl_clk), .status_clear(status_clear),
    .clk_lp_p(clock_lp[1]), .clk_lp_n(clock_lp[0]),
    .data_lp_p({lp1[1],lp0[1]}), .data_lp_n({lp1[0],lp0[0]}),
    .data0_hs_in(8'b0), .data1_hs_in(8'b0), .data_fifo_empty(2'b11), .status(status[0]));
csi2_rx_narrow #(.INIT_GUARD_CYCLES(4), .LPX_CYCLES(6),
    .SOT_SETTLE_CYCLES(11), .CLOCK_TIMEOUT_CYCLES(77), .PIXEL_FIFO_DEPTH(64)) narrow (
    .reset_n, .ctrl_clk, .byte_clk, .pixel_clk(ctrl_clk), .status_clear(status_clear),
    .clk_lp_p(clock_lp[1]), .clk_lp_n(clock_lp[0]),
    .data_lp_p({lp1[1],lp0[1]}), .data_lp_n({lp1[0],lp0[0]}),
    .data0_hs_in(8'b0), .data1_hs_in(8'b0), .data_fifo_empty(2'b11), .status(status[1]));

  // Assert all control and boundary timing is unchanged on every clock,
  // including the cases deliberately producing a diagnostic difference.
  reg [1:0] expect_immediate_stop;
  reg [1:0] previous_burst_end;
  always @(posedge ctrl_clk) begin
    previous_burst_end=narrow.burst_end_toggle_ctrl;
    for(integer lane=0;lane<2;lane++)
      expect_immediate_stop[lane]=reset_n && narrow.session_enable_ctrl &&
        narrow.byte_session_up_sync1 && narrow.data_state[lane]==4 &&
        narrow.data_lane_lp[lane]==3;
    #0.01;
    if(reset_n) begin
      // A production-derived raw model shares unrelated control code. Assert
      // the critical LP11 timing independently of that diagnostic comparison.
      for(integer lane=0;lane<2;lane++)
        if(expect_immediate_stop[lane] &&
           (narrow.data_state[lane]!=0 || narrow.data_hs_term[lane] ||
            narrow.data_hs_ena[lane] || narrow.data_phase_count[lane]!=0 ||
            narrow.burst_end_toggle_ctrl[lane]==previous_burst_end[lane]))
          $fatal(1,"LP11 must terminate ACTIVE immediately on lane%0d",lane);
      if ((status[0] & ~16'h0020) !== (status[1] & ~16'h0020))
        $fatal(1,"a diagnostic other than LP10 changed");
      if ({raw.clk_hs_term,raw.clk_hs_ena,raw.data_hs_term,raw.data_hs_ena,
           raw.burst_start_toggle_ctrl,raw.burst_end_toggle_ctrl,raw.session_enable_ctrl,
           raw.clock_state,raw.data_state[0],raw.data_state[1]} !==
          {narrow.clk_hs_term,narrow.clk_hs_ena,narrow.data_hs_term,narrow.data_hs_ena,
           narrow.burst_start_toggle_ctrl,narrow.burst_end_toggle_ctrl,narrow.session_enable_ctrl,
           narrow.clock_state,narrow.data_state[0],narrow.data_state[1]})
        $fatal(1,"control/entry/exit timing changed");
    end
  end
  task hold(input [1:0] v0,input [1:0] v1,input integer n);
    @(negedge ctrl_clk); lp0=v0;lp1=v1;
    repeat(n-1) @(negedge ctrl_clk);
  endtask
  task start;
    @(negedge ctrl_clk); reset_n=0; byte_running=1; status_clear=0; clock_lp=3; lp0=3;lp1=3;
    repeat(5) @(negedge ctrl_clk); reset_n=1;
    repeat(20) @(negedge ctrl_clk); clock_lp=1;
    repeat(8) @(negedge ctrl_clk); clock_lp=0;
    repeat(30) @(negedge ctrl_clk);
  endtask
  task active;
    start();hold(1,1,8);hold(0,0,30);
    if(raw.data_state[0]!=4 || raw.data_state[1]!=4) $fatal(1,"not active");
  endtask
  task check(input string name,input bit old_error,input bit new_error);
    repeat(20) @(negedge ctrl_clk);
    if(status[0][5]!=old_error || status[1][5]!=new_error)
      $fatal(1,"%s old/new=%d/%d expected=%d/%d",name,status[0][5],status[1][5],old_error,new_error);
    $display("PASS %s old/new LP=%d/%d",name,status[0][5],status[1][5]);cases++;
  endtask
  task tick(input integer n);
    repeat(n) @(negedge ctrl_clk);
  endtask
  task lane_value(input integer lane, input [1:0] value);
    if(lane==0) lp0=value; else lp1=value;
  endtask
  task wait_history(input integer lane);
    for(integer n=0;n<12;n++) begin
      @(negedge ctrl_clk);
      if(narrow.active_lp10_previous[lane]) return;
    end
    $fatal(1,"ACTIVE LP10 history never became set on lane%0d",lane);
  endtask
  task clear_status;
    @(negedge ctrl_clk);status_clear=1;
    @(negedge ctrl_clk);status_clear=0;
    tick(16);
  endtask
  task restart_clock_session;
    // No global reset: complete a real LP11 stop and a second LP entry.
    // Keep the dirty lane at LP10 until the session is definitely down.
    @(negedge ctrl_clk);clock_lp=3;
    tick(20);
    if(raw.session_enable_ctrl || narrow.session_enable_ctrl ||
       narrow.active_lp10_previous != 0)
      $fatal(1,"session stop retained enable or ACTIVE LP10 history");
    lp0=3;lp1=3;
    clear_status();
    @(negedge ctrl_clk);clock_lp=1;tick(8);clock_lp=0;tick(30);
    hold(1,1,8);hold(0,0,30);
    if(raw.data_state[0]!=4 || raw.data_state[1]!=4 ||
       narrow.active_lp10_previous != 0)
      $fatal(1,"new session did not re-enter clean ACTIVE state");
  endtask
  task reset_history_case(input integer lane);
    active();
    @(negedge ctrl_clk);lane_value(lane,2);
    wait_history(lane);
    // Assert asynchronously while history is definitely one. This catches
    // reset logic that merely happens to clear after several clock edges.
    #1;reset_n=0;#0.01;
    if(narrow.active_lp10_previous != 0)
      $fatal(1,"asynchronous reset retained lane%0d LP10 history",lane);
    active();
    if(lane==0)begin hold(2,0,1);end else begin hold(0,2,1);end
    hold(3,3,1);
    check($sformatf("lane%0d reset clears history",lane),1,0);
  endtask
  task session_history_case(input integer lane);
    active();
    @(negedge ctrl_clk);lane_value(lane,2);
    wait_history(lane);
    restart_clock_session();
    if(lane==0)begin hold(2,0,1);end else begin hold(0,2,1);end
    hold(3,3,1);
    check($sformatf("lane%0d session restart clears history",lane),1,0);
  endtask
  task boundary_case(input integer lane, input bit persistent);
    start();hold(1,1,8);hold(0,0,1);
    // Enter LP10 during SETTLE using physical pins, never forcing RTL state.
    // Two synchronizer stages make the value visible before the transition.
    for(integer n=0;n<30;n++) begin
      @(negedge ctrl_clk);
      if(raw.data_state[lane]==3 && raw.data_phase_count[lane]==7) break;
      if(n==29)$fatal(1,"SETTLE timing boundary not found");
    end
    lane_value(lane,2);
    for(integer n=0;n<12;n++) begin
      @(negedge ctrl_clk);
      if(raw.data_state[lane]==3 && raw.data_phase_count[lane]==10) break;
      if(n==11)$fatal(1,"last SETTLE cycle not found");
    end
    if(!persistent)lane_value(lane,3);
    @(negedge ctrl_clk);
    if(raw.data_state[lane]!=4 || narrow.active_lp10_previous[lane])
      $fatal(1,"SETTLE LP10 incorrectly seeded ACTIVE history");
    @(negedge ctrl_clk);
    if(!raw.data_lp_error_pulse || narrow.data_lp_error_pulse ||
       !narrow.active_lp10_previous[lane])
      $fatal(1,"first ACTIVE LP10 sample was not qualified independently");
    if(persistent)begin
      @(negedge ctrl_clk);
      if(!raw.data_lp_error_pulse || !narrow.data_lp_error_pulse)
        $fatal(1,"second ACTIVE LP10 sample was not reported");
    end
    hold(3,3,1);
    check($sformatf("lane%0d SETTLE ACTIVE boundary persistent=%0d",lane,persistent),1,persistent);
  endtask
  task sustained_clear_case(input integer lane);
    active();
    if(lane==0)begin hold(2,0,12);end else begin hold(0,2,12);end
    if(!status[0][5] || !status[1][5])$fatal(1,"sustained LP10 was not latched");
    @(negedge ctrl_clk);status_clear=1;
    @(negedge ctrl_clk);
    if(!raw.status_clear_pending_ctrl || !narrow.status_clear_pending_ctrl)
      $fatal(1,"sustained LP10 test did not accept status clear");
    if(!status[0][5] || !status[1][5] ||
       !narrow.active_lp10_previous[lane] || !narrow.data_lp_error_pulse)
      $fatal(1,"accepted status clear masked sustained ACTIVE LP10");
    status_clear=0;
    repeat(16)begin
      @(negedge ctrl_clk);
      if(!status[0][5] || !status[1][5] ||
         !narrow.active_lp10_previous[lane] || !narrow.data_lp_error_pulse)
        $fatal(1,"status clear masked sustained ACTIVE LP10");
    end
    hold(3,3,1);tick(12);clear_status();
    if(status[0][5] || status[1][5] || narrow.active_lp10_previous != 0)
      $fatal(1,"completed LP10 condition did not clear normally");
    $display("PASS lane%0d sustained ACTIVE LP10 survives status clear",lane);cases++;
    hold(1,1,8);hold(0,0,30);
    if(lane==0)begin hold(2,0,1);end else begin hold(0,2,1);end
    hold(3,3,1);
    check($sformatf("lane%0d new burst after clear has fresh history",lane),1,0);
  endtask
  task reset_wait_request;
    @(negedge ctrl_clk);reset_n=0;byte_running=1;status_clear=0;
    clock_lp=3;lp0=3;lp1=3;tick(5);reset_n=1;tick(20);
    if(raw.clock_state!=1)$fatal(1,"clock did not reach WAIT_REQUEST");
  endtask
  task clock_case(input integer kind);
    reset_wait_request();
    case(kind)
      0:clock_lp=2;
      1:clock_lp=0;
      2:begin clock_lp=1;tick(8);clock_lp=2;end
      3:begin clock_lp=1;tick(8);clock_lp=3;end
    endcase
    check($sformatf("clock-lane request error kind%0d unchanged",kind),1,1);
  endtask
  task heartbeat_error_case(input bit during_active);
    if(during_active)begin
      active();hold(2,2,12);
      if(narrow.active_lp10_previous!=3)$fatal(1,"clock-loss history was not seeded");
      byte_running=0;tick(120);
      if(!status[0][12] || !status[1][12] || status[0]!==status[1])
        $fatal(1,"clock heartbeat loss changed diagnosis");
    end else begin
      reset_wait_request();byte_running=0;clock_lp=1;tick(8);clock_lp=0;tick(120);
      if(!status[0][13] || !status[1][13] || status[0]!==status[1])
        $fatal(1,"clock initialization heartbeat timeout changed diagnosis");
    end
    if(narrow.active_lp10_previous!=0)$fatal(1,"clock session failure retained LP10 history");
    $display("PASS clock heartbeat failure active=%0d unchanged",during_active);cases++;
    byte_running=1;
  endtask
  task run_extra_cases;
    for(integer lane=0;lane<2;lane++)begin
      reset_history_case(lane);
      session_history_case(lane);
      boundary_case(lane,0);
      boundary_case(lane,1);
      sustained_clear_case(lane);
      active();
      if(lane==0)begin hold(2,0,1);hold(1,0,1);hold(2,0,1);end
      else begin hold(0,2,1);hold(0,1,1);hold(0,2,1);end
      hold(3,3,1);
      check($sformatf("lane%0d LP01 separates LP10 samples",lane),1,0);
    end
    for(integer kind=0;kind<4;kind++)clock_case(kind);
    heartbeat_error_case(0);heartbeat_error_case(1);
  endtask

  initial begin
    active();hold(2,0,1);hold(3,3,1);check("lane0 one-sample exit10",1,0);
    active();hold(0,2,1);hold(3,3,1);check("lane1 one-sample exit10",1,0);
    active();hold(2,0,2);hold(3,3,1);check("lane0 persistent exit10",1,1);
    active();hold(0,2,2);hold(3,3,1);check("lane1 persistent exit10",1,1);
    active();hold(2,0,1);hold(0,2,1);hold(3,3,1);check("independent lane history",1,0);
    active();hold(2,0,1);hold(0,0,1);hold(2,0,1);hold(3,3,1);check("separated LP10 samples",1,0);
    active();hold(1,1,1);hold(3,3,1);check("LP01 exit",0,0);
    active();hold(3,3,1);check("direct LP11 stop",0,0);
    start();hold(0,0,1);check("invalid entry skip request",1,1);
    start();hold(2,3,1);check("invalid LP10 request",1,1);
    start();hold(1,1,8);hold(3,3,1);check("invalid request cancellation",1,1);
    start();hold(1,1,8);hold(0,0,2);hold(3,3,1);check("early settle stop",1,1);
    start();hold(1,1,1);hold(0,0,30);hold(3,3,1);check("one-sample entry timing unchanged",0,0);
    for(integer phase=0;phase<9;phase++) begin
      active();
      // One physical rising edge precedes the other by 2ns. Sweep its
      // placement relative to ctrl_clk, without forcing synchronizer state.
      @(negedge ctrl_clk); #(phase);lp0=2;#2;lp0=3;lp1=3;
      repeat(20) @(negedge ctrl_clk);
      if(status[1][5]) $fatal(1,"narrow qualifier rejected 2ns skew at phase%0d",phase);
      if(status[0][5])raw_skew_faults++;
      cases++;
    end
    run_extra_cases();
    if(raw_skew_faults==0) $fatal(1,"phase sweep missed CDC mixed sample");
    $display("PASS narrow LP qualification cases=%0d raw2nsSkewFaults=%0d/9; control timing identical",cases,raw_skew_faults);
    $finish;
  end
  initial #500000 $fatal(1,"timeout");
endmodule
