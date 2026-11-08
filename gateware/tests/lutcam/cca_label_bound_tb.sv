// Exercise the public TwoPPC ingress and its actual label RAM interfaces.
// S0 masks both edge pixels, so 847 runs (labels 0..846) is the attainable
// production maximum. Depth 848 also covers the terminal speculative read 847.
module cca_label_bound_tb #(
  parameter int TABLE_DEPTH = 848
);
  localparam int WIDTH = 1696;
  localparam int HEIGHT = 8;
  localparam int PAIRS = WIDTH/2;
  logic clk = 0;
  always #5 clk = !clk;
  logic rst = 1, valid = 0, d0 = 0, d1 = 0, sof = 0, eol = 0;
  logic [3:0] feature_y = 0;
  wire [1:0] ready;
  wire [1:0][31:0] peak_runs, peak_allocated, peak_counter, peak_issue, checks;
  wire [1:0][31:0] peak_next_read;
  wire [1:0][31:0] checked_rows;
  logic audit_memory = 0;
  integer case_id = 0;
  integer sent_pairs = 0;
  logic [31:0] random_state = 32'h96d4a173;

  for (genvar mode=0; mode<2; mode++) begin : modes
    wire fv_valid;
    wire [9:0] fv_data;
    TwoPPC #(
      .WIDTH(WIDTH), .HEIGHT(HEIGHT), .TABLE_DEPTH(TABLE_DEPTH),
      .HOLE_FILL(mode != 0), .FEATURE_BBOX(0), .FEATURE_AREA(0),
      .FEATURE_PRESENT(1), .FEATURE_MOMENTS(0), .FEATURE_VALUE_WIDTH(1),
      .CHILD_COUNT_BITS(9), .CHILD_ID_MODE(1), .EXPOSE_S5_FV(1)
    ) dut (
      .clk, .rst, .datavalid(valid), .in_ready(ready[mode]), .d0, .d1,
      .feature_d0(d0), .feature_d1(d1), .feature_value_d0(1'b0),
      .feature_value_d1(1'b0), .feature_y, .sof, .eol,
      .child_add0(1'b0), .child_add1(1'b0), .child_id0(9'd0), .child_id1(9'd0),
      .child_parent0_valid(), .child_parent1_valid(), .child_parent0_id(),
      .child_parent1_id(), .child_parent0_has_child(), .child_parent1_has_child(),
      .child_merge_valid(), .child_merge_id_a(), .child_merge_id_b(),
      .s5_fv_valid(), .s5_fv_lane(), .s5_fv_data(),
      .s5_close_moment_valid(), .s5_close_moment_lane(), .s5_close_moment_data(),
      .fv_valid, .fv_data, .fv_child_id(), .fv_child_valid()
    );

    integer maximum_runs = 0, maximum_allocated = 0, maximum_counter = 0;
    integer maximum_issue = 0, address_checks = 0, total_rows = 0;
    integer maximum_next_read = 0;
    integer row_runs[0:HEIGHT-1];
    integer starts = 0, closed = 0, issues = 0;
    logic previous_pixel = 0, previous_s1_end = 0, previous_s5_end = 0;
    assign peak_runs[mode] = 32'(maximum_runs);
    assign peak_allocated[mode] = 32'(maximum_allocated);
    assign peak_counter[mode] = 32'(maximum_counter);
    assign peak_issue[mode] = 32'(maximum_issue);
    assign peak_next_read[mode] = 32'(maximum_next_read);
    assign checks[mode] = 32'(address_checks);
    assign checked_rows[mode] = 32'(total_rows);

    task automatic check_label(input logic [9:0] value, input string field_name);
      if ($isunknown(value) || int'(value) >= TABLE_DEPTH)
        $fatal(1, "CCA_LABEL_CAPACITY depth=%0d mode=%0d case=%0d %s=%0d",
          TABLE_DEPTH, mode, case_id, field_name, value);
      address_checks++;
    endtask

    always @(posedge clk) begin
      if (rst) begin
        starts = 0; closed = 0; issues = 0; previous_pixel = 0;
        previous_s1_end = 0; previous_s5_end = 0;
        for (integer row=0; row<HEIGHT; row++) row_runs[row] = 0;
      end else begin
        if (valid && !ready[mode]) $fatal(1, "test violated ingress ready");
        // Independent run count from the processed topology, before labeling.
        if (dut.s0.valid) begin
          if (dut.s0.row_start && (previous_pixel || dut.s0.d0))
            $fatal(1, "nonzero left boundary mode=%0d", mode);
          if (!previous_pixel && dut.s0.d0) starts++;
          if (!dut.s0.d0 && dut.s0.d1) starts++;
          previous_pixel = dut.s0.d1;
          if (dut.s0.row_end) begin
            if (previous_pixel || starts > 847)
              $fatal(1, "processed row run bound mode=%0d starts=%0d", mode, starts);
            row_runs[int'(dut.s0.row)] = starts;
            starts = 0; previous_pixel = 0;
          end
        end

        // Observe the unguarded issuer: a guarded RAM must not hide overflow.
        if (previous_s1_end && dut.s1_stage.p_issue != 0)
          $fatal(1, "S1 did not reset at row end mode=%0d", mode);
        previous_s1_end = dut.s1.valid && dut.s1.row_end;
        if (dut.s1.valid) begin
          if (int'(dut.s1_stage.p_issue) != issues || issues > 847)
            $fatal(1, "S1 counter wrap mode=%0d got=%0d expected=%0d", mode,
              dut.s1_stage.p_issue, issues);
          if (issues > maximum_issue) maximum_issue = issues;
          if (dut.s1.pf_valid) begin
            check_label(10'(dut.s1.pf_index), "s1.pf_index");
            issues++;
          end
          if (dut.s1.row_end) issues = 0;
        end

        if (previous_s5_end && dut.s5_stage.c_reg != 0)
          $fatal(1, "S5 did not reset at row end mode=%0d", mode);
        previous_s5_end = dut.s5_stage.s5.valid && dut.s5_stage.s5.row_end;
        if (dut.s5_stage.s5.valid) begin
          if (int'(dut.s5_stage.c_reg) != closed)
            $fatal(1, "S5 counter wrap mode=%0d got=%0d expected=%0d", mode,
              dut.s5_stage.c_reg, closed);
          if (closed > maximum_counter) maximum_counter = closed;
          // Before pair k, closed runs plus a carried foreground pixel <= k.
          if (closed + int'(dut.s5_stage.s5.c) > int'(dut.s5_stage.s5.col))
            $fatal(1, "prefix run invariant mode=%0d row=%0d pair=%0d", mode,
              dut.s5_stage.s5.row, dut.s5_stage.s5.col);
          if (dut.s5_stage.lane0.close_w.valid) begin
            check_label(10'(dut.s5_stage.lane0.close_w.addr), "lane0.close.addr");
            check_label(10'(dut.s5_stage.lane0.close_w.tail), "lane0.close.tail");
            check_label(10'(dut.s5_stage.lane0.close_w.head_addr), "lane0.close.head");
            if (int'(dut.s5_stage.lane0.close_w.head_addr) > maximum_allocated)
              maximum_allocated = int'(dut.s5_stage.lane0.close_w.head_addr);
            closed++;
          end
          if (int'(dut.s5_stage.lane0.state_ret.c) != closed)
            $fatal(1, "lane0 close count mismatch mode=%0d", mode);
          if (dut.s5_stage.lane1.close_w.valid) begin
            check_label(10'(dut.s5_stage.lane1.close_w.addr), "lane1.close.addr");
            check_label(10'(dut.s5_stage.lane1.close_w.tail), "lane1.close.tail");
            check_label(10'(dut.s5_stage.lane1.close_w.head_addr), "lane1.close.head");
            if (int'(dut.s5_stage.lane1.close_w.head_addr) > maximum_allocated)
              maximum_allocated = int'(dut.s5_stage.lane1.close_w.head_addr);
            closed++;
          end
          if (int'(dut.s5_stage.lane1.state_ret.c) != closed || closed > 847)
            $fatal(1, "lane1 close count mismatch mode=%0d closed=%0d", mode, closed);
          if (closed > maximum_runs) maximum_runs = closed;
          if (dut.s5_stage.s5.row_end) begin
            if (closed != row_runs[int'(dut.s5_stage.s5.row)])
              $fatal(1, "row label count differs from topology mode=%0d row=%0d labels=%0d runs=%0d",
                mode, dut.s5_stage.s5.row, closed, row_runs[int'(dut.s5_stage.s5.row)]);
            total_rows++;
            closed = 0;
          end
        end

        // S3 next RAM port A reads even when prefetch is not valid. Check those
        // addresses too, including the legal unused terminal index847.
        check_label(10'(dut.s3_stage.next_port_a_addr0), "next.port_a0");
        check_label(10'(dut.s3_stage.next_port_a_addr1), "next.port_a1");
        if (int'(dut.s3_stage.next_port_a_addr0) > maximum_next_read)
          maximum_next_read = int'(dut.s3_stage.next_port_a_addr0);
        if (int'(dut.s3_stage.next_port_a_addr1) > maximum_next_read)
          maximum_next_read = int'(dut.s3_stage.next_port_a_addr1);
        if (dut.s3_stage.next0_en0 || dut.s3_stage.next0_en1)
          check_label(10'(dut.s6b.next0_data), "next.write0.data");
        if (dut.s3_stage.next1_en0 || dut.s3_stage.next1_en1) begin
          check_label(10'(dut.s6b.next1_addr), "next.write1.addr");
          check_label(10'(dut.s6b.next1_data), "next.write1.data");
        end
        if (dut.s3_stage.head_curr_en0 || dut.s3_stage.head_curr_en1)
          check_label(10'(dut.s6b.head_curr_addr), "head.current.addr");
        if (dut.s3_stage.head_prev_en0 || dut.s3_stage.head_prev_en1) begin
          check_label(10'(dut.s6b.head_prev_addr), "head.previous.addr");
          check_label(10'(dut.s6b.head_prev_head), "head.previous.value");
        end
        if (dut.s3.valid && dut.s3.pf_valid) begin
          check_label(10'(dut.s3.pf_index), "prefetch.index");
          check_label(10'(dut.s3.pf_next), "prefetch.next");
          check_label(10'(dut.s3.pf_head), "prefetch.decoded_head");
        end
        if (dut.s5_stage.data_r_en0)
          check_label(10'(dut.s5_stage.data_r_addr0), "data.read0");
        if (dut.s5_stage.data_r_en1)
          check_label(10'(dut.s5_stage.data_r_addr1), "data.read1");
        if (dut.s5_stage.data_w_en0)
          check_label(10'(dut.s5_stage.data_w_addr0), "data.write0");
        if (dut.s5_stage.data_w_en1)
          check_label(10'(dut.s5_stage.data_w_addr1), "data.write1");
        if (dut.s5_stage.tail_r_en)
          check_label(10'(dut.s5_stage.tail_r_addr), "tail.read");
        if (dut.s5_stage.tail_w_en) begin
          check_label(10'(dut.s5_stage.tail_w_addr), "tail.write.addr");
          check_label(10'(dut.s5_stage.tail_w_data), "tail.write.value");
        end
        if (audit_memory) begin
          for (integer index=0; index<TABLE_DEPTH; index++) begin
            check_label(10'(dut.s3_stage.next_mem0[index]), "stored.next0");
            check_label(10'(dut.s3_stage.next_mem1[index]), "stored.next1");
            if (dut.s3_stage.head_mem0[index] != {dut.LABEL_BITS{1'b1}})
              check_label(10'(dut.s3_stage.head_mem0[index]), "stored.mapped_head0");
            if (dut.s3_stage.head_mem1[index] != {dut.LABEL_BITS{1'b1}})
              check_label(10'(dut.s3_stage.head_mem1[index]), "stored.mapped_head1");
            check_label(10'(dut.s5_stage.tail_mem[index]), "stored.tail");
          end
        end
      end
    end
  end

  task automatic tick;
    @(posedge clk); #1; @(negedge clk); #1;
  endtask

  function automatic logic mask(input integer which, input integer x, input integer y);
    case (which)
      0: mask = 0;
      1: mask = 1;
      2: mask = (x & 1) == 0;
      3: mask = (x & 1) != 0;
      4: mask = ((x+y) & 1) != 0;
      5: mask = y[0] ? x[0] : 1'b1;
      6: mask = y[0] ? 1'b0 : !x[0];
      7: mask = (x == 4 || x == 1283 || x == 1608 || x == 1691 ||
                 (x > 4 && x < 1691 && (x % 3 == 0)));
      8: mask = (y == 1 || y == 6 || x == 1 || x == 1694 ||
                 (x > 8 && x < 1688 && x[0]));
      9: mask = ((x < 1280 || (x >= 1284 && x < 1604) ||
                  (x >= 1612 && x < 1692)) && x[0]);
      default: mask = 0;
    endcase
  endfunction

  task automatic send_pair(input logic a, input logic b, input logic first, input logic last);
    valid = 0;
    while (ready != 2'b11) tick();
    d0 = a; d1 = b; sof = first; eol = last; valid = 1;
    tick();
    sent_pairs++;
    valid = 0; sof = 0; eol = 0;
  endtask

  initial begin
    integer first_case, final_case;
    logic p0, p1;
    first_case = 0; final_case = 41;
    if ($test$plusargs("ONLY_WORST")) begin first_case=2; final_case=2; end
    repeat (3) tick();
    for (case_id=first_case; case_id<=final_case; case_id++) begin
      rst=1; valid=0; repeat(3)tick(); rst=0;
      for (integer row=0; row<HEIGHT; row++) begin
        feature_y=4'(row);
        for (integer pair=0; pair<PAIRS; pair++) begin
          if (case_id >= 10) begin
            random_state = random_state ^ (random_state << 13);
            random_state = random_state ^ (random_state >> 17);
            random_state = random_state ^ (random_state << 5);
            p0=random_state[0]; p1=random_state[1];
          end else begin
            p0=mask(case_id,2*pair,row); p1=mask(case_id,2*pair+1,row);
          end
          send_pair(p0,p1,row==0 && pair==0,pair==PAIRS-1);
          if (case_id >= 10 && pair%97 == 23) repeat(2)tick();
        end
        repeat(10)tick();
      end
      repeat(20)tick();
      audit_memory=1;tick();audit_memory=0;
    end
    if (peak_runs[0] != 847 || peak_runs[1] != 847 ||
        peak_allocated[0] != 846 || peak_allocated[1] != 846)
      $fatal(1,"worst-case pattern did not attain the production bound runs=%p labels=%p",
        peak_runs,peak_allocated);
    for (integer mode=0; mode<2; mode++) begin
      if (checked_rows[mode] != 8*(final_case-first_case+1))
        $fatal(1,"missing rows mode=%0d rows=%0d",mode,checked_rows[mode]);
      if (peak_counter[mode] != 846 || peak_issue[mode] != 847 || peak_next_read[mode] != 847)
        $fatal(1,"terminal counter/read peaks not reached mode=%0d c=%0d p=%0d next=%0d",
          mode,peak_counter[mode],peak_issue[mode],peak_next_read[mode]);
      $display("CCA_BOUND mode=%0d depth=%0d rows=%0d peak_runs=%0d peak_allocated=%0d peak_c_reg=%0d peak_p_issue=%0d peak_next_read=%0d label_checks=%0d",
        mode,TABLE_DEPTH,checked_rows[mode],peak_runs[mode],peak_allocated[mode],
        peak_counter[mode],peak_issue[mode],peak_next_read[mode],checks[mode]);
    end
    $display("PASS CCA label bound depth=%0d pairs=%0d",TABLE_DEPTH,sent_pairs);
    $finish;
  end
  initial begin #100000000; $fatal(1,"CCA label bound timeout"); end
endmodule
