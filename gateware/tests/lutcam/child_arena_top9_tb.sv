`timescale 1ns/1ps
module child_arena_top9_tb #(
  parameter int ENTRY_COUNT = 108,
  parameter int FEATURE_WIDTH = 34,
  parameter bit BUNDLED_COMMANDS = 1'b0,
  localparam int ID_BITS = lrcca_util_pkg::bit_width(longint'(ENTRY_COUNT) + 1)
);
  typedef logic [FEATURE_WIDTH-1:0] feature_t;
  logic clk = 0;
  always #5 clk = !clk;
  logic rst = 1;
  logic new_valid = 0, new_ready, new_rsp_valid, new_rsp_ready = 0;
  logic [ID_BITS-1:0] new_rsp_id;
  logic new_rsp_status;
  logic cmd_valid = 0, cmd_ready;
  logic [1:0] cmd_op = 0;
  logic [ID_BITS-1:0] cmd_id_a = 0, cmd_id_b = 0;
  feature_t cmd_fv = 0, out_fv;
  logic out_valid, out_ready = 0, out_first, out_last, idle;
  logic [ID_BITS-1:0] free_count;
  int silent_reads = 0;
  int checked_features = 0;

  // Match the production full-log-mass comparator and its three-bit coarse
  // tail rank. Distinct payloads above bit 7 make tie order observable.
  ChildArena #(
    .ENTRY_COUNT(ENTRY_COUNT), .MAX_LIST_LEN(9), .FEATURE_WIDTH(FEATURE_WIDTH),
    .SORT_KEY_WIDTH(8), .TAIL_KEY_LSB(5), .TAIL_KEY_WIDTH(3),
    .TAIL_KEY_DENSE_AREA_RANK(0), .EQUAL_KEY_SERVICE_MIN(1),
    .CMD_QUEUE_DEPTH(512), .BUNDLED_COMMANDS(BUNDLED_COMMANDS)
  ) dut (
    .clk, .rst, .new_valid, .new_ready, .new_alloc_id(),
    .new_rsp_valid, .new_rsp_ready, .new_rsp_id, .new_rsp_status,
    .cmd_valid, .cmd_ready, .cmd_op, .cmd_id_a, .cmd_id_b, .cmd_fv,
    .bundle_child_valid(BUNDLED_COMMANDS && (cmd_op == 2'd0 || cmd_op == 2'd3)),
    .bundle_merge_valid(BUNDLED_COMMANDS && cmd_op == 2'd1),
    .bundle_merge_id_a(cmd_id_a), .bundle_merge_id_b(cmd_id_b),
    .bundle_read_valid(BUNDLED_COMMANDS && cmd_op == 2'd2),
    .bundle_read_id(cmd_id_a),
    .cmd_admit_ready(), .cmd_queue_occupancy(),
    .out_valid, .out_ready, .out_fv, .out_first, .out_last, .idle, .free_count
  );

  always @(posedge clk)
    if (!rst && dut.read_launch && !dut.read_launch_emit)
      silent_reads++;

  function automatic feature_t fv(input int payload, input int sort_key);
    fv = (feature_t'(payload) << 8) | feature_t'(sort_key);
  endfunction

  task automatic tick;
    @(posedge clk); #1; @(negedge clk); #1;
  endtask

  task automatic wait_idle;
    int cycles;
    cycles = 0;
    while (!idle) begin
      tick();
      cycles++;
      if (cycles > 2000) $fatal(1, "ChildArena did not become idle");
    end
  endtask

  task automatic command(input int op, input int a, input int b, input feature_t value);
    while (!cmd_ready) tick();
    cmd_op = 2'(op);
    cmd_id_a = ID_BITS'(a);
    cmd_id_b = ID_BITS'(b);
    cmd_fv = value;
    cmd_valid = 1;
    tick();
    cmd_valid = 0;
  endtask

  task automatic new_list(input feature_t first, output int id);
    wait_idle();
    while (!new_ready) tick();
    new_valid = 1;
    tick();
    new_valid = 0;
    while (!new_rsp_valid) tick();
    if (!new_rsp_status) $fatal(1, "allocation failed");
    id = int'(new_rsp_id);
    new_rsp_ready = 1;
    tick();
    new_rsp_ready = 0;
    command(3, id, 0, first);
    wait_idle();
  endtask

  task automatic add(input int id, input feature_t value);
    command(0, id, 0, value);
    wait_idle();
  endtask

  task automatic assert_free(input int expected);
    if (int'(free_count) != expected)
      $fatal(1, "free_count %0d, expected %0d", free_count, expected);
  endtask

  task automatic read_expected(input int id, input feature_t expected [9]);
    feature_t held_fv;
    logic held_first, held_last;
    logic [ID_BITS-1:0] held_free;
    int cycles;
    out_ready = 0;
    command(2, id, 0, '0);
    for (int index = 0; index < 9; index++) begin
      cycles = 0;
      while (!out_valid) begin
        tick();
        cycles++;
        if (cycles > 2000) $fatal(1, "READ timed out at feature %0d", index);
      end
      if (out_fv !== expected[index])
        $fatal(1, "feature %0d was payload %0d key %0d, expected payload %0d key %0d",
          index, out_fv >> 8, out_fv[7:0], expected[index] >> 8, expected[index][7:0]);
      if (out_first !== (index == 0) || out_last !== (index == 8))
        $fatal(1, "invalid first/last flags at feature %0d", index);
      held_fv = out_fv;
      held_first = out_first;
      held_last = out_last;
      held_free = free_count;
      // Stall every beat, including both ends, and require complete stability.
      repeat (2 + index % 3) begin
        tick();
        if (!out_valid || out_fv !== held_fv || out_first !== held_first ||
            out_last !== held_last || free_count !== held_free)
          $fatal(1, "READ changed or freed a node during backpressure");
      end
      out_ready = 1;
      tick();
      out_ready = 0;
      checked_features++;
    end
    wait_idle();
    if (out_valid) $fatal(1, "READ emitted more than nine features");
    assert_free(ENTRY_COUNT);
  endtask

  task automatic shuffled_nine(output int id);
    new_list(fv(1, 40), id);
    add(id, fv(2, 80));
    add(id, fv(3, 50));
    add(id, fv(4, 70));
    add(id, fv(5, 60));
    add(id, fv(6, 100));
    add(id, fv(7, 90));
    add(id, fv(8, 120));
    add(id, fv(9, 110));
    assert_free(ENTRY_COUNT - 9);
  endtask

  initial begin : run
    int a, b, silent_before;
    feature_t expected [9];
    repeat (3) tick();
    rst = 0;
    tick();
    assert_free(ENTRY_COUNT);

    // Queue twelve lower/equal children on consecutive clocks. The very first
    // discarded child may immediately dispatch the next command. None may
    // replace the head's payload, even though its payload word is larger.
    shuffled_nine(a);
    for (int index = 0; index < 12; index++)
      command(0, a, 0, fv(10+index,39+index%2));
    wait_idle();
    assert_free(ENTRY_COUNT - 9);
    expected = '{fv(1,40), fv(3,50), fv(5,60), fv(4,70), fv(2,80),
                 fv(7,90), fv(6,100), fv(9,110), fv(8,120)};
    read_expected(a, expected);

    // Exercise both full-list head replacement and recycled-node insertion.
    // A new key 100 precedes the existing 100 under the service-min policy.
    shuffled_nine(a);
    add(a, fv(12, 45));
    add(a, fv(13, 100));
    add(a, fv(14, 255));
    assert_free(ENTRY_COUNT - 9);
    expected = '{fv(5,60), fv(4,70), fv(2,80), fv(7,90), fv(13,100),
                 fv(6,100), fv(9,110), fv(8,120), fv(14,255)};
    read_expected(a, expected);

    // Merge eighteen children with equal keys on both sides of the retained
    // boundary. While dropping, ties discard the noncanonical list first;
    // once retaining, ties choose the list with more entries remaining.
    new_list(fv(101,40), a);
    add(a,fv(102,40)); add(a,fv(103,40)); add(a,fv(104,50)); add(a,fv(105,60));
    add(a,fv(106,70)); add(a,fv(107,80)); add(a,fv(108,90)); add(a,fv(109,100));
    new_list(fv(201,40), b);
    add(b,fv(202,40)); add(b,fv(203,50)); add(b,fv(204,50)); add(b,fv(205,60));
    add(b,fv(206,70)); add(b,fv(207,80)); add(b,fv(208,90)); add(b,fv(209,100));
    if (a >= b) $fatal(1, "test requires first head to be canonical");
    assert_free(ENTRY_COUNT - 18);
    silent_before = silent_reads;
    command(1, a, b, '0);
    wait_idle();
    if (silent_reads != silent_before) $fatal(1, "equal-key merge used fast-drop path");
    assert_free(ENTRY_COUNT - 9);
    expected = '{fv(105,60), fv(106,70), fv(206,70), fv(107,80), fv(207,80),
                 fv(108,90), fv(208,90), fv(109,100), fv(209,100)};
    read_expected(a, expected);

    // Donor's complete unsaturated rank is strictly below canonical head.
    // Verify that the silent read path really retires all nine donor nodes.
    new_list(fv(301,100), a);
    for (int index = 1; index < 9; index++) add(a, fv(301+index,100+index));
    new_list(fv(401,1), b);
    for (int index = 1; index < 9; index++) add(b, fv(401+index,1+index));
    silent_before = silent_reads;
    command(1, a, b, '0);
    wait_idle();
    if (silent_reads != silent_before + 1) $fatal(1, "strict-low merge missed fast-drop path");
    assert_free(ENTRY_COUNT - 9);
    expected = '{fv(301,100), fv(302,101), fv(303,102), fv(304,103), fv(305,104),
                 fv(306,105), fv(307,106), fv(308,107), fv(309,108)};
    read_expected(a, expected);

    // Both coarse tail ranks saturate at seven. The exact keys still select
    // the correct nine, but the coarse summary cannot authorize fast-drop.
    new_list(fv(501,240), a);
    for (int index = 1; index < 9; index++) add(a, fv(501+index,240+index));
    new_list(fv(601,224), b);
    for (int index = 1; index < 9; index++) add(b, fv(601+index,224+index));
    silent_before = silent_reads;
    command(1, a, b, '0);
    wait_idle();
    if (silent_reads != silent_before) $fatal(1, "saturated tail used fast-drop path");
    assert_free(ENTRY_COUNT - 9);
    expected = '{fv(501,240), fv(502,241), fv(503,242), fv(504,243), fv(505,244),
                 fv(506,245), fv(507,246), fv(508,247), fv(509,248)};
    read_expected(a, expected);

    $display("PASS ChildArena literal top-nine selection: %0d payloads, stalled outputs, free_count restored",
      checked_features);
    $finish;
  end

  initial begin
    #10000000;
    $fatal(1, "ChildArena top-nine test timed out");
  end
endmodule
