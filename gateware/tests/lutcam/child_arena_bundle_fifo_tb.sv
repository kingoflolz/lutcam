`timescale 1ns/1ps

// Independent stream scoreboard: expand an accepted epoch immediately into
// child, merge, read commands. A bundle remains occupied until its last command
// is consumed, regardless of output stalls or commands in subsequent epochs.
module child_arena_bundle_fifo_tb #(
  parameter int DEPTH = 128,
  parameter int ADMISSION_RESERVE = 22,
  parameter int ID_BITS = 7,
  parameter int FEATURE_WIDTH = 34
);
  localparam int OCCUPANCY_WIDTH = $clog2(DEPTH + 1);
  localparam int MAX_COMMANDS = 131072;

  logic clk = 0;
  logic rst = 0;
  logic in_valid = 0;
  logic in_ready, admit_ready;
  logic in_child_valid = 0;
  logic in_child_init = 0;
  logic [ID_BITS-1:0] in_child_id = 0;
  logic [FEATURE_WIDTH-1:0] in_child_fv = 0;
  logic in_merge_valid = 0;
  logic [ID_BITS-1:0] in_merge_id_a = 0, in_merge_id_b = 0;
  logic in_read_valid = 0;
  logic [ID_BITS-1:0] in_read_id = 0;
  logic out_valid;
  logic out_ready = 0;
  logic [1:0] out_op;
  logic [ID_BITS-1:0] out_id_a, out_id_b;
  logic [FEATURE_WIDTH-1:0] out_fv;
  logic [OCCUPANCY_WIDTH-1:0] occupancy;

  ChildArenaBundleFifo #(
    .ID_BITS(ID_BITS), .FEATURE_WIDTH(FEATURE_WIDTH), .DEPTH(DEPTH),
    .ADMISSION_RESERVE(ADMISSION_RESERVE)
  ) dut (.*);

  logic [1:0] expected_op [0:MAX_COMMANDS-1];
  logic [ID_BITS-1:0] expected_a [0:MAX_COMMANDS-1];
  logic [ID_BITS-1:0] expected_b [0:MAX_COMMANDS-1];
  logic [FEATURE_WIDTH-1:0] expected_fv [0:MAX_COMMANDS-1];
  logic expected_last [0:MAX_COMMANDS-1];
  integer expected_head = 0, expected_tail = 0, expected_bundles = 0;
  integer cycles = 0, accepted_bundles = 0, consumed_commands = 0;
  integer commands_by_op [0:3];
  integer full_replacements = 0, maximum_occupancy = 0;
  integer stalled_cycles = 0, reset_count = 0;
  logic [FEATURE_WIDTH-1:0] observed_feature_ones = '0;
  logic [FEATURE_WIDTH-1:0] observed_feature_zeroes = '0;
  logic last_input_accepted = 0;
  logic was_stalled = 0;
  logic [2+2*ID_BITS+FEATURE_WIDTH-1:0] stalled_payload;
  logic [31:0] random_state = 32'h6712a53d;
  integer selected_case = 0;
  integer seed_argument = 0;

  function automatic logic [31:0] next_random();
    logic [31:0] value;
    begin
      value = random_state;
      value = value ^ (value << 13);
      value = value ^ (value >> 17);
      value = value ^ (value << 5);
      random_state = value;
      next_random = value;
    end
  endfunction

  task automatic append_command(
    input logic [1:0] operation,
    input logic [ID_BITS-1:0] id_a,
    input logic [ID_BITS-1:0] id_b,
    input logic [FEATURE_WIDTH-1:0] feature,
    input logic last
  );
    begin
      if (expected_tail >= MAX_COMMANDS) $fatal(1, "scoreboard capacity exceeded");
      expected_op[expected_tail] = operation;
      expected_a[expected_tail] = id_a;
      expected_b[expected_tail] = id_b;
      expected_fv[expected_tail] = feature;
      expected_last[expected_tail] = last;
      expected_tail = expected_tail + 1;
    end
  endtask

  task automatic step();
    logic push, pop, pop_last;
    begin
      // All inputs change while clk is low; inspect the complete combinational
      // handshake before the active edge and the registered occupancy after it.
      #4;
      cycles = cycles + 1;
      last_input_accepted = 0;
      if (rst) begin
        // Reset is synchronous. Old handshakes may remain visible until the
        // reset edge, but neither an input nor an output transfer takes effect.
        expected_head = 0;
        expected_tail = 0;
        expected_bundles = 0;
        was_stalled = 0;
        reset_count = reset_count + 1;
      end else begin
        if (int'(occupancy) != expected_bundles)
          $fatal(1, "cycle %0d occupancy before edge actual=%0d expected=%0d",
                 cycles, occupancy, expected_bundles);
        if (out_valid !== (expected_bundles != 0))
          $fatal(1, "cycle %0d output-valid timing changed before edge", cycles);
        if (admit_ready !== (expected_bundles < DEPTH - ADMISSION_RESERVE))
          $fatal(1, "cycle %0d admission threshold actual=%b occupancy=%0d",
                 cycles, admit_ready, expected_bundles);
        if (was_stalled && (!out_valid ||
            {out_op, out_id_a, out_id_b, out_fv} !== stalled_payload))
          $fatal(1, "cycle %0d output changed under backpressure", cycles);
        if (out_valid && expected_head == expected_tail)
          $fatal(1, "cycle %0d unexpected output from empty FIFO", cycles);
        pop = out_valid && out_ready;
        pop_last = pop && expected_last[expected_head];
        if (in_ready !== ((expected_bundles < DEPTH) || pop_last))
          $fatal(1, "cycle %0d in_ready=%b occupancy=%0d pop_last=%b",
                 cycles, in_ready, expected_bundles, pop_last);
        push = in_valid && in_ready;
        if (push) begin
          if (!(in_child_valid || in_merge_valid || in_read_valid))
            $fatal(1, "test attempted an empty input epoch");
          if (in_child_valid) begin
            append_command(in_child_init ? 2'd3 : 2'd0,
                           in_child_id, '0, in_child_fv,
                           !(in_merge_valid || in_read_valid));
            observed_feature_ones = observed_feature_ones | in_child_fv;
            observed_feature_zeroes = observed_feature_zeroes | ~in_child_fv;
          end
          if (in_merge_valid)
            append_command(2'd1, in_merge_id_a, in_merge_id_b, '0, !in_read_valid);
          if (in_read_valid)
            append_command(2'd2, in_read_id, '0, '0, 1'b1);
          if (expected_bundles == DEPTH) full_replacements = full_replacements + 1;
          expected_bundles = expected_bundles + 1;
          accepted_bundles = accepted_bundles + 1;
          last_input_accepted = 1;
        end
        if (pop) begin
          if (out_op !== expected_op[expected_head] || out_id_a !== expected_a[expected_head])
            $fatal(1, "cycle %0d command %0d got op/id=%0d/%0d expected=%0d/%0d",
                   cycles, expected_head, out_op, out_id_a,
                   expected_op[expected_head], expected_a[expected_head]);
          if (out_op == 2'd1 && out_id_b !== expected_b[expected_head])
            $fatal(1, "cycle %0d merge second ID mismatch", cycles);
          if ((out_op == 2'd0 || out_op == 2'd3) && out_fv !== expected_fv[expected_head])
            $fatal(1, "cycle %0d child feature mismatch", cycles);
          if (expected_last[expected_head]) expected_bundles = expected_bundles - 1;
          commands_by_op[int'(out_op)] = commands_by_op[int'(out_op)] + 1;
          expected_head = expected_head + 1;
          consumed_commands = consumed_commands + 1;
        end
        was_stalled = out_valid && !out_ready;
        if (was_stalled) begin
          stalled_payload = {out_op, out_id_a, out_id_b, out_fv};
          stalled_cycles = stalled_cycles + 1;
        end
      end
      #1 clk = 1;
      #1;
      if (int'(occupancy) != expected_bundles)
        $fatal(1, "cycle %0d occupancy after edge actual=%0d expected=%0d",
               cycles, occupancy, expected_bundles);
      if (rst && (out_valid !== 1'b0 || in_ready !== 1'b1 || admit_ready !== 1'b1))
        $fatal(1, "synchronous reset did not restore empty FIFO handshakes");
      // Check the newly exposed head immediately after every edge, including
      // empty-to-first and one-entry/full simultaneous replacements.
      if (out_valid !== (expected_bundles != 0))
        $fatal(1, "cycle %0d output-valid timing changed after edge", cycles);
      if (out_valid) begin
        if ({out_op, out_id_a, out_id_b, out_fv} !==
            {expected_op[expected_head], expected_a[expected_head],
             expected_b[expected_head], expected_fv[expected_head]})
          $fatal(1, "cycle %0d post-edge head data mismatch", cycles);
      end else if ({out_op, out_id_a, out_id_b, out_fv} !== '0)
        $fatal(1, "cycle %0d invalid output fields must be zero", cycles);
      if (expected_bundles > maximum_occupancy) maximum_occupancy = expected_bundles;
      #4 clk = 0;
    end
  endtask

  task automatic bundle(input integer fields, input integer identity, input logic init);
    begin
      in_valid = 1;
      in_child_valid = (fields & 1) != 0;
      in_merge_valid = (fields & 2) != 0;
      in_read_valid = (fields & 4) != 0;
      in_child_init = init;
      in_child_id = ID_BITS'(identity);
      in_merge_id_a = ID_BITS'(identity + 17);
      in_merge_id_b = ID_BITS'(identity + 39);
      in_read_id = ID_BITS'(identity + 73);
      in_child_fv = (FEATURE_WIDTH'(identity) << (FEATURE_WIDTH - 2)) |
          FEATURE_WIDTH'({16'(identity * 193), 16'(identity * 313 + 1)});
    end
  endtask

  task automatic idle_input();
    begin
      in_valid = 0;
      in_child_valid = 0;
      in_merge_valid = 0;
      in_read_valid = 0;
    end
  endtask

  task automatic reset_fifo();
    begin
      rst = 1;
      step();
      step();
      rst = 0;
      idle_input();
      out_ready = 0;
      step();
    end
  endtask

  task automatic drain();
    integer remaining;
    begin
      idle_input();
      out_ready = 1;
      remaining = 3 * DEPTH + 8;
      while ((expected_bundles != 0 || out_valid) && remaining > 0) begin
        step();
        remaining = remaining - 1;
      end
      if (expected_bundles != 0 || expected_head != expected_tail || out_valid)
        $fatal(1, "drain failed: %0d bundles, %0d commands remain",
               expected_bundles, expected_tail - expected_head);
      repeat (4) step();
    end
  endtask

  task automatic chronological_cases();
    integer mask, repetition;
    begin
      // The old separate queues could prioritize the newer ADD ahead of these
      // earlier MERGE and READ operations. They must stay in epoch order.
      bundle(2, 1, 0); step();
      bundle(4, 2, 0); step();
      bundle(1, 3, 0); step();
      bundle(7, 4, 1); step();
      idle_input();
      repeat (9) step();
      drain();
      for (repetition = 0; repetition < 4; repetition = repetition + 1) begin
        out_ready = 0;
        for (mask = 1; mask <= 7; mask = mask + 1) begin
          bundle(mask, 7 * repetition + mask, (repetition & 1) != 0);
          step();
          if (!last_input_accepted) $fatal(1, "small directed batch unexpectedly full");
        end
        idle_input();
        repeat (3) step();
        // Repeated fields across adjacent epochs must never collapse or reorder.
        drain();
      end
      // A registered RAM head needs an explicit bypass when its only epoch is
      // consumed on the same edge as a new epoch arrives. Cover every possible
      // replacement shape, including an INIT, with no intervening empty cycle.
      for (mask = 1; mask <= 7; mask = mask + 1) begin
        out_ready = 1;
        bundle(1, 1200 + mask, 0); step();
        if (expected_bundles != 1) $fatal(1, "single-item setup failed");
        bundle(mask, 1300 + mask, 1); step();
        if (!last_input_accepted || expected_bundles != 1)
          $fatal(1, "single-item replacement changed bundle count");
        drain();
      end
      if (commands_by_op[0] == 0 || commands_by_op[1] == 0 ||
          commands_by_op[2] == 0 || commands_by_op[3] == 0)
        $fatal(1, "directed test missed an opcode");
    end
  endtask

  task automatic full_and_reserved_capacity();
    integer index;
    begin
      out_ready = 0;
      // Keep a three-command epoch at the head so the full queue must reject
      // replacement until the READ (the last command), not its ADD or MERGE.
      for (index = 0; index < DEPTH - ADMISSION_RESERVE; index = index + 1) begin
        bundle(index == 0 ? 7 : 1, index, 0);
        step();
        if (!last_input_accepted) $fatal(1, "admitted epoch rejected");
      end
      if (admit_ready || expected_bundles != DEPTH - ADMISSION_RESERVE)
        $fatal(1, "admission did not close at reserve boundary");
      // Model the maximum promised late arrivals after upstream admission stops.
      for (index = 0; index < ADMISSION_RESERVE; index = index + 1) begin
        bundle(1 + (index % 7), 400 + index, (index & 1) != 0);
        step();
        if (!last_input_accepted || admit_ready)
          $fatal(1, "reserved late arrival %0d rejected or admission reopened", index);
      end
      if (expected_bundles != DEPTH) $fatal(1, "reserved arrivals failed to fill FIFO");
      bundle(7, 601, 1);
      repeat (5) begin
        step();
        if (last_input_accepted) $fatal(1, "full stalled FIFO accepted input");
      end
      out_ready = 1;
      step();
      if (last_input_accepted) $fatal(1, "full FIFO popped bundle on child command");
      step();
      if (last_input_accepted) $fatal(1, "full FIFO popped bundle on merge command");
      step();
      if (!last_input_accepted || expected_bundles != DEPTH)
        $fatal(1, "full FIFO failed simultaneous last-command pop and push");
      // Exercise many replacements at wraparound before finally draining.
      for (index = 0; index < 4 * DEPTH; index = index + 1) begin
        if (last_input_accepted) bundle(1 + (index % 7), 900 + index, (index & 1) != 0);
        step();
      end
      drain();
      if (maximum_occupancy != DEPTH || full_replacements < DEPTH)
        $fatal(1, "full/wrap coverage insufficient peak=%0d replacements=%0d",
               maximum_occupancy, full_replacements);
    end
  endtask

  task automatic reset_cases();
    integer index;
    begin
      for (index = 0; index < 3; index = index + 1) begin
        out_ready = 0;
        bundle(7, 200 + index, 1); step();
        bundle(7, 300 + index, 0); step();
        idle_input();
        out_ready = 1;
        repeat (index) step();
        out_ready = 0;
        repeat (3) step();
        // Abort at each serializer phase, including a stable stalled payload.
        reset_fifo();
        bundle(4, 700 + index, 0); step();
        drain();
      end
      out_ready = 0;
      for (index = 0; index < DEPTH; index = index + 1) begin
        bundle(7, index, 0); step();
      end
      // Reset wins over valid input and an otherwise enabled output handshake.
      bundle(1, 999, 1);
      out_ready = 1;
      reset_fifo();
      bundle(7, 1001, 1); step();
      drain();
    end
  endtask

  task automatic randomized_bursts();
    integer index;
    logic [31:0] value, feature_low, feature_high;
    begin
      for (index = 0; index < 24000; index = index + 1) begin
        // Hold a producer request unchanged until accepted. Long blocked bursts
        // reach full capacity, interleaved with randomly stalled consumption.
        if (!in_valid || last_input_accepted) begin
          value = next_random();
          if (value[2:0] == 0) begin
            idle_input();
          end else begin
            bundle(1 + int'(value[15:8]) % 7, int'(value[30:16]), value[7]);
            feature_low = next_random();
            feature_high = next_random();
            in_child_fv = (FEATURE_WIDTH'(feature_high) << 32) |
                FEATURE_WIDTH'(feature_low);
          end
        end
        value = next_random();
        out_ready = (index % 1600 >= 450) && (value[2:0] != 0);
        step();
      end
      drain();
      if (maximum_occupancy != DEPTH || full_replacements < 100 || stalled_cycles < 1000)
        $fatal(1, "random coverage insufficient peak=%0d full=%0d stalled=%0d",
               maximum_occupancy, full_replacements, stalled_cycles);
      if (accepted_bundles < 5000 || consumed_commands < 10000)
        $fatal(1, "random stream too short bundles=%0d commands=%0d",
               accepted_bundles, consumed_commands);
      if (observed_feature_ones !== {FEATURE_WIDTH{1'b1}} ||
          observed_feature_zeroes !== {FEATURE_WIDTH{1'b1}})
        $fatal(1, "random accepted children did not exercise both states of every feature bit");
    end
  endtask

  initial begin
    for (integer operation = 0; operation < 4; operation = operation + 1)
      commands_by_op[operation] = 0;
    if ($value$plusargs("CASE=%d", selected_case)) begin end
    if ($value$plusargs("SEED=%d", seed_argument)) random_state = 32'(seed_argument);
    if (random_state == 0) $fatal(1, "random seed must be nonzero");
    if (DEPTH <= ADMISSION_RESERVE ||
        (ADMISSION_RESERVE != 22 && ADMISSION_RESERVE != 23))
      $fatal(1, "test requires depth above the 22/23-epoch centroid reserve");
    if (ID_BITS < 1 || FEATURE_WIDTH < 34)
      $fatal(1, "test feature patterns require at least 34 feature bits");
    reset_fifo();
    case (selected_case)
      0: chronological_cases();
      1: full_and_reserved_capacity();
      2: reset_cases();
      3: randomized_bursts();
      default: $fatal(1, "unknown case %0d", selected_case);
    endcase
    $display("PASS ChildArenaBundleFifo depth=%0d case=%0d id_bits=%0d feature_width=%0d cycles=%0d bundles=%0d commands=%0d peak=%0d full_replacements=%0d stalls=%0d resets=%0d",
             DEPTH, selected_case, ID_BITS, FEATURE_WIDTH, cycles, accepted_bundles, consumed_commands,
             maximum_occupancy, full_replacements, stalled_cycles, reset_count);
    $finish;
  end
endmodule
