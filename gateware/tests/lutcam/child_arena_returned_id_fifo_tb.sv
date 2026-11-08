`timescale 1ns/1ps
module child_arena_returned_id_fifo_tb #(
  parameter integer WIDTH = 9,
  parameter integer PRELOAD_COUNT = 512
);
  logic clk = 0, rst = 0, w_en = 0, r_en = 0;
  logic [WIDTH-1:0] w_data = 0, r_data;
  logic w_rdy, r_rdy;
  ChildArenaReturnedIdFifo #(.WIDTH(WIDTH), .PRELOAD_COUNT(PRELOAD_COUNT)) dut (.*);
  logic [WIDTH-1:0] expected [0:1023];
  integer head = 0, tail = 0, count = 0, prefix = 0;
  integer cycles = 0, pushes = 0, ram_pops = 0, prefix_pops = 0;
  integer empty_refills = 0, single_replacements = 0, full_replacements = 0, resets = 0;
  integer seed = 1729275197;
  logic [31:0] random_state = 32'h6712a53d;

  function automatic logic [31:0] random_word();
    logic [31:0] value;
    begin
      value = random_state;
      value = value ^ (value << 13);
      value = value ^ (value >> 17);
      value = value ^ (value << 5);
      random_state = value;
      random_word = value;
    end
  endfunction

  task automatic check();
    logic ready;
    logic [WIDTH-1:0] data;
    begin
      ready = (prefix < PRELOAD_COUNT) || count != 0;
      data = prefix < PRELOAD_COUNT ? WIDTH'(prefix) : expected[head % 1024];
      if (w_rdy !== 1'b1 || r_rdy !== ready)
        $fatal(1, "cycle%0d ready mismatch w=%b r=%b prefix=%0d count=%0d", cycles,w_rdy,r_rdy,prefix,count);
      // ReturnedIdFifo intentionally leaves data unspecified when not ready.
      if (ready && r_data !== data)
        $fatal(1, "cycle%0d data mismatch got=%h expected=%h prefix=%0d count=%0d",cycles,r_data,data,prefix,count);
    end
  endtask

  task automatic step();
    logic pop_ram, pop_prefix;
    begin
      #4;
      cycles = cycles + 1;
      if (rst) begin
        head = 0; tail = 0; count = 0; prefix = 0;
        resets = resets + 1;
      end else begin
        check();
        pop_prefix = r_en && (prefix < PRELOAD_COUNT);
        pop_ram = r_en && (prefix == PRELOAD_COUNT) && (count != 0);
        if (w_en && count == 1024 && !pop_ram) $fatal(1,"test attempted overflow");
        if (w_en && count == 0) empty_refills = empty_refills + 1;
        if (w_en && pop_ram && count == 1) single_replacements = single_replacements + 1;
        if (w_en && pop_ram && count == 1024) full_replacements = full_replacements + 1;
        if (pop_prefix) begin prefix = prefix + 1; prefix_pops = prefix_pops + 1; end
        if (pop_ram) begin head = head + 1; count = count - 1; ram_pops = ram_pops + 1; end
        if (w_en) begin
          expected[tail % 1024] = w_data;
          tail = tail + 1; count = count + 1; pushes = pushes + 1;
        end
      end
      #1 clk = 1;
      #1; check();
      #4 clk = 0;
    end
  endtask

  task automatic reset_fifo();
    begin
      // Both requests are active: reset must win without consuming prefix0
      // or allowing the asserted write to reappear after reset.
      rst = 1; w_en = 1; r_en = 1; w_data = '1; step(); step();
      rst = 0; w_en = 0; r_en = 0; step();
    end
  endtask

  task automatic drain_prefix();
    begin
      w_en = 0; r_en = 1;
      while (prefix < PRELOAD_COUNT) step();
    end
  endtask

  task automatic drain();
    begin
      drain_prefix();
      while (count != 0) step();
      repeat (4) step();
    end
  endtask

  initial begin : test
    integer i;
    logic [31:0] choice;
    if ($value$plusargs("SEED=%d", seed)) random_state = 32'(seed);
    if (PRELOAD_COUNT < 0 || PRELOAD_COUNT > 1024) $fatal(1,"unsupported test prefix");
    reset_fifo();
    // Fill RAM while initial IDs are still offered. Prefix consumption must
    // leave every returned word untouched, including the exact last-ID edge.
    r_en = 0; w_en = 1;
    for (i = 0; i < 1024; i = i + 1) begin w_data = WIDTH'(i * 37 + 13); step(); end
    drain_prefix();
    // Simultaneous full pop+push, crossing both modulo pointers four times.
    w_en = 1; r_en = 1;
    for (i = 0; i < 4096; i = i + 1) begin w_data = WIDTH'(random_word()); step(); end
    drain();
    // Empty push with r_en already asserted, followed by one-word replacement.
    w_en = 1; r_en = 1;
    for (i = 0; i < 4097; i = i + 1) begin w_data = WIDTH'(random_word()); step(); end
    drain();
    // Repeated empty/refill with no valid bubble following an accepted word.
    for (i = 0; i < 64; i = i + 1) begin
      w_en = 1; r_en = 1; w_data = WIDTH'(random_word()); step();
      w_en = 0; step(); step();
    end
    // Reset mutated RAM, then consume the initial prefix again while returns
    // arrive. Reset cannot rewind into stale RAM contents.
    reset_fifo();
    for (i = 0; i < PRELOAD_COUNT + 35; i = i + 1) begin
      r_en = 1; w_en = i < 31; w_data = WIDTH'(i * 53 + 7); step();
    end
    drain();
    for (i = 0; i < 30000; i = i + 1) begin
      choice = random_word();
      r_en = choice[0] || choice[2];
      w_en = choice[1] || choice[3];
      if (count == 1024 && !(r_en && prefix == PRELOAD_COUNT)) w_en = 0;
      w_data = WIDTH'(random_word());
      rst = (i % 4093 == 4077);
      step();
    end
    rst = 0; drain();
    if (full_replacements < 4096 || single_replacements < 4096 || empty_refills < 64 || resets < 3)
      $fatal(1,"insufficient directed coverage");
    $display("PASS ReturnedId width=%0d preload=%0d cycles=%0d pushes=%0d ram_pops=%0d prefix_pops=%0d empty_refills=%0d single_replacements=%0d full_replacements=%0d resets=%0d",WIDTH,PRELOAD_COUNT,cycles,pushes,ram_pops,prefix_pops,empty_refills,single_replacements,full_replacements,resets);
    $finish;
  end
endmodule
