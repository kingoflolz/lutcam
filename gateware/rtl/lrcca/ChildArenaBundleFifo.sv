// Queue child-arena operations by their source epoch before serializing them.
//
// Every nonempty input bundle is one epoch. All operations in an older epoch
// are emitted before any operation in a newer epoch; operations within an
// epoch are emitted in child, merge, read order. Keeping the fields together
// prevents a newer child from overtaking a pending merge or read.
module ChildArenaBundleFifo #(
  parameter int ID_BITS = 7,
  parameter int FEATURE_WIDTH = 34,
  parameter int DEPTH = 128,
  parameter int ADMISSION_RESERVE = 22,
  localparam int ADDR_BITS = (DEPTH > 1) ? $clog2(DEPTH) : 1,
  localparam int COUNT_BITS = $clog2(DEPTH + 1),
  localparam int BUNDLE_WIDTH = FEATURE_WIDTH + 4 * ID_BITS + 4
)(
  input  logic clk,
  input  logic rst,

  input  logic in_valid,
  output logic in_ready,
  output logic admit_ready,
  input  logic in_child_valid,
  input  logic in_child_init,
  input  logic [ID_BITS-1:0] in_child_id,
  input  logic [FEATURE_WIDTH-1:0] in_child_fv,
  input  logic in_merge_valid,
  input  logic [ID_BITS-1:0] in_merge_id_a,
  input  logic [ID_BITS-1:0] in_merge_id_b,
  input  logic in_read_valid,
  input  logic [ID_BITS-1:0] in_read_id,

  output logic out_valid,
  input  logic out_ready,
  output logic [1:0] out_op,
  output logic [ID_BITS-1:0] out_id_a,
  output logic [ID_BITS-1:0] out_id_b,
  output logic [FEATURE_WIDTH-1:0] out_fv,
  output logic [COUNT_BITS-1:0] occupancy
);
  localparam logic [ADDR_BITS-1:0] LAST_ADDR = ADDR_BITS'(DEPTH - 1);
  localparam logic [COUNT_BITS-1:0] CAPACITY = COUNT_BITS'(DEPTH);
  localparam logic [COUNT_BITS-1:0] ADMISSION_LIMIT =
      COUNT_BITS'(DEPTH - ADMISSION_RESERVE);
  localparam logic [1:0] PHASE_CHILD = 2'd0;
  localparam logic [1:0] PHASE_MERGE = 2'd1;
  localparam logic [1:0] PHASE_READ = 2'd2;

  // Pack epochs into independent 20-bit memory lanes. Efinity otherwise
  // splits the production 78-bit FIFO into five RAM10 blocks; these four
  // lanes use four blocks at both 128 and 512 entries. Padding is unused.
  // The serializer holds only its phase, without another copy of the head.
  localparam int STORAGE_LANES = (BUNDLE_WIDTH + 19) / 20;
  localparam int STORAGE_BITS = STORAGE_LANES * 20;
  logic [STORAGE_BITS-1:0] storage_read, storage_write;
  logic [STORAGE_BITS-1:0] memory_read, forwarded_data;
  logic forwarded_valid;
  logic [ADDR_BITS-1:0] fetch_address;
  logic unused_storage_padding;
  assign unused_storage_padding = ^(storage_read >> BUNDLE_WIDTH);
  logic [ADDR_BITS-1:0] write_ptr;
  logic [ADDR_BITS-1:0] read_ptr;
  logic [1:0] phase;
  logic [1:0] selected_phase;
  logic head_child_valid;
  logic head_child_init;
  logic [ID_BITS-1:0] head_child_id;
  logic [FEATURE_WIDTH-1:0] head_child_fv;
  logic head_merge_valid;
  logic [ID_BITS-1:0] head_merge_id_a;
  logic [ID_BITS-1:0] head_merge_id_b;
  logic head_read_valid;
  logic [ID_BITS-1:0] head_read_id;
  logic last_command;
  logic command_fire;
  logic bundle_pop;
  logic bundle_push;

  function automatic logic [ADDR_BITS-1:0] next_addr(
    input logic [ADDR_BITS-1:0] value
  );
    next_addr = (value == LAST_ADDR) ? '0 : value + ADDR_BITS'(1);
  endfunction

  assign {
      head_read_id, head_read_valid,
      head_merge_id_b, head_merge_id_a, head_merge_valid,
      head_child_fv, head_child_id, head_child_init, head_child_valid
  } = storage_read[BUNDLE_WIDTH-1:0];

  assign storage_write = STORAGE_BITS'({
      in_read_id, in_read_valid,
      in_merge_id_b, in_merge_id_a, in_merge_valid,
      in_child_fv, in_child_id, in_child_init, in_child_valid
  });
  // Prefetch the post-edge head, sharing one explicit write-through decision
  // across all physical lanes. A replacement of the only epoch and a first
  // write to an empty queue must both forward the just-accepted full epoch.
  assign fetch_address = bundle_pop ? next_addr(read_ptr) : read_ptr;
  assign storage_read = forwarded_valid ? forwarded_data : memory_read;
  always_ff @(posedge clk) begin
    if (rst) forwarded_valid <= 1'b0;
    else forwarded_valid <= bundle_push && (write_ptr == fetch_address);
  end
  // Collision forwarding is storage, so retain the payload in fabric registers.
  always_ff @(posedge clk)
    if (!rst && bundle_push) forwarded_data <= storage_write;
  for (genvar lane = 0; lane < STORAGE_LANES; lane++) begin : packed_lanes
    logic [19:0] storage [0:DEPTH-1];
    always_ff @(posedge clk) begin
      memory_read[lane*20 +: 20] <= storage[fetch_address];
      if (!rst && bundle_push)
        storage[write_ptr] <= storage_write[lane*20 +: 20];
    end
  end

  // Reset is synchronous. Keeping it out of ready/valid combinational paths
  // also permits users to derive a retirement reset from the queue being idle.
  assign out_valid = occupancy != '0;
  assign command_fire = out_valid && out_ready;
  assign bundle_pop = command_fire && last_command;
  // A final-command pop makes room immediately, including when full. A
  // nonfinal command does not release a bundle slot.
  assign in_ready = (occupancy < CAPACITY) || bundle_pop;
  assign bundle_push = in_valid && in_ready;
  // Admission is deliberately conservative and depends on occupied epochs,
  // not the number of commands or a speculative same-cycle departure.
  assign admit_ready = occupancy < ADMISSION_LIMIT;

  always_comb begin
    out_op = 2'd0;
    out_id_a = '0;
    out_id_b = '0;
    out_fv = '0;
    selected_phase = PHASE_READ;
    last_command = 1'b1;
    if (out_valid) begin
      if ((phase == PHASE_CHILD) && head_child_valid) begin
        selected_phase = PHASE_CHILD;
        out_op = head_child_init ? 2'd3 : 2'd0;
        out_id_a = head_child_id;
        out_fv = head_child_fv;
        last_command = !head_merge_valid && !head_read_valid;
      end else if ((phase != PHASE_READ) && head_merge_valid) begin
        selected_phase = PHASE_MERGE;
        out_op = 2'd1;
        out_id_a = head_merge_id_a;
        out_id_b = head_merge_id_b;
        last_command = !head_read_valid;
      end else begin
        selected_phase = PHASE_READ;
        out_op = 2'd2;
        out_id_a = head_read_id;
      end
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      write_ptr <= '0;
      read_ptr <= '0;
      occupancy <= '0;
      phase <= PHASE_CHILD;
    end else begin
      if (bundle_push) begin
        write_ptr <= next_addr(write_ptr);
      end
      if (bundle_pop) begin
        read_ptr <= next_addr(read_ptr);
        phase <= PHASE_CHILD;
      end else if (command_fire) begin
        phase <= (selected_phase == PHASE_CHILD) ? PHASE_MERGE : PHASE_READ;
      end
      unique case ({bundle_push, bundle_pop})
        2'b10: occupancy <= occupancy + COUNT_BITS'(1);
        2'b01: occupancy <= occupancy - COUNT_BITS'(1);
        default: occupancy <= occupancy;
      endcase
    end
  end

`ifndef SYNTHESIS
  initial begin
    assert (ID_BITS > 0 && FEATURE_WIDTH > 0 && DEPTH > 0)
      else $fatal(1, "ChildArenaBundleFifo widths and depth must be positive");
    assert (ADMISSION_RESERVE >= 0 && ADMISSION_RESERVE < DEPTH)
      else $fatal(1, "ChildArenaBundleFifo admission reserve must be below depth");
  end
  always_ff @(posedge clk) begin
    if (!rst) begin
      assert (!bundle_push || in_child_valid || in_merge_valid || in_read_valid)
        else $fatal(1, "ChildArenaBundleFifo accepted an empty epoch");
      assert (occupancy <= CAPACITY)
        else $fatal(1, "ChildArenaBundleFifo occupancy exceeded capacity");
      assert (!bundle_push || (occupancy < CAPACITY) || bundle_pop)
        else $fatal(1, "ChildArenaBundleFifo overflow");
      assert (!out_valid || head_child_valid || head_merge_valid || head_read_valid)
        else $fatal(1, "ChildArenaBundleFifo exposed an empty epoch");
      assert (!out_valid || (selected_phase != PHASE_READ) || head_read_valid)
        else $fatal(1, "ChildArenaBundleFifo serializer advanced past its epoch");
    end
  end
`endif
endmodule
