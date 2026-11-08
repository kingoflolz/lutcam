// Scalar-port wrapper freezing the configuration used by the native
// ChildArena regression.  The production command/output protocols are passed
// through unchanged.
module CoreChildArena (
  input  logic        clk,
  input  logic        rst,
  input  logic        new_valid,
  output logic        new_ready,
  output logic        new_rsp_valid,
  input  logic        new_rsp_ready,
  output logic [6:0]  new_rsp_id,
  output logic        new_rsp_status,
  input  logic        cmd_valid,
  output logic        cmd_ready,
  input  logic [1:0]  cmd_op,
  input  logic [6:0]  cmd_id_a,
  input  logic [6:0]  cmd_id_b,
  input  logic [33:0] cmd_fv,
  output logic        out_valid,
  input  logic        out_ready,
  output logic [33:0] out_fv,
  output logic        out_first,
  output logic        out_last,
  output logic        idle,
  output logic [6:0]  free_count
);
  ChildArena #(
    .ENTRY_COUNT(108),
    .MAX_LIST_LEN(9),
    .FEATURE_WIDTH(34),
    .SORT_KEY_WIDTH(11),
    .CMD_QUEUE_DEPTH(512)
  ) dut (
    .clk,
    .rst,
    .new_valid,
    .new_ready,
    .new_alloc_id(),
    .new_rsp_valid,
    .new_rsp_ready,
    .new_rsp_id,
    .new_rsp_status,
    .cmd_valid,
    .cmd_ready,
    .cmd_op,
    .cmd_id_a,
    .cmd_id_b,
    .cmd_fv,
    .bundle_child_valid(1'b0), .bundle_merge_valid(1'b0),
    .bundle_merge_id_a('0), .bundle_merge_id_b('0),
    .bundle_read_valid(1'b0), .bundle_read_id('0),
    .cmd_admit_ready(), .cmd_queue_occupancy(),
    .out_valid,
    .out_ready,
    .out_fv,
    .out_first,
    .out_last,
    .idle,
    .free_count
  );
endmodule
