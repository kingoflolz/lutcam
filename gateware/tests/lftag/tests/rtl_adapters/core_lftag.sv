`default_nettype none

// Standalone Marlin adapter for the complete production LFTag decoder and
// localizer. Keep this wrapper transparent: tests must observe the same
// configuration, ingress, result packet, and diagnostics as integration RTL.
module CoreLfTag (
  input  logic        clk,
  input  logic        rst,
  input  logic        clear_diagnostics,

  input  logic        in_valid,
  input  logic        in_group_end,
  input  logic [14:0] in_x_q4,
  input  logic [13:0] in_y_q4,
  input  logic [7:0]  in_log_mass_q5_3,
  input  logic        in_centroid_valid,
  input  logic [1:0]  in_scale,
  input  logic [7:0]  in_frame_seq,

  input  logic [31:0] cfg_inv_fx_q2_30,
  input  logic [31:0] cfg_inv_fy_q2_30,
  input  logic [31:0] cfg_cx_q16_16,
  input  logic [31:0] cfg_cy_q16_16,
  input  logic [31:0] cfg_printed_width_q16_16,
  input  logic [15:0] cfg_mean_residual_limit_q16,
  input  logic [7:0]  cfg_max_mass_spread,
  input  logic [15:0] cfg_min_baseline_q4,

  output logic        out_valid,
  input  logic        out_ready,
  output logic [31:0] out_data,
  output logic        out_last,

  output logic        busy,
  output logic [5:0]  fifo_level,
  output logic        candidate_start,
  output logic        candidate_done,
  output logic [9:0]  candidate_cycles,
  output logic        fifo_drop_sticky,
  output logic        protocol_error_sticky
);
  LfTagCompact3x3DecoderLocalizer dut (
      .config_read_valid(), .config_read_address(), .config_read_ready(1'b0),
      .config_response_valid(1'b0), .config_read_data(32'b0),
    .centroid_valid(), .centroid_ready(1'b1), .centroid_data(), .centroid_last(),
    .core_idle(),
    .core_dispatch_ready(),
    .external_fifo_head_valid(1'b0),
    .external_fifo_head_scale(2'b0),
    .external_fifo_head_frame_seq(8'b0),
    .external_fifo_head_protocol(1'b0),
    .external_fifo_head_mass_spread(8'b0),
    .external_fifo_head_pop(),
    .external_fifo_desc_read_valid(),
    .external_fifo_desc_read_index(),
    .external_fifo_desc_read_data_valid(1'b0),
    .external_fifo_desc_read_data(40'b0),
    .external_fifo_desc_read_centroid_valid(1'b0),
    .external_fifo_level(6'b0),
    .external_fifo_drop_sticky(1'b0),
    .external_protocol_error_sticky(1'b0),
    .*
  );
endmodule

`default_nettype wire
