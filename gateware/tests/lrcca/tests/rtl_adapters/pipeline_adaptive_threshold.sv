`default_nettype none

// Test-only closure: real wavelet producer and formatter, without CCA/LFTag.
module PipelineAdaptiveThreshold (
  input wire clk,
  input wire rst,
  input wire in_vref,
  input wire in_href,
  input wire [15:0] in_pair,
  input wire out_ready,
  output wire out_valid,
  output wire [31:0] out_record,
  output wire [7:0] status
);
  logic frame_start_pending;
  wire active_pair = in_vref && in_href;
  always_ff @(posedge clk) begin
    if (rst || !in_vref) frame_start_pending <= 1'b1;
    else if (active_pair) frame_start_pending <= 1'b0;
  end

  wire [11:0] unused_raw_valid;
  wire signed [11:0][13:0] unused_raw_value;
  wire ll2_valid, ll2_first, ll2_last;
  wire [9:0] ll2_pair_x, ll2_y;
  wire signed [1:0][13:0] ll2_value;
  wire ll4_valid, ll4_first, ll4_last;
  wire [9:0] ll4_pair_x, ll4_y;
  wire signed [1:0][13:0] ll4_value;
  wire unused_h4_valid, unused_h4_first, unused_h4_last;
  wire [9:0] unused_h4_pair_x, unused_h4_y;
  wire signed [1:0][13:0] unused_h4_value;
  wire frame_done, frontend_error;
  forward_wavelet_transform transform (
    .clk, .rst_n(!rst), .in_sof(frame_start_pending && active_pair),
    .in_valid(active_pair), .in_px0(in_pair[7:0]), .in_px1(in_pair[15:8]),
    .out_raw_valid(unused_raw_valid), .out_raw_value(unused_raw_value),
    .out_ll2_valid(ll2_valid), .out_ll2_first(ll2_first),
    .out_ll2_last(ll2_last), .out_ll2_pair_x(ll2_pair_x),
    .out_ll2_y(ll2_y), .out_ll2_value(ll2_value),
    .out_h4_low_valid(unused_h4_valid), .out_h4_low_first(unused_h4_first),
    .out_h4_low_last(unused_h4_last), .out_h4_low_pair_x(unused_h4_pair_x),
    .out_h4_low_y(unused_h4_y), .out_h4_low_value(unused_h4_value),
    .out_ll4_valid(ll4_valid), .out_ll4_first(ll4_first),
    .out_ll4_last(ll4_last), .out_ll4_pair_x(ll4_pair_x),
    .out_ll4_y(ll4_y), .out_ll4_value(ll4_value),
    .frame_done, .protocol_error(frontend_error)
  );

  wire out_vref, out_href, out_t0, out_t1, out_eol;
  wire out_row_replicated, out_feature_valid_d0, out_feature_valid_d1;
  wire [7:0] out_d0, out_d1;
  wire [9:0] out_source_y;
  wire busy, threshold_error, overrun;
  AdaptiveThresholdFrontend formatter (
    .clk, .rst, .in_vref, .in_href,
    .in_d0(in_pair[7:0]), .in_d1(in_pair[15:8]),
    .ll2_valid, .ll2_first, .ll2_last, .ll2_pair_x, .ll2_y, .ll2_value,
    .ll4_valid, .ll4_first, .ll4_last, .ll4_pair_x, .ll4_y, .ll4_value,
    .out_ready, .out_vref, .out_href, .out_d0, .out_d1, .out_t0, .out_t1,
    .out_eol, .out_row_replicated, .out_feature_valid_d0,
    .out_feature_valid_d1, .out_source_y, .busy,
    .protocol_error(threshold_error), .overrun
  );
  assign out_valid = out_vref && out_href;
  assign out_record = {out_source_y, out_feature_valid_d1,
      out_feature_valid_d0, out_row_replicated, out_eol,
      out_t1, out_t0, out_d1, out_d0};
  assign status = {1'b0, frame_done, busy, out_eol && !out_valid,
      out_vref != out_href, overrun, threshold_error, frontend_error};
endmodule

`default_nettype wire
