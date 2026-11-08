// Shared, vendor-neutral centroid normalization logic.

import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::max_int;

module CentroidSignedMomentNormalizer #(
  parameter int VALUE_BITS = 2,
  parameter int OUTPUT_BITS = 2,
  parameter int FRAC_BITS = 4,
  parameter int EXP_BITS = 1
)(
  input logic signed [VALUE_BITS-1:0] value,
  input logic [EXP_BITS-1:0] exponent,
  output logic signed [OUTPUT_BITS-1:0] normalized
);
  localparam int VALUE_EXT_BITS = VALUE_BITS + 1;
  localparam int ROUND_WINDOW_BITS = OUTPUT_BITS + 1;
  localparam int ROUND_SOURCE_BITS = VALUE_EXT_BITS + 1;

  logic signed [VALUE_EXT_BITS-1:0] value_ext;
  logic [EXP_BITS-1:0] right_shift_amount;
  logic [EXP_BITS-1:0] left_shift_amount;
  logic signed [ROUND_SOURCE_BITS-1:0] round_source;
  logic signed [ROUND_WINDOW_BITS-1:0] round_window;
  logic signed [OUTPUT_BITS-1:0] right_quotient;
  logic signed [OUTPUT_BITS-1:0] right_rounded;
  logic signed [OUTPUT_BITS-1:0] left_exact;

  assign value_ext = $signed({value[VALUE_BITS-1], value});
  assign right_shift_amount =
      (exponent > EXP_BITS'(FRAC_BITS)) ? exponent - EXP_BITS'(FRAC_BITS) : '0;
  assign left_shift_amount =
      (exponent <= EXP_BITS'(FRAC_BITS)) ? EXP_BITS'(FRAC_BITS) - exponent : '0;

  // For a positive right shift k, shifting {value_ext, 1'b0} by k places
  // value[k-1] in bit zero and the low OUTPUT_BITS quotient bits above it.
  assign round_source = {value_ext, 1'b0};

  CentroidSignedWindowStage #(
    .WINDOW_BITS(ROUND_WINDOW_BITS),
    .SHIFT_BITS(EXP_BITS),
    .STAGE(EXP_BITS - 1),
    .IN_BITS(ROUND_SOURCE_BITS)
  ) right_window (
    .in_value(round_source),
    .shift_amount(right_shift_amount),
    .out_value(round_window)
  );

  assign right_quotient = $signed(round_window[ROUND_WINDOW_BITS-1:1]);
  assign right_rounded =
      right_quotient + OUTPUT_BITS'({{(OUTPUT_BITS-1){1'b0}}, round_window[0]});
  assign left_exact = OUTPUT_BITS'(value_ext <<< left_shift_amount);
  assign normalized =
      (exponent > EXP_BITS'(FRAC_BITS)) ? right_rounded : left_exact;
endmodule
