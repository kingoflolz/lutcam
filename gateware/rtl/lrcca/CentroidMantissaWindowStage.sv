// Canonical readable SystemVerilog implementation of centroid compression.

import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::max_int;

module CentroidMantissaWindowStage #(
  parameter int VALUE_BITS = 1,
  parameter int FRAC_BITS = 21,
  parameter int EXP_BITS = 1,
  parameter int STAGE = 0,
  parameter int IN_BITS = VALUE_BITS + FRAC_BITS,
  parameter int OUT_BITS = FRAC_BITS
)(
  input logic [IN_BITS-1:0] in_value,
  input logic [EXP_BITS-1:0] exponent,
  output logic [OUT_BITS-1:0] out_value
);
  localparam int AMOUNT = 1 << STAGE;
  localparam int LOWER_SHIFT_MAX = AMOUNT - 1;
  localparam int VALUE_SHIFT_MAX = VALUE_BITS - 1;
  localparam int REMAINING_SHIFT_MAX =
      (LOWER_SHIFT_MAX < VALUE_SHIFT_MAX) ? LOWER_SHIFT_MAX : VALUE_SHIFT_MAX;
  localparam int NEEDED_BITS = FRAC_BITS + REMAINING_SHIFT_MAX;
  localparam int STAGE_BITS = (NEEDED_BITS < IN_BITS) ? NEEDED_BITS : IN_BITS;

  logic [STAGE_BITS-1:0] stage_value;
  // Recursion shares a generic input width; bits above this stage window are discarded.
  logic unused_input_window_bits;
  assign unused_input_window_bits = ^in_value;

  generate
    if (AMOUNT < VALUE_BITS) begin : active_shift
      logic [STAGE_BITS-1:0] shifted_value;

      assign shifted_value = STAGE_BITS'(in_value >> AMOUNT);
      assign stage_value =
          exponent[STAGE] ? shifted_value[STAGE_BITS-1:0] : in_value[STAGE_BITS-1:0];
    end else begin : inactive_shift
      assign stage_value = in_value[STAGE_BITS-1:0];
    end

    if (STAGE == 0) begin : leaf
      assign out_value = stage_value[FRAC_BITS-1:0];
    end else begin : next_stage
      CentroidMantissaWindowStage #(
        .VALUE_BITS(VALUE_BITS),
        .FRAC_BITS(FRAC_BITS),
        .EXP_BITS(EXP_BITS),
        .STAGE(STAGE - 1),
        .IN_BITS(STAGE_BITS),
        .OUT_BITS(OUT_BITS)
      ) lower (
        .in_value(stage_value),
        .exponent(exponent),
        .out_value(out_value)
      );
    end
  endgenerate
endmodule
