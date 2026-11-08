// Shared, vendor-neutral centroid normalization logic.

import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::max_int;

module CentroidSignedWindowStage #(
  parameter int WINDOW_BITS = 2,
  parameter int SHIFT_BITS = 1,
  parameter int STAGE = 0,
  parameter int IN_BITS = WINDOW_BITS
)(
  input logic signed [IN_BITS-1:0] in_value,
  input logic [SHIFT_BITS-1:0] shift_amount,
  output logic signed [WINDOW_BITS-1:0] out_value
);
  localparam int AMOUNT = 1 << STAGE;
  localparam int LOWER_SHIFT_MAX = AMOUNT - 1;
  localparam int NEEDED_BITS = WINDOW_BITS + LOWER_SHIFT_MAX;
  localparam int STAGE_BITS = (NEEDED_BITS < IN_BITS) ? NEEDED_BITS : IN_BITS;

  logic signed [STAGE_BITS-1:0] stage_value;

  generate
    if (AMOUNT < IN_BITS) begin : active_shift
      logic signed [STAGE_BITS-1:0] shifted_value;

      assign shifted_value = STAGE_BITS'(in_value >>> AMOUNT);
      assign stage_value = shift_amount[STAGE] ?
          shifted_value[STAGE_BITS-1:0] : in_value[STAGE_BITS-1:0];
    end else begin : sign_fill_shift
      assign stage_value = shift_amount[STAGE] ?
          {STAGE_BITS{in_value[IN_BITS-1]}} : in_value[STAGE_BITS-1:0];
    end

    if (STAGE == 0) begin : leaf
      assign out_value = stage_value[WINDOW_BITS-1:0];
    end else begin : next_stage
      CentroidSignedWindowStage #(
        .WINDOW_BITS(WINDOW_BITS),
        .SHIFT_BITS(SHIFT_BITS),
        .STAGE(STAGE - 1),
        .IN_BITS(STAGE_BITS)
      ) lower (
        .in_value(stage_value),
        .shift_amount(shift_amount),
        .out_value(out_value)
      );
    end
  endgenerate
endmodule
