// Shared, vendor-neutral centroid normalization logic.

import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::max_int;

module CentroidMantissaWindow #(
  parameter int VALUE_BITS = 1,
  parameter int FRAC_BITS = 21,
  parameter int EXP_BITS = 1
)(
  input logic [VALUE_BITS-1:0] value,
  input logic [EXP_BITS-1:0] exponent,
  output logic [FRAC_BITS-1:0] window
);
  localparam int SOURCE_BITS = VALUE_BITS + FRAC_BITS;

  logic [SOURCE_BITS-1:0] source;

  assign source = {value, {FRAC_BITS{1'b0}}};

  CentroidMantissaWindowStage #(
    .VALUE_BITS(VALUE_BITS),
    .FRAC_BITS(FRAC_BITS),
    .EXP_BITS(EXP_BITS),
    .STAGE(EXP_BITS - 1),
    .IN_BITS(SOURCE_BITS),
    .OUT_BITS(FRAC_BITS)
  ) normalize (
    .in_value(source),
    .exponent(exponent),
    .out_value(window)
  );
endmodule
