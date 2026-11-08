// Shared, vendor-neutral centroid normalization logic.

import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::max_int;

module CentroidMidpointReflectionReciprocal #(
  parameter int FRACTION_BITS = 21,
  parameter int SEGMENT_BITS = 9,
  parameter int DISCARD_BITS = 3,
  parameter int COEFFICIENT_BITS = 18,
  parameter int RECIPROCAL_BITS = 17,
  parameter int PRODUCT_SHIFT = 19,
  localparam int RESIDUAL_BITS = FRACTION_BITS - SEGMENT_BITS,
  localparam int REFLECTED_BITS = FRACTION_BITS - DISCARD_BITS + 1,
  localparam int PRODUCT_BITS = COEFFICIENT_BITS + REFLECTED_BITS
)(
  input logic [FRACTION_BITS-1:0] normalized_fraction,
  input logic [COEFFICIENT_BITS-1:0] coefficient,
  output logic [RECIPROCAL_BITS-1:0] reciprocal
);
  logic [REFLECTED_BITS-1:0] reflected_mantissa;
  logic [PRODUCT_BITS-1:0] reflection_product;

  // Within a segment [L,L+W), {1,segment,~fraction} is exactly
  // 2*T-N for midpoint T=L+(W-1)/2.  Dropping three low bits makes
  // the reflected mantissa fit one unsigned 19-by-18 DSP multiply.
  assign reflected_mantissa = {
      1'b1,
      normalized_fraction[FRACTION_BITS-1 -: SEGMENT_BITS],
      ~normalized_fraction[RESIDUAL_BITS-1:DISCARD_BITS]
  };
  assign reflection_product = coefficient * reflected_mantissa;
  assign reciprocal = reflection_product[
      PRODUCT_SHIFT + RECIPROCAL_BITS - 1 -: RECIPROCAL_BITS
  ];
  // Low mantissa and out-of-window product bits are intentionally discarded.
  logic unused_fixed_point_bits;
  assign unused_fixed_point_bits = ^{normalized_fraction, reflection_product};

endmodule
