// Wiring-only adapter for the production midpoint-reflection reciprocal
// datapath.  The table remains owned and checked by the Rust test so that each
// cell can be selected without adding test-only behavior to the RTL wrapper.
module CentroidMidpointReflection (
  input  logic [20:0] normalized_fraction,
  input  logic [17:0] coefficient,
  output logic [16:0] reciprocal
);
  CentroidMidpointReflectionReciprocal #(
    .FRACTION_BITS(21),
    .SEGMENT_BITS(9),
    .DISCARD_BITS(3),
    .COEFFICIENT_BITS(18),
    .RECIPROCAL_BITS(17),
    .PRODUCT_SHIFT(19)
  ) dut (
    .normalized_fraction,
    .coefficient,
    .reciprocal
  );
endmodule
