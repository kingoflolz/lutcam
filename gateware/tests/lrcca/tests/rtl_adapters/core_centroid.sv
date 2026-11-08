// Fixed production configuration from the centroid-compression equivalence
// suite.  Centered doubled moments are signed two's-complement inputs.
module CoreCentroid (
  input  logic               clk,
  input  logic               rst,
  input  logic               in_valid,
  input  logic [27:0]        m,
  input  logic signed [37:0] sx,
  input  logic signed [36:0] sy,
  output logic               out_valid,
  output logic               centroid_valid,
  output logic [14:0]        x_q4,
  output logic [13:0]        y_q4,
  output logic [7:0]         m_log_q5_3
);
  CentroidCompressor #(
    .WIDTH(1274),
    .HEIGHT(794),
    .PIXEL_BITS(8),
    // Cargo runs an integration test from the package directory while the
    // production default is workspace-root-relative.
    .RECIPROCAL_SEGMENT_BIN("../../rtl/lrcca/centroid_reciprocal_segments_q17.bin")
  ) dut (
    .m,
    .sx,
    .sy,
    .clk,
    .rst,
    .out_valid,
    .centroid_valid,
    .x_q4,
    .y_q4,
    .m_log_q5_3,
    .in_valid
  );
endmodule
