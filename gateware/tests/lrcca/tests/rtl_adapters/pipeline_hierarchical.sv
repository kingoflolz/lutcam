// Fixed-size test adapter for the production preframed hierarchy.
module PipelineHierarchical (
  input  logic clk,
  input  logic rst,
  input  logic datavalid,
  output logic in_ready,
  input  logic d0,
  input  logic d1,
  input  logic [7:0] raw_d0,
  input  logic [7:0] raw_d1,
  input  logic feature_valid_d0,
  input  logic feature_valid_d1,
  input  logic [5:0] feature_y,
  input  logic sof,
  input  logic eol,
  output logic inner_fv_valid,
  output logic [28:0] inner_fv_data,
  output logic inner_end
);
  wire unused_pipeline_busy;
  HierarchicalTwoPPC #(
    .WIDTH(48),
    .HEIGHT(36),
    .CENTROID_SOURCE_WIDTH(48),
    .CENTROID_SOURCE_HEIGHT(36),
    .FULL_LOG_MASS_CHILD_SORT(1'b1),
    .OUTER_CCA_TABLE_DEPTH(30),
    .INNER_CCA_TABLE_DEPTH(30),
    .CENTROID_RECIPROCAL_SEGMENT_BIN(
      "../../rtl/lrcca/centroid_reciprocal_segments_q17.bin"
    ),
    .CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN(
      "../../rtl/lrcca/centroid_reciprocal_midpoint_reflection_q17.bin"
    )
  ) dut (
    .clk,
    .rst,
    .datavalid,
    .in_ready,
    .d0,
    .d1,
    .raw_d0,
    .raw_d1,
    .feature_valid_d0,
    .feature_valid_d1,
    .feature_y,
    .sof,
    .eol,
    .inner_fv_valid,
    .inner_fv_data,
    .inner_end,
    .pipeline_busy(unused_pipeline_busy)
  );
endmodule
