`default_nettype none

// Shared-wavelet TagDetect: the full-resolution camera stream and the LL2/LL4
// taps are thresholded concurrently, composed into the framed
// raster, and consumed by one hierarchical CCA instance.
module WaveletTagDetect #(
  // Main Titanium profile only; generic targets retain fabric arithmetic.
  parameter bit USE_DSP_HIGH_ZOM = 1'b0,
  parameter bit USE_DSP_HIGH_FOM = 1'b0,
  parameter int WIDTH = 1280,
  parameter int HEIGHT = 800,
  parameter int PIXEL_BITS = 8,
  parameter int COEFFICIENT_BITS = 14,
  parameter int COORD_WIDTH = 10,
  parameter int ROW_CYCLES = 896,
  parameter int STAGE2_Y_OFFSET = 27,
  parameter int STAGE4_Y_OFFSET = 127,
  parameter int INNER_CLOSE_DELAY = 0,
  parameter int MAX_CHILD_LIST_LEN = 9,
  parameter int CHILD_ARENA_CMD_QUEUE_DEPTH = 512,
  parameter int CHILD_ARENA_ENTRIES =
      ((WIDTH + (WIDTH >> 2) + (WIDTH >> 4) + 16) * MAX_CHILD_LIST_LEN) / 4,
  parameter int OUTER_CCA_TABLE_DEPTH = 0,
  parameter int INNER_CCA_TABLE_DEPTH = 0,
  parameter int CHILD_AREA_BITS = 0,
  parameter int CHILD_AREA_MAX = -1,
  parameter bit ASSERT_READ_STAGE_OVERFLOW = 1'b0,
  parameter string CENTROID_RECIPROCAL_SEGMENT_BIN =
      "rtl/lrcca/centroid_reciprocal_segments_q17.bin",
  parameter string CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN =
      "rtl/lrcca/centroid_reciprocal_midpoint_reflection_q17.bin",
  localparam int SOURCE_Y_BITS = lrcca_util_pkg::bit_width(longint'(HEIGHT)),
  localparam int CENTROID_SOURCE_MAX =
      ((WIDTH + (WIDTH >> 2) + (WIDTH >> 4) + 16) > HEIGHT) ?
      (WIDTH + (WIDTH >> 2) + (WIDTH >> 4) + 16) : HEIGHT,
  localparam int CENTROID_Q5_BITS =
      lrcca_util_pkg::bit_width(longint'(CENTROID_SOURCE_MAX) << 5),
  localparam int INNER_DATA_WIDTH = 1 + (2 * CENTROID_Q5_BITS),
  localparam int CENTROID_GROUP_X_Q4_BITS = lrcca_util_pkg::bit_width(
      (longint'(WIDTH) + (longint'(WIDTH) >> 2) +
      (longint'(WIDTH) >> 4) + 16) << 4
  ),
  localparam int CENTROID_GROUP_Y_Q4_BITS =
      lrcca_util_pkg::bit_width(longint'(HEIGHT) << 4)
) (
  input  wire clk,
  input  wire rst,
  input  wire in_vref,
  input  wire in_href,
  input  wire [PIXEL_BITS-1:0] in_d0,
  input  wire [PIXEL_BITS-1:0] in_d1,
  input  wire ll2_valid,
  input  wire ll2_first,
  input  wire ll2_last,
  input  wire [COORD_WIDTH-1:0] ll2_pair_x,
  input  wire [COORD_WIDTH-1:0] ll2_y,
  input  wire signed [1:0][COEFFICIENT_BITS-1:0] ll2_value,
  input  wire ll4_valid,
  input  wire ll4_first,
  input  wire ll4_last,
  input  wire [COORD_WIDTH-1:0] ll4_pair_x,
  input  wire [COORD_WIDTH-1:0] ll4_y,
  input  wire signed [1:0][COEFFICIENT_BITS-1:0] ll4_value,
  output logic inner_fv_valid,
  output logic [INNER_DATA_WIDTH-1:0] inner_fv_data,
  output logic hier_inner_end,
  output logic centroid_group_valid,
  output logic centroid_group_centroid_valid,
  output logic [CENTROID_GROUP_X_Q4_BITS-1:0] centroid_group_x_q4,
  output logic [CENTROID_GROUP_Y_Q4_BITS-1:0] centroid_group_y_q4,
  output logic [7:0] centroid_group_log_m_q5_3,
  output logic centroid_group_end,
  output logic threshold_busy,
  output logic threshold_protocol_error,
  output logic threshold_overrun,
  output wire hierarchy_busy
);
  import lrcca_util_pkg::bit_width;
  import lrcca_util_pkg::max_int;

  function automatic int covered_logical_rows(
    input int image_height,
    input int stage_height,
    input int vrep,
    input int y_offset
  );
    int replicated_rows;
    int rounded_rows;
    begin
      replicated_rows = image_height - y_offset;
      if (replicated_rows < 0) replicated_rows = 0;
      if (replicated_rows > image_height) replicated_rows = image_height;
      rounded_rows = (replicated_rows + vrep - 1) / vrep;
      covered_logical_rows = (rounded_rows < stage_height) ?
          rounded_rows : stage_height;
    end
  endfunction

  localparam int STAGE0_WIDTH = WIDTH;
  localparam int STAGE2_WIDTH = WIDTH >> 2;
  localparam int STAGE4_WIDTH = WIDTH >> 4;
  localparam int STAGE0_HEIGHT = HEIGHT;
  localparam int STAGE2_HEIGHT = HEIGHT >> 2;
  localparam int STAGE4_HEIGHT = HEIGHT >> 4;
  localparam int TAG_FRAME_X_DEPTH = 4;
  localparam int STAGE0_OUTPUT_WIDTH = STAGE0_WIDTH;
  localparam int STAGE2_OUTPUT_WIDTH = STAGE2_WIDTH + (2 * TAG_FRAME_X_DEPTH);
  localparam int STAGE4_OUTPUT_WIDTH = STAGE4_WIDTH + (2 * TAG_FRAME_X_DEPTH);
  localparam int HIER_WIDTH =
      STAGE0_OUTPUT_WIDTH + STAGE2_OUTPUT_WIDTH + STAGE4_OUTPUT_WIDTH;
  localparam int STAGE0_COVERED_ROWS = HEIGHT;
  localparam int STAGE2_COVERED_ROWS = covered_logical_rows(
      HEIGHT, STAGE2_HEIGHT, 4, STAGE2_Y_OFFSET);
  localparam int STAGE4_COVERED_ROWS = covered_logical_rows(
      HEIGHT, STAGE4_HEIGHT, 16, STAGE4_Y_OFFSET);
  localparam int STAGE0_TAIL_ROWS = STAGE0_HEIGHT - STAGE0_COVERED_ROWS;
  localparam int STAGE2_TAIL_ROWS = STAGE2_HEIGHT - STAGE2_COVERED_ROWS;
  localparam int STAGE4_TAIL_ROWS = STAGE4_HEIGHT - STAGE4_COVERED_ROWS;
  localparam int HIER_HEIGHT = HEIGHT + max_int(
      STAGE0_TAIL_ROWS, max_int(STAGE2_TAIL_ROWS, STAGE4_TAIL_ROWS));
  localparam int HIER_Y_BITS = bit_width(longint'(HIER_HEIGHT));
  localparam int HIER_ROW_BITS = bit_width(longint'(HIER_HEIGHT) + 1);
  localparam int unused_in_synthesis_hier_pair_count = HIER_WIDTH >> 1;
  // The composite frontend owns the 896-cycle row epoch. This gap only
  // covers the hierarchy's mandatory post-EOL drain interval.
  localparam int HIER_ROW_GAP_CYCLES = 10;
  localparam int HIER_ROW_GAP_BITS =
      bit_width(longint'(HIER_ROW_GAP_CYCLES) + 1);
  localparam int CHILD_ARENA_SORT_KEY_BITS = 8;
  localparam int CENTROID_X_Q4_BITS = bit_width(longint'(HIER_WIDTH) << 4);
  localparam int CENTROID_Y_Q4_BITS = bit_width(longint'(HEIGHT) << 4);
  localparam int HIER_INNER_DATA_WIDTH = CHILD_ARENA_SORT_KEY_BITS + 1 +
      CENTROID_X_Q4_BITS + CENTROID_Y_Q4_BITS;

  logic threshold_vref;
  logic threshold_href;
  logic [PIXEL_BITS-1:0] threshold_d0;
  logic [PIXEL_BITS-1:0] threshold_d1;
  logic threshold_t0;
  logic threshold_t1;
  logic threshold_eol;
  logic threshold_row_replicated;
  logic threshold_feature_valid_d0;
  logic threshold_feature_valid_d1;
  logic [SOURCE_Y_BITS-1:0] threshold_source_y;
  logic threshold_frontend_overrun;

  logic [HIER_Y_BITS-1:0] threshold_y;
  logic threshold_sof;
  logic threshold_last_frame_word;
  logic threshold_present;
  logic threshold_valid;
  logic [PIXEL_BITS-1:0] threshold_inverted_d0;
  logic [PIXEL_BITS-1:0] threshold_inverted_d1;
  wire unused_weight_bits = ^{threshold_inverted_d0[PIXEL_BITS-1:5], threshold_inverted_d1[PIXEL_BITS-1:5]};
  logic threshold_out_ready;
  logic [HIER_ROW_GAP_BITS-1:0] threshold_row_gap;
  wire hierarchy_overrun = 1'b0;

  logic hier_in_ready;
  logic [HIER_INNER_DATA_WIDTH-1:0] hier_inner_fv_data;
  logic [CHILD_ARENA_SORT_KEY_BITS-1:0] hier_child_log_m_q5_3;
  logic hier_centroid_valid;
  logic [CENTROID_X_Q4_BITS-1:0] hier_centroid_x_q4;
  logic [CENTROID_Y_Q4_BITS-1:0] hier_centroid_y_q4;
  logic [CENTROID_Q5_BITS-1:0] public_centroid_x_q5;
  logic [CENTROID_Q5_BITS-1:0] public_centroid_y_q5;

  assign threshold_last_frame_word =
      threshold_eol && threshold_y == HIER_Y_BITS'(HIER_HEIGHT - 1);
  assign threshold_present = threshold_vref && threshold_href;
  assign threshold_inverted_d0 = {PIXEL_BITS{1'b1}} - threshold_d0;
  assign threshold_inverted_d1 = {PIXEL_BITS{1'b1}} - threshold_d1;
  assign threshold_out_ready = hier_in_ready && threshold_row_gap == '0;
  assign threshold_valid = threshold_present && threshold_out_ready;
  assign threshold_overrun = threshold_frontend_overrun || hierarchy_overrun;

  assign hier_child_log_m_q5_3 =
      hier_inner_fv_data[CHILD_ARENA_SORT_KEY_BITS-1:0];
  assign hier_centroid_valid = inner_fv_valid &&
      hier_inner_fv_data[CHILD_ARENA_SORT_KEY_BITS];
  assign hier_centroid_x_q4 = hier_inner_fv_data[
      CHILD_ARENA_SORT_KEY_BITS + 1 +: CENTROID_X_Q4_BITS];
  assign hier_centroid_y_q4 = hier_inner_fv_data[
      CHILD_ARENA_SORT_KEY_BITS + 1 + CENTROID_X_Q4_BITS +: CENTROID_Y_Q4_BITS];
  assign public_centroid_x_q5 = CENTROID_Q5_BITS'({hier_centroid_x_q4, 1'b0});
  assign public_centroid_y_q5 = CENTROID_Q5_BITS'({hier_centroid_y_q4, 1'b0});
  assign inner_fv_data = {
      public_centroid_y_q5,
      public_centroid_x_q5,
      hier_centroid_valid
  };
  assign centroid_group_valid = inner_fv_valid;
  assign centroid_group_centroid_valid = hier_centroid_valid;
  assign centroid_group_x_q4 = hier_centroid_x_q4;
  assign centroid_group_y_q4 = hier_centroid_y_q4;
  assign centroid_group_log_m_q5_3 = hier_child_log_m_q5_3;
  assign centroid_group_end = hier_inner_end;

  AdaptiveThresholdFrontend #(
    .WIDTH(WIDTH),
    .HEIGHT(HEIGHT),
    .PIXEL_BITS(PIXEL_BITS),
    .COEFFICIENT_BITS(COEFFICIENT_BITS),
    .COORD_WIDTH(COORD_WIDTH),
    .STAGE2_Y_OFFSET(STAGE2_Y_OFFSET),
    .STAGE4_Y_OFFSET(STAGE4_Y_OFFSET),
    .ROW_CYCLES(ROW_CYCLES)
  ) threshold_frontend (
    .clk,
    .rst,
    .in_vref,
    .in_href,
    .in_d0,
    .in_d1,
    .ll2_valid,
    .ll2_first,
    .ll2_last,
    .ll2_pair_x,
    .ll2_y,
    .ll2_value,
    .ll4_valid,
    .ll4_first,
    .ll4_last,
    .ll4_pair_x,
    .ll4_y,
    .ll4_value,
    .out_ready(threshold_out_ready),
    .out_vref(threshold_vref),
    .out_href(threshold_href),
    .out_d0(threshold_d0),
    .out_d1(threshold_d1),
    .out_t0(threshold_t0),
    .out_t1(threshold_t1),
    .out_eol(threshold_eol),
    .out_row_replicated(threshold_row_replicated),
    .out_feature_valid_d0(threshold_feature_valid_d0),
    .out_feature_valid_d1(threshold_feature_valid_d1),
    .out_source_y(threshold_source_y),
    .busy(threshold_busy),
    .protocol_error(threshold_protocol_error),
    .overrun(threshold_frontend_overrun)
  );

  HierarchicalTwoPPC #(
    .USE_DSP_HIGH_ZOM(USE_DSP_HIGH_ZOM),
    .USE_DSP_HIGH_FOM(USE_DSP_HIGH_FOM),
    .RAW_PIXEL_BITS(5),
    .WIDTH(HIER_WIDTH),
    .HEIGHT(HIER_HEIGHT),
    .INNER_CLOSE_DELAY(INNER_CLOSE_DELAY),
    .MAX_CHILD_LIST_LEN(MAX_CHILD_LIST_LEN),
    .CHILD_ARENA_CMD_QUEUE_DEPTH(CHILD_ARENA_CMD_QUEUE_DEPTH),
    .CHILD_ARENA_ENTRIES(CHILD_ARENA_ENTRIES),
    .OUTER_CCA_TABLE_DEPTH(OUTER_CCA_TABLE_DEPTH),
    .INNER_CCA_TABLE_DEPTH(INNER_CCA_TABLE_DEPTH),
    .CHILD_AREA_BITS(CHILD_AREA_BITS),
    .CHILD_AREA_MAX(CHILD_AREA_MAX),
    .CENTROID_SOURCE_WIDTH(HIER_WIDTH),
    .CENTROID_SOURCE_HEIGHT(HEIGHT),
    .INNER_CENTROID_ALWAYS_VALID(1'b0),
    .FULL_LOG_MASS_CHILD_SORT(1'b1),
    .ASSERT_READ_STAGE_OVERFLOW(ASSERT_READ_STAGE_OVERFLOW),
    .CENTROID_REGISTER_INPUT(1'b1),
    .CENTROID_POWER2_MULTIPLIER_NORMALIZER(1'b1),
    .CENTROID_MIDPOINT_REFLECTION_RECIPROCAL(1'b1),
    .CENTROID_RECIPROCAL_SEGMENT_BIN(CENTROID_RECIPROCAL_SEGMENT_BIN),
    .CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN(
      CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN
    )
  ) hierarchy_core (
    .clk,
    .rst,
    .datavalid(threshold_valid),
    .in_ready(hier_in_ready),
    .d0(threshold_t0),
    .d1(threshold_t1),
    .raw_d0(threshold_inverted_d0[4:0]),
    .raw_d1(threshold_inverted_d1[4:0]),
    .feature_valid_d0(threshold_feature_valid_d0),
    .feature_valid_d1(threshold_feature_valid_d1),
    .feature_y(HIER_ROW_BITS'(threshold_source_y)),
    .sof(threshold_sof),
    .eol(threshold_eol),
    .inner_fv_valid,
    .inner_fv_data(hier_inner_fv_data),
    .inner_end(hier_inner_end),
    .pipeline_busy(hierarchy_busy)
  );

  always_ff @(posedge clk) begin
    if (rst) begin
      threshold_y <= '0;
      threshold_sof <= 1'b1;
      threshold_row_gap <= '0;
    end else begin
      if (threshold_valid && threshold_eol)
        threshold_row_gap <= HIER_ROW_GAP_BITS'(HIER_ROW_GAP_CYCLES);
      else if (threshold_row_gap != '0)
        threshold_row_gap <= threshold_row_gap - 1'b1;

      if (threshold_valid) begin
        if (threshold_last_frame_word) begin
          threshold_y <= '0;
          threshold_sof <= 1'b1;
        end else if (threshold_eol) begin
          threshold_y <= threshold_y + 1'b1;
          threshold_sof <= 1'b0;
        end else threshold_sof <= 1'b0;
      end

      if (ASSERT_READ_STAGE_OVERFLOW) begin
        assert (!threshold_protocol_error);
        assert (!threshold_overrun);
        assert (!(threshold_valid && threshold_row_replicated) |
            !(threshold_feature_valid_d0 || threshold_feature_valid_d1));
      end
    end
  end

`ifndef SYNTHESIS
  initial begin
    assert (HIER_ROW_GAP_CYCLES >= 10)
      else $error("hierarchy recovery gap must be at least ten clocks");
    assert (ROW_CYCLES >= unused_in_synthesis_hier_pair_count + HIER_ROW_GAP_CYCLES + 1);
  end
`endif
endmodule

`default_nettype wire
