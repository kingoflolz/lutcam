// Canonical readable SystemVerilog implementation of the hierarchical 2PPC core.

import lrcca_util_pkg::area_width;
import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::moment_zom_width;

module HierarchicalTwoPPC #(
  // Main Titanium profile only; generic targets retain fabric arithmetic.
  parameter bit USE_DSP_HIGH_ZOM = 1'b0,
  parameter bit USE_DSP_HIGH_FOM = 1'b0,
  parameter int WIDTH = 48,
  parameter int HEIGHT = 36,
  // Inner pixels now follow the outer hole-fill acceptance schedule. Their
  // S5 close epochs coincide, so an additional clock delay is not valid.
  parameter int INNER_CLOSE_DELAY = 0,
  parameter int MAX_CHILD_LIST_LEN = 9,
  parameter int CHILD_ARENA_CMD_QUEUE_DEPTH = 512,
  parameter int CHILD_ARENA_ENTRIES = (WIDTH * MAX_CHILD_LIST_LEN) / 4,
  parameter int OUTER_CCA_TABLE_DEPTH = 0,
  parameter int INNER_CCA_TABLE_DEPTH = 0,
  parameter int CHILD_AREA_BITS = 0,
  parameter int CHILD_AREA_MAX = -1,
  parameter int CENTROID_SOURCE_WIDTH = WIDTH,
  parameter int CENTROID_SOURCE_HEIGHT = HEIGHT,
  parameter bit INNER_CENTROID_ALWAYS_VALID = 1'b0,
  parameter bit FULL_LOG_MASS_CHILD_SORT = 1'b0,
  parameter bit ASSERT_READ_STAGE_OVERFLOW = 1'b0,
  parameter bit CENTROID_REGISTER_INPUT = 1'b0,
  parameter bit CENTROID_POWER2_MULTIPLIER_NORMALIZER = 1'b0,
  parameter bit CENTROID_MIDPOINT_REFLECTION_RECIPROCAL = 1'b0,
  parameter string CENTROID_RECIPROCAL_SEGMENT_BIN =
      "rtl/lrcca/centroid_reciprocal_segments_q17.bin",
  parameter string CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN =
      "rtl/lrcca/centroid_reciprocal_midpoint_reflection_q17.bin",
  parameter int RAW_PIXEL_BITS = 8,
  localparam int CENTROID_FRAC_BITS = 4,
  localparam int AREA_BITS = area_width(WIDTH, HEIGHT),
  localparam int CHILD_AREA_BITS_RES =
      (CHILD_AREA_BITS == 0) ? AREA_BITS : CHILD_AREA_BITS,
  localparam int CHILD_KEY_AREA_BITS = 3,
  localparam int CHILD_KEY_LOGM_BITS = 8,
  localparam int CHILD_ARENA_SORT_KEY_BITS = FULL_LOG_MASS_CHILD_SORT ?
      CHILD_KEY_LOGM_BITS : CHILD_KEY_AREA_BITS,
  localparam int MOMENT_ZOM_BITS = moment_zom_width(WIDTH, HEIGHT, RAW_PIXEL_BITS),
  localparam int MOMENT_X_BITS = centered_moment_bits(WIDTH, HEIGHT, RAW_PIXEL_BITS),
  localparam int MOMENT_Y_BITS = centered_moment_bits(HEIGHT, WIDTH, RAW_PIXEL_BITS),
  localparam int MOMENT_X_QUOTIENT_BITS = MOMENT_X_BITS - 1,
  localparam int MOMENT_Y_QUOTIENT_BITS = MOMENT_Y_BITS - 1,
  localparam int MOMENT_FEATURE_WIDTH =
      MOMENT_ZOM_BITS + MOMENT_X_QUOTIENT_BITS + MOMENT_Y_QUOTIENT_BITS,
  localparam logic MOMENT_X_CENTER_ODD = logic'((WIDTH - 1) & 1),
  localparam logic MOMENT_Y_CENTER_ODD = logic'((CENTROID_SOURCE_HEIGHT - 1) & 1),
  localparam int CENTROID_X_Q4_BITS =
      bit_width(longint'(CENTROID_SOURCE_WIDTH) << CENTROID_FRAC_BITS),
  localparam int CENTROID_Y_Q4_BITS =
      bit_width(longint'(CENTROID_SOURCE_HEIGHT) << CENTROID_FRAC_BITS),
  localparam int CHILD_ID_BITS = bit_width(longint'(CHILD_ARENA_ENTRIES) + 1),
  localparam int INNER_AREA_BITS =
      (CHILD_AREA_BITS_RES < CHILD_KEY_AREA_BITS) ?
      CHILD_AREA_BITS_RES : CHILD_KEY_AREA_BITS,
  // Full-log ordering and the centroid compressor consume only the weighted
  // mass/moments.  A close-valid pulse already proves that the component has
  // nonzero binary area, so carrying the saturated area through every inner
  // token is redundant in that specialization.
  localparam int INNER_AREA_FIELD_BITS =
      FULL_LOG_MASS_CHILD_SORT ? 0 : INNER_AREA_BITS,
  localparam int INNER_MOMENT_DATA_WIDTH =
      INNER_AREA_FIELD_BITS + MOMENT_FEATURE_WIDTH,
  localparam int INNER_CENTROID_VALID_BITS = INNER_CENTROID_ALWAYS_VALID ? 0 : 1,
  localparam int INNER_DATA_WIDTH =
      CHILD_ARENA_SORT_KEY_BITS + INNER_CENTROID_VALID_BITS +
      CENTROID_X_Q4_BITS + CENTROID_Y_Q4_BITS,
  localparam int ROW_BITS = bit_width(longint'(HEIGHT) + 1),
  localparam int PAIR_COUNT = WIDTH / 2,
  localparam int PAIR_BITS = bit_width(longint'(PAIR_COUNT)),
  localparam int OUTER_TABLE_DEPTH_RES =
      (OUTER_CCA_TABLE_DEPTH == 0) ? (1 << bit_width((longint'(WIDTH) + 1) / 2)) : OUTER_CCA_TABLE_DEPTH,
  localparam int INNER_TABLE_DEPTH_RES =
      (INNER_CCA_TABLE_DEPTH == 0) ? (1 << bit_width((longint'(WIDTH) + 1) / 2)) : INNER_CCA_TABLE_DEPTH,
  localparam int OUTER_DATA_WIDTH = 1 + CHILD_ID_BITS,
  localparam int CENTROID_M_BITS =
      moment_zom_width(CENTROID_SOURCE_WIDTH, CENTROID_SOURCE_HEIGHT, RAW_PIXEL_BITS),
  localparam int CENTROID_SX_BITS =
      centered_moment_bits(CENTROID_SOURCE_WIDTH, CENTROID_SOURCE_HEIGHT, RAW_PIXEL_BITS),
  localparam int CENTROID_SY_BITS =
      centered_moment_bits(CENTROID_SOURCE_HEIGHT, CENTROID_SOURCE_WIDTH, RAW_PIXEL_BITS),
  localparam int CENTROID_LATENCY = 4 + int'(CENTROID_REGISTER_INPUT),
  localparam int CHILD_ARENA_COUNT_BITS = bit_width(longint'(CHILD_ARENA_ENTRIES) + 1),
  // At most one bundle is produced by each accepted pair. Sixteen clocks
  // conservatively cover CCA/hole-fill drain, plus the centroid registers
  // and two skid/pending slots after admission stops.
  localparam int BUNDLE_ADMISSION_RESERVE = 16 + CENTROID_LATENCY + 2,
  localparam int COMMAND_COUNT_BITS = $clog2(CHILD_ARENA_CMD_QUEUE_DEPTH + 1)
)(
  input  logic clk,
  input  logic rst,

  input  logic datavalid,
  output logic in_ready,
  input  logic d0,
  input  logic d1,
  input  logic [RAW_PIXEL_BITS-1:0] raw_d0,
  input  logic [RAW_PIXEL_BITS-1:0] raw_d1,
  input  logic feature_valid_d0,
  input  logic feature_valid_d1,
  input  logic [ROW_BITS-1:0] feature_y,
  input  logic sof,
  input  logic eol,

  output logic inner_fv_valid,
  output logic [INNER_DATA_WIDTH-1:0] inner_fv_data,
  output logic inner_end,
  output logic pipeline_busy
);
  localparam longint unsigned CHILD_AREA_REPRESENTABLE_MAX_U =
      (64'd1 << CHILD_AREA_BITS_RES) - 1;
  localparam int CHILD_AREA_MAX_RES =
      (CHILD_AREA_MAX < 0) ? int'(CHILD_AREA_REPRESENTABLE_MAX_U) : CHILD_AREA_MAX;
  localparam logic [1:0] CHILD_ARENA_OP_ADD = 2'd0;
  localparam logic [1:0] CHILD_ARENA_OP_INIT = 2'd3;
  localparam logic [CHILD_KEY_AREA_BITS-1:0] CHILD_KEY_AREA_MAX =
      {CHILD_KEY_AREA_BITS{1'b1}};
  localparam int CHILD_KEY_AREA_MAX_RES =
      (CHILD_AREA_MAX_RES < int'(CHILD_KEY_AREA_MAX)) ?
      CHILD_AREA_MAX_RES : int'(CHILD_KEY_AREA_MAX);

  localparam int OFF_INNER_ZOM = INNER_AREA_FIELD_BITS;
  localparam int OFF_INNER_X_FOM = OFF_INNER_ZOM + MOMENT_ZOM_BITS;
  localparam int OFF_INNER_Y_FOM = OFF_INNER_X_FOM + MOMENT_X_QUOTIENT_BITS;

  logic [ROW_BITS-1:0] row;
  logic [PAIR_BITS-1:0] col;

  logic accept;
  logic core_ready;
  logic input_fire;
  logic input_buffer_valid;
  logic input_buffer_d0;
  logic input_buffer_d1;
  logic [RAW_PIXEL_BITS-1:0] input_buffer_raw_d0;
  logic [RAW_PIXEL_BITS-1:0] input_buffer_raw_d1;
  logic input_buffer_feature_valid_d0;
  logic input_buffer_feature_valid_d1;
  logic [ROW_BITS-1:0] input_buffer_feature_y;
  logic input_buffer_sof;
  logic input_buffer_eol;
  logic stream_valid;
  logic stream_d0;
  logic stream_d1;
  logic [RAW_PIXEL_BITS-1:0] stream_raw_d0;
  logic [RAW_PIXEL_BITS-1:0] stream_raw_d1;
  logic stream_feature_valid_d0;
  logic stream_feature_valid_d1;
  logic [ROW_BITS-1:0] stream_feature_y;
  logic stream_sof;
  logic stream_eol;
  logic input_buffer_next_valid;

  logic fg_topology0;
  logic fg_topology1;
  logic fg_feature0;
  logic fg_feature1;
  logic bg_feature0;
  logic bg_feature1;
  logic bg0;
  logic bg1;

  logic inner_pending_valid;
  logic inner_pending_bg0, inner_pending_bg1;
  logic inner_pending_feature0, inner_pending_feature1;
  logic [RAW_PIXEL_BITS-1:0] inner_pending_raw0, inner_pending_raw1;
  logic [ROW_BITS-1:0] inner_pending_y;
  logic inner_pending_sof, inner_pending_eol;
  logic inner_emit;

  logic outer_in_ready;
  logic inner_in_ready;

  logic outer_child_parent0_valid;
  logic outer_child_parent1_valid;
  logic [CHILD_ID_BITS-1:0] outer_child_parent0_id;
  logic [CHILD_ID_BITS-1:0] outer_child_parent1_id;
  logic outer_child_parent0_has_child;
  logic outer_child_parent1_has_child;
  logic outer_child_merge_valid;
  logic [CHILD_ID_BITS-1:0] outer_child_merge_id_a;
  logic [CHILD_ID_BITS-1:0] outer_child_merge_id_b;
  logic [OUTER_DATA_WIDTH-1:0] outer_s5_fv_data;
  logic outer_s5_fv_valid;
  logic unused_outer_s5_fv_lane;
  logic unused_outer_close_moment_valid;
  logic unused_outer_close_moment_lane;
  logic [0:0] unused_outer_close_moment_data;
  logic outer_fv_valid;
  logic [OUTER_DATA_WIDTH-2:0] unused_outer_fv_payload;
  logic [CHILD_ID_BITS-1:0] unused_outer_fv_child_id;
  logic unused_outer_fv_child_valid;

  logic unused_inner_child_parent0_valid;
  logic unused_inner_child_parent1_valid;
  logic unused_inner_child_merge_valid;
  logic [0:0] unused_inner_child_parent0_id;
  logic [0:0] unused_inner_child_parent1_id;
  logic unused_inner_child_parent0_has_child;
  logic unused_inner_child_parent1_has_child;
  logic [0:0] unused_inner_child_merge_id_a;
  logic [0:0] unused_inner_child_merge_id_b;
  logic unused_inner_s5_fv_valid;
  logic unused_inner_s5_fv_lane;
  logic [INNER_MOMENT_DATA_WIDTH-1:0] unused_inner_s5_fv_data;
  logic inner_close_moment_valid;
  logic inner_close_moment_lane;
  logic [INNER_MOMENT_DATA_WIDTH-1:0] inner_close_moment_data;
  logic unused_inner_final_fv_valid;
  logic [INNER_MOMENT_DATA_WIDTH-1:0] unused_inner_final_fv_data;
  logic [0:0] unused_inner_fv_child_id;
  logic unused_inner_fv_child_valid;

  logic child_valid;
  logic child_lane;
  logic [INNER_MOMENT_DATA_WIDTH-1:0] child_moment;
  logic [INNER_AREA_BITS-1:0] child_moment_area;
  logic [MOMENT_ZOM_BITS-1:0] child_moment_zom;
  logic [MOMENT_X_QUOTIENT_BITS-1:0] child_moment_x_fom_q;
  logic [MOMENT_Y_QUOTIENT_BITS-1:0] child_moment_y_fom_q;
  logic [MOMENT_X_BITS-1:0] child_moment_x_fom;
  logic [MOMENT_Y_BITS-1:0] child_moment_y_fom;

  logic parent_valid;
  logic [CHILD_ID_BITS-1:0] parent_id;
  logic parent_has_child;
  logic child_event;
  logic raw_new_child;

  logic centroid_in_valid;
  logic [CENTROID_M_BITS-1:0] centroid_m;
  logic [CENTROID_SX_BITS-1:0] centroid_sx;
  logic [CENTROID_SY_BITS-1:0] centroid_sy;
  logic centroid_out_valid;
  logic centroid_centroid_valid;
  logic [CENTROID_X_Q4_BITS-1:0] centroid_x_q4;
  logic [CENTROID_Y_Q4_BITS-1:0] centroid_y_q4;
  logic [CHILD_KEY_LOGM_BITS-1:0] centroid_log_m_q5_3;

  logic [CENTROID_LATENCY-1:0] centroid_valid_pipe;
  logic [CHILD_ID_BITS-1:0] centroid_id_pipe [0:CENTROID_LATENCY-1];
  logic [CHILD_KEY_AREA_BITS-1:0] centroid_area_key_pipe [0:CENTROID_LATENCY-1];
  logic [CENTROID_LATENCY-1:0] centroid_init_pipe;
  logic [CENTROID_LATENCY-1:0] centroid_merge_pipe;
  logic [CHILD_ID_BITS-1:0] centroid_merge_id_a_pipe [0:CENTROID_LATENCY-1];
  logic [CHILD_ID_BITS-1:0] centroid_merge_id_b_pipe [0:CENTROID_LATENCY-1];

  logic compressed_child_valid;
  logic [CHILD_ID_BITS-1:0] compressed_child_id;
  logic [CHILD_KEY_AREA_BITS-1:0] compressed_child_area_key;
  logic [CHILD_ARENA_SORT_KEY_BITS-1:0] compressed_child_key;
  logic [INNER_DATA_WIDTH-1:0] compressed_child_fv;
  logic [CENTROID_LATENCY-1:0] centroid_read_pipe;
  logic [CHILD_ID_BITS-1:0] centroid_read_id_pipe [0:CENTROID_LATENCY-1];
  logic outer_read_event;
  logic bundle_child_valid;
  logic bundle_merge_valid;
  logic bundle_read_valid;
  logic child_arena_admit_ready;
  logic [COMMAND_COUNT_BITS-1:0] child_arena_queue_occupancy;
  logic child_arena_new_valid;
  logic child_arena_new_ready;
  logic [CHILD_ID_BITS-1:0] child_arena_new_alloc_id;
  logic unused_child_arena_new_rsp_valid;
  logic unused_child_arena_new_rsp_status;
  logic [CHILD_ID_BITS-1:0] unused_child_arena_new_rsp_id;
  logic child_arena_cmd_valid;
  logic child_arena_cmd_ready;
  logic [1:0] child_arena_cmd_op;
  logic [CHILD_ID_BITS-1:0] child_arena_cmd_id_a;
  logic [CHILD_ID_BITS-1:0] child_arena_cmd_id_b;
  logic [INNER_DATA_WIDTH-1:0] child_arena_cmd_fv;
  logic child_arena_out_valid;
  logic [INNER_DATA_WIDTH-1:0] child_arena_out_fv;
  logic unused_child_arena_out_first;
  logic child_arena_out_last;
  logic unused_child_arena_idle;
  logic [CHILD_ARENA_COUNT_BITS-1:0] unused_child_arena_free_count;

  // TwoPPC's PIPE_DRAIN_CYCLES is ten; hole filling adds its pending-pair
  // flush. Sixteen clocks cover those fixed stages. Include every variable
  // queue and downstream token separately rather than guessing a quiet gap.
  logic [15:0] raster_drain;
  always_ff @(posedge clk) begin
    if (rst) raster_drain <= '0;
    else raster_drain <= {raster_drain[14:0], accept};
  end
  assign pipeline_busy = datavalid || input_buffer_valid || (|raster_drain)
      || inner_pending_valid || (|centroid_valid_pipe)
      || (|centroid_merge_pipe) || centroid_out_valid
      || outer_fv_valid || inner_close_moment_valid
      || (|centroid_read_pipe) || child_arena_cmd_valid
      || !unused_child_arena_idle || child_arena_out_valid || inner_fv_valid;

  logic unused_outer_feature_source_present;

  assign core_ready = outer_in_ready & inner_in_ready & child_arena_new_ready
      & child_arena_admit_ready;
  assign input_fire = datavalid & in_ready;
  assign stream_valid = input_buffer_valid | input_fire;
  assign stream_d0 = input_buffer_valid ? input_buffer_d0 : d0;
  assign stream_d1 = input_buffer_valid ? input_buffer_d1 : d1;
  assign stream_raw_d0 = input_buffer_valid ? input_buffer_raw_d0 : raw_d0;
  assign stream_raw_d1 = input_buffer_valid ? input_buffer_raw_d1 : raw_d1;
  assign stream_feature_valid_d0 =
      input_buffer_valid ? input_buffer_feature_valid_d0 : feature_valid_d0;
  assign stream_feature_valid_d1 =
      input_buffer_valid ? input_buffer_feature_valid_d1 : feature_valid_d1;
  assign stream_feature_y =
      input_buffer_valid ? input_buffer_feature_y : feature_y;
  assign stream_sof = input_buffer_valid ? input_buffer_sof : sof;
  assign stream_eol = input_buffer_valid ? input_buffer_eol : eol;
  assign accept = core_ready & stream_valid;
  assign input_buffer_next_valid = (input_buffer_valid | input_fire) & ~accept;

  function automatic logic [CHILD_KEY_AREA_BITS-1:0] child_area_key(
    input logic [CHILD_AREA_BITS_RES-1:0] area
  );
    begin
      child_area_key =
          (area > CHILD_AREA_BITS_RES'(CHILD_KEY_AREA_MAX_RES)) ?
          CHILD_KEY_AREA_BITS'(CHILD_KEY_AREA_MAX_RES) :
          CHILD_KEY_AREA_BITS'(area);
    end
  endfunction

  function automatic logic [CHILD_ARENA_SORT_KEY_BITS-1:0] child_sort_key(
    input logic [CHILD_KEY_AREA_BITS-1:0] area_key,
    input logic [CHILD_KEY_LOGM_BITS-1:0] log_m_q5_3
  );
    logic [CHILD_KEY_AREA_BITS-1:0] dense_area;
    begin
      dense_area = area_key - CHILD_KEY_AREA_BITS'(1);
      if (FULL_LOG_MASS_CHILD_SORT)
        child_sort_key = CHILD_ARENA_SORT_KEY_BITS'(log_m_q5_3);
      else
        child_sort_key = CHILD_ARENA_SORT_KEY_BITS'(dense_area) +
            CHILD_ARENA_SORT_KEY_BITS'(
                log_m_q5_3[CHILD_KEY_LOGM_BITS-1]
            );
    end
  endfunction

  assign fg_topology0 = stream_d0;
  assign fg_topology1 = stream_d1;
  assign fg_feature0 = stream_d0 & stream_feature_valid_d0;
  assign fg_feature1 = stream_d1 & stream_feature_valid_d1;
  assign bg_feature0 = ~stream_d0 & stream_feature_valid_d0;
  assign bg_feature1 = ~stream_d1 & stream_feature_valid_d1;
  assign bg0 = ~stream_d0;
  assign bg1 = ~stream_d1;

  TwoPPC #(
    .WIDTH(WIDTH),
    .HEIGHT(HEIGHT),
    .TABLE_DEPTH(OUTER_TABLE_DEPTH_RES),
    .HOLE_FILL(1'b1),
    .FEATURE_BBOX(1'b0),
    .FEATURE_AREA(1'b0),
    .FEATURE_PRESENT(1'b1),
    .FEATURE_MOMENTS(1'b0),
    .FEATURE_VALUE_WIDTH(1),
    .CHILD_COUNT_BITS(CHILD_ID_BITS),
    .CHILD_ID_MODE(1'b1),
    .EXPOSE_S5_FV(1'b1)
  ) outer_hier_2ppc (
    .clk(clk),
    .rst(rst),
    .datavalid(accept),
    .in_ready(outer_in_ready),
    .d0(fg_topology0),
    .d1(fg_topology1),
    .feature_d0(fg_feature0),
    .feature_d1(fg_feature1),
    .feature_value_d0(1'b0),
    .feature_value_d1(1'b0),
    .feature_y(stream_feature_y),
    .sof(stream_sof),
    .eol(stream_eol),
    .child_add0(child_event & ~child_lane),
    .child_add1(child_event & child_lane),
    .child_id0(child_arena_new_alloc_id),
    .child_id1(child_arena_new_alloc_id),
    .child_parent0_valid(outer_child_parent0_valid),
    .child_parent1_valid(outer_child_parent1_valid),
    .child_parent0_id(outer_child_parent0_id),
    .child_parent1_id(outer_child_parent1_id),
    .child_parent0_has_child(outer_child_parent0_has_child),
    .child_parent1_has_child(outer_child_parent1_has_child),
    .child_merge_valid(outer_child_merge_valid),
    .child_merge_id_a(outer_child_merge_id_a),
    .child_merge_id_b(outer_child_merge_id_b),
    .s5_fv_valid(outer_s5_fv_valid),
    .s5_fv_lane(unused_outer_s5_fv_lane),
    .s5_fv_data(outer_s5_fv_data),
    .s5_close_moment_valid(unused_outer_close_moment_valid),
    .s5_close_moment_lane(unused_outer_close_moment_lane),
    .s5_close_moment_data(unused_outer_close_moment_data),
    .fv_valid(outer_fv_valid),
    .fv_data({unused_outer_fv_payload, unused_outer_feature_source_present}),
    .fv_child_id(unused_outer_fv_child_id),
    .fv_child_valid(unused_outer_fv_child_valid)
  );

  TwoPPC #(
    .WIDTH(WIDTH),
    .HEIGHT(HEIGHT),
    .FEATURE_Y_HEIGHT(CENTROID_SOURCE_HEIGHT),
    .TABLE_DEPTH(INNER_TABLE_DEPTH_RES),
    .HOLE_FILL(1'b0),
    .FEATURE_BBOX(1'b0),
    .FEATURE_AREA(!FULL_LOG_MASS_CHILD_SORT),
    .FEATURE_AREA_BITS(INNER_AREA_BITS),
    .FEATURE_PRESENT(1'b0),
    .FEATURE_MOMENTS(1'b1),
    .CENTERED_MOMENTS(1'b1),
    .COMPRESS_CENTERED_MOMENTS(1'b1),
    .PAIR_FUSED_MOMENTS(1'b1),
    .USE_DSP_HIGH_ZOM(USE_DSP_HIGH_ZOM),
    .USE_DSP_HIGH_FOM(USE_DSP_HIGH_FOM),
    .FEATURE_VALUE_WIDTH(RAW_PIXEL_BITS),
    .FEATURE_Y_OVERRIDE(1'b1),
    .CHILD_COUNT_BITS(0),
    .CHILD_ID_MODE(1'b0),
    .EXPOSE_S5_FV(1'b0)
  ) inner_hier_2ppc (
    .clk(clk),
    .rst(rst),
    .datavalid(inner_emit),
    .in_ready(inner_in_ready),
    .d0(inner_pending_bg0),
    .d1(inner_pending_bg1),
    .feature_d0(inner_pending_feature0),
    .feature_d1(inner_pending_feature1),
    .feature_value_d0(inner_pending_raw0),
    .feature_value_d1(inner_pending_raw1),
    .feature_y(inner_pending_y),
    .sof(inner_pending_sof),
    .eol(inner_pending_eol),
    .child_add0(1'b0),
    .child_add1(1'b0),
    .child_id0(1'b0),
    .child_id1(1'b0),
    .child_parent0_valid(unused_inner_child_parent0_valid),
    .child_parent1_valid(unused_inner_child_parent1_valid),
    .child_parent0_id(unused_inner_child_parent0_id),
    .child_parent1_id(unused_inner_child_parent1_id),
    .child_parent0_has_child(unused_inner_child_parent0_has_child),
    .child_parent1_has_child(unused_inner_child_parent1_has_child),
    .child_merge_valid(unused_inner_child_merge_valid),
    .child_merge_id_a(unused_inner_child_merge_id_a),
    .child_merge_id_b(unused_inner_child_merge_id_b),
    .s5_fv_valid(unused_inner_s5_fv_valid),
    .s5_fv_lane(unused_inner_s5_fv_lane),
    .s5_fv_data(unused_inner_s5_fv_data),
    .s5_close_moment_valid(inner_close_moment_valid),
    .s5_close_moment_lane(inner_close_moment_lane),
    .s5_close_moment_data(inner_close_moment_data),
    .fv_valid(unused_inner_final_fv_valid),
    .fv_data(unused_inner_final_fv_data),
    .fv_child_id(unused_inner_fv_child_id),
    .fv_child_valid(unused_inner_fv_child_valid)
  );

  // The outer CCA needs the next accepted pair's first pixel to fill
  // diagonals. Its S0 therefore holds one pair across arbitrary input gaps,
  // except that EOL flushes on the following clock. Mirror that schedule for
  // the inner CCA instead of delaying its close pulse by a fixed wall clock.
  assign inner_emit = inner_pending_valid & (accept | inner_pending_eol);
  always_ff @(posedge clk) begin
    if (rst) begin
      inner_pending_valid <= 1'b0;
    end else if (accept) begin
      inner_pending_valid <= 1'b1;
      inner_pending_bg0 <= bg0;
      inner_pending_bg1 <= bg1;
      inner_pending_feature0 <= bg_feature0;
      inner_pending_feature1 <= bg_feature1;
      inner_pending_raw0 <= bg_feature0 ? stream_raw_d0 : '0;
      inner_pending_raw1 <= bg_feature1 ? stream_raw_d1 : '0;
      inner_pending_y <= stream_feature_y;
      inner_pending_sof <= stream_sof;
      inner_pending_eol <= stream_eol;
    end else if (inner_emit) begin
      inner_pending_valid <= 1'b0;
    end
  end
  initial begin
    assert (INNER_CLOSE_DELAY == 0)
      else $error("aligned hierarchy requires INNER_CLOSE_DELAY=0");
  end
  always_ff @(posedge clk) begin
    if (!rst)
      assert (!inner_emit || inner_in_ready)
        else $error("aligned inner CCA rejected a pending pair");
  end

  assign child_valid = inner_close_moment_valid;
  assign child_lane = inner_close_moment_lane;
  assign child_moment = inner_close_moment_data;

  // In full-log mode this value is used only as a nonempty marker and as a
  // don't-care input to the compile-time-selected area-key branch.
  assign child_moment_area = FULL_LOG_MASS_CHILD_SORT ?
      INNER_AREA_BITS'(1) : INNER_AREA_BITS'(child_moment);
  assign child_moment_zom = child_moment[OFF_INNER_ZOM +: MOMENT_ZOM_BITS];
  assign child_moment_x_fom_q =
      child_moment[OFF_INNER_X_FOM +: MOMENT_X_QUOTIENT_BITS];
  assign child_moment_y_fom_q =
      child_moment[OFF_INNER_Y_FOM +: MOMENT_Y_QUOTIENT_BITS];
  assign child_moment_x_fom = {
      child_moment_x_fom_q,
      MOMENT_X_CENTER_ODD & child_moment_zom[0]
  };
  assign child_moment_y_fom = {
      child_moment_y_fom_q,
      MOMENT_Y_CENTER_ODD & child_moment_zom[0]
  };

  assign parent_valid = child_lane ? outer_child_parent1_valid : outer_child_parent0_valid;
  assign parent_id = child_lane ? outer_child_parent1_id : outer_child_parent0_id;
  assign parent_has_child =
      child_lane ? outer_child_parent1_has_child : outer_child_parent0_has_child;
  assign child_event = child_valid & parent_valid & (child_moment_area != '0);
  assign raw_new_child = child_event & ~parent_has_child;

  assign centroid_in_valid = child_event;
  assign centroid_m = child_moment_zom[CENTROID_M_BITS-1:0];
  assign centroid_sx = child_moment_x_fom[CENTROID_SX_BITS-1:0];
  assign centroid_sy = child_moment_y_fom[CENTROID_SY_BITS-1:0];

  CentroidCompressor #(
    .WIDTH(CENTROID_SOURCE_WIDTH),
    .HEIGHT(CENTROID_SOURCE_HEIGHT),
    .PIXEL_BITS(RAW_PIXEL_BITS),
    .REGISTER_INPUT(CENTROID_REGISTER_INPUT),
    .POWER2_MULTIPLIER_NORMALIZER(CENTROID_POWER2_MULTIPLIER_NORMALIZER),
    .MIDPOINT_REFLECTION_RECIPROCAL(
        CENTROID_MIDPOINT_REFLECTION_RECIPROCAL
    ),
    .RECIPROCAL_SEGMENT_BIN(CENTROID_RECIPROCAL_SEGMENT_BIN),
    .RECIPROCAL_MIDPOINT_REFLECTION_BIN(
        CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN
    )
  ) inner_centroid_compressor (
    .clk(clk),
    .rst(rst),
    .in_valid(centroid_in_valid),
    .m(centroid_m),
    .sx(centroid_sx),
    .sy(centroid_sy),
    .out_valid(centroid_out_valid),
    .centroid_valid(centroid_centroid_valid),
    .x_q4(centroid_x_q4),
    .y_q4(centroid_y_q4),
    .m_log_q5_3(centroid_log_m_q5_3)
  );

  always_ff @(posedge clk) begin
    if (rst) begin
      centroid_valid_pipe <= '0;
      centroid_merge_pipe <= '0;
      centroid_read_pipe <= '0;
    end else begin
      centroid_valid_pipe[0] <= child_event;
      centroid_merge_pipe[0] <= outer_child_merge_valid;
      centroid_read_pipe[0] <= outer_read_event;
      for (int control_index = 1; control_index < CENTROID_LATENCY; control_index++) begin
        centroid_valid_pipe[control_index] <= centroid_valid_pipe[control_index - 1];
        centroid_merge_pipe[control_index] <= centroid_merge_pipe[control_index - 1];
        centroid_read_pipe[control_index] <= centroid_read_pipe[control_index - 1];
      end
    end
    centroid_id_pipe[0] <= raw_new_child ? child_arena_new_alloc_id : parent_id;
    centroid_area_key_pipe[0] <=
        child_area_key(CHILD_AREA_BITS_RES'(child_moment_area));
    centroid_init_pipe[0] <= raw_new_child;
    centroid_merge_id_a_pipe[0] <= outer_child_merge_id_a;
    centroid_merge_id_b_pipe[0] <= outer_child_merge_id_b;
    centroid_read_id_pipe[0] <= outer_s5_fv_data[1 +: CHILD_ID_BITS];
    for (int sideband_index = 1; sideband_index < CENTROID_LATENCY; sideband_index++) begin
      centroid_id_pipe[sideband_index] <= centroid_id_pipe[sideband_index - 1];
      centroid_area_key_pipe[sideband_index] <= centroid_area_key_pipe[sideband_index - 1];
      centroid_init_pipe[sideband_index] <= centroid_init_pipe[sideband_index - 1];
      centroid_merge_id_a_pipe[sideband_index] <=
          centroid_merge_id_a_pipe[sideband_index - 1];
      centroid_merge_id_b_pipe[sideband_index] <=
          centroid_merge_id_b_pipe[sideband_index - 1];
      centroid_read_id_pipe[sideband_index] <= centroid_read_id_pipe[sideband_index - 1];
    end
  end

  // FEATURE_PRESENT occupies bit zero in the outer CCA specialization;
  // its child-list handle follows immediately. Capture closes at the same
  // S5 epoch as child/merge events, then delay all three by centroid latency.
  // The read event must follow the delayed last child.
  assign outer_read_event = outer_s5_fv_valid & outer_s5_fv_data[0]
      & (outer_s5_fv_data[1 +: CHILD_ID_BITS] != {CHILD_ID_BITS{1'b1}});

  assign compressed_child_valid = centroid_out_valid & centroid_valid_pipe[CENTROID_LATENCY-1];
  assign compressed_child_id = centroid_id_pipe[CENTROID_LATENCY-1];
  assign compressed_child_area_key = centroid_area_key_pipe[CENTROID_LATENCY-1];
  assign compressed_child_key = child_sort_key(
      compressed_child_area_key,
      centroid_log_m_q5_3
  );
  generate
    if (INNER_CENTROID_ALWAYS_VALID) begin : always_valid_child_fv
      assign compressed_child_fv = {
          centroid_y_q4,
          centroid_x_q4,
          compressed_child_key
      };
    end else begin : generic_child_fv
      assign compressed_child_fv = {
          centroid_y_q4,
          centroid_x_q4,
          centroid_centroid_valid,
          compressed_child_key
      };
    end
  endgenerate
  assign child_arena_new_valid = raw_new_child;
  assign bundle_child_valid = compressed_child_valid;
  assign bundle_merge_valid = centroid_merge_pipe[CENTROID_LATENCY-1];
  assign bundle_read_valid = centroid_read_pipe[CENTROID_LATENCY-1];
  assign child_arena_cmd_valid = bundle_child_valid | bundle_merge_valid | bundle_read_valid;
  assign child_arena_cmd_op = centroid_init_pipe[CENTROID_LATENCY-1] ?
      CHILD_ARENA_OP_INIT : CHILD_ARENA_OP_ADD;
  assign child_arena_cmd_id_a = compressed_child_id;
  assign child_arena_cmd_id_b = '0;
  assign child_arena_cmd_fv = compressed_child_fv;

  // Admission credit reserves space for every event already in the fixed
  // pipeline. In-flight events continue entering the FIFO after admission
  // stops; dropping any of them would corrupt list ownership and selection.
  always_ff @(posedge clk) begin
    if (!rst) begin
      assert (centroid_out_valid == centroid_valid_pipe[CENTROID_LATENCY-1])
        else $error("centroid output and hierarchy sidebands lost alignment");
      assert (child_arena_queue_occupancy <= COMMAND_COUNT_BITS'(CHILD_ARENA_CMD_QUEUE_DEPTH))
        else $error("hierarchy ordered event FIFO count exceeds capacity");
      assert (!child_arena_cmd_valid || child_arena_cmd_ready)
        else $error("hierarchy ordered event FIFO overflow");
      assert (!raw_new_child || child_arena_new_ready)
        else $error("hierarchy child-list allocation exhausted");
    end
  end

  ChildArena #(
    .ENTRY_COUNT(CHILD_ARENA_ENTRIES),
    .MAX_LIST_LEN(MAX_CHILD_LIST_LEN),
    .FEATURE_WIDTH(INNER_DATA_WIDTH),
    .SORT_KEY_WIDTH(CHILD_ARENA_SORT_KEY_BITS),
    .TAIL_KEY_LSB(FULL_LOG_MASS_CHILD_SORT ? 5 : 0),
    .TAIL_KEY_WIDTH(CHILD_KEY_AREA_BITS),
    .TAIL_KEY_DENSE_AREA_RANK(!FULL_LOG_MASS_CHILD_SORT),
    .EQUAL_KEY_SERVICE_MIN(1'b1),
    .CMD_QUEUE_DEPTH(CHILD_ARENA_CMD_QUEUE_DEPTH),
    .BUNDLED_COMMANDS(1'b1),
    .BUNDLE_ADMISSION_RESERVE(BUNDLE_ADMISSION_RESERVE)
  ) child_arena (
    .clk(clk),
    .rst(rst),
    .new_valid(child_arena_new_valid),
    .new_ready(child_arena_new_ready),
    .new_alloc_id(child_arena_new_alloc_id),
    .new_rsp_valid(unused_child_arena_new_rsp_valid),
    .new_rsp_ready(1'b1),
    .new_rsp_id(unused_child_arena_new_rsp_id),
    .new_rsp_status(unused_child_arena_new_rsp_status),
    .cmd_valid(child_arena_cmd_valid),
    .cmd_ready(child_arena_cmd_ready),
    .cmd_op(child_arena_cmd_op),
    .cmd_id_a(child_arena_cmd_id_a),
    .cmd_id_b(child_arena_cmd_id_b),
    .cmd_fv(child_arena_cmd_fv),
    .bundle_child_valid(bundle_child_valid), .bundle_merge_valid(bundle_merge_valid),
    .bundle_merge_id_a(centroid_merge_id_a_pipe[CENTROID_LATENCY-1]),
    .bundle_merge_id_b(centroid_merge_id_b_pipe[CENTROID_LATENCY-1]),
    .bundle_read_valid(bundle_read_valid),
    .bundle_read_id(centroid_read_id_pipe[CENTROID_LATENCY-1]),
    .cmd_admit_ready(child_arena_admit_ready),
    .cmd_queue_occupancy(child_arena_queue_occupancy),
    .out_valid(child_arena_out_valid),
    .out_ready(1'b1),
    .out_fv(child_arena_out_fv),
    .out_first(unused_child_arena_out_first),
    .out_last(child_arena_out_last),
    .idle(unused_child_arena_idle),
    .free_count(unused_child_arena_free_count)
  );

  always_ff @(posedge clk) begin
    if (rst) begin
      row <= '0;
      col <= '0;
      in_ready <= 1'b1;
      input_buffer_valid <= 1'b0;
      inner_fv_valid <= 1'b0;
      inner_end <= 1'b0;
    end else begin
      inner_fv_valid <= 1'b0;
      inner_end <= 1'b0;

      if (child_arena_out_valid) begin
        inner_fv_valid <= 1'b1;
        inner_fv_data <= child_arena_out_fv;
        inner_end <= child_arena_out_last;
      end

      in_ready <= ~input_buffer_next_valid;
      if (input_fire & ~accept) begin
        input_buffer_valid <= 1'b1;
        input_buffer_d0 <= d0;
        input_buffer_d1 <= d1;
        input_buffer_raw_d0 <= raw_d0;
        input_buffer_raw_d1 <= raw_d1;
        input_buffer_feature_valid_d0 <= feature_valid_d0;
        input_buffer_feature_valid_d1 <= feature_valid_d1;
        input_buffer_feature_y <= feature_y;
        input_buffer_sof <= sof;
        input_buffer_eol <= eol;
      end else if (accept & input_buffer_valid) begin
        input_buffer_valid <= 1'b0;
      end

      if (accept) begin
        if (stream_eol) begin
          col <= '0;
          row <= row + ROW_BITS'(1);
        end else begin
          col <= col + PAIR_BITS'(1);
        end
      end

      if (ASSERT_READ_STAGE_OVERFLOW) begin
        assert (!compressed_child_valid | (compressed_child_area_key != '0));
        if (INNER_CENTROID_ALWAYS_VALID) begin
          assert (!child_event | (child_moment_zom != '0));
          assert (!compressed_child_valid | centroid_centroid_valid);
        end
      end
    end
  end
endmodule
