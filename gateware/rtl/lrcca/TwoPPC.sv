// Canonical readable SystemVerilog stage for the two-pixel-per-cycle CCA pipeline.

import lrcca_util_pkg::area_width;
import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::moment_x_width;
import lrcca_util_pkg::moment_y_width;
import lrcca_util_pkg::moment_zom_width;

module TwoPPC #(
  // Main Titanium profile only; generic targets retain fabric arithmetic.
  parameter bit USE_DSP_HIGH_ZOM = 1'b0,
  parameter bit USE_DSP_HIGH_FOM = 1'b0,
  parameter int WIDTH = 48,
  parameter int HEIGHT = 36,
  parameter int TABLE_DEPTH = 30,
  parameter bit HOLE_FILL = 1'b0,
  parameter bit FEATURE_BBOX = 1'b1,
  parameter bit FEATURE_AREA = 1'b1,
  parameter int FEATURE_AREA_BITS = 0,
  parameter int FEATURE_AREA_MAX = -1,
  parameter bit FEATURE_PRESENT = 1'b0,
  parameter bit FEATURE_MOMENTS = 1'b0,
  parameter bit CENTERED_MOMENTS = 1'b0,
  parameter bit COMPRESS_CENTERED_MOMENTS = 1'b0,
  parameter bit PAIR_FUSED_MOMENTS = 1'b0,
  parameter int FEATURE_VALUE_WIDTH = 1,
  parameter bit FEATURE_Y_OVERRIDE = 1'b0,
  parameter int FEATURE_Y_HEIGHT = HEIGHT,
  parameter int CHILD_COUNT_BITS = 0,
  parameter bit CHILD_ID_MODE = 1'b0,
  parameter bit EXPOSE_S5_FV = 1'b1,
  localparam int FULL_AREA_BITS = area_width(WIDTH, HEIGHT),
  localparam int AREA_BITS = (FEATURE_AREA_BITS == 0) ? FULL_AREA_BITS : FEATURE_AREA_BITS,
  localparam int X_BITS = bit_width(longint'(WIDTH)),
  localparam int Y_BITS = bit_width(longint'(HEIGHT)),
  localparam int ROW_BITS = bit_width(longint'(HEIGHT) + 1),
  localparam int PAIR_COUNT = WIDTH / 2,
  localparam int PAIR_BITS = bit_width(longint'(PAIR_COUNT)),
  localparam int LABEL_BITS = bit_width(longint'(TABLE_DEPTH)),
  localparam int MOMENT_ZOM_BITS = moment_zom_width(WIDTH, HEIGHT, FEATURE_VALUE_WIDTH),
  localparam bit CENTERED_MOMENT_QUOTIENTS =
      FEATURE_MOMENTS && CENTERED_MOMENTS && COMPRESS_CENTERED_MOMENTS,
  localparam int MOMENT_X_FULL_BITS = CENTERED_MOMENTS ?
      centered_moment_bits(WIDTH, HEIGHT, FEATURE_VALUE_WIDTH) :
      moment_x_width(WIDTH, HEIGHT, FEATURE_VALUE_WIDTH),
  localparam int MOMENT_Y_FULL_BITS = CENTERED_MOMENTS ?
      centered_moment_bits(HEIGHT, WIDTH, FEATURE_VALUE_WIDTH) :
      moment_y_width(WIDTH, HEIGHT, FEATURE_VALUE_WIDTH),
  localparam int MOMENT_X_BITS =
      MOMENT_X_FULL_BITS - (CENTERED_MOMENT_QUOTIENTS ? 1 : 0),
  localparam int MOMENT_Y_BITS =
      MOMENT_Y_FULL_BITS - (CENTERED_MOMENT_QUOTIENTS ? 1 : 0),
  localparam int CHILD_WIDTH = (CHILD_COUNT_BITS > 0) ? CHILD_COUNT_BITS : 1,
  localparam int BBOX_FIELD_BITS = FEATURE_BBOX ? ((2 * X_BITS) + (2 * Y_BITS)) : 0,
  localparam int AREA_FIELD_BITS = FEATURE_AREA ? AREA_BITS : 0,
  localparam int PRESENT_FIELD_BITS = FEATURE_PRESENT ? 1 : 0,
  localparam int MOMENT_FIELD_BITS =
      FEATURE_MOMENTS ? (MOMENT_ZOM_BITS + MOMENT_X_BITS + MOMENT_Y_BITS) : 0,
  localparam int CHILD_FIELD_BITS = (CHILD_COUNT_BITS > 0) ? CHILD_COUNT_BITS : 0,
  localparam int S5_CLOSE_MOMENT_WIDTH_RAW = AREA_FIELD_BITS + MOMENT_FIELD_BITS,
  localparam int S5_CLOSE_MOMENT_WIDTH =
      (S5_CLOSE_MOMENT_WIDTH_RAW == 0) ? 1 : S5_CLOSE_MOMENT_WIDTH_RAW,
  localparam int DATA_WIDTH_RAW =
      BBOX_FIELD_BITS + AREA_FIELD_BITS + PRESENT_FIELD_BITS +
      MOMENT_FIELD_BITS + CHILD_FIELD_BITS,
  localparam int DATA_WIDTH = (DATA_WIDTH_RAW == 0) ? 1 : DATA_WIDTH_RAW
)(
  input  logic clk,
  input  logic rst,

  input  logic datavalid,
  output logic in_ready,
  input  logic d0,
  input  logic d1,
  input  logic feature_d0,
  input  logic feature_d1,
  input  logic [FEATURE_VALUE_WIDTH-1:0] feature_value_d0,
  input  logic [FEATURE_VALUE_WIDTH-1:0] feature_value_d1,
  input  logic [ROW_BITS-1:0] feature_y,
  input  logic sof,
  input  logic eol,

  input  logic child_add0,
  input  logic child_add1,
  input  logic [CHILD_WIDTH-1:0] child_id0,
  input  logic [CHILD_WIDTH-1:0] child_id1,
  output logic child_parent0_valid,
  output logic child_parent1_valid,
  output logic [CHILD_WIDTH-1:0] child_parent0_id,
  output logic [CHILD_WIDTH-1:0] child_parent1_id,
  output logic child_parent0_has_child,
  output logic child_parent1_has_child,
  output logic child_merge_valid,
  output logic [CHILD_WIDTH-1:0] child_merge_id_a,
  output logic [CHILD_WIDTH-1:0] child_merge_id_b,

  output logic s5_fv_valid,
  output logic s5_fv_lane,
  output logic [DATA_WIDTH-1:0] s5_fv_data,
  output logic s5_close_moment_valid,
  output logic s5_close_moment_lane,
  output logic [S5_CLOSE_MOMENT_WIDTH-1:0] s5_close_moment_data,
  output logic fv_valid,
  output logic [DATA_WIDTH-1:0] fv_data,
  output logic [CHILD_WIDTH-1:0] fv_child_id,
  output logic fv_child_valid
);
  // SOF is retained for interface compatibility; stream geometry is derived from EOL.
  logic unused_sof;
  assign unused_sof = sof;

  localparam int PIPE_DRAIN_CYCLES = 10;

  localparam int OFF_MAX_Y = 0;
  localparam int OFF_MIN_Y = OFF_MAX_Y + (FEATURE_BBOX ? Y_BITS : 0);
  localparam int OFF_MAX_X = OFF_MIN_Y + (FEATURE_BBOX ? Y_BITS : 0);
  localparam int OFF_MIN_X = OFF_MAX_X + (FEATURE_BBOX ? X_BITS : 0);
  localparam int OFF_AREA = OFF_MIN_X + (FEATURE_BBOX ? X_BITS : 0);
  localparam int OFF_PRESENT = OFF_AREA + AREA_FIELD_BITS;
  localparam int OFF_ZOM = OFF_PRESENT + PRESENT_FIELD_BITS;
  localparam int OFF_X_FOM = OFF_ZOM + (FEATURE_MOMENTS ? MOMENT_ZOM_BITS : 0);
  localparam int OFF_Y_FOM = OFF_X_FOM + (FEATURE_MOMENTS ? MOMENT_X_BITS : 0);
  localparam int OFF_CHILD = OFF_Y_FOM + (FEATURE_MOMENTS ? MOMENT_Y_BITS : 0);
  localparam logic [CHILD_WIDTH-1:0] CHILD_ID_NULL =
      CHILD_ID_MODE ? {CHILD_WIDTH{1'b1}} : '0;

  typedef struct packed {
    logic valid;
    logic row_start;
    logic row_end;
    logic curr_bank;
    logic prev_bank;
    logic [ROW_BITS-1:0] row;
    logic [PAIR_BITS-1:0] col;
    logic a;
    logic c;
    logic d0;
    logic d1;
    logic raw_d0;
    logic raw_d1;
    logic feature_d0;
    logic feature_d1;
    logic [FEATURE_VALUE_WIDTH-1:0] feature_value_d0;
    logic [FEATURE_VALUE_WIDTH-1:0] feature_value_d1;
    logic [ROW_BITS-1:0] feature_y;
  } s0_t;

  typedef struct packed {
    logic valid;
    logic row_start;
    logic row_end;
    logic curr_bank;
    logic prev_bank;
    logic [ROW_BITS-1:0] row;
    logic [PAIR_BITS-1:0] col;
    logic d0;
    logic d1;
    logic raw_d0;
    logic raw_d1;
    logic feature_d0;
    logic feature_d1;
    logic [FEATURE_VALUE_WIDTH-1:0] feature_value_d0;
    logic [FEATURE_VALUE_WIDTH-1:0] feature_value_d1;
    logic [ROW_BITS-1:0] feature_y;
    logic a;
    logic b0;
    logic b1;
    logic c;
    logic pf_valid;
    logic [LABEL_BITS-1:0] pf_index;
    logic [LABEL_BITS-1:0] pf_next;
    logic pf_mapped;
    logic [LABEL_BITS-1:0] pf_head;
  } pipe_t;

  typedef struct packed {
    logic valid;
    logic [LABEL_BITS-1:0] index;
    logic [LABEL_BITS-1:0] next;
    logic has_next;
    logic mapped;
    logic [LABEL_BITS-1:0] head;
    logic [LABEL_BITS-1:0] tail;
    logic tail_valid;
    logic [DATA_WIDTH-1:0] data;
  } token_t;

  typedef struct packed {
    logic [LABEL_BITS-1:0] c;
    logic f;
    logic [LABEL_BITS-1:0] h;
    logic [LABEL_BITS-1:0] t;
    logic [DATA_WIDTH-1:0] d;
  } state_t;

  typedef struct packed {
    logic head_prev_valid;
    logic [LABEL_BITS-1:0] head_prev_addr;
    logic [LABEL_BITS-1:0] head_prev_head;
    logic [LABEL_BITS-1:0] head_prev_tail;
    logic [DATA_WIDTH-1:0] head_prev_data;
    logic data_prev_valid;
    logic [LABEL_BITS-1:0] data_prev_addr;
    logic [DATA_WIDTH-1:0] data_prev_data;
    logic close_valid;
    logic [LABEL_BITS-1:0] close_addr;
    logic [DATA_WIDTH-1:0] close_data;
    logic tail_valid;
    logic [LABEL_BITS-1:0] tail_addr;
    logic [LABEL_BITS-1:0] tail_data;
  } repair_t;

  typedef struct packed {
    logic head_prev_valid;
    logic [LABEL_BITS-1:0] head_prev_addr;
    logic [LABEL_BITS-1:0] head_prev_head;
    logic [LABEL_BITS-1:0] head_prev_tail;
    logic data_prev_valid;
    logic [LABEL_BITS-1:0] data_prev_addr;
    logic tail_valid;
    logic [LABEL_BITS-1:0] tail_addr;
    logic [LABEL_BITS-1:0] tail_data;
  } repair_d1_t;

  typedef struct packed {
    logic head_prev_valid;
    logic [LABEL_BITS-1:0] head_prev_addr;
    logic [LABEL_BITS-1:0] head_prev_head;
    logic data_prev_valid;
    logic [LABEL_BITS-1:0] data_prev_addr;
  } repair_d2_t;

  typedef struct packed {
    logic valid;
    logic curr_bank;
    logic prev_bank;
    logic close_next_valid;
    logic [LABEL_BITS-1:0] close_next_addr;
    logic touch_valid;
    logic [LABEL_BITS-1:0] touch_addr;
    logic [LABEL_BITS-1:0] touch_data;
    logic head_curr_valid;
    logic [LABEL_BITS-1:0] head_curr_addr;
    logic head_prev_valid;
    logic [LABEL_BITS-1:0] head_prev_addr;
    logic [LABEL_BITS-1:0] head_prev_head;
    logic [LABEL_BITS-1:0] head_prev_tail;
    logic [DATA_WIDTH-1:0] head_prev_data;
    logic data_prev_valid;
    logic [LABEL_BITS-1:0] data_prev_addr;
    logic [DATA_WIDTH-1:0] data_prev_data;
    logic close_valid;
    logic [LABEL_BITS-1:0] close_addr;
    logic [DATA_WIDTH-1:0] close_data;
    logic tail_valid;
    logic [LABEL_BITS-1:0] tail_addr;
    logic [LABEL_BITS-1:0] tail_data;
    logic output_valid;
    logic [DATA_WIDTH-1:0] output_data;
  } s6a_t;

  typedef struct packed {
    logic valid;
    logic curr_bank;
    logic prev_bank;
    logic next0_valid;
    logic [LABEL_BITS-1:0] next0_addr;
    logic [LABEL_BITS-1:0] next0_data;
    logic next1_valid;
    logic [LABEL_BITS-1:0] next1_addr;
    logic [LABEL_BITS-1:0] next1_data;
    logic head_curr_valid;
    logic [LABEL_BITS-1:0] head_curr_addr;
    logic head_prev_valid;
    logic [LABEL_BITS-1:0] head_prev_addr;
    logic [LABEL_BITS-1:0] head_prev_head;
    logic [LABEL_BITS-1:0] head_prev_tail;
    logic [DATA_WIDTH-1:0] head_prev_data;
    logic data_prev_valid;
    logic [LABEL_BITS-1:0] data_prev_addr;
    logic [DATA_WIDTH-1:0] data_prev_data;
    logic tail_valid;
    logic [LABEL_BITS-1:0] tail_addr;
    logic [LABEL_BITS-1:0] tail_data;
    logic curr_first_valid;
    logic [LABEL_BITS-1:0] curr_first_addr;
    logic [DATA_WIDTH-1:0] curr_first_word;
    logic output_valid;
    logic [DATA_WIDTH-1:0] output_data;
  } s6b_t;

  typedef struct packed {
    logic next_valid;
    logic [LABEL_BITS-1:0] next_addr;
    logic [LABEL_BITS-1:0] next_data;
    logic tail_valid;
    logic [LABEL_BITS-1:0] tail_addr;
    logic [LABEL_BITS-1:0] tail_data;
  } pix_w_t;

  typedef struct packed {
    logic valid;
    logic [LABEL_BITS-1:0] addr;
    logic [LABEL_BITS-1:0] tail;
    logic [DATA_WIDTH-1:0] data;
    logic next_valid;
    logic [LABEL_BITS-1:0] next_addr;
    logic [LABEL_BITS-1:0] next_data;
    logic head_valid;
    logic [LABEL_BITS-1:0] head_addr;
  } close_w_t;

  typedef struct packed {
    logic head_valid;
    logic [LABEL_BITS-1:0] head_addr;
    logic [LABEL_BITS-1:0] head_head;
    logic [LABEL_BITS-1:0] head_tail;
    logic [DATA_WIDTH-1:0] head_data;
    logic data_valid;
    logic [LABEL_BITS-1:0] data_addr;
    logic [DATA_WIDTH-1:0] data_data;
    logic out_valid;
    logic [DATA_WIDTH-1:0] out_data;
  } ret_w_t;

  typedef struct packed {
    state_t state_ret;
    token_t token_ret;
    pix_w_t pix_w;
    close_w_t close_w;
    ret_w_t ret_w;
    logic child_merge_valid;
    logic [CHILD_WIDTH-1:0] child_merge_id_a;
    logic [CHILD_WIDTH-1:0] child_merge_id_b;
  } lane_result_t;

  s0_t s0;
  logic s0_stage_b0;
  logic s0_stage_b1;
  pipe_t s1;
  pipe_t s2;
  pipe_t s3;
  pipe_t s4b;
  token_t s4b_token;
  s6a_t s6a_next;
  s6b_t s6b;
  typedef struct packed {
    logic head_prev_valid;
    logic [LABEL_BITS-1:0] head_prev_addr;
    logic [LABEL_BITS-1:0] head_prev_head;
    logic [LABEL_BITS-1:0] head_prev_tail;
    logic data_prev_valid;
    logic [LABEL_BITS-1:0] data_prev_addr;
    logic tail_valid;
    logic [LABEL_BITS-1:0] tail_addr;
    logic [LABEL_BITS-1:0] tail_data;
  } repair_s4_d1_t;

  repair_t s6_repair;
  repair_d1_t s6_repair_d1;
  repair_d2_t s6_repair_d2;
  repair_s4_d1_t s4_repair_d1;

  assign s4_repair_d1 = s6_repair_d1;

  function automatic logic [CHILD_WIDTH-1:0] feature_count_of(
    input logic [DATA_WIDTH-1:0] feature
  );
    begin
      if (CHILD_COUNT_BITS > 0) begin
        feature_count_of = CHILD_WIDTH'(feature >> OFF_CHILD);
      end else begin
        feature_count_of = '0;
      end
    end
  endfunction

  function automatic logic child_count_has_child(
    input logic [CHILD_WIDTH-1:0] value
  );
    begin
      if (CHILD_COUNT_BITS == 0) begin
        child_count_has_child = 1'b0;
      end else if (CHILD_ID_MODE) begin
        child_count_has_child = value != CHILD_ID_NULL;
      end else begin
        child_count_has_child = value != '0;
      end
    end
  endfunction

  initial begin
    if (PAIR_FUSED_MOMENTS && HOLE_FILL) begin
      $error("PAIR_FUSED_MOMENTS requires HOLE_FILL=0");
    end
  end

  TwoPPCS0Stage #(
    .WIDTH(WIDTH),
    .HEIGHT(HEIGHT),
    .HOLE_FILL(HOLE_FILL),
    .ROW_BITS(ROW_BITS),
    .PAIR_BITS(PAIR_BITS),
    .FEATURE_VALUE_WIDTH(FEATURE_VALUE_WIDTH),
    .PIPE_DRAIN_CYCLES(PIPE_DRAIN_CYCLES)
  ) s0_stage (
    .clk(clk),
    .rst(rst),
    .datavalid(datavalid),
    .d0(d0),
    .d1(d1),
    .feature_d0(feature_d0),
    .feature_d1(feature_d1),
    .feature_value_d0(feature_value_d0),
    .feature_value_d1(feature_value_d1),
    .feature_y(feature_y),
    .eol(eol),
    .in_ready(in_ready),
    .b0(s0_stage_b0),
    .b1(s0_stage_b1),
    .out(s0)
  );

  TwoPPCS1Stage #(
    .ROW_BITS(ROW_BITS),
    .PAIR_BITS(PAIR_BITS),
    .LABEL_BITS(LABEL_BITS),
    .FEATURE_VALUE_WIDTH(FEATURE_VALUE_WIDTH)
  ) s1_stage (
    .clk(clk),
    .rst(rst),
    .s0_valid(s0.valid),
    .s0_row_start(s0.row_start),
    .s0_row_end(s0.row_end),
    .s0_curr_bank(s0.curr_bank),
    .s0_prev_bank(s0.prev_bank),
    .s0_row(s0.row),
    .s0_col(s0.col),
    .s0_a(s0.a),
    .s0_c(s0.c),
    .s0_d0(s0.d0),
    .s0_d1(s0.d1),
    .s0_raw_d0(s0.raw_d0),
    .s0_raw_d1(s0.raw_d1),
    .s0_feature_d0(s0.feature_d0),
    .s0_feature_d1(s0.feature_d1),
    .s0_feature_value_d0(s0.feature_value_d0),
    .s0_feature_value_d1(s0.feature_value_d1),
    .s0_feature_y(s0.feature_y),
    .b0(s0_stage_b0),
    .b1(s0_stage_b1),
    .out_valid(s1.valid),
    .out_row_start(s1.row_start),
    .out_row_end(s1.row_end),
    .out_curr_bank(s1.curr_bank),
    .out_prev_bank(s1.prev_bank),
    .out_row(s1.row),
    .out_col(s1.col),
    .out_d0(s1.d0),
    .out_d1(s1.d1),
    .out_raw_d0(s1.raw_d0),
    .out_raw_d1(s1.raw_d1),
    .out_feature_d0(s1.feature_d0),
    .out_feature_d1(s1.feature_d1),
    .out_feature_value_d0(s1.feature_value_d0),
    .out_feature_value_d1(s1.feature_value_d1),
    .out_feature_y(s1.feature_y),
    .out_a(s1.a),
    .out_b0(s1.b0),
    .out_b1(s1.b1),
    .out_c(s1.c),
    .out_pf_valid(s1.pf_valid),
    .out_pf_index(s1.pf_index),
    .out_pf_next(s1.pf_next),
    .out_pf_mapped(s1.pf_mapped),
    .out_pf_head(s1.pf_head)
  );

  TwoPPCS2Stage #(
    .ROW_BITS(ROW_BITS),
    .PAIR_BITS(PAIR_BITS),
    .LABEL_BITS(LABEL_BITS),
    .FEATURE_VALUE_WIDTH(FEATURE_VALUE_WIDTH)
  ) s2_stage (
    .clk(clk),
    .rst(rst),
    .inp_valid(s1.valid),
    .inp_row_start(s1.row_start),
    .inp_row_end(s1.row_end),
    .inp_curr_bank(s1.curr_bank),
    .inp_prev_bank(s1.prev_bank),
    .inp_row(s1.row),
    .inp_col(s1.col),
    .inp_d0(s1.d0),
    .inp_d1(s1.d1),
    .inp_raw_d0(s1.raw_d0),
    .inp_raw_d1(s1.raw_d1),
    .inp_feature_d0(s1.feature_d0),
    .inp_feature_d1(s1.feature_d1),
    .inp_feature_value_d0(s1.feature_value_d0),
    .inp_feature_value_d1(s1.feature_value_d1),
    .inp_feature_y(s1.feature_y),
    .inp_a(s1.a),
    .inp_b0(s1.b0),
    .inp_b1(s1.b1),
    .inp_c(s1.c),
    .inp_pf_valid(s1.pf_valid),
    .inp_pf_index(s1.pf_index),
    .out_valid(s2.valid),
    .out_row_start(s2.row_start),
    .out_row_end(s2.row_end),
    .out_curr_bank(s2.curr_bank),
    .out_prev_bank(s2.prev_bank),
    .out_row(s2.row),
    .out_col(s2.col),
    .out_d0(s2.d0),
    .out_d1(s2.d1),
    .out_raw_d0(s2.raw_d0),
    .out_raw_d1(s2.raw_d1),
    .out_feature_d0(s2.feature_d0),
    .out_feature_d1(s2.feature_d1),
    .out_feature_value_d0(s2.feature_value_d0),
    .out_feature_value_d1(s2.feature_value_d1),
    .out_feature_y(s2.feature_y),
    .out_a(s2.a),
    .out_b0(s2.b0),
    .out_b1(s2.b1),
    .out_c(s2.c),
    .out_pf_valid(s2.pf_valid),
    .out_pf_index(s2.pf_index),
    .out_pf_next(s2.pf_next),
    .out_pf_mapped(s2.pf_mapped),
    .out_pf_head(s2.pf_head)
  );

  TwoPPCS3Stage #(
    .ROW_BITS(ROW_BITS),
    .PAIR_BITS(PAIR_BITS),
    .LABEL_BITS(LABEL_BITS),
    .FEATURE_VALUE_WIDTH(FEATURE_VALUE_WIDTH),
    .DATA_WIDTH(DATA_WIDTH),
    .TABLE_DEPTH(TABLE_DEPTH)
  ) s3_stage (
    .clk(clk),
    .rst(rst),
    .inp(s2),
    .s1_in(s1),
    .s6b_in(s6b),
    .s6_repair_in(s6_repair),
    .s6_repair_d1_in(s6_repair_d1),
    .s6_repair_d2_in(s6_repair_d2),
    .out(s3)
  );

  TwoPPCS4Stage #(
    .ROW_BITS(ROW_BITS),
    .PAIR_BITS(PAIR_BITS),
    .LABEL_BITS(LABEL_BITS),
    .FEATURE_VALUE_WIDTH(FEATURE_VALUE_WIDTH),
    .DATA_WIDTH(DATA_WIDTH)
  ) s4_stage (
    .inp(s3),
    .s6_repair(s6_repair),
    .s6_repair_d1(s4_repair_d1),
    .out(s4b),
    .token_out(s4b_token)
  );

  TwoPPCS5Stage #(
    .USE_DSP_HIGH_ZOM(USE_DSP_HIGH_ZOM),
    .USE_DSP_HIGH_FOM(USE_DSP_HIGH_FOM),
    .WIDTH(WIDTH),
    .HEIGHT(HEIGHT),
    .FEATURE_Y_HEIGHT(FEATURE_Y_HEIGHT),
    .TABLE_DEPTH(TABLE_DEPTH),
    .FEATURE_BBOX(FEATURE_BBOX),
    .FEATURE_AREA(FEATURE_AREA),
    .FEATURE_AREA_BITS(FEATURE_AREA_BITS),
    .FEATURE_AREA_MAX(FEATURE_AREA_MAX),
    .FEATURE_PRESENT(FEATURE_PRESENT),
    .FEATURE_MOMENTS(FEATURE_MOMENTS),
    .CENTERED_MOMENTS(CENTERED_MOMENTS),
    .COMPRESS_CENTERED_MOMENTS(COMPRESS_CENTERED_MOMENTS),
    .PAIR_FUSED_MOMENTS(PAIR_FUSED_MOMENTS),
    .FEATURE_VALUE_WIDTH(FEATURE_VALUE_WIDTH),
    .FEATURE_Y_OVERRIDE(FEATURE_Y_OVERRIDE),
    .CHILD_COUNT_BITS(CHILD_COUNT_BITS),
    .CHILD_ID_MODE(CHILD_ID_MODE),
    .EXPOSE_S5_FV(EXPOSE_S5_FV)
  ) s5_stage (
    .clk(clk),
    .rst(rst),
    .inp(s4b),
    .token_in(s4b_token),
    .child_add0(child_add0),
    .child_add1(child_add1),
    .child_id0(child_id0),
    .child_id1(child_id1),
    .s6a_out(s6a_next),
    .s6_repair_out(s6_repair),
    .s6_repair_d1_out(s6_repair_d1),
    .s6_repair_d2_out(s6_repair_d2),
    .child_parent0_valid(child_parent0_valid),
    .child_parent1_valid(child_parent1_valid),
    .child_parent0_id(child_parent0_id),
    .child_parent1_id(child_parent1_id),
    .child_parent0_has_child(child_parent0_has_child),
    .child_parent1_has_child(child_parent1_has_child),
    .child_merge_valid(child_merge_valid),
    .child_merge_id_a(child_merge_id_a),
    .child_merge_id_b(child_merge_id_b),
    .s5_fv_valid(s5_fv_valid),
    .s5_fv_lane(s5_fv_lane),
    .s5_fv_data(s5_fv_data),
    .s5_close_moment_valid(s5_close_moment_valid),
    .s5_close_moment_lane(s5_close_moment_lane),
    .s5_close_moment_data(s5_close_moment_data)
  );

  TwoPPCS6BStage #(
    .LABEL_BITS(LABEL_BITS),
    .DATA_WIDTH(DATA_WIDTH)
  ) s6b_stage (
    .clk(clk),
    .rst(rst),
    .inp(s6a_next),
    .out(s6b)
  );

  assign fv_valid = s6b.output_valid;
  assign fv_data = s6b.output_data;
  assign fv_child_id = feature_count_of(s6b.output_data);
  assign fv_child_valid = child_count_has_child(fv_child_id);

endmodule
