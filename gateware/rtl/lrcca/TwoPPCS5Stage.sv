// Canonical readable SystemVerilog stage for the two-pixel-per-cycle CCA pipeline.

import lrcca_util_pkg::area_width;
import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::moment_x_width;
import lrcca_util_pkg::moment_y_width;
import lrcca_util_pkg::moment_zom_width;

module TwoPPCS5Stage #(
  // Main Titanium profile only; generic targets retain fabric arithmetic.
  parameter bit USE_DSP_HIGH_ZOM = 1'b0,
  parameter bit USE_DSP_HIGH_FOM = 1'b0,
  parameter int WIDTH = 48,
  parameter int HEIGHT = 36,
  parameter int FEATURE_Y_HEIGHT = HEIGHT,
  parameter int TABLE_DEPTH = 30,
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
  localparam int PAIR_VALUE_BITS = FEATURE_VALUE_WIDTH + 1,
  localparam int PAIR_X_CENTER_BITS = X_BITS + 1,
  localparam int PAIR_Y_CENTER_BITS = Y_BITS + 1,
  localparam int PAIR_X_PRODUCT_BITS = PAIR_X_CENTER_BITS + PAIR_VALUE_BITS + 1,
  localparam int PAIR_Y_PRODUCT_BITS = PAIR_Y_CENTER_BITS + PAIR_VALUE_BITS + 1,
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
  localparam int DATA_WIDTH = (DATA_WIDTH_RAW == 0) ? 1 : DATA_WIDTH_RAW,
  localparam int FEATURE_AREA_REPRESENTABLE_MAX = (1 << AREA_BITS) - 1,
  localparam int FEATURE_AREA_MAX_RES =
      (FEATURE_AREA_MAX < 0) ? FEATURE_AREA_REPRESENTABLE_MAX : FEATURE_AREA_MAX,
  localparam logic MOMENT_X_CENTER_ODD = logic'((WIDTH - 1) & 1),
  localparam logic MOMENT_Y_CENTER_ODD = logic'((FEATURE_Y_HEIGHT - 1) & 1),
  localparam logic [CHILD_WIDTH-1:0] CHILD_ID_NULL =
      CHILD_ID_MODE ? {CHILD_WIDTH{1'b1}} : '0,
  localparam int OFF_MAX_Y = 0,
  localparam int OFF_MIN_Y = OFF_MAX_Y + (FEATURE_BBOX ? Y_BITS : 0),
  localparam int OFF_MAX_X = OFF_MIN_Y + (FEATURE_BBOX ? Y_BITS : 0),
  localparam int OFF_MIN_X = OFF_MAX_X + (FEATURE_BBOX ? X_BITS : 0),
  localparam int OFF_AREA = OFF_MIN_X + (FEATURE_BBOX ? X_BITS : 0),
  localparam int OFF_PRESENT = OFF_AREA + AREA_FIELD_BITS,
  localparam int OFF_ZOM = OFF_PRESENT + PRESENT_FIELD_BITS,
  localparam int OFF_X_FOM = OFF_ZOM + (FEATURE_MOMENTS ? MOMENT_ZOM_BITS : 0),
  localparam int OFF_Y_FOM = OFF_X_FOM + (FEATURE_MOMENTS ? MOMENT_X_BITS : 0),
  localparam int OFF_CHILD = OFF_Y_FOM + (FEATURE_MOMENTS ? MOMENT_Y_BITS : 0),
  localparam int OFF_CLOSE_AREA = 0,
  localparam int OFF_CLOSE_ZOM = OFF_CLOSE_AREA + AREA_FIELD_BITS,
  localparam int OFF_CLOSE_X_FOM = OFF_CLOSE_ZOM + (FEATURE_MOMENTS ? MOMENT_ZOM_BITS : 0),
  localparam int OFF_CLOSE_Y_FOM = OFF_CLOSE_X_FOM + (FEATURE_MOMENTS ? MOMENT_X_BITS : 0),
  localparam int PIPE_WIDTH =
      17 + (2 * ROW_BITS) + PAIR_BITS + (2 * FEATURE_VALUE_WIDTH) + (3 * LABEL_BITS),
  localparam int TOKEN_WIDTH = 4 + (4 * LABEL_BITS) + DATA_WIDTH,
  localparam int REPAIR_WIDTH = 4 + (7 * LABEL_BITS) + (3 * DATA_WIDTH),
  localparam int D1_REPAIR_WIDTH = 3 + (6 * LABEL_BITS),
  localparam int D2_REPAIR_WIDTH = 2 + (3 * LABEL_BITS),
  localparam int S6A_WIDTH = 11 + (11 * LABEL_BITS) + (4 * DATA_WIDTH)
)(
  input  logic clk,
  input  logic rst,
  input  logic [PIPE_WIDTH-1:0] inp,
  input  logic [TOKEN_WIDTH-1:0] token_in,
  input  logic child_add0,
  input  logic child_add1,
  input  logic [CHILD_WIDTH-1:0] child_id0,
  input  logic [CHILD_WIDTH-1:0] child_id1,
  output logic [S6A_WIDTH-1:0] s6a_out,
  output logic [REPAIR_WIDTH-1:0] s6_repair_out,
  output logic [D1_REPAIR_WIDTH-1:0] s6_repair_d1_out,
  output logic [D2_REPAIR_WIDTH-1:0] s6_repair_d2_out,
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
  output logic [S5_CLOSE_MOMENT_WIDTH-1:0] s5_close_moment_data
);
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

  // Feature payload forwarding is resolved in the synchronous RAM read
  // registers. Only the label hazards consumed by S4 and S3 need to remain
  // in the one- and two-cycle delayed repair records.
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
    logic valid;
    logic bank;
    logic [LABEL_BITS-1:0] addr;
    logic [DATA_WIDTH-1:0] data;
  } data_w_t;
  typedef struct packed {
    logic valid;
    logic [LABEL_BITS-1:0] addr;
    logic [DATA_WIDTH-1:0] data;
  } selected_data_w_t;

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

  typedef enum logic [1:0] {
    DATA_REF_LITERAL,
    DATA_REF_BASE_CURRENT,
    DATA_REF_PAIR_CURRENT
  } data_ref_t;

  typedef struct packed {
    data_ref_t state_ret;
    data_ref_t token_ret;
    data_ref_t close_data;
    data_ref_t ret_data;
    logic merge_prev;
  } lane_data_refs_t;

  pipe_t s4b;
  pipe_t s5;
  token_t s4b_token;
  token_t s5_token;
  repair_d1_t s6_repair_d1_reg;
  repair_d2_t s6_repair_d2_reg;
  s6a_t s6a_next;

  logic [DATA_WIDTH-1:0] data_mem0 [0:TABLE_DEPTH-1];
  logic [DATA_WIDTH-1:0] data_mem1 [0:TABLE_DEPTH-1];
  logic [LABEL_BITS-1:0] tail_mem [0:TABLE_DEPTH-1];
  logic [DATA_WIDTH-1:0] data_r_data0;
  logic [DATA_WIDTH-1:0] data_r_data1;
  logic [LABEL_BITS-1:0] tail_r_data;
  logic data_r_en0;
  logic data_r_en1;
  logic [LABEL_BITS-1:0] data_r_addr0;
  logic [LABEL_BITS-1:0] data_r_addr1;
  logic data_w_en0;
  logic data_w_en1;
  logic [LABEL_BITS-1:0] data_w_addr0;
  logic [LABEL_BITS-1:0] data_w_addr1;
  logic [DATA_WIDTH-1:0] data_w_data0;
  logic [DATA_WIDTH-1:0] data_w_data1;
  logic tail_r_en;
  logic [LABEL_BITS-1:0] tail_r_addr;
  logic tail_w_en;
  logic [LABEL_BITS-1:0] tail_w_addr;
  logic [LABEL_BITS-1:0] tail_w_data;
  token_t p_active;
  localparam logic [1:0] S5_DATA_BYPASS_FROM_CLOSE = 2'd0;
  localparam logic [1:0] S5_DATA_BYPASS_FROM_PREV = 2'd1;
  localparam logic [1:0] S5_DATA_BYPASS_FROM_ACTIVE = 2'd2;
  logic s5_token_data_bypass_valid;
  logic [1:0] s5_token_data_bypass_source;
  logic [DATA_WIDTH-1:0] s5_token_data_bypass_data;
  logic [LABEL_BITS-1:0] c_reg;
  logic f_reg;
  logic [LABEL_BITS-1:0] h_reg;
  logic [LABEL_BITS-1:0] t_reg;
  logic [DATA_WIDTH-1:0] d_reg;
  logic [DATA_WIDTH-1:0] s5_token_data;
  logic [LABEL_BITS-1:0] s5_token_ram_tail;
  logic [LABEL_BITS-1:0] s5_token_tail;
  token_t s5_token_with_data;
  state_t s5_state_reg;
  state_t state_start;
  logic [X_BITS-1:0] x0;
  logic [X_BITS-1:0] x1;
  logic s5_sp0;
  logic s5_sp1;
  logic s5_o0;
  logic s5_o1;
  logic s5_ec0;
  logic s5_ec1;
  logic s5_ep0;
  logic s5_ep1;
  token_t p_active_repaired;
  lane_result_t lane0;
  lane_result_t lane1;
  repair_t lane0_token_repair;
  token_t token_for_lane1;
  logic lane0_child_parent_valid;
  logic lane1_child_parent_valid;
  logic [CHILD_WIDTH-1:0] lane0_child_parent_id;
  logic [CHILD_WIDTH-1:0] lane1_child_parent_id;
  logic lane0_child_parent_has_child;
  logic lane1_child_parent_has_child;
  logic tail_valid;
  logic [LABEL_BITS-1:0] tail_addr;
  logic [LABEL_BITS-1:0] tail_data;
  logic head_prev_valid;
  logic [LABEL_BITS-1:0] head_prev_addr;
  logic [LABEL_BITS-1:0] head_prev_head;
  logic [LABEL_BITS-1:0] head_prev_raw_tail;
  logic [DATA_WIDTH-1:0] head_prev_raw_data;
  logic head_prev_close1_hit;
  logic [LABEL_BITS-1:0] head_prev_tail;
  logic [DATA_WIDTH-1:0] head_prev_data;
  data_w_t close_data_w;
  data_w_t data_prev_w;
  selected_data_w_t data_ram0_w;
  selected_data_w_t data_ram1_w;
  repair_t s5_prev_tail_repair;
  logic next_state_valid;
  logic [LABEL_BITS-1:0] next_state_head;
  logic [LABEL_BITS-1:0] next_state_tail;
  logic s5_token_next_mapped;
  logic s5_token_next_unmapped;
  logic s5_token_next_curr_hit;
  logic s5_token_next_prev_hit;
  logic s5_token_next_active_hit;
  logic s5_token_data_bypass_next_valid;
  logic [1:0] s5_token_data_bypass_source_next;
  logic s5_token_next_tail_write_hit;
  logic [LABEL_BITS-1:0] s5_token_tail_bypass_next;
  logic s5_token_tail_bypass_next_valid;
  token_t s5_token_next;
  token_t p_active_next;
  data_ref_t p_active_data_ref;
  data_ref_t s5_token_data_ref;
  data_ref_t token_for_lane1_data_ref;
  lane_data_refs_t lane0_data_refs;
  lane_data_refs_t lane1_data_refs;
  logic fused_use_prev0;
  logic fused_use_prev1;
  logic fused_use_prev;
  logic fused_base_to_pair;
  logic [DATA_WIDTH-1:0] fused_pair_base_data;
  logic [DATA_WIDTH-1:0] fused_prev_data;
  logic [DATA_WIDTH-1:0] fused_pair_data;
  logic [FEATURE_VALUE_WIDTH-1:0] fused_value0;
  logic [FEATURE_VALUE_WIDTH-1:0] fused_value1;
  logic [PAIR_VALUE_BITS-1:0] fused_value_sum;
  logic signed [PAIR_X_CENTER_BITS-1:0] fused_centered_x0;
  logic signed [PAIR_Y_CENTER_BITS-1:0] fused_centered_y;
  logic signed [PAIR_X_PRODUCT_BITS-1:0] fused_x_product;
  logic signed [PAIR_X_PRODUCT_BITS-1:0] fused_x_delta;
  logic signed [PAIR_Y_PRODUCT_BITS-1:0] fused_y_delta;
  logic [AREA_BITS-1:0] fused_area_lane0;
  logic [AREA_BITS-1:0] fused_area_final;
  logic [MOMENT_ZOM_BITS-1:0] fused_base_zom;
  logic [MOMENT_ZOM_BITS-1:0] fused_prev_zom;
  logic [MOMENT_ZOM_BITS-1:0] fused_zom_after_delta;
  logic [MOMENT_ZOM_BITS-1:0] fused_zom_final;
  logic [MOMENT_X_BITS-1:0] fused_prev_x_fom;
  logic [MOMENT_X_BITS-1:0] fused_x_delta_q;
  logic [MOMENT_X_BITS-1:0] fused_x_final;
  logic [MOMENT_Y_BITS-1:0] fused_prev_y_fom;
  logic [MOMENT_Y_BITS-1:0] fused_y_delta_q;
  logic [MOMENT_Y_BITS-1:0] fused_y_final;
  logic fused_three_mass_parity;
  logic fused_x_three_mass_carry;
  logic fused_y_three_mass_carry;

  logic [MOMENT_X_BITS-1:0] dsp_x_merge;
  logic [MOMENT_Y_BITS-1:0] dsp_y_merge;
  S5HighChunkAdd #(.WIDTH(MOMENT_X_BITS), .USE_DSP(USE_DSP_HIGH_FOM)) dsp_x_merge_add (
    .x(feature_x_fom_of(fused_pair_base_data)), .y(fused_prev_x_fom), .result(dsp_x_merge)
  );
  S5HighChunkAdd #(.WIDTH(MOMENT_Y_BITS), .USE_DSP(USE_DSP_HIGH_FOM)) dsp_y_merge_add (
    .x(feature_y_fom_of(fused_pair_base_data)), .y(fused_prev_y_fom), .result(dsp_y_merge)
  );
  logic [MOMENT_ZOM_BITS-1:0] dsp_zom_merge;
  S5HighChunkAdd #(.WIDTH(MOMENT_ZOM_BITS), .USE_DSP(USE_DSP_HIGH_ZOM)) dsp_zom_merge_add (
    .x(fused_base_zom), .y(fused_prev_zom), .result(dsp_zom_merge)
  );

  assign s4b = pipe_t'(inp);
  assign s4b_token = token_t'(token_in);
  assign s6a_out = s6a_next;
  assign s6_repair_out = s5_prev_tail_repair;
  assign s6_repair_d1_out = s6_repair_d1_reg;
  assign s6_repair_d2_out = s6_repair_d2_reg;
  // The S5 pipeline bus is shared across feature configurations; consume the
  // parameter-disabled fields explicitly.
  logic unused_s5_payload;
  assign unused_s5_payload = ^s5;

  function automatic logic [DATA_WIDTH-1:0] pack_feature(
    input logic [X_BITS-1:0] min_x,
    input logic [X_BITS-1:0] max_x,
    input logic [Y_BITS-1:0] min_y,
    input logic [Y_BITS-1:0] max_y,
    input logic [AREA_BITS-1:0] area,
    input logic present,
    input logic [MOMENT_ZOM_BITS-1:0] zom,
    input logic [MOMENT_X_BITS-1:0] x_fom,
    input logic [MOMENT_Y_BITS-1:0] y_fom,
    input logic [CHILD_WIDTH-1:0] child_count
  );
    logic [DATA_WIDTH-1:0] value;
    logic unused_parameterized_pack_fields;
    begin
      unused_parameterized_pack_fields = ^{min_x, max_x, area, zom, x_fom, y_fom};
      value = '0;
      if (FEATURE_BBOX) begin
        value = value | (DATA_WIDTH'(max_y) << OFF_MAX_Y);
        value = value | (DATA_WIDTH'(min_y) << OFF_MIN_Y);
        value = value | (DATA_WIDTH'(max_x) << OFF_MAX_X);
        value = value | (DATA_WIDTH'(min_x) << OFF_MIN_X);
      end
      if (FEATURE_AREA) begin
        value = value | (DATA_WIDTH'(area) << OFF_AREA);
      end
      if (FEATURE_PRESENT) begin
        value = value | (DATA_WIDTH'(present) << OFF_PRESENT);
      end
      if (FEATURE_MOMENTS) begin
        value = value | (DATA_WIDTH'(zom) << OFF_ZOM);
        value = value | (DATA_WIDTH'(x_fom) << OFF_X_FOM);
        value = value | (DATA_WIDTH'(y_fom) << OFF_Y_FOM);
      end
      if (CHILD_COUNT_BITS > 0) begin
        value = value | (DATA_WIDTH'(child_count) << OFF_CHILD);
      end
      pack_feature = value;
    end
  endfunction

  function automatic logic [DATA_WIDTH-1:0] empty_feature;
    begin
      empty_feature = pack_feature(
        {X_BITS{1'b1}},
        '0,
        {Y_BITS{1'b1}},
        '0,
        '0,
        1'b0,
        '0,
        '0,
        '0,
        CHILD_ID_NULL
      );
    end
  endfunction

  function automatic logic [AREA_BITS-1:0] feature_area_of(
    input logic [DATA_WIDTH-1:0] feature
  );
    begin
      if (FEATURE_AREA) begin
        feature_area_of = AREA_BITS'(feature >> OFF_AREA);
      end else begin
        feature_area_of = '0;
      end
    end
  endfunction

  function automatic logic [X_BITS-1:0] feature_min_x_of(
    input logic [DATA_WIDTH-1:0] feature
  );
    begin
      feature_min_x_of = FEATURE_BBOX ? X_BITS'(feature >> OFF_MIN_X) : '0;
    end
  endfunction

  function automatic logic [X_BITS-1:0] feature_max_x_of(
    input logic [DATA_WIDTH-1:0] feature
  );
    begin
      feature_max_x_of = FEATURE_BBOX ? X_BITS'(feature >> OFF_MAX_X) : '0;
    end
  endfunction

  function automatic logic [Y_BITS-1:0] feature_min_y_of(
    input logic [DATA_WIDTH-1:0] feature
  );
    begin
      feature_min_y_of = FEATURE_BBOX ? Y_BITS'(feature >> OFF_MIN_Y) : '0;
    end
  endfunction

  function automatic logic [Y_BITS-1:0] feature_max_y_of(
    input logic [DATA_WIDTH-1:0] feature
  );
    begin
      feature_max_y_of = FEATURE_BBOX ? Y_BITS'(feature >> OFF_MAX_Y) : '0;
    end
  endfunction

  function automatic logic feature_present_of(input logic [DATA_WIDTH-1:0] feature);
    begin
      feature_present_of = FEATURE_PRESENT ? logic'(feature >> OFF_PRESENT) : 1'b0;
    end
  endfunction

  function automatic logic [MOMENT_ZOM_BITS-1:0] feature_zom_of(
    input logic [DATA_WIDTH-1:0] feature
  );
    begin
      feature_zom_of = FEATURE_MOMENTS ? MOMENT_ZOM_BITS'(feature >> OFF_ZOM) : '0;
    end
  endfunction

  function automatic logic [MOMENT_X_BITS-1:0] feature_x_fom_of(
    input logic [DATA_WIDTH-1:0] feature
  );
    begin
      feature_x_fom_of = FEATURE_MOMENTS ? MOMENT_X_BITS'(feature >> OFF_X_FOM) : '0;
    end
  endfunction

  function automatic logic [MOMENT_Y_BITS-1:0] feature_y_fom_of(
    input logic [DATA_WIDTH-1:0] feature
  );
    begin
      feature_y_fom_of = FEATURE_MOMENTS ? MOMENT_Y_BITS'(feature >> OFF_Y_FOM) : '0;
    end
  endfunction

  function automatic logic [MOMENT_X_BITS-1:0] feature_x_moment_delta(
    input logic [X_BITS-1:0] x,
    input logic [FEATURE_VALUE_WIDTH-1:0] value
  );
    logic signed [MOMENT_X_FULL_BITS-1:0] centered_x;
    logic signed [MOMENT_X_FULL_BITS-1:0] value_s;
    logic signed [MOMENT_X_FULL_BITS-1:0] centered_delta;
    begin
      if (CENTERED_MOMENTS) begin
        centered_x =
            $signed(MOMENT_X_FULL_BITS'({x, 1'b0})) -
            $signed(MOMENT_X_FULL_BITS'(WIDTH - 1));
        value_s = $signed(MOMENT_X_FULL_BITS'(value));
        centered_delta = MOMENT_X_FULL_BITS'(centered_x * value_s);
        feature_x_moment_delta = CENTERED_MOMENT_QUOTIENTS ?
            MOMENT_X_BITS'(centered_delta >>> 1) : MOMENT_X_BITS'(centered_delta);
      end else begin
        feature_x_moment_delta = MOMENT_X_BITS'(x) * MOMENT_X_BITS'(value);
      end
    end
  endfunction

  function automatic logic [MOMENT_Y_BITS-1:0] feature_y_moment_delta(
    input logic [Y_BITS-1:0] y,
    input logic [FEATURE_VALUE_WIDTH-1:0] value
  );
    logic signed [MOMENT_Y_FULL_BITS-1:0] centered_y;
    logic signed [MOMENT_Y_FULL_BITS-1:0] value_s;
    logic signed [MOMENT_Y_FULL_BITS-1:0] centered_delta;
    begin
      if (CENTERED_MOMENTS) begin
        centered_y =
            $signed(MOMENT_Y_FULL_BITS'({y, 1'b0})) -
            $signed(MOMENT_Y_FULL_BITS'(FEATURE_Y_HEIGHT - 1));
        value_s = $signed(MOMENT_Y_FULL_BITS'(value));
        centered_delta = MOMENT_Y_FULL_BITS'(centered_y * value_s);
        feature_y_moment_delta = CENTERED_MOMENT_QUOTIENTS ?
            MOMENT_Y_BITS'(centered_delta >>> 1) : MOMENT_Y_BITS'(centered_delta);
      end else begin
        feature_y_moment_delta = MOMENT_Y_BITS'(y) * MOMENT_Y_BITS'(value);
      end
    end
  endfunction

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

  function automatic logic [S5_CLOSE_MOMENT_WIDTH-1:0] pack_close_moment(
    input logic [DATA_WIDTH-1:0] feature
  );
    logic [S5_CLOSE_MOMENT_WIDTH-1:0] value;
    begin
      value = '0;
      if (FEATURE_AREA) begin
        value = value | (S5_CLOSE_MOMENT_WIDTH'(feature_area_of(feature)) << OFF_CLOSE_AREA);
      end
      if (FEATURE_MOMENTS) begin
        value = value | (S5_CLOSE_MOMENT_WIDTH'(feature_zom_of(feature)) << OFF_CLOSE_ZOM);
        value = value | (S5_CLOSE_MOMENT_WIDTH'(feature_x_fom_of(feature)) << OFF_CLOSE_X_FOM);
        value = value | (S5_CLOSE_MOMENT_WIDTH'(feature_y_fom_of(feature)) << OFF_CLOSE_Y_FOM);
      end
      pack_close_moment = value;
    end
  endfunction

  function automatic logic [CHILD_WIDTH-1:0] merged_child_count_from_counts(
    input logic [CHILD_WIDTH-1:0] left_count,
    input logic [CHILD_WIDTH-1:0] right_count
  );
    begin
      if (CHILD_COUNT_BITS == 0) begin
        merged_child_count_from_counts = '0;
      end else if (!CHILD_ID_MODE) begin
        merged_child_count_from_counts = left_count + right_count;
      end else begin
        // In ID mode null is the maximum unsigned code. Minimum already
        // selects the other child when either operand is null, including
        // null+null, so separate null-case payload muxes are redundant.
        merged_child_count_from_counts = (left_count < right_count) ? left_count : right_count;
      end
    end
  endfunction

  function automatic logic [CHILD_WIDTH-1:0] child_count_after_add(
    input logic [CHILD_WIDTH-1:0] current,
    input logic add,
    input logic [CHILD_WIDTH-1:0] child_value
  );
    begin
      if (CHILD_COUNT_BITS == 0) begin
        child_count_after_add = '0;
      end else if (CHILD_ID_MODE) begin
        child_count_after_add = (add & (current == CHILD_ID_NULL)) ? child_value : current;
      end else begin
        child_count_after_add = current + CHILD_WIDTH'(add);
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

  function automatic logic [AREA_BITS-1:0] feature_area_sum(
    input logic [AREA_BITS-1:0] left,
    input logic [AREA_BITS-1:0] right
  );
    logic [AREA_BITS:0] area_sum;
    begin
      area_sum = {1'b0, left} + {1'b0, right};
      if ((AREA_BITS == FULL_AREA_BITS) &&
          (FEATURE_AREA_MAX_RES == FEATURE_AREA_REPRESENTABLE_MAX)) begin
        feature_area_sum = area_sum[AREA_BITS-1:0];
      end else if ((AREA_BITS < FULL_AREA_BITS) &&
                   (FEATURE_AREA_MAX_RES == FEATURE_AREA_REPRESENTABLE_MAX)) begin
        feature_area_sum = area_sum[AREA_BITS] ?
            AREA_BITS'(FEATURE_AREA_REPRESENTABLE_MAX) : area_sum[AREA_BITS-1:0];
      end else begin
        feature_area_sum = (area_sum > (AREA_BITS + 1)'(FEATURE_AREA_MAX_RES)) ?
            AREA_BITS'(FEATURE_AREA_MAX_RES) : area_sum[AREA_BITS-1:0];
      end
    end
  endfunction

  function automatic logic [AREA_BITS-1:0] feature_area_inc(
    input logic [AREA_BITS-1:0] area
  );
    begin
      feature_area_inc = feature_area_sum(area, AREA_BITS'(1));
    end
  endfunction

  function automatic logic same_label(
    input logic [LABEL_BITS-1:0] left,
    input logic [LABEL_BITS-1:0] right
  );
    begin
      same_label = left == right;
    end
  endfunction

  task automatic repair_ordered_token_full(
    output token_t dst,
    input token_t src,
    input repair_t repair
  );
    logic head_hit;
    logic data_hit;
    logic base_mapped;
    logic [LABEL_BITS-1:0] base_head;
    logic [LABEL_BITS-1:0] base_tail;
    logic [DATA_WIDTH-1:0] base_data;
    logic repair_key_mapped;
    logic [LABEL_BITS-1:0] repair_key_head;
    logic close_hit;
    logic [DATA_WIDTH-1:0] close_data;
    logic tail_hit;
    logic [LABEL_BITS-1:0] final_tail;
    begin
      head_hit =
          src.valid & repair.head_prev_valid &
          same_label(src.index, repair.head_prev_addr);
      data_hit =
          src.valid & !src.mapped & repair.data_prev_valid &
          same_label(src.index, repair.data_prev_addr);
      base_mapped = head_hit ? 1'b1 : (data_hit ? 1'b0 : src.mapped);
      base_head = head_hit ? repair.head_prev_head : (data_hit ? '0 : src.head);
      base_tail = head_hit ? repair.head_prev_tail : (data_hit ? '0 : src.tail);
      base_data = head_hit ? repair.head_prev_data :
          (data_hit ? repair.data_prev_data : src.data);
      repair_key_mapped = head_hit ? 1'b1 : src.mapped;
      repair_key_head = head_hit ? repair.head_prev_head : src.head;
      close_hit =
          src.valid & repair_key_mapped & repair.close_valid &
          same_label(repair_key_head, repair.close_addr);
      close_data = close_hit ? repair.close_data : base_data;
      tail_hit =
          src.valid & repair_key_mapped & repair.tail_valid &
          same_label(repair_key_head, repair.tail_addr);
      final_tail = tail_hit ? repair.tail_data : base_tail;
      dst.valid = src.valid;
      dst.index = src.index;
      dst.next = src.next;
      dst.has_next = src.has_next;
      dst.mapped = base_mapped;
      dst.head = base_head;
      dst.tail = final_tail;
      dst.tail_valid =
          tail_hit | (head_hit ? 1'b1 : (data_hit ? 1'b0 : src.tail_valid));
      dst.data = close_data;
    end
  endtask

  function automatic logic [DATA_WIDTH-1:0] materialize_data_ref(
    input data_ref_t data_ref_value,
    input logic [DATA_WIDTH-1:0] literal_data,
    input logic base_to_pair,
    input logic [DATA_WIDTH-1:0] pair_data
  );
    begin
      case (data_ref_value)
        DATA_REF_PAIR_CURRENT: materialize_data_ref = pair_data;
        DATA_REF_BASE_CURRENT: materialize_data_ref =
            base_to_pair ? pair_data : literal_data;
        default: materialize_data_ref = literal_data;
      endcase
    end
  endfunction

  task automatic repair_ordered_token_data_ref(
    output data_ref_t dst_ref,
    input token_t src,
    input data_ref_t src_ref,
    input repair_t repair,
    input data_ref_t head_ref,
    input data_ref_t data_ref,
    input data_ref_t close_ref
  );
    logic head_hit;
    logic data_hit;
    logic repair_key_mapped;
    logic [LABEL_BITS-1:0] repair_key_head;
    logic close_hit;
    data_ref_t base_ref;
    logic unused_repair_fields;
    begin
      unused_repair_fields = ^{src, repair};
      head_hit =
          src.valid & repair.head_prev_valid &
          same_label(src.index, repair.head_prev_addr);
      data_hit =
          src.valid & !src.mapped & repair.data_prev_valid &
          same_label(src.index, repair.data_prev_addr);
      base_ref = head_hit ? head_ref : (data_hit ? data_ref : src_ref);
      repair_key_mapped = head_hit ? 1'b1 : src.mapped;
      repair_key_head = head_hit ? repair.head_prev_head : src.head;
      close_hit =
          src.valid & repair_key_mapped & repair.close_valid &
          same_label(repair_key_head, repair.close_addr);
      dst_ref = close_hit ? close_ref : base_ref;
    end
  endtask

  task automatic s5_lane_transition_fused(
    output lane_result_t result,
    output lane_data_refs_t result_refs,
    output logic [CHILD_WIDTH-1:0] child_parent_id_out,
    output logic child_parent_has_child_out,
    input state_t state_in,
    input data_ref_t state_data_ref_in,
    input token_t lane_token_in,
    input data_ref_t lane_token_data_ref_in,
    input token_t token_override,
    input data_ref_t token_override_data_ref_in,
    input logic sp,
    input logic d,
    input logic o,
    input logic ec,
    input logic ep,
    input logic return_from_start
  );
    state_t lane_state_start;
    token_t token_start;
    state_t state_pix;
    token_t token_pix;
    state_t state_close;
    token_t token_close;
    data_ref_t state_pix_ref;
    data_ref_t token_pix_ref;
    data_ref_t state_close_ref;
    data_ref_t token_close_ref;
    data_ref_t token_start_ref;
    data_ref_t ret_data_ref;
    data_ref_t ret_write_ref;
    logic prev_valid;
    logic need_prev;
    logic ret_valid;
    logic ret_has_next;
    logic ret_mapped;
    logic [LABEL_BITS-1:0] ret_next;
    logic [LABEL_BITS-1:0] ret_head;
    logic [LABEL_BITS-1:0] ret_tail;
    logic [DATA_WIDTH-1:0] ret_data;
    logic [LABEL_BITS-1:0] close_addr;
    logic [LABEL_BITS-1:0] close_tail;
    logic ret_close_hit;
    logic ret_active_hit;
    logic [LABEL_BITS-1:0] ret_write_tail;
    logic [DATA_WIDTH-1:0] ret_write_data;
    begin
      lane_state_start = state_in;
      token_start = sp ? token_override : lane_token_in;
      token_start_ref = sp ? token_override_data_ref_in : lane_token_data_ref_in;
      state_pix = lane_state_start;
      token_pix = token_start;
      state_pix_ref = state_data_ref_in;
      token_pix_ref = token_start_ref;
      prev_valid = token_start.valid;
      need_prev = (!token_start.mapped) || (!lane_state_start.f) ||
          (token_start.head != lane_state_start.h);

      result = '0;
      result_refs = '0;
      result_refs.merge_prev = prev_valid & need_prev;
      child_parent_id_out = '0;
      child_parent_has_child_out = 1'b0;
      // Payloads are only consumed under their unchanged valid signals.
      result.pix_w.next_addr = token_start.tail;
      result.pix_w.next_data = lane_state_start.f ? lane_state_start.h : lane_state_start.c;
      result.pix_w.tail_addr = token_start.mapped ? token_start.head :
          (lane_state_start.f ? lane_state_start.h : lane_state_start.c);
      result.pix_w.tail_data = lane_state_start.f ? lane_state_start.t : lane_state_start.c;

      if (d) begin
        state_pix_ref = DATA_REF_PAIR_CURRENT;
        if (o) begin
          // All five connectivity cases share the same resulting head/tail.
          state_pix.h = token_start.mapped ? token_start.head :
              (lane_state_start.f ? lane_state_start.h : lane_state_start.c);
          state_pix.t = lane_state_start.f ? lane_state_start.t : lane_state_start.c;
          state_pix.f = 1'b1;
          token_pix.mapped = 1'b1;
          token_pix.head = state_pix.h;
          token_pix.tail = state_pix.t;
          token_pix.tail_valid = 1'b1;
          token_pix_ref = DATA_REF_PAIR_CURRENT;
          if (token_start.mapped &&
              (!lane_state_start.f || (token_start.head != lane_state_start.h))) begin
            result.pix_w.next_valid = 1'b1;
          end
          if ((!token_start.mapped && !lane_state_start.f) ||
              (token_start.mapped &&
               (!lane_state_start.f || (token_start.head != lane_state_start.h)))) begin
            result.pix_w.tail_valid = 1'b1;
          end
        end
      end

      state_close = state_pix;
      token_close = token_pix;
      state_close_ref = state_pix_ref;
      token_close_ref = token_pix_ref;
      close_addr = lane_state_start.f ? lane_state_start.h : lane_state_start.c;
      close_tail = lane_state_start.f ? lane_state_start.t : lane_state_start.c;
      result.close_w.valid = 1'b0;
      result.close_w.addr = close_addr;
      result.close_w.tail = close_tail;
      result.close_w.data = lane_state_start.d;
      result_refs.close_data = state_data_ref_in;
      result.close_w.next_valid = 1'b0;
      result.close_w.next_addr = lane_state_start.c;
      result.close_w.next_data = lane_state_start.c;
      result.close_w.head_valid = 1'b0;
      result.close_w.head_addr = lane_state_start.c;
      if (ec) begin
        result.close_w.valid = 1'b1;
        result.close_w.next_valid =
            (!lane_state_start.f) | (lane_state_start.t == lane_state_start.c);
        result.close_w.head_valid = 1'b1;
        state_close.c = state_pix.c + LABEL_BITS'(1);
        state_close.f = 1'b0;
        state_close.h = '0;
        state_close.t = '0;
        state_close.d = empty_feature();
        state_close_ref = DATA_REF_LITERAL;
        if (token_pix.valid && token_pix.mapped && (token_pix.head == close_addr)) begin
          token_close.tail = close_tail;
          token_close.tail_valid = 1'b1;
          token_close.data = lane_state_start.d;
          token_close_ref = state_data_ref_in;
        end
      end

      ret_valid = return_from_start ? token_start.valid : token_close.valid;
      ret_has_next = return_from_start ? token_start.has_next : token_close.has_next;
      ret_mapped = return_from_start ? token_start.mapped : token_close.mapped;
      ret_next = return_from_start ? token_start.next : token_close.next;
      ret_head = return_from_start ? token_start.head : token_close.head;
      ret_tail = return_from_start ? token_start.tail : token_close.tail;
      ret_data = return_from_start ? token_start.data : token_close.data;
      ret_data_ref = return_from_start ? token_start_ref : token_close_ref;
      ret_close_hit =
          result.close_w.valid & ret_valid & ret_mapped &
          same_label(ret_head, result.close_w.addr);
      ret_active_hit =
          ret_valid & ret_mapped & state_pix.f &
          same_label(ret_head, state_pix.h);
      ret_write_tail = ret_close_hit ? result.close_w.tail :
          (ret_active_hit ? state_pix.t : ret_tail);
      ret_write_data = ret_close_hit ? result.close_w.data :
          (ret_active_hit ? state_pix.d : ret_data);
      ret_write_ref = ret_close_hit ? state_data_ref_in :
          (ret_active_hit ? state_pix_ref : ret_data_ref);
      result.ret_w.head_valid = 1'b0;
      result.ret_w.head_addr = ret_next;
      result.ret_w.head_head = ret_head;
      result.ret_w.head_tail = ret_write_tail;
      result.ret_w.head_data = ret_write_data;
      result.ret_w.data_valid = 1'b0;
      result.ret_w.data_addr = ret_next;
      result.ret_w.data_data = ret_write_data;
      result.ret_w.out_valid = 1'b0;
      result.ret_w.out_data = ret_write_data;
      result_refs.ret_data = ret_write_ref;
      if (ep) begin
        if (ret_mapped) begin
          result.ret_w.head_valid = ret_has_next;
        end else begin
          result.ret_w.data_valid = ret_has_next;
          result.ret_w.out_valid = !ret_has_next;
        end
        token_close.valid = 1'b0;
      end
      result.state_ret = state_close;
      result.token_ret = token_close;
      result_refs.state_ret = state_close_ref;
      result_refs.token_ret = token_close_ref;
    end
  endtask

  task automatic s5_lane_transition(
    output lane_result_t result,
    output logic [CHILD_WIDTH-1:0] child_parent_id_out,
    output logic child_parent_has_child_out,
    input logic s5_valid,
    input logic [ROW_BITS-1:0] s5_row,
    input logic [X_BITS-1:0] x,
    input state_t state_in,
    input token_t lane_token_in,
    input token_t token_override,
    input logic sp,
    input logic d,
    input logic o,
    input logic ec,
    input logic ep,
    input logic feature,
    input logic [FEATURE_VALUE_WIDTH-1:0] feature_value,
    input logic [ROW_BITS-1:0] lane_feature_y,
    input logic child_add,
    input logic [CHILD_WIDTH-1:0] child_id,
    input logic return_from_start
  );
    state_t lane_state_start;
    token_t token_start;
    state_t state_pix;
    token_t token_pix;
    state_t state_close;
    token_t token_close;
    state_t state_ret;
    token_t token_ret;
    logic ret_valid;
    logic ret_has_next;
    logic ret_mapped;
    logic [LABEL_BITS-1:0] ret_next;
    logic [LABEL_BITS-1:0] ret_head;
    logic [LABEL_BITS-1:0] ret_tail;
    logic [DATA_WIDTH-1:0] ret_data;
    logic [DATA_WIDTH-1:0] prev_data;
    logic prev_valid;
    logic need_prev;
    logic merge_prev;
    logic child_add_active;
    logic [X_BITS-1:0] prev_min_x;
    logic [X_BITS-1:0] prev_max_x;
    logic [Y_BITS-1:0] prev_min_y;
    logic [AREA_BITS-1:0] prev_area;
    logic prev_present;
    logic [MOMENT_ZOM_BITS-1:0] prev_zom;
    logic [MOMENT_X_BITS-1:0] prev_x_fom;
    logic [MOMENT_Y_BITS-1:0] prev_y_fom;
    logic [CHILD_WIDTH-1:0] prev_count;
    logic [X_BITS-1:0] add_pix_min_x;
    logic [X_BITS-1:0] add_pix_max_x;
    logic [Y_BITS-1:0] add_pix_min_y;
    logic [Y_BITS-1:0] add_pix_max_y;
    logic [AREA_BITS-1:0] add_pix_area;
    logic add_pix_present;
    logic [MOMENT_ZOM_BITS-1:0] add_pix_base_zom;
    logic [MOMENT_ZOM_BITS-1:0] add_pix_zom;
    logic [MOMENT_X_BITS-1:0] add_pix_x_fom;
    logic [MOMENT_Y_BITS-1:0] add_pix_y_fom;
    logic [FEATURE_VALUE_WIDTH-1:0] add_pix_value;
    logic [Y_BITS-1:0] add_pix_y;
    logic add_pix_x_quotient_carry;
    logic add_pix_y_quotient_carry;
    logic [CHILD_WIDTH-1:0] add_pix_count;
    logic [CHILD_WIDTH-1:0] add_pix_child_count;
    logic [X_BITS-1:0] touch_min_x;
    logic [X_BITS-1:0] touch_max_x;
    logic [Y_BITS-1:0] touch_min_y;
    logic [Y_BITS-1:0] touch_max_y;
    logic [AREA_BITS-1:0] touch_area;
    logic touch_present;
    logic [MOMENT_ZOM_BITS-1:0] touch_zom;
    logic [MOMENT_X_BITS-1:0] touch_x_fom;
    logic [MOMENT_Y_BITS-1:0] touch_y_fom;
    logic touch_x_quotient_carry;
    logic touch_y_quotient_carry;
    logic [CHILD_WIDTH-1:0] touch_count;
    logic [CHILD_WIDTH-1:0] new_d_child_count;
    logic [DATA_WIDTH-1:0] add_pix_with_child;
    logic [DATA_WIDTH-1:0] new_d;
    logic [CHILD_WIDTH-1:0] child_merge_id_a_local;
    logic [CHILD_WIDTH-1:0] child_merge_id_b_local;
    logic [LABEL_BITS-1:0] close_addr;
    logic [LABEL_BITS-1:0] close_tail;
    logic ret_close_hit;
    logic ret_active_hit;
    logic [LABEL_BITS-1:0] ret_write_tail;
    logic [DATA_WIDTH-1:0] ret_write_data;
    begin
      lane_state_start = state_in;
      token_start = sp ? token_override : lane_token_in;
      state_pix = lane_state_start;
      token_pix = token_start;
      prev_data = token_start.data;
      prev_valid = token_start.valid;
      need_prev = (!token_start.mapped) || (!lane_state_start.f) ||
          (token_start.head != lane_state_start.h);
      merge_prev = prev_valid & need_prev;
      child_add_active = child_add & s5_valid & d;
      add_pix_y = FEATURE_Y_OVERRIDE ? Y_BITS'(lane_feature_y) : s5_row[Y_BITS-1:0];

      prev_min_x = feature_min_x_of(prev_data);
      prev_max_x = feature_max_x_of(prev_data);
      prev_min_y = feature_min_y_of(prev_data);
      prev_area = feature_area_of(prev_data);
      prev_present = feature_present_of(prev_data);
      prev_zom = feature_zom_of(prev_data);
      prev_x_fom = feature_x_fom_of(prev_data);
      prev_y_fom = feature_y_fom_of(prev_data);
      prev_count = feature_count_of(prev_data);

      add_pix_min_x = feature_min_x_of(lane_state_start.d);
      add_pix_max_x = feature_max_x_of(lane_state_start.d);
      add_pix_min_y = feature_min_y_of(lane_state_start.d);
      add_pix_max_y = feature_max_y_of(lane_state_start.d);
      if (FEATURE_BBOX && feature) begin
        add_pix_min_x = (add_pix_min_x == {X_BITS{1'b1}}) ? x : add_pix_min_x;
        add_pix_max_x = (add_pix_max_x > x) ? add_pix_max_x : x;
        add_pix_min_y = (add_pix_min_y == {Y_BITS{1'b1}}) ?
            add_pix_y : add_pix_min_y;
        add_pix_max_y = add_pix_y;
      end
      add_pix_area = feature_area_sum(
        feature_area_of(lane_state_start.d),
        AREA_BITS'(feature)
      );
      add_pix_present = feature_present_of(lane_state_start.d) | (FEATURE_PRESENT & feature);
      add_pix_value = feature_value & {FEATURE_VALUE_WIDTH{feature}};
      add_pix_base_zom = feature_zom_of(lane_state_start.d);
      add_pix_zom = add_pix_base_zom + MOMENT_ZOM_BITS'(add_pix_value);
      add_pix_x_quotient_carry =
          CENTERED_MOMENT_QUOTIENTS & MOMENT_X_CENTER_ODD &
          add_pix_base_zom[0] & add_pix_value[0];
      add_pix_y_quotient_carry =
          CENTERED_MOMENT_QUOTIENTS & MOMENT_Y_CENTER_ODD &
          add_pix_base_zom[0] & add_pix_value[0];
      add_pix_x_fom =
          feature_x_fom_of(lane_state_start.d) +
          feature_x_moment_delta(x, add_pix_value) +
          MOMENT_X_BITS'(add_pix_x_quotient_carry);
      add_pix_y_fom =
          feature_y_fom_of(lane_state_start.d) +
          feature_y_moment_delta(add_pix_y, add_pix_value) +
          MOMENT_Y_BITS'(add_pix_y_quotient_carry);
      add_pix_count = feature_count_of(lane_state_start.d);
      add_pix_child_count = child_count_after_add(add_pix_count, child_add_active, child_id);
      add_pix_with_child = pack_feature(
        add_pix_min_x,
        add_pix_max_x,
        add_pix_min_y,
        add_pix_max_y,
        add_pix_area,
        add_pix_present,
        add_pix_zom,
        add_pix_x_fom,
        add_pix_y_fom,
        add_pix_child_count
      );

      touch_min_x =
          (merge_prev && (prev_min_x < add_pix_min_x)) ? prev_min_x : add_pix_min_x;
      touch_max_x =
          (merge_prev && (prev_max_x > add_pix_max_x)) ? prev_max_x : add_pix_max_x;
      touch_min_y =
          (merge_prev && (prev_min_y < add_pix_min_y)) ? prev_min_y : add_pix_min_y;
      touch_max_y = add_pix_max_y;
      touch_area = feature_area_sum(
        add_pix_area,
        merge_prev ? prev_area : '0
      );
      touch_present = add_pix_present | (merge_prev & prev_present);
      touch_zom = add_pix_zom + (merge_prev ? prev_zom : '0);
      touch_x_quotient_carry =
          CENTERED_MOMENT_QUOTIENTS & MOMENT_X_CENTER_ODD &
          merge_prev & add_pix_zom[0] & prev_zom[0];
      touch_y_quotient_carry =
          CENTERED_MOMENT_QUOTIENTS & MOMENT_Y_CENTER_ODD &
          merge_prev & add_pix_zom[0] & prev_zom[0];
      touch_x_fom =
          add_pix_x_fom + (merge_prev ? prev_x_fom : '0) +
          MOMENT_X_BITS'(touch_x_quotient_carry);
      touch_y_fom =
          add_pix_y_fom + (merge_prev ? prev_y_fom : '0) +
          MOMENT_Y_BITS'(touch_y_quotient_carry);
      touch_count = merge_prev ?
          merged_child_count_from_counts(add_pix_count, prev_count) :
          add_pix_count;
      new_d_child_count = child_count_after_add(touch_count, child_add_active, child_id);
      new_d = pack_feature(
        touch_min_x,
        touch_max_x,
        touch_min_y,
        touch_max_y,
        touch_area,
        touch_present,
        touch_zom,
        touch_x_fom,
        touch_y_fom,
        new_d_child_count
      );
      child_parent_id_out = touch_count;
      child_parent_has_child_out = child_count_has_child(touch_count);
      child_merge_id_a_local = touch_count;
      child_merge_id_b_local = CHILD_ID_MODE ?
          ((merge_prev && (add_pix_count > prev_count)) ? add_pix_count : prev_count) :
          ((touch_count == add_pix_count) ? prev_count : add_pix_count);

      result = '0;
      // Payloads are only consumed under their unchanged valid signals.
      result.pix_w.next_addr = token_start.tail;
      result.pix_w.next_data = lane_state_start.f ? lane_state_start.h : lane_state_start.c;
      result.pix_w.tail_addr = token_start.mapped ? token_start.head :
          (lane_state_start.f ? lane_state_start.h : lane_state_start.c);
      result.pix_w.tail_data = lane_state_start.f ? lane_state_start.t : lane_state_start.c;
      result.child_merge_id_a = child_merge_id_a_local;
      result.child_merge_id_b = child_merge_id_b_local;

      if (d) begin
        if (o) begin
          if (CHILD_ID_MODE &&
              merge_prev &&
              (add_pix_count != CHILD_ID_NULL) &&
              (prev_count != CHILD_ID_NULL) &&
              (add_pix_count != prev_count)) begin
            result.child_merge_valid = 1'b1;
          end
          // All five connectivity cases share the same resulting head/tail.
          state_pix.h = token_start.mapped ? token_start.head :
              (lane_state_start.f ? lane_state_start.h : lane_state_start.c);
          state_pix.t = lane_state_start.f ? lane_state_start.t : lane_state_start.c;
          state_pix.f = 1'b1;
          token_pix.mapped = 1'b1;
          token_pix.head = state_pix.h;
          token_pix.tail = state_pix.t;
          token_pix.tail_valid = 1'b1;
          state_pix.d = new_d;
          token_pix.data = new_d;
          if (token_start.mapped &&
              (!lane_state_start.f || (token_start.head != lane_state_start.h))) begin
            result.pix_w.next_valid = 1'b1;
          end
          if ((!token_start.mapped && !lane_state_start.f) ||
              (token_start.mapped &&
               (!lane_state_start.f || (token_start.head != lane_state_start.h)))) begin
            result.pix_w.tail_valid = 1'b1;
          end
        end else begin
          state_pix.d = add_pix_with_child;
        end
      end

      state_close = state_pix;
      token_close = token_pix;
      close_addr = lane_state_start.f ? lane_state_start.h : lane_state_start.c;
      close_tail = lane_state_start.f ? lane_state_start.t : lane_state_start.c;
      result.close_w.valid = 1'b0;
      result.close_w.addr = close_addr;
      result.close_w.tail = close_tail;
      result.close_w.data = lane_state_start.d;
      result.close_w.next_valid = 1'b0;
      result.close_w.next_addr = lane_state_start.c;
      result.close_w.next_data = lane_state_start.c;
      result.close_w.head_valid = 1'b0;
      result.close_w.head_addr = lane_state_start.c;
      if (ec) begin
        result.close_w.valid = 1'b1;
        result.close_w.next_valid =
            (!lane_state_start.f) | (lane_state_start.t == lane_state_start.c);
        result.close_w.head_valid = 1'b1;
        state_close.c = state_pix.c + LABEL_BITS'(1);
        state_close.f = 1'b0;
        state_close.h = '0;
        state_close.t = '0;
        state_close.d = empty_feature();
        if (token_pix.valid && token_pix.mapped && (token_pix.head == close_addr)) begin
          token_close.tail = close_tail;
          token_close.tail_valid = 1'b1;
          token_close.data = lane_state_start.d;
        end
      end

      state_ret = state_close;
      token_ret = token_close;
      ret_valid = return_from_start ? token_start.valid : token_close.valid;
      ret_has_next = return_from_start ? token_start.has_next : token_close.has_next;
      ret_mapped = return_from_start ? token_start.mapped : token_close.mapped;
      ret_next = return_from_start ? token_start.next : token_close.next;
      ret_head = return_from_start ? token_start.head : token_close.head;
      ret_tail = return_from_start ? token_start.tail : token_close.tail;
      ret_data = return_from_start ? token_start.data : token_close.data;
      ret_close_hit =
          result.close_w.valid & ret_valid & ret_mapped &
          same_label(ret_head, result.close_w.addr);
      ret_active_hit =
          ret_valid & ret_mapped & state_pix.f &
          same_label(ret_head, state_pix.h);
      ret_write_tail = ret_close_hit ? result.close_w.tail :
          (ret_active_hit ? state_pix.t : ret_tail);
      ret_write_data = ret_close_hit ? result.close_w.data :
          (ret_active_hit ? state_pix.d : ret_data);
      result.ret_w.head_valid = 1'b0;
      result.ret_w.head_addr = ret_next;
      result.ret_w.head_head = ret_head;
      result.ret_w.head_tail = ret_write_tail;
      result.ret_w.head_data = ret_write_data;
      result.ret_w.data_valid = 1'b0;
      result.ret_w.data_addr = ret_next;
      result.ret_w.data_data = ret_write_data;
      result.ret_w.out_valid = 1'b0;
      result.ret_w.out_data = ret_write_data;
      if (ep) begin
        if (ret_mapped) begin
          result.ret_w.head_valid = ret_has_next;
        end else begin
          result.ret_w.data_valid = ret_has_next;
          result.ret_w.out_valid = !ret_has_next;
        end
        token_ret.valid = 1'b0;
      end
      result.state_ret = state_ret;
      result.token_ret = token_ret;
    end
  endtask

  logic [DATA_WIDTH-1:0] s5_token_ram_data;
  logic [DATA_WIDTH-1:0] s5_token_bypass_data;

  assign s5_token_ram_data =
      (s5_token.mapped ? s5.curr_bank : s5.prev_bank) ?
      data_r_data1 : data_r_data0;
  assign s5_token_bypass_data =
      (s5_token_data_bypass_source == S5_DATA_BYPASS_FROM_ACTIVE) ?
      d_reg : s5_token_data_bypass_data;
  assign s5_token_data = s5_token_data_bypass_valid ?
      s5_token_bypass_data : s5_token_ram_data;

  always_comb begin
    s5_token_ram_tail = tail_r_data;
    s5_token_tail = s5_token.tail_valid ? s5_token.tail : s5_token_ram_tail;
    s5_token_with_data = s5_token;
    s5_token_with_data.tail = s5_token_tail;
    s5_token_with_data.tail_valid = s5_token.tail_valid | s5_token.mapped;
    s5_token_with_data.data = s5_token_data;

    s5_state_reg.c = c_reg;
    s5_state_reg.f = f_reg;
    s5_state_reg.h = h_reg;
    s5_state_reg.t = t_reg;
    s5_state_reg.d = d_reg;
    state_start = s5_state_reg;
    x0 = X_BITS'({s5.col, 1'b0});
    x1 = X_BITS'({s5.col, 1'b1});
    s5_sp0 = s5.valid & !s5.a & s5.b0;
    s5_sp1 = s5.valid & !s5.b0 & s5.b1;
    s5_o0 = s5.valid & s5.b0 & s5.d0 & (!s5.a | !s5.c);
    s5_o1 = s5.valid & s5.b1 & s5.d1 & (!s5.b0 | !s5.d0);
    s5_ec0 = s5.valid & s5.c & !s5.d0;
    s5_ec1 = s5.valid & s5.d0 & !s5.d1;
    s5_ep0 = s5.valid & s5.a & !s5.b0;
    s5_ep1 = s5.valid & s5.b0 & !s5.b1;
    p_active_repaired = p_active;
    p_active_data_ref = DATA_REF_LITERAL;
    s5_token_data_ref = DATA_REF_LITERAL;
    token_for_lane1_data_ref = DATA_REF_LITERAL;
    lane0_data_refs = '0;
    lane1_data_refs = '0;
    fused_use_prev0 = 1'b0;
    fused_use_prev1 = 1'b0;
    fused_use_prev = 1'b0;
    fused_base_to_pair = 1'b0;
    fused_pair_base_data = '0;
    fused_prev_data = '0;
    fused_pair_data = '0;
    fused_value0 = '0;
    fused_value1 = '0;
    fused_value_sum = '0;
    fused_centered_x0 = '0;
    fused_centered_y = '0;
    fused_x_product = '0;
    fused_x_delta = '0;
    fused_y_delta = '0;
    fused_area_lane0 = '0;
    fused_area_final = '0;
    fused_base_zom = '0;
    fused_prev_zom = '0;
    fused_zom_after_delta = '0;
    fused_zom_final = '0;
    fused_prev_x_fom = '0;
    fused_x_delta_q = '0;
    fused_x_final = '0;
    fused_prev_y_fom = '0;
    fused_y_delta_q = '0;
    fused_y_final = '0;
    fused_three_mass_parity = 1'b0;
    fused_x_three_mass_carry = 1'b0;
    fused_y_three_mass_carry = 1'b0;

    if (PAIR_FUSED_MOMENTS) begin
      p_active_data_ref =
          p_active.valid & p_active.mapped & state_start.f &
          same_label(p_active.head, state_start.h) ?
          DATA_REF_BASE_CURRENT : DATA_REF_LITERAL;
      s5_token_data_ref =
          s5_token_with_data.valid & s5_token_with_data.mapped & state_start.f &
          same_label(s5_token_with_data.head, state_start.h) ?
          DATA_REF_BASE_CURRENT : DATA_REF_LITERAL;
      s5_lane_transition_fused(
        lane0,
        lane0_data_refs,
        lane0_child_parent_id,
        lane0_child_parent_has_child,
        state_start,
        DATA_REF_BASE_CURRENT,
        p_active_repaired,
        p_active_data_ref,
        s5_token_with_data,
        s5_token_data_ref,
        s5_sp0,
        s5.d0,
        s5_o0,
        s5_ec0,
        s5_ep0,
        1'b1
      );
    end else begin
      s5_lane_transition(
        lane0,
        lane0_child_parent_id,
        lane0_child_parent_has_child,
        s5.valid,
        s5.row,
        x0,
        state_start,
        p_active_repaired,
        s5_token_with_data,
        s5_sp0,
        s5.d0,
        s5_o0,
        s5_ec0,
        s5_ep0,
        s5.feature_d0,
        s5.feature_value_d0,
        s5.feature_y,
        child_add0,
        child_id0,
        1'b1
      );
    end

    lane0_token_repair.head_prev_valid = lane0.ret_w.head_valid;
    lane0_token_repair.head_prev_addr = lane0.ret_w.head_addr;
    lane0_token_repair.head_prev_head = lane0.ret_w.head_head;
    lane0_token_repair.head_prev_tail = lane0.ret_w.head_tail;
    lane0_token_repair.head_prev_data = lane0.ret_w.head_data;
    lane0_token_repair.data_prev_valid = lane0.ret_w.data_valid;
    lane0_token_repair.data_prev_addr = lane0.ret_w.data_addr;
    lane0_token_repair.data_prev_data = lane0.ret_w.data_data;
    lane0_token_repair.close_valid = lane0.close_w.valid;
    lane0_token_repair.close_addr = lane0.close_w.addr;
    lane0_token_repair.close_data = lane0.close_w.data;
    lane0_token_repair.tail_valid = 1'b0;
    lane0_token_repair.tail_addr = '0;
    lane0_token_repair.tail_data = '0;

    repair_ordered_token_full(
      token_for_lane1,
      s5_token_with_data,
      lane0_token_repair
    );
    if (PAIR_FUSED_MOMENTS) begin
      repair_ordered_token_data_ref(
        token_for_lane1_data_ref,
        s5_token_with_data,
        s5_token_data_ref,
        lane0_token_repair,
        lane0_data_refs.ret_data,
        lane0_data_refs.ret_data,
        lane0_data_refs.close_data
      );
      s5_lane_transition_fused(
        lane1,
        lane1_data_refs,
        lane1_child_parent_id,
        lane1_child_parent_has_child,
        lane0.state_ret,
        lane0_data_refs.state_ret,
        lane0.token_ret,
        lane0_data_refs.token_ret,
        token_for_lane1,
        token_for_lane1_data_ref,
        s5_sp1,
        s5.d1,
        s5_o1,
        s5_ec1,
        s5_ep1,
        1'b0
      );
    end else begin
      s5_lane_transition(
        lane1,
        lane1_child_parent_id,
        lane1_child_parent_has_child,
        s5.valid,
        s5.row,
        x1,
        lane0.state_ret,
        lane0.token_ret,
        token_for_lane1,
        s5_sp1,
        s5.d1,
        s5_o1,
        s5_ec1,
        s5_ep1,
        s5.feature_d1,
        s5.feature_value_d1,
        s5.feature_y,
        child_add1,
        child_id1,
        1'b0
      );
    end

    if (PAIR_FUSED_MOMENTS) begin
      fused_use_prev0 = s5_o0 & lane0_data_refs.merge_prev;
      fused_use_prev1 = s5_o1 & lane1_data_refs.merge_prev;
      fused_use_prev = fused_use_prev0 | fused_use_prev1;
      fused_base_to_pair =
          s5.valid & (s5.d0 | (s5.d1 & !s5_ec0));
      fused_pair_base_data = s5_ec0 ? empty_feature() : state_start.d;
      if (fused_use_prev1) begin
        fused_prev_data = s5_sp1 ? token_for_lane1.data : lane0.token_ret.data;
      end else if (fused_use_prev0) begin
        fused_prev_data = s5_sp0 ? s5_token_with_data.data : p_active_repaired.data;
      end

      fused_value0 = s5.d0 ?
          (s5.feature_value_d0 & {FEATURE_VALUE_WIDTH{s5.feature_d0}}) : '0;
      fused_value1 = s5.d1 ?
          (s5.feature_value_d1 & {FEATURE_VALUE_WIDTH{s5.feature_d1}}) : '0;
      fused_value_sum =
          {1'b0, fused_value0} + {1'b0, fused_value1};
      fused_centered_x0 =
          $signed(PAIR_X_CENTER_BITS'({x0, 1'b0})) -
          $signed(PAIR_X_CENTER_BITS'(WIDTH - 1));
      fused_centered_y =
          $signed(PAIR_Y_CENTER_BITS'(Y_BITS'(s5.feature_y) << 1)) -
          $signed(PAIR_Y_CENTER_BITS'(FEATURE_Y_HEIGHT - 1));
      fused_x_product =
          $signed(fused_centered_x0) * $signed({1'b0, fused_value_sum});
      fused_x_delta =
          fused_x_product +
          $signed(PAIR_X_PRODUCT_BITS'({1'b0, fused_value1, 1'b0}));
      fused_y_delta =
          $signed(fused_centered_y) * $signed({1'b0, fused_value_sum});

      fused_area_lane0 = feature_area_of(state_start.d);
      if (s5.d0) begin
        fused_area_lane0 = feature_area_sum(
          fused_area_lane0,
          AREA_BITS'(s5.feature_d0)
        );
        if (fused_use_prev0) begin
          fused_area_lane0 = feature_area_sum(
            fused_area_lane0,
            feature_area_of(fused_prev_data)
          );
        end
      end
      if (s5_ec0) begin
        fused_area_lane0 = feature_area_of(empty_feature());
      end
      fused_area_final = fused_area_lane0;
      if (s5.d1) begin
        fused_area_final = feature_area_sum(
          fused_area_final,
          AREA_BITS'(s5.feature_d1)
        );
        if (fused_use_prev1) begin
          fused_area_final = feature_area_sum(
            fused_area_final,
            feature_area_of(fused_prev_data)
          );
        end
      end

      fused_base_zom = feature_zom_of(fused_pair_base_data);
      fused_prev_zom = fused_use_prev ? feature_zom_of(fused_prev_data) : '0;
      fused_zom_after_delta =
          fused_base_zom + MOMENT_ZOM_BITS'(fused_value_sum);
      if (USE_DSP_HIGH_ZOM)
      fused_zom_final = dsp_zom_merge + MOMENT_ZOM_BITS'(fused_value_sum);
    else
      fused_zom_final = fused_zom_after_delta + fused_prev_zom;
      fused_x_delta_q = MOMENT_X_BITS'($signed(fused_x_delta) >>> 1);
      fused_y_delta_q = MOMENT_Y_BITS'($signed(fused_y_delta) >>> 1);
      fused_prev_x_fom = fused_use_prev ? feature_x_fom_of(fused_prev_data) : '0;
      fused_prev_y_fom = fused_use_prev ? feature_y_fom_of(fused_prev_data) : '0;
      fused_three_mass_parity =
          (fused_base_zom[0] & fused_value_sum[0]) |
          ((fused_base_zom[0] ^ fused_value_sum[0]) & fused_prev_zom[0]);
      fused_x_three_mass_carry =
          CENTERED_MOMENT_QUOTIENTS & MOMENT_X_CENTER_ODD &
          fused_three_mass_parity;
      fused_y_three_mass_carry =
          CENTERED_MOMENT_QUOTIENTS & MOMENT_Y_CENTER_ODD &
          fused_three_mass_parity;
      if (USE_DSP_HIGH_FOM) begin
        fused_x_final = dsp_x_merge + fused_x_delta_q + MOMENT_X_BITS'(fused_x_three_mass_carry);
        fused_y_final = dsp_y_merge + fused_y_delta_q + MOMENT_Y_BITS'(fused_y_three_mass_carry);
      end else begin
      fused_x_final =
          feature_x_fom_of(fused_pair_base_data) + fused_x_delta_q +
          fused_prev_x_fom + MOMENT_X_BITS'(fused_x_three_mass_carry);
      fused_y_final =
          feature_y_fom_of(fused_pair_base_data) + fused_y_delta_q +
          fused_prev_y_fom + MOMENT_Y_BITS'(fused_y_three_mass_carry);
      end
      fused_pair_data = pack_feature(
        '0,
        '0,
        '0,
        '0,
        fused_area_final,
        1'b0,
        fused_zom_final,
        fused_x_final,
        fused_y_final,
        '0
      );

      lane0.state_ret.d = materialize_data_ref(
        lane0_data_refs.state_ret, lane0.state_ret.d,
        fused_base_to_pair, fused_pair_data
      );
      lane0.token_ret.data = materialize_data_ref(
        lane0_data_refs.token_ret, lane0.token_ret.data,
        fused_base_to_pair, fused_pair_data
      );
      lane0.close_w.data = materialize_data_ref(
        lane0_data_refs.close_data, lane0.close_w.data,
        fused_base_to_pair, fused_pair_data
      );
      lane0.ret_w.head_data = materialize_data_ref(
        lane0_data_refs.ret_data, lane0.ret_w.head_data,
        fused_base_to_pair, fused_pair_data
      );
      lane0.ret_w.data_data = materialize_data_ref(
        lane0_data_refs.ret_data, lane0.ret_w.data_data,
        fused_base_to_pair, fused_pair_data
      );
      lane0.ret_w.out_data = materialize_data_ref(
        lane0_data_refs.ret_data, lane0.ret_w.out_data,
        fused_base_to_pair, fused_pair_data
      );
      lane1.state_ret.d = materialize_data_ref(
        lane1_data_refs.state_ret, lane1.state_ret.d,
        fused_base_to_pair, fused_pair_data
      );
      lane1.token_ret.data = materialize_data_ref(
        lane1_data_refs.token_ret, lane1.token_ret.data,
        fused_base_to_pair, fused_pair_data
      );
      lane1.close_w.data = materialize_data_ref(
        lane1_data_refs.close_data, lane1.close_w.data,
        fused_base_to_pair, fused_pair_data
      );
      lane1.ret_w.head_data = materialize_data_ref(
        lane1_data_refs.ret_data, lane1.ret_w.head_data,
        fused_base_to_pair, fused_pair_data
      );
      lane1.ret_w.data_data = materialize_data_ref(
        lane1_data_refs.ret_data, lane1.ret_w.data_data,
        fused_base_to_pair, fused_pair_data
      );
      lane1.ret_w.out_data = materialize_data_ref(
        lane1_data_refs.ret_data, lane1.ret_w.out_data,
        fused_base_to_pair, fused_pair_data
      );
    end

    s5_close_moment_valid =
        s5.valid & (lane0.ret_w.out_valid | lane1.ret_w.out_valid);
    s5_fv_valid = EXPOSE_S5_FV & s5_close_moment_valid;
    s5_fv_lane = lane1.ret_w.out_valid;
    s5_fv_data = lane1.ret_w.out_valid ? lane1.ret_w.out_data : lane0.ret_w.out_data;
    s5_close_moment_lane = s5_fv_lane;
    s5_close_moment_data =
        lane1.ret_w.out_valid ? pack_close_moment(lane1.ret_w.out_data) :
        pack_close_moment(lane0.ret_w.out_data);
    child_merge_valid = lane0.child_merge_valid | lane1.child_merge_valid;
    child_merge_id_a = lane1.child_merge_valid ?
        lane1.child_merge_id_a : lane0.child_merge_id_a;
    child_merge_id_b = lane1.child_merge_valid ?
        lane1.child_merge_id_b : lane0.child_merge_id_b;
    lane0_child_parent_valid = s5.valid & s5.d0;
    lane1_child_parent_valid = s5.valid & s5.d1;
    child_parent0_valid = lane0_child_parent_valid;
    child_parent1_valid = lane1_child_parent_valid;
    child_parent0_id = lane0_child_parent_id;
    child_parent1_id = lane1_child_parent_id;
    child_parent0_has_child = lane0_child_parent_has_child;
    child_parent1_has_child = lane1_child_parent_has_child;

    tail_valid = lane0.pix_w.tail_valid | lane1.pix_w.tail_valid;
    tail_addr = lane1.pix_w.tail_valid ? lane1.pix_w.tail_addr : lane0.pix_w.tail_addr;
    tail_data = lane1.pix_w.tail_valid ? lane1.pix_w.tail_data : lane0.pix_w.tail_data;
    head_prev_valid = lane0.ret_w.head_valid | lane1.ret_w.head_valid;
    head_prev_addr = lane1.ret_w.head_valid ? lane1.ret_w.head_addr : lane0.ret_w.head_addr;
    head_prev_head = lane1.ret_w.head_valid ? lane1.ret_w.head_head : lane0.ret_w.head_head;
    head_prev_raw_tail = lane1.ret_w.head_valid ? lane1.ret_w.head_tail : lane0.ret_w.head_tail;
    head_prev_raw_data = lane1.ret_w.head_valid ? lane1.ret_w.head_data : lane0.ret_w.head_data;
    head_prev_close1_hit =
        head_prev_valid & lane1.close_w.valid & same_label(head_prev_head, lane1.close_w.addr);
    head_prev_tail = head_prev_close1_hit ? lane1.close_w.tail : head_prev_raw_tail;
    head_prev_data = head_prev_close1_hit ? lane1.close_w.data : head_prev_raw_data;

    close_data_w.valid = s5.valid & (lane0.close_w.valid | lane1.close_w.valid);
    close_data_w.bank = s5.curr_bank;
    close_data_w.addr = lane1.close_w.valid ? lane1.close_w.addr : lane0.close_w.addr;
    close_data_w.data = lane1.close_w.valid ? lane1.close_w.data : lane0.close_w.data;
    data_prev_w.valid = s5.valid & (lane0.ret_w.data_valid | lane1.ret_w.data_valid);
    data_prev_w.bank = s5.prev_bank;
    data_prev_w.addr =
        lane1.ret_w.data_valid ? lane1.ret_w.data_addr : lane0.ret_w.data_addr;
    data_prev_w.data =
        lane1.ret_w.data_valid ? lane1.ret_w.data_data : lane0.ret_w.data_data;

    s5_prev_tail_repair.head_prev_valid = s5.valid & head_prev_valid;
    s5_prev_tail_repair.head_prev_addr = head_prev_addr;
    s5_prev_tail_repair.head_prev_head = head_prev_head;
    s5_prev_tail_repair.head_prev_tail = head_prev_tail;
    s5_prev_tail_repair.head_prev_data = head_prev_data;
    s5_prev_tail_repair.data_prev_valid = data_prev_w.valid;
    s5_prev_tail_repair.data_prev_addr = data_prev_w.addr;
    s5_prev_tail_repair.data_prev_data = data_prev_w.data;
    s5_prev_tail_repair.close_valid = close_data_w.valid;
    s5_prev_tail_repair.close_addr = close_data_w.addr;
    s5_prev_tail_repair.close_data = close_data_w.data;
    s5_prev_tail_repair.tail_valid = s5.valid & tail_valid;
    s5_prev_tail_repair.tail_addr = tail_addr;
    s5_prev_tail_repair.tail_data = tail_data;
  end

  always_comb begin
    next_state_valid = s5.valid ? ((!s5.row_end) & lane1.state_ret.f) : s5_state_reg.f;
    next_state_head = (s5.valid & !s5.row_end) ? lane1.state_ret.h : s5_state_reg.h;
    next_state_tail = (s5.valid & !s5.row_end) ? lane1.state_ret.t : s5_state_reg.t;
    s5_token_next_mapped = s4b_token.valid & s4b_token.mapped;
    s5_token_next_unmapped = s4b_token.valid & !s4b_token.mapped;
    s5_token_next_curr_hit =
        s5_token_next_mapped & s5_prev_tail_repair.close_valid &
        (s4b.curr_bank == s5.curr_bank) &
        same_label(s4b_token.head, s5_prev_tail_repair.close_addr);
    s5_token_next_prev_hit =
        s5_token_next_unmapped & s5_prev_tail_repair.data_prev_valid &
        (s4b.prev_bank == s5.prev_bank) &
        same_label(s4b_token.index, s5_prev_tail_repair.data_prev_addr);
    s5_token_next_active_hit =
        s5_token_next_mapped & next_state_valid &
        (s4b.curr_bank == s5.curr_bank) &
        same_label(s4b_token.head, next_state_head);
    s5_token_data_bypass_next_valid =
        s5_token_next_curr_hit | s5_token_next_prev_hit | s5_token_next_active_hit;
    s5_token_data_bypass_source_next =
        s5_token_next_curr_hit ? S5_DATA_BYPASS_FROM_CLOSE :
        (s5_token_next_prev_hit ? S5_DATA_BYPASS_FROM_PREV : S5_DATA_BYPASS_FROM_ACTIVE);
    s5_token_next_tail_write_hit =
        s5_token_next_mapped & s5_prev_tail_repair.tail_valid &
        same_label(s4b_token.head, s5_prev_tail_repair.tail_addr);
    s5_token_tail_bypass_next_valid =
        s5_token_next_tail_write_hit | s5_token_next_active_hit;
    s5_token_tail_bypass_next =
        s5_token_next_tail_write_hit ? s5_prev_tail_repair.tail_data : next_state_tail;
    s5_token_next = s4b_token;
    s5_token_next.tail_valid = s4b_token.tail_valid | s5_token_tail_bypass_next_valid;
    s5_token_next.tail = s4b_token.tail_valid ?
        s4b_token.tail : s5_token_tail_bypass_next;

    tail_r_en = 1'b1;
    tail_r_addr = s4b_token.head;
    tail_w_en = s5_prev_tail_repair.tail_valid;
    tail_w_addr = s5_prev_tail_repair.tail_addr;
    tail_w_data = s5_prev_tail_repair.tail_data;

    data_r_en0 = 1'b1;
    data_r_en1 = 1'b1;
    // Both synchronous banks may read the token address: only the bank selected
    // by the aligned token is consumed. Invalid token read payloads are unused.
    data_r_addr0 = s4b_token.mapped ? s4b_token.head : s4b_token.index;
    data_r_addr1 = s4b_token.mapped ? s4b_token.head : s4b_token.index;
    data_ram0_w.valid =
        (close_data_w.valid & !close_data_w.bank) |
        (data_prev_w.valid & !data_prev_w.bank);
    data_ram0_w.addr =
        (close_data_w.valid & !close_data_w.bank) ? close_data_w.addr : data_prev_w.addr;
    data_ram0_w.data =
        (close_data_w.valid & !close_data_w.bank) ? close_data_w.data : data_prev_w.data;
    data_ram1_w.valid =
        (close_data_w.valid & close_data_w.bank) |
        (data_prev_w.valid & data_prev_w.bank);
    data_ram1_w.addr =
        (close_data_w.valid & close_data_w.bank) ? close_data_w.addr : data_prev_w.addr;
    data_ram1_w.data =
        (close_data_w.valid & close_data_w.bank) ? close_data_w.data : data_prev_w.data;
    data_w_en0 = data_ram0_w.valid;
    data_w_en1 = data_ram1_w.valid;
    data_w_addr0 = data_ram0_w.addr;
    data_w_addr1 = data_ram1_w.addr;
    data_w_data0 = data_ram0_w.data;
    data_w_data1 = data_ram1_w.data;

    p_active_next = lane1.token_ret;

    s6a_next.valid = s5.valid;
    s6a_next.curr_bank = s5.curr_bank;
    s6a_next.prev_bank = s5.prev_bank;
    s6a_next.close_valid = s5_prev_tail_repair.close_valid;
    s6a_next.close_addr = s5_prev_tail_repair.close_addr;
    s6a_next.close_data = s5_prev_tail_repair.close_data;
    s6a_next.close_next_valid = s5.valid & (lane0.close_w.next_valid | lane1.close_w.next_valid);
    s6a_next.close_next_addr =
        lane1.close_w.next_valid ? lane1.close_w.next_addr : lane0.close_w.next_addr;
    s6a_next.head_curr_valid = s5.valid & (lane0.close_w.head_valid | lane1.close_w.head_valid);
    s6a_next.head_curr_addr =
        lane1.close_w.head_valid ? lane1.close_w.head_addr : lane0.close_w.head_addr;
    s6a_next.touch_valid = s5.valid & (lane0.pix_w.next_valid | lane1.pix_w.next_valid);
    s6a_next.touch_addr =
        lane1.pix_w.next_valid ? lane1.pix_w.next_addr : lane0.pix_w.next_addr;
    s6a_next.touch_data =
        lane1.pix_w.next_valid ? lane1.pix_w.next_data : lane0.pix_w.next_data;
    s6a_next.tail_valid = s5_prev_tail_repair.tail_valid;
    s6a_next.tail_addr = s5_prev_tail_repair.tail_addr;
    s6a_next.tail_data = s5_prev_tail_repair.tail_data;
    s6a_next.head_prev_valid = s5_prev_tail_repair.head_prev_valid;
    s6a_next.head_prev_addr = s5_prev_tail_repair.head_prev_addr;
    s6a_next.head_prev_head = s5_prev_tail_repair.head_prev_head;
    s6a_next.head_prev_tail = s5_prev_tail_repair.head_prev_tail;
    s6a_next.head_prev_data = s5_prev_tail_repair.head_prev_data;
    s6a_next.data_prev_valid = s5_prev_tail_repair.data_prev_valid;
    s6a_next.data_prev_addr = s5_prev_tail_repair.data_prev_addr;
    s6a_next.data_prev_data = s5_prev_tail_repair.data_prev_data;
    s6a_next.output_valid = s5.valid & (lane0.ret_w.out_valid | lane1.ret_w.out_valid);
    s6a_next.output_data = lane1.ret_w.out_valid ? lane1.ret_w.out_data : lane0.ret_w.out_data;
  end

  function automatic logic label_addr_in_range(input logic [LABEL_BITS-1:0] addr);
    begin
      label_addr_in_range = ({1'b0, addr} < (LABEL_BITS + 1)'(TABLE_DEPTH));
    end
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      s5 <= '0;
      s5_token <= '0;
      p_active <= '0;
      s5_token_data_bypass_valid <= 1'b0;
      s5_token_data_bypass_source <= S5_DATA_BYPASS_FROM_ACTIVE;
      s5_token_data_bypass_data <= '0;
      s6_repair_d1_reg.head_prev_valid <= 1'b0;
      s6_repair_d1_reg.data_prev_valid <= 1'b0;
      s6_repair_d1_reg.tail_valid <= 1'b0;
      s6_repair_d2_reg.head_prev_valid <= 1'b0;
      s6_repair_d2_reg.data_prev_valid <= 1'b0;
    end else begin
      if (data_r_en0 && label_addr_in_range(data_r_addr0)) begin
        data_r_data0 <= data_mem0[data_r_addr0];
      end
      if (data_r_en1 && label_addr_in_range(data_r_addr1)) begin
        data_r_data1 <= data_mem1[data_r_addr1];
      end
      if (tail_r_en && label_addr_in_range(tail_r_addr)) begin
        tail_r_data <= tail_mem[tail_r_addr];
      end
      if (data_w_en0 && label_addr_in_range(data_w_addr0)) begin
        data_mem0[data_w_addr0] <= data_w_data0;
      end
      if (data_w_en1 && label_addr_in_range(data_w_addr1)) begin
        data_mem1[data_w_addr1] <= data_w_data1;
      end
      if (tail_w_en && label_addr_in_range(tail_w_addr)) begin
        tail_mem[tail_w_addr] <= tail_w_data;
      end

      s5 <= s4b;
      s5_token <= s5_token_next;

      if (s5.valid) begin
        p_active <= p_active_next;
      end
      s5_token_data_bypass_valid <= s5_token_data_bypass_next_valid;
      s5_token_data_bypass_source <= s5_token_data_bypass_source_next;
      // A valid close bypass is always mapped and a valid previous-row bypass
      // is always unmapped.  Active-state bypasses ignore this payload.
      s5_token_data_bypass_data <= s5_token_next_mapped ?
          s5_prev_tail_repair.close_data : s5_prev_tail_repair.data_prev_data;
      s6_repair_d1_reg.head_prev_valid <= s5_prev_tail_repair.head_prev_valid;
      s6_repair_d1_reg.data_prev_valid <= s5_prev_tail_repair.data_prev_valid;
      s6_repair_d1_reg.tail_valid <= s5_prev_tail_repair.tail_valid;
      s6_repair_d2_reg.head_prev_valid <= s6_repair_d1_reg.head_prev_valid;
      s6_repair_d2_reg.data_prev_valid <= s6_repair_d1_reg.data_prev_valid;
    end
  end

  // Keep the row-boundary clear in the register control path. Writing the
  // clear value through the state mux costs LUTs on Efinity, while this form
  // maps to the flip-flops' synchronous clear/set control and preserves the
  // valid-cycle clock enable.
  always_ff @(posedge clk) begin
    if (rst || (s5.valid && s5.row_end)) begin
      c_reg <= '0;
      f_reg <= 1'b0;
      h_reg <= '0;
      t_reg <= '0;
      d_reg <= empty_feature();
    end else if (s5.valid) begin
      c_reg <= lane1.state_ret.c;
      f_reg <= lane1.state_ret.f;
      h_reg <= lane1.state_ret.h;
      t_reg <= lane1.state_ret.t;
      d_reg <= lane1.state_ret.d;
    end
  end

  // Each payload is consumed only under its independently reset validity.
  // Capture payload/source every edge so reset does not enter the wide datapath.
  always_ff @(posedge clk) begin
    s6_repair_d1_reg.head_prev_addr <= s5_prev_tail_repair.head_prev_addr;
    s6_repair_d1_reg.head_prev_head <= s5_prev_tail_repair.head_prev_head;
    s6_repair_d1_reg.head_prev_tail <= s5_prev_tail_repair.head_prev_tail;
    s6_repair_d1_reg.data_prev_addr <= s5_prev_tail_repair.data_prev_addr;
    s6_repair_d1_reg.tail_addr <= s5_prev_tail_repair.tail_addr;
    s6_repair_d1_reg.tail_data <= s5_prev_tail_repair.tail_data;
    s6_repair_d2_reg.head_prev_addr <= s6_repair_d1_reg.head_prev_addr;
    s6_repair_d2_reg.head_prev_head <= s6_repair_d1_reg.head_prev_head;
    s6_repair_d2_reg.data_prev_addr <= s6_repair_d1_reg.data_prev_addr;
  end

  integer s5_init_index;
  initial begin
    if (PAIR_FUSED_MOMENTS &&
        (FEATURE_BBOX || FEATURE_PRESENT || !FEATURE_MOMENTS ||
         !CENTERED_MOMENTS || !COMPRESS_CENTERED_MOMENTS ||
         !FEATURE_Y_OVERRIDE || (CHILD_COUNT_BITS != 0) || CHILD_ID_MODE)) begin
      $error("PAIR_FUSED_MOMENTS is only supported by the centered inner-moment feature shape, with optional area");
    end
    for (s5_init_index = 0; s5_init_index < TABLE_DEPTH; s5_init_index = s5_init_index + 1) begin
      data_mem0[s5_init_index] = empty_feature();
      data_mem1[s5_init_index] = empty_feature();
      tail_mem[s5_init_index] = LABEL_BITS'(s5_init_index);
    end
  end
endmodule
