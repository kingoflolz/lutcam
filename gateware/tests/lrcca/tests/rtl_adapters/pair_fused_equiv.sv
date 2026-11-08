// Wiring-only side-by-side adapter for the ordinary and pair-fused S5 moment
// implementations.  Debug outputs are aliases of production state used by the
// original exhaustive equivalence test; no reference behavior lives here.
module PairFusedEquiv (
  input  logic        clk,
  input  logic        rst,
  input  logic        datavalid,
  input  logic        d0,
  input  logic        d1,
  input  logic        feature_d0,
  input  logic        feature_d1,
  input  logic [7:0]  feature_value_d0,
  input  logic [7:0]  feature_value_d1,
  input  logic [2:0]  feature_y,
  input  logic        sof,
  input  logic        eol,

  output logic        ref_ready,
  output logic        fused_ready,
  output logic [1:0]  ref_s5_control,
  output logic [1:0]  fused_s5_control,
  output logic [44:0] ref_s5_data,
  output logic [44:0] fused_s5_data,
  output logic [1:0]  ref_close_control,
  output logic [1:0]  fused_close_control,
  output logic [44:0] ref_close_data,
  output logic [44:0] fused_close_data,
  output logic        ref_fv_valid,
  output logic        fused_fv_valid,
  output logic [44:0] ref_fv_data,
  output logic [44:0] fused_fv_data,
  output logic [1:0]  ref_fv_child,
  output logic [1:0]  fused_fv_child,
  output logic [8:0]  ref_child_debug,
  output logic [8:0]  fused_child_debug,

  output logic [7:0]  ref_predicates,
  output logic [7:0]  fused_predicates,
  output logic [2:0]  ref_s5_shape,
  output logic [2:0]  fused_s5_shape,
  output logic [6:0]  ref_lane_state_meta,
  output logic [6:0]  fused_lane_state_meta,
  output logic [44:0] ref_lane_state_data,
  output logic [44:0] fused_lane_state_data,
  output logic [10:0] ref_write_control,
  output logic [10:0] fused_write_control,
  output logic [44:0] ref_write_data0,
  output logic [44:0] fused_write_data0,
  output logic [44:0] ref_write_data1,
  output logic [44:0] fused_write_data1,
  output logic [6:0]  ref_registered_state_meta,
  output logic [6:0]  fused_registered_state_meta,
  output logic [44:0] ref_registered_state_data,
  output logic [44:0] fused_registered_state_data,
  output logic [11:0] ref_active_meta,
  output logic [11:0] fused_active_meta,
  output logic [44:0] ref_active_data,
  output logic [44:0] fused_active_data,
  output logic [1:0]  fused_use_prev
);
  assign ref_predicates = {
      ref_dut.s5_stage.s5_sp0, ref_dut.s5_stage.s5_sp1,
      ref_dut.s5_stage.s5_o0, ref_dut.s5_stage.s5_o1,
      ref_dut.s5_stage.s5_ec0, ref_dut.s5_stage.s5_ec1,
      ref_dut.s5_stage.s5_ep0, ref_dut.s5_stage.s5_ep1
  };
  assign fused_predicates = {
      fused_dut.s5_stage.s5_sp0, fused_dut.s5_stage.s5_sp1,
      fused_dut.s5_stage.s5_o0, fused_dut.s5_stage.s5_o1,
      fused_dut.s5_stage.s5_ec0, fused_dut.s5_stage.s5_ec1,
      fused_dut.s5_stage.s5_ep0, fused_dut.s5_stage.s5_ep1
  };
  assign ref_s5_shape = {
      ref_dut.s5_stage.s5.valid,
      ref_dut.s5_stage.s5.d0,
      ref_dut.s5_stage.s5.d1
  };
  assign fused_s5_shape = {
      fused_dut.s5_stage.s5.valid,
      fused_dut.s5_stage.s5.d0,
      fused_dut.s5_stage.s5.d1
  };
  assign ref_lane_state_meta = {
      ref_dut.s5_stage.lane1.state_ret.c,
      ref_dut.s5_stage.lane1.state_ret.f,
      ref_dut.s5_stage.lane1.state_ret.h,
      ref_dut.s5_stage.lane1.state_ret.t
  };
  assign fused_lane_state_meta = {
      fused_dut.s5_stage.lane1.state_ret.c,
      fused_dut.s5_stage.lane1.state_ret.f,
      fused_dut.s5_stage.lane1.state_ret.h,
      fused_dut.s5_stage.lane1.state_ret.t
  };
  assign ref_lane_state_data = ref_dut.s5_stage.lane1.state_ret.d;
  assign fused_lane_state_data = fused_dut.s5_stage.lane1.state_ret.d;
  assign ref_write_control = {
      ref_dut.s5_stage.data_w_en0, ref_dut.s5_stage.data_w_addr0,
      ref_dut.s5_stage.data_w_en1, ref_dut.s5_stage.data_w_addr1,
      ref_dut.s5_stage.tail_w_en, ref_dut.s5_stage.tail_w_addr,
      ref_dut.s5_stage.tail_w_data
  };
  assign fused_write_control = {
      fused_dut.s5_stage.data_w_en0, fused_dut.s5_stage.data_w_addr0,
      fused_dut.s5_stage.data_w_en1, fused_dut.s5_stage.data_w_addr1,
      fused_dut.s5_stage.tail_w_en, fused_dut.s5_stage.tail_w_addr,
      fused_dut.s5_stage.tail_w_data
  };
  assign ref_write_data0 = ref_dut.s5_stage.data_w_data0;
  assign fused_write_data0 = fused_dut.s5_stage.data_w_data0;
  assign ref_write_data1 = ref_dut.s5_stage.data_w_data1;
  assign fused_write_data1 = fused_dut.s5_stage.data_w_data1;
  assign ref_registered_state_meta = {
      ref_dut.s5_stage.c_reg, ref_dut.s5_stage.f_reg,
      ref_dut.s5_stage.h_reg, ref_dut.s5_stage.t_reg
  };
  assign fused_registered_state_meta = {
      fused_dut.s5_stage.c_reg, fused_dut.s5_stage.f_reg,
      fused_dut.s5_stage.h_reg, fused_dut.s5_stage.t_reg
  };
  assign ref_registered_state_data = ref_dut.s5_stage.d_reg;
  assign fused_registered_state_data = fused_dut.s5_stage.d_reg;
  assign ref_active_meta = {
      ref_dut.s5_stage.p_active.valid,
      ref_dut.s5_stage.p_active.index,
      ref_dut.s5_stage.p_active.next,
      ref_dut.s5_stage.p_active.has_next,
      ref_dut.s5_stage.p_active.mapped,
      ref_dut.s5_stage.p_active.head,
      ref_dut.s5_stage.p_active.tail,
      ref_dut.s5_stage.p_active.tail_valid
  };
  assign fused_active_meta = {
      fused_dut.s5_stage.p_active.valid,
      fused_dut.s5_stage.p_active.index,
      fused_dut.s5_stage.p_active.next,
      fused_dut.s5_stage.p_active.has_next,
      fused_dut.s5_stage.p_active.mapped,
      fused_dut.s5_stage.p_active.head,
      fused_dut.s5_stage.p_active.tail,
      fused_dut.s5_stage.p_active.tail_valid
  };
  assign ref_active_data = ref_dut.s5_stage.p_active.data;
  assign fused_active_data = fused_dut.s5_stage.p_active.data;
  assign fused_use_prev = {
      fused_dut.s5_stage.fused_use_prev0,
      fused_dut.s5_stage.fused_use_prev1
  };

  TwoPPC #(
    .WIDTH(6),
    .HEIGHT(6),
    .TABLE_DEPTH(4),
    .HOLE_FILL(1'b0),
    .FEATURE_BBOX(1'b0),
    .FEATURE_AREA(1'b1),
    .FEATURE_AREA_BITS(3),
    .FEATURE_PRESENT(1'b0),
    .FEATURE_MOMENTS(1'b1),
    .CENTERED_MOMENTS(1'b1),
    .COMPRESS_CENTERED_MOMENTS(1'b1),
    .PAIR_FUSED_MOMENTS(1'b0),
    .FEATURE_VALUE_WIDTH(8),
    .FEATURE_Y_OVERRIDE(1'b1),
    .FEATURE_Y_HEIGHT(6),
    .CHILD_COUNT_BITS(0),
    .EXPOSE_S5_FV(1'b1)
  ) ref_dut (
    .clk,
    .rst,
    .datavalid,
    .in_ready(ref_ready),
    .d0,
    .d1,
    .feature_d0,
    .feature_d1,
    .feature_value_d0,
    .feature_value_d1,
    .feature_y,
    .sof,
    .eol,
    .child_add0(1'b0),
    .child_add1(1'b0),
    .child_id0(1'b0),
    .child_id1(1'b0),
    .child_parent0_valid(ref_child_debug[8]),
    .child_parent1_valid(ref_child_debug[7]),
    .child_parent0_id(ref_child_debug[6]),
    .child_parent1_id(ref_child_debug[5]),
    .child_parent0_has_child(ref_child_debug[4]),
    .child_parent1_has_child(ref_child_debug[3]),
    .child_merge_valid(ref_child_debug[2]),
    .child_merge_id_a(ref_child_debug[1]),
    .child_merge_id_b(ref_child_debug[0]),
    .s5_fv_valid(ref_s5_control[1]),
    .s5_fv_lane(ref_s5_control[0]),
    .s5_fv_data(ref_s5_data),
    .s5_close_moment_valid(ref_close_control[1]),
    .s5_close_moment_lane(ref_close_control[0]),
    .s5_close_moment_data(ref_close_data),
    .fv_valid(ref_fv_valid),
    .fv_data(ref_fv_data),
    .fv_child_id(ref_fv_child[1]),
    .fv_child_valid(ref_fv_child[0])
  );

  TwoPPC #(
    .WIDTH(6),
    .HEIGHT(6),
    .TABLE_DEPTH(4),
    .HOLE_FILL(1'b0),
    .FEATURE_BBOX(1'b0),
    .FEATURE_AREA(1'b1),
    .FEATURE_AREA_BITS(3),
    .FEATURE_PRESENT(1'b0),
    .FEATURE_MOMENTS(1'b1),
    .CENTERED_MOMENTS(1'b1),
    .COMPRESS_CENTERED_MOMENTS(1'b1),
    .PAIR_FUSED_MOMENTS(1'b1),
    .FEATURE_VALUE_WIDTH(8),
    .FEATURE_Y_OVERRIDE(1'b1),
    .FEATURE_Y_HEIGHT(6),
    .CHILD_COUNT_BITS(0),
    .EXPOSE_S5_FV(1'b1)
  ) fused_dut (
    .clk,
    .rst,
    .datavalid,
    .in_ready(fused_ready),
    .d0,
    .d1,
    .feature_d0,
    .feature_d1,
    .feature_value_d0,
    .feature_value_d1,
    .feature_y,
    .sof,
    .eol,
    .child_add0(1'b0),
    .child_add1(1'b0),
    .child_id0(1'b0),
    .child_id1(1'b0),
    .child_parent0_valid(fused_child_debug[8]),
    .child_parent1_valid(fused_child_debug[7]),
    .child_parent0_id(fused_child_debug[6]),
    .child_parent1_id(fused_child_debug[5]),
    .child_parent0_has_child(fused_child_debug[4]),
    .child_parent1_has_child(fused_child_debug[3]),
    .child_merge_valid(fused_child_debug[2]),
    .child_merge_id_a(fused_child_debug[1]),
    .child_merge_id_b(fused_child_debug[0]),
    .s5_fv_valid(fused_s5_control[1]),
    .s5_fv_lane(fused_s5_control[0]),
    .s5_fv_data(fused_s5_data),
    .s5_close_moment_valid(fused_close_control[1]),
    .s5_close_moment_lane(fused_close_control[0]),
    .s5_close_moment_data(fused_close_data),
    .fv_valid(fused_fv_valid),
    .fv_data(fused_fv_data),
    .fv_child_id(fused_fv_child[1]),
    .fv_child_valid(fused_fv_child[0])
  );
endmodule
