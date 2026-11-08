// Fixed 48x36 adapters for the two ordinary TwoPPC configurations.  Both
// instances see the same input stream; `hole_fill` selects the independently
// reset production instance whose ready/output interface is under test.
module Core2Ppc (
  input  logic        clk,
  input  logic        rst,
  input  logic        hole_fill,
  input  logic        datavalid,
  output logic        in_ready,
  input  logic        d0,
  input  logic        d1,
  input  logic        feature_d0,
  input  logic        feature_d1,
  input  logic [5:0]  feature_y,
  input  logic        sof,
  input  logic        eol,
  output logic        fv_valid,
  output logic [34:0] fv_data
);
  logic        plain_ready;
  logic        plain_valid;
  logic [34:0] plain_data;
  logic        filled_ready;
  logic        filled_valid;
  logic [34:0] filled_data;

  TwoPPC #(
    .WIDTH(48),
    .HEIGHT(36),
    .TABLE_DEPTH(30),
    .HOLE_FILL(1'b0)
  ) plain (
    .clk,
    .rst,
    .datavalid,
    .in_ready(plain_ready),
    .d0,
    .d1,
    .feature_d0,
    .feature_d1,
    .feature_value_d0(1'b1),
    .feature_value_d1(1'b1),
    .feature_y,
    .sof,
    .eol,
    .child_add0(1'b0),
    .child_add1(1'b0),
    .child_id0(1'b0),
    .child_id1(1'b0),
    .child_parent0_valid(),
    .child_parent1_valid(),
    .child_parent0_id(),
    .child_parent1_id(),
    .child_parent0_has_child(),
    .child_parent1_has_child(),
    .child_merge_valid(),
    .child_merge_id_a(),
    .child_merge_id_b(),
    .s5_fv_valid(),
    .s5_fv_lane(),
    .s5_fv_data(),
    .s5_close_moment_valid(),
    .s5_close_moment_lane(),
    .s5_close_moment_data(),
    .fv_valid(plain_valid),
    .fv_data(plain_data),
    .fv_child_id(),
    .fv_child_valid()
  );

  TwoPPC #(
    .WIDTH(48),
    .HEIGHT(36),
    .TABLE_DEPTH(30),
    .HOLE_FILL(1'b1)
  ) filled (
    .clk,
    .rst,
    .datavalid,
    .in_ready(filled_ready),
    .d0,
    .d1,
    .feature_d0,
    .feature_d1,
    .feature_value_d0(1'b1),
    .feature_value_d1(1'b1),
    .feature_y,
    .sof,
    .eol,
    .child_add0(1'b0),
    .child_add1(1'b0),
    .child_id0(1'b0),
    .child_id1(1'b0),
    .child_parent0_valid(),
    .child_parent1_valid(),
    .child_parent0_id(),
    .child_parent1_id(),
    .child_parent0_has_child(),
    .child_parent1_has_child(),
    .child_merge_valid(),
    .child_merge_id_a(),
    .child_merge_id_b(),
    .s5_fv_valid(),
    .s5_fv_lane(),
    .s5_fv_data(),
    .s5_close_moment_valid(),
    .s5_close_moment_lane(),
    .s5_close_moment_data(),
    .fv_valid(filled_valid),
    .fv_data(filled_data),
    .fv_child_id(),
    .fv_child_valid()
  );

  always_comb begin
    if (hole_fill) begin
      in_ready = filled_ready;
      fv_valid = filled_valid;
      fv_data = filled_data;
    end else begin
      in_ready = plain_ready;
      fv_valid = plain_valid;
      fv_data = plain_data;
    end
  end
endmodule

// The second supported native-test geometry is only exercised in the
// hole-fill configuration by the ordinary regression suite.
module Core2Ppc160HoleFill (
  input  logic        clk,
  input  logic        rst,
  input  logic        hole_fill,
  input  logic        datavalid,
  output logic        in_ready,
  input  logic        d0,
  input  logic        d1,
  input  logic        feature_d0,
  input  logic        feature_d1,
  input  logic [6:0]  feature_y,
  input  logic        sof,
  input  logic        eol,
  output logic        fv_valid,
  output logic [43:0] fv_data
);
  // Keep the selector in the common Rust driver interface and assert that
  // callers select the configuration represented by this top.
  always_comb begin
    if (!rst) begin
      assert (hole_fill);
    end
  end

  TwoPPC #(
    .WIDTH(160),
    .HEIGHT(80),
    .TABLE_DEPTH(90),
    .HOLE_FILL(1'b1)
  ) dut (
    .clk,
    .rst,
    .datavalid,
    .in_ready,
    .d0,
    .d1,
    .feature_d0,
    .feature_d1,
    .feature_value_d0(1'b1),
    .feature_value_d1(1'b1),
    .feature_y,
    .sof,
    .eol,
    .child_add0(1'b0),
    .child_add1(1'b0),
    .child_id0(1'b0),
    .child_id1(1'b0),
    .child_parent0_valid(),
    .child_parent1_valid(),
    .child_parent0_id(),
    .child_parent1_id(),
    .child_parent0_has_child(),
    .child_parent1_has_child(),
    .child_merge_valid(),
    .child_merge_id_a(),
    .child_merge_id_b(),
    .s5_fv_valid(),
    .s5_fv_lane(),
    .s5_fv_data(),
    .s5_close_moment_valid(),
    .s5_close_moment_lane(),
    .s5_close_moment_data(),
    .fv_valid,
    .fv_data,
    .fv_child_id(),
    .fv_child_valid()
  );
endmodule
