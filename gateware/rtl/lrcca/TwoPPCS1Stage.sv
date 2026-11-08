// Canonical readable SystemVerilog stage for the two-pixel-per-cycle CCA pipeline.

import lrcca_util_pkg::area_width;
import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::moment_x_width;
import lrcca_util_pkg::moment_y_width;
import lrcca_util_pkg::moment_zom_width;

module TwoPPCS1Stage #(
  parameter int ROW_BITS = 6,
  parameter int PAIR_BITS = 5,
  parameter int LABEL_BITS = 5,
  parameter int FEATURE_VALUE_WIDTH = 1
)(
  input  logic clk,
  input  logic rst,
  input  logic s0_valid,
  input  logic s0_row_start,
  input  logic s0_row_end,
  input  logic s0_curr_bank,
  input  logic s0_prev_bank,
  input  logic [ROW_BITS-1:0] s0_row,
  input  logic [PAIR_BITS-1:0] s0_col,
  input  logic s0_a,
  input  logic s0_c,
  input  logic s0_d0,
  input  logic s0_d1,
  input  logic s0_raw_d0,
  input  logic s0_raw_d1,
  input  logic s0_feature_d0,
  input  logic s0_feature_d1,
  input  logic [FEATURE_VALUE_WIDTH-1:0] s0_feature_value_d0,
  input  logic [FEATURE_VALUE_WIDTH-1:0] s0_feature_value_d1,
  input  logic [ROW_BITS-1:0] s0_feature_y,
  input  logic b0,
  input  logic b1,
  output logic out_valid,
  output logic out_row_start,
  output logic out_row_end,
  output logic out_curr_bank,
  output logic out_prev_bank,
  output logic [ROW_BITS-1:0] out_row,
  output logic [PAIR_BITS-1:0] out_col,
  output logic out_d0,
  output logic out_d1,
  output logic out_raw_d0,
  output logic out_raw_d1,
  output logic out_feature_d0,
  output logic out_feature_d1,
  output logic [FEATURE_VALUE_WIDTH-1:0] out_feature_value_d0,
  output logic [FEATURE_VALUE_WIDTH-1:0] out_feature_value_d1,
  output logic [ROW_BITS-1:0] out_feature_y,
  output logic out_a,
  output logic out_b0,
  output logic out_b1,
  output logic out_c,
  output logic out_pf_valid,
  output logic [LABEL_BITS-1:0] out_pf_index,
  output logic [LABEL_BITS-1:0] out_pf_next,
  output logic out_pf_mapped,
  output logic [LABEL_BITS-1:0] out_pf_head
);
  logic [LABEL_BITS-1:0] p_issue;
  logic s1_sp;

  always_comb begin
    s1_sp = (!s0_a & b0) | (!b0 & b1);
    out_valid = s0_valid;
    out_row_start = s0_row_start;
    out_row_end = s0_row_end;
    out_curr_bank = s0_curr_bank;
    out_prev_bank = s0_prev_bank;
    out_row = s0_row;
    out_col = s0_col;
    out_d0 = s0_d0;
    out_d1 = s0_d1;
    out_raw_d0 = s0_raw_d0;
    out_raw_d1 = s0_raw_d1;
    out_feature_d0 = s0_feature_d0;
    out_feature_d1 = s0_feature_d1;
    out_feature_value_d0 = s0_feature_value_d0;
    out_feature_value_d1 = s0_feature_value_d1;
    out_feature_y = s0_feature_y;
    out_a = s0_a;
    out_b0 = b0;
    out_b1 = b1;
    out_c = s0_c;
    out_pf_valid = s0_valid & s1_sp;
    out_pf_index = p_issue;
    out_pf_next = '0;
    out_pf_mapped = 1'b0;
    out_pf_head = '0;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      p_issue <= '0;
    end else if (out_valid) begin
      if (out_row_end) begin
        p_issue <= '0;
      end else if (out_pf_valid) begin
        p_issue <= p_issue + LABEL_BITS'(1);
      end
    end
  end
endmodule
