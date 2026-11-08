// Canonical readable SystemVerilog stage for the two-pixel-per-cycle CCA pipeline.

import lrcca_util_pkg::area_width;
import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::moment_x_width;
import lrcca_util_pkg::moment_y_width;
import lrcca_util_pkg::moment_zom_width;

module TwoPPCS2Stage #(
  parameter int ROW_BITS = 6,
  parameter int PAIR_BITS = 5,
  parameter int LABEL_BITS = 5,
  parameter int FEATURE_VALUE_WIDTH = 1
)(
  input  logic clk,
  input  logic rst,
  input  logic inp_valid,
  input  logic inp_row_start,
  input  logic inp_row_end,
  input  logic inp_curr_bank,
  input  logic inp_prev_bank,
  input  logic [ROW_BITS-1:0] inp_row,
  input  logic [PAIR_BITS-1:0] inp_col,
  input  logic inp_d0,
  input  logic inp_d1,
  input  logic inp_raw_d0,
  input  logic inp_raw_d1,
  input  logic inp_feature_d0,
  input  logic inp_feature_d1,
  input  logic [FEATURE_VALUE_WIDTH-1:0] inp_feature_value_d0,
  input  logic [FEATURE_VALUE_WIDTH-1:0] inp_feature_value_d1,
  input  logic [ROW_BITS-1:0] inp_feature_y,
  input  logic inp_a,
  input  logic inp_b0,
  input  logic inp_b1,
  input  logic inp_c,
  input  logic inp_pf_valid,
  input  logic [LABEL_BITS-1:0] inp_pf_index,
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
  // Payload is overwritten with its own valid transaction. Only the
  // independent stream and prefetch validity authorities need reset.
  always_ff @(posedge clk) begin
    out_valid <= inp_valid;
    out_row_start <= inp_row_start;
    out_row_end <= inp_row_end;
    out_curr_bank <= inp_curr_bank;
    out_prev_bank <= inp_prev_bank;
    out_row <= inp_row;
    out_col <= inp_col;
    out_d0 <= inp_d0;
    out_d1 <= inp_d1;
    out_raw_d0 <= inp_raw_d0;
    out_raw_d1 <= inp_raw_d1;
    out_feature_d0 <= inp_feature_d0;
    out_feature_d1 <= inp_feature_d1;
    out_feature_value_d0 <= inp_feature_value_d0;
    out_feature_value_d1 <= inp_feature_value_d1;
    out_feature_y <= inp_feature_y;
    out_a <= inp_valid & inp_a;
    out_b0 <= inp_valid & inp_b0;
    out_b1 <= inp_valid & inp_b1;
    out_c <= inp_valid & inp_c;
    out_pf_valid <= inp_pf_valid;
    out_pf_index <= inp_pf_index;
    out_pf_next <= '0;
    out_pf_mapped <= 1'b0;
    out_pf_head <= '0;
    if (rst) begin
      out_valid <= 1'b0;
      out_pf_valid <= 1'b0;
    end
  end

endmodule
