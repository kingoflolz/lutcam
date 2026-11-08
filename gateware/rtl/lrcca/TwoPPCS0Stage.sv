// Canonical readable SystemVerilog implementation of the two-pixel-per-cycle
// linked-run CCA pipeline.

import lrcca_util_pkg::area_width;
import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::moment_x_width;
import lrcca_util_pkg::moment_y_width;
import lrcca_util_pkg::moment_zom_width;

module TwoPPCS0Stage #(
  parameter int WIDTH = 48,
  parameter int HEIGHT = 36,
  parameter bit HOLE_FILL = 1'b0,
  parameter int ROW_BITS = 6,
  parameter int PAIR_BITS = 5,
  parameter int FEATURE_VALUE_WIDTH = 1,
  parameter int PIPE_DRAIN_CYCLES = 10,
  localparam int PAIR_COUNT = WIDTH / 2,
  localparam int DRAIN_BITS = bit_width(longint'(PIPE_DRAIN_CYCLES) + 1),
  localparam int HF_BANK_STRIDE = 1 << PAIR_BITS,
  localparam int S0_WIDTH =
      13 + (2 * ROW_BITS) + PAIR_BITS + (2 * FEATURE_VALUE_WIDTH)
)(
  input  logic clk,
  input  logic rst,
  input  logic datavalid,
  input  logic d0,
  input  logic d1,
  input  logic feature_d0,
  input  logic feature_d1,
  input  logic [FEATURE_VALUE_WIDTH-1:0] feature_value_d0,
  input  logic [FEATURE_VALUE_WIDTH-1:0] feature_value_d1,
  input  logic [ROW_BITS-1:0] feature_y,
  input  logic eol,
  output logic in_ready,
  output logic b0,
  output logic b1,
  output logic [S0_WIDTH-1:0] out
);
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

  logic [1:0] rowbuf_mem [0:PAIR_COUNT-1];
  logic [1:0] rowbuf_r_data;
  logic rowbuf_r_en;
  logic [PAIR_BITS-1:0] rowbuf_r_addr;
  logic rowbuf_w_en;
  logic [PAIR_BITS-1:0] rowbuf_w_addr;
  logic [1:0] rowbuf_w_data;

  logic [3:0] hf_row_mem [0:(2*HF_BANK_STRIDE)-1];
  logic [3:0] hf_row_r_data;
  logic hf_row_r_en;
  logic [PAIR_BITS:0] hf_row_r_addr;
  logic hf_row_w_en;
  logic [PAIR_BITS:0] hf_row_w_addr;
  logic [3:0] hf_row_w_data;

  logic [ROW_BITS-1:0] ingress_row;
  logic [PAIR_BITS-1:0] ingress_col;
  logic [DRAIN_BITS-1:0] drain;
  s0_t s0;
  logic b0_reg;
  logic b1_reg;

  logic accept;
  logic first_row;
  logic last_row;
  logic row_start;
  logic row_end;
  logic row_boundary_wait;
  logic rowbuf_drain_read;
  logic d0_eff;
  logic d1_eff;
  logic feature_d0_eff;
  logic feature_d1_eff;
  logic [FEATURE_VALUE_WIDTH-1:0] feature_value_d0_eff;
  logic [FEATURE_VALUE_WIDTH-1:0] feature_value_d1_eff;
  logic [ROW_BITS-1:0] feature_y_eff;

  logic hf_pending_valid;
  logic hf_flush_pending;
  logic hf_pending_row_start;
  logic hf_pending_row_end;
  logic hf_pending_curr_bank;
  logic hf_pending_prev_bank;
  logic [ROW_BITS-1:0] hf_pending_row;
  logic [PAIR_BITS-1:0] hf_pending_col;
  logic hf_pending_raw_d0;
  logic hf_pending_raw_d1;
  logic hf_pending_feature_d0;
  logic hf_pending_feature_d1;
  logic [FEATURE_VALUE_WIDTH-1:0] hf_pending_feature_value_d0;
  logic [FEATURE_VALUE_WIDTH-1:0] hf_pending_feature_value_d1;
  logic [ROW_BITS-1:0] hf_pending_feature_y;
  logic hf_pending_left_raw;
  logic hf_pending_top_left;
  logic hf_current_left_raw;
  logic hf_prev_filled_d1;
  logic hf_output_valid;
  logic hf_right_raw;
  logic hf_filled_d0;
  logic hf_filled_d1;
  logic [1:0] hf_top_raw;
  logic [1:0] hf_top_filled;

  assign out = s0;

  integer init_index;
  initial begin
    for (init_index = 0; init_index < PAIR_COUNT; init_index = init_index + 1) begin
      rowbuf_mem[init_index] = '0;
      hf_row_mem[init_index] = '0;
      hf_row_mem[HF_BANK_STRIDE + init_index] = '0;
    end
  end

  always_comb begin
    first_row = ingress_row == '0;
    last_row = ingress_row == ROW_BITS'(HEIGHT - 1);
    row_start = ingress_col == '0;
    row_end = eol;
    d0_eff = d0 & !first_row & !last_row & !row_start;
    d1_eff = d1 & !first_row & !last_row & !row_end;
    feature_d0_eff = feature_d0 & !first_row & !last_row & !row_start;
    feature_d1_eff = feature_d1 & !first_row & !last_row & !row_end;
    feature_value_d0_eff = feature_d0_eff ? feature_value_d0 : '0;
    feature_value_d1_eff = feature_d1_eff ? feature_value_d1 : '0;
    feature_y_eff = (feature_d0_eff | feature_d1_eff) ? feature_y : '0;
    rowbuf_drain_read = drain != '0;
    row_boundary_wait = HOLE_FILL ? ((drain != '0) | hf_flush_pending) :
        (drain != '0);
    in_ready = !row_boundary_wait;
    accept = datavalid & in_ready;

    rowbuf_r_en = 1'b0;
    rowbuf_r_addr = '0;
    rowbuf_w_en = 1'b0;
    rowbuf_w_addr = '0;
    rowbuf_w_data = '0;
    hf_row_r_en = 1'b0;
    hf_row_r_addr = '0;
    hf_row_w_en = 1'b0;
    hf_row_w_addr = '0;
    hf_row_w_data = '0;

    hf_output_valid = 1'b0;
    hf_right_raw = 1'b0;
    hf_top_raw = '0;
    hf_top_filled = '0;
    hf_filled_d0 = 1'b0;
    hf_filled_d1 = 1'b0;
    // Line RAM survives a frame abort/reset. Row zero has no predecessor,
    // so suppress its old topology using the row aligned with the RAM read.
    b0 = (s0.row != '0) && rowbuf_r_data[0];
    b1 = (s0.row != '0) && rowbuf_r_data[1];

    if (HOLE_FILL) begin
      hf_output_valid = hf_pending_valid & (accept | hf_flush_pending);
      hf_right_raw = accept ? d0_eff : 1'b0;
      hf_top_raw = (hf_pending_row == '0) ? '0 : hf_row_r_data[1:0];
      hf_top_filled = (hf_pending_row == '0) ? '0 : hf_row_r_data[3:2];
      hf_filled_d0 =
          hf_pending_raw_d0 |
          (hf_top_raw[0] & (hf_pending_left_raw | hf_pending_raw_d1));
      hf_filled_d1 =
          hf_pending_raw_d1 |
          (hf_top_raw[1] & (hf_pending_raw_d0 | hf_right_raw));
      b0 = b0_reg;
      b1 = b1_reg;
      hf_row_r_en = accept;
      hf_row_r_addr = {!ingress_row[0], ingress_col};
      hf_row_w_en = hf_output_valid;
      hf_row_w_addr = {hf_pending_curr_bank, hf_pending_col};
      hf_row_w_data = {hf_filled_d1, hf_filled_d0,
          hf_pending_raw_d1, hf_pending_raw_d0};
    end else begin
      rowbuf_r_en = accept | rowbuf_drain_read;
      rowbuf_r_addr = rowbuf_drain_read ? PAIR_BITS'(PAIR_COUNT - 1) : ingress_col;
      rowbuf_w_en = accept;
      rowbuf_w_addr = ingress_col;
      rowbuf_w_data = {d1_eff, d0_eff};
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      ingress_row <= '0;
      ingress_col <= '0;
      drain <= '0;
      s0 <= '0;
      b0_reg <= 1'b0;
      b1_reg <= 1'b0;
      hf_pending_valid <= 1'b0;
      hf_flush_pending <= 1'b0;
      hf_current_left_raw <= 1'b0;
      hf_prev_filled_d1 <= 1'b0;
    end else begin
      if (rowbuf_r_en) begin
        rowbuf_r_data <= rowbuf_mem[rowbuf_r_addr];
      end
      if (rowbuf_w_en) begin
        rowbuf_mem[rowbuf_w_addr] <= rowbuf_w_data;
      end
      if (hf_row_r_en) hf_row_r_data <= hf_row_mem[hf_row_r_addr];
      if (hf_row_w_en) hf_row_mem[hf_row_w_addr] <= hf_row_w_data;

      if (HOLE_FILL) begin
        if (hf_output_valid) begin
          s0.valid <= 1'b1;
          s0.row_start <= hf_pending_row_start;
          s0.row_end <= hf_pending_row_end;
          s0.curr_bank <= hf_pending_curr_bank;
          s0.prev_bank <= hf_pending_prev_bank;
          s0.row <= hf_pending_row;
          s0.col <= hf_pending_col;
          s0.a <= hf_pending_top_left;
          b0_reg <= hf_top_filled[0];
          b1_reg <= hf_top_filled[1];
          s0.c <= hf_prev_filled_d1;
          s0.d0 <= hf_filled_d0;
          s0.d1 <= hf_filled_d1;
          s0.raw_d0 <= hf_pending_raw_d0;
          s0.raw_d1 <= hf_pending_raw_d1;
          s0.feature_d0 <= hf_pending_feature_d0;
          s0.feature_d1 <= hf_pending_feature_d1;
          s0.feature_value_d0 <= hf_pending_feature_value_d0;
          s0.feature_value_d1 <= hf_pending_feature_value_d1;
          s0.feature_y <= hf_pending_feature_y;
          hf_prev_filled_d1 <= hf_filled_d1;
        end else begin
          s0.valid <= 1'b0;
        end

        if (accept) begin
          hf_pending_valid <= 1'b1;
          hf_pending_row_start <= row_start;
          hf_pending_row_end <= row_end;
          hf_pending_curr_bank <= ingress_row[0];
          hf_pending_prev_bank <= !ingress_row[0];
          hf_pending_row <= ingress_row;
          hf_pending_col <= ingress_col;
          hf_pending_raw_d0 <= d0_eff;
          hf_pending_raw_d1 <= d1_eff;
          hf_pending_feature_d0 <= feature_d0_eff;
          hf_pending_feature_d1 <= feature_d1_eff;
          hf_pending_feature_value_d0 <= feature_value_d0_eff;
          hf_pending_feature_value_d1 <= feature_value_d1_eff;
          hf_pending_feature_y <= feature_y_eff;
          hf_pending_left_raw <= hf_current_left_raw;
          hf_pending_top_left <= (first_row || row_start) ? 1'b0 : hf_row_r_data[3];
          hf_current_left_raw <= d1_eff;
          if (row_end) begin
            hf_flush_pending <= 1'b1;
          end else begin
            ingress_col <= ingress_col + PAIR_BITS'(1);
          end
        end else if (hf_flush_pending) begin
          hf_pending_valid <= 1'b0;
          hf_flush_pending <= 1'b0;
          hf_current_left_raw <= 1'b0;
          hf_prev_filled_d1 <= 1'b0;
          ingress_col <= '0;
          ingress_row <= ingress_row + ROW_BITS'(1);
          drain <= DRAIN_BITS'(PIPE_DRAIN_CYCLES);
        end else if (drain != '0) begin
          drain <= drain - DRAIN_BITS'(1);
        end
      end else begin
        if (accept) begin
          s0.valid <= 1'b1;
          s0.row_start <= row_start;
          s0.row_end <= row_end;
          s0.curr_bank <= ingress_row[0];
          s0.prev_bank <= !ingress_row[0];
          s0.row <= ingress_row;
          s0.col <= ingress_col;
          s0.a <= first_row ? 1'b0 : rowbuf_r_data[1];
          s0.c <= s0.d1;
          s0.d0 <= d0_eff;
          s0.d1 <= d1_eff;
          s0.raw_d0 <= d0_eff;
          s0.raw_d1 <= d1_eff;
          s0.feature_d0 <= feature_d0_eff;
          s0.feature_d1 <= feature_d1_eff;
          s0.feature_value_d0 <= feature_value_d0_eff;
          s0.feature_value_d1 <= feature_value_d1_eff;
          s0.feature_y <= feature_y_eff;
          if (row_end) begin
            ingress_col <= '0;
            ingress_row <= ingress_row + ROW_BITS'(1);
            drain <= DRAIN_BITS'(PIPE_DRAIN_CYCLES);
          end else begin
            ingress_col <= ingress_col + PAIR_BITS'(1);
          end
        end else begin
          s0.valid <= 1'b0;
          if (drain != '0) begin
            drain <= drain - DRAIN_BITS'(1);
          end
        end
      end
    end
  end
endmodule
