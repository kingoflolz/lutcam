// Canonical readable SystemVerilog stage for the two-pixel-per-cycle CCA pipeline.

import lrcca_util_pkg::area_width;
import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::moment_x_width;
import lrcca_util_pkg::moment_y_width;
import lrcca_util_pkg::moment_zom_width;

module TwoPPCS6BStage #(
  parameter int LABEL_BITS = 5,
  parameter int DATA_WIDTH = 34,
  localparam int S6A_WIDTH = 11 + (11 * LABEL_BITS) + (4 * DATA_WIDTH),
  localparam int S6B_WIDTH = 11 + (12 * LABEL_BITS) + (4 * DATA_WIDTH)
)(
  input  logic clk,
  input  logic rst,
  input  logic [S6A_WIDTH-1:0] inp,
  output logic [S6B_WIDTH-1:0] out
);
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

  s6a_t s6a;
  s6b_t s6b;
  logic next0_valid_c;
  logic [LABEL_BITS-1:0] next0_addr_c;
  logic [LABEL_BITS-1:0] next0_data_c;
  logic next1_valid_c;
  logic [LABEL_BITS-1:0] next1_addr_c;
  logic [LABEL_BITS-1:0] next1_data_c;

  assign s6a = s6a_t'(inp);
  assign out = s6b;

  always_comb begin
    next0_valid_c = 1'b0;
    next0_addr_c = '0;
    next0_data_c = '0;
    next1_valid_c = 1'b0;
    next1_addr_c = '0;
    next1_data_c = '0;
    if (s6a.close_next_valid & s6a.touch_valid) begin
      next0_valid_c = s6a.close_next_addr != s6a.touch_addr;
      next0_addr_c = s6a.close_next_addr;
      next0_data_c = s6a.close_next_addr;
      next1_valid_c = 1'b1;
      next1_addr_c = s6a.touch_addr;
      next1_data_c = s6a.touch_data;
    end else if (s6a.close_next_valid) begin
      next0_valid_c = 1'b1;
      next0_addr_c = s6a.close_next_addr;
      next0_data_c = s6a.close_next_addr;
    end else if (s6a.touch_valid) begin
      next0_valid_c = 1'b1;
      next0_addr_c = s6a.touch_addr;
      next0_data_c = s6a.touch_data;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      s6b <= '0;
    end else begin
      s6b.valid <= s6a.valid;
      s6b.curr_bank <= s6a.curr_bank;
      s6b.prev_bank <= s6a.prev_bank;
      s6b.next0_valid <= s6a.valid & next0_valid_c;
      s6b.next0_addr <= next0_addr_c;
      s6b.next0_data <= next0_data_c;
      s6b.next1_valid <= s6a.valid & next1_valid_c;
      s6b.next1_addr <= next1_addr_c;
      s6b.next1_data <= next1_data_c;
      s6b.head_curr_valid <= s6a.valid & s6a.head_curr_valid;
      s6b.head_curr_addr <= s6a.head_curr_addr;
      s6b.head_prev_valid <= s6a.valid & s6a.head_prev_valid;
      s6b.head_prev_addr <= s6a.head_prev_addr;
      s6b.head_prev_head <= s6a.head_prev_head;
      s6b.head_prev_tail <= s6a.head_prev_tail;
      s6b.head_prev_data <= s6a.head_prev_data;
      s6b.data_prev_valid <= s6a.valid & s6a.data_prev_valid;
      s6b.data_prev_addr <= s6a.data_prev_addr;
      s6b.data_prev_data <= s6a.data_prev_data;
      s6b.tail_valid <= s6a.valid & s6a.tail_valid;
      s6b.tail_addr <= s6a.tail_addr;
      s6b.tail_data <= s6a.tail_data;
      s6b.curr_first_valid <= s6a.valid & s6a.close_valid;
      s6b.curr_first_addr <= s6a.close_addr;
      s6b.curr_first_word <= s6a.close_data;
      s6b.output_valid <= s6a.valid & s6a.output_valid;
      s6b.output_data <= s6a.output_data;
    end
  end
endmodule
