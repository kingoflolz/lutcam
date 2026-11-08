// Canonical readable SystemVerilog stage for the two-pixel-per-cycle CCA pipeline.

import lrcca_util_pkg::area_width;
import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::moment_x_width;
import lrcca_util_pkg::moment_y_width;
import lrcca_util_pkg::moment_zom_width;

module TwoPPCS4Stage #(
  parameter int ROW_BITS = 6,
  parameter int PAIR_BITS = 5,
  parameter int LABEL_BITS = 5,
  parameter int FEATURE_VALUE_WIDTH = 1,
  parameter int DATA_WIDTH = 34,
  localparam int PIPE_WIDTH =
      17 + (2 * ROW_BITS) + PAIR_BITS + (2 * FEATURE_VALUE_WIDTH) + (3 * LABEL_BITS),
  localparam int TOKEN_WIDTH = 4 + (4 * LABEL_BITS) + DATA_WIDTH,
  localparam int REPAIR_WIDTH = 4 + (7 * LABEL_BITS) + (3 * DATA_WIDTH),
  localparam int D1_REPAIR_WIDTH = 3 + (6 * LABEL_BITS)
)(
  input  logic [PIPE_WIDTH-1:0] inp,
  input  logic [REPAIR_WIDTH-1:0] s6_repair,
  input  logic [D1_REPAIR_WIDTH-1:0] s6_repair_d1,
  output logic [PIPE_WIDTH-1:0] out,
  output logic [TOKEN_WIDTH-1:0] token_out
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

  pipe_t inp_s;
  token_t token_raw;
  token_t token_raw_repaired_d1;
  token_t token_repaired_s6;
  repair_t repair_s6;
  repair_d1_t repair_d1;

  assign inp_s = pipe_t'(inp);
  assign repair_s6 = repair_t'(s6_repair);
  assign repair_d1 = repair_d1_t'(s6_repair_d1);
  assign out = inp_s;
  assign token_out = token_repaired_s6;

  function automatic logic same_label(
    input logic [LABEL_BITS-1:0] left,
    input logic [LABEL_BITS-1:0] right
  );
    begin
      same_label = ~|(left ^ right);
    end
  endfunction

  task automatic repair_ordered_token_prev_tail(
    output token_t dst,
    input token_t src,
    input repair_d1_t repair
  );
    logic head_hit;
    logic data_hit;
    logic base_mapped;
    logic [LABEL_BITS-1:0] base_head;
    logic [LABEL_BITS-1:0] base_tail;
    logic repair_key_mapped;
    logic [LABEL_BITS-1:0] repair_key_head;
    logic tail_hit;
    logic [LABEL_BITS-1:0] final_tail;
    begin
      head_hit =
          src.valid & repair.head_prev_valid & same_label(src.index, repair.head_prev_addr);
      data_hit =
          src.valid & !src.mapped & repair.data_prev_valid &
          same_label(src.index, repair.data_prev_addr);
      base_mapped = head_hit ? 1'b1 : (data_hit ? 1'b0 : src.mapped);
      base_head = head_hit ? repair.head_prev_head : (data_hit ? '0 : src.head);
      base_tail = head_hit ? repair.head_prev_tail : (data_hit ? '0 : src.tail);
      repair_key_mapped = head_hit ? 1'b1 : src.mapped;
      repair_key_head = head_hit ? repair.head_prev_head : src.head;
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
      dst.tail_valid = tail_hit | (head_hit ? 1'b1 : (data_hit ? 1'b0 : src.tail_valid));
      dst.data = src.data;
    end
  endtask

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
          src.valid & repair.head_prev_valid & same_label(src.index, repair.head_prev_addr);
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
      dst.tail_valid = tail_hit | (head_hit ? 1'b1 : (data_hit ? 1'b0 : src.tail_valid));
      dst.data = close_data;
    end
  endtask

  always_comb begin
    token_raw.valid = inp_s.pf_valid;
    token_raw.index = inp_s.pf_index;
    token_raw.next = inp_s.pf_next;
    token_raw.has_next = inp_s.pf_next != inp_s.pf_index;
    token_raw.mapped = inp_s.pf_mapped;
    token_raw.head = inp_s.pf_head;
    token_raw.tail = '0;
    token_raw.tail_valid = 1'b0;
    token_raw.data = '0;
    repair_ordered_token_prev_tail(
      token_raw_repaired_d1,
      token_raw,
      repair_d1
    );
    repair_ordered_token_full(
      token_repaired_s6,
      token_raw_repaired_d1,
      repair_s6
    );
  end

endmodule
