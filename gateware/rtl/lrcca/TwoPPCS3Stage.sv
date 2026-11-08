// Canonical readable SystemVerilog stage for the two-pixel-per-cycle CCA pipeline.

import lrcca_util_pkg::area_width;
import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::moment_x_width;
import lrcca_util_pkg::moment_y_width;
import lrcca_util_pkg::moment_zom_width;

module TwoPPCS3Stage #(
  parameter int ROW_BITS = 6,
  parameter int PAIR_BITS = 5,
  parameter int LABEL_BITS = 5,
  parameter int FEATURE_VALUE_WIDTH = 1,
  parameter int DATA_WIDTH = 34,
  parameter int TABLE_DEPTH = 30,
  localparam int NEXT_MEMORY_DEPTH = 1 << LABEL_BITS,
  localparam bit COMPACT_HEAD_WORD = TABLE_DEPTH < NEXT_MEMORY_DEPTH,
  localparam int HEAD_WORD_WIDTH =
      LABEL_BITS + (COMPACT_HEAD_WORD ? 0 : 1),
  localparam int PIPE_WIDTH =
      17 + (2 * ROW_BITS) + PAIR_BITS + (2 * FEATURE_VALUE_WIDTH) + (3 * LABEL_BITS),
  localparam int REPAIR_WIDTH = 4 + (7 * LABEL_BITS) + (3 * DATA_WIDTH),
  localparam int D1_REPAIR_WIDTH = 3 + (6 * LABEL_BITS),
  localparam int D2_REPAIR_WIDTH = 2 + (3 * LABEL_BITS),
  localparam int S6B_WIDTH = 11 + (12 * LABEL_BITS) + (4 * DATA_WIDTH)
)(
  input  logic clk,
  input  logic rst,
  input  logic [PIPE_WIDTH-1:0] inp,
  input  logic [PIPE_WIDTH-1:0] s1_in,
  input  logic [S6B_WIDTH-1:0] s6b_in,
  input  logic [REPAIR_WIDTH-1:0] s6_repair_in,
  input  logic [D1_REPAIR_WIDTH-1:0] s6_repair_d1_in,
  input  logic [D2_REPAIR_WIDTH-1:0] s6_repair_d2_in,
  output logic [PIPE_WIDTH-1:0] out
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

  pipe_t s1;
  pipe_t s2;
  pipe_t out_reg;
  s6b_t s6b;
  repair_t s6_repair;
  repair_d1_t s6_repair_d1;
  repair_d2_t s6_repair_d2;

  // When TABLE_DEPTH leaves one label code unused, all ones is an unambiguous
  // unmapped sentinel and the RAM word needs only LABEL_BITS.  A full
  // power-of-two table has no spare code, so retain the explicit mapped bit.
  localparam logic [LABEL_BITS-1:0] UNMAPPED_HEAD = {LABEL_BITS{1'b1}};

  // Keep the logical label limit at TABLE_DEPTH, but give Efinity a native
  // power-of-two shape for these dual-write-port memories. Every access is
  // still guarded by label_addr_in_range().
  logic [LABEL_BITS-1:0] next_mem0 [0:NEXT_MEMORY_DEPTH-1];
  logic [LABEL_BITS-1:0] next_mem1 [0:NEXT_MEMORY_DEPTH-1];
  logic [HEAD_WORD_WIDTH-1:0] head_mem0 [0:TABLE_DEPTH-1];
  logic [HEAD_WORD_WIDTH-1:0] head_mem1 [0:TABLE_DEPTH-1];
  logic [LABEL_BITS-1:0] next_r_data0;
  logic [LABEL_BITS-1:0] next_r_data1;
  logic [HEAD_WORD_WIDTH-1:0] head_r_data0;
  logic [HEAD_WORD_WIDTH-1:0] head_r_data1;
  logic [LABEL_BITS-1:0] next_port_a_addr0;
  logic [LABEL_BITS-1:0] next_port_a_addr1;
  logic [LABEL_BITS-1:0] next_port_b_addr0;
  logic [LABEL_BITS-1:0] next_port_b_addr1;
  logic next0_en0;
  logic next0_en1;
  logic next1_en0;
  logic next1_en1;
  logic head_curr_en0;
  logic head_curr_en1;
  logic head_prev_en0;
  logic head_prev_en1;
  logic [LABEL_BITS-1:0] pf0_next;
  logic [HEAD_WORD_WIDTH-1:0] pf0_head_word_raw;
  logic [HEAD_WORD_WIDTH-1:0] pf0_head_word;
  logic pf0_head_repair_hit;
  logic pf0_data_repair_hit;
  logic pf0_head_repair_d1_hit;
  logic pf0_data_repair_d1_hit;
  logic pf0_head_repair_d2_hit;
  logic pf0_data_repair_d2_hit;
  logic pf0_data_repair_selected;
  logic pf0_head_repair_d1_selected;
  logic pf0_data_repair_d1_selected;
  logic pf0_head_repair_d2_selected;
  logic pf0_data_repair_d2_selected;
  logic pf0_data_head_selected;

  assign s1 = pipe_t'(s1_in);
  assign s2 = pipe_t'(inp);
  assign s6b = s6b_t'(s6b_in);
  assign s6_repair = repair_t'(s6_repair_in);
  assign s6_repair_d1 = repair_d1_t'(s6_repair_d1_in);
  assign s6_repair_d2 = repair_d2_t'(s6_repair_d2_in);
  assign out = out_reg;
  // These packed stage interfaces share one canonical layout; this stage uses
  // only its lookup and repair fields.
  logic unused_stage_payload;
  assign unused_stage_payload = ^{s1, s6b, s6_repair, s6_repair_d1, s6_repair_d2};

  function automatic logic same_label(
    input logic [LABEL_BITS-1:0] left,
    input logic [LABEL_BITS-1:0] right
  );
    begin
      same_label = ~|(left ^ right);
    end
  endfunction

  function automatic logic [HEAD_WORD_WIDTH-1:0] head_word(
    input logic [LABEL_BITS-1:0] head,
    input logic mapped
  );
    begin
      if (COMPACT_HEAD_WORD) begin
        head_word = mapped ? HEAD_WORD_WIDTH'(head) :
            HEAD_WORD_WIDTH'(UNMAPPED_HEAD);
      end else begin
        head_word = HEAD_WORD_WIDTH'({mapped, head});
      end
    end
  endfunction

  function automatic logic head_mapped(input logic [HEAD_WORD_WIDTH-1:0] word);
    begin
      if (COMPACT_HEAD_WORD)
        head_mapped = word != HEAD_WORD_WIDTH'(UNMAPPED_HEAD);
      else
        head_mapped = word[HEAD_WORD_WIDTH-1];
    end
  endfunction

  function automatic logic [LABEL_BITS-1:0] head_value(
    input logic [LABEL_BITS-1:0] word
  );
    begin
      head_value = word[LABEL_BITS-1:0];
    end
  endfunction

  function automatic logic [LABEL_BITS-1:0] decoded_head(
    input logic [HEAD_WORD_WIDTH-1:0] word,
    input logic [LABEL_BITS-1:0] unmapped_index
  );
    begin
      decoded_head = head_mapped(word) ?
          head_value(word[LABEL_BITS-1:0]) : unmapped_index;
    end
  endfunction

  integer s3_init_index;
  initial begin
    for (s3_init_index = 0; s3_init_index < TABLE_DEPTH; s3_init_index = s3_init_index + 1) begin
      next_mem0[s3_init_index] = LABEL_BITS'(s3_init_index);
      next_mem1[s3_init_index] = LABEL_BITS'(s3_init_index);
      head_mem0[s3_init_index] = head_word(LABEL_BITS'(s3_init_index), 1'b0);
      head_mem1[s3_init_index] = head_word(LABEL_BITS'(s3_init_index), 1'b0);
    end
  end

  function automatic logic label_addr_in_range(input logic [LABEL_BITS-1:0] addr);
    begin
      label_addr_in_range = ({1'b0, addr} < (LABEL_BITS + 1)'(TABLE_DEPTH));
    end
  endfunction

  always_comb begin
    next0_en0 = s6b.valid & !s6b.curr_bank & s6b.next0_valid;
    next0_en1 = s6b.valid & s6b.curr_bank & s6b.next0_valid;
    next1_en0 = s6b.valid & !s6b.curr_bank & s6b.next1_valid;
    next1_en1 = s6b.valid & s6b.curr_bank & s6b.next1_valid;
    head_curr_en0 = s6b.valid & s6b.head_curr_valid & !s6b.curr_bank;
    head_curr_en1 = s6b.valid & s6b.head_curr_valid & s6b.curr_bank;
    head_prev_en0 = s6b.valid & s6b.head_prev_valid & !s6b.prev_bank;
    head_prev_en1 = s6b.valid & s6b.head_prev_valid & s6b.prev_bank;
    next_port_a_addr0 = next0_en0 ? s6b.next0_addr : s1.pf_index;
    next_port_a_addr1 = next0_en1 ? s6b.next0_addr : s1.pf_index;
    next_port_b_addr0 = s6b.next1_addr;
    next_port_b_addr1 = s6b.next1_addr;

    pf0_next = s2.prev_bank ? next_r_data1 : next_r_data0;
    pf0_head_word_raw = s2.prev_bank ? head_r_data1 : head_r_data0;
    pf0_head_repair_hit =
        s2.pf_valid & s6_repair.head_prev_valid &
        same_label(s2.pf_index, s6_repair.head_prev_addr);
    pf0_data_repair_hit =
        s2.pf_valid & s6_repair.data_prev_valid &
        same_label(s2.pf_index, s6_repair.data_prev_addr);
    pf0_head_repair_d1_hit =
        s2.pf_valid & s6_repair_d1.head_prev_valid &
        same_label(s2.pf_index, s6_repair_d1.head_prev_addr);
    pf0_data_repair_d1_hit =
        s2.pf_valid & s6_repair_d1.data_prev_valid &
        same_label(s2.pf_index, s6_repair_d1.data_prev_addr);
    pf0_head_repair_d2_hit =
        s2.pf_valid & s6_repair_d2.head_prev_valid &
        same_label(s2.pf_index, s6_repair_d2.head_prev_addr);
    pf0_data_repair_d2_hit =
        s2.pf_valid & s6_repair_d2.data_prev_valid &
        same_label(s2.pf_index, s6_repair_d2.data_prev_addr);
    pf0_data_repair_selected = pf0_data_repair_hit & !pf0_head_repair_hit;
    pf0_head_repair_d1_selected =
        pf0_head_repair_d1_hit & !pf0_head_repair_hit & !pf0_data_repair_hit;
    pf0_data_repair_d1_selected =
        pf0_data_repair_d1_hit & !pf0_head_repair_hit & !pf0_data_repair_hit &
        !pf0_head_repair_d1_hit;
    pf0_head_repair_d2_selected =
        pf0_head_repair_d2_hit & !pf0_head_repair_hit & !pf0_data_repair_hit &
        !pf0_head_repair_d1_hit & !pf0_data_repair_d1_hit;
    pf0_data_repair_d2_selected =
        pf0_data_repair_d2_hit & !pf0_head_repair_hit & !pf0_data_repair_hit &
        !pf0_head_repair_d1_hit & !pf0_data_repair_d1_hit &
        !pf0_head_repair_d2_hit;
    pf0_data_head_selected =
        pf0_data_repair_selected | pf0_data_repair_d1_selected |
        pf0_data_repair_d2_selected;
    if (pf0_head_repair_hit) begin
      pf0_head_word = head_word(s6_repair.head_prev_head, 1'b1);
    end else if (pf0_data_head_selected) begin
      pf0_head_word = head_word(s2.pf_index, 1'b0);
    end else if (pf0_head_repair_d1_selected) begin
      pf0_head_word = head_word(s6_repair_d1.head_prev_head, 1'b1);
    end else if (pf0_head_repair_d2_selected) begin
      pf0_head_word = head_word(s6_repair_d2.head_prev_head, 1'b1);
    end else begin
      pf0_head_word = pf0_head_word_raw;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      out_reg <= '0;
      next_r_data0 <= '0;
      next_r_data1 <= '0;
      head_r_data0 <= head_word('0, 1'b0);
      head_r_data1 <= head_word('0, 1'b0);
    end else begin
      if (label_addr_in_range(next_port_a_addr0)) begin
        next_r_data0 <= next_mem0[next_port_a_addr0];
        if (next0_en0) begin
          next_mem0[next_port_a_addr0] <= s6b.next0_data;
        end
      end
      if (label_addr_in_range(next_port_a_addr1)) begin
        next_r_data1 <= next_mem1[next_port_a_addr1];
        if (next0_en1) begin
          next_mem1[next_port_a_addr1] <= s6b.next0_data;
        end
      end
      if (s1.pf_valid && !s1.prev_bank && label_addr_in_range(s1.pf_index)) begin
        head_r_data0 <= head_mem0[s1.pf_index];
      end
      if (s1.pf_valid && s1.prev_bank && label_addr_in_range(s1.pf_index)) begin
        head_r_data1 <= head_mem1[s1.pf_index];
      end
      if (next1_en0 && label_addr_in_range(next_port_b_addr0)) begin
        next_mem0[next_port_b_addr0] <= s6b.next1_data;
      end
      if (next1_en1 && label_addr_in_range(next_port_b_addr1)) begin
        next_mem1[next_port_b_addr1] <= s6b.next1_data;
      end
      if ((head_curr_en0 | head_prev_en0) &&
          label_addr_in_range(head_prev_en0 ? s6b.head_prev_addr : s6b.head_curr_addr)) begin
        head_mem0[head_prev_en0 ? s6b.head_prev_addr : s6b.head_curr_addr] <=
            head_prev_en0 ? head_word(s6b.head_prev_head, 1'b1) :
            head_word(s6b.head_curr_addr, 1'b0);
      end
      if ((head_curr_en1 | head_prev_en1) &&
          label_addr_in_range(head_prev_en1 ? s6b.head_prev_addr : s6b.head_curr_addr)) begin
        head_mem1[head_prev_en1 ? s6b.head_prev_addr : s6b.head_curr_addr] <=
            head_prev_en1 ? head_word(s6b.head_prev_head, 1'b1) :
            head_word(s6b.head_curr_addr, 1'b0);
      end

      out_reg <= s2;
      out_reg.pf_next <= pf0_next;
      out_reg.pf_mapped <= head_mapped(pf0_head_word);
      out_reg.pf_head <= decoded_head(pf0_head_word, s2.pf_index);
    end
  end

`ifndef SYNTHESIS
  initial begin : validate_head_encoding
    assert (TABLE_DEPTH > 0)
      else $error("TABLE_DEPTH must be positive");
    assert (TABLE_DEPTH <= NEXT_MEMORY_DEPTH)
      else $error("TABLE_DEPTH exceeds the label address space");
    if (COMPACT_HEAD_WORD) begin
      assert (!label_addr_in_range(UNMAPPED_HEAD))
        else $error("compact head sentinel aliases a legal table index");
    end
  end

  always_ff @(posedge clk) if (!rst) begin
    if (head_prev_en0 || head_prev_en1) begin
      assert (label_addr_in_range(s6b.head_prev_head))
        else $error("mapped head write is outside TABLE_DEPTH");
    end
    if (pf0_head_repair_hit) begin
      assert (label_addr_in_range(s6_repair.head_prev_head))
        else $error("mapped repair head is outside TABLE_DEPTH");
    end
    if (pf0_head_repair_d1_selected) begin
      assert (label_addr_in_range(s6_repair_d1.head_prev_head))
        else $error("mapped d1 repair head is outside TABLE_DEPTH");
    end
    if (pf0_head_repair_d2_selected) begin
      assert (label_addr_in_range(s6_repair_d2.head_prev_head))
        else $error("mapped d2 repair head is outside TABLE_DEPTH");
    end
    if (s2.pf_valid && label_addr_in_range(s2.pf_index)) begin
      assert (!head_mapped(pf0_head_word) ||
              label_addr_in_range(
                  head_value(pf0_head_word[LABEL_BITS-1:0])))
        else $error("decoded mapped head is outside TABLE_DEPTH");
      assert (head_mapped(pf0_head_word) ||
              same_label(decoded_head(pf0_head_word, s2.pf_index),
                         s2.pf_index))
        else $error("unmapped head did not reconstruct pf_index");
    end
  end
`endif
endmodule
