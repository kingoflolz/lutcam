// Canonical readable SystemVerilog implementation of the child-feature arena.

import lrcca_util_pkg::bit_width;

module ChildArena #(
  parameter int ENTRY_COUNT = 108,
  parameter int MAX_LIST_LEN = 9,
  parameter int FEATURE_WIDTH = 34,
  parameter int SORT_KEY_WIDTH = 11,
  parameter int TAIL_KEY_LSB = 0,
  parameter int TAIL_KEY_WIDTH = SORT_KEY_WIDTH,
  parameter bit TAIL_KEY_DENSE_AREA_RANK = 1'b0,
  parameter bit EQUAL_KEY_SERVICE_MIN = 1'b0,
  parameter int CMD_QUEUE_DEPTH = 512,
  parameter bit BUNDLED_COMMANDS = 1'b0,
  parameter int BUNDLE_ADMISSION_RESERVE = 22,
  localparam int ID_BITS = bit_width(longint'(ENTRY_COUNT) + 1),
  localparam int ADDR_BITS = bit_width(longint'(ENTRY_COUNT)),
  localparam int LEN_BITS = bit_width(longint'(MAX_LIST_LEN) + 1),
  localparam int COUNT_BITS = bit_width(longint'(ENTRY_COUNT) + 1),
  localparam int MERGE_CAPACITY = 2 * MAX_LIST_LEN,
  localparam int MERGE_COUNT_BITS = bit_width(longint'(MERGE_CAPACITY) + 1),
  localparam int TAIL_KEY_SAT_BITS = 3,
  localparam int CMD_PAYLOAD_WIDTH =
      (FEATURE_WIDTH > ID_BITS) ? FEATURE_WIDTH : ID_BITS,
  localparam int CMD_WIDTH = 2 + ID_BITS + CMD_PAYLOAD_WIDTH,
  localparam int CMD_COUNT_BITS = $clog2(CMD_QUEUE_DEPTH + 1)
)(
  input  logic clk,
  input  logic rst,

  input  logic new_valid,
  output logic new_ready,
  output logic [ID_BITS-1:0] new_alloc_id,
  output logic new_rsp_valid,
  input  logic new_rsp_ready,
  output logic [ID_BITS-1:0] new_rsp_id,
  output logic new_rsp_status,

  input  logic cmd_valid,
  output logic cmd_ready,
  input  logic [1:0] cmd_op,
  input  logic [ID_BITS-1:0] cmd_id_a,
  input  logic [ID_BITS-1:0] cmd_id_b,
  input  logic [FEATURE_WIDTH-1:0] cmd_fv,

  // Optional chronological event input. A bundle contains up to three arena
  // commands from one hierarchy epoch; child INIT/ADD precedes MERGE and READ.
  input  logic bundle_child_valid,
  input  logic bundle_merge_valid,
  input  logic [ID_BITS-1:0] bundle_merge_id_a,
  input  logic [ID_BITS-1:0] bundle_merge_id_b,
  input  logic bundle_read_valid,
  input  logic [ID_BITS-1:0] bundle_read_id,
  output logic cmd_admit_ready,
  output logic [CMD_COUNT_BITS-1:0] cmd_queue_occupancy,

  output logic out_valid,
  input  logic out_ready,
  output logic [FEATURE_WIDTH-1:0] out_fv,
  output logic out_first,
  output logic out_last,

  output logic idle,
  output logic [COUNT_BITS-1:0] free_count
);
  localparam int OP_ADD = 0;
  localparam int OP_MERGE = 1;
  localparam int OP_READ = 2;
  localparam int OP_INIT = 3;

  localparam logic [ID_BITS-1:0] NULL_ID = {ID_BITS{1'b1}};
  localparam logic [ID_BITS-1:0] ENTRY_COUNT_ID = ID_BITS'(ENTRY_COUNT);
  localparam logic [ID_BITS-1:0] ID_ZERO = '0;
  localparam logic [LEN_BITS-1:0] LEN_ZERO = '0;
  localparam logic [LEN_BITS-1:0] LEN_ONE = LEN_BITS'(1);
  localparam logic [LEN_BITS-1:0] MAX_LIST_LEN_LEN = LEN_BITS'(MAX_LIST_LEN);
  localparam logic [COUNT_BITS-1:0] COUNT_ONE = COUNT_BITS'(1);
  localparam logic [MERGE_COUNT_BITS-1:0] MAX_LIST_LEN_MERGE =
      MERGE_COUNT_BITS'(MAX_LIST_LEN);
  localparam logic [TAIL_KEY_SAT_BITS-1:0] TAIL_KEY_SAT_MAX =
      {TAIL_KEY_SAT_BITS{1'b1}};
  localparam int META_WIDTH = LEN_BITS + TAIL_KEY_SAT_BITS;
  localparam int META_TAIL_LSB = LEN_BITS;

  localparam int SERVICE_STATE_COUNT = 23;
  localparam int SERVICE_STATE_INDEX_BITS = bit_width(longint'(SERVICE_STATE_COUNT));
  typedef logic [SERVICE_STATE_INDEX_BITS-1:0] service_state_index_t;
  localparam service_state_index_t S_IDLE = service_state_index_t'(0);
  localparam service_state_index_t S_ADD_HEAD = service_state_index_t'(1);
  localparam service_state_index_t S_ADD_ALLOC = service_state_index_t'(2);
  localparam service_state_index_t S_ADD_WRITE_HEAD_AFTER_NODE = service_state_index_t'(3);
  localparam service_state_index_t S_ADD_SCAN = service_state_index_t'(4);
  localparam service_state_index_t S_ADD_WRITE_INSERT_NODE = service_state_index_t'(5);
  localparam service_state_index_t S_ADD_WRITE_PREV = service_state_index_t'(6);
  localparam service_state_index_t S_ADD_WRITE_INSERT_REUSE = service_state_index_t'(7);
  localparam service_state_index_t S_ADD_WRITE_PREV_REUSE = service_state_index_t'(8);
  localparam service_state_index_t S_ADD_WRITE_HEAD_LEN = service_state_index_t'(9);
  localparam service_state_index_t S_ADD_READ_REUSE = service_state_index_t'(10);
  localparam service_state_index_t S_ADD_FULL_SCAN_REQ = service_state_index_t'(11);
  localparam service_state_index_t S_ADD_FULL_SCAN = service_state_index_t'(12);
  localparam service_state_index_t S_MERGE_READ_C_HEAD = service_state_index_t'(13);
  localparam service_state_index_t S_MERGE_READ_L_HEAD = service_state_index_t'(14);
  localparam service_state_index_t S_MERGE_FAST_DROP_WAIT_READ = service_state_index_t'(15);
  localparam service_state_index_t S_MERGE_DIRECT_SETUP = service_state_index_t'(16);
  localparam service_state_index_t S_MERGE_DIRECT_SELECT = service_state_index_t'(17);
  localparam service_state_index_t S_MERGE_DIRECT_ADVANCE_REQ = service_state_index_t'(18);
  localparam service_state_index_t S_MERGE_DIRECT_ADVANCE = service_state_index_t'(19);
  localparam service_state_index_t S_MERGE_DIRECT_WRITE_TAIL = service_state_index_t'(20);
  localparam service_state_index_t S_MERGE_DIRECT_FREE_SPARE = service_state_index_t'(21);
  localparam service_state_index_t S_MERGE_DIRECT_WRITE_HEAD = service_state_index_t'(22);
  typedef logic [SERVICE_STATE_COUNT-1:0] service_state_t;
  localparam service_state_t SERVICE_IDLE_STATE =
      service_state_t'(1'b1) << S_IDLE;

  localparam int READ_STATE_COUNT = 3;
  localparam int READ_STATE_INDEX_BITS = bit_width(longint'(READ_STATE_COUNT));
  typedef logic [READ_STATE_INDEX_BITS-1:0] read_state_index_t;
  localparam read_state_index_t R_READ_IDLE = read_state_index_t'(0);
  localparam read_state_index_t R_READ_REQ = read_state_index_t'(1);
  localparam read_state_index_t R_READ_CAPTURE = read_state_index_t'(2);
  typedef logic [READ_STATE_COUNT-1:0] read_state_t;
  localparam read_state_t READ_IDLE_STATE =
      read_state_t'(1'b1) << R_READ_IDLE;

  logic [ID_BITS-1:0] next_mem [0:ENTRY_COUNT-1];
  logic [META_WIDTH-1:0] meta_mem [0:ENTRY_COUNT-1];

  logic [ID_BITS-1:0] returned_ids_w_data;
  logic returned_ids_w_en;
  logic returned_ids_w_rdy;
  logic [ADDR_BITS-1:0] returned_ids_r_data;
  logic returned_ids_r_en;
  logic returned_ids_r_rdy;

  ChildArenaReturnedIdFifo #(
    .WIDTH(ADDR_BITS),
    .PRELOAD_COUNT(ENTRY_COUNT)
  ) child_arena_returned_ids (
    .clk(clk),
    .rst(rst),
    .w_data(returned_ids_w_data[ADDR_BITS-1:0]),
    .w_en(returned_ids_w_en),
    .w_rdy(returned_ids_w_rdy),
    .r_data(returned_ids_r_data),
    .r_en(returned_ids_r_en),
    .r_rdy(returned_ids_r_rdy)
  );

  logic cmd_fifo_w_rdy;
  logic [CMD_WIDTH-1:0] cmd_fifo_r_data;
  logic cmd_fifo_r_en;
  logic cmd_fifo_r_rdy;
  logic cmd_fifo_empty;

  generate
    if (BUNDLED_COMMANDS) begin : ordered_bundle_commands
      logic [1:0] ordered_op;
      logic [ID_BITS-1:0] ordered_id_a, ordered_id_b;
      logic [FEATURE_WIDTH-1:0] ordered_fv;
      logic [CMD_PAYLOAD_WIDTH-1:0] ordered_payload;
      ChildArenaBundleFifo #(
        .ID_BITS(ID_BITS), .FEATURE_WIDTH(FEATURE_WIDTH),
        .DEPTH(CMD_QUEUE_DEPTH), .ADMISSION_RESERVE(BUNDLE_ADMISSION_RESERVE)
      ) child_arena_cmd_fifo (
        .clk(clk), .rst(rst),
        .in_valid(cmd_valid), .in_ready(cmd_fifo_w_rdy),
        .admit_ready(cmd_admit_ready), .occupancy(cmd_queue_occupancy),
        .in_child_valid(bundle_child_valid), .in_child_init(cmd_op == 2'd3),
        .in_child_id(cmd_id_a), .in_child_fv(cmd_fv),
        .in_merge_valid(bundle_merge_valid),
        .in_merge_id_a(bundle_merge_id_a), .in_merge_id_b(bundle_merge_id_b),
        .in_read_valid(bundle_read_valid), .in_read_id(bundle_read_id),
        .out_valid(cmd_fifo_r_rdy), .out_ready(cmd_fifo_r_en),
        .out_op(ordered_op), .out_id_a(ordered_id_a),
        .out_id_b(ordered_id_b), .out_fv(ordered_fv)
      );
      assign ordered_payload = (ordered_op == 2'd1) ?
          CMD_PAYLOAD_WIDTH'(ordered_id_b) : CMD_PAYLOAD_WIDTH'(ordered_fv);
      assign cmd_fifo_r_data = {ordered_payload, ordered_id_a, ordered_op};
      assign cmd_fifo_empty = !cmd_fifo_r_rdy;
    end else begin : individual_commands
      logic [CMD_WIDTH-1:0] cmd_fifo_w_data;
      logic [CMD_PAYLOAD_WIDTH-1:0] cmd_fifo_w_payload;
      logic cmd_fifo_w_en;
      always_comb begin
        cmd_fifo_w_payload = CMD_PAYLOAD_WIDTH'(cmd_fv);
        if (cmd_op == OP_MERGE[1:0])
          cmd_fifo_w_payload[0 +: ID_BITS] = cmd_id_b;
      end
      assign cmd_fifo_w_data = {cmd_fifo_w_payload, cmd_id_a, cmd_op};
      assign cmd_fifo_w_en = cmd_valid & cmd_ready;
      ChildArenaSyncFifo #(
        .WIDTH(CMD_WIDTH), .DEPTH(CMD_QUEUE_DEPTH)
      ) child_arena_cmd_fifo (
        .clk(clk), .rst(rst),
        .w_data(cmd_fifo_w_data), .w_en(cmd_fifo_w_en), .w_rdy(cmd_fifo_w_rdy),
        .r_data(cmd_fifo_r_data), .r_en(cmd_fifo_r_en), .r_rdy(cmd_fifo_r_rdy),
        .empty(cmd_fifo_empty)
      );
      assign cmd_admit_ready = cmd_fifo_w_rdy;
      assign cmd_queue_occupancy = '0;
    end
  endgenerate

  logic [1:0] cmd_head_op;
  logic [ID_BITS-1:0] cmd_head_id_a;
  logic [ID_BITS-1:0] cmd_head_id_b;
  logic [FEATURE_WIDTH-1:0] cmd_head_fv;
  logic [CMD_PAYLOAD_WIDTH-1:0] cmd_head_payload;
  logic cmd_pop;

  service_state_t service_state;
  service_state_t service_state_n;
  service_state_t service_state_req;
  logic service_state_req_valid;
  read_state_t read_state;
  read_state_t read_state_n;
  read_state_t read_state_req;
  logic read_state_req_valid;

  logic rsp_full;
  logic rsp_full_n;
  logic [ID_BITS-1:0] new_rsp_id_n;

  logic new_fire;
  logic svc_alloc_req;
  logic svc_alloc_ready;
  logic svc_alloc_fire;
  logic [ID_BITS-1:0] svc_alloc_id;
  logic alloc_pop;
  logic [ID_BITS-1:0] alloc_id;
  logic alloc_ready;
  logic svc_alloc_available;
  logic return_push_valid;
  logic [ID_BITS-1:0] return_push_id;
  logic read_free_valid;
  logic [ID_BITS-1:0] read_free_id;
  logic service_free_valid;
  logic [ID_BITS-1:0] service_free_id;
  logic read_free_valid_pre;

  // Command IDs remain stable throughout ADD/MERGE service; dispatch only
  // replaces them after the current operation has consumed its final values.
  // MERGE therefore uses these directly instead of retaining duplicate IDs.
  logic [ID_BITS-1:0] id_a;
  logic [ID_BITS-1:0] id_b;
  logic [FEATURE_WIDTH-1:0] cmd_fv_reg;

  logic [LEN_BITS-1:0] add_len;
  logic [LEN_BITS-1:0] add_len_n;
  logic [TAIL_KEY_SAT_BITS-1:0] add_tail_key_sat;
  logic [TAIL_KEY_SAT_BITS-1:0] add_tail_key_sat_n;
  logic [ID_BITS-1:0] add_new_node;
  logic [ID_BITS-1:0] add_new_node_n;
  logic [ID_BITS-1:0] add_prev;
  logic [ID_BITS-1:0] add_prev_n;
  logic [ID_BITS-1:0] add_curr;
  logic [ID_BITS-1:0] add_curr_n;
  logic [ID_BITS-1:0] add_reuse;
  logic [ID_BITS-1:0] add_reuse_n;
  logic add_head_insert_before;
  logic add_head_insert_before_n;

  logic [TAIL_KEY_SAT_BITS-1:0] merge_c_tail_key_sat;
  logic [TAIL_KEY_SAT_BITS-1:0] merge_c_tail_key_sat_n;
  logic [LEN_BITS-1:0] merge_len_c;
  logic [LEN_BITS-1:0] merge_len_c_n;
  logic [LEN_BITS-1:0] merge_len_l;
  logic [LEN_BITS-1:0] merge_len_l_n;
  logic [MERGE_COUNT_BITS-1:0] merge_len_sum;
  logic [LEN_BITS-1:0] merge_target;
  logic [LEN_BITS-1:0] merge_target_n;
  logic [LEN_BITS-1:0] merge_drop_remaining;
  logic [LEN_BITS-1:0] merge_drop_remaining_n;
  logic [ID_BITS-1:0] merge_c_cur_node;
  logic [ID_BITS-1:0] merge_l_cur_node;
  logic [FEATURE_WIDTH-1:0] merge_c_cur_fv;
  logic [FEATURE_WIDTH-1:0] merge_l_cur_fv;
  logic [ID_BITS-1:0] merge_c_cur_next;
  logic [ID_BITS-1:0] merge_l_cur_next;
  logic merge_c_cur_w_en;
  logic merge_l_cur_w_en;
  logic [LEN_BITS-1:0] merge_c_remaining;
  logic [LEN_BITS-1:0] merge_c_remaining_n;
  logic [LEN_BITS-1:0] merge_l_remaining;
  logic [LEN_BITS-1:0] merge_l_remaining_n;
  logic merge_selected_from_c;
  logic merge_selected_from_c_n;
  logic merge_selected_drop;
  logic merge_selected_drop_n;
  logic [ID_BITS-1:0] merge_selected_next;
  logic [ID_BITS-1:0] merge_selected_next_n;
  logic [LEN_BITS-1:0] merge_selected_remaining_after;
  logic [LEN_BITS-1:0] merge_selected_remaining_after_n;
  logic merge_selected_has_next;
  logic merge_selected_has_next_n;
  logic [LEN_BITS-1:0] merge_output_count;
  logic [LEN_BITS-1:0] merge_output_count_n;
  logic merge_pending_valid;
  logic merge_pending_valid_n;
  logic [ID_BITS-1:0] merge_pending_node;
  logic [ID_BITS-1:0] merge_pending_node_n;
  logic [FEATURE_WIDTH-1:0] merge_pending_fv;
  logic [FEATURE_WIDTH-1:0] merge_pending_fv_n;
  logic [FEATURE_WIDTH-1:0] merge_head_fv;
  logic [FEATURE_WIDTH-1:0] merge_head_fv_n;
  logic [ID_BITS-1:0] merge_head_next;
  logic [ID_BITS-1:0] merge_head_next_n;
  logic [TAIL_KEY_SAT_BITS-1:0] merge_tail_key_sat;
  logic [TAIL_KEY_SAT_BITS-1:0] merge_tail_key_sat_n;
  logic merge_spare_valid;
  logic merge_spare_valid_n;
  logic [ID_BITS-1:0] merge_spare_node;
  logic [ID_BITS-1:0] merge_spare_node_n;
  logic merge_c_dropped;
  logic merge_c_dropped_n;
  logic merge_carry_valid;
  logic merge_carry_valid_n;

  logic [ID_BITS-1:0] read_node;
  logic [ID_BITS-1:0] read_node_n;
  logic [LEN_BITS-1:0] read_remaining;
  logic [LEN_BITS-1:0] read_remaining_n;
  logic read_first;
  logic read_first_n;
  logic read_active;
  logic read_active_n;
  logic read_emit;
  logic read_emit_n;
  logic read_capture_emit;
  logic [LEN_BITS-1:0] read_capture_remaining;
  logic read_capture_last;
  logic read_launch;
  logic [ID_BITS-1:0] read_launch_id;
  logic read_launch_emit;
  logic read_launch_reserved;
  logic [ID_BITS-1:0] read_port_addr;
  logic read_advance;

  logic [ID_BITS-1:0] port_a_addr;
  logic [ID_BITS-1:0] port_a_write_addr;
  logic [ID_BITS-1:0] port_b_addr;
  logic [ID_BITS-1:0] port_b_write_addr_fabric;
  logic [ID_BITS-1:0] service_read_addr;
  logic service_read_addr_valid;
  logic port_a_w_en;
  logic [FEATURE_WIDTH-1:0] port_a_fv_w_data;
  logic [ID_BITS-1:0] port_a_next_w_data;
  logic [LEN_BITS-1:0] port_a_len_w_data;
  logic [TAIL_KEY_SAT_BITS-1:0] port_a_tail_key_sat_w_data;
  logic port_b_fv_w_en;
  logic port_b_next_w_en;
  logic port_b_meta_w_en;
  logic port_b_any_w_en;
  logic port_b_addr_valid;
  logic port_b_hold_read_data;
  logic [FEATURE_WIDTH-1:0] port_b_fv_w_data;
  logic [FEATURE_WIDTH-1:0] port_b_fv_w_data_fabric;
  logic [ID_BITS-1:0] port_b_next_w_data;
  logic [LEN_BITS-1:0] port_b_len_w_data;
  logic [TAIL_KEY_SAT_BITS-1:0] port_b_tail_key_sat_w_data;

  logic [FEATURE_WIDTH-1:0] port_a_fv_r_data;
  logic [ID_BITS-1:0] port_a_next_r_data;
  logic [LEN_BITS-1:0] port_a_len_r_data;
  logic [FEATURE_WIDTH-1:0] port_b_fv_r_data;
  logic [ID_BITS-1:0] port_b_next_r_data;
  logic [LEN_BITS-1:0] port_b_len_r_data;
  logic [TAIL_KEY_SAT_BITS-1:0] port_b_tail_key_sat_r_data;
  logic [ADDR_BITS-1:0] port_a_mem_raddr;
  logic [ADDR_BITS-1:0] port_b_mem_raddr;
  logic port_a_mem_rvalid;
  logic port_b_mem_rvalid;
  logic [FEATURE_WIDTH-1:0] port_a_fv_mem_r_data;
  logic [FEATURE_WIDTH-1:0] port_b_fv_mem_r_data;
  logic [ID_BITS-1:0] port_a_next_mem_r_data;
  logic [ID_BITS-1:0] port_b_next_mem_r_data;
  logic [META_WIDTH-1:0] port_a_meta_mem_r_data;
  logic [META_WIDTH-1:0] port_b_meta_mem_r_data;
`ifndef SYNTHESIS
  logic port_b_read_data_required;
`endif

  logic merge_choose_c;
  logic [ID_BITS-1:0] merge_selected_node_comb;
  logic [FEATURE_WIDTH-1:0] merge_selected_fv_comb;
  logic [ID_BITS-1:0] merge_selected_next_comb;
  logic [LEN_BITS-1:0] merge_selected_remaining_comb;
  logic [LEN_BITS-1:0] merge_selected_remaining_after_comb;
  logic merge_selected_has_next_comb;
  logic merge_selected_is_c_head_comb;
  logic merge_drop_selected_comb;
  logic merge_free_selected_comb;
  logic merge_first_output_comb;
  logic merge_use_carry_comb;
  logic merge_use_spare_comb;
  logic [ID_BITS-1:0] merge_output_node_comb;
  logic [FEATURE_WIDTH-1:0] merge_output_fv_comb;
  logic [LEN_BITS-1:0] merge_output_count_next_comb;
  logic merge_pending_needs_write_comb;
  logic merge_can_drop_comb;

  function automatic logic is_nonzero_len(input logic [LEN_BITS-1:0] value);
    begin
      is_nonzero_len = value != LEN_ZERO;
    end
  endfunction

  function automatic logic is_zero_len(input logic [LEN_BITS-1:0] value);
    begin
      is_zero_len = value == LEN_ZERO;
    end
  endfunction

  function automatic logic is_null_id(input logic [ID_BITS-1:0] value);
    begin
      is_null_id = value == NULL_ID;
    end
  endfunction

  function automatic logic is_valid_id(input logic [ID_BITS-1:0] value);
    begin
      is_valid_id = value < ENTRY_COUNT_ID;
    end
  endfunction

  function automatic logic [SORT_KEY_WIDTH-1:0] key(
    input logic [SORT_KEY_WIDTH-1:0] fv_key
  );
    begin
      key = fv_key;
    end
  endfunction

  function automatic logic add_scan_stops(
    input logic [SORT_KEY_WIDTH-1:0] existing_key,
    input logic [SORT_KEY_WIDTH-1:0] candidate_key
  );
    begin
      add_scan_stops = EQUAL_KEY_SERVICE_MIN ?
          (key(existing_key) >= key(candidate_key)) :
          (key(existing_key) > key(candidate_key));
    end
  endfunction

  function automatic logic equal_merge_choose_c(
    input logic dropping,
    input logic [LEN_BITS-1:0] c_remaining,
    input logic [LEN_BITS-1:0] l_remaining
  );
    begin
      if (!EQUAL_KEY_SERVICE_MIN) begin
        equal_merge_choose_c = 1'b1;
      end else if (dropping) begin
        equal_merge_choose_c = 1'b0;
      end else begin
        equal_merge_choose_c = c_remaining >= l_remaining;
      end
    end
  endfunction

  function automatic logic [TAIL_KEY_SAT_BITS-1:0] sat_tail_key(
    input logic [TAIL_KEY_WIDTH-1:0] fv_key
  );
    logic tail_key_overflow;
    begin
      tail_key_overflow = 1'b0;
      for (int bit_index = TAIL_KEY_SAT_BITS; bit_index < TAIL_KEY_WIDTH; bit_index++) begin
        tail_key_overflow |= fv_key[bit_index];
      end
      sat_tail_key = tail_key_overflow ?
          TAIL_KEY_SAT_MAX : TAIL_KEY_SAT_BITS'(fv_key);
    end
  endfunction

  function automatic logic [TAIL_KEY_SAT_BITS-1:0] tail_key(
    input logic [TAIL_KEY_WIDTH-1:0] fv_key
  );
    logic [TAIL_KEY_SAT_BITS-1:0] rank_high;
    begin
      rank_high = TAIL_KEY_SAT_BITS'(fv_key);
      if (TAIL_KEY_DENSE_AREA_RANK) begin
        tail_key = (rank_high >= TAIL_KEY_SAT_BITS'(6)) ?
            TAIL_KEY_SAT_MAX : rank_high + TAIL_KEY_SAT_BITS'(1);
      end else begin
        tail_key = sat_tail_key(fv_key);
      end
    end
  endfunction

  function automatic logic tail_key_allows_fast_drop(
    input logic [TAIL_KEY_SAT_BITS-1:0] list_tail_key,
    input logic [TAIL_KEY_WIDTH-1:0] candidate_head_key
  );
    logic [TAIL_KEY_SAT_BITS-1:0] candidate_head_tail_key;
    begin
      candidate_head_tail_key = tail_key(candidate_head_key);
      tail_key_allows_fast_drop = list_tail_key < candidate_head_tail_key;
    end
  endfunction

  function automatic logic [TAIL_KEY_SAT_BITS-1:0] max_tail_key(
    input logic [TAIL_KEY_SAT_BITS-1:0] left,
    input logic [TAIL_KEY_SAT_BITS-1:0] right
  );
    begin
      max_tail_key = (left < right) ? right : left;
    end
  endfunction

  function automatic logic [META_WIDTH-1:0] pack_meta(
    input logic [LEN_BITS-1:0] length,
    input logic [TAIL_KEY_SAT_BITS-1:0] tail_key_sat
  );
    begin
      pack_meta = {tail_key_sat, length};
    end
  endfunction

  task automatic write_a_full(
    input logic [ID_BITS-1:0] addr,
    input logic [ID_BITS-1:0] next_id,
    input logic [LEN_BITS-1:0] length,
    input logic [TAIL_KEY_SAT_BITS-1:0] tail_key_sat
  );
    begin
      port_a_w_en = 1'b1;
      port_a_write_addr = addr;
      port_a_next_w_data = next_id;
      port_a_len_w_data = length;
      port_a_tail_key_sat_w_data = tail_key_sat;
    end
  endtask

  task automatic write_b_full(
    input logic [ID_BITS-1:0] addr,
    input logic [FEATURE_WIDTH-1:0] fv,
    input logic [ID_BITS-1:0] next_id,
    input logic [LEN_BITS-1:0] length,
    input logic [TAIL_KEY_SAT_BITS-1:0] tail_key_sat
  );
    begin
      port_b_fv_w_en = 1'b1;
      port_b_next_w_en = 1'b1;
      port_b_meta_w_en = 1'b1;
      port_b_write_addr_fabric = addr;
      port_b_fv_w_data_fabric = fv;
      port_b_next_w_data = next_id;
      port_b_len_w_data = length;
      port_b_tail_key_sat_w_data = tail_key_sat;
    end
  endtask

  task automatic write_b_next_meta(
    input logic [ID_BITS-1:0] addr,
    input logic [ID_BITS-1:0] next_id,
    input logic [LEN_BITS-1:0] length,
    input logic [TAIL_KEY_SAT_BITS-1:0] tail_key_sat
  );
    begin
      port_b_next_w_en = 1'b1;
      port_b_meta_w_en = 1'b1;
      port_b_write_addr_fabric = addr;
      port_b_next_w_data = next_id;
      port_b_len_w_data = length;
      port_b_tail_key_sat_w_data = tail_key_sat;
    end
  endtask

  task automatic write_b_fv_meta(
    input logic [ID_BITS-1:0] addr,
    input logic [FEATURE_WIDTH-1:0] fv,
    input logic [LEN_BITS-1:0] length,
    input logic [TAIL_KEY_SAT_BITS-1:0] tail_key_sat
  );
    begin
      port_b_fv_w_en = 1'b1;
      port_b_meta_w_en = 1'b1;
      port_b_write_addr_fabric = addr;
      port_b_fv_w_data_fabric = fv;
      port_b_len_w_data = length;
      port_b_tail_key_sat_w_data = tail_key_sat;
    end
  endtask

  task automatic write_b_next(
    input logic [ID_BITS-1:0] addr,
    input logic [ID_BITS-1:0] next_id
  );
    begin
      port_b_next_w_en = 1'b1;
      port_b_write_addr_fabric = addr;
      port_b_next_w_data = next_id;
    end
  endtask

  task automatic write_b_meta(
    input logic [ID_BITS-1:0] addr,
    input logic [LEN_BITS-1:0] length,
    input logic [TAIL_KEY_SAT_BITS-1:0] tail_key_sat
  );
    begin
      port_b_meta_w_en = 1'b1;
      port_b_write_addr_fabric = addr;
      port_b_len_w_data = length;
      port_b_tail_key_sat_w_data = tail_key_sat;
    end
  endtask

  task automatic service_next(
    input service_state_index_t state
  );
    begin
      service_state_req[state] = 1'b1;
      service_state_req_valid = 1'b1;
    end
  endtask

  task automatic read_next(
    input read_state_index_t state
  );
    begin
      read_state_req[state] = 1'b1;
      read_state_req_valid = 1'b1;
    end
  endtask

  task automatic pop_command;
    begin
      cmd_pop = 1'b1;
    end
  endtask

  task automatic read_b_addr(
    input logic [ID_BITS-1:0] addr,
    input logic addr_valid
  );
    begin
      service_read_addr = addr;
      service_read_addr_valid = addr_valid;
    end
  endtask

  task automatic dispatch_or_idle(input logic port_b_available);
    begin
      if (cmd_fifo_r_rdy) begin
        unique case (cmd_head_op)
          OP_INIT[1:0]: begin
            pop_command();
            if (is_valid_id(cmd_head_id_a)) begin
              write_a_full(
                cmd_head_id_a,
                NULL_ID,
                LEN_ONE,
                tail_key(cmd_head_fv[TAIL_KEY_LSB +: TAIL_KEY_WIDTH])
              );
            end
            service_next(S_IDLE);
          end

          OP_ADD[1:0]: begin
            if (!is_valid_id(cmd_head_id_a)) begin
              pop_command();
              service_next(S_IDLE);
            end else if (port_b_available) begin
              pop_command();
              read_b_addr(cmd_head_id_a, 1'b1);
              service_next(S_ADD_HEAD);
            end else begin
              service_next(S_IDLE);
            end
          end

          OP_MERGE[1:0]: begin
            if (
              !is_valid_id(cmd_head_id_a) ||
              !is_valid_id(cmd_head_id_b) ||
              (cmd_head_id_a == cmd_head_id_b)
            ) begin
              pop_command();
              service_next(S_IDLE);
            end else if (port_b_available) begin
              pop_command();
              read_b_addr(cmd_head_id_a, 1'b1);
              service_next(S_MERGE_READ_C_HEAD);
            end else begin
              service_next(S_IDLE);
            end
          end

          OP_READ[1:0]: begin
            if (!is_valid_id(cmd_head_id_a)) begin
              pop_command();
              service_next(S_IDLE);
            end else if (!read_active && !read_launch_reserved) begin
              pop_command();
              read_launch = 1'b1;
              read_launch_id = cmd_head_id_a;
              read_launch_emit = 1'b1;
              service_next(S_IDLE);
            end else begin
              service_next(S_IDLE);
            end
          end

          default: begin
            pop_command();
            service_next(S_IDLE);
          end
        endcase
      end else begin
        service_next(S_IDLE);
      end
    end
  endtask

  assign cmd_head_op = cmd_fifo_r_data[1:0];
  assign cmd_head_id_a = cmd_fifo_r_data[2 +: ID_BITS];
  assign cmd_head_payload = cmd_fifo_r_data[2 + ID_BITS +: CMD_PAYLOAD_WIDTH];
  assign cmd_head_id_b = cmd_head_payload[0 +: ID_BITS];
  assign cmd_head_fv = FEATURE_WIDTH'(cmd_head_payload);
  assign cmd_fifo_r_en = cmd_pop;
  assign cmd_ready = cmd_fifo_w_rdy;

  assign port_b_any_w_en = port_b_fv_w_en | port_b_next_w_en | port_b_meta_w_en;
  assign port_b_addr_valid = port_b_any_w_en | service_read_addr_valid;
  assign port_a_addr = port_a_w_en ? port_a_write_addr : read_port_addr;
  assign port_b_addr = port_b_any_w_en ? port_b_write_addr :
      (service_read_addr_valid ? service_read_addr :
          (port_b_hold_read_data ? ID_BITS'(port_b_mem_raddr) : service_read_addr));

  // Port A writes only OP_INIT commands. Qualifying the memory write with
  // port_a_w_en makes the payload a don't-care on every other cycle, so keep
  // the sole real payload source out of the service-state output mux.
  assign port_a_fv_w_data = cmd_head_fv;
  // The feature-memory write enable already qualifies this payload. Select
  // only its four possible data sources, without repeating every write's
  // service-state/handshake predicates across the feature bits.
  always_comb begin
    port_b_fv_w_data = cmd_fv_reg;
    if (service_state[S_MERGE_DIRECT_WRITE_HEAD]) begin
      port_b_fv_w_data = merge_head_fv;
    end else if (service_state[S_MERGE_DIRECT_SELECT] ||
                 service_state[S_MERGE_DIRECT_WRITE_TAIL]) begin
      port_b_fv_w_data = merge_pending_fv;
    end else if (
        (service_state[S_ADD_ALLOC] &&
         (key(cmd_fv_reg[SORT_KEY_WIDTH-1:0]) <
          key(port_b_fv_r_data[SORT_KEY_WIDTH-1:0]))) ||
        (service_state[S_ADD_READ_REUSE] &&
         (key(cmd_fv_reg[SORT_KEY_WIDTH-1:0]) >
          key(port_b_fv_r_data[SORT_KEY_WIDTH-1:0])))) begin
      port_b_fv_w_data = port_b_fv_r_data;
    end
  end

`ifndef SYNTHESIS
  // Preserve the original task-derived payload as an independent simulation
  // oracle. Values on cycles without a write are intentionally unconstrained.
  always_ff @(posedge clk) begin
    if (!rst && port_b_fv_w_en) begin
      assert (port_b_fv_w_data == port_b_fv_w_data_fabric)
        else $fatal(1, "ChildArena factored feature write payload changed");
    end
  end
`endif

  // The write enable qualifies the address just as it qualifies the payload.
  // This mux depends only on the service state, not every write condition.
  logic [ID_BITS-1:0] port_b_write_addr;
  always_comb begin
    port_b_write_addr = id_a;
    if (service_state[S_ADD_ALLOC])
      port_b_write_addr = svc_alloc_id;
    else if (service_state[S_ADD_WRITE_INSERT_NODE])
      port_b_write_addr = add_new_node;
    else if (service_state[S_ADD_WRITE_PREV] || service_state[S_ADD_WRITE_PREV_REUSE])
      port_b_write_addr = add_prev;
    else if (service_state[S_ADD_WRITE_INSERT_REUSE])
      port_b_write_addr = add_reuse;
    else if (service_state[S_MERGE_DIRECT_SELECT] || service_state[S_MERGE_DIRECT_WRITE_TAIL])
      port_b_write_addr = merge_pending_node;
  end
`ifndef SYNTHESIS
  always_ff @(posedge clk)
    if (!rst && port_b_any_w_en)
      assert (port_b_write_addr == port_b_write_addr_fabric)
        else $fatal(1, "Factored write address changed");
`endif

  // Each service state below is entered only after a port-B read request, or
  // after S_ADD_HEAD has explicitly held that read while allocation stalls.
  // Keeping this predicate beside the raw-memory aliases makes that timing
  // contract executable in simulation.
`ifndef SYNTHESIS
  assign port_b_read_data_required =
      service_state[S_ADD_HEAD] |
      service_state[S_ADD_ALLOC] |
      service_state[S_ADD_SCAN] |
      service_state[S_ADD_READ_REUSE] |
      service_state[S_ADD_FULL_SCAN] |
      service_state[S_MERGE_READ_C_HEAD] |
      service_state[S_MERGE_READ_L_HEAD] |
      service_state[S_MERGE_DIRECT_ADVANCE];
`endif

  assign new_fire = new_valid & new_ready;
  assign svc_alloc_fire = svc_alloc_req & svc_alloc_ready;
  assign return_push_valid = read_free_valid | service_free_valid;
  assign return_push_id = read_free_valid ? read_free_id : service_free_id;
  assign alloc_id = ID_BITS'(returned_ids_r_data);
  assign new_alloc_id = alloc_id;
  assign alloc_pop = new_fire | svc_alloc_fire;
  assign alloc_ready = returned_ids_r_rdy;
  assign svc_alloc_available = returned_ids_r_rdy;
  assign svc_alloc_ready = !new_fire & svc_alloc_available;
  assign svc_alloc_id = alloc_id;
  assign returned_ids_r_en = alloc_pop;
  assign returned_ids_w_data = return_push_id;
  assign returned_ids_w_en = return_push_valid;
  assign new_ready = alloc_ready & (!rsp_full | new_rsp_ready);
  assign new_rsp_valid = rsp_full;
  assign new_rsp_status = 1'b1;

  assign merge_len_sum =
      MERGE_COUNT_BITS'(merge_len_c) + MERGE_COUNT_BITS'(merge_len_l);
  assign merge_c_cur_w_en =
      service_state[S_MERGE_READ_C_HEAD] |
      (service_state[S_MERGE_DIRECT_ADVANCE] & merge_selected_from_c);
  assign merge_l_cur_w_en =
      service_state[S_MERGE_READ_L_HEAD] |
      (service_state[S_MERGE_DIRECT_ADVANCE] & !merge_selected_from_c);
  assign out_fv = port_a_fv_r_data;

  always @* begin
    read_capture_emit =
        read_emit & (!read_first | (port_a_len_r_data >= MAX_LIST_LEN_LEN));
    read_capture_remaining = read_first ? port_a_len_r_data : read_remaining;
    read_capture_last = read_capture_remaining == LEN_ONE;
    read_advance = read_state[R_READ_CAPTURE] & returned_ids_w_rdy &
        (!read_capture_emit | out_ready);
    read_free_valid_pre = read_advance;

    service_state_req = '0;
    read_state_req = '0;
    service_state_req_valid = 1'b0;
    read_state_req_valid = 1'b0;
    service_state_n = service_state;
    read_state_n = read_state;
    rsp_full_n = rsp_full;
    new_rsp_id_n = new_rsp_id;

    add_len_n = add_len;
    add_tail_key_sat_n = add_tail_key_sat;
    add_new_node_n = add_new_node;
    add_prev_n = add_prev;
    add_curr_n = add_curr;
    add_reuse_n = add_reuse;
    add_head_insert_before_n = add_head_insert_before;

    merge_c_tail_key_sat_n = merge_c_tail_key_sat;
    merge_len_c_n = merge_len_c;
    merge_len_l_n = merge_len_l;
    merge_target_n = merge_target;
    merge_drop_remaining_n = merge_drop_remaining;
    merge_c_remaining_n = merge_c_remaining;
    merge_l_remaining_n = merge_l_remaining;
    merge_selected_from_c_n = merge_selected_from_c;
    merge_selected_drop_n = merge_selected_drop;
    merge_selected_next_n = merge_selected_next;
    merge_selected_remaining_after_n = merge_selected_remaining_after;
    merge_selected_has_next_n = merge_selected_has_next;
    merge_output_count_n = merge_output_count;
    merge_pending_valid_n = merge_pending_valid;
    merge_pending_node_n = merge_pending_node;
    merge_pending_fv_n = merge_pending_fv;
    merge_head_fv_n = merge_head_fv;
    merge_head_next_n = merge_head_next;
    merge_tail_key_sat_n = merge_tail_key_sat;
    merge_spare_valid_n = merge_spare_valid;
    merge_spare_node_n = merge_spare_node;
    merge_c_dropped_n = merge_c_dropped;
    merge_carry_valid_n = merge_carry_valid;

    read_node_n = read_node;
    read_remaining_n = read_remaining;
    read_first_n = read_first;
    read_active_n = read_active;
    read_emit_n = read_emit;

    svc_alloc_req = 1'b0;
    cmd_pop = 1'b0;
    read_launch = 1'b0;
    read_launch_id = ID_ZERO;
    read_launch_emit = 1'b0;
    read_launch_reserved = 1'b0;
    service_read_addr = ID_ZERO;
    service_read_addr_valid = 1'b0;
    read_port_addr = ID_ZERO;
    port_a_w_en = 1'b0;
    port_a_write_addr = ID_ZERO;
    port_a_next_w_data = ID_ZERO;
    port_a_len_w_data = LEN_ZERO;
    port_a_tail_key_sat_w_data = '0;
    port_b_fv_w_en = 1'b0;
    port_b_next_w_en = 1'b0;
    port_b_meta_w_en = 1'b0;
    port_b_hold_read_data = 1'b0;
    port_b_write_addr_fabric = ID_ZERO;
    port_b_fv_w_data_fabric = '0;
    port_b_next_w_data = ID_ZERO;
    port_b_len_w_data = LEN_ZERO;
    port_b_tail_key_sat_w_data = '0;
    read_free_valid = 1'b0;
    read_free_id = ID_ZERO;
    service_free_valid = 1'b0;
    service_free_id = ID_ZERO;
    idle = 1'b0;
    out_valid = 1'b0;
    out_first = 1'b0;
    out_last = 1'b0;

    merge_choose_c = 1'b0;
    merge_selected_node_comb = ID_ZERO;
    merge_selected_fv_comb = '0;
    merge_selected_next_comb = ID_ZERO;
    merge_selected_remaining_comb = LEN_ZERO;
    merge_selected_remaining_after_comb = LEN_ZERO;
    merge_selected_has_next_comb = 1'b0;
    merge_selected_is_c_head_comb = 1'b0;
    merge_drop_selected_comb = 1'b0;
    merge_free_selected_comb = 1'b0;
    merge_first_output_comb = 1'b0;
    merge_use_carry_comb = 1'b0;
    merge_use_spare_comb = 1'b0;
    merge_output_node_comb = ID_ZERO;
    merge_output_fv_comb = '0;
    merge_output_count_next_comb = LEN_ZERO;
    merge_pending_needs_write_comb = 1'b0;
    merge_can_drop_comb = 1'b0;

    if (rsp_full && new_rsp_ready) begin
      rsp_full_n = 1'b0;
    end
    if (new_fire) begin
      new_rsp_id_n = alloc_id;
      rsp_full_n = 1'b1;
    end

    unique case (1'b1)
      service_state[S_IDLE]: begin
        idle = cmd_fifo_empty && !read_active;
        dispatch_or_idle(1'b1);
      end

      service_state[S_ADD_HEAD]: begin
        add_len_n = port_b_len_r_data;
        add_tail_key_sat_n = port_b_tail_key_sat_r_data;

        if (port_b_len_r_data < MAX_LIST_LEN_LEN) begin
          port_b_hold_read_data = 1'b1;
          service_next(S_ADD_ALLOC);
        end else if (key(cmd_fv_reg[SORT_KEY_WIDTH-1:0]) <= key(port_b_fv_r_data[SORT_KEY_WIDTH-1:0])) begin
          // The retained head is already at least as large. Discard the
          // candidate without touching metadata, including on the first drop.
          dispatch_or_idle(1'b1);
        end else if (MAX_LIST_LEN == 1) begin
          write_b_full(
            id_a,
            cmd_fv_reg,
            NULL_ID,
            port_b_len_r_data,
            tail_key(cmd_fv_reg[TAIL_KEY_LSB +: TAIL_KEY_WIDTH])
          );
          dispatch_or_idle(1'b0);
        end else begin
          add_curr_n = port_b_next_r_data;
          read_b_addr(port_b_next_r_data, !is_null_id(port_b_next_r_data));
          service_next(S_ADD_READ_REUSE);
        end
      end

      service_state[S_ADD_ALLOC]: begin
        svc_alloc_req = 1'b1;
        if (!svc_alloc_ready) begin
          port_b_hold_read_data = 1'b1;
        end
        if (svc_alloc_ready) begin
          add_new_node_n = svc_alloc_id;
          add_prev_n = id_a;
          add_curr_n = port_b_next_r_data;

          if (key(cmd_fv_reg[SORT_KEY_WIDTH-1:0]) < key(port_b_fv_r_data[SORT_KEY_WIDTH-1:0])) begin
            write_b_full(
              svc_alloc_id,
              port_b_fv_r_data,
              port_b_next_r_data,
              LEN_ZERO,
              '0
            );
            add_head_insert_before_n = 1'b1;
            service_next(S_ADD_WRITE_HEAD_AFTER_NODE);
          end else if (key(cmd_fv_reg[SORT_KEY_WIDTH-1:0]) == key(port_b_fv_r_data[SORT_KEY_WIDTH-1:0])) begin
            write_b_full(
              svc_alloc_id,
              cmd_fv_reg,
              port_b_next_r_data,
              LEN_ZERO,
              '0
            );
            add_head_insert_before_n = 1'b0;
            service_next(S_ADD_WRITE_HEAD_AFTER_NODE);
          end else if (is_null_id(port_b_next_r_data)) begin
            service_next(S_ADD_WRITE_INSERT_NODE);
          end else begin
            read_b_addr(port_b_next_r_data, 1'b1);
            service_next(S_ADD_SCAN);
          end
        end
      end

      service_state[S_ADD_WRITE_HEAD_AFTER_NODE]: begin
        write_b_next_meta(
          id_a,
          add_new_node,
          add_len + LEN_ONE,
          add_tail_key_sat
        );
        if (add_head_insert_before) begin
          port_b_fv_w_en = 1'b1;
          port_b_fv_w_data_fabric = cmd_fv_reg;
        end
        dispatch_or_idle(1'b0);
      end

      service_state[S_ADD_SCAN]: begin
        if (add_scan_stops(
            port_b_fv_r_data[SORT_KEY_WIDTH-1:0],
            cmd_fv_reg[SORT_KEY_WIDTH-1:0])) begin
          service_next(S_ADD_WRITE_INSERT_NODE);
        end else begin
          add_prev_n = add_curr;
          add_curr_n = port_b_next_r_data;
          if (is_null_id(port_b_next_r_data)) begin
            service_next(S_ADD_WRITE_INSERT_NODE);
          end else begin
            read_b_addr(port_b_next_r_data, 1'b1);
            service_next(S_ADD_SCAN);
          end
        end
      end

      service_state[S_ADD_WRITE_INSERT_NODE]: begin
        write_b_full(
          add_new_node,
          cmd_fv_reg,
          add_curr,
          LEN_ZERO,
          '0
        );
        service_next(S_ADD_WRITE_PREV);
      end

      service_state[S_ADD_WRITE_PREV]: begin
        if (add_prev == id_a) begin
          write_b_next_meta(
            add_prev,
            add_new_node,
            add_len + LEN_ONE,
            is_null_id(add_curr) ? tail_key(cmd_fv_reg[TAIL_KEY_LSB +: TAIL_KEY_WIDTH]) : add_tail_key_sat
          );
        end else begin
          write_b_next(add_prev, add_new_node);
        end

        if (add_prev == id_a) begin
          dispatch_or_idle(1'b0);
        end else begin
          service_next(S_ADD_WRITE_HEAD_LEN);
        end
      end

      service_state[S_ADD_WRITE_INSERT_REUSE]: begin
        write_b_full(
          add_reuse,
          cmd_fv_reg,
          add_curr,
          LEN_ZERO,
          '0
        );
        service_next(S_ADD_WRITE_PREV_REUSE);
      end

      service_state[S_ADD_WRITE_PREV_REUSE]: begin
        write_b_next(add_prev, add_reuse);
        dispatch_or_idle(1'b0);
      end

      service_state[S_ADD_WRITE_HEAD_LEN]: begin
        write_b_meta(
          id_a,
          add_len + LEN_ONE,
          is_null_id(add_curr) ? tail_key(cmd_fv_reg[TAIL_KEY_LSB +: TAIL_KEY_WIDTH]) : add_tail_key_sat
        );
        dispatch_or_idle(1'b0);
      end

      service_state[S_ADD_READ_REUSE]: begin
        add_reuse_n = add_curr;
        if (key(cmd_fv_reg[SORT_KEY_WIDTH-1:0]) <= key(port_b_fv_r_data[SORT_KEY_WIDTH-1:0])) begin
          write_b_fv_meta(
            id_a,
            cmd_fv_reg,
            add_len,
            add_tail_key_sat
          );
          dispatch_or_idle(1'b0);
        end else begin
          write_b_full(
            id_a,
            port_b_fv_r_data,
            port_b_next_r_data,
            add_len,
            max_tail_key(add_tail_key_sat, tail_key(cmd_fv_reg[TAIL_KEY_LSB +: TAIL_KEY_WIDTH]))
          );
          add_prev_n = id_a;
          add_curr_n = port_b_next_r_data;
          if (is_null_id(port_b_next_r_data)) begin
            service_next(S_ADD_WRITE_INSERT_REUSE);
          end else begin
            service_next(S_ADD_FULL_SCAN_REQ);
          end
        end
      end

      service_state[S_ADD_FULL_SCAN_REQ]: begin
        read_b_addr(add_curr, 1'b1);
        service_next(S_ADD_FULL_SCAN);
      end

      service_state[S_ADD_FULL_SCAN]: begin
        if (add_scan_stops(
            port_b_fv_r_data[SORT_KEY_WIDTH-1:0],
            cmd_fv_reg[SORT_KEY_WIDTH-1:0])) begin
          service_next(S_ADD_WRITE_INSERT_REUSE);
        end else begin
          add_prev_n = add_curr;
          add_curr_n = port_b_next_r_data;
          if (is_null_id(port_b_next_r_data)) begin
            service_next(S_ADD_WRITE_INSERT_REUSE);
          end else begin
            read_b_addr(port_b_next_r_data, 1'b1);
            service_next(S_ADD_FULL_SCAN);
          end
        end
      end

      service_state[S_MERGE_READ_C_HEAD]: begin
        merge_len_c_n = port_b_len_r_data;
        merge_c_tail_key_sat_n = port_b_tail_key_sat_r_data;
        read_b_addr(id_b, 1'b1);
        service_next(S_MERGE_READ_L_HEAD);
      end

      service_state[S_MERGE_READ_L_HEAD]: begin
        merge_len_l_n = port_b_len_r_data;

        if (
          (merge_len_c == MAX_LIST_LEN_LEN) &&
          (port_b_tail_key_sat_r_data != TAIL_KEY_SAT_MAX) &&
          tail_key_allows_fast_drop(port_b_tail_key_sat_r_data, merge_c_cur_fv[TAIL_KEY_LSB +: TAIL_KEY_WIDTH])
        ) begin
          if (!read_active) begin
            read_launch = 1'b1;
            read_launch_id = id_b;
            read_launch_emit = 1'b0;
            read_launch_reserved = 1'b1;
            dispatch_or_idle(1'b0);
          end else begin
            service_next(S_MERGE_FAST_DROP_WAIT_READ);
          end
        end else begin
          service_next(S_MERGE_DIRECT_SETUP);
        end
      end

      service_state[S_MERGE_FAST_DROP_WAIT_READ]: begin
        if (!read_active) begin
          read_launch = 1'b1;
          read_launch_id = id_b;
          read_launch_emit = 1'b0;
          read_launch_reserved = 1'b1;
          dispatch_or_idle(1'b0);
        end
      end

      service_state[S_MERGE_DIRECT_SETUP]: begin
        merge_target_n = (merge_len_sum > MAX_LIST_LEN_MERGE) ?
            MAX_LIST_LEN_LEN : merge_len_sum[LEN_BITS-1:0];
        merge_drop_remaining_n = (merge_len_sum > MAX_LIST_LEN_MERGE) ?
            LEN_BITS'(merge_len_sum - MAX_LIST_LEN_MERGE) : LEN_ZERO;
        merge_c_remaining_n = merge_len_c;
        merge_l_remaining_n = merge_len_l;
        merge_output_count_n = LEN_ZERO;
        merge_pending_valid_n = 1'b0;
        merge_spare_valid_n = 1'b0;
        merge_c_dropped_n = 1'b0;
        merge_carry_valid_n = 1'b0;
        service_next(S_MERGE_DIRECT_SELECT);
      end

      service_state[S_MERGE_DIRECT_SELECT]: begin
        merge_choose_c = is_zero_len(merge_l_remaining) ||
            (is_nonzero_len(merge_c_remaining) &&
             ((key(merge_c_cur_fv[SORT_KEY_WIDTH-1:0]) <
               key(merge_l_cur_fv[SORT_KEY_WIDTH-1:0])) ||
              ((key(merge_c_cur_fv[SORT_KEY_WIDTH-1:0]) ==
                key(merge_l_cur_fv[SORT_KEY_WIDTH-1:0])) &&
               equal_merge_choose_c(
                   merge_drop_remaining != LEN_ZERO,
                   merge_c_remaining,
                   merge_l_remaining))));
        merge_selected_node_comb = merge_choose_c ? merge_c_cur_node : merge_l_cur_node;
        merge_selected_fv_comb = merge_choose_c ? merge_c_cur_fv : merge_l_cur_fv;
        merge_selected_next_comb = merge_choose_c ? merge_c_cur_next : merge_l_cur_next;
        merge_selected_remaining_comb =
            merge_choose_c ? merge_c_remaining : merge_l_remaining;
        merge_selected_remaining_after_comb =
            merge_selected_remaining_comb - LEN_ONE;
        merge_selected_has_next_comb =
            merge_selected_remaining_comb != LEN_ONE;
        merge_selected_is_c_head_comb =
            merge_choose_c && (merge_selected_node_comb == id_a);
        merge_drop_selected_comb = merge_drop_remaining != LEN_ZERO;
        merge_free_selected_comb =
            merge_drop_selected_comb && !merge_selected_is_c_head_comb;
        merge_first_output_comb = merge_output_count == LEN_ZERO;
        merge_use_carry_comb =
            !merge_first_output_comb && merge_carry_valid &&
            merge_selected_is_c_head_comb;
        merge_use_spare_comb =
            !merge_first_output_comb && !merge_use_carry_comb && merge_spare_valid;
        merge_output_node_comb = merge_first_output_comb ? id_a :
            ((merge_use_carry_comb || merge_use_spare_comb) ?
                merge_spare_node : merge_selected_node_comb);
        merge_output_fv_comb = merge_selected_fv_comb;
        merge_output_count_next_comb = merge_output_count + LEN_ONE;
        merge_pending_needs_write_comb =
            merge_pending_valid && (merge_pending_node != id_a);
        merge_can_drop_comb =
            !merge_free_selected_comb || (!read_free_valid_pre && returned_ids_w_rdy);

        if (merge_drop_selected_comb) begin
          if (merge_can_drop_comb) begin
            if (merge_free_selected_comb) begin
              service_free_valid = 1'b1;
              service_free_id = merge_selected_node_comb;
            end else begin
              merge_c_dropped_n = 1'b1;
            end
            merge_selected_from_c_n = merge_choose_c;
            merge_selected_next_n = merge_selected_next_comb;
            merge_selected_remaining_after_n = merge_selected_remaining_after_comb;
            merge_selected_has_next_n = merge_selected_has_next_comb;
            merge_selected_drop_n = 1'b1;
            merge_drop_remaining_n = merge_drop_remaining - LEN_ONE;

            if (merge_selected_has_next_comb) begin
              read_b_addr(merge_selected_next_comb, 1'b1);
              service_next(S_MERGE_DIRECT_ADVANCE);
            end else begin
              if (merge_choose_c) begin
                merge_c_remaining_n = LEN_ZERO;
              end else begin
                merge_l_remaining_n = LEN_ZERO;
              end
              service_next(S_MERGE_DIRECT_SELECT);
            end
          end
        end else begin
          if (merge_pending_needs_write_comb) begin
            write_b_full(
              merge_pending_node,
              merge_pending_fv,
              merge_output_node_comb,
              LEN_ZERO,
              '0
            );
          end else if (merge_pending_valid) begin
            merge_head_next_n = merge_output_node_comb;
          end

          merge_selected_from_c_n = merge_choose_c;
          merge_selected_next_n = merge_selected_next_comb;
          merge_selected_remaining_after_n = merge_selected_remaining_after_comb;
          merge_selected_has_next_n = merge_selected_has_next_comb;
          merge_selected_drop_n = 1'b0;
          merge_pending_valid_n = 1'b1;
          merge_pending_node_n = merge_output_node_comb;
          merge_pending_fv_n = merge_output_fv_comb;
          merge_tail_key_sat_n = tail_key(merge_output_fv_comb[TAIL_KEY_LSB +: TAIL_KEY_WIDTH]);
          merge_output_count_n = merge_output_count_next_comb;

          if (merge_first_output_comb) begin
            merge_head_fv_n = merge_output_fv_comb;
            merge_head_next_n = NULL_ID;
            if (merge_selected_node_comb != id_a) begin
              merge_spare_valid_n = 1'b1;
              merge_spare_node_n = merge_selected_node_comb;
              merge_carry_valid_n = !merge_c_dropped;
            end else begin
              merge_spare_valid_n = 1'b0;
              merge_carry_valid_n = 1'b0;
            end
          end else if (merge_use_carry_comb) begin
            merge_spare_valid_n = 1'b0;
            merge_carry_valid_n = 1'b0;
          end else if (merge_use_spare_comb) begin
            if (merge_selected_node_comb != id_a) begin
              merge_spare_valid_n = 1'b1;
              merge_spare_node_n = merge_selected_node_comb;
            end else begin
              merge_spare_valid_n = 1'b0;
            end
          end

          if (merge_selected_has_next_comb) begin
            if (merge_pending_needs_write_comb) begin
              service_next(S_MERGE_DIRECT_ADVANCE_REQ);
            end else begin
              read_b_addr(merge_selected_next_comb, 1'b1);
              service_next(S_MERGE_DIRECT_ADVANCE);
            end
          end else begin
            if (merge_choose_c) begin
              merge_c_remaining_n = LEN_ZERO;
            end else begin
              merge_l_remaining_n = LEN_ZERO;
            end
            if (merge_output_count_next_comb == merge_target) begin
              service_next(S_MERGE_DIRECT_WRITE_TAIL);
            end else begin
              service_next(S_MERGE_DIRECT_SELECT);
            end
          end
        end
      end

      service_state[S_MERGE_DIRECT_ADVANCE_REQ]: begin
        read_b_addr(merge_selected_next, 1'b1);
        service_next(S_MERGE_DIRECT_ADVANCE);
      end

      service_state[S_MERGE_DIRECT_ADVANCE]: begin
        if (merge_selected_from_c) begin
          merge_c_remaining_n = merge_selected_remaining_after;
        end else begin
          merge_l_remaining_n = merge_selected_remaining_after;
        end
        if (!merge_selected_drop && (merge_output_count == merge_target)) begin
          service_next(S_MERGE_DIRECT_WRITE_TAIL);
        end else begin
          service_next(S_MERGE_DIRECT_SELECT);
        end
      end

      service_state[S_MERGE_DIRECT_WRITE_TAIL]: begin
        if (merge_pending_node != id_a) begin
          write_b_full(
            merge_pending_node,
            merge_pending_fv,
            NULL_ID,
            LEN_ZERO,
            '0
          );
        end
        if (merge_spare_valid) begin
          service_next(S_MERGE_DIRECT_FREE_SPARE);
        end else begin
          service_next(S_MERGE_DIRECT_WRITE_HEAD);
        end
      end

      service_state[S_MERGE_DIRECT_FREE_SPARE]: begin
        if (!read_free_valid_pre && returned_ids_w_rdy) begin
          service_free_valid = 1'b1;
          service_free_id = merge_spare_node;
          merge_spare_valid_n = 1'b0;
          service_next(S_MERGE_DIRECT_WRITE_HEAD);
        end
      end

      service_state[S_MERGE_DIRECT_WRITE_HEAD]: begin
        write_b_full(
          id_a,
          merge_head_fv,
          merge_head_next,
          merge_target,
          merge_tail_key_sat
        );
        dispatch_or_idle(1'b0);
      end

      default: begin
        service_next(S_IDLE);
      end
    endcase
    service_state_n =
        service_state_req_valid ? service_state_req : service_state;

    unique case (1'b1)
      read_state[R_READ_IDLE]: begin
        if (read_launch) begin
          read_active_n = 1'b1;
          read_emit_n = read_launch_emit;
          read_node_n = read_launch_id;
          read_first_n = 1'b1;
          read_next(R_READ_REQ);
        end
      end

      read_state[R_READ_REQ]: begin
        if (!port_a_w_en) begin
          read_port_addr = read_node;
          read_next(R_READ_CAPTURE);
        end
      end

      read_state[R_READ_CAPTURE]: begin
        out_valid = read_capture_emit & returned_ids_w_rdy;
        out_first = read_first;
        out_last = read_capture_last;
        if (read_advance) begin
          read_free_valid = 1'b1;
          read_free_id = read_node;
          read_node_n = port_a_next_r_data;
          read_remaining_n = read_capture_remaining - LEN_ONE;
          read_first_n = 1'b0;
          read_emit_n = read_capture_emit;
          if (read_capture_last) begin
            read_active_n = 1'b0;
            read_next(R_READ_IDLE);
          end else begin
            read_next(R_READ_REQ);
          end
        end else begin
          read_port_addr = read_node;
        end
      end

      default: begin
        read_next(R_READ_IDLE);
      end
    endcase
    read_state_n = read_state_req_valid ? read_state_req : read_state;
  end

  generate
    for (genvar init_entry = 0; init_entry < ENTRY_COUNT; init_entry++) begin : initialize
      initial begin
        next_mem[init_entry] = NULL_ID;
        meta_mem[init_entry] = pack_meta('0, '0);
      end
    end
  endgenerate

  ChildArenaFvMem #(
    .ENTRY_COUNT(ENTRY_COUNT),
    .ADDR_BITS(ADDR_BITS),
    .DATA_WIDTH(FEATURE_WIDTH)
  ) child_arena_fv_mem (
    .clk(clk),
    .port_a_w_en(port_a_w_en),
    .port_a_w_data(port_a_fv_w_data),
    .port_a_addr(port_a_addr[ADDR_BITS-1:0]),
    .port_a_r_data(port_a_fv_mem_r_data),
    .port_b_w_en(port_b_fv_w_en),
    .port_b_w_data(port_b_fv_w_data),
    .port_b_addr(port_b_addr[ADDR_BITS-1:0]),
    .port_b_r_data(port_b_fv_mem_r_data)
  );

  assign port_a_next_mem_r_data = next_mem[port_a_mem_raddr];
  assign port_b_next_mem_r_data = next_mem[port_b_mem_raddr];
  assign port_a_meta_mem_r_data = meta_mem[port_a_mem_raddr];
  assign port_b_meta_mem_r_data = meta_mem[port_b_mem_raddr];

  assign port_a_fv_r_data = port_a_fv_mem_r_data;
  assign port_a_next_r_data =
      port_a_mem_rvalid ? port_a_next_mem_r_data : NULL_ID;
  assign port_a_len_r_data =
      port_a_mem_rvalid ? port_a_meta_mem_r_data[0 +: LEN_BITS] : '0;

  assign port_b_fv_r_data = port_b_fv_mem_r_data;
  assign port_b_next_r_data = port_b_next_mem_r_data;
  assign port_b_len_r_data = port_b_meta_mem_r_data[0 +: LEN_BITS];
  assign port_b_tail_key_sat_r_data =
      port_b_meta_mem_r_data[META_TAIL_LSB +: TAIL_KEY_SAT_BITS];

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (!rst) begin
      assert (!port_a_w_en ||
          (cmd_pop && (cmd_head_op == OP_INIT[1:0]) &&
           is_valid_id(cmd_head_id_a)))
        else $fatal(1, "ChildArena port-A write was not a valid INIT");
      assert (!port_b_read_data_required || port_b_mem_rvalid)
        else $fatal(1, "ChildArena consumed port-B data without a valid read");
    end
  end
`endif

  always_ff @(posedge clk) begin
    if (port_a_w_en) begin
      next_mem[port_a_addr[ADDR_BITS-1:0]] <= port_a_next_w_data;
      meta_mem[port_a_addr[ADDR_BITS-1:0]] <=
          pack_meta(port_a_len_w_data,
              port_a_tail_key_sat_w_data);
    end
  end

  always_ff @(posedge clk) begin
    if (port_b_next_w_en) begin
      next_mem[port_b_addr[ADDR_BITS-1:0]] <= port_b_next_w_data;
    end
    if (port_b_meta_w_en) begin
      meta_mem[port_b_addr[ADDR_BITS-1:0]] <=
          pack_meta(port_b_len_w_data,
              port_b_tail_key_sat_w_data);
    end
  end

  always_ff @(posedge clk) begin
    port_a_mem_raddr <= port_a_addr[ADDR_BITS-1:0];
    port_b_mem_raddr <= port_b_addr[ADDR_BITS-1:0];
    port_a_mem_rvalid <= is_valid_id(port_a_addr);
    if (!port_b_addr_valid && port_b_hold_read_data) begin
      port_b_mem_rvalid <= port_b_mem_rvalid;
    end else begin
      port_b_mem_rvalid <= port_b_addr_valid;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      service_state <= SERVICE_IDLE_STATE;
      read_state <= READ_IDLE_STATE;
      free_count <= COUNT_BITS'(ENTRY_COUNT);
      rsp_full <= 1'b0;
      new_rsp_id <= '0;
      id_a <= '0;
      id_b <= '0;
      add_len <= '0;
      add_tail_key_sat <= '0;
      add_new_node <= '0;
      add_prev <= '0;
      add_curr <= '0;
      add_reuse <= '0;
      add_head_insert_before <= 1'b0;
      merge_c_tail_key_sat <= '0;
      merge_len_c <= '0;
      merge_len_l <= '0;
      merge_target <= '0;
      merge_drop_remaining <= '0;
      merge_c_cur_node <= '0;
      merge_l_cur_node <= '0;
      merge_c_cur_next <= '0;
      merge_l_cur_next <= '0;
      merge_c_remaining <= '0;
      merge_l_remaining <= '0;
      merge_selected_from_c <= 1'b0;
      merge_selected_drop <= 1'b0;
      merge_selected_next <= '0;
      merge_selected_remaining_after <= '0;
      merge_selected_has_next <= 1'b0;
      merge_output_count <= '0;
      merge_pending_valid <= 1'b0;
      merge_pending_node <= '0;
      merge_head_next <= '0;
      merge_tail_key_sat <= '0;
      merge_spare_valid <= 1'b0;
      merge_spare_node <= '0;
      merge_c_dropped <= 1'b0;
      merge_carry_valid <= 1'b0;
      read_node <= '0;
      read_remaining <= '0;
      read_first <= 1'b0;
      read_active <= 1'b0;
      read_emit <= 1'b0;
    end else begin
      service_state <= service_state_n;
      read_state <= read_state_n;
      unique case ({return_push_valid, alloc_pop})
        2'b10: free_count <= free_count + COUNT_ONE;
        2'b01: free_count <= free_count - COUNT_ONE;
        default: free_count <= free_count;
      endcase
      rsp_full <= rsp_full_n;
      new_rsp_id <= new_rsp_id_n;
      if (cmd_pop) begin
        id_a <= cmd_head_id_a;
        id_b <= cmd_head_id_b;
        cmd_fv_reg <= cmd_head_fv;
      end
      add_len <= add_len_n;
      add_tail_key_sat <= add_tail_key_sat_n;
      add_new_node <= add_new_node_n;
      add_prev <= add_prev_n;
      add_curr <= add_curr_n;
      add_reuse <= add_reuse_n;
      add_head_insert_before <= add_head_insert_before_n;
      merge_c_tail_key_sat <= merge_c_tail_key_sat_n;
      merge_len_c <= merge_len_c_n;
      merge_len_l <= merge_len_l_n;
      merge_target <= merge_target_n;
      merge_drop_remaining <= merge_drop_remaining_n;
      if (merge_c_cur_w_en) begin
        merge_c_cur_node <= ID_BITS'(port_b_mem_raddr);
        merge_c_cur_fv <= port_b_fv_r_data;
        merge_c_cur_next <= port_b_next_r_data;
      end
      if (merge_l_cur_w_en) begin
        merge_l_cur_node <= ID_BITS'(port_b_mem_raddr);
        merge_l_cur_fv <= port_b_fv_r_data;
        merge_l_cur_next <= port_b_next_r_data;
      end
      merge_c_remaining <= merge_c_remaining_n;
      merge_l_remaining <= merge_l_remaining_n;
      merge_selected_from_c <= merge_selected_from_c_n;
      merge_selected_drop <= merge_selected_drop_n;
      merge_selected_next <= merge_selected_next_n;
      merge_selected_remaining_after <= merge_selected_remaining_after_n;
      merge_selected_has_next <= merge_selected_has_next_n;
      merge_output_count <= merge_output_count_n;
      merge_pending_valid <= merge_pending_valid_n;
      merge_pending_node <= merge_pending_node_n;
      merge_pending_fv <= merge_pending_fv_n;
      merge_head_fv <= merge_head_fv_n;
      merge_head_next <= merge_head_next_n;
      merge_tail_key_sat <= merge_tail_key_sat_n;
      merge_spare_valid <= merge_spare_valid_n;
      merge_spare_node <= merge_spare_node_n;
      merge_c_dropped <= merge_c_dropped_n;
      merge_carry_valid <= merge_carry_valid_n;
      read_node <= read_node_n;
      read_remaining <= read_remaining_n;
      read_first <= read_first_n;
      read_active <= read_active_n;
      read_emit <= read_emit_n;
    end
  end
endmodule
