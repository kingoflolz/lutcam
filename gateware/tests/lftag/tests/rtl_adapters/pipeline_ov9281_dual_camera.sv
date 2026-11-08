`default_nettype none

// Test-only synchronized dual-camera proof. Each adapter contains its own
// production CSI receiver, MonoIsp, CameraPipeline, and frontend memories.
// Candidate queues share one LFTag numeric engine, matching the board design.
// Camera 0 is checked against the software references. Camera 1 must match
// its frontend cycles and ordered result packets, but may retire later.
// Both links receive identical data and clocks. This does not model separate
// link phase, independent image content, or the shared UDP transport.
module pipeline_ov9281_dual_camera (
  input  logic        reset_n,
  input  logic        byte_clk,
  input  logic        core_clk,
  input  logic        status_clear,
  input  logic        clk_lp_p,
  input  logic        clk_lp_n,
  output logic        clk_hs_term,
  output logic        clk_hs_ena,
  input  logic [1:0]  data_lp_p,
  input  logic [1:0]  data_lp_n,
  input  logic [7:0]  data0_hs_in,
  input  logic [7:0]  data1_hs_in,
  input  logic [1:0]  data_fifo_empty,
  output logic [1:0]  data_fifo_rd,
  output logic [1:0]  data_rst,
  output logic [1:0]  data_hs_term,
  output logic [1:0]  data_hs_ena,
  output logic        init_done,
  output logic [15:0] receiver_status,

  output logic        inner_fv_valid,
  output logic [32:0] inner_fv_data,
  output logic        hier_inner_end,
  output logic        lwc_out_sof,
  output logic        lwc_out_valid,
  output logic [15:0] lwc_out_data,
  output logic        lwc_protocol_error,
  output logic        lftag_result_valid,
  output logic [31:0] lftag_result_data,
  output logic        lftag_result_last,
  output logic        lftag_busy,
  output logic [5:0]  lftag_fifo_level,
  output logic        lftag_candidate_start,
  output logic        lftag_candidate_done,
  output logic [9:0]  lftag_candidate_cycles,
  output logic        lftag_fifo_drop_sticky,
  output logic        lftag_protocol_error_sticky,
  output logic        threshold_protocol_error,
  output logic        threshold_buffer_overrun,
  output logic        threshold_frontend_overrun,
  output logic        hierarchy_overrun,
  output logic [7:0]  hier_child_sort_key,
  output logic [63:0] threshold_stream_hash,
  output logic [31:0] threshold_stream_pairs,
  output logic [31:0] outer_table_overflows,
  output logic [31:0] inner_table_overflows,
  output logic        tap_overflow,
  output logic [63:0] lwc_group_schedule_hash,
  output logic [31:0] lwc_group_schedule_count,
  output logic        lwc_group_schedule_valid,
  output logic [3:0]  lwc_group_schedule_band,
  output logic [11:0] lwc_raw_valid,

  output logic        receiver_protocol_error,
  output logic        receiver_marker_error,
  output logic [9:0]  camera_rows_received,
  output logic [9:0]  core_rows_started,
  output logic [9:0]  core_rows_completed,
  output logic        camera_line_done,
  output logic        core_row_start,
  output logic        core_pair_valid,
  output logic        core_frame_active,
  output wire         camera_fault,
  output wire         camera_idle,
  output wire [31:0]  frames_completed,
  output wire [31:0]  current_frame_id,
  output logic [31:0] receiver_pairs,
  output logic [31:0] capture_sof_clock,
  output logic [31:0] capture_eof_clock,
  output logic [31:0] capture_retire_clock,
  output wire [7:0] dual_compare_error,
  output logic [31:0] dual_core_compare_cycles,
  output logic [31:0] dual_byte_compare_cycles,
  output wire [31:0] second_frames_completed,
  output wire second_camera_idle,
  output wire [31:0] second_capture_retire_clock
);
  wire second_clk_hs_term;
  wire second_clk_hs_ena;
  wire [1:0] second_data_fifo_rd;
  wire [1:0] second_data_rst;
  wire [1:0] second_data_hs_term;
  wire [1:0] second_data_hs_ena;
  wire second_init_done;
  wire [15:0] second_receiver_status;
  wire second_inner_fv_valid;
  wire [32:0] second_inner_fv_data;
  wire second_hier_inner_end;
  wire second_lwc_out_sof;
  wire second_lwc_out_valid;
  wire [15:0] second_lwc_out_data;
  wire second_lwc_protocol_error;
  wire second_lftag_result_valid;
  wire [31:0] second_lftag_result_data;
  wire second_lftag_result_last;
  wire second_lftag_busy;
  wire [5:0] second_lftag_fifo_level;
  wire second_lftag_candidate_start;
  wire second_lftag_candidate_done;
  wire [9:0] second_lftag_candidate_cycles;
  wire second_lftag_fifo_drop_sticky;
  wire second_lftag_protocol_error_sticky;
  wire second_threshold_protocol_error;
  wire second_threshold_buffer_overrun;
  wire second_threshold_frontend_overrun;
  wire second_hierarchy_overrun;
  wire [7:0] second_hier_child_sort_key;
  wire [63:0] second_threshold_stream_hash;
  wire [31:0] second_threshold_stream_pairs;
  wire [31:0] second_outer_table_overflows;
  wire [31:0] second_inner_table_overflows;
  wire second_tap_overflow;
  wire [63:0] second_lwc_group_schedule_hash;
  wire [31:0] second_lwc_group_schedule_count;
  wire second_lwc_group_schedule_valid;
  wire [3:0] second_lwc_group_schedule_band;
  wire [11:0] second_lwc_raw_valid;
  wire second_receiver_protocol_error;
  wire second_receiver_marker_error;
  wire [9:0] second_camera_rows_received;
  wire [9:0] second_core_rows_started;
  wire [9:0] second_core_rows_completed;
  wire second_camera_line_done;
  wire second_core_row_start;
  wire second_core_pair_valid;
  wire second_core_frame_active;
  wire second_camera_fault;
  wire [31:0] second_current_frame_id;
  wire [31:0] second_receiver_pairs;
  wire [31:0] second_capture_sof_clock;
  wire [31:0] second_capture_eof_clock;

  wire [1:0] candidate_valid, candidate_group_end, candidate_centroid_valid;
  wire [1:0][14:0] candidate_x_q4;
  wire [1:0][13:0] candidate_y_q4;
  wire [1:0][7:0] candidate_log_mass_q5_3, candidate_frame_seq;
  wire [1:0][1:0] candidate_scale;
  wire [1:0] processing_reset;
  wire [1:0] shared_valid, shared_last, shared_busy;
  wire [31:0] shared_data;
  wire [1:0][5:0] shared_fifo_level;
  wire [1:0] shared_candidate_start, shared_candidate_done;
  wire [1:0][9:0] shared_candidate_cycles;
  wire [1:0] shared_fifo_drop, shared_protocol_error;

  wire config_read_valid;
  wire [3:0] config_read_address;
  logic config_response_valid;
  logic [31:0] config_read_data;

  // Match the production configuration bank's one-clock loaded-bank response,
  // retaining the same calibration as the standalone camera reference.
  function automatic [31:0] config_word(input [3:0] address);
    case (address)
      0: config_word = 32'h0001_0000;
      1: config_word = 32'h147a_e148;
      2, 4, 6: config_word = 32'd0;
      3: config_word = 32'd43;
      5: config_word = 32'h0400_0000;
      7: config_word = {8'b0, 16'd16, 8'd19};
      8, 9, 12, 13: config_word = 32'h0022_2222;
      10, 14: config_word = 32'h0140_0000;
      11, 15: config_word = 32'h00f0_0000;
    endcase
  endfunction

  always_ff @(posedge core_clk) begin
    if (!reset_n) begin
      config_response_valid <= 1'b0;
      config_read_data <= '0;
    end else begin
      config_response_valid <= config_read_valid;
      if (config_read_valid) begin
        config_read_data <= config_word(config_read_address);
        if (config_read_address[3] && config_read_address[2] != shared_decoder.owner)
          $fatal(1, "configuration request selected the wrong camera");
      end
    end
  end

  SharedLfTagDecoder #(
    .EXPORT_ACCEPTED_CENTROIDS(1'b1), .EXTERNAL_CONFIG(1'b1)
  ) shared_decoder (
    .config_read_valid, .config_read_address, .config_read_ready(1'b1),
    .config_response_valid, .config_read_data,
    .clk(core_clk), .rst(!reset_n), .camera_reset(processing_reset),
    .in_valid(candidate_valid), .in_group_end(candidate_group_end),
    .in_x_q4(candidate_x_q4), .in_y_q4(candidate_y_q4),
    .in_log_mass_q5_3(candidate_log_mass_q5_3),
    .in_centroid_valid(candidate_centroid_valid), .in_scale(candidate_scale),
    .in_frame_seq(candidate_frame_seq),
    // Match the fixed calibration in pipeline_ov9281_camera.
    .cfg_inv_fx_q2_30({2{32'h0022_2222}}),
    .cfg_inv_fy_q2_30({2{32'h0022_2222}}),
    .cfg_cx_q16_16({2{32'h0140_0000}}),
    .cfg_cy_q16_16({2{32'h00f0_0000}}),
    .cfg_printed_width_q16_16({2{32'h0001_0000}}),
    .cfg_mean_residual_limit_q16({2{16'd1024}}),
    .cfg_max_mass_spread({2{8'd19}}),
    .cfg_min_baseline_q4({2{16'd16}}),
    .centroid_valid(), .centroid_ready(2'b11), .centroid_data(), .centroid_last(),
    .out_valid(shared_valid), .out_ready(2'b11), .out_data(shared_data),
    .out_last(shared_last), .busy(shared_busy), .fifo_level(shared_fifo_level),
    .candidate_start(shared_candidate_start), .candidate_done(shared_candidate_done),
    .candidate_cycles(shared_candidate_cycles), .clear_diagnostics(2'b00),
    .fifo_drop_sticky(shared_fifo_drop), .protocol_error_sticky(shared_protocol_error),
    .debug_status(), .debug_protocol(), .debug_drop()
  );

  pipeline_ov9281_camera #(.EXTERNAL_LFTAG(1'b1)) camera0 (
    .lftag_in_valid(candidate_valid[0]), .lftag_in_group_end(candidate_group_end[0]),
    .lftag_in_x_q4(candidate_x_q4[0]), .lftag_in_y_q4(candidate_y_q4[0]),
    .lftag_in_log_mass_q5_3(candidate_log_mass_q5_3[0]),
    .lftag_in_centroid_valid(candidate_centroid_valid[0]),
    .lftag_in_scale(candidate_scale[0]),
    .lftag_in_frame_seq(candidate_frame_seq[0]), .lftag_processing_reset(processing_reset[0]),
    .external_lftag_result_valid(shared_valid[0]), .external_lftag_result_data(shared_data),
    .external_lftag_result_last(shared_last[0]), .external_lftag_busy(shared_busy[0]),
    .external_lftag_fifo_level(shared_fifo_level[0]),
    .external_lftag_candidate_start(shared_candidate_start[0]),
    .external_lftag_candidate_done(shared_candidate_done[0]),
    .external_lftag_candidate_cycles(shared_candidate_cycles[0]),
    .external_lftag_fifo_drop_sticky(shared_fifo_drop[0]),
    .external_lftag_protocol_error_sticky(shared_protocol_error[0]),
    .reset_n(reset_n), .byte_clk(byte_clk), .core_clk(core_clk),
    .status_clear(status_clear), .clk_lp_p(clk_lp_p), .clk_lp_n(clk_lp_n),
    .clk_hs_term(clk_hs_term), .clk_hs_ena(clk_hs_ena), .data_lp_p(data_lp_p),
    .data_lp_n(data_lp_n), .data0_hs_in(data0_hs_in), .data1_hs_in(data1_hs_in),
    .data_fifo_empty(data_fifo_empty), .data_fifo_rd(data_fifo_rd), .data_rst(data_rst),
    .data_hs_term(data_hs_term), .data_hs_ena(data_hs_ena), .init_done(init_done),
    .receiver_status(receiver_status), .inner_fv_valid(inner_fv_valid), .inner_fv_data(inner_fv_data),
    .hier_inner_end(hier_inner_end), .lwc_out_sof(lwc_out_sof), .lwc_out_valid(lwc_out_valid),
    .lwc_out_data(lwc_out_data), .lwc_protocol_error(lwc_protocol_error), .lftag_result_valid(lftag_result_valid),
    .lftag_result_data(lftag_result_data), .lftag_result_last(lftag_result_last), .lftag_busy(lftag_busy),
    .lftag_fifo_level(lftag_fifo_level), .lftag_candidate_start(lftag_candidate_start), .lftag_candidate_done(lftag_candidate_done),
    .lftag_candidate_cycles(lftag_candidate_cycles), .lftag_fifo_drop_sticky(lftag_fifo_drop_sticky), .lftag_protocol_error_sticky(lftag_protocol_error_sticky),
    .threshold_protocol_error(threshold_protocol_error), .threshold_buffer_overrun(threshold_buffer_overrun), .threshold_frontend_overrun(threshold_frontend_overrun),
    .hierarchy_overrun(hierarchy_overrun), .hier_child_sort_key(hier_child_sort_key),
    .threshold_stream_hash(threshold_stream_hash), .threshold_stream_pairs(threshold_stream_pairs), .outer_table_overflows(outer_table_overflows),
    .inner_table_overflows(inner_table_overflows), .tap_overflow(tap_overflow), .lwc_group_schedule_hash(lwc_group_schedule_hash),
    .lwc_group_schedule_count(lwc_group_schedule_count), .lwc_group_schedule_valid(lwc_group_schedule_valid), .lwc_group_schedule_band(lwc_group_schedule_band),
    .lwc_raw_valid(lwc_raw_valid), .receiver_protocol_error(receiver_protocol_error), .receiver_marker_error(receiver_marker_error),
    .camera_rows_received(camera_rows_received), .core_rows_started(core_rows_started), .core_rows_completed(core_rows_completed),
    .camera_line_done(camera_line_done), .core_row_start(core_row_start), .core_pair_valid(core_pair_valid),
    .core_frame_active(core_frame_active), .camera_fault(camera_fault), .camera_idle(camera_idle),
    .frames_completed(frames_completed), .current_frame_id(current_frame_id), .receiver_pairs(receiver_pairs),
    .capture_sof_clock(capture_sof_clock), .capture_eof_clock(capture_eof_clock), .capture_retire_clock(capture_retire_clock)
  );

  pipeline_ov9281_camera #(.EXTERNAL_LFTAG(1'b1)) camera1 (
    .lftag_in_valid(candidate_valid[1]), .lftag_in_group_end(candidate_group_end[1]),
    .lftag_in_x_q4(candidate_x_q4[1]), .lftag_in_y_q4(candidate_y_q4[1]),
    .lftag_in_log_mass_q5_3(candidate_log_mass_q5_3[1]),
    .lftag_in_centroid_valid(candidate_centroid_valid[1]),
    .lftag_in_scale(candidate_scale[1]),
    .lftag_in_frame_seq(candidate_frame_seq[1]), .lftag_processing_reset(processing_reset[1]),
    .external_lftag_result_valid(shared_valid[1]), .external_lftag_result_data(shared_data),
    .external_lftag_result_last(shared_last[1]), .external_lftag_busy(shared_busy[1]),
    .external_lftag_fifo_level(shared_fifo_level[1]),
    .external_lftag_candidate_start(shared_candidate_start[1]),
    .external_lftag_candidate_done(shared_candidate_done[1]),
    .external_lftag_candidate_cycles(shared_candidate_cycles[1]),
    .external_lftag_fifo_drop_sticky(shared_fifo_drop[1]),
    .external_lftag_protocol_error_sticky(shared_protocol_error[1]),
    .reset_n(reset_n), .byte_clk(byte_clk), .core_clk(core_clk),
    .status_clear(status_clear), .clk_lp_p(clk_lp_p), .clk_lp_n(clk_lp_n),
    .clk_hs_term(second_clk_hs_term), .clk_hs_ena(second_clk_hs_ena), .data_lp_p(data_lp_p),
    .data_lp_n(data_lp_n), .data0_hs_in(data0_hs_in), .data1_hs_in(data1_hs_in),
    .data_fifo_empty(data_fifo_empty), .data_fifo_rd(second_data_fifo_rd), .data_rst(second_data_rst),
    .data_hs_term(second_data_hs_term), .data_hs_ena(second_data_hs_ena), .init_done(second_init_done),
    .receiver_status(second_receiver_status), .inner_fv_valid(second_inner_fv_valid), .inner_fv_data(second_inner_fv_data),
    .hier_inner_end(second_hier_inner_end), .lwc_out_sof(second_lwc_out_sof), .lwc_out_valid(second_lwc_out_valid),
    .lwc_out_data(second_lwc_out_data), .lwc_protocol_error(second_lwc_protocol_error), .lftag_result_valid(second_lftag_result_valid),
    .lftag_result_data(second_lftag_result_data), .lftag_result_last(second_lftag_result_last), .lftag_busy(second_lftag_busy),
    .lftag_fifo_level(second_lftag_fifo_level), .lftag_candidate_start(second_lftag_candidate_start), .lftag_candidate_done(second_lftag_candidate_done),
    .lftag_candidate_cycles(second_lftag_candidate_cycles), .lftag_fifo_drop_sticky(second_lftag_fifo_drop_sticky), .lftag_protocol_error_sticky(second_lftag_protocol_error_sticky),
    .threshold_protocol_error(second_threshold_protocol_error), .threshold_buffer_overrun(second_threshold_buffer_overrun), .threshold_frontend_overrun(second_threshold_frontend_overrun),
    .hierarchy_overrun(second_hierarchy_overrun), .hier_child_sort_key(second_hier_child_sort_key),
    .threshold_stream_hash(second_threshold_stream_hash), .threshold_stream_pairs(second_threshold_stream_pairs), .outer_table_overflows(second_outer_table_overflows),
    .inner_table_overflows(second_inner_table_overflows), .tap_overflow(second_tap_overflow), .lwc_group_schedule_hash(second_lwc_group_schedule_hash),
    .lwc_group_schedule_count(second_lwc_group_schedule_count), .lwc_group_schedule_valid(second_lwc_group_schedule_valid), .lwc_group_schedule_band(second_lwc_group_schedule_band),
    .lwc_raw_valid(second_lwc_raw_valid), .receiver_protocol_error(second_receiver_protocol_error), .receiver_marker_error(second_receiver_marker_error),
    .camera_rows_received(second_camera_rows_received), .core_rows_started(second_core_rows_started), .core_rows_completed(second_core_rows_completed),
    .camera_line_done(second_camera_line_done), .core_row_start(second_core_row_start), .core_pair_valid(second_core_pair_valid),
    .core_frame_active(second_core_frame_active), .camera_fault(second_camera_fault), .camera_idle(second_camera_idle),
    .frames_completed(second_frames_completed), .current_frame_id(second_current_frame_id), .receiver_pairs(second_receiver_pairs),
    .capture_sof_clock(second_capture_sof_clock), .capture_eof_clock(second_capture_eof_clock), .capture_retire_clock(second_capture_retire_clock)
  );

  logic [6:0] core_compare_error;
  logic byte_compare_error;
  assign dual_compare_error = {byte_compare_error, core_compare_error};

  // The shared engine serializes candidates, so neither result timing nor
  // retirement timing is equal between cameras. Compare accepted words by
  // source-local ordinal, including the full frame ID.
  // These test-only queues exceed the 32 queued + one active candidate bound.
  wire [1:0] result_valid = {second_lftag_result_valid, lftag_result_valid};
  wire [1:0] result_last = {second_lftag_result_last, lftag_result_last};
  wire [1:0][31:0] result_data = {second_lftag_result_data, lftag_result_data};
  wire [1:0][31:0] frame_ids = {second_current_frame_id, current_frame_id};
  wire [1:0] camera_idles = {second_camera_idle, camera_idle};
  logic [64:0] result_words [0:1][0:1023];
  logic [39:0] group_frames [0:1][0:63];
  logic [31:0] group_count [0:1];
  logic [31:0] start_count [0:1];
  logic [31:0] done_count [0:1];
  logic [31:0] packet_count [0:1];
  logic [31:0] word_count [0:1];
  logic [31:0] packet_frame [0:1];
  logic [7:0] packet_private_frame [0:1];
  logic [4:0] packet_index [0:1];
  logic [4:0] packet_last_index [0:1];
  logic [31:0] compared_words;
  logic solver_compare_error;

  always_ff @(posedge core_clk) begin
    if (!reset_n) begin
      compared_words <= '0;
      solver_compare_error <= 1'b0;
      for (integer camera = 0; camera < 2; camera = camera + 1) begin
        group_count[camera] <= '0;
        start_count[camera] <= '0;
        done_count[camera] <= '0;
        packet_count[camera] <= '0;
        word_count[camera] <= '0;
        packet_frame[camera] <= '0;
        packet_private_frame[camera] <= '0;
        packet_index[camera] <= '0;
        packet_last_index[camera] <= 5'd7;
      end
    end else begin
      if ((|shared_fifo_drop) || (|shared_protocol_error) ||
          (shared_valid == 2'b11) || (shared_candidate_start == 2'b11) ||
          (shared_candidate_done == 2'b11))
        solver_compare_error <= 1'b1;
      for (integer camera = 0; camera < 2; camera = camera + 1) begin
        if (candidate_valid[camera] && candidate_group_end[camera] &&
            !processing_reset[camera]) begin
          if (group_count[camera] - packet_count[camera] >= 64)
            solver_compare_error <= 1'b1;
          group_frames[camera][group_count[camera][5:0]] <=
              {frame_ids[camera], candidate_frame_seq[camera]};
          group_count[camera] <= group_count[camera] + 1'b1;
        end
        if (shared_candidate_start[camera]) begin
          if (start_count[camera] >= group_count[camera] || !shared_busy[camera])
            solver_compare_error <= 1'b1;
          start_count[camera] <= start_count[camera] + 1'b1;
        end
        if (shared_candidate_done[camera]) begin
          if (done_count[camera] >= start_count[camera] || !shared_busy[camera] ||
              shared_candidate_cycles[camera] == 0 || shared_candidate_cycles[camera] > 1000)
            solver_compare_error <= 1'b1;
          done_count[camera] <= done_count[camera] + 1'b1;
        end
        if (result_last[camera] && !result_valid[camera])
          solver_compare_error <= 1'b1;
        // Both adapters keep result_ready high, so every valid word is accepted.
        if (result_valid[camera]) begin
          if (!shared_busy[camera] || word_count[camera] - compared_words >= 1024)
            solver_compare_error <= 1'b1;
          result_words[camera][word_count[camera][9:0]] <=
              {frame_ids[camera], result_last[camera], result_data[camera]};
          word_count[camera] <= word_count[camera] + 1'b1;
          if (packet_index[camera] == 0) begin
            if (packet_count[camera] >= group_count[camera] ||
                (packet_count[camera] >= done_count[camera] && !shared_candidate_done[camera]) ||
                frame_ids[camera] != group_frames[camera][packet_count[camera][5:0]][39:8])
              solver_compare_error <= 1'b1;
            packet_frame[camera] <= frame_ids[camera];
            packet_private_frame[camera] <= group_frames[camera][packet_count[camera][5:0]][7:0];
          end else if (frame_ids[camera] != packet_frame[camera]) begin
            solver_compare_error <= 1'b1;
          end
          if (packet_index[camera] == 1) begin
            if (result_data[camera][27:20] != packet_frame[camera][7:0] ||
                shared_data[27:20] != packet_private_frame[camera])
              solver_compare_error <= 1'b1;
            packet_last_index[camera] <= result_data[camera][31] ? 5'd21 : 5'd7;
          end
          if (result_last[camera] !=
              ((packet_index[camera] >= 2) && (packet_index[camera] == packet_last_index[camera])))
            solver_compare_error <= 1'b1;
          if (packet_index[camera] > 21)
            solver_compare_error <= 1'b1;
          if (result_last[camera]) begin
            packet_index[camera] <= '0;
            packet_count[camera] <= packet_count[camera] + 1'b1;
          end else begin
            packet_index[camera] <= packet_index[camera] + 1'b1;
          end
        end
        // A camera may become idle while the other camera still owns the
        // solver, but only after all of its own groups and packets finish.
        if (camera_idles[camera] &&
            (shared_busy[camera] || packet_index[camera] != 0 ||
             group_count[camera] != start_count[camera] ||
             start_count[camera] != done_count[camera] ||
             done_count[camera] != packet_count[camera]))
          solver_compare_error <= 1'b1;
      end
      if ((compared_words < word_count[0]) && (compared_words < word_count[1])) begin
        if (result_words[0][compared_words[9:0]] != result_words[1][compared_words[9:0]])
          solver_compare_error <= 1'b1;
        compared_words <= compared_words + 1'b1;
      end
      if ((&camera_idles) && (word_count[0] != word_count[1]))
        solver_compare_error <= 1'b1;
    end
  end

  // Error bits: receiver, compressed image, children, solver, diagnostics,
  // frame admission/retirement, ISP, byte-clock link control (bit 7).
  always_ff @(posedge core_clk) begin
    if (!reset_n) begin
      core_compare_error <= '0;
      dual_core_compare_cycles <= '0;
    end else begin
      dual_core_compare_cycles <= dual_core_compare_cycles + 1'b1;
      if ((clk_hs_term != second_clk_hs_term) ||
          (clk_hs_ena != second_clk_hs_ena) ||
          (data_fifo_rd != second_data_fifo_rd) ||
          (data_rst != second_data_rst) ||
          (data_hs_term != second_data_hs_term) ||
          (data_hs_ena != second_data_hs_ena) ||
          (init_done != second_init_done) ||
          (receiver_status != second_receiver_status) ||
          (receiver_protocol_error != second_receiver_protocol_error) ||
          (receiver_marker_error != second_receiver_marker_error) ||
          (camera_rows_received != second_camera_rows_received) ||
          (core_rows_started != second_core_rows_started) ||
          (core_rows_completed != second_core_rows_completed) ||
          (camera_line_done != second_camera_line_done) ||
          (core_row_start != second_core_row_start) ||
          (core_pair_valid != second_core_pair_valid) ||
          (receiver_pairs != second_receiver_pairs) ||
          (camera0.receiver_pixel_valid && ({camera0.receiver_pixel_data, camera0.receiver_pixel_sof, camera0.receiver_pixel_sol, camera0.receiver_pixel_eol, camera0.receiver_pixel_eof} != {camera1.receiver_pixel_data, camera1.receiver_pixel_sof, camera1.receiver_pixel_sol, camera1.receiver_pixel_eol, camera1.receiver_pixel_eof})))
        core_compare_error[0] <= 1'b1;
      if ((lwc_out_sof != second_lwc_out_sof) ||
          (lwc_out_valid != second_lwc_out_valid) ||
          (lwc_protocol_error != second_lwc_protocol_error) ||
          (lwc_out_valid && ({lwc_out_data, camera0.dut.lwc_out_padding} !=
                            {second_lwc_out_data, camera1.dut.lwc_out_padding})))
        core_compare_error[1] <= 1'b1;
      if ((inner_fv_valid != second_inner_fv_valid) ||
          (hier_inner_end != second_hier_inner_end) ||
          (inner_fv_valid && ({inner_fv_data, hier_child_sort_key} != {second_inner_fv_data, second_hier_child_sort_key})))
        core_compare_error[2] <= 1'b1;
      if (solver_compare_error || lftag_fifo_drop_sticky || second_lftag_fifo_drop_sticky ||
          lftag_protocol_error_sticky || second_lftag_protocol_error_sticky)
        core_compare_error[3] <= 1'b1;
      if ((threshold_protocol_error != second_threshold_protocol_error) ||
          (threshold_buffer_overrun != second_threshold_buffer_overrun) ||
          (threshold_frontend_overrun != second_threshold_frontend_overrun) ||
          (hierarchy_overrun != second_hierarchy_overrun) ||
          (threshold_stream_hash != second_threshold_stream_hash) ||
          (threshold_stream_pairs != second_threshold_stream_pairs) ||
          (outer_table_overflows != second_outer_table_overflows) ||
          (inner_table_overflows != second_inner_table_overflows) ||
          (tap_overflow != second_tap_overflow) ||
          (lwc_group_schedule_hash != second_lwc_group_schedule_hash) ||
          (lwc_group_schedule_count != second_lwc_group_schedule_count) ||
          (lwc_group_schedule_valid != second_lwc_group_schedule_valid) ||
          (lwc_raw_valid != second_lwc_raw_valid) ||
          (lwc_group_schedule_valid && (lwc_group_schedule_band != second_lwc_group_schedule_band)))
        core_compare_error[4] <= 1'b1;
      if (camera_fault || second_camera_fault ||
          ((camera_idle && second_camera_idle) && (frames_completed != second_frames_completed)) ||
          (current_frame_id != second_current_frame_id) ||
          (capture_sof_clock != second_capture_sof_clock) ||
          (capture_eof_clock != second_capture_eof_clock))
        core_compare_error[5] <= 1'b1;
      if (({camera0.dut.isp_in_ready, camera0.dut.isp_out_valid} != {camera1.dut.isp_in_ready, camera1.dut.isp_out_valid}) ||
          (camera0.dut.isp_out_valid && ({camera0.dut.isp_out_pixel0, camera0.dut.isp_out_pixel1, camera0.dut.isp_out_sof, camera0.dut.isp_out_eol} != {camera1.dut.isp_out_pixel0, camera1.dut.isp_out_pixel1, camera1.dut.isp_out_sof, camera1.dut.isp_out_eol})))
        core_compare_error[6] <= 1'b1;
    end
  end

  // The source advances from camera 0 reads. Check camera 1 consumes exactly
  // the same lane words on every recovered byte edge, not just core edges.
  always_ff @(posedge byte_clk) begin
    if (!reset_n) begin
      byte_compare_error <= 1'b0;
      dual_byte_compare_cycles <= '0;
    end else begin
      dual_byte_compare_cycles <= dual_byte_compare_cycles + 1'b1;
      if ({data_fifo_rd, data_rst, data_hs_ena, data_hs_term} !=
          {second_data_fifo_rd, second_data_rst, second_data_hs_ena, second_data_hs_term})
        byte_compare_error <= 1'b1;
    end
  end
endmodule
`default_nettype wire
