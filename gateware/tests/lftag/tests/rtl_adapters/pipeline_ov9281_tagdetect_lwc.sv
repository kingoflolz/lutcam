`default_nettype none

// Simulation-only composition of the production camera architecture. Receiver
// valid drives TagDetectLwc directly; frame-valid is held from SOF through EOF
// exactly as in the Ti60 shell. There is intentionally no row or pixel buffer
// outside the receiver's one-RAM CDC FIFO. processing_reset is a test-only
// per-frame reset: reset_n alone owns the receiver reset.
module pipeline_ov9281_tagdetect_lwc (
  input  logic        reset_n,
  input  logic        processing_reset,
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
  output logic        pipeline_busy,
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
  output logic        core_frame_active
);
  logic [19:0] receiver_pixel_data;
  logic receiver_pixel_valid;
  logic receiver_pixel_sof;
  logic receiver_pixel_sol;
  logic receiver_pixel_eol;
  logic receiver_pixel_eof;
  logic receiver_frame_active;

  csi2_rx #(
    .INIT_GUARD_CYCLES(4),
    .LPX_CYCLES(2),
    .SOT_SETTLE_CYCLES(2),
    .CLOCK_TIMEOUT_CYCLES(77),
    .PIXEL_FIFO_DEPTH(512),
    .PACK_MSB8_MARKERS(1'b1)
  ) receiver (
    .reset_n,
    .ctrl_clk(core_clk),
    .byte_clk,
    .pixel_clk(core_clk),
    .status_clear,
    .clk_lp_p,
    .clk_lp_n,
    .clk_hs_term,
    .clk_hs_ena,
    .data_lp_p,
    .data_lp_n,
    .data0_hs_in,
    .data1_hs_in,
    .data_fifo_empty,
    .data_fifo_rd,
    .data_rst,
    .data_hs_term,
    .data_hs_ena,
    .init_done,
    .status(receiver_status),
    .pixel_data(receiver_pixel_data),
    .pixel_valid(receiver_pixel_valid),
    .pixel_sof(receiver_pixel_sof),
    .pixel_sol(receiver_pixel_sol),
    .pixel_eol(receiver_pixel_eol),
    .pixel_eof(receiver_pixel_eof)
  );

  wire receiver_fatal_status = |(receiver_status & 16'h3fba);
  wire core_in_vref = init_done && !receiver_fatal_status &&
      (receiver_frame_active ||
       (receiver_pixel_valid && receiver_pixel_sof));
  wire core_in_href = receiver_pixel_valid;
  wire [7:0] core_in_d0 = receiver_pixel_data[9:2];
  wire [7:0] core_in_d1 = receiver_pixel_data[19:12];

  assign camera_line_done = receiver_pixel_valid && receiver_pixel_eol;
  assign core_row_start = receiver_pixel_valid && receiver_pixel_sol;
  assign core_pair_valid = core_in_href;
  assign core_frame_active = core_in_vref;

  always_ff @(posedge core_clk or negedge reset_n) begin
    if (!reset_n || processing_reset) begin
      receiver_frame_active <= 1'b0;
      receiver_protocol_error <= 1'b0;
      receiver_marker_error <= 1'b0;
      camera_rows_received <= '0;
      core_rows_started <= '0;
      core_rows_completed <= '0;
    end else begin
      if (receiver_status != 16'h0000)
        receiver_protocol_error <= 1'b1;
      if (receiver_fatal_status)
        receiver_frame_active <= 1'b0;
      else begin
        if (receiver_pixel_valid && receiver_pixel_sof)
          receiver_frame_active <= 1'b1;
        if (receiver_pixel_valid && receiver_pixel_eof)
          receiver_frame_active <= 1'b0;
      end

      if (receiver_pixel_valid && receiver_pixel_sol) begin
        camera_rows_received <= camera_rows_received + 1'b1;
        core_rows_started <= core_rows_started + 1'b1;
      end
      if (receiver_pixel_valid && receiver_pixel_eol)
        core_rows_completed <= core_rows_completed + 1'b1;
      if (receiver_pixel_valid &&
          ((receiver_pixel_sof && !receiver_pixel_sol) ||
           (receiver_pixel_eof && !receiver_pixel_eol)))
        receiver_marker_error <= 1'b1;
    end
  end

  pipeline_tagdetect_lwc core (
    .clk(core_clk),
    .rst(~reset_n | processing_reset),
    .in_vref(core_in_vref),
    .in_href(core_in_href),
    .in_d0(core_in_d0),
    .in_d1(core_in_d1),
    .inner_fv_valid,
    .inner_fv_data,
    .hier_inner_end,
    .lwc_out_sof,
    .lwc_out_valid,
    .lwc_out_data,
    .lwc_protocol_error,
    .pipeline_busy,
    .lftag_result_valid,
    .lftag_result_data,
    .lftag_result_last,
    .lftag_busy,
    .lftag_fifo_level,
    .lftag_candidate_start,
    .lftag_candidate_done,
    .lftag_candidate_cycles,
    .lftag_fifo_drop_sticky,
    .lftag_protocol_error_sticky,
    .threshold_protocol_error,
    .threshold_buffer_overrun,
    .threshold_frontend_overrun,
    .hierarchy_overrun,
    .hier_child_sort_key,
    .threshold_stream_hash,
    .threshold_stream_pairs,
    .outer_table_overflows,
    .inner_table_overflows,
    .tap_overflow,
    .lwc_group_schedule_hash,
    .lwc_group_schedule_count,
    .lwc_group_schedule_valid,
    .lwc_group_schedule_band,
    .lwc_raw_valid
  );
endmodule

`default_nettype wire
