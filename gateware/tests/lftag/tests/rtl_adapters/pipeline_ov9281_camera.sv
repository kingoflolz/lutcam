`default_nettype none

// Simulation-only actual CSI RAW10 receiver -> MonoIsp -> CameraPipeline.
// The receiver uses the production 64-entry full-RAW10 FIFO.
// The ISP is started through its real Wishbone interface in bypass mode.
module pipeline_ov9281_camera #(
  parameter bit EXTERNAL_LFTAG = 1'b0
) (
  output wire lftag_in_valid,
  output wire lftag_in_group_end,
  output wire [14:0] lftag_in_x_q4,
  output wire [13:0] lftag_in_y_q4,
  output wire [7:0] lftag_in_log_mass_q5_3,
  output wire lftag_in_centroid_valid,
  output wire [1:0] lftag_in_scale,
  output wire [7:0] lftag_in_frame_seq,
  output wire lftag_processing_reset,
  input wire external_lftag_result_valid,
  input wire [31:0] external_lftag_result_data,
  input wire external_lftag_result_last,
  input wire external_lftag_busy,
  input wire [5:0] external_lftag_fifo_level,
  input wire external_lftag_candidate_start,
  input wire external_lftag_candidate_done,
  input wire [9:0] external_lftag_candidate_cycles,
  input wire external_lftag_fifo_drop_sticky,
  input wire external_lftag_protocol_error_sticky,
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
  output logic [31:0] capture_retire_clock
);
  logic [19:0] receiver_pixel_data;
  logic receiver_pixel_valid;
  logic receiver_pixel_sof;
  logic receiver_pixel_sol;
  logic receiver_pixel_eol;
  logic receiver_pixel_eof;

  // Match fpga/litex/lutcam/camera.py, including every control-clock guard.
  csi2_rx #(
    .INIT_GUARD_CYCLES(12000),
    .LPX_CYCLES(6),
    .SOT_SETTLE_CYCLES(11),
    .CLOCK_TIMEOUT_CYCLES(77),
    .PIXEL_FIFO_DEPTH(64),
    .PACK_MSB8_MARKERS(1'b0)
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

  logic [31:0] core_clock_count;
  always_ff @(posedge core_clk) begin
    if (!reset_n) begin
      core_clock_count <= '0;
      capture_sof_clock <= '0;
      capture_eof_clock <= '0;
      capture_retire_clock <= '0;
    end else begin
      core_clock_count <= core_clock_count + 1'b1;
      if (dut.begin_frame) capture_sof_clock <= core_clock_count;
      if (dut.accept_pixel && receiver_pixel_eof) capture_eof_clock <= core_clock_count;
      if (dut.retire_frame) capture_retire_clock <= core_clock_count;
    end
  end
  wire config_ack, config_error;
  logic configured;
  always_ff @(posedge core_clk) begin
    if (!reset_n) configured <= 1'b0;
    else if (config_ack) configured <= 1'b1;
  end
  assign camera_line_done = receiver_pixel_valid && receiver_pixel_eol;
  assign core_row_start = receiver_pixel_valid && receiver_pixel_sol;
  assign core_pair_valid = receiver_pixel_valid;
  assign core_frame_active = dut.core_frame_active;
  always_ff @(posedge core_clk) begin
    if (!reset_n) begin
      receiver_protocol_error <= 1'b0;
      receiver_marker_error <= 1'b0;
      camera_rows_received <= '0;
      core_rows_started <= '0;
      core_rows_completed <= '0;
      receiver_pairs <= '0;
    end else begin
      if ((receiver_status != 16'd0) || config_error)
        receiver_protocol_error <= 1'b1;
      if (receiver_pixel_valid) begin
        receiver_pairs <= receiver_pixel_sof ? 32'd1 : receiver_pairs + 1'b1;
        if (receiver_pixel_sol) begin
          camera_rows_received <= receiver_pixel_sof ? 10'd1 : camera_rows_received + 1'b1;
          core_rows_started <= receiver_pixel_sof ? 10'd1 : core_rows_started + 1'b1;
        end
        if (receiver_pixel_sof) core_rows_completed <= '0;
        else if (receiver_pixel_eol) core_rows_completed <= core_rows_completed + 1'b1;
        if ((receiver_pixel_sof && !receiver_pixel_sol) ||
            (receiver_pixel_eof && !receiver_pixel_eol))
          receiver_marker_error <= 1'b1;
      end
    end
  end

  CameraPipeline #(
    .EXTERNAL_LFTAG(EXTERNAL_LFTAG),
    .GROUP_FAMILY_SCHEDULE_BIN("../../rtl/lwc/raw_group_family_schedule.bin"),
    .CENTROID_RECIPROCAL_SEGMENT_BIN("../../rtl/lrcca/centroid_reciprocal_segments_q17.bin"),
    .CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN(
      "../../rtl/lrcca/centroid_reciprocal_midpoint_reflection_q17.bin")
  ) dut (
    .lftag_in_valid,
    .lftag_in_group_end,
    .lftag_in_x_q4,
    .lftag_in_y_q4,
    .lftag_in_log_mass_q5_3,
    .lftag_in_centroid_valid,
    .lftag_in_scale,
    .lftag_in_frame_seq,
    .lftag_processing_reset,
    .external_lftag_result_valid,
    .external_lftag_result_data,
    .external_lftag_result_last,
    .external_lftag_busy,
    .external_lftag_fifo_level,
    .external_lftag_candidate_start,
    .external_lftag_candidate_done,
    .external_lftag_candidate_cycles,
    .external_lftag_fifo_drop_sticky,
    .external_lftag_protocol_error_sticky,
    .clk(core_clk), .rst(!reset_n),
    .pixel_data(receiver_pixel_data), .pixel_valid(receiver_pixel_valid),
    .pixel_sof(receiver_pixel_sof), .pixel_eol(receiver_pixel_eol),
    .pixel_eof(receiver_pixel_eof), .receiver_fault(|receiver_status),
    .run_request(configured), .capture_permit(1'b1), .clear_diagnostics(1'b0),
    .pair_abort(1'b0), .capture_frame_id(32'd0),
    .active(), .idle(camera_idle), .fault_sticky(camera_fault),
    .admission_ready(), .frame_admitted(), .frame_retired(), .local_fault(),
    .frame_id(current_frame_id), .frames_started(), .frames_completed,
    .ingress_high_water(),
    .wb_cyc_i(!configured && !config_ack && reset_n), .wb_stb_i(!configured && !config_ack && reset_n),
    .wb_we_i(1'b1), .wb_sel_i(4'hf), .wb_adr_i(16'h0004), .wb_dat_i(32'd1),
    .wb_dat_o(), .wb_ack_o(config_ack), .wb_err_o(config_error),
    .hist_dump_req(1'b0), .hist_dump_ready(), .hist_out_valid(),
    .hist_out_ready(1'b1), .hist_out_data(),
    .inner_fv_valid, .inner_fv_data, .hier_inner_end,
    .lwc_out_sof, .lwc_out_valid, .lwc_out_data, .lwc_out_padding(),
    .lwc_protocol_error,
    .lftag_cfg_inv_fx_q2_30(32'h0022_2222),
    .lftag_cfg_inv_fy_q2_30(32'h0022_2222),
    .lftag_cfg_cx_q16_16(32'sh0140_0000),
    .lftag_cfg_cy_q16_16(32'sh00f0_0000),
    .lftag_cfg_printed_width_q16_16(32'h0001_0000),
    .lftag_cfg_mean_residual_limit_q16(16'd1024),
    .lftag_cfg_max_mass_spread(8'd19),
    .lftag_cfg_min_baseline_q4(16'd16),
    .lftag_result_ready(1'b1),
    .lftag_clear_diagnostics(1'b0),
    .lftag_result_valid,
    .lftag_result_data,
    .lftag_result_last,
    .lftag_busy,
    .lftag_fifo_level,
    .lftag_candidate_start,
    .lftag_candidate_done,
    .lftag_candidate_cycles,
    .lftag_fifo_drop_sticky,
    .lftag_protocol_error_sticky
  );
  assign threshold_protocol_error = dut.processing.threshold_protocol_error;
  assign threshold_buffer_overrun = dut.processing.threshold_overrun;
  assign threshold_frontend_overrun =
      dut.processing.wavelet_tag_detect.threshold_frontend_overrun;
  assign hierarchy_overrun = dut.processing.wavelet_tag_detect.hierarchy_overrun;
  assign hier_child_sort_key = dut.processing.centroid_group_log_m_q5_3;
  assign tap_overflow = dut.processing.frontend_protocol_error;
  assign lwc_group_schedule_valid = dut.processing.lwc_pair_valid &&
      dut.processing.lwc_pair_ready && dut.processing.lwc_pair_group_start;
  assign lwc_group_schedule_band = dut.processing.lwc_pair_band;
  assign lwc_raw_valid = dut.processing.raw_valid;

  // FNV-1a over one tightly packed little-endian word per valid composite
  // pair: d0[7:0], d1[15:8], t0, t1, eol, row_replicated, fv0, fv1,
  // source_y[31:22]. This independently fingerprints the entire threshold
  // and composition stream before ChildArena scheduling can affect results.
  localparam logic [63:0] FNV1A_OFFSET = 64'hcbf29ce484222325;
  localparam logic [63:0] FNV1A_PRIME = 64'h00000100000001b3;
  logic [31:0] threshold_stream_word;
  // Keep an independent row-allocation counter so the regression detects the
  // production table's reserved-sentinel capacity before an invalid address
  // can be truncated or filtered by a later stage.
  logic [9:0] outer_row_allocations;

  function automatic logic [63:0] fnv1a_word(
    input logic [63:0] hash,
    input logic [31:0] word
  );
    logic [63:0] next_hash;
    begin
      next_hash = (hash ^ {56'b0, word[7:0]}) * FNV1A_PRIME;
      next_hash = (next_hash ^ {56'b0, word[15:8]}) * FNV1A_PRIME;
      next_hash = (next_hash ^ {56'b0, word[23:16]}) * FNV1A_PRIME;
      fnv1a_word = (next_hash ^ {56'b0, word[31:24]}) * FNV1A_PRIME;
    end
  endfunction

  always_comb begin
    threshold_stream_word = {
      dut.processing.wavelet_tag_detect.threshold_source_y,
      dut.processing.wavelet_tag_detect.threshold_feature_valid_d1,
      dut.processing.wavelet_tag_detect.threshold_feature_valid_d0,
      dut.processing.wavelet_tag_detect.threshold_row_replicated,
      dut.processing.wavelet_tag_detect.threshold_eol,
      dut.processing.wavelet_tag_detect.threshold_t1,
      dut.processing.wavelet_tag_detect.threshold_t0,
      dut.processing.wavelet_tag_detect.threshold_d1,
      dut.processing.wavelet_tag_detect.threshold_d0
    };
  end

  always_ff @(posedge core_clk) begin
    if (!reset_n || dut.begin_frame) begin
      threshold_stream_hash <= FNV1A_OFFSET;
      threshold_stream_pairs <= '0;
      outer_table_overflows <= '0;
      inner_table_overflows <= '0;
      outer_row_allocations <= '0;
      lwc_group_schedule_hash <= FNV1A_OFFSET;
      lwc_group_schedule_count <= '0;
    end else if (dut.processing.wavelet_tag_detect.threshold_valid) begin
      threshold_stream_hash <=
          fnv1a_word(threshold_stream_hash, threshold_stream_word);
      threshold_stream_pairs <= threshold_stream_pairs + 1'b1;
    end
    if (reset_n && !dut.begin_frame &&
        dut.processing.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s1_stage.out_valid) begin
      if (dut.processing.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s1_stage.out_pf_valid &&
          outer_row_allocations >= 10'(dut.processing.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.TABLE_DEPTH))
        outer_table_overflows <= outer_table_overflows + 1'b1;
      if (dut.processing.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s1_stage.out_row_end)
        outer_row_allocations <= '0;
      else if (dut.processing.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s1_stage.out_pf_valid)
        outer_row_allocations <= outer_row_allocations + 1'b1;
    end
    if (reset_n && !dut.begin_frame &&
        dut.processing.wavelet_tag_detect.hierarchy_core.inner_hier_2ppc.s1_stage.out_valid &&
        dut.processing.wavelet_tag_detect.hierarchy_core.inner_hier_2ppc.s1_stage.out_pf_valid &&
        dut.processing.wavelet_tag_detect.hierarchy_core.inner_hier_2ppc.s1_stage.p_issue >= 10'(dut.processing.wavelet_tag_detect.hierarchy_core.inner_hier_2ppc.TABLE_DEPTH))
      inner_table_overflows <= inner_table_overflows + 1'b1;
    if (reset_n && !dut.begin_frame && dut.processing.lwc_pair_valid && dut.processing.lwc_pair_ready &&
        dut.processing.lwc_pair_group_start) begin
      lwc_group_schedule_hash <=
          (lwc_group_schedule_hash ^ {60'b0, dut.processing.lwc_pair_band}) *
          FNV1A_PRIME;
      lwc_group_schedule_count <= lwc_group_schedule_count + 1'b1;
    end
  end
endmodule
`default_nettype wire
