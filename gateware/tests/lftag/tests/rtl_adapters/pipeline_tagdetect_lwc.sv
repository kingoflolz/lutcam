// Simulation-only adapter for the fixed-profile combined production top.
// It supplies crate-relative centroid ROM paths and exposes sticky internal
// protocol/overrun diagnostics without adding production output pins. Ordinary
// pipeline_busy covers all pending outputs.
module pipeline_tagdetect_lwc (
  input  logic clk,
  input  logic rst,
  input  logic in_vref,
  input  logic in_href,
  input  logic [7:0] in_d0,
  input  logic [7:0] in_d1,
  output logic inner_fv_valid,
  output logic [32:0] inner_fv_data,
  output logic hier_inner_end,
  output logic lwc_out_sof,
  output logic lwc_out_valid,
  output logic [15:0] lwc_out_data,
  output logic lwc_protocol_error,
  output logic pipeline_busy,
  output logic lftag_result_valid,
  output logic [31:0] lftag_result_data,
  output logic lftag_result_last,
  output logic lftag_busy,
  output logic [5:0] lftag_fifo_level,
  output logic lftag_candidate_start,
  output logic lftag_candidate_done,
  output logic [9:0] lftag_candidate_cycles,
  output logic lftag_fifo_drop_sticky,
  output logic lftag_protocol_error_sticky,
  output logic threshold_protocol_error,
  output logic threshold_buffer_overrun,
  output logic threshold_frontend_overrun,
  output logic hierarchy_overrun,
  output logic [7:0] hier_child_sort_key,
  output logic [63:0] threshold_stream_hash,
  output logic [31:0] threshold_stream_pairs,
  output logic [31:0] outer_table_overflows,
  output logic [31:0] inner_table_overflows,
  output logic tap_overflow,
  output logic [63:0] lwc_group_schedule_hash,
  output logic [31:0] lwc_group_schedule_count,
  output logic lwc_group_schedule_valid,
  output logic [3:0] lwc_group_schedule_band,
  output logic [11:0] lwc_raw_valid
);
  wire unused_lwc_out_padding;
  TagDetectLwc #(
    // Match the checked-in Ti60 shell: transport calibrated H and recover
    // reciprocal pose scale plus R/t in software.
    .CENTROID_RECIPROCAL_SEGMENT_BIN(
      "../../rtl/lrcca/centroid_reciprocal_segments_q17.bin"
    ),
    .CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN(
      "../../rtl/lrcca/centroid_reciprocal_midpoint_reflection_q17.bin"
    )
  ) dut (
    .clk,
    .rst,
    .in_vref,
    .in_href,
    .in_d0,
    .in_d1,
    .inner_fv_valid,
    .inner_fv_data,
    .hier_inner_end,
    .lwc_out_sof,
    .lwc_out_valid,
    .lwc_out_data,
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
    .lftag_protocol_error_sticky,
    .lftag_in_valid(),
    .lftag_in_group_end(),
    .lftag_in_x_q4(),
    .lftag_in_y_q4(),
    .lftag_in_log_mass_q5_3(),
    .lftag_in_centroid_valid(),
    .lftag_in_scale(),
    .lftag_in_frame_seq(),
    .external_lftag_result_valid(1'b0),
    .external_lftag_result_data(32'd0),
    .external_lftag_result_last(1'b0),
    .external_lftag_busy(1'b0),
    .external_lftag_fifo_level(6'd0),
    .external_lftag_candidate_start(1'b0),
    .external_lftag_candidate_done(1'b0),
    .external_lftag_candidate_cycles(10'd0),
    .external_lftag_fifo_drop_sticky(1'b0),
    .external_lftag_protocol_error_sticky(1'b0),
    .pipeline_busy,
    .lwc_out_padding(unused_lwc_out_padding)
  );

  assign threshold_protocol_error = dut.threshold_protocol_error;
  assign threshold_buffer_overrun = dut.threshold_overrun;
  assign threshold_frontend_overrun =
      dut.wavelet_tag_detect.threshold_frontend_overrun;
  assign hierarchy_overrun = dut.wavelet_tag_detect.hierarchy_overrun;
  assign hier_child_sort_key = dut.centroid_group_log_m_q5_3;
  assign tap_overflow = dut.frontend_protocol_error;
  assign lwc_group_schedule_valid = dut.lwc_pair_valid &&
      dut.lwc_pair_ready && dut.lwc_pair_group_start;
  assign lwc_group_schedule_band = dut.lwc_pair_band;
  assign lwc_raw_valid = dut.raw_valid;

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
      dut.wavelet_tag_detect.threshold_source_y,
      dut.wavelet_tag_detect.threshold_feature_valid_d1,
      dut.wavelet_tag_detect.threshold_feature_valid_d0,
      dut.wavelet_tag_detect.threshold_row_replicated,
      dut.wavelet_tag_detect.threshold_eol,
      dut.wavelet_tag_detect.threshold_t1,
      dut.wavelet_tag_detect.threshold_t0,
      dut.wavelet_tag_detect.threshold_d1,
      dut.wavelet_tag_detect.threshold_d0
    };
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      threshold_stream_hash <= FNV1A_OFFSET;
      threshold_stream_pairs <= '0;
      outer_table_overflows <= '0;
      inner_table_overflows <= '0;
      outer_row_allocations <= '0;
      lwc_group_schedule_hash <= FNV1A_OFFSET;
      lwc_group_schedule_count <= '0;
    end else if (dut.wavelet_tag_detect.threshold_valid) begin
      threshold_stream_hash <=
          fnv1a_word(threshold_stream_hash, threshold_stream_word);
      threshold_stream_pairs <= threshold_stream_pairs + 1'b1;
    end
    if (!rst &&
        dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s1_stage.out_valid) begin
      if (dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s1_stage.out_pf_valid &&
          32'(outer_row_allocations) >=
              dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.TABLE_DEPTH)
        outer_table_overflows <= outer_table_overflows + 1'b1;
      if (dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s1_stage.out_row_end)
        outer_row_allocations <= '0;
      else if (dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s1_stage.out_pf_valid)
        outer_row_allocations <= outer_row_allocations + 1'b1;
    end
    if (!rst &&
        dut.wavelet_tag_detect.hierarchy_core.inner_hier_2ppc.s1_stage.out_valid &&
        dut.wavelet_tag_detect.hierarchy_core.inner_hier_2ppc.s1_stage.out_pf_valid &&
        32'(dut.wavelet_tag_detect.hierarchy_core.inner_hier_2ppc.s1_stage.p_issue) >=
          dut.wavelet_tag_detect.hierarchy_core.inner_hier_2ppc.TABLE_DEPTH)
      inner_table_overflows <= inner_table_overflows + 1'b1;
    if (!rst && dut.lwc_pair_valid && dut.lwc_pair_ready &&
        dut.lwc_pair_group_start) begin
      lwc_group_schedule_hash <=
          (lwc_group_schedule_hash ^ {60'b0, dut.lwc_pair_band}) *
          FNV1A_PRIME;
      lwc_group_schedule_count <= lwc_group_schedule_count + 1'b1;
    end
  end
  // Preserve production assertions, but report the failing row/table context
  // first so capacity failures cannot be mistaken for reference mismatches.
  always @(negedge clk) if (!rst &&
      (dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s3_stage.head_prev_en0 ||
       dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s3_stage.head_prev_en1) &&
      32'(dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s3_stage.s6b.head_prev_head) >=
          dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.TABLE_DEPTH)
    $display("OUTER_TABLE_CONTEXT row=%0d pair=%0d allocations=%0d overflows=%0d head=%0d addr=%0d threshold_pairs=%0d hash=%016x",
      dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s3_stage.s2.row,
      dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s3_stage.s2.col,
      outer_row_allocations, outer_table_overflows,
      dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s3_stage.s6b.head_prev_head,
      dut.wavelet_tag_detect.hierarchy_core.outer_hier_2ppc.s3_stage.s6b.head_prev_addr,
      threshold_stream_pairs, threshold_stream_hash);


  // Observe the ordered-command queue without changing production admission.
  logic [31:0] bundle_queue_peak;
  logic [31:0] arena_live_peak;
  logic [31:0] bundle_admission_stall_cycles;
  logic diagnostic_pipeline_busy_d1;
  always_ff @(posedge clk) begin
    if (rst) begin
      bundle_queue_peak <= 0;
      arena_live_peak <= 0;
      bundle_admission_stall_cycles <= 0;
      diagnostic_pipeline_busy_d1 <= 0;
    end else begin
      diagnostic_pipeline_busy_d1 <= pipeline_busy;
      if (32'(dut.wavelet_tag_detect.hierarchy_core.CHILD_ARENA_ENTRIES) -
          32'(dut.wavelet_tag_detect.hierarchy_core.unused_child_arena_free_count) > arena_live_peak)
        arena_live_peak <= 32'(dut.wavelet_tag_detect.hierarchy_core.CHILD_ARENA_ENTRIES) -
            32'(dut.wavelet_tag_detect.hierarchy_core.unused_child_arena_free_count);
      if (32'(dut.wavelet_tag_detect.hierarchy_core.child_arena_queue_occupancy) > bundle_queue_peak)
        bundle_queue_peak <= 32'(dut.wavelet_tag_detect.hierarchy_core.child_arena_queue_occupancy);
      if (dut.wavelet_tag_detect.hierarchy_core.stream_valid &&
          !dut.wavelet_tag_detect.hierarchy_core.child_arena_admit_ready)
        bundle_admission_stall_cycles <= bundle_admission_stall_cycles + 1;
      if (diagnostic_pipeline_busy_d1 && !pipeline_busy && threshold_stream_pairs != 0)
        $display("ARENA_FRAME_BUDGET bundle_queue_peak=%0d admission_stall_cycles=%0d live_entries_peak=%0d",
          bundle_queue_peak, bundle_admission_stall_cycles, arena_live_peak);
    end
  end

  always @(posedge threshold_buffer_overrun) if (!rst)
    $display("THRESHOLD_OVERRUN threshold_pairs=%0d queued_bundles=%0d admission_stall_cycles=%0d",
      threshold_stream_pairs, dut.wavelet_tag_detect.hierarchy_core.child_arena_queue_occupancy,
      bundle_admission_stall_cycles);


endmodule
