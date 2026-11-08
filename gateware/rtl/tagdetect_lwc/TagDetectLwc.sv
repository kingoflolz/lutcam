`default_nettype none

// Fixed-profile 1280x800 two-pixel pipeline with one wavelet analysis
// frontend shared by the LWC codec and wavelet-scale TagDetect.
module TagDetectLwc #(
  // Main Titanium profile only; generic targets retain fabric arithmetic.
  parameter bit USE_DSP_HIGH_ZOM = 1'b0,
  parameter bit USE_DSP_HIGH_FOM = 1'b0,
    parameter bit BURST_LIMIT_ENABLE = 1'b1,
    parameter bit ENABLE_PROTOCOL_DIAGNOSTICS = 1'b1,
    parameter bit EXTERNAL_LFTAG = 1'b0,
    parameter string GROUP_FAMILY_SCHEDULE_BIN =
        "../../../gateware/rtl/lwc/raw_group_family_schedule.bin",
    parameter string CENTROID_RECIPROCAL_SEGMENT_BIN =
        "rtl/lrcca/centroid_reciprocal_segments_q17.bin",
    parameter string CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN =
        "rtl/lrcca/centroid_reciprocal_midpoint_reflection_q17.bin",
    parameter bit BOUNDED_BYTE_RASTER = 1'b0
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         in_vref,
    input  wire         in_href,
    input  wire  [7:0]  in_d0,
    input  wire  [7:0]  in_d1,
    output wire         inner_fv_valid,
    output wire  [32:0] inner_fv_data,
    output wire         hier_inner_end,
    output wire         lwc_out_sof,
    output wire         lwc_out_valid,
    output wire  [15:0] lwc_out_data,
    output wire         lwc_protocol_error,

    input  wire  [31:0] lftag_cfg_inv_fx_q2_30,
    input  wire  [31:0] lftag_cfg_inv_fy_q2_30,
    input  wire signed [31:0] lftag_cfg_cx_q16_16,
    input  wire signed [31:0] lftag_cfg_cy_q16_16,
    input  wire  [31:0] lftag_cfg_printed_width_q16_16,
    input  wire  [15:0] lftag_cfg_mean_residual_limit_q16,
    input  wire  [7:0]  lftag_cfg_max_mass_spread,
    input  wire  [15:0] lftag_cfg_min_baseline_q4,
    input  wire         lftag_result_ready,
    input  wire         lftag_clear_diagnostics,
    output wire         lftag_result_valid,
    output wire  [31:0] lftag_result_data,
    output wire         lftag_result_last,
    output wire         lftag_busy,
    output wire  [5:0]  lftag_fifo_level,
    output wire         lftag_candidate_start,
    output wire         lftag_candidate_done,
    output wire  [9:0]  lftag_candidate_cycles,
    output wire         lftag_fifo_drop_sticky,
    output wire         lftag_protocol_error_sticky,
    output wire         lftag_in_valid,
    output wire         lftag_in_group_end,
    output wire  [14:0] lftag_in_x_q4,
    output wire  [13:0] lftag_in_y_q4,
    output wire  [7:0]  lftag_in_log_mass_q5_3,
    output wire         lftag_in_centroid_valid,
    output wire  [1:0]  lftag_in_scale,
    output wire  [7:0]  lftag_in_frame_seq,
    input  wire         external_lftag_result_valid,
    input  wire  [31:0] external_lftag_result_data,
    input  wire         external_lftag_result_last,
    input  wire         external_lftag_busy,
    input  wire  [5:0]  external_lftag_fifo_level,
    input  wire         external_lftag_candidate_start,
    input  wire         external_lftag_candidate_done,
    input  wire  [9:0]  external_lftag_candidate_cycles,
    input  wire         external_lftag_fifo_drop_sticky,
    input  wire         external_lftag_protocol_error_sticky,
    output wire         pipeline_busy,
    output wire         lwc_out_padding
);
    logic active_pair;
    logic frame_start_pending;
    logic lwc_in_sof;
    logic clear_frame;

    logic [11:0] raw_valid;
    logic signed [11:0][13:0] raw_value;
    logic ll2_valid;
    logic ll2_first;
    logic ll2_last;
    logic [9:0] ll2_pair_x;
    logic [9:0] ll2_y;
    logic signed [1:0][13:0] ll2_value;
    logic ll4_valid;
    logic ll4_first;
    logic ll4_last;
    logic [9:0] ll4_pair_x;
    logic [9:0] ll4_y;
    logic signed [1:0][13:0] ll4_value;

    logic unused_frontend_frame_done;
    logic frontend_protocol_error;
    logic backend_protocol_error;
    logic unused_threshold_busy;
    logic hierarchy_busy;
    logic codec_frame_pending;
    logic threshold_protocol_error;
    logic threshold_overrun;

    // Generic native-Q4, full-log-mass centroid groups from TagDetect. The
    // compatibility stream retains its format, coordinates, and grouping;
    // full-mass sorting intentionally changes tuple order and may change which
    // members survive an overflowing retained list.
    logic centroid_group_valid;
    logic centroid_group_centroid_valid;
    logic [14:0] centroid_group_x_q4;
    logic [13:0] centroid_group_y_q4;
    logic [7:0] centroid_group_log_m_q5_3;
    logic centroid_group_end;
    logic [14:0] lftag_composite_x_q4;
    logic [13:0] lftag_logical_y_q4;
    logic [7:0] lftag_log_mass_q5_3;
    logic lftag_centroid_valid;
    logic [1:0] lftag_scale;
    logic [14:0] lftag_logical_x_q4;
    logic lftag_coordinate_valid;
    logic [7:0] lftag_frame_seq;

    localparam logic [14:0] LFTAG_RAW_X_END_Q4 = 15'd20480;
    localparam logic [14:0] LFTAG_LL2_X_START_Q4 = 15'd20544;
    localparam logic [14:0] LFTAG_LL2_X_END_Q4 = 15'd25664;
    localparam logic [14:0] LFTAG_LL2_PLANE_END_Q4 = 15'd25728;
    localparam logic [14:0] LFTAG_LL4_X_START_Q4 = 15'd25792;
    localparam logic [14:0] LFTAG_LL4_X_END_Q4 = 15'd27072;
    localparam logic [13:0] LFTAG_RAW_Y_END_Q4 = 14'd12800;
    localparam logic [13:0] LFTAG_LL2_Y_END_Q4 = 14'd3200;
    localparam logic [13:0] LFTAG_LL4_Y_END_Q4 = 14'd800;

    logic [11:0] lwc_quantized_valid;
    logic signed [11:0][7:0] lwc_quantized_q;
    logic [11:0] lwc_quantized_correction;
    logic [11:0] lwc_quantized_residual_class;
    logic unused_in_synthesis_lwc_quantizer_error;

    logic lwc_pair_valid;
    logic lwc_pair_ready;
    logic lwc_pair_group_start;
    logic lwc_pair_group_last;
    logic [3:0] lwc_pair_band;
    logic signed [7:0] lwc_pair_q0;
    logic signed [7:0] lwc_pair_q1;
    logic [1:0] lwc_pair_correction;
    logic [1:0] lwc_pair_residual_class;
    logic lwc_pair_queue_error;

    logic unused_in_synthesis_lwc_packets_done;
    logic [20:0] unused_in_synthesis_lwc_selected_packet_bits;
    logic [15:0] unused_in_synthesis_lwc_groups_committed;
    logic [15:0] unused_in_synthesis_lwc_groups_serialized;
    logic signed [20:0] unused_in_synthesis_lwc_rate_debt;
    logic [19:0] unused_in_synthesis_lwc_extra_room;
    logic unused_in_synthesis_lwc_frame_done;
    logic unused_in_synthesis_lwc_group_encoder_error;

    assign active_pair = in_vref && in_href;
    assign lwc_in_sof = frame_start_pending && active_pair;
    assign clear_frame = lwc_in_sof && active_pair;
    always_ff @(posedge clk) begin
        if (rst) codec_frame_pending <= 1'b0;
        else if (clear_frame) codec_frame_pending <= 1'b1;
        else if (unused_in_synthesis_lwc_frame_done) codec_frame_pending <= 1'b0;
    end
    assign pipeline_busy = codec_frame_pending || unused_threshold_busy
        || hierarchy_busy || centroid_group_valid || lftag_busy
        || (lftag_fifo_level != 0) || (!EXTERNAL_LFTAG && lftag_result_valid);
    // A shared solver retains busy through the final output handshake.
    // Its visible result may cancel immediately on this camera's processing
    // reset; feeding that result-valid back into retirement would form a
    // combinational reset/retirement loop. Private output-valid is unmasked.
    assign lwc_protocol_error = ENABLE_PROTOCOL_DIAGNOSTICS
        ? (frontend_protocol_error || backend_protocol_error ||
            threshold_protocol_error || threshold_overrun)
        : 1'b0;

    // vref may rise before href; SOF accompanies the first accepted pair.
    always_ff @(posedge clk) begin
        if (rst || !in_vref)
            frame_start_pending <= 1'b1;
        else if (active_pair)
            frame_start_pending <= 1'b0;
    end

    assign lftag_composite_x_q4 = centroid_group_x_q4;
    assign lftag_logical_y_q4 = centroid_group_y_q4;
    assign lftag_log_mass_q5_3 = centroid_group_log_m_q5_3;
    assign lftag_centroid_valid = centroid_group_centroid_valid;

    // The hierarchy concatenates three fenced planes.  Convert each native
    // Q4 centroid back to its logical plane before the standalone boundary.
    // Fence/padding centroids retain their plane classification but are
    // marked invalid so unsigned offset subtraction can never masquerade as
    // an active coordinate.
    always_comb begin
        lftag_scale = 2'd0;
        lftag_logical_x_q4 = lftag_composite_x_q4;
        lftag_coordinate_valid =
            (lftag_composite_x_q4 < LFTAG_RAW_X_END_Q4) &&
            (lftag_logical_y_q4 < LFTAG_RAW_Y_END_Q4);
        if (lftag_composite_x_q4 >= LFTAG_RAW_X_END_Q4 &&
                lftag_composite_x_q4 < LFTAG_LL2_PLANE_END_Q4) begin
            lftag_scale = 2'd1;
            lftag_logical_x_q4 =
                lftag_composite_x_q4 - LFTAG_LL2_X_START_Q4;
            lftag_coordinate_valid =
                (lftag_composite_x_q4 >= LFTAG_LL2_X_START_Q4) &&
                (lftag_composite_x_q4 < LFTAG_LL2_X_END_Q4) &&
                (lftag_logical_y_q4 < LFTAG_LL2_Y_END_Q4);
        end else if (lftag_composite_x_q4 >= LFTAG_LL2_PLANE_END_Q4) begin
            lftag_scale = 2'd2;
            lftag_logical_x_q4 =
                lftag_composite_x_q4 - LFTAG_LL4_X_START_Q4;
            lftag_coordinate_valid =
                (lftag_composite_x_q4 >= LFTAG_LL4_X_START_Q4) &&
                (lftag_composite_x_q4 < LFTAG_LL4_X_END_Q4) &&
                (lftag_logical_y_q4 < LFTAG_LL4_Y_END_Q4);
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            lftag_frame_seq <= '1;
        end else begin
            if (lwc_in_sof)
                lftag_frame_seq <= lftag_frame_seq + 1'b1;
        end
    end

    unified_wavelet_frontend #(
        .BOUNDED_BYTE_RASTER(BOUNDED_BYTE_RASTER)
    ) frontend (
        .clk,
        .rst_n(~rst),
        .in_sof(lwc_in_sof),
        .in_valid(active_pair),
        .in_px0(in_d0),
        .in_px1(in_d1),
        .out_raw_valid(raw_valid),
        .out_raw_value(raw_value),
        .out_ll2_valid(ll2_valid),
        .out_ll2_first(ll2_first),
        .out_ll2_last(ll2_last),
        .out_ll2_pair_x(ll2_pair_x),
        .out_ll2_y(ll2_y),
        .out_ll2_value(ll2_value),
        .out_ll4_valid(ll4_valid),
        .out_ll4_first(ll4_first),
        .out_ll4_last(ll4_last),
        .out_ll4_pair_x(ll4_pair_x),
        .out_ll4_y(ll4_y),
        .out_ll4_value(ll4_value),
        .frame_done(unused_frontend_frame_done),
        .protocol_error(frontend_protocol_error)
    );

    assign backend_protocol_error = lwc_pair_queue_error
        || unused_in_synthesis_lwc_group_encoder_error;

    coefficient_quantizer lwc_quantizer (
        .clk,
        .rst,
        .clear_frame,
        .in_valid(raw_valid),
        .in_value(raw_value),
        .out_valid(lwc_quantized_valid),
        .out_q(lwc_quantized_q),
        .out_correction(lwc_quantized_correction),
        .out_residual_class(lwc_quantized_residual_class),
        .protocol_error(unused_in_synthesis_lwc_quantizer_error)
    );

    coefficient_pair_queues #(
        .GROUP_FAMILY_SCHEDULE_BIN(GROUP_FAMILY_SCHEDULE_BIN),
        .COMPACT_PRODUCTION_PROFILE(1'b1)
    ) lwc_pair_queues (
        .clk,
        .rst,
        .clear_frame,
        .in_valid(lwc_quantized_valid),
        .in_q(lwc_quantized_q),
        .in_correction(lwc_quantized_correction),
        .in_residual_class(lwc_quantized_residual_class),
        .pair_valid(lwc_pair_valid),
        .pair_ready(lwc_pair_ready),
        .pair_group_start(lwc_pair_group_start),
        .pair_group_last(lwc_pair_group_last),
        .pair_band(lwc_pair_band),
        .pair_q0(lwc_pair_q0),
        .pair_q1(lwc_pair_q1),
        .pair_correction(lwc_pair_correction),
        .pair_residual_class(lwc_pair_residual_class),
        .protocol_error(lwc_pair_queue_error)
    );

    causal_group_encoder #(.BURST_LIMIT_ENABLE(BURST_LIMIT_ENABLE)) lwc_group_encoder (
        .clk,
        .rst,
        .clear_frame,
        .pair_valid(lwc_pair_valid),
        .pair_ready(lwc_pair_ready),
        .pair_group_start(lwc_pair_group_start),
        .pair_group_last(lwc_pair_group_last),
        .pair_band(lwc_pair_band),
        .pair_q0(lwc_pair_q0),
        .pair_q1(lwc_pair_q1),
        .pair_correction(lwc_pair_correction),
        .pair_residual_class(lwc_pair_residual_class),
        .out_sof(lwc_out_sof),
        .out_valid(lwc_out_valid),
        .out_ready(1'b1),
        .out_data(lwc_out_data),
        .out_padding(lwc_out_padding),
        .frame_done(unused_in_synthesis_lwc_frame_done),
        .packets_done(unused_in_synthesis_lwc_packets_done),
        .packet_bits(unused_in_synthesis_lwc_selected_packet_bits),
        .groups_committed(unused_in_synthesis_lwc_groups_committed),
        .groups_serialized(unused_in_synthesis_lwc_groups_serialized),
        .rate_debt(unused_in_synthesis_lwc_rate_debt),
        .extra_room(unused_in_synthesis_lwc_extra_room),
        .protocol_error(unused_in_synthesis_lwc_group_encoder_error)
    );

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (!rst && !clear_frame) begin
            assert (!unused_in_synthesis_lwc_quantizer_error)
                else $error("quantizer protocol error");
            assert (!lwc_pair_queue_error)
                else $error("pair queue protocol error committed=%0d serialized=%0d selected_bits=%0d debt=%0d room=%0d",
                    unused_in_synthesis_lwc_groups_committed,
                    unused_in_synthesis_lwc_groups_serialized,
                    unused_in_synthesis_lwc_selected_packet_bits,
                    unused_in_synthesis_lwc_rate_debt,
                    unused_in_synthesis_lwc_extra_room);
            assert (!unused_in_synthesis_lwc_group_encoder_error)
                else $error("causal group encoder protocol error");
            assert (unused_in_synthesis_lwc_groups_serialized
                <= unused_in_synthesis_lwc_groups_committed);
            assert (unused_in_synthesis_lwc_groups_committed
                - unused_in_synthesis_lwc_groups_serialized <= 16'd257);
            if (unused_in_synthesis_lwc_packets_done) begin
                assert (unused_in_synthesis_lwc_groups_committed == 16'd64_000);
                assert (unused_in_synthesis_lwc_groups_serialized == 16'd64_000);
            end
        end
    end
`endif

    WaveletTagDetect #(
        .USE_DSP_HIGH_ZOM(USE_DSP_HIGH_ZOM),
    .USE_DSP_HIGH_FOM(USE_DSP_HIGH_FOM),
        .WIDTH(1280),
        .HEIGHT(800),
        .PIXEL_BITS(8),
        .COEFFICIENT_BITS(14),
        .COORD_WIDTH(10),
        .ROW_CYCLES(896),
        .STAGE2_Y_OFFSET(27),
        .STAGE4_Y_OFFSET(127),
        // Keep nine-bit list handles and reserve 511 as the null ID. The
        // dense coarser-threshold regression peaks at 445 live entries.
        .CHILD_ARENA_ENTRIES(511),
        // Ordered event bundles preserve same-epoch child/merge/read order.
        // The dense regression peaks at 222 bundles; 512 absorbs this burst
        // without stalling beyond the frontend's retained RAW rows.
        .CHILD_ARENA_CMD_QUEUE_DEPTH(512),
        // A 1696-pixel composite row can close at most 848 foreground runs.
        // Both tables cover that structural bound, including dense masks
        // produced by the coarser threshold references. Their ten-bit labels
        // still leave 1023 available as the compact unmapped sentinel.
        .OUTER_CCA_TABLE_DEPTH(848),
        .INNER_CCA_TABLE_DEPTH(848),
        .CHILD_AREA_BITS(21),
        .CHILD_AREA_MAX(1368672),
        .CENTROID_RECIPROCAL_SEGMENT_BIN(CENTROID_RECIPROCAL_SEGMENT_BIN),
        .CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN(
            CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN
        )
    ) wavelet_tag_detect (
        .clk,
        .rst,
        .in_vref,
        .in_href,
        .in_d0,
        .in_d1,
        .ll2_valid,
        .ll2_first,
        .ll2_last,
        .ll2_pair_x,
        .ll2_y,
        .ll2_value,
        .ll4_valid,
        .ll4_first,
        .ll4_last,
        .ll4_pair_x,
        .ll4_y,
        .ll4_value,
        .inner_fv_valid,
        .inner_fv_data,
        .hier_inner_end,
        .centroid_group_valid,
        .centroid_group_centroid_valid,
        .centroid_group_x_q4,
        .centroid_group_y_q4,
        .centroid_group_log_m_q5_3,
        .centroid_group_end,
        .threshold_busy(unused_threshold_busy),
        .threshold_protocol_error,
        .threshold_overrun,
        .hierarchy_busy
    );

    // The native candidate stream has no backpressure. External sharing
    // therefore retains an independent ingress queue for this camera.
    assign lftag_in_valid = centroid_group_valid;
    assign lftag_in_group_end = centroid_group_end;
    assign lftag_in_x_q4 = lftag_logical_x_q4;
    assign lftag_in_y_q4 = lftag_logical_y_q4;
    assign lftag_in_log_mass_q5_3 = lftag_log_mass_q5_3;
    assign lftag_in_centroid_valid = lftag_centroid_valid && lftag_coordinate_valid;
    assign lftag_in_scale = lftag_scale;
    assign lftag_in_frame_seq = lftag_frame_seq;

    generate if (EXTERNAL_LFTAG) begin : g_external_lftag
        assign lftag_result_valid = external_lftag_result_valid;
        assign lftag_result_data = external_lftag_result_data;
        assign lftag_result_last = external_lftag_result_last;
        assign lftag_busy = external_lftag_busy;
        assign lftag_fifo_level = external_lftag_fifo_level;
        assign lftag_candidate_start = external_lftag_candidate_start;
        assign lftag_candidate_done = external_lftag_candidate_done;
        assign lftag_candidate_cycles = external_lftag_candidate_cycles;
        assign lftag_fifo_drop_sticky = external_lftag_fifo_drop_sticky;
        assign lftag_protocol_error_sticky = external_lftag_protocol_error_sticky;
    end else begin : g_private_lftag
    LfTagCompact3x3DecoderLocalizer lftag_decoder_localizer (
        .config_read_valid(), .config_read_address(), .config_read_ready(1'b0),
        .config_response_valid(1'b0), .config_read_data(32'b0),
        .clk,
        .rst,
        .in_valid(centroid_group_valid),
        .in_group_end(centroid_group_end),
        .in_x_q4(lftag_logical_x_q4),
        .in_y_q4(lftag_logical_y_q4),
        .in_log_mass_q5_3(lftag_log_mass_q5_3),
        .in_centroid_valid(lftag_centroid_valid && lftag_coordinate_valid),
        .in_scale(lftag_scale),
        .in_frame_seq(lftag_frame_seq),
        .cfg_inv_fx_q2_30(lftag_cfg_inv_fx_q2_30),
        .cfg_inv_fy_q2_30(lftag_cfg_inv_fy_q2_30),
        .cfg_cx_q16_16(lftag_cfg_cx_q16_16),
        .cfg_cy_q16_16(lftag_cfg_cy_q16_16),
        .cfg_printed_width_q16_16(lftag_cfg_printed_width_q16_16),
        .cfg_mean_residual_limit_q16(lftag_cfg_mean_residual_limit_q16),
        .cfg_max_mass_spread(lftag_cfg_max_mass_spread),
        .cfg_min_baseline_q4(lftag_cfg_min_baseline_q4),
        .centroid_valid(), .centroid_ready(1'b1), .centroid_data(), .centroid_last(),
        .out_valid(lftag_result_valid),
        .out_ready(lftag_result_ready),
        .out_data(lftag_result_data),
        .out_last(lftag_result_last),
        .busy(lftag_busy),
        .fifo_level(lftag_fifo_level),
        .candidate_start(lftag_candidate_start),
        .candidate_done(lftag_candidate_done),
        .candidate_cycles(lftag_candidate_cycles),
        .clear_diagnostics(lftag_clear_diagnostics),
        .fifo_drop_sticky(lftag_fifo_drop_sticky),
        .protocol_error_sticky(lftag_protocol_error_sticky),
        .core_idle(),
        .core_dispatch_ready(),
        .external_fifo_head_valid(1'b0),
        .external_fifo_head_scale(2'b0),
        .external_fifo_head_frame_seq(8'b0),
        .external_fifo_head_protocol(1'b0),
        .external_fifo_head_mass_spread(8'b0),
        .external_fifo_head_pop(),
        .external_fifo_desc_read_valid(),
        .external_fifo_desc_read_index(),
        .external_fifo_desc_read_data_valid(1'b0),
        .external_fifo_desc_read_data(40'b0),
        .external_fifo_desc_read_centroid_valid(1'b0),
        .external_fifo_level(6'b0),
        .external_fifo_drop_sticky(1'b0),
        .external_protocol_error_sticky(1'b0)
    );
    end endgenerate
endmodule

`default_nettype wire
