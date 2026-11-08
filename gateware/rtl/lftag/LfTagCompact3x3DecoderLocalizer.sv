// Compact streaming LFTag3x3 decoder and calibrated localizer.
//
// The direct-pose profile uses eight RAM10 memories and emits unversioned packets
// with rotation and translation. The homogeneous scorer uses nine numeric
// DSP products while the pose leaf serializes one multiplier lane, because
// the target is one candidate in at most 900 clocks rather than one candidate
// per pixel clock.
module LfTagCompact3x3DecoderLocalizer #(
    // Standalone users retain the original queue and ingress interface.
    // Shared users provide the same synchronous queue port externally.
    parameter bit EXTERNAL_FIFO = 1'b0,
    parameter bit EXPORT_ACCEPTED_CENTROIDS = 1'b0,
    parameter bit EXTERNAL_CONFIG = 1'b0
) (
    input  logic         clk,
    input  logic         rst,

    /* verilator lint_off UNUSEDSIGNAL */
    input  logic         in_valid,
    input  logic         in_group_end,
    input  logic [14:0]  in_x_q4,
    input  logic [13:0]  in_y_q4,
    input  logic [7:0]   in_log_mass_q5_3,
    input  logic         in_centroid_valid,
    input  logic [1:0]   in_scale,
    input  logic [7:0]   in_frame_seq,
    /* verilator lint_on UNUSEDSIGNAL */

    input  logic [31:0]  cfg_inv_fx_q2_30,
    input  logic [31:0]  cfg_inv_fy_q2_30,
    input  logic signed [31:0] cfg_cx_q16_16,
    input  logic signed [31:0] cfg_cy_q16_16,
    input  logic [31:0]  cfg_printed_width_q16_16,
    input  logic [15:0]  cfg_mean_residual_limit_q16,
    input  logic [7:0]   cfg_max_mass_spread,
    input  logic [15:0]  cfg_min_baseline_q4,

    // One outstanding synchronous configuration read. Shared integration maps
    // logical intrinsics 8..11 to the selected camera's physical bank.
    output logic         config_read_valid,
    output logic [3:0]   config_read_address,
    input  logic         config_read_ready,
    input  logic         config_response_valid,
    input  logic [31:0]  config_read_data,

    output logic         out_valid,
    input  logic         out_ready,
    output logic [31:0]  out_data,
    output logic         out_last,
    // Optional accepted-group replay. Coordinates retain the composite
    // plane Q5 wire layout; nine tuples appear only for accepted poses.
    output logic         centroid_valid,
    input  logic         centroid_ready,
    output logic [32:0]  centroid_data,
    output logic         centroid_last,

    output logic         busy,
    output logic [5:0]   fifo_level,
    output logic         candidate_start,
    output logic         candidate_done,
    output logic [9:0]   candidate_cycles,
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic         clear_diagnostics,
    /* verilator lint_on UNUSEDSIGNAL */
    output logic         fifo_drop_sticky,
    output logic         protocol_error_sticky,

    // Idle is independent of queue occupancy. A head is accepted on an edge
    // with core_dispatch_ready && external_fifo_head_valid in external mode.
    output logic         core_idle,
    output logic         core_dispatch_ready,
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic         external_fifo_head_valid,
    input  logic [1:0]   external_fifo_head_scale,
    input  logic [7:0]   external_fifo_head_frame_seq,
    input  logic         external_fifo_head_protocol,
    input  logic [7:0]   external_fifo_head_mass_spread,
    output logic         external_fifo_head_pop,
    output logic         external_fifo_desc_read_valid,
    output logic [3:0]   external_fifo_desc_read_index,
    input  logic         external_fifo_desc_read_data_valid,
    input  logic [39:0]  external_fifo_desc_read_data,
    input  logic         external_fifo_desc_read_centroid_valid,
    input  logic [5:0]   external_fifo_level,
    input  logic         external_fifo_drop_sticky,
    input  logic         external_protocol_error_sticky
    /* verilator lint_on UNUSEDSIGNAL */
);
    import lftag_pkg::*;

    typedef enum logic [3:0] {
        S_IDLE,
        S_VALIDATE,
        S_CORNER_START,
        S_CORNER_WAIT,
        S_RECT_START,
        S_RECT_WAIT,
        S_SCORE_WAIT,
        S_H_START,
        S_H_WAIT,
        S_POSE_START,
        S_POSE_WAIT,
        S_PACKET,
        S_WAIT_PACKET,
        S_CENTROIDS,
        S_CONFIG_REQUEST,
        S_CONFIG_RESPONSE
    } state_t;

    state_t state;
    logic [2:0] config_index;
    logic [31:0] config_work;
    logic [7:0] selected_max_mass_spread;
    logic [15:0] selected_min_baseline_q4;
    logic [31:0] selected_printed_width;
    assign selected_max_mass_spread = EXTERNAL_CONFIG
        ? config_work[7:0] : cfg_max_mass_spread;
    assign selected_min_baseline_q4 = EXTERNAL_CONFIG
        ? config_work[23:8] : cfg_min_baseline_q4;
    assign selected_printed_width = EXTERNAL_CONFIG
        ? config_work : cfg_printed_width_q16_16;

    lftag_reject_t reject_reason;
    logic [9:0] cycle_counter;
    logic core_soft_reset;

    // Configuration payload uses fabric registers; DSPs are reserved for arithmetic.
    always_ff @(posedge clk) begin : config_work_storage
        if (rst) config_work <= 32'b0;
        else if (!rst && (state == S_CONFIG_RESPONSE) && config_response_valid
            && ((config_index == 3'd7) || (config_index == 3'd0))
            && (cycle_counter < 10'd999)) config_work <= config_read_data;
    end


    logic [1:0] work_scale;
    logic [7:0] work_frame;
    logic candidate_popped;

    logic [4:0] result_index;
    logic release_results;

    // Normalize the selected winner once, while homography runs. The Q32
    // quality remains exact, but acceptance needs only one Q16 limit. A
    // uniform restoring step shares comparison and subtraction; the quotient
    // shifts instead of requiring a variable-index write decoder. Retaining
    // the remainder prevents accepting a ratio just above an exact limit.
    logic mean_active;
    logic [4:0] mean_bit;
    logic [31:0] mean_quotient;
    logic [38:0] normalization_remainder;
    logic [15:0] selected_mean_residual_limit;
    logic mean_limit_pass;
    assign selected_mean_residual_limit = EXTERNAL_CONFIG
        ? config_read_data[15:0] : cfg_mean_residual_limit_q16;
    assign mean_limit_pass = (mean_quotient[31:16] < selected_mean_residual_limit)
        || ((mean_quotient[31:16] == selected_mean_residual_limit)
            && (mean_quotient[15:0] == 16'b0) && (normalization_remainder == 39'b0));

    // Accepted packet header words are materialized in otherwise-unused low
    // addresses of the pose-result banks while homography and pose run.  This
    // replaces the wide result-index mux on the serialized output without
    // allocating another RAM10.  Rejected packets use only their two nonzero
    // transport words and never consume these stored values.
    logic header_fill_active;
    logic [2:0] header_fill_index;
    logic mean_write_pending;
    logic header_fill_write_fire;
    logic mean_write_fire;

    // Candidate FIFO and shared synchronous descriptor read port.
    logic fifo_head_valid;
    logic [1:0] fifo_head_scale;
    logic [7:0] fifo_head_frame;
    logic fifo_head_protocol;
    logic [7:0] fifo_head_mass_spread;
    logic fifo_head_pop;
    logic fifo_desc_read_valid;
    logic [3:0] fifo_desc_read_index;
    logic fifo_desc_data_valid;
    logic [39:0] fifo_desc_data;
    logic fifo_desc_centroid_valid;
    logic [3:0] centroid_descriptor_index;
    logic [14:0] centroid_composite_x_q4;
    logic [32:0] centroid_cache_word;
    // The corner selector reads every descriptor before the early queue pop.
    // Save those reads into unused addresses 32..40 of the existing result
    // banks. Header/pose use 0..25; inverse coefficients use 64..127.
    always_ff @(posedge clk)
        centroid_descriptor_index <= fifo_desc_read_index;
    always_comb begin
        centroid_composite_x_q4 = fifo_desc_data[14:0];
        case (work_scale)
            2'd1: centroid_composite_x_q4 = fifo_desc_data[14:0] + 15'd20544;
            2'd2: centroid_composite_x_q4 = fifo_desc_data[14:0] + 15'd25792;
            default: begin end
        endcase
    end
    assign centroid_cache_word = {1'b0, fifo_desc_data[28:15], 1'b0,
        centroid_composite_x_q4, 1'b0, fifo_desc_data[37]};
    /* verilator lint_off UNUSEDSIGNAL */
    logic fifo_full;
    logic [31:0] unused_fifo_debug_status;
    logic [31:0] unused_fifo_debug_protocol;
    logic [31:0] unused_fifo_debug_drop;
    /* verilator lint_on UNUSEDSIGNAL */

    logic signed [17:0] fifo_desc_x_q4;
    logic signed [17:0] fifo_desc_y_q4;
    logic fifo_desc_reserved_error;
    assign fifo_desc_x_q4 = $signed({3'b000, fifo_desc_data[14:0]});
    assign fifo_desc_y_q4 = $signed({4'b0000, fifo_desc_data[28:15]});
    assign fifo_desc_reserved_error = fifo_desc_data_valid
        && (|fifo_desc_data[39:38]);

    generate if (EXTERNAL_FIFO) begin : g_external_fifo
        assign fifo_head_valid = external_fifo_head_valid;
        assign fifo_head_scale = external_fifo_head_scale;
        assign fifo_head_frame = external_fifo_head_frame_seq;
        assign fifo_head_protocol = external_fifo_head_protocol;
        assign fifo_head_mass_spread = external_fifo_head_mass_spread;
        assign fifo_desc_data_valid = external_fifo_desc_read_data_valid;
        assign fifo_desc_data = external_fifo_desc_read_data;
        assign fifo_desc_centroid_valid = external_fifo_desc_read_centroid_valid;
        assign fifo_level = external_fifo_level;
        assign fifo_full = external_fifo_level == 6'd32;
        assign fifo_drop_sticky = external_fifo_drop_sticky;
        assign protocol_error_sticky = external_protocol_error_sticky;
    end else begin : g_internal_fifo
        LfTagCandidateFifoRamMetadata candidate_fifo (
            .clk,
            .rst,
            .in_valid,
            .in_group_end,
            .in_x_q4,
            .in_y_q4,
            .in_log_mass_q5_3,
            .in_centroid_valid,
            .in_scale,
            .in_frame_seq,
            .head_valid(fifo_head_valid),
            .head_scale(fifo_head_scale),
            .head_frame_seq(fifo_head_frame),
            .head_protocol(fifo_head_protocol),
            .head_mass_spread(fifo_head_mass_spread),
            .head_pop(fifo_head_pop),
            .desc_read_valid(fifo_desc_read_valid),
            .desc_read_index(fifo_desc_read_index),
            .desc_read_data_valid(fifo_desc_data_valid),
            .desc_read_data(fifo_desc_data),
            .desc_read_centroid_valid(fifo_desc_centroid_valid),
            .fifo_level,
            .fifo_full(fifo_full),
            .clear_diagnostics,
            .fifo_drop_sticky,
            .protocol_error_sticky,
            .diagnostic_reset(1'b0),
            .diagnostic_context(11'b0),
            .debug_status(unused_fifo_debug_status),
            .debug_protocol(unused_fifo_debug_protocol),
            .debug_drop(unused_fifo_debug_drop)
        );
    end endgenerate

    assign external_fifo_head_pop = EXTERNAL_FIFO && fifo_head_pop;
    assign external_fifo_desc_read_valid = EXTERNAL_FIFO && fifo_desc_read_valid;
    assign external_fifo_desc_read_index = fifo_desc_read_index;

    // Streaming corner selection also consumes validity and the strict mass
    // cutoff while it is already rereading all nine descriptors.  No separate
    // validation scan or point register bank is needed.
    logic corner_in_ready;
    logic corner_desc_req_valid;
    logic [3:0] corner_desc_req_index;
    logic corner_out_valid;
    logic [2:0] corner_out_status;
    logic corner_out_input_valid;
    /* verilator lint_off UNUSEDSIGNAL */
    logic corner_out_baseline_valid;
    logic corner_out_dominant_valid;
    logic corner_out_corners_valid;
    logic corner_out_quad_valid;
    logic [7:0] corner_out_cycles;
    /* verilator lint_on UNUSEDSIGNAL */
    /* verilator lint_off UNUSEDSIGNAL */
    logic [15:0] corner_indices;
    /* verilator lint_on UNUSEDSIGNAL */
    logic [19:0] remaining_indices;
    logic [71:0] corner_x_q4_flat;
    logic [71:0] corner_y_q4_flat;

    LfTagCornerSelect9 #(.USE_DSP_SUBTRACT(1'b1)) corner_select (
        .clk,
        .rst(rst || core_soft_reset),
        .in_valid(state == S_CORNER_START),
        .in_ready(corner_in_ready),
        .in_points_valid(1'b1),
        .in_min_baseline_q4(selected_min_baseline_q4),
        .desc_req_valid(corner_desc_req_valid),
        .desc_req_index(corner_desc_req_index),
        .desc_rsp_valid(fifo_desc_data_valid
            && ((state == S_CORNER_START) || (state == S_CORNER_WAIT))),
        .desc_rsp_x_q4(fifo_desc_x_q4),
        .desc_rsp_y_q4(fifo_desc_y_q4),
        .desc_rsp_mass_q5_3(fifo_desc_data[36:29]),
        .desc_rsp_centroid_valid(fifo_desc_centroid_valid
            && fifo_desc_data[37]
            && !fifo_desc_reserved_error),
        .out_valid(corner_out_valid),
        .out_ready(release_results),
        .out_status(corner_out_status),
        .out_input_valid(corner_out_input_valid),
        .out_baseline_valid(corner_out_baseline_valid),
        .out_dominant_valid(corner_out_dominant_valid),
        .out_corners_valid(corner_out_corners_valid),
        .out_quad_valid(corner_out_quad_valid),
        .out_corner_index_flat(corner_indices),
        .out_remaining_index_flat(remaining_indices),
        .out_corner_x_q4_flat(corner_x_q4_flat),
        .out_corner_y_q4_flat(corner_y_q4_flat),
        .out_cycles(corner_out_cycles)
    );

    // Divide-free four-corner geometry and sixteen-hypothesis scoring share
    // one Q15 cross-product engine and two signed arithmetic multiplier lanes.
    logic rect_start_ready;
    logic rect_desc_req_valid;
    logic rect_desc_req_ready;
    logic [3:0] rect_desc_req_index;
    logic signed [17:0] rect_map_cden;
    logic signed [17:0] rect_map_cg;
    logic signed [17:0] rect_map_ch;
    logic rect_done_valid;
    logic rect_done_invalid;
    logic scorer_clear;
    logic scorer_out_valid;
    logic scorer_best_valid;
    logic scorer_input_invalid;
    logic [3:0] scorer_best_hypothesis;
    logic [13:0] scorer_best_id;
    logic [63:0] scorer_best_score;
    logic [37:0] scorer_best_residual_sum;
    logic [38:0] scorer_best_weight_sum;
    /* verilator lint_off UNUSEDSIGNAL */
    logic [9:0] homogeneous_cycles;
    logic [14:0] unused_scorer_cell_map;
    /* verilator lint_on UNUSEDSIGNAL */

    LfTagHomogeneousGeometryScorer homogeneous_scorer (
        .clk,
        .rst(rst || core_soft_reset),
        .clear(scorer_clear),
        .start_valid(state == S_RECT_START),
        .start_ready(rect_start_ready),
        .in_scale(work_scale),
        .corner_x_flat(corner_x_q4_flat),
        .corner_y_flat(corner_y_q4_flat),
        .point_input_index_flat(remaining_indices),
        .descriptor_req_valid(rect_desc_req_valid),
        .descriptor_req_ready(rect_desc_req_ready),
        .descriptor_req_index(rect_desc_req_index),
        .descriptor_rsp_valid(fifo_desc_data_valid && (state == S_RECT_WAIT)),
        .descriptor_rsp_x(fifo_desc_x_q4),
        .descriptor_rsp_y(fifo_desc_y_q4),
        .geometry_done_valid(rect_done_valid),
        .geometry_done_invalid(rect_done_invalid),
        .map_cden(rect_map_cden),
        .map_cg(rect_map_cg),
        .map_ch(rect_map_ch),
        .out_valid(scorer_out_valid),
        .out_ready(release_results),
        .out_best_valid(scorer_best_valid),
        .out_input_invalid(scorer_input_invalid),
        .out_best_hypothesis(scorer_best_hypothesis),
        .out_best_id(scorer_best_id),
        .out_best_cell_map(unused_scorer_cell_map),
        .out_best_score_q32(scorer_best_score),
        .out_best_residual_sum(scorer_best_residual_sum),
        .out_best_weight_sum(scorer_best_weight_sum),
        .out_cycles(homogeneous_cycles)
    );

    // Direct four-key calibrated homography. H streams directly into the
    // shared geometry RAM in common-scale signed Q16.
    logic h_in_ready;
    logic h_out_valid;
    logic h_result_write_valid;
    logic [3:0] h_result_write_address;
    logic signed [17:0] h_result_write_data;
    logic h_out_invalid;
    logic h_out_saturation;
    /* verilator lint_off UNUSEDSIGNAL */
    logic [6:0] h_out_cycles;
    /* verilator lint_on UNUSEDSIGNAL */
    logic [6:0] h_inverse_read_address;

    logic h_config_read_valid;
    logic [1:0] h_config_read_address;
    logic h_config_owner;
    assign h_config_owner = (state == S_H_START) || (state == S_H_WAIT);
    assign config_read_valid = EXTERNAL_CONFIG &&
        ((state == S_CONFIG_REQUEST) || (h_config_owner && h_config_read_valid));
    assign config_read_address = h_config_owner
        ? {2'b10, h_config_read_address} : {1'b0, config_index};

    LfTagInitialHomographyFused #(.EXTERNAL_CONFIG(EXTERNAL_CONFIG)) homography (
        .clk,
        .rst(rst || core_soft_reset),
        .in_valid(state == S_H_START),
        .in_ready(h_in_ready),
        .in_key_x_q4_flat(corner_x_q4_flat),
        .in_key_y_q4_flat(corner_y_q4_flat),
        .in_cden(rect_map_cden),
        .in_cg(rect_map_cg),
        .in_ch(rect_map_ch),
        .in_hypothesis(scorer_best_hypothesis),
        .in_scale(work_scale),
        .in_inv_fx_q2_30(cfg_inv_fx_q2_30),
        .in_inv_fy_q2_30(cfg_inv_fy_q2_30),
        .in_cx_q16(cfg_cx_q16_16),
        .in_cy_q16(cfg_cy_q16_16),
        .in_saturation(rect_done_invalid),
        .config_read_valid(h_config_read_valid),
        .config_read_address(h_config_read_address),
        .config_read_ready(h_config_owner && config_read_ready),
        .config_response_valid(h_config_owner && config_response_valid),
        .config_read_data,
        .out_valid(h_out_valid),
        .out_ready(release_results),
        .result_write_valid(h_result_write_valid),
        .result_write_address(h_result_write_address),
        .result_write_data(h_result_write_data),
        .out_invalid(h_out_invalid),
        .out_saturation(h_out_saturation),
        .out_cycles(h_out_cycles),
        .inverse_read_address(h_inverse_read_address),
        .inverse_read_data_a($signed(pose_result_read_data_a)),
        .inverse_read_data_b($signed(pose_result_read_data_b))
    );

    // Two RAM10 banks hold H and then the direct pose in low addresses. Their
    // reserved high half stores paired canonical inverse coefficients, so the
    // homography frontend needs no separate logic ROM or additional RAM.
    (* syn_ramstyle = "block_ram" *) logic [17:0] pose_result_bank_a [0:127];
    (* syn_ramstyle = "block_ram" *) logic [17:0] pose_result_bank_b [0:127];
    logic [17:0] pose_result_read_data_a;
    logic [17:0] pose_result_read_data_b;
    logic [6:0] pose_result_read_address_a;
    logic [6:0] pose_result_read_address_b;
    logic [3:0] pose_result_read_address_q;
    logic pose_result_write_enable_a;
    logic pose_result_write_enable_b;
    logic [6:0] pose_result_write_address;
    logic [17:0] pose_result_write_data_a;
    logic [17:0] pose_result_write_data_b;
    logic [31:0] pose_result_data;
    logic [31:0] packet_header_data;
    logic [31:0] header_fill_data;

    logic [31:0] result_status;

    logic pose_in_ready;
    logic pose_out_valid;
    logic pose_valid;
    logic pose_weak;
    logic pose_conditioning;
    logic pose_saturation;
    logic [3:0] pose_leaf_read_address_a;
    logic [3:0] pose_leaf_read_address_b;
    logic pose_leaf_write_valid;
    logic [3:0] pose_leaf_write_address;
    logic [17:0] pose_leaf_write_data_a;
    logic [17:0] pose_leaf_write_data_b;
    logic pose_rsqrt_req_valid;
    logic pose_rsqrt_req_ready;
    logic [36:0] pose_rsqrt_radicand;
    logic pose_rsqrt_rsp_valid;
    logic pose_rsqrt_rsp_ready;
    logic [18:0] pose_rsqrt_lambda_q10;
    logic [3:0] pose_rsqrt_left_shift;

    assign result_status = {28'b0, pose_saturation,
        pose_conditioning, pose_weak, pose_valid};

    initial begin : pose_result_inverse_init
`include "lftag_h_inverse_pair_init.svh"
    end

    LfTagPoseReconstruct pose_reconstruct (
        .clk,
        .rst(rst || core_soft_reset),
        .in_valid(state == S_POSE_START),
        .in_ready(pose_in_ready),
        .in_pose_enable(!h_out_invalid),
        .in_saturation(h_out_saturation),
        .in_printed_width_q16(selected_printed_width),
        .out_valid(pose_out_valid),
        .out_ready(release_results),
        .out_pose_valid(pose_valid),
        .out_weak_perspective(pose_weak),
        .out_conditioning_warning(pose_conditioning),
        .out_saturation(pose_saturation),
        .result_read_address_a(pose_leaf_read_address_a),
        .result_read_address_b(pose_leaf_read_address_b),
        .result_read_data_a($signed(pose_result_read_data_a)),
        .result_read_data_b($signed(pose_result_read_data_b)),
        .result_write_valid(pose_leaf_write_valid),
        .result_write_address(pose_leaf_write_address),
        .result_write_data_a(pose_leaf_write_data_a),
        .result_write_data_b(pose_leaf_write_data_b),
        .rsqrt_req_valid(pose_rsqrt_req_valid),
        .rsqrt_req_ready(pose_rsqrt_req_ready),
        .rsqrt_req_radicand(pose_rsqrt_radicand),
        .rsqrt_rsp_valid(pose_rsqrt_rsp_valid),
        .rsqrt_rsp_ready(pose_rsqrt_rsp_ready),
        .rsqrt_rsp_lambda_q10(pose_rsqrt_lambda_q10),
        .rsqrt_rsp_left_shift(pose_rsqrt_left_shift)
    );

    LfTagReciprocalSqrt512 reciprocal_sqrt (
        .clk,
        .rst(rst || core_soft_reset),
        .in_valid(pose_rsqrt_req_valid),
        .in_ready(pose_rsqrt_req_ready),
        .in_radicand(pose_rsqrt_radicand),
        .out_valid(pose_rsqrt_rsp_valid),
        .out_ready(pose_rsqrt_rsp_ready),
        .out_lambda_q10(pose_rsqrt_lambda_q10),
        .out_lambda_left_shift(pose_rsqrt_left_shift)
    );

    always_comb begin : accepted_header_fill_word
        header_fill_data = 32'b0;
        case (header_fill_index)
            3'd0: header_fill_data = {18'b0, scorer_best_id};
            // Bit 30 is reserved zero; retained-list truncation is not a fault.
            3'd1: header_fill_data = {
                1'b1, 1'b0, work_scale, work_frame,
                scorer_best_hypothesis, 16'b0
            };
            3'd2: header_fill_data = scorer_best_score[31:0];
            3'd3: header_fill_data = scorer_best_score[63:32];
            // Words 4 and 5 are reserved zero.
            default: header_fill_data = 32'b0;
        endcase
    end

    always_comb begin : pose_result_ram_arbiter
        pose_result_read_address_a = {3'b0, pose_leaf_read_address_a};
        pose_result_read_address_b = {3'b0, pose_leaf_read_address_b};
        pose_result_write_enable_a = pose_leaf_write_valid;
        pose_result_write_enable_b = pose_leaf_write_valid;
        pose_result_write_address = {3'b0, pose_leaf_write_address};
        pose_result_write_data_a = pose_leaf_write_data_a;
        pose_result_write_data_b = pose_leaf_write_data_b;
        header_fill_write_fire = 1'b0;
        mean_write_fire = 1'b0;

        if ((state == S_H_START) || (state == S_H_WAIT)) begin
            pose_result_read_address_a = h_inverse_read_address;
            pose_result_read_address_b = h_inverse_read_address;
            pose_result_write_enable_a = h_result_write_valid;
            pose_result_write_enable_b = 1'b0;
            pose_result_write_address = {3'b0, h_result_write_address};
            pose_result_write_data_a = h_result_write_data;
            pose_result_write_data_b = 18'b0;
        end else if (state == S_PACKET) begin
            if (reject_reason == REJECT_NONE) begin
                pose_result_read_address_a = 7'd16;
                pose_result_read_address_b = 7'd16;
            end else begin
                pose_result_read_address_a = 7'd0;
                pose_result_read_address_b = 7'd0;
            end
        end else if (EXPORT_ACCEPTED_CENTROIDS && (state == S_CENTROIDS)) begin
            pose_result_read_address_a = 7'd32 + {2'b0, result_index}
                + ((centroid_ready && (result_index != 5'd8)) ? 7'd1 : 7'd0);
            pose_result_read_address_b = pose_result_read_address_a;
        end else if (state == S_WAIT_PACKET) begin
            if (reject_reason != REJECT_NONE) begin
                pose_result_read_address_a = 7'd0;
                pose_result_read_address_b = 7'd0;
            end else if (EXPORT_ACCEPTED_CENTROIDS && out_ready && out_last) begin
                pose_result_read_address_a = 7'd32;
                pose_result_read_address_b = 7'd32;
            end else if (result_index < 5'd9) begin
                if (out_ready) begin
                    pose_result_read_address_a
                        = 7'd17 + {2'b0, result_index};
                    pose_result_read_address_b
                        = 7'd17 + {2'b0, result_index};
                end else begin
                    pose_result_read_address_a
                        = 7'd16 + {2'b0, result_index};
                    pose_result_read_address_b
                        = 7'd16 + {2'b0, result_index};
                end
            end else if (result_index == 5'd9) begin
                if (out_ready) begin
                    pose_result_read_address_a = 7'd0;
                    pose_result_read_address_b = 7'd0;
                end else begin
                    pose_result_read_address_a = 7'd25;
                    pose_result_read_address_b = 7'd25;
                end
            end else if (out_ready && (result_index < 5'd21)) begin
                pose_result_read_address_a = 7'(result_index - 5'd9);
                pose_result_read_address_b = 7'(result_index - 5'd9);
            end else begin
                pose_result_read_address_a = 7'(result_index - 5'd10);
                pose_result_read_address_b = 7'(result_index - 5'd10);
            end
        end

        // Functional datapaths have priority over background packet-header
        // materialization.  Their writes are sparse enough that all eight
        // fixed header words and the mean residual complete long before pose.
        // Status is written in S_PACKET while header word zero is prefetched.
        if (EXPORT_ACCEPTED_CENTROIDS && (state == S_CORNER_WAIT)
                && fifo_desc_data_valid) begin
            pose_result_write_enable_a = 1'b1;
            pose_result_write_enable_b = 1'b1;
            pose_result_write_address = 7'd32 + {3'b0, centroid_descriptor_index};
            pose_result_write_data_a = centroid_cache_word[17:0];
            pose_result_write_data_b = {3'b0, centroid_cache_word[32:18]};
        end else if ((state == S_PACKET) && (reject_reason == REJECT_NONE)) begin
            pose_result_write_enable_a = 1'b1;
            pose_result_write_enable_b = 1'b1;
            pose_result_write_address = 7'd24;
            pose_result_write_data_a = result_status[17:0];
            pose_result_write_data_b = {4'b0, result_status[31:18]};
        end else if (!pose_result_write_enable_a) begin
            if (header_fill_active) begin
                pose_result_write_enable_a = 1'b1;
                pose_result_write_enable_b = 1'b1;
                pose_result_write_address
                    = 7'd16 + {4'b0, header_fill_index};
                pose_result_write_data_a = header_fill_data[17:0];
                pose_result_write_data_b
                    = {4'b0, header_fill_data[31:18]};
                header_fill_write_fire = 1'b1;
            end else if (mean_write_pending) begin
                pose_result_write_enable_a = 1'b1;
                pose_result_write_enable_b = 1'b1;
                pose_result_write_address = 7'd25;
                pose_result_write_data_a = mean_quotient[17:0];
                pose_result_write_data_b = {4'b0, mean_quotient[31:18]};
                mean_write_fire = 1'b1;
            end
        end
    end

    always_comb begin : pose_result_format
        pose_result_data = 32'b0;
        packet_header_data = {pose_result_read_data_b[13:0],
            pose_result_read_data_a};
        if (pose_result_read_address_q < 4'd9) begin
            pose_result_data = {pose_result_read_data_a[17],
                pose_result_read_data_a, 13'b0};
        end else if (pose_result_read_address_q <= 4'd11) begin
            pose_result_data = {pose_result_read_data_b[13:0],
                pose_result_read_data_a};
        end
    end

    always_ff @(posedge clk) begin : pose_result_ram
        if (pose_result_write_enable_a)
            pose_result_bank_a[pose_result_write_address]
                <= pose_result_write_data_a;
        if (pose_result_write_enable_b)
            pose_result_bank_b[pose_result_write_address]
                <= pose_result_write_data_b;
        pose_result_read_data_a
            <= pose_result_bank_a[pose_result_read_address_a];
        pose_result_read_data_b
            <= pose_result_bank_b[pose_result_read_address_b];
        pose_result_read_address_q <= pose_result_read_address_a[3:0];
    end

    // Phase-exclusive FIFO descriptor arbitration.
    always_comb begin
        fifo_desc_read_valid = 1'b0;
        fifo_desc_read_index = 4'b0;
        rect_desc_req_ready = 1'b0;
        if ((state == S_CORNER_START) || (state == S_CORNER_WAIT)) begin
            fifo_desc_read_valid = corner_desc_req_valid;
            fifo_desc_read_index = corner_desc_req_index;
        end else if (state == S_RECT_WAIT) begin
            fifo_desc_read_valid = rect_desc_req_valid;
            fifo_desc_read_index = rect_desc_req_index;
            rect_desc_req_ready = 1'b1;
        end
    end

    assign core_idle = (state == S_IDLE);
    assign core_dispatch_ready = core_idle && !rst;
    assign busy = !core_idle || fifo_head_valid;
    assign out_valid = (state == S_WAIT_PACKET);
    assign out_last = out_valid && (result_index
        == ((reject_reason == REJECT_NONE) ? 5'd21 : 5'd7));
    assign release_results = out_valid && out_ready && out_last;
    assign centroid_valid = EXPORT_ACCEPTED_CENTROIDS && (state == S_CENTROIDS);
    assign centroid_last = centroid_valid && (result_index == 5'd8);
    assign centroid_data = {pose_result_read_data_b[14:0], pose_result_read_data_a};

    always_comb begin : packet_mux
        out_data = 32'b0;
        if (reject_reason != REJECT_NONE) begin
            if (result_index == 5'd1) begin
                out_data = {1'b0, 1'b0, work_scale, work_frame,
                    4'b0, 16'b0};
            end else if (result_index == 5'd6) begin
                out_data = {27'b0, reject_reason};
            end
        end else if (result_index < 5'd10) begin
            out_data = packet_header_data;
        end else begin
            out_data = pose_result_data;
        end
    end

    always_ff @(posedge clk) begin : controller
        logic [39:0] normalization_trial;
        logic [39:0] normalization_difference;
        logic normalization_quotient_bit;

        normalization_trial = {normalization_remainder, 1'b0};
        normalization_difference = normalization_trial - {1'b0, scorer_best_weight_sum};
        // With remainder < D < 2^39, trial-D is in (-2^39,2^39),
        // so its top bit is the borrow flag without a separate comparator.
        normalization_quotient_bit = !normalization_difference[39];

        if (rst) begin
            state <= S_IDLE;
            config_index <= 3'b0;
            reject_reason <= REJECT_NONE;
            cycle_counter <= 10'b0;
            core_soft_reset <= 1'b0;
            work_scale <= 2'b0;
            work_frame <= 8'b0;
            candidate_popped <= 1'b0;
            fifo_head_pop <= 1'b0;
            scorer_clear <= 1'b0;
            result_index <= 5'b0;
            candidate_start <= 1'b0;
            candidate_done <= 1'b0;
            candidate_cycles <= 10'b0;
            mean_active <= 1'b0;
            mean_bit <= 5'b0;
            mean_quotient <= 32'b0;
            normalization_remainder <= 39'b0;
            header_fill_active <= 1'b0;
            header_fill_index <= 3'b0;
            mean_write_pending <= 1'b0;
        end else begin
            candidate_start <= 1'b0;
            candidate_done <= 1'b0;
            fifo_head_pop <= 1'b0;
            scorer_clear <= 1'b0;
            core_soft_reset <= 1'b0;

            if (header_fill_write_fire) begin
                if (header_fill_index == 3'd7) begin
                    header_fill_active <= 1'b0;
                end else begin
                    header_fill_index <= header_fill_index + 1'b1;
                end
            end
            if (mean_write_fire)
                mean_write_pending <= 1'b0;

            if (mean_active) begin
                normalization_remainder <= normalization_quotient_bit
                    ? normalization_difference[38:0] : normalization_trial[38:0];
                mean_quotient <= {mean_quotient[30:0], normalization_quotient_bit};
                if (mean_bit == 0) begin
                    mean_active <= 1'b0;
                    mean_write_pending <= 1'b1;
                end else begin
                    mean_bit <= mean_bit - 1'b1;
                end
            end

            if ((state != S_IDLE) && (state != S_PACKET)
                    && (state != S_WAIT_PACKET) && (state != S_CENTROIDS)
                    && (cycle_counter >= 10'd999)) begin
                reject_reason <= REJECT_WATCHDOG;
                scorer_clear <= 1'b1;
                core_soft_reset <= 1'b1;
                state <= S_PACKET;
            end else begin
                if ((state != S_IDLE) && (state != S_PACKET)
                        && (state != S_WAIT_PACKET) && (state != S_CENTROIDS))
                    cycle_counter <= cycle_counter + 1'b1;

                case (state)
                    S_IDLE: begin
                        if (fifo_head_valid) begin
                            work_scale <= fifo_head_scale;
                            work_frame <= fifo_head_frame;
                            candidate_popped <= 1'b0;
                            reject_reason <= REJECT_NONE;
                            cycle_counter <= 10'd1;
                            candidate_start <= 1'b1;
                            scorer_clear <= 1'b1;
                            mean_active <= 1'b0;
                            mean_quotient <= 32'b0;
                            header_fill_active <= 1'b0;
                            header_fill_index <= 3'b0;
                            mean_write_pending <= 1'b0;
                            if (EXTERNAL_CONFIG) begin
                                config_index <= 3'd7;
                                state <= S_CONFIG_REQUEST;
                            end else begin
                                state <= S_VALIDATE;
                            end
                        end
                    end

                    S_VALIDATE: begin
                        if (fifo_head_protocol) begin
                            reject_reason <= REJECT_PROTOCOL;
                            state <= S_PACKET;
                        end else if (fifo_head_mass_spread
                                > selected_max_mass_spread) begin
                            reject_reason <= REJECT_MASS_SPREAD;
                            state <= S_PACKET;
                        end else begin
                            state <= S_CORNER_START;
                        end
                    end

                    S_CORNER_START: begin
                        if (corner_in_ready)
                            state <= S_CORNER_WAIT;
                    end

                    S_CORNER_WAIT: begin
                        if (corner_out_valid) begin
                            if (!corner_out_input_valid
                                    || (corner_out_status == 3'd1)) begin
                                reject_reason <= REJECT_INVALID_CENTROID;
                                state <= S_PACKET;
                            end else begin
                                case (corner_out_status)
                                    3'd0: state <= S_RECT_START;
                                    3'd2: begin
                                        reject_reason <= REJECT_BASELINE_SHORT;
                                        state <= S_PACKET;
                                    end
                                    3'd3: begin
                                        reject_reason <= REJECT_DOMINANT_SIDE;
                                        state <= S_PACKET;
                                    end
                                    3'd5: begin
                                        reject_reason <= REJECT_INVALID_QUAD;
                                        state <= S_PACKET;
                                    end
                                    default: begin
                                        reject_reason <= REJECT_CORNER_SELECTION;
                                        state <= S_PACKET;
                                    end
                                endcase
                            end
                        end
                    end

                    S_RECT_START: begin
                        if (rect_start_ready)
                            state <= S_RECT_WAIT;
                    end

                    S_RECT_WAIT: begin
                        if (rect_done_valid) begin
                            if (!candidate_popped) begin
                                fifo_head_pop <= 1'b1;
                                candidate_popped <= 1'b1;
                            end
                            if (rect_done_invalid) begin
                                reject_reason <= REJECT_SINGULAR_MAP;
                                scorer_clear <= 1'b1;
                                state <= S_PACKET;
                            end else begin
                                state <= S_SCORE_WAIT;
                            end
                        end
                    end

                    S_SCORE_WAIT: begin
                        if (scorer_out_valid) begin
                            if (!scorer_best_valid || scorer_input_invalid) begin
                                reject_reason <= REJECT_NO_HYPOTHESIS;
                                state <= S_PACKET;
                            end else begin
                                mean_active <= 1'b1;
                                mean_bit <= 5'd31;
                                mean_quotient <= 32'b0;
                                normalization_remainder <= {1'b0, scorer_best_residual_sum};
                                header_fill_active <= 1'b1;
                                header_fill_index <= 3'b0;
                                state <= S_H_START;
                            end
                        end
                    end

                    S_H_START: begin
                        if (h_in_ready)
                            state <= S_H_WAIT;
                    end

                    S_H_WAIT: begin
                        if (h_out_valid && !mean_active) begin
                            if (EXTERNAL_CONFIG) begin
                                config_index <= 3'd1;
                                state <= S_CONFIG_REQUEST;
                            end else if (!mean_limit_pass) begin
                                reject_reason <= REJECT_RESIDUAL;
                                state <= S_PACKET;
                            end else if (h_out_invalid) begin
                                reject_reason <= REJECT_SINGULAR_MAP;
                                state <= S_PACKET;
                            end else begin
                                state <= S_POSE_START;
                            end
                        end
                    end

                    S_CONFIG_REQUEST: begin
                        if (config_read_ready)
                            state <= S_CONFIG_RESPONSE;
                    end

                    S_CONFIG_RESPONSE: begin
                        if (config_response_valid) begin
                            // Fetch only the selected limit or geometry word
                            // instead of reloading the configuration bank.
                            case (config_index)
                                3'd7: begin
                                    state <= S_VALIDATE;
                                end
                                3'd1: begin
                                    if (!mean_limit_pass) begin
                                        reject_reason <= REJECT_RESIDUAL;
                                        state <= S_PACKET;
                                    end else if (h_out_invalid) begin
                                        reject_reason <= REJECT_SINGULAR_MAP;
                                        state <= S_PACKET;
                                    end else begin
                                        config_index <= 3'd0;
                                        state <= S_CONFIG_REQUEST;
                                    end
                                end
                                default: begin
                                    state <= S_POSE_START;
                                end
                            endcase
                        end
                    end

                    S_POSE_START: begin
                        if (pose_in_ready)
                            state <= S_POSE_WAIT;
                    end

                    S_POSE_WAIT: begin
                        if (pose_out_valid) begin
                            if (!pose_valid)
                                reject_reason <= REJECT_SINGULAR_MAP;
                            state <= S_PACKET;
                        end
                    end

                    S_PACKET: begin
                        if (!candidate_popped) begin
                            fifo_head_pop <= 1'b1;
                            candidate_popped <= 1'b1;
                        end
                        result_index <= 5'b0;
                        candidate_cycles <= cycle_counter;
                        candidate_done <= 1'b1;
                        state <= S_WAIT_PACKET;
                    end

                    S_WAIT_PACKET: begin
                        if (out_ready) begin
                            if (out_last) begin
                                result_index <= 5'b0;
                                state <= (EXPORT_ACCEPTED_CENTROIDS && (reject_reason == REJECT_NONE))
                                    ? S_CENTROIDS : S_IDLE;
                            end else begin
                                result_index <= result_index + 1'b1;
                            end
                        end
                    end

                    S_CENTROIDS: begin
                        if (centroid_ready) begin
                            if (centroid_last) begin
                                result_index <= '0;
                                state <= S_IDLE;
                            end else begin
                                result_index <= result_index + 1'b1;
                            end
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

`ifndef SYNTHESIS
    logic [8:0] centroid_cache_seen;
    always_ff @(posedge clk) begin
        if (rst || ((state == S_IDLE) && fifo_head_valid))
            centroid_cache_seen <= '0;
        else if (EXPORT_ACCEPTED_CENTROIDS && state == S_CORNER_WAIT && fifo_desc_data_valid)
            centroid_cache_seen[centroid_descriptor_index] <= 1'b1;
        if (!rst && EXPORT_ACCEPTED_CENTROIDS) begin
            if (state == S_PACKET && reject_reason == REJECT_NONE)
                assert (&centroid_cache_seen) else $error("accepted centroid cache incomplete");
            if (pose_result_write_enable_a && pose_result_write_address >= 32
                    && pose_result_write_address <= 40)
                assert (state == S_CORNER_WAIT && fifo_desc_data_valid)
                    else $error("centroid cache overwritten by numeric datapath");
        end
    end
    always_ff @(posedge clk) begin
        if (!rst) begin
            if (state == S_H_START) begin
                // Legal nearest-cell residuals have N/D < 1. The divider
                // emits 32 fractional bits with no integer or saturation path.
                assert (scorer_best_weight_sum != 39'b0)
                    else $error("normalized residual denominator is zero");
                assert ({1'b0, scorer_best_residual_sum} < scorer_best_weight_sum)
                    else $error("normalized residual exceeds divider range");
            end
            if (mean_active)
                assert (normalization_remainder < scorer_best_weight_sum)
                    else $error("normalized residual remainder exceeds denominator");
            assert (fifo_level <= 6'd32);
            assert (!(fifo_head_pop && !fifo_head_valid));
            assert (!(out_valid && (result_index > 5'd21)));
            if ((state == S_PACKET) && (reject_reason == REJECT_NONE)) begin
                assert (!header_fill_active);
                assert (!mean_write_pending);
            end
        end
    end
`endif

endmodule
