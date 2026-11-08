`default_nettype none

// One camera: RAW10 directly through the nonblocking MonoIsp into an
// independent image/child/pose pipeline. Canonical defect tables preserve
// the receiver cadence with only the ISP's fixed one-clock output register.
module CameraPipeline #(
  // Main Titanium profile only; generic targets retain fabric arithmetic.
  parameter bit USE_DSP_HIGH_ZOM = 1'b0,
  parameter bit USE_DSP_HIGH_FOM = 1'b0,
    parameter bit PAIRED_MODE = 1'b0,
    parameter bit USE_DSP_COUNTERS = 1'b0,
    parameter bit EXTERNAL_LFTAG = 1'b0,
    parameter string GROUP_FAMILY_SCHEDULE_BIN =
        "rtl/lwc/raw_group_family_schedule.bin",
    parameter string CENTROID_RECIPROCAL_SEGMENT_BIN =
        "rtl/lrcca/centroid_reciprocal_segments_q17.bin",
    parameter string CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN =
        "rtl/lrcca/centroid_reciprocal_midpoint_reflection_q17.bin"
) (
    input wire clk, rst,
    input wire [19:0] pixel_data,
    input wire pixel_valid, pixel_sof, pixel_eol, pixel_eof,
    input wire receiver_fault,
    input wire run_request, capture_permit, clear_diagnostics,
    input wire pair_abort,
    input wire [31:0] capture_frame_id,
    output wire admission_ready, frame_admitted, frame_retired, local_fault,
    output logic active,
    output wire idle,
    output logic fault_sticky,
    output logic [31:0] frame_id, frames_started, frames_completed,
    output wire [15:0] ingress_high_water,
    input wire wb_cyc_i, wb_stb_i, wb_we_i,
    input wire [3:0] wb_sel_i,
    input wire [15:0] wb_adr_i,
    input wire [31:0] wb_dat_i,
    output wire [31:0] wb_dat_o,
    output wire wb_ack_o, wb_err_o,
    input wire hist_dump_req,
    output wire hist_dump_ready, hist_out_valid,
    input wire hist_out_ready,
    output wire [31:0] hist_out_data,
    output wire inner_fv_valid,
    output wire [32:0] inner_fv_data,
    output wire hier_inner_end,
    output wire lwc_out_sof, lwc_out_valid, lwc_out_padding,
    output wire [15:0] lwc_out_data,
    output wire lwc_protocol_error,
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
    output wire         lftag_processing_reset,
    input  wire         external_lftag_result_valid,
    input  wire  [31:0] external_lftag_result_data,
    input  wire         external_lftag_result_last,
    input  wire         external_lftag_busy,
    input  wire  [5:0]  external_lftag_fifo_level,
    input  wire         external_lftag_candidate_start,
    input  wire         external_lftag_candidate_done,
    input  wire  [9:0]  external_lftag_candidate_cycles,
    input  wire         external_lftag_fifo_drop_sticky,
    input  wire         external_lftag_protocol_error_sticky
);
    wire local_reset = rst || fault_sticky;
    wire diagnostic_clear = clear_diagnostics && !run_request && !active;
    wire isp_in_valid, isp_in_ready, isp_in_sof, isp_in_eol;
    wire [9:0] isp_in_pixel0, isp_in_pixel1;
    wire isp_out_valid, isp_out_ready, isp_out_sof, isp_out_eol;
    wire [7:0] isp_out_pixel0, isp_out_pixel1;
    wire isp_protocol_error, core_busy;
    wire isp_position_sof, isp_position_eol, isp_position_eof;
    wire [31:0] solver_data;
    logic [4:0] solver_word_index;
    wire raw_lwc_protocol_error, raw_lftag_fifo_drop, raw_lftag_protocol_error;
    wire isp_hist_dump_ready;
    wire histogram_allowed = !local_reset;
    logic retained_lwc_protocol_error, retained_lftag_fifo_drop;
    logic retained_lftag_protocol_error;
    wire core_valid, core_sof, core_eol;
    wire [7:0] core_pixel0, core_pixel1;
    logic source_frame_active, core_frame_active, frame_pixels_done;
    // The ISP owns the accepted-pair raster. Its three position flags
    // replace duplicate source counters; every malformed prefix cancels here
    // on the same edge. Track only the final marker across the fixed one-cycle
    // ISP output register. Fault priority clears this marker with the frame.
    logic core_last_pair;
    logic [2:0] drain_quiet;
    wire candidate_begin = pixel_valid && pixel_sof && run_request
        && capture_permit && idle && isp_in_ready;
    wire begin_frame = candidate_begin && !pair_abort;
    // The reusable processing closure is verified from reset for each frame.
    // Reinitialize at frame retirement, before admitting the next SOF. The
    // hierarchy's resettable initial-ID prefix immediately restores its free
    // pool without a RAM initialization walk. MonoIsp retains its histogram,
    // configuration, and tables across this processing reset.
    wire retire_frame = active && frame_pixels_done && !isp_out_valid
        && !core_busy && (drain_quiet == 3'd7);
    wire processing_reset = local_reset || retire_frame;
    assign lftag_processing_reset = processing_reset;
    wire candidate_accept = pixel_valid && (source_frame_active || candidate_begin)
        && !fault_sticky;
    wire accept_pixel = candidate_accept && !pair_abort;
    wire isp_input_stall = candidate_accept && !isp_in_ready;
    wire core_eof = core_valid && core_eol && core_last_pair;
    wire marker_fault = candidate_accept
        && ((pixel_sof != isp_position_sof)
            || (pixel_eol != isp_position_eol)
            || (pixel_eof != isp_position_eof));
    wire start_fault = pixel_valid && pixel_sof && run_request
        && capture_permit && !candidate_begin;
    // Malformed LFTag groups are rejected by the decoder before numeric
    // processing. Retain their diagnostic below, but keep image capture
    // running; receiver/codec faults and candidate queue loss remain fatal.
    assign local_fault = (active && (!capture_permit || receiver_fault))
        || marker_fault || start_fault || isp_input_stall || isp_protocol_error
        || lwc_protocol_error || lftag_fifo_drop_sticky;
    wire stream_fault = local_fault || pair_abort;
    assign idle = !active && !fault_sticky && !isp_out_valid && !core_busy;
    assign admission_ready = idle && isp_in_ready && capture_permit && !pair_abort;
    assign frame_admitted = begin_frame;
    assign frame_retired = retire_frame && !stream_fault && !fault_sticky;
    // Retained register-map compatibility: there is no extra ingress FIFO.
    assign ingress_high_water = 16'd0;
    assign isp_in_valid = accept_pixel;
    assign isp_in_pixel0 = pixel_data[9:0];
    assign isp_in_pixel1 = pixel_data[19:10];
    assign isp_in_sof = pixel_sof;
    assign isp_in_eol = pixel_eol;
    assign isp_out_ready = 1'b1;
    assign core_valid = isp_out_valid;
    assign core_sof = isp_out_sof;
    assign core_eol = isp_out_eol;
    assign core_pixel0 = isp_out_pixel0;
    assign core_pixel1 = isp_out_pixel1;
    assign hist_dump_ready = isp_hist_dump_ready && histogram_allowed;

    // Preserve the public cause bits when a capture fault resets the child
    // processing logic, and across the normal per-frame boundary reset.
    assign lwc_protocol_error = raw_lwc_protocol_error || retained_lwc_protocol_error;
    assign lftag_fifo_drop_sticky = raw_lftag_fifo_drop || retained_lftag_fifo_drop;
    assign lftag_protocol_error_sticky = raw_lftag_protocol_error
        || retained_lftag_protocol_error;
    always_ff @(posedge clk) begin
        if (rst || diagnostic_clear) begin
            retained_lwc_protocol_error <= 1'b0;
            retained_lftag_fifo_drop <= 1'b0;
            retained_lftag_protocol_error <= 1'b0;
        end else begin
            retained_lwc_protocol_error <= retained_lwc_protocol_error
                || raw_lwc_protocol_error;
            retained_lftag_fifo_drop <= retained_lftag_fifo_drop
                || raw_lftag_fifo_drop;
            retained_lftag_protocol_error <= retained_lftag_protocol_error
                || raw_lftag_protocol_error;
        end
    end

    // The core's private eight-bit sequence restarts at each boundary reset.
    // Keep the native packet header's frame byte consistent with
    // the authoritative 32-bit transport frame ID. Track accepted words so
    // the replacement remains stable under result backpressure.
    assign lftag_result_data = (solver_word_index == 5'd1)
        ? {solver_data[31:28], frame_id[7:0], solver_data[19:0]}
        : solver_data;
    always_ff @(posedge clk) begin
        if (processing_reset)
            solver_word_index <= '0;
        else if (lftag_result_valid && lftag_result_ready)
            solver_word_index <= lftag_result_last ? '0 : solver_word_index + 1'b1;
    end

    // ISP programming is allowed only outside an owned frame. This prevents
    // a software stop/table update from interrupting the sensor's timing.
    wire block_write = active && wb_we_i;
    wire isp_wb_ack, isp_wb_err;
    logic blocked_ack;
    assign wb_ack_o = isp_wb_ack;
    assign wb_err_o = isp_wb_err || blocked_ack;
    always_ff @(posedge clk) begin
        if (rst) blocked_ack <= 1'b0;
        else blocked_ack <= wb_cyc_i && wb_stb_i && block_write;
    end

    MonoIsp isp (
        .clk, .rst(local_reset),
        .wb_cyc_i(wb_cyc_i && !block_write),
        .wb_stb_i(wb_stb_i && !block_write),
        .wb_we_i, .wb_sel_i, .wb_adr_i, .wb_dat_i, .wb_dat_o,
        .wb_ack_o(isp_wb_ack), .wb_err_o(isp_wb_err),
        .in_valid(isp_in_valid), .in_ready(isp_in_ready),
        .in_pixel0(isp_in_pixel0), .in_pixel1(isp_in_pixel1),
        .in_sof(isp_in_sof), .in_eol(isp_in_eol),
        .out_valid(isp_out_valid), .out_ready(isp_out_ready),
        .out_pixel0(isp_out_pixel0), .out_pixel1(isp_out_pixel1),
        .out_sof(isp_out_sof), .out_eol(isp_out_eol),
        .hist_dump_req(hist_dump_req && histogram_allowed),
        .hist_dump_ready(isp_hist_dump_ready),
        .hist_out_valid, .hist_out_ready, .hist_out_data,
        .protocol_error(isp_protocol_error),
        .input_position_sof(isp_position_sof),
        .input_position_eol(isp_position_eol),
        .input_position_eof(isp_position_eof)
    );

    always_ff @(posedge clk) begin
        if (rst) begin
            active <= 1'b0;
            fault_sticky <= 1'b0;
            source_frame_active <= 1'b0;
            core_frame_active <= 1'b0;
            frame_pixels_done <= 1'b0;
            core_last_pair <= 1'b0;
            drain_quiet <= '0;
            frame_id <= '0;
        end else begin
            core_last_pair <= isp_in_valid && isp_in_ready && isp_position_eof;
            if (diagnostic_clear) begin
                fault_sticky <= 1'b0;
                // Global paired CLEAR starts both cameras' accounting at
                // the same origin without resetting retained ISP settings.
                if (PAIRED_MODE) begin
                    frame_id <= '0;
                end
            end
            if (begin_frame) begin
                active <= 1'b1;
                source_frame_active <= 1'b1;
                frame_pixels_done <= 1'b0;
                frame_id <= PAIRED_MODE ? capture_frame_id : frames_started;
            end
            if (accept_pixel) begin
                if (pixel_eof) source_frame_active <= 1'b0;
            end
            if (core_valid) begin
                if (core_sof) core_frame_active <= 1'b1;
                if (core_eof) begin
                    core_frame_active <= 1'b0;
                    frame_pixels_done <= 1'b1;
                end
            end
            if (active && frame_pixels_done && !isp_out_valid && !core_busy) begin
                if (drain_quiet == 3'd7) begin
                    active <= 1'b0;
                    frame_pixels_done <= 1'b0;
                end else drain_quiet <= drain_quiet + 1'b1;
            end else drain_quiet <= '0;
            if (stream_fault || fault_sticky) begin
                fault_sticky <= 1'b1;
                active <= 1'b0;
                source_frame_active <= 1'b0;
                core_frame_active <= 1'b0;
                frame_pixels_done <= 1'b0;
                core_last_pair <= 1'b0;
                drain_quiet <= '0;
                // Clearing is valid only after software has disarmed capture.
                if (diagnostic_clear)
                    fault_sticky <= 1'b0;
            end
        end
    end

    // Accounting events precede fault handling, as in the original controller.
    CameraEventCounter #(.USE_DSP(USE_DSP_COUNTERS)) started_counter (
        .clk, .rst, .clear(PAIRED_MODE && diagnostic_clear),
        .enable(begin_frame), .value(frames_started)
    );
    CameraEventCounter #(.USE_DSP(USE_DSP_COUNTERS)) completed_counter (
        .clk, .rst, .clear(PAIRED_MODE && diagnostic_clear),
        .enable(retire_frame), .value(frames_completed)
    );

    // Source markers are checked before the registered ISP output enters
    // processing. A source fault or normal retirement resets this closure.
    TagDetectLwc #(
        .USE_DSP_HIGH_ZOM(USE_DSP_HIGH_ZOM),
    .USE_DSP_HIGH_FOM(USE_DSP_HIGH_FOM),
        .BOUNDED_BYTE_RASTER(1'b1),
        .EXTERNAL_LFTAG(EXTERNAL_LFTAG),
        .GROUP_FAMILY_SCHEDULE_BIN(GROUP_FAMILY_SCHEDULE_BIN),
        .CENTROID_RECIPROCAL_SEGMENT_BIN(CENTROID_RECIPROCAL_SEGMENT_BIN),
        .CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN(
            CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN)
    ) processing (
        .clk, .rst(processing_reset),
        .in_vref(core_frame_active || (core_valid && core_sof)),
        .in_href(core_valid), .in_d0(core_pixel0), .in_d1(core_pixel1),
        .inner_fv_valid, .inner_fv_data, .hier_inner_end,
        .lwc_out_sof, .lwc_out_valid, .lwc_out_padding,
        .lwc_out_data, .lwc_protocol_error(raw_lwc_protocol_error),
        .lftag_cfg_inv_fx_q2_30, .lftag_cfg_inv_fy_q2_30,
        .lftag_cfg_cx_q16_16, .lftag_cfg_cy_q16_16,
        .lftag_cfg_printed_width_q16_16, .lftag_cfg_mean_residual_limit_q16,
        .lftag_cfg_max_mass_spread, .lftag_cfg_min_baseline_q4,
        .lftag_result_ready,
        .lftag_clear_diagnostics, .lftag_result_valid,
        .lftag_result_data(solver_data),
        .lftag_result_last, .lftag_busy, .lftag_fifo_level,
        .lftag_candidate_start, .lftag_candidate_done, .lftag_candidate_cycles,
        .lftag_fifo_drop_sticky(raw_lftag_fifo_drop),
        .lftag_protocol_error_sticky(raw_lftag_protocol_error),
        .lftag_in_valid, .lftag_in_group_end,
        .lftag_in_x_q4, .lftag_in_y_q4, .lftag_in_log_mass_q5_3,
        .lftag_in_centroid_valid, .lftag_in_scale,
        .lftag_in_frame_seq,
        .external_lftag_result_valid, .external_lftag_result_data,
        .external_lftag_result_last, .external_lftag_busy,
        .external_lftag_fifo_level, .external_lftag_candidate_start,
        .external_lftag_candidate_done, .external_lftag_candidate_cycles,
        .external_lftag_fifo_drop_sticky, .external_lftag_protocol_error_sticky,
        .pipeline_busy(core_busy)
    );

endmodule

// A local diagnostic clear yields to a simultaneous event; global reset wins.
module CameraEventCounter #(
    parameter bit USE_DSP = 1'b0
) (
    input wire clk, rst, clear, enable,
    output wire [31:0] value
);
    generate if (USE_DSP) begin : native_counter
        wire [47:0] accumulator;
        EFX_DSP48 #(
            .MODE("NORMAL"), .SIGNED(1'b0), .M_SEL("C"), .N_SEL("W"),
            .W_SEL("X"), .W_REG(1'b1), .RST_SYNC(1'b1)
        ) counter (
            .A(19'b0), .B(18'b0), .C(18'd1), .CASCIN(48'b0),
            .OP(2'b0), .SHIFT_ENA(1'b0), .CLK(clk), .CE(enable),
            .RST(rst || (clear && !enable)), .O(accumulator),
            .CASCOUT(), .OVFL()
        );
        assign value = accumulator[31:0];
    end else begin : fabric_counter
        reg [31:0] accumulator;
        always @(posedge clk) begin
            if (rst) accumulator <= 32'b0;
            else if (enable) accumulator <= accumulator + 1'b1;
            else if (clear) accumulator <= 32'b0;
        end
        assign value = accumulator;
    end endgenerate
endmodule
`default_nettype wire
