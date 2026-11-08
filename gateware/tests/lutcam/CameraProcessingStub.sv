`default_nettype none

// Control-test double only. The actual MonoIsp and transport-facing camera
// wrapper are exercised, but this module does not compress or solve images.
// Its busy tail makes frame ownership observable after the last input pixel.
module TagDetectLwc #(
    parameter bit USE_DSP_HIGH_ZOM = 1'b0,
    parameter bit USE_DSP_HIGH_FOM = 1'b0,
    parameter bit ENABLE_PROTOCOL_DIAGNOSTICS = 1'b1,
    parameter bit EXTERNAL_LFTAG = 1'b0,
    parameter string GROUP_FAMILY_SCHEDULE_BIN = "",
    parameter string CENTROID_RECIPROCAL_SEGMENT_BIN = "",
    parameter string CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN = "",
    parameter bit BOUNDED_BYTE_RASTER = 1'b0
) (
    input wire clk, rst,
    input wire in_vref, in_href,
    input wire [7:0] in_d0, in_d1,
    output wire inner_fv_valid,
    output wire [32:0] inner_fv_data,
    output wire hier_inner_end,
    output wire lwc_out_sof, lwc_out_valid, lwc_out_padding,
    output wire [15:0] lwc_out_data,
    output wire lwc_protocol_error,
    input wire [31:0] lftag_cfg_inv_fx_q2_30, lftag_cfg_inv_fy_q2_30,
    input wire signed [31:0] lftag_cfg_cx_q16_16, lftag_cfg_cy_q16_16,
    input wire [31:0] lftag_cfg_printed_width_q16_16,
    input wire [15:0] lftag_cfg_mean_residual_limit_q16,
    input wire [7:0] lftag_cfg_max_mass_spread,
    input wire [15:0] lftag_cfg_min_baseline_q4,
    input wire lftag_result_ready,
    input wire lftag_clear_diagnostics,
    output wire lftag_result_valid,
    output wire [31:0] lftag_result_data,
    output wire lftag_result_last, lftag_busy,
    output wire [5:0] lftag_fifo_level,
    output wire lftag_candidate_start, lftag_candidate_done,
    output wire [9:0] lftag_candidate_cycles,
    output wire lftag_fifo_drop_sticky, lftag_protocol_error_sticky,
    output wire lftag_in_valid, lftag_in_group_end,
    output wire [14:0] lftag_in_x_q4,
    output wire [13:0] lftag_in_y_q4,
    output wire [7:0] lftag_in_log_mass_q5_3,
    output wire lftag_in_centroid_valid,
    output wire [1:0] lftag_in_scale,
    output wire [7:0] lftag_in_frame_seq,
    input wire external_lftag_result_valid,
    input wire [31:0] external_lftag_result_data,
    input wire external_lftag_result_last, external_lftag_busy,
    input wire [5:0] external_lftag_fifo_level,
    input wire external_lftag_candidate_start, external_lftag_candidate_done,
    input wire [9:0] external_lftag_candidate_cycles,
    input wire external_lftag_fifo_drop_sticky,
    input wire external_lftag_protocol_error_sticky,
    output wire pipeline_busy
);
    integer received_pairs = 0;
    integer frames_seen = 0, frames_retired = 0;
    integer cycle_number = 0, frame_start_cycle = 0;
    integer tail = 0;
    logic was_vref = 0;
    logic check_bypass_pixels = 1;
    logic check_raster_cadence = 1;
    // Testbench-driven pulses create sticky child faults. The wrapper's
    // fault reset clears these raw flags, just as the real processing core
    // would, so diagnostics must be retained outside that reset domain.
    logic inject_lwc = 0, inject_drop = 0, inject_protocol = 0;
    logic raw_lwc_fault = 0, raw_drop_fault = 0, raw_protocol_fault = 0;
    function automatic [7:0] expected0(input integer index);
        expected0 = 8'((10'((index * 29 + 3) ^ (index >> 2))) >> 2);
    endfunction
    function automatic [7:0] expected1(input integer index);
        expected1 = 8'((10'((index * 47 + 101) ^ (index >> 3))) >> 2);
    endfunction

    assign pipeline_busy = in_vref || tail != 0 || (EXTERNAL_LFTAG
        && (external_lftag_busy || external_lftag_fifo_level != 0
            || external_lftag_result_valid));
    assign lftag_busy = EXTERNAL_LFTAG ? external_lftag_busy : tail != 0;
    assign inner_fv_valid = 0;
    assign inner_fv_data = 0;
    assign hier_inner_end = 0;
    assign lwc_out_sof = 0;
    assign lwc_out_valid = 0;
    assign lwc_out_padding = 0;
    assign lwc_out_data = 0;
    assign lwc_protocol_error = raw_lwc_fault;
    assign lftag_result_valid = EXTERNAL_LFTAG ? external_lftag_result_valid : 0;
    assign lftag_result_data = EXTERNAL_LFTAG ? external_lftag_result_data : 0;
    assign lftag_result_last = EXTERNAL_LFTAG ? external_lftag_result_last : 0;
    assign lftag_fifo_level = EXTERNAL_LFTAG ? external_lftag_fifo_level : 0;
    assign lftag_candidate_start = EXTERNAL_LFTAG ? external_lftag_candidate_start : 0;
    assign lftag_candidate_done = EXTERNAL_LFTAG ? external_lftag_candidate_done : 0;
    assign lftag_candidate_cycles = EXTERNAL_LFTAG ? external_lftag_candidate_cycles : 0;
    assign lftag_fifo_drop_sticky = EXTERNAL_LFTAG
        ? external_lftag_fifo_drop_sticky : raw_drop_fault;
    assign lftag_protocol_error_sticky = EXTERNAL_LFTAG
        ? external_lftag_protocol_error_sticky : raw_protocol_fault;
    // This neutral processing model generates no centroid candidates.
    assign lftag_in_valid = 0;
    assign lftag_in_group_end = 0;
    assign lftag_in_x_q4 = 0;
    assign lftag_in_y_q4 = 0;
    assign lftag_in_log_mass_q5_3 = 0;
    assign lftag_in_centroid_valid = 0;
    assign lftag_in_scale = 0;
    assign lftag_in_frame_seq = 0;

    always_ff @(posedge clk) begin
        if (rst) begin
            raw_lwc_fault <= 0;
            raw_drop_fault <= 0;
            raw_protocol_fault <= 0;
        end else begin
            if (inject_lwc) raw_lwc_fault <= 1;
            if (inject_drop) raw_drop_fault <= 1;
            if (inject_protocol) raw_protocol_fault <= 1;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            received_pairs = 0;
            frames_seen = 0;
            frames_retired = 0;
            cycle_number = 0;
            frame_start_cycle = 0;
            tail = 0;
            was_vref = 0;
        end else begin
            if (in_href && !in_vref)
                $fatal(1, "processing input HREF outside VREF");
            if (in_vref && !was_vref) begin
                if (!in_href || tail != 0)
                    $fatal(1, "new frame does not start with its first pixel or overlaps drain");
                received_pairs = 0;
                frame_start_cycle = cycle_number;
                frames_seen = frames_seen + 1;
            end
            if (in_vref && check_raster_cadence) begin
                if (in_href !== ((cycle_number - frame_start_cycle) % 896 < 640))
                    $fatal(1, "processing line cadence changed at cycle %0d", cycle_number);
            end
            if (in_href) begin
                if (received_pairs >= 512000)
                    $fatal(1, "processing received too many pairs");
                if (check_bypass_pixels && (in_d0 !== expected0(received_pairs)
                        || in_d1 !== expected1(received_pairs)))
                    $fatal(1, "ISP bypass data mismatch at pair %0d: %h %h",
                           received_pairs, in_d0, in_d1);
                received_pairs = received_pairs + 1;
            end
            if (in_vref)
                tail = 32;
            else if (tail != 0)
                tail = tail - 1;
            if (was_vref && !in_vref) begin
                if (received_pairs != 512000)
                    $fatal(1, "processing frame ended with %0d pairs", received_pairs);
                frames_retired = frames_retired + 1;
            end
            was_vref = in_vref;
            cycle_number = cycle_number + 1;
        end
    end
endmodule

`default_nettype wire
