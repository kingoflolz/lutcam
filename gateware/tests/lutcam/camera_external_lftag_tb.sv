`timescale 1ns/1ps
`default_nettype none

// Real CSI-facing control, ISP and detection frontend; only the shared
// solver's return interface is driven by this boundary test.
module camera_external_lftag_tb;
    logic clk = 0;
    always #5 clk = !clk;
    logic rst = 1, run_request = 0, clear_diagnostics = 0;
    logic pixel_valid = 0, pixel_sof = 0, pixel_eol = 0, pixel_eof = 0;
    logic isp_enable_write = 0;
    wire wb_ack_o, wb_err_o;
    wire active, idle, fault_sticky, frame_retired;
    wire [31:0] frame_id, frames_started, frames_completed;
    wire lftag_processing_reset, lftag_result_valid, lftag_result_last;
    wire [31:0] lftag_result_data;
    wire lftag_busy, lftag_candidate_start, lftag_candidate_done;
    wire [5:0] lftag_fifo_level;
    wire [9:0] lftag_candidate_cycles;
    wire lftag_fifo_drop_sticky, lftag_protocol_error_sticky;
    wire lftag_in_valid, lftag_in_group_end, lftag_in_centroid_valid;
    wire [14:0] lftag_in_x_q4;
    wire [13:0] lftag_in_y_q4;
    wire [7:0] lftag_in_log_mass_q5_3, lftag_in_frame_seq;
    wire [1:0] lftag_in_scale;
    logic lftag_result_ready = 0;
    logic external_lftag_result_valid = 0, external_lftag_result_last = 0;
    logic [31:0] external_lftag_result_data = 0;
    logic external_lftag_busy = 0;
    logic [5:0] external_lftag_fifo_level = 0;
    logic external_lftag_candidate_start = 0, external_lftag_candidate_done = 0;
    logic [9:0] external_lftag_candidate_cycles = 0;
    logic external_lftag_fifo_drop_sticky = 0, external_lftag_protocol_error_sticky = 0;
    integer retired_pulses = 0, reset_pulses = 0;
    logic last_processing_reset = 1;

    CameraPipeline #(.PAIRED_MODE(1), .EXTERNAL_LFTAG(1)) dut (
        .clk, .rst, .pixel_data(20'h80200),
        .pixel_valid, .pixel_sof, .pixel_eol, .pixel_eof,
        .receiver_fault(1'b0), .run_request, .capture_permit(1'b1),
        .clear_diagnostics, .pair_abort(1'b0),
        .capture_frame_id(32'h123456a5),
        .active, .idle, .fault_sticky, .frame_id, .frames_started, .frames_completed,
        .frame_retired,
        .wb_cyc_i(isp_enable_write), .wb_stb_i(isp_enable_write), .wb_we_i(isp_enable_write),
        .wb_sel_i(4'hf), .wb_adr_i(16'h0004), .wb_dat_i(32'd1), .wb_ack_o, .wb_err_o,
        .hist_dump_req(1'b0), .hist_out_ready(1'b1),
        .lftag_cfg_inv_fx_q2_30(32'd0), .lftag_cfg_inv_fy_q2_30(32'd0),
        .lftag_cfg_cx_q16_16(32'd0), .lftag_cfg_cy_q16_16(32'd0),
        .lftag_cfg_printed_width_q16_16(32'd0),
        .lftag_cfg_mean_residual_limit_q16(16'd0),
        .lftag_cfg_max_mass_spread(8'd0), .lftag_cfg_min_baseline_q4(16'd0),
        .lftag_clear_diagnostics(clear_diagnostics),
        .lftag_result_ready, .lftag_result_valid, .lftag_result_data, .lftag_result_last,
        .lftag_busy, .lftag_fifo_level, .lftag_candidate_start, .lftag_candidate_done,
        .lftag_candidate_cycles, .lftag_fifo_drop_sticky, .lftag_protocol_error_sticky,
        .lftag_in_valid, .lftag_in_group_end, .lftag_in_x_q4, .lftag_in_y_q4,
        .lftag_in_log_mass_q5_3, .lftag_in_centroid_valid, .lftag_in_scale,
        .lftag_in_frame_seq, .lftag_processing_reset,
        .external_lftag_result_valid, .external_lftag_result_data,
        .external_lftag_result_last, .external_lftag_busy, .external_lftag_fifo_level,
        .external_lftag_candidate_start, .external_lftag_candidate_done,
        .external_lftag_candidate_cycles, .external_lftag_fifo_drop_sticky,
        .external_lftag_protocol_error_sticky
    );

    always @(posedge clk) begin
        if (!rst) begin
            if (frame_retired) retired_pulses = retired_pulses + 1;
            if (lftag_processing_reset && !last_processing_reset)
                reset_pulses = reset_pulses + 1;
        end
        last_processing_reset = lftag_processing_reset;
    end

    task automatic tick;
        @(posedge clk); #1;
        @(negedge clk);
    endtask

    task automatic expect_owned;
        if (!active || frames_completed != 0 || frame_id != 32'h123456a5
                || lftag_processing_reset || fault_sticky)
            $fatal(1, "external work lost frame ownership before final output");
    endtask

    // Force only the native hierarchy result boundary to exercise conversion
    // and forwarding. The full-frame test below releases these forces.
    logic inject_valid = 0, inject_end = 0, inject_centroid_valid = 1;
    logic [14:0] inject_x = 0;
    logic [13:0] inject_y = 0;
    logic [7:0] inject_mass = 8'h91;
    task automatic check_coordinate(input logic [14:0] composite_x,
            input logic [13:0] y, input logic [1:0] scale,
            input logic [14:0] logical_x, input logic valid);
        inject_x = composite_x;
        inject_y = y;
        #1;
        if (!lftag_in_valid || lftag_in_x_q4 != logical_x || lftag_in_y_q4 != y
                || lftag_in_scale != scale || lftag_in_centroid_valid != valid
                || lftag_in_log_mass_q5_3 != inject_mass)
            $fatal(1, "native coordinate forwarding mismatch at x=%0d y=%0d", composite_x, y);
    endtask

    integer timeout;
    initial begin
        repeat (5) tick();
        if (!lftag_processing_reset) $fatal(1, "reset export missing");
        rst = 0;
        repeat (10) tick();
        force dut.processing.centroid_group_valid = inject_valid;
        force dut.processing.centroid_group_end = inject_end;
        force dut.processing.centroid_group_centroid_valid = inject_centroid_valid;
        force dut.processing.centroid_group_x_q4 = inject_x;
        force dut.processing.centroid_group_y_q4 = inject_y;
        force dut.processing.centroid_group_log_m_q5_3 = inject_mass;
        #1;
        if (lftag_in_valid || lftag_in_group_end)
            $fatal(1, "inactive native group asserted valid or end");
        inject_valid = 1;
        check_coordinate(15'd320, 14'd100, 2'd0, 15'd320, 1'b1);
        inject_centroid_valid = 0;
        check_coordinate(15'd320, 14'd100, 2'd0, 15'd320, 1'b0);
        inject_centroid_valid = 1;
        check_coordinate(15'd20704, 14'd200, 2'd1, 15'd160, 1'b1);
        check_coordinate(15'd25824, 14'd40, 2'd2, 15'd32, 1'b1);
        check_coordinate(15'd20480, 14'd0, 2'd1, 15'(20480-20544), 1'b0);
        check_coordinate(15'd25728, 14'd0, 2'd2, 15'(25728-25792), 1'b0);
        check_coordinate(15'd320, 14'd12800, 2'd0, 15'd320, 1'b0);
        check_coordinate(15'd20704, 14'd3200, 2'd1, 15'd160, 1'b0);
        check_coordinate(15'd25824, 14'd800, 2'd2, 15'd32, 1'b0);
        inject_end = 1;
        #1;
        if (!lftag_in_group_end)
            $fatal(1, "group end was not forwarded");
        tick();
        inject_end = 0;
        #1;
        if (lftag_in_group_end)
            $fatal(1, "end leaked into next group");
        inject_valid = 0;
        #1;
        if (lftag_in_valid) $fatal(1, "native valid remained asserted after group");
        release dut.processing.centroid_group_valid;
        release dut.processing.centroid_group_end;
        release dut.processing.centroid_group_centroid_valid;
        release dut.processing.centroid_group_x_q4;
        release dut.processing.centroid_group_y_q4;
        release dut.processing.centroid_group_log_m_q5_3;

        rst = 1;
        repeat (5) tick();
        rst = 0;
        isp_enable_write = 1;
        tick();
        if (!wb_ack_o || wb_err_o) $fatal(1, "ISP bypass initialization failed");
        isp_enable_write = 0;
        timeout = 0;
        while (!dut.admission_ready && timeout < 20000) begin tick(); timeout++; end
        if (!dut.admission_ready) $fatal(1, "ISP did not become ready");
        external_lftag_candidate_start = 1;
        external_lftag_candidate_done = 1;
        external_lftag_candidate_cycles = 10'd777;
        #1;
        if (!lftag_candidate_start || !lftag_candidate_done || lftag_candidate_cycles != 777)
            $fatal(1, "external diagnostic event forwarding mismatch");
        external_lftag_candidate_start = 0;
        external_lftag_candidate_done = 0;
        run_request = 1;
        for (integer row = 0; row < 800; row++) begin
            for (integer column = 0; column < 896; column++) begin
                pixel_valid = column < 640;
                pixel_sof = row == 0 && column == 0;
                pixel_eol = column == 639;
                pixel_eof = row == 799 && column == 639;
                tick();
                if (row == 0 && column == 0) external_lftag_busy = 1;
            end
        end
        pixel_valid = 0;
        pixel_sof = 0;
        pixel_eol = 0;
        pixel_eof = 0;
        run_request = 0;
        timeout = 0;
        while ((dut.processing.codec_frame_pending || dut.processing.unused_threshold_busy
                || dut.processing.hierarchy_busy || dut.processing.centroid_group_valid)
                && timeout < 200000) begin tick(); timeout++; end
        if (timeout == 200000) $fatal(1, "actual frontend did not drain");
        expect_owned();
        if (!lftag_busy || lftag_in_frame_seq != 0 || frames_started != 1)
            $fatal(1, "external busy or native frame sequence mismatch");
        repeat (12) begin tick(); expect_owned(); end
        external_lftag_busy = 0;
        external_lftag_fifo_level = 6'd32;
        repeat (12) begin tick(); expect_owned(); end
        if (lftag_fifo_level != 32) $fatal(1, "external queue level forwarding mismatch");
        external_lftag_fifo_level = 0;
        // The shared owner remains busy through its final accepted word;
        // result-valid may cancel combinationally on processing reset.
        external_lftag_busy = 1;
        external_lftag_result_valid = 1;
        external_lftag_result_data = 32'h4c465447;
        repeat (4) begin
            tick(); expect_owned();
            if (!lftag_result_valid || lftag_result_data != 32'h4c465447)
                $fatal(1, "stalled first output word changed");
        end
        lftag_result_ready = 1;
        tick();
        lftag_result_ready = 0;
        external_lftag_result_data = 32'h41abcdef;
        repeat (4) begin
            tick(); expect_owned();
            if (lftag_result_data != 32'h4a5bcdef)
                $fatal(1, "stalled output frame byte was not the paired frame ID: %h", lftag_result_data);
        end
        lftag_result_ready = 1;
        tick();
        external_lftag_result_data = 32'h01234567;
        external_lftag_result_last = 1;
        #1;
        if (lftag_result_data != 32'h01234567 || !lftag_result_last)
            $fatal(1, "final external output was changed");
        tick();
        external_lftag_result_valid = 0;
        external_lftag_result_last = 0;
        external_lftag_busy = 0;
        timeout = 0;
        while (!idle && timeout < 100) begin tick(); timeout++; end
        if (!idle || frames_completed != 1 || retired_pulses != 1 || reset_pulses != 1)
            $fatal(1, "frame did not retire/reset exactly once after external output drained");

        for (integer cause = 0; cause < 2; cause++) begin
            external_lftag_fifo_drop_sticky = cause == 0;
            external_lftag_protocol_error_sticky = cause == 1;
            tick();
            external_lftag_fifo_drop_sticky = 0;
            external_lftag_protocol_error_sticky = 0;
            tick();
            // A lost queue entry remains fatal. A malformed candidate is
            // rejected by the external solver and only retains a diagnostic.
            if (fault_sticky != (cause == 0) || lftag_processing_reset != (cause == 0)
                    || dut.local_fault != (cause == 0)
                    || lftag_fifo_drop_sticky != (cause == 0)
                    || lftag_protocol_error_sticky != (cause == 1))
                $fatal(1, "external solver diagnostic severity or retention changed");
            clear_diagnostics = 1;
            tick();
            clear_diagnostics = 0;
            tick();
            if (fault_sticky || lftag_fifo_drop_sticky || lftag_protocol_error_sticky)
                $fatal(1, "disarmed clear did not clear retained external diagnostics");
        end
        $display("PASS external LFTag frontend: native tuples, status, ownership, output stalls, reset and faults");
        $finish;
    end
endmodule

`default_nettype wire
