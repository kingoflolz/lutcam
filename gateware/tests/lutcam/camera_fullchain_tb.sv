`timescale 1ns/1ps
`default_nettype none

// Full production chain: no processing or ISP test doubles. The clock period
// is irrelevant to synchronous behavior; one frame occupies exactly 1,000,000
// clocks, matching 120 frames/s at the hardware's 120 MHz processing clock.
module camera_fullchain_tb;
    localparam integer FRAME_CLOCKS = 1000000;
    localparam integer LINE_CLOCKS = 896;
    logic clk = 0;
    always #5 clk = !clk;
    logic rst = 1;
    logic [19:0] pixel_data = 0;
    logic pixel_valid = 0, pixel_sof = 0, pixel_eol = 0, pixel_eof = 0;
    logic run_request = 0;
    wire active, idle, fault_sticky;
    wire [31:0] frame_id, frames_started, frames_completed;
    wire [15:0] ingress_high_water;
    logic wb_cyc_i = 0, wb_stb_i = 0, wb_we_i = 0;
    logic [15:0] wb_adr_i = 0;
    logic [31:0] wb_dat_i = 0;
    wire [31:0] wb_dat_o;
    wire wb_ack_o, wb_err_o;
    wire lwc_out_valid, lwc_out_sof, lwc_out_padding, lwc_protocol_error;
    wire [15:0] lwc_out_data;
    wire lftag_fifo_drop_sticky, lftag_protocol_error_sticky;
    wire inner_fv_valid, hier_inner_end;
    wire lftag_result_valid, lftag_result_last;
    wire [31:0] lftag_result_data;
    wire lftag_candidate_start, lftag_candidate_done;

    CameraPipeline dut (
        .clk, .rst, .pixel_data, .pixel_valid, .pixel_sof, .pixel_eol, .pixel_eof,
        .receiver_fault(1'b0), .run_request, .capture_permit(1'b1),
        .pair_abort(1'b0),
        .capture_frame_id(32'd0),
        .clear_diagnostics(1'b0), .active, .idle, .fault_sticky,
        .frame_id, .frames_started, .frames_completed, .ingress_high_water,
        .wb_cyc_i, .wb_stb_i, .wb_we_i, .wb_sel_i(4'hf),
        .wb_adr_i, .wb_dat_i, .wb_dat_o, .wb_ack_o, .wb_err_o,
        .hist_dump_req(1'b0), .hist_dump_ready(), .hist_out_valid(),
        .hist_out_ready(1'b1), .hist_out_data(),
        .inner_fv_valid, .inner_fv_data(), .hier_inner_end,
        .lwc_out_sof, .lwc_out_valid, .lwc_out_data, .lwc_out_padding,
        .lwc_protocol_error,
        .lftag_cfg_inv_fx_q2_30(32'h00222222),
        .lftag_cfg_inv_fy_q2_30(32'h00222222),
        .lftag_cfg_cx_q16_16(32'h01400000),
        .lftag_cfg_cy_q16_16(32'h00f00000),
        .lftag_cfg_printed_width_q16_16(32'd65536),
        .lftag_cfg_mean_residual_limit_q16(16'd1024),
        .lftag_cfg_max_mass_spread(8'd19), .lftag_cfg_min_baseline_q4(16'd16),
        .lftag_result_ready(1'b1),
        .lftag_clear_diagnostics(1'b0), .lftag_result_valid,
        .lftag_result_data, .lftag_result_last, .lftag_busy(),
        .lftag_fifo_level(), .lftag_candidate_start, .lftag_candidate_done,
        .lftag_candidate_cycles(), .lftag_fifo_drop_sticky,
        .lftag_protocol_error_sticky
    );

    integer current_frame = -1, frame_clock = 0;
    integer fixture_first = 0, all_fixture = 0, source_frame = 0;
    integer input_pairs = 0, processed_pairs = 0;
    integer output_words = 0, output_sofs = 0, padding_words = 0;
    integer completion_clock = -1;
    integer child_tuples = 0, child_groups = 0;
    integer candidate_starts = 0, candidate_completions = 0;
    integer solver_packets = 0, solver_packet_words = 0;
    integer expected_groups;

    function automatic [9:0] sample(input integer frame_number,
                                    input integer row, input integer column);
        integer value, horizon, dx, dy, shift;
        logic [31:0] texture;
        case (frame_number)
            0: sample = 10'd512;
            1: sample = 10'(256 + (column >> 2));
            default: begin
                // Exact fixture from tests/lftag/tests/rtl_tagdetect_lwc.rs
                // load_frame(), expanded from 8-bit to RAW10 for ISP bypass.
                horizon = 800 * 3 / 7;
                if (row < horizon) value = 42 + row * 92 / horizon;
                else value = 76 + (row - horizon) * 88 / (800 - horizon);
                dx = column - 1280 * 3 / 4;
                dy = row - 800 / 5;
                if (dx * dx + dy * dy < 52 * 52) value = value + 78;
                if (row > horizon && column > 1280 / 7 && column < 1280 / 7 + 180)
                    value = value - 35;
                if (row > horizon + 100 && ((column / 96) & 1) == 0)
                    value = value + 12;
                texture = 32'(column) * 32'h9e3779b1;
                shift = row & 15;
                if (shift != 0)
                    texture = (texture << shift) | (texture >> (32 - shift));
                texture = texture ^ (32'(row) * 32'h85ebca6b);
                value = value + integer'(texture >> 29) - 3;
                if (value < 0) value = 0;
                if (value > 255) value = 255;
                sample = 10'(value << 2);
            end
        endcase
    endfunction

    always @(posedge clk) begin
        if (!rst && current_frame >= 0) begin
            if (pixel_valid)
                input_pairs = input_pairs + 1;
            if (dut.core_valid) begin
                if (!active || frame_id != 32'(current_frame))
                    $fatal(1, "processed pixels have the wrong frame owner");
                processed_pairs = processed_pairs + 1;
            end
            if (lwc_out_sof && !lwc_out_valid)
                $fatal(1, "compressor SOF asserted without a valid word");
            if (lwc_out_padding && !lwc_out_valid)
                $fatal(1, "compressor padding marker asserted without a valid word");
            if (lwc_out_valid) begin
                if (!active || frame_id != 32'(current_frame))
                    $fatal(1, "compressed output lost frame ownership at frame %0d clock %0d",
                           current_frame, frame_clock);
                if (lwc_out_sof) begin
                    if (output_words != 0)
                        $fatal(1, "compressor SOF appeared after the first word");
                    output_sofs = output_sofs + 1;
                end
                output_words = output_words + 1;
                if (lwc_out_padding) begin
                    if (lwc_out_data != 0)
                        $fatal(1, "compressor marked a nonzero word as padding");
                    padding_words = padding_words + 1;
                end else if (padding_words != 0)
                    $fatal(1, "compressor emitted non-padding data after its padding tail");
                if (output_words > 64000)
                    $fatal(1, "compressor emitted more than 64000 words");
            end
            if (inner_fv_valid || hier_inner_end || lftag_result_valid
                    || lftag_candidate_start || lftag_candidate_done) begin
                if (!active || frame_id != 32'(current_frame))
                    $fatal(1, "child/solver output lost frame ownership");
            end
            if (hier_inner_end && !inner_fv_valid)
                $fatal(1, "child group terminator outside a valid tuple");
            if (inner_fv_valid) child_tuples = child_tuples + 1;
            if (hier_inner_end) child_groups = child_groups + 1;
            if (lftag_candidate_start) candidate_starts = candidate_starts + 1;
            if (lftag_candidate_done) candidate_completions = candidate_completions + 1;
            if (lftag_result_valid) begin
                if (solver_packet_words == 1
                        && lftag_result_data[27:20] != 8'(current_frame))
                    $fatal(1, "native solver header has frame byte %0d; expected %0d",
                           lftag_result_data[27:20], current_frame);
                solver_packet_words = solver_packet_words + 1;
                if (lftag_result_last) begin
                    if (solver_packet_words != 8 && solver_packet_words != 22)
                        $fatal(1, "internal solver emitted a malformed packet length %0d", solver_packet_words);
                    solver_packets = solver_packets + 1;
                    solver_packet_words = 0;
                end
            end
        end
    end

    task automatic step(input logic send_pixel, input integer row,
                        input integer pair_column);
        @(negedge clk);
        pixel_valid = send_pixel;
        pixel_sof = send_pixel && row == 0 && pair_column == 0;
        pixel_eol = send_pixel && pair_column == 639;
        pixel_eof = send_pixel && row == 799 && pair_column == 639;
        pixel_data = {
            sample(source_frame, row, pair_column * 2 + 1),
            sample(source_frame, row, pair_column * 2)
        };
        @(posedge clk);
        #1;
        if (!rst && fault_sticky)
            $fatal(1, "camera fault frame=%0d clock=%0d words=%0d core_busy=%b ISP_ready=%b ISP=%b LWC=%b LFdrop=%b LFprotocol=%b",
                   current_frame, frame_clock, output_words, dut.core_busy,
                   dut.isp_in_ready, dut.isp_protocol_error, lwc_protocol_error,
                   lftag_fifo_drop_sticky, lftag_protocol_error_sticky);
        if (!rst && current_frame >= 0 && frame_id != 32'(current_frame))
            $fatal(1, "frame ID changed while frame %0d owns the pipeline", current_frame);
        if (!rst && ingress_high_water != 0)
            $fatal(1, "unbuffered pipeline reported a FIFO high-water mark");
        if (!rst && idle
                && !dut.processing.wavelet_tag_detect.hierarchy_core.child_arena_new_ready)
            $fatal(1, "camera advertised idle before child-arena allocation became ready");
    endtask

    integer row, column, wait_cycles;
    initial begin
        if (!$value$plusargs("FIXTURE_FIRST=%d", fixture_first)) fixture_first = 0;
        if (!$value$plusargs("ALL_FIXTURE=%d", all_fixture)) all_fixture = 0;
        repeat (5) step(0, 0, 0);
        rst = 0;
        repeat (5) step(0, 0, 0);
        wb_cyc_i = 1;
        wb_stb_i = 1;
        wb_we_i = 1;
        wb_adr_i = 16'h0004;
        wb_dat_i = 32'd1;
        step(0, 0, 0);
        if (!wb_ack_o || wb_err_o)
            $fatal(1, "ISP bypass initialization failed");
        wb_cyc_i = 0;
        wb_stb_i = 0;
        wb_we_i = 0;
        wait_cycles = 0;
        while (!idle || !dut.isp_in_ready) begin
            step(0, 0, 0);
            wait_cycles = wait_cycles + 1;
            if (wait_cycles > 10000)
                $fatal(1, "production pipeline did not become ready after reset");
        end
        run_request = 1;
        for (current_frame = 0; current_frame < 3; current_frame = current_frame + 1) begin
            source_frame = fixture_first ? (current_frame == 0 ? 2 : current_frame - 1) : current_frame;
            if (all_fixture) source_frame = 2;
            // Independently derived from the canonical scalar frontend and
            // hierarchy reference (rtl_tagdetect_lwc.rs texture fixture).
            expected_groups = source_frame == 2 ? 4 : 0;
            input_pairs = 0;
            processed_pairs = 0;
            output_words = 0;
            output_sofs = 0;
            padding_words = 0;
            completion_clock = -1;
            child_tuples = 0;
            child_groups = 0;
            candidate_starts = 0;
            candidate_completions = 0;
            solver_packets = 0;
            solver_packet_words = 0;
            if (!idle || frames_completed != 32'(current_frame))
                $fatal(1, "previous frame failed to drain before next SOF");
            for (frame_clock = 0; frame_clock < FRAME_CLOCKS; frame_clock = frame_clock + 1) begin
                row = frame_clock / LINE_CLOCKS;
                column = frame_clock % LINE_CLOCKS;
                step(row < 800 && column < 640, row, column);
                if (idle && frames_completed == 32'(current_frame + 1)
                        && completion_clock < 0)
                    completion_clock = frame_clock;
                if (frame_clock == 900000) begin
                    if (!idle || active || completion_clock < 0)
                        $fatal(1, "frame %0d did not drain with 100000 clocks before next frame; busy=%b words=%0d",
                               current_frame, dut.core_busy, output_words);
                end
            end
            if (input_pairs != 512000 || processed_pairs != 512000
                    || output_words != 64000 || output_sofs != 1
                    || frames_started != 32'(current_frame + 1)
                    || frames_completed != 32'(current_frame + 1) || !idle)
                $fatal(1, "frame %0d counts: input=%0d processed=%0d words=%0d SOFs=%0d started=%0d completed=%0d",
                       current_frame, input_pairs, processed_pairs, output_words,
                       output_sofs, frames_started, frames_completed);
            $display("frame %0d: 512000 pairs, 64000 words (%0d padding), drained by clock %0d, ingress high-water %0d",
                     current_frame, padding_words, completion_clock, ingress_high_water);
            if (child_tuples != 9 * expected_groups || child_groups != expected_groups
                    || candidate_starts != expected_groups || candidate_completions != expected_groups
                    || solver_packets != expected_groups || solver_packet_words != 0)
                $fatal(1, "fixture hierarchy/solver counts: tuples=%0d groups=%0d starts=%0d done=%0d packets=%0d partial=%0d",
                       child_tuples, child_groups, candidate_starts,
                       candidate_completions, solver_packets, solver_packet_words);
            if (candidate_starts != child_groups || candidate_completions != child_groups
                    || solver_packets != child_groups || solver_packet_words != 0)
                $fatal(1, "frame ended with unretired child/solver work");
            $display("frame %0d: %0d child tuples/%0d groups, %0d solver candidates and packets retired",
                     current_frame, child_tuples, child_groups, solver_packets);
        end
        $display("PASS camera fullchain: three 120 fps frames");
        $finish;
    end
    initial begin
        #40000000;
        $fatal(1, "simulation timeout");
    end
endmodule

`default_nettype wire
