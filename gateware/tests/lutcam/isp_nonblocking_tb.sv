`timescale 1ns/1ps
`default_nettype none

// Canonically packed defect tables must never stall an active sensor raster.
// These tests drive the real 1280x800 ISP at one pair on every active clock,
// with only the short decoder restart gap between complete frames.
module isp_nonblocking_tb;
    localparam integer WIDTH = 1280;
    localparam integer HEIGHT = 800;
    localparam integer ROW_PAIRS = WIDTH / 2;
    localparam integer FRAME_PAIRS = ROW_PAIRS * HEIGHT;
    localparam integer DELTA_MAX = 262143;

    logic clk = 0;
    always #5 clk = !clk;
    logic rst = 1;
    logic wb_cyc_i = 0, wb_stb_i = 0, wb_we_i = 0;
    logic [3:0] wb_sel_i = 4'hf;
    logic [15:0] wb_adr_i = 0;
    logic [31:0] wb_dat_i = 0;
    wire [31:0] wb_dat_o;
    wire wb_ack_o, wb_err_o;
    logic in_valid = 0, in_sof = 0, in_eol = 0;
    logic [9:0] in_pixel0 = 0, in_pixel1 = 0;
    wire in_ready;
    wire out_valid, out_sof, out_eol;
    wire [7:0] out_pixel0, out_pixel1;
    wire protocol_error;

    wire unused_position_sof, unused_position_eol, unused_position_eof;
  MonoIsp isp (
        .clk, .rst,
        .wb_cyc_i, .wb_stb_i, .wb_we_i, .wb_sel_i, .wb_adr_i,
        .wb_dat_i, .wb_dat_o, .wb_ack_o, .wb_err_o,
        .in_valid, .in_ready, .in_pixel0, .in_pixel1, .in_sof, .in_eol,
        .out_valid, .out_ready(1'b1), .out_pixel0, .out_pixel1,
        .out_sof, .out_eol,
        .hist_dump_req(1'b0), .hist_dump_ready(), .hist_out_valid(),
        .hist_out_ready(1'b0), .hist_out_data(), .protocol_error,
        .input_position_sof(unused_position_sof),
        .input_position_eol(unused_position_eol),
        .input_position_eof(unused_position_eof)
    );

    logic [1:0] masks [0:FRAME_PAIRS-1];
    integer encoded_words = 0;
    integer previous_address = 0;
    integer case_number;
    logic use_lut;
    logic expected_valid = 0;
    logic [17:0] expected_data = 0;
    logic [9:0] previous_corrected = 0;
    integer accepted_pairs = 0;
    integer output_pairs = 0;
    integer stalled_cycles = 0;

    function automatic [9:0] raw0(input integer address);
        raw0 = 10'(address * 17 + 9);
    endfunction
    function automatic [9:0] raw1(input integer address);
        raw1 = 10'(address * 29 + 307);
    endfunction
    function automatic [7:0] tone(input logic [9:0] value);
        tone = 8'((value >> 2) ^ ((value & 10'd3) * 10'd85));
    endfunction

    task automatic wb_write(input logic [15:0] address, input logic [31:0] value);
        logic done;
        integer attempts;
        @(negedge clk);
        wb_cyc_i = 1;
        wb_stb_i = 1;
        wb_we_i = 1;
        wb_adr_i = address;
        wb_dat_i = value;
        done = 0;
        for (attempts = 0; attempts < 8 && !done; attempts = attempts + 1) begin
            @(posedge clk);
            #1;
            if (wb_err_o) $fatal(1, "Wishbone rejected %h = %h", address, value);
            done = wb_ack_o;
        end
        if (!done) $fatal(1, "Wishbone timeout at %h", address);
        @(negedge clk);
        wb_cyc_i = 0;
        wb_stb_i = 0;
        wb_we_i = 0;
    endtask

    task automatic write_word(input integer delta, input logic [1:0] mask);
        if (encoded_words >= 512) $fatal(1, "test defect table exceeds capacity");
        wb_write(16'(16'h2000 + encoded_words * 4), (32'(delta) << 2) | 32'(mask));
        encoded_words = encoded_words + 1;
    endtask

    // Same minimal continuation rule as mono_isp::pack_defect_table.
    task automatic add_defect(input integer address);
        integer delta;
        logic [1:0] mask;
        delta = address - previous_address;
        while (delta > DELTA_MAX) begin
            write_word(DELTA_MAX, 2'b00);
            delta = delta - DELTA_MAX;
        end
        mask = address % ROW_PAIRS == 0 ? 2'b11 : 2'((address % 3) + 1);
        write_word(delta, mask);
        masks[address] = mask;
        previous_address = address;
    endtask

    task automatic step(
        input logic valid, input integer address, input logic require_ready,
        output logic accepted
    );
        logic [9:0] corrected0, corrected1;
        logic [7:0] mapped0, mapped1;
        @(negedge clk);
        in_valid = valid;
        in_pixel0 = raw0(address);
        in_pixel1 = raw1(address);
        in_sof = valid && address == 0;
        in_eol = valid && address % ROW_PAIRS == ROW_PAIRS - 1;
        @(posedge clk);
        if (out_valid !== expected_valid)
            $fatal(1, "case %0d output cadence changed at input pair %0d", case_number, address);
        if (out_valid) begin
            if ({out_sof, out_eol, out_pixel1, out_pixel0} !== expected_data)
                $fatal(1, "case %0d output %0d: got %h expected %h", case_number,
                       output_pairs, {out_sof, out_eol, out_pixel1, out_pixel0}, expected_data);
            output_pairs = output_pairs + 1;
        end
        accepted = valid && in_ready;
        if (valid && !in_ready) begin
            stalled_cycles = stalled_cycles + 1;
            if (require_ready)
                $fatal(1, "canonical table stalled active pair %0d in case %0d", address, case_number);
        end
        expected_valid = accepted;
        if (accepted) begin
            corrected0 = raw0(address);
            corrected1 = raw1(address);
            case (masks[address])
                2'b01: corrected0 = corrected1;
                2'b10: corrected1 = corrected0;
                2'b11: begin
                    corrected0 = address % ROW_PAIRS == 0 ? 10'd0 : previous_corrected;
                    corrected1 = corrected0;
                end
                default: begin end
            endcase
            previous_corrected = corrected1;
            mapped0 = use_lut ? tone(corrected0) : corrected0[9:2];
            mapped1 = use_lut ? tone(corrected1) : corrected1[9:2];
            expected_data = {in_sof, in_eol, mapped1, mapped0};
            accepted_pairs = accepted_pairs + 1;
        end
        #1;
        if (protocol_error) $fatal(1, "MonoIsp protocol error at pair %0d", address);
    endtask

    integer index, frame, pair_index, initial_wait;
    logic accepted;
    initial begin
        if (!$value$plusargs("CASE=%d", case_number)) case_number = 0;
        if (case_number < 0 || case_number > 5) $fatal(1, "unknown case");
        use_lut = case_number == 1 || case_number == 2;
        for (index = 0; index < FRAME_PAIRS; index = index + 1) masks[index] = 0;
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst = 0;
        if (use_lut)
            for (index = 0; index < 1024; index = index + 1)
                wb_write(16'(16'h1000 + index * 4), {24'd0, tone(10'(index))});
        case (case_number)
            0: for (index = 0; index < 512; index = index + 1) add_defect(index);
            1: begin
                add_defect(0); add_defect(1); add_defect(262145); add_defect(262146);
            end
            2: begin
                for (index = 0; index < 255; index = index + 1) add_defect(index);
                for (index = 400000; index < 400256; index = index + 1) add_defect(index);
            end
            3: add_defect(FRAME_PAIRS - 1);
            4: begin end
            5: begin
                // At address one, the continuation alone does not prove that
                // the pair is clear: the next word may have zero delta.
                write_word(0, 2'b01); masks[0] = 2'b01;
                write_word(1, 2'b00);
                write_word(0, 2'b10); masks[1] = 2'b10;
            end
        endcase
        wb_write(16'h000c, 32'(encoded_words));
        wb_write(16'h0004, use_lut ? 32'd3 : 32'd1);
        initial_wait = 0;
        while (!in_ready) begin
            step(0, 0, 0, accepted);
            initial_wait = initial_wait + 1;
            if (initial_wait > 8) $fatal(1, "canonical prefetch took more than eight clocks");
        end
        for (frame = 0; frame < 2; frame = frame + 1) begin
            pair_index = 0;
            while (pair_index < FRAME_PAIRS) begin
                step(1, pair_index, case_number != 5, accepted);
                if (accepted) pair_index = pair_index + 1;
            end
            // Exercise decoder reload at the frame boundary without touching
            // software configuration or introducing an active-raster bubble.
            repeat (8) step(0, 0, 0, accepted);
            if (!in_ready) $fatal(1, "decoder not prefetched after frame blanking");
        end
        if (accepted_pairs != 2 * FRAME_PAIRS || output_pairs != 2 * FRAME_PAIRS)
            $fatal(1, "unexpected accepted/output count %0d/%0d", accepted_pairs, output_pairs);
        if (case_number == 5 && stalled_cycles == 0)
            $fatal(1, "noncanonical frontier test did not exercise the required safe stall");
        if (case_number != 5 && stalled_cycles != 0)
            $fatal(1, "canonical table stalled %0d cycles", stalled_cycles);
        $display("PASS MonoIsp nonblocking case %0d: %0d pairs, %0d active stalls",
                 case_number, output_pairs, stalled_cycles);
        $finish;
    end
endmodule
`default_nettype wire
