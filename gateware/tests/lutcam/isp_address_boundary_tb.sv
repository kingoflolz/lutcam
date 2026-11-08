`timescale 1ns/1ps
`default_nettype none
// This regression uses only public MonoIsp ports. It can run against one
// implementation or the private exact-cycle miter with the same module name.
module isp_address_boundary_case #(
    parameter integer FRAME_PAIRS = 4
) (output logic done = 0);
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
    logic [9:0] in_pixel0 = 10'd136, in_pixel1 = 10'd244;
    wire in_ready, out_valid, out_sof, out_eol;
    logic out_ready = 1;
    wire [7:0] out_pixel0, out_pixel1;
    logic hist_dump_req = 0, hist_out_ready = 0;
    wire hist_dump_ready, hist_out_valid, protocol_error;
    wire [31:0] hist_out_data;
    wire unused_position_sof, unused_position_eol, unused_position_eof;
  MonoIsp #(.WIDTH(2), .HEIGHT(FRAME_PAIRS)) dut (.*, .input_position_sof(unused_position_sof),
        .input_position_eol(unused_position_eol),
        .input_position_eof(unused_position_eof));
    logic [19:0] words [0:511];
    integer checks = 0;

    task automatic idle(input integer clocks);
        repeat (clocks) @(negedge clk);
    endtask

    task automatic bus(input bit write_enable, input logic [15:0] address,
                       input logic [31:0] value);
        bit complete;
        @(negedge clk);
        wb_cyc_i = 1; wb_stb_i = 1; wb_we_i = write_enable;
        wb_adr_i = address; wb_dat_i = value; complete = 0;
        for (integer attempt = 0; attempt < 8 && !complete; attempt++) begin
            @(posedge clk); #1;
            if (wb_err_o) $fatal(1, "WB error frame=%0d address=%h", FRAME_PAIRS, address);
            if (wb_ack_o) complete = 1;
        end
        if (!complete) $fatal(1, "WB timeout frame=%0d", FRAME_PAIRS);
        @(negedge clk); wb_cyc_i = 0; wb_stb_i = 0; wb_we_i = 0;
    endtask

    task automatic program_table(input integer count);
        for (integer index = 0; index < count; index++)
            bus(1, 16'(16'h2000 + index * 4), {12'd0, words[index]});
        bus(1, 16'h000c, 32'(count));
        for (integer index = 0; index < count; index++) begin
            bus(0, 16'(16'h2000 + index * 4), 0);
            if (wb_dat_o !== {12'd0, words[index]})
                $fatal(1, "Defect RAM readback frame=%0d index=%0d", FRAME_PAIRS, index);
        end
    endtask

    task automatic start_expect(input integer count, input bit expected_error);
        bus(1, 16'h0004, 1);
        idle(count + 12);
        if (protocol_error !== expected_error || in_ready !== !expected_error)
            $fatal(1, "Decode boundary frame=%0d error=%b ready=%b expected_error=%b", FRAME_PAIRS, protocol_error, in_ready, expected_error);
        bus(0, 16'h0008, 0);
        if (wb_dat_o[6] !== expected_error || wb_dat_o[0] !== 1'b1)
            $fatal(1, "Status boundary frame=%0d status=%h", FRAME_PAIRS, wb_dat_o);
        checks++;
    endtask

    task automatic stop_clear;
        bus(1, 16'h0004, 0); idle(4);
        bus(1, 16'h0010, 2); idle(3);
        if (protocol_error || in_ready)
            $fatal(1, "Stop/reconfigure boundary frame=%0d", FRAME_PAIRS);
    endtask

    // Encode an absolute target as nonnegative 18-bit continuation deltas.
    // All earlier words have zero mask, so no pixel stream is needed to drain
    // the decoder. The external table capacity remains exactly 512 words.
    task automatic target_case(input integer target, input bit expected_error);
        integer remaining, count, delta;
        remaining = target; count = 0;
        while (remaining > 262143 && count < 512) begin
            words[count] = {18'h3ffff, 2'b00};
            remaining -= 262143; count++;
        end
        if (count < 512) begin
            delta = remaining;
            words[count] = {18'(delta), 2'b01}; count++;
            program_table(count); start_expect(count, expected_error); stop_clear();
        end
    endtask

    initial begin
        idle(3); rst = 0; idle(3);
        program_table(0); start_expect(0, 0); stop_clear();
        target_case(FRAME_PAIRS - 1, 0);
        target_case(FRAME_PAIRS, 1);

        // The maximum single delta must not truncate at the frame width;
        // an out-of-range zero-mask continuation is still a protocol error.
        words[0] = {18'h3ffff, 2'b00};
        program_table(1); start_expect(1, FRAME_PAIRS <= 262143); stop_clear();
        words[0] = {18'h3ffff, 2'b00};
        words[1] = {18'h3ffff, 2'b01};
        program_table(2); start_expect(2, FRAME_PAIRS <= 524286); stop_clear();

        // Zero-mask words do not establish a previous nonzero descriptor.
        words[0] = 0; words[1] = 0; words[2] = 20'd3;
        program_table(3); start_expect(3, 0);
        @(negedge clk); in_valid = 1; in_sof = 1; in_eol = 1; out_ready = 0;
        @(posedge clk); #1;
        if (!out_valid || out_pixel0 != 0 || out_pixel1 != 0 || !out_sof || !out_eol)
            $fatal(1, "Zero continuation correction frame=%0d", FRAME_PAIRS);
        @(negedge clk); in_valid = 0; idle(3);
        if (!out_valid) $fatal(1, "Stalled output lost frame=%0d", FRAME_PAIRS);
        // Dirty global reset while a corrected output is stalled.
        rst = 1; idle(2); rst = 0; out_ready = 1; idle(3);
        if (out_valid || protocol_error || in_ready)
            $fatal(1, "Dirty reset frame=%0d", FRAME_PAIRS);

        // Duplicate real descriptors are rejected even across zero deltas.
        words[0] = 20'd1; words[1] = 0; words[2] = 20'd2;
        program_table(3); start_expect(3, 1); stop_clear();
        // Advancing through a zero-mask continuation must permit a real
        // descriptor with delta zero at the new address. The two pending
        // descriptors fit without consuming pixels or reducing table capacity.
        words[0] = 20'd1; words[1] = {18'd1, 2'b00}; words[2] = 20'd2;
        program_table(3); start_expect(3, FRAME_PAIRS <= 1); stop_clear();
        // After a real descriptor, arbitrary zero continuations must retain
        // the no-advance state and reject another real descriptor there.
        words[0] = {18'd1, 2'b00}; words[1] = 20'd1;
        words[2] = 0; words[3] = 0; words[4] = 20'd2;
        program_table(5); start_expect(5, 1); stop_clear();
        // Maximum reachable cumulative address from the 512-word table.
        // This stays legal in the largest supported frame and equals the
        // first invalid address in the nearby 134217216-pair frame.
        for (integer index = 0; index < 512; index++)
            words[index] = {18'h3ffff, 2'b00};
        program_table(512); start_expect(512, FRAME_PAIRS <= 134217216); stop_clear();
        program_table(0); start_expect(0, 0); stop_clear();
        $display("PASS ISP address boundaries frame_pairs=%0d checks=%0d", FRAME_PAIRS, checks);
        done = 1;
    end
endmodule

module isp_address_boundary_tb;
    wire [7:0] done;
    isp_address_boundary_case #(.FRAME_PAIRS(1)) minimum(done[0]);
    isp_address_boundary_case #(.FRAME_PAIRS(4)) tiny_power2(done[1]);
    isp_address_boundary_case #(.FRAME_PAIRS(21)) non_power2(done[2]);
    isp_address_boundary_case #(.FRAME_PAIRS(262144)) delta_power2(done[3]);
    isp_address_boundary_case #(.FRAME_PAIRS(512000)) production_count(done[4]);
    isp_address_boundary_case #(.FRAME_PAIRS(67108864)) large_power2(done[5]);
    isp_address_boundary_case #(.FRAME_PAIRS(134217216)) maximum_table_sum(done[6]);
    isp_address_boundary_case #(.FRAME_PAIRS(134217727)) largest_legal(done[7]);
    initial begin
        wait (&done);
        $display("PASS all ISP public address boundary cases");
        $finish;
    end
    initial begin #1000000; $fatal(1, "ISP boundary test timeout"); end
endmodule
`default_nettype wire
