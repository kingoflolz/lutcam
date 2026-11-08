`timescale 1ns/1ps
`default_nettype none

// Exercise control registers and RAM address selection extracted verbatim
// from current wavelet RTL. An independent monotonic request counter is the
// oracle; it does not reconstruct a cursor from the DUT's retained column.
// Complete transform coefficient tests cover arithmetic and integration.
module wavelet_request_cursor_tb;
    localparam integer PAIRS = 320;
    logic clk = 0;
    always #5 clk = !clk;
    logic rst_n = 0;
    logic inject_clear = 0;
    logic inject_request = 0;
    logic inject_consume = 0;

    wavelet_request_cursor_dut dut (
        .clk, .rst_n, .frame_clear(inject_clear), .inject_request, .inject_consume
    );

    integer next_index = 0;
    integer response_column = 0;
    logic requests_done = 0;
    logic response_valid = 0;
    integer cycles = 0;
    integer requests = 0;
    integer clear_request = 0;
    integer clear_consume = 0;
    integer terminal_request = 0;
    integer terminal_consume = 0;
    integer stalled_response = 0;
    integer reset_count = 0;
    logic [31:0] random_state = 32'hbd26e143;

    task automatic check_state;
        if (dut.v2_high_read_column_q !== 9'(response_column))
            $fatal(1, "response column cycle %0d: actual %0d expected %0d",
                   cycles, dut.v2_high_read_column_q, response_column);
        if (dut.v2_high_requests_done !== requests_done)
            $fatal(1, "requests_done cycle %0d", cycles);
        if (dut.v2_high_read_valid_q !== response_valid)
            $fatal(1, "response_valid cycle %0d", cycles);
    endtask

    task automatic reset_cursor;
        @(negedge clk);
        rst_n = 0;
        inject_clear = 0;
        inject_request = 0;
        inject_consume = 0;
        next_index = 0;
        response_column = 0;
        requests_done = 0;
        response_valid = 0;
        #1;
        check_state();
        @(negedge clk);
        rst_n = 1;
        reset_count = reset_count + 1;
    endtask

    task automatic step(input logic clear_row, input logic want_request,
                        input logic consume);
        integer old_index, old_column;
        @(negedge clk);
        inject_clear = clear_row;
        // This is the sole scheduler assumption: no request after done.
        inject_request = want_request && !requests_done;
        inject_consume = consume;
        #1;
        check_state();
        old_index = next_index;
        old_column = response_column;
        if (inject_request) begin
            if (dut.v2_high_request_index !== 9'(old_index))
                $fatal(1, "request index cycle %0d: actual %0d expected %0d",
                       cycles, dut.v2_high_request_index, old_index);
            if (!dut.local_row_read_enable
                    || dut.local_row_read_address !== 9'(old_index / 2))
                $fatal(1, "RAM request address cycle %0d: actual %0d expected %0d",
                       cycles, dut.local_row_read_address, old_index / 2);
            requests = requests + 1;
            if (clear_row) clear_request = clear_request + 1;
        end
        if (clear_row && consume) clear_consume = clear_consume + 1;
        if (response_valid && !inject_request && !consume)
            stalled_response = stalled_response + 1;

        // An ordinary next-address counter models the original interface.
        // Apply sequential source order, retaining the OLD column for consume.
        if (clear_row) begin
            next_index = 0;
            requests_done = 0;
            response_valid = 0;
        end
        if (inject_request) begin
            response_column = old_index;
            response_valid = 1;
            if (old_index == PAIRS - 1) begin
                requests_done = 1;
                terminal_request = terminal_request + 1;
            end else begin
                next_index = old_index + 1;
            end
        end else if (consume) begin
            response_valid = 0;
        end
        if (consume && old_column == PAIRS - 1) begin
            next_index = 0;
            requests_done = 0;
            terminal_consume = terminal_consume + 1;
        end
        @(posedge clk);
        #1;
        cycles = cycles + 1;
        check_state();
    endtask

    task automatic advance_to(input integer target);
        while (next_index != target || requests_done)
            step(0, 1, response_valid);
    endtask

    initial begin
        reset_cursor();

        // Complete rows, held terminal responses, and restart at address zero.
        for (integer row = 0; row < 3; row = row + 1) begin
            for (integer column = 0; column < PAIRS; column = column + 1) begin
                if ((column % 7) == 3) step(0, 0, 0);
                step(0, 1, response_valid);
            end
            repeat (4) step(0, 1, 0);
            step(0, 0, 1);
        end

        // Each combination at the beginning, middle, and end of a row.
        for (integer location = 0; location < 6; location = location + 1) begin
            for (integer controls = 0; controls < 8; controls = controls + 1) begin
                reset_cursor();
                case (location)
                    0: advance_to(0);
                    1: advance_to(1);
                    2: advance_to(159);
                    3: advance_to(160);
                    4: advance_to(318);
                    default: advance_to(319);
                endcase
                step(controls[2], controls[1], controls[0]);
                repeat (4) step(0, 0, 0);
                repeat (4) step(0, 1, response_valid);
            end
        end

        // Clear with a held final response, then request+consume at the same
        // edge. Terminal consumption must use the OLD response column and
        // override a request's advance, even for this malformed schedule.
        reset_cursor();
        advance_to(319);
        step(0, 1, 0);
        step(1, 0, 0);
        step(0, 1, 1);
        step(0, 1, 0);

        // Deterministic unrestricted control combinations and async resets.
        for (integer trial = 0; trial < 12000; trial = trial + 1) begin
            random_state = {random_state[30:0], random_state[31]
                            ^ random_state[21] ^ random_state[1] ^ random_state[0]};
            step(random_state[7:2] == 0, random_state[0], random_state[1]);
            if ((trial % 997) == 996) reset_cursor();
        end
        if (requests < 1000 || clear_request == 0 || clear_consume == 0
                || terminal_request < 3 || terminal_consume < 3
                || stalled_response == 0 || reset_count < 10)
            $fatal(1, "insufficient request-cursor coverage");
        $display("PASS wavelet request cursor cycles=%0d requests=%0d clear_request=%0d clear_consume=%0d terminal_request=%0d terminal_consume=%0d stalls=%0d resets=%0d",
                 cycles, requests, clear_request, clear_consume, terminal_request,
                 terminal_consume, stalled_response, reset_count);
        $finish;
    end

    initial begin
        #10000000;
        $fatal(1, "request-cursor test timed out");
    end
endmodule

`default_nettype wire
