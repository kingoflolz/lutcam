`timescale 1ns/1ps

module counter_check #(parameter WIDTH = 14)(
    input wire clk, reset_n, clear, enable, checking,
    output wire done
);
    wire [WIDTH-1:0] optional_value, portable_value;
    reg [WIDTH-1:0] expected;
    integer cycles = 0, wraps = 0, holds = 0, clears = 0, async_checks = 0;
    integer sync_clear_checks = 0;
    Ov9281Counter #(.WIDTH(WIDTH), .USE_DSP(1)) optional_counter (
        .clk, .reset_n, .clear, .enable, .value(optional_value));
    Ov9281Counter #(.WIDTH(WIDTH), .USE_DSP(0)) portable_counter (
        .clk, .reset_n, .clear, .enable, .value(portable_value));

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) expected <= '0;
        else if (clear) expected <= '0;
        else if (enable) expected <= expected + 1'b1;
        if (checking && reset_n) begin
            if (clear) clears = clears + 1;
            else if (!enable && expected != 0) holds = holds + 1;
            else if (enable && (&expected)) wraps = wraps + 1;
        end
    end
    task check_value;
        if (optional_value !== expected || portable_value !== expected)
            $fatal(1, "width=%0d cycle=%0d expected=%h optional=%h portable=%h",
                WIDTH, cycles, expected, optional_value, portable_value);
    endtask
    always @(negedge clk) if (checking) begin
        check_value();
        cycles = cycles + 1;
    end
    always @(negedge reset_n) if (checking) begin
        #0.1;
        check_value();
        if (optional_value !== 0) $fatal(1, "asynchronous reset failed");
        async_checks = async_checks + 1;
    end
    always @(posedge clear) if (checking && reset_n && expected != 0) begin
        // A clear request between clocks must leave the visible value intact.
        #0.1;
        check_value();
        sync_clear_checks = sync_clear_checks + 1;
    end
    assign done = cycles > 524288 && (WIDTH > 19 || wraps > 0) &&
        holds > 0 && clears > 0 && async_checks == 17 && sync_clear_checks > 0;
    final $display("RESULT width=%0d cycles=%0d wraps=%0d holds=%0d clears=%0d async_checks=%0d sync_clear_checks=%0d",
        WIDTH, cycles, wraps, holds, clears, async_checks, sync_clear_checks);
endmodule

module csi_counter_tb;
    reg clk = 0, reset_n = 1, clear = 0, enable = 1, checking = 0;
    wire [5:0] done;
    counter_check #(7) c7(.*, .done(done[0]));
    counter_check #(10) c10(.*, .done(done[1]));
    counter_check #(14) c14(.*, .done(done[2]));
    counter_check #(19) c19(.*, .done(done[3]));
    counter_check #(20) c20(.*, .done(done[4]));
    counter_check #(32) c32(.*, .done(done[5]));
    reg [31:0] random_state = 32'hc51c0a17;
    task tick;
        #1 clk = 1;
        #1 clk = 0;
        #0.1;
    endtask
    initial begin
        // The vendor Icarus model's combinational shifter does not re-evaluate
        // a constant initial zero. Prime it, then begin with a fresh async
        // reset. No internal signal is forced; full-receiver Verilator tests
        // separately exercise cold initialization at the production profile.
        #1 reset_n = 0;
        tick();
        reset_n = 1;
        tick();
        reset_n = 0;
        #1 checking = 1;
        reset_n = 1;

        // Cross the 19-bit boundary, including bit18 as an unsigned operand.
        repeat (524292) tick();
        for (integer phase = 0; phase < 17; phase++) begin
            for (integer i = 0; i < 100; i++) begin
                random_state = (random_state >> 1) ^
                    (32'h80200003 & {32{random_state[0]}});
                clear = random_state[3:0] == 0;
                enable = random_state[4];
                tick();
            end
            // Clear must act on the clock edge even when CE was disabled,
            // and must win over a simultaneous increment request.
            clear = 1; enable = phase[0]; tick();
            clear = 0; enable = 1; repeat (5) tick();
            #0.2 reset_n = 0;
            #0.2;
            reset_n = 1;
        end
        clear = 0; enable = 0; tick();
        if (&done !== 1'b1) $fatal(1, "coverage incomplete: %b", done);
        $display("PASS CSI counters: six widths, wrap/hold/clear priority/async reset");
        $finish;
    end
endmodule
