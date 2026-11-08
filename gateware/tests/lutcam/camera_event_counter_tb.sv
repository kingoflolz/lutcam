`timescale 1ns/1ps
`default_nettype none

module camera_event_counter_tb;
    reg clk = 0, rst = 1, clear = 0, enable = 0;
    wire [31:0] fabric, dsp;
    // Omitting USE_DSP also checks that portable behavior is the default.
    CameraEventCounter f(.clk, .rst, .clear, .enable, .value(fabric));
    CameraEventCounter #(.USE_DSP(1)) d(.clk, .rst, .clear, .enable, .value(dsp));
    reg [31:0] expected = 0;
    reg [63:0] rng = 64'h81ca921579ef786d;
    integer cycle, wraps = 0, reset_enable = 0, clear_enable = 0, clears = 0;
    reg [31:0] previous;
    always #5 clk = ~clk;

    // Literal original CameraPipeline ordering: the later event assignment
    // overrides a local clear and increments the OLD counter, not zero.
    always @(posedge clk) begin
        if (rst) expected <= 0;
        else begin
            if (clear) expected <= 0;
            if (enable) expected <= expected + 1'b1;
        end
    end

    initial begin
        repeat (3) @(negedge clk);
        rst = 0;
        for (cycle = 0; cycle < 1000000; cycle = cycle + 1) begin
            @(negedge clk);
            rng = {rng[62:0], rng[63] ^ rng[62] ^ rng[60] ^ rng[59]};
            enable = rng[0];
            clear = (cycle % 97 == 0);
            rst = (cycle % 2039 == 100);
            if (cycle % 10000 == 0) begin
                // Exercise 32-bit wrap while the DSP's hidden upper bits
                // are nonzero. They must never affect the public low word.
                expected = 32'hfffffff0;
                f.fabric_counter.accumulator = 32'hfffffff0;
                d.native_counter.counter.genblk1.i_DSP48.W_pipe.O_r = 48'hbcdefffffff0;
                rst = 0;
                clear = 0;
            end
            previous = expected;
            @(posedge clk);
            #1;
            if (fabric !== expected || dsp !== expected)
                $fatal(1, "count mismatch cycle=%0d fabric=%h dsp=%h expected=%h",
                    cycle, fabric, dsp, expected);
            if (!rst && !clear && dsp < previous) wraps++;
            if (rst && enable) reset_enable++;
            if (!rst && clear && enable) clear_enable++;
            if (!rst && clear && !enable) clears++;
        end
        if (wraps < 20 || reset_enable < 100 || clear_enable < 100 || clears < 100)
            $fatal(1, "coverage incomplete");
        $display("PASS camera counters: 1000000 cycles wraps=%0d reset_enable=%0d clear_enable=%0d clear_only=%0d",
            wraps, reset_enable, clear_enable, clears);
        $finish;
    end
endmodule
`default_nettype wire
