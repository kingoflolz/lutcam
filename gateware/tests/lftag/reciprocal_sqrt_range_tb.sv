`timescale 1ns/1ps
module reciprocal_sqrt_range_tb;
    logic clk = 0;
    always #5 clk = ~clk;
    logic rst = 1;
    logic in_valid = 0;
    wire in_ready;
    logic [36:0] in_radicand = 0;
    wire out_valid;
    logic out_ready = 0;
    wire [18:0] out_lambda_q10;
    wire [3:0] out_lambda_left_shift;
    integer checked = 0;
    logic [31:0] random_state = 32'h1a2b3c4d;

    LfTagReciprocalSqrt512 dut (.*);

    task automatic tick;
        @(posedge clk);
        #1;
    endtask

    task automatic check_radicand(input logic [36:0] radicand);
        integer latency;
        logic [18:0] held_lambda;
        logic [3:0] held_shift;
        logic [31:0] effective_lambda;
        real ideal;
        real error;
        begin
            @(negedge clk);
            assert (in_ready && !out_valid) else $fatal(1, "not idle");
            in_valid = 1;
            in_radicand = radicand;
            out_ready = 0;
            tick();
            // Keep another request asserted throughout processing and output
            // backpressure. It must never overwrite the accepted request.
            @(negedge clk);
            in_radicand = ~radicand;
            latency = 0;
            while (!out_valid && latency < 24) begin
                assert (!in_ready) else $fatal(1, "accepted while busy");
                tick();
                latency = latency + 1;
            end
            assert (out_valid) else $fatal(1, "unbounded latency r=%0d", radicand);
            if (radicand == 0 || radicand[36]) begin
                assert (out_lambda_q10 == 0)
                    else $fatal(1, "invalid r=%0d returned %0d", radicand, out_lambda_q10);
            end else begin
                // Independent math oracle, without the DUT table or address
                // normalization. ROM midpoint error plus integer truncation.
                ideal = 67108864.0 / $sqrt(real'(radicand));
                effective_lambda = 32'(out_lambda_q10) << out_lambda_left_shift;
                error = real'(effective_lambda) - ideal;
                if (error < 0.0) error = -error;
                assert (out_lambda_q10 > 0 && out_lambda_q10 < (19'd1 << 17))
                    else $fatal(1, "representable r=%0d rejected: %0d", radicand, out_lambda_q10);
                assert (error <= ideal / 1024.0 + real'(32'd1 << out_lambda_left_shift))
                    else $fatal(1, "r=%0d got=%0d ideal=%f error=%f", radicand,
                                effective_lambda, ideal, error);
                assert (out_lambda_left_shift <= 9)
                    else $fatal(1, "invalid exponent r=%0d shift=%0d", radicand, out_lambda_left_shift);
                assert (latency == 12 + int'(out_lambda_left_shift))
                    else $fatal(1, "valid r=%0d latency=%0d expected=%0d", radicand, latency,
                                12 + int'(out_lambda_left_shift));
                if (radicand > (37'd1 << 18))
                    assert (out_lambda_left_shift == 0)
                        else $fatal(1, "previously supported radicand changed scaling");
            end
            held_lambda = out_lambda_q10;
            held_shift = out_lambda_left_shift;
            repeat (3) begin
                tick();
                assert (out_valid && !in_ready && out_lambda_q10 == held_lambda
                        && out_lambda_left_shift == held_shift)
                    else $fatal(1, "response changed under backpressure");
            end
            @(negedge clk);
            in_valid = 0;
            out_ready = 1;
            tick();
            assert (in_ready && !out_valid) else $fatal(1, "did not consume output");
            @(negedge clk);
            out_ready = 0;
            checked = checked + 1;
        end
    endtask

    task automatic reset_in_flight(input logic [36:0] radicand, input integer delay);
        begin
            @(negedge clk);
            in_valid = 1;
            in_radicand = radicand;
            tick();
            @(negedge clk);
            in_valid = 0;
            repeat (delay) tick();
            @(negedge clk);
            rst = 1;
            tick();
            assert (in_ready && !out_valid && out_lambda_q10 == 0 && out_lambda_left_shift == 0)
                else $fatal(1, "reset did not abort delay=%0d", delay);
            @(negedge clk);
            rst = 0;
            repeat (2) begin
                tick();
                assert (in_ready && !out_valid) else $fatal(1, "stale response after reset");
            end
            check_radicand(37'd1 << 26);
        end
    endtask

    initial begin : test
        logic [36:0] base;
        logic [36:0] step;
        logic [36:0] normalized;
        logic [36:0] radicand;
        repeat (2) tick();
        @(negedge clk);
        rst = 0;

        // Previously rejected: leading bit26 overshot bit35 to bit36.
        check_radicand(37'd1 << 26);
        // Maximum six signed18 squares / 2.
        check_radicand(37'd3 << 34);
        check_radicand(0);
        check_radicand(1);
        check_radicand((37'd1 << 18) - 1);
        check_radicand(37'd1 << 18);
        check_radicand((37'd1 << 18) + 1);
        check_radicand((37'd1 << 36) - 1);
        check_radicand(37'd1 << 36);
        check_radicand((37'd1 << 37) - 1);

        // Probe both sides of every exponent transition, including every
        // small-input exponent and the upper supported boundary.
        for (integer exponent = 1; exponent <= 36; exponent = exponent + 1) begin
            base = 37'd1 << exponent;
            check_radicand(base - 1);
            check_radicand(base);
            check_radicand(base + 1);
        end

        // Every midpoint-table bin endpoint at every normalization scale.
        // The reciprocal is monotonic within a bin, so these are its worst
        // approximation cases. Integer midpoints also exercise each address.
        for (integer shift = 0; shift <= 17; shift = shift + 1) begin
            for (integer bank = 0; bank < 2; bank = bank + 1) begin
                base = 37'd1 << (34 + bank);
                step = 37'd1 << (26 + bank);
                for (integer bin = 0; bin < 256; bin = bin + 1) begin
                    normalized = base + 37'(bin) * step;
                    check_radicand(normalized >> (2 * shift));
                    check_radicand((normalized + step / 2) >> (2 * shift));
                    check_radicand(((normalized + step) >> (2 * shift)) - 1);
                end
            end
        end

        for (integer sample_index = 0; sample_index < 1000; sample_index = sample_index + 1) begin
            random_state = random_state ^ (random_state << 13);
            random_state = random_state ^ (random_state >> 17);
            random_state = random_state ^ (random_state << 5);
            radicand = {1'b0, random_state, random_state[3:0]};
            check_radicand(radicand);
        end

        // Full small-norm integer range straddling the old lambda cap.
        for (integer tiny = 1; tiny < 1024; tiny++)
            check_radicand(37'(tiny));

        // Reset during normalization, both ROM waits, scaling, and a stalled
        // response. The three radicands vary the time spent in each state.
        for (integer delay = 0; delay <= 23; delay = delay + 1) begin
            reset_in_flight(1, delay);
            reset_in_flight((37'd1 << 18) + 1, delay);
            reset_in_flight(37'd1 << 26, delay);
            reset_in_flight(37'd1 << 35, delay);
        end
        $display("PASS reciprocal sqrt range: %0d oracle/backpressure checks, 96 resets", checked);
        $finish;
    end
endmodule
