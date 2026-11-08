`timescale 1ns/1ps
`default_nettype none

module horizontal_leaf_checker #(parameter integer WIDTH = 14)(
    input wire clk,
    output logic done = 0
);
    logic rst_n = 0;
    logic in_valid = 0, in_first = 0, in_last = 0;
    logic signed [WIDTH-1:0] in_even = 0, in_odd = 0;
    wire baseline_valid, baseline_first, baseline_last;
    wire candidate_valid, candidate_first, candidate_last;
    wire signed [WIDTH-1:0] baseline_low, baseline_high;
    wire signed [WIDTH-1:0] candidate_low, candidate_high;
    localparam integer MINIMUM = -(1 << (WIDTH-1));
    localparam integer MAXIMUM = (1 << (WIDTH-1))-1;
    integer cycles = 0, outputs = 0, rows = 0;
    integer bubbles = 0, held_cycles = 0, nonzero_held_cycles = 0;
    integer high_wraps = 0, low_wraps = 0;
    integer active_resets = 0, idle_resets = 0, async_checks = 0;
    integer one_pair_rows = 0, odd_pair_rows = 0, minimum_inputs = 0, maximum_inputs = 0;
    logic [31:0] random_state = 32'h7585d34b ^ WIDTH;
    logic [2*WIDTH+1:0] previous_held = 0;
    logic previous_running = 0;
    logic checking = 0;

    horizontal_lift_stage_baseline #(.DATA_WIDTH(WIDTH)) baseline (
        .clk, .rst_n, .in_valid, .in_first, .in_last, .in_even, .in_odd,
        .out_valid(baseline_valid), .out_first(baseline_first),
        .out_last(baseline_last), .out_low(baseline_low), .out_high(baseline_high)
    );
    horizontal_lift_stage_candidate #(.DATA_WIDTH(WIDTH)) candidate (
        .clk, .rst_n, .in_valid, .in_first, .in_last, .in_even, .in_odd,
        .out_valid(candidate_valid), .out_first(candidate_first),
        .out_last(candidate_last), .out_low(candidate_low), .out_high(candidate_high)
    );

    function automatic logic [31:0] next_random();
        random_state = random_state * 32'd1664525 + 32'd1013904223;
        return random_state;
    endfunction

    task automatic compare_all();
        if ({baseline_valid, baseline_first, baseline_last, baseline_low, baseline_high}
                !== {candidate_valid, candidate_first, candidate_last, candidate_low, candidate_high})
            $fatal(1, "WIDTH=%0d cycle=%0d output mismatch baseline=%h candidate=%h", WIDTH, cycles,
                {baseline_valid, baseline_first, baseline_last, baseline_low, baseline_high},
                {candidate_valid, candidate_first, candidate_last, candidate_low, candidate_high});
        if ({baseline.predict_valid_q, baseline.predict_high_q}
                !== {candidate.predict_valid_q, candidate.predict_high_q})
            $fatal(1, "WIDTH=%0d cycle=%0d predict register mismatch", WIDTH, cycles);
    endtask

    // Check every clock, including invalid cycles and reset assertion cycles.
    always @(posedge clk) begin : monitor
        integer high_unwrapped, low_unwrapped;
        if (checking && rst_n && !done) begin
            if (!in_valid) bubbles = bubbles + 1;
            if (in_valid) begin
                if ($signed(in_even) == MINIMUM || $signed(in_odd) == MINIMUM)
                    minimum_inputs = minimum_inputs + 1;
                if ($signed(in_even) == MAXIMUM || $signed(in_odd) == MAXIMUM)
                    maximum_inputs = maximum_inputs + 1;
            end
            if (baseline.launch) begin
                high_unwrapped = int'($signed(baseline.pending_odd))
                    - int'($signed(baseline.predicted_half));
                if (high_unwrapped < MINIMUM || high_unwrapped > MAXIMUM)
                    high_wraps = high_wraps + 1;
            end
            if (baseline.predict_valid_q) begin
                low_unwrapped = int'($signed(baseline.predict_even_q))
                    + int'($signed(baseline.update_quarter));
                if (low_unwrapped < MINIMUM || low_unwrapped > MAXIMUM)
                    low_wraps = low_wraps + 1;
            end
        end
        #1;
        if (checking && !done) begin
            compare_all();
            cycles = cycles + 1;
            if (rst_n) begin
                if (baseline_valid) begin
                    outputs = outputs + 1;
                    if (baseline_last) rows = rows + 1;
                end else if (previous_running) begin
                    if ({baseline_first, baseline_last, baseline_low, baseline_high} !== previous_held)
                        $fatal(1, "WIDTH=%0d invalid-cycle data/metadata hold failed", WIDTH);
                    held_cycles = held_cycles + 1;
                    if (baseline_low != 0 || baseline_high != 0)
                        nonzero_held_cycles = nonzero_held_cycles + 1;
                end
            end
            previous_held = {baseline_first, baseline_last, baseline_low, baseline_high};
            previous_running = rst_n;
        end
    end

    task automatic cycle(input logic valid, first, last,
        input logic signed [WIDTH-1:0] even_value, odd_value);
        @(negedge clk);
        in_valid = valid;
        in_first = first;
        in_last = last;
        in_even = even_value;
        in_odd = odd_value;
        @(posedge clk);
        #2;
    endtask

    task automatic idle_cycles(input integer count);
        for (integer i = 0; i < count; i = i + 1)
            cycle(0, 1, 1, WIDTH'(next_random()), WIDTH'(next_random()));
    endtask

    task automatic async_reset(input logic expect_active);
        @(negedge clk);
        #2;
        if (expect_active) begin
            if (!baseline.launch || !baseline.predict_valid_q)
                $fatal(1, "WIDTH=%0d expected both enables active before reset", WIDTH);
            active_resets = active_resets + 1;
        end else begin
            if (baseline.launch || baseline.predict_valid_q)
                $fatal(1, "WIDTH=%0d expected both enables low before reset", WIDTH);
            idle_resets = idle_resets + 1;
        end
        rst_n = 0;
        in_valid = 0;
        #1;
        // This check occurs two nanoseconds before the next rising clock edge.
        compare_all();
        if ({candidate_valid, candidate_first, candidate_last, candidate_low, candidate_high,
                candidate.predict_valid_q, candidate.predict_high_q} !== '0)
            $fatal(1, "WIDTH=%0d asynchronous reset failed before clock edge", WIDTH);
        async_checks = async_checks + 1;
        previous_running = 0;
        @(negedge clk);
        #2;
        rst_n = 1;
    endtask

    initial begin : stimulus
        integer row_length, bubble_count;
        logic signed [WIDTH-1:0] even_value, odd_value;
        // The vendor model initializes W and shift to zero, but Icarus does
        // not initially evaluate its plain always @(*) shifter when neither
        // input changes. Prime a nonzero row before the proof's first reset;
        // the frozen vendor source itself remains unmodified. All subsequent
        // clocks and every reset, including nonzero-to-zero reset, are checked.
        #2;
        rst_n = 1;
        cycle(1, 1, 0, WIDTH'(3), WIDTH'(9));
        cycle(1, 0, 1, WIDTH'(5), WIDTH'(11));
        idle_cycles(5);
        checking = 1;
        async_reset(0);
        idle_cycles(8);
        for (integer row_index = 0; row_index < 400; row_index = row_index + 1) begin
            case (row_index % 9)
                0: row_length = 1;
                1: row_length = 3;
                2: row_length = 5;
                3: row_length = 7;
                4: row_length = 17;
                default: row_length = 1 + int'(next_random() % 31);
            endcase
            if (row_length == 1) one_pair_rows = one_pair_rows + 1;
            if (row_length % 2) odd_pair_rows = odd_pair_rows + 1;
            for (integer pair_index = 0; pair_index < row_length; pair_index = pair_index + 1) begin
                // Some rows are gap-free; others contain independently varied valid bubbles.
                bubble_count = (row_index % 4 == 0) ? 0 : int'(next_random() % 4);
                idle_cycles(bubble_count);
                case (row_index % 6)
                    0: begin even_value = WIDTH'(MINIMUM); odd_value = WIDTH'(MAXIMUM); end
                    1: begin
                        even_value = (pair_index % 2) ? WIDTH'(MAXIMUM) : WIDTH'(MINIMUM);
                        odd_value = (pair_index % 2) ? WIDTH'(MINIMUM) : WIDTH'(MAXIMUM);
                    end
                    2: begin even_value = WIDTH'(MAXIMUM); odd_value = WIDTH'(MINIMUM); end
                    default: begin even_value = WIDTH'(next_random()); odd_value = WIDTH'(next_random()); end
                endcase
                cycle(1, pair_index == 0, pair_index == row_length-1, even_value, odd_value);
            end
            if (row_index % 50 == 49) begin
                idle_cycles(5);
                // Reset with both register enables low, then with both high mid-row.
                async_reset(0);
                idle_cycles(8);
                cycle(1, 1, 0, WIDTH'(MINIMUM), WIDTH'(MAXIMUM));
                cycle(1, 0, 1, WIDTH'(MAXIMUM), WIDTH'(MINIMUM));
                async_reset(1);
                idle_cycles(8);
            end
        end
        idle_cycles(24);
        if (rows != 400 || outputs < 2000 || high_wraps < 100 || low_wraps < 100
                || nonzero_held_cycles < 100 || active_resets != 8 || idle_resets != 9
                || async_checks != 17 || one_pair_rows < 40 || odd_pair_rows < 200
                || minimum_inputs < 100 || maximum_inputs < 100)
            $fatal(1, "WIDTH=%0d insufficient coverage: rows=%0d outputs=%0d high_wraps=%0d low_wraps=%0d holds=%0d",
                WIDTH, rows, outputs, high_wraps, low_wraps, nonzero_held_cycles);
        $display("RESULT width=%0d cycles=%0d outputs=%0d rows=%0d bubbles=%0d held_cycles=%0d nonzero_held_cycles=%0d high_wraps=%0d low_wraps=%0d active_resets=%0d idle_resets=%0d async_checks=%0d one_pair_rows=%0d odd_pair_rows=%0d minimum_inputs=%0d maximum_inputs=%0d",
            WIDTH, cycles, outputs, rows, bubbles, held_cycles, nonzero_held_cycles,
            high_wraps, low_wraps, active_resets, idle_resets, async_checks,
            one_pair_rows, odd_pair_rows, minimum_inputs, maximum_inputs);
        done = 1;
    end
endmodule

module leaf_parity_tb;
`ifdef LEAF_TRACE
    initial begin
        $dumpfile("startup.vcd");
        $dumpvars(0, leaf_parity_tb);
    end
`endif
    logic clk = 0;
    always #5 clk = ~clk;
    wire [3:0] done;
    horizontal_leaf_checker #(.WIDTH(9)) width9 (.clk, .done(done[0]));
    horizontal_leaf_checker #(.WIDTH(10)) width10 (.clk, .done(done[1]));
    horizontal_leaf_checker #(.WIDTH(14)) width14 (.clk, .done(done[2]));
    horizontal_leaf_checker #(.WIDTH(18)) width18 (.clk, .done(done[3]));
    initial begin
        wait (&done);
        $display("PASS horizontal DSP leaf parity: all four widths");
        $finish;
    end
    initial begin
        #2000000;
        $fatal(1, "horizontal DSP leaf parity timeout");
    end
endmodule
`default_nettype wire
