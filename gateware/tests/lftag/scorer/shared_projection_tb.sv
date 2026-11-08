module shared_projection_tb;
    logic clk = 0;
    always #5 clk = !clk;
    logic rst = 1, clear = 0, start_valid = 0;
    wire start_ready;
    logic [1:0] in_scale = 0;
    // Exact accepted ID0855 geometry captured from the existing all-sixteen-
    // hypotheses Rust/RTL fixture, with its original descriptor ordering.
    logic [71:0] corner_x_flat = 72'h0430016c005c001100;
    logic [71:0] corner_y_flat = 72'h0470011c0030000c00;
    logic [19:0] point_input_index_flat = 20'h53210;
    wire descriptor_req_valid;
    wire descriptor_req_ready;
    wire [3:0] descriptor_req_index;
    logic descriptor_rsp_valid = 0;
    logic signed [17:0] descriptor_rsp_x = 0, descriptor_rsp_y = 0;
    wire geometry_done_valid, geometry_done_invalid;
    wire signed [17:0] map_cden, map_cg, map_ch;
    wire out_valid;
    logic out_ready = 0;
    wire out_best_valid, out_input_invalid;
    wire [3:0] out_best_hypothesis;
    wire [13:0] out_best_id;
    wire [63:0] out_best_score_q32;
    wire [37:0] out_best_residual_sum;
    wire [38:0] out_best_weight_sum;
    wire [9:0] out_cycles;
    LfTagHomogeneousGeometryScorer dut (.*);

    integer cycles = 0, requests = 0;
    integer response_x_cycles = 0, response_y_cycles = 0, projection_checks = 0;
    integer pending_index = 0, response_delay = 0;
    bit pending = 0;
    integer complete_jobs = 0, clear_cases = 0, reset_cases = 0;
    longint unsigned expected_residual [0:15], expected_weight [0:15];
    always @(posedge clk) begin
        if (rst || clear) begin
            foreach (expected_residual[h]) begin
                expected_residual[h] = 0;
                expected_weight[h] = 0;
            end
        end else if (dut.state == 6'd32) begin
            expected_residual[dut.score_hypothesis] = 0;
            expected_weight[dut.score_hypothesis] = 0;
        end else if (dut.state == 6'd39 || dut.state == 6'd40) begin
            expected_residual[dut.score_hypothesis]
                += 64'($signed(dut.decoded_axis_error)) * 64'($signed(dut.decoded_axis_error));
            if (dut.state == 6'd39)
                expected_weight[dut.score_hypothesis]
                    += 64'($signed(dut.projected_w)) * 64'($signed(dut.projected_w));
        end
    end
    integer state_hits [0:44];
    integer abort_targets [0:8] = '{0, 23, 24, 36, 38, 39, 40, 41, 43};
    assign descriptor_req_ready = (cycles % 5) != 0;

    function automatic integer point_x(input integer index);
        integer gx, symbol;
        case (index)
            1, 4: gx = -1;
            3, 6: gx = 1;
            default: gx = 0;
        endcase
        symbol = ('h0855 >> (2*index)) & 3;
        return 5120 + gx*768 + ((symbol & 2) ? 64 : -64);
    endfunction
    function automatic integer point_y(input integer index);
        integer gy, symbol;
        if (index == 0) gy = -1;
        else if (index >= 4) gy = 1;
        else gy = 0;
        symbol = ('h0855 >> (2*index)) & 3;
        return 3840 + gy*768 + ((symbol & 1) ? 64 : -64);
    endfunction

    // Independent one-outstanding descriptor responder with changing request
    // backpressure and response latency. Abort flushes the external request.
    always @(posedge clk) begin
        cycles <= cycles + 1;
        descriptor_rsp_valid <= 0;
        if (rst || clear) begin
            pending <= 0;
            response_delay <= 0;
        end else begin
            if (descriptor_req_valid && descriptor_req_ready) begin
                if (pending) $fatal(1, "overlapping descriptor request");
                pending <= 1;
                pending_index <= descriptor_req_index;
                response_delay <= 1 + (requests % 4);
                requests <= requests + 1;
            end
            if (pending) begin
                if (response_delay != 0) response_delay <= response_delay - 1;
                else begin
                    descriptor_rsp_valid <= 1;
                    descriptor_rsp_x <= 18'(point_x(pending_index));
                    descriptor_rsp_y <= 18'(point_y(pending_index));
                    pending <= 0;
                end
            end
        end
    end
    always @(negedge clk) begin
        if (!rst && dut.state <= 44)
            state_hits[dut.state]++;
        if (!rst && dut.state == 39) response_x_cycles++;
        if (!rst && dut.state == 40) response_y_cycles++;
    end

    task automatic tick(input integer count = 1);
        repeat (count) begin @(negedge clk); #1; end
    endtask
    task automatic start_job;
        if (!start_ready) $fatal(1, "start issued while not ready");
        start_valid = 1;
        tick();
        start_valid = 0;
    endtask
    task automatic reach_state(input integer target);
        bit found;
        found = 0;
        for (integer n = 0; n < 1500; n++) begin
            if (dut.state == target) begin found = 1; break; end
            tick();
        end
        if (!found) $fatal(1, "did not reach target state %0d", target);
    endtask
    task automatic finish_job;
        logic [168:0] held;
        reach_state(43);
        if (!out_valid || !out_best_valid || out_input_invalid || out_best_id != 14'h0855)
            $fatal(1, "direct meaningful fixture was not accepted");
        if (out_best_score_q32 != 64'h2ae)
            $fatal(1, "known accepted geometry score changed");
        if (out_best_residual_sum != expected_residual[out_best_hypothesis]
                || out_best_weight_sum != expected_weight[out_best_hypothesis])
            $fatal(1, "winning raw residual / projective denominator retirement changed");
        if (out_cycles < 540 || out_cycles > 560)
            $fatal(1, "known scorer fixture service budget changed: %0d", out_cycles);
        held = {out_best_hypothesis, out_best_id, out_best_score_q32,
                out_cycles, out_best_residual_sum, out_best_weight_sum};
        tick(7);
        if (!out_valid || held !== {out_best_hypothesis, out_best_id,
                                   out_best_score_q32, out_cycles,
                                   out_best_residual_sum, out_best_weight_sum})
            $fatal(1, "output changed under backpressure");
        out_ready = 1;
        tick();
        out_ready = 0;
        if (!start_ready) $fatal(1, "output drain did not return idle");
        complete_jobs++;
    endtask

    initial begin
        foreach (state_hits[i]) state_hits[i] = 0;
        tick(3); rst = 0; tick(2);
        start_job(); finish_job();
        // Test all important control phases with clear+start, then reset+clear+
        // start. The overlapping start must lose, followed by a clean job.
        foreach (abort_targets[i]) begin
            for (integer reset_mode = 0; reset_mode < 2; reset_mode++) begin
                if (abort_targets[i] != 0) start_job();
                reach_state(abort_targets[i]);
                clear = 1;
                rst = (reset_mode != 0);
                start_valid = 1;
                tick();
                if (!start_ready || out_valid)
                    $fatal(1, "abort did not override state/start at %0d", abort_targets[i]);
                clear = 0; rst = 0; start_valid = 0;
                tick(2);
                if (!start_ready || out_valid) $fatal(1, "aborted start leaked");
                if (reset_mode) reset_cases++; else clear_cases++;
                start_job(); finish_job();
            end
        end
        if (complete_jobs != 19 || clear_cases != 9 || reset_cases != 9 ||
            response_x_cycles < 1360 || response_y_cycles < 1360 ||
            state_hits[23] == 0 || state_hits[24] == 0 || state_hits[36] == 0 ||
            state_hits[39] == 0 || state_hits[40] == 0 || state_hits[41] == 0 ||
            state_hits[38] == 0 || projection_checks < 4800)
            $fatal(1, "vacuous directed control coverage");
        $display("PASS shared projection jobs=%0d clear=%0d reset=%0d cycles=%0d X=%0d Y=%0d requests=%0d",
                 complete_jobs, clear_cases, reset_cases, cycles,
                 response_x_cycles, response_y_cycles, requests);
        $finish;
    end
    logic reference_projection_valid = 0;
    logic reference_projection_w;
    logic signed [19:0] reference_projection;
    logic signed [36:0] reference_sum;
    always @(posedge clk) begin
        reference_projection_valid = !rst && ((dut.state == 6'd36)
            || (dut.state == 6'd38) || (dut.state == 6'd39)
            || (dut.state == 6'd41));
        if (reference_projection_valid) begin
            projection_checks++;
            reference_projection_w = (dut.state == 6'd38) || (dut.state == 6'd41);
            if (reference_projection_w)
                reference_sum = $signed(dut.coefficient_6)*$signed(dut.score_u)
                    + $signed(dut.coefficient_7)*$signed(dut.score_v);
            else if (dut.state == 6'd39)
                reference_sum = $signed(dut.coefficient_3)*$signed(dut.score_u)
                    + $signed(dut.coefficient_4)*$signed(dut.score_v);
            else
                reference_sum = $signed(dut.coefficient_0)*$signed(dut.score_u)
                    + $signed(dut.coefficient_1)*$signed(dut.score_v);
            reference_projection = 20'($signed(reference_sum) >>> 8)
                + (reference_projection_w
                    ? {{2{dut.memory_q_0[17]}},dut.memory_q_0[17:0]}
                    : -$signed({{2{dut.memory_q_0[17]}},dut.memory_q_0[17:0]}));
        end
    end
    always @(negedge clk) begin
        if (reference_projection_valid && (reference_projection_w
            ? (dut.projected_axis[18:0] !== reference_projection[18:0])
            : (dut.projected_axis !== reference_projection)))
            $fatal(1,"native projection differs from original shift-then-Z expression");
    end

    initial begin #2000000; $fatal(1, "testbench timeout"); end
endmodule
