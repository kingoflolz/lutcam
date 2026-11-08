// Compare the optional narrow configuration port with the direct-port mode.
// Requests, responses, pose output, and accepted-centroid output all stall.
module config_fetch_tb;
    logic clk = 0, rst = 1;
    always
        #5 clk = ~clk;
    logic [1 : 0] camera_reset = 0, in_valid = 0, in_group_end = 0, in_centroid_valid = 0;
    logic [1 : 0][14 : 0] in_x_q4 = 0;
    logic [1 : 0][13 : 0] in_y_q4 = 0;
    logic [1 : 0][7 : 0] in_log_mass_q5_3 = 0, in_frame_seq = 0;
    logic [1 : 0][1 : 0] in_scale = 0;
    logic [1 : 0][31 : 0] cfg_inv_fx_q2_30, cfg_inv_fy_q2_30, cfg_cx_q16_16, cfg_cy_q16_16,
                          cfg_printed_width_q16_16;
    logic [1 : 0][15 : 0] cfg_mean_residual_limit_q16;
    logic [1 : 0][7 : 0] cfg_max_mass_spread;
    logic [1 : 0][15 : 0] cfg_min_baseline_q4;
    logic [1 : 0] out_ready, centroid_ready, clear_diagnostics = 0;
    wire [1 : 0] bv, bl, bcv, bcl, bb, cv, cl, ccv, ccl, cb, bdone, cdone, bdrop, cdrop, berr, cerr;
    wire [31 : 0] bdata, cdata;
    wire [32 : 0] bcdata, ccdata;
    wire [1 : 0][9 : 0] ccycles;
    wire config_read_valid;
    wire [3 : 0] config_read_address;
    logic config_read_ready, config_response_valid = 0;
    logic [31 : 0] config_read_data = 0;
    logic [31 : 0] rng = 32'h652199ac, pending_data = 0;
    logic pending = 0, delay_response = 0;
    logic [1 : 0] request_wait = 0;
    logic fixed_response = 0, block_request = 0;
    logic [3 : 0] abort_address = 0;
    logic held = 0;
    logic [3 : 0] held_address = 0;
    logic [15 : 0] addresses_seen = 0;
    integer req_count = 0, job_count = 0, cycle_count = 0, max_cycles = 0;
    logic [32 : 0] packets[0 : 1][0 : 4095];
    logic [33 : 0] centroids[0 : 1][0 : 1023];
    integer pw[2] = '{0, 0}, pr[2] = '{0, 0}, cw[2] = '{0, 0}, cr[2] = '{0, 0};
    integer packet_slot[2] = '{0, 0};
    logic [32 : 0] expected_packet;
    logic [33 : 0] expected_centroid;
    logic [14 : 0] px[2][9];
    logic [13 : 0] py[2][9];
    logic [7 : 0] mass[2][9];
    logic [63 : 0] forced_best_score;
    logic [37:0] forced_residual_sum;
    logic [38:0] forced_weight_sum;
    assign out_ready = {rng[7 : 6] != 0, rng[5 : 4] != 0};
    assign centroid_ready = {rng[11 : 10] != 0, rng[9 : 8] != 0};
    assign config_read_ready = fixed_response
                                   ? !(block_request && config_read_address == abort_address)
                                   : !pending && (request_wait == 2 || rng[0]);
    function automatic logic [31 : 0] cfg_word
        (input logic [3 : 0] addr);
        case (addr)
            0:
                cfg_word = cfg_printed_width_q16_16[0];
            1:
                // Upper bits are reserved and cannot affect the Q0.16 limit.
                cfg_word = {16'hbad5, cfg_mean_residual_limit_q16[0]};
            2, 3, 4, 5, 6:
                cfg_word = 32'hdeadbeef;
            7:
                cfg_word = {8'b0, cfg_min_baseline_q4[0], cfg_max_mass_spread[0]};
            8, 12:
                cfg_word = cfg_inv_fx_q2_30[addr[2]];
            9, 13:
                cfg_word = cfg_inv_fy_q2_30[addr[2]];
            10, 14:
                cfg_word = cfg_cx_q16_16[addr[2]];
            11, 15:
                cfg_word = cfg_cy_q16_16[addr[2]];
        endcase
    endfunction
    always @(posedge clk) begin
        if (rst) begin
            rng <= 32'h652199ac;
            pending <= 0;
            config_response_valid <= 0;
            request_wait <= 0;
            held <= 0;
            pw[0] <= 0;
            pw[1] <= 0;
            pr[0] <= 0;
            pr[1] <= 0;
            cw[0] <= 0;
            cw[1] <= 0;
            cr[0] <= 0;
            cr[1] <= 0;
            packet_slot[0] <= 0;
            packet_slot[1] <= 0;
            job_count <= 0;
            req_count <= 0;
            cycle_count <= 0;
            addresses_seen <= 0;
        end else begin
            rng <= {rng[30 : 0], rng[31] ^ rng[21] ^ rng[1] ^ rng[0]};
            cycle_count <= cycle_count + 1;
            if (cycle_count > 20000000)
                $fatal(1, "shared timeout");
            if (held && !camera_reset[dut.request_camera] &&
                (!config_read_valid || held_address != config_read_address))
                $fatal(1, "shared request moved under backpressure");
            held <= config_read_valid && !config_read_ready && !camera_reset[dut.request_camera];
            held_address <= config_read_address;
            if (config_read_valid && !config_read_ready)
                request_wait <= request_wait + 1'b1;
            else
                request_wait <= 0;
            config_response_valid <= 0;
            if (fixed_response) begin
                pending <= 0;
                config_response_valid <= config_read_valid && config_read_ready;
                if (config_read_valid && config_read_ready)
                    config_read_data <= cfg_word(config_read_address);
            end
            if (config_read_valid && config_read_ready) begin
                if (config_read_address >= 2 && config_read_address <= 6)
                    $fatal(1, "reserved configuration word was fetched");
                if (!fixed_response) begin
                    pending <= 1;
                    delay_response <= rng[1];
                    pending_data <= cfg_word(config_read_address);
                end
                addresses_seen[config_read_address] <= 1;
                req_count <= req_count + 1;
                if (config_read_address[3] && config_read_address[2] != dut.request_camera)
                    $fatal(1, "wrong owner config bank");
            end
            if (pending && !fixed_response) begin
                if (delay_response)
                    delay_response <= 0;
                else begin
                    pending <= 0;
                    config_response_valid <= 1;
                    config_read_data <= pending_data;
                end
            end
            for (integer camera = 0; camera < 2; camera++) begin
                if (camera_reset[camera]) begin
                    pw[camera] <= 0;
                    pr[camera] <= 0;
                    cw[camera] <= 0;
                    cr[camera] <= 0;
                    packet_slot[camera] <= 0;
                end else begin
                    if (bv[camera] && out_ready[camera]) begin
                        if ((packet_slot[camera] == 4 || packet_slot[camera] == 5) && bdata !== 0)
                            $fatal(1, "reserved packet word is nonzero camera=%0d word=%0d",
                                   camera, packet_slot[camera]);
                        packet_slot[camera] <= bl[camera] ? 0 : packet_slot[camera] + 1;
                        packets[camera][pw[camera]] <= {bl[camera], bdata};
                        pw[camera] <= pw[camera] + 1;
                    end
                    if (cv[camera] && out_ready[camera]) begin
                        if (pr[camera] > pw[camera] ||
                            (pr[camera] == pw[camera] && !(bv[camera] && out_ready[camera])))
                            $fatal(1, "shared candidate precedes baseline");
                        expected_packet = pr[camera] == pw[camera] ? {bl[camera], bdata}
                                                                   : packets[camera][pr[camera]];
                        if ({cl[camera], cdata} !== expected_packet)
                            $fatal(1, "shared packet camera%0d index%0d got%h expected%h", camera,
                                   pr[camera], {cl[camera], cdata}, expected_packet);
                        pr[camera] <= pr[camera] + 1;
                    end
                    if (bcv[camera] && centroid_ready[camera]) begin
                        centroids[camera][cw[camera]] <= {bcl[camera], bcdata};
                        cw[camera] <= cw[camera] + 1;
                    end
                    if (ccv[camera] && centroid_ready[camera]) begin
                        if (cr[camera] > cw[camera] ||
                            (cr[camera] == cw[camera] && !(bcv[camera] && centroid_ready[camera])))
                            $fatal(1, "shared centroid precedes baseline");
                        expected_centroid = cr[camera] == cw[camera]
                                                ? {bcl[camera], bcdata}
                                                : centroids[camera][cr[camera]];
                        if ({ccl[camera], ccdata} !== expected_centroid)
                            $fatal(1, "shared centroid mismatch");
                        cr[camera] <= cr[camera] + 1;
                    end
                    if (cdone[camera]) begin
                        job_count <= job_count + 1;
                        if (ccycles[camera] > max_cycles)
                            max_cycles <= ccycles[camera];
                        if (ccycles[camera] > 900)
                            $fatal(1, "accepted compute budget exceeded");
                    end
                end
            end
            if (bdrop || cdrop || berr || cerr)
                $fatal(1, "unexpected shared FIFO diagnostic");
        end
    end
    SharedLfTagDecoder #(.EXPORT_ACCEPTED_CENTROIDS(1),
                         .EXTERNAL_CONFIG(1))
        dut(.out_valid(cv),
            .out_last(cl),
            .out_data(cdata),
            .centroid_valid(ccv),
            .centroid_last(ccl),
            .centroid_data(ccdata),
            .busy(cb),
            .candidate_done(cdone),
            .candidate_cycles(ccycles),
            .candidate_start(),
            .fifo_level(),
            .fifo_drop_sticky(cdrop),
            .protocol_error_sticky(cerr),
            .debug_status(),
            .debug_protocol(),
            .debug_drop(),
            .*);
    SharedLfTagDecoder #(.EXPORT_ACCEPTED_CENTROIDS(1))
        baseline(.config_read_valid(),
                 .config_read_address(),
                 .config_read_ready(1'b0),
                 .config_response_valid(1'b0),
                 .config_read_data(32'b0),
                 .out_valid(bv),
                 .out_last(bl),
                 .out_data(bdata),
                 .centroid_valid(bcv),
                 .centroid_last(bcl),
                 .centroid_data(bcdata),
                 .busy(bb),
                 .candidate_done(bdone),
                 .candidate_cycles(),
                 .candidate_start(),
                 .fifo_level(),
                 .fifo_drop_sticky(bdrop),
                 .protocol_error_sticky(berr),
                 .debug_status(),
                 .debug_protocol(),
                 .debug_drop(),
                 .*);
    task tick;
        begin
            @(posedge clk);
            @(negedge clk);
        end
    endtask

    // Keep shared words identical between cameras; only intrinsics have two banks.
    task defaults;
        begin
            cfg_inv_fx_q2_30 = {32'h00234567, 32'h00222222};
            cfg_inv_fy_q2_30 = {32'h00245678, 32'h00222222};
            cfg_cx_q16_16 = {32'd322 << 16, 32'd320 << 16};
            cfg_cy_q16_16 = {32'd238 << 16, 32'd240 << 16};
            cfg_printed_width_q16_16 = {2{32'h3333}};
            cfg_mean_residual_limit_q16 = {2{16'd1024}};
            cfg_max_mass_spread = {2{8'd19}};
            cfg_min_baseline_q4 = {2{16'd16}};
        end
    endtask

    task send_pair
        (input integer frame,
         input integer alternate_ids,
         input logic invalid_centroid);
        begin
            for (integer slot = 0; slot < 9; slot++) begin
                in_valid = 3;
                in_group_end = {2{slot == 8}};
                in_frame_seq = {8'(frame), 8'(frame)};
                for (integer camera = 0; camera < 2; camera++) begin
                    in_x_q4[camera] = px[alternate_ids ? (camera + frame) % 2 : 0][slot];
                    in_y_q4[camera] = py[alternate_ids ? (camera + frame) % 2 : 0][slot];
                    in_log_mass_q5_3[camera] = mass[alternate_ids ? (camera + frame) % 2 : 0][slot];
                    in_centroid_valid[camera] = !(invalid_centroid && slot == 3);
                end
                tick();
                in_valid = 0;
                in_group_end = 0;
                tick();
            end
        end
    endtask

    task drain;
        begin
            while (cb || bb)
                tick();
            repeat (3)
                tick();
            for (integer camera = 0; camera < 2; camera++) begin
                if (pr[camera] != pw[camera] || cr[camera] != cw[camera])
                    $fatal(1, "undrained output camera %0d", camera);
            end
        end
    endtask

    task run_boundary
        (input integer field,
         input logic [63 : 0] value);
        begin
            // Config writes are legal only after both camera jobs and output replays drain.
            drain();
            defaults();
            case (field)
                0:
                    cfg_mean_residual_limit_q16 = {2{value[15 : 0]}};
                3:
                    cfg_max_mass_spread = {2{value[7 : 0]}};
                4:
                    cfg_min_baseline_q4 = {2{value[15 : 0]}};
                5:
                    cfg_printed_width_q16_16 = {2{value[31 : 0]}};
                6:
                    cfg_inv_fx_q2_30 = {2{value[31 : 0]}};
                7:
                    cfg_inv_fy_q2_30 = {2{value[31 : 0]}};
                8:
                    cfg_cx_q16_16 = {2{value[31 : 0]}};
                9:
                    cfg_cy_q16_16 = {2{value[31 : 0]}};
                default:
                    $fatal(1, "unknown config field");
            endcase
            send_pair(job_count / 2, 0, 0);
            drain();
        end
    endtask


    task run_tied_candidate;
        integer start_words[2], start_centroids[2], first_words[2], first_centroids[2];
        begin
            drain(); defaults(); in_scale = 0;
            // Symmetric cell-center geometry creates an exact hypothesis tie.
            // Descriptors use ascending mass, with the two anchors last.
            px[0] = '{5120,4864,5120,5376,4864,5120,5376,4864,5376};
            py[0] = '{3584,3840,3840,3840,4096,4096,4096,3584,3584};
            mass[0] = '{104,104,104,104,104,104,104,112,112};
            for (integer repeat_index = 0; repeat_index < 2; repeat_index++) begin
                for (integer camera = 0; camera < 2; camera++) begin
                    start_words[camera] = pw[camera];
                    start_centroids[camera] = cw[camera];
                    if (repeat_index == 0) begin
                        first_words[camera] = pw[camera];
                        first_centroids[camera] = cw[camera];
                    end
                end
                send_pair(19, 0, 0); drain();
                for (integer camera = 0; camera < 2; camera++) begin
                    if (pw[camera] - start_words[camera] != 22
                            || cw[camera] - start_centroids[camera] != 9
                            || packets[camera][start_words[camera]][13:0] != 14'd3
                            || packets[camera][start_words[camera]+1][19:16] != 0
                            || {packets[camera][start_words[camera]+3][31:0],
                                packets[camera][start_words[camera]+2][31:0]} != 64'd395136
                            || packets[camera][start_words[camera]+6][31:0] != 0
                            || packets[camera][start_words[camera]+9][31:0] != 32'd20230963)
                        $fatal(1, "tied candidate changed its accepted winner/score/residual");
                    if (repeat_index != 0) begin
                        for (integer word_index = 0; word_index < 22; word_index++)
                            if (packets[camera][start_words[camera]+word_index]
                                    !== packets[camera][first_words[camera]+word_index])
                                $fatal(1, "tied candidate packet was not deterministic");
                        for (integer child_index = 0; child_index < 9; child_index++)
                            if (centroids[camera][start_centroids[camera]+child_index]
                                    !== centroids[camera][first_centroids[camera]+child_index])
                                $fatal(1, "tied candidate centroid replay changed");
                    end
                end
            end
        end
    endtask

    // Independent wide integer oracle: use the rational N/D itself for the
    // acceptance decision, never the implementation's rounded Q32 quotient.
    task check_normalized_boundary(input logic [37:0] numerator,
                                    input logic [38:0] denominator,
                                    input logic [15:0] mean_limit);
        logic [127:0] scaled_numerator, expected_mean, expected_remainder;
        logic expected_accept;
        integer start_words[2];
        begin
            drain(); defaults();
            if (denominator == 0 || 39'(numerator) >= denominator)
                $fatal(1, "test vector outside unsigned fractional divider range");
            scaled_numerator = 128'(numerator) << 32;
            expected_mean = scaled_numerator / denominator;
            expected_remainder = scaled_numerator % denominator;
            expected_accept = (128'(numerator) << 16) <= 128'(mean_limit) * denominator;
            cfg_mean_residual_limit_q16 = {2{mean_limit}};
            forced_residual_sum = numerator;
            forced_weight_sum = denominator;
            // Forcing N also changes the scorer's public saturated score.
            // Keep diagnostic score fields stable while testing arbitrary
            // full-width N and D through normalization.
            forced_best_score = 0;
            force baseline.frontend.scorer_best_score = forced_best_score;
            force dut.frontend.scorer_best_score = forced_best_score;
            force baseline.frontend.scorer_best_residual_sum = forced_residual_sum;
            force baseline.frontend.scorer_best_weight_sum = forced_weight_sum;
            force dut.frontend.scorer_best_residual_sum = forced_residual_sum;
            force dut.frontend.scorer_best_weight_sum = forced_weight_sum;
            for (integer camera = 0; camera < 2; camera++) start_words[camera] = pw[camera];
            send_pair(job_count / 2, 0, 0); drain();
            if (baseline.frontend.mean_quotient != expected_mean[31:0]
                    || dut.frontend.mean_quotient != expected_mean[31:0]
                    || baseline.frontend.remainder != expected_remainder[38:0]
                    || dut.frontend.remainder != expected_remainder[38:0])
                $fatal(1, "normalized quotient/remainder mismatch N=%0d D=%0d expected=%h rem=%h direct=%h rem=%h banked=%h rem=%h",
                       numerator, denominator, expected_mean[31:0], expected_remainder[38:0],
                       baseline.frontend.mean_quotient, baseline.frontend.remainder,
                       dut.frontend.mean_quotient, dut.frontend.remainder);
            for (integer camera = 0; camera < 2; camera++) begin
                if (pw[camera] - start_words[camera] != (expected_accept ? 22 : 8)
                        || packets[camera][start_words[camera]+6][31:0]
                            != (expected_accept ? 32'd0 : 32'd12))
                    $fatal(1, "normalized exact Q16 boundary N=%0d D=%0d M=%0d expected=%0d",
                           numerator, denominator, mean_limit, expected_accept);
                if (expected_accept && (packets[camera][start_words[camera]+9][31:0]
                        != expected_mean[31:0] || packets[camera][start_words[camera]+8][31:28] != 0))
                    $fatal(1, "normalized packet units/reserved field changed");
            end
            release baseline.frontend.scorer_best_score;
            release dut.frontend.scorer_best_score;
            release baseline.frontend.scorer_best_residual_sum;
            release baseline.frontend.scorer_best_weight_sum;
            release dut.frontend.scorer_best_residual_sum;
            release dut.frontend.scorer_best_weight_sum;
            tick();
        end
    endtask

    logic [63:0] boundary_rng = 64'hf39ae58217c6d40b;
    logic [37:0] random_numerator;
    logic [38:0] random_denominator;
    logic [127:0] random_limit;
    task random_boundary;
        begin
            boundary_rng = boundary_rng ^ (boundary_rng << 13);
            boundary_rng = boundary_rng ^ (boundary_rng >> 7);
            boundary_rng = boundary_rng ^ (boundary_rng << 17);
            random_numerator = boundary_rng[37:0];
            boundary_rng = boundary_rng ^ (boundary_rng << 13);
            boundary_rng = boundary_rng ^ (boundary_rng >> 7);
            boundary_rng = boundary_rng ^ (boundary_rng << 17);
            // N may use every numerator bit; D may use every denominator bit.
            // N<D includes high quotient bits and values arbitrarily near one.
            random_denominator = 39'(random_numerator) + 39'(boundary_rng[37:0]) + 1;
            random_limit = (128'(random_numerator) << 16) / random_denominator;
            check_normalized_boundary(random_numerator, random_denominator, random_limit[15:0]);
            if (random_limit != 0)
                check_normalized_boundary(random_numerator, random_denominator, 16'(random_limit-1));
            if (random_limit != 65535)
                check_normalized_boundary(random_numerator, random_denominator, 16'(random_limit+1));
        end
    endtask

    // Exercise aborts under the implemented bank's fixed one-cycle response.
    // Camera reset does not flush the shared bank response: the abandoned reply
    // must expire before a new owner can sample a configuration value.
    task send_one
        (input integer camera,
         input integer frame);
        begin
            for (integer slot = 0; slot < 9; slot++) begin
                in_valid = 2'(1 << camera);
                in_group_end = slot == 8 ? in_valid : 0;
                in_frame_seq[camera] = 8'(frame);
                in_centroid_valid = 3;
                in_x_q4[camera] = px[camera][slot];
                in_y_q4[camera] = py[camera][slot];
                in_log_mass_q5_3[camera] = mass[camera][slot];
                tick();
                in_valid = 0;
                in_group_end = 0;
                tick();
            end
        end
    endtask

    task abort_job
        (input integer victim,
         input integer phase,
         input logic reset_all);
        begin
            rst = 1;
            camera_reset = 0;
            block_request = 0;
            repeat (3)
                tick();
            defaults();
            rst = 0;
            tick();
            abort_address = phase < 2 ? 4'd7 : (victim == 0 ? 4'd8 : 4'd12);
            block_request = 1;
            send_one(victim, 0);
            send_one(1 - victim, 1);
            while (!(config_read_valid && config_read_address == abort_address))
                tick();
            if (dut.request_camera != 1'(victim))
                $fatal(1, "abort hit wrong camera owner");
            // REQUEST aborts happen before the target handshake. RESPONSE aborts happen
            // after the handshake, with its registered response already asserted.
            if (phase % 2 == 1) begin
                block_request = 0;
                tick();
                if (!config_response_valid)
                    $fatal(1, "bank violated one-cycle response");
            end
            if (reset_all)
                rst = 1;
            else
                camera_reset[victim] = 1;
            tick();
            block_request = 0;
            if (reset_all) begin
                rst = 0;
                tick();
                send_pair(2, 1, 0);
                drain();
                if (job_count != 2 || pr[0] != 22 || pr[1] != 22 || cr[0] != 9 || cr[1] != 9)
                    $fatal(1, "global reset leaked or lost a result");
            end else begin
                camera_reset = 0;
                tick();
                drain();
                // Calibration is captured before commit, so every abort phase
                // cancels the victim before candidate_done can be emitted.
                if (job_count != 1 || pr[victim] != 0 || pr[1 - victim] != 22)
                    $fatal(1, "camera reset damaged the peer or leaked the aborted result");
                send_pair(2, 1, 0);
                drain();
                if (job_count != 3 || pr[victim] != 22 || pr[1 - victim] != 44 || cr[victim] != 9 ||
                    cr[1 - victim] != 18)
                    $fatal(1, "post-abort camera results changed");
            end
            $display("ABORT_PASS victim=%0d phase=%0d global=%0d", victim, phase, reset_all);
        end
    endtask

    integer test_case = 0;
    logic [63 : 0] best, edge_value, normalized_mean_limit;
    logic [63 : 0] values[11];
    initial begin
`include "config_smoke_init.svh"
        defaults();
        if (!$value$plusargs("CASE=%d", test_case))
            test_case = 0;
        repeat (4)
            tick();
        rst = 0;
        tick();
        if (test_case == 0) begin
            // Both FIFOs stay backlogged while owners alternate, with distinct intrinsics.
            for (integer job = 0; job < 6; job++)
                send_pair(job, 1, job % 3 == 1);
            while (job_count < 12 || cb || bb)
                tick();
            drain();
            for (integer camera = 0; camera < 2; camera++) begin
                if (pr[camera] != 104 || cr[camera] != 36)
                    $fatal(1, "wrong owner test counts camera%0d packets%0d centroids%0d", camera,
                           pr[camera], cr[camera]);
            end
        end else if (test_case == 1) begin
            send_pair(0, 0, 0);
            drain();
            best = {packets[0][3][31 : 0], packets[0][2][31 : 0]};
            normalized_mean_limit = {48'b0, baseline.frontend.mean_quotient[31:16]}
                + 64'(baseline.frontend.remainder != 0
                    || baseline.frontend.mean_quotient[15:0] != 0);
            if (best == 0 || pr[0] != 22 || pr[1] != 22)
                $fatal(1, "reference candidate was not accepted");
            begin
                edge_value = normalized_mean_limit;
                values[0] = 0;
                values[1] = 1;
                values[2] = edge_value - 1;
                values[3] = edge_value;
                values[4] = edge_value + 1;
                values[5] = 64'd1023;
                values[6] = 64'd1024;
                values[7] = 64'd1025;
                values[8] = 64'd32768;
                values[9] = 64'd65534;
                values[10] = 64'd65535;
                for (integer j = 0; j < 11; j++)
                    run_boundary(0, values[j]);
            end
            values[0] = 0;
            values[1] = 16;
            values[2] = 17;
            values[3] = 18;
            values[4] = 255;
            for (integer j = 0; j < 5; j++)
                run_boundary(3, values[j]);
            values[0] = 0;
            values[1] = 1;
            values[2] = 16;
            values[3] = 1535;
            values[4] = 1536;
            values[5] = 1537;
            values[6] = 65535;
            for (integer j = 0; j < 7; j++)
                run_boundary(4, values[j]);
            values[0] = 0;
            values[1] = 1;
            values[2] = 64'h3333;
            values[3] = 64'h1ffff;
            values[4] = 64'h20000;
            values[5] = 64'h80000000;
            values[6] = 64'hffffffff;
            for (integer j = 0; j < 7; j++)
                run_boundary(5, values[j]);
            values[0] = 0;
            values[1] = 1;
            values[2] = 64'h222222;
            values[3] = 64'h3ffffff;
            values[4] = 64'h4000000;
            values[5] = 64'hffffffff;
            for (integer field = 6; field < 8; field++)
                for (integer j = 0; j < 6; j++)
                    run_boundary(field, values[j]);
            values[0] = 0;
            values[1] = 64'h7ffffff;
            values[2] = 64'h8000000;
            values[3] = 64'hf8000000;
            values[4] = 64'hf7ffffff;
            values[5] = 64'hffffffff;
            for (integer field = 8; field < 10; field++)
                for (integer j = 0; j < 6; j++)
                    run_boundary(field, values[j]);
            if (job_count != 110)
                $fatal(1, "wrong boundary job count %0d", job_count);
        end else if (test_case == 2) begin
            fixed_response = 1;
            for (integer victim = 0; victim < 2; victim++)
                for (integer phase = 0; phase < 4; phase++)
                    for (integer global_abort = 0; global_abort < 2; global_abort++)
                        abort_job(victim, phase, 1'(global_abort));
        end else if (test_case == 3) begin
            run_tied_candidate();
        end else if (test_case == 4) begin
            check_normalized_boundary(0, 1, 0);
            check_normalized_boundary(0, 39'h7fffffffff, 0);
            check_normalized_boundary(1, 39'h7fffffffff, 0);
            check_normalized_boundary(1, 39'h7fffffffff, 1);
            // Default 1/64 equality, immediately lower/higher Q0.16 limits.
            check_normalized_boundary(1, 64, 1023);
            check_normalized_boundary(1, 64, 1024);
            check_normalized_boundary(1, 64, 1025);
            check_normalized_boundary(3, 65536, 2);
            check_normalized_boundary(3, 65536, 3);
            check_normalized_boundary(3, 65536, 4);
            // Low quotient bits and division remainders independently prevent
            // accepting values rounded down onto the threshold.
            check_normalized_boundary(1, 3, 21845);
            check_normalized_boundary(1, 3, 21846);
            check_normalized_boundary(1, 5, 13107);
            check_normalized_boundary(1, 5, 13108);
            // floor(Q32)=1024<<16 exactly, yet the residual remainder is nonzero.
            check_normalized_boundary(38'd4294967296, 39'd274877906943, 1024);
            check_normalized_boundary(38'd4294967296, 39'd274877906943, 1025);
            check_normalized_boundary(38'd137438953472, 39'd412316860416, 21845);
            check_normalized_boundary(38'd137438953472, 39'd412316860416, 21846);
            check_normalized_boundary(38'd17179869184, 39'd343597383680, 3276);
            check_normalized_boundary(38'd17179869184, 39'd343597383680, 3277);
            // All quotient bits, numerator bits and denominator bits matter.
            check_normalized_boundary(38'h3fffffffff, 39'h7fffffffff, 32767);
            check_normalized_boundary(38'h3fffffffff, 39'h7fffffffff, 32768);
            check_normalized_boundary(38'h3fffffffff, 39'h4000000000, 65535);
            check_normalized_boundary(3, 4, 49151);
            check_normalized_boundary(3, 4, 49152);
            check_normalized_boundary(3, 4, 65535);
            check_normalized_boundary(65535, 65536, 65535);
            check_normalized_boundary(65536, 65537, 65535);
        end else if (test_case == 5) begin
            for (integer vector = 0; vector < 32; vector++)
                random_boundary();
        end else
            $fatal(1, "unknown test case %0d", test_case);
        if (addresses_seen !== 16'hff83)
            $fatal(1, "not all camera config addresses exercised %h", addresses_seen);
        $display(
            "PASS config fetch case %0d jobs=%0d requests=%0d addresses=%h packet_words=%0d centroid_words=%0d max_cycles=%0d",
            test_case, job_count, req_count, addresses_seen, pr[0] + pr[1], cr[0] + cr[1],
            max_cycles);
        $finish;
    end
endmodule
