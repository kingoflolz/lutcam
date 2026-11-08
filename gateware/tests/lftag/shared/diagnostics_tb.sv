// Exercise real FIFO diagnostics and shared-owner context without a sensor.
module diagnostics_tb;
    logic clk = 0;
    always #5 clk = !clk;
    logic global_reset = 1, processing_reset = 1, clear_diagnostics = 0;
    logic in_valid = 0, in_group_end = 0;
    logic [1:0] in_scale = 0;
    logic [7:0] in_frame_seq = 0;
    logic head_pop = 0;
    logic [10:0] diagnostic_context = {1'b0,1'b0,1'b1,1'b0,1'b1,6'd7};
    wire [1:0][5:0] fifo_level;
    wire [1:0] fifo_drop, protocol_error;
    wire [1:0][31:0] status_word, protocol_word, drop_word;

    // Index zero preserves the standalone reset contract; index one retains
    // new diagnostic words through a processing-only reset.
    for (genvar retained = 0; retained < 2; retained++) begin : direct
        LfTagCandidateFifoRamMetadata #(.RETAIN_DIAGNOSTICS(retained == 1)) fifo (
            .clk, .rst(global_reset || processing_reset),
            .in_valid, .in_group_end, .in_x_q4(15'd123), .in_y_q4(14'd456),
            .in_log_mass_q5_3(8'd64), .in_centroid_valid(1'b1),
            .in_scale, .in_frame_seq,
            .head_valid(), .head_scale(), .head_frame_seq(),
            .head_protocol(), .head_mass_spread(), .head_pop,
            .desc_read_valid(1'b0), .desc_read_index(4'b0),
            .desc_read_data_valid(), .desc_read_data(), .desc_read_centroid_valid(),
            .fifo_level(fifo_level[retained]), .fifo_full(), .clear_diagnostics,
            .fifo_drop_sticky(fifo_drop[retained]), .protocol_error_sticky(protocol_error[retained]),
            .diagnostic_reset(global_reset), .diagnostic_context,
            .debug_status(status_word[retained]), .debug_protocol(protocol_word[retained]),
            .debug_drop(drop_word[retained])
        );
    end

    logic [1:0] camera_reset = 0, shared_valid = 0, shared_end = 0;
    logic [1:0][14:0] shared_x = 0;
    logic [1:0][13:0] shared_y = 0;
    logic [1:0][7:0] shared_mass = 0, shared_frame = 0;
    logic [1:0][1:0] shared_scale = 0;
    logic [1:0] shared_clear = 0, shared_ready = 3;
    wire [1:0] shared_out_valid, shared_out_last, shared_busy, shared_drop, shared_protocol;
    wire [1:0][5:0] shared_level;
    wire [1:0][31:0] shared_status, shared_protocol_word, shared_drop_word;
    SharedLfTagDecoder shared (
        .config_read_valid(), .config_read_address(), .config_read_ready(1'b0),
        .config_response_valid(1'b0), .config_read_data(32'b0),
        .clk, .rst(global_reset), .camera_reset,
        .in_valid(shared_valid), .in_group_end(shared_end),
        .in_x_q4(shared_x), .in_y_q4(shared_y), .in_log_mass_q5_3(shared_mass),
        .in_centroid_valid(2'b11), .in_scale(shared_scale),
        .in_frame_seq(shared_frame),
        .cfg_inv_fx_q2_30('0), .cfg_inv_fy_q2_30('0), .cfg_cx_q16_16('0), .cfg_cy_q16_16('0),
        .cfg_printed_width_q16_16('0), .cfg_mean_residual_limit_q16('0),
        .cfg_max_mass_spread('0), .cfg_min_baseline_q4('0),
        .centroid_valid(), .centroid_ready(2'b11), .centroid_data(), .centroid_last(),
        .out_valid(shared_out_valid), .out_ready(shared_ready), .out_data(),
        .out_last(shared_out_last), .busy(shared_busy), .fifo_level(shared_level),
        .candidate_start(), .candidate_done(), .candidate_cycles(),
        .clear_diagnostics(shared_clear), .fifo_drop_sticky(shared_drop),
        .protocol_error_sticky(shared_protocol), .debug_status(shared_status),
        .debug_protocol(shared_protocol_word), .debug_drop(shared_drop_word)
    );

    // The diagnostic word layout is fixed; removed overflow fields are reserved.
    always @(negedge clk) begin
        if (!global_reset) begin
            for (integer camera = 0; camera < 2; camera++) begin
                if (protocol_word[camera][4] !== 1'b0 ||
                    protocol_word[camera][31:30] !== 2'b0 ||
                    drop_word[camera][28] !== 1'b0)
                    $fatal(1, "direct FIFO reserved diagnostic bits are nonzero");
                if (shared_protocol_word[camera][4] !== 1'b0 ||
                    shared_protocol_word[camera][31:30] !== 2'b0 ||
                    shared_drop_word[camera][28] !== 1'b0)
                    $fatal(1, "shared FIFO reserved diagnostic bits are nonzero");
            end
        end
    end

    task automatic tick(input integer count = 1);
        repeat(count) @(negedge clk);
    endtask

    task automatic check(input logic [31:0] actual, input logic [31:0] expected,
                         input string message);
        if (actual !== expected) $fatal(1, "%s: got %08h expected %08h", message, actual, expected);
    endtask

    function automatic logic [31:0] protocol_context(
        input logic [4:0] reasons, input logic [3:0] staged, input logic ending,
        input logic [1:0] first_scale, input logic [1:0] current_scale,
        input logic [7:0] first_frame, input logic [7:0] current_frame);
        return {2'b0, current_frame, first_frame,
                current_scale, first_scale, ending, staged, reasons};
    endfunction

    function automatic logic [31:0] denied_context(
        input logic [5:0] own_level, input logic [10:0] context_bits,
        input logic [1:0] scale, input logic [7:0] frame, input logic ending);
        return {2'b0, ending, 1'b0, frame, scale,
                context_bits[10:6], 1'b0, context_bits[5:0], own_level};
    endfunction

    task automatic tuple(input logic [1:0] scale, input logic [7:0] frame,
                         input logic ending);
        in_valid = 1; in_scale = scale; in_frame_seq = frame;
        in_group_end = ending;
        tick(); in_valid = 0; in_group_end = 0;
    endtask

    task automatic group(input logic [7:0] frame);
        for (integer index = 0; index < 9; index++) tuple(1, frame, index == 8);
    endtask

    task automatic clean_direct;
        processing_reset = 1; clear_diagnostics = 1;
        in_valid = 0; in_group_end = 0; head_pop = 0;
        tick(); processing_reset = 0; clear_diagnostics = 0; tick();
        check(status_word[1], 0, "explicit clear did not rearm retained status");
        check(protocol_word[1], 0, "explicit clear did not erase protocol context");
        check(drop_word[1], 0, "explicit clear did not erase drop context");
    endtask

    task automatic shared_tuple(input integer camera, input integer index,
                                input logic [1:0] scale, input logic [7:0] frame);
        shared_valid[camera] = 1; shared_end[camera] = index == 8;
        shared_x[camera] = 15'(index * 16); shared_y[camera] = 14'(index * 8);
        shared_mass[camera] = 8'(64 + index);
        shared_scale[camera] = scale; shared_frame[camera] = frame;
        tick(); shared_valid[camera] = 0; shared_end[camera] = 0;
    endtask

    task automatic shared_group(input integer camera, input logic [7:0] frame);
        for (integer index = 0; index < 9; index++) shared_tuple(camera, index, 0, frame);
    endtask

    integer which_case;
    logic [31:0] saved_status, saved_protocol, saved_drop, expected_protocol;
    initial begin
        if (!$value$plusargs("CASE=%d", which_case)) which_case = 0;
        tick(4); global_reset = 0; processing_reset = 0; tick();
        case (which_case)
            0: begin
                tuple(1, 'h12, 0);
                tuple(2, 'h34, 0);
                expected_protocol = protocol_context('h0c, 1, 0, 1, 2, 'h12, 'h34);
                check(protocol_word[1], expected_protocol, "first malformed tuple");
                check(status_word[1], 'h40, "capture must precede group-end flag");
                check(protocol_error[1], 0, "raw error must wait for group end");
                processing_reset = 1; tick();
                check(status_word[1], 'h40, "partial protocol evidence lost on processing reset");
                check(protocol_word[1], expected_protocol, "first context lost on processing reset");
                check(status_word[0], 0, "default FIFO diagnostics did not reset");
                check(protocol_word[0], 0, "default FIFO context did not reset");
                // Reset suppresses captures even if malformed ingress remains asserted.
                tuple(3, 'hff, 1);
                check(status_word[1], 'h40, "processing reset captured an event");
                processing_reset = 0;
                tuple(3, 'h99, 1);
                check(protocol_word[1], expected_protocol, "later error replaced first context");
                check(status_word[1], 'h141, "group-end protocol or high-water missing");
                saved_status = status_word[1]; processing_reset = 1; tick();
                check(status_word[1], saved_status, "completed protocol evidence was not retained");
                check(protocol_error[1], 0, "raw sticky must clear on processing reset");
                clear_diagnostics = 1; tick(); clear_diagnostics = 0;
                check(status_word[1], 0, "clear during processing reset failed");
                check(protocol_word[1], 0, "clear failed to erase first context");
            end
            1: begin
                for (integer reason = 0; reason < 5; reason++) begin
                    clean_direct();
                    if (reason == 1) begin
                        tuple(3, 'ha5, 0);
                        expected_protocol = protocol_context(2, 0, 0, 3, 3, 'ha5, 'ha5);
                    end else begin
                        tuple(1, 'h12, 0);
                        // The final case changes both scale and frame; bit 4
                        // remains reserved rather than naming a fifth cause.
                        tuple(reason == 2 || reason == 4 ? 2 : 1,
                              reason == 3 || reason == 4 ? 'h34 : 'h12, reason == 0);
                        expected_protocol = protocol_context(reason == 4 ? 5'h0c : 5'(1 << reason),
                            1, reason == 0, 1, reason == 2 || reason == 4 ? 2 : 1,
                            'h12, reason == 3 || reason == 4 ? 'h34 : 'h12);
                    end
                    check(protocol_word[1], expected_protocol, "protocol reason or tuple metadata mismatch");
                    if (!status_word[1][6]) $fatal(1, "missing protocol context validity");
                    tuple(3, 'hff, 1);
                    check(protocol_word[1], expected_protocol, "subsequent reason overwrote first cause");
                end
                clean_direct();
                for (integer index = 0; index < 9; index++) tuple(1, 'h44, 0);
                check(status_word[1][6], 0, "nine unterminated tuples are not yet overlength");
                tuple(1, 'h44, 0);
                check(protocol_word[1], protocol_context(1, 9, 0, 1, 1, 'h44, 'h44),
                      "overlength first-error position");
            end
            2: begin
                for (integer index = 0; index < 32; index++) group(8'(index));
                check(fifo_level[1], 32, "queue was not full");
                check(status_word[1], 32, "high-water must include latest commit immediately");
                tuple(2, 'hd3, 0);
                saved_drop = denied_context(32, diagnostic_context, 2, 'hd3, 0);
                check(drop_word[1], saved_drop, "denied admission context");
                check(status_word[1], 'ha0, "drop admission must precede eventual drop flag");
                check(fifo_drop[1], 0, "raw drop must wait for group end");
                head_pop = 1; tick(32); head_pop = 0;
                check(fifo_level[1], 0, "queue did not drain");
                diagnostic_context = '1;
                for (integer index = 1; index < 9; index++) tuple(2, 'hd3, index == 8);
                check(status_word[1], 'h2a0, "drop at empty queue lost admission history");
                check(drop_word[1], saved_drop, "group-end context replaced admission context");
                check(fifo_drop[1], 1, "completed denied group did not drop");
                check(protocol_error[1], 0, "well-formed denied group created protocol error");
                processing_reset = 1; tick();
                check(status_word[1], 'h2a0, "drop/high-water lost on processing reset");
                check(drop_word[1], saved_drop, "drop context lost on processing reset");
                check(status_word[0], 0, "default drop diagnostics did not reset");
                check(fifo_drop[1], 0, "raw raw drop must reset independently");
                processing_reset = 0;
                for (integer index = 0; index < 32; index++) group(8'(index + 32));
                tuple(0, 'h55, 0);
                check(drop_word[1], saved_drop, "later denied group replaced first context");
                global_reset = 1; tick();
                check(status_word[1], 0, "global diagnostic reset failed");
                check(drop_word[1], 0, "global reset retained stale drop context");
            end
            3: begin
                for (integer old_valid = 0; old_valid < 2; old_valid++) begin
                    clean_direct();
                    if (old_valid) begin
                        tuple(1, 'h11, 1);
                        processing_reset = 1; tick(); processing_reset = 0;
                    end
                    clear_diagnostics = 1; tuple(3, 'h99, 1); clear_diagnostics = 0;
                    check(status_word[1], 0, "clear must win simultaneous new diagnostic event");
                    check(protocol_word[1], 0, "clear retained/replaced old context on same edge");
                    check(protocol_error[1], 1, "raw sticky event behavior changed");
                    tuple(2, 'h42, 1);
                    check(protocol_word[1], protocol_context(1, 0, 1, 2, 2, 'h42, 'h42),
                          "first error did not rearm after clear");
                end
                clean_direct();
                tuple(0, 'h12, 0);
                clear_diagnostics = 1; tuple(0, 'h34, 0); clear_diagnostics = 0;
                for (integer index = 2; index < 9; index++) tuple(0, 'h12, index == 8);
                check(status_word[1], 'h101, "partial group should retain protocol state, not cleared context");
                check(protocol_word[1], 0, "cleared first error must not be captured retroactively");
                clean_direct();
                for (integer index = 0; index < 32; index++) group(8'(index));
                tuple(1, 'h77, 0);
                clear_diagnostics = 1; tick(); clear_diagnostics = 0;
                head_pop = 1; tick(32); head_pop = 0;
                for (integer index = 1; index < 9; index++) tuple(1, 'h77, index == 8);
                check(status_word[1][9:6], 8, "drop seen may survive with deliberately cleared first context");
                check(drop_word[1], 0, "clear during rejected group must not retroactively recapture admission");
                global_reset = 1; tuple(3, 'hff, 1);
                check(status_word[1], 0, "global reset must suppress simultaneous malformed ingress");
            end
            4: begin
                shared_ready[0] = 0;
                shared_group(0, 'h11);
                begin : wait_stalled_owner
                    bit found = 0;
                    for (integer cycle = 0; cycle < 100; cycle++) begin
                        tick(); if (shared_out_valid[0]) begin found = 1; break; end
                    end
                    if (!found) $fatal(1, "missing real shared-core stalled output");
                end
                // One peer candidate can now enter the second job slot while
                // the first packet is stalled; 33 groups fill its job plus FIFO.
                tick();
                check(shared_level[0], 0, "owner should already have popped its candidate");
                for (integer index = 0; index < 33; index++) shared_group(1, 8'(index + 32));
                shared_tuple(1, 0, 2, 'he1);
                saved_drop = denied_context(32, {1'b0,1'b0,1'b1,1'b0,1'b1,6'd0}, 2, 'he1, 0);
                check(shared_drop_word[1], saved_drop, "shared stall/owner/peer context mismatch");
                check(shared_status[1], 'ha0, "shared admission flags mismatch");
                for (integer index = 1; index < 9; index++) shared_tuple(1, index, 2, 'he1);
                check(shared_status[1], 'h2a0, "shared completed-drop flag missing");
                check(shared_protocol[1], 0, "valid shared groups created protocol error");
                check(shared_drop[1], 1, "shared raw drop missing");
                camera_reset[1] = 1; tick();
                check(shared_level[1], 0, "camera processing reset did not flush queue");
                check(shared_status[1], 'h2a0, "camera reset erased retained drop status");
                check(shared_drop_word[1], saved_drop, "camera reset erased first owner context");
                check(shared_drop[1], 0, "camera reset must clear raw raw drop");
                check(shared_out_valid[0], 1, "peer reset interrupted stalled owner");
                shared_clear[1] = 1; tick(); shared_clear[1] = 0;
                check(shared_status[1], 0, "shared per-camera explicit clear failed");
                check(shared_drop_word[1], 0, "shared per-camera clear retained context");
                check(shared_status[0], 1, "peer clear erased owner high-water");
                camera_reset[1] = 0; global_reset = 1; tick();
                check(shared_status[0] | shared_status[1], 0, "global shared reset failed");
                check(shared_out_valid, 0, "global reset failed to cancel output");
            end
            5: begin
                for (integer index = 0; index < 32; index++) group(8'(index));
                head_pop = 1; tuple(2, 'h81, 0); head_pop = 0;
                for (integer index = 1; index < 9; index++) tuple(2, 'h81, index == 8);
                check(fifo_level[1], 32, "full plus first-tuple pop failed to admit new group");
                check(status_word[1], 32, "full plus pop generated false first-error evidence");
                check(drop_word[1], 0, "full plus pop captured denied admission");
                check(fifo_drop[1] | protocol_error[1], 0, "valid full-boundary replacement failed");
                processing_reset = 1; tick();
                check(status_word[1], 32, "one-cycle occupancy peak not retained");
            end
            default: $fatal(1, "unknown diagnostics case");
        endcase
        $display("PASS retained diagnostics case %0d", which_case);
        $finish;
    end
    initial begin #2000000; $fatal(1, "diagnostics simulation timeout"); end
endmodule
