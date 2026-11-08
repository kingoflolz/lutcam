`timescale 1ns/1ps
`default_nettype none

module camera_control_tb #(
    parameter bit PAIRED_MODE = 1'b0,
    parameter bit USE_DSP_COUNTERS = 1'b0
);
    localparam integer LATENCY = 1;
    logic clk = 0;
    always #5 clk = !clk;
    logic rst = 1;
    logic [19:0] pixel_data = 0;
    logic pixel_valid = 0, pixel_sof = 0, pixel_eol = 0, pixel_eof = 0;
    logic receiver_fault = 0, run_request = 0, capture_permit = 1;
    logic clear_diagnostics = 0;
    logic pair_abort = 0;
    logic flip_sof = 0, flip_eol = 0, flip_eof = 0;
    logic [31:0] capture_frame_id = 0;
    wire active, idle, fault_sticky;
    wire [31:0] frame_id, frames_started, frames_completed;
    wire [15:0] ingress_high_water;
    logic wb_cyc_i = 0, wb_stb_i = 0, wb_we_i = 0;
    logic [3:0] wb_sel_i = 4'hf;
    logic [15:0] wb_adr_i = 0;
    logic [31:0] wb_dat_i = 0;
    wire [31:0] wb_dat_o;
    wire wb_ack_o, wb_err_o;
    wire lwc_protocol_error, lftag_fifo_drop_sticky, lftag_protocol_error_sticky;
    wire [2:0] retained_causes = {
        lftag_protocol_error_sticky, lftag_fifo_drop_sticky, lwc_protocol_error
    };

    CameraPipeline #(
        .PAIRED_MODE(PAIRED_MODE), .USE_DSP_COUNTERS(USE_DSP_COUNTERS)
    ) dut (
        .clk, .rst, .pixel_data, .pixel_valid, .pixel_sof, .pixel_eol, .pixel_eof,
        .receiver_fault, .run_request, .capture_permit, .clear_diagnostics,
        .pair_abort,
        .capture_frame_id,
        .active, .idle, .fault_sticky, .frame_id, .frames_started, .frames_completed,
        .ingress_high_water, .wb_cyc_i, .wb_stb_i, .wb_we_i, .wb_sel_i,
        .wb_adr_i, .wb_dat_i, .wb_dat_o, .wb_ack_o, .wb_err_o,
        .hist_dump_req(1'b0), .hist_dump_ready(), .hist_out_valid(),
        .hist_out_ready(1'b1), .hist_out_data(),
        .inner_fv_valid(), .inner_fv_data(), .hier_inner_end(),
        .lwc_out_sof(), .lwc_out_valid(), .lwc_out_padding(),
        .lwc_out_data(), .lwc_protocol_error,
        .lftag_cfg_inv_fx_q2_30(32'd0), .lftag_cfg_inv_fy_q2_30(32'd0),
        .lftag_cfg_cx_q16_16(32'd0), .lftag_cfg_cy_q16_16(32'd0),
        .lftag_cfg_printed_width_q16_16(32'd0),
        .lftag_cfg_mean_residual_limit_q16(16'd0),
        .lftag_cfg_max_mass_spread(8'd0), .lftag_cfg_min_baseline_q4(16'd0),
        .lftag_result_ready(1'b1),
        .lftag_clear_diagnostics(1'b0), .lftag_result_valid(),
        .lftag_result_data(), .lftag_result_last(), .lftag_busy(),
        .lftag_fifo_level(), .lftag_candidate_start(), .lftag_candidate_done(),
        .lftag_candidate_cycles(), .lftag_fifo_drop_sticky,
        .lftag_protocol_error_sticky
    );

    function automatic [9:0] sample0(input integer index);
        sample0 = 10'((index * 29 + 3) ^ (index >> 2));
    endfunction
    function automatic [9:0] sample1(input integer index);
        sample1 = 10'((index * 47 + 101) ^ (index >> 3));
    endfunction

    // eof_mode: zero follows the full-size sensor format; one forces EOF;
    // two suppresses EOF. State assertions observe registered values.
    task automatic step(input logic send_pixel, input integer index,
                        input integer eof_mode = 0);
        logic was_active;
        logic [31:0] owned_id;
        @(negedge clk);
        pixel_valid = send_pixel;
        pixel_data = {sample1(index), sample0(index)};
        pixel_sof = send_pixel && ((index == 0) ^ flip_sof);
        pixel_eol = send_pixel && ((index % 640 == 639) ^ flip_eol);
        pixel_eof = send_pixel && (eof_mode == 1
            || (eof_mode == 0 && index == 511999));
        pixel_eof = pixel_eof ^ (send_pixel && flip_eof);
        was_active = active;
        owned_id = frame_id;
        @(posedge clk);
        if (case_number == 6 && !rst && dut.local_fault)
            $fatal(1, "candidate rejection raised a sampled local capture fault");
        #1;
        if (!rst && was_active && frame_id != owned_id)
            $fatal(1, "frame ownership changed before the old frame drained");
        if (idle && (active || fault_sticky))
            $fatal(1, "idle asserted while active or faulted");
        if (ingress_high_water != 0)
            $fatal(1, "unbuffered camera path reported a nonzero FIFO high-water mark");
    endtask

    task automatic wait_idle;
        integer clocks;
        clocks = 0;
        while (!idle) begin
            step(0, 0);
            clocks = clocks + 1;
            if (fault_sticky || clocks > 2048)
                $fatal(1, "processing initialization failed to reach idle");
        end
    endtask

    task automatic bus_idle;
        wb_cyc_i = 0;
        wb_stb_i = 0;
        wb_we_i = 0;
    endtask

    task automatic reset_design;
        rst = 1;
        run_request = 0;
        receiver_fault = 0;
        pair_abort = 0;
        flip_sof = 0; flip_eol = 0; flip_eof = 0;
        wb_sel_i = 4'hf;
        clear_diagnostics = 0;
        capture_permit = 1;
        bus_idle();
        repeat (3) step(0, 0);
        if (active || fault_sticky || frames_started || frames_completed || frame_id)
            $fatal(1, "reset did not clear capture state");
        rst = 0;
        repeat (3) step(0, 0);
        wait_idle();
    endtask

    task automatic write_isp(input logic [15:0] address,
                             input logic [31:0] value);
        wb_cyc_i = 1;
        wb_stb_i = 1;
        wb_we_i = 1;
        wb_adr_i = address;
        wb_dat_i = value;
        step(0, 0);
        if (!wb_ack_o || wb_err_o)
            $fatal(1, "ISP write to %h was rejected", address);
        bus_idle();
        step(0, 0);
    endtask

    task automatic start_isp;
        write_isp(16'h0004, 32'd1); // bypass LUT and histogram processing
        repeat (5) step(0, 0);
        wait_idle();
    endtask

    task automatic send_frame(input logic exercise_control,
                              input logic suppress_final_eof = 0,
                              input logic exercise_pair_control = 0,
                              input logic inject_candidate_protocol = 0);
        integer row, column, index;
        for (row = 0; row < 800; row = row + 1) begin
            for (column = 0; column < 640; column = column + 1) begin
                index = row * 640 + column;
                if (inject_candidate_protocol && index == 64)
                    dut.processing.inject_protocol = 1;
                if (inject_candidate_protocol && index == 65)
                    dut.processing.inject_protocol = 0;
                if (inject_candidate_protocol && index == 128)
                    clear_diagnostics = 1;
                if (inject_candidate_protocol && index == 129)
                    clear_diagnostics = 0;
                if (exercise_control && index == 1000)
                    run_request = 0;
                if (exercise_pair_control && index == 1001)
                    capture_frame_id = 32'hdead_beef;
                if (exercise_pair_control && index == 1002)
                    clear_diagnostics = 1;
                if (exercise_pair_control && index == 1003)
                    clear_diagnostics = 0;
                if (exercise_control && index == 2000) begin
                    wb_cyc_i = 1;
                    wb_stb_i = 1;
                    wb_we_i = 1;
                    wb_adr_i = 16'h0004;
                    wb_dat_i = 0;
                end
                if (exercise_control && index == 2002) begin
                    wb_cyc_i = 1;
                    wb_stb_i = 1;
                    wb_we_i = 0;
                    wb_adr_i = 16'h0004;
                end
                step(1, index, suppress_final_eof ? 2 : 0);
                if (inject_candidate_protocol && index >= 66 &&
                    (!lftag_protocol_error_sticky || fault_sticky))
                    $fatal(1, "candidate rejection at %0d diagnostic=%b local=%b capture=%b raw=%b reset=%b",
                        index, lftag_protocol_error_sticky, dut.local_fault, fault_sticky,
                        dut.processing.lftag_protocol_error_sticky, dut.processing_reset);
                if (exercise_control && index == 2000) begin
                    if (!wb_err_o || wb_ack_o)
                        $fatal(1, "midframe ISP stop was not rejected");
                    bus_idle();
                end
                if (exercise_control && index == 2002) begin
                    if (!wb_ack_o || wb_err_o || wb_dat_o != 1)
                        $fatal(1, "blocked midframe write stopped or reconfigured the ISP");
                    bus_idle();
                end
                if (!suppress_final_eof || index != 511999) begin
                    if (!active || fault_sticky)
                        $fatal(1, "capture stopped before frame end at pair %0d", index);
                end
            end
            // The final blanking interval belongs to the drain checks below.
            if (row != 799)
                repeat (256) step(0, 0);
        end
    endtask

    task automatic drain_frame(input integer expected_frame,
                               input logic [31:0] expected_id);
        integer clocks;
        if (!active || frame_id != expected_id
                || frames_completed != 32'(expected_frame))
            $fatal(1, "source EOF released frame ownership before processing drain");
        for (clocks = 0; clocks < LATENCY + 32; clocks = clocks + 1) begin
            step(0, 0);
            if (!active || frame_id != expected_id
                    || frames_completed != 32'(expected_frame) || fault_sticky)
                $fatal(1, "frame ownership released inside processing drain at clock %0d", clocks);
        end
        if (dut.processing.received_pairs != 512000
                || dut.processing.frames_retired != 1)
            $fatal(1, "processing did not consume the complete frame before retirement");
        repeat (20) step(0, 0);
        wait_idle();
        if (active || !idle || fault_sticky || frame_id != expected_id
                || frames_started != 32'(expected_frame + 1)
                || frames_completed != 32'(expected_frame + 1)
                || ingress_high_water != 0)
            $fatal(1, "frame %0d did not drain cleanly", expected_frame);
    endtask

    integer case_number;
    integer index;
    logic [2:0] expected_cause;
    initial begin
        if (!$value$plusargs("CASE=%d", case_number)) case_number = 0;
        reset_design();
        start_isp();
        case (case_number)
            0: begin
                // Disarmed SOF and an armed partial frame must both be ignored.
                step(1, 0);
                run_request = 1;
                for (index = 17; index < 49; index = index + 1)
                    step(1, index);
                repeat (LATENCY + 2) step(0, 0);
                if (active || frames_started || dut.processing.frames_seen)
                    $fatal(1, "capture began before an armed SOF");
                send_frame(1);
                drain_frame(0, 0);
                // The dropped request remains disarmed after the old frame.
                step(1, 0);
                repeat (LATENCY + 2) step(0, 0);
                if (active || frames_started != 1)
                    $fatal(1, "capture restarted while disarmed");
                run_request = 1;
                send_frame(0);
                drain_frame(1, 1);
                // A clean disarmed clear must preserve ISP programming and
                // keep the reserved ingress high-water register at zero.
                run_request = 0;
                clear_diagnostics = 1;
                step(0, 0);
                clear_diagnostics = 0;
                repeat (5) begin
                    step(0, 0);
                    if (ingress_high_water != 0 || fault_sticky || retained_causes != 0)
                        $fatal(1, "clean diagnostic clear did not remain cleared");
                end
                wb_cyc_i = 1;
                wb_stb_i = 1;
                wb_we_i = 0;
                wb_adr_i = 16'h0004;
                step(0, 0);
                if (!wb_ack_o || wb_err_o || wb_dat_o != 1)
                    $fatal(1, "clean diagnostic clear reset retained ISP configuration");
                bus_idle();
            end
            1: begin
                run_request = 1;
                for (index = 0; index < 64; index = index + 1) step(1, index);
                receiver_fault = 1;
                step(0, 0);
                if (!fault_sticky || active || idle)
                    $fatal(1, "receiver fault did not stop and latch capture");
                receiver_fault = 0;
                clear_diagnostics = 1;
                repeat (5) step(0, 0);
                if (!fault_sticky || idle)
                    $fatal(1, "armed clear incorrectly released the receiver fault");
                clear_diagnostics = 0;
                run_request = 0;
                repeat (3) step(0, 0);
                if (!fault_sticky)
                    $fatal(1, "disarm alone cleared the fault");
                clear_diagnostics = 1;
                step(0, 0);
                clear_diagnostics = 0;
                repeat (3) step(0, 0);
                wait_idle();
                if (fault_sticky || !idle || frames_started != 1 || frames_completed != 0
                    || ingress_high_water != 0)
                    $fatal(1, "disarmed clear did not recover capture or changed frame counters");
                start_isp();
                run_request = 1;
                for (index = 0; index < 1280; index = index + 1) begin
                    if (index == 640) repeat (256) step(0, 0);
                    step(1, index);
                end
                if (fault_sticky || !active || frame_id != 1 || frames_started != 2
                        || dut.processing.received_pairs < 32)
                    $fatal(1, "capture did not restart cleanly after fault recovery");
            end
            2: begin
                run_request = 1;
                for (index = 0; index < 7; index = index + 1) step(1, index);
                step(1, 7, 1);
                if (!fault_sticky || active || frames_completed)
                    $fatal(1, "early EOF marker was accepted");
            end
            3: begin
                run_request = 1;
                send_frame(0, 1);
                if (!fault_sticky || active || frames_completed)
                    $fatal(1, "missing EOF marker was accepted");
            end
            4, 5: begin
                run_request = 1;
                for (index = 0; index < 64; index = index + 1) step(1, index);
                if (!active || dut.processing.received_pairs == 0)
                    $fatal(1, "fault test did not first exercise active processing");
                expected_cause = 3'b001 << (case_number - 4);
                dut.processing.inject_lwc = case_number == 4;
                dut.processing.inject_drop = case_number == 5;
                // Registered child faults take one clock to propagate; keep
                // supplying the sensor cadence until capture sees the fault.
                step(1, 64);
                dut.processing.inject_lwc = 0;
                dut.processing.inject_drop = 0;
                repeat (5) step(0, 0);
                if (!fault_sticky || active || idle || retained_causes !== expected_cause)
                    $fatal(1, "core fault cause was lost: got %b, expected %b",
                           retained_causes, expected_cause);
                if (dut.processing.lwc_protocol_error
                    || dut.processing.lftag_fifo_drop_sticky
                    || dut.processing.lftag_protocol_error_sticky)
                    $fatal(1, "child reset did not clear the injected raw fault flags");
                if (ingress_high_water != 0)
                    $fatal(1, "unbuffered path reported an ingress peak after fault");

                clear_diagnostics = 1;
                repeat (5) begin
                    step(0, 0);
                    if (!fault_sticky || retained_causes !== expected_cause)
                        $fatal(1, "armed clear erased retained core fault diagnostics");
                end
                clear_diagnostics = 0;
                run_request = 0;
                repeat (3) begin
                    step(0, 0);
                    if (!fault_sticky || retained_causes !== expected_cause)
                        $fatal(1, "disarm alone erased retained core fault diagnostics");
                end
                clear_diagnostics = 1;
                step(0, 0);
                clear_diagnostics = 0;
                wait_idle();
                repeat (5) begin
                    step(0, 0);
                    if (fault_sticky || !idle || retained_causes != 0 || ingress_high_water != 0)
                        $fatal(1, "disarmed clear did not clear retained core diagnostics");
                end
                if (frames_started != 1 || frames_completed != 0)
                    $fatal(1, "fault diagnostic clear changed frame accounting");
            end
            6: begin
                // A rejected candidate must not truncate its image or block
                // later frames. The raw child flag resets at retirement,
                // while the public diagnostic survives until disarmed CLEAR.
                run_request = 1;
                capture_frame_id = 0;
                send_frame(0, 0, 0, 1);
                drain_frame(0, 0);
                if (retained_causes != 3'b100 || dut.local_fault ||
                    dut.processing.lftag_protocol_error_sticky)
                    $fatal(1, "candidate diagnostic was not retained across frame retirement");
                capture_frame_id = 1;
                send_frame(0);
                drain_frame(1, 1);
                if (retained_causes != 3'b100 || dut.local_fault || fault_sticky)
                    $fatal(1, "retained candidate diagnostic prevented the following frame");
                run_request = 0;
                clear_diagnostics = 1;
                step(0, 0);
                clear_diagnostics = 0;
                repeat (5) step(0, 0);
                if (retained_causes != 0 || fault_sticky || !idle ||
                    frames_started != (PAIRED_MODE ? 0 : 2) ||
                    frames_completed != (PAIRED_MODE ? 0 : 2))
                    $fatal(1, "disarmed candidate diagnostic clear changed capture accounting");
            end
            7: begin
                // A hand-written, noncanonical stream can exhaust both
                // decoded descriptors before hundreds of useless zero gaps
                // are read. With no ingress FIFO, losing ISP readiness must
                // fault capture immediately rather than lose pixels silently.
                write_isp(16'h0004, 32'd0);
                write_isp(16'h2000, 32'd1); // pair 0, correct pixel 0
                write_isp(16'h2004, 32'd5); // pair 1, correct pixel 0
                for (index = 2; index < 512; index = index + 1)
                    write_isp(16'(16'h2000 + index * 4), 32'd0);
                write_isp(16'h000c, 32'd512);
                start_isp();
                // This case deliberately applies real defect corrections;
                // the test double still checks framing and drain behavior.
                dut.processing.check_bypass_pixels = 0;
                run_request = 1;
                index = 0;
                while (!fault_sticky && index < 16) begin
                    step(1, index);
                    index = index + 1;
                end
                if (!fault_sticky || active || idle || frames_started != 1
                        || frames_completed != 0 || index >= 16)
                    $fatal(1, "noncanonical table readiness loss did not fault capture promptly");
            end
            8: begin
                if (!PAIRED_MODE)
                    $fatal(1, "paired control test requires PAIRED_MODE");
                // The shared epoch ID must remain authoritative even when
                // it differs from either camera's local admitted count.
                capture_frame_id = 32'h8123_4567;
                run_request = 1;
                // Also changes the pending ID and attempts CLEAR after
                // RUN=0 while the already admitted frame is still active.
                send_frame(1, 0, 1);
                drain_frame(0, 32'h8123_4567);
                if (frames_started != 1 || frames_completed != 1)
                    $fatal(1, "active paired CLEAR changed frame counters");

                clear_diagnostics = 1;
                step(0, 0);
                clear_diagnostics = 0;
                repeat (5) step(0, 0);
                if (frame_id || frames_started || frames_completed || fault_sticky)
                    $fatal(1, "paused paired CLEAR did not reset frame accounting");
                wb_cyc_i = 1;
                wb_stb_i = 1;
                wb_we_i = 0;
                wb_adr_i = 16'h0004;
                step(0, 0);
                if (!wb_ack_o || wb_err_o || wb_dat_o != 1)
                    $fatal(1, "paired CLEAR reset retained ISP configuration");
                bus_idle();
                step(0, 0);

                // Resume without reprogramming the ISP. The private count
                // restarts at zero; the global ID need not do so.
                capture_frame_id = 32'hfedc_ba98;
                run_request = 1;
                send_frame(1);
                drain_frame(0, 32'hfedc_ba98);
            end

            9: begin : marker_fault_edges
                integer fault_kind, final_index;
                // Each first bad pair may still enter the real ISP on the
                // fault edge. The controller must immediately cancel and
                // recover its position before accepting another frame.
                for (fault_kind = 0; fault_kind < 7; fault_kind = fault_kind + 1) begin
                    reset_design();
                    start_isp();
                    run_request = 1;
                    final_index = fault_kind == 2 ? 639 : 7;
                    for (index = 0; index < final_index; index = index + 1)
                        step(1, index);
                    case (fault_kind)
                        0: flip_sof = 1;
                        1, 2: flip_eol = 1;
                        3: flip_eof = 1;
                        4: pair_abort = 1;
                        5: capture_permit = 0;
                        6: receiver_fault = 1;
                    endcase
                    step(1, final_index);
                    if (!fault_sticky || active || idle || frames_completed != 0
                        || dut.core_eof || !dut.processing_reset)
                        $fatal(1, "fault edge %0d did not cancel the pending ISP output", fault_kind);
                    flip_sof = 0; flip_eol = 0; flip_eof = 0;
                    pair_abort = 0; capture_permit = 1; receiver_fault = 0;
                    repeat (2) step(0, 0);
                    run_request = 0;
                    clear_diagnostics = 1;
                    step(0, 0);
                    clear_diagnostics = 0;
                    repeat (3) step(0, 0);
                    wait_idle();
                    start_isp();
                    run_request = 1;
                    step(1, 0);
                    step(1, 1);
                    if (!active || fault_sticky || !dut.processing.in_href)
                        $fatal(1, "fault edge %0d did not recover the first-pair position", fault_kind);
                end
                $display("COVER fault_accept_edges=6 pair_abort_edges=1");
            end
            10: begin : reset_near_eof
                integer reset_phase;
                for (reset_phase = 0; reset_phase < 3; reset_phase = reset_phase + 1) begin
                    reset_design();
                    start_isp();
                    run_request = 1;
                    for (index = 0; index < 511999; index = index + 1) begin
                        if (index != 0 && index % 640 == 0)
                            repeat (256) step(0, 0);
                        step(1, index);
                    end
                    if (reset_phase == 2) step(1, 511999);
                    rst = 1;
                    step(reset_phase == 1, 511999);
                    if (active || fault_sticky || frame_id || frames_started || frames_completed
                        || dut.core_eof || !dut.processing_reset)
                        $fatal(1, "one-cycle reset phase %0d leaked final-pair state", reset_phase);
                    rst = 0;
                    run_request = 0;
                    repeat (3) step(0, 0);
                    wait_idle();
                    start_isp();
                    run_request = 1;
                    step(1, 0);
                    step(1, 1);
                    if (!active || fault_sticky || frames_started != 1 || frames_completed != 0)
                        $fatal(1, "one-cycle reset phase %0d did not restart", reset_phase);
                end
                $display("COVER final_pair_reset_before=1 simultaneous=1 after=1");
            end
            11: begin : bus_and_sensor_bubbles
                integer round_number, cycle_in_round, stream_index;
                logic [31:0] random_state;
                random_state = 32'hcf01_3297;
                for (round_number = 0; round_number < 32; round_number = round_number + 1) begin
                    reset_design();
                    // Stopped-mode CONTROL writes can reset the ISP position;
                    // no camera frame can be owned during those writes.
                    write_isp(16'h0004, 0);
                    write_isp(16'h0004, 0);
                    start_isp();
                    dut.processing.check_raster_cadence = 0;
                    run_request = 1;
                    stream_index = 0;
                    for (cycle_in_round = 0; cycle_in_round < 384; cycle_in_round = cycle_in_round + 1) begin
                        random_state = {random_state[30:0], random_state[31] ^ random_state[21] ^ random_state[1] ^ random_state[0]};
                        wb_cyc_i = stream_index != 0 && random_state[0];
                        wb_stb_i = random_state[1];
                        wb_we_i = random_state[2];
                        wb_sel_i = random_state[6:3];
                        case (random_state[9:7])
                            0: wb_adr_i = 16'h0004;
                            1: wb_adr_i = 16'h0005;
                            2: wb_adr_i = 16'h000c;
                            3: wb_adr_i = 16'h2000;
                            4: wb_adr_i = 16'h1000;
                            default: wb_adr_i = {random_state[15:2], 2'b00};
                        endcase
                        wb_dat_i = random_state;
                        step(random_state[10] || stream_index == 0, stream_index);
                        if (random_state[10] || stream_index == 0) stream_index = stream_index + 1;
                        if (fault_sticky || !active)
                            $fatal(1, "bubbled sensor or blocked/read WB traffic disturbed capture");
                    end
                    bus_idle();
                    wb_sel_i = 4'hf;
                    pair_abort = 1;
                    step(0, 0);
                    pair_abort = 0;
                    if (!fault_sticky || active)
                        $fatal(1, "bus stress cancellation did not latch fault");
                    dut.processing.check_raster_cadence = 1;
                end
                $display("COVER bubbled_bus_clocks=12288 aborts=32");
            end
            12: begin : wishbone_at_admission
                integer bus_mode;
                for (bus_mode = 0; bus_mode < 8; bus_mode = bus_mode + 1) begin
                    reset_design();
                    start_isp();
                    run_request = 1;
                    wb_cyc_i = bus_mode != 4;
                    wb_stb_i = bus_mode != 5;
                    wb_we_i = bus_mode != 7;
                    wb_sel_i = bus_mode == 2 ? 4'h3 : 4'hf;
                    wb_adr_i = bus_mode == 1 ? 16'h0005
                        : (bus_mode == 6 ? 16'h2000 : 16'h0004);
                    wb_dat_i = bus_mode == 3 ? 32'd1 : 32'd0;
                    step(1, 0);
                    if (bus_mode == 0) begin
                        // A simultaneous valid stop suppresses input ready;
                        // the attempted SOF must fault, without admission.
                        if (!fault_sticky || active || frames_started != 0 || !wb_ack_o || wb_err_o)
                            $fatal(1, "simultaneous ISP stop/SOF did not reject admission");
                    end else begin
                        if (fault_sticky || !active || frames_started != 1)
                            $fatal(1, "non-stopping WB request disturbed SOF admission mode %0d", bus_mode);
                        if (bus_mode == 4 || bus_mode == 5) begin
                            if (wb_ack_o || wb_err_o)
                                $fatal(1, "withdrawn WB request received a response");
                        end else if (bus_mode == 7) begin
                            if (!wb_ack_o || wb_err_o || wb_dat_o != 1)
                                $fatal(1, "concurrent CONTROL read changed configuration");
                        end else if (wb_ack_o || !wb_err_o)
                            $fatal(1, "invalid idle WB write was not rejected mode %0d", bus_mode);
                        bus_idle();
                        wb_sel_i = 4'hf;
                        step(1, 1);
                        if (fault_sticky || !active)
                            $fatal(1, "WB admission edge lost the next-pair position");
                    end
                    bus_idle();
                end
                $display("COVER first_pair_bus_modes=8");
            end
            default: $fatal(1, "unknown case %0d", case_number);
        endcase
        $display("PASS camera control case %0d", case_number);
        $finish;
    end
    initial begin
        #30000000;
        $fatal(1, "simulation timeout");
    end
endmodule

`default_nettype wire
