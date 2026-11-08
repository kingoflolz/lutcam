// Exercise the retained-nine contract through the real FIFO, numerical core,
// ROMs, and packet output. Packet bit 30 is reserved and must remain zero.
module retained_nine_tb;
    logic clk = 0;
    always #5 clk = !clk;
    logic rst = 1;
    logic in_valid = 0, in_group_end = 0, in_centroid_valid = 0;
    logic [14:0] in_x_q4 = 0;
    logic [13:0] in_y_q4 = 0;
    logic [7:0] in_log_mass_q5_3 = 0, in_frame_seq = 0;
    logic [1:0] in_scale = 0;
    logic out_ready = 1;
    wire out_valid, out_last, busy;
    wire [31:0] out_data;
    wire [5:0] fifo_level;
    wire candidate_start, candidate_done;
    wire fifo_drop_sticky, protocol_error_sticky;

    LfTagCompact3x3DecoderLocalizer dut (
        .config_read_valid(), .config_read_address(), .config_read_ready(1'b0),
        .config_response_valid(1'b0), .config_read_data(32'b0),
        .clk, .rst, .in_valid, .in_group_end, .in_x_q4, .in_y_q4,
        .in_log_mass_q5_3, .in_centroid_valid, .in_scale, .in_frame_seq,
        .cfg_inv_fx_q2_30(32'h00222222), .cfg_inv_fy_q2_30(32'h00222222),
        .cfg_cx_q16_16(32'(320 << 16)), .cfg_cy_q16_16(32'(240 << 16)),
        .cfg_printed_width_q16_16(32'h00003333),
        .cfg_mean_residual_limit_q16(16'd1024),
        .cfg_max_mass_spread(8'd19), .cfg_min_baseline_q4(16'd16),
        .centroid_valid(), .centroid_ready(1'b1), .centroid_data(), .centroid_last(),
        .out_valid, .out_ready, .out_data, .out_last, .busy, .fifo_level,
        .candidate_start, .candidate_done, .candidate_cycles(),
        .clear_diagnostics(1'b0), .fifo_drop_sticky, .protocol_error_sticky,
        .core_idle(), .core_dispatch_ready(),
        .external_fifo_head_valid(1'b0), .external_fifo_head_scale(2'b0),
        .external_fifo_head_frame_seq(8'b0),
        .external_fifo_head_protocol(1'b0), .external_fifo_head_mass_spread(8'b0),
        .external_fifo_head_pop(), .external_fifo_desc_read_valid(),
        .external_fifo_desc_read_index(), .external_fifo_desc_read_data_valid(1'b0),
        .external_fifo_desc_read_data(40'b0), .external_fifo_desc_read_centroid_valid(1'b0),
        .external_fifo_level(6'b0), .external_fifo_drop_sticky(1'b0),
        .external_protocol_error_sticky(1'b0)
    );

    localparam integer TAG_ID = 'h1234;
    localparam logic [7:0] FRAME = 8'h5a;
    localparam integer FAULT_NONE = 0;
    localparam integer FAULT_CENTROID = 1;
    localparam integer FAULT_SCALE = 2;
    localparam integer FAULT_FRAME = 3;
    logic [31:0] packet [8][22];
    integer packet_length [8];
    integer packet_count = 0, word_count = 0;
    integer starts = 0, dones = 0, cycle = 0;
    integer which_case;
    bit stall_enable = 0, expect_protocol = 0, was_stalled = 0;
    logic [32:0] held;

    // Repeated multi-clock stalls exercise both headers and pose payloads.
    always @(negedge clk)
        out_ready = !stall_enable || ((cycle % 13) >= 5);

    always @(posedge clk) begin
        cycle = cycle + 1;
        if (rst) begin
            packet_count = 0;
            word_count = 0;
            starts = 0;
            dones = 0;
            was_stalled = 0;
        end else begin
            if (was_stalled && (!out_valid || {out_last, out_data} !== held))
                $fatal(1, "packet changed while stalled");
            was_stalled = out_valid && !out_ready;
            held = {out_last, out_data};
            if (candidate_start) starts = starts + 1;
            if (candidate_done) dones = dones + 1;
            if (fifo_drop_sticky || (protocol_error_sticky && !expect_protocol))
                $fatal(1, "unexpected FIFO/protocol diagnostic");
            if (out_valid && out_ready) begin
                if (packet_count >= 8 || word_count >= 22)
                    $fatal(1, "unexpected packet length/count");
                if (!busy) $fatal(1, "busy fell before packet drain");
                if (word_count == 1 && out_data[30] !== 1'b0)
                    $fatal(1, "reserved packet metadata bit 30 was set");
                packet[packet_count][word_count] = out_data;
                word_count = word_count + 1;
                if (out_last) begin
                    if (word_count != 8 && word_count != 22)
                        $fatal(1, "malformed packet length %0d", word_count);
                    if (word_count == 8 && packet[packet_count][6] == 32'd3)
                        $fatal(1, "removed overflow policy emitted rejection reason 3");
                    packet_length[packet_count] = word_count;
                    packet_count = packet_count + 1;
                    word_count = 0;
                end
            end
        end
    end

    task automatic tick(input integer n = 1);
        repeat (n) @(negedge clk);
    endtask

    // The same ideal retained-nine fixture used by the shared-core tests:
    // 48-pixel grid pitch, center (320,240), seven data cells then two anchors,
    // in increasing log-mass order. No image or candidate-generation stub is
    // substituted for any part of the decoder under test.
    task automatic candidate(input integer fault = FAULT_NONE,
                             input integer gap = 0);
        integer gx, gy, symbol, xx, yy;
        for (integer i = 0; i < 9; i++) begin
            if (i < 7) begin
                case (i)
                    0: begin gx = 0; gy = -1; end
                    1: begin gx = -1; gy = 0; end
                    2: begin gx = 0; gy = 0; end
                    3: begin gx = 1; gy = 0; end
                    4: begin gx = -1; gy = 1; end
                    5: begin gx = 0; gy = 1; end
                    default: begin gx = 1; gy = 1; end
                endcase
                symbol = (TAG_ID >> (2 * i)) & 3;
                xx = 5120 + gx * 768 + ((symbol & 2) ? 64 : -64);
                yy = 3840 + gy * 768 + ((symbol & 1) ? 64 : -64);
                in_log_mass_q5_3 = 8'('h40 + i);
            end else begin
                xx = 5120 + ((i == 7) ? -768 : 768);
                yy = 3840 - 768;
                in_log_mass_q5_3 = 8'('h50 + i - 7);
            end
            in_x_q4 = 15'(xx);
            in_y_q4 = 14'(yy);
            in_valid = 1;
            in_group_end = i == 8;
            in_centroid_valid = !(fault == FAULT_CENTROID && i == 4);
            in_scale = (fault == FAULT_SCALE && i == 4) ? 2'd1 : 2'd0;
            in_frame_seq = (fault == FAULT_FRAME && i == 4) ? FRAME ^ 8'd1 : FRAME;
            tick();
            in_valid = 0;
            in_group_end = 0;
            if (gap != 0) tick(gap);
        end
    endtask

    task automatic finish_packets(input integer expected_count);
        bit finished;
        finished = 0;
        for (integer i = 0; i < 10000; i++) begin
            tick();
            if (packet_count == expected_count && !busy) begin
                finished = 1;
                break;
            end
        end
        if (!finished)
            $fatal(1, "drain timeout: got %0d wanted %0d", packet_count, expected_count);
        tick(8);
        if (packet_count != expected_count || word_count != 0 || fifo_level != 0 ||
                starts != expected_count || dones != expected_count)
            $fatal(1, "candidate/packet accounting mismatch");
        if (protocol_error_sticky !== expect_protocol)
            $fatal(1, "protocol diagnostic disagrees with injected fault");
    endtask

    task automatic check_accepted(input integer index);
        if (packet_length[index] != 22 || !packet[index][1][31] ||
                packet[index][0] != 32'(TAG_ID) || packet[index][6] != 0 ||
                packet[index][1][29:28] != 0 || packet[index][1][27:20] != FRAME ||
                packet[index][8][31:28] != 4'd0 || !packet[index][8][0] ||
                packet[index][21] == 0)
            $fatal(1, "packet %0d did not contain the expected accepted ID and pose", index);
    endtask

    task automatic compare_accepted(input integer first, input integer second);
        check_accepted(first);
        check_accepted(second);
        for (integer i = 0; i < 22; i++)
            if (packet[first][i] !== packet[second][i])
                $fatal(1, "accepted packet word %0d changed: %08h != %08h",
                    i, packet[first][i], packet[second][i]);
    endtask

    task automatic check_rejected(input integer index, input integer reason);
        logic [31:0] expected;
        if (packet_length[index] != 8)
            $fatal(1, "malformed group %0d was accepted", index);
        for (integer i = 0; i < 8; i++) begin
            expected = 0;
            if (i == 1) expected = {4'b0, FRAME, 20'b0};
            if (i == 6) expected = 32'(reason);
            if (packet[index][i] !== expected)
                $fatal(1, "rejection %0d word %0d was %08h expected %08h",
                    index, i, packet[index][i], expected);
        end
    endtask

    string packet_output;
    integer packet_output_file;
    initial begin
        if (!$value$plusargs("CASE=%d", which_case)) which_case = 0;
        tick(5);
        rst = 0;
        tick(3);
        stall_enable = 1;
        case (which_case)
            0: begin
                candidate();
                candidate(FAULT_NONE, 1);
                finish_packets(2);
                compare_accepted(0, 1);
            end
            1: begin
                candidate();
                candidate(FAULT_CENTROID);
                candidate();
                finish_packets(3);
                check_rejected(1, 1);
                compare_accepted(0, 2);
            end
            2: begin
                candidate();
                expect_protocol = 1;
                candidate(FAULT_SCALE);
                candidate(FAULT_FRAME);
                candidate();
                finish_packets(4);
                check_rejected(1, 14);
                check_rejected(2, 14);
                compare_accepted(0, 3);
            end
            default: $fatal(1, "unknown case %0d", which_case);
        endcase
        // Feed the actual localizer result into downstream packetizer tests,
        // keeping their unversioned status contract coupled to production RTL.
        if ($value$plusargs("PACKET_OUTPUT=%s", packet_output)) begin
            check_accepted(0);
            packet_output_file = $fopen(packet_output, "w");
            if (!packet_output_file) $fatal(1, "cannot open packet output");
            for (integer i = 0; i < 22; i++)
                $fdisplay(packet_output_file, "%08x", packet[0][i]);
            $fclose(packet_output_file);
        end
        $display("PASS retained nine case %0d packets %0d", which_case, packet_count);
        $finish;
    end
    initial begin #1000000; $fatal(1, "global timeout"); end
endmodule
