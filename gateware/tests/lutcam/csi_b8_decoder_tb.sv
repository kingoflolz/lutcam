module csi_b8_decoder_tb;
    logic byte_clk = 1'b0;
    logic reset_n = 1'b0;
    logic burst_start = 1'b0;
    logic [7:0] raw_word = 8'd0;
    logic raw_valid = 1'b0;
    wire unused_byte_valid;
    wire [7:0] unused_byte_data;
    wire unused_sync_found, unused_sync_error, unused_queue_overflow;

    Ov9281LaneAligner uut (
        .byte_clk, .reset_n, .burst_start, .discard(1'b0),
        .raw_word, .raw_valid, .pair_pop(1'b0),
        .byte_valid(unused_byte_valid), .byte_data(unused_byte_data),
        .sync_found(unused_sync_found), .sync_error(unused_sync_error),
        .queue_overflow(unused_queue_overflow)
    );

    logic [15:0] expected_window;
    logic [7:0] expected_matches;
    logic expected_unique, expected_ambiguous;
    logic [2:0] expected_offset;
    logic [7:0] seen_offsets = 8'd0;
    integer unmatched = 0;
    integer matched = 0;
    integer offset_counts [0:7];

    initial begin
        for (integer offset = 0; offset < 8; offset++)
            offset_counts[offset] = 0;
        #1 byte_clk = 1'b1;
        #1 byte_clk = 1'b0;
        reset_n = 1'b1;
        raw_valid = 1'b1;

        // Load history through the actual burst-start register path, then
        // drive the next word. No force, hierarchical assignment, or archived
        // implementation is used. This enumerates binary inputs; it does not
        // claim four-state X/Z propagation equivalence.
        for (integer window = 0; window < 65536; window++) begin
            expected_window = window[15:0];
            burst_start = 1'b1;
            raw_word = window[7:0];
            #1 byte_clk = 1'b1;
            #1 byte_clk = 1'b0;
            burst_start = 1'b0;
            raw_word = window[15:8];
            #1;

            for (integer offset = 0; offset < 8; offset++)
                expected_matches[offset] =
                    expected_window[offset +: 8] == 8'hb8;
            expected_unique = 1'b1;
            expected_offset = 3'd0;
            case (expected_matches)
                8'b0000_0001: expected_offset = 3'd0;
                8'b0000_0010: expected_offset = 3'd1;
                8'b0000_0100: expected_offset = 3'd2;
                8'b0000_1000: expected_offset = 3'd3;
                8'b0001_0000: expected_offset = 3'd4;
                8'b0010_0000: expected_offset = 3'd5;
                8'b0100_0000: expected_offset = 3'd6;
                8'b1000_0000: expected_offset = 3'd7;
                default: begin
                    expected_unique = 1'b0;
                    expected_offset = 3'd0;
                end
            endcase
            expected_ambiguous = (expected_matches != 8'd0) && !expected_unique;

            if (uut.search_window !== expected_window ||
                uut.sync_matches !== expected_matches ||
                uut.unique_sync_match !== expected_unique ||
                uut.sync_match_offset !== expected_offset ||
                uut.ambiguous_sync_match !== expected_ambiguous)
                $fatal(1, "B8 mismatch window=%04h matches=%02h/%02h unique=%b/%b offset=%0d/%0d ambiguous=%b/%b",
                    expected_window, uut.sync_matches, expected_matches,
                    uut.unique_sync_match, expected_unique,
                    uut.sync_match_offset, expected_offset,
                    uut.ambiguous_sync_match, expected_ambiguous);
            if (expected_ambiguous)
                $fatal(1, "B8 overlap in binary window=%04h", expected_window);
            if (expected_unique) begin
                matched++;
                seen_offsets[expected_offset] = 1'b1;
                offset_counts[expected_offset]++;
            end else begin
                unmatched++;
            end
        end

        if (matched != 2048 || unmatched != 63488 || seen_offsets != 8'hff)
            $fatal(1, "Incomplete B8 window coverage");
        for (integer offset = 0; offset < 8; offset++)
            if (offset_counts[offset] != 256)
                $fatal(1, "Incomplete B8 offset coverage offset=%0d count=%0d",
                    offset, offset_counts[offset]);
        $display("PASS B8 binary windows=65536 unmatched=%0d unique=%0d offsets=%02h",
            unmatched, matched, seen_offsets);
        $finish;
    end
endmodule
