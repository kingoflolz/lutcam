`timescale 1ns/1ps

// Legal production-geometry raster stimulus. The runner replaces TwoPPCS5Stage
// with a differential wrapper; every S5 input still comes from S0..S4.
module tb;
    localparam integer WIDTH = 1696;
    localparam integer HEIGHT = 807;
    localparam integer PAIRS = WIDTH / 2;
    localparam integer ACTIVE_ROWS = 17;
    localparam integer RASTER_ROWS = ACTIVE_ROWS + 3;
    localparam integer RASTERS = 6;
    localparam integer ABORT_ROWS = 7;
    localparam integer ABORT_PAIRS = 37;
    localparam integer EXPECTED_PAIRS =
        RASTERS * RASTER_ROWS * PAIRS + ABORT_ROWS * PAIRS + ABORT_PAIRS;

    logic clk = 0;
    logic rst = 1;
    logic datavalid = 0;
    wire in_ready;
    logic d0 = 0, d1 = 0;
    logic feature_d0 = 0, feature_d1 = 0;
    logic [7:0] feature_value_d0 = 0, feature_value_d1 = 0;
    logic [9:0] feature_y = 0;
    logic sof = 0, eol = 0;
    wire s5_fv_valid, s5_fv_lane;
    wire [103:0] s5_fv_data;
    wire s5_close_moment_valid, s5_close_moment_lane;
    wire [103:0] s5_close_moment_data;
    wire fv_valid, fv_child_id, fv_child_valid;
    wire [103:0] fv_data;

    integer cycles = 0;
    integer accepted_pairs = 0;
    integer accepted_eols = 0;
    integer inserted_gaps = 0;
    integer ready_waits = 0;
    integer reset_edges = 0;
    integer close_events = 0;
    integer final_events = 0;
    logic [1:0] close_lanes = 0;
    logic [3:0] input_shapes = 0;
    logic [7:0] pixel_ones = 0;
    logic [7:0] pixel_zeroes = 0;
    logic [103:0] close_fingerprint = 0;
    logic [103:0] final_fingerprint = 0;

    TwoPPC #(
        .WIDTH(WIDTH), .HEIGHT(HEIGHT), .TABLE_DEPTH(848),
        .HOLE_FILL(0), .FEATURE_BBOX(0), .FEATURE_AREA(0),
        .FEATURE_AREA_BITS(3), .FEATURE_PRESENT(0), .FEATURE_MOMENTS(1),
        .CENTERED_MOMENTS(1), .COMPRESS_CENTERED_MOMENTS(1),
        .PAIR_FUSED_MOMENTS(1), .FEATURE_VALUE_WIDTH(8),
        .FEATURE_Y_OVERRIDE(1), .FEATURE_Y_HEIGHT(800),
        .CHILD_COUNT_BITS(0), .CHILD_ID_MODE(0), .EXPOSE_S5_FV(0)
    ) dut (
        .clk, .rst, .datavalid, .in_ready, .d0, .d1,
        .feature_d0, .feature_d1, .feature_value_d0, .feature_value_d1,
        .feature_y, .sof, .eol,
        .child_add0(1'b0), .child_add1(1'b0),
        .child_id0(1'b0), .child_id1(1'b0),
        .child_parent0_valid(), .child_parent1_valid(),
        .child_parent0_id(), .child_parent1_id(),
        .child_parent0_has_child(), .child_parent1_has_child(),
        .child_merge_valid(), .child_merge_id_a(), .child_merge_id_b(),
        .s5_fv_valid, .s5_fv_lane, .s5_fv_data,
        .s5_close_moment_valid, .s5_close_moment_lane, .s5_close_moment_data,
        .fv_valid, .fv_data, .fv_child_id, .fv_child_valid
    );

    function automatic logic foreground(
        input integer pattern, input integer x, input integer y
    );
        integer bx, by, radius, tile;
        logic [31:0] mixed;
        logic [15:0] bitmap;
        begin
            foreground = 0;
            bx = x % 64;
            by = y % 6;
            radius = (y <= 8) ? y : 16 - y;
            tile = x / 6 + 283 * (y / 6);
            mixed = 32'(x) * 32'h45d9f3b
                ^ 32'(y) * 32'h9e3779b9 ^ 32'hb5297a4d;
            mixed = mixed ^ (mixed >> 16);
            bitmap = 16'(32'(tile) * 32'h9e37 + 32'h6996);
            if (x > 0 && x < WIDTH-1 && y > 0 && y < ACTIVE_ROWS) begin
                case (pattern)
                    0: begin
                        // Several vertical runs merge, split, and merge again.
                        // Odd tiles shift boundaries to exercise both lanes.
                        bx = (x + ((x / 64) & 1)) % 64;
                        if (y <= 4)
                            foreground = (bx >= 3 && bx <= 6)
                                || (bx >= 13 && bx <= 16)
                                || (bx >= 25 && bx <= 28)
                                || (bx >= 41 && bx <= 44);
                        else if (y == 5)
                            foreground = bx >= 3 && bx <= 44;
                        else if (y <= 9)
                            foreground = (bx >= 5 && bx <= 8)
                                || (bx >= 19 && bx <= 22)
                                || (bx >= 35 && bx <= 38);
                        else if (y == 10)
                            foreground = bx >= 5 && bx <= 38;
                        else if (y <= 14)
                            foreground = bx >= 17 && bx <= 24;
                    end
                    1: begin
                        // Small islands, diagonal contacts, and blank rows.
                        bx = (x + ((y / 4) & 1)) % 8;
                        foreground = (y % 4 == 1 || y % 4 == 2)
                            && (bx == 1 || bx == 2);
                        if (y % 4 == 3)
                            foreground = x % 16 == (y % 8) + 4;
                    end
                    2: begin
                        // Expanding and shrinking diamonds; one bridge row.
                        foreground = (bx >= 16-radius && bx <= 16+radius)
                            || (bx >= 43-radius && bx <= 43+radius);
                        if (y == 9)
                            foreground = bx >= 9 && bx <= 50;
                    end
                    3: begin
                        // Wide connected components accumulate large moments.
                        // Sparse rows split the run before a full-width join.
                        if (y <= 2 || y == 7 || y >= 13)
                            foreground = 1;
                        else if (y <= 6)
                            foreground = x % 17 < 8;
                        else if (y >= 9)
                            foreground = x % 5 < 3;
                    end
                    4: begin
                        // Deterministic irregular islands and bridge rows.
                        foreground = mixed[7:0] < 8'd88;
                        if (y == 5 || y == 12)
                            foreground = x % 32 >= 3 && x % 32 <= 25;
                        if (y == 8)
                            foreground = 0;
                    end
                    default: begin
                        // Varied four-by-four interiors with a zero boundary.
                        bx = x % 6;
                        if (bx >= 1 && bx <= 4 && by >= 1 && by <= 4)
                            foreground = bitmap[(by-1)*4 + bx-1];
                    end
                endcase
            end
        end
    endfunction

    function automatic logic [7:0] pixel_weight(
        input integer pattern, input integer x, input integer y
    );
        begin
            if (pattern == 4) begin
                pixel_weight = 8'(x * 73 + y * 131 + (x >> 3));
                if (pixel_weight == 0) pixel_weight = 1;
            end
            else begin
                case ((x + 3*y + pattern) & 7)
                    0: pixel_weight = 1;
                    1: pixel_weight = 1;
                    2: pixel_weight = 2;
                    3: pixel_weight = 3;
                    4: pixel_weight = 127;
                    5: pixel_weight = 128;
                    6: pixel_weight = 254;
                    default: pixel_weight = 255;
                endcase
            end
        end
    endfunction

    function automatic logic [9:0] source_y(
        input integer pattern, input integer y
    );
        begin
            case (pattern)
                0: source_y = 10'(y);
                1: source_y = 10'(799-y);
                2: source_y = 10'((43*y) % 800);
                3: source_y = ((y & 1) != 0) ? 10'd799 : 10'd0;
                4: source_y = 10'(400+y);
                default: source_y = 10'(392+y/2);
            endcase
        end
    endfunction

    task automatic idle_inputs;
        begin
            datavalid = 0;
            d0 = 0; d1 = 0;
            feature_d0 = 0; feature_d1 = 0;
            feature_value_d0 = 0; feature_value_d1 = 0;
            feature_y = 0; sof = 0; eol = 0;
        end
    endtask

    task automatic tick;
        begin
            #5;
            if (rst)
                reset_edges++;
            else if (datavalid && in_ready) begin
                accepted_pairs++;
                if (eol) accepted_eols++;
                input_shapes[{d0,d1}] = 1;
                if (feature_d0) begin
                    pixel_ones = pixel_ones | feature_value_d0;
                    pixel_zeroes = pixel_zeroes | ~feature_value_d0;
                end
                if (feature_d1) begin
                    pixel_ones = pixel_ones | feature_value_d1;
                    pixel_zeroes = pixel_zeroes | ~feature_value_d1;
                end
            end
            clk = 1;
            #1;
            cycles++;
            if (!rst && s5_close_moment_valid) begin
                close_events++;
                close_lanes[s5_close_moment_lane] = 1;
                close_fingerprint = {close_fingerprint[102:0],close_fingerprint[103]}
                    ^ s5_close_moment_data;
            end
            if (!rst && fv_valid) begin
                final_events++;
                final_fingerprint = {final_fingerprint[102:0],final_fingerprint[103]}
                    ^ fv_data;
            end
            #4;
            clk = 0;
        end
    endtask

    task automatic idle_clocks(input integer count);
        begin
            idle_inputs();
            repeat (count) tick();
        end
    endtask

    task automatic reset_pipeline;
        begin
            idle_inputs();
            rst = 1;
            repeat (3) tick();
            rst = 0;
            idle_clocks(3);
            if (!in_ready)
                $fatal(1,"S0 not ready after reset");
        end
    endtask

    task automatic send_pair(
        input integer pattern, input integer y, input integer pair_index
    );
        integer waited, gap, x;
        begin
            gap = 0;
            if ((pair_index + 11*y + 17*pattern) % 97 == 0) gap = 1;
            if ((pair_index + 7*y + 23*pattern) % 257 == 0) gap = 3;
            inserted_gaps += gap;
            if (gap != 0) idle_clocks(gap);
            waited = 0;
            while (!in_ready) begin
                idle_clocks(1);
                ready_waits++;
                waited++;
                if (waited > 64)
                    $fatal(1,"S0 ready timeout pattern=%0d row=%0d pair=%0d",
                        pattern,y,pair_index);
            end
            x = pair_index * 2;
            datavalid = 1;
            d0 = foreground(pattern,x,y);
            d1 = foreground(pattern,x+1,y);
            // Sparse missing feature samples remain foreground, while all
            // valid feature samples have a nonzero weight in1..255.
            feature_d0 = d0 && (pattern != 5 || (x+y) % 13 != 0);
            feature_d1 = d1 && (pattern != 5 || (x+1+y) % 13 != 0);
            feature_value_d0 = pixel_weight(pattern,x,y);
            feature_value_d1 = pixel_weight(pattern,x+1,y);
            feature_y = source_y(pattern,y);
            sof = y == 0 && pair_index == 0;
            eol = pair_index == PAIRS-1;
            tick();
        end
    endtask

    initial begin : stimulus
        integer previous_closes, previous_finals, previous_pairs;
        reset_pipeline();
        // Abort with live pixels in every pipeline stage, without first
        // draining the partial row. The following reset must invalidate the
        // row history even though the line RAM itself is not reset.
        for (integer y=0; y<ABORT_ROWS; y++)
            for (integer pair_index=0; pair_index<PAIRS; pair_index++)
                send_pair(3,y,pair_index);
        for (integer pair_index=0; pair_index<ABORT_PAIRS; pair_index++)
            send_pair(3,ABORT_ROWS,pair_index);

        for (integer pattern=0; pattern<RASTERS; pattern++) begin
            reset_pipeline();
            previous_closes = close_events;
            previous_finals = final_events;
            previous_pairs = accepted_pairs;
            for (integer y=0; y<RASTER_ROWS; y++) begin
                for (integer pair_index=0; pair_index<PAIRS; pair_index++)
                    send_pair(pattern,y,pair_index);
                if (y == 4 || y == 11) begin
                    inserted_gaps += 7;
                    idle_clocks(7);
                end
            end
            // Three full background rows close all finite components before
            // this partial raster is reset. HEIGHT remains production807.
            idle_clocks(64);
            if (accepted_pairs - previous_pairs != RASTER_ROWS * PAIRS)
                $fatal(1,"Incorrect raster pair count");
            if (close_events == previous_closes || final_events == previous_finals)
                $fatal(1,"No component closures for pattern=%0d",pattern);
            $display("RASTER pattern=%0d closes=%0d final=%0d cycles=%0d",
                pattern,close_events-previous_closes,final_events-previous_finals,cycles);
        end
        if (accepted_pairs != EXPECTED_PAIRS)
            $fatal(1,"Incorrect total pair count got=%0d expected=%0d",accepted_pairs,EXPECTED_PAIRS);
        if (accepted_eols != RASTERS * RASTER_ROWS + ABORT_ROWS)
            $fatal(1,"Incorrect accepted EOL count");
        if (input_shapes != 4'b1111 || pixel_ones != 8'hff || pixel_zeroes != 8'hff)
            $fatal(1,"Missing input shape/feature-bit coverage");
        if (inserted_gaps == 0 || ready_waits == 0 || close_lanes != 2'b11)
            $fatal(1,"Missing gap, row-drain, or closure-lane coverage");
        $display("PASS legal rasters=%0d WIDTH=%0d HEIGHT=%0d TABLE=848 FV=104 cycles=%0d pairs=%0d eols=%0d gaps=%0d ready_waits=%0d reset_edges=%0d closes=%0d finals=%0d lanes=%b close_hash=%h final_hash=%h",
            RASTERS,WIDTH,HEIGHT,cycles,accepted_pairs,accepted_eols,inserted_gaps,
            ready_waits,reset_edges,close_events,final_events,close_lanes,
            close_fingerprint,final_fingerprint);
        $finish;
    end

    initial begin
        #20000000;
        $fatal(1,"Global raster stimulus timeout");
    end
endmodule
