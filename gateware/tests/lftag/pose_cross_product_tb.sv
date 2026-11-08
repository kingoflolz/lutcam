`timescale 1ns/1ps
module pose_cross_product_tb;
    logic clk = 0;
    always #5 clk = ~clk;
    logic rst = 1;
    logic in_valid = 0;
    wire in_ready;
    logic in_pose_enable = 1;
    logic in_saturation = 0;
    logic [31:0] in_printed_width_q16 = 65536;
    wire out_valid;
    logic out_ready = 0;
    wire out_pose_valid;
    wire out_weak_perspective;
    wire out_conditioning_warning;
    wire out_saturation;
    wire [3:0] result_read_address_a;
    wire [3:0] result_read_address_b;
    logic signed [17:0] result_read_data_a;
    logic signed [17:0] result_read_data_b;
    wire result_write_valid;
    wire [3:0] result_write_address;
    wire [17:0] result_write_data_a;
    wire [17:0] result_write_data_b;
    wire rsqrt_req_valid;
    logic rsqrt_req_ready = 1;
    wire [36:0] rsqrt_req_radicand;
    logic rsqrt_rsp_valid = 0;
    wire rsqrt_rsp_ready;
    logic [18:0] rsqrt_rsp_lambda_q10 = 0;
    logic [3:0] rsqrt_rsp_left_shift = 0;
    logic [3:0] requested_left_shift = 0;
    logic [18:0] requested_lambda;
    logic [36:0] expected_radicand;
    logic signed [17:0] bank_a [0:15];
    logic signed [17:0] bank_b [0:15];
    integer checked = 0;

    LfTagPoseReconstruct dut (.*);

    // Match the production registered, read-first result RAM exactly. H is
    // loaded only in A; B retains the preceding candidate's rotation columns.
    always @(posedge clk) begin
        if (result_write_valid) begin
            bank_a[result_write_address] <= result_write_data_a;
            bank_b[result_write_address] <= result_write_data_b;
        end
        result_read_data_a <= bank_a[result_read_address_a];
        result_read_data_b <= bank_b[result_read_address_b];
    end

    // Isolate pose arithmetic from the independently tested rsqrt service.
    // Its exact Q10 response is an input to the reference calculation below.
    always @(posedge clk) begin
        if (rst) begin
            rsqrt_rsp_valid <= 0;
        end else begin
            if (rsqrt_rsp_valid && rsqrt_rsp_ready)
                rsqrt_rsp_valid <= 0;
            if (rsqrt_req_valid && rsqrt_req_ready) begin
                assert (rsqrt_req_radicand == expected_radicand)
                    else $fatal(1, "wrong norm got=%0d expected=%0d", rsqrt_req_radicand, expected_radicand);
                rsqrt_rsp_valid <= 1;
                rsqrt_rsp_lambda_q10 <= requested_lambda;
                rsqrt_rsp_left_shift <= requested_left_shift;
            end
        end
    end

    task automatic tick;
        @(posedge clk);
        #1;
    endtask

    task automatic check_skewed_conditioning(
        input integer signed first_column,
        input integer signed second_column
    );
        integer signed h [0:8];
        integer signed lambda_q10;
        real square_sum;
        begin
            // Two orthogonal 3D directions, each using every row. Integer
            // division also introduces realistic Q16 coefficient rounding.
            h = '{2 * first_column / 3, 2 * second_column / 3, 0,
                -2 * first_column / 3, second_column / 3, 0,
                first_column / 3, -2 * second_column / 3, 65536};
            square_sum = 0.0;
            for (integer row = 0; row < 3; row += 1)
                square_sum += real'(h[row * 3]) * real'(h[row * 3])
                    + real'(h[row * 3 + 1]) * real'(h[row * 3 + 1]);
            lambda_q10 = $rtoi(67108864.0 / $sqrt(square_sum * 0.5));
            check_pose(h, lambda_q10, 13107);
        end
    endtask

    task automatic check_pose(
        input integer signed h [0:8],
        input integer signed lambda_q10,
        input integer signed width_q16,
        input integer unsigned left_shift = 0
    );
        longint signed r1 [0:2];
        longint signed r2 [0:2];
        longint signed r3 [0:2];
        longint signed expected_rotation [0:8];
        longint signed expected_translation [0:2];
        longint signed square_sum;
        longint signed rotation_norm1_square;
        longint signed rotation_norm2_square;
        longint signed smaller_rotation_norm_square;
        longint signed larger_rotation_norm_square;
        logic expected_conditioning_warning;
        real rotation_norm_ratio;
        longint signed translation_operand;
        integer signed actual_translation;
        integer signed actual_rotation;
        integer latency;
        begin
            // Independent fixed-point oracle: arithmetic shifts round toward
            // minus infinity, with half-up rounding only for translation's
            // positive shared multiplier. Use 64-bit products throughout.
            square_sum = 0;
            rotation_norm1_square = 0;
            rotation_norm2_square = 0;
            for (integer row = 0; row < 3; row = row + 1) begin
                square_sum = square_sum + 64'(h[row * 3]) * 64'(h[row * 3])
                    + 64'(h[row * 3 + 1]) * 64'(h[row * 3 + 1]);
                r1[row] = ((64'(lambda_q10) * 64'(h[row * 3])) <<< left_shift) >>> 10;
                r2[row] = ((64'(lambda_q10) * 64'(h[row * 3 + 1])) <<< left_shift) >>> 10;
                rotation_norm1_square += r1[row] * r1[row];
                rotation_norm2_square += r2[row] * r2[row];
            end
            smaller_rotation_norm_square = rotation_norm1_square < rotation_norm2_square
                ? rotation_norm1_square : rotation_norm2_square;
            larger_rotation_norm_square = rotation_norm1_square < rotation_norm2_square
                ? rotation_norm2_square : rotation_norm1_square;
            expected_conditioning_warning = 64 * smaller_rotation_norm_square
                < 49 * larger_rotation_norm_square;
            rotation_norm_ratio = $sqrt(real'(smaller_rotation_norm_square)
                / real'(larger_rotation_norm_square));
            r3[0] = (r1[1] * r2[2] - r1[2] * r2[1]) >>> 16;
            r3[1] = (r1[2] * r2[0] - r1[0] * r2[2]) >>> 16;
            r3[2] = (r1[0] * r2[1] - r1[1] * r2[0]) >>> 16;
            translation_operand = (((64'(lambda_q10) * 64'(width_q16)) >>> 12) + 32) >>> 6;
            for (integer row = 0; row < 3; row = row + 1) begin
                expected_rotation[row * 3] = r1[row] <<< 13;
                expected_rotation[row * 3 + 1] = r2[row] <<< 13;
                expected_rotation[row * 3 + 2] = r3[row] <<< 13;
                expected_translation[row] = ((translation_operand * 64'(h[row * 3 + 2])) <<< left_shift) >>> 10;
            end

            @(negedge clk);
            assert (in_ready && !out_valid) else $fatal(1, "not ready for next pose");
            // Deliberately preserve all B contents across candidates.
            for (integer i = 0; i < 9; i = i + 1)
                bank_a[i] = 18'(h[i]);
            expected_radicand = 37'(square_sum >> 1);
            requested_lambda = 19'(lambda_q10);
            requested_left_shift = 4'(left_shift);
            in_printed_width_q16 = 32'(width_q16);
            in_valid = 1;
            tick();
            @(negedge clk);
            in_valid = 0;
            latency = 0;
            while (!out_valid && latency < 200) begin
                assert (!in_ready) else $fatal(1, "accepted while busy");
                tick();
                latency = latency + 1;
            end
            assert (out_valid && out_pose_valid && !out_saturation)
                else $fatal(1, "pose %0d rejected or timed out", checked);
            // Independent full-precision comparison of the transmitted Q16
            // rotation columns. The compact RTL comparison may differ only
            // within its documented 0.0002 norm-ratio quantization band.
            assert ((out_conditioning_warning == expected_conditioning_warning)
                    || ((rotation_norm_ratio > 0.8748) && (rotation_norm_ratio < 0.8752)))
                else $fatal(1, "pose %0d conditioning got=%0d expected=%0d ratio=%0.9f", checked,
                            out_conditioning_warning, expected_conditioning_warning, rotation_norm_ratio);
            // Recheck while the result is backpressured to catch late writes.
            repeat (4) begin
                assert (out_valid && !in_ready) else $fatal(1, "lost stalled pose");
                for (integer i = 0; i < 9; i = i + 1) begin
                    actual_rotation = $signed(bank_a[i]) * 8192;
                    assert (64'(actual_rotation) == expected_rotation[i])
                        else $fatal(1, "pose %0d R[%0d] got=%0d expected=%0d", checked,
                                    i, actual_rotation, expected_rotation[i]);
                end
                for (integer i = 0; i < 3; i = i + 1) begin
                    actual_translation = $signed({bank_b[9 + i][13:0], bank_a[9 + i]});
                    assert (64'(actual_translation) == expected_translation[i])
                        else $fatal(1, "pose %0d T[%0d] got=%0d expected=%0d", checked,
                                    i, actual_translation, expected_translation[i]);
                end
                tick();
            end
            @(negedge clk);
            out_ready = 1;
            tick();
            assert (in_ready && !out_valid) else $fatal(1, "did not consume pose");
            @(negedge clk);
            out_ready = 0;
            checked = checked + 1;
            $display("pose %0d matched all R/T words in %0d cycles", checked, latency);
        end
    endtask

    task automatic check_diagonal_conditioning(
        input integer signed first_column,
        input integer signed second_column
    );
        integer signed lambda_q10;
        begin
            lambda_q10 = $rtoi(67108864.0 / $sqrt(
                (real'(first_column) * real'(first_column)
                + real'(second_column) * real'(second_column)) * 0.5));
            check_pose('{first_column,0,0,0,second_column,0,0,0,65536}, lambda_q10, 13107);
        end
    endtask

    task automatic check_product_reset(input integer phase);
        integer latency;
        begin
            @(negedge clk);
            for (integer i=0; i<9; i++) bank_a[i]=0;
            bank_a[0]=256; bank_a[4]=256; bank_a[8]=65536;
            expected_radicand=65536; requested_lambda=65536; requested_left_shift=2;
            in_printed_width_q16=65536; in_valid=1;
            tick(); @(negedge clk); in_valid=0; latency=0;
            while (!(dut.state == (phase % 2 == 0 ? 24 : 25)
                    && dut.product_is_column == (phase >= 2)) && latency < 160) begin
                tick(); latency++;
            end
            assert(latency < 160) else $fatal(1, "did not reach product reset phase %0d", phase);
            @(negedge clk); rst=1; tick();
            assert(in_ready && !out_valid && !out_pose_valid)
                else $fatal(1, "product reset leaked a result");
            @(negedge clk); rst=0;
            repeat(3) begin tick(); assert(in_ready && !out_valid); end
            check_pose('{256,0,-100,0,256,99,0,0,1000},65536,65536,2);
        end
    endtask

    task automatic check_product_overflow(input logic translation_overflow,
                                           input logic negative_translation);
        integer latency;
        begin
            @(negedge clk);
            for(integer i=0;i<9;i++)bank_a[i]=0;
            bank_a[0]=1000; bank_a[4]=1000; bank_a[8]=translation_overflow ? 131071 : 1000;
            if (negative_translation)bank_a[2]=-131072;
            expected_radicand=1000000; requested_lambda=131071; requested_left_shift=9;
            in_printed_width_q16=translation_overflow ? 131071 : 65536;
            in_valid=1; tick(); @(negedge clk); in_valid=0; latency=0;
            while(!out_valid && latency<200)begin tick();latency++;end
            assert(out_valid && !out_pose_valid && out_saturation)
                else $fatal(1,"scaled %s overflow was not rejected",translation_overflow?"translation":"rotation");
            repeat(3)begin tick();assert(out_valid && !out_pose_valid && out_saturation);end
            @(negedge clk);out_ready=1;tick();@(negedge clk);out_ready=0;
        end
    endtask

    initial begin
        for (integer i = 0; i < 16; i = i + 1) begin
            bank_a[i] = 0;
            bank_b[i] = 18'(1000 + i);
        end
        repeat (2) tick();
        @(negedge clk);
        rst = 0;
        // Equal columns must be well conditioned at every representable
        // homography scale. The old H-square truncation warns at 513, 600,
        // 900, 1000, 1200 and 1400 despite their exact 1:1 norm ratio.
        // Mantissa/exponent products cover the old artificial lambda limit,
        // every new exponent, signed coefficients, and the full small-H range.
        for (integer shift = 1; shift <= 9; shift++) begin
            check_pose('{(512 >> (shift-1)),0,-100,0,(512 >> (shift-1)),99,0,0,1000},
                       65536, 65536, shift);
            check_pose('{-(512 >> (shift-1)),0,-99,0,(512 >> (shift-1)),-100,0,0,1000},
                       65536, 13107, shift);
        end
        check_diagonal_conditioning(513, 513);
        check_diagonal_conditioning(600, 600);
        check_diagonal_conditioning(900, 900);
        check_diagonal_conditioning(1000, 1000);
        check_diagonal_conditioning(1200, 1200);
        check_diagonal_conditioning(1400, 1400);
        check_diagonal_conditioning(8192, 8192);
        check_diagonal_conditioning(65536, 65536);
        check_diagonal_conditioning(131071, 131071);
        // Both column orders around the 7/8 boundary, including the exact
        // boundary; only its explicitly bounded quantization band may differ.
        for (integer delta = -32; delta <= 32; delta += 4) begin
            check_diagonal_conditioning(7000 + delta, 8000);
            check_diagonal_conditioning(8000, 7000 + delta);
            check_skewed_conditioning(7000 + delta, 8000);
            check_skewed_conditioning(8000, 7000 + delta);
        end
        check_diagonal_conditioning(1000, 2000);
        check_diagonal_conditioning(120000, 60000);
        check_diagonal_conditioning(-120000, 60000);
        for (integer phase=0;phase<4;phase++)check_product_reset(phase);
        check_product_overflow(0,0);
        check_product_overflow(1,0);
        check_product_overflow(1,1);
        check_pose('{256,0,-100,0,256,99,0,0,1000},65536,65536,2);
        $display("PASS pose cross product: %0d sequential full rotation/translation checks", checked);
        $finish;
    end
endmodule
