// Exercise the actual current scorer, including the native EFX_DSP48 instance.
// Force only its upstream projected_w register to enumerate all signed19
// values; no vendor-model internals, arithmetic nets, or results are forced.
module offset_arithmetic_tb;
    logic clk = 0;
    logic rst = 1;
    logic clear = 0;
    logic signed [18:0] probe_w = 0;
    integer checked = 0, negative_w = 0, zero_w = 0;
    integer positive_wrap = 0, negative_wrap = 0;
    integer positive_offset = 0, negative_offset = 0;
    integer domain_values [0:2];

    LfTagHomogeneousGeometryScorer dut (
        .clk(clk), .rst(rst), .clear(clear),
        .start_valid(1'b0), .start_ready(), .in_scale(2'b0),
        .corner_x_flat(72'b0), .corner_y_flat(72'b0),
        .point_input_index_flat(20'b0),
        .descriptor_req_valid(), .descriptor_req_ready(1'b0),
        .descriptor_req_index(), .descriptor_rsp_valid(1'b0),
        .descriptor_rsp_x(18'sb0), .descriptor_rsp_y(18'sb0),
        .geometry_done_valid(), .geometry_done_invalid(),
        .map_cden(), .map_cg(), .map_ch(), .out_valid(),
        .out_ready(1'b0), .out_best_valid(), .out_input_invalid(),
        .out_best_hypothesis(), .out_best_id(),
        .out_best_score_q32(),
        .out_best_residual_sum(), .out_best_weight_sum(), .out_cycles()
    );

    task automatic check_value(input integer value, input integer phase);
        longint signed mathematical_numerator;
        longint signed signed_modulo23;
        longint signed signed_q8;
        begin
            probe_w = 19'(value);
            force dut.projected_w = probe_w;
            // Settling time, not a clock edge: phase0 keeps clk low and phase1
            // keeps clk high across the complete domain. A registered result
            // cannot pass these sweeps by tracking a previous input.
            #1;
            mathematical_numerator = 64'sd21 * value + 64'sd128;
            signed_modulo23 = mathematical_numerator & 64'h7f_ffff;
            if (signed_modulo23 >= 64'sd4194304)
                signed_modulo23 = signed_modulo23 - 64'sd8388608;
            signed_q8 = signed_modulo23 >>> 8;
            if ($signed(dut.projected_w) !== value)
                $fatal(1, "upstream test input did not reach scorer W=%0d", value);
            if (dut.axis_offset_numerator !== mathematical_numerator[22:0])
                $fatal(1, "offset numerator phase=%0d W=%0d expected=%0d got=%0d",
                    phase, value, signed_modulo23, $signed(dut.axis_offset_numerator));
            if ($signed(dut.response_axis_decode.offset_wide) !== signed_q8
                    || $signed(dut.response_axis_decode.offset) !== signed_q8)
                $fatal(1, "signed modulo-before-Q8 order changed W=%0d expected=%0d", value, signed_q8);
`ifdef SYNTHESIS
            if ($signed(dut.native_axis_offset.numerator) !== mathematical_numerator)
                $fatal(1, "native full48 result differs W=%0d expected=%0d got=%0d",
                    value, mathematical_numerator, $signed(dut.native_axis_offset.numerator));
`endif
            checked++;
            domain_values[phase]++;
            if (value < 0) negative_w++;
            if (value == 0) zero_w++;
            if (mathematical_numerator > 64'sd4194303) positive_wrap++;
            if (mathematical_numerator < -64'sd4194304) negative_wrap++;
            if (signed_q8 > 0) positive_offset++;
            if (signed_q8 < 0) negative_offset++;
        end
    endtask

    task check_weight_square(input integer value);
        longint unsigned expected_sum;
        begin
            clk = 0; rst = 0; clear = 1;
            #1; clk = 1; #1; clk = 0; clear = 0;
            probe_w = 19'(value);
            force dut.projected_w = probe_w;
            expected_sum = 0;
            repeat (5) begin
                force dut.state = dut.S_RESPONSE_X;
                #1; clk = 1; #1; clk = 0;
                force dut.state = dut.S_RESPONSE_Y;
                #1; clk = 1; #1; clk = 0;
                expected_sum += 64'(value) * 64'(value);
                if (dut.cross_product_accumulator != expected_sum)
                    $fatal(1, "split W-square accumulator mismatch W=%0d expected=%0d got=%0d",
                           value, expected_sum, dut.cross_product_accumulator);
            end
            release dut.state;
            release dut.projected_w;
        end
    endtask

    initial begin
        foreach (domain_values[p]) domain_values[p] = 0;
`ifdef SYNTHESIS
        if (!dut.USE_NATIVE_PROJECTION)
            $fatal(1, "native vendor-model path was not selected");
        if (dut.native_axis_offset.offset_dsp.MODE != "NORMAL"
                || !dut.native_axis_offset.offset_dsp.SIGNED
                || dut.native_axis_offset.offset_dsp.M_SEL != "P"
                || dut.native_axis_offset.offset_dsp.N_SEL != "C"
                || dut.native_axis_offset.offset_dsp.W_SEL != "X"
                || dut.native_axis_offset.offset_dsp.A_REG
                || dut.native_axis_offset.offset_dsp.B_REG
                || dut.native_axis_offset.offset_dsp.C_REG
                || dut.native_axis_offset.offset_dsp.P_REG
                || dut.native_axis_offset.offset_dsp.OP_REG
                || dut.native_axis_offset.offset_dsp.W_REG
                || dut.native_axis_offset.offset_dsp.O_REG
                || dut.native_axis_offset.offset_dsp.SHIFTER)
            $fatal(1, "native offset no longer uses the expected combinational configuration");
`else
        if (dut.USE_NATIVE_PROJECTION)
            $fatal(1, "portable arithmetic path was not selected");
`endif
        // Reset the complete scorer before isolating its combinational cone.
        #1; clk = 1; #1; clk = 0; #1;
        for (integer phase = 0; phase < 3; phase++) begin
            if (phase == 0) begin clk = 0; rst = 1; clear = 0; end
            if (phase == 1) begin clk = 1; rst = 0; clear = 1; end
            for (integer bits = 0; bits < 524288; bits++) begin
                integer permutation, value;
                // Odd affine multipliers permute every19-bit pattern exactly.
                // The three passes cover both sequential and distant changes.
                if (phase == 0) permutation = bits;
                else if (phase == 1) permutation = (bits * 32771 + 17) & 524287;
                else permutation = bits ^ 19'h55555;
                value = permutation >= 262144 ? permutation - 524288 : permutation;
                if (phase == 2) begin
                    clk = bits[0]; rst = bits[1]; clear = bits[2];
                end
                check_value(value, phase);
            end
        end
        release dut.projected_w;
        check_weight_square(0);
        check_weight_square(1);
        check_weight_square(131071);
        check_weight_square(131072);
        check_weight_square(131073);
        check_weight_square(262143);
        for (integer sample_index = 0; sample_index < 256; sample_index++)
            check_weight_square((sample_index * 32771 + 17) & 262143);
        if (checked != 1572864 || negative_w != 786432 || zero_w != 3
                || positive_wrap == 0 || negative_wrap == 0
                || positive_offset == 0 || negative_offset == 0)
            $fatal(1, "vacuous arithmetic coverage");
        foreach (domain_values[p])
            if (domain_values[p] != 524288) $fatal(1, "incomplete W domain in phase %0d", p);
        $display("PASS offset arithmetic checks=%0d domains=3 values_per_domain=524288 negative_w=%0d zero_w=%0d positive_wrap=%0d negative_wrap=%0d",
            checked, negative_w, zero_w, positive_wrap, negative_wrap);
        $finish;
    end
    initial begin #2000000; $fatal(1, "timeout"); end
endmodule
