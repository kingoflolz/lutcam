// Area-oriented direct planar-pose reconstruction from common-scale LFTag H.
//
// Bank A initially contains the nine signed-Q16 homography coefficients.  The
// leaf derives the shared reciprocal scale, writes the final rotation back
// over H, and writes translation into words 9..11.  Bank B duplicates the
// Q16 rotation columns while the cross product is formed, then holds the high
// translation bits.  This lets the complete decoder retain two RAM10 result
// banks instead of adding a register file or a separate pose-result store.
//
// One signed 18-by-18 DSP feedback lane is serialized across norm
// measurement, conditioning, rotation, cross-product, and translation.
module LfTagPoseReconstruct (
    input  logic                clk,
    input  logic                rst,

    input  logic                in_valid,
    output logic                in_ready,
    input  logic                in_pose_enable,
    input  logic                in_saturation,
    input  logic [31:0]         in_printed_width_q16,

    output logic                out_valid,
    input  logic                out_ready,
    output logic                out_pose_valid,
    output logic                out_weak_perspective,
    output logic                out_conditioning_warning,
    output logic                out_saturation,

    output logic [3:0]          result_read_address_a,
    output logic [3:0]          result_read_address_b,
    input  logic signed [17:0]  result_read_data_a,
    input  logic signed [17:0]  result_read_data_b,
    output logic                result_write_valid,
    output logic [3:0]          result_write_address,
    output logic [17:0]         result_write_data_a,
    output logic [17:0]         result_write_data_b,

    output logic                rsqrt_req_valid,
    input  logic                rsqrt_req_ready,
    output logic [36:0]         rsqrt_req_radicand,
    input  logic                rsqrt_rsp_valid,
    output logic                rsqrt_rsp_ready,
    input  logic [18:0]         rsqrt_rsp_lambda_q10,
    input  logic [3:0]          rsqrt_rsp_left_shift
);
    typedef enum logic [4:0] {
        S_IDLE,
        S_NORM_PREFETCH,
        S_NORM_MUL,
        S_RSQRT_REQ,
        S_WEAK_ACCUM,
        S_WEAK_CHECK,
        S_CONDITION_PREFETCH,
        S_CONDITION_NORM_MUL,
        S_CONDITION_NORM_RESULT,
        S_CONDITION_MIN,
        S_CONDITION_MAX,
        S_CONDITION_CHECK,
        S_RSQRT_WAIT,
        S_TRANSLATE_SCALE,
        S_TRANSLATE_SCALE_RESULT,
        S_TRANSLATE_MUL,
        S_TRANSLATE_RESULT,
        S_COLUMN_MUL,
        S_COLUMN_RESULT,
        S_CROSS_PREFETCH,
        S_CROSS_FIRST,
        S_CROSS_SECOND,
        S_CROSS_CORRECT,
        S_CROSS_RESULT,
        S_PRODUCT_SCALE,
        S_PRODUCT_RESULT
    } state_t;

    state_t state;
    logic [2:0] work_index;
    logic [16:0] norm1_square_coarse;
    logic [16:0] norm2_square_coarse;
    logic [15:0] weak_norm_threshold;
    logic signed [17:0] pose_lambda_q10;
    logic [3:0] pose_lambda_left_shift;
    logic signed [17:0] translation_operand_q10;
    logic translation_z_positive;

    logic signed [18:0] mul_operand_a;
    logic signed [17:0] mul_operand_b;
    logic dsp_clear;
    logic dsp_enable;
    (* syn_use_dsp = "yes" *) logic signed [47:0] dsp_accumulator;
    logic [16:0] minimum_square_coarse;
    logic [16:0] maximum_square_coarse;
    logic signed [17:0] rounded_translation_operand;
    logic signed [31:0] translated_q16;
    logic signed [17:0] column_q16;
    logic signed [17:0] cross_q16;
    logic column_range_ok;
    logic cross_range_ok;
    // Keep multiplier operands signed18. A small-norm reciprocal exponent
    // scales completed products serially in fabric, shared by all nine H
    // products. For exponent <=9, product bit0 never reaches the Q16 result;
    // bits41:1 retain every contributing bit, including negative round-down.
    logic signed [40:0] scaled_product;
    logic [3:0] product_shifts_left;
    logic product_is_column;
    logic product_range_ok;
    logic scaled_column_range_ok;
    logic signed [31:0] scaled_translation_q16;
    logic signed [17:0] scaled_column_q16;

    function automatic logic [3:0] column_address(input logic [2:0] index);
        case (index)
            3'd0: column_address = 4'd0;
            3'd1: column_address = 4'd3;
            3'd2: column_address = 4'd6;
            3'd3: column_address = 4'd1;
            3'd4: column_address = 4'd4;
            default: column_address = 4'd7;
        endcase
    endfunction

    function automatic logic [3:0] translation_h_address(
        input logic [1:0] index
    );
        case (index)
            2'd0: translation_h_address = 4'd2;
            2'd1: translation_h_address = 4'd5;
            default: translation_h_address = 4'd8;
        endcase
    endfunction

    assign minimum_square_coarse = (norm1_square_coarse
            < norm2_square_coarse)
        ? norm1_square_coarse : norm2_square_coarse;
    assign maximum_square_coarse = (norm1_square_coarse
            < norm2_square_coarse)
        ? norm2_square_coarse : norm1_square_coarse;

    assign in_ready = (state == S_IDLE) && (!out_valid || out_ready);
    assign rsqrt_req_valid = state == S_RSQRT_REQ;
    assign rsqrt_req_radicand = dsp_accumulator[37:1];
    assign rsqrt_rsp_ready = state == S_RSQRT_WAIT;

    // These slices implement signed arithmetic shifts.  The validated positive lambda and printed width bound the
    // rounded translation operand to signed 18 bits.
    assign rounded_translation_operand
        = $signed(dsp_accumulator[35:18])
        + $signed({17'b0, dsp_accumulator[17]});
    assign translated_q16 = dsp_accumulator[41:10];
    assign column_q16 = dsp_accumulator[27:10];
    assign cross_q16 = dsp_accumulator[33:16];
    assign column_range_ok = (&dsp_accumulator[47:27])
        || !(|dsp_accumulator[47:27]);
    assign cross_range_ok = (&dsp_accumulator[47:33])
        || !(|dsp_accumulator[47:33]);
    assign product_range_ok = (&dsp_accumulator[47:41])
        || !(|dsp_accumulator[47:41]);
    assign scaled_column_range_ok = (&scaled_product[40:26])
        || !(|scaled_product[40:26]);
    assign scaled_translation_q16 = scaled_product[40:9];
    assign scaled_column_q16 = scaled_product[26:9];

    always_comb begin : multiplier_operand_mux
        mul_operand_a = 19'sb0;
        mul_operand_b = 18'sb0;
        case (state)
            S_NORM_MUL, S_WEAK_ACCUM, S_CONDITION_NORM_MUL: begin
                mul_operand_a = $signed({result_read_data_a[17],
                    result_read_data_a});
                mul_operand_b = result_read_data_a;
            end
            S_CONDITION_MIN: begin
                mul_operand_a = $signed({2'b0, minimum_square_coarse});
                mul_operand_b = 18'sd64;
            end
            S_CONDITION_MAX: begin
                mul_operand_a = $signed({2'b0, maximum_square_coarse});
                mul_operand_b = -18'sd49;
            end
            S_TRANSLATE_SCALE: begin
                mul_operand_a = $signed({pose_lambda_q10[17],
                    pose_lambda_q10});
                mul_operand_b = $signed(in_printed_width_q16[17:0]);
            end
            S_TRANSLATE_MUL: begin
                mul_operand_a = $signed({translation_operand_q10[17],
                    translation_operand_q10});
                mul_operand_b = result_read_data_a;
            end
            S_COLUMN_MUL: begin
                mul_operand_a = $signed({pose_lambda_q10[17],
                    pose_lambda_q10});
                mul_operand_b = result_read_data_a;
            end
            S_CROSS_FIRST: begin
                mul_operand_a = $signed({result_read_data_a[17],
                    result_read_data_a});
                mul_operand_b = result_read_data_b;
            end
            S_CROSS_SECOND: begin
                // ~x is (-x - 1); the following correction cycle adds the
                // missing +1 times the other operand without a fabric negate.
                mul_operand_a = ~$signed({result_read_data_a[17],
                    result_read_data_a});
                mul_operand_b = result_read_data_b;
            end
            S_CROSS_CORRECT: begin
                mul_operand_a = 19'sd1;
                mul_operand_b = result_read_data_b;
            end
            default: begin end
        endcase
    end

    always_comb begin : dsp_feedback_control
        dsp_clear = 1'b0;
        dsp_enable = 1'b0;
        case (state)
            S_NORM_MUL,
            S_CONDITION_NORM_MUL,
            S_WEAK_ACCUM,
            S_CONDITION_MIN,
            S_TRANSLATE_SCALE,
            S_TRANSLATE_MUL,
            S_COLUMN_MUL,
            S_CROSS_FIRST,
            S_CONDITION_MAX,
            S_CROSS_SECOND,
            S_CROSS_CORRECT: dsp_enable = 1'b1;
            S_RSQRT_REQ: dsp_clear = rsqrt_req_ready;
            S_WEAK_CHECK,
            S_CONDITION_NORM_RESULT,
            S_CONDITION_CHECK,
            S_TRANSLATE_SCALE_RESULT,
            S_TRANSLATE_RESULT,
            S_COLUMN_RESULT,
            S_CROSS_RESULT: dsp_clear = 1'b1;
            S_RSQRT_WAIT: dsp_clear = rsqrt_rsp_valid;
            default: begin end
        endcase
    end

    // Keeping the multiply and feedback expression in one registered process
    // lets Efinity use a single DSP48 W-feedback lane.  Negative products use
    // either a negative constant or a complement-plus-correction sequence, so
    // every signed-18 input including -131072 remains exact without a fabric
    // negator.
    always_ff @(posedge clk) begin : pose_dsp_feedback
        if (rst || dsp_clear) begin
            dsp_accumulator <= 48'sb0;
        end else if (dsp_enable) begin
            dsp_accumulator <= dsp_accumulator
                + mul_operand_a * mul_operand_b;
        end
    end

    always_comb begin : result_ram_control
        result_read_address_a = 4'd0;
        result_read_address_b = 4'd0;
        case (state)
            S_NORM_PREFETCH, S_CONDITION_PREFETCH: begin
                result_read_address_a = column_address(3'd0);
            end
            S_NORM_MUL, S_CONDITION_NORM_MUL: begin
                if (work_index < 3'd5)
                    result_read_address_a = column_address(work_index + 1'b1);
                else
                    result_read_address_a = column_address(3'd5);
            end
            S_RSQRT_REQ,
            S_WEAK_ACCUM: begin
                result_read_address_a = column_address(3'd5);
            end
            S_TRANSLATE_SCALE,
            S_TRANSLATE_SCALE_RESULT: begin
                result_read_address_a = translation_h_address(2'd0);
            end
            S_TRANSLATE_MUL,
            S_TRANSLATE_RESULT: begin
                if (work_index < 3'd2) begin
                    result_read_address_a = translation_h_address(
                        work_index[1:0] + 1'b1);
                end else begin
                    result_read_address_a = column_address(3'd0);
                end
            end
            S_COLUMN_MUL,
            S_COLUMN_RESULT: begin
                if (work_index < 3'd5) begin
                    result_read_address_a = column_address(work_index + 1'b1);
                end else begin
                    result_read_address_a = 4'd3;
                    result_read_address_b = 4'd7;
                end
            end
            S_PRODUCT_SCALE, S_PRODUCT_RESULT: begin
                // Preserve the synchronous RAM prefetch while scaling the
                // preceding product; the next multiply consumes this value.
                if (product_is_column) begin
                    if (work_index < 3'd5)
                        result_read_address_a = column_address(work_index + 1'b1);
                    else begin
                        result_read_address_a = 4'd3;
                        result_read_address_b = 4'd7;
                    end
                end else if (work_index < 3'd2) begin
                    result_read_address_a = translation_h_address(work_index[1:0] + 1'b1);
                end else begin
                    result_read_address_a = column_address(3'd0);
                end
            end
            S_CROSS_PREFETCH: begin
                // The final column write updates B7 on the preceding edge.
                // Read it again before multiplying: simultaneous RAM reads
                // and writes return the preceding candidate's B7 value.
                result_read_address_a = 4'd3;
                result_read_address_b = 4'd7;
            end
            S_CROSS_FIRST,
            S_CROSS_SECOND: begin
                // Hold the second product's operands through SECOND so its
                // registered B value is also used by the +B correction.
                case (work_index)
                    3'd0: begin
                        result_read_address_a = 4'd6;
                        result_read_address_b = 4'd4;
                    end
                    3'd2: begin
                        result_read_address_a = 4'd0;
                        result_read_address_b = 4'd7;
                    end
                    default: begin
                        result_read_address_a = 4'd3;
                        result_read_address_b = 4'd1;
                    end
                endcase
            end
            S_CROSS_CORRECT,
            S_CROSS_RESULT: begin
                case (work_index)
                    3'd0: begin
                        result_read_address_a = 4'd6;
                        result_read_address_b = 4'd1;
                    end
                    3'd2: begin
                        result_read_address_a = 4'd0;
                        result_read_address_b = 4'd4;
                    end
                    default: begin end
                endcase
            end
            default: begin end
        endcase

        result_write_valid = 1'b0;
        result_write_address = 4'd0;
        result_write_data_a = 18'b0;
        result_write_data_b = 18'b0;
        if ((state == S_TRANSLATE_RESULT) && (pose_lambda_left_shift == 0)) begin
            result_write_valid = product_range_ok;
            result_write_address = 4'd9 + work_index;
            result_write_data_a = translated_q16[17:0];
            result_write_data_b = {{4{translated_q16[31]}},
                translated_q16[31:18]};
        end else if ((state == S_COLUMN_RESULT) && (pose_lambda_left_shift == 0)) begin
            result_write_valid = column_range_ok;
            result_write_address = column_address(work_index);
            result_write_data_a = column_q16;
            result_write_data_b = column_q16;
        end else if (state == S_PRODUCT_RESULT) begin
            if (product_is_column) begin
                result_write_valid = scaled_column_range_ok;
                result_write_address = column_address(work_index);
                result_write_data_a = scaled_column_q16;
                result_write_data_b = scaled_column_q16;
            end else begin
                result_write_valid = 1'b1;
                result_write_address = 4'd9 + work_index;
                result_write_data_a = scaled_translation_q16[17:0];
                result_write_data_b = {{4{scaled_translation_q16[31]}},
                    scaled_translation_q16[31:18]};
            end
        end else if (state == S_CROSS_RESULT) begin
            result_write_valid = cross_range_ok;
            case (work_index)
                3'd0: result_write_address = 4'd2;
                3'd2: result_write_address = 4'd5;
                default: result_write_address = 4'd8;
            endcase
            result_write_data_a = cross_q16;
            result_write_data_b = cross_q16;
        end
    end

    always_ff @(posedge clk) begin : pose_sequencer
        if (rst) begin
            state <= S_IDLE;
            work_index <= 3'b0;
            norm1_square_coarse <= 17'b0;
            norm2_square_coarse <= 17'b0;
            weak_norm_threshold <= 16'b0;
            pose_lambda_q10 <= 18'sb0;
            pose_lambda_left_shift <= 4'b0;
            scaled_product <= 41'sb0;
            product_shifts_left <= 4'b0;
            product_is_column <= 1'b0;
            translation_operand_q10 <= 18'sb0;
            translation_z_positive <= 1'b0;
            out_valid <= 1'b0;
            out_pose_valid <= 1'b0;
            out_weak_perspective <= 1'b0;
            out_conditioning_warning <= 1'b0;
            out_saturation <= 1'b0;
        end else begin
            if (out_valid && out_ready)
                out_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (in_valid && in_ready) begin
                        work_index <= 3'b0;
                        norm1_square_coarse <= 17'b0;
                        norm2_square_coarse <= 17'b0;
                        weak_norm_threshold <= 16'b0;
                        out_pose_valid <= 1'b0;
                        out_weak_perspective <= 1'b0;
                        out_conditioning_warning <= 1'b0;
                        out_saturation <= in_saturation;
                        translation_z_positive <= 1'b0;
                        if (!in_pose_enable
                                || (in_printed_width_q16[17:0] == 0)
                                || in_printed_width_q16[17]
                                || (|in_printed_width_q16[31:18])) begin
                            out_conditioning_warning <= 1'b1;
                            out_valid <= 1'b1;
                        end else begin
                            state <= S_NORM_PREFETCH;
                        end
                    end
                end

                S_NORM_PREFETCH: state <= S_NORM_MUL;

                S_NORM_MUL: begin
                    if (work_index == 5) begin
                        state <= S_RSQRT_REQ;
                    end else begin
                        work_index <= work_index + 1'b1;
                    end
                end

                S_RSQRT_REQ: begin
                    if (rsqrt_req_ready) begin
                        weak_norm_threshold
                            <= dsp_accumulator[37:22];
                        state <= S_WEAK_ACCUM;
                    end
                end

                S_WEAK_ACCUM: state <= S_WEAK_CHECK;

                S_WEAK_CHECK: begin
                    out_weak_perspective
                        <= (|weak_norm_threshold)
                            && ($unsigned(dsp_accumulator)
                                < {32'b0, weak_norm_threshold});
                    state <= S_RSQRT_WAIT;
                end

                // Measure conditioning after the common RMS normalization.
                // Small, distant tags can have H columns whose squared norms
                // are only 0..3 at this coarse precision; truncating those
                // norms before comparing them spuriously warns even for
                // equal columns. The normalized Q16 columns have a combined
                // squared norm near two, preserving about 13 fraction bits
                // here without widening the existing DSP operand registers.
                // The comparison's rounding uncertainty is below 0.0002 in
                // the column-norm ratio around the 7/8 warning boundary.
                S_CONDITION_PREFETCH: state <= S_CONDITION_NORM_MUL;

                S_CONDITION_NORM_MUL: begin
                    if (work_index == 3)
                        norm1_square_coarse <= dsp_accumulator[35:19];
                    if (work_index == 5) begin
                        state <= S_CONDITION_NORM_RESULT;
                    end else begin
                        work_index <= work_index + 1'b1;
                    end
                end

                S_CONDITION_NORM_RESULT: begin
                    norm2_square_coarse <= dsp_accumulator[35:19]
                        - norm1_square_coarse;
                    state <= S_CONDITION_MIN;
                end

                S_CONDITION_MIN: state <= S_CONDITION_MAX;

                S_CONDITION_MAX: state <= S_CONDITION_CHECK;

                S_CONDITION_CHECK: begin
                    out_conditioning_warning <= dsp_accumulator[47];
                    state <= S_CROSS_PREFETCH;
                    work_index <= 3'd0;
                end

                S_RSQRT_WAIT: begin
                    if (rsqrt_rsp_valid) begin
                        if ((rsqrt_rsp_lambda_q10 == 0)
                                || (|rsqrt_rsp_lambda_q10[18:17])
                                || (rsqrt_rsp_left_shift > 9)) begin
                            out_saturation <= 1'b1;
                            out_conditioning_warning <= 1'b1;
                            out_valid <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            pose_lambda_q10 <= rsqrt_rsp_lambda_q10[17:0];
                            pose_lambda_left_shift <= rsqrt_rsp_left_shift;
                            state <= S_TRANSLATE_SCALE;
                        end
                    end
                end

                S_TRANSLATE_SCALE: begin
                    state <= S_TRANSLATE_SCALE_RESULT;
                end

                S_TRANSLATE_SCALE_RESULT: begin
                    translation_operand_q10
                        <= rounded_translation_operand;
                    work_index <= 3'd0;
                    state <= S_TRANSLATE_MUL;
                end

                S_TRANSLATE_MUL: state <= S_TRANSLATE_RESULT;

                S_TRANSLATE_RESULT: begin
                    if (!product_range_ok) begin
                        out_saturation <= 1'b1;
                        out_conditioning_warning <= 1'b1;
                        out_valid <= 1'b1;
                        state <= S_IDLE;
                    end else if (pose_lambda_left_shift != 0) begin
                        scaled_product <= dsp_accumulator[41:1];
                        product_shifts_left <= pose_lambda_left_shift;
                        product_is_column <= 1'b0;
                        state <= S_PRODUCT_SCALE;
                    end else if (work_index == 2) begin
                        translation_z_positive <= !translated_q16[31]
                            && (translated_q16 != 0);
                        work_index <= 3'd0;
                        state <= S_COLUMN_MUL;
                    end else begin
                        work_index <= work_index + 1'b1;
                        state <= S_TRANSLATE_MUL;
                    end
                end

                S_COLUMN_MUL: state <= S_COLUMN_RESULT;

                S_COLUMN_RESULT: begin
                    if (!column_range_ok) begin
                        out_saturation <= 1'b1;
                        out_conditioning_warning <= 1'b1;
                        out_valid <= 1'b1;
                        state <= S_IDLE;
                    end else if (pose_lambda_left_shift != 0) begin
                        scaled_product <= dsp_accumulator[41:1];
                        product_shifts_left <= pose_lambda_left_shift;
                        product_is_column <= 1'b1;
                        state <= S_PRODUCT_SCALE;
                    end else if (work_index == 5) begin
                        work_index <= 3'd0;
                        state <= S_CONDITION_PREFETCH;
                    end else begin
                        work_index <= work_index + 1'b1;
                        state <= S_COLUMN_MUL;
                    end
                end

                S_PRODUCT_SCALE: begin
                    if (scaled_product[40] != scaled_product[39]) begin
                        out_saturation <= 1'b1;
                        out_conditioning_warning <= 1'b1;
                        out_valid <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        scaled_product <= scaled_product <<< 1;
                        product_shifts_left <= product_shifts_left - 1'b1;
                        if (product_shifts_left == 1)
                            state <= S_PRODUCT_RESULT;
                    end
                end

                S_PRODUCT_RESULT: begin
                    if (product_is_column) begin
                        if (!scaled_column_range_ok) begin
                            out_saturation <= 1'b1;
                            out_conditioning_warning <= 1'b1;
                            out_valid <= 1'b1;
                            state <= S_IDLE;
                        end else if (work_index == 5) begin
                            work_index <= 3'd0;
                            state <= S_CONDITION_PREFETCH;
                        end else begin
                            work_index <= work_index + 1'b1;
                            state <= S_COLUMN_MUL;
                        end
                    end else if (work_index == 2) begin
                        translation_z_positive <= !scaled_translation_q16[31]
                            && (scaled_translation_q16 != 0);
                        work_index <= 3'd0;
                        state <= S_COLUMN_MUL;
                    end else begin
                        work_index <= work_index + 1'b1;
                        state <= S_TRANSLATE_MUL;
                    end
                end

                S_CROSS_PREFETCH: state <= S_CROSS_FIRST;

                S_CROSS_FIRST: state <= S_CROSS_SECOND;

                S_CROSS_SECOND: state <= S_CROSS_CORRECT;

                S_CROSS_CORRECT: state <= S_CROSS_RESULT;

                S_CROSS_RESULT: begin
                    if (!cross_range_ok) begin
                        out_saturation <= 1'b1;
                        out_conditioning_warning <= 1'b1;
                        out_valid <= 1'b1;
                        state <= S_IDLE;
                    end else if (work_index == 4) begin
                        out_pose_valid <= translation_z_positive;
                        out_valid <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        work_index <= work_index + 2'd2;
                        state <= S_CROSS_FIRST;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
