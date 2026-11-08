// Fused compact four-key LFTag homography frontend.
//
// Camera calibration, unit-square map construction, canonical-hypothesis
// composition, and common power-of-two scaling are one sequential datapath.
// The calibration and composition products each accumulate in an existing
// DSP feedback lane, avoiding duplicate wide fabric adders.
module LfTagInitialHomographyFused #(
    parameter bit EXTERNAL_CONFIG = 1'b0
) (
    input  logic                         clk,
    input  logic                         rst,

    input  logic                         in_valid,
    output logic                         in_ready,
    input  logic [71:0]                  in_key_x_q4_flat,
    input  logic [71:0]                  in_key_y_q4_flat,
    input  logic signed [17:0]           in_cden,
    input  logic signed [17:0]           in_cg,
    input  logic signed [17:0]           in_ch,
    input  logic [3:0]                   in_hypothesis,
    input  logic [1:0]                   in_scale,
    input  logic [31:0]                  in_inv_fx_q2_30,
    input  logic [31:0]                  in_inv_fy_q2_30,
    input  logic signed [31:0]           in_cx_q16,
    input  logic signed [31:0]           in_cy_q16,
    input  logic                         in_saturation,

    output logic                         config_read_valid,
    output logic [1:0]                   config_read_address,
    input  logic                         config_read_ready,
    input  logic                         config_response_valid,
    input  logic [31:0]                  config_read_data,

    output logic                         out_valid,
    input  logic                         out_ready,
    output logic                         result_write_valid,
    output logic [3:0]                   result_write_address,
    output logic signed [17:0]           result_write_data,
    output logic                         out_invalid,
    output logic                         out_saturation,
    output logic [6:0]                   out_cycles,

    output logic [6:0]                   inverse_read_address,
    input  logic signed [17:0]           inverse_read_data_a,
    input  logic signed [17:0]           inverse_read_data_b
);
    typedef enum logic [3:0] {
        S_IDLE,
        S_INVERSE_FETCH,
        S_DENOMINATOR,
        S_DENOMINATOR_RESULT,
        S_DENOMINATOR_NORMALIZE,
        S_CALIBRATE_X,
        S_CALIBRATE_Y,
        S_CALIBRATE_RESULT,
        S_BUILD_X,
        S_BUILD_Y,
        S_BUILD_RESULT,
        S_COEFFICIENT,
        S_COEFFICIENT_RESULT,
        S_COEFFICIENT_NORMALIZE,
        S_OUTPUT,
        S_CONFIG_FETCH
    } state_t;

    state_t state;
    // One active axis pair, fetched from the stable shared configuration bank.
    logic [31:0] current_inverse_focal;
    logic signed [31:0] current_principal;
    logic config_wait;
    logic config_word;
    logic config_axis;
    logic config_abort;

    // Configuration payload uses fabric registers; DSPs are reserved for arithmetic.
    always_ff @(posedge clk) begin : current_inverse_focal_storage
        if (!rst && (state == S_CONFIG_FETCH) && config_wait
            && config_response_valid && !config_word) current_inverse_focal <= config_read_data;
    end

    assign config_read_valid = EXTERNAL_CONFIG
        && (state == S_CONFIG_FETCH) && !config_wait;
    assign config_read_address = {config_word, config_axis};
    logic [1:0] calibration_index;
    logic [2:0] build_index;
    logic [2:0] coefficient_index;
    logic [6:0] cycle_counter;
    logic signed [17:0] key_q16 [0:2];
    logic [53:0] d_row_q16_flat;
    // Three signed 18x18 products fit in 37 signed bits. Reuse one serial
    // shifter for all nine composition results, leaving the DSP feedback
    // accumulators intact. The denominator chooses one common power of two;
    // it must be chosen after canonical-hypothesis composition, which can
    // move an otherwise normalized input denominator across a scale boundary.
    logic signed [36:0] scale_work;
    logic signed [5:0] common_shift;
    logic signed [5:0] coefficient_shift;
    logic denominator_normalized;
    logic coefficient_shift_overflow;
    logic signed [22:0] common_scaled_value;
    logic common_scaled_range_error;
    logic [6:0] next_cycle_count;

    logic signed [17:0] calibration_operand_a;
    logic signed [17:0] calibration_operand_b;
    logic signed [17:0] mul_a;
    logic signed [17:0] mul_b;
    logic signed [37:0] calibrated_coordinate;
    logic calibration_input_range_error;
    logic signed [37:0] build_value;
    logic [1:0] arithmetic_stage;
    logic inverse_fetch_denominator;
    logic [1:0] inverse_group;
    logic normalize_requires_second_stage;
    logic calibration_dsp_enable;
    logic calibration_dsp_clear;
    logic normalize_dsp_enable;
    logic normalize_dsp_clear;
    (* syn_use_dsp = "yes" *) logic signed [47:0] shared_accumulator;
    logic signed [17:0] shared_operand_a;
    logic signed [17:0] shared_operand_b;
    wire signed [47:0] calibration_accumulator = shared_accumulator;
    // Retain the full MAC result for verification; normalization uses [36:0].
    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [47:0] normalize_accumulator = shared_accumulator;
    /* verilator lint_on UNUSEDSIGNAL */
    assign shared_operand_a = calibration_dsp_enable ? calibration_operand_a : mul_a;
    assign shared_operand_b = calibration_dsp_enable ? calibration_operand_b : mul_b;

    LfTagInitialHomographyCalibrationDatapath calibration_datapath (
        .in_key_x_q4_flat,
        .in_key_y_q4_flat,
        .in_scale,
        .in_inv_fx_q2_30(EXTERNAL_CONFIG ? current_inverse_focal : in_inv_fx_q2_30),
        .in_inv_fy_q2_30(EXTERNAL_CONFIG ? current_inverse_focal : in_inv_fy_q2_30),
        .in_cx_q16(EXTERNAL_CONFIG ? current_principal : in_cx_q16),
        .in_cy_q16(EXTERNAL_CONFIG ? current_principal : in_cy_q16),
        .calibration_axis(state == S_CALIBRATE_Y),
        .calibration_index,
        .build_mode((state == S_BUILD_X) || (state == S_BUILD_Y)),
        .build_index,
        .key_0_q16(key_q16[0]),
        .key_1_q16(key_q16[1]),
        .key_3_q16(key_q16[2]),
        .cden(in_cden),
        .cg(in_cg),
        .ch(in_ch),
        .calibration_accumulator,
        .calibration_operand_a,
        .calibration_operand_b,
        .calibrated_coordinate,
        .calibration_input_range_error,
        .build_value
    );

    LfTagInitialHomographyNormalizeDatapath normalize_datapath (
        .d_row_q16_flat,
        .coefficient_index,
        .denominator_mode(state == S_DENOMINATOR),
        .coefficient_mode(state == S_COEFFICIENT),
        .arithmetic_stage,
        .inverse_pair_a(inverse_read_data_a),
        .inverse_pair_b(inverse_read_data_b),
        .mul_a,
        .mul_b,
        .requires_second_stage(normalize_requires_second_stage)
    );

    assign in_ready = state == S_IDLE;
    assign out_valid = state == S_OUTPUT;

    // Each hypothesis occupies four paired words in the reserved upper half
    // of the shared pose-result RAM banks. The extra group supplies q21 for
    // the second arithmetic stage of the middle-column coefficients.
    always_comb begin
        case (coefficient_index)
            3'd0, 3'd3, 3'd6: inverse_group = 2'd0;
            3'd1, 3'd4, 3'd7: inverse_group = 2'd1;
            default: inverse_group = 2'd2;
        endcase
        if ((state == S_INVERSE_FETCH && inverse_fetch_denominator)
                || (state == S_DENOMINATOR)) begin
            inverse_group = 2'd2;
        end else if ((state == S_COEFFICIENT) && (arithmetic_stage == 1)
                && ((coefficient_index == 3'd1)
                    || (coefficient_index == 3'd4)
                    || (coefficient_index == 3'd7))) begin
            inverse_group = 2'd3;
        end
        inverse_read_address = {1'b1, in_hypothesis, inverse_group};
    end

    assign common_scaled_value = 23'($signed(scale_work) >>> 14);
    assign common_scaled_range_error = (common_scaled_value < -23'sd131072)
        || (common_scaled_value > 23'sd131071);
    assign denominator_normalized = (scale_work >= 37'sd536870912)
        && (scale_work < 37'sd2147483648);
    assign coefficient_shift_overflow = (coefficient_shift < 0)
        && (scale_work[36] != scale_work[35]);
    assign result_write_valid = ((state == S_DENOMINATOR_NORMALIZE)
            && denominator_normalized)
        || ((state == S_COEFFICIENT_NORMALIZE)
            && (coefficient_shift == 0) && !common_scaled_range_error);
    assign result_write_address = (state == S_DENOMINATOR_NORMALIZE)
        ? 4'd8 : {1'b0, coefficient_index};
    assign result_write_data = common_scaled_value[17:0];
    assign next_cycle_count = (&cycle_counter)
        ? cycle_counter : cycle_counter + 1'b1;

    always_comb begin : dsp_feedback_control
        calibration_dsp_enable = (state == S_CALIBRATE_X)
            || (state == S_CALIBRATE_Y) || (state == S_BUILD_X)
            || (state == S_BUILD_Y);
        calibration_dsp_clear = (state == S_CALIBRATE_RESULT)
            || (state == S_BUILD_RESULT);
        normalize_dsp_enable = (state == S_DENOMINATOR)
            || (state == S_COEFFICIENT);
        normalize_dsp_clear = (state == S_DENOMINATOR_RESULT)
            || (state == S_COEFFICIENT_RESULT);
    end

    // Calibration/build and composition execute in disjoint states. Every
    // result state consumes and clears the previous sum before another lane
    // starts, so one feedback DSP preserves the original arithmetic and cycles.
    always_ff @(posedge clk) begin : homography_dsp_feedback
        if (rst || calibration_dsp_clear || normalize_dsp_clear) begin
            shared_accumulator <= 48'sb0;
        end else if (calibration_dsp_enable || normalize_dsp_enable) begin
            shared_accumulator <= shared_accumulator
                + shared_operand_a * shared_operand_b;
        end
    end

    always_ff @(posedge clk) begin : homography_sequencer
        logic signed [37:0] scaled_build;
        logic build_range_error;
        logic coordinate_range_error;

        scaled_build = $signed(build_value) >>> 18;
        build_range_error = (scaled_build < -38'sd131072)
            || (scaled_build > 38'sd131071);
        // The original result-state check selects X even after a Y multiply.
        // Its three X input flags are already sticky before the Y pass. Avoid
        // rechecking those X coordinates against the newly loaded Y pair.
        coordinate_range_error = (calibration_input_range_error
                && (!EXTERNAL_CONFIG || (coefficient_index == 0)))
            || (calibrated_coordinate < -38'sd131072)
            || (calibrated_coordinate > 38'sd131071);
        if (rst) begin
            state <= S_IDLE;
            config_wait <= 1'b0;
            config_word <= 1'b0;
            config_axis <= 1'b0;
            config_abort <= 1'b0;
            calibration_index <= 2'b0;
            build_index <= 3'b0;
            coefficient_index <= 3'b0;
            arithmetic_stage <= 2'b0;
            inverse_fetch_denominator <= 1'b0;
            cycle_counter <= 7'b0;
            scale_work <= 37'sb0;
            common_shift <= 6'sb0;
            coefficient_shift <= 6'sb0;
            out_invalid <= 1'b0;
            out_saturation <= 1'b0;
            out_cycles <= 7'b0;
        end else begin
            if (state != S_IDLE && state != S_OUTPUT)
                cycle_counter <= next_cycle_count;

            case (state)
                S_IDLE: begin
                    if (in_valid) begin
                        calibration_index <= 2'b0;
                        build_index <= 3'b0;
                        coefficient_index <= 3'b0;
                        arithmetic_stage <= 2'b0;
                        inverse_fetch_denominator <= 1'b1;
                        cycle_counter <= 7'b0;
                        out_invalid <= 1'b0;
                        out_saturation <= in_saturation || (in_scale == 2'd3)
                            || (in_cden == 0)
                            || (!EXTERNAL_CONFIG && (
                                (in_inv_fx_q2_30[31:26] != 0)
                                || (in_inv_fy_q2_30[31:26] != 0)
                                || (in_cx_q16[31:27] != {5{in_cx_q16[27]}})
                                || (in_cy_q16[31:27] != {5{in_cy_q16[27]}})));
                        out_cycles <= 7'b0;
                        d_row_q16_flat[17:0] <= $signed(in_cg) >>> 2;
                        d_row_q16_flat[35:18] <= $signed(in_ch) >>> 2;
                        d_row_q16_flat[53:36] <= $signed(in_cden) >>> 2;
                        config_wait <= 1'b0;
                        config_word <= 1'b0;
                        config_axis <= 1'b0;
                        config_abort <= 1'b0;
                        if (EXTERNAL_CONFIG)
                            state <= S_CONFIG_FETCH;
                        else
                            state <= S_INVERSE_FETCH;
                    end
                end

                S_CONFIG_FETCH: begin
                    if (!config_wait) begin
                        if (config_read_ready)
                            config_wait <= 1'b1;
                    end else if (config_response_valid) begin
                        config_wait <= 1'b0;
                        if (!config_word) begin
                            out_saturation <= out_saturation
                                || (config_read_data[31:26] != 0);
                            config_word <= 1'b1;
                        end else begin
                            current_principal <= $signed(config_read_data);
                            out_saturation <= out_saturation
                                || (config_read_data[31:27]
                                    != {5{config_read_data[27]}});
                            if (config_abort) begin
                                out_cycles <= next_cycle_count;
                                state <= S_OUTPUT;
                            end else begin
                                if (config_axis)
                                    state <= S_CALIBRATE_Y;
                                else
                                    state <= S_INVERSE_FETCH;
                            end
                        end
                    end
                end

                S_INVERSE_FETCH: begin
                    if (inverse_fetch_denominator) begin
                        inverse_fetch_denominator <= 1'b0;
                        state <= S_DENOMINATOR;
                    end else begin
                        state <= S_COEFFICIENT;
                    end
                end

                S_DENOMINATOR: begin
                    if (normalize_requires_second_stage) begin
                        arithmetic_stage <= arithmetic_stage + 1'b1;
                    end else begin
                        state <= S_DENOMINATOR_RESULT;
                    end
                end

                S_DENOMINATOR_RESULT: begin
                    arithmetic_stage <= 2'b0;
                    coefficient_index <= 3'd0;
                    calibration_index <= 2'd0;
                    scale_work <= normalize_accumulator[36:0];
                    common_shift <= 6'sb0;
                    state <= S_DENOMINATOR_NORMALIZE;
                end

                S_DENOMINATOR_NORMALIZE: begin
                    if (scale_work <= 0) begin
                        out_invalid <= 1'b1;
                        if (EXTERNAL_CONFIG) begin
                            // Even an unusable denominator reports the raw
                            // range flags from both axes, as the direct path does.
                            config_wait <= 1'b0;
                            config_word <= 1'b0;
                            config_axis <= 1'b1;
                            config_abort <= 1'b1;
                            state <= S_CONFIG_FETCH;
                        end else begin
                            out_cycles <= next_cycle_count;
                            state <= S_OUTPUT;
                        end
                    end else if (denominator_normalized) begin
                        state <= S_CALIBRATE_X;
                    end else if (scale_work < 37'sd536870912) begin
                        scale_work <= scale_work <<< 1;
                        common_shift <= common_shift - 1'b1;
                    end else begin
                        scale_work <= scale_work >>> 1;
                        common_shift <= common_shift + 1'b1;
                    end
                end

                S_CALIBRATE_X, S_CALIBRATE_Y: begin
                    state <= S_CALIBRATE_RESULT;
                end

                S_CALIBRATE_RESULT: begin
                    case (calibration_index)
                        0: key_q16[0] <= calibrated_coordinate[17:0];
                        1: key_q16[1] <= calibrated_coordinate[17:0];
                        default: key_q16[2]
                            <= calibrated_coordinate[17:0];
                    endcase
                    out_saturation <= out_saturation || coordinate_range_error;
                    if (calibration_index == 2) begin
                        build_index <= 3'b0;
                        if (coefficient_index == 0)
                            state <= S_BUILD_X;
                        else
                            state <= S_BUILD_Y;
                    end else begin
                        calibration_index <= calibration_index + 1'b1;
                        if (coefficient_index == 0)
                            state <= S_CALIBRATE_X;
                        else
                            state <= S_CALIBRATE_Y;
                    end
                end

                S_BUILD_X, S_BUILD_Y: begin
                    case (build_index)
                        0: build_index <= 1;
                        1: state <= S_BUILD_RESULT;
                        2: build_index <= 3;
                        3: state <= S_BUILD_RESULT;
                        default: state <= S_BUILD_RESULT;
                    endcase
                end

                S_BUILD_RESULT: begin
                    out_saturation <= out_saturation || build_range_error;
                    case (build_index)
                        1: begin
                            d_row_q16_flat[17:0] <= scaled_build[17:0];
                            build_index <= 2;
                            if (coefficient_index == 0)
                                state <= S_BUILD_X;
                            else
                                state <= S_BUILD_Y;
                        end
                        3: begin
                            d_row_q16_flat[35:18] <= scaled_build[17:0];
                            build_index <= 4;
                            if (coefficient_index == 0)
                                state <= S_BUILD_X;
                            else
                                state <= S_BUILD_Y;
                        end
                        default: begin
                            d_row_q16_flat[53:36] <= scaled_build[17:0];
                            inverse_fetch_denominator <= 1'b0;
                            state <= S_INVERSE_FETCH;
                        end
                    endcase
                end

                S_COEFFICIENT: begin
                    if (normalize_requires_second_stage) begin
                        arithmetic_stage <= arithmetic_stage + 1'b1;
                    end else begin
                        state <= S_COEFFICIENT_RESULT;
                    end
                end

                S_COEFFICIENT_RESULT: begin
                    arithmetic_stage <= 2'b0;
                    scale_work <= normalize_accumulator[36:0];
                    coefficient_shift <= common_shift;
                    state <= S_COEFFICIENT_NORMALIZE;
                end

                S_COEFFICIENT_NORMALIZE: begin
                    if (coefficient_shift_overflow
                            || ((coefficient_shift == 0)
                                && common_scaled_range_error)) begin
                        out_invalid <= 1'b1;
                        out_saturation <= 1'b1;
                        if (EXTERNAL_CONFIG && (coefficient_index < 3)) begin
                            config_wait <= 1'b0;
                            config_word <= 1'b0;
                            config_axis <= 1'b1;
                            config_abort <= 1'b1;
                            state <= S_CONFIG_FETCH;
                        end else begin
                            out_cycles <= next_cycle_count;
                            state <= S_OUTPUT;
                        end
                    end else if (coefficient_shift < 0) begin
                        scale_work <= scale_work <<< 1;
                        coefficient_shift <= coefficient_shift + 1'b1;
                    end else if (coefficient_shift > 0) begin
                        scale_work <= scale_work >>> 1;
                        coefficient_shift <= coefficient_shift - 1'b1;
                    end else if (coefficient_index == 3'd2) begin
                        coefficient_index <= 3'd3;
                        calibration_index <= 2'd0;
                        if (EXTERNAL_CONFIG) begin
                            config_wait <= 1'b0;
                            config_word <= 1'b0;
                            config_axis <= 1'b1;
                            config_abort <= 1'b0;
                            state <= S_CONFIG_FETCH;
                        end else begin
                            state <= S_CALIBRATE_Y;
                        end
                    end else if (coefficient_index == 3'd5) begin
                        coefficient_index <= 3'd6;
                        d_row_q16_flat[17:0] <= $signed(in_cg) >>> 2;
                        d_row_q16_flat[35:18] <= $signed(in_ch) >>> 2;
                        d_row_q16_flat[53:36] <= $signed(in_cden) >>> 2;
                        state <= S_INVERSE_FETCH;
                    end else if (coefficient_index == 3'd7) begin
                        out_cycles <= next_cycle_count;
                        state <= S_OUTPUT;
                    end else begin
                        coefficient_index <= coefficient_index + 1'b1;
                        state <= S_INVERSE_FETCH;
                    end
                end

                S_OUTPUT: begin
                    if (out_ready)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
