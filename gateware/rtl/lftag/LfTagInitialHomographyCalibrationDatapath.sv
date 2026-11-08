// Combinational calibration/build datapath for the fused four-key frontend.
module LfTagInitialHomographyCalibrationDatapath (
    input  logic [71:0]                  in_key_x_q4_flat,
    input  logic [71:0]                  in_key_y_q4_flat,
    input  logic [1:0]                   in_scale,
    input  logic [31:0]                  in_inv_fx_q2_30,
    input  logic [31:0]                  in_inv_fy_q2_30,
    input  logic signed [31:0]           in_cx_q16,
    input  logic signed [31:0]           in_cy_q16,
    input  logic                         calibration_axis,
    input  logic [1:0]                   calibration_index,
    input  logic                         build_mode,
    input  logic [2:0]                   build_index,
    input  logic signed [17:0]           key_0_q16,
    input  logic signed [17:0]           key_1_q16,
    input  logic signed [17:0]           key_3_q16,
    input  logic signed [17:0]           cden,
    input  logic signed [17:0]           cg,
    input  logic signed [17:0]           ch,
    input  logic signed [47:0]           calibration_accumulator,

    output logic signed [17:0]           calibration_operand_a,
    output logic signed [17:0]           calibration_operand_b,
    output logic signed [37:0]           calibrated_coordinate,
    output logic                         calibration_input_range_error,
    output logic signed [37:0]           build_value
);
    function automatic logic signed [17:0] key_q4(
        input logic axis,
        input logic [1:0] index
    );
        if (axis)
            key_q4 = $signed(in_key_y_q4_flat[7'(index * 18) +: 18]);
        else
            key_q4 = $signed(in_key_x_q4_flat[7'(index * 18) +: 18]);
    endfunction

    function automatic logic signed [17:0] source_q6(
        input logic signed [17:0] coordinate_q4,
        input logic [1:0] scale,
        input logic axis
    );
        // CDF lifting retains the even sample: LL2 is at (4*x,4*y).
        // LL4 averages adjacent H4/V3 rows, at (16*x,16*y+4).
        case (scale)
            2'd0: source_q6 = coordinate_q4 <<< 2;
            2'd1: source_q6 = coordinate_q4 <<< 4;
            default: source_q6 = (coordinate_q4 <<< 6)
                + (axis ? 18'sd256 : 18'sd0);
        endcase
    endfunction

    always_comb begin
        logic axis;
        logic [1:0] point;
        logic signed [17:0] source_coordinate;
        logic signed [18:0] camera_delta;
        logic signed [18:0] principal_q6;
        logic [31:0] inverse_focal_q30;
        logic inverse_round_increment;
        logic [18:0] inverse_focal_q21;
        logic signed [17:0] build_difference_key;

        axis = calibration_axis;
        case (calibration_index[1:0])
            0: point = 2'd0;
            1: point = 2'd1;
            default: point = 2'd3;
        endcase
        source_coordinate = source_q6(key_q4(axis, point), in_scale, axis);
        principal_q6 = axis
            ? 19'($signed(in_cy_q16) >>> 10)
            : 19'($signed(in_cx_q16) >>> 10);
        camera_delta = {source_coordinate[17], source_coordinate}
            - principal_q6;
        inverse_focal_q30 = axis ? in_inv_fy_q2_30 : in_inv_fx_q2_30;
        inverse_round_increment = inverse_focal_q30[8]
            && ((|inverse_focal_q30[7:0]) || inverse_focal_q30[9]);
        inverse_focal_q21 = {1'b0, inverse_focal_q30[26:9]}
            + {{18{1'b0}}, inverse_round_increment};
        build_difference_key = build_index[1] ? key_3_q16 : key_1_q16;

        calibration_operand_a = camera_delta[17:0];
        calibration_operand_b = $signed(inverse_focal_q21[17:0]);
        if (build_mode) begin
            case (build_index)
                0, 2: begin
                    calibration_operand_a = build_difference_key - key_0_q16;
                    calibration_operand_b = cden;
                end
                1: begin
                    calibration_operand_a = key_1_q16;
                    calibration_operand_b = cg;
                end
                3: begin
                    calibration_operand_a = key_3_q16;
                    calibration_operand_b = ch;
                end
                default: begin
                    calibration_operand_a = key_0_q16;
                    calibration_operand_b = cden;
                end
            endcase
        end

        calibration_input_range_error = (camera_delta[18] != camera_delta[17])
            || inverse_focal_q21[18] || (|inverse_focal_q30[31:26]);
        calibrated_coordinate = 38'(
            $signed(calibration_accumulator) >>> 11
        );
        build_value = calibration_accumulator[37:0];
    end
endmodule
