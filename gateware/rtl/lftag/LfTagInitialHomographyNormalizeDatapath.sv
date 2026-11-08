// Combinational canonical-composition datapath for the fused four-key frontend.
module LfTagInitialHomographyNormalizeDatapath (
    input  logic [53:0]                  d_row_q16_flat,
    input  logic [2:0]                   coefficient_index,
    input  logic                         denominator_mode,
    input  logic                         coefficient_mode,
    input  logic [1:0]                   arithmetic_stage,
    input  logic signed [17:0]           inverse_pair_a,
    input  logic signed [17:0]           inverse_pair_b,

    output logic [17:0]                  mul_a,
    output logic [17:0]                  mul_b,
    output logic                         requires_second_stage
);
    logic [1:0] coefficient_column;
    logic signed [17:0] coefficient_d0;
    logic signed [17:0] coefficient_d1;
    logic signed [17:0] coefficient_d2;
    function automatic logic signed [17:0] d_at(input logic [1:0] index);
        d_at = $signed(d_row_q16_flat[6'(index * 18) +: 18]);
    endfunction

    always_comb begin
        coefficient_column = 2'd0;
        coefficient_d0 = d_at(0);
        coefficient_d1 = d_at(1);
        coefficient_d2 = d_at(2);
        case (coefficient_index)
            3'd0, 3'd3, 3'd6: coefficient_column = 2'd0;
            3'd1, 3'd4, 3'd7: coefficient_column = 2'd1;
            default: coefficient_column = 2'd2;
        endcase
    end

    always_comb begin
        mul_a = 18'sb0;
        mul_b = 18'sb0;

        if (denominator_mode) begin
            case (arithmetic_stage)
                0: begin
                    mul_a = d_at(0);
                    mul_b = inverse_pair_a;
                end
                1: begin
                    mul_a = d_at(1);
                    mul_b = inverse_pair_b;
                end
                default: begin
                    mul_a = d_at(2);
                    mul_b = 18'sd65536;
                end
            endcase
        end else if (coefficient_mode) begin
            case (arithmetic_stage)
                0: begin
                    mul_a = coefficient_d0;
                    mul_b = inverse_pair_a;
                end
                1: begin
                    if (coefficient_column == 0)
                        mul_a = coefficient_d2;
                    else
                        mul_a = coefficient_d1;
                    mul_b = inverse_pair_b;
                end
                default: begin
                    mul_a = coefficient_d2;
                    if (coefficient_column == 1)
                        mul_b = inverse_pair_a;
                    else
                        mul_b = 18'sd65536;
                end
            endcase
        end
    end

    always_comb begin
        logic final_stage;

        final_stage = (arithmetic_stage == 2)
            || (coefficient_mode && (coefficient_column == 0)
                && (arithmetic_stage == 1));
        requires_second_stage = (denominator_mode || coefficient_mode)
            && !final_stage;
    end
endmodule
