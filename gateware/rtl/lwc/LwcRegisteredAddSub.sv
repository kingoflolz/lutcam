`default_nettype none
// Single-cycle signed add/sub, truncated modulo WIDTH.
// A*1 uses the ordinary multiplier path and C uses the signed right-aligned
// operand. W replaces the original enabled async-reset fabric register.
module LwcRegisteredAddSub #(parameter integer WIDTH=14, parameter bit SUBTRACT=0)(
    input wire clk, input wire rst_n, input wire enable,
    input wire signed [WIDTH-1:0] a, b,
    output logic signed [WIDTH-1:0] result
);
    // Efinity does not predefine SYNTHESIS. Its documented translation
    // directives retain the native path for hardware; normal simulators keep
    // the portable path. Defining SYNTHESIS explicitly selects the native
    // model in regression tests. Wider generic stages retain fabric arithmetic.
    localparam bit USE_NATIVE_DSP = (WIDTH <= 18)
`ifndef SYNTHESIS
        // synthesis translate_off
        && 1'b0
        // synthesis translate_on
`endif
        ;
    generate if (USE_NATIVE_DSP) begin : native
    wire [47:0] dsp_result;
    EFX_DSP48 #(
        .MODE("NORMAL"), .SIGNED(1), .M_SEL("P"), .N_SEL("C"),
        .P_EXT("ALIGN_RIGHT"), .C_EXT("ALIGN_RIGHT"),
        .W_SEL("X"), .CASCOUT_SEL("W"),
        .A_REG(0), .B_REG(0), .C_REG(0), .P_REG(0), .OP_REG(0),
        .W_REG(1), .O_REG(0), .SHIFTER(0), .RST_SYNC(0),
        .RST_POLARITY(0)
    ) arithmetic (
        .A({{(19-WIDTH){a[WIDTH-1]}},a}), .B(18'd1),
        .C({{(18-WIDTH){b[WIDTH-1]}},b}), .CASCIN(48'd0),
        .OP({1'b0,SUBTRACT}), .SHIFT_ENA(1'b0),
        .CLK(clk), .CE(enable), .RST(rst_n),
        .O(dsp_result), .CASCOUT(), .OVFL()
    );
    assign result=dsp_result[WIDTH-1:0];
    end else begin : portable
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n)
                result <= '0;
            else if (enable)
                result <= SUBTRACT ? a-b : a+b;
        end
    end endgenerate
endmodule
`default_nettype wire
