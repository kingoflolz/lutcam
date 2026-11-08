`default_nettype none

// Enabled modulo counter: asynchronous global reset, synchronous clear, then
// increment. The clear input must not be combined into the asynchronous reset.
module Ov9281Counter #(
    parameter integer WIDTH = 14,
    parameter bit USE_DSP = 1'b0
) (
    input wire clk, reset_n, clear, enable,
    output logic [WIDTH-1:0] value
);
    // Efinity does not predefine SYNTHESIS. Its translation directives retain
    // the native branch in hardware while ordinary simulators use portable
    // arithmetic. Explicit SYNTHESIS enables native-model regression tests.
    localparam bit USE_NATIVE_DSP = USE_DSP && (WIDTH <= 19)
`ifndef SYNTHESIS
        // synthesis translate_off
        && 1'b0
        // synthesis translate_on
`endif
        ;
    generate if (USE_NATIVE_DSP) begin : native
        wire [47:0] accumulator;
        // A * 1 + 1 increments; A * 0 + 0 clears on the clock edge. W holds
        // the count, and low-width feedback retains the exact modulo wrap.
        EFX_DSP48 #(
            .MODE("NORMAL"), .SIGNED(0), .M_SEL("P"), .N_SEL("C"),
            .P_EXT("ALIGN_RIGHT"), .C_EXT("ALIGN_RIGHT"),
            .W_SEL("X"), .CASCOUT_SEL("W"),
            .A_REG(0), .B_REG(0), .C_REG(0), .P_REG(0), .OP_REG(0),
            .W_REG(1), .O_REG(0), .SHIFTER(0),
            .RST_SYNC(0), .RST_POLARITY(0)
        ) counter (
            .A({{(19-WIDTH){1'b0}}, value}),
            .B({17'd0, !clear}), .C({17'd0, !clear}),
            .CASCIN(48'd0), .OP(2'b00), .SHIFT_ENA(1'b0),
            .CLK(clk), .CE(clear || enable), .RST(reset_n),
            .O(accumulator), .CASCOUT(), .OVFL()
        );
        assign value = accumulator[WIDTH-1:0];
    end else begin : portable
        always_ff @(posedge clk or negedge reset_n) begin
            if (!reset_n)
                value <= '0;
            else if (clear)
                value <= '0;
            else if (enable)
                value <= value + 1'b1;
        end
    end endgenerate
endmodule

`default_nettype wire
