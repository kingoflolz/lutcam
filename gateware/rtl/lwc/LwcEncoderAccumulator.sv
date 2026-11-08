`default_nettype none
// Enabled signed increment with synchronous clear and modulo output width.
// The W feedback path performs the addition inside the DSP; no plain state
// is stored in a DSP. Ordinary simulation retains the same portable update.
module LwcEncoderAccumulator #(parameter integer WIDTH = 16)(
    input wire clk, input wire reset, input wire enable,
    input wire signed [17:0] increment,
    output wire [WIDTH-1:0] value
);
    localparam bit USE_NATIVE_DSP = 1'b1
`ifndef SYNTHESIS
        // synthesis translate_off
        && 1'b0
        // synthesis translate_on
`endif
        ;
    generate if (USE_NATIVE_DSP) begin : native
        wire [47:0] accumulator;
        wire [47:0] unused_cascade;
        wire unused_overflow;
        wire unused_accumulator_high = ^accumulator[47:WIDTH];
        EFX_DSP48 #(
            .MODE("NORMAL"), .SIGNED(1'b1), .M_SEL("C"), .N_SEL("W"),
            .W_SEL("X"), .W_REG(1'b1), .RST_SYNC(1'b1)
        ) add_accumulator (
            .A(19'd0), .B(18'd0), .C(increment), .CASCIN(48'd0),
            .OP(2'b00), .SHIFT_ENA(1'b0), .CLK(clk), .CE(enable),
            .RST(reset), .O(accumulator),
            .CASCOUT(unused_cascade), .OVFL(unused_overflow)
        );
        assign value = accumulator[WIDTH-1:0];
    end else begin : portable
        logic [WIDTH-1:0] accumulator;
        always_ff @(posedge clk) begin
            if (reset) accumulator <= '0;
            else if (enable) accumulator <= WIDTH'($signed({1'b0,accumulator}) + increment);
        end
        assign value = accumulator;
    end endgenerate
endmodule
`default_nettype wire
