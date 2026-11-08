// Exact modulo-WIDTH addition with low fabric carry and one unregistered DSP.
module S5HighChunkAdd #(parameter integer WIDTH=29, parameter bit USE_DSP=1'b1)(
    input wire [WIDTH-1:0] x,y,
    output wire [WIDTH-1:0] result
);
    localparam bit USE_NATIVE_HIGH = USE_DSP && (WIDTH > 17)
`ifndef SYNTHESIS
        // synthesis translate_off
        && 1'b0
        // synthesis translate_on
`endif
        ;
    generate if (USE_NATIVE_HIGH) begin : native
        localparam integer LOW_BITS=WIDTH-17;
        wire [LOW_BITS:0] low_sum={1'b0,x[LOW_BITS-1:0]}+{1'b0,y[LOW_BITS-1:0]};
        /* verilator lint_off UNUSEDSIGNAL */
        wire [47:0] high_sum;
        /* verilator lint_on UNUSEDSIGNAL */
        wire [47:0] unused_cascade;
        wire unused_overflow;
        // floor((2*xhi+1 + 2*yhi+carry)/2) = xhi+yhi+carry.
        EFX_DSP48 #(
            .MODE("NORMAL"),.SIGNED(0),.M_SEL("P"),.N_SEL("C"),
            .P_EXT("ALIGN_RIGHT"),.C_EXT("ALIGN_RIGHT"),
            .W_SEL("X"),.CASCOUT_SEL("W"),
            .A_REG(0),.B_REG(0),.C_REG(0),.P_REG(0),.OP_REG(0),
            .W_REG(0),.O_REG(0),.SHIFTER(0)
        ) arithmetic (
            .A({1'b0,x[WIDTH-1:LOW_BITS],1'b1}),.B(18'd1),
            .C({y[WIDTH-1:LOW_BITS],low_sum[LOW_BITS]}),.CASCIN(48'd0),
            .OP(2'b00),.SHIFT_ENA(1'b0),.CLK(1'b0),.CE(1'b0),.RST(1'b0),
            .O(high_sum),.CASCOUT(unused_cascade),.OVFL(unused_overflow)
        );
        assign result={high_sum[17:1],low_sum[LOW_BITS-1:0]};
    end else begin : portable
        assign result=x+y;
    end endgenerate
endmodule
