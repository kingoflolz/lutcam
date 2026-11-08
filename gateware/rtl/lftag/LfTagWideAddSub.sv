// Wide add/subtract built from native arithmetic chunks. Each DSP performs
// an actual addition/subtraction; no DSP is used as a mux or pass-through.
// Doubling upper chunks incorporates the incoming carry/borrow without a
// separate fabric adder: (2A+1+2B+c)>>1 or (2A-2B-c)>>1.
module LfTagWideAddSub #(parameter integer WIDTH=33)(
    input logic signed [WIDTH-1:0] a,b,
    input logic subtract,
    output wire signed [WIDTH-1:0] result
);
    localparam bit NATIVE=1'b1
`ifndef SYNTHESIS
        // synthesis translate_off
        && 1'b0
        // synthesis translate_on
`endif
        ;
    generate if(NATIVE)begin:native
        localparam integer CHUNKS=(WIDTH+15)/16;
        wire [CHUNKS:0] carry;
        assign carry[0]=0;
        for(genvar n=0;n<CHUNKS;n=n+1)begin:chunk
            localparam integer BITS=(WIDTH-16*n>16)?16:WIDTH-16*n;
            wire sign_a=(n==CHUNKS-1)?a[WIDTH-1]:1'b0;
            wire sign_b=(n==CHUNKS-1)?b[WIDTH-1]:1'b0;
            wire [18:0] lhs;
            wire [17:0] rhs;
            wire [47:0] value,unused_cascade;
            wire unused_overflow;
            if(n==0)begin:low
                assign lhs={{(19-BITS){sign_a}},a[16*n+:BITS]};
                assign rhs={{(18-BITS){sign_b}},b[16*n+:BITS]};
                assign result[16*n+:BITS]=value[BITS-1:0];
                assign carry[n+1]=value[BITS];
            end else begin:upper
                assign lhs={{(18-BITS){sign_a}},a[16*n+:BITS],!subtract};
                assign rhs={{(17-BITS){sign_b}},b[16*n+:BITS],carry[n]};
                assign result[16*n+:BITS]=value[BITS:1];
                assign carry[n+1]=value[BITS+1];
            end
            EFX_DSP48 #(.MODE("NORMAL"),.SIGNED(1),.M_SEL("P"),.N_SEL("C"),
                .P_EXT("ALIGN_RIGHT"),.C_EXT("ALIGN_RIGHT"),.W_SEL("X"),.CASCOUT_SEL("W")) arithmetic(
                .A(lhs),.B(18'd1),.C(rhs),.CASCIN(48'd0),.OP({1'b0,subtract}),
                .SHIFT_ENA(1'b0),.CLK(1'b0),.CE(1'b0),.RST(1'b0),
                .O(value),.CASCOUT(unused_cascade),.OVFL(unused_overflow));
        end
    end else begin:portable
        assign result=subtract?a-b:a+b;
    end endgenerate
endmodule

// Narrow coordinate arithmetic fits the native multiplier/add input widths.
module LfTagSmallAddSub #(parameter integer WIDTH=16)(
    input wire signed [WIDTH-1:0] a,b,
    input wire subtract,
    output wire signed [WIDTH-1:0] result
);
    localparam bit NATIVE=1'b1
`ifndef SYNTHESIS
        // synthesis translate_off
        && 1'b0
        // synthesis translate_on
`endif
        ;
    generate if(NATIVE)begin:native
        wire [47:0] value,unused_cascade;
        wire unused_overflow;
        EFX_DSP48 #(.MODE("NORMAL"),.SIGNED(1),.M_SEL("P"),.N_SEL("C"),
            .P_EXT("ALIGN_RIGHT"),.C_EXT("ALIGN_RIGHT"),.W_SEL("X"),.CASCOUT_SEL("W")) arithmetic(
            .A({{(19-WIDTH){a[WIDTH-1]}},a}),.B(18'd1),.C({{(18-WIDTH){b[WIDTH-1]}},b}),
            .CASCIN(48'd0),.OP({1'b0,subtract}),.SHIFT_ENA(1'b0),.CLK(1'b0),.CE(1'b0),.RST(1'b0),
            .O(value),.CASCOUT(unused_cascade),.OVFL(unused_overflow));
        assign result=value[WIDTH-1:0];
    end else begin:portable
        assign result=subtract?a-b:a+b;
    end endgenerate
endmodule
