`timescale 1ns/1ps

// The reference is a widened ordinary unsigned sum, independent of DSP packing.
// No clock exists in this testbench: both outputs must follow changing inputs.
module S5HighAddCase #(parameter integer WIDTH=29,parameter bit FORCE_FABRIC=0)(output logic done=0);
    localparam integer LOW_BITS=(WIDTH>17) ? WIDTH-17 : 1;
    localparam integer LOW_RANGE=(LOW_BITS<4) ? (1<<LOW_BITS) : 16;
    localparam integer SMALL_RANGE=(WIDTH<8) ? (1<<WIDTH) : 256;
    localparam logic [WIDTH-1:0] ALL_ONES='1;
    localparam logic [WIDTH-1:0] LOW_MASK=(WIDTH'(1)<<LOW_BITS)-WIDTH'(1);
    logic [WIDTH-1:0] x=0,y=0;
    wire [WIDTH-1:0] native_result,portable_result;
    S5HighChunkAdd #(.WIDTH(WIDTH),.USE_DSP(!FORCE_FABRIC)) native_dut(.x,.y,.result(native_result));
    S5HighChunkAddPortable #(.WIDTH(WIDTH)) portable_dut(.x,.y,.result(portable_result));
    wire native_clock_held;
`ifdef S5_HIGH_ADD_NO_VENDOR
    // No DSP instance exists in this profile. Avoid hierarchical references
    // to an undeclared primitive before Verilator eliminates dead branches.
    assign native_clock_held=1'b1;
    initial assert(FORCE_FABRIC) else $fatal(1,"no-vendor profile must force fabric");
`else
    generate if(WIDTH>17 && !FORCE_FABRIC) begin : native_checks
        assign native_clock_held = native_dut.native.arithmetic.CLK===1'b0;
        initial begin
            assert(native_dut.native.arithmetic.A_REG==0);
            assert(native_dut.native.arithmetic.B_REG==0);
            assert(native_dut.native.arithmetic.C_REG==0);
            assert(native_dut.native.arithmetic.P_REG==0);
            assert(native_dut.native.arithmetic.OP_REG==0);
            assert(native_dut.native.arithmetic.W_REG==0);
            assert(native_dut.native.arithmetic.O_REG==0);
        end
    end else begin : fallback_checks
        assign native_clock_held=1'b1;
    end endgenerate
`endif

    integer checks=0,directed=0,boundary=0,random_cases=0,overflow=0,low_carry=0;
    logic [63:0] random_state=64'h6a09e667f3bcc909 ^ (64'(WIDTH)*64'h9e3779b97f4a7c15);
    function automatic logic [31:0] next_random();
        random_state=random_state^(random_state<<13);
        random_state=random_state^(random_state>>7);
        random_state=random_state^(random_state<<17);
        return random_state[31:0];
    endfunction
    function automatic logic [WIDTH-1:0] random_word();
        logic [WIDTH-1:0] value;
        value='0;
        for(integer offset=0;offset<WIDTH;offset+=32)
            value=value | (WIDTH'(next_random())<<offset);
        return value;
    endfunction

    task automatic check(input logic [WIDTH-1:0] a,b,input integer kind);
        logic [WIDTH:0] expected,low_reference;
        expected={1'b0,a}+{1'b0,b};
        low_reference=({1'b0,a}&{1'b0,LOW_MASK})+({1'b0,b}&{1'b0,LOW_MASK});
        x=a;y=b;
        #1;
        if(native_result !== expected[WIDTH-1:0] || portable_result !== expected[WIDTH-1:0])
            $fatal(1,"WIDTH=%0d case=%0d a=%h b=%h expected=%h native=%h portable=%h",
                WIDTH,checks,a,b,expected[WIDTH-1:0],native_result,portable_result);
        if(!native_clock_held) $fatal(1,"WIDTH=%0d native clock changed",WIDTH);
        checks++;
        case(kind)
            0:directed++;
            1:boundary++;
            2:random_cases++;
            default:$fatal(1,"unknown test category");
        endcase
        if(expected[WIDTH]) overflow++;
        if(low_reference>{1'b0,LOW_MASK}) low_carry++;
    endtask

    logic [WIDTH-1:0] a,b,alternating;
    initial begin
        // Assert preprocessing actually selected native arithmetic and fallback.
        assert(native_dut.USE_NATIVE_HIGH==(WIDTH>17 && !FORCE_FABRIC)) else $fatal(1,"native branch not selected");
        assert(portable_dut.USE_NATIVE_HIGH==0) else $fatal(1,"portable branch not selected");
        check('0,'0,0);check(ALL_ONES,'0,0);check('0,ALL_ONES,0);
        check(ALL_ONES,WIDTH'(1),0);check(ALL_ONES,ALL_ONES,0);
        alternating='0;
        for(integer bit_index=0;bit_index<WIDTH;bit_index+=2) alternating[bit_index]=1'b1;
        check(alternating,~alternating,0);check(alternating,alternating,0);
        check(~alternating,~alternating,0);
        for(integer bit_index=0;bit_index<WIDTH;bit_index++) begin
            a=WIDTH'(1)<<bit_index;
            check(a,'0,0);check('0,a,0);check(a,a,0);
            check(a-WIDTH'(1),WIDTH'(1),0);
            check(ALL_ONES-a,a,0);check(~a,WIDTH'(1),0);
            check(a,ALL_ONES,0);check(ALL_ONES,a,0);
        end
        if(WIDTH>17) begin
            // Exhaust every combination of four reduced operand dimensions:
            // low x/y and high x/y. Each independently visits its bottom/top
            // range, crossing low carries, high carry-in and modular overflow.
            for(integer low_mode=0;low_mode<4;low_mode++)
            for(integer high_mode=0;high_mode<4;high_mode++)
            for(integer xl=0;xl<LOW_RANGE;xl++)
            for(integer yl=0;yl<LOW_RANGE;yl++)
            for(integer xh=0;xh<16;xh++)
            for(integer yh=0;yh<16;yh++) begin
                a=((((high_mode&1)!=0 ? WIDTH'(131056) : WIDTH'(0))+WIDTH'(xh))<<LOW_BITS) |
                  (((low_mode&1)!=0) ? LOW_MASK-WIDTH'(LOW_RANGE-1)+WIDTH'(xl) : WIDTH'(xl));
                b=((((high_mode&2)!=0 ? WIDTH'(131056) : WIDTH'(0))+WIDTH'(yh))<<LOW_BITS) |
                  (((low_mode&2)!=0) ? LOW_MASK-WIDTH'(LOW_RANGE-1)+WIDTH'(yl) : WIDTH'(yl));
                check(a,b,1);
            end
        end else begin
            // Exhaust bottom/top ranges on both fallback operands too.
            for(integer mode=0;mode<4;mode++)
            for(integer i=0;i<SMALL_RANGE;i++)
            for(integer j=0;j<SMALL_RANGE;j++) begin
                a=((mode&1)!=0) ? ALL_ONES-WIDTH'(SMALL_RANGE-1)+WIDTH'(i) : WIDTH'(i);
                b=((mode&2)!=0) ? ALL_ONES-WIDTH'(SMALL_RANGE-1)+WIDTH'(j) : WIDTH'(j);
                check(a,b,1);
            end
        end
        for(integer i=0;i<400000;i++) begin
            a=random_word();b=random_word();check(a,b,2);
        end
        assert(random_cases==400000 && overflow>0 && overflow<checks && low_carry>0 && low_carry<checks);
        $display("PASS width=%0d native=%0d checks=%0d directed=%0d boundary=%0d random=%0d overflow=%0d low_carry=%0d",
            WIDTH,WIDTH>17 && !FORCE_FABRIC,checks,directed,boundary,random_cases,overflow,low_carry);
        done=1;
    end
endmodule

// Compile this top with SYNTHESIS but without any vendor library: the public
// USE_DSP=0 profile must completely remove the primitive dependency.
module s5_high_add_fabric_tb;
    wire [1:0] done;
    S5HighAddCase #(.WIDTH(29),.FORCE_FABRIC(1)) w29(done[0]);
    S5HighAddCase #(.WIDTH(97),.FORCE_FABRIC(1)) w97(done[1]);
    initial begin
        wait(&done);#1;
        $display("PASS all explicit fabric widths; no vendor model or clock transitions");
        $finish;
    end
endmodule

module s5_high_add_tb;
    wire [14:0] done;
    S5HighAddCase #(.WIDTH(1)) w1(done[0]);
    S5HighAddCase #(.WIDTH(17)) w17(done[1]);
    S5HighAddCase #(.WIDTH(18)) w18(done[2]);
    S5HighAddCase #(.WIDTH(28)) w28(done[3]);
    S5HighAddCase #(.WIDTH(29)) w29(done[4]);
    S5HighAddCase #(.WIDTH(32)) w32(done[5]);
    S5HighAddCase #(.WIDTH(37)) w37(done[6]);
    S5HighAddCase #(.WIDTH(38)) w38(done[7]);
    S5HighAddCase #(.WIDTH(39)) w39(done[8]);
    S5HighAddCase #(.WIDTH(40)) w40(done[9]);
    S5HighAddCase #(.WIDTH(64)) w64(done[10]);
    S5HighAddCase #(.WIDTH(97)) w97(done[11]);
    // Production five-bit weight moments: mass, X, and Y widths.
    S5HighAddCase #(.WIDTH(26)) w26(done[12]);
    S5HighAddCase #(.WIDTH(35)) w35(done[13]);
    S5HighAddCase #(.WIDTH(33)) w33(done[14]);
    initial begin
        wait(&done);#1;
        $display("PASS all native/portable widths; no clock transitions");
        $finish;
    end
endmodule
