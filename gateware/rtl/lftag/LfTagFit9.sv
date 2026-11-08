// Nine-point linear homography fit: shared 3x3 and 2x2 Schur solves.
// A pipelined MAC consumes one dot-product term per clock. Scratch values
// are signed Q2.17; dot products retain six extra rounding guard bits.
module LfTagFit9 (
    input logic clk,rst,start,
    output logic ready,done,
    input logic done_ready,
    output logic ok,
    input logic [13:0] tag_id,
    input logic [1:0] scale,
    output logic [7:0] program_address,
    output logic program_read,
    input logic [19:0] instruction,
    output logic [3:0] point_number,
    output logic point_axis,
    input logic [3:0] point_world_index,
    input logic [14:0] point_x_q4,
    input logic [13:0] point_y_q4,
    output logic config_read_valid,
    output logic [1:0] config_read_address,
    input logic config_read_ready,config_response_valid,
    input logic [19:0] config_read_data,
    output logic [8:0] address_a,address_b,
    input logic signed [19:0] read_data_a,read_data_b,
    output logic write_a,write_b,
    output logic signed [19:0] write_data_a,write_data_b
);
    // Arithmetic opcodes encode accumulate (bit 0), subtract (bits 1:0),
    // and product scale (bits 3:2). Bit 4 selects non-MAC instructions.
    localparam [4:0] MUL=0,MAC=1,MSUB=3,DOT9=4,DOT9_ADD=5,
        DOT3=6,DOT3_SUB=7,MUL8=8,MSUB8=11,CAL=12,
        RCP=16,COPY=17,ADD=18,SUB=19,CONFIG=20,OUTPUT=21,VECTOR=22,HALT=23;
    typedef enum logic [4:0] {IDLE,SCAN_REQUEST,SCAN,SCAN_WORLD,BOUNDS,NORMALIZE,NORMALIZE_DRAIN,
        CONSTANTS,FETCH,ISSUE,RUN,SIMPLE,VEC,RECIP_INIT,DIVIDE_STEP,
        CONFIG_REQUEST,CONFIG_RESPONSE,DONE} state_t;
    state_t state;
    logic [7:0] pc,source_a,source_b,destination,vector_destination;
    logic [4:0] operation;
    wire [4:0] next_operation=instruction[19:15];
    wire [7:0] next_a=instruction[14:7];
    wire [7:0] next_b={(next_operation==DOT9 || next_operation==DOT9_ADD || next_operation==VECTOR),instruction[6:0]};
    logic [3:0] remaining;
    logic product_valid;
    logic signed [39:0] product;
    logic signed [15:0] lower_delta,upper_delta,normal_delta,normal_delta_q;
    logic signed [17:0] normal_factor;
    logic signed [33:0] normal_product;
    wire signed [19:0] accumulator=sum[25:6];
    logic signed [25:0] sum,product_scaled;
    logic signed [26:0] alu_a,alu_b,alu;
    logic subtract;
    logic [1:0] product_mode;
    logic product_fits,alu_fits;
    logic [19:0] remainder;
    wire [19:0] divisor=read_data_a;
    logic [4:0] divide_bit;
    logic reciprocal_small;
    wire signed [20:0] reciprocal_difference;
    logic [1:0] vector_phase;
    logic [3:0] scan_index;
    logic [14:0] minimum,maximum,center;
    logic [4:0] exponent;
    logic normalize_axis;
    logic [3:0] normalize_index;
    logic [2:0] normalize_valid;
    logic [3:0] normalize_write_index;
    logic constant_index;
    logic [19:0] constant_source,constant_scaled;
    logic [14:0] coordinate;
    logic signed [19:0] world_coordinate;
    logic [2:0] cell_index;
    logic symbol;
    logic [1:0] region;
    wire [14:0] span=upper_delta[15:1];
    function automatic [4:0] bit_length(input logic [14:0] value);
        bit_length=0;
        for(integer n=0;n<15;n++)if(value[n])bit_length=5'(n+1);
    endfunction
    assign program_address=pc;
    assign program_read=state!=SCAN_REQUEST && state!=SCAN && state!=SCAN_WORLD && state!=IDLE;
    assign ready=state==IDLE;
    assign done=state==DONE;
    assign point_number=scan_index+4'(state==SCAN_WORLD && scan_index!=8);
    assign point_axis=normalize_axis;
    assign config_read_valid=state==CONFIG_REQUEST;
    assign config_read_address=source_a[1:0];
    LfTagWideAddSub #(.WIDTH(21)) reciprocal_subtractor(
        .a($signed({remainder,1'b0})),.b($signed({1'b0,divisor})),
        .subtract(1'b1),.result(reciprocal_difference));
    LfTagWideAddSub #(.WIDTH(27)) arithmetic(.a(alu_a),.b(alu_b),.subtract,.result(alu));
    LfTagSmallAddSub lower_bound(.a($signed(state==BOUNDS?{1'b0,minimum}:{1'b0,coordinate})),
        .b($signed(state==BOUNDS?{1'b0,maximum}:{1'b0,minimum})),.subtract(state!=BOUNDS),.result(lower_delta));
    LfTagSmallAddSub upper_bound(.a($signed({1'b0,maximum})),
        .b($signed(state==BOUNDS?{1'b0,minimum}:{1'b0,coordinate})),.subtract(1'b1),.result(upper_delta));
    LfTagSmallAddSub normalize_delta(.a($signed({1'b0,read_data_b[14:0]})),
        .b($signed({1'b0,center})),.subtract(1'b1),.result(normal_delta));
    assign normal_factor=18'sd131072>>exponent;
    always_comb begin
        constant_source=constant_index?{5'b0,center}:(20'd1<<exponent);
        case(scale)
            1:constant_scaled={constant_source[17:0],2'b0};
            2:constant_scaled={constant_source[15:0],4'b0};
            default:constant_scaled=constant_source;
        endcase
        coordinate=normalize_axis?{1'b0,point_y_q4}:point_x_q4;
        cell_index=3'(point_world_index-4'd2);
        symbol=tag_id[{cell_index,!normalize_axis}];region=1;
        if(normalize_axis)begin if(cell_index==0)region=0;else if(cell_index>=4)region=2;end
        else case(cell_index)1,4:region=0;3,6:region=2;default:begin end endcase
        case(region)
            0:world_coordinate=symbol?-20'sd120149:-20'sd141995;
            2:world_coordinate=symbol?20'sd141995:20'sd120149;
            default:world_coordinate=symbol?20'sd10923:-20'sd10923;
        endcase
        if(point_world_index<2)world_coordinate=(!normalize_axis&&point_world_index==1)?20'sd131072:-20'sd131072;
        case(product_mode)
            1:begin product_scaled=product[39:14];product_fits=1;end
            2:begin product_scaled=product[33:8];product_fits=product[39:34]=={6{product[33]}};end
            3:begin product_scaled=product[31:6];product_fits=product[39:32]=={8{product[31]}};end
            default:begin product_scaled=product[36:11];product_fits=product[39:37]=={3{product[36]}};end
        endcase
        alu_a=$signed({sum[25],sum});
        alu_b=$signed({product_scaled[25],product_scaled});
        subtract=operation[0]&&operation[1];
        if(state==SIMPLE)begin
            // Keep simple arithmetic in the MAC's guarded scale so every
            // result uses the same RAM write slice.
            alu_a=operation==ADD?$signed({accumulator[19],accumulator,6'b0}):
                (operation==OUTPUT?$signed({{2{read_data_a[19]}},read_data_a,5'b0}):$signed({read_data_a[19],read_data_a,6'b0}));
            alu_b=operation==SUB?$signed({read_data_b[19],read_data_b,6'b0}):
                (operation==ADD?$signed({read_data_a[19],read_data_a,6'b0}):(operation==OUTPUT?27'sd32:27'sd0));
            subtract=operation==SUB;
        end else if(state==VEC)begin
            alu_a=27'sd32;subtract=0;
        end
        alu_fits=alu[26]==alu[25];
        address_a={1'b0,8'(source_a+1'b1)};address_b={1'b0,8'(source_b+1'b1)};
        // Keep the determinant in the synchronous RAM output during division.
        if(state==RECIP_INIT || state==DIVIDE_STEP)address_a={1'b0,source_a};
        write_a=0;write_b=0;write_data_b=world_coordinate;
        // State groups are disjoint. Combine them directly instead of a
        // priority chain on the 20-bit write port.
        write_data_a=({20{state==SCAN}}&{5'b0,coordinate}) |
            ({20{state==SCAN_WORLD}}&world_coordinate) |
            ({20{state==NORMALIZE || state==NORMALIZE_DRAIN}}&normal_product[19:0]) |
            ({20{state==CONSTANTS}}&(constant_scaled+((constant_index&&normalize_axis&&scale==2)?20'd64:20'd0))) |
            ({20{state==RUN || state==SIMPLE || state==VEC}}&alu[25:6]) |
            ({20{state==DIVIDE_STEP}}&{accumulator[18:0],!reciprocal_difference[20]}) |
            ({20{state==CONFIG_RESPONSE}}&config_read_data);
        if(state==SCAN || state==SCAN_WORLD)begin
            address_a=(normalize_axis?9'd155:9'd146)+{5'b0,scan_index};
            address_b=(normalize_axis?9'd137:9'd128)+{5'b0,scan_index};
            write_a=1;
            if(state==SCAN_WORLD)address_a=address_b;
        end else if(state==NORMALIZE || state==NORMALIZE_DRAIN)begin
            address_b=(normalize_axis?9'd155:9'd146)+{5'b0,normalize_index};
            address_a=(normalize_axis?9'd155:9'd146)+{5'b0,normalize_write_index};
            write_a=normalize_valid[2];
        end else if(state==CONSTANTS)begin
            address_a=(constant_index?9'd211:9'd209)+{8'b0,normalize_axis};write_a=1;
        end else if(state==ISSUE)begin
            address_a={1'b0,next_a};address_b={1'b0,next_b};
        end else if(state==RUN && remaining==0 && product_valid)begin
            address_a={1'b0,destination};write_a=product_fits&&alu_fits;
        end else if(state==SIMPLE)begin
            address_a={1'b0,destination};write_a=1;
            if(operation==OUTPUT)begin
                address_a=9'd256+{1'b0,source_b,1'b0};
                write_a=alu[26:23]=={4{alu[23]}};
            end
        end else if(state==VEC && vector_phase[1])begin
            address_a={1'b0,vector_destination};write_a=1;
        end else if(state==DIVIDE_STEP && divide_bit==0)begin
            address_a={1'b0,destination};write_a=1;
        end else if(state==CONFIG_RESPONSE && config_response_valid)begin
            address_a={1'b0,destination};write_a=1;
        end
    end
    always_ff @(posedge clk)begin
        if(state==RUN || state==VEC)product<=read_data_a*read_data_b;
        if(state==NORMALIZE || state==NORMALIZE_DRAIN)begin
            normal_delta_q<=normal_delta;normal_product<=normal_delta_q*normal_factor;
        end
    end
    always_ff @(posedge clk)begin
        // All datapath words are initialized by the input scan or their
        // producing instruction before use. Reset only transaction control.
        if(rst)begin state<=IDLE;ok<=0;
        end else case(state)
            IDLE:if(start)begin
                pc<=0;ok<=1;scan_index<=0;normalize_axis<=0;minimum<=32767;maximum<=0;state<=SCAN_REQUEST;
            end
            SCAN_REQUEST:state<=SCAN;
            SCAN:begin
                if(lower_delta[15])minimum<=coordinate;if(upper_delta[15])maximum<=coordinate;
                state<=SCAN_WORLD;
            end
            SCAN_WORLD:begin
                if(scan_index==8)state<=BOUNDS;
                else begin scan_index<=scan_index+1'b1;state<=SCAN;end
            end
            BOUNDS:begin
                center<=lower_delta[15:1];
                exponent<=bit_length(span+1'b1);
                normalize_index<=0;normalize_valid<=0;normalize_write_index<=0;state<=NORMALIZE;
            end
            NORMALIZE,NORMALIZE_DRAIN:begin
                normalize_valid<={normalize_valid[1:0],state==NORMALIZE};
                if(normalize_valid[2])normalize_write_index<=normalize_write_index+1'b1;
                if(state==NORMALIZE)begin
                    if(normalize_index==8)state<=NORMALIZE_DRAIN;else normalize_index<=normalize_index+1'b1;
                end else if(normalize_valid==3'b100)begin
                    normalize_index<=0;normalize_valid<=0;normalize_write_index<=0;
                    constant_index<=0;state<=CONSTANTS;
                end
            end
            CONSTANTS:begin
                constant_index<=1;
                if(constant_index)begin
                    if(normalize_axis)state<=FETCH;
                    else begin normalize_axis<=1;scan_index<=0;minimum<=32767;maximum<=0;state<=SCAN_REQUEST;end
                end
            end
            FETCH:state<=ISSUE;
            ISSUE:begin
                operation<=next_operation;source_a<=next_a;source_b<=next_b;destination<=pc;pc<=pc+1'b1;
                remaining<=next_operation[3:2]==2'b01 ? (next_operation[1]?4'd3:4'd9):4'd1;
                product_valid<=0;
                if(!next_operation[4])sum<=next_operation[0]?{accumulator,6'b100000}:26'd32;
                product_mode<=next_operation[3:2]==2'b01 ? (next_operation[1]?2'd0:2'd1):next_operation[3:2];
                case(next_operation)
                    HALT:state<=DONE;
                    CONFIG:state<=CONFIG_REQUEST;
                    COPY,ADD,SUB,OUTPUT:state<=SIMPLE;
                    RCP:begin reciprocal_small<=next_b[0];state<=RECIP_INIT;end
                    VECTOR:begin
                        source_a<=next_a;source_b<=next_b;remaining<=9;vector_phase<=1;state<=VEC;
                        case(pc[1:0])2:vector_destination<=164;3:vector_destination<=173;0:vector_destination<=182;1:vector_destination<=191;endcase
                    end
                    default:begin source_a<=next_a;source_b<=next_b;state<=RUN;end
                endcase
            end
            RUN:begin
                product_valid<=remaining!=0;
                if(remaining!=0)begin remaining<=remaining-1'b1;source_a<=source_a+1'b1;source_b<=source_b+1'b1;end
                if(product_valid)begin
                    sum<=alu[25:0];
                    if(!product_fits||!alu_fits)begin ok<=0;state<=DONE;end
                    else if(remaining==0)state<=ISSUE;
                end
            end
            SIMPLE:begin
                sum<={write_data_a,6'b0};state<=ISSUE;
                if((operation==ADD || operation==SUB) && !alu_fits)begin ok<=0;state<=DONE;end
                if(operation==OUTPUT&&!write_a)begin ok<=0;state<=DONE;end
            end
            VEC:begin
                vector_phase<=vector_phase+1'b1;
                if(!vector_phase[1])begin source_a<=source_a+1'b1;source_b<=source_b+1'b1;end
                else begin
                    vector_destination<=vector_destination+1'b1;remaining<=remaining-1'b1;
                    if(remaining==1)state<=ISSUE;
                end
            end
            RECIP_INIT:begin
                remainder<=reciprocal_small?20'd4096:20'd16384;sum<=0;divide_bit<=19;state<=DIVIDE_STEP;
                if(read_data_a<=(reciprocal_small?20'sd8192:20'sd32768))begin ok<=0;state<=DONE;end
            end
            DIVIDE_STEP:begin
                sum<={accumulator[18:0],!reciprocal_difference[20],6'b0};
                remainder<=reciprocal_difference[20]?{remainder[18:0],1'b0}:reciprocal_difference[19:0];
                divide_bit<=divide_bit-1'b1;
                // RECIP_INIT bounds the divisor so the quotient fits signed20.
                if(divide_bit==0)state<=ISSUE;
            end
            CONFIG_REQUEST:if(config_read_ready)state<=CONFIG_RESPONSE;
            CONFIG_RESPONSE:if(config_response_valid)begin
                sum<={write_data_a,6'b0};state<=ISSUE;
                if(!source_a[1] && config_read_data==0)begin ok<=0;state<=DONE;end
            end
            DONE:if(done_ready)state<=IDLE;
            default:state<=IDLE;
        endcase
    end
endmodule
