// Candidate validation and winning decode. A job is committed only after
// its complete header, correspondence map and nine centroids are in RAM.
// All-nine fitting begins after the winning decode passes the residual limit.
module LfTagDecodeFrontend (
    input logic clk,rst,
    input logic head_valid,head_protocol,
    input logic [1:0] head_scale,
    input logic [7:0] head_frame,head_mass_spread,
    output logic head_pop,
    output logic desc_read_valid,
    output logic [3:0] desc_read_index,
    input logic fifo_desc_data_valid,
    input logic [39:0] fifo_desc_data,
    input logic fifo_desc_centroid_valid,
    output logic config_read_valid,
    output logic [3:0] config_read_address,
    input logic config_read_ready,config_response_valid,
    input logic [31:0] config_read_data,
    input logic job_ready,
    output logic job_valid,
    output logic job_write_valid,
    output logic [6:0] job_write_address,
    output logic [19:0] job_write_data,
    output logic core_idle,core_dispatch_ready,candidate_start,candidate_done,
    output logic [9:0] candidate_cycles
);
    import lftag_pkg::*;
    typedef enum logic[4:0] {S_IDLE,S_CONFIG_REQUEST,S_CONFIG_RESPONSE,S_VALIDATE,
        S_CORNER_START,S_CORNER_WAIT,S_RECT_START,S_RECT_WAIT,S_SCORE_WAIT,
        S_MEAN,S_COPY_REQUEST,S_COPY_WAIT,S_COPY_Y,S_HEADER,S_COMMIT} state_t;
    state_t state;
    logic [3:0] config_index;
    logic [1:0] work_scale;
    logic [7:0] work_frame;
    logic [7:0] selected_max_mass_spread;
    logic [15:0] selected_min_baseline_q4;
    logic [9:0] cycle_counter;
    logic [3:0] copy_index;
    logic [13:0] copy_y;
    logic [4:0] header_index;
    logic [31:0] header_word;
    logic [3:0] copy_world_index;
    logic [4:0] mean_bit;
    logic [31:0] mean_quotient;
    logic [38:0] remainder;
    wire [39:0] mean_difference;
    LfTagWideAddSub #(.WIDTH(40)) residual_subtractor(.a($signed({remainder,1'b0})),
        .b($signed({1'b0,scorer_best_weight_sum})),.subtract(1'b1),.result(mean_difference));
    logic mean_limit_pass;
    lftag_reject_t reject_reason;
    logic core_soft_reset,release_results;
    wire signed[17:0] fifo_desc_x_q4=$signed({3'b0,fifo_desc_data[14:0]});
    wire signed[17:0] fifo_desc_y_q4=$signed({4'b0,fifo_desc_data[28:15]});
    wire fifo_desc_reserved_error=|fifo_desc_data[39:38];
    assign core_idle=state==S_IDLE;
    assign core_dispatch_ready=core_idle&&job_ready&&!rst;
    assign config_read_valid=state==S_CONFIG_REQUEST;
    assign config_read_address=config_index;
    assign release_results=state==S_COMMIT;
    assign job_valid=state==S_COMMIT;
    assign head_pop=job_valid;
    assign mean_limit_pass=(mean_quotient[31:16]<config_read_data[15:0]) ||
        ((mean_quotient[31:16]==config_read_data[15:0]) && mean_quotient[15:0]==0 && remainder==0);
    logic corner_in_ready;
    logic corner_desc_req_valid;
    logic [3:0] corner_desc_req_index;
    logic corner_out_valid;
    logic [2:0] corner_out_status;
    logic corner_out_input_valid;
    /* verilator lint_off UNUSEDSIGNAL */
    logic corner_out_baseline_valid;
    logic corner_out_dominant_valid;
    logic corner_out_corners_valid;
    logic corner_out_quad_valid;
    logic [7:0] corner_out_cycles;
    /* verilator lint_on UNUSEDSIGNAL */
    /* verilator lint_off UNUSEDSIGNAL */
    logic [15:0] corner_indices;
    /* verilator lint_on UNUSEDSIGNAL */
    logic [19:0] remaining_indices;
    logic [71:0] corner_x_q4_flat;
    logic [71:0] corner_y_q4_flat;

    LfTagCornerSelect9 #(.USE_DSP_SUBTRACT(1'b1)) corner_select (
        .clk,
        .rst(rst || core_soft_reset),
        .in_valid(state == S_CORNER_START),
        .in_ready(corner_in_ready),
        .in_points_valid(1'b1),
        .in_min_baseline_q4(selected_min_baseline_q4),
        .desc_req_valid(corner_desc_req_valid),
        .desc_req_index(corner_desc_req_index),
        .desc_rsp_valid(fifo_desc_data_valid
            && ((state == S_CORNER_START) || (state == S_CORNER_WAIT))),
        .desc_rsp_x_q4(fifo_desc_x_q4),
        .desc_rsp_y_q4(fifo_desc_y_q4),
        .desc_rsp_mass_q5_3(fifo_desc_data[36:29]),
        .desc_rsp_centroid_valid(fifo_desc_centroid_valid
            && fifo_desc_data[37]
            && !fifo_desc_reserved_error),
        .out_valid(corner_out_valid),
        .out_ready(release_results),
        .out_status(corner_out_status),
        .out_input_valid(corner_out_input_valid),
        .out_baseline_valid(corner_out_baseline_valid),
        .out_dominant_valid(corner_out_dominant_valid),
        .out_corners_valid(corner_out_corners_valid),
        .out_quad_valid(corner_out_quad_valid),
        .out_corner_index_flat(corner_indices),
        .out_remaining_index_flat(remaining_indices),
        .out_corner_x_q4_flat(corner_x_q4_flat),
        .out_corner_y_q4_flat(corner_y_q4_flat),
        .out_cycles(corner_out_cycles)
    );

    // Divide-free four-corner geometry and sixteen-hypothesis scoring share
    // one Q15 cross-product engine and two signed arithmetic multiplier lanes.
    logic rect_start_ready;
    logic rect_desc_req_valid;
    logic rect_desc_req_ready;
    logic [3:0] rect_desc_req_index;
    logic signed [17:0] rect_map_cden;
    logic signed [17:0] rect_map_cg;
    logic signed [17:0] rect_map_ch;
    logic rect_done_valid;
    logic rect_done_invalid;
    logic scorer_clear;
    logic scorer_out_valid;
    logic scorer_best_valid;
    logic scorer_input_invalid;
    logic [3:0] scorer_best_hypothesis;
    logic [13:0] scorer_best_id;
    logic [14:0] scorer_best_cell_map;
    logic [63:0] scorer_best_score;
    logic [37:0] scorer_best_residual_sum;
    logic [38:0] scorer_best_weight_sum;
    /* verilator lint_off UNUSEDSIGNAL */
    logic [9:0] homogeneous_cycles;
    /* verilator lint_on UNUSEDSIGNAL */

    LfTagHomogeneousGeometryScorer homogeneous_scorer (
        .clk,
        .rst(rst || core_soft_reset),
        .clear(scorer_clear),
        .start_valid(state == S_RECT_START),
        .start_ready(rect_start_ready),
        .in_scale(work_scale),
        .corner_x_flat(corner_x_q4_flat),
        .corner_y_flat(corner_y_q4_flat),
        .point_input_index_flat(remaining_indices),
        .descriptor_req_valid(rect_desc_req_valid),
        .descriptor_req_ready(rect_desc_req_ready),
        .descriptor_req_index(rect_desc_req_index),
        .descriptor_rsp_valid(fifo_desc_data_valid && (state == S_RECT_WAIT)),
        .descriptor_rsp_x(fifo_desc_x_q4),
        .descriptor_rsp_y(fifo_desc_y_q4),
        .geometry_done_valid(rect_done_valid),
        .geometry_done_invalid(rect_done_invalid),
        .map_cden(rect_map_cden),
        .map_cg(rect_map_cg),
        .map_ch(rect_map_ch),
        .out_valid(scorer_out_valid),
        .out_ready(release_results),
        .out_best_valid(scorer_best_valid),
        .out_input_invalid(scorer_input_invalid),
        .out_best_hypothesis(scorer_best_hypothesis),
        .out_best_id(scorer_best_id),
        .out_best_cell_map(scorer_best_cell_map),
        .out_best_score_q32(scorer_best_score),
        .out_best_residual_sum(scorer_best_residual_sum),
        .out_best_weight_sum(scorer_best_weight_sum),
        .out_cycles(homogeneous_cycles)
    );


    always_comb begin
        desc_read_valid=0;desc_read_index=copy_index;rect_desc_req_ready=0;
        if(state==S_CORNER_START || state==S_CORNER_WAIT)begin
            desc_read_valid=corner_desc_req_valid;desc_read_index=corner_desc_req_index;
        end else if(state==S_RECT_WAIT)begin
            desc_read_valid=rect_desc_req_valid;desc_read_index=rect_desc_req_index;rect_desc_req_ready=1;
        end else if(state==S_COPY_REQUEST)desc_read_valid=1;
        copy_world_index=0;
        for(integer n=0;n<5;n=n+1)
            if(copy_index==remaining_indices[n*4+:4])copy_world_index={1'b0,scorer_best_cell_map[n*3+:3]}+4'd2;
        if(copy_index==corner_indices[3:0])copy_world_index=0;
        else if(copy_index==corner_indices[7:4])copy_world_index=1;
        else if(copy_index==corner_indices[11:8])copy_world_index=8;
        else if(copy_index==corner_indices[15:12])copy_world_index=6;
        header_word=0;
        if(reject_reason==REJECT_NONE)case(header_index[4:1])
            0:header_word={18'b0,scorer_best_id};
            1:header_word={2'b10,work_scale,work_frame,scorer_best_hypothesis,16'b0};
            2:header_word=scorer_best_score[31:0];
            3:header_word=scorer_best_score[63:32];
            8:header_word=mean_quotient;
            default:begin end
        endcase
        else case(header_index[4:1])
            1:header_word={2'b0,work_scale,work_frame,20'b0};
            6:header_word={27'b0,reject_reason};
            default:begin end
        endcase
        job_write_valid=0;job_write_address=0;job_write_data=0;
        if(state==S_COPY_WAIT && fifo_desc_data_valid)begin
            job_write_valid=1;job_write_address={2'b0,copy_index,1'b0};
            job_write_data={copy_world_index,fifo_desc_data[37],fifo_desc_data[14:0]};
        end else if(state==S_COPY_Y)begin
            job_write_valid=1;job_write_address={2'b0,copy_index,1'b1};job_write_data={copy_world_index,2'b0,copy_y};
        end else if(state==S_HEADER)begin
            job_write_valid=1;
            job_write_address={2'b01,header_index};
            job_write_data=header_index[0]?{6'b0,header_word[31:18]}:{2'b0,header_word[17:0]};
        end else if(state==S_CONFIG_RESPONSE && config_response_valid && (config_index[3] || config_index==0))begin
            // Accepted jobs carry their own calibration snapshot. The fitter
            // reads its 20-bit native format directly from the job RAM.
            job_write_valid=1;
            job_write_address=config_index[3]?7'd18+{5'b0,config_index[1:0]}:7'd31;
            if(config_index==0)job_write_data=(|config_read_data[31:17])?20'd0:{3'b0,config_read_data[16:0]};
            else if(config_index[1])job_write_data=config_read_data[31:12];
            else job_write_data=(|config_read_data[31:24])?20'd0:config_read_data[24:5];
        end
    end
    always_ff @(posedge clk)begin
        if(rst)begin
            state<=S_IDLE;core_soft_reset<=0;scorer_clear<=0;
            candidate_start<=0;candidate_done<=0;candidate_cycles<=0;
        end else begin
            candidate_start<=0;candidate_done<=0;core_soft_reset<=0;scorer_clear<=0;
            if(state!=S_IDLE && state!=S_COMMIT)cycle_counter<=cycle_counter+1'b1;
            if(state!=S_IDLE && state!=S_HEADER && state!=S_COMMIT && cycle_counter==999)begin
                reject_reason<=REJECT_WATCHDOG;state<=S_HEADER;header_index<=0;core_soft_reset<=1;
            end else case(state)
                S_IDLE:if(head_valid&&job_ready)begin
                    work_scale<=head_scale;work_frame<=head_frame;cycle_counter<=1;
                    reject_reason<=REJECT_NONE;candidate_start<=1;scorer_clear<=1;
                    config_index<=7;state<=S_CONFIG_REQUEST;header_index<=0;
                end
                S_CONFIG_REQUEST:if(config_read_ready)state<=S_CONFIG_RESPONSE;
                S_CONFIG_RESPONSE:if(config_response_valid)begin
                    if(config_index==7)begin selected_max_mass_spread<=config_read_data[7:0];selected_min_baseline_q4<=config_read_data[23:8];state<=S_VALIDATE;end
                    else if(config_index==1)begin
                        if(!mean_limit_pass)begin reject_reason<=REJECT_RESIDUAL;state<=S_HEADER;end
                        else begin config_index<=8;state<=S_CONFIG_REQUEST;end
                    end else if(config_index[3])begin config_index<=config_index==11?4'd0:config_index+1'b1;state<=S_CONFIG_REQUEST;end
                    else begin copy_index<=0;state<=S_COPY_REQUEST;end
                end
                S_VALIDATE:begin
                    if(head_protocol)begin reject_reason<=REJECT_PROTOCOL;state<=S_HEADER;end
                    else if(head_mass_spread>selected_max_mass_spread)begin reject_reason<=REJECT_MASS_SPREAD;state<=S_HEADER;end
                    else state<=S_CORNER_START;
                end
                S_CORNER_START:if(corner_in_ready)state<=S_CORNER_WAIT;
                S_CORNER_WAIT:if(corner_out_valid)begin
                    state<=S_HEADER;
                    if(!corner_out_input_valid || corner_out_status==1)reject_reason<=REJECT_INVALID_CENTROID;
                    else case(corner_out_status)
                        0:state<=S_RECT_START;
                        2:reject_reason<=REJECT_BASELINE_SHORT;
                        3:reject_reason<=REJECT_DOMINANT_SIDE;
                        5:reject_reason<=REJECT_INVALID_QUAD;
                        default:reject_reason<=REJECT_CORNER_SELECTION;
                    endcase
                end
                S_RECT_START:if(rect_start_ready)state<=S_RECT_WAIT;
                S_RECT_WAIT:if(rect_done_valid)begin
                    if(rect_done_invalid)begin reject_reason<=REJECT_SINGULAR_MAP;state<=S_HEADER;end
                    else state<=S_SCORE_WAIT;
                end
                S_SCORE_WAIT:if(scorer_out_valid)begin
                    if(!scorer_best_valid||scorer_input_invalid)begin reject_reason<=REJECT_NO_HYPOTHESIS;state<=S_HEADER;end
                    else begin mean_bit<=31;mean_quotient<=0;remainder<={1'b0,scorer_best_residual_sum};state<=S_MEAN;end
                end
                S_MEAN:begin
                    remainder<=mean_difference[39]?{remainder[37:0],1'b0}:mean_difference[38:0];
                    mean_quotient<={mean_quotient[30:0],!mean_difference[39]};mean_bit<=mean_bit-1'b1;
                    if(mean_bit==0)begin config_index<=1;state<=S_CONFIG_REQUEST;end
                end
                S_COPY_REQUEST:state<=S_COPY_WAIT;
                S_COPY_WAIT:if(fifo_desc_data_valid)begin copy_y<=fifo_desc_data[28:15];state<=S_COPY_Y;end
                S_COPY_Y:begin
                    if(copy_index==8)state<=S_HEADER;
                    else begin copy_index<=copy_index+1'b1;state<=S_COPY_REQUEST;end
                end
                S_HEADER:begin header_index<=header_index+1'b1;if(header_index==17)state<=S_COMMIT;end
                S_COMMIT:begin candidate_done<=1;candidate_cycles<=cycle_counter;state<=S_IDLE;end
                default:state<=S_IDLE;
            endcase
        end
    end
endmodule
