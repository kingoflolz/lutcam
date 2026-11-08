// Ordered accepted-job backend. Decode can fill the other job slot while
// this module fits, decomposes, and emits the current tag. Rejects bypass fit.
module LfTagAcceptedRefiner #(
    parameter bit EXPORT_ACCEPTED_CENTROIDS=1'b0
)(
    input logic clk,rst,cancel,job_valid,
    output logic job_pop,
    output logic [8:0] job_read_address,
    input logic [19:0] job_read_data,
    output logic out_valid,
    input logic out_ready,
    output logic [31:0] out_data,
    output logic out_last,
    output logic centroid_valid,
    input logic centroid_ready,
    output logic [32:0] centroid_data,
    output logic centroid_last
);
    import lftag_pkg::*;
    typedef enum logic [4:0] {IDLE,META_READ,META_WAIT,META_CAPTURE,FIT_START,FIT_WAIT,FIT_CONFIG_WAIT,
        WIDTH_REQUEST,POSE_START,POSE_WAIT,
        PACKET_LOW,PACKET_LOW_WAIT,PACKET_HIGH,PACKET_OUT,
        CENTROID_LOW,CENTROID_LOW_WAIT,CENTROID_HIGH,CENTROID_OUT,RETIRE} state_t;
    state_t state;
    logic metadata_index;
    logic [13:0] tag_id;
    logic [1:0] scale;
    logic decoded,failed;
    logic [4:0] index;
    logic [17:0] low_word;
    logic [3:0] point_number;
    logic point_axis;
    logic [14:0] composite_x,composite_offset;
    logic fit_ready,fit_done,fit_ok;
    logic fit_config_valid;
    logic [1:0] fit_config_address;
    logic [8:0] fit_address_a,fit_address_b;
    logic fit_write_a,fit_write_b;
    logic [7:0] program_address;
    logic program_read;
    logic signed[19:0] fit_write_data_a,fit_write_data_b;
    (* syn_ramstyle="block_ram" *) logic [17:0] bank_a[0:255],bank_b[0:255];
    (* syn_ramstyle="block_ram" *) logic [19:0] mirror[0:255];
    logic [7:0] ram_address_a,ram_address_b,ram_write_address;
    logic [17:0] qa,qb,ram_data_a,ram_data_b;
    logic [19:0] mirror_q;
    logic ram_write;
    logic pose_ready,pose_done,pose_valid,pose_weak,pose_conditioning,pose_saturation;
    logic [3:0] pose_read_a,pose_read_b,pose_write_address;
    logic pose_write;
    logic [17:0] pose_write_a,pose_write_b;
    logic rsqrt_valid,rsqrt_ready,rsqrt_rsp_valid,rsqrt_rsp_ready;
    logic [36:0] radicand;
    logic [18:0] lambda;
    logic [3:0] left_shift;
    wire fit_active=state==FIT_START || state==FIT_WAIT || state==FIT_CONFIG_WAIT;
    wire reset_engine=rst||cancel;
    wire accepted=decoded&&!failed;
    initial begin
`include "lftag_fit9_constants.svh"
    end
    assign job_pop=state==RETIRE;
    assign out_valid=state==PACKET_OUT&&!cancel;
    assign out_last=out_valid&&(index==(accepted?5'd21:5'd7));
    assign centroid_valid=state==CENTROID_OUT&&!cancel;
    assign centroid_last=centroid_valid&&index==8;
    assign centroid_data={1'b0,job_read_data[13:0],1'b0,composite_x,1'b0,low_word[15]};
    // Keep the synchronous RAM outputs selected while the consumer stalls.
    // They already register the payload; no second packet register is needed.
    always_comb begin
        out_data={job_read_data[13:0],low_word};
        if(index>=10)out_data=index<19?{qa[17],qa,13'b0}:{qb[13:0],qa};
        else if(index==8)out_data={28'b0,pose_saturation,pose_conditioning,pose_weak,pose_valid};
        else if(failed)case(index)
            1:out_data={job_read_data[13:0],low_word}&32'h3ff00000;
            6:out_data={27'b0,REJECT_SINGULAR_MAP};
            default:out_data=0;
        endcase
    end
    always_comb begin
        job_read_address={4'b0,point_number,point_axis};
        if(fit_active&&program_read)job_read_address={1'b1,program_address};
        if(state==FIT_WAIT&&fit_config_valid)job_read_address=9'd18+{7'b0,fit_config_address};
        if(state==WIDTH_REQUEST || state==POSE_START || state==POSE_WAIT)job_read_address=9'd31;
        if(state==META_READ || state==META_WAIT || state==META_CAPTURE)
            job_read_address=metadata_index?9'd35:9'd32;
        else if(state==PACKET_LOW || state==PACKET_LOW_WAIT || state==PACKET_HIGH || state==PACKET_OUT)
            job_read_address={4'b0001,(index==9?4'd8:index[3:0]),(state==PACKET_HIGH || state==PACKET_OUT)};
        else if(state==CENTROID_LOW || state==CENTROID_LOW_WAIT || state==CENTROID_HIGH || state==CENTROID_OUT)
            job_read_address={4'b0,index[3:0],(state==CENTROID_HIGH || state==CENTROID_OUT)};
        composite_offset=scale==1?15'd20544:(scale==2?15'd25792:15'd0);
        composite_x=low_word[14:0]+composite_offset;
        ram_address_a={4'b0,pose_read_a};ram_address_b={4'b0,pose_read_b};
        ram_write=pose_write;ram_write_address={4'b0,pose_write_address};
        ram_data_a=pose_write_a;ram_data_b=pose_write_b;
        if(fit_active)begin
            ram_address_a=fit_address_a[7:0];ram_address_b=fit_address_a[7:0];
            ram_write=fit_write_a;
            ram_write_address=fit_address_a[8]?{4'b0,fit_address_a[4:1]}:fit_address_a[7:0];
            ram_data_a=fit_write_data_a[17:0];ram_data_b={16'b0,fit_write_data_a[19:18]};
        end else if(state==PACKET_LOW || state==PACKET_LOW_WAIT || state==PACKET_OUT)begin
            ram_address_a={4'b0,4'(index-10)};ram_address_b={4'b0,4'(index-10)};
        end
    end
    always_ff @(posedge clk)begin
        qa<=bank_a[ram_address_a];qb<=bank_b[ram_address_b];
        mirror_q<=mirror[fit_address_b[7:0]];
        if(ram_write)begin bank_a[ram_write_address]<=ram_data_a;bank_b[ram_write_address]<=ram_data_b;end
        if(fit_active&&fit_write_a)mirror[ram_write_address]<=fit_write_data_a;
    end
    LfTagFit9 fit(.clk,.rst(reset_engine),.start(state==FIT_START),.ready(fit_ready),
        .done(fit_done),.done_ready(state==FIT_WAIT),.ok(fit_ok),.tag_id,.scale,
        .program_address,.program_read,.instruction(job_read_data),
        .point_number,.point_axis,.point_world_index(job_read_data[19:16]),.point_x_q4(job_read_data[14:0]),.point_y_q4(job_read_data[13:0]),
        .config_read_valid(fit_config_valid),.config_read_address(fit_config_address),
        .config_read_ready(state==FIT_WAIT),.config_response_valid(state==FIT_CONFIG_WAIT),.config_read_data(job_read_data),
        .address_a(fit_address_a),.address_b(fit_address_b),.read_data_a($signed({qb[1:0],qa})),.read_data_b($signed(mirror_q)),
        .write_a(fit_write_a),.write_b(fit_write_b),.write_data_a(fit_write_data_a),.write_data_b(fit_write_data_b));
    LfTagPoseReconstruct pose(.clk,.rst(reset_engine),.in_valid(state==POSE_START),.in_ready(pose_ready),
        .in_pose_enable(1'b1),.in_saturation(1'b0),.in_printed_width_q16({15'b0,job_read_data[16:0]}),
        .out_valid(pose_done),.out_ready(state==RETIRE),.out_pose_valid(pose_valid),
        .out_weak_perspective(pose_weak),.out_conditioning_warning(pose_conditioning),.out_saturation(pose_saturation),
        .result_read_address_a(pose_read_a),.result_read_address_b(pose_read_b),
        .result_read_data_a($signed(qa[17:0])),.result_read_data_b($signed(qb[17:0])),
        .result_write_valid(pose_write),.result_write_address(pose_write_address),
        .result_write_data_a(pose_write_a),.result_write_data_b(pose_write_b),
        .rsqrt_req_valid(rsqrt_valid),.rsqrt_req_ready(rsqrt_ready),.rsqrt_req_radicand(radicand),
        .rsqrt_rsp_valid,.rsqrt_rsp_ready,.rsqrt_rsp_lambda_q10(lambda),.rsqrt_rsp_left_shift(left_shift));
    LfTagReciprocalSqrt512 rsqrt(.clk,.rst(reset_engine),.in_valid(rsqrt_valid),.in_ready(rsqrt_ready),
        .in_radicand(radicand),.out_valid(rsqrt_rsp_valid),.out_ready(rsqrt_rsp_ready),
        .out_lambda_q10(lambda),.out_lambda_left_shift(left_shift));
    always_ff @(posedge clk)begin
        // Metadata and payload registers are loaded before their valid state.
        if(reset_engine)begin state<=IDLE;
        end else case(state)
            IDLE:if(job_valid)begin metadata_index<=0;failed<=0;index<=0;state<=META_READ;end
            META_READ:state<=META_WAIT;
            META_WAIT:state<=META_CAPTURE;
            META_CAPTURE:begin
                if(!metadata_index)tag_id<=job_read_data[13:0];
                else begin decoded<=job_read_data[13];scale<=job_read_data[11:10];end
                metadata_index<=1;
                if(metadata_index)state<=job_read_data[13]?FIT_START:PACKET_LOW;
                else state<=META_READ;
            end
            FIT_START:if(fit_ready)state<=FIT_WAIT;
            FIT_WAIT:begin
                if(fit_config_valid)state<=FIT_CONFIG_WAIT;
                else if(fit_done)begin
                    if(!fit_ok)begin failed<=1;state<=PACKET_LOW;end else state<=WIDTH_REQUEST;
                end
            end
            FIT_CONFIG_WAIT:state<=FIT_WAIT;
            WIDTH_REQUEST:state<=POSE_START;
            POSE_START:if(pose_ready)state<=POSE_WAIT;
            POSE_WAIT:if(pose_done)begin failed<=!pose_valid;state<=PACKET_LOW;end
            PACKET_LOW:state<=PACKET_LOW_WAIT;
            PACKET_LOW_WAIT:begin
                low_word<=job_read_data[17:0];state<=(index>=8&&index!=9)?PACKET_OUT:PACKET_HIGH;
            end
            PACKET_HIGH:state<=PACKET_OUT;
            PACKET_OUT:if(out_ready)begin
                if(out_last)begin index<=0;state<=(accepted&&EXPORT_ACCEPTED_CENTROIDS)?CENTROID_LOW:RETIRE;end
                else begin index<=index+1'b1;state<=PACKET_LOW;end
            end
            CENTROID_LOW:state<=CENTROID_LOW_WAIT;
            CENTROID_LOW_WAIT:begin low_word<=job_read_data[17:0];state<=CENTROID_HIGH;end
            CENTROID_HIGH:state<=CENTROID_OUT;
            CENTROID_OUT:if(centroid_ready)begin
                if(centroid_last)state<=RETIRE;else begin index<=index+1'b1;state<=CENTROID_LOW;end
            end
            RETIRE:state<=IDLE;
            default:state<=IDLE;
        endcase
    end
endmodule
