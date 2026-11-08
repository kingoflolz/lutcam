// Two camera queues feed a decode frontend and an independent accepted-tag
// refinement backend. The two-entry job queue preserves packet order and
// owns camera/frame context until packet and centroid replay have drained.
module SharedLfTagDecoder #(
    parameter bit EXPORT_ACCEPTED_CENTROIDS = 1'b0,
    parameter bit EXTERNAL_CONFIG = 1'b0
) (
    input  logic clk,
    input  logic rst,
    input  logic [1:0] camera_reset,

    input  logic [1:0] in_valid,
    input  logic [1:0] in_group_end,
    input  logic [1:0][14:0] in_x_q4,
    input  logic [1:0][13:0] in_y_q4,
    input  logic [1:0][7:0] in_log_mass_q5_3,
    input  logic [1:0] in_centroid_valid,
    input  logic [1:0][1:0] in_scale,
    input  logic [1:0][7:0] in_frame_seq,

    input  logic [1:0][31:0] cfg_inv_fx_q2_30,
    input  logic [1:0][31:0] cfg_inv_fy_q2_30,
    input  logic signed [1:0][31:0] cfg_cx_q16_16,
    input  logic signed [1:0][31:0] cfg_cy_q16_16,
    input  logic [1:0][31:0] cfg_printed_width_q16_16,
    input  logic [1:0][15:0] cfg_mean_residual_limit_q16,
    input  logic [1:0][7:0] cfg_max_mass_spread,
    input  logic [1:0][15:0] cfg_min_baseline_q4,

    output logic config_read_valid,
    output logic [3:0] config_read_address,
    input  logic config_read_ready,
    input  logic config_response_valid,
    input  logic [31:0] config_read_data,

    output logic [1:0] out_valid,
    input  logic [1:0] out_ready,
    output logic [31:0] out_data,
    output logic [1:0] out_last,
    output logic [1:0] centroid_valid,
    input  logic [1:0] centroid_ready,
    output logic [32:0] centroid_data,
    output logic [1:0] centroid_last,
    output logic [1:0] busy,
    output logic [1:0][5:0] fifo_level,
    output logic [1:0] candidate_start,
    output logic [1:0] candidate_done,
    output logic [1:0][9:0] candidate_cycles,
    input  logic [1:0] clear_diagnostics,
    output logic [1:0] fifo_drop_sticky,
    output logic [1:0] protocol_error_sticky,
    output logic [1:0][31:0] debug_status,
    output logic [1:0][31:0] debug_protocol,
    output logic [1:0][31:0] debug_drop
);

    logic [1:0] head_valid,head_protocol,head_pop,desc_read_valid,desc_data_valid,desc_centroid_valid;
    logic [1:0][1:0] head_scale;
    logic [1:0][7:0] head_frame_seq,head_mass_spread;
    logic [1:0][39:0] desc_data;
    logic [1:0] unused_fifo_full;
    logic owner_active,owner,issued,next_owner,selected_owner,selection_valid,owner_enabled;
    logic core_rst,core_idle,core_dispatch_ready,core_head_valid,core_head_pop;
    logic core_desc_read_valid;
    logic [3:0] core_desc_read_index;
    logic core_candidate_start,core_candidate_done;
    logic [9:0] core_candidate_cycles;
    logic [1:0][9:0] last_candidate_cycles;
    logic front_config_valid,front_config_ready,front_config_response;
    logic [3:0] front_config_address;
    logic config_pending;
    logic [31:0] direct_config_data;
    logic request_camera,request_valid,request_fire,response_valid;
    logic [3:0] request_address;
    logic [31:0] response_data;
    logic job_commit,job_write_valid;
    logic [6:0] job_write_address;
    logic [8:0] job_read_address;
    logic [19:0] job_write_data,job_read_data;
    // Lower half: two job slots. Upper half: the read-only fit schedule.
    // Each 128-word job: 0..17 coordinates/correspondence; 18..21 calibration;
    // 31 printed width; 32..49 header and decode residual. The aligned header
    // addresses concatenate their word index rather than requiring an adder.
    // Only job input/packet replay or instruction fetch uses the read port.
    (* syn_ramstyle="block_ram" *) logic[19:0] job_memory[0:511];
    initial begin
`include "lftag_fit9_program.svh"
    end
    logic write_slot,read_slot;
    logic [1:0] job_count,job_live,job_camera;
    logic job_pop,backend_pop,backend_cancel,backend_available;
    logic back_out_valid,back_out_last,back_centroid_valid,back_centroid_last;
    logic core_out_ready,core_out_valid;
    wire backend_camera=job_camera[read_slot];
    assign core_out_ready=out_ready[backend_camera];
    assign core_out_valid=back_out_valid;
    for (genvar camera = 0; camera < 2; camera = camera + 1) begin : g_camera
        LfTagCandidateFifoRamMetadata #(.RETAIN_DIAGNOSTICS(1'b1)) candidate_fifo (
            .clk,
            .rst(rst || camera_reset[camera]),
            .in_valid(in_valid[camera] && !camera_reset[camera]),
            .in_group_end(in_group_end[camera]),
            .in_x_q4(in_x_q4[camera]),
            .in_y_q4(in_y_q4[camera]),
            .in_log_mass_q5_3(in_log_mass_q5_3[camera]),
            .in_centroid_valid(in_centroid_valid[camera]),
            .in_scale(in_scale[camera]),
            .in_frame_seq(in_frame_seq[camera]),
            .head_valid(head_valid[camera]),
            .head_scale(head_scale[camera]),
            .head_frame_seq(head_frame_seq[camera]),
            .head_protocol(head_protocol[camera]),
            .head_mass_spread(head_mass_spread[camera]),
            .head_pop(head_pop[camera]),
            .desc_read_valid(desc_read_valid[camera]),
            .desc_read_index(core_desc_read_index),
            .desc_read_data_valid(desc_data_valid[camera]),
            .desc_read_data(desc_data[camera]),
            .desc_read_centroid_valid(desc_centroid_valid[camera]),
            .fifo_level(fifo_level[camera]),
            .fifo_full(unused_fifo_full[camera]),
            .clear_diagnostics(clear_diagnostics[camera]),
            .fifo_drop_sticky(fifo_drop_sticky[camera]),
            .protocol_error_sticky(protocol_error_sticky[camera]),
            .diagnostic_reset(rst),
            .diagnostic_context({(core_idle && job_count==0), core_out_ready, core_out_valid,
                backend_camera, (job_count!=0), fifo_level[1-camera]}),
            .debug_status(debug_status[camera]),
            .debug_protocol(debug_protocol[camera]),
            .debug_drop(debug_drop[camera])
        );
    end


    always_comb begin
        selection_valid=0;selected_owner=next_owner;
        if(head_valid[next_owner]&&!camera_reset[next_owner])selection_valid=1;
        else if(head_valid[!next_owner]&&!camera_reset[!next_owner])begin selection_valid=1;selected_owner=!next_owner;end
    end
    assign owner_enabled=owner_active&&!rst&&!camera_reset[owner];
    assign core_rst=rst||(owner_active&&camera_reset[owner]);
    assign core_head_valid=owner_enabled&&head_valid[owner]&&(!issued||!core_idle);
    assign backend_cancel=job_count!=0&&(!job_live[read_slot]||camera_reset[backend_camera]);
    assign backend_available=job_count!=0&&!backend_cancel;
    assign job_pop=job_count!=0&&(backend_cancel||backend_pop);
    always_comb begin
        busy=head_valid|(owner_active?(2'b01<<owner):2'b0);
        for(integer n=0;n<2;n=n+1)if(job_live[n])busy[job_camera[n]]=1;
        if(rst)busy=0;
        head_pop=0;desc_read_valid=0;candidate_start=0;candidate_done=0;
        candidate_cycles=last_candidate_cycles;
        out_valid=0;out_last=0;centroid_valid=0;centroid_last=0;
        if(owner_enabled)begin
            head_pop[owner]=core_head_pop;desc_read_valid[owner]=core_desc_read_valid;
            candidate_start[owner]=core_candidate_start;candidate_done[owner]=core_candidate_done;
            if(core_candidate_done)candidate_cycles[owner]=core_candidate_cycles;
        end
        if(backend_available&&!rst)begin
            out_valid[backend_camera]=back_out_valid;out_last[backend_camera]=back_out_last;
            centroid_valid[backend_camera]=back_centroid_valid;centroid_last[backend_camera]=back_centroid_last;
        end
    end
    always_ff @(posedge clk)begin
        if(job_write_valid&&owner_enabled)job_memory[{write_slot,job_write_address}]<=job_write_data;
        job_read_data<=job_memory[job_read_address[8]?job_read_address:{1'b0,read_slot,job_read_address[6:0]}];
        if(rst)begin
            owner_active<=0;owner<=0;issued<=0;next_owner<=0;last_candidate_cycles<=0;
            write_slot<=0;read_slot<=0;job_count<=0;job_live<=0;job_camera<=0;
        end else begin
            for(integer c=0;c<2;c=c+1)begin
                if(camera_reset[c])last_candidate_cycles[c]<=0;
                else if(owner_active&&owner==1'(c)&&core_candidate_done)last_candidate_cycles[c]<=core_candidate_cycles;
                if(camera_reset[job_camera[c]])job_live[c]<=0;
            end
            if(owner_active)begin
                if(camera_reset[owner]||(issued&&core_idle))begin owner_active<=0;issued<=0;end
                else if(core_dispatch_ready&&core_head_valid)issued<=1;
            end else if(core_dispatch_ready&&selection_valid)begin owner<=selected_owner;owner_active<=1;issued<=0;next_owner<=!selected_owner;end
            if(job_pop)begin job_live[read_slot]<=0;read_slot<=!read_slot;end
            if(job_commit&&owner_enabled)begin
                job_live[write_slot]<=1;job_camera[write_slot]<=owner;write_slot<=!write_slot;
            end
            case({job_commit&&owner_enabled,job_pop})
                2'b10:job_count<=job_count+1'b1;
                2'b01:job_count<=job_count-1'b1;
                default:begin end
            endcase
        end
    end
    // Only the frontend reads configuration. Accepted jobs cache calibration
    // before commit, so the independent backend needs no bank arbitration.
    assign request_camera=owner;
    assign request_address=front_config_address;
    assign request_valid=!config_pending&&front_config_valid&&!core_rst;
    assign config_read_valid=EXTERNAL_CONFIG&&request_valid;
    assign config_read_address=request_address[3]?{1'b1,request_camera,request_address[1:0]}:request_address;
    assign request_fire=request_valid&&(!EXTERNAL_CONFIG||config_read_ready);
    assign front_config_ready=request_fire;
    assign response_valid=config_pending&&(!EXTERNAL_CONFIG||config_response_valid);
    assign response_data=EXTERNAL_CONFIG?config_read_data:direct_config_data;
    assign front_config_response=response_valid;
    always_ff @(posedge clk)begin
        if(rst)begin config_pending<=0;direct_config_data<=0;end
        else begin
            if(response_valid)config_pending<=0;
            if(request_fire)begin
                config_pending<=1;
                case(request_address)
                    0:direct_config_data<=cfg_printed_width_q16_16[request_camera];
                    1:direct_config_data<={16'b0,cfg_mean_residual_limit_q16[request_camera]};
                    7:direct_config_data<={8'b0,cfg_min_baseline_q4[request_camera],cfg_max_mass_spread[request_camera]};
                    8:direct_config_data<=cfg_inv_fx_q2_30[request_camera];
                    9:direct_config_data<=cfg_inv_fy_q2_30[request_camera];
                    10:direct_config_data<=cfg_cx_q16_16[request_camera];
                    default:direct_config_data<=cfg_cy_q16_16[request_camera];
                endcase
            end
        end
    end
    LfTagDecodeFrontend frontend(.clk,.rst(core_rst),
        .head_valid(core_head_valid),.head_protocol(head_protocol[owner]),.head_scale(head_scale[owner]),
        .head_frame(head_frame_seq[owner]),.head_mass_spread(head_mass_spread[owner]),.head_pop(core_head_pop),
        .desc_read_valid(core_desc_read_valid),.desc_read_index(core_desc_read_index),
        .fifo_desc_data_valid(owner_enabled&&desc_data_valid[owner]),.fifo_desc_data(desc_data[owner]),
        .fifo_desc_centroid_valid(desc_centroid_valid[owner]),
        .config_read_valid(front_config_valid),.config_read_address(front_config_address),
        .config_read_ready(front_config_ready),.config_response_valid(front_config_response),.config_read_data(response_data),
        .job_ready(job_count<2),.job_valid(job_commit),.job_write_valid,.job_write_address,.job_write_data,
        .core_idle,.core_dispatch_ready,.candidate_start(core_candidate_start),.candidate_done(core_candidate_done),.candidate_cycles(core_candidate_cycles));
    LfTagAcceptedRefiner #(.EXPORT_ACCEPTED_CENTROIDS(EXPORT_ACCEPTED_CENTROIDS)) backend(
        .clk,.rst,.cancel(backend_cancel),.job_valid(backend_available),.job_pop(backend_pop),
        .job_read_address,.job_read_data,
        .out_valid(back_out_valid),.out_ready(out_ready[backend_camera]),.out_data,.out_last(back_out_last),
        .centroid_valid(back_centroid_valid),.centroid_ready(centroid_ready[backend_camera]),.centroid_data,.centroid_last(back_centroid_last));
`ifndef SYNTHESIS
    always_ff @(posedge clk)if(!rst)begin
        assert(job_count<=2);
        assert(!(job_commit&&job_count==2));
        assert($onehot0(out_valid));assert($onehot0(centroid_valid));
        assert($onehot0(head_pop));assert($onehot0(desc_read_valid));
    end
`endif
endmodule
