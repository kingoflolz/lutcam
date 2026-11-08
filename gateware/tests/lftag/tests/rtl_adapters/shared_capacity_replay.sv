// Simulation-only packed-port wrapper for arrival-timed shared solver replay.
module SharedCapacityReplay (
    input logic clk, rst,
    input logic [1:0] in_valid, in_group_end, in_centroid_valid,
    input logic [29:0] in_x_q4,
    input logic [27:0] in_y_q4,
    input logic [15:0] in_log_mass_q5_3,
    input logic [3:0] in_scale,
    input logic [15:0] in_frame_seq,
    input logic [1:0] out_ready,
    output wire [1:0] out_valid, out_last, busy,
    output wire [31:0] out_data,
    output wire [11:0] fifo_level,
    output wire [1:0] candidate_start, candidate_done,
    output wire [19:0] candidate_cycles,
    output wire [1:0] fifo_drop_sticky, protocol_error_sticky,
    output wire [63:0] debug_status, debug_protocol, debug_drop
);
    wire config_read_valid;
    wire [3:0] config_read_address;
    logic config_response_valid;
    logic [31:0] config_read_data;

    // Match the production calibration bank's one-clock read response while
    // capture owns the configuration. Host arbitration/reset initialization
    // have separate bank tests; these capacity tests use fixed loaded values.
    function automatic [31:0] config_word(input [3:0] address);
        case (address)
            0: config_word = 32'd65536;
            1: config_word = 32'h147ae148;
            2, 4, 6: config_word = 32'd0;
            3: config_word = 32'd43;
            5: config_word = 32'h04000000;
            7: config_word = {8'b0, 16'd16, 8'd19};
            8, 9, 12, 13: config_word = 32'd1150438;
            10, 14: config_word = 32'd41910272;
            11, 15: config_word = 32'd26181632;
        endcase
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            config_response_valid <= 1'b0;
            config_read_data <= '0;
        end else begin
            config_response_valid <= config_read_valid;
            if (config_read_valid) begin
                config_read_data <= config_word(config_read_address);
                if (config_read_address[3] && config_read_address[2] != dut.owner)
                    $fatal(1, "configuration request selected the wrong camera");
            end
        end
    end

    SharedLfTagDecoder #(
        .EXPORT_ACCEPTED_CENTROIDS(1'b1), .EXTERNAL_CONFIG(1'b1)
    ) dut (
        .config_read_valid, .config_read_address, .config_read_ready(1'b1),
        .config_response_valid, .config_read_data,
        .clk, .rst, .camera_reset(2'b00),
        .in_valid, .in_group_end, .in_centroid_valid, .in_x_q4, .in_y_q4,
        .in_log_mass_q5_3, .in_scale, .in_frame_seq,
        .cfg_inv_fx_q2_30({2{32'd1150438}}),
        .cfg_inv_fy_q2_30({2{32'd1150438}}),
        .cfg_cx_q16_16({2{32'd41910272}}),
        .cfg_cy_q16_16({2{32'd26181632}}),
        .cfg_printed_width_q16_16({2{32'd65536}}),
        .cfg_mean_residual_limit_q16({2{16'd1024}}),
        .cfg_max_mass_spread({2{8'd19}}),
        .cfg_min_baseline_q4({2{16'd16}}),
        .centroid_valid(), .centroid_ready(2'b11), .centroid_data(), .centroid_last(),
        .out_ready, .out_valid, .out_last, .out_data, .busy, .fifo_level,
        .candidate_start, .candidate_done, .candidate_cycles,
        .clear_diagnostics(2'b00), .fifo_drop_sticky, .protocol_error_sticky,
        .debug_status, .debug_protocol, .debug_drop
    );
endmodule
