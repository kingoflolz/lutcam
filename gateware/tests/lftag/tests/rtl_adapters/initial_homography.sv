`default_nettype none

// This structural adapter keeps packed datapaths inside Verilog while exposing
// scalar transaction ports to Rust. It contains no stimulus or sequencing.
module LfTagInitialHomographyFusedMarlin (
    input  logic clk,
    input  logic rst,
    input  logic in_valid,
    output logic in_ready,
    input  logic [17:0] in_key_x0_q4,
    input  logic [17:0] in_key_x1_q4,
    input  logic [17:0] in_key_x2_q4,
    input  logic [17:0] in_key_x3_q4,
    input  logic [17:0] in_key_y0_q4,
    input  logic [17:0] in_key_y1_q4,
    input  logic [17:0] in_key_y2_q4,
    input  logic [17:0] in_key_y3_q4,
    input  logic [17:0] in_cden,
    input  logic [17:0] in_cg,
    input  logic [17:0] in_ch,
    input  logic [3:0] in_hypothesis,
    input  logic [1:0] in_scale,
    input  logic [31:0] in_inv_fx_q2_30,
    input  logic [31:0] in_inv_fy_q2_30,
    input  logic [31:0] in_cx_q16,
    input  logic [31:0] in_cy_q16,
    input  logic in_saturation,
    output logic out_valid,
    input  logic out_ready,
    output logic result_write_valid,
    output logic [3:0] result_write_address,
    output logic [17:0] result_write_data,
    output logic out_invalid,
    output logic out_saturation,
    output logic [6:0] out_cycles
);
    logic [71:0] in_key_x_q4_flat;
    logic [71:0] in_key_y_q4_flat;
    logic [6:0] inverse_read_address;
    logic signed [17:0] inverse_read_data_a;
    logic signed [17:0] inverse_read_data_b;
    logic [17:0] pose_result_bank_a [0:127];
    logic [17:0] pose_result_bank_b [0:127];

    initial begin
`include "lftag_h_inverse_pair_init.svh"
    end

    always_ff @(posedge clk) begin
        inverse_read_data_a <= pose_result_bank_a[inverse_read_address];
        inverse_read_data_b <= pose_result_bank_b[inverse_read_address];
    end

    assign in_key_x_q4_flat = {
        in_key_x3_q4, in_key_x2_q4, in_key_x1_q4, in_key_x0_q4
    };
    assign in_key_y_q4_flat = {
        in_key_y3_q4, in_key_y2_q4, in_key_y1_q4, in_key_y0_q4
    };
    LfTagInitialHomographyFused dut (
        .config_read_valid(), .config_read_address(), .config_read_ready(1'b0),
        .config_response_valid(1'b0), .config_read_data(32'b0),.*);
endmodule

`default_nettype wire
