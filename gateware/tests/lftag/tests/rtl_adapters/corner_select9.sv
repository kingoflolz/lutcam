// Test-only wide-output split for Marlin versions whose dynamic WData FFI
// cannot expose a 72-bit port with current Verilator releases.
module LfTagCornerSelect9Marlin (
    input  logic               clk,
    input  logic               rst,
    input  logic               in_valid,
    output logic               in_ready,
    input  logic               in_points_valid,
    input  logic [15:0]        in_min_baseline_q4,
    output logic               desc_req_valid,
    output logic [3:0]         desc_req_index,
    input  logic               desc_rsp_valid,
    input  logic signed [17:0] desc_rsp_x_q4,
    input  logic signed [17:0] desc_rsp_y_q4,
    input  logic [7:0]         desc_rsp_mass_q5_3,
    input  logic               desc_rsp_centroid_valid,
    output logic               out_valid,
    input  logic               out_ready,
    output logic [2:0]         out_status,
    output logic               out_input_valid,
    output logic               out_baseline_valid,
    output logic               out_dominant_valid,
    output logic               out_corners_valid,
    output logic               out_quad_valid,
    output logic [15:0]        out_corner_index_flat,
    output logic [19:0]        out_remaining_index_flat,
    output logic [17:0]        out_corner_x0_q4,
    output logic [17:0]        out_corner_x1_q4,
    output logic [17:0]        out_corner_x2_q4,
    output logic [17:0]        out_corner_x3_q4,
    output logic [17:0]        out_corner_y0_q4,
    output logic [17:0]        out_corner_y1_q4,
    output logic [17:0]        out_corner_y2_q4,
    output logic [17:0]        out_corner_y3_q4,
    output logic [7:0]         out_cycles
);
    logic [71:0] out_corner_x_q4_flat;
    logic [71:0] out_corner_y_q4_flat;

    assign out_corner_x0_q4 = out_corner_x_q4_flat[17:0];
    assign out_corner_x1_q4 = out_corner_x_q4_flat[35:18];
    assign out_corner_x2_q4 = out_corner_x_q4_flat[53:36];
    assign out_corner_x3_q4 = out_corner_x_q4_flat[71:54];
    assign out_corner_y0_q4 = out_corner_y_q4_flat[17:0];
    assign out_corner_y1_q4 = out_corner_y_q4_flat[35:18];
    assign out_corner_y2_q4 = out_corner_y_q4_flat[53:36];
    assign out_corner_y3_q4 = out_corner_y_q4_flat[71:54];

    LfTagCornerSelect9 dut (.*);
endmodule
