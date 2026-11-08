`default_nettype none
// Exercise the sticky error output for deliberately conflicting input bands.
// Keep the portable arithmetic branch; SYNTHESIS selects vendor primitives.
`define LUTCAM_TEST_ALLOW_QUANTIZER_CONFLICTS

// Port-shape adapter only. Marlin 0.16's dynamic FFI cannot compile wide
// packed ports, so expose each coefficient lane separately.
module coefficient_quantizer_adapter (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     clear_frame,
    input  wire [11:0]              in_valid,
    input  wire signed [13:0]       in_value_0,
    input  wire signed [13:0]       in_value_1,
    input  wire signed [13:0]       in_value_2,
    input  wire signed [13:0]       in_value_3,
    input  wire signed [13:0]       in_value_4,
    input  wire signed [13:0]       in_value_5,
    input  wire signed [13:0]       in_value_6,
    input  wire signed [13:0]       in_value_7,
    input  wire signed [13:0]       in_value_8,
    input  wire signed [13:0]       in_value_9,
    input  wire signed [13:0]       in_value_10,
    input  wire signed [13:0]       in_value_11,
    output wire [11:0]              out_valid,
    output wire signed [7:0]        out_q_0,
    output wire signed [7:0]        out_q_1,
    output wire signed [7:0]        out_q_2,
    output wire signed [7:0]        out_q_3,
    output wire signed [7:0]        out_q_4,
    output wire signed [7:0]        out_q_5,
    output wire signed [7:0]        out_q_6,
    output wire signed [7:0]        out_q_7,
    output wire signed [7:0]        out_q_8,
    output wire signed [7:0]        out_q_9,
    output wire signed [7:0]        out_q_10,
    output wire signed [7:0]        out_q_11,
    output wire [11:0]              out_correction,
    output wire [11:0]              out_residual_class,
    output wire                     protocol_error
);
    wire signed [11:0][13:0] in_value;
    wire signed [11:0][7:0] out_q;

    assign in_value = {
        in_value_11, in_value_10, in_value_9, in_value_8,
        in_value_7, in_value_6, in_value_5, in_value_4,
        in_value_3, in_value_2, in_value_1, in_value_0
    };

    coefficient_quantizer dut (
        .clk(clk),
        .rst(rst),
        .clear_frame(clear_frame),
        .in_valid(in_valid),
        .in_value(in_value),
        .out_valid(out_valid),
        .out_q(out_q),
        .out_correction(out_correction),
        .out_residual_class(out_residual_class),
        .protocol_error(protocol_error)
    );

    assign out_q_0 = out_q[0];
    assign out_q_1 = out_q[1];
    assign out_q_2 = out_q[2];
    assign out_q_3 = out_q[3];
    assign out_q_4 = out_q[4];
    assign out_q_5 = out_q[5];
    assign out_q_6 = out_q[6];
    assign out_q_7 = out_q[7];
    assign out_q_8 = out_q[8];
    assign out_q_9 = out_q[9];
    assign out_q_10 = out_q[10];
    assign out_q_11 = out_q[11];
endmodule

`default_nettype wire
