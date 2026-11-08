`default_nettype none

// Test-only scalar-port adapter for the two packed adaptive-analysis lanes.
module unified_wavelet_frontend_adapter (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       in_sof,
    input  wire                       in_valid,
    input  wire [7:0]                 in_px0,
    input  wire [7:0]                 in_px1,
    output wire                       out_ll2_valid,
    output wire                       out_ll2_first,
    output wire                       out_ll2_last,
    output wire [9:0]                 out_ll2_pair_x,
    output wire [9:0]                 out_ll2_y,
    output wire signed [13:0]         out_ll2_value0,
    output wire signed [13:0]         out_ll2_value1,
    output wire                       out_ll4_valid,
    output wire                       out_ll4_first,
    output wire                       out_ll4_last,
    output wire [9:0]                 out_ll4_pair_x,
    output wire [9:0]                 out_ll4_y,
    output wire signed [13:0]         out_ll4_value0,
    output wire signed [13:0]         out_ll4_value1,
    output wire                       frame_done,
    output wire                       protocol_error
);
    wire [11:0] unused_out_raw_valid;
    wire signed [11:0][13:0] unused_out_raw_value;
    wire signed [1:0][13:0] out_ll2_value;
    wire signed [1:0][13:0] out_ll4_value;

    unified_wavelet_frontend frontend (
        .clk,
        .rst_n,
        .in_sof,
        .in_valid,
        .in_px0,
        .in_px1,
        .out_raw_valid(unused_out_raw_valid),
        .out_raw_value(unused_out_raw_value),
        .out_ll2_valid,
        .out_ll2_first,
        .out_ll2_last,
        .out_ll2_pair_x,
        .out_ll2_y,
        .out_ll2_value,
        .out_ll4_valid,
        .out_ll4_first,
        .out_ll4_last,
        .out_ll4_pair_x,
        .out_ll4_y,
        .out_ll4_value,
        .frame_done,
        .protocol_error
    );

    assign out_ll2_value0 = out_ll2_value[0];
    assign out_ll2_value1 = out_ll2_value[1];
    assign out_ll4_value0 = out_ll4_value[0];
    assign out_ll4_value1 = out_ll4_value[1];
endmodule

`default_nettype wire
