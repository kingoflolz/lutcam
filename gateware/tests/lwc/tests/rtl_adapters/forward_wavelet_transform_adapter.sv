`default_nettype none

// Marlin 0.16 cannot read a packed output wider than 64 bits with the
// repository's Verilator version.  This test-only adapter exposes the twelve
// production coefficient lanes as scalar ports; it contains no test logic.
module forward_wavelet_transform_adapter (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       in_sof,
    input  wire                       in_valid,
    input  wire [7:0]                 in_px0,
    input  wire [7:0]                 in_px1,
    output wire [11:0]                out_raw_valid,
    output wire signed [13:0]         out_raw_value0,
    output wire signed [13:0]         out_raw_value1,
    output wire signed [13:0]         out_raw_value2,
    output wire signed [13:0]         out_raw_value3,
    output wire signed [13:0]         out_raw_value4,
    output wire signed [13:0]         out_raw_value5,
    output wire signed [13:0]         out_raw_value6,
    output wire signed [13:0]         out_raw_value7,
    output wire signed [13:0]         out_raw_value8,
    output wire signed [13:0]         out_raw_value9,
    output wire signed [13:0]         out_raw_value10,
    output wire signed [13:0]         out_raw_value11,
    output wire                       frame_done,
    output wire                       protocol_error
);
    wire signed [11:0][13:0] out_raw_value;
    wire unused_ll2_valid;
    wire unused_ll2_first;
    wire unused_ll2_last;
    wire [9:0] unused_ll2_pair_x;
    wire [9:0] unused_ll2_y;
    wire signed [1:0][13:0] unused_ll2_value;
    wire unused_h4_low_valid;
    wire unused_h4_low_first;
    wire unused_h4_low_last;
    wire [9:0] unused_h4_low_pair_x;
    wire [9:0] unused_h4_low_y;
    wire signed [1:0][13:0] unused_h4_low_value;
    wire unused_ll4_valid;
    wire unused_ll4_first;
    wire unused_ll4_last;
    wire [9:0] unused_ll4_pair_x;
    wire [9:0] unused_ll4_y;
    wire signed [1:0][13:0] unused_ll4_value;

    forward_wavelet_transform #(.BOUNDED_BYTE_RASTER(1'b1)) transform (
        .clk,
        .rst_n,
        .in_sof,
        .in_valid,
        .in_px0,
        .in_px1,
        .out_raw_valid,
        .out_raw_value,
        .out_ll2_valid(unused_ll2_valid),
        .out_ll2_first(unused_ll2_first),
        .out_ll2_last(unused_ll2_last),
        .out_ll2_pair_x(unused_ll2_pair_x),
        .out_ll2_y(unused_ll2_y),
        .out_ll2_value(unused_ll2_value),
        .out_h4_low_valid(unused_h4_low_valid),
        .out_h4_low_first(unused_h4_low_first),
        .out_h4_low_last(unused_h4_low_last),
        .out_h4_low_pair_x(unused_h4_low_pair_x),
        .out_h4_low_y(unused_h4_low_y),
        .out_h4_low_value(unused_h4_low_value),
        .out_ll4_valid(unused_ll4_valid),
        .out_ll4_first(unused_ll4_first),
        .out_ll4_last(unused_ll4_last),
        .out_ll4_pair_x(unused_ll4_pair_x),
        .out_ll4_y(unused_ll4_y),
        .out_ll4_value(unused_ll4_value),
        .frame_done,
        .protocol_error
    );

    assign out_raw_value0 = out_raw_value[0];
    assign out_raw_value1 = out_raw_value[1];
    assign out_raw_value2 = out_raw_value[2];
    assign out_raw_value3 = out_raw_value[3];
    assign out_raw_value4 = out_raw_value[4];
    assign out_raw_value5 = out_raw_value[5];
    assign out_raw_value6 = out_raw_value[6];
    assign out_raw_value7 = out_raw_value[7];
    assign out_raw_value8 = out_raw_value[8];
    assign out_raw_value9 = out_raw_value[9];
    assign out_raw_value10 = out_raw_value[10];
    assign out_raw_value11 = out_raw_value[11];
endmodule

`default_nettype wire
