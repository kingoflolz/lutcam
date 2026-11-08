`default_nettype none

// Shared image-analysis frontend for the LWC codec and adaptive threshold.
//
// The twelve raw coefficient lanes are the unmodified codec interface. LL2 is
// tapped from the existing 320x200 gearbox. LL4 is an inexpensive 80x50
// approximation made by averaging adjacent rows of the existing 80x100 H4-low
// stream. The previous H4-low row occupies unused addresses in the transform's
// existing local-row RAM, so LL4 adds no standalone row memory.
//
// Both adaptive outputs are row-major two-sample transfers. pair_x is the pair
// index, so value[0] is scalar x=2*pair_x and value[1] is x+1.
module unified_wavelet_frontend #(
    parameter integer FRAME_WIDTH = 1280,
    parameter integer FRAME_HEIGHT = 800,
    parameter integer DATA_WIDTH = 14,
    parameter integer COORD_WIDTH = 10,
    parameter bit BOUNDED_BYTE_RASTER = 1'b0
) (
    input  wire                                      clk,
    input  wire                                      rst_n,
    input  wire                                      in_sof,
    input  wire                                      in_valid,
    input  wire [7:0]                                in_px0,
    input  wire [7:0]                                in_px1,
    output logic [11:0]                              out_raw_valid,
    output logic signed [11:0][DATA_WIDTH-1:0]       out_raw_value,
    output logic                                     out_ll2_valid,
    output logic                                     out_ll2_first,
    output logic                                     out_ll2_last,
    output logic [COORD_WIDTH-1:0]                   out_ll2_pair_x,
    output logic [COORD_WIDTH-1:0]                   out_ll2_y,
    output logic signed [1:0][DATA_WIDTH-1:0]        out_ll2_value,
    output logic                                     out_ll4_valid,
    output logic                                     out_ll4_first,
    output logic                                     out_ll4_last,
    output logic [COORD_WIDTH-1:0]                   out_ll4_pair_x,
    output logic [COORD_WIDTH-1:0]                   out_ll4_y,
    output logic signed [1:0][DATA_WIDTH-1:0]        out_ll4_value,
    output logic                                     frame_done,
    output logic                                     protocol_error
);
    logic unused_h4_low_valid;
    logic unused_h4_low_first;
    logic unused_h4_low_last;
    logic [COORD_WIDTH-1:0] unused_h4_low_pair_x;
    logic [COORD_WIDTH-1:0] unused_h4_low_y;
    logic signed [1:0][DATA_WIDTH-1:0] unused_h4_low_value;

    forward_wavelet_transform #(
        .FRAME_WIDTH(FRAME_WIDTH),
        .FRAME_HEIGHT(FRAME_HEIGHT),
        .DATA_WIDTH(DATA_WIDTH),
        .COORD_WIDTH(COORD_WIDTH),
        .BOUNDED_BYTE_RASTER(BOUNDED_BYTE_RASTER)
    ) transform (
        .clk,
        .rst_n,
        .in_sof,
        .in_valid,
        .in_px0,
        .in_px1,
        .out_raw_valid,
        .out_raw_value,
        .out_ll2_valid,
        .out_ll2_first,
        .out_ll2_last,
        .out_ll2_pair_x,
        .out_ll2_y,
        .out_ll2_value,
        .out_h4_low_valid(unused_h4_low_valid),
        .out_h4_low_first(unused_h4_low_first),
        .out_h4_low_last(unused_h4_low_last),
        .out_h4_low_pair_x(unused_h4_low_pair_x),
        .out_h4_low_y(unused_h4_low_y),
        .out_h4_low_value(unused_h4_low_value),
        .out_ll4_valid,
        .out_ll4_first,
        .out_ll4_last,
        .out_ll4_pair_x,
        .out_ll4_y,
        .out_ll4_value,
        .frame_done,
        .protocol_error
    );
endmodule

`default_nettype wire
