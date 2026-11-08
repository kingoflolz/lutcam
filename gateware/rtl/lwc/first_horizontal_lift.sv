`default_nettype none
module first_horizontal_lift #(parameter integer OUTPUT_WIDTH = 14) (
    input wire clk, input wire rst_n, input wire in_valid,
    input wire in_first, input wire in_last,
    input wire signed [8:0] in_even, input wire signed [8:0] in_odd,
    output wire out_valid, output wire out_first, output wire out_last,
    output wire signed [OUTPUT_WIDTH-1:0] out_low, output wire signed [OUTPUT_WIDTH-1:0] out_high
);
    wire signed [8:0] low9;
    wire signed [8:0] high9;
    horizontal_lift_stage #(.DATA_WIDTH(9)) core (
        .clk, .rst_n, .in_valid, .in_first, .in_last,
        .in_even, .in_odd,
        .out_valid, .out_first, .out_last, .out_low(low9), .out_high(high9)
    );
    assign out_low = OUTPUT_WIDTH'($signed(low9));
    assign out_high = OUTPUT_WIDTH'($signed(high9));
endmodule
`default_nettype wire
