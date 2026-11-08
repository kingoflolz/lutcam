`default_nettype none
module first_vertical_lift #(parameter integer OUTPUT_WIDTH = 14) (
    input wire clk, input wire rst_n, input wire in_valid,
    input wire [1:0] in_op, input wire [9:0] in_state_address,
    input wire [1:0] in_context, input wire [9:0] in_column,
    input wire [8:0] in_output_y, input wire in_output_first,
    input wire in_output_last, input wire signed [1:0][OUTPUT_WIDTH-1:0] in_value,
    output wire out_valid, output wire [1:0] out_context,
    output wire [9:0] out_column, output wire [8:0] out_y,
    output wire out_first, output wire out_last,
    output wire signed [1:0][OUTPUT_WIDTH-1:0] out_low,
    output wire signed [1:0][OUTPUT_WIDTH-1:0] out_high
);
    wire signed [1:0][9:0] value10;
    wire signed [1:0][9:0] low10;
    wire signed [1:0][9:0] high10;
    assign value10[0] = in_value[0][9:0];
    assign value10[1] = in_value[1][9:0];
    vertical_lift_scheduler #(
        .LANES(2), .DATA_WIDTH(10), .RAW_WIDTH(9), .HIGH_WIDTH(10),
        .STATE_DEPTH(640), .ADDRESS_WIDTH(10), .CONTEXT_WIDTH(2),
        .COLUMN_WIDTH(10), .ROW_WIDTH(9), .SPLIT_STATE_FIELDS(1)
    ) core (
        .clk, .rst_n, .in_valid, .in_op, .in_state_address, .in_context,
        .in_column, .in_output_y, .in_output_first, .in_output_last,
        .in_value(value10), .out_valid, .out_context, .out_column, .out_y,
        .out_first, .out_last, .out_low(low10), .out_high(high10)
    );
    assign out_low[0] = OUTPUT_WIDTH'($signed(low10[0]));
    assign out_low[1] = OUTPUT_WIDTH'($signed(low10[1]));
    assign out_high[0] = OUTPUT_WIDTH'($signed(high10[0]));
    assign out_high[1] = OUTPUT_WIDTH'($signed(high10[1]));
endmodule
`default_nettype wire
