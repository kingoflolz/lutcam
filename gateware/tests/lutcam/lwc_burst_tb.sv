`default_nettype none

// Observe the real encoder without introducing a different coefficient source
// or output pacing. Inputs and outputs are sampled by lwc_burst_probe.cpp.
module lwc_burst_tb #(
    parameter bit BURST_LIMIT_ENABLE = 1'b0
) (
    input wire clk,
    input wire rst,
    input wire in_sof,
    input wire in_valid,
    input wire [7:0] in_px0,
    input wire [7:0] in_px1,
    output wire out_sof,
    output wire out_valid,
    output wire [15:0] out_data,
    output wire out_padding,
    output wire protocol_error,
    output wire packets_done,
    output wire [15:0] groups_serialized,
    output wire [20:0] packet_bits
);
    codec_encoder_top #(
        .BURST_LIMIT_ENABLE(BURST_LIMIT_ENABLE),
        .GROUP_FAMILY_SCHEDULE_BIN("rtl/lwc/raw_group_family_schedule.bin")
    ) dut (
        .clk, .rst, .in_sof, .in_valid, .in_px0, .in_px1,
        .out_sof, .out_valid, .out_data, .protocol_error
    );

    assign out_padding = dut.backend.group_encoder.out_padding;
    assign packets_done = dut.backend.unused_in_synthesis_packets_done;
    assign groups_serialized = dut.backend.unused_in_synthesis_groups_serialized;
    assign packet_bits = dut.backend.unused_in_synthesis_selected_packet_bits;
endmodule

`default_nettype wire
