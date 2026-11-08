`default_nettype none

// Observe actual decision edges for the canonical scalar-reference cadence.
// The production encoder never reads the resulting timing fixture.
module lwc_group_timing_tb (
    input wire clk, rst, in_sof, in_valid,
    input wire [7:0] in_px0, in_px1,
    output wire protocol_error,
    output wire [15:0] groups_committed
);
    wire out_sof, out_valid;
    wire [15:0] out_data;
    codec_encoder_top #(
        .GROUP_FAMILY_SCHEDULE_BIN("rtl/lwc/raw_group_family_schedule.bin")
    ) dut (
        .clk, .rst, .in_sof, .in_valid, .in_px0, .in_px1,
        .out_sof, .out_valid, .out_data, .protocol_error
    );
    assign groups_committed = dut.backend.unused_in_synthesis_groups_committed;
endmodule

`default_nettype wire
