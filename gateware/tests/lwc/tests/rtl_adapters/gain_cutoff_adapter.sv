`default_nettype none

// Exercise the production gain decision independently of the packet schedule.
module gain_cutoff_adapter (
    input wire [10:0] energy,
    input wire [7:0] extra_bits,
    input wire [3:0] cutoff,
    input wire [1:0] metric_shift,
    output wire passes
);
    causal_group_encoder dut (
        .clk(1'b0), .rst(1'b0), .clear_frame(1'b0),
        .pair_valid(1'b0), .pair_ready(),
        .pair_group_start(1'b0), .pair_group_last(1'b0),
        .pair_band(4'b0), .pair_q0(8'b0), .pair_q1(8'b0),
        .pair_correction(2'b0), .pair_residual_class(2'b0),
        .out_sof(), .out_valid(), .out_padding(), .out_ready(1'b0),
        .out_data(), .frame_done(), .packets_done(), .packet_bits(),
        .groups_committed(), .groups_serialized(), .rate_debt(),
        .extra_room(), .protocol_error()
    );
    assign passes = dut.gain_passes_cutoff(energy, extra_bits, cutoff,
        metric_shift);
endmodule

`default_nettype wire
