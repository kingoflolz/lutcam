`default_nettype none

// Test-only scalar wrapper for the production compact queue profile.  Payload
// is tied to zero because the focused tests exercise only admission guards.
module compact_coefficient_pair_queues_adapter (
    input  wire          clk,
    input  wire          rst,
    input  wire          clear_frame,
    input  wire [11:0]   in_valid,
    input  wire          pair_ready,
    output wire          protocol_error
);
    wire unused_pair_valid;
    wire unused_pair_group_start;
    wire unused_pair_group_last;
    wire [3:0] unused_pair_band;
    wire signed [7:0] unused_pair_q0;
    wire signed [7:0] unused_pair_q1;
    wire [1:0] unused_pair_correction;
    wire [1:0] unused_pair_residual_class;

    coefficient_pair_queues #(
        .COMPACT_PRODUCTION_PROFILE(1'b1)
    ) queues (
        .clk,
        .rst,
        .clear_frame,
        .in_valid,
        .in_q('0),
        .in_correction('0),
        .in_residual_class('0),
        .pair_valid(unused_pair_valid),
        .pair_ready,
        .pair_group_start(unused_pair_group_start),
        .pair_group_last(unused_pair_group_last),
        .pair_band(unused_pair_band),
        .pair_q0(unused_pair_q0),
        .pair_q1(unused_pair_q1),
        .pair_correction(unused_pair_correction),
        .pair_residual_class(unused_pair_residual_class),
        .protocol_error
    );
endmodule

`default_nettype wire
