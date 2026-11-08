`default_nettype none

// Coefficient-domain half of the fixed-profile LWC encoder. Keeping the
// transform outside this module lets the combined TagDetect/LWC top share one
// wavelet frontend while the standalone codec retains its original interface.
module codec_encoder_backend #(
    parameter bit BURST_LIMIT_ENABLE = 1'b1,
    parameter string GROUP_FAMILY_SCHEDULE_BIN =
        "../../../gateware/rtl/lwc/raw_group_family_schedule.bin"
) (
    input  wire                       clk,
    input  wire                       rst,
    input  wire                       clear_frame,
    input  wire [11:0]                in_raw_valid,
    input  wire signed [11:0][13:0]   in_raw_value,
    output logic                      out_sof,
    output logic                      out_valid,
    output logic [15:0]               out_data,
    output logic                      protocol_error
);
    logic [11:0] quantized_valid;
    logic signed [11:0][7:0] quantized_q;
    logic [11:0] quantized_correction;
    logic [11:0] quantized_residual_class;
    logic unused_in_synthesis_quantizer_error;

    logic pair_valid;
    logic pair_ready;
    logic pair_group_start;
    logic pair_group_last;
    logic [3:0] pair_band;
    logic signed [7:0] pair_q0;
    logic signed [7:0] pair_q1;
    logic [1:0] pair_correction;
    logic [1:0] pair_residual_class;
    logic pair_queue_error;

    logic unused_in_synthesis_packets_done;
    logic unused_in_synthesis_out_padding;
    logic [20:0] unused_in_synthesis_selected_packet_bits;
    logic [15:0] unused_in_synthesis_groups_committed;
    logic [15:0] unused_in_synthesis_groups_serialized;
    logic signed [20:0] unused_in_synthesis_rate_debt;
    logic [19:0] unused_in_synthesis_extra_room;
    logic unused_in_synthesis_frame_done;
    logic unused_in_synthesis_group_encoder_error;

    always_comb begin
        // Queue overflow is the only content/backpressure-dependent hardware
        // capacity failure. Keep it externally visible so synthesis cannot
        // remove the full/wrap guards as an unobservable diagnostic cone.
        protocol_error = pair_queue_error
            || unused_in_synthesis_group_encoder_error;
    end

    coefficient_quantizer quantizer (
        .clk,
        .rst,
        .clear_frame,
        .in_valid(in_raw_valid),
        .in_value(in_raw_value),
        .out_valid(quantized_valid),
        .out_q(quantized_q),
        .out_correction(quantized_correction),
        .out_residual_class(quantized_residual_class),
        .protocol_error(unused_in_synthesis_quantizer_error)
    );

    coefficient_pair_queues #(
        .GROUP_FAMILY_SCHEDULE_BIN(GROUP_FAMILY_SCHEDULE_BIN),
        .COMPACT_PRODUCTION_PROFILE(1'b1)
    ) pair_queues (
        .clk,
        .rst,
        .clear_frame,
        .in_valid(quantized_valid),
        .in_q(quantized_q),
        .in_correction(quantized_correction),
        .in_residual_class(quantized_residual_class),
        .pair_valid,
        .pair_ready,
        .pair_group_start,
        .pair_group_last,
        .pair_band,
        .pair_q0,
        .pair_q1,
        .pair_correction,
        .pair_residual_class,
        .protocol_error(pair_queue_error)
    );

    causal_group_encoder #(.BURST_LIMIT_ENABLE(BURST_LIMIT_ENABLE)) group_encoder (
        .clk,
        .rst,
        .clear_frame,
        .pair_valid,
        .pair_ready,
        .pair_group_start,
        .pair_group_last,
        .pair_band,
        .pair_q0,
        .pair_q1,
        .pair_correction,
        .pair_residual_class,
        .out_sof,
        .out_valid,
        .out_ready(1'b1),
        .out_data,
        .out_padding(unused_in_synthesis_out_padding),
        .frame_done(unused_in_synthesis_frame_done),
        .packets_done(unused_in_synthesis_packets_done),
        .packet_bits(unused_in_synthesis_selected_packet_bits),
        .groups_committed(unused_in_synthesis_groups_committed),
        .groups_serialized(unused_in_synthesis_groups_serialized),
        .rate_debt(unused_in_synthesis_rate_debt),
        .extra_room(unused_in_synthesis_extra_room),
        .protocol_error(unused_in_synthesis_group_encoder_error)
    );

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (!rst && !clear_frame) begin
            assert (!unused_in_synthesis_quantizer_error)
                else $error("quantizer protocol error");
            assert (!pair_queue_error)
                else $error("pair queue protocol error committed=%0d serialized=%0d selected_bits=%0d debt=%0d room=%0d",
                    unused_in_synthesis_groups_committed,
                    unused_in_synthesis_groups_serialized,
                    unused_in_synthesis_selected_packet_bits,
                    unused_in_synthesis_rate_debt,
                    unused_in_synthesis_extra_room);
            assert (!unused_in_synthesis_group_encoder_error)
                else $error("causal group encoder protocol error");
            assert (unused_in_synthesis_groups_serialized
                <= unused_in_synthesis_groups_committed);
            assert (unused_in_synthesis_groups_committed
                - unused_in_synthesis_groups_serialized <= 16'd257);
            if (unused_in_synthesis_packets_done) begin
                assert (unused_in_synthesis_groups_committed == 16'd64_000);
                assert (unused_in_synthesis_groups_serialized == 16'd64_000);
            end
        end
    end
`endif
endmodule

`default_nettype wire
