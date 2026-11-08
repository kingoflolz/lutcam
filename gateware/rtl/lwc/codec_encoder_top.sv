`default_nettype none

// Fixed-profile 1280x800 causal scalar encoder. The standalone wrapper keeps
// only the coefficient taps it needs; TagDetectLwc instead places the unified
// frontend in front of codec_encoder_backend so both consumers share it.
module codec_encoder_top #(
    parameter bit BURST_LIMIT_ENABLE = 1'b1,
    parameter string GROUP_FAMILY_SCHEDULE_BIN =
        "../../../gateware/rtl/lwc/raw_group_family_schedule.bin"
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         in_sof,
    input  wire         in_valid,
    input  wire [7:0]   in_px0,
    input  wire [7:0]   in_px1,
    output logic        out_sof,
    output logic        out_valid,
    output logic [15:0] out_data,
    output logic        protocol_error
);
    logic clear_frame;
    logic [11:0] raw_valid;
    logic signed [11:0][13:0] raw_value;
    logic transform_error;
    logic backend_error;

    logic unused_transform_ll2_valid;
    logic unused_transform_ll2_first;
    logic unused_transform_ll2_last;
    logic [9:0] unused_transform_ll2_pair_x;
    logic [9:0] unused_transform_ll2_y;
    logic signed [1:0][13:0] unused_transform_ll2_value;
    logic unused_transform_h4_low_valid;
    logic unused_transform_h4_low_first;
    logic unused_transform_h4_low_last;
    logic [9:0] unused_transform_h4_low_pair_x;
    logic [9:0] unused_transform_h4_low_y;
    logic signed [1:0][13:0] unused_transform_h4_low_value;
    logic unused_transform_ll4_valid;
    logic unused_transform_ll4_first;
    logic unused_transform_ll4_last;
    logic [9:0] unused_transform_ll4_pair_x;
    logic [9:0] unused_transform_ll4_y;
    logic signed [1:0][13:0] unused_transform_ll4_value;
    logic unused_transform_frame_done;

    always_comb begin
        clear_frame = in_sof && in_valid;
        protocol_error = transform_error || backend_error;
    end

    forward_wavelet_transform transform (
        .clk,
        .rst_n(~rst),
        .in_sof,
        .in_valid,
        .in_px0,
        .in_px1,
        .out_raw_valid(raw_valid),
        .out_raw_value(raw_value),
        .out_ll2_valid(unused_transform_ll2_valid),
        .out_ll2_first(unused_transform_ll2_first),
        .out_ll2_last(unused_transform_ll2_last),
        .out_ll2_pair_x(unused_transform_ll2_pair_x),
        .out_ll2_y(unused_transform_ll2_y),
        .out_ll2_value(unused_transform_ll2_value),
        .out_h4_low_valid(unused_transform_h4_low_valid),
        .out_h4_low_first(unused_transform_h4_low_first),
        .out_h4_low_last(unused_transform_h4_low_last),
        .out_h4_low_pair_x(unused_transform_h4_low_pair_x),
        .out_h4_low_y(unused_transform_h4_low_y),
        .out_h4_low_value(unused_transform_h4_low_value),
        .out_ll4_valid(unused_transform_ll4_valid),
        .out_ll4_first(unused_transform_ll4_first),
        .out_ll4_last(unused_transform_ll4_last),
        .out_ll4_pair_x(unused_transform_ll4_pair_x),
        .out_ll4_y(unused_transform_ll4_y),
        .out_ll4_value(unused_transform_ll4_value),
        .frame_done(unused_transform_frame_done),
        .protocol_error(transform_error)
    );

    codec_encoder_backend #(
        .BURST_LIMIT_ENABLE(BURST_LIMIT_ENABLE),
        .GROUP_FAMILY_SCHEDULE_BIN(GROUP_FAMILY_SCHEDULE_BIN)
    ) backend (
        .clk,
        .rst,
        .clear_frame,
        .in_raw_valid(raw_valid),
        .in_raw_value(raw_value),
        .out_sof,
        .out_valid,
        .out_data,
        .protocol_error(backend_error)
    );

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (!rst && !clear_frame) begin
            assert (!transform_error)
                else $error("forward transform protocol error");
        end
    end
`endif
endmodule

`default_nettype wire
