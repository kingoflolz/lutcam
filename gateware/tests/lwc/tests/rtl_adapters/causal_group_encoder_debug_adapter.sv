`default_nettype none

// Port-only Marlin adapter for observing the production packer's handshake
// and occupancy. No state or behavior is added to the encoder.
module causal_group_encoder_debug_adapter (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear_frame,
    input  wire                         pair_valid,
    output wire                         pair_ready,
    input  wire                         pair_group_start,
    input  wire                         pair_group_last,
    input  wire [3:0]                   pair_band,
    input  wire signed [7:0]            pair_q0,
    input  wire signed [7:0]            pair_q1,
    input  wire [1:0]                   pair_correction,
    input  wire [1:0]                   pair_residual_class,
    output wire                         out_sof,
    output wire                         out_valid,
    output wire                         out_padding,
    input  wire                         out_ready,
    output wire [15:0]                  out_data,
    output wire                         frame_done,
    output wire                         packets_done,
    output wire [20:0]                  packet_bits,
    output wire [15:0]                  groups_committed,
    output wire [15:0]                  groups_serialized,
    output wire signed [20:0]           rate_debt,
    output wire [19:0]                  extra_room,
    output wire                         protocol_error,
    output wire [5:0]                   debug_output_count,
    output wire [4:0]                   debug_token_count,
    output wire                         debug_token_valid,
    output wire                         debug_token_ready,
    output wire                         debug_token_fire,
    output wire [7:0]                   debug_final_fine_bits,
    output wire [7:0]                   debug_final_coarse_bits,
    output wire [1:0]                   debug_final_selected_state,
    output wire [7:0]                   debug_final_selected_bits,
    output wire                         debug_final_selected_huffman,
    output wire [1:0]                   debug_finalizer_state,
    output wire [10:0]                  debug_pair_direct_energy0,
    output wire [10:0]                  debug_pair_direct_energy1
);
    causal_group_encoder dut (
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
        .out_padding,
        .out_ready,
        .out_data,
        .frame_done,
        .packets_done,
        .packet_bits,
        .groups_committed,
        .groups_serialized,
        .rate_debt,
        .extra_room,
        .protocol_error
    );

    assign debug_output_count = dut.output_count;
    assign debug_token_count = dut.token_count;
    assign debug_token_valid = dut.token_valid;
    assign debug_token_ready = dut.token_ready;
    assign debug_token_fire = dut.token_fire;
    assign debug_final_fine_bits = dut.final_fine_bits;
    assign debug_final_coarse_bits = dut.final_coarse_bits;
    assign debug_final_selected_state = dut.final_selected_state;
    assign debug_final_selected_bits = dut.final_selected_bits;
    assign debug_final_selected_huffman = dut.final_selected_huffman;
    assign debug_finalizer_state = dut.finalizer_state;
    assign debug_pair_direct_energy0 = dut.pair_direct_energy0;
    assign debug_pair_direct_energy1 = dut.pair_direct_energy1;
endmodule

`default_nettype wire
