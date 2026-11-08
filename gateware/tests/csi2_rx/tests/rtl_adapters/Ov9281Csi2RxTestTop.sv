`default_nettype none

module Ov9281Csi2RxTestTop (
    input  wire         reset_n,
    input  wire         ctrl_clk,
    input  wire         byte_clk,
    input  wire         pixel_clk,
    input  wire         status_clear,
    input  wire         clk_lp_p,
    input  wire         clk_lp_n,
    output wire         clk_hs_term,
    output wire         clk_hs_ena,
    input  wire  [1:0]  data_lp_p,
    input  wire  [1:0]  data_lp_n,
    input  wire  [7:0]  data0_hs_in,
    input  wire  [7:0]  data1_hs_in,
    input  wire  [1:0]  data_fifo_empty,
    output wire  [1:0]  data_fifo_rd,
    output wire  [1:0]  data_rst,
    output wire  [1:0]  data_hs_term,
    output wire  [1:0]  data_hs_ena,
    output wire         init_done,
    output wire [15:0]  status,
    output wire [19:0]  pixel_data,
    output wire         pixel_valid,
    output wire         pixel_sof,
    output wire         pixel_sol,
    output wire         pixel_eol,
    output wire         pixel_eof,
    output wire [3:0]   burst_sources,
    output wire [11:0]  burst_destinations,
    output wire [3:0]   burst_pulses,
    output wire         byte_session_active,
    output wire [2:0] debug_parser_state,
    output wire [2:0] debug_gearbox_phase,
    output wire debug_pair_pop
);
    csi2_rx #(
        .INIT_GUARD_CYCLES(4),
        .LPX_CYCLES(2),
        .SOT_SETTLE_CYCLES(2),
        .CLOCK_TIMEOUT_CYCLES(64),
        // Exercise the depth selected by the production 120 MHz Ti60 shell.
        .PIXEL_FIFO_DEPTH(64)
    ) dut (.*);

    assign debug_parser_state = dut.parser_state;
    assign debug_gearbox_phase = dut.gearbox_phase;
    assign debug_pair_pop = dut.parser_pair_pop;
    // Observe both ends of the CDC boundary without changing production RTL.
    assign burst_sources = {dut.burst_end_toggle_ctrl, dut.burst_start_toggle_ctrl};
    assign burst_destinations = {dut.burst_end_seen, dut.burst_end_sync1,
        dut.burst_end_sync0, dut.burst_start_seen, dut.burst_start_sync1,
        dut.burst_start_sync0};
    assign burst_pulses = {dut.burst_end_pulse, dut.burst_start_pulse};
    assign byte_session_active = dut.byte_session_reset_n;
endmodule

`default_nettype wire
