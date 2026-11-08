`default_nettype none

// Small fixed-raster wrapper for the Rust/Marlin end-to-end regression.  The
// production block and all of its memories remain unchanged.
module MonoIspSmall (
  input  logic        clk,
  input  logic        rst,
  input  logic        wb_cyc_i,
  input  logic        wb_stb_i,
  input  logic        wb_we_i,
  input  logic [3:0]  wb_sel_i,
  input  logic [15:0] wb_adr_i,
  input  logic [31:0] wb_dat_i,
  output logic [31:0] wb_dat_o,
  output logic        wb_ack_o,
  output logic        wb_err_o,
  input  logic        in_valid,
  output logic        in_ready,
  input  logic [9:0]  in_pixel0,
  input  logic [9:0]  in_pixel1,
  input  logic        in_sof,
  input  logic        in_eol,
  output logic        out_valid,
  input  logic        out_ready,
  output logic [7:0]  out_pixel0,
  output logic [7:0]  out_pixel1,
  output logic        out_sof,
  output logic        out_eol,
  input  logic        hist_dump_req,
  output logic        hist_dump_ready,
  output logic        hist_out_valid,
  input  logic        hist_out_ready,
  output logic [31:0] hist_out_data,
  output logic        protocol_error
);
  wire unused_position_sof, unused_position_eol, unused_position_eof;
  MonoIsp #(
    .WIDTH(16),
    .HEIGHT(4)
  ) dut (.*, .input_position_sof(unused_position_sof),
        .input_position_eol(unused_position_eol),
        .input_position_eof(unused_position_eof));
endmodule

`default_nettype wire
