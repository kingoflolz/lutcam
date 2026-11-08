// Synchronous feature-value memory helper for ChildArena.

import lrcca_util_pkg::bit_width;

module ChildArenaFvMem #(
  parameter int ENTRY_COUNT = 108,
  parameter int ADDR_BITS = bit_width(longint'(ENTRY_COUNT)),
  parameter int DATA_WIDTH = 34
)(
  input  logic clk,
  input  logic port_a_w_en,
  input  logic [DATA_WIDTH-1:0] port_a_w_data,
  input  logic [ADDR_BITS-1:0] port_a_addr,
  output logic [DATA_WIDTH-1:0] port_a_r_data,
  input  logic port_b_w_en,
  input  logic [DATA_WIDTH-1:0] port_b_w_data,
  input  logic [ADDR_BITS-1:0] port_b_addr,
  output logic [DATA_WIDTH-1:0] port_b_r_data
);
  localparam int RAM_DEPTH = 1 << ADDR_BITS;

  logic [DATA_WIDTH-1:0] mem [0:RAM_DEPTH-1];
  logic [ADDR_BITS-1:0] port_a_raddr;
  logic [ADDR_BITS-1:0] port_b_raddr;

  generate
    for (genvar init_addr = 0; init_addr < RAM_DEPTH; init_addr++) begin : initialize
      initial begin
        mem[init_addr] = '0;
      end
    end
  endgenerate

  assign port_a_r_data = mem[port_a_raddr];
  assign port_b_r_data = mem[port_b_raddr];

  always_ff @(posedge clk) begin
    port_a_raddr <= port_a_addr;
    port_b_raddr <= port_b_addr;
    if (port_a_w_en) begin
      mem[port_a_addr] <= port_a_w_data;
    end
    if (port_b_w_en) begin
      mem[port_b_addr] <= port_b_w_data;
    end
  end
endmodule
