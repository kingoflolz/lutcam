// Synchronous FIFO helper for ChildArena command and returned-id queues.

import lrcca_util_pkg::bit_width;

module ChildArenaSyncFifo #(
  parameter int WIDTH = 1,
  parameter int DEPTH = 2,
  localparam int ADDR_WIDTH = bit_width(longint'(DEPTH))
)(
  input  logic clk,
  input  logic rst,
  input  logic [WIDTH-1:0] w_data,
  input  logic w_en,
  output logic w_rdy,
  output logic [WIDTH-1:0] r_data,
  input  logic r_en,
  output logic r_rdy,
  output logic empty
);
  localparam logic [ADDR_WIDTH-1:0] LAST_ADDR = ADDR_WIDTH'(DEPTH - 1);

  logic [WIDTH-1:0] storage [0:DEPTH-1];
  logic [ADDR_WIDTH-1:0] write_ptr;
  logic [ADDR_WIDTH-1:0] read_ptr;
  logic [ADDR_WIDTH-1:0] write_ptr_next;
  logic [ADDR_WIDTH-1:0] read_ptr_next;
  logic full;
  logic push;
  logic pop;

  function automatic logic [ADDR_WIDTH-1:0] next_addr(input logic [ADDR_WIDTH-1:0] value);
    begin
      next_addr = (value == LAST_ADDR) ? '0 : (value + ADDR_WIDTH'(1));
    end
  endfunction

  assign write_ptr_next = next_addr(write_ptr);
  assign read_ptr_next = next_addr(read_ptr);
  assign w_rdy = !full;
  assign r_rdy = !empty;
  assign push = w_en & w_rdy;
  assign pop = r_en & r_rdy;
  assign r_data = r_rdy ? storage[read_ptr] : '0;

  always_ff @(posedge clk) begin
    if (rst) begin
      write_ptr <= '0;
      read_ptr <= '0;
      full <= 1'b0;
      empty <= 1'b1;
    end else begin
      if (push) begin
        storage[write_ptr] <= w_data;
        write_ptr <= write_ptr_next;
      end
      if (pop) begin
        read_ptr <= read_ptr_next;
      end
      unique case ({push, pop})
        2'b10: begin
          empty <= 1'b0;
          full <= write_ptr_next == read_ptr;
        end
        2'b01: begin
          empty <= read_ptr_next == write_ptr;
          full <= 1'b0;
        end
        2'b11: begin
          empty <= 1'b0;
          full <= 1'b0;
        end
        default: begin
          empty <= empty;
          full <= full;
        end
      endcase
    end
  end
endmodule
