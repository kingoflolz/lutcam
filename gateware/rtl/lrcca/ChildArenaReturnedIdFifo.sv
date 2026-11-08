// Synchronous returned-id FIFO helper for ChildArena.

module ChildArenaReturnedIdFifo #(
  parameter int WIDTH = 1,
  parameter int PRELOAD_COUNT = 0,
  localparam int ADDR_WIDTH = 10,
  localparam int DEPTH = 1 << ADDR_WIDTH
)(
  input  logic clk,
  input  logic rst,
  input  logic [WIDTH-1:0] w_data,
  input  logic w_en,
  output logic w_rdy,
  output logic [WIDTH-1:0] r_data,
  input  logic r_en,
  output logic r_rdy
);
  logic [WIDTH-1:0] storage [0:DEPTH-1];
  logic [WIDTH-1:0] memory_read, forwarded_data;
  logic forwarded_valid;
  logic [ADDR_WIDTH-1:0] fetch_address;
  logic [ADDR_WIDTH-1:0] write_ptr;
  logic [ADDR_WIDTH-1:0] read_ptr;
  logic empty;
  logic push;
  logic pop;
  logic preload_active;
  logic [ADDR_WIDTH-1:0] preload_id;
  logic ram_pop;

  // Generate the initial free IDs before consuming returned IDs. Rewinding
  // pointers into one-time initialized RAM on reset is unsafe after that RAM
  // has been mutated by allocations/returns: its old window can contain
  // duplicate or missing IDs. This resettable prefix preserves the original
  // allocation order without clearing RAM or waiting for an initialization
  // walk. The RAM queue starts empty, so stale entries are never observed.

  assign w_rdy = 1'b1;
  assign r_rdy = preload_active || !empty;
  assign push = w_en;
  assign pop = r_en & r_rdy;
  assign ram_pop = pop && !preload_active;
  assign fetch_address = ram_pop ? read_ptr + ADDR_WIDTH'(1) : read_ptr;
  assign r_data = preload_active ? WIDTH'(preload_id) :
      (forwarded_valid ? forwarded_data : memory_read);
  always_ff @(posedge clk) begin
    memory_read <= storage[fetch_address];
    if (rst) forwarded_valid <= 1'b0;
    else forwarded_valid <= push && (write_ptr == fetch_address);
  end

  // Collision forwarding is storage, so retain the payload in fabric registers.
  always_ff @(posedge clk)
    if (!rst && push) forwarded_data <= w_data;
  always_ff @(posedge clk) begin
    if (rst) begin
      write_ptr <= '0;
      read_ptr <= '0;
      empty <= 1'b1;
      preload_id <= '0;
      preload_active <= PRELOAD_COUNT != 0;
    end else begin
      if (push) begin
        storage[write_ptr] <= w_data;
        write_ptr <= write_ptr + ADDR_WIDTH'(1);
      end
      if (pop && preload_active) begin
        if (preload_id == ADDR_WIDTH'(PRELOAD_COUNT - 1))
          preload_active <= 1'b0;
        else
          preload_id <= preload_id + ADDR_WIDTH'(1);
      end
      if (ram_pop) begin
        read_ptr <= read_ptr + ADDR_WIDTH'(1);
      end
      unique case ({push, ram_pop})
        2'b10: empty <= 1'b0;
        2'b01: empty <= ((read_ptr + ADDR_WIDTH'(1)) == write_ptr);
        2'b11: empty <= 1'b0;
        default: empty <= empty;
      endcase
    end
  end
endmodule
