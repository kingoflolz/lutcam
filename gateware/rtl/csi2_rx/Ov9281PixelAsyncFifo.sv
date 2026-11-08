// Dual-clock pixel FIFO for the fixed OV9281 receive path.

module Ov9281PixelAsyncFifo #(
  parameter int DATA_WIDTH = 20,
  parameter int DEPTH = 512
)(
  input  logic                  wr_clk,
  input  logic                  wr_reset_n,
  input  logic                  wr_en,
  input  logic [DATA_WIDTH-1:0] wr_data,
  output logic                  wr_full,
  output logic                  wr_overflow,

  input  logic                  rd_clk,
  input  logic                  rd_reset_n,
  input  logic                  rd_ready,
  output logic                  rd_valid,
  output logic [DATA_WIDTH-1:0] rd_data,
  output logic                  rd_empty
);
  localparam int ADDR_WIDTH = $clog2(DEPTH);
  localparam int POINTER_WIDTH = ADDR_WIDTH + 1;

  // The default 512-entry geometry and the smaller production Ti60 profile
  // each fit one Titanium RAM10. The memory has no reset; only pointers and
  // the read output register reset.
  (* ram_style = "block", syn_ramstyle = "block_ram" *)
  logic [DATA_WIDTH-1:0] memory [0:DEPTH-1];

  logic [POINTER_WIDTH-1:0] wr_binary;
  logic [POINTER_WIDTH-1:0] wr_binary_next;
  logic [POINTER_WIDTH-1:0] wr_gray;
  logic [POINTER_WIDTH-1:0] wr_gray_next;
  logic [POINTER_WIDTH-1:0] rd_binary;
  logic [POINTER_WIDTH-1:0] rd_binary_next;
  logic [POINTER_WIDTH-1:0] rd_gray;
  logic [POINTER_WIDTH-1:0] rd_gray_next;

  (* async_reg = "true", syn_preserve = 1 *)
  logic [POINTER_WIDTH-1:0] rd_gray_wr_sync1;
  (* async_reg = "true", syn_preserve = 1 *)
  logic [POINTER_WIDTH-1:0] rd_gray_wr_sync2;
  (* async_reg = "true", syn_preserve = 1 *)
  logic [POINTER_WIDTH-1:0] wr_gray_rd_sync1;
  (* async_reg = "true", syn_preserve = 1 *)
  logic [POINTER_WIDTH-1:0] wr_gray_rd_sync2;

  logic [POINTER_WIDTH-1:0] wr_full_compare;
  logic                     wr_push;
  logic                     wr_full_next;
  logic                     rd_has_next;
  logic                     rd_pop;
  logic                     ram_read_enable;
  logic [ADDR_WIDTH-1:0]    ram_read_address;

  function automatic logic [POINTER_WIDTH-1:0] to_gray(
    input logic [POINTER_WIDTH-1:0] binary
  );
    begin
      to_gray = (binary >> 1) ^ binary;
    end
  endfunction

  assign wr_push = wr_en && !wr_full;
  assign wr_binary_next =
    wr_binary + POINTER_WIDTH'(wr_push);
  assign wr_gray_next = to_gray(wr_binary_next);

  always_comb begin
    wr_full_compare = rd_gray_wr_sync2;
    wr_full_compare[POINTER_WIDTH-1 -: 2] =
      ~rd_gray_wr_sync2[POINTER_WIDTH-1 -: 2];
  end
  assign wr_full_next = wr_gray_next == wr_full_compare;

  assign rd_binary_next = rd_binary + POINTER_WIDTH'(1);
  assign rd_gray_next = to_gray(rd_binary_next);
  assign rd_empty = rd_gray == wr_gray_rd_sync2;
  assign rd_has_next = rd_gray_next != wr_gray_rd_sync2;
  assign rd_pop = rd_valid && rd_ready;

  // rd_binary names the word currently exposed (when rd_valid is set), or
  // the next word to fetch (when it is clear). On a consumed beat the next
  // address is issued to the synchronous RAM in the same cycle, sustaining
  // one transfer per rd_clk without adding a hidden FIFO entry.
  assign ram_read_enable = (!rd_valid && !rd_empty) ||
    (rd_pop && rd_has_next);
  assign ram_read_address = rd_pop ?
    rd_binary_next[ADDR_WIDTH-1:0] : rd_binary[ADDR_WIDTH-1:0];

  // The resets must be derived from the same global asynchronous assertion.
  // Their deassertion is expected to have been synchronized by the caller to
  // the corresponding local clock. Resetting only one side is unsupported.
  always_ff @(posedge wr_clk or negedge wr_reset_n) begin
    if (!wr_reset_n) begin
      rd_gray_wr_sync1 <= '0;
      rd_gray_wr_sync2 <= '0;
    end else begin
      rd_gray_wr_sync1 <= rd_gray;
      rd_gray_wr_sync2 <= rd_gray_wr_sync1;
    end
  end

  always_ff @(posedge rd_clk or negedge rd_reset_n) begin
    if (!rd_reset_n) begin
      wr_gray_rd_sync1 <= '0;
      wr_gray_rd_sync2 <= '0;
    end else begin
      wr_gray_rd_sync1 <= wr_gray;
      wr_gray_rd_sync2 <= wr_gray_rd_sync1;
    end
  end

  always_ff @(posedge wr_clk or negedge wr_reset_n) begin
    if (!wr_reset_n) begin
      wr_binary <= '0;
      wr_gray <= '0;
      wr_full <= 1'b0;
      wr_overflow <= 1'b0;
    end else begin
      wr_overflow <= wr_en && wr_full;
      wr_binary <= wr_binary_next;
      wr_gray <= wr_gray_next;
      wr_full <= wr_full_next;
    end
  end

  // Keep each RAM port in a dedicated, reset-free process. This is the
  // canonical Titanium simple-dual-port template and maps every supported
  // power-of-two geometry through the same dual-clock memory port shape.
  always_ff @(posedge wr_clk) begin
    if (wr_push)
      memory[wr_binary[ADDR_WIDTH-1:0]] <= wr_data;
  end

  always_ff @(posedge rd_clk) begin
    if (ram_read_enable)
      rd_data <= memory[ram_read_address];
  end

  always_ff @(posedge rd_clk or negedge rd_reset_n) begin
    if (!rd_reset_n) begin
      rd_binary <= '0;
      rd_gray <= '0;
      rd_valid <= 1'b0;
    end else begin
      if (rd_pop) begin
        rd_binary <= rd_binary_next;
        rd_gray <= rd_gray_next;
      end

      if (ram_read_enable)
        rd_valid <= 1'b1;
      else if (rd_pop)
        rd_valid <= 1'b0;
    end
  end

`ifndef SYNTHESIS
  initial begin
    assert (DATA_WIDTH > 0);
    assert (DEPTH >= 4);
    assert ((DEPTH & (DEPTH - 1)) == 0);
  end

  always_ff @(posedge wr_clk or negedge wr_reset_n) begin
    if (!wr_reset_n) begin
      // Assertion state follows the same asynchronous reset as the design.
    end else begin
      assert (!(wr_push && wr_full));
    end
  end

  always_ff @(posedge rd_clk or negedge rd_reset_n) begin
    if (!rd_reset_n) begin
      // Assertion state follows the same asynchronous reset as the design.
    end else begin
      assert (!(rd_valid && rd_empty));
    end
  end
`endif
endmodule
