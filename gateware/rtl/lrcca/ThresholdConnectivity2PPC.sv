`default_nettype none

// Resolve ambiguous diagonal connections from the sign and three-bit
// magnitude of pixel-threshold, plus one thin-edge continuation bit. One
// packed row reuses each address after its previous-row value is read.
// Added topology pixels carry no
// centroid mass. All payload and metadata obey the ready/valid handshake.
module ThresholdConnectivity2PPC #(
  parameter int RAW_PAIRS = 640,
  parameter int SOURCE_Y_BITS = 10,
  localparam int X_BITS = $clog2(RAW_PAIRS)
) (
  input wire clk,
  input wire rst,
  input wire in_valid,
  output wire in_ready,
  input wire [7:0] in_d0, in_d1,
  input wire in_t0, in_t1,
  input wire in_fv0, in_fv1,
  input wire in_eol,
  input wire in_row_replicated,
  input wire [SOURCE_Y_BITS-1:0] in_source_y,
  input wire [3:0] in_strength0, in_strength1,
  input wire in_raw,
  input wire in_allow_repair,
  input wire in_first_row,
  input wire [X_BITS-1:0] in_pair_x,
  output logic out_valid,
  input wire out_ready,
  output logic [7:0] out_d0, out_d1,
  output logic out_t0, out_t1,
  output logic out_fv0, out_fv1,
  output logic out_eol,
  output logic out_row_replicated,
  output logic [SOURCE_Y_BITS-1:0] out_source_y,
  output wire busy
);
  typedef struct packed {
    logic [7:0] d0, d1;
    logic t0, t1, fv0, fv1, eol, replicated;
    logic [SOURCE_Y_BITS-1:0] source_y;
    logic [3:0] strength0, strength1;
    logic raw, allow_repair, first_row;
    logic [X_BITS-1:0] pair_x;
  } packet_t;
  packet_t read_packet, pending;
  wire unused_pending_first_row = pending.first_row;
  logic read_valid, pending_valid;
  logic [9:0] row_q, pending_top;
  logic [4:0] previous_strengths;
  logic [1:0] pending_left_signs;
  logic [3:0] previous_top;
  logic pending_top_left_white, pending_left_wins;
  (* syn_ramstyle = "block_ram" *) logic [9:0] row_memory [0:RAW_PAIRS-1];
  wire output_ready = !out_valid || out_ready;
  wire emit_pending = pending_valid && (read_valid || pending.eol) && output_ready;
  wire pending_ready = !pending_valid || emit_pending;
  wire read_transfer = read_valid && pending_ready;
  assign in_ready = !read_valid || read_transfer;
  wire accept = in_valid && in_ready;
  wire [9:0] read_top = read_packet.raw && !read_packet.first_row ? row_q : 10'h088;
  wire right_white = !read_valid || read_packet.strength0[3];
  wire top_right_white = !read_valid || read_top[3];
  assign busy = read_valid || pending_valid || out_valid;

  // Explicit small partial products keep these 3x3 products in logic.
  function automatic logic [5:0] product(input logic [2:0] a, b);
    product = (b[0] ? {3'b0, a} : 6'b0) +
        (b[1] ? {2'b0, a, 1'b0} : 6'b0) +
        (b[2] ? {1'b0, a, 2'b0} : 6'b0);
  endfunction

  wire has_left_pair = pending.pair_x != 0;
  wire has_right_pair = pending.pair_x != X_BITS'(RAW_PAIRS-1);
  // The two lanes inspect the same central 2x2 cell from opposite sides.
  // Share its two products and subtraction; the sign and zero flags express
  // both strict comparisons, including equal-strength ties.
  wire [5:0] central_product0 = product(pending_top[2:0], pending.strength1[2:0]);
  wire [5:0] central_product1 = product(pending_top[6:4], pending.strength0[2:0]);
  wire [6:0] central_difference = {1'b0, central_product0} - {1'b0, central_product1};
  wire central_wins0 = !central_difference[6] && |central_difference[5:0];
  wire central_wins1 = central_difference[6];
  // The boundary cell is the right neighbor of the pending pair and the left
  // neighbor of the incoming pair. Compute it once, then retain its opposite
  // comparison when that incoming pair advances. Previous taps survive gaps.
  wire [5:0] boundary_product0 = product(previous_top[2:0], read_packet.strength0[2:0]);
  wire [5:0] boundary_product1 = product(previous_strengths[3:1], read_top[2:0]);
  wire [6:0] boundary_difference = {1'b0, boundary_product0} - {1'b0, boundary_product1};
  wire right_wins = read_valid && !boundary_difference[6] && |boundary_difference[5:0];
  wire repair0 = pending.raw && pending.allow_repair && pending.fv0 &&
      pending.strength0[3] && !pending_top[3] &&
      ((!pending_left_signs[1] && pending_top_left_white &&
        (pending_left_wins || (pending_top[8] && has_left_pair && pending_left_signs[0]))) ||
       (!pending.strength1[3] && pending_top[7] &&
        (central_wins0 || (pending_top[8] && has_right_pair && right_white))));
  wire repair1 = pending.raw && pending.allow_repair && pending.fv1 &&
      pending.strength1[3] && !pending_top[7] &&
      ((!pending.strength0[3] && pending_top[3] &&
        (central_wins1 || (pending_top[9] && has_left_pair && pending_left_signs[1]))) ||
       (!right_white && top_right_white &&
        (right_wins || (pending_top[9] && has_right_pair && read_packet.strength1[3]))));
  // This bit is consumed one row later. It records an original black pixel
  // with white horizontal neighbors and a black diagonal above it. Repaired
  // pixels never feed back into the residual history or continuation flags.
  wire continues0 = !pending.strength0[3] && pending_left_signs[1] &&
      pending.strength1[3] && (!pending_top_left_white || !pending_top[7]);
  wire continues1 = !pending.strength1[3] && pending.strength0[3] &&
      right_white && (!pending_top[3] || !top_right_white);

  always_ff @(posedge clk) begin
    if (accept && in_raw && !rst) row_q <= row_memory[in_pair_x];
    if (emit_pending && pending.raw && !rst)
      row_memory[pending.pair_x] <= {continues1, continues0, pending.strength1, pending.strength0};
  end
  always_ff @(posedge clk) begin
    if (rst) begin
      read_valid <= 0;
      pending_valid <= 0;
      read_packet <= '0;
      pending <= '0;
      pending_top <= 10'h088;
      previous_strengths <= 5'b10001;
      previous_top <= 4'h8;
      pending_left_signs <= 2'b11;
      pending_top_left_white <= 1'b1;
      pending_left_wins <= 1'b0;
      out_valid <= 0;
      out_d0 <= 0; out_d1 <= 0;
      out_t0 <= 0; out_t1 <= 0;
      out_fv0 <= 0; out_fv1 <= 0;
      out_eol <= 0; out_row_replicated <= 0; out_source_y <= 0;
    end else begin
      if (in_ready) begin
        read_valid <= in_valid;
        if (in_valid) begin
          read_packet <= '{d0:in_d0, d1:in_d1, t0:in_t0, t1:in_t1,
              fv0:in_fv0, fv1:in_fv1, eol:in_eol, replicated:in_row_replicated,
              source_y:in_source_y, strength0:in_strength0, strength1:in_strength1,
              raw:in_raw, allow_repair:in_allow_repair, first_row:in_first_row, pair_x:in_pair_x};
        end
      end
      if (pending_ready) begin
        pending_valid <= read_valid;
        if (read_valid) begin
          pending <= read_packet;
          pending_top <= read_top;
          pending_left_signs <= {previous_strengths[4], previous_strengths[0]};
          pending_top_left_white <= previous_top[3];
          pending_left_wins <= boundary_difference[6];
          previous_strengths <= {read_packet.strength1, read_packet.strength0[3]};
          previous_top <= read_top[7:4];
        end
      end
      if (output_ready) begin
        out_valid <= emit_pending;
        out_eol <= emit_pending && pending.eol;
        if (emit_pending) begin
          out_d0 <= pending.d0; out_d1 <= pending.d1;
          out_t0 <= pending.t0 && !repair0; out_t1 <= pending.t1 && !repair1;
          out_fv0 <= pending.fv0 && !repair0; out_fv1 <= pending.fv1 && !repair1;
          out_row_replicated <= pending.replicated;
          out_source_y <= (pending.fv0 && !repair0) || (pending.fv1 && !repair1) ?
              pending.source_y : '0;
        end
      end
    end
  end
`ifndef SYNTHESIS
  always_ff @(posedge clk) if (!rst) begin
    assert (!(accept && in_raw && int'(in_pair_x) >= RAW_PAIRS))
      else $error("Connectivity row address exceeds RAW width");
    assert (!(accept && in_raw && emit_pending && pending.raw &&
        in_pair_x == pending.pair_x))
      else $error("Connectivity row memory has a same-address read/write collision");
  end
`endif
endmodule

`default_nettype wire
