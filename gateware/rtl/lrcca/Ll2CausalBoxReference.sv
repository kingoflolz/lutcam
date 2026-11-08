`default_nettype none

// Read a 13-column x 3-row causal box from the formatter's EXISTING LL2 RAM.
// No image storage is added. Packed vertical sums form a small register cache;
// each word needs three RAM reads and serves four RAW output pairs. The
// single RAM port is returned to LL2 display after the final RAW pair.
// Edges replicate the first/last scalar, not a packed pair. Output is the
// unrounded window sum, allowing an exact comparison without division.
module Ll2CausalBoxReference #(
  localparam int RADIUS = 6,
  localparam int CACHE_WORDS = RADIUS + 2,
  localparam int SLOT_BITS = $clog2(CACHE_WORDS)
) (
  input wire clk,
  input wire rst,
  input wire start,
  input wire [3:0] current_bank,
  input wire [7:0] current_y,
  input wire take,
  input wire [9:0] pair_x,
  output logic ready,
  output logic signed [15:0] window_sum,
  output wire signed [9:0] direct_value,
  output wire signed [11:0] local_three_sum,
  output wire read_en,
  output logic [3:0] read_bank,
  output wire [7:0] read_pair_x,
  input wire [19:0] read_data
);
  logic active;
  logic [3:0] row0, row1, row2;
  logic priming;
  logic [SLOT_BITS-1:0] prime_slot;
  logic [1:0] fetch_row;
  logic [7:0] fetch_word;
  logic response_valid, response_prime;
  logic [1:0] response_row;
  logic [SLOT_BITS-1:0] response_slot;
  logic response_left, response_right;
  logic signed [11:0] accumulator0, accumulator1;
  logic [23:0] cache [0:CACHE_WORDS-1];
  // Retain current/future columns for the local blend, and one preceding
  // scalar for the horizontal three-tap cap. These are register taps on
  // reads already needed by the wide box; no extra RAM access is needed.
  logic [19:0] direct_cache [0:RADIUS/2+1];
  logic signed [9:0] previous_direct;
  assign direct_value = pair_x[1] ? $signed(direct_cache[0][19:10]) :
      $signed(direct_cache[0][9:0]);
  // Both scalar phases contain the complete current pair. Add it once and
  // select only the third sample, instead of selecting three separate terms.
  wire signed [10:0] local_pair = 11'($signed(direct_cache[0][9:0])) +
      11'($signed(direct_cache[0][19:10]));
  wire signed [9:0] local_edge = !pair_x[1] ? previous_direct : pair_x >= 10'd638 ?
      $signed(direct_cache[0][19:10]) : $signed(direct_cache[1][9:0]);
  assign local_three_sum = 12'(local_pair) + 12'(local_edge);
  wire rotate_words = take && ready && pair_x[1:0] == 2'b11;
  wire last_pair = pair_x == 10'd639;
  logic [1:0] request_row;
  logic [SLOT_BITS-1:0] request_slot;
  logic [7:0] request_word;
  logic request_left, request_right;

  function automatic logic [3:0] previous_bank(input logic [3:0] bank);
    previous_bank = bank == 0 ? 4'd8 : bank - 1'b1;
  endfunction

  always_comb begin
    request_row = fetch_row;
    request_slot = SLOT_BITS'(CACHE_WORDS-1);
    request_word = fetch_word;
    request_left = 1'b0;
    request_right = fetch_word >= 8'd160;
    if (priming) begin
      request_slot = prime_slot;
      request_left = prime_slot < SLOT_BITS'(RADIUS/2);
      request_word = request_left ? 8'd0 : 8'(prime_slot) - 8'(RADIUS/2);
      request_right = 1'b0;
    end else if (rotate_words && !last_pair) begin
      request_row = 0;
      request_word = pair_x[9:2] + 8'(RADIUS/2+2);
      request_right = request_word >= 8'd160;
    end
    case (request_row)
      0: read_bank = row0;
      1: read_bank = row1;
      default: read_bank = row2;
    endcase
  end
  assign read_en = active && (priming || fetch_row != 0 || (rotate_words && !last_pair));
  assign read_pair_x = request_right ? 8'd159 : request_word;
  wire signed [11:0] scalar0 = 12'($signed(read_data[9:0]));
  wire signed [11:0] scalar1 = 12'($signed(read_data[19:10]));
  wire signed [11:0] vertical0 = accumulator0 + scalar0;
  wire signed [11:0] vertical1 = accumulator1 + scalar1;
  wire [23:0] vertical_word = response_left ? {2{vertical0}} :
      response_right ? {2{vertical1}} : {vertical1, vertical0};
  // Retain the sum of the twelve middle columns instead of rebuilding an
  // adder tree for every RAW pair. A word rotation replaces two columns;
  // the thirteenth column is selected by the scalar phase at the output.
  logic signed [15:0] middle;
  wire signed [12:0] entering =
      13'($signed(cache[RADIUS][23:12])) + 13'($signed(cache[RADIUS+1][11:0]));
  wire signed [12:0] leaving =
      13'($signed(cache[0][23:12])) + 13'($signed(cache[1][11:0]));
  wire signed [13:0] rotation_delta = 14'(entering) - 14'(leaving);
  wire signed [12:0] prime_delta =
      (response_slot == 0 ? 13'sd0 : 13'($signed(vertical_word[11:0]))) +
      (response_slot == SLOT_BITS'(RADIUS) ? 13'sd0 : 13'($signed(vertical_word[23:12])));
  wire add_prime = response_valid && response_row == 2 && response_prime &&
      response_slot <= SLOT_BITS'(RADIUS);
  wire signed [15:0] middle_delta = add_prime ? 16'(prime_delta) : 16'(rotation_delta);
  always_comb begin
    window_sum = middle + (pair_x[1] ? 16'($signed(cache[RADIUS][23:12])) :
        16'($signed(cache[0][11:0])));
  end

  // Priming and normal advance both use the same shift path. Every cache
  // entry is overwritten before ready, so resetting its payload is redundant.
  always_ff @(posedge clk) begin
    if (response_valid && response_row == 2) cache[CACHE_WORDS-1] <= vertical_word;
    if (rotate_words || (response_valid && response_row == 2 && response_prime))
      for (int i=0; i<CACHE_WORDS-1; i++) cache[i] <= cache[i+1];
    if (response_valid && response_row == 0 && response_slot >= SLOT_BITS'(RADIUS/2))
      direct_cache[RADIUS/2+1] <= read_data;
    if (rotate_words || (response_valid && response_row == 0 && response_prime &&
        response_slot >= SLOT_BITS'(RADIUS/2)))
      for (int i=0; i<RADIUS/2+1; i++) direct_cache[i] <= direct_cache[i+1];
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      active <= 0; ready <= 0;
      row0 <= 0; row1 <= 0; row2 <= 0;
      priming <= 0; prime_slot <= 0; fetch_row <= 0; fetch_word <= 0;
      response_valid <= 0; response_prime <= 0; response_row <= 0; response_slot <= 0;
      response_left <= 0; response_right <= 0;
      accumulator0 <= 0; accumulator1 <= 0;
      previous_direct <= 0;
      middle <= 0;
    end else if (start) begin
      active <= 1; ready <= 0; middle <= 0;
      row0 <= current_bank;
      row1 <= current_y == 0 ? current_bank : previous_bank(current_bank);
      row2 <= current_y == 0 ? current_bank : current_y == 1 ?
          previous_bank(current_bank) : previous_bank(previous_bank(current_bank));
      priming <= 1; prime_slot <= 0; fetch_row <= 0; response_valid <= 0;
    end else begin
      response_valid <= read_en;
      if (read_en) begin
        response_row <= request_row; response_slot <= request_slot; response_prime <= priming;
        response_left <= request_left; response_right <= request_right;
        fetch_row <= request_row == 2 ? 2'd0 : request_row + 1'b1;
        if (priming && request_row == 2) begin
          if (prime_slot == SLOT_BITS'(CACHE_WORDS-1)) priming <= 0;
          else prime_slot <= prime_slot + 1'b1;
        end
        if (!priming) fetch_word <= request_word;
      end
      if (add_prime || rotate_words) middle <= middle + middle_delta;
      if (response_valid) begin
        if (response_row == 0 && response_slot == SLOT_BITS'(RADIUS/2))
          previous_direct <= $signed(read_data[9:0]);
        if (response_row == 0) begin accumulator0 <= scalar0; accumulator1 <= scalar1; end
        else if (response_row == 1) begin accumulator0 <= vertical0; accumulator1 <= vertical1; end
        else begin
          if (response_slot == SLOT_BITS'(CACHE_WORDS-1)) ready <= 1;
        end
      end
      if (rotate_words) begin
        previous_direct <= $signed(direct_cache[0][19:10]);
        if (last_pair) begin active <= 0; ready <= 0; end
      end
    end
  end
`ifndef SYNTHESIS
  always_ff @(posedge clk) if (!rst) begin
    assert (!(start && active)) else $error("RAW reference started before prior row drained");
    assert (!(rotate_words && fetch_row != 0)) else $error("RAW reference prefetch missed deadline");
  end
`endif
endmodule
`default_nettype wire
