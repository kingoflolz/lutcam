`default_nettype none

// LL4 reference for LL2 segmentation. Five horizontal samples are centered
// on x, and VERTICAL_ROWS samples end at y. Both dimensions replicate edges.
// The signed mean rounds once after the complete window sum. Serialize the
// incoming pair so the two preceding rows use one 20-bit RAM.
// There is no downstream backpressure: the formatter stores complete rows.
module Ll4CausalMean5x3 #(
  parameter int WIDTH = 80,
  parameter int HEIGHT = 50,
  parameter int DATA_BITS = 10,
  localparam int VERTICAL_ROWS = 3,
  parameter int COORD_WIDTH = 10,
  localparam int X_BITS = $clog2(WIDTH),
  localparam int PAIR_BITS = $clog2(WIDTH / 2),
  localparam int Y_BITS = HEIGHT > 1 ? $clog2(HEIGHT) : 1
) (
  input wire clk,
  input wire rst,
  input wire in_valid,
  input wire in_first,
  input wire in_last,
  input wire [COORD_WIDTH-1:0] in_pair_x,
  input wire [COORD_WIDTH-1:0] in_y,
  input wire signed [1:0][DATA_BITS-1:0] in_value,
  output logic out_valid,
  output logic out_first,
  output logic out_last,
  output logic [COORD_WIDTH-1:0] out_pair_x,
  output logic [COORD_WIDTH-1:0] out_y,
  output logic signed [1:0][DATA_BITS-1:0] out_value,
  output wire busy,
  output logic protocol_error
);
  logic lane_pending;
  logic signed [DATA_BITS-1:0] lane_value;
  logic [PAIR_BITS-1:0] expected_pair;
  logic [Y_BITS-1:0] expected_y;
  wire scalar_valid = lane_pending || in_valid;
  wire [X_BITS-1:0] scalar_x = lane_pending ? lane_x :
      X_BITS'(in_pair_x << 1);
  wire scalar_first_row = lane_pending ? read_first_row : in_y == 0;
  wire signed [DATA_BITS-1:0] scalar_value = lane_pending ? lane_value :
      $signed(in_value[0]);

  // Lane zero is y-1. A new frame seeds every history lane with
  // row zero, masking stale RAM after both normal frame boundaries and aborts.
  (* syn_ramstyle = "block_ram" *)
  logic [(VERTICAL_ROWS-1)*DATA_BITS-1:0] history [0:WIDTH-1];
  logic [(VERTICAL_ROWS-1)*DATA_BITS-1:0] history_q;
  logic read_valid;
  logic [X_BITS-1:0] read_x;
  logic read_first_row;
  wire [X_BITS-1:0] lane_x = read_x | X_BITS'(1);
  logic signed [DATA_BITS-1:0] read_value;
  always_ff @(posedge clk) begin
    if (scalar_valid && !rst) history_q <= history[scalar_x];
    if (read_valid && !rst) begin
      if (read_first_row) history[read_x] <= {(VERTICAL_ROWS-1){read_value}};
      else history[read_x] <= {history_q[(VERTICAL_ROWS-2)*DATA_BITS-1:0], read_value};
    end
  end

  logic signed [11:0] vertical_sum;
  wire signed [10:0] prior01 =
      11'($signed(history_q[0*DATA_BITS +: DATA_BITS])) +
      11'($signed(history_q[1*DATA_BITS +: DATA_BITS]));
  always_comb begin
    vertical_sum = 12'(read_value) +
        (read_first_row ? (12'(read_value) <<< 1) : 12'(prior01));
  end

  logic vertical_valid;
  logic signed [11:0] vertical_q;
  logic [X_BITS-1:0] vertical_x;

  logic signed [11:0] horizontal_delay [0:3];
  logic signed [13:0] horizontal_sum;
  logic signed [11:0] edge_vertical;
  logic [1:0] flush_count;
  logic sum_valid;
  logic signed [13:0] window_sum;
  logic sum_odd;
  wire horizontal_valid = vertical_valid || flush_count != 0;
  wire signed [11:0] horizontal_value = flush_count != 0 ?
      edge_vertical : vertical_q;
  wire first_column = flush_count == 0 && vertical_x == 0;
  wire signed [11:0] leaving_column = flush_count == 0 && int'(vertical_x) < 4 ?
      edge_vertical : horizontal_delay[3];
  wire signed [13:0] next_window_sum = horizontal_sum + 14'(horizontal_value);

  // Retain only the oldest column needed to update the rolling four-column
  // sum. The delay has no reset or replicated-fill mux: edge_vertical supplies
  // the left extension until all four slots belong to the current row.
  always_ff @(posedge clk) if (horizontal_valid) begin
    horizontal_delay[0] <= horizontal_value;
    for (int i=1; i<4; i++) horizontal_delay[i] <= horizontal_delay[i-1];
    if (first_column) horizontal_sum <= 14'(vertical_q) <<< 2;
    else horizontal_sum <= next_window_sum - 14'(leaving_column);
  end

  // The positive offset makes floor division unambiguous for negative sums.
  // For 0 <= n < 15360, floor(n/15) = (n*17477)>>18. The biased
  // addition is signed 15-bit before narrowing; the product is unsigned 28.
  localparam int DIVISOR = 5*VERTICAL_ROWS;
  wire signed [14:0] biased_sum = 15'(window_sum) + 15'(512*DIVISOR + DIVISOR/2);
  logic [27:0] mean_product;
  logic product_valid;
  logic product_odd;
  // Outputs remain in raster order through bubbles. Count completed pairs
  // once, instead of carrying X and Y through every arithmetic stage.
  logic [PAIR_BITS-1:0] result_pair;
  logic [Y_BITS-1:0] result_y;
  wire result_last = result_pair == PAIR_BITS'(WIDTH/2-1);
  wire signed [9:0] mean_value = $signed(mean_product[27:18] ^ 10'd512);
  wire [28:0] unused_division_bits = {biased_sum[14], mean_product};
  logic signed [DATA_BITS-1:0] first_mean;

  always_ff @(posedge clk) begin
    if (rst) begin
      lane_pending <= 1'b0;
      lane_value <= '0;
      expected_pair <= '0;
      expected_y <= '0;
      read_valid <= 1'b0;
      read_x <= '0;
      read_first_row <= 1'b0;
      read_value <= '0;
      vertical_valid <= 1'b0;
      vertical_q <= '0;
      vertical_x <= '0;
      edge_vertical <= '0;
      flush_count <= '0;
      sum_valid <= 1'b0;
      window_sum <= '0;
      sum_odd <= 1'b0;
      mean_product <= '0;
      product_valid <= 1'b0;
      product_odd <= 1'b0;
      result_pair <= '0;
      result_y <= '0;
      first_mean <= '0;
      out_valid <= 1'b0;
      out_first <= 1'b0;
      out_last <= 1'b0;
      out_pair_x <= '0;
      out_y <= '0;
      out_value <= '0;
      protocol_error <= 1'b0;
    end else begin
      lane_pending <= in_valid && !lane_pending;
      if (in_valid) begin
        if (lane_pending || in_pair_x != COORD_WIDTH'(expected_pair) ||
            in_y != COORD_WIDTH'(expected_y) ||
            in_first != (expected_pair == 0) ||
            in_last != (expected_pair == PAIR_BITS'(WIDTH/2-1)))
          protocol_error <= 1'b1;
        if (!lane_pending) begin
          lane_value <= in_value[1];
          if (expected_pair == PAIR_BITS'(WIDTH/2-1)) begin
            expected_pair <= '0;
            expected_y <= expected_y == Y_BITS'(HEIGHT-1) ? '0 :
                expected_y + 1'b1;
          end else expected_pair <= expected_pair + 1'b1;
        end
      end
      read_valid <= scalar_valid;
      if (scalar_valid) begin
        read_x <= scalar_x;
        read_first_row <= scalar_first_row;
        read_value <= scalar_value;
      end
      vertical_valid <= read_valid;
      if (read_valid) begin
        vertical_q <= vertical_sum;
        vertical_x <= read_x;
      end

      sum_valid <= 1'b0;
      if (vertical_valid && flush_count != 0) protocol_error <= 1'b1;
      if (horizontal_valid) begin
        if (first_column) begin
          edge_vertical <= vertical_q;
        end else begin
          if (flush_count != 0 || vertical_x >= 2) begin
            sum_valid <= 1'b1;
            window_sum <= next_window_sum;
            sum_odd <= flush_count != 0 ? flush_count[0] : vertical_x[0];
          end
        end
        if (flush_count != 0) flush_count <= flush_count - 1'b1;
        else if (vertical_x == X_BITS'(WIDTH-1)) begin
          edge_vertical <= vertical_q;
          flush_count <= 2;
        end
      end

      product_valid <= sum_valid;
      if (sum_valid) begin
        mean_product <= $unsigned(biased_sum[13:0]) * 15'd17477;
        product_odd <= sum_odd;
      end
      out_valid <= product_valid && product_odd;
      if (product_valid) begin
        if (!product_odd) first_mean <= mean_value;
        else begin
          out_value <= {mean_value, first_mean};
          out_pair_x <= COORD_WIDTH'(result_pair);
          out_y <= COORD_WIDTH'(result_y);
          out_first <= result_pair == 0;
          out_last <= result_last;
          if (result_last) begin
            result_pair <= '0;
            result_y <= result_y == Y_BITS'(HEIGHT-1) ? '0 : result_y + 1'b1;
          end else result_pair <= result_pair + 1'b1;
        end
      end
    end
  end
  assign busy = lane_pending || read_valid || vertical_valid || flush_count != 0 ||
      sum_valid || product_valid || out_valid;

`ifndef SYNTHESIS
  initial begin
    assert (DATA_BITS == 10 && WIDTH >= 4 && WIDTH % 2 == 0 && HEIGHT >= 1)
      else $error("LL4 mean requires signed10 pixels and an even width >=4");
  end
`endif
endmodule

`default_nettype wire
