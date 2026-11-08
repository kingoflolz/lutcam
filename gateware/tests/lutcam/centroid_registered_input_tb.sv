`timescale 1ns/1ps

// Compare the optional input stage to the original four-stage compressor.
// Every payload bit must match one clock later, including bubbles and zero
// mass; reset must discard both pipelines without replaying an old sample.
module centroid_input_pair #(
  parameter bit POWER2 = 1'b0,
  parameter bit MIDPOINT = 1'b0
)(
  input logic clk, rst, in_valid,
  input logic [28:0] m,
  input logic [38:0] sx,
  input logic [37:0] sy
);
  logic direct_valid, direct_centroid_valid;
  logic registered_valid, registered_centroid_valid;
  logic [14:0] direct_x, registered_x;
  logic [13:0] direct_y, registered_y;
  logic [7:0] direct_log, registered_log;
  logic [38:0] expected;
  integer compared = 0;

  CentroidCompressor #(
    .WIDTH(1696), .HEIGHT(800), .PIXEL_BITS(8), .REGISTER_INPUT(1'b0),
    .POWER2_MULTIPLIER_NORMALIZER(POWER2),
    .MIDPOINT_REFLECTION_RECIPROCAL(MIDPOINT)
  ) direct (
    .clk, .rst, .in_valid, .m, .sx, .sy,
    .out_valid(direct_valid), .centroid_valid(direct_centroid_valid),
    .x_q4(direct_x), .y_q4(direct_y), .m_log_q5_3(direct_log)
  );
  CentroidCompressor #(
    .WIDTH(1696), .HEIGHT(800), .PIXEL_BITS(8), .REGISTER_INPUT(1'b1),
    .POWER2_MULTIPLIER_NORMALIZER(POWER2),
    .MIDPOINT_REFLECTION_RECIPROCAL(MIDPOINT)
  ) registered (
    .clk, .rst, .in_valid, .m, .sx, .sy,
    .out_valid(registered_valid), .centroid_valid(registered_centroid_valid),
    .x_q4(registered_x), .y_q4(registered_y), .m_log_q5_3(registered_log)
  );

  always @(posedge clk) begin
    #1;
    if (rst) begin
      if (registered_valid || registered_centroid_valid)
        $fatal(1, "registered centroid stayed valid during reset");
      expected = '0;
    end else begin
      if ({registered_valid, registered_centroid_valid, registered_x,
           registered_y, registered_log} !== expected)
        $fatal(1, "centroid input stage mismatch power2=%0d midpoint=%0d cycle=%0d expected=%h actual=%h",
               POWER2, MIDPOINT, compared, expected,
               {registered_valid, registered_centroid_valid, registered_x,
                registered_y, registered_log});
      expected = {direct_valid, direct_centroid_valid, direct_x, direct_y, direct_log};
      compared = compared + 1;
    end
  end
endmodule

module centroid_registered_input_tb;
  logic clk = 1'b0;
  logic rst = 1'b1;
  logic in_valid = 1'b0;
  logic [28:0] m = '0;
  logic [38:0] sx = '0;
  logic [37:0] sy = '0;
  logic [63:0] rng = 64'h7b19d624e83a015f;
  integer cycles = 0;

  for (genvar power2 = 0; power2 < 2; power2++) begin : normalizer
    for (genvar midpoint = 0; midpoint < 2; midpoint++) begin : reciprocal
      centroid_input_pair #(.POWER2(1'(power2)), .MIDPOINT(1'(midpoint))) pair (
        .clk, .rst, .in_valid, .m, .sx, .sy
      );
    end
  end

  task automatic tick;
    begin
      #4; clk = 1'b1;
      #4; clk = 1'b0;
      cycles = cycles + 1;
    end
  endtask

  function automatic logic [63:0] random_word;
    begin
      rng = rng ^ (rng << 13);
      rng = rng ^ (rng >> 7);
      rng = rng ^ (rng << 17);
      return rng;
    end
  endfunction

  initial begin
    repeat (3) tick();
    rst = 1'b0;
    // Every mass exponent and both sides of each power of two; signed
    // endpoint moments and alternating input validity include every field.
    for (integer exponent = 0; exponent < 29; exponent++) begin
      for (integer delta = -1; delta <= 1; delta++) begin
        m = (29'(1) << exponent) + 29'(delta);
        for (integer pattern = 0; pattern < 6; pattern++) begin
          case (pattern)
            0: begin sx = '0; sy = '0; end
            1: begin sx = 39'd1; sy = 38'd1; end
            2: begin sx = '1; sy = '1; end
            3: begin sx = 39'h3fffffffff; sy = 38'h1fffffffff; end
            4: begin sx = 39'h4000000000; sy = 38'h2000000000; end
            default: begin sx = 39'(random_word()); sy = 38'(random_word()); end
          endcase
          in_valid = 1'b1;
          tick();
          in_valid = 1'b0;
          tick();
        end
      end
    end
    // Consecutive samples, arbitrary bubbles, zero masses, and resets with
    // each possible stage occupied. Payload keeps changing during bubbles.
    for (integer sample = 0; sample < 20000; sample++) begin
      m = (sample % 31 == 0) ? '0 : 29'(random_word());
      sx = 39'(random_word());
      sy = 38'(random_word());
      in_valid = (sample % 17 < 11) || 1'(random_word());
      rst = sample % 113 == 0 || sample % 619 < 3;
      tick();
    end
    rst = 1'b0;
    in_valid = 1'b0;
    repeat (8) tick();
    $display("PASS centroid input register: four arithmetic configurations, %0d cycles each", cycles);
    $finish;
  end
endmodule
