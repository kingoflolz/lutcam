// Shared, vendor-neutral centroid normalization logic.

import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::max_int;

module CentroidCompressor #(
  parameter int WIDTH = 1280,
  parameter int HEIGHT = 800,
  parameter int PIXEL_BITS = 8,
  // Isolate upstream moment finalization from normalization. The optional
  // stage adds one clock to every output without changing its arithmetic.
  parameter bit REGISTER_INPUT = 1'b0,
  parameter bit POWER2_MULTIPLIER_NORMALIZER = 1'b0,
  parameter bit MIDPOINT_REFLECTION_RECIPROCAL = 1'b0,
  parameter string RECIPROCAL_SEGMENT_BIN =
      "rtl/lrcca/centroid_reciprocal_segments_q17.bin",
  parameter string RECIPROCAL_MIDPOINT_REFLECTION_BIN =
      "rtl/lrcca/centroid_reciprocal_midpoint_reflection_q17.bin"
)(m, sx, sy, clk, rst, out_valid, centroid_valid, x_q4, y_q4, m_log_q5_3, in_valid);
  localparam int INTERNAL_FRAC_BITS = 21;
  localparam int OUTPUT_FRAC_BITS = 4;
  localparam int LOG_M_Q53_BITS = 8;
  localparam int LOG_M_Q53_FRAC_BITS = 3;
  localparam int LOG_M_Q53_WIDE_BITS = LOG_M_Q53_BITS + 1;
  localparam int RECIPROCAL_SEGMENT_BITS = 8;
  localparam int RECIPROCAL_INTERPOLATION_BITS =
      INTERNAL_FRAC_BITS - RECIPROCAL_SEGMENT_BITS;
  localparam int RECIPROCAL_BITS = 17;
  localparam int RECIPROCAL_DROP_BITS = 9;
  localparam int RECIPROCAL_STORED_BASE_BITS = 14;
  localparam int RECIPROCAL_STORED_DROP_BITS = 6;
  localparam int RECIPROCAL_WORD_BITS =
      RECIPROCAL_STORED_BASE_BITS + RECIPROCAL_STORED_DROP_BITS;
  localparam int RECIPROCAL_MIDPOINT_SEGMENT_BITS = 9;
  localparam int RECIPROCAL_MIDPOINT_DISCARD_BITS = 3;
  localparam int RECIPROCAL_MIDPOINT_COEFFICIENT_BITS = 18;
  localparam int RECIPROCAL_MIDPOINT_PRODUCT_SHIFT = 19;
  localparam int RECIPROCAL_TABLE_SEGMENT_BITS =
      MIDPOINT_REFLECTION_RECIPROCAL ?
      RECIPROCAL_MIDPOINT_SEGMENT_BITS : RECIPROCAL_SEGMENT_BITS;
  localparam string RECIPROCAL_TABLE_BIN =
      MIDPOINT_REFLECTION_RECIPROCAL ?
      RECIPROCAL_MIDPOINT_REFLECTION_BIN : RECIPROCAL_SEGMENT_BIN;
  localparam int RECIPROCAL_PRODUCT_BITS =
      RECIPROCAL_DROP_BITS + RECIPROCAL_INTERPOLATION_BITS;
  localparam int RECIPROCAL_BASE_SPLIT_BITS = 4;
  localparam int RECIPROCAL_BASE_HIGH_BITS =
      RECIPROCAL_BITS - RECIPROCAL_BASE_SPLIT_BITS;
  localparam int RECIPROCAL_BASE_HIGH_SUM_BITS = RECIPROCAL_BASE_HIGH_BITS + 1;
  localparam int RECIPROCAL_SPLIT_C_BITS = 18;
  localparam int RECIPROCAL_SPLIT_TERM_BITS = RECIPROCAL_DROP_BITS + 1;
  localparam int NORMALIZED_MOMENT_FRAC_BITS = 4;
  localparam int NORMALIZED_PRODUCT_SHIFT =
      NORMALIZED_MOMENT_FRAC_BITS + RECIPROCAL_BITS + 1 - OUTPUT_FRAC_BITS;

  localparam longint unsigned WIDTH_U = longint'(WIDTH);
  localparam longint unsigned HEIGHT_U = longint'(HEIGHT);
  localparam longint unsigned MAX_PIXEL_U = (64'd1 << PIXEL_BITS) - 1;
  localparam int M_BITS = bit_width((WIDTH_U * HEIGHT_U * MAX_PIXEL_U) + 1);
  localparam int CENTER_X_BITS = centered_moment_bits(WIDTH, HEIGHT, PIXEL_BITS);
  localparam int CENTER_Y_BITS = centered_moment_bits(HEIGHT, WIDTH, PIXEL_BITS);
  localparam int SX_BITS = CENTER_X_BITS;
  localparam int SY_BITS = CENTER_Y_BITS;
  localparam int NORM_BITS = max_int(M_BITS, max_int(CENTER_X_BITS, CENTER_Y_BITS));
  localparam int NORM_EXP_BITS = bit_width(longint'(NORM_BITS));
  localparam int X_OUTPUT_BITS = bit_width(WIDTH_U << OUTPUT_FRAC_BITS);
  localparam int Y_OUTPUT_BITS = bit_width(HEIGHT_U << OUTPUT_FRAC_BITS);
  localparam int X_OUTPUT_SIGNED_BITS = X_OUTPUT_BITS + 1;
  localparam int Y_OUTPUT_SIGNED_BITS = Y_OUTPUT_BITS + 1;
  localparam longint unsigned X_NORMALIZED_BOUND =
      (WIDTH_U - 1) << (NORMALIZED_MOMENT_FRAC_BITS + 1);
  localparam longint unsigned Y_NORMALIZED_BOUND =
      (HEIGHT_U - 1) << (NORMALIZED_MOMENT_FRAC_BITS + 1);
  localparam int X_NORMALIZED_BITS = bit_width(X_NORMALIZED_BOUND + 1) + 1;
  localparam int Y_NORMALIZED_BITS = bit_width(Y_NORMALIZED_BOUND + 1) + 1;
  localparam int X_PRODUCT_BITS = X_NORMALIZED_BITS + RECIPROCAL_BITS + 1;
  localparam int Y_PRODUCT_BITS = Y_NORMALIZED_BITS + RECIPROCAL_BITS + 1;
  localparam bit POWER2_MULTIPLIER_NORMALIZER_SUPPORTED =
      (M_BITS - 1) >= INTERNAL_FRAC_BITS;

  input  logic clk;
  input  logic rst;
  input  logic in_valid;
  input  logic [M_BITS-1:0] m;
  input  logic [SX_BITS-1:0] sx;
  input  logic [SY_BITS-1:0] sy;
  output logic out_valid;
  output logic centroid_valid;
  output logic [X_OUTPUT_BITS-1:0] x_q4;
  output logic [Y_OUTPUT_BITS-1:0] y_q4;
  output logic [LOG_M_Q53_BITS-1:0] m_log_q5_3;

  logic normalize_in_valid;
  logic [M_BITS-1:0] normalize_m;
  logic [SX_BITS-1:0] normalize_sx;
  logic [SY_BITS-1:0] normalize_sy;

  generate
    if (REGISTER_INPUT) begin : registered_input
      always_ff @(posedge clk) begin
        if (rst) normalize_in_valid <= 1'b0;
        else normalize_in_valid <= in_valid;
        normalize_m <= m;
        normalize_sx <= sx;
        normalize_sy <= sy;
      end
    end else begin : direct_input
      assign normalize_in_valid = in_valid;
      assign normalize_m = m;
      assign normalize_sx = sx;
      assign normalize_sy = sy;
    end
  endgenerate

  logic [RECIPROCAL_WORD_BITS-1:0] reciprocal_segment_rom
      [0:(1 << RECIPROCAL_TABLE_SEGMENT_BITS)-1];

  initial begin
    $readmemb(RECIPROCAL_TABLE_BIN, reciprocal_segment_rom);
  end

  function automatic logic [NORM_EXP_BITS-1:0] norm_leading_one(
    input logic [NORM_BITS-1:0] bits
  );
    begin
      norm_leading_one = '0;
      for (int bit_index = 0; bit_index < NORM_BITS; bit_index++) begin
        if (bits[bit_index]) begin
          norm_leading_one = bit_index[NORM_EXP_BITS-1:0];
        end
      end
    end
  endfunction

  function automatic logic [2:0] log_m_fraction_q3(
    input logic [INTERNAL_FRAC_BITS-1:0] normalized_fraction
  );
    begin
      if (normalized_fraction < 21'd868669) begin
        if (normalized_fraction < 21'd396812) begin
          log_m_fraction_q3 =
              (normalized_fraction < 21'd189812) ? 3'd0 : 3'd1;
        end else begin
          log_m_fraction_q3 =
              (normalized_fraction < 21'd622506) ? 3'd2 : 3'd3;
        end
      end else begin
        if (normalized_fraction < 21'd1429837) begin
          log_m_fraction_q3 =
              (normalized_fraction < 21'd1137124) ? 3'd4 : 3'd5;
        end else if (normalized_fraction < 21'd1749040) begin
          log_m_fraction_q3 = 3'd6;
        end else begin
          // The quantized logarithm has a one-code dip at this
          // normalized mantissa.
          log_m_fraction_q3 =
              (normalized_fraction == 21'd1749041) ? 3'd6 : 3'd7;
        end
      end
    end
  endfunction

  function automatic logic [LOG_M_Q53_BITS-1:0] log_m_q5_3_from_normalized(
    input logic [NORM_EXP_BITS-1:0] exponent,
    input logic [INTERNAL_FRAC_BITS-1:0] normalized_fraction
  );
    logic [LOG_M_Q53_WIDE_BITS-1:0] wide_key;
    begin
      wide_key = LOG_M_Q53_WIDE_BITS'(exponent);
      wide_key = (wide_key << LOG_M_Q53_FRAC_BITS) |
          LOG_M_Q53_WIDE_BITS'(log_m_fraction_q3(normalized_fraction));
      if (wide_key[LOG_M_Q53_BITS]) begin
        log_m_q5_3_from_normalized = {LOG_M_Q53_BITS{1'b1}};
      end else begin
        log_m_q5_3_from_normalized = wide_key[LOG_M_Q53_BITS-1:0];
      end
    end
  endfunction

  function automatic logic [1:0] reciprocal_base_high(
    input logic [RECIPROCAL_SEGMENT_BITS-1:0] segment
  );
    begin
      if (segment < 8'd86) begin
        reciprocal_base_high = (segment < 8'd37) ? 2'd3 : 2'd2;
      end else begin
        reciprocal_base_high = (segment < 8'd154) ? 2'd1 : 2'd0;
      end
    end
  endfunction

  function automatic logic [2:0] reciprocal_drop_high(
    input logic [RECIPROCAL_SEGMENT_BITS-1:0] segment
  );
    begin
      if (segment < 8'd68) begin
        if (segment < 8'd40) begin
          reciprocal_drop_high = (segment < 8'd18) ? 3'd7 : 3'd6;
        end else begin
          reciprocal_drop_high = 3'd5;
        end
      end else if (segment < 8'd107) begin
        reciprocal_drop_high = 3'd4;
      end else begin
        reciprocal_drop_high = (segment < 8'd163) ? 3'd3 : 3'd2;
      end
    end
  endfunction

  function automatic logic signed [X_OUTPUT_SIGNED_BITS-1:0] x_product_to_q4(
    input logic signed [X_PRODUCT_BITS-1:0] product
  );
    logic signed [X_PRODUCT_BITS:0] product_ext;
    logic signed [X_PRODUCT_BITS:0] round_bias;
    begin
      product_ext = $signed({product[X_PRODUCT_BITS-1], product});
      round_bias = '0;
      round_bias[NORMALIZED_PRODUCT_SHIFT - 1] = 1'b1;
      x_product_to_q4 = X_OUTPUT_SIGNED_BITS'((product_ext + round_bias) >>> NORMALIZED_PRODUCT_SHIFT);
    end
  endfunction

  function automatic logic signed [Y_OUTPUT_SIGNED_BITS-1:0] y_product_to_q4(
    input logic signed [Y_PRODUCT_BITS-1:0] product
  );
    logic signed [Y_PRODUCT_BITS:0] product_ext;
    logic signed [Y_PRODUCT_BITS:0] round_bias;
    begin
      product_ext = $signed({product[Y_PRODUCT_BITS-1], product});
      round_bias = '0;
      round_bias[NORMALIZED_PRODUCT_SHIFT - 1] = 1'b1;
      y_product_to_q4 = Y_OUTPUT_SIGNED_BITS'((product_ext + round_bias) >>> NORMALIZED_PRODUCT_SHIFT);
    end
  endfunction

  logic signed [CENTER_X_BITS-1:0] x2;
  logic signed [CENTER_Y_BITS-1:0] y2;
  logic signed [X_NORMALIZED_BITS-1:0] x_normalized;
  logic signed [Y_NORMALIZED_BITS-1:0] y_normalized;
  logic [NORM_EXP_BITS-1:0] m_exp;
  logic [INTERNAL_FRAC_BITS-1:0] m_u;
  logic [INTERNAL_FRAC_BITS-1:0] m_norm_window;

  assign x2 = $signed(normalize_sx);
  assign y2 = $signed(normalize_sy);
  assign m_exp = norm_leading_one(NORM_BITS'(normalize_m));

  assign m_u = (normalize_m != '0) ? m_norm_window : '0;

  generate
    if (POWER2_MULTIPLIER_NORMALIZER &&
        POWER2_MULTIPLIER_NORMALIZER_SUPPORTED) begin : power2_normalizer
      CentroidPower2MultiplierNormalizer #(
        .MASS_BITS(M_BITS),
        .X_BITS(CENTER_X_BITS),
        .Y_BITS(CENTER_Y_BITS),
        .X_OUTPUT_BITS(X_NORMALIZED_BITS),
        .Y_OUTPUT_BITS(Y_NORMALIZED_BITS),
        .MASS_FRAC_BITS(INTERNAL_FRAC_BITS),
        .MOMENT_FRAC_BITS(NORMALIZED_MOMENT_FRAC_BITS),
        .EXP_BITS(NORM_EXP_BITS)
      ) normalize (
        .mass(normalize_m),
        .x_value(x2),
        .y_value(y2),
        .exponent(m_exp),
        .mass_window(m_norm_window),
        .x_normalized(x_normalized),
        .y_normalized(y_normalized)
      );
    end else begin : staged_normalizer
      CentroidMantissaWindow #(
        .VALUE_BITS(M_BITS),
        .FRAC_BITS(INTERNAL_FRAC_BITS),
        .EXP_BITS(NORM_EXP_BITS)
      ) m_window (
        .value(normalize_m),
        .exponent(m_exp),
        .window(m_norm_window)
      );

      CentroidSignedMomentNormalizer #(
        .VALUE_BITS(CENTER_X_BITS),
        .OUTPUT_BITS(X_NORMALIZED_BITS),
        .FRAC_BITS(NORMALIZED_MOMENT_FRAC_BITS),
        .EXP_BITS(NORM_EXP_BITS)
      ) x_moment_normalizer (
        .value(x2),
        .exponent(m_exp),
        .normalized(x_normalized)
      );

      CentroidSignedMomentNormalizer #(
        .VALUE_BITS(CENTER_Y_BITS),
        .OUTPUT_BITS(Y_NORMALIZED_BITS),
        .FRAC_BITS(NORMALIZED_MOMENT_FRAC_BITS),
        .EXP_BITS(NORM_EXP_BITS)
      ) y_moment_normalizer (
        .value(y2),
        .exponent(m_exp),
        .normalized(y_normalized)
      );
    end
  endgenerate

  logic s1_valid;
  logic s1_centroid_valid;
  logic [NORM_EXP_BITS-1:0] s1_m_exp;
  logic [INTERNAL_FRAC_BITS-1:0] s1_m_u;
  logic signed [X_NORMALIZED_BITS-1:0] s1_x_normalized;
  logic signed [Y_NORMALIZED_BITS-1:0] s1_y_normalized;
  logic [RECIPROCAL_WORD_BITS-1:0] s1_reciprocal_word;
  logic unused_reciprocal_rom_padding;
  assign unused_reciprocal_rom_padding = ^s1_reciprocal_word;

  logic [RECIPROCAL_BITS-1:0] interpolated_reciprocal;

  generate
    if (MIDPOINT_REFLECTION_RECIPROCAL) begin : midpoint_reflection_reciprocal
      logic [RECIPROCAL_MIDPOINT_COEFFICIENT_BITS-1:0] coefficient;

      assign coefficient = s1_reciprocal_word[
          RECIPROCAL_MIDPOINT_COEFFICIENT_BITS-1:0
      ];

      CentroidMidpointReflectionReciprocal #(
        .FRACTION_BITS(INTERNAL_FRAC_BITS),
        .SEGMENT_BITS(RECIPROCAL_MIDPOINT_SEGMENT_BITS),
        .DISCARD_BITS(RECIPROCAL_MIDPOINT_DISCARD_BITS),
        .COEFFICIENT_BITS(RECIPROCAL_MIDPOINT_COEFFICIENT_BITS),
        .RECIPROCAL_BITS(RECIPROCAL_BITS),
        .PRODUCT_SHIFT(RECIPROCAL_MIDPOINT_PRODUCT_SHIFT)
      ) direct_reciprocal (
        .normalized_fraction(s1_m_u),
        .coefficient(coefficient),
        .reciprocal(interpolated_reciprocal)
      );
    end else begin : segmented_reciprocal
      logic [RECIPROCAL_BITS-1:0] s1_reciprocal_base;
      logic [RECIPROCAL_DROP_BITS-1:0] s1_reciprocal_drop;
      logic [RECIPROCAL_SEGMENT_BITS-1:0] s1_reciprocal_segment;
      logic signed [RECIPROCAL_PRODUCT_BITS:0] reciprocal_drop_product_signed;
      logic signed [RECIPROCAL_SPLIT_C_BITS-1:0] reciprocal_split_c;
      logic signed [RECIPROCAL_PRODUCT_BITS:0] reciprocal_split_c_extended;
      logic signed [RECIPROCAL_PRODUCT_BITS:0] reciprocal_split_numerator;
      logic signed [RECIPROCAL_SPLIT_TERM_BITS-1:0] reciprocal_split_term;
      logic signed [RECIPROCAL_BASE_HIGH_SUM_BITS-1:0] reciprocal_split_term_high;
      logic signed [RECIPROCAL_BASE_HIGH_BITS-1:0] reciprocal_split_high;

      assign s1_reciprocal_segment =
          s1_m_u[INTERNAL_FRAC_BITS-1 -: RECIPROCAL_SEGMENT_BITS];
      assign s1_reciprocal_base = {
          1'b1,
          reciprocal_base_high(s1_reciprocal_segment),
          s1_reciprocal_word[
              RECIPROCAL_WORD_BITS-1 -: RECIPROCAL_STORED_BASE_BITS
          ]
      };
      assign s1_reciprocal_drop = {
          reciprocal_drop_high(s1_reciprocal_segment),
          s1_reciprocal_word[RECIPROCAL_STORED_DROP_BITS-1:0]
      };
      assign reciprocal_drop_product_signed =
          $signed({1'b0, s1_reciprocal_drop}) *
          $signed({1'b0, s1_m_u[RECIPROCAL_INTERPOLATION_BITS-1:0]});
      assign reciprocal_split_c = $signed({
          1'b0,
          s1_reciprocal_base[RECIPROCAL_BASE_SPLIT_BITS-1:0],
          1'b0,
          {(RECIPROCAL_INTERPOLATION_BITS-1){1'b1}}
      });
      assign reciprocal_split_c_extended = {
          {(RECIPROCAL_PRODUCT_BITS + 1 - RECIPROCAL_SPLIT_C_BITS){
              reciprocal_split_c[RECIPROCAL_SPLIT_C_BITS-1]
          }},
          reciprocal_split_c
      };
      assign reciprocal_split_numerator =
          reciprocal_split_c_extended - reciprocal_drop_product_signed;
      assign reciprocal_split_term = RECIPROCAL_SPLIT_TERM_BITS'(
          $signed(reciprocal_split_numerator) >>> RECIPROCAL_INTERPOLATION_BITS
      );
      assign reciprocal_split_term_high = RECIPROCAL_BASE_HIGH_SUM_BITS'(
          $signed(reciprocal_split_term) >>> RECIPROCAL_BASE_SPLIT_BITS
      );
      assign reciprocal_split_high = RECIPROCAL_BASE_HIGH_BITS'(
          $signed({
              1'b0,
              s1_reciprocal_base[
                  RECIPROCAL_BITS-1 -: RECIPROCAL_BASE_HIGH_BITS
              ]
          }) + reciprocal_split_term_high);
      assign interpolated_reciprocal = {
          reciprocal_split_high,
          reciprocal_split_term[RECIPROCAL_BASE_SPLIT_BITS-1:0]
      };
    end
  endgenerate

  logic s2_valid;
  logic s2_centroid_valid;
  logic signed [X_NORMALIZED_BITS-1:0] s2_x_normalized;
  logic signed [Y_NORMALIZED_BITS-1:0] s2_y_normalized;
  logic [RECIPROCAL_BITS-1:0] s2_reciprocal;
  logic [LOG_M_Q53_BITS-1:0] s2_m_log_q5_3;

  logic signed [X_PRODUCT_BITS-1:0] x_product;
  logic signed [Y_PRODUCT_BITS-1:0] y_product;

  assign x_product = s2_x_normalized * $signed({1'b0, s2_reciprocal});
  assign y_product = s2_y_normalized * $signed({1'b0, s2_reciprocal});

  logic s3_valid;
  logic s3_centroid_valid;
  logic signed [X_PRODUCT_BITS-1:0] s3_x_product;
  logic signed [Y_PRODUCT_BITS-1:0] s3_y_product;
  logic [LOG_M_Q53_BITS-1:0] s3_m_log_q5_3;

  logic signed [X_OUTPUT_SIGNED_BITS-1:0] qx_delta_q4;
  logic signed [Y_OUTPUT_SIGNED_BITS-1:0] qy_delta_q4;
  logic signed [X_OUTPUT_BITS-1:0] x_q4_sum;
  logic signed [Y_OUTPUT_BITS-1:0] y_q4_sum;
  localparam logic signed [X_OUTPUT_SIGNED_BITS-1:0] X_OFFSET_Q4 =
    $signed(X_OUTPUT_SIGNED_BITS'((WIDTH_U - 1) << (OUTPUT_FRAC_BITS - 1)));
  localparam logic signed [Y_OUTPUT_SIGNED_BITS-1:0] Y_OFFSET_Q4 =
    $signed(Y_OUTPUT_SIGNED_BITS'((HEIGHT_U - 1) << (OUTPUT_FRAC_BITS - 1)));

  assign qx_delta_q4 = x_product_to_q4(s3_x_product);
  assign qy_delta_q4 = y_product_to_q4(s3_y_product);
  assign x_q4_sum = X_OUTPUT_BITS'(X_OFFSET_Q4 + qx_delta_q4);
  assign y_q4_sum = Y_OUTPUT_BITS'(Y_OFFSET_Q4 + qy_delta_q4);

  always_ff @(posedge clk) begin
    if (rst) begin
      s1_valid <= 1'b0;
      s1_centroid_valid <= 1'b0;
      s2_valid <= 1'b0;
      s2_centroid_valid <= 1'b0;
      s3_valid <= 1'b0;
      s3_centroid_valid <= 1'b0;
      out_valid <= 1'b0;
      centroid_valid <= 1'b0;
      m_log_q5_3 <= '0;
    end else begin
      s1_valid <= normalize_in_valid;
      s1_centroid_valid <= normalize_in_valid && (normalize_m != '0);
      s1_m_exp <= m_exp;
      s1_m_u <= m_u;
      s1_x_normalized <= x_normalized;
      s1_y_normalized <= y_normalized;
      s1_reciprocal_word <= reciprocal_segment_rom[
          m_u[INTERNAL_FRAC_BITS-1 -: RECIPROCAL_TABLE_SEGMENT_BITS]
      ];

      s2_valid <= s1_valid;
      s2_centroid_valid <= s1_centroid_valid;
      s2_x_normalized <= s1_x_normalized;
      s2_y_normalized <= s1_y_normalized;
      s2_reciprocal <= interpolated_reciprocal;
      s2_m_log_q5_3 <= s1_centroid_valid ?
          log_m_q5_3_from_normalized(s1_m_exp, s1_m_u) : '0;

      s3_valid <= s2_valid;
      s3_centroid_valid <= s2_centroid_valid;
      s3_x_product <= x_product;
      s3_y_product <= y_product;
      s3_m_log_q5_3 <= s2_m_log_q5_3;

      out_valid <= s3_valid;
      centroid_valid <= s3_centroid_valid;
      m_log_q5_3 <= s3_valid ? s3_m_log_q5_3 : '0;
    end

    x_q4 <= s3_centroid_valid ? x_q4_sum : '0;
    y_q4 <= s3_centroid_valid ? y_q4_sum : '0;
  end
endmodule
