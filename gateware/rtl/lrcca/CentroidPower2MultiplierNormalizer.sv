// Shared, vendor-neutral centroid normalization logic.

import lrcca_util_pkg::bit_width;
import lrcca_util_pkg::centered_moment_bits;
import lrcca_util_pkg::max_int;

module CentroidPower2MultiplierNormalizer #(
  parameter int MASS_BITS = 22,
  parameter int X_BITS = 2,
  parameter int Y_BITS = 2,
  parameter int X_OUTPUT_BITS = 2,
  parameter int Y_OUTPUT_BITS = 2,
  parameter int MASS_FRAC_BITS = 21,
  parameter int MOMENT_FRAC_BITS = 4,
  parameter int EXP_BITS = 1,
  localparam int E_MAX = MASS_BITS - 1,
  localparam int SCALE_BITS = E_MAX + 1,
  localparam int SIGNED_SCALE_BITS = SCALE_BITS + 1,
  localparam int MASS_PRODUCT_BITS = MASS_BITS + SCALE_BITS,
  localparam int X_PRODUCT_BITS = X_BITS + SIGNED_SCALE_BITS,
  localparam int Y_PRODUCT_BITS = Y_BITS + SIGNED_SCALE_BITS,
  localparam int MASS_WINDOW_LSB = E_MAX - MASS_FRAC_BITS,
  localparam int MOMENT_SHIFT = E_MAX - MOMENT_FRAC_BITS
)(
  input logic [MASS_BITS-1:0] mass,
  input logic signed [X_BITS-1:0] x_value,
  input logic signed [Y_BITS-1:0] y_value,
  input logic [EXP_BITS-1:0] exponent,
  output logic [MASS_FRAC_BITS-1:0] mass_window,
  output logic signed [X_OUTPUT_BITS-1:0] x_normalized,
  output logic signed [Y_OUTPUT_BITS-1:0] y_normalized
);
  logic [EXP_BITS-1:0] scale_shift;
  logic [SCALE_BITS-1:0] scale;
  logic signed [SIGNED_SCALE_BITS-1:0] signed_scale;
  logic [MASS_PRODUCT_BITS-1:0] mass_product;
  logic signed [X_PRODUCT_BITS-1:0] x_product;
  logic signed [Y_PRODUCT_BITS-1:0] y_product;
  logic signed [X_PRODUCT_BITS-1:0] x_shifted;
  logic signed [Y_PRODUCT_BITS-1:0] y_shifted;
  logic signed [X_PRODUCT_BITS-1:0] x_round_increment;
  logic signed [Y_PRODUCT_BITS-1:0] y_round_increment;

  assign scale_shift = EXP_BITS'(E_MAX) - exponent;
  assign scale = SCALE_BITS'(1) << scale_shift;
  assign signed_scale = $signed({1'b0, scale});

  assign mass_product = mass * scale;
  assign x_product = x_value * signed_scale;
  assign y_product = y_value * signed_scale;

  // mass_product is mass * 2^(E_MAX-exponent).  Bit E_MAX is the
  // normalized leading one, so this fixed slice is the fractional window.
  assign mass_window = mass_product[MASS_WINDOW_LSB +: MASS_FRAC_BITS];

  // The same scale turns both variable moment shifts into one fixed shift.
  // For exponent <= MOMENT_FRAC_BITS the discarded bits are all zero.  For a
  // larger exponent, bit MOMENT_SHIFT-1 is exactly the old signed half bit.
  assign x_shifted = x_product >>> MOMENT_SHIFT;
  assign y_shifted = y_product >>> MOMENT_SHIFT;
  assign x_round_increment = X_PRODUCT_BITS'(x_product[MOMENT_SHIFT-1]);
  assign y_round_increment = Y_PRODUCT_BITS'(y_product[MOMENT_SHIFT-1]);
  assign x_normalized = X_OUTPUT_BITS'(x_shifted + x_round_increment);
  assign y_normalized = Y_OUTPUT_BITS'(y_shifted + y_round_increment);
  // The full product is intentional; only the normalized mantissa window is emitted.
  logic unused_mass_product_bits;
  assign unused_mass_product_bits = ^mass_product;

endmodule
