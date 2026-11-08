// Exhaustive production proof for the midpoint-reflection reciprocal table.

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

namespace {

constexpr int kWidth = 1696;
constexpr int kHeight = 800;
constexpr int kPixelBits = 8;
constexpr int kInternalFracBits = 21;
constexpr int kMomentFracBits = 4;
constexpr int kOutputFracBits = 4;
constexpr int kReciprocalFracBits = 17;
constexpr int kSegmentBits = 9;
constexpr int kResidualBits = kInternalFracBits - kSegmentBits;
constexpr int kDiscardBits = 3;
constexpr int kProductShift = 19;
constexpr int kSegmentCount = 1 << kSegmentBits;
constexpr std::int64_t kNormalizedScale = std::int64_t{1} << kInternalFracBits;
constexpr std::int64_t kReciprocalTarget =
    std::int64_t{1} << (kInternalFracBits + kReciprocalFracBits);
constexpr std::int64_t kMaxMass =
    std::int64_t{kWidth} * kHeight * ((std::int64_t{1} << kPixelBits) - 1);

std::array<int, kSegmentCount> LoadCoefficients() {
  std::ifstream input(
      "rtl/lrcca/centroid_reciprocal_midpoint_reflection_q17.bin");
  assert(input.good());
  std::array<int, kSegmentCount> coefficients{};
  std::string line;
  int index = 0;
  while (input >> line) {
    assert(index < kSegmentCount);
    assert(line.size() == 20);
    assert(line[0] == '0' && line[1] == '0');
    coefficients[index++] = std::stoi(line, nullptr, 2);
  }
  assert(index == kSegmentCount);
  assert(*std::min_element(coefficients.begin(), coefficients.end()) == 65'601);
  assert(*std::max_element(coefficients.begin(), coefficients.end()) == 261'633);
  return coefficients;
}

std::int64_t FloorDividePowerOfTwo(std::int64_t value, int shift) {
  assert(shift > 0);
  const std::int64_t scale = std::int64_t{1} << shift;
  if (value >= 0) {
    return value / scale;
  }
  return -((-value + scale - 1) / scale);
}

std::int64_t RoundShiftTowardPositive(std::int64_t value, int shift) {
  assert(shift > 0);
  return FloorDividePowerOfTwo(value + (std::int64_t{1} << (shift - 1)), shift);
}

int ReciprocalQ17(std::uint32_t normalized_fraction,
                  const std::array<int, kSegmentCount>& coefficients) {
  const std::uint32_t segment = normalized_fraction >> kResidualBits;
  const std::uint32_t fraction =
      normalized_fraction & ((std::uint32_t{1} << kResidualBits) - 1);
  const std::int64_t reflected = (
      kNormalizedScale + (std::int64_t{segment} << kResidualBits)
      + ((std::int64_t{1} << kResidualBits) - 1) - fraction) >> kDiscardBits;
  const std::int64_t product = std::int64_t{coefficients[segment]} * reflected;
  return static_cast<int>(product >> kProductShift);
}

std::int64_t NormalizeMomentQ4(std::int64_t centered_twice_moment, int exponent) {
  const int shift = exponent - kMomentFracBits;
  if (shift <= 0) {
    return centered_twice_moment << -shift;
  }
  return RoundShiftTowardPositive(centered_twice_moment, shift);
}

std::int64_t ProductToQ4(std::int64_t normalized_moment, int reciprocal_q17) {
  constexpr int kShift =
      kMomentFracBits + kReciprocalFracBits + 1 - kOutputFracBits;
  return RoundShiftTowardPositive(normalized_moment * reciprocal_q17, kShift);
}

struct Bounds {
  std::int64_t minimum = std::numeric_limits<std::int64_t>::max();
  std::int64_t maximum = std::numeric_limits<std::int64_t>::min();

  void Observe(std::int64_t value) {
    minimum = std::min(minimum, value);
    maximum = std::max(maximum, value);
  }
};

void CheckAxis(std::int64_t mass, std::uint32_t cell, int exponent,
               int last_coordinate,
               const std::array<int, kSegmentCount>& coefficients,
               Bounds* bounds) {
  const int reciprocal = ReciprocalQ17(cell, coefficients);
  const std::int64_t offset =
      std::int64_t{last_coordinate} << (kOutputFracBits - 1);
  const std::int64_t endpoint_moment = mass * last_coordinate;
  for (const int sign : {-1, 1}) {
    const std::int64_t normalized =
        NormalizeMomentQ4(sign * endpoint_moment, exponent);
    bounds->Observe(offset + ProductToQ4(normalized, reciprocal));
  }
}

}  // namespace

int main() {
  const auto coefficients = LoadCoefficients();

  std::int64_t maximum_error = 0;
  std::uint32_t worst_fraction = 0;
  std::uint32_t worst_endpoint = 0;
  int worst_reciprocal = 0;
  int minimum_reciprocal = 1 << kReciprocalFracBits;
  int maximum_reciprocal = 0;
  std::int64_t minimum_product = std::numeric_limits<std::int64_t>::max();
  std::int64_t maximum_product = 0;
  for (std::uint32_t normalized_fraction = 0;
       normalized_fraction < (std::uint32_t{1} << kInternalFracBits);
       ++normalized_fraction) {
    const std::uint32_t segment = normalized_fraction >> kResidualBits;
    const std::uint32_t fraction =
        normalized_fraction & ((std::uint32_t{1} << kResidualBits) - 1);
    const std::int64_t reflected = (
        kNormalizedScale + (std::int64_t{segment} << kResidualBits)
        + ((std::int64_t{1} << kResidualBits) - 1) - fraction) >> kDiscardBits;
    const std::int64_t product = std::int64_t{coefficients[segment]} * reflected;
    const int reciprocal = static_cast<int>(product >> kProductShift);
    assert(reciprocal == ReciprocalQ17(normalized_fraction, coefficients));
    minimum_reciprocal = std::min(minimum_reciprocal, reciprocal);
    maximum_reciprocal = std::max(maximum_reciprocal, reciprocal);
    minimum_product = std::min(minimum_product, product);
    maximum_product = std::max(maximum_product, product);

    for (const std::uint32_t endpoint :
         {normalized_fraction, normalized_fraction + 1}) {
      const std::int64_t denominator = kNormalizedScale + endpoint;
      const std::int64_t signed_error = reciprocal * denominator - kReciprocalTarget;
      const std::int64_t error = signed_error < 0 ? -signed_error : signed_error;
      if (error > maximum_error) {
        maximum_error = error;
        worst_fraction = normalized_fraction;
        worst_endpoint = endpoint;
        worst_reciprocal = reciprocal;
      }
    }
  }
  assert(maximum_error == 4'257'008);
  assert(worst_fraction == 2'055'271);
  assert(worst_endpoint == 2'055'272);
  assert(worst_reciprocal == 66'198);
  assert(minimum_reciprocal == 65'536);
  assert(maximum_reciprocal == 131'071);
  assert(minimum_product == 34'360'229'376);
  assert(maximum_product == 68'719'215'615);
  assert(maximum_product < (std::int64_t{1} << 36));

  Bounds x_bounds;
  Bounds y_bounds;
  std::uint64_t endpoint_evaluations = 0;
  const auto evaluate = [&](std::int64_t mass, std::uint32_t cell, int exponent) {
    CheckAxis(mass, cell, exponent, kWidth - 1, coefficients, &x_bounds);
    CheckAxis(mass, cell, exponent, kHeight - 1, coefficients, &y_bounds);
    ++endpoint_evaluations;
  };

  for (int exponent = 0; exponent <= 21; ++exponent) {
    const std::int64_t first_mass = std::int64_t{1} << exponent;
    const std::int64_t mass_count = std::int64_t{1} << exponent;
    for (std::int64_t offset = 0; offset < mass_count; ++offset) {
      const std::int64_t mass = first_mass + offset;
      const std::uint32_t cell = exponent == 21
          ? static_cast<std::uint32_t>(offset)
          : static_cast<std::uint32_t>(offset << (kInternalFracBits - exponent));
      evaluate(mass, cell, exponent);
    }
  }

  for (int exponent = 22; exponent <= 28; ++exponent) {
    const std::int64_t first_mass = std::int64_t{1} << exponent;
    const int discarded_bits = exponent - kInternalFracBits;
    const std::int64_t cell_width = std::int64_t{1} << discarded_bits;
    const std::uint32_t last_cell = static_cast<std::uint32_t>(
        (std::min(kMaxMass, (std::int64_t{1} << (exponent + 1)) - 1)
         - first_mass) / cell_width);
    for (std::uint32_t cell = 0; cell <= last_cell; ++cell) {
      const std::int64_t low = first_mass + std::int64_t{cell} * cell_width;
      const std::int64_t high = std::min(low + cell_width - 1, kMaxMass);
      evaluate(low, cell, exponent);
      evaluate(high, cell, exponent);
    }
  }

  assert(endpoint_evaluations == 30'571'825);
  assert(x_bounds.minimum == 0 && x_bounds.maximum == 27'120);
  assert(y_bounds.minimum == 0 && y_bounds.maximum == 12'784);
  std::cout << "midpoint_reflection_error=" << maximum_error << "/2^38\n";
  std::cout << "endpoint_evaluations=" << endpoint_evaluations << '\n';
  std::cout << "x_range=[" << x_bounds.minimum << ',' << x_bounds.maximum << "]\n";
  std::cout << "y_range=[" << y_bounds.minimum << ',' << y_bounds.maximum << "]\n";
  std::cout << "midpoint-reflection production endpoint proof passed\n";
}
