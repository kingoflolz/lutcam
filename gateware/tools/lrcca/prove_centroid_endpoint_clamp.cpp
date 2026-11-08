// Exhaustive production-geometry proof for CentroidCompressor's endpoint clamp.
//
// Compile and run from the repository root:
//   c++ -O3 -std=c++20 tools/lrcca/prove_centroid_endpoint_clamp.cpp \
//       -o build/prove_centroid_endpoint_clamp
//   build/prove_centroid_endpoint_clamp

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>

namespace {

constexpr int kWidth = 1696;
constexpr int kHeight = 800;
constexpr int kPixelBits = 8;
constexpr int kInternalFracBits = 21;
constexpr int kMomentFracBits = 4;
constexpr int kOutputFracBits = 4;
constexpr int kReciprocalFracBits = 17;
constexpr int kSegmentBits = 8;
constexpr int kInterpolationBits = kInternalFracBits - kSegmentBits;
constexpr std::int64_t kMaxMass =
    std::int64_t{kWidth} * kHeight * ((std::int64_t{1} << kPixelBits) - 1);

constexpr int ReciprocalEndpoint(int index) {
  const std::int64_t denominator = (1 << kSegmentBits) + index;
  const std::int64_t numerator =
      (std::int64_t{1} << kReciprocalFracBits) * (1 << kSegmentBits);
  const int rounded = static_cast<int>((numerator + denominator / 2) / denominator);
  return std::min(rounded, (1 << kReciprocalFracBits) - 1);
}

constexpr std::array<int, (1 << kSegmentBits) + 1> MakeEndpoints() {
  std::array<int, (1 << kSegmentBits) + 1> endpoints{};
  for (int index = 0; index <= (1 << kSegmentBits); ++index) {
    endpoints[index] = ReciprocalEndpoint(index);
  }
  return endpoints;
}

constexpr auto kEndpoints = MakeEndpoints();

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

int ReciprocalQ17(std::uint32_t normalized_fraction) {
  const int segment = normalized_fraction >> kInterpolationBits;
  const int fraction = normalized_fraction & ((1 << kInterpolationBits) - 1);
  const int base = kEndpoints[segment];
  const int drop = base - kEndpoints[segment + 1];
  const int interpolated_drop =
      (drop * fraction + (1 << (kInterpolationBits - 1))) >> kInterpolationBits;
  return base - interpolated_drop;
}

std::int64_t NormalizeMomentQ4(std::int64_t centered_twice_moment, int exponent) {
  const int shift = exponent - kMomentFracBits;
  if (shift <= 0) {
    return centered_twice_moment << -shift;
  }
  return RoundShiftTowardPositive(centered_twice_moment, shift);
}

std::int64_t ProductToQ4(std::int64_t normalized_moment, int reciprocal_q17) {
  constexpr int kProductShift =
      kMomentFracBits + kReciprocalFracBits + 1 - kOutputFracBits;
  return RoundShiftTowardPositive(normalized_moment * reciprocal_q17, kProductShift);
}

std::int64_t OldClamp(std::int64_t value, int last_coordinate) {
  const std::int64_t maximum = std::int64_t{last_coordinate} << kOutputFracBits;
  return value < 0 ? 0 : std::min(value, maximum);
}

std::int64_t DirectCoordinate(std::int64_t value) {
  return value;
}

struct Witness {
  std::int64_t sum = 0;
  std::int64_t mass = 0;
  std::uint32_t cell = 0;
  int exponent = 0;
  int sign = 0;
};

struct Bounds {
  Witness minimum{std::numeric_limits<std::int64_t>::max()};
  Witness maximum{std::numeric_limits<std::int64_t>::min()};

  void Observe(std::int64_t sum, std::int64_t mass, std::uint32_t cell,
               int exponent, int sign) {
    const Witness witness{sum, mass, cell, exponent, sign};
    if (sum < minimum.sum) {
      minimum = witness;
    }
    if (sum > maximum.sum) {
      maximum = witness;
    }
  }
};

void CheckAxis(std::int64_t mass, std::uint32_t cell, int exponent,
               int last_coordinate, Bounds* bounds) {
  const int reciprocal = ReciprocalQ17(cell);
  const std::int64_t offset =
      std::int64_t{last_coordinate} << (kOutputFracBits - 1);
  const std::int64_t endpoint_moment = mass * last_coordinate;

  for (const int sign : {-1, 1}) {
    const std::int64_t normalized = NormalizeMomentQ4(sign * endpoint_moment, exponent);
    const std::int64_t sum = offset + ProductToQ4(normalized, reciprocal);
    assert(OldClamp(sum, last_coordinate) == DirectCoordinate(sum));
    bounds->Observe(sum, mass, cell, exponent, sign);
  }
}

void PrintWitness(const char* label, const Witness& witness) {
  std::cout << label << '=' << witness.sum << " mass=" << witness.mass
            << " exponent=" << witness.exponent << " cell=" << witness.cell
            << " sign=" << witness.sign << '\n';
}

}  // namespace

int main() {
  Bounds x_bounds;
  Bounds y_bounds;
  std::uint64_t endpoint_evaluations = 0;

  const auto evaluate = [&](std::int64_t mass, std::uint32_t cell, int exponent) {
    CheckAxis(mass, cell, exponent, kWidth - 1, &x_bounds);
    CheckAxis(mass, cell, exponent, kHeight - 1, &y_bounds);
    ++endpoint_evaluations;
  };

  // Through exponent 21, the Q21 normalized mass is exact, so each mass is
  // one cell with one endpoint. Above exponent 21, each discarded-bit cell
  // contributes both inclusive source-mass endpoints.
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
        (std::min(kMaxMass, (std::int64_t{1} << (exponent + 1)) - 1) - first_mass) /
        cell_width);
    for (std::uint32_t cell = 0; cell <= last_cell; ++cell) {
      const std::int64_t low = first_mass + std::int64_t{cell} * cell_width;
      const std::int64_t high = std::min(low + cell_width - 1, kMaxMass);
      evaluate(low, cell, exponent);
      evaluate(high, cell, exponent);
    }
  }

  constexpr std::uint64_t kExpectedEndpointEvaluations = 30'571'825;
  assert(endpoint_evaluations == kExpectedEndpointEvaluations);

  std::cout << "endpoint_evaluations=" << endpoint_evaluations << '\n';
  PrintWitness("x_min", x_bounds.minimum);
  PrintWitness("x_max", x_bounds.maximum);
  PrintWitness("y_min", y_bounds.minimum);
  PrintWitness("y_max", y_bounds.maximum);
  assert(x_bounds.minimum.sum == 0);
  assert(x_bounds.maximum.sum == ((kWidth - 1) << kOutputFracBits));
  assert(y_bounds.minimum.sum == 0);
  assert(y_bounds.maximum.sum == ((kHeight - 1) << kOutputFracBits));
  std::cout << "old_clamp_vs_direct_coordinate=identical\n";
}
