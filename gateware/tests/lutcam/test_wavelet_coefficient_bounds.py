"""Certify the fixed byte-raster wavelet widths using exact arithmetic.

The opt-in BOUNDED_BYTE_RASTER implementation receives a 1280x800 raster of
unsigned bytes, centered to [-128, 127]. Its reversible CDF 5/3 equations are
    d[j] = odd[j] - floor((even[j] + even[min(j+1,last)]) / 2)
    l[j] = even[j] + floor((d[max(j-1,0)] + d[j] + 2) / 4).
LL4 additionally averages adjacent H4-low rows as floor((a+b+1)/2).

Replacing each floor with real arithmetic gives a separable linear operator.
We construct every finite-axis output stencil with Fraction coefficients;
this includes the duplicated even sample at the right/bottom boundary and
the duplicated detail at the left/top boundary. A centered byte has midpoint
-1/2 and radius 255/2. A stencil's row sum propagates the midpoint; its L1 norm
bounds the radius. Products of the two axis norms bound the 2-D operator.

Local roundoff is bounded independently at each lifting stage. Exhausting
all five input residues modulo 8 gives L=[-1/4,3/4], H=[0,1/2]; averaging has
A=[0,1/2]. Modulo 8 suffices because changing any input by 8 changes both its
integer result and ideal result by the same integer. Each error interval is
propagated through the remaining stencils, conservatively ignoring correlation.
The final real interval is intersected with the integers. This proves a bound
for every byte raster, rather than sampling a set of example images.

The 23 paths cover all stored/emitted coefficients. All fit 11 signed bits;
the four raw input streams of the shared H2-H5 service fit 10 signed bits.
The first H/V stages retain their original 9/10-bit arithmetic. Predict sums
have W+1 bits and update sums have W+2 bits, so once coefficients fit W, their
intermediate sums do not require an additional range assumption.

This certificate assumes valid raster scheduling. CameraPipeline supplies a
reset before each frame, validates markers, and resets on a failed prefix.
The generic transform keeps BOUNDED_BYTE_RASTER disabled by default. This
mathematical test complements RTL equivalence tests; update the model if the
transform equations, boundary rules, dimensions, or stage topology change.
No RTL simulator, generated file, NumPy, or external proof artifact is needed.
"""

from collections import defaultdict
from fractions import Fraction
from functools import lru_cache
from itertools import product
import unittest


FRAME_SHAPE = (1280, 800)
EXPECTED_INTERVALS = {
    "H1-L": (-192, 191),
    "V1-LL": (-288, 288),
    "V1-LH": (-383, 384),
    "H1-H": (-255, 255),
    "V1-HL": (-382, 383),
    "V1-HH": (-510, 511),
    "H2-L": (-312, 313),
    "V2-LL": (-339, 340),
    "V2-LH": (-522, 522),
    "H2-H": (-480, 481),
    "V2-HL": (-521, 522),
    "V2-HH": (-801, 802),
    "H3-L": (-353, 355),
    "V3-LL": (-368, 370),
    "V3-LH": (-600, 600),
    "H3-H": (-576, 576),
    "V3-HL": (-599, 600),
    "V3-HH": (-975, 976),
    "H4-L": (-371, 373),
    "H4-H": (-614, 614),
    "H5-L": (-374, 377),
    "H5-H": (-619, 620),
    "LL4-average": (-295, 298),
}


def _combine(terms):
    """Add scaled sparse stencils without losing cancellation."""
    result = defaultdict(Fraction)
    for scale, row in terms:
        for index, coefficient in row.items():
            result[index] += scale * coefficient
    return {index: coefficient for index, coefficient in result.items()
            if coefficient}


@lru_cache(maxsize=None)
def _axis_norm_and_sum(size, operations):
    """Return exact maximum row L1 norm and common row sum."""
    rows = [{index: Fraction(1)} for index in range(size)]
    for operation in operations:
        if len(rows) % 2:
            raise AssertionError("Every transformed axis must have even size")
        count = len(rows) // 2
        if operation == "A":
            rows = [
                _combine(((Fraction(1, 2), rows[2 * j]),
                          (Fraction(1, 2), rows[2 * j + 1])))
                for j in range(count)
            ]
            continue
        details = [
            _combine(((Fraction(1), rows[2 * j + 1]),
                      (Fraction(-1, 2), rows[2 * j]),
                      (Fraction(-1, 2), rows[2 * min(j + 1, count - 1)])))
            for j in range(count)
        ]
        if operation == "H":
            rows = details
        elif operation == "L":
            rows = [
                _combine(((Fraction(1), rows[2 * j]),
                          (Fraction(1, 4), details[max(j - 1, 0)]),
                          (Fraction(1, 4), details[j])))
                for j in range(count)
            ]
        else:
            raise AssertionError(f"Unknown operation: {operation}")
    expected_sum = Fraction(0 if "H" in operations else 1)
    if any(sum(row.values()) != expected_sum for row in rows):
        raise AssertionError("The finite boundary stencils must preserve DC")
    return max(sum(abs(c) for c in row.values()) for row in rows), expected_sum


def _operator_norm_and_sum(shape, path):
    norm = Fraction(1)
    row_sum = Fraction(1)
    for axis, size in enumerate(shape):
        operations = tuple(kind for selected_axis, kind in path
                           if selected_axis == axis)
        axis_norm, axis_sum = _axis_norm_and_sum(size, operations)
        norm *= axis_norm
        row_sum *= axis_sum
    return norm, row_sum


@lru_cache(maxsize=None)
def _local_roundoff_intervals():
    low_errors, high_errors, average_errors = set(), set(), set()
    for even_before, odd_before, even, odd, even_after in product(range(8), repeat=5):
        high_real = Fraction(odd) - Fraction(even + even_after, 2)
        previous_real = Fraction(odd_before) - Fraction(even_before + even, 2)
        high = odd - (even + even_after) // 2
        previous = odd_before - (even_before + even) // 2
        low = even + (previous + high + 2) // 4
        low_errors.add(Fraction(low) - (even + (previous_real + high_real) / 4))
        high_errors.add(Fraction(high) - high_real)
    for even, odd in product(range(8), repeat=2):
        average_errors.add(Fraction((even + odd + 1) // 2)
                           - Fraction(even + odd, 2))
    return {kind: (min(errors), max(errors)) for kind, errors in (
        ("L", low_errors), ("H", high_errors), ("A", average_errors)
    )}


def _coefficient_paths():
    paths = {}
    prefix = ()
    for level in (1, 2, 3):
        for horizontal in ("L", "H"):
            horizontal_path = prefix + ((0, horizontal),)
            paths[f"H{level}-{horizontal}"] = horizontal_path
            for vertical in ("L", "H"):
                paths[f"V{level}-{horizontal}{vertical}"] = (
                    horizontal_path + ((1, vertical),))
        prefix += ((0, "L"), (1, "L"))
    for level in (4, 5):
        for horizontal in ("L", "H"):
            paths[f"H{level}-{horizontal}"] = prefix + ((0, horizontal),)
        prefix += ((0, "L"),)
    paths["LL4-average"] = paths["H4-L"] + ((1, "A"),)
    return paths


def _integer_interval(path):
    norm, row_sum = _operator_norm_and_sum(FRAME_SHAPE, path)
    midpoint = Fraction(-1, 2) * row_sum
    radius = Fraction(255, 2) * norm
    shape = list(FRAME_SHAPE)
    roundoff = _local_roundoff_intervals()
    for step, (axis, operation) in enumerate(path):
        shape[axis] //= 2
        norm, row_sum = _operator_norm_and_sum(shape, path[step + 1:])
        lower, upper = roundoff[operation]
        midpoint += (upper + lower) / 2 * row_sum
        radius += (upper - lower) / 2 * norm
    lower, upper = midpoint - radius, midpoint + radius
    # Integer outputs intersect [lower,upper] as [ceil(lower),floor(upper)].
    return (-((-lower.numerator) // lower.denominator),
            upper.numerator // upper.denominator)


class WaveletCoefficientBoundsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.intervals = {name: _integer_interval(path)
                         for name, path in _coefficient_paths().items()}

    def test_local_roundoff_intervals(self):
        self.assertEqual(_local_roundoff_intervals(), {
            "L": (Fraction(-1, 4), Fraction(3, 4)),
            "H": (Fraction(0), Fraction(1, 2)),
            "A": (Fraction(0), Fraction(1, 2)),
        })

    def test_all_23_coefficient_bounds_fit_11_bits(self):
        self.assertEqual(len(self.intervals), 23)
        self.assertEqual(self.intervals, EXPECTED_INTERVALS)
        for name, (lower, upper) in self.intervals.items():
            with self.subTest(coefficient=name):
                self.assertGreaterEqual(lower, -(1 << 10))
                self.assertLess(upper, 1 << 10)

    def test_shared_horizontal_service_raw_inputs_fit_10_bits(self):
        # H2 consumes V1-LL; H3 consumes V2-LL; H4 consumes V3-LL;
        # H5 consumes H4-L. Detail outputs still need 11 bits.
        for name in ("V1-LL", "V2-LL", "V3-LL", "H4-L"):
            with self.subTest(raw_stream=name):
                lower, upper = self.intervals[name]
                self.assertGreaterEqual(lower, -(1 << 9))
                self.assertLess(upper, 1 << 9)

    def test_first_stage_widths_remain_sufficient(self):
        for name, width in (("H1-L", 9), ("H1-H", 9), ("V1-LL", 10),
                            ("V1-LH", 10), ("V1-HL", 10), ("V1-HH", 10)):
            with self.subTest(coefficient=name, width=width):
                lower, upper = self.intervals[name]
                self.assertGreaterEqual(lower, -(1 << (width - 1)))
                self.assertLess(upper, 1 << (width - 1))


if __name__ == "__main__":
    unittest.main()
