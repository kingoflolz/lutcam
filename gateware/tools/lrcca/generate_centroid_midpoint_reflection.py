#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "rtl"
    / "lrcca"
    / "centroid_reciprocal_midpoint_reflection_q17.bin"
)
INTERNAL_FRAC_BITS = 21
RECIPROCAL_BITS = 17
SEGMENT_BITS = 9
SEGMENT_COUNT = 1 << SEGMENT_BITS
RESIDUAL_BITS = INTERNAL_FRAC_BITS - SEGMENT_BITS
REFLECTION_DISCARD_BITS = 3
COEFFICIENT_BITS = 18
PRODUCT_SHIFT = 19
WORD_BITS = 20


def reflected_mantissa(normalized_fraction: int) -> int:
    if not 0 <= normalized_fraction < (1 << INTERNAL_FRAC_BITS):
        raise ValueError("normalized fraction must be an unsigned Q21 value")
    segment = normalized_fraction >> RESIDUAL_BITS
    fraction = normalized_fraction & ((1 << RESIDUAL_BITS) - 1)
    reflected_full = (
        (1 << INTERNAL_FRAC_BITS)
        + (segment << RESIDUAL_BITS)
        + ((1 << RESIDUAL_BITS) - 1)
        - fraction
    )
    return reflected_full >> REFLECTION_DISCARD_BITS


def reciprocal_from_coefficient(normalized_fraction: int, coefficient: int) -> int:
    return (coefficient * reflected_mantissa(normalized_fraction)) >> PRODUCT_SHIFT


def reflection_coefficient(index: int) -> int:
    if not 0 <= index < SEGMENT_COUNT:
        raise ValueError(f"segment index outside [0, {SEGMENT_COUNT}): {index}")

    normalized_scale = 1 << INTERNAL_FRAC_BITS
    segment_width = 1 << RESIDUAL_BITS
    left = normalized_scale + (index << RESIDUAL_BITS)
    midpoint_twice = (2 * left) + segment_width - 1

    # The reciprocal tangent at midpoint T is (2*T-N)/T^2.  The reflected
    # mantissa has already been divided by eight and the product is divided by
    # 2^19, so the nearest ideal coefficient is round(2^62/(2*T)^2).
    denominator = midpoint_twice * midpoint_twice
    nearest = ((1 << 62) + (denominator // 2)) // denominator
    target = 1 << (RECIPROCAL_BITS + INTERNAL_FRAC_BITS)
    segment_start = index << RESIDUAL_BITS

    best: tuple[int, int] | None = None
    for coefficient in (nearest, nearest + 1, nearest + 2):
        maximum_error = 0
        maximum_reciprocal = 0
        for fraction in range(segment_width):
            normalized_fraction = segment_start + fraction
            reciprocal = reciprocal_from_coefficient(
                normalized_fraction,
                coefficient,
            )
            maximum_reciprocal = max(maximum_reciprocal, reciprocal)
            denominator_low = normalized_scale + normalized_fraction
            maximum_error = max(
                maximum_error,
                abs((reciprocal * denominator_low) - target),
                abs((reciprocal * (denominator_low + 1)) - target),
            )

        if maximum_reciprocal >= (1 << RECIPROCAL_BITS):
            continue
        candidate = (maximum_error, coefficient)
        if best is None or candidate < best:
            best = candidate

    if best is None:
        raise ValueError(f"segment {index} has no bounded coefficient")
    coefficient = best[1]
    if not 0 <= coefficient < (1 << COEFFICIENT_BITS):
        raise ValueError(f"segment {index} coefficient does not fit: {coefficient}")
    return coefficient


def rendered_table() -> str:
    return "".join(
        f"{reflection_coefficient(index):0{WORD_BITS}b}\n"
        for index in range(SEGMENT_COUNT)
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate midpoint-reflection Q17 reciprocal coefficients",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail instead of writing when the checked-in table differs",
    )
    args = parser.parse_args()

    expected = rendered_table()
    if args.check:
        if not args.output.is_file() or args.output.read_text() != expected:
            parser.error(f"{args.output} is not the generated reciprocal table")
        return 0

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(expected)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
