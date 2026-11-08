#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = (
    REPO_ROOT / "rtl" / "lrcca" / "centroid_reciprocal_segments_q17.bin"
)
SEGMENT_COUNT = 1 << 8
RECIPROCAL_BITS = 17
STORED_BASE_BITS = 14
STORED_DROP_BITS = 6


def endpoint_q17(index: int) -> int:
    if not 0 <= index <= SEGMENT_COUNT:
        raise ValueError(f"endpoint index outside [0, {SEGMENT_COUNT}]: {index}")
    denominator = SEGMENT_COUNT + index
    rounded_q17 = (
        ((1 << RECIPROCAL_BITS) * SEGMENT_COUNT)
        + (denominator // 2)
    ) // denominator
    return min(rounded_q17, (1 << RECIPROCAL_BITS) - 1)


def segment_word(index: int) -> int:
    if not 0 <= index < SEGMENT_COUNT:
        raise ValueError(f"segment index outside [0, {SEGMENT_COUNT}): {index}")
    base = endpoint_q17(index)
    drop = base - endpoint_q17(index + 1)
    expected_base_high = (
        3 if index < 37 else 2 if index < 86 else 1 if index < 154 else 0
    )
    expected_drop_high = (
        7 if index < 18 else
        6 if index < 40 else
        5 if index < 68 else
        4 if index < 107 else
        3 if index < 163 else
        2
    )
    if (
        base >> 16 != 1
        or ((base >> STORED_BASE_BITS) & 0b11) != expected_base_high
    ):
        raise ValueError(
            f"segment {index} base does not match its implicit high bits: {base}"
        )
    if drop >> STORED_DROP_BITS != expected_drop_high:
        raise ValueError(f"segment {index} drop does not match its implicit high bits: {drop}")
    return (
        ((base & ((1 << STORED_BASE_BITS) - 1)) << STORED_DROP_BITS)
        | (drop & ((1 << STORED_DROP_BITS) - 1))
    )


def rendered_table() -> str:
    word_bits = STORED_BASE_BITS + STORED_DROP_BITS
    return "".join(
        f"{segment_word(index):0{word_bits}b}\n" for index in range(SEGMENT_COUNT)
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate the centroid piecewise-linear Q17 reciprocal coefficients",
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
