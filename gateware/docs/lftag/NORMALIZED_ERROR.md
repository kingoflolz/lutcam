# Tag-relative decoder acceptance

The decoder measures residual error relative to tag geometry. For each of the
five noncorner points, the scorer has signed errors `ex`, `ey` and a positive
projective weight `W`:

```text
N = sum(ex² + ey²)
D = sum(W²)
normalized_mean = N / D
```

The mean is in squared canonical grid-pitch units. Its square root is an RMS
2D distance in the canonical tag plane, weighted by `W²`. Common homogeneous
scaling cancels before quantization; fixed-point rounding limits invariance
across image scales. The metric measures noncorner data points against the
corner-derived fixed-point model, including its `21/256` symbol-offset
approximation. It is not image-pixel RMSE or an independent pose-accuracy bound.

One unsigned 16-bit setting, `mean_residual_limit_q16`, bounds the ratio in
units of `1/65536` of a squared grid pitch. Its default is 1024 (`1/64`),
equivalent to one-eighth of a grid pitch RMS. Zero permits only exact zero
residual; 65535 permits all legal scorer ratios. Mass and minimum-baseline
checks also apply. Ranking uses the raw score, and rejection does not promote
a different hypothesis. See the [decoder contract](DECODER.md).

## Arithmetic

The setup cross-product MAC accumulates `D` during hypothesis scoring. Each
positive signed-19 weight is squared using two exact signed-19 by signed-18
products. The winning `N` retains its unsaturated 38-bit sum independently of
the saturated 34-bit ranking score; `D` has 39 bits.

The localizer computes `floor((N << 32) / D)` in 32 restoring steps alongside
homography processing. A 39-bit remainder and one 40-bit subtraction implement
the division. Legal inputs satisfy `0 <= N < D < 2^39`.

Acceptance compares the quotient's upper 16 bits with the Q16 limit. Equality
also requires zero lower quotient bits and zero remainder, making the test
exactly `(N << 16) <= limit * D`. Portable and native-DSP paths use the same
logic.

## Configuration and output

The Q16 limit occupies camera offset `0x94`. Offsets `0x98`, `0xa4`, and `0xa8`
are reserved. Calibration files store intrinsics; host tools derive detector
settings from the current defaults.

Accepted poses use the [22-word packet layout](LOCALIZATION.md#packet-format).
Word 9 contains `floor(normalized_mean * 2^32)`. The common header contains the
raw best score; its gap words are reserved zero. The status upper nibble and
outer transport header byte are also reserved zero.

The viewer ranks overlapping detections by normalized residual, then scale and
raw score. Pose conditioning is checked from the transmitted rotation columns.

## Verification

The Rust fixed model and RTL tests cover threshold sweeps, exact Q16 boundaries,
remainder-only rejection, zero and maximum limits, and full-width divisions.
Configuration tests cover direct and banked paths, both camera owners, stalled
reads, resets, byte enables, and reserved register words. See the
[RTL test layout](../../tests/README.md).
