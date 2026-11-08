# LFTag homogeneous hypothesis decoder

The decoder tests all sixteen values of the two unknown bottom-corner symbols
against five non-corner weighted centroids. It returns the first minimum
score, the selected hypothesis, the 14-bit ID, and the
winning point association.

## Hypothesis search

The four selected corners define projectively equivalent `(U,V,W)` values for
each non-corner point. All five values use one candidate-wide power-of-two
scale. Each hypothesis then applies a small canonical transform without ever
dividing by `W`; cell boundaries, symbols, and residuals are evaluated by
signed homogeneous comparisons.

One 128-by-20 true-dual-port store holds both the sixteen coefficient sets and
the five homogeneous points. Unused addresses also retain short-lived geometry
setup values, replacing wide candidate-lifetime registers without adding RAM.
Its width maps to two RAM10s. Coefficients are loaded once for each hypothesis,
while both ports prefetch point words during the serialized X/Y response work.

The arithmetic allocation uses one shared cross-product DSP, two shared
homogeneous-product DSPs, six fixed projection DSPs, and one residual-square
accumulator. Two more DSP blocks implement the fixed signed `W/12`
residual-offset MAC. This split keeps projection throughput high without
duplicating the setup datapaths. Operand selection happens in LUT fabric;
every DSP performs genuine signed numeric arithmetic and none is used as a
selector.

## Cell assignment and ID

Projected coordinates are classified against the three-by-three cell
boundaries. Every point must land inside the legal grid and each cell may be
used only once. The two selected bottom corners occupy fixed cells; the other
seven regions provide two displacement bits each, forming the 14-bit ID.

The ranking score is the saturating sum of squared homogeneous errors on the
candidate-wide geometry scale. Since the corner cross products are normalized
once per candidate, every hypothesis for that candidate is directly
comparable without a reciprocal, leading-one detector, or variable shifter.
Illegal cell reuse, an out-of-grid coordinate, singular projection, or
arithmetic overflow makes that hypothesis ineligible.

The block-scaled public score is also the ranking score. The first strict
minimum wins; equal later scores do not replace it. This deterministic
convention is shared by the Rust fixed-point model and RTL. No runner-up
score is tracked.

## Acceptance decisions

The complete candidate is accepted only when:

- at least one hypothesis is valid;
- the selected winner's normalized pooled mean is within the Q16 limit.

Neither the absolute nor relative best-to-second score gap affects acceptance.
Equal scores retain the first valid minimum in hypothesis order. Rejecting a
winner for another check never promotes a different hypothesis. Both direct
and shared external configuration use the same rule.

Packet gap words are reserved zero. Host defaults are normalized pooled mean
`1/64` (Q16 integer 1024), mass spread 19, and minimum baseline one logical
pixel. The floating decoder uses an unweighted per-point canonical residual
and its own defaults.

For the winning hypothesis, let `N = sum(error_x² + error_y²)` and
`D = sum(W²)` over its five noncorner points. The pooled mean is `N/D`, in
squared tag-grid-pitch units. This is
a `W²`-weighted mean of individual normalized point errors, not an unweighted
average under perspective. Multiplying all homogeneous coordinates by any
common scale leaves the ratio unchanged. The raw best score remains in the
common header for diagnostics; accepted packet word 9 carries the
normalized pooled mean. See [NORMALIZED_ERROR.md](NORMALIZED_ERROR.md).

The setup cross-product MAC is reused for `D` during scoring, with full unsigned
weight magnitudes split into two signed products. The unsaturated `N` and `D`
are retained with the winning hypothesis. A 32-cycle serial divider runs during
homography calculation and emits the Q32 mean directly. Acceptance compares
its upper 16 bits with the Q16 limit; equality additionally requires zero low
bits and remainder. Equality with a floored quotient cannot accept a rational
value above the threshold. No image gradients or additional centroid refinement
are computed.

This serialized organization is sized for the candidate service target. It
removes the rectifier/divider boundary and its reciprocal state while the
16-entry descriptor FIFO continues to absorb short upstream bursts.
