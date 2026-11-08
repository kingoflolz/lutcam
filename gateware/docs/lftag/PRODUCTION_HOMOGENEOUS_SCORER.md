# Production homogeneous geometry and scorer

`LfTagHomogeneousGeometryScorer` combines the old projective frontend and
hypothesis search into one divide-free block. The selected corners and all
five data points remain the weighted centroids produced by TagDetect; the
optimization changes their mathematical representation, not their location.

Corner and point cross products produce one common-scale homogeneous map.
Each of the sixteen LFTag bottom-corner hypotheses is applied directly to
that map. Cell classification and symbol decisions compare signed numerators
against the projective denominator, so no Cartesian coordinate or reciprocal
is needed.

A single 128-by-20 true-dual-port store holds both point words and packed
hypothesis coefficients. Otherwise unused addresses hold short-lived map
values during setup, eliminating candidate-lifetime registers without adding
memory. Its width maps to two RAM10s. Coefficients are loaded once per
hypothesis, and point words are prefetched while the serialized X/Y response
logic is active, so the smaller memory does not cost candidate throughput.

Twelve signed arithmetic blocks are mapped to DSPs: one time-shared
cross-product lane, two time-shared homogeneous-product lanes, six fixed
projection products, one residual square-accumulator, and the two-block
decomposition of the fixed signed `W/12` residual-offset MAC. Operand
selection is ordinary fabric control, and no DSP is used as a selector. X and
Y classification share the same comparison logic on consecutive clocks.

Each consecutive X/Y residual is squared and accumulated immediately through
the DSP's registered arithmetic feedback. A 39-bit non-negative accumulator
holds all ten squares without overflow, so applying 34-bit saturation once at
hypothesis completion is bit-exact with saturating after every non-negative
addition. This raw score drives best/second selection, ambiguity and common
header reporting. Absolute residual acceptance instead uses the selected
winner's tag-relative `N/D`, where `N` is the full unsaturated squared-error sum
and `D` is the sum of its five squared projective weights. The setup cross MAC
accumulates `D` during scoring; a serial divider in the localizer normalizes
only the selected winner while homography work runs. Accepted packet word 9
reports the normalized pooled mean. See [NORMALIZED_ERROR.md](NORMALIZED_ERROR.md).
The Rust fixed model reproduces the
common-scale normalization, the `21/256` symbol offset, saturation, and strict
tie-breaking rule used by RTL.
