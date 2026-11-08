# LFTag pose localization

The shared production backend fits a homography to all nine measured centroids
only after the winning decode passes its residual threshold. It then emits the
existing object-to-camera rotation and translation. No image pixels, gradients,
or full-frame storage are needed after TagDetect produces the centroid group.

## Nine-point homography

The two anchor locations are `(-1,-1)` and `(1,-1)`. The seven data locations
are their grid-cell centers plus the decoded `±1/12` offsets on X and Y. For
observed normalized coordinates `(u,v)` and tag coordinates `(X,Y)`, each point
contributes two rows:

```text
[X Y 1 0 0 0 -uX -uY] h = u
[0 0 0 X Y 1 -vX -vY] h = v
```

The backend forms `AᵀA` and `Aᵀb` and solves the eight unknowns with `h33=1`.
Each observed image axis is centered and scaled by a power of two before this
step, keeping small or off-center tags numerically useful in signed 20-bit Q2.17.
The final conversion undoes that normalization and applies camera intrinsics:
`Hcamera = K⁻¹ T⁻¹ Hnormalized`. RAW, LL2 and LL4 scale factors, including LL4's
four-pixel vertical offset, are applied during this conversion.

The solve uses the repeated structure of the normal equations. With
`W=[X,Y,1]` and `P=[X,Y]`, it forms `S=WᵀW`, `Cu=Wᵀdiag(u)P`,
`Cv=Wᵀdiag(v)P`, `bu=Wᵀu`, `bv=Wᵀv`, `D=Pᵀdiag(u²+v²)P`, and
`r=Pᵀ(u²+v²)`. All these sums share a factor of 1/8 for numerical headroom.
It inverts the common 3-by-3 `S`, then solves only the perspective terms:

```text
Au = S⁻¹ Cu; Av = S⁻¹ Cv; zu = S⁻¹ bu; zv = S⁻¹ bv
T = D - Cuᵀ Au - Cvᵀ Av
T [g,h]ᵀ = Cuᵀ zu + Cvᵀ zv - r
au = zu + Au [g,h]ᵀ
av = zv + Av [g,h]ᵀ
```

This is the full linear least-squares system, including both perspective
terms. The pipeline issues one dot-product term per clock. Cofactors and two
20-step reciprocals replace serial eight-pivot elimination. Calibration
snapshots store inverse focal lengths in Q25 and principal points in Q4;
the calibrated output remains signed 18-bit Q16.

The calibrated matrix is narrowed to signed 18-bit Q16 for the unchanged pose
leaf. A singular determinant, arithmetic overflow, out-of-range matrix, or invalid
pose rejects the result. This linear fit is not an iterative reprojection-error
minimizer. The published quality field remains the pre-fit decoder residual.
The standalone four-corner reference is documented in
[PRODUCTION_INITIAL_HOMOGRAPHY.md](PRODUCTION_INITIAL_HOMOGRAPHY.md); it is not
instantiated by the shared production backend.

## Direct pose decomposition

Let `b1`, `b2`, and `b3` be the columns of the calibrated common-scale
homography. The hardware derives the reciprocal RMS norm `lambda` from the
first two columns, then computes:

```text
r1 = lambda * b1
r2 = lambda * b2
r3 = cross(r1, r2)
t  = lambda * (printed_tag_width / 4) * b3
```

A 512 by 18 reciprocal-square-root table replaces a general square root and
divide path. One DSP feedback lane serializes norm, conditioning, rotation,
cross-product, and translation arithmetic because pose recovery is performed
once per candidate, not once per pixel. Rotation is emitted row-major in
signed Q29 and translation in signed Q16. Translation uses the same physical
unit as the configured printed-tag width. The weak-perspective check rereads
one homography term through the existing result RAM instead of retaining its
36-bit square for the lifetime of the norm scan.

The integer Q32 norm radicand must satisfy `2^18 < r < 2^36`. Normalization
uses powers of four, and the ROM contains 256 midpoint bins for each of the
two possible leading-bit parities. Both parities are valid. The resulting
Q10 reciprocal norm differs from the ideal by no more than
`ideal / 1024 + 1` integer count, including serial scaling truncation.
The service returns every valid result in 12 clocks after acceptance and
holds it stable under output backpressure.

The reciprocal-square-root result register also carries the scale while the
unit is busy; that value is not externally observable until the output state.
This lifetime reuse leaves the request/result handshake, candidate buffering,
cycle count, and table read schedule unchanged.

Pose reconstruction uses synchronous read-first result RAM. One explicit
prefetch cycle separates the final rotation-column write from the first
cross-product read, and the subtracted operand remains selected through the
complement-correction cycle. Sequential-candidate regressions verify every
rotation and translation word so a previous candidate cannot contaminate the
next orientation.

This is deliberately a compact planar decomposition. It uses one shared RMS
scale for `r1` and `r2`, and forms `r3` with a cross product. It does not run
SVD, nonlinear pose refinement, or image-gradient optimization. Decoder
residual and the status flags expose result quality to downstream software.

The conditioning warning tests whether the smaller first/second column norm
is below `7/8` of the larger. It measures the normalized Q16 rotation columns
through the same DSP lane, after the columns have been written to result RAM.
Comparing coarsely truncated homography norms before normalization loses most
of their significant bits for small tags and can incorrectly warn even when
the two column norms are equal. The normalized comparison retains about 13
fraction bits; its boundary uncertainty is below 0.0002 in the column-norm
ratio. This adds eleven accepted-candidate clocks without adding a DSP or
widening the norm registers. The shared nine-point backend takes 887 clocks
for fitting before this decomposition; the integrated regression fixtures
measure 955 clocks for fit plus pose. The frontend decodes the next candidate concurrently;
see [README.md](README.md) for the current service and diagnostic definitions.

## Packet format

All packets use 32-bit ready/valid words. A rejected candidate ends after the
common eight-word header. An accepted unversioned packet contains 22 words:

| Word | Meaning |
| ---: | --- |
| 0..7 | Common decode header |
| 8 | Localization status in bits 0..3; all higher bits emitted as zero |
| 9 | Normalized pooled squared error `sum(error²)/sum(W²)`, Q32 |
| 10..18 | Row-major rotation matrix, signed Q29 |
| 19..21 | Translation vector, signed Q16 |

The common header contains the decoded ID and candidate metadata, best
residual and rejection code. Header words 4 and 5 are reserved and emitted as zero. Status bit zero marks a
valid localization; the next three bits report weak perspective, conditioning
warning, and fixed-point saturation. Bits 4..27 are reserved and checked by
the FPGA packetizer; bits 28..31 are opaque to transport and never select or
reject a packet format. Result words remain stable while the consumer applies
backpressure. The FPGA emits no packet version and performs no version check.

The Rust `PosePacket` parser exposes the status, residual, and fixed-point pose
without further arithmetic. The crate also keeps a pure fixed-point
H-to-pose calculation as a golden model for the internal RTL datapath; it models the internal arithmetic.
