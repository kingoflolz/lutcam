# AdaptiveThresholdFrontend

`rtl/lrcca/AdaptiveThresholdFrontend.sv` is the adaptive threshold frontend for
all application builds. It consumes the 1280×800 unsigned camera raster and the
320×200 LL2 and 80×50 LL4 taps from the shared LWC wavelet transform. Each frame
supplies its own threshold references. Equality is white at all three levels.

## Thresholds

RAW uses a 13-column by 3-row LL2 window. Columns are centered on the current
LL2 sample; rows are the current row and its two predecessors. All boundaries
replicate the nearest sample. For unsigned LL2 samples, let `S` be the window
sum, `D` the current LL2 sample and `N = 39`:

```text
wide = round((35*S + 8*N*D) / (64*N)) + 20
local = round((left + D + right) / 3)
cap = round(local / 2) + 64
Traw = clamp(min(wide, cap), 40, 128)
RAW white = camera_pixel >= Traw
```

Rounding is to nearest, with ties toward positive infinity. LL2 coefficients
remain signed and unclipped during reference arithmetic; the unsigned form
above adds 128 to each coefficient. Each LL2 threshold supplies the corresponding
4×4 RAW block. `Ll2CausalBoxReference` reads the existing LL2 row memory; its
small register cache adds no image RAM. The RTL maintains a running window
sum and uses bounded reciprocal multiplication to implement the exact integer
divisions. It shares the two samples common to both phases of the local
three-tap cap and retains the completed eight-bit threshold beside the RAW
RAM read.

LL2 compares its centered coefficient against the nearest sample of a
**5-column by 3-row causal mean of LL4**. `Ll4CausalMean5x3` rounds
only after summing the complete signed window. A rolling four-column sum and
one entering column form each five-column window, with replicated edges.
Output coordinates advance with completed pairs; input coordinate and framing
checks remain active across bubbles and resets. LL4 sample centers lie at LL2
coordinates `(4*x, 4*y + 1)`. Nearest selection uses `(x+2)/4, (y+1)/4`,
with integer division, ties toward the larger coordinate and replicated edges.
The reference RAM is read alongside the LL2 pixels; each pair shares one
reference sample.

LL4 compares its centered coefficient against **zero**.

## Connectivity repair

`ThresholdConnectivity2PPC` repairs ambiguous diagonal black connections in
RAW where `Traw <= 120`. It retains the original mask sign, a three-bit residual
magnitude `min(7, round(abs(pixel - Traw) / 16))`, and a continuation bit.
For opposing black and white corners of a 2×2 cell, a black-strength product
larger than the white-strength product permits filling the intervening white
pixel. A guarded thin-diagonal continuation also permits repair.

A repaired pixel becomes black in the mask and has `feature_valid = 0`, so it
connects components without contributing mass or shifting their centroids.
Repaired values do not feed back into the retained residuals. Each 2×2 cell's
strength products are calculated once: the central cell serves both lanes,
and a boundary comparison is retained for the following pair. The packed
640×10-bit row memory uses one BRAM per camera. Ready/valid stalls preserve
both pixel data and metadata.

## Moment weights

For an unsigned pixel `p`, the fixed analytic curves are:

```text
RAW curve    = round(0.85 * round(255 * (min(255, 279-p) / 255)^5))
LL2/LL4 curve = round(255 * ((255-p) / 255)^2.5)
weight = min(31, floor((curve + 4) / 8))
```

Coarse coefficients are converted to unsigned bytes with saturation before
weight lookup. `ThresholdMomentWeightRom` stores `255 - weight` in one 512×8
ROM per camera with two read ports. `WaveletTagDetect` inverts that byte before
passing its five weight bits to the hierarchy. Moment storage and arithmetic
are sized for these five-bit samples. Framing, replicated rows and repaired pixels are
excluded from moment accumulation through `feature_valid`.

## Buffering and output

Nine tagged rows at each scale retain samples until their composite output
slots. RAW and LL2 reference reads share the LL2 row RAM. The LL4 mean filter
has two preceding rows and stores its filtered output in a tagged row buffer.
Coarse source-row counters advance with row release. Framing flags are
computed before the RAM read, so the pending output does not need to retain
and decode the full composite row coordinate again.

| Storage per camera | Logical organization | Mapped BRAMs |
| --- | --- | ---: |
| Tagged raster/reference buffers | RAW 5760×16; LL2 1440×20; LL4 and filtered LL4 each 360×20 | 15 total |
| LL4 mean preceding rows | 80×20 | 1 |
| Moment weight ROM | 512×8, two reads | 1 |
| Connectivity residual row | 640×10 | 1 |
| **Frontend total** | | **18** |

Both camera frontends use **36 BRAMs**. The shared wavelet transform, CCA,
LFTag decoder, ISP and transports are additional consumers. Physical packing
is reported by the Efinity mapped resource report.

The frontend emits a framed 1696×807 raster: RAW at X=0, LL2 at X=1284 and LL4
at X=1612. Coarse rows start at composite Y=27 and Y=127 and are replicated
four and sixteen times respectively. Only the first copy contributes moments;
`out_source_y` always identifies the logical source row. Seven tail rows finish
the coarse planes after the camera's 800 rows.

The minimum row epoch is 896 clocks. Actual output starts when the required
row tags are ready and pauses under downstream backpressure. The camera input
has no ready signal. Malformed raster order or overwritten live rows raise
sticky protocol/overrun diagnostics. A frame must drain before another is
admitted; reset clears the formatter and repair pipeline for a fresh frame.

## Reference and verification

`software/crates/lrcca/src/adaptive_threshold.rs` implements the same arithmetic
using direct scalar division and analytic weights, independently of the RTL's
RAM schedule and reciprocal multipliers. The viewer's offline calibration
extractor and `threshold-bench` use this model.

From the repository root:

```bash
cargo test --offline -p lrcca
cargo test --offline -p lrcca-rtl-tests --test rtl_adaptive_threshold
cargo test --offline -p lftag-rtl-tests --test rtl_tagdetect_lwc
python -m unittest discover -s gateware/tests/lrcca/threshold -v
cargo run --offline --release -p threshold-bench -- \
  --dataset dataset --output results
python gateware/tools/lrcca/threshold_detection_metrics.py \
  --dataset dataset/manifest.json results/adaptive.json
```

Set `LUTCAM_THRESHOLD_DATASET_DIR` to include natural-background projected-tag
frames in the RTL parity tests. `LUTCAM_DUMP_MASKS` exports the benchmark's
three masks, the RAW mask before repair, its thresholds and source taps.

The 100-image development corpus contains 2000 tags. The canonical frontend
detects **1961**, with **841** detected at two or more levels and **33** false
positives. Per-level counts are RAW **1955**, LL2 **847**, LL4 **1**. These are
development results, not an independent holdout estimate. The benchmark's
primary metric is unique correct tags at any level; multi-level agreement and
centroid error against projected theoretical centers are secondary metrics.
Across matched detections, dot-centroid RMS error is **0.227 source pixels**.
