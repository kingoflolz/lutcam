# TagDetect architecture

TagDetect finds nested, tag-like connected components in an 8-bit monochrome
raster. It thresholds the image at three spatial scales, places those masks in
one deliberately framed composite raster, and runs complementary foreground
and background connected-component analyses over that raster. Each reported
outer component carries a bounded list of inner-component centroids.

The grouped descriptor boundary is consumer-neutral: TagDetect reports
geometry, mass ranking, validity, and group metadata, but does not assign a
tag family or convert coordinates for a downstream decoder.

`WaveletTagDetect` uses `AdaptiveThresholdFrontend` to threshold the RAW, LL2
and LL4 outputs of the shared LWC wavelet frontend. The frontend applies local
RAW thresholds, LL2/LL4 thresholds, fixed moment weights and RAW connectivity
repair, then feeds `HierarchicalTwoPPC`. The Q5 centroid stream and native Q4
centroid-group descriptors carry the same detected features.

## Production dataflow

```text
two 8-bit camera pixels/clock
            |
            +-------------------------- raw samples
            |
    shared LWC wavelet frontend ------- LL2 and LL4 low-pass taps
            |
  AdaptiveThresholdFrontend
    - local thresholds and RAW connectivity repair
    - fixed analytic moment weights
    - scale framing and row replication
    - 1696 x 807 composite raster
            |
       WaveletTagDetect
    - recover moment weights from inverted ROM bytes
    - enforce the CCA row-drain interval
            |
      HierarchicalTwoPPC
    - foreground CCA with hole fill
    - complementary-background CCA with moments
    - centroid compression and bounded child lists
            |
 generic centroid groups and Q5 compatibility tuples
```

The production 1280x800 geometry is:

| Region | Source plane | Logical size | Composite width | Vertical placement |
| --- | --- | ---: | ---: | --- |
| Stage 0 | camera | 1280x800 | 1280 | starts at row 0, no replication |
| Stage 2 | LL2 | 320x200 | 328 | starts at row 27, each row shown four times |
| Stage 4 | LL4 | 80x50 | 88 | starts at row 127, each row shown sixteen times |

The eight extra pixels on each coarse region are framing columns. The three
regions concatenate to 1696 pixels, or 848 two-pixel transfers. Rows of a
coarse plane that cannot be placed inside the 800 source rows are appended as
tail rows, making the hierarchy 807 rows high.

## Why the raster is framed

Each active scale region is surrounded by a one-pixel background border and a
one-pixel foreground fence, with neutral spacing between the fence and source
mask. This creates explicit topological boundaries between scales and between
the source data and the outside of the composite raster. It lets one ordinary
streaming CCA backend process all three scales without scale-specific reset or
flush operations.

Replicated coarse rows participate in topology so coarse objects remain
connected across the composite display. They do not contribute intensity or
moment samples: `feature_valid` is asserted only for the first copy of a
logical coarse row. The supplied feature Y coordinate is the logical scale Y,
not the replicated composite row number.

## Output groups

### Compatibility centroid stream

A centroid tuple is present when `inner_fv_valid` is high. The public word is
packed as:

```text
{ y_q5, x_q5, centroid_valid }
```

For the production geometry, X and Y each occupy 16 bits and the complete word
is 33 bits. Q5 values have five fractional bits. `hier_inner_end` accompanies
the last valid tuple belonging to the current outer component; it is a group
delimiter, not a frame-done signal. There is no output backpressure, so every
valid tuple must be captured.

An invalid centroid represents a zero accumulated feature mass. Its coordinate
fields are zero. The internal hierarchy computes Q4 coordinates and the top
level appends a zero low bit to express the same value in the public Q5 format.

### Generic descriptor stream

The production wavelet path also exposes each retained centroid at its native
Q4 precision together with its full unsigned Q5.3 logarithmic mass. Group
metadata identifies the final descriptor. Like the
compatibility stream, this boundary is non-backpressured.

The descriptors remain in the hierarchy's composite coordinate system: X
selects a framed scale region, while Y is already expressed in that region's
logical source rows. TagDetect deliberately does not classify a descriptor as
Raw, LL2, or LL4, remove fence offsets, or rescale it into another coordinate
system. Those choices belong to the integration that consumes the generic
group.

The production wavelet hierarchy retains the nine entries with greatest full
log mass and emits them in ascending mass order. This policy affects both
output views when a component has more than nine children. No truncation flag
is stored or emitted: downstream consumers process the retained descriptors
normally. This ranking policy is independent of any particular decoder.

To project a composite centroid back to the camera, a consumer first identifies
its horizontal scale region. Raw X needs no offset; a coarse-region X has its
four-pixel left frame removed before applying:

```text
camera_x = logical_x * scale
camera_y = logical_y * scale + (4 if scale == 16 else 0)
```

where `scale` is 1, 4, or 16. The centroid Y value is already in the logical
coordinate system of its scale. This conversion is a consumer-side operation,
not part of TagDetect.

## Throughput and scheduling

At the canonical cadence, 640 active pair clocks and 256 blank clocks form one
896-clock row epoch. `AdaptiveThresholdFrontend` emits 848 composite
pairs and leaves time for `HierarchicalTwoPPC` to drain its ten clocks of
row-local state. That recovery is not a multi-thousand-cycle line gap.

The OV9281 integration is slower and elastic: RAW10 conversion averages 80
million pair beats/s during active payload, while the core runs at 120 MHz.
The current camera mode has 910 byte clocks, or 1,092 core clocks, per row.
The 1,042-byte-clock test fixture has approximately
1,250 core clocks per row. The
formatter's epoch counter saturates after 896 clocks and starts the next row
when its source banks are ready; it does not wrap and impose another epoch.
Seven composite tail rows are emitted after the 800 camera rows, so a new frame
must still wait until the formatter and hierarchy have drained.

The frontend has no camera-side ready signal. Valid beats may contain bubbles,
but violating raster order, starting another frame too early, or allowing the
hierarchy to fall behind raises sticky protocol or overrun diagnostics.

## Configuration boundaries

The frontend is fixed to 1280×800, 8-bit camera samples, 14-bit wavelet taps,
equality-as-white comparisons and 27/127 coarse-scale row offsets. Every
application build uses the same algorithm and parameters.

The most important capacity parameters belong to the backend:

- CCA table depth bounds provisional runs within one row.
- Child-arena entry count bounds simultaneously live list nodes.
- Command FIFO depth absorbs the difference between one-pair CCA issue and
  multi-cycle list updates.
- Maximum child-list length bounds output work and intentionally truncates
  low-ranked inner features.

The production combined top uses outer/inner CCA depths of 848/848, 511 child
arena entries, a 512-bundle command queue, and lists of at most nine features.
Each CCA table covers the maximum of 848 runs in a 1696-pixel composite row.

## Submodule pages

- [Adaptive threshold frontend](adaptive-threshold-frontend.md)
- [Hierarchical CCA](hierarchical-cca.md)
- [Two-pixel CCA pipeline](two-ppc.md)
- [Child feature arena](child-arena.md)
- [Centroid compressor](centroid-compressor.md)
- [Elaboration utility package](utility-package.md)

## Verification

The Rust models independently build the three masks, composite raster,
connected components, child lists, and compressed centroids. RTL parity tests
exercise each backend block separately and the complete production path. The
combined test also hashes all 684,336 composite pairs, checks the exact grouped
centroid and generic descriptor streams, verifies that the production CCA
tables do not overflow, and runs LWC bit-exactly beside TagDetect at both the
canonical cadence and the nominal OV9281 100 MHz byte-clock/120 MHz core-clock
cadence.

Run the principal regressions from the repository root with:

```bash
cargo test -p lrcca-rtl-tests --test rtl_adaptive_threshold
cargo test -p lrcca-rtl-tests --test rtl_2ppc
cargo test -p lrcca-rtl-tests --test rtl_child_arena
cargo test -p lrcca-rtl-tests --test rtl_hierarchical
cargo test -p lftag-rtl-tests --test rtl_tagdetect_lwc
```

`rtl/lrcca/sources.f` is the authoritative ordered production closure used by
Rust/Marlin, Verilator lint, and synthesis. The centroid compressor's `.bin`
files are required ROM contents rather than compilation units, so they do not
appear in the manifest.

Run warning-fatal ordinary and synthesis-mode closure lint from the repository
gateware directory with:

```bash
verilator --lint-only -sv -Wall -Wpedantic -Irtl/lrcca \
  -f rtl/lrcca/sources.f --top-module WaveletTagDetect
verilator --lint-only -sv -DSYNTHESIS -Wall -Wpedantic -Irtl/lrcca \
  -f rtl/lrcca/sources.f --top-module WaveletTagDetect
```
