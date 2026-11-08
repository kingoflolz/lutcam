# Wavelet frontend

The LWC frontend converts the fixed 1280x800, two-pixel camera stream into the
twelve coefficient bands consumed by the codec. It also exposes the LL2 and
LL4 low-pass planes used by image-analysis logic, allowing those consumers to
share the transform rather than build separate downscalers and row stores.

The frontend is deliberately streaming. It stores transform state and a small
number of complete intermediate rows, but never stores the input frame.

## Transform structure

The transform is a reversible integer CDF 5/3 decomposition with five
horizontal and three vertical levels:

```text
two centered pixels per active clock
             |
       first H/V level
             |
      shared deeper levels
             +---------------------> LL2 and LL4 analysis planes
             |
             +---------------------> twelve codec coefficient bands
```

The first level has dedicated datapaths because it operates at the full pixel
rate. Deeper horizontal levels operate on progressively shorter rows and share
one row-service datapath. Vertical work is expressed as explicit per-column
jobs backed by compact state RAMs. This arrangement trades short, predictable
bursts of intermediate work for a substantial reduction in duplicated
arithmetic.

The resulting band layout is:

```text
0 LL5   1 H5    2 H4
3 LH3   4 HL3   5 HH3
6 LH2   7 HL2   8 HH2
9 LH1  10 HL1  11 HH1
```

Every band is emitted in row-major order. Different bands may produce events
on the same clock, which is why the codec backend begins with independently
valid coefficient lanes and per-band queues.

## Internal modules

### `forward_wavelet_transform`

This is the stateful transform coordinator. It tracks the accepted raster,
assembles intermediate rows, schedules horizontal and vertical work, maps
results onto the twelve band lanes, and drains the bottom boundary after the
last input row. It also creates the LL2, H4-low, and LL4 analysis streams.

Input has no ready signal, so the production raster cadence is part of the
design contract. A new frame clears frame-local schedulers; completion occurs
only after all boundary jobs and buffered rows have drained, not with the last
input pixel.

### Horizontal lifting blocks

`horizontal_lift_stage` implements one complete CDF 5/3 row with an initiation
interval of one sample pair per clock. One retained pair provides right-side
lookahead, while explicit first/last handling duplicates edge samples for the
reversible boundary rule.

`first_horizontal_lift` specializes that primitive for centered eight-bit
camera pixels. Its narrower input arithmetic relies on proven first-level
ranges and sign-extends the result into the common coefficient width.

`horizontal_lift_row_service` shares one wider lift stage across H2 through
H5. Intermediate rows are built in RAM, placed into a small job queue, and
replayed through the shared datapath. Alternating H2 banks permit construction
of a new row while an older row is being serviced; smaller lower-level regions
reuse the remainder of the memory. A single pending write absorbs the one
write-port conflict permitted by the fixed schedule.

### Vertical lifting blocks

`vertical_lift_scheduler` is the common RAM-backed column engine. Callers issue
even, odd, advance, and tail jobs for a column context. The RAM retains only
the samples and previous high value needed to finish the next low/high pair.
The pipelined scheduler can accept one independent job each clock, provided a
caller does not violate its same-address dependency interval.

`first_vertical_lift` specializes the scheduler for the two full-rate streams
produced by the first horizontal level. As with the first horizontal wrapper,
stored widths are narrowed to proven bounds and results return in the common
coefficient width. Its two-lane state already occupies six native RAM blocks
whether represented as one wide record or as separate even, odd, and previous-
high fields. The implementation uses the separate fields so each lifting job
writes only the state it changes, eliminating a wide read-modify-write mux
without adding memory. All three fields are read together and reassembled on
the original synchronous-read boundary, so pipeline timing is unchanged.
Deeper vertical levels share another scheduler through separate logical
contexts; their geometry favors the original packed state record.

### Registered lifting arithmetic

The prediction subtraction and final update addition use
`LwcRegisteredAddSub`. For the production widths, each operation occupies one
native Titanium DSP output register, replacing its original enabled fabric
register without adding a pipeline stage. Five arithmetic lanes use ten DSPs
per camera. The helper preserves each register's enable and asynchronous reset;
`out_high` still holds the previous emitted high value used by horizontal
update feedback.

Prediction still sums both even samples at full width before its arithmetic
right shift. Update still adds both high values and two before shifting by two.
Only the final add/subtract and destination register move to the DSP; results
retain the original low-bit wrap behavior. Normal simulation uses portable
arithmetic, while `SYNTHESIS` selects the native model for dedicated tests.
Efinity uses the documented `synthesis translate_off/on` directives to select
the native hardware path without requiring a project-wide macro. Widths above
18 retain portable fabric arithmetic.

The isolated 1280x800 hierarchy probe reduced packing from 1,895 to 1,782 XLRs
per camera, with ten additional DSPs and the same fifteen RAM blocks. These
are standalone measurements; combined-chip resource and timing qualification
remain part of the normal release build. The native regression
`tests/lutcam/test_lifting_dsp.py` checks signed extrema, wrapping, valid bubbles,
held outputs, row edges, and asynchronous resets against the retained fabric
reference. Existing Rust transform tests validate complete-frame coefficients
and scheduling.

### Shared wrapper and row averaging

`unified_wavelet_frontend` is a structural system wrapper. It exposes the
original coefficient lanes for LWC alongside LL2 and LL4 without adding state
or latency. These outputs are independent event streams; consumers must not
assume that low-pass events align with a particular codec band.

LL4 is an 80x50 approximation formed by averaging adjacent rows of the 80x100
H4-low stream:

```text
average = (even_row + odd_row + 1) >>> 1
```

The production transform stores the even H4-low row in unused addresses of an
existing local-row RAM, avoiding another block memory or a separate averaging
stage.

## Memory and scheduling decisions

The frontend's main area-saving choices are:

- share one horizontal datapath across the four deeper levels;
- time-multiplex deeper vertical work through explicit contexts;
- split the full-rate vertical state into independently written fields without
  increasing its native RAM count;
- pack unrelated intermediate rows into non-overlapping regions of the same
  local-row memory;
- reuse that memory's spare tail for LL4 row averaging; and
- expose bursty band events directly instead of normalizing them through a
  large coefficient reorder buffer.

These choices make the fixed camera cadence and memory-port schedule part of
correctness. Intermediate gearboxes, job queues, and alternating row banks are
sized for the production geometry rather than arbitrary blanking patterns.

### Register reuse and gearbox capacity proof

The H2 and H3 adjacent-pair gearboxes retain only the even coefficient. Their
row tag is taken from the odd coefficient that closes the pair. For either
stream, let `E(t)` be an accepted even-column event, `D(t)` an accepted
odd-column event, and `O(t)` the retained-value occupancy after clock `t`.
The row-major producer guarantees the per-stream order `E,D,E,D,...`; bubbles
and events for other vertical contexts may appear between those events but do
not change that order. Therefore

```text
O(t+1) = O(t) + E(t) - D(t),  O(0)=0
```

and every prefix has `0 <= sum(E)-sum(D) <= 1`. An even arrival is possible
only at occupancy zero, while the next event at occupancy one is the matching
odd service. The capacity inequality is consequently `peak(O)=1 <= C=1`,
including arbitrary producer stalls and the saturated state. The matching odd
event is in the same row as its immediately preceding even event, so removing
the duplicate stored row tag cannot alter job metadata, admission, or timing.

One coefficient register is also shared by the H2 row packer and the H4/LL4
average alignment. Its H2 value is live from an even H2 output until the next
odd H2 output writes the complete RAM word. Its H4 value is live from an odd
H4 output until the synchronous average-row response one clock later. The row
service never interleaves service kinds: an arbitrary bubble can extend an H2
live interval only while the kind remains H2, and the clock after an H4 odd
output is still H4 or the directly related H5 continuation. Thus the two live
intervals are disjoint. In occupancy terms, each capture has exactly one
ordered service before another capture can occur, so `peak(O)=1 <= C=1` under
arbitrary stalls as well as continuous saturation.

These are storage-lifetime changes only. FIFO depth, job enqueue/dequeue
events, RAM ports and access clocks, ready/valid behavior, and all service
bounds are unchanged.

## Boundary handling and diagnostics

Horizontal lifting duplicates the edge samples needed by the CDF 5/3 rule.
Vertical lifting uses explicit tail jobs after the last real row. The transform
therefore continues producing coefficient events during frame drain.

Sticky diagnostics detect malformed frame starts, row-service overflow,
unexpected memory-port overlap, and invalid scheduler use. Simulation checks
also protect the narrowed arithmetic ranges, row gearbox occupancy, bank
ownership, and the placement of the integrated H4 averaging row. These are
design invariants rather than runtime recovery mechanisms.
