# `MonoIsp`

`MonoIsp` is the single production RTL module for the monochrome ISP. It
accepts adjacent pairs of 10-bit raw pixels and emits pairs of 8-bit pixels
without storing a row or frame. The default raster is 1280x800; width must be
positive and even, and height must be positive.

## Architecture

Every accepted pair follows one fixed path:

```text
10-bit pair -> sparse defect correction -> histogram tap -> tone LUT -> 8-bit pair
                                                 |              |
                                            optional update   optional bypass
```

The order is significant. Corrected values feed both the exposure statistics
and output mapping, while the histogram remains in the 10-bit domain.
Black-level subtraction is folded into the programmed tone LUT instead of
occupying a separate arithmetic stage.

The module has three interfaces:

- a ready/valid pixel-pair stream with start-of-frame and end-of-line markers;
- Wishbone control plus tone/defect memory programming; and
- a backpressured stream that dumps the histogram.

Raster position advances only when a pair is accepted. A one-entry elastic
output stage holds the complete result stable under backpressure. With a ready
consumer and prefetched defect state, the steady-state rate is one pair per
clock, or two pixels per clock.

## Sparse defect correction

The defect RAM stores 512 cumulative 20-bit words. Bits 19:2 are a delta added
to the current pair address; bits 1:0 are the defect mask. Decoding begins at
address zero. A zero mask advances the address without creating a defect, so it
acts as a continuation for gaps larger than the 18-bit delta.

The configured count is the number of encoded words, including continuation
words, not necessarily the number of defective pairs. Nonzero descriptors must
be strictly ordered and remain inside the frame. The Rust pack/unpack helpers
generate and validate this representation.

Correction uses only the current pair and one sample of left-neighbor history:

| Mask | Replacement |
| ---: | --- |
| `01` | Replace lane 0 with lane 1 |
| `10` | Replace lane 1 with lane 0 |
| `11` | Replace both lanes with the previous corrected pair's right pixel |

A `11` mask at row start produces zeros because there is no left neighbor.
This policy handles sparse hot/cold defects with no line buffer, trading a more
sophisticated spatial interpolation for minimal memory.

A two-descriptor queue hides synchronous RAM latency and permits consecutive
defects at full rate. Readiness uses the earliest buffered descriptor, or the
strict lower bound established by an already decoded cumulative address when
the queue is empty. It does not wait for a second descriptor merely to
classify a pair whose correction is already known. Canonically packed tables
therefore introduce no in-frame bubbles with a ready output consumer,
including a continuation between dense runs of defects. A continuation's
address proves the intervening pixels are clear while the next descriptor
is fetched; the comparison is strict because a following descriptor may
have delta zero at that address.

The queue is refilled at each frame boundary, which can briefly pause input
between frames; it does not restart at line boundaries. General noncanonical
tables containing redundant zero-mask chains remain supported through
ready/valid backpressure. The production camera loader validates canonical
encoding so its unbackpressured receiver never encounters such chains.

## Tone LUT and black level

The 1024x8 tone RAM is indexed directly by each corrected 10-bit sample. It is
a true-dual-port memory so both lanes are mapped in one clock. Runtime reads use
both ports; software reads and writes share port zero while processing is
stopped.

The Rust LUT builder combines black-level correction and gamma mapping:

```text
tone_lut[raw] = gamma_curve[saturating_sub(raw, black_level)]
```

This supports an arbitrary mapping and removes a subtract stage, at the cost of
rewriting the LUT when either curve parameter changes. Reset does not
initialize the LUT. When it is disabled, corrected pixels bypass it as
`corrected_raw[9:2]`, with no black-level correction.

## Exposure histogram

The histogram contains 512 saturating 20-bit counters and observes one value
per accepted pair:

```text
bin = (corrected_pixel0 + corrected_pixel1 + 1) >> 2
```

This rounded pair average gives a compact auto-exposure distribution. A
forwarding path handles repeated bins on consecutive clocks, so collection
does not reduce pixel throughput. Counts saturate at `0xfffff`. With CONTROL
bit 3 clear, counts accumulate across frames until explicitly cleared.

The RAM must complete a 512-clock clear before collection is first enabled. A
dump emits all 512 bin/count records in order through a backpressured stream.
The accumulating mode requires stopped, drained pixel processing before a dump.

CONTROL bit 3 selects complete-frame snapshots. Collection starts at an accepted
SOF, freezes after EOF and its final pending increment, and permits a dump while
video continues. The frozen RAM receives no pixel writes. Consuming bin 511
starts another 512-clock clear; collection resumes at the next SOF after that
clear. Frames during the dump/clear are skipped for metering, while their image
pixels continue normally. Restarting RUN discards an unread or partial snapshot
and starts a fresh clear. No second histogram RAM is needed.

## Wishbone programming model

Wishbone accesses are 32-bit aligned, full-word transactions. Illegal,
misaligned, partial, or state-forbidden accesses return ERR. The compact map is:

| Address range | Region | Purpose |
| --- | --- | --- |
| `0x0000` | ID | Constant `0x4d495350` (`"MISP"`) |
| `0x0004` | CONTROL | Bit 0 RUN; bit 1 LUT; bit 2 histogram; bit 3 frame snapshots |
| `0x0008` | STATUS | Run/pipeline, defect-ready, histogram, and error state |
| `0x000c` | DEFECT_COUNT | Number of active encoded words, 0..512 |
| `0x0010` | COMMAND | Start histogram clear or clear protocol error |
| `0x1000`..`0x1ffc` | Tone LUT | 1024 word-spaced 8-bit entries |
| `0x2000`..`0x27fc` | Defect table | 512 word-spaced 20-bit entries |

Tone and defect memories are reprogrammed only while stopped because their
ports are shared with runtime logic. Tone access also waits for a stalled
output to drain. The histogram is intentionally not Wishbone-mapped: COMMAND
clears it and the dedicated stream dumps it.

Feature configuration is latched when CONTROL is written while stopped. Once
running, the only legal CONTROL write is zero, so changes follow a
stop/reconfigure/start sequence. Starting with histogram collection enabled is
rejected until its clear has completed. Software also waits for defect-ready
before supplying pixels.

A typical lifecycle is:

1. Stop, program the tone LUT and cumulative defect table, and set its word
   count.
2. Clear the histogram if exposure collection is required.
3. Enable the selected features and RUN; wait for defect-ready.
4. Stream one or more complete frames.
5. Stop, drain the pixel pipeline, and optionally dump the histogram.

Omitting another clear intentionally accumulates statistics across frames in
accumulating mode. The camera supervisor uses CONTROL=13: RAW/4 bypass and
complete-frame histogram snapshots.

## State, invariants, and tradeoffs

Start-of-frame must mark exactly the first accepted pair, and end-of-line must
mark exactly the last pair of each configured row. Marker faults and invalid
defect ordering/addressing set a sticky protocol error and stop further input.
Software stops, repairs the source or table, clears the diagnostic, and
restarts. Wishbone access errors are separate from this stream diagnostic.

Reset stops processing, selects zero defect words, clears control/diagnostic
state, and marks the histogram uninitialized. It deliberately leaves every RAM
uninitialized.

The block-memory footprint is:

| Memory | Logical size | Role |
| --- | ---: | --- |
| Tone LUT | 8,192 bits | Two raw-to-output lookups per clock |
| Defect table | 10,240 bits | Sparse cumulative descriptors |
| Histogram | 10,240 bits | Saturating exposure counts |

The 28,672-bit total is arranged to infer three RAM10-class blocks. Outside the
RAMs, retained state is limited to raster/control registers, one previous
corrected sample, two defect descriptors, histogram forwarding state, and one
output pair. Output backpressure or frame-boundary defect prefetch can pause the
stream, but LUT mapping and histogram collection do not lower its steady-state
throughput.

## Reference model and verification

The `mono-isp` Rust crate is bit-exact for pixel processing, LUT construction,
defect packing, and histogram contents. The RTL regression programs a small
raster through Wishbone and compares full frames and histogram dumps with that
model under bubbles and backpressure. Directed cases cover consecutive defects,
continuations, repeated histogram bins, control arbitration, and sticky raster
errors. Structural tests lock the three-memory budget, true-dual-port tone
topology, and warning-free lint.
