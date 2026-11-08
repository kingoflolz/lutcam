# Lean Wavelet Codec canonical design

This document specifies the current Lean Wavelet Codec (LWC) encoder and its
direct RTL implementation. It records the detailed fixed-profile contracts
behind the higher-level module descriptions in this directory.

The wire format and decoder arithmetic are normative in
[`bitstream_format.md`](bitstream_format.md).

## 1. Fixed profile and implementation point

| Property | Canonical value |
|---|---:|
| Input | 1280x800 unsigned q8 grayscale |
| Input transfer | two horizontally adjacent pixels |
| Active transfers per row | 640 |
| Canonical row cadence | 640 valid clocks, then 256 blank clocks |
| Transform | reversible integer CDF 5/3, 5H/3V |
| Transform output | twelve independent signed q14 event lanes |
| Coding group | sixteen coefficients, replayed as eight pairs |
| Payload | exactly 1,024,000 bits / 64,000 16-bit words |
| Decoder | scalar affine reconstruction, inverse 5/3, clipping |

The encoder contains no frame buffer and performs no inverse transform. Its
rate-distortion metric is computed from the source coefficient, fine scalar,
coarse correction, and one local residual class. It has no spatial filter,
candidate window, content classifier, threshold history, or image-specific
state. The decoder has no local or nonlocal restoration stage.

## 2. Top-level dataflow

```text
two q8 pixels/clock
        |
        v
forward_wavelet_transform
  dedicated H1 + V1
  shared H2..H5 row service
  shared V2/V3 vertical state
        | twelve independent q14 coefficient events
        v
coefficient_quantizer
  five conflict-colored scalar lanes
        | q8 + correction + residual class
        v
coefficient_pair_queues
  band stores + fixed family-schedule replay
        | one ordered ready/valid coefficient pair
        v
causal_group_encoder
  group analysis + causal choice + entropy tokens
  + 46-bit reservoir + 16-bit word packing + fixed-frame padding
        |
        v
64,000 big-endian chronological words
```

The transform produces bands in their natural completion cadence, including
simultaneous events. The queues are the only rate-matching boundary between
that non-stallable frontend and the single group encoder. Per-band completed
group pointers provide availability credits; a fixed family-schedule ROM
selects their canonical replay order. This is the group schedule consumed by
the software decoder.

## 3. External RTL contract

`codec_encoder_top` has the following ports:

| Signal | Width | Direction | Meaning |
|---|---:|---|---|
| `clk` | 1 | input | single encoder clock |
| `rst` | 1 | input | active-high reset |
| `in_sof` | 1 | input | marks the first accepted pixel pair |
| `in_valid` | 1 | input | transfers `in_px0,in_px1` when high |
| `in_px0` | 8 | input | even-x pixel of the pair |
| `in_px1` | 8 | input | odd-x pixel of the pair |
| `out_sof` | 1 | output | high with output word zero |
| `out_valid` | 1 | output | transfers `out_data` when high |
| `out_data` | 16 | output | next chronological payload bits, MSB first |
| `protocol_error` | 1 | output | sticky queue/group failure for the frame |

There is no input `ready`. The source supplies raster-order adjacent pairs,
640 accepted transfers per row and 800 rows per frame. `in_sof` is meaningful
only with `in_valid`; that accepted transfer also generates `clear_frame` for
frame-scoped control. The canonical compact-queue contract has exactly 256
invalid clocks after each active row. Changing that interval changes event
phase relative to transform/replay service and is outside this losslessness
claim unless the resulting timing trace is analyzed separately.

There is no output `ready` at the top. The internal group encoder has a
ready/valid output, but the root ties `out_ready=1`; a consumer must capture
every `out_valid` word. A new frame may start only after the previous frame
has completely drained. Accepted SOF is not a mechanism for aborting a live
transform pipeline.

The active-pixel datapath is two pixels per clock. At the 120.005 MHz
constraint that is 240.010 Mpixel/s during `in_valid`. Including the canonical
640/256 line cadence, the raster rate is
`1280/896 * 120.005 = 171.436 Mpixel/s`. These are interface rates, not a
claim that gap-free 640-clock rows are supported.

## 4. Numerical transform

The input pixel pair is centered to signed q9 by subtracting 128. One CDF 5/3
analysis step over even `E` and odd `O` samples is:

```text
H[i] = O[i] - ((E[i] + E[i+1]) >>> 1)
L[i] = E[i] + ((H[i-1] + H[i] + 2) >>> 2)
```

The nearest sample is duplicated at each boundary. Analysis performs H1,V1,
H2,V2,H3,V3,H4,H5 in low-low Mallat order. H1 uses a q9 arithmetic leaf. V1
uses q10 arithmetic with q9 raw state and q10 previous-high state. All later
external datapaths are signed q14; the shared V2/V3 state stores q13 raw
samples and q14 previous highs.

The twelve q14 event lanes are:

| ID | Source | Size | Coefficients | Groups |
|---:|---|---:|---:|---:|
| 0 | H5 low / LL5 | 40x100 | 4,000 | 250 |
| 1 | H5 high | 40x100 | 4,000 | 250 |
| 2 | H4 high | 80x100 | 8,000 | 500 |
| 3 | V3 high of H3 low | 160x100 | 16,000 | 1,000 |
| 4 | V3 low of H3 high | 160x100 | 16,000 | 1,000 |
| 5 | V3 high of H3 high | 160x100 | 16,000 | 1,000 |
| 6 | V2 high of H2 low | 320x200 | 64,000 | 4,000 |
| 7 | V2 low of H2 high | 320x200 | 64,000 | 4,000 |
| 8 | V2 high of H2 high | 320x200 | 64,000 | 4,000 |
| 9 | V1 high of H1 low | 640x400 | 256,000 | 16,000 |
| 10 | V1 low of H1 high | 640x400 | 256,000 | 16,000 |
| 11 | V1 high of H1 high | 640x400 | 256,000 | 16,000 |

The total is exactly 1,024,000 coefficients and 64,000 groups.

## 5. RTL modules

### 5.1 `codec_profile_pkg`

This stateless package owns the synthesizable fixed profile:

- fine/coarse shifts, signed adjustments, and scalar biases;
- fine fixed Rice parameters;
- nominal group charges and the two energy-normalization shifts;
- signed-debt cutoff construction;
- Huffman nibble codes and lengths;
- signed-q8 saturation and magnitude helpers; and
- Rice literal-length helpers.

All functions are fixed-width combinational logic. The package owns no clock,
register, RAM, or DSP. Decoder reconstruction tables live in Rust because the
decoder is not part of the FPGA closure.

### 5.2 `horizontal_lift_stage`

The generic interface accepts one signed `{even,odd}` pair and
`in_valid,in_first,in_last`. It emits one signed `{low,high}` pair with matching
flags. `DATA_WIDTH` selects the complete arithmetic width.

State is:

| Register state | Width |
|---|---:|
| pending even/odd pair | `2*DATA_WIDTH` |
| pending valid/first/last | 3 |
| predict even/high | `2*DATA_WIDTH` |
| predict valid/first/last | 3 |
| registered output and flags | `2*DATA_WIDTH + 3` |

The registered output high is also the preceding high for the following
update. Because launched predictions are emitted in order, it remains the
correct left neighbour across valid bubbles; a row's first update duplicates
its own high and does not consume the preceding output.

Cycle schedule for a row pair accepted at edge N is:

1. Edge N stores it in the pending/lookahead register.
2. When the next pair is present, or the pending pair is the last pair, the
   predict result enters registers. The next pair supplies the right even;
   the tail instead duplicates its own even.
3. One edge later the update result and registered high assert `out_valid`.

The first output therefore waits for one-pair lookahead and two register
edges. A gap-free row has initiation interval one after startup. The final
pair self-launches even when `in_valid` drops. The leaf has no RAM or DSP.

The production instances are H1 with `DATA_WIDTH=9` and the shared H2-H5
leaf with `DATA_WIDTH=14`.

### 5.3 `first_horizontal_lift`

This wrapper exposes signed q9 centered pixels, instantiates the q9 horizontal
leaf, and sign-extends its q9 low/high results to q14. It adds no state or
latency. Making the narrow width structural prevents the H1 arithmetic from
being synthesized as q14.

### 5.4 `vertical_lift_scheduler`

The generic scheduler accepts at most one state operation per clock:

```text
in_valid, in_op[1:0], in_state_address, in_context,
in_column, in_output_y, in_output_first, in_output_last,
in_value[LANES][DATA_WIDTH]
```

Its operations are:

| Operation | State action | Output |
|---|---|---|
| `EVEN` | write current even raw sample | none |
| `ODD` | write current odd raw sample | none |
| `ADVANCE` | use supplied next even; store it and the new high | completed low/high |
| `TAIL` | duplicate stored even as next even | final low/high |

Each lane's RAM record is packed as:

```text
{previous_high[HIGH_WIDTH], odd[RAW_WIDTH], even[RAW_WIDTH]}
```

The RAM has one synchronous read and one independent write. A request sampled
at edge N loads the RAM/read metadata stage; edge N+1 registers predict and
the old record; edge N+2 performs update, writes the revised record, and
asserts output for `ADVANCE` or `TAIL`. The request-to-output latency is two
clocks and the initiation interval is one. The static schedule separates
dependent operations on the same state address.

Production configurations are:

| Instance | Lanes | Arithmetic | Raw/high state | Depth x record | RAM10 |
|---|---:|---:|---:|---:|---:|
| V1 core | 2 | q10 | q9 / q10 | 640x56 | 6 |
| shared V2/V3 | 1 | q14 | q13 / q14 | 1024x40, 960 live | 4 |

### 5.5 `first_vertical_lift`

The wrapper narrows each q14 input lane to q10, instantiates the two-lane V1
scheduler, and sign-extends q10 outputs to q14. Its parameters are
`LANES=2`, `DATA_WIDTH=10`, `RAW_WIDTH=9`, `HIGH_WIDTH=10`, and
`STATE_DEPTH=640`. It adds no independent storage.

### 5.6 `horizontal_lift_row_service`

One q14 horizontal leaf is time-multiplexed across H2, H3, H4, and H5. H2,
H3, and H4 input interfaces carry valid, pair index, even/odd q14 data, and a
10-bit row tag. H2 and H3 also carry sink-ready. Output carries valid,
two-bit kind, first/last, q14 low/high, x, and row.

The logical `pair_memory` is 928x28:

| Logical region | Contents | Pair words |
|---|---|---:|
| 0..319 | H2 bank 0 | 320 |
| 320..639 | H2 bank 1 | 320 |
| 640..799 | H3 row | 160 |
| 800..879 | H4 row | 80 |
| sparse locations in 880..927 | H5 row | 40 |
| padding in the final 128-word region | unused | 8 |

H2 alternates construction banks so one row can be built while the preceding
row is serviced. H3 and H4 each use one busy-checked bank. Adjacent H4 low
outputs are paired and written into the H5 region; H5 is always serviced
immediately after its parent H4 row.

The physical memory has one synchronous service-read port and one construction
write port. Upper H2/H3 writes and lower H4/H5 writes are merged. A one-entry
retained-write register absorbs a two-source collision; a third obligation
before it clears is a protocol failure. Fixed address concatenations place H5
in holes in the padded lower region, avoiding a runtime base adder.

A two-entry job FIFO stores `{kind,H2_bank,row}`. The service FSM is:

```text
IDLE --pop ready row--> FEED --issue final RAM read--> DRAIN
 ^                                                      |
 +------------------ final lift output -----------------+
```

`FEED` issues one synchronous read each clock. The next clock presents the
RAM result to the horizontal leaf, which retains II=1. H2/H3 jobs wait in
`IDLE` until their downstream row handoff is free. The fixed schedule's peak
job occupancy is two. On the last H4 output, the FSM starts H5 `FEED` directly
instead of returning to the FIFO.

This block maps to three RAM10 and no DSP.

### 5.7 `forward_wavelet_transform`

The transform root fixes `FRAME_WIDTH=1280`, `FRAME_HEIGHT=800`, q14 output,
and 10-bit coordinates. It has no output ready. Its public coefficient
interface is:

```systemverilog
out_raw_valid[11:0]
out_raw_value[11:0][13:0]
```

Each asserted valid independently advances one band's row-major stream.
Several bits can assert together.

H1 is a dedicated q9 leaf. V1 processes H1 low and high in two lanes. Small
one-value gearboxes form adjacent low pairs for H2, H3, and H4. H2-H5 use the
shared row service. The lower vertical scheduler uses four address contexts:
V2 of H2-low, V2 of H2-high, V3 of H3-low, and V3 of H3-high.

The lower vertical state uses a sparse injective address map. Two 320-column
V2 contexts occupy the low 320 words in each 512-word half; the two 160-column
V3 contexts occupy unused codes in those halves. The physical organization is
1024x40 with 960 live states and no constant-offset adder chain.

A separate 512x28 local row memory maps to two RAM10:

| Addresses | Contents |
|---|---|
| 0..159 | adjacent H2 highs, two q14 values per word |
| 160..255 | unused |
| 256..415 | `{H3 high,H3 low}` for both V3 contexts |
| 416..511 | unused |

The lower scheduler input priority is tail flush, live H2-low, buffered
H2-high, then buffered H3 low/high. H2-high reads repeat each word address for
its two halves. H3 drains each column in three phases: synchronous local-row
read, V3-low submission, V3-high submission.

After the final H1 output row, the flush FSM executes:

| State | Work |
|---|---|
| V1 wait | 32 minimum wait clocks, extended until row service is idle |
| V1 tail | 640 tail operations |
| V2 input wait | 768 minimum clocks, extended until handoffs are safe |
| V2 tail | 640 operations alternating two V2 contexts |
| V3 input wait | 768 minimum clocks, extended until handoffs are safe |
| V3 tail | 320 operations alternating two V3 contexts |
| final wait | 512 minimum clocks, extended until row service is idle |

The module then pulses `frame_done`. Coding completion is independently
defined by the exact 64,000-group count.

Transform RAM is `6 + 4 + 3 + 2 = 15` RAM10.

### 5.8 `coefficient_quantizer`

This combinational frontend maps twelve q14 event lanes to signed fine q8,
one coarse-rounding correction bit, and one residual-class bit. The fixed
transform conflict graph needs only five physical scalar lanes:

| Physical lane | Mutually exclusive bands |
|---:|---|
| 0 | 6, 7, 9 |
| 1 | 8, 10 |
| 2 | 11 |
| 3 | 0, 2, 3, 4 |
| 4 | 1, 5 |

Each lane sign-extends q14 to fifteen bits, adds the selected signed
adjustment, shifts, and saturates to `-128..127`. Exact low-bit interval and
saturation predicates compute the correction and residual class without a
second quantizer or reconstructed-coefficient subtractor. The independent
arithmetic definitions are in the bitstream specification.

`out_valid` is the input event mask. Data outputs are qualified only by their
corresponding valid bit and are statically fanned from each physical lane to
its band set. There is no register latency. A conflict within a color set
sets a sticky diagnostic bit. The mapped canonical hierarchy uses 280 LUT4,
75 adders, no FF, no RAM, and no DSP; the diagnostic cone is simulation-only
at the root.

### 5.9 `coefficient_pair_queues`

This block converts the twelve coefficient event lanes into one canonical
ready/valid pair stream. Ten-bit coefficient words are written one at a time;
adjacent words are read as a twenty-bit pair. Eight pairs make one group, and
only a completed group advances the per-band credit visible to replay.

A 1,605-entry family schedule expands to the fixed 64,000-group order. The
reader waits for the expected band's credit, then reads indices zero through
seven without exposing partial groups. One response slot and one output stage
hold data and markers stable under backpressure.

The reusable leaf-module default retains twelve independent queues and uses
sixteen RAM10s including the schedule. Production selects a fixed-transform
profile with five color-banked 1024x10 payload RAMs and the schedule ROM, for
six RAM10s total. Group-aligned static regions hold 64, 192, 176, or 336 pairs
according to band. Wrap phases and a group-collision guard prevent an
out-of-contract source from overwriting an address identified as occupied.
The narrowed-contract adversarial proof covers the exact pair/half recurrence,
an exhaustive reservoir closure, the eleven-clock encoder service induction,
all 64,000 schedule prefixes, static region injectivity, RAM-port coloring,
wrap and frame-clear rules, and fail-safe behavior. It is specified in
[coefficient pair queues](coefficient_pair_queues.md).

### 5.10 `causal_group_encoder`

This module accepts eight ordered pair transfers and performs candidate
analysis, causal rate choice, packet construction, run coding, 16-bit packing,
and fixed-frame padding. Its pair input includes start/last, four-bit band,
two q8 fine values, two correction bits, and two residual-class bits.

The standalone output is `out_valid/out_ready/out_data[15:0]` with `out_sof`
and `frame_done`. Diagnostic state outputs are `packets_done`, 21-bit semantic
`packet_bits`, 16-bit committed and serialized group counts, signed 21-bit
`rate_debt`, 20-bit `extra_room`, and sticky `protocol_error`. The production
root consumes only the word stream and error; module tests observe the other
outputs directly.

#### Collection and candidate analysis

On each accepted pair the collector:

- reconstructs both coarse q8 values;
- accumulates fine and coarse masks and four Huffman-nibble lengths;
- accumulates exact fine and coarse Rice lengths;
- tracks packet-local coarse previous magnitude;
- flags either candidate if a pair exceeds 17 Rice bits; and
- accumulates direct and incremental observable energy.

The live fill state is a three-bit pair index, four-bit band, fourteen saved
mask bits for each candidate, six-bit Huffman-length sums, eight-bit saturated
Rice-prefix sums, an eleven-bit saturated direct-energy sum, a nine-bit
incremental-energy sum, a three-bit coarse previous magnitude, and two pair
overflow flags. Pair seven combines the current two lanes with those saved
fourteen lanes to form the final sixteen-bit masks.

Fine and correction data are also written to a 16x18 context RAM:

```text
address = {fill_context, pair_index[2:0]}
word = {correction1, q1[7:0], correction0, q0[7:0]}
```

Residual classes are consumed by energy analysis and are not stored in this
RAM. Two eight-word contexts allow the next group to collect while the prior
group is serialized.

For fine magnitude `m`, local coefficient `C`, and assist bit, direct energy
is factored as `m*(8*m +/- C)`. Two 5x8 products evaluate the accepted pair.
The direct sum saturates at 1,992; the incremental sum is at most `16*24=384`.
The largest selected-packet threshold is `134*12=1,608`, so even after the
largest incremental subtraction, clipping above `1,608+384` cannot alter a
decision.

A normal pair is limited to 17 bits. A longer pair forces that candidate to
its raw group form. The selected packet maximum is 135 bits: a 147-bit coarse
raw candidate is never selected because its matching 135-bit fine raw
candidate fits whenever it does and dominates by length.

#### Finalizer and rate choice

Pair seven captures completed masks, lengths, energy, band, nominal charge,
and normalization shifts. The finalizer first tests the coarse candidate,
then resolves the incremental/fine choice. Low and high Huffman mask halves
are requested on consecutive clocks from one ROM port before header service.
A selected zero is absorbed into the run counter at the decision boundary.

Rate state starts as:

```text
extra_room = 1,024,000 - 64,000 = 960,000
rate_debt = 0
```

For each group:

```text
pre_debt = rate_debt - nominal_bits[band]
coarse_cutoff = clamp(5 + (pre_debt >>> 14), 0, 8)
incremental_cutoff = clamp(6 + (pre_debt >>> 14), 0, 8)

coarse_extra = coarse_bits - 1
fine_extra = fine_bits - 1
incremental_extra = max(fine_bits-coarse_bits, 0)

if coarse fits room and passes the coarse slope:
    choose fine when it fits and either dominates by length
        or passes the incremental slope
    otherwise choose coarse
else:
    choose zero
```

Both slope tests use the bounded multiplier table
`1,2,2,3,4,6,8,12`. The mutually exclusive coarse and incremental decisions
share one fabric shift/add comparison. There is no divider and no direct
zero-to-fine edge.

The only inter-group allocation state is signed rate debt, remaining room,
and the zero-run count needed to account for the syntax. Rice previous
magnitude is packet-local and resets to zero for every group.

#### Bounded nonzero service

The pair context RAM and Rice lookup have synchronous reads. Header-time
prefetch hides both latencies, so every selected nonzero packet has this
accepted token schedule:

| Service clock | Action |
|---:|---|
| 0 | append prefix; issue context pair 0 read |
| 1 | append low mask; issue pair 0 Rice lookup and context pair 1 read |
| 2 | append high mask; start pair builder |
| 3 | append pair 0; issue pair 1 lookup and pair 2 read |
| 4-9 | append pairs 1-6 while prefetching the following pair |
| 10 | append pair 7; complete packet |

This is a hard eleven accepted-token-clock bound: three header slices plus
eight pairs, independent of coefficient magnitudes or code lengths. A
zero-length normal pair still consumes its pair clock. Raw pairs contribute
exactly sixteen bits. Finalizer clocks precede this table and remain part of
elapsed group-boundary latency.

Collection of the next group uses the opposite context and may overlap header
and pair service. Its first seven pairs can be retained while the earlier
descriptor drains; the final pair is held when committing it would overwrite
that descriptor. Context ownership therefore remains unambiguous under
downstream stalls.

#### Huffman ROM and output reservoir

The complete two-nibble table is a logical 256x18 ROM. Each entry contains
`{combined_length,left_aligned_code}`. Low and high mask bytes are read on
consecutive finalizer clocks, allowing one RAM10 read port to serve both.

Normal Rice suffixes use a 1024x21 true-dual-port table. Five packed 129-entry
regions cover magnitude 0 through 128 for Rice parameters 0 through 4. The
two read ports return both coefficient suffixes and lengths together; two
sideband sign bits restore the signed code. This table maps to three DPRAM10s
and replaces two variable unary shifters without changing packet timing.

Header tokens are at most 14 bits. Normal pair tokens are at most 17 bits;
raw pair tokens are 16 bits. A 46-bit chronological reservoir first removes
an accepted 16-bit output word, then appends one complete token. The append
position is split into a 16-bit word index and a four-bit intra-word offset,
so only a 34-bit fixed-direction window is shifted; there is no arbitrary
bit-addressed RAM.

Standalone `out_ready` backpressure can stall token acceptance. In the top it
is tied high. `token_ready` checks `count_after_output + token_count <= 46`,
so the reservoir cannot overwrite state even under a standalone stall.

After all 64,000 groups serialize, padding mode drains any partial semantic
word with zero low bits and emits zero words through physical word 63,999.
That last physical word is withheld until packet completion, preventing an
exact-budget stream from ending before its final fixed service clocks retire.

This module owns one pair-context RAM10, one Huffman RAM10, three Rice-token
DPRAM10s, and two inferred DSP24s.

### 5.11 `codec_encoder_top`

The root is structural. It instantiates the transform, quantizer, queues, and
group encoder; derives `clear_frame=in_sof&&in_valid`; converts `rst` to
active-low `rst_n` for transform leaves; ties group output ready high; and
exports queue OR group-encoder error. It owns no RAM or datapath state.

Transform and quantizer diagnostic errors feed simulation assertions. They
are not part of the synthesized public error because the fixed transform
schedule and its five-color event graph are input-contract invariants. Queue
capacity and group packet/reservoir failures remain visible at the top.

## 6. End-to-end cycle schedule

| Boundary | Latency / initiation interval | Bounded state |
|---|---|---|
| Raster input | two pixels per valid clock; 640 valid + 256 blank per row | x/y counters |
| H1 lift | one-pair lookahead, predict register, update/output register; II=1 | one pending pair; output high is reused as history |
| Vertical lift | synchronous read, predict, update/write; two-clock output latency; II=1 | one packed state RAM |
| H2-H5 service | complete-row jobs, one RAM read/pair, shared lift II=1 | two jobs + one retained write |
| Quantizer | same-cycle combinational | no payload state |
| Coefficient pairing | two 10-bit writes per logical 20-bit pair | twelve half selectors |
| Schedule replay | synchronous family token then synchronous pair reads | one token + one response + one output |
| Group collection | eight accepted pairs | masks, lengths, energies, one fill context |
| Finalizer | three coarse/fine/high-read clocks | selected state and header metadata |
| Nonzero service | three header tokens plus eight pair responses | alternate 8-pair context |
| Word packing | remove 16 bits, append one token | 46-bit reservoir |

No production RTL loop has a data-dependent iteration count. Unary Rice is
represented by a count and fixed shift, not emitted one bit per clock. The
only variable elapsed latency is ordinary ready/valid stalling or waiting for
fixed transform handoffs.

## 7. RAM10 ownership and ports

| Owner | Logical organization | Port use | RAM10 |
|---|---|---|---:|
| V1 state | 640x56 | one synchronous read + independent write | 6 |
| V2/V3 state | 1024x40, 960 live | one synchronous read + independent write | 4 |
| H2-H5 rows | 928x28 | one service read + one arbitrated construction write | 3 |
| H2/H3 handoff | 512x28 | fixed scheduled read/write | 2 |
| **transform subtotal** |  |  | **15** |
| compact coefficient banks | 5 x 1024x10 | one color-safe write + one adjacent pair read per bank | 5 |
| canonical family schedule | 1605x3 | one synchronous read | 1 |
| **queue subtotal** |  |  | **6** |
| selected contexts | 16x18 | collector write + serializer read | 1 |
| Huffman table | 256x18 | consecutive low/high reads | 1 |
| Rice token table | 1024x21 | two coefficient reads | 3 |
| **group subtotal** |  |  | **5** |
| **encoder total** | 26 RAM10-equivalent blocks |  | **26** |

Memories are word addressed. No payload RAM is reset or frame-cleared; reset
invalidates pointers, valid bits, busy flags, and pipeline metadata so stale
contents cannot be authorized for reading. The Huffman and Rice ROMs are the
only initialized memories.

## 8. DSP and arithmetic ownership

Both DSP24s are inferred in `causal_group_encoder` from ordinary SystemVerilog
multiplication:

| Products per clock | Width | Purpose |
|---:|---:|---|
| 2 | 5x8 | `m*(8m +/- C)` direct energy for two coefficients |

No vendor primitive is instantiated. Transform lifting uses adders and fixed
arithmetic shifts. Quantization uses fixed shifts and saturation. The
synthesis flow permits up to 40 multipliers but maps exactly two in this block.

## 9. Safety and overflow arguments

### 9.1 Physical payload cannot exceed one bit per pixel

Let `F=1,024,000`, `G=64,000`, and initialize `room=F-G`. Before a group is
chosen, one bit is already reserved for every unprocessed group. A selected
packet of `b` bits is admitted only when `b-1 <= room`, then subtracts `b-1`.
Therefore:

```text
semantic bits already charged
+ one reserved bit for every remaining group
+ room
= F
```

is invariant. At run length sixteen, the fifteen-bit run representation is
guaranteed; each saved zero bit is removed from semantic charge and returned
to room, preserving the same equality. Induction over all 64,000 choices
proves that no coefficient values or rate-debt trajectory can exceed F.

### 9.2 Packet construction cannot exceed its hardware token bounds

Both candidates count every pair before selection. A normal pair over 17 bits
forces raw representation. Raw pairs are exactly 16 bits. A normal entropy
candidate at or above the format threshold is also raw. Coarse raw is 147
bits but cannot be selected because fine raw is 135 bits and length-dominates
whenever coarse raw fits. Thus every selected nonzero has:

```text
header <= 19 bits
eight pair tokens, each <= 17 bits
total packet <= 135 bits
```

and always completes in the fixed eleven token opportunities.

### 9.3 Reservoir overflow is impossible

The fused packer computes the post-output count before append and asserts
`token_ready` only if the new count is at most 46. State changes only on that
handshake. Hence a stalled standalone output cannot overflow the register.
With root `out_ready=1`, the oldest full word drains before the next token is
placed. The final-word reservation and the physical-payload invariant ensure
padding begins before word 63,999 is released.

### 9.4 Queue overwrite is fail-safe

The transform has no backpressure, so queues cannot prevent a source that
violates the specified cadence from continuing. Instead, every circular store
has independent wrap phases. In the compact profile a write is blocked when
its group pointer reaches the read group with opposite phase. A same-bank
write conflict is likewise reduced to one physical port access. The leaf sets
sticky `protocol_error` for either event; the combined Ti60 target masks that
diagnostic when `ENABLE_PROTOCOL_DIAGNOSTICS=0`. These mechanisms protect the
physical RAM port and guarded addresses, but they are not a proof that legal
traffic never reaches a guard.

For the exact canonical 640/256 cadence and the explicit nominal CSI cadence,
transform control and group completion times do not depend on pixel values.
The full-prefix proof combines those fixed arrival traces with the exhaustive
reservoir closure and eleven-clock service induction. Its conservative peaks,
including a live coefficient half, remain strictly below every compact region
and the group-collision guard. The direct OV9281 fixture additionally verifies
the fixed receiver path end to end. No claim is made for altered row or packet
timing without rerunning the same prefix analysis.

## 10. Reset, frame clear, and `SYNTHESIS`

Top-level `rst` is active high. Horizontal/vertical transform leaves and the
forward transform controller use asynchronous active-low reset through
`rst_n=~rst`. Quantizer, coefficient queues, and group encoder use synchronous
active-high reset. An accepted SOF synchronously clears frame-scoped control,
but does not erase block RAM contents.

Sticky errors clear on reset or accepted SOF. Reset initializes pointers,
wrap phases, valids, FSMs, counters, the output reservoir, and rate state.
Because transform leaf payload RAM and lifting RAM are uninitialized, validity
and operation scheduling are the sole authority for reads.

`ifndef SYNTHESIS` contains assertions and reference-only checks:

- horizontal predict/update width and row-boundary checks;
- vertical address and state-truncation checks;
- H2/H3/H4 gearbox, bank, job, and write-collision checks;
- quantizer color-conflict assertion;
- exact pair-queue full/empty comparisons and empty-read assertion;
- group rate/reservoir, token-width/alignment, service-count, and
  frame-completion consistency assertions; and
- root integration assertions.

These blocks do not select functional values or create alternate packet
behavior. The synthesized conservative queue guards, schedule admission,
rate accounting, raw fallback, reservoir admission, and public errors are the
same logic exercised in ordinary simulation. Group start/band/last checks,
committed/serialized-count overruns, the post-selection pair-token and service
bounds, and premature final-word release all set the synthesized sticky group
error and remain visible through top-level `protocol_error`. The synthesizable
Huffman ROM `initial` block is intentionally present in both modes.

## 11. Functional validation

The Rust codec and RTL share the fixed tables but use independent transform
implementations. Marlin drives Verilator models of each production module and
the complete top. The end-to-end test compares all 64,000 RTL output words
against the software encoder. Coverage includes the natural frame and full
frames containing zeros, 255, two-row stripes, checkerboard, horizontal
stripes, impulse, deterministic noise, and mixed stress.

Focused checks cover the complete q14 transform trace, every signed-q14
quantizer input in all bands, queue guard behavior, packet-local Rice reset,
17-bit pair fallback, exact-budget tail handling, the 46-bit reservoir under
backpressure, and nine-clock nonzero service. Ordinary and `SYNTHESIS`
configurations are checked with strict Verilator lint. Build the current
sources with `rtl/lwc/efinity/run_codec_encoder_top.sh` from `gateware` to
measure resource use and timing.
