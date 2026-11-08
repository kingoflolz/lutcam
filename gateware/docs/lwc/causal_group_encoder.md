# `causal_group_encoder`

## Role

`causal_group_encoder` accepts completed sixteen-coefficient groups, chooses
zero/coarse/fine representation using only past rate state, serializes the
selected packet, and pads the result to one fixed-size frame. It is single-pass:
no complete packet payload or frame bitmap is materialized.

## Interface

Ingress is a ready/valid stream of eight coefficient pairs. Each transfer has
group start/last, band ID, two signed fine values, correction bits, and
residual-class bits. Egress is a backpressured sixteen-bit ready/valid stream;
`out_sof` qualifies its first word.

`out_padding` marks only complete zero-fill words after all semantic bits have
left the output reservoir. A final word containing both semantic bits and zero
fill is not marked. Transport may represent the marked suffix as a zero run
without changing the fixed-size frame, as described in
[the transport implementation](../../fpga/litex/lutcam/packet.py).

Status outputs expose one-cycle `frame_done` and `packets_done`, selected
`packet_bits`, committed/serialized group counts, signed `rate_debt`, remaining
`extra_room`, and sticky `protocol_error`.

## Production burst allocation

The pixel wrappers enable `BURST_LIMIT_ENABLE`; the reusable leaf defaults to
frame-budget-only allocation. Production selection also considers a 45,056-bit credit
register, replenished by two bits per actual core clock and cleared to zero by
reset or frame clear. Its deficit, shifted right by thirteen bits, supplies a
short-term cutoff from zero to five that
is combined with the existing frame cutoff using `max`. Fine/coarse candidates
must fit the available credit, and an unaffordable choice falls back to coarse
or zero. This adds no output stalls or payload memory.

The quality cutoff is captured in three bits when the eighth pair completes
the group. Both following decision clocks use the predicted admission credit
`min(credit + 6, 45056)`: one two-bit replenishment at group completion, one at
the coarse decision, and one at fine admission. The finalizer cannot charge a
different group or stall between these clocks. Capturing this common cutoff
separates the credit arithmetic from the shared gain comparator without
changing the selected packet or its commit cycle. Hard admission and credit
accounting still use the current credit; simulation checks the saved cutoff
against the original live-credit calculation on both decision clocks.

The resulting non-padding word stream is bounded by `5696 + T/4` bytes over
any `T` core clocks. The selected coefficients can change, while packet syntax,
decoder behavior, and the fixed 64,000-word frame length remain unchanged.
The [buffer-bound regression](../../tests/lutcam/test_lwc_buffer_bound.py)
checks the semantic output envelope.
Simulation assertions check both nonnegative capped credit and the complete
semantic output envelope, retaining the latter monitor across frame clears.

## Collection and causal decision

While eight pairs arrive, the collector builds fine and corrected-coarse
nonzero masks, exact Rice payload lengths, Huffman mask lengths, and bounded
direct/incremental energy. Fine values and coarse-correction bits are written
to one 16x18 dual-context pair RAM, allowing the next group to collect while a
selected earlier context is serialized.

At group completion, a short finalizer:

1. computes the pre-decision rate debt and coarse/incremental cutoffs;
2. tests coarse distortion gain and remaining-room fit;
3. tests whether fine dominates or passes incremental gain and fits;
4. commits zero, coarse, or fine and charges the exact selected length.

The frame starts with 960,000 extra bits beyond the mandatory one-bit packet
per group. There are exactly 64,000 groups. Consecutive zero selections are
run-coded; selected nonzero packets use canonical mask Huffman coding when it
is shorter, otherwise a literal mask. Oversized Rice candidates fall back to
bounded raw packet forms.

Packet length uses the equivalent direct form `literal + mask + 3`; the raw
threshold is therefore the same comparison against 135. During serialization,
coarse reconstruction uses an arithmetic half plus the exact signed rounding
adjustment. These forms are bit-identical to the analyzer definitions for all
eight-bit values, including zero and -128, but avoid rebuilding analyzer-only
magnitude logic on the serializer path.

## Serialization and output timing

Every selected header is serviced as three bounded token phases. A synchronous
Huffman ROM supplies the low and high mask bytes on consecutive finalizer
clocks. The selected context RAM then supplies eight pair tokens, normally at
one per clock; coarse Rice coding carries packet-local previous-magnitude
context.

Rice suffixes and lengths come from a 1024-by-21 true-dual-port lookup. Five
129-entry tables cover magnitudes 0 through 128 for each legal Rice parameter;
the two coefficient signs travel as a two-bit sideband. The table maps to three
RAM10s and replaces both variable unary-code shifters. Each RAM10 removes more
than fifty placed XLRs in the unified target. Pair zero is prefetched during
header phase zero and enters the lookup during phase one, so the third header
phase is followed by the first pair on the same clock schedule as before.

All header, pair, and zero-run tokens enter one chronological 46-bit reservoir.
The reservoir removes a sixteen-bit output word before appending a new token,
so output backpressure is honored without reordering. The final physical word
is reserved until all 64,000 groups are serialized. `packets_done` starts
padding mode; `frame_done` pulses when output word 63,999 transfers. The output
is therefore always exactly 64,000 words.

## Constraints and diagnostics

Start/last markers and band ID must describe exactly eight pairs per group.
Only bounded normal pair tokens (at most seventeen bits) may enter the regular
reservoir path. The implementation checks context overlap, pending descriptor
overwrite, counter bounds, output reservoir occupancy, frame group counts, and
illegal tail sequencing. `clear_frame` or reset clears all frame-local state.

For a given selected packet, token contents, external ready/valid behavior,
and the 46-bit output reservoir retain their existing serialization behavior.
Header-time prefetch hides the lookup
latency, so an unstalled packet retains the same service schedule. The new
internal response slots are covered explicitly below.

## Adversarial pipeline and reservoir safety

The Rice lookup adds one pair-RAM response slot and one lookup-response slot;
both have capacity one. Let `P` and `R` be their occupancies, `I` a pair-RAM
read issue, `E` a transfer from the pair response into the Rice lookup, and `D`
an accepted Rice token. Their exact occupancy recurrences are
`P_next = P + I - E` and `R_next = R + E - D`. A new pair read is issued only
when `P` is empty or the current pair transfers on the same clock. A lookup is
issued only when `P=1` and either `R=0` or the current lookup result transfers
on the same clock. A token can transfer only when `R=1`. Starting from
`P=R=0`, induction therefore gives `0 <= P <= 1` and `0 <= R <= 1` for every
input and output-stall sequence; simultaneous consume-and-replace leaves the
corresponding occupancy at one.

Header phase zero issues pair zero, phase one simultaneously consumes that
pair and issues pair one, and phase two leaves both responses stable until the
builder starts. Thereafter each accepted Rice token can replace itself with
the next pair. Exactly eight pair reads are authorized by the existing issued
counter, so neither address wrap nor a ninth read is possible. The selected
context remains owned until packet completion, while collection writes only
the opposite context through the independent RAM port.

Output stalls set `D=0`. Once `R=1`, the lookup-issue guard also forces `E=0`,
which in turn prevents another pair read; token bits, signs, context, counters,
and header/build state all remain stable until service resumes. The unchanged
output-reservoir recurrence is `Q_next = Q - S + A`, where `S` is an accepted
sixteen-bit word (or the final partial padding word) and `A` is the one
priority-selected token of at most seventeen bits. `token_ready` admits `A`
only when `Q-S+A <= 46`. Thus arbitrary downstream stalls can saturate and
backpressure the pipeline but cannot overwrite, duplicate, or reorder data.
The full-frame, exact-budget tail, maximum-token, and stall/saturation Rust RTL
regressions exercise these cases and reproduce the reference bitstream.
