# `coefficient_pair_queues`

## Role

`coefficient_pair_queues` is the rate-matching boundary between the parallel
wavelet transform and the scalar group encoder. It pairs successive
coefficients in each of twelve bands, makes only complete sixteen-coefficient
groups visible, and replays those groups in the codec's fixed canonical order.
The output is an eight-transfer ready/valid pair stream with group start/last
markers.

The module has two storage profiles. The default profile remains the reusable,
cadence-independent implementation. The production LWC backend selects a
compact profile specialized to the fixed 1280x800 transform and encoder.

## Common replay behavior

Each coefficient is stored as `{residual_class, correction, q}` in a ten-bit
word. Two adjacent words are read as one twenty-bit pair. A per-band half flag
selects the even or odd coefficient half, and the pair write pointer advances
only after the odd half is accepted. The reader compares only completed group
pointers, so it cannot authorize a partially written group.

The 1,605-entry, three-bit schedule ROM expands fixed simultaneous-band
families into exactly 64,000 groups. Once a group starts, the reader issues its
eight pairs in order. A registered output stage and one pending RAM response
hold payload and markers stable while `pair_ready` is low.

## Storage profiles

The default profile gives every band an independent ring: 256 pairs for band
0, 512 pairs for bands 1 through 8, and 1,024 pairs for bands 9 through 11.
Together with the schedule ROM this maps to sixteen RAM10s. Its existing early
capacity guards and simulation-only exact pointer checks are unchanged.

The production profile shares five 1024x10 payload RAMs. Every band owns a
disjoint, group-aligned pair region within its bank:

| Bank | Band regions in logical pair addresses | Used pairs |
|---:|---|---:|
| 0 | band 6: 0..175; band 9: 176..511 | 512 |
| 1 | band 8: 0..175; band 10: 176..511 | 512 |
| 2 | band 7: 0..175; band 11: 176..511 | 512 |
| 3 | band 0: 0..63; band 2: 64..127; band 3: 128..319; band 4: 320..511 | 512 |
| 4 | band 1: 0..63; band 5: 64..255 | 256 |

Each physical address appends the band's half flag to the logical pair
address. The five payload banks plus the common schedule ROM therefore use six
RAM10s. The unchanged general profile remains the module default; only
`codec_encoder_backend` and the combined `TagDetectLwc` production path enable
the compact profile.

### Trace-equivalent pointer specialization

The compact write pointers use only the bits required by their modulo class:
six bits for 64-pair regions, eight bits for the 176- and 192-pair regions,
and nine bits for 336-pair regions. The unused upper bits remain at their
reset value. This preserves the logical pointer sequence and wrap transition;
it only prevents synthesis from widening every incrementer to the largest
region. The address sequence, half selection, group visibility, and wrap phase
therefore remain cycle-for-cycle equivalent modulo the same fixed
64/192/176/336-pair capacities.

## Compact-profile adversarial no-overflow proof

### Contract and quantifiers

The capacity argument is for the fixed transform source, not for arbitrary
traffic applied directly to the leaf queue. A legal frame has 800 ordered rows
and 640 accepted two-pixel transfers per row. The direct formatter profile is
exactly 640 consecutive transfers followed by 256 idle clocks. The other
covered sources are the explicitly modeled nominal CSI trace and the fixed
OV9281 receiver fixture described below; this is not a claim about arbitrary
receiver bubbles or line periods. Pixel values are arbitrary because transform
valid timing and band selection do not depend on coefficient values. The
production group encoder has `out_ready=1`; arbitrary external `pair_ready`
stalls, altered direct-profile blanking, and arbitrary leaf-queue valid
patterns are outside the compact lossless contract. A new frame is cleared
only after the previous transform, queue, and encoder have drained.

Generalizing beyond the enumerated nominal and receiver traces requires a
proved receiver arrival envelope in the core-clock domain. FIFO occupancy
alone does not establish one: synchronizer phase, gearbox cadence,
packet-marker generation, and any permitted mid-row bubbles must be included.
Consequently no broader CSI cadence is inferred here by subtracting a FIFO
depth from nominal packet time.

Let `W_b(t)` be a completed pair write for band `b`, `R_b(t)` a pair read
issue, and `Q_b(t)` the unread complete-pair occupancy immediately after clock
`t`. The exact recurrence, including simultaneous endpoints, is

```text
Q_b(t+1) = Q_b(t) + W_b(t) - R_b(t).
```

More explicitly, let `A_b` be a coefficient admitted after the capacity and
bank-port grants, and let `H_b` be the pre-clock half flag. Then
`W_b=A_b&H_b` and `H_b(next)=H_b xor A_b`. A read issue occurs only for the
authorized current group. Induction from `Q=H=0` gives `Q>=0` and
`H in {0,1}`; a simultaneous `W=R=1` leaves `Q` unchanged. The first
coefficient of a pair is already resident while it is not yet included in
`W_b`, so physical pair slots in use are bounded by `Q_b+H_b`.

The hardware read-group pointer advances only when pair seven is issued. Let
`G_hw` add each `W` and subtract eight at that actual pointer advance. The
capacity checker uses a more conservative group-held occupancy `G_hat`, whose
all-eight-pair retirement is no earlier than the corresponding RTL event.
Coupling the same writes and delaying only releases proves
`Q_b(t)<=G_hw_b(t)<=G_hat_b(t)` for every prefix. The virtual delay can
accumulate across groups, so no fixed upper bound on `G_hat-Q` is assumed.

### Encoder service lemma

With production `out_ready=1`, the output reservoir cannot stall packet
service. Before the reserved final word, one sixteen-bit word is removed on
every clock whose pre-append occupancy `c` is at least sixteen. For a token of
length `a`, the exact recurrence is

```text
c_next = c - 16*[c >= 16] + a.
```

An exhaustive finite-state closure starts with packet-boundary set `{0}` and
allows an optional run token of 1..15 bits followed by either:

- a defensive one-bit packet with exactly three padded header opportunities;
  or
- a selected nonzero packet with eleven opportunities, each 0..17 bits, and
  total length 2..135 bits.

Every token-length combination satisfying only those bounds is included, so
the closure over-approximates every Huffman, literal-mask, raw, one-bit, and
zero-run sequence. It reaches the fixed packet-boundary set `0..38`; the
largest occupancy at any intermediate prefix is 45, below the 46-bit
reservoir, and no transition is rejected. Once physical word 63,999 is
reserved, at most sixteen semantic bits remain: the already buffered bits plus
all future bits are bounded by the one-bit-per-pixel payload invariant. Thus
the final packet and padding cannot overflow either.

The remaining control-state induction gives an eleven-clock group-start
bound. If a group starts at `S`, its eight input pairs occupy `S..S+7`. The
coarse, fine/low, and high-read finalizer clocks are `S+8..S+10`. When an older
nonzero packet exists, its header phase zero coincides with `S`; its three
header opportunities and eight pair opportunities complete by the end of
`S+10`, while the new group uses the alternate context. The older pending
descriptor is cleared before the new group's pair seven. A one-bit packet or
a state-zero group completes earlier.

A zero run also adds no twelfth clock. If a run precedes selected group `B`,
at least one state-zero group separates `B` from the preceding nonzero build,
so that build is already complete when `B` reaches its high-read finalizer
clock. `run_flush_pending` therefore consumes the single run token on
`B.start+10`, before the finalizer returns idle, and `B`'s header phase zero
can coincide with the successor's pair zero at `B.start+11`. The 255-to-zero
run-counter wrap has 256 intervening zero groups and obeys the same argument.
A tail run has no successor to delay. Induction from reset therefore proves
that a continuously available next group starts no later than eleven clocks
after the preceding group for every coefficient and rate state.

### Full-prefix arrival/service evaluation

For scheduled group `i`, let `a_i` be the clock of its eighth pair write. The
executable proof evaluates the entire fixed 64,000-group schedule with

```text
s_i = max(a_i + 1, s_(i-1) + 11).
```

The completed group cannot be visible before `a_i+1`. By the service lemma and
induction, this virtual start is no earlier than the corresponding RTL read
start. The RTL can prefetch pair zero, hold it through the three finalizer
clocks, and then issue the remaining reads consecutively; its read-group
pointer advances no later than ten clocks after read start. The proof delays
the all-eight-pair release until `s_i+11`, so the resulting `G_hat` dominates
the synthesized queue state.

Transform and quantizer valid control depends only on source valid timing,
row/column counters, and scheduler state; coefficient values feed only the
arithmetic data path. Consequently each covered timing profile has one event
trace for all possible pixel values. The checker records every pair and group
arrival from the RTL for all 800 rows, applies the recurrence above in exact
canonical schedule order, combines simultaneous write/release deltas, and
tests every prefix. It obtains:

| Timing profile | Exact `G_hat` peaks for bands 0..11 |
|---|---|
| 640 consecutive + 256 idle | `24,24,40,80,80,80,160,160,160,318,320,320` |
| nominal 2-valid/1-idle CSI cadence | `24,24,40,68,69,71,160,146,152,208,208,208` |
| direct OV9281 receiver fixture | `24,24,40,68,69,71,160,147,152,208,208,208` |

The receiver fixture drives 800 fixed-period RAW10 packets with varied lane
alignment and packet phase. Its observed core row periods are 1,246..1,252
clocks. A test-only monitor records the transform raw-valid trace, applies the
same dominated recurrence to every prefix, asserts all capacity inequalities,
and checks that the combined RTL produces a bit-exact frame with no receiver,
queue, or codec protocol error. This covers that exact fixture; it is not a
claim about arbitrary packet spacing.

The common bound `B` and the compact capacities are:

| Bands | Required pair bound `B` | Region capacity `C` | Capacity check |
|---|---:|---:|---|
| 0, 1 | 24 | 64 | `B+1=25<C` |
| 2 | 40 | 64 | `B+1=41<C` |
| 3, 4, 5 | 80 | 192 | `B+1=81<C` |
| 6, 7, 8 | 160 | 176 | `B+1=161<C` |
| 9, 10, 11 | 320 | 336 | `B+1=321<C` |

Since `Q<=G_hat`, the extra one in `B+1<C` covers a resident first coefficient
of an incomplete pair and keeps physical usage strictly below capacity. It
also keeps the virtual group-held state below capacity, so the real
group-collision guard cannot fire. The implementation additionally leaves more
than one complete group of margin (`B<C-8`) in every region. That is a sizing
margin, not the guard's trigger equation: the guard coincides with exact full
when no group is being read and can act up to seven pair positions early while
a group read is active.

This also removes circularity between admission and the occupancy trace.
Initially all pointers and half flags are zero. Assume every requested legal
write through a prefix has been admitted. The checker includes the next
requested event and still gives `G_hat+H<C`; the bank-mask proof gives at most
one request per physical write port. Therefore neither compact guard can block
that event, so admitted and requested writes remain identical. Induction over
the complete trace proves that the guard and priority mux drop no legal
coefficient.

Uniform 610-, 1,024-, and 4,096-clock blanking traces are retained as
additional regressions, but are deliberately outside the quantified compact
contract rather than used to infer a general stutter-monotonic theorem.

### RAM-port proof

The five write-bank color sets are `{6,9}`, `{8,10}`, `{7,11}`,
`{0,2,3,4}`, and `{1,5}`. The global replay controller issues at most one pair
read in total. A priority grant also limits every physical bank to one write
even for malformed input. Thus the physical RAM port count is safe
unconditionally; proving that the priority grant never drops a legal write
requires mutual exclusion of the raw-valid producers within every color set.

The producer identities reduce that obligation substantially. Bands 9, 10,
and 11 are all assigned from the same `v1_out_valid`, so in particular
`valid11 == valid9`. Bands 7 and 8 are the two outputs of lower context 1;
band 6 is lower context 0; band 3 is lower context 2; bands 4 and 5 are lower
context 3; band 2 is the H4 service event; and bands 0 and 1 are the H5 service
event. The direct and nominal full-frame control-trace checkers assert all five
masks on every raw event, including the non-obvious `{7,11}` bank. The OV9281
fixture captures the same raw-valid stream for its prefix recurrence and keeps
the compact leaf protocol diagnostic asserted false throughout, which also
excludes a bank conflict. Since these valid traces are coefficient-data
independent, this quantifies all pixel values rather than a pixel corpus. Thus
no legal write is discarded by the priority grant in a covered profile.

For band `b`, local pair pointer `p` and half `h`, the coefficient address is
`F_b(p,h)=2*(base_b+p)+h`, with `0<=p<C_b` and `h in {0,1}`. Every table row
above satisfies `base_b+C_b<=512`, and regions sharing a bank are disjoint
half-open intervals `[base_b,base_b+C_b)`. Therefore `F` is injective across
all legal writers in a bank. A pair read uses `base_b+p_r` and returns exactly
the two coefficient addresses `F_b(p_r,0)` and `F_b(p_r,1)`.

For the same band, the proved `Q+H<C` prefix invariant makes writer/read alias
impossible, including a simultaneous endpoint, so correctness does not depend
on the device's read-during-write value.

### Fail-safe behavior outside the contract

Write and read pointers wrap at their region terminal count and carry separate
phase bits. A compact write is blocked when its write-group pointer reaches the
read-group pointer with opposite phase. The guard coincides with exact full
when no group is active and may be up to seven pair positions early during a
group read. If malformed input requests two writes to one bank, a priority mux
enables only one physical RAM write. Either condition sets the leaf module's
sticky `protocol_error`; neither can create a multi-driven port or overwrite
an address already identified by the guard. These guards are fail-safe
mechanisms, not evidence that a legal trace cannot reach them.

Simulation retains an exact-empty read assertion and an assertion that exact
full implies the group guard. Those assertions check the guard relationship;
they do not prove the legal-trace occupancy bound. In the combined production
target, `ENABLE_PROTOCOL_DIAGNOSTICS=0` masks the diagnostic at the wrapper, so
the priority/blocking behavior remains functional but the leaf error is not an
externally visible target signal.

Reset or `clear_frame` clears half flags, pointers, wrap phases, schedule state,
pending responses, output valid, and the sticky error. Payload RAM is not
erased; cleared validity and pointer state make stale words unreachable. Local
write pointers wrap after `C_b` pairs and local read pointers after `C_b/8`
groups, toggling their independent phase bits. The established legal occupancy
invariant is strict across the complete frame, so equal pointers with equal
phase are unambiguously empty and equal group pointers with opposite phase are
the collision state across any number of wraps.

The synchronous clear has priority over all writes, reads, and pending output
state. In the production wrappers it is generated by the first accepted SOF.
That pixel still has the transform pipeline latency before any coefficient can
reach this queue, so clearing cannot discard a new-frame coefficient. The
contract requires the previous frame to have drained first; using SOF as an
abort would intentionally discard old queued data. Multi-frame end-to-end
tests reset once, cross the compact rings repeatedly, clear at each drained
SOF, and compare every frame bit-exactly. The compact profile promises
bit-exact output only under the stated transform and drain contract. Outside
it, the guards promise physical memory safety and set the leaf diagnostic, not
recovery of a malformed frame or visibility through a wrapper that masks that
diagnostic.
