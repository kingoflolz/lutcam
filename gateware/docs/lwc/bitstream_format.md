# Lean Wavelet Codec bitstream format

This is the normative bit-exact specification of Lean Wavelet Codec (LWC).
LWC has one profile: a headerless 1280x800, eight-bit grayscale frame with a
one-bit-per-pixel maximum budget. The canonical encoder emits a fixed-size
frame; compact transport omits whole zero-padding words. Both framings use
identical group packets. No field negotiates dimensions, transform, tables,
or packet order.

The FPGA architecture that produces this format is described in
[`canonical_codec.md`](canonical_codec.md).

## 1. Fixed frame contract

| Property | Value |
|---|---:|
| Input | 1280x800 unsigned eight-bit grayscale |
| Input samples | 1,024,000 |
| Transform | integer CDF 5/3, five horizontal and three vertical levels |
| Coefficients per group | 16 |
| Logical groups | 64,000 |
| Canonical fixed payload | 1,024,000 bits |
| Canonical fixed byte extent | 128,000 bytes |
| Canonical fixed 16-bit-word extent | 64,000 words |
| Canonical fixed rate | 1.0 bit/pixel |

Bits are chronological and most-significant first. Payload bit zero is byte
zero bit 7. On the RTL word interface, the first chronological bit is
`out_data[15]` and each accepted word contributes the next sixteen bits.

Packets are not aligned to bytes or words. A decoder consumes exactly 64,000
logical groups, then requires every remaining payload bit to be zero. In
canonical fixed framing, the extent must be exactly 128,000 bytes.

### 1.1 Compact framing

The 64,000-group schedule determines the semantic endpoint without an end
token. A single-zero packet represents one group, and a zero-run packet
represents its explicit group count; the last run must not exceed the number
of groups remaining. After decoding the final logical group, let `S` be the
exclusive bit position immediately following its packet. Compact framing
contains exactly:

```text
compact_bits  = 16 * ceil(S / 16)
compact_bytes = 2 * ceil(S / 16)
```

The final word retains its semantic bits and 0–15 trailing zero alignment
bits. Every complete word after that word is omitted, so no padding record
or zero reconstruction is required. A zero-valued word inside the semantic
stream is still required; trimming zero bytes by value is invalid. Compact
payloads are even-length, no larger than 128,000 bytes, and reject any extra
word, including a zero word. An exactly word-aligned semantic endpoint has
no alignment bits. A compact payload may occupy the full 128,000 bytes when
its final semantic bit lies in the last word.

The [UDP transport](../../fpga/litex/lutcam/packet.py) sends
nonempty, even-length payload fragments with all header flags zero. There is
no FINAL flag or empty terminating datagram. Validated fragments go directly
to the decoder with their camera ID, frame ID, byte offset, and sequence
information. Group packets can cross fragment boundaries; the decoder keeps
its parsing state, recognizes the 64,000th logical group, and checks the
final word's alignment bits. It needs neither a separate framing scanner nor
a fixed-size zero suffix.

Frame identity, ordering, and loss detection belong to the enclosing
transport. The semantic endpoint belongs to the decoder. The self-delimiting
schedule does not replace packet-loss detection or integrity checking.

```text
cargo run --manifest-path ../Cargo.toml -p lwc -- decode INPUT.lwc OUTPUT.png
cargo run --manifest-path ../Cargo.toml -p lwc -- decode INPUT.lwc OUTPUT.png --compact
```

The CLI decodes compact payloads. The Rust `EncodedFrame` and `Codec::decode`
APIs represent the fixed-size RTL frame used by internal codec verification.

## 2. Analysis transform

Subtract 128 from every input sample. All following transform arithmetic is
signed integer arithmetic with arithmetic right shifts and no clipping.

For an even-length one-dimensional vector, split
`E[i] = x[2i]` and `O[i] = x[2i+1]`, then compute:

```text
H[i] = O[i] - ((E[i] + E[min(i+1,last)]) >> 1)
L[i] = E[i] + ((H[max(i-1,0)] + H[i] + 2) >> 2)
```

The first expression duplicates the last even sample at the right boundary.
The second duplicates the first high sample at the left boundary. Store the
low half before the high half.

The low-low branch is decomposed in this order:

1. Horizontal level 1 over 1280x800, then vertical level 1 over the complete
   1280x800 transformed rectangle.
2. Horizontal level 2 over its 640x400 low-low rectangle, then vertical level
   2 over that complete rectangle.
3. Horizontal level 3 over its 320x200 low-low rectangle, then vertical level
   3 over that complete rectangle.
4. Horizontal level 4 over the remaining 160x100 low rectangle.
5. Horizontal level 5 over the remaining 80x100 low rectangle.

Thus the transform is 5H/3V. Vertical lifting covers both horizontal halves
at levels 1 through 3; only the low-low result continues to the next level.

The inverse visits levels 5 through 1. At a level with vertical analysis it
first reverses the vertical lift and then reverses the horizontal lift:

```text
E[i] = L[i] - ((H[max(i-1,0)] + H[i] + 2) >> 2)
O[i] = H[i] + ((E[i] + E[min(i+1,last)]) >> 1)
```

Interleave `E,O`, complete all inverse levels, and add 128.

## 3. Bands and groups

The final bands use the following IDs and rectangles in the 1280-wide
low-first Mallat layout:

| ID | Band | Rectangle `(x,y,w,h)` | Groups |
|---:|---|---|---:|
| 0 | LL5 | `(0,0,40,100)` | 250 |
| 1 | H5 | `(40,0,40,100)` | 250 |
| 2 | H4 | `(80,0,80,100)` | 500 |
| 3 | LH3 | `(0,100,160,100)` | 1,000 |
| 4 | HL3 | `(160,0,160,100)` | 1,000 |
| 5 | HH3 | `(160,100,160,100)` | 1,000 |
| 6 | LH2 | `(0,200,320,200)` | 4,000 |
| 7 | HL2 | `(320,0,320,200)` | 4,000 |
| 8 | HH2 | `(320,200,320,200)` | 4,000 |
| 9 | LH1 | `(0,400,640,400)` | 16,000 |
| 10 | HL1 | `(640,0,640,400)` | 16,000 |
| 11 | HH1 | `(640,400,640,400)` | 16,000 |

Within a band, coefficients are row-major and divided into consecutive
groups of sixteen. For band group `g` and lane `l`:

```text
flat = 16*g + l
x = band_x + (flat mod band_width)
y = band_y + floor(flat / band_width)
```

A group may therefore cross a row boundary within its band.

## 4. Normative group schedule

Packets are not emitted in band-ID order. The 64,000-entry schedule is the
LF-terminated decimal text obtained by Base64 decoding and gzip decompressing
`../software/crates/lwc/src/raw_group_schedule.b64`. Each line is one band ID in `0..11`.
For every entry, consume the next unconsumed row-major group from that band.

| Schedule check | Value |
|---|---|
| Entries | 64,000 |
| Band census | `250,250,500,1000,1000,1000,4000,4000,4000,16000,16000,16000` |
| SHA-256 of decompressed LF text | `0b5126bb22c24c296fc5981c64c07d54a8f45da53b4c67f902faa9eec3ddc29a` |
| FNV-1a-64 of parsed IDs | `f073d4481f893953` |

The decompressed sequence is normative. Base64 and gzip are repository
storage conventions only.

## 5. Scalar candidates

For signed transform coefficient `x`, shift `s`, and nonnegative band bias
`b`, define:

```text
m = abs(x)
r = (m + b) >> s
Q(x,s,b) = clamp(x < 0 ? -r : r, -128, 127)
```

Each group has a fine and a coarse candidate. The per-band parameters are:

| ID | Fine shift | Fine bias | Coarse shift | Coarse bias |
|---:|---:|---:|---:|---:|
| 0 | 1 | 0 | 2 | 1 |
| 1 | 1 | 0 | 2 | 1 |
| 2 | 2 | 0 | 3 | 3 |
| 3 | 2 | 0 | 3 | 3 |
| 4 | 2 | 1 | 3 | 3 |
| 5 | 3 | 3 | 4 | 7 |
| 6 | 3 | 2 | 4 | 7 |
| 7 | 3 | 3 | 4 | 7 |
| 8 | 4 | 6 | 5 | 15 |
| 9 | 4 | 6 | 5 | 15 |
| 10 | 4 | 6 | 5 | 17 |
| 11 | 4 | 6 | 5 | 15 |

The fine value is `qf = Q(x,fine_shift,fine_bias)`. The normative coarse
value is `Q(x,coarse_shift,coarse_bias)`. The encoder may compute that value
from `qf` and a correction bit as follows; the result must be identical:

```text
if qf == 0:
    qc = 0
else:
    base = abs(qf) >> (coarse_shift - fine_shift)
    independent = abs(Q(x,coarse_shift,coarse_bias))
    correction = independent > base
    qc = sign(qf) * (base + correction)
```

Both candidates are signed q8 vectors of sixteen values. The correction bit
and the encoder-only residual class are not serialized.

## 6. Packet-local Rice coding

For a nonzero signed q8 value `q`, let `r = abs(q)-1`. Its Rice literal is:

```text
sign(q), (r >> k) zero bits, one stop bit, low k bits of r
```

The sign is `1` for negative and `0` for positive. Low bits are emitted MSB
first. A zero value contributes no literal; its absence is represented by the
lane mask.

The parameter context is local to one packet. Initialize previous magnitude
`p=0`. Visit all lanes 0 through 15, including masked-zero lanes. Select `k`
from the current `p`, emit a literal only for a set lane, then assign
`p=abs(q[lane])`. Consequently, a zero lane resets the context for the next
lane. No state crosses a packet boundary.

Fine packets use a fixed `k` per band:

| Band ID | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Fine `k` | 4 | 4 | 2 | 1 | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

Coarse packets use three cases. `k_zero` applies when `p=0`;
`k_large` applies when `p > 2^threshold_exp`; otherwise `k_small` applies.

| ID | `k_zero` | `k_small` | `k_large` | `threshold_exp` |
|---:|---:|---:|---:|---:|
| 0 | 1 | 3 | 2 | 0 |
| 1 | 2 | 1 | 2 | 0 |
| 2 | 2 | 2 | 2 | 0 |
| 3 | 1 | 1 | 2 | 2 |
| 4 | 1 | 1 | 2 | 2 |
| 5 | 0 | 0 | 1 | 0 |
| 6 | 0 | 0 | 1 | 1 |
| 7 | 0 | 0 | 1 | 1 |
| 8 | 0 | 0 | 0 | 0 |
| 9 | 0 | 0 | 0 | 0 |
| 10 | 0 | 0 | 0 | 0 |
| 11 | 0 | 0 | 0 | 0 |

The hardware may saturate `p` to 7 because every context threshold is at
most 4; doing so does not alter any selected `k`.

## 7. Mask code

Mask bit `l` corresponds to coefficient lane `l`. A normal sparse mask is
split into nibbles `[3:0]`, `[7:4]`, `[11:8]`, and `[15:12]` and those four
nibbles are coded in that order.

| Nibble | Code | Bits |
|---:|:---|---:|
| 0 | `0` | 1 |
| 1 | `1010` | 4 |
| 2 | `111010` | 6 |
| 3 | `11000` | 5 |
| 4 | `11001` | 5 |
| 5 | `1111110` | 7 |
| 6 | `111011` | 6 |
| 7 | `11010` | 5 |
| 8 | `1011` | 4 |
| 9 | `111100` | 6 |
| 10 | `1111111` | 7 |
| 11 | `111101` | 6 |
| 12 | `11011` | 5 |
| 13 | `111110` | 6 |
| 14 | `11100` | 5 |
| 15 | `100` | 3 |

Use the four coded nibbles when their total length is at most sixteen bits.
Otherwise use a literal 16-bit mask, emitted from bit 15 through bit 0. A
nonzero mask whose Huffman length is exactly sixteen uses the Huffman form.

## 8. Packet grammar

The `quality` bit is `1` for fine and `0` for coarse. The `mask_mode` bit is
`0` for the Huffman mask and `1` for a literal mask.

```text
single zero group:
    0

normal Huffman packet:
    1, quality, 0, four Huffman nibbles, Rice literals

normal literal-mask packet:
    1, quality, 1, mask[15:0], Rice literals

fine raw packet (135 bits):
    1, 1, 0, 0, 0, 0, 0, q8[0], ..., q8[15]

coarse raw packet (147 bits):
    1, 0, 1, sixteen zero mask bits, q8[0], ..., q8[15]
```

The four zeros in the fine raw prefix are four codewords for nibble zero, not
one four-bit symbol. Raw q8 bytes are emitted lane 0 through lane 15, each in
two's-complement form and MSB first.

For a nonzero candidate, define:

```text
mask_cost = min(sum of four Huffman lengths, 16)
entropy_without_outer_tag = 2 + mask_cost + sum(Rice literal lengths)
```

Use the raw form if either condition holds:

- either consecutive lane pair `(0,1)`, `(2,3)`, ..., `(14,15)` contributes
  more than 17 Rice bits; or
- `entropy_without_outer_tag >= 134`.

Otherwise use the corresponding normal form. The pair rule gives the RTL a
bounded one-pair token and does not change reconstructed values.

The zero mask is reserved in both mask modes and disambiguates escapes:

| Prefix interpretation | Meaning |
|---|---|
| `quality=0`, Huffman zero mask | zero-run token; read an eight-bit count |
| `quality=1`, Huffman zero mask | fine raw; read sixteen q8 bytes |
| `quality=0`, literal zero mask | coarse raw; read sixteen q8 bytes |
| `quality=1`, literal zero mask | invalid |

A normal packet must have at least one set mask bit.

## 9. Zero runs

A logical run is divided into chunks no longer than 256 groups. A chunk of
length 1 through 14 is encoded as that many single zero bits. A chunk of
length 15 through 256 is encoded in fifteen bits:

```text
1, 0, 0, 0, 0, 0, 0, count[7:0]
```

The first seven bits are the nonzero tag, coarse quality, Huffman mask mode,
and four zero-nibble codewords. `count=0` means 256; every other value is its
literal run length. An encoder must not use the run token for a count below
15. A decoder must reject a run that exceeds the remaining logical groups.

## 10. Causal encoder decision

The following section fixes the canonical encoder. It does not add fields to
the bitstream.

For fine value `qf`, coarse value `qc`, fine shift `s`, source coefficient
`x`, and correction bit `a`, define `e=x-(qf<<s)`, `m=abs(qf)`, and:

```text
h = (4*abs(e) >= 2^s)  if m is odd
    (qf*e >= 0)         if m is even

C = 6  if m is odd and h=1
    2  if m is odd and h=0
    4  if m is even

assist = a  if m is odd
         h  if m is even

direct = max(0, 8*m*m + (assist ? C*m : -C*m))
r = abs(qf - (qc << (coarse_shift-fine_shift)))
incremental = max(0, 8*r*r - C*r)
```

Sum both terms over sixteen lanes. The per-band rate parameters are:

| ID | Nominal bits | Coarse energy shift | Incremental energy shift |
|---:|---:|---:|---:|
| 0 | 139 | 0 | 3 |
| 1 | 135 | 2 | 1 |
| 2 | 102 | 2 | 2 |
| 3 | 58 | 0 | 1 |
| 4 | 59 | 0 | 1 |
| 5 | 43 | 0 | 1 |
| 6 | 27 | 1 | 1 |
| 7 | 30 | 1 | 1 |
| 8 | 21 | 1 | 1 |
| 9 | 10 | 2 | 1 |
| 10 | 13 | 1 | 1 |
| 11 | 5 | 0 | 0 |

```text
coarse_gain = max(0, sum(direct)-sum(incremental))
              << coarse_energy_shift
incremental_gain = sum(incremental) << incremental_energy_shift
```

For cutoff `c`, extra-bit count `b`, and gain `g`:

```text
pass(g,b,c) = true                 if c == 0
              g != 0               if c != 0 and b == 0
              g >= b*T[c]          otherwise

T[1..8] = 1,2,2,3,4,6,8,12
```

Initialize `room=960000` and signed `debt=0`. For every scheduled group:

```text
pre_debt = debt - nominal_bits[band]
coarse_cutoff = clamp(5 + (pre_debt >> 14), 0, 8)
incremental_cutoff = clamp(6 + (pre_debt >> 14), 0, 8)

coarse_extra = coarse_packet_bits - 1
fine_extra = fine_packet_bits - 1
incremental_extra = max(fine_packet_bits-coarse_packet_bits, 0)

coarse_ok = coarse_extra <= room
            and pass(coarse_gain,coarse_extra,coarse_cutoff)

if coarse_ok:
    choose fine if fine_extra <= room and
        (fine_packet_bits <= coarse_packet_bits or
         pass(incremental_gain,incremental_extra,incremental_cutoff))
    otherwise choose coarse
else:
    choose zero

room -= selected_packet_bits - 1
debt = pre_debt + selected_packet_bits
```

The initial room reserves one bit for each of 64,000 groups. For every zero
chunk, once its sixteenth zero is committed, the fifteen-bit run form is
guaranteed; refund one room bit for that zero and each later zero through
length 256. Apply the same refund to `debt`. Then start a new chunk. This is
the only adjustment to the selected one-bit zero costs.

The encoder has two numeric inter-group rate accumulators: signed debt and
remaining room. A zero-run count is retained only to form and account for the
run syntax. Rice magnitude context resets at every packet and is not
rate-control history.

## 11. Reconstruction

For each decoded q8 value `q`, select the gain and offset for its band and
quality:

| ID | Fine gain Q8 | Fine offset Q8 | Coarse gain Q8 | Coarse offset Q8 |
|---:|---:|---:|---:|---:|
| 0 | 512 | 125 | 1023 | 57 |
| 1 | 512 | 120 | 1029 | 124 |
| 2 | 1025 | 354 | 2051 | 165 |
| 3 | 1030 | 422 | 2069 | 3 |
| 4 | 1022 | 82 | 2054 | 9 |
| 5 | 2060 | -33 | 4124 | -140 |
| 6 | 2087 | 102 | 4163 | -172 |
| 7 | 2081 | -37 | 4171 | -273 |
| 8 | 4174 | 1 | 8236 | -683 |
| 9 | 4148 | 13 | 8410 | -1028 |
| 10 | 4167 | -8 | 8139 | -551 |
| 11 | 4251 | -142 | 8019 | -655 |

```text
reconstruct(0) = 0
mag = max(0, abs(q)*gain_q8 + offset_q8)
reconstruct(q) = sign(q) * ((mag + 128) >> 8)
```

Place reconstructed coefficients into their scheduled band positions, apply
the inverse transform, add 128, and clamp each output sample to `0..255`.
This affine scalar dequantization is the complete decoder-side reconstruction;
there is no spatial restoration filter.

## 12. Decoder validation

A conforming decoder rejects:

- dimensions other than the fixed profile;
- a payload extent inconsistent with the selected framing (fixed or compact);
- payload underflow;
- an invalid or unterminated Huffman code;
- a fine literal zero mask;
- a zero run longer than the remaining group schedule;
- a Rice quotient that cannot represent signed q8;
- positive magnitude above 127 or negative magnitude above 128;
- any nonzero padding bit after the 64,000th logical group;
- trailing bytes beyond the fixed 128,000-byte budget; and
- in compact framing, extra words after the final semantic word, even if zero.

The decoder need not reproduce encoder rate state. Band identity comes from
the fixed schedule; candidate quality, masks, literals, and zero runs are
self-delimiting under the grammar above. Padding is checked only after the
schedule is complete and is not interpreted as more group packets.
