# LWC RTL architecture

This directory describes the synthesizable loss-aware wavelet codec (LWC)
directly from its RTL interfaces and the executable Rust/Marlin regressions.
The production profile accepts one 1280x800, eight-bit monochrome frame as two
pixels per active clock and produces exactly 1,024,000 payload bits: 64,000
big-endian 16-bit words, or one output bit per input pixel.

The encoder is a streaming reversible integer CDF 5/3 transform followed by
fixed per-band scalar quantization, chronological sixteen-coefficient groups,
causal zero/coarse/fine rate selection, bounded Rice and mask coding, and a
fused fixed-frame packer. It has no frame buffer, content classifier,
threshold history, learned component, or image-specific state.

## Fixed profile

The payload is 1,024,000 bits per frame: exactly one bit per input pixel.
The [canonical design](canonical_codec.md) describes the implementation
contract, and the [bitstream format](bitstream_format.md) is the normative
wire specification. Run the current tests and FPGA build to measure quality,
resource use, and timing for the checked-out sources.

## End-to-end dataflow

```text
2 x 8-bit pixels
        |
        v
5-horizontal/3-vertical reversible CDF 5/3 transform
        |
        +--> LL2 and LL4 analysis taps
        |
        v
12 simultaneous-capable signed coefficient event lanes
        |
        v
five physical quantizer lanes
        |
        v
12 per-band pair queues + cadence-independent canonical replay
        |
        v
16-coefficient causal group analysis and packet serialization
        |
        v
fixed 64,000-word output frame
```

The standalone pixel-to-bitstream hierarchy is [codec_encoder_top](codec_encoder_top.md).
Systems that already own the wavelet transform can use
[codec_encoder_backend](codec_encoder_backend.md). The shared image-analysis
path is described with the rest of the [wavelet frontend](wavelet-frontend.md).

## Fixed coefficient layout

The transform emits one row-major stream per band. Multiple `out_raw_valid`
bits may be high on the same clock; each asserted bit advances only that band.

| ID | Band | Dimensions | Pair writes/frame |
|---:|---|---:|---:|
| 0 | LL5 | 40x100 | 2,000 |
| 1 | H5 | 40x100 | 2,000 |
| 2 | H4 | 80x100 | 4,000 |
| 3 | LH3 | 160x100 | 8,000 |
| 4 | HL3 | 160x100 | 8,000 |
| 5 | HH3 | 160x100 | 8,000 |
| 6 | LH2 | 320x200 | 32,000 |
| 7 | HL2 | 320x200 | 32,000 |
| 8 | HH2 | 320x200 | 32,000 |
| 9 | LH1 | 640x400 | 128,000 |
| 10 | HL1 | 640x400 | 128,000 |
| 11 | HH1 | 640x400 | 128,000 |

Every encoder group contains sixteen coefficients, transported as eight
two-coefficient transfers. There are exactly 64,000 groups per frame.

## Clocking and stream rules

- All logic is synchronous to `clk`. Low-level transform primitives use
  active-low `rst_n`; encoder-facing wrappers use active-high `rst` and perform
  the polarity conversion where needed.
- Pixel ingress has `in_valid` but no ready signal. Valid bubbles are allowed,
  ordering remains raster-major, and `in_sof` accompanies the first valid pair.
  The fixed queue capacities remain the limit on unusually bursty sources.
- The standalone top and backend have no output backpressure port. Internally,
  the group encoder is a conventional ready/valid producer, but those wrappers
  tie `out_ready` high.
- `protocol_error` is a sticky fail-visible indication within a frame. A new
  accepted SOF generates `clear_frame` and resets frame-local encoder state.
- The transform drains bottom-boundary tail jobs after the last input pair.
  Encoder output can therefore continue well after active video ends.

## Module guide

### Transform and shared frontend

- [Wavelet frontend](wavelet-frontend.md): the complete 5H/3V transform and its
  horizontal lift, vertical scheduler, shared row-service, LL2/LL4, boundary
  drain, and adjacent-row averaging blocks.

### Profile and codec backend

- [codec_profile_pkg](codec_profile_pkg.md): canonical quantizer, entropy, and
  rate-control tables and arithmetic helpers.
- [coefficient_quantizer](coefficient_quantizer.md): five-lane fine
  quantization and coarse-correction metadata.
- [coefficient_pair_queues](coefficient_pair_queues.md): per-band buffering,
  grouping, and cadence-independent canonical replay.
- [causal_group_encoder](causal_group_encoder.md): causal rate decision,
  entropy serialization, zero runs, and fixed-frame padding.
- [codec_encoder_backend](codec_encoder_backend.md): coefficient-domain
  composition wrapper.
- [codec_encoder_top](codec_encoder_top.md): complete standalone encoder.

## Executable contracts

The Rust/Marlin tests are the behavioral acceptance boundary. They compare all
transform coefficients to an independent CDF 5/3 model, compare quantizer and
rate-profile decisions to Rust, exercise queue capacity and group-encoder
backpressure, and compare complete adversarial frame bitstreams byte-for-byte
against the Rust codec. The end-to-end test also requires exactly 64,000 words,
`out_sof` only on the first valid word, and a full row period of silence after
the frame tail.

`rtl/lwc/codec_encoder_sources.f` is the sole ordered production RTL closure.
Rust/Marlin tests consume that manifest directly; scalar-port adapters under
`tests/lwc/tests/rtl_adapters` are test-only and never enter synthesis.

Run the ordinary serial release suite from the repository root with:

```bash
cargo test --manifest-path ../Cargo.toml -p lwc -p lwc-rtl-tests --release -- --test-threads=1
```

Run warning-fatal ordinary and synthesis-mode lint with:

```bash
verilator --lint-only --timing --stats -Wall -Wpedantic \
  -Irtl/lwc -f rtl/lwc/codec_encoder_sources.f --top-module codec_encoder_top
verilator --lint-only --timing --stats -DSYNTHESIS -Wall -Wpedantic \
  -Irtl/lwc -f rtl/lwc/codec_encoder_sources.f --top-module codec_encoder_top
```
