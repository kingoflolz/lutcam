# `codec_encoder_top`

## Role

`codec_encoder_top` is the standalone pixel-to-bitstream LWC encoder. It
instantiates `forward_wavelet_transform` and `codec_encoder_backend` for the
fixed 1280x800 production profile.

## Interface

- Inputs: `clk`, active-high `rst`, `in_sof`, `in_valid`, and two eight-bit
  monochrome pixels.
- Outputs: `out_sof`, `out_valid`, sixteen-bit `out_data`, and aggregate sticky
  `protocol_error`.
- Neither side has a ready signal. The source must meet the production raster
  cadence, and the sink must accept every valid output word.
- `BURST_LIMIT_ENABLE` defaults to one. It bounds semantic output by changing
  causal quality admission; setting it to zero preserves frame-budget-only allocation.

## Dataflow and frame behavior

`in_sof && in_valid` forms `clear_frame` for the backend. The transform receives
the same SOF and converts active-high reset to active-low. Only its twelve raw
coefficient lanes feed the backend; LL2, H4-low, LL4, and transform frame-done
ports are intentionally unused in this wrapper.

The transform centers pixels and emits all twelve bands while it continues to
drain row and boundary state. The backend quantizes, groups, makes causal rate
decisions, and serializes exactly 64,000 words. `out_sof` accompanies word zero
only. There is no requirement that the first output word align with input SOF
or that the last output align with the last pixel.

## Constraints and verification contract

The public shape is fixed to 1280x800, two pixels per active clock, and eight
bits per pixel. A frame must begin with exactly one valid SOF and provide the
complete raster; blank clocks may appear between valid transfers as permitted
by the transform schedule. `protocol_error` ORs transform and backend errors.

End-to-end regressions drive realistic horizontal and vertical blanking, then
drain the encoder. They compare every output byte with the Rust reference over
flat, checkerboard, stripe, impulse, noise, and mixed-stress frames; require
exactly 64,000 words; and verify a full row period with no extra tail word.
