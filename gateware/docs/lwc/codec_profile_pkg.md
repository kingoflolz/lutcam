# `codec_profile_pkg`

## Role

`codec_profile_pkg` is the canonical combinational profile shared by the
quantizer and group encoder. It contains no state or ports. Changing a table or
helper here changes the encoded format or the causal selection policy and must
be mirrored exactly by the Rust codec.

## Public functions and constants

- `fine_shift` and `coarse_shift` select the power-of-two quantizer steps for
  each of the twelve band IDs.
- `fine_bias`, `coarse_bias`, and `fine_signed_adjustment` define the
  sign-aware reconstruction/rounding offsets.
- `fine_rice_k` and `nominal_group_bits` provide per-band entropy and budget
  profiles.
- `energy_shift_profile` packs the incremental and coarse metric scaling for a
  band.
- `rate_cutoffs` converts signed causal rate debt into saturated coarse and
  incremental decision thresholds. `RATE_DEBT_SHIFT` is 14.
- `nibble_huffman_length`, `nibble_huffman_code`, and
  `mask_huffman_length` define the canonical nonzero-mask code.
- `quantize_signed8_with_clip` applies the signed adjustment, arithmetic shift,
  and saturation to `[-128, 127]`; bit 8 reports true clipping.
- `magnitude8` and `rice_literal_bits` provide signed-eight magnitude and exact
  Rice length arithmetic.

## Combinational behavior

All functions are pure and have no clock latency. Huffman codes are returned
right-aligned together with a separate length; consumers append the selected
low bits most-significant bit first. Rate cutoffs saturate to the implemented
range 0 through 8.

## Constraints

Band arguments are four bits, but only IDs 0 through 11 are produced by the
transform. Default branches deliberately cover the upper production bands and
are not a promise that arbitrary extra band IDs form a compatible profile.
The signed quantizer helper assumes a 14-bit coefficient source and produces
the exact nine-bit `{clipped, q}` result used by RTL and Rust parity tests.
