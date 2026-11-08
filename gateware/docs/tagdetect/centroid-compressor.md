# CentroidCompressor

`CentroidCompressor` converts an inner component's intensity mass and centered
first moments into fixed-point X/Y centroids. It also produces a coarse
logarithmic mass value used to rank saturated child lists.

## Arithmetic model

The inner CCA accumulates:

```text
m  = sum(weight)
sx = sum((2*x - (width-1)) * weight)
sy = sum((2*y - (height-1)) * weight)
```

The compressor normalizes all three values around the leading one of `m`,
approximates the reciprocal of the normalized mass, multiplies the normalized
moments by that reciprocal, and adds the image-center offset. Results are Q4
coordinates. Zero mass produces `centroid_valid = 0` and zero coordinates.

The mass rank is an unsigned Q5.3 approximation of `log2(m)`, saturated to
eight bits. The hierarchy can either reduce it into the compact area-derived
child rank used by the standalone path or retain all eight bits. The production
wavelet path retains the full value for list ordering and its generic grouped
descriptor output.

## Pipeline and optional input boundary

The block accepts one input per clock and has four clocks of valid latency:

1. Find the mass exponent, extract normalized windows, and read the reciprocal
   coefficient ROM.
2. Interpolate or reflect the reciprocal and form the logarithmic mass code.
3. Multiply normalized X/Y moments by the reciprocal.
4. Round, restore the coordinate-center offset, and register Q4 outputs.

`REGISTER_INPUT` defaults to false, preserving this four-clock interface.
Production `WaveletTagDetect` enables it through the hierarchy's
`CENTROID_REGISTER_INPUT` parameter, adding a register for the finalized mass,
moments, and valid bit before normalization. This separates the inner CCA's
moment-finalization arithmetic from the normalization multiplier chain; it
does not change any numerical result or the one-input-per-clock throughput.

`HierarchicalTwoPPC` delays child IDs, initialization flags, merge events, and
read events by the complete compressor latency: four clocks by default, five
with the input boundary. Child-list allocation and parent feedback stay at
the original CCA event cycle. The FIFO admission reserve increases from 22 to
23 epochs with the extra stage, and the valid pipeline keeps frame processing
busy until the delayed events have drained. Reset discards pending samples.

## Reciprocal ROM choices

Production enables midpoint-reflection reciprocal reconstruction. The ROM
stores one coefficient per normalized-mantissa segment, and
`CentroidMidpointReflectionReciprocal` reconstructs the 17-bit reciprocal with
a fixed multiply and shift. A segmented base-and-drop interpolation path is
also available for other elaborations.

The ROM files are runtime synthesis/simulation assets. Changing their paths or
contents changes centroid rounding and therefore requires rerunning exact Rust
parity tests.

## Helper submodules

- `CentroidPower2MultiplierNormalizer` is the production normalizer. It uses
  power-of-two scaling around the shared mass exponent to reduce general shift
  and selection logic.
- `CentroidMantissaWindow` recursively composes
  `CentroidMantissaWindowStage` blocks for the generic mass-window path.
- `CentroidSignedMomentNormalizer` recursively composes
  `CentroidSignedWindowStage` blocks for generic signed X/Y normalization.
- `CentroidMidpointReflectionReciprocal` implements the production direct
  reciprocal reconstruction.

The normalization options are elaboration-time alternatives, not runtime
modes. The production choice favors lower mapped area while preserving the
same bit-exact fixed-point result. The optional input register independently
controls whether the interface has four or five clocks of latency.

`tests/lutcam/test_centroid_registered_input.py` checks every output one clock
after the unregistered version for all four normalizer/reciprocal combinations,
including signed endpoints, every mass exponent, bubbles, and in-flight resets.
