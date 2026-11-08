# `coefficient_quantizer`

## Role

`coefficient_quantizer` converts twelve signed fourteen-bit transform-event
lanes into signed eight-bit fine coefficients plus the one-bit metadata needed
to derive coarse values and evaluate distortion. It exploits the exact event
conflict graph so only five physical arithmetic lanes are implemented.

## Interface

Inputs are active-high `rst`, synchronous `clear_frame`, twelve valid bits, and
twelve signed coefficient values. Outputs have the same twelve valid bits,
twelve signed fine values, twelve coarse-rounding correction bits, twelve
residual-class bits, and sticky `protocol_error`. There is no ready signal and
the datapath is combinational.

## Lane coloring and dataflow

The legal physical-lane assignment is:

```text
lane 0: bands 6, 7, 9
lane 1: bands 8, 10
lane 2: band 11
lane 3: bands 0, 2, 3, 4
lane 4: bands 1, 5
```

For each active lane, package profile functions select the fine shift and
sign-dependent adjustment. Quantization is a signed adjusted arithmetic shift
with saturation to eight bits. Low source bits and the clip condition directly
produce the correction and residual-class metadata; no second quantizer or
reconstruction subtractor is instantiated. Each physical result is fanned
back to its fixed logical bands, qualified by `out_valid`.

## Timing and constraints

Values and metadata are available in the same cycle as `in_valid`; only the
sticky error register is clocked. The transform schedule must never assert two
bands assigned to the same physical lane. Any collision sets
`protocol_error` until reset or `clear_frame`, and simulation asserts the same
invariant. The profile is fixed to the twelve production band IDs and
fourteen-bit transform range.
