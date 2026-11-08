# Shared LFTag decoder checks

Run from `gateware`:

```sh
python3 -m unittest discover -s tests/lftag/shared
python3 tests/lftag/refine9/test_shared.py --native
```

The shared decoder is checked against an independent nine-point fixed-point
homography reference. Twelve fixtures cover both cameras, RAW/LL2/LL4,
different intrinsics, rotation and perspective. The harness compares fitted
H coefficients within two Q16 counts (different accumulation orders), checks
IDs and frame/scale identity, and requires exact original centroid replay.

Directed cases cover actual decode/refine overlap, reset of an active or
queued camera, reset during a stalled packet or centroid group, random output
backpressure, rejected-candidate bypass, and 32 entries in both input queues
in addition to the two occupied job slots. No camera reset may disturb the
peer's work. The same cases run with native Efinity DSP models.

`candidate_cycles` measures frontend work through job commit. Refinement is
measured separately from frontend service. The configuration suite
checks residual-boundary semantics and serial-bank/direct-input parity.
