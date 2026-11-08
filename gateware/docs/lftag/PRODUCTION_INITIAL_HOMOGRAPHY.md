# Production calibrated homography frontend

`LfTagInitialHomographyFused` combines camera calibration, four-corner map
construction, canonical-hypothesis composition, and common scaling. It uses
the winning key corners and the projective terms already produced by the
homogeneous geometry scorer.

Logical Raw, LL2, and LL4 coordinates are first converted to source-image
pixel centers. Focal-length reciprocals and the principal point then place the
corners in normalized camera coordinates. The selected canonical hypothesis
is composed with that map to produce row-major H.

Only a common projective scale matters. After canonical-hypothesis
composition, the frontend chooses one power-of-two scale for all nine
coefficients and writes signed-18 Q16 values directly into the parent's
geometry RAM. `H[2][2]` remains positive in `[0.5, 2)` rather than being divided
to one. A shared 37-bit serial shifter retains each complete composition sum
until the final Q16 truncation. Existing in-range denominators keep their
previous scale; smaller and larger positive denominators select the nearest
power of two that restores this range. Nonpositive denominators and
unrepresentable coefficients are rejected, including overflow during a left
shift. This preserves the geometric ratios without a divider or an additional
DSP or RAM.

Choosing the scale after composition matters: saved camera 0 frame 9496 decoded
ID 4269, but composition placed its original Q16 denominator at 31654, just
below 32768. Applying one common factor of two produces a valid denominator
63309 and preserves the tag geometry. Decode-score thresholds are unchanged.

Composition uses one signed 18-by-18 DSP feedback lane, alongside an identical
calibration/build lane. Both are genuine arithmetic accumulators: each enabled
cycle adds one product, and a result cycle consumes and clears the sum. This
keeps the same two DSP allocation while removing the wide fabric adders and
their intermediate register bank. Denominator and coefficient terms retain
the reference implementation's fixed order. The leaf owns no block RAM. The
2,056-vector noise corpus takes at most 79 clocks; the saved frame takes 88.
Serial normalization adds nine clocks for each common shift, and the
seven-bit leaf cycle diagnostic saturates at 127. The controller's independent
compute watchdog remains unchanged. Calibration/build saturation remains
reported separately from invalid denominator or coefficient checks.

The change preserves FIFO storage and capacity, candidate ownership, pending
metadata, arbitration, and ready/valid backpressure. The serial arithmetic
adds bounded compute latency; the complete decoder regression checks the
resulting service interval and queued-candidate behavior.

The Rust/RTL regression covers all image planes, pixel-center phase,
perspective, every hypothesis, Q4 noise, range failures, result writes, stalls,
and normal and synthesis lint. Another 450 directed vectors cover the saved
candidate, both normalization directions, power-of-two boundaries, singular
denominators, coefficient overflow, and cycle-diagnostic saturation.
