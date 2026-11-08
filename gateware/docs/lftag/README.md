# LFTag3x3 decoder and localizer

The dual-camera image uses `SharedLfTagDecoder` to turn nine weighted centroids
into a 14-bit tag ID and calibrated rotation/translation. The winning decode is
followed by an independent **all-nine-point homography fit**. No image pixels,
gradients, or frame buffer are needed.

```text
camera 0: 32-candidate FIFO ─┐
                           ├─ corner selection → 16-hypothesis decode
camera 1: 32-candidate FIFO ─┘                       ↓ residual acceptance
                                             two-entry job queue
                                                    ↓
                                             nine-point fit → pose
                                                    ↓
                                             packet + centroid replay
```

The frontend can decode the next candidate while the backend refines the
previous accepted tag. Rejected candidates bypass fitting. The queue retains
camera identity and output order; a full queue stops dispatch, while the camera
candidate FIFOs continue accepting complete groups until their own capacity is
exhausted.

## Candidate contract

Candidates contain exactly nine centroids in ascending log-mass order. Slots
7 and 8 are the two large fixed regions used as the top baseline. Coordinates
are unsigned Q4 in one logical input plane. Scale, frame sequence and centroid
validity travel with the group. `TagDetectLwc` removes composite-plane offsets
before this boundary. Retaining the nine largest children is normal selection,
not a truncation fault.

Ingress is not backpressured. Each camera FIFO reserves a slot on the first
tuple and commits groups atomically. Malformed groups retain a typed rejection;
full-group drops and protocol errors retain sticky diagnostics. The current
head is popped when its job has been completely written. There are 32 FIFO
slots per camera plus two shared jobs, including the active backend job.

The corner selector orients the large-region baseline, identifies the bottom
row and its extremes, and rejects invalid or unsupported geometry. The scorer
uses the four corners to form a homogeneous map and evaluates the other five
points against all sixteen bottom-corner symbol hypotheses. Every point must
occupy a distinct legal cell. Strict first-minimum tie breaking, decoded IDs,
raw scores, and normalized-residual acceptance are unchanged.

The winning point-to-cell correspondence is saved alongside each cached X and
Y coordinate. This preserves the association even after the frontend starts
another candidate, without retaining another index register bank.

See [FRONTEND.md](FRONTEND.md), [DECODER.md](DECODER.md), and
[the homogeneous scorer](PRODUCTION_HOMOGENEOUS_SCORER.md) for decoding details.
The image frontend is described in
[adaptive thresholding](../tagdetect/adaptive-threshold-frontend.md).

## Accepted-tag refinement

`LfTagFit9` normalizes each image axis about the centroid bounding-box midpoint
with a power-of-two scale. All nine correspondences contribute to the 18
linear homography equations. A block Schur solve reduces the eight-unknown
normal system to one shared 3-by-3 inverse and one 2-by-2 solve. Only two
reciprocals are needed. Scratch values are signed 20-bit Q2.17; dot products
retain six additional guard bits and issue one product per clock.

A 161-word schedule shares the pipelined multiplier and arithmetic lanes.
Its control words occupy the unused upper half of the job RAM. This engine is
independent of the camera-control PicoRV32 and does not change its 4 KiB RAM
or firmware. Three scratch banks supply two reads and one write per clock;
the existing pose engine reuses two of them after fitting. The fitted matrix
is converted into calibrated camera coordinates, including the LL4 vertical
offset, and passed to the existing pose decomposition module. Singular or
out-of-range fits and invalid poses produce a singular-map rejection.
DSPs perform multiplication, addition, or subtraction; none serves as a mux
or pass-through stage.

This is a linear least-squares homography fit, followed by the existing compact
planar pose decomposition. It is not an iterative reprojection-error or SVD
pose optimizer. The packet's quality field remains the **decode residual**;
it is not relabeled as a nine-point fit residual.

See [LOCALIZATION.md](LOCALIZATION.md) for equations and packet fields.

## Configuration, reset, and outputs

The shared 16-word configuration bank retains its existing addresses: printed
width at 0, residual limit at 1, mass/baseline policy at 7, and camera intrinsics
at 8–15. Only the frontend reads this bank, with one outstanding transaction.
After residual acceptance it caches the camera intrinsics and printed width
in the job RAM before committing the job. The backend then reads that snapshot
without contending with the next decode. `EXTERNAL_CONFIG=1` holds each request
under backpressure; direct inputs remain available with `EXTERNAL_CONFIG=0`.

Configuration writes require paused cameras with drained work. `busy` includes
candidate FIFOs, the frontend owner, and both live jobs. Camera reset cancels
that camera's queued/active work and any partial output; it preserves the other
camera. An outstanding configuration response is consumed before another
transaction can issue, including after a camera reset.

Accepted packets contain 22 words: the common eight-word header, pose flags,
normalized decode residual, nine signed-Q29 rotation entries, and three
signed-Q16 translation entries. Rejections contain eight words. Header bit 30
and reserved words remain zero. Accepted-centroid replay
preserves the nine original measurements, mass order, validity, and the
existing composite-plane Q5 wire format. Packet and centroid outputs hold data
stable under backpressure, with ownership retained through the last tuple.

## Timing and memory

`candidate_start`, `candidate_done`, and the 10-bit `candidate_cycles` now
measure **frontend decoding through job commit**. The 999-clock watchdog applies
to that computation, excluding idle waiting for a free job slot. These fields
do not include refinement, queue waiting, or output serialization.

The integrated fit completes in **887 clocks**, or **7.39 µs at 120 MHz**,
including point normalization, matrix accumulation, both solves, and camera
calibration. The regression fixtures take **955 clocks including pose**
(7.96 µs). Standalone fitting measures 886 clocks because its interface omits
one integration handshake. Packet output and centroid replay follow pose;
backpressure can extend that service interval. Frontend decoding overlaps the
fit, and waits only when both job slots remain occupied.

A saturated native-DSP RTL run with 120 accepted tags measures one completed
result every **1,075 clocks**, or **8.958 microseconds / 111,628 accepted
detections per second at 120 MHz**, shared across both cameras and all three
scales. This includes accepted-centroid export and assumes no output stalls
or host contention for configuration RAM. The fixtures use the production
single-cycle configuration response and exercise RAW, LL2, and LL4.

Frontend decode takes 670–682 clocks on those fixtures. It overlaps the
955-clock fit/pose operation. The backend additionally spends 75 clocks on
the 22-word pose packet, 36 on the nine original centroids, and nine on job
dispatch/retirement, giving the 1,075-clock service interval. Fitting, pose,
and output for different jobs do not overlap within this backend, so it is
the accepted-stream bottleneck. Rejected candidates bypass fitting and pose.
The first fixture takes 1,713 clocks (14.275 microseconds) from frontend start
to the end of its pose packet, excluding input-queue wait; all original
centroids finish at 1,749 clocks. This latency is distinct from the sustained
1,075-clock output interval. Network backpressure and finite candidate queues
can reduce delivered throughput; repeated detections at multiple scales each
consume a job. The test reports zero FIFO drops and verifies all IDs, pose-fit
coefficients, camera/scale ownership, and centroid replay while decoding and
refinement overlap. The [shared-backend test](../../tests/lftag/refine9/test_shared.py)
checks the production path.

The shared LFTag hierarchy uses 13 BRAMs:

| Use | BRAMs |
| --- | ---: |
| Two 32-candidate input queues | 6 |
| Homogeneous geometry/scorer | 2 |
| Two-entry job queue and fit control words | 1 |
| Fit scratch and pose result banks | 3 |
| Reciprocal-square-root table | 1 |

## RTL and verification

`rtl/lftag/sources.f` lists the source closure. The production shared path uses
`LfTagDecodeFrontend`, `LfTagAcceptedRefiner`, `LfTagFit9`, the existing scorer,
and the existing pose/reciprocal-square-root modules. The standalone
`LfTagCompact3x3DecoderLocalizer` retains its four-corner path for its independent
reference and leaf regressions; its leaf cycle counts do not describe
the shared production path.

`tests/lftag/refine9` contains a scheduled fixed-point reference and an
independent NumPy least-squares solve,
100 complete calibrated-fit fixtures, and the integrated two-camera harness.
The latter checks RAW/LL2/LL4, differing calibration banks, preserved IDs and
centroids under varying mass order, full-circle rotation, actual
frontend/backend overlap, reset during fitting or output,
backpressure, rejected-candidate bypass, and both full input queues. Native DSP
models run the same integration checks. The existing configuration-fetch suite
also checks exact residual boundaries, randomized stalls, stale responses, and
both camera banks against direct configuration.

Run from `gateware`:

```sh
python3 tests/lftag/refine9/test_fit.py
python3 tests/lftag/refine9/test_shared.py
python3 tests/lftag/refine9/test_shared.py --native
python3 tests/lftag/refine9/test_accuracy.py
python3 tests/lftag/refine9/test_arithmetic.py
LITEX_ENV_EFINITY="$EFINITY_HOME" .venv/bin/python -m unittest discover -s tests/lftag/config_fetch
```
