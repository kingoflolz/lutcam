# LFTag descriptor frontend

The descriptor frontend combines candidate buffering and key-corner
selection. These stages share the candidate FIFO read port and pass only the
state needed by the decoder, avoiding a second nine-point register bank.

## Candidate buffering

The input integration presents one non-backpressured group of nine logical
descriptors for each retained component. The candidate FIFO has 32 group
slots and commits a group only when its end marker arrives. Two 512-by-20-bit
RAM10 banks hold descriptor words, using a 16-address stride per group.
Five-bit group pointers select the slots; the six-bit occupancy represents
every level from zero through 32. Nineteen 32-bit shift lanes hold per-group
metadata with the same registered head prefetch. In the unified
camera top, `TagDetectLwc` produces these logical descriptors from TagDetect's
generic grouped output; coordinate interpretation is not part of the LFTag
frontend.

Atomic commit gives the decoder a simple transaction boundary. A short,
overlong, or internally inconsistent group is still represented as one queue
entry so it can produce a protocol-reject packet. Slot reservation happens on
the first tuple and remains fixed through the group. A simultaneous pop makes
a full queue available for that reservation. Otherwise, if no slot is
available at the first tuple, the complete group is discarded and a sticky
drop diagnostic is set, even if space becomes available before its end.

Descriptors are stored in the ascending log-mass order produced by the
hierarchy. The two strongest entries are therefore slots 7 and 8. The frontend
also requires slot 6 to be strictly lighter than slot 7; this prevents a mass
tie from making anchor selection dependent on truncation order.

## Bounded-pose corner selection

The two anchors define the top baseline. The selector counts which side of
that baseline contains the other points and orients it toward the side with at
least six of the seven data regions.

LFTag3x3 has three canonical data rows. Within the supported pose envelope,
the projected bottom row is the set of three points with greatest positive
signed depth from the top baseline. Three short FIFO passes retain those
indices without a rank array. Their minimum and maximum baseline projections
become the bottom-left and bottom-right corners.

Descriptor arithmetic is an initiation-interval-one pipeline. The two shared
numeric DSP products are registered before the add and comparison, while a
small metadata stage carries the associated index and depth pass. Side
orientation and pass completion are folded into pipeline drain clocks, so the
extra arithmetic stage does not reduce candidate throughput. The projection
passes retain indices only and reread the two winners for their coordinates,
avoiding a temporary coordinate bank.

This rule is deliberately bounded rather than a general projective corner
solver. A weak dominant side, duplicate corner, folded quadrilateral, short
baseline, or coordinate outside the 15-bit-X/14-bit-Y Q4 envelope is rejected.
The final projective singularity and decode residual provide the remaining
geometry checks.

The selector takes 47 leaf clocks on the accepted path and occupies 49 clocks
at the parent-controller boundary.

## Handoff to homogeneous decoding

The ordered weighted corners and the five remaining weighted centroids pass
directly to the divide-free decoder. It forms projective cross products on one
candidate-wide scale and retains homogeneous coordinates in a unified
true-dual-port store shared with the hypothesis coefficient tables. The
20-bit store maps to two physical RAM10s. There is no intermediate Cartesian
rectification stream or reciprocal service.

Keeping the projective denominator with each point avoids five divisions and
preserves the weighted-centroid locations selected upstream. The same corner
cross products also provide the compact map terms needed by pose recovery, so
the later homography stage does not reread or invert the corner matrix.

## Failure behavior

Validity accumulates across the complete frontend. Invalid descriptors,
out-of-range intermediate values, and singular or sign-inconsistent
projective geometry invalidate the candidate rather than silently truncating
it. Partial candidate state is cleared atomically before the next transaction.
