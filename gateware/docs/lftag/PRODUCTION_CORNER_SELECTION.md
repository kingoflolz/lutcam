# Production nine-point corner selection

`LfTagCornerSelect9` finds four ordered key corners by repeatedly reading the
current candidate FIFO head. It uses no block RAM and does not copy all nine
coordinates into registers.

ChildArena supplies descriptors in ascending log-mass order. Slots 7 and 8
are the two anchor regions, and slot 6 must be strictly lighter than slot 7 so
the cutoff is unambiguous. The anchor direction is chosen so at least six of
the seven data points lie on its positive side.

The supported LFTag3x3 pose envelope preserves the tag's three data rows. The
selector therefore finds the three greatest positive baseline depths with
three short FIFO scans, then chooses the minimum and maximum baseline
projections among those three as the bottom corners. This top-three rule uses
one comparison datapath instead of angular sorting or a rank array.

The two numeric multiplier lanes accept one descriptor every clock. Their
products use the DSP output registers before the shared fabric add and
comparison, with only the descriptor index and depth-pass metadata advancing
beside them. Pass-boundary work overlaps the following FIFO read. Projection
passes first retain only the two winning indices, then reread those two
descriptors to populate the corner coordinates; this avoids a transient point
register bank while preserving the candidate service interval.

The output order is top-left, top-right, bottom-right, bottom-left. The leaf
rejects invalid centroids, out-of-envelope coordinates, a short baseline,
weak side dominance, an ambiguous mass cutoff, duplicate corners, and folded
quadrilaterals. Projective singularity and residual checks occur in later
stages.

The measured accepted leaf path takes 47 clocks with one-cycle FIFO reads; the
parent controller accounts for 49 clocks including launch and result
handshakes. The Rust/Marlin differential regression covers rotations,
perspective, Q4 noise, coordinate limits, ties, degenerate geometry, output
stalls, and strict normal and synthesis lint.
