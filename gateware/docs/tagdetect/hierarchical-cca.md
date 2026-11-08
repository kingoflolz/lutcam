# HierarchicalTwoPPC

`HierarchicalTwoPPC` converts the framed binary raster into groups of inner
centroids. It runs two `TwoPPC` instances over the same accepted pair and joins
their events through a centroid pipeline and a bounded child-feature arena.

## Complementary analyses

The outer CCA analyzes threshold foreground and enables one-pixel hole fill.
It tracks whether each outer component contains a real, feature-qualified
source pixel and maintains the child-list ID associated with that component.

The inner CCA analyzes the complementary background without hole fill. It
accumulates area plus centered intensity moments using inverted source
intensity as the feature value. Framing and replicated pixels remain part of
binary connectivity, but only feature-qualified source pixels contribute to
these moments.

The inner input follows the outer hole-fill stage's pending-pair schedule,
including its end-of-row flush. Both pipelines therefore process the same
pair in the same stage even when input pauses within a row. An inner-component close
can be associated with the outer parent label reported for the corresponding
lane. A first child allocates a new arena list; later children add to it. When
outer labels merge, their child-list IDs are merged as well.

## Input scheduling

Both CCAs, the arena allocator, and the event queue's admission credit must
be ready before a pair advances. A
one-entry input buffer catches a pair accepted at the public boundary if an
internal readiness condition changes. Backpressure then propagates through
`in_ready` to the threshold formatter.

The topology input and feature sidebands are deliberately separate. A pixel
can connect a component while contributing no area or moment, which is how
coarse-row replication and synthetic tag framing preserve topology without
biasing centroids.

## Child event pipeline

Closed inner moments pass through four stages of centroid compression. Child
ID, area rank, initialization state, simultaneous outer merge IDs, and outer
component closes are delayed beside the arithmetic result. Events from each
source cycle enter the queue together. The queue emits that cycle's initialize
or add, then merge, then read before advancing to any later source cycle.

The production queue stores 512 event bundles and reserves 22 entries for
events already in flight when admission stops. A stalled arena retains the
current operation. This preserves chronological order and prevents pulses
from disappearing when the service machine falls behind.

## Output behavior

When an outer component closes and has a feature-qualified child-list ID, the
arena is asked to read and retire that list. Lists shorter than `MAX_LIST_LEN`
are retired without emitting tuples. Production therefore emits nine tuples
or none. Each returned feature asserts
`inner_fv_valid`; the final feature also asserts `inner_end`. The arena emits
its retained largest children without a truncation flag. A component with
more than nine children therefore produces the same descriptor interface as
one with exactly nine retained children.

The hierarchy's feature word contains a configurable child sort key, an
optional centroid-valid bit, and Q4 X/Y centroids. The standalone configuration
uses the compact area-derived rank. The production wavelet configuration uses
the full eight-bit logarithmic mass, both to select the strongest retained
children and to expose that ranking information on its generic descriptor
boundary. The compatibility output drops the key and converts Q4 to Q5.

## Capacity and invariants

Outer and inner CCA table depths bound provisional labels per row, not total
components per frame. Arena capacity bounds simultaneously live child nodes.
The maximum list length bounds retained features per outer component.

Assertions check that queued events and parent-list allocations are accepted.
Optional assertions reject zero-area child events and ensure configurations
that promise valid centroids never pass zero mass. These checks are useful when table,
arena, or queue depths are retuned.

### Instantiated submodules

- Two `TwoPPC` pipelines implement foreground and complementary-background
  connectivity.
- `CentroidCompressor` converts the inner centered moments to fixed-point
  coordinates and a coarse mass rank.
- `ChildArena` stores, sorts, merges, emits, and retires the bounded child
  feature lists.
