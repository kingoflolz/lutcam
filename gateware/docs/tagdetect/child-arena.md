# ChildArena

`ChildArena` stores a bounded, ranked list of inner features for every live
outer component. It exists because an outer CCA label may acquire children over
many rows and may later merge with another label, while the final features are
not emitted until the outer component closes.

## Storage model

Each allocated ID names the head node of a singly linked list. The head also
stores list length and a compact tail-rank summary. Feature,
next-link, and metadata fields are held in RAM-oriented arrays. Non-head nodes
reuse the same entry format but do not need meaningful list metadata.

`ChildArenaReturnedIdFifo` owns the free-ID pool and is preloaded with every
entry on reset. `ChildArenaFvMem` is the dual-port feature store. Read or merge
retirement returns every discarded node to the free pool, so capacity measures
simultaneously live state rather than cumulative frame activity.

## Commands

The service machine accepts four commands:

- initialize a newly allocated list with its first feature;
- add a feature while preserving rank order;
- merge two lists into the first list ID;
- read and retire a completed list, emitting only a full `MAX_LIST_LEN` list.

Allocation has its own ready/response handshake so a CCA parent can receive an
ID before the corresponding initialize command reaches the service machine.
The standalone interface queues individual commands through
`ChildArenaSyncFifo`. The hierarchy enables `BUNDLED_COMMANDS`, which uses
`ChildArenaBundleFifo` to preserve each source cycle's child, merge, and read
events together. Both modes feed the same service machine. The queue lets the
hierarchy continue issuing short event bursts while the arena performs
multi-cycle list scans and merges.

The production queue holds 512 event bundles. Admission stops with 23 free
slots reserved for CCA and registered-centroid events already in flight; those
events continue to enter the queue while new pixel admission is paused. The
generic hierarchy without the centroid input register reserves 22 slots.

## Bounded ranking

Lists are sorted by the low feature-key bits and retain at most
`MAX_LIST_LEN` entries. When a full list receives a better-ranked feature, an
existing low-ranked node is reused; a worse feature is dropped. Ranking always
retains the greatest keys, without storing or emitting a truncation flag.
The production limit is nine, and reaching that limit does not invalidate
the retained list.

Merging performs an ordered two-list merge and keeps only the best bounded
suffix. The compact tail-rank summary permits a fast drop when one full list is
known to dominate the other. Equal-key service policy is configurable and
deterministic, but does not guarantee contributions from both input lists.
During a capped merge, the hierarchy's policy discards old-list entries first
on equal keys; one input can therefore supply the entire retained list.

## Concurrent service and output

The main service machine owns add and merge updates on one RAM port. A separate
read machine walks completed lists on the other port and honors output
backpressure. Port arbitration and explicit hold states cover collisions
between service reads, feature writes, and list retirement.

The final emitted beat marks the group end. Reading is destructive: every
visited node is returned to the ID pool. Lists shorter than `MAX_LIST_LEN`
emit no tuples; production emits exactly nine tuples or none. A merge path
may also retire an entire dominated list without emitting it.

## Helper submodules

- `ChildArenaSyncFifo` is the bounded command queue. Its width is a tagged
  union: merge uses the payload for the second ID, while add/init use it for a
  feature.
- `ChildArenaBundleFifo` is the hierarchy's ordered event queue. It serializes
  child, merge, and read operations from one source cycle before later cycles.
  Explicit 20-bit storage lanes avoid an inefficient Efinity decomposition of
  the production 78-bit bundle; 512 bundles occupy four RAM10 blocks.
- `ChildArenaReturnedIdFifo` is the preloaded free-ID queue and accepts IDs
  returned by reads, merges, and dropped nodes.
- `ChildArenaFvMem` supplies the feature RAM's independent read/write behavior.

Feature write data is selected by ordinary logic. The arena deliberately has
no multiplier-based selector path, so DSP blocks remain arithmetic resources.
Port-B write enables qualify separate feature-data and address selectors; the
selectors do not repeat every write handshake across all payload bits. Simulation
assertions compare their values with the original command/state selection on
every actual write. Merge service reuses the command ID registers, which remain
stable until dispatch of the next command, instead of storing duplicate IDs.

The critical sizing tradeoff is queue depth versus arena service latency. The
CCA can create events at pair rate, but sorted adds and merges are not constant
time. Queue exhaustion backpressures the hierarchy; arena exhaustion prevents
new parent-list allocation.
