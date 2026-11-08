# Production LFTag candidate FIFO

`LfTagCandidateFifoRamMetadata` decouples the non-backpressured logical
descriptor stream from the sequential decoder. It stores 32 complete
nine-centroid candidates in two descriptor RAM10 blocks and one metadata
RAM10 block. Any
TagDetect-specific coordinate adaptation occurs before this boundary in the
combined integration wrapper.

Each descriptor RAM is 512 by 20 bits, within the native RAM10 geometry.
Every group uses a 16-address stride, with nine descriptor addresses occupied.
The five-bit circular group pointer and four-bit tuple index concatenate into
the nine-bit RAM address. Queue occupancy and its public diagnostic use six
bits to represent zero through 32 without truncation.

A 32 by 19-bit memory holds per-candidate scale, frame, protocol and mass-spread
metadata at the same circular group pointers as the descriptors. A pop
prefetches the next head on that edge. Empty-to-nonempty commits and replacing
the sole head use explicit one-cycle forwarding, so the interface does not
depend on a RAM's read-during-write mode. Per-centroid validity travels in an
otherwise unused descriptor bit.

Groups reserve their slot on the first tuple and commit atomically at their
end marker. Malformed groups are committed with protocol status so the decoder
can emit a typed rejection. A pop on the first-tuple edge permits admission
when the queue is full. Otherwise, when no slot is available at group start,
the entire group is dropped and a sticky diagnostic is set, even if the queue
drains before the end marker. The input stream is never stalled partway
through a group.

The head remains owned while the frontend rereads descriptors for geometry.
After decode acceptance, it copies the nine measurements and their winning
cell associations into the job RAM, captures calibration, writes the header,
and then pops the head.
Rejected groups commit their short header without copying centroids.

The dual-camera design has two independent input queues and two shared job
slots, one of which includes the active refinement/output transaction. Camera
identity is retained separately for the frontend and each job. Both input
queues can hold 32 additional groups while both shared jobs are occupied.
Larger queues absorb bursts; they do not increase the refinement service rate
or remove the full-queue diagnostic.

Retained diagnostics use magic `0x4c464400`. The high-water, own-level, and
peer-level fields are six bits. The protocol-context word records the reason,
scale, frame, and tuple count at the first error.

The Rust/Marlin regression covers partial groups, malformed metadata,
simultaneous read and write, commit/pop boundaries, full-queue drops, pointer
wrap, forwarding, and diagnostic clearing.

Run the current production RTL regressions from `gateware`:

```sh
cargo test --manifest-path ../Cargo.toml -p lftag-rtl-tests --test rtl_candidate_fifo_ram_metadata --test rtl_lftag --offline
```
