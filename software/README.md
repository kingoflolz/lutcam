# Lutcam software

Lutcam's original software is licensed under the
[Apache License, Version 2.0](../LICENSE). Dependencies and third-party assets
retain their own terms; see [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md).

This directory contains the camera viewer and the reusable `lwc`, `lftag`, and
`lrcca` libraries, together with their ordinary Rust tests. All four are default
members of the repository's root Cargo workspace and share its lockfile.

Run commands from this directory:

```sh
cargo run --locked -p lutcam-viewer --release
cargo test --locked --release -- --test-threads=1
cargo fmt --all -- --check
```

The [viewer guide](crates/lutcam-viewer/README.md) documents capture, calibration,
network options, and playback. FPGA sources, shared test fixtures, Python board
helpers, and architecture documentation are in [../gateware](../gateware/).
Rust RTL verification crates live under [../gateware/tests](../gateware/tests/)
and require Verilator. Run a subsystem with `cargo test -p lrcca-rtl-tests`, or
include all Rust software, reference-model, and RTL tests with
`cargo test --workspace -- --test-threads=1`. RTL simulation artifacts are written
to `../gateware/target/marlin`; Cargo artifacts use the root `target` directory.
