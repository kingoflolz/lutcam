# Gateware verification

Rust RTL verification lives beside the Python and SystemVerilog subsystem tests.
The repository root owns the shared Cargo workspace and lockfile. The test crates
depend on the application libraries in `software/crates` and on test-only models
in `gateware/models`; the application does not depend on these test crates.

Tests use synthetic inputs, plus the Set12 codec benchmarks and
`gateware/fixtures/lrcca/test12.png`. Camera recordings and embedded vectors
extracted from those recordings are not test inputs.

| Directory | Cargo package | Coverage |
| --- | --- | --- |
| `lwc` | `lwc-rtl-tests` | Encoder, transform, quantizer, queues, and unified frontend |
| `lrcca` | `lrcca-rtl-tests` | 2PPC, hierarchy, centroids, child arena, thresholding, and TagDetect |
| `lftag` | `lftag-rtl-tests` | Decoder, pose arithmetic, shared capacity, and complete camera pipeline |
| `isp` | `isp-rtl-tests` | Monochrome ISP parity and RTL contracts |
| `csi2_rx` | `csi2-rx-rtl-tests` | CSI-2 packets, clocks, reset, and session handling |
| `support` | `rtl-test-support` | Shared Marlin/Verilator and source-manifest helpers |

Each verification crate uses `tests/*.rs` for Cargo integration tests and keeps
its SystemVerilog adapters under `tests/rtl_adapters`. Existing Python/SV test
directories retain their own runners; `cargo test --workspace` runs Rust tests
only. Rust reference-model tests stay with their model crates.

From the repository root:

```sh
cargo test --locked -p lrcca-rtl-tests -- --test-threads=1
cargo test --locked -p mono-isp -p ov9281-csi2-rx
cargo test --locked --workspace -- --test-threads=1
```

RTL simulations require Verilator and a C++ toolchain. Generated Marlin models
are cached under `gateware/target/marlin`. Long-running stress/proof tests retain
their `#[ignore]` markers and can be selected explicitly with `--ignored`.
