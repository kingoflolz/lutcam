# LUTCAM Gateware

**The modules are generally written in portable verilog, but will likely need some adjustment for different FPGAs as most
have been optimized against the particular Efinix PnR flow, and the optimizations might be complicated/counterproductive
for other targets.**

Lutcam's original RTL, firmware, reference models, and supporting tools are
licensed under the [Apache License, Version 2.0](../LICENSE). Vendored code and
other third-party material retain their own terms; see
[THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md).

This directory contains the FPGA sources, verification tools, and board bring-up
utilities for the Lutcam streaming camera pipeline:

- **OV9281 CSI-2 and monochrome ISP** — RAW10 reception, tone mapping, sparse
  defect correction, and exposure statistics.
- **LWC** — a fixed-rate, one-bit-per-pixel grayscale wavelet codec and its
  production encoder RTL.
- **LRCCA / TagDetect** — linked-run connected-component analysis, hierarchical
  feature collection, adaptive thresholding, and the integrated TagDetect top
  level.
- **LFTag** — descriptor-only LFTag3x3 decoding and calibrated compact
  localization.
- **Unified integration** — one shared wavelet frontend feeding TagDetect and
  LWC, with wrapper-owned adaptation of generic centroid groups into the
  LFTag decoder.
- **Dual-camera board target** — LiteX with a flash-loaded PicoRV32 supervisor and two RAW10/MonoIsp/TagDetect
  pipelines, Etherbone and LUNA control, and image/child/accepted-pose UDP output.

The [camera supervisor](docs/integration/camera-supervisor.md) initializes both
sensors and ISPs, controls linked exposure from the ISP histograms, and runs
DHCP. Its RV32EC firmware loads from the running A/B flash slot; all code,
initialization data, working data and stack fit in 4 KiB of SRAM. New builds
derive a stable local unicast MAC from the flash UID. The supervisor guide describes DHCP behavior and verification coverage.

The [board target](fpga/litex/lutcam_soc.py) provides the build command and
configuration options. The [camera supervisor guide](docs/integration/camera-supervisor.md)
describes initialization, control registers, and firmware packaging.
The [LFTag guide](docs/lftag/README.md) describes the accepted-tag backend.
Use the [OTA bundle tool](tools/lutcam/ota_bundle.py) and
[updater](tools/lutcam/update.py) for flash packaging and updates.

The Cargo workspace is at the repository root. Application libraries live in
`../software/crates`; test-only models and Rust RTL verification live in
`models` and `tests`. Production SystemVerilog is compiled through Verilator
by the shared Marlin test support crate. See the [test layout](tests/README.md).

Run the commands below from the `gateware` directory.

## Live camera view

Run the native dual-camera viewer:

```bash
cargo run --manifest-path ../Cargo.toml --locked -p lutcam-viewer --release
```

Use **Start both** and **Stop both** for synchronized capture, or view an already
running stream. It discovers the board's current address automatically and
receives image UDP on ports `6000` and `6003`; `--board IP` overrides discovery.
Capture controls and an embedded Rerun viewer share one window, with
image zoom/pan, recent-frame timeline playback, stacked or side-by-side views,
and latest-frame grayscale PNG snapshots. Linked **Auto exposure** is on by
default: both camera meters drive one shared exposure/gain setting, with target
brightness 100 adjustable from 40 to 160. Turning it off holds the current
setting. Closing stops only capture started by this window.
See the [viewer guide](../software/crates/lutcam-viewer/README.md) for network options,
headless checks, and stream metrics.

## Layout

| Path | Purpose |
|---|---|
| `../software/crates/lwc` | LWC codec, CLI, tuning tools, fixtures, and software tests |
| `../software/crates/lutcam-viewer` | Dual-camera controls, embedded Rerun live view, and compact LWC decoding |
| `../software/crates/lrcca` | LRCCA and TagDetect Rust reference models and tests |
| `../software/crates/lftag` | LFTag reference, packet parser, and software tests |
| `models/mono-isp` | Monochrome ISP reference model and model tests |
| `models/ov9281-csi2-rx` | CSI-2 receiver reference model and model tests |
| `tests/{lwc,lrcca,lftag,isp,csi2_rx}` | Rust RTL verification crates, adapters, and subsystem Python/SV tests |
| `tests/support` | Shared production-manifest and Marlin helpers |
| `rtl/lwc` | Canonical LWC synthesizable SystemVerilog |
| `rtl/lrcca` | Canonical LRCCA/TagDetect synthesizable SystemVerilog |
| `rtl/lftag` | LFTag decoder/localizer synthesizable SystemVerilog |
| `rtl/isp` | Monochrome ISP synthesizable SystemVerilog |
| `rtl/csi2_rx` | CSI-2 receiver synthesizable SystemVerilog |
| `rtl/tagdetect_lwc` | Fixed 1280x800 unified TagDetect + LFTag + LWC top |
| `rtl/lutcam` | Direct per-camera ISP, capture, and frame-ownership integration |
| `docs/lwc` | LWC architecture and bitstream specification |
| `docs/tagdetect` | LRCCA and TagDetect architecture |
| `docs/lftag` | LFTag decoding and localization architecture |
| `docs/isp` | Monochrome ISP architecture |
| `docs/ov9281` | OV9281 CSI-2 receiver architecture |
| `docs/integration` | Combined production-top documentation |
| `fpga/litex` | Ti60F225 dual-camera LiteX target, C3 timing, and pinned dependencies |
| `vendor/luna` | LUNA 0.2.3 source with Lutcam's USB fixes and upstream tests |
| `vendor/litex_luna` | LiteX's LUNA wrapper with native transfer-marker and reset wiring |
| `vendor/liteeth` | LiteEth with Lutcam's network fixes and Ti60 defaults |
| `tools/lrcca` | Independent analysis, generation, proof, and FPGA tools |
| `tools/lutcam` | Software I2C, canonical ISP defect programming, and UDP capture helpers |
| `fixtures/lrcca` | LRCCA image fixtures |

The `csi2_rx` receiver is specialized for the OV9281's two-lane RAW10 mode
at 1280 × 800. Its fixed geometry, virtual channel, and packet handling are
described in the [receiver guide](docs/ov9281/README.md).

## Verification

Rust crates share the repository's root Cargo workspace and lockfile. Bare
`cargo test` selects the software crates. Select a hardware suite explicitly,
or use `--workspace` to include every Rust software, model, and RTL test:

```bash
cargo test --manifest-path ../Cargo.toml -p lrcca-rtl-tests --release -- --test-threads=1
cargo test --manifest-path ../Cargo.toml --workspace --release -- --test-threads=1
```

Run the workspace style and lint gates:

```bash
cargo fmt --manifest-path ../Cargo.toml --all -- --check
cargo clippy --manifest-path ../Cargo.toml --workspace --all-targets --offline -- -D warnings
```

Long-running adversarial and stress tests are ignored by default. Run a named
ignored test with `cargo test --manifest-path ../Cargo.toml -p <test-crate> --release -- --ignored <name> --test-threads=1`.

Production RTL manifests are line-oriented and contain no testbench adapters.
Run warning-fatal ordinary and synthesis-mode lint from this `gateware` directory.
These commands use the broad `-Wall -Wpedantic` checks with no warning
suppressions. The copied sources retain existing LFTag lint warnings:

```bash
verilator --lint-only --timing -Wall -Wpedantic -Irtl/lwc \
  -f rtl/lwc/codec_encoder_sources.f --top-module codec_encoder_top
verilator --lint-only --timing -DSYNTHESIS -Wall -Wpedantic -Irtl/lwc \
  -f rtl/lwc/codec_encoder_sources.f --top-module codec_encoder_top
verilator --lint-only -sv -Wall -Wpedantic -Irtl/lrcca \
  -f rtl/lrcca/sources.f --top-module WaveletTagDetect
verilator --lint-only -sv -DSYNTHESIS -Wall -Wpedantic -Irtl/lrcca \
  -f rtl/lrcca/sources.f --top-module WaveletTagDetect
verilator --lint-only --timing -Wall -Wpedantic -Irtl/lftag \
  -f rtl/lftag/sources.f --top-module LfTagCompact3x3DecoderLocalizer
verilator --lint-only --timing -DSYNTHESIS -Wall -Wpedantic -Irtl/lftag \
  -f rtl/lftag/sources.f --top-module LfTagCompact3x3DecoderLocalizer
verilator --lint-only --timing -Wall -Wpedantic \
  -Irtl/lrcca -Irtl/lwc -Irtl/lftag \
  -f rtl/tagdetect_lwc/sources.f --top-module TagDetectLwc
verilator --lint-only --timing -DSYNTHESIS -Wall -Wpedantic \
  -Irtl/lrcca -Irtl/lwc -Irtl/lftag -f rtl/tagdetect_lwc/sources.f \
  --top-module TagDetectLwc
```

The complete board build starts at `fpga/litex/lutcam_soc.py`. LiteX reads the
production RTL manifests and generates its Efinity project, peripheral setup,
and constraints under the selected build directory. Pass `--build` to run
synthesis, placement, routing, and bitstream generation.

Every application build uses the [adaptive threshold frontend](docs/tagdetect/adaptive-threshold-frontend.md):
local RAW thresholds with connectivity repair, nearest causal 5×3 LL4
mean samples for LL2, and zero-centered LL4 thresholds. The source hash identifies
the build; `ota-build.json` records `threshold_frontend: "adaptive"`.

See the [TagDetect architecture](docs/tagdetect/README.md) for shared wavelet
inputs, output contracts, buffering and verification.

Run the TagDetect visualizer:

```bash
cargo run --manifest-path ../Cargo.toml -p lrcca --release --bin tag_detect_visualize -- \
  fixtures/lrcca/test12.png --output output/tag-detect.png \
  --gif-output output/tag-detect.gif
```

## Toolchain

The Rust tests require a current Rust toolchain; RTL tests also need Verilator.
Generating or building the complete LiteX board requires Efinity 2025.2 and
[uv](https://docs.astral.sh/uv/).
The `pyproject.toml`, `.python-version`, and `uv.lock` files define the build
environment, including Python 3.11 and the pinned LiteX revision.
[LUNA](vendor/luna/VENDORED.md), the [LiteX LUNA wrapper](vendor/litex_luna/VENDORED.md),
and [LiteEth](vendor/liteeth/VENDORED.md) are vendored with the fixes applied
directly to their source. The build uses these local packages by default;
the remaining LiteX framework stays pinned upstream. No runtime USB or network
patch installers or opt-in flags are needed.

From this `gateware` directory, install the locked environment and build the
complete board image with USB enabled (the default):

```bash
uv sync --locked
uv run --locked python fpga/litex/lutcam_soc.py \
  --efinity "$EFINITY_HOME" --output-dir build/lutcam --build
```

Omit `--build` to generate HDL, constraints, peripheral configuration, and
register metadata without synthesis or place-and-route. The generated ROM
files are kept beside the HDL and referenced by relative filenames. The build
does not program hardware. `--without-usb` is only for diagnostic builds.

Run the USB regressions with the same environment:

```bash
uv run --locked python -m pytest -q tests/litex/test_usb_*.py
```

`fpga/litex/requirements.txt` is the exported pip-compatible dependency list;
regenerate it after changing dependencies with:

```bash
uv export --locked --no-hashes --output-file fpga/litex/requirements.txt
```
