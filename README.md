# LUTCam - FPGA image fiducial detection

**Note, this repo is just a snapshot of a side project. Don't expect any support or new features.**

**All the software/RTL is AI generated, but it does work ¯\\\_(ツ)\_/¯**

See blog post describing this project [here](https://benwang.dev/2026/11/01/FPGA-Fiducial.html).

LUTCam's original RTL/gateware, software, firmware, and hardware designs are
licensed under the [Apache License, Version 2.0](LICENSE). See [NOTICE](NOTICE)
for the scope and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for third-party
code, library elements, models, documentation, and data that retain their own terms.

The project is organized into three directories:

| Directory | Contents |
| --- | --- |
| [software](software/) | Camera viewer, reusable Rust libraries, and their software tests |
| [hardware](hardware/) | KiCad board and schematics, local libraries, 3D models, and datasheets |
| [gateware](gateware/) | Production RTL, FPGA targets, Rust reference models and RTL tests, Python simulations, board tools, fixtures, and design documentation |

One root `Cargo.toml` and `Cargo.lock` cover both software and gateware crates.
The default members are the four software crates; RTL verification crates are
selected explicitly or with `--workspace`.

Run these commands from the repository root:

```sh
cargo run --locked -p lutcam-viewer --release
cargo test --locked                                      # Software tests
cargo test --locked -p lrcca-rtl-tests -- --test-threads=1 # LRCCA RTL tests
cargo test --locked --workspace -- --test-threads=1       # All Rust tests
```

Run FPGA builds, Python board helpers, and HDL checks from `gateware`.
See the [gateware guide](gateware/README.md),
[board build setup](gateware/README.md#toolchain), and
[viewer guide](software/crates/lutcam-viewer/README.md).
Open [hardware/lutcam-hardware.kicad_pro](hardware/lutcam-hardware.kicad_pro)
in KiCad to work on the board.

The software and gateware directories depend on each other; keep them as
siblings. The LiteX Python environment is defined in `gateware/pyproject.toml`
and `gateware/uv.lock`; run `uv sync --locked` from `gateware` to create it.
LUNA, its LiteX wrapper, and LiteEth are vendored with the required fixes under
`gateware/vendor`; the build uses them by default.
The viewer uses `gateware/.venv/bin/python`
when available, otherwise `python3`.

Version control contains the current sources, dependency locks, required ROM
tables, test fixtures, component models, and design documentation. Build products,
captures, benchmark output, fabrication exports, historical snapshots, editor
state, and backups are local artifacts excluded by `.gitignore`.

Repository paths are relative to the documented working directory or resolved
from the calling script's location. Set `EFINITY_HOME` (or `LITEX_ENV_EFINITY`
for LiteX) to your external Efinity installation when running native FPGA tools
or vendor-model tests.
