# Monochrome ISP

The monochrome ISP accepts two 10-bit raw pixels per transfer and produces two
8-bit pixels. It combines three functions in one streaming block:

1. sparse hot/cold-pixel replacement in the 10-bit domain;
2. a 512-bin histogram for auto-exposure measurements; and
3. a programmable 1024-entry tone lookup table (LUT).

Black-level subtraction is deliberately folded into the programmed tone LUT.
There is no separate black-level arithmetic stage in the hardware.

```text
  2 x 10-bit raw pixels
             |
             v
  sparse defect correction <--- 512 x 20-bit cumulative defect table
             |
             +------------------> 512 x 20-bit exposure histogram
             |
             v
  1024 x 8-bit tone LUT or raw[9:2] bypass
             |
             v
  2 x 8-bit output pixels
```

All three memories are programmable or observable through the module's control
interfaces. The intended implementation uses three block RAMs in total:

| Memory | Logical shape | Purpose |
| --- | ---: | --- |
| Tone LUT | 1024 x 8 | Raw-to-output mapping for both pixel lanes |
| Defect table | 512 x 20 | Sparse, cumulative defect descriptors |
| Histogram | 512 x 20 | Saturating exposure counts |

The production RTL is a single module, `MonoIsp`, parameterized by frame width
and height. [MonoIsp.md](MonoIsp.md) describes its interfaces, dataflow, memory
formats, programming lifecycle, error behavior, and throughput contract.

## Implementation and verification

The implementation is split between these source components:

- `rtl/isp/MonoIsp.sv`: synthesizable RTL;
- `rtl/isp/sources.f`: production RTL source manifest;
- `models/mono-isp/src/lib.rs`: bit-exact functional Rust model, register
  constants, tone-LUT construction, and defect-table packing;
- `models/mono-isp/tests/reference_contracts.rs`: functional-format contracts;
- `tests/isp/tests/rtl_isp.rs`: end-to-end Rust/Verilator comparison; and
- `tests/isp/tests/rtl_contracts.rs`: block-RAM topology and warning-free
  lint contracts.

Run the complete ISP regression from the repository root with:

```sh
cargo test --manifest-path ../Cargo.toml -p mono-isp -p isp-rtl-tests -- --test-threads=1
```
