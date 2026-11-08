# LiteX LUNA wrapper

Only `litex/soc/cores/luna_cdc_acm.py` is copied from LiteX commit
`b6ae9e0b227354aecffef5339d3e946f2395ac09` (BSD-2-Clause). The rest of LiteX
remains the upstream dependency pinned by the gateware uv project.

Local changes:

- Pass the TX clock crossing's `first` and `last` markers into LUNA. Producers
  such as UARTBone must mark transfer boundaries; `valid` is not a boundary.
- Connect the raw full-speed `usb` protocol reset to the `usb_12` reset.
- Declare the tested dependencies in `pyproject.toml` instead of invoking
  `pip3 show` during HDL finalization.

The package is installed editable by `uv sync` and imported as `litex_luna`.
These behaviors are defaults; no runtime patching or feature flags are needed.
Regression coverage is in `tests/litex/test_usb_packetization.py` and
`test_usb_diagnostics.py`, including generated reset wiring, UARTBone clock
crossings, physical USB packets, enumeration, and bus reset recovery.
