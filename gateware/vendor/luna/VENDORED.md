# Lutcam's LUNA source

This is the official [LUNA USB 0.2.3 source distribution](https://pypi.org/project/luna-usb/0.2.3/),
with local version `0.2.3+lutcam.1`. The upstream package, tests, README, and
BSD license are retained. Generated distribution metadata is omitted.

Upstream archive: `luna_usb-0.2.3.tar.gz`

SHA-256: `32958b5e97481d2eb8ce9c8adc784cb44e59f9f07ca860a103272230822c8285`

Local changes:

- `gateware/interface/gateware_phy/phy.py`: correct UTMI operating-mode and
  line-state encodings; clamp both output enables and the pull-up while
  disconnected or in reset.
- `gateware/interface/gateware_phy/transmitter.py`: run the existing NRZI
  encoder and registered outputs in the 12 MHz transmit domain. Remove the
  independent data/output-enable crossings and free-running 48 MHz strobe.
  The 48 MHz receiver and clock recovery are unchanged.
- `gateware/usb/usb2/device.py`: reset endpoints and shared packet state on
  host bus reset. Keep the physical PHY/translator outside that reset, and
  connect raw-PHY output gating to the device's connection control.
- `gateware/usb/devices/acm.py`: select direct descriptor lookup on the 12 MHz
  full-speed path to meet response turnaround without a global environment
  override.
- `pyproject.toml`: use a static local version, avoiding Git-derived versions
  in a vendored source tree.

Code paths above are relative to `luna/` in this directory. The application
uses these fixes through the ordinary LUNA API; it no longer fingerprints
upstream ASTs or replaces elaborated fragments. The remaining helpers in
`fpga/litex/lutcam/usb.py` fix LiteX's stream-marker and reset connections and
implement the board's reconnect delay.

From the enclosing `gateware` directory:

```bash
uv sync --locked
uv run --locked python -m pytest -q tests/litex/test_usb_*.py
uv run --locked python -m pytest -q --import-mode=importlib vendor/luna/tests
uv run --locked python fpga/litex/lutcam_soc.py \
  --efinity /path/to/efinity/2025.2 --output-dir build/lutcam --build
```

USB regressions cover physical packets, disconnect clamping, abandoned
control transfers, resets during bulk transfers, descriptor bytes and response
turnaround across related-clock phases, and transmit-source skew. Preserve
these checks when updating the vendor snapshot. Update this note, the local
version and uv lock/export whenever the vendored fixes change.
