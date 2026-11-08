# Lutcam's LiteEth source

Copied from the tracked tree of [enjoy-digital/liteeth](https://github.com/enjoy-digital/liteeth/tree/85901f7f8bb8c3a52b3b8fbd4ee56947da3a6a99),
commit `85901f7f8bb8c3a52b3b8fbd4ee56947da3a6a99`. Upstream version: `2026.08`.
Local package version: `2026.8+lutcam.1`. The upstream BSD-2-Clause license,
copyright notices, tests, documentation, and examples are retained.

The gateware uv project installs this directory editable. The following changes
are constructed directly by LiteEth; the SoC needs no patch installers or new
feature flags:

- IPv4 TX and conventional RX use an iterative checksum with the original
  completion latency. TX resets its checksum after packetizer drain, including
  packets discarded after a failed ARP lookup.
- MAC, ARP, IPv4, UDP, and Etherbone use indexed header serialization for aligned
  headers. Unaligned headers retain the upstream implementation.
- Fixed-address ARP receivers validate header fields as they arrive. Dynamic
  addresses retain the general receiver. The 16-bit IPv4 receiver accumulates
  its checksum while receiving the header.
- The ARP cache keeps its selected read address during RESPONSE. Two-entry
  caches use 81 Efinix SRL8 cells, with DSP48 retry and expiry timers. Larger
  tables retain RAM and countdown timers. These defaults target Lutcam's Ti60.
- Byte-wide MAC TX uses a synchronous CRC lookup ROM with the original
  backpressure and cycle behavior. Wider CRC datapaths retain upstream logic.
- The 16-bit crossbar MAC validates the complete physical frame, including FCS
  and fixed IPv4/UDP lengths, before forwarding it. Its single 2048-byte slot
  consumes and drops frames arriving while occupied; it cannot pause the wire.
- Etherbone automatically uses a streaming record receiver only when connected
  through that validated MAC/IP/UDP path. Other paths retain the buffered
  receiver. Record lengths are checked before issuing bus operations.
- Etherbone's probe parameter FIFO has one effective entry. Buffered record
  payloads carry the Efinix block-RAM mapping attribute.

Application policy, including ARP expiry while capture is armed, remains in
`fpga/litex/lutcam/soc.py`.

Regression tests in `tests/litex` cover packet bytes, transfer cycles, stalls,
reset, malformed frames, and bus side effects. `liteeth_reference.py` retains
small frozen upstream implementations solely as independent test references.
The native ARP and wire tests compile the Efinity SRL8/DSP48 simulation models
with Verilator. Set `LITEX_ENV_EFINITY` or `EFINITY_HOME` to the Efinity directory;
`LUTCAM_REQUIRE_EFINIX_NATIVE=1` makes missing native tools fail qualification.

Run from `gateware`:

```sh
uv sync --locked
uv run --locked python -m pytest -q tests/litex/test_vendored_liteeth.py \
  tests/litex/test_arp_*.py tests/litex/test_ipv4_checksum.py \
  tests/litex/test_tx_checksum.py tests/litex/test_ip_rx.py \
  tests/litex/test_stream_headers.py tests/litex/test_ethernet_crc.py \
  tests/litex/test_etherbone_*.py tests/litex/test_frame_gate*.py \
  tests/litex/test_network_control.py tests/litex/test_mac_datapath.py \
  tests/litex/test_udp_wire.py
uv run --locked python fpga/litex/lutcam_soc.py \
  --efinity /path/to/efinity/2025.2 --output-dir build/lutcam --build
```
