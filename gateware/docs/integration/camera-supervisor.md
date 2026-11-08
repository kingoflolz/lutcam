# Flash-loaded camera and network supervisor

Application images contain a minimal PicoRV32 configured for RV32EC and
4 KiB of local SRAM. The single-port register file uses the ilp32e ABI, with
gp fixed at 2032, tp fixed at zero, and t1/t2 reserved by the compiler. PC state
is 12 bits and SP is 13 bits (including the initial value 4096); other writable
registers, data and peripheral addresses remain 32 bits. Counters, interrupts,
PCPI, hardware multiply/divide and the barrel shifter are disabled. Illegal
instructions and misaligned accesses still trap. Firmware uses aligned accesses. Instruction and stack traffic
stay local; peripheral accesses use a third 4096-cycle guarded Wishbone master. The CPU initializes the cameras, controls exposure and runs
DHCP. Golden recovery has no CPU or firmware dependency.

The source is pinned under `vendor/picorv32` with its ISC license and upstream
commit. `firmware/control.c`, `start.S` and `link.ld` are freestanding firmware;
the existing verified OV9281 board table generates its sensor register list.
Camera 0 uses seven-bit SCCB address 0x60, camera 1 uses 0x10.

## Boot and initialization

Before releasing the CPU, a read-only loader takes exclusive ownership of
the existing SPI byte shifter. A continuous `03` read streams the selected
A/B slot's firmware header and payload directly into SRAM. A serial CRC32
validates the payload; there is no packet staging, page splitting, Wishbone
loader master or flash status polling. The loader then reads the flash UID,
derives the MAC, deasserts chip select and releases the shared shifter.
Invalid headers, byte timeouts, bad CRCs or a blank UID leave the CPU in reset
and release flash ownership for recovery. There is no boot ROM or XIP cache.
Firmware changes compatible with this resident ABI do not require a new core.

Each 2 MiB application slot stores firmware at relative offset `0x180000`:

| Offset | Contents |
| --- | --- |
| +0 | Little-endian magic `0x3256524c` (`LRV2`) |
| +4 | Code/data length, nonzero, multiple of four, at most 3968 bytes |
| +8 | IEEE/zlib CRC32 of the following code/data bytes |
| +12 | Required RAM size, currently 4096 |
| +16 | Firmware loaded at CPU address zero |

The linker reserves 128 bytes for stack and 56 bytes for the shared mailbox. Zero-initialized globals are included
in the CRC-checked flash image, so the loader initializes them without a BSS
clearing loop. All executable code and the compressed sensor table remain
resident; there are no overlays or runtime firmware flash reads.
The application commit record covers FPGA configuration, erased padding and
firmware together, so an interrupted update cannot commit mismatched code.
`LRV2` separates this specialized ABI from earlier firmware, which the loader
and bundle validator reject. Existing programmed golden recovery remains
untouched; newly built golden images perform the same UID read without a CPU.

At boot firmware resets both sensors, writes the 1280x800/120-fps FSIN-slave
table, and initializes both ISPs in RAW/4 bypass with frame histograms enabled.
The 131 writes per sensor retain the board table's order. The packed table
occupies 312 bytes, reusing register-page prefixes. Each I2C transaction checks
completion and ACK/status with a bounded wait; sensor-ID reads and register
readbacks are omitted. Sensors remain in standby. The live helper owns
destinations, calibration and paired admission; it asks firmware to prepare the
ISPs after a paired reset and to change sensor streaming state. Initialization
and stream-state changes require idle paired admission.

Camera initialization, I2C, histogram or command errors revoke capture admission
and reboot the current FPGA slot. This replaces partial recovery and cleanup
states with a fresh firmware load and full sensor initialization.

## Resident memory budget

The current resident firmware uses:

| Use | Bytes |
| --- | ---: |
| Instructions and constants, including the sensor table | 3554 |
| Alignment | 2 |
| Initialized globals | 16 |
| Zero-initialized globals and shared workspace | 336 |
| Padding before the fixed mailbox | 4 |
| SRAM-backed mailbox | 56 |
| Reserved stack | 128 |
| **Total physical SRAM** | **4096** |

The loaded image is 3968 bytes. Compiler stack-usage records and the linked
call graph give a conservative maximum of 100 bytes, leaving 28 bytes inside
the stack reservation. The build rejects overlaps, an excessive stack bound,
recursion, unknown indirect calls, use of the reserved registers, changes to
gp/tp, and writes to SP outside ordinary stack adjustments. Actual PicoRV32
simulations also assert that SP stays within this bound.

DHCP TX, DHCP option parsing and histogram calculations share a 268-byte work
union. Camera work waits until the outgoing packet DMA has drained. The single
32-bit × 1024-word SRAM port arbitrates the loader, host mailbox, packet DMA
and CPU. Native byte enables implement byte/halfword stores without a read /
modify / write sequence. DHCP uses one held output word, and a stalled UDP
transmitter does not reserve SRAM. TX length is fixed at the firmware's
268-byte packet size; accepting its SRAM address checks that the entire
payload fits. Received packets remain in the existing
checked MAC packet storage. The mailbox keeps its external register addresses
but uses 14 words of SRAM instead of 448 separate state bits.

## Per-board MAC address

The default address comes from the GD25LQ128E flash's factory-set 128-bit UID.
The command is `4b 00 00 00 00`, followed by 16 received UID bytes; see section
7.22 of the [manufacturer's datasheet](https://www.novitronic.com/Datenbl%C3%A4tter/Aktive%20Komponenten/DS-00533-GD25LQ128E-Rev1.1.pdf).
The loader hashes all 16 bytes with reflected CRC-64/XZ (initial/final XOR
`ffffffffffffffff`, polynomial `c96c5795d7870f42`), takes the low 48 bits and
sets the locally administered / unicast bits. All-zero and all-FF UIDs fail
boot with loader error 8. This derives a stable 46-bit address; it is not an
authentication mechanism or a guarantee against hash collisions.

The firmware CRC32 and UID CRC64 reuse one serial accumulator. After UID
completion it holds the MAC for Ethernet, ARP, DHCP and the network diagnostic
registers. A/B slots and newly built golden recovery derive the same address.
`--mac-address` remains available as an explicit build-time override. Changing
flash chips changes the default identity. Discovery needs no DHCP address;
a saved `--discover-mac` filter must use the newly derived MAC after upgrading.

## Exposure control

The ISP freezes its existing 512-bin histogram after a complete accepted frame.
The CPU consumes both snapshots while images continue, checks all bin markers
and exactly 512,000 pixel pairs per camera, and extracts P50, P90, P99 and the count
in the top four bins. Measurements use corrected RAW10 pair averages before
the tone LUT; clipping here measures those pairs, not individual RAW pixels.

At zero EV compensation, P90 is placed at 75% of the RAW range. A P99 ceiling
of 95% preserves highlight headroom. The brighter camera's percentiles limit
the linked pair. Highlights covering at most 1% of the histogram do not set
exposure. The controller computes:

```
baseline_error_stops = min(log2(384 / P90), log2(486 / P99))
error_stops = baseline_error_stops + compensation_EV
```

Values are 9-bit histogram bins; zero is treated as one. At zero or negative
compensation, if more than 1% of either camera's pairs are in bins 508..511,
the controller backs off at least half a stop per update. It also does this
when starting from a clipped scene before a usable baseline is established.
Otherwise it applies half the logarithmic error,
limited to +0.5 / -1 stop per update, with a 1/8-stop deadband. Integer log2
and a small exp2 table operate in 1/16-stop units without floating point.
The viewer exposes -2..+2 EV compensation in quarter-stop increments. Positive
compensation applies after the baseline's highlight guard: +1 EV requests
twice its exposure, and +2 EV requests four times, subject to sensor limits.
Positive compensation can intentionally clip highlights.

A clipped percentile cannot reveal how bright the scene really is. The
controller keeps the last usable percentiles with exposure removed in log
units, and predicts clipped values from that history. At a fixed exposure it
tracks illumination changes using P90, or P50 if P90 is clipped. It does not
infer illumination changes during its own exposure adjustments: sensor offsets
and imperfect gain response would otherwise amplify positive compensation.
While P99 is clipped, the illumination anchor is retained until a quarter-stop
change accumulates. This rejects small fluctuations in shadow percentiles
without losing gradual lighting changes.
If all tracked percentiles are clipped, it holds the last scene estimate until
usable measurements return, rather than continuing to increase exposure.

Integration time is preferred before gain, within 1..885 lines and gain codes
16..255. Hardware settings are rounded; a quantized move is accepted only if
it reduces the estimated EV error, preventing a one-line/two-line limit cycle.
Both sensors receive staged group updates, then END and LAUNCH commands,
followed by 25 ms settling.
The next already-frozen histogram is discarded so a subsequent measurement
comes from a full frame after settling. A failed staging or END write reboots
before either group launches. A failed LAUNCH also reboots and reinitializes
both cameras; no rollback or register readback is attempted.

The viewer's Auto exposure and Exposure compensation (EV) controls select
firmware settings. Its image-based loop is then inactive, and it displays each
camera's P90 alongside the applied exposure time and gain.
Older FPGA images retain the existing host control path. Raw diagnostic tools
must wait for firmware READY and retain the single-management-owner rule.

## DHCP

The application obtains its IPv4 address, subnet mask and gateway through DHCP.
`192.168.1.50` is a preferred address requested from the server, not a static
fallback. A DHCP server is required, and host tools must use the assigned
address. Golden recovery retains its static network configuration.

`python -m tools.lutcam.discover` finds board addresses with read-only broadcast
identity requests. The updater accepts `--discover-mac 02:4c:55:54:00:01` to
reconnect after each DHCP boot, checking the MAC before using a new address.
`--broadcast` selects a subnet broadcast on hosts with multiple interfaces.
Flash writes remain unicast, and the usual slot/build identity and full image
readback checks still apply.

The firmware performs DISCOVER/OFFER/REQUEST/ACK acquisition, then unicasts
renewals to the selected server at T1. A missing or unusable T1 defaults to
half the lease, rounded up to a whole second. Lease age is measured from the
request, not from reception of the ACK. Infinite leases do not renew.
Replies are checked for transaction ID, hardware address, selected server and
bounded option lengths; PAD, END and overloaded `file`/`sname` options are
supported.

This is a deliberately small client: requests retry every 4–5 seconds, and a
transaction still incomplete after 16 seconds restarts acquisition. There is
no T2/rebinding state, exponential backoff, duplicate-address probing, DECLINE
or RELEASE. A NAK or expired lease also restarts acquisition. Restarting DHCP
clears the address and revokes capture admission without rebooting the CPU.
Changing the address, mask or gateway revokes admission too; renewal with the
same configuration keeps capture running. A host must resolve its route and
arm capture again after revocation.

Board STATUS bit 5 advertises the CPU-owned network aperture at `0x30029000`.
Hosts can inspect it; only the CPU can consume packets or change configuration.
The hardware supplies UDP transport on client port 68, while DHCP policy lives
entirely in firmware. All outgoing payloads are the same 268-byte BOOTP buffer.
The length CSR accepts only 268, retaining the firmware write sequence without
a programmable-length register; no general-purpose UDP TX API is exposed.

| Offset | Register |
| --- | --- |
| 0x00 / 0x04 | Magic `0x4c4e4554` / RX available bit 0, TX busy bit 1 |
| 0x08 / 0x0c / 0x10 | IPv4 address / subnet mask / gateway |
| 0x14 / 0x18 / 0x1c / 0x20 | RX byte length / source IPv4 / data / release |
| 0x24 / 0x28 / 0x2c / 0x30 | TX SRAM address / fixed length 268 / destination IPv4 / start |
| 0x34 / 0x38 / 0x3c | MAC bytes 0–3 / MAC bytes 4–5, packed little-endian / preferred IPv4 |
| 0x40 / 0x44 / 0x48 | State (SELECT=1, REQUEST=2, BOUND=3, RENEW=4) / lease seconds / server IPv4 |
| 0x4c / 0x50 | Revoke capture admission / enable packet reception |

## Camera mailbox

Board STATUS bit 4 advertises the supervisor. Hosts discover this bit before
accessing the mailbox at `0x30028000`; older builds never receive missing-slave
reads. CPU result registers are read-only to hosts.

| Offset | Register |
| --- | --- |
| 0x00 | Magic `0x4c435256` |
| 0x04 | Hardware: loaded bit 0, loading bit 1, CPU trap bit 2, bus fault bit 3, loader error bits 11:8 |
| 0x08 / 0x0c | Firmware status (READY=1, BUSY=2, FAILED=4) / error |
| 0x10 | Host command sequence, written last to commit |
| 0x14 / 0x18 / 0x1c | Opcode / argument 0 / argument 1 |
| 0x20 | Completed sequence |
| 0x24 / 0x28 / 0x2c | Applied exposure lines / gain code / EV compensation code |
| 0x30 / 0x34 | Camera P90 values in RAW/4 units |
| 0x38 / 0x3c | Auto enable / main-loop heartbeat |
| 0x40 | Low 32 bits of the shared 120 MHz timestamp |
| 0x44 | Trusted I2C record length; writable only by CPU |
| 0x48 / 0x4c | Loaded length / expected firmware CRC32 |

Opcodes: 1 sensor initialization; 2 ISP preparation; 3 stream on; 4 stream off;
5 manual exposure (lines, gain; disables auto); 6 auto settings (enable, EV
code). The EV code is `0xc0 + quarters`, with `quarters` in -8..8; `0xc0` is
zero EV. Codes 0xb8..0xc8 also distinguish this firmware from the previous
mean-target protocol without changing the mailbox hardware. The live helper
uses JSON `auto_exposure` commands with `id`, `enabled` and signed integer
`ev_quarters`. A single host must serialize commands, wait for the completed
sequence and check status. Do not replay an uncertain command. A firmware
error reboots the board, so the host must reconnect and prepare capture again.

## Build and package

Run from `gateware`:

```sh
python3 firmware/build.py --output-dir build/firmware
.venv/bin/python fpga/litex/lutcam_soc.py --efinity "$EFINITY_HOME" --output-dir build/application --build
```

The normal application build invokes the firmware compiler automatically and
includes its bytes in the default build identity. LLVM clang, llvm-objcopy and
ld.lld are required; the build also accepts rust-lld from an installed Rust
toolchain. `CLANG` and `LD_LLD` override compiler/linker discovery.

Generate A/B identity variants with `tools.lutcam.ota_variants`, then supply
`--firmware build/application/firmware/firmware.flash.bin` to
`tools.lutcam.ota_bundle build` alongside the three FPGA SPI images. The bundle
validator checks firmware headers and CRC, rejects overlap, and verifies the
same bytes inside both committed application images. The OTA health check
waits for firmware READY before accepting a new application.

Verification includes actual PicoRV32 execution from modeled flash in both
slots, malformed images, flash timeouts, sensor initialization, histogram-based
exposure convergence, linked updates, reboot recovery, and live video during
histogram dumps in portable and Efinix-native ISP simulations. A separate
pin-level NOR test exercises continuous firmware reads through the maximum
supported length, the UID command, blank-UID rejection and handover to normal
packet flash commands. Viewer tests distinguish
autonomous firmware telemetry from acknowledgments of new manual commands.

DHCP verification includes native firmware packet/lease tests and actual
PicoRV32 execution through the shared SRAM and UDP DMA aperture. It exercises
acquisition, renewal, expiry, backpressure, malformed replies, short leases,
overloaded options and the timer rollover. Peripheral tests cover CPU ownership,
DMA transfers ending exactly at the 4 KiB boundary and capture revocation taking
priority over a simultaneous host arm.
