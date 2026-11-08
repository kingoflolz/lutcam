# LUTCam Live

Run Cargo commands in this guide from the `software` directory. Python board
helpers live in `../gateware/tools/lutcam`. Cargo uses the workspace and lockfile
at the repository root, with executables written to `../target`.

A native Rust app with capture controls and an embedded Rerun viewer in one
window. It receives both 1280 × 800 monochrome camera feeds, assembles LUTCam
image UDP packets, and decodes them with the workspace's existing `lwc` codec.
Rerun supplies image zoom, pan, and a timeline of recent frames. Camera 1 is on
the left and camera 0 on the right; both images and their overlays are flipped
top-to-bottom in software. Camera identities and recorded source coordinates
remain unchanged.

## Run

From the repository root:

```bash
cargo run --locked -p lutcam-viewer --release
```

The viewer automatically discovers the camera board on the local network,
including its DHCP-assigned address. It uses that address for capture control
and as the expected image source on UDP **6000** (camera 0) and **6003** (camera 1).
Click **Start both** to configure both sensors and begin synchronized optical
capture at the 120 fps preset. Existing camera streams can also be viewed
without taking control.

`--board ADDRESS` overrides discovery for a board on another subnet or when
several camera boards are present. `--bind IP` selects a local address for
discovery and video reception; the default listens on all local IPv4 interfaces.
An explicit `--source IP|any` remains an override for the video filter. Headless
receive-only runs with an explicit source skip discovery.
The GUI uses the native X11 or Wayland desktop and Rerun's renderer; no separate
Rerun process or window is required. Rerun dependencies are pinned to **0.34.1**.
Keep `Cargo.lock` and use `--locked`: its `fixed` **1.30.0** selection supports
the checkout's Rust 1.92 toolchain.

Capture control uses the repository's Python helper over Etherbone. Run from
this checkout with its Python dependencies installed (`.venv/bin/python` is
preferred when available, otherwise `python3`). The helper powers/configures
both sensors, applies the pinhole calibration and ISP bypass, and starts
the FPGA's paired FSYNC generator with zero phase offset. Initial values use
2.8 mm lenses and the [OV9281's 3 µm pixels](https://www.ovt.com/products/ov9281/):
fx = fy ≈ 933.33 pixels, principal
point (639.5, 399.5), and a 120 mm baseline. These are estimates until a
successful calibration. The viewer uses read-back intrinsics including the
FPGA datapath's fixed-point quantization for pose backprojection.
Live Etherbone control requires the ABI3 ordered-record bridge. Each management
write shares a record with a paired-control identity read, so the helper waits
for write completion before sending the next command. Uncertain mutating
commands are sent once rather than replayed.

**Stop both** stops a capture started by this window. Closing the window also
stops a capture it owns; an externally started capture stays running. The app
never flashes or reconfigures the FPGA. Startup and retained hardware faults
are shown in the toolbar. See the [camera supervisor guide](../../../gateware/docs/integration/camera-supervisor.md)
for the control interface and capture validation.

Another receiver cannot own the same local ports. The viewer binds both ports
before starting its workers and fails clearly if either is occupied; it does
not use socket reuse to divide incoming packets between applications. Close
the other host receiver, or use distinct ports supplied by an existing relay.
Changing viewer ports alone does not change an externally configured stream.
**Start both** configures the FPGA image destinations to the selected local
interface and ports; an unspecified bind uses the host route to the selected board.

## Controls

- **Start both / Stop both:** control both sensors and the paired FPGA capture
  together. Starting an already owned/busy pair is refused. Stop also cancels
  startup; controls run in a separate process so the window stays responsive.
- **Auto exposure / Target brightness:** enabled by default with target **100**
  on the 0–255 grayscale scale, adjustable from **40 to 160**. Both camera
  meters drive one shared sensor exposure/gain setting. Unchecking holds the
  current setting; an already pending update may still finish.
- **Pause view / Live:** pause the Rerun timeline or return to following the
  newest images. Capture, UDP reception, decoding, and history logging continue
  while the view is paused. Use Rerun's timeline to inspect retained history,
  and its image controls to zoom and pan.
- **Stack cameras:** switch the two image views between vertical and
  side-by-side layouts (camera 1 first in either layout).
- **Save latest PNGs:** save the latest received images as original decoded
  8-bit grayscale PNGs, vertically flipped to match the display. Filenames include a timestamp, camera number, and frame
  ID. This saves the latest receiver frames even when the timeline is paused or
  showing an older frame; it does not export the currently scrubbed history.
- **Resync:** forget local stream ordering after a quick FPGA or capture
  restart. This resets the host receiver state for both cameras without sending
  commands to the board. It clears pending/latest images and the live Rerun
  views, while preserving cumulative counters and retained timeline history.
  A gap of at least two seconds also allows automatic stream restart detection.

## LFTag layers and stereo calibration

**Start both** enables image, raw centroid and accepted-pose UDP products on
both camera ports. Each Rerun view has independently visible **centroids** and
**lftags** layers. Centroids are measured FPGA candidate points, including
candidates the decoder rejects; LFTag outlines, labels and canonical points
come from accepted poses. Their 2D projections use the exact horizontal and
vertical focal lengths; the original 3D geometry is logged separately for
inspection. This avoids Rerun 0.34's unequal-focal-length projection limitation. Layers match the full camera/frame ID, clear on the next image,
and allow late packets without copying detections onto a different frame.
Use the eye icons in Rerun's blueprint panel to hide either layer.
Enable **Tag outlines only** to show plain tag borders without IDs, warning
labels, or canonical pose points. This updates the current matched overlay and
future live frames; raw detections and the independent centroid layer remain
available. Previously logged history keeps its recorded presentation.
The `conditioned` suffix warns about pose geometry, not tag-ID acceptance.
The viewer computes it from the transmitted pose-column norm ratio (below
7/8), correcting older gateware's coarse warning calculation. Raw metadata
retains both the original bit and the recomputed result.
Displayed tags are deduplicated within each camera/frame using their projected
quadrilaterals. At intersection-over-union of at least 0.5, the candidate with
the lowest size-normalized decode error wins; equal errors prefer RAW, then
LL2, then LL4. This lets a reliable coarse detection replace a conflicting
fine-resolution ID. Separate tags remain visible, and every original detection
remains in raw metadata.
Calibration independently resolves repeated same-ID measurements in favor of
the finest scale, while rejecting ambiguous spatially distinct same-ID targets.

Set **Duration** in seconds, then click **Calibrate cameras** while both cameras
are capturing. The default is 60 seconds; click the number to type any whole
number from 1 to 86400. `--calibration-seconds N` sets the initial value at launch.
The duration is fixed for each recording, shown in its progress bar, and saved
in its manifest. Keep the
camera rig fixed with a **120 mm optical-center baseline**. Move and tilt one
flat printed LFTag through the shared field of view, at several distances and
across both image axes. Include the image edges. Multiple tags are supported
if they have **unique IDs and identical printed outer widths**. Two physical
tags with the same ID can be falsely paired when each camera sees a different
copy, so do not use duplicate IDs during calibration. A stationary tag or
insufficient stereo overlap cannot provide a reliable calibration.

The button records for the selected duration on a background worker. Each uniquely named
`--output/calibration-TIMESTAMP/` directory contains:

- `camera0.udp`, `camera1.udp`: all received image, centroid and pose datagrams,
  preserved byte for byte, with per-packet host timestamps. This preserves the
  compressed camera stream; it does not undo camera-codec loss or network loss.
- `centroids.jsonl`: every validated raw centroid packet, with Q5 coordinates,
  validity bits, group order, camera, frame and sequence numbers.
- `detections.jsonl`: accepted FPGA pose records and their metadata.
- `calibration.rrd`: all raw centroid/pose packets plus final centroid and LFTag
  layers for Rerun review. Normal processing does not transcode camera images;
  their original compressed packets remain in `.udp`.
- `observations.jsonl` and extraction diagnostics: host geometric decoding of
  measured FPGA centroid groups, in original unflipped source-pixel coordinates.
- `manifest.json`, processing logs and, only after a successful fit,
  `calibration.json`.

CDF centroid coordinates use the frontend's actual sample centers: RAW `(x,y)`,
LL2 `(4x,4y)`, and hybrid LL4 `(16x,16y+4)`. Observations identify this as
`source_geometry: cdf_even_ll4_y4_v1`. FPGA localization and host calibration
use these same source-pixel coordinates.

Processing first decodes only the selected calibration pairs (at most 120),
then fits their measurements. A successful fit becomes available to apply
immediately, while the worker saves the centroid/pose Rerun archive. It uses
measured FPGA centroids, **not pose backprojections**, to fit both cameras'
intrinsics, radial/tangential distortion and stereo rotation/translation.
The baseline fixes metric scale and allows the common tag width to be inferred.
No full-rate JPEG or preview-video encoding is required for calibration.
The fit checks image coverage, tilt diversity, convergence, physical bounds
and held-out cross-camera reprojection. Inadequate data leaves the current
calibration intact, while preserving the recording for inspection.
The bounded fitting subset preserves stereo image position, tag size and time
diversity, so scenes with many tags do not crowd out useful edge observations.
Failure messages show the specific coverage or uncertainty limit that failed.

**Apply calibration & restart both** installs a successful result after
stopping both cameras cleanly. The FPGA currently applies only pinhole
intrinsics; the full distortion and stereo fit is saved, but live images and
FPGA poses are not distortion-corrected. FPGA translation remains in normalized
tag-width units. To reuse a saved result on a later launch, pass
`--calibration PATH/TO/calibration.json`.

Build both executables before using the calibration button:

```bash
cargo build --locked -p lutcam-viewer --release --bins
```

Offline processing uses `python3` from `PATH` with NumPy and SciPy, and `ffmpeg` for
review clips. To reprocess a saved packet recording:

```bash
../target/release/lutcam-calibrate --recording captures/live-view/calibration-TIMESTAMP --fit-only
../target/release/lutcam-calibrate --recording captures/live-view/calibration-TIMESTAMP --metadata-only
```

To export all recorded images into the Rerun archive and generate 2 fps FFV1
review clips **on demand**, run:

```bash
../target/release/lutcam-calibrate --recording captures/live-view/calibration-TIMESTAMP --archive-only
```

This replaces `calibration.rrd` with the full image/metadata archive and creates
`camera0.mkv` and `camera1.mkv`. It preserves the raw clips, fitted calibration,
and measured observations. JPEGs and review videos are display derivatives;
missing preview samples are reported as repeated or blank frames. Full export
can take several minutes and is independent of fitting. The CLI without a
processing-mode flag retains the original full-export-then-fit behavior.

Closing, cancelling or resyncing during recording preserves a partial recording
and does not fit/apply it. A full local writer queue or disk error is reported
as incomplete. Network gaps and malformed packets remain visible in diagnostics.
The writer serializes centroid records directly from parsed tuples into a
1 MiB buffered output, avoiding temporary JSON objects and per-field allocations
on the recording path. All centroid tuples and the existing file formats are
preserved; the bounded queue remains 8192 packets / 32 MiB.

## Linked auto exposure

Auto exposure changes the sensors' integration time and analog gain. It runs
only for capture owned by this window after **Start both**; viewing an external
stream or using headless mode does not change sensor settings. Metering uses
fresh decoded receiver images, so pausing or scrubbing the Rerun timeline does
not freeze feedback.

Each meter samples the central 80% of its image and trims the darkest and
brightest 5% of samples. The controller averages the two meter readings and
applies identical exposure and gain to both sensors. It prefers longer exposure
before adding gain, uses an 8% brightness deadband, and limits each adjustment
to half an exposure stop. The two cameras can still have different brightness
when they see different scenes.

Metering and updates run at most **twice per second**. After an acknowledged
setting change, feedback waits at least **200 ms** and requires a fresh image
from each camera received after that settling interval. Only one setting
request can be pending. Exposure, gain, frame timing, and sensor synchronization
are separate: these updates leave HTS, VTS, and the FPGA FSYNC generator
unchanged. The sensors receive sequential group launches, so linked settings
do not promise that a change takes effect on the same physical frame.

Exposure errors are shown in the UI and disable auto adjustment while image
capture continues. Re-enable the checkbox to explicitly retry a rejected
command. An uncertain sensor write disables exposure changes for that capture
session; restart the owned capture before trying again. Turning auto exposure
off does not restore the startup settings.

## Command line

```bash
cargo run --locked -p lutcam-viewer --release -- --help

# Bind to the camera-facing interface explicitly.
cargo run --locked -p lutcam-viewer --release -- --bind 192.168.1.251

# Receive an existing local relay on different ports.
cargo run --locked -p lutcam-viewer --release -- \
  --bind 127.0.0.1 --source 127.0.0.1 --port0 6100 --port1 6103

# Receive and decode without opening a window.
cargo run --locked -p lutcam-viewer --release -- \
  --headless-seconds 10 --output captures/live-check
```

| Option | Default | Meaning |
|---|---|---|
| `--bind IP` | `0.0.0.0` | Local IPv4 interface |
| `--source IP\|any` | Selected board | Accepted source IPv4 address, or disable source filtering |
| `--board IP` | Automatic discovery | Override the capture-control destination and default source filter |
| `--port0 PORT` | `6000` | Camera 0 image UDP port |
| `--port1 PORT` | `6003` | Camera 1 image UDP port |
| `--output DIR` | `captures/live-view` | Snapshot, calibration or headless output directory |
| `--calibration PATH` | Lens estimate | Fitted intrinsics to load on Start both |
| `--calibration-seconds N` | `60` | Initial calibration recording duration, 1–86400 whole seconds; editable in the GUI |
| `--headless-seconds N` | GUI mode | Receive for 0.1–86,400 seconds, write results, and exit |

Camera ports must be distinct and nonzero. Headless mode writes `camera0.png`,
`camera1.png` when frames are available, and `report.json`, replacing those
filenames in the chosen output directory. It succeeds only if both cameras
have delivered a decoded frame within the last two seconds. The report includes
stream counters and decode timings; a successful exit is a recent-image check,
not a lossless-capture qualification.
Headless mode only receives images; it does not start capture. Capture control
requires a physical network interface: a loopback bind/relay is receive-only.

## Stream handling and metrics

The parser accepts the current LUTCam packet format, with a zero reserved
header byte and compact image payloads. The next frame makes its contiguous
predecessor a decode candidate, and the compact LWC decoder establishes the exact endpoint. A receive
timeout also tries the final frame when traffic stops. Truncated candidates
cannot be displayed; a reordered tail can complete them later.

Each camera retains a three-frame assembly window, with at most 128,000 payload
bytes per frame plus fragment metadata. Identical duplicates are harmless;
overlap, changed duplicate data, and inconsistent adjacent
packet sequences invalidate the affected frame. Frame and packet counter wrap
are supported. Old frames are discarded as newer ones arrive.

Each camera has a separate receive worker, decode worker, and one pending decode
slot. A newer image replaces queued work when decoding cannot keep up. This
keeps latency and memory bounded; the application is a live preview rather than
a recording queue. In live mode, the two images are independently selected
latest frames, not a matched stereo pair.

Rerun logging samples the latest decoded image at most once per 16 ms UI update;
decoding continues independently at the incoming camera rate. Consequently,
120 fps capture does not produce 120 logged history images per camera per
second. A dedicated logger has one replaceable latest-image slot per camera,
an **8 MiB** SDK in-flight command limit, and a **128 MiB** log channel. SDK
backpressure stays off the GUI and UDP workers. Each camera snapshot submits
its image and changed metadata together, and the logger explicitly flushes
complete image/overlay updates. This avoids independent SDK timer flushes
exposing an image before its available outlines; Rerun entities are still
separate, so this is not a rendering transaction.

Pose and centroid caches each retain up to 64 frames per camera, expiring after
two seconds. The 128-pose and 256-centroid-group limits apply to each frame;
dense scenes therefore retain the same frame window as sparse scenes. Images
and overlays still join only by the exact camera/frame ID.

The embedded viewer has a **256 MiB memory target** and evicts old recording
data as needed. History duration therefore depends on traffic and available
memory; pausing does not preserve the entire session indefinitely. This target
is not a strict process RSS cap, and GPU/driver allocations are separate. The
`capture_time` timeline uses host receipt time, survives FPGA frame-ID restarts,
and does not measure sensor exposure time.

- **fps** measures host decode throughput, not sensor exposure cadence or GUI
  refresh rate.
- **MiB/s** measures accepted-source UDP datagram bytes at the host, including
  LUTCam headers, not Ethernet wire bandwidth.
- **Frame age** starts when an assembled candidate is submitted for decoding;
  it is not exposure-to-display latency.
- **Skipped** counts replaced or stale decode candidates. **Incomplete frames**
  counts assembly-window evictions with holes or no emitted candidate. Neither
  is a complete camera-frame-loss counter.
- **Codec errors** counts decode failures other than a truncated speculative
  version-3 candidate. **Incomplete preview attempts** counts those truncated
  candidates; a later tail may still complete the same frame.
- Receiver status describes image availability and transport/decode checks.
  When this app owns capture, the separate control helper also reports retained
  FPGA diagnostics. Neither stream statistics nor timeline alignment establishes
  exposure synchronization or LFTag pose quality.

## Tests

```bash
cargo test --locked -p lutcam-viewer --release
cargo clippy --locked -p lutcam-viewer --all-targets -- -D warnings
```

Tests cover malformed headers, fragment reordering, duplicates and conflicts,
counter wrap, late-tail recovery, final-frame polling, bounded frame history,
current packet boundaries, and a textured image through the real LWC codec.
Receiver tests use local UDP sockets to exercise source filtering, binding,
worker lifecycle, and real decoding without FPGA access.
