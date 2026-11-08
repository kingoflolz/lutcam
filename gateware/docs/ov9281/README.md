# CSI-2 receiver

The `csi2_rx` core is specialized for the OV9281. It is a fixed-function bridge
between Titanium MIPI HSIO receive lanes and the camera processing pipeline.
It accepts the sensor's two-lane, 800 Mb/s-per-lane RAW10 mode at 1280 × 800
and produces two ordered
10-bit monochrome pixels per output beat. Fixing the geometry, data type,
virtual channel, and lane count keeps the protocol machinery small and avoids
the buffering and control registers needed by a general-purpose CSI-2 core.

The hard HSIO blocks perform the electrical receive and x8 deserialization.
The soft receiver controls high-speed termination and enable sequencing,
continuously drains each hard lane FIFO, finds the D-PHY synchronization byte,
aligns the two striped byte streams, validates CSI-2 packets, unpacks RAW10,
and transfers pixels into the independent output clock domain.

## Processing architecture

The receive path is organized as a streaming pipeline:

1. Control logic qualifies low-power lane states and acquires the continuous
   clock lane. A recovered-clock heartbeat distinguishes an intentionally idle
   sensor from a stopped link.
2. Each data lane searches all eight possible bit phases for the D-PHY `0xB8`
   synchronization byte. An eight-entry queue per lane supports a conservative
   seven-byte aligned-lane skew and temporary hard-FIFO bubbles.
3. The paired byte stream enters a fixed CSI-2 parser. Header ECC can correct a
   single protected-bit error, and every long-packet payload is checked with
   the CSI-2 CRC-16.
4. Only virtual-channel-zero RAW10 lines with a 1600-byte payload contribute
   pixels. A specialized five-cycle gearbox converts every ten payload bytes
   into eight pixels, emitted as four 20-bit beats.
5. A dual-clock pixel FIFO crosses into the output domain. The reusable RAW10
   profile generates markers with output-domain geometry counters. A compact
   production profile, used when only the eight most-significant bits of each
   sample are consumed, carries the four marker bits in the otherwise discarded
   sample bits and removes the duplicate counters.

The expected packet sequence is Frame Start, 800 RAW10 line packets, and a
matching Frame End. Line Start, Line End, embedded data, alternate virtual
channels, and other image formats are deliberately unsupported.

## Clocking and rate adaptation

The design has three independent clock domains. The always-running control
clock owns link sequencing and status. The recovered byte clock owns lane
draining, packet validation, and RAW10 conversion. The continuously running
pixel clock owns output presentation and image markers. Reset asserts
asynchronously and releases through local clock-domain synchronization.

At the nominal link rate, the two lanes deliver two bytes on every 100 MHz byte
clock. A 1600-byte line payload therefore occupies 800 byte-clock cycles. The
gearbox writes four pixel beats per five payload cycles, an active-line average
of 80 million beats/s. At 120 frames/s, 512,000 beats per frame produce a frame
average of 61.44 million beats/s. A 100 MHz or faster output clock consequently
has headroom both during active lines and over the complete frame, although
`pixel_valid` naturally contains bubbles.

There is no output backpressure. The output clock must remain active while the
sensor streams; buffering is intended to absorb clock phase and small frequency
differences, not downstream stalls.

The Ti60 TagDetect/LWC target runs the output and processing domain at 120 MHz.
It feeds valid receiver beats directly into the shared frontend; a one- or
two-row buffer cannot change the camera's lower average row rate and is not
needed for correctness. TagDetect treats 896 clocks as a minimum row-service
epoch, while LWC replays completed coefficient groups from a fixed canonical
schedule so source bubbles cannot change the encoded format.

## Buffering decisions

The receiver avoids frame and line storage:

- The two lane-alignment queues use eight-entry synchronous-read memories.
  Empty-queue bypass retains a fall-through interface while leaving their
  implementation as registers or small RAMs to synthesis.
- The pixel clock crossing is a configurable true dual-clock FIFO. The
  reusable receiver defaults to 512 × 20. The Ti60 camera target uses 64 × 20
  because its gearbox writes no faster than 80 million beats/s and its pixel
  domain drains at 120 million beats/s. Sixty-four entries still provide more
  than half a microsecond of startup and phase margin while narrowing every
  binary pointer, Gray pointer, and pointer synchronizer. Its packed-marker
  profile does not change this width or depth: two eight-bit samples and four
  marker bits still occupy exactly 20 bits.
- Pixels are released before the line CRC is known. A CRC failure is therefore
  diagnostic and cannot retract data already observed by the consumer. Making
  lines transactional would require an intentionally omitted line buffer.

The hard HSIO FIFOs are treated only as phase elasticity. They are drained
whenever nonempty, including during synchronization search, malformed packets,
trail, and recovery. Protocol backpressure must never reach those FIFOs.

### Packed-marker FIFO occupancy proof

Packing markers changes only the 20-bit payload, not the pixel FIFO's queue
semantics. Let `Q` be accepted entries, `W` an accepted gearbox write, and `R`
an accepted pixel read. Both profiles obey the same recurrence
`Q(next) = Q + W - R`, with `0 <= Q <= 64`: write enable, full detection,
read-valid, read-ready, pointers, and capacity are unchanged.

The first marker is attached to schedule zero's phase-two write, and the last
marker to the footer write. Those are exactly the beats at which the original
output counters were `(0, line)` and `(639, line)`. The existing line counter
supplies the same line index before it is incremented at the footer. Therefore
the EOF-driven read-state transition occurs on the same dequeue for every
well-formed frame. Lane bubbles, aborts, and truncated packets retain their
original write and drain controls, so their occupancy traces are identical.

For adversarial byte contents and lane bubbles, the parser can still issue no
more than four pixel writes per five 100 MHz byte clocks, with at most four
consecutive writes at a schedule/footer boundary. Once a frame is armed, the
120 MHz pixel side offers six read clocks over the same interval and never
backpressures. Start-event and Gray-pointer synchronization plus the registered
RAM read take fewer than eight pixel clocks; even pessimistically charging one
write per byte clock during that entire transient gives eight entries. Adding
the maximum four-write burst gives `Q_peak <= 12 < C=64`. Malformed headers or
payload truncation can only remove writes, and abort handling keeps reads
enabled while draining. Thus no admissible input sequence can reach full; the
unchanged full guard remains a final protection against an out-of-contract
clock ratio and cannot overwrite an occupied entry.

## Error and recovery policy

A corrected header continues normally. Payload CRC errors are recorded while
the frame continues because the affected pixels may already be visible.
Errors that make byte count, alignment, or frame ownership unreliable suppress
the remainder of the current frame while physical lane draining continues.

Frame aborts use an explicit cross-domain handshake. The byte side stops pixel
writes, the pixel side suppresses output and drains stale FIFO entries, and a
new frame is admitted only after the pixel side reports idle. This prevents an
aborted frame from being mixed with a later valid frame without resetting one
side of the asynchronous FIFO independently.

Sticky status distinguishes corrected headers, uncorrectable headers, CRC and
packet errors, synchronization and lane-alignment failures, illegal low-power
sequences, frame or line sequence errors, pixel FIFO overflow, and recovered
clock failure. Status clearing is also acknowledged across the active source
clock domains so short events are not lost.

## RTL modules

`csi2_rx` coordinates initialization, low-power/high-speed transitions,
packet parsing, ECC and CRC validation, RAW10 conversion, frame recovery, and
output marker generation. The fixed parser decodes the six-bit header syndrome
directly and implements the two-byte CRC update as its exact Boolean transform.
Mutually exclusive header, payload-history, LP-timing, and line-schedule state
share storage and arithmetic without changing byte-pair consumption timing.

`Ov9281LaneAligner` is instantiated once per data lane. It performs per-burst
bit-phase discovery, consumes the synchronization byte, and presents aligned
bytes through the bounded skew queue.

`Ov9281PixelAsyncFifo` implements the Gray-pointer clock crossing and a
synchronous-read, ready/valid output path. Its storage is not reset; global
reset initializes only the control state and pointers in both domains.

## Resource intent and integration assumptions

The area target is below 2,000 XLRs, fewer than five RAM10 blocks, and no DSPs.
The intended steady-state implementation uses one RAM10 for pixel CDC, with
the small lane queues eligible for RAM inference. Timing-counter widths follow
their compile-time bounds instead of fixed container widths. Synthesis must
confirm the configured memory mapping rather than relying on source-array
dimensions alone.

In the July 20, 2026 unchanged-option unified Ti60 placement, the packed-marker
receiver hierarchy uses 1,077 XLRs, three RAM10s, and no DSPs.

The fixed architecture assumes a continuous sensor clock lane and one CSI-2
packet per data-lane high-speed burst. Board integration must preserve lane
ordering, route termination and receiver-enable controls independently, and
validate static input delay, lane skew, burst boundaries, and the recovered
100 MHz byte clock using the final sensor configuration. The smaller Ti60
pixel FIFO does not specialize bit phase: both production lanes still search
all eight alignments independently at the start of every burst.
