# Project-independent OV9281 fixed-function CSI-2 receiver for Titanium HSIO

Design specification, not an implementation
Deployment project: **Unspecified; this is a standalone IP contract**
Input-side reference inspected: **Ti60F225_dsi**
Target sensor: **OmniVision OV9281**

## 1. Scope and fixed design decisions

This document specifies a future soft CSI-2 receive core named **csi2_rx**. The core is specialized for one known camera mode and sits immediately above Efinix Titanium hard HSIO receive lanes. It implements the D-PHY/CSI-2 and pixel functions that the raw hard lanes do not provide, but it does not replace the hard HSIO deserializers at the device edge.

The design is **project-generic, not sensor- or protocol-generic**. It can be instantiated in another project that supplies the documented clocks and an equivalent Titanium HSIO boundary. It has no dependency on the clocks, resets, camera pipeline, DMA, display path, top-level names, or image geometry of **Ti60F225_dsi**. That project was inspected only to establish the input-facing hard-HSIO signal set, generated-lane configuration, FIFO handshake, and timing arcs.

This document intentionally contains no RTL, constraints, firmware changes, or project integration changes.

The fixed contract is:

| Property | Fixed value |
|---|---:|
| Sensor | OV9281 monochrome global-shutter sensor |
| Active image | 1280 × 800 |
| Frame rate | Nominal 120 frames/s mode; cadence is not measured by the receiver |
| CSI-2 virtual channel | VC 0 |
| CSI-2 long-packet data type | RAW10, DT = 0x2B |
| Number of data lanes | 2 |
| Lane rate | 800 Mb/s per lane |
| Sensor EXTCLK | 24 MHz |
| Recovered x8 byte clock | 100 MHz nominal |
| Control clock | 100 MHz, always running |
| Pixel clock | 100 MHz, independent phase/domain |
| Output format | Two 10-bit monochrome pixels per 20-bit beat |
| Output backpressure | None |
| Clock-lane mode | Continuous/free-running while the sensor streams |
| Expected line packets | One 1600-byte RAW10 long packet per active line |
| Expected short packets | Frame Start and Frame End only |

The fixed image dimensions, word count, virtual channel, data type, lane count, and clocks are compile-time architectural constants. They are not registers and are not runtime configurable.

The design does not implement an AXI control interface, an interrupt controller, format selection, virtual-channel routing, or arbitrary image geometry. It exposes a clearable sticky status vector instead.

## 2. What the CSI-2 receive core does

The Titanium HSIO blocks provide electrical receive functions, x8 high-speed deserialization, raw low-power lane states, and a small per-lane elastic FIFO. They do **not** implement the D-PHY receive state machine, Start-of-Transmission synchronization, CSI-2 packet parsing, header ECC, payload CRC, RAW10 unpacking, frame/line tracking, or pixel clock-domain crossing.

The soft core therefore performs the following functions:

1. Observes clock-lane and data-lane LP states and controls the hard receivers and terminations.
2. Drains the two hard HSIO FIFOs without ever reading an empty FIFO.
3. Finds the D-PHY 0xB8 synchronization byte independently on each lane and removes bit-slip.
4. Absorbs bounded inter-lane skew and reconstructs the original CSI-2 byte order.
5. Decodes CSI-2 short and long packet headers.
6. Corrects a single-bit header/ECC error and rejects an uncorrectable header.
7. Checks the CRC-16 of every long-packet payload.
8. Accepts only the fixed OV9281 VC0/RAW10/1600-byte image stream.
9. Unpacks RAW10 into two 10-bit pixels per output beat.
10. Crosses pixel data from the recovered byte-clock domain into the independent 100 MHz pixel domain.
11. Generates deterministic frame and line markers from the fixed geometry.
12. Reports sticky diagnostic and protocol-error status.

The core is deliberately still a real D-PHY/CSI-2 receiver. Fixed sensor settings remove generality, but they do not make bit alignment, lane alignment, ECC, CRC, packet length checks, hard-FIFO handling, or clock-domain crossing optional.

## 3. Reference inspection of the camera edge IO and HSIO blocks

In the inspected reference project, the camera lanes are instantiated by Efinity Interface Designer in:

**Ti60F225_dsi/edge_vision_soc.peri.xml**

The three observed blocks are:

| Interface Designer name | Titanium block | Mode | Hard FIFO | Static input delay | Fabric clock connection |
|---|---|---|---:|---:|---|
| cam_d0 | GPIOR_PN_00 | RX data lane 0 | Enabled | 5 taps | GCLK |
| cam_d1 | GPIOR_PN_01 | RX data lane 1 | Enabled | 5 taps | GCLK |
| cam_ck | GPIOR_PN_02 | RX clock lane | Disabled | 0 taps | GCLK |

These exact Interface Designer names, **GPIOR_PN** indices, and delay taps are not requirements of the standalone core. A destination project may place an equivalent clock lane and two neighboring data lanes in another compatible Titanium MIPI HSIO group. The destination periphery must still expose the signal classes in Section 4 and arrange the dedicated clock connections required by the device.

The observed data-lane delay setting of 5 is approximately 125 ps when using the documented nominal 25 ps/tap value. It is a reference-board starting point, not a portable setting or a guaranteed calibrated value. The destination board must determine its own static delay.

Interface Designer automatically connects the clock lane's dedicated fast/slow clocks and the neighboring data-lane FIFO clocks inside the MIPI group. Those dedicated connections are not fabric ports and must not be recreated in RTL.

### 3.1 Fabric-visible clock-lane signals

The configured clock HSIO exposes:

| Observed reference signal | Direction into soft core | Purpose |
|---|---|---|
| i_cam_ck_LP_P_IN | Input | Raw LP positive state |
| i_cam_ck_LP_N_IN | Input | Raw LP negative state |
| i_cam_ck_CLKOUT_0 | Input | Divided/recovered global byte clock |
| o_cam_ck_HS_TERM | Output | Differential termination control |
| o_cam_ck_HS_ENA | Output | High-speed differential receiver enable |

The clock-lane high-speed data bus is not needed. The useful clock-lane payload is **i_cam_ck_CLKOUT_0**, which is the recovered/divided clock used by both the soft byte-domain logic and the hard data-lane FIFOs.

### 3.2 Fabric-visible data-lane signals

Each configured data HSIO exposes:

| Observed reference signal pattern | Direction into soft core | Purpose |
|---|---|---|
| i_cam_dN_LP_P_IN | Input | Raw LP positive state |
| i_cam_dN_LP_N_IN | Input | Raw LP negative state |
| i_cam_dN_HS_IN_0[7:0] | Input | One x8 deserialized high-speed word |
| i_cam_dN_FIFO_EMPTY | Input | Hard receive FIFO empty indication |
| o_cam_dN_FIFO_RD | Output | Consume the current hard-FIFO word |
| o_cam_dN_RST | Output | Active-high asynchronous hard receive-lane/deserializer/FIFO reset |
| o_cam_dN_HS_TERM | Output | Differential termination control |
| o_cam_dN_HS_ENA | Output | High-speed differential receiver enable |

Here and below, **N** is 0 or 1.

The inspected Interface Designer configuration generates only **HS_IN_0[7:0]** for each lane. Declarations named **i_cam_d0_HS_IN_1** through **_3** and **i_cam_d1_HS_IN_1** through **_3** in the reference top-level Verilog do not correspond to generated HSIO buses. Its 32-bit concatenations are assigned to 8-bit wires, so Verilog truncates them back to **HS_IN_0**. The portable interface consequently requires exactly one real x8 input bus per data lane; it does not inherit those reference-wrapper artifacts.

The LP output, LP output-enable, dynamic delay, and per-data-lane CLKOUT ports in the generated periphery are not needed by this unidirectional receiver. Dynamic delay is intentionally not part of the soft-core interface because delay remains a static Interface Designer setting.

## 4. Proposed soft-core boundary

The soft core directly consumes the fabric side of the hard HSIO blocks. There must not be an external register pipeline between **HS_IN/FIFO_EMPTY** and the hard-FIFO reader. Any synchronization or pipelining needed by the protocol logic belongs inside the core and in the appropriate clock domain.

### 4.1 Clock, reset, and control interface

| Signal | Direction | Width | Domain | Contract |
|---|---|---:|---|---|
| reset_n | Input | 1 | Asynchronous assertion | Active-low core reset; deasserted synchronously in each active domain |
| ctrl_clk | Input | 1 | — | Always-running 100 MHz control clock |
| byte_clk | Input | 1 | — | Recovered nominal 100 MHz clock from clock HSIO CLKOUT |
| pixel_clk | Input | 1 | — | Independent 100 MHz output clock |
| status_clear | Input | 1 | ctrl_clk | One-cycle pulse that launches the cross-domain sticky-status clear handshake; a coincident error wins |
| init_done | Output | 1 | ctrl_clk | High when control-side initialization is complete and firmware may start the sensor; it does not imply that the recovered clock is already present |
| status[15:0] | Output | 16 | ctrl_clk | Sticky status described in Section 13 |

The three 100 MHz clocks are not assumed phase related. **ctrl_clk** must remain available while the sensor clock lane is stopped. **byte_clk** may be absent before the sensor starts or after a fault. **pixel_clk** must run continuously whenever output is expected.

### 4.2 Clock-HSIO interface

| Signal | Direction | Width | Domain | Contract |
|---|---|---:|---|---|
| clk_lp_p | Input | 1 | Asynchronous to ctrl_clk | Raw clock-lane LP P state |
| clk_lp_n | Input | 1 | Asynchronous to ctrl_clk | Raw clock-lane LP N state |
| clk_hs_term | Output | 1 | ctrl_clk control | Drives hard clock-lane HS_TERM |
| clk_hs_ena | Output | 1 | ctrl_clk control | Drives hard clock-lane HS_ENA |

The LP inputs require ordinary multi-flop synchronization into **ctrl_clk** before state-machine use. **HS_TERM** and **HS_ENA** are separate controls with separate meanings and must remain separate all the way to the edge block.

### 4.3 Data-HSIO interface

| Signal | Direction | Width | Domain | Contract |
|---|---|---:|---|---|
| data_lp_p[1:0] | Input | 2 | Asynchronous to ctrl_clk | Raw LP P states; bit 0 is lane 0 |
| data_lp_n[1:0] | Input | 2 | Asynchronous to ctrl_clk | Raw LP N states; bit 0 is lane 0 |
| data0_hs_in[7:0] | Input | 8 | byte_clk | Current show-ahead hard-FIFO word for lane 0 |
| data1_hs_in[7:0] | Input | 8 | byte_clk | Current show-ahead hard-FIFO word for lane 1 |
| data_fifo_empty[1:0] | Input | 2 | byte_clk | Per-lane hard-FIFO empty flags |
| data_fifo_rd[1:0] | Output | 2 | byte_clk | Per-lane consume strobes; never asserted while the corresponding FIFO is empty |
| data_rst[1:0] | Output | 2 | Reset/control | Active-high asynchronous resets for the two hard receive-lane deserializers and FIFOs |
| data_hs_term[1:0] | Output | 2 | ctrl_clk control | Per-lane hard termination enables |
| data_hs_ena[1:0] | Output | 2 | ctrl_clk control | Per-lane hard high-speed receiver enables |

The two lanes are read independently until both have found 0xB8 and entered the aligned-byte queues. They are then consumed as a pair. A temporary empty indication on one lane must not cause a read on that lane or make the other lane stop draining into its soft queue.

### 4.4 Pixel-stream interface

| Signal | Direction | Width | Domain | Contract |
|---|---|---:|---|---|
| pixel_data[19:0] | Output | 20 | pixel_clk | Two unsigned RAW10 monochrome pixels |
| pixel_valid | Output | 1 | pixel_clk | Qualifies pixel_data and every marker |
| pixel_sof | Output | 1 | pixel_clk | One valid cycle on the first beat of a frame |
| pixel_sol | Output | 1 | pixel_clk | One valid cycle on the first beat of each line |
| pixel_eol | Output | 1 | pixel_clk | One valid cycle on the last beat of each line |
| pixel_eof | Output | 1 | pixel_clk | One valid cycle on the last beat of the 800th line |

Pixel ordering is fixed:

- **pixel_data[9:0]** is the earlier, left-hand pixel.
- **pixel_data[19:10]** is the immediately following, right-hand pixel.

There is no **ready** input. **pixel_valid** may contain bubbles. Data and markers change only in the normal registered output sense; consumers must sample them only when **pixel_valid** is high.

### 4.5 Input-side correspondence observed in the reference project

This table is evidence for the portable boundary, not a required destination-project naming scheme:

| Proposed core signal | Observed Ti60F225_dsi HSIO-side signal |
|---|---|
| byte_clk | i_cam_ck_CLKOUT_0 |
| clk_lp_p / clk_lp_n | i_cam_ck_LP_P_IN / i_cam_ck_LP_N_IN |
| clk_hs_term / clk_hs_ena | o_cam_ck_HS_TERM / o_cam_ck_HS_ENA |
| data_lp_p[0] / data_lp_n[0] | i_cam_d0_LP_P_IN / i_cam_d0_LP_N_IN |
| data0_hs_in | i_cam_d0_HS_IN_0 |
| data_fifo_empty[0] | i_cam_d0_FIFO_EMPTY |
| data_fifo_rd[0] | o_cam_d0_FIFO_RD |
| data_rst[0] | o_cam_d0_RST |
| data_hs_term[0] / data_hs_ena[0] | o_cam_d0_HS_TERM / o_cam_d0_HS_ENA |
| data_lp_p[1] / data_lp_n[1] | i_cam_d1_LP_P_IN / i_cam_d1_LP_N_IN |
| data1_hs_in | i_cam_d1_HS_IN_0 |
| data_fifo_empty[1] | i_cam_d1_FIFO_EMPTY |
| data_fifo_rd[1] | o_cam_d1_FIFO_RD |
| data_rst[1] | o_cam_d1_RST |
| data_hs_term[1] / data_hs_ena[1] | o_cam_d1_HS_TERM / o_cam_d1_HS_ENA |

The destination project independently supplies **ctrl_clk**, **pixel_clk**, and **reset_n** according to Section 4.1. No reference-project clock or reset name is part of the IP contract.

The two-stage byte-clock pipeline at **edge_vision_soc.v:661-683** is a reference-wrapper detail, not part of the hard-HSIO interface. The portable core connects to the generated hard-FIFO signals directly and owns their domain handling.

The reference assignments at **edge_vision_soc.v:688-693** drive physical **HS_TERM** from **HS_ENA**, although the generated IP has distinct termination outputs. The portable contract deliberately corrects that ambiguity: every destination project must route **hs_term** and **hs_ena** as independent signals.

## 5. HSIO receive semantics

This section is the mandatory contract between the soft core and the Titanium hard blocks.

### 5.1 Division of responsibility

The hard HSIO receiver provides:

- Differential HS input buffering and termination controls.
- Raw LP P/N input states.
- x8 deserialization of each data lane.
- A divided/recovered clock from the clock lane.
- A small asynchronous elastic FIFO per data lane.

The soft core provides everything above that boundary:

- LP/HS sequencing and timing policy.
- Hard receive-lane reset and hard-FIFO read policy.
- D-PHY synchronization and lane alignment.
- CSI-2 protocol and pixel processing.

The hard block is not a complete D-PHY receiver. In particular, a byte visible on **HS_IN** is not automatically a CSI-2-aligned byte.

### 5.2 Bit ordering

The official Titanium receive-lane simulation model describes x8 DDR deserialization with serial data arriving least-significant bit first. Once the soft aligner has located the 0xB8 sync byte, the aligned **HS_IN[7:0]** byte is used in its presented bit order. It must not be bit-reversed.

The sliding aligner, rather than a fixed assumption about the hard deserializer's initial byte phase, accounts for bit slips 0 through 7.

### 5.3 Hard-FIFO read rule

The official receive-lane model behaves as a show-ahead/FWFT FIFO:

- **HS_IN** presents the current head word.
- **FIFO_EMPTY = 0** means that head word is valid.
- On a **byte_clk** edge for which **FIFO_RD = 1**, the current head word is consumed.
- Advancing the FIFO while empty is illegal even though a simplified simulation model may still move an internal pointer.

Therefore, for each lane and each byte-clock edge:

1. If **FIFO_EMPTY is 0**, capture the current **HS_IN** word and assert **FIFO_RD** for that same transfer.
2. If **FIFO_EMPTY is 1**, do not consume **HS_IN** and keep **FIFO_RD** deasserted.
3. Never infer validity from **FIFO_RD** alone; validity is the nonempty observation associated with the transfer.

The read path must sustain one word per lane per **byte_clk** without inserting deliberate stalls. The reference SDC confirms explicit recovered-clock input/output timing arcs for **HS_IN**, **FIFO_EMPTY**, **FIFO_RD**, and **RST**. A destination project must generate and validate the equivalent device-specific HSIO constraints for its chosen blocks and a 10 ns nominal recovered byte clock; it must not copy reference pin paths blindly.

### 5.4 Why the hard FIFO must always be drained

The hard FIFO has no fabric-visible full indication or write enable. The official model writes a deserialized word on every slow/byte-clock event. Consequently, the core must continuously drain every nonempty lane FIFO, including:

- Before a valid 0xB8 has been found.
- Between CSI-2 packets.
- During malformed packets.
- While discarding the remainder of an aborted frame.
- During HS trail until the lane returns to stable LP11.

Stopping reads to apply parser backpressure is not allowed. If downstream soft storage fills, the core continues draining and discarding hard-FIFO words, records an error, and resynchronizes at a later legal packet boundary.

The hard FIFO is phase elasticity only. Its usable hardware depth is not documented in the local material and must not be used as a line buffer, packet buffer, or lane-deskew buffer.

### 5.5 Hard receive-lane reset

**data_rst[1:0]** is active high and asynchronously assertable. The Titanium lane reset covers both the x8 deserializer and its receive FIFO, not only the FIFO. It is asserted:

- During global reset.
- While the recovered clock is unavailable during initial clock-lane acquisition.
- Following a detected continuous-clock loss, before starting a new receive session.

It is not pulsed at every packet. Per-packet reset would destroy elasticity and can discard the beginning of the next burst.

Deassertion must be synchronized to actual **byte_clk** edges after the recovered clock is observed running. If the recovered clock stops, reset assertion cannot depend on another byte-clock edge. The soft byte-domain state and the hard receive lanes leave reset together as a clock-session operation, not a packet operation.

## 6. Clock and reset domains

The design has three independent domains.

### 6.1 Control domain

The always-running 100 MHz **ctrl_clk** domain contains:

- LP input synchronizers.
- The 100 µs initialization timer.
- Clock-lane and data-lane LP/HS state machines.
- Fixed D-PHY timing counters.
- Recovered-clock heartbeat monitoring.
- Sticky status collection and clear handling.

Asynchronous LP inputs are used only after synchronization and stability qualification. A raw LP transition never directly changes an HSIO output.

### 6.2 Recovered byte domain

The nominal 100 MHz **byte_clk** domain contains:

- Hard-FIFO read handshakes.
- Per-lane bit-slip/B8 aligners.
- Per-lane eight-byte soft queues.
- Lane merging and CSI-2 byte reconstruction.
- Header ECC and payload CRC.
- Packet, line, and frame validation.
- RAW10 unpacking.
- Pixel async-FIFO writes.

Control-domain acquire/abort indications cross into this domain through explicit synchronizers or toggles. Byte-domain error indications are made sticky at their source before being synchronized to **ctrl_clk**, so a one-cycle error pulse cannot be lost.

A four-bit free-running Gray heartbeat counter is clocked by **byte_clk** but reset only by the global **reset_n**, not by the byte-session reset. Global reset asserts it asynchronously and releases it synchronously once byte-clock edges exist. Its synchronized Gray value is compared in **ctrl_clk**. Any change proves that **byte_clk** is running. This avoids directly sampling a 100 MHz clock level with another nominally 100 MHz clock and also avoids a reset/heartbeat circular dependency.

### 6.3 Pixel domain

The independent 100 MHz **pixel_clk** domain contains:

- Async-FIFO reads.
- Fixed horizontal and vertical beat counters.
- SOF/SOL/EOL/EOF marker generation.

Counters advance only on **pixel_valid**. Clock equality does not remove the CDC requirement: byte and pixel clocks have unrelated phase and may have small frequency error.

### 6.4 Reset behavior

**reset_n** asserts all domain resets asynchronously. Each domain releases reset only after a local synchronizer has observed enough active clock edges. Because **byte_clk** can be absent, **init_done** must not wait on a byte-domain signal that can only change while that clock is stopped. The control domain first declares that it is safe for firmware to start the sensor. It then observes the sensor's legal clock-lane transition and a change in the synchronized Gray heartbeat before permitting byte-domain/hard-lane reset release. The heartbeat counter itself remains outside that session reset.

No pixel output is valid until a valid Frame Start event has crossed into the pixel domain and the first accepted pixel beat is readable.

## 7. Specialized D-PHY receive behavior

### 7.1 Fixed timing values

At 800 Mb/s per lane, one unit interval is 1.25 ns and the x8 recovered byte clock is 100 MHz. The future core uses these fixed 100 MHz receiver-side constants:

| Receiver function | Time | 100 MHz count | Basis and use |
|---|---:|---:|---|
| Initialization guard | 100 µs | 10,000 cycles | Conservative receiver startup assumption inherited from the reference input-side IP |
| LPX observation minimum | 50 ns | 5 cycles | Sequence validation only; never delays reaction to LP01 |
| Clock termination-to-HS guard | 38 ns | 4 cycles | Reference receiver assumption; not an OV9281 termination specification |
| Data termination-to-HS guard | 35 ns | 4 cycles | Reference receiver assumption; not stated in the OV9281 datasheet |
| Receiver SoT settle gate | 85 ns | 9 cycles | Early B8-acceptance inhibit after synchronized LP00 |
| B8 search timeout | 640 ns | 64 byte-clock cycles | Bounded synchronization failure detection after the settle gate |
| Stop-pipeline guard | 80 ns | 8 byte-clock cycles | Allows already captured hard/soft receive data to emerge after synchronized LP11 |

Counts are ceiling-rounded where necessary. They are fixed implementation assumptions, not runtime registers and not claims about the exact waveform emitted by the sensor. The destination design must validate the 38 ns, 35 ns, 85 ns, and eight-cycle guard against the applicable Efinix/D-PHY requirements and hardware.

The OV9281 remains in automatic MIPI transmit-timing mode through **0x4802 = 0x00**. Its register tables document manual/reference values including LPX 50 ns, HS prepare 50–85 ns, HS exit 100 ns, HS zero 112 ns + 6 UI, HS trail 60 ns + 4 UI, clock prepare 38–95 ns, clock zero 300 ns, clock post 60 ns + 52 UI, and clock trail 60 ns. With automatic timing selected, those inactive manual fields are not a guarantee of the exact transmitted intervals.

The receiver is transition- and 0xB8-driven; it is not a general timing-compliance checker. It reacts immediately to synchronized LP transitions, inhibits early B8 acceptance for the receiver settle interval, and uses the much wider 64-byte-cycle B8 timeout for failure detection. It does not use a tight prepare-plus-zero failure window.

### 7.2 LP state interpretation

For both clock and data lanes:

| LP P/N | Meaning used by this design |
|---|---|
| 11 | Stop state / LP11 |
| 01 | HS request / LP01 |
| 00 | HS zero / active HS interval |
| 10 | Escape-mode request / unsupported LP10 |

The legal receive entry is stable LP11 → LP01 → LP00. LP10, a skipped state, or a reversed transition sets **LP_SEQUENCE_ERROR**. This design does not implement escape mode, LPDT, bus turn-around, remote reset, suspend, or ULPS.

The five-cycle LPX value does not mean “wait five cycles before enabling the receiver.” The first two-flop-synchronized LP01 observation asserts **HS_TERM**. The first synchronized LP00 observation asserts **HS_ENA**. LPX counting validates the transition history and initial/final stable states while the nine-cycle receiver settle gate prevents premature B8 acceptance.

### 7.3 Initialization

After reset:

1. Hold hard receive lanes reset and both HS enables/terminations low.
2. Run the fixed 100 µs initialization timer in **ctrl_clk**.
3. Require stable LP11 on the clock lane and both data lanes for at least the fixed LPX qualification.
4. Assert **init_done**. This means the FPGA control side is ready and firmware may start sensor streaming.
5. Arm clock-lane acquisition and wait indefinitely while the sensor intentionally remains in standby.
6. After the sensor begins the legal clock-lane HS transition, enable the clock HSIO and require a recovered-clock heartbeat.
7. Release the byte-domain and hard-lane session reset only after the heartbeat is present.

Streaming must not be enabled at the sensor until **init_done** is high. The recommended sensor sequence in Section 15 configures the sensor in software standby and starts streaming last.

There is deliberately no timeout while the sensor remains in stable LP11 standby. **init_done** remains high until global reset and is not a live link-status signal. **INITIALIZATION_ERROR** is set only if a clock-lane acquisition has actually begun—LP01/LP00 was observed—but either no synchronized heartbeat change is observed or no byte-domain reset-release acknowledgement arrives within the fixed 64-cycle acquisition bound.

### 7.4 Continuous clock lane

The sensor is configured for a free-running clock lane. The clock-lane FSM acquires HS once at stream start:

1. Qualify LP11.
2. On LP01, assert **clk_hs_term**.
3. On LP00, assert **clk_hs_ena** and begin the fixed enable/settle guards.
4. Observe a change in the synchronized recovered-byte-clock Gray heartbeat.
5. Keep **clk_hs_term** and **clk_hs_ena** asserted for the entire streaming session.

The clock lane is not reacquired between lines or packets. It is released only after sensor standby/stable LP11 or after a clock-loss/reset recovery. This is the principal D-PHY timing simplification enabled by the OV9281 settings.

Once acquired, 64 **ctrl_clk** cycles with no synchronized Gray-heartbeat change is treated as clock loss. The monitor is disabled before acquisition. Stable clock-lane LP11 after a correctly closed Frame End, or a global reset, identifies intentional standby; a stopped clock during an open frame is an error. Clock loss sets **CLOCK_STOP_ERROR**, asynchronously reasserts the hard-lane/byte-session reset, suppresses pixel writes, and requires a new legal LP11 acquisition.

### 7.5 Data-lane packet bursts

The OV9281 data lanes still enter and leave HS for packet bursts even though the clock is continuous. The datasheet's MIPI timing diagram shows a B8 sync byte, one short or long packet, and trail for each illustrated burst. This specialized receiver assumes one CSI-2 packet per B8/trail burst.

That assumption is a hard architectural precondition, not a normative guarantee stated by the datasheet. A captured OV9281 stream using the final register script must confirm it before RTL is frozen. If the sensor emits back-to-back CSI-2 packets inside one HS interval, the sensor mode must be changed or the receiver must be expanded to find consecutive packet headers without a new B8/LP boundary.

For each lane:

1. Qualify stable LP11.
2. On LP01, assert that lane's **data_hs_term**.
3. On LP00, assert that lane's **data_hs_ena**.
4. Keep draining the hard FIFO while the fixed term/settle guards expire.
5. Enable B8 acceptance and bit alignment.
6. Receive exactly one CSI-2 packet.
7. After the expected header/payload/footer is complete, continue draining HS trail.
8. When stable LP11 returns, deassert that lane's **data_hs_ena** and **data_hs_term**, enter EOT-pending, and continue the unconditional hard-FIFO drain.
9. Retain the current aligner/parser/queue state for the fixed eight-**byte_clk** stop-pipeline guard so bytes already captured before LP11 can emerge.
10. If the counted footer completes during the guard, finish normally. If it remains incomplete at guard expiry, set **PACKET_LENGTH_ERROR**. Then clear per-burst alignment/parser queues and discard all out-of-burst words until the next legal HS entry.

The hard FIFO is not required to become empty at LP11. In the official model its write side continues to advance with the supplied slow clock, so emptiness is not a reliable packet-end condition. Normal packet completion is established by the fixed CSI byte counts, followed by the guarded LP11 cleanup above.

The local Efinity documentation available for this design does not specify an external timing guarantee for the relative order/spacing of **HS_TERM** and **HS_ENA**. Keeping them separate and asserting termination at LP01 before enabling the HS receiver at LP00 is the design assumption. It must be validated on hardware. The hard-lane RTL simulation model does not model analog termination or delay behavior.

## 8. Start-of-Transmission and lane alignment

### 8.1 Per-lane bit alignment

Each lane owns a 16-bit sliding history of successfully drained hard-FIFO data. For every new x8 word, the aligner considers all eight bit offsets and searches for the D-PHY synchronization byte 0xB8.

The first accepted B8 after the synchronized HS-acquire/settle gate defines that lane's byte phase for the current burst. B8 is consumed as framing and is not passed to the CSI-2 parser. A valid B8 must be found on both lanes within 64 recovered byte-clock transfers of the search window opening. Timeout sets **SOT_SYNC_ERROR**, discards the burst, and aborts the active image frame if one is open.

Alignment is reacquired for every data-lane burst. No previous packet's bit phase is trusted.

### 8.2 Per-lane aligned-byte queues

After a lane finds B8, its aligned bytes enter an eight-byte soft queue while the other lane finishes alignment. The queues serve three purposes:

- Absorb the small unknown phase difference between the hard FIFOs.
- Absorb occasional **FIFO_EMPTY** bubbles.
- Preserve continuously drained bytes while the parser waits for a complete lane pair.

The implementation uses eight parallel eight-bit shift histories per lane.
Accepted pushes shift the histories; occupancy selects the oldest queued byte.
A registered head plus a fresh-byte bypass preserves empty fall-through and
simultaneous pop/push behavior without a fabric register-ring read mux.

The queues are not substitutes for the hard FIFO and are not packet buffers. If either queue overflows, the core continues draining the hard FIFO, sets **LANE_ALIGNMENT_ERROR**, discards the burst, and aborts the active frame.

The supported post-B8 lane skew is therefore at most eight complete aligned bytes, subject to implementation pointer convention. The acceptance test must use a conservative maximum of seven queued bytes; an eighth-byte-full condition is the failure boundary. No local Titanium document specifies the maximum physical lane skew, so this bound must be checked against the routed hardware.

### 8.3 Lane-to-byte ordering

After both lanes are aligned, lane 0 supplies the first sequential CSI-2 byte and lane 1 supplies the next:

| Paired byte-clock transfer | Lane 0 | Lane 1 |
|---|---|---|
| Header pair 0 | Data ID | Word count/data field LSB |
| Header pair 1 | Word count/data field MSB | ECC |
| Payload pair 0 | Payload byte 0 | Payload byte 1 |
| Payload pair 1 | Payload byte 2 | Payload byte 3 |
| ... | ... | ... |
| Footer pair | CRC LSB | CRC MSB |

A pair is presented to the CSI parser only when both aligned-byte queues are nonempty. Each queue is popped exactly once for that pair. Lane indices must not be swapped.

## 9. CSI-2 packet contract

### 9.1 Header layout

Every packet begins with four bytes after B8:

| Byte | Meaning |
|---:|---|
| 0 | Data ID: VC in bits [7:6], data type in bits [5:0] |
| 1 | Word count or short-packet data field [7:0] |
| 2 | Word count or short-packet data field [15:8] |
| 3 | Header ECC in bits [5:0], reserved zeros in bits [7:6] |

Supported VC0 short-packet data types are:

| Packet | DT |
|---|---:|
| Frame Start | 0x00 |
| Frame End | 0x01 |

Line Start (0x02), Line End (0x03), any nonzero virtual channel, and any other data type are outside the expected sensor contract.

The only decoded long packet is VC0 RAW10, DT 0x2B, with word count 1600 (0x0640). Correctly headed but unsupported packets are flagged and their entire one-packet HS burst is drained to LP11 without interpreting the body. If a malformed or uncorrectable header means the internal length cannot be trusted, the same physical drain-to-LP11 boundary is used.

### 9.2 Fixed frame sequence

The accepted image sequence is:

1. VC0 Frame Start short packet.
2. Exactly 800 VC0 RAW10 long packets.
3. VC0 Frame End short packet.

The Frame End data field must equal the most recently accepted Frame Start data field. The value is not required to increment monotonically, because equality at the two boundaries is sufficient for this fixed receiver.

A RAW10 packet before Frame Start is drained but not output. A new valid Frame Start while a frame is already active aborts the old frame and starts a new byte-domain frame only if the pixel-domain handoff is safe as described in Section 12.

Line Start/End packets are disabled by the proposed sensor configuration. Embedded-data output is not documented or expected, but the listed registers do not prove that every possible sensor script disables it. If LS/LE, embedded data, or any other non-RAW10 packet appears with a correct header, the receiver drains its burst to LP11, sets **UNEXPECTED_PACKET**, and does not increment the image line counter.

### 9.3 Fixed packet arithmetic

For each line:

- Pixels: 1280.
- RAW bits: 1280 × 10 = 12,800.
- Payload bytes: 12,800 / 8 = 1600 = 0x0640.
- Two-lane payload transfers: 1600 / 2 = 800.
- RAW10 five-byte groups: 1600 / 5 = 320.
- Output beats: 1280 / 2 = 640.
- Paired transfers after B8: 2 header + 800 payload + 1 CRC = 803.

For each frame:

- Lines: 800.
- Pixels: 1,024,000.
- Output beats: 512,000.

The parser uses these fixed counts. A RAW10 word count other than 1600 sets **WORD_COUNT_ERROR**, suppresses that packet's pixel writes, and aborts the active image frame.

## 10. Header ECC

The 24 protected header bits are numbered **D0 through D23** in transmission order:

- D0 is bit 0 of Data ID.
- D7 is bit 7 of Data ID.
- D8 is bit 0 of the second header byte.
- D16 is bit 0 of the third header byte.

The six calculated parity bits are:

    P0 = D0 ^ D1 ^ D2 ^ D4 ^ D5 ^ D7 ^ D10 ^ D11 ^ D13 ^ D16 ^ D20 ^ D21 ^ D22 ^ D23
    P1 = D0 ^ D1 ^ D3 ^ D4 ^ D6 ^ D8 ^ D10 ^ D12 ^ D14 ^ D17 ^ D20 ^ D21 ^ D22 ^ D23
    P2 = D0 ^ D2 ^ D3 ^ D5 ^ D6 ^ D9 ^ D11 ^ D12 ^ D15 ^ D18 ^ D20 ^ D21 ^ D22
    P3 = D1 ^ D2 ^ D3 ^ D7 ^ D8 ^ D9 ^ D13 ^ D14 ^ D15 ^ D19 ^ D20 ^ D21 ^ D23
    P4 = D4 ^ D5 ^ D6 ^ D7 ^ D8 ^ D9 ^ D16 ^ D17 ^ D18 ^ D19 ^ D20 ^ D22 ^ D23
    P5 = D10 ^ D11 ^ D12 ^ D13 ^ D14 ^ D15 ^ D16 ^ D17 ^ D18 ^ D19 ^ D21 ^ D22 ^ D23

The syndrome is the received ECC bits [5:0] XOR the recalculated parity vector.

The decoder behavior is:

- Zero syndrome: header is unchanged.
- Syndrome equal to one of the 24 data-bit parity columns: flip that one header bit, set **HEADER_ECC_CORRECTED**, then validate the corrected VC/DT/word count.
- One-hot syndrome corresponding to an ECC parity-bit error: leave the header unchanged and set **HEADER_ECC_CORRECTED**.
- Any other nonzero syndrome: set **HEADER_ECC_UNCORRECTABLE**, discard the packet, drain the burst to LP11, and abort an active image frame.
- Received ECC bits [7:6] nonzero: treat the header as malformed and uncorrectable.

Correction occurs before packet type and length validation. A corrected header does not bypass any fixed-format check.

## 11. Payload CRC

Every long packet ends with a two-byte CSI-2 CRC:

- Polynomial: x^16 + x^12 + x^5 + 1.
- Initial value: 0xFFFF.
- Payload processing order: wire byte order, lane 0 byte before lane 1 byte.
- Bit order within each byte: least-significant bit first.
- Equivalent reflected shift-right polynomial: 0x8408.
- Footer order: received CRC LSB, then received CRC MSB.

Only payload bytes are included; the four-byte header and two-byte received CRC are not.

The CRC result is compared during the footer pair. A mismatch sets **PAYLOAD_CRC_ERROR**. It does not retract pixels already written to or read from the async FIFO. A full-line buffer would be required for transactional “CRC-good-only” output, and that cost is deliberately excluded. CRC status is therefore a post-line integrity diagnostic, not a guarantee attached to each output beat.

Malformed early end-of-transmission or a missing counted footer sets **PACKET_LENGTH_ERROR**. After the counted footer, all remaining deserialized words are treated as HS trail or out-of-burst hard-FIFO data and are drained without being parsed or flagged as extra packet bytes.

## 12. RAW10 unpacking and pixel CDC

### 12.1 RAW10 group mapping

For five sequential payload bytes **d0, d1, d2, d3, d4**, the four output pixels are:

    p0 = {d0, d4[1:0]}
    p1 = {d1, d4[3:2]}
    p2 = {d2, d4[5:4]}
    p3 = {d3, d4[7:6]}

The eight-bit byte supplies pixel bits [9:2]; the two packed bits supply pixel bits [1:0]. Pixels are emitted in p0, p1, p2, p3 order.

### 12.2 Two-lane gearbox schedule

Ten payload bytes, or two RAW10 groups, arrive in five paired transfers. Let A0–A4 be the first RAW10 group and B0–B4 the second:

| Pair phase | Lane 0 / lane 1 input | Pixel action |
|---:|---|---|
| 0 | A0 / A1 | Emit pending B p2/p3 from the preceding schedule, if any |
| 1 | A2 / A3 | Accumulate |
| 2 | A4 / B0 | Emit A p0/p1; retain A p2/p3; save B0 |
| 3 | B1 / B2 | Emit retained A p2/p3 |
| 4 | B3 / B4 | Emit B p0/p1; retain B p2/p3 |

The next phase 0 emits the retained B p2/p3 while accepting the next schedule. On the final line schedule, the retained final pair is emitted on the following footer clock while the CRC bytes are independently compared.

The 1600-byte line is exactly divisible by five. There are 160 ten-byte schedules, 320 RAW10 groups, and 640 output beats. No partial RAW10 group is legal.

During active RAW10 payload, the gearbox writes four 20-bit beats per five **byte_clk** cycles: 80 million beats/s at the nominal link rate. The full-frame average at nominal 120 fps is 512,000 × 120 = 61.44 million beats/s. Both are below the required pixel interface's 100 million beat/s capacity.

### 12.3 Async pixel FIFO

The CDC structure is a power-of-two-depth, 20-bit dual-clock FIFO with
Gray-coded pointers. The reusable default is 512 entries; the integrated Ti60
profile uses 64 entries:

- Write clock: **byte_clk**.
- Read clock: **pixel_clk**.
- Data stored: **pixel_data[19:0]** only.
- Intended memory mapping: one Titanium RAM10/EFX_RAM10 block, subject to synthesis packing.

The 64-entry specialization is tied to the Ti60 rate contract. The gearbox
writes at most four beats per five 100 MHz byte clocks, whereas that target's
pixel side drains at 120 MHz. Occupancy therefore cannot grow during a payload;
the remaining depth covers frame-start CDC latency, phase, and RAM-read
prefetch. The larger default remains available for integrations with a smaller
rate margin.

The FIFO is elasticity for phase and small frequency differences; it is not a complete line buffer. The read side may empty between gearbox writes, so **pixel_valid** is allowed to bubble.

There is no downstream backpressure. If the FIFO becomes full:

1. Set **PIXEL_FIFO_OVERFLOW**.
2. Stop writing image pixels for the remainder of the active frame.
3. Continue draining and parsing the physical CSI-2 input.
4. Send a frame-abort toggle to the pixel domain.
5. In the pixel domain, suppress further public **pixel_valid**, internally drain/discard every remaining old-frame FIFO entry, reset the marker counters, and return a pixel-idle acknowledgement.
6. Resynchronize at a later valid Frame Start only after that acknowledgement proves that no old-frame beat can be confused with new-frame data.

The same abort handshake is used for any byte-domain error that suppresses the remainder of an active frame. Async FIFO pointers must not be independently reset in only one domain; reset is global, while routine frame recovery uses drain-and-acknowledge.

Normal operation requires **pixel_clk** to run continuously at 100 MHz as part of the public contract. The active-line write rate is 80 MHz, while the nominal frame-average rate is 61.44 MHz. A clock between those rates might use blanking to recover occupancy, so “below 80 MHz always overflows” is not a valid general claim. Rates other than 100 MHz are unsupported unless FIFO occupancy is reanalyzed against the complete sensor timing.

### 12.4 Frame event and marker generation

Markers are not stored per beat. The byte domain sends an accepted-Frame-Start event to the pixel domain using a toggle/event synchronizer. The pixel reader must latch that event and arm the frame **before it may pop the first FIFO entry**, even if the async FIFO's data-ready indication arrives first. It then waits for the first readable pixel beat. The event crosses before the minimum supported 64-entry FIFO can accumulate under the fixed rate contract. A separate abort toggle and pixel-idle acknowledgement implement the recovery sequence in Section 12.3.

To prevent a separately synchronized frame event from overtaking prior pixel data, a new output frame is accepted only after a normal pixel-frame-complete/idle acknowledgement or an abort-drain idle acknowledgement has returned to the byte domain. Normal idle means that the **pixel_eof** beat was presented with **pixel_valid**, the async FIFO is empty, and no registered RAM read or output-valid beat remains pending. FIFO empty alone is insufficient. If a new Frame Start arrives while old-frame data or any completion/abort handshake remains pending, set **FRAME_SEQUENCE_ERROR**, drop the incoming frame, and recover at a later safe Frame Start.

If an active frame is aborted after some pixels were already presented, the public stream is a deliberately incomplete frame: it stops without a fabricated **pixel_eof**. Sticky status identifies the reason. The next accepted frame starts cleanly with a new **pixel_sof** only after the drain handshake completes.

In the pixel domain:

- Horizontal beat count is 0 through 639.
- Vertical line count is 0 through 799.
- The first valid beat at (line 0, beat 0) asserts **pixel_sof** and **pixel_sol**.
- Every valid beat with horizontal count 0 asserts **pixel_sol**.
- Every valid beat with horizontal count 639 asserts **pixel_eol**.
- The valid beat at (line 799, beat 639) asserts both **pixel_eol** and **pixel_eof**.
- Counters advance only on **pixel_valid**.

Because the CSI-2 Frame End short packet arrives after the final pixels, **pixel_eof** necessarily precedes the Frame End consistency check. It means “fixed final image beat,” not “the entire frame has subsequently passed CRC and FE checks.”

## 13. Status interface

All defined bits are sticky until **reset_n** is asserted or a **ctrl_clk** pulse on **status_clear** clears them. Byte- and pixel-domain events are captured as source-domain sticky levels before synchronization, so a one-cycle event cannot be lost.

**status_clear** launches a clear-toggle handshake into both source domains. Each active source domain locally clears its sticky bits, with a coincident new set taking priority, and returns an acknowledgement toggle. Control-domain copies clear locally. A source-derived control status bit is considered cleared only after the corresponding acknowledgement returns, preventing an uncleared source level from immediately reasserting it. Only one clear transaction is outstanding at a time; additional pulses while it is pending are coalesced. If **byte_clk** is stopped, its clear is deferred until that clock resumes; software can poll until the requested bits clear or use global reset. No clear pulse is sampled directly across a clock boundary.

| Bit | Name | Meaning |
|---:|---|---|
| 0 | HEADER_ECC_CORRECTED | One header data bit or ECC parity bit was corrected; informational |
| 1 | HEADER_ECC_UNCORRECTABLE | Header could not be safely corrected |
| 2 | PAYLOAD_CRC_ERROR | Long-packet payload CRC mismatch |
| 3 | SOT_SYNC_ERROR | Missing/ambiguous B8 or B8 timeout |
| 4 | LANE_ALIGNMENT_ERROR | Lane queue overflow or lanes could not be paired |
| 5 | LP_SEQUENCE_ERROR | Unsupported or illegal LP-state transition |
| 6 | UNEXPECTED_PACKET | Wrong VC/DT or unexpected LS/LE/embedded/config packet |
| 7 | WORD_COUNT_ERROR | RAW10 word count was not 1600 |
| 8 | PACKET_LENGTH_ERROR | Premature end or missing counted packet/footer data |
| 9 | FRAME_SEQUENCE_ERROR | RAW before FS, nested FS, FE mismatch, missing/early FE, or unsafe CDC frame handoff |
| 10 | LINE_COUNT_ERROR | FE did not close exactly 800 accepted lines or more than 800 lines arrived |
| 11 | PIXEL_FIFO_OVERFLOW | No-backpressure output CDC FIFO overflowed |
| 12 | CLOCK_STOP_ERROR | Continuous recovered clock disappeared during an active session |
| 13 | INITIALIZATION_ERROR | A started clock-lane acquisition failed to produce heartbeat/reset-release acknowledgement |
| 15:14 | RESERVED | Read as zero |

No dedicated interrupt is part of this block. System firmware may poll **status** or connect a reduction of selected bits externally if an interrupt is later needed.

## 14. Error handling and resynchronization

The receiver uses fixed, deterministic recovery rules:

| Condition | Pixel effect | Physical/link recovery |
|---|---|---|
| Corrected header/ECC bit | Continue normally | Continue current packet |
| Payload CRC error | Already emitted line remains visible | Record status and continue frame |
| Unsupported but correctly headed packet | No pixels; do not increment line | Ignore its body and drain the one-packet burst to LP11 |
| RAW10 before Frame Start | Drop packet | Drain and wait for valid FS |
| Wrong RAW10 word count | Suppress line and remainder of frame | Drain burst; wait for next valid FS |
| Uncorrectable header | Suppress remainder of frame | Drain to LP11 because length is untrusted |
| B8 timeout/lane alignment failure | Suppress remainder of frame | Drain to LP11; reacquire at next burst |
| Premature EOT/missing footer | Suppress remainder of frame | Drain/return to LP11; wait for next FS |
| More than 800 image lines | Drop extra data and flag | Drain until next FS/FE boundary |
| FE before/after expected 800 lines | Final pixels cannot be retracted | Flag frame and line sequence errors |
| Pixel FIFO overflow | Suppress remainder of frame | Continue draining/parsing; resume only at safe FS |
| New FS during active byte frame | Abort old frame | Treat new FS as candidate restart, subject to safe CDC handoff |
| Continuous clock loss | Suppress all writes | Assert session reset and perform complete reacquisition |

An abort suppresses future pixel-FIFO writes for that frame, but it never stops hard-FIFO draining. The strongest trustworthy resynchronization boundary is a new legal LP11 → HS → B8 burst containing a correct VC0 Frame Start header.

## 15. OV9281 sensor configuration contract

The OV9281 datasheet supports the intended mode: full-resolution 1280 × 800 at nominal 120 fps, RAW10, two lanes, and a typical 800 Mb/s lane rate. Its 24 MHz clock example produces a 100 MHz MIPI PCLK and the nominal 800 Mb/s serial rate; x8 deserialization therefore produces the required 100 MHz byte clock.

The following list is the CSI-related contract, not a complete sensor initialization table or a claim that the listed full-byte combinations are an official OmniVision mode script. Board-specific analog settings, exposure, gain, and the complete validated vendor initialization sequence still have to be supplied by camera firmware.

Program PLL and MIPI settings only while the sensor is in software standby (**0x0100 = 0**). Start streaming (**0x0100 = 1**) only after all relevant settings and the FPGA receiver are ready.

The sensor timing section documents up to 0.2 ms for PLL lock and a stream-start-to-first-Frame-Start fixed component of up to 10 ms. Firmware must honor the complete power/PLL sequence. The receiver does not impose a shorter first-frame timeout; once the continuous clock is valid, it may wait for Frame Start without treating ordinary sensor startup latency as a packet error.

### 15.1 Link-relevant PLL1 and mode fields

| Register | Required value or field | Purpose and caveat |
|---:|---:|---|
| 0x030A | 0x00 | PLL1 pre-divider 0 |
| 0x0300 | 0x01 | PLL1 pre-divider |
| 0x0301 | 0x00 | PLL1 loop divider high bits |
| 0x0302 | 0x32 | PLL1 loop divider low byte |
| 0x0303 | 0x00 | PLL1 M divider |
| 0x0304 | 0x03 | PLL1 MIPI divider |
| 0x0305 | 0x02 | PLL1 system pre-divider |
| 0x0306 | 0x01 | PLL1 system divider |
| 0x3022 | Bits [7:4] = 0101; reset-based full byte is 0x51 | Select 10-bit MIPI mode while preserving lower control bits; 0x51 is a logically derived proposed write, not a published 120 fps script value |
| 0x3039 | 0x32 | Enable MIPI and select two data lanes |
| 0x303A | 0x00 | Do not disable either lane |
| 0x3808 / 0x3809 | 0x05 / 0x00 | Output width = 1280 |
| 0x380A / 0x380B | 0x03 / 0x20 | Output height = 800 |
| 0x380C / 0x380D | 0x02 / 0xD8 | Full-resolution HTS = 0x02D8 for the nominal 120 fps timing |
| 0x380E / 0x380F | 0x03 / 0x8E | Full-resolution VTS = 0x038E for the nominal 120 fps timing |

These PLL1 values are the datasheet's documented/default 24 MHz example. Its complete clock example also lists PLL2 values **0x0314=0x00, 0x030B=0x04, 0x030C=0x00, 0x030D=0x50, 0x030F=0x03, 0x030E=0x02, 0x0313=0x01, 0x0312=0x07**. They are outside the CSI receiver boundary but must be preserved or replaced by the validated complete sensor script.

A different EXTCLK or any modified PLL requires re-deriving the lane and recovered-byte frequencies; this fixed receiver does not negotiate or measure a different lane rate. Exact frame cadence also depends on the complete mode script and timing registers. The receiver checks geometry and packet structure, not the interval between Frame Start packets.

### 15.2 Required MIPI behavior

| Register | Required setting | Effect on receiver specialization |
|---:|---|---|
| 0x3014 | Bits [2:1] = 10; reset-default full byte is 0x04 | Keep MIPI enabled and the separate MIPI suspend control deasserted |
| 0x4800 | Bits [5:4] = 00; full reset-default byte is 0x04 | Free-running clock lane; do not transmit LS/LE short packets |
| 0x4802 | 0x00 | Let the sensor automatically generate MIPI transmit timing |
| 0x4806 | Bits [3:1] = 000; reset-default byte is 0x00 | Disable remote reset, suspend, and automatic ULPS |
| 0x4813 | Leave reset default 0x00; bits [1:0] are 00 | Proposed VC0 behavior, but the datasheet does not define the polarity of vc_sel bit 2; verify VC0 from captured headers/vendor script |
| 0x4814 | Bit 6 = 1 and bits [5:0] = 0x2B; derived full byte is 0x6B | Proposed manual RAW10 data-type label; validate on the sensor because this field labels the packet and does not itself guarantee RAW10 packing |
| 0x484B | Preserve reserved bits; require documented bits [1:0] = 11 | Start clock after reset and use SOF-based frame behavior; do not blindly write the reset byte 0x07 because bit 2 is documented reserved |
| 0x484C | Bit 1 = 0 | Keep HS-only test mode disabled |

Continuous clock simplifies only the clock lane. Data lanes still burst for Frame Start, each long line packet, and Frame End; they still require LP transition handling, termination/enable control, B8 alignment, and trail draining.

The complete firmware configuration must also avoid low-frame-rate/automatic-sleep modes or any other power-management path that can stop the MIPI PHY during an open frame. Clearing **0x4806[1]** alone does not disable every possible sensor power-down mechanism.

The datasheet's Chapter 6 prose contains a sentence describing a single data lane, but the pinout, supported-mode table, lane-count register, and 1280 × 800 mode all explicitly support the two-lane configuration used here.

### 15.3 Bring-up test patterns

For link and unpacker validation:

- Set **0x5E00[7] = 1** to enable the sensor test-pattern generator. Preserve the other fields while selecting the desired pattern.
- Set **0x4320[1] = 1** for the solid test pattern if desired; preserve the other control bits, including the reset-default high bit.
- Keep **0x484C[1] = 0**. The HS-only mode is a PHY test mode and bypasses normal LP/packet behavior that this receiver must validate.

Documented bars, random, square, black, or a programmed solid value should be checked at the 10-bit pixel output before any truncation or display processing. This datasheet does not document a ramp-pattern mode.

## 16. Differences from a general-purpose CSI-2 RX

This analysis is architectural and remains valid in any destination project. The fixed core differs from the general generated Efinix-style receiver as follows:

| General-purpose receiver capability | Fixed OV9281 core |
|---|---|
| General CSI-2 packet/data-type receiver | VC0 RAW10 image decoding only |
| Multiple lane-rate/byte-clock configurations | Nominal 100 MHz recovered byte clock only |
| Continuous and discontinuous clock-lane modes | Acquire continuous clock once per streaming session |
| Multiple packing widths and pixels/clock | Fixed 20-bit, two-pixel output |
| Runtime data type and word-count outputs | Compile-time DT 0x2B and WC 1600 checks |
| Generic frame geometry | Fixed 1280 × 800 counters |
| Configurable/deep generic pixel buffering | Fixed-rate 64- or 512-entry × 20 CDC FIFO profiles |
| Generic virtual-channel handling | VC0 only; no VCX |
| AXI control/status and IRQ ports | No AXI; one clearable sticky status vector |
| LS/LE and arbitrary short packets | FS/FE only; LS/LE are unexpected |
| Variable lane/format support | Exactly two lanes, RAW10 |
| General pixel remapping downstream | RAW10 unpacker directly emits two ordered pixels |

The general core's protocol work cannot simply be removed. The specialized design still needs:

- Hard-HSIO control and reset.
- LP/HS receive state machines.
- Continuous hard-FIFO draining.
- Per-lane B8 bit alignment.
- Inter-lane byte alignment.
- Header ECC correction/detection.
- Payload CRC checking.
- Packet/frame sequencing and length checks.
- RAW10 gearbox.
- Pixel CDC and overflow handling.

The fixed design should use substantially less memory than a broadly configurable receiver because its target pixel FIFO fits one Titanium RAM10/EFX_RAM10 block and it has no format-selection buffers. No total resource number is promised until the future RTL is synthesized in the destination project.

## 17. Further simplifications that are intentionally used

The sensor contract allows these simplifications:

- No clock-lane stop/restart between packets.
- No one-/four-lane variants.
- No alternate lane rates or byte-clock measurement.
- No RAW8/RAW12/RGB/YUV/generic-byte unpackers.
- No arbitrary width, height, stride, or partial RAW group.
- No virtual-channel routing or extended VC.
- No LS/LE dependency.
- No embedded-data decoding.
- No ULPS, escape mode, LPDT, bus turn-around, suspend, or remote-reset handling.
- No dynamic deskew calibration; use static HSIO delay plus bounded digital lane queues.
- No full-line CRC quarantine buffer.
- No output **ready** path.
- No software register file, AXI slave, or internal IRQ block.
- No Bayer color reconstruction.

The OV9281 is monochrome. A destination pipeline does not need Bayer color gain or demosaicing. If an 8-bit grayscale consumer is required, use pixel bits [9:2]. If an RGB-only boundary is unavoidable, replicate that grayscale value into R, G, and B at the boundary rather than treating the sensor as a Bayer array.

## 18. Timing constraints required by a future integration

This document does not edit constraints, but the implementation must account for:

- A 10 ns nominal generated clock on **byte_clk**, sourced by the destination clock HSIO's divided CLKOUT, for the 800 Mb/s x8 receive mode.
- Destination-generated Interface Designer timing arcs from that recovered clock to **data_fifo_rd** and **data_rst**.
- Destination-generated timing arcs from **data0_hs_in**, **data1_hs_in**, and **data_fifo_empty** to the recovered clock.
- A 10 ns **pixel_clk**.
- A 10 ns **ctrl_clk**.
- Asynchronous relationships among ctrl, byte, and pixel clocks, except for the dedicated HSIO relationships encoded by the generated constraints.
- Proper false-path/max-delay treatment inside the Gray-pointer async FIFO rather than treating its buses as ordinary synchronous paths.
- Synchronizers for LP states, heartbeat/events, status, and reset release.

The destination project may constrain the recovered clock more tightly than its 10 ns nominal period as deliberate margin, but the selected period and HSIO timing arcs must be documented and revalidated rather than inherited from an unrelated project.

## 19. Verification and acceptance plan for the future RTL

No implementation is part of this document, but the eventual core is not complete until all of the following are tested.

### 19.1 HSIO boundary tests

- Use the official **EFX_MIPI_RX_LN_V1** model for each data lane.
- Exercise every initial bit slip from 0 through 7.
- Exercise independent lane slips and post-B8 lane skew from 0 through the supported queue bound.
- Insert independent **FIFO_EMPTY** bubbles.
- Prove/assert that **FIFO_RD** is never high when its lane is empty.
- Prove/assert that every nonempty hard FIFO is drained even during discard/recovery.
- Check show-ahead capture and consume occur on the same intended byte-clock transfer.
- Check hard reset asynchronous assertion and synchronized deassertion after clock start.

### 19.2 Valid-stream tests

- Receive FS, exactly 800 1600-byte RAW10 lines, and matching FE.
- Check exactly 512,000 output beats and 1,024,000 pixels.
- Check exactly 640 valid beats per line and 800 lines.
- Check SOF/SOL/EOL/EOF locations in the presence of pixel-valid bubbles.
- Check lane 0 supplies each earlier striped byte and lane 1 each later byte.
- Check the final pending pixel pair is emitted correctly during the CRC-footer input cycle.

### 19.3 RAW10 tests

- All-zero and all-one pixels.
- Alternating 0x000 and 0x3FF.
- Walking bits through both the upper eight and packed lower two bits.
- Random pixels with a software reference packer/unpacker.
- Boundaries between two five-byte groups and between two ten-byte gearbox schedules.
- First and final pixels of every line.

### 19.4 ECC and CRC tests

- Clean headers.
- Every one of the 24 single header-data-bit errors.
- Every one of the six ECC parity-bit errors.
- Every one of the 435 two-bit combinations across the 24 protected header bits and six ECC parity bits; each must be detected as uncorrectable.
- Nonzero reserved ECC bits.
- Good payload CRC.
- Corrupt every payload-byte position and each received CRC byte.
- Confirm a CRC error is reported without pretending to retract already emitted data.

### 19.5 Protocol and recovery tests

- Wrong VC, wrong DT, LS, LE, and an unsupported long packet with a trustworthy length.
- RAW10 word counts 1599, 1601, zero, and large malformed values.
- RAW packet before FS.
- Nested FS, wrong FE frame number, early FE, missing FE, and more/fewer than 800 lines.
- Missing B8 on one/both lanes and B8 timeout.
- Premature LP11 and missing CRC footer; also verify that legal HS trail is drained without being parsed.
- Soft queue overflow.
- Pixel FIFO overflow and recovery without mixing frames.
- Global reset in LP11, during B8 search, header, payload, CRC, and output.
- Sensor standby and restart.
- Continuous-clock loss and complete reacquisition.

### 19.6 CDC and rate tests

- Sweep relative byte/pixel clock phase.
- Apply small positive and negative frequency offsets around 100 MHz.
- Confirm normal headroom with 80 Mbeats/s active-line bursts and 61.44 Mbeats/s nominal frame average.
- Demonstrate overflow/recovery by stopping or gating **pixel_clk**, and with a test rate below 61.44 MHz; do not assume every clock below 80 MHz must overflow.
- Use CDC analysis for every synchronizer, toggle, reset, and Gray pointer.

### 19.7 Hardware bring-up

- Begin with OV9281 test pattern at 1280 × 800 RAW10.
- Confirm the byte clock is nominally 100 MHz and the physical D-PHY clock lane is nominally 400 MHz for 800 Mb/s DDR data.
- Observe separate clock/data **HS_TERM** and **HS_ENA** behavior.
- Sweep the destination data-lane static delay across a safe supported range and select the center of the stable eye; reference-project tap 5 is not a portable default.
- Validate the assumed HS_TERM-before-HS_ENA sequence.
- Capture the final sensor mode and prove that every FS, RAW10 line, and FE packet has its own B8/trail burst before freezing the one-packet-per-burst architecture.
- Confirm captured headers use VC0 and DT 0x2B with the proposed 0x4813/0x4814 fields.
- Validate that the eight-byte-clock stop-pipeline guard is sufficient for the selected hard-lane placement and timing.
- Measure or stress actual inter-lane skew against the eight-byte queue design.
- Check error status over long runs before enabling normal imagery.

## 20. Known assumptions and caveats

1. The hard receive FIFO's actual silicon depth is not specified in the local documentation. The design relies only on nonempty/show-ahead behavior and continuous draining.
2. Show-ahead semantics are derived from the official Efinix receive-lane model and timing interface. They must be confirmed in hardware.
3. The Efinix RTL lane model does not model analog termination, settle behavior, input-delay taps, or signal integrity.
4. No local document gives a maximum data-lane skew for the chosen HSIO group. The eight-byte soft queue is a bounded design choice, not a device guarantee.
5. No local document gives a precise required fabric sequence for **HS_TERM** relative to **HS_ENA**. The proposed sequence is physically sensible and matches the LP request progression, but requires board validation.
6. The design assumes the OV9281 produces one CSI-2 packet per B8/trail data-lane burst as illustrated by its MIPI timing diagram. The datasheet does not state this as a guarantee; a captured final-mode stream is a precondition for implementation.
7. Continuous clock is a sensor configuration requirement. If firmware enables gated clock, this receiver's clock-lane assumptions are invalid.
8. CRC errors can be discovered after pixels from that line have crossed the public interface. Avoiding that requires a line buffer and a different interface contract.
9. The reusable 512 × 20 and Ti60 64 × 20 FIFO profiles should each fit one Titanium RAM10/EFX_RAM10 block, but final RAM mapping and total resources are synthesis results, not specification guarantees.
10. The sensor datasheet is preliminary revision 1.01. Any later sensor revision or vendor register script must be checked for changes to the stated register behavior.
11. The eight-byte-clock stop-pipeline guard is a bounded receiver assumption based on the modeled hard/soft pipeline. It must be validated in the destination placement; it is never replaced by waiting for FIFO empty.
12. The proposed 0x3022, 0x4813, and 0x4814 settings are field-level deductions, not a published complete 1280 × 800 mode script. Captured VC, DT, packing, word count, and packet boundaries are required bring-up checks.
13. With 0x4802 selecting automatic transmit timing, manual timing-register values are references rather than guarantees. The receiver constants in Section 7.1 remain hardware-validation assumptions.

## 21. Source material used

- **Ti60F225_dsi/edge_vision_soc.peri.xml** — reference-only camera HSIO definitions and generated input-side fabric pins.
- **Ti60F225_dsi/edge_vision_soc.v** — reference-only wiring between the hard HSIO boundary and the CSI-2 IP inputs.
- **Ti60F225_dsi/edge_vision_soc.sdc** — reference-only recovered-clock hard-lane timing arcs.
- [OV9281 sensor datasheet](../OV9281_CSP5_DS_1.01.pdf) — OV9281 version 1.01 sensor behavior, timing, modes, PLL, and MIPI registers.
- `$EFINITY_HOME/pt/doc/topics/titanium-hsio-as-mipi-1.html` — Titanium HSIO MIPI usage and fabric interface.
- `$EFINITY_HOME/pt/sim_models/verilog/EFX_MIPI_RX_LN_V1.v` — official x8 receive-lane and FIFO simulation semantics.

## 22. Completion boundary

This specification defines a standalone, destination-project-independent core boundary and behavior. The next engineering phase, when explicitly authorized, would be to write and verify standalone RTL against this contract and bind it to an equivalent HSIO interface in the actual destination project. No RTL or project integration is performed here.


## Optional native counter storage

`USE_DSP_COUNTERS` defaults to zero. The Ti60 camera integration enables it for
the initialization guard, clock timeout, and pixel X/Y counters: four DSP48s
per receiver. Arithmetic width, terminal holds, clear priority, and output
cadence remain unchanged. No status bits, byte-domain parser state, FIFO
pointers, or CDC synchronizers move into DSPs.

`Ov9281Counter` keeps asynchronous global reset separate from synchronous
operational clear. The native unsigned operation is `value * !clear + !clear`,
enabled on clear or increment. Its registered low-width feedback wraps exactly
like the original counter. Pixel increments retain priority over a simultaneous
frame-start initialization; reset and abort take priority over both.

Ordinary simulators use portable arithmetic without vendor libraries. Efinity
translation directives select native hardware without requiring a predefined
`SYNTHESIS` macro. Native-model tests explicitly define that macro. Disabling
the option, or using a width above the native 19-bit input limit, retains the
portable implementation.

Run `pytest tests/lutcam/test_csi_counters.py` for native/portable counter checks
(native cases require the installed Efinity model), and `cargo test --manifest-path ../Cargo.toml -p
csi2-rx-rtl-tests` for raw HSIO packet, asynchronous clock, and reset/session
coverage. Board electrical timing and full-SoC C3 closure remain separate
integration checks.
