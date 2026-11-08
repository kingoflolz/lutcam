//! End-to-end Marlin/Verilator checks at the raw Titanium HSIO FIFO boundary.

use std::collections::VecDeque;
use std::path::{Path, PathBuf};

use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use ov9281_csi2_rx::{
    DATA_TYPE_FRAME_START, DATA_TYPE_RAW10, HeaderCorrection, IMAGE_WIDTH, OV9281_VIRTUAL_CHANNEL,
    Ov9281PacketReference, PixelBeat, RAW10_BYTES_PER_LINE, ReceiverStatus, build_long_packet,
    build_short_packet, pack_raw10,
};
use rtl_test_support::{TestResult, read_u64, with_model};

const MANIFEST: &str = "tests/csi2_rx/tests/rtl_sources.f";
const ADAPTER: &str = "tests/csi2_rx/tests/rtl_adapters/Ov9281Csi2RxTestTop.sv";

const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("debug_parser_state", 2, 0, PortDirection::Output),
    ("debug_gearbox_phase", 2, 0, PortDirection::Output),
    ("debug_pair_pop", 0, 0, PortDirection::Output),
    ("reset_n", 0, 0, PortDirection::Input),
    ("ctrl_clk", 0, 0, PortDirection::Input),
    ("byte_clk", 0, 0, PortDirection::Input),
    ("pixel_clk", 0, 0, PortDirection::Input),
    ("status_clear", 0, 0, PortDirection::Input),
    ("clk_lp_p", 0, 0, PortDirection::Input),
    ("clk_lp_n", 0, 0, PortDirection::Input),
    ("clk_hs_term", 0, 0, PortDirection::Output),
    ("clk_hs_ena", 0, 0, PortDirection::Output),
    ("data_lp_p", 1, 0, PortDirection::Input),
    ("data_lp_n", 1, 0, PortDirection::Input),
    ("data0_hs_in", 7, 0, PortDirection::Input),
    ("data1_hs_in", 7, 0, PortDirection::Input),
    ("data_fifo_empty", 1, 0, PortDirection::Input),
    ("data_fifo_rd", 1, 0, PortDirection::Output),
    ("data_rst", 1, 0, PortDirection::Output),
    ("data_hs_term", 1, 0, PortDirection::Output),
    ("data_hs_ena", 1, 0, PortDirection::Output),
    ("init_done", 0, 0, PortDirection::Output),
    ("status", 15, 0, PortDirection::Output),
    ("pixel_data", 19, 0, PortDirection::Output),
    ("pixel_valid", 0, 0, PortDirection::Output),
    ("pixel_sof", 0, 0, PortDirection::Output),
    ("pixel_sol", 0, 0, PortDirection::Output),
    ("pixel_eol", 0, 0, PortDirection::Output),
    ("pixel_eof", 0, 0, PortDirection::Output),
    ("burst_sources", 3, 0, PortDirection::Output),
    ("burst_destinations", 11, 0, PortDirection::Output),
    ("burst_pulses", 3, 0, PortDirection::Output),
    ("byte_session_active", 0, 0, PortDirection::Output),
];

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn with_receiver(
    test: impl for<'ctx> FnOnce(&mut DynamicVerilatedModel<'ctx>) -> TestResult,
) -> TestResult {
    let root = gateware_root();
    let adapter = root.join(ADAPTER);
    with_model(
        &root,
        Path::new(MANIFEST),
        "ov9281-csi2-rx-e2e",
        "Ov9281Csi2RxTestTop",
        Path::new(ADAPTER),
        PORTS,
        &[Path::new("rtl/csi2_rx")],
        &[adapter.as_path()],
        test,
    )
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CapturedBeat {
    data: u32,
    sof: bool,
    sol: bool,
    eol: bool,
    eof: bool,
}

impl From<PixelBeat> for CapturedBeat {
    fn from(beat: PixelBeat) -> Self {
        Self {
            data: beat.packed_data(),
            sof: beat.sof,
            sol: beat.sol,
            eol: beat.eol,
            eof: beat.eof,
        }
    }
}

#[derive(Default)]
struct LaneSource {
    words: VecDeque<u8>,
    bubble_cycles: Vec<usize>,
    cycle: usize,
}

impl LaneSource {
    fn load(&mut self, words: Vec<u8>, mut bubble_cycles: Vec<usize>) {
        assert!(self.words.is_empty());
        bubble_cycles.sort_unstable();
        bubble_cycles.dedup();
        self.words = words.into();
        self.bubble_cycles = bubble_cycles;
        self.cycle = 0;
    }

    fn is_bubble(&self) -> bool {
        self.bubble_cycles.binary_search(&self.cycle).is_ok()
    }

    fn empty_flag(&self) -> bool {
        self.words.is_empty() || self.is_bubble()
    }

    fn head(&self) -> u8 {
        self.words.front().copied().unwrap_or(0)
    }

    fn byte_edge(&mut self, read: bool) {
        let empty = self.empty_flag();
        assert!(!(read && empty), "test source was read while empty");
        if read {
            self.words.pop_front();
        }
        if !self.words.is_empty() {
            self.cycle += 1;
        }
    }

    fn drained(&self) -> bool {
        self.words.is_empty()
    }
}

/// Scheduler tick is one sixth of a nanosecond. The two 100 MHz clocks toggle
/// every 30 ticks; the 120 MHz clock toggles every 25 ticks.
struct Simulation<'model, 'context> {
    dut: &'model mut DynamicVerilatedModel<'context>,
    now: u64,
    next_ctrl: u64,
    next_byte: u64,
    next_pixel: u64,
    ctrl_high: bool,
    byte_high: bool,
    byte_running: bool,
    pixel_high: bool,
    ctrl_edges: usize,
    byte_edges: usize,
    pixel_edges: usize,
    lanes: [LaneSource; 2],
    captured: Vec<CapturedBeat>,
}

impl<'model, 'context> Simulation<'model, 'context> {
    fn new(dut: &'model mut DynamicVerilatedModel<'context>) -> TestResult<Self> {
        dut.pin("reset_n", 0_u8)?;
        dut.pin("ctrl_clk", 0_u8)?;
        dut.pin("byte_clk", 0_u8)?;
        dut.pin("pixel_clk", 0_u8)?;
        dut.pin("status_clear", 0_u8)?;
        dut.pin("clk_lp_p", 1_u8)?;
        dut.pin("clk_lp_n", 1_u8)?;
        dut.pin("data_lp_p", 3_u8)?;
        dut.pin("data_lp_n", 3_u8)?;
        dut.pin("data0_hs_in", 0_u8)?;
        dut.pin("data1_hs_in", 0_u8)?;
        dut.pin("data_fifo_empty", 3_u8)?;
        dut.eval();
        Ok(Self {
            dut,
            now: 0,
            next_ctrl: 30,
            next_byte: 17,
            next_pixel: 25,
            ctrl_high: false,
            byte_high: false,
            byte_running: true,
            pixel_high: false,
            ctrl_edges: 0,
            byte_edges: 0,
            pixel_edges: 0,
            lanes: [LaneSource::default(), LaneSource::default()],
            captured: Vec::new(),
        })
    }

    fn drive_lane_heads(&mut self) -> TestResult {
        let empty =
            u8::from(self.lanes[0].empty_flag()) | (u8::from(self.lanes[1].empty_flag()) << 1);
        self.dut.pin("data0_hs_in", self.lanes[0].head())?;
        self.dut.pin("data1_hs_in", self.lanes[1].head())?;
        self.dut.pin("data_fifo_empty", empty)?;
        Ok(())
    }

    fn step(&mut self) -> TestResult {
        let next_byte = if self.byte_running {
            self.next_byte
        } else {
            u64::MAX
        };
        let event_time = self.next_ctrl.min(next_byte).min(self.next_pixel);
        self.now = event_time;
        let toggle_ctrl = self.next_ctrl == event_time;
        let toggle_byte = self.byte_running && self.next_byte == event_time;
        let toggle_pixel = self.next_pixel == event_time;
        let ctrl_rising = toggle_ctrl && !self.ctrl_high;
        let byte_rising = toggle_byte && !self.byte_high;
        let pixel_rising = toggle_pixel && !self.pixel_high;

        let reads = if byte_rising {
            read_u64(self.dut, "data_fifo_rd")? as u8
        } else {
            0
        };

        if toggle_ctrl {
            self.ctrl_high = !self.ctrl_high;
            self.next_ctrl += 30;
            self.dut.pin("ctrl_clk", u8::from(self.ctrl_high))?;
        }
        if toggle_byte {
            self.byte_high = !self.byte_high;
            self.next_byte += 30;
            self.dut.pin("byte_clk", u8::from(self.byte_high))?;
        }
        if toggle_pixel {
            self.pixel_high = !self.pixel_high;
            self.next_pixel += 25;
            self.dut.pin("pixel_clk", u8::from(self.pixel_high))?;
        }
        self.dut.eval();

        if ctrl_rising {
            self.ctrl_edges += 1;
        }
        if byte_rising {
            self.byte_edges += 1;
            self.lanes[0].byte_edge(reads & 1 != 0);
            self.lanes[1].byte_edge(reads & 2 != 0);
            self.drive_lane_heads()?;
            self.dut.eval();
        }
        if pixel_rising {
            self.pixel_edges += 1;
            if read_u64(self.dut, "pixel_valid")? != 0 {
                self.captured.push(CapturedBeat {
                    data: read_u64(self.dut, "pixel_data")? as u32,
                    sof: read_u64(self.dut, "pixel_sof")? != 0,
                    sol: read_u64(self.dut, "pixel_sol")? != 0,
                    eol: read_u64(self.dut, "pixel_eol")? != 0,
                    eof: read_u64(self.dut, "pixel_eof")? != 0,
                });
            }
        }
        Ok(())
    }

    fn run_events(&mut self, count: usize) -> TestResult {
        for _ in 0..count {
            self.step()?;
        }
        Ok(())
    }

    fn run_ctrl_edges(&mut self, count: usize) -> TestResult {
        let target = self.ctrl_edges + count;
        while self.ctrl_edges < target {
            self.step()?;
        }
        Ok(())
    }

    fn run_byte_edges(&mut self, count: usize) -> TestResult {
        assert!(
            self.byte_running,
            "cannot await edges of a stopped byte clock"
        );
        let target = self.byte_edges + count;
        while self.byte_edges < target {
            self.step()?;
        }
        Ok(())
    }

    fn stop_byte_clock(&mut self, high: bool) -> TestResult {
        while self.byte_high != high {
            self.step()?;
        }
        self.byte_running = false;
        Ok(())
    }

    fn resume_byte_clock(&mut self) {
        assert!(!self.byte_running);
        self.next_byte = self.now + 17;
        self.byte_running = true;
    }

    fn wait_output(&mut self, port: &str, value: u64, max_events: usize) -> TestResult {
        for _ in 0..max_events {
            if read_u64(self.dut, port)? == value {
                return Ok(());
            }
            self.step()?;
        }
        Err(format!(
            "timed out waiting for {port}={value:#x}; current={:#x}",
            read_u64(self.dut, port)?
        )
        .into())
    }

    fn wait_for_capture(&mut self, count: usize, max_events: usize) -> TestResult {
        for _ in 0..max_events {
            if self.captured.len() >= count {
                return Ok(());
            }
            self.step()?;
        }
        Err(format!(
            "timed out after capturing {} of {count} pixel beats; status={:#06x}",
            self.captured.len(),
            read_u64(self.dut, "status")?
        )
        .into())
    }

    fn set_clock_lp(&mut self, state: u8) -> TestResult {
        self.dut.pin("clk_lp_p", (state >> 1) & 1)?;
        self.dut.pin("clk_lp_n", state & 1)?;
        self.dut.eval();
        Ok(())
    }

    fn set_data_lp(&mut self, state: u8) -> TestResult {
        let p = if state & 2 != 0 { 3_u8 } else { 0_u8 };
        let n = if state & 1 != 0 { 3_u8 } else { 0_u8 };
        self.dut.pin("data_lp_p", p)?;
        self.dut.pin("data_lp_n", n)?;
        self.dut.eval();
        Ok(())
    }

    fn reset_and_acquire_link(&mut self) -> TestResult {
        self.run_events(24)?;
        self.dut.pin("reset_n", 1_u8)?;
        self.dut.eval();
        self.acquire_link()
    }

    fn acquire_link(&mut self) -> TestResult {
        self.wait_output("init_done", 1, 300)?;

        self.set_clock_lp(0b01)?;
        self.wait_output("clk_hs_term", 1, 200)?;
        self.set_clock_lp(0b00)?;
        self.wait_output("clk_hs_ena", 1, 200)?;
        self.wait_output("data_rst", 0, 300)?;
        self.run_ctrl_edges(8)
    }

    fn send_packet(
        &mut self,
        packet: &[u8],
        offsets: [usize; 2],
        lane1_skew: usize,
        bubble_seed: usize,
    ) -> TestResult {
        self.send_packet_open(packet, offsets, lane1_skew, bubble_seed)?;
        self.set_data_lp(0b11)?;
        self.wait_output("data_hs_ena", 0, 240)?;
        self.run_ctrl_edges(8)
    }

    // Keep LP00 after the packet's trail so a test can coordinate its end
    // with a clock-session transition.
    fn send_packet_open(
        &mut self,
        packet: &[u8],
        offsets: [usize; 2],
        lane1_skew: usize,
        bubble_seed: usize,
    ) -> TestResult {
        assert!(self.lanes.iter().all(LaneSource::drained));
        self.set_data_lp(0b11)?;
        self.run_ctrl_edges(6)?;
        self.set_data_lp(0b01)?;
        self.wait_output("data_hs_term", 3, 240)?;
        self.set_data_lp(0b00)?;
        self.wait_output("data_hs_ena", 3, 240)?;

        // Wait for the settle event and its toggle synchronizer to arm both
        // per-lane aligners before exposing hard-FIFO data.
        self.run_byte_edges(8)?;
        let lane0 = raw_lane_words(packet, 0, offsets[0]);
        let lane1 = raw_lane_words(packet, 1, offsets[1]);
        let bubbles0 = vec![19 + bubble_seed % 7, 211 + bubble_seed % 13];
        let mut bubbles1 = (0..lane1_skew).collect::<Vec<_>>();
        bubbles1.extend([73 + bubble_seed % 11, 389 + bubble_seed % 17]);
        self.lanes[0].load(lane0, bubbles0);
        self.lanes[1].load(lane1, bubbles1);
        self.drive_lane_heads()?;
        self.dut.eval();

        for _ in 0..12_000 {
            if self.lanes.iter().all(LaneSource::drained) {
                break;
            }
            self.step()?;
        }
        if !self.lanes.iter().all(LaneSource::drained) {
            return Err("timed out draining emulated hard-lane FIFOs".into());
        }
        self.run_byte_edges(12)
    }
}

fn raw_lane_words(packet: &[u8], lane: usize, offset: usize) -> Vec<u8> {
    assert!(lane < 2);
    assert!(offset < 8);
    assert!(packet.len().is_multiple_of(2));
    let mut aligned = Vec::with_capacity(1 + packet.len() / 2);
    aligned.push(0xb8);
    aligned.extend(packet.iter().copied().skip(lane).step_by(2));

    let mut bits = vec![false; offset];
    for byte in aligned {
        for bit in 0..8 {
            bits.push(byte & (1 << bit) != 0);
        }
    }
    // The aligner needs a following raw word to expose the final aligned byte.
    bits.extend([false; 16]);
    while !bits.len().is_multiple_of(8) {
        bits.push(false);
    }
    bits.chunks_exact(8)
        .map(|chunk| {
            chunk
                .iter()
                .enumerate()
                .fold(0_u8, |word, (bit, set)| word | (u8::from(*set) << bit))
        })
        .collect()
}

fn pixels_for_line(line: usize) -> Vec<u16> {
    (0..IMAGE_WIDTH)
        .map(|column| ((line * 97 + column * 29 + (column >> 3)) & 0x03ff) as u16)
        .collect()
}

#[test]
fn global_reset_clears_dirty_burst_events_with_stopped_byte_clock() -> TestResult {
    with_receiver(|dut| {
        let mut sim = Simulation::new(dut)?;
        sim.reset_and_acquire_link()?;

        // Exercise asynchronous assertion while the recovered clock is held
        // at either level. Three complete bursts leave every event toggle high.
        for (iteration, stopped_high) in [false, true].into_iter().enumerate() {
            let frame_start = build_short_packet(
                OV9281_VIRTUAL_CHANNEL,
                DATA_TYPE_FRAME_START,
                iteration as u16,
            )?;
            sim.send_packet(&frame_start, [0, 7], 3, 0)?;
            for line in 0..2 {
                let payload = pack_raw10(&pixels_for_line(line))?;
                let packet = build_long_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_RAW10, &payload)?;
                sim.send_packet(&packet, [line + 1, 6 - line], 2, line * 13)?;
            }
            sim.wait_output("burst_sources", 0xf, 100)?;
            sim.wait_output("burst_destinations", 0xfff, 100)?;
            assert_eq!(read_u64(sim.dut, "burst_pulses")?, 0);
            assert_eq!(read_u64(sim.dut, "status")?, 0);

            sim.stop_byte_clock(stopped_high)?;
            let stopped_edges = sim.byte_edges;
            sim.set_clock_lp(0b11)?;
            sim.set_data_lp(0b11)?;
            sim.dut.pin("reset_n", 0_u8)?;
            sim.dut.eval();
            // No clock edge is allowed to repair a stale destination toggle.
            assert_eq!(sim.byte_edges, stopped_edges);
            assert_eq!(read_u64(sim.dut, "burst_sources")?, 0);
            assert_eq!(read_u64(sim.dut, "burst_destinations")?, 0);
            assert_eq!(read_u64(sim.dut, "burst_pulses")?, 0);
            assert_eq!(read_u64(sim.dut, "pixel_valid")?, 0);
            assert_eq!(read_u64(sim.dut, "data_rst")?, 3);
            sim.run_ctrl_edges(8)?;
            sim.dut.pin("reset_n", 1_u8)?;
            sim.dut.eval();
            sim.run_ctrl_edges(12)?;
            assert_eq!(sim.byte_edges, stopped_edges);
            assert_eq!(read_u64(sim.dut, "burst_destinations")?, 0);
            assert_eq!(read_u64(sim.dut, "burst_pulses")?, 0);
            assert_eq!(read_u64(sim.dut, "data_rst")?, 3);

            sim.resume_byte_clock();
            sim.acquire_link()?;
            sim.captured.clear();
            let restart = build_short_packet(
                OV9281_VIRTUAL_CHANNEL,
                DATA_TYPE_FRAME_START,
                0x5100 + iteration as u16,
            )?;
            let payload = pack_raw10(&pixels_for_line(23 + iteration))?;
            let packet = build_long_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_RAW10, &payload)?;
            let mut oracle = Ov9281PacketReference::new();
            oracle.ingest_packet(&restart);
            let expected: Vec<_> = oracle
                .ingest_packet(&packet)
                .beats
                .into_iter()
                .map(CapturedBeat::from)
                .collect();
            sim.send_packet(&restart, [6, 1], 2, 9)?;
            sim.send_packet(&packet, [3, 4], 3, 37)?;
            sim.wait_for_capture(expected.len(), 20_000)?;
            assert_eq!(
                sim.captured, expected,
                "restart after byte clock held {stopped_high}"
            );
            assert_eq!(read_u64(sim.dut, "status")?, 0);

            // Return to a clean global epoch before dirtying all toggles again.
            sim.set_clock_lp(0b11)?;
            sim.dut.pin("reset_n", 0_u8)?;
            sim.dut.eval();
            sim.reset_and_acquire_link()?;
            sim.captured.clear();
        }
        Ok(())
    })
}

#[test]
fn session_reset_rebases_live_burst_events_without_phantom_packets() -> TestResult {
    with_receiver(|dut| {
        let mut sim = Simulation::new(dut)?;
        sim.reset_and_acquire_link()?;
        let old_frame = build_short_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_FRAME_START, 0x1200)?;
        sim.send_packet_open(&old_frame, [2, 5], 3, 0)?;
        sim.wait_output("burst_destinations", 0x03f, 100)?;
        assert_eq!(read_u64(sim.dut, "burst_sources")?, 0x3);

        // Only the receive session stops. Its source event toggles deliberately
        // survive. Ending the data burst on the same control edge changes the
        // end toggle as session reset asserts, requiring a live rebase rather
        // than either a zero reset or retention of the earlier synchronized value.
        sim.set_clock_lp(0b11)?;
        sim.set_data_lp(0b11)?;
        sim.wait_output("data_rst", 3, 300)?;
        sim.wait_output("clk_hs_ena", 0, 300)?;
        sim.run_ctrl_edges(32)?;
        assert_eq!(read_u64(sim.dut, "byte_session_active")?, 0);
        assert_eq!(read_u64(sim.dut, "burst_sources")?, 0xf);
        assert_eq!(read_u64(sim.dut, "burst_destinations")?, 0xfff);
        assert_eq!(read_u64(sim.dut, "burst_pulses")?, 0);
        assert!(sim.captured.is_empty());
        assert_eq!(read_u64(sim.dut, "status")?, 0);

        sim.acquire_link()?;
        sim.run_byte_edges(16)?;
        assert_eq!(read_u64(sim.dut, "burst_destinations")?, 0xfff);
        assert_eq!(read_u64(sim.dut, "burst_pulses")?, 0);
        assert!(sim.captured.is_empty());
        assert_eq!(read_u64(sim.dut, "status")?, 0);

        let restart = build_short_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_FRAME_START, 0x1201)?;
        let payload = pack_raw10(&pixels_for_line(41))?;
        let packet = build_long_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_RAW10, &payload)?;
        let mut oracle = Ov9281PacketReference::new();
        oracle.ingest_packet(&restart);
        let expected: Vec<_> = oracle
            .ingest_packet(&packet)
            .beats
            .into_iter()
            .map(CapturedBeat::from)
            .collect();
        sim.send_packet(&restart, [7, 0], 1, 4)?;
        sim.send_packet(&packet, [4, 3], 2, 29)?;
        sim.wait_for_capture(expected.len(), 20_000)?;
        assert_eq!(sim.captured, expected);
        assert_eq!(sim.captured.iter().filter(|beat| beat.sof).count(), 1);
        assert_eq!(read_u64(sim.dut, "status")?, 0);
        Ok(())
    })
}

#[test]
fn raw_hsio_receiver_covers_alignment_protocol_and_async_pixel_output() -> TestResult {
    with_receiver(|dut| {
        let mut sim = Simulation::new(dut)?;
        sim.reset_and_acquire_link()?;

        let mut oracle = Ov9281PacketReference::new();
        let mut expected = Vec::new();
        let frame_id = 0x4631;
        let frame_start =
            build_short_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_FRAME_START, frame_id)?;
        expected.extend(
            oracle
                .ingest_packet(&frame_start)
                .beats
                .into_iter()
                .map(CapturedBeat::from),
        );
        sim.send_packet(&frame_start, [0, 7], 3, 0)?;

        for line in 0..8 {
            let payload = pack_raw10(&pixels_for_line(line))?;
            assert_eq!(payload.len(), RAW10_BYTES_PER_LINE);
            let mut packet = build_long_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_RAW10, &payload)?;

            // One corrected header exercises correction-before-decode. A bad
            // footer on another line is diagnostic and must not retract data.
            if line == 4 {
                packet[1] ^= 1 << 3;
            }
            if line == 6 {
                let footer_msb = packet.len() - 1;
                packet[footer_msb] ^= 0x20;
            }
            let outcome = oracle.ingest_packet(&packet);
            if line == 4 {
                assert!(
                    outcome
                        .new_status
                        .contains(ReceiverStatus::HEADER_ECC_CORRECTED)
                );
                assert!(matches!(
                    parse_corrected_header(&packet),
                    HeaderCorrection::DataBit(_)
                ));
            }
            if line == 6 {
                assert!(
                    outcome
                        .new_status
                        .contains(ReceiverStatus::PAYLOAD_CRC_ERROR)
                );
            }
            expected.extend(outcome.beats.into_iter().map(CapturedBeat::from));
            sim.send_packet(&packet, [line, 7 - line], line.min(6), 17 * line)?;
            sim.wait_for_capture(expected.len(), 20_000)?;
        }

        assert_eq!(sim.captured, expected);
        assert_eq!(sim.captured.iter().filter(|beat| beat.sof).count(), 1);
        assert_eq!(sim.captured.iter().filter(|beat| beat.sol).count(), 8);
        assert_eq!(sim.captured.iter().filter(|beat| beat.eol).count(), 8);
        assert_eq!(sim.captured.iter().filter(|beat| beat.eof).count(), 0);

        // A double header-data-bit error is uncorrectable and aborts the open
        // frame without emitting another pixel beat.
        let payload = pack_raw10(&pixels_for_line(8))?;
        let mut double_error =
            build_long_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_RAW10, &payload)?;
        double_error[0] ^= 0x03;
        let expected_before_abort = sim.captured.len();
        let oracle_abort = oracle.ingest_packet(&double_error);
        assert!(oracle_abort.beats.is_empty());
        assert!(
            oracle_abort
                .new_status
                .contains(ReceiverStatus::HEADER_ECC_UNCORRECTABLE)
        );
        sim.send_packet(&double_error, [5, 2], 4, 91)?;
        sim.run_ctrl_edges(12)?;
        sim.run_byte_edges(24)?;
        assert_eq!(sim.captured.len(), expected_before_abort);

        let relevant_status = ReceiverStatus::HEADER_ECC_CORRECTED.bits()
            | ReceiverStatus::HEADER_ECC_UNCORRECTABLE.bits()
            | ReceiverStatus::PAYLOAD_CRC_ERROR.bits();
        let rtl_status = read_u64(sim.dut, "status")? as u16;
        assert_eq!(rtl_status, relevant_status, "unexpected receiver status");
        assert_eq!(rtl_status, oracle.status().bits());

        // The event scheduler models byte/control at 100 MHz and pixel at
        // 120 MHz. Over a long packet run the edge ratio converges exactly.
        let ratio_error = (sim.pixel_edges * 5).abs_diff(sim.byte_edges * 6);
        assert!(ratio_error <= 2, "clock ratio drifted: {ratio_error}");
        Ok(())
    })
}

fn parse_corrected_header(packet: &[u8]) -> HeaderCorrection {
    ov9281_csi2_rx::parse_packet(packet)
        .expect("packet with one header error must parse")
        .correction()
}

impl Simulation<'_, '_> {
    fn stage_packet(&mut self, packet: &[u8], offsets: [usize; 2], skew: usize) -> TestResult {
        assert!(self.lanes.iter().all(LaneSource::drained));
        self.set_data_lp(0b11)?;
        self.run_ctrl_edges(10)?;
        self.set_data_lp(0b01)?;
        self.wait_output("data_hs_term", 3, 240)?;
        self.set_data_lp(0b00)?;
        self.wait_output("data_hs_ena", 3, 240)?;
        self.run_byte_edges(20)?;
        self.lanes[0].load(raw_lane_words(packet, 0, offsets[0]), vec![]);
        self.lanes[1].load(raw_lane_words(packet, 1, offsets[1]), (0..skew).collect());
        self.drive_lane_heads()?;
        self.dut.eval();
        Ok(())
    }

    fn one_byte_edge_reset(&mut self) -> TestResult {
        self.dut.pin("reset_n", 0u8)?;
        self.dut.eval();
        self.lanes = [LaneSource::default(), LaneSource::default()];
        self.drive_lane_heads()?;
        self.set_data_lp(0b11)?;
        self.set_clock_lp(0b11)?;
        let before = self.byte_edges;
        self.run_byte_edges(1)?;
        assert_eq!(self.byte_edges - before, 1);
        self.dut.pin("reset_n", 1u8)?;
        self.dut.eval();
        self.captured.clear();
        self.acquire_link()
    }

    fn check_first_line_after_reset(&mut self, id: u16, pattern: usize) -> TestResult {
        let start = build_short_packet(OV9281_VIRTUAL_CHANNEL, DATA_TYPE_FRAME_START, id)?;
        let packet = build_long_packet(
            OV9281_VIRTUAL_CHANNEL,
            DATA_TYPE_RAW10,
            &pack_raw10(&pixels_for_line(pattern))?,
        )?;
        let mut oracle = Ov9281PacketReference::new();
        oracle.ingest_packet(&start);
        let expected: Vec<_> = oracle
            .ingest_packet(&packet)
            .beats
            .into_iter()
            .map(CapturedBeat::from)
            .collect();
        self.send_packet(&start, [6, 1], 2, pattern)?;
        self.send_packet(&packet, [3, 4], 3, pattern * 11)?;
        self.wait_for_capture(expected.len(), 20_000)?;
        assert_eq!(self.captured, expected, "first line after reset id={id:#x}");
        assert_eq!(read_u64(self.dut, "status")?, 0);
        Ok(())
    }
}

#[test]
fn dirty_single_byte_edge_reset_at_each_packet_word_position() -> TestResult {
    with_receiver(|dut| {
        let mut sim = Simulation::new(dut)?;
        sim.reset_and_acquire_link()?;
        // Header first/second pair, header-check boundary, every RAW10
        // gearbox phase, and footer pair. Reset occurs before the selected
        // pending pair can be consumed, with dirty prior stream history.
        let targets = [
            (0, 0),
            (1, 0),
            (4, 0),
            (2, 0),
            (2, 1),
            (2, 2),
            (2, 3),
            (2, 4),
            (3, 0),
        ];
        for (trial, (state, phase)) in targets.into_iter().enumerate() {
            if trial != 0 {
                sim.one_byte_edge_reset()?;
            }
            let start = build_short_packet(
                OV9281_VIRTUAL_CHANNEL,
                DATA_TYPE_FRAME_START,
                0x8100 + trial as u16,
            )?;
            sim.send_packet(&start, [trial % 8, 7 - trial % 8], trial % 6, trial)?;
            let packet = build_long_packet(
                OV9281_VIRTUAL_CHANNEL,
                DATA_TYPE_RAW10,
                &pack_raw10(&pixels_for_line(300 + trial))?,
            )?;
            sim.stage_packet(&packet, [(trial + 2) % 8, (trial + 5) % 8], trial % 5)?;
            let mut found = false;
            for _ in 0..20_000 {
                let actual = read_u64(sim.dut, "debug_parser_state")?;
                let actual_phase = read_u64(sim.dut, "debug_gearbox_phase")?;
                let pop = read_u64(sim.dut, "debug_pair_pop")?;
                if actual == state
                    && (state != 2 || actual_phase == phase)
                    && (state == 4 || pop == 1)
                {
                    found = true;
                    break;
                }
                sim.step()?;
            }
            assert!(
                found,
                "unreached reset position state={state} phase={phase}"
            );
            sim.one_byte_edge_reset()?;
            sim.check_first_line_after_reset(0x9100 + trial as u16, 400 + trial)?;
            println!(
                "RESET_POSITION_PASS trial={trial} state={state} phase={phase} held_byte_edges=1 recovery_beats=640"
            );
        }
        Ok(())
    })
}

#[test]
fn incomplete_and_malformed_bursts_preserve_next_header_and_recovery() -> TestResult {
    with_receiver(|dut| {
        let mut sim = Simulation::new(dut)?;
        sim.reset_and_acquire_link()?;
        let complete = build_long_packet(
            OV9281_VIRTUAL_CHANNEL,
            DATA_TYPE_RAW10,
            &pack_raw10(&pixels_for_line(719))?,
        )?;
        // Before/after both header pairs, each of the five initial payload
        // pairs, and the final payload/footer boundary. raw_lane_words adds
        // two zero trail bytes per lane; the actual burst boundary is retained.
        let cuts = [0usize, 2, 4, 6, 8, 10, 12, 14, 1602, 1604];
        let mut packets: Vec<Vec<u8>> = cuts.iter().map(|&n| complete[..n].to_vec()).collect();
        let mut bad_ecc = complete.clone();
        bad_ecc[0] ^= 3;
        packets.push(bad_ecc);
        packets.push(build_long_packet(
            1,
            DATA_TYPE_RAW10,
            &pack_raw10(&pixels_for_line(720))?,
        )?);
        packets.push(build_long_packet(0, DATA_TYPE_RAW10, &[0x57; 14])?);
        packets.push(build_short_packet(0, 2, 0xa35c)?.to_vec());
        for (trial, packet) in packets.iter().enumerate() {
            if trial != 0 {
                sim.one_byte_edge_reset()?;
            }
            let start = build_short_packet(0, DATA_TYPE_FRAME_START, 0xa100 + trial as u16)?;
            sim.send_packet(&start, [1, 6], 2, trial)?;
            sim.send_packet(packet, [trial % 8, 7 - trial % 8], trial % 6, trial * 17)?;
            sim.run_byte_edges(40)?;
            // Do not reset between malformed and fresh burst: this exposes
            // contamination across incomplete old/new header boundaries.
            let next = build_short_packet(0, DATA_TYPE_FRAME_START, 0xb100 + trial as u16)?;
            sim.send_packet(&next, [7, 0], 1, 91 + trial)?;
            sim.run_byte_edges(32)?;
            let status = read_u64(sim.dut, "status")?;
            assert_ne!(status, 0, "fault stimulus was not observed trial={trial}");
            sim.one_byte_edge_reset()?;
            sim.check_first_line_after_reset(0xc100 + trial as u16, 800 + trial)?;
            println!(
                "MALFORMED_PASS trial={trial} bytes={} status={status:#x} recovery_beats=640",
                packet.len()
            );
        }
        Ok(())
    })
}
