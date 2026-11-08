use std::collections::VecDeque;

use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, read_u64};

use super::{Capture, collect};

pub(super) const CAMERA_PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("reset_n", 0, 0, PortDirection::Input),
    ("byte_clk", 0, 0, PortDirection::Input),
    ("core_clk", 0, 0, PortDirection::Input),
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
    ("receiver_status", 15, 0, PortDirection::Output),
    ("inner_fv_valid", 0, 0, PortDirection::Output),
    ("inner_fv_data", 32, 0, PortDirection::Output),
    ("hier_inner_end", 0, 0, PortDirection::Output),
    ("lwc_out_sof", 0, 0, PortDirection::Output),
    ("lwc_out_valid", 0, 0, PortDirection::Output),
    ("lwc_out_data", 15, 0, PortDirection::Output),
    ("lwc_protocol_error", 0, 0, PortDirection::Output),
    ("lftag_result_valid", 0, 0, PortDirection::Output),
    ("lftag_result_data", 31, 0, PortDirection::Output),
    ("lftag_result_last", 0, 0, PortDirection::Output),
    ("lftag_busy", 0, 0, PortDirection::Output),
    ("lftag_fifo_level", 5, 0, PortDirection::Output),
    ("lftag_candidate_start", 0, 0, PortDirection::Output),
    ("lftag_candidate_done", 0, 0, PortDirection::Output),
    ("lftag_candidate_cycles", 9, 0, PortDirection::Output),
    ("lftag_fifo_drop_sticky", 0, 0, PortDirection::Output),
    ("lftag_protocol_error_sticky", 0, 0, PortDirection::Output),
    ("threshold_protocol_error", 0, 0, PortDirection::Output),
    ("threshold_buffer_overrun", 0, 0, PortDirection::Output),
    ("threshold_frontend_overrun", 0, 0, PortDirection::Output),
    ("hierarchy_overrun", 0, 0, PortDirection::Output),
    ("hier_child_sort_key", 7, 0, PortDirection::Output),
    ("threshold_stream_hash", 63, 0, PortDirection::Output),
    ("threshold_stream_pairs", 31, 0, PortDirection::Output),
    ("lwc_group_schedule_hash", 63, 0, PortDirection::Output),
    ("lwc_group_schedule_count", 31, 0, PortDirection::Output),
    ("lwc_group_schedule_valid", 0, 0, PortDirection::Output),
    ("lwc_group_schedule_band", 3, 0, PortDirection::Output),
    ("lwc_raw_valid", 11, 0, PortDirection::Output),
    ("outer_table_overflows", 31, 0, PortDirection::Output),
    ("inner_table_overflows", 31, 0, PortDirection::Output),
    ("tap_overflow", 0, 0, PortDirection::Output),
    ("receiver_protocol_error", 0, 0, PortDirection::Output),
    ("receiver_marker_error", 0, 0, PortDirection::Output),
    ("camera_rows_received", 9, 0, PortDirection::Output),
    ("core_rows_started", 9, 0, PortDirection::Output),
    ("core_rows_completed", 9, 0, PortDirection::Output),
    ("camera_line_done", 0, 0, PortDirection::Output),
    ("core_row_start", 0, 0, PortDirection::Output),
    ("core_pair_valid", 0, 0, PortDirection::Output),
    ("core_frame_active", 0, 0, PortDirection::Output),
];

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

    fn empty_flag(&self) -> bool {
        self.words.is_empty() || self.bubble_cycles.binary_search(&self.cycle).is_ok()
    }

    fn head(&self) -> u8 {
        self.words.front().copied().unwrap_or(0)
    }

    fn byte_edge(&mut self, read: bool) {
        assert!(!(read && self.empty_flag()));
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

/// One scheduler tick is one sixth of a nanosecond. The recovered byte clock
/// toggles every 30 ticks (100 MHz) and the core/pixel clock every 25 (120 MHz).
pub(super) struct CameraSimulation<'model, 'context> {
    pub(super) dut: &'model mut DynamicVerilatedModel<'context>,
    pub(super) capture: Capture,
    pub(super) byte_edges: usize,
    pub(super) core_edges: usize,
    pub(super) packet_starts: Vec<usize>,
    pub(super) camera_line_core_cycles: Vec<usize>,
    pub(super) core_row_start_cycles: Vec<usize>,
    next_byte: u64,
    next_core: u64,
    byte_high: bool,
    core_high: bool,
    lanes: [LaneSource; 2],
    board_guards: bool,
}

impl<'model, 'context> CameraSimulation<'model, 'context> {
    pub(super) fn new(dut: &'model mut DynamicVerilatedModel<'context>) -> TestResult<Self> {
        dut.pin("reset_n", 0_u8)?;
        dut.pin("byte_clk", 0_u8)?;
        dut.pin("core_clk", 0_u8)?;
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
            capture: Capture::default(),
            byte_edges: 0,
            core_edges: 0,
            packet_starts: Vec::new(),
            camera_line_core_cycles: Vec::new(),
            core_row_start_cycles: Vec::new(),
            next_byte: 17,
            next_core: 25,
            byte_high: false,
            core_high: false,
            lanes: [LaneSource::default(), LaneSource::default()],
            board_guards: false,
        })
    }

    /// Preserve the board receiver's 12,000-clock initialization and its
    /// six-clock LPX / eleven-clock SOT guards. Source setup stays inside
    /// each fixed line interval; only initial link acquisition takes longer.
    pub(super) fn with_board_guards(
        dut: &'model mut DynamicVerilatedModel<'context>,
    ) -> TestResult<Self> {
        let mut simulation = Self::new(dut)?;
        simulation.board_guards = true;
        Ok(simulation)
    }

    fn drive_lane_heads(&mut self) -> TestResult {
        let empty =
            u8::from(self.lanes[0].empty_flag()) | (u8::from(self.lanes[1].empty_flag()) << 1);
        self.dut.pin("data0_hs_in", self.lanes[0].head())?;
        self.dut.pin("data1_hs_in", self.lanes[1].head())?;
        self.dut.pin("data_fifo_empty", empty)?;
        Ok(())
    }

    pub(super) fn step(&mut self) -> TestResult {
        let event_time = self.next_byte.min(self.next_core);
        let toggle_byte = self.next_byte == event_time;
        let toggle_core = self.next_core == event_time;
        let byte_rising = toggle_byte && !self.byte_high;
        let core_rising = toggle_core && !self.core_high;
        let reads = if byte_rising {
            read_u64(self.dut, "data_fifo_rd")? as u8
        } else {
            0
        };

        if toggle_byte {
            self.byte_high = !self.byte_high;
            self.next_byte += 30;
            self.dut.pin("byte_clk", u8::from(self.byte_high))?;
        }
        if toggle_core {
            self.core_high = !self.core_high;
            self.next_core += 25;
            self.dut.pin("core_clk", u8::from(self.core_high))?;
        }
        self.dut.eval();

        if byte_rising {
            self.byte_edges += 1;
            self.lanes[0].byte_edge(reads & 1 != 0);
            self.lanes[1].byte_edge(reads & 2 != 0);
            self.drive_lane_heads()?;
            self.dut.eval();
        }
        if core_rising {
            self.core_edges += 1;
            self.capture.clocks += 1;
            collect(self.dut, &mut self.capture)?;
            if read_u64(self.dut, "camera_line_done")? != 0 {
                self.camera_line_core_cycles.push(self.core_edges);
            }
            if read_u64(self.dut, "core_row_start")? != 0 {
                self.core_row_start_cycles.push(self.core_edges);
            }
            if read_u64(self.dut, "receiver_marker_error")? != 0 {
                return Err("receiver produced inconsistent SOF/SOL/EOL/EOF markers".into());
            }
        }
        Ok(())
    }

    pub(super) fn run_core_edges(&mut self, count: usize) -> TestResult {
        let target = self.core_edges + count;
        while self.core_edges < target {
            self.step()?;
        }
        Ok(())
    }

    pub(super) fn run_to_byte_edge(&mut self, target: usize) -> TestResult {
        while self.byte_edges < target {
            self.step()?;
        }
        Ok(())
    }

    fn run_byte_edges(&mut self, count: usize) -> TestResult {
        self.run_to_byte_edge(self.byte_edges + count)
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

    fn set_clock_lp(&mut self, state: u8) -> TestResult {
        self.dut.pin("clk_lp_p", (state >> 1) & 1)?;
        self.dut.pin("clk_lp_n", state & 1)?;
        self.dut.eval();
        Ok(())
    }

    fn set_data_lp(&mut self, state: u8) -> TestResult {
        self.dut
            .pin("data_lp_p", if state & 2 != 0 { 3_u8 } else { 0_u8 })?;
        self.dut
            .pin("data_lp_n", if state & 1 != 0 { 3_u8 } else { 0_u8 })?;
        self.dut.eval();
        Ok(())
    }

    pub(super) fn reset_and_acquire_link(&mut self) -> TestResult {
        self.run_core_edges(8)?;
        self.dut.pin("reset_n", 1_u8)?;
        self.dut.eval();
        self.wait_output("init_done", 1, if self.board_guards { 60_000 } else { 300 })?;
        self.set_clock_lp(0b01)?;
        if self.board_guards {
            self.run_core_edges(6)?;
        }
        self.wait_output("clk_hs_term", 1, 200)?;
        self.set_clock_lp(0b00)?;
        self.wait_output("clk_hs_ena", 1, 200)?;
        self.wait_output("data_rst", 0, 300)?;
        self.run_core_edges(8)
    }

    pub(super) fn send_packet_at(
        &mut self,
        packet: &[u8],
        offsets: [usize; 2],
        lane1_skew: usize,
        bubble_seed: usize,
        target_start: Option<usize>,
    ) -> TestResult<usize> {
        assert!(self.lanes.iter().all(LaneSource::drained));
        if let Some(target) = target_start {
            self.run_to_byte_edge(target.saturating_sub(if self.board_guards { 48 } else { 32 }))?;
        }

        self.set_data_lp(0b11)?;
        self.run_core_edges(if self.board_guards { 12 } else { 6 })?;
        self.set_data_lp(0b01)?;
        if self.board_guards {
            self.run_core_edges(6)?;
        }
        self.wait_output("data_hs_term", 3, 240)?;
        self.set_data_lp(0b00)?;
        self.wait_output("data_hs_ena", 3, 240)?;
        self.run_byte_edges(if self.board_guards { 16 } else { 8 })?;
        if let Some(target) = target_start {
            if self.byte_edges > target {
                return Err(format!(
                    "packet setup missed byte-edge target {target} at {}",
                    self.byte_edges
                )
                .into());
            }
            if target - self.byte_edges >= 48 {
                return Err("packet waited too long after opening the B8 search window".into());
            }
            self.run_to_byte_edge(target)?;
        }
        let actual_start = self.byte_edges;
        self.packet_starts.push(actual_start);

        let lane0 = raw_lane_words(packet, 0, offsets[0]);
        let lane1 = raw_lane_words(packet, 1, offsets[1]);
        let bubbles0 = vec![41 + bubble_seed % 7, 503 + bubble_seed % 13];
        let mut bubbles1 = (0..lane1_skew).collect::<Vec<_>>();
        bubbles1.extend([127 + bubble_seed % 11, 617 + bubble_seed % 17]);
        self.lanes[0].load(lane0, bubbles0);
        self.lanes[1].load(lane1, bubbles1);
        self.drive_lane_heads()?;
        self.dut.eval();

        for _ in 0..15_000 {
            if self.lanes.iter().all(LaneSource::drained) {
                break;
            }
            self.step()?;
        }
        if !self.lanes.iter().all(LaneSource::drained) {
            return Err("timed out draining emulated hard-lane FIFOs".into());
        }
        self.run_byte_edges(12)?;
        self.set_data_lp(0b11)?;
        self.wait_output("data_hs_ena", 0, 240)?;
        self.run_core_edges(8)?;
        Ok(actual_start)
    }

    pub(super) fn drain_outputs(
        &mut self,
        tag_events: usize,
        lwc_words: usize,
        max_core_edges: usize,
    ) -> TestResult<usize> {
        let start = self.core_edges;
        while (self.capture.tag_events.len() < tag_events
            || self.capture.lwc_words.len() < lwc_words)
            && self.core_edges - start < max_core_edges
        {
            self.step()?;
        }
        if self.capture.tag_events.len() != tag_events {
            return Err(format!(
                "camera pipeline emitted {} of {tag_events} TagDetect events",
                self.capture.tag_events.len()
            )
            .into());
        }
        if self.capture.lwc_words.len() != lwc_words {
            return Err(format!(
                "camera pipeline emitted {} of {lwc_words} LWC words",
                self.capture.lwc_words.len()
            )
            .into());
        }
        Ok(self.core_edges - start)
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
