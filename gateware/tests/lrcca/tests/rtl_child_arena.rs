//! Ready/valid and functional equivalence tests for the production ChildArena.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use lrcca::ChildArenaReferenceModel;
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, clock, read_u64};

const ENTRY_COUNT: usize = 108;
const MAX_LIST_LEN: usize = 9;
const WAIT_LIMIT: usize = 2_000;
const ADAPTER: &str = "tests/lrcca/tests/rtl_adapters/core_child_arena.sv";
const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("new_valid", 0, 0, PortDirection::Input),
    ("new_ready", 0, 0, PortDirection::Output),
    ("new_rsp_valid", 0, 0, PortDirection::Output),
    ("new_rsp_ready", 0, 0, PortDirection::Input),
    ("new_rsp_id", 6, 0, PortDirection::Output),
    ("new_rsp_status", 0, 0, PortDirection::Output),
    ("cmd_valid", 0, 0, PortDirection::Input),
    ("cmd_ready", 0, 0, PortDirection::Output),
    ("cmd_op", 1, 0, PortDirection::Input),
    ("cmd_id_a", 6, 0, PortDirection::Input),
    ("cmd_id_b", 6, 0, PortDirection::Input),
    ("cmd_fv", 33, 0, PortDirection::Input),
    ("out_valid", 0, 0, PortDirection::Output),
    ("out_ready", 0, 0, PortDirection::Input),
    ("out_fv", 33, 0, PortDirection::Output),
    ("out_first", 0, 0, PortDirection::Output),
    ("out_last", 0, 0, PortDirection::Output),
    ("idle", 0, 0, PortDirection::Output),
    ("free_count", 6, 0, PortDirection::Output),
];

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn with_core(
    test: impl for<'ctx> FnOnce(&mut DynamicVerilatedModel<'ctx>) -> TestResult,
) -> TestResult {
    let root = gateware_root();
    let adapter = root.join(ADAPTER);
    rtl_test_support::with_model(
        &root,
        Path::new("rtl/lrcca/sources.f"),
        "lrcca-core-child-arena",
        "CoreChildArena",
        Path::new(ADAPTER),
        PORTS,
        &[Path::new("rtl/lrcca")],
        &[adapter.as_path()],
        test,
    )
}

struct Driver<'model, 'ctx> {
    dut: &'model mut DynamicVerilatedModel<'ctx>,
    reference: ChildArenaReferenceModel<u64>,
    /// Hardware list ID to independently allocated Rust reference head.
    heads: BTreeMap<usize, usize>,
    cycles: usize,
}

impl<'model, 'ctx> Driver<'model, 'ctx> {
    fn new(dut: &'model mut DynamicVerilatedModel<'ctx>) -> TestResult<Self> {
        let mut driver = Self {
            dut,
            reference: ChildArenaReferenceModel::new(ENTRY_COUNT, MAX_LIST_LEN)?,
            heads: BTreeMap::new(),
            cycles: 0,
        };
        driver.reset()?;
        Ok(driver)
    }

    fn reset(&mut self) -> TestResult {
        self.reference = ChildArenaReferenceModel::new(ENTRY_COUNT, MAX_LIST_LEN)?;
        self.heads.clear();
        self.cycles = 0;
        self.dut.pin("clk", 0_u8)?;
        self.dut.pin("rst", 1_u8)?;
        self.dut.pin("new_valid", 0_u8)?;
        self.dut.pin("new_rsp_ready", 1_u8)?;
        self.dut.pin("cmd_valid", 0_u8)?;
        self.dut.pin("cmd_op", 0_u8)?;
        self.dut.pin("cmd_id_a", 0_u8)?;
        self.dut.pin("cmd_id_b", 0_u8)?;
        self.dut.pin("cmd_fv", 0_u64)?;
        self.dut.pin("out_ready", 1_u8)?;
        self.dut.eval();
        self.tick()?;
        self.dut.pin("rst", 0_u8)?;
        self.tick()?;
        self.check_state()
    }

    fn tick(&mut self) -> TestResult {
        clock(self.dut)?;
        self.cycles += 1;
        Ok(())
    }

    fn wait_high(&mut self, port: &str) -> TestResult {
        for _ in 0..WAIT_LIMIT {
            self.dut.eval();
            if read_u64(self.dut, port)? != 0 {
                return Ok(());
            }
            self.dut.pin("new_valid", 0_u8)?;
            self.dut.pin("cmd_valid", 0_u8)?;
            self.tick()?;
        }
        Err(format!(
            "ChildArena timed out waiting for {port} after {} cycles",
            self.cycles
        )
        .into())
    }

    fn wait_idle(&mut self) -> TestResult {
        self.wait_high("idle")
    }

    fn issue_command(&mut self, op: u8, id_a: usize, id_b: usize, fv: u64) -> TestResult {
        self.wait_high("cmd_ready")?;
        self.dut.pin("cmd_op", op)?;
        self.dut.pin("cmd_id_a", id_a as u8)?;
        self.dut.pin("cmd_id_b", id_b as u8)?;
        self.dut.pin("cmd_fv", fv)?;
        self.dut.pin("cmd_valid", 1_u8)?;
        self.tick()?;
        self.dut.pin("cmd_valid", 0_u8)?;
        Ok(())
    }

    fn new_list(&mut self, value: u64) -> TestResult<usize> {
        self.wait_high("new_ready")?;
        self.dut.pin("new_valid", 1_u8)?;
        self.tick()?;
        self.dut.pin("new_valid", 0_u8)?;
        self.wait_high("new_rsp_valid")?;
        assert_eq!(read_u64(self.dut, "new_rsp_status")?, 1);
        let hardware_id = read_u64(self.dut, "new_rsp_id")? as usize;
        assert!(hardware_id < ENTRY_COUNT);
        assert!(
            !self.heads.contains_key(&hardware_id),
            "hardware reallocated live ID"
        );

        let reference_id = self.reference.allocate()?;
        self.heads.insert(hardware_id, reference_id);
        self.issue_command(3, hardware_id, 0, value)?;
        self.wait_idle()?;
        self.reference.init(reference_id, value)?;
        self.check_state()?;
        Ok(hardware_id)
    }

    fn add(&mut self, hardware_id: usize, value: u64) -> TestResult {
        let reference_id = self.heads[&hardware_id];
        self.issue_command(0, hardware_id, 0, value)?;
        self.wait_idle()?;
        self.reference.add(reference_id, value)?;
        self.check_state()
    }

    fn merge_with_payload(
        &mut self,
        left: usize,
        right: usize,
        union_payload: u64,
    ) -> TestResult<usize> {
        let hardware_canonical = left.min(right);
        let hardware_old = left.max(right);
        let reference_left = self.heads[&hardware_canonical];
        let reference_right = self.heads[&hardware_old];
        self.issue_command(1, hardware_canonical, hardware_old, union_payload)?;
        self.wait_idle()?;
        let reference_canonical = self
            .reference
            .merge(Some(reference_left), Some(reference_right))?
            .expect("two live lists must merge to a live list");
        self.heads.remove(&hardware_canonical);
        self.heads.remove(&hardware_old);
        self.heads.insert(hardware_canonical, reference_canonical);
        self.check_state()?;
        Ok(hardware_canonical)
    }

    fn merge(&mut self, left: usize, right: usize) -> TestResult<usize> {
        self.merge_with_payload(left, right, 0)
    }

    fn read_and_free(&mut self, hardware_id: usize) -> TestResult {
        let reference_id = self
            .heads
            .remove(&hardware_id)
            .expect("read of non-live hardware ID");
        let expected = self.reference.read(reference_id)?;
        self.issue_command(2, hardware_id, 0, 0)?;

        let mut actual = Vec::new();
        let mut first_flags = Vec::new();
        let mut last_flags = Vec::new();
        let expect_output = !expected.features.is_empty();
        let mut finished = false;
        for _ in 0..WAIT_LIMIT {
            self.dut.eval();
            if read_u64(self.dut, "out_valid")? != 0 {
                actual.push(read_u64(self.dut, "out_fv")?);
                let first = read_u64(self.dut, "out_first")? != 0;
                let last = read_u64(self.dut, "out_last")? != 0;
                first_flags.push(first);
                last_flags.push(last);
                finished |= last;
            }
            self.tick()?;
            if (expect_output && finished) || (!expect_output && read_u64(self.dut, "idle")? != 0) {
                break;
            }
        }
        assert_eq!(
            actual, expected.features,
            "READ feature stream mismatch for ID {hardware_id}"
        );
        if expect_output {
            assert!(finished, "READ never emitted out_last for ID {hardware_id}");
            assert_eq!(first_flags.iter().filter(|&&flag| flag).count(), 1);
            assert_eq!(first_flags.first(), Some(&true));
            assert_eq!(last_flags.iter().filter(|&&flag| flag).count(), 1);
            assert_eq!(last_flags.last(), Some(&true));
        } else {
            assert!(first_flags.is_empty());
            assert!(last_flags.is_empty());
        }
        self.wait_idle()?;
        self.dut.eval();
        assert_eq!(
            read_u64(self.dut, "out_valid")?,
            0,
            "READ output persisted after idle"
        );
        self.check_state()
    }

    fn check_state(&mut self) -> TestResult {
        self.dut.eval();
        assert_eq!(
            read_u64(self.dut, "free_count")? as usize,
            self.reference.free_count,
            "free_count mismatch after {} clocks",
            self.cycles,
        );
        self.reference.check_invariants()?;
        Ok(())
    }

    fn drain(&mut self) -> TestResult {
        let live = self.heads.keys().copied().collect::<Vec<_>>();
        for id in live {
            self.read_and_free(id)?;
        }
        assert!(self.heads.is_empty());
        assert_eq!(self.reference.free_count, ENTRY_COUNT);
        self.check_state()
    }
}

#[derive(Clone, Copy)]
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        let mut value = self.0;
        value ^= value >> 12;
        value ^= value << 25;
        value ^= value >> 27;
        self.0 = value;
        value.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }

    fn pick(&mut self, values: &[usize]) -> usize {
        values[self.next() as usize % values.len()]
    }
}

fn randomized_sequence(driver: &mut Driver<'_, '_>, seed: u64, operations: usize) -> TestResult {
    let mut rng = Rng(seed);
    for _ in 0..operations {
        let live = driver.heads.keys().copied().collect::<Vec<_>>();
        let addable = live
            .iter()
            .copied()
            .filter(|id| {
                let len = driver.reference.features(driver.heads[id]).unwrap().len();
                len == MAX_LIST_LEN || driver.reference.free_count > 0
            })
            .collect::<Vec<_>>();

        let mut choices = Vec::new();
        if driver.reference.free_count > 0 {
            choices.extend([0_u8; 3]);
        }
        if !addable.is_empty() {
            choices.extend([1_u8; 8]);
        }
        if live.len() >= 2 {
            choices.extend([2_u8; 4]);
        }
        if !live.is_empty() {
            choices.extend([3_u8; 2]);
        }
        if choices.is_empty() {
            break;
        }

        match choices[rng.next() as usize % choices.len()] {
            0 => {
                driver.new_list(rng.next() % 2_048)?;
            }
            1 => {
                let id = rng.pick(&addable);
                driver.add(id, rng.next() % 2_048)?;
            }
            2 => {
                let left = rng.pick(&live);
                let mut right = rng.pick(&live);
                while right == left {
                    right = rng.pick(&live);
                }
                driver.merge(left, right)?;
            }
            3 => {
                let id = rng.pick(&live);
                driver.read_and_free(id)?;
            }
            _ => unreachable!(),
        }
    }
    driver.drain()
}

#[test]
fn randomized_command_sequences_match_rust_reference() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;
        for seed in [0xca11_da9e0, 0xca11_da9e1, 0xca11_da9e2] {
            randomized_sequence(&mut driver, seed, 300)?;
            driver.reset()?;
        }
        Ok(())
    })
}

#[test]
fn opcode_union_keeps_high_feature_payload_out_of_merge_ids() -> TestResult {
    with_core(|model| {
        let mut driver = Driver::new(model)?;
        let base = 1_u64 << 24;
        let left = driver.new_list(base + 10)?;
        for value in [30_u64, 50, 70, 90] {
            driver.add(left, base + value)?;
        }
        let right = driver.new_list(base + 20)?;
        for value in [40_u64, 60, 80] {
            driver.add(right, base + value)?;
        }
        let canonical = driver.merge_with_payload(left, right, base | 0x3ff)?;
        driver.read_and_free(canonical)?;
        assert_eq!(driver.reference.free_count, ENTRY_COUNT);
        driver.check_state()
    })
}
