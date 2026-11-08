//! Direct Marlin regression for the two-RAM LFTag candidate FIFO.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, clock, read_u64, with_model_sources};

const TOP: &str = "LfTagCandidateFifoRamMetadata";
const SOURCE: &str = "rtl/lftag/LfTagCandidateFifoRamMetadata.sv";
const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("in_valid", 0, 0, PortDirection::Input),
    ("in_group_end", 0, 0, PortDirection::Input),
    ("in_x_q4", 14, 0, PortDirection::Input),
    ("in_y_q4", 13, 0, PortDirection::Input),
    ("in_log_mass_q5_3", 7, 0, PortDirection::Input),
    ("in_centroid_valid", 0, 0, PortDirection::Input),
    ("in_scale", 1, 0, PortDirection::Input),
    ("in_frame_seq", 7, 0, PortDirection::Input),
    ("head_valid", 0, 0, PortDirection::Output),
    ("head_scale", 1, 0, PortDirection::Output),
    ("head_frame_seq", 7, 0, PortDirection::Output),
    ("head_protocol", 0, 0, PortDirection::Output),
    ("head_mass_spread", 7, 0, PortDirection::Output),
    ("head_pop", 0, 0, PortDirection::Input),
    ("desc_read_valid", 0, 0, PortDirection::Input),
    ("desc_read_index", 3, 0, PortDirection::Input),
    ("desc_read_data_valid", 0, 0, PortDirection::Output),
    ("desc_read_data", 39, 0, PortDirection::Output),
    ("desc_read_centroid_valid", 0, 0, PortDirection::Output),
    ("fifo_level", 5, 0, PortDirection::Output),
    ("fifo_full", 0, 0, PortDirection::Output),
    ("clear_diagnostics", 0, 0, PortDirection::Input),
    ("fifo_drop_sticky", 0, 0, PortDirection::Output),
    ("protocol_error_sticky", 0, 0, PortDirection::Output),
];

fn gateware_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn assert_success(description: &str, output: &Output) {
    assert!(
        output.status.success(),
        "{description} failed:\n{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
}

fn strict_lint(root: &Path, synthesis: bool) {
    let mut command = Command::new("verilator");
    command
        .current_dir(root)
        .args(["--lint-only", "--Wall", "--top-module", TOP]);
    if synthesis {
        command.arg("-DSYNTHESIS");
    }
    let output = command
        .arg(SOURCE)
        .output()
        .expect("failed to launch Verilator lint");
    assert_success("candidate-FIFO lint", &output);
}

fn pattern_x(group: usize, tuple: usize) -> u16 {
    ((group * 100 + tuple) & 0x7fff) as u16
}

fn pattern_y(group: usize, tuple: usize) -> u16 {
    ((group * 50 + tuple) & 0x3fff) as u16
}

fn pattern_mass(group: usize, tuple: usize) -> u8 {
    (32 + group + tuple) as u8
}

fn pattern_descriptor(group: usize, tuple: usize, valid: bool) -> u64 {
    (u64::from(valid) << 37)
        | (u64::from(pattern_mass(group, tuple)) << 29)
        | (u64::from(pattern_y(group, tuple)) << 15)
        | u64::from(pattern_x(group, tuple))
}

struct Driver<'model, 'ctx> {
    dut: &'model mut DynamicVerilatedModel<'ctx>,
}

impl<'model, 'ctx> Driver<'model, 'ctx> {
    fn new(dut: &'model mut DynamicVerilatedModel<'ctx>) -> TestResult<Self> {
        let mut driver = Self { dut };
        driver.idle()?;
        driver.dut.pin("head_pop", 0_u8)?;
        driver.dut.pin("desc_read_valid", 0_u8)?;
        driver.dut.pin("desc_read_index", 0_u8)?;
        driver.dut.pin("clear_diagnostics", 0_u8)?;
        driver.dut.pin("rst", 1_u8)?;
        for _ in 0..3 {
            clock(driver.dut)?;
        }
        driver.dut.pin("rst", 0_u8)?;
        clock(driver.dut)?;
        assert_eq!(read_u64(driver.dut, "fifo_level")?, 0);
        Ok(driver)
    }

    fn idle(&mut self) -> TestResult {
        self.dut.pin("in_valid", 0_u8)?;
        self.dut.pin("in_group_end", 0_u8)?;
        self.dut.pin("in_x_q4", 0_u16)?;
        self.dut.pin("in_y_q4", 0_u16)?;
        self.dut.pin("in_log_mass_q5_3", 0_u8)?;
        self.dut.pin("in_centroid_valid", 1_u8)?;
        self.dut.pin("in_scale", 0_u8)?;
        self.dut.pin("in_frame_seq", 0_u8)?;
        Ok(())
    }

    fn pin_tuple(&mut self, group: usize, tuple: usize, count: usize, mutation: u8) -> TestResult {
        self.dut.pin("in_valid", 1_u8)?;
        self.dut.pin("in_group_end", u8::from(tuple + 1 == count))?;
        self.dut.pin("in_x_q4", pattern_x(group, tuple))?;
        self.dut.pin("in_y_q4", pattern_y(group, tuple))?;
        self.dut
            .pin("in_log_mass_q5_3", pattern_mass(group, tuple))?;
        self.dut.pin(
            "in_centroid_valid",
            u8::from(!(mutation == 2 && tuple == 3)),
        )?;
        let mut scale = (group % 3) as u8;
        if mutation == 1 && tuple == 4 {
            scale = scale.wrapping_add(1) & 3;
        }
        self.dut.pin("in_scale", scale)?;
        self.dut.pin("in_frame_seq", group as u8)?;
        Ok(())
    }

    fn send_group(
        &mut self,
        group: usize,
        count: usize,
        mutation: u8,
        insert_idle: bool,
    ) -> TestResult {
        for tuple in 0..count {
            self.pin_tuple(group, tuple, count, mutation)?;
            clock(self.dut)?;
        }
        if insert_idle {
            self.idle()?;
            clock(self.dut)?;
        }
        Ok(())
    }

    fn check_head(&self, group: usize, count: usize, protocol: bool) -> TestResult {
        assert_eq!(read_u64(self.dut, "head_valid")?, 1, "group {group}");
        assert_eq!(read_u64(self.dut, "head_frame_seq")?, group as u64);
        assert_eq!(read_u64(self.dut, "head_scale")?, (group % 3) as u64);
        assert_eq!(read_u64(self.dut, "head_protocol")?, u64::from(protocol));
        assert_eq!(
            read_u64(self.dut, "head_mass_spread")?,
            u64::from(pattern_mass(group, count - 1) - pattern_mass(group, 0))
        );
        Ok(())
    }

    fn read_descriptor(
        &mut self,
        group: usize,
        tuple: usize,
        invalid_index: Option<usize>,
        pop: bool,
    ) -> TestResult {
        self.dut.pin("desc_read_valid", 1_u8)?;
        self.dut.pin("desc_read_index", tuple as u8)?;
        self.dut.pin("head_pop", u8::from(pop))?;
        clock(self.dut)?;
        let valid = invalid_index != Some(tuple);
        assert_eq!(read_u64(self.dut, "desc_read_data_valid")?, 1);
        let descriptor = read_u64(self.dut, "desc_read_data")?;
        assert_eq!(descriptor, pattern_descriptor(group, tuple, valid));
        assert_eq!(
            read_u64(self.dut, "desc_read_centroid_valid")?,
            u64::from(valid)
        );
        assert_eq!((descriptor >> 37) & 1, u64::from(valid));
        Ok(())
    }

    fn read_descriptors(
        &mut self,
        group: usize,
        count: usize,
        invalid_index: Option<usize>,
        pop: bool,
    ) -> TestResult {
        for tuple in 0..count {
            self.read_descriptor(group, tuple, invalid_index, pop && tuple + 1 == count)?;
        }
        self.dut.pin("desc_read_valid", 0_u8)?;
        self.dut.pin("head_pop", 0_u8)?;
        Ok(())
    }

    fn pop(&mut self) -> TestResult {
        self.dut.pin("head_pop", 1_u8)?;
        clock(self.dut)?;
        self.dut.pin("head_pop", 0_u8)?;
        Ok(())
    }
}

#[test]
fn ram_metadata_fifo_is_lint_clean_and_matches_its_cycle_contract() -> TestResult {
    let root = gateware_root();
    strict_lint(&root, false);
    strict_lint(&root, true);

    with_model_sources(
        &root,
        &[Path::new(SOURCE)],
        "lftag-candidate-fifo-rammeta-direct-v1",
        TOP,
        Path::new(SOURCE),
        PORTS,
        &[],
        &[],
        |dut| {
            let mut driver = Driver::new(dut)?;

            for tuple in 0..8 {
                driver.pin_tuple(0, tuple, 9, 0)?;
                clock(driver.dut)?;
                assert_eq!(read_u64(driver.dut, "fifo_level")?, 0);
                assert_eq!(read_u64(driver.dut, "head_valid")?, 0);
            }
            driver.pin_tuple(0, 8, 9, 0)?;
            clock(driver.dut)?;
            // The registered metadata read must not add a visible bubble.
            driver.check_head(0, 9, false)?;
            driver.idle()?;
            driver.read_descriptors(0, 9, None, true)?;

            // Back-to-back groups exercise continuous one-tuple-per-clock ingress.
            driver.send_group(1, 9, 0, false)?;
            driver.send_group(2, 9, 0, true)?;
            driver.check_head(1, 9, false)?;
            driver.read_descriptors(1, 9, None, true)?;
            driver.check_head(2, 9, false)?;
            driver.read_descriptors(2, 9, None, true)?;

            for (group, count, mutation, protocol, invalid) in [
                (3, 8, 0, true, None),
                (4, 10, 0, true, None),
                (5, 9, 1, true, None),
                (6, 9, 2, false, Some(3)),
            ] {
                driver.send_group(group, count, mutation, true)?;
                driver.check_head(group, count, protocol)?;
                if group == 4 || group == 6 {
                    driver.read_descriptors(group, 9, invalid, true)?;
                } else {
                    driver.pop()?;
                }
            }
            assert_eq!(read_u64(driver.dut, "protocol_error_sticky")?, 1);

            for group in 10..42 {
                driver.send_group(group, 9, 0, true)?;
            }
            assert_eq!(read_u64(driver.dut, "fifo_full")?, 1);
            assert_eq!(read_u64(driver.dut, "fifo_level")?, 32);
            driver.send_group(42, 9, 0, true)?;
            assert_eq!(read_u64(driver.dut, "fifo_drop_sticky")?, 1);
            assert_eq!(read_u64(driver.dut, "fifo_level")?, 32);

            driver.read_descriptors(10, 8, None, false)?;
            driver.pin_tuple(43, 0, 9, 0)?;
            driver.read_descriptor(10, 8, None, true)?;
            for tuple in 1..9 {
                driver.dut.pin("desc_read_valid", 0_u8)?;
                driver.dut.pin("head_pop", 0_u8)?;
                driver.pin_tuple(43, tuple, 9, 0)?;
                clock(driver.dut)?;
            }
            driver.idle()?;
            assert_eq!(read_u64(driver.dut, "fifo_full")?, 1);
            assert_eq!(read_u64(driver.dut, "fifo_level")?, 32);

            for group in 11..42 {
                driver.check_head(group, 9, false)?;
                driver.read_descriptors(group, 9, None, true)?;
            }
            driver.check_head(43, 9, false)?;
            driver.read_descriptors(43, 9, None, true)?;
            assert_eq!(read_u64(driver.dut, "fifo_level")?, 0);
            assert_eq!(read_u64(driver.dut, "head_valid")?, 0);

            driver.dut.pin("clear_diagnostics", 1_u8)?;
            clock(driver.dut)?;
            driver.dut.pin("clear_diagnostics", 0_u8)?;
            assert_eq!(read_u64(driver.dut, "fifo_drop_sticky")?, 0);
            assert_eq!(read_u64(driver.dut, "protocol_error_sticky")?, 0);
            Ok(())
        },
    )
}
