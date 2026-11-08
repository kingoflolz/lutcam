//! Marlin tests for the coefficient-pair queue's public protocol behavior.

mod rtl_support;

use std::path::Path;

use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};

use rtl_support::{TestResult, clock, read_u64, with_model};

const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("clear_frame", 0, 0, PortDirection::Input),
    ("in_valid", 11, 0, PortDirection::Input),
    ("pair_ready", 0, 0, PortDirection::Input),
    ("protocol_error", 0, 0, PortDirection::Output),
];

const COMPACT_ADAPTER: &str = "tests/rtl_adapters/compact_coefficient_pair_queues_adapter.sv";

fn clock_and_read_error(model: &mut DynamicVerilatedModel<'_>) -> TestResult<bool> {
    clock(model)?;
    match read_u64(model, "protocol_error")? {
        0 => Ok(false),
        1 => Ok(true),
        value => Err(format!("one-bit protocol_error port has value {value}").into()),
    }
}

#[test]
fn band_one_capacity_guard_is_early_and_sticky() -> TestResult {
    // Coefficient and correction payloads do not feed any queue-capacity
    // state, so the dynamic model exposes only the relevant public controls.
    // This also avoids Marlin 0.16's broken FFI for Verilator VlWide inputs.
    with_model(
        "coefficient_pair_queues",
        "coefficient_pair_queues",
        "rtl/lwc/coefficient_pair_queues.sv",
        PORTS,
        &[],
        |model| {
            model.pin("clk", 0_u8)?;
            model.pin("rst", 1_u8)?;
            model.pin("clear_frame", 0_u8)?;
            model.pin("in_valid", 0_u16)?;
            model.pin("pair_ready", 0_u8)?;
            model.eval();
            for _ in 0..4 {
                assert!(
                    !clock_and_read_error(model)?,
                    "protocol_error asserted during reset"
                );
            }
            model.pin("rst", 0_u8)?;

            // Band one has 512 pair words. One coefficient arrives per clock
            // and one pair is written every other clock. The guard is
            // registered one clock after the final safe pair write, on the
            // first half of the next pair.
            model.pin("in_valid", 1_u16 << 1)?;
            for coefficient_count in 1..=1024 {
                assert!(
                    !clock_and_read_error(model)?,
                    "protocol_error asserted early at coefficient {coefficient_count}"
                );
            }
            assert!(
                clock_and_read_error(model)?,
                "protocol_error did not assert at coefficient 1025"
            );

            model.pin("in_valid", 0_u16)?;
            for idle_clock in 1..=16 {
                assert!(
                    clock_and_read_error(model)?,
                    "protocol_error lost stickiness after {idle_clock} idle clocks"
                );
            }

            Ok(())
        },
    )
}

#[test]
fn compact_profile_blocks_full_write_before_address_alias() -> TestResult {
    let adapter = Path::new(COMPACT_ADAPTER);
    with_model(
        "compact_coefficient_pair_queues_full_guard",
        "compact_coefficient_pair_queues_adapter",
        COMPACT_ADAPTER,
        PORTS,
        &[adapter],
        |model| {
            model.pin("clk", 0_u8)?;
            model.pin("rst", 1_u8)?;
            model.pin("clear_frame", 0_u8)?;
            model.pin("in_valid", 0_u16)?;
            model.pin("pair_ready", 0_u8)?;
            model.eval();
            for _ in 0..4 {
                assert!(!clock_and_read_error(model)?);
            }
            model.pin("rst", 0_u8)?;

            // Band zero's compact region has 64 pair positions.  The first
            // 128 coefficient writes are safe; coefficient 129 is suppressed
            // while the sticky error is raised, before address zero aliases.
            model.pin("in_valid", 1_u16 << 0)?;
            for coefficient_count in 1..=128 {
                assert!(
                    !clock_and_read_error(model)?,
                    "compact full guard asserted early at coefficient {coefficient_count}",
                );
            }
            assert!(
                clock_and_read_error(model)?,
                "compact full guard missed coefficient 129",
            );
            Ok(())
        },
    )
}

#[test]
fn compact_profile_blocks_two_writes_to_one_bank() -> TestResult {
    let adapter = Path::new(COMPACT_ADAPTER);
    with_model(
        "compact_coefficient_pair_queues_conflict_guard",
        "compact_coefficient_pair_queues_adapter",
        COMPACT_ADAPTER,
        PORTS,
        &[adapter],
        |model| {
            model.pin("clk", 0_u8)?;
            model.pin("rst", 1_u8)?;
            model.pin("clear_frame", 0_u8)?;
            model.pin("in_valid", 0_u16)?;
            model.pin("pair_ready", 0_u8)?;
            model.eval();
            for _ in 0..4 {
                assert!(!clock_and_read_error(model)?);
            }
            model.pin("rst", 0_u8)?;

            // Bands six and nine intentionally share compact bank zero.  This
            // pattern is outside the fixed transform color contract.  The
            // priority grant permits one physical write and fails visibly.
            model.pin("in_valid", (1_u16 << 6) | (1_u16 << 9))?;
            assert!(clock_and_read_error(model)?);
            Ok(())
        },
    )
}
