//! Marlin equivalence check for the two canonical causal transitions.

mod rtl_support;

use std::path::Path;

use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};

use rtl_support::{TestResult, read_u64, with_model};

const ADAPTER: &str = "tests/rtl_adapters/controller_profile_adapter.sv";
const NOMINAL_BITS: [u8; 12] = [139, 135, 102, 58, 59, 43, 27, 30, 21, 10, 13, 5];
const COARSE_ENERGY_SHIFT: [u8; 12] = [0, 2, 2, 0, 0, 0, 1, 1, 1, 2, 1, 0];
const INCREMENTAL_ENERGY_SHIFT: [u8; 12] = [3, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0];
const NIBBLE_CODES: [(u8, u8); 16] = [
    (0, 1),
    (10, 4),
    (58, 6),
    (24, 5),
    (25, 5),
    (126, 7),
    (59, 6),
    (26, 5),
    (11, 4),
    (60, 6),
    (127, 7),
    (61, 6),
    (27, 5),
    (62, 6),
    (28, 5),
    (4, 3),
];
const DEBT_SHIFT: u32 = 14;

const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("band", 3, 0, PortDirection::Input),
    ("pre_debt", 21, 0, PortDirection::Input),
    ("nibble", 3, 0, PortDirection::Input),
    ("nominal_bits", 7, 0, PortDirection::Output),
    ("coarse_energy_shift", 1, 0, PortDirection::Output),
    ("incremental_energy_shift", 1, 0, PortDirection::Output),
    ("coarse_cutoff", 3, 0, PortDirection::Output),
    ("incremental_cutoff", 3, 0, PortDirection::Output),
    ("huffman_length", 3, 0, PortDirection::Output),
    ("huffman_code", 6, 0, PortDirection::Output),
];

fn pin_signed_22(value: i64) -> u32 {
    assert!((-(1_i64 << 21)..(1_i64 << 21)).contains(&value));
    (value & ((1_i64 << 22) - 1)) as u32
}

#[test]
fn production_rtl_has_exact_prefix_table_and_bounded_headers() -> TestResult {
    with_model(
        "controller_profile_adapter",
        "controller_profile_adapter",
        ADAPTER,
        PORTS,
        &[Path::new(ADAPTER)],
        |model| {
            let mut kraft_units = 0u32;
            for (symbol, &(code, length)) in NIBBLE_CODES.iter().enumerate() {
                model.pin("nibble", symbol as u8)?;
                model.eval();
                assert_eq!(read_u64(model, "huffman_length")?, u64::from(length));
                assert_eq!(read_u64(model, "huffman_code")?, u64::from(code));
                kraft_units += 1u32 << (7 - length);
            }
            assert_eq!(kraft_units, 1 << 7, "prefix tree must be complete");
            assert_eq!(
                NIBBLE_CODES.iter().map(|&(_, length)| length).max(),
                Some(7)
            );

            let max_header = (1u32..=u16::MAX as u32)
                .filter_map(|mask| {
                    let bits = (0..4)
                        .map(|nibble| {
                            u32::from(NIBBLE_CODES[((mask >> (4 * nibble)) & 15) as usize].1)
                        })
                        .sum::<u32>();
                    (bits <= 16).then_some(3 + bits)
                })
                .max();
            assert_eq!(max_header, Some(19));
            Ok(())
        },
    )
}

#[test]
fn production_rtl_matches_both_software_transition_rules() -> TestResult {
    with_model(
        "controller_profile_adapter",
        "controller_profile_adapter",
        ADAPTER,
        PORTS,
        &[Path::new(ADAPTER)],
        |model| {
            for band in 0..12_u8 {
                model.pin("band", band)?;
                for octave in -8_i64..=8 {
                    for offset in [-1_i64, 0, 1] {
                        let pre_debt = (octave << DEBT_SHIFT) + offset;
                        model.pin("pre_debt", pin_signed_22(pre_debt))?;
                        model.eval();

                        assert_eq!(
                            read_u64(model, "nominal_bits")?,
                            u64::from(NOMINAL_BITS[usize::from(band)]),
                            "nominal table differs for band {band}",
                        );
                        assert_eq!(
                            read_u64(model, "coarse_energy_shift")?,
                            u64::from(COARSE_ENERGY_SHIFT[usize::from(band)]),
                            "coarse shift differs for band {band}",
                        );
                        assert_eq!(
                            read_u64(model, "incremental_energy_shift")?,
                            u64::from(INCREMENTAL_ENERGY_SHIFT[usize::from(band)]),
                            "incremental shift differs for band {band}",
                        );

                        let debt_octaves = pre_debt >> DEBT_SHIFT;
                        let expected_coarse = (5 + debt_octaves).clamp(0, 8) as u64;
                        let expected_incremental = (6 + debt_octaves).clamp(0, 8) as u64;
                        assert_eq!(
                            read_u64(model, "coarse_cutoff")?,
                            expected_coarse,
                            "coarse cutoff differs at pre_debt={pre_debt}",
                        );
                        assert_eq!(
                            read_u64(model, "incremental_cutoff")?,
                            expected_incremental,
                            "incremental cutoff differs at pre_debt={pre_debt}",
                        );
                    }
                }
            }
            Ok(())
        },
    )
}
