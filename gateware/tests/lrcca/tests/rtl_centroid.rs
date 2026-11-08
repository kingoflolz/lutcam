//! Cycle-accurate Marlin checks for the production centroid compressor.

use std::path::{Path, PathBuf};

use lrcca::{compress_centroid, log_m_q5_3};
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, clock, read_u64};

const WIDTH: usize = 1274;
const HEIGHT: usize = 794;
const PIXEL_BITS: usize = 8;
// Arbitrary (m, sx, sy) triples need not be realizable by a bounded 8-bit
// image.  Keep randomized endpoint moments inside the centered signed-port
// bounds; directed cases separately exercise the normalization extremes.
const RANDOM_MAX_MASS: u64 = 80_000_000;
const X_BITS: usize = 38;
const Y_BITS: usize = 37;
const LATENCY: usize = 3;
const ADAPTER: &str = "tests/lrcca/tests/rtl_adapters/core_centroid.sv";
const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("in_valid", 0, 0, PortDirection::Input),
    ("m", 27, 0, PortDirection::Input),
    ("sx", 37, 0, PortDirection::Input),
    ("sy", 36, 0, PortDirection::Input),
    ("out_valid", 0, 0, PortDirection::Output),
    ("centroid_valid", 0, 0, PortDirection::Output),
    ("x_q4", 14, 0, PortDirection::Output),
    ("y_q4", 13, 0, PortDirection::Output),
    ("m_log_q5_3", 7, 0, PortDirection::Output),
];

#[derive(Clone, Copy, Debug, Default)]
struct Input {
    valid: bool,
    m: u64,
    sx: u64,
    sy: u64,
}

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
        "lrcca-core-centroid",
        "CoreCentroid",
        Path::new(ADAPTER),
        PORTS,
        &[Path::new("rtl/lrcca")],
        &[adapter.as_path()],
        test,
    )
}

fn twos_complement(value: i128, width: usize) -> u64 {
    let minimum = -(1_i128 << (width - 1));
    let maximum = (1_i128 << (width - 1)) - 1;
    assert!((minimum..=maximum).contains(&value));
    (value & ((1_i128 << width) - 1)) as u64
}

fn drive(model: &mut DynamicVerilatedModel<'_>, input: Input) -> TestResult {
    let x2 = (input.sx as i128 * 2) - (input.m as i128 * (WIDTH - 1) as i128);
    let y2 = (input.sy as i128 * 2) - (input.m as i128 * (HEIGHT - 1) as i128);
    model.pin("in_valid", u8::from(input.valid))?;
    model.pin("m", input.m)?;
    model.pin("sx", twos_complement(x2, X_BITS))?;
    model.pin("sy", twos_complement(y2, Y_BITS))?;
    clock(model)
}

fn assert_error_bound(input: Input, x_q5: u64, y_q5: u64) {
    let x_error_numerator = (x_q5 as i128 * input.m as i128 - input.sx as i128 * 32).abs();
    let y_error_numerator = (y_q5 as i128 * input.m as i128 - input.sy as i128 * 32).abs();
    let bound = 9_i128 * 32 * input.m as i128;
    assert!(
        x_error_numerator * 100 < bound,
        "x error bound failed for {input:?}"
    );
    assert!(
        y_error_numerator * 100 < bound,
        "y error bound failed for {input:?}"
    );
}

fn check_output(
    model: &DynamicVerilatedModel<'_>,
    cycle: usize,
    source: Option<Input>,
) -> TestResult {
    let source = source.unwrap_or_default();
    assert_eq!(
        read_u64(model, "out_valid")?,
        u64::from(source.valid),
        "out_valid latency mismatch on cycle {cycle}",
    );

    let centroid_valid = read_u64(model, "centroid_valid")?;
    let x_q5 = read_u64(model, "x_q4")? << 1;
    let y_q5 = read_u64(model, "y_q4")? << 1;
    let mass_log = read_u64(model, "m_log_q5_3")?;
    if !source.valid || source.m == 0 {
        assert_eq!(
            (centroid_valid, x_q5, y_q5, mass_log),
            (0, 0, 0, 0),
            "invalid/zero input leaked data on cycle {cycle}",
        );
        return Ok(());
    }

    let expected = compress_centroid(
        source.m, source.sx, source.sy, WIDTH, HEIGHT, PIXEL_BITS, false,
    )?;
    assert_eq!(
        centroid_valid, 1,
        "nonzero centroid lost validity on cycle {cycle}"
    );
    assert_eq!(
        (x_q5, y_q5),
        (u64::from(expected.x_q5), u64::from(expected.y_q5))
    );
    assert_eq!(mass_log, u64::from(log_m_q5_3(source.m)));
    assert_eq!(x_q5 & 1, 0);
    assert_eq!(y_q5 & 1, 0);
    assert_error_bound(source, x_q5, y_q5);
    Ok(())
}

fn moment_from_centered(m: u64, x2: i64, y2: i64) -> Input {
    let x_numerator = x2 as i128 + m as i128 * (WIDTH - 1) as i128;
    let y_numerator = y2 as i128 + m as i128 * (HEIGHT - 1) as i128;
    assert_eq!(x_numerator & 1, 0);
    assert_eq!(y_numerator & 1, 0);
    Input {
        valid: true,
        m,
        sx: (x_numerator / 2) as u64,
        sy: (y_numerator / 2) as u64,
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
}

fn test_stream() -> Vec<Input> {
    let mut stream = vec![
        Input {
            valid: true,
            m: 1,
            sx: 0,
            sy: 0,
        },
        Input {
            valid: true,
            m: 2,
            sx: 1,
            sy: 1,
        },
        Input {
            valid: false,
            m: 123,
            sx: 456,
            sy: 789,
        },
        Input {
            valid: true,
            m: 0,
            sx: 0,
            sy: 0,
        },
        Input {
            valid: true,
            m: 65_537,
            sx: 17 * 65_537,
            sy: 23 * 65_537,
        },
        Input::default(),
        Input {
            valid: true,
            m: 1_234_567,
            sx: 901 * 1_234_567,
            sy: 513 * 1_234_567,
        },
    ];

    for exponent in 0..28 {
        let (m, x2, y2) = if exponent <= 4 {
            let mass = 1_u64 << exponent;
            (mass, -(mass as i64), mass as i64)
        } else {
            let scale = 1_i64 << (exponent - 4);
            let half = 1_i64 << (exponent - 5);
            (
                (1_u64 << exponent) + u64::from(exponent == 5),
                -37 * scale + half,
                29 * scale + half,
            )
        };
        stream.push(moment_from_centered(m, x2, y2));
    }
    for m in [39_u64, 7_837] {
        stream.push(Input {
            valid: true,
            m,
            sx: 0,
            sy: 0,
        });
        stream.push(Input {
            valid: true,
            m,
            sx: m * (WIDTH - 1) as u64,
            sy: m * (HEIGHT - 1) as u64,
        });
    }

    let mut rng = Rng(0x00ce_171d);
    for index in 0..2_048 {
        if index % 11 == 3 {
            stream.push(Input {
                valid: false,
                m: 77,
                sx: 88,
                sy: 99,
            });
        }
        let m = 1 + rng.next() % RANDOM_MAX_MASS;
        let sx = rng.next() % (m * (WIDTH - 1) as u64 + 1);
        let sy = rng.next() % (m * (HEIGHT - 1) as u64 + 1);
        stream.push(Input {
            valid: true,
            m,
            sx,
            sy,
        });
    }
    stream
}

#[test]
fn production_centroid_matches_rust_reference_and_three_cycle_protocol() -> TestResult {
    with_core(|model| {
        model.pin("clk", 0_u8)?;
        model.pin("rst", 1_u8)?;
        model.pin("in_valid", 0_u8)?;
        model.pin("m", 0_u32)?;
        model.pin("sx", 0_u64)?;
        model.pin("sy", 0_u64)?;
        model.eval();
        clock(model)?;
        model.pin("rst", 0_u8)?;

        let stream = test_stream();
        for (cycle, &input) in stream.iter().enumerate() {
            drive(model, input)?;
            check_output(
                model,
                cycle,
                cycle.checked_sub(LATENCY).map(|index| stream[index]),
            )?;
        }
        for flush in 0..8 {
            let cycle = stream.len() + flush;
            drive(model, Input::default())?;
            check_output(
                model,
                cycle,
                cycle
                    .checked_sub(LATENCY)
                    .and_then(|index| stream.get(index).copied()),
            )?;
        }
        Ok(())
    })
}
