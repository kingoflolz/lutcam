//! Marlin equivalence coverage for the production two-pixels-per-clock core.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use lrcca::{BBox, ClassicalBBoxReference, TwoPpcReferenceModel, decode_bbox, sort_boxes};
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, clock, read_u64};

const WIDTH: usize = 48;
const HEIGHT: usize = 36;
const LARGE_WIDTH: usize = 160;
const LARGE_HEIGHT: usize = 80;
const ADAPTER: &str = "tests/lrcca/tests/rtl_adapters/core_2ppc.sv";
const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("hole_fill", 0, 0, PortDirection::Input),
    ("datavalid", 0, 0, PortDirection::Input),
    ("in_ready", 0, 0, PortDirection::Output),
    ("d0", 0, 0, PortDirection::Input),
    ("d1", 0, 0, PortDirection::Input),
    ("feature_d0", 0, 0, PortDirection::Input),
    ("feature_d1", 0, 0, PortDirection::Input),
    ("feature_y", 5, 0, PortDirection::Input),
    ("sof", 0, 0, PortDirection::Input),
    ("eol", 0, 0, PortDirection::Input),
    ("fv_valid", 0, 0, PortDirection::Output),
    ("fv_data", 34, 0, PortDirection::Output),
];

const LARGE_PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("hole_fill", 0, 0, PortDirection::Input),
    ("datavalid", 0, 0, PortDirection::Input),
    ("in_ready", 0, 0, PortDirection::Output),
    ("d0", 0, 0, PortDirection::Input),
    ("d1", 0, 0, PortDirection::Input),
    ("feature_d0", 0, 0, PortDirection::Input),
    ("feature_d1", 0, 0, PortDirection::Input),
    ("feature_y", 6, 0, PortDirection::Input),
    ("sof", 0, 0, PortDirection::Input),
    ("eol", 0, 0, PortDirection::Input),
    ("fv_valid", 0, 0, PortDirection::Output),
    ("fv_data", 43, 0, PortDirection::Output),
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
        "lrcca-core-2ppc",
        "Core2Ppc",
        Path::new(ADAPTER),
        PORTS,
        &[Path::new("rtl/lrcca")],
        &[adapter.as_path()],
        test,
    )
}

fn with_large_core(
    test: impl for<'ctx> FnOnce(&mut DynamicVerilatedModel<'ctx>) -> TestResult,
) -> TestResult {
    let root = gateware_root();
    let adapter = root.join(ADAPTER);
    rtl_test_support::with_model(
        &root,
        Path::new("rtl/lrcca/sources.f"),
        "lrcca-core-2ppc-160-hole-fill",
        "Core2Ppc160HoleFill",
        Path::new(ADAPTER),
        LARGE_PORTS,
        &[Path::new("rtl/lrcca")],
        &[adapter.as_path()],
        test,
    )
}

fn collect_box(
    model: &DynamicVerilatedModel<'_>,
    boxes: &mut Vec<BBox>,
    width: usize,
    height: usize,
) -> TestResult {
    if read_u64(model, "fv_valid")? != 0 {
        boxes.push(decode_bbox(
            read_u64(model, "fv_data")? as u128,
            width,
            height,
        ));
    }
    Ok(())
}

fn clear_stream(model: &mut DynamicVerilatedModel<'_>) -> TestResult {
    model.pin("datavalid", 0_u8)?;
    model.pin("d0", 0_u8)?;
    model.pin("d1", 0_u8)?;
    model.pin("feature_d0", 0_u8)?;
    model.pin("feature_d1", 0_u8)?;
    model.pin("feature_y", 0_u8)?;
    model.pin("sof", 0_u8)?;
    model.pin("eol", 0_u8)?;
    Ok(())
}

fn run_frame(
    model: &mut DynamicVerilatedModel<'_>,
    image: &[Vec<u8>],
    hole_fill: bool,
    width: usize,
    height: usize,
) -> TestResult<Vec<BBox>> {
    assert_eq!(image.len(), height);
    assert!(image.iter().all(|row| row.len() == width));

    model.pin("clk", 0_u8)?;
    model.pin("rst", 1_u8)?;
    model.pin("hole_fill", u8::from(hole_fill))?;
    clear_stream(model)?;
    model.eval();
    clock(model)?;
    clock(model)?;
    model.pin("rst", 0_u8)?;

    let mut boxes = Vec::new();
    for (y, row) in image.iter().enumerate() {
        for pair in 0..(width / 2) {
            while read_u64(model, "in_ready")? == 0 {
                clear_stream(model)?;
                clock(model)?;
                collect_box(model, &mut boxes, width, height)?;
            }
            let x = 2 * pair;
            model.pin("datavalid", 1_u8)?;
            model.pin("d0", row[x])?;
            model.pin("d1", row[x + 1])?;
            model.pin("feature_d0", row[x])?;
            model.pin("feature_d1", row[x + 1])?;
            model.pin("feature_y", y as u8)?;
            model.pin("sof", u8::from(y == 0 && pair == 0))?;
            model.pin("eol", u8::from(pair + 1 == width / 2))?;
            clock(model)?;
            collect_box(model, &mut boxes, width, height)?;
        }
    }

    clear_stream(model)?;
    for _ in 0..64 {
        clock(model)?;
        collect_box(model, &mut boxes, width, height)?;
    }
    Ok(boxes)
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

fn random_bordered_image(
    width: usize,
    height: usize,
    seed: u64,
    numerator: u64,
    denominator: u64,
) -> Vec<Vec<u8>> {
    let mut rng = Rng(seed);
    let mut image = vec![vec![0; width]; height];
    for row in image.iter_mut().take(height - 1).skip(1) {
        for pixel in row.iter_mut().take(width - 1).skip(1) {
            *pixel = u8::from(rng.next() % denominator < numerator);
        }
    }
    image
}

fn check_case(
    model: &mut DynamicVerilatedModel<'_>,
    image: &[Vec<u8>],
    hole_fill: bool,
    width: usize,
    height: usize,
) -> TestResult {
    let reference = TwoPpcReferenceModel::new(width, height, hole_fill)?;
    let expected = sort_boxes(&reference.boxes(image)?);
    let connectivity = if hole_fill { 8 } else { 4 };
    assert_eq!(
        expected,
        sort_boxes(&ClassicalBBoxReference::new(image, connectivity)?.boxes()),
        "Rust 2PPC and classical references disagree",
    );
    let actual = sort_boxes(&run_frame(model, image, hole_fill, width, height)?);
    assert_eq!(
        actual, expected,
        "production TwoPPC differs (hole_fill={hole_fill})"
    );
    Ok(())
}

#[test]
fn production_2ppc_matches_rust_references_for_directed_and_random_frames() -> TestResult {
    with_core(|model| {
        check_case(
            model,
            &random_bordered_image(WIDTH, HEIGHT, 41, 1, 5),
            false,
            WIDTH,
            HEIGHT,
        )?;
        check_case(
            model,
            &random_bordered_image(WIDTH, HEIGHT, 0x8c0c, 1, 5),
            true,
            WIDTH,
            HEIGHT,
        )?;

        let mut diagonal = vec![vec![0; WIDTH]; HEIGHT];
        diagonal[1][1] = 1;
        diagonal[2][2] = 1;
        diagonal[1][30] = 1;
        diagonal[2][29] = 1;
        check_case(model, &diagonal, true, WIDTH, HEIGHT)?;
        assert_eq!(
            sort_boxes(&run_frame(model, &diagonal, true, WIDTH, HEIGHT)?),
            vec![(1, 2, 1, 2, 2), (29, 30, 1, 2, 2)],
        );
        Ok(())
    })
}

#[test]
fn production_160x80_hole_fill_matches_rust_reference() -> TestResult {
    with_large_core(|model| {
        for seed in [0x8c0c, 0x8c0d, 0x8c0e] {
            check_case(
                model,
                &random_bordered_image(LARGE_WIDTH, LARGE_HEIGHT, seed, 1, 5),
                true,
                LARGE_WIDTH,
                LARGE_HEIGHT,
            )?;
        }
        Ok(())
    })
}

fn stress_for_30_seconds(model: &mut DynamicVerilatedModel<'_>, hole_fill: bool) -> TestResult {
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut seed = if hole_fill { 0x8c0c } else { 0x2c0c };
    let mut frames = 0_usize;
    while Instant::now() < deadline {
        check_case(
            model,
            &random_bordered_image(WIDTH, HEIGHT, seed, 1, 5),
            hole_fill,
            WIDTH,
            HEIGHT,
        )?;
        seed = seed.wrapping_add(0x9e37_79b9);
        frames += 1;
    }
    assert!(frames > 0);
    Ok(())
}

#[test]
#[ignore = "30-second randomized hardware stress"]
fn production_2ppc_30_second_stress() -> TestResult {
    with_core(|model| stress_for_30_seconds(model, false))
}

#[test]
#[ignore = "30-second randomized hardware stress"]
fn production_2ppc_hole_fill_30_second_stress() -> TestResult {
    with_core(|model| stress_for_30_seconds(model, true))
}
