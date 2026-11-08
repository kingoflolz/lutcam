//! End-to-end Marlin/Verilator test for the canonical LWC encoder.
//!
//! Only the stable raster input and payload output ports are observed. The
//! production source manifest is the sole RTL closure; Marlin manages the
//! generated-model cache in a stable per-top artifact directory.

use std::error::Error;

use lwc::{Codec, FRAME_HEIGHT, FRAME_PIXELS, FRAME_WIDTH};
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};

mod rtl_support;

use rtl_support::{read_u64, with_model};

const ACTIVE_CLOCKS_PER_ROW: usize = FRAME_WIDTH / 2;
const HORIZONTAL_BLANK_CLOCKS: usize = 256;
const CLOCKS_PER_ROW: usize = ACTIVE_CLOCKS_PER_ROW + HORIZONTAL_BLANK_CLOCKS;
const VERTICAL_BLANK_ROWS: usize = 16;
const OUTPUT_WORDS: usize = FRAME_PIXELS / 16;
const MAX_POST_FRAME_DRAIN_CLOCKS: usize = 2 * FRAME_PIXELS;
const OUTPUT_QUIET_CLOCKS: usize = CLOCKS_PER_ROW;

const TOP_PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("in_sof", 0, 0, PortDirection::Input),
    ("in_valid", 0, 0, PortDirection::Input),
    ("in_px0", 7, 0, PortDirection::Input),
    ("in_px1", 7, 0, PortDirection::Input),
    ("out_sof", 0, 0, PortDirection::Output),
    ("out_valid", 0, 0, PortDirection::Output),
    ("out_data", 15, 0, PortDirection::Output),
    ("protocol_error", 0, 0, PortDirection::Output),
];

fn drive_idle(model: &mut DynamicVerilatedModel<'_>) -> Result<(), Box<dyn Error>> {
    model.pin("in_sof", 0_u8)?;
    model.pin("in_valid", 0_u8)?;
    model.pin("in_px0", 0_u8)?;
    model.pin("in_px1", 0_u8)?;
    Ok(())
}

fn clock(
    model: &mut DynamicVerilatedModel<'_>,
    output: &mut Vec<u16>,
) -> Result<(), Box<dyn Error>> {
    model.pin("clk", 0_u8)?;
    model.eval();
    model.pin("clk", 1_u8)?;
    model.eval();

    if read_u64(model, "protocol_error")? != 0 {
        return Err("RTL asserted protocol_error".into());
    }

    if read_u64(model, "out_valid")? != 0 {
        if output.len() == OUTPUT_WORDS {
            return Err("RTL emitted more than 64,000 output words".into());
        }
        let start = read_u64(model, "out_sof")? != 0;
        if start != output.is_empty() {
            return Err(format!(
                "out_sof={} on output word {}",
                u8::from(start),
                output.len()
            )
            .into());
        }
        output.push(u16::try_from(read_u64(model, "out_data")?)?);
    } else if read_u64(model, "out_sof")? != 0 {
        return Err("out_sof asserted without out_valid".into());
    }

    model.pin("clk", 0_u8)?;
    model.eval();
    Ok(())
}

fn check_frame(
    model: &mut DynamicVerilatedModel<'_>,
    pixels: &[u8],
    label: &str,
    reset_before_frame: bool,
) -> Result<(), Box<dyn Error>> {
    if pixels.len() != FRAME_PIXELS {
        return Err(format!(
            "test frame {label} has {} pixels, expected {FRAME_PIXELS}",
            pixels.len()
        )
        .into());
    }
    let expected = Codec::encode(pixels, FRAME_WIDTH, FRAME_HEIGHT)?;
    let mut output = Vec::with_capacity(OUTPUT_WORDS);

    model.pin("clk", 0_u8)?;
    drive_idle(model)?;
    if reset_before_frame {
        model.pin("rst", 1_u8)?;
        model.eval();
        for _ in 0..4 {
            clock(model, &mut output)?;
        }
    }
    model.pin("rst", 0_u8)?;

    for row in 0..FRAME_HEIGHT {
        for pair in 0..ACTIVE_CLOCKS_PER_ROW {
            let pixel = row * FRAME_WIDTH + pair * 2;
            model.pin("in_sof", u8::from(pixel == 0))?;
            model.pin("in_valid", 1_u8)?;
            model.pin("in_px0", pixels[pixel])?;
            model.pin("in_px1", pixels[pixel + 1])?;
            clock(model, &mut output)?;
        }
        drive_idle(model)?;
        for _ in 0..HORIZONTAL_BLANK_CLOCKS {
            clock(model, &mut output)?;
        }
    }

    drive_idle(model)?;
    for _ in 0..VERTICAL_BLANK_ROWS * CLOCKS_PER_ROW {
        clock(model, &mut output)?;
    }

    let mut drain_clocks = 0usize;
    while output.len() < OUTPUT_WORDS && drain_clocks < MAX_POST_FRAME_DRAIN_CLOCKS {
        clock(model, &mut output)?;
        drain_clocks += 1;
    }
    if output.len() != OUTPUT_WORDS {
        return Err(format!(
            "RTL frame {label} emitted {} of {OUTPUT_WORDS} words after {drain_clocks} drain clocks",
            output.len()
        )
        .into());
    }

    // Do not stop on the first apparent frame length. A full row period of
    // quiescence catches an accidental 64,001st word or repeated frame tail.
    for _ in 0..OUTPUT_QUIET_CLOCKS {
        clock(model, &mut output)?;
    }
    eprintln!("RTL frame {label}: {OUTPUT_WORDS} words, drain={drain_clocks} clocks");

    let actual = output
        .into_iter()
        .flat_map(u16::to_be_bytes)
        .collect::<Vec<_>>();
    if let Some(index) = actual
        .iter()
        .zip(expected.bytes())
        .position(|(actual, expected)| actual != expected)
    {
        let differing_bit_in_byte = (0..8)
            .find(|bit| ((actual[index] ^ expected.bytes()[index]) & (0x80 >> bit)) != 0)
            .unwrap_or(0);
        let differing_bit = index * 8 + differing_bit_in_byte;
        let packet_ends = Codec::packet_end_bits(&expected)?;
        let group = packet_ends.partition_point(|&end| end <= differing_bit);
        let packet_start = group
            .checked_sub(1)
            .map_or(0, |previous| packet_ends[previous]);
        let packet_end = packet_ends.get(group).copied().unwrap_or(expected.bit_len);
        let word = index / 2;
        let first_word = word.saturating_sub(4);
        let last_word = (word + 5).min(OUTPUT_WORDS);
        let actual_words = actual[first_word * 2..last_word * 2]
            .chunks_exact(2)
            .map(|bytes| u16::from_be_bytes([bytes[0], bytes[1]]))
            .collect::<Vec<_>>();
        let expected_words = expected.bytes()[first_word * 2..last_word * 2]
            .chunks_exact(2)
            .map(|bytes| u16::from_be_bytes([bytes[0], bytes[1]]))
            .collect::<Vec<_>>();
        return Err(format!(
            "RTL/software payload mismatch for {label} at byte {index} \
             (bit {differing_bit}, expected group {group}, packet \
             {packet_start}..{packet_end}): \
             RTL={:#04x}, software={:#04x}; words {first_word}..{last_word}: \
             RTL={actual_words:04x?}, software={expected_words:04x?}",
            actual[index],
            expected.bytes()[index]
        )
        .into());
    }
    if actual != expected.bytes() {
        return Err(format!(
            "RTL/software payload length mismatch for {label}: RTL={} bytes, software={} bytes",
            actual.len(),
            expected.bytes().len()
        )
        .into());
    }
    Ok(())
}

fn run_frames(frames: Vec<(&'static str, Vec<u8>)>) -> Result<(), Box<dyn Error>> {
    with_model(
        "codec_encoder_top",
        "codec_encoder_top",
        "rtl/lwc/codec_encoder_top.sv",
        TOP_PORTS,
        &[],
        |model| {
            for (index, (label, pixels)) in frames.into_iter().enumerate() {
                // Reset the model once. Later SOFs exercise the production
                // clear path after the preceding frame has fully drained.
                check_frame(model, &pixels, label, index == 0)?;
            }
            Ok(())
        },
    )
}

fn naturalish_frame() -> Vec<u8> {
    (0..FRAME_PIXELS)
        .map(|index| {
            let x = index % FRAME_WIDTH;
            let y = index / FRAME_WIDTH;
            let horizon = FRAME_HEIGHT * 3 / 7;
            let mut value = if y < horizon {
                42 + (y * 92 / horizon) as i32
            } else {
                76 + ((y - horizon) * 88 / (FRAME_HEIGHT - horizon)) as i32
            };

            let sun_x = FRAME_WIDTH * 3 / 4;
            let sun_y = FRAME_HEIGHT / 5;
            let dx = x as i32 - sun_x as i32;
            let dy = y as i32 - sun_y as i32;
            if dx * dx + dy * dy < 52 * 52 {
                value += 78;
            }
            if y > horizon && x > FRAME_WIDTH / 7 && x < FRAME_WIDTH / 7 + 180 {
                value -= 35;
            }
            if y > horizon + 100 && ((x / 96) & 1) == 0 {
                value += 12;
            }

            let texture = (x as u32)
                .wrapping_mul(0x9e37_79b1)
                .rotate_left((y & 15) as u32)
                ^ (y as u32).wrapping_mul(0x85eb_ca6b);
            value += ((texture >> 29) as i32) - 3;
            value.clamp(0, 255) as u8
        })
        .collect()
}

/// Horizontally constant two-row stripes. This legal raster used to overflow
/// the high-band pair queues even though its complete frame fit the bit budget.
fn two_row_stripes_frame() -> Vec<u8> {
    (0..FRAME_PIXELS)
        .map(|index| {
            let y = index / FRAME_WIDTH;
            if ((y / 2) & 1) != 0 { 255 } else { 0 }
        })
        .collect()
}

fn adversarial_frames() -> Vec<(&'static str, Vec<u8>)> {
    let checkerboard = (0..FRAME_PIXELS)
        .map(|index| {
            let x = index % FRAME_WIDTH;
            let y = index / FRAME_WIDTH;
            if (x + y) & 1 == 0 { 0 } else { 255 }
        })
        .collect();
    let stripes = (0..FRAME_PIXELS)
        .map(|index| {
            if (index / FRAME_WIDTH) & 1 == 0 {
                0
            } else {
                255
            }
        })
        .collect();
    let mut impulse = vec![128; FRAME_PIXELS];
    impulse[(FRAME_HEIGHT / 2) * FRAME_WIDTH + FRAME_WIDTH / 2] = 255;
    let mut state = 0x243f_6a88_85a3_08d3_u64;
    let noise = (0..FRAME_PIXELS)
        .map(|_| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state as u8
        })
        .collect();
    let mixed_stress = (0..FRAME_PIXELS)
        .map(|index| {
            let x = index % FRAME_WIDTH;
            let y = index / FRAME_WIDTH;
            let hash = ((x * 73) ^ (y * 151) ^ (x * y * 3) ^ ((x + 17) * (y + 29))) & 255;
            if y < 160 {
                if (x & 1) ^ (y & 1) != 0 { 255 } else { 0 }
            } else if y < 320 {
                hash as u8
            } else if y < 480 {
                if ((x >> 3) & 1) ^ ((y >> 2) & 1) != 0 {
                    0xe0
                } else {
                    0x20
                }
            } else if y < 640 {
                ((x + 3 * y + if x & 1 != 0 { 91 } else { 0 }) & 255) as u8
            } else if (x == FRAME_WIDTH / 2 && y == 3 * FRAME_HEIGHT / 4)
                || (x & 63 == 0 && y & 31 == 0)
            {
                255
            } else {
                128
            }
        })
        .collect();

    vec![
        ("all-zero", vec![0; FRAME_PIXELS]),
        ("all-255", vec![255; FRAME_PIXELS]),
        ("two-row-stripes", two_row_stripes_frame()),
        ("checkerboard", checkerboard),
        ("horizontal-stripes", stripes),
        ("impulse", impulse),
        ("deterministic-noise", noise),
        ("mixed-stress", mixed_stress),
    ]
}

#[test]
fn rtl_matches_software_on_naturalish_frame() -> Result<(), Box<dyn Error>> {
    run_frames(vec![("natural-ish", naturalish_frame())])
}

#[test]
fn rtl_handles_two_row_stripe_queue_pressure() -> Result<(), Box<dyn Error>> {
    run_frames(vec![("two-row-stripes", two_row_stripes_frame())])
}

#[test]
#[ignore = "long-running adversarial RTL coverage"]
fn rtl_matches_software_on_adversarial_frames() -> Result<(), Box<dyn Error>> {
    let mut frames = adversarial_frames();
    if let Ok(label) = std::env::var("RTL_FRAME") {
        frames.retain(|(candidate, _)| *candidate == label);
        if frames.is_empty() {
            return Err(format!("unknown adversarial RTL_FRAME={label}").into());
        }
    }
    run_frames(frames)
}
