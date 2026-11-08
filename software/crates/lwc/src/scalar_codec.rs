//! Canonical fixed-rate scalar packet codec.
//!
//! The encoder consumes 16-coefficient groups in the raw FPGA transform's
//! measured completion order. Each group is selected causally from its exact
//! fine/coarse packet lengths, an observable local distortion proxy, and one
//! signed frame-rate debt counter. An initially empty, 5.5 KiB token bucket
//! bounds production packet bursts without backpressure, refilling at two
//! bits per core clock. The external frame is exactly one bit per pixel:
//! 64,000 self-delimiting logical groups followed by zero padding. Compact
//! transport omits whole padding words without changing the group grammar.
//! The decoder performs only packet reconstruction and inverse 5/3.

use flate2::read::GzDecoder;
use std::error::Error;
use std::fmt;
use std::io::Read;
use std::sync::OnceLock;

use crate::transform::{Band, bands, forward_transform, inverse_transform};
use crate::{FRAME_HEIGHT, FRAME_PIXELS, FRAME_WIDTH};

const GROUP_LANES: usize = 16;
const GROUP_COUNT: usize = FRAME_PIXELS / GROUP_LANES;
const RATE_DEBT_SHIFT: u32 = 14;
const RATE_COARSE_CUTOFF_BASE: i64 = 5;
const RATE_INCREMENTAL_CUTOFF_BASE: i64 = 6;
const MAX_PAIR_TOKEN_BITS: usize = 17;
const NONZERO_PACKET_SERVICE_CYCLES: usize = 9;
const ZERO_RUN_MIN: usize = 15;
const ZERO_RUN_MAX: usize = 256;
const ZERO_RUN_TOKEN_BITS: usize = 15;
const ZERO_RUN_CREDIT_START: usize = ZERO_RUN_TOKEN_BITS + 1;
const FINE_RAW_PACKET_BITS: usize = 135;
const COARSE_RAW_PACKET_BITS: usize = 147;
const BURST_CAPACITY_BITS: usize = 45_056;
const BURST_REFILL_BITS_PER_CLOCK: usize = 2;
// Each 1 KiB of spent credit adds one slope-cutoff octave. At this capacity
// pressure ranges from zero to five; exact packet admission enforces the cap.
const BURST_CUTOFF_SHIFT: u32 = 13;

const FINE_SHIFT: [u8; 12] = [1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4];
const COARSE_SHIFT: [u8; 12] = [2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5];
const FINE_BIAS: [u8; 12] = [0, 0, 0, 0, 1, 3, 2, 3, 6, 6, 6, 6];
const COARSE_BIAS: [u8; 12] = [1, 1, 3, 3, 3, 7, 7, 7, 15, 15, 17, 15];
const FINE_RICE_ZERO: [u8; 12] = [4, 4, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0];
const FINE_RICE_SMALL: [u8; 12] = FINE_RICE_ZERO;
const FINE_RICE_LARGE: [u8; 12] = FINE_RICE_ZERO;
const FINE_RICE_THRESHOLD_EXP: [u8; 12] = [0; 12];
const COARSE_RICE_ZERO: [u8; 12] = [1, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0];
const COARSE_RICE_SMALL: [u8; 12] = [3, 1, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0];
const COARSE_RICE_LARGE: [u8; 12] = [2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0];
const COARSE_RICE_THRESHOLD_EXP: [u8; 12] = [0, 0, 0, 2, 2, 0, 1, 1, 0, 0, 0, 0];
const FINE_RECONSTRUCTION_GAIN_Q8: [u32; 12] = [
    512, 512, 1025, 1030, 1022, 2060, 2087, 2081, 4174, 4148, 4167, 4251,
];
const COARSE_RECONSTRUCTION_GAIN_Q8: [u32; 12] = [
    1023, 1029, 2051, 2069, 2054, 4124, 4163, 4171, 8236, 8410, 8139, 8019,
];
const FINE_RECONSTRUCTION_OFFSET_Q8: [i32; 12] =
    [125, 120, 354, 422, 82, -33, 102, -37, 1, 13, -8, -142];
const COARSE_RECONSTRUCTION_OFFSET_Q8: [i32; 12] = [
    57, 124, 165, 3, 9, -140, -172, -273, -683, -1028, -551, -655,
];
const NOMINAL_BITS: [usize; 12] = [139, 135, 102, 58, 59, 43, 27, 30, 21, 10, 13, 5];
const COARSE_ENERGY_SHIFT: [u8; 12] = [0, 2, 2, 0, 0, 0, 1, 1, 1, 2, 1, 0];
const INCREMENTAL_ENERGY_SHIFT: [u8; 12] = [3, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0];

// (canonical code, length), emitted MSB first.  Four low-to-high nibbles
// describe lane masks; the all-zero Huffman mask is reserved as raw escape
// after a nonzero packet tag.
const NIBBLE_CODES: [(u8, u8); 16] = [
    (0b0, 1),
    (0b1010, 4),
    (0b111010, 6),
    (0b11000, 5),
    (0b11001, 5),
    (0b1111110, 7),
    (0b111011, 6),
    (0b11010, 5),
    (0b1011, 4),
    (0b111100, 6),
    (0b1111111, 7),
    (0b111101, 6),
    (0b11011, 5),
    (0b111110, 6),
    (0b11100, 5),
    (0b100, 3),
];

const NIBBLE_LOOKUP: [(u8, u8); 128] = {
    let mut table = [(0, 0); 128];
    let mut symbol = 0;
    while symbol < NIBBLE_CODES.len() {
        let (code, length) = NIBBLE_CODES[symbol];
        let suffix_count = 1 << (7 - length);
        let mut suffix = 0;
        while suffix < suffix_count {
            table[((code as usize) << (7 - length)) | suffix] = (symbol as u8, length);
            suffix += 1;
        }
        symbol += 1;
    }
    table
};

const EXTERNAL_RAW_BITS_WITHOUT_QUALITY: usize = 134;
/// Stateless entry point for the fixed 1-bit-per-pixel codec.
/// Encoding defaults to the canonical 640-active/256-blank raster cadence.
/// Explicit group-decision intervals support other hardware input cadences.
#[derive(Clone, Copy, Debug, Default)]
pub struct Codec;

/// Headerless fixed-size compressed frame. Dimensions are out of band.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EncodedFrame {
    pub width: usize,
    pub height: usize,
    pub bytes: Vec<u8>,
    pub bit_len: usize,
}

/// Encoder-only diagnostics for the causal fixed-rate selector.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AllocationStats {
    pub fine_groups: usize,
    pub coarse_groups: usize,
    pub zero_groups: usize,
    pub min_rate_debt: i32,
    pub max_rate_debt: i32,
    pub used_output_bits: usize,
    pub padding_bits: usize,
    pub serializer_cycles: usize,
    pub max_packet_cycles: usize,
    pub nonzero_packets: usize,
    pub zero_run_tokens: usize,
    pub zero_run_bits: usize,
    pub zero_run_savings: usize,
    pub max_zero_run: usize,
}

impl EncodedFrame {
    pub fn from_bytes(width: usize, height: usize, bytes: Vec<u8>) -> Result<Self, CodecError> {
        let bit_len = validate_dimensions(width, height)?;
        let frame = Self {
            width,
            height,
            bytes,
            bit_len,
        };
        validate_payload_extent(&frame, bit_len)?;
        Ok(frame)
    }

    pub fn payload(&self) -> &[u8] {
        &self.bytes
    }
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }
    pub fn payload_bits(&self) -> usize {
        self.bit_len
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CodecError {
    InvalidDimensions {
        width: usize,
        height: usize,
        reason: &'static str,
    },
    PixelCountMismatch {
        expected: usize,
        actual: usize,
    },
    GroupTimingCountMismatch {
        expected: usize,
        actual: usize,
    },
    InvalidGroupTiming {
        group: usize,
        reason: &'static str,
    },
    SizeOverflow,
    BitstreamUnderflow {
        expected_bits: usize,
        available_bits: usize,
    },
    TrailingBits {
        expected_bits: usize,
        actual_bits: usize,
    },
    InvalidWordAlignment {
        byte_len: usize,
    },
    InvalidPageMap(&'static str),
    AllocationImpossible {
        block: usize,
        requested_quanta: usize,
        capacity_quanta: usize,
    },
    InvalidPacket {
        group: usize,
        reason: &'static str,
    },
    NonzeroPadding {
        position: usize,
    },
    Io(String),
}

impl fmt::Display for CodecError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDimensions {
                width,
                height,
                reason,
            } => write!(f, "unsupported {width}x{height} frame: {reason}"),
            Self::PixelCountMismatch { expected, actual } => {
                write!(f, "pixel count mismatch: expected {expected}, got {actual}")
            }
            Self::GroupTimingCountMismatch { expected, actual } => {
                write!(
                    f,
                    "group timing count mismatch: expected {expected}, got {actual}"
                )
            }
            Self::InvalidGroupTiming { group, reason } => {
                write!(f, "invalid group timing {group}: {reason}")
            }
            Self::SizeOverflow => f.write_str("frame dimensions overflow addressable memory"),
            Self::BitstreamUnderflow {
                expected_bits,
                available_bits,
            } => write!(
                f,
                "bitstream underflow: need {expected_bits} bits, only {available_bits} available"
            ),
            Self::TrailingBits {
                expected_bits,
                actual_bits,
            } => write!(
                f,
                "trailing payload: expected {expected_bits} bits, got {actual_bits}"
            ),
            Self::InvalidWordAlignment { byte_len } => write!(
                f,
                "compact payload must contain complete 16-bit words, got {byte_len} bytes"
            ),
            Self::InvalidPageMap(reason) => write!(f, "invalid deterministic group map: {reason}"),
            Self::AllocationImpossible {
                block,
                requested_quanta,
                capacity_quanta,
            } => write!(
                f,
                "frame allocation at group {block} needs {requested_quanta} bits (capacity {capacity_quanta})"
            ),
            Self::InvalidPacket { group, reason } => {
                write!(f, "invalid group packet {group}: {reason}")
            }
            Self::NonzeroPadding { position } => {
                write!(f, "nonzero frame padding at bit {position}")
            }
            Self::Io(message) => write!(f, "I/O error: {message}"),
        }
    }
}

impl Error for CodecError {}

#[derive(Clone)]
struct GroupCandidate {
    band: usize,
    fine: [i16; GROUP_LANES],
    coarse: [i16; GROUP_LANES],
    fine_bits: usize,
    coarse_bits: usize,
    fine_incremental_gain: u64,
    coarse_gain: u64,
}

impl Codec {
    pub fn encode(pixels: &[u8], width: usize, height: usize) -> Result<EncodedFrame, CodecError> {
        Self::encode_with_stats(pixels, width, height).map(|(frame, _)| frame)
    }

    /// Selects production burst limiting or the frame-budget-only mode.
    /// The decoder and fixed-size wire format are identical in both modes.
    #[doc(hidden)]
    pub fn encode_with_burst_mode(
        pixels: &[u8],
        width: usize,
        height: usize,
        burst_limit: bool,
    ) -> Result<EncodedFrame, CodecError> {
        Self::encode_with_stats_and_burst_mode(pixels, width, height, burst_limit)
            .map(|(frame, _)| frame)
    }

    /// Encodes one frame and returns the causal selection census from that pass.
    #[doc(hidden)]
    pub fn encode_with_stats(
        pixels: &[u8],
        width: usize,
        height: usize,
    ) -> Result<(EncodedFrame, AllocationStats), CodecError> {
        Self::encode_with_stats_and_burst_mode(pixels, width, height, true)
    }

    /// Encodes with an explicit burst policy, for explicit frame-budget experiments.
    #[doc(hidden)]
    pub fn encode_with_stats_and_burst_mode(
        pixels: &[u8],
        width: usize,
        height: usize,
        burst_limit: bool,
    ) -> Result<(EncodedFrame, AllocationStats), CodecError> {
        Self::encode_with_stats_for_timing(
            pixels,
            width,
            height,
            burst_limit.then(canonical_group_cycle_deltas),
        )
    }

    /// Encodes using measured group-decision intervals, in core clocks.
    /// The first interval starts at input SOF; subsequent intervals start at
    /// the preceding decision. This models clock-dependent burst admission
    /// when the raster cadence differs from the canonical 640/256 profile.
    #[doc(hidden)]
    pub fn encode_with_group_cycle_deltas(
        pixels: &[u8],
        width: usize,
        height: usize,
        group_cycle_deltas: &[u32],
    ) -> Result<EncodedFrame, CodecError> {
        Self::encode_with_stats_and_group_cycle_deltas(pixels, width, height, group_cycle_deltas)
            .map(|(frame, _)| frame)
    }

    /// Returns selection statistics for an explicitly timed production frame.
    #[doc(hidden)]
    pub fn encode_with_stats_and_group_cycle_deltas(
        pixels: &[u8],
        width: usize,
        height: usize,
        group_cycle_deltas: &[u32],
    ) -> Result<(EncodedFrame, AllocationStats), CodecError> {
        Self::encode_with_stats_for_timing(pixels, width, height, Some(group_cycle_deltas))
    }

    fn encode_with_stats_for_timing(
        pixels: &[u8],
        width: usize,
        height: usize,
        group_cycle_deltas: Option<&[u32]>,
    ) -> Result<(EncodedFrame, AllocationStats), CodecError> {
        let frame_bits = validate_dimensions(width, height)?;
        if pixels.len() != frame_bits {
            return Err(CodecError::PixelCountMismatch {
                expected: frame_bits,
                actual: pixels.len(),
            });
        }

        let mut coefficients = pixels
            .iter()
            .map(|&pixel| i32::from(pixel))
            .collect::<Vec<_>>();
        forward_transform(&mut coefficients, width, height);
        let band_list = bands(width, height);
        let groups = build_groups(&coefficients, &band_list)?;
        let (states, used_bits, stats) = allocate(&groups, frame_bits, group_cycle_deltas)?;

        let mut writer = BitWriter::with_capacity(frame_bits);
        let mut pending_zero_run = 0usize;
        for (group, &state) in groups.iter().zip(&states) {
            match state {
                2 | 1 => {
                    if pending_zero_run != 0 {
                        write_zero_run(&mut writer, pending_zero_run);
                        pending_zero_run = 0;
                    }
                    if state == 2 {
                        write_packet(&mut writer, &group.fine, true, group.band);
                    } else {
                        write_packet(&mut writer, &group.coarse, false, group.band);
                    }
                }
                _ => {
                    pending_zero_run += 1;
                    if pending_zero_run == ZERO_RUN_MAX {
                        write_zero_run(&mut writer, pending_zero_run);
                        pending_zero_run = 0;
                    }
                }
            }
        }
        if pending_zero_run != 0 {
            write_zero_run(&mut writer, pending_zero_run);
        }
        debug_assert_eq!(writer.position(), used_bits);
        if writer.position() > frame_bits {
            return Err(CodecError::AllocationImpossible {
                block: groups.len(),
                requested_quanta: writer.position(),
                capacity_quanta: frame_bits,
            });
        }
        writer.write_zeros(frame_bits - writer.position());
        Ok((
            EncodedFrame {
                width,
                height,
                bytes: writer.finish(),
                bit_len: frame_bits,
            },
            stats,
        ))
    }

    /// Runs the exact encoder analysis without materializing the output stream.
    #[doc(hidden)]
    pub fn allocation_stats(
        pixels: &[u8],
        width: usize,
        height: usize,
    ) -> Result<AllocationStats, CodecError> {
        Self::allocation_stats_with_burst_mode(pixels, width, height, true)
    }

    /// Runs the selector with an explicit burst-limited or frame-budget-only policy.
    #[doc(hidden)]
    pub fn allocation_stats_with_burst_mode(
        pixels: &[u8],
        width: usize,
        height: usize,
        burst_limit: bool,
    ) -> Result<AllocationStats, CodecError> {
        let frame_bits = validate_dimensions(width, height)?;
        if pixels.len() != frame_bits {
            return Err(CodecError::PixelCountMismatch {
                expected: frame_bits,
                actual: pixels.len(),
            });
        }
        let mut coefficients = pixels
            .iter()
            .map(|&pixel| i32::from(pixel))
            .collect::<Vec<_>>();
        forward_transform(&mut coefficients, width, height);
        let groups = build_groups(&coefficients, &bands(width, height))?;
        let (_, _, stats) = allocate(
            &groups,
            frame_bits,
            burst_limit.then(canonical_group_cycle_deltas),
        )?;
        Ok(stats)
    }

    /// Decodes the fixed-size RTL frame, including its required zero fill.
    /// Use [`Self::decode_compact`] for semantic words received over UDP.
    pub fn decode(frame: &EncodedFrame) -> Result<Vec<u8>, CodecError> {
        let frame_bits = validate_dimensions(frame.width, frame.height)?;
        validate_payload_extent(frame, frame_bits)?;
        Self::decode_payload(&frame.bytes, frame.width, frame.height, frame_bits, false)
    }

    /// Decodes only the semantic 16-bit words of one frame.
    ///
    /// The group schedule determines the endpoint: after the 64,000th logical
    /// group, only 0..15 zero bits may remain to complete the final word. Extra
    /// words, even zero words, are invalid. Dimensions remain fixed at 1280x800
    /// and the payload must not exceed the 128,000-byte frame extent.
    pub fn decode_compact(
        bytes: &[u8],
        width: usize,
        height: usize,
    ) -> Result<Vec<u8>, CodecError> {
        let frame_bits = validate_dimensions(width, height)?;
        let available_bits = bytes.len().checked_mul(8).ok_or(CodecError::SizeOverflow)?;
        if !bytes.len().is_multiple_of(2) {
            return Err(CodecError::InvalidWordAlignment {
                byte_len: bytes.len(),
            });
        }
        if available_bits > frame_bits {
            return Err(CodecError::TrailingBits {
                expected_bits: frame_bits,
                actual_bits: available_bits,
            });
        }
        Self::decode_payload(bytes, width, height, frame_bits, true)
    }

    fn decode_payload(
        bytes: &[u8],
        width: usize,
        height: usize,
        frame_bits: usize,
        compact: bool,
    ) -> Result<Vec<u8>, CodecError> {
        let layout = decode_group_layout();
        let mut coefficients = vec![0i32; frame_bits];
        let payload_bits = bytes.len() * 8;
        let mut reader = BitReader::new(bytes, payload_bits);

        let mut zero_run_remaining = 0usize;
        for (group_index, group) in layout.iter().enumerate() {
            let band = usize::from(group.band);
            let packet = if zero_run_remaining != 0 {
                zero_run_remaining -= 1;
                DecodedPacket::Zero
            } else {
                read_packet(&mut reader, group_index, band)?
            };
            if let DecodedPacket::ZeroRun(length) = packet {
                if length > GROUP_COUNT - group_index {
                    return Err(CodecError::InvalidPacket {
                        group: group_index,
                        reason: "zero run exceeds remaining group schedule",
                    });
                }
                zero_run_remaining = length - 1;
            } else if let DecodedPacket::Values { fine, values } = packet {
                let gain_q8 = if fine {
                    FINE_RECONSTRUCTION_GAIN_Q8[band]
                } else {
                    COARSE_RECONSTRUCTION_GAIN_Q8[band]
                };
                let offset_q8 = if fine {
                    FINE_RECONSTRUCTION_OFFSET_Q8[band]
                } else {
                    COARSE_RECONSTRUCTION_OFFSET_Q8[band]
                };
                // A sixteen-coefficient group crosses at most one band row.
                // These offsets are fixed by the normative frame schedule and
                // are shared by both camera decoder threads.
                let first = group.first as usize;
                let split = usize::from(group.first_row_lanes);
                for (output, &value) in coefficients[first..first + split]
                    .iter_mut()
                    .zip(&values[..split])
                {
                    *output = reconstruct_with_affine(value, gain_q8, offset_q8);
                }
                if split != GROUP_LANES {
                    let second = first + split + usize::from(group.row_skip);
                    for (output, &value) in coefficients[second..second + GROUP_LANES - split]
                        .iter_mut()
                        .zip(&values[split..])
                    {
                        *output = reconstruct_with_affine(value, gain_q8, offset_q8);
                    }
                }
            }
        }
        let expected_bits = reader.position().div_ceil(16) * 16;
        if compact && payload_bits != expected_bits {
            return Err(CodecError::TrailingBits {
                expected_bits,
                actual_bits: payload_bits,
            });
        }
        for position in reader.position()..payload_bits {
            if reader.read_bit_at(position) {
                return Err(CodecError::NonzeroPadding { position });
            }
        }
        inverse_transform(&mut coefficients, width, height);
        Ok(coefficients
            .into_iter()
            .map(|value| value.clamp(0, 255) as u8)
            .collect())
    }

    /// Returns the exclusive payload-bit end of every self-delimiting group
    /// packet. This is encoder/RTL diagnostic data, not part of the format.
    #[doc(hidden)]
    pub fn packet_end_bits(frame: &EncodedFrame) -> Result<Vec<usize>, CodecError> {
        let frame_bits = validate_dimensions(frame.width, frame.height)?;
        validate_payload_extent(frame, frame_bits)?;
        let schedule = raw_group_schedule();
        let mut reader = BitReader::new(&frame.bytes, frame.bit_len);
        let mut ends = Vec::with_capacity(GROUP_COUNT);
        let mut zero_run_remaining = 0usize;
        let mut run_end = 0usize;
        for (group, &band_id) in schedule.iter().enumerate() {
            if zero_run_remaining != 0 {
                zero_run_remaining -= 1;
                ends.push(run_end);
                continue;
            }
            let band = usize::from(band_id);
            if let DecodedPacket::ZeroRun(length) = read_packet(&mut reader, group, band)? {
                if length > GROUP_COUNT - group {
                    return Err(CodecError::InvalidPacket {
                        group,
                        reason: "zero run exceeds remaining group schedule",
                    });
                }
                zero_run_remaining = length - 1;
                run_end = reader.position();
            }
            ends.push(reader.position());
        }
        Ok(ends)
    }
}

fn observable_energy(
    source: i32,
    fine: i16,
    coarse: i16,
    correction: bool,
    fine_shift: u8,
    coarse_shift: u8,
) -> (u64, u64) {
    let magnitude = u64::from(fine.unsigned_abs());
    let error = source - (i32::from(fine) << fine_shift);
    let odd = magnitude & 1 != 0;
    let residual_class = if odd {
        error.unsigned_abs().saturating_mul(4) >= 1_u32 << fine_shift
    } else {
        i64::from(fine) * i64::from(error) >= 0
    };
    let coefficient = if odd {
        if residual_class { 6 } else { 2 }
    } else {
        4
    };
    let direct_base = 8 * magnitude * magnitude;
    let direct_adjustment = coefficient * magnitude;
    let direct_assist = if odd { correction } else { residual_class };
    let direct = if direct_assist {
        direct_base.saturating_add(direct_adjustment)
    } else {
        direct_base.saturating_sub(direct_adjustment)
    };

    let delta = coarse_shift - fine_shift;
    let residual = (i32::from(fine) - (i32::from(coarse) << delta)).unsigned_abs() as u64;
    let incremental = (8 * residual * residual).saturating_sub(coefficient * residual);
    (direct, incremental)
}

fn build_groups(
    coefficients: &[i32],
    band_list: &[Band],
) -> Result<Vec<GroupCandidate>, CodecError> {
    let schedule = raw_group_schedule();
    let mut band_groups = [0usize; 12];
    let mut result = Vec::with_capacity(GROUP_COUNT);
    for &band_id in schedule {
        let band = usize::from(band_id);
        let indices = group_indices(&band_list[band], band_groups[band], FRAME_WIDTH)?;
        band_groups[band] += 1;
        let fine: [i16; GROUP_LANES] = indices
            .map(|index| quantize_signed8(coefficients[index], FINE_SHIFT[band], FINE_BIAS[band]));
        let mut coarse = [0i16; GROUP_LANES];
        let mut direct_gain = 0u64;
        let mut incremental_gain = 0u64;
        for lane in 0..GROUP_LANES {
            let source = coefficients[indices[lane]];
            let (value, correction) = corrected_coarse(
                source,
                fine[lane],
                FINE_SHIFT[band],
                COARSE_SHIFT[band],
                COARSE_BIAS[band],
            );
            coarse[lane] = value;
            let (direct, incremental) = observable_energy(
                source,
                fine[lane],
                coarse[lane],
                correction,
                FINE_SHIFT[band],
                COARSE_SHIFT[band],
            );
            direct_gain = direct_gain.saturating_add(direct);
            incremental_gain = incremental_gain.saturating_add(incremental);
        }

        let fine_entropy = entropy_packet_bits(&fine, band, true);
        let coarse_entropy = entropy_packet_bits(&coarse, band, false);
        let fine_bits = external_packet_bits(&fine, fine_entropy, true);
        let coarse_bits = external_packet_bits(&coarse, coarse_entropy, false);
        let coarse_gain = direct_gain.saturating_sub(incremental_gain) << COARSE_ENERGY_SHIFT[band];
        let incremental_gain = incremental_gain << INCREMENTAL_ENERGY_SHIFT[band];
        result.push(GroupCandidate {
            band,
            fine,
            coarse,
            fine_bits,
            coarse_bits,
            fine_incremental_gain: incremental_gain,
            coarse_gain,
        });
    }
    if band_groups
        != [
            250, 250, 500, 1000, 1000, 1000, 4000, 4000, 4000, 16000, 16000, 16000,
        ]
    {
        return Err(CodecError::InvalidPageMap(
            "raw completion schedule has wrong band census",
        ));
    }
    Ok(result)
}

fn allocate(
    groups: &[GroupCandidate],
    frame_bits: usize,
    group_cycle_deltas: Option<&[u32]>,
) -> Result<(Vec<u8>, usize, AllocationStats), CodecError> {
    if let Some(deltas) = group_cycle_deltas {
        if deltas.len() != groups.len() {
            return Err(CodecError::GroupTimingCountMismatch {
                expected: groups.len(),
                actual: deltas.len(),
            });
        }
        if let Some(group) = deltas.iter().position(|&delta| delta == 0) {
            return Err(CodecError::InvalidGroupTiming {
                group,
                reason: "group decisions must advance core time",
            });
        }
    }
    // Reserve the mandatory zero tag for every group up front. Every causal
    // choice then consumes only its exact excess over that one-bit floor, so
    // overflow is impossible without a frame buffer or future lookahead.
    let mut extra_room =
        frame_bits
            .checked_sub(groups.len())
            .ok_or(CodecError::AllocationImpossible {
                block: 0,
                requested_quanta: groups.len(),
                capacity_quanta: frame_bits,
            })?;
    let mut states = Vec::with_capacity(groups.len());
    let mut total_used = 0usize;
    let mut debt = 0i64;
    let mut stats = AllocationStats::default();
    let mut pending_zero_run = 0usize;
    let mut logical_zero_run = 0usize;
    let mut burst_tokens = 0usize;

    for (index, group) in groups.iter().enumerate() {
        // Charge the current band's deterministic nominal allocation before
        // choosing its state. Arithmetic shift is mathematical floor division
        // by 32768 and maps directly to signed wires in RTL.
        let pre_debt = debt - NOMINAL_BITS[group.band] as i64;
        let available_burst_bits = group_cycle_deltas.map_or(usize::MAX, |deltas| {
            burst_tokens
                .saturating_add(
                    (deltas[index] as usize).saturating_mul(BURST_REFILL_BITS_PER_CLOCK),
                )
                .min(BURST_CAPACITY_BITS)
        });
        let (frame_coarse_cutoff, frame_incremental_cutoff) = rate_cutoffs(pre_debt);
        // Raise the existing energy/bit thresholds before hard admission is
        // exhausted. Keeping this term separate from signed frame debt means
        // a low-detail prefix cannot conceal local output congestion.
        let pressure = group_cycle_deltas.map_or(0, |_| burst_cutoff(available_burst_bits));
        let coarse_cutoff = frame_coarse_cutoff.max(pressure);
        let incremental_cutoff = frame_incremental_cutoff.max(pressure);

        let fine_extra = group.fine_bits - 1;
        let coarse_extra = group.coarse_bits - 1;
        let fine_fits = fine_extra <= extra_room && group.fine_bits <= available_burst_bits;
        let coarse_allowed = passes_slope(group.coarse_gain, coarse_extra, coarse_cutoff)
            && coarse_extra <= extra_room;
        let state = if coarse_allowed {
            if (group.fine_bits <= group.coarse_bits
                || passes_slope(
                    group.fine_incremental_gain,
                    group.fine_bits.saturating_sub(group.coarse_bits),
                    incremental_cutoff,
                ))
                && fine_fits
            {
                2
            } else if group.coarse_bits <= available_burst_bits {
                1
            } else {
                0
            }
        } else {
            0
        };
        let selected_bits = match state {
            2 => group.fine_bits,
            1 => group.coarse_bits,
            _ => 1,
        };
        if group_cycle_deltas.is_some() {
            // Charge the full logical packet, including a zero's mandatory
            // tag. Zero-run savings refund only the frame budget below; they
            // must not replenish burst credits. Physical padding is excluded.
            burst_tokens = available_burst_bits - selected_bits;
        }
        extra_room -= selected_bits - 1;
        total_used += selected_bits;
        debt = pre_debt + selected_bits as i64;
        if state == 0 {
            pending_zero_run += 1;
            logical_zero_run += 1;
            stats.max_zero_run = stats.max_zero_run.max(logical_zero_run);
            // At length sixteen the fixed fifteen-bit representation is
            // guaranteed. Credit that bit, and every later bit in the capped
            // chunk, immediately for the following causal decision.
            if pending_zero_run >= ZERO_RUN_CREDIT_START {
                extra_room += 1;
                total_used -= 1;
                debt -= 1;
                stats.zero_run_savings += 1;
            }
            if pending_zero_run == ZERO_RUN_MAX {
                stats.zero_run_tokens += 1;
                stats.zero_run_bits += ZERO_RUN_TOKEN_BITS;
                stats.serializer_cycles += 1;
                pending_zero_run = 0;
            }
        } else {
            logical_zero_run = 0;
            if pending_zero_run != 0 {
                let run_bits = zero_run_bits(pending_zero_run);
                stats.zero_run_tokens += 1;
                stats.zero_run_bits += run_bits;
                stats.serializer_cycles += 1;
                pending_zero_run = 0;
            }
            let packet_cycles = packet_service_cycles(selected_bits);
            stats.serializer_cycles += packet_cycles;
            stats.nonzero_packets += usize::from(selected_bits > 1);
        }
        let debt_i32 = i32::try_from(debt).map_err(|_| CodecError::SizeOverflow)?;
        stats.min_rate_debt = stats.min_rate_debt.min(debt_i32);
        stats.max_rate_debt = stats.max_rate_debt.max(debt_i32);
        match state {
            2 => stats.fine_groups += 1,
            1 => stats.coarse_groups += 1,
            _ => stats.zero_groups += 1,
        }
        // A zero packet is its one-bit tag. Every nonzero packet is serviced
        // in the fixed header-plus-eight-pair schedule used by the RTL,
        // irrespective of how many of its sixteen-bit output chunks are full.
        let packet_cycles = packet_service_cycles(selected_bits);
        stats.max_packet_cycles = stats.max_packet_cycles.max(packet_cycles);
        states.push(state);

        debug_assert!(total_used + groups.len() - index - 1 <= frame_bits);
    }

    if pending_zero_run != 0 {
        stats.zero_run_tokens += 1;
        stats.zero_run_bits += zero_run_bits(pending_zero_run);
        stats.serializer_cycles += 1;
    }

    stats.used_output_bits = total_used;
    stats.padding_bits = frame_bits - total_used;
    Ok((states, total_used, stats))
}

fn zero_run_bits(length: usize) -> usize {
    debug_assert!((1..=ZERO_RUN_MAX).contains(&length));
    if length < ZERO_RUN_MIN {
        length
    } else {
        ZERO_RUN_TOKEN_BITS
    }
}

fn rate_cutoffs(pre_debt: i64) -> (u8, u8) {
    let debt_octaves = pre_debt >> RATE_DEBT_SHIFT;
    (
        (RATE_COARSE_CUTOFF_BASE + debt_octaves).clamp(0, 8) as u8,
        (RATE_INCREMENTAL_CUTOFF_BASE + debt_octaves).clamp(0, 8) as u8,
    )
}

fn burst_cutoff(available_bits: usize) -> u8 {
    ((BURST_CAPACITY_BITS - available_bits) >> BURST_CUTOFF_SHIFT).min(8) as u8
}

fn packet_service_cycles(packet_bits: usize) -> usize {
    debug_assert!((1..=COARSE_RAW_PACKET_BITS).contains(&packet_bits));
    if packet_bits == 1 {
        1
    } else {
        NONZERO_PACKET_SERVICE_CYCLES
    }
}

fn quantize_signed8(value: i32, shift: u8, bias: u8) -> i16 {
    let magnitude = value.unsigned_abs();
    let rounded = ((magnitude + u32::from(bias)) >> shift) as i32;
    let signed = if value < 0 { -rounded } else { rounded };
    signed.clamp(i32::from(i8::MIN), i32::from(i8::MAX)) as i16
}

fn reconstruct_with_affine(value: i16, gain_q8: u32, offset_q8: i32) -> i32 {
    if value == 0 {
        return 0;
    }
    let magnitude = i64::from(value.unsigned_abs()) * i64::from(gain_q8) + i64::from(offset_q8);
    let reconstructed = ((magnitude.max(0) + 128) >> 8) as i32;
    if value < 0 {
        -reconstructed
    } else {
        reconstructed
    }
}

fn corrected_coarse(
    source: i32,
    fine: i16,
    fine_shift: u8,
    coarse_shift: u8,
    coarse_bias: u8,
) -> (i16, bool) {
    if fine == 0 {
        return (0, false);
    }
    let delta = coarse_shift - fine_shift;
    let derived = fine.unsigned_abs() >> delta;
    let independent = quantize_signed8(source, coarse_shift, coarse_bias).unsigned_abs();
    let correction = independent > derived;
    let magnitude = derived + u16::from(correction);
    let value = if fine < 0 {
        -(magnitude as i16)
    } else {
        magnitude as i16
    };
    (value, correction)
}

fn passes_slope(gain: u64, extra_bits: usize, cutoff: u8) -> bool {
    if cutoff == 0 {
        return true;
    }
    if extra_bits == 0 {
        return gain != 0;
    }
    let threshold = match cutoff {
        1 => 1,
        2 | 3 => 2,
        4 => 3,
        5 => 4,
        6 => 6,
        7 => 8,
        8 => 12,
        _ => unreachable!("causal cutoff is clamped to the hardware range"),
    };
    gain >= (extra_bits as u64).saturating_mul(threshold)
}

fn mask_huffman_bits(mask: u16) -> usize {
    (0..4)
        .map(|nibble| usize::from(NIBBLE_CODES[usize::from((mask >> (4 * nibble)) & 15)].1))
        .sum()
}

fn coefficient_rice_bits(value: i16, k: u8) -> usize {
    if value == 0 {
        return 0;
    }
    let remainder = value.unsigned_abs().saturating_sub(1);
    1 + 1 + usize::from(k) + usize::from(remainder >> k)
}

fn contextual_rice_k(band: usize, fine: bool, previous_magnitude: u16) -> u8 {
    let (zero, small, large, threshold_exp) = if fine {
        (
            FINE_RICE_ZERO[band],
            FINE_RICE_SMALL[band],
            FINE_RICE_LARGE[band],
            FINE_RICE_THRESHOLD_EXP[band],
        )
    } else {
        (
            COARSE_RICE_ZERO[band],
            COARSE_RICE_SMALL[band],
            COARSE_RICE_LARGE[band],
            COARSE_RICE_THRESHOLD_EXP[band],
        )
    };
    if previous_magnitude == 0 {
        zero
    } else if previous_magnitude > (1u16 << threshold_exp) {
        large
    } else {
        small
    }
}

fn entropy_packet_bits(values: &[i16; GROUP_LANES], band: usize, fine: bool) -> usize {
    let mask = lane_mask(values);
    if mask == 0 {
        return 1;
    }
    let mut previous_magnitude = 0u16;
    let mut coefficient_bits = 0usize;
    let mut pair_bits = 0usize;
    // Keep every normal pair token within the bounded 17-bit hardware append
    // profile. A group with a longer pair uses the existing raw escape;
    // raw pairs remain exactly sixteen bits.
    for (lane, &value) in values.iter().enumerate() {
        let rice_k = contextual_rice_k(band, fine, previous_magnitude);
        let length = coefficient_rice_bits(value, rice_k);
        coefficient_bits += length;
        pair_bits += length;
        previous_magnitude = value.unsigned_abs();
        if lane & 1 != 0 {
            if pair_bits > MAX_PAIR_TOKEN_BITS {
                return EXTERNAL_RAW_BITS_WITHOUT_QUALITY;
            }
            pair_bits = 0;
        }
    }
    2 + mask_huffman_bits(mask).min(16) + coefficient_bits
}

fn external_packet_bits(
    values: &[i16; GROUP_LANES],
    entropy_without_quality: usize,
    fine: bool,
) -> usize {
    if lane_mask(values) == 0 {
        1
    } else if entropy_without_quality >= EXTERNAL_RAW_BITS_WITHOUT_QUALITY {
        if fine {
            FINE_RAW_PACKET_BITS
        } else {
            COARSE_RAW_PACKET_BITS
        }
    } else {
        1 + entropy_without_quality
    }
}

fn lane_mask(values: &[i16; GROUP_LANES]) -> u16 {
    values.iter().enumerate().fold(0u16, |mask, (lane, value)| {
        mask | (u16::from(*value != 0) << lane)
    })
}

fn write_zero_run(writer: &mut BitWriter, length: usize) {
    debug_assert!((1..=ZERO_RUN_MAX).contains(&length));
    if length < ZERO_RUN_MIN {
        writer.write_zeros(length);
        return;
    }
    writer.write_bit(true);
    writer.write_bit(false);
    writer.write_bit(false);
    writer.write_zeros(4);
    writer.write_bits(u32::from((length & 0xff) as u8), 8);
}

fn write_packet(writer: &mut BitWriter, values: &[i16; GROUP_LANES], fine: bool, band: usize) {
    let mask = lane_mask(values);
    if mask == 0 {
        writer.write_bit(false);
        return;
    }
    let entropy = entropy_packet_bits(values, band, fine);
    writer.write_bit(true);
    writer.write_bit(fine);
    if entropy >= EXTERNAL_RAW_BITS_WITHOUT_QUALITY {
        // Fine raw occupies the impossible all-zero Huffman mask. Coarse raw
        // uses the impossible all-zero literal mask, leaving the shorter
        // coarse/Huffman prefix available for the universal zero-run escape.
        writer.write_bit(!fine);
        writer.write_zeros(if fine { 4 } else { 16 });
        for &value in values {
            writer.write_bits(u32::from(value as i8 as u8), 8);
        }
        return;
    }
    let huffman = mask_huffman_bits(mask) <= 16;
    writer.write_bit(!huffman);
    if huffman {
        for nibble in 0..4 {
            let symbol = usize::from((mask >> (4 * nibble)) & 15);
            let (code, length) = NIBBLE_CODES[symbol];
            writer.write_bits(u32::from(code), usize::from(length));
        }
    } else {
        writer.write_bits(u32::from(mask), 16);
    }
    let mut previous_magnitude = 0u16;
    for &value in values {
        let rice_k = contextual_rice_k(band, fine, previous_magnitude);
        previous_magnitude = value.unsigned_abs();
        if value == 0 {
            continue;
        }
        writer.write_bit(value < 0);
        let remainder = value.unsigned_abs() - 1;
        writer.write_zeros(usize::from(remainder >> rice_k));
        writer.write_bit(true);
        writer.write_bits(
            u32::from(remainder & ((1u16 << rice_k) - 1)),
            usize::from(rice_k),
        );
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DecodedPacket {
    Zero,
    ZeroRun(usize),
    Values {
        fine: bool,
        values: [i16; GROUP_LANES],
    },
}

fn read_packet(
    reader: &mut BitReader<'_>,
    group: usize,
    band: usize,
) -> Result<DecodedPacket, CodecError> {
    if !reader.read_bit()? {
        return Ok(DecodedPacket::Zero);
    }
    let fine = reader.read_bit()?;
    let raw_mask = reader.read_bit()?;
    let mask = if raw_mask {
        let mask = reader.read_bits(16)? as u16;
        if mask == 0 {
            if fine {
                return Err(CodecError::InvalidPacket {
                    group,
                    reason: "fine literal mask cannot be zero",
                });
            }
            let mut values = [0i16; GROUP_LANES];
            for value in &mut values {
                *value = i16::from(reader.read_bits(8)? as u8 as i8);
            }
            return Ok(DecodedPacket::Values { fine, values });
        }
        mask
    } else {
        let mut mask = 0u16;
        for nibble in 0..4 {
            mask |= u16::from(read_huffman_nibble(reader, group)?) << (4 * nibble);
        }
        if mask == 0 {
            if !fine {
                let count = usize::from(reader.read_bits(8)? as u8);
                return Ok(DecodedPacket::ZeroRun(if count == 0 {
                    ZERO_RUN_MAX
                } else {
                    count
                }));
            }
            let mut values = [0i16; GROUP_LANES];
            for value in &mut values {
                *value = i16::from(reader.read_bits(8)? as u8 as i8);
            }
            return Ok(DecodedPacket::Values { fine, values });
        }
        mask
    };
    let mut values = [0i16; GROUP_LANES];
    let mut previous_magnitude = 0u16;
    for (lane, value) in values.iter_mut().enumerate() {
        if mask & (1 << lane) == 0 {
            previous_magnitude = 0;
            continue;
        }
        let rice_k = contextual_rice_k(band, fine, previous_magnitude);
        let negative = reader.read_bit()?;
        let mut quotient = 0u16;
        while !reader.read_bit()? {
            quotient = quotient.checked_add(1).ok_or(CodecError::InvalidPacket {
                group,
                reason: "Rice quotient exceeds signed-eight profile",
            })?;
            if quotient > 255 {
                return Err(CodecError::InvalidPacket {
                    group,
                    reason: "Rice quotient exceeds signed-eight profile",
                });
            }
        }
        let low = reader.read_bits(usize::from(rice_k))? as u16;
        let remainder = (quotient << rice_k) | low;
        let magnitude = remainder + 1;
        if magnitude > 128 || (!negative && magnitude > 127) {
            return Err(CodecError::InvalidPacket {
                group,
                reason: "magnitude exceeds signed-eight profile",
            });
        }
        *value = if negative {
            -(magnitude as i16)
        } else {
            magnitude as i16
        };
        previous_magnitude = magnitude;
    }
    Ok(DecodedPacket::Values { fine, values })
}

fn read_huffman_nibble(reader: &mut BitReader<'_>, group: usize) -> Result<u8, CodecError> {
    if reader.bit_len - reader.position >= 7 {
        let (symbol, length) = NIBBLE_LOOKUP[reader.peek_short_bits(7) as usize];
        debug_assert!(length != 0, "the mask Huffman code covers every prefix");
        reader.position += usize::from(length);
        return Ok(symbol);
    }
    // A short final code must not require seven available bits. Keep the
    // scalar path at EOF, including its exact underflow position.
    let mut code = 0u8;
    for length in 1..=7 {
        code = (code << 1) | u8::from(reader.read_bit()?);
        if let Some(symbol) = NIBBLE_CODES
            .iter()
            .position(|&(expected, expected_length)| expected_length == length && expected == code)
        {
            return Ok(symbol as u8);
        }
    }
    Err(CodecError::InvalidPacket {
        group,
        reason: "invalid mask Huffman code",
    })
}

#[derive(Clone, Copy)]
struct DecodeGroup {
    first: u32,
    row_skip: u16,
    first_row_lanes: u8,
    band: u8,
}

fn decode_group_layout() -> &'static [DecodeGroup] {
    static LAYOUT: OnceLock<Vec<DecodeGroup>> = OnceLock::new();
    LAYOUT.get_or_init(|| {
        let band_list = bands(FRAME_WIDTH, FRAME_HEIGHT);
        let mut band_groups = [0; 12];
        raw_group_schedule()
            .iter()
            .map(|&band_id| {
                let band = &band_list[usize::from(band_id)];
                let ordinal = &mut band_groups[usize::from(band_id)];
                let indices = group_indices(band, *ordinal, FRAME_WIDTH)
                    .expect("embedded raw schedule fits each coefficient band");
                let column = *ordinal * GROUP_LANES % band.rect.width;
                *ordinal += 1;
                DecodeGroup {
                    first: indices[0] as u32,
                    row_skip: (FRAME_WIDTH - band.rect.width) as u16,
                    first_row_lanes: (band.rect.width - column).min(GROUP_LANES) as u8,
                    band: band_id,
                }
            })
            .collect()
    })
}

fn group_indices(
    band: &Band,
    group: usize,
    width: usize,
) -> Result<[usize; GROUP_LANES], CodecError> {
    let flat_start = group
        .checked_mul(GROUP_LANES)
        .ok_or(CodecError::SizeOverflow)?;
    if flat_start + GROUP_LANES > band.rect.width * band.rect.height {
        return Err(CodecError::InvalidPageMap("raw schedule exceeds a band"));
    }
    Ok(std::array::from_fn(|lane| {
        let flat = flat_start + lane;
        (band.rect.y + flat / band.rect.width) * width + band.rect.x + flat % band.rect.width
    }))
}

/// Normative coefficient-group order for the fixed LWC profile.
///
/// Hardware regressions use this to prove that input clock elasticity cannot
/// change the encoded bitstream's implicit band order.
pub fn raw_group_schedule() -> &'static [u8] {
    static SCHEDULE: OnceLock<Vec<u8>> = OnceLock::new();
    SCHEDULE.get_or_init(|| {
        // Decompressed SHA-256:
        // 0b5126bb22c24c296fc5981c64c07d54a8f45da53b4c67f902faa9eec3ddc29a
        let compressed = decode_base64(include_str!("raw_group_schedule.b64"));
        let mut decoder = GzDecoder::new(compressed.as_slice());
        let mut text = String::new();
        decoder
            .read_to_string(&mut text)
            .expect("embedded raw schedule gzip is valid");
        let schedule = text
            .lines()
            .map(|line| {
                line.parse::<u8>()
                    .expect("embedded raw schedule contains band IDs")
            })
            .collect::<Vec<_>>();
        assert_eq!(schedule.len(), GROUP_COUNT);
        schedule
    })
}

/// Canonical group-decision timing for 640 active plus 256 blank clocks per
/// row. Each interval is measured from the preceding decision, except the
/// first, which is measured from input SOF. Timing affects encoder quality
/// admission, while group order and the decoder wire format remain fixed.
#[doc(hidden)]
pub fn canonical_group_cycle_deltas() -> &'static [u32] {
    static DELTAS: OnceLock<Vec<u32>> = OnceLock::new();
    DELTAS.get_or_init(|| {
        let compressed = decode_base64(include_str!("canonical_group_cycle_deltas.b64"));
        let mut decoder = GzDecoder::new(compressed.as_slice());
        let mut bytes = Vec::new();
        decoder
            .read_to_end(&mut bytes)
            .expect("embedded group timing gzip is valid");
        assert_eq!(bytes.len(), GROUP_COUNT * 4);
        let deltas = bytes
            .chunks_exact(4)
            .map(|chunk| u32::from_le_bytes(chunk.try_into().unwrap()))
            .collect::<Vec<_>>();
        assert!(deltas.iter().all(|&delta| delta != 0));
        deltas
    })
}

fn decode_base64(text: &str) -> Vec<u8> {
    let mut output = Vec::with_capacity(text.len() * 3 / 4);
    let mut accumulator = 0u32;
    let mut bits = 0usize;
    for byte in text.bytes().filter(|byte| !byte.is_ascii_whitespace()) {
        if byte == b'=' {
            break;
        }
        let value = match byte {
            b'A'..=b'Z' => byte - b'A',
            b'a'..=b'z' => byte - b'a' + 26,
            b'0'..=b'9' => byte - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => panic!("invalid base64 in raw group schedule"),
        };
        accumulator = (accumulator << 6) | u32::from(value);
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            output.push((accumulator >> bits) as u8);
            accumulator &= (1u32 << bits).wrapping_sub(1);
        }
    }
    output
}

fn validate_dimensions(width: usize, height: usize) -> Result<usize, CodecError> {
    if (width, height) != (FRAME_WIDTH, FRAME_HEIGHT) {
        return Err(CodecError::InvalidDimensions {
            width,
            height,
            reason: "the canonical profile is fixed at 1280x800",
        });
    }
    width.checked_mul(height).ok_or(CodecError::SizeOverflow)
}

fn validate_payload_extent(frame: &EncodedFrame, expected_bits: usize) -> Result<(), CodecError> {
    let available_bits = frame
        .bytes
        .len()
        .checked_mul(8)
        .ok_or(CodecError::SizeOverflow)?;
    if available_bits < expected_bits {
        return Err(CodecError::BitstreamUnderflow {
            expected_bits,
            available_bits,
        });
    }
    if available_bits != expected_bits || frame.bit_len != expected_bits {
        return Err(CodecError::TrailingBits {
            expected_bits,
            actual_bits: available_bits.max(frame.bit_len),
        });
    }
    Ok(())
}

struct BitWriter {
    bytes: Vec<u8>,
    position: usize,
}

impl BitWriter {
    fn with_capacity(bits: usize) -> Self {
        Self {
            bytes: vec![0; bits.div_ceil(8)],
            position: 0,
        }
    }
    fn position(&self) -> usize {
        self.position
    }
    fn write_bit(&mut self, value: bool) {
        if value {
            self.bytes[self.position / 8] |= 1 << (7 - self.position % 8);
        }
        self.position += 1;
    }
    fn write_bits(&mut self, value: u32, bits: usize) {
        for bit in (0..bits).rev() {
            self.write_bit(((value >> bit) & 1) != 0);
        }
    }
    fn write_zeros(&mut self, bits: usize) {
        self.position += bits;
    }
    fn finish(self) -> Vec<u8> {
        self.bytes
    }
}

#[derive(Clone, Copy)]
struct BitReader<'a> {
    bytes: &'a [u8],
    bit_len: usize,
    position: usize,
}

impl<'a> BitReader<'a> {
    fn new(bytes: &'a [u8], bit_len: usize) -> Self {
        Self {
            bytes,
            bit_len,
            position: 0,
        }
    }
    fn position(self) -> usize {
        self.position
    }
    fn read_bit(&mut self) -> Result<bool, CodecError> {
        if self.position >= self.bit_len {
            return Err(CodecError::BitstreamUnderflow {
                expected_bits: self.position + 1,
                available_bits: self.bit_len,
            });
        }
        let value = self.read_bit_at(self.position);
        self.position += 1;
        Ok(value)
    }
    fn read_bits(&mut self, bits: usize) -> Result<u32, CodecError> {
        if bits == 0 {
            return Ok(0);
        }
        if bits <= 16 && self.bit_len - self.position >= bits {
            let value = self.peek_short_bits(bits);
            self.position += bits;
            return Ok(value);
        }
        // Preserve consumption and the first-missing-bit error on truncated
        // packets, rather than reporting the end of the requested field.
        let mut value = 0u32;
        for _ in 0..bits {
            value = (value << 1) | u32::from(self.read_bit()?);
        }
        Ok(value)
    }
    #[inline]
    fn peek_short_bits(self, bits: usize) -> u32 {
        let byte = self.position / 8;
        let offset = self.position % 8;
        let mut value = u32::from(self.bytes[byte]) << 16;
        if offset + bits > 8 {
            value |= u32::from(self.bytes[byte + 1]) << 8;
        }
        if offset + bits > 16 {
            value |= u32::from(self.bytes[byte + 2]);
        }
        (value >> (24 - offset - bits)) & ((1 << bits) - 1)
    }
    fn read_bit_at(self, position: usize) -> bool {
        ((self.bytes[position / 8] >> (7 - position % 8)) & 1) != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cached_decode_layout_matches_every_normative_coefficient_index() {
        let band_list = bands(FRAME_WIDTH, FRAME_HEIGHT);
        let mut band_groups = [0; 12];
        let mut seen = vec![false; FRAME_PIXELS];
        for (layout, &band_id) in decode_group_layout().iter().zip(raw_group_schedule()) {
            let band = usize::from(band_id);
            let indices = group_indices(&band_list[band], band_groups[band], FRAME_WIDTH).unwrap();
            band_groups[band] += 1;
            assert_eq!(layout.band, band_id);
            for (lane, expected) in indices.into_iter().enumerate() {
                let actual = layout.first as usize
                    + lane
                    + if lane >= usize::from(layout.first_row_lanes) {
                        usize::from(layout.row_skip)
                    } else {
                        0
                    };
                assert_eq!(actual, expected);
                assert!(!seen[actual], "coefficient visited twice");
                seen[actual] = true;
            }
        }
        assert!(seen.into_iter().all(|visited| visited));
    }

    #[test]
    fn fast_bit_fields_preserve_scalar_values_errors_and_consumption() {
        let bytes = [
            0xb5, 0x31, 0xe7, 0x04, 0xff, 0x00, 0x95, 0xab, 0x18, 0x72, 0xc3, 0x5e,
        ];
        for bit_len in 0..=bytes.len() * 8 {
            for start in 0..=bit_len {
                for bits in 0..=32 {
                    let mut actual = BitReader::new(&bytes, bit_len);
                    actual.position = start;
                    let mut expected = actual;
                    let scalar = (|| {
                        let mut value = 0;
                        for _ in 0..bits {
                            value = (value << 1) | u32::from(expected.read_bit()?);
                        }
                        Ok::<_, CodecError>(value)
                    })();
                    assert_eq!(
                        actual.read_bits(bits),
                        scalar,
                        "end={bit_len} start={start} bits={bits}"
                    );
                    assert_eq!(actual.position(), expected.position());
                }
            }
        }
    }

    #[test]
    fn huffman_lookup_preserves_all_prefixes_and_truncation_boundaries() {
        for prefix in 0..128u16 {
            for offset in 0..8 {
                let bytes = (prefix << (9 - offset)).to_be_bytes();
                for available in 0..=7 {
                    let mut actual = BitReader::new(&bytes, offset + available);
                    actual.position = offset;
                    let mut expected = actual;
                    let scalar = (|| {
                        let mut code = 0;
                        for length in 1..=7 {
                            code = (code << 1) | u8::from(expected.read_bit()?);
                            if let Some(symbol) = NIBBLE_CODES
                                .iter()
                                .position(|&(value, bits)| bits == length && value == code)
                            {
                                return Ok(symbol as u8);
                            }
                        }
                        Err(CodecError::InvalidPacket {
                            group: 42,
                            reason: "invalid mask Huffman code",
                        })
                    })();
                    assert_eq!(read_huffman_nibble(&mut actual, 42), scalar);
                    assert_eq!(actual.position(), expected.position());
                }
            }
        }
    }

    #[test]
    fn schedule_and_budget_profile_are_exact() {
        let schedule = raw_group_schedule();
        let mut counts = [0usize; 12];
        for &band in schedule {
            counts[usize::from(band)] += 1;
        }
        let schedule_fnv = schedule
            .iter()
            .fold(0xcbf2_9ce4_8422_2325u64, |hash, &band| {
                (hash ^ u64::from(band)).wrapping_mul(0x0000_0100_0000_01b3)
            });
        assert_eq!(schedule_fnv, 0xf073_d448_1f89_3953);
        assert_eq!(
            counts,
            [
                250, 250, 500, 1000, 1000, 1000, 4000, 4000, 4000, 16000, 16000, 16000
            ]
        );
        assert_eq!(
            counts
                .iter()
                .zip(NOMINAL_BITS)
                .map(|(&count, bits)| count * bits)
                .sum::<usize>(),
            1_039_500
        );
        assert_eq!(RATE_DEBT_SHIFT, 14);
        assert_eq!(RATE_COARSE_CUTOFF_BASE, 5);
        assert_eq!(RATE_INCREMENTAL_CUTOFF_BASE, 6);
        assert_eq!((-1_i64) >> RATE_DEBT_SHIFT, -1);
    }

    #[test]
    fn controller_transition_profile_is_exact() {
        assert_eq!(COARSE_ENERGY_SHIFT, [0, 2, 2, 0, 0, 0, 1, 1, 1, 2, 1, 0]);
        assert_eq!(
            INCREMENTAL_ENERGY_SHIFT,
            [3, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0]
        );
        assert_eq!(
            NOMINAL_BITS,
            [139, 135, 102, 58, 59, 43, 27, 30, 21, 10, 13, 5]
        );
        assert_eq!(rate_cutoffs(-6_i64 << RATE_DEBT_SHIFT), (0, 0));
        assert_eq!(rate_cutoffs(-5_i64 << RATE_DEBT_SHIFT), (0, 1));
        assert_eq!(rate_cutoffs(0), (5, 6));
        assert_eq!(rate_cutoffs(2_i64 << RATE_DEBT_SHIFT), (7, 8));
        assert_eq!(rate_cutoffs(3_i64 << RATE_DEBT_SHIFT), (8, 8));
    }

    #[test]
    fn packet_codes_are_prefix_free_and_raw_is_bounded() {
        assert_eq!(
            NIBBLE_CODES,
            [
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
            ]
        );
        for (left, &(left_code, left_len)) in NIBBLE_CODES.iter().enumerate() {
            for (right, &(right_code, right_len)) in NIBBLE_CODES.iter().enumerate() {
                if left == right {
                    continue;
                }
                let (short_code, short_len, long_code, long_len) = if left_len <= right_len {
                    (left_code, left_len, right_code, right_len)
                } else {
                    (right_code, right_len, left_code, left_len)
                };
                assert_ne!(long_code >> (long_len - short_len), short_code);
            }
        }
        assert_eq!(1 + 1 + 1 + 4 + 16 * 8, 135);
        for low in 1u16..=255 {
            assert!(mask_huffman_bits(low) <= 16);
        }
    }

    #[test]
    fn huffman_nibbles_round_trip_and_headers_are_bounded() {
        assert_eq!(
            NIBBLE_CODES.iter().map(|&(_, length)| length).max(),
            Some(7)
        );
        for (symbol, &(code, length)) in NIBBLE_CODES.iter().enumerate() {
            let mut writer = BitWriter::with_capacity(usize::from(length));
            writer.write_bits(u32::from(code), usize::from(length));
            let bit_len = writer.position();
            let bytes = writer.finish();
            let mut reader = BitReader::new(&bytes, bit_len);
            assert_eq!(read_huffman_nibble(&mut reader, 0), Ok(symbol as u8));
            assert_eq!(reader.position(), usize::from(length));
        }

        let mut max_huffman_header = 0usize;
        for mask in 1u16..=u16::MAX {
            let huffman_bits = mask_huffman_bits(mask);
            if huffman_bits <= 16 {
                max_huffman_header = max_huffman_header.max(3 + huffman_bits);
            }
        }
        assert_eq!(max_huffman_header, 19);
        assert_eq!(3 + 16, 19, "literal-mask header must keep the same bound");
    }

    #[test]
    fn zero_run_boundaries_are_self_delimiting_and_monotone() {
        for (length, expected_bits, expected_appends, expected_packets) in [
            (1, 1, 1, 1),
            (14, 14, 1, 14),
            (15, 15, 1, 1),
            (16, 15, 1, 1),
            (255, 15, 1, 1),
            (256, 15, 1, 1),
            (257, 16, 2, 2),
            (1070, 75, 5, 5),
        ] {
            let mut writer = BitWriter::with_capacity(length);
            let mut remaining = length;
            let mut written_tokens = 0;
            while remaining != 0 {
                let chunk = remaining.min(ZERO_RUN_MAX);
                write_zero_run(&mut writer, chunk);
                remaining -= chunk;
                written_tokens += 1;
            }
            assert_eq!(writer.position(), expected_bits);
            assert!(writer.position() <= length);
            assert_eq!(written_tokens, expected_appends);
            let bit_len = writer.position();
            let bytes = writer.finish();
            let mut reader = BitReader::new(&bytes, bit_len);
            let mut decoded = 0usize;
            let mut packets = 0usize;
            while decoded < length {
                match read_packet(&mut reader, decoded, 0).unwrap() {
                    DecodedPacket::Zero => decoded += 1,
                    DecodedPacket::ZeroRun(run) => decoded += run,
                    DecodedPacket::Values { .. } => panic!("run decoded as coefficients"),
                }
                packets += 1;
            }
            assert_eq!(decoded, length);
            assert_eq!(packets, expected_packets);
            assert_eq!(reader.position(), expected_bits);
        }
    }

    #[test]
    fn pair_token_limit_forces_raw_only_above_seventeen_bits() {
        let mut over_limit = [0i16; GROUP_LANES];
        over_limit[0] = 8;
        over_limit[1] = 8;
        let over_pair_bits = over_limit[..2]
            .iter()
            .map(|&value| coefficient_rice_bits(value, 0))
            .sum::<usize>();
        assert_eq!(over_pair_bits, MAX_PAIR_TOKEN_BITS + 1);
        let over_entropy = entropy_packet_bits(&over_limit, 8, true);
        assert_eq!(over_entropy, EXTERNAL_RAW_BITS_WITHOUT_QUALITY);
        assert_eq!(external_packet_bits(&over_limit, over_entropy, true), 135);
        assert_eq!(external_packet_bits(&over_limit, over_entropy, false), 147);
        assert_eq!(packet_service_cycles(135), NONZERO_PACKET_SERVICE_CYCLES);

        let mut at_limit = [0i16; GROUP_LANES];
        at_limit[0] = 7;
        at_limit[1] = 8;
        let boundary_pair_bits = at_limit[..2]
            .iter()
            .map(|&value| coefficient_rice_bits(value, 0))
            .sum::<usize>();
        assert_eq!(boundary_pair_bits, MAX_PAIR_TOKEN_BITS);
        let boundary_entropy = entropy_packet_bits(&at_limit, 8, true);
        assert!(boundary_entropy < EXTERNAL_RAW_BITS_WITHOUT_QUALITY);
        let boundary_packet_bits = external_packet_bits(&at_limit, boundary_entropy, true);
        assert!(boundary_packet_bits < 135);
        assert_eq!(
            packet_service_cycles(boundary_packet_bits),
            NONZERO_PACKET_SERVICE_CYCLES
        );
        assert_eq!(packet_service_cycles(1), 1);

        let mut raw_writer = BitWriter::with_capacity(135);
        write_packet(&mut raw_writer, &over_limit, true, 8);
        assert_eq!(raw_writer.position(), 135);
        let raw_bytes = raw_writer.finish();
        let mut raw_reader = BitReader::new(&raw_bytes, 135);
        assert_eq!(
            read_packet(&mut raw_reader, 0, 8).unwrap(),
            DecodedPacket::Values {
                fine: true,
                values: over_limit,
            }
        );
        assert_eq!(raw_reader.position(), 135);

        let mut coarse_raw_writer = BitWriter::with_capacity(147);
        write_packet(&mut coarse_raw_writer, &over_limit, false, 8);
        assert_eq!(coarse_raw_writer.position(), 147);
        let coarse_raw_bytes = coarse_raw_writer.finish();
        let mut coarse_raw_reader = BitReader::new(&coarse_raw_bytes, 147);
        assert_eq!(
            read_packet(&mut coarse_raw_reader, 0, 8).unwrap(),
            DecodedPacket::Values {
                fine: false,
                values: over_limit,
            }
        );
        assert_eq!(coarse_raw_reader.position(), 147);

        let mut boundary_writer = BitWriter::with_capacity(boundary_packet_bits);
        write_packet(&mut boundary_writer, &at_limit, true, 8);
        assert_eq!(boundary_writer.position(), boundary_packet_bits);
        let boundary_bytes = boundary_writer.finish();
        let mut boundary_reader = BitReader::new(&boundary_bytes, boundary_packet_bits);
        assert_eq!(
            read_packet(&mut boundary_reader, 0, 8).unwrap(),
            DecodedPacket::Values {
                fine: true,
                values: at_limit,
            }
        );
        assert_eq!(boundary_reader.position(), boundary_packet_bits);
    }

    #[test]
    fn rice_context_profiles_reset_and_cross_power_of_two_boundaries() {
        assert_eq!(contextual_rice_k(0, true, 0), 4);
        assert_eq!(contextual_rice_k(0, true, 1), 4);
        assert_eq!(contextual_rice_k(0, true, 7), 4);

        assert_eq!(contextual_rice_k(0, false, 0), 1);
        assert_eq!(contextual_rice_k(0, false, 1), 3);
        assert_eq!(contextual_rice_k(0, false, 2), 2);

        assert_eq!(contextual_rice_k(1, false, 0), 2);
        assert_eq!(contextual_rice_k(1, false, 1), 1);
        assert_eq!(contextual_rice_k(1, false, 2), 2);

        assert_eq!(contextual_rice_k(2, false, 0), 2);
        assert_eq!(contextual_rice_k(2, false, 1), 2);
        assert_eq!(contextual_rice_k(2, false, 7), 2);

        assert_eq!(contextual_rice_k(3, false, 0), 1);
        assert_eq!(contextual_rice_k(3, false, 4), 1);
        assert_eq!(contextual_rice_k(3, false, 5), 2);

        assert_eq!(contextual_rice_k(7, false, 0), 0);
        assert_eq!(contextual_rice_k(7, false, 2), 0);
        assert_eq!(contextual_rice_k(7, false, 3), 1);

        assert_eq!(contextual_rice_k(9, false, 0), 0);
        assert_eq!(contextual_rice_k(9, false, 1), 0);
        assert_eq!(contextual_rice_k(9, false, 7), 0);
    }

    #[test]
    fn contextual_packet_length_matches_two_reset_round_trips() {
        let mut values = [0i16; GROUP_LANES];
        values[0] = 3;
        values[GROUP_LANES - 1] = 3;
        let entropy = entropy_packet_bits(&values, 3, false);
        let packet_bits = external_packet_bits(&values, entropy, false);
        assert_eq!(packet_bits, 21);

        let mut writer = BitWriter::with_capacity(packet_bits * 2);
        write_packet(&mut writer, &values, false, 3);
        write_packet(&mut writer, &values, false, 3);
        assert_eq!(writer.position(), packet_bits * 2);
        let bytes = writer.finish();
        let bit_at = |position: usize| (bytes[position / 8] >> (7 - position % 8)) & 1;
        for bit in 0..packet_bits {
            assert_eq!(bit_at(bit), bit_at(packet_bits + bit));
        }

        let mut reader = BitReader::new(&bytes, packet_bits * 2);
        assert_eq!(
            read_packet(&mut reader, 0, 3).unwrap(),
            DecodedPacket::Values {
                fine: false,
                values,
            }
        );
        assert_eq!(
            read_packet(&mut reader, 1, 3).unwrap(),
            DecodedPacket::Values {
                fine: false,
                values,
            }
        );
        assert_eq!(reader.position(), packet_bits * 2);
    }

    #[test]
    fn causal_allocator_reserves_every_future_zero_tag() {
        let candidate = GroupCandidate {
            band: 0,
            fine: [0; GROUP_LANES],
            coarse: [0; GROUP_LANES],
            fine_bits: 5,
            coarse_bits: 3,
            fine_incremental_gain: 1 << 20,
            coarse_gain: 1 << 20,
        };
        let mut second = candidate.clone();
        second.fine_bits = 4;
        let (states, used, stats) = allocate(&[candidate, second], 5, Some(&[256, 11])).unwrap();
        assert_eq!(states, [1, 0]);
        assert_eq!(used, 4);
        assert_eq!(stats.coarse_groups, 1);
        assert_eq!(stats.zero_groups, 1);
        assert_eq!(stats.padding_bits, 1);
    }

    fn high_gain_candidate(fine_bits: usize, coarse_bits: usize) -> GroupCandidate {
        GroupCandidate {
            band: 0,
            fine: [1; GROUP_LANES],
            coarse: [1; GROUP_LANES],
            fine_bits,
            coarse_bits,
            fine_incremental_gain: 1 << 20,
            coarse_gain: 1 << 20,
        }
    }

    fn dense_group_cycle_deltas(count: usize) -> Vec<u32> {
        let mut deltas = vec![11; count];
        // Fill the empty bucket before the first candidate, then use the
        // RTL's minimum group-decision spacing.
        deltas[0] = (BURST_CAPACITY_BITS / BURST_REFILL_BITS_PER_CLOCK) as u32;
        deltas
    }

    fn drain_to_available(available_at_next_group: usize) -> Vec<GroupCandidate> {
        let mut groups = Vec::new();
        let mut available = BURST_CAPACITY_BITS;
        let refill = 11 * BURST_REFILL_BITS_PER_CLOCK;
        loop {
            let cost = available + refill - available_at_next_group;
            if cost <= FINE_RAW_PACKET_BITS {
                groups.push(high_gain_candidate(cost, cost));
                return groups;
            }
            groups.push(high_gain_candidate(
                FINE_RAW_PACKET_BITS,
                FINE_RAW_PACKET_BITS,
            ));
            available += refill;
            available -= FINE_RAW_PACKET_BITS;
        }
    }

    #[test]
    fn burst_selector_admits_shorter_fine_and_falls_back_to_coarse_or_zero() {
        let mut groups = drain_to_available(143);
        groups.push(high_gain_candidate(135, 147));
        let (states, _, _) = allocate(
            &groups,
            100_000,
            Some(&dense_group_cycle_deltas(groups.len())),
        )
        .unwrap();
        // The last group has 143 credits: its fine literal fits, although
        // its longer coarse literal does not.
        assert!(states.iter().all(|&state| state == 2));

        let mut groups = drain_to_available(125);
        groups.push(high_gain_candidate(135, 110));
        let (states, _, _) = allocate(
            &groups,
            100_000,
            Some(&dense_group_cycle_deltas(groups.len())),
        )
        .unwrap();
        // With 125 credits, the preferred fine packet is unaffordable, but
        // the coarse packet fits.
        assert_eq!(states.last(), Some(&1));

        *groups.last_mut().unwrap() = high_gain_candidate(135, 147);
        let (states, _, _) = allocate(
            &groups,
            100_000,
            Some(&dense_group_cycle_deltas(groups.len())),
        )
        .unwrap();
        assert_eq!(states.last(), Some(&0));
        let (frame_budget_only, _, _) = allocate(&groups, 100_000, None).unwrap();
        assert!(frame_budget_only.iter().all(|&state| state == 2));
    }

    #[test]
    fn zero_run_refunds_do_not_replenish_burst_credits() {
        let mut groups = drain_to_available(24);
        let prefix_length = groups.len();
        let mut zero = high_gain_candidate(1, 1);
        zero.coarse_gain = 0;
        zero.fine_incremental_gain = 0;
        groups.extend(vec![zero; 16]);
        groups.extend(vec![high_gain_candidate(135, 147); 3]);

        let (states, _, stats) = allocate(
            &groups,
            100_000,
            Some(&dense_group_cycle_deltas(groups.len())),
        )
        .unwrap();
        assert!(states[..prefix_length].iter().all(|&state| state == 2));
        assert_eq!(&states[prefix_length..prefix_length + 16], &[0; 16]);
        assert_eq!(&states[prefix_length + 16..], &[2, 2, 0]);
        assert_eq!(stats.zero_run_savings, 1);
        // Exactly 134 credits remain available to the last candidate. The
        // one-bit frame-budget refund from the 16-zero run cannot buy it.
    }

    #[test]
    fn burst_bucket_caps_credit_and_bounds_every_group_interval() {
        let mut random = 0x243f_6a88_85a3_08d3_u64;
        // Sustain enough randomized traffic to exhaust the 5.5 KiB allowance,
        // while keeping the independent frame budget comfortably out of play.
        let groups = (0..2048)
            .map(|index| {
                random ^= random << 13;
                random ^= random >> 7;
                random ^= random << 17;
                if (96..192).contains(&index) {
                    let mut zero = high_gain_candidate(1, 1);
                    zero.coarse_gain = 0;
                    zero.fine_incremental_gain = 0;
                    zero
                } else {
                    high_gain_candidate(
                        2 + random as usize % (FINE_RAW_PACKET_BITS - 1),
                        2 + (random >> 32) as usize % (COARSE_RAW_PACKET_BITS - 1),
                    )
                }
            })
            .collect::<Vec<_>>();
        let mut deltas = dense_group_cycle_deltas(groups.len());
        for index in (173..groups.len()).step_by(173) {
            deltas[index] = 400;
        }
        let decision_clocks = deltas
            .iter()
            .scan(0usize, |clocks, &delta| {
                *clocks += delta as usize;
                Some(*clocks)
            })
            .collect::<Vec<_>>();
        let (states, _, _) = allocate(&groups, 1_000_000, Some(&deltas)).unwrap();
        assert!(
            groups
                .iter()
                .zip(&states)
                .any(|(group, &state)| group.coarse_gain != 0 && state == 0)
        );
        let mut prefix_bits = vec![0];
        for (group, state) in groups.iter().zip(states) {
            let selected_bits = match state {
                2 => group.fine_bits,
                1 => group.coarse_bits,
                _ => 1,
            };
            prefix_bits.push(prefix_bits.last().unwrap() + selected_bits);
        }
        for start in 0..groups.len() {
            for end in start + 1..=groups.len() {
                assert!(
                    prefix_bits[end] - prefix_bits[start]
                        <= BURST_CAPACITY_BITS
                            + (decision_clocks[end - 1] - decision_clocks[start])
                                * BURST_REFILL_BITS_PER_CLOCK,
                    "burst budget exceeded for group interval {start}..{end}"
                );
            }
        }

        // A long idle/zero interval cannot accumulate an arbitrarily large
        // allowance. It replenishes only the original full-bucket burst.
        let mut zero = high_gain_candidate(1, 1);
        zero.coarse_gain = 0;
        zero.fine_incremental_gain = 0;
        let idle_groups = BURST_CAPACITY_BITS;
        let heavy = vec![high_gain_candidate(135, 147); BURST_CAPACITY_BITS / 100 + 8];
        let mut after_idle = vec![zero; idle_groups];
        after_idle.extend(heavy.clone());
        let (states, _, _) = allocate(
            &after_idle,
            10_000_000,
            Some(&dense_group_cycle_deltas(after_idle.len())),
        )
        .unwrap();
        let (from_full, _, _) = allocate(
            &heavy,
            100_000,
            Some(&dense_group_cycle_deltas(heavy.len())),
        )
        .unwrap();
        assert!(from_full.contains(&0));
        assert_eq!(&states[idle_groups..], &from_full);
    }

    #[test]
    fn burst_credit_accrues_from_elapsed_clocks_and_starts_empty() {
        let groups = vec![high_gain_candidate(135, 147); BURST_CAPACITY_BITS / 100 + 8];
        let dense = dense_group_cycle_deltas(groups.len());
        let mut sparse = dense.clone();
        sparse[1..].fill(100);
        let (dense_states, _, _) = allocate(&groups, 100_000, Some(&dense)).unwrap();
        let (sparse_states, _, _) = allocate(&groups, 100_000, Some(&sparse)).unwrap();
        assert!(dense_states.contains(&0));
        assert!(sparse_states.iter().all(|&state| state == 2));

        let (states, _, _) = allocate(&groups[..2], 10_000, Some(&[11, 300])).unwrap();
        assert_eq!(states, [0, 2]);
        assert!(matches!(
            allocate(&groups, 10_000, Some(&[11])),
            Err(CodecError::GroupTimingCountMismatch { .. })
        ));
        assert!(matches!(
            allocate(&groups[..2], 10_000, Some(&[256, 0])),
            Err(CodecError::InvalidGroupTiming { group: 1, .. })
        ));
    }

    #[test]
    fn burst_pressure_remains_effective_after_a_low_detail_prefix() {
        let mut zero = high_gain_candidate(1, 1);
        zero.coarse_gain = 0;
        zero.fine_incremental_gain = 0;
        let mut groups = vec![zero; 1024];
        groups.extend(drain_to_available(BURST_CAPACITY_BITS / 2));
        let mut low_gain = high_gain_candidate(100, 80);
        low_gain.coarse_gain = 0;
        low_gain.fine_incremental_gain = 0;
        groups.push(low_gain);
        let (bounded, _, stats) = allocate(
            &groups,
            1_000_000,
            Some(&dense_group_cycle_deltas(groups.len())),
        )
        .unwrap();
        let (frame_budget_only, _, _) = allocate(&groups, 1_000_000, None).unwrap();
        assert_eq!(bounded.last(), Some(&0));
        assert_eq!(frame_budget_only.last(), Some(&2));
        assert!(stats.min_rate_debt < -(8 << RATE_DEBT_SHIFT));
        assert_eq!(burst_cutoff(BURST_CAPACITY_BITS), 0);
        assert_eq!(burst_cutoff(BURST_CAPACITY_BITS - 8191), 0);
        assert_eq!(burst_cutoff(BURST_CAPACITY_BITS - 8192), 1);
        assert_eq!(burst_cutoff(BURST_CAPACITY_BITS / 2), 2);
        assert_eq!(burst_cutoff(0), 5);
    }

    #[test]
    fn canonical_group_timing_matches_the_measured_raster_profile() {
        let deltas = canonical_group_cycle_deltas();
        assert_eq!(deltas.len(), GROUP_COUNT);
        assert_eq!(deltas[0], 1825);
        assert_eq!(deltas.iter().sum::<u32>(), 719_880);
        assert_eq!(deltas[1..].iter().min(), Some(&11));
        assert_eq!(deltas[1..].iter().max(), Some(&483));
    }

    #[test]
    fn default_encoder_uses_the_production_burst_policy() {
        let pixels = (0..FRAME_PIXELS)
            .map(|index| ((index * 73) ^ (index / FRAME_WIDTH * 151)) as u8)
            .collect::<Vec<_>>();
        let default = Codec::encode_with_stats(&pixels, FRAME_WIDTH, FRAME_HEIGHT).unwrap();
        let bounded =
            Codec::encode_with_stats_and_burst_mode(&pixels, FRAME_WIDTH, FRAME_HEIGHT, true)
                .unwrap();
        let frame_budget_only =
            Codec::encode_with_burst_mode(&pixels, FRAME_WIDTH, FRAME_HEIGHT, false).unwrap();
        assert_eq!(default, bounded);
        assert_eq!(
            default,
            Codec::encode_with_stats_and_group_cycle_deltas(
                &pixels,
                FRAME_WIDTH,
                FRAME_HEIGHT,
                canonical_group_cycle_deltas(),
            )
            .unwrap()
        );
        assert_ne!(default.0, frame_budget_only);
        assert_eq!(default.0.payload_bits(), FRAME_PIXELS);
        assert_eq!(default.0.bytes().len(), FRAME_PIXELS / 8);
        assert_eq!(Codec::decode(&default.0).unwrap().len(), FRAME_PIXELS);
        assert_eq!(
            Codec::decode(&frame_budget_only).unwrap().len(),
            FRAME_PIXELS
        );
        assert_eq!(
            Codec::allocation_stats(&pixels, FRAME_WIDTH, FRAME_HEIGHT).unwrap(),
            default.1
        );
    }

    #[test]
    fn adversarial_frames_are_bounded_and_exact_size() {
        let mut frames = vec![vec![0u8; FRAME_PIXELS], vec![255u8; FRAME_PIXELS]];
        frames.push(
            (0..FRAME_PIXELS)
                .map(|index| {
                    if ((index % FRAME_WIDTH) + index / FRAME_WIDTH) & 1 == 0 {
                        0
                    } else {
                        255
                    }
                })
                .collect(),
        );
        let mut state = 0x243f_6a88_85a3_08d3u64;
        frames.push(
            (0..FRAME_PIXELS)
                .map(|_| {
                    state ^= state << 13;
                    state ^= state >> 7;
                    state ^= state << 17;
                    state as u8
                })
                .collect(),
        );
        for pixels in frames {
            let (encoded, stats) =
                Codec::encode_with_stats(&pixels, FRAME_WIDTH, FRAME_HEIGHT).unwrap();
            assert_eq!(encoded.payload_bits(), FRAME_PIXELS);
            assert_eq!(encoded.bytes().len(), FRAME_PIXELS / 8);
            assert_eq!(
                stats.fine_groups + stats.coarse_groups + stats.zero_groups,
                GROUP_COUNT
            );
            assert_eq!(stats.used_output_bits + stats.padding_bits, FRAME_PIXELS);
            assert!(stats.serializer_cycles <= GROUP_COUNT * NONZERO_PACKET_SERVICE_CYCLES);
            assert!(stats.max_packet_cycles <= NONZERO_PACKET_SERVICE_CYCLES);
            assert_eq!(Codec::decode(&encoded).unwrap().len(), FRAME_PIXELS);
        }
    }

    #[test]
    fn malformed_extent_and_padding_are_rejected() {
        assert!(matches!(
            Codec::encode(&[], 32, 32),
            Err(CodecError::InvalidDimensions { .. })
        ));
        let encoded = Codec::encode(&vec![128; FRAME_PIXELS], FRAME_WIDTH, FRAME_HEIGHT).unwrap();
        let mut short = encoded.clone();
        short.bytes.pop();
        assert!(matches!(
            Codec::decode(&short),
            Err(CodecError::BitstreamUnderflow { .. })
        ));
        let mut bad_padding = encoded;
        *bad_padding.bytes.last_mut().unwrap() |= 1;
        assert!(matches!(
            Codec::decode(&bad_padding),
            Err(CodecError::NonzeroPadding { .. })
        ));

        let mut writer = BitWriter::with_capacity(FRAME_PIXELS);
        writer.write_zeros(GROUP_COUNT - 1);
        writer.write_bits(0b1000000, 7);
        writer.write_bits(2, 8);
        writer.write_zeros(FRAME_PIXELS - writer.position());
        let impossible_run = EncodedFrame {
            width: FRAME_WIDTH,
            height: FRAME_HEIGHT,
            bytes: writer.finish(),
            bit_len: FRAME_PIXELS,
        };
        assert!(matches!(
            Codec::decode(&impossible_run),
            Err(CodecError::InvalidPacket {
                reason: "zero run exceeds remaining group schedule",
                ..
            })
        ));
    }

    #[test]
    fn compact_decode_matches_fixed_for_real_encoded_frames() {
        let mut noise = 0x243f_6a88_85a3_08d3u64;
        let scenes = [
            vec![128; FRAME_PIXELS],
            (0..FRAME_PIXELS)
                .map(|index| {
                    if (index / FRAME_WIDTH + index % FRAME_WIDTH) & 1 == 0 {
                        0
                    } else {
                        255
                    }
                })
                .collect(),
            (0..FRAME_PIXELS)
                .map(|index| ((index % FRAME_WIDTH * 7) ^ (index / FRAME_WIDTH * 13)) as u8)
                .collect(),
            (0..FRAME_PIXELS)
                .map(|_| {
                    noise ^= noise << 13;
                    noise ^= noise >> 7;
                    noise ^= noise << 17;
                    noise as u8
                })
                .collect(),
        ];
        for pixels in scenes {
            let fixed = Codec::encode(&pixels, FRAME_WIDTH, FRAME_HEIGHT).unwrap();
            let semantic_end = *Codec::packet_end_bits(&fixed).unwrap().last().unwrap();
            let compact = &fixed.bytes[..semantic_end.div_ceil(16) * 2];
            assert_eq!(
                Codec::decode_compact(compact, FRAME_WIDTH, FRAME_HEIGHT).unwrap(),
                Codec::decode(&fixed).unwrap(),
            );
            assert!(EncodedFrame::from_bytes(FRAME_WIDTH, FRAME_HEIGHT, compact.to_vec()).is_err());
        }
    }

    #[test]
    fn compact_decode_checks_every_final_word_alignment() {
        for runs in 0..16 {
            let mut writer = BitWriter::with_capacity(FRAME_PIXELS);
            for _ in 0..runs {
                write_zero_run(&mut writer, ZERO_RUN_MAX);
            }
            writer.write_zeros(GROUP_COUNT - runs * ZERO_RUN_MAX);
            let used_bits = writer.position();
            assert_eq!(used_bits % 16, (16 - runs) % 16);
            let compact = &writer.finish()[..used_bits.div_ceil(16) * 2];
            assert_eq!(
                Codec::decode_compact(compact, FRAME_WIDTH, FRAME_HEIGHT).unwrap(),
                vec![128; FRAME_PIXELS],
            );
            if !used_bits.is_multiple_of(16) {
                let mut corrupt = compact.to_vec();
                *corrupt.last_mut().unwrap() = 1;
                assert!(matches!(
                    Codec::decode_compact(&corrupt, FRAME_WIDTH, FRAME_HEIGHT),
                    Err(CodecError::NonzeroPadding { .. }),
                ));
            }
        }
    }

    #[test]
    fn compact_decode_rejects_invalid_extents_and_extra_zero_words() {
        let mut writer = BitWriter::with_capacity(FRAME_PIXELS);
        for _ in 0..GROUP_COUNT / ZERO_RUN_MAX {
            write_zero_run(&mut writer, ZERO_RUN_MAX);
        }
        let fixed = EncodedFrame::from_bytes(FRAME_WIDTH, FRAME_HEIGHT, writer.finish()).unwrap();
        let semantic_end = *Codec::packet_end_bits(&fixed).unwrap().last().unwrap();
        assert_eq!(semantic_end, 3750);
        let compact = &fixed.bytes[..semantic_end.div_ceil(16) * 2];
        assert_eq!(compact.len(), 470);
        assert!(matches!(
            Codec::decode_compact(&[], FRAME_WIDTH, FRAME_HEIGHT),
            Err(CodecError::BitstreamUnderflow { .. }),
        ));
        assert!(matches!(
            Codec::decode_compact(&compact[..compact.len() - 1], FRAME_WIDTH, FRAME_HEIGHT),
            Err(CodecError::InvalidWordAlignment { .. }),
        ));
        assert!(matches!(
            Codec::decode_compact(&vec![0; FRAME_PIXELS / 8 + 2], FRAME_WIDTH, FRAME_HEIGHT),
            Err(CodecError::TrailingBits { .. }),
        ));
        assert!(matches!(
            Codec::decode_compact(compact, 32, 32),
            Err(CodecError::InvalidDimensions { .. }),
        ));
        let mut extra_word = compact.to_vec();
        extra_word.extend_from_slice(&[0, 0]);
        for extended in [extra_word.as_slice(), fixed.bytes.as_slice()] {
            assert!(matches!(
                Codec::decode_compact(extended, FRAME_WIDTH, FRAME_HEIGHT),
                Err(CodecError::TrailingBits {
                    expected_bits: 3760,
                    ..
                }),
            ));
        }
        let mut inconsistent = fixed;
        inconsistent.bit_len -= 16;
        assert!(matches!(
            Codec::decode(&inconsistent),
            Err(CodecError::TrailingBits { .. })
        ));
    }

    #[test]
    fn compact_decode_rejects_truncated_packets_and_zero_run_overshoot() {
        for final_values in [[127; GROUP_LANES], [1; GROUP_LANES]] {
            let mut writer = BitWriter::with_capacity(FRAME_PIXELS);
            writer.write_zeros(GROUP_COUNT - 1);
            write_packet(
                &mut writer,
                &final_values,
                true,
                usize::from(raw_group_schedule()[GROUP_COUNT - 1]),
            );
            let compact_len = writer.position().div_ceil(16) * 2;
            let bytes = writer.finish();
            assert!(matches!(
                Codec::decode_compact(&bytes[..compact_len - 2], FRAME_WIDTH, FRAME_HEIGHT),
                Err(CodecError::BitstreamUnderflow { .. }),
            ));
        }
        let mut writer = BitWriter::with_capacity(FRAME_PIXELS);
        for _ in 0..GROUP_COUNT / ZERO_RUN_MAX {
            write_zero_run(&mut writer, ZERO_RUN_MAX);
        }
        let compact_len = writer.position().div_ceil(16) * 2;
        let bytes = writer.finish();
        assert!(matches!(
            Codec::decode_compact(&bytes[..compact_len - 2], FRAME_WIDTH, FRAME_HEIGHT),
            Err(CodecError::BitstreamUnderflow { .. }),
        ));

        let mut writer = BitWriter::with_capacity(FRAME_PIXELS);
        writer.write_zeros(GROUP_COUNT - ZERO_RUN_MIN + 1);
        write_zero_run(&mut writer, ZERO_RUN_MIN);
        let compact_len = writer.position().div_ceil(16) * 2;
        assert!(matches!(
            Codec::decode_compact(&writer.finish()[..compact_len], FRAME_WIDTH, FRAME_HEIGHT),
            Err(CodecError::InvalidPacket {
                reason: "zero run exceeds remaining group schedule",
                ..
            }),
        ));
    }

    #[test]
    fn compact_decode_accepts_semantic_data_at_full_extent() {
        // A legal grammar fixture reaching exactly the fixed frame budget:
        // 7,162 fine raw + 2 coarse raw + 56,836 single-zero group packets.
        // All raw coefficients are zero, so reconstruction is neutral gray.
        let mut writer = BitWriter::with_capacity(FRAME_PIXELS);
        for _ in 0..7162 {
            writer.write_bits(0b1100000, 7);
            writer.write_zeros(128);
        }
        for _ in 0..2 {
            writer.write_bits(0b101, 3);
            writer.write_zeros(144);
        }
        writer.write_zeros(GROUP_COUNT - 7164);
        assert_eq!(writer.position(), FRAME_PIXELS);
        let bytes = writer.finish();
        assert_eq!(
            Codec::decode_compact(&bytes, FRAME_WIDTH, FRAME_HEIGHT).unwrap(),
            Codec::decode(&EncodedFrame::from_bytes(FRAME_WIDTH, FRAME_HEIGHT, bytes).unwrap())
                .unwrap(),
        );
    }
}
