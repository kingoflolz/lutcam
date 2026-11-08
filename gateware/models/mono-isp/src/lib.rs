//! Bit-exact software reference model for the ten-bit monochrome ISP.
//!
//! Pixels are processed two at a time.  Sparse defect correction operates on
//! the ten-bit raw samples, the exposure histogram observes the corrected
//! pair, and the programmable tone LUT produces the final eight-bit pixels.

use std::error::Error;
use std::fmt;

/// Maximum value accepted on a ten-bit raw-pixel input.
pub const RAW_MAX: u16 = 0x03ff;
/// Number of entries in the raw-to-eight-bit tone LUT.
pub const TONE_LUT_ENTRIES: usize = 1024;
/// Maximum number of words in the sparse defect table.
pub const DEFECT_TABLE_CAPACITY: usize = 512;
/// Largest cumulative-address delta represented by one defect-table word.
pub const DEFECT_DELTA_MAX: u32 = 0x03_ffff;
/// Mask selecting the low two bits of a packed defect-table word.
pub const DEFECT_MASK_BITS: u32 = 0x3;
/// Mask selecting every valid bit of a packed defect-table word.
pub const DEFECT_WORD_MASK: u32 = 0x0f_ffff;
/// Number of exposure-histogram bins.
pub const HISTOGRAM_BINS: usize = 512;
/// Saturation value of a twenty-bit histogram counter.
pub const HISTOGRAM_COUNT_MAX: u32 = 0x0f_ffff;

/// Wishbone-visible registers and bit definitions.
pub mod registers {
    /// Monochrome ISP identification value (`"MISP"`).
    pub const ISP_ID: u32 = 0x4d49_5350;

    pub const REG_ID: u16 = 0x0000;
    pub const REG_CONTROL: u16 = 0x0004;
    pub const REG_STATUS: u16 = 0x0008;
    pub const REG_DEFECT_COUNT: u16 = 0x000c;
    pub const REG_COMMAND: u16 = 0x0010;

    pub const TONE_LUT_BASE: u16 = 0x1000;
    pub const TONE_LUT_LAST: u16 = 0x1ffc;
    pub const DEFECT_TABLE_BASE: u16 = 0x2000;
    pub const DEFECT_TABLE_LAST: u16 = 0x27fc;

    pub const CONTROL_RUN: u32 = 1 << 0;
    pub const CONTROL_LUT_ENABLE: u32 = 1 << 1;
    pub const CONTROL_HISTOGRAM_ENABLE: u32 = 1 << 2;

    pub const STATUS_RUN: u32 = 1 << 0;
    pub const STATUS_PIPELINE_BUSY: u32 = 1 << 1;
    pub const STATUS_DEFECT_READY: u32 = 1 << 2;
    pub const STATUS_HISTOGRAM_CLEAR_BUSY: u32 = 1 << 3;
    pub const STATUS_HISTOGRAM_DUMP_BUSY: u32 = 1 << 4;
    pub const STATUS_HISTOGRAM_INITIALIZED: u32 = 1 << 5;
    pub const STATUS_PROTOCOL_ERROR: u32 = 1 << 6;

    pub const COMMAND_HISTOGRAM_CLEAR: u32 = 1 << 0;
    pub const COMMAND_PROTOCOL_ERROR_CLEAR: u32 = 1 << 1;

    pub const HISTOGRAM_RECORD_FIRST: u32 = 1 << 31;
    pub const HISTOGRAM_RECORD_LAST: u32 = 1 << 30;
    pub const HISTOGRAM_RECORD_BIN_SHIFT: u32 = 20;
    pub const HISTOGRAM_RECORD_BIN_MASK: u32 = 0x1ff << HISTOGRAM_RECORD_BIN_SHIFT;
    pub const HISTOGRAM_RECORD_COUNT_MASK: u32 = 0x0f_ffff;

    /// Byte address of one tone-LUT entry.
    #[must_use]
    pub const fn tone_lut_address(index: usize) -> Option<u16> {
        if index < 1024 {
            Some(TONE_LUT_BASE + (index as u16) * 4)
        } else {
            None
        }
    }

    /// Byte address of one sparse-defect-table word.
    #[must_use]
    pub const fn defect_table_address(index: usize) -> Option<u16> {
        if index < 512 {
            Some(DEFECT_TABLE_BASE + (index as u16) * 4)
        } else {
            None
        }
    }
}

pub use registers::*;

/// Which pixels in a two-pixel input word are defective.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum DefectMask {
    Lane0 = 1,
    Lane1 = 2,
    Both = 3,
}

impl DefectMask {
    #[must_use]
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

impl TryFrom<u8> for DefectMask {
    type Error = MonoIspError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::Lane0),
            2 => Ok(Self::Lane1),
            3 => Ok(Self::Both),
            _ => Err(MonoIspError::InvalidDefectMask(value)),
        }
    }
}

/// One semantic sparse-defect entry.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DefectEntry {
    /// Linear address of a two-pixel word within the frame.
    pub pair_address: u32,
    pub mask: DefectMask,
}

impl DefectEntry {
    #[must_use]
    pub const fn new(pair_address: u32, mask: DefectMask) -> Self {
        Self { pair_address, mask }
    }

    /// Construct an entry from the two mask bits used by software interfaces.
    pub fn try_new(pair_address: u32, mask: u8) -> Result<Self, MonoIspError> {
        Ok(Self::new(pair_address, DefectMask::try_from(mask)?))
    }
}

/// Wishbone-ready cumulative sparse-defect table.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct PackedDefectTable {
    /// Twenty-bit words; only bits 19:0 are used.
    pub words: Vec<u32>,
}

impl PackedDefectTable {
    #[must_use]
    pub fn len(&self) -> usize {
        self.words.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.words.is_empty()
    }
}

/// Errors detected by reference-model configuration or frame processing.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum MonoIspError {
    InvalidDimensions {
        width: usize,
        height: usize,
    },
    FrameTooLarge {
        width: usize,
        height: usize,
    },
    FrameLength {
        expected: usize,
        actual: usize,
    },
    PairFrameLength {
        expected: usize,
        actual: usize,
    },
    RawPixelOutOfRange {
        index: usize,
        value: u16,
    },
    BlackLevelOutOfRange(u16),
    InvalidDefectMask(u8),
    DefectEntriesNotStrictlySorted {
        index: usize,
        previous: u32,
        address: u32,
    },
    DefectAddressOutOfRange {
        index: usize,
        address: u32,
        frame_pairs: u32,
    },
    DefectTableCapacity {
        required: usize,
        capacity: usize,
    },
    InvalidDefectWord {
        index: usize,
        word: u32,
    },
    DefectAddressOverflow {
        index: usize,
    },
    HistogramNotInitialized,
}

impl fmt::Display for MonoIspError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDimensions { width, height } => {
                write!(
                    f,
                    "dimensions must be nonzero with an even width, got {width}x{height}"
                )
            }
            Self::FrameTooLarge { width, height } => {
                write!(
                    f,
                    "frame {width}x{height} has more pair addresses than u32 can represent"
                )
            }
            Self::FrameLength { expected, actual } => {
                write!(f, "expected {expected} pixels, got {actual}")
            }
            Self::PairFrameLength { expected, actual } => {
                write!(f, "expected {expected} pixel pairs, got {actual}")
            }
            Self::RawPixelOutOfRange { index, value } => {
                write!(f, "raw pixel {index} is {value}, exceeding ten bits")
            }
            Self::BlackLevelOutOfRange(value) => {
                write!(f, "black level {value} exceeds ten bits")
            }
            Self::InvalidDefectMask(mask) => write!(f, "invalid semantic defect mask {mask}"),
            Self::DefectEntriesNotStrictlySorted {
                index,
                previous,
                address,
            } => write!(
                f,
                "defect entry {index} has address {address}, not greater than {previous}"
            ),
            Self::DefectAddressOutOfRange {
                index,
                address,
                frame_pairs,
            } => write!(
                f,
                "defect word or entry {index} resolves to pair {address}, outside {frame_pairs} pairs"
            ),
            Self::DefectTableCapacity { required, capacity } => write!(
                f,
                "defect table needs {required} words, exceeding its {capacity}-word capacity"
            ),
            Self::InvalidDefectWord { index, word } => {
                write!(
                    f,
                    "defect table word {index} has bits above bit 19 set: {word:#x}"
                )
            }
            Self::DefectAddressOverflow { index } => {
                write!(f, "cumulative defect address overflows at word {index}")
            }
            Self::HistogramNotInitialized => {
                write!(f, "histogram collection requires an explicit clear first")
            }
        }
    }
}

impl Error for MonoIspError {}

fn frame_pair_count(width: usize, height: usize) -> Result<u32, MonoIspError> {
    if width == 0 || height == 0 || width & 1 != 0 {
        return Err(MonoIspError::InvalidDimensions { width, height });
    }
    let pairs = (width / 2)
        .checked_mul(height)
        .ok_or(MonoIspError::FrameTooLarge { width, height })?;
    u32::try_from(pairs).map_err(|_| MonoIspError::FrameTooLarge { width, height })
}

/// Convert sorted semantic entries into cumulative twenty-bit table words.
///
/// The cumulative address starts at zero.  A word with mask zero advances the
/// address without describing a defect, so the packer emits continuation words
/// when a gap is larger than [`DEFECT_DELTA_MAX`].
pub fn pack_defect_table(
    entries: &[DefectEntry],
    width: usize,
    height: usize,
) -> Result<PackedDefectTable, MonoIspError> {
    let frame_pairs = frame_pair_count(width, height)?;
    let mut words = Vec::with_capacity(entries.len());
    let mut accumulated_address = 0u32;
    let mut previous_entry = None;

    for (index, entry) in entries.iter().copied().enumerate() {
        if entry.pair_address >= frame_pairs {
            return Err(MonoIspError::DefectAddressOutOfRange {
                index,
                address: entry.pair_address,
                frame_pairs,
            });
        }
        if let Some(previous) = previous_entry
            && entry.pair_address <= previous
        {
            return Err(MonoIspError::DefectEntriesNotStrictlySorted {
                index,
                previous,
                address: entry.pair_address,
            });
        }

        let mut delta = entry.pair_address - accumulated_address;
        while delta > DEFECT_DELTA_MAX {
            words.push(DEFECT_DELTA_MAX << 2);
            accumulated_address += DEFECT_DELTA_MAX;
            delta -= DEFECT_DELTA_MAX;
        }
        words.push((delta << 2) | u32::from(entry.mask.bits()));
        accumulated_address = entry.pair_address;
        previous_entry = Some(entry.pair_address);

        if words.len() > DEFECT_TABLE_CAPACITY {
            return Err(MonoIspError::DefectTableCapacity {
                required: words.len(),
                capacity: DEFECT_TABLE_CAPACITY,
            });
        }
    }

    Ok(PackedDefectTable { words })
}

/// Resolve cumulative table words into semantic defect entries.
///
/// Mask-zero words are address continuations and are not returned.  The first
/// defect may legally resolve to address zero; later defects must increase.
pub fn unpack_defect_table(
    words: &[u32],
    width: usize,
    height: usize,
) -> Result<Vec<DefectEntry>, MonoIspError> {
    let frame_pairs = frame_pair_count(width, height)?;
    if words.len() > DEFECT_TABLE_CAPACITY {
        return Err(MonoIspError::DefectTableCapacity {
            required: words.len(),
            capacity: DEFECT_TABLE_CAPACITY,
        });
    }

    let mut entries = Vec::new();
    let mut accumulated_address = 0u32;
    let mut previous_entry = None;

    for (index, &word) in words.iter().enumerate() {
        if word & !DEFECT_WORD_MASK != 0 {
            return Err(MonoIspError::InvalidDefectWord { index, word });
        }
        let delta = word >> 2;
        accumulated_address = accumulated_address
            .checked_add(delta)
            .ok_or(MonoIspError::DefectAddressOverflow { index })?;
        if accumulated_address >= frame_pairs {
            return Err(MonoIspError::DefectAddressOutOfRange {
                index,
                address: accumulated_address,
                frame_pairs,
            });
        }

        let mask_bits = (word & DEFECT_MASK_BITS) as u8;
        if mask_bits == 0 {
            continue;
        }
        if let Some(previous) = previous_entry
            && accumulated_address <= previous
        {
            return Err(MonoIspError::DefectEntriesNotStrictlySorted {
                index,
                previous,
                address: accumulated_address,
            });
        }
        entries.push(DefectEntry::new(
            accumulated_address,
            DefectMask::try_from(mask_bits)?,
        ));
        previous_entry = Some(accumulated_address);
    }

    Ok(entries)
}

/// A linear ten-bit-to-eight-bit gamma curve.
#[must_use]
pub fn linear_gamma_curve() -> [u8; TONE_LUT_ENTRIES] {
    std::array::from_fn(|index| (index >> 2) as u8)
}

/// Build the programmed raw-to-output LUT with black-level subtraction folded
/// into the table indices.
///
/// `gamma_curve` is indexed by a black-corrected ten-bit sample.  Inputs below
/// `black_level` therefore map through entry zero.
pub fn build_tone_lut(
    black_level: u16,
    gamma_curve: &[u8; TONE_LUT_ENTRIES],
) -> Result<[u8; TONE_LUT_ENTRIES], MonoIspError> {
    if black_level > RAW_MAX {
        return Err(MonoIspError::BlackLevelOutOfRange(black_level));
    }
    Ok(std::array::from_fn(|raw| {
        let corrected = (raw as u16).saturating_sub(black_level);
        gamma_curve[usize::from(corrected)]
    }))
}

/// Descriptive alias for [`build_tone_lut`].
pub fn build_black_level_gamma_lut(
    black_level: u16,
    gamma_curve: &[u8; TONE_LUT_ENTRIES],
) -> Result<[u8; TONE_LUT_ENTRIES], MonoIspError> {
    build_tone_lut(black_level, gamma_curve)
}

/// Static configuration of the functional reference model.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MonoIspConfig {
    pub width: usize,
    pub height: usize,
    pub tone_lut: [u8; TONE_LUT_ENTRIES],
    pub lut_enabled: bool,
    pub histogram_enabled: bool,
    pub defects: Vec<DefectEntry>,
}

impl MonoIspConfig {
    #[must_use]
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            tone_lut: linear_gamma_curve(),
            lut_enabled: false,
            histogram_enabled: false,
            defects: Vec::new(),
        }
    }

    #[must_use]
    pub fn with_tone_lut(mut self, tone_lut: [u8; TONE_LUT_ENTRIES]) -> Self {
        self.tone_lut = tone_lut;
        self
    }

    #[must_use]
    pub fn with_lut_enabled(mut self, enabled: bool) -> Self {
        self.lut_enabled = enabled;
        self
    }

    #[must_use]
    pub fn with_histogram_enabled(mut self, enabled: bool) -> Self {
        self.histogram_enabled = enabled;
        self
    }

    #[must_use]
    pub fn with_defects(mut self, defects: Vec<DefectEntry>) -> Self {
        self.defects = defects;
        self
    }
}

impl Default for MonoIspConfig {
    fn default() -> Self {
        Self::new(1280, 800)
    }
}

/// Stateful functional model of the monochrome ISP.
#[derive(Clone, Debug)]
pub struct MonoIspReference {
    config: MonoIspConfig,
    packed_defects: PackedDefectTable,
    histogram: [u32; HISTOGRAM_BINS],
    histogram_initialized: bool,
}

impl MonoIspReference {
    /// Validate a semantic configuration and create a reference model.
    pub fn new(config: MonoIspConfig) -> Result<Self, MonoIspError> {
        let packed_defects = pack_defect_table(&config.defects, config.width, config.height)?;
        Ok(Self {
            config,
            packed_defects,
            histogram: [0; HISTOGRAM_BINS],
            histogram_initialized: false,
        })
    }

    /// Create a model from the exact cumulative words programmed into RTL.
    pub fn from_packed_defects(
        mut config: MonoIspConfig,
        words: &[u32],
    ) -> Result<Self, MonoIspError> {
        config.defects = unpack_defect_table(words, config.width, config.height)?;
        let mut model = Self::new(config)?;
        model.packed_defects = PackedDefectTable {
            words: words.to_vec(),
        };
        Ok(model)
    }

    #[must_use]
    pub fn config(&self) -> &MonoIspConfig {
        &self.config
    }

    #[must_use]
    pub fn packed_defects(&self) -> &PackedDefectTable {
        &self.packed_defects
    }

    #[must_use]
    pub fn histogram(&self) -> &[u32; HISTOGRAM_BINS] {
        &self.histogram
    }

    #[must_use]
    pub const fn histogram_initialized(&self) -> bool {
        self.histogram_initialized
    }

    /// Model the completed 512-cycle hardware histogram-clear operation.
    pub fn clear_histogram(&mut self) {
        self.histogram.fill(0);
        self.histogram_initialized = true;
    }

    /// Generate the 512 records produced by the histogram dump stream.
    #[must_use]
    pub fn histogram_dump(&self) -> Vec<u32> {
        self.histogram
            .iter()
            .enumerate()
            .map(|(bin, &count)| {
                let first = if bin == 0 { HISTOGRAM_RECORD_FIRST } else { 0 };
                let last = if bin + 1 == HISTOGRAM_BINS {
                    HISTOGRAM_RECORD_LAST
                } else {
                    0
                };
                first
                    | last
                    | ((bin as u32) << HISTOGRAM_RECORD_BIN_SHIFT)
                    | (count & HISTOGRAM_RECORD_COUNT_MASK)
            })
            .collect()
    }

    /// Process one complete row-major frame of scalar pixels.
    pub fn process_frame(&mut self, raw_pixels: &[u16]) -> Result<Vec<u8>, MonoIspError> {
        let pair_count = frame_pair_count(self.config.width, self.config.height)? as usize;
        let expected = pair_count * 2;
        if raw_pixels.len() != expected {
            return Err(MonoIspError::FrameLength {
                expected,
                actual: raw_pixels.len(),
            });
        }
        if let Some((index, &value)) = raw_pixels
            .iter()
            .enumerate()
            .find(|&(_, &value)| value > RAW_MAX)
        {
            return Err(MonoIspError::RawPixelOutOfRange { index, value });
        }
        if self.config.histogram_enabled && !self.histogram_initialized {
            return Err(MonoIspError::HistogramNotInitialized);
        }

        let pairs: Vec<[u16; 2]> = raw_pixels
            .chunks_exact(2)
            .map(|pair| [pair[0], pair[1]])
            .collect();
        let output_pairs = self.process_validated_pairs(&pairs);
        Ok(output_pairs.into_iter().flatten().collect())
    }

    /// Process one complete row-major frame already grouped into pixel pairs.
    pub fn process_frame_pairs(
        &mut self,
        raw_pairs: &[[u16; 2]],
    ) -> Result<Vec<[u8; 2]>, MonoIspError> {
        let expected = frame_pair_count(self.config.width, self.config.height)? as usize;
        if raw_pairs.len() != expected {
            return Err(MonoIspError::PairFrameLength {
                expected,
                actual: raw_pairs.len(),
            });
        }
        if let Some((pair_index, lane, value)) =
            raw_pairs.iter().enumerate().find_map(|(pair_index, pair)| {
                pair.iter().enumerate().find_map(|(lane, &value)| {
                    (value > RAW_MAX).then_some((pair_index, lane, value))
                })
            })
        {
            return Err(MonoIspError::RawPixelOutOfRange {
                index: pair_index * 2 + lane,
                value,
            });
        }
        if self.config.histogram_enabled && !self.histogram_initialized {
            return Err(MonoIspError::HistogramNotInitialized);
        }
        Ok(self.process_validated_pairs(raw_pairs))
    }

    fn process_validated_pairs(&mut self, raw_pairs: &[[u16; 2]]) -> Vec<[u8; 2]> {
        let pairs_per_row = self.config.width / 2;
        let mut next_defect_index = 0usize;
        let mut previous_corrected = 0u16;
        let mut output = Vec::with_capacity(raw_pairs.len());

        for (pair_index, &[raw0, raw1]) in raw_pairs.iter().enumerate() {
            if pair_index % pairs_per_row == 0 {
                previous_corrected = 0;
            }

            let mask = self
                .config
                .defects
                .get(next_defect_index)
                .filter(|entry| entry.pair_address == pair_index as u32)
                .map(|entry| entry.mask);
            if mask.is_some() {
                next_defect_index += 1;
            }

            let corrected = match mask {
                None => [raw0, raw1],
                Some(DefectMask::Lane0) => [raw1, raw1],
                Some(DefectMask::Lane1) => [raw0, raw0],
                Some(DefectMask::Both) => [previous_corrected, previous_corrected],
            };
            previous_corrected = corrected[1];

            if self.config.histogram_enabled {
                let average = (u32::from(corrected[0]) + u32::from(corrected[1]) + 1) >> 1;
                let bin = (average >> 1) as usize;
                self.histogram[bin] = self.histogram[bin]
                    .saturating_add(1)
                    .min(HISTOGRAM_COUNT_MAX);
            }

            let mapped = if self.config.lut_enabled {
                [
                    self.config.tone_lut[usize::from(corrected[0])],
                    self.config.tone_lut[usize::from(corrected[1])],
                ]
            } else {
                [(corrected[0] >> 2) as u8, (corrected[1] >> 2) as u8]
            };
            output.push(mapped);
        }

        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cumulative_table_uses_continuations_and_round_trips() {
        let entries = [
            DefectEntry::new(0, DefectMask::Lane0),
            DefectEntry::new(DEFECT_DELTA_MAX + 7, DefectMask::Both),
        ];
        let packed = pack_defect_table(&entries, 1024, 1024).unwrap();
        assert_eq!(packed.words[0], 1);
        assert_eq!(packed.words[1], DEFECT_DELTA_MAX << 2);
        assert_eq!(packed.words[2], (7 << 2) | 3);
        assert_eq!(
            unpack_defect_table(&packed.words, 1024, 1024).unwrap(),
            entries
        );
    }

    #[test]
    fn correction_histogram_and_tone_order_match_specification() {
        let mut gamma = [0u8; TONE_LUT_ENTRIES];
        gamma[20] = 77;
        gamma[40] = 99;
        let tone_lut = build_tone_lut(10, &gamma).unwrap();
        let config = MonoIspConfig::new(4, 1)
            .with_tone_lut(tone_lut)
            .with_lut_enabled(true)
            .with_histogram_enabled(true)
            .with_defects(vec![DefectEntry::new(0, DefectMask::Lane0)]);
        let mut isp = MonoIspReference::new(config).unwrap();
        isp.clear_histogram();
        let output = isp.process_frame(&[10, 30, 50, 0]).unwrap();
        assert_eq!(output, [77, 77, 99, 0]);
        assert_eq!(isp.histogram()[15], 1);
        assert_eq!(isp.histogram()[12], 1);
    }

    #[test]
    fn double_defect_at_row_start_uses_zero() {
        let config = MonoIspConfig::new(4, 2).with_defects(vec![
            DefectEntry::new(1, DefectMask::Both),
            DefectEntry::new(2, DefectMask::Both),
        ]);
        let mut isp = MonoIspReference::new(config).unwrap();
        let output = isp.process_frame(&[4, 8, 12, 16, 20, 24, 28, 32]).unwrap();
        assert_eq!(output, [1, 2, 2, 2, 0, 0, 7, 8]);
    }

    #[test]
    fn histogram_dump_marks_endpoints() {
        let isp = MonoIspReference::new(MonoIspConfig::new(2, 1)).unwrap();
        let dump = isp.histogram_dump();
        assert_eq!(dump.len(), HISTOGRAM_BINS);
        assert_ne!(dump[0] & HISTOGRAM_RECORD_FIRST, 0);
        assert_eq!(dump[0] & HISTOGRAM_RECORD_LAST, 0);
        assert_ne!(dump[511] & HISTOGRAM_RECORD_LAST, 0);
        assert_eq!((dump[511] & HISTOGRAM_RECORD_BIN_MASK) >> 20, 511);
    }
}
