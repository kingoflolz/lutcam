//! Packet-level reference model for the fixed OV9281 CSI-2 receiver.
//!
//! The model starts at the aligned CSI-2 packet header. It intentionally does
//! not model the Titanium hard-lane FIFO, D-PHY low-power sequencing, B8 search,
//! lane deskew, or clock-domain crossing. It provides the bit-exact protocol
//! operations needed to verify the RTL above those physical receive stages:
//! header ECC, payload CRC, RAW10 packing, packet parsing, frame sequencing,
//! and the expected two-pixel output beats.

use core::fmt;

pub const IMAGE_WIDTH: usize = 1280;
pub const IMAGE_HEIGHT: usize = 800;
pub const PIXELS_PER_BEAT: usize = 2;
pub const BEATS_PER_LINE: usize = IMAGE_WIDTH / PIXELS_PER_BEAT;
pub const FRAME_PIXELS: usize = IMAGE_WIDTH * IMAGE_HEIGHT;
pub const FRAME_BEATS: usize = BEATS_PER_LINE * IMAGE_HEIGHT;
pub const RAW10_BYTES_PER_LINE: usize = IMAGE_WIDTH * 10 / 8;

pub const OV9281_VIRTUAL_CHANNEL: u8 = 0;
pub const DATA_TYPE_FRAME_START: u8 = 0x00;
pub const DATA_TYPE_FRAME_END: u8 = 0x01;
pub const DATA_TYPE_LINE_START: u8 = 0x02;
pub const DATA_TYPE_LINE_END: u8 = 0x03;
pub const DATA_TYPE_RAW10: u8 = 0x2b;

const ECC_COLUMNS: [u8; 24] = [
    0x07, 0x0b, 0x0d, 0x0e, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c, 0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c,
    0x31, 0x32, 0x34, 0x38, 0x1f, 0x2f, 0x37, 0x3b,
];

/// Calculate the six CSI-2 ECC parity bits for the first three header bytes.
///
/// Bit zero of `header_data[0]` is D0 and bit seven of `header_data[2]` is D23.
#[must_use]
pub fn header_ecc(header_data: [u8; 3]) -> u8 {
    let mut ecc = 0_u8;
    for (bit, column) in ECC_COLUMNS.iter().copied().enumerate() {
        let byte = bit / 8;
        let bit_in_byte = bit % 8;
        if header_data[byte] & (1_u8 << bit_in_byte) != 0 {
            ecc ^= column;
        }
    }
    ecc
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeaderCorrection {
    None,
    DataBit(u8),
    EccBit(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeaderEccError {
    ReservedBitsSet(u8),
    Uncorrectable { syndrome: u8 },
}

impl fmt::Display for HeaderEccError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ReservedBitsSet(bits) => {
                write!(
                    formatter,
                    "reserved header ECC bits are nonzero: {bits:#04x}"
                )
            }
            Self::Uncorrectable { syndrome } => {
                write!(
                    formatter,
                    "uncorrectable CSI-2 header syndrome {syndrome:#04x}"
                )
            }
        }
    }
}

impl std::error::Error for HeaderEccError {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CsiHeader {
    pub data_id: u8,
    pub data_field: u16,
}

impl CsiHeader {
    #[must_use]
    pub const fn virtual_channel(self) -> u8 {
        self.data_id >> 6
    }

    #[must_use]
    pub const fn data_type(self) -> u8 {
        self.data_id & 0x3f
    }

    #[must_use]
    pub const fn data_bytes(self) -> [u8; 3] {
        let field = self.data_field.to_le_bytes();
        [self.data_id, field[0], field[1]]
    }

    #[must_use]
    pub fn encode(self) -> [u8; 4] {
        let data = self.data_bytes();
        [data[0], data[1], data[2], header_ecc(data)]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DecodedHeader {
    pub header: CsiHeader,
    pub correction: HeaderCorrection,
}

/// Correct a single header data or ECC parity bit and reject every double-bit
/// error in the 30 protected bits.
pub fn decode_header(bytes: [u8; 4]) -> Result<DecodedHeader, HeaderEccError> {
    if bytes[3] & 0xc0 != 0 {
        return Err(HeaderEccError::ReservedBitsSet(bytes[3] & 0xc0));
    }

    let mut data = [bytes[0], bytes[1], bytes[2]];
    let syndrome = (bytes[3] & 0x3f) ^ header_ecc(data);
    let correction = if syndrome == 0 {
        HeaderCorrection::None
    } else if let Some(bit) = ECC_COLUMNS.iter().position(|column| *column == syndrome) {
        data[bit / 8] ^= 1_u8 << (bit % 8);
        HeaderCorrection::DataBit(bit as u8)
    } else if syndrome.is_power_of_two() {
        HeaderCorrection::EccBit(syndrome.trailing_zeros() as u8)
    } else {
        return Err(HeaderEccError::Uncorrectable { syndrome });
    };

    Ok(DecodedHeader {
        header: CsiHeader {
            data_id: data[0],
            data_field: u16::from_le_bytes([data[1], data[2]]),
        },
        correction,
    })
}

/// CSI-2 payload CRC-16, reflected, with polynomial 0x8408 and initial 0xffff.
#[must_use]
pub fn payload_crc16(payload: &[u8]) -> u16 {
    let mut crc = 0xffff_u16;
    for byte in payload {
        crc ^= u16::from(*byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0x8408
            } else {
                crc >> 1
            };
        }
    }
    crc
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ParsedPacket {
    Short {
        header: CsiHeader,
        correction: HeaderCorrection,
    },
    Long {
        header: CsiHeader,
        correction: HeaderCorrection,
        payload: Vec<u8>,
        received_crc: u16,
        calculated_crc: u16,
    },
}

impl ParsedPacket {
    #[must_use]
    pub const fn header(&self) -> CsiHeader {
        match self {
            Self::Short { header, .. } | Self::Long { header, .. } => *header,
        }
    }

    #[must_use]
    pub const fn correction(&self) -> HeaderCorrection {
        match self {
            Self::Short { correction, .. } | Self::Long { correction, .. } => *correction,
        }
    }

    #[must_use]
    pub const fn crc_is_valid(&self) -> Option<bool> {
        match self {
            Self::Short { .. } => None,
            Self::Long {
                received_crc,
                calculated_crc,
                ..
            } => Some(*received_crc == *calculated_crc),
        }
    }

    #[must_use]
    pub fn payload(&self) -> Option<&[u8]> {
        match self {
            Self::Short { .. } => None,
            Self::Long { payload, .. } => Some(payload),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PacketParseError {
    TruncatedHeader { actual: usize },
    HeaderEcc(HeaderEccError),
    LengthMismatch { expected: usize, actual: usize },
}

impl fmt::Display for PacketParseError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TruncatedHeader { actual } => {
                write!(formatter, "CSI-2 header is truncated to {actual} bytes")
            }
            Self::HeaderEcc(error) => error.fmt(formatter),
            Self::LengthMismatch { expected, actual } => write!(
                formatter,
                "CSI-2 packet length is {actual} bytes; expected {expected}"
            ),
        }
    }
}

impl std::error::Error for PacketParseError {}

impl From<HeaderEccError> for PacketParseError {
    fn from(error: HeaderEccError) -> Self {
        Self::HeaderEcc(error)
    }
}

const fn is_short_data_type(data_type: u8) -> bool {
    data_type <= 0x0f
}

/// Parse one packet after its B8 synchronization byte and before HS trail.
///
/// Short-packet types occupy data-type values 0x00 through 0x0f and must be
/// exactly four bytes. Other types are interpreted as long packets whose exact
/// payload length comes from the corrected header.
pub fn parse_packet(bytes: &[u8]) -> Result<ParsedPacket, PacketParseError> {
    if bytes.len() < 4 {
        return Err(PacketParseError::TruncatedHeader {
            actual: bytes.len(),
        });
    }
    let decoded = decode_header([bytes[0], bytes[1], bytes[2], bytes[3]])?;
    if is_short_data_type(decoded.header.data_type()) {
        if bytes.len() != 4 {
            return Err(PacketParseError::LengthMismatch {
                expected: 4,
                actual: bytes.len(),
            });
        }
        return Ok(ParsedPacket::Short {
            header: decoded.header,
            correction: decoded.correction,
        });
    }

    let payload_len = usize::from(decoded.header.data_field);
    let expected_len = 4 + payload_len + 2;
    if bytes.len() != expected_len {
        return Err(PacketParseError::LengthMismatch {
            expected: expected_len,
            actual: bytes.len(),
        });
    }
    let payload = bytes[4..4 + payload_len].to_vec();
    let received_crc = u16::from_le_bytes([bytes[4 + payload_len], bytes[5 + payload_len]]);
    let calculated_crc = payload_crc16(&payload);
    Ok(ParsedPacket::Long {
        header: decoded.header,
        correction: decoded.correction,
        payload,
        received_crc,
        calculated_crc,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PacketBuildError {
    InvalidVirtualChannel(u8),
    InvalidDataType(u8),
    WrongPacketKind(u8),
    PayloadTooLong(usize),
}

impl fmt::Display for PacketBuildError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidVirtualChannel(vc) => write!(formatter, "invalid CSI-2 VC {vc}"),
            Self::InvalidDataType(data_type) => {
                write!(formatter, "invalid CSI-2 data type {data_type:#04x}")
            }
            Self::WrongPacketKind(data_type) => {
                write!(
                    formatter,
                    "data type {data_type:#04x} has the wrong packet kind"
                )
            }
            Self::PayloadTooLong(length) => {
                write!(
                    formatter,
                    "CSI-2 payload has {length} bytes; maximum is 65535"
                )
            }
        }
    }
}

impl std::error::Error for PacketBuildError {}

fn data_id(virtual_channel: u8, data_type: u8) -> Result<u8, PacketBuildError> {
    if virtual_channel >= 4 {
        return Err(PacketBuildError::InvalidVirtualChannel(virtual_channel));
    }
    if data_type >= 64 {
        return Err(PacketBuildError::InvalidDataType(data_type));
    }
    Ok((virtual_channel << 6) | data_type)
}

pub fn build_short_packet(
    virtual_channel: u8,
    data_type: u8,
    data_field: u16,
) -> Result<[u8; 4], PacketBuildError> {
    if !is_short_data_type(data_type) {
        return Err(PacketBuildError::WrongPacketKind(data_type));
    }
    Ok(CsiHeader {
        data_id: data_id(virtual_channel, data_type)?,
        data_field,
    }
    .encode())
}

pub fn build_long_packet(
    virtual_channel: u8,
    data_type: u8,
    payload: &[u8],
) -> Result<Vec<u8>, PacketBuildError> {
    if is_short_data_type(data_type) {
        return Err(PacketBuildError::WrongPacketKind(data_type));
    }
    let payload_len = u16::try_from(payload.len())
        .map_err(|_| PacketBuildError::PayloadTooLong(payload.len()))?;
    let header = CsiHeader {
        data_id: data_id(virtual_channel, data_type)?,
        data_field: payload_len,
    }
    .encode();
    let mut packet = Vec::with_capacity(4 + payload.len() + 2);
    packet.extend_from_slice(&header);
    packet.extend_from_slice(payload);
    packet.extend_from_slice(&payload_crc16(payload).to_le_bytes());
    Ok(packet)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Raw10Error {
    PixelCountNotMultipleOfFour(usize),
    ByteCountNotMultipleOfFive(usize),
    PixelOutOfRange { index: usize, value: u16 },
}

impl fmt::Display for Raw10Error {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PixelCountNotMultipleOfFour(count) => {
                write!(
                    formatter,
                    "RAW10 input has {count} pixels; expected groups of four"
                )
            }
            Self::ByteCountNotMultipleOfFive(count) => {
                write!(
                    formatter,
                    "RAW10 input has {count} bytes; expected groups of five"
                )
            }
            Self::PixelOutOfRange { index, value } => {
                write!(formatter, "RAW10 pixel {index} is out of range: {value}")
            }
        }
    }
}

impl std::error::Error for Raw10Error {}

pub fn pack_raw10(pixels: &[u16]) -> Result<Vec<u8>, Raw10Error> {
    if !pixels.len().is_multiple_of(4) {
        return Err(Raw10Error::PixelCountNotMultipleOfFour(pixels.len()));
    }
    for (index, value) in pixels.iter().copied().enumerate() {
        if value > 0x03ff {
            return Err(Raw10Error::PixelOutOfRange { index, value });
        }
    }

    let mut bytes = Vec::with_capacity(pixels.len() * 5 / 4);
    for group in pixels.chunks_exact(4) {
        bytes.extend_from_slice(&[
            (group[0] >> 2) as u8,
            (group[1] >> 2) as u8,
            (group[2] >> 2) as u8,
            (group[3] >> 2) as u8,
            (group[0] & 3) as u8
                | ((group[1] & 3) << 2) as u8
                | ((group[2] & 3) << 4) as u8
                | ((group[3] & 3) << 6) as u8,
        ]);
    }
    Ok(bytes)
}

pub fn unpack_raw10(bytes: &[u8]) -> Result<Vec<u16>, Raw10Error> {
    if !bytes.len().is_multiple_of(5) {
        return Err(Raw10Error::ByteCountNotMultipleOfFive(bytes.len()));
    }
    let mut pixels = Vec::with_capacity(bytes.len() * 4 / 5);
    for group in bytes.chunks_exact(5) {
        let low = group[4];
        pixels.extend_from_slice(&[
            (u16::from(group[0]) << 2) | u16::from(low & 0x03),
            (u16::from(group[1]) << 2) | u16::from((low >> 2) & 0x03),
            (u16::from(group[2]) << 2) | u16::from((low >> 4) & 0x03),
            (u16::from(group[3]) << 2) | u16::from((low >> 6) & 0x03),
        ]);
    }
    Ok(pixels)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PixelBeat {
    pub pixels: [u16; 2],
    pub line: u16,
    pub beat: u16,
    pub sof: bool,
    pub sol: bool,
    pub eol: bool,
    pub eof: bool,
}

impl PixelBeat {
    #[must_use]
    pub const fn packed_data(self) -> u32 {
        self.pixels[0] as u32 | ((self.pixels[1] as u32) << 10)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PixelBeatError {
    WrongFramePixelCount { expected: usize, actual: usize },
    WrongLinePixelCount { expected: usize, actual: usize },
    LineOutOfRange(usize),
    Raw10(Raw10Error),
}

impl fmt::Display for PixelBeatError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongFramePixelCount { expected, actual } => {
                write!(formatter, "frame has {actual} pixels; expected {expected}")
            }
            Self::WrongLinePixelCount { expected, actual } => {
                write!(formatter, "line has {actual} pixels; expected {expected}")
            }
            Self::LineOutOfRange(line) => write!(formatter, "line {line} is out of range"),
            Self::Raw10(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for PixelBeatError {}

fn validate_pixels(pixels: &[u16]) -> Result<(), Raw10Error> {
    for (index, value) in pixels.iter().copied().enumerate() {
        if value > 0x03ff {
            return Err(Raw10Error::PixelOutOfRange { index, value });
        }
    }
    Ok(())
}

pub fn expected_line_beats(
    line_pixels: &[u16],
    line: usize,
) -> Result<Vec<PixelBeat>, PixelBeatError> {
    if line >= IMAGE_HEIGHT {
        return Err(PixelBeatError::LineOutOfRange(line));
    }
    if line_pixels.len() != IMAGE_WIDTH {
        return Err(PixelBeatError::WrongLinePixelCount {
            expected: IMAGE_WIDTH,
            actual: line_pixels.len(),
        });
    }
    validate_pixels(line_pixels).map_err(PixelBeatError::Raw10)?;

    let mut beats = Vec::with_capacity(BEATS_PER_LINE);
    for (beat, pair) in line_pixels.chunks_exact(2).enumerate() {
        beats.push(PixelBeat {
            pixels: [pair[0], pair[1]],
            line: line as u16,
            beat: beat as u16,
            sof: line == 0 && beat == 0,
            sol: beat == 0,
            eol: beat + 1 == BEATS_PER_LINE,
            eof: line + 1 == IMAGE_HEIGHT && beat + 1 == BEATS_PER_LINE,
        });
    }
    Ok(beats)
}

pub fn expected_frame_beats(frame_pixels: &[u16]) -> Result<Vec<PixelBeat>, PixelBeatError> {
    if frame_pixels.len() != FRAME_PIXELS {
        return Err(PixelBeatError::WrongFramePixelCount {
            expected: FRAME_PIXELS,
            actual: frame_pixels.len(),
        });
    }
    validate_pixels(frame_pixels).map_err(PixelBeatError::Raw10)?;
    let mut beats = Vec::with_capacity(FRAME_BEATS);
    for (line, pixels) in frame_pixels.chunks_exact(IMAGE_WIDTH).enumerate() {
        beats.extend(expected_line_beats(pixels, line)?);
    }
    Ok(beats)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ReceiverStatus(u16);

impl ReceiverStatus {
    pub const HEADER_ECC_CORRECTED: Self = Self(1 << 0);
    pub const HEADER_ECC_UNCORRECTABLE: Self = Self(1 << 1);
    pub const PAYLOAD_CRC_ERROR: Self = Self(1 << 2);
    pub const UNEXPECTED_PACKET: Self = Self(1 << 6);
    pub const WORD_COUNT_ERROR: Self = Self(1 << 7);
    pub const PACKET_LENGTH_ERROR: Self = Self(1 << 8);
    pub const FRAME_SEQUENCE_ERROR: Self = Self(1 << 9);
    pub const LINE_COUNT_ERROR: Self = Self(1 << 10);

    #[must_use]
    pub const fn bits(self) -> u16 {
        self.0
    }

    #[must_use]
    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 == flag.0
    }

    fn insert(&mut self, flag: Self) {
        self.0 |= flag.0;
    }
}

impl core::ops::BitOr for ReceiverStatus {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PacketOutcome {
    pub beats: Vec<PixelBeat>,
    pub new_status: ReceiverStatus,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FrameState {
    Idle,
    Active {
        frame_number: u16,
        lines: usize,
        extra_line_seen: bool,
    },
    DiscardUntilFrameStart,
}

/// Fixed-format frame and line validator used as an RTL scoreboarding oracle.
#[derive(Clone, Debug)]
pub struct Ov9281PacketReference {
    state: FrameState,
    status: ReceiverStatus,
}

impl Default for Ov9281PacketReference {
    fn default() -> Self {
        Self::new()
    }
}

impl Ov9281PacketReference {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: FrameState::Idle,
            status: ReceiverStatus(0),
        }
    }

    #[must_use]
    pub const fn status(&self) -> ReceiverStatus {
        self.status
    }

    #[must_use]
    pub const fn frame_is_active(&self) -> bool {
        matches!(self.state, FrameState::Active { .. })
    }

    fn abort_active_frame(&mut self) {
        if matches!(self.state, FrameState::Active { .. }) {
            self.state = FrameState::DiscardUntilFrameStart;
        }
    }

    fn begin_frame(&mut self, frame_number: u16) {
        if let FrameState::Active { lines, .. } = self.state {
            self.status.insert(ReceiverStatus::FRAME_SEQUENCE_ERROR);
            if lines != IMAGE_HEIGHT {
                self.status.insert(ReceiverStatus::LINE_COUNT_ERROR);
            }
        }
        self.state = FrameState::Active {
            frame_number,
            lines: 0,
            extra_line_seen: false,
        };
    }

    fn end_frame(&mut self, frame_number: u16) {
        match self.state {
            FrameState::Active {
                frame_number: expected,
                lines,
                extra_line_seen,
            } => {
                if frame_number != expected || lines != IMAGE_HEIGHT || extra_line_seen {
                    self.status.insert(ReceiverStatus::FRAME_SEQUENCE_ERROR);
                }
                if lines != IMAGE_HEIGHT || extra_line_seen {
                    self.status.insert(ReceiverStatus::LINE_COUNT_ERROR);
                }
                self.state = FrameState::Idle;
            }
            FrameState::Idle => self.status.insert(ReceiverStatus::FRAME_SEQUENCE_ERROR),
            FrameState::DiscardUntilFrameStart => {}
        }
    }

    fn accept_raw10(&mut self, payload: &[u8]) -> Vec<PixelBeat> {
        let FrameState::Active {
            frame_number: _,
            lines,
            extra_line_seen,
        } = &mut self.state
        else {
            if matches!(self.state, FrameState::Idle) {
                self.status.insert(ReceiverStatus::FRAME_SEQUENCE_ERROR);
            }
            return Vec::new();
        };

        if *lines >= IMAGE_HEIGHT {
            *extra_line_seen = true;
            self.status.insert(ReceiverStatus::LINE_COUNT_ERROR);
            return Vec::new();
        }

        let pixels = unpack_raw10(payload).expect("fixed RAW10 word count is group aligned");
        let beats = expected_line_beats(&pixels, *lines)
            .expect("fixed RAW10 word count produces one complete image line");
        *lines += 1;
        beats
    }

    /// Consume one complete packet burst, excluding B8 and HS trail.
    ///
    /// Protocol failures become sticky status bits and suppress output according
    /// to the fixed receiver recovery policy. CRC failure is diagnostic only:
    /// the line's already-visible pixels remain in the returned beat stream.
    pub fn ingest_packet(&mut self, bytes: &[u8]) -> PacketOutcome {
        let status_before = self.status;
        let parsed = match parse_packet(bytes) {
            Ok(packet) => packet,
            Err(PacketParseError::HeaderEcc(_)) => {
                self.status.insert(ReceiverStatus::HEADER_ECC_UNCORRECTABLE);
                self.abort_active_frame();
                return self.outcome(Vec::new(), status_before);
            }
            Err(
                PacketParseError::TruncatedHeader { .. } | PacketParseError::LengthMismatch { .. },
            ) => {
                self.status.insert(ReceiverStatus::PACKET_LENGTH_ERROR);
                self.abort_active_frame();
                return self.outcome(Vec::new(), status_before);
            }
        };

        if parsed.correction() != HeaderCorrection::None {
            self.status.insert(ReceiverStatus::HEADER_ECC_CORRECTED);
        }
        if parsed.crc_is_valid() == Some(false) {
            self.status.insert(ReceiverStatus::PAYLOAD_CRC_ERROR);
        }

        let header = parsed.header();
        let vc = header.virtual_channel();
        let data_type = header.data_type();
        let beats = match (vc, data_type, parsed) {
            (OV9281_VIRTUAL_CHANNEL, DATA_TYPE_FRAME_START, ParsedPacket::Short { .. }) => {
                self.begin_frame(header.data_field);
                Vec::new()
            }
            (OV9281_VIRTUAL_CHANNEL, DATA_TYPE_FRAME_END, ParsedPacket::Short { .. }) => {
                self.end_frame(header.data_field);
                Vec::new()
            }
            (OV9281_VIRTUAL_CHANNEL, DATA_TYPE_RAW10, ParsedPacket::Long { payload, .. }) => {
                if header.data_field != RAW10_BYTES_PER_LINE as u16 {
                    self.status.insert(ReceiverStatus::WORD_COUNT_ERROR);
                    if matches!(self.state, FrameState::Idle) {
                        self.status.insert(ReceiverStatus::FRAME_SEQUENCE_ERROR);
                    }
                    self.abort_active_frame();
                    Vec::new()
                } else {
                    self.accept_raw10(&payload)
                }
            }
            _ => {
                self.status.insert(ReceiverStatus::UNEXPECTED_PACKET);
                Vec::new()
            }
        };
        self.outcome(beats, status_before)
    }

    fn outcome(&self, beats: Vec<PixelBeat>, status_before: ReceiverStatus) -> PacketOutcome {
        PacketOutcome {
            beats,
            new_status: ReceiverStatus(self.status.bits() & !status_before.bits()),
        }
    }

    /// Mark the input stream complete, detecting a missing Frame End.
    pub fn finish_stream(&mut self) -> ReceiverStatus {
        let before = self.status;
        if let FrameState::Active { lines, .. } = self.state {
            self.status.insert(ReceiverStatus::FRAME_SEQUENCE_ERROR);
            if lines != IMAGE_HEIGHT {
                self.status.insert(ReceiverStatus::LINE_COUNT_ERROR);
            }
            self.state = FrameState::DiscardUntilFrameStart;
        }
        ReceiverStatus(self.status.bits() & !before.bits())
    }
}
