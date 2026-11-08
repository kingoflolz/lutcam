//! Bounded image-stream assembly; the LWC decoder establishes current-format completion.
//!
//! Packets reserve their header byte and flags as zero. Once another frame
//! starts, a contiguous predecessor becomes a decode candidate. A reordered tail can
//! update that candidate until it ages out of the small frame window. Never
//! display candidate bytes without a successful [`EncodedImage::decode`].

use std::collections::BTreeMap;
use std::fmt;

use lwc::{Codec, CodecError};

pub const WIDTH: usize = lwc::FRAME_WIDTH;
pub const HEIGHT: usize = lwc::FRAME_HEIGHT;
pub const HEADER_SIZE: usize = 32;
pub const MAX_DATAGRAM_SIZE: usize = 1440;
pub const MAX_IMAGE_SIZE: usize = WIDTH * HEIGHT / 8;
const MAGIC: u32 = 0x4c55_5443;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct StreamError(pub &'static str);

impl fmt::Display for StreamError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.0)
    }
}

impl std::error::Error for StreamError {}

#[derive(Clone, Copy, Debug)]
pub struct ImagePacket<'a> {
    pub camera: u8,
    /// Reserved header byte, always zero.
    pub version: u8,
    pub frame_id: u32,
    pub sequence: u32,
    pub offset: usize,
    pub flags: u32,
    pub payload: &'a [u8],
}

/// Parse only the image product. Image, child and pose products use separate
/// UDP ports on the board; accepting another product here would hide bad setup.
pub fn parse_datagram(data: &[u8]) -> Result<ImagePacket<'_>, StreamError> {
    if !(HEADER_SIZE..=MAX_DATAGRAM_SIZE).contains(&data.len()) {
        return Err(StreamError("truncated or oversized UDP datagram"));
    }
    let word =
        |index: usize| u32::from_be_bytes(data[index * 4..index * 4 + 4].try_into().unwrap());
    let [version, camera, stream, header_size] = word(1).to_be_bytes();
    if word(0) != MAGIC || version != 0 || header_size != 32 {
        return Err(StreamError("unsupported LUTCam header"));
    }
    if camera > 1 || stream != 1 {
        return Err(StreamError("not a supported camera image product"));
    }
    let flags = word(6);
    if word(7) != 0 || flags != 0 {
        return Err(StreamError("nonzero reserved header bits"));
    }
    let offset = word(4) as usize;
    let length = word(5) as usize;
    if length != data.len() - HEADER_SIZE {
        return Err(StreamError("payload length does not match datagram"));
    }
    if !offset.is_multiple_of(2) || !length.is_multiple_of(2) || length == 0 {
        return Err(StreamError("unaligned or empty image fragment"));
    }
    let end = offset
        .checked_add(length)
        .ok_or(StreamError("image extent overflow"))?;
    if end > MAX_IMAGE_SIZE {
        return Err(StreamError("invalid image end boundary"));
    }
    Ok(ImagePacket {
        camera,
        version,
        frame_id: word(2),
        sequence: word(3),
        offset,
        flags,
        payload: &data[HEADER_SIZE..],
    })
}

#[derive(Clone, Debug)]
pub struct EncodedImage {
    pub camera: u8,
    pub frame_id: u32,
    /// Reserved header byte, always zero.
    pub version: u8,
    pub bytes: Vec<u8>,
}

#[derive(Clone, Debug)]
pub struct DecodedImage {
    pub camera: u8,
    pub frame_id: u32,
    pub pixels: Vec<u8>,
}

impl EncodedImage {
    pub fn decode(self) -> Result<DecodedImage, CodecError> {
        let pixels = Codec::decode_compact(&self.bytes, WIDTH, HEIGHT)?;
        Ok(DecodedImage {
            camera: self.camera,
            frame_id: self.frame_id,
            pixels,
        })
    }
}

#[derive(Clone, Debug, Default)]
pub struct StreamStats {
    pub packets: u64,
    pub image_packets: u64,
    pub duplicates: u64,
    pub invalid_packets: u64,
    pub conflicting_frames: u64,
    pub stale_packets: u64,
    /// Frames evicted with holes, or without ever yielding a decode candidate.
    /// A candidate may still be rejected by the codec; this is not a loss total.
    pub incomplete_evictions: u64,
    pub candidates: u64,
}

#[derive(Debug)]
struct Fragment {
    sequence: u32,
    bytes: Vec<u8>,
}

#[derive(Debug)]
struct FrameParts {
    id: u32,
    version: u8,
    fragments: BTreeMap<usize, Fragment>,
    sequences: BTreeMap<u32, usize>,
    received_bytes: usize,
    revision: usize,
    emitted_revision: usize,
    invalid: bool,
}

impl FrameParts {
    fn new(packet: &ImagePacket<'_>) -> Self {
        Self {
            id: packet.frame_id,
            version: packet.version,
            fragments: BTreeMap::new(),
            sequences: BTreeMap::new(),
            received_bytes: 0,
            revision: 0,
            emitted_revision: 0,
            invalid: false,
        }
    }

    fn insert(&mut self, packet: &ImagePacket<'_>) -> Result<bool, StreamError> {
        let end = packet.offset + packet.payload.len();
        if let Some(fragment) = self.fragments.get(&packet.offset) {
            return if fragment.bytes == packet.payload && fragment.sequence == packet.sequence {
                Ok(false)
            } else {
                Err(StreamError("conflicting duplicate image fragment"))
            };
        }
        if self.sequences.contains_key(&packet.sequence) {
            return Err(StreamError("packet sequence reused for another range"));
        }
        if let Some((offset, previous)) = self.fragments.range(..packet.offset).next_back() {
            let previous_end = offset + previous.bytes.len();
            if previous_end > packet.offset {
                return Err(StreamError("overlapping image fragment ranges"));
            }
            if previous_end == packet.offset && previous.sequence.wrapping_add(1) != packet.sequence
            {
                return Err(StreamError("nonconsecutive adjacent fragment sequences"));
            }
        }
        if let Some((offset, next)) = self.fragments.range(packet.offset..).next() {
            if end > *offset {
                return Err(StreamError("overlapping image fragment ranges"));
            }
            if end == *offset && packet.sequence.wrapping_add(1) != next.sequence {
                return Err(StreamError("nonconsecutive adjacent fragment sequences"));
            }
        }
        self.fragments.insert(
            packet.offset,
            Fragment {
                sequence: packet.sequence,
                bytes: packet.payload.to_vec(),
            },
        );
        self.received_bytes += packet.payload.len();
        self.sequences.insert(packet.sequence, packet.offset);
        self.revision += 1;
        Ok(true)
    }

    fn contiguous(&self) -> bool {
        !self.invalid
            && self.received_bytes != 0
            && self
                .fragments
                .last_key_value()
                .is_some_and(|(offset, fragment)| {
                    offset + fragment.bytes.len() == self.received_bytes
                })
    }

    fn candidate(&mut self, camera: u8, allow_compact: bool) -> Option<EncodedImage> {
        if self.revision == self.emitted_revision || !self.contiguous() || !allow_compact {
            return None;
        }
        let mut bytes = Vec::with_capacity(self.received_bytes);
        for fragment in self.fragments.values() {
            bytes.extend_from_slice(&fragment.bytes);
        }
        self.emitted_revision = self.revision;
        Some(EncodedImage {
            camera,
            frame_id: self.id,
            version: self.version,
            bytes,
        })
    }
}

/// RFC-1982-style comparison of the 32-bit counter. Exactly half a cycle apart
/// is ambiguous and is deliberately not treated as newer.
pub fn frame_is_newer(candidate: u32, reference: u32) -> bool {
    let distance = candidate.wrapping_sub(reference);
    distance != 0 && distance < 0x8000_0000
}

/// Stores at most `max_pending` frame IDs, each with at most 128,000 payload
/// bytes. Old IDs cannot evict newer frames, including around counter wrap.
pub struct StreamAssembler {
    camera: u8,
    max_pending: u32,
    newest: Option<u32>,
    frames: Vec<FrameParts>,
    stats: StreamStats,
}

impl StreamAssembler {
    pub fn new(camera: u8, max_pending: usize) -> Self {
        assert!(camera <= 1, "camera must be 0 or 1");
        assert!(
            (1..=16).contains(&max_pending),
            "frame window must be 1..=16"
        );
        Self {
            camera,
            max_pending: max_pending as u32,
            newest: None,
            frames: Vec::new(),
            stats: StreamStats::default(),
        }
    }

    pub fn stats(&self) -> &StreamStats {
        &self.stats
    }

    /// Forget frame/sequence history after a known board or capture restart.
    /// Statistics remain cumulative for the current viewer session.
    pub fn reset(&mut self) {
        self.newest = None;
        self.frames.clear();
    }

    pub fn push(&mut self, datagram: &[u8]) -> Result<Vec<EncodedImage>, StreamError> {
        self.stats.packets += 1;
        let packet = match parse_datagram(datagram) {
            Ok(packet) if packet.camera == self.camera => packet,
            result => {
                self.stats.invalid_packets += 1;
                return Err(result
                    .err()
                    .unwrap_or(StreamError("packet for another camera")));
            }
        };
        self.stats.image_packets += 1;
        if let Some(newest) = self.newest {
            if frame_is_newer(packet.frame_id, newest) {
                self.newest = Some(packet.frame_id);
            } else if newest.wrapping_sub(packet.frame_id) >= self.max_pending {
                self.stats.stale_packets += 1;
                return Ok(Vec::new());
            }
        } else {
            self.newest = Some(packet.frame_id);
        }
        let newest = self.newest.unwrap();
        // Candidate creation precedes eviction, allowing max_pending=1 to
        // produce its predecessor when the next frame starts.
        let mut candidates = self.collect_candidates(false);
        self.frames.retain(|frame| {
            let retain = newest.wrapping_sub(frame.id) < self.max_pending;
            if !retain && !frame.invalid && (!frame.contiguous() || frame.emitted_revision == 0) {
                self.stats.incomplete_evictions += 1;
            }
            retain
        });
        let index = self
            .frames
            .iter()
            .position(|frame| frame.id == packet.frame_id);
        let index = match index {
            Some(index) => index,
            None => {
                self.frames.push(FrameParts::new(&packet));
                self.frames.len() - 1
            }
        };
        let frame = &mut self.frames[index];
        if frame.invalid {
            self.stats.stale_packets += 1;
            return Ok(candidates);
        }
        match frame.insert(&packet) {
            Ok(false) => self.stats.duplicates += 1,
            Ok(true) => {}
            Err(error) => {
                frame.invalid = true;
                frame.fragments.clear();
                frame.sequences.clear();
                self.stats.conflicting_frames += 1;
                return Err(error);
            }
        }
        candidates.extend(self.collect_candidates(false));
        Ok(candidates)
    }

    /// Call after a receive timeout (about 10 ms), not after every packet.
    /// This permits the last image frame to finish even if the stream stops. A
    /// truncated candidate simply fails decode; later data can still amend it.
    pub fn poll(&mut self) -> Vec<EncodedImage> {
        self.collect_candidates(true)
    }

    fn collect_candidates(&mut self, include_latest: bool) -> Vec<EncodedImage> {
        let Some(newest) = self.newest else {
            return Vec::new();
        };
        self.frames
            .sort_by_key(|frame| std::cmp::Reverse(newest.wrapping_sub(frame.id)));
        let candidates: Vec<_> = self
            .frames
            .iter_mut()
            .filter_map(|frame| {
                let age = newest.wrapping_sub(frame.id);
                // A large counter jump must not publish a long-stale frame.
                // Age one is allowed even for a one-frame assembly window.
                if age >= self.max_pending && age != 1 {
                    return None;
                }
                frame.candidate(self.camera, include_latest || frame.id != newest)
            })
            .collect();
        self.stats.candidates += candidates.len() as u64;
        candidates
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(
        version: u8,
        camera: u8,
        frame: u32,
        sequence: u32,
        offset: usize,
        flags: u32,
        payload: &[u8],
    ) -> Vec<u8> {
        let words = [
            MAGIC,
            u32::from_be_bytes([version, camera, 1, 32]),
            frame,
            sequence,
            offset as u32,
            payload.len() as u32,
            flags,
            0,
        ];
        let mut bytes: Vec<_> = words.into_iter().flat_map(u32::to_be_bytes).collect();
        bytes.extend_from_slice(payload);
        bytes
    }

    fn image_packets(frame: u32, first_sequence: u32, bytes: &[u8]) -> Vec<Vec<u8>> {
        bytes
            .chunks(1024)
            .enumerate()
            .map(|(index, payload)| {
                packet(
                    0,
                    0,
                    frame,
                    first_sequence.wrapping_add(index as u32),
                    index * 1024,
                    0,
                    payload,
                )
            })
            .collect()
    }

    // 64,000 single zero-group bits form a valid compact constant-gray frame.
    fn gray_frame() -> Vec<u8> {
        vec![0; WIDTH * HEIGHT / 16 / 8]
    }

    #[test]
    fn validates_header_length_alignment_and_reserved_bits() {
        let valid = packet(0, 0, 7, 9, 0, 0, &[0; 2]);
        assert_eq!(parse_datagram(&valid).unwrap().frame_id, 7);
        for (index, value) in [
            (0, 0),
            (5, 2),
            (6, 2),
            (7, 31),
            (19, 1),
            (23, 4),
            (27, 1),
            (31, 1),
        ] {
            let mut bad = valid.clone();
            bad[index] = value;
            assert!(parse_datagram(&bad).is_err(), "header byte {index}");
        }
        assert!(parse_datagram(&valid[..31]).is_err());
        assert!(parse_datagram(&vec![0; MAX_DATAGRAM_SIZE + 1]).is_err());
        assert!(parse_datagram(&packet(0, 0, 0, 0, MAX_IMAGE_SIZE, 0, &[0; 2])).is_err());
        assert!(parse_datagram(&packet(0, 0, 0, 0, 0, 0, &[])).is_err());
    }

    #[test]
    fn compact_frames_decode() {
        let mut assembler = StreamAssembler::new(0, 3);
        for bytes in image_packets(7, 0, &gray_frame()) {
            assembler.push(&bytes).unwrap();
        }
        let image = assembler.poll().pop().unwrap();
        assert_eq!(image.decode().unwrap().pixels, vec![128; WIDTH * HEIGHT]);
    }

    #[test]
    fn out_of_order_wrap_duplicate_and_next_frame_completion() {
        let bytes = gray_frame();
        let packets = image_packets(u32::MAX, u32::MAX - 3, &bytes);
        let mut assembler = StreamAssembler::new(0, 3);
        for index in [7, 2, 5, 0, 6, 1, 4, 3] {
            assert!(assembler.push(&packets[index]).unwrap().is_empty());
        }
        assert!(assembler.push(&packets[2]).unwrap().is_empty());
        let candidates = assembler.push(&packet(0, 0, 0, 4, 0, 0, &[0; 2])).unwrap();
        assert_eq!(candidates.len(), 1);
        assert_eq!(candidates[0].frame_id, u32::MAX);
        let decoded = candidates.into_iter().next().unwrap().decode().unwrap();
        assert_eq!(decoded.pixels, vec![128; WIDTH * HEIGHT]);
        assert_eq!(assembler.stats().duplicates, 1);
        assert!(!frame_is_newer(0x8000_0000, 0));
    }

    #[test]
    fn reordered_tail_after_boundary_amends_underflow_candidate() {
        let packets = image_packets(10, 0, &gray_frame());
        let mut assembler = StreamAssembler::new(0, 3);
        for datagram in &packets[..7] {
            assembler.push(datagram).unwrap();
        }
        let partial = assembler
            .push(&packet(0, 0, 11, 8, 0, 0, &[0; 2]))
            .unwrap()
            .pop()
            .unwrap();
        assert!(matches!(
            partial.decode(),
            Err(CodecError::BitstreamUnderflow { .. })
        ));
        let completed = assembler.push(&packets[7]).unwrap().pop().unwrap();
        assert_eq!(completed.frame_id, 10);
        assert!(completed.decode().is_ok());
        assert!(assembler.push(&packets[7]).unwrap().is_empty());
    }

    #[test]
    fn hole_waits_for_reordered_fragment_and_poll_finishes_last_frame() {
        let packets = image_packets(12, 100, &gray_frame());
        let mut assembler = StreamAssembler::new(0, 3);
        for (index, datagram) in packets.iter().enumerate() {
            if index != 3 {
                assembler.push(datagram).unwrap();
            }
        }
        assert!(assembler.poll().is_empty());
        assembler.push(&packets[3]).unwrap();
        let image = assembler.poll().pop().unwrap();
        assert!(image.decode().is_ok());
        assert!(assembler.poll().is_empty());
    }

    #[test]
    fn conflicts_invalidate_entire_frame_and_cannot_be_replayed() {
        for conflict in [
            packet(0, 0, 3, 2, 0, 0, &[1; 4]),
            packet(0, 0, 3, 3, 2, 0, &[0; 4]),
            packet(0, 0, 3, 2, 4, 0, &[0; 4]),
            packet(0, 0, 3, 8, 4, 0, &[0; 4]),
        ] {
            let mut assembler = StreamAssembler::new(0, 3);
            assembler.push(&packet(0, 0, 3, 2, 0, 0, &[0; 4])).unwrap();
            assert!(assembler.push(&conflict).is_err());
            assert!(
                assembler
                    .push(&packet(0, 0, 3, 3, 4, 0, &[0; 4]))
                    .unwrap()
                    .is_empty()
            );
            assert!(assembler.poll().is_empty());
            assert_eq!(assembler.stats().conflicting_frames, 1);
        }
    }

    #[test]
    fn old_frames_do_not_evict_new_frames_and_reset_accepts_restart() {
        let mut assembler = StreamAssembler::new(0, 3);
        for frame in [100, 102, 101, 103] {
            assembler
                .push(&packet(0, 0, frame, frame, 2, 0, &[0; 2]))
                .unwrap();
        }
        assert_eq!(assembler.frames.len(), 3);
        assert_eq!(assembler.stats().incomplete_evictions, 1);
        assembler.push(&packet(0, 0, 0, 0, 0, 0, &[0; 2])).unwrap();
        assert_eq!(assembler.newest, Some(103));
        assert_eq!(assembler.stats().stale_packets, 1);
        assembler.reset();
        assembler.push(&packet(0, 0, 0, 0, 0, 0, &[0; 2])).unwrap();
        assert_eq!(assembler.newest, Some(0));
        assert_eq!(assembler.stats().stale_packets, 1);
    }

    #[test]
    fn actual_codec_generated_texture_roundtrips_through_fragments() {
        let pixels: Vec<_> = (0..WIDTH * HEIGHT)
            .map(|index| ((index % WIDTH * 7) ^ (index / WIDTH * 13)) as u8)
            .collect();
        let fixed = Codec::encode(&pixels, WIDTH, HEIGHT).unwrap();
        let end = *Codec::packet_end_bits(&fixed).unwrap().last().unwrap();
        let compact = &fixed.bytes[..end.div_ceil(16) * 2];
        let packets = image_packets(0, 0, compact);
        let mut assembler = StreamAssembler::new(0, 3);
        for datagram in packets.iter().rev() {
            assembler.push(datagram).unwrap();
        }
        let decoded = assembler.poll().pop().unwrap().decode().unwrap();
        assert_eq!(decoded.pixels, Codec::decode(&fixed).unwrap());
    }

    #[test]
    fn single_frame_window_is_supported() {
        let mut compact = StreamAssembler::new(0, 1);
        for datagram in image_packets(2, 0, &gray_frame()) {
            assert!(compact.push(&datagram).unwrap().is_empty());
        }
        let ready = compact.push(&packet(0, 0, 3, 8, 0, 0, &[0; 2])).unwrap();
        assert_eq!(ready.len(), 1);
        assert!(ready.into_iter().next().unwrap().decode().is_ok());
        assert_eq!(compact.frames.len(), 1);
    }

    #[test]
    fn large_frame_jump_discards_stale_candidate_and_wrong_camera_is_rejected() {
        let mut assembler = StreamAssembler::new(0, 3);
        for datagram in image_packets(1, 0, &gray_frame()) {
            assembler.push(&datagram).unwrap();
        }
        assert!(
            assembler
                .push(&packet(0, 0, 100, 900, 0, 0, &[0; 2]))
                .unwrap()
                .is_empty()
        );
        assert_eq!(assembler.stats().incomplete_evictions, 1);
        assert!(
            assembler
                .push(&packet(0, 1, 101, 901, 0, 0, &[0; 2]))
                .is_err()
        );
        assert_eq!(assembler.newest, Some(100));
    }
}
