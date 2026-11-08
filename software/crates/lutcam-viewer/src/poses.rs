//! Bounded accepted-pose cache, joined to images only by the full outer frame ID.
//!
//! UDP ordering and duplicate delivery do not establish record order. A reused
//! packet sequence with different bytes invalidates the entire frame; its empty
//! tombstone tells consumers to clear previously displayed detections.

use std::collections::BTreeMap;
use std::fmt;
use std::sync::Arc;
use std::time::{Duration, Instant};

pub const MAX_POSE_FRAMES: usize = 64;
/// Maximum accepted records in one frame, independent of history retention.
pub const MAX_POSE_RECORDS: usize = 128;
pub const POSE_TTL: Duration = Duration::from_secs(2);
const HEADER_SIZE: usize = 32;
const RECORD_SIZE: usize = 88;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PoseError(pub &'static str);

impl fmt::Display for PoseError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.0)
    }
}

impl std::error::Error for PoseError {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TagDetection {
    pub sequence: u32,
    pub record_index: u8,
    pub tag_id: u32,
    pub common_words: [u32; 8],
    /// 0 = raw, 1 = LL2, 2 = LL4.
    pub scale: u8,
    pub frame_sequence: u8,
    pub hypothesis: u8,
    pub best_score_q32: u64,
    pub pose: lftag::PosePacket,
}

#[derive(Clone, Debug)]
pub struct FrameDetections {
    pub camera: u8,
    pub frame_id: u32,
    pub revision: u64,
    /// Time of the most recent unique addition or invalidation, not a duplicate.
    pub received_at: Instant,
    pub detections: Vec<TagDetection>,
    pub invalid: bool,
}

#[derive(Clone, Debug, Default)]
pub struct PoseStats {
    /// All datagrams offered to this pose parser, including malformed packets.
    pub packets: u64,
    /// Unique accepted records, including records subsequently evicted.
    pub detections: u64,
    pub invalid: u64,
    pub duplicates: u64,
    pub stale_packets: u64,
    pub conflicting_frames: u64,
    pub evicted_frames: u64,
}

struct Packet<'a> {
    version: u8,
    frame_id: u32,
    sequence: u32,
    payload: &'a [u8],
}

fn word(bytes: &[u8], index: usize) -> u32 {
    u32::from_be_bytes(bytes[index * 4..index * 4 + 4].try_into().unwrap())
}

fn parse_header(data: &[u8], camera: u8) -> Result<Packet<'_>, PoseError> {
    if !(HEADER_SIZE..=HEADER_SIZE + 2 * RECORD_SIZE).contains(&data.len()) {
        return Err(PoseError("truncated or oversized pose datagram"));
    }
    let [version, packet_camera, stream, header_size] = word(data, 1).to_be_bytes();
    if word(data, 0) != 0x4c55_5443 || version != 0 || header_size != HEADER_SIZE as u8 {
        return Err(PoseError("unsupported LUTCam pose header"));
    }
    if packet_camera > 1 || packet_camera != camera || stream != 3 {
        return Err(PoseError("wrong camera or stream for pose cache"));
    }
    if word(data, 4) != 0 || word(data, 6) != 0 || word(data, 7) != 0 {
        return Err(PoseError("nonzero pose offset or reserved header fields"));
    }
    if word(data, 5) as usize != data.len() - HEADER_SIZE {
        return Err(PoseError("pose payload length does not match datagram"));
    }
    Ok(Packet {
        version,
        frame_id: word(data, 2),
        sequence: word(data, 3),
        payload: &data[HEADER_SIZE..],
    })
}

fn parse_records(packet: &Packet<'_>) -> Result<Vec<TagDetection>, PoseError> {
    if packet.payload.len() != RECORD_SIZE && packet.payload.len() != 2 * RECORD_SIZE {
        return Err(PoseError(
            "pose payload must contain one or two complete records",
        ));
    }
    packet
        .payload
        .chunks_exact(RECORD_SIZE)
        .enumerate()
        .map(|(index, bytes)| {
            let words: [u32; 22] = core::array::from_fn(|index| word(bytes, index));
            let metadata = words[1];
            let scale = ((metadata >> 28) & 3) as u8;
            let frame_sequence = (metadata >> 20) as u8;
            // CameraPipeline patches the inner frame byte to the outer frame
            // counter. Reserved metadata fields do not contribute to geometry.
            if words[0] & !0x3fff != 0
                || metadata & 0x8000_0000 == 0
                || metadata & 0xffff != 0
                || scale > 2
                || frame_sequence != packet.frame_id as u8
                || words[6] != 0
                || words[7] != 0
            {
                return Err(PoseError("invalid accepted pose common header"));
            }
            let pose = lftag::PosePacket::from_packet(&words)
                .map_err(|_| PoseError("invalid accepted direct-pose packet"))?;
            Ok(TagDetection {
                sequence: packet.sequence,
                record_index: index as u8,
                tag_id: words[0],
                common_words: words[..8].try_into().unwrap(),
                scale,
                frame_sequence,
                hypothesis: ((metadata >> 16) & 15) as u8,
                best_score_q32: u64::from(words[2]) | (u64::from(words[3]) << 32),
                pose,
            })
        })
        .collect()
}

fn newer(candidate: u32, reference: u32) -> bool {
    let distance = candidate.wrapping_sub(reference);
    distance != 0 && distance < 0x8000_0000
}

struct Entry {
    version: u8,
    frame: Arc<FrameDetections>,
    // Exact datagram bytes establish duplicate identity, including the header.
    packets: BTreeMap<u32, Vec<u8>>,
}

/// At most 64 frame snapshots, each with at most 128 records and bounded packet
/// bytes. Dense frames need the same history window as sparse frames so their
/// detections survive until the matching image finishes decoding.
/// Invalid frame tombstones count toward the frame limit. An eviction watermark
/// prevents delayed packets from bringing back retired frame IDs. Call `reset`
/// after a known capture restart; counter wrap alone does not require a reset.
pub struct PoseCache {
    camera: u8,
    entries: Vec<Entry>,
    newest: Option<u32>,
    retired_through: Option<u32>,
    revision: u64,
    stats: PoseStats,
}

impl PoseCache {
    pub fn new(camera: u8) -> Self {
        assert!(camera <= 1, "camera must be 0 or 1");
        Self {
            camera,
            entries: Vec::new(),
            newest: None,
            retired_through: None,
            revision: 0,
            stats: PoseStats::default(),
        }
    }

    pub fn stats(&self) -> &PoseStats {
        &self.stats
    }

    /// Changes on additions, invalidations, evictions, and resets.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Snapshots in oldest-to-newest wrapping frame order. Only Arcs are cloned.
    pub fn frames(&self) -> Vec<Arc<FrameDetections>> {
        self.entries
            .iter()
            .map(|entry| entry.frame.clone())
            .collect()
    }

    /// Clear ordering and cached data while preserving session counters.
    pub fn reset(&mut self) {
        self.entries.clear();
        self.newest = None;
        self.retired_through = None;
        self.revision = self.revision.wrapping_add(1);
    }

    fn retire_first(&mut self) {
        let entry = self.entries.remove(0);
        self.retired_through = Some(entry.frame.frame_id);
        self.stats.evicted_frames += 1;
        self.revision = self.revision.wrapping_add(1);
    }

    /// Expire old snapshots. Expiring a newer frame also retires predecessors so
    /// an out-of-order arrival can never move the eviction watermark backwards.
    pub fn poll(&mut self, now: Instant) -> bool {
        let before = self.revision;
        if let Some(last_expired) = self
            .entries
            .iter()
            .rposition(|entry| now.saturating_duration_since(entry.frame.received_at) >= POSE_TTL)
        {
            for _ in 0..=last_expired {
                self.retire_first();
            }
        }
        before != self.revision
    }

    fn invalidate(&mut self, index: usize, now: Instant) {
        self.revision = self.revision.wrapping_add(1);
        let entry = &mut self.entries[index];
        let frame = Arc::make_mut(&mut entry.frame);
        frame.invalid = true;
        frame.detections.clear();
        frame.received_at = now;
        frame.revision = self.revision;
        entry.packets.clear();
        self.stats.conflicting_frames += 1;
    }

    fn enforce_bounds(&mut self) {
        while self.entries.len() > MAX_POSE_FRAMES {
            self.retire_first();
        }
    }

    /// Add a datagram atomically. `Ok(true)` means snapshots changed. Errors can
    /// also invalidate a frame; consumers must publish snapshots on an error or
    /// compare `revision()` to clear any previously displayed detections.
    pub fn push(&mut self, data: &[u8], now: Instant) -> Result<bool, PoseError> {
        let before = self.revision;
        self.poll(now);
        self.stats.packets += 1;
        let packet = parse_header(data, self.camera).inspect_err(|_| self.stats.invalid += 1)?;
        if self
            .retired_through
            .is_some_and(|retired| !newer(packet.frame_id, retired))
            || self
                .newest
                .is_some_and(|newest| packet.frame_id.wrapping_sub(newest) == 0x8000_0000)
        {
            self.stats.stale_packets += 1;
            return Ok(before != self.revision);
        }

        let index = if let Some(index) = self
            .entries
            .iter()
            .position(|entry| entry.frame.frame_id == packet.frame_id)
        {
            index
        } else {
            if self
                .newest
                .is_none_or(|newest| newer(packet.frame_id, newest))
            {
                self.newest = Some(packet.frame_id);
            }
            self.entries.push(Entry {
                version: packet.version,
                frame: Arc::new(FrameDetections {
                    camera: self.camera,
                    frame_id: packet.frame_id,
                    revision: self.revision,
                    received_at: now,
                    detections: Vec::new(),
                    invalid: false,
                }),
                packets: BTreeMap::new(),
            });
            let newest = self.newest.unwrap();
            self.entries
                .sort_by_key(|entry| std::cmp::Reverse(newest.wrapping_sub(entry.frame.frame_id)));
            self.entries
                .iter()
                .position(|entry| entry.frame.frame_id == packet.frame_id)
                .unwrap()
        };

        let entry = &self.entries[index];
        if entry.frame.invalid {
            self.stats.stale_packets += 1;
            return Ok(before != self.revision);
        }
        if entry.version != packet.version
            || entry
                .packets
                .get(&packet.sequence)
                .is_some_and(|previous| previous != data)
        {
            self.stats.invalid += 1;
            self.invalidate(index, now);
            self.enforce_bounds();
            return Err(PoseError("conflicting pose packet sequence or version"));
        }
        if entry.packets.contains_key(&packet.sequence) {
            self.stats.duplicates += 1;
            return Ok(before != self.revision);
        }
        let detections = match parse_records(&packet) {
            Ok(detections) => detections,
            Err(error) => {
                self.stats.invalid += 1;
                self.invalidate(index, now);
                self.enforce_bounds();
                return Err(error);
            }
        };
        if self.entries[index].frame.detections.len() + detections.len() > MAX_POSE_RECORDS {
            // Keep an explicit empty revision so consumers can retract a
            // previously displayed partial result for this oversized frame.
            self.stats.invalid += 1;
            self.invalidate(index, now);
            return Err(PoseError("pose frame exceeds record capacity"));
        }
        self.stats.detections += detections.len() as u64;
        self.revision = self.revision.wrapping_add(1);
        let entry = &mut self.entries[index];
        entry.packets.insert(packet.sequence, data.to_vec());
        let frame = Arc::make_mut(&mut entry.frame);
        frame.detections.extend(detections);
        frame
            .detections
            .sort_by_key(|record| (record.sequence, record.record_index));
        frame.received_at = now;
        frame.revision = self.revision;
        self.enforce_bounds();
        Ok(before != self.revision)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(frame: u32, tag_id: u32) -> [u32; 22] {
        [
            tag_id,
            0x8000_0000 | ((frame & 255) << 20),
            8,
            0,
            0x047b_ceae,
            0,
            0,
            0,
            0x0000_0003,
            1,
            0x1fff_0000,
            0xfffd_a000,
            0,
            0,
            0x1ffc_a000,
            0,
            0,
            0,
            0x1ffb_a000,
            0xffff_fffb,
            0xffff_fffb,
            0x8007,
        ]
    }

    fn datagram(
        version: u8,
        camera: u8,
        frame: u32,
        sequence: u32,
        records: &[[u32; 22]],
    ) -> Vec<u8> {
        let words = [
            0x4c55_5443,
            u32::from_be_bytes([version, camera, 3, 32]),
            frame,
            sequence,
            0,
            (records.len() * RECORD_SIZE) as u32,
            0,
            0,
        ];
        words
            .into_iter()
            .chain(records.iter().flatten().copied())
            .flat_map(u32::to_be_bytes)
            .collect()
    }

    fn packet(frame: u32, sequence: u32, ids: &[u32]) -> Vec<u8> {
        datagram(
            0,
            0,
            frame,
            sequence,
            &ids.iter().map(|id| record(frame, *id)).collect::<Vec<_>>(),
        )
    }

    #[test]
    fn actual_pose_words_and_two_record_udp_batch_decode() {
        let now = Instant::now();
        let mut first = record(257, 0x1234);
        first[1] |= 0x200b_0000; // LL4, hypothesis 11.
        first[3] = 2;
        first[5] = 3;
        let mut cache = PoseCache::new(1);
        assert!(
            cache
                .push(&datagram(0, 1, 257, 99, &[first, record(257, 7)]), now)
                .unwrap()
        );
        let frames = cache.frames();
        let frame = &frames[0];
        assert_eq!(
            (frame.camera, frame.frame_id, frame.detections.len()),
            (1, 257, 2)
        );
        let detection = &frame.detections[0];
        assert_eq!(
            (detection.sequence, detection.record_index, detection.tag_id),
            (99, 0, 0x1234)
        );
        assert_eq!(
            (
                detection.scale,
                detection.frame_sequence,
                detection.hypothesis
            ),
            (2, 1, 11)
        );
        assert_eq!(detection.best_score_q32, (2 << 32) | 8);
        assert_eq!(detection.pose.pose.translation_q16, [-5, -5, 0x8007]);
        assert_eq!(detection.pose.pose.rotation_q29[1], 0xfffd_a000u32 as i32);
        assert!(detection.pose.weak_perspective);
        assert_eq!(frame.detections[1].record_index, 1);
        assert_eq!((cache.stats().packets, cache.stats().detections), (1, 2));
    }

    #[test]
    fn reordered_packets_accumulate_only_on_their_exact_outer_frame() {
        let now = Instant::now();
        let mut cache = PoseCache::new(0);
        for (frame, sequence, ids) in [
            (258, 50, vec![1]),
            (257, 49, vec![2]),
            (258, 48, vec![3, 4]),
            (2, 47, vec![5]),
        ] {
            cache.push(&packet(frame, sequence, &ids), now).unwrap();
        }
        let frames = cache.frames();
        assert_eq!(
            frames
                .iter()
                .map(|frame| frame.frame_id)
                .collect::<Vec<_>>(),
            [2, 257, 258]
        );
        assert_eq!(
            frames[2]
                .detections
                .iter()
                .map(|tag| tag.tag_id)
                .collect::<Vec<_>>(),
            [3, 4, 1]
        );
        assert_eq!(frames[0].detections[0].tag_id, 5);
    }

    #[test]
    fn duplicates_are_idempotent_and_conflict_publishes_a_permanent_tombstone() {
        let now = Instant::now();
        let mut cache = PoseCache::new(0);
        let data = packet(4, 8, &[1]);
        cache.push(&data, now).unwrap();
        let original = cache.frames()[0].clone();
        assert!(!cache.push(&data, now + Duration::from_millis(100)).unwrap());
        assert!(Arc::ptr_eq(&original, &cache.frames()[0]));
        assert_eq!(cache.stats().duplicates, 1);
        assert!(cache.push(&packet(4, 8, &[2]), now).is_err());
        let invalid = cache.frames()[0].clone();
        assert!(invalid.invalid && invalid.detections.is_empty());
        assert!(invalid.revision > original.revision);
        assert_eq!(
            original.detections[0].tag_id, 1,
            "published snapshots remain immutable"
        );
        assert!(!cache.push(&packet(4, 9, &[3]), now).unwrap());
        assert!(Arc::ptr_eq(&invalid, &cache.frames()[0]));
        assert_eq!(
            (
                cache.stats().detections,
                cache.stats().invalid,
                cache.stats().conflicting_frames
            ),
            (1, 1, 1)
        );
    }

    #[test]
    fn malformed_second_record_does_not_publish_first_record() {
        let now = Instant::now();
        let mut cache = PoseCache::new(0);
        let mut bad = record(2, 2);
        bad[8] = 0;
        assert!(
            cache
                .push(&datagram(0, 0, 2, 0, &[record(2, 1), bad]), now)
                .is_err()
        );
        assert!(cache.frames()[0].invalid);
        assert!(cache.frames()[0].detections.is_empty());
        assert_eq!(cache.stats().detections, 0);
    }

    #[test]
    fn validates_all_common_metadata_and_direct_pose_status() {
        let now = Instant::now();
        for (index, value) in [
            (0, 0x4000),
            (1, 0),
            (1, 0xb070_0000),
            (1, 0x8070_0001),
            (1, 0x8080_0000),
            (6, 1),
            (7, 1),
            (8, 0x3000_0001),
            (8, 0x0000_0011),
        ] {
            let mut bad = record(7, 1);
            bad[index] = value;
            let mut cache = PoseCache::new(0);
            assert!(
                cache.push(&datagram(0, 0, 7, 0, &[bad]), now).is_err(),
                "word {index}: {value:#x}"
            );
            assert!(cache.frames()[0].invalid);
        }
    }

    #[test]
    fn rejects_invalid_transport_fields_without_poisoning_a_valid_frame() {
        let now = Instant::now();
        let base = packet(7, 0, &[1]);
        for (index, value) in [
            (0, 0),
            (1, u32::from_be_bytes([4, 0, 3, 32])),
            (1, u32::from_be_bytes([0, 1, 3, 32])),
            (1, u32::from_be_bytes([0, 0, 1, 32])),
            (1, u32::from_be_bytes([0, 0, 3, 31])),
            (4, 1),
            (5, 0),
            (6, 1),
            (7, 1),
        ] {
            let mut cache = PoseCache::new(0);
            cache.push(&base, now).unwrap();
            let mut bad = base.clone();
            bad[index * 4..index * 4 + 4].copy_from_slice(&value.to_be_bytes());
            assert!(cache.push(&bad, now).is_err());
            assert!(!cache.frames()[0].invalid);
        }
        let mut cache = PoseCache::new(0);
        for length in 0..32 {
            assert!(cache.push(&base[..length], now).is_err());
        }
        assert!(cache.push(&vec![0; 209], now).is_err());
        assert!(cache.frames().is_empty());
    }

    #[test]
    fn malformed_payload_invalidates_existing_frame() {
        let now = Instant::now();
        for conflict in [datagram(0, 0, 7, 1, &[])] {
            let mut cache = PoseCache::new(0);
            cache.push(&packet(7, 0, &[1]), now).unwrap();
            assert!(cache.push(&conflict, now).is_err());
            assert!(cache.frames()[0].invalid && cache.frames()[0].detections.is_empty());
        }
    }

    #[test]
    fn frame_counter_wrap_and_sequence_wrap_preserve_identity() {
        let now = Instant::now();
        let mut cache = PoseCache::new(0);
        for frame in [u32::MAX, 1, 0, u32::MAX - 1] {
            cache.push(&packet(frame, u32::MAX, &[1]), now).unwrap();
        }
        cache.push(&packet(0, 0, &[2]), now).unwrap();
        assert_eq!(
            cache
                .frames()
                .iter()
                .map(|frame| frame.frame_id)
                .collect::<Vec<_>>(),
            [u32::MAX - 1, u32::MAX, 0, 1]
        );
        assert_eq!(cache.frames()[2].detections.len(), 2);
        assert!(!cache.push(&packet(0x8000_0001, 1, &[3]), now).unwrap());
    }

    #[test]
    fn ttl_duplicates_and_late_packets_cannot_extend_or_revive_history() {
        let now = Instant::now();
        let mut cache = PoseCache::new(0);
        cache.push(&packet(9, 0, &[1]), now).unwrap();
        let later = now + Duration::from_secs(1);
        assert!(!cache.push(&packet(9, 0, &[1]), later).unwrap());
        cache.push(&packet(8, 1, &[2]), later).unwrap();
        assert!(cache.poll(now + POSE_TTL));
        assert!(
            cache.frames().is_empty(),
            "expired newer frame also retires older arrivals"
        );
        assert!(!cache.push(&packet(9, 2, &[3]), now + POSE_TTL).unwrap());
        assert!(!cache.push(&packet(7, 3, &[3]), now + POSE_TTL).unwrap());
        assert!(cache.push(&packet(10, 4, &[3]), now + POSE_TTL).unwrap());
        assert_eq!(cache.stats().stale_packets, 2);
    }

    #[test]
    fn frame_limit_evicts_oldest_even_for_late_arrivals() {
        let now = Instant::now();
        let mut cache = PoseCache::new(0);
        for frame in 10..74 {
            cache.push(&packet(frame, frame, &[1]), now).unwrap();
        }
        assert_eq!(cache.frames().len(), MAX_POSE_FRAMES);
        cache.push(&packet(9, 1, &[1]), now).unwrap();
        assert_eq!(cache.frames().first().unwrap().frame_id, 10);
        cache.push(&packet(74, 1, &[1]), now).unwrap();
        assert_eq!(cache.frames().first().unwrap().frame_id, 11);
        assert!(!cache.push(&packet(10, 2, &[2]), now).unwrap());
        assert_eq!(cache.stats().evicted_frames, 2);
    }

    #[test]
    fn dense_frames_keep_the_full_history_window_and_retire_only_the_oldest() {
        let now = Instant::now();
        let mut cache = PoseCache::new(0);
        for frame in 0..MAX_POSE_FRAMES as u32 {
            for sequence in 0..64 {
                cache.push(&packet(frame, sequence, &[1, 2]), now).unwrap();
            }
        }
        assert_eq!(cache.frames().len(), MAX_POSE_FRAMES);
        assert!(cache.entries.iter().all(|entry| {
            entry.frame.detections.len() == MAX_POSE_RECORDS && entry.packets.len() == 64
        }));
        assert_eq!(cache.stats().evicted_frames, 0);
        assert!(!cache.push(&packet(0, 0, &[1, 2]), now).unwrap());
        let original = cache.frames()[1].clone();
        assert!(cache.push(&packet(1, 0, &[3, 4]), now).is_err());
        assert!(cache.frames()[1].invalid);
        assert!(cache.frames()[1].detections.is_empty());
        assert_eq!(original.detections.len(), MAX_POSE_RECORDS);
        cache.push(&packet(64, 0, &[3]), now).unwrap();
        assert_eq!(cache.frames().len(), MAX_POSE_FRAMES);
        assert_eq!(cache.frames()[0].frame_id, 1);
        assert!(
            cache.frames()[0].invalid,
            "retained tombstone cannot revive"
        );
        assert!(!cache.push(&packet(0, 0, &[1, 2]), now).unwrap());
        assert_eq!(cache.stats().evicted_frames, 1);
    }

    #[test]
    fn invalid_tombstones_bound_packet_storage() {
        let now = Instant::now();
        let mut cache = PoseCache::new(0);
        for frame in 0..100 {
            cache.push(&packet(frame, 0, &[1]), now).unwrap();
            assert!(cache.push(&packet(frame, 0, &[2]), now).is_err());
        }
        assert_eq!(cache.frames().len(), MAX_POSE_FRAMES);
        assert!(cache.entries.iter().all(|entry| entry.packets.is_empty()));
        assert!(cache.frames().iter().all(|frame| frame.invalid));
    }

    #[test]
    fn oversized_single_frame_retains_an_empty_invalid_revision() {
        let now = Instant::now();
        let mut cache = PoseCache::new(0);
        for sequence in 0..64 {
            cache.push(&packet(7, sequence, &[1, 2]), now).unwrap();
        }
        let original = cache.frames()[0].clone();
        assert_eq!(original.detections.len(), MAX_POSE_RECORDS);
        assert!(!original.invalid);
        assert_eq!(
            cache.push(&packet(7, 64, &[3]), now),
            Err(PoseError("pose frame exceeds record capacity"))
        );
        let invalid = cache.frames()[0].clone();
        assert_eq!(invalid.frame_id, 7);
        assert!(invalid.invalid && invalid.detections.is_empty());
        assert!(invalid.revision > original.revision);
        assert_eq!(original.detections.len(), MAX_POSE_RECORDS);
        assert!(cache.entries[0].packets.is_empty());
        assert_eq!(cache.stats().detections, MAX_POSE_RECORDS as u64);
        assert_eq!(cache.stats().invalid, 1);
        assert_eq!(cache.stats().evicted_frames, 0);
        assert!(!cache.push(&packet(7, 65, &[4]), now).unwrap());
        assert!(Arc::ptr_eq(&invalid, &cache.frames()[0]));
    }

    #[test]
    fn reset_clears_tombstones_and_watermark_but_preserves_stats() {
        let now = Instant::now();
        let mut cache = PoseCache::new(0);
        cache.push(&packet(5, 0, &[1]), now).unwrap();
        assert!(cache.push(&packet(5, 0, &[2]), now).is_err());
        let revision = cache.revision();
        cache.reset();
        assert!(cache.frames().is_empty());
        assert!(cache.revision() > revision);
        cache.push(&packet(0, 0, &[1]), now).unwrap();
        cache.push(&packet(5, 0, &[1]), now).unwrap();
        assert_eq!(
            (
                cache.stats().packets,
                cache.stats().detections,
                cache.stats().invalid
            ),
            (4, 3, 1)
        );
    }
}
