//! Measured FPGA child centroids from LUTCam UDP product 2.
//!
//! Coordinates are unsigned Q5 values in the originating scaled ISP domain.
//! The wire record does not identify that domain or its scale, so dividing by
//! 32 does not establish a coordinate in the full camera RAW image. Preserve
//! tuple order and validity; these are measurements, not reconstructed tag
//! corners. Groups associate with images through the full outer `frame_id`.
//!
//! `parse_datagram` is stateless. `CentroidCache` accumulates reordered packets
//! and retracts a frame on conflicting or malformed data. Packets carry up to
//! four complete nine-centroid groups.

use std::collections::BTreeMap;
use std::fmt;
use std::sync::Arc;
use std::time::{Duration, Instant};

pub const MAX_CENTROID_FRAMES: usize = 64;
/// Maximum measured groups in one frame, independent of history retention.
pub const MAX_CENTROID_GROUPS: usize = 256;
pub const CENTROID_TTL: Duration = Duration::from_secs(2);

const HEADER_SIZE: usize = 32;
const CENTROID_SIZE: usize = 8;
const GROUP_SIZE: usize = 9 * CENTROID_SIZE;
const MAX_PAYLOAD_SIZE: usize = 4 * GROUP_SIZE;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CentroidError(pub &'static str);

impl fmt::Display for CentroidError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.0)
    }
}

impl std::error::Error for CentroidError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq, serde::Serialize)]
pub struct Centroid {
    pub valid: bool,
    /// Raw unsigned fixed-point coordinate, with five fractional bits.
    /// Retained verbatim even when `valid` is false.
    pub x_q5: u16,
    /// Raw unsigned fixed-point coordinate, with five fractional bits.
    /// Retained verbatim even when `valid` is false.
    pub y_q5: u16,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CentroidGroup {
    /// Outer packet sequence; together with `group_index`, identifies a group
    /// within the full outer frame ID.
    pub sequence: u32,
    /// Zero-based group position within this packet, not within the frame.
    pub group_index: u8,
    /// Original tuple order, including invalid tuples.
    pub centroids: Vec<Centroid>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CentroidPacket {
    pub camera: u8,
    pub frame_id: u32,
    pub sequence: u32,
    /// Reserved header byte, always zero.
    pub version: u8,
    pub groups: Vec<CentroidGroup>,
}

/// Validate and decode one product-2 datagram without assigning scale or
/// synthesizing coordinates. All packet and tuple reserved bits must be zero.
pub fn parse_datagram(data: &[u8]) -> Result<CentroidPacket, CentroidError> {
    if !(HEADER_SIZE..=HEADER_SIZE + MAX_PAYLOAD_SIZE).contains(&data.len()) {
        return Err(CentroidError("truncated or oversized centroid datagram"));
    }
    let word =
        |index: usize| u32::from_be_bytes(data[index * 4..index * 4 + 4].try_into().unwrap());
    let [version, camera, stream, header_size] = word(1).to_be_bytes();
    if word(0) != 0x4c55_5443 || version != 0 || header_size != HEADER_SIZE as u8 {
        return Err(CentroidError("unsupported LUTCam centroid header"));
    }
    if camera > 1 || stream != 2 {
        return Err(CentroidError("not a supported camera centroid product"));
    }
    if word(4) != 0 || word(6) != 0 || word(7) != 0 {
        return Err(CentroidError(
            "nonzero centroid offset or reserved header fields",
        ));
    }
    let length = word(5) as usize;
    if length != data.len() - HEADER_SIZE {
        return Err(CentroidError(
            "centroid payload length does not match datagram",
        ));
    }
    if !(GROUP_SIZE..=MAX_PAYLOAD_SIZE).contains(&length) || !length.is_multiple_of(GROUP_SIZE) {
        return Err(CentroidError(
            "centroid batch must contain one through four complete nine-tuple groups",
        ));
    }
    let group_size = GROUP_SIZE;

    let mut groups = Vec::with_capacity(length / group_size);
    for (index, group) in data[HEADER_SIZE..].chunks_exact(group_size).enumerate() {
        let mut centroids = Vec::with_capacity(group_size / CENTROID_SIZE);
        for encoded in group.chunks_exact(CENTROID_SIZE) {
            let tuple = u64::from_be_bytes(encoded.try_into().unwrap());
            if tuple >> 33 != 0 {
                return Err(CentroidError(
                    "centroid tuple is not a zero-extended 33-bit value",
                ));
            }
            centroids.push(Centroid {
                valid: tuple & 1 != 0,
                x_q5: (tuple >> 1) as u16,
                y_q5: (tuple >> 17) as u16,
            });
        }
        groups.push(CentroidGroup {
            sequence: word(3),
            group_index: index as u8,
            centroids,
        });
    }

    Ok(CentroidPacket {
        camera,
        frame_id: word(2),
        sequence: word(3),
        version,
        groups,
    })
}

#[derive(Clone, Debug)]
pub struct FrameCentroids {
    pub camera: u8,
    pub frame_id: u32,
    pub revision: u64,
    /// Time of the most recent unique addition or invalidation, not a duplicate.
    pub received_at: Instant,
    pub groups: Vec<CentroidGroup>,
    /// An empty tombstone that retracts previously published measurements.
    pub invalid: bool,
}

#[derive(Clone, Debug, Default)]
pub struct CentroidStats {
    /// All datagrams offered to this cache, including malformed packets.
    pub packets: u64,
    /// Unique accepted groups, including groups subsequently invalidated/evicted.
    pub groups: u64,
    pub invalid: u64,
    pub duplicates: u64,
    pub stale_packets: u64,
    pub conflicting_frames: u64,
    pub evicted_frames: u64,
}

fn newer(candidate: u32, reference: u32) -> bool {
    let distance = candidate.wrapping_sub(reference);
    distance != 0 && distance < 0x8000_0000
}

struct Entry {
    version: u8,
    frame: Arc<FrameCentroids>,
    // Exact datagrams establish duplicate identity, including all header bits.
    packets: BTreeMap<u32, Vec<u8>>,
    newest_sequence: Option<u32>,
}

/// At most 64 frame snapshots, each with at most 256 groups and bounded packet
/// bytes. Dense frames need the same history window as sparse frames so their
/// measurements survive until the matching image finishes decoding.
/// Invalid tombstones count toward the frame limit. An eviction watermark
/// prevents delayed data from reviving retired IDs. Call `reset` after a known
/// capture restart; wrapping frame and packet counters do not require a reset.
pub struct CentroidCache {
    camera: u8,
    entries: Vec<Entry>,
    newest: Option<u32>,
    retired_through: Option<u32>,
    revision: u64,
    stats: CentroidStats,
}

impl CentroidCache {
    pub fn new(camera: u8) -> Self {
        assert!(camera <= 1, "camera must be 0 or 1");
        Self {
            camera,
            entries: Vec::new(),
            newest: None,
            retired_through: None,
            revision: 0,
            stats: CentroidStats::default(),
        }
    }

    pub fn stats(&self) -> &CentroidStats {
        &self.stats
    }

    /// Changes on additions, invalidations, evictions, and resets.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Snapshots in oldest-to-newest wrapping frame order. Only Arcs are cloned.
    pub fn frames(&self) -> Vec<Arc<FrameCentroids>> {
        self.entries
            .iter()
            .map(|entry| entry.frame.clone())
            .collect()
    }

    /// Clear cached data and ordering while preserving session counters.
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

    /// Expiring a newer frame also retires its predecessors, preventing the
    /// eviction watermark from moving backwards after reordered arrivals.
    pub fn poll(&mut self, now: Instant) -> bool {
        let before = self.revision;
        if let Some(last_expired) = self.entries.iter().rposition(|entry| {
            now.saturating_duration_since(entry.frame.received_at) >= CENTROID_TTL
        }) {
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
        frame.groups.clear();
        frame.received_at = now;
        frame.revision = self.revision;
        entry.packets.clear();
        entry.newest_sequence = None;
        self.stats.conflicting_frames += 1;
    }

    fn enforce_bounds(&mut self) {
        while self.entries.len() > MAX_CENTROID_FRAMES {
            self.retire_first();
        }
    }

    /// Find or create a live frame. Stale and half-range ambiguous frame IDs
    /// cannot introduce a new ordering generation or revive retired snapshots.
    fn frame_index(&mut self, version: u8, frame_id: u32, now: Instant) -> Option<usize> {
        if self
            .retired_through
            .is_some_and(|retired| !newer(frame_id, retired))
            || self
                .newest
                .is_some_and(|newest| frame_id.wrapping_sub(newest) == 0x8000_0000)
        {
            self.stats.stale_packets += 1;
            return None;
        }
        let prospective_newest = self
            .newest
            .filter(|newest| !newer(frame_id, *newest))
            .unwrap_or(frame_id);
        if self
            .entries
            .iter()
            .any(|entry| prospective_newest.wrapping_sub(entry.frame.frame_id) >= 0x8000_0000)
        {
            // Pairwise serial comparisons cease to be transitive once the
            // live window spans half the counter range. Do not move newest or
            // the retirement watermark into that ambiguous generation.
            self.stats.stale_packets += 1;
            return None;
        }
        if let Some(index) = self
            .entries
            .iter()
            .position(|entry| entry.frame.frame_id == frame_id)
        {
            return Some(index);
        }
        if self.newest.is_none_or(|newest| newer(frame_id, newest)) {
            self.newest = Some(frame_id);
        }
        self.entries.push(Entry {
            version,
            frame: Arc::new(FrameCentroids {
                camera: self.camera,
                frame_id,
                revision: self.revision,
                received_at: now,
                groups: Vec::new(),
                invalid: false,
            }),
            packets: BTreeMap::new(),
            newest_sequence: None,
        });
        let newest = self.newest.unwrap();
        self.entries
            .sort_by_key(|entry| std::cmp::Reverse(newest.wrapping_sub(entry.frame.frame_id)));
        self.entries
            .iter()
            .position(|entry| entry.frame.frame_id == frame_id)
    }

    fn invalidate_malformed(&mut self, data: &[u8], now: Instant) {
        // A complete recognizable outer header establishes the frame even if
        // its version, reserved fields, declared length or payload are invalid.
        // Truncated/unrecognizable headers and other cameras/products cannot
        // poison this camera's cached measurements.
        if data.len() < HEADER_SIZE
            || data[..4] != 0x4c55_5443_u32.to_be_bytes()
            || data[5] != self.camera
            || data[6] != 2
            || data[7] != HEADER_SIZE as u8
        {
            return;
        }
        let frame_id = u32::from_be_bytes(data[8..12].try_into().unwrap());
        if let Some(index) = self.frame_index(data[4], frame_id, now) {
            if self.entries[index].frame.invalid {
                self.stats.stale_packets += 1;
            } else {
                self.invalidate(index, now);
            }
            self.enforce_bounds();
        }
    }

    /// Add a datagram atomically. `Ok(true)` means snapshots changed. Errors can
    /// also invalidate a frame; publish snapshots on errors or compare
    /// `revision()` to retract measurements already shown by a consumer.
    pub fn push(&mut self, data: &[u8], now: Instant) -> Result<bool, CentroidError> {
        let before = self.revision;
        self.poll(now);
        self.stats.packets += 1;
        let packet = match parse_datagram(data) {
            Ok(packet) if packet.camera == self.camera => packet,
            Ok(_) => {
                self.stats.invalid += 1;
                return Err(CentroidError("wrong camera for centroid cache"));
            }
            Err(error) => {
                self.stats.invalid += 1;
                self.invalidate_malformed(data, now);
                return Err(error);
            }
        };
        let Some(index) = self.frame_index(packet.version, packet.frame_id, now) else {
            return Ok(before != self.revision);
        };
        let entry = &self.entries[index];
        if entry.frame.invalid {
            self.stats.stale_packets += 1;
            return Ok(before != self.revision);
        }
        let prospective_sequence = entry
            .newest_sequence
            .filter(|newest| !newer(packet.sequence, *newest))
            .unwrap_or(packet.sequence);
        if entry.version != packet.version
            || entry
                .packets
                .get(&packet.sequence)
                .is_some_and(|previous| previous != data)
            || entry
                .newest_sequence
                .is_some_and(|newest| packet.sequence.wrapping_sub(newest) == 0x8000_0000)
            || entry
                .packets
                .keys()
                .any(|sequence| prospective_sequence.wrapping_sub(*sequence) >= 0x8000_0000)
        {
            self.stats.invalid += 1;
            self.invalidate(index, now);
            self.enforce_bounds();
            return Err(CentroidError(
                "conflicting centroid packet sequence or version",
            ));
        }
        if entry.packets.contains_key(&packet.sequence) {
            self.stats.duplicates += 1;
            return Ok(before != self.revision);
        }
        if entry.frame.groups.len() + packet.groups.len() > MAX_CENTROID_GROUPS {
            // Keep an empty revision rather than silently evicting this frame:
            // consumers must retract any partial measurements already shown.
            self.stats.invalid += 1;
            self.invalidate(index, now);
            return Err(CentroidError("centroid frame exceeds group capacity"));
        }

        self.stats.groups += packet.groups.len() as u64;
        self.revision = self.revision.wrapping_add(1);
        let entry = &mut self.entries[index];
        entry.packets.insert(packet.sequence, data.to_vec());
        if entry
            .newest_sequence
            .is_none_or(|newest| newer(packet.sequence, newest))
        {
            entry.newest_sequence = Some(packet.sequence);
        }
        let newest_sequence = entry.newest_sequence.unwrap();
        let frame = Arc::make_mut(&mut entry.frame);
        frame.groups.extend(packet.groups);
        frame.groups.sort_by_key(|group| {
            (
                std::cmp::Reverse(newest_sequence.wrapping_sub(group.sequence)),
                group.group_index,
            )
        });
        frame.received_at = now;
        frame.revision = self.revision;
        self.enforce_bounds();
        Ok(before != self.revision)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(version: u8, camera: u8, frame: u32, sequence: u32, payload: &[u8]) -> Vec<u8> {
        let mut bytes = Vec::new();
        for word in [
            0x4c55_5443,
            u32::from_be_bytes([version, camera, 2, 32]),
            frame,
            sequence,
            0,
            payload.len() as u32,
            0,
            0,
        ] {
            bytes.extend_from_slice(&word.to_be_bytes());
        }
        bytes.extend_from_slice(payload);
        bytes
    }

    fn tuple(valid: bool, x_q5: u16, y_q5: u16) -> [u8; 8] {
        ((u64::from(y_q5) << 17) | (u64::from(x_q5) << 1) | u64::from(valid)).to_be_bytes()
    }

    #[test]
    fn preserves_measured_coordinates_validity_order_and_group_boundaries() {
        let expected: Vec<Centroid> = (0..36)
            .map(|index| Centroid {
                valid: index % 3 == 0,
                x_q5: index * 41 + 7,
                y_q5: u16::MAX - index * 31,
            })
            .collect();
        let payload: Vec<u8> = expected
            .iter()
            .flat_map(|value| tuple(value.valid, value.x_q5, value.y_q5))
            .collect();
        for camera in 0..=1 {
            for count in 1..=4 {
                let bytes = packet(0, camera, 0xfedc_ba98, 0x9876_5432, &payload[..count * 72]);
                let parsed = parse_datagram(&bytes).unwrap();
                assert_eq!(parsed.camera, camera);
                assert_eq!(parsed.version, 0);
                assert_eq!(parsed.frame_id, 0xfedc_ba98);
                assert_eq!(parsed.sequence, 0x9876_5432);
                assert_eq!(parsed.groups.len(), count);
                for (index, group) in parsed.groups.iter().enumerate() {
                    assert_eq!(group.sequence, 0x9876_5432);
                    assert_eq!(group.group_index, index as u8);
                    assert_eq!(group.centroids, expected[index * 9..index * 9 + 9]);
                }
            }
        }
    }

    #[test]
    fn decodes_known_wire_bits_including_coordinate_extremes() {
        let expected = [
            Centroid {
                valid: true,
                x_q5: u16::MAX,
                y_q5: u16::MAX,
            },
            Centroid {
                valid: false,
                x_q5: 0,
                y_q5: 0,
            },
            Centroid {
                valid: true,
                x_q5: 0,
                y_q5: 0,
            },
            Centroid {
                valid: false,
                x_q5: 1,
                y_q5: 1,
            },
        ];
        let mut payload: Vec<_> = expected
            .iter()
            .flat_map(|value| tuple(value.valid, value.x_q5, value.y_q5))
            .collect();
        payload.resize(72, 0);
        let bytes = packet(0, 0, 7, 2, &payload);
        assert_eq!(
            &parse_datagram(&bytes).unwrap().groups[0].centroids[..4],
            &expected
        );
    }

    #[test]
    fn validates_header_fields_and_all_reserved_bits() {
        let valid = packet(0, 1, 0, 0, &[0; 72]);
        for bit in 0..32 {
            let mut bad_magic = valid.clone();
            bad_magic[bit / 8] ^= 1 << (bit % 8);
            assert!(parse_datagram(&bad_magic).is_err());
        }
        for (index, values) in [
            (5, vec![2, 3, 255]),
            (6, vec![0, 1, 3, 255]),
            (7, vec![0, 31, 33, 255]),
        ] {
            for value in values {
                let mut bad = valid.clone();
                bad[index] = value;
                assert!(parse_datagram(&bad).is_err(), "byte {index}, value {value}");
            }
        }
        for version in [0] {
            let valid = packet(version, 0, 0, 0, &[0; 72]);
            for word in [4, 6, 7] {
                for bit in 0..32 {
                    let mut bad = valid.clone();
                    bad[word * 4..word * 4 + 4].copy_from_slice(&(1_u32 << bit).to_be_bytes());
                    assert!(
                        parse_datagram(&bad).is_err(),
                        "version {version}, word {word}, bit {bit}"
                    );
                }
            }
        }
    }

    #[test]
    fn rejects_truncation_extra_bytes_and_declared_length_mismatches() {
        let valid = packet(0, 0, 0, 0, &[0; 288]);
        for end in 0..valid.len() {
            assert!(parse_datagram(&valid[..end]).is_err(), "truncated at {end}");
        }
        for declared in [0, 8, 72, 287, 289, u32::MAX] {
            let mut bad = valid.clone();
            bad[20..24].copy_from_slice(&declared.to_be_bytes());
            assert!(parse_datagram(&bad).is_err());
        }
        let mut extra = packet(0, 0, 0, 0, &[0; 72]);
        extra.push(0);
        assert!(parse_datagram(&extra).is_err());
    }

    #[test]
    fn rejects_any_high_tuple_bit_even_in_last_group_or_invalid_centroid() {
        for version in [0] {
            let count = 36;
            for index in 0..count {
                for bit in 33..64 {
                    let mut payload = vec![0; count * 8];
                    payload[index * 8..index * 8 + 8]
                        .copy_from_slice(&(1_u64 << bit).to_be_bytes());
                    assert!(parse_datagram(&packet(version, 0, 0, 0, &payload)).is_err());
                }
            }
        }
    }

    #[test]
    fn parsing_is_independent_of_order_duplicates_and_sequence_wrap() {
        let newer = packet(0, 0, 0, 0, &[0; 72]);
        let older = packet(0, 0, u32::MAX, u32::MAX, &[0; 144]);
        let first = parse_datagram(&newer).unwrap();
        let previous = parse_datagram(&older).unwrap();
        assert_eq!(previous.frame_id, u32::MAX);
        assert_eq!(previous.sequence, u32::MAX);
        assert_eq!(previous.groups.len(), 2);
        assert_eq!(parse_datagram(&newer).unwrap(), first);
        assert_eq!(parse_datagram(&older).unwrap(), previous);
    }

    fn cache_packet(frame: u32, sequence: u32, count: usize) -> Vec<u8> {
        packet(0, 0, frame, sequence, &vec![0; count * GROUP_SIZE])
    }

    #[test]
    fn cache_accumulates_reordered_groups_by_full_frame_id_with_immutable_snapshots() {
        let now = Instant::now();
        let mut cache = CentroidCache::new(0);
        assert!(cache.push(&cache_packet(257, 4, 2), now).unwrap());
        let original = cache.frames()[0].clone();
        assert!(cache.push(&cache_packet(258, 9, 1), now).unwrap());
        assert!(cache.push(&cache_packet(1, 1, 1), now).unwrap());
        assert!(cache.push(&cache_packet(257, 3, 1), now).unwrap());
        let frames = cache.frames();
        assert_eq!(
            frames
                .iter()
                .map(|frame| frame.frame_id)
                .collect::<Vec<_>>(),
            [1, 257, 258]
        );
        assert_eq!(
            frames[1]
                .groups
                .iter()
                .map(|group| (group.sequence, group.group_index))
                .collect::<Vec<_>>(),
            [(3, 0), (4, 0), (4, 1)]
        );
        assert_eq!(original.groups.len(), 2);
        assert_eq!(frames[1].groups.len(), 3);
        assert!(frames[1].revision > original.revision);
        assert_eq!(cache.stats().packets, 4);
        assert_eq!(cache.stats().groups, 5);
        assert_eq!(cache.stats().invalid, 0);
    }

    #[test]
    fn exact_duplicates_do_not_refresh_ttl_or_revision_and_expired_frames_stay_retired() {
        let now = Instant::now();
        let mut cache = CentroidCache::new(0);
        let packet = cache_packet(40, 7, 4);
        cache.push(&packet, now).unwrap();
        let revision = cache.revision();
        assert!(
            !cache
                .push(&packet, now + Duration::from_millis(1900))
                .unwrap()
        );
        assert_eq!(cache.revision(), revision);
        assert_eq!(cache.frames()[0].received_at, now);
        assert_eq!(cache.stats().groups, 4);
        assert_eq!(cache.stats().duplicates, 1);
        assert!(!cache.poll(now + CENTROID_TTL - Duration::from_nanos(1)));
        assert!(cache.poll(now + CENTROID_TTL));
        assert!(cache.frames().is_empty());
        assert!(!cache.push(&packet, now + CENTROID_TTL).unwrap());
        assert!(
            !cache
                .push(&cache_packet(39, 8, 1), now + CENTROID_TTL)
                .unwrap()
        );
        assert_eq!(cache.stats().stale_packets, 2);
        assert_eq!(cache.stats().evicted_frames, 1);
        assert!(
            cache
                .push(&cache_packet(41, 8, 1), now + CENTROID_TTL)
                .unwrap()
        );
    }

    #[test]
    fn conflicting_duplicate_retracts_frame_once_and_reset_preserves_counters() {
        let now = Instant::now();
        let mut cache = CentroidCache::new(0);
        let valid = cache_packet(12, 2, 1);
        cache.push(&valid, now).unwrap();
        let original = cache.frames()[0].clone();
        let mut conflicting = valid.clone();
        conflicting[HEADER_SIZE + 7] = 1;
        assert!(
            cache
                .push(&conflicting, now + Duration::from_millis(1))
                .is_err()
        );
        let tombstone = cache.frames()[0].clone();
        assert!(tombstone.invalid);
        assert!(tombstone.groups.is_empty());
        assert!(tombstone.revision > original.revision);
        assert!(!original.invalid);
        assert_eq!(original.groups.len(), 1);
        let revision = cache.revision();
        assert!(!cache.push(&valid, now + Duration::from_millis(2)).unwrap());
        assert!(
            !cache
                .push(&conflicting, now + Duration::from_millis(3))
                .unwrap()
        );
        assert_eq!(cache.revision(), revision);
        assert_eq!(cache.frames()[0].received_at, tombstone.received_at);
        assert_eq!(cache.stats().conflicting_frames, 1);
        assert_eq!(cache.stats().invalid, 1);
        assert_eq!(cache.stats().stale_packets, 2);
        cache.reset();
        assert!(cache.frames().is_empty());
        assert!(cache.revision() > revision);
        assert_eq!(cache.stats().packets, 4);
        assert_eq!(cache.stats().groups, 1);
        assert!(cache.push(&valid, now).unwrap());
        assert_eq!(cache.stats().groups, 2);
    }

    #[test]
    fn malformed_recognizable_packets_retract_existing_and_unseen_frames() {
        let now = Instant::now();
        let valid = cache_packet(9, 1, 1);
        let mut malformed = Vec::new();
        malformed.push(packet(0, 0, 9, 2, &[0; 71]));
        for (index, value) in [(16, 1), (20, 1), (24, 1), (28, 1), (32, 1)] {
            let mut bytes = valid.clone();
            bytes[index] = value;
            malformed.push(bytes);
        }
        malformed.push(valid[..HEADER_SIZE].to_vec());
        malformed.push(packet(0, 0, 9, 2, &[0; 289]));
        for bytes in malformed {
            for existing in [false, true] {
                let mut cache = CentroidCache::new(0);
                if existing {
                    cache.push(&valid, now).unwrap();
                }
                let before = cache.revision();
                assert!(cache.push(&bytes, now).is_err());
                assert!(cache.revision() > before);
                assert_eq!(cache.frames().len(), 1);
                assert_eq!(cache.frames()[0].frame_id, 9);
                assert!(cache.frames()[0].invalid);
                assert!(cache.frames()[0].groups.is_empty());
                assert!(!cache.push(&valid, now).unwrap());
                let tombstone_revision = cache.revision();
                assert!(cache.push(&bytes, now + Duration::from_millis(1)).is_err());
                assert_eq!(cache.revision(), tombstone_revision);
                assert_eq!(cache.stats().conflicting_frames, 1);
            }
        }
    }

    #[test]
    fn unrecognizable_or_wrong_source_headers_cannot_invalidate_a_frame() {
        let now = Instant::now();
        let valid = cache_packet(9, 1, 1);
        let mut invalid = Vec::new();
        for (index, value) in [(0, 0), (5, 1), (5, 255), (6, 1), (6, 3), (7, 31)] {
            let mut bytes = valid.clone();
            bytes[index] = value;
            invalid.push(bytes);
        }
        invalid.push(valid[..HEADER_SIZE - 1].to_vec());
        let mut cache = CentroidCache::new(0);
        cache.push(&valid, now).unwrap();
        let revision = cache.revision();
        for bytes in invalid {
            assert!(cache.push(&bytes, now).is_err());
            assert_eq!(cache.revision(), revision);
            assert_eq!(cache.frames().len(), 1);
            assert!(!cache.frames()[0].invalid);
            assert_eq!(cache.frames()[0].groups.len(), 1);
        }
        assert_eq!(cache.stats().invalid, 7);
        assert_eq!(cache.stats().conflicting_frames, 0);
    }

    #[test]
    fn overflowing_one_frame_keeps_an_empty_tombstone_instead_of_evicting_it() {
        let now = Instant::now();
        let mut cache = CentroidCache::new(0);
        for sequence in 0..64 {
            cache.push(&cache_packet(5, sequence, 4), now).unwrap();
        }
        let full = cache.frames()[0].clone();
        assert_eq!(full.groups.len(), MAX_CENTROID_GROUPS);
        assert!(cache.push(&cache_packet(5, 64, 1), now).is_err());
        assert_eq!(cache.frames().len(), 1);
        assert!(cache.frames()[0].invalid);
        assert!(cache.frames()[0].groups.is_empty());
        assert!(cache.frames()[0].revision > full.revision);
        assert_eq!(full.groups.len(), MAX_CENTROID_GROUPS);
        assert_eq!(cache.stats().groups, MAX_CENTROID_GROUPS as u64);
        assert_eq!(cache.stats().evicted_frames, 0);
        assert_eq!(cache.stats().conflicting_frames, 1);
        assert!(!cache.push(&cache_packet(5, 65, 1), now).unwrap());
        assert!(cache.poll(now + CENTROID_TTL));
        assert!(
            !cache
                .push(&cache_packet(5, 66, 1), now + CENTROID_TTL)
                .unwrap()
        );
        assert!(cache.frames().is_empty());
    }

    #[test]
    fn total_groups_do_not_shorten_the_frame_history_or_reject_late_additions() {
        let now = Instant::now();
        let mut cache = CentroidCache::new(0);
        for sequence in 0..50 {
            cache.push(&cache_packet(100, sequence, 4), now).unwrap();
        }
        for sequence in 0..14 {
            cache.push(&cache_packet(101, sequence, 4), now).unwrap();
        }
        assert_eq!(
            cache
                .frames()
                .iter()
                .map(|frame| frame.groups.len())
                .sum::<usize>(),
            256
        );
        cache.push(&cache_packet(102, 0, 4), now).unwrap();
        assert_eq!(
            cache
                .frames()
                .iter()
                .map(|frame| frame.frame_id)
                .collect::<Vec<_>>(),
            [100, 101, 102]
        );
        assert_eq!(
            cache
                .frames()
                .iter()
                .map(|frame| frame.groups.len())
                .sum::<usize>(),
            260
        );
        assert_eq!(cache.stats().evicted_frames, 0);
        assert!(cache.push(&cache_packet(100, 50, 1), now).unwrap());
        assert_eq!(cache.frames()[0].groups.len(), 201);
        assert_eq!(cache.stats().stale_packets, 0);
    }

    #[test]
    fn dense_frames_keep_the_full_history_window_and_retire_only_the_oldest() {
        let now = Instant::now();
        let mut cache = CentroidCache::new(0);
        for frame in 0..MAX_CENTROID_FRAMES as u32 {
            for sequence in 0..64 {
                cache.push(&cache_packet(frame, sequence, 4), now).unwrap();
            }
        }
        assert_eq!(cache.frames().len(), MAX_CENTROID_FRAMES);
        assert!(cache.entries.iter().all(|entry| {
            entry.frame.groups.len() == MAX_CENTROID_GROUPS && entry.packets.len() == 64
        }));
        assert_eq!(cache.stats().evicted_frames, 0);
        assert!(!cache.push(&cache_packet(0, 0, 4), now).unwrap());
        let original = cache.frames()[1].clone();
        assert!(cache.push(&cache_packet(1, 0, 3), now).is_err());
        assert!(cache.frames()[1].invalid);
        assert!(cache.frames()[1].groups.is_empty());
        assert_eq!(original.groups.len(), MAX_CENTROID_GROUPS);
        cache.push(&cache_packet(64, 0, 1), now).unwrap();
        assert_eq!(cache.frames().len(), MAX_CENTROID_FRAMES);
        assert_eq!(cache.frames()[0].frame_id, 1);
        assert!(
            cache.frames()[0].invalid,
            "retained tombstone cannot revive"
        );
        assert!(!cache.push(&cache_packet(0, 0, 4), now).unwrap());
        assert_eq!(cache.stats().evicted_frames, 1);
    }

    #[test]
    fn frame_limit_includes_tombstones_and_keeps_the_eviction_watermark() {
        let now = Instant::now();
        let mut cache = CentroidCache::new(0);
        for frame in 0..65 {
            if frame % 2 == 0 {
                assert!(cache.push(&packet(0, 0, frame, 0, &[]), now).is_err());
            } else {
                cache.push(&cache_packet(frame, 0, 1), now).unwrap();
            }
        }
        assert_eq!(cache.frames().len(), MAX_CENTROID_FRAMES);
        assert_eq!(cache.frames()[0].frame_id, 1);
        assert_eq!(cache.frames().last().unwrap().frame_id, 64);
        assert!(cache.frames().last().unwrap().invalid);
        assert!(!cache.push(&cache_packet(0, 1, 1), now).unwrap());
        assert_eq!(cache.stats().evicted_frames, 1);
    }

    #[test]
    fn expiry_of_a_newer_frame_retires_its_recently_updated_predecessors() {
        let now = Instant::now();
        let mut cache = CentroidCache::new(0);
        cache.push(&cache_packet(102, 0, 1), now).unwrap();
        cache
            .push(&cache_packet(101, 1, 1), now + Duration::from_secs(1))
            .unwrap();
        assert!(cache.poll(now + CENTROID_TTL));
        assert!(cache.frames().is_empty());
        assert_eq!(cache.stats().evicted_frames, 2);
        assert!(
            !cache
                .push(&cache_packet(101, 2, 1), now + CENTROID_TTL)
                .unwrap()
        );
        assert!(
            !cache
                .push(&cache_packet(102, 2, 1), now + CENTROID_TTL)
                .unwrap()
        );
        assert!(
            cache
                .push(&cache_packet(103, 2, 1), now + CENTROID_TTL)
                .unwrap()
        );
    }

    #[test]
    fn frame_and_sequence_wrap_preserve_order_identity_and_retirement() {
        let now = Instant::now();
        let mut cache = CentroidCache::new(0);
        for (frame, sequence) in [(0, 0), (u32::MAX, 0), (1, 0), (u32::MAX - 1, 0)] {
            cache.push(&cache_packet(frame, sequence, 1), now).unwrap();
        }
        for sequence in [u32::MAX, 1, u32::MAX - 1] {
            cache.push(&cache_packet(0, sequence, 1), now).unwrap();
        }
        assert_eq!(
            cache
                .frames()
                .iter()
                .map(|frame| frame.frame_id)
                .collect::<Vec<_>>(),
            [u32::MAX - 1, u32::MAX, 0, 1]
        );
        assert_eq!(
            cache.frames()[2]
                .groups
                .iter()
                .map(|group| group.sequence)
                .collect::<Vec<_>>(),
            [u32::MAX - 1, u32::MAX, 0, 1]
        );
        assert!(!cache.push(&cache_packet(0, u32::MAX, 1), now).unwrap());
        assert_eq!(cache.stats().duplicates, 1);
        assert!(cache.poll(now + CENTROID_TTL));
        assert!(
            !cache
                .push(&cache_packet(u32::MAX, 2, 1), now + CENTROID_TTL)
                .unwrap()
        );
        assert!(
            !cache
                .push(&cache_packet(1, 2, 1), now + CENTROID_TTL)
                .unwrap()
        );
        assert!(
            cache
                .push(&cache_packet(2, 2, 1), now + CENTROID_TTL)
                .unwrap()
        );
    }

    #[test]
    fn ambiguous_half_range_ids_cannot_corrupt_frame_or_packet_ordering() {
        let now = Instant::now();
        let mut cache = CentroidCache::new(0);
        cache.push(&cache_packet(0, 0, 1), now).unwrap();
        assert!(!cache.push(&cache_packet(0x8000_0000, 1, 1), now).unwrap());
        assert_eq!(cache.frames().len(), 1);
        assert_eq!(cache.stats().stale_packets, 1);
        assert!(cache.push(&cache_packet(0, 0x8000_0000, 1), now).is_err());
        assert!(cache.frames()[0].invalid);
    }

    #[test]
    fn advancing_counters_cannot_expand_live_ordering_beyond_half_range() {
        let now = Instant::now();
        let mut cache = CentroidCache::new(0);
        cache.push(&cache_packet(0, 0, 1), now).unwrap();
        cache.push(&cache_packet(0x8000_0001, 0, 1), now).unwrap();
        let revision = cache.revision();
        assert!(!cache.push(&cache_packet(2, 0, 1), now).unwrap());
        assert_eq!(cache.revision(), revision);
        assert_eq!(
            cache
                .frames()
                .iter()
                .map(|frame| frame.frame_id)
                .collect::<Vec<_>>(),
            [0x8000_0001, 0]
        );
        assert!(cache.push(&cache_packet(0, 1, 1), now).unwrap());
        assert_eq!(cache.stats().stale_packets, 1);

        let mut cache = CentroidCache::new(0);
        cache.push(&cache_packet(7, 0, 1), now).unwrap();
        cache.push(&cache_packet(7, 0x8000_0001, 1), now).unwrap();
        assert!(cache.push(&cache_packet(7, 2, 1), now).is_err());
        assert!(cache.frames()[0].invalid);
        assert!(cache.frames()[0].groups.is_empty());
    }
}
