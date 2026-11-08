//! Complete offline calibration recording, independent of the lossy live view.
//!
//! JPEGs are display derivatives; the companion UDP recording is authoritative.
//! Every raw centroid/pose packet has an `archive_record_id` timeline entry,
//! including duplicates and values that cannot be projected. Full-image mode
//! joins overlays to complete images; metadata mode emits one final overlay
//! per bounded camera/frame join without decoding or encoding image pixels.

use std::collections::{BTreeMap, VecDeque};
use std::path::Path;
use std::time::{Duration, Instant};

use image::codecs::jpeg::JpegEncoder;
use serde::{Serialize, Serializer, ser::SerializeSeq};
use serde_json::{Value, json};

use crate::centroids::{CentroidPacket, FrameCentroids};
use crate::poses::PoseCache;
use crate::rerun_feed::{self, CameraCalibration, FeedError};
use crate::stream::{DecodedImage, HEIGHT, WIDTH};

const FRAME_CAPACITY: usize = 32;
const GROUP_CAPACITY: usize = 256;
const JPEG_QUALITY: u8 = 90;
const MAX_METADATA_FRAMES: usize = 100_000;

struct FrameState {
    camera: u8,
    frame_id: u32,
    timestamp_ns: u64,
    image_logged: bool,
    centroid_packets: BTreeMap<u32, CentroidPacket>,
    centroid_invalid: bool,
    centroid_revision: u64,
    centroid_groups: usize,
    poses: PoseCache,
    // This is an offline per-frame cache. A fixed Instant disables live TTL
    // expiry while retaining PoseCache's atomic validation/conflict handling.
    pose_clock: Instant,
}

impl FrameState {
    fn new(camera: u8, frame_id: u32, timestamp_ns: u64) -> Self {
        Self {
            camera,
            frame_id,
            timestamp_ns,
            image_logged: false,
            centroid_packets: BTreeMap::new(),
            centroid_invalid: false,
            centroid_revision: 0,
            centroid_groups: 0,
            poses: PoseCache::new(camera),
            pose_clock: Instant::now(),
        }
    }

    fn add_centroids(&mut self, packet: &CentroidPacket) {
        if self.centroid_invalid {
            return;
        }
        if let Some(previous) = self.centroid_packets.get(&packet.sequence) {
            if previous == packet {
                return;
            }
            self.centroid_invalid = true;
        } else if self
            .centroid_packets
            .values()
            .next()
            .is_some_and(|previous| previous.version != packet.version)
            || self.centroid_groups + packet.groups.len() > GROUP_CAPACITY
        {
            self.centroid_invalid = true;
        } else {
            self.centroid_groups += packet.groups.len();
            self.centroid_packets
                .insert(packet.sequence, packet.clone());
        }
        self.centroid_revision += 1;
        if self.centroid_invalid {
            // Raw packets were already persisted independently. Retract a
            // partial or conflicting overlay instead of displaying it as whole.
            self.centroid_packets.clear();
            self.centroid_groups = 0;
        }
    }

    fn centroid_frame(&self) -> FrameCentroids {
        FrameCentroids {
            camera: self.camera,
            frame_id: self.frame_id,
            revision: self.centroid_revision,
            received_at: self.pose_clock,
            groups: self
                .centroid_packets
                .values()
                .flat_map(|packet| packet.groups.iter().cloned())
                .collect(),
            invalid: self.centroid_invalid,
        }
    }
}

#[derive(Default)]
struct FrameHistory {
    frames: VecDeque<FrameState>,
}

impl FrameHistory {
    fn get(
        &mut self,
        camera: u8,
        frame_id: u32,
        timestamp_ns: u64,
    ) -> Result<&mut FrameState, FeedError> {
        if let Some(index) = self
            .frames
            .iter()
            .position(|frame| frame.camera == camera && frame.frame_id == frame_id)
        {
            let frame = &mut self.frames[index];
            if frame.timestamp_ns != timestamp_ns {
                return Err("one camera frame was assigned inconsistent capture timestamps".into());
            }
            return Ok(frame);
        }
        if self.frames.len() == FRAME_CAPACITY {
            self.frames.pop_front();
        }
        self.frames
            .push_back(FrameState::new(camera, frame_id, timestamp_ns));
        Ok(self.frames.back_mut().unwrap())
    }
}

/// Synchronous file recorder: callers pass every complete image and packet,
/// without live-view throttling. `timestamp_ns` must always be the first image
/// packet timestamp for this camera/frame (or a consistently chosen fallback
/// for frames with no image). Input pixels remain native and unflipped.
pub struct CalibrationArchive {
    recording: rerun::RecordingStream,
    calibrations: [CameraCalibration; 2],
    history: FrameHistory,
    raw_record_id: i64,
    metadata_only: bool,
    // Retain only identities/timestamps after emitting a bounded join. Late
    // packets remain raw records, but cannot replace a final overlay with a
    // newly-created partial frame or revive an invalid tombstone.
    retired_metadata: BTreeMap<(u8, u32), u64>,
}

impl CalibrationArchive {
    pub fn new(path: &Path, calibrations: [CameraCalibration; 2]) -> Result<Self, FeedError> {
        Self::with_mode(path, calibrations, false)
    }

    /// Preserve every raw metadata packet and final camera/frame overlays,
    /// without image JPEG generation. The raw UDP clip retains every image.
    pub fn new_metadata(
        path: &Path,
        calibrations: [CameraCalibration; 2],
    ) -> Result<Self, FeedError> {
        Self::with_mode(path, calibrations, true)
    }

    fn with_mode(
        path: &Path,
        calibrations: [CameraCalibration; 2],
        metadata_only: bool,
    ) -> Result<Self, FeedError> {
        for calibration in calibrations {
            calibration.validate()?;
        }
        if !rerun::decide_logging_enabled(true) {
            return Err(
                "calibration archive requires Rerun logging; unset RERUN or use RERUN=on".into(),
            );
        }
        let recording = rerun::RecordingStreamBuilder::new("LUTCam Calibration")
            .enabled(true)
            .batcher_config(rerun::log::ChunkBatcherConfig {
                max_bytes_in_flight: 16 * 1024 * 1024,
                ..Default::default()
            })
            .save(path)?;
        rerun_feed::set_layout(&recording, false, true)?;
        recording.log_static(
            "archive/format",
            &rerun::TextDocument::new(
                json!({
                    "image_encoding": if metadata_only { "companion raw UDP only" } else { "JPEG" },
                    "jpeg_quality": (!metadata_only).then_some(JPEG_QUALITY),
                    "metadata_only": metadata_only,
                    "display_rows_flipped": true, "camera_0_side": "right",
                    "authoritative_source": "companion raw UDP recording",
                    "capture_time": "first image packet timestamp, supplied by caller",
                    "camera_frame_id": "full outer frame ID; cameras remain separate entities",
                    "archive_record_id": "unique raw packet record; includes duplicates",
                    "raw_centroid_coordinates": "unmodified composite unsigned Q5",
                    "overlay_frame_capacity": FRAME_CAPACITY,
                    "overlay_centroid_group_capacity": GROUP_CAPACITY,
                    "overlay_pose_capacity": crate::poses::MAX_POSE_RECORDS,
                })
                .to_string(),
            ),
        )?;
        Ok(Self {
            recording,
            calibrations,
            history: FrameHistory::default(),
            raw_record_id: 0,
            metadata_only,
            retired_metadata: BTreeMap::new(),
        })
    }

    /// Record one complete JPEG display image and restore any earlier metadata
    /// for this exact frame after clearing the previous frame's overlays.
    pub fn image(&mut self, image: &DecodedImage, timestamp_ns: u64) -> Result<(), FeedError> {
        validate_camera(image.camera)?;
        if self.metadata_only {
            return Err("metadata-only archive does not accept decoded images".into());
        }
        let jpeg = display_jpeg(&image.pixels)?;
        let frame = self
            .history
            .get(image.camera, image.frame_id, timestamp_ns)?;
        set_frame_time(&self.recording, image.frame_id, timestamp_ns);
        let camera = image.camera;
        self.recording.log(
            format!("camera{camera}/image"),
            &self.calibrations[usize::from(camera)].pinhole(),
        )?;
        self.recording.log(
            format!("camera{camera}/image"),
            &rerun::EncodedImage::from_file_contents(jpeg).with_media_type("image/jpeg"),
        )?;
        self.recording.log(
            format!("camera{camera}/frame_id"),
            &rerun::Scalars::new([f64::from(image.frame_id)]),
        )?;
        self.recording.log(
            format!("camera{camera}/image/centroids"),
            &rerun::Clear::recursive(),
        )?;
        self.recording.log(
            format!("camera{camera}/image/lftags"),
            &rerun::Clear::recursive(),
        )?;
        self.recording
            .log(format!("camera{camera}/lftags"), &rerun::Clear::recursive())?;
        frame.image_logged = true;
        log_overlays(
            &self.recording,
            self.calibrations[usize::from(camera)],
            frame,
        )
    }

    /// Preserve every tuple and flag, including invalid/nonprojectable values.
    /// Duplicate/conflicting packets remain separate raw archive records.
    pub fn centroids(
        &mut self,
        packet: &CentroidPacket,
        timestamp_ns: u64,
    ) -> Result<(), FeedError> {
        validate_camera(packet.camera)?;
        self.raw_centroids(packet, timestamp_ns)?;
        if packet.version != 0
            || packet.groups.is_empty()
            || packet.groups.len() > 4
            || packet
                .groups
                .iter()
                .any(|group| group.sequence != packet.sequence || group.centroids.len() != 9)
        {
            return Err("invalid parsed centroid packet shape; raw record was retained".into());
        }
        if !self.prepare_metadata_frame(packet.camera, packet.frame_id, timestamp_ns)? {
            return Ok(());
        }
        let frame = self
            .history
            .get(packet.camera, packet.frame_id, timestamp_ns)?;
        frame.add_centroids(packet);
        if self.metadata_only {
            return Ok(());
        }
        log_overlays(
            &self.recording,
            self.calibrations[usize::from(packet.camera)],
            frame,
        )
    }

    /// Retain a rejected centroid datagram and retract measurements only when
    /// its complete outer header establishes this camera and frame identity.
    pub fn malformed_centroids(
        &mut self,
        data: &[u8],
        camera: u8,
        timestamp_ns: u64,
    ) -> Result<(), FeedError> {
        validate_camera(camera)?;
        let parse_error = match crate::centroids::parse_datagram(data) {
            Ok(packet) if packet.camera == camera => {
                return Err("malformed centroid logging requires a rejected packet".into());
            }
            Ok(_) => "wrong camera in centroid packet".to_owned(),
            Err(error) => error.to_string(),
        };
        let frame_id = trusted_centroid_frame(data, camera);
        self.raw_record(
            camera,
            frame_id,
            timestamp_ns,
            "centroids",
            json!({
                "camera": camera, "frame_id": frame_id,
                "datagram_hex": hex_bytes(data), "parse_error": parse_error,
            }),
        )?;
        if let Some(frame_id) = frame_id {
            self.invalidate_centroids(camera, frame_id, timestamp_ns)?;
        }
        Ok(())
    }

    /// Retract a frame found invalid by the offline packet history, including
    /// one whose image has already left this archive's bounded join cache.
    pub fn invalidate_centroids(
        &mut self,
        camera: u8,
        frame_id: u32,
        timestamp_ns: u64,
    ) -> Result<(), FeedError> {
        validate_camera(camera)?;
        if !self.prepare_metadata_frame(camera, frame_id, timestamp_ns)? {
            let mut frame = FrameState::new(camera, frame_id, timestamp_ns);
            frame.centroid_invalid = true;
            frame.centroid_revision = 1;
            set_frame_time(&self.recording, frame_id, timestamp_ns);
            return rerun_feed::log_centroid_frame(&self.recording, &frame.centroid_frame());
        }
        let frame = self.history.get(camera, frame_id, timestamp_ns)?;
        frame.centroid_invalid = true;
        frame.centroid_revision += 1;
        frame.centroid_packets.clear();
        frame.centroid_groups = 0;
        if self.metadata_only {
            return Ok(());
        }
        set_frame_time(&self.recording, frame_id, timestamp_ns);
        rerun_feed::log_centroid_frame(&self.recording, &frame.centroid_frame())
    }

    /// Preserve the exact datagram bytes plus accepted pose fields/geometry.
    /// Malformed packets are retained with their parser error, without aborting
    /// the remaining archive. Conflicts invalidate the matching pose overlay.
    pub fn poses(&mut self, data: &[u8], camera: u8, timestamp_ns: u64) -> Result<(), FeedError> {
        validate_camera(camera)?;
        let mut parsed = PoseCache::new(camera);
        let parse_error = parsed
            .push(data, Instant::now())
            .err()
            .map(|error| error.to_string());
        let frames = parsed.frames();
        let frame_id = frames.first().map(|frame| frame.frame_id);
        self.raw_record(
            camera,
            frame_id,
            timestamp_ns,
            "poses",
            json!({
                "camera": camera, "frame_id": frame_id,
                "datagram_hex": hex_bytes(data), "parse_error": parse_error,
            }),
        )?;
        if let Some(frame_id) = frame_id {
            if !self.prepare_metadata_frame(camera, frame_id, timestamp_ns)? {
                for frame in frames.iter().filter(|frame| frame.invalid) {
                    set_frame_time(&self.recording, frame_id, timestamp_ns);
                    rerun_feed::log_detection_frame(
                        &self.recording,
                        self.calibrations[usize::from(camera)],
                        frame,
                    )?;
                }
                return Ok(());
            }
            let frame = self.history.get(camera, frame_id, timestamp_ns)?;
            // The per-frame cache checks duplicate bytes, version consistency,
            // common/pose headers and aggregate capacity. Its invalid tombstone
            // is logged even when push returns an error.
            let _ = frame.poses.push(data, frame.pose_clock);
            if self.metadata_only {
                return Ok(());
            }
            log_overlays(
                &self.recording,
                self.calibrations[usize::from(camera)],
                frame,
            )?;
        }
        Ok(())
    }

    /// Wait until all queued SDK data reaches the file sink; report I/O errors.
    pub fn finish(&mut self) -> Result<(), FeedError> {
        if self.metadata_only {
            while let Some(frame) = self.history.frames.pop_front() {
                self.finalize_metadata_frame(frame)?;
            }
        }
        self.recording.flush_blocking()?;
        Ok(())
    }

    fn prepare_metadata_frame(
        &mut self,
        camera: u8,
        frame_id: u32,
        timestamp_ns: u64,
    ) -> Result<bool, FeedError> {
        if !self.metadata_only {
            return Ok(true);
        }
        if let Some(&previous) = self.retired_metadata.get(&(camera, frame_id)) {
            if previous != timestamp_ns {
                return Err("one camera frame was assigned inconsistent capture timestamps".into());
            }
            return Ok(false);
        }
        if self.history.frames.len() == FRAME_CAPACITY
            && !self
                .history
                .frames
                .iter()
                .any(|frame| frame.camera == camera && frame.frame_id == frame_id)
        {
            let frame = self.history.frames.pop_front().unwrap();
            self.finalize_metadata_frame(frame)?;
        }
        Ok(true)
    }

    fn finalize_metadata_frame(&mut self, mut frame: FrameState) -> Result<(), FeedError> {
        if self.retired_metadata.len() == MAX_METADATA_FRAMES {
            return Err("metadata archive exceeds bounded 100,000-frame history".into());
        }
        set_frame_time(&self.recording, frame.frame_id, frame.timestamp_ns);
        self.recording.log(
            format!("camera{}/image", frame.camera),
            &self.calibrations[usize::from(frame.camera)].pinhole(),
        )?;
        self.recording.log(
            format!("camera{}/frame_id", frame.camera),
            &rerun::Scalars::new([f64::from(frame.frame_id)]),
        )?;
        self.recording.log(
            format!("camera{}/image/lftags", frame.camera),
            &rerun::Clear::recursive(),
        )?;
        self.recording.log(
            format!("camera{}/lftags", frame.camera),
            &rerun::Clear::recursive(),
        )?;
        frame.image_logged = true;
        log_overlays(
            &self.recording,
            self.calibrations[usize::from(frame.camera)],
            &frame,
        )?;
        self.retired_metadata
            .insert((frame.camera, frame.frame_id), frame.timestamp_ns);
        Ok(())
    }

    fn next_raw_record_id(&mut self) -> Result<i64, FeedError> {
        self.raw_record_id = self
            .raw_record_id
            .checked_add(1)
            .ok_or("archive record counter overflow")?;
        Ok(self.raw_record_id)
    }

    fn raw_centroids(
        &mut self,
        packet: &CentroidPacket,
        timestamp_ns: u64,
    ) -> Result<(), FeedError> {
        let archive_record_id = self.next_raw_record_id()?;
        let text = serde_json::to_string(&RawCentroids {
            archive_record_id,
            camera: packet.camera,
            coordinate_domain: "compact composite unsigned Q5",
            frame_id: packet.frame_id,
            frame_timestamp_ns: timestamp_ns,
            groups: RawCentroidGroups(&packet.groups),
            sequence: packet.sequence,
            version: packet.version,
        })?;
        self.raw_text(
            packet.camera,
            Some(packet.frame_id),
            timestamp_ns,
            "centroids",
            text,
        )
    }

    fn raw_record(
        &mut self,
        camera: u8,
        frame_id: Option<u32>,
        timestamp_ns: u64,
        product: &str,
        mut value: Value,
    ) -> Result<(), FeedError> {
        value["archive_record_id"] = self.next_raw_record_id()?.into();
        value["frame_timestamp_ns"] = timestamp_ns.into();
        self.raw_text(camera, frame_id, timestamp_ns, product, value.to_string())
    }

    fn raw_text(
        &mut self,
        camera: u8,
        frame_id: Option<u32>,
        timestamp_ns: u64,
        product: &str,
        text: String,
    ) -> Result<(), FeedError> {
        self.recording
            .set_time("capture_time", Duration::from_nanos(timestamp_ns));
        if let Some(frame_id) = frame_id {
            self.recording
                .set_time_sequence("camera_frame_id", i64::from(frame_id));
        } else {
            self.recording.disable_timeline("camera_frame_id");
        }
        self.recording
            .set_time_sequence("archive_record_id", self.raw_record_id);
        let result = self.recording.log(
            format!("camera{camera}/archive/{product}"),
            &rerun::TextDocument::new(text),
        );
        // Never leave this packet-specific timeline attached to image/overlay
        // rows, including when logging fails.
        self.recording.disable_timeline("archive_record_id");
        result?;
        Ok(())
    }
}

/// A recognizable full product-2 header establishes identity even when its
/// version, declared length, reserved bits or payload fail strict validation.
/// Truncated headers and other cameras/products cannot invalidate a frame.
pub fn trusted_centroid_frame(data: &[u8], camera: u8) -> Option<u32> {
    (camera <= 1
        && data.len() >= 32
        && data[..4] == 0x4c55_5443_u32.to_be_bytes()
        && data[5] == camera
        && data[6] == 2
        && data[7] == 32)
        .then(|| u32::from_be_bytes(data[8..12].try_into().unwrap()))
}

fn validate_camera(camera: u8) -> Result<(), FeedError> {
    if camera > 1 {
        return Err("camera must be 0 or 1".into());
    }
    Ok(())
}

fn set_frame_time(recording: &rerun::RecordingStream, frame_id: u32, timestamp_ns: u64) {
    recording.disable_timeline("archive_record_id");
    recording.set_time("capture_time", Duration::from_nanos(timestamp_ns));
    recording.set_time_sequence("camera_frame_id", i64::from(frame_id));
}

fn log_overlays(
    recording: &rerun::RecordingStream,
    calibration: CameraCalibration,
    frame: &FrameState,
) -> Result<(), FeedError> {
    if !frame.image_logged {
        return Ok(());
    }
    set_frame_time(recording, frame.frame_id, frame.timestamp_ns);
    rerun_feed::log_centroid_frame(recording, &frame.centroid_frame())?;
    for detections in frame.poses.frames() {
        rerun_feed::log_detection_frame(recording, calibration, &detections)?;
    }
    Ok(())
}

// Match the established BTreeMap JSON field order while borrowing every tuple.
// A dense recording contains millions of packets; constructing a Value tree
// here would allocate several strings and map nodes for every individual point.
#[derive(Serialize)]
struct RawCentroids<'a> {
    archive_record_id: i64,
    camera: u8,
    coordinate_domain: &'static str,
    frame_id: u32,
    frame_timestamp_ns: u64,
    groups: RawCentroidGroups<'a>,
    sequence: u32,
    version: u8,
}

struct RawCentroidGroups<'a>(&'a [crate::centroids::CentroidGroup]);

impl Serialize for RawCentroidGroups<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        #[derive(Serialize)]
        struct Group<'a> {
            centroids: RawCentroidPoints<'a>,
            group_index: u8,
            sequence: u32,
        }
        let mut sequence = serializer.serialize_seq(Some(self.0.len()))?;
        for group in self.0 {
            sequence.serialize_element(&Group {
                centroids: RawCentroidPoints(&group.centroids),
                group_index: group.group_index,
                sequence: group.sequence,
            })?;
        }
        sequence.end()
    }
}

struct RawCentroidPoints<'a>(&'a [crate::centroids::Centroid]);

impl Serialize for RawCentroidPoints<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        #[derive(Serialize)]
        struct Point {
            raw_tuple: u64,
            valid: bool,
            x_q5: u16,
            y_q5: u16,
        }
        let mut sequence = serializer.serialize_seq(Some(self.0.len()))?;
        for point in self.0 {
            sequence.serialize_element(&Point {
                raw_tuple: (u64::from(point.y_q5) << 17)
                    | (u64::from(point.x_q5) << 1)
                    | u64::from(point.valid),
                valid: point.valid,
                x_q5: point.x_q5,
                y_q5: point.y_q5,
            })?;
        }
        sequence.end()
    }
}

#[cfg(test)]
fn centroid_json(packet: &CentroidPacket) -> Value {
    json!({
        "camera": packet.camera, "frame_id": packet.frame_id,
        "sequence": packet.sequence, "version": packet.version,
        "coordinate_domain": "compact composite unsigned Q5",
        "groups": packet.groups.iter().map(|group| json!({
            "sequence": group.sequence, "group_index": group.group_index,
            "centroids": group.centroids.iter().map(|point| json!({
                "valid": point.valid, "x_q5": point.x_q5, "y_q5": point.y_q5,
                "raw_tuple": (u64::from(point.y_q5) << 17) | (u64::from(point.x_q5) << 1) | u64::from(point.valid),
            })).collect::<Vec<_>>(),
        })).collect::<Vec<_>>(),
    })
}

fn hex_bytes(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut text = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        text.push(char::from(HEX[usize::from(byte >> 4)]));
        text.push(char::from(HEX[usize::from(byte & 15)]));
    }
    text
}

fn display_jpeg(pixels: &[u8]) -> Result<Vec<u8>, FeedError> {
    if pixels.len() != WIDTH * HEIGHT {
        return Err("calibration image must contain exactly 1280 by 800 monochrome pixels".into());
    }
    let flipped: Vec<u8> = pixels
        .chunks_exact(WIDTH)
        .rev()
        .flatten()
        .copied()
        .collect();
    let mut encoded = Vec::new();
    JpegEncoder::new_with_quality(&mut encoded, JPEG_QUALITY).encode(
        &flipped,
        WIDTH as u32,
        HEIGHT as u32,
        image::ExtendedColorType::L8,
    )?;
    Ok(encoded)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::centroids::{Centroid, CentroidGroup};

    fn packet(frame_id: u32, sequence: u32) -> CentroidPacket {
        CentroidPacket {
            camera: 0,
            frame_id,
            sequence,
            version: 0,
            groups: vec![CentroidGroup {
                sequence,
                group_index: 0,
                centroids: vec![
                    Centroid {
                        valid: false,
                        x_q5: u16::MAX,
                        y_q5: 12345
                    };
                    9
                ],
            }],
        }
    }

    fn pose_packet(frame_id: u32, sequence: u32) -> Vec<u8> {
        let mut words = vec![0x4c55_5443, 0x0000_0320, frame_id, sequence, 0, 88, 0, 0];
        words.extend([
            123,
            0x8000_0000 | ((frame_id & 255) << 20),
            0,
            0,
            0,
            0,
            0,
            0,
            0x0000_0001,
            0,
            1 << 29,
            0,
            0,
            0,
            1 << 29,
            0,
            0,
            0,
            1 << 29,
            0,
            0,
            10 << 16,
        ]);
        words.into_iter().flat_map(u32::to_be_bytes).collect()
    }

    #[test]
    fn jpeg_is_complete_and_physically_row_flipped_without_changing_input() {
        let mut pixels = vec![20; WIDTH * HEIGHT];
        pixels[WIDTH * (HEIGHT / 2)..].fill(220);
        let jpeg = display_jpeg(&pixels).unwrap();
        let decoded = image::load_from_memory_with_format(&jpeg, image::ImageFormat::Jpeg)
            .unwrap()
            .to_luma8();
        assert_eq!(decoded.dimensions(), (WIDTH as u32, HEIGHT as u32));
        assert!(decoded.get_pixel(100, 100)[0].abs_diff(220) <= 2);
        assert!(decoded.get_pixel(100, 700)[0].abs_diff(20) <= 2);
        assert_eq!(pixels[0], 20);
        assert!(display_jpeg(&pixels[..pixels.len() - 1]).is_err());
    }

    #[test]
    fn raw_invalid_centroids_and_original_q5_bits_survive_aggregation() {
        let packet = packet(7, 2);
        let json = centroid_json(&packet);
        assert_eq!(json["groups"][0]["centroids"][0]["valid"], false);
        assert_eq!(json["groups"][0]["centroids"][0]["x_q5"], 65535);
        assert_eq!(json["groups"][0]["centroids"][0]["y_q5"], 12345);
        let mut frame = FrameState::new(0, 7, 100);
        frame.add_centroids(&packet);
        frame.add_centroids(&packet);
        assert_eq!(frame.centroid_frame().groups, packet.groups);
        assert_eq!(frame.centroid_revision, 1);
        let mut other = packet.clone();
        other.sequence = 1;
        other.groups[0].sequence = 1;
        frame.add_centroids(&other);
        assert_eq!(frame.centroid_frame().groups.len(), 2);
    }

    #[test]
    fn conflicting_or_oversized_centroid_frames_retract_overlay() {
        let mut frame = FrameState::new(0, 7, 100);
        let original = packet(7, 0);
        frame.add_centroids(&original);
        let mut conflict = original.clone();
        conflict.groups[0].centroids[0].valid = true;
        frame.add_centroids(&conflict);
        assert!(frame.centroid_frame().invalid);
        assert!(frame.centroid_frame().groups.is_empty());
        frame.add_centroids(&original);
        assert!(frame.centroid_frame().groups.is_empty());
        let mut bounded = FrameState::new(0, 7, 100);
        for sequence in 0..=GROUP_CAPACITY as u32 {
            bounded.add_centroids(&packet(7, sequence));
        }
        assert!(bounded.centroid_invalid);
        assert!(bounded.centroid_packets.is_empty());
    }

    #[test]
    fn frame_cache_keeps_camera_identity_wrap_and_insertion_order() {
        let mut history = FrameHistory::default();
        history.get(0, u32::MAX, 1).unwrap().image_logged = true;
        history.get(1, u32::MAX, 2).unwrap();
        assert!(!history.get(1, u32::MAX, 2).unwrap().image_logged);
        assert!(history.get(0, u32::MAX, 1).unwrap().image_logged);
        assert!(history.get(0, u32::MAX, 3).is_err());
        for frame in 0..FRAME_CAPACITY as u32 {
            history.get(0, frame, u64::from(frame) + 10).unwrap();
        }
        assert_eq!(history.frames.len(), FRAME_CAPACITY);
        assert_eq!(history.frames.front().unwrap().frame_id, 0);
    }

    #[test]
    fn per_frame_pose_cache_preserves_accepted_fields_and_retracts_conflict() {
        let mut frame = FrameState::new(0, 7, 100);
        let packet = pose_packet(7, 2);
        assert!(frame.poses.push(&packet, frame.pose_clock).unwrap());
        assert!(!frame.poses.push(&packet, frame.pose_clock).unwrap());
        let frames = frame.poses.frames();
        assert_eq!(frames[0].detections.len(), 1);
        assert_eq!(frames[0].detections[0].tag_id, 123);
        assert_eq!(
            frames[0].detections[0].pose.pose.translation_q16,
            [0, 0, 10 << 16]
        );
        let mut conflict = packet.clone();
        conflict[35] = 124;
        assert!(frame.poses.push(&conflict, frame.pose_clock).is_err());
        assert!(frame.poses.frames()[0].invalid);
        assert_eq!(hex_bytes(&[0, 15, 128, 255]), "000f80ff");
    }

    #[test]
    fn borrowed_raw_centroid_json_matches_owned_record() {
        for version in [0] {
            for count in 1..=4 {
                let mut packet = packet(u32::MAX, u32::MAX);
                packet.camera = 1;
                packet.version = version;
                packet.groups = (0..count)
                    .map(|index| {
                        let mut group = packet.groups[0].clone();
                        group.group_index = index;
                        group.centroids[0].valid = true;
                        group.centroids[0].y_q5 = u16::MAX;
                        group
                    })
                    .collect();
                let mut expected = centroid_json(&packet);
                expected["archive_record_id"] = i64::MAX.into();
                expected["frame_timestamp_ns"] = u64::MAX.into();
                let actual = RawCentroids {
                    archive_record_id: i64::MAX,
                    camera: packet.camera,
                    coordinate_domain: "compact composite unsigned Q5",
                    frame_id: packet.frame_id,
                    frame_timestamp_ns: u64::MAX,
                    groups: RawCentroidGroups(&packet.groups),
                    sequence: packet.sequence,
                    version: packet.version,
                };
                assert_eq!(
                    serde_json::to_string(&actual).unwrap(),
                    expected.to_string()
                );
            }
        }
    }

    #[test]
    fn metadata_archive_defers_complete_overlays_and_retains_raw_packets_and_tombstones() {
        let (recording, storage) =
            rerun::RecordingStreamBuilder::new("metadata-archive-regression")
                .enabled(true)
                .memory()
                .unwrap();
        let mut archive = CalibrationArchive {
            recording,
            calibrations: [CameraCalibration::default(); 2],
            history: FrameHistory::default(),
            raw_record_id: 0,
            metadata_only: true,
            retired_metadata: BTreeMap::new(),
        };
        let mut first = packet(7, 1);
        first.groups[0].centroids[0].valid = true;
        archive.centroids(&first, 1000).unwrap();
        archive.centroids(&first, 1000).unwrap(); // Duplicate is a separate raw record.
        archive.centroids(&packet(7, 2), 1000).unwrap();
        archive.poses(&pose_packet(7, 3), 0, 1000).unwrap();
        let mut second = packet(7, 1);
        second.camera = 1;
        archive.centroids(&second, 1100).unwrap();
        let mut conflict = second.clone();
        conflict.groups[0].centroids[0].x_q5 = 1;
        archive.centroids(&conflict, 1100).unwrap();
        let before = storage.take();
        let chunks = |messages: Vec<rerun::log::LogMsg>| -> Vec<rerun::log::Chunk> {
            messages
                .into_iter()
                .filter_map(|message| match message {
                    rerun::log::LogMsg::ArrowMsg(_, arrow) => {
                        Some(rerun::log::Chunk::from_arrow_msg(&arrow).unwrap())
                    }
                    _ => None,
                })
                .collect()
        };
        let mut all = chunks(before);
        assert!(all.iter().all(|chunk| {
            let path = chunk.entity_path().to_string();
            !path.contains("/image") && !path.contains("/lftags")
        }));
        // Force eviction of both camera/frame joins. Their final snapshots
        // must include every accumulated group without requiring an image.
        for frame in 100..132 {
            archive
                .centroids(&packet(frame, frame), 10_000 + u64::from(frame))
                .unwrap();
        }
        all.extend(chunks(storage.take()));
        archive.invalidate_centroids(0, 7, 1000).unwrap();
        // A late valid packet must not resurrect the evicted invalid frame.
        archive.centroids(&first, 1000).unwrap();
        archive.finish().unwrap();
        archive.finish().unwrap();
        all.extend(chunks(storage.take()));

        let text_component = rerun::TextDocument::descriptor_text().component;
        let blob_component = rerun::EncodedImage::descriptor_blob().component;
        let mut raw = Vec::new();
        let mut poses = Vec::new();
        let mut snapshots = Vec::new();
        let mut pose_snapshots = Vec::new();
        for chunk in &all {
            assert_eq!(
                chunk
                    .iter_component::<rerun::components::Blob>(blob_component)
                    .count(),
                0
            );
            let path = chunk.entity_path().to_string();
            let path = path.trim_start_matches('/');
            let indices = chunk.iter_component_indices("capture_time".into(), text_component);
            for ((time, row), batch) in
                indices.zip(chunk.iter_component::<rerun::components::Text>(text_component))
            {
                for text in batch.iter() {
                    let value: Value = serde_json::from_str(text.0.as_str()).unwrap();
                    if path.contains("/archive/") {
                        assert!(chunk.timelines().contains_key(&"archive_record_id".into()));
                        assert!(chunk.timelines().contains_key(&"camera_frame_id".into()));
                    }
                    match path {
                        "camera0/archive/centroids" | "camera1/archive/centroids" => {
                            raw.push(value)
                        }
                        "camera0/archive/poses" => poses.push(value),
                        "camera0/image/centroids/raw" | "camera1/image/centroids/raw" => {
                            assert!(!chunk.timelines().contains_key(&"archive_record_id".into()));
                            snapshots.push((time.as_i64(), row, value));
                        }
                        "camera0/lftags/raw" => pose_snapshots.push(value),
                        _ => {}
                    }
                }
            }
        }
        assert_eq!(raw.len(), 38);
        assert_eq!(poses.len(), 1);
        assert_eq!(poses[0]["datagram_hex"], hex_bytes(&pose_packet(7, 3)));
        raw.sort_by_key(|value| value["archive_record_id"].as_i64());
        assert_eq!(raw[0]["groups"][0]["centroids"][1]["valid"], false);
        assert_eq!(raw[0]["groups"][0]["centroids"][1]["x_q5"], 65535);
        assert_ne!(raw[0]["archive_record_id"], raw[1]["archive_record_id"]);
        assert_eq!(raw[0]["sequence"], raw[1]["sequence"]);
        let original: Vec<_> = snapshots
            .iter()
            .filter(|(time, _, _)| *time == 1000)
            .collect();
        assert_eq!(original.len(), 2); // One final snapshot, then explicit retraction.
        assert!(
            original
                .iter()
                .any(|(_, _, value)| value["groups"].as_array().unwrap().len() == 2)
        );
        let final_original = original.iter().max_by_key(|(_, row, _)| row).unwrap();
        assert_eq!(final_original.2["invalid"], true);
        assert!(final_original.2["groups"].as_array().unwrap().is_empty());
        let other = snapshots.iter().find(|(time, _, _)| *time == 1100).unwrap();
        assert_eq!(other.2["invalid"], true);
        assert!(
            pose_snapshots
                .iter()
                .any(|value| value["detections"][0]["tag_id"] == 123)
        );
        assert_eq!(snapshots.len(), 35);
        assert!(archive.centroids(&first, 1001).is_err());
    }

    #[test]
    fn metadata_archive_preserves_malformed_packets_and_pose_conflict_tombstones() {
        let (recording, storage) =
            rerun::RecordingStreamBuilder::new("metadata-malformed-regression")
                .enabled(true)
                .memory()
                .unwrap();
        let mut archive = CalibrationArchive {
            recording,
            calibrations: [CameraCalibration::default(); 2],
            history: FrameHistory::default(),
            raw_record_id: 0,
            metadata_only: true,
            retired_metadata: BTreeMap::new(),
        };
        archive.centroids(&packet(7, 1), 1000).unwrap();
        let mut malformed = Vec::new();
        for word in [0x4c55_5443_u32, 0x0000_0220, 7, 9, 0, 72, 0, 0] {
            malformed.extend(word.to_be_bytes());
        }
        malformed.resize(104, 0);
        malformed[32] = 0x80;
        archive.malformed_centroids(&malformed, 0, 1000).unwrap();
        let mut wrong_camera = malformed.clone();
        wrong_camera[5] = 1;
        archive.malformed_centroids(&wrong_camera, 0, 1000).unwrap();
        let pose = pose_packet(7, 3);
        archive.poses(&pose, 0, 1000).unwrap();
        let mut conflicting_pose = pose.clone();
        conflicting_pose[35] = 124;
        archive.poses(&conflicting_pose, 0, 1000).unwrap();
        assert!(
            archive
                .image(
                    &DecodedImage {
                        camera: 0,
                        frame_id: 7,
                        pixels: Vec::new(),
                    },
                    1000
                )
                .is_err()
        );
        archive.finish().unwrap();
        let text_component = rerun::TextDocument::descriptor_text().component;
        let mut raw = Vec::new();
        let mut pose_raw = Vec::new();
        let mut centroid_snapshots = Vec::new();
        let mut pose_snapshots = Vec::new();
        for message in storage.take() {
            let rerun::log::LogMsg::ArrowMsg(_, arrow) = message else {
                continue;
            };
            let chunk = rerun::log::Chunk::from_arrow_msg(&arrow).unwrap();
            let path = chunk.entity_path().to_string();
            for batch in chunk.iter_component::<rerun::components::Text>(text_component) {
                for text in batch.iter() {
                    let value: Value = serde_json::from_str(text.0.as_str()).unwrap();
                    match path.trim_start_matches('/') {
                        "camera0/archive/centroids" => {
                            if value["frame_id"].is_null() {
                                assert!(!chunk.timelines().contains_key(&"camera_frame_id".into()));
                            }
                            raw.push(value);
                        }
                        "camera0/archive/poses" => pose_raw.push(value),
                        "camera0/image/centroids/raw" => centroid_snapshots.push(value),
                        "camera0/lftags/raw" => pose_snapshots.push(value),
                        _ => {}
                    }
                }
            }
        }
        raw.sort_by_key(|value| value["archive_record_id"].as_i64());
        assert_eq!(raw.len(), 3);
        assert_eq!(raw[1]["datagram_hex"], hex_bytes(&malformed));
        assert_eq!(raw[1]["frame_id"], 7);
        assert!(raw[1]["parse_error"].is_string());
        assert_eq!(raw[2]["datagram_hex"], hex_bytes(&wrong_camera));
        assert!(raw[2]["frame_id"].is_null());
        assert_eq!(pose_raw.len(), 2);
        assert_eq!(centroid_snapshots.len(), 1);
        assert_eq!(centroid_snapshots[0]["invalid"], true);
        assert_eq!(pose_snapshots.len(), 1);
        assert_eq!(pose_snapshots[0]["invalid"], true);
    }

    #[test]
    fn sdk_stream_preserves_all_raw_packets_and_restores_metadata_after_image() {
        let (recording, storage) = rerun::RecordingStreamBuilder::new("archive-regression")
            .enabled(true)
            .memory()
            .unwrap();
        let mut archive = CalibrationArchive {
            recording,
            calibrations: [CameraCalibration::default(); 2],
            history: FrameHistory::default(),
            raw_record_id: 0,
            metadata_only: false,
            retired_metadata: BTreeMap::new(),
        };
        let mut first = packet(7, 1);
        first.groups[0].centroids[0] = Centroid {
            valid: true,
            x_q5: 321,
            y_q5: 643,
        };
        archive.centroids(&first, 1000).unwrap();
        archive.centroids(&first, 1000).unwrap(); // Keep duplicate raw records.
        archive.poses(&pose_packet(7, 3), 0, 1000).unwrap();
        let before_image = storage.take();
        let chunks = |messages: Vec<rerun::log::LogMsg>| -> Vec<rerun::log::Chunk> {
            messages
                .into_iter()
                .filter_map(|message| match message {
                    rerun::log::LogMsg::ArrowMsg(_, arrow) => {
                        Some(rerun::log::Chunk::from_arrow_msg(&arrow).unwrap())
                    }
                    _ => None,
                })
                .collect()
        };
        let mut all = chunks(before_image);
        assert!(
            all.iter()
                .all(|chunk| !chunk.entity_path().to_string().contains("/image")
                    && !chunk.entity_path().to_string().contains("/lftags"))
        );
        archive
            .image(
                &DecodedImage {
                    camera: 0,
                    frame_id: 7,
                    pixels: vec![32; WIDTH * HEIGHT],
                },
                1000,
            )
            .unwrap();
        archive.centroids(&packet(7, 2), 1000).unwrap();
        archive
            .image(
                &DecodedImage {
                    camera: 0,
                    frame_id: 8,
                    pixels: vec![64; WIDTH * HEIGHT],
                },
                2000,
            )
            .unwrap();
        // Retraction must reach persisted overlays even after the bounded
        // association cache has forgotten the original image.
        for frame_id in 100..140 {
            archive.history.get(0, frame_id, 3000).unwrap();
        }
        let mut malformed = Vec::new();
        for word in [0x4c55_5443_u32, 0x0000_0220, 7, 9, 0, 72, 0, 0] {
            malformed.extend(word.to_be_bytes());
        }
        malformed.resize(104, 0);
        malformed[32] = 0x80;
        archive.malformed_centroids(&malformed, 0, 1000).unwrap();
        let mut wrong_camera = malformed.clone();
        wrong_camera[5] = 1;
        archive.malformed_centroids(&wrong_camera, 0, 1000).unwrap();
        archive.finish().unwrap();
        all.extend(chunks(storage.take()));

        let text_component = rerun::TextDocument::descriptor_text().component;
        let clear_component = rerun::Clear::descriptor_is_recursive().component;
        let blob_component = rerun::EncodedImage::descriptor_blob().component;
        let mut raw_packets = Vec::new();
        let mut pose_packets = Vec::new();
        let mut centroid_snapshots = Vec::new();
        let mut pose_snapshots = Vec::new();
        let mut clears = Vec::new();
        let mut pose_overlay_clears = Vec::new();
        let mut image_count = 0;
        for chunk in &all {
            let path = chunk.entity_path().to_string();
            let path = path.trim_start_matches('/');
            let text_indices = chunk.iter_component_indices("capture_time".into(), text_component);
            for ((time, row_id), batch) in
                text_indices.zip(chunk.iter_component::<rerun::components::Text>(text_component))
            {
                for text in batch.iter() {
                    let value: Value = serde_json::from_str(text.0.as_str()).unwrap();
                    match path {
                        "camera0/archive/centroids" => {
                            assert!(chunk.timelines().contains_key(&"archive_record_id".into()));
                            raw_packets.push(value);
                        }
                        "camera0/archive/poses" => pose_packets.push(value),
                        "camera0/image/centroids/raw" => {
                            centroid_snapshots.push((time.as_i64(), row_id, value))
                        }
                        "camera0/lftags/raw" => pose_snapshots.push(value),
                        _ => {}
                    }
                }
            }
            if path == "camera0/image/centroids" {
                clears.extend(
                    chunk
                        .iter_component_indices("capture_time".into(), clear_component)
                        .map(|(time, row)| (time.as_i64(), row)),
                );
            }
            if path == "camera0/image/lftags" {
                pose_overlay_clears.extend(
                    chunk
                        .iter_component_indices("capture_time".into(), clear_component)
                        .map(|(time, row)| (time.as_i64(), row)),
                );
            }
            if path == "camera0/image" {
                for batch in chunk.iter_component::<rerun::components::Blob>(blob_component) {
                    for blob in batch.iter() {
                        let decoded = image::load_from_memory_with_format(
                            blob.0.0.as_ref(),
                            image::ImageFormat::Jpeg,
                        )
                        .unwrap();
                        assert_eq!(
                            (decoded.width(), decoded.height()),
                            (WIDTH as u32, HEIGHT as u32)
                        );
                        image_count += 1;
                    }
                }
            }
        }
        raw_packets.sort_by_key(|value| value["archive_record_id"].as_i64());
        assert_eq!(raw_packets.len(), 5);
        assert_eq!(
            raw_packets[..3]
                .iter()
                .map(|value| value["sequence"].as_u64().unwrap())
                .collect::<Vec<_>>(),
            [1, 1, 2]
        );
        assert_eq!(raw_packets[0]["groups"][0]["centroids"][0]["x_q5"], 321);
        assert_eq!(raw_packets[0]["groups"][0]["centroids"][1]["valid"], false);
        assert_eq!(raw_packets[0]["groups"][0]["centroids"][1]["x_q5"], 65535);
        assert_eq!(raw_packets[0]["groups"][0]["centroids"][1]["y_q5"], 12345);
        assert_eq!(raw_packets[3]["frame_id"], 7);
        assert_eq!(raw_packets[3]["datagram_hex"], hex_bytes(&malformed));
        assert!(raw_packets[3]["parse_error"].as_str().is_some());
        assert!(raw_packets[4]["frame_id"].is_null());
        assert_eq!(raw_packets[4]["datagram_hex"], hex_bytes(&wrong_camera));
        assert_eq!(pose_packets.len(), 1);
        assert_eq!(
            pose_packets[0]["datagram_hex"],
            hex_bytes(&pose_packet(7, 3))
        );
        assert!(
            pose_snapshots
                .iter()
                .any(|value| value["detections"][0]["tag_id"] == 123)
        );
        assert_eq!(image_count, 2);
        let final_seven = centroid_snapshots
            .iter()
            .filter(|(time, _, _)| *time == 1000)
            .max_by_key(|(_, row, _)| *row)
            .unwrap();
        assert!(centroid_snapshots.iter().any(|(time, _, value)| {
            *time == 1000 && value["groups"].as_array().unwrap().len() == 2
        }));
        assert_eq!(final_seven.2["invalid"], true);
        assert!(final_seven.2["groups"].as_array().unwrap().is_empty());
        assert!(
            clears
                .iter()
                .filter(|(time, _)| *time == 1000)
                .all(|(_, row)| *row < final_seven.1)
        );
        let final_eight = centroid_snapshots
            .iter()
            .filter(|(time, _, _)| *time == 2000)
            .max_by_key(|(_, row, _)| *row)
            .unwrap();
        assert!(final_eight.2["groups"].as_array().unwrap().is_empty());
        assert!(clears.iter().any(|(time, _)| *time == 2000));
        assert!(pose_overlay_clears.iter().any(|(time, _)| *time == 2000));
    }
}
