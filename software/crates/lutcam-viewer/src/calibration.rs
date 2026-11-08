//! Background, bounded recording of the original compressed camera UDP stream.
//!
//! Each `cameraN.udp` starts with ASCII `LUTCAP01`. Each following record is a
//! little-endian u64 elapsed nanoseconds, u32 datagram length, and those exact
//! datagram bytes. All products are retained before validation. This preserves
//! the existing codec payload without transcoding; it does not make the camera
//! codec itself lossless. `detections.jsonl` is an auxiliary decoded pose log.

use std::collections::HashSet;
use std::ffi::OsString;
use std::fs::{self, File};
use std::io::{self, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, AtomicUsize, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serde::ser::{Serialize, SerializeSeq, Serializer};
use serde_json::{Value, json};

use crate::poses::PoseCache;
use crate::stream::parse_datagram;

pub const RECORDING_MAGIC: &[u8; 8] = b"LUTCAP01";
pub const RECORDING_DURATION: Duration = Duration::from_secs(60);
pub const MAX_RECORDING_DURATION: Duration = Duration::from_secs(86_400);
const QUEUE_BYTES: usize = 32 * 1024 * 1024;
const QUEUE_PACKETS: usize = 8192;
const PACKET_OVERHEAD: usize = 128;
const FRAME_IDS_LIMIT: usize = 16_384;
const POLL: Duration = Duration::from_millis(25);
const FAILURE_OVERFLOW: u8 = 1;
const FAILURE_DISCONNECTED: u8 = 2;
const FAILURE_INVALID: u8 = 3;
const LIFECYCLE_PREPARING: u8 = 0;
const LIFECYCLE_RECORDING: u8 = 1;
const LIFECYCLE_PROCESSING: u8 = 2;
const LIFECYCLE_CANCELLED: u8 = 3;
const LIFECYCLE_FINALIZED: u8 = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecordingStatus {
    Idle,
    Recording,
    Processing,
    Ready,
    Incomplete,
    Error,
}

#[derive(Clone, Debug, Default)]
pub struct RecordingCameraCounts {
    pub packets: u64,
    pub bytes: u64,
    pub image_packets: u64,
    pub pose_packets: u64,
    pub centroid_packets: u64,
    pub centroid_groups: u64,
    /// Distinct valid image frame IDs observed, not successfully decoded frames.
    pub image_frames_observed: u64,
    pub frame_count_saturated: bool,
    pub queue_dropped: u64,
    pub malformed_packets: u64,
    /// Valid records in observed pose packets, including UDP duplicates.
    pub detections: u64,
}

#[derive(Clone, Debug)]
pub struct RecordingSnapshot {
    pub status: RecordingStatus,
    /// False while files are prepared; the timed clip has not started yet.
    pub accepting: bool,
    pub elapsed: Duration,
    /// Requested length of this clip, fixed when recording starts.
    pub duration: Duration,
    pub progress: f32,
    pub directory: Option<PathBuf>,
    pub cameras: [RecordingCameraCounts; 2],
    pub message: String,
    pub error: Option<String>,
    pub calibration_path: Option<PathBuf>,
}

impl Default for RecordingSnapshot {
    fn default() -> Self {
        Self {
            status: RecordingStatus::Idle,
            accepting: false,
            elapsed: Duration::ZERO,
            duration: RECORDING_DURATION,
            progress: 0.0,
            directory: None,
            cameras: Default::default(),
            message: String::new(),
            error: None,
            calibration_path: None,
        }
    }
}

struct Admission {
    start: OnceLock<Instant>,
    duration: Duration,
    accepting: AtomicBool,
    lifecycle: AtomicU8,
    producers: AtomicUsize,
    queued_bytes: AtomicUsize,
    byte_limit: usize,
    failure: AtomicU8,
    dropped: [AtomicU64; 2],
}

impl Admission {
    fn elapsed(&self) -> Duration {
        self.start.get().map_or(Duration::ZERO, Instant::elapsed)
    }

    fn expired(&self) -> bool {
        self.start
            .get()
            .is_some_and(|start| start.elapsed() >= self.duration)
    }

    fn arm(&self) -> bool {
        if self
            .lifecycle
            .compare_exchange(
                LIFECYCLE_PREPARING,
                LIFECYCLE_RECORDING,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_err()
        {
            return false;
        }
        let _ = self.start.set(Instant::now());
        self.accepting.store(true, Ordering::SeqCst);
        if self.lifecycle.load(Ordering::Acquire) != LIFECYCLE_RECORDING {
            self.close();
            return false;
        }
        true
    }

    fn close(&self) {
        self.accepting.store(false, Ordering::SeqCst);
    }

    fn fail(&self, code: u8) {
        let _ = self
            .failure
            .compare_exchange(0, code, Ordering::AcqRel, Ordering::Relaxed);
        self.close();
    }

    fn is_cancelled(&self) -> bool {
        self.lifecycle.load(Ordering::Acquire) == LIFECYCLE_CANCELLED
    }

    fn finish_recording(&self) {
        self.close();
        let _ = self.lifecycle.compare_exchange(
            LIFECYCLE_RECORDING,
            LIFECYCLE_PROCESSING,
            Ordering::AcqRel,
            Ordering::Acquire,
        );
    }

    fn cancel(&self, recording_only: bool) {
        let mut phase = self.lifecycle.load(Ordering::Acquire);
        loop {
            if phase != LIFECYCLE_PREPARING
                && phase != LIFECYCLE_RECORDING
                && (recording_only || phase != LIFECYCLE_PROCESSING)
            {
                return;
            }
            match self.lifecycle.compare_exchange(
                phase,
                LIFECYCLE_CANCELLED,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => {
                    self.close();
                    return;
                }
                Err(current) => phase = current,
            }
        }
    }
}

struct ProducerGuard<'a>(&'a Admission);
impl Drop for ProducerGuard<'_> {
    fn drop(&mut self) {
        self.0.producers.fetch_sub(1, Ordering::SeqCst);
    }
}

#[derive(Debug)]
struct Datagram {
    camera: usize,
    elapsed_ns: u64,
    bytes: Vec<u8>,
    charge: usize,
}

/// Clone into receiver workers. Admission never waits for the writer or takes
/// its mutex. `false` never means that camera reception should stop or retry.
#[derive(Clone)]
pub struct RecordingSink {
    sender: SyncSender<Datagram>,
    admission: Arc<Admission>,
}

impl RecordingSink {
    pub fn try_record(&self, camera: u8, received_at: Instant, datagram: &[u8]) -> bool {
        let admission = &self.admission;
        // Closing admission followed by waiting for this counter prevents a
        // producer that passed the open check from enqueueing after final drain.
        admission.producers.fetch_add(1, Ordering::SeqCst);
        let _producer = ProducerGuard(admission);
        if !admission.accepting.load(Ordering::SeqCst)
            || admission.lifecycle.load(Ordering::Acquire) != LIFECYCLE_RECORDING
        {
            return false;
        }
        let Some(elapsed) = admission
            .start
            .get()
            .and_then(|&start| received_at.checked_duration_since(start))
        else {
            return false;
        };
        if admission.expired() {
            admission.finish_recording();
            return false;
        }
        if elapsed >= admission.duration {
            return false;
        }
        if camera > 1 || datagram.len() > 65_535 {
            admission.fail(FAILURE_INVALID);
            return false;
        }
        let camera = usize::from(camera);
        let charge = datagram.len() + PACKET_OVERHEAD;
        if admission
            .queued_bytes
            .fetch_update(Ordering::AcqRel, Ordering::Relaxed, |current| {
                current
                    .checked_add(charge)
                    .filter(|&next| next <= admission.byte_limit)
            })
            .is_err()
        {
            admission.dropped[camera].fetch_add(1, Ordering::Relaxed);
            admission.fail(FAILURE_OVERFLOW);
            return false;
        }
        let packet = Datagram {
            camera,
            elapsed_ns: elapsed.as_nanos() as u64,
            bytes: datagram.to_vec(),
            charge,
        };
        match self.sender.try_send(packet) {
            Ok(()) => true,
            Err(error) => {
                admission.queued_bytes.fetch_sub(charge, Ordering::AcqRel);
                admission.dropped[camera].fetch_add(1, Ordering::Relaxed);
                admission.fail(match error {
                    TrySendError::Full(_) => FAILURE_OVERFLOW,
                    TrySendError::Disconnected(_) => FAILURE_DISCONNECTED,
                });
                false
            }
        }
    }
}

struct Worker {
    admission: Arc<Admission>,
    thread: JoinHandle<()>,
}

#[derive(Clone)]
struct Options {
    duration: Duration,
    queue_bytes: usize,
    processor: Option<(PathBuf, Vec<OsString>)>,
    #[cfg(test)]
    staged_processor: Option<(PathBuf, Vec<OsString>)>,
    processing_timeout: Duration,
    #[cfg(test)]
    preparation_delay: Duration,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            duration: RECORDING_DURATION,
            queue_bytes: QUEUE_BYTES,
            processor: None,
            #[cfg(test)]
            staged_processor: None,
            processing_timeout: Duration::from_secs(15 * 60),
            #[cfg(test)]
            preparation_delay: Duration::ZERO,
        }
    }
}

/// Disk I/O, clip finalization, extraction and fitting all run on one worker.
/// Dropping requests cancellation without waiting for a slow disk or process.
pub struct Recorder {
    shared: Arc<Mutex<RecordingSnapshot>>,
    worker: Option<Worker>,
    options: Options,
}

impl Default for Recorder {
    fn default() -> Self {
        Self {
            shared: Arc::new(Mutex::new(RecordingSnapshot::default())),
            worker: None,
            options: Options::default(),
        }
    }
}

impl Recorder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn start(&mut self, output_dir: PathBuf, metadata: Value) -> Result<RecordingSink, String> {
        self.start_with_duration(output_dir, metadata, self.options.duration)
    }

    /// Select the length for this recording without changing any active clip.
    pub fn start_with_duration(
        &mut self,
        output_dir: PathBuf,
        metadata: Value,
        duration: Duration,
    ) -> Result<RecordingSink, String> {
        if duration.is_zero() || duration > MAX_RECORDING_DURATION {
            return Err("Recording duration must be positive and at most 86400 seconds".into());
        }
        if self
            .worker
            .as_ref()
            .is_some_and(|worker| !worker.thread.is_finished())
        {
            return Err(
                "A calibration recording, fit, or archive export is already running".into(),
            );
        }
        if let Some(worker) = self.worker.take() {
            let _ = worker.thread.join();
        }
        let (sender, receiver) = mpsc::sync_channel(QUEUE_PACKETS);
        let admission = Arc::new(Admission {
            start: OnceLock::new(),
            duration,
            accepting: AtomicBool::new(false),
            lifecycle: AtomicU8::new(LIFECYCLE_PREPARING),
            producers: AtomicUsize::new(0),
            queued_bytes: AtomicUsize::new(0),
            byte_limit: self.options.queue_bytes,
            failure: AtomicU8::new(0),
            dropped: std::array::from_fn(|_| AtomicU64::new(0)),
        });
        *self.shared.lock().unwrap() = RecordingSnapshot {
            status: RecordingStatus::Recording,
            duration,
            directory: Some(output_dir.clone()),
            message: "Preparing recording files; timed capture has not started".into(),
            ..Default::default()
        };
        let shared = Arc::clone(&self.shared);
        let worker_admission = Arc::clone(&admission);
        let options = Options {
            duration,
            ..self.options.clone()
        };
        let thread = thread::Builder::new()
            .name("calibration-recording".into())
            .spawn(move || {
                run_worker(
                    output_dir,
                    metadata,
                    receiver,
                    worker_admission,
                    shared,
                    options,
                )
            })
            .map_err(|error| {
                admission.close();
                let message = format!("Could not start recording worker: {error}");
                let mut state = self.shared.lock().unwrap();
                state.status = RecordingStatus::Error;
                state.error = Some(message.clone());
                state.message = message.clone();
                message
            })?;
        self.worker = Some(Worker {
            admission: Arc::clone(&admission),
            thread,
        });
        Ok(RecordingSink { sender, admission })
    }

    pub fn snapshot(&self) -> RecordingSnapshot {
        let mut snapshot = self.shared.lock().unwrap().clone();
        if snapshot.status == RecordingStatus::Recording
            && let Some(worker) = &self.worker
        {
            if worker.admission.expired() {
                worker.admission.finish_recording();
            }
            snapshot.elapsed = worker.admission.elapsed().min(worker.admission.duration);
            snapshot.accepting = worker.admission.accepting.load(Ordering::SeqCst)
                && worker.admission.lifecycle.load(Ordering::Acquire) == LIFECYCLE_RECORDING;
            snapshot.progress =
                (snapshot.elapsed.as_secs_f32() / worker.admission.duration.as_secs_f32()).min(1.0);
            for (camera, counts) in snapshot.cameras.iter_mut().enumerate() {
                counts.queue_dropped = worker.admission.dropped[camera].load(Ordering::Relaxed);
            }
            if worker.admission.lifecycle.load(Ordering::Acquire) == LIFECYCLE_PROCESSING {
                snapshot.status = RecordingStatus::Processing;
                snapshot.accepting = false;
                snapshot.message = "Recording complete; finalizing clips before calibration".into();
            }
        }
        snapshot
    }

    pub fn cancel(&self) {
        if let Some(worker) = &self.worker {
            worker.admission.cancel(false);
            if worker.admission.is_cancelled() {
                // Synchronize with early fit publication so cancelling an
                // export cannot leave an applicable, stale result in the UI.
                self.shared.lock().unwrap().calibration_path = None;
            }
        }
    }

    /// Stop an unfinished clip when capture stops or resynchronizes. Unlike
    /// explicit cancellation, this cannot cancel extraction/fitting, even if
    /// the caller saw a stale Recording snapshot at the deadline boundary.
    pub fn cancel_recording(&self) {
        if let Some(worker) = &self.worker {
            if worker.admission.expired() {
                worker.admission.finish_recording();
            }
            worker.admission.cancel(true);
        }
    }

    /// During normal window close, cancel and keep the event loop alive until
    /// this is false. This lets partial files finalize without blocking the UI.
    pub fn is_running(&self) -> bool {
        self.worker
            .as_ref()
            .is_some_and(|worker| !worker.thread.is_finished())
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        self.cancel();
        // Detach: the worker owns all resources until it finalizes partial data
        // and kills/reaps a child if necessary. Never join disk I/O from the UI.
    }
}

struct RecordingFiles {
    clips: [BufWriter<File>; 2],
    detections: BufWriter<File>,
    centroids: BufWriter<File>,
    frame_ids: [HashSet<u32>; 2],
}

/// Borrow the parsed packet directly: the recording hot path must not allocate
/// a JSON object and owned field names for every measured centroid.
#[derive(serde::Serialize)]
struct RecordedCentroids<'a> {
    camera: usize,
    elapsed_ns: u64,
    frame_id: u32,
    groups: RecordedCentroidGroups<'a>,
    sequence: u32,
    version: u8,
}

struct RecordedCentroidGroups<'a>(&'a [crate::centroids::CentroidGroup]);

impl Serialize for RecordedCentroidGroups<'_> {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        #[derive(serde::Serialize)]
        struct Group<'a> {
            centroids: &'a [crate::centroids::Centroid],
            group_index: u8,
        }
        let mut groups = serializer.serialize_seq(Some(self.0.len()))?;
        for group in self.0 {
            groups.serialize_element(&Group {
                centroids: &group.centroids,
                group_index: group.group_index,
            })?;
        }
        groups.end()
    }
}

impl RecordingFiles {
    fn create(directory: &Path) -> io::Result<Self> {
        let mut clips = [
            BufWriter::with_capacity(1024 * 1024, File::create(directory.join("camera0.udp"))?),
            BufWriter::with_capacity(1024 * 1024, File::create(directory.join("camera1.udp"))?),
        ];
        for clip in &mut clips {
            clip.write_all(RECORDING_MAGIC)?;
        }
        fs::write(directory.join("FORMAT.md"), FORMAT_DESCRIPTION)?;
        Ok(Self {
            clips,
            detections: BufWriter::with_capacity(
                1024 * 1024,
                File::create(directory.join("detections.jsonl"))?,
            ),
            centroids: BufWriter::with_capacity(
                1024 * 1024,
                File::create(directory.join("centroids.jsonl"))?,
            ),
            frame_ids: std::array::from_fn(|_| HashSet::new()),
        })
    }

    fn write(&mut self, packet: &Datagram, counts: &mut RecordingCameraCounts) -> io::Result<()> {
        let clip = &mut self.clips[packet.camera];
        clip.write_all(&packet.elapsed_ns.to_le_bytes())?;
        clip.write_all(&(packet.bytes.len() as u32).to_le_bytes())?;
        clip.write_all(&packet.bytes)?;
        counts.packets += 1;
        counts.bytes += packet.bytes.len() as u64;
        let is_pose = packet.bytes.len() >= 8
            && packet.bytes[..4] == [0x4c, 0x55, 0x54, 0x43]
            && packet.bytes[6] == 3;
        let is_centroid = packet.bytes.len() >= 8
            && packet.bytes[..4] == [0x4c, 0x55, 0x54, 0x43]
            && packet.bytes[6] == 2;
        if is_centroid {
            counts.centroid_packets += 1;
            match crate::centroids::parse_datagram(&packet.bytes) {
                Ok(decoded) if usize::from(decoded.camera) == packet.camera => {
                    counts.centroid_groups += decoded.groups.len() as u64;
                    serde_json::to_writer(
                        &mut self.centroids,
                        &RecordedCentroids {
                            camera: packet.camera,
                            elapsed_ns: packet.elapsed_ns,
                            frame_id: decoded.frame_id,
                            groups: RecordedCentroidGroups(&decoded.groups),
                            sequence: decoded.sequence,
                            version: decoded.version,
                        },
                    )?;
                }
                decoded => {
                    counts.malformed_packets += 1;
                    serde_json::to_writer(
                        &mut self.centroids,
                        &json!({
                            "camera": packet.camera, "elapsed_ns": packet.elapsed_ns,
                            "error": decoded.err().map_or_else(|| "Wrong camera in centroid packet".into(), |error| error.to_string()),
                        }),
                    )?;
                }
            }
            self.centroids.write_all(b"\n")?;
        } else if is_pose {
            counts.pose_packets += 1;
            // Parse each original packet independently, retaining duplicate
            // observations in JSONL as well as in the authoritative raw clips.
            let mut cache = PoseCache::new(packet.camera as u8);
            let decoded = cache.push(&packet.bytes, Instant::now());
            let detections: Vec<_> =
                cache
                    .frames()
                    .iter()
                    .flat_map(|frame| {
                        frame.detections.iter().map(|detection| json!({
                    "frame_id": frame.frame_id, "sequence": detection.sequence,
                    "record_index": detection.record_index, "tag_id": detection.tag_id,
                    "scale": detection.scale,
                    "hypothesis": detection.hypothesis,
                    "best_score_q32": detection.best_score_q32,
                    "rotation_q29": detection.pose.pose.rotation_q29,
                    "translation_q16": detection.pose.pose.translation_q16,
                    "weak_perspective": detection.pose.weak_perspective,
                    "conditioning_warning": detection.pose.conditioning_warning,
                    "saturation": detection.pose.saturation,
                })).collect::<Vec<_>>()
                    })
                    .collect();
            counts.detections += detections.len() as u64;
            if decoded.is_err() {
                counts.malformed_packets += 1;
            }
            serde_json::to_writer(
                &mut self.detections,
                &json!({
                    "camera": packet.camera, "elapsed_ns": packet.elapsed_ns,
                    "detections": detections, "error": decoded.err().map(|error| error.to_string()),
                }),
            )?;
            self.detections.write_all(b"\n")?;
        } else if let Ok(image) = parse_datagram(&packet.bytes) {
            if usize::from(image.camera) != packet.camera {
                counts.malformed_packets += 1;
            } else {
                counts.image_packets += 1;
                let ids = &mut self.frame_ids[packet.camera];
                if ids.contains(&image.frame_id) || ids.len() < FRAME_IDS_LIMIT {
                    ids.insert(image.frame_id);
                    counts.image_frames_observed = ids.len() as u64;
                } else {
                    counts.frame_count_saturated = true;
                }
            }
        } else {
            counts.malformed_packets += 1;
        }
        Ok(())
    }

    fn finish(&mut self) -> io::Result<()> {
        for clip in &mut self.clips {
            clip.flush()?;
            clip.get_ref().sync_all()?;
        }
        self.detections.flush()?;
        self.detections.get_ref().sync_all()?;
        self.centroids.flush()?;
        self.centroids.get_ref().sync_all()
    }
}

const FORMAT_DESCRIPTION: &str = "# LUTCam UDP recording v1\n\nEach camera0.udp / camera1.udp starts with the 8 ASCII bytes LUTCAP01. Each following record is a little-endian uint64 elapsed nanoseconds since recording start, a little-endian uint32 payload length, and that many original UDP payload bytes. Timestamps are receive times from one monotonic host clock. Records follow writer admission order; compare timestamps when merging cameras. No extra padding, checksum, or compression is added. The original camera codec remains compressed and unchanged; recording is byte-lossless for admitted datagrams, not a claim that the image codec is lossless.\n\nImage, pose, other products, duplicates, and malformed source-accepted packets are retained. Parse the original big-endian LUTCam header to distinguish products. A partial final record indicates incomplete data. detections.jsonl is an auxiliary human-readable decode of each pose datagram; centroids.jsonl preserves each FPGA centroid group, tuple order, raw Q5 coordinates and validity. Centroid scale is not carried as a separate wire field: the extractor infers RAW, LL2, or LL4 from the fixed composite coordinate ranges of the FPGA pipeline. Raw Q5 values are preserved and are not automatically full-resolution pixel coordinates. Duplicate observations are retained. Raw clips are authoritative.\n\nmanifest.json records completion, queue loss, observed frame IDs, and processing state. image_frames_observed is not a count of complete decoded images. Network losses and frame completeness must be checked by the extractor. A successful 60-second recording still requires adequate calibration observations. Failed or cancelled recordings preserve their partial files and never replace an existing calibration.\n";

fn publish(shared: &Mutex<RecordingSnapshot>, snapshot: &RecordingSnapshot) {
    *shared.lock().unwrap() = snapshot.clone();
}

fn write_manifest(
    directory: &Path,
    metadata: &Value,
    state: &str,
    complete: bool,
    snapshot: &RecordingSnapshot,
    duration: Duration,
) -> io::Result<()> {
    let cameras: Vec<_> = snapshot.cameras.iter().enumerate().map(|(camera, counts)| json!({
        "camera": camera, "packets": counts.packets, "bytes": counts.bytes,
        "image_packets": counts.image_packets, "pose_packets": counts.pose_packets,
        "centroid_packets": counts.centroid_packets, "centroid_groups": counts.centroid_groups,
        "image_frames_observed": counts.image_frames_observed,
        "frame_count_saturated": counts.frame_count_saturated,
        "queue_dropped": counts.queue_dropped, "malformed_packets": counts.malformed_packets,
        "detections": counts.detections,
    })).collect();
    let manifest = json!({
        "format": "lutcam-udp-v1", "state": state, "complete": complete,
        "duration_seconds": duration.as_secs_f64(), "elapsed_seconds": snapshot.elapsed.as_secs_f64(),
        "metadata": metadata, "cameras": cameras, "message": snapshot.message,
        "error": snapshot.error, "calibration_path": snapshot.calibration_path,
    });
    let temporary = directory.join("manifest.json.tmp");
    let mut file = File::create(&temporary)?;
    serde_json::to_writer_pretty(&mut file, &manifest)?;
    file.write_all(b"\n")?;
    file.sync_all()?;
    fs::rename(temporary, directory.join("manifest.json"))
}

fn run_worker(
    directory: PathBuf,
    mut metadata: Value,
    receiver: Receiver<Datagram>,
    admission: Arc<Admission>,
    shared: Arc<Mutex<RecordingSnapshot>>,
    options: Options,
) {
    let mut snapshot = shared.lock().unwrap().clone();
    if let Some(object) = metadata.as_object_mut() {
        object.entry("baseline_mm").or_insert(json!(120.0));
    }
    let create_directory = || -> io::Result<()> {
        if let Some(parent) = directory
            .parent()
            .filter(|path| !path.as_os_str().is_empty())
        {
            fs::create_dir_all(parent)?;
        }
        // Never append to or replace an existing recording/calibration.
        fs::create_dir(&directory)
    };
    if let Err(error) = create_directory() {
        admission.close();
        snapshot.status = RecordingStatus::Error;
        snapshot.message = format!("Could not create recording directory: {error}");
        snapshot.error = Some(snapshot.message.clone());
        publish(&shared, &snapshot);
        return;
    }
    let mut files = match RecordingFiles::create(&directory) {
        Ok(files) => files,
        Err(error) => {
            admission.close();
            snapshot.status = RecordingStatus::Error;
            snapshot.message = format!("Could not create recording: {error}");
            snapshot.error = Some(snapshot.message.clone());
            let _ = write_manifest(
                &directory,
                &metadata,
                "incomplete",
                false,
                &snapshot,
                options.duration,
            );
            publish(&shared, &snapshot);
            return;
        }
    };
    #[cfg(test)]
    thread::sleep(options.preparation_delay);
    let mut disk_error = write_manifest(
        &directory,
        &metadata,
        "preparing",
        false,
        &snapshot,
        options.duration,
    )
    .err();
    if disk_error.is_none() && admission.arm() {
        if let Some(object) = metadata.as_object_mut() {
            object.insert(
                "started_unix_ms".into(),
                json!(
                    SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_millis() as u64
                ),
            );
        }
        snapshot.accepting = true;
        snapshot.message = "Recording both cameras and LFTag packets".into();
        publish(&shared, &snapshot);
    }
    let mut last_publish = Instant::now();
    loop {
        if admission.expired() {
            admission.finish_recording();
        }
        if disk_error.is_some() {
            admission.close();
        }
        if snapshot.status == RecordingStatus::Recording
            && admission.lifecycle.load(Ordering::Acquire) == LIFECYCLE_PROCESSING
        {
            snapshot.status = RecordingStatus::Processing;
            snapshot.accepting = false;
            snapshot.elapsed = options.duration;
            snapshot.progress = 1.0;
            snapshot.message = "Recording complete; finalizing clips before calibration".into();
            publish(&shared, &snapshot);
        }
        let closed = !admission.accepting.load(Ordering::SeqCst);
        // Check in-flight producers before the final empty observation. A
        // producer entering after this check sees closed and cannot enqueue.
        let packet = if closed && admission.producers.load(Ordering::SeqCst) == 0 {
            match receiver.try_recv() {
                Ok(packet) => Some(packet),
                Err(TryRecvError::Empty | TryRecvError::Disconnected) => break,
            }
        } else {
            match receiver.recv_timeout(POLL) {
                Ok(packet) => Some(packet),
                Err(RecvTimeoutError::Timeout) => None,
                Err(RecvTimeoutError::Disconnected) => {
                    // Detaching the last tap is expected after a cancellation
                    // or deadline. The worker may already be inside recv when
                    // another thread closes admission and drops the senders.
                    if admission.expired() {
                        admission.finish_recording();
                    } else if admission.lifecycle.load(Ordering::Acquire) == LIFECYCLE_RECORDING
                        && admission.accepting.load(Ordering::SeqCst)
                    {
                        admission.fail(FAILURE_DISCONNECTED);
                    }
                    if admission.producers.load(Ordering::SeqCst) != 0 {
                        thread::sleep(Duration::from_millis(1));
                    }
                    None
                }
            }
        };
        if let Some(packet) = packet {
            if disk_error.is_none() {
                if let Err(error) = files.write(&packet, &mut snapshot.cameras[packet.camera]) {
                    disk_error = Some(error);
                    admission.dropped[packet.camera].fetch_add(1, Ordering::Relaxed);
                }
            } else {
                admission.dropped[packet.camera].fetch_add(1, Ordering::Relaxed);
            }
            admission
                .queued_bytes
                .fetch_sub(packet.charge, Ordering::AcqRel);
        }
        if last_publish.elapsed() >= Duration::from_millis(100) {
            snapshot.elapsed = admission.elapsed().min(options.duration);
            snapshot.progress =
                (snapshot.elapsed.as_secs_f32() / options.duration.as_secs_f32()).min(1.0);
            publish(&shared, &snapshot);
            last_publish = Instant::now();
        }
    }
    if let Err(error) = files.finish() {
        disk_error.get_or_insert(error);
    }
    snapshot.accepting = false;
    snapshot.elapsed = admission.elapsed().min(options.duration);
    snapshot.progress = (snapshot.elapsed.as_secs_f32() / options.duration.as_secs_f32()).min(1.0);
    for (camera, counts) in snapshot.cameras.iter_mut().enumerate() {
        counts.queue_dropped = admission.dropped[camera].load(Ordering::Relaxed);
    }
    let failure = admission.failure.load(Ordering::Acquire);
    let reason = if let Some(error) = disk_error {
        Some(format!("Recording disk error: {error}"))
    } else if failure != 0 {
        Some(
            match failure {
                FAILURE_OVERFLOW => "Recording queue overflowed; partial data was preserved",
                FAILURE_DISCONNECTED => "Recording writer disconnected",
                _ => "Invalid camera index or oversized recording datagram",
            }
            .to_owned(),
        )
    } else if admission.is_cancelled() {
        Some("Recording cancelled; partial data was preserved".into())
    } else if snapshot
        .cameras
        .iter()
        .any(|counts| counts.image_packets == 0)
    {
        Some("Recording incomplete: both cameras must deliver image packets".into())
    } else {
        None
    };
    if let Some(reason) = reason {
        snapshot.status = RecordingStatus::Incomplete;
        snapshot.message = reason.clone();
        snapshot.error = Some(reason);
        if let Err(error) = write_manifest(
            &directory,
            &metadata,
            "incomplete",
            false,
            &snapshot,
            options.duration,
        ) {
            snapshot
                .message
                .push_str(&format!("; could not save manifest: {error}"));
            snapshot.error = Some(snapshot.message.clone());
        }
        publish(&shared, &snapshot);
        return;
    }
    snapshot.status = RecordingStatus::Processing;
    snapshot.progress = 1.0;
    snapshot.message = "Recording complete; extracting observations and fitting calibration".into();
    if let Err(error) = write_manifest(
        &directory,
        &metadata,
        "completed",
        true,
        &snapshot,
        options.duration,
    ) {
        snapshot.status = RecordingStatus::Error;
        snapshot.message = format!("Could not finalize recording manifest: {error}");
        snapshot.error = Some(snapshot.message.clone());
        publish(&shared, &snapshot);
        return;
    }
    publish(&shared, &snapshot);
    let result = run_processor(&directory, &admission, &options, |fit| {
        snapshot.status = RecordingStatus::Processing;
        match fit {
            Ok(path) => {
                snapshot.calibration_path = Some(path.clone());
                snapshot.error = None;
                snapshot.message = "Calibration ready; saving centroid archive for Rerun".into();
            }
            Err(error) => {
                snapshot.calibration_path = None;
                snapshot.error = Some(error.clone());
                snapshot.message =
                    "Calibration fit failed; saving centroid archive for Rerun".into();
            }
        }
        let mut published = shared.lock().unwrap();
        if !admission.is_cancelled() {
            *published = snapshot.clone();
        }
    });
    match result {
        Ok(result) => {
            snapshot.status = RecordingStatus::Ready;
            snapshot.calibration_path = Some(result.path);
            snapshot.error = result.archive_error;
            snapshot.message = if snapshot.error.is_some() {
                "Calibration is ready to apply; recording export failed".into()
            } else {
                "Calibration is ready to review and apply".into()
            };
        }
        Err(error) => {
            snapshot.status = if admission.is_cancelled() {
                RecordingStatus::Incomplete
            } else {
                RecordingStatus::Error
            };
            snapshot.calibration_path = None;
            snapshot.message = error.clone();
            snapshot.error = Some(error);
        }
    }
    // Commit the outcome atomically against cancellation. A cancel accepted
    // before this transition must never publish a ready calibration, including
    // cancellation while the processor's successful JSON output was read.
    if admission
        .lifecycle
        .compare_exchange(
            LIFECYCLE_PROCESSING,
            LIFECYCLE_FINALIZED,
            Ordering::AcqRel,
            Ordering::Acquire,
        )
        .is_err()
    {
        snapshot.status = RecordingStatus::Incomplete;
        snapshot.calibration_path = None;
        snapshot.message = "Calibration cancelled; recording was preserved".into();
        snapshot.error = Some(snapshot.message.clone());
    }
    let state = if snapshot.status == RecordingStatus::Ready {
        "ready"
    } else {
        "processing_failed"
    };
    if let Err(error) = write_manifest(
        &directory,
        &metadata,
        state,
        true,
        &snapshot,
        options.duration,
    ) {
        snapshot.status = RecordingStatus::Error;
        snapshot.calibration_path = None;
        snapshot.message = format!("Could not save calibration result status: {error}");
        snapshot.error = Some(snapshot.message.clone());
    }
    publish(&shared, &snapshot);
}

struct ProcessorResult {
    path: PathBuf,
    archive_error: Option<String>,
}

fn run_processor(
    directory: &Path,
    admission: &Admission,
    options: &Options,
    mut publish_fit: impl FnMut(&Result<PathBuf, String>),
) -> Result<ProcessorResult, String> {
    // Explicit processor overrides retain their single-command contract.
    if let Some((program, arguments)) = &options.processor {
        run_processor_stage(directory, admission, options, program, arguments, false)?;
        return Ok(ProcessorResult {
            path: read_calibration_output(directory)?,
            archive_error: None,
        });
    }
    let command = (
        std::env::current_exe()
            .map_err(|error| error.to_string())?
            .with_file_name("lutcam-calibrate"),
        Vec::new(),
    );
    #[cfg(test)]
    let command = options.staged_processor.clone().unwrap_or(command);
    let (program, mut arguments) = command;
    arguments.push("--fit-only".into());
    let fit = run_processor_stage(directory, admission, options, &program, &arguments, false)
        .and_then(|()| read_calibration_output(directory));
    if admission.is_cancelled() {
        return Err("Calibration cancelled; recording was preserved".into());
    }
    publish_fit(&fit);
    arguments.pop();
    arguments.push("--metadata-only".into());
    // Preserve the centroid archive even if the selected observations
    // cannot support a fit. A failed export cannot invalidate a successful fit.
    let archive = run_processor_stage(directory, admission, options, &program, &arguments, true);
    if admission.is_cancelled() {
        return Err("Calibration cancelled; recording was preserved".into());
    }
    match fit {
        Ok(path) => Ok(ProcessorResult {
            path,
            archive_error: archive.err(),
        }),
        Err(error) => Err(match archive {
            Ok(()) => error,
            Err(archive_error) => format!("{error}; {archive_error}"),
        }),
    }
}

fn read_calibration_output(directory: &Path) -> Result<PathBuf, String> {
    let path = directory.join("calibration.json");
    let result: Value = serde_json::from_reader(
        File::open(&path).map_err(|error| format!("Calibration output is missing: {error}"))?,
    )
    .map_err(|error| format!("Calibration output is invalid JSON: {error}"))?;
    if !result.is_object() {
        return Err("Calibration output must be a JSON object".into());
    }
    Ok(path)
}

fn run_processor_stage(
    directory: &Path,
    admission: &Admission,
    options: &Options,
    program: &Path,
    arguments: &[OsString],
    archive: bool,
) -> Result<(), String> {
    if admission.is_cancelled() {
        return Err("Calibration cancelled; recording was preserved".into());
    }
    let (stage, log_name) = if archive {
        ("Recording export", "archive.log")
    } else {
        ("Calibration fitting", "calibration.log")
    };
    let log = File::create(directory.join(log_name)).map_err(|error| error.to_string())?;
    let mut command = Command::new(program);
    command
        .args(arguments)
        .arg("--recording")
        .arg(directory)
        .stdin(Stdio::null())
        .stdout(log.try_clone().map_err(|error| error.to_string())?)
        .stderr(log);
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt;
        command.process_group(0);
    }
    let mut child = command
        .spawn()
        .map_err(|error| format!("Could not start {}: {error}", program.display()))?;
    let started = Instant::now();
    loop {
        if admission.is_cancelled() || started.elapsed() >= options.processing_timeout {
            stop_processor(&mut child);
            return Err(if admission.is_cancelled() {
                "Calibration cancelled; recording was preserved".into()
            } else {
                format!("{stage} timed out; recording was preserved")
            });
        }
        match child.try_wait() {
            Ok(Some(status)) => {
                // Kill remaining descendants even after a successful leader
                // exit, before validating or publishing its output.
                stop_processor(&mut child);
                return if status.success() {
                    Ok(())
                } else {
                    let detail = processor_error_detail(&directory.join(log_name))
                        .map(|reason| format!(": {reason}"))
                        .unwrap_or_default();
                    Err(format!("{stage} failed ({status}){detail}; see {log_name}"))
                };
            }
            Ok(None) => thread::sleep(POLL),
            Err(error) => {
                stop_processor(&mut child);
                return Err(format!("Could not monitor {stage}: {error}"));
            }
        }
    }
}

/// The fitter emits a structured error before its launcher reports the exit
/// status. Surface that actionable reason without loading a whole replay log.
fn processor_error_detail(path: &Path) -> Option<String> {
    let mut file = File::open(path).ok()?;
    let length = file.metadata().ok()?.len();
    file.seek(SeekFrom::Start(length.saturating_sub(65_536)))
        .ok()?;
    let mut tail = Vec::new();
    file.take(65_536).read_to_end(&mut tail).ok()?;
    String::from_utf8_lossy(&tail)
        .lines()
        .rev()
        .find_map(|line| {
            let value: Value = serde_json::from_str(line).ok()?;
            if value.get("ok")?.as_bool()? {
                return None;
            }
            let reason = value.get("error")?.as_str()?.trim();
            (!reason.is_empty()).then(|| reason.chars().take(512).collect())
        })
}

fn stop_processor(child: &mut std::process::Child) {
    #[cfg(unix)]
    // The processor owns a fresh process group. Kill its Python fitter too,
    // rather than leaving it writing a supposedly cancelled calibration.
    unsafe {
        libc::kill(-(child.id() as i32), libc::SIGKILL);
    }
    let _ = child.kill();
    let _ = child.wait();
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestDirectory(PathBuf);
    impl TestDirectory {
        fn new() -> Self {
            static NEXT: AtomicU64 = AtomicU64::new(0);
            let stamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos();
            let path = std::env::temp_dir().join(format!(
                "lutcam-recording-{}-{stamp}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            fs::create_dir(&path).unwrap();
            Self(path)
        }
    }
    impl Drop for TestDirectory {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    fn recorder(root: &Path, script: &str) -> Recorder {
        let processor = root.join("processor.sh");
        fs::write(&processor, script).unwrap();
        Recorder {
            shared: Arc::new(Mutex::new(RecordingSnapshot::default())),
            worker: None,
            options: Options {
                duration: Duration::from_millis(60),
                processor: Some((PathBuf::from("/bin/sh"), vec![processor.into_os_string()])),
                processing_timeout: Duration::from_secs(2),
                ..Default::default()
            },
        }
    }

    fn staged_recorder(root: &Path, script: &str) -> Recorder {
        let mut recorder = recorder(root, script);
        recorder.options.staged_processor = recorder.options.processor.take();
        recorder
    }

    fn wait_for_file(path: &Path) {
        let deadline = Instant::now() + Duration::from_secs(3);
        while !path.exists() {
            assert!(
                Instant::now() < deadline,
                "{} was not created",
                path.display()
            );
            thread::sleep(Duration::from_millis(5));
        }
    }

    fn packet(camera: u8, product: u8, frame: u32, payload: &[u8]) -> Vec<u8> {
        let mut bytes = Vec::new();
        for word in [
            0x4c55_5443,
            u32::from_be_bytes([0, camera, product, 32]),
            frame,
            17,
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

    fn images(sink: &RecordingSink) {
        wait_until_armed(sink);
        for camera in 0..2 {
            assert!(sink.try_record(camera, Instant::now(), &packet(camera, 1, 42, &[1, 2])));
        }
    }

    fn wait_until_armed(sink: &RecordingSink) {
        let deadline = Instant::now() + Duration::from_secs(2);
        while !sink.admission.accepting.load(Ordering::SeqCst) {
            assert!(Instant::now() < deadline, "recording never armed");
            assert_eq!(sink.admission.failure.load(Ordering::Acquire), 0);
            assert!(!sink.admission.is_cancelled());
            thread::sleep(Duration::from_millis(2));
        }
    }

    fn finished(recorder: &Recorder) -> RecordingSnapshot {
        let deadline = Instant::now() + Duration::from_secs(4);
        loop {
            let state = recorder.snapshot();
            if !matches!(
                state.status,
                RecordingStatus::Recording | RecordingStatus::Processing
            ) {
                return state;
            }
            assert!(
                Instant::now() < deadline,
                "recorder did not finish: {state:?}"
            );
            thread::sleep(Duration::from_millis(5));
        }
    }

    fn records(path: &Path) -> Vec<(u64, Vec<u8>)> {
        let bytes = fs::read(path).unwrap();
        assert_eq!(&bytes[..8], RECORDING_MAGIC);
        let mut offset = 8;
        let mut records = Vec::new();
        while offset < bytes.len() {
            let elapsed = u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap());
            let length =
                u32::from_le_bytes(bytes[offset + 8..offset + 12].try_into().unwrap()) as usize;
            offset += 12;
            records.push((elapsed, bytes[offset..offset + length].to_vec()));
            offset += length;
        }
        records
    }

    // Use an owned Value-based representation as a format oracle. The
    // production writer must retain every field/type/tuple in its faster path.
    fn owned_centroid_json(packet: &Datagram) -> Value {
        let decoded = crate::centroids::parse_datagram(&packet.bytes).unwrap();
        json!({
            "camera": packet.camera, "elapsed_ns": packet.elapsed_ns,
            "frame_id": decoded.frame_id, "sequence": decoded.sequence,
            "version": decoded.version,
            "groups": decoded.groups.iter().map(|group| json!({
                "group_index": group.group_index,
                "centroids": group.centroids.iter().map(|point| json!({
                    "valid": point.valid, "x_q5": point.x_q5, "y_q5": point.y_q5,
                })).collect::<Vec<_>>(),
            })).collect::<Vec<_>>(),
        })
    }

    #[test]
    fn centroid_json_preserves_all_batch_sizes_and_duplicates() {
        let temporary = TestDirectory::new();
        let mut files = RecordingFiles::create(&temporary.0).unwrap();
        let mut counts: [RecordingCameraCounts; 2] = Default::default();
        let mut expected = Vec::new();
        let mut expected_raw: [Vec<(u64, Vec<u8>)>; 2] = Default::default();
        for tuple_count in [9, 18, 27, 36] {
            let tuples: Vec<_> = (0..tuple_count)
                .flat_map(|index| {
                    let x = if index % 2 == 0 { u16::MAX } else { index };
                    let y = u16::MAX - index;
                    // Invalid flags do not discard coordinates or an entirely
                    // invalid group from this authoritative observation log.
                    let valid = tuple_count != 9 && index % 3 != 0;
                    ((u64::from(y) << 17) | (u64::from(x) << 1) | u64::from(valid)).to_be_bytes()
                })
                .collect();
            for camera in 0..2 {
                let mut bytes = packet(camera as u8, 2, u32::MAX, &tuples);
                bytes[12..16].copy_from_slice(&u32::MAX.to_be_bytes());
                for elapsed_ns in [0, u64::MAX] {
                    let datagram = Datagram {
                        camera,
                        elapsed_ns,
                        charge: 0,
                        bytes: bytes.clone(),
                    };
                    let owned = owned_centroid_json(&datagram);
                    serde_json::to_writer(&mut expected, &owned).unwrap();
                    expected.push(b'\n');
                    files.write(&datagram, &mut counts[camera]).unwrap();
                    expected_raw[camera].push((elapsed_ns, bytes.clone()));
                }
            }
        }
        files.finish().unwrap();
        let actual = fs::read(temporary.0.join("centroids.jsonl")).unwrap();
        assert_eq!(
            actual, expected,
            "typed output must preserve the original JSONL exactly"
        );
        for (camera, count) in counts.iter().enumerate() {
            assert_eq!(count.centroid_packets, 8);
            assert_eq!(count.centroid_groups, 20);
            assert_eq!(count.malformed_packets, 0);
            assert_eq!(
                records(&temporary.0.join(format!("camera{camera}.udp"))),
                expected_raw[camera]
            );
        }
    }

    #[test]
    fn centroid_json_rejection_paths_preserve_errors_and_original_datagrams() {
        let temporary = TestDirectory::new();
        let mut files = RecordingFiles::create(&temporary.0).unwrap();
        let mut counts = RecordingCameraCounts::default();
        let valid = packet(0, 2, 42, &[0; 72]);
        let mut reserved_tuple = valid.clone();
        reserved_tuple[32..40].copy_from_slice(&(1u64 << 33).to_be_bytes());
        let mut reserved_header = valid.clone();
        reserved_header[31] = 1;
        let mut wrong_camera = valid;
        wrong_camera[5] = 1;
        let cases = [
            (
                reserved_tuple,
                "centroid tuple is not a zero-extended 33-bit value",
            ),
            (
                reserved_header,
                "nonzero centroid offset or reserved header fields",
            ),
            (wrong_camera, "Wrong camera in centroid packet"),
            (
                packet(0, 2, 42, &[0; 64]),
                "centroid batch must contain one through four complete nine-tuple groups",
            ),
        ];
        let mut expected = Vec::new();
        let mut expected_raw = Vec::new();
        for (bytes, error) in cases {
            for elapsed_ns in [23, 24] {
                let datagram = Datagram {
                    camera: 0,
                    elapsed_ns,
                    charge: 0,
                    bytes: bytes.clone(),
                };
                files.write(&datagram, &mut counts).unwrap();
                expected.push(json!({ "camera": 0, "elapsed_ns": elapsed_ns, "error": error }));
                expected_raw.push((elapsed_ns, bytes.clone()));
            }
        }
        files.finish().unwrap();
        let actual: Vec<Value> = fs::read_to_string(temporary.0.join("centroids.jsonl"))
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect();
        assert_eq!(actual, expected);
        assert_eq!(counts.centroid_packets, 8);
        assert_eq!(counts.centroid_groups, 0);
        assert_eq!(counts.malformed_packets, 8);
        assert_eq!(records(&temporary.0.join("camera0.udp")), expected_raw);
    }

    #[test]
    fn selected_duration_controls_admission_progress_and_manifest() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("selected-duration");
        let mut recorder = recorder(
            &temporary.0,
            "printf '{\"baseline_mm\":120}' > \"$2/calibration.json\"\n",
        );
        let duration = Duration::from_millis(250);
        let sink = recorder
            .start_with_duration(directory.clone(), json!({}), duration)
            .unwrap();
        images(&sink);
        // The old default for this test recorder is 60 ms. The selected clip
        // must still accept packets after that deadline.
        thread::sleep(Duration::from_millis(80));
        assert!(sink.try_record(0, Instant::now(), &packet(0, 1, 43, &[3, 4])));
        let state = recorder.snapshot();
        assert_eq!(state.duration, duration);
        assert!(state.progress > 0.0 && state.progress < 1.0);
        assert!(
            recorder
                .start_with_duration(temporary.0.join("busy"), json!({}), Duration::from_secs(5))
                .is_err()
        );
        assert_eq!(recorder.snapshot().duration, duration);
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Ready, "{state:?}");
        assert_eq!(state.duration, duration);
        assert_eq!(state.elapsed, duration);
        assert!(!sink.try_record(0, Instant::now(), b"late"));
        let manifest: Value =
            serde_json::from_slice(&fs::read(directory.join("manifest.json")).unwrap()).unwrap();
        assert_eq!(manifest["duration_seconds"], 0.25);
        assert_eq!(manifest["elapsed_seconds"], 0.25);
    }

    #[test]
    fn invalid_duration_does_not_start_a_recording() {
        let temporary = TestDirectory::new();
        let mut recorder = Recorder::new();
        for duration in [
            Duration::ZERO,
            MAX_RECORDING_DURATION + Duration::from_secs(1),
        ] {
            assert!(
                recorder
                    .start_with_duration(temporary.0.join("invalid"), json!({}), duration)
                    .is_err()
            );
            assert_eq!(recorder.snapshot().status, RecordingStatus::Idle);
            assert!(!temporary.0.join("invalid").exists());
        }
    }

    #[test]
    fn complete_clip_preserves_all_products_and_runs_processor() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("clip");
        let mut recorder = recorder(
            &temporary.0,
            "printf '{\"baseline_mm\":120}' > \"$2/calibration.json\"\n",
        );
        let sink = recorder
            .start(directory.clone(), json!({"test": true}))
            .unwrap();
        images(&sink);
        let tuples: Vec<_> = (0u64..9)
            .flat_map(|index| ((index << 17) | (index << 1) | (index % 2)).to_be_bytes())
            .collect();
        let centroids = packet(0, 2, 42, &tuples);
        assert!(sink.try_record(0, Instant::now(), &centroids));
        let mut words = [0u32; 22];
        words[0] = 7;
        words[1] = 0x8000_0000 | (42 << 20);
        words[8] = 0x0000_0001;
        let pose = packet(
            1,
            3,
            42,
            &words
                .into_iter()
                .flat_map(u32::to_be_bytes)
                .collect::<Vec<_>>(),
        );
        assert!(sink.try_record(1, Instant::now(), &pose));
        let malformed = b"original malformed UDP";
        assert!(sink.try_record(1, Instant::now(), malformed));
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Ready, "{state:?}");
        assert_eq!(
            state.calibration_path,
            Some(directory.join("calibration.json"))
        );
        assert_eq!(state.cameras[0].centroid_groups, 1);
        assert_eq!(state.cameras[1].detections, 1);
        assert_eq!(state.cameras[1].malformed_packets, 1);
        let first = records(&directory.join("camera0.udp"));
        assert_eq!(first.len(), 2);
        assert_eq!(first[1].1, centroids);
        assert!(first.iter().all(|(ns, _)| *ns < 60_000_000));
        let second = records(&directory.join("camera1.udp"));
        assert_eq!(second[1].1, pose);
        assert_eq!(second[2].1, malformed);
        let manifest: Value =
            serde_json::from_slice(&fs::read(directory.join("manifest.json")).unwrap()).unwrap();
        assert_eq!(manifest["complete"], true);
        assert_eq!(manifest["state"], "ready");
        assert_eq!(manifest["metadata"]["baseline_mm"], 120.0);
        let centroid: Value =
            serde_json::from_str(&fs::read_to_string(directory.join("centroids.jsonl")).unwrap())
                .unwrap();
        assert_eq!(centroid["groups"][0]["centroids"][0]["valid"], false);
        assert_eq!(centroid["groups"][0]["centroids"][8]["x_q5"], 8);
        let detection_log = fs::read_to_string(directory.join("detections.jsonl")).unwrap();
        assert_eq!(detection_log.lines().count(), 1);
        let detection: Value = serde_json::from_str(&detection_log).unwrap();
        assert_eq!(detection["detections"][0]["tag_id"], 7);
        assert!(detection["detections"][0].get("overflow").is_none());
        assert!(!sink.try_record(0, Instant::now(), b"late"));
    }

    #[test]
    fn deadline_finalizes_without_traffic_and_never_processes() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("idle");
        let mut recorder = recorder(&temporary.0, "touch \"$2/should-not-run\"\n");
        let _sink = recorder.start(directory.clone(), json!({})).unwrap();
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Incomplete);
        assert_eq!(state.cameras[0].packets, 0);
        assert_eq!(state.progress, 1.0);
        assert!(records(&directory.join("camera0.udp")).is_empty());
        assert!(!directory.join("should-not-run").exists());
        assert!(directory.join("manifest.json").exists());
    }

    #[test]
    fn preparation_longer_than_packet_queue_horizon_does_not_admit_or_overflow() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("slow-prepare");
        let mut recorder = recorder(
            &temporary.0,
            "printf '{\"baseline_mm\":120}' > \"$2/calibration.json\"\n",
        );
        // At the observed 38k packets/s, the unchanged 8192-packet queue
        // represents about 215ms. File preparation may take longer than that.
        recorder.options.preparation_delay = Duration::from_millis(350);
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        let before_arm = Instant::now();
        for _ in 0..(QUEUE_PACKETS * 3) {
            assert!(!sink.try_record(0, before_arm, &[0; 1000]));
        }
        let state = recorder.snapshot();
        assert!(!state.accepting);
        assert_eq!(state.elapsed, Duration::ZERO);
        assert_eq!(sink.admission.queued_bytes.load(Ordering::Acquire), 0);
        assert_eq!(sink.admission.dropped[0].load(Ordering::Acquire), 0);
        images(&sink);
        assert!(!sink.try_record(0, before_arm, b"received before clip start"));
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Ready, "{state:?}");
        assert_eq!(state.elapsed, recorder.options.duration);
        assert_eq!(records(&directory.join("camera0.udp")).len(), 1);
        assert!(state.cameras.iter().all(|counts| counts.queue_dropped == 0));
    }

    #[test]
    fn cancellation_during_preparation_never_arms_the_clip() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("cancel-prepare");
        let mut recorder = recorder(&temporary.0, "exit 99\n");
        recorder.options.preparation_delay = Duration::from_millis(80);
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        assert!(!recorder.snapshot().accepting);
        recorder.cancel_recording();
        assert!(!sink.try_record(0, Instant::now(), b"cancelled"));
        drop(sink);
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Incomplete);
        assert_eq!(state.elapsed, Duration::ZERO);
        assert!(!state.accepting);
        assert_eq!(state.cameras[0].packets, 0);
        assert!(state.error.as_deref().unwrap().contains("cancelled"));
        assert!(!directory.join("calibration.log").exists());
    }

    #[test]
    fn natural_deadline_detach_and_capture_stop_do_not_cancel_processing() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("natural-finish");
        let mut recorder = recorder(
            &temporary.0,
            "touch \"$2/processing\"\nwhile [ ! -f \"$2/finish\" ]; do sleep 0.01; done\nprintf '{\"baseline_mm\":120}' > \"$2/calibration.json\"\n",
        );
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        images(&sink);
        assert_eq!(recorder.snapshot().status, RecordingStatus::Recording);
        let admission = Arc::clone(&sink.admission);
        // Hold a producer across the deadline to force GUI detachment while
        // the worker is still draining, before it can start the processor.
        admission.producers.fetch_add(1, Ordering::SeqCst);
        let deadline = Instant::now() + Duration::from_secs(2);
        while recorder.snapshot().status != RecordingStatus::Processing {
            assert!(Instant::now() < deadline);
            thread::sleep(Duration::from_millis(2));
        }
        // Simulate a delayed Stop both action that had seen Recording earlier.
        recorder.cancel_recording();
        drop(sink);
        thread::sleep(Duration::from_millis(30));
        assert_eq!(admission.failure.load(Ordering::Acquire), 0);
        assert!(!admission.is_cancelled());
        admission.producers.fetch_sub(1, Ordering::SeqCst);
        while !directory.join("processing").exists() {
            assert!(Instant::now() < deadline);
            thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(recorder.snapshot().status, RecordingStatus::Processing);
        recorder.cancel_recording();
        fs::write(directory.join("finish"), []).unwrap();
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Ready, "{state:?}");
        assert!(state.cameras.iter().all(|camera| camera.queue_dropped == 0));
        assert_eq!(records(&directory.join("camera0.udp")).len(), 1);
        let manifest: Value =
            serde_json::from_slice(&fs::read(directory.join("manifest.json")).unwrap()).unwrap();
        assert_eq!(manifest["complete"], true);
        assert_eq!(manifest["state"], "ready");
    }

    #[test]
    fn intentional_cancel_and_last_sender_detach_report_cancellation() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("cancel-detach");
        let mut recorder = recorder(&temporary.0, "exit 99\n");
        recorder.options.duration = Duration::from_secs(2);
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        wait_until_armed(&sink);
        let admission = Arc::clone(&sink.admission);
        // Keep the drain barrier pending so sender disconnection is delivered
        // through recv_timeout rather than the already-closed try_recv path.
        admission.producers.fetch_add(1, Ordering::SeqCst);
        recorder.cancel_recording();
        drop(sink);
        thread::sleep(Duration::from_millis(30));
        assert_eq!(admission.failure.load(Ordering::Acquire), 0);
        admission.producers.fetch_sub(1, Ordering::SeqCst);
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Incomplete);
        assert_eq!(
            state.error.as_deref(),
            Some("Recording cancelled; partial data was preserved")
        );
        assert!(!directory.join("calibration.log").exists());
    }

    #[test]
    fn overflow_is_nonblocking_and_preserves_an_incomplete_clip() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("overflow");
        let mut recorder = recorder(&temporary.0, "exit 99\n");
        recorder.options.queue_bytes = PACKET_OVERHEAD + 8;
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        wait_until_armed(&sink);
        let before = Instant::now();
        assert!(!sink.try_record(1, Instant::now(), &[0; 9]));
        assert!(before.elapsed() < Duration::from_millis(100));
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Incomplete);
        assert_eq!(state.cameras[1].queue_dropped, 1);
        assert_eq!(sink.admission.queued_bytes.load(Ordering::Acquire), 0);
        assert!(!directory.join("calibration.log").exists());
        assert!(state.error.unwrap().contains("overflow"));
    }

    #[test]
    fn cancellation_waits_for_an_already_admitted_producer_before_final_drain() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("barrier");
        let mut recorder = recorder(&temporary.0, "exit 99\n");
        recorder.options.duration = Duration::from_secs(2);
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        wait_until_armed(&sink);
        // Pause one producer after passing admission, before enqueue. This
        // models the exact race between a receiver and cancellation/deadline.
        sink.admission.producers.fetch_add(1, Ordering::SeqCst);
        recorder.cancel();
        thread::sleep(Duration::from_millis(40));
        assert_eq!(recorder.snapshot().status, RecordingStatus::Recording);
        let bytes = packet(0, 1, 7, &[3, 4]);
        let charge = bytes.len() + PACKET_OVERHEAD;
        sink.admission
            .queued_bytes
            .fetch_add(charge, Ordering::AcqRel);
        sink.sender
            .try_send(Datagram {
                camera: 0,
                elapsed_ns: 1,
                bytes: bytes.clone(),
                charge,
            })
            .unwrap();
        sink.admission.producers.fetch_sub(1, Ordering::SeqCst);
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Incomplete);
        assert_eq!(state.cameras[0].packets, 1);
        assert_eq!(records(&directory.join("camera0.udp")), [(1, bytes)]);
        assert_eq!(sink.admission.queued_bytes.load(Ordering::Acquire), 0);
    }

    #[test]
    fn existing_recording_is_never_overwritten() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("existing");
        fs::create_dir(&directory).unwrap();
        fs::write(directory.join("calibration.json"), b"original calibration").unwrap();
        let mut recorder = recorder(&temporary.0, "exit 99\n");
        let _sink = recorder.start(directory.clone(), json!({})).unwrap();
        assert_eq!(finished(&recorder).status, RecordingStatus::Error);
        assert_eq!(
            fs::read(directory.join("calibration.json")).unwrap(),
            b"original calibration"
        );
        assert!(!directory.join("camera0.udp").exists());
    }

    #[test]
    fn failed_processing_keeps_completed_raw_recording_without_ready_result() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("fit-failure");
        let mut recorder = recorder(
            &temporary.0,
            "echo '{\"ok\":false,\"error\":\"camera 0: insufficient observations; cover the image edges\"}'\necho 'launcher: fitter exited with status 2'\nexit 2\n",
        );
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        images(&sink);
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Error);
        assert!(state.calibration_path.is_none());
        assert!(
            state
                .error
                .as_ref()
                .unwrap()
                .contains("camera 0: insufficient observations; cover the image edges")
        );
        let manifest: Value =
            serde_json::from_slice(&fs::read(directory.join("manifest.json")).unwrap()).unwrap();
        assert_eq!(manifest["complete"], true);
        assert_eq!(manifest["state"], "processing_failed");
        assert_eq!(manifest["error"], state.error.unwrap());
        assert!(
            fs::read_to_string(directory.join("calibration.log"))
                .unwrap()
                .contains("insufficient observations")
        );
    }

    #[test]
    fn cancel_during_processing_returns_promptly_and_preserves_complete_clips() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("cancel-fit");
        let mut recorder = recorder(&temporary.0, "touch \"$2/processing\"\nsleep 30\n");
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        images(&sink);
        let deadline = Instant::now() + Duration::from_secs(3);
        while !directory.join("processing").exists() {
            assert!(Instant::now() < deadline);
            thread::sleep(Duration::from_millis(5));
        }
        let before = Instant::now();
        recorder.cancel();
        assert!(before.elapsed() < Duration::from_millis(100));
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Incomplete);
        assert!(state.calibration_path.is_none());
        let manifest: Value =
            serde_json::from_slice(&fs::read(directory.join("manifest.json")).unwrap()).unwrap();
        assert_eq!(manifest["complete"], true);
        assert_eq!(records(&directory.join("camera1.udp")).len(), 1);
    }

    #[test]
    fn successful_fit_is_published_while_centroid_archive_is_still_running() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("early-fit");
        let mut recorder = staged_recorder(
            &temporary.0,
            r#"case "$1" in
                --fit-only) printf '{"fit":"ready"}' > "$3/calibration.json" ;;
                --metadata-only) touch "$3/archiving"
                    while [ ! -f "$3/release" ]; do sleep 0.01; done ;;
                *) exit 99 ;;
            esac
            "#,
        );
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        images(&sink);
        wait_for_file(&directory.join("archiving"));
        let state = recorder.snapshot();
        assert_eq!(state.status, RecordingStatus::Processing);
        assert_eq!(
            state.calibration_path,
            Some(directory.join("calibration.json"))
        );
        assert!(state.message.contains("Calibration ready"));
        assert!(state.message.contains("centroid archive"));
        assert!(state.error.is_none());
        assert!(recorder.is_running());
        assert!(
            recorder
                .start(temporary.0.join("too-soon"), json!({}))
                .is_err()
        );
        fs::write(directory.join("release"), b"").unwrap();
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Ready);
        assert!(state.error.is_none());
        assert!(directory.join("calibration.log").exists());
        assert!(directory.join("archive.log").exists());
    }

    #[test]
    fn failed_fit_still_exports_centroid_archive() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("failed-fit-archive");
        let mut recorder = staged_recorder(
            &temporary.0,
            r#"case "$1" in
                --fit-only) echo "insufficient observations"; exit 2 ;;
                --metadata-only) echo "saved all centroids"; touch "$3/archived" ;;
                *) exit 99 ;;
            esac
            "#,
        );
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        images(&sink);
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Error);
        assert!(state.calibration_path.is_none());
        assert!(state.error.unwrap().contains("Calibration fitting failed"));
        assert!(directory.join("archived").exists());
        assert!(
            fs::read_to_string(directory.join("calibration.log"))
                .unwrap()
                .contains("insufficient observations")
        );
        assert!(
            fs::read_to_string(directory.join("archive.log"))
                .unwrap()
                .contains("saved all centroids")
        );
        let manifest: Value =
            serde_json::from_reader(File::open(directory.join("manifest.json")).unwrap()).unwrap();
        assert_eq!(manifest["complete"], true);
    }

    #[test]
    fn failed_centroid_archive_keeps_successful_calibration_available() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("failed-archive");
        let mut recorder = staged_recorder(
            &temporary.0,
            r#"case "$1" in
                --fit-only) printf '{}' > "$3/calibration.json" ;;
                --metadata-only) echo "archive disk error"; exit 3 ;;
                *) exit 99 ;;
            esac
            "#,
        );
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        images(&sink);
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Ready);
        assert_eq!(
            state.calibration_path,
            Some(directory.join("calibration.json"))
        );
        assert!(state.error.unwrap().contains("archive.log"));
        let manifest: Value =
            serde_json::from_reader(File::open(directory.join("manifest.json")).unwrap()).unwrap();
        assert_eq!(manifest["state"], "ready");
        assert_eq!(manifest["complete"], true);
        assert!(manifest["calibration_path"].is_string());
    }

    #[test]
    fn invalid_fit_output_cannot_be_published_but_still_exports_centroids() {
        for output in ["[]", "not JSON"] {
            let temporary = TestDirectory::new();
            let directory = temporary.0.join("invalid-fit");
            let mut recorder = staged_recorder(
                &temporary.0,
                &format!(
                    r#"case "$1" in
                    --fit-only) printf '%s' '{output}' > "$3/calibration.json" ;;
                    --metadata-only) touch "$3/archiving"
                        while [ ! -f "$3/release" ]; do sleep 0.01; done ;;
                    *) exit 99 ;;
                esac
                "#
                ),
            );
            let sink = recorder.start(directory.clone(), json!({})).unwrap();
            images(&sink);
            wait_for_file(&directory.join("archiving"));
            let state = recorder.snapshot();
            assert_eq!(state.status, RecordingStatus::Processing);
            assert!(state.calibration_path.is_none());
            assert!(state.message.contains("fit failed"));
            fs::write(directory.join("release"), b"").unwrap();
            assert_eq!(finished(&recorder).status, RecordingStatus::Error);
        }
    }

    #[cfg(target_os = "linux")]
    fn assert_child_stopped(path: &Path) {
        let pid: u32 = fs::read_to_string(path).unwrap().trim().parse().unwrap();
        let deadline = Instant::now() + Duration::from_secs(1);
        loop {
            let stat = fs::read_to_string(format!("/proc/{pid}/stat"));
            if stat.is_err()
                || stat
                    .as_ref()
                    .unwrap()
                    .rsplit_once(") ")
                    .unwrap()
                    .1
                    .starts_with('Z')
            {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "processor child remained alive: {stat:?}"
            );
            thread::sleep(Duration::from_millis(5));
        }
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn cancelling_either_stage_stops_descendants_and_clears_early_result() {
        for cancel_fit in [true, false] {
            let temporary = TestDirectory::new();
            let directory = temporary.0.join("cancel-stage");
            let target = if cancel_fit {
                "--fit-only"
            } else {
                "--metadata-only"
            };
            let mut recorder = staged_recorder(
                &temporary.0,
                &format!(
                    r#"if [ "$1" = '{target}' ]; then
                    sleep 30 &
                    echo $! > "$3/child.pid"
                    wait
                elif [ "$1" = '--fit-only' ]; then
                    printf '{{}}' > "$3/calibration.json"
                else
                    touch "$3/archived"
                fi
                "#
                ),
            );
            let sink = recorder.start(directory.clone(), json!({})).unwrap();
            images(&sink);
            wait_for_file(&directory.join("child.pid"));
            assert_eq!(recorder.snapshot().calibration_path.is_some(), !cancel_fit);
            recorder.cancel();
            assert!(recorder.snapshot().calibration_path.is_none());
            let state = finished(&recorder);
            assert_eq!(state.status, RecordingStatus::Incomplete);
            assert!(state.calibration_path.is_none());
            assert!(!directory.join("archived").exists());
            assert_child_stopped(&directory.join("child.pid"));
        }
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn centroid_archive_timeout_keeps_fit_and_stops_descendants() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("archive-timeout");
        let mut recorder = staged_recorder(
            &temporary.0,
            r#"case "$1" in
                --fit-only) printf '{}' > "$3/calibration.json" ;;
                --metadata-only) sleep 30 &
                    echo $! > "$3/child.pid"
                    wait ;;
                *) exit 99 ;;
            esac
            "#,
        );
        recorder.options.processing_timeout = Duration::from_millis(200);
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        images(&sink);
        let state = finished(&recorder);
        assert_eq!(state.status, RecordingStatus::Ready);
        assert_eq!(
            state.calibration_path,
            Some(directory.join("calibration.json"))
        );
        assert!(state.error.unwrap().contains("Recording export timed out"));
        assert_child_stopped(&directory.join("child.pid"));
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn failed_processor_does_not_leave_its_child_running() {
        let temporary = TestDirectory::new();
        let directory = temporary.0.join("orphan-fit");
        let mut recorder = recorder(
            &temporary.0,
            "sleep 30 &\necho $! > \"$2/child.pid\"\nexit 2\n",
        );
        let sink = recorder.start(directory.clone(), json!({})).unwrap();
        images(&sink);
        assert_eq!(finished(&recorder).status, RecordingStatus::Error);
        let pid: u32 = fs::read_to_string(directory.join("child.pid"))
            .unwrap()
            .trim()
            .parse()
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(1);
        loop {
            let stat = fs::read_to_string(format!("/proc/{pid}/stat"));
            // The child's new parent may not have reaped a zombie yet, but it
            // must be dead and unable to write after our terminal Error state.
            if stat.is_err()
                || stat
                    .as_ref()
                    .unwrap()
                    .rsplit_once(") ")
                    .unwrap()
                    .1
                    .starts_with('Z')
            {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "processor child remained alive: {stat:?}"
            );
            thread::sleep(Duration::from_millis(5));
        }
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn final_buffer_flush_error_is_reported() {
        let temporary = TestDirectory::new();
        let mut files = RecordingFiles::create(&temporary.0).unwrap();
        files.clips[0] = BufWriter::new(File::options().write(true).open("/dev/full").unwrap());
        files.clips[0].write_all(b"buffered record").unwrap();
        assert!(files.finish().is_err());
    }
}
