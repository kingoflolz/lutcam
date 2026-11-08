//! Feed an embedded Rerun viewer without blocking the GUI or UDP workers.

use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use re_viewer::external::re_log_channel::{self, LogSource};
use rerun::blueprint::{
    Blueprint, BlueprintPanel, ContainerLike, Horizontal, SelectionPanel, Spatial2DView, TimePanel,
    Vertical,
    components::{PanelState, PlayState},
};

use crate::centroids::{Centroid, FrameCentroids};
use crate::poses::{FrameDetections, TagDetection};
use crate::receiver::DecodedFrame;

pub use re_viewer::external::re_log_channel::LogReceiver;
pub type FeedError = Box<dyn std::error::Error + Send + Sync>;

const MATCH_CAPACITY: usize = 64;
const MATCH_AGE: Duration = Duration::from_secs(2);
const MAX_DETECTIONS: usize = 128;
const CAMERA_ORDER: [usize; 2] = [1, 0];

/// Intrinsics use integer-centered sensor pixels, before the display row flip.
/// `tag_width` must use the same unit/value configured in the FPGA pose solver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraCalibration {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub tag_width: f32,
}

impl Default for CameraCalibration {
    fn default() -> Self {
        Self {
            fx: 2.8 / 0.003,
            fy: 2.8 / 0.003,
            cx: (lwc::FRAME_WIDTH as f32 - 1.0) / 2.0,
            cy: (lwc::FRAME_HEIGHT as f32 - 1.0) / 2.0,
            tag_width: 1.0,
        }
    }
}

impl CameraCalibration {
    pub fn validate(self) -> Result<(), FeedError> {
        if ![self.fx, self.fy, self.cx, self.cy, self.tag_width]
            .into_iter()
            .all(f32::is_finite)
            || self.fx <= 0.0
            || self.fy <= 0.0
            || self.tag_width <= 0.0
        {
            return Err(
                "camera calibration requires finite positive focal lengths and tag width".into(),
            );
        }
        Ok(())
    }

    fn display_principal(self) -> [f32; 2] {
        // Rerun image edges are [0,width] x [0,height]. Sensor pixel centers
        // are integers, so include the half pixel before reflecting vertically.
        [self.cx + 0.5, lwc::FRAME_HEIGHT as f32 - 0.5 - self.cy]
    }

    pub fn pinhole(self) -> rerun::Pinhole {
        let [cx, cy] = self.display_principal();
        rerun::Pinhole::new([[self.fx, 0.0, 0.0], [0.0, self.fy, 0.0], [cx, cy, 1.0]])
            .with_resolution([lwc::FRAME_WIDTH as f32, lwc::FRAME_HEIGHT as f32])
            .with_camera_xyz(rerun::components::ViewCoordinates::RDF)
    }
}

#[derive(Default)]
struct Pending {
    frames: [Option<Arc<DecodedFrame>>; 2],
    detections: [VecDeque<Arc<FrameDetections>>; 2],
    centroids: [VecDeque<Arc<FrameCentroids>>; 2],
    calibrations: [Option<CameraCalibration>; 2],
    layout: Option<bool>,
    tag_outlines_only: Option<bool>,
    clear: bool,
    stop: bool,
    error: Option<String>,
}

#[derive(Clone, Copy)]
struct LoggedImage {
    frame_id: u32,
    received_at: Instant,
    time: Duration,
    calibration: CameraCalibration,
    pose_revision: Option<u64>,
    centroid_revision: Option<u64>,
}

#[derive(Default)]
struct CameraHistory {
    calibration: CameraCalibration,
    last_image_received: Option<Instant>,
    epoch_started: Option<Instant>,
    images: VecDeque<LoggedImage>,
    detections: VecDeque<Arc<FrameDetections>>,
    centroids: VecDeque<Arc<FrameCentroids>>,
    // Keep only the currently displayed overlay after join-cache expiry so a
    // presentation toggle still works when capture is stopped. A new image or
    // resync removes it before any new pose can be associated.
    current_overlay: Option<(LoggedImage, Arc<FrameDetections>)>,
}

impl CameraHistory {
    fn prune(&mut self, now: Instant) {
        self.images
            .retain(|image| now.saturating_duration_since(image.received_at) <= MATCH_AGE);
        self.detections
            .retain(|frame| now.saturating_duration_since(frame.received_at) <= MATCH_AGE);
        self.centroids
            .retain(|frame| now.saturating_duration_since(frame.received_at) <= MATCH_AGE);
    }

    fn image(&mut self, frame: &DecodedFrame, time: Duration) -> bool {
        // The receiver starts a new ordering epoch after two seconds of
        // silence. Do not match a reused frame ID against the previous epoch,
        // even if a late old pose kept that cache entry younger than its image.
        if self.last_image_received.is_some_and(|previous| {
            frame.received_at.saturating_duration_since(previous) >= MATCH_AGE
        }) {
            self.clear();
            self.epoch_started = Some(frame.received_at);
        }
        self.last_image_received = Some(frame.received_at);
        if self
            .images
            .iter()
            .any(|image| image.frame_id == frame.frame_id && image.received_at == frame.received_at)
        {
            return false;
        }
        self.current_overlay = None;
        self.images.retain(|image| image.frame_id != frame.frame_id);
        self.images.push_back(LoggedImage {
            frame_id: frame.frame_id,
            received_at: frame.received_at,
            time,
            calibration: self.calibration,
            pose_revision: None,
            centroid_revision: None,
        });
        while self.images.len() > MATCH_CAPACITY {
            self.images.pop_front();
        }
        true
    }

    fn join(&mut self, frame_id: u32) -> Option<(LoggedImage, Arc<FrameDetections>)> {
        let detections = self
            .detections
            .iter()
            .find(|frame| frame.frame_id == frame_id)?;
        let image = self
            .images
            .iter_mut()
            .find(|image| image.frame_id == frame_id)?;
        let distance = image
            .received_at
            .max(detections.received_at)
            .duration_since(image.received_at.min(detections.received_at));
        if image
            .pose_revision
            .is_some_and(|revision| revision >= detections.revision)
            || distance > MATCH_AGE
            || self
                .epoch_started
                .is_some_and(|started| detections.received_at < started)
        {
            return None;
        }
        image.pose_revision = Some(detections.revision);
        let matched = (*image, Arc::clone(detections));
        if self
            .images
            .back()
            .is_some_and(|latest| latest.time == matched.0.time)
        {
            self.current_overlay = Some((matched.0, Arc::clone(&matched.1)));
        }
        Some(matched)
    }

    fn clear(&mut self) {
        self.last_image_received = None;
        self.epoch_started = None;
        self.images.clear();
        self.detections.clear();
        self.centroids.clear();
        self.current_overlay = None;
    }

    fn join_centroids(&mut self, frame_id: u32) -> Option<(LoggedImage, Arc<FrameCentroids>)> {
        let centroids = self
            .centroids
            .iter()
            .find(|frame| frame.frame_id == frame_id)?;
        let image = self
            .images
            .iter_mut()
            .find(|image| image.frame_id == frame_id)?;
        let distance = image
            .received_at
            .max(centroids.received_at)
            .duration_since(image.received_at.min(centroids.received_at));
        if image
            .centroid_revision
            .is_some_and(|revision| revision >= centroids.revision)
            || distance > MATCH_AGE
            || self
                .epoch_started
                .is_some_and(|started| centroids.received_at < started)
        {
            return None;
        }
        image.centroid_revision = Some(centroids.revision);
        Some((*image, Arc::clone(centroids)))
    }
}

fn remember_detections(cache: &mut VecDeque<Arc<FrameDetections>>, frame: Arc<FrameDetections>) {
    if let Some(previous) = cache
        .iter_mut()
        .find(|previous| previous.frame_id == frame.frame_id)
    {
        if frame.revision > previous.revision {
            *previous = frame;
        }
        return;
    }
    cache.push_back(frame);
    while cache.len() > MATCH_CAPACITY {
        cache.pop_front();
    }
}

fn remember_centroids(cache: &mut VecDeque<Arc<FrameCentroids>>, frame: Arc<FrameCentroids>) {
    if let Some(previous) = cache
        .iter_mut()
        .find(|previous| previous.frame_id == frame.frame_id)
    {
        if frame.revision > previous.revision {
            *previous = frame;
        }
        return;
    }
    cache.push_back(frame);
    while cache.len() > MATCH_CAPACITY {
        cache.pop_front();
    }
}

pub struct RerunFeed {
    pending: Arc<(Mutex<Pending>, Condvar)>,
}

impl RerunFeed {
    pub fn new() -> Result<(Self, LogReceiver), FeedError> {
        if !rerun::decide_logging_enabled(true) {
            return Err(
                "The embedded viewer requires Rerun logging; unset RERUN or set RERUN=on".into(),
            );
        }
        // The channel is limited to 128 MiB and the SDK applies backpressure.
        // Only this logger worker may call the SDK, never GUI/UDP workers.
        let (sender, receiver) = re_log_channel::log_channel(LogSource::Sdk);
        let pending = Arc::new((
            Mutex::new(Pending {
                layout: Some(false),
                ..Default::default()
            }),
            Condvar::new(),
        ));
        let worker_pending = Arc::clone(&pending);
        thread::Builder::new()
            .name("rerun-images".into())
            .spawn(move || {
                let result = (|| -> Result<(), FeedError> {
                    let recording = rerun::RecordingStreamBuilder::new("LUTCam Live")
                        .enabled(true)
                        .batcher_config(rerun::log::ChunkBatcherConfig {
                            max_bytes_in_flight: 8 * 1024 * 1024,
                            // Flush complete logical updates below. Independent
                            // timer/size thresholds can otherwise expose the new
                            // image and its clear a flush before its geometry.
                            ..rerun::log::ChunkBatcherConfig::NEVER
                        })
                        .buffered()?;
                    recording.set_sink(Box::new(rerun::sink::CallbackSink::new(move |messages| {
                        for message in messages {
                            if sender.send(message.clone().into()).is_err() {
                                break;
                            }
                        }
                    })));
                    let started = Instant::now();
                    let mut last_time = Duration::ZERO;
                    let mut initial_layout = true;
                    let mut tag_outlines_only = false;
                    let mut cameras: [CameraHistory; 2] = Default::default();
                    loop {
                        let (
                            frames,
                            detections,
                            centroids,
                            calibrations,
                            layout,
                            outlines_only,
                            clear,
                        ) = {
                            let (lock, wake) = &*worker_pending;
                            let mut work = lock.lock().unwrap();
                            while !work.stop
                                && !work.clear
                                && work.layout.is_none()
                                && work.tag_outlines_only.is_none()
                                && work.frames.iter().all(Option::is_none)
                                && work.calibrations.iter().all(Option::is_none)
                                && work.detections.iter().all(VecDeque::is_empty)
                                && work.centroids.iter().all(VecDeque::is_empty)
                            {
                                work = wake.wait(work).unwrap();
                            }
                            if work.stop {
                                break;
                            }
                            (
                                std::mem::take(&mut work.frames),
                                std::mem::take(&mut work.detections),
                                std::mem::take(&mut work.centroids),
                                std::mem::take(&mut work.calibrations),
                                work.layout.take(),
                                work.tag_outlines_only.take(),
                                std::mem::take(&mut work.clear),
                            )
                        };
                        let presentation_changed =
                            outlines_only.is_some_and(|value| value != tag_outlines_only);
                        if let Some(value) = outlines_only {
                            tag_outlines_only = value;
                        }
                        if let Some(stacked) = layout {
                            set_layout(&recording, stacked, initial_layout)?;
                            initial_layout = false;
                        }
                        if clear {
                            last_time = started.elapsed().max(last_time + Duration::from_nanos(1));
                            recording.set_time("capture_time", last_time);
                            for (camera, history) in cameras.iter_mut().enumerate() {
                                history.clear();
                                recording
                                    .log(format!("camera{camera}"), &rerun::Clear::recursive())?;
                            }
                        }
                        for (camera, history) in cameras.iter_mut().enumerate() {
                            history.prune(Instant::now());
                            if let Some(calibration) = calibrations[camera] {
                                history.calibration = calibration;
                            }
                        }
                        // Register image epochs before accepting auxiliary data:
                        // an image following a capture restart clears old joins.
                        // Publish only after all products from this snapshot have
                        // entered their caches, so an available pose is not held
                        // until a separate worker wake after its image/clear.
                        let mut new_images: [Option<(Arc<DecodedFrame>, Duration)>; 2] =
                            [None, None];
                        for (camera, frame) in frames.into_iter().enumerate() {
                            if let Some(frame) = frame {
                                let history = &mut cameras[camera];
                                let time = frame
                                    .received_at
                                    .saturating_duration_since(started)
                                    .max(last_time + Duration::from_nanos(1));
                                if !history.image(&frame, time) {
                                    continue;
                                }
                                last_time = time;
                                new_images[camera] = Some((frame, time));
                            }
                        }
                        let mut changed_detections: [Vec<u32>; 2] = Default::default();
                        for (camera, frames) in detections.into_iter().enumerate() {
                            let history = &mut cameras[camera];
                            for frame in frames {
                                if Instant::now().saturating_duration_since(frame.received_at)
                                    > MATCH_AGE
                                {
                                    continue;
                                }
                                changed_detections[camera].push(frame.frame_id);
                                remember_detections(&mut history.detections, frame);
                            }
                        }
                        let mut changed_centroids: [Vec<u32>; 2] = Default::default();
                        for (camera, frames) in centroids.into_iter().enumerate() {
                            let history = &mut cameras[camera];
                            for frame in frames {
                                if Instant::now().saturating_duration_since(frame.received_at)
                                    > MATCH_AGE
                                {
                                    continue;
                                }
                                changed_centroids[camera].push(frame.frame_id);
                                remember_centroids(&mut history.centroids, frame);
                            }
                        }
                        for (camera, frame) in new_images.into_iter().enumerate() {
                            if let Some((frame, time)) = frame {
                                let history = &mut cameras[camera];
                                recording.set_time("capture_time", time);
                                log_image_frame(
                                    &recording,
                                    camera,
                                    &frame.pixels,
                                    frame.frame_id,
                                    history.calibration,
                                )?;
                                if let Some((image, tags)) = history.join(frame.frame_id) {
                                    log_tags(&recording, image, &tags, tag_outlines_only)?;
                                }
                                if let Some((image, centroids)) =
                                    history.join_centroids(frame.frame_id)
                                {
                                    recording.set_time("capture_time", image.time);
                                    log_centroid_frame(&recording, &centroids)?;
                                }
                                recording.flush_async()?;
                            }
                        }
                        // Late revisions still update their original image time.
                        // The new-image joins above have already recorded their
                        // revisions, preventing a duplicate geometry publication.
                        for (camera, frames) in changed_detections.into_iter().enumerate() {
                            let history = &mut cameras[camera];
                            for frame_id in frames {
                                if let Some((image, tags)) = history.join(frame_id) {
                                    log_tags(&recording, image, &tags, tag_outlines_only)?;
                                    recording.flush_async()?;
                                }
                            }
                        }
                        for (camera, frames) in changed_centroids.into_iter().enumerate() {
                            let history = &mut cameras[camera];
                            for frame_id in frames {
                                if let Some((image, centroids)) = history.join_centroids(frame_id) {
                                    recording.set_time("capture_time", image.time);
                                    log_centroid_frame(&recording, &centroids)?;
                                    recording.flush_async()?;
                                }
                            }
                        }
                        if presentation_changed {
                            for history in &cameras {
                                if let Some((image, tags)) = &history.current_overlay {
                                    log_tags(&recording, *image, tags, tag_outlines_only)?;
                                    recording.flush_async()?;
                                }
                            }
                        }
                        // Also release layout/clear-only updates. Large bursts
                        // of late auxiliary frames are flushed individually
                        // above rather than growing one unbounded SDK batch.
                        recording.flush_async()?;
                    }
                    Ok(())
                })();
                if let Err(error) = result {
                    worker_pending.0.lock().unwrap().error = Some(error.to_string());
                }
            })?;
        Ok((Self { pending }, receiver))
    }

    pub fn error(&self) -> Option<String> {
        self.pending.0.lock().unwrap().error.clone()
    }

    /// Enqueue one camera snapshot with a single worker wake. Image and matching
    /// auxiliary data cannot be separated by the logger draining its mailbox.
    /// Rerun still stores each image/geometry entity independently; this is not
    /// a cross-entity rendering transaction.
    pub fn log_snapshot(
        &self,
        camera: usize,
        image: Option<Arc<DecodedFrame>>,
        detections: &[Arc<FrameDetections>],
        centroids: &[Arc<FrameCentroids>],
    ) -> Result<(), FeedError> {
        if camera >= 2
            || image
                .as_ref()
                .is_some_and(|frame| frame.pixels.len() != lwc::FRAME_WIDTH * lwc::FRAME_HEIGHT)
            || detections.iter().any(|frame| {
                usize::from(frame.camera) != camera || frame.detections.len() > MAX_DETECTIONS
            })
            || centroids.iter().any(|frame| {
                usize::from(frame.camera) != camera
                    || frame.groups.len() > 256
                    || frame.groups.iter().any(|group| group.centroids.len() > 9)
            })
        {
            return Err("invalid camera snapshot or unbounded auxiliary frame".into());
        }
        self.update(|pending| {
            if let Some(image) = image {
                pending.frames[camera] = Some(image);
            }
            for frame in detections {
                remember_detections(&mut pending.detections[camera], Arc::clone(frame));
            }
            for frame in centroids {
                remember_centroids(&mut pending.centroids[camera], Arc::clone(frame));
            }
        })
    }

    pub fn log_latest(&self, camera: usize, frame: Arc<DecodedFrame>) -> Result<(), FeedError> {
        if camera >= 2 || frame.pixels.len() != lwc::FRAME_WIDTH * lwc::FRAME_HEIGHT {
            return Err("invalid camera or decoded image dimensions".into());
        }
        self.update(|pending| pending.frames[camera] = Some(frame))
    }

    pub fn log_detections(&self, frame: Arc<FrameDetections>) -> Result<(), FeedError> {
        if frame.camera >= 2 || frame.detections.len() > MAX_DETECTIONS {
            return Err("invalid camera or unbounded detection frame".into());
        }
        let camera = usize::from(frame.camera);
        self.update(|pending| remember_detections(&mut pending.detections[camera], frame))
    }

    pub fn log_centroids(&self, frame: Arc<FrameCentroids>) -> Result<(), FeedError> {
        if frame.camera >= 2
            || frame.groups.len() > 256
            || frame.groups.iter().any(|group| group.centroids.len() > 9)
        {
            return Err("invalid camera or unbounded centroid frame".into());
        }
        let camera = usize::from(frame.camera);
        self.update(|pending| remember_centroids(&mut pending.centroids[camera], frame))
    }

    /// Apply to subsequently logged images; late poses retain their image's calibration.
    pub fn set_calibration(
        &self,
        camera: usize,
        calibration: CameraCalibration,
    ) -> Result<(), FeedError> {
        calibration.validate()?;
        if camera >= 2 {
            return Err("invalid calibration camera".into());
        }
        self.update(|pending| pending.calibrations[camera] = Some(calibration))
    }

    pub fn set_layout(&self, stacked: bool) -> Result<(), FeedError> {
        self.update(|pending| pending.layout = Some(stacked))
    }

    /// Change future overlays and refresh the current image at its original
    /// capture time. Earlier timeline images retain their logged presentation.
    pub fn set_tag_outlines_only(&self, enabled: bool) -> Result<(), FeedError> {
        self.update(|pending| pending.tag_outlines_only = Some(enabled))
    }

    /// Clear live image/pose matching after resync, preserving browsable history.
    pub fn clear(&self) -> Result<(), FeedError> {
        self.update(|pending| {
            pending.frames = [None, None];
            pending.detections = Default::default();
            pending.centroids = Default::default();
            pending.clear = true;
        })
    }

    fn update(&self, change: impl FnOnce(&mut Pending)) -> Result<(), FeedError> {
        let mut pending = self.pending.0.lock().unwrap();
        if let Some(error) = &pending.error {
            return Err(error.clone().into());
        }
        change(&mut pending);
        drop(pending);
        self.pending.1.notify_one();
        Ok(())
    }
}

impl Drop for RerunFeed {
    fn drop(&mut self) {
        // Closing the viewer channel releases SDK backpressure; never join here.
        let mut pending = self.pending.0.lock().unwrap();
        pending.stop = true;
        pending.frames = [None, None];
        pending.detections = Default::default();
        pending.centroids = Default::default();
        self.pending.1.notify_one();
    }
}

fn flip_rows(pixels: &[u8], width: usize) -> Vec<u8> {
    pixels
        .chunks_exact(width)
        .rev()
        .flatten()
        .copied()
        .collect()
}

/// Log an image at the caller's current timeline position, shared by live and
/// archival writers. Pixel storage remains untouched outside this display copy.
pub fn log_image_frame(
    recording: &rerun::RecordingStream,
    camera: usize,
    pixels: &[u8],
    frame_id: u32,
    calibration: CameraCalibration,
) -> Result<(), FeedError> {
    calibration.validate()?;
    if camera >= 2 || pixels.len() != lwc::FRAME_WIDTH * lwc::FRAME_HEIGHT {
        return Err("invalid image dimensions or camera".into());
    }
    recording.log(format!("camera{camera}/image"), &calibration.pinhole())?;
    recording.log(
        format!("camera{camera}/image"),
        &rerun::Image::from_l8(
            flip_rows(pixels, lwc::FRAME_WIDTH),
            [lwc::FRAME_WIDTH as u32, lwc::FRAME_HEIGHT as u32],
        ),
    )?;
    recording.log(
        format!("camera{camera}/frame_id"),
        &rerun::Scalars::new([f64::from(frame_id)]),
    )?;
    clear_tags(recording, camera)?;
    recording.log(
        format!("camera{camera}/image/centroids"),
        &rerun::Clear::recursive(),
    )?;
    Ok(())
}

/// Map integer-centered source image coordinates to the row-flipped display.
pub fn display_pixel([x, y]: [f32; 2]) -> [f32; 2] {
    [x + 0.5, lwc::FRAME_HEIGHT as f32 - 0.5 - y]
}

/// Reject invalid/fence tuples and map RAW/LL2/LL4 composite coordinates back
/// into the source camera image before applying the display row flip.
pub fn display_centroid(centroid: &Centroid) -> Option<([f32; 2], usize)> {
    let (x, y, scale) = lrcca::project_wavelet_centroid(
        lrcca::TagDetectCentroid {
            centroid_valid: centroid.valid,
            x_q5: u32::from(centroid.x_q5),
            y_q5: u32::from(centroid.y_q5),
        },
        lwc::FRAME_WIDTH,
        lwc::FRAME_HEIGHT,
    )?;
    Some((display_pixel([x as f32, y as f32]), scale))
}

/// Log every supplied raw tuple, including invalid/fence coordinates, and a
/// separate measured 2D overlay. The caller owns frame/time association.
pub fn log_centroid_frame(
    recording: &rerun::RecordingStream,
    frame: &FrameCentroids,
) -> Result<(), FeedError> {
    if frame.camera >= 2
        || frame.groups.len() > 256
        || frame.groups.iter().any(|group| group.centroids.len() > 9)
    {
        return Err("invalid camera or unbounded centroid frame".into());
    }
    let root = format!("camera{}/image/centroids", frame.camera);
    recording.log(root.as_str(), &rerun::Clear::recursive())?;
    let groups: Vec<_> = frame
        .groups
        .iter()
        .map(|group| {
            serde_json::json!({
                "sequence": group.sequence, "group_index": group.group_index,
                "centroids": group.centroids.iter().map(|point| serde_json::json!({
                    "valid": point.valid, "x_q5": point.x_q5, "y_q5": point.y_q5,
                })).collect::<Vec<_>>(),
            })
        })
        .collect();
    let raw = serde_json::json!({
        "camera": frame.camera, "frame_id": frame.frame_id, "revision": frame.revision,
        "invalid": frame.invalid, "coordinate_domain": "compact composite unsigned Q5",
        "groups": groups,
    });
    recording.log(
        format!("{root}/raw"),
        &rerun::TextDocument::new(raw.to_string()),
    )?;
    if !frame.invalid {
        let projected: Vec<_> = frame
            .groups
            .iter()
            .flat_map(|group| &group.centroids)
            .filter_map(display_centroid)
            .collect();
        let colors = projected.iter().map(|(_, scale)| match scale {
            1 => [64, 208, 255],
            4 => [192, 128, 255],
            _ => [255, 128, 192],
        });
        recording.log(
            root.as_str(),
            &rerun::Points2D::new(projected.iter().map(|(point, _)| *point))
                .with_colors(colors)
                .with_radii([rerun::Radius::new_ui_points(2.0)]),
        )?;
    }
    Ok(())
}

fn clear_tags(recording: &rerun::RecordingStream, camera: usize) -> Result<(), FeedError> {
    recording.log(format!("camera{camera}/lftags"), &rerun::Clear::recursive())?;
    recording.log(
        format!("camera{camera}/image/lftags"),
        &rerun::Clear::recursive(),
    )?;
    Ok(())
}

struct TagGeometry {
    rotation_columns: [[f32; 3]; 3],
    translation: [f32; 3],
    points: Vec<[f32; 3]>,
    outline: Vec<[f32; 3]>,
}

impl TagGeometry {
    fn camera_point(&self, point: [f32; 3]) -> [f32; 3] {
        std::array::from_fn(|row| {
            self.translation[row]
                + (0..3)
                    .map(|column| self.rotation_columns[column][row] * point[column])
                    .sum::<f32>()
        })
    }

    fn project(&self, point: [f32; 3], calibration: CameraCalibration) -> [f32; 2] {
        let camera = self.camera_point(point);
        display_pixel([
            calibration.fx * camera[0] / camera[2] + calibration.cx,
            calibration.fy * camera[1] / camera[2] + calibration.cy,
        ])
    }
}

fn tag_geometry(tag: &TagDetection, calibration: CameraCalibration) -> Option<TagGeometry> {
    if tag.tag_id >= 1 << 14 || tag.pose.saturation {
        return None;
    }
    let rotation = tag
        .pose
        .pose
        .rotation_q29
        .map(|value| value as f32 / (1u32 << 29) as f32);
    // Rerun matrices are column-major. Keep the raw matrix, including its
    // decomposition error; converting it to a quaternion would change the pose.
    let rotation_columns =
        std::array::from_fn(|column| std::array::from_fn(|row| rotation[row * 3 + column]));
    let translation = tag
        .pose
        .pose
        .translation_q16
        .map(|value| value as f32 / 65536.0);
    let pitch = calibration.tag_width / 4.0;
    let points: Vec<_> = lftag::canonical_points(tag.tag_id as u16)
        .into_iter()
        .map(|point| {
            [
                point.position.x as f32 * pitch,
                point.position.y as f32 * pitch,
                0.0,
            ]
        })
        .collect();
    let outline: Vec<_> = [
        [-2.0, -2.0],
        [2.0, -2.0],
        [2.0, 2.0],
        [-2.0, 2.0],
        [-2.0, -2.0],
    ]
    .into_iter()
    .map(|[x, y]| [x * pitch, y * pitch, 0.0])
    .collect();
    if points.iter().chain(&outline).any(|point| {
        let z = rotation[6] * point[0] + rotation[7] * point[1] + translation[2];
        !z.is_finite() || z <= 0.0
    }) {
        return None;
    }
    let determinant = rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7])
        - rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6])
        + rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6]);
    if !determinant.is_finite() || determinant == 0.0 {
        return None;
    }
    Some(TagGeometry {
        rotation_columns,
        translation,
        points,
        outline,
    })
}

/// A valid projected tag footprint, with counterclockwise corners. Use the
/// actual quadrilateral: bounding boxes alone merge nearby tilted tags.
struct TagFootprint {
    corners: [[f64; 2]; 4],
    area: f64,
}

fn cross(origin: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])
}

fn polygon_area(points: &[[f64; 2]]) -> f64 {
    (1..points.len().saturating_sub(1))
        .map(|index| cross(points[0], points[index], points[index + 1]))
        .sum::<f64>()
        / 2.0
}

impl TagFootprint {
    fn new(geometry: &TagGeometry, calibration: CameraCalibration) -> Option<Self> {
        let mut corners: [[f64; 2]; 4] = std::array::from_fn(|index| {
            geometry
                .project(geometry.outline[index], calibration)
                .map(f64::from)
        });
        if corners.iter().flatten().any(|value| !value.is_finite()) {
            return None;
        }
        let signed_area = polygon_area(&corners);
        if !signed_area.is_finite() || signed_area.abs() < 1e-8 {
            return None;
        }
        if signed_area < 0.0 {
            corners.reverse();
        }
        let area = signed_area.abs();
        if (0..4).any(|index| {
            cross(
                corners[index],
                corners[(index + 1) % 4],
                corners[(index + 2) % 4],
            ) <= area * 1e-12
        }) {
            return None;
        }
        Some(Self { corners, area })
    }

    /// Intersection over union >= 0.5 identifies duplicate spatial detections
    /// even when a coarser plane decodes the same physical tag to a different ID.
    fn overlaps(&self, other: &Self) -> bool {
        let mut intersection = self.corners.to_vec();
        for edge in 0..4 {
            let start = other.corners[edge];
            let end = other.corners[(edge + 1) % 4];
            let mut clipped = Vec::with_capacity(8);
            let Some(&last) = intersection.last() else {
                return false;
            };
            let mut previous = last;
            let mut previous_side = cross(start, end, previous);
            for current in intersection {
                let current_side = cross(start, end, current);
                if (previous_side >= 0.0) != (current_side >= 0.0) {
                    let fraction = previous_side / (previous_side - current_side);
                    clipped.push(std::array::from_fn(|axis| {
                        previous[axis] + fraction * (current[axis] - previous[axis])
                    }));
                }
                if current_side >= 0.0 {
                    clipped.push(current);
                }
                previous = current;
                previous_side = current_side;
            }
            intersection = clipped;
        }
        // I / (A + B - I) >= 1/2, avoiding a division for every pair.
        3.0 * polygon_area(&intersection).abs() >= self.area + other.area
    }
}

fn displayed_tags(
    frame: &FrameDetections,
    calibration: CameraCalibration,
) -> Vec<(usize, TagGeometry)> {
    let mut candidates: Vec<_> = frame
        .detections
        .iter()
        .enumerate()
        .filter_map(|(index, tag)| {
            let geometry = tag_geometry(tag, calibration)?;
            let footprint = TagFootprint::new(&geometry, calibration)?;
            Some((index, geometry, footprint))
        })
        .collect();
    // A finer raster can split a nearby tag into a plausible but incorrect ID.
    // Normalized residuals are comparable across planes; resolution alone must
    // not let that candidate hide a better fit from a coarser plane.
    candidates.sort_by_key(|(index, _, _)| {
        let tag = &frame.detections[*index];
        (
            u64::from(tag.pose.mean_decode_residual_q32),
            tag.scale,
            tag.best_score_q32,
            tag.pose.mean_decode_residual_q32,
            (tag.tag_id, tag.sequence, tag.record_index, tag.hypothesis),
            (tag.pose.pose.rotation_q29, tag.pose.pose.translation_q16),
        )
    });
    let mut selected: Vec<(usize, TagGeometry, TagFootprint)> = Vec::new();
    for candidate in candidates {
        if !selected
            .iter()
            .any(|(_, _, footprint)| candidate.2.overlaps(footprint))
        {
            selected.push(candidate);
        }
    }
    selected
        .into_iter()
        .map(|(index, geometry, _)| (index, geometry))
        .collect()
}

fn log_tags(
    recording: &rerun::RecordingStream,
    image: LoggedImage,
    frame: &FrameDetections,
    outlines_only: bool,
) -> Result<(), FeedError> {
    recording.set_time("capture_time", image.time);
    log_detection_frame_with_presentation(recording, image.calibration, frame, outlines_only)
}

/// Log accepted pose records and their back-projected geometry at the caller's
/// current timeline position. Overlapping footprints display the lowest
/// size-normalized decode error, with the finest plane breaking ties. All
/// records, including suppressed duplicates and saturated/invalid poses,
/// remain in raw metadata.
pub fn log_detection_frame(
    recording: &rerun::RecordingStream,
    calibration: CameraCalibration,
    frame: &FrameDetections,
) -> Result<(), FeedError> {
    log_detection_frame_with_presentation(recording, calibration, frame, false)
}

fn log_detection_frame_with_presentation(
    recording: &rerun::RecordingStream,
    calibration: CameraCalibration,
    frame: &FrameDetections,
    outlines_only: bool,
) -> Result<(), FeedError> {
    calibration.validate()?;
    if frame.camera >= 2 || frame.detections.len() > MAX_DETECTIONS {
        return Err("invalid camera or unbounded detection frame".into());
    }
    let camera = usize::from(frame.camera);
    clear_tags(recording, camera)?;
    let root = format!("camera{camera}/lftags");
    let detections: Vec<_> = frame.detections.iter().map(|tag| serde_json::json!({
        "sequence": tag.sequence, "record_index": tag.record_index, "tag_id": tag.tag_id,
        "common_words": tag.common_words, "scale": tag.scale,
        "frame_sequence": tag.frame_sequence, "hypothesis": tag.hypothesis,
        "best_score_q32": tag.best_score_q32,
        "mean_decode_residual_q32": tag.pose.mean_decode_residual_q32,
        "weak_perspective": tag.pose.weak_perspective, "conditioning_warning": tag.pose.conditioning_warning,
        "conditioning_warning_recomputed": tag.pose.pose.conditioning_warning(),
        "saturation": tag.pose.saturation, "rotation_q29": tag.pose.pose.rotation_q29,
        "translation_q16": tag.pose.pose.translation_q16,
    })).collect();
    recording.log(
        format!("{root}/raw"),
        &rerun::TextDocument::new(
            serde_json::json!({
                "camera": frame.camera, "frame_id": frame.frame_id, "revision": frame.revision,
                "invalid": frame.invalid, "detections": detections,
            })
            .to_string(),
        ),
    )?;
    if frame.invalid {
        return Ok(());
    }
    recording.log(
        root.as_str(),
        &rerun::Transform3D::from_scale([1.0, -1.0, 1.0]),
    )?;
    let displayed = displayed_tags(frame, calibration);
    let point_capacity = if outlines_only {
        0
    } else {
        displayed.len() * 9
    };
    let mut camera_points = Vec::with_capacity(point_capacity);
    let mut camera_outlines = Vec::with_capacity(displayed.len());
    let mut overlay_points = Vec::with_capacity(point_capacity);
    let mut point_colors = Vec::with_capacity(point_capacity);
    let mut overlay_outlines = Vec::with_capacity(displayed.len());
    let mut outline_colors = Vec::with_capacity(displayed.len());
    let mut outline_labels = Vec::with_capacity(if outlines_only { 0 } else { displayed.len() });
    for (index, geometry) in displayed {
        let tag = &frame.detections[index];
        // Older gateware discarded low bits of the unnormalized homography
        // norms. Small tags could spuriously set (or miss) its warning bit.
        // The transmitted columns share one RMS scale and retain enough
        // precision to evaluate the same criterion directly. Keep the raw
        // packet flag above for diagnostics and faithful archived metadata.
        let conditioning_warning = tag.pose.pose.conditioning_warning();
        let warned = tag.pose.weak_perspective || conditioning_warning;
        let color = if warned && !outlines_only {
            [255, 184, 64]
        } else {
            [64, 255, 160]
        };
        if !outlines_only {
            let mut label = format!("{}", tag.tag_id);
            if tag.pose.weak_perspective {
                label.push_str(" weak");
            }
            if conditioning_warning {
                label.push_str(" conditioned");
            }
            outline_labels.push(label);
        }
        // Rerun 0.34's Spatial2D renderer uses one harmonic-mean focal length
        // for 3D geometry. Project explicitly with both calibrated focal lengths
        // so a fitted fx != fy still aligns exactly with the image. Both 2D
        // and 3D geometry use two batched entities rather than a transform
        // subtree per tag: recursive frame clears otherwise grow the viewer's
        // transform cache for every historical child. Apply the transmitted
        // matrix directly, including decomposition error; never fit a rotation
        // quaternion. Raw matrices/translations remain in metadata above.
        if !outlines_only {
            camera_points.extend(
                geometry
                    .points
                    .iter()
                    .map(|point| geometry.camera_point(*point)),
            );
        }
        camera_outlines.push(
            geometry
                .outline
                .iter()
                .map(|point| geometry.camera_point(*point))
                .collect::<Vec<_>>(),
        );
        if !outlines_only {
            overlay_points.extend(
                geometry
                    .points
                    .iter()
                    .map(|point| geometry.project(*point, calibration)),
            );
            point_colors.extend(std::iter::repeat_n(color, geometry.points.len()));
        }
        overlay_outlines.push(
            geometry
                .outline
                .iter()
                .map(|point| geometry.project(*point, calibration))
                .collect::<Vec<_>>(),
        );
        outline_colors.push(color);
    }
    recording.log(
        format!("{root}/centroids"),
        &rerun::Points3D::new(camera_points)
            .with_colors(point_colors.iter().copied())
            .with_radii([rerun::Radius::new_ui_points(2.0)]),
    )?;
    recording.log(
        format!("{root}/outlines"),
        &rerun::LineStrips3D::new(camera_outlines)
            .with_colors(outline_colors.iter().copied())
            .with_radii([rerun::Radius::new_ui_points(1.0)])
            .with_labels(outline_labels.iter().cloned())
            .with_show_labels(!outlines_only),
    )?;
    let overlay = format!("camera{camera}/image/lftags");
    recording.log(
        format!("{overlay}/centroids"),
        &rerun::Points2D::new(overlay_points)
            .with_colors(point_colors)
            .with_radii([rerun::Radius::new_ui_points(2.0)]),
    )?;
    recording.log(
        format!("{overlay}/outlines"),
        &rerun::LineStrips2D::new(overlay_outlines)
            .with_colors(outline_colors)
            .with_radii([rerun::Radius::new_ui_points(1.0)])
            .with_labels(outline_labels)
            .with_show_labels(!outlines_only),
    )?;
    Ok(())
}

pub fn set_layout(
    recording: &rerun::RecordingStream,
    stacked: bool,
    initial: bool,
) -> Result<(), FeedError> {
    let views: Vec<ContainerLike> = CAMERA_ORDER
        .into_iter()
        .map(|camera| {
            Spatial2DView::new(format!("Camera {camera}"))
                .with_origin(format!("/camera{camera}/image"))
                .with_contents([format!("/camera{camera}/image/**")])
                .into()
        })
        .collect();
    let layout: ContainerLike = if stacked {
        Vertical::new(views).into()
    } else {
        Horizontal::new(views).into()
    };
    let mut blueprint = Blueprint::new(layout)
        .with_auto_layout(false)
        .with_auto_views(false)
        .with_blueprint_panel(BlueprintPanel::new().with_state(PanelState::Expanded))
        .with_selection_panel(SelectionPanel::new().with_state(PanelState::Collapsed));
    if initial {
        blueprint = blueprint.with_time_panel(
            TimePanel::new()
                .with_timeline("capture_time")
                .with_play_state(PlayState::Following),
        );
    }
    blueprint.send(recording, Default::default())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image(frame_id: u32, received_at: Instant) -> DecodedFrame {
        DecodedFrame {
            frame_id,
            pixels: Vec::new(),
            received_at,
            decoded_at: received_at,
            decode_ms: 0.0,
        }
    }

    fn detections(frame_id: u32, revision: u64, received_at: Instant) -> Arc<FrameDetections> {
        Arc::new(FrameDetections {
            camera: 0,
            frame_id,
            revision,
            received_at,
            detections: Vec::new(),
            invalid: false,
        })
    }

    fn tag() -> TagDetection {
        TagDetection {
            sequence: 1,
            record_index: 0,
            tag_id: 123,
            common_words: [0; 8],
            scale: 0,
            frame_sequence: 0,
            hypothesis: 0,
            best_score_q32: 0,
            pose: lftag::PosePacket {
                mean_decode_residual_q32: 0,
                weak_perspective: false,
                conditioning_warning: false,
                saturation: false,
                pose: lftag::FixedPose {
                    rotation_q29: [1 << 29, 0, 0, 0, 1 << 29, 0, 0, 0, 1 << 29],
                    translation_q16: [0, 0, 4 << 16],
                },
            },
        }
    }

    #[test]
    fn vertical_flip_preserves_columns_and_camera_order() {
        assert_eq!(flip_rows(&[1, 2, 3, 4, 5, 6], 2), [5, 6, 3, 4, 1, 2]);
        assert_eq!(CAMERA_ORDER, [1, 0]);
    }

    #[test]
    fn reflected_projection_matches_flipped_pixel_centers() {
        for calibration in [
            CameraCalibration::default(),
            CameraCalibration {
                fx: 812.0,
                fy: 946.0,
                cx: 611.25,
                cy: 381.75,
                tag_width: 0.12,
            },
        ] {
            let projection = calibration
                .pinhole()
                .image_from_camera_from_arrow()
                .unwrap();
            assert!(projection.focal_length_in_pixels().x() > 0.0);
            assert!(projection.focal_length_in_pixels().y() > 0.0);
            let [cx, cy] = calibration.display_principal();
            for [u, v] in [
                [0.0, 0.0],
                [1279.0, 799.0],
                [320.25, 701.75],
                [640.0, 400.0],
            ] {
                let z = 2.3;
                let x = (u - calibration.cx) * z / calibration.fx;
                let y = (v - calibration.cy) * z / calibration.fy;
                let display = [calibration.fx * x / z + cx, calibration.fy * -y / z + cy];
                assert!((display[0] - (u + 0.5)).abs() < 0.0002);
                assert!((display[1] - (799.5 - v)).abs() < 0.0002);
            }
        }
    }

    #[test]
    fn raw_matrix_and_canonical_tag_geometry_are_preserved() {
        let mut detection = tag();
        detection.pose.pose.rotation_q29[1] = 1 << 27; // A genuine shear, not a rigid rotation.
        let calibration = CameraCalibration {
            tag_width: 0.8,
            ..Default::default()
        };
        let geometry = tag_geometry(&detection, calibration).unwrap();
        assert_eq!(geometry.rotation_columns[1], [0.25, 1.0, 0.0]);
        assert_eq!(geometry.translation, [0.0, 0.0, 4.0]);
        assert_eq!(geometry.points.len(), 9);
        assert_eq!(geometry.points[0], [-0.2, -0.2, 0.0]);
        assert_eq!(geometry.points[1], [0.2, -0.2, 0.0]);
        assert_eq!(geometry.outline[0], [-0.4, -0.4, 0.0]);
        assert_eq!(geometry.outline[4], geometry.outline[0]);
        let fitted = CameraCalibration {
            fx: 812.0,
            fy: 946.0,
            cx: 611.25,
            cy: 381.75,
            ..calibration
        };
        let displayed = geometry.project(geometry.points[0], fitted);
        assert!((displayed[0] - 561.0).abs() < 0.0001);
        assert!((displayed[1] - 465.05).abs() < 0.0001);
        detection.pose.saturation = true;
        assert!(tag_geometry(&detection, calibration).is_none());
        detection.pose.saturation = false;
        detection.pose.pose.translation_q16[2] = -1;
        assert!(tag_geometry(&detection, calibration).is_none());
    }

    fn displayed_sequences(tags: Vec<TagDetection>) -> Vec<u32> {
        let frame = FrameDetections {
            camera: 0,
            frame_id: 1,
            revision: 1,
            received_at: Instant::now(),
            detections: tags,
            invalid: false,
        };
        displayed_tags(&frame, CameraCalibration::default())
            .iter()
            .map(|(index, _)| frame.detections[*index].sequence)
            .collect()
    }

    #[test]
    fn overlapping_planes_prefer_normalized_error_even_when_the_finer_id_conflicts() {
        let tags: Vec<_> = (0..3)
            .map(|scale| {
                let mut detection = tag();
                detection.scale = scale;
                detection.sequence = u32::from(scale);
                detection.tag_id += u32::from(scale);
                detection.best_score_q32 = u64::from(3 - scale) * 100;
                detection.pose.mean_decode_residual_q32 = u32::from(3 - scale) * 100;
                detection
            })
            .collect();
        for order in [
            [0, 1, 2],
            [0, 2, 1],
            [1, 0, 2],
            [1, 2, 0],
            [2, 0, 1],
            [2, 1, 0],
        ] {
            assert_eq!(
                displayed_sequences(order.map(|index| tags[index].clone()).into()),
                [2]
            );
        }
        assert_eq!(
            displayed_sequences(vec![tags[2].clone(), tags[1].clone()]),
            [2]
        );
    }

    #[test]
    fn overlapping_planes_prefer_finest_when_normalized_error_is_equal() {
        let mut fine = tag();
        fine.scale = 0;
        fine.sequence = 1;
        fine.best_score_q32 = 1000;
        let mut coarse = fine.clone();
        coarse.scale = 1;
        coarse.sequence = 2;
        coarse.tag_id += 1;
        coarse.best_score_q32 = 1;
        assert_eq!(displayed_sequences(vec![fine.clone(), coarse.clone()]), [1]);
        assert_eq!(displayed_sequences(vec![coarse, fine]), [1]);
    }

    #[test]
    fn overlap_selection_keeps_distinct_tags_and_uses_residual_then_stable_ties() {
        let mut near = tag();
        near.pose.mean_decode_residual_q32 = 20;
        near.sequence = 1;
        near.best_score_q32 = 20;
        let mut better = near.clone();
        better.sequence = 2;
        better.best_score_q32 = 10;
        better.pose.mean_decode_residual_q32 = 10;
        let mut best = better.clone();
        best.sequence = 3;
        best.pose.mean_decode_residual_q32 = 1;
        let mut tied = best.clone();
        tied.sequence = 4;
        let mut neighbor = near.clone();
        neighbor.sequence = 5;
        // Same ID and partially overlapping outlines, but distinct tags: IoU 1/7.
        neighbor.pose.pose.translation_q16[0] = 3 << 14;
        let mut far = near.clone();
        far.sequence = 6;
        far.pose.pose.translation_q16[0] = 2 << 16;
        let mut candidates = vec![near, better, best, tied, neighbor, far];
        assert_eq!(displayed_sequences(candidates.clone()), [3, 5, 6]);
        candidates.reverse();
        assert_eq!(displayed_sequences(candidates), [3, 5, 6]);
    }

    #[test]
    fn normalized_overlap_selection_uses_tag_relative_error_before_raw_score() {
        let mut scaled_score_small = tag();
        scaled_score_small.sequence = 1;
        scaled_score_small.best_score_q32 = 1;
        scaled_score_small.pose.mean_decode_residual_q32 = 100;
        let mut geometrically_better = scaled_score_small.clone();
        geometrically_better.sequence = 2;
        geometrically_better.best_score_q32 = 1_000;
        geometrically_better.pose.mean_decode_residual_q32 = 10;
        assert_eq!(
            displayed_sequences(vec![scaled_score_small, geometrically_better]),
            [2]
        );
    }

    #[test]
    fn invalid_fine_geometry_cannot_hide_a_valid_coarse_detection() {
        let mut valid = tag();
        valid.scale = 1;
        valid.sequence = 42;
        let mut saturated = tag();
        saturated.pose.saturation = true;
        let mut behind = tag();
        behind.pose.pose.translation_q16[2] = -1;
        let mut edge_on = tag();
        edge_on.pose.pose.rotation_q29 = [1 << 29, 0, 0, 0, 0, 1 << 29, 0, 1 << 29, 0];
        assert_eq!(
            displayed_sequences(vec![saturated, behind, edge_on, valid]),
            [42]
        );

        let calibration = CameraCalibration::default();
        let mut geometry = tag_geometry(&tag(), calibration).unwrap();
        geometry.outline.swap(1, 2); // Self-crossing bow tie.
        assert!(TagFootprint::new(&geometry, calibration).is_none());
        geometry.outline[0][0] = f32::NAN;
        assert!(TagFootprint::new(&geometry, calibration).is_none());
    }

    #[test]
    fn quadrilateral_overlap_uses_area_not_just_intersecting_bounding_boxes() {
        let diamond = TagFootprint {
            corners: [[0.0, -1.0], [1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]],
            area: 2.0,
        };
        let corner = TagFootprint {
            corners: [[0.5, 0.5], [1.0, 0.5], [1.0, 1.0], [0.5, 1.0]],
            area: 0.25,
        };
        assert!(diamond.overlaps(&diamond));
        assert!(!diamond.overlaps(&corner));
        assert!(!corner.overlaps(&diamond));
        let square = |x| TagFootprint {
            corners: [[x, 0.0], [x + 3.0, 0.0], [x + 3.0, 3.0], [x, 3.0]],
            area: 9.0,
        };
        assert!(square(0.0).overlaps(&square(1.0))); // Intersection 6, union 12.
        for offset in [1.00001, 3.0, 4.0] {
            assert!(!square(0.0).overlaps(&square(offset)));
            assert!(!square(offset).overlaps(&square(0.0)));
        }
    }

    #[test]
    fn exact_frame_matching_handles_late_revisions_wrap_and_calibration() {
        let now = Instant::now();
        let mut history = CameraHistory::default();
        remember_detections(&mut history.detections, detections(u32::MAX, 1, now));
        assert!(history.join(u32::MAX).is_none());
        let original = history.calibration;
        assert!(history.image(&image(u32::MAX, now), Duration::from_secs(3)));
        history.calibration.fx = 1100.0;
        let (matched, _) = history.join(u32::MAX).unwrap();
        assert_eq!(matched.time, Duration::from_secs(3));
        assert_eq!(matched.calibration, original);
        assert!(history.join(u32::MAX).is_none());
        assert!(!history.image(&image(u32::MAX, now), Duration::from_secs(4)));
        assert!(history.image(&image(0, now), Duration::from_secs(4)));
        assert!(history.join(0).is_none());
        remember_detections(&mut history.detections, detections(u32::MAX, 2, now));
        let (matched, revision) = history.join(u32::MAX).unwrap();
        assert_eq!(revision.revision, 2);
        assert_eq!(matched.time, Duration::from_secs(3));
        remember_detections(&mut history.detections, detections(u32::MAX, 1, now));
        assert!(history.join(u32::MAX).is_none());
        remember_detections(&mut history.detections, detections(0, 3, now));
        assert_eq!(history.join(0).unwrap().0.calibration.fx, 1100.0);
    }

    #[test]
    fn current_presentation_survives_ttl_but_never_reuses_tags_for_an_empty_new_image() {
        let now = Instant::now();
        let mut history = CameraHistory::default();
        history.image(&image(9, now), Duration::from_secs(1));
        remember_detections(&mut history.detections, detections(9, 1, now));
        history.join(9).unwrap();
        history.prune(now + MATCH_AGE + Duration::from_nanos(1));
        assert!(history.images.is_empty());
        assert_eq!(history.current_overlay.as_ref().unwrap().0.frame_id, 9);
        history.image(
            &image(10, now + Duration::from_secs(3)),
            Duration::from_secs(4),
        );
        assert!(history.current_overlay.is_none());
        // A valid late update to a historical frame can be logged on its own
        // timeline, but cannot become the currently displayed presentation.
        history.clear();
        history.image(&image(9, now), Duration::from_secs(1));
        history.image(&image(10, now), Duration::from_secs(2));
        remember_detections(&mut history.detections, detections(9, 2, now));
        assert!(history.join(9).is_some());
        assert!(history.current_overlay.is_none());
        remember_detections(&mut history.detections, detections(10, 3, now));
        history.join(10).unwrap();
        assert_eq!(history.current_overlay.as_ref().unwrap().1.revision, 3);
        history.clear();
        assert!(history.current_overlay.is_none());
    }

    #[test]
    fn matching_memory_is_bounded_and_resync_or_silence_discards_old_epoch() {
        let now = Instant::now();
        let mut history = CameraHistory::default();
        for id in 0..80 {
            history.image(&image(id, now), Duration::from_nanos(id as u64));
            remember_detections(&mut history.detections, detections(id, id as u64, now));
        }
        assert_eq!(history.images.len(), MATCH_CAPACITY);
        assert_eq!(history.detections.len(), MATCH_CAPACITY);
        assert!(history.join(0).is_none());
        history.prune(now + MATCH_AGE + Duration::from_nanos(1));
        assert!(history.images.is_empty() && history.detections.is_empty());
        history.image(
            &image(0, now + Duration::from_secs(3)),
            Duration::from_secs(3),
        );
        remember_detections(&mut history.detections, detections(0, 1, now));
        assert!(history.join(0).is_none());
        history.clear();
        assert!(history.images.is_empty() && history.detections.is_empty());

        history.image(&image(0, now), Duration::from_secs(1));
        remember_detections(
            &mut history.detections,
            detections(0, 1, now + Duration::from_millis(2)),
        );
        // A late old pose is still inside its TTL when frame zero restarts.
        history.prune(now + MATCH_AGE);
        history.image(&image(0, now + MATCH_AGE), Duration::from_secs(3));
        assert!(history.join(0).is_none());
        // Pending old products can be dequeued after the new image caused the
        // reset. They must not rejoin the reused full frame ID either.
        remember_detections(
            &mut history.detections,
            detections(0, 1, now + Duration::from_millis(2)),
        );
        assert!(history.join(0).is_none());
        remember_detections(&mut history.detections, detections(0, 2, now + MATCH_AGE));
        assert!(history.join(0).is_some());
    }

    #[test]
    fn invalid_calibration_is_rejected_before_logging() {
        for calibration in [
            CameraCalibration {
                fx: -1.0,
                ..Default::default()
            },
            CameraCalibration {
                cy: f32::NAN,
                ..Default::default()
            },
            CameraCalibration {
                tag_width: 0.0,
                ..Default::default()
            },
        ] {
            assert!(calibration.validate().is_err());
        }
    }

    #[test]
    fn measured_centroids_project_each_scale_and_reject_invalid_or_fenced_points() {
        let centroid = |x, y| Centroid {
            valid: true,
            x_q5: x,
            y_q5: y,
        };
        assert_eq!(
            display_centroid(&centroid(10 * 32 + 1, 20 * 32)),
            Some(([10.53125, 779.5], 1))
        );
        assert_eq!(
            display_centroid(&centroid(1284 * 32, 0)),
            Some(([0.5, 799.5], 4))
        );
        assert_eq!(
            display_centroid(&centroid(1612 * 32, 0)),
            Some(([0.5, 795.5], 16))
        );
        for point in [
            centroid(1282 * 32, 0),
            centroid(1608 * 32, 0),
            centroid(1284 * 32, 200 * 32),
            Centroid {
                valid: false,
                x_q5: 0,
                y_q5: 0,
            },
        ] {
            assert!(display_centroid(&point).is_none());
        }
    }

    #[test]
    fn centroid_revisions_join_only_their_exact_image_and_reset_clears_both_histories() {
        let now = Instant::now();
        let mut history = CameraHistory::default();
        let centroid_frame = |revision| {
            Arc::new(FrameCentroids {
                camera: 0,
                frame_id: 23,
                revision,
                received_at: now,
                groups: Vec::new(),
                invalid: revision > 1,
            })
        };
        remember_centroids(&mut history.centroids, centroid_frame(1));
        history.image(&image(22, now), Duration::from_secs(1));
        assert!(history.join_centroids(22).is_none());
        assert!(history.join_centroids(23).is_none());
        history.image(&image(23, now), Duration::from_secs(2));
        assert_eq!(
            history.join_centroids(23).unwrap().0.time,
            Duration::from_secs(2)
        );
        assert!(history.join_centroids(23).is_none());
        remember_centroids(&mut history.centroids, centroid_frame(2));
        assert!(history.join_centroids(23).unwrap().1.invalid);
        history.clear();
        assert!(history.centroids.is_empty());
        assert!(history.join_centroids(23).is_none());
    }
}
