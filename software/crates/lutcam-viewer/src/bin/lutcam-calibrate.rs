//! Replay stereo UDP clips, archive every product, and fit measured FPGA centroids.

use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;
use std::fs::{self, File};
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::Instant;

use lutcam_viewer::calibration_archive::{CalibrationArchive, trusted_centroid_frame};
use lutcam_viewer::calibration_features::decode_fpga_group;
use lutcam_viewer::centroids;
use lutcam_viewer::poses::PoseCache;
use lutcam_viewer::rerun_feed::CameraCalibration;
use lutcam_viewer::stream::{self, EncodedImage, StreamAssembler};
use serde_json::{Value, json};

type Result<T> = std::result::Result<T, Box<dyn Error + Send + Sync>>;
const CLIP_MAGIC: &[u8; 8] = b"LUTCAP01";
const MAX_PACKET: usize = 65_535;
const PENDING_PAIRS: usize = 32;
const MAX_CHILD_GROUPS: usize = 2048;
const MAX_SELECTED_PAIRS: usize = 120;
const MAX_POSE_HINT_PACKETS: usize = 32_768;
const MIN_SELECTION_SPACING_NS: u64 = 100_000_000;
type ChildFrameRecords = BTreeMap<u32, ChildPacketRecords>;

#[derive(Default)]
struct ChildPacketRecords {
    version: Option<u8>,
    packets: BTreeMap<u32, Vec<u8>>,
    groups: usize,
}

#[derive(Debug, Eq, PartialEq)]
enum ChildPacketStatus {
    New,
    Duplicate,
    Conflict,
}

impl ChildPacketRecords {
    fn insert(
        &mut self,
        version: u8,
        sequence: u32,
        groups: usize,
        bytes: &[u8],
    ) -> ChildPacketStatus {
        if self.version.is_some_and(|previous| previous != version) {
            return ChildPacketStatus::Conflict;
        }
        if let Some(previous) = self.packets.get(&sequence) {
            return if previous == bytes {
                ChildPacketStatus::Duplicate
            } else {
                ChildPacketStatus::Conflict
            };
        }
        if groups > MAX_CHILD_GROUPS.saturating_sub(self.groups) {
            return ChildPacketStatus::Conflict;
        }
        self.version = Some(version);
        self.groups += groups;
        self.packets.insert(sequence, bytes.to_vec());
        ChildPacketStatus::New
    }
}

#[derive(Default)]
struct FrameIndex {
    timestamps: [BTreeMap<u32, u64>; 2],
    invalid_images: [BTreeSet<u32>; 2],
    image_ids: [BTreeSet<u32>; 2],
    pose_ids: [BTreeMap<u32, BTreeSet<u32>>; 2],
    pose_hint_limited: [bool; 2],
}

#[derive(Default)]
struct SelectionPlan {
    frames: BTreeSet<u32>,
    shared_pose_frames: usize,
    pose_hint_limited: [bool; 2],
}

fn spread_indices(count: usize, take: usize) -> Vec<usize> {
    match take.min(count) {
        0 => Vec::new(),
        1 => vec![count / 2],
        take => (0..take).map(|i| i * (count - 1) / (take - 1)).collect(),
    }
}

fn selection_plan(index: &FrameIndex, interval_ns: u64, max_pairs: usize) -> SelectionPlan {
    let limit = max_pairs.min(MAX_SELECTED_PAIRS);
    let mut bins: BTreeMap<u64, Vec<u32>> = BTreeMap::new();
    let shared = |frame: &u32| {
        index.pose_ids[0]
            .get(frame)
            .zip(index.pose_ids[1].get(frame))
            .is_some_and(|(left, right)| !left.is_disjoint(right))
    };
    for frame in index.image_ids[0].intersection(&index.image_ids[1]) {
        if index
            .invalid_images
            .iter()
            .any(|invalid| invalid.contains(frame))
        {
            continue;
        }
        bins.entry(index.timestamps[0][frame] / interval_ns)
            .or_default()
            .push(*frame);
    }
    let shared_bins: Vec<_> = bins
        .iter()
        .filter(|(_, frames)| frames.iter().any(shared))
        .collect();
    let active_bins = spread_indices(shared_bins.len(), limit);
    let mut plan = SelectionPlan {
        pose_hint_limited: index.pose_hint_limited,
        ..Default::default()
    };
    let mut times = BTreeSet::<u64>::new();
    let separated = |timestamp: u64, times: &BTreeSet<u64>| {
        times
            .iter()
            .all(|&other| timestamp.abs_diff(other) >= MIN_SELECTION_SPACING_NS)
    };
    // Give each occupied time bin one choice before allocating a second/third.
    // Accepted poses rank frames only; all fitting coordinates still come from
    // independently decoded measured centroid packets during replay.
    for round in 0..3 {
        for &bin_index in &active_bins {
            if plan.frames.len() == limit {
                break;
            }
            let (&bin, frames) = shared_bins[bin_index];
            let center = bin * interval_ns + interval_ns / 2;
            let candidate = frames
                .iter()
                .filter(|frame| {
                    shared(frame)
                        && !plan.frames.contains(frame)
                        && separated(index.timestamps[0][frame], &times)
                })
                .max_by_key(|frame| {
                    let timestamp = index.timestamps[0][frame];
                    let spacing = if round == 0 {
                        0
                    } else {
                        times
                            .iter()
                            .map(|&other| timestamp.abs_diff(other))
                            .min()
                            .unwrap_or(0)
                    };
                    (
                        spacing,
                        std::cmp::Reverse(timestamp.abs_diff(center)),
                        std::cmp::Reverse(timestamp),
                    )
                });
            if let Some(&frame) = candidate {
                plan.frames.insert(frame);
                times.insert(index.timestamps[0][&frame]);
            }
        }
    }
    plan.shared_pose_frames = plan.frames.len();
    // Keep uniform fallback coverage when the uncalibrated FPGA pose frontend
    // misses a tag. This is not a requirement that every fit input has a pose.
    let fallback: Vec<_> = bins
        .iter()
        .filter(|(_, frames)| !frames.iter().any(shared))
        .filter_map(|(&bin, frames)| {
            let center = bin * interval_ns + interval_ns / 2;
            frames
                .iter()
                .filter(|frame| separated(index.timestamps[0][frame], &times))
                .min_by_key(|frame| index.timestamps[0][frame].abs_diff(center))
                .copied()
        })
        .collect();
    for position in spread_indices(fallback.len(), limit.saturating_sub(plan.frames.len())) {
        let frame = fallback[position];
        if separated(index.timestamps[0][&frame], &times) {
            plan.frames.insert(frame);
            times.insert(index.timestamps[0][&frame]);
        }
    }
    plan
}

#[derive(Default)]
struct CounterCoverage {
    first: Option<u32>,
    ranges: BTreeMap<i64, i64>,
    unique: u64,
    duplicates: u64,
}

impl CounterCoverage {
    fn insert(&mut self, value: u32) {
        let first = *self.first.get_or_insert(value);
        let position = i64::from(value.wrapping_sub(first) as i32);
        let mut start = position;
        let mut end = position;
        if let Some((&previous_start, &previous_end)) = self.ranges.range(..=position).next_back() {
            if previous_end >= position {
                self.duplicates += 1;
                return;
            }
            if previous_end + 1 == position {
                start = previous_start;
                self.ranges.remove(&previous_start);
            }
        }
        if let Some((&next_start, &next_end)) = self.ranges.range(position..).next()
            && next_start == position + 1
        {
            end = next_end;
            self.ranges.remove(&next_start);
        }
        self.ranges.insert(start, end);
        self.unique += 1;
    }

    fn report(&self) -> Value {
        let span = self
            .ranges
            .first_key_value()
            .zip(self.ranges.last_key_value())
            .map_or(0, |((first, _), (_, last))| (last - first + 1) as u64);
        json!({"unique":self.unique,"duplicates":self.duplicates,
               "missing_between_first_and_last":span - self.unique})
    }

    fn frame_report(&self) -> Value {
        let mut result = self.report();
        let repeated = result
            .as_object_mut()
            .unwrap()
            .remove("duplicates")
            .unwrap();
        result["repeated_frame_packets"] = repeated;
        result
    }
}

struct ObservationChoice {
    value: Value,
    ambiguous: bool,
    repeats: usize,
}

fn pending_observation_count(
    choices: &BTreeMap<(u8, u32, u16), ObservationChoice>,
    invalid_frames: &[BTreeSet<u32>; 2],
) -> usize {
    choices
        .iter()
        .filter(|((camera, frame_id, _), choice)| {
            !choice.ambiguous && !invalid_frames[usize::from(*camera)].contains(frame_id)
        })
        .count()
}

fn same_observed_tag(left: &Value, right: &Value) -> bool {
    let points = |value: &Value| -> Option<Vec<[f64; 2]>> {
        value["points"]
            .as_array()?
            .iter()
            .map(|point| Some([point[0].as_f64()?, point[1].as_f64()?]))
            .collect()
    };
    let (Some(left_points), Some(right_points)) = (points(left), points(right)) else {
        return false;
    };
    if left_points.len() != 9 || right_points.len() != 9 {
        return false;
    }
    let diameter = |points: &[[f64; 2]]| {
        points
            .iter()
            .flat_map(|a| points.iter().map(move |b| (a[0] - b[0]).hypot(a[1] - b[1])))
            .fold(0.0_f64, f64::max)
    };
    let size = diameter(&left_points).max(diameter(&right_points));
    let scale = left["scale"]
        .as_f64()
        .unwrap_or(1.0)
        .max(right["scale"].as_f64().unwrap_or(1.0));
    let tolerance = 2.0_f64.max(size * 0.05).max(scale * 0.5);
    let squared = left_points
        .iter()
        .zip(&right_points)
        .map(|(a, b)| (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2))
        .collect::<Vec<_>>();
    squared.iter().sum::<f64>() / 9.0 <= tolerance.powi(2)
        && squared
            .iter()
            .all(|distance| *distance <= (3.0 * tolerance).powi(2))
}

fn prefer_observation(candidate: &Value, current: &Value) -> bool {
    let candidate_scale = candidate["scale"].as_u64().unwrap_or(u64::MAX);
    let current_scale = current["scale"].as_u64().unwrap_or(u64::MAX);
    candidate_scale < current_scale
        || (candidate_scale == current_scale
            && candidate["score"].as_f64().unwrap_or(f64::INFINITY)
                < current["score"].as_f64().unwrap_or(f64::INFINITY))
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
enum ReplayMode {
    #[default]
    Full,
    FitOnly,
    ArchiveOnly,
    MetadataOnly,
}

impl ReplayMode {
    fn extracts_observations(self) -> bool {
        matches!(self, Self::Full | Self::FitOnly)
    }

    fn exports_images(self) -> bool {
        matches!(self, Self::Full | Self::ArchiveOnly)
    }
}

struct Options {
    recording: PathBuf,
    sample_hz: f64,
    max_pairs: usize,
    baseline_mm: f64,
    fit: bool,
    mode: ReplayMode,
}

fn options() -> Result<Options> {
    let mut args = std::env::args().skip(1);
    let mut recording = None;
    let mut sample_hz = 2.0;
    let mut max_pairs = MAX_SELECTED_PAIRS;
    let mut baseline_mm = 120.0;
    let mut fit = true;
    let mut mode = ReplayMode::Full;
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--recording" => {
                recording = Some(PathBuf::from(
                    args.next().ok_or("missing recording directory")?,
                ))
            }
            "--sample-hz" => {
                sample_hz = args.next().ok_or("missing sample rate")?.parse::<f64>()?
            }
            "--max-pairs" => {
                max_pairs = args.next().ok_or("missing pair limit")?.parse::<usize>()?
            }
            "--baseline-mm" => {
                baseline_mm = args.next().ok_or("missing baseline")?.parse::<f64>()?
            }
            "--no-fit" => fit = false,
            "--fit-only" | "--archive-only" | "--metadata-only" => {
                if mode != ReplayMode::Full {
                    return Err("choose only one processing mode".into());
                }
                mode = match argument.as_str() {
                    "--fit-only" => ReplayMode::FitOnly,
                    "--archive-only" => ReplayMode::ArchiveOnly,
                    _ => ReplayMode::MetadataOnly,
                };
            }
            "--help" | "-h" => {
                println!(
                    "lutcam-calibrate --recording DIR [--sample-hz 2] [--max-pairs 120] [--baseline-mm 120] [--no-fit] [--fit-only | --metadata-only | --archive-only]\n\n--fit-only: decode selected pairs and fit, without Rerun or video export\n--metadata-only: save all centroids/poses to Rerun, without decoding images\n--archive-only: export the full Rerun image archive and preview videos on demand"
                );
                std::process::exit(0);
            }
            _ => return Err(format!("unknown argument: {argument}").into()),
        }
    }
    if !sample_hz.is_finite() || !(0.1..=10.0).contains(&sample_hz) {
        return Err("sample rate must be between 0.1 and 10 Hz".into());
    }
    if !(1..=MAX_SELECTED_PAIRS).contains(&max_pairs)
        || !baseline_mm.is_finite()
        || baseline_mm <= 0.0
    {
        return Err("pair limit must be 1..120 and baseline must be positive".into());
    }
    Ok(Options {
        recording: fs::canonicalize(recording.ok_or("--recording DIR is required")?)?,
        sample_hz,
        max_pairs,
        baseline_mm,
        fit: fit && mode.extracts_observations(),
        mode,
    })
}

struct ClipRecord {
    timestamp_ns: u64,
    datagram: Vec<u8>,
}

struct ClipReader<R> {
    input: R,
    previous_timestamp: Option<u64>,
}

impl<R: Read> ClipReader<R> {
    fn new(mut input: R) -> io::Result<Self> {
        let mut magic = [0; 8];
        input.read_exact(&mut magic)?;
        if &magic != CLIP_MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "unsupported UDP clip magic",
            ));
        }
        Ok(Self {
            input,
            previous_timestamp: None,
        })
    }

    fn next_record(&mut self) -> io::Result<Option<ClipRecord>> {
        let mut header = [0; 12];
        // A clean EOF is permitted only between records, never mid-header.
        let count = loop {
            match self.input.read(&mut header[..1]) {
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                result => break result?,
            }
        };
        if count == 0 {
            return Ok(None);
        }
        self.input.read_exact(&mut header[1..])?;
        let timestamp_ns = u64::from_le_bytes(header[..8].try_into().unwrap());
        let length = u32::from_le_bytes(header[8..].try_into().unwrap()) as usize;
        if length > MAX_PACKET {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "oversized UDP clip record",
            ));
        }
        if self
            .previous_timestamp
            .is_some_and(|previous| timestamp_ns < previous)
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "UDP clip timestamps moved backwards",
            ));
        }
        let mut datagram = vec![0; length];
        self.input.read_exact(&mut datagram)?;
        self.previous_timestamp = Some(timestamp_ns);
        Ok(Some(ClipRecord {
            timestamp_ns,
            datagram,
        }))
    }
}

#[derive(Default)]
struct PairParts {
    timestamps: [Option<u64>; 2],
    images: [Option<EncodedImage>; 2],
    children: [Vec<ChildGroup>; 2],
}

struct ChildGroup {
    timestamp_ns: u64,
    sequence: u32,
    group_index: usize,
    tuples: [u64; 9],
}

struct Preview {
    process: Option<Child>,
    previous: Vec<u8>,
    next_bin: u64,
    repeated_frames: u64,
    black_frames: u64,
    has_real_frame: bool,
}

impl Preview {
    fn new(path: &Path, sample_hz: f64) -> Result<Self> {
        let process = Command::new("ffmpeg")
            .args([
                "-loglevel",
                "error",
                "-nostdin",
                "-y",
                "-f",
                "rawvideo",
                "-pixel_format",
                "gray",
                "-video_size",
                "1280x800",
                "-framerate",
            ])
            .arg(sample_hz.to_string())
            .args(["-i", "pipe:0", "-an", "-c:v", "ffv1", "-level", "3"])
            .arg(path)
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .spawn()?;
        Ok(Self {
            process: Some(process),
            previous: Vec::new(),
            next_bin: 0,
            repeated_frames: 0,
            black_frames: 0,
            has_real_frame: false,
        })
    }

    fn fill_to(&mut self, bin: u64) -> Result<()> {
        if bin > 600 {
            return Err("preview timeline exceeds bounded 600-frame limit".into());
        }
        while self.next_bin < bin {
            if self.previous.is_empty() {
                self.previous.resize(stream::WIDTH * stream::HEIGHT, 0);
            }
            self.process
                .as_mut()
                .unwrap()
                .stdin
                .as_mut()
                .unwrap()
                .write_all(&self.previous)?;
            self.next_bin += 1;
            if !self.has_real_frame {
                self.black_frames += 1;
            } else {
                self.repeated_frames += 1;
            }
        }
        Ok(())
    }

    fn push(&mut self, bin: u64, pixels: &[u8]) -> Result<()> {
        self.fill_to(bin)?;
        self.process
            .as_mut()
            .unwrap()
            .stdin
            .as_mut()
            .unwrap()
            .write_all(pixels)?;
        self.previous.clear();
        self.previous.extend_from_slice(pixels);
        self.has_real_frame = true;
        self.next_bin += 1;
        Ok(())
    }

    fn finish(&mut self, bins: u64) -> Result<()> {
        self.fill_to(bins.max(self.next_bin))?;
        let mut process = self.process.take().unwrap();
        drop(process.stdin.take());
        let status = process.wait()?;
        if !status.success() {
            return Err(format!("FFV1 preview encoder failed ({status})").into());
        }
        Ok(())
    }
}

impl Drop for Preview {
    fn drop(&mut self) {
        if let Some(mut process) = self.process.take() {
            drop(process.stdin.take());
            let _ = process.kill();
            let _ = process.wait();
        }
    }
}

struct Replay {
    mode: ReplayMode,
    assemblers: [StreamAssembler; 2],
    pending: BTreeMap<u32, PairParts>,
    bins: BTreeSet<u64>,
    selected: BTreeMap<u32, u64>,
    plan: Option<SelectionPlan>,
    interval_ns: u64,
    max_pairs: usize,
    packets: [u64; 2],
    nonimage_packets: [u64; 2],
    invalid_packets: [u64; 2],
    decode_failures: [u64; 2],
    observations: usize,
    rejected_groups: BTreeMap<String, usize>,
    child_groups: [u64; 2],
    child_duplicates: [u64; 2],
    child_seen: [ChildFrameRecords; 2],
    invalid_child_frames: [BTreeSet<u32>; 2],
    invalid_image_frames: [BTreeSet<u32>; 2],
    previews: Option<[Preview; 2]>,
    archive: Option<CalibrationArchive>,
    archived: [BTreeSet<u32>; 2],
    frame_timestamps: [BTreeMap<u32, u64>; 2],
    sequences: [[CounterCoverage; 3]; 2],
    image_frames: [CounterCoverage; 2],
    observation_choices: BTreeMap<(u8, u32, u16), ObservationChoice>,
    output: BufWriter<File>,
    diagnostics: BufWriter<File>,
}

impl Replay {
    fn new(
        directory: &Path,
        sample_hz: f64,
        max_pairs: usize,
        frame_timestamps: [BTreeMap<u32, u64>; 2],
        invalid_image_frames: [BTreeSet<u32>; 2],
        calibrations: [CameraCalibration; 2],
        mode: ReplayMode,
    ) -> Result<Self> {
        Ok(Self {
            mode,
            assemblers: [StreamAssembler::new(0, 16), StreamAssembler::new(1, 16)],
            pending: BTreeMap::new(),
            bins: BTreeSet::new(),
            selected: BTreeMap::new(),
            plan: None,
            interval_ns: (1_000_000_000.0 / sample_hz).round() as u64,
            max_pairs: max_pairs.min(MAX_SELECTED_PAIRS),
            packets: [0; 2],
            nonimage_packets: [0; 2],
            invalid_packets: [0; 2],
            decode_failures: [0; 2],
            observations: 0,
            rejected_groups: BTreeMap::new(),
            child_groups: [0; 2],
            child_duplicates: [0; 2],
            child_seen: std::array::from_fn(|_| BTreeMap::new()),
            invalid_child_frames: std::array::from_fn(|_| BTreeSet::new()),
            invalid_image_frames,
            previews: if mode.exports_images() {
                Some([
                    Preview::new(&directory.join("camera0.mkv"), sample_hz)?,
                    Preview::new(&directory.join("camera1.mkv"), sample_hz)?,
                ])
            } else {
                None
            },
            archive: match mode {
                ReplayMode::FitOnly => None,
                ReplayMode::MetadataOnly => Some(CalibrationArchive::new_metadata(
                    &directory.join("calibration.rrd"),
                    calibrations,
                )?),
                _ => Some(CalibrationArchive::new(
                    &directory.join("calibration.rrd"),
                    calibrations,
                )?),
            },
            archived: std::array::from_fn(|_| BTreeSet::new()),
            frame_timestamps,
            sequences: std::array::from_fn(|_| std::array::from_fn(|_| CounterCoverage::default())),
            image_frames: std::array::from_fn(|_| CounterCoverage::default()),
            observation_choices: BTreeMap::new(),
            output: BufWriter::with_capacity(
                1024 * 1024,
                File::create(directory.join(if mode.extracts_observations() {
                    "observations.jsonl"
                } else {
                    "archive-observations.jsonl"
                }))?,
            ),
            diagnostics: BufWriter::with_capacity(
                1024 * 1024,
                File::create(directory.join(if mode.extracts_observations() {
                    "extraction-frames.jsonl"
                } else {
                    "archive-extraction-frames.jsonl"
                }))?,
            ),
        })
    }

    fn record(&mut self, camera: usize, record: ClipRecord) -> Result<()> {
        self.packets[camera] += 1;
        if record.datagram.len() >= 32
            && record.datagram[..4] == 0x4c55_5443_u32.to_be_bytes()
            && (0..=3).contains(&record.datagram[4])
            && record.datagram[5] == camera as u8
            && (1..=3).contains(&record.datagram[6])
            && record.datagram[7] == 32
            && u32::from_be_bytes(record.datagram[20..24].try_into().unwrap()) as usize
                == record.datagram.len() - 32
        {
            let sequence = u32::from_be_bytes(record.datagram[12..16].try_into().unwrap());
            self.sequences[camera][usize::from(record.datagram[6]) - 1].insert(sequence);
        }
        if record.datagram.len() >= 8 && record.datagram[6] == 2 {
            self.children(camera, record)?;
            self.trim_pending();
            return Ok(());
        }
        if record.datagram.len() >= 8 && record.datagram[6] != 1 {
            self.nonimage_packets[camera] += 1;
            if record.datagram[6] == 3 && record.datagram.len() >= 12 {
                let frame_id = u32::from_be_bytes(record.datagram[8..12].try_into().unwrap());
                let timestamp = self.frame_timestamps[camera]
                    .get(&frame_id)
                    .copied()
                    .unwrap_or(record.timestamp_ns);
                if let Some(archive) = &mut self.archive {
                    archive.poses(&record.datagram, camera as u8, timestamp)?;
                }
            }
            return Ok(());
        }
        match stream::parse_datagram(&record.datagram) {
            Ok(packet) if usize::from(packet.camera) == camera => {
                self.image_frames[camera].insert(packet.frame_id);
                // Metadata export never decodes images. Fit-only still feeds
                // every fragment into the assembler: the stream uses the following
                // frame to finish the previous image before the age eviction.
                // candidate() avoids decoding all unselected frames.
                if self.mode == ReplayMode::MetadataOnly {
                    return Ok(());
                }
                let frame = self.pending.entry(packet.frame_id).or_default();
                frame.timestamps[camera].get_or_insert(record.timestamp_ns);
            }
            _ => {
                self.invalid_packets[camera] += 1;
                return Ok(());
            }
        }
        let candidates = match self.assemblers[camera].push(&record.datagram) {
            Ok(candidates) => candidates,
            Err(_) => {
                self.invalid_packets[camera] += 1;
                Vec::new()
            }
        };
        for image in candidates {
            self.candidate(camera, image)?;
        }
        self.trim_pending();
        Ok(())
    }

    fn trim_pending(&mut self) {
        let unplanned = |frame: &&u32| {
            self.plan
                .as_ref()
                .is_none_or(|plan| !plan.frames.contains(frame))
        };
        while self.pending.keys().filter(unplanned).count() > PENDING_PAIRS {
            let oldest = self
                .pending
                .iter()
                .filter(|(frame, _)| unplanned(frame))
                .min_by_key(|(_, pair)| {
                    pair.timestamps
                        .iter()
                        .flatten()
                        .copied()
                        .min()
                        .unwrap_or(u64::MAX)
                })
                .map(|(&id, _)| id)
                .unwrap();
            self.pending.remove(&oldest);
            if !self.selected.contains_key(&oldest) {
                self.child_seen[0].remove(&oldest);
                self.child_seen[1].remove(&oldest);
            }
        }
    }

    fn children(&mut self, camera: usize, record: ClipRecord) -> Result<()> {
        let packet = match centroids::parse_datagram(&record.datagram) {
            Ok(packet) if usize::from(packet.camera) == camera => packet,
            _ => {
                self.invalid_packets[camera] += 1;
                let frame_id = trusted_centroid_frame(&record.datagram, camera as u8);
                let timestamp = frame_id
                    .and_then(|frame| self.frame_timestamps[camera].get(&frame).copied())
                    .unwrap_or(record.timestamp_ns);
                if let Some(archive) = &mut self.archive {
                    archive.malformed_centroids(&record.datagram, camera as u8, timestamp)?;
                }
                if let Some(frame_id) = frame_id {
                    self.invalid_child_frames[camera].insert(frame_id);
                    self.child_seen[camera].remove(&frame_id);
                }
                return Ok(());
            }
        };
        let timestamp = self.frame_timestamps[camera]
            .get(&packet.frame_id)
            .copied()
            .unwrap_or(record.timestamp_ns);
        if let Some(archive) = &mut self.archive {
            archive.centroids(&packet, timestamp)?;
        }
        self.child_groups[camera] += packet.groups.len() as u64;
        if self.mode == ReplayMode::MetadataOnly
            || (self.mode == ReplayMode::FitOnly
                && self
                    .plan
                    .as_ref()
                    .is_some_and(|plan| !plan.frames.contains(&packet.frame_id)))
        {
            return Ok(());
        }
        if self.invalid_child_frames[camera].contains(&packet.frame_id) {
            return Ok(());
        }
        // A packet sequence identifies the entire batch, not individual
        // groups. Added/removed groups and a protocol change must retract all
        // observations from this frame, including already decoded choices.
        match self.child_seen[camera]
            .entry(packet.frame_id)
            .or_default()
            .insert(
                packet.version,
                packet.sequence,
                packet.groups.len(),
                &record.datagram,
            ) {
            ChildPacketStatus::New => {}
            ChildPacketStatus::Duplicate => {
                self.child_duplicates[camera] += packet.groups.len() as u64;
                return Ok(());
            }
            ChildPacketStatus::Conflict => {
                self.invalid_packets[camera] += 1;
                self.invalid_child_frames[camera].insert(packet.frame_id);
                self.child_seen[camera].remove(&packet.frame_id);
                if let Some(archive) = &mut self.archive {
                    archive.invalidate_centroids(camera as u8, packet.frame_id, timestamp)?;
                }
                return Ok(());
            }
        }
        // Bound packet history to the pending frame lifetime.
        self.pending.entry(packet.frame_id).or_default().timestamps[camera]
            .get_or_insert(timestamp);
        for group in packet.groups {
            let group_index = usize::from(group.group_index);
            let tuples = std::array::from_fn(|index| {
                let centroid = &group.centroids[index];
                u64::from(centroid.valid)
                    | (u64::from(centroid.x_q5) << 1)
                    | (u64::from(centroid.y_q5) << 17)
            });
            let group = ChildGroup {
                timestamp_ns: record.timestamp_ns,
                sequence: packet.sequence,
                group_index,
                tuples,
            };
            if let Some(&timestamp_ns) = self.selected.get(&packet.frame_id) {
                self.write_group(camera, packet.frame_id, timestamp_ns, group)?;
            } else {
                let pair = self.pending.entry(packet.frame_id).or_default();
                pair.timestamps[camera].get_or_insert(timestamp);
                if pair.children[camera].len() >= MAX_CHILD_GROUPS {
                    return Err("more than 2048 child groups in one camera frame".into());
                }
                pair.children[camera].push(group);
            }
        }
        Ok(())
    }

    fn write_group(
        &mut self,
        camera: usize,
        frame_id: u32,
        timestamp_ns: u64,
        group: ChildGroup,
    ) -> Result<()> {
        if !self.mode.extracts_observations() {
            return Ok(());
        }
        let result = decode_fpga_group(group.tuples);
        let diagnostic = match result {
            Ok(detection) => {
                let mut observation = detection.to_json();
                observation["camera"] = json!(camera);
                observation["frame_id"] = json!(frame_id);
                observation["timestamp_ns"] = json!(timestamp_ns);
                observation["centroid_timestamp_ns"] = json!(group.timestamp_ns);
                observation["sequence"] = json!(group.sequence);
                observation["group_index"] = json!(group.group_index);
                observation["pixel_coordinates"] = json!("native_unflipped_integer_centers");
                let key = (camera as u8, frame_id, detection.id);
                if let Some(choice) = self.observation_choices.get_mut(&key) {
                    choice.repeats += 1;
                    if !same_observed_tag(&choice.value, &observation) {
                        choice.ambiguous = true;
                    } else if prefer_observation(&observation, &choice.value) {
                        choice.value = observation;
                    }
                } else {
                    self.observation_choices.insert(
                        key,
                        ObservationChoice {
                            value: observation,
                            ambiguous: false,
                            repeats: 1,
                        },
                    );
                }
                json!({"tag_id":detection.id, "score":detection.best_score})
            }
            Err(reason) => {
                let reason = format!("{reason:?}");
                *self.rejected_groups.entry(reason.clone()).or_default() += 1;
                json!({"rejection":reason})
            }
        };
        serde_json::to_writer(
            &mut self.diagnostics,
            &json!({
                "camera":camera, "frame_id":frame_id, "timestamp_ns":timestamp_ns,
                "sequence":group.sequence, "group_index":group.group_index, "result":diagnostic,
            }),
        )?;
        writeln!(self.diagnostics)?;
        Ok(())
    }

    fn candidate(&mut self, camera: usize, image: EncodedImage) -> Result<()> {
        let frame_id = image.frame_id;
        if self.invalid_image_frames[camera].contains(&frame_id) {
            return Ok(());
        }
        if self.mode.exports_images() && !self.archived[camera].contains(&frame_id) {
            match image.clone().decode() {
                Ok(decoded) => {
                    let timestamp = self.frame_timestamps[camera]
                        .get(&frame_id)
                        .copied()
                        .unwrap_or(0);
                    self.archive.as_mut().unwrap().image(&decoded, timestamp)?;
                    self.archived[camera].insert(frame_id);
                }
                Err(_) => {
                    self.decode_failures[camera] += 1;
                }
            }
        }
        if self.selected.len() >= self.max_pairs
            || self.selected.contains_key(&frame_id)
            || self
                .plan
                .as_ref()
                .is_some_and(|plan| !plan.frames.contains(&frame_id))
        {
            return Ok(());
        }
        let pair = self.pending.entry(frame_id).or_default();
        pair.images[camera] = Some(image);
        let Some(timestamp_ns) = pair.timestamps[0] else {
            return Ok(());
        };
        let bin = timestamp_ns / self.interval_ns;
        if (self.plan.is_none() && self.bins.last().is_some_and(|&last| bin <= last))
            || pair.images.iter().any(Option::is_none)
        {
            return Ok(());
        }
        // Speculative image candidates can be amended by a reordered tail. Do not
        // consume a time bin until both independent codec decodes succeed.
        let decoded = [
            pair.images[0].as_ref().unwrap().clone().decode(),
            pair.images[1].as_ref().unwrap().clone().decode(),
        ];
        if decoded.iter().any(std::result::Result::is_err) {
            for (camera, result) in decoded.iter().enumerate() {
                if result.is_err() {
                    self.decode_failures[camera] += 1;
                }
            }
            return Ok(());
        }
        self.selected.insert(frame_id, timestamp_ns);
        let children = std::mem::take(&mut pair.children);
        if self.bins.last().is_none_or(|&last| bin > last) {
            self.bins.insert(bin);
            if let Some(previews) = &mut self.previews {
                for (camera, decoded) in decoded.into_iter().enumerate() {
                    previews[camera].push(bin, &decoded.unwrap().pixels)?;
                }
            }
        }
        for (camera, groups) in children.into_iter().enumerate() {
            for group in groups {
                self.write_group(camera, frame_id, timestamp_ns, group)?;
            }
        }
        self.output.flush()?;
        self.diagnostics.flush()?;
        self.pending.remove(&frame_id);
        println!("{}", self.extraction_progress(frame_id));
        Ok(())
    }

    fn extraction_progress(&self, frame_id: u32) -> Value {
        // These distinct choices are eligible now, but a late conflicting
        // packet or second same-ID instance can still retract them. The final
        // report counts only observations actually written by finish().
        json!({
            "stage": "extracting", "sampled_pairs": self.selected.len(),
            "pending_observations": pending_observation_count(
                &self.observation_choices, &self.invalid_child_frames,
            ),
            "observations_final": false, "frame_id": frame_id,
        })
    }

    fn finish(&mut self, elapsed_ns: u64) -> Result<()> {
        for camera in 0..2 {
            for image in self.assemblers[camera].poll() {
                self.candidate(camera, image)?;
            }
        }
        for (&(camera, frame_id, _), choice) in &self.observation_choices {
            if self.invalid_child_frames[usize::from(camera)].contains(&frame_id) {
                *self
                    .rejected_groups
                    .entry("conflicting_child_packet_frame".into())
                    .or_default() += 1;
                continue;
            }
            if choice.ambiguous {
                *self
                    .rejected_groups
                    .entry("ambiguous_same_id_instances".into())
                    .or_default() += 1;
                continue;
            }
            serde_json::to_writer(&mut self.output, &choice.value)?;
            writeln!(self.output)?;
            self.observations += 1;
        }
        self.output.flush()?;
        self.diagnostics.flush()?;
        let bins = elapsed_ns.div_ceil(self.interval_ns).max(1);
        if let Some(previews) = &mut self.previews {
            for preview in previews {
                preview.finish(bins)?;
            }
        }
        if let Some(archive) = &mut self.archive {
            archive.finish()?;
        }
        Ok(())
    }

    fn report(&self) -> Value {
        let preview = self.previews.as_ref().map(|previews| json!({
            "files":["camera0.mkv","camera1.mkv"], "codec":"ffv1", "coordinates":"native_unflipped",
            "sample_hz":1_000_000_000.0 / self.interval_ns as f64,
            "gap_policy":"repeat previous sampled frame; black before first sample",
            "repeated_frames":[previews[0].repeated_frames,previews[1].repeated_frames],
            "leading_black_frames":[previews[0].black_frames,previews[1].black_frames],
            "total_frames":[previews[0].next_bin,previews[1].next_bin],
        }));
        let mut report = json!({
            "format":"lutcam-observed-lftag-v1", "complete":true,
            "pixel_coordinates":"native_unflipped_integer_centers", "image_size":[1280,800],
            "measurement_source":"fpga_centroids", "pose_feedback":false,
            "mass_policy":"rank_only_sorted", "mass_spread_checked":false,
            "baseline_mass_ties_observable":false,
            "sampled_pairs":self.selected.len(), "frame_timestamps_ns":self.selected,
            "observations":self.observations, "sample_interval_ns":self.interval_ns,
            "selection":{
                "policy":"shared_pose_priority_with_uniform_fallback",
                "planned_pairs":self.plan.as_ref().map(|plan| plan.frames.len()),
                "planned_shared_pose_pairs":self.plan.as_ref().map(|plan| plan.shared_pose_frames),
                "pose_hint_limited":self.plan.as_ref().map(|plan| plan.pose_hint_limited),
                "maximum_pairs":self.max_pairs, "maximum_shared_pairs_per_bin":3,
                "minimum_spacing_ns":MIN_SELECTION_SPACING_NS,
                "pose_used_for_coordinates":false,
            },
            "packets":self.packets, "nonimage_packets":self.nonimage_packets,
            "packet_sequences":[self.sequences[0].iter().map(CounterCoverage::report).collect::<Vec<_>>(),
                                  self.sequences[1].iter().map(CounterCoverage::report).collect::<Vec<_>>()],
            "packet_sequence_order":["image","centroids","pose"],
            "image_frame_coverage":[self.image_frames[0].frame_report(),self.image_frames[1].frame_report()],
            "complete_images_archived":[self.archived[0].len(),self.archived[1].len()],
            "invalid_product_packets":self.invalid_packets, "decode_attempt_failures":self.decode_failures,
            "candidate_rejections":self.rejected_groups,
            "child_groups":self.child_groups, "duplicate_child_groups":self.child_duplicates,
            "conflicting_child_frames":self.invalid_child_frames,
            "conflicting_image_frames":self.invalid_image_frames,
            "archive_semantics":"exact received datagrams; network loss is not reconstructed",
            "rerun_archive":self.archive.as_ref().map(|_| "calibration.rrd"),
            "preview":preview,
            "incomplete_frame_evictions":[self.assemblers[0].stats().incomplete_evictions,
                                           self.assemblers[1].stats().incomplete_evictions],
        });
        report["processing_mode"] = json!(format!("{:?}", self.mode));
        report["image_validation_scope"] = json!(if self.mode == ReplayMode::FitOnly {
            "selected_calibration_frames"
        } else if self.mode == ReplayMode::MetadataOnly {
            "packet_headers_only"
        } else {
            "all_frames"
        });
        report["child_conflict_validation_scope"] = json!(if self.mode == ReplayMode::FitOnly {
            "selected_calibration_frames"
        } else if self.mode == ReplayMode::MetadataOnly {
            "rerun_archive_history"
        } else {
            "all_frames"
        });
        report
    }
}

fn run(options: Options) -> Result<()> {
    let started = Instant::now();
    let manifest: Value =
        serde_json::from_reader(File::open(options.recording.join("manifest.json"))?)?;
    if manifest["format"] != "lutcam-udp-v1" || manifest["complete"] != true {
        return Err(
            "recording is incomplete or uses an unsupported format; source clips were preserved"
                .into(),
        );
    }
    let (calibrations, known_calibration) = recording_calibrations(&manifest)?;
    let index = index_frames(&options.recording)?;
    let index_seconds = started.elapsed().as_secs_f64();
    let plan = selection_plan(
        &index,
        (1_000_000_000.0 / options.sample_hz).round() as u64,
        options.max_pairs,
    );
    println!(
        "{}",
        json!({"stage":"selection", "index_seconds":index_seconds, "planned_pairs":plan.frames.len(),
        "shared_pose_pairs":plan.shared_pose_frames, "pose_hint_limited":plan.pose_hint_limited})
    );
    let FrameIndex {
        timestamps: frame_timestamps,
        invalid_images: invalid_image_frames,
        ..
    } = index;
    let mut readers = [
        ClipReader::new(BufReader::with_capacity(
            1024 * 1024,
            File::open(options.recording.join("camera0.udp"))?,
        ))?,
        ClipReader::new(BufReader::with_capacity(
            1024 * 1024,
            File::open(options.recording.join("camera1.udp"))?,
        ))?,
    ];
    let mut next = [readers[0].next_record()?, readers[1].next_record()?];
    let mut replay = Replay::new(
        &options.recording,
        options.sample_hz,
        options.max_pairs,
        frame_timestamps,
        invalid_image_frames,
        calibrations,
        options.mode,
    )?;
    replay.plan = Some(plan);
    let mut elapsed_ns = 0;
    while next.iter().any(Option::is_some) {
        let camera = match (&next[0], &next[1]) {
            (Some(left), Some(right)) => usize::from(right.timestamp_ns < left.timestamp_ns),
            (Some(_), None) => 0,
            _ => 1,
        };
        let record = next[camera].take().unwrap();
        elapsed_ns = elapsed_ns.max(record.timestamp_ns);
        replay.record(camera, record)?;
        next[camera] = readers[camera].next_record()?;
    }
    replay.finish(elapsed_ns)?;
    let mut report = replay.report();
    report["display_calibration_from_manifest"] = json!(known_calibration);
    report["display_calibration_nominal_fallback"] = json!(!known_calibration);
    report["index_seconds"] = json!(index_seconds);
    report["extraction_seconds"] = json!(started.elapsed().as_secs_f64());
    fs::write(
        options
            .recording
            .join(if options.mode.extracts_observations() {
                "extraction.json"
            } else {
                "archive-extraction.json"
            }),
        serde_json::to_vec_pretty(&report)?,
    )?;
    println!("{}", json!({"stage":"extracted", "report":report}));
    if options.fit {
        println!(
            "{}",
            json!({"stage":"fitting", "elapsed_seconds":started.elapsed().as_secs_f64()})
        );
        let repository = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../gateware");
        let status = Command::new("python3")
            .current_dir(repository)
            .env("PYTHONDONTWRITEBYTECODE", "1")
            .env("OPENBLAS_NUM_THREADS", "1")
            .env("OMP_NUM_THREADS", "1")
            .args(["-m", "tools.lutcam.calibrate", "--observations"])
            .arg(options.recording.join("observations.jsonl"))
            .arg("--output")
            .arg(options.recording.join("calibration.json"))
            .arg("--baseline-mm")
            .arg(options.baseline_mm.to_string())
            .status()?;
        if !status.success() {
            return Err(format!("calibration fitter failed ({status}); recording and measured observations were preserved").into());
        }
    }
    println!(
        "{}",
        json!({"stage":"finished", "elapsed_seconds":started.elapsed().as_secs_f64()})
    );
    Ok(())
}

fn index_frames(directory: &Path) -> Result<FrameIndex> {
    let mut timestamps: [BTreeMap<u32, u64>; 2] = std::array::from_fn(|_| BTreeMap::new());
    let mut invalid: [BTreeSet<u32>; 2] = std::array::from_fn(|_| BTreeSet::new());
    let mut image_ids: [BTreeSet<u32>; 2] = Default::default();
    let mut pose_ids: [BTreeMap<u32, BTreeSet<u32>>; 2] = Default::default();
    let mut pose_hint_limited = [false; 2];
    for (camera, frames) in timestamps.iter_mut().enumerate() {
        let mut assembler = StreamAssembler::new(camera as u8, 16);
        let mut poses: BTreeMap<u32, PoseCache> = BTreeMap::new();
        let pose_clock = Instant::now();
        let mut pose_packets = 0;
        let mut fallback = BTreeMap::new();
        let mut input = ClipReader::new(BufReader::with_capacity(
            1024 * 1024,
            File::open(directory.join(format!("camera{camera}.udp")))?,
        ))?;
        while let Some(record) = input.next_record()? {
            let bytes = &record.datagram;
            if bytes.len() >= 32
                && bytes[..4] == 0x4c55_5443_u32.to_be_bytes()
                && bytes[5] == camera as u8
                && bytes[6] == 3
                && bytes[7] == 32
            {
                pose_packets += 1;
                if pose_packets <= MAX_POSE_HINT_PACKETS {
                    let frame = u32::from_be_bytes(bytes[8..12].try_into().unwrap());
                    // A fixed per-frame clock disables live TTL, while strict
                    // whole-packet duplicate/version conflicts remain sticky.
                    let _ = poses
                        .entry(frame)
                        .or_insert_with(|| PoseCache::new(camera as u8))
                        .push(bytes, pose_clock);
                } else if !pose_hint_limited[camera] {
                    poses.clear();
                    pose_hint_limited[camera] = true;
                }
            }
            let auxiliary_frame = trusted_centroid_frame(bytes, camera as u8).or_else(|| {
                (bytes.len() >= 32
                    && bytes[..4] == 0x4c55_5443_u32.to_be_bytes()
                    && (0..=3).contains(&bytes[4])
                    && bytes[5] == camera as u8
                    && bytes[6] == 3
                    && bytes[7] == 32)
                    .then(|| u32::from_be_bytes(bytes[8..12].try_into().unwrap()))
            });
            if let Some(frame) = auxiliary_frame {
                fallback.entry(frame).or_insert(record.timestamp_ns);
                if fallback.len() > 50_000 {
                    return Err("recording exceeds bounded product-frame index".into());
                }
            }
            if let Ok(packet) = stream::parse_datagram(&record.datagram)
                && usize::from(packet.camera) == camera
            {
                image_ids[camera].insert(packet.frame_id);
                frames.entry(packet.frame_id).or_insert(record.timestamp_ns);
                if assembler.push(&record.datagram).is_err() {
                    invalid[camera].insert(packet.frame_id);
                }
                if frames.len() > 50_000 {
                    return Err("recording exceeds bounded 50,000-frame replay limit".into());
                }
            }
        }
        for (frame_id, cache) in poses {
            for frame in cache.frames().into_iter().filter(|frame| !frame.invalid) {
                pose_ids[camera].insert(
                    frame_id,
                    frame.detections.iter().map(|tag| tag.tag_id).collect(),
                );
            }
        }
        // An image timestamp is authoritative. A frame with only auxiliary
        // packets still needs one stable timestamp for every archived record.
        for (frame, timestamp) in fallback {
            frames.entry(frame).or_insert(timestamp);
        }
    }
    Ok(FrameIndex {
        timestamps,
        invalid_images: invalid,
        image_ids,
        pose_ids,
        pose_hint_limited,
    })
}

fn recording_calibrations(manifest: &Value) -> Result<([CameraCalibration; 2], bool)> {
    let calibration = &manifest["metadata"]["calibration"];
    if calibration.is_null() {
        return Ok(([CameraCalibration::default(); 2], false));
    }
    let cameras = calibration["cameras"]
        .as_array()
        .filter(|cameras| cameras.len() == 2)
        .ok_or("recording calibration must contain exactly two camera intrinsics")?;
    let width = calibration["tag_width"].as_f64().unwrap_or(1.0) as f32;
    let mut result = [CameraCalibration::default(); 2];
    for camera in 0..2 {
        let get = |key: &str| {
            cameras[camera][key]
                .as_f64()
                .map(|value| value as f32)
                .ok_or_else(|| format!("recording camera{camera} calibration lacks {key}"))
        };
        result[camera] = CameraCalibration {
            fx: get("fx")?,
            fy: get("fy")?,
            cx: get("cx")?,
            cy: get("cy")?,
            tag_width: width,
        };
        result[camera].validate()?;
    }
    Ok((result, true))
}

fn main() {
    if let Err(error) = options().and_then(run) {
        eprintln!("lutcam-calibrate: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn selection_index(samples: &[(u32, u64, bool)]) -> FrameIndex {
        let mut index = FrameIndex::default();
        for &(frame, timestamp, shared) in samples {
            for camera in 0..2 {
                index.timestamps[camera].insert(frame, timestamp);
                index.image_ids[camera].insert(frame);
                if shared {
                    index.pose_ids[camera].insert(frame, BTreeSet::from([4269]));
                }
            }
        }
        index
    }

    #[test]
    fn selection_keeps_three_spaced_shared_frames_and_ordinary_fallback() {
        let mut index = selection_index(&[
            (10, 0, false),
            (11, 100_000_000, true),
            (12, 250_000_000, true),
            (13, 400_000_000, true),
            (20, 1_000_000_000, false),
            (21, 1_250_000_000, false),
        ]);
        // Auxiliary-only frames must not masquerade as complete stereo images.
        for camera in 0..2 {
            index.timestamps[camera].insert(99, 200_000_000);
            index.pose_ids[camera].insert(99, BTreeSet::from([4269]));
        }
        let plan = selection_plan(&index, 500_000_000, 120);
        assert_eq!(plan.frames, BTreeSet::from([11, 12, 13, 21]));
        assert_eq!(plan.shared_pose_frames, 3);
    }

    #[test]
    fn selection_budget_spreads_over_time_and_counter_wrap() {
        let samples: Vec<_> = (0..200_u32)
            .map(|number| {
                (
                    (u32::MAX - 1).wrapping_add(number),
                    u64::from(number) * 500_000_000 + 250_000_000,
                    true,
                )
            })
            .collect();
        let index = selection_index(&samples);
        let plan = selection_plan(&index, 500_000_000, 1000);
        assert_eq!(plan.frames.len(), 120);
        assert!(plan.frames.contains(&samples[0].0));
        assert!(plan.frames.contains(&samples[199].0));
        assert_eq!(
            selection_plan(&index, 500_000_000, 2).frames,
            BTreeSet::from([samples[0].0, samples[199].0])
        );
    }

    fn clip(records: &[(u64, &[u8])]) -> Vec<u8> {
        let mut bytes = CLIP_MAGIC.to_vec();
        for (timestamp, data) in records {
            bytes.extend(timestamp.to_le_bytes());
            bytes.extend((data.len() as u32).to_le_bytes());
            bytes.extend_from_slice(data);
        }
        bytes
    }

    #[test]
    fn clip_preserves_datagrams_and_distinguishes_clean_eof() {
        let bytes = clip(&[(9, b"abc"), (9, b""), (12, b"def")]);
        let mut reader = ClipReader::new(Cursor::new(bytes)).unwrap();
        let first = reader.next_record().unwrap().unwrap();
        assert_eq!(first.timestamp_ns, 9);
        assert_eq!(first.datagram, b"abc");
        assert!(reader.next_record().unwrap().unwrap().datagram.is_empty());
        assert_eq!(reader.next_record().unwrap().unwrap().datagram, b"def");
        assert!(reader.next_record().unwrap().is_none());
    }

    #[test]
    fn truncated_headers_payloads_and_backwards_time_are_errors() {
        for truncate in 1..15 {
            let mut bytes = clip(&[(12, b"abc")]);
            bytes.truncate(bytes.len() - truncate);
            let mut reader = ClipReader::new(Cursor::new(bytes)).unwrap();
            assert!(reader.next_record().is_err());
        }
        let mut reader = ClipReader::new(Cursor::new(clip(&[(12, b"a"), (11, b"b")]))).unwrap();
        reader.next_record().unwrap();
        assert_eq!(
            reader.next_record().err().unwrap().kind(),
            io::ErrorKind::InvalidData
        );
    }

    #[test]
    fn oversized_record_is_rejected_before_payload_allocation() {
        let mut bytes = CLIP_MAGIC.to_vec();
        bytes.extend(0_u64.to_le_bytes());
        bytes.extend(u32::MAX.to_le_bytes());
        let mut reader = ClipReader::new(Cursor::new(bytes)).unwrap();
        assert_eq!(
            reader.next_record().err().unwrap().kind(),
            io::ErrorKind::InvalidData
        );
    }

    #[test]
    fn counter_coverage_handles_reorder_duplicates_and_wrap_without_false_loss() {
        let mut coverage = CounterCoverage::default();
        for sequence in [u32::MAX - 1, 1, u32::MAX, 0, 1, 3] {
            coverage.insert(sequence);
        }
        assert_eq!(coverage.report()["unique"], 5);
        assert_eq!(coverage.report()["duplicates"], 1);
        assert_eq!(coverage.report()["missing_between_first_and_last"], 1);
        coverage.insert(2);
        assert_eq!(coverage.report()["missing_between_first_and_last"], 0);
    }

    #[test]
    fn multiscale_dedup_keeps_measured_fine_points_but_rejects_separate_instances() {
        let observation = |offset: f64, scale: u32, score: f64| {
            json!({
                "points":(0..9).map(|index| [offset+(index%3) as f64*20.0,(index/3) as f64*20.0]).collect::<Vec<_>>(),
                "scale":scale,"score":score,
            })
        };
        let raw = observation(0.0, 1, 0.02);
        let ll2 = observation(0.3, 4, 0.01);
        assert!(same_observed_tag(&raw, &ll2));
        assert!(prefer_observation(&raw, &ll2));
        assert!(!same_observed_tag(&raw, &observation(100.0, 1, 0.0)));
        assert!(!prefer_observation(&ll2, &raw));
    }

    #[test]
    fn pending_observation_progress_counts_distinct_eligible_camera_frame_tags() {
        let choice = |ambiguous, repeats| ObservationChoice {
            value: Value::Null,
            ambiguous,
            repeats,
        };
        let mut choices = BTreeMap::from([
            ((0, 10, 7), choice(false, 1)),
            ((1, 10, 7), choice(false, 1)),
            ((0, 10, 8), choice(true, 2)),
            ((0, 11, 7), choice(false, 1)),
        ]);
        let mut invalid = [BTreeSet::from([11]), BTreeSet::new()];
        assert_eq!(pending_observation_count(&choices, &invalid), 2);
        choices.insert((0, 10, 7), choice(false, 25));
        assert_eq!(pending_observation_count(&choices, &invalid), 2);
        invalid[0].insert(10);
        assert_eq!(pending_observation_count(&choices, &invalid), 1);
        invalid[1].insert(10);
        assert_eq!(pending_observation_count(&choices, &invalid), 0);
    }

    #[test]
    fn recorded_intrinsics_are_used_and_malformed_values_are_not_silently_defaulted() {
        let mut metadata = json!({"metadata":{"calibration":{"tag_width":1.0,"cameras":[
            {"fx":910.0,"fy":930.0,"cx":638.0,"cy":401.0},
            {"fx":920.0,"fy":940.0,"cx":640.0,"cy":399.0}
        ]}}});
        let (calibrations, known) = recording_calibrations(&metadata).unwrap();
        assert!(known);
        assert_eq!(calibrations[1].fx, 920.0);
        metadata["metadata"]["calibration"]["cameras"][0]["fx"] = json!(-1.0);
        assert!(recording_calibrations(&metadata).is_err());
        assert!(!recording_calibrations(&json!({})).unwrap().1);
    }

    fn datagram(
        camera: u8,
        frame: u32,
        stream: u8,
        sequence: u32,
        offset: u32,
        flags: u32,
        payload: &[u8],
    ) -> Vec<u8> {
        let version = 0;
        let mut result = Vec::new();
        for word in [
            0x4c55_5443,
            u32::from_be_bytes([version, camera, stream, 32]),
            frame,
            sequence,
            offset,
            payload.len() as u32,
            flags,
            0,
        ] {
            result.extend(word.to_be_bytes());
        }
        result.extend_from_slice(payload);
        result
    }

    fn pose_datagram(camera: u8, frame: u32, sequence: u32, tag_id: u32) -> Vec<u8> {
        let mut words = [0_u32; 22];
        words[0] = tag_id;
        words[1] = (1 << 31) | ((frame & 255) << 20);
        words[8] = 1;
        for index in [10, 14, 18] {
            words[index] = 1 << 29;
        }
        words[21] = 10 << 16;
        datagram(
            camera,
            frame,
            3,
            sequence,
            0,
            0,
            &words
                .iter()
                .flat_map(|word| word.to_be_bytes())
                .collect::<Vec<_>>(),
        )
    }

    #[test]
    fn pose_selection_hints_exclude_conflicting_records() {
        let directory = std::env::temp_dir().join(format!(
            "lutcam-pose-selection-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        fs::create_dir(&directory).unwrap();
        for camera in 0..2 {
            let good = pose_datagram(camera, 10, 0, 4269);
            let first = pose_datagram(camera, 11, 1, 4269);
            let conflict = pose_datagram(camera, 11, 1, 4781);
            let packets = [&good, &good, &first, &conflict];
            let records: Vec<_> = packets
                .into_iter()
                .enumerate()
                .map(|(i, data)| (i as u64, data.as_slice()))
                .collect();
            fs::write(
                directory.join(format!("camera{camera}.udp")),
                clip(&records),
            )
            .unwrap();
        }
        let index = index_frames(&directory).unwrap();
        for camera in 0..2 {
            assert_eq!(
                index.pose_ids[camera],
                BTreeMap::from([(10, BTreeSet::from([4269]))])
            );
        }
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn child_packet_history_is_bounded() {
        let mut history = ChildPacketRecords::default();
        for sequence in 0..MAX_CHILD_GROUPS as u32 {
            assert_eq!(
                history.insert(0, sequence, 1, &[sequence as u8]),
                ChildPacketStatus::New
            );
        }
        assert_eq!(history.insert(0, 0, 1, &[0]), ChildPacketStatus::Duplicate);
        assert_eq!(
            history.insert(0, MAX_CHILD_GROUPS as u32, 1, &[0]),
            ChildPacketStatus::Conflict
        );
        assert_eq!(history.packets.len(), MAX_CHILD_GROUPS);
        assert_eq!(history.groups, MAX_CHILD_GROUPS);
    }

    #[test]
    fn centroid_conflicts_and_malformed_packets_exclude_whole_frames_from_fitting() {
        if Command::new("ffmpeg").arg("-version").output().is_err() {
            return;
        }
        let directory = std::env::temp_dir().join(format!(
            "lutcam-centroid-conflicts-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        fs::create_dir(&directory).unwrap();
        let mut replay = Replay::new(
            &directory,
            2.0,
            180,
            std::array::from_fn(|_| (10..=19).map(|frame| (frame, 1000)).collect()),
            Default::default(),
            [CameraCalibration::default(); 2],
            ReplayMode::Full,
        )
        .unwrap();
        replay.plan = Some(SelectionPlan {
            frames: BTreeSet::from([10]),
            ..Default::default()
        });
        for frame in std::iter::once(10).chain(100..140) {
            let mut pair = PairParts::default();
            pair.timestamps[0] = Some(u64::from(frame));
            replay.pending.insert(frame, pair);
        }
        replay.trim_pending();
        assert!(replay.pending.contains_key(&10));
        assert_eq!(replay.pending.len(), PENDING_PAIRS + 1);
        // Exercise late invalidation after real geometric decoding has already
        // produced a candidate observation. Only final accepted choices fit.
        for frame in 10..=19 {
            replay.selected.insert(frame, 1000);
        }
        let canonical = lftag::canonical_points(0x2a55);
        let mut payload = Vec::new();
        for point in canonical[2..].iter().chain(&canonical[..2]) {
            let x = ((point.position.x * 30.0 + 400.0) * 32.0).round() as u64;
            let y = ((point.position.y * 30.0 + 300.0) * 32.0).round() as u64;
            payload.extend((1 | (x << 1) | (y << 17)).to_be_bytes());
        }
        let send = |replay: &mut Replay, bytes: Vec<u8>| {
            replay
                .children(
                    0,
                    ClipRecord {
                        timestamp_ns: 1000,
                        datagram: bytes,
                    },
                )
                .unwrap();
        };
        for frame in 10..=19 {
            let one = datagram(0, frame, 2, 1, 0, 0, &payload);
            match frame {
                12 => send(
                    &mut replay,
                    datagram(0, frame, 2, 1, 0, 0, &payload.repeat(2)),
                ),
                16 => {
                    let mut bad = one.clone();
                    bad[32] = 0x80;
                    send(&mut replay, bad);
                }
                17 => {
                    let short = datagram(0, frame, 2, 0, 0, 0, &payload[..8]);
                    send(&mut replay, short);
                }
                _ => send(&mut replay, one.clone()),
            }
            let second = match frame {
                11 => datagram(0, frame, 2, 1, 0, 0, &payload.repeat(2)),
                13 => {
                    let mut other = datagram(0, frame, 2, 2, 0, 0, &payload);
                    other[20..24].copy_from_slice(&1_u32.to_be_bytes());
                    other
                }
                14 => {
                    let mut bad = one.clone();
                    bad[32] = 0x80;
                    bad
                }
                15 => {
                    let mut wrong_camera = one.clone();
                    wrong_camera[5] = 1;
                    wrong_camera
                }
                18 => {
                    let mut unsupported = one.clone();
                    unsupported[24..28].copy_from_slice(&2_u32.to_be_bytes());
                    unsupported
                }
                19 => {
                    let mut unrecognizable = one.clone();
                    unrecognizable[0] ^= 1;
                    unrecognizable
                }
                _ => one,
            };
            send(&mut replay, second);
        }
        assert_eq!(replay.child_duplicates[0], 1);
        assert_eq!(
            replay.invalid_child_frames[0],
            BTreeSet::from([11, 12, 13, 14, 16, 17, 18])
        );
        assert!(replay.observation_choices.contains_key(&(0, 11, 0x2a55)));
        let progress = replay.extraction_progress(19);
        assert_eq!(progress["pending_observations"], 3);
        assert_eq!(progress["observations_final"], false);
        assert!(progress.get("observations").is_none());
        assert_eq!(replay.report()["observations"], 0);
        replay.finish(1).unwrap();
        assert_eq!(replay.report()["observations"], 3);
        let observations = fs::read_to_string(directory.join("observations.jsonl"))
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str::<Value>(line).unwrap())
            .collect::<Vec<_>>();
        assert_eq!(
            observations
                .iter()
                .map(|observation| observation["frame_id"].as_u64().unwrap())
                .collect::<Vec<_>>(),
            [10, 15, 19]
        );
        assert!(
            fs::metadata(directory.join("calibration.rrd"))
                .unwrap()
                .len()
                > 100
        );
        drop(replay);
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn malformed_centroid_only_frames_have_stable_indexed_timestamps() {
        let directory = std::env::temp_dir().join(format!(
            "lutcam-malformed-index-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        fs::create_dir(&directory).unwrap();
        for camera in 0..2 {
            let mut unsupported = datagram(camera, 7, 2, 1, 0, 0, &[0; 72]);
            unsupported[24..28].copy_from_slice(&2_u32.to_be_bytes());
            fs::write(
                directory.join(format!("camera{camera}.udp")),
                clip(&[(100, &unsupported), (200, &unsupported)]),
            )
            .unwrap();
        }
        let index = index_frames(&directory).unwrap();
        assert_eq!(index.timestamps[0][&7], 100);
        assert_eq!(index.timestamps[1][&7], 100);
        fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn real_udp_replay_selects_three_shared_frames_without_adding_preview_frames() {
        if Command::new("ffmpeg").arg("-version").output().is_err() {
            return;
        }
        let directory = std::env::temp_dir().join(format!(
            "lutcam-calibration-replay-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        fs::create_dir(&directory).unwrap();
        fs::write(
            directory.join("manifest.json"),
            br#"{"format":"lutcam-udp-v1","complete":true}"#,
        )
        .unwrap();
        // The image is deliberately blank: detections can only come from the
        // recorded FPGA coordinates, never pose reprojection or image fitting.
        let encoded = lwc::Codec::encode(
            &vec![150; stream::WIDTH * stream::HEIGHT],
            stream::WIDTH,
            stream::HEIGHT,
        )
        .unwrap();
        let end = *lwc::Codec::packet_end_bits(&encoded)
            .unwrap()
            .last()
            .unwrap();
        let compact = &encoded.bytes()[..end.div_ceil(16) * 2];
        let canonical = lftag::canonical_points(0x2a55);
        for camera in 0..2_u8 {
            let mut records: Vec<(u64, Vec<u8>)> = Vec::new();
            let mut image_sequence = 0;
            for (index, frame) in [10, 100, 200].into_iter().enumerate() {
                let timestamp = 100_000_000 + index as u64 * 150_000_000;
                let mut children = Vec::new();
                // ChildArena sorts the seven smaller data blobs before the
                // two larger baseline blobs. Tuple order carries that rank.
                for point in canonical[2..].iter().chain(&canonical[..2]) {
                    let x = ((point.position.x * 30.0 + 400.0 - 20.0 * f64::from(camera)) * 32.0)
                        .round() as u64;
                    let y = ((point.position.y * 30.0 + 300.0) * 32.0).round() as u64;
                    children.extend((1 | (x << 1) | (y << 17)).to_be_bytes());
                }
                // Accepted-only firmware emits the pose before replaying
                // its measured centroids. Also exercise the reversed order in
                // this fixture; neither order needs rejected candidate groups.
                let children = datagram(camera, frame, 2, index as u32, 0, 0, &children);
                let pose = pose_datagram(camera, frame, index as u32, 0x2a55);
                if index % 2 == 0 {
                    records.push((timestamp, pose));
                    records.push((timestamp + 1, children));
                } else {
                    records.push((timestamp, children));
                    records.push((timestamp + 1, pose));
                }
                for (fragment, payload) in compact.chunks(1024).enumerate() {
                    let offset = fragment * 1024;
                    let bytes =
                        datagram(camera, frame, 1, image_sequence, offset as u32, 0, payload);
                    records.push((timestamp + 1000 + fragment as u64, bytes));
                    image_sequence += 1;
                }
                // Image streams omit a final marker. The next unselected
                // image makes the candidate available before a large jump to
                // the next selected frame; fit-only must retain this behavior.
                let next_frame = datagram(camera, frame + 1, 1, image_sequence, 0, 0, &[0; 2]);
                records.push((timestamp + 1_000_000, next_frame));
                image_sequence += 1;
            }
            let payload = records.iter().find(|(_, bytes)| bytes[6] == 2).unwrap().1[32..].to_vec();
            for (index, timestamp) in [1_000_000_000, 1_000_000_100].into_iter().enumerate() {
                records.push((
                    timestamp,
                    datagram(camera, 13, 2, 3 + index as u32, 0, 0, &payload),
                ));
            }
            let refs = records
                .iter()
                .map(|(time, bytes)| (*time, bytes.as_slice()))
                .collect::<Vec<_>>();
            fs::write(directory.join(format!("camera{camera}.udp")), clip(&refs)).unwrap();
        }
        run(Options {
            recording: directory.clone(),
            sample_hz: 2.0,
            max_pairs: 120,
            baseline_mm: 120.0,
            fit: false,
            mode: ReplayMode::Full,
        })
        .unwrap();
        let observations = fs::read_to_string(directory.join("observations.jsonl"))
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str::<Value>(line).unwrap())
            .collect::<Vec<_>>();
        assert_eq!(observations.len(), 6);
        for observation in &observations {
            assert_eq!(observation["tag_id"], 0x2a55);
            assert_eq!(observation["source"], "fpga_centroids");
            assert_eq!(observation["points"].as_array().unwrap().len(), 9);
            let camera = observation["camera"].as_u64().unwrap();
            assert!(
                (observation["points"][0][0].as_f64().unwrap() - (370.0 - 20.0 * camera as f64))
                    .abs()
                    < 1e-9
            );
            assert_eq!(observation["mass_spread_checked"], false);
        }
        let report: Value =
            serde_json::from_reader(File::open(directory.join("extraction.json")).unwrap())
                .unwrap();
        assert_eq!(report["sampled_pairs"], 3);
        assert_eq!(report["selection"]["planned_shared_pose_pairs"], 3);
        assert_eq!(report["complete_images_archived"], json!([3, 3]));
        assert_eq!(report["preview"]["total_frames"], json!([3, 3]));
        assert_eq!(report["preview"]["repeated_frames"], json!([2, 2]));
        assert_eq!(report["child_groups"], json!([5, 5]));
        for file in ["camera0.mkv", "camera1.mkv", "calibration.rrd"] {
            assert!(fs::metadata(directory.join(file)).unwrap().len() > 100);
        }
        let fast = directory.join("fit-only");
        fs::create_dir(&fast).unwrap();
        for file in ["camera0.udp", "camera1.udp", "manifest.json"] {
            fs::copy(directory.join(file), fast.join(file)).unwrap();
        }
        run(Options {
            recording: fast.clone(),
            sample_hz: 2.0,
            max_pairs: 120,
            baseline_mm: 120.0,
            fit: false,
            mode: ReplayMode::FitOnly,
        })
        .unwrap();
        for file in ["observations.jsonl", "extraction-frames.jsonl"] {
            assert_eq!(
                fs::read(fast.join(file)).unwrap(),
                fs::read(directory.join(file)).unwrap(),
                "fast extraction must preserve measured coordinates and decisions"
            );
        }
        for file in ["calibration.rrd", "camera0.mkv", "camera1.mkv"] {
            assert!(
                !fast.join(file).exists(),
                "fit-only must not create display derivatives"
            );
        }
        fs::write(fast.join("calibration.json"), b"fit result sentinel").unwrap();
        let fit_report = fs::read(fast.join("extraction.json")).unwrap();
        run(Options {
            recording: fast.clone(),
            sample_hz: 2.0,
            max_pairs: 120,
            baseline_mm: 120.0,
            fit: false,
            mode: ReplayMode::MetadataOnly,
        })
        .unwrap();
        assert!(fs::metadata(fast.join("calibration.rrd")).unwrap().len() > 100);
        for file in ["camera0.mkv", "camera1.mkv"] {
            assert!(
                !fast.join(file).exists(),
                "metadata export must not transcode images"
            );
        }
        for file in ["observations.jsonl", "extraction-frames.jsonl"] {
            assert_eq!(
                fs::read(fast.join(file)).unwrap(),
                fs::read(directory.join(file)).unwrap()
            );
        }
        assert_eq!(fs::read(fast.join("extraction.json")).unwrap(), fit_report);
        assert_eq!(
            fs::read(fast.join("calibration.json")).unwrap(),
            b"fit result sentinel"
        );
        let metadata: Value =
            serde_json::from_reader(File::open(fast.join("archive-extraction.json")).unwrap())
                .unwrap();
        assert_eq!(metadata["child_groups"], json!([5, 5]));
        assert_eq!(metadata["complete_images_archived"], json!([0, 0]));
        assert_eq!(metadata["sampled_pairs"], 0);
        assert!(metadata["preview"].is_null());
        run(Options {
            recording: fast.clone(),
            sample_hz: 2.0,
            max_pairs: 120,
            baseline_mm: 120.0,
            fit: false,
            mode: ReplayMode::ArchiveOnly,
        })
        .unwrap();
        for file in ["observations.jsonl", "extraction-frames.jsonl"] {
            assert_eq!(
                fs::read(fast.join(file)).unwrap(),
                fs::read(directory.join(file)).unwrap()
            );
        }
        assert_eq!(fs::read(fast.join("extraction.json")).unwrap(), fit_report);
        assert_eq!(
            fs::read(fast.join("calibration.json")).unwrap(),
            b"fit result sentinel"
        );
        for file in ["camera0.mkv", "camera1.mkv", "calibration.rrd"] {
            assert!(fs::metadata(fast.join(file)).unwrap().len() > 100);
        }
        let probe = Command::new("ffprobe")
            .args([
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-count_frames",
                "-show_entries",
                "stream=codec_name,width,height,nb_read_frames",
                "-of",
                "json",
            ])
            .arg(directory.join("camera0.mkv"))
            .output()
            .unwrap();
        assert!(probe.status.success());
        let probe: Value = serde_json::from_slice(&probe.stdout).unwrap();
        assert_eq!(probe["streams"][0]["codec_name"], "ffv1");
        assert_eq!(probe["streams"][0]["width"], 1280);
        assert_eq!(probe["streams"][0]["nb_read_frames"], "3");
        fs::remove_dir_all(directory).unwrap();
    }
}
