use lutcam_viewer::calibration::{
    MAX_RECORDING_DURATION, RECORDING_DURATION, Recorder, RecordingStatus,
};
use lutcam_viewer::capture::{CaptureController, CaptureSnapshot, CaptureState, ExposureError};
use lutcam_viewer::exposure::{meter, settings_for_brightness};
use lutcam_viewer::receiver::{CameraSnapshot, DecodedFrame, ReceiverConfig, ReceiverSet};
use lutcam_viewer::rerun_feed::{CameraCalibration, RerunFeed};
use lwc::{FRAME_HEIGHT, FRAME_WIDTH};
use re_viewer::SystemCommandSender as _;
use re_viewer::external::egui::{Color32, RichText};
use re_viewer::external::re_sdk_types::blueprint::components::PlayState;
use re_viewer::external::re_viewer_context::TimeControlCommand;
use re_viewer::external::{eframe, egui};
use std::{
    env,
    error::Error,
    fs,
    net::Ipv4Addr,
    path::PathBuf,
    sync::Arc,
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

// Rerun uses live allocation counts to collect old recording data at its
// configured memory limit. GPU/driver allocations remain outside this count.
#[global_allocator]
static ALLOCATOR: re_memory::AccountingAllocator<std::alloc::System> =
    re_memory::AccountingAllocator::new(std::alloc::System);

const ACCENT: Color32 = Color32::from_rgb(67, 222, 179);
const MUTED: Color32 = Color32::from_rgb(143, 158, 176);
const MEMORY_LIMIT_BYTES: u64 = 256 * 1024 * 1024;
const EXPOSURE_INTERVAL: Duration = Duration::from_millis(500);
const EXPOSURE_SETTLE: Duration = Duration::from_millis(200);
const EXPOSURE_FRAME_MAX_AGE: Duration = Duration::from_millis(300);
type UiError = Box<dyn Error + Send + Sync>;

struct AutoExposure {
    enabled: bool,
    compensation_ev: f32,
    brightness: [Option<f32>; 2],
    acknowledgment: Option<(u32, Instant)>,
    last_sample: Option<Instant>,
    last_request: Option<Instant>,
    seen_error: Option<ExposureError>,
    error: Option<String>,
    firmware: bool,
}

impl Default for AutoExposure {
    fn default() -> Self {
        Self {
            enabled: true,
            compensation_ev: 0.0,
            brightness: [None, None],
            acknowledgment: None,
            last_sample: None,
            last_request: None,
            seen_error: None,
            error: None,
            firmware: false,
        }
    }
}

impl AutoExposure {
    fn reset_tracking(&mut self) {
        *self = Self {
            enabled: self.enabled,
            compensation_ev: self.compensation_ev,
            ..Self::default()
        };
    }

    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if enabled {
            // A helper error is retained until a later successful update. Only
            // an explicit user retry may acknowledge that same old error.
            self.error = None;
        }
    }

    fn fail(&mut self, message: String) {
        self.enabled = false;
        self.error = Some(message);
    }

    fn observe(&mut self, capture: &CaptureSnapshot, now: Instant) {
        if let Some(error) = &capture.exposure_error
            && self.seen_error.as_ref() != Some(error)
        {
            self.seen_error = Some(error.clone());
            self.fail(error.message.clone());
        }
        if let Some(status) = capture.exposure
            && self.acknowledgment.map(|(id, _)| id) != Some(status.id)
        {
            self.acknowledgment = Some((status.id, now));
        }
    }

    fn sample_due(&mut self, now: Instant) -> bool {
        if self
            .last_sample
            .is_some_and(|last| now.saturating_duration_since(last) < EXPOSURE_INTERVAL)
        {
            return false;
        }
        self.last_sample = Some(now);
        true
    }

    fn frame_is_fresh(received_at: Instant, now: Instant) -> bool {
        now.checked_duration_since(received_at)
            .is_some_and(|age| age < EXPOSURE_FRAME_MAX_AGE)
    }

    fn can_adjust(
        &self,
        capture: &CaptureSnapshot,
        frames: [Option<&DecodedFrame>; 2],
        now: Instant,
    ) -> bool {
        if !self.enabled
            || self.error.is_some()
            || !capture.owned
            || capture.state != CaptureState::Running
            || capture.exposure_pending.is_some()
            || self
                .last_request
                .is_some_and(|last| now.saturating_duration_since(last) < EXPOSURE_INTERVAL)
        {
            return false;
        }
        let (Some(status), Some((acknowledged_id, acknowledged_at))) =
            (capture.exposure, self.acknowledgment)
        else {
            return false;
        };
        let settled_at = acknowledged_at + EXPOSURE_SETTLE;
        status.id == acknowledged_id
            && now >= settled_at
            && frames.into_iter().all(|frame| {
                frame.is_some_and(|frame| {
                    frame.received_at >= settled_at && Self::frame_is_fresh(frame.received_at, now)
                })
            })
    }
}

struct Options {
    board: Option<Ipv4Addr>,
    network: ReceiverConfig,
    explicit_source: bool,
    headless: Option<Duration>,
    output: PathBuf,
    calibration: Option<PathBuf>,
    calibration_seconds: u64,
}
impl Options {
    fn parse(args: impl Iterator<Item = String>) -> Result<Option<Self>, Box<dyn Error>> {
        let mut options = Self {
            board: None,
            network: ReceiverConfig {
                bind_ip: Ipv4Addr::UNSPECIFIED,
                source_ip: None,
                ports: [6000, 6003],
            },
            explicit_source: false,
            headless: None,
            output: PathBuf::from("captures/live-view"),
            calibration: None,
            calibration_seconds: RECORDING_DURATION.as_secs(),
        };
        let mut args = args;
        while let Some(arg) = args.next() {
            if arg == "--help" || arg == "-h" {
                println!(
                    "LUTCam Live - two-camera Rust Rerun viewer\n\nUsage: lutcam-viewer [OPTIONS]\n\n  --board IP            Use this FPGA address (default: discover on the local network)\n  --bind IP             Local IPv4 interface (default 0.0.0.0; capture selects the route to the board)\n  --source IP|any       Accept packets from this source (default: selected board)\n  --port0 PORT          Camera 0 UDP port (default 6000)\n  --port1 PORT          Camera 1 UDP port (default 6003)\n  --output DIR          Snapshots and calibration recordings (default captures/live-view)\n  --calibration PATH    Load fitted pinhole calibration for Start both\n  --calibration-seconds N  Initial recording duration in seconds (1–86400, default 60)\n  --headless-seconds N  Receive/decode for N seconds, save latest PNGs + report, then exit\n\nStart both configures and starts paired 120 fps capture; Stop both stops capture started here.\nClosing the window stops only capture started by this viewer.\nRerun pause and timeline controls affect the view; capture and reception keep running.\nResync forgets frame IDs and clears live images; Rerun history remains available.\nHeadless mode only receives; it never starts or stops capture."
                );
                return Ok(None);
            }
            let value = args
                .next()
                .ok_or_else(|| format!("{arg} requires a value"))?;
            match arg.as_str() {
                "--board" => options.board = Some(value.parse()?),
                "--bind" => options.network.bind_ip = value.parse()?,
                "--source" => {
                    options.explicit_source = true;
                    options.network.source_ip = if value == "any" {
                        None
                    } else {
                        Some(value.parse()?)
                    }
                }
                "--port0" | "--port1" => {
                    let index = usize::from(arg == "--port1");
                    options.network.ports[index] = value.parse()?;
                }
                "--output" => options.output = value.into(),
                "--calibration" => options.calibration = Some(value.into()),
                "--calibration-seconds" => {
                    options.calibration_seconds = value.parse()?;
                    if !(1..=MAX_RECORDING_DURATION.as_secs())
                        .contains(&options.calibration_seconds)
                    {
                        return Err(
                            "calibration duration must be between 1 and 86400 seconds".into()
                        );
                    }
                }
                "--headless-seconds" => {
                    let seconds: f64 = value.parse()?;
                    if !seconds.is_finite() || !(0.1..=86400.0).contains(&seconds) {
                        return Err(
                            "headless duration must be between 0.1 and 86400 seconds".into()
                        );
                    }
                    options.headless = Some(Duration::from_secs_f64(seconds));
                }
                _ => return Err(format!("unknown option: {arg}").into()),
            }
        }
        if options.board.is_some_and(|board| {
            board.is_unspecified() || board.is_multicast() || board.is_broadcast()
        }) {
            return Err("board must be a unicast IPv4 address".into());
        }
        if !options.explicit_source {
            options.network.source_ip = options.board;
        }
        if options.network.ports.contains(&0)
            || options.network.ports[0] == options.network.ports[1]
        {
            return Err("camera ports must be distinct and nonzero".into());
        }
        Ok(Some(options))
    }

    fn resolve_board(
        &mut self,
        discover: impl FnOnce(Ipv4Addr) -> Result<Ipv4Addr, String>,
    ) -> Result<(), String> {
        // A headless receiver with an explicit source needs no camera board.
        // All interactive launches resolve control and default video filtering
        // together before opening the receivers.
        if self.board.is_none() && !(self.headless.is_some() && self.explicit_source) {
            self.board = Some(discover(self.network.bind_ip)?);
        }
        if !self.explicit_source {
            self.network.source_ip = self.board;
        }
        Ok(())
    }
}

fn save_frame(path: &std::path::Path, frame: &DecodedFrame) -> Result<(), Box<dyn Error>> {
    let pixels: Vec<u8> = frame
        .pixels
        .chunks_exact(FRAME_WIDTH)
        .rev()
        .flatten()
        .copied()
        .collect();
    image::save_buffer(
        path,
        &pixels,
        FRAME_WIDTH as u32,
        FRAME_HEIGHT as u32,
        image::ColorType::L8,
    )?;
    Ok(())
}

fn calibration_params(value: &serde_json::Value) -> Result<[CameraCalibration; 2], String> {
    let cameras = value
        .get("cameras")
        .and_then(serde_json::Value::as_array)
        .filter(|cameras| cameras.len() == 2)
        .ok_or("Calibration must describe both cameras")?;
    let mut result = [CameraCalibration::default(); 2];
    for (camera, value) in cameras.iter().enumerate() {
        let effective = value.get("effective").unwrap_or(value);
        let number = |key| {
            effective
                .get(key)
                .and_then(serde_json::Value::as_f64)
                .filter(|x| x.is_finite())
                .ok_or_else(|| format!("Camera {camera}: invalid {key}"))
        };
        result[camera] = CameraCalibration {
            fx: number("fx")? as f32,
            fy: number("fy")? as f32,
            cx: number("cx")? as f32,
            cy: number("cy")? as f32,
            // The fixed-point gateware uses normalized tag-width units. Its
            // Q16 port cannot represent physical millimetre dimensions.
            tag_width: 1.0,
        };
        if !(100.0..=10000.0).contains(&result[camera].fx)
            || !(100.0..=10000.0).contains(&result[camera].fy)
            || !(0.0..FRAME_WIDTH as f32).contains(&result[camera].cx)
            || !(0.0..FRAME_HEIGHT as f32).contains(&result[camera].cy)
        {
            return Err(format!(
                "Camera {camera}: calibration is outside the sensor's supported range"
            ));
        }
    }
    Ok(result)
}

fn read_calibration(path: &std::path::Path) -> Result<[CameraCalibration; 2], String> {
    if fs::metadata(path).map_err(|error| error.to_string())?.len() > 1024 * 1024 {
        return Err("Calibration file exceeds 1 MiB".into());
    }
    let bytes = fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?;
    let value: serde_json::Value =
        serde_json::from_slice(&bytes).map_err(|error| error.to_string())?;
    if value["schema_version"] != 1
        || value["image_size"] != serde_json::json!([FRAME_WIDTH, FRAME_HEIGHT])
    {
        return Err("Calibration must use schema 1 and 1280 × 800 source pixel coordinates".into());
    }
    let positive = |value: &serde_json::Value| {
        value
            .as_f64()
            .is_some_and(|number| number.is_finite() && number > 0.0)
    };
    if !positive(&value["baseline_mm"])
        || value
            .get("tag_width_mm")
            .is_some_and(|width| !positive(width))
    {
        return Err("Calibration needs a positive baseline and tag width, if supplied".into());
    }
    let cameras = value["cameras"]
        .as_array()
        .filter(|cameras| cameras.len() == 2)
        .ok_or("Calibration must describe both cameras")?;
    for (index, camera) in cameras.iter().enumerate() {
        if camera
            .get("camera")
            .is_some_and(|id| id.as_u64() != Some(index as u64))
        {
            return Err("Calibration camera identities must match camera0, camera1 order".into());
        }
        if !camera["distortion"].as_array().is_some_and(|values| {
            values.len() == 5
                && values
                    .iter()
                    .all(|value| value.as_f64().is_some_and(f64::is_finite))
        }) {
            return Err(format!(
                "Camera {index}: five finite distortion coefficients are required"
            ));
        }
    }
    calibration_params(&value)
}

fn headless(
    receivers: ReceiverSet,
    seconds: Duration,
    output: PathBuf,
) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(&output)?;
    let start = Instant::now();
    while start.elapsed() < seconds {
        thread::sleep(Duration::from_millis(50));
    }
    let snapshots = receivers.snapshot();
    let mut records = Vec::new();
    let mut both_live = true;
    for (camera, state) in snapshots.iter().enumerate() {
        let mut record = serde_json::json!({"camera":camera,"packets":state.packets,"bytes":state.bytes,
            "decoded":state.decoded,"fps":state.fps,"mib_s":state.mib_s,"invalid":state.invalid,
            "dropped":state.dropped,"display_drops":state.display_drops,"duplicates":state.duplicates,
            "decode_errors":state.decode_errors,"incomplete_decodes":state.incomplete_decodes,
            "ignored_source":state.ignored_source,
            "pose_packets":state.pose_packets,"pose_detections":state.pose_detections,
            "pose_invalid":state.pose_invalid,"pose_duplicates":state.pose_duplicates,
            "centroid_packets":state.centroid_packets,"centroid_groups":state.centroid_groups,
            "centroid_invalid":state.centroid_invalid,
            "error":state.error,"socket_buffer":state.socket_buffer});
        if let Some(frame) = &state.latest {
            let path = output.join(format!("camera{camera}.png"));
            save_frame(&path, frame)?;
            record["frame_id"] = frame.frame_id.into();
            record["image"] = path.display().to_string().into();
            record["frame_age_ms"] = (frame.received_at.elapsed().as_millis() as u64).into();
            record["decode_ms"] = frame.decode_ms.into();
            both_live &= frame.received_at.elapsed() < Duration::from_secs(2);
        } else {
            both_live = false;
        }
        records.push(record);
    }
    let report = serde_json::json!({"ok":both_live,"elapsed_seconds":start.elapsed().as_secs_f64(),"cameras":records});
    fs::write(
        output.join("report.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    if !both_live {
        return Err("both cameras must deliver a recent decoded frame; inspect report.json".into());
    }
    Ok(())
}

struct Viewer {
    // Fields drop in declaration order. Keep reception alive while our owned
    // capture helper stops and drains the paired camera pipeline.
    capture: CaptureController,
    receivers: ReceiverSet,
    rerun_app: re_viewer::App,
    feed: RerunFeed,
    logged: [Option<Arc<DecodedFrame>>; 2],
    logged_poses: [std::collections::HashMap<u32, u64>; 2],
    logged_centroids: [std::collections::HashMap<u32, u64>; 2],
    last_log: Instant,
    board: Ipv4Addr,
    network: ReceiverConfig,
    output: PathBuf,
    stacked: bool,
    tag_outlines_only: bool,
    auto_exposure: AutoExposure,
    recorder: Recorder,
    calibration_seconds: u64,
    calibration_path: Option<PathBuf>,
    calibration_description: String,
    calibration_restart: bool,
    effective_calibration: Option<serde_json::Value>,
    closing: bool,
    notice: Option<(String, Instant)>,
}

impl Viewer {
    fn new(
        cc: &eframe::CreationContext<'_>,
        main_thread: re_viewer::MainThreadToken,
        runtime: re_viewer::AsyncRuntimeHandle,
        receivers: ReceiverSet,
        options: Options,
    ) -> Result<Self, UiError> {
        re_viewer::customize_eframe_and_setup_renderer(cc)?;
        cc.egui_ctx.set_theme(egui::ThemePreference::Dark);
        let startup = re_viewer::StartupOptions {
            persist_state: false,
            hide_welcome_screen: true,
            expect_data_soon: Some(true),
            ..Default::default()
        };
        let mut rerun_app = re_viewer::App::new(
            main_thread,
            re_viewer::build_info(),
            re_viewer::AppEnvironment::Custom("LUTCam Live".into()),
            startup,
            cc,
            None,
            runtime,
        );
        rerun_app.app_options_mut().custom_window_decorations = false;
        rerun_app.app_options_mut().memory_limit =
            re_memory::MemoryLimit::from_bytes(MEMORY_LIMIT_BYTES);
        let (feed, receiver) = RerunFeed::new()?;
        rerun_app.add_log_receiver(receiver);
        if let Some(path) = &options.calibration {
            let params = read_calibration(path)?;
            for (camera, params) in params.into_iter().enumerate() {
                feed.set_calibration(camera, params)?;
            }
        }
        let calibration_description = options.calibration.as_ref().map_or_else(
            || "2.8 mm lens estimate | 120 mm baseline | not yet calibrated".into(),
            |path| format!("Calibration: {}", path.display()),
        );
        Ok(Self {
            capture: CaptureController::new(),
            receivers,
            rerun_app,
            feed,
            logged: [None, None],
            logged_poses: Default::default(),
            logged_centroids: Default::default(),
            last_log: Instant::now(),
            board: options
                .board
                .ok_or("Camera board was not resolved before viewer startup")?,
            network: options.network,
            output: options.output,
            stacked: false,
            tag_outlines_only: false,
            auto_exposure: AutoExposure::default(),
            recorder: Recorder::new(),
            calibration_seconds: options.calibration_seconds,
            calibration_path: options.calibration,
            calibration_description,
            calibration_restart: false,
            effective_calibration: None,
            closing: false,
            notice: None,
        })
    }

    fn log_latest(&mut self) {
        // Capture metadata and UDP arrive independently. Do not permanently
        // attach a previous session's intrinsics to the first new images.
        if self.capture.snapshot().owned && self.effective_calibration.is_none() {
            return;
        }
        // Rerun may request repaints faster than the display. Sample the bounded
        // decoder mailboxes at most 60 times/s, independent of timeline playback.
        if self.last_log.elapsed() < Duration::from_millis(16) {
            return;
        }
        self.last_log = Instant::now();
        for (camera, state) in self.receivers.snapshot().iter().enumerate() {
            let image = state.latest.as_ref().filter(|frame| {
                self.logged[camera]
                    .as_ref()
                    .is_none_or(|old| !Arc::ptr_eq(old, frame))
            });
            let detections: Vec<_> = state
                .pose_frames
                .iter()
                .filter(|frame| {
                    self.logged_poses[camera].get(&frame.frame_id) != Some(&frame.revision)
                })
                .map(Arc::clone)
                .collect();
            let centroids: Vec<_> = state
                .centroid_frames
                .iter()
                .filter(|frame| {
                    self.logged_centroids[camera].get(&frame.frame_id) != Some(&frame.revision)
                })
                .map(Arc::clone)
                .collect();
            if image.is_some() || !detections.is_empty() || !centroids.is_empty() {
                match self
                    .feed
                    .log_snapshot(camera, image.map(Arc::clone), &detections, &centroids)
                {
                    Ok(()) => {
                        if let Some(image) = image {
                            self.logged[camera] = Some(Arc::clone(image));
                        }
                        for frame in detections {
                            self.logged_poses[camera].insert(frame.frame_id, frame.revision);
                        }
                        for frame in centroids {
                            self.logged_centroids[camera].insert(frame.frame_id, frame.revision);
                        }
                    }
                    Err(error) => {
                        self.notice =
                            Some((format!("Rerun logging failed: {error}"), Instant::now()));
                    }
                }
            }
            self.logged_poses[camera].retain(|frame_id, _| {
                state
                    .pose_frames
                    .iter()
                    .any(|frame| frame.frame_id == *frame_id)
            });
            self.logged_centroids[camera].retain(|frame_id, _| {
                state
                    .centroid_frames
                    .iter()
                    .any(|frame| frame.frame_id == *frame_id)
            });
        }
    }

    fn adjust_exposure(&mut self) {
        let now = Instant::now();
        let capture = self.capture.snapshot();
        if let Some(firmware) = capture
            .details
            .as_ref()
            .and_then(|d| d.get("firmware_exposure"))
        {
            if !self.auto_exposure.firmware {
                self.auto_exposure.enabled = firmware["enabled"].as_bool().unwrap_or(true);
                self.auto_exposure.compensation_ev =
                    firmware["ev_compensation"].as_f64().unwrap_or(0.0) as f32;
                self.auto_exposure.firmware = true;
            }
            self.auto_exposure.error = capture.exposure_error.as_ref().map(|e| e.message.clone());
            self.auto_exposure.brightness = [0, 1].map(|i| {
                firmware.get("p90").unwrap_or(&firmware["means"])[i]
                    .as_f64()
                    .map(|x| x as f32)
            });
            return;
        }
        // Observe every acknowledgment immediately, even between metering
        // samples, so the settling interval starts when the GUI learns of it.
        self.auto_exposure.observe(&capture, now);
        if !self.auto_exposure.sample_due(now) {
            return;
        }
        let cameras = self.receivers.snapshot();
        let frames = cameras.each_ref().map(|camera| camera.latest.as_deref());
        self.auto_exposure.brightness = frames.map(|frame| {
            frame
                .filter(|frame| AutoExposure::frame_is_fresh(frame.received_at, now))
                .and_then(|frame| meter(&frame.pixels))
        });
        if !self.auto_exposure.can_adjust(&capture, frames, now) {
            return;
        }
        let (Some(status), [Some(first), Some(second)]) =
            (capture.exposure, self.auto_exposure.brightness)
        else {
            return;
        };
        if let Some(settings) = settings_for_brightness(
            &status,
            [first, second],
            (100.0 * self.auto_exposure.compensation_ev.exp2()).clamp(1.0, 254.0),
        ) {
            // The controller queues work for its helper. Neither sensor I/O
            // nor awaiting an acknowledgment occurs on the GUI thread.
            self.auto_exposure.last_request = Some(now);
            if let Err(error) = self
                .capture
                .set_exposure(settings.exposure_lines, settings.gain_code)
            {
                self.auto_exposure.fail(error);
            }
        }
    }

    fn save_snapshots(&mut self) {
        let result = (|| -> Result<usize, Box<dyn Error>> {
            fs::create_dir_all(&self.output)?;
            let stamp = SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis();
            let mut count = 0;
            for (camera, state) in self.receivers.snapshot().iter().enumerate() {
                if let Some(frame) = &state.latest {
                    save_frame(
                        &self.output.join(format!(
                            "{stamp}-camera{camera}-frame{}.png",
                            frame.frame_id
                        )),
                        frame,
                    )?;
                    count += 1;
                }
            }
            Ok(count)
        })();
        let message = match result {
            Ok(0) => "No decoded images to save yet".into(),
            Ok(n) => format!(
                "Saved {n} latest grayscale PNGs to {}",
                self.output.display()
            ),
            Err(error) => format!("Snapshot failed: {error}"),
        };
        self.notice = Some((message, Instant::now()));
    }

    fn resync(&mut self) -> bool {
        self.cancel_recording();
        if let Err(error) = self.feed.clear() {
            self.notice = Some((format!("Resync failed: {error}"), Instant::now()));
            return false;
        }
        self.receivers.resync();
        self.logged = [None, None];
        self.logged_poses = Default::default();
        self.logged_centroids = Default::default();
        self.set_play_state(PlayState::Following);
        self.notice = Some((
            "Receiver resynced; waiting for new images. Earlier frames remain in the timeline."
                .into(),
            Instant::now(),
        ));
        true
    }

    fn set_play_state(&self, state: PlayState) {
        if let Some(store_id) = self.rerun_app.active_recording_id() {
            self.rerun_app.command_sender.send_system(
                re_viewer::SystemCommand::TimeControlCommands {
                    store_id: store_id.clone(),
                    time_commands: vec![TimeControlCommand::SetPlayState(state)],
                },
            );
        }
    }

    fn start_capture(&mut self) {
        if self
            .network
            .source_ip
            .is_some_and(|source| source != self.board)
        {
            self.notice = Some((
                "Start both requires --source to match --board, or --source any".into(),
                Instant::now(),
            ));
            return;
        }
        if !self.resync() {
            return;
        }
        match self.capture.start_with_calibration(
            self.board,
            self.network.bind_ip,
            self.network.ports,
            self.calibration_path.as_deref(),
        ) {
            Ok(()) => {
                self.auto_exposure.reset_tracking();
                self.effective_calibration = None;
                self.notice = None;
            }
            Err(error) => {
                self.notice = Some((format!("Capture start failed: {error}"), Instant::now()))
            }
        }
    }

    fn cancel_recording(&mut self) {
        self.recorder.cancel_recording();
        self.receivers.set_recording(None);
    }

    fn start_calibration(&mut self) {
        let capture = self.capture.snapshot();
        if !capture.owned
            || capture.state != CaptureState::Running
            || self.effective_calibration.is_none()
        {
            return;
        }
        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis();
        let directory = self.output.join(format!("calibration-{stamp}"));
        let metadata = serde_json::json!({
            "baseline_mm":120.0, "lens_focal_length_mm":2.8, "pixel_pitch_um":3.0,
            "image_size":[FRAME_WIDTH,FRAME_HEIGHT], "board":self.board.to_string(),
            "camera_ports":self.network.ports, "source_coordinates":"unflipped integer pixel centers",
            "display_order":[1,0], "display_vertical_flip":true,
            "calibration":self.effective_calibration, "capture_details":capture.details,
            "target":"one flat LFTag, or uniquely identified tags with identical printed widths, visible to both cameras",
        });
        match self.recorder.start_with_duration(
            directory,
            metadata,
            Duration::from_secs(self.calibration_seconds),
        ) {
            Ok(sink) => self.receivers.set_recording(Some(sink)),
            Err(error) => {
                self.notice = Some((
                    format!("Calibration could not start: {error}"),
                    Instant::now(),
                ))
            }
        }
    }

    fn update_calibration(&mut self) {
        let capture = self.capture.snapshot();
        if let Some(calibration) = capture
            .details
            .as_ref()
            .and_then(|details| details.get("calibration"))
            && self.effective_calibration.as_ref() != Some(calibration)
        {
            match calibration_params(calibration) {
                Ok(cameras) => {
                    for (camera, params) in cameras.into_iter().enumerate() {
                        if let Err(error) = self.feed.set_calibration(camera, params) {
                            self.notice = Some((
                                format!("Calibration display failed: {error}"),
                                Instant::now(),
                            ));
                        }
                    }
                    self.effective_calibration = Some(calibration.clone());
                }
                Err(error) => {
                    self.notice = Some((
                        format!("Invalid camera calibration readback: {error}"),
                        Instant::now(),
                    ))
                }
            }
        }
        let recording = self.recorder.snapshot();
        if recording.status != RecordingStatus::Recording {
            self.receivers.set_recording(None);
        } else if !capture.owned || capture.state != CaptureState::Running {
            self.cancel_recording();
        }
        if self.calibration_restart && self.capture.ready_to_start() {
            self.calibration_restart = false;
            if self.capture.snapshot().state == CaptureState::Stopped {
                self.start_capture();
            } else {
                self.notice = Some(("Calibration saved; automatic restart cancelled because capture did not stop cleanly".into(), Instant::now()));
            }
        }
    }

    fn apply_calibration(&mut self, path: PathBuf) {
        match read_calibration(&path) {
            Ok(_) => {
                // Apply K only when the helper restarts the idle pair. Keep the
                // current overlay's calibration until that helper's readback.
                if self.capture.snapshot().owned {
                    match self.capture.stop() {
                        Ok(()) => self.calibration_restart = true,
                        Err(error) => {
                            self.notice = Some((
                                format!("Calibration saved, but capture stop failed: {error}"),
                                Instant::now(),
                            ));
                            return;
                        }
                    }
                }
                self.calibration_description =
                    format!("Calibrated pinhole parameters: {}", path.display());
                self.calibration_path = Some(path);
                self.calibration_restart |= !self.capture.ready_to_start();
                if !self.calibration_restart {
                    self.start_capture();
                }
            }
            Err(error) => {
                self.notice = Some((format!("Calibration not applied: {error}"), Instant::now()))
            }
        }
    }

    fn calibration_controls(&mut self, ui: &mut egui::Ui) {
        let state = self.recorder.snapshot();
        let capture = self.capture.snapshot();
        let busy = matches!(
            state.status,
            RecordingStatus::Recording | RecordingStatus::Processing
        );
        ui.horizontal_wrapped(|ui| {
            ui.add_enabled(!busy, egui::DragValue::new(&mut self.calibration_seconds)
                .range(1..=MAX_RECORDING_DURATION.as_secs())
                .prefix("Duration: ").suffix(" s"))
                .on_hover_text("Calibration recording length in seconds. Click the number to type a value.");
            if ui.add_enabled(!busy && capture.owned && capture.state == CaptureState::Running && self.effective_calibration.is_some(),
                egui::Button::new("Calibrate cameras"))
                .on_hover_text("Record both image streams, all FPGA centroid groups and LFTag poses for the selected duration. Move and tilt one flat LFTag through both views at several distances, including the edges. If using several tags, they must have unique IDs and the same printed width. The 120 mm baseline sets scale. Calibration needs varied observations; an insufficient recording leaves existing parameters intact.")
                .clicked() { self.start_calibration(); }
            if busy && ui.button("Cancel calibration").clicked() {
                self.recorder.cancel();
                self.receivers.set_recording(None);
                return;
            }
            if state.status == RecordingStatus::Recording && state.accepting {
                ui.add(egui::ProgressBar::new(state.progress).desired_width(180.0)
                    .text(format!("Recording {:.0} / {} s", state.elapsed.as_secs_f32(), state.duration.as_secs())));
            } else if busy { ui.spinner(); }
            if state.status != RecordingStatus::Idle { ui.label(&state.message); }
            if let Some(path) = &state.calibration_path
                && matches!(state.status, RecordingStatus::Ready | RecordingStatus::Processing)
                && self.calibration_path.as_ref() != Some(path)
                && ui.button("Apply calibration & restart both").clicked() {
                self.apply_calibration(path.clone());
            }
        });
        if state.status == RecordingStatus::Recording && state.accepting {
            ui.small("Move and tilt a flat LFTag through both views, near/far and across the image. Keep the cameras fixed 120 mm apart.");
        }
        if let Some(error) = &state.error {
            ui.label(RichText::new(error).color(Color32::from_rgb(255, 153, 130)));
        }
        egui::CollapsingHeader::new("Calibration and LFTag layers").show(ui, |ui| {
            ui.small(&self.calibration_description);
            ui.small("Camera 1 is on the left; both images and overlays are vertically flipped. Toggle the centroids and lftags layers with their eye icons in Rerun's blueprint panel.");
            ui.small("The fit saves lens distortion and stereo geometry. Current gateware applies pinhole intrinsics only; distortion is not corrected in live images or FPGA poses.");
            if let Some(directory) = &state.directory {
                ui.monospace(directory.display().to_string());
            }
        });
    }
    fn capture_controls(&mut self, ui: &mut egui::Ui) {
        let before = self.capture.snapshot();
        ui.horizontal_wrapped(|ui| {
            if ui
                .add_enabled(
                    self.capture.ready_to_start(),
                    egui::Button::new("Start both"),
                )
                .clicked()
            {
                self.start_capture();
            }
            if ui
                .add_enabled(
                    before.owned && before.state != CaptureState::Stopping,
                    egui::Button::new("Stop both"),
                )
                .clicked()
            {
                self.cancel_recording();
                self.calibration_restart = false;
                if let Err(error) = self.capture.stop() {
                    self.notice = Some((format!("Capture stop failed: {error}"), Instant::now()));
                }
            }
            let current = self.capture.snapshot();
            let (label, color) = match current.state {
                CaptureState::Idle => ("RECEIVE ONLY", MUTED),
                CaptureState::Starting => ("STARTING", Color32::from_rgb(245, 191, 91)),
                CaptureState::Running => ("CAPTURING", ACCENT),
                CaptureState::Stopping => ("STOPPING", Color32::from_rgb(245, 191, 91)),
                CaptureState::Stopped => ("STOPPED", MUTED),
                CaptureState::Fault => ("FPGA FAULT", Color32::from_rgb(255, 153, 130)),
                CaptureState::Error => ("CAPTURE ERROR", Color32::from_rgb(255, 153, 130)),
            };
            ui.label(RichText::new(label).color(color).strong());
            ui.label(
                RichText::new(format!("120 fps preset | Board {}", self.board))
                    .size(11.0)
                    .color(MUTED),
            );
            if !current.message.is_empty() {
                ui.label(RichText::new(&current.message).size(11.0).color(color));
            }
        });
        let current = self.capture.snapshot();
        if matches!(current.state, CaptureState::Fault | CaptureState::Error)
            && (current.details.is_some() || !current.stderr_tail.is_empty())
        {
            egui::CollapsingHeader::new("Capture error details").show(ui, |ui| {
                egui::ScrollArea::vertical()
                    .max_height(140.0)
                    .show(ui, |ui| {
                        if let Some(details) = &current.details {
                            ui.monospace(serde_json::to_string_pretty(details).unwrap_or_default());
                        }
                        for line in &current.stderr_tail {
                            ui.monospace(line);
                        }
                    });
            });
        }
    }

    fn exposure_controls(&mut self, ui: &mut egui::Ui) {
        let capture = self.capture.snapshot();
        ui.horizontal_wrapped(|ui| {
            let mut enabled = self.auto_exposure.enabled;
            let mut changed = false;
            if ui
                .checkbox(&mut enabled, "Auto exposure")
                .on_hover_text(
                    if self.auto_exposure.firmware {
                        "The camera processor meters histogram percentiles and adjusts both sensors in stops, with highlight protection. It keeps running while the view is paused. Turning this off leaves the current exposure and gain."
                    } else {
                        "Adjusts both cameras together using live receiver frames, including while the view is paused. Turning this off leaves the current exposure and gain; a pending update may still finish."
                    },
                )
                .changed()
            {
                self.auto_exposure.set_enabled(enabled);
                changed = true;
            }
            changed |= ui.add(
                egui::Slider::new(&mut self.auto_exposure.compensation_ev, -2.0..=2.0)
                    .text("Exposure compensation (EV)")
                    .step_by(0.25),
            ).on_hover_text("+1 EV doubles exposure relative to the automatic baseline; -1 EV halves it. Positive compensation can clip highlights. Sensor limits still apply.").changed();
            if self.auto_exposure.firmware {
                ui.label("ISP histogram");
                if changed {
                    if let Err(error) = self.capture.set_auto_exposure(
                        self.auto_exposure.enabled, (self.auto_exposure.compensation_ev * 4.0).round() as i32) {
                        self.auto_exposure.error = Some(error);
                    }
                }
            }
            let percentile = capture.details.as_ref().and_then(|d| d.get("firmware_exposure"))
                .is_some_and(|f| f.get("p90").is_some());
            let levels = self.auto_exposure.brightness.map(|value| {
                value.map_or_else(|| "waiting".into(), |value| if percentile {
                    format!("{:.0}%", value * 100.0 / 255.0)
                } else { format!("{value:.0}") })
            });
            ui.label(format!("{}: Cam 0 {} | Cam 1 {}", if percentile { "P90" } else { "Mean" }, levels[0], levels[1]));
            if let Some(status) = capture.exposure {
                let exposure_ms =
                    f64::from(status.settings.exposure_lines) * status.line_time_us / 1000.0;
                ui.label(format!(
                    "Both cameras: {exposure_ms:.2} ms | gain {:.2}x",
                    f64::from(status.settings.gain_code) / 16.0
                ));
            }
            if capture.exposure_pending.is_some() {
                ui.label(RichText::new("Applying exposure...").color(MUTED));
            } else if self.auto_exposure.enabled
                && (!capture.owned || capture.state != CaptureState::Running)
            {
                ui.label(RichText::new("Auto exposure waits for Start both").color(MUTED));
            }
        });
        if let Some(error) = &self.auto_exposure.error {
            ui.label(
                RichText::new(format!("Exposure control: {error}"))
                    .color(Color32::from_rgb(255, 153, 130)),
            );
        }
    }
    fn camera_status(ui: &mut egui::Ui, camera: usize, state: &CameraSnapshot) {
        let (status, color) =
            if state.error.is_some() || state.invalid + state.dropped + state.decode_errors > 0 {
                ("CHECK STREAM", Color32::from_rgb(255, 153, 130))
            } else if state
                .latest
                .as_ref()
                .is_some_and(|frame| frame.received_at.elapsed() < Duration::from_secs(1))
            {
                ("LIVE", ACCENT)
            } else if state.latest.is_some() {
                ("STALE", Color32::from_rgb(245, 191, 91))
            } else {
                ("WAITING", MUTED)
            };
        ui.label(
            RichText::new(format!(
                "Cam {camera}: {status} | {:.1} fps | {:.2} MiB/s | {} decoded | {} LFTag detections",
                state.fps, state.mib_s, state.decoded, state.pose_detections
            ))
            .color(color),
        );
        if let Some(error) = &state.error {
            ui.label(RichText::new(error).color(color));
        } else if state.invalid + state.dropped + state.decode_errors > 0 {
            ui.label(
                RichText::new(format!(
                    "{} invalid packets, {} incomplete frames, {} codec errors",
                    state.invalid, state.dropped, state.decode_errors
                ))
                .color(color),
            );
        }
    }
}

impl eframe::App for Viewer {
    fn logic(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        if ctx.input(|input| input.viewport().close_requested()) {
            self.closing = true;
            self.recorder.cancel();
            self.receivers.set_recording(None);
            self.calibration_restart = false;
            let _ = self.capture.stop();
            if self.recorder.is_running() || self.capture.snapshot().owned {
                ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            }
        }
        if self.closing && !self.recorder.is_running() && !self.capture.snapshot().owned {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        self.update_calibration();
        self.log_latest();
        self.adjust_exposure();
        self.rerun_app.logic(ctx, frame);
        // Receiving/decoding and capture control continue while the native
        // Rerun timeline is paused or the window is temporarily hidden.
        ctx.request_repaint_after(Duration::from_millis(16));
    }

    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        egui::Frame::new()
            .inner_margin(egui::Margin::symmetric(14, 8))
            .show(ui, |ui| {
                if self.closing {
                    ui.label("Finishing the recording and stopping both cameras before closing…");
                    ui.disable();
                }
                ui.horizontal_wrapped(|ui| {
                    ui.label(RichText::new("LUTCAM").strong().size(20.0));
                    ui.label(RichText::new("RERUN LIVE VIEW").color(MUTED));
                    if ui.button("Pause view").clicked() {
                        self.set_play_state(PlayState::Paused);
                    }
                    if ui
                        .button("Live")
                        .on_hover_text("Follow the newest images")
                        .clicked()
                    {
                        self.set_play_state(PlayState::Following);
                    }
                    if ui
                        .button("Resync")
                        .on_hover_text(
                            "Forget receiver frame IDs and clear live images; timeline history is retained",
                        )
                        .clicked()
                    {
                        self.resync();
                    }
                    if ui.button("Save latest PNGs").clicked() {
                        self.save_snapshots();
                    }
                    if ui.checkbox(&mut self.stacked, "Stack cameras").changed()
                        && let Err(error) = self.feed.set_layout(self.stacked)
                    {
                        self.notice = Some((format!("Layout failed: {error}"), Instant::now()));
                    }
                    if ui.checkbox(&mut self.tag_outlines_only, "Tag outlines only")
                        .on_hover_text("Hide tag IDs, warning styling, and pose points on current/live images. Measured centroids stay independent; earlier timeline images retain their recorded presentation.")
                        .changed()
                        && let Err(error) = self.feed.set_tag_outlines_only(self.tag_outlines_only)
                    {
                        self.notice = Some((format!("Tag presentation failed: {error}"), Instant::now()));
                    }
                });
                self.capture_controls(ui);
                self.exposure_controls(ui);
                self.calibration_controls(ui);
                if let Some(error) = self.feed.error() {
                    ui.label(
                        RichText::new(format!("Rerun image feed failed: {error}"))
                            .color(Color32::from_rgb(255, 153, 130)),
                    );
                }
                let snapshots = self.receivers.snapshot();
                for camera in [1, 0] {
                    Self::camera_status(ui, camera, &snapshots[camera]);
                }
                if let Some((notice, when)) = &self.notice
                    && when.elapsed() < Duration::from_secs(8)
                {
                    ui.label(RichText::new(notice).color(ACCENT));
                }
                ui.horizontal_wrapped(|ui| {
                    let source = self
                        .network
                        .source_ip
                        .map_or_else(|| "any source".into(), |ip| ip.to_string());
                    ui.small(format!(
                        "{source} -> UDP {} / {} | Rerun memory target: 256 MiB",
                        self.network.ports[0], self.network.ports[1]
                    ));
                    ui.small(if self.capture.snapshot().owned {
                        "Closing stops capture started here"
                    } else {
                        "Receive only | Closing leaves external capture unchanged"
                    });
                });
            });
        ui.separator();
        // Rerun owns the remaining region, including image pan/zoom, tabs,
        // contrast controls, and its native playback timeline.
        self.rerun_app.ui(ui, frame);
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        self.rerun_app.save(storage);
    }
}

impl Drop for Viewer {
    fn drop(&mut self) {
        self.recorder.cancel();
        self.receivers.set_recording(None);
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let Some(mut options) = Options::parse(env::args().skip(1))? else {
        return Ok(());
    };
    options.resolve_board(lutcam_viewer::discovery::find_board)?;
    let receivers = ReceiverSet::start(options.network.clone()).map_err(|error| {
        format!("Cannot open camera UDP ports: {error}. Close another receiver, or use distinct relay ports with --port0/--port1.")
    })?;
    if let Some(seconds) = options.headless {
        return headless(receivers, seconds, options.output);
    }
    let main_thread = re_viewer::MainThreadToken::i_promise_i_am_on_the_main_thread();
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    // Some native Rerun UI actions also spawn through Tokio's current runtime.
    let _runtime_guard = runtime.enter();
    let runtime_handle = re_viewer::AsyncRuntimeHandle::new_native(runtime.handle().clone());
    let mut native = re_viewer::native::eframe_options(None);
    native.viewport = native
        .viewport
        .with_app_id("lutcam-viewer")
        .with_decorations(true)
        .with_transparent(false)
        .with_title("LUTCam Live - Rerun")
        .with_inner_size([1480.0, 940.0])
        .with_min_inner_size([760.0, 620.0]);
    eframe::run_native(
        "LUTCam Live - Rerun",
        native,
        Box::new(move |cc| {
            Ok(Box::new(Viewer::new(
                cc,
                main_thread,
                runtime_handle,
                receivers,
                options,
            )?))
        }),
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use lutcam_viewer::capture::{ExposureSettings, ExposureStatus};

    #[test]
    fn calibration_file_is_checked_before_stopping_a_capture() {
        let path = std::env::temp_dir().join(format!(
            "lutcam-viewer-calibration-{}.json",
            std::process::id()
        ));
        let valid = serde_json::json!({
            "schema_version":1, "image_size":[1280,800], "baseline_mm":120.0,
            "tag_width_mm":90.0, "validated":true, "metrics":{"training_rms_px":0.2},
            "cameras":[
                {"camera":0,"fx":933.3,"fy":934.0,"cx":639.5,"cy":399.5,"distortion":[-0.1,0.02,0,0,0]},
                {"camera":1,"fx":932.0,"fy":935.0,"cx":640.0,"cy":400.0,"distortion":[-0.1,0.02,0,0,0]}
            ]
        });
        fs::write(&path, serde_json::to_vec(&valid).unwrap()).unwrap();
        let params = read_calibration(&path).unwrap();
        assert_eq!(params[0].tag_width, 1.0);
        assert_eq!(params[1].fx, 932.0);
        for (pointer, invalid) in [
            ("/baseline_mm", serde_json::json!(0)),
            ("/tag_width_mm", serde_json::json!(-1)),
            ("/cameras/1/camera", serde_json::json!(0)),
            ("/cameras/0/distortion", serde_json::json!([0, 0, 0, 0])),
            ("/cameras/0/fx", serde_json::json!("933")),
            ("/image_size", serde_json::json!([800, 1280])),
        ] {
            let mut malformed = valid.clone();
            *malformed.pointer_mut(pointer).unwrap() = invalid;
            fs::write(&path, serde_json::to_vec(&malformed).unwrap()).unwrap();
            assert!(read_calibration(&path).is_err(), "must reject {pointer}");
        }
        fs::remove_file(path).unwrap();
    }

    fn running_capture() -> CaptureSnapshot {
        CaptureSnapshot {
            state: CaptureState::Running,
            message: String::new(),
            details: None,
            owned: true,
            stderr_tail: Vec::new(),
            exposure: Some(ExposureStatus {
                id: 0,
                settings: ExposureSettings {
                    exposure_lines: 100,
                    gain_code: 16,
                },
                line_time_us: 9.1,
                min_lines: 1,
                max_lines: 885,
                min_gain_code: 16,
                max_gain_code: 255,
            }),
            exposure_pending: None,
            exposure_error: None,
        }
    }

    fn exposure_frame(received_at: Instant) -> DecodedFrame {
        DecodedFrame {
            frame_id: 0,
            pixels: Vec::new(),
            received_at,
            decoded_at: received_at + Duration::from_millis(100),
            decode_ms: 100.0,
        }
    }

    #[test]
    fn exposure_requires_two_fresh_frames_after_each_acknowledgment() {
        let start = Instant::now();
        let mut feedback = AutoExposure::default();
        let mut capture = running_capture();
        feedback.observe(&capture, start);
        let early = exposure_frame(start + Duration::from_millis(199));
        let settled = exposure_frame(start + EXPOSURE_SETTLE);
        let now = start + Duration::from_millis(250);
        assert!(!feedback.can_adjust(&capture, [Some(&early), Some(&settled)], now));
        assert!(!feedback.can_adjust(&capture, [Some(&settled), Some(&early)], now));
        assert!(!feedback.can_adjust(&capture, [Some(&settled), None], now));
        assert!(feedback.can_adjust(&capture, [Some(&settled); 2], now));

        capture.exposure.as_mut().unwrap().id = 1;
        feedback.observe(&capture, now);
        assert!(!feedback.can_adjust(&capture, [Some(&settled); 2], now));
        let after_second_ack = exposure_frame(now + EXPOSURE_SETTLE);
        assert!(feedback.can_adjust(
            &capture,
            [Some(&after_second_ack); 2],
            now + EXPOSURE_SETTLE,
        ));
        // Repeated status heartbeats must not continually restart settling.
        feedback.observe(&capture, now + EXPOSURE_SETTLE);
        assert!(feedback.can_adjust(
            &capture,
            [Some(&after_second_ack); 2],
            now + EXPOSURE_SETTLE,
        ));
    }

    #[test]
    fn exposure_rejects_stale_future_pending_and_unowned_feedback() {
        let start = Instant::now();
        let now = start + Duration::from_millis(500);
        let mut feedback = AutoExposure::default();
        let mut capture = running_capture();
        feedback.observe(&capture, start);
        let stale = exposure_frame(start + Duration::from_millis(200));
        let fresh = exposure_frame(start + Duration::from_millis(201));
        let future = exposure_frame(now + Duration::from_millis(1));
        assert!(!feedback.can_adjust(&capture, [Some(&stale); 2], now));
        assert!(!feedback.can_adjust(&capture, [Some(&future); 2], now));
        assert!(feedback.can_adjust(&capture, [Some(&fresh); 2], now));
        capture.exposure_pending = Some(1);
        assert!(!feedback.can_adjust(&capture, [Some(&fresh); 2], now));
        capture.exposure_pending = None;
        capture.owned = false;
        assert!(!feedback.can_adjust(&capture, [Some(&fresh); 2], now));
        capture.owned = true;
        for state in [
            CaptureState::Idle,
            CaptureState::Starting,
            CaptureState::Stopping,
            CaptureState::Stopped,
            CaptureState::Fault,
            CaptureState::Error,
        ] {
            capture.state = state;
            assert!(!feedback.can_adjust(&capture, [Some(&fresh); 2], now));
        }
        capture.state = CaptureState::Running;
        capture.exposure = None;
        assert!(!feedback.can_adjust(&capture, [Some(&fresh); 2], now));
    }

    #[test]
    fn exposure_meter_and_commands_are_limited_to_two_per_second() {
        let start = Instant::now();
        let mut feedback = AutoExposure::default();
        assert!(feedback.sample_due(start));
        assert!(!feedback.sample_due(start + Duration::from_millis(499)));
        assert!(feedback.sample_due(start + EXPOSURE_INTERVAL));
        let capture = running_capture();
        feedback.observe(&capture, start);
        feedback.last_request = Some(start);
        let frame = exposure_frame(start + Duration::from_millis(400));
        assert!(!feedback.can_adjust(
            &capture,
            [Some(&frame); 2],
            start + Duration::from_millis(499),
        ));
        // Disabling/re-enabling must not bypass the last attempted request.
        feedback.set_enabled(false);
        assert!(!feedback.can_adjust(&capture, [Some(&frame); 2], start + EXPOSURE_INTERVAL));
        feedback.set_enabled(true);
        assert!(!feedback.can_adjust(
            &capture,
            [Some(&frame); 2],
            start + Duration::from_millis(499),
        ));
        assert!(feedback.can_adjust(&capture, [Some(&frame); 2], start + EXPOSURE_INTERVAL));
    }

    #[test]
    fn exposure_errors_need_explicit_retry_and_late_ack_does_not_enable() {
        let start = Instant::now();
        let mut feedback = AutoExposure::default();
        let mut capture = running_capture();
        capture.exposure_error = Some(ExposureError {
            id: 1,
            message: "Update timed out".into(),
        });
        feedback.observe(&capture, start);
        assert!(!feedback.enabled);
        assert_eq!(feedback.error.as_deref(), Some("Update timed out"));
        capture.exposure.as_mut().unwrap().id = 1;
        feedback.observe(&capture, start + EXPOSURE_INTERVAL);
        assert!(!feedback.enabled);
        assert!(feedback.error.is_some());

        feedback.set_enabled(true);
        feedback.observe(&capture, start + EXPOSURE_INTERVAL);
        assert!(feedback.enabled);
        assert!(feedback.error.is_none());
        // Protocol errors may reuse the currently applied ID. A new message
        // still disables feedback after an explicit retry of the old error.
        capture.exposure_error.as_mut().unwrap().message = "Malformed status".into();
        feedback.observe(&capture, start + EXPOSURE_INTERVAL);
        assert!(!feedback.enabled);
        assert_eq!(feedback.error.as_deref(), Some("Malformed status"));
    }

    #[test]
    fn exposure_start_resets_reused_ids_but_preserves_user_preferences() {
        let start = Instant::now();
        let mut feedback = AutoExposure::default();
        let capture = running_capture();
        feedback.observe(&capture, start);
        feedback.compensation_ev = 0.75;
        feedback.set_enabled(false);
        feedback.last_request = Some(start);
        feedback.brightness = [Some(80.0); 2];
        feedback.reset_tracking();
        assert!(!feedback.enabled);
        assert_eq!(feedback.compensation_ev, 0.75);
        assert_eq!(feedback.brightness, [None; 2]);
        assert!(feedback.last_request.is_none());
        assert!(feedback.acknowledgment.is_none());
        let restarted = start + Duration::from_secs(1);
        feedback.observe(&capture, restarted);
        assert_eq!(feedback.acknowledgment, Some((0, restarted)));
    }

    #[test]
    fn reject_bad_cli() {
        for args in [
            vec!["--port0", "0"],
            vec!["--port1", "6000"],
            vec!["--headless-seconds", "NaN"],
            vec!["--headless-seconds", "0"],
            vec!["--calibration-seconds", "0"],
            vec!["--calibration-seconds", "86401"],
            vec!["--calibration-seconds", "NaN"],
            vec!["--calibration-seconds", "1.5"],
            vec!["--bind", "bad"],
            vec!["--board", "bad"],
            vec!["--board", "0.0.0.0"],
            vec!["--board", "255.255.255.255"],
        ] {
            assert!(Options::parse(args.into_iter().map(str::to_string)).is_err());
        }
    }
    #[test]
    fn calibration_duration_defaults_to_one_minute_and_accepts_custom_seconds() {
        let defaults = Options::parse(std::iter::empty()).unwrap().unwrap();
        assert_eq!(defaults.calibration_seconds, 60);
        for seconds in ["1", "15", "120", "86400"] {
            let options = Options::parse(
                ["--calibration-seconds", seconds]
                    .into_iter()
                    .map(str::to_string),
            )
            .unwrap()
            .unwrap();
            assert_eq!(options.calibration_seconds.to_string(), seconds);
        }
    }

    #[test]
    fn board_selects_default_source_but_preserves_explicit_relay_filter() {
        let board = Ipv4Addr::new(192, 0, 2, 10);
        let mut options = Options::parse(["--board", "192.0.2.10"].into_iter().map(str::to_string))
            .unwrap()
            .unwrap();
        options
            .resolve_board(|_| panic!("explicit board must bypass discovery"))
            .unwrap();
        assert_eq!(options.board, Some(board));
        assert_eq!(options.network.source_ip, Some(board));
        for args in [
            vec!["--board", "192.0.2.10", "--source", "any"],
            vec!["--source", "any", "--board", "192.0.2.10"],
        ] {
            let options = Options::parse(args.into_iter().map(str::to_string))
                .unwrap()
                .unwrap();
            assert_eq!(options.network.source_ip, None);
        }
        let options = Options::parse(
            ["--board", "192.0.2.10", "--source", "127.0.0.1"]
                .into_iter()
                .map(str::to_string),
        )
        .unwrap()
        .unwrap();
        assert_eq!(options.network.source_ip, Some(Ipv4Addr::LOCALHOST));
    }

    #[test]
    fn default_startup_discovers_control_and_video_address_together() {
        let board = Ipv4Addr::new(192, 0, 2, 75);
        for args in [vec![], vec!["--headless-seconds", "1"]] {
            let mut options = Options::parse(args.into_iter().map(str::to_string))
                .unwrap()
                .unwrap();
            assert_eq!(options.board, None);
            options
                .resolve_board(|bind| {
                    assert_eq!(bind, Ipv4Addr::UNSPECIFIED);
                    Ok(board)
                })
                .unwrap();
            assert_eq!(options.board, Some(board));
            assert_eq!(options.network.source_ip, Some(board));
        }
    }

    #[test]
    fn discovery_honors_interface_and_preserves_explicit_source_filters() {
        for source in ["any", "127.0.0.1"] {
            let mut options = Options::parse(
                ["--bind", "192.0.2.1", "--source", source]
                    .into_iter()
                    .map(str::to_string),
            )
            .unwrap()
            .unwrap();
            let filter = options.network.source_ip;
            options
                .resolve_board(|bind| {
                    assert_eq!(bind, Ipv4Addr::new(192, 0, 2, 1));
                    Ok(Ipv4Addr::new(192, 0, 2, 75))
                })
                .unwrap();
            assert_eq!(options.network.source_ip, filter);
        }
    }

    #[test]
    fn explicit_headless_source_needs_no_board_or_discovery() {
        for source in ["any", "127.0.0.1"] {
            let mut options = Options::parse(
                ["--source", source, "--headless-seconds", "1"]
                    .into_iter()
                    .map(str::to_string),
            )
            .unwrap()
            .unwrap();
            options
                .resolve_board(|_| panic!("receive-only source must bypass discovery"))
                .unwrap();
            assert_eq!(options.board, None);
        }
    }

    #[test]
    fn failed_discovery_does_not_fall_back_to_a_stale_address() {
        let mut options = Options::parse(std::iter::empty()).unwrap().unwrap();
        assert_eq!(
            options.resolve_board(|_| Err("no board".into())).unwrap_err(),
            "no board"
        );
        assert_eq!(options.board, None);
        assert_eq!(options.network.source_ip, None);
    }
}
