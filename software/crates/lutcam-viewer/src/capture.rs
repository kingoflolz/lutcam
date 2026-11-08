//! Own the optional camera-control helper without blocking the GUI thread.
//!
//! Merely constructing a controller does not start capture. The Python helper
//! owns hardware cleanup; the viewer sends it a graceful stop request before
//! closing, and reports when that cleanup cannot be confirmed.

use std::collections::VecDeque;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::net::Ipv4Addr;
use std::process::{Command, ExitStatus, Stdio};
use std::sync::{Arc, Mutex, MutexGuard, mpsc};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use serde_json::Value;

const STOP_TIMEOUT: Duration = Duration::from_secs(5);
const POLL_INTERVAL: Duration = Duration::from_millis(20);
const MAX_LOG_LINES: usize = 24;
const MAX_LINE_BYTES: usize = 16 * 1024;
const EXPOSURE_TIMEOUT: Duration = Duration::from_secs(3);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CaptureState {
    Idle,
    Starting,
    Running,
    Stopping,
    Stopped,
    Fault,
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExposureSettings {
    pub exposure_lines: u32,
    pub gain_code: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExposureStatus {
    pub id: u32,
    pub settings: ExposureSettings,
    pub line_time_us: f64,
    pub min_lines: u32,
    pub max_lines: u32,
    pub min_gain_code: u32,
    pub max_gain_code: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExposureError {
    pub id: u32,
    pub message: String,
}

#[derive(Clone, Copy)]
struct ExposureRequest {
    id: u32,
    settings: ExposureSettings,
    queued_at: Instant,
}

#[derive(Clone, Debug)]
pub struct CaptureSnapshot {
    pub state: CaptureState,
    pub message: String,
    pub details: Option<Value>,
    /// True until the helper process has exited and been reaped, including
    /// time spent cleaning up after a terminal JSON status message.
    pub owned: bool,
    pub stderr_tail: Vec<String>,
    pub exposure: Option<ExposureStatus>,
    pub exposure_pending: Option<u32>,
    /// Retained across ordinary heartbeats, until a newer request succeeds.
    pub exposure_error: Option<ExposureError>,
}

struct SharedState {
    snapshot: CaptureSnapshot,
    generation: u64,
    stop_requested: bool,
    log: VecDeque<String>,
    next_exposure_id: u32,
    last_exposure_request: Option<ExposureRequest>,
}

type Shared = Arc<Mutex<SharedState>>;

enum Control {
    Stop,
    Kill,
    Exposure(ExposureRequest),
    AutoExposure { enabled: bool, ev_quarters: i32 },
}

struct Worker {
    control: mpsc::Sender<Control>,
    done: mpsc::Receiver<()>,
    thread: JoinHandle<()>,
}

pub struct CaptureController {
    shared: Shared,
    worker: Option<Worker>,
}

impl Default for CaptureController {
    fn default() -> Self {
        Self::new()
    }
}

impl CaptureController {
    pub fn set_auto_exposure(&self, enabled: bool, ev_quarters: i32) -> Result<(), String> {
        let snapshot = self.snapshot();
        if !snapshot.owned
            || snapshot.state != CaptureState::Running
            || !(-8..=8).contains(&ev_quarters)
        {
            return Err(
                "Firmware exposure needs a running capture and compensation within +/-2 EV".into(),
            );
        }
        self.worker
            .as_ref()
            .ok_or("Capture helper is unavailable")?
            .control
            .send(Control::AutoExposure {
                enabled,
                ev_quarters,
            })
            .map_err(|_| "Capture helper has exited".into())
    }
    pub fn new() -> Self {
        Self {
            shared: Arc::new(Mutex::new(SharedState {
                snapshot: CaptureSnapshot {
                    state: CaptureState::Idle,
                    message: "Capture has not been started by this viewer".to_owned(),
                    details: None,
                    owned: false,
                    stderr_tail: Vec::new(),
                    exposure: None,
                    exposure_pending: None,
                    exposure_error: None,
                },
                generation: 0,
                stop_requested: false,
                log: VecDeque::new(),
                next_exposure_id: 1,
                last_exposure_request: None,
            })),
            worker: None,
        }
    }

    pub fn start(
        &mut self,
        host: Ipv4Addr,
        bind_ip: Ipv4Addr,
        ports: [u16; 2],
    ) -> Result<(), String> {
        self.start_with_calibration(host, bind_ip, ports, None)
    }

    pub fn start_with_calibration(
        &mut self,
        host: Ipv4Addr,
        bind_ip: Ipv4Addr,
        ports: [u16; 2],
        calibration: Option<&std::path::Path>,
    ) -> Result<(), String> {
        let mut command = helper_command(host, bind_ip, ports)?;
        if let Some(path) = calibration {
            let path = path
                .canonicalize()
                .map_err(|error| format!("Cannot open calibration {}: {error}", path.display()))?;
            command.arg("--calibration").arg(path);
        }
        self.start_command(command, STOP_TIMEOUT)
    }

    /// Queue a graceful paired-camera stop. This does not wait for I/O or
    /// hardware cleanup and is safe to call from a GUI callback.
    pub fn stop(&mut self) -> Result<(), String> {
        let Some(worker) = &self.worker else {
            return Ok(());
        };
        {
            let mut state = lock_state(&self.shared);
            if !state.snapshot.owned || state.stop_requested {
                return Ok(());
            }
            state.stop_requested = true;
            state.snapshot.exposure_pending = None;
            state.snapshot.state = CaptureState::Stopping;
            state.snapshot.message = "Stopping both cameras and draining capture".to_owned();
        }
        worker
            .control
            .send(Control::Stop)
            .map_err(|_| "Capture helper supervisor has already exited".to_owned())
    }

    pub fn snapshot(&self) -> CaptureSnapshot {
        lock_state(&self.shared).snapshot.clone()
    }

    /// A terminal status can arrive just before the supervisor returns. Wait
    /// for that thread too before queuing a restart; this never joins or waits.
    pub fn ready_to_start(&self) -> bool {
        !self.snapshot().owned
            && self
                .worker
                .as_ref()
                .is_none_or(|worker| worker.thread.is_finished())
    }

    /// Queue one paired exposure update; hardware I/O remains in the helper.
    /// A request is never retried automatically if its acknowledgment is lost.
    pub fn set_exposure(&mut self, exposure_lines: u32, gain_code: u32) -> Result<u32, String> {
        let worker = self
            .worker
            .as_ref()
            .ok_or("Capture is not owned by this viewer")?;
        let (generation, request) = {
            let mut state = lock_state(&self.shared);
            if !state.snapshot.owned
                || state.snapshot.state != CaptureState::Running
                || state.stop_requested
            {
                return Err(
                    "Exposure can only change during this viewer's running capture".to_owned(),
                );
            }
            if state.snapshot.exposure_pending.is_some() {
                return Err("An exposure update is already awaiting acknowledgment".to_owned());
            }
            let limits = state
                .snapshot
                .exposure
                .ok_or("Camera exposure limits have not arrived yet")?;
            if !(limits.min_lines..=limits.max_lines).contains(&exposure_lines)
                || !(limits.min_gain_code..=limits.max_gain_code).contains(&gain_code)
            {
                return Err(format!(
                    "Exposure must be {}..={} lines and gain code {}..={}",
                    limits.min_lines, limits.max_lines, limits.min_gain_code, limits.max_gain_code
                ));
            }
            let id = state.next_exposure_id;
            state.next_exposure_id = id
                .checked_add(1)
                .ok_or("Exposure command IDs are exhausted; restart capture")?;
            let request = ExposureRequest {
                id,
                settings: ExposureSettings {
                    exposure_lines,
                    gain_code,
                },
                queued_at: Instant::now(),
            };
            state.snapshot.exposure_pending = Some(id);
            state.last_exposure_request = Some(request);
            (state.generation, request)
        };
        if worker.control.send(Control::Exposure(request)).is_err() {
            let message =
                "Could not queue exposure update: capture supervisor has exited".to_owned();
            exposure_failure(&self.shared, generation, request.id, message.clone());
            return Err(message);
        }
        Ok(request.id)
    }

    fn start_command(&mut self, command: Command, stop_timeout: Duration) -> Result<(), String> {
        if self
            .worker
            .as_ref()
            .is_some_and(|worker| worker.thread.is_finished())
        {
            let worker = self.worker.take().unwrap();
            let _ = worker.thread.join();
        }
        if self.worker.is_some() {
            return Err("The previous capture helper is still running or cleaning up".to_owned());
        }
        let generation = {
            let mut state = lock_state(&self.shared);
            state.generation = state.generation.wrapping_add(1);
            state.stop_requested = false;
            state.log.clear();
            state.next_exposure_id = 1;
            state.last_exposure_request = None;
            state.snapshot = CaptureSnapshot {
                state: CaptureState::Starting,
                message: "Starting paired camera capture".to_owned(),
                details: None,
                owned: true,
                stderr_tail: Vec::new(),
                exposure: None,
                exposure_pending: None,
                exposure_error: None,
            };
            state.generation
        };
        let (control_tx, control_rx) = mpsc::channel();
        let (done_tx, done_rx) = mpsc::channel();
        let shared = Arc::clone(&self.shared);
        match thread::Builder::new()
            .name("camera-control".to_owned())
            .spawn(move || {
                supervise(command, control_rx, &shared, generation, stop_timeout);
                let _ = done_tx.send(());
            }) {
            Ok(thread) => {
                self.worker = Some(Worker {
                    control: control_tx,
                    done: done_rx,
                    thread,
                });
                Ok(())
            }
            Err(error) => {
                let message = format!("Could not start capture supervisor: {error}");
                fail(&self.shared, generation, message.clone(), false);
                Err(message)
            }
        }
    }
}

impl Drop for CaptureController {
    fn drop(&mut self) {
        let _ = self.stop();
        let Some(worker) = self.worker.take() else {
            return;
        };
        // The supervisor enforces the same deadline. This extra allowance is
        // only for process reaping and normal pipe-reader shutdown.
        let mut done = worker
            .done
            .recv_timeout(STOP_TIMEOUT + Duration::from_millis(500));
        if matches!(done, Err(mpsc::RecvTimeoutError::Timeout)) {
            let _ = worker.control.send(Control::Kill);
            done = worker.done.recv_timeout(Duration::from_millis(500));
        }
        if !matches!(done, Err(mpsc::RecvTimeoutError::Timeout)) || worker.thread.is_finished() {
            let _ = worker.thread.join();
        } else {
            // Do not hang window shutdown on a broken OS process wait. The
            // supervisor retains process ownership and the kill request.
            eprintln!("Capture cleanup is still pending; camera shutdown is unconfirmed");
        }
    }
}

fn helper_command(host: Ipv4Addr, bind_ip: Ipv4Addr, ports: [u16; 2]) -> Result<Command, String> {
    if bind_ip.is_loopback() {
        return Err("Camera capture cannot target a loopback address; bind to the host's Ethernet address or 0.0.0.0".to_owned());
    }
    let mut command = crate::helper::command("tools.lutcam.live");
    command
        .args(["--host", &host.to_string()])
        .args([
            "--port0",
            &ports[0].to_string(),
            "--port1",
            &ports[1].to_string(),
        ]);
    if !bind_ip.is_unspecified() {
        command.args(["--destination", &bind_ip.to_string()]);
    }
    Ok(command)
}

fn lock_state(shared: &Shared) -> MutexGuard<'_, SharedState> {
    shared
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

fn update(shared: &Shared, generation: u64, f: impl FnOnce(&mut SharedState)) {
    let mut state = lock_state(shared);
    if state.generation == generation {
        f(&mut state);
    }
}

fn fail(shared: &Shared, generation: u64, message: String, owned: bool) {
    update(shared, generation, |state| {
        state.snapshot.state = CaptureState::Error;
        state.snapshot.message = message;
        state.snapshot.owned = owned;
    });
}

fn append_log(shared: &Shared, generation: u64, line: String) {
    update(shared, generation, |state| {
        if state.log.len() == MAX_LOG_LINES {
            state.log.pop_front();
        }
        state.log.push_back(line);
        state.snapshot.stderr_tail = state.log.iter().cloned().collect();
    });
}

fn exposure_u32(value: &Value, key: &str) -> Result<u32, String> {
    value
        .get(key)
        .and_then(Value::as_u64)
        .and_then(|number| u32::try_from(number).ok())
        .ok_or_else(|| format!("Exposure status has an invalid {key}"))
}

fn parse_exposure(value: &Value) -> Result<ExposureStatus, String> {
    let status = ExposureStatus {
        id: exposure_u32(value, "id")?,
        settings: ExposureSettings {
            exposure_lines: exposure_u32(value, "exposure_lines")?,
            gain_code: exposure_u32(value, "gain_code")?,
        },
        line_time_us: value
            .get("line_time_us")
            .and_then(Value::as_f64)
            .filter(|time| time.is_finite() && *time > 0.0)
            .ok_or("Exposure status has an invalid line_time_us")?,
        min_lines: exposure_u32(value, "min_lines")?,
        max_lines: exposure_u32(value, "max_lines")?,
        min_gain_code: exposure_u32(value, "min_gain_code")?,
        max_gain_code: exposure_u32(value, "max_gain_code")?,
    };
    if status.min_lines == 0
        || status.min_lines > status.max_lines
        || status.min_gain_code == 0
        || status.min_gain_code > status.max_gain_code
        || !(status.min_lines..=status.max_lines).contains(&status.settings.exposure_lines)
        || !(status.min_gain_code..=status.max_gain_code).contains(&status.settings.gain_code)
    {
        return Err("Exposure status has inconsistent settings or limits".to_owned());
    }
    Ok(status)
}

fn record_exposure_error(state: &mut SharedState, id: u32, message: String) {
    if state
        .snapshot
        .exposure
        .is_some_and(|applied| applied.id > id)
        || state
            .snapshot
            .exposure_error
            .as_ref()
            .is_some_and(|error| error.id > id)
    {
        return;
    }
    if state.snapshot.exposure_pending == Some(id) {
        state.snapshot.exposure_pending = None;
    }
    state.snapshot.exposure_error = Some(ExposureError { id, message });
}

fn exposure_failure(shared: &Shared, generation: u64, id: u32, message: String) {
    update(shared, generation, |state| {
        record_exposure_error(state, id, message)
    });
}

fn apply_exposure_details(state: &mut SharedState, details: &Value) {
    if let Some(value) = details.get("exposure") {
        match parse_exposure(value) {
            Ok(exposure) if exposure.id >= state.next_exposure_id => {
                let applied_id = state.snapshot.exposure.map_or(0, |status| status.id);
                record_exposure_error(
                    state,
                    applied_id,
                    format!(
                        "Ignoring unknown exposure acknowledgment ID {}",
                        exposure.id
                    ),
                );
            }
            Ok(exposure)
                if state
                    .snapshot
                    .exposure
                    .is_some_and(|previous| exposure.id < previous.id) => {}
            Ok(exposure)
                if state
                    .last_exposure_request
                    .is_some_and(|request| exposure.id < request.id)
                    && state
                        .snapshot
                        .exposure
                        .is_none_or(|previous| exposure.id != previous.id) =>
            {
                // An older timed-out request was superseded. Its requested
                // settings are no longer retained, so do not accept an
                // unverifiable late acknowledgment as the applied state.
            }
            Ok(exposure) => {
                // Firmware can change the measured settings after a host
                // command is acknowledged, without issuing another host ID.
                // A new manual acknowledgment must still match its request.
                let firmware_update = state
                    .snapshot
                    .exposure
                    .is_some_and(|previous| previous.id == exposure.id)
                    && details.get("firmware_exposure").is_some_and(|firmware| {
                        firmware.get("enabled").is_some_and(Value::is_boolean)
                            && firmware.get("exposure_lines") == value.get("exposure_lines")
                            && firmware.get("gain_code") == value.get("gain_code")
                    });
                let request_mismatch = state.last_exposure_request.is_some_and(|request| {
                    request.id == exposure.id && request.settings != exposure.settings
                });
                let applied_mismatch = state.snapshot.exposure.is_some_and(|previous| {
                    previous.id == exposure.id && previous.settings != exposure.settings
                });
                if !firmware_update && (request_mismatch || applied_mismatch) {
                    record_exposure_error(
                        state,
                        exposure.id,
                        "Exposure acknowledgment settings do not match the requested update"
                            .to_owned(),
                    );
                } else {
                    state.snapshot.exposure = Some(exposure);
                    if state.snapshot.exposure_pending == Some(exposure.id) {
                        state.snapshot.exposure_pending = None;
                    }
                    if state
                        .snapshot
                        .exposure_error
                        .as_ref()
                        .is_some_and(|error| exposure.id > error.id)
                    {
                        state.snapshot.exposure_error = None;
                    }
                }
            }
            Err(message) => {
                let applied_id = state.snapshot.exposure.map_or(0, |status| status.id);
                record_exposure_error(state, applied_id, message);
            }
        }
    }
    if let Some(error) = details.get("exposure_error") {
        match exposure_u32(error, "id") {
            Ok(id) if id < state.next_exposure_id => {
                let message = error
                    .get("message")
                    .and_then(Value::as_str)
                    .unwrap_or("Camera exposure update failed")
                    .to_owned();
                record_exposure_error(state, id, message);
            }
            _ => {
                let applied_id = state.snapshot.exposure.map_or(0, |status| status.id);
                record_exposure_error(
                    state,
                    applied_id,
                    "Ignoring exposure error with an unknown command ID".to_owned(),
                );
            }
        }
    }
}

fn expire_exposure(shared: &Shared, generation: u64) {
    update(shared, generation, |state| {
        if !state.stop_requested
            && let Some(request) = state.last_exposure_request
            && state.snapshot.exposure_pending == Some(request.id)
            && request.queued_at.elapsed() >= EXPOSURE_TIMEOUT
        {
            record_exposure_error(state, request.id,
                    "Exposure acknowledgment timed out; the update was not retried and applied settings are unconfirmed".to_owned());
        }
    });
}

fn parse_status(line: &str) -> Result<(CaptureState, String, Option<Value>), String> {
    let value: Value = serde_json::from_str(line).map_err(|error| error.to_string())?;
    let state = match value.get("state").and_then(Value::as_str) {
        Some("starting") => CaptureState::Starting,
        Some("running") => CaptureState::Running,
        Some("stopping") => CaptureState::Stopping,
        Some("stopped") => CaptureState::Stopped,
        Some("fault") => CaptureState::Fault,
        Some("error") => CaptureState::Error,
        _ => return Err("missing or unknown capture state".to_owned()),
    };
    let message = value
        .get("message")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_owned();
    let details = value
        .get("details")
        .filter(|value| !value.is_null())
        .cloned();
    Ok((state, message, details))
}

fn handle_stdout(shared: &Shared, generation: u64, line: String) {
    match parse_status(&line) {
        Ok((new_state, message, details)) => update(shared, generation, |state| {
            if !state.snapshot.owned {
                return;
            }
            // Stop may have been clicked while Python was still initializing.
            // A delayed startup event must not make the UI appear running.
            if state.stop_requested
                && matches!(new_state, CaptureState::Starting | CaptureState::Running)
            {
                return;
            }
            if new_state == CaptureState::Running
                && let Some(details) = &details
            {
                apply_exposure_details(state, details);
            }
            if matches!(
                new_state,
                CaptureState::Stopping
                    | CaptureState::Stopped
                    | CaptureState::Fault
                    | CaptureState::Error
            ) {
                state.snapshot.exposure_pending = None;
            }
            state.snapshot.state = new_state;
            state.snapshot.message = message;
            state.snapshot.details = details;
        }),
        Err(_) => append_log(shared, generation, format!("helper stdout: {line}")),
    }
}

/// Limit each line while continuing to consume its remainder. An accidental
/// giant traceback or log line cannot create an unbounded allocation.
fn read_bounded_line(reader: &mut impl BufRead) -> io::Result<Option<(String, bool)>> {
    let mut line = Vec::new();
    let mut truncated = false;
    loop {
        let available = reader.fill_buf()?;
        if available.is_empty() {
            return if line.is_empty() && !truncated {
                Ok(None)
            } else {
                Ok(Some((
                    String::from_utf8_lossy(&line).into_owned(),
                    truncated,
                )))
            };
        }
        let newline = available.iter().position(|byte| *byte == b'\n');
        let consumed = newline.map_or(available.len(), |position| position + 1);
        let content = newline.unwrap_or(available.len());
        let kept = content.min(MAX_LINE_BYTES.saturating_sub(line.len()));
        line.extend_from_slice(&available[..kept]);
        truncated |= kept < content;
        reader.consume(consumed);
        if newline.is_some() {
            if line.last() == Some(&b'\r') {
                line.pop();
            }
            return Ok(Some((
                String::from_utf8_lossy(&line).into_owned(),
                truncated,
            )));
        }
    }
}

fn pipe_reader(
    pipe: impl Read + Send + 'static,
    shared: Shared,
    generation: u64,
    stdout: bool,
) -> io::Result<JoinHandle<()>> {
    thread::Builder::new()
        .name(
            if stdout {
                "camera-status"
            } else {
                "camera-errors"
            }
            .to_owned(),
        )
        .spawn(move || {
            let mut reader = BufReader::new(pipe);
            loop {
                match read_bounded_line(&mut reader) {
                    Ok(Some((mut line, truncated))) => {
                        if truncated {
                            line.push_str(" [line truncated]");
                        }
                        if stdout && !truncated {
                            handle_stdout(&shared, generation, line);
                        } else {
                            append_log(&shared, generation, line);
                        }
                    }
                    Ok(None) => break,
                    Err(error) => {
                        append_log(&shared, generation, format!("helper pipe read: {error}"));
                        break;
                    }
                }
            }
        })
}

fn write_exposure(
    writer: &mut impl Write,
    shared: &Shared,
    generation: u64,
    request: ExposureRequest,
) -> io::Result<bool> {
    {
        let state = lock_state(shared);
        // Stop is latched before it enters the queue, so it takes priority
        // over an exposure command already waiting in that queue.
        if state.generation != generation
            || state.stop_requested
            || !state.snapshot.owned
            || state.snapshot.state != CaptureState::Running
            || state.snapshot.exposure_pending != Some(request.id)
        {
            return Ok(false);
        }
    }
    let command = serde_json::json!({
        "command": "exposure",
        "id": request.id,
        "exposure_lines": request.settings.exposure_lines,
        "gain_code": request.settings.gain_code,
    });
    let mut line = command.to_string();
    line.push('\n');
    writer.write_all(line.as_bytes())?;
    Ok(true)
}

fn supervise(
    mut command: Command,
    controls: mpsc::Receiver<Control>,
    shared: &Shared,
    generation: u64,
    stop_timeout: Duration,
) {
    command
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = match command.spawn() {
        Ok(child) => child,
        Err(error) => {
            fail(
                shared,
                generation,
                format!("Could not launch camera capture helper: {error}"),
                false,
            );
            return;
        }
    };
    let mut stdin = child.stdin.take();
    let stdout = pipe_reader(
        child.stdout.take().unwrap(),
        Arc::clone(shared),
        generation,
        true,
    );
    let stderr = pipe_reader(
        child.stderr.take().unwrap(),
        Arc::clone(shared),
        generation,
        false,
    );
    let readers = match (stdout, stderr) {
        (Ok(stdout), Ok(stderr)) => [stdout, stderr],
        (stdout, stderr) => {
            let message = format!(
                "Could not read capture helper output: {}",
                stdout.err().or_else(|| stderr.err()).unwrap()
            );
            let _ = child.kill();
            let _ = child.wait();
            fail(
                shared,
                generation,
                format!("{message}; camera shutdown is unconfirmed"),
                false,
            );
            return;
        }
    };
    let mut deadline = None;
    let mut forced = false;
    let exit = loop {
        expire_exposure(shared, generation);
        match controls.recv_timeout(POLL_INTERVAL) {
            Ok(Control::AutoExposure {
                enabled,
                ev_quarters,
            }) => {
                let state = lock_state(shared);
                if state.generation == generation && !state.stop_requested {
                    let id = state.snapshot.exposure.map_or(0, |status| status.id);
                    let command = serde_json::json!({"command":"auto_exposure", "id":id,
                                                     "enabled":enabled, "ev_quarters":ev_quarters});
                    if let Some(input) = stdin.as_mut() {
                        if let Err(error) = writeln!(input, "{command}") {
                            drop(state);
                            exposure_failure(
                                shared,
                                generation,
                                id,
                                format!("Could not send firmware exposure settings: {error}"),
                            );
                        }
                    }
                }
            }
            Ok(Control::Exposure(request)) => {
                let result = match stdin.as_mut() {
                    Some(input) => write_exposure(input, shared, generation, request),
                    None => Err(io::Error::new(
                        io::ErrorKind::BrokenPipe,
                        "capture helper input is closed",
                    )),
                };
                if let Err(error) = result {
                    exposure_failure(
                        shared,
                        generation,
                        request.id,
                        format!(
                            "Could not send exposure update: {error}; applied settings are unconfirmed"
                        ),
                    );
                }
            }
            Ok(Control::Stop) | Err(mpsc::RecvTimeoutError::Disconnected) => {
                if deadline.is_none() {
                    deadline = Some(Instant::now() + stop_timeout);
                    if let Some(mut input) = stdin.take()
                        && let Err(error) = input.write_all(b"stop\n")
                    {
                        append_log(
                            shared,
                            generation,
                            format!("Could not send graceful stop: {error}"),
                        );
                    }
                } else {
                    // A dropped controller disconnects the channel; avoid
                    // spinning while its already requested cleanup completes.
                    thread::sleep(POLL_INTERVAL);
                }
            }
            Ok(Control::Kill) => {
                forced = true;
                let _ = child.kill();
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
        }
        if !forced && deadline.is_some_and(|deadline| Instant::now() >= deadline) {
            forced = true;
            let _ = child.kill();
        }
        match child.try_wait() {
            Ok(Some(status)) => break Ok(status),
            Ok(None) => {}
            Err(error) => {
                let _ = child.kill();
                let _ = child.wait();
                break Err(error);
            }
        }
    };
    // Usually both readers are at EOF already. Bound this wait because an
    // accidentally inherited pipe in a helper descendant could stay open.
    let readers_deadline = Instant::now() + Duration::from_millis(200);
    while readers.iter().any(|reader| !reader.is_finished()) && Instant::now() < readers_deadline {
        thread::sleep(Duration::from_millis(2));
    }
    for reader in readers {
        if reader.is_finished() {
            let _ = reader.join();
        }
    }
    finish(shared, generation, exit, forced);
}

fn finish(shared: &Shared, generation: u64, exit: io::Result<ExitStatus>, forced: bool) {
    update(shared, generation, |state| {
        state.snapshot.owned = false;
        state.snapshot.exposure_pending = None;
        if forced {
            state.snapshot.state = CaptureState::Error;
            state.snapshot.message = "Capture helper did not stop in time and was terminated; camera shutdown is unconfirmed".to_owned();
            return;
        }
        match exit {
            Ok(status) if status.success() => {
                // Sending Stop is intent, not proof that cleanup happened.
                // Only the helper's explicit terminal event can confirm it.
                if !matches!(
                    state.snapshot.state,
                    CaptureState::Stopped | CaptureState::Fault | CaptureState::Error
                ) {
                    state.snapshot.state = CaptureState::Error;
                    state.snapshot.message =
                        "Capture helper exited unexpectedly without confirming camera shutdown"
                            .to_owned();
                }
            }
            Ok(status) => {
                if !matches!(
                    state.snapshot.state,
                    CaptureState::Fault | CaptureState::Error
                ) {
                    state.snapshot.state = CaptureState::Error;
                    state.snapshot.message = format!(
                        "Capture helper exited with {status}; camera shutdown is unconfirmed"
                    );
                }
            }
            Err(error) => {
                state.snapshot.state = CaptureState::Error;
                state.snapshot.message = format!(
                    "Could not wait for capture helper: {error}; camera shutdown is unconfirmed"
                );
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn terminal_status_does_not_make_an_active_supervisor_restartable() {
        let mut controller = CaptureController::new();
        let (release, wait) = mpsc::channel();
        let (control, _) = mpsc::channel();
        let (finished, done) = mpsc::channel();
        let thread = thread::spawn(move || {
            wait.recv().unwrap();
            let _ = finished.send(());
        });
        controller.worker = Some(Worker {
            control,
            done,
            thread,
        });
        // finish() publishes unowned before the supervisor thread returns.
        assert!(!controller.snapshot().owned);
        assert!(!controller.ready_to_start());
        release.send(()).unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while !controller.ready_to_start() && Instant::now() < deadline {
            thread::yield_now();
        }
        assert!(controller.ready_to_start());
    }

    fn exposure_json(id: u32, exposure_lines: u32, gain_code: u32) -> Value {
        serde_json::json!({"id":id, "exposure_lines":exposure_lines, "gain_code":gain_code,
            "line_time_us":9.1, "min_lines":1, "max_lines":885,
            "min_gain_code":16, "max_gain_code":255})
    }

    fn exposure_test_state() -> Shared {
        let controller = CaptureController::new();
        let shared = Arc::clone(&controller.shared);
        {
            let mut state = lock_state(&shared);
            state.snapshot.owned = true;
            state.snapshot.state = CaptureState::Running;
            state.snapshot.exposure = Some(parse_exposure(&exposure_json(0, 400, 16)).unwrap());
        }
        shared
    }

    fn firmware_exposure_json(id: u32, lines: u32, gain: u32, enabled: bool) -> Value {
        serde_json::json!({"exposure": exposure_json(id, lines, gain),
            "firmware_exposure": {"exposure_lines": lines, "gain_code": gain,
                "enabled": enabled, "target": 100, "means": [95, 105], "heartbeat": 100}})
    }

    #[test]
    fn firmware_telemetry_can_change_settings_after_an_acknowledged_command() {
        let shared = exposure_test_state();
        let mut state = lock_state(&shared);
        apply_exposure_details(&mut state, &firmware_exposure_json(0, 500, 16, true));
        assert_eq!(
            state.snapshot.exposure.unwrap().settings.exposure_lines,
            500
        );
        assert!(state.snapshot.exposure_error.is_none());
        state.next_exposure_id = 2;
        state.last_exposure_request = Some(ExposureRequest {
            id: 1,
            settings: ExposureSettings {
                exposure_lines: 200,
                gain_code: 32,
            },
            queued_at: Instant::now(),
        });
        state.snapshot.exposure_pending = Some(1);
        apply_exposure_details(&mut state, &firmware_exposure_json(0, 600, 16, true));
        assert_eq!(state.snapshot.exposure_pending, Some(1));
        apply_exposure_details(&mut state, &firmware_exposure_json(1, 200, 32, false));
        assert_eq!(state.snapshot.exposure_pending, None);
        // Re-enabling AE reuses the last manual command ID. Disabling it can
        // also report settings that changed since the previous telemetry.
        for enabled in [true, false] {
            apply_exposure_details(&mut state, &firmware_exposure_json(1, 300, 16, enabled));
            assert_eq!(
                state.snapshot.exposure.unwrap().settings.exposure_lines,
                300
            );
            assert!(state.snapshot.exposure_error.is_none());
        }
    }

    #[test]
    fn firmware_telemetry_does_not_acknowledge_an_incorrect_manual_update() {
        let shared = exposure_test_state();
        let mut state = lock_state(&shared);
        state.next_exposure_id = 2;
        state.last_exposure_request = Some(ExposureRequest {
            id: 1,
            settings: ExposureSettings {
                exposure_lines: 200,
                gain_code: 32,
            },
            queued_at: Instant::now(),
        });
        state.snapshot.exposure_pending = Some(1);
        apply_exposure_details(&mut state, &firmware_exposure_json(1, 201, 32, false));
        assert_eq!(state.snapshot.exposure.unwrap().id, 0);
        assert!(
            state
                .snapshot
                .exposure_error
                .as_ref()
                .unwrap()
                .message
                .contains("do not match")
        );
    }

    #[test]
    fn repeated_host_acknowledgment_cannot_change_its_settings() {
        let shared = exposure_test_state();
        let mut state = lock_state(&shared);
        apply_exposure_details(
            &mut state,
            &serde_json::json!({"exposure": exposure_json(0, 500, 16)}),
        );
        assert_eq!(
            state.snapshot.exposure.unwrap().settings.exposure_lines,
            400
        );
        assert!(
            state
                .snapshot
                .exposure_error
                .as_ref()
                .unwrap()
                .message
                .contains("do not match")
        );
    }

    #[test]
    fn firmware_ev_control_sends_signed_quarter_stops() {
        let mut controller = CaptureController::new();
        controller
            .start_command(
                python(
                    r#"
import json, sys
def emit(ev=0, enabled=True):
    exposure = dict(id=0, exposure_lines=400, gain_code=16, line_time_us=9.1,
                    min_lines=1, max_lines=885, min_gain_code=16, max_gain_code=255)
    firmware = dict(exposure_lines=400, gain_code=16, enabled=enabled,
                    ev_compensation=ev, p90=[180,190], heartbeat=1)
    print(json.dumps(dict(state='running', message='ready',
                         details=dict(exposure=exposure, firmware_exposure=firmware))), flush=True)
emit()
command = json.loads(sys.stdin.readline())
assert command == dict(command='auto_exposure', id=0, enabled=False, ev_quarters=-3)
emit(-0.75, False)
assert sys.stdin.readline().strip() == 'stop'
print(json.dumps(dict(state='stopped', message='both stopped')), flush=True)
"#,
                ),
                Duration::from_secs(1),
            )
            .unwrap();
        wait_for(&controller, |snapshot| snapshot.exposure.is_some());
        assert!(controller.set_auto_exposure(true, -9).is_err());
        assert!(controller.set_auto_exposure(true, 9).is_err());
        controller.set_auto_exposure(false, -3).unwrap();
        let status = wait_for(&controller, |snapshot| {
            snapshot
                .details
                .as_ref()
                .is_some_and(|d| d["firmware_exposure"]["ev_compensation"] == -0.75)
        });
        assert!(status.exposure_error.is_none());
        controller.stop().unwrap();
        wait_for(&controller, |snapshot| !snapshot.owned);
    }

    fn python(script: &str) -> Command {
        let mut command = Command::new("python3");
        command.args(["-u", "-c", script]);
        command
    }

    fn wait_for(
        controller: &CaptureController,
        predicate: impl Fn(&CaptureSnapshot) -> bool,
    ) -> CaptureSnapshot {
        let deadline = Instant::now() + Duration::from_secs(3);
        loop {
            let snapshot = controller.snapshot();
            if predicate(&snapshot) {
                return snapshot;
            }
            assert!(
                Instant::now() < deadline,
                "capture status did not arrive: {snapshot:?}"
            );
            thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn helper_arguments_use_auto_destination_or_explicit_interface() {
        let host = Ipv4Addr::new(192, 0, 2, 50);
        let auto = helper_command(host, Ipv4Addr::UNSPECIFIED, [5000, 5001]).unwrap();
        let args: Vec<_> = auto.get_args().map(|arg| arg.to_str().unwrap()).collect();
        assert_eq!(
            args,
            [
                "-u",
                "-m",
                "tools.lutcam.live",
                "--host",
                "192.0.2.50",
                "--port0",
                "5000",
                "--port1",
                "5001"
            ]
        );
        let explicit = helper_command(host, Ipv4Addr::new(192, 0, 2, 1), [5000, 5001]).unwrap();
        assert!(explicit.get_args().any(|arg| arg == "--destination"));
        assert!(helper_command(host, Ipv4Addr::LOCALHOST, [5000, 5001]).is_err());
    }

    #[test]
    fn oversized_lines_are_bounded_and_following_status_survives() {
        let mut bytes = vec![b'x'; MAX_LINE_BYTES * 5];
        bytes.extend_from_slice(
            b"\n{\"state\":\"running\",\"message\":\"ready\",\"details\":{\"fps\":120}}\n",
        );
        let mut reader = BufReader::new(bytes.as_slice());
        let (line, truncated) = read_bounded_line(&mut reader).unwrap().unwrap();
        assert_eq!(line.len(), MAX_LINE_BYTES);
        assert!(truncated);
        let (line, truncated) = read_bounded_line(&mut reader).unwrap().unwrap();
        assert!(!truncated);
        let (state, message, details) = parse_status(&line).unwrap();
        assert_eq!(state, CaptureState::Running);
        assert_eq!(message, "ready");
        assert_eq!(details.unwrap()["fps"], 120);
    }

    #[test]
    fn exposure_protocol_acks_one_request_and_retains_error_until_newer_success() {
        let mut controller = CaptureController::new();
        assert!(controller.set_exposure(100, 16).is_err());
        controller
            .start_command(
                python(
                    r#"
import json, sys, time
exposure = dict(id=0, exposure_lines=400, gain_code=16, line_time_us=9.1,
                min_lines=1, max_lines=885, min_gain_code=16, max_gain_code=255)
def emit(error=None):
    details = dict(exposure=exposure.copy())
    if error is not None: details['exposure_error'] = error
    print(json.dumps(dict(state='running', message='streaming', details=details)), flush=True)
emit()
for line in sys.stdin:
    if line.strip() == 'stop':
        print(json.dumps(dict(state='stopped', message='both stopped')), flush=True)
        break
    command = json.loads(line)
    assert command['command'] == 'exposure'
    emit()  # repeated old applied status must not acknowledge the new request
    time.sleep(.06)
    if command['id'] == 2:
        emit(dict(id=2, message='simulated sensor write failure'))
        emit()
    else:
        exposure.update({key: command[key] for key in ('id','exposure_lines','gain_code')})
        emit()
"#,
                ),
                Duration::from_secs(1),
            )
            .unwrap();
        wait_for(&controller, |snapshot| snapshot.exposure.is_some());
        assert!(controller.set_exposure(0, 16).is_err());
        assert!(controller.set_exposure(886, 16).is_err());
        assert!(controller.set_exposure(400, 256).is_err());
        let before = Instant::now();
        assert_eq!(controller.set_exposure(200, 32).unwrap(), 1);
        assert!(before.elapsed() < Duration::from_millis(100));
        assert!(controller.set_exposure(300, 48).is_err());
        let pending = controller.snapshot();
        assert_eq!(pending.exposure_pending, Some(1));
        assert_eq!(pending.exposure.unwrap().settings.exposure_lines, 400);
        wait_for(&controller, |snapshot| {
            snapshot.exposure.is_some_and(|exposure| exposure.id == 1)
        });
        assert_eq!(controller.set_exposure(300, 48).unwrap(), 2);
        let error = wait_for(&controller, |snapshot| {
            snapshot
                .exposure_error
                .as_ref()
                .is_some_and(|error| error.id == 2)
        });
        assert_eq!(error.state, CaptureState::Running);
        assert_eq!(error.exposure_pending, None);
        assert_eq!(error.exposure.unwrap().id, 1);
        thread::sleep(Duration::from_millis(30));
        assert_eq!(controller.snapshot().exposure_error.unwrap().id, 2);
        assert_eq!(controller.set_exposure(500, 64).unwrap(), 3);
        let recovered = wait_for(&controller, |snapshot| {
            snapshot.exposure.is_some_and(|exposure| exposure.id == 3)
        });
        assert!(recovered.exposure_error.is_none());
        let generation = lock_state(&controller.shared).generation;
        handle_stdout(
            &controller.shared,
            generation,
            serde_json::json!({"state":"running","details":{
            "exposure_error":{"id":2,"message":"late old error"}}})
            .to_string(),
        );
        assert!(controller.snapshot().exposure_error.is_none());
        controller.stop().unwrap();
        assert!(controller.set_exposure(300, 16).is_err());
        wait_for(&controller, |snapshot| !snapshot.owned);
    }

    #[test]
    fn exposure_limits_unknown_ack_and_mismatched_settings_are_rejected() {
        for (key, value) in [
            ("line_time_us", serde_json::json!(0)),
            ("line_time_us", serde_json::json!("NaN")),
            ("min_lines", serde_json::json!(900)),
            ("gain_code", serde_json::json!(256)),
            ("max_lines", serde_json::json!(-1)),
        ] {
            let mut status = exposure_json(0, 400, 16);
            status[key] = value;
            assert!(parse_exposure(&status).is_err());
        }
        let shared = exposure_test_state();
        let mut state = lock_state(&shared);
        let request = ExposureRequest {
            id: 1,
            settings: ExposureSettings {
                exposure_lines: 200,
                gain_code: 32,
            },
            queued_at: Instant::now(),
        };
        state.next_exposure_id = 2;
        state.last_exposure_request = Some(request);
        state.snapshot.exposure_pending = Some(1);
        apply_exposure_details(
            &mut state,
            &serde_json::json!({"exposure":exposure_json(99,200,32)}),
        );
        assert_eq!(state.snapshot.exposure.unwrap().id, 0);
        assert_eq!(state.snapshot.exposure_pending, Some(1));
        apply_exposure_details(
            &mut state,
            &serde_json::json!({"exposure":exposure_json(1,201,32)}),
        );
        assert_eq!(state.snapshot.exposure.unwrap().id, 0);
        assert_eq!(state.snapshot.exposure_pending, None);
        assert!(
            state
                .snapshot
                .exposure_error
                .as_ref()
                .unwrap()
                .message
                .contains("do not match")
        );
        state.next_exposure_id = 3;
        state.last_exposure_request = Some(ExposureRequest {
            id: 2,
            settings: ExposureSettings {
                exposure_lines: 300,
                gain_code: 48,
            },
            queued_at: Instant::now(),
        });
        state.snapshot.exposure_pending = Some(2);
        apply_exposure_details(
            &mut state,
            &serde_json::json!({"exposure":exposure_json(1,201,32)}),
        );
        assert_eq!(state.snapshot.exposure.unwrap().id, 0);
        assert_eq!(state.snapshot.exposure_pending, Some(2));
    }

    #[test]
    fn exposure_timeout_is_nonfatal_and_stop_cancels_queued_write() {
        let shared = exposure_test_state();
        let request = ExposureRequest {
            id: 1,
            settings: ExposureSettings {
                exposure_lines: 200,
                gain_code: 32,
            },
            queued_at: Instant::now() - EXPOSURE_TIMEOUT,
        };
        {
            let mut state = lock_state(&shared);
            state.next_exposure_id = 2;
            state.last_exposure_request = Some(request);
            state.snapshot.exposure_pending = Some(1);
        }
        expire_exposure(&shared, 0);
        {
            let state = lock_state(&shared);
            assert_eq!(state.snapshot.state, CaptureState::Running);
            assert_eq!(state.snapshot.exposure_pending, None);
            assert!(
                state
                    .snapshot
                    .exposure_error
                    .as_ref()
                    .unwrap()
                    .message
                    .contains("timed out")
            );
        }
        {
            let mut state = lock_state(&shared);
            state.snapshot.exposure_pending = Some(1);
            state.stop_requested = true;
            state.snapshot.state = CaptureState::Stopping;
        }
        let mut sent = Vec::new();
        assert!(!write_exposure(&mut sent, &shared, 0, request).unwrap());
        assert!(sent.is_empty());
        handle_stdout(
            &shared,
            0,
            serde_json::json!({"state":"running", "details":{
            "exposure":exposure_json(1,200,32)}})
            .to_string(),
        );
        assert_eq!(lock_state(&shared).snapshot.state, CaptureState::Stopping);
        assert_eq!(lock_state(&shared).snapshot.exposure.unwrap().id, 0);
    }

    #[test]
    fn exposure_pipe_failure_is_nonfatal_and_clears_the_pending_request() {
        let mut controller = CaptureController::new();
        controller.start_command(python(r#"
import json, os, time
os.close(0)
exposure = dict(id=0, exposure_lines=400, gain_code=16, line_time_us=9.1,
                min_lines=1, max_lines=885, min_gain_code=16, max_gain_code=255)
print(json.dumps(dict(state='running', message='ready', details=dict(exposure=exposure))), flush=True)
time.sleep(30)
"#), Duration::from_millis(100)).unwrap();
        wait_for(&controller, |snapshot| snapshot.exposure.is_some());
        assert_eq!(controller.set_exposure(200, 32).unwrap(), 1);
        let status = wait_for(&controller, |snapshot| snapshot.exposure_error.is_some());
        assert_eq!(status.state, CaptureState::Running);
        assert!(status.owned);
        assert_eq!(status.exposure_pending, None);
        assert!(
            status
                .exposure_error
                .unwrap()
                .message
                .contains("Could not send exposure")
        );
        controller.stop().unwrap();
        wait_for(&controller, |snapshot| !snapshot.owned);
    }

    #[test]
    fn immediate_stop_is_latched_and_waits_for_actual_process_exit() {
        let mut controller = CaptureController::new();
        controller
            .start_command(
                python(
                    r#"
import json, sys, time
time.sleep(.05)
print(json.dumps({'state': 'running', 'message': 'late startup'}), flush=True)
assert sys.stdin.readline().strip() == 'stop'
print(json.dumps({'state': 'stopped', 'message': 'cleaning up'}), flush=True)
time.sleep(.15)
"#,
                ),
                Duration::from_secs(1),
            )
            .unwrap();
        controller.stop().unwrap();
        let terminal = wait_for(&controller, |snapshot| {
            snapshot.state == CaptureState::Stopped
        });
        assert!(
            terminal.owned,
            "terminal JSON must not release ownership before exit"
        );
        assert!(
            controller
                .start_command(python("pass"), Duration::from_secs(1))
                .is_err()
        );
        let final_status = wait_for(&controller, |snapshot| !snapshot.owned);
        assert_eq!(final_status.state, CaptureState::Stopped);
    }

    #[test]
    fn stderr_flood_does_not_block_stop_and_logs_remain_bounded() {
        let mut controller = CaptureController::new();
        controller
            .start_command(
                python(
                    r#"
import json, sys
sys.stderr.write(('diagnostic' * 1000 + '\n') * 100)
sys.stderr.flush()
print(json.dumps({'state': 'running', 'message': 'ready'}), flush=True)
assert sys.stdin.readline().strip() == 'stop'
print(json.dumps({'state': 'stopped', 'message': 'done'}), flush=True)
"#,
                ),
                Duration::from_secs(1),
            )
            .unwrap();
        wait_for(&controller, |snapshot| {
            snapshot.state == CaptureState::Running
        });
        let before = Instant::now();
        controller.stop().unwrap();
        assert!(before.elapsed() < Duration::from_millis(100));
        let status = wait_for(&controller, |snapshot| !snapshot.owned);
        assert_eq!(status.state, CaptureState::Stopped);
        assert_eq!(status.stderr_tail.len(), MAX_LOG_LINES);
    }

    #[test]
    fn uncooperative_helper_is_killed_and_reaped_with_uncertain_shutdown_status() {
        let mut controller = CaptureController::new();
        controller
            .start_command(
                python("import time; time.sleep(30)"),
                Duration::from_millis(100),
            )
            .unwrap();
        controller.stop().unwrap();
        let status = wait_for(&controller, |snapshot| !snapshot.owned);
        assert_eq!(status.state, CaptureState::Error);
        assert!(status.message.contains("shutdown is unconfirmed"));
    }

    #[test]
    fn zero_exit_after_stop_does_not_claim_unconfirmed_camera_shutdown() {
        let mut controller = CaptureController::new();
        controller
            .start_command(
                python("import sys; sys.stdin.readline()"),
                Duration::from_secs(1),
            )
            .unwrap();
        controller.stop().unwrap();
        let status = wait_for(&controller, |snapshot| !snapshot.owned);
        assert_eq!(status.state, CaptureState::Error);
        assert!(
            status
                .message
                .contains("without confirming camera shutdown")
        );
    }

    #[test]
    fn dropping_running_controller_requests_and_waits_for_graceful_stop() {
        let mut controller = CaptureController::new();
        controller
            .start_command(
                python(
                    r#"
import json, sys
print(json.dumps({'state':'running', 'message':'ready'}), flush=True)
assert sys.stdin.readline().strip() == 'stop'
print(json.dumps({'state':'stopped', 'message':'both cameras stopped'}), flush=True)
"#,
                ),
                Duration::from_secs(1),
            )
            .unwrap();
        wait_for(&controller, |snapshot| {
            snapshot.state == CaptureState::Running
        });
        let shared = Arc::clone(&controller.shared);
        let before = Instant::now();
        drop(controller);
        assert!(before.elapsed() < Duration::from_secs(1));
        let snapshot = &lock_state(&shared).snapshot;
        assert_eq!(snapshot.state, CaptureState::Stopped);
        assert!(!snapshot.owned);
    }

    #[test]
    fn unexpected_exit_is_an_error_and_stale_generation_cannot_change_new_run() {
        let mut controller = CaptureController::new();
        controller
            .start_command(python("pass"), Duration::from_secs(1))
            .unwrap();
        let status = wait_for(&controller, |snapshot| !snapshot.owned);
        assert_eq!(status.state, CaptureState::Error);
        let old_generation = lock_state(&controller.shared).generation;
        while !controller.worker.as_ref().unwrap().thread.is_finished() {
            thread::sleep(Duration::from_millis(1));
        }
        controller
            .start_command(
                python(
                    r#"
import json, sys
print(json.dumps({'state':'running', 'message':'new run'}), flush=True)
sys.stdin.readline()
print(json.dumps({'state':'stopped', 'message':'done'}), flush=True)
"#,
                ),
                Duration::from_secs(1),
            )
            .unwrap();
        wait_for(&controller, |snapshot| {
            snapshot.state == CaptureState::Running
        });
        handle_stdout(
            &controller.shared,
            old_generation,
            r#"{"state":"error","message":"old run"}"#.to_owned(),
        );
        assert_eq!(controller.snapshot().state, CaptureState::Running);
        controller.stop().unwrap();
        wait_for(&controller, |snapshot| !snapshot.owned);
    }
}
