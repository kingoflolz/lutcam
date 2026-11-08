//! Bounded UDP reception and image decoding, independent of the GUI thread.

use std::io;
use std::net::{Ipv4Addr, SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use socket2::{Domain, Protocol, Socket, Type};

use crate::calibration::RecordingSink;
use crate::centroids::{CentroidCache, FrameCentroids};
use crate::poses::{FrameDetections, PoseCache};
use crate::stream::{DecodedImage, EncodedImage, StreamAssembler};

const SOCKET_BUFFER_REQUEST: usize = 8 * 1024 * 1024;
const RECEIVE_TIMEOUT: Duration = Duration::from_millis(50);
const RESTART_SILENCE: Duration = Duration::from_secs(2);
const STATS_INTERVAL: Duration = Duration::from_millis(50);
const RATE_INTERVAL: Duration = Duration::from_millis(500);

#[derive(Clone, Debug)]
pub struct ReceiverConfig {
    pub bind_ip: Ipv4Addr,
    pub source_ip: Option<Ipv4Addr>,
    pub ports: [u16; 2],
}

#[derive(Debug)]
pub struct DecodedFrame {
    pub frame_id: u32,
    pub pixels: Vec<u8>,
    pub received_at: Instant,
    pub decoded_at: Instant,
    pub decode_ms: f32,
}

#[derive(Clone, Debug)]
pub struct CameraSnapshot {
    pub latest: Option<Arc<DecodedFrame>>,
    pub packets: u64,
    pub bytes: u64,
    pub decoded: u64,
    /// Encoded images replaced in the bounded decode slot or arriving too late.
    pub display_drops: u64,
    pub invalid: u64,
    pub dropped: u64,
    pub duplicates: u64,
    pub ignored_source: u64,
    /// Codec failures other than incomplete speculative image tails.
    pub decode_errors: u64,
    pub incomplete_decodes: u64,
    pub fps: f32,
    pub mib_s: f32,
    pub last_packet: Option<Instant>,
    pub error: Option<String>,
    /// The operating system's reported receive buffer, which may be clamped.
    pub socket_buffer: usize,
    pub bound_addr: SocketAddr,
    /// Accepted tag results belong only to their exact outer UDP frame ID.
    pub pose_frames: Vec<Arc<FrameDetections>>,
    pub pose_packets: u64,
    pub pose_detections: u64,
    pub pose_invalid: u64,
    pub pose_duplicates: u64,
    pub centroid_packets: u64,
    pub centroid_groups: u64,
    pub centroid_invalid: u64,
    pub centroid_duplicates: u64,
    pub centroid_frames: Vec<Arc<FrameCentroids>>,
}

struct CameraState {
    snapshot: CameraSnapshot,
    published_epoch: u64,
    decode_rate_at: Instant,
    decode_rate_count: u64,
}

struct PendingImage {
    image: EncodedImage,
    received_at: Instant,
    epoch: u64,
}

struct CameraShared {
    state: Mutex<CameraState>,
    pending: Mutex<Option<PendingImage>>,
    ready: Condvar,
    epoch: AtomicU64,
}

impl CameraShared {
    fn resync(&self) -> u64 {
        // Use the same lock order as candidate submission. Changing the epoch
        // while holding state serializes reset against decoder publication:
        // an old result is either cleared here or rejected after this returns.
        let mut pending = self.pending.lock().unwrap();
        let mut state = self.state.lock().unwrap();
        let epoch = self.epoch.fetch_add(1, Ordering::AcqRel).wrapping_add(1);
        if pending.take().is_some() {
            state.snapshot.display_drops += 1;
        }
        state.snapshot.latest = None;
        state.snapshot.pose_frames.clear();
        state.snapshot.centroid_frames.clear();
        state.snapshot.last_packet = None;
        state.snapshot.error = None;
        state.snapshot.fps = 0.0;
        state.snapshot.mib_s = 0.0;
        state.published_epoch = epoch;
        state.decode_rate_at = Instant::now();
        state.decode_rate_count = state.snapshot.decoded;
        epoch
    }
}

/// Owns two UDP workers and two codec workers. Dropping this only closes the
/// host receiver; it never changes camera or FPGA capture state.
pub struct ReceiverSet {
    cameras: [Arc<CameraShared>; 2],
    stop: Arc<AtomicBool>,
    threads: Vec<JoinHandle<()>>,
    recording: Arc<Mutex<Option<RecordingSink>>>,
}

impl ReceiverSet {
    pub fn start(config: ReceiverConfig) -> io::Result<Self> {
        // Bind both ports before spawning anything, so a busy second port does
        // not leave a half-running viewer behind.
        let sockets = [
            open_socket(config.bind_ip, config.ports[0])?,
            open_socket(config.bind_ip, config.ports[1])?,
        ];
        let mut shared = Vec::with_capacity(2);
        for socket in &sockets {
            shared.push(Arc::new(CameraShared {
                state: Mutex::new(CameraState {
                    snapshot: CameraSnapshot {
                        latest: None,
                        packets: 0,
                        bytes: 0,
                        decoded: 0,
                        display_drops: 0,
                        invalid: 0,
                        dropped: 0,
                        duplicates: 0,
                        ignored_source: 0,
                        decode_errors: 0,
                        incomplete_decodes: 0,
                        fps: 0.0,
                        mib_s: 0.0,
                        last_packet: None,
                        error: None,
                        socket_buffer: socket.recv_buffer_size()?,
                        bound_addr: socket.local_addr()?.as_socket().ok_or_else(|| {
                            io::Error::other("UDP socket did not have an IP address")
                        })?,
                        pose_frames: Vec::new(),
                        pose_packets: 0,
                        pose_detections: 0,
                        pose_invalid: 0,
                        pose_duplicates: 0,
                        centroid_packets: 0,
                        centroid_groups: 0,
                        centroid_invalid: 0,
                        centroid_duplicates: 0,
                        centroid_frames: Vec::new(),
                    },
                    published_epoch: 0,
                    decode_rate_at: Instant::now(),
                    decode_rate_count: 0,
                }),
                pending: Mutex::new(None),
                ready: Condvar::new(),
                epoch: AtomicU64::new(0),
            }));
        }
        let cameras: [Arc<CameraShared>; 2] = shared.try_into().unwrap_or_else(|_| unreachable!());
        let mut receivers = Self {
            cameras,
            stop: Arc::new(AtomicBool::new(false)),
            threads: Vec::with_capacity(4),
            recording: Arc::new(Mutex::new(None)),
        };
        for (camera, socket) in sockets.into_iter().enumerate() {
            let shared = Arc::clone(&receivers.cameras[camera]);
            let stop = Arc::clone(&receivers.stop);
            // If spawning fails, ReceiverSet's Drop joins the workers already
            // created and all remaining sockets are closed normally.
            receivers.threads.push(
                thread::Builder::new()
                    .name(format!("camera{camera}-decode"))
                    .spawn(move || decode_worker(shared, stop))?,
            );
            let shared = Arc::clone(&receivers.cameras[camera]);
            let stop = Arc::clone(&receivers.stop);
            let source_ip = config.source_ip;
            let recording = Arc::clone(&receivers.recording);
            receivers.threads.push(
                thread::Builder::new()
                    .name(format!("camera{camera}-udp"))
                    .spawn(move || {
                        receive_worker(
                            camera as u8,
                            socket.into(),
                            source_ip,
                            shared,
                            stop,
                            recording,
                        )
                    })?,
            );
        }
        Ok(receivers)
    }

    /// Forget both streams' local frame history after a quick capture restart.
    /// Cumulative counters and listening sockets are retained. This never sends
    /// a command to the cameras or FPGA. UDP workers reset their assemblers
    /// before processing the next received packet or receive timeout.
    pub fn resync(&self) {
        for camera in &self.cameras {
            camera.resync();
        }
    }

    /// Attach or detach a bounded raw UDP recording tap. Recording is offered
    /// every source-accepted datagram before parsing, independently of image
    /// decode or display rate. A full recorder queue never stalls reception.
    pub fn set_recording(&self, sink: Option<RecordingSink>) {
        *self.recording.lock().unwrap() = sink;
    }

    /// Cheap snapshots clone the pixel Arc, never the image itself.
    pub fn snapshot(&self) -> [CameraSnapshot; 2] {
        let now = Instant::now();
        std::array::from_fn(|camera| {
            let mut snapshot = self.cameras[camera].state.lock().unwrap().snapshot.clone();
            if snapshot
                .last_packet
                .is_none_or(|last| now.duration_since(last) > Duration::from_secs(1))
            {
                snapshot.mib_s = 0.0;
            }
            if snapshot
                .latest
                .as_ref()
                .is_none_or(|frame| now.duration_since(frame.decoded_at) > Duration::from_secs(1))
            {
                snapshot.fps = 0.0;
            }
            snapshot
        })
    }
}

impl Drop for ReceiverSet {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        for camera in &self.cameras {
            camera.ready.notify_all();
        }
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}

fn open_socket(bind_ip: Ipv4Addr, port: u16) -> io::Result<Socket> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.set_recv_buffer_size(SOCKET_BUFFER_REQUEST)?;
    socket.set_read_timeout(Some(RECEIVE_TIMEOUT))?;
    let address = SocketAddr::from((bind_ip, port));
    socket.bind(&address.into()).map_err(|error| {
        io::Error::new(error.kind(), format!("cannot listen on {address}: {error}"))
    })?;
    Ok(socket)
}

#[derive(Default)]
struct ReceiveCounters {
    packets: u64,
    bytes: u64,
    ignored_source: u64,
    last_packet: Option<Instant>,
    mib_s: f32,
}

fn receive_worker(
    camera: u8,
    socket: UdpSocket,
    source_ip: Option<Ipv4Addr>,
    shared: Arc<CameraShared>,
    stop: Arc<AtomicBool>,
    recording: Arc<Mutex<Option<RecordingSink>>>,
) {
    let mut assembler = StreamAssembler::new(camera, 3);
    let mut poses = PoseCache::new(camera);
    let mut centroids = CentroidCache::new(camera);
    let mut counters = ReceiveCounters::default();
    let mut buffer = vec![0; 65_536];
    let mut stats_at = Instant::now();
    let mut rate_at = stats_at;
    let mut rate_bytes = 0;
    let mut epoch = shared.epoch.load(Ordering::Acquire);
    while !stop.load(Ordering::Acquire) {
        let received = socket.recv_from(&mut buffer);
        let current_epoch = shared.epoch.load(Ordering::Acquire);
        if current_epoch != epoch {
            assembler.reset();
            poses.reset();
            centroids.reset();
            epoch = current_epoch;
            counters.last_packet = None;
            counters.mib_s = 0.0;
            rate_at = Instant::now();
            rate_bytes = counters.bytes;
        }
        let now;
        match received {
            Ok((length, source)) => {
                now = Instant::now();
                if source_ip.is_some_and(|expected| source.ip() != expected) {
                    counters.ignored_source += 1;
                } else {
                    let sink = recording.lock().unwrap().clone();
                    if let Some(sink) = sink {
                        let _ = sink.try_record(camera, now, &buffer[..length]);
                    }
                    if counters
                        .last_packet
                        .is_some_and(|last| now.duration_since(last) >= RESTART_SILENCE)
                    {
                        assembler.reset();
                        poses.reset();
                        centroids.reset();
                        epoch = shared.resync();
                    }
                    counters.packets += 1;
                    counters.bytes += length as u64;
                    counters.last_packet = Some(now);
                    if is_product(&buffer[..length], 3) {
                        let _ = poses.push(&buffer[..length], now);
                        publish_pose_stats(&shared, &poses, epoch);
                    } else if is_product(&buffer[..length], 2) {
                        let _ = centroids.push(&buffer[..length], now);
                        publish_centroid_stats(&shared, &centroids, epoch);
                    } else if let Ok(images) = assembler.push(&buffer[..length]) {
                        submit_images(images, now, epoch, &shared);
                    }
                }
            }
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                now = Instant::now();
                submit_images(assembler.poll(), now, epoch, &shared);
            }
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => {
                shared.state.lock().unwrap().snapshot.error = Some(format!("UDP receive: {error}"));
                break;
            }
        }
        if now.duration_since(rate_at) >= RATE_INTERVAL {
            counters.mib_s = (counters.bytes - rate_bytes) as f32
                / now.duration_since(rate_at).as_secs_f32()
                / (1024.0 * 1024.0);
            rate_bytes = counters.bytes;
            rate_at = now;
        }
        if now.duration_since(stats_at) >= STATS_INTERVAL {
            poses.poll(now);
            centroids.poll(now);
            update_receive_stats(&shared, &counters, &assembler, &poses, &centroids, epoch);
            stats_at = now;
        }
    }
    update_receive_stats(&shared, &counters, &assembler, &poses, &centroids, epoch);
}

/// Route recognizable auxiliary headers to their own strict validators.
/// Malformed pose/centroid data must not enter the image assembler.
fn is_product(datagram: &[u8], product: u8) -> bool {
    datagram.len() >= 8 && datagram[..4] == [0x4c, 0x55, 0x54, 0x43] && datagram[6] == product
}

fn update_pose_snapshot(snapshot: &mut CameraSnapshot, poses: &PoseCache) {
    let stats = poses.stats();
    snapshot.pose_frames = poses.frames();
    snapshot.pose_packets = stats.packets;
    snapshot.pose_detections = stats.detections;
    snapshot.pose_invalid = stats.invalid;
    snapshot.pose_duplicates = stats.duplicates;
}

fn publish_pose_stats(shared: &CameraShared, poses: &PoseCache, epoch: u64) {
    let mut state = shared.state.lock().unwrap();
    if shared.epoch.load(Ordering::Acquire) == epoch {
        update_pose_snapshot(&mut state.snapshot, poses);
    }
}

fn update_centroid_snapshot(snapshot: &mut CameraSnapshot, centroids: &CentroidCache) {
    let stats = centroids.stats();
    snapshot.centroid_frames = centroids.frames();
    snapshot.centroid_packets = stats.packets;
    snapshot.centroid_groups = stats.groups;
    snapshot.centroid_invalid = stats.invalid;
    snapshot.centroid_duplicates = stats.duplicates;
}

fn publish_centroid_stats(shared: &CameraShared, centroids: &CentroidCache, epoch: u64) {
    let mut state = shared.state.lock().unwrap();
    if shared.epoch.load(Ordering::Acquire) == epoch {
        update_centroid_snapshot(&mut state.snapshot, centroids);
    }
}

fn update_receive_stats(
    shared: &CameraShared,
    counters: &ReceiveCounters,
    assembler: &StreamAssembler,
    poses: &PoseCache,
    centroids: &CentroidCache,
    epoch: u64,
) {
    let stats = assembler.stats();
    let mut state = shared.state.lock().unwrap();
    if shared.epoch.load(Ordering::Acquire) != epoch {
        return;
    }
    let snapshot = &mut state.snapshot;
    snapshot.packets = counters.packets;
    snapshot.bytes = counters.bytes;
    snapshot.last_packet = counters.last_packet;
    snapshot.ignored_source = counters.ignored_source;
    snapshot.mib_s = counters.mib_s;
    snapshot.invalid = stats.invalid_packets + stats.conflicting_frames;
    snapshot.dropped = stats.incomplete_evictions;
    snapshot.duplicates = stats.duplicates;
    update_pose_snapshot(snapshot, poses);
    update_centroid_snapshot(snapshot, centroids);
}

/// Sequence ordering tolerates the u32 frame counter wrapping naturally.
fn newer_frame(candidate: u32, previous: u32) -> bool {
    candidate != previous && candidate.wrapping_sub(previous) < (1 << 31)
}

fn submit_images(
    images: Vec<EncodedImage>,
    received_at: Instant,
    epoch: u64,
    shared: &CameraShared,
) {
    for image in images {
        let mut pending = shared.pending.lock().unwrap();
        let mut state = shared.state.lock().unwrap();
        // A candidate can be amended when the last image packet arrives late.
        // Permit equal IDs in the pending slot, but never re-publish an image
        // that has already decoded successfully or go backwards in time.
        let already_published = state.published_epoch == epoch
            && state
                .snapshot
                .latest
                .as_ref()
                .is_some_and(|last| !newer_frame(image.frame_id, last.frame_id));
        let older_than_pending = pending.as_ref().is_some_and(|last| {
            last.epoch == epoch
                && image.frame_id != last.image.frame_id
                && !newer_frame(image.frame_id, last.image.frame_id)
        });
        if epoch != shared.epoch.load(Ordering::Acquire) || already_published || older_than_pending
        {
            state.snapshot.display_drops += 1;
            continue;
        }
        if pending.is_some() {
            state.snapshot.display_drops += 1;
        }
        *pending = Some(PendingImage {
            image,
            received_at,
            epoch,
        });
        drop(state);
        drop(pending);
        shared.ready.notify_one();
    }
}

fn decode_worker(shared: Arc<CameraShared>, stop: Arc<AtomicBool>) {
    loop {
        let pending = {
            let mut slot = shared.pending.lock().unwrap();
            while slot.is_none() && !stop.load(Ordering::Acquire) {
                slot = shared.ready.wait_timeout(slot, RECEIVE_TIMEOUT).unwrap().0;
            }
            if stop.load(Ordering::Acquire) {
                return;
            }
            slot.take().unwrap()
        };
        if pending.epoch != shared.epoch.load(Ordering::Acquire) {
            shared.state.lock().unwrap().snapshot.display_drops += 1;
            continue;
        }
        let started_at = Instant::now();
        match pending.image.decode() {
            Ok(image) => {
                publish_decoded(
                    &shared,
                    image,
                    pending.epoch,
                    pending.received_at,
                    started_at,
                );
            }
            Err(error) => {
                // An image has no last-packet marker. Its speculative tail
                // may become complete on a later datagram; this is a dropped
                // preview attempt, not a permanent receiver failure.
                let mut state = shared.state.lock().unwrap();
                if matches!(error, lwc::CodecError::BitstreamUnderflow { .. }) {
                    state.snapshot.incomplete_decodes += 1;
                } else {
                    state.snapshot.decode_errors += 1;
                }
            }
        }
    }
}

fn publish_decoded(
    shared: &CameraShared,
    image: DecodedImage,
    epoch: u64,
    received_at: Instant,
    started_at: Instant,
) {
    let decoded_at = Instant::now();
    let mut state = shared.state.lock().unwrap();
    state.snapshot.decoded += 1;
    let stale = epoch != shared.epoch.load(Ordering::Acquire)
        || (state.published_epoch == epoch
            && state
                .snapshot
                .latest
                .as_ref()
                .is_some_and(|last| !newer_frame(image.frame_id, last.frame_id)));
    if stale {
        state.snapshot.display_drops += 1;
        return;
    }
    let elapsed = decoded_at.duration_since(state.decode_rate_at);
    if elapsed >= RATE_INTERVAL {
        state.snapshot.fps =
            (state.snapshot.decoded - state.decode_rate_count) as f32 / elapsed.as_secs_f32();
        state.decode_rate_count = state.snapshot.decoded;
        state.decode_rate_at = decoded_at;
    }
    state.published_epoch = epoch;
    state.snapshot.latest = Some(Arc::new(DecodedFrame {
        frame_id: image.frame_id,
        pixels: image.pixels,
        received_at,
        decoded_at,
        decode_ms: decoded_at.duration_since(started_at).as_secs_f32() * 1000.0,
    }));
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose_datagram(camera: u8, frame_id: u32, sequence: u32, tags: &[u32]) -> Vec<u8> {
        let header = [
            0x4c55_5443,
            u32::from_be_bytes([0, camera, 3, 32]),
            frame_id,
            sequence,
            0,
            (tags.len() * 88) as u32,
            0,
            0,
        ];
        let mut packet: Vec<u8> = header.into_iter().flat_map(u32::to_be_bytes).collect();
        for tag in tags {
            let mut words = [0u32; 22];
            words[0] = *tag;
            words[1] = 0x8000_0000 | ((frame_id & 0xff) << 20);
            words[8] = 0x0000_0001;
            for index in [10, 14, 18] {
                words[index] = 1 << 29;
            }
            words[21] = 1 << 16;
            packet.extend(words.into_iter().flat_map(u32::to_be_bytes));
        }
        packet
    }

    fn centroid_datagram(camera: u8, frame_id: u32, sequence: u32, groups: usize) -> Vec<u8> {
        let header = [
            0x4c55_5443,
            u32::from_be_bytes([0, camera, 2, 32]),
            frame_id,
            sequence,
            0,
            (groups * 9 * 8) as u32,
            0,
            0,
        ];
        let mut packet: Vec<u8> = header.into_iter().flat_map(u32::to_be_bytes).collect();
        for index in 0..groups * 9 {
            let tuple = 1 | ((160 + index as u64) << 1) | ((320 + index as u64) << 17);
            packet.extend(tuple.to_be_bytes());
        }
        packet
    }

    fn send_gray_image(sender: &UdpSocket, destination: SocketAddr, camera: u8, frame_id: u32) {
        // A compact all-zero coefficient stream reconstructs a gray-128
        // image. This exercises the real codec, not a mocked decode worker.
        let bytes = [0u8; 8_000];
        for (sequence, payload) in bytes.chunks(1024).enumerate() {
            let words = [
                0x4c55_5443,
                u32::from_be_bytes([0, camera, 1, 32]),
                frame_id,
                sequence as u32,
                (sequence * 1024) as u32,
                payload.len() as u32,
                0,
                0,
            ];
            let mut datagram = Vec::with_capacity(32 + payload.len());
            for word in words {
                datagram.extend_from_slice(&word.to_be_bytes());
            }
            datagram.extend_from_slice(payload);
            sender.send_to(&datagram, destination).unwrap();
        }
    }

    fn wait_for_frame(receivers: &ReceiverSet, frame_id: u32) {
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            if receivers.snapshot().iter().all(|snapshot| {
                snapshot
                    .latest
                    .as_ref()
                    .is_some_and(|frame| frame.frame_id == frame_id)
            }) {
                return;
            }
            assert!(Instant::now() < deadline, "frame {frame_id} did not arrive");
            thread::sleep(Duration::from_millis(10));
        }
    }

    fn wait_for_poses(receivers: &ReceiverSet, predicate: impl Fn(&[CameraSnapshot; 2]) -> bool) {
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            let snapshots = receivers.snapshot();
            if predicate(&snapshots) {
                return;
            }
            assert!(
                Instant::now() < deadline,
                "pose state did not arrive: {snapshots:?}"
            );
            thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn sequence_ordering_handles_wrap_and_late_packets() {
        assert!(newer_frame(1, 0));
        assert!(newer_frame(0, u32::MAX));
        assert!(!newer_frame(42, 42));
        assert!(!newer_frame(4, 5));
        assert!(!newer_frame(u32::MAX, 0));
    }

    #[test]
    fn occupied_port_fails_before_starting_workers() {
        let occupied = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let result = ReceiverSet::start(ReceiverConfig {
            bind_ip: Ipv4Addr::LOCALHOST,
            source_ip: None,
            ports: [0, occupied.local_addr().unwrap().port()],
        });
        let error = result.err().expect("a bound port must be rejected");
        assert_eq!(error.kind(), io::ErrorKind::AddrInUse);
        assert!(error.to_string().contains("cannot listen on"));
    }

    #[test]
    fn decodes_both_streams_rejects_old_frames_and_accepts_restart_after_silence() {
        let receivers = ReceiverSet::start(ReceiverConfig {
            bind_ip: Ipv4Addr::LOCALHOST,
            source_ip: Some(Ipv4Addr::LOCALHOST),
            ports: [0, 0],
        })
        .unwrap();
        let sender = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let addresses = receivers.snapshot().map(|snapshot| snapshot.bound_addr);
        for frame_id in [100, 101] {
            for (camera, address) in addresses.iter().enumerate() {
                send_gray_image(&sender, *address, camera as u8, frame_id);
            }
        }
        wait_for_frame(&receivers, 101);
        for snapshot in receivers.snapshot() {
            let frame = snapshot.latest.unwrap();
            assert_eq!(
                frame.pixels.len(),
                crate::stream::WIDTH * crate::stream::HEIGHT
            );
            assert!(frame.pixels.iter().all(|pixel| *pixel == 128));
            assert!(frame.decoded_at >= frame.received_at);
            assert_eq!(snapshot.invalid, 0);
            assert_eq!(snapshot.decode_errors, 0);
        }
        for (camera, address) in addresses.iter().enumerate() {
            send_gray_image(&sender, *address, camera as u8, 90);
        }
        thread::sleep(Duration::from_millis(100));
        assert!(
            receivers
                .snapshot()
                .iter()
                .all(|snapshot| snapshot.latest.as_ref().unwrap().frame_id == 101)
        );

        thread::sleep(RESTART_SILENCE);
        for (camera, address) in addresses.iter().enumerate() {
            send_gray_image(&sender, *address, camera as u8, 0);
        }
        wait_for_frame(&receivers, 0);
    }

    #[test]
    fn explicit_resync_recovers_immediate_restart_and_rejects_old_decoder_results() {
        let receivers = ReceiverSet::start(ReceiverConfig {
            bind_ip: Ipv4Addr::LOCALHOST,
            source_ip: Some(Ipv4Addr::LOCALHOST),
            ports: [0, 0],
        })
        .unwrap();
        let sender = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let addresses = receivers.snapshot().map(|snapshot| snapshot.bound_addr);
        for (camera, address) in addresses.iter().enumerate() {
            send_gray_image(&sender, *address, camera as u8, 100);
        }
        wait_for_frame(&receivers, 100);
        // A fast restart is intentionally ambiguous to sequence ordering.
        // Prove it is stale before the explicit local resync.
        for (camera, address) in addresses.iter().enumerate() {
            send_gray_image(&sender, *address, camera as u8, 0);
        }
        thread::sleep(Duration::from_millis(100));
        assert!(
            receivers
                .snapshot()
                .iter()
                .all(|snapshot| snapshot.latest.as_ref().unwrap().frame_id == 100)
        );
        let old_epochs = receivers
            .cameras
            .each_ref()
            .map(|camera| camera.epoch.load(Ordering::Acquire));
        let decoded_before = receivers.snapshot().map(|snapshot| snapshot.decoded);
        for camera in &receivers.cameras {
            camera.state.lock().unwrap().snapshot.error = Some("old diagnostic".to_string());
        }
        receivers.resync();
        for (camera, snapshot) in receivers.snapshot().iter().enumerate() {
            assert!(snapshot.latest.is_none());
            assert!(snapshot.error.is_none());
            assert_eq!(snapshot.decoded, decoded_before[camera]);
        }
        // Complete a decode that started in the old epoch after resync. It
        // must not restore the old image, even with a numerically newer ID.
        for (camera, old_epoch) in old_epochs.iter().enumerate() {
            publish_decoded(
                &receivers.cameras[camera],
                DecodedImage {
                    camera: camera as u8,
                    frame_id: 101,
                    pixels: vec![128; 1],
                },
                *old_epoch,
                Instant::now(),
                Instant::now(),
            );
            assert!(receivers.snapshot()[camera].latest.is_none());
            send_gray_image(&sender, addresses[camera], camera as u8, 1);
        }
        wait_for_frame(&receivers, 1);
        assert!(
            receivers
                .snapshot()
                .iter()
                .all(|snapshot| snapshot.display_drops >= 1)
        );
    }

    #[test]
    fn shared_udp_ports_keep_pose_batches_separate_from_images_and_clear_on_resync() {
        let receivers = ReceiverSet::start(ReceiverConfig {
            bind_ip: Ipv4Addr::LOCALHOST,
            source_ip: Some(Ipv4Addr::LOCALHOST),
            ports: [0, 0],
        })
        .unwrap();
        let sender = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let addresses = receivers.snapshot().map(|snapshot| snapshot.bound_addr);
        let batch = pose_datagram(0, 100, 10, &[1, 2]);
        sender.send_to(&batch, addresses[0]).unwrap();
        sender
            .send_to(&pose_datagram(1, 100, 55, &[4]), addresses[1])
            .unwrap();
        for (camera, address) in addresses.iter().enumerate() {
            send_gray_image(&sender, *address, camera as u8, 100);
        }
        // Pose packets can arrive before or after their image and out of order.
        sender
            .send_to(&pose_datagram(0, 100, 9, &[3]), addresses[0])
            .unwrap();
        sender.send_to(&batch, addresses[0]).unwrap();
        wait_for_frame(&receivers, 100);
        wait_for_poses(&receivers, |snapshots| {
            snapshots[0].pose_detections == 3
                && snapshots[1].pose_detections == 1
                && snapshots[0].pose_duplicates == 1
        });
        let initial = receivers.snapshot();
        assert_eq!(initial[0].pose_frames[0].detections.len(), 3);
        assert_eq!(initial[1].pose_frames[0].detections.len(), 1);
        assert!(initial.iter().all(|snapshot| snapshot.invalid == 0));
        let revision = initial[0].pose_frames[0].revision;

        let mut malformed = pose_datagram(0, 101, 11, &[8]);
        malformed[32 + 8 * 4..32 + 9 * 4].fill(0);
        sender.send_to(&malformed, addresses[0]).unwrap();
        for (camera, address) in addresses.iter().enumerate() {
            send_gray_image(&sender, *address, camera as u8, 101);
        }
        wait_for_frame(&receivers, 101);
        wait_for_poses(&receivers, |snapshots| snapshots[0].pose_invalid == 1);
        assert_eq!(
            receivers.snapshot()[0].invalid,
            0,
            "invalid pose must not enter image assembly"
        );

        // A conflicting duplicate empties only its own frame's detections.
        sender
            .send_to(&pose_datagram(0, 100, 10, &[9, 2]), addresses[0])
            .unwrap();
        wait_for_poses(&receivers, |snapshots| {
            snapshots[0].pose_frames.iter().any(|frame| {
                frame.frame_id == 100
                    && frame.invalid
                    && frame.detections.is_empty()
                    && frame.revision > revision
            })
        });
        assert_eq!(
            receivers.snapshot()[0].latest.as_ref().unwrap().frame_id,
            101
        );
        receivers.resync();
        assert!(
            receivers
                .snapshot()
                .iter()
                .all(|snapshot| snapshot.pose_frames.is_empty())
        );
        sender
            .send_to(&pose_datagram(0, 0, 0, &[7]), addresses[0])
            .unwrap();
        wait_for_poses(&receivers, |snapshots| {
            snapshots[0].pose_frames.len() == 1
                && snapshots[0].pose_frames[0].frame_id == 0
                && snapshots[0].pose_frames[0].detections[0].tag_id == 7
        });
        assert!(receivers.snapshot()[1].pose_frames.is_empty());
    }

    #[test]
    fn measured_centroid_products_are_counted_without_affecting_image_assembly() {
        let receivers = ReceiverSet::start(ReceiverConfig {
            bind_ip: Ipv4Addr::LOCALHOST,
            source_ip: Some(Ipv4Addr::LOCALHOST),
            ports: [0, 0],
        })
        .unwrap();
        let sender = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let addresses = receivers.snapshot().map(|snapshot| snapshot.bound_addr);
        sender
            .send_to(&centroid_datagram(0, 100, 9, 4), addresses[0])
            .unwrap();
        sender
            .send_to(&centroid_datagram(1, 100, 55, 1), addresses[1])
            .unwrap();
        let mut malformed = centroid_datagram(0, 100, 10, 1);
        malformed[32] = 0x80; // A tuple's reserved bits must stay zero.
        sender.send_to(&malformed, addresses[0]).unwrap();
        sender
            .send_to(&centroid_datagram(1, 100, 56, 1), addresses[0])
            .unwrap();
        for (camera, address) in addresses.iter().enumerate() {
            send_gray_image(&sender, *address, camera as u8, 100);
        }
        wait_for_frame(&receivers, 100);
        wait_for_poses(&receivers, |snapshots| {
            snapshots[0].centroid_packets == 3
                && snapshots[0].centroid_groups == 4
                && snapshots[0].centroid_invalid == 2
                && snapshots[1].centroid_groups == 1
        });
        assert!(
            receivers
                .snapshot()
                .iter()
                .all(|snapshot| snapshot.invalid == 0)
        );
        let frames = receivers.snapshot();
        assert!(frames[0].centroid_frames[0].invalid);
        assert!(frames[0].centroid_frames[0].groups.is_empty());
        assert_eq!(frames[1].centroid_frames[0].groups[0].centroids.len(), 9);
        receivers.resync();
        assert!(
            receivers
                .snapshot()
                .iter()
                .all(|snapshot| snapshot.centroid_frames.is_empty())
        );
        let first = centroid_datagram(0, 0, 0, 1);
        sender.send_to(&first, addresses[0]).unwrap();
        sender
            .send_to(&centroid_datagram(0, 0, 1, 1), addresses[0])
            .unwrap();
        sender.send_to(&first, addresses[0]).unwrap();
        wait_for_poses(&receivers, |snapshots| {
            snapshots[0].centroid_duplicates == 1
                && snapshots[0]
                    .centroid_frames
                    .first()
                    .is_some_and(|frame| frame.frame_id == 0 && frame.groups.len() == 2)
        });
        let mut conflicting = first;
        conflicting[39] ^= 2;
        sender.send_to(&conflicting, addresses[0]).unwrap();
        wait_for_poses(&receivers, |snapshots| {
            snapshots[0].centroid_frames.first().is_some_and(|frame| {
                frame.frame_id == 0 && frame.invalid && frame.groups.is_empty()
            })
        });
    }

    #[test]
    fn recording_tap_preserves_original_auxiliary_and_malformed_packets_before_parsing() {
        use crate::calibration::{RECORDING_MAGIC, Recorder, RecordingStatus};
        let directory = std::env::temp_dir().join(format!(
            "lutcam-receiver-recording-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let mut recorder = Recorder::new();
        let sink = recorder
            .start(directory.clone(), serde_json::json!({"test":true}))
            .unwrap();
        let receivers = ReceiverSet::start(ReceiverConfig {
            bind_ip: Ipv4Addr::LOCALHOST,
            source_ip: Some(Ipv4Addr::LOCALHOST),
            ports: [0, 0],
        })
        .unwrap();
        receivers.set_recording(Some(sink));
        let deadline = Instant::now() + Duration::from_secs(3);
        while !recorder.snapshot().accepting {
            assert!(
                Instant::now() < deadline,
                "recording did not finish preparation"
            );
            thread::sleep(Duration::from_millis(5));
        }
        let sender = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let addresses = receivers.snapshot().map(|snapshot| snapshot.bound_addr);
        let originals = vec![
            pose_datagram(0, 100, 1, &[7]),
            centroid_datagram(0, 100, 2, 1),
            b"malformed source-accepted data".to_vec(),
        ];
        for packet in &originals {
            sender.send_to(packet, addresses[0]).unwrap();
        }
        send_gray_image(&sender, addresses[1], 1, 100);
        let ignored = UdpSocket::bind((Ipv4Addr::new(127, 0, 0, 2), 0)).unwrap();
        ignored.send_to(b"wrong source", addresses[0]).unwrap();
        wait_for_poses(&receivers, |snapshots| {
            snapshots[0].packets == 3
                && snapshots[1].packets == 8
                && snapshots[0].ignored_source == 1
        });
        receivers.set_recording(None);
        sender.send_to(b"after tap detached", addresses[0]).unwrap();
        wait_for_poses(&receivers, |snapshots| snapshots[0].packets == 4);
        recorder.cancel();
        let deadline = Instant::now() + Duration::from_secs(3);
        while recorder.snapshot().status == RecordingStatus::Recording {
            assert!(
                Instant::now() < deadline,
                "recording did not flush after cancellation"
            );
            thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(recorder.snapshot().status, RecordingStatus::Incomplete);
        let bytes = std::fs::read(directory.join("camera0.udp")).unwrap();
        assert_eq!(&bytes[..8], RECORDING_MAGIC);
        let mut cursor = 8;
        let mut recorded = Vec::new();
        while cursor < bytes.len() {
            let length =
                u32::from_le_bytes(bytes[cursor + 8..cursor + 12].try_into().unwrap()) as usize;
            cursor += 12;
            recorded.push(bytes[cursor..cursor + length].to_vec());
            cursor += length;
        }
        assert_eq!(recorded, originals);
        assert_eq!(recorder.snapshot().cameras[1].packets, 8);
        drop(recorder);
        std::fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn udp_workers_filter_sources_count_invalid_packets_and_shut_down() {
        for filter_matches in [true, false] {
            let receivers = ReceiverSet::start(ReceiverConfig {
                bind_ip: Ipv4Addr::LOCALHOST,
                source_ip: Some(if filter_matches {
                    Ipv4Addr::LOCALHOST
                } else {
                    Ipv4Addr::new(192, 0, 2, 1)
                }),
                ports: [0, 0],
            })
            .unwrap();
            let sender = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
            for snapshot in receivers.snapshot() {
                assert!(snapshot.socket_buffer > 0);
                sender
                    .send_to(b"invalid image datagram", snapshot.bound_addr)
                    .unwrap();
            }
            let deadline = Instant::now() + Duration::from_secs(2);
            loop {
                let snapshots = receivers.snapshot();
                let ready = snapshots.iter().all(|snapshot| {
                    if filter_matches {
                        snapshot.packets == 1 && snapshot.invalid == 1
                    } else {
                        snapshot.packets == 0 && snapshot.ignored_source == 1
                    }
                });
                if ready {
                    break;
                }
                assert!(
                    Instant::now() < deadline,
                    "receiver statistics did not arrive: {snapshots:?}"
                );
                thread::sleep(Duration::from_millis(10));
            }
            let close_started = Instant::now();
            let addresses = receivers.snapshot().map(|snapshot| snapshot.bound_addr);
            drop(receivers);
            assert!(close_started.elapsed() < Duration::from_secs(1));
            for address in addresses {
                let _reopened =
                    UdpSocket::bind(address).expect("Drop must release both listening ports");
            }
        }
    }
}
