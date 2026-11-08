//! Exercise the real SDK and in-process viewer channel without a GPU or camera.

use std::sync::{Arc, mpsc};
use std::thread;
use std::time::{Duration, Instant};

use lutcam_viewer::receiver::DecodedFrame;
use lutcam_viewer::rerun_feed::{LogReceiver, RerunFeed};
use re_viewer::external::re_chunk::Chunk;
use re_viewer::external::re_log_channel::{
    DataSourceMessage, RecvTimeoutError, SmartMessagePayload,
};
use re_viewer::external::re_log_types::{EntityPath, LogMsg};

fn next_chunk(receiver: &LogReceiver, deadline: Instant) -> Chunk {
    loop {
        assert!(
            Instant::now() < deadline,
            "timed out waiting for Rerun SDK data"
        );
        match receiver.recv_timeout(Duration::from_millis(20)) {
            Ok(message) => match message.payload {
                SmartMessagePayload::Msg(DataSourceMessage::LogMsg(LogMsg::ArrowMsg(_, arrow))) => {
                    return Chunk::from_arrow_msg(&arrow).expect("valid SDK Arrow chunk");
                }
                SmartMessagePayload::Flush { on_flush_done } => on_flush_done(),
                SmartMessagePayload::Quit(error) => {
                    panic!("Rerun sender quit before expected data: {error:?}");
                }
                _ => {}
            },
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => panic!("Rerun sender disconnected"),
        }
    }
}

#[test]
fn images_and_clear_reach_the_sdk_receiver_and_drop_is_prompt() {
    let (feed, receiver) = RerunFeed::new().expect("start embedded Rerun feed");
    let now = Instant::now();
    let original = (0..lwc::FRAME_WIDTH * lwc::FRAME_HEIGHT)
        .map(|index| (index % 251) as u8)
        .collect::<Vec<_>>();
    let expected = original
        .chunks_exact(lwc::FRAME_WIDTH)
        .rev()
        .flatten()
        .copied()
        .collect::<Vec<_>>();
    let frame = Arc::new(DecodedFrame {
        frame_id: 123,
        pixels: original,
        received_at: now,
        decoded_at: now,
        decode_ms: 1.5,
    });
    for camera in 0..2 {
        feed.log_latest(camera, Arc::clone(&frame)).unwrap();
    }

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut images_seen = [false; 2];
    while !images_seen.iter().all(|seen| *seen) {
        let chunk = next_chunk(&receiver, deadline);
        for (camera, seen) in images_seen.iter_mut().enumerate() {
            if chunk.entity_path() != &EntityPath::from(format!("camera{camera}/image")) {
                continue;
            }
            let buffers = chunk
                .iter_component::<rerun::components::Blob>(
                    rerun::Image::descriptor_buffer().component,
                )
                .collect::<Vec<_>>();
            // The same entity also receives the calibrated Pinhole component.
            if buffers.is_empty() {
                continue;
            }
            assert_eq!(buffers.len(), 1);
            assert_eq!(buffers[0].len(), 1);
            assert_eq!(buffers[0][0].0.0.as_ref(), expected.as_slice());
            let formats = chunk
                .iter_component::<rerun::components::ImageFormat>(
                    rerun::Image::descriptor_format().component,
                )
                .collect::<Vec<_>>();
            assert_eq!(formats.len(), 1);
            assert_eq!(formats[0].len(), 1);
            let format = &formats[0][0].0;
            assert_eq!(format.width, lwc::FRAME_WIDTH as u32);
            assert_eq!(format.height, lwc::FRAME_HEIGHT as u32);
            assert_eq!(format.color_model, Some(rerun::datatypes::ColorModel::L));
            assert_eq!(
                format.channel_datatype,
                Some(rerun::datatypes::ChannelDatatype::U8)
            );
            assert!(
                chunk
                    .timelines()
                    .keys()
                    .any(|name| name.as_str() == "capture_time")
            );
            *seen = true;
        }
    }

    feed.clear().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut clears_seen = [false; 2];
    while !clears_seen.iter().all(|seen| *seen) {
        let chunk = next_chunk(&receiver, deadline);
        for (camera, seen) in clears_seen.iter_mut().enumerate() {
            if chunk.entity_path() == &EntityPath::from(format!("camera{camera}")) {
                assert!(chunk.component_descriptors().any(|descriptor| {
                    descriptor.component == rerun::Clear::descriptor_is_recursive().component
                }));
                *seen = true;
            }
        }
    }
    assert!(feed.error().is_none());

    // The production Viewer closes its log receiver before dropping the feed.
    // Perform that order on another thread so a regression cannot hang this test.
    let (finished, done) = mpsc::channel();
    thread::spawn(move || {
        drop(receiver);
        drop(feed);
        finished.send(()).unwrap();
    });
    done.recv_timeout(Duration::from_secs(1))
        .expect("receiver/feed drop must not wait on SDK backpressure");
}

fn frame(id: u32) -> Arc<DecodedFrame> {
    let now = Instant::now();
    Arc::new(DecodedFrame {
        frame_id: id,
        pixels: vec![37; lwc::FRAME_WIDTH * lwc::FRAME_HEIGHT],
        received_at: now,
        decoded_at: now,
        decode_ms: 0.0,
    })
}

fn pose_frame(id: u32, received_at: Instant) -> lutcam_viewer::poses::FrameDetections {
    use lutcam_viewer::poses::{FrameDetections, TagDetection};
    FrameDetections {
        camera: 0,
        frame_id: id,
        revision: 1,
        received_at,
        invalid: false,
        detections: vec![TagDetection {
            sequence: 17,
            record_index: 0,
            tag_id: 123,
            common_words: [0; 8],
            scale: 0,
            frame_sequence: id as u8,
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
        }],
    }
}

fn capture_time(chunk: &Chunk) -> i64 {
    let column = chunk
        .timelines()
        .iter()
        .find(|(name, _)| name.as_str() == "capture_time")
        .unwrap()
        .1;
    *column.times_raw().last().unwrap()
}

/// SDK batching may flush different entities in any order. Keep unmatched
/// chunks so assertions depend on their timeline, not callback delivery order.
struct LoggedChunks<'a> {
    receiver: &'a LogReceiver,
    pending: Vec<Chunk>,
}

impl<'a> LoggedChunks<'a> {
    fn new(receiver: &'a LogReceiver) -> Self {
        Self {
            receiver,
            pending: Vec::new(),
        }
    }

    fn next(
        &mut self,
        path: &str,
        component: rerun::ComponentDescriptor,
        time: Option<i64>,
    ) -> Chunk {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if let Some(index) = self.pending.iter().position(|chunk| {
                chunk.entity_path() == &EntityPath::from(path)
                    && chunk
                        .component_descriptors()
                        .any(|descriptor| descriptor.component == component.component)
                    && time.is_none_or(|time| capture_time(chunk) == time)
            }) {
                return self.pending.remove(index);
            }
            let chunk = next_chunk(self.receiver, deadline);
            // One SDK chunk can contain several capture times, especially when
            // a snapshot includes late revisions alongside its newest image.
            // Match individual rows instead of silently selecting the last one.
            self.pending
                .extend((0..chunk.num_rows()).map(|row| chunk.row_sliced_shallow(row, 1)));
        }
    }
}

#[test]
fn late_pose_revisions_use_original_image_time_and_clear_before_each_image() {
    let (feed, receiver) = RerunFeed::new().unwrap();
    let mut logs = LoggedChunks::new(&receiver);
    feed.set_calibration(
        0,
        lutcam_viewer::rerun_feed::CameraCalibration {
            fx: 800.0,
            fy: 1200.0,
            cx: 600.25,
            cy: 300.75,
            tag_width: 1.0,
        },
    )
    .unwrap();
    let earlier = frame(42);
    feed.log_latest(0, Arc::clone(&earlier)).unwrap();
    let image_42 = logs.next("camera0/image", rerun::Image::descriptor_buffer(), None);
    let time_42 = capture_time(&image_42);
    let later = frame(43);
    feed.log_latest(0, Arc::clone(&later)).unwrap();
    let image_43 = logs.next("camera0/image", rerun::Image::descriptor_buffer(), None);
    let time_43 = capture_time(&image_43);
    assert!(time_43 > time_42);
    let clear = logs.next(
        "camera0/lftags",
        rerun::Clear::descriptor_is_recursive(),
        Some(time_43),
    );
    assert_eq!(capture_time(&clear), time_43);

    feed.log_detections(Arc::new(pose_frame(42, earlier.received_at)))
        .unwrap();
    let projected = logs.next(
        "camera0/image/lftags/centroids",
        rerun::Points2D::descriptor_positions(),
        Some(time_42),
    );
    assert_eq!(capture_time(&projected), time_42);
    let projected_values = projected
        .iter_component::<rerun::components::Position2D>(
            rerun::Points2D::descriptor_positions().component,
        )
        .last()
        .unwrap();
    assert_eq!(projected_values.len(), 9);
    assert_eq!(projected_values[0].x(), 550.75);
    assert_eq!(projected_values[0].y(), 573.75);
    let points = logs.next(
        "camera0/lftags/centroids",
        rerun::Points3D::descriptor_positions(),
        Some(time_42),
    );
    assert_eq!(capture_time(&points), time_42);
    let values = points
        .iter_component::<rerun::components::Position3D>(
            rerun::Points3D::descriptor_positions().component,
        )
        .collect::<Vec<_>>();
    assert_eq!(values.last().unwrap().len(), 9);
    assert_eq!(values.last().unwrap()[0].x(), -0.25);
    assert_eq!(values.last().unwrap()[0].y(), -0.25);
    assert_eq!(values.last().unwrap()[0].z(), 4.0);

    let mut invalid = pose_frame(42, earlier.received_at);
    invalid.revision = 2;
    invalid.invalid = true;
    invalid.detections.clear();
    logs.pending.clear(); // Discard earlier clears; the next ones belong to the invalid revision.
    feed.log_detections(Arc::new(invalid)).unwrap();
    let clear = logs.next(
        "camera0/lftags",
        rerun::Clear::descriptor_is_recursive(),
        Some(time_42),
    );
    assert_eq!(capture_time(&clear), time_42);
    let overlay_clear = logs.next(
        "camera0/image/lftags",
        rerun::Clear::descriptor_is_recursive(),
        Some(time_42),
    );
    assert_eq!(capture_time(&overlay_clear), time_42);
    let raw = logs.next(
        "camera0/lftags/raw",
        rerun::TextDocument::descriptor_text(),
        Some(time_42),
    );
    let text = raw
        .iter_component::<rerun::components::Text>(rerun::TextDocument::descriptor_text().component)
        .last()
        .unwrap()[0]
        .0
        .to_string();
    let metadata: serde_json::Value = serde_json::from_str(&text).unwrap();
    assert_eq!(metadata["frame_id"], 42);
    assert_eq!(metadata["invalid"], true);
    assert!(feed.error().is_none());
}

#[test]
fn conditioning_labels_use_pose_columns_and_archive_both_warning_values() {
    let (feed, receiver) = RerunFeed::new().unwrap();
    let mut logs = LoggedChunks::new(&receiver);
    for (id, raw_warning, first_column, expected_label) in [
        (51, true, 1 << 29, "123"),
        (52, false, 1 << 28, "123 conditioned"),
    ] {
        let image = frame(id);
        feed.log_latest(0, Arc::clone(&image)).unwrap();
        let image_chunk = logs.next("camera0/image", rerun::Image::descriptor_buffer(), None);
        let time = capture_time(&image_chunk);
        let mut poses = pose_frame(id, image.received_at);
        poses.detections[0].pose.conditioning_warning = raw_warning;
        poses.detections[0].pose.pose.rotation_q29[0] = first_column;
        feed.log_detections(Arc::new(poses)).unwrap();

        let outline = logs.next(
            "camera0/image/lftags/outlines",
            rerun::LineStrips2D::descriptor_labels(),
            Some(time),
        );
        let labels = outline
            .iter_component::<rerun::components::Text>(
                rerun::LineStrips2D::descriptor_labels().component,
            )
            .last()
            .unwrap();
        assert_eq!(labels[0].0.to_string(), expected_label);

        let raw = logs.next(
            "camera0/lftags/raw",
            rerun::TextDocument::descriptor_text(),
            Some(time),
        );
        let text = raw
            .iter_component::<rerun::components::Text>(
                rerun::TextDocument::descriptor_text().component,
            )
            .last()
            .unwrap()[0]
            .0
            .to_string();
        let metadata: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(
            metadata["detections"][0]["conditioning_warning"],
            raw_warning
        );
        assert_eq!(
            metadata["detections"][0]["conditioning_warning_recomputed"],
            !raw_warning,
        );
    }
    assert!(feed.error().is_none());
}

#[derive(Debug, PartialEq)]
struct TagPresentation {
    points_2d: usize,
    points_3d: usize,
    outlines_2d: Vec<rerun::components::LineStrip2D>,
    outlines_3d: Vec<rerun::components::LineStrip3D>,
    labels: Vec<String>,
    colors: Vec<rerun::components::Color>,
    show_labels: rerun::components::ShowLabels,
    raw: String,
}

fn tag_presentation(logs: &mut LoggedChunks<'_>, time: i64) -> TagPresentation {
    let points_2d = logs.next(
        "camera0/image/lftags/centroids",
        rerun::Points2D::descriptor_positions(),
        Some(time),
    );
    let points_3d = logs.next(
        "camera0/lftags/centroids",
        rerun::Points3D::descriptor_positions(),
        Some(time),
    );
    let outlines_2d = logs.next(
        "camera0/image/lftags/outlines",
        rerun::LineStrips2D::descriptor_strips(),
        Some(time),
    );
    let outlines_3d = logs.next(
        "camera0/lftags/outlines",
        rerun::LineStrips3D::descriptor_strips(),
        Some(time),
    );
    let raw = logs.next(
        "camera0/lftags/raw",
        rerun::TextDocument::descriptor_text(),
        Some(time),
    );
    let result = TagPresentation {
        points_2d: points_2d
            .iter_component::<rerun::components::Position2D>(
                rerun::Points2D::descriptor_positions().component,
            )
            .last()
            .unwrap()
            .len(),
        points_3d: points_3d
            .iter_component::<rerun::components::Position3D>(
                rerun::Points3D::descriptor_positions().component,
            )
            .last()
            .unwrap()
            .len(),
        outlines_2d: outlines_2d
            .iter_component::<rerun::components::LineStrip2D>(
                rerun::LineStrips2D::descriptor_strips().component,
            )
            .last()
            .unwrap()
            .to_vec(),
        outlines_3d: outlines_3d
            .iter_component::<rerun::components::LineStrip3D>(
                rerun::LineStrips3D::descriptor_strips().component,
            )
            .last()
            .unwrap()
            .to_vec(),
        labels: outlines_2d
            .iter_component::<rerun::components::Text>(
                rerun::LineStrips2D::descriptor_labels().component,
            )
            .last()
            .unwrap()
            .iter()
            .map(|text| text.0.to_string())
            .collect(),
        colors: outlines_2d
            .iter_component::<rerun::components::Color>(
                rerun::LineStrips2D::descriptor_colors().component,
            )
            .last()
            .unwrap()
            .to_vec(),
        show_labels: outlines_2d
            .iter_component::<rerun::components::ShowLabels>(
                rerun::LineStrips2D::descriptor_show_labels().component,
            )
            .last()
            .unwrap()[0],
        raw: raw
            .iter_component::<rerun::components::Text>(
                rerun::TextDocument::descriptor_text().component,
            )
            .last()
            .unwrap()[0]
            .0
            .to_string(),
    };
    assert_eq!(
        outlines_3d
            .iter_component::<rerun::components::Color>(
                rerun::LineStrips3D::descriptor_colors().component
            )
            .last()
            .unwrap()
            .to_vec(),
        result.colors
    );
    assert_eq!(
        outlines_3d
            .iter_component::<rerun::components::ShowLabels>(
                rerun::LineStrips3D::descriptor_show_labels().component
            )
            .last()
            .unwrap()[0],
        result.show_labels
    );
    result
}

#[test]
fn outlines_only_toggle_refreshes_current_frame_and_preserves_raw_poses() {
    let (feed, receiver) = RerunFeed::new().unwrap();
    let mut logs = LoggedChunks::new(&receiver);
    let image = frame(61);
    feed.log_latest(0, Arc::clone(&image)).unwrap();
    let time = capture_time(&logs.next("camera0/image", rerun::Image::descriptor_buffer(), None));
    let mut poses = pose_frame(61, image.received_at);
    let mut good = poses.detections[0].clone();
    good.tag_id = 124;
    good.pose.pose.translation_q16[0] = 2 << 16;
    poses.detections[0].pose.weak_perspective = true;
    poses.detections[0].pose.pose.rotation_q29[0] = 1 << 28;
    poses.detections.push(good);
    feed.log_detections(Arc::new(poses)).unwrap();
    let normal = tag_presentation(&mut logs, time);
    assert_eq!(normal.points_2d, 18);
    assert_eq!(normal.points_3d, 18);
    assert_eq!(normal.labels, ["123 weak conditioned", "124"]);
    assert_eq!(normal.show_labels, true.into());
    assert_eq!(
        normal.colors,
        [[255, 184, 64].into(), [64, 255, 160].into()]
    );

    // No new image or detection is supplied: presentation must update the
    // already logged frame, including its original capture timestamp.
    feed.set_tag_outlines_only(true).unwrap();
    let plain = tag_presentation(&mut logs, time);
    assert_eq!((plain.points_2d, plain.points_3d), (0, 0));
    assert!(plain.labels.is_empty());
    assert_eq!(plain.show_labels, false.into());
    assert_eq!(plain.colors, [[64, 255, 160].into(), [64, 255, 160].into()]);
    assert_eq!(plain.outlines_2d, normal.outlines_2d);
    assert_eq!(plain.outlines_3d, normal.outlines_3d);
    assert_eq!(plain.raw, normal.raw);
    feed.set_tag_outlines_only(false).unwrap();
    assert_eq!(tag_presentation(&mut logs, time), normal);

    // The newest image has no tags. Toggling may never resurrect the previous
    // frame's geometry; another camera's image provides a worker progress check.
    let newer = frame(62);
    feed.log_latest(0, Arc::clone(&newer)).unwrap();
    let newer_time =
        capture_time(&logs.next("camera0/image", rerun::Image::descriptor_buffer(), None));
    logs.pending.clear();
    feed.set_tag_outlines_only(true).unwrap();
    feed.log_latest(1, frame(62)).unwrap();
    logs.next("camera1/image", rerun::Image::descriptor_buffer(), None);
    let deadline = Instant::now() + Duration::from_millis(100);
    while Instant::now() < deadline {
        match receiver.recv_timeout(Duration::from_millis(10)) {
            Ok(message) => match message.payload {
                SmartMessagePayload::Msg(DataSourceMessage::LogMsg(LogMsg::ArrowMsg(_, arrow))) => {
                    logs.pending.push(Chunk::from_arrow_msg(&arrow).unwrap());
                }
                SmartMessagePayload::Flush { on_flush_done } => on_flush_done(),
                _ => {}
            },
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => panic!("Rerun sender disconnected"),
        }
    }
    assert!(!logs.pending.iter().any(|chunk| {
        chunk.entity_path() == &EntityPath::from("camera0/image/lftags/outlines")
            && chunk.component_descriptors().any(|descriptor| {
                descriptor.component == rerun::LineStrips2D::descriptor_strips().component
            })
    }));
    feed.log_detections(Arc::new(pose_frame(62, newer.received_at)))
        .unwrap();
    let next = tag_presentation(&mut logs, newer_time);
    assert_eq!((next.points_2d, next.points_3d), (0, 0));
    assert!(next.labels.is_empty());
    assert_eq!(next.outlines_2d.len(), 1);
    assert_eq!(next.show_labels, false.into());
    assert!(feed.error().is_none());
}

#[test]
fn camera_snapshot_keeps_new_epoch_poses_and_centroids_with_the_reused_image_id() {
    use lutcam_viewer::centroids::{Centroid, CentroidGroup, FrameCentroids};
    let (feed, receiver) = RerunFeed::new().unwrap();
    let mut logs = LoggedChunks::new(&receiver);
    let old = frame(71);
    let mut old_poses = pose_frame(71, old.received_at);
    old_poses.revision = 99;
    feed.log_snapshot(0, Some(Arc::clone(&old)), &[Arc::new(old_poses)], &[])
        .unwrap();
    let old_time =
        capture_time(&logs.next("camera0/image", rerun::Image::descriptor_buffer(), None));
    assert_eq!(tag_presentation(&mut logs, old_time).labels, ["123"]);

    // The full frame ID is reused after capture has stopped. Registration must
    // clear the old epoch before the same snapshot's new products are cached.
    let restarted_at = old.received_at + Duration::from_secs(3);
    let restarted = Arc::new(DecodedFrame {
        frame_id: 71,
        pixels: vec![17; lwc::FRAME_WIDTH * lwc::FRAME_HEIGHT],
        received_at: restarted_at,
        decoded_at: restarted_at,
        decode_ms: 0.0,
    });
    let mut poses = pose_frame(71, restarted_at);
    poses.detections[0].tag_id = 456;
    let centroids = Arc::new(FrameCentroids {
        camera: 0,
        frame_id: 71,
        revision: 1,
        received_at: restarted_at,
        invalid: false,
        groups: vec![CentroidGroup {
            sequence: 91,
            group_index: 0,
            centroids: vec![Centroid {
                valid: true,
                x_q5: 10 * 32,
                y_q5: 20 * 32,
            }],
        }],
    });
    feed.set_tag_outlines_only(true).unwrap();
    feed.log_snapshot(0, Some(restarted), &[Arc::new(poses)], &[centroids])
        .unwrap();
    let time = capture_time(&logs.next("camera0/image", rerun::Image::descriptor_buffer(), None));
    assert!(time > old_time);
    let presentation = tag_presentation(&mut logs, time);
    assert_eq!(presentation.outlines_2d.len(), 1);
    assert_eq!((presentation.points_2d, presentation.points_3d), (0, 0));
    assert!(presentation.labels.is_empty());
    let metadata: serde_json::Value = serde_json::from_str(&presentation.raw).unwrap();
    assert_eq!(metadata["revision"], 1);
    assert_eq!(metadata["detections"][0]["tag_id"], 456);
    let points = logs.next(
        "camera0/image/centroids",
        rerun::Points2D::descriptor_positions(),
        Some(time),
    );
    let points = points
        .iter_component::<rerun::components::Position2D>(
            rerun::Points2D::descriptor_positions().component,
        )
        .last()
        .unwrap();
    assert_eq!(points.len(), 1);
    assert_eq!([points[0].x(), points[0].y()], [10.5, 779.5]);
    assert!(feed.error().is_none());
}

#[test]
fn camera_snapshot_logs_empty_current_frame_and_late_revision_at_separate_original_times() {
    let (feed, receiver) = RerunFeed::new().unwrap();
    let mut logs = LoggedChunks::new(&receiver);
    let earlier = frame(81);
    feed.log_snapshot(
        0,
        Some(Arc::clone(&earlier)),
        &[Arc::new(pose_frame(81, earlier.received_at))],
        &[],
    )
    .unwrap();
    let earlier_time =
        capture_time(&logs.next("camera0/image", rerun::Image::descriptor_buffer(), None));
    assert_eq!(tag_presentation(&mut logs, earlier_time).labels, ["123"]);

    let newer = frame(82);
    let mut empty = pose_frame(82, newer.received_at);
    empty.detections.clear();
    let mut late = pose_frame(81, earlier.received_at);
    late.revision = 2;
    late.detections[0].tag_id = 789;
    feed.log_snapshot(0, Some(newer), &[Arc::new(late), Arc::new(empty)], &[])
        .unwrap();
    let newer_time =
        capture_time(&logs.next("camera0/image", rerun::Image::descriptor_buffer(), None));
    assert!(newer_time > earlier_time);
    let current = tag_presentation(&mut logs, newer_time);
    assert!(current.outlines_2d.is_empty());
    assert!(current.outlines_3d.is_empty());
    assert_eq!((current.points_2d, current.points_3d), (0, 0));
    let metadata: serde_json::Value = serde_json::from_str(&current.raw).unwrap();
    assert_eq!(metadata["frame_id"], 82);
    assert!(metadata["detections"].as_array().unwrap().is_empty());
    let previous = tag_presentation(&mut logs, earlier_time);
    assert_eq!(previous.labels, ["789"]);
    let metadata: serde_json::Value = serde_json::from_str(&previous.raw).unwrap();
    assert_eq!(metadata["frame_id"], 81);
    assert_eq!(metadata["revision"], 2);

    // Presentation refresh must continue to show the current empty image,
    // even though the most recently processed pose revision was for an old one.
    feed.set_tag_outlines_only(true).unwrap();
    assert!(
        tag_presentation(&mut logs, newer_time)
            .outlines_2d
            .is_empty()
    );
    assert!(feed.error().is_none());
}

#[test]
fn many_tags_share_four_geometry_entities_without_losing_labels_matrices_or_raw_records() {
    use lutcam_viewer::rerun_feed::{CameraCalibration, log_detection_frame};
    let calibration = CameraCalibration {
        fx: 850.0,
        fy: 975.0,
        ..Default::default()
    };
    let (recording, storage) = rerun::RecordingStreamBuilder::new("many-tags-parity")
        .enabled(true)
        .memory()
        .unwrap();
    for count in [1, 8, 128] {
        let mut frame = pose_frame(77, Instant::now());
        let seed = frame.detections[0].clone();
        frame.detections = (0..count)
            .map(|index| {
                let mut tag = seed.clone();
                tag.tag_id = 100 + index as u32;
                tag.sequence = index as u32;
                tag.pose.weak_perspective = index % 2 != 0;
                if index % 3 == 2 {
                    // A nonorthogonal transmitted matrix must be applied as-is,
                    // without a quaternion conversion or rotation normalization.
                    tag.pose.pose.rotation_q29[1] = 1 << 26;
                    tag.pose.pose.rotation_q29[6] = 1 << 25;
                }
                tag.pose.pose.translation_q16 = [
                    (index as i32 % 16 * 2 - 15) * 65536,
                    (index as i32 / 16 * 2 - 7) * 65536,
                    40 << 16,
                ];
                tag
            })
            .collect();
        recording.set_time("capture_time", Duration::from_secs(count as u64));
        log_detection_frame(&recording, calibration, &frame).unwrap();
        let chunks: Vec<_> = storage
            .take()
            .into_iter()
            .filter_map(|message| match message {
                LogMsg::ArrowMsg(_, arrow) => Some(Chunk::from_arrow_msg(&arrow).unwrap()),
                _ => None,
            })
            .collect();
        let find = |path: &str| {
            chunks
                .iter()
                .find(|chunk| chunk.entity_path() == &EntityPath::from(path))
                .unwrap()
        };
        let points = find("camera0/image/lftags/centroids");
        let positions = points
            .iter_component::<rerun::components::Position2D>(
                rerun::Points2D::descriptor_positions().component,
            )
            .last()
            .unwrap();
        let point_colors = points
            .iter_component::<rerun::components::Color>(
                rerun::Points2D::descriptor_colors().component,
            )
            .last()
            .unwrap();
        let outlines = find("camera0/image/lftags/outlines");
        let strips = outlines
            .iter_component::<rerun::components::LineStrip2D>(
                rerun::LineStrips2D::descriptor_strips().component,
            )
            .last()
            .unwrap();
        let labels = outlines
            .iter_component::<rerun::components::Text>(
                rerun::LineStrips2D::descriptor_labels().component,
            )
            .last()
            .unwrap();
        let outline_colors = outlines
            .iter_component::<rerun::components::Color>(
                rerun::LineStrips2D::descriptor_colors().component,
            )
            .last()
            .unwrap();
        let positions_3d = find("camera0/lftags/centroids")
            .iter_component::<rerun::components::Position3D>(
                rerun::Points3D::descriptor_positions().component,
            )
            .last()
            .unwrap();
        let outlines_3d = find("camera0/lftags/outlines")
            .iter_component::<rerun::components::LineStrip3D>(
                rerun::LineStrips3D::descriptor_strips().component,
            )
            .last()
            .unwrap();
        assert_eq!(positions.len(), count * 9);
        assert_eq!(positions_3d.len(), count * 9);
        assert_eq!(outlines_3d.len(), count);
        assert_eq!(point_colors.len(), count * 9);
        assert_eq!(strips.len(), count);
        assert_eq!(labels.len(), count);
        assert_eq!(outline_colors.len(), count);
        assert_eq!(
            chunks
                .iter()
                .filter(|chunk| {
                    let path = chunk.entity_path().to_string();
                    path.starts_with("/camera0/image/lftags/")
                })
                .count(),
            2,
        );
        assert!(
            chunks
                .iter()
                .all(|chunk| !chunk.entity_path().to_string().contains("/tag_"))
        );
        let reflections: Vec<_> = chunks
            .iter()
            .filter(|chunk| chunk.entity_path() == &EntityPath::from("camera0/lftags"))
            .flat_map(|chunk| {
                chunk.iter_component::<rerun::components::Scale3D>(
                    rerun::Transform3D::descriptor_scale().component,
                )
            })
            .flat_map(|scales| scales.iter().map(|scale| scale.0.0).collect::<Vec<_>>())
            .collect();
        assert_eq!(reflections, [[1.0, -1.0, 1.0]]);
        assert_eq!(
            find("camera0/lftags/outlines")
                .iter_component::<rerun::components::Text>(
                    rerun::LineStrips3D::descriptor_labels().component,
                )
                .last()
                .unwrap()
                .as_slice(),
            labels.as_slice(),
        );
        assert_eq!(
            find("camera0/lftags/centroids")
                .iter_component::<rerun::components::Color>(
                    rerun::Points3D::descriptor_colors().component,
                )
                .last()
                .unwrap()
                .as_slice(),
            point_colors.as_slice(),
        );
        assert_eq!(
            find("camera0/lftags/outlines")
                .iter_component::<rerun::components::Color>(
                    rerun::LineStrips3D::descriptor_colors().component,
                )
                .last()
                .unwrap()
                .as_slice(),
            outline_colors.as_slice(),
        );
        for (index, tag) in frame.detections.iter().enumerate() {
            let translation = tag
                .pose
                .pose
                .translation_q16
                .map(|value| value as f32 / 65536.0);
            let matrix = tag
                .pose
                .pose
                .rotation_q29
                .map(|value| value as f32 / (1u32 << 29) as f32);
            let transform = |[x, y]: [f32; 2]| -> [f32; 3] {
                std::array::from_fn(|row| {
                    translation[row] + matrix[row * 3] * x + matrix[row * 3 + 1] * y
                })
            };
            let project = |point: [f32; 2]| {
                let [x, y, z] = transform(point);
                [
                    calibration.fx * x / z + calibration.cx + 0.5,
                    799.5 - (calibration.fy * y / z + calibration.cy),
                ]
            };
            let conditioned = tag.pose.pose.conditioning_warning();
            let color = rerun::components::Color::from(if index % 2 != 0 || conditioned {
                [255, 184, 64]
            } else {
                [64, 255, 160]
            });
            assert_eq!(outline_colors[index], color);
            assert_eq!(
                labels[index].0.to_string(),
                format!(
                    "{}{}{}",
                    tag.tag_id,
                    if index % 2 != 0 { " weak" } else { "" },
                    if conditioned { " conditioned" } else { "" }
                ),
            );
            for (point_index, point) in lftag::canonical_points(tag.tag_id as u16)
                .iter()
                .enumerate()
            {
                let expected =
                    project([point.position.x as f32 / 4.0, point.position.y as f32 / 4.0]);
                let actual = &positions[index * 9 + point_index];
                assert!((actual.x() - expected[0]).abs() < 0.0001);
                assert!((actual.y() - expected[1]).abs() < 0.0001);
                assert_eq!(point_colors[index * 9 + point_index], color);
                let expected_3d =
                    transform([point.position.x as f32 / 4.0, point.position.y as f32 / 4.0]);
                let actual_3d = &positions_3d[index * 9 + point_index];
                for (actual, expected) in [actual_3d.x(), actual_3d.y(), actual_3d.z()]
                    .into_iter()
                    .zip(expected_3d)
                {
                    assert!((actual - expected).abs() < 0.0001);
                }
            }
            let expected_outline = [
                [-0.5, -0.5],
                [0.5, -0.5],
                [0.5, 0.5],
                [-0.5, 0.5],
                [-0.5, -0.5],
            ];
            for (actual, expected) in strips[index].0.iter().zip(expected_outline.map(project)) {
                assert!((actual.0[0] - expected[0]).abs() < 0.0001);
                assert!((actual.0[1] - expected[1]).abs() < 0.0001);
            }
            for (actual, expected) in outlines_3d[index]
                .0
                .iter()
                .zip(expected_outline.map(transform))
            {
                for (actual, expected) in actual.0.into_iter().zip(expected) {
                    assert!((actual - expected).abs() < 0.0001);
                }
            }
        }
        let raw = find("camera0/lftags/raw")
            .iter_component::<rerun::components::Text>(
                rerun::TextDocument::descriptor_text().component,
            )
            .last()
            .unwrap();
        let metadata: serde_json::Value = serde_json::from_str(&raw[0].0).unwrap();
        assert_eq!(metadata["detections"].as_array().unwrap().len(), count);
        assert_eq!(metadata["frame_id"], 77);
        for (index, tag) in frame.detections.iter().enumerate() {
            assert_eq!(
                metadata["detections"][index]["rotation_q29"],
                serde_json::json!(tag.pose.pose.rotation_q29)
            );
            assert_eq!(
                metadata["detections"][index]["translation_q16"],
                serde_json::json!(tag.pose.pose.translation_q16)
            );
        }
        assert!(
            chunks
                .iter()
                .filter(|chunk| chunk.entity_path().to_string().starts_with("/camera0/"))
                .all(|chunk| capture_time(chunk) == count as i64 * 1_000_000_000)
        );
    }
}

#[test]
fn valid_empty_and_filtered_revisions_clear_batched_geometry_at_the_original_time() {
    use lutcam_viewer::rerun_feed::{CameraCalibration, log_detection_frame};
    let (recording, storage) = rerun::RecordingStreamBuilder::new("empty-pose-revisions")
        .enabled(true)
        .memory()
        .unwrap();
    let mut frame = pose_frame(42, Instant::now());
    for revision in 1..=3 {
        frame.revision = revision;
        if revision == 2 {
            frame.detections[0].pose.saturation = true;
        } else if revision == 3 {
            frame.detections.clear();
        }
        recording.set_time("capture_time", Duration::from_secs(5));
        log_detection_frame(&recording, CameraCalibration::default(), &frame).unwrap();
        let chunks: Vec<_> = storage
            .take()
            .into_iter()
            .filter_map(|message| match message {
                LogMsg::ArrowMsg(_, arrow) => Some(Chunk::from_arrow_msg(&arrow).unwrap()),
                _ => None,
            })
            .collect();
        for root in ["camera0/lftags", "camera0/image/lftags"] {
            assert!(chunks.iter().any(|chunk| {
                chunk.entity_path() == &EntityPath::from(root)
                    && chunk.component_descriptors().any(|descriptor| {
                        descriptor.component == rerun::Clear::descriptor_is_recursive().component
                    })
            }));
            let outline = chunks
                .iter()
                .find(|chunk| chunk.entity_path() == &EntityPath::from(format!("{root}/outlines")))
                .unwrap();
            let descriptor = if root.contains("/image/") {
                rerun::LineStrips2D::descriptor_labels()
            } else {
                rerun::LineStrips3D::descriptor_labels()
            };
            let labels = outline
                .iter_component::<rerun::components::Text>(descriptor.component)
                .last()
                .unwrap();
            assert_eq!(labels.len(), usize::from(revision == 1));
            assert_eq!(capture_time(outline), 5_000_000_000);
        }
        let raw = chunks
            .iter()
            .find(|chunk| chunk.entity_path() == &EntityPath::from("camera0/lftags/raw"))
            .unwrap()
            .iter_component::<rerun::components::Text>(
                rerun::TextDocument::descriptor_text().component,
            )
            .last()
            .unwrap();
        let metadata: serde_json::Value = serde_json::from_str(&raw[0].0).unwrap();
        assert_eq!(metadata["invalid"], false);
        assert_eq!(
            metadata["detections"].as_array().unwrap().len(),
            usize::from(revision != 3)
        );
    }
}

#[test]
fn overlapping_pose_revisions_keep_the_best_fit_and_preserve_every_raw_record() {
    use lutcam_viewer::rerun_feed::{CameraCalibration, log_detection_frame};
    let (recording, storage) = rerun::RecordingStreamBuilder::new("overlap-test")
        .enabled(true)
        .memory()
        .unwrap();
    recording.set_time("capture_time", Duration::from_secs(1));
    let mut poses = pose_frame(55, Instant::now());
    poses.detections[0].scale = 2;
    poses.detections[0].pose.mean_decode_residual_q32 = 10;
    for revision in 1..=3 {
        poses.revision = revision;
        if revision == 2 {
            let mut fine = poses.detections[0].clone();
            fine.scale = 0;
            fine.tag_id = 456; // A conflicting coarse ID must not bypass suppression.
            fine.best_score_q32 = 0;
            fine.pose.mean_decode_residual_q32 = 100; // A worse fine ID must not hide the coarse fit.
            fine.sequence += 1;
            poses.detections.push(fine);
        } else if revision == 3 {
            let mut better = poses.detections[1].clone();
            better.pose.mean_decode_residual_q32 = 1;
            better.sequence += 1;
            poses.detections.push(better);
        }
        log_detection_frame(&recording, CameraCalibration::default(), &poses).unwrap();
        let chunks: Vec<_> = storage
            .take()
            .into_iter()
            .filter_map(|message| match message {
                LogMsg::ArrowMsg(_, arrow) => Some(Chunk::from_arrow_msg(&arrow).unwrap()),
                _ => None,
            })
            .collect();
        for prefix in ["camera0/image/lftags", "camera0/lftags"] {
            assert!(chunks.iter().any(|chunk| {
                chunk.entity_path() == &EntityPath::from(prefix)
                    && chunk.component_descriptors().any(|descriptor| {
                        descriptor.component == rerun::Clear::descriptor_is_recursive().component
                    })
            }));
            let outlines: Vec<_> = chunks
                .iter()
                .filter(|chunk| {
                    let path = chunk.entity_path().to_string();
                    path == format!("/{prefix}/outlines")
                })
                .collect();
            assert_eq!(outlines.len(), 1);
            let expected_path = format!("{prefix}/outlines");
            assert_eq!(outlines[0].entity_path(), &EntityPath::from(expected_path));
            let labels = outlines[0]
                .iter_component::<rerun::components::Text>(if prefix.contains("/image/") {
                    rerun::LineStrips2D::descriptor_labels().component
                } else {
                    rerun::LineStrips3D::descriptor_labels().component
                })
                .last()
                .unwrap();
            assert_eq!(labels.len(), 1);
            assert_eq!(
                labels[0].0.to_string(),
                if revision < 3 { "123" } else { "456" }
            );
            let strip_count = if prefix.contains("/image/") {
                outlines[0]
                    .iter_component::<rerun::components::LineStrip2D>(
                        rerun::LineStrips2D::descriptor_strips().component,
                    )
                    .last()
                    .unwrap()
                    .len()
            } else {
                outlines[0]
                    .iter_component::<rerun::components::LineStrip3D>(
                        rerun::LineStrips3D::descriptor_strips().component,
                    )
                    .last()
                    .unwrap()
                    .len()
            };
            assert_eq!(strip_count, 1);
        }
        let raw = chunks
            .iter()
            .find(|chunk| chunk.entity_path() == &EntityPath::from("camera0/lftags/raw"))
            .unwrap();
        let text = raw
            .iter_component::<rerun::components::Text>(
                rerun::TextDocument::descriptor_text().component,
            )
            .last()
            .unwrap()[0]
            .0
            .to_string();
        let metadata: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(
            metadata["detections"].as_array().unwrap().len(),
            revision as usize
        );
        assert_eq!(metadata["detections"][0]["scale"], 2);
        if revision >= 2 {
            assert_eq!(metadata["detections"][1]["scale"], 0);
            assert_eq!(metadata["detections"][1]["tag_id"], 456);
        }
        assert!(
            chunks
                .iter()
                .filter(|chunk| chunk.entity_path().to_string().starts_with("/camera0/"))
                .all(|chunk| capture_time(chunk) == 1_000_000_000)
        );
    }
}

#[test]
fn raw_centroid_archive_keeps_invalid_tuples_and_projects_valid_planes() {
    use lutcam_viewer::centroids::{Centroid, CentroidGroup, FrameCentroids};
    let (feed, receiver) = RerunFeed::new().unwrap();
    let mut logs = LoggedChunks::new(&receiver);
    let image = frame(19);
    let centroids = FrameCentroids {
        camera: 0,
        frame_id: 19,
        revision: 1,
        received_at: image.received_at,
        invalid: false,
        groups: vec![CentroidGroup {
            sequence: 123,
            group_index: 0,
            centroids: vec![
                Centroid {
                    valid: true,
                    x_q5: 10 * 32,
                    y_q5: 20 * 32,
                },
                Centroid {
                    valid: true,
                    x_q5: 1284 * 32,
                    y_q5: 0,
                },
                Centroid {
                    valid: true,
                    x_q5: 1612 * 32,
                    y_q5: 0,
                },
                Centroid {
                    valid: false,
                    x_q5: 65535,
                    y_q5: 65535,
                },
                Centroid {
                    valid: true,
                    x_q5: 1282 * 32,
                    y_q5: 0,
                },
            ],
        }],
    };
    // Arrival before the image must be retained until exact-frame association.
    feed.log_centroids(Arc::new(centroids)).unwrap();
    feed.log_latest(0, image).unwrap();
    let raw = logs.next(
        "camera0/image/centroids/raw",
        rerun::TextDocument::descriptor_text(),
        None,
    );
    let text = raw
        .iter_component::<rerun::components::Text>(rerun::TextDocument::descriptor_text().component)
        .last()
        .unwrap()[0]
        .0
        .to_string();
    let metadata: serde_json::Value = serde_json::from_str(&text).unwrap();
    assert_eq!(metadata["groups"][0]["sequence"], 123);
    assert_eq!(
        metadata["groups"][0]["centroids"].as_array().unwrap().len(),
        5
    );
    assert_eq!(metadata["groups"][0]["centroids"][3]["x_q5"], 65535);
    assert_eq!(metadata["groups"][0]["centroids"][3]["valid"], false);
    let points = logs.next(
        "camera0/image/centroids",
        rerun::Points2D::descriptor_positions(),
        Some(capture_time(&raw)),
    );
    let values = points
        .iter_component::<rerun::components::Position2D>(
            rerun::Points2D::descriptor_positions().component,
        )
        .collect::<Vec<_>>();
    let values = values.last().unwrap();
    assert_eq!(values.len(), 3);
    let actual: Vec<_> = values.iter().map(|point| [point.x(), point.y()]).collect();
    assert_eq!(actual, [[10.5, 779.5], [0.5, 799.5], [0.5, 795.5]]);
    assert_eq!(capture_time(&points), capture_time(&raw));
    assert!(feed.error().is_none());
}

#[test]
fn blueprint_orders_camera_zero_on_right_and_exposes_separate_layers() {
    use re_viewer::external::re_sdk_types::blueprint::archetypes::{
        ContainerBlueprint, ViewBlueprint, ViewContents,
    };
    use rerun::blueprint::components::{IncludedContent, QueryExpression};
    use std::collections::HashMap;
    let (feed, receiver) = RerunFeed::new().unwrap();
    let mut views = HashMap::new();
    let mut order = Vec::new();
    let mut queries = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(5);
    while views.len() != 2 || order.len() != 2 || queries.len() < 2 {
        let chunk = next_chunk(&receiver, deadline);
        for names in chunk.iter_component::<rerun::components::Name>(
            ViewBlueprint::descriptor_display_name().component,
        ) {
            if let Some(name) = names.first() {
                views.insert(
                    chunk
                        .entity_path()
                        .to_string()
                        .trim_start_matches('/')
                        .to_string(),
                    name.0.to_string(),
                );
            }
        }
        for contents in chunk
            .iter_component::<IncludedContent>(ContainerBlueprint::descriptor_contents().component)
        {
            order = contents
                .iter()
                .map(|path| path.0.to_string().trim_start_matches('/').to_string())
                .collect();
        }
        for contents in
            chunk.iter_component::<QueryExpression>(ViewContents::descriptor_query().component)
        {
            queries.extend(contents.iter().map(|query| query.0.to_string()));
        }
    }
    assert_eq!(
        order
            .iter()
            .map(|path| views[path].as_str())
            .collect::<Vec<_>>(),
        ["Camera 1", "Camera 0"]
    );
    assert!(
        queries
            .iter()
            .any(|query| query.contains("camera0/image/**"))
    );
    assert!(
        !queries
            .iter()
            .any(|query| query.contains("camera0/lftags/**"))
    );
    assert!(feed.error().is_none());
}
