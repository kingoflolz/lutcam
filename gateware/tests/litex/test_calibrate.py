"""Seeded raw-centroid stereo calibration, including metric scale recovery."""

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from tools.lutcam.calibrate import (CalibrationError, calibrate, canonical_points,
                                   load_observations, main, project, _pairs,
                                   _check_coverage, _initial_pose,
                                   _undistort_points)


def synthetic_observations(count=45, *, noise=0.10, outliers=True, stationary=False,
                           constant_orientation=False):
    rng = np.random.default_rng(9281)
    cameras = np.array([[915, 931, 635, 406, -.19, .065, .0015, -.0009],
                       [944, 924, 647, 394, -.16, .045, -.0011, .0017]])
    rotation = Rotation.from_rotvec([.009, -.013, .007]).as_matrix()
    translation = np.array([-120, 1.8, .6])
    translation *= 120 / np.linalg.norm(translation)
    if constant_orientation:
        rotation, translation = np.eye(3), np.array([-120., 0., 0.])
    width = 90.0
    observations = []
    for index in range(count):
        tag_id = (123, 4567, 13901)[index % 3]
        world = np.column_stack((canonical_points(tag_id) * width / 4, np.zeros(9)))
        if stationary:
            angles, center, depth = [0.2, -0.2, 0.1], [720, 390], 700
        else:
            angles = rng.uniform([-.55, -.55, -.6], [.55, .55, .6])
            center = rng.uniform([350, 125], [1110, 675])
            depth = rng.uniform(500, 1000)
        if constant_orientation:
            angles = [0, 0, 0]
        pose_rotation = Rotation.from_rotvec(angles).as_matrix()
        pose_translation = np.array([(center[0] - cameras[0, 2]) / cameras[0, 0] * depth,
                                      (center[1] - cameras[0, 3]) / cameras[0, 1] * depth, depth])
        left = world @ pose_rotation.T + pose_translation
        points = [left, left @ rotation.T + translation]
        for camera in (0, 1):
            measured = project(points[camera], cameras[camera]) + rng.normal(0, noise, (9, 2))
            if outliers and (index * 2 + camera) % 11 == 3:
                measured[(index + camera) % 9] += [8, -9]
            if np.any(measured < 0) or np.any(measured >= [1280, 800]):
                raise AssertionError("synthetic fixture escaped image")
            observations.append({"camera": camera, "frame_id": 1000 + index * 10,
                                 "tag_id": tag_id, "points": measured.tolist(), "score": .005})
    return observations, {"cameras": cameras, "rotation": rotation,
                          "translation": translation, "width": width}


class CalibrationTests(unittest.TestCase):
    def test_zero_distortion_preserves_pixel_coordinates_exactly(self):
        intrinsics = np.array([915., 931., 635., 406., 0., 0., 0., 0.])
        observed = np.array([[0., 0.], [1279., 799.], [635., 406.],
                             [15.125, 782.75], [1243.875, 11.625]])
        ideal = _undistort_points(observed, intrinsics, time.monotonic() + 5)
        np.testing.assert_array_equal(ideal, observed)

    def test_strong_barrel_and_tangential_distortion_round_trip(self):
        # Include sensor corners: a raw-pixel homography cannot represent this
        # nonlinear mapping even though the planar target geometry is exact.
        intrinsics = np.array([1010., 997., 675., 435., -.52, .26, -.012, .008])
        ideal = np.array([[12., 11.], [1260., 18.], [15., 780.], [1250., 785.],
                          [675., 435.], [170., 430.], [1200., 440.],
                          [680., 40.], [660., 760.]])
        rays = np.column_stack(((ideal - intrinsics[2:4]) / intrinsics[:2],
                                np.ones(len(ideal))))
        observed = project(rays, intrinsics)
        self.assertGreater(np.max(np.linalg.norm(observed - ideal, axis=1)), 100)
        recovered = _undistort_points(observed, intrinsics, time.monotonic() + 5)
        np.testing.assert_allclose(recovered, ideal, atol=1e-6, rtol=0)
        recovered_rays = np.column_stack(((recovered - intrinsics[2:4]) / intrinsics[:2],
                                          np.ones(len(recovered))))
        np.testing.assert_allclose(project(recovered_rays, intrinsics), observed,
                                   atol=1e-6, rtol=0)

    def test_source_only_distorted_edge_homography_recovers_planar_pose(self):
        world = np.column_stack((canonical_points(4567) / 4, np.zeros(9)))
        intrinsics = np.array([920., 935., 642., 403., -.52, .26, .012, -.008])
        rotation = Rotation.from_rotvec([.35, -.28, .16]).as_matrix()
        translation = np.array([3.8, 1.75, 5.7])
        observed = project(world @ rotation.T + translation, intrinsics)
        self.assertGreater(observed[:, 0].min(), 1050)
        self.assertTrue(np.all((observed >= 0) & (observed < [1280, 800])))
        # No stereo geometry, other-camera measurements, or nonlinear pose
        # refinement is supplied to obtain this initialization.
        pose = _initial_pose(world, observed, intrinsics, time.monotonic() + 5)
        recovered_rotation = Rotation.from_rotvec(pose[:3]).as_matrix()
        np.testing.assert_allclose(recovered_rotation, rotation, atol=1e-6, rtol=0)
        np.testing.assert_allclose(pose[3:], translation, atol=1e-6, rtol=0)
        np.testing.assert_allclose(project(world @ recovered_rotation.T + pose[3:], intrinsics),
                                   observed, atol=1e-5, rtol=0)

    def test_distortion_inversion_respects_validation_deadline(self):
        for distortion in ((-.52, .26, -.012, .008), (0., 0., 0., 0.)):
            intrinsics = np.array([1010., 997., 675., 435., *distortion])
            with self.subTest(distortion=distortion), self.assertRaisesRegex(CalibrationError, "time budget"):
                _undistort_points(np.array([[100., 100.]]), intrinsics,
                                  time.monotonic() - 1)

    def test_distortion_inverse_rejects_a_folded_outer_branch(self):
        intrinsics = np.array([920., 935., 642., 403., -1., 0., 0., 0.])
        # x -> x(1-x^2) reaches only 2/(3sqrt(3)) on its physical
        # orientation-preserving branch. x_distorted=.6 cannot invert there.
        # The distant negative root has a positive determinant but reverses
        # both local axes, so checking determinant alone is insufficient.
        observed = np.array([[642. + .6 * 920., 403.]])
        with self.assertRaises(CalibrationError):
            _undistort_points(observed, intrinsics, time.monotonic() + 5)

    def test_canonical_geometry_matches_id_bits_and_outer_width_contract(self):
        points = canonical_points(0b10_01_11)
        np.testing.assert_array_equal(points[:2], [[-1, -1], [1, -1]])
        np.testing.assert_allclose(points[2], [1/12, -1+1/12])
        np.testing.assert_allclose(points[3], [-1-1/12, 1/12])
        np.testing.assert_allclose(points[4], [1/12, -1/12])
        self.assertEqual(np.linalg.norm((points[1] - points[0]) / 4), .5)

    def test_seeded_distorted_stereo_recovers_intrinsics_baseline_and_tag_width(self):
        observations, truth = synthetic_observations()
        # One plausible-looking but mismatched stereo observation must be
        # discarded as a pair, rather than biasing the fitted camera model.
        observations[11]["points"] = (np.array(observations[11]["points"]) + [25, -20]).tolist()
        result = calibrate(observations, max_pairs=45, timeout_seconds=90)
        self.assertTrue(result["validated"])
        self.assertEqual(result["schema_version"], 1)
        self.assertEqual(result["image_size"], [1280, 800])
        for camera, expected in zip(result["cameras"], truth["cameras"]):
            np.testing.assert_allclose([camera["fx"], camera["fy"]], expected[:2], rtol=.025)
            np.testing.assert_allclose([camera["cx"], camera["cy"]], expected[2:4], atol=10)
            np.testing.assert_allclose(camera["distortion"][:2], expected[4:6], atol=.06)
            np.testing.assert_allclose(camera["distortion"][2:4], expected[6:8], atol=.006)
            self.assertEqual(camera["distortion"][4], 0)
        recovered_rotation = np.array(result["stereo"]["rotation"])
        difference = Rotation.from_matrix(recovered_rotation @ truth["rotation"].T).magnitude()
        self.assertLess(difference, np.deg2rad(.6))
        np.testing.assert_allclose(result["stereo"]["translation_mm"], truth["translation"], atol=1.5)
        self.assertAlmostEqual(np.linalg.norm(result["stereo"]["translation_mm"]), 120, places=9)
        self.assertLess(abs(result["tag_width_mm"] - truth["width"]), 1.8)
        self.assertLess(result["metrics"]["training_rms_px"], .3)
        self.assertLess(result["metrics"]["holdout_cross_camera_rms_px"], .7)
        self.assertGreater(result["metrics"]["rejected_point_fraction"], 0)
        self.assertEqual(result["metrics"]["rejected_training_pairs"], 1)
        self.assertGreater(result["metrics"]["holdout_pairs"], 5)

    def test_inadequate_no_overlap_and_stationary_sequences_are_rejected(self):
        moving, _ = synthetic_observations(20, outliers=False)
        still, _ = synthetic_observations(20, stationary=True, outliers=False)
        for name, observations in (("inadequate", moving[:12]),
                                   ("no overlap", [item for item in moving if item["camera"] == 0]),
                                   ("stationary", still)):
            with self.subTest(name=name), self.assertRaises(CalibrationError):
                calibrate(observations)

    def test_translation_without_tilt_diversity_is_rejected(self):
        observations, _ = synthetic_observations(25, outliers=False)
        # Homographies whose only changes are translation/scale provide no
        # independent plane orientations for a reliable intrinsic solution.
        template = canonical_points(123) * 45
        for index, item in enumerate(observations):
            item["tag_id"] = 123
            item["points"] = (template + [250 + (index // 2 % 5) * 180 - item["camera"] * 100,
                                          120 + (index // 10) * 140]).tolist()
        with self.assertRaisesRegex(CalibrationError, "tilt diversity"):
            calibrate(observations)

    def test_distorted_frontoparallel_motion_does_not_fake_pose_diversity(self):
        observations, _ = synthetic_observations(35, noise=.1, outliers=False,
                                                 constant_orientation=True)
        with self.assertRaises(CalibrationError):
            calibrate(observations, max_pairs=35, timeout_seconds=60)

    def test_central_patch_cannot_validate_full_image_distortion(self):
        observations, _ = synthetic_observations(20, outliers=False)
        for item in observations:
            item["points"] = ((np.asarray(item["points"]) - [639.5, 399.5]) * .4
                               + [639.5, 399.5]).tolist()
        with self.assertRaisesRegex(CalibrationError, "coverage"):
            calibrate(observations)

    def test_many_tag_sampling_keeps_rare_edge_observations_and_distinct_frames(self):
        records = []
        edges = ([220, 120], [220, 680], [1080, 120], [1080, 680],
                 [220, 400], [1080, 400], [640, 100], [640, 700])
        uniform_indices = set(np.linspace(0, 22 * 100 - 1, 60, dtype=int))
        for frame in range(22):
            centers = [[640 + tag % 10, 400 + tag // 10] for tag in range(100)]
            edge_tags = [tag for tag in range(80, 100)
                         if frame * 100 + tag not in uniform_indices][:len(edges)]
            for tag_id, center in zip(edge_tags, edges):
                centers[tag_id] = center
            for tag_id, center in enumerate(centers):
                for camera in (0, 1):
                    points = canonical_points(tag_id) * 25 + center - np.array([camera * 80, 0])
                    records.append({"camera": camera, "frame_id": frame, "tag_id": tag_id,
                                    "points": points.tolist(), "score": .005})
        all_pairs, all_points, _, _ = _pairs(records, 20, 3000)
        # Uniform offsets in frame/tag order discard the rare edge evidence.
        old_subset = all_points[np.linspace(0, len(all_pairs) - 1, 60, dtype=int)]
        with self.assertRaisesRegex(CalibrationError, "coverage"):
            _check_coverage(old_subset)
        selected, points, _, metrics = _pairs(records, 20, 60)
        self.assertEqual(len(selected), 60)
        self.assertGreaterEqual(metrics["sampled_frames"], 20)
        _check_coverage(points[np.arange(len(points)) % 5 != 2])
        repeat, _, _, _ = _pairs(list(reversed(records)), 20, 60)
        self.assertEqual([key for key, _ in selected], [key for key, _ in repeat])

    def test_coverage_rejection_identifies_the_missing_extent(self):
        observations, _ = synthetic_observations(20, stationary=True, outliers=False)
        with self.assertRaisesRegex(CalibrationError, r"horizontal center span .* < 448 px"):
            _pairs(observations, 20, 60)

    def test_conflicting_duplicates_and_bad_canonical_order_are_rejected(self):
        observations, _ = synthetic_observations(20, outliers=False)
        conflict = dict(observations[0])
        conflict["points"] = (np.array(conflict["points"]) + .1).tolist()
        with self.assertRaisesRegex(CalibrationError, "conflicting duplicate"):
            calibrate(observations + [conflict])
        observations[0]["canonical_points"] = [[0, 0]] * 9
        with self.assertRaisesRegex(CalibrationError, "canonical order"):
            calibrate(observations)

    def test_jsonl_errors_and_failure_do_not_overwrite_an_existing_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "observations.jsonl"
            output = Path(directory) / "calibration.json"
            path.write_text("{bad\n")
            output.write_text("previous calibration\n")
            with self.assertRaisesRegex(CalibrationError, "line 1"):
                load_observations(path)
            stream = io.StringIO()
            with redirect_stdout(stream):
                self.assertEqual(main(["--observations", str(path), "--output", str(output)]), 2)
            self.assertFalse(json.loads(stream.getvalue())["ok"])
            self.assertEqual(output.read_text(), "previous calibration\n")

    def test_cli_writes_only_a_validated_solver_result(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "observations.jsonl"
            output = Path(directory) / "calibration.json"
            path.write_text("")
            result = {"schema_version": 1, "validated": True, "metrics": {"training_rms_px": .1}}
            with patch("tools.lutcam.calibrate.calibrate", return_value=result), redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--observations", str(path), "--output", str(output)]), 0)
            self.assertEqual(json.loads(output.read_text()), result)


if __name__ == "__main__":
    unittest.main()
