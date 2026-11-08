"""Stereo calibration from measured LFTag centroids in raw source pixels.

All printed tags must have the same unknown outer width. Nine ID-specific
centroids are reconstructed from the LFTag geometry; their pitch is one quarter
of that width. The known camera baseline supplies metric scale. No FPGA pose or
previous camera calibration is used as an observation, and this program never
changes hardware. Python needs NumPy and SciPy in the selected Python environment.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation


IMAGE_SIZE = (1280, 800)
DEFAULT_FOCAL = 2.8 / 0.003
GLOBAL_PARAMETERS = 22


class CalibrationError(ValueError):
    """The observations cannot support a sufficiently reliable calibration."""


def canonical_points(tag_id):
    """LFTag order: fixed anchors, then ascending two-bit data fields."""
    if type(tag_id) is not int or not 0 <= tag_id < 16384:
        raise CalibrationError("tag_id must be an unsigned fourteen-bit integer")
    points = np.array([[-1, -1], [1, -1], [0, -1], [-1, 0], [0, 0],
                       [1, 0], [-1, 1], [0, 1], [1, 1]], dtype=float)
    for index in range(7):
        symbol = (tag_id >> (2 * index)) & 3
        points[index + 2] += np.array([1 if symbol & 2 else -1,
                                        1 if symbol & 1 else -1]) / 12
    return points


def load_observations(path):
    records = []
    with Path(path).open() as stream:
        for line_number, line in enumerate(stream, 1):
            if line.strip():
                try:
                    records.append(json.loads(line))
                except (ValueError, TypeError) as error:
                    raise CalibrationError(f"invalid JSON on line {line_number}: {error}") from error
    return records


def _pairs(records, min_pairs, max_pairs):
    observations = {}
    rejected_score = 0
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            raise CalibrationError(f"observation {index} is not an object")
        camera, frame_id, tag_id = (record.get(name) for name in ("camera", "frame_id", "tag_id"))
        if (type(camera) is not int or camera not in (0, 1)
                or type(frame_id) is not int or not 0 <= frame_id <= 0xffffffff):
            raise CalibrationError(f"observation {index} needs camera 0/1 and uint32 frame_id")
        expected = canonical_points(tag_id)
        try:
            points = np.asarray(record.get("points"), dtype=float)
            score = float(record.get("score"))
        except (ValueError, TypeError) as error:
            raise CalibrationError(f"invalid points/score in observation {index}") from error
        if points.shape != (9, 2) or not np.isfinite(points).all() or not math.isfinite(score):
            raise CalibrationError(f"observation {index} needs nine finite raw-pixel points and score")
        if np.any(points < 0) or np.any(points >= np.array(IMAGE_SIZE)):
            raise CalibrationError(f"observation {index} points are outside the 1280x800 source image")
        if "canonical_points" in record:
            given = np.asarray(record["canonical_points"], dtype=float)
            if given.shape != (9, 2) or not np.allclose(given, expected, atol=1e-7, rtol=0):
                raise CalibrationError(f"observation {index} canonical order does not match tag_id")
        # Score is a canonical decode residual, not a pixel-variance estimate.
        if score < 0 or score > 0.08:
            rejected_score += 1
            continue
        key = (frame_id, tag_id)
        previous = observations.setdefault(key, {}).get(camera)
        if previous is not None and not np.array_equal(previous, points):
            raise CalibrationError(f"conflicting duplicate camera/frame/tag observation {camera}/{key}")
        observations[key][camera] = points
    pairs = [(key, np.stack([views[0], views[1]]))
             for key, views in sorted(observations.items()) if len(views) == 2]
    if len(pairs) < min_pairs or len({key[0] for key, _ in pairs}) < min_pairs:
        raise CalibrationError(f"need at least {min_pairs} shared stereo frames; found {len(pairs)} matched frame/tag pairs")
    selected = _sample_pairs(pairs, max_pairs, min_pairs)
    points = np.stack([pair[1] for pair in selected])
    _check_coverage(points)
    world = np.stack([np.column_stack((canonical_points(key[1]) / 4, np.zeros(9)))
                      for key, _ in selected])
    return selected, points, world, {"matched_pairs": len(pairs), "sampled_pairs": len(selected),
                                    "sampled_frames": len({key[0] for key, _ in selected}),
                                    "pair_selection": "stereo_spatial_scale_temporal_diversity",
                                    "rejected_decode_scores": rejected_score}


def _sample_pairs(pairs, maximum, minimum_frames):
    """Bound fitting cost without losing rare edge/scale observations to tag order."""
    if len(pairs) <= maximum:
        return pairs
    image = np.stack([points for _, points in pairs])
    centers = image.mean(axis=2) / np.array(IMAGE_SIZE)
    size = np.linalg.norm(image[:, :, 1] - image[:, :, 0], axis=2)
    _, frame_indices = np.unique([key[0] for key, _ in pairs], return_inverse=True)
    time_coordinate = frame_indices / max(1, int(frame_indices.max()))
    # Both camera positions matter: an edge in one view can be central in the
    # other. Log size retains near/far views, and time discourages selecting
    # only different tags from one frame of a many-tag scene.
    features = np.column_stack((centers.reshape(-1, 4),
                                np.log(np.maximum(size, 1)) / 3,
                                time_coordinate * .3))
    distance = np.sum((features - features.mean(axis=0)) ** 2, axis=1)
    selected, frames = [], set()
    for _ in range(maximum):
        candidates = distance.copy()
        if maximum - len(selected) <= minimum_frames - len(frames):
            candidates[np.isin(frame_indices, list(frames))] = -1
        index = int(np.argmax(candidates))
        selected.append(index)
        frames.add(int(frame_indices[index]))
        separation = np.sum((features - features[index]) ** 2, axis=1)
        distance = separation if len(selected) == 1 else np.minimum(distance, separation)
        distance[selected] = -1
    # Preserve temporal ordering for the independent holdout split.
    return [pairs[index] for index in sorted(selected)]


def _check_coverage(points):
    for camera in (0, 1):
        observed = points[:, camera]
        centers = observed.mean(axis=1)
        span = np.ptp(centers, axis=0)
        radius = np.linalg.norm((observed - [639.5, 399.5]) / DEFAULT_FOCAL, axis=2)
        missing = []
        for axis, extent, required in zip(("horizontal", "vertical"), span,
                                           np.array(IMAGE_SIZE) * .35):
            if extent < required:
                missing.append(f"{axis} center span {extent:.0f} < {required:.0f} px")
        inner, outer = np.quantile(radius, [.1, .95])
        if outer < .40:
            missing.append(f"outer radial coverage {outer * DEFAULT_FOCAL:.0f} < {.40 * DEFAULT_FOCAL:.0f} px")
        if inner > .25:
            missing.append(f"inner radial coverage {inner * DEFAULT_FOCAL:.0f} > {.25 * DEFAULT_FOCAL:.0f} px")
        if missing:
            raise CalibrationError(f"camera {camera}: insufficient image/radial coverage ({'; '.join(missing)}); cover the center and edges along both axes")
        if np.median(np.linalg.norm(observed[:, 1] - observed[:, 0], axis=1)) < 20:
            raise CalibrationError(f"camera {camera}: tags are too small for reliable calibration; move them closer")


def _homography(world, image):
    def normalize(points):
        center = points.mean(axis=0)
        scale = math.sqrt(2) / max(np.linalg.norm(points - center, axis=1).mean(), 1e-9)
        transform = np.array([[scale, 0, -scale * center[0]],
                              [0, scale, -scale * center[1]], [0, 0, 1]])
        return (points - center) * scale, transform

    def fit(indices):
        xy, tx = normalize(world[indices])
        uv, tu = normalize(image[indices])
        rows = []
        for (x, y), (u, v) in zip(xy, uv):
            rows.extend([[-x, -y, -1, 0, 0, 0, u*x, u*y, u],
                         [0, 0, 0, -x, -y, -1, v*x, v*y, v]])
        _, _, vh = np.linalg.svd(rows, full_matrices=True)
        matrix = np.linalg.inv(tu) @ vh[-1].reshape(3, 3) @ tx
        if abs(matrix[2, 2]) < 1e-10 or abs(np.linalg.det(matrix)) < 1e-10:
            raise np.linalg.LinAlgError("degenerate homography")
        return matrix / matrix[2, 2]

    xy = np.column_stack((world, np.ones(len(world))))
    rng = np.random.default_rng(81)
    best = None
    for indices in [np.arange(len(world)), *[rng.choice(len(world), 4, replace=False) for _ in range(32)]]:
        try:
            matrix = fit(indices)
            projected = xy @ matrix.T
            error = np.linalg.norm(projected[:, :2] / projected[:, 2:] - image, axis=1)
            cost = np.median(error) + 0.05 * np.minimum(error, 3).sum()
            if np.isfinite(cost) and (best is None or cost < best[0]):
                best = cost, matrix, error
        except np.linalg.LinAlgError:
            pass
    if best is None:
        raise CalibrationError("degenerate tag point geometry")
    inliers = best[2] <= max(2.0, 3 * np.median(best[2]))
    return fit(np.flatnonzero(inliers)) if inliers.sum() >= 5 else best[1]


def _zhang(homographies, nominal):
    def v(matrix, i, j):
        a, b = matrix[:, i], matrix[:, j]
        return [a[0]*b[0], a[0]*b[1]+a[1]*b[0], a[1]*b[1],
                a[2]*b[0]+a[0]*b[2], a[2]*b[1]+a[1]*b[2], a[2]*b[2]]
    rows = []
    for matrix in homographies:
        rows.extend([v(matrix, 0, 1), np.array(v(matrix, 0, 0)) - v(matrix, 1, 1)])
    try:
        _, _, vh = np.linalg.svd(rows, full_matrices=False)
        b11, b12, b22, b13, b23, b33 = vh[-1]
        if b11 < 0:
            b11, b12, b22, b13, b23, b33 = -vh[-1]
        with np.errstate(invalid="raise", divide="raise"):
            cy = (b12*b13 - b11*b23) / (b11*b22 - b12*b12)
            lam = b33 - (b13*b13 + cy*(b12*b13 - b11*b23)) / b11
            fx = np.sqrt(lam / b11)
            fy = np.sqrt(lam*b11 / (b11*b22 - b12*b12))
            skew = -b12*fx*fx*fy / lam
            cx = skew*cy/fy - b13*fx*fx/lam
        result = np.array([fx, fy, cx, cy, 0, 0, 0, 0])
        if (np.isfinite(result).all() and np.all(result[:2] > nominal * 0.7)
                and np.all(result[:2] < nominal * 1.3) and abs(skew) < nominal * 0.03
                and abs(cx - 639.5) < 100 and abs(cy - 399.5) < 80):
            return result
    except (ValueError, np.linalg.LinAlgError, ZeroDivisionError, FloatingPointError):
        pass
    return np.array([nominal, nominal, 639.5, 399.5, 0, 0, 0, 0])


def _pose_from_homography(matrix, intrinsics):
    fx, fy, cx, cy = intrinsics[:4]
    normalized = np.linalg.solve([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], matrix)
    if normalized[2, 2] < 0:
        normalized = -normalized
    scale = 2 / (np.linalg.norm(normalized[:, 0]) + np.linalg.norm(normalized[:, 1]))
    first, second = normalized[:, 0] * scale, normalized[:, 1] * scale
    rotation = np.column_stack((first, second, np.cross(first, second)))
    u, _, vh = np.linalg.svd(rotation)
    rotation = u @ np.diag([1, 1, np.linalg.det(u @ vh)]) @ vh
    return rotation, normalized[:, 2] * scale


def project(points, intrinsics):
    """Brown-Conrady projection; k3 is fixed at zero."""
    fx, fy, cx, cy, k1, k2, p1, p2 = intrinsics
    xy = points[..., :2] / np.maximum(points[..., 2:], 0.02)
    x, y = xy[..., 0], xy[..., 1]
    radius = x*x + y*y
    radial = 1 + k1*radius + k2*radius*radius
    u = x*radial + 2*p1*x*y + p2*(radius + 2*x*x)
    v = y*radial + p1*(radius + 2*y*y) + 2*p2*x*y
    return np.stack((fx*u + cx, fy*v + cy), axis=-1)


def _undistort_points(observed, intrinsics, deadline):
    """Invert the fitted lens model for pose initialization, preserving pixels."""
    if time.monotonic() > deadline:
        raise CalibrationError("calibration validation exceeded its time budget")
    observed, intrinsics = np.asarray(observed, dtype=float), np.asarray(intrinsics, dtype=float)
    if (observed.ndim != 2 or observed.shape[1] != 2 or not np.isfinite(observed).all()
            or intrinsics.shape != (8,) or not np.isfinite(intrinsics).all()
            or np.any(intrinsics[:2] <= 0)):
        raise CalibrationError("invalid source-camera distortion inversion inputs")
    if not np.any(intrinsics[4:]):
        return observed.copy()
    normalized = (observed - intrinsics[2:4]) / intrinsics[:2]

    def residual(coordinates):
        if time.monotonic() > deadline:
            raise CalibrationError("calibration validation exceeded its time budget")
        points = np.column_stack((coordinates.reshape(-1, 2), np.ones(len(observed))))
        return ((project(points, intrinsics) - observed) / intrinsics[:2]).ravel()

    # Nine points make this bounded inverse inexpensive. A numerical solve is
    # more reliable than fixed-point iteration for strong barrel distortion.
    inverse = least_squares(residual, normalized.ravel(), max_nfev=100,
                            ftol=1e-12, xtol=1e-12, gtol=1e-12)
    error = residual(inverse.x).reshape(-1, 2) * intrinsics[:2]
    if not inverse.success or not np.isfinite(inverse.x).all() or np.max(np.abs(error)) > 1e-5:
        raise CalibrationError("held-out source-camera distortion inversion did not converge")
    xy = inverse.x.reshape(-1, 2)
    x, y = xy.T
    k1, k2, p1, p2 = intrinsics[4:]
    radius = x*x + y*y
    radial, gradient = 1 + k1*radius + k2*radius*radius, 2*(k1 + 2*k2*radius)
    xx = radial + x*x*gradient + 2*p1*y + 6*p2*x
    yy = radial + y*y*gradient + 6*p1*y + 2*p2*x
    xy_derivative = x*y*gradient + 2*p1*x + 2*p2*y
    if np.any(xx + yy <= 0) or np.any(xx*yy - xy_derivative**2 <= 1e-8):
        raise CalibrationError("held-out source-camera distortion inversion reached a folded mapping")
    return xy * intrinsics[:2] + intrinsics[2:4]


def _initial_pose(world, observed, intrinsics, deadline):
    ideal_pixels = _undistort_points(observed, intrinsics, deadline)
    rotation, translation = _pose_from_homography(
        _homography(world[:, :2], ideal_pixels), intrinsics)
    return np.r_[Rotation.from_matrix(rotation).as_rotvec(), translation]


def _direction_basis(direction):
    direction = direction / np.linalg.norm(direction)
    axis = np.eye(3)[np.argmin(np.abs(direction))]
    first = np.cross(direction, axis)
    first /= np.linalg.norm(first)
    return np.column_stack((direction, first, np.cross(direction, first)))


def _unpack(parameters, basis, baseline_mm):
    intrinsics = parameters[:16].reshape(2, 8)
    stereo_r = Rotation.from_rotvec(parameters[16:19]).as_matrix()
    direction = basis @ np.r_[1.0, parameters[19:21]]
    direction /= np.linalg.norm(direction)
    width = np.exp(parameters[21])
    stereo_t = (baseline_mm / width) * direction
    poses = parameters[GLOBAL_PARAMETERS:].reshape(-1, 6)
    return intrinsics, stereo_r, stereo_t, width, poses


def _prediction(parameters, world, basis, baseline_mm):
    intrinsics, stereo_r, stereo_t, _, poses = _unpack(parameters, basis, baseline_mm)
    rotations = Rotation.from_rotvec(poses[:, :3]).as_matrix()
    left = np.einsum("nij,nkj->nki", rotations, world) + poses[:, None, 3:]
    right = left @ stereo_r.T + stereo_t
    predicted = np.stack((project(left, intrinsics[0]), project(right, intrinsics[1])), axis=1)
    depths = np.stack((left[..., 2], right[..., 2]), axis=1)
    return predicted, depths


def _initialize(world, image, baseline_mm, nominal):
    homographies = [[_homography(points[:, :2], observed[camera])
                     for points, observed in zip(world, image)] for camera in (0, 1)]
    intrinsics = np.stack([_zhang(matrices, nominal) for matrices in homographies])
    poses = [[_pose_from_homography(matrix, intrinsics[camera])
              for matrix in homographies[camera]] for camera in (0, 1)]
    rotations = np.stack([right[0] @ left[0].T for left, right in zip(*poses)])
    stereo_r = Rotation.from_matrix(rotations).mean().as_matrix()
    translations = np.stack([right[1] - stereo_r @ left[1] for left, right in zip(*poses)])
    stereo_t = np.median(translations, axis=0)
    if not 0.02 < np.linalg.norm(stereo_t) < 100:
        raise CalibrationError("stereo overlap does not provide a usable baseline/scale initialization")
    width = baseline_mm / np.linalg.norm(stereo_t)
    if not 5 < width < 2000:
        raise CalibrationError("inferred common tag width is implausible")
    basis = _direction_basis(stereo_t)
    per_pose = np.array([np.r_[Rotation.from_matrix(rotation).as_rotvec(), translation]
                         for rotation, translation in poses[0]])
    _check_tilts(np.stack([pose[0] for pose in poses[0]]))
    return np.r_[intrinsics.ravel(), Rotation.from_matrix(stereo_r).as_rotvec(),
                 0, 0, np.log(width), per_pose.ravel()], basis


def _check_tilts(rotations):
    normals = rotations[:, :, 2]
    spread = np.degrees(np.arccos(np.clip(normals @ normals.T, -1, 1))).max()
    if spread < 12 or np.linalg.svd(normals[:, :2] - normals[:, :2].mean(axis=0),
                                  compute_uv=False)[1] / math.sqrt(len(normals)) < 0.025:
        raise CalibrationError("insufficient tilt diversity; tilt the tag around both axes, not only translate it")


def _bounds(count):
    lower_camera = [300, 300, 320, 200, -1, -1, -0.1, -0.1]
    upper_camera = [2500, 2500, 960, 600, 1, 1, 0.1, 0.1]
    lower = lower_camera * 2 + [-math.pi] * 3 + [-1, -1, np.log(5)]
    upper = upper_camera * 2 + [math.pi] * 3 + [1, 1, np.log(2000)]
    return (np.array(lower + [-math.pi, -math.pi, -math.pi, -100, -100, 0.3] * count),
            np.array(upper + [math.pi, math.pi, math.pi, 100, 100, 200] * count))


def _bundle(initial, world, image, basis, baseline_mm, deadline, mask=None, *, dense=False):
    count = len(world)
    mask = np.ones((count, 2, 9), dtype=bool) if mask is None else mask
    sparsity = lil_matrix((count * 36 + count * 2, GLOBAL_PARAMETERS + count * 6), dtype=int)
    for index in range(count):
        rows = list(range(index * 36, (index + 1) * 36)) + [count * 36 + index * 2, count * 36 + index * 2 + 1]
        sparsity[index*36:index*36+18, :8] = 1
        sparsity[index*36+18:(index+1)*36, 8:GLOBAL_PARAMETERS] = 1
        sparsity[count*36+index*2+1, 16:GLOBAL_PARAMETERS] = 1
        sparsity[rows, GLOBAL_PARAMETERS + index * 6:GLOBAL_PARAMETERS + (index + 1) * 6] = 1

    def residual(parameters):
        if time.monotonic() > deadline:
            raise CalibrationError("calibration fit exceeded its time budget; collect fewer, more varied frames")
        predicted, depths = _prediction(parameters, world, basis, baseline_mm)
        errors = (predicted - image) * mask[..., None]
        return np.r_[errors.ravel(), (1000 * np.maximum(0.03 - depths.min(axis=2), 0)).ravel()]

    bounds = _bounds(count)
    initial = np.clip(initial, bounds[0] + 1e-7, bounds[1] - 1e-7)
    # Sparse steps cheaply find the basin. A small dense final solve resolves
    # the coupled global parameters more accurately than an inexact LSMR step.
    return least_squares(residual, initial, bounds=bounds, method="trf", loss="soft_l1",
                         f_scale=0.6, x_scale="jac", jac_sparsity=None if dense else sparsity.tocsr(),
                         max_nfev=80 if dense else 200, ftol=2e-7, xtol=2e-7, gtol=2e-7)


def _pose_fit(world, observed, intrinsics, initial, deadline):
    def residual(pose):
        if time.monotonic() > deadline:
            raise CalibrationError("calibration validation exceeded its time budget")
        points = world @ Rotation.from_rotvec(pose[:3]).as_matrix().T + pose[3:]
        return (project(points, intrinsics) - observed).ravel()
    options = dict(loss="soft_l1", f_scale=0.6, max_nfev=80, x_scale="jac")
    fit = least_squares(residual, initial, **options)
    if fit.status == 0:
        # Small planar targets can stall with an overly cautious trust region.
        # Restart it once at the best source-only pose, retaining convergence
        # tolerances, the shared deadline, and every stereo quality check.
        fit = least_squares(residual, fit.x, **options)
    if not fit.success:
        raise CalibrationError("held-out monocular pose did not converge")
    return fit.x


def _uncertainty(fit, point_errors, mask, basis, baseline_mm):
    """Eliminate pose nuisance parameters before testing global observability."""
    jacobian = fit.jac.toarray() if hasattr(fit.jac, "toarray") else fit.jac
    count = len(point_errors)
    information = np.zeros((GLOBAL_PARAMETERS, GLOBAL_PARAMETERS))
    for index in range(count):
        rows = list(range(index*36, (index+1)*36)) + [count*36+index*2, count*36+index*2+1]
        global_block = jacobian[rows, :GLOBAL_PARAMETERS]
        local = jacobian[rows, GLOBAL_PARAMETERS+index*6:GLOBAL_PARAMETERS+(index+1)*6]
        vectors, values, _ = np.linalg.svd(local, full_matrices=False)
        vectors = vectors[:, values > max(values[0] * 1e-10, 1e-12)]
        projected = global_block - vectors @ (vectors.T @ global_block)
        information += projected.T @ projected
    scale = np.sqrt(np.maximum(np.diag(information), 0))
    if np.any(scale < 1e-8):
        raise CalibrationError("calibration has unobservable global parameters")
    normalized = information / np.outer(scale, scale)
    values, vectors = np.linalg.eigh((normalized + normalized.T) / 2)
    if values[0] <= 1e-9:
        raise CalibrationError("calibration is degenerate after removing individual tag poses")
    degrees = max(1, int(mask.sum()) * 2 - len(fit.x))
    variance = float(np.sum(point_errors[mask] ** 2) / degrees)
    covariance = ((vectors / values) @ vectors.T) / np.outer(scale, scale) * variance
    sigma = np.sqrt(np.maximum(np.diag(covariance), 0))
    for camera, offset in enumerate((0, 8)):
        uncertain = []
        for name, value, limit, unit in (
                ("focal length", np.max(sigma[offset:offset+2] / fit.x[offset:offset+2]) * 100, 5, "%"),
                ("principal point", np.max(sigma[offset+2:offset+4]), 20, " px"),
                ("k1", sigma[offset+4], .15, ""),
                ("k2", sigma[offset+5], .35, ""),
                ("tangential distortion", np.max(sigma[offset+6:offset+8]), .02, "")):
            if value > limit:
                uncertain.append(f"{name} standard error {value:.3g}{unit} > {limit:g}{unit}")
        if uncertain:
            raise CalibrationError(f"camera {camera} intrinsics/distortion are too uncertain ({'; '.join(uncertain)}); collect larger tags at varied tilts and image positions")
    if sigma[21] > .05:
        raise CalibrationError("common tag width is too uncertain for metric calibration")
    direction = basis @ np.r_[1., fit.x[19:21]]
    norm = np.linalg.norm(direction)
    direction /= norm
    derivative = baseline_mm / norm * (np.eye(3) - np.outer(direction, direction)) @ basis[:, 1:]
    translation_sigma = np.sqrt(np.maximum(np.diag(derivative @ covariance[19:21, 19:21] @ derivative.T), 0))
    if np.max(sigma[16:19]) > np.deg2rad(1) or np.max(translation_sigma) > baseline_mm * .03:
        raise CalibrationError("stereo orientation/translation is too uncertain for reliable calibration")
    return {"global_normalized_information_min_eigenvalue": float(values[0]),
            "focal_std_px": [[float(sigma[offset]), float(sigma[offset+1])] for offset in (0, 8)],
            "principal_point_std_px": [[float(sigma[offset+2]), float(sigma[offset+3])] for offset in (0, 8)],
            "distortion_std": [sigma[offset+4:offset+8].tolist() for offset in (0, 8)],
            "stereo_rotation_std_deg": np.degrees(sigma[16:19]).tolist(),
            "stereo_translation_std_mm": translation_sigma.tolist(),
            "tag_width_relative_std": float(sigma[21]),
            "uncertainty_note": "Local linearized estimate conditional on the common-width model; excludes baseline measurement error."}


def _validate_distortion(intrinsics):
    # A physically usable image mapping must not fold anywhere on the sensor.
    for camera, calibration in enumerate(intrinsics):
        fx, fy, cx, cy = calibration[:4]
        xx, yy = np.meshgrid(np.linspace(-cx/fx, (1279-cx)/fx, 17),
                             np.linspace(-cy/fy, (799-cy)/fy, 11))
        points = np.stack((xx, yy, np.ones_like(xx)), axis=-1)
        epsilon = 1e-5
        base = project(points, calibration)
        dx = (project(points + [epsilon, 0, 0], calibration) - base) / epsilon
        dy = (project(points + [0, epsilon, 0], calibration) - base) / epsilon
        determinant = dx[..., 0]*dy[..., 1] - dx[..., 1]*dy[..., 0]
        if not np.isfinite(determinant).all() or np.min(determinant) < 0.15 * fx * fy:
            raise CalibrationError(f"camera {camera} distortion folds or compresses excessively near the image edge")


def calibrate(records, *, baseline_mm=120.0, max_pairs=60, min_pairs=20,
              initial_focal_px=DEFAULT_FOCAL, timeout_seconds=90.0,
              max_reprojection_px=1.5, progress=None):
    """Fit and validate a calibration, raising instead of exporting a bad fit."""
    if (not math.isfinite(baseline_mm) or baseline_mm <= 0
            or not math.isfinite(initial_focal_px) or initial_focal_px <= 0
            or not math.isfinite(timeout_seconds) or timeout_seconds <= 0
            or not math.isfinite(max_reprojection_px) or max_reprojection_px <= 0
            or type(min_pairs) is not int or min_pairs < 12
            or type(max_pairs) is not int or not min_pairs <= max_pairs <= 120):
        raise CalibrationError("invalid calibration limits")
    start = time.monotonic()
    deadline = start + timeout_seconds
    selected, image, world, metrics = _pairs(records, min_pairs, max_pairs)
    # Hold out entire frame/tag groups; their camera1 measurements never enter
    # training or the camera0-only pose estimate used to validate stereo.
    holdout = np.arange(len(world)) % 5 == 2
    train = ~holdout
    _check_coverage(image[train])
    training_initial, basis = _initialize(world[train], image[train], baseline_mm, initial_focal_px)
    if progress:
        progress("fit", {"training_pairs": int(train.sum()), "holdout_pairs": int(holdout.sum())})
    fit = _bundle(training_initial, world[train], image[train], basis, baseline_mm, deadline)
    predicted, depths = _prediction(fit.x, world[train], basis, baseline_mm)
    errors = np.linalg.norm(predicted - image[train], axis=-1)
    threshold = max(2.5, 4 * float(np.median(errors)))
    mask = errors < threshold
    reliable_pairs = np.all(mask.sum(axis=2) >= 6, axis=1)
    rejected_pairs = int((~reliable_pairs).sum())
    if rejected_pairs:
        if reliable_pairs.mean() < .85 or reliable_pairs.sum() < 12:
            raise CalibrationError("too many mismatched stereo pairs; check decoding and synchronized frame matching")
        indices = np.flatnonzero(train)
        train[indices[~reliable_pairs]] = False
        initial_kept = np.r_[fit.x[:GLOBAL_PARAMETERS],
                              fit.x[GLOBAL_PARAMETERS:].reshape(-1, 6)[reliable_pairs].ravel()]
        mask = mask[reliable_pairs]
        _check_coverage(image[train])
    else:
        initial_kept = fit.x
    if np.mean(mask) < 0.88:
        raise CalibrationError("too many inconsistent tag centroids; check decoding and synchronized frame matching")
    if rejected_pairs or not mask.all() or not fit.success:
        fit = _bundle(initial_kept, world[train], image[train], basis, baseline_mm, deadline, mask, dense=True)
        predicted, depths = _prediction(fit.x, world[train], basis, baseline_mm)
        errors = np.linalg.norm(predicted - image[train], axis=-1)
    if not fit.success:
        raise CalibrationError("calibration did not converge; collect more varied tag poses")
    if np.min(depths) <= 0:
        raise CalibrationError("calibration placed a tag behind a camera")
    lower, upper = _bounds(int(train.sum()))
    relative_margin = np.minimum(fit.x[:GLOBAL_PARAMETERS] - lower[:GLOBAL_PARAMETERS],
                                 upper[:GLOBAL_PARAMETERS] - fit.x[:GLOBAL_PARAMETERS]) / (upper[:GLOBAL_PARAMETERS] - lower[:GLOBAL_PARAMETERS])
    if np.min(relative_margin) < 0.001:
        raise CalibrationError("calibration reached a parameter bound; data do not constrain the model reliably")
    intrinsics, stereo_r, stereo_t, width, _ = _unpack(fit.x, basis, baseline_mm)
    fitted_poses = fit.x[GLOBAL_PARAMETERS:].reshape(-1, 6)
    _check_tilts(Rotation.from_rotvec(fitted_poses[:, :3]).as_matrix())
    metrics.update(_uncertainty(fit, errors, mask, basis, baseline_mm))
    _validate_distortion(intrinsics)
    if progress:
        progress("validate", {"holdout_pairs": int(holdout.sum())})
    cross_errors = []
    for target_world, measured in zip(world[holdout], image[holdout]):
        for source in (0, 1):
            # Initialize and refine using this camera alone. The homography
            # needs ideal pixels; refinement still uses measured sensor pixels.
            initial_pose = _initial_pose(target_world, measured[source], intrinsics[source], deadline)
            pose = _pose_fit(target_world, measured[source], intrinsics[source],
                             initial_pose, deadline)
            points = target_world @ Rotation.from_rotvec(pose[:3]).as_matrix().T + pose[3:]
            other = points @ stereo_r.T + stereo_t if source == 0 else (points - stereo_t) @ stereo_r
            if np.min(points[:, 2]) <= 0 or np.min(other[:, 2]) <= 0:
                raise CalibrationError("held-out tag projects behind the other camera")
            cross_errors.extend(np.linalg.norm(project(other, intrinsics[1-source]) - measured[1-source], axis=1))
    cross_errors = np.array(cross_errors)
    cross_inliers = cross_errors < max(3.0, 3 * max_reprojection_px)
    train_rms = float(np.sqrt(np.mean(errors[mask] ** 2)))
    cross_rms = float(np.sqrt(np.mean(cross_errors[cross_inliers] ** 2))) if cross_inliers.any() else math.inf
    if train_rms > max_reprojection_px or cross_rms > max_reprojection_px or cross_inliers.mean() < 0.88:
        raise CalibrationError(f"held-out stereo validation failed (training {train_rms:.2f}px, cross-camera {cross_rms:.2f}px)")
    metrics.update(training_pairs=int(train.sum()), holdout_pairs=int(holdout.sum()),
                   rejected_training_pairs=rejected_pairs,
                   training_rms_px=train_rms, training_median_px=float(np.median(errors[mask])),
                   training_p95_px=float(np.quantile(errors[mask], 0.95)),
                   rejected_point_fraction=float(1-mask.mean()),
                   holdout_cross_camera_rms_px=cross_rms,
                   holdout_cross_camera_median_px=float(np.median(cross_errors)),
                   holdout_cross_camera_p95_px=float(np.quantile(cross_errors, 0.95)),
                   holdout_outlier_fraction=float(1-cross_inliers.mean()),
                   fit_evaluations=int(fit.nfev), elapsed_seconds=time.monotonic()-start,
                   shared_tag_ids=sorted({key[1] for key, _ in selected}))
    return {"schema_version": 1, "image_size": list(IMAGE_SIZE), "baseline_mm": baseline_mm,
            "cameras": [{"camera": camera, **dict(zip(("fx", "fy", "cx", "cy"), map(float, values[:4]))),
                         "distortion": [*map(float, values[4:]), 0.0]}
                        for camera, values in enumerate(intrinsics)],
            "stereo": {"rotation": stereo_r.tolist(), "translation_mm": (stereo_t * width).tolist(),
                       "convention": "P_camera1 = rotation @ P_camera0 + translation_mm"},
            "tag_width_mm": float(width), "metrics": metrics,
            "assumptions": ["All observed tags share one printed outer width.",
                            "Each tag ID identifies one physical target; different copies must use unique IDs.",
                            "Points are measured in raw source-image pixel-center coordinates.",
                            "Matched frame IDs share one capture epoch and simultaneous exposure timing with both camera phase offsets zero.",
                            "Zero skew; Brown-Conrady k1,k2,p1,p2 fitted, k3 fixed at zero.",
                            "The measured camera-center baseline fixes metric scale."],
            "validated": True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observations", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--baseline-mm", type=float, default=120)
    parser.add_argument("--initial-focal-px", type=float, default=DEFAULT_FOCAL)
    parser.add_argument("--max-pairs", type=int, default=60)
    parser.add_argument("--timeout-seconds", type=float, default=90)
    parser.add_argument("--max-reprojection-px", type=float, default=1.5)
    args = parser.parse_args(argv)
    try:
        result = calibrate(load_observations(args.observations), baseline_mm=args.baseline_mm,
            initial_focal_px=args.initial_focal_px, max_pairs=args.max_pairs,
            timeout_seconds=args.timeout_seconds, max_reprojection_px=args.max_reprojection_px,
            progress=lambda stage, details: print(json.dumps({"stage": stage, **details}), file=sys.stderr, flush=True))
        temporary = args.output.with_name(args.output.name + f".{os.getpid()}.tmp")
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        os.replace(temporary, args.output)
        print(json.dumps({"ok": True, "output": str(args.output), "metrics": result["metrics"]}))
        return 0
    except (CalibrationError, OSError, ValueError, np.linalg.LinAlgError) as error:
        print(json.dumps({"ok": False, "error": str(error)}))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
