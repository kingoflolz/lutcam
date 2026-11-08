"""Validated live-viewer pinhole calibration and LFTag stream configuration.

The pose solver consumes a pinhole model; retained lens distortion and stereo
baseline metadata are not corrections performed by the FPGA. Printed width is
always one, so translations and overlay geometry use normalized tag widths.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

IMAGE_SIZE = [1280, 800]
REGISTER_OFFSETS = {
    "inv_fx_q2_30": 0x80, "inv_fy_q2_30": 0x84,
    "cx_q16_16": 0x88, "cy_q16_16": 0x8c, "printed_width_q16_16": 0x90,
    "mean_residual_limit_q16": 0x94,
    "max_mass_spread": 0xac,
    "min_baseline_q4": 0xb0,
}
THRESHOLD_REGISTERS = {
    # N=sum(ex**2+ey**2), D=sum(W**2), over five noncorner data points.
    # Acceptance uses canonical grid-pitch squared; ranking retains raw N.
    "mean_residual_limit_q16": 1024,  # N/D <= 1024/65536 = 1/64
    "max_mass_spread": 19, "min_baseline_q4": 16,
}


def _number(value, name, *, positive=False):
    if type(value) not in (int, float):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except OverflowError as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(f"{name} must be {'positive and ' if positive else ''}finite")
    return result


def _round(value):
    """Match fixed-point conversion's nearest, ties away from zero."""
    return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)


def _camera(value, index):
    fields = {"fx", "fy", "cx", "cy", "distortion"}
    if not isinstance(value, dict) or not fields <= set(value):
        raise ValueError(f"camera{index} must contain fx, fy, cx, cy and distortion")
    if "camera" in value and (type(value["camera"]) is not int or value["camera"] != index):
        raise ValueError(f"camera{index}.camera must match its array position")
    requested = {name: _number(value[name], f"camera{index}.{name}", positive=name in ("fx", "fy"))
                 for name in ("fx", "fy", "cx", "cy")}
    distortion = value["distortion"]
    if not isinstance(distortion, list) or len(distortion) != 5:
        raise ValueError(f"camera{index}.distortion must contain k1,k2,p1,p2,k3")
    distortion = [_number(v, f"camera{index}.distortion[{i}]") for i, v in enumerate(distortion)]
    registers = dict(THRESHOLD_REGISTERS, printed_width_q16_16=1 << 16)
    effective, register_intrinsics = {}, {}
    for name in ("fx", "fy"):
        # The input ABI is Q2.30, but the datapath rounds to Q21, ties to
        # even, then uses a signed18-bit multiplier input. Reject saturation
        # and values which become zero rather than silently producing no pose.
        scaled = (1 << 30) / requested[name]
        if not math.isfinite(scaled) or not 0 < scaled < (1 << 26):
            raise ValueError(f"camera{index}.{name} exceeds the solver's inverse-focal range")
        raw = _round(scaled)
        q21, remainder = divmod(raw, 512)
        q21 += int(remainder > 256 or (remainder == 256 and q21 & 1))
        if not 0 < raw < (1 << 26) or not 0 < q21 < (1 << 17):
            raise ValueError(f"camera{index}.{name} is not representable by the pose solver")
        registers[f"inv_{name}_q2_30"] = raw
        register_intrinsics[name] = (1 << 30) / raw
        effective[name] = (1 << 21) / q21
    for name in ("cx", "cy"):
        scaled = requested[name] * (1 << 16)
        if not math.isfinite(scaled) or not -(1 << 27) <= scaled < (1 << 27):
            raise ValueError(f"camera{index}.{name} exceeds the solver's signed principal-point range")
        raw = _round(scaled)
        if not -(1 << 27) <= raw < (1 << 27):
            raise ValueError(f"camera{index}.{name} is not representable by the pose solver")
        registers[f"{name}_q16_16"] = raw & 0xffffffff
        register_intrinsics[name] = raw / (1 << 16)
        effective[name] = (raw >> 10) / 64
    return {"camera": index, **effective, "requested": requested,
            "register_intrinsics": register_intrinsics, "registers": registers,
            "distortion": distortion, "distortion_applied": False}


def load_calibration(path=None):
    """Load schema1 or construct nominal lens values; no hardware access.

    Fit diagnostics and other extra fields are accepted but not echoed in every
    live status. The exact file hash identifies all retained source metadata.
    """
    if path is None:
        value = {"schema_version": 1, "image_size": list(IMAGE_SIZE), "baseline_mm": 120,
                 "cameras": [dict(fx=2.8 / 0.003, fy=2.8 / 0.003, cx=639.5, cy=399.5,
                                  distortion=[0, 0, 0, 0, 0]) for _ in range(2)]}
        provenance = {"kind": "nominal_lens", "focal_length_mm": 2.8,
                      "pixel_pitch_mm": 0.003, "fitted": False,
                      "distortion_known": False}
    else:
        source = Path(path).expanduser().absolute()
        with source.open("rb") as handle:
            data = handle.read(1024 * 1024 + 1)
        if len(data) > 1024 * 1024:
            raise ValueError("calibration file exceeds1MiB")
        try:
            value = json.loads(data)
        except (ValueError, RecursionError) as error:
            raise ValueError(f"invalid calibration JSON: {error}") from error
        provenance = {"kind": "file", "path": str(source),
                      "sha256": hashlib.sha256(data).hexdigest(),
                      "measurement_status": "user_supplied_unverified"}
    required = {"schema_version", "image_size", "baseline_mm", "cameras"}
    if not isinstance(value, dict) or not required <= set(value):
        raise ValueError("calibration requires schema_version, image_size, baseline_mm, cameras and optional tag_width_mm")
    if type(value["schema_version"]) is not int or value["schema_version"] != 1:
        raise ValueError("unsupported calibration schema_version (expected1)")
    if (not isinstance(value["image_size"], list) or value["image_size"] != IMAGE_SIZE
            or any(type(v) is not int for v in value["image_size"])):
        raise ValueError("calibration image_size must be [1280,800]")
    if not isinstance(value["cameras"], list) or len(value["cameras"]) != 2:
        raise ValueError("calibration requires exactly two cameras in camera0,camera1 order")
    result = {"schema_version": 1, "image_size": list(IMAGE_SIZE),
              "baseline_mm": _number(value["baseline_mm"], "baseline_mm", positive=True),
              "tag_width": 1.0, "pose_units": "tag_widths", "distortion_applied": False,
              "intrinsics_basis": "fpga_datapath_effective", "camera_model": "pinhole",
              "provenance": provenance, "cameras": [_camera(c, i) for i, c in enumerate(value["cameras"])],
              "decoder_thresholds": {
                  # score_units applies to hypothesis ranking.
                  "score_units": "squared_homogeneous_decoder_score",
                  "residual_units": "canonical_grid_pitch_squared",
                  "mean_decode_residual_definition": "sum(ex^2 + ey^2) / sum(W^2)",
                  "mean_residual_limit_q16": THRESHOLD_REGISTERS["mean_residual_limit_q16"],
                  "mean_decode_residual_limit": THRESHOLD_REGISTERS["mean_residual_limit_q16"] / (1 << 16),
                  "max_mass_spread": 19, "min_baseline_pixels": 1.0},
              "centroid_stream": True, "pose_stream": True, "readback_verified": False}
    if "tag_width_mm" in value:
        result["tag_width_mm"] = _number(value["tag_width_mm"], "tag_width_mm", positive=True)
    return result


def register_words(camera):
    """Return active CSR offset/value pairs, each in one 32-bit word."""
    result = []
    for name, offset in REGISTER_OFFSETS.items():
        value = camera["registers"][name]
        if name == "mean_residual_limit_q16" and (type(value) is not int or not 0 <= value <= 0xffff):
            raise ValueError("mean_residual_limit_q16 must be an integer in 0..65535")
        result.append((offset, value))
    return result


def configure_camera(capture, camera, *, check_stop=lambda: None):
    """Program and verify an idle camera, then route children and accepted poses."""
    from .capture import CaptureError
    from .ov9281 import require_pair_idle

    check_stop()
    require_pair_idle(capture.bus)
    words = register_words(camera)
    # Each camera has independent intrinsics; common detector fields alias
    # the shared configuration and receive the same values in either bank.
    for offset, value in words:
        check_stop()
        capture.write(capture.base + offset, value)
    check_stop()
    capture.bus.write(capture.base + 0x34, 1)
    if int(capture.bus.read(capture.base + 0x38)) != 1 or int(capture.bus.read(capture.base + 0x3c)) != 0:
        raise CaptureError(f"camera{capture.camera} calibration commit failed")
    # Calibration-valid may have already been set by the placeholder setup.
    # Verify the complete active bank after committing, not only that flag.
    for offset, value in words:
        check_stop()
        if int(capture.bus.read(capture.base + offset)) != value:
            raise CaptureError(f"camera{capture.camera} calibration readback mismatch at 0x{capture.base + offset:08x}")
    for channel in (1, 2):
        product = capture.image + channel * 0x80
        for offset, value in ((0, 0), (4, capture.destination), (8, capture.port), (0, 1)):
            check_stop()
            capture.write(product + offset, value)
    return dict(camera, calibration_commit_verified=True,
                destination=capture.report["destination"], port=capture.port,
                products=[1, 2, 3])
