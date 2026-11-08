"""Live calibration encoding and checked setup; no board or network access."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.lutcam.calibration_config import (
    configure_camera, load_calibration, register_words,
)
from tools.lutcam.capture import CaptureError


def source_document():
    return {"schema_version": 1, "image_size": [1280, 800], "baseline_mm": 120.5,
            "cameras": [
                {"fx": 817.35, "fy": 903.125, "cx": 641.125, "cy": 398.25,
                 "distortion": [-0.17, 0.004, 0.001, -0.002, 0.0003]},
                {"fx": 1000.125, "fy": 990.75, "cx": 637.75, "cy": 403.125,
                 "distortion": [0.012, -0.002, 0.003, 0.004, -0.005]}]}


class CalibrationLoadTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "calibration.json"

    def load(self, value):
        self.path.write_text(json.dumps(value), encoding="utf-8")
        return load_calibration(self.path)

    def test_nominal_defaults_have_exact_registers_and_effective_intrinsics(self):
        report = load_calibration()
        self.assertEqual(report["image_size"], [1280, 800])
        self.assertEqual(report["baseline_mm"], 120)
        self.assertEqual((report["tag_width"], report["pose_units"]), (1, "tag_widths"))
        self.assertFalse(report["readback_verified"])
        self.assertFalse(report["distortion_applied"])
        self.assertEqual(report["provenance"]["kind"], "nominal_lens")
        self.assertFalse(report["provenance"]["fitted"])
        self.assertFalse(report["provenance"]["distortion_known"])
        for index, camera in enumerate(report["cameras"]):
            self.assertEqual(camera["camera"], index)
            expected = {
                "inv_fx_q2_30": 1_150_438, "inv_fy_q2_30": 1_150_438,
                "cx_q16_16": 639 * 65536 + 32768,
                "cy_q16_16": 399 * 65536 + 32768,
                "printed_width_q16_16": 65536,
                "mean_residual_limit_q16": 1024,
                "max_mass_spread": 19, "min_baseline_q4": 16,
            }
            self.assertEqual(camera["registers"], expected)
            self.assertEqual(camera["requested"]["fx"], 2.8 / 0.003)
            self.assertEqual(camera["register_intrinsics"]["fx"], 1073741824 / 1150438)
            self.assertEqual(camera["fx"], 2097152 / 2247)
            self.assertEqual((camera["cx"], camera["cy"]), (639.5, 399.5))
            self.assertFalse(camera["distortion_applied"])
        self.assertEqual(report["decoder_thresholds"]["score_units"],
                         "squared_homogeneous_decoder_score")
        self.assertEqual(report["decoder_thresholds"]["residual_units"],
                         "canonical_grid_pitch_squared")
        self.assertEqual(report["decoder_thresholds"]["mean_decode_residual_definition"],
                         "sum(ex^2 + ey^2) / sum(W^2)")
        self.assertEqual(report["decoder_thresholds"]["mean_decode_residual_limit"], 1 / 64)
        self.assertEqual(report["decoder_thresholds"]["mean_residual_limit_q16"], 1024)
        self.assertNotIn("residual_limit", report["decoder_thresholds"])
        self.assertNotIn("ambiguity_margin", report["decoder_thresholds"])
        self.assertNotIn("allow_overflow", report["decoder_thresholds"])



    def test_supplied_camera_values_metadata_and_exact_file_provenance(self):
        value = source_document()
        value["tag_width_mm"] = 173.25
        encoded = (json.dumps(value, indent=2) + "\n").encode()
        self.path.write_bytes(encoded)
        report = load_calibration(self.path)
        self.assertEqual(report["provenance"]["kind"], "file")
        self.assertEqual(report["provenance"]["path"], str(self.path.absolute()))
        self.assertEqual(report["provenance"]["sha256"], hashlib.sha256(encoded).hexdigest())
        self.assertEqual(report["provenance"]["measurement_status"], "user_supplied_unverified")
        self.assertEqual(report["baseline_mm"], 120.5)
        self.assertEqual(report["tag_width_mm"], 173.25)
        self.assertEqual(report["tag_width"], 1.0)
        for camera, requested in zip(report["cameras"], value["cameras"]):
            self.assertEqual(camera["requested"], {key: requested[key] for key in ("fx", "fy", "cx", "cy")})
            self.assertEqual(camera["distortion"], requested["distortion"])
            self.assertFalse(camera["distortion_applied"])
            self.assertEqual(camera["registers"]["printed_width_q16_16"], 0x10000)
        # Formatting is part of the provenance, not only parsed values.
        self.path.write_bytes(json.dumps(value).encode())
        self.assertNotEqual(load_calibration(self.path)["provenance"]["sha256"],
                            report["provenance"]["sha256"])

    def test_full_solver_export_is_accepted_without_echoing_fit_metadata(self):
        value = source_document()
        value.update(
            stereo={"rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                    "translation_mm": [-120.5, 0, 0],
                    "convention": "P_camera1 = rotation @ P_camera0 + translation_mm"},
            tag_width_mm=180,
            metrics={"rms_reprojection_px": 0.18, "paired_frames": 24},
            assumptions=["All observed tags share one printed outer width.",
                         "Zero skew; Brown-Conrady k1,k2,p1,p2 fitted, k3 fixed at zero."],
            provenance={"observations_sha256": "0123456789abcdef" * 4},
            validated=True)
        for index, camera in enumerate(value["cameras"]):
            camera.update(camera=index, fit_diagnostics={"rms_px": 0.17 + index * 0.02})
        report = self.load(value)
        self.assertEqual([camera["camera"] for camera in report["cameras"]], [0, 1])
        self.assertEqual(report["tag_width_mm"], 180)
        self.assertEqual(report["camera_model"], "pinhole")
        self.assertEqual(report["intrinsics_basis"], "fpga_datapath_effective")
        self.assertEqual(report["provenance"]["sha256"], hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertEqual(report["provenance"]["measurement_status"], "user_supplied_unverified")
        for field in ("stereo", "metrics", "assumptions", "validated"):
            self.assertNotIn(field, report)
        self.assertTrue(all("fit_diagnostics" not in camera for camera in report["cameras"]))

    def test_optional_camera_identity_must_match_its_position(self):
        for camera, invalid_id in ((0, 1), (1, 0), (0, True), (0, 0.0), (1, "1")):
            with self.subTest(camera=camera, invalid_id=invalid_id):
                value = source_document()
                value["cameras"][camera]["camera"] = invalid_id
                with self.assertRaises(ValueError):
                    self.load(value)

    def test_inverse_focal_inner_rounding_uses_nearest_even(self):
        # Independent vectors straddle exact half-LSBs after Q30 quantization.
        for raw, expected_q21 in ((1000 * 512 + 255, 1000),
                                  (1000 * 512 + 256, 1000),
                                  (1000 * 512 + 257, 1001),
                                  (1001 * 512 + 256, 1002),
                                  (257, 1), ((1 << 26) - 257, 131071)):
            with self.subTest(raw=raw):
                value = source_document()
                value["cameras"][0].update(fx=1073741824 / raw, fy=1073741824 / raw)
                camera = self.load(value)["cameras"][0]
                for axis in ("fx", "fy"):
                    self.assertEqual(camera["registers"][f"inv_{axis}_q2_30"], raw)
                    self.assertEqual(camera["register_intrinsics"][axis], 1073741824 / raw)
                    self.assertEqual(camera[axis], 2097152 / expected_q21)

    def test_signed_principal_rounding_and_arithmetic_truncation(self):
        for requested, encoded, effective in (
                (-0.5 / 65536, 0xffffffff, -1 / 64),
                (0.5 / 65536, 1, 0),
                (-1023 / 65536, 0xfffffc01, -1 / 64),
                (-1025 / 65536, 0xfffffbff, -2 / 64),
                (-2048, 0xf8000000, -2048),
                (2048 - 1 / 65536, 0x07ffffff, 2048 - 1 / 64)):
            with self.subTest(requested=requested):
                value = source_document()
                value["cameras"][0].update(cx=requested, cy=requested)
                camera = self.load(value)["cameras"][0]
                for axis in ("cx", "cy"):
                    self.assertEqual(camera["registers"][f"{axis}_q16_16"], encoded)
                    self.assertEqual(camera[axis], effective)
                    signed = encoded if encoded < 0x80000000 else encoded - 0x100000000
                    self.assertEqual(camera["register_intrinsics"][axis], signed / 65536)

    def test_numeric_range_and_quantization_failures_are_rejected(self):
        invalid = []
        for field in ("fx", "fy"):
            invalid += [(field, item) for item in
                        (0, -1, True, "800", None, float("nan"), float("inf"),
                         10**400, 1e-308, 1e308, 16,
                         1073741824 / 256, 1073741824 / ((1 << 26) - 256))]
        for field in ("cx", "cy"):
            invalid += [(field, item) for item in
                        (True, "0", None, float("nan"), float("inf"),
                         10**400, -2048 - 1 / 65536, 2048,
                         2048 - 0.5 / 65536)]
        for field, item in invalid:
            with self.subTest(field=field, value=item):
                value = source_document()
                value["cameras"][1][field] = item
                with self.assertRaises(ValueError):
                    self.load(value)

    def test_strict_schema_and_metadata_validation(self):
        invalid = [None, [], {}]
        for field in source_document():
            value = source_document()
            del value[field]
            invalid.append(value)
        for field, replacements in (
                ("schema_version", [True, 0, 2, 1.0, "1"]),
                ("image_size", [[1280.0, 800], [800, 1280], [1280, 800, 1], "1280x800"]),
                ("baseline_mm", [0, -1, True, float("nan"), float("inf")]),
                ("cameras", [[], [source_document()["cameras"][0]], {}, [None, None]]),
                ("tag_width_mm", [0, -1, True, float("nan")])):
            for replacement in replacements:
                value = source_document()
                value[field] = replacement
                invalid.append(value)
        for change in ("missing", "distortion-short", "distortion-object",
                       "distortion-bool", "distortion-nonfinite"):
            value = source_document()
            if change == "missing":
                del value["cameras"][0]["cx"]
            else:
                value["cameras"][0]["distortion"] = {
                    "distortion-short": [0] * 4, "distortion-object": {},
                    "distortion-bool": [0, 0, 0, 0, True],
                    "distortion-nonfinite": [0, 0, 0, 0, float("nan")]}[change]
            invalid.append(value)
        for index, value in enumerate(invalid):
            with self.subTest(case=index):
                with self.assertRaises(ValueError):
                    self.load(value)

    def test_invalid_json_and_oversized_files_are_rejected(self):
        for data in (b"{", b"\xff", b" " * (1024 * 1024 + 1)):
            with self.subTest(length=len(data)):
                self.path.write_bytes(data)
                with self.assertRaises(ValueError):
                    load_calibration(self.path)

    def test_active_configuration_uses_single_words_and_preserves_offsets(self):
        camera = load_calibration()["cameras"][0]
        camera["registers"]["mean_residual_limit_q16"] = 0x1234
        words = register_words(camera)
        self.assertEqual([offset for offset, _ in words],
                         [0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0xac, 0xb0])
        self.assertEqual(words[5], (0x94, 0x1234))

    def test_mean_limit_units_preserve_the_previous_stricter_default(self):
        thresholds = load_calibration()["decoder_thresholds"]
        limit_q16 = thresholds["mean_residual_limit_q16"]
        self.assertGreater(limit_q16, 0)
        self.assertLess(limit_q16, 1 << 16)
        self.assertEqual(limit_q16 / 65536, thresholds["mean_decode_residual_limit"])
        self.assertEqual(limit_q16 << 16, 0x04000000)
        # The removed total-residual check was looser than this mean check.
        self.assertLess(limit_q16 / 65536, 0x147ae148 / (5 * (1 << 32)))

    def test_mean_limit_rejects_non_integer_and_out_of_range_values(self):
        camera = load_calibration()["cameras"][0]
        for value in (-1, 65536, 1 << 64, 1024.0, True, False, "1024", None):
            with self.subTest(value=value):
                camera["registers"]["mean_residual_limit_q16"] = value
                with self.assertRaisesRegex(ValueError, "mean_residual_limit_q16.*0..65535"):
                    register_words(camera)
        for value in (0, 1, 1024, 65535):
            with self.subTest(value=value):
                camera["registers"]["mean_residual_limit_q16"] = value
                self.assertEqual(dict(register_words(camera))[0x94], value)


class FakeCapture:
    def __init__(self, camera):
        self.camera = camera
        self.base = 0x30024000 + camera * 0x100
        self.image = 0x30024400 + camera * 0x180
        self.destination, self.port = 0xc0a801fb, 6000 + 3 * camera
        self.report = {"destination": "192.168.1.251"}
        self.bus = self
        # A previously committed placeholder deliberately starts valid.
        self.values = {self.base + 0x38: 1, self.base + 0x3c: 0}
        self.events = []
        self.commit_seen = False
        self.on_commit = None
        self.fail_address = None

    def read(self, address):
        self.events.append(("read", address))
        return self.values.get(address, 0)

    def write(self, address, value):
        self.events.append(("write", address, value))
        if address == self.fail_address:
            raise TimeoutError("injected management write failure")
        self.values[address] = value
        if address == self.base + 0x34:
            self.commit_seen = True
            if self.on_commit is not None:
                self.on_commit()
        # Match DiagnosticCapture.write for ordinary storage registers. The
        # raw commit shares this method in the fake but is a write-only pulse.
        if address != self.base + 0x34 and self.read(address) != value:
            raise CaptureError("immediate readback mismatch")


class CalibrationSetupTests(unittest.TestCase):
    def fixture(self, camera=0):
        capture = FakeCapture(camera)
        configuration = deepcopy(load_calibration()["cameras"][camera])
        return capture, configuration

    def configure(self, capture, configuration, **kwargs):
        with patch("tools.lutcam.ov9281.require_pair_idle") as guard:
            result = configure_camera(capture, configuration, **kwargs)
        guard.assert_called_once_with(capture.bus)
        return result

    def assert_no_stream_enable(self, capture):
        enables = {capture.image + 0x80, capture.image + 0x100}
        self.assertFalse(any(event[0] == "write" and event[1] in enables and event[2] == 1
                             for event in capture.events))

    def test_full_bank_is_verified_after_commit_before_shared_port_streams(self):
        for camera in (0, 1):
            with self.subTest(camera=camera):
                capture, configuration = self.fixture(camera)
                result = self.configure(capture, configuration)
                reserved = {capture.base + offset for offset in (0x98, 0xa4, 0xa8, 0xb4)}
                self.assertFalse(any(event[0] == "write" and event[1] in reserved
                                     for event in capture.events))
                commit_index = capture.events.index(("write", capture.base + 0x34, 1))
                first_route = next(index for index, event in enumerate(capture.events)
                                   if event[0] == "write" and event[1] == capture.image + 0x80)
                for offset, value in register_words(configuration):
                    self.assertIn(("read", capture.base + offset),
                                  capture.events[commit_index + 1:first_route])
                    self.assertEqual(capture.values[capture.base + offset], value)
                route_writes = [event for event in capture.events[first_route:] if event[0] == "write"]
                self.assertEqual(route_writes, [
                    ("write", capture.image + channel * 0x80 + offset, value)
                    for channel in (1, 2)
                    for offset, value in ((0, 0), (4, capture.destination), (8, capture.port), (0, 1))])
                self.assertTrue(result["calibration_commit_verified"])
                self.assertEqual(result["products"], [1, 2, 3])
                self.assertEqual(result["port"], capture.port)

    def test_busy_pair_guard_prevents_all_mutations(self):
        capture, configuration = self.fixture()
        with patch("tools.lutcam.ov9281.require_pair_idle",
                   side_effect=CaptureError("pair active")) as guard:
            with self.assertRaises(CaptureError):
                configure_camera(capture, configuration)
        guard.assert_called_once_with(capture.bus)
        self.assertEqual(capture.events, [])

    def test_stale_active_bank_fails_even_when_placeholder_valid_is_one(self):
        capture, configuration = self.fixture()
        capture.on_commit = lambda: capture.values.__setitem__(capture.base + 0x94, 0)
        with self.assertRaisesRegex(CaptureError, "readback mismatch"):
            self.configure(capture, configuration)
        self.assertEqual(capture.values[capture.base + 0x38], 1)
        self.assert_no_stream_enable(capture)

    def test_commit_flags_are_checked_before_enabling_streams(self):
        for valid, error in ((0, 0), (1, 1), (0xffffffff, 0), (1, 0xffffffff)):
            with self.subTest(valid=valid, error=error):
                capture, configuration = self.fixture()
                capture.on_commit = lambda: capture.values.update(
                    {capture.base + 0x38: valid, capture.base + 0x3c: error})
                with self.assertRaisesRegex(CaptureError, "commit failed"):
                    self.configure(capture, configuration)
                self.assert_no_stream_enable(capture)

    def test_management_write_failure_does_not_commit_or_enable_streams(self):
        capture, configuration = self.fixture()
        capture.fail_address = capture.base + 0x88
        with self.assertRaises(TimeoutError):
            self.configure(capture, configuration)
        self.assertFalse(capture.commit_seen)
        self.assert_no_stream_enable(capture)

    def test_stop_before_setup_or_during_bank_write_prevents_stream_enable(self):
        class Cancelled(Exception):
            pass

        for phase in ("before", "writing", "after-commit"):
            with self.subTest(phase=phase):
                capture, configuration = self.fixture()

                def check():
                    writes = sum(event[0] == "write" for event in capture.events)
                    if (phase == "before" or phase == "writing" and writes >= 3
                            or phase == "after-commit" and capture.commit_seen):
                        raise Cancelled()

                with self.assertRaises(Cancelled):
                    self.configure(capture, configuration, check_stop=check)
                self.assert_no_stream_enable(capture)
                if phase == "before":
                    self.assertEqual(capture.events, [])
                elif phase == "writing":
                    self.assertFalse(capture.commit_seen)
                    self.assertEqual(sum(event[0] == "write" for event in capture.events), 3)
                else:
                    self.assertTrue(capture.commit_seen)


if __name__ == "__main__":
    unittest.main()
