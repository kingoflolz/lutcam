"""Direct camera calibration controls, without the pixel-processing datapath.

The compact Wishbone decoder separately tests paused-write enforcement and byte
selects. These tests verify Camera's active-bank wiring, CSR compatibility, and
its calibration-valid/configuration-error lifecycle.
"""

from pathlib import Path
from types import SimpleNamespace
import unittest

from migen import ClockDomain, Instance, Signal
from migen.genlib.cdc import MultiReg
from migen.sim import run_simulation
from litex.build.io import DDROutput
from litex.soc.interconnect.csr import CSRStatus

from fpga.litex.lutcam.camera import (
    CALIBRATION, SHARED_CALIBRATION, Camera, shared_detector_configuration,
)


ROOT = Path(__file__).resolve().parents[2]


def camera_fixture(*, direct=False, paired=False, detector_configuration=None):
    clock = SimpleNamespace(**{name: Signal(name="test_clock_" + name)
                               for name in ("clk", "lp_p", "lp_n", "hs_term", "hs_ena")})
    lanes = []
    for index in range(2):
        lane = {name: Signal(name=f"test_lane{index}_" + name)
                for name in ("lp_p", "lp_n", "fifo_empty", "fifo_rd", "rst",
                             "hs_term", "hs_ena")}
        lane["hs_in"] = Signal(8, name=f"test_lane{index}_hs_in")
        lanes.append(SimpleNamespace(**lane))
    pads = SimpleNamespace(**{name: Signal(name="test_pad_" + name)
                              for name in ("strobe", "shutdown", "fsin", "xvclk")})
    return Camera(None, SimpleNamespace(clock=clock, data=tuple(lanes)), pads, ROOT,
                  direct_control=direct, paired=paired,
                  detector_configuration=detector_configuration)


def control_fragment(camera):
    fragment = camera.get_fragment()
    fragment.clock_domains.append(ClockDomain("sys"))
    # Controls-only simulation: external receiver/processing outputs are
    # driven by the bench; no behavioral substitute claims pixel coverage.
    fragment.specials = {special for special in fragment.specials
                         if not isinstance(special, (Instance, MultiReg, DDROutput))}
    return fragment


def pulse(*assignments):
    for signal, value in assignments:
        yield signal.eq(value)
    yield
    for signal, _value in assignments:
        yield signal.eq(0)
    yield


class CameraCalibrationTests(unittest.TestCase):
    def test_pair_shares_only_detector_fields_and_preserves_intrinsics(self):
        shared = shared_detector_configuration()
        cameras = [camera_fixture(direct=True, paired=True, detector_configuration=shared)
                   for _ in range(2)]
        self.assertEqual(sum(len(signal) for signal in shared.values()), 72)
        self.assertNotIn("residual_limit_q32", shared)
        self.assertNotIn("reprojection_limit_q32", shared)
        self.assertNotIn("ambiguity_margin_q32", shared)
        for name, width, default in CALIBRATION:
            for camera in cameras:
                self.assertEqual(len(camera.configuration[name]), width)
                self.assertEqual(camera.configuration[name].reset.value, default)
            if name in SHARED_CALIBRATION:
                self.assertIs(cameras[0].configuration[name], cameras[1].configuration[name])
            else:
                self.assertIsNot(cameras[0].configuration[name], cameras[1].configuration[name])
        self.assertIsNot(cameras[0].calibration_valid.status, cameras[1].calibration_valid.status)

    def test_shared_configuration_rejects_unsafe_control_and_wrong_shape(self):
        shared = shared_detector_configuration()
        for direct, paired in ((False, False), (False, True), (True, False)):
            with self.assertRaisesRegex(ValueError, "paired direct control"):
                camera_fixture(direct=direct, paired=paired, detector_configuration=shared)
        with self.assertRaisesRegex(ValueError, "exactly the common fields"):
            camera_fixture(direct=True, paired=True, detector_configuration={})
        shared["max_mass_spread"] = Signal(9)
        with self.assertRaisesRegex(ValueError, "field width"):
            camera_fixture(direct=True, paired=True, detector_configuration=shared)

    def test_direct_bank_has_exact_widths_defaults_and_no_shadow_csrs(self):
        camera = camera_fixture(direct=True)
        second = camera_fixture(direct=True)
        expected_names = {name for name, _width, _default in CALIBRATION}
        self.assertEqual(set(camera.configuration), expected_names)
        self.assertNotIn("allow_overflow", camera.configuration)
        self.assertEqual(camera.get_csrs(), [])
        self.assertEqual(second.get_csrs(), [])
        for name, width, default in CALIBRATION:
            signal = camera.configuration[name]
            self.assertEqual(len(signal), width)
            self.assertEqual(signal.reset.value, default)
            self.assertFalse(signal.reset_less)
            self.assertFalse(hasattr(camera, "cfg_" + name))
            self.assertIsNot(signal, second.configuration[name])
        self.assertEqual(len({id(value) for value in camera.configuration.values()}),
                         len(CALIBRATION))

    def test_direct_configuration_signals_feed_actual_processing_instance(self):
        camera = camera_fixture(direct=True)
        fragment = camera.get_fragment()
        pipelines = [special for special in fragment.specials
                     if isinstance(special, Instance) and special.of == "CameraPipeline"]
        self.assertEqual(len(pipelines), 1)
        inputs = {item.name: item.expr for item in pipelines[0].items
                  if isinstance(item, Instance.Input)}
        for name, _width, _default in CALIBRATION:
            self.assertIs(inputs["lftag_cfg_" + name], camera.configuration[name])
        self.assertNotIn("lftag_cfg_allow_overflow", inputs)
        self.assertNotIn("lftag_cfg_ambiguity_margin_q32", inputs)
        outputs = {item.name for item in pipelines[0].items if isinstance(item, Instance.Output)}
        self.assertNotIn("lftag_in_overflow", outputs)
        self.assertNotIn("overflow", camera.lftag_candidate)

    def test_standalone_mode_keeps_named_shadow_csrs_and_atomic_copy(self):
        camera = camera_fixture()
        names = {csr.name for csr in camera.get_csrs()}
        self.assertTrue({"run", "calibration_commit", "calibration_valid",
                         "configuration_error"}.issubset(names))
        self.assertNotIn("cfg_allow_overflow", names)
        self.assertNotIn("cfg_ambiguity_margin_q32", names)
        self.assertIn("cfg_reserved", names)
        self.assertIsInstance(camera.cfg_reserved, CSRStatus)
        self.assertFalse(hasattr(camera.cfg_reserved, "storage"))
        self.assertEqual(camera.get_csrs()[-1].name, "cfg_reserved")
        for offset in (0x98, 0x9c, 0xa0, 0xa4, 0xa8):
            reserved = getattr(camera, f"cfg_reserved_{offset:02x}")
            self.assertIsInstance(reserved, CSRStatus)
            self.assertFalse(hasattr(reserved, "storage"))
        self.assertEqual([csr.name for csr in camera.get_csrs() if csr.name.startswith("cfg_")],
                         ["cfg_inv_fx_q2_30", "cfg_inv_fy_q2_30", "cfg_cx_q16_16",
                          "cfg_cy_q16_16", "cfg_printed_width_q16_16", "cfg_mean_residual_limit_q16",
                          "cfg_reserved_98", "cfg_reserved_9c", "cfg_reserved_a0",
                          "cfg_reserved_a4", "cfg_reserved_a8", "cfg_max_mass_spread",
                          "cfg_min_baseline_q4", "cfg_reserved"])
        for name, width, default in CALIBRATION:
            self.assertIn("cfg_" + name, names)
            shadow = getattr(camera, "cfg_" + name)
            self.assertEqual(len(shadow.storage), width)
            self.assertEqual(shadow.storage.reset.value, default)
        values = {name: (0xfedcba9876543211 ^ (index * 0x102030405)) & ((1 << width) - 1)
                  for index, (name, width, _default) in enumerate(CALIBRATION)}
        fragment = control_fragment(camera)

        def bench():
            yield camera.idle.eq(1)
            self.assertEqual((yield camera.cfg_reserved.status), 0)
            for name, _width, _default in CALIBRATION:
                yield getattr(camera, "cfg_" + name).storage.eq(values[name])
            yield
            for name, _width, default in CALIBRATION:
                self.assertEqual((yield camera.configuration[name]), default)
            yield from pulse((camera.calibration_commit.re, 1))
            self.assertEqual((yield camera.calibration_valid.status), 1)
            for name, _width, _default in CALIBRATION:
                self.assertEqual((yield camera.configuration[name]), values[name])
            # A later shadow update does not leak into the active bank, even
            # when an illegal armed commit is attempted.
            yield camera.cfg_inv_fx_q2_30.storage.eq(0x12345678)
            yield camera.run.storage.eq(1)
            yield from pulse((camera.calibration_commit.re, 1))
            self.assertEqual((yield camera.configuration["inv_fx_q2_30"]),
                             values["inv_fx_q2_30"])
            self.assertEqual((yield camera.configuration_error.status), 1)

        run_simulation(fragment, bench())

    def test_direct_paused_updates_are_immediate_and_retain_existing_valid(self):
        camera = camera_fixture(direct=True)
        fragment = control_fragment(camera)

        def bench():
            self.assertEqual((yield camera.calibration_valid.status), 0)
            self.assertEqual((yield camera.configuration_error.status), 0)
            yield camera.idle.eq(1)
            for index, (name, width, _default) in enumerate(CALIBRATION):
                value = (0x81234567fedcba91 ^ (index * 0x112233)) & ((1 << width) - 1)
                yield camera.configuration[name].eq(value)
                yield
                self.assertEqual((yield camera.configuration[name]), value)
                self.assertEqual((yield camera.calibration_valid.status), 0)
            yield from pulse((camera.calibration_commit.re, 1))
            self.assertEqual((yield camera.calibration_valid.status), 1)
            yield camera.configuration["mean_residual_limit_q16"].eq(0x3210)
            yield
            self.assertEqual((yield camera.configuration["mean_residual_limit_q16"]), 0x3210)
            self.assertEqual((yield camera.calibration_valid.status), 1)
            self.assertEqual((yield camera.configuration_error.status), 0)
            yield from pulse((camera.clear.re, 1))
            self.assertEqual((yield camera.calibration_valid.status), 1)

        run_simulation(fragment, bench())

    def test_direct_commit_requires_idle_and_disarmed_and_errors_are_sticky(self):
        camera = camera_fixture(direct=True)
        fragment = control_fragment(camera)

        def bench():
            for idle, run in ((0, 0), (0, 1), (1, 1)):
                yield camera.idle.eq(idle)
                yield camera.run.storage.eq(run)
                yield from pulse((camera.calibration_commit.re, 1))
                self.assertEqual((yield camera.calibration_valid.status), 0)
                self.assertEqual((yield camera.configuration_error.status), 1)
                yield from pulse((camera.clear.re, 1))
                self.assertEqual((yield camera.configuration_error.status), 0)
            yield camera.idle.eq(1)
            yield camera.run.storage.eq(0)
            yield from pulse((camera.configuration_write_error, 1))
            self.assertEqual((yield camera.configuration_error.status), 1)
            yield from pulse((camera.calibration_commit.re, 1))
            self.assertEqual((yield camera.calibration_valid.status), 1)
            self.assertEqual((yield camera.configuration_error.status), 1,
                             "a legal commit must not clear a previous write error")
            yield camera.run.storage.eq(1)
            yield from pulse((camera.calibration_commit.re, 1))
            self.assertEqual((yield camera.calibration_valid.status), 1)
            self.assertEqual((yield camera.configuration_error.status), 1)
            yield from pulse((camera.clear.re, 1))
            self.assertEqual((yield camera.calibration_valid.status), 1)
            self.assertEqual((yield camera.configuration_error.status), 0)

        run_simulation(fragment, bench())

    def test_direct_new_errors_win_same_edge_clear(self):
        camera = camera_fixture(direct=True)
        fragment = control_fragment(camera)

        def bench():
            # An illegal commit and an external write rejection each dominate
            # a simultaneous clear, including when no old error was set.
            yield camera.idle.eq(0)
            yield from pulse((camera.clear.re, 1), (camera.calibration_commit.re, 1))
            self.assertEqual((yield camera.configuration_error.status), 1)
            self.assertEqual((yield camera.calibration_valid.status), 0)
            yield from pulse((camera.clear.re, 1))
            self.assertEqual((yield camera.configuration_error.status), 0)
            yield from pulse((camera.clear.re, 1), (camera.configuration_write_error, 1))
            self.assertEqual((yield camera.configuration_error.status), 1)
            yield camera.idle.eq(1)
            yield from pulse((camera.clear.re, 1), (camera.calibration_commit.re, 1),
                             (camera.configuration_write_error, 1))
            self.assertEqual((yield camera.calibration_valid.status), 1)
            self.assertEqual((yield camera.configuration_error.status), 1)
            yield from pulse((camera.clear.re, 1))
            self.assertEqual((yield camera.configuration_error.status), 0)
            self.assertEqual((yield camera.calibration_valid.status), 1)

        run_simulation(fragment, bench())


if __name__ == "__main__":
    unittest.main()
