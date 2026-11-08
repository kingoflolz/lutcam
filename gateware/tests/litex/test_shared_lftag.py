"""Shared LFTag Python/RTL boundary, including packed camera lane ordering."""

from pathlib import Path
from types import SimpleNamespace
import random
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "fpga" / "litex"))

from migen import Instance, Signal
from migen.fhdl import verilog

from lutcam.camera import Camera, CALIBRATION, LFTAG_CANDIDATE_FIELDS, LFTAG_RETURN_FIELDS, LFTAG_DEBUG_FIELDS
from lutcam.shared_lftag import SharedLfTag
from lutcam.calibration import CalibrationBank


def fake_camera(index):
    def signal(name, width=1):
        return Signal(width, name=f"c{index}_{name}")
    return SimpleNamespace(
        shared_lftag=True,
        lftag_processing_reset=signal("reset"),
        clear=SimpleNamespace(re=signal("clear")),
        pose=SimpleNamespace(ready=signal("ready")),
        lftag_candidate={name: signal("in_" + name, width)
                         for name, width in LFTAG_CANDIDATE_FIELDS},
        lftag_return={name: signal("out_" + name, width)
                      for name, width in LFTAG_RETURN_FIELDS},
        lftag_debug={name: signal("debug_" + name, 32) for name in LFTAG_DEBUG_FIELDS},
        configuration={name: signal("cfg_" + name, width)
                       for name, width, _ in CALIBRATION},
    )


class SharedLfTagGlueTests(unittest.TestCase):
    def test_external_configuration_uses_shared_sequential_bank(self):
        cameras = [fake_camera(index) for index in range(2)]
        bank = CalibrationBank()
        fragment = SharedLfTag(cameras, configuration_bank=bank).get_fragment()
        instance = next(item for item in fragment.specials
                        if isinstance(item, Instance) and item.of == "SharedLfTagDecoder")
        ports = {item.name: item.expr for item in instance.items
                 if isinstance(item, (Instance.Input, Instance.Output))}
        parameters = {item.name: item.value.value for item in instance.items
                      if isinstance(item, Instance.Parameter)}
        self.assertEqual(parameters["EXTERNAL_CONFIG"], 1)
        for port, field in (("config_read_valid", "cfg_req_valid"),
                            ("config_read_address", "cfg_req_addr"),
                            ("config_read_ready", "cfg_req_ready"),
                            ("config_response_valid", "cfg_rsp_valid"),
                            ("config_read_data", "cfg_rsp_data")):
            self.assertIs(ports[port], getattr(bank, field))
        for name, width, _ in CALIBRATION:
            self.assertEqual(len(ports["cfg_" + name]), width * 2)
            self.assertEqual(ports["cfg_" + name].value, 0)

    def test_accepted_centroid_ports_preserve_camera_lane_order(self):
        cameras = [fake_camera(index) for index in range(2)]
        for camera in cameras:
            camera.accepted_centroids = True
            camera.children = SimpleNamespace(
                valid=Signal(), ready=Signal(), data=Signal(33), end=Signal(),
            )
        fragment = SharedLfTag(cameras, accepted_centroids=True).get_fragment()
        instance = next(item for item in fragment.specials
                        if isinstance(item, Instance) and item.of == "SharedLfTagDecoder")
        ports = {item.name: item.expr for item in instance.items
                 if isinstance(item, (Instance.Input, Instance.Output))}
        for port, field in (("centroid_ready", "ready"), ("centroid_valid", "valid"),
                            ("centroid_last", "end")):
            for index, camera in enumerate(cameras):
                self.assertIs(ports[port].l[index], getattr(camera.children, field))
        self.assertEqual(len(ports["centroid_data"]), 33)
        cameras[1].accepted_centroids = False
        with self.assertRaises(ValueError):
            SharedLfTag(cameras, accepted_centroids=True)

    def test_rejects_missing_or_private_camera(self):
        for cameras in ([], [fake_camera(0)], [fake_camera(i) for i in range(3)]):
            with self.subTest(count=len(cameras)), self.assertRaises(ValueError):
                SharedLfTag(cameras)
        cameras = [fake_camera(0), fake_camera(1)]
        cameras[1].shared_lftag = False
        with self.assertRaises(ValueError):
            SharedLfTag(cameras)

    def test_camera_selects_external_mode_and_retains_ownership_boundary(self):
        pads = SimpleNamespace(**{name: Signal() for name in ("strobe", "xvclk", "shutdown", "fsin")})
        clock = SimpleNamespace(**{name: Signal() for name in ("clk", "lp_p", "lp_n", "hs_term", "hs_ena")})
        lanes = [SimpleNamespace(**{
            name: Signal(8 if name == "hs_in" else 1)
            for name in ("lp_p", "lp_n", "hs_in", "fifo_empty", "fifo_rd", "rst", "hs_term", "hs_ena")
        }) for _ in range(2)]
        camera = Camera(None, SimpleNamespace(clock=clock, data=lanes), pads,
                        ROOT, direct_control=True, paired=True, shared_lftag=True)
        fragment = camera.get_fragment()
        pipeline = next(s for s in fragment.specials
                        if isinstance(s, Instance) and s.of == "CameraPipeline")
        parameters = {item.name: item.value.value for item in pipeline.items
                      if isinstance(item, Instance.Parameter) and hasattr(item.value, "value")}
        self.assertEqual(parameters["EXTERNAL_LFTAG"], 1)
        self.assertEqual(parameters["PAIRED_MODE"], 1)
        ports = {item.name: item.expr for item in pipeline.items
                 if isinstance(item, (Instance.Input, Instance.Output))}
        for name, signal in camera.lftag_candidate.items():
            self.assertIs(ports["lftag_in_" + name], signal)
        for name, signal in camera.lftag_return.items():
            self.assertIs(ports["external_lftag_" + name], signal)
        self.assertIs(ports["lftag_processing_reset"], camera.lftag_processing_reset)
        self.assertIs(ports["capture_frame_id"], camera.pending_frame_id)
        self.assertIs(ports["frame_id"], camera.frame_id)
        self.assertIs(ports["lftag_result_ready"], camera.pose.ready)
        self.assertIs(ports["lftag_result_valid"], camera.pose.valid)
        self.assertIs(ports["lftag_result_last"], camera.pose.last)
        self.assertIs(ports["lftag_result_data"], camera.pose.data)
        self.assertNotIn("lftag_in_overflow", ports)
        self.assertNotIn("lftag_cfg_allow_overflow", ports)

    @unittest.skipUnless(shutil.which("iverilog") and shutil.which("vvp"), "Icarus is required")
    def test_generated_rtl_preserves_all_packed_camera_lanes(self):
        # Compile the actual generated Migen wiring against a boundary probe.
        # The numeric solver has its own RTL tests; this deliberately catches
        # Python packing, port-name, width and result-demultiplexing mistakes.
        cameras = [fake_camera(0), fake_camera(1)]
        inputs = {
            "camera_reset": [c.lftag_processing_reset for c in cameras],
            "clear_diagnostics": [c.clear.re for c in cameras],
            "out_ready": [c.pose.ready for c in cameras],
        }
        inputs.update({"in_" + name: [c.lftag_candidate[name] for c in cameras]
                       for name, _ in LFTAG_CANDIDATE_FIELDS})
        inputs.update({"cfg_" + name: [c.configuration[name] for c in cameras]
                       for name, _, _ in CALIBRATION})
        outputs = {
            {"result_valid": "out_valid", "result_last": "out_last"}.get(name, name):
            [c.lftag_return[name] for c in cameras]
            for name, _ in LFTAG_RETURN_FIELDS if name != "result_data"
        }
        outputs.update({"debug_" + name: [c.lftag_debug[name] for c in cameras]
                        for name in LFTAG_DEBUG_FIELDS})
        input_signals = [s for signals in inputs.values() for s in signals]
        output_signals = [s for c in cameras for s in (*c.lftag_return.values(), *c.lftag_debug.values())]
        generated = verilog.convert(SharedLfTag(cameras),
            ios=set(input_signals + output_signals), name="shared_glue")
        self.assertNotIn(".in_overflow(", str(generated))
        self.assertNotIn(".cfg_allow_overflow(", str(generated))
        name = generated.ns.get_name
        rng = random.Random(0x4C465447)
        values = [{name(s): rng.getrandbits(len(s)) for s in input_signals + output_signals}
                  for _ in range(2)]
        # Every one-bit lane changes between phases, so constant/tied outputs
        # cannot accidentally match the randomized vector.
        for signal in input_signals + output_signals:
            if len(signal) == 1:
                values[1][name(signal)] = values[0][name(signal)] ^ 1
        data = [0x89abcdef, 0x12345678]
        for phase in range(2):
            for camera_index, camera in enumerate(cameras):
                values[phase][name(camera.lftag_return["result_data"])] = data[phase]
                # Occupancy 32 exercises the new high bit independently in
                # both packed camera lanes, including its position in lane 1.
                values[phase][name(camera.lftag_return["fifo_level"])] = (
                    ((32, 17), (3, 32))[phase][camera_index])

        def literal(width, value):
            return f"{width}'h{value:x}"

        def packed(signals, phase):
            width = len(signals[0])
            return literal(width * 2,
                values[phase][name(signals[0])] | (values[phase][name(signals[1])] << width))

        declarations = ["input wire clk, rst"]
        declarations += [f"input wire [{2 * len(signals[0]) - 1}:0] {port}"
                         for port, signals in inputs.items()]
        declarations += [f"output wire [{2 * len(signals[0]) - 1}:0] {port}"
                         for port, signals in outputs.items()]
        declarations += ["output wire [31:0] out_data"]
        probe = ["module SharedLfTagDecoder(\n" + ",\n".join(declarations) + "\n);"]
        probe += [f"assign {port} = rst ? {packed(signals, 1)} : {packed(signals, 0)};"
                  for port, signals in outputs.items()]
        probe += [f"assign out_data = rst ? 32'h{data[1]:x} : 32'h{data[0]:x};", "initial begin"]
        for phase in range(2):
            probe += ["#1;" if phase == 0 else "#2;"]
            probe += [f'if ({port} !== {packed(signals, phase)}) $fatal(1, "packed input {port} phase {phase}");'
                      for port, signals in inputs.items()]
        probe += ["end", "endmodule"]

        bench = ["module tb;", "reg sys_clk = 0, sys_rst = 0;"]
        bench += [f"reg [{len(s)-1}:0] {name(s)};" for s in input_signals]
        bench += [f"wire [{len(s)-1}:0] {name(s)};" for s in output_signals]
        bench += ["shared_glue dut(.*);", "initial begin"]
        for phase in range(2):
            bench += [f"sys_rst = {phase};"]
            bench += [f"{name(s)} = {literal(len(s), values[phase][name(s)])};" for s in input_signals]
            bench += ["#2;"]
            bench += [f'if ({name(s)} !== {literal(len(s), values[phase][name(s)])}) $fatal(1, "return {name(s)} phase {phase}");'
                      for s in output_signals]
        bench += ['$display("PASS generated shared LFTag camera packing");', "$finish;", "end", "endmodule"]
        with tempfile.TemporaryDirectory(prefix="lutcam-shared-glue-") as temporary:
            directory = Path(temporary)
            generated.write(str(directory / "glue.v"))
            (directory / "probe.sv").write_text("\n".join(probe + bench) + "\n")
            result = subprocess.run(["iverilog", "-g2012", "-s", "tb", "-o", str(directory / "sim"),
                str(directory / "glue.v"), str(directory / "probe.sv")], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run(["vvp", str(directory / "sim")], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS generated shared LFTag camera packing", result.stdout)


if __name__ == "__main__":
    unittest.main()
