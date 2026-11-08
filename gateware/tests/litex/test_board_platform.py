"""Run the real Efinity peripheral validator when the toolchain is installed."""

import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from litex.gen import LiteXContext, LiteXModule
from litex.build.efinix.toolchain import find_efinity_path
from migen.fhdl.tools import list_signals
from migen.genlib.resetsync import AsyncResetSynchronizer
from lutcam.crg import LutcamCRG, LutcamRGMII
from lutcam.platform import Platform, request_camera_hsio
from lutcam.soc import LutcamSoC


class BoardPeripheralTest(unittest.TestCase):
    def test_efinity_validates_real_pinout_and_exact_clocks(self):
        self._validate_peripherals("C3")

    def test_unsupported_timing_model_is_rejected(self):
        for timing_model in ("C2", "C4"):
            with self.subTest(timing_model=timing_model):
                with self.assertRaisesRegex(ValueError, "require C3"):
                    Platform(timing_model=timing_model)

    def test_soc_cannot_override_the_required_timing_model(self):
        with self.assertRaisesRegex(ValueError, "require C3"):
            LutcamSoC(timing_model="C4")
        with self.assertRaisesRegex(ValueError, "platform with C3"):
            LutcamSoC(platform=SimpleNamespace(timing_model="C4"))

    def test_golden_maintenance_cannot_be_built_as_the_boot_scanner(self):
        with self.assertRaisesRegex(ValueError, "must execute as an application"):
            LutcamSoC(golden=True, golden_maintenance=True)

    def _validate_peripherals(self, timing_model):
        try:
            find_efinity_path()
        except OSError:
            self.skipTest("Set LITEX_ENV_EFINITY to run real peripheral validation")
        previous_platform = LiteXContext.platform
        previous_cwd = Path.cwd()
        try:
            platform = Platform()
            self.assertEqual(platform.timing_model, timing_model)
            LiteXContext.platform = platform
            design = LiteXModule()
            design.crg = LutcamCRG(platform)
            design.phy = LutcamRGMII(platform, platform.request("eth_clocks"),
                                    platform.request("eth"), design.crg)
            # Link loss must reset RX locally while the oscillator-driven
            # system, USB and Ethernet TX domains remain independent.
            pll_blocks = [block for block in platform.toolchain.ifacewriter.blocks
                          if block["type"] == "PLL"]
            self.assertEqual(len(pll_blocks), 3)
            rx_pll = next(block for block in pll_blocks if block["name"] == design.phy.rx_pll.name)
            self.assertEqual(rx_pll["input_clock"], "INTERNAL")
            self.assertEqual(rx_pll["input_signal"], "eth_rx_clock")
            self.assertEqual([output[1:3] for output in rx_pll["clk_out"] if output],
                             [[125e6, 0], [125e6, 0]])
            self.assertIs(design.phy.cd_eth_tx.clk, design.crg.cd_eth_ref.clk)
            self.assertEqual(platform.clks["eth_rx"], design.phy.cd_eth_rx.clk.name_override)
            resets = {special.cd.name: special.async_reset
                      for special in design.phy._fragment.specials
                      if isinstance(special, AsyncResetSynchronizer)}
            self.assertEqual(set(list_signals(resets["eth_rx"])),
                             {design.phy.reset, design.phy.rx_pll.locked})
            self.assertIs(resets["eth_tx"], design.phy.reset)
            ready = next(statement.r for statement in design.crg._fragment.comb
                         if getattr(statement, "l", None) is design.crg.locked)
            self.assertEqual(set(list_signals(ready)),
                             {design.crg.sys_pll.locked, design.crg.eth_pll.locked})
            for index in (0, 1):
                camera = request_camera_hsio(platform, index)
                design.comb += [camera.clock.hs_term.eq(0), camera.clock.hs_ena.eq(0)]
                for lane in camera.data:
                    design.comb += [lane.hs_term.eq(0), lane.hs_ena.eq(0),
                                    lane.rst.eq(1), lane.fifo_rd.eq(0)]
            with tempfile.TemporaryDirectory(prefix="lutcam-periphery-") as directory:
                # run=False still invokes Interface Designer and its checks;
                # it omits synthesis, placement, routing and bitstream creation.
                platform.build(design, build_dir=directory, build_name="board", run=False)
                outflow = Path(directory) / "outflow"
                issues = ET.parse(outflow / "design_issue.xml").getroot()
                self.assertEqual(len(issues), 0)
                report = (outflow / "board.pt.rpt").read_text()
                self.assertIn(f"Timing Model: {timing_model}", report)
                project = ET.parse(Path(directory) / "board.xml").getroot()
                timing = project.find("{http://www.efinixinc.com/enf_proj}device_info/"
                                      "{http://www.efinixinc.com/enf_proj}timing_model")
                self.assertEqual(timing.attrib["name"], timing_model)
                programming_widths = [element.attrib["value"] for element in project.iter()
                                      if element.attrib.get("name") == "width"]
                self.assertEqual(programming_widths, ["4"])
                self.assertIn("GPIOB_P_07.ALT", report)
                for lane in ("GPIOT_PN_06", "GPIOT_PN_07", "GPIOT_PN_09",
                             "GPIOT_PN_13", "GPIOT_PN_14", "GPIOT_PN_17"):
                    self.assertIn(lane, report)
                active_sdc = "\n".join(line for line in (outflow / "board.pt.sdc").read_text().splitlines()
                                       if not line.lstrip().startswith("#"))
                self.assertNotIn("<USER_PERIOD>", active_sdc)
                # Recovered clocks must exist when Interface Designer's
                # peripheral I/O delay commands are parsed.
                rx_clock = design.phy.cd_eth_rx.clk.name_override
                self.assertLess(active_sdc.index("-name " + rx_clock),
                                active_sdc.index("set_input_delay -clock " + rx_clock))
                periphery = ET.parse(Path(directory) / "board.peri.xml").getroot()
                rx_inputs = [element for element in periphery.iter()
                             if element.tag.endswith("input_config")
                             and element.attrib.get("name", "").startswith(("eth0_rx_data", "eth0_rx_ctl"))]
                self.assertEqual(len(rx_inputs), 5)
                self.assertTrue(all(element.attrib["clock_name"] == rx_clock for element in rx_inputs))
                self.assertLess(active_sdc.index("create_clock -name cam0_ck_clk"),
                                active_sdc.index("set_input_delay -clock cam0_ck_clk"))
                self.assertIn("-period 8.333", active_sdc)
                self.assertIn("-period 20.833", active_sdc)
                self.assertNotIn("-period 83.333", active_sdc)
                self.assertIn("create_generated_clock -name " + design.crg.cd_usb_12.clk.name_override
                              + " -source [get_ports {" + design.crg.cd_usb_48.clk.name_override
                              + "}] -divide_by 4 [get_ports {" + design.crg.cd_usb_12.clk.name_override
                              + "}]", active_sdc)
                self.assertIn("-period 8.000", active_sdc)
                self.assertIn("-period 40.000", active_sdc)
                user_sdc = (Path(directory) / "board.sdc").read_text()
                for domain in (design.crg.cd_usb_12, design.crg.cd_eth_ref, design.crg.cd_cam_ref):
                    self.assertIn("set_false_path -from [get_clocks {" + design.crg.cd_sys.clk.name_override
                                  + "}] -to [get_clocks {" + domain.clk.name_override + "}]", user_sdc)
                self.assertNotIn("set_false_path -from [get_clocks {" + design.crg.cd_usb_12.clk.name_override
                                 + "}] -to [get_clocks {" + design.crg.cd_usb_48.clk.name_override + "}]", user_sdc)
        finally:
            os.chdir(previous_cwd)
            LiteXContext.platform = previous_platform


if __name__ == "__main__":
    unittest.main()
