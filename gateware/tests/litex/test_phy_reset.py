"""Keep the external PHY alive across FPGA startup and local PLL recovery."""

from types import SimpleNamespace
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from migen import ClockDomain, Module, Signal
from migen.fhdl.specials import Tristate
from migen.sim import run_simulation
from litex.build.efinix.toolchain import find_efinity_path
from litex.gen import LiteXContext, LiteXModule

from fpga.litex.lutcam import crg


class _PLL(LiteXModule):
    """Expose lock/reset pins without simulating the vendor PLL primitive."""

    def __init__(self, platform):
        self.reset = Signal()
        self.locked = Signal()

    def register_clkin(self, clock, frequency):
        pass

    def create_clkout(self, domain, frequency, **kwargs):
        pass


class _Datapath(LiteXModule):
    def __init__(self, *args):
        self.sink = Signal()
        self.source = Signal()


class PHYResetTests(unittest.TestCase):
    def test_peripheral_uses_constant_low_output_and_csr_enable(self):
        try:
            find_efinity_path()
        except OSError:
            self.skipTest("Set LITEX_ENV_EFINITY to run real peripheral validation")
        from fpga.litex.lutcam.platform import Platform

        previous_platform, previous_cwd = LiteXContext.platform, Path.cwd()
        try:
            platform = Platform()
            LiteXContext.platform = platform
            design = LiteXModule()
            design.crg = crg.LutcamCRG(platform)
            design.phy = crg.LutcamRGMII(platform, platform.request("eth_clocks"),
                                       platform.request("eth"), design.crg)
            with tempfile.TemporaryDirectory(prefix="lutcam-phy-reset-") as directory:
                platform.build(design, build_dir=directory, build_name="reset", run=False)
                periphery = ET.parse(Path(directory) / "reset.peri.xml").getroot()
                gpio, = (element for element in periphery.iter()
                         if element.tag.endswith("comp_gpio")
                         and element.attrib.get("gpio_def") == "GPIOR_28")
                self.assertEqual(gpio.attrib["mode"], "inout")
                self.assertEqual(gpio.attrib["gpio_def"], "GPIOR_28")
                output, = (child for child in gpio if child.tag.endswith("output_config"))
                self.assertEqual(output.attrib["tied_option"], "__gnd__")
                enable, = (child for child in gpio
                           if child.tag.endswith("output_enable_config"))
                self.assertEqual(enable.attrib["name"], gpio.attrib["name"] + "_OE")
                self.assertEqual(enable.attrib["is_register"], "false")
                generated = (Path(directory) / "reset.v").read_text()
                self.assertRegex(generated, "assign " + enable.attrib["name"] +
                                 r" = \w*reset_storage;")
                self.assertEqual(len(ET.parse(Path(directory) / "outflow" /
                                             "design_issue.xml").getroot()), 0)
        finally:
            os.chdir(previous_cwd)
            LiteXContext.platform = previous_platform

    def test_startup_and_lock_loss_preserve_phy_but_manual_reset_does_not(self):
        platform = SimpleNamespace(
            toolchain=SimpleNamespace(
                ifacewriter=SimpleNamespace(blocks=[]), excluded_ios=[]),
            clks={},
            add_iface_io=lambda name: Signal(name_override=name),
            get_pin_location=lambda pin: ["R8"],
            get_pin_properties=lambda pin: {},
            add_period_constraint=lambda *args: None,
            add_false_path_constraint=lambda *args: None,
        )
        clocks = SimpleNamespace(rx=Signal(), tx=Signal())
        pads = SimpleNamespace(rst_n=Signal())
        parent = SimpleNamespace(locked=Signal(),
                                 cd_eth_ref=ClockDomain("eth_ref"),
                                 cd_sys=ClockDomain("sys"))
        with patch.object(crg, "TITANIUMPLL", _PLL), \
                patch.object(crg, "LiteEthPHYRGMIITX", _Datapath), \
                patch.object(crg, "LiteEthPHYRGMIIRX", _Datapath), \
                patch.object(crg, "LiteEthPHYMDIO", _Datapath):
            phy = crg.LutcamRGMII(platform, clocks, pads, parent, hw_reset_cycles=4)

        reset_driver, = (special for special in phy._fragment.specials
                        if isinstance(special, Tristate))
        self.assertIs(reset_driver.target, pads.rst_n)
        self.assertEqual(reset_driver.o.value, 0)
        self.assertIs(reset_driver.oe, phy._reset.storage)
        self.assertEqual(reset_driver.oe.reset.value, 0)

        # Simulate the actual reset expressions and startup timer. The DDR
        # primitives and asynchronous domain synchronizers are covered by the
        # existing real-platform test; they are not vendor behavioral models.
        dut = Module()
        dut.submodules.startup = phy.hw_reset
        dut.comb += phy._fragment.comb
        # Board R63 pulls the released pad high; enabling the constant-low
        # output asserts reset. Simulate that electrical truth table explicitly.
        dut.comb += pads.rst_n.eq(~reset_driver.oe)

        def bench():
            yield parent.locked.eq(1)
            yield
            self.assertEqual((yield pads.rst_n), 1)
            self.assertEqual((yield phy.reset), 1)
            self.assertEqual((yield phy.rx_pll.reset), 1)
            for _ in range(6):
                self.assertEqual((yield pads.rst_n), 1)
                yield
            self.assertEqual((yield phy.reset), 0)
            self.assertEqual((yield phy.rx_pll.reset), 0)

            # Local clock failure must reset the MAC and RX PLL, while keeping
            # the already-negotiated copper link and PHY configuration alive.
            yield parent.locked.eq(0)
            yield
            self.assertEqual((yield pads.rst_n), 1)
            self.assertEqual((yield phy.reset), 1)
            self.assertEqual((yield phy.rx_pll.reset), 1)
            yield parent.locked.eq(1)
            yield
            self.assertEqual((yield phy.reset), 0)
            self.assertEqual((yield pads.rst_n), 1)

            yield phy._reset.storage.eq(1)
            yield
            self.assertEqual((yield pads.rst_n), 0)
            self.assertEqual((yield phy.reset), 1)
            self.assertEqual((yield phy.rx_pll.reset), 1)
            yield phy._reset.storage.eq(0)
            yield
            self.assertEqual((yield pads.rst_n), 1)
            self.assertEqual((yield phy.reset), 0)
            self.assertEqual((yield phy.rx_pll.reset), 0)

        run_simulation(dut, bench())


if __name__ == "__main__":
    unittest.main()
