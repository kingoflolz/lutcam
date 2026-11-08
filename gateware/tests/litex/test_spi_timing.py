"""SPI external timing contracts include physical pads and exact capture phase."""

from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))
from lutcam import spi_timing
from lutcam.timing import TimingError, validate_rgmii_constraints
from tests.litex.test_timing import peripheral_fixture


XML = '<root><comp_gpio name="spiflash_miso"/></root>'
REPORT = """
| spiflash_clk | spiflash_clk | GPIO_OUT | 2.536 | 1.470 |
| spiflash_miso | spiflash_miso | GPIO_IN | 0.953 | 0.552 |
| spiflash_mosi | spiflash_mosi | GPIO_OUT | 2.536 | 1.470 |
| spiflash_cs_n | spiflash_cs_n | GPIO_OUT | 2.536 | 1.470 |
"""


class SPITimingTest(unittest.TestCase):
    def config(self, half=2):
        return spi_timing.make_config(XML, REPORT, half)

    def test_read_budget_includes_both_pads_flash_board_and_margin(self):
        config = self.config()
        self.assertEqual(config["input_max_ns"], 12.989)
        self.assertEqual(config["input_min_ns"], 2.022)
        self.assertEqual(config["receive_capture_cycles"], 2)
        lines = spi_timing.constraints(config)
        self.assertIn("-divide_by 4 [get_ports {spiflash_clk}]", lines[0])
        self.assertIn("-clock_fall -reference_pin [get_ports {spiflash_clk}]", lines[1])
        self.assertIn("set_multicycle_path 2 -setup -end -from [get_ports {spiflash_miso}] "
                      "-to [get_pins {flash_miso_meta~FF|D}]", lines)

    def test_output_half_period_setup_and_hold_are_explicit(self):
        config = self.config()
        lines = spi_timing.constraints(config)
        self.assertIn("set_multicycle_path 2 -setup -start -from [get_clocks {sys_pll0_clk}] "
                      "-to [get_ports {spiflash_mosi}]", lines)
        self.assertIn("set_multicycle_path 3 -hold -start -from [get_clocks {sys_pll0_clk}] "
                      "-to [get_ports {spiflash_mosi}]", lines)
        self.assertEqual(config["outputs"]["spiflash_mosi"], {"max_ns": 4.0, "min_ns": -4.0})

    def test_real_divider_and_pipeline_latency_drive_constraints(self):
        config = self.config(12)
        self.assertEqual(config["divider"], 24)
        self.assertEqual(config["receive_capture_cycles"], 22)
        for half in (None, 1, 2.0):
            with self.assertRaises(ValueError):
                self.config(half)
        self.assertIsNone(spi_timing.make_config("<root/>", "", None))

    def test_changed_or_missing_constraint_fails(self):
        config = self.config()
        sdc = "\n".join(spi_timing.constraints(config))
        spi_timing.validate_constraints(config, sdc)
        for broken in (sdc.replace("12.989", "1.000"), sdc + "\n" + sdc.splitlines()[1],
                       sdc + "\nset_false_path -from [get_ports {spiflash_miso}]"):
            with self.assertRaises(ValueError):
                spi_timing.validate_constraints(config, broken)

    def test_only_exact_spi_exceptions_bypass_rgmii_guard(self):
        xml, sdc = peripheral_fixture()
        config = self.config()
        complete = sdc + "\n" + "\n".join(spi_timing.constraints(config))
        validate_rgmii_constraints(xml, complete, spi_timing.exceptions(config))
        with self.assertRaises(TimingError):
            validate_rgmii_constraints(xml, complete.replace("-setup -end", "-setup -start"),
                                       spi_timing.exceptions(config))

    def test_no_missing_pad_characterization_is_accepted(self):
        with self.assertRaises(ValueError):
            spi_timing.make_config(XML, REPORT.replace("GPIO_IN", "WRONG"), 2)

    def test_real_audit_checks_latency_capture_edge_and_slack(self):
        config = self.config()
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for endpoint, phase, filename in spi_timing.reports(config, "test"):
                is_input = endpoint == "miso"
                constraint = (16.666 if phase == "setup" else 0 if is_input else -16.666)
                text = (f"Path Begin : {'spiflash_miso' if is_input else 'spiflash_mosi~FF|CLK'}\n"
                        f"Path End : {'flash_miso_meta~FF|D' if is_input else 'spiflash_' + endpoint}\n"
                        f"Launch Clock : {'flash_spi_clk (FALL)' if is_input else 'sys_pll0_clk (RISE)'}\n"
                        f"Capture Clock : {'sys_pll0_clk (RISE)' if is_input else 'flash_spi_clk (RISE)'}\n"
                        f"Slack : 1.357\nConstraint : {constraint}\nspiflash_clk~FF|CLK\n"
                        "Launch Clock Path Delay : 2.0\n+ Clock To Q + Data Path Delay : 1.0\n")
                (directory / filename).write_text(text)
            self.assertTrue(spi_timing.validate_reports(config, directory, "test")["passed"])
            path = directory / "test.spi-miso-setup.rpt"
            original = path.read_text()
            for broken in (original.replace("1.357", "-0.001"),
                           original.replace("16.666", "33.332"),
                           original.replace("spiflash_clk~FF|CLK", "missing")):
                path.write_text(broken)
                with self.assertRaises(ValueError):
                    spi_timing.validate_reports(config, directory, "test")


if __name__ == "__main__":
    unittest.main()
