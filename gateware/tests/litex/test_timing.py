"""Fail closed when Efinity drops DDR constraints or reports failing timing."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from lutcam.timing import (TimingError, ddr_path_reports, ddr_sta_script,
                          validate_clock_coverage, validate_ddr_path_report, validate_rgmii_constraints,
                          validate_timing_report)


def peripheral_fixture():
    blocks = []
    constraints = ["create_clock -name rx_clock -period 8 [get_ports {rx_clock}]",
                   "create_clock -name tx_clock -period 8 [get_ports {tx_clock}]"]
    for side, direction in (("rx", "input"), ("tx", "output")):
        for lane in range(5):
            name = f"eth0_{side}_ctl0" if lane == 0 else f"eth0_{side}_data_gen[{lane - 1}]"
            hi = f"eth0_{side}_ctl0_HI" if lane == 0 else f"eth0_{side}_data_gen_HI[{lane - 1}]"
            lo = hi.replace("_HI", "_LO")
            register = 'is_register="true"' if side == "rx" else 'register_option="register"'
            blocks.append(f'<comp_gpio name="{name}"><{direction}_config '
                          f'name="{hi}" name_ddio_lo="{lo}" clock_name="{side}_clock" '
                          f'{register}/></comp_gpio>')
            for bound, delay in (("max", "0.476"), ("min", "0.276")):
                constraints.append(f"set_{direction}_delay -clock {side}_clock "
                                   f"-reference_pin [get_ports {{{side}_clock~CLKOUT~{lane}~1}}] "
                                   f"-{bound} {delay} [get_ports {{{lo} {hi}}}]")
    return "<root>" + "".join(blocks) + "</root>", "\n".join(constraints)


REPORT = """
Timing Model: C3
---------- 2. Clock Relationship Summary (begin) ----------
Setup (Max) Clock Relationship
  Launch Clock    Capture Clock   Constraint (ns)     Slack (ns)          Edge
rx_clock rx_clock 8.000 1.234 (R-R)
tx_clock tx_clock 8.000 2.345 (R-R)
sys_clock sys_clock 8.333 0.177 (R-R)
Hold (Min) Clock Relationship
  Launch Clock    Capture Clock   Constraint (ns)     Slack (ns)          Edge
rx_clock rx_clock 0.000 0.100 (R-R)
tx_clock tx_clock 0.000 0.092 (R-R)
sys_clock sys_clock 0.000 0.027 (R-R)
NOTE: Values are in nanoseconds.
---------- Clock Relationship Summary (end) ---------------
"""


class TimingGateTest(unittest.TestCase):
    def test_complete_ddr_interface_and_report_pass(self):
        ports = validate_rgmii_constraints(*peripheral_fixture())
        self.assertEqual(len(ports), 20)
        rows = validate_timing_report(REPORT, {clock for _, clock in ports.values()})
        self.assertEqual(min(row["slack_ns"] for row in rows["hold"]), 0.027)

    def test_missing_one_ddr_half_is_rejected(self):
        xml, sdc = peripheral_fixture()
        with self.assertRaisesRegex(TimingError, "Missing RGMII"):
            validate_rgmii_constraints(xml, sdc.replace("eth0_rx_data_gen_HI[3]", "missing"))

    def test_missing_entire_interface_is_rejected(self):
        with self.assertRaisesRegex(TimingError, "Expected ten RX"):
            validate_rgmii_constraints("<root/>", "")

    def test_missing_min_delay_is_rejected(self):
        xml, sdc = peripheral_fixture()
        sdc = "\n".join(line for line in sdc.splitlines()
                        if not ("-min " in line and "eth0_tx_ctl0" in line))
        with self.assertRaisesRegex(TimingError, "Missing RGMII"):
            validate_rgmii_constraints(xml, sdc)

    def test_clock_must_exist_before_delay_command(self):
        xml, sdc = peripheral_fixture()
        lines = sdc.splitlines()
        with self.assertRaisesRegex(TimingError, "undefined or incorrect clock"):
            validate_rgmii_constraints(xml, "\n".join(lines[2:] + lines[:2]))

    def test_wrong_clock_or_missing_tap_is_rejected(self):
        xml, sdc = peripheral_fixture()
        for broken in (sdc.replace("-clock rx_clock", "-clock tx_clock"),
                       sdc.replace("rx_clock~CLKOUT~", "other_clock~CLKOUT~")):
            with self.subTest(sdc=broken):
                with self.assertRaises(TimingError):
                    validate_rgmii_constraints(xml, broken)

    def test_overridden_vendor_delay_is_rejected(self):
        xml, sdc = peripheral_fixture()
        with self.assertRaisesRegex(TimingError, "Duplicate or overridden"):
            validate_rgmii_constraints(xml, sdc + "\n" + sdc.splitlines()[-1])

    def test_non_ddr_gpio_cannot_silently_pass(self):
        xml, sdc = peripheral_fixture()
        with self.assertRaisesRegex(TimingError, "must use clocked DDR"):
            validate_rgmii_constraints(xml.replace('is_register="true"', 'is_register="false"'), sdc)

    def test_setup_and_hold_violations_fail_even_outside_ethernet(self):
        for value in ("0.177", "0.027"):
            with self.subTest(slack=value):
                with self.assertRaisesRegex(TimingError, "timing failed"):
                    validate_timing_report(REPORT.replace(value, "-0.001"), ("rx_clock", "tx_clock"))

    def test_missing_rgmii_clock_analysis_is_rejected(self):
        with self.assertRaisesRegex(TimingError, "Missing setup analysis"):
            validate_timing_report(REPORT, ("recovered_rx_pll",))
        with self.assertRaisesRegex(TimingError, "Missing hold analysis"):
            validate_timing_report(REPORT.replace("rx_clock rx_clock 0.000 0.100 (R-R)", ""),
                                   ("rx_clock",))

    def test_wrong_model_truncation_and_unknown_rows_fail_closed(self):
        for report in (REPORT.replace("C3", "C4"), REPORT.split("Hold (Min)")[0],
                       REPORT.replace("0.177", "NaN")):
            with self.subTest(report=report):
                with self.assertRaises(TimingError):
                    validate_timing_report(report)

    def test_async_pairs_allowed_but_broad_or_self_exceptions_rejected(self):
        xml, sdc = peripheral_fixture()
        validate_rgmii_constraints(xml, sdc + "\nset_false_path -from [get_clocks {rx_clock}] "
                                   "-to [get_clocks {sys_clock}]")
        for exception in ("set_false_path -from [get_ports {eth0_rx_ctl0_HI}]",
                          "set_false_path -from [get_clocks {rx_clock}] -to [get_clocks {rx_clock}]",
                          "set_false_path -from [get_clocks {*}] -to [get_clocks {rx_clock}]"):
            with self.subTest(exception=exception):
                with self.assertRaises(TimingError):
                    validate_rgmii_constraints(xml, sdc + "\n" + exception)

    def test_each_ddr_endpoint_gets_setup_and_hold_audit(self):
        ports = validate_rgmii_constraints(*peripheral_fixture())
        reports = list(ddr_path_reports(ports, "test"))
        self.assertEqual(len(reports), 40)
        self.assertEqual(len({path["filename"] for path in reports}), 40)
        script = ddr_sta_script(ports, "test")
        self.assertIn("-from [get_ports {eth0_rx_ctl0_HI}]", script)
        self.assertIn("-to [get_ports {eth0_tx_ctl0_HI}]", script)
        path = {"port": "eth0_rx_ctl0_HI", "direction": "input", "clock": "rx_clock", "analysis": "hold"}
        report = """Path Begin : eth0_rx_ctl0_HI
Launch Clock : rx_clock (RISE)
Capture Clock : rx_clock (RISE)
Slack : 0.020 (arrival time - required time)
"""
        self.assertEqual(validate_ddr_path_report(report, path), 0.020)
        for broken in (report.replace("0.020", "-0.001"), "No path found",
                       report.replace("Capture Clock : rx_clock", "Capture Clock : sys_clock")):
            with self.subTest(report=broken):
                with self.assertRaises(TimingError):
                    validate_ddr_path_report(broken, path)

    def test_unclocked_or_unconstrained_core_registers_fail(self):
        report = """1. checking no_clock (0)
2. checking unconstrained_internal_endpoints (0)
3. checking multiple_clock (0)
"""
        validate_clock_coverage(report)
        for broken in (report.replace("no_clock (0)", "no_clock (1)"),
                       report.replace("endpoints (0)", "endpoints (2)"),
                       report.replace("multiple_clock (0)", "multiple_clock (1)"), ""):
            with self.subTest(report=broken):
                with self.assertRaises(TimingError):
                    validate_clock_coverage(broken)


if __name__ == "__main__":
    unittest.main()
