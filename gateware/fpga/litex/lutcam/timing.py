"""Reject unconstrained RGMII core ports and failing Efinity timing reports.

The DDR HI/LO ports terminate at hard peripheral registers, not package pins.
Their vendor-generated delays describe the peripheral/core boundary. PHY-to-FPGA
electrical setup/hold must be checked separately; substituting the PHY's timing
numbers into these SDC constraints would describe the wrong paths.
"""

import argparse
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from . import spi_timing


class TimingError(ValueError):
    """The build does not have complete, passing timing evidence."""


def rgmii_ports(peripheral_xml):
    """Read both DDR halves and their actual clocks from Interface Designer."""
    root = ET.fromstring(peripheral_xml)
    ports = {}
    counts = {"input": 0, "output": 0}
    for gpio in root.iter():
        if gpio.tag.rsplit("}", 1)[-1] != "comp_gpio":
            continue
        match = re.fullmatch(r"eth\d*_(rx|tx)_(?:ctl\d*|data_gen\[\d+\])",
                             gpio.get("name", ""))
        if match is None:
            continue
        direction = "input" if match[1] == "rx" else "output"
        config = next((child for child in gpio
                       if child.tag.rsplit("}", 1)[-1] == direction + "_config"), None)
        if config is None:
            raise TimingError(f"Missing {direction} configuration for {gpio.get('name')}")
        names = (config.get("name"), config.get("name_ddio_lo"))
        clock = config.get("clock_name")
        registered = (config.get("is_register") == "true" if direction == "input"
                      else config.get("register_option") == "register")
        if not registered or not all(names) or not clock:
            raise TimingError(f"RGMII GPIO {gpio.get('name')} must use clocked DDR registers")
        for name in names:
            if name in ports:
                raise TimingError(f"Duplicate RGMII DDR port {name}")
            ports[name] = (direction, clock.lstrip("~!"))
            counts[direction] += 1
    if counts != {"input": 10, "output": 10}:
        raise TimingError(f"Expected ten RX and ten TX DDR core ports; found {counts}")
    return ports


def _option(command, name):
    match = re.search(r"(?:^|\s)-" + name + r"\s+(?:\{([^}]+)\}|([^\s\[\]]+))", command)
    return None if match is None else (match[1] or match[2])


def validate_rgmii_constraints(peripheral_xml, sdc, allowed_exceptions=()):
    """Require active min/max vendor constraints with defined clocks and taps.

    This deliberately recognizes Efinity's emitted single-line syntax instead
    of pretending to evaluate arbitrary Tcl. A changed generator fails closed.
    """
    ports = rgmii_ports(peripheral_xml)
    rgmii_clocks = {clock for _, clock in ports.values()}
    coverage = {port: set() for port in ports}
    clocks = set()
    for line in sdc.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line in allowed_exceptions:
            continue
        if line.startswith("set_false_path "):
            # The target uses explicit asynchronous clock-pair exceptions.
            # Reject broad/endpoint exceptions that could hide DDR I/O paths.
            exception = re.fullmatch(r"set_false_path -from \[get_clocks \{([^}]+)\}\] "
                                     r"-to \[get_clocks \{([^}]+)\}\]", line)
            if exception is None or any(char in "".join(exception.groups()) for char in "*?[]"):
                raise TimingError("Unreviewed timing exception could mask RGMII paths: " + line)
            source, target = (set(group.split()) for group in exception.groups())
            if source & target & rgmii_clocks:
                raise TimingError("False path removes RGMII synchronous timing: " + line)
        if line.startswith(("set_clock_groups ", "set_multicycle_path ",
                            "set_max_delay ", "set_min_delay ")):
            raise TimingError("Unreviewed timing override could mask RGMII paths: " + line)
        if line.startswith(("create_clock ", "create_generated_clock ")):
            name = _option(line, "name")
            if name:
                clocks.add(name)
        if not line.startswith(("set_input_delay ", "set_output_delay ")):
            continue
        groups = re.findall(r"\[get_ports\s+\{([^}]+)\}\]", line)
        if not groups:
            continue
        targets = groups[-1].split()
        for port in targets:
            if port not in ports:
                continue
            direction, clock = ports[port]
            if not line.startswith(f"set_{direction}_delay "):
                raise TimingError(f"Wrong delay direction for RGMII port {port}")
            if _option(line, "clock") != clock or clock not in clocks:
                raise TimingError(f"RGMII port {port} uses an undefined or incorrect clock {clock}")
            reference = re.search(r"-reference_pin\s+\[get_ports\s+\{([^}]+)\}\]", line)
            if reference is None or not reference[1].startswith(clock + "~CLKOUT~"):
                raise TimingError(f"RGMII port {port} lacks its peripheral clock reference pin")
            for bound in ("min", "max"):
                value = _option(line, bound)
                if value is None:
                    continue
                if re.fullmatch(r"-?\d+(?:\.\d+)?", value) is None:
                    raise TimingError(f"Non-numeric {bound} delay for RGMII port {port}")
                if bound in coverage[port]:
                    raise TimingError(f"Duplicate or overridden {bound} delay for RGMII port {port}")
                coverage[port].add(bound)
    missing = [f"{port}: {','.join(sorted({'min', 'max'} - bounds))}"
               for port, bounds in coverage.items() if bounds != {"min", "max"}]
    if missing:
        raise TimingError("Missing RGMII delay constraints: " + "; ".join(missing))
    return ports


def validate_timing_report(report, required_clocks=()):
    """Check every clock-pair setup/hold result, not only reported critical paths."""
    if re.search(r"^Timing Model:\s+C3\s*$", report, re.MULTILINE) is None:
        raise TimingError("Timing report must use the required C3 model")
    match = re.search(r"Clock Relationship Summary \(begin\).*?\n(.*?)"
                      r"^-+ Clock Relationship Summary \(end\)", report,
                      re.DOTALL | re.MULTILINE)
    if match is None:
        raise TimingError("Missing complete clock relationship summary")
    rows = {"setup": [], "hold": []}
    section = None
    row_pattern = re.compile(r"(\S+)\s+(\S+)\s+(-?\d+(?:\.\d+)?)\s+"
                             r"(-?\d+(?:\.\d+)?)\s+\(([RF]-[RF])\)")
    for line in match[1].splitlines():
        line = line.strip()
        if line == "Setup (Max) Clock Relationship":
            section = "setup"
        elif line == "Hold (Min) Clock Relationship":
            section = "hold"
        elif line and section and not line.startswith(("Launch Clock", "NOTE:")):
            row = row_pattern.fullmatch(line)
            if row is None:
                raise TimingError(f"Unrecognized {section} timing summary row: {line}")
            rows[section].append({"launch": row[1], "capture": row[2],
                                  "constraint_ns": float(row[3]), "slack_ns": float(row[4]),
                                  "edge": row[5]})
    for section, values in rows.items():
        if not values:
            raise TimingError(f"No {section} clock relationships were analyzed")
        for clock in required_clocks:
            if not any(row["launch"] == row["capture"] == clock for row in values):
                raise TimingError(f"Missing {section} analysis for RGMII clock {clock}")
        failures = [f"{row['launch']} -> {row['capture']}: {row['slack_ns']:.3f} ns"
                    for row in values if row["slack_ns"] < 0]
        if failures:
            raise TimingError(f"C3 {section} timing failed: " + "; ".join(failures))
    return rows


def ddr_path_reports(ports, build_name):
    """Generate one setup and hold report per DDR half to prove path coverage."""
    for index, (port, (direction, clock)) in enumerate(sorted(ports.items())):
        for analysis in ("setup", "hold"):
            yield {"port": port, "direction": direction, "clock": clock,
                   "analysis": analysis,
                   "filename": f"{build_name}.rgmii-{index:02d}-{analysis}.rpt"}


def ddr_sta_script(ports, build_name):
    lines = []
    for path in ddr_path_reports(ports, build_name):
        endpoint = "from" if path["direction"] == "input" else "to"
        lines.append(f"report_timing -{path['analysis']} -{endpoint} "
                     f"[get_ports {{{path['port']}}}] -npaths 1 -detail full_path "
                     f"-file {{{path['filename']}}}")
    lines.append(f"check_timing -verbose -file {{{build_name}.check-timing.rpt}}")
    return "\n".join(lines) + "\n"


def validate_ddr_path_report(report, path):
    endpoint = "Begin" if path["direction"] == "input" else "End"
    match = re.search(r"^Path " + endpoint + r"\s*:\s*(\S+)", report, re.MULTILINE)
    if match is None or match[1] != path["port"]:
        raise TimingError(f"No analyzed {path['analysis']} path for DDR port {path['port']}")
    for role in ("Launch", "Capture"):
        match = re.search(r"^" + role + r" Clock\s*:\s*(\S+)", report, re.MULTILINE)
        if match is None or match[1] != path["clock"]:
            raise TimingError(f"Incorrect {role.lower()} clock for DDR port {path['port']}")
    match = re.search(r"^Slack\s*:\s*(-?\d+(?:\.\d+)?)\s", report, re.MULTILINE)
    if match is None:
        raise TimingError(f"Missing {path['analysis']} slack for DDR port {path['port']}")
    slack = float(match[1])
    if slack < 0:
        raise TimingError(f"DDR port {path['port']} {path['analysis']} failed: {slack:.3f} ns")
    return slack


def validate_clock_coverage(report):
    """Do not accept unclocked or unconstrained synchronous core endpoints."""
    checks = ("no_clock", "unconstrained_internal_endpoints", "multiple_clock")
    for check in checks:
        counts = re.findall(r"checking " + check + r" \((\d+)\)", report)
        if not counts:
            raise TimingError(f"Missing Efinity check_timing result: {check}")
        if any(int(count) for count in counts):
            raise TimingError(f"Efinity check_timing failed: {check} ({counts[0]})")


def check_build(gateware_dir, build_name):
    directory = Path(gateware_dir)
    peripheral_xml = (directory / f"{build_name}.peri.xml").read_text()
    sdc = (directory / f"{build_name}_merged.sdc").read_text()
    spi = None
    if spi_timing.has_spi(peripheral_xml):
        spi = json.loads((directory / "outflow" / f"{build_name}.spi-timing-config.json").read_text())
        expected_spi = spi_timing.make_config(peripheral_xml,
            (directory / "outflow" / f"{build_name}.pt_timing.rpt").read_text(), spi["half_period_cycles"])
        if spi != expected_spi:
            raise TimingError("SPI timing configuration differs from peripheral timing report")
        spi_timing.validate_constraints(spi, sdc)
    ports = validate_rgmii_constraints(
        peripheral_xml, sdc, spi_timing.exceptions(spi))
    clocks = sorted({clock for _direction, clock in ports.values()})
    rows = validate_timing_report(
        (directory / "outflow" / f"{build_name}.timing.rpt").read_text(), clocks)
    validate_clock_coverage(
        (directory / "outflow" / f"{build_name}.check-timing.rpt").read_text())
    ddr_paths = []
    for path in ddr_path_reports(ports, build_name):
        report = (directory / "outflow" / path["filename"]).read_text()
        ddr_paths.append({**path, "slack_ns": validate_ddr_path_report(report, path)})
    result = {"passed": True, "timing_model": "C3", "rgmii_ddr_core_ports": len(ports),
            "rgmii_clocks": clocks,
            "worst_setup_slack_ns": min(row["slack_ns"] for row in rows["setup"]),
            "worst_hold_slack_ns": min(row["slack_ns"] for row in rows["hold"]),
            "scope": "Routed core timing and hard-DDR/core boundary; external PHY timing is separate",
            "clock_relationships": rows, "rgmii_ddr_paths": ddr_paths}
    if spi:
        result["spi_flash"] = spi_timing.validate_reports(spi, directory / "outflow", build_name)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gateware_dir", type=Path)
    parser.add_argument("--build-name", default="lutcam")
    args = parser.parse_args()
    print(json.dumps(check_build(args.gateware_dir, args.build_name), indent=2))


if __name__ == "__main__":
    main()
