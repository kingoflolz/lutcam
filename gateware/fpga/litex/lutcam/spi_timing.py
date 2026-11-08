"""GD25LQ128EW timing for the software-controlled, system-clocked SPI engine.

Efinity's forwarded GPIO clock model includes the routed SCLK register and
output route through -reference_pin. Its nonregistered GPIO pad delays must
still be added explicitly (Input Forward Clock Delay, GPIO output, Efinity
2025.2 documentation). The receive pipeline consumes at the end of high, two
system cycles after its selected first-stage sample.
"""

import re
import xml.etree.ElementTree as ET


SYS_CLOCK = "sys_pll0_clk"
SPI_CLOCK = "flash_spi_clk"
MISO = "spiflash_miso"
META_D = "flash_miso_meta~FF|D"
SCLK = "spiflash_clk"
BOARD_SKEW_NS = 2.0
FLASH_TCLQV_NS = 7.0
EXTRA_MARGIN_NS = 0.5


def has_spi(peripheral_xml):
    return any(element.get("name") == MISO for element in ET.fromstring(peripheral_xml).iter())


def gpio_delays(report, port, kind):
    for line in report.splitlines():
        fields = [field.strip() for field in line.split("|")[1:-1]]
        if len(fields) == 5 and fields[1:3] == [port, kind]:
            return float(fields[3]), float(fields[4])
    raise ValueError(f"Missing Efinity {kind} delay for {port}")


def make_config(peripheral_xml, peripheral_report, half_period):
    if not has_spi(peripheral_xml):
        return None
    if type(half_period) is not int or half_period < 2:
        raise ValueError("SPI timing requires the controller's actual half period (at least two cycles)")
    sclk_max, sclk_min = gpio_delays(peripheral_report, SCLK, "GPIO_OUT")
    miso_max, miso_min = gpio_delays(peripheral_report, MISO, "GPIO_IN")
    return {
        "half_period_cycles": half_period, "divider": 2 * half_period,
        "receive_capture_cycles": 2 * half_period - 2,
        "input_max_ns": round(sclk_max + miso_max + FLASH_TCLQV_NS + BOARD_SKEW_NS + EXTRA_MARGIN_NS, 3),
        # Zero external minimum is conservative for hold; neither trace nor
        # flash propagation can make data precede the launching SCLK edge.
        "input_min_ns": round(sclk_min + miso_min, 3),
        "gpio_sclk_max_ns": sclk_max, "gpio_miso_max_ns": miso_max,
        "flash_tclqv_max_ns": FLASH_TCLQV_NS, "board_allowance_ns": BOARD_SKEW_NS,
        "additional_margin_ns": EXTRA_MARGIN_NS,
        "cs_high_min_cycles": 2 * half_period + 1,
        "cs_high_requirement_ns": 20.0,
        "gpio_cs_max_ns": gpio_delays(peripheral_report, "spiflash_cs_n", "GPIO_OUT")[0],
        "gpio_cs_min_ns": gpio_delays(peripheral_report, "spiflash_cs_n", "GPIO_OUT")[1],
        "outputs": {port: {
            "max_ns": round(requirement + BOARD_SKEW_NS + gpio_delays(peripheral_report, port, "GPIO_OUT")[0] - sclk_max, 3),
            "min_ns": round(-requirement - BOARD_SKEW_NS + gpio_delays(peripheral_report, port, "GPIO_OUT")[1] - sclk_min, 3),
        } for port, requirement in (("spiflash_mosi", 2.0), ("spiflash_cs_n", 5.0))},
    }


def constraints(config):
    if config is None:
        return []
    capture = config["receive_capture_cycles"]
    half = config["half_period_cycles"]
    lines = [
        f"create_generated_clock -name {SPI_CLOCK} -source [get_ports {{{SYS_CLOCK}}}] "
        f"-divide_by {config['divider']} [get_ports {{{SCLK}}}]",
    ]
    for bound in ("max", "min"):
        lines.append(f"set_input_delay -clock {SPI_CLOCK} -clock_fall "
                     f"-reference_pin [get_ports {{{SCLK}}}] -{bound} "
                     f"{config['input_' + bound + '_ns']:.3f} [get_ports {{{MISO}}}]")
    for phase, cycles in (("setup", capture), ("hold", capture - 1)):
        lines.append(f"set_multicycle_path {cycles} -{phase} -end "
                     f"-from [get_ports {{{MISO}}}] -to [get_pins {{{META_D}}}]")
    for port, delays in config["outputs"].items():
        for bound in ("max", "min"):
            lines.append(f"set_output_delay -clock {SPI_CLOCK} -reference_pin [get_ports {{{SCLK}}}] "
                         f"-{bound} {delays[bound + '_ns']:.3f} [get_ports {{{port}}}]")
        # MOSI changes on falling SCLK. Registered CS assertion precedes the
        # first rise by half+2 cycles, and deassertion follows the last rise
        # by half+1. A half-period setup/hold bound is conservative for both.
        for phase, cycles in (("setup", half), ("hold", 2 * half - 1)):
            lines.append(f"set_multicycle_path {cycles} -{phase} -start "
                         f"-from [get_clocks {{{SYS_CLOCK}}}] -to [get_ports {{{port}}}]")
    return lines


def exceptions(config):
    return {line for line in constraints(config) if line.startswith("set_multicycle_path ")}


def reports(config, build_name):
    if config is None:
        return
    for endpoint in ("miso", *[port.removeprefix("spiflash_") for port in config["outputs"]]):
        for phase in ("setup", "hold"):
            yield endpoint, phase, f"{build_name}.spi-{endpoint}-{phase}.rpt"


def sta_script(config, build_name):
    lines = []
    for endpoint, phase, filename in reports(config, build_name):
        target = (f"-from [get_ports {{{MISO}}}] -to [get_pins {{{META_D}}}]" if endpoint == "miso"
                  else f"-to [get_ports {{spiflash_{endpoint}}}]")
        lines.append(f"report_timing -{phase} {target} -npaths 1 -detail full_path -file {{{filename}}}")
    return "\n".join(lines) + "\n"


def validate_constraints(config, sdc):
    expected = constraints(config)
    lines = [line.strip() for line in sdc.splitlines() if line.strip() and not line.lstrip().startswith("#")]
    for line in expected:
        if lines.count(line) != 1:
            raise ValueError("Missing, changed, or duplicate SPI timing constraint: " + line)
    for line in lines:
        if line.startswith(("set_input_delay ", "set_output_delay ", "set_false_path ",
                            "set_multicycle_path ", "set_min_delay ", "set_max_delay ",
                            "set_clock_groups ")) and any(name in line for name in (MISO, SCLK, META_D, SPI_CLOCK, *config["outputs"])):
            if line not in expected:
                raise ValueError("Unreviewed SPI timing override: " + line)


def validate_reports(config, directory, build_name):
    result = {}
    cs_latencies = {}
    for endpoint, phase, filename in reports(config, build_name):
        report = (directory / filename).read_text()
        expected = {"Path End": META_D if endpoint == "miso" else f"spiflash_{endpoint}",
                    "Launch Clock": SPI_CLOCK + " (FALL)" if endpoint == "miso" else SYS_CLOCK + " (RISE)",
                    "Capture Clock": SYS_CLOCK + " (RISE)" if endpoint == "miso" else SPI_CLOCK + " (RISE)"}
        if endpoint == "miso":
            expected["Path Begin"] = MISO
        for field, value in expected.items():
            match = re.search(r"^" + field + r"\s*:\s*([^\n]+)", report, re.MULTILINE)
            if match is None or match[1].strip() != value:
                raise ValueError(f"SPI {endpoint} {phase} has missing or incorrect {field}")
        def number(field):
            match = re.search(r"^" + field + r"\s*:\s*(-?\d+(?:\.\d+)?)", report, re.MULTILINE)
            if match is None:
                raise ValueError(f"SPI {endpoint} {phase} missing {field}")
            return float(match[1])
        slack = number("Slack")
        if slack < 0:
            raise ValueError(f"SPI {endpoint} {phase} failed: {slack:.3f} ns")
        # A missing propagated generated clock would omit the entire output
        # path while still yielding a deceptively positive input slack.
        if SCLK + "~FF|CLK" not in report and "flash_sclk~FF|CLK" not in report:
            raise ValueError(f"SPI {endpoint} {phase} omitted SCLK register latency")
        expected_cycles = (config["receive_capture_cycles"] if endpoint == "miso" else config["half_period_cycles"])
        expected_constraint = (expected_cycles * 8.333 if phase == "setup" else
                               0.0 if endpoint == "miso" else -expected_cycles * 8.333)
        if abs(number("Constraint") - expected_constraint) > 0.002:
            raise ValueError(f"SPI {endpoint} {phase} uses the wrong capture window")
        result[endpoint + "_" + phase + "_slack_ns"] = slack
        if endpoint == "cs_n":
            # Slow-corner latest edge versus fast-corner earliest edge is a
            # conservative pulse-width bound, including core clock insertion,
            # register CQ, route and package GPIO delay.
            data = re.search(r"Clock To Q \+ Data Path Delay\s*:\s*(\d+(?:\.\d+)?)", report)
            if data is None:
                raise ValueError("SPI CS timing report omitted register/output latency")
            cs_latencies[phase] = number("Launch Clock Path Delay") + float(data[1])
    minimum_high = (config["cs_high_min_cycles"] * 8.333
                    - (cs_latencies["setup"] + config["gpio_cs_max_ns"]
                       - cs_latencies["hold"] - config["gpio_cs_min_ns"])
                    - EXTRA_MARGIN_NS)
    if minimum_high < config["cs_high_requirement_ns"]:
        raise ValueError(f"SPI CS high pulse too short: {minimum_high:.3f} ns")
    result["cs_high_min_ns"] = round(minimum_high, 3)
    return {"passed": True, **config, **result}
