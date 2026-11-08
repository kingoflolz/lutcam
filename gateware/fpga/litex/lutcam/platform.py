"""Lutcam Ti60F225 board resources, checked against the hardware netlist.

The camera lanes are hard MIPI resources, not generic differential GPIOs.
The older standalone Efinity timing shells intentionally use a different pinout.
All builds use C3 timing, including the board whose BOM specifies C4.
"""

from pathlib import Path
import json
import re
import subprocess
from types import SimpleNamespace

from litex.build.efinix import EfinixProgrammer
from litex.build.efinix.efinity import EfinityToolchain
from litex.build.efinix.platform import EfinixPlatform
from litex.build.generic_platform import IOStandard, Pins, Subsignal

from .timing import check_build, ddr_path_reports, ddr_sta_script, validate_rgmii_constraints
from . import spi_timing


# GPIO balls wired directly to ground. Never expose these as spare output pins.
GROUNDED_GPIO_PINS = frozenset("A13 B6 E15 F7 F8 G13 G5 H11 J10 K8 L12 M6".split())

# Logical sensor lane order is deliberately different from physical lane order
# for camera 1 (I5 data lanes 1 and 4).
CAMERA_HSIO = (
    {"clock": "GPIOT_PN_09", "data": ("GPIOT_PN_06", "GPIOT_PN_07"),
     "clock_pins": ("E8", "D8"), "data_pins": (("B7", "A7"), ("B8", "A8"))},
    {"clock": "GPIOT_PN_14", "data": ("GPIOT_PN_13", "GPIOT_PN_17"),
     "clock_pins": ("A10", "B10"), "data_pins": (("C10", "D10"), ("A11", "B11"))},
)

_io = [
    ("clk25", 0, Pins("P2"), IOStandard("1.8_V_LVCMOS")),
    ("eth_clocks", 0,
        Subsignal("rx", Pins("R8")), Subsignal("tx", Pins("L9")),
        IOStandard("1.8_V_LVCMOS")),
    ("eth", 0,
        Subsignal("rx_ctl", Pins("P5")),
        Subsignal("rx_data", Pins("R6 P6 R7 P7")),
        Subsignal("tx_ctl", Pins("R12")),
        Subsignal("tx_data", Pins("R11 R10 R9 P9")),
        Subsignal("mdio", Pins("P8")),
        Subsignal("mdc", Pins("R13"), IOStandard("3.3_V_LVCMOS")),
        Subsignal("rst_n", Pins("R14"), IOStandard("3.3_V_LVCMOS")),
        IOStandard("1.8_V_LVCMOS")),
    ("usb", 0,
        Subsignal("d_p", Pins("P14")), Subsignal("d_n", Pins("P15")),
        Subsignal("pullup", Pins("N14")), Subsignal("vbus_valid", Pins("P13")),
        IOStandard("3.3_V_LVCMOS")),
    ("camera", 0,
        Subsignal("sda", Pins("B1")), Subsignal("scl", Pins("A2")),
        Subsignal("shutdown", Pins("C1")), Subsignal("xvclk", Pins("B2")),
        Subsignal("fsin", Pins("C2")), Subsignal("strobe", Pins("D1")),
        IOStandard("1.8_V_LVCMOS")),
    ("camera", 1,
        Subsignal("sda", Pins("J15")), Subsignal("scl", Pins("K14")),
        Subsignal("shutdown", Pins("H15")), Subsignal("xvclk", Pins("K15")),
        Subsignal("fsin", Pins("J14")), Subsignal("strobe", Pins("H14")),
        IOStandard("1.8_V_LVCMOS")),
    ("user_led", 0, Pins("M14"), IOStandard("1.8_V_LVCMOS")),
    ("user_led", 1, Pins("M15"), IOStandard("1.8_V_LVCMOS")),
    # U7 is external flash. Configuration pins become GPIO in user mode;
    # the SPI_FLASH interface block is for packages with flash in the SiP.
    ("spiflash", 0,
        Subsignal("clk", Pins("N1")), Subsignal("cs_n", Pins("P1")),
        Subsignal("mosi", Pins("M1")), Subsignal("miso", Pins("L1")),
        Subsignal("wp", Pins("H5")), Subsignal("hold", Pins("H4")),
        IOStandard("1.8_V_LVCMOS")),
]

IOBANK_INFO = [
    (name, voltage + " V LVCMOS") for name, voltage in (
        ("1A", "1.8"), ("1B", "1.8"), ("2A", "1.2"), ("2B", "1.2"),
        ("3A", "1.8"), ("3B", "1.8"), ("4A", "1.8"), ("4B", "1.8"),
        ("BL", "1.8"), ("BR", "3.3"), ("TL", "1.8"), ("TR", "1.8"))
]


def _check_resources(resources):
    """Reject extensions that could drive grounded GPIOs."""
    for resource in resources:
        constraints = list(resource[2:])
        while constraints:
            constraint = constraints.pop()
            if isinstance(constraint, Subsignal):
                constraints.extend(constraint.constraints)
            elif isinstance(constraint, Pins):
                forbidden = GROUNDED_GPIO_PINS.intersection(constraint.identifiers)
                if forbidden:
                    raise ValueError("Board GPIO tied to ground: " + ", ".join(sorted(forbidden)))


class _LutcamToolchain(EfinityToolchain):
    def build_project(self):
        super().build_project()
        # Efinity 2025.2 emits active <USER_PERIOD> templates for recovered
        # clocks. LiteX appends its constraints after this file, so leaving
        # those templates active would break STA before the real clocks.
        path = Path("outflow") / f"{self._build_name}.pt.sdc"
        clocks = {name or self._vns.get_name(signal)
                  for signal, (_period, name) in self.clocks.items()}
        related = {self._vns.get_name(target): (self._vns.get_name(source), divide)
                   for source, target, divide in self.platform.related_clocks}
        lines = []
        for line in path.read_text().splitlines(keepends=True):
            if not line.lstrip().startswith("#") and "<USER_PERIOD>" in line:
                match = re.search(r"-name\s+([^\s]+)", line)
                if match is None or match.group(1) not in clocks:
                    raise ValueError(f"Unconstrained Efinity peripheral clock: {line.strip()}")
                line = "# Clock constrained by LiteX: " + line
            elif line.lstrip().startswith("create_clock "):
                match = re.search(r"-name\s+([^\s]+)", line)
                if match is not None and match.group(1) in related:
                    target = match.group(1)
                    source, divide = related[target]
                    # Independently rounded PLL periods (20.833/83.333 ns)
                    # create a fictitious 1 ps beat period in STA. Express the
                    # real divider relationship, preserving interclock timing.
                    line = (f"create_generated_clock -name {target} "
                            f"-source [get_ports {{{source}}}] -divide_by {divide} "
                            f"[get_ports {{{target}}}]\n")
            lines.append(line)
        # LiteX merges peripheral constraints before its own SDC. Recovered
        # clocks must nevertheless exist *before* peripheral input/output
        # delays refer to them; Efinity otherwise drops those constraints.
        user_path = Path(f"{self._build_name}.sdc")
        user_lines = user_path.read_text().splitlines(keepends=True)
        clock_lines = [line for line in user_lines if line.lstrip().startswith("create_clock ")]
        user_lines = [line for line in user_lines if not line.lstrip().startswith("create_clock ")]
        path.write_text("# Board clock definitions precede peripheral delays.\n"
                        + "".join(clock_lines) + "\n" + "".join(lines))
        user_path.write_text("".join(user_lines))
        peripheral_xml = Path(f"{self._build_name}.peri.xml").read_text()
        self.spi_timing = spi_timing.make_config(peripheral_xml,
            (Path("outflow") / f"{self._build_name}.pt_timing.rpt").read_text(),
            getattr(self.platform, "spi_flash_half_period", None))
        if self.spi_timing:
            user_path.write_text(user_path.read_text() + "\n# Forwarded SPI clock and external flash timing.\n"
                                 + "\n".join(spi_timing.constraints(self.spi_timing)) + "\n")
            (Path("outflow") / f"{self._build_name}.spi-timing-config.json").write_text(
                json.dumps(self.spi_timing, indent=2) + "\n")
        # These are the hard DDR registers' core-facing ports. Preserve the
        # vendor delays and clock reference taps; external PHY setup/hold is a
        # separate board-level calculation, not a replacement for these SDCs.
        merged_sdc = path.read_text() + "\n" + user_path.read_text()
        validate_rgmii_constraints(peripheral_xml, merged_sdc, spi_timing.exceptions(self.spi_timing))
        if self.spi_timing:
            spi_timing.validate_constraints(self.spi_timing, merged_sdc)

    def run_script(self, script):
        result_path = Path("outflow") / f"{self._build_name}.timing-check.json"
        result_path.unlink(missing_ok=True)
        super().run_script(script)
        ports = validate_rgmii_constraints(Path(f"{self._build_name}.peri.xml").read_text(),
                                           Path(f"{self._build_name}_merged.sdc").read_text(),
                                           spi_timing.exceptions(self.spi_timing))
        # A clock-pair summary can pass even when an individual I/O constraint
        # was dropped. Audit every DDR endpoint using the routed design.
        for report in ddr_path_reports(ports, self._build_name):
            (Path("outflow") / report["filename"]).unlink(missing_ok=True)
        (Path("outflow") / f"{self._build_name}.check-timing.rpt").unlink(missing_ok=True)
        for _endpoint, _phase, filename in spi_timing.reports(self.spi_timing, self._build_name):
            (Path("outflow") / filename).unlink(missing_ok=True)
        audit_script = Path(f"{self._build_name}.rgmii-timing.tcl")
        audit_script.write_text(ddr_sta_script(ports, self._build_name)
                               + spi_timing.sta_script(self.spi_timing, self._build_name))
        audit_log = Path("outflow") / f"{self._build_name}.rgmii-timing.log"
        with audit_log.open("w") as output:
            result = subprocess.run([
                str(Path(self.efinity_path) / "bin" / "python3"),
                str(Path(self.efinity_path) / "scripts" / "efx_run.py"),
                f"{self._build_name}.xml", "--flow", "sta_tclsh",
                "--tcl_script", str(audit_script),
            ], env=self.env, stdout=output, stderr=subprocess.STDOUT)
        if result.returncode:
            raise OSError(f"RGMII routed timing audit failed; see {audit_log.resolve()}")
        # Efinity may generate programming files despite timing violations.
        # A successful board build must have both setup and hold closure.
        result = check_build(Path.cwd(), self._build_name)
        result_path.write_text(json.dumps(result, indent=2) + "\n")
        print(f"C3 timing verified: setup {result['worst_setup_slack_ns']:.3f} ns, "
              f"hold {result['worst_hold_slack_ns']:.3f} ns; "
              f"{result['rgmii_ddr_core_ports']} RGMII DDR core ports constrained")


class Platform(EfinixPlatform):
    default_clk_name = "clk25"
    default_clk_period = 40.0

    def __init__(self, toolchain="efinity", *, timing_model="C3"):
        if timing_model != "C3":
            raise ValueError("Lutcam builds require C3 timing")
        _check_resources(_io)
        # U7 GD25LQ128EWIGR connects IO0..IO3 to the configuration bus.
        # C3 is the required timing model for this board's complete design.
        super().__init__("Ti60F225C3", _io, iobank_info=IOBANK_INFO,
                         toolchain=toolchain, spi_mode="active", spi_width="4")
        self.toolchain = _LutcamToolchain(self.efinity_path)
        self._camera_hsio = {}
        self._rom_files = {}
        self.related_clocks = []

    def add_rom_file(self, path):
        """Package a ROM as a local data file in the generated HDL output."""
        path = Path(path)
        contents = path.read_text(encoding="ascii")
        previous = self._rom_files.get(path.name)
        if previous is not None and previous != contents:
            raise ValueError(f"Conflicting ROM contents for {path.name}")
        self._rom_files[path.name] = contents

    def get_verilog(self, *args, **kwargs):
        output = super().get_verilog(*args, **kwargs)
        for filename, contents in self._rom_files.items():
            if filename in output.data_files:
                if output.data_files[filename] != contents:
                    raise ValueError(f"Generated data file conflicts with ROM {filename}")
            else:
                output.add_data_file(filename, contents)
        return output

    def add_extension(self, io, *args, **kwargs):
        _check_resources(io)
        return super().add_extension(io, *args, **kwargs)

    def create_programmer(self):
        return EfinixProgrammer()

    def do_finalize(self, fragment):
        super().do_finalize(fragment)
        # PLL input constraints are generated by Efinity's peripheral flow.


def request_camera_hsio(platform, index, data_delay=5, byte_clk_freq=100e6):
    """Return hard RX signals in *logical sensor lane order*.

    Result: ``clock.{clk,lp_p,lp_n,hs_term,hs_ena}`` and two
    ``data`` entries with ``hs_in[7:0],lp_p,lp_n,fifo_empty,fifo_rd,rst,
    hs_term,hs_ena``. Inputs from the hard PHY have no GPIO assignment; the
    Interface Designer lane blocks own the real balls. The hard lane FIFO
    read clocks auto-connect to this camera's clock lane.
    """
    if index not in (0, 1):
        raise ValueError("Lutcam has camera indices 0 and 1")
    if index in platform._camera_hsio:
        raise ValueError(f"Camera {index} HSIO already requested")
    if not 0 <= data_delay <= 31:
        raise ValueError("MIPI static data delay must be in 0..31")
    mapping = CAMERA_HSIO[index]

    def port(name, width=1):
        return platform.add_iface_io(name, width)

    clock_name = f"cam{index}_ck"
    clock = SimpleNamespace(**{field: port(f"{clock_name}_{field}") for field in
        ("clk", "lp_p", "lp_n", "hs_term", "hs_ena")})
    platform.toolchain.ifacewriter.blocks.append({
        "type": "MIPI_RX_LANE", "name": clock_name,
        "mode": "CLOCK_LANE", "conn_type": "GCLK", "ressource": mapping["clock"],
        "props": {
            "DELAY": "0", "DELAY_MODE": "STATIC", "FIFO": "0", "REVERSIBLE": "0",
            "CLKOUT_PIN": clock.clk.name_override,
            "LP_P_IN_PIN": clock.lp_p.name_override, "LP_N_IN_PIN": clock.lp_n.name_override,
            "HS_TERM_PIN": clock.hs_term.name_override, "HS_ENA_PIN": clock.hs_ena.name_override,
        },
    })
    platform.add_period_constraint(clock.clk, 1e9 / byte_clk_freq)

    data = []
    for lane, resource in enumerate(mapping["data"]):
        name = f"cam{index}_d{lane}"
        signals = SimpleNamespace(**{field: port(f"{name}_{field}", width) for field, width in (
            ("hs_in", 8), ("lp_p", 1), ("lp_n", 1), ("fifo_empty", 1),
            ("fifo_rd", 1), ("rst", 1), ("hs_term", 1), ("hs_ena", 1))})
        props = {"DELAY": str(data_delay), "DELAY_MODE": "STATIC", "FIFO": "1", "REVERSIBLE": "0"}
        for prop, field in (
            ("HS_IN_PIN", "hs_in"), ("LP_P_IN_PIN", "lp_p"), ("LP_N_IN_PIN", "lp_n"),
            ("FIFO_EMPTY_PIN", "fifo_empty"), ("FIFO_RD_PIN", "fifo_rd"), ("RST_PIN", "rst"),
            ("HS_TERM_PIN", "hs_term"), ("HS_ENA_PIN", "hs_ena")):
            props[prop] = getattr(signals, field).name_override
        platform.toolchain.ifacewriter.blocks.append({
            "type": "MIPI_RX_LANE", "name": name, "mode": "DATA_LANE",
            "ressource": resource, "props": props,
        })
        data.append(signals)

    result = SimpleNamespace(clock=clock, data=tuple(data))
    platform._camera_hsio[index] = result
    return result
