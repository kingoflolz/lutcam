"""Oscillator-based system/USB clocks and independently recovered RGMII RX."""

from migen import ClockDomain, ClockDomainsRenamer, ClockSignal, Constant, Signal
from migen.fhdl.specials import Tristate
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.io import ClkOutput
from litex.gen import LiteXModule
from litex.soc.cores.clock import TITANIUMPLL
from litex.soc.interconnect.csr import CSRStorage
from liteeth.phy.common import LiteEthPHYHWReset, LiteEthPHYMDIO
from liteeth.phy.titaniumrgmii import LiteEthPHYRGMIIRX, LiteEthPHYRGMIITX


class LutcamCRG(LiteXModule):
    """120 MHz processing and phase-related 48/12 MHz USB clocks.

    Both PLLs depend only on the board oscillator. Ethernet link loss never
    stops Wishbone, USB, or either sensor clock. The second PLL makes exact
    125/25 MHz clocks because those cannot share the USB/120 MHz divider set.
    """
    def __init__(self, platform, sys_clk_freq=120e6):
        if sys_clk_freq != 120e6:
            raise ValueError("The camera processing cadence requires 120 MHz")
        self.rst = Signal()
        self.cd_sys = ClockDomain("sys")
        self.cd_usb_12 = ClockDomain("usb_12")
        self.cd_usb_48 = ClockDomain("usb_48")
        self.cd_pll_ref = ClockDomain("pll_ref", reset_less=True)
        self.cd_eth_pll_ref = ClockDomain("eth_pll_ref", reset_less=True)
        self.cd_eth_ref = ClockDomain("eth_ref", reset_less=True)
        self.cd_cam_ref = ClockDomain("cam_ref", reset_less=True)

        # P2 is the fixed PLL_BL0 reference. Allocate it before other PLLs.
        self.sys_pll = TITANIUMPLL(platform)
        self.sys_pll.register_clkin(platform.request("clk25"), 25e6)
        # Keep Titanium's local feedback on CLKOUT0 at the vendor API's
        # default 100 MHz. The API preserves that feedback target while
        # computing exact integer dividers for the remaining outputs.
        self.sys_pll.create_clkout(self.cd_pll_ref, 100e6, with_reset=False)
        self.sys_pll.create_clkout(self.cd_sys, 120e6)
        self.sys_pll.create_clkout(self.cd_usb_48, 48e6)
        self.sys_pll.create_clkout(self.cd_usb_12, 12e6)
        platform.related_clocks.append((self.cd_usb_48.clk, self.cd_usb_12.clk, 4))
        self.comb += self.sys_pll.reset.eq(self.rst)

        self.eth_pll = TITANIUMPLL(platform)
        self.eth_pll.register_clkin(self.cd_pll_ref.clk, 100e6)
        self.eth_pll.create_clkout(self.cd_eth_pll_ref, 100e6, with_reset=False)
        self.eth_pll.create_clkout(self.cd_eth_ref, 125e6, with_reset=False)
        self.eth_pll.create_clkout(self.cd_cam_ref, 25e6, with_reset=False)
        self.comb += self.eth_pll.reset.eq(self.rst | ~self.sys_pll.locked)
        self.locked = Signal()
        self.comb += self.locked.eq(self.sys_pll.locked & self.eth_pll.locked)

        # These crossings use asynchronous stream FIFOs (USB/Ethernet) or
        # two-register synchronizers (sensor clock enable and USB diagnostics).
        # Do not time synchronizer inputs as ordinary synchronous data paths. Retain
        # timing between USB12 and USB48, which LUNA deliberately relates.
        for domain in (self.cd_usb_12, self.cd_usb_48, self.cd_eth_ref, self.cd_cam_ref):
            platform.add_false_path_constraint(self.cd_sys.clk, domain.clk)


class LutcamRGMII(LiteXModule):
    """LiteEth PHY with board-validated RTL8211F receive-clock timing.

    A dedicated recovered RX PLL provides the hardware-validated zero-degree
    receive clock with the RTL8211F's strapped RX delay enabled. TX uses
    the oscillator-based 125 MHz reference and retains the strapped TX delay.
    Ethernet link loss affects only the RX clock and its local reset.
    The design supports a 1000BASE-T full-duplex link only.

    PHYRSTB is driven low only for manual reset; R63 holds it high otherwise,
    including while FPGA I/O is tri-stated during reconfiguration. The
    independent 25 MHz oscillator keeps running. Cold startup relies on the RTL8211F internal
    POR: its external 1.8 V I/O rail must become ready within 5 ms of 3.3 V
    (datasheet section 9.3, Case I). U2 derives that rail from 3.3 V; cold
    power-on still needs hardware validation. Manual recovery through the
    reset CSR remains available over USB: assert for at least 10 ms, release,
    then wait at least 50 ms before MDIO register access (section 7.17).
    """
    dw = 8
    tx_clk_freq = 125e6
    rx_clk_freq = 125e6

    def __init__(self, platform, clock_pads, pads, crg, hw_reset_cycles=1_200_000):
        self.cd_eth_rx = ClockDomain("eth_rx")
        self.cd_eth_tx = ClockDomain("eth_tx")
        self.cd_rx_pll_ref = ClockDomain("rx_pll_ref", reset_less=True)
        self._reset = CSRStorage(name="reset", description="Assert external Ethernet PHY reset.")
        self.hw_reset = LiteEthPHYHWReset(cycles=hw_reset_cycles)
        self.reset = Signal()
        self.comb += self.reset.eq(self._reset.storage | self.hw_reset.reset | ~crg.locked)
        # A push-pull ~CSR output still lost a persistent PHY register marker
        # during warmboot, indicating a reset; the exact pad transient has not
        # been scoped. Keep the peripheral output tied LOW and drive its enable
        # directly from the reset-zero CSR. Thus normal operation releases the
        # pad to R63 and does not depend on a fabric inverter driving it high.
        # Startup/PLL resets remain local to the newly configured FPGA logic.
        self.specials += Tristate(pads.rst_n, Constant(0, 1), self._reset.storage)

        # R8 is a global clock input, not a dedicated PLL reference pin.
        # Route it through the global input buffer to an independent RX PLL.
        rx_clk = platform.add_iface_io("eth_rx_clock")
        platform.toolchain.ifacewriter.blocks.append({
            "type": "GPIO", "size": 1, "mode": "INPUT_CLK",
            "location": platform.get_pin_location(clock_pads.rx)[0],
            "properties": platform.get_pin_properties(clock_pads.rx),
            "name": rx_clk.name_override,
        })
        platform.toolchain.excluded_ios.append(clock_pads.rx)
        self.rx_pll = TITANIUMPLL(platform)
        self.rx_pll.register_clkin(rx_clk, 125e6)
        # The fixed local feedback output matches this 125 MHz reference.
        # Hardware stress also passed at +/-45 degrees around the selected
        # zero-degree RX phase, with both original PHY delay straps enabled.
        self.rx_pll.create_clkout(self.cd_rx_pll_ref, 125e6, with_reset=False)
        self.rx_pll.create_clkout(self.cd_eth_rx, 125e6, phase=0, with_reset=False)
        self.comb += self.rx_pll.reset.eq(self.reset)
        self.cd_eth_tx.clk = crg.cd_eth_ref.clk
        platform.clks["eth_tx"] = crg.cd_eth_ref.clk.name_override
        platform.add_period_constraint(rx_clk, 8.0)
        platform.add_false_path_constraint(self.cd_eth_rx.clk, crg.cd_sys.clk)

        self.specials += [
            ClkOutput(crg.cd_eth_ref.clk, clock_pads.tx),
            AsyncResetSynchronizer(self.cd_eth_tx, self.reset),
            AsyncResetSynchronizer(self.cd_eth_rx, self.reset | ~self.rx_pll.locked),
        ]
        self.tx = ClockDomainsRenamer("eth_tx")(LiteEthPHYRGMIITX(pads, ClockSignal("eth_tx")))
        self.rx = ClockDomainsRenamer("eth_rx")(LiteEthPHYRGMIIRX(pads, ClockSignal("eth_rx")))
        self.sink, self.source = self.tx.sink, self.rx.source
        self.mdio = LiteEthPHYMDIO(pads)


# Conventional target import name.
CRG = LutcamCRG
