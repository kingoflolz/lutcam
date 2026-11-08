"""Read-only electrical USB observations, readable through either host bus.

This block observes existing pins and controls; it never drives the USB PHY.
All observations pass through two synchronizer stages into ``sys`` so they
remain readable when either USB domain is held in reset. Live bits are sampled
individually, not an atomic USB-domain snapshot. Counters describe electrical
activity, not decoded packets or successful enumeration.
"""

from functools import reduce
from operator import or_

from migen import Cat, Case, Constant, If, Replicate, Signal
from migen.genlib.cdc import MultiReg
from litex.gen import LiteXModule
from litex.soc.integration.export import MockCSR
from litex.soc.integration.soc import SoCCSRRegion
from litex.soc.interconnect import wishbone

from .dsp import add_dsp_counter


USB_LINK_BASE = 0x30027000
USB_LINK_MAGIC = 0x55445331  # UDS1

# These are fixed byte offsets, independent of LiteX's automatic CSR numbering.
USB_LINK_REGISTERS = (
    ("magic", 0x00), ("reserved0", 0x04), ("live", 0x08), ("seen", 0x0c),
    ("rx_edges", 0x10), ("tx_bursts", 0x14), ("se0_events", 0x18),
    ("se0_cycles", 0x1c), ("se0_threshold", 0x20), ("sys_hz", 0x24),
    ("counter_bits", 0x28), ("usb_12_heartbeat", 0x2c),
    ("usb_48_heartbeat", 0x30), ("heartbeat_divisor", 0x34),
)
USB_LINK_HEARTBEAT_DIVISOR = 128
USB_LINK_LIVE_BITS = (
    "dp_i", "dm_i", "dp_oe", "dm_oe", "pullup", "connect",
    "usb_12_reset", "usb_48_reset", "pll_locked", "vbus_valid", "dp_o", "dm_o",
)
USB_LINK_SEEN_BITS = (
    "rx_edge", "tx_drive", "long_se0", "connect", "pullup",
    "usb_12_reset", "usb_48_reset", "vbus_valid", "pll_locked",
)


class USBLinkDiagnostics(LiteXModule):
    """Fixed 4 KiB, read-only Wishbone aperture for USB link diagnosis.

    ``live`` follows USB_LINK_LIVE_BITS; ``seen`` follows USB_LINK_SEEN_BITS.
    Sticky bits and counters clear only with system reset/reconfiguration.
    Event counters saturate at ``(1 << counter_bits) - 1``. Reads do not clear
    them. The two 32-bit heartbeats wrap, and count one synchronized transition
    per 128 source clocks, regardless of attachment. A source-domain reset may
    contribute one extra transition. They indicate clock activity, rather than
    measuring exact frequency; a held USB domain reset stops its heartbeat.

    RX edges count sampled D+/D- changes only when both adjacent samples are
    attached, powered, out of reset and not driven by the FPGA. TX bursts count
    rising edges of either output enable while connect is asserted. The sticky
    TX-drive bit also records unexpected drive while disconnected.
    ``se0_events`` counts each continuous undriven SE0 interval reaching 5 us
    while attachment is enabled. This is evidence of host reset signaling (or
    stuck-low wiring), not proof that LUNA accepted a reset. Short EOPs do not
    qualify. ``se0_cycles`` saturates at the reported threshold until SE0 ends.

    ``use_dsp_counters`` moves the five event/heartbeat accumulators into
    Titanium DSP feedback registers. The default uses portable fabric logic.
    """

    @classmethod
    def from_luna(cls, usb, *, pll_locked, vbus_valid, sys_clk_freq=120_000_000,
                  use_dsp_counters=False, shared_readback=None):
        """Observe the existing LiteX converter bindings without modifying them.

        The vendored wrapper supplies the actual USB protocol reset.
        The returned module owns a Wishbone ``bus`` and fixed-address ``regions``
        metadata; the caller installs both in the SoC.
        """
        fields = {
            "dp_i": "i__bus_d_p_i", "dm_i": "i__bus_d_n_i",
            "dp_o": "o__bus_d_p_o", "dm_o": "o__bus_d_n_o",
            "dp_oe": "o__bus_d_p_oe", "dm_oe": "o__bus_d_n_oe",
            "pullup": "o__bus_pullup_o", "connect": "i_connect",
            "usb_12_reset": "i_usb_rst", "usb_48_reset": "i_usb_io_rst",
        }
        parameters = getattr(usb, "core_params", {})
        if any(name not in parameters for name in fields.values()):
            raise ValueError("USB diagnostics require the raw PHY and bound USB resets")
        return cls(**{field: parameters[name] for field, name in fields.items()},
                   pll_locked=pll_locked, vbus_valid=vbus_valid, sys_clk_freq=sys_clk_freq,
                   use_dsp_counters=use_dsp_counters, shared_readback=shared_readback)

    def __init__(self, *, dp_i, dm_i, dp_o, dm_o, dp_oe, dm_oe, pullup, connect,
                 usb_12_reset, usb_48_reset, pll_locked, vbus_valid,
                 sys_clk_freq=120_000_000, se0_threshold=None, counter_bits=16,
                 use_dsp_counters=False, shared_readback=None):
        inputs = (dp_i, dm_i, dp_oe, dm_oe, pullup, connect, usb_12_reset,
                  usb_48_reset, pll_locked, vbus_valid, dp_o, dm_o)
        if any(len(signal) != 1 for signal in inputs):
            raise ValueError("USB diagnostic inputs must be one bit")
        if type(sys_clk_freq) is not int or not 0 < sys_clk_freq <= 0xffffffff:
            raise ValueError("USB diagnostic system frequency must be a positive uint32")
        if se0_threshold is None:
            se0_threshold = (sys_clk_freq + 199_999) // 200_000  # At least 5 us.
        if type(se0_threshold) is not int or not 1 <= se0_threshold <= 0xffffffff:
            raise ValueError("USB SE0 threshold must be a positive uint32")
        if type(counter_bits) is not int or not 2 <= counter_bits <= 32:
            raise ValueError("USB diagnostic counters require 2 through 32 bits")

        self.bus = bus = wishbone.Interface(data_width=32, addressing="word")
        response = Signal(32) if shared_readback is None else shared_readback.word
        self.comb += bus.dat_r.eq(response)
        self.live = live = Signal(len(inputs))
        self.seen = seen = Signal(len(USB_LINK_SEEN_BITS))
        self.rx_edges = rx_edges = Signal(counter_bits)
        self.tx_bursts = tx_bursts = Signal(counter_bits)
        self.se0_events = se0_events = Signal(counter_bits)
        self.se0_cycles = se0_cycles = Signal(max=se0_threshold + 1)
        self.specials += MultiReg(Cat(*inputs), live)

        driven = live[2] | live[3]
        attached = live[4] & live[5] & ~live[6] & ~live[7] & live[8] & live[9]
        receiving = attached & ~driven
        previous_lines = Signal(2)
        previous_receiving = Signal()
        previous_driven = Signal()
        rx_edge = receiving & previous_receiving & (live[:2] != previous_lines)
        tx_start = live[5] & driven & ~previous_driven
        se0 = receiving & (live[:2] == 0)
        long_se0 = se0 & (se0_cycles == se0_threshold - 1)
        maximum = (1 << counter_bits) - 1
        self.sync += [
            previous_lines.eq(live[:2]), previous_receiving.eq(receiving),
            previous_driven.eq(driven),
            seen.eq(seen | Cat(rx_edge, driven, long_se0, live[5], live[4],
                               live[6], live[7], live[9], live[8])),
            If(~se0, se0_cycles.eq(0)).Elif(se0_cycles != se0_threshold,
                                           se0_cycles.eq(se0_cycles + 1)),
        ]
        for value, event in ((rx_edges, rx_edge), (tx_bursts, tx_start),
                             (se0_events, long_se0)):
            enable = event & (value != maximum)
            if use_dsp_counters:
                add_dsp_counter(self, value, enable)
            else:
                self.sync += If(enable, value.eq(value + 1))

        # Only a registered divider MSB crosses domains. Its 128-cycle dwell
        # is far longer than the sys synchronizer latency, avoiding a wide
        # binary counter crossing or loss of clock transitions at 120 MHz.
        heartbeats = []
        for domain in ("usb_12", "usb_48"):
            divider = Signal(8)
            sample = Signal()
            previous_sample = Signal()
            heartbeat = Signal(32)
            setattr(self, domain + "_heartbeat", heartbeat)
            getattr(self.sync, domain).__iadd__(divider.eq(divider + 1))
            self.specials += MultiReg(divider[-1], sample)
            self.sync += previous_sample.eq(sample)
            if use_dsp_counters:
                add_dsp_counter(self, heartbeat, sample != previous_sample)
            else:
                self.sync += If(sample != previous_sample, heartbeat.eq(heartbeat + 1))
            heartbeats.append(heartbeat)

        values = (USB_LINK_MAGIC, 0, live, seen, rx_edges,
                  tx_bursts, se0_events, se0_cycles, se0_threshold,
                  sys_clk_freq, counter_bits, *heartbeats,
                  USB_LINK_HEARTBEAT_DIVISOR)
        # Mutually exclusive masked values share decode without changing the
        # registered ACK/ERR behavior or idle/error data values.
        selected_read = Signal(32)
        self.comb += selected_read.eq(reduce(or_, (
            (Constant(value, 32) if isinstance(value, int) else value)
            & Replicate(bus.adr[:10] == offset // 4, 32)
            for (_, offset), value in zip(USB_LINK_REGISTERS, values))))
        reads = {offset // 4: bus.ack.eq(1) for _, offset in USB_LINK_REGISTERS}
        # One registered response per classic transfer, including contiguous
        # Etherbone accesses. Writes/undefined offsets are errors with no effect.
        self.sync += [
            bus.ack.eq(0), bus.err.eq(0),
            If(bus.cyc & bus.stb & ~bus.ack & ~bus.err,
                response.eq(0),
                If(bus.we, bus.err.eq(1)).Else(
                    response.eq(selected_read),
                    Case(bus.adr[:10], {**reads, "default": bus.err.eq(1)}))),
        ]
        self.regions = {"usb_link": SoCCSRRegion(USB_LINK_BASE, 32, [
            MockCSR(name, 1, "ro", origin=USB_LINK_BASE + offset)
            for name, offset in USB_LINK_REGISTERS
        ])}

        if shared_readback is not None:
            shared_readback.attach(self)
