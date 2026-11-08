"""Board USB connection sequencing.

The USB core and wrapper fixes live in vendor/luna and vendor/litex_luna.
"""
from migen import If, ResetSignal, Signal
from migen.genlib.cdc import MultiReg
from litex.gen import LiteXModule


class USBConnectDelay(LiteXModule):
    """Hold the pull-up disconnected for 100 ms of stable USB readiness.

    ``ready`` is VBUS presence qualified by the PLL supplying USB clocks and
    the absence of a pending FPGA reboot. Its
    synchronized value must stay asserted for ``delay_cycles`` USB 12 MHz
    cycles. VBUS loss or domain reset restarts the complete interval, ensuring
    each configured image presents a deliberate detach before reconnecting.
    """

    def __init__(self, ready, *, delay_cycles=1_200_000):
        if type(delay_cycles) is not int or delay_cycles < 1:
            raise ValueError("USB connection delay must be a positive cycle count")
        self.connect = Signal()
        ready_usb = Signal()
        elapsed = Signal(max=max(2, delay_cycles + 1))
        self.specials += MultiReg(ready, ready_usb, odomain="usb_12")
        self.sync.usb_12 += If(~ready_usb,
            elapsed.eq(0)
        ).Elif(elapsed != delay_cycles, elapsed.eq(elapsed + 1))
        self.comb += self.connect.eq(ready_usb & (elapsed == delay_cycles)
                                    & ~ResetSignal("usb_12"))
