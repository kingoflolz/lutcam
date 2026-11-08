"""Iterative IPv4 checksum with the original enabled-cycle latency."""
from migen import Array, Cat, If, Signal
from litex.gen import LiteXModule
from liteeth.common import ipv4_header


class IterativeIPV4Checksum(LiteXModule):
    """Reuse one accumulator while retaining LiteEth's enabled-cycle latency.

    The header remains stable during calculation, as in LiteEth IP TX/RX.
    Only ``value`` with ``done`` asserted is valid. An existing interface can
    supply the same Signal objects when replacing a constructed submodule.
    """

    def __init__(self, words_per_clock_cycle=1, skip_checksum=False, interface=None):
        if words_per_clock_cycle != 1:
            raise ValueError("Iterative IPv4 checksum supports one word per clock")
        for name, width in (("header", 160), ("value", 16), ("done", 1),
                            ("ce", 1), ("reset", 1)):
            setattr(self, name, Signal(width) if interface is None else getattr(interface, name))

        words = [self.header[i * 16:(i + 1) * 16]
                 for i in range(ipv4_header.length // 2)
                 if not (skip_checksum and i == ipv4_header.fields["checksum"].byte // 2)]
        # Retain the original RX checksum's additional enabled completion cycle.
        n_cycles = len(words) + (not skip_checksum)
        count = Signal(max=n_cycles + 1)
        accumulator = Signal(16)
        total = Signal(17)
        self.comb += [
            total.eq(accumulator + Array(words)[count]),
            self.value.eq(~Cat(accumulator[8:16], accumulator[:8])),
            self.done.eq(count == n_cycles),
        ]
        self.sync += If(self.reset,
            count.eq(0),
            accumulator.eq(0),
        ).Elif(self.ce & ~self.done,
            count.eq(count + 1),
            If(count < len(words), accumulator.eq(total[:16] + total[16])),
        )

