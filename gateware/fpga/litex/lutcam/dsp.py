"""Small Titanium DSP helpers used by explicitly enabled board optimizations."""

from migen import ClockSignal, Constant, Instance, ResetSignal, Signal


def add_dsp_counter(module, value, enable, *, clock_domain="sys"):
    """Increment ``value`` on enabled edges, clearing on its domain reset.

    The native 48-bit W feedback register has synchronous reset priority over
    clock enable. Only the low bits are exposed, preserving modulo arithmetic
    at the caller's width; saturating callers gate ``enable`` at their maximum.
    All event/CDC logic stays in the caller's original clock domain.
    """
    if not 1 <= len(value) <= 48:
        raise ValueError("DSP counters require 1 through 48 output bits")
    result = Signal(48)
    module.specials += Instance("EFX_DSP48",
        p_MODE="NORMAL", p_SIGNED=0, p_M_SEL="C", p_N_SEL="W",
        p_W_SEL="X", p_W_REG=1, p_RST_SYNC=1,
        i_A=Constant(0, 19), i_B=Constant(0, 18), i_C=Constant(1, 18),
        i_CASCIN=Constant(0, 48), i_OP=Constant(0, 2), i_SHIFT_ENA=0,
        i_CLK=ClockSignal(clock_domain), i_CE=enable,
        i_RST=ResetSignal(clock_domain), o_O=result)
    module.comb += value.eq(result[:len(value)])
