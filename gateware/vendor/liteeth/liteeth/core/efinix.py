"""Titanium arithmetic primitives used by the Lutcam build."""
from migen import ClockSignal, Constant, If, Instance, Module, ResetSignal, Signal


class DspWaitTimer(Module):
    """Same wait/done cycle contract as WaitTimer, with an elapsed counter."""
    def __init__(self, cycles):
        cycles = int(cycles)
        if cycles < 1:
            raise ValueError("native wait timer requires at least one cycle")
        self.wait, self.done = Signal(), Signal()
        count, result = Signal(max=cycles+1), Signal(48)
        if len(count) > 48:
            raise ValueError("native wait timer must fit 48 bits")
        self.specials += Instance("EFX_DSP48",
            p_MODE="NORMAL", p_SIGNED=0, p_M_SEL="C", p_N_SEL="W",
            p_W_SEL="X", p_W_REG=1, p_RST_SYNC=1,
            i_A=Constant(0,19), i_B=Constant(0,18), i_C=Constant(1,18),
            i_CASCIN=Constant(0,48), i_OP=Constant(0,2), i_SHIFT_ENA=0,
            i_CLK=ClockSignal(), i_CE=self.wait & ~self.done,
            i_RST=ResetSignal() | ~self.wait, o_O=result)
        self.comb += count.eq(result[:len(count)])
        # Predict the same terminal edge as the elapsed counter. Keeping done
        # registered removes its wide comparison from the live bus/reset path.
        # Global reset clears this register together with the DSP accumulator.
        self.sync += If(~self.wait,
            self.done.eq(0),
        ).Else(self.done.eq(self.done | (count == cycles - 1)))

