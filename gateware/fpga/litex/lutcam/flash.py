"""SPI NOR constants and the shared mode-zero byte engine."""

from migen.genlib.cdc import MultiReg
from migen import Cat, Constant, If, Module, Signal


FLASH_BASE = 0x30025000
FLASH_SIZE = 16 * 1024 * 1024
SLOT_SIZE = 2 * 1024 * 1024
FLASH_MAGIC = 0x4c465331
READ, PROGRAM, ERASE4K, ERASE64K, JEDEC, STATUS = range(1, 7)
ERR_NONE, ERR_RANGE, ERR_PROTECTED, ERR_LENGTH, ERR_ALIGNMENT, ERR_OPCODE, ERR_WEL, ERR_TIMEOUT = range(8)


class _SPIByte(Module):
    """Mode-zero, MSB-first byte engine; all logic uses the system clock."""

    def __init__(self, pads, half_period):
        self.start, self.done = Signal(), Signal()
        self.tx, self.rx = Signal(8), Signal(8)
        shift = Signal(8)
        bit = Signal(3)
        timer = Signal(max=max(2, half_period))
        active = Signal()
        self.sclk = sclk = Signal()
        sclk.name_override = "flash_sclk"
        sclk.attr.add("no_retiming")
        miso_sample = Signal()
        # Lower the standard two-FF synchronizer here so the board can name
        # the first capture stage in its forwarded-SCLK timing constraints.
        # The complete external loop and both internal stages remain timed.
        self.submodules.miso_cdc = miso_cdc = MultiReg.lower(MultiReg(pads.miso, miso_sample))
        self.miso_meta = miso_cdc.regs[0]
        self.miso_meta.name_override = "flash_miso_meta"
        miso_cdc.regs[1].name_override = "flash_miso_sync"
        self.comb += [pads.mosi.eq(shift[7]), pads.clk.eq(sclk)]
        self.sync += [
            self.done.eq(0),
            If(~active,
                sclk.eq(0),
                If(self.start,
                    shift.eq(self.tx), self.rx.eq(0), bit.eq(7),
                    timer.eq(half_period - 1), active.eq(1),
                ),
            ).Elif(timer != 0,
                timer.eq(timer - 1),
            ).Else(
                timer.eq(half_period - 1),
                If(~sclk,
                    sclk.eq(1),
                ).Else(
                    # Consume the previous bit at the end of the high phase.
                    # The two-FF input pipeline still contains that bit when
                    # this falling edge launches the next bit at the flash.
                    # This gives one full SPI period for flash/output/input
                    # delay and the synchronizer, rather than half a period.
                    sclk.eq(0), self.rx.eq(Cat(miso_sample, self.rx[:7])),
                    If(bit == 0,
                        active.eq(0), self.done.eq(1),
                    ).Else(bit.eq(bit - 1), shift.eq(Cat(Constant(0, 1), shift[:7]))),
                ),
            ),
        ]
