"""Small software-operated camera controls and bounded Wishbone hosts.

All register offsets below are byte offsets. Buses are 32-bit, word-addressed,
and peripherals decode a 4 KiB aperture. Registered responses consume a request
once and suppress side effects while that response retires. Consecutive classic
transfers may keep CYC/STB asserted, as Etherbone burst writes do.
"""

from migen import Cat, Case, Constant, FSM, If, Module, Mux, NextState, NextValue, Signal, TSTriple
from migen.genlib.cdc import MultiReg
from litex.soc.interconnect import wishbone


class CameraI2C(Module):
    """Open-drain SCL/SDA registers; all I2C/SCCB protocol lives in software.

    0x00 CONTROL (RW): bit 0 pulls SCL low, bit 1 pulls SDA low. Zero releases
         both lines and is the reset value. Only byte lane zero writes matter.
    0x04 STATUS (RO): bit 0 is sampled SCL, bit 1 sampled SDA, after two stages
         of synchronization. External pull-ups are required on both lines.

    ``pads`` has ``scl`` and ``sda`` members. With pads=None, the exposed
    ``scl``/``sda`` TSTriples may instead connect to vendor IO primitives.
    """

    def __init__(self, pads=None):
        self.bus = bus = wishbone.Interface(data_width=32, addressing="word")
        self.scl = scl = TSTriple()
        self.sda = sda = TSTriple()
        self.control = control = Signal(2)
        self.pin_status = pin_status = Signal(2)
        request = Signal()
        self.comb += [
            scl.o.eq(0), sda.o.eq(0),
            scl.oe.eq(control[0]), sda.oe.eq(control[1]),
            request.eq(bus.cyc & bus.stb & ~bus.ack & ~bus.err),
        ]
        self.specials += MultiReg(Cat(scl.i, sda.i), pin_status)
        if pads is not None:
            self.specials += [scl.get_tristate(pads.scl), sda.get_tristate(pads.sda)]
        self.sync += [
            bus.ack.eq(0), bus.err.eq(0),
            If(request,
                bus.dat_r.eq(0),
                Case(bus.adr[:10], {
                    0: [
                        bus.ack.eq(1),
                        bus.dat_r.eq(control),
                        If(bus.we & bus.sel[0], control.eq(bus.dat_w[:2])),
                    ],
                    1: If(bus.we, bus.err.eq(1)).Else(
                        bus.ack.eq(1), bus.dat_r.eq(pin_status)),
                    "default": bus.err.eq(1),
                }),
            ),
        ]


class HistogramReader(Module):
    """Read a stopped MonoIsp's histogram without a second histogram buffer.

    Connect the five ``hist_*`` signals to the corresponding MonoIsp ports.
    0x00 COMMAND (WO): write bit 0 to start; errors if busy or ISP not ready.
    0x04 STATUS (RO): bit 0 ready to start, bit 1 dump active, bit 2 word valid.
    0x08 DATA (RO): reads and consumes exactly one histogram word. An empty
         read errors immediately. Poll STATUS bit 2 first. Data is unmodified:
         bit 31 first, bit 30 last, bits 28:20 bin, bits 19:0 pixel-pair count.
    0x0c COUNT (RO): words consumed since the last successful start (0..512).

    The last word ends the dump. Run/clear must remain disabled in MonoIsp
    until the dump completes. Reading status never consumes histogram data.
    When shared_readback is provided, acknowledged responses use its common
    storage; inactive response data is unspecified. The default stays private.
    """

    def __init__(self, *, shared_readback=None):
        self.bus = bus = wishbone.Interface(data_width=32, addressing="word")
        response = Signal(32) if shared_readback is None else shared_readback.word
        self.comb += bus.dat_r.eq(response)
        self.hist_dump_req = Signal()
        self.hist_dump_ready = Signal()
        self.hist_out_valid = Signal()
        self.hist_out_ready = Signal()
        self.hist_out_data = Signal(32)
        self.busy = busy = Signal()
        self.words_read = words_read = Signal(10)
        request, start, pop = Signal(), Signal(), Signal()
        can_start = Signal()
        self.comb += [
            request.eq(bus.cyc & bus.stb & ~bus.ack & ~bus.err),
            can_start.eq(self.hist_dump_ready & ~busy),
            start.eq(request & (bus.adr[:10] == 0) & bus.we & bus.sel[0]
                     & bus.dat_w[0] & can_start),
            pop.eq(request & (bus.adr[:10] == 2) & ~bus.we
                   & busy & self.hist_out_valid),
            self.hist_dump_req.eq(start),
            self.hist_out_ready.eq(pop),
        ]
        self.sync += [
            bus.ack.eq(0), bus.err.eq(0),
            If(start, busy.eq(1), words_read.eq(0)),
            If(pop,
                If(words_read != 512, words_read.eq(words_read + 1)),
                If(self.hist_out_data[30], busy.eq(0)),
            ),
            If(request,
                response.eq(0),
                Case(bus.adr[:10], {
                    0: If(~bus.we,
                        bus.err.eq(1),
                    ).Elif(bus.sel[0] & bus.dat_w[0] & ~can_start,
                        bus.err.eq(1),
                    ).Else(bus.ack.eq(1)),
                    1: If(bus.we, bus.err.eq(1)).Else(
                        bus.ack.eq(1),
                        response.eq(Cat(can_start, busy, self.hist_out_valid & busy)),
                    ),
                    2: If(pop,
                        bus.ack.eq(1), response.eq(self.hist_out_data),
                    ).Else(bus.err.eq(1)),
                    3: If(bus.we, bus.err.eq(1)).Else(
                        bus.ack.eq(1), response.eq(words_read),
                    ),
                    "default": bus.err.eq(1),
                }),
            ),
        ]

        if shared_readback is not None:
            shared_readback.attach(self)


class WishboneGuard(Module):
    """Bound one ACK-only host's transactions and terminate downstream errors.

    ``bus`` is the upstream host-facing port; ``master`` connects to the SoC
    interconnect. The host holds request fields until ACK, as classic Wishbone
    requires. Responses pass through during the active transfer, followed by
    a recovery cycle with downstream CYC/STB deasserted. ACK-only
    hosts (UARTBone/Etherbone) receive ACK for success, downstream ERR, and
    timeout. Errors return 0xffffffff; no failed operation is retried.
    Connect ``granted`` to the shared arbiter's selection when several hosts
    share the bus. The timeout counts only granted cycles, so another host's
    stalled access cannot cause a waiting valid transaction to time out.
    ``granted`` defaults to one for a direct connection.

    Exposed diagnostic signals suit CSRStatus: fault (sticky), fault_address
    (last failing byte address), fault_status (bit 0 ERR, bit 1 timeout,
    bit 2 write), and fault_count (saturating). Pulse clear_fault to clear all.
    A newly observed failure takes precedence over a simultaneous clear.
    With ``external_count=True``, count ``fault_event`` pulses in the shared
    diagnostic bank instead; ``fault_count`` then has no local accumulator.
    """

    def __init__(self, timeout_cycles=4096, *, external_count=False):
        if timeout_cycles < 1:
            raise ValueError("timeout_cycles must be positive")
        self.timeout_cycles = timeout_cycles
        self.bus = bus = wishbone.Interface(data_width=32, addressing="word")
        self.master = master = wishbone.Interface(data_width=32, addressing="word")
        self.fault = Signal()
        self.fault_address = Signal(32)
        self.fault_status = Signal(3)
        self.fault_count = Signal(32)
        self.external_count = external_count
        self.clear_fault = Signal()
        self.granted = Signal(reset=1)
        timer = Signal(max=max(2, timeout_cycles))
        failure = Signal()
        # The board can accumulate this pulse in its shared diagnostic RAM.
        # Standalone guards retain the original directly readable counter.
        self.fault_event = failure
        timeout = Signal()
        self.comb += [
            master.adr.eq(bus.adr), master.dat_w.eq(bus.dat_w),
            master.sel.eq(bus.sel), master.we.eq(bus.we),
            # This bridge emits individual classic transfers, including when
            # Etherbone keeps its upstream CYC asserted across a write burst.
            master.cti.eq(wishbone.CTI_BURST_NONE), master.bte.eq(0),
            bus.dat_r.eq(Mux(failure, 0xffffffff, master.dat_r)), bus.err.eq(0),
        ]
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(bus.cyc & bus.stb,
                NextValue(timer, 0), NextState("ACTIVE"),
            ),
        )
        fsm.act("ACTIVE",
            master.cyc.eq(bus.cyc), master.stb.eq(bus.cyc),
            If(~bus.cyc,
                NextState("IDLE"),
            ).Elif(~self.granted,
                NextValue(timer, 0),
            ).Elif(master.err,
                failure.eq(1), bus.ack.eq(bus.stb),
                NextState("RECOVERY"),
            ).Elif(master.ack,
                bus.ack.eq(bus.stb), NextState("RECOVERY"),
            ).Elif(timer == timeout_cycles - 1,
                failure.eq(1), timeout.eq(1), bus.ack.eq(bus.stb),
                NextState("RECOVERY"),
            ).Else(NextValue(timer, timer + 1)),
        )
        fsm.act("RECOVERY",
            NextState("IDLE"),
        )
        self.sync += [
            If(self.clear_fault,
                self.fault.eq(0), self.fault_address.eq(0),
                self.fault_status.eq(0),
            ),
            If(failure,
                self.fault.eq(1),
                self.fault_address.eq(Cat(Constant(0, 2), bus.adr)),
                self.fault_status.eq(Cat(~timeout, timeout, bus.we)),
            ),
        ]
        if not external_count:
            self.sync += [
                If(self.clear_fault, self.fault_count.eq(0)),
                If(failure,
                    If(self.clear_fault, self.fault_count.eq(1)).Elif(
                        self.fault_count != 0xffffffff,
                        self.fault_count.eq(self.fault_count + 1))),
            ]
