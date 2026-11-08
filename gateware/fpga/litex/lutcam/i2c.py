"""Bounded, shared OV9281 SCCB transactions with GPIO recovery.

Both camera apertures retain CONTROL/STATUS at +0/+4. STATUS bit 8 advertises
the engine before accessing its command registers.
One engine serves both buses; the command-record aperture selects the camera.
Hosts must serialize complete transactions, including their result collection.

Offsets (bytes): 10 magic 49324301; 14 status; 18 descriptor; 1c command ID;
20 done ID; 24 next ID; 28 result; 2c maximum bytes (16); 60..6c RX;
80..98 complete command record. All offsets except GPIO CONTROL and the
command record are read-only; 40..4c aliases the RX buffer for inspection.
Descriptor: device[6:0], read[7], register[23:8], length-minus-one[27:24],
slow[28], reserved[31:29]=0. Slow selects 100 kHz rather than nominal 400 kHz.
Status: busy[0], done-valid[1], accepted-camera[2], accepted-read[3].
Result: error[7:0], failing wire-byte index[15:8], completed data bytes[23:16].
Errors: 0 success, 1 NACK, 2 SCL-high timeout, 3 SDA low before START/after STOP.

One incrementing transport record at +80 contains NEXT_ID, descriptor, write
payload words if any, and the matching ID. Reads require 3 words; writes need
3+ceil(length/4), at most 7. Bytes occupy low-to-high lanes. Trusted transport
metadata must identify the complete record, its source and every boundary;
unqualified/stale/partial records never execute. A source-qualified abort wins
commit. The 16-byte TX/RX buffer is shared, and a new next-ID record invalidates
the old result as soon as its header is accepted. RX reads never consume it.
IDs are globally sequential modulo 2**32. Duplicate/stale/busy records drain
without mutation or ID consumption. A partial record owns the ingress until
completion, its source starts a new record, abort, or the bounded lease timer.
"""

from migen import (Array, Cat, Case, Constant, FSM, If, Module, Mux,
                   NextState, NextValue, Signal, TSTriple, Memory, ClockSignal, ResetSignal, Instance)
from migen.genlib.cdc import MultiReg
from litex.soc.integration.export import MockCSR
from litex.soc.integration.soc import SoCCSRRegion
from litex.soc.interconnect import wishbone


I2C_MAGIC = 0x49324301
I2C_CAPABILITY = 1 << 8
I2C_MAX_BYTES = 16
I2C_BASES = (0x30020000, 0x30021000)


class _I2CByteMemory(Memory):
    def __init__(self):
        super().__init__(8, 16, name="i2c_bytes")
        self.attr = {("syn_ramstyle", "block_ram")}

    @staticmethod
    def emit_verilog(memory, namespace, add_data_file):
        return '(* syn_ramstyle = "block_ram" *) ' + Memory.emit_verilog(
            memory, namespace, add_data_file)


def _capture_dsp(module, target, value, enable):
    """Capture up to 37 bits with the ordinary synchronous reset priority."""
    if not 1 <= len(target) <= 37:
        raise ValueError("DSP capture requires 1 through 37 bits")
    word, result = Signal(37), Signal(48)
    module.comb += word.eq(value)
    module.specials += Instance("EFX_DSP48",
        p_MODE="NORMAL", p_SIGNED=0, p_M_SEL="P", p_N_SEL="C",
        p_C_EXT="ALIGN_LEFT", p_W_SEL="X", p_W_REG=1, p_RST_SYNC=1,
        i_A=word[:19], i_B=Constant(1, 18), i_C=word[19:37],
        i_CASCIN=Constant(0, 48), i_OP=Constant(0, 2), i_SHIFT_ENA=0,
        i_CLK=ClockSignal(), i_CE=enable, i_RST=ResetSignal(), o_O=result)
    module.comb += target.eq(Cat(result[:19], result[30:48])[:len(target)])


class I2CTransactionEngine(Module):
    """Single asynchronous transaction, using already synchronized pin levels.

    Actual SCL-high observation starts each high interval. At 120 MHz the
    default 180/120 cycles respect OV9281's 1.3 us low minimum; a symmetric
    400 kHz divider would not. START/STOP use the same conservative intervals.
    A timeout immediately releases both pins; recovery is an explicit host
    operation after the engine becomes idle.
    """

    def __init__(self, *, fast_low_cycles=180, fast_high_cycles=120,
                 stretch_timeout_cycles=3_000_000, use_dsp_watchdog=False):
        if min(fast_low_cycles, fast_high_cycles) < 2:
            raise ValueError("I2C phases must last at least two system cycles")
        if stretch_timeout_cycles < 2:
            raise ValueError("I2C timeout must be at least two system cycles")
        self.start, self.busy, self.done = Signal(), Signal(), Signal()
        self.device, self.register = Signal(7), Signal(16)
        self.read, self.slow, self.count_minus_one = Signal(), Signal(), Signal(4)
        self.scl_i, self.sda_i = Signal(), Signal()
        self.scl_low, self.sda_low = Signal(), Signal()
        self.tx_index, self.tx_data = Signal(4), Signal(8)
        self.tx_prefetch_index, self.tx_prefetch_priority = Signal(4), Signal()
        self.rx_index, self.rx_data, self.rx_valid = Signal(4), Signal(8), Signal()
        self.error, self.fail_index, self.completed = Signal(2), Signal(5), Signal(5)
        # Reuse the otherwise idle stretch watchdog for bounded packet ingress.
        self.lease_wait, self.lease_expired, self.lease_reset = Signal(), Signal(), Signal()

        low_reload = Mux(self.slow, 4 * fast_low_cycles - 1, fast_low_cycles - 1)
        high_reload = Mux(self.slow, 4 * fast_high_cycles - 1, fast_high_cycles - 1)
        timer = Signal(max=4 * max(fast_low_cycles, fast_high_cycles))
        wait_high = Signal()
        stretch = Signal(max=stretch_timeout_cycles)
        expired = Signal()
        self.comb += expired.eq(wait_high & ~self.scl_i
                                 & (stretch == stretch_timeout_cycles - 1))
        self.comb += self.lease_expired.eq(self.lease_wait
                                         & (stretch == stretch_timeout_cycles - 1))
        if use_dsp_watchdog:
            if len(stretch) > 48:
                raise ValueError("DSP watchdog requires at most 48 bits")
            waiting = (wait_high & ~self.scl_i) | self.lease_wait
            result = Signal(48)
            self.specials += Instance("EFX_DSP48",
                p_MODE="NORMAL", p_SIGNED=0, p_M_SEL="C", p_N_SEL="W",
                p_W_SEL="X", p_W_REG=1, p_RST_SYNC=1,
                i_A=Constant(0, 19), i_B=Constant(0, 18), i_C=Constant(1, 18),
                i_CASCIN=Constant(0, 48), i_OP=Constant(0, 2), i_SHIFT_ENA=0,
                i_CLK=ClockSignal(), i_CE=waiting & (stretch != stretch_timeout_cycles - 1),
                i_RST=ResetSignal() | self.start | self.lease_reset | ~waiting, o_O=result)
            self.comb += stretch.eq(result[:len(stretch)])
        else:
            self.sync += If(self.start | self.lease_reset, stretch.eq(0)).Elif(
                (wait_high & ~self.scl_i) | self.lease_wait,
                If(stretch != stretch_timeout_cycles - 1, stretch.eq(stretch + 1)),
            ).Else(stretch.eq(0))

        phase = Signal(3)  # 0 address-W, 1 register-H, 2 register-L, 3 address-R, 4 data
        index, bit = Signal(4), Signal(3)
        wire_index = Signal(5)
        shift = Signal(8)
        receiving = (phase == 4) & self.read
        self.comb += [self.tx_index.eq(index), self.rx_index.eq(index),
                      self.rx_data.eq(shift)]
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += [self.busy.eq(~fsm.ongoing("IDLE")),
                      self.done.eq(fsm.ongoing("FINISH")),
            self.tx_prefetch_priority.eq(fsm.ongoing("BYTE_NEXT")),
            self.tx_prefetch_index.eq(Mux(fsm.ongoing("BYTE_NEXT")
                & (phase == 4) & (index != self.count_minus_one), index + 1, index))]

        def timeout():
            return [NextValue(self.error, 2), NextValue(self.fail_index, wire_index),
                    NextState("FINISH")]

        fsm.act("IDLE",
            If(self.start,
                NextValue(self.error, 0), NextValue(self.fail_index, 0),
                NextValue(self.completed, 0), NextValue(phase, 0),
                NextValue(index, 0), NextValue(wire_index, 0),
                NextValue(timer, low_reload), NextState("BUS_FREE")))
        fsm.act("BUS_FREE", wait_high.eq(1),
            If(expired, *timeout()).Elif(~self.scl_i,
                NextValue(timer, low_reload),
            ).Elif(timer != 0, NextValue(timer, timer - 1)).Elif(~self.sda_i,
                NextValue(self.error, 3), NextState("FINISH"),
            ).Else(NextValue(timer, high_reload), NextState("START_HOLD")))
        fsm.act("START_HOLD", self.sda_low.eq(1),
            If(timer != 0, NextValue(timer, timer - 1)).Else(NextState("LOAD_BYTE")))
        fsm.act("LOAD_BYTE", self.scl_low.eq(1),
            NextValue(bit, 7), NextValue(timer, low_reload),
            Case(phase, {
                0: NextValue(shift, Cat(Constant(0, 1), self.device)),
                1: NextValue(shift, self.register[8:16]),
                2: NextValue(shift, self.register[:8]),
                3: NextValue(shift, Cat(Constant(1, 1), self.device)),
                4: NextValue(shift, Mux(self.read, 0, self.tx_data)),
            }), NextState("BIT_LOW"))
        fsm.act("BIT_LOW", self.scl_low.eq(1), self.sda_low.eq(~receiving & ~shift[7]),
            If(timer != 0, NextValue(timer, timer - 1)).Else(NextState("BIT_WAIT")))
        fsm.act("BIT_WAIT", wait_high.eq(1), self.sda_low.eq(~receiving & ~shift[7]),
            If(expired, *timeout()).Elif(self.scl_i,
                NextValue(timer, high_reload), NextState("BIT_HIGH")))
        fsm.act("BIT_HIGH", self.sda_low.eq(~receiving & ~shift[7]),
            If(timer != 0, NextValue(timer, timer - 1)).Else(
                NextValue(shift, Cat(self.sda_i, shift[:7])),
                NextValue(timer, low_reload),
                If(bit == 0, NextState("ACK_LOW")).Else(
                    NextValue(bit, bit - 1), NextState("BIT_LOW"))))
        ack_low = receiving & (index != self.count_minus_one)
        fsm.act("ACK_LOW", self.scl_low.eq(1), self.sda_low.eq(ack_low),
            If(timer != 0, NextValue(timer, timer - 1)).Else(NextState("ACK_WAIT")))
        fsm.act("ACK_WAIT", wait_high.eq(1), self.sda_low.eq(ack_low),
            If(expired, *timeout()).Elif(self.scl_i,
                NextValue(timer, high_reload), NextState("ACK_HIGH")))
        fsm.act("ACK_HIGH", self.sda_low.eq(ack_low),
            If(timer != 0, NextValue(timer, timer - 1)).Elif(~receiving & self.sda_i,
                NextValue(self.error, 1), NextValue(self.fail_index, wire_index),
                NextValue(timer, low_reload), NextState("STOP_LOW"),
            ).Else(NextState("BYTE_NEXT")))
        fsm.act("BYTE_NEXT", self.scl_low.eq(1),
            NextValue(wire_index, wire_index + 1),
            Case(phase, {
                0: [NextValue(phase, 1), NextState("LOAD_BYTE")],
                1: [NextValue(phase, 2), NextState("LOAD_BYTE")],
                2: If(self.read,
                    NextValue(phase, 3), NextValue(timer, low_reload), NextState("RESTART_LOW"),
                ).Else(NextValue(phase, 4), NextState("LOAD_BYTE")),
                3: [NextValue(phase, 4), NextState("LOAD_BYTE")],
                4: [self.rx_valid.eq(self.read), NextValue(self.completed, self.completed + 1),
                    If(index == self.count_minus_one,
                        NextValue(timer, low_reload), NextState("STOP_LOW"),
                    ).Else(NextValue(index, index + 1), NextState("LOAD_BYTE"))],
            }))
        fsm.act("RESTART_LOW", self.scl_low.eq(1),
            If(timer != 0, NextValue(timer, timer - 1)).Else(NextState("RESTART_WAIT")))
        fsm.act("RESTART_WAIT", wait_high.eq(1),
            If(expired, *timeout()).Elif(self.scl_i,
                NextValue(timer, high_reload), NextState("RESTART_SETUP")))
        fsm.act("RESTART_SETUP",
            If(timer != 0, NextValue(timer, timer - 1)).Elif(~self.sda_i,
                NextValue(self.error, 3), NextValue(self.fail_index, wire_index), NextState("FINISH"),
            ).Else(NextValue(timer, high_reload), NextState("START_HOLD")))
        fsm.act("STOP_LOW", self.scl_low.eq(1), self.sda_low.eq(1),
            If(timer != 0, NextValue(timer, timer - 1)).Else(NextState("STOP_WAIT")))
        fsm.act("STOP_WAIT", self.sda_low.eq(1), wait_high.eq(1),
            If(expired, *timeout()).Elif(self.scl_i,
                NextValue(timer, high_reload), NextState("STOP_HIGH")))
        fsm.act("STOP_HIGH", self.sda_low.eq(1),
            If(timer != 0, NextValue(timer, timer - 1)).Else(
                NextValue(timer, low_reload), NextState("STOP_FREE")))
        fsm.act("STOP_FREE", wait_high.eq(1),
            If(expired, *timeout()).Elif(~self.scl_i,
                NextValue(timer, low_reload),
            ).Elif(timer != 0, NextValue(timer, timer - 1)).Else(
                If(~self.sda_i & (self.error == 0),
                    NextValue(self.error, 3), NextValue(self.fail_index, wire_index)),
                NextState("FINISH")))
        fsm.act("FINISH", NextState("IDLE"))


class SharedCameraI2C(Module):
    """Two GPIO apertures and complete records sharing one byte buffer.

    Record metadata has the same contract as PacketFlashController: stable for
    each Wishbone word, and sourced before the guards. Out-of-band abort bits
    remain source-qualified even when no bus word is granted. The Ethernet
    receiver must already validate physical/UDP/record lengths before writes.
    """

    def __init__(self, pads=None, *, clk_freq=120_000_000, fast_low_cycles=180,
                 fast_high_cycles=120, stretch_timeout_cycles=3_000_000,
                 single_aperture=False, compact_record=False, use_dsp_state=False, use_dsp_watchdog=False):
        if clk_freq != 120_000_000:
            raise ValueError("I2C timing constants require the 120 MHz system clock")
        if pads is not None and len(pads) != 2:
            raise ValueError("exactly two camera pad groups are required")
        if single_aperture:
            self.bus = wishbone.Interface(data_width=32, addressing="word")
        else:
            self.buses = tuple(wishbone.Interface(data_width=32, addressing="word") for _ in range(2))
        self.scl = scl = tuple(TSTriple() for _ in range(2))
        self.sda = sda = tuple(TSTriple() for _ in range(2))
        self.control = controls = tuple(Signal(2) for _ in range(2))
        self.pin_status = pins = tuple(Signal(2) for _ in range(2))
        self.record_first, self.record_last = Signal(), Signal()
        self.record_count, self.record_source = Signal(9), Signal(2)
        self.record_abort_sources = Signal(4)
        self.submodules.engine = engine = I2CTransactionEngine(
            fast_low_cycles=fast_low_cycles, fast_high_cycles=fast_high_cycles,
            stretch_timeout_cycles=stretch_timeout_cycles, use_dsp_watchdog=use_dsp_watchdog)
        self.busy = Signal()
        owner, descriptor = Signal(), Signal(32)
        accepted_id, done_id, done_valid, accepted_read = Signal(32), Signal(32), Signal(), Signal()
        # A single byte-wide memory serves both the slow engine and serialized
        # host words. Validity bits preserve reset-to-zero byte semantics.
        buffer = _I2CByteMemory()
        write_port = buffer.get_port(write_capable=True, we_granularity=8)
        read_port = buffer.get_port()
        self.specials += buffer, write_port, read_port
        valid_bytes = Array(Signal(name=f"i2c_valid{index}") for index in range(16))
        read_valid = Signal()
        self.sync += read_valid.eq(valid_bytes[read_port.adr])
        for index in range(16):
            self.sync += If(write_port.we & (write_port.adr == index), valid_bytes[index].eq(1))
        host_state = Signal(2)  # 0 idle,1 payload writes,2 read issue,3 read capture
        host_owner, host_lane = Signal(), Signal(2)
        host_base = Signal(2)
        host_waiting, host_cancelled = Signal(), Signal()
        host_buffer_read = Signal()
        host_response = Signal(32)
        self.comb += [read_port.adr.eq(engine.tx_prefetch_index),
            write_port.adr.eq(engine.rx_index), write_port.dat_w.eq(engine.rx_data),
            write_port.we.eq(engine.rx_valid)]
        ingress_active = Signal()
        ingress_source, ingress_camera = Signal(2), Signal()
        ingress_count, ingress_index = Signal(3), Signal(3)
        candidate_id = Signal(32)
        self.comb += [self.busy.eq(engine.busy | ingress_active),
            engine.lease_wait.eq(ingress_active & ~engine.busy),
            engine.device.eq(descriptor[:7]), engine.read.eq(descriptor[7]),
            engine.register.eq(descriptor[8:24]), engine.count_minus_one.eq(descriptor[24:28]),
            engine.slow.eq(descriptor[28]), engine.tx_data.eq(Mux(read_valid, read_port.dat_r, 0)),
            engine.scl_i.eq(Array(pin[0] for pin in pins)[owner]),
            engine.sda_i.eq(Array(pin[1] for pin in pins)[owner])]
        if use_dsp_state:
            _capture_dsp(self, done_id, accepted_id, engine.done)
        else:
            self.sync += If(engine.done, done_id.eq(accepted_id))
        self.sync += If(engine.done, done_valid.eq(1))
        for index in range(2):
            owns = engine.busy & (owner == index)
            self.comb += [scl[index].o.eq(0), sda[index].o.eq(0)]
            # Physical bus edges must come from FFs, not binary FSM decode or
            # ownership mux glitches. Pin-high feedback already accounts for
            # this extra cycle before measuring every SCL-high interval.
            self.sync += [
                scl[index].oe.eq(Mux(owns, engine.scl_low, controls[index][0])),
                sda[index].oe.eq(Mux(owns, engine.sda_low, controls[index][1]))]
            self.specials += MultiReg(Cat(scl[index].i, sda[index].i), pins[index])
            if pads is not None:
                self.specials += [scl[index].get_tristate(pads[index].scl),
                                  sda[index].get_tristate(pads[index].sda)]

        selected = Signal()
        if single_aperture:
            bus = self.bus
            self.comb += host_waiting.eq(bus.cyc & bus.stb & (bus.adr[10] == host_owner))
            self.comb += selected.eq(Mux(host_state != 0, host_owner, bus.adr[10]))
            request = bus.cyc & bus.stb & ~bus.ack & ~bus.err & (host_state == 0)
        else:
            buses = self.buses
            requests = [bus.cyc & bus.stb & ~bus.ack & ~bus.err for bus in buses]
            self.comb += host_waiting.eq(Array(port.cyc & port.stb for port in buses)[host_owner])
            self.comb += selected.eq(Mux(host_state != 0, host_owner, ~requests[0]))
            request = (requests[0] | requests[1]) & (host_state == 0)
            bus = wishbone.Interface(data_width=32, addressing="word")
            self.comb += [bus.adr.eq(Mux(selected, buses[1].adr, buses[0].adr)),
                bus.dat_w.eq(Mux(selected, buses[1].dat_w, buses[0].dat_w)),
                bus.we.eq(Mux(selected, buses[1].we, buses[0].we)),
                bus.sel.eq(Mux(selected, buses[1].sel, buses[0].sel))]
        response, error_response, read_data = Signal(), Signal(), Signal(32)
        write_gpio = Signal()
        next_id = (accepted_id + Constant(1, 32))[:32]
        status = Cat(self.busy, done_valid, owner, accepted_read)
        result = Cat(engine.error, Constant(0, 6), engine.fail_index, Constant(0, 3),
                     engine.completed, Constant(0, 11))

        def read_only(value):
            return If(bus.we, error_response.eq(1)).Else(response.eq(1), read_data.eq(value))

        gpio_owned = ((engine.busy & (owner == selected))
                        | (ingress_active & (ingress_camera == selected)))
        cases = {
            0x00 // 4: [read_data.eq(Array(controls)[selected]),
                If(bus.we & bus.sel[0] & gpio_owned, error_response.eq(1)).Else(
                    response.eq(1), write_gpio.eq(bus.we & bus.sel[0]))],
            0x04 // 4: read_only(Array(pins)[selected] | I2C_CAPABILITY),
            0x10 // 4: read_only(I2C_MAGIC),
            0x14 // 4: read_only(status),
            0x18 // 4: read_only(descriptor),
            0x1c // 4: read_only(accepted_id),
            0x20 // 4: read_only(done_id),
            0x24 // 4: read_only(next_id),
            0x28 // 4: read_only(result),
            0x2c // 4: read_only(I2C_MAX_BYTES),
            "default": error_response.eq(1),
        }
        for index in range(4):
            # Only valid reads enter the byte sequencer; writes retain ERR.
            action = If(bus.we, error_response.eq(1)).Else(host_buffer_read.eq(1))
            cases[(0x40 // 4) + index] = action
            cases[(0x60 // 4) + index] = action
        for index in range(7):
            # Invalid/stale records drain harmlessly, rather than turning a
            # delayed duplicate into a host guard fault or another mutation.
            cases[(0x80 // 4) + index] = If(bus.we, response.eq(1)).Else(error_response.eq(1))
        self.comb += If(request, Case(bus.adr[:10], cases))

        abort_current = Array(self.record_abort_sources)[self.record_source]
        abort_owner = Array(self.record_abort_sources)[ingress_source]
        packet_word = (bus.adr[:10] >= 0x80 // 4) & (bus.adr[:10] <= 0x98 // 4) & bus.we
        same_source = self.record_source == ingress_source
        start_record, take_descriptor, take_payload, commit, clear_record = [Signal() for _ in range(5)]
        header_shape = ((bus.adr[:10] == 0x80 // 4) & (bus.sel == 15)
                        & (self.record_count >= 3) & (self.record_count <= 7)
                        & ~self.record_last)
        continuation_shape = ((selected == ingress_camera) & (bus.sel == 15)
            & (self.record_count == ingress_count)
            & (bus.adr[:10] == (0x80 // 4) + ingress_index)
            & (self.record_last == (ingress_index == ingress_count - 1)))
        expected_count = Mux(descriptor[7], 3, Constant(4, 4) + descriptor[26:28])
        register_end = Cat(descriptor[8:24], Constant(0, 1)) + descriptor[24:28]
        descriptor_ok = ((descriptor[29:32] == 0) & ~register_end[16]
                         & (ingress_count == expected_count))
        self.comb += [
            If(ingress_active & (abort_owner | engine.lease_expired), clear_record.eq(1)),
            If(request,
                # A foreign host cannot replace or corrupt the current source's
                # partial packet. An explicit new record from that source can.
                If(~ingress_active | same_source,
                    If(self.record_first,
                        clear_record.eq(ingress_active),
                        If(packet_word & header_shape & ~engine.busy & ~abort_current
                           & ~engine.lease_expired & (bus.dat_w == next_id)
                           & (Array(controls)[selected] == 0), start_record.eq(1)),
                    ).Elif(ingress_active,
                        If(abort_owner | engine.lease_expired | ~packet_word | ~continuation_shape,
                            clear_record.eq(1),
                        ).Elif(self.record_last,
                            clear_record.eq(1),
                            If((bus.dat_w == candidate_id) & (candidate_id == next_id)
                               & descriptor_ok & ~engine.busy
                               & (Array(controls)[ingress_camera] == 0), commit.eq(1)),
                        ).Elif(ingress_index == 1, take_descriptor.eq(1)).Else(take_payload.eq(1))
                    )
                )
            ), engine.start.eq(commit), engine.lease_reset.eq(start_record),
        ]
        # clear wins over continuation; a validated replacement header can
        # claim a fresh lease in the same cycle that its old one is discarded.
        self.sync += [
            If(take_descriptor, ingress_index.eq(ingress_index + 1)),
            If(take_payload, ingress_index.eq(ingress_index + 1)),
            If(clear_record, ingress_active.eq(0)),
            If(start_record, ingress_active.eq(1), ingress_source.eq(self.record_source),
                ingress_camera.eq(selected), ingress_count.eq(self.record_count[:3]),
                ingress_index.eq(1), done_valid.eq(0)),
            If(commit, owner.eq(ingress_camera),
                accepted_read.eq(descriptor[7]), done_valid.eq(0)),
        ]
        if use_dsp_state:
            _capture_dsp(self, descriptor, bus.dat_w, take_descriptor)
            if not compact_record:
                _capture_dsp(self, candidate_id, bus.dat_w, start_record)
            _capture_dsp(self, accepted_id, candidate_id, commit)
        else:
            self.sync += [If(take_descriptor, descriptor.eq(bus.dat_w)),
                          If(commit, accepted_id.eq(candidate_id))]
            if not compact_record:
                self.sync += If(start_record, candidate_id.eq(bus.dat_w))
        if compact_record:
            # Every accepted header equals next_id. accepted_id changes only
            # on a commit, which ends that record. While ingress is active,
            # its captured candidate ID therefore equals the unchanged next_id.
            # Alias the internal word; preserve the default register for generic
            # callers. Abort/lease discard does not change any public ID.
            self.comb += candidate_id.eq(next_id)
        # Capture all payload bytes and their location at logical acceptance.
        # A withdrawn host cycle may change every broadcast bus signal while
        # those bounded writes finish. Reuse the otherwise idle response word
        # as the three-byte shift register; its final shift restores write ACK
        # data to zero. Record aborts still clear ingress immediately.
        # Read withdrawal cancels immediately. Write withdrawal suppresses any
        # late ACK permanently for that word, even if a new cycle starts.
        self.comb += [
            If(take_payload,
                write_port.we.eq(1), write_port.adr.eq(Cat(Constant(0, 2), ingress_index - 2)),
                write_port.dat_w.eq(bus.dat_w[:8])),
            If(host_state == 1,
                write_port.we.eq(1), write_port.adr.eq(Cat(host_lane, host_base)),
                write_port.dat_w.eq(host_response[:8])),
            If((host_state == 2) & host_waiting & ~engine.tx_prefetch_priority,
                read_port.adr.eq(Cat(host_lane, host_base))),
        ]
        self.sync += [
            If(host_buffer_read,
                host_owner.eq(selected), host_lane.eq(0), host_state.eq(2),
                host_base.eq(bus.adr[:2]), host_cancelled.eq(0)),
            If(take_payload,
                host_owner.eq(selected), host_lane.eq(1), host_state.eq(1),
                host_base.eq(ingress_index - 2), host_cancelled.eq(0)),
            If(host_state == 1,
                If(host_lane == 3, host_state.eq(0)).Else(host_lane.eq(host_lane + 1))),
            If((host_state == 2) & ~engine.tx_prefetch_priority, host_state.eq(3)),
            If(host_state == 3,
                If(host_lane == 3, host_state.eq(0)).Else(
                    host_lane.eq(host_lane + 1), host_state.eq(2))),
            If((host_state != 0) & ~host_waiting,
                host_cancelled.eq(1),
                If(host_state != 1, host_state.eq(0))),
        ]
        self.sync += [If(request, host_response.eq(read_data)),
            If(take_payload, host_response.eq(bus.dat_w[8:32])),
            If(host_state == 1, host_response.eq(host_response[8:32])),
            If((host_state == 3) & host_waiting,
                Case(host_lane, {lane: host_response[lane * 8:lane * 8 + 8].eq(
                    Mux(read_valid, read_port.dat_r, 0)) for lane in range(4)}))]
        if single_aperture:
            self.comb += bus.dat_r.eq(host_response)
            self.sync += [bus.ack.eq(0), bus.err.eq(0),
                If(request, bus.ack.eq(response & ~take_payload), bus.err.eq(error_response)),
                If(host_waiting & ~host_cancelled & (host_state == 1) & (host_lane == 3), bus.ack.eq(1)),
                If(host_waiting & ~host_cancelled & (host_state == 3) & (host_lane == 3), bus.ack.eq(1))]
        else:
            for index, port in enumerate(buses):
                self.comb += port.dat_r.eq(host_response)
                self.sync += [port.ack.eq(0), port.err.eq(0),
                    If(request & (selected == index),
                        port.ack.eq(response & ~take_payload), port.err.eq(error_response)),
                    If((host_owner == index) & host_waiting & ~host_cancelled
                       & (host_state == 1) & (host_lane == 3), port.ack.eq(1)),
                    If((host_owner == index) & host_waiting & ~host_cancelled
                       & (host_state == 3) & (host_lane == 3), port.ack.eq(1))]
        for index in range(2):
            self.sync += If(write_gpio & (selected == index), controls[index].eq(bus.dat_w[:2]))

        register_names = (("control", 0x00, "rw"), ("pin_status", 0x04, "ro"),
            ("magic", 0x10, "ro"), ("status", 0x14, "ro"), ("descriptor", 0x18, "ro"),
            ("command_id", 0x1c, "ro"), ("done_id", 0x20, "ro"),
            ("next_id", 0x24, "ro"), ("result", 0x28, "ro"), ("max_bytes", 0x2c, "ro"))
        self.regions = {f"camera{index}_i2c_engine": SoCCSRRegion(base, 32,
            [MockCSR(name, 1, mode, origin=base + offset) for name, offset, mode in register_names]
            + [MockCSR("buffer", 4, "ro", origin=base + 0x60),
               MockCSR("command_packet", 7, "rw", origin=base + 0x80)])
            for index, base in enumerate(I2C_BASES)}
