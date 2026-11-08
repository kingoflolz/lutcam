"""One Wishbone read/write engine for the camera and packet control banks.

The exported register names remain LiteX-compatible, but their implementation
does not replicate a registered 32-bit readback mux for every packet stream.
Diagnostic counters use a shared RAM engine. Configuration has one copy
and may only be written while its camera is stopped and idle.
"""

from functools import reduce
from operator import or_

from migen import Cat, Case, Constant, If, Module, Mux, Replicate, Signal
from litex.soc.integration.export import MockCSR
from litex.soc.integration.soc import SoCCSRRegion
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import CSR, CSRStorage

from .camera import CALIBRATION, CALIBRATION_OFFSETS, CALIBRATION_RESERVED


CONTROL_BASE = 0x30024000
CAMERA_REGISTERS = (
    "run", "clear", "soft_reset", "sensor_control", "sensor_status", "status",
    "receiver_status", "current_frame", "frames_started", "frames_completed",
    "ingress_high_water", "solver_fifo_level", "solver_cycles",
    "calibration_commit", "calibration_valid", "configuration_error",
    "solver_debug_status", "solver_debug_protocol", "solver_debug_drop", "solver_debug_magic",
)
PACKET_REGISTERS = (
    "enable", "destination_ip", "destination_port", "active_destination_ip",
    "active_destination_port", "fifo_level", "high_water", "faults", "clear",
    "packets_committed", "packets_sent", "bytes_sent", "overflows",
    "protocol_errors", "rejections",
)


class IntegrationRegisters(Module):
    """Classic Wishbone slave and priority arbiter for counter snapshots.

    Camera banks occupy 0x000/0x100; calibration fields start at +0x80.
    A configuration bank can store active calibration in RAM while the solver
    reads individual words. RAM writes recheck the paired idle/drained guard at
    their commit edge; pair reset preserves settings, global reset clears them.
    Production cameras share tag geometry and acceptance-limit storage: their
    +0x90 through +0xb0 fields are read/write aliases. Intrinsics stay separate.
    Removed controls at +0x98/+0xa4/+0xa8/+0xb4 read zero and ignore writes.
    Optional paired acquisition controls occupy 0x200 and own reset/clear.
    Six packet banks occupy 0x400 + stream*0x80. All offsets are bytes.
    Names, widths and big-endian multiword ordering are exported in csr.csv.
    A request remains held until ACK; packet sequence queries have priority
    over software counter reads, and both include all preceding events.
    By default address, byte lanes, direction and data are captured before
    decoding. ``held_request=True`` instead uses the live classic bus and is
    qualified only for the main Etherbone and RecordUARTBone producers through
    WishboneGuard, the cycle arbiter and registered response-data decoder.
    Producers must hold ADR/WE/SEL/DAT_W through the sampled ACK edge, including
    guard-generated error/timeout ACK, or withdraw CYC before changing them.
    The guard's recovery/cancellation must provide a sampled downstream CYC-low
    edge before another owner arrives. All producers, guards, arbiter and this
    engine share the sys clock/reset. Shared readback alone does not establish
    this lifetime contract. The DECODE stage and a 22-bit merged timing scratch
    remain; generic capture also tolerates payload changes before its ACK.
    Dropping CYC or STB before the decode edge cancels its side effects;
    completed side effects are never replayed by the response stage.
    When shared_readback is provided, acknowledged responses use its common
    storage; inactive response data is unspecified. The default stays private.
    """

    def __init__(self, cameras, transport, counters, *, banked_destinations=False,
                 diagnostics=(), pair=None, configuration_bank=None, shared_readback=None, held_request=False):
        if configuration_bank is not None and pair is None:
            raise ValueError("RAM-backed calibration requires paired capture control")
        self.bus = bus = wishbone.Interface(data_width=32, addressing="word")
        self.regions = regions = {}
        # Generic mode captures payload to cut arbitration/address decode.
        # The qualified main-host mode reuses the held bus payload; both modes
        # retain live CYC/STB cancellation and identical commit/response edges.
        address = bus.adr[:10] if held_request else Signal(10, name="request_address")
        request_write = bus.we if held_request else Signal(name="request_write")
        request_select = bus.sel if held_request else Signal(4, name="request_select")
        request_data = bus.dat_w if held_request else Signal(32, name="request_data")
        state = Signal(4)
        IDLE, DECODE, COUNT_REQUEST, COUNT_WAIT, CONFIG_WRITE, ACK, CAL_READ_WAIT, CAL_WRITE, TIMING_WAIT = range(9)
        read_value = Signal(32) if shared_readback is None else shared_readback.word
        # Calibration and packet destinations commit in exclusive FSM states.
        # Both merge selected captured bytes with the same response scratch.
        merged_write_data = Signal(32)
        merge_input = Signal(32)
        merge_select = Signal(4)
        self.comb += [
            merge_input.eq(request_data), merge_select.eq(request_select),
            merged_write_data.eq(Cat(*(Mux(merge_select[byte],
                merge_input[byte * 8:byte * 8 + 8], read_value[byte * 8:byte * 8 + 8])
                for byte in range(4)))),
        ]
        response_error = Signal()
        request_counter = Signal(max=50)
        request = bus.cyc & bus.stb
        accept_simple = request & (state == DECODE)
        read_cases, write_cases = {}, {}
        timing_wait_operation = []
        counter_read = {}

        self.comb += [bus.ack.eq(request & (state == ACK) & ~response_error),
                      bus.err.eq(request & (state == ACK) & response_error),
                      bus.dat_r.eq(read_value)]

        def metadata(bank, name, offset, words=1, read_only=False):
            if bank not in regions:
                regions[bank] = SoCCSRRegion(CONTROL_BASE + offset, 32, [])
            regions[bank].obj.append(MockCSR(name, words, "ro" if read_only else "rw",
                                            origin=CONTROL_BASE + offset))

        if pair is not None:
            # Faulted pipelines need not assert idle. Recovery only requires
            # stopped/drained state; configuration additionally needs both idle.
            pair_stopped = ~pair.busy & pair.drained
            pair_paused = pair_stopped
            for camera in cameras:
                pair_paused = pair_paused & camera.idle
            start_allowed = pair.sync_enable & (pair.run_request |
                (~pair.busy & pair.ready & pair.locked & ~pair.fault))
            run_write = accept_simple & request_write & request_select[0] & (address == 0x208 // 4)
            self.comb += [pair.start.eq(run_write & request_data[0] & start_allowed),
                          pair.stop.eq(run_write & ~request_data[0])]
            pair_fields = {
                0x00: ("magic", 0x4c435031), 0x04: ("reserved", 0),
                0x08: ("run_request", pair.run_request), 0x0c: ("status", pair.status),
                0x14: ("fault_cause", pair.fault_cause),
                0x18: ("pairs_started", pair.pairs_started),
                0x1c: ("pairs_completed", pair.pairs_completed),
                0x20: ("watchdog_cycles", pair.watchdog_cycles),
                0x24: ("max_skew_cycles", pair.max_skew_cycles),
                0x2c: ("snap_epoch", pair.snap_epoch),
                0x30: ("snap_sof0_low", pair.snap_sof0[:32]),
                0x34: ("snap_sof0_high", pair.snap_sof0[32:64]),
                0x38: ("snap_sof1_low", pair.snap_sof1[:32]),
                0x3c: ("snap_sof1_high", pair.snap_sof1[32:64]),
                0x40: ("snap_skew", pair.snap_skew),
                0x44: ("snap_min_skew", pair.snap_min_skew),
                0x48: ("snap_max_skew", pair.snap_max_skew),
                0x4c: ("snap_sof0_count", pair.snap_sof0_count),
                0x50: ("snap_sof1_count", pair.snap_sof1_count),
                0x54: ("snap_sync_count", pair.snap_sync_count),
                0x58: ("snap_matched_pairs", pair.snap_matched_pairs),
                0x5c: ("snap_missing_events", pair.snap_missing_events),
                0x64: ("sync_period", pair.sync_period),
                0x68: ("sync_width", pair.sync_width),
                0x6c: ("sync_enable", pair.sync_enable),
                0x70: ("snap_sync_low", pair.snap_sync[:32]),
                0x74: ("snap_sync_high", pair.snap_sync[32:64]),
                0x78: ("snap_latency0", pair.snap_latency0),
                0x7c: ("snap_latency1", pair.snap_latency1),
                0x80: ("snap_min_latency", pair.snap_min_latency),
                0x84: ("snap_max_latency", pair.snap_max_latency),
                0x88: ("min_latency_cycles", pair.min_latency_cycles),
                0x8c: ("max_latency_cycles", pair.max_latency_cycles),
                0x90: ("phase0_cycles", pair.phase0_cycles),
                0x94: ("phase1_cycles", pair.phase1_cycles),
                0x98: ("snap_residual_skew", pair.snap_residual_skew),
                0x9c: ("snap_min_residual_skew", pair.snap_min_residual_skew),
                0xa0: ("snap_max_residual_skew", pair.snap_max_residual_skew),
            }
            for offset, (name, value) in sorted(pair_fields.items()):
                word = (0x200 + offset) // 4
                metadata("pair", name, word * 4,
                         read_only=offset not in (0x08, 0x20, 0x24, 0x64, 0x68, 0x6c,
                                                 0x88, 0x8c, 0x90, 0x94))
                read_cases[word] = read_value.eq(value)
                write_cases[word] = response_error.eq(1)
            write_cases[0x208 // 4] = If(request_select[0] & request_data[0] & ~start_allowed,
                                       response_error.eq(1))
            for offset, name in ((0x10, "reset"), (0x28, "snapshot"), (0x60, "clear")):
                word = (0x200 + offset) // 4
                allowed = 1 if name == "snapshot" else pair_stopped
                metadata("pair", name, word * 4)
                read_cases[word] = read_value.eq(0)
                write_cases[word] = If((request_select != 0) & ~allowed, response_error.eq(1))
                self.comb += getattr(pair, name).eq(
                    accept_simple & request_write & (request_select != 0) & (address == word) & allowed)
            timing_fields = ((0x20, "watchdog_cycles"), (0x24, "max_skew_cycles"),
                             (0x64, "sync_period"), (0x68, "sync_width"),
                             (0x88, "min_latency_cycles"), (0x8c, "max_latency_cycles"),
                             (0x90, "phase0_cycles"), (0x94, "phase1_cycles"))
            # One merge/validation datapath serves the eight 21/22-bit timing
            # registers. The ABI remains 32 bits; selected nonzero reserved bits
            # are rejected before narrowing, never silently truncated.
            timing_old = Signal(22)
            timing_word = Cat(timing_old, Constant(0, 10))
            timing_merged = Signal(32)
            timing_phase = Signal(3)
            timing_index = Signal(6)
            timing_field = address[:6]
            self.comb += [
                timing_index.eq(timing_field),
                If(state == TIMING_WAIT, Case(timing_phase, {
                    i: timing_index.eq((offset // 4) & 63)
                    for i, (offset, name) in enumerate(timing_fields)})),
                timing_old.eq(0),
                Case(timing_index, {(offset // 4) & 63: timing_old.eq(getattr(pair, name))
                                   for offset, name in timing_fields}),
                timing_merged.eq(Cat(*(Mux(request_select[byte], request_data[byte * 8:byte * 8 + 8],
                    timing_word[byte * 8:byte * 8 + 8]) for byte in range(4)))),
            ]
            candidate = Signal(22, name="timing_candidate") if held_request else request_data[:22]
            # One strict comparator services all bounds. Equality is forbidden
            # in both directions; zero is separately legal only for phases.
            timing_less = candidate < timing_old
            timing_equal = candidate == timing_old
            timing_check = Signal()
            self.comb += [
                timing_check.eq(1),
                Case(timing_phase, {
                    0: If(timing_field == 0x24 // 4, timing_check.eq(timing_less)),
                    1: If(timing_field == 0x20 // 4, timing_check.eq(~timing_less & ~timing_equal)),
                    2: If((timing_field == 0x68 // 4) | (timing_field == (0x8c // 4) & 63) |
                          (timing_field == (0x90 // 4) & 63) | (timing_field == (0x94 // 4) & 63),
                          timing_check.eq(timing_less)),
                    3: If(timing_field == 0x64 // 4, timing_check.eq(~timing_less & ~timing_equal)),
                    4: If(timing_field == (0x8c // 4) & 63, timing_check.eq(~timing_less & ~timing_equal)),
                    5: If(timing_field == (0x88 // 4) & 63, timing_check.eq(timing_less)).Elif(
                          timing_field == 0x64 // 4, timing_check.eq(~timing_less & ~timing_equal)),
                    6: If(timing_field == 0x64 // 4, timing_check.eq(~timing_less & ~timing_equal)),
                    7: If(timing_field == 0x64 // 4, timing_check.eq(~timing_less & ~timing_equal)),
                }),
            ]
            timing_allowed = pair_paused & ~pair.sync_enable
            timing_commits = {}
            for offset, name in timing_fields:
                register = getattr(pair, name)
                field_width = len(register)
                reserved_zero = (timing_merged[field_width:32] == 0)
                nonzero = 1 if name in ("phase0_cycles", "phase1_cycles") else (timing_merged[:22] != 0)
                write_cases[(0x200 + offset) // 4] = If(timing_allowed & reserved_zero & nonzero,
                    (candidate.eq(timing_merged[:22]) if held_request else request_data.eq(timing_merged)),
                    timing_phase.eq(0), state.eq(TIMING_WAIT),
                ).Else(response_error.eq(1))
                timing_commits[(offset // 4) & 63] = register.eq(candidate[:field_width])
            timing_wait_operation = [
                If(~timing_check, response_error.eq(1)),
                If(timing_phase == 7,
                    state.eq(ACK),
                    If(timing_allowed & timing_check & ~response_error,
                        Case(timing_field, timing_commits),
                    ).Else(response_error.eq(1)),
                ).Else(timing_phase.eq(timing_phase + 1))
            ]
            # Disabling a drained faulted pair must not require camera idle:
            # sticky faults hold idle low until the explicit evidence clear.
            sync_write_allowed = Mux(request_select[0] & request_data[0], pair_paused, pair_stopped)
            write_cases[0x26c // 4] = If(sync_write_allowed,
                If(request_select[0], pair.sync_enable.eq(request_data[0]))).Else(response_error.eq(1))
            regions["pair"].obj.sort(key=lambda register: register.origin)

        for index, camera in enumerate(cameras):
            base = index * 0x100
            bank = f"camera{index}"
            paused = camera.idle & ~camera.run.storage if pair is None else pair_paused
            for position, name in enumerate(CAMERA_REGISTERS):
                register = getattr(camera, name)
                word = (base + position * 4) // 4
                storage = isinstance(register, CSRStorage)
                strobe = isinstance(register, CSR)
                metadata(bank, name, word * 4, read_only=not (storage or strobe))
                if pair is not None and name == "run":
                    read_cases[word] = read_value.eq(pair.busy)
                    write_cases[word] = response_error.eq(1)
                elif pair is not None and name in ("clear", "soft_reset"):
                    self.comb += [register.re.eq(pair.clear if name == "clear" else pair.reset),
                                  register.r.eq(0)]
                    read_cases[word] = read_value.eq(0)
                    write_cases[word] = response_error.eq(1)
                elif pair is not None and name == "sensor_control":
                    read_cases[word] = read_value.eq(register.storage)
                    write_cases[word] = If(paused & ~(request_select[0] & request_data[1]),
                        If(request_select[0], register.storage.eq(request_data))).Else(response_error.eq(1))
                elif storage:
                    read_cases[word] = read_value.eq(register.storage)
                    write_cases[word] = If(request_select[0], register.storage.eq(request_data))
                elif strobe:
                    allowed = 1 if pair is None else paused
                    self.comb += [register.re.eq(accept_simple & request_write &
                        (address == word) & (request_select != 0) & allowed), register.r.eq(request_data)]
                    read_cases[word] = read_value.eq(0)
                    if pair is not None:
                        write_cases[word] = If(~paused, response_error.eq(1))
                else:
                    read_cases[word] = read_value.eq(register.status)
                    if pair is not None:
                        write_cases[word] = response_error.eq(1)
            for name, width, _default in CALIBRATION:
                word = (base + CALIBRATION_OFFSETS[name]) // 4
                words = (width + 31) // 32
                metadata(bank, "cfg_" + name, word * 4, words)
                for part in range(words):
                    if configuration_bank is None:
                        register = camera.configuration[name]
                        lo = (words - part - 1) * 32
                        hi = min(width, lo + 32)
                        read_cases[word + part] = read_value.eq(register[lo:hi])
                        writes = []
                        for byte in range((hi - lo + 7) // 8):
                            end = min(hi - lo, byte * 8 + 8)
                            writes.append(If(request_select[byte],
                                register[lo + byte * 8:lo + end].eq(request_data[byte * 8:end])))
                        write_cases[word + part] = If(paused, *writes).Else(response_error.eq(1))
                    else:
                        read_cases[word + part] = state.eq(Mux(configuration_bank.host_ready,
                            CAL_READ_WAIT, DECODE))
                        write_cases[word + part] = If(paused,
                            state.eq(Mux(configuration_bank.host_ready, CAL_READ_WAIT, DECODE))
                        ).Else(response_error.eq(1))
            # Removed fields retain only stateless reserved addresses.
            for offset in CALIBRATION_RESERVED:
                reserved_word = (base + offset) // 4
                name = "cfg_reserved" if offset == 0xb4 else f"cfg_reserved_{offset:02x}"
                metadata(bank, name, reserved_word * 4, read_only=True)
                read_cases[reserved_word] = read_value.eq(0)
            calibration_write = (address[5:] == (base + 0x80) // 0x80) & (
                (address[:5] < 6) | (address[:5] == 11) | (address[:5] == 12))
            self.comb += camera.configuration_write_error.eq(
                (accept_simple | ((state == CAL_WRITE) & request &
                                  (configuration_bank is not None))) &
                request_write & calibration_write & ~paused)
            regions[bank].obj.sort(key=lambda register: register.origin)

        cal_wait_operation, cal_write_operation = [], []
        if configuration_bank is not None:
            cal = configuration_bank
            # Camera words retain their offsets. The compact mean limit maps
            # to numeric word 1; reserved words are stateless holes.
            position = address[:5]
            region = (((address[5:] == 1) | (address[5:] == 3)) &
                      ((position < 6) | (position == 11) | (position == 12)))
            mean_limit, mass, baseline = position == 5, position == 11, position == 12
            host_index = Signal(4)
            self.comb += [
                host_index.eq(0),
                Case(position, {
                    0: host_index.eq(8 + Cat(Constant(0, 2), address[6])),
                    1: host_index.eq(9 + Cat(Constant(0, 2), address[6])),
                    2: host_index.eq(10 + Cat(Constant(0, 2), address[6])),
                    3: host_index.eq(11 + Cat(Constant(0, 2), address[6])),
                    4: host_index.eq(0), 5: host_index.eq(1),
                    11: host_index.eq(7), 12: host_index.eq(7),
                }),
                cal.host_address.eq(host_index),
                cal.host_read.eq(accept_simple & region & (~request_write | pair_paused)),
                cal.host_write.eq(request & (state == CAL_WRITE) & pair_paused),
            ]
            shifted_data = Mux(baseline, Cat(Constant(0, 8), request_data[:16], Constant(0, 8)),
                               request_data)
            shifted_select = Mux(baseline, Cat(Constant(0, 1), request_select[:2], Constant(0, 1)),
                                Mux(mass, Cat(request_select[0], Constant(0, 3)),
                                    Mux(mean_limit, Cat(request_select[:2], Constant(0, 2)), request_select)))
            self.comb += [
                If(state == CAL_WRITE,
                    merge_input.eq(shifted_data), merge_select.eq(shifted_select)),
                cal.host_write_data.eq(merged_write_data),
            ]
            cal_wait_operation = [If(cal.host_response_valid,
                read_value.eq(Mux(request_write, cal.host_read_data,
                    Mux(mass, cal.host_read_data[:8],
                        Mux(baseline, cal.host_read_data[8:24],
                            Mux(mean_limit, cal.host_read_data[:16], cal.host_read_data))))),
                state.eq(Mux(request_write, CAL_WRITE, ACK)))]
            cal_write_operation = [If(~pair_paused,
                state.eq(ACK), read_value.eq(0), response_error.eq(1)
            ).Elif(cal.host_ready, state.eq(ACK), read_value.eq(0))]

        # Remote-host diagnostics share this readback register as well. Their
        # values still belong to the guards, so clearing retains the existing
        # fault/count/last-address semantics and never acknowledges a retry.
        for index, (name, diagnostic) in enumerate(diagnostics):
            base = 0x800 + index * 0x80
            shared_count = getattr(diagnostic, "external_count", False)
            if shared_count:
                self.comb += [counters.host_event[index].eq(diagnostic.fault_event),
                              counters.clear_host[index].eq(diagnostic.clear.re)]
            for position, field in enumerate(("status", "address", "count", "clear")):
                register = getattr(diagnostic, field)
                word = base // 4 + position
                metadata(name, field, word * 4, read_only=field != "clear")
                if field == "clear":
                    self.comb += [register.re.eq(accept_simple & request_write &
                        (address == word) & (request_select != 0)), register.r.eq(request_data)]
                    read_cases[word] = read_value.eq(0)
                elif field == "count" and shared_count:
                    read_cases[word] = [request_counter.eq(48 + index), state.eq(COUNT_REQUEST)]
                else:
                    read_cases[word] = read_value.eq(register.status)
                if pair is not None and field != "clear":
                    write_cases[word] = response_error.eq(1)

        enables = [Signal(name=f"packet_enable{i}") for i in range(6)]
        for index, channel in enumerate(transport.channels):
            camera = index // 3
            base = 0x400 + index * 0x80
            product = ("image", "children", "pose")[index % 3]
            bank = f"cam{camera}_{product}"
            ip_write = address == base // 4 + 1
            port_write = address == base // 4 + 2
            paused = (cameras[camera].idle & ~cameras[camera].run.storage
                      if pair is None else pair_paused) & channel.configuration_idle
            clear_allowed = 1 if pair is None else pair_paused
            self.comb += [channel.enable.eq(enables[index]),
                channel.clear.eq((accept_simple & request_write & (request_select != 0) &
                                  (address == (base + 8 * 4) // 4) & clear_allowed) |
                                 (0 if pair is None else pair.clear))]
            if not banked_destinations:
                self.comb += [channel.destination_ip.eq(channel.active_destination_ip),
                    channel.destination_port.eq(channel.active_destination_port),
                    channel.destination_update.eq(accept_simple & request_write & paused & (ip_write | port_write))]
                for byte in range(4):
                    self.comb += If(ip_write & request_select[byte],
                        channel.destination_ip[byte * 8:byte * 8 + 8].eq(request_data[byte * 8:byte * 8 + 8]))
                for byte in range(2):
                    self.comb += If(port_write & request_select[byte],
                        channel.destination_port[byte * 8:byte * 8 + 8].eq(request_data[byte * 8:byte * 8 + 8]))
            self.sync += If(cameras[camera].soft_reset.re, enables[index].eq(0))
            for position, name in enumerate(PACKET_REGISTERS):
                if name == "rejections" and index % 3 != 2:
                    continue
                word = base // 4 + position
                metadata(bank, name, word * 4, read_only=position not in (0, 1, 2, 8))
                if pair is not None:
                    write_cases[word] = response_error.eq(1)
                if position == 0:
                    read_cases[word] = read_value.eq(enables[index])
                    update = If(request_select[0], enables[index].eq(request_data[0]))
                    write_cases[word] = update if pair is None else If(pair_paused, update).Else(response_error.eq(1))
                elif position in (1, 2):
                    if banked_destinations:
                        counter_read[word] = 36 + index * 2 + position - 1
                        write_cases[word] = If(paused,
                            request_counter.eq(36 + index * 2 + position - 1),
                            # Full words need no read; byte writes merge with
                            # the single active RAM value before storing it.
                            state.eq(Mux(request_select == 15, CONFIG_WRITE, COUNT_REQUEST)),
                        ).Else(response_error.eq(1))
                    else:
                        read_cases[word] = read_value.eq(channel.active_destination_ip if position == 1
                                                        else channel.active_destination_port)
                        write_cases[word] = If(~paused, response_error.eq(1))
                elif position in (3, 4) and banked_destinations:
                    counter_read[word] = 36 + index * 2 + position - 3
                elif position == 8:
                    read_cases[word] = read_value.eq(0)
                    if pair is not None:
                        write_cases[word] = If(~pair_paused, response_error.eq(1))
                elif position >= 9:
                    counter_read[word] = index * 6 + position - 9
                else:
                    read_cases[word] = read_value.eq(getattr(channel, name))
            event = counters.events[index]
            self.comb += [counters.committed_levels[index].eq(channel.counter_committed_level),
                event.committed.eq(channel.commit_event),
                event.sent.eq(channel.sent_event), event.bytes.eq(channel.bytes_event),
                event.overflow.eq(channel.overflow_event),
                event.protocol.eq(channel.protocol_event), event.reject.eq(channel.reject_event)]

        for index, camera in enumerate(cameras):
            self.comb += [counters.clear_camera[index].eq(camera.soft_reset.re),
                transport.camera_reset[index].eq(camera.soft_reset.re | (0 if pair is None else pair.clear))]
            if pair is not None:
                self.comb += counters.clear_statistics[index].eq(pair.clear)

        # A single query may be outstanding. Record ownership so a late RAM
        # response cannot be mistaken for a newly arriving software request.
        pending, sequence_owner = Signal(), Signal()
        counter_accept = counters.query_valid & counters.query_ready
        sequence_query = Signal(6)
        self.comb += sequence_query.eq(transport.sequence_stream * 6 + 1)
        if banked_destinations:
            phase = Signal(2)
            destination_ip, destination_port = Signal(32), Signal(16)
            self.comb += [
                If(phase != 2,
                    sequence_query.eq(36 + transport.sequence_stream * 2 + (phase == 1))),
                transport.sequence_valid.eq(pending & sequence_owner &
                    counters.query_response_valid & (phase == 2) & transport.sequence_request),
                transport.sequence_value.eq(counters.query_data),
                transport.sequence_destination_ip.eq(destination_ip),
                transport.sequence_destination_port.eq(destination_port),
                counters.config_write_valid.eq(request & (state == CONFIG_WRITE)),
                counters.config_write_address.eq(request_counter - 36),
                counters.config_write_data.eq(merged_write_data),
            ]
            self.sync += If(~transport.sequence_request, phase.eq(0)).Elif(
                pending & sequence_owner & counters.query_response_valid,
                Case(phase, {
                    0: [destination_ip.eq(counters.query_data), phase.eq(1)],
                    1: [destination_port.eq(counters.query_data), phase.eq(2)],
                    2: phase.eq(0),
                }))
        else:
            self.comb += [transport.sequence_valid.eq(pending & sequence_owner & counters.query_response_valid),
                          transport.sequence_value.eq(counters.query_data)]
        self.comb += [
            counters.query_valid.eq(~pending & (transport.sequence_request |
                ((state == COUNT_REQUEST) & request))),
            counters.query_address.eq(request_counter),
            If(transport.sequence_request,
                counters.query_address.eq(sequence_query)),
        ]
        self.sync += [
            If(counter_accept, pending.eq(1), sequence_owner.eq(transport.sequence_request)),
            If(counters.query_response_valid, pending.eq(0)),
        ]

        # Factor read-data selection into 256-byte banks. Writes retain their
        # independent 128-byte decode; neither request nor capture timing changes.
        # The explicit masked OR keeps local register selection shared across
        # banks without changing the edge at which live status is captured.
        selected_read = Signal(32)
        selected_banks = {}
        for word, action in list(read_cases.items()):
            if hasattr(action, 'l') and action.l is read_value:
                selected_banks.setdefault(word >> 6, []).append(
                    action.r & Replicate(address[:6] == (word & 63), 32))
                read_cases[word] = []
        self.comb += selected_read.eq(reduce(or_, [
            reduce(or_, values) & Replicate(address[6:] == bank, 32)
            for bank, values in selected_banks.items()
        ]))
        read_dispatch, write_dispatch = dict(read_cases), dict(write_cases)
        for word, counter_address in counter_read.items():
            read_dispatch[word] = [request_counter.eq(counter_address), state.eq(COUNT_REQUEST)]

        if banked_destinations:
            stream_index = address[5:8]
            position = address[:5]
            packet_region = (address[8:10] == 1) & (stream_index < 6)
            destination_read = (position >= 1) & (position <= 4)
            diagnostic_read = ((position >= 9) & (position <= 13)) | (
                (position == 14) & ((stream_index == 2) | (stream_index == 5)))
            diagnostic_index = Signal(6)
            self.comb += Case(Cat(position[:3], stream_index), {
                stream * 8 + local: diagnostic_index.eq(stream * 6 + local - 1)
                for stream in range(6) for local in range(1, 7)
            })
            # 36 + 2*stream + the IP/port bit, specialized to stream 0..5.
            destination_index = Cat(~position[0], stream_index[0],
                ~stream_index[1], stream_index[1] | stream_index[2], 0, 1)
            decoded_counter = Mux(destination_read, destination_index, diagnostic_index)
            read_operation = If(packet_region & (destination_read | diagnostic_read),
                request_counter.eq(decoded_counter), state.eq(COUNT_REQUEST),
            ).Else(Case(address, read_cases))
        else:
            read_operation = Case(address, read_dispatch)

        grouped_writes = {}
        for word, action in write_dispatch.items():
            grouped_writes.setdefault(word >> 5, {})[word & 31] = action
        write_operation = Case(address[5:], {
            bank: Case(address[:5], actions) for bank, actions in grouped_writes.items()
        })

        self.sync += [
            If(~request, state.eq(IDLE)).Elif(state == IDLE,
                *([address.eq(bus.adr[:10]), request_write.eq(bus.we),
                   request_select.eq(bus.sel), request_data.eq(bus.dat_w)] if not held_request else []),
                state.eq(DECODE),
            ).Elif(state == DECODE,
                state.eq(ACK), read_value.eq(0), response_error.eq(0),
                If(request_write, write_operation).Else(
                    read_value.eq(selected_read), read_operation),
            ).Elif(state == TIMING_WAIT,
                *timing_wait_operation,
            ).Elif(state == CAL_READ_WAIT,
                *cal_wait_operation,
            ).Elif(state == CAL_WRITE,
                *cal_write_operation,
            ).Elif(state == COUNT_REQUEST,
                If(counter_accept & ~transport.sequence_request, state.eq(COUNT_WAIT)),
            ).Elif(state == COUNT_WAIT,
                If(pending & ~sequence_owner & counters.query_response_valid,
                    read_value.eq(counters.query_data),
                    state.eq(Mux(request_write, CONFIG_WRITE, ACK) if banked_destinations else ACK)),
            ).Elif(state == CONFIG_WRITE,
                If(counters.config_write_ready if banked_destinations else 1, state.eq(ACK)),
            ).Elif(state == ACK, state.eq(IDLE)),
        ]

        if shared_readback is not None:
            shared_readback.attach(self)
