"""Paused configuration and counter arbitration through the real register bus."""

from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from migen import If, Module, Signal
from migen.sim import passive, run_simulation
from litex.soc.interconnect.csr import CSR, CSRStatus, CSRStorage

from lutcam.camera import CALIBRATION, CALIBRATION_OFFSETS, CALIBRATION_RESERVED
from lutcam.counters import SharedPacketCounters
from lutcam.packet import SharedPacketTransport
from lutcam.registers import CAMERA_REGISTERS, IntegrationRegisters
from test_packet import Capture, child_group, idle


def packet_address(channel, position):
    return 0x400 + channel * 0x80 + position * 4


def camera_address(camera, name):
    return camera * 0x100 + CAMERA_REGISTERS.index(name) * 4


def calibration_addresses(camera):
    return {name: camera * 0x100 + offset for name, offset in CALIBRATION_OFFSETS.items()}


def transfer(bus, address, data=None, sel=15, limit=300):
    """One held classic transaction, observing both ACK and ERR."""
    yield bus.adr.eq(address // 4)
    yield bus.dat_w.eq(0 if data is None else data)
    yield bus.sel.eq(sel)
    yield bus.we.eq(data is not None)
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)
    yield
    for _ in range(limit):
        ack, error = (yield bus.ack), (yield bus.err)
        assert not (ack and error), "register bus asserted ACK and ERR together"
        if ack or error:
            result = ((yield bus.dat_r), error)
            yield bus.cyc.eq(0)
            yield bus.stb.eq(0)
            yield
            return result
        yield
    raise AssertionError("register access did not terminate")


class FakeCamera(Module):
    """Camera control surfaces; packet queues and counter service remain real."""

    def __init__(self, detector_configuration=None):
        self.idle = Signal(reset=1)
        self.configuration_write_error = Signal()
        for name in CAMERA_REGISTERS:
            if name in ("clear", "soft_reset", "calibration_commit"):
                register = CSR(name=name)
            elif name in ("run", "sensor_control"):
                register = CSRStorage(1 if name == "run" else 3, name=name)
            else:
                register = CSRStatus(32, name=name)
            setattr(self, name, register)
        self.configuration = {name: Signal(width, reset=default, name=name)
                              for name, width, default in CALIBRATION}
        if detector_configuration is not None:
            self.configuration.update(detector_configuration)
        self.sync += [
            If(self.clear.re, self.configuration_error.status.eq(0)),
            If(self.configuration_write_error, self.configuration_error.status.eq(1)),
            If(self.calibration_commit.re,
                If(self.idle & ~self.run.storage,
                    self.calibration_valid.status.eq(1),
                ).Else(self.configuration_error.status.eq(1))),
        ]


class DelayedCounterResponse(Module):
    """Rephase the precise reset-overlap fixture after request staging.

    Only the response is delayed; the real counter engine still owns query
    acceptance and stored values. Other fixtures retain its native latency.
    """

    def __init__(self, **kwargs):
        self.submodules.core = core = SharedPacketCounters(**kwargs)
        for name in (
            "query_valid", "query_ready", "query_address", "events",
            "committed_levels", "clear_camera", "clear_statistics",
            "config_write_valid", "config_write_ready", "config_write_address",
            "config_write_data", "host_event", "clear_host",
        ):
            if hasattr(core, name):
                setattr(self, name, getattr(core, name))
        self.query_response_valid = Signal()
        self.query_data = Signal(32)
        self.sync += [self.query_response_valid.eq(core.query_response_valid),
                      self.query_data.eq(core.query_data)]


class RegisterHarness(Module):
    def __init__(self, banked_destinations=False, host_diagnostics=False,
                 counter_response_delay=False):
        self.cameras = [FakeCamera(), FakeCamera()]
        self.submodules.camera0, self.submodules.camera1 = self.cameras
        self.submodules.transport = SharedPacketTransport(image_slots=2, child_slots=2,
            pose_slots=2, frame_words=16, fragment_words=8,
            external_destination=banked_destinations, external_context_held=banked_destinations)
        counter_class = DelayedCounterResponse if counter_response_delay else SharedPacketCounters
        self.submodules.counters = counter_class(packet_totals=True,
            with_destinations=banked_destinations, fixed_packet_layout=True,
            with_host_counters=host_diagnostics)
        diagnostics = []
        if host_diagnostics:
            from lutcam.control import WishboneGuard
            from lutcam.soc import HostDiagnostics
            self.guards = []
            for index, name in enumerate(("etherbone_diagnostics", "usb_diagnostics")):
                guard = WishboneGuard(timeout_cycles=8, external_count=True)
                diagnostic = HostDiagnostics(guard, direct_control=True)
                setattr(self.submodules, f"guard{index}", guard)
                setattr(self.submodules, name, diagnostic)
                self.comb += guard.master.err.eq(guard.master.cyc & guard.master.stb)
                self.guards.append(guard)
                diagnostics.append((name, diagnostic))
        self.submodules.registers = IntegrationRegisters(self.cameras, self.transport, self.counters,
            banked_destinations=banked_destinations, diagnostics=diagnostics)


class IntegrationRegisterTests(unittest.TestCase):
    def test_host_diagnostics_share_readback_and_clear_only_selected_guard(self):
        dut = RegisterHarness(banked_destinations=True, host_diagnostics=True)

        def bench():
            for index, guard in enumerate(dut.guards):
                # Produce real downstream errors through both guarded hosts.
                address = 0x30028000 + index * 4
                result = yield from transfer(guard.bus, address, 7 if index == 0 else None)
                self.assertEqual(result, (0xFFFFFFFF, 0))
                base = 0x800 + index * 0x80
                self.assertEqual((yield from transfer(dut.registers.bus, base)),
                                 (11 if index == 0 else 3, 0))
                self.assertEqual((yield from transfer(dut.registers.bus, base + 4)), (address, 0))
                self.assertEqual((yield from transfer(dut.registers.bus, base + 8)), (1, 0))
            yield from transfer(dut.registers.bus, 0x80C, 1)
            yield from idle(16)
            self.assertEqual((yield from transfer(dut.registers.bus, 0x808)), (0, 0))
            self.assertEqual((yield from transfer(dut.registers.bus, 0x804)), (0, 0))
            self.assertEqual((yield from transfer(dut.registers.bus, 0x888)), (1, 0))

        run_simulation(dut, bench())

    def test_direct_calibration_byte_lanes_and_rejected_writes(self):
        dut = RegisterHarness()
        bus = dut.registers.bus

        def bench():
            for camera in range(2):
                cfg = calibration_addresses(camera)
                self.assertEqual((yield from transfer(bus, cfg["max_mass_spread"])), (19, 0))
                self.assertEqual((yield from transfer(bus, cfg["min_baseline_q4"])), (16, 0))
                yield from transfer(bus, cfg["inv_fx_q2_30"], 0x12345678)
                yield from transfer(bus, cfg["inv_fx_q2_30"], 0xAABBCCDD, sel=0b0101)
                self.assertEqual((yield dut.cameras[camera].configuration["inv_fx_q2_30"]), 0x12BB56DD)
                self.assertEqual((yield from transfer(bus, cfg["inv_fx_q2_30"])), (0x12BB56DD, 0))
                # The mean-error limit is a single unsigned Q16 halfword.
                self.assertEqual((yield from transfer(bus, cfg["mean_residual_limit_q16"])), (1024, 0))
                yield from transfer(bus, cfg["mean_residual_limit_q16"], 0x12345678)
                yield from transfer(bus, cfg["mean_residual_limit_q16"], 0xabcdef12, sel=5)
                self.assertEqual((yield from transfer(bus, cfg["mean_residual_limit_q16"])), (0x5612, 0))
                yield from transfer(bus, cfg["mean_residual_limit_q16"], 0xffffffff, sel=12)
                self.assertEqual((yield dut.cameras[camera].configuration["mean_residual_limit_q16"]), 0x5612)
                # Narrow fields ignore nonexistent upper byte lanes.
                yield from transfer(bus, cfg["max_mass_spread"], 0xFFFFFFFF, sel=14)
                self.assertEqual((yield from transfer(bus, cfg["max_mass_spread"])), (19, 0))

                yield from transfer(bus, camera_address(camera, "run"), 1)
                self.assertEqual((yield from transfer(bus, cfg["inv_fx_q2_30"], 0xFFFFFFFF))[1], 1)
                self.assertEqual((yield from transfer(bus, cfg["inv_fx_q2_30"])), (0x12BB56DD, 0))
                self.assertEqual((yield dut.cameras[camera].configuration_error.status), 1)
                yield from transfer(bus, camera_address(camera, "run"), 0)
                yield dut.cameras[camera].idle.eq(0)
                yield
                self.assertEqual((yield from transfer(bus, cfg["mean_residual_limit_q16"], 0))[1], 1)
                self.assertEqual((yield from transfer(bus, cfg["mean_residual_limit_q16"])), (0x5612, 0))
                yield dut.cameras[camera].idle.eq(1)
                yield
                yield from transfer(bus, camera_address(camera, "clear"), 1)
                self.assertEqual((yield dut.cameras[camera].configuration_error.status), 0)
                yield from transfer(bus, camera_address(camera, "calibration_commit"), 1)
                self.assertEqual((yield dut.cameras[camera].calibration_valid.status), 1)

        run_simulation(dut, bench())

    def test_reserved_calibration_word_reads_zero_and_ignores_writes(self):
        dut = RegisterHarness()
        bus = dut.registers.bus
        for camera in range(2):
            registers = {register.name: register
                         for register in dut.registers.regions[f"camera{camera}"].obj}
            self.assertNotIn("cfg_ambiguity_margin_q32", registers)
            for offset in CALIBRATION_RESERVED:
                name = "cfg_reserved" if offset == 0xb4 else f"cfg_reserved_{offset:02x}"
                self.assertEqual(registers[name].origin, 0x30024000 + camera * 0x100 + offset)
                self.assertEqual(registers[name].type, "ro")

        def bench():
            for camera in range(2):
                cfg = calibration_addresses(camera)
                for idle, run in ((1, 0), (0, 0), (0, 1), (1, 1)):
                    yield dut.cameras[camera].idle.eq(idle)
                    yield from transfer(bus, camera_address(camera, "run"), run)
                    for offset in CALIBRATION_RESERVED:
                        reserved_address = camera * 0x100 + offset
                        for value, select in ((0, 15), (1, 1), (0xffffffff, 15), (0xffffffff, 14)):
                            self.assertEqual((yield from transfer(bus, reserved_address, value, sel=select)), (0, 0))
                            self.assertEqual((yield from transfer(bus, reserved_address)), (0, 0))
                    self.assertEqual((yield dut.cameras[camera].configuration_error.status), 0)
                    self.assertEqual((yield from transfer(bus, cfg["mean_residual_limit_q16"])), (1024, 0))
                    self.assertEqual((yield from transfer(bus, cfg["min_baseline_q4"])), (16, 0))

        run_simulation(dut, bench())


    def test_destinations_are_active_aliases_with_paused_and_queue_empty_guards(self):
        dut = RegisterHarness()
        bus, transport = dut.registers.bus, dut.transport
        drain = {"on": False}
        capture = Capture(transport.source, ready=lambda _: drain["on"])

        def bench():
            for index, channel in enumerate(transport.channels):
                for position, value in ((1, 0), (2, 6000 + index), (3, 0), (4, 6000 + index)):
                    self.assertEqual((yield from transfer(bus, packet_address(index, position))), (value, 0))
                yield from transfer(bus, packet_address(index, 1), 0xC0A80102 + index)
                yield from transfer(bus, packet_address(index, 1), 0x00120034, sel=0b0101)
                expected_ip = (0xC0A80102 + index) & 0xFF00FF00 | 0x00120034
                self.assertEqual((yield from transfer(bus, packet_address(index, 1))), (expected_ip, 0))
                self.assertEqual((yield from transfer(bus, packet_address(index, 3))), (expected_ip, 0))
                yield from transfer(bus, packet_address(index, 2), 0xAABB1234)
                yield from transfer(bus, packet_address(index, 2), 0xDEADBEEF, sel=12)
                self.assertEqual((yield from transfer(bus, packet_address(index, 2))), (0x1234, 0))
                yield from transfer(bus, packet_address(index, 2), 0x00005600, sel=2)
                self.assertEqual((yield from transfer(bus, packet_address(index, 4))), (0x5634, 0))
                camera = index // 3
                yield from transfer(bus, camera_address(camera, "run"), 1)
                self.assertEqual((yield from transfer(bus, packet_address(index, 1), 0xFFFFFFFF))[1], 1)
                yield from transfer(bus, camera_address(camera, "run"), 0)
                yield dut.cameras[camera].idle.eq(0)
                yield
                self.assertEqual((yield from transfer(bus, packet_address(index, 2), 1))[1], 1)
                yield dut.cameras[camera].idle.eq(1)
                yield
                self.assertEqual((yield channel.active_destination_ip), expected_ip)
                self.assertEqual((yield channel.active_destination_port), 0x5634)

            channel = transport.channels[1]
            old_ip, old_port = (yield channel.active_destination_ip), (yield channel.active_destination_port)
            yield from transfer(bus, packet_address(1, 0), 1)
            yield from child_group(channel, [0x100000001, 7], 55)
            yield from idle(5)
            self.assertEqual((yield channel.configuration_idle), 0)
            self.assertEqual((yield from transfer(bus, packet_address(1, 1), 0x0A010203))[1], 1)
            self.assertEqual((yield from transfer(bus, packet_address(1, 2), 9000))[1], 1)
            self.assertEqual((yield channel.active_destination_ip), old_ip)
            self.assertEqual((yield channel.active_destination_port), old_port)
            drain["on"] = True
            for _ in range(150):
                if capture.packets:
                    break
                yield
            self.assertEqual(len(capture.packets), 1)
            self.assertEqual(capture.metadata[0][1:], (old_ip, old_port))
            yield from idle(3)
            self.assertEqual((yield from transfer(bus, packet_address(1, 1), 0x0A010203))[1], 0)
            self.assertEqual((yield channel.active_destination_ip), 0x0A010203)

        run_simulation(dut, [bench(), capture.run()])

    def test_counter_queries_share_sequence_service_without_misrouting_responses(self):
        dut = RegisterHarness()
        bus, transport = dut.registers.bus, dut.transport
        capture = Capture(transport.source)
        priority_accepts = []

        @passive
        def watch():
            while True:
                if ((yield bus.cyc) and (yield transport.sequence_request)
                        and (yield dut.counters.query_valid) and (yield dut.counters.query_ready)):
                    priority_accepts.append((yield dut.counters.query_address))
                yield

        def bench():
            yield from transfer(bus, packet_address(1, 0), 1)
            for frame in range(4):
                yield from child_group(transport.channels[1], [frame + 1], 100 + frame)
                # Start a software query at the same time as packet selection.
                result = yield from transfer(bus, packet_address(1, 9))
                self.assertEqual(result, (frame + 1, 0))
                for _ in range(200):
                    if len(capture.packets) == frame + 1:
                        break
                    yield
                self.assertEqual(len(capture.packets), frame + 1)
                yield from idle(2)
                self.assertEqual((yield from transfer(bus, packet_address(1, 10))), (frame + 1, 0))
                self.assertEqual((yield from transfer(bus, packet_address(1, 11))), ((frame + 1) * 40, 0))
            self.assertTrue(priority_accepts, "test failed to exercise query contention")
            self.assertTrue(all(address == 7 for address in priority_accepts))
            self.assertEqual([struct.unpack(">8I", packet[:32])[3] for packet in capture.packets], [0, 1, 2, 3])

        run_simulation(dut, [bench(), capture.run(), watch()])

    def test_soft_reset_is_per_camera_and_resets_packet_controls_and_counters(self):
        dut = RegisterHarness()
        bus, transport = dut.registers.bus, dut.transport

        def bench():
            # Queues remain full/stalled until reset, so only committed counts
            # increase. A reset is explicitly allowed to abort such a packet.
            yield transport.source.ready.eq(0)
            for index in range(6):
                yield from transfer(bus, packet_address(index, 0), 1)
                yield from transfer(bus, packet_address(index, 1), 0xC0A80201 + index)
                yield from transfer(bus, packet_address(index, 2), 7000 + index)
            for camera in range(2):
                yield from child_group(transport.channels[camera * 3 + 1], [1, 2], camera + 10)
                yield from idle(3)
                self.assertEqual((yield from transfer(bus, packet_address(camera * 3 + 1, 9))), (1, 0))

            for resetting in range(2):
                yield from transfer(bus, camera_address(resetting, "soft_reset"), 1)
                yield from idle(3)
                for index in range(resetting * 3, resetting * 3 + 3):
                    channel = transport.channels[index]
                    self.assertEqual((yield from transfer(bus, packet_address(index, 0))), (0, 0))
                    self.assertEqual((yield channel.active_destination_ip), 0)
                    self.assertEqual((yield channel.active_destination_port), 6000 + index)
                    self.assertEqual((yield channel.fifo_level), 0)
                    for counter in range(9, 14):
                        self.assertEqual((yield from transfer(bus, packet_address(index, counter))), (0, 0))
                if resetting == 0:
                    self.assertEqual((yield from transfer(bus, packet_address(4, 9))), (1, 0))
                    self.assertEqual((yield from transfer(bus, packet_address(4, 0))), (1, 0))
                    self.assertEqual((yield transport.channels[4].active_destination_ip), 0xC0A80205)
                    self.assertEqual((yield transport.channels[4].fifo_level), 1)

        run_simulation(dut, bench())


class BankedDestinationRegisterTests(unittest.TestCase):
    def test_defaults_partial_writes_paused_guards_and_per_camera_reset(self):
        dut = RegisterHarness(banked_destinations=True)
        bus = dut.registers.bus

        def bench():
            for index in range(6):
                for position, value in ((1, 0), (2, 6000 + index), (3, 0), (4, 6000 + index)):
                    self.assertEqual((yield from transfer(bus, packet_address(index, position))), (value, 0))
                self.assertEqual((yield from transfer(bus, packet_address(index, 1), 0xC0A80102 + index))[1], 0)
                self.assertEqual((yield from transfer(bus, packet_address(index, 1), 0x12345678, sel=5))[1], 0)
                expected_ip = 0xC0340178
                self.assertEqual((yield from transfer(bus, packet_address(index, 1))), (expected_ip, 0))
                self.assertEqual((yield from transfer(bus, packet_address(index, 3))), (expected_ip, 0))
                yield from transfer(bus, packet_address(index, 1), 0xFFFFFFFF, sel=0)
                self.assertEqual((yield from transfer(bus, packet_address(index, 3))), (expected_ip, 0))
                yield from transfer(bus, packet_address(index, 2), 0xAABB1234)
                yield from transfer(bus, packet_address(index, 2), 0xFFFF0000, sel=12)
                yield from transfer(bus, packet_address(index, 2), 0x00005600, sel=2)
                self.assertEqual((yield from transfer(bus, packet_address(index, 2))), (0x5634, 0))
                self.assertEqual((yield from transfer(bus, packet_address(index, 4))), (0x5634, 0))
                camera = index // 3
                yield from transfer(bus, camera_address(camera, "run"), 1)
                self.assertEqual((yield from transfer(bus, packet_address(index, 1), 0xFFFFFFFF))[1], 1)
                yield from transfer(bus, camera_address(camera, "run"), 0)
                yield dut.cameras[camera].idle.eq(0)
                yield
                self.assertEqual((yield from transfer(bus, packet_address(index, 2), 1, sel=1))[1], 1)
                yield dut.cameras[camera].idle.eq(1)
                yield
                self.assertEqual((yield from transfer(bus, packet_address(index, 3))), (expected_ip, 0))
                self.assertEqual((yield from transfer(bus, packet_address(index, 4))), (0x5634, 0))
                yield from transfer(bus, packet_address(index, 0), 1)

            for camera in range(2):
                yield from transfer(bus, camera_address(camera, "soft_reset"), 1)
                for index in range(camera * 3, camera * 3 + 3):
                    self.assertEqual((yield from transfer(bus, packet_address(index, 0))), (0, 0))
                    for position, value in ((1, 0), (2, 6000 + index), (3, 0), (4, 6000 + index)):
                        self.assertEqual((yield from transfer(bus, packet_address(index, position))), (value, 0))
                if camera == 0:
                    for index in range(3, 6):
                        self.assertEqual((yield from transfer(bus, packet_address(index, 0))), (1, 0))
                        self.assertEqual((yield from transfer(bus, packet_address(index, 3))), (0xC0340178, 0))
                        self.assertEqual((yield from transfer(bus, packet_address(index, 4))), (0x5634, 0))

        run_simulation(dut, bench())

    def test_packet_metadata_and_sequence_survive_software_query_contention(self):
        dut = RegisterHarness(banked_destinations=True)
        bus, transport = dut.registers.bus, dut.transport
        drain = {"on": False}
        capture = Capture(transport.source, ready=lambda _: drain["on"])
        priority_accepts = []

        @passive
        def watch():
            while True:
                if ((yield bus.cyc) and (yield transport.sequence_request)
                        and (yield dut.counters.query_valid) and (yield dut.counters.query_ready)):
                    priority_accepts.append(((yield transport.sequence_stream),
                                             (yield dut.counters.query_address)))
                yield

        def bench():
            for index in (1, 4):
                yield from transfer(bus, packet_address(index, 1), 0xC0A80200 + index)
                yield from transfer(bus, packet_address(index, 2), 7000 + index)
                yield from transfer(bus, packet_address(index, 0), 1)
            count = 0
            for frame in range(3):
                for index in (1, 4):
                    drain["on"] = False
                    yield from child_group(transport.channels[index], [frame + 1], 100 + frame)
                    self.assertEqual((yield from transfer(bus, packet_address(index, 9))), (frame + 1, 0))
                    # Paused cameras still cannot change a queued packet's target.
                    self.assertEqual((yield from transfer(bus, packet_address(index, 1), 0xFFFFFFFF))[1], 1)
                    self.assertEqual((yield from transfer(bus, packet_address(index, 2), 0xFF, sel=1))[1], 1)
                    self.assertEqual((yield from transfer(bus, packet_address(index, 3))), (0xC0A80200 + index, 0))
                    drain["on"] = True
                    for _ in range(200):
                        if len(capture.packets) == count + 1:
                            break
                        yield
                    self.assertEqual(len(capture.packets), count + 1)
                    self.assertEqual(capture.metadata[count][1:], (0xC0A80200 + index, 7000 + index))
                    self.assertEqual(struct.unpack(">8I", capture.packets[count][:32])[3], frame)
                    count += 1
                    yield from idle(2)
                    self.assertEqual((yield from transfer(bus, packet_address(index, 10))), (frame + 1, 0))
                    self.assertEqual((yield from transfer(bus, packet_address(index, 11))), ((frame + 1) * 40, 0))
            for index in (1, 4):
                self.assertTrue({index * 6 + 1, 36 + index * 2, 37 + index * 2}.issubset(
                    {address for stream, address in priority_accepts if stream == index}),
                    "each of the three packet-context queries must contend with software")

        run_simulation(dut, [bench(), capture.run(), watch()])

    def test_soft_reset_cancels_each_inflight_context_query(self):
        # A reset may interrupt the sent-count, IP, or port query. Another
        # camera's next packet must receive its own complete context afterward.
        for reset_query in (7, 38, 39):
            with self.subTest(reset_query=reset_query):
                # Request staging adds one bus cycle. Delay the counter
                # response equally so reset still lands on that response;
                # retain the overlap assertion and all recovery checks below.
                dut = RegisterHarness(banked_destinations=True, counter_response_delay=True)
                bus, transport = dut.registers.bus, dut.transport
                capture = Capture(transport.source)
                reset_with_response = []

                @passive
                def watch_reset():
                    while True:
                        if ((yield dut.cameras[0].soft_reset.re)
                                and (yield dut.counters.query_response_valid)):
                            reset_with_response.append(True)
                        yield

                def bench():
                    for index in (1, 4):
                        yield from transfer(bus, packet_address(index, 1), 0xC0A80200 + index)
                        yield from transfer(bus, packet_address(index, 2), 7000 + index)
                        yield from transfer(bus, packet_address(index, 0), 1)
                    yield from child_group(transport.channels[1], [1], 100)
                    for _ in range(60):
                        if ((yield dut.counters.query_valid) and (yield dut.counters.query_ready)
                                and (yield dut.counters.query_address) == reset_query):
                            break
                        yield
                    else:
                        self.fail("target packet-context query did not occur")
                    # The accepted query's response overlaps the reset on the
                    # next cycle; the monitor proves that overlap occurred.
                    yield from transfer(bus, camera_address(0, "soft_reset"), 1)
                    self.assertTrue(reset_with_response, "reset must cancel an in-flight response")
                    yield from child_group(transport.channels[4], [2], 200)
                    self.assertEqual((yield from transfer(bus, packet_address(4, 9))), (1, 0))
                    for _ in range(200):
                        if capture.packets:
                            break
                        yield
                    self.assertEqual(len(capture.packets), 1)
                    self.assertEqual(capture.metadata[0][1:], (0xC0A80204, 7004))
                    self.assertEqual(struct.unpack(">8I", capture.packets[0][:32])[3], 0)
                    self.assertEqual((yield from transfer(bus, packet_address(1, 9))), (0, 0))
                    self.assertEqual((yield from transfer(bus, packet_address(1, 10))), (0, 0))
                    self.assertEqual((yield from transfer(bus, packet_address(1, 3))), (0, 0))
                    self.assertEqual((yield from transfer(bus, packet_address(1, 4))), (6001, 0))
                    # Same-camera restart must also use the reset defaults.
                    yield from transfer(bus, packet_address(1, 0), 1)
                    yield from child_group(transport.channels[1], [3], 300)
                    for _ in range(200):
                        if len(capture.packets) == 2:
                            break
                        yield
                    self.assertEqual(len(capture.packets), 2)
                    self.assertEqual(capture.metadata[1][1:], (0, 6001))
                    self.assertEqual(struct.unpack(">8I", capture.packets[1][:32])[3], 0)

                run_simulation(dut, [bench(), capture.run(), watch_reset()])


if __name__ == "__main__":
    unittest.main()
