"""UARTBone response boundaries through the actual CDC and LUNA packetizer."""

from pathlib import Path
from types import SimpleNamespace
import gc
import os
import sys
import unittest
from unittest.mock import patch
import warnings

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from migen import ClockDomain, ClockSignal, If, Memory, ResetSignal, Signal
from migen.fhdl.structure import _Assign
from migen.sim import passive, run_simulation
from litex.gen import LiteXModule
from litex.build.amaranth2v_converter import Amaranth2VConverter
from litex_luna import LunaCDCACM
from litex.soc.cores.uart import UARTBone
from litex.soc.interconnect import stream

import amaranth as a
from amaranth.hdl import UnusedElaboratable
from amaranth.sim import Simulator as AmaranthSimulator
from luna.gateware.interface.utmi import UTMIInterface, UTMIOperatingMode
from luna.gateware.interface.gateware_phy.phy import GatewarePHY
from luna.gateware.interface.gateware_phy.transmitter import TxNRZIEncoder, TxPipeline
from luna.gateware.usb.devices.acm import USBSerialDevice
from luna.gateware.usb.request.standard import StandardRequestHandler
from luna.gateware.usb.usb2.descriptor import (
    GetDescriptorHandlerBlock, GetDescriptorHandlerDistributed,
)
from luna.gateware.usb.usb2.device import USBDevice
from luna.gateware.usb.usb2.endpoint import EndpointInterface
from luna.gateware.usb.usb2.endpoints.stream import USBStreamInEndpoint, USBStreamOutEndpoint
from luna.gateware.usb.usb2.transfer import USBInTransferManager
from luna.gateware.test.contrib import usb_packet
from luna.gateware.usb.usb2 import USBPacketID
from usb_protocol.emitters import DeviceDescriptorCollection

from lutcam.usb import USBConnectDelay


def luna_wrapper():
    """Construct the pinned wrapper without invoking its HDL converter."""
    usb = LunaCDCACM(None, SimpleNamespace(d_p=Signal(), d_n=Signal(), pullup=Signal()))
    # These tests intentionally borrow the wrapper's wiring/CDC without
    # elaborating its unrelated physical USB device. Silence only that
    # instance's unused-elaboratable diagnostic, not LUNA simulation warnings.
    usb.usb._MustUse__silence = True
    return usb


class USBProtocolResetBindingTests(unittest.TestCase):
    @staticmethod
    def converter(usb):
        converter = Amaranth2VConverter(None, name="usb_reset_regression", module=usb.usb,
                                       ports=usb.core_params, domains=list(usb.cd_list))
        converter.connect_wrapper()
        return converter

    def test_actual_converter_preserves_usb_reset_as_a_driven_input(self):
        usb = luna_wrapper()
        reset = usb.core_params["i_sync_rst"]
        self.assertIs(usb.core_params["i_usb_rst"], reset)
        converter = self.converter(usb)
        usb_reset = converter.m._domains["usb"].rst
        matches = [(direction, source) for direction, target, source in converter.conn_list
                   if target is usb_reset]
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0][0], "i")
        self.assertIs(matches[0][1], reset)
        generated = converter.generate_verilog()
        top = generated.split("endmodule", 1)[0]
        self.assertRegex(top, r"input\s+usb_rst\s*;")
        self.assertIn(".usb_rst(usb_rst)", top)
        self.assertNotRegex(generated, r"assign\s+usb_rst\s*=\s*1'h0\s*;")


class USBConnectDelayTests(unittest.TestCase):
    def test_ready_glitches_loss_and_domain_reset_restart_full_delay(self):
        delay_cycles = 5
        dut = LiteXModule()
        dut.cd_usb_12 = ClockDomain("usb_12")
        dut.ready = Signal()
        dut.gate = USBConnectDelay(dut.ready, delay_cycles=delay_cycles)

        def wait_connected():
            for elapsed in range(1, delay_cycles + 9):
                yield
                if (yield dut.gate.connect):
                    self.assertGreaterEqual(elapsed, delay_cycles)
                    return
            raise AssertionError("USB never connected after sustained readiness")

        def bench():
            for _ in range(8):
                yield
                self.assertEqual((yield dut.gate.connect), 0)
            # A readiness pulse shorter than the interval cannot attach.
            yield dut.ready.eq(1)
            for _ in range(delay_cycles - 1):
                yield
                self.assertEqual((yield dut.gate.connect), 0)
            yield dut.ready.eq(0)
            for _ in range(8):
                yield
                self.assertEqual((yield dut.gate.connect), 0)

            yield dut.ready.eq(1)
            yield from wait_connected()
            for _ in range(12):
                yield
                self.assertEqual((yield dut.gate.connect), 1)
            yield dut.ready.eq(0)
            for _ in range(5):
                yield
            self.assertEqual((yield dut.gate.connect), 0)
            yield dut.ready.eq(1)
            yield from wait_connected()

            # PLL/domain reset disconnects even while ready remains high,
            # and releasing reset requires the full interval again.
            yield dut.cd_usb_12.rst.eq(1)
            for _ in range(3):
                yield
                self.assertEqual((yield dut.gate.connect), 0)
            yield dut.cd_usb_12.rst.eq(0)
            yield from wait_connected()

        run_simulation(dut, {"usb_12": bench()}, clocks={"usb_12": 10})


class UARTBoneUSBFixture(LiteXModule):
    """Reuse the real wrapper's CDCs and connections, without its converter."""

    def __init__(self):
        usb = luna_wrapper()

        self.tx_cdc = usb.tx_cdc
        self.rx_cdc = usb.rx_cdc
        self.comb += usb._fragment.comb
        self.core_params = usb.core_params
        self.phy = SimpleNamespace(source=usb.source, sink=usb.sink)
        self.uart = UARTBone(self.phy, clk_freq=1_000_000)

        # Hold each read for several cycles; response bytes vary with the
        # address, so duplicated/missing bus reads cannot pass the payload check.
        bus = self.uart.wishbone
        delay = Signal(3)
        self.comb += bus.dat_r.eq(bus.adr ^ 0xA5C38000)
        self.sync += [
            bus.ack.eq(0),
            If(~bus.cyc | ~bus.stb, delay.eq(0)).Elif(~bus.ack,
                If(delay == 3, bus.ack.eq(1), delay.eq(0)).Else(delay.eq(delay + 1))),
        ]


class UARTBoneUSBBoundaryTests(unittest.TestCase):
    def test_real_uartbone_responses_keep_final_marker_across_async_cdc(self):
        dut = UARTBoneUSBFixture()
        lengths = (4, 64, 512)
        base_words = (0x1234, 0x2000, 0x3456)
        expected = [b"".join(((base + index) ^ 0xA5C38000).to_bytes(4, "big")
                             for index in range(length // 4))
                    for base, length in zip(base_words, lengths)]
        captured = []
        source_captured = []
        backpressure = []
        usb_valid = dut.core_params["i_tx_valid"]
        usb_ready = dut.core_params["o_tx_ready"]
        usb_data = dut.core_params["i_tx_payload"]
        usb_first = dut.core_params["i_tx_first"]
        usb_last = dut.core_params["i_tx_last"]

        def send_commands():
            incoming = dut.rx_cdc.sink
            target = 0
            for length, address in zip(lengths, base_words):
                command = bytes([2, length // 4]) + address.to_bytes(4, "big")
                for value in command:
                    yield incoming.data.eq(value)
                    yield incoming.valid.eq(1)
                    yield
                    for _ in range(1000):
                        if (yield incoming.ready):
                            break
                        yield
                    else:
                        raise AssertionError("UARTBone command stalled in RX CDC")
                    yield incoming.valid.eq(0)
                    yield
                target += length
                for _ in range(20000):
                    if len(captured) >= target:
                        break
                    yield
                else:
                    raise AssertionError(f"UARTBone response stalled at {len(captured)} of {target} bytes")
                for _ in range(10):
                    yield
            for _ in range(40):
                yield

        @passive
        def consume_usb():
            cycle = 0
            held = None
            while True:
                valid, ready = (yield usb_valid), (yield usb_ready)
                item = ((yield usb_data), (yield usb_first), (yield usb_last))
                if held is not None:
                    self.assertTrue(valid, "CDC dropped valid while USB stalled")
                    self.assertEqual(item, held, "CDC changed payload/markers while USB stalled")
                if valid and ready:
                    captured.append(item)
                held = item if valid and not ready else None
                # Fill the FIFO at startup, then repeatedly interrupt drains.
                yield usb_ready.eq(cycle >= 100 and cycle % 11 not in (2, 3, 4, 5))
                cycle += 1
                yield

        @passive
        def monitor_uart_source():
            source = dut.uart.source
            while True:
                if (yield source.valid):
                    if (yield source.ready):
                        source_captured.append(((yield source.data), (yield source.first), (yield source.last)))
                    else:
                        backpressure.append(1)
                yield

        fragment = dut.get_fragment()
        # The pinned Migen simulator assumes every RAM port has read data.
        # Preserve the actual FIFO storage/control and provide an unused
        # simulation-only read result for its legitimate write-only ports.
        for special in fragment.specials:
            if isinstance(special, Memory):
                for port in special.ports:
                    if port.dat_r is None:
                        port.dat_r = Signal(special.width)
        run_simulation(fragment, {"sys": monitor_uart_source(),
                             "usb_12": [send_commands(), consume_usb()]},
                       clocks={"sys": 10, "usb_12": 17})
        self.assertTrue(backpressure, "test did not exercise UARTBone backpressure")
        self.assertEqual(source_captured, captured)
        self.assertEqual(bytes(value for value, _, _ in captured), b"".join(expected))
        self.assertEqual([index for index, (_, _, last) in enumerate(captured) if last],
                         [3, 67, 579])
        self.assertTrue(all(first == 0 for _, first, _ in captured),
                        "UARTBone should not invent per-byte first markers")


def simulate_luna_in_transfer(payload, *, last_on_every_byte=False):
    """Return USB packets, including ZLPs, emitted by a stalled 64-byte endpoint.

    The host issues IN tokens and ACKs every completed packet. Source gaps and
    output backpressure exercise real ready/valid handling. After all payload
    arrives, twelve further NAKs ensure a terminal ZLP is emitted only once.
    """
    dut = USBInTransferManager(max_packet_size=64)
    sim = AmaranthSimulator(dut)
    sim.add_clock(1 / 60e6, domain="usb")
    packets = []
    pids = []

    async def bench(ctx):
        ctx.set(dut.active, 1)
        ctx.set(dut.generate_zlps, 1)
        ctx.set(dut.tokenizer.is_in, 1)
        sent = 0
        received = 0
        host_state = "token"
        host_gap = 0
        current_packet = None
        packet_pid = None
        drained_naks = 0
        stalled = None
        producer_holding = False

        for cycle in range(40000):
            # A producer must retain its byte and last flag while backpressured.
            source_valid = sent < len(payload) and (producer_holding or cycle % 7 != 3)
            ctx.set(dut.transfer_stream.valid, source_valid)
            ctx.set(dut.transfer_stream.payload, payload[sent] if sent < len(payload) else 0)
            ctx.set(dut.transfer_stream.last,
                    source_valid and (last_on_every_byte or sent == len(payload) - 1))
            ready = cycle % 5 != 2
            ctx.set(dut.packet_stream.ready, ready)
            token = host_state == "token"
            ctx.set(dut.tokenizer.ready_for_response, token)
            ctx.set(dut.handshakes_in.ack, host_state == "ack")

            (_, _, in_ready, valid, first, last, value, nak, pid) = await ctx.tick("usb").sample(
                dut.transfer_stream.ready,
                dut.packet_stream.valid,
                dut.packet_stream.first,
                dut.packet_stream.last,
                dut.packet_stream.payload,
                dut.handshakes_out.nak,
                dut.data_pid,
            )
            if source_valid and in_ready:
                sent += 1
            producer_holding = source_valid and not in_ready

            # ZLPs use valid=last=1, first=0 on the IN-token cycle; no
            # payload byte is consumed. This differs from a one-byte packet,
            # whose first and last markers are both asserted.
            zlp = token and valid and last and not first
            if stalled is not None:
                assert valid, "packet valid dropped while stalled"
                assert (first, last, value, pid) == stalled, "packet changed while stalled"
            stalled = (first, last, value, pid) if valid and not ready and not zlp else None

            if zlp:
                assert current_packet is None
                packets.append(b"")
                pids.append(pid)
                drained_naks = 0
                host_state = "ack"
            elif valid and ready:
                if first:
                    assert current_packet is None, "unexpected start inside packet"
                    current_packet = bytearray()
                    packet_pid = pid
                assert current_packet is not None, "packet missing first marker"
                assert pid == packet_pid, "data PID changed inside packet"
                current_packet.append(value)
                received += 1
                assert len(current_packet) <= 64, "oversized USB packet"
                drained_naks = 0
                if last:
                    packets.append(bytes(current_packet))
                    pids.append(pid)
                    current_packet = None
                    host_state = "ack"
            elif token:
                if nak:
                    if sent == len(payload) and received == len(payload):
                        drained_naks += 1
                    if drained_naks == 12:
                        break
                    host_state = "gap"
                    host_gap = 3
                else:
                    host_state = "receive"
            elif host_state == "ack":
                host_state = "gap"
                host_gap = 3
            elif host_state == "gap":
                host_gap -= 1
                if not host_gap:
                    host_state = "token"
        else:
            raise AssertionError(
                f"USB transfer stalled: sent={sent}, received={received}, "
                f"packets={[len(packet) for packet in packets]}, state={host_state}"
            )

        assert current_packet is None
        assert sent == len(payload)
        assert b"".join(packets) == payload, "USB payload was dropped or corrupted"
        assert pids == [i & 1 for i in range(len(pids))], "ACKed packet PIDs did not alternate"

    sim.add_testbench(bench)
    sim.run()
    return packets


class LUNAInTransferPacketizationTests(unittest.TestCase):
    @staticmethod
    def payload(length):
        return bytes((i * 29 + 7) & 0xff for i in range(length))

    def test_full_packets_have_exactly_one_terminal_zlp(self):
        for length in (64, 512):
            with self.subTest(length=length):
                packets = simulate_luna_in_transfer(self.payload(length))
                self.assertEqual([len(packet) for packet in packets], [64] * (length // 64) + [0])
                self.assertEqual(packets.count(b""), 1)

    def test_short_packet_terminates_without_zlp(self):
        for length in (4, 65):
            with self.subTest(length=length):
                packets = simulate_luna_in_transfer(self.payload(length))
                self.assertEqual([len(packet) for packet in packets], [64] * (length // 64) + [length % 64])
                self.assertNotIn(b"", packets)

    def test_last_tied_to_valid_fragments_every_byte(self):
        # Reproduce the old bridge wiring: last=valid makes every byte a
        # separate short transfer, defeating 64-byte endpoint packetization.
        for length in (4, 64):
            with self.subTest(length=length):
                packets = simulate_luna_in_transfer(self.payload(length), last_on_every_byte=True)
                self.assertEqual([len(packet) for packet in packets], [1] * length)
                self.assertNotIn(b"", packets)


class AddressConfigurationProbe(a.Elaboratable):
    def __init__(self):
        self.interface = EndpointInterface()
        self.set_address = a.Signal()
        self.address = a.Signal(7)
        self.set_config = a.Signal()
        self.configuration = a.Signal(8)

    def elaborate(self, platform):
        m = a.Module()
        m.d.comb += [
            self.interface.address_changed.eq(self.set_address),
            self.interface.new_address.eq(self.address),
            self.interface.config_changed.eq(self.set_config),
            self.interface.new_config.eq(self.configuration),
        ]
        return m


class LUNAResetFixture(a.Elaboratable):
    def __init__(self):
        self.usb_reset = a.Signal()
        self.utmi = UTMIInterface()
        self.device = USBDevice(bus=self.utmi)
        self.endpoint = AddressConfigurationProbe()
        self.device.add_endpoint(self.endpoint)
        self.transfer = USBInTransferManager(max_packet_size=64)

    def elaborate(self, platform):
        m = a.Module()
        m.domains.usb = a.ClockDomain("usb")
        m.d.comb += a.ResetSignal("usb").eq(self.usb_reset)
        m.submodules.device = self.device
        m.submodules.transfer = self.transfer
        return m


class LUNAUSBDomainResetTests(unittest.TestCase):
    def run_probe(self, bench):
        dut = LUNAResetFixture()
        sim = AmaranthSimulator(dut)
        sim.add_clock(1 / 12e6, domain="usb")

        async def wrapped(ctx):
            # Keep the UTMI bus attached and at full-speed J throughout. The
            # reset below is the FPGA domain reset, not a host SE0 bus reset.
            ctx.set(dut.device.connect, 1)
            ctx.set(dut.utmi.session_end, 0)
            ctx.set(dut.utmi.line_state, 1)
            ctx.set(dut.utmi.tx_ready, 1)
            ctx.set(dut.transfer.active, 1)
            ctx.set(dut.transfer.generate_zlps, 1)
            ctx.set(dut.transfer.tokenizer.is_in, 1)
            ctx.set(dut.transfer.packet_stream.ready, 1)
            for _ in range(4):
                await ctx.tick("usb")
            await bench(ctx, dut)

        sim.add_testbench(wrapped)
        sim.run()

    async def set_identity(self, ctx, dut, address, configuration):
        probe = dut.endpoint
        ctx.set(probe.address, address)
        ctx.set(probe.configuration, configuration)
        ctx.set(probe.set_address, 1)
        ctx.set(probe.set_config, 1)
        await ctx.tick("usb")
        ctx.set(probe.set_address, 0)
        ctx.set(probe.set_config, 0)
        await ctx.tick("usb")
        self.assertEqual(ctx.get(probe.interface.active_address), address)
        self.assertEqual(ctx.get(probe.interface.active_config), configuration)
        self.assertEqual(ctx.get(dut.device.reset_detected), 0,
                         "host bus reset would invalidate the domain-reset probe")

    async def domain_reset(self, ctx, dut):
        ctx.set(dut.usb_reset, 1)
        for _ in range(3):
            await ctx.tick("usb")
        self.assertEqual(ctx.get(dut.endpoint.interface.active_address), 0)
        self.assertEqual(ctx.get(dut.endpoint.interface.active_config), 0)
        ctx.set(dut.usb_reset, 0)
        for _ in range(4):
            await ctx.tick("usb")
        self.assertEqual(ctx.get(dut.device.reset_detected), 0)

    async def queue_packet(self, ctx, dut, payload):
        transfer = dut.transfer
        self.assertFalse(ctx.get(transfer.tokenizer.ready_for_response))
        for index, byte in enumerate(payload):
            ctx.set(transfer.transfer_stream.valid, 1)
            ctx.set(transfer.transfer_stream.payload, byte)
            ctx.set(transfer.transfer_stream.last, index == len(payload) - 1)
            for _ in range(100):
                _, _, ready = await ctx.tick("usb").sample(transfer.transfer_stream.ready)
                if ready:
                    break
            else:
                self.fail("real LUNA transfer manager did not accept the queued byte")
        ctx.set(transfer.transfer_stream.valid, 0)
        ctx.set(transfer.transfer_stream.last, 0)
        for _ in range(3):
            await ctx.tick("usb")
        self.assertEqual(ctx.get(transfer.packet_stream.valid), 0,
                         "packet must remain queued until an IN token arrives")

    async def request_packet(self, ctx, dut):
        transfer = dut.transfer
        ctx.set(transfer.tokenizer.ready_for_response, 1)
        _, _, nak, valid = await ctx.tick("usb").sample(
            transfer.handshakes_out.nak, transfer.packet_stream.valid)
        ctx.set(transfer.tokenizer.ready_for_response, 0)
        if nak:
            self.assertFalse(valid, "NAK must not accompany stale packet data")
            for _ in range(3):
                _, _, valid = await ctx.tick("usb").sample(transfer.packet_stream.valid)
                self.assertFalse(valid, "NAK was followed by unsolicited stale packet data")
            return None
        self.assertFalse(valid, "this probe expects nonempty packets, not ZLPs")
        result = bytearray()
        for _ in range(100):
            _, _, valid, first, last, payload = await ctx.tick("usb").sample(
                transfer.packet_stream.valid, transfer.packet_stream.first,
                transfer.packet_stream.last, transfer.packet_stream.payload)
            if valid:
                self.assertEqual(bool(first), not result)
                result.append(payload)
                if last:
                    ctx.set(transfer.handshakes_in.ack, 1)
                    await ctx.tick("usb")
                    ctx.set(transfer.handshakes_in.ack, 0)
                    for _ in range(3):
                        await ctx.tick("usb")
                    return bytes(result)
        self.fail("IN token produced neither NAK nor a complete packet")

    def test_real_device_address_and_configuration_clear_on_domain_reset(self):
        async def bench(ctx, dut):
            await self.set_identity(ctx, dut, address=37, configuration=1)
            for _ in range(8):
                await ctx.tick("usb")
                self.assertEqual(ctx.get(dut.endpoint.interface.active_address), 37)
            await self.domain_reset(ctx, dut)
            self.assertEqual(ctx.get(dut.endpoint.interface.active_address), 0)
            self.assertEqual(ctx.get(dut.endpoint.interface.active_config), 0)
            # Prove reset did not just disable the device's update interface.
            await self.set_identity(ctx, dut, address=91, configuration=2)
        self.run_probe(bench)

    def test_shared_reset_discards_queued_packet_and_accepts_fresh_data(self):
        async def bench(ctx, dut):
            await self.set_identity(ctx, dut, address=53, configuration=1)
            await self.queue_packet(ctx, dut, b"OLD!")
            self.assertEqual(ctx.get(dut.transfer.buffer_toggle), 1,
                             "last-marked input must have committed the first packet")
            for _ in range(8):
                await ctx.tick("usb")
            # No IN token has been issued: OLD! is committed but unconsumed.
            await self.domain_reset(ctx, dut)
            self.assertEqual(ctx.get(dut.transfer.buffer_toggle), 0)
            for _ in range(4):
                self.assertIsNone(await self.request_packet(ctx, dut),
                                  "reset leaked a pre-reset packet")
            await self.set_identity(ctx, dut, address=23, configuration=1)
            await self.queue_packet(ctx, dut, b"ok")
            self.assertEqual(await self.request_packet(ctx, dut), b"ok")
            self.assertIsNone(await self.request_packet(ctx, dut))
            # A second transfer exercises the opposite buffer after reset.
            await self.queue_packet(ctx, dut, b"fresh")
            self.assertEqual(await self.request_packet(ctx, dut), b"fresh")
            self.assertIsNone(await self.request_packet(ctx, dut))
        self.run_probe(bench)


def raw_usb_record():
    return a.Record([
        ("d_p", [("i", 1), ("o", 1), ("oe", 1)]),
        ("d_n", [("i", 1), ("o", 1), ("oe", 1)]),
        ("pullup", [("o", 1)]),
    ])


class USBRawPHYBindingTests(unittest.TestCase):
    def test_converter_preserves_native_physical_ports_streams_and_reset(self):
        usb = luna_wrapper()


        self.assertIs(type(usb.usb), USBSerialDevice)
        physical = usb.usb._bus
        parameters = dict(usb.core_params)
        converter = USBProtocolResetBindingTests.converter(usb)
        for name, direction, target in (
                ("i__bus_d_p_i", "i", physical.d_p.i),
                ("o__bus_d_p_oe", "o", physical.d_p.oe),
                ("o__bus_d_n_oe", "o", physical.d_n.oe),
                ("o__bus_pullup_o", "o", physical.pullup.o)):
            matches = [(actual_direction, actual_target)
                       for actual_direction, actual_target, source in converter.conn_list
                       if source is parameters[name]]
            self.assertEqual(len(matches), 1)
            self.assertEqual(matches[0][0], direction)
            self.assertIs(matches[0][1], target)
        generated = converter.generate_verilog()
        top = generated.split("endmodule", 1)[0]
        self.assertRegex(top, r"input\s+usb_rst\s*;")
        self.assertNotRegex(generated, r"assign\s+usb_rst\s*=\s*1'h0\s*;")


class RawUSBFixture(a.Elaboratable):
    """Actual PHY/device with resolved pins and an independent host driver."""

    def __init__(self, *, with_device=True, standard_control=False):
        self.io = raw_usb_record()
        self.connect = a.Signal()
        self.usb_reset = a.Signal()
        self.host_oe = a.Signal()
        self.host_dp = a.Signal()
        self.host_dn = a.Signal()
        self.device = None
        if with_device:
            self.device = USBDevice(bus=self.io)
            if standard_control:
                descriptors = DeviceDescriptorCollection()
                self.descriptors = descriptors
                with descriptors.DeviceDescriptor() as descriptor:
                    descriptor.idVendor = 0x1209
                    descriptor.idProduct = 1
                    descriptor.bNumConfigurations = 1
                self.endpoint = self.device.add_standard_control_endpoint(descriptors)
            else:
                self.endpoint = AddressConfigurationProbe()
                self.device.add_endpoint(self.endpoint)
        self.phy = self.device.utmi if with_device else GatewarePHY(io=self.io)

    def elaborate(self, platform):
        m = a.Module()
        m.domains.usb = a.ClockDomain("usb")
        m.domains.usb_io = a.ClockDomain("usb_io")
        m.d.comb += [a.ResetSignal("usb").eq(self.usb_reset),
                     a.ResetSignal("usb_io").eq(self.usb_reset)]
        if self.device is not None:
            m.submodules.device = self.device
            m.d.comb += self.device.connect.eq(self.connect)
        else:
            m.submodules.phy = self.phy
            m.d.comb += self.phy.connect.eq(self.connect)
        # With neither side driving, the device pull-up and host pulldowns
        # determine the line state. Host reset deliberately overrides this.
        m.d.comb += [
            self.io.d_p.i.eq(a.Mux(self.host_oe, self.host_dp,
                                  a.Mux(self.io.d_p.oe, self.io.d_p.o, self.io.pullup.o))),
            self.io.d_n.i.eq(a.Mux(self.host_oe, self.host_dn,
                                  a.Mux(self.io.d_n.oe, self.io.d_n.o, 0))),
        ]
        return m


class RawUSBPHYTests(unittest.TestCase):
    def simulate(self, dut, bench):
        sim = AmaranthSimulator(dut)
        sim.add_clock(1 / 12e6, phase=0, domain="usb")
        sim.add_clock(1 / 48e6, phase=0, domain="usb_io")

        async def wrapped(ctx):
            await bench(ctx, dut)

        sim.add_testbench(wrapped)
        sim.run()

    def assert_detached(self, ctx, dut):
        self.assertEqual(ctx.get(dut.io.pullup.o), 0)
        self.assertEqual(ctx.get(dut.io.d_p.oe), 0)
        self.assertEqual(ctx.get(dut.io.d_n.oe), 0)

    def test_utmi_line_states_and_non_driving_mode_match_physical_pins(self):
        async def bench(ctx, dut):
            utmi = dut.phy
            ctx.set(dut.host_oe, 1)
            for dp, dn, expected in ((0, 0, 0), (1, 0, 1), (0, 1, 2), (1, 1, 3)):
                ctx.set(dut.host_dp, dp)
                ctx.set(dut.host_dn, dn)
                await ctx.tick("usb")
                self.assertEqual(ctx.get(utmi.line_state), expected)

            ctx.set(dut.host_oe, 0)
            ctx.set(dut.connect, 1)
            ctx.set(utmi.term_select, 1)
            ctx.set(utmi.xcvr_select, 1)
            ctx.set(utmi.tx_valid, 1)
            ctx.set(utmi.tx_data, 1)
            ctx.set(utmi.op_mode, UTMIOperatingMode.NON_DRIVING)
            for _ in range(8):
                await ctx.tick("usb_io")
                self.assertEqual(ctx.get(dut.io.pullup.o), 1)
                self.assertEqual(ctx.get(dut.io.d_p.oe), 0)
                self.assertEqual(ctx.get(dut.io.d_n.oe), 0)

            ctx.set(utmi.op_mode, UTMIOperatingMode.RAW_DRIVE)
            for bit in (0, 1):
                ctx.set(utmi.tx_data, bit)
                await ctx.tick("usb_io")
                self.assertEqual(ctx.get(dut.io.d_p.oe), 1)
                self.assertEqual(ctx.get(dut.io.d_n.oe), 1)
                self.assertEqual(ctx.get(dut.io.d_p.o), bit)
                self.assertEqual(ctx.get(dut.io.d_n.o), bit ^ 1)

            # Disconnect and domain reset must clamp physical pins even
            # while upstream explicitly requests raw drive and termination.
            ctx.set(dut.connect, 0)
            await ctx.tick("usb_io")
            self.assert_detached(ctx, dut)
            ctx.set(dut.connect, 1)
            ctx.set(dut.usb_reset, 1)
            await ctx.tick("usb_io")
            self.assert_detached(ctx, dut)

        self.simulate(RawUSBFixture(with_device=False), bench)

    def test_pending_ack_cannot_drive_disconnected_pins(self):
        async def bench(ctx, dut):
            ctx.set(dut.connect, 1)
            await ctx.tick("usb").repeat(5)
            ctx.set(dut.endpoint.interface.handshakes_out.ack, 1)
            await ctx.tick("usb")
            ctx.set(dut.endpoint.interface.handshakes_out.ack, 0)
            ctx.set(dut.connect, 0)
            for _ in range(500 * 4):
                await ctx.tick("usb_io")
                self.assert_detached(ctx, dut)

        self.simulate(RawUSBFixture(), bench)

    def test_reconnect_host_reset_clears_identity_and_allows_new_response(self):
        async def bench(ctx, dut):
            ctx.set(dut.connect, 1)
            for _ in range(5):
                await ctx.tick("usb")
            probe = dut.endpoint
            ctx.set(probe.address, 37)
            ctx.set(probe.configuration, 1)
            ctx.set(probe.set_address, 1)
            ctx.set(probe.set_config, 1)
            await ctx.tick("usb")
            ctx.set(probe.set_address, 0)
            ctx.set(probe.set_config, 0)
            self.assertEqual(ctx.get(probe.interface.active_address), 37)
            self.assertEqual(ctx.get(probe.interface.active_config), 1)

            ctx.set(dut.connect, 0)
            for _ in range(20):
                await ctx.tick("usb")
                self.assert_detached(ctx, dut)
            ctx.set(dut.connect, 1)
            for _ in range(200):
                await ctx.tick("usb")
            self.assertEqual(ctx.get(dut.io.pullup.o), 1)
            self.assertEqual(ctx.get(dut.phy.line_state), 1)

            # The physical host holds both lines low. Allow the pinned
            # sequencer's 300-cycle threshold (25 us at its actual 12 MHz).
            ctx.set(dut.host_oe, 1)
            ctx.set(dut.host_dp, 0)
            ctx.set(dut.host_dn, 0)
            resets = 0
            for _ in range(400):
                await ctx.tick("usb")
                resets += ctx.get(dut.device.reset_detected)
            self.assertGreater(resets, 0)
            self.assertEqual(ctx.get(probe.interface.active_address), 0)
            self.assertEqual(ctx.get(probe.interface.active_config), 0)
            ctx.set(dut.host_oe, 0)
            for _ in range(12):
                await ctx.tick("usb")

            # A real handshake response must leave the PHY, receive its
            # PHY acknowledgement, and release the pins after EOP.
            ctx.set(probe.interface.handshakes_out.ack, 1)
            await ctx.tick("usb")
            ctx.set(probe.interface.handshakes_out.ack, 0)
            driven = []
            for _ in range(140):
                await ctx.tick("usb_io")
                if ctx.get(dut.io.d_p.oe):
                    driven.append((ctx.get(dut.io.d_p.o), ctx.get(dut.io.d_n.o)))
            self.assertIn((1, 0), driven)
            self.assertIn((0, 1), driven)
            self.assertIn((0, 0), driven)  # EOP SE0, not a permanent raw K.
            self.assertEqual(ctx.get(dut.device.utmi.tx_valid), 0)
            self.assertEqual(ctx.get(dut.io.d_p.oe), 0)
            self.assertEqual(ctx.get(dut.io.d_n.oe), 0)

        self.simulate(RawUSBFixture(), bench)

    def test_physical_setup_status_ack_and_host_reset_after_reconnect(self):
        async def bench(ctx, dut):
            async def send(packet):
                ctx.set(dut.host_oe, 1)
                for symbol in usb_packet.wrap_packet(packet, cycles=4):
                    dp, dn = {"J": (1, 0), "K": (0, 1), "_": (0, 0)}[symbol]
                    ctx.set(dut.host_dp, dp)
                    ctx.set(dut.host_dn, dn)
                    await ctx.tick("usb_io")
                ctx.set(dut.host_oe, 0)

            async def receive():
                waveform = ""
                for _ in range(3000):
                    _, _, oe, dp, dn = await ctx.tick("usb_io").sample(
                        dut.io.d_p.oe, dut.io.d_p.o, dut.io.d_n.o)
                    if oe:
                        waveform += {(1, 0): "J", (0, 1): "K", (0, 0): "_",
                                     (1, 1): "X"}[(dp, dn)]
                    elif waveform:
                        return waveform
                self.fail("host packet produced no complete physical USB response")

            async def host_reset():
                ctx.set(dut.host_oe, 1)
                ctx.set(dut.host_dp, 0)
                ctx.set(dut.host_dn, 0)
                detected = False
                for _ in range(1600):
                    _, _, reset = await ctx.tick("usb_io").sample(dut.device.reset_detected)
                    detected |= bool(reset)
                self.assertTrue(detected)
                ctx.set(dut.host_oe, 0)
                await ctx.tick("usb_io").repeat(480)

            async def set_address(address):
                # Exercise the entire physical RX/CRC/EP0/TX path. Comparing
                # all 48 MHz pin samples also verifies SYNC, NRZI and EOP.
                await send(usb_packet.token_packet(USBPacketID.SETUP, 0, 0))
                await ctx.tick("usb_io").repeat(8)
                await send(usb_packet.data_packet(
                    USBPacketID.DATA0, [0, 5, address, 0, 0, 0, 0, 0]))
                self.assertEqual(await receive(), usb_packet.wrap_packet(
                    usb_packet.handshake_packet(USBPacketID.ACK), cycles=4))
                self.assertEqual(ctx.get(dut.endpoint.interface.active_address), 0)
                await ctx.tick("usb_io").repeat(8)
                await send(usb_packet.token_packet(USBPacketID.IN, 0, 0))
                self.assertEqual(await receive(), usb_packet.wrap_packet(
                    usb_packet.data_packet(USBPacketID.DATA1, []), cycles=4))
                await ctx.tick("usb_io").repeat(8)
                await send(usb_packet.handshake_packet(USBPacketID.ACK))
                await ctx.tick("usb_io").repeat(80)
                self.assertEqual(ctx.get(dut.endpoint.interface.active_address), address)

            ctx.set(dut.connect, 1)
            await ctx.tick("usb_io").repeat(80)
            await host_reset()
            await set_address(7)
            ctx.set(dut.connect, 0)
            for _ in range(1200):
                await ctx.tick("usb_io")
                self.assert_detached(ctx, dut)
            ctx.set(dut.connect, 1)
            await ctx.tick("usb_io").repeat(800)
            await host_reset()
            self.assertEqual(ctx.get(dut.endpoint.interface.active_address), 0)
            await set_address(11)

        self.simulate(RawUSBFixture(standard_control=True), bench)


class RawUSBSerialFixture(RawUSBFixture):
    """The production ACM endpoint types/order, using real physical packets."""

    def __init__(self):
        super().__init__(standard_control=True)
        self.status = USBStreamInEndpoint(endpoint_number=3, max_packet_size=64)
        self.data_out = USBStreamOutEndpoint(endpoint_number=4, max_packet_size=64)
        self.data_in = USBStreamInEndpoint(endpoint_number=4, max_packet_size=64)
        for endpoint in (self.status, self.data_out, self.data_in):
            self.device.add_endpoint(endpoint)


class PhysicalUSBHost:
    """Pin-level host, including CRC, NRZI, EOP and complete control status."""

    def __init__(self, case, ctx, dut):
        self.case, self.ctx, self.dut = case, ctx, dut

    async def gap(self):
        await self.ctx.tick("usb_io").repeat(8)

    async def send(self, packet):
        ctx, dut = self.ctx, self.dut
        ctx.set(dut.host_oe, 1)
        for symbol in usb_packet.wrap_packet(packet, cycles=4):
            dp, dn = {"J": (1, 0), "K": (0, 1), "_": (0, 0)}[symbol]
            ctx.set(dut.host_dp, dp)
            ctx.set(dut.host_dn, dn)
            await ctx.tick("usb_io")
        ctx.set(dut.host_oe, 0)

    async def receive(self):
        waveform = ""
        for _ in range(3000):
            _, _, oe, dp, dn = await self.ctx.tick("usb_io").sample(
                self.dut.io.d_p.oe, self.dut.io.d_p.o, self.dut.io.d_n.o)
            if oe:
                waveform += {(1, 0): "J", (0, 1): "K", (0, 0): "_",
                             (1, 1): "X"}[(dp, dn)]
            elif waveform:
                return waveform
        return ""

    async def expect(self, packet):
        self.case.assertEqual(await self.receive(), usb_packet.wrap_packet(packet, cycles=4))
        await self.gap()

    async def host_reset(self, *, cycles=1600):
        ctx, dut = self.ctx, self.dut
        ctx.set(dut.host_oe, 1)
        ctx.set(dut.host_dp, 0)
        ctx.set(dut.host_dn, 0)
        detected = False
        for _ in range(cycles):
            _, _, reset = await ctx.tick("usb_io").sample(dut.device.reset_detected)
            detected |= bool(reset)
        self.case.assertTrue(detected)
        ctx.set(dut.host_oe, 0)
        await ctx.tick("usb_io").repeat(480)
        self.case.assertEqual(ctx.get(dut.endpoint.interface.active_address), 0)
        self.case.assertEqual(ctx.get(dut.endpoint.interface.active_config), 0)

    async def setup(self, payload):
        await self.send(usb_packet.token_packet(USBPacketID.SETUP, 0, 0))
        await self.gap()
        await self.send(usb_packet.data_packet(USBPacketID.DATA0, payload))
        await self.expect(usb_packet.handshake_packet(USBPacketID.ACK))

    async def get_descriptor(self):
        await self.setup([0x80, 6, 0, 1, 0, 0, 18, 0])
        await self.send(usb_packet.token_packet(USBPacketID.IN, 0, 0))
        response = await self.receive()
        if not response:
            return False
        expected = usb_packet.wrap_packet(usb_packet.data_packet(
            USBPacketID.DATA1, self.dut.descriptors.get_descriptor_bytes(1)), cycles=4)
        self.case.assertEqual(response, expected)
        await self.gap()
        await self.send(usb_packet.handshake_packet(USBPacketID.ACK))
        await self.gap()
        await self.send(usb_packet.token_packet(USBPacketID.OUT, 0, 0))
        await self.gap()
        await self.send(usb_packet.data_packet(USBPacketID.DATA1, []))
        await self.expect(usb_packet.handshake_packet(USBPacketID.ACK))
        await self.ctx.tick("usb_io").repeat(80)
        return True

    async def configure(self):
        await self.setup([0, 9, 1, 0, 0, 0, 0, 0])
        await self.send(usb_packet.token_packet(USBPacketID.IN, 0, 0))
        await self.expect(usb_packet.data_packet(USBPacketID.DATA1, []))
        await self.send(usb_packet.handshake_packet(USBPacketID.ACK))
        await self.ctx.tick("usb_io").repeat(80)
        self.case.assertEqual(self.ctx.get(self.dut.endpoint.interface.active_config), 1)


class USBEndpointBusResetTests(unittest.TestCase):
    simulate = RawUSBPHYTests.simulate

    def test_abandoned_set_address_cannot_poison_repeated_enumeration_resets(self):
        async def bench(ctx, dut):
            host = PhysicalUSBHost(self, ctx, dut)
            ctx.set(dut.connect, 1)
            await ctx.tick("usb_io").repeat(80)
            self.assertTrue(await host.get_descriptor())
            # Accept SET_ADDRESS, then abandon its status IN. The next bus
            # reset must clear the handler as well as address/configuration.
            await host.setup([0, 5, 42, 0, 0, 0, 0, 0])
            for attempt in range(4):
                await host.host_reset()
                self.assertTrue(await host.get_descriptor())
                if attempt < 2:
                    # Recreate the fault after successful recovery.
                    await host.setup([0, 5, 43 + attempt, 0, 0, 0, 0, 0])

        self.simulate(RawUSBSerialFixture(), bench)

    def test_bus_reset_discards_buffered_bulk_packet_and_restarts_data_pid(self):
        async def bench(ctx, dut):
            host = PhysicalUSBHost(self, ctx, dut)
            ctx.set(dut.connect, 1)
            await ctx.tick("usb_io").repeat(80)
            await host.configure()

            async def queue(payload):
                stream = dut.data_in.stream
                for index, byte in enumerate(payload):
                    ctx.set(stream.valid, 1)
                    ctx.set(stream.payload, byte)
                    ctx.set(stream.last, index == len(payload) - 1)
                    for _ in range(100):
                        _, _, ready = await ctx.tick("usb").sample(stream.ready)
                        if ready:
                            break
                    else:
                        self.fail("bulk endpoint did not accept queued data")
                ctx.set(stream.valid, 0)
                ctx.set(stream.last, 0)
                await ctx.tick("usb_io").repeat(40)

            async def consume(payload, pid):
                await host.send(usb_packet.token_packet(USBPacketID.IN, 0, 4))
                await host.expect(usb_packet.data_packet(pid, payload))
                await host.send(usb_packet.handshake_packet(USBPacketID.ACK))
                await ctx.tick("usb_io").repeat(40)

            await queue(b"first")
            await consume(b"first", USBPacketID.DATA0)
            await queue(b"stale")
            # This packet is committed but has never been requested by IN.
            await host.host_reset()
            await host.configure()
            await host.send(usb_packet.token_packet(USBPacketID.IN, 0, 4))
            await host.expect(usb_packet.handshake_packet(USBPacketID.NAK))
            await queue(b"fresh")
            await consume(b"fresh", USBPacketID.DATA0)
            await queue(b"next")
            await consume(b"next", USBPacketID.DATA1)

        self.simulate(RawUSBSerialFixture(), bench)

    def test_bus_reset_during_bulk_transmit_allows_first_control_request(self):
        async def bench(ctx, dut):
            host = PhysicalUSBHost(self, ctx, dut)
            ctx.set(dut.connect, 1)
            await ctx.tick("usb_io").repeat(80)
            await host.configure()
            stream = dut.data_in.stream
            for index in range(64):
                ctx.set(stream.valid, 1)
                ctx.set(stream.payload, (index * 3 + 7) & 0xFF)
                ctx.set(stream.last, index == 63)
                for _ in range(100):
                    _, _, ready = await ctx.tick("usb").sample(stream.ready)
                    if ready:
                        break
                else:
                    self.fail("bulk endpoint did not accept the full packet")
            ctx.set(stream.valid, 0)
            ctx.set(stream.last, 0)
            await ctx.tick("usb_io").repeat(40)
            await host.send(usb_packet.token_packet(USBPacketID.IN, 0, 4))
            for _ in range(1000):
                await ctx.tick("usb_io")
                if ctx.get(dut.io.d_p.oe):
                    break
            else:
                self.fail("bulk IN packet never started transmitting")
            await ctx.tick("usb_io").repeat(40)
            self.assertEqual(ctx.get(dut.device.utmi.tx_valid), 1)
            # SE0 starts while a 64-byte packet is still transmitting. The
            # bus-reset pulse arrives before its last byte, so clearing only
            # endpoint FIFOs strands the shared packet generator mid-payload.
            await host.host_reset(cycles=4800)
            # Check the FIRST ACK and descriptor, without relying on a retry
            # to drain an old CRC or packet-generator state.
            self.assertTrue(await host.get_descriptor())

        self.simulate(RawUSBSerialFixture(), bench)


class USBDescriptorSelectionTests(unittest.TestCase):
    def test_full_speed_acm_selects_direct_descriptors_without_global_override(self):
        self.enterContext(warnings.catch_warnings())
        warnings.filterwarnings("ignore", category=UnusedElaboratable)
        self.addCleanup(gc.collect)
        environment = {key: value for key, value in os.environ.items()
                       if key != "LUNA_AVOID_BLOCKRAM"}
        with patch.dict(os.environ, environment, clear=True):
            serial = USBSerialDevice(bus=UTMIInterface(), idVendor=0x1209, idProduct=1)
            module = serial.elaborate(None)
            standard = module.submodules.usb._endpoints[0]._request_handlers[0]
            other = StandardRequestHandler(serial.create_descriptors())
            self.assertIs(type(standard.get_descriptor_handler_submodule()),
                          GetDescriptorHandlerDistributed)
            self.assertIs(type(other.get_descriptor_handler_submodule()),
                          GetDescriptorHandlerBlock)
            self.assertEqual(tuple(standard.descriptors), tuple(other.descriptors))


class USBNativeTransmitDomainTests(unittest.TestCase):
    def test_encoder_state_and_output_registers_stay_in_usb_domain(self):
        fragment = a.Fragment.get(GatewarePHY(io=raw_usb_record()), None)
        encoders = []

        def find(child):
            if any(type(origin) is TxNRZIEncoder for origin in child.origins):
                encoders.append(child)
            for nested, _, _ in child.subfragments:
                find(nested)

        find(fragment)
        self.assertEqual(len(encoders), 1)
        domains = set()

        def collect_domains(child):
            domains.update(name for name, statements in child.statements.items() if statements)
            for nested, _, _ in child.subfragments:
                collect_domains(nested)

        collect_domains(encoders[0])
        self.assertIn("usb", domains)
        self.assertNotIn("usb_io", domains)


class USBNativeTransmitSkewTests(unittest.TestCase):
    def test_native_encoder_preserves_ack_under_modeled_source_path_skew(self):
        # These fixed 2 ns/7 ns transport delays are illustrative, not measured
        # silicon timings. Exercise the actual mux, shifter and native encoder
        # under unequal source-path delay; no transmitted bits are edited.
        class Fixture(a.Elaboratable):
            def __init__(self):
                self.pipeline = TxPipeline()
                self.paths = []

            def elaborate(self, platform):
                m = a.Module()
                m.domains.usb = a.ClockDomain("usb")
                m.domains.usb_io = a.ClockDomain("usb_io")
                pipeline = self.pipeline
                body = pipeline.elaborate(platform)
                pipeline._MustUse__silence = True

                def delayed(source, delay_ns):
                    wire = a.Signal()
                    self.paths.append((source, wire, delay_ns))
                    return wire

                # Split the actual fit_dat mux into its source paths, retaining
                # the original expression and the native encoder wiring.
                assignment = next(statement for statement in body._statements["comb"]
                                  if hasattr(statement, "lhs")
                                  and statement.lhs is pipeline.fit_dat)
                payload, sync = assignment.rhs.operands
                state_and_data, not_stall = payload.operands
                state, data = state_and_data.operands
                assignment._rhs = ((delayed(state, 2) & delayed(data, 2) & not_stall)
                                   | delayed(sync, 7))
                delayed_oe = delayed(pipeline.fit_oe, 7)
                oe_assignment = next(statement for statement in body._statements["comb"]
                                     if hasattr(statement, "rhs")
                                     and statement.rhs is pipeline.fit_oe)
                oe_assignment._rhs = delayed_oe
                m.submodules.pipeline = body
                return m

        dut = Fixture()
        sim = AmaranthSimulator(dut)
        sim.add_clock(1 / 12e6, phase=14e-9, domain="usb")
        sim.add_clock(1 / 48e6, phase=0, domain="usb_io")
        for source, wire, delay_ns in dut.paths:
            async def propagate(ctx, source=source, wire=wire, delay_ns=delay_ns):
                ctx.set(wire, ctx.get(source))
                while True:
                    await ctx.changed(source)
                    value = ctx.get(source)
                    await ctx.delay(delay_ns * 1e-9)
                    ctx.set(wire, value)

            sim.add_testbench(propagate, background=True)

        waveform = []

        async def capture(ctx):
            pipeline = dut.pipeline
            while True:
                _, _, oe, dp, dn = await ctx.tick("usb_io").sample(
                    pipeline.o_oe, pipeline.o_usbp, pipeline.o_usbn)
                if oe:
                    waveform.append({(1, 0): "J", (0, 1): "K",
                                     (0, 0): "_", (1, 1): "X"}[dp, dn])
                elif waveform:
                    return

        async def bench(ctx):
            pipeline = dut.pipeline
            await ctx.tick("usb").repeat(5)
            ctx.set(pipeline.i_data_payload, 0xd2)  # ACK.
            ctx.set(pipeline.i_oe, 1)
            for _ in range(100):
                _, _, ready = await ctx.tick("usb").sample(pipeline.o_data_strobe)
                if ready:
                    break
            else:
                self.fail("transmitter never accepted the ACK byte")
            ctx.set(pipeline.i_oe, 0)
            await ctx.tick("usb").repeat(30)

        sim.add_testbench(capture, background=True)
        sim.add_testbench(bench)
        sim.run()
        # Verify all 76 physical samples: complete SYNC, exactly eight
        # PID bits and normal EOP, with the ACK PID intact.
        expected = usb_packet.wrap_packet(
            usb_packet.encode_data([0xd2]), cycles=4)
        self.assertEqual(len(waveform), 76)
        self.assertEqual("".join(waveform), expected)


class ProductionUSBSerialFixture(a.Elaboratable):
    """The complete production ACM wrapper with independently driven pins."""

    def __init__(self):
        self.io = raw_usb_record()
        self.serial = USBSerialDevice(bus=self.io, idVendor=0x1209, idProduct=1)
        self.core = self.serial
        self.connect = self.core.connect
        self.usb_reset = a.Signal(init=1)
        self.io_reset = a.Signal(init=1)
        self.host_oe = a.Signal()
        self.host_dp = a.Signal()
        self.host_dn = a.Signal()

    def elaborate(self, platform):
        m = a.Module()
        m.domains.usb = a.ClockDomain("usb")
        m.domains.usb_io = a.ClockDomain("usb_io")
        m.submodules.core = self.core
        m.d.comb += [
            a.ResetSignal("usb").eq(self.usb_reset),
            a.ResetSignal("usb_io").eq(self.io_reset),
            self.core.rx.ready.eq(1),
            self.io.d_p.i.eq(a.Mux(self.host_oe, self.host_dp,
                                  a.Mux(self.io.d_p.oe, self.io.d_p.o, self.io.pullup.o))),
            self.io.d_n.i.eq(a.Mux(self.host_oe, self.host_dn,
                                  a.Mux(self.io.d_n.oe, self.io.d_n.o, 0))),
        ]
        return m


class USBProductionDescriptorTests(unittest.TestCase):
    @staticmethod
    def expected_descriptors():
        # Fixed expected descriptors make this an independent check of the
        # actual ACM data, rather than deriving expectations from its ROM.
        return {
            1: bytes.fromhex("120100020000004009120100000001020301"),
            2: bytes.fromhex(
                "0902470002010080fa080b000202020100090400000102020100"
                "0524001001052406000105240100010705830340000b"
                "09040100020a000000070584024000ff070504024000ff"),
            (3, 0): bytes.fromhex("04030904"),
            (3, 1): bytes.fromhex("0a034c0055004e004100"),
            (3, 2): bytes.fromhex("1c035500530042002d0074006f002d00730065007200690061006c00"),
        }

    def test_physical_descriptors_address_and_turnaround_across_clock_phases(self):
        descriptors = self.expected_descriptors()
        # The vendored ACM default, independent of environment-wide options,
        # must select the faster descriptor implementation.
        environment = {key: value for key, value in os.environ.items()
                       if key != "LUNA_AVOID_BLOCKRAM"}
        with patch.dict(os.environ, environment, clear=True):
            for clock_phase in range(4):
                for reset_phase in range(4):
                    with self.subTest(clock_phase=clock_phase, reset_phase=reset_phase):
                        self.check_enumeration(descriptors, clock_phase, reset_phase)

    def test_fractional_related_clock_phases_preserve_complete_native_transmit_packets(self):
        descriptors = self.expected_descriptors()
        environment = {key: value for key, value in os.environ.items()
                       if key != "LUNA_AVOID_BLOCKRAM"}
        with patch.dict(os.environ, environment, clear=True):
            for clock_phase in (0.01, 0.25, 0.5, 0.75, 0.99):
                for reset_phase in range(4):
                    with self.subTest(clock_phase=clock_phase, reset_phase=reset_phase):
                        self.check_enumeration(descriptors, clock_phase, reset_phase,
                                               extended=False)

    def check_enumeration(self, descriptors, clock_phase, reset_phase, *, extended=True):
        dut = ProductionUSBSerialFixture()
        sim = AmaranthSimulator(dut)
        # Even femtosecond periods keep the simulated PLL clocks exactly 4:1.
        # Independently rounding 1/12e6 and 1/48e6 introduces artificial drift.
        fast_period = 20_833_334e-15
        sim.add_clock(4 * fast_period, phase=clock_phase * fast_period, domain="usb")
        sim.add_clock(fast_period, phase=0, domain="usb_io")
        current = [None]

        async def response_edges(ctx):
            while True:
                await ctx.changed(dut.io.d_p.oe, dut.io.d_n.oe,
                                  dut.io.d_p.o, dut.io.d_n.o)
                if (current[0] is not None and ctx.get(dut.io.d_p.oe)
                        and not ctx.get(dut.io.d_p.o) and ctx.get(dut.io.d_n.o)):
                    current[0].setdefault("response", sim._engine.now)

        async def bench(ctx):
            host = PhysicalUSBHost(self, ctx, dut)

            async def timed_in(address):
                row = {}
                current[0] = row
                waveform = usb_packet.wrap_packet(
                    usb_packet.token_packet(USBPacketID.IN, address, 0), cycles=4)
                eop_j = waveform.rfind("_") + 1
                ctx.set(dut.host_oe, 1)
                for index, symbol in enumerate(waveform):
                    dp, dn = {"J": (1, 0), "K": (0, 1), "_": (0, 0)}[symbol]
                    ctx.set(dut.host_dp, dp)
                    ctx.set(dut.host_dn, dn)
                    if index == eop_j:
                        row["request"] = sim._engine.now
                    await ctx.tick("usb_io")
                ctx.set(dut.host_oe, 0)
                return row

            async def expect_data(row, pid, payload):
                # Every physical sample is checked: PID, payload, CRC,
                # bit stuffing, NRZI, SYNC, EOP and packet length.
                await host.expect(usb_packet.data_packet(pid, payload))
                current[0] = None
                self.assertIn("response", row)
                # Engine time is in femtoseconds. Timestamp pin transitions
                # directly so receive() sampling cannot add a clock tick.
                cycles = (row["response"] - row["request"]) / (fast_period * 1e15)
                self.assertGreater(cycles, 0)
                self.assertLessEqual(cycles, 52.001,
                                     "descriptor reply lost the measured host-timeout margin")

            async def silence():
                for _ in range(160):
                    await ctx.tick("usb_io")
                    self.assertFalse(ctx.get(dut.io.d_p.oe))
                    self.assertFalse(ctx.get(dut.io.d_n.oe))

            async def setup(address, payload):
                await host.send(usb_packet.token_packet(USBPacketID.SETUP, address, 0))
                await host.gap()
                await host.send(usb_packet.data_packet(USBPacketID.DATA0, payload))
                await host.expect(usb_packet.handshake_packet(USBPacketID.ACK))

            async def descriptor(address, kind, length, reject_address=None, index=0):
                language = 0x0409 if kind == 3 and index else 0
                await setup(address, [0x80, 6, index, kind,
                                      language & 0xff, language >> 8, length, 0])
                if reject_address is not None:
                    # EP0 is now ready to send data, so silence at the old
                    # address proves filtering rather than an idle endpoint.
                    await host.send(usb_packet.token_packet(
                        USBPacketID.IN, reject_address, 0))
                    await silence()
                key = (kind, index) if kind == 3 else kind
                payload = descriptors[key][:length]
                for index, offset in enumerate(range(0, len(payload), 64)):
                    row = await timed_in(address)
                    pid = USBPacketID.DATA1 if index % 2 == 0 else USBPacketID.DATA0
                    await expect_data(row, pid, payload[offset:offset + 64])
                    await host.send(usb_packet.handshake_packet(USBPacketID.ACK))
                    await ctx.tick("usb_io").repeat(16)
                await host.send(usb_packet.token_packet(USBPacketID.OUT, address, 0))
                await host.gap()
                await host.send(usb_packet.data_packet(USBPacketID.DATA1, []))
                await host.expect(usb_packet.handshake_packet(USBPacketID.ACK))
                await ctx.tick("usb_io").repeat(80)

            # Individual ticks permit observing the deliberately asserted
            # domain reset; repeat() raises DomainReset during that interval.
            for _ in range(16):
                await ctx.tick("usb_io")
            ctx.set(dut.usb_reset, 0)
            for _ in range(reset_phase + 1):
                await ctx.tick("usb_io")
            ctx.set(dut.io_reset, 0)
            await ctx.tick("usb_io").repeat(80)
            ctx.set(dut.connect, 1)
            await ctx.tick("usb_io").repeat(80)
            ctx.set(dut.host_oe, 1)
            ctx.set(dut.host_dp, 0)
            ctx.set(dut.host_dn, 0)
            await ctx.tick("usb_io").repeat(4800)
            ctx.set(dut.host_oe, 0)
            await ctx.tick("usb_io").repeat(480)

            requests = ((1, 8), (1, 18), (1, 64), (2, 9), (2, 71)) if extended else (
                (1, 18), (2, 71))
            for kind, length in requests:
                await descriptor(0, kind, length)
            # Exercise direct lookup's string-index selection and unsupported
            # type handling, then prove a fresh SETUP recovers from STALL.
            if extended:
                for index in range(3):
                    await descriptor(0, 3, 255, index=index)
                await setup(0, [0x80, 6, 0, 6, 0, 0, 10, 0])
                await host.send(usb_packet.token_packet(USBPacketID.IN, 0, 0))
                await host.expect(usb_packet.handshake_packet(USBPacketID.STALL))

            await setup(0, [0, 5, 7, 0, 0, 0, 0, 0])
            await host.send(usb_packet.token_packet(USBPacketID.IN, 7, 0))
            await silence()
            # Retry the status packet without acknowledging its first copy:
            # address must change on ACK, not on transmission of the ZLP.
            for _ in range(2):
                await expect_data(await timed_in(0), USBPacketID.DATA1, [])
            await host.send(usb_packet.handshake_packet(USBPacketID.ACK))
            await ctx.tick("usb_io").repeat(80)
            await descriptor(7, 1, 18, reject_address=0)

        sim.add_testbench(response_edges, background=True)
        sim.add_testbench(bench)
        sim.run()

class USBWrapperDefaultTests(unittest.TestCase):
    def test_transfer_markers_reach_luna(self):
        usb = luna_wrapper()
        for name in ("first", "last", "valid", "data"):
            binding = "payload" if name == "data" else name
            self.assertIs(usb.core_params["i_tx_" + binding], getattr(usb.tx_cdc.source, name))


if __name__ == "__main__":
    unittest.main()
