"""Staged classic Wishbone payload, abort, and counter ownership contracts."""
from pathlib import Path
from types import SimpleNamespace
import shutil
import subprocess
import sys
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'fpga' / 'litex'))
from migen import Module, Signal
from migen.fhdl import verilog
from migen.sim import run_simulation, passive
from lutcam.registers import IntegrationRegisters
from test_integration_registers import FakeCamera, transfer, packet_address, calibration_addresses
from test_pair_registers import FakePair

class StagedRequestHarness(Module):
    """Controllable counter responses exercise abort and ownership races."""

    def __init__(self, configuration_bank=None):
        self.cameras = [FakeCamera(), FakeCamera()]
        self.submodules.cam0, self.submodules.cam1 = self.cameras
        self.submodules.pair = FakePair()
        channels = []
        for i in range(6):
            channel = SimpleNamespace(**{
                name: Signal(32, name=f'ch{i}_{name}') for name in (
                    'enable', 'destination_ip', 'destination_port',
                    'active_destination_ip', 'active_destination_port',
                    'destination_update', 'clear', 'counter_committed_level',
                    'commit_event', 'sent_event', 'bytes_event', 'overflow_event',
                    'protocol_event', 'reject_event', 'fifo_level', 'high_water', 'faults',
                )
            })
            channel.configuration_idle = Signal(reset=1)
            channels.append(channel)
        self.transport = transport = SimpleNamespace(
            channels=channels, camera_reset=Signal(2), sequence_stream=Signal(3),
            sequence_request=Signal(), sequence_valid=Signal(), sequence_value=Signal(32),
            sequence_destination_ip=Signal(32), sequence_destination_port=Signal(16),
        )
        self.counters = counters = SimpleNamespace(
            query_valid=Signal(), query_ready=Signal(reset=1), query_address=Signal(6),
            query_response_valid=Signal(), query_data=Signal(32),
            config_write_valid=Signal(), config_write_ready=Signal(reset=1),
            config_write_address=Signal(4), config_write_data=Signal(32),
            clear_camera=Signal(2), clear_statistics=Signal(2),
            committed_levels=[Signal(16) for _ in range(6)],
            events=[SimpleNamespace(**{name: Signal(32) for name in (
                'committed', 'sent', 'bytes', 'overflow', 'protocol', 'reject',
            )}) for _ in range(6)],
        )
        self.submodules.regs = IntegrationRegisters(self.cameras, transport, counters,
            banked_destinations=True, pair=self.pair, configuration_bank=configuration_bank)
        self.events = []

    def monitor(self):

        @passive
        def run():
            while True:
                event = {}
                event['reset'] = (yield self.pair.reset)
                event['snapshot'] = (yield self.pair.snapshot)
                event['query'] = (yield self.counters.query_valid) and (yield self.counters.query_ready)
                event['query_address'] = (yield self.counters.query_address)
                event['config'] = (yield self.counters.config_write_valid) and (yield self.counters.config_write_ready)
                event['config_data'] = (yield self.counters.config_write_data)
                if any((event[n] for n in ('reset', 'snapshot', 'query', 'config'))):
                    self.events.append(event)
                yield
        return run()

def set_request(bus, address, data=None, sel=15):
    yield bus.adr.eq(address // 4)
    yield bus.we.eq(data is not None)
    yield bus.dat_w.eq(data or 0)
    yield bus.sel.eq(sel)
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)

def cancel(bus, signal='cyc'):
    yield getattr(bus, signal).eq(0)
    yield
    yield bus.cyc.eq(0)
    yield bus.stb.eq(0)
    yield

def wait_response(bus):
    for _ in range(100):
        if (yield bus.ack) or (yield bus.err):
            return ((yield bus.dat_r), (yield bus.err))
        yield
    raise AssertionError('response timeout')

class IntegrationRequestStageTests(unittest.TestCase):

    def test_abort_after_capture_has_no_side_effect_for_cyc_or_stb(self):
        for drop in ('cyc', 'stb'):
            for address in (0x210, 0x228, calibration_addresses(0)['inv_fx_q2_30'], packet_address(0, 9)):
                with self.subTest(drop=drop, address=address):
                    dut = StagedRequestHarness()
                    bus = dut.regs.bus

                    def bench():
                        yield from set_request(bus, address, None if address == packet_address(0, 9) else 0x12345678)
                        yield
                        yield from cancel(bus, drop)
                        for _ in range(8):
                            self.assertFalse((yield bus.ack) or (yield bus.err))
                            yield
                        self.assertEqual((yield dut.cameras[0].configuration['inv_fx_q2_30']), 0)
                        self.assertFalse(dut.events)
                        self.assertEqual((yield from transfer(bus, 0x200)), (0x4c435031, 0))
                    run_simulation(dut, [bench(), dut.monitor()])

    def test_continuous_classic_requests_strobe_once_and_keep_payload(self):
        dut = StagedRequestHarness()
        bus = dut.regs.bus

        def bench():
            for address in (0x228, 0x210, 0x228):
                yield from set_request(bus, address, 1)
                yield
                result = (yield from wait_response(bus))
                self.assertEqual(result, (0, 0))
                yield
            yield from cancel(bus)
            self.assertEqual(sum((event['snapshot'] for event in dut.events)), 2)
            self.assertEqual(sum((event['reset'] for event in dut.events)), 1)
        run_simulation(dut, [bench(), dut.monitor()])

    def test_captured_payload_and_live_guard_at_decode(self):
        dut = StagedRequestHarness()
        bus = dut.regs.bus
        address = calibration_addresses(0)['inv_fx_q2_30']

        def bench():
            yield from set_request(bus, address, 0x12345678)
            yield
            yield bus.dat_w.eq(0xffffffff)
            yield bus.adr.eq(0x210 // 4)
            yield bus.sel.eq(0)
            yield
            self.assertEqual((yield from wait_response(bus)), (0, 0))
            yield from cancel(bus)
            self.assertEqual((yield dut.cameras[0].configuration['inv_fx_q2_30']), 0x12345678)
            self.assertFalse(any((event['reset'] for event in dut.events)))
            yield from set_request(bus, address, 0xabcdef01)
            yield
            yield dut.cameras[1].idle.eq(0)
            yield
            self.assertEqual((yield from wait_response(bus))[1], 1)
            yield from cancel(bus)
            self.assertEqual((yield dut.cameras[0].configuration['inv_fx_q2_30']), 0x12345678)
        run_simulation(dut, [bench(), dut.monitor()])

    def test_abandoned_counter_response_cannot_feed_new_partial_write(self):
        dut = StagedRequestHarness()
        bus = dut.regs.bus
        counters = dut.counters

        def bench():
            yield from set_request(bus, packet_address(0, 9))
            for _ in range(20):
                yield
                if (yield counters.query_valid):
                    break
            else:
                self.fail('old query not accepted')
            yield
            yield from cancel(bus)
            yield from set_request(bus, packet_address(1, 1), 0xaabbccdd, sel=5)
            yield
            yield
            yield
            self.assertFalse((yield counters.query_valid))
            yield counters.query_data.eq(0xdeadbeef)
            yield counters.query_response_valid.eq(1)
            yield
            yield counters.query_response_valid.eq(0)
            for _ in range(20):
                yield
                if (yield counters.query_valid):
                    break
            else:
                self.fail('replacement query blocked')
            self.assertEqual((yield counters.query_address), 38)
            yield
            yield counters.query_data.eq(0x12345678)
            yield counters.query_response_valid.eq(1)
            yield
            yield counters.query_response_valid.eq(0)
            self.assertEqual((yield from wait_response(bus)), (0x12345678, 0))
            yield from cancel(bus)
            writes = [event['config_data'] for event in dut.events if event['config']]
            self.assertEqual(writes, [0x12bb56dd])
        run_simulation(dut, [bench(), dut.monitor()])

    def test_aborted_ram_write_is_cancelled_while_not_ready(self):
        dut = StagedRequestHarness()
        bus = dut.regs.bus
        counters = dut.counters

        def bench():
            yield counters.config_write_ready.eq(0)
            yield from set_request(bus, packet_address(0, 1), 0x12345678)
            for _ in range(10):
                yield
                if (yield counters.config_write_valid):
                    break
            else:
                self.fail('configuration write not issued')
            yield from cancel(bus, 'stb')
            yield counters.config_write_ready.eq(1)
            for _ in range(5):
                self.assertFalse((yield counters.config_write_valid))
                yield
            self.assertFalse(any((event['config'] for event in dut.events)))
            self.assertEqual((yield from transfer(bus, 0x200)), (0x4c435031, 0))
        run_simulation(dut, [bench(), dut.monitor()])

    def test_packet_sequence_has_priority_over_staged_counter_read(self):
        dut = StagedRequestHarness()
        bus = dut.regs.bus
        counters = dut.counters
        transport = dut.transport

        def respond(address, value):
            for _ in range(30):
                yield
                if (yield counters.query_valid):
                    break
            else:
                self.fail('expected query missing')
            self.assertEqual((yield counters.query_address), address)
            yield
            yield counters.query_data.eq(value)
            yield counters.query_response_valid.eq(1)
            yield
            yield counters.query_response_valid.eq(0)

        def bench():
            yield transport.sequence_stream.eq(4)
            yield transport.sequence_request.eq(1)
            yield from set_request(bus, packet_address(1, 9))
            yield from respond(44, 0xc0a8010a)
            yield from respond(45, 0x1b5c)
            yield from respond(25, 0x55556666)
            self.assertEqual((yield transport.sequence_valid), 1)
            self.assertEqual((yield transport.sequence_value), 0x55556666)
            self.assertEqual((yield transport.sequence_destination_ip), 0xc0a8010a)
            self.assertEqual((yield transport.sequence_destination_port), 0x1b5c)
            yield transport.sequence_request.eq(0)
            yield from respond(6, 0x12345678)
            self.assertEqual((yield from wait_response(bus)), (0x12345678, 0))
            yield from cancel(bus)
            self.assertEqual([event['query_address'] for event in dut.events if event['query']], [44, 45, 25, 6])
        run_simulation(dut, [bench(), dut.monitor()])
    @unittest.skipUnless(shutil.which("iverilog") and shutil.which("vvp"), "Icarus required")
    def test_generated_rtl_request_ack_strobe_and_abort(self):
        dut = StagedRequestHarness()
        bus = dut.regs.bus
        ports = {"adr": bus.adr, "dat_w": bus.dat_w, "sel": bus.sel,
                 "we": bus.we, "cyc": bus.cyc, "stb": bus.stb,
                 "ack": bus.ack, "err": bus.err, "dat_r": bus.dat_r,
                 "inv_fx": dut.cameras[0].configuration["inv_fx_q2_30"],
                 "snapshot": dut.pair.snapshot}
        for name, signal in ports.items():
            signal.name_override = name
        with tempfile.TemporaryDirectory(prefix="lutcam-register-stage-") as directory:
            path = Path(directory)
            generated = path / "staged.v"
            generated.write_text(str(verilog.convert(dut, ios=set(ports.values()), name="staged")))
            executable = path / "smoke.vvp"
            subprocess.run(["iverilog", "-g2012", "-s", "tb", "-o", str(executable),
                            str(generated), str(Path(__file__).with_name("integration_registers_stage_tb.v"))],
                           check=True, capture_output=True, text=True)
            result = subprocess.run(["vvp", str(executable)], check=True, capture_output=True, text=True)
            self.assertIn("PASS generated RTL request/ACK/strobe/abort", result.stdout)


if __name__ == '__main__':
    unittest.main(verbosity=2)
