"""Real CSR/transport recovery with an unfinished held-frame parser.

Camera status/admission is modeled; the actual IntegrationRegisters, complete
production-capacity transport and counter bank are composed. This covers the
faulted/drained exception where an uncommitted pose prefix or child group is
still owned, and checks the same-edge producer/transport reset wiring.
"""
import unittest
from migen import If, Module, Signal
from migen.sim import passive, run_simulation
from fpga.litex.lutcam.counters import SharedPacketCounters
from fpga.litex.lutcam.packet import SharedPacketTransport
from fpga.litex.lutcam.registers import IntegrationRegisters
from test_integration_registers import FakeCamera, camera_address, packet_address, transfer
from test_pair_registers import FakePair


class Harness(Module):
    def __init__(self):
        self.cameras = [FakeCamera(), FakeCamera()]
        self.submodules.camera0, self.submodules.camera1 = self.cameras
        self.submodules.pair = FakePair()
        self.submodules.transport = SharedPacketTransport(
            child_slots=8, frame_words=64000, external_destination=True,
            external_context_held=True, compact_padding=True, batched_packets=True,
            accepted_centroids=True, shared_solver_ingress=True, held_frame_id=True)
        self.submodules.counters = SharedPacketCounters(packet_totals=True,
            with_destinations=True, fixed_packet_layout=True, with_host_counters=True,
            compact_image_packets=True, batched_packets=True, serialized_datagrams=True,
            serialized_event_descriptor=True)
        self.submodules.registers = IntegrationRegisters(self.cameras, self.transport,
            self.counters, banked_destinations=True, pair=self.pair)
        self.comb += self.pair.drained.eq(self.transport.drained)
        self.frames = [Signal(32, reset=0xffffffff), Signal(32, reset=0x80000001)]
        for index, camera in enumerate(self.cameras):
            # The only modeled producer transition is CameraPipeline's paired
            # diagnostic-clear/soft-reset frame-ID assignment.
            self.sync += If(camera.clear.re | camera.soft_reset.re, self.frames[index].eq(0))
            for kind in range(3):
                self.comb += self.transport.channels[3*index+kind].sink.frame_id.eq(self.frames[index])
        self.events = []


def monitor(dut):
    @passive
    def run():
        while True:
            if (yield dut.pair.clear) or (yield dut.pair.reset):
                row = [(yield dut.pair.clear), (yield dut.pair.reset)]
                for camera in range(2):
                    row.extend(((yield dut.cameras[camera].clear.re),
                                (yield dut.cameras[camera].soft_reset.re),
                                (yield dut.transport.camera_reset[camera]),
                                (yield dut.frames[camera])))
                dut.events.append(tuple(row))
            yield
    return run()


def beat(sink, data, last=False, end=False):
    yield sink.valid.eq(1)
    yield sink.data.eq(data)
    yield sink.last.eq(last)
    if hasattr(sink, 'end'):
        yield sink.end.eq(end)
    yield
    for _ in range(100):
        if (yield sink.ready):
            yield sink.valid.eq(0)
            return
        yield
    raise AssertionError('producer did not become ready')


class HeldFrameRegisterRecoveryTests(unittest.TestCase):
    def test_faulted_partial_owner_resets_with_camera_id_through_real_csr(self):
        for camera in range(2):
            for kind in ('pose', 'child'):
                for operation in ('clear', 'reset'):
                    with self.subTest(camera=camera, kind=kind, operation=operation):
                        dut = Harness()
                        def bench():
                            bus, pair = dut.registers.bus, dut.pair
                            selected = 3*camera + (2 if kind == 'pose' else 1)
                            channel = dut.transport.channels[selected]
                            self.assertEqual((yield from transfer(bus, packet_address(selected, 0), 1))[1], 0)
                            yield pair.run_request.eq(1)
                            yield
                            if kind == 'pose':
                                yield from beat(channel.sink, 0x50700000)
                                yield from beat(channel.sink, 0x813579bd)
                            else:
                                yield from beat(channel.sink, 0x1f2345678)
                                yield from beat(channel.sink, 0x1e2345678)
                            # Model paired abort: no replacement owner, old ID
                            # retained, nonidle sticky faults, stopped controller.
                            yield pair.run_request.eq(0)
                            yield pair.fault.eq(1)
                            for cam in dut.cameras:
                                yield cam.idle.eq(0)
                            for _ in range(8):
                                yield
                            self.assertEqual((yield dut.transport.drained), 1)
                            self.assertEqual((yield channel.counter_committed_level), 0)
                            self.assertGreater((yield channel.fifo_level), 0)
                            for index in range(2):
                                for name in ('clear', 'soft_reset'):
                                    self.assertEqual((yield from transfer(bus, camera_address(index, name), 1))[1], 1)
                            # A stream-local clear is not a substitute for paired
                            # recovery while either camera is faulted/nonidle.
                            self.assertEqual((yield from transfer(bus, packet_address(selected, 8), 1))[1], 1)
                            self.assertEqual(dut.events, [])
                            self.assertEqual((yield dut.frames[camera]), 0xffffffff if camera == 0 else 0x80000001)
                            address = 0x260 if operation == 'clear' else 0x210
                            self.assertEqual((yield from transfer(bus, address, 1))[1], 0)
                            for _ in range(5):
                                yield
                            self.assertEqual(len(dut.events), 1)
                            expected_pair = (1, 0) if operation == 'clear' else (0, 1)
                            self.assertEqual(dut.events[0][:2], expected_pair)
                            for index in range(2):
                                expected_cam = (*expected_pair, 1, 0xffffffff if index == 0 else 0x80000001)
                                self.assertEqual(dut.events[0][2+4*index:6+4*index], expected_cam)
                                self.assertEqual((yield dut.frames[index]), 0)
                                for product in range(3):
                                    self.assertEqual((yield dut.transport.channels[3*index+product].fifo_level), 0)
                            self.assertEqual((yield dut.transport.source.valid), 0)
                            self.assertEqual((yield dut.transport.drained), 1)
                            # Reuse the same parser with frame0 and no stale old
                            # prefix/tuple. Soft reset also clears stream enable.
                            yield pair.fault.eq(0)
                            for cam in dut.cameras:
                                yield cam.idle.eq(1)
                            yield
                            if operation == 'reset':
                                self.assertEqual((yield from transfer(bus, packet_address(selected, 0), 1))[1], 0)
                            yield pair.run_request.eq(1)
                            yield
                            if kind == 'pose':
                                for index in range(22):
                                    word = 0x813579bd if index == 1 else 0x00000001 if index == 8 else 0x50700000+index
                                    yield from beat(channel.sink, word, last=index == 21)
                            else:
                                for index in range(9):
                                    yield from beat(channel.sink, 0x112340000+index, end=index == 8)
                            for _ in range(8):
                                yield
                            self.assertEqual((yield channel.faults), 0)
                            self.assertEqual((yield channel.fifo_level), 1)
                            self.assertEqual((yield dut.transport.drained), 0)
                        run_simulation(dut, [bench(), monitor(dut)])


if __name__ == '__main__':
    unittest.main()
