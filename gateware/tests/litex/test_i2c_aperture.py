"""Physical camera windows and cancelled host words through the SoC decoder."""
from pathlib import Path
import sys
import unittest

ROOT = next(path for path in Path(__file__).resolve().parents
            if (path / "fpga/litex/lutcam/i2c.py").exists())
sys.path[:0] = [str(ROOT / "fpga/litex"), str(ROOT / "tests/litex")]

from migen import Module, ResetInserter, Signal
from migen.sim import run_simulation
from litex.soc.interconnect import wishbone
from lutcam.i2c import SharedCameraI2C
import test_i2c_engine as wire


class _PhysicalAddress:
    def __init__(self, signal, camera):
        self.signal, self.base = signal, (0x30020000 + camera * 0x1000) // 4

    def eq(self, value):
        return self.signal.eq(self.base + value)


class _Aperture:
    """Python-only address adapter; the hardware sees one physical master."""
    def __init__(self, master, camera, engine):
        self.master, self._i2c_engine = master, engine
        self.adr = _PhysicalAddress(master.adr, camera)

    def __getattr__(self, name):
        return getattr(self.master, name)


class PhysicalHarness(Module):
    def __init__(self, single=True):
        options = dict(fast_low_cycles=6, fast_high_cycles=4, stretch_timeout_cycles=240)
        core = SharedCameraI2C(single_aperture=True, **options) if single else SharedCameraI2C(**options)
        self.submodules.engine = ResetInserter()(core)
        master = wishbone.Interface(data_width=32, addressing="word")
        slaves = [(lambda address: address[11:] == (0x30020000 >> 13), core.bus)] if single else [
            (lambda address: address[10:] == (0x30020000 >> 12), core.buses[0]),
            (lambda address: address[10:] == (0x30021000 >> 12), core.buses[1]),
        ]
        self.submodules.decoder = wishbone.Decoder(master, slaves, register=True)
        core.buses = tuple(_Aperture(master, camera, core) for camera in range(2))
        self.target_scl_low = [Signal(), Signal()]
        self.target_sda_low = [Signal(), Signal()]
        for camera in range(2):
            self.comb += [
                core.scl[camera].i.eq(~(core.scl[camera].oe | self.target_scl_low[camera])),
                core.sda[camera].i.eq(~(core.sda[camera].oe | self.target_sda_low[camera])),
            ]


class PhysicalI2CApertureTests(unittest.TestCase):
    def simulate(self, bench, single=True):
        dut = PhysicalHarness(single)
        targets = [wire.RegisterTarget(dut, camera, address)
                   for camera, address in enumerate((0x60, 0x10))]
        run_simulation(dut, [bench(dut, targets), *(t.process() for t in targets)])

    def test_physical_windows_keep_controls_and_wire_ownership_separate(self):
        def bench(dut, targets):
            for camera, bus in enumerate(dut.engine.buses):
                yield from wire.write_word(bus, wire.CONTROL, camera + 1)
            self.assertEqual((yield from wire.read_word(dut.engine.buses[0], wire.CONTROL)), 1)
            self.assertEqual((yield from wire.read_word(dut.engine.buses[1], wire.CONTROL)), 2)
            for bus in dut.engine.buses:
                yield from wire.write_word(bus, wire.CONTROL, 0)
            for _ in range(10):
                yield
            command = 0
            for camera, bus in enumerate(dut.engine.buses):
                prior_transactions = len(targets[camera].transactions)
                payload = bytes((camera * 19 + byte * 31) & 255 for byte in range(16))
                yield from wire.stage(bus, targets[camera].address, 0x4200, payload)
                command += 1
                yield from wire.submit(bus, command, continuous=True)
                self.assertEqual((yield from wire.wait_done(bus, command)), 16 << 16)
                self.assertEqual(len(targets[camera].transactions), prior_transactions + 1)
                yield from wire.stage(bus, targets[camera].address, 0x4200, length=16, read=True)
                command += 1
                yield from wire.submit(bus, command)
                self.assertEqual((yield from wire.wait_done(bus, command)), 16 << 16)
                for index in range(4):
                    self.assertEqual((yield from wire.read_word(bus, wire.RX + index * 4)),
                                     int.from_bytes(payload[index*4:index*4+4], "little"))
        for single in (False, True):
            with self.subTest(single=single):
                self.simulate(bench, single)

    def pending_word(self, dut, camera, write, delay):
        bus = dut.engine.buses[camera]
        yield from wire.record_word(bus, 0, 1, first=True, count=4, source=2)
        yield from wire.record_word(bus, 1, wire.descriptor(0x60, 0x4300, 4), count=4, source=2)
        if not write:
            yield from wire.record_word(bus, 2, 0x44332211, count=4, source=2)
            yield dut.engine.record_abort_sources.eq(4)
            yield
            yield dut.engine.record_abort_sources.eq(0)
        yield bus.adr.eq((wire.RECORD + 8 if write else wire.RX) // 4)
        yield bus.dat_w.eq(0x88776655)
        yield bus.sel.eq(15)
        yield bus.we.eq(write)
        yield bus.cyc.eq(1)
        yield bus.stb.eq(1)
        # Sample after acceptance, before a serialized word can acknowledge.
        for _ in range(2 + delay):
            yield
        self.assertFalse((yield bus.ack))

    def test_cancelled_word_cannot_ack_next_camera_request_or_change_payload(self):
        for single in (False, True):
            for camera in range(2):
                for destination in range(2):
                    for write, delay in ((True, 0), (False, 0), (False, 3)):
                        with self.subTest(single=single, camera=camera,
                                          destination=destination, write=write, delay=delay):
                            def bench(dut, targets):
                                yield from self.pending_word(dut, camera, write, delay)
                                bus, other = dut.engine.buses[camera], dut.engine.buses[destination]
                                yield bus.cyc.eq(0)
                                yield bus.stb.eq(0)
                                yield bus.adr.eq(0x3fc // 4)
                                yield bus.dat_w.eq(0xdeadbeef)
                                yield dut.engine.record_abort_sources.eq(4)
                                yield
                                yield dut.engine.record_abort_sources.eq(0)
                                self.assertEqual((yield from wire.read_word(other, wire.MAGIC)), 0x49324301)
                                self.assertEqual((yield from wire.read_word(other, wire.RX)),
                                                 0x88776655 if write else 0x44332211)
                                self.assertEqual((yield from wire.read_word(other, wire.COMMAND)), 0)
                                self.assertEqual((yield from wire.read_word(other, wire.NEXT)), 1)
                                self.assertTrue(all(not target.starts for target in targets))
                            self.simulate(bench, single)

    def test_reset_discards_pending_word_and_restores_zero_readback(self):
        for single in (False, True):
            for write, delay in ((True, 0), (False, 0), (False, 3)):
                with self.subTest(single=single, write=write, delay=delay):
                    def bench(dut, targets):
                        yield from self.pending_word(dut, 1, write, delay)
                        yield dut.engine.reset.eq(1)
                        yield dut.engine.buses[1].cyc.eq(0)
                        yield dut.engine.buses[1].stb.eq(0)
                        for _ in range(3):
                            yield
                        yield dut.engine.reset.eq(0)
                        for _ in range(4):
                            yield
                        for bus in dut.engine.buses:
                            for index in range(4):
                                self.assertEqual((yield from wire.read_word(bus, wire.RX + 4*index)), 0)
                            self.assertEqual((yield from wire.read_word(bus, wire.NEXT)), 1)
                            self.assertEqual((yield from wire.read_word(bus, wire.STATUS)), 0)
                        self.assertTrue(all(not target.starts for target in targets))
                    self.simulate(bench, single)


if __name__ == "__main__":
    unittest.main()
