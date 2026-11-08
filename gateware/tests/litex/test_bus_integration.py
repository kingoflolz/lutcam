"""Concurrent remote hosts through the real LiteX shared Wishbone fabric."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from migen import If, Module, ResetInserter, Signal
from migen.sim import passive, run_simulation
from litex.soc.interconnect import wishbone

from lutcam.control import CameraI2C, HistogramReader, WishboneGuard


I2C_BASES = (0x30020000, 0x30021000)


def transfer(bus, address, data=None, *, keep_cycle=False, limit=160):
    yield bus.adr.eq(address >> 2)
    yield bus.dat_w.eq(0 if data is None else data)
    yield bus.we.eq(data is not None)
    yield bus.sel.eq(0xf)
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)
    yield
    for _ in range(limit):
        if (yield bus.ack) or (yield bus.err):
            result = (yield bus.dat_r), (yield bus.err)
            if not keep_cycle:
                yield bus.cyc.eq(0)
                yield bus.stb.eq(0)
                yield
            return result
        yield
    raise AssertionError("remote host did not receive a bounded response")


class SharedControlFixture(Module):
    def __init__(self):
        self.hosts = []
        self.cameras = []
        self.writes = []
        for index in range(2):
            host = WishboneGuard(timeout_cycles=64)
            camera = CameraI2C()
            setattr(self.submodules, "host" + str(index), host)
            setattr(self.submodules, "camera" + str(index), camera)
            self.hosts.append(host)
            self.cameras.append(camera)
            writes = Signal(16)
            self.writes.append(writes)
            bus = camera.bus
            self.sync += If(bus.cyc & bus.stb & ~bus.ack & ~bus.err & bus.we
                            & bus.sel[0] & (bus.adr[:10] == 0), writes.eq(writes + 1))
            # Physical readback is distinct for each camera, independent of
            # software-driven output enables, to verify correct bus routing.
            self.comb += [camera.scl.i.eq(1), camera.sda.i.eq(index)]
        self.submodules.fabric = wishbone.InterconnectShared(
            [host.master for host in self.hosts],
            [(lambda adr, base=base: adr[10:] == (base >> 12), camera.bus)
             for base, camera in zip(I2C_BASES, self.cameras)],
            register=True, timeout_cycles=None,
        )
        for index, host in enumerate(self.hosts):
            self.comb += host.granted.eq(self.fabric.arbiter.rr.grant == index)


class BusIntegrationTests(unittest.TestCase):
    def test_concurrent_hosts_arbitrate_bursts_and_both_camera_apertures(self):
        dut = SharedControlFixture()
        finished = [False, False]
        contended = []

        def host_program(index):
            host = dut.hosts[index]
            own = I2C_BASES[index]
            peer = I2C_BASES[1 - index]
            for step in range(12):
                value = (step + index) & 3
                _, error = yield from transfer(host.bus, own, value, keep_cycle=True)
                self.assertEqual(error, 0)
                self.assertEqual((yield from transfer(host.bus, own, keep_cycle=True)), (value, 0))
                self.assertEqual((yield from transfer(host.bus, peer + 4, keep_cycle=True)),
                                 (1 | ((1 - index) << 1), 0))
            yield host.bus.cyc.eq(0)
            yield host.bus.stb.eq(0)
            yield
            self.assertEqual((yield host.fault), 0)
            self.assertEqual((yield dut.writes[index]), 12)
            finished[index] = True

        @passive
        def monitor():
            while True:
                if (yield dut.hosts[0].master.cyc) and (yield dut.hosts[1].master.cyc):
                    contended.append(1)
                selected = [(yield dut.cameras[0].bus.cyc), (yield dut.cameras[1].bus.cyc)]
                self.assertLessEqual(sum(selected), 1)
                yield

        run_simulation(dut, [host_program(0), host_program(1), monitor()])
        self.assertEqual(finished, [True, True])
        self.assertGreater(len(contended), 20)

    def test_unmapped_host_cannot_time_out_waiting_valid_host(self):
        dut = SharedControlFixture()
        finished = [False, False]
        # Both requests arrive together. Host zero initially owns the arbiter
        # and blocks it for its complete 64-cycle timeout. Host one's timeout
        # must start only after ownership transfers to it.
        def unmapped_host():
            host = dut.hosts[0]
            self.assertEqual((yield from transfer(host.bus, 0x31000000)), (0xffffffff, 0))
            self.assertEqual((yield host.fault_status), 2)
            self.assertEqual((yield host.fault_count), 1)
            self.assertEqual((yield from transfer(host.bus, I2C_BASES[1] + 4)), (3, 0))
            finished[0] = True

        def valid_host():
            host = dut.hosts[1]
            self.assertEqual((yield from transfer(host.bus, I2C_BASES[0], 2))[1], 0)
            self.assertEqual((yield from transfer(host.bus, I2C_BASES[0])), (2, 0))
            self.assertEqual((yield host.fault_count), 0)
            self.assertEqual((yield dut.writes[0]), 1)
            finished[1] = True

        run_simulation(dut, [unmapped_host(), valid_host()])
        self.assertEqual(finished, [True, True])

    def test_slave_error_and_unmapped_timeout_are_isolated_and_recover(self):
        dut = SharedControlFixture()

        def bench():
            for index, host in enumerate(dut.hosts):
                peer = dut.hosts[1 - index]
                base = I2C_BASES[index]
                # Writes to physical status are explicitly rejected by I2C.
                self.assertEqual((yield from transfer(host.bus, base + 4, 1)), (0xffffffff, 0))
                self.assertEqual((yield host.fault_address), base + 4)
                self.assertEqual((yield host.fault_status), 5)
                self.assertEqual((yield host.fault_count), 1)
                self.assertEqual((yield peer.fault_count), 0 if index == 0 else 2)
                self.assertEqual((yield dut.writes[index]), 0)
                unmapped = 0x31000000 + index * 4
                self.assertEqual((yield from transfer(host.bus, unmapped)), (0xffffffff, 0))
                self.assertEqual((yield host.fault_address), unmapped)
                self.assertEqual((yield host.fault_status), 2)
                self.assertEqual((yield host.fault_count), 2)
                # A terminated bad request releases the shared bus and both
                # remote hosts can still access either physical camera.
                self.assertEqual((yield from transfer(peer.bus, base, 3))[1], 0)
                self.assertEqual((yield from transfer(host.bus, base)), (3, 0))
                self.assertEqual((yield dut.writes[index]), 1)

        run_simulation(dut, bench())

    def test_histogram_soft_reset_cancels_partial_dump_and_allows_restart(self):
        reader = ResetInserter()(HistogramReader())

        def bench():
            yield reader.hist_dump_ready.eq(1)
            self.assertEqual((yield from transfer(reader.bus, 0, 1)), (0, 0))
            yield reader.hist_dump_ready.eq(0)
            yield reader.hist_out_valid.eq(1)
            yield reader.hist_out_data.eq(0x80000011)
            self.assertEqual((yield from transfer(reader.bus, 8)), (0x80000011, 0))
            self.assertEqual((yield reader.busy), 1)
            self.assertEqual((yield reader.words_read), 1)
            # Same reset pulse used by Camera for the ISP and its adapter.
            yield reader.reset.eq(1)
            yield reader.hist_out_valid.eq(0)
            yield
            yield reader.reset.eq(0)
            yield
            self.assertEqual((yield reader.busy), 0)
            self.assertEqual((yield reader.words_read), 0)
            yield reader.hist_dump_ready.eq(1)
            self.assertEqual((yield from transfer(reader.bus, 4)), (1, 0))
            self.assertEqual((yield from transfer(reader.bus, 0, 1)), (0, 0))
            self.assertEqual((yield reader.busy), 1)

        run_simulation(reader, bench())


if __name__ == "__main__":
    unittest.main()
