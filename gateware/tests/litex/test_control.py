"""Behavioral checks for the minimal camera controls and ACK-only host guard."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from migen import Cat, Constant, If, Module, Signal
from migen.sim import run_simulation

from lutcam.control import CameraI2C, HistogramReader, WishboneGuard


def transfer(bus, address, data=None, sel=0xf, *, idle=True, limit=100):
    """Hold a classic request through its response; optionally retain CYC/STB."""
    yield bus.adr.eq(address // 4)
    yield bus.dat_w.eq(0 if data is None else data)
    yield bus.we.eq(data is not None)
    yield bus.sel.eq(sel)
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)
    yield
    for _ in range(limit):
        if (yield bus.ack) or (yield bus.err):
            result = ((yield bus.dat_r), (yield bus.err))
            assert not ((yield bus.ack) and (yield bus.err))
            if idle:
                yield bus.cyc.eq(0)
                yield bus.stb.eq(0)
                yield
            return result
        yield
    raise AssertionError("Wishbone transfer failed to terminate")


class HistogramFixture(Module):
    """Streaming 512-bin histogram with MonoIsp's first/last word format."""

    def __init__(self):
        self.submodules.reader = reader = HistogramReader()
        self.enabled = Signal(reset=1)
        self.active = active = Signal()
        self.bin = index = Signal(9)
        self.pops = pops = Signal(12)
        self.starts = starts = Signal(4)
        self.comb += [
            reader.hist_dump_ready.eq(~active & self.enabled),
            reader.hist_out_valid.eq(active),
            reader.hist_out_data.eq(Cat((index + Constant(17, 20))[:20], index,
                                       Constant(0, 1), index == 511, index == 0)),
        ]
        self.sync += [
            If(reader.hist_dump_req & reader.hist_dump_ready,
                active.eq(1), index.eq(0), starts.eq(starts + 1),
            ),
            If(reader.hist_out_valid & reader.hist_out_ready,
                pops.eq(pops + 1),
                If(index == 511, active.eq(0)).Else(index.eq(index + 1)),
            ),
        ]


class ControlTests(unittest.TestCase):
    def test_i2c_release_pin_readback_and_byte_enable(self):
        dut = CameraI2C()

        def bench():
            self.assertEqual((yield dut.scl.oe), 0)
            self.assertEqual((yield dut.sda.oe), 0)
            self.assertEqual((yield dut.scl.o), 0)
            self.assertEqual((yield dut.sda.o), 0)
            yield dut.scl.i.eq(1)
            yield dut.sda.i.eq(1)
            for _ in range(4):
                yield
            self.assertEqual((yield from transfer(dut.bus, 4)), (3, 0))
            self.assertEqual((yield from transfer(dut.bus, 0, 3))[1], 0)
            self.assertEqual((yield dut.scl.oe), 1)
            self.assertEqual((yield dut.sda.oe), 1)
            # A non-selected low byte must not release either line.
            yield from transfer(dut.bus, 0, 0, sel=0xe)
            self.assertEqual((yield dut.control), 3)
            # Status observes the pin, not the software's requested level:
            # model a device holding SDA low while software releases it.
            yield from transfer(dut.bus, 0, 0)
            yield dut.sda.i.eq(0)
            for _ in range(4):
                yield
            self.assertEqual((yield from transfer(dut.bus, 4)), (1, 0))
            self.assertEqual((yield from transfer(dut.bus, 4, 0))[1], 1)
            self.assertEqual((yield from transfer(dut.bus, 8))[1], 1)

        run_simulation(dut, bench())

    def test_i2c_etherbone_style_continuous_strobe(self):
        dut = CameraI2C()

        def bench():
            for value in (1, 3, 2, 0, 0, 3):
                self.assertEqual((yield from transfer(dut.bus, 0, value, idle=False))[1], 0)
                self.assertEqual((yield dut.control), value)
            yield dut.bus.cyc.eq(0)
            yield dut.bus.stb.eq(0)
            yield

        run_simulation(dut, bench())

    def test_histogram_request_legality_and_exactly_once_reads(self):
        dut = HistogramFixture()
        reader = dut.reader

        def bench():
            yield dut.enabled.eq(0)
            yield
            self.assertEqual((yield from transfer(reader.bus, 0, 1))[1], 1)
            self.assertEqual((yield dut.starts), 0)
            self.assertEqual((yield from transfer(reader.bus, 8))[1], 1)
            yield dut.enabled.eq(1)
            yield
            self.assertEqual((yield from transfer(reader.bus, 0, 1)), (0, 0))
            self.assertEqual((yield dut.starts), 1)
            self.assertEqual((yield from transfer(reader.bus, 0, 1))[1], 1)
            self.assertEqual((yield dut.starts), 1)
            self.assertEqual((yield from transfer(reader.bus, 4)), (6, 0))
            self.assertEqual((yield dut.pops), 0)
            # Keep STB high between reads to exercise the same transfer
            # retirement behavior used by continuously asserted bus hosts.
            for index in range(512):
                value, error = yield from transfer(reader.bus, 8, idle=False)
                expected = index + 17 | index << 20
                expected |= (index == 0) << 31
                expected |= (index == 511) << 30
                self.assertEqual((value, error), (expected, 0))
                self.assertEqual((yield dut.pops), index + 1)
            yield reader.bus.cyc.eq(0)
            yield reader.bus.stb.eq(0)
            yield
            self.assertEqual((yield from transfer(reader.bus, 12)), (512, 0))
            self.assertEqual((yield from transfer(reader.bus, 4)), (1, 0))
            self.assertEqual((yield from transfer(reader.bus, 8))[1], 1)
            self.assertEqual((yield dut.pops), 512)
            self.assertEqual((yield from transfer(reader.bus, 0, 1))[1], 0)
            self.assertEqual((yield dut.starts), 2)

        run_simulation(dut, bench())

    def test_guard_success_error_timeout_and_recovery(self):
        dut = Module()
        dut.submodules.guard = guard = WishboneGuard(timeout_cycles=5)
        slave = guard.master
        writes = Signal(8)
        # Address 0 succeeds, address 4 errors, all others never respond.
        dut.sync += [
            slave.ack.eq(0), slave.err.eq(0),
            If(slave.cyc & slave.stb & ~slave.ack & ~slave.err,
                If(slave.adr == 0,
                    slave.ack.eq(1), slave.dat_r.eq(0x12345678),
                    If(slave.we, writes.eq(writes + 1)),
                ).Elif(slave.adr == 1,
                    slave.err.eq(1),
                    If(slave.we, writes.eq(writes + 1)),
                ),
            ),
        ]

        def bench():
            self.assertEqual((yield from transfer(guard.bus, 0)), (0x12345678, 0))
            self.assertEqual((yield guard.fault), 0)
            # Consecutive writes with no upstream STB drop must each reach
            # the slave once, including identical writes to the same address.
            for _ in range(3):
                self.assertEqual((yield from transfer(guard.bus, 0, 7, idle=False))[1], 0)
            self.assertEqual((yield writes), 3)
            self.assertEqual((yield from transfer(guard.bus, 4, 9)), (0xffffffff, 0))
            self.assertEqual((yield writes), 4)
            self.assertEqual((yield guard.fault), 1)
            self.assertEqual((yield guard.fault_address), 4)
            self.assertEqual((yield guard.fault_status), 5)
            self.assertEqual((yield guard.fault_count), 1)
            self.assertEqual((yield from transfer(guard.bus, 8)), (0xffffffff, 0))
            self.assertEqual((yield guard.fault_address), 8)
            self.assertEqual((yield guard.fault_status), 2)
            self.assertEqual((yield guard.fault_count), 2)
            self.assertEqual((yield slave.cyc), 0)
            self.assertEqual((yield from transfer(guard.bus, 0)), (0x12345678, 0))
            yield guard.clear_fault.eq(1)
            yield
            yield guard.clear_fault.eq(0)
            yield
            self.assertEqual((yield guard.fault), 0)
            self.assertEqual((yield guard.fault_count), 0)

        run_simulation(dut, bench())

    def test_guard_zero_wait_responses_and_minimum_timeout(self):
        dut = Module()
        dut.submodules.guard = guard = WishboneGuard(timeout_cycles=1)
        slave = guard.master
        writes, last_data, last_sel = Signal(8), Signal(32), Signal(4)
        dut.comb += [
            slave.ack.eq(slave.cyc & slave.stb & (slave.adr < 8)),
            slave.err.eq(slave.cyc & slave.stb & (slave.adr == 8)),
            slave.dat_r.eq(slave.adr ^ 0x1234abcd),
        ]
        dut.sync += If(slave.cyc & slave.stb & slave.ack & slave.we,
            writes.eq(writes + 1), last_data.eq(slave.dat_w), last_sel.eq(slave.sel))

        def bench():
            for address, data, sel in ((4, 0x12345678, 1), (12, 0x89abcdef, 4),
                                       (16, 0xfeedbeef, 15)):
                yield from transfer(guard.bus, address, data, sel, idle=False)
            yield guard.bus.cyc.eq(0)
            yield guard.bus.stb.eq(0)
            yield
            yield
            self.assertEqual((yield writes), 3)
            self.assertEqual((yield last_data), 0xfeedbeef)
            self.assertEqual((yield last_sel), 15)
            self.assertEqual((yield from transfer(guard.bus, 20)), (0x1234abcd ^ 5, 0))
            self.assertEqual((yield from transfer(guard.bus, 32)), (0xffffffff, 0))
            self.assertEqual((yield guard.fault_address), 32)
            self.assertEqual((yield guard.fault_status), 1)
            self.assertEqual((yield from transfer(guard.bus, 36)), (0xffffffff, 0))
            self.assertEqual((yield guard.fault_status), 2)
            self.assertEqual((yield guard.fault_count), 2)
            self.assertEqual((yield from transfer(guard.bus, 0)), (0x1234abcd, 0))

        run_simulation(dut, bench())

    def test_guard_aborted_request_cannot_ack_a_later_one(self):
        guard = WishboneGuard(timeout_cycles=8)

        def bench():
            yield guard.bus.cyc.eq(1)
            yield guard.bus.stb.eq(1)
            for _ in range(3):
                yield
            yield guard.bus.cyc.eq(0)
            yield guard.bus.stb.eq(0)
            yield
            yield
            self.assertEqual((yield guard.master.cyc), 0)
            self.assertEqual((yield guard.bus.ack), 0)
            self.assertEqual((yield guard.fault), 0)
            self.assertEqual((yield from transfer(guard.bus, 0x1234)), (0xffffffff, 0))
            self.assertEqual((yield guard.fault_address), 0x1234)

        run_simulation(guard, bench())


if __name__ == "__main__":
    unittest.main()
