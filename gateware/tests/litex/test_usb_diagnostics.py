"""Read-only USB electrical monitoring with independent USB/system clocks."""

from types import SimpleNamespace
import unittest

from migen import ClockDomain, Module, Signal
from migen.sim import run_simulation

from fpga.litex.lutcam.usb_diagnostics import (
    USB_LINK_BASE, USB_LINK_HEARTBEAT_DIVISOR, USB_LINK_LIVE_BITS,
    USB_LINK_MAGIC, USB_LINK_REGISTERS, USB_LINK_SEEN_BITS,
    USBLinkDiagnostics,
)


CLOCKS = {"sys": 10, "usb_12": 70, "usb_48": 22}


def cycles(count):
    for _ in range(count):
        yield


def transfer(bus, address, value=None, *, sel=15):
    yield bus.adr.eq(address // 4)
    yield bus.we.eq(value is not None)
    yield bus.dat_w.eq(0 if value is None else value)
    yield bus.sel.eq(sel)
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)
    yield
    for _ in range(20):
        ack, err = (yield bus.ack), (yield bus.err)
        assert not (ack and err), "Wishbone ACK and ERR asserted together"
        if ack or err:
            result = (yield bus.dat_r), err
            yield bus.cyc.eq(0)
            yield bus.stb.eq(0)
            yield
            yield
            return result
        yield
    raise AssertionError("USB diagnostics transaction did not terminate")


class Fixture(Module):
    def __init__(self, **kwargs):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.clock_domains.cd_usb_12 = ClockDomain("usb_12")
        self.clock_domains.cd_usb_48 = ClockDomain("usb_48")
        self.inputs = {name: Signal(name=name) for name in USB_LINK_LIVE_BITS}
        self.inputs["usb_12_reset"] = self.cd_usb_12.rst
        self.inputs["usb_48_reset"] = self.cd_usb_48.rst
        self.submodules.link = USBLinkDiagnostics(**self.inputs, **kwargs)

    def attach(self):
        for name in ("connect", "pullup", "pll_locked", "vbus_valid", "dp_i"):
            yield self.inputs[name].eq(1)
        yield from cycles(7)


class USBLinkDiagnosticsTests(unittest.TestCase):
    def test_register_map_metadata_and_parameter_validation(self):
        dut = Fixture()
        region = dut.link.regions["usb_link"]
        self.assertEqual(region.origin, USB_LINK_BASE)
        self.assertEqual(region.busword, 32)
        self.assertEqual([(csr.name, csr.origin - USB_LINK_BASE)
                          for csr in region.obj], list(USB_LINK_REGISTERS))
        self.assertTrue(all(csr.read_only and csr.size == 32 for csr in region.obj))
        for options in ({"sys_clk_freq": 0}, {"sys_clk_freq": 120e6},
                        {"se0_threshold": 0}, {"counter_bits": 1},
                        {"counter_bits": 33}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                Fixture(**options)
        inputs = dict(dut.inputs, dp_i=Signal(2))
        with self.assertRaises(ValueError):
            USBLinkDiagnostics(**inputs)

    def test_status_cdc_records_live_and_sticky_observations(self):
        dut = Fixture()
        values = {name: int(index % 2 == 0) for index, name in enumerate(USB_LINK_LIVE_BITS)}

        def bench():
            self.assertEqual((yield dut.link.live), 0)
            for name, value in values.items():
                yield dut.inputs[name].eq(value)
            # MultiReg must not expose asynchronous changes immediately.
            self.assertEqual((yield dut.link.live), 0)
            yield from cycles(7)
            expected = sum(value << index for index, value in enumerate(values.values()))
            self.assertEqual((yield dut.link.live), expected)
            seen = yield dut.link.seen
            for name in ("tx_drive", "pullup", "usb_12_reset", "pll_locked"):
                self.assertTrue(seen & (1 << USB_LINK_SEEN_BITS.index(name)), name)
            for signal in dut.inputs.values():
                yield signal.eq(0)
            yield from cycles(7)
            self.assertEqual((yield dut.link.live), 0)
            self.assertEqual((yield dut.link.seen), seen)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_host_reset_evidence_excludes_eop_local_tx_and_detach(self):
        dut = Fixture(se0_threshold=8)

        def bench():
            yield from dut.attach()
            # A short undriven SE0 (EOP) is not host reset evidence.
            yield dut.inputs["dp_i"].eq(0)
            yield from cycles(2)
            yield dut.inputs["dp_i"].eq(1)
            yield from cycles(7)
            self.assertEqual((yield dut.link.se0_events), 0)
            self.assertGreaterEqual((yield dut.link.rx_edges), 2)
            # A long low interval while the FPGA drives either pin is local TX.
            yield dut.inputs["dp_oe"].eq(1)
            yield dut.inputs["dp_i"].eq(0)
            yield from cycles(30)
            self.assertEqual((yield dut.link.se0_events), 0)
            self.assertEqual((yield dut.link.tx_bursts), 1)
            yield dut.inputs["dp_i"].eq(1)
            yield dut.inputs["dp_oe"].eq(0)
            yield from cycles(7)
            # Exactly one event per long continuous host-driven SE0.
            yield dut.inputs["dp_i"].eq(0)
            yield from cycles(30)
            self.assertEqual((yield dut.link.se0_events), 1)
            self.assertEqual((yield dut.link.se0_cycles), 8)
            yield from cycles(20)
            self.assertEqual((yield dut.link.se0_events), 1)
            yield dut.inputs["dp_i"].eq(1)
            yield from cycles(7)
            self.assertEqual((yield dut.link.se0_cycles), 0)
            yield dut.inputs["dp_i"].eq(0)
            yield from cycles(20)
            self.assertEqual((yield dut.link.se0_events), 2)
            self.assertTrue((yield dut.link.seen) & (1 << USB_LINK_SEEN_BITS.index("long_se0")))
            # A disconnected bus must not increase any event counter.
            yield dut.inputs["connect"].eq(0)
            yield from cycles(7)
            before = ((yield dut.link.rx_edges), (yield dut.link.tx_bursts),
                      (yield dut.link.se0_events))
            for value in (1, 0, 1, 0):
                yield dut.inputs["dp_i"].eq(value)
                yield dut.inputs["dp_oe"].eq(value)
                yield from cycles(20)
            self.assertEqual(((yield dut.link.rx_edges), (yield dut.link.tx_bursts),
                              (yield dut.link.se0_events)), before)
            self.assertEqual((yield dut.link.se0_cycles), 0)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_power_reset_and_pullup_inhibit_host_event_detection(self):
        for field, value in (("vbus_valid", 0), ("pll_locked", 0), ("pullup", 0),
                             ("usb_12_reset", 1), ("usb_48_reset", 1)):
            with self.subTest(inhibit=field):
                dut = Fixture(se0_threshold=4)

                def bench():
                    yield from dut.attach()
                    yield dut.inputs[field].eq(value)
                    yield from cycles(7)
                    yield dut.inputs["dp_i"].eq(0)
                    yield from cycles(20)
                    self.assertEqual((yield dut.link.se0_events), 0)
                    self.assertEqual((yield dut.link.rx_edges), 0)

                run_simulation(dut, bench(), clocks=CLOCKS)

    def test_event_counters_saturate_instead_of_wrapping(self):
        dut = Fixture(se0_threshold=3, counter_bits=2)

        def bench():
            yield from dut.attach()
            for _ in range(6):
                yield dut.inputs["dp_oe"].eq(1)
                yield from cycles(7)
                yield dut.inputs["dp_oe"].eq(0)
                yield from cycles(7)
                yield dut.inputs["dp_i"].eq(0)
                yield from cycles(10)
                yield dut.inputs["dp_i"].eq(1)
                yield from cycles(7)
            for counter in (dut.link.rx_edges, dut.link.tx_bursts, dut.link.se0_events):
                self.assertEqual((yield counter), 3)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_bus_is_read_only_and_reads_do_not_clear_history(self):
        dut = Fixture()

        def bench():
            yield from dut.attach()
            yield dut.inputs["dm_oe"].eq(1)
            yield from cycles(7)
            yield dut.inputs["dm_oe"].eq(0)
            yield from cycles(7)
            before = (yield dut.link.seen), (yield dut.link.tx_bursts)
            expected = {0x00: USB_LINK_MAGIC, 0x04: 0,
                        0x20: 600, 0x24: 120_000_000, 0x28: 16,
                        0x34: USB_LINK_HEARTBEAT_DIVISOR}
            for offset, value in expected.items():
                self.assertEqual((yield from transfer(dut.link.bus, USB_LINK_BASE + offset)),
                                 (value, 0))
            for _, offset in USB_LINK_REGISTERS:
                _, err = yield from transfer(dut.link.bus, offset)
                self.assertEqual(err, 0)
                for sel in (0, 1, 15):
                    _, err = yield from transfer(dut.link.bus, offset, 0xffffffff, sel=sel)
                    self.assertEqual(err, 1)
            for offset in (0x38, 0x100, 0xffc):
                self.assertEqual((yield from transfer(dut.link.bus, offset))[1], 1)
                self.assertEqual((yield from transfer(dut.link.bus, offset, 1))[1], 1)
            self.assertEqual(((yield dut.link.seen), (yield dut.link.tx_bursts)), before)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_clock_heartbeats_run_detached_and_report_a_held_domain_reset(self):
        dut = Fixture()

        def bench():
            yield dut.cd_usb_12.rst.eq(1)
            yield from cycles(1200)
            self.assertEqual((yield dut.link.usb_12_heartbeat), 0)
            self.assertGreater((yield dut.link.usb_48_heartbeat), 0)
            self.assertEqual((yield dut.link.seen) & 3, 0)
            self.assertEqual((yield dut.link.se0_events), 0)
            yield dut.cd_usb_12.rst.eq(0)
            yield from cycles(2000)
            self.assertGreater((yield dut.link.usb_12_heartbeat), 0)
            yield dut.cd_usb_48.rst.eq(1)
            yield from cycles(20)
            before12, before48 = (yield dut.link.usb_12_heartbeat), (yield dut.link.usb_48_heartbeat)
            yield from cycles(2000)
            self.assertGreater((yield dut.link.usb_12_heartbeat), before12)
            self.assertEqual((yield dut.link.usb_48_heartbeat), before48)
            value, err = yield from transfer(dut.link.bus, 0x30)
            self.assertEqual((value, err), (before48, 0))

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_system_reset_clears_event_history(self):
        dut = Fixture(se0_threshold=4)

        def bench():
            yield from dut.attach()
            yield dut.inputs["dp_i"].eq(0)
            yield from cycles(20)
            self.assertGreater((yield dut.link.seen), 0)
            self.assertEqual((yield dut.link.se0_events), 1)
            for signal in dut.inputs.values():
                yield signal.eq(0)
            yield from cycles(7)
            yield dut.cd_sys.rst.eq(1)
            yield from cycles(4)
            yield dut.cd_sys.rst.eq(0)
            yield from cycles(4)
            self.assertEqual((yield dut.link.seen), 0)
            for counter in (dut.link.rx_edges, dut.link.tx_bursts, dut.link.se0_events):
                self.assertEqual((yield counter), 0)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_from_luna_observes_existing_converter_bindings_without_mutation(self):
        from litex_luna import LunaCDCACM

        usb = LunaCDCACM(None, SimpleNamespace(d_p=Signal(), d_n=Signal(), pullup=Signal()))
        usb.usb._MustUse__silence = True
        before = dict(usb.core_params)
        monitor = USBLinkDiagnostics.from_luna(usb, pll_locked=Signal(), vbus_valid=Signal())
        self.assertEqual(set(usb.core_params), set(before))
        for key, value in before.items():
            self.assertIs(usb.core_params[key], value, key)
        self.assertEqual(monitor.regions["usb_link"].origin, USB_LINK_BASE)
        self.assertFalse(usb.finalized)


if __name__ == "__main__":
    unittest.main()
