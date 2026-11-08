"""Check the UDP reader at the 4 KiB SRAM boundary and host ownership."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga/litex"))
from migen import ClockDomain, Memory, Module, Signal
from migen.sim import run_simulation
from liteeth.core.udp import LiteEthUDPUserPort
from lutcam.network import NetworkControl, DHCP_TX_BYTES
from lutcam.supervisor import SupervisorRam, RAM_BYTES


class Fixture(Module):
    def __init__(self):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.submodules.ram = ram = SupervisorRam()
        self.udp = LiteEthUDPUserPort(32)
        self.submodules.network = NetworkControl(self.udp, ram.dma, ram_bytes=RAM_BYTES,
            mac_address=0x024c55540001, preferred_ip=0xc0a80132)


def write(bus, offset, value, *, error=False):
    yield bus.adr.eq(offset//4)
    yield bus.dat_w.eq(value)
    yield bus.sel.eq(15)
    yield bus.we.eq(1)
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)
    yield
    for _ in range(20):
        if (yield bus.ack) or (yield bus.err):
            assert bool((yield bus.err)) == error
            break
        yield
    else:
        raise AssertionError("network aperture did not acknowledge")
    yield bus.cyc.eq(0)
    yield bus.stb.eq(0)
    yield
    yield


def simulate(dut, bench):
    fragment = dut.get_fragment()
    # Migen's Python memory model predates LiteX's write-only FIFO ports.
    for special in fragment.specials:
        if isinstance(special, Memory):
            for port in special.ports:
                if port.dat_r is None:
                    port.dat_r = Signal(special.width)
    run_simulation(fragment, bench)


class DhcpApertureTests(unittest.TestCase):
    def test_native_byte_writes_preserve_other_lanes_and_host_reads(self):
        dut = Fixture()

        def bench():
            ram = dut.ram
            def access(strobes, value):
                yield ram.address.eq(64)
                yield ram.wdata.eq(value)
                yield ram.strobes.eq(strobes)
                yield ram.valid.eq(1)
                yield
                for _ in range(12):
                    if (yield ram.ready):
                        result = (yield ram.rdata)
                        break
                    yield
                else:
                    self.fail("SRAM request did not complete")
                yield ram.valid.eq(0)
                yield
                yield
                return result

            yield from access(15, 0x11223344)
            yield from access(2, 0x0000aa00)
            yield from access(12, 0xbeef0000)
            self.assertEqual((yield from access(0, 0)), 0xbeefaa44)
            yield ram.host.address.eq(16)
            yield ram.host.request.eq(1)
            for _ in range(12):
                yield
                if (yield ram.host.ack):
                    self.assertEqual((yield ram.host.rdata), 0xbeefaa44)
                    break
            else:
                self.fail("host mailbox did not receive its SRAM word")

        simulate(dut, bench())

    def test_dma_finishes_at_last_sram_word_and_holds_under_backpressure(self):
        dut = Fixture()

        def bench():
            yield dut.network.granted.eq(1)
            yield dut.ram.load_busy.eq(1)
            yield dut.ram.load_write.eq(1)
            for word in range(DHCP_TX_BYTES//4):
                yield dut.ram.load_address.eq((RAM_BYTES-DHCP_TX_BYTES)//4+word)
                yield dut.ram.load_data.eq(0xdeadbeef if word==DHCP_TX_BYTES//4-1 else word)
                yield
            yield dut.ram.load_write.eq(0)
            yield dut.ram.load_busy.eq(0)
            yield from write(dut.network.bus, 0x24, RAM_BYTES-DHCP_TX_BYTES)
            yield from write(dut.network.bus, 0x28, DHCP_TX_BYTES)
            yield from write(dut.network.bus, 0x2c, 0xffffffff)
            yield from write(dut.network.bus, 0x30, 1)
            for _ in range(20):
                if (yield dut.udp.sink.valid):
                    break
                yield
            else:
                self.fail("DMA did not produce the final SRAM word")
            self.assertEqual((yield dut.udp.sink.length), DHCP_TX_BYTES)
            # Every word must hold during stalls, and the final read must
            # reach byte 4095 without wrapping to instruction RAM.
            for word in range(DHCP_TX_BYTES//4):
                for _ in range(20):
                    if (yield dut.udp.sink.valid): break
                    yield
                else: self.fail("DMA did not produce the next SRAM word")
                final = word == DHCP_TX_BYTES//4-1
                for _ in range(3):
                    self.assertEqual((yield dut.udp.sink.data), 0xdeadbeef if final else word)
                    self.assertEqual((yield dut.udp.sink.last), int(final))
                    self.assertEqual((yield dut.udp.sink.last_be), 8 if final else 0)
                    self.assertEqual((yield dut.udp.sink.src_port), 68)
                    self.assertEqual((yield dut.udp.sink.dst_port), 67)
                    self.assertEqual((yield dut.ram.dma.request), 0)
                    yield
                yield dut.udp.sink.ready.eq(1)
                yield
                yield dut.udp.sink.ready.eq(0)
                yield
            for _ in range(4): yield
            self.assertEqual((yield dut.udp.sink.valid), 0)
            self.assertEqual((yield dut.ram.dma.request), 0)
            # A completed transfer releases the staging registers.
            yield from write(dut.network.bus, 0x24, 0)

        simulate(dut, bench())

    def test_host_cannot_write_and_cpu_cannot_start_out_of_bounds_dma(self):
        dut = Fixture()

        def bench():
            yield from write(dut.network.bus, 8, 0xc0a80101, error=True)
            self.assertEqual((yield dut.network.ip_address), 0)
            yield dut.network.granted.eq(1)
            yield from write(dut.network.bus, 0x28, DHCP_TX_BYTES)
            for address in (RAM_BYTES-DHCP_TX_BYTES+4, RAM_BYTES, RAM_BYTES-1, 0xfffffffc):
                yield from write(dut.network.bus, 0x24, address, error=True)
                self.assertEqual((yield dut.ram.dma.request), 0)
            yield from write(dut.network.bus, 0x28, 0x10004, error=True)

        simulate(dut, bench())


if __name__ == "__main__":
    unittest.main()
