"""Exercise capture ARP retention against the actual LiteEth cache."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from migen import ClockDomain, Module
from migen.sim import run_simulation
from efinix_test_models import SPECIAL_OVERRIDES
from liteeth.core.arp import LiteEthARPCache

from lutcam.soc import BoardControl


CLOCK_FREQUENCY = 64
CAPTURE_IP = 0xc0a80102
CAPTURE_MAC = 0x021122334455


def idle(cycles):
    for _ in range(cycles):
        yield


def seed(cache, ip, mac):
    yield cache.update.valid.eq(1)
    yield cache.update.ip_address.eq(ip)
    yield cache.update.mac_address.eq(mac)
    yield
    for _ in range(16):
        if (yield cache.update.ready):
            yield cache.update.valid.eq(0)
            yield
            return
        yield
    raise AssertionError("ARP cache did not accept an update")


def lookup(cache, ip):
    yield cache.request.valid.eq(1)
    yield cache.request.ip_address.eq(ip)
    yield
    for _ in range(16):
        if (yield cache.response.valid):
            result = (yield cache.response.error), (yield cache.response.mac_address)
            yield cache.request.valid.eq(0)
            yield
            return result
        yield
    raise AssertionError("ARP cache did not complete a lookup")


class NetworkFixture(Module):
    def __init__(self):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.submodules.board = BoardControl()
        self.submodules.cache = LiteEthARPCache(entries=2, clk_freq=CLOCK_FREQUENCY)
        # This is the board's actual capture arming signal and SoC wiring.
        self.comb += self.cache.clear_enable.eq(~self.board.network_ready.storage)


class NetworkControlTests(unittest.TestCase):
    def test_firmware_revocation_wins_over_simultaneous_host_arm(self):
        dut = NetworkFixture()
        dut.board.network_ready.finalize(32, "big")
        dut.submodules.ready_storage = dut.board.network_ready # Normally owned by CSRBank.
        csr = dut.board.network_ready.simple_csrs[0]

        def bench():
            yield csr.wr_data.eq(1)
            yield csr.wr_stb.eq(1)
            yield from idle(2)
            self.assertEqual((yield dut.board.network_ready.storage), 1)
            yield dut.board.revoke_capture.eq(1)
            yield from idle(2)
            self.assertEqual((yield dut.board.network_ready.storage), 0)
            yield csr.wr_stb.eq(0)
            yield dut.board.revoke_capture.eq(0)
            yield from idle(2)
            self.assertEqual((yield dut.board.network_ready.storage), 0)

        run_simulation(dut, bench(), special_overrides=SPECIAL_OVERRIDES)

    def test_capture_retains_resolved_peer_across_expiry_and_disarm_restores_clear(self):
        dut = NetworkFixture()

        def bench():
            yield from idle(4)  # Cache performs its initial validity clear.
            self.assertEqual((yield dut.cache.clear_enable), 1)
            yield from seed(dut.cache, CAPTURE_IP, CAPTURE_MAC)
            self.assertEqual((yield from lookup(dut.cache, CAPTURE_IP)), (0, CAPTURE_MAC))
            yield from seed(dut.cache, CAPTURE_IP + 1, CAPTURE_MAC + 1)
            self.assertEqual((yield from lookup(dut.cache, CAPTURE_IP + 1)),
                             (0, CAPTURE_MAC + 1))
            yield dut.board.network_ready.storage.eq(1)
            for _ in range(3):
                # Long completely idle intervals matter: lookups themselves
                # restart LiteEth's idle-only expiry countdown.
                yield from idle(CLOCK_FREQUENCY * 2)
                self.assertEqual((yield dut.cache.clear_enable), 0)
                self.assertEqual((yield from lookup(dut.cache, CAPTURE_IP)), (0, CAPTURE_MAC))
                self.assertEqual((yield from lookup(dut.cache, CAPTURE_IP + 1)),
                                 (0, CAPTURE_MAC + 1))
            yield dut.board.network_ready.storage.eq(0)
            yield from idle(CLOCK_FREQUENCY * 2)
            self.assertEqual((yield dut.cache.clear_enable), 1)
            self.assertEqual((yield from lookup(dut.cache, CAPTURE_IP))[0], 1)
            self.assertEqual((yield from lookup(dut.cache, CAPTURE_IP + 1))[0], 1)

        run_simulation(dut, bench(), special_overrides=SPECIAL_OVERRIDES)


    def test_disabling_expiry_does_not_disable_round_robin_replacement(self):
        dut = NetworkFixture()

        def bench():
            yield dut.board.network_ready.storage.eq(1)
            yield from idle(4)
            yield from seed(dut.cache, CAPTURE_IP, CAPTURE_MAC)
            self.assertEqual((yield from lookup(dut.cache, CAPTURE_IP)), (0, CAPTURE_MAC))
            yield from seed(dut.cache, CAPTURE_IP + 1, CAPTURE_MAC + 1)
            self.assertEqual((yield from lookup(dut.cache, CAPTURE_IP))[0], 0)
            # A resolved third local destination evicts slot zero despite
            # capture arming. Network operating limits must account for this.
            yield from seed(dut.cache, CAPTURE_IP + 2, CAPTURE_MAC + 2)
            self.assertEqual((yield from lookup(dut.cache, CAPTURE_IP))[0], 1)
            self.assertEqual((yield from lookup(dut.cache, CAPTURE_IP + 2)),
                             (0, CAPTURE_MAC + 2))

        run_simulation(dut, bench(), special_overrides=SPECIAL_OVERRIDES)


if __name__ == "__main__":
    unittest.main()
