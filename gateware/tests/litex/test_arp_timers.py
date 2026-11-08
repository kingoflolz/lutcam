"""Verify timer defaults and enable/reset timing against the countdown reference."""
import pytest
from migen import ClockDomain, Module, ResetInserter
from litex.gen.genlib.misc import WaitTimer
from migen.sim import run_simulation
from liteeth.core.arp import LiteEthARPTable
from liteeth.core.efinix import DspWaitTimer
from efinix_test_models import SPECIAL_OVERRIDES


@pytest.mark.parametrize("entries", [1, 2, 4])
def test_table_timer_defaults(entries):
    table = LiteEthARPTable(120_000_000, entries=entries)
    timer_type = DspWaitTimer if entries <= 2 else WaitTimer
    assert isinstance(table.request_timer, timer_type)
    assert isinstance(table.cache.clear_timer, timer_type)


@pytest.mark.parametrize("cycles", [1, 4, 64, 1200])
def test_wait_done_cycle_equivalence(cycles):
    dut = Module()
    dut.clock_domains.cd_sys = ClockDomain("sys")
    dut.submodules.reference = reference = ResetInserter()(WaitTimer(cycles))
    dut.submodules.native = native = ResetInserter()(DspWaitTimer(cycles))
    # Native reset is driven by its clock domain; explicit wait withdrawal
    # also resets the accumulator, including when done is already asserted.
    def bench():
        for wait in [0] * 3 + [1] * (cycles + 4) + [0] * 2 + [1] * (cycles + 4):
            yield reference.wait.eq(wait)
            yield native.wait.eq(wait)
            yield
            assert (yield reference.done) == (yield native.done)
    run_simulation(dut, bench(), special_overrides=SPECIAL_OVERRIDES)
