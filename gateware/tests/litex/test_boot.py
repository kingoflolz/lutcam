"""Simulate reboot admission, clock-domain sequencing, and golden flash ownership."""

from types import SimpleNamespace
import inspect
import unittest

from migen import Module, Signal
from migen.sim import passive, run_simulation
from litex.soc.interconnect import wishbone

from fpga.litex.lutcam.boot import (
    BOOT_BASE, BOOT_MAGIC, BOOT_REGISTERS, BootControl, FlashAccessMux,
)
from fpga.litex.lutcam.image_identity import ImageIdentity, identity_words


CLOCKS = {"sys": 10, "cam_ref": 48}
BUILD_ID = "0123456789abcdeffedcba9876543210"
TARGET, COMMAND, ERROR, PENDING_TARGET = 0x20, 0x24, 0x28, 0x2C


def transfer(bus, address, value=None, *, sel=15, idle=True):
    """Bounded classic transfer; a held transaction is released by its caller."""
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
            if idle:
                yield bus.cyc.eq(0)
                yield bus.stb.eq(0)
                yield
                yield
            return result
        yield
    raise AssertionError("register transaction did not terminate")


class BootControlTests(unittest.TestCase):
    def read(self, bus, address):
        value, err = yield from transfer(bus, address)
        self.assertEqual(err, 0, f"read {address:#x} failed")
        return value

    def write(self, bus, address, value, *, error=0, **kwargs):
        _, err = yield from transfer(bus, address, value, **kwargs)
        self.assertEqual(err, error, f"write {address:#x} -> {value:#x}")

    def test_metadata_matches_host_register_map_and_default_delay(self):
        dut = BootControl()
        region = dut.regions["boot"]
        self.assertEqual(region.origin, BOOT_BASE)
        self.assertEqual(region.busword, 32)
        self.assertEqual([(csr.name, csr.origin - BOOT_BASE, csr.read_only)
                          for csr in region.obj], list(BOOT_REGISTERS))
        self.assertTrue(all(csr.size == 32 for csr in region.obj))
        self.assertEqual(inspect.signature(BootControl).parameters["delay_cycles"].default,
                         120_000_000 // 10)
        for kwargs in ({"image_slot": -1}, {"image_slot": 4},
                       {"build_id": "1234"}, {"build_id": "z" * 32},
                       {"delay_cycles": 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                BootControl(**kwargs)

    def test_static_identity_and_status_registers(self):
        dut = BootControl(image_slot=3, build_id=BUILD_ID)

        def bench():
            yield dut.golden_status.eq(0x123403)
            yield dut.pins.ERROR.eq(1)
            for _ in range(5):
                yield
            expected = {0: BOOT_MAGIC, 4: 0, 8: 3, 0x0C: 14,
                        0x10: 0x01234567, 0x14: 0x89ABCDEF,
                        0x18: 0xFEDCBA98, 0x1C: 0x76543210,
                        TARGET: 0, COMMAND: 0, ERROR: 0,
                        PENDING_TARGET: 0, 0x30: 0x123403}
            for address, value in expected.items():
                self.assertEqual((yield from self.read(dut.bus, address)), value)
            for _, address, read_only in BOOT_REGISTERS:
                if read_only:
                    yield from self.write(dut.bus, address, 0xFFFFFFFF, error=1)
            self.assertEqual((yield from transfer(dut.bus, 0x34))[1], 1)
            yield from self.write(dut.bus, 0x34, 1, error=1)
            self.assertEqual((yield dut.pending), 0)
            self.assertEqual((yield dut.pins.CONFIG), 0)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_dynamic_identity_is_read_from_patched_rom(self):
        for slot_name, slot_value in (("A", 1), ("B", 2)):
            with self.subTest(slot=slot_name):
                dut = Module()
                dut.submodules.identity = identity = ImageIdentity(
                    build_id="0" * 32, init_words=identity_words(BUILD_ID, slot_name))
                dut.submodules.boot = boot = BootControl(identity=identity, delay_cycles=9)

                def bench():
                    for _ in range(40):
                        if (yield identity.ready):
                            break
                        yield
                    self.assertEqual((yield identity.valid), 1)
                    self.assertEqual((yield from self.read(boot.bus, 8)), slot_value)
                    words = []
                    for offset in (0x10, 0x14, 0x18, 0x1C):
                        words.append((yield from self.read(boot.bus, offset)))
                    self.assertEqual(b"".join(word.to_bytes(4, "big") for word in words).hex(), BUILD_ID)
                    self.assertEqual((yield boot.auto_ready), 1)

                run_simulation(dut, bench(), clocks=CLOCKS)

    def test_invalid_dynamic_identity_blocks_host_and_automatic_reboot(self):
        identity = SimpleNamespace(valid=Signal(), image_slot=Signal(2),
                                   build_id_words=[Signal(32) for _ in range(4)])
        dut = BootControl(identity=identity, delay_cycles=2)

        def bench():
            yield dut.auto_slot.eq(2)
            yield dut.auto_request.eq(1)
            yield from self.write(dut.bus, TARGET, 2)
            yield from self.write(dut.bus, COMMAND, 6, error=1)
            self.assertEqual((yield from self.read(dut.bus, ERROR)), 2)
            for _ in range(20):
                self.assertEqual((yield dut.auto_ready), 0)
                self.assertEqual((yield dut.pending), 0)
                self.assertEqual((yield dut.pins.CONFIG), 0)
                yield
            yield dut.auto_request.eq(0)
            yield identity.valid.eq(1)
            yield identity.image_slot.eq(1)
            yield
            yield from self.write(dut.bus, COMMAND, 6)
            self.assertEqual((yield dut.pending), 1)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_rejects_invalid_target_partial_writes_busy_and_stale_ids(self):
        dut = BootControl(delay_cycles=1000)

        def bench():
            for sel in (0, 1, 5, 14):
                yield from self.write(dut.bus, TARGET, 2, sel=sel, error=1)
                yield from self.write(dut.bus, COMMAND, 7, sel=sel, error=1)
            self.assertEqual((yield from self.read(dut.bus, TARGET)), 0)
            yield from self.write(dut.bus, COMMAND, 0, error=1)
            self.assertEqual((yield from self.read(dut.bus, ERROR)), 3)
            for target in (4, 0x100, 0xFFFFFFFF):
                yield from self.write(dut.bus, TARGET, target, error=1)
                self.assertEqual((yield from self.read(dut.bus, ERROR)), 1)
                self.assertEqual((yield from self.read(dut.bus, TARGET)), 0)
                self.assertEqual((yield from self.read(dut.bus, COMMAND)), 0)
            yield from self.write(dut.bus, TARGET, 3)
            yield dut.flash_busy.eq(1)
            yield from self.write(dut.bus, COMMAND, 7, error=1)
            self.assertEqual((yield from self.read(dut.bus, ERROR)), 2)
            self.assertEqual((yield dut.pending), 0)
            yield dut.flash_busy.eq(0)
            yield from self.write(dut.bus, COMMAND, 7)  # Sequence 1, selector 3.
            self.assertEqual((yield dut.pending), 1)
            self.assertEqual((yield from self.read(dut.bus, PENDING_TARGET)), 3)
            self.assertEqual((yield from self.read(dut.bus, COMMAND)), 7)
            yield from self.write(dut.bus, COMMAND, 7)  # Lost UDP ACK replay.
            yield from self.write(dut.bus, TARGET, 1, error=1)
            self.assertEqual((yield from self.read(dut.bus, TARGET)), 3)
            for command in (0, 4, 6, 8):
                yield from self.write(dut.bus, COMMAND, command, error=1)
                self.assertEqual((yield from self.read(dut.bus, ERROR)), 3)
                self.assertEqual((yield from self.read(dut.bus, COMMAND)), 7)
                self.assertEqual((yield from self.read(dut.bus, PENDING_TARGET)), 3)
            yield from self.write(dut.bus, COMMAND, 7)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_atomic_command_selector_survives_reordered_target_write(self):
        for slot in range(4):
            with self.subTest(slot=slot):
                dut = BootControl(delay_cycles=5)

                def bench():
                    yield from self.write(dut.bus, TARGET, slot)
                    # A delayed older datagram reaches the staging register
                    # after our desired selector. The commit word carries its
                    # own selector, so this cannot redirect configuration.
                    old_target = (slot + 1) % 4
                    yield from self.write(dut.bus, TARGET, old_target)
                    command = 0x18 | slot
                    yield from self.write(dut.bus, COMMAND, command)
                    self.assertEqual((yield from self.read(dut.bus, TARGET)), old_target)
                    self.assertEqual((yield from self.read(dut.bus, COMMAND)), command)
                    self.assertEqual((yield from self.read(dut.bus, PENDING_TARGET)), slot)
                    for _ in range(100):
                        if (yield dut.pins.CONFIG):
                            break
                        yield
                    self.assertEqual((yield dut.pins.CONFIG), 1)
                    self.assertEqual((yield dut.pins.CBSEL), slot)

                run_simulation(dut, bench(), clocks=CLOCKS)

    def test_high_target_bits_reject_without_changing_valid_target(self):
        dut = BootControl()

        def bench():
            yield from self.write(dut.bus, TARGET, 2)
            for target in (4, 0x102, 0x80000002, 0xFFFFFFFF):
                yield from self.write(dut.bus, TARGET, target, error=1)
                self.assertEqual((yield from self.read(dut.bus, TARGET)), 2)
                self.assertEqual((yield from self.read(dut.bus, ERROR)), 1)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_exact_replay_keeps_sticky_error_despite_all_admission_blockers(self):
        identity = SimpleNamespace(valid=Signal(reset=1), image_slot=Signal(2),
                                   build_id_words=[Signal(32) for _ in range(4)])
        dut = BootControl(identity=identity, delay_cycles=1000)

        def bench():
            yield from self.write(dut.bus, COMMAND, 0x80000002)
            yield from self.write(dut.bus, COMMAND, 0x80000001, error=1)
            self.assertEqual((yield from self.read(dut.bus, ERROR)), 3)
            yield dut.flash_busy.eq(1)
            yield identity.valid.eq(0)
            yield dut.auto_request.eq(1)
            yield dut.auto_slot.eq(3)
            yield from self.write(dut.bus, COMMAND, 0x80000002)
            self.assertEqual((yield from self.read(dut.bus, ERROR)), 3)
            self.assertEqual((yield from self.read(dut.bus, COMMAND)), 0x80000002)
            self.assertEqual((yield from self.read(dut.bus, PENDING_TARGET)), 2)
            self.assertEqual((yield dut.pending), 1)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_automatic_boot_with_zero_accepted_id_rejects_all_host_commands(self):
        dut = BootControl(delay_cycles=1000)

        def bench():
            yield dut.auto_slot.eq(2)
            yield dut.auto_request.eq(1)
            yield
            yield
            yield dut.auto_request.eq(0)
            self.assertEqual((yield dut.pending), 1)
            for command in (0, 1, 2, 0xFFFFFFFF):
                yield from self.write(dut.bus, COMMAND, command, error=1)
                self.assertEqual((yield from self.read(dut.bus, COMMAND)), 0)
                self.assertEqual((yield from self.read(dut.bus, ERROR)), 3)
                self.assertEqual((yield from self.read(dut.bus, PENDING_TARGET)), 2)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_command_counter_wrap_and_older_selectors_are_rejected(self):
        dut = BootControl(delay_cycles=1000)

        def bench():
            yield from self.write(dut.bus, COMMAND, 0xFFFFFFFF)
            self.assertEqual((yield from self.read(dut.bus, COMMAND)), 0xFFFFFFFF)
            self.assertEqual((yield from self.read(dut.bus, PENDING_TARGET)), 3)
            # Counter wrap cannot turn a fresh low token, regardless of its
            # selector, into another accepted request within this boot.
            for command in (0, 1, 2, 3, 4, 5, 6, 7, 0xFFFFFFFC, 0xFFFFFFFE):
                yield from self.write(dut.bus, COMMAND, command, error=1)
                self.assertEqual((yield from self.read(dut.bus, ERROR)), 3)
                self.assertEqual((yield from self.read(dut.bus, COMMAND)), 0xFFFFFFFF)
                self.assertEqual((yield from self.read(dut.bus, PENDING_TARGET)), 3)
            yield from self.write(dut.bus, COMMAND, 0xFFFFFFFF)
            self.assertEqual((yield dut.pending), 1)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_held_classic_request_cannot_recommit_or_mutate_target(self):
        dut = BootControl(delay_cycles=1000)

        def release():
            yield dut.bus.cyc.eq(0)
            yield dut.bus.stb.eq(0)
            yield
            yield

        def bench():
            yield from self.write(dut.bus, TARGET, 2, idle=False)
            yield dut.bus.dat_w.eq(3)
            for _ in range(8):
                yield
                self.assertEqual((yield dut.bus.ack), 0)
                self.assertEqual((yield dut.bus.err), 0)
            yield from release()
            self.assertEqual((yield from self.read(dut.bus, TARGET)), 2)
            yield from self.write(dut.bus, COMMAND, 6, idle=False)
            yield dut.bus.dat_w.eq(8)
            for _ in range(8):
                yield
                self.assertEqual((yield dut.bus.ack), 0)
                self.assertEqual((yield dut.bus.err), 0)
            yield from release()
            self.assertEqual((yield from self.read(dut.bus, COMMAND)), 6)
            self.assertEqual((yield from self.read(dut.bus, PENDING_TARGET)), 2)
            self.assertEqual((yield from self.read(dut.bus, ERROR)), 0)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_delay_and_remote_clock_sequence_survive_duplicate_commands(self):
        delay_cycles = 37
        dut = BootControl(delay_cycles=delay_cycles)
        samples = {"sys": [], "ru": []}

        @passive
        def sys_monitor():
            while True:
                samples["sys"].append(((yield dut.pending), (yield dut.pins.ENA)))
                yield

        @passive
        def ru_monitor():
            while True:
                samples["ru"].append(((yield dut.pins.ENA), (yield dut.pins.CONFIG),
                                      (yield dut.pins.CBSEL)))
                yield

        def bench():
            yield from self.write(dut.bus, TARGET, 2)
            yield from self.write(dut.bus, COMMAND, 18)
            for _ in range(4):
                yield from self.write(dut.bus, COMMAND, 18)
            for _ in range(150):
                if (yield dut.pins.CONFIG):
                    break
                yield
            else:
                self.fail("reboot did not arrive after the bounded delay")
            self.assertEqual((yield dut.pending), 1)
            self.assertEqual((yield dut.auto_ready), 0)
            for _ in range(20):
                self.assertEqual((yield dut.pins.CONFIG), 1)
                self.assertEqual((yield dut.pins.ENA), 1)
                self.assertEqual((yield dut.pins.CBSEL), 2)
                yield

        run_simulation(dut, {"sys": [bench(), sys_monitor()], "cam_ref": ru_monitor()}, clocks=CLOCKS)
        first_pending = next(i for i, (pending, _) in enumerate(samples["sys"]) if pending)
        first_enable = next(i for i, (_, ena) in enumerate(samples["sys"]) if ena)
        elapsed = first_enable - first_pending
        self.assertGreaterEqual(elapsed, delay_cycles)
        # The 25 MHz CDC adds only bounded synchronization latency. Replaying
        # command 18 four times must not restart the acknowledgement delay.
        self.assertLessEqual(elapsed, delay_cycles + 20)
        ru_enable = next(i for i, (ena, _, _) in enumerate(samples["ru"]) if ena)
        ru_config = next(i for i, (_, config, _) in enumerate(samples["ru"]) if config)
        self.assertGreaterEqual(ru_config - ru_enable, 3)
        self.assertTrue(all(ena and selector == 2
                            for ena, _, selector in samples["ru"][ru_enable:]))

    def test_automatic_request_waits_for_flash_then_is_accepted_once(self):
        dut = BootControl(delay_cycles=7)

        def bench():
            yield dut.flash_busy.eq(1)
            yield dut.auto_slot.eq(2)
            yield dut.auto_request.eq(1)
            yield
            for _ in range(15):
                self.assertEqual((yield dut.auto_ready), 0)
                self.assertEqual((yield dut.pending), 0)
                yield
            yield dut.flash_busy.eq(0)
            for _ in range(5):
                if (yield dut.pending):
                    break
                yield
            self.assertEqual((yield dut.pending), 1)
            self.assertEqual((yield dut.auto_ready), 0)
            # Holding VALID, as the golden scanner does until it sees READY,
            # cannot restart the timer or change a selected image.
            yield dut.auto_slot.eq(1)
            self.assertEqual((yield from self.read(dut.bus, PENDING_TARGET)), 2)
            self.assertEqual((yield from self.read(dut.bus, COMMAND)), 0)
            for _ in range(70):
                if (yield dut.pins.CONFIG):
                    break
                yield
            self.assertEqual((yield dut.pins.CONFIG), 1)
            self.assertEqual((yield dut.pins.CBSEL), 2)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_automatic_selection_has_priority_over_simultaneous_host_commit(self):
        dut = BootControl(delay_cycles=1000)

        def bench():
            yield from self.write(dut.bus, TARGET, 1)
            yield dut.auto_slot.eq(2)
            yield dut.auto_request.eq(1)
            yield from self.write(dut.bus, COMMAND, 17, error=1)
            yield dut.auto_request.eq(0)
            self.assertEqual((yield dut.pending), 1)
            self.assertEqual((yield from self.read(dut.bus, PENDING_TARGET)), 2)
            # A host must not receive a success response or accepted command
            # ID when the independently selected golden target actually wins.
            self.assertEqual((yield from self.read(dut.bus, COMMAND)), 0)
            self.assertEqual((yield from self.read(dut.bus, TARGET)), 1)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_late_flash_busy_defers_remote_configuration(self):
        dut = BootControl(delay_cycles=5)

        def bench():
            yield from self.write(dut.bus, TARGET, 1)
            yield from self.write(dut.bus, COMMAND, 5)
            yield dut.flash_busy.eq(1)
            for _ in range(80):
                self.assertEqual((yield dut.pins.CONFIG), 0,
                                 "remote boot began while serial flash was busy")
                yield
            yield dut.flash_busy.eq(0)
            for _ in range(100):
                if (yield dut.pins.CONFIG):
                    break
                yield
            self.assertEqual((yield dut.pins.CONFIG), 1)
            self.assertEqual((yield dut.pins.CBSEL), 1)

        run_simulation(dut, bench(), clocks=CLOCKS)



class FlashAccessMuxTests(unittest.TestCase):
    def test_scanner_owns_flash_and_host_is_explicitly_rejected_until_done(self):
        flash = wishbone.Interface(data_width=32, addressing="word")
        scanner = SimpleNamespace(busy=Signal(), bus=wishbone.Interface(
            data_width=32, addressing="word"))
        dut = FlashAccessMux(flash, scanner)
        host = dut.bus

        def bench():
            yield scanner.busy.eq(1)
            yield host.adr.eq(0x100)
            yield host.dat_w.eq(0x11223344)
            yield host.sel.eq(5)
            yield host.we.eq(1)
            yield host.cyc.eq(1)
            yield host.stb.eq(1)
            yield scanner.bus.adr.eq(0x12345)
            yield scanner.bus.dat_w.eq(0x55667788)
            yield scanner.bus.sel.eq(15)
            yield scanner.bus.we.eq(0)
            yield scanner.bus.cyc.eq(1)
            yield scanner.bus.stb.eq(1)
            yield flash.dat_r.eq(0xC86018)
            yield flash.ack.eq(1)
            yield
            self.assertEqual((yield flash.adr), 0x12345)
            self.assertEqual((yield flash.dat_w), 0x55667788)
            self.assertEqual((yield flash.sel), 15)
            self.assertEqual((yield flash.we), 0)
            self.assertEqual((yield flash.cyc), 1)
            self.assertEqual((yield flash.stb), 1)
            self.assertEqual((yield scanner.bus.dat_r), 0xC86018)
            self.assertEqual((yield scanner.bus.ack), 1)
            self.assertEqual((yield host.dat_r), 0)
            self.assertEqual((yield host.ack), 0)
            self.assertEqual((yield host.err), 1)
            yield host.stb.eq(0)
            yield
            self.assertEqual((yield host.err), 0)
            yield host.stb.eq(1)
            yield scanner.busy.eq(0)
            yield
            self.assertEqual((yield flash.adr), 0x100)
            self.assertEqual((yield flash.dat_w), 0x11223344)
            self.assertEqual((yield flash.sel), 5)
            self.assertEqual((yield flash.we), 1)
            self.assertEqual((yield host.dat_r), 0xC86018)
            self.assertEqual((yield host.ack), 1)
            self.assertEqual((yield host.err), 0)
            self.assertEqual((yield scanner.bus.dat_r), 0)
            self.assertEqual((yield scanner.bus.ack), 0)
            self.assertEqual((yield scanner.bus.err), 0)
            yield flash.ack.eq(0)
            yield flash.err.eq(1)
            yield
            self.assertEqual((yield host.ack), 0)
            self.assertEqual((yield host.err), 1)
            self.assertEqual((yield scanner.bus.err), 0)

        run_simulation(dut, bench())


if __name__ == "__main__":
    unittest.main()
