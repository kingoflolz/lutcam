"""Exercise boot-register reads sharing the validated image identity ROM."""

import unittest

from migen import Module, ResetInserter
from migen.sim import run_simulation

from fpga.litex.lutcam.boot import BOOT_MAGIC, BootControl
from fpga.litex.lutcam.image_identity import ImageIdentity, identity_words


CLOCKS = {"sys": 10, "cam_ref": 48}
BUILD_ID = "f00123456789abcdea5510559f602bad"
BUILD_WORDS = [0xF0012345, 0x6789ABCD, 0xEA551055, 0x9F602BAD]
BUILD_BASE, COMMAND, PENDING_TARGET = 0x10, 0x24, 0x2C


def make_dut(words=None):
    dut = Module()
    dut.submodules.identity = identity = ImageIdentity(
        build_id=BUILD_ID, init_words=words, sequential_build_id=True)
    dut.submodules.boot = boot = BootControl(identity=identity, delay_cycles=8)
    return dut, identity, boot


def drive(bus, address, value=None):
    yield bus.adr.eq(address // 4)
    yield bus.dat_w.eq(0 if value is None else value)
    yield bus.we.eq(value is not None)
    yield bus.sel.eq(15)
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)


def release(bus):
    yield bus.cyc.eq(0)
    yield bus.stb.eq(0)
    yield
    yield


def transfer(bus, address, value=None, *, hold=0):
    """Allow startup to finish, and require a single response to a held request."""
    yield from drive(bus, address, value)
    yield
    for latency in range(70):
        ack, error = (yield bus.ack), (yield bus.err)
        assert not (ack and error), "Wishbone ACK and ERR asserted together"
        if ack or error:
            result = (yield bus.dat_r)
            for _ in range(hold):
                yield
                assert not (yield bus.ack) and not (yield bus.err), "duplicate reply"
                assert (yield bus.dat_r) == result, "held response changed"
            yield from release(bus)
            return result, error, latency
        yield
    raise AssertionError("identity register transaction did not terminate")


class ImageIdentityReadbackTests(unittest.TestCase):
    def test_startup_first_read_and_rom_latency(self):
        # Exercise both sides of ready's rising edge. Every word differs from
        # the final checksum word, which must never leak into the first reply.
        for start in (0, 1, 28, 29, 30, 31, 32, 33, 40):
            for index, expected in enumerate(BUILD_WORDS):
                with self.subTest(start=start, index=index):
                    dut, identity, boot = make_dut()

                    def bench():
                        for _ in range(start):
                            yield
                        value, error, latency = yield from transfer(
                            boot.bus, BUILD_BASE + 4 * index, hold=3)
                        self.assertEqual((value, error), (expected, 0))
                        self.assertEqual((yield identity.ready), 1)
                        self.assertEqual((yield identity.valid), 1)
                        self.assertEqual((yield identity.image_slot), 1)
                        if start == 0:
                            self.assertGreater(latency, 31)
                        _, error, ordinary_latency = yield from transfer(boot.bus, 0)
                        self.assertEqual(error, 0)
                        _, error, rom_latency = yield from transfer(
                            boot.bus, BUILD_BASE + 4 * index)
                        self.assertEqual(error, 0)
                        self.assertEqual(rom_latency, ordinary_latency + 2)

                    run_simulation(dut, bench(), clocks=CLOCKS)

    def test_startup_status_and_corrupt_table_fail_closed(self):
        for slot in ("A", "B"):
            for corrupt in (None, 0, 1, 2, 4, 15):
                with self.subTest(slot=slot, corrupt=corrupt):
                    words = identity_words(BUILD_ID, slot)
                    if corrupt is not None:
                        words[corrupt] ^= 2 if corrupt == 2 else 1
                    if corrupt == 0:
                        # Magic must reject even a consistent XOR. Reserved
                        # word 1 is protected only by the checksum.
                        words[-1] = 0
                        for index, word in enumerate(words[:-1]):
                            if index != 2:
                                words[-1] ^= word
                    dut, identity, boot = make_dut(words)
                    dut.submodules.reference = reference = ImageIdentity(
                        build_id=BUILD_ID, init_words=words)

                    def bench():
                        yield from drive(boot.bus, BUILD_BASE + 12)
                        replies = []
                        for cycle in range(50):
                            self.assertEqual((yield identity.ready),
                                             (yield reference.ready), cycle)
                            self.assertEqual((yield identity.valid),
                                             (yield reference.valid), cycle)
                            self.assertEqual((yield identity.image_slot),
                                             (yield reference.image_slot), cycle)
                            if (yield boot.bus.ack):
                                replies.append((yield boot.bus.dat_r))
                            self.assertEqual((yield boot.bus.err), 0)
                            yield
                        self.assertEqual(replies, [words[6]])
                        yield from release(boot.bus)
                        self.assertEqual((yield identity.valid), int(corrupt is None))
                        if corrupt is not None:
                            self.assertEqual((yield identity.image_slot), 0)
                            # No accepted command exists to replay: both the
                            # first command and its duplicate must be rejected.
                            for _ in range(2):
                                _, error, _ = yield from transfer(boot.bus, COMMAND, 6)
                                self.assertEqual(error, 1)
                                self.assertEqual((yield boot.pending), 0)
                            value, error, _ = yield from transfer(boot.bus, COMMAND)
                            self.assertEqual((value, error), (0, 0))
                            yield boot.auto_slot.eq(2)
                            yield boot.auto_request.eq(1)
                            for _ in range(40):
                                self.assertEqual((yield boot.auto_ready), 0)
                                self.assertEqual((yield boot.pending), 0)
                                self.assertEqual((yield boot.pins.CONFIG), 0)
                                yield

                    run_simulation(dut, bench(), clocks=CLOCKS)

    def test_cancellation_then_unrelated_request_has_no_stale_reply(self):
        for start in (0, 28, 32, 40):
            for age in (1, 2):
                with self.subTest(start=start, age=age):
                    dut, _, boot = make_dut()

                    def bench():
                        for _ in range(start):
                            yield
                        yield from drive(boot.bus, BUILD_BASE + 12)
                        for _ in range(age):
                            self.assertEqual((yield boot.bus.ack), 0)
                            yield
                        yield from release(boot.bus)
                        for _ in range(4):
                            self.assertEqual((yield boot.bus.ack), 0)
                            self.assertEqual((yield boot.bus.err), 0)
                            yield
                        value, error, _ = yield from transfer(boot.bus, 0)
                        self.assertEqual((value, error), (BOOT_MAGIC, 0))
                        for index in (1, 3, 0, 2):
                            value, error, _ = yield from transfer(
                                boot.bus, BUILD_BASE + 4 * index)
                            self.assertEqual((value, error), (BUILD_WORDS[index], 0))

                    run_simulation(dut, bench(), clocks=CLOCKS)

    def test_held_request_captures_index_and_preserves_command_replay(self):
        dut, _, boot = make_dut()

        def bench():
            for _ in range(40):
                yield
            yield from drive(boot.bus, BUILD_BASE + 4)
            yield
            yield
            # Pins changing without withdrawal cannot alter the captured index
            # or turn the accepted read into a reboot command.
            yield boot.bus.adr.eq(COMMAND // 4)
            yield boot.bus.we.eq(1)
            yield boot.bus.dat_w.eq(6)
            replies = []
            for _ in range(12):
                if (yield boot.bus.ack):
                    replies.append((yield boot.bus.dat_r))
                self.assertEqual((yield boot.bus.err), 0)
                self.assertEqual((yield boot.pending), 0)
                yield
            self.assertEqual(replies, [BUILD_WORDS[1]])
            yield from release(boot.bus)
            _, error, _ = yield from transfer(boot.bus, COMMAND, 6, hold=5)
            self.assertEqual(error, 0)
            _, error, _ = yield from transfer(boot.bus, COMMAND, 6)
            self.assertEqual(error, 0)
            _, error, _ = yield from transfer(boot.bus, COMMAND, 10)
            self.assertEqual(error, 1)
            value, error, _ = yield from transfer(boot.bus, PENDING_TARGET)
            self.assertEqual((value, error), (2, 0))
            self.assertEqual((yield boot.pending), 1)

        run_simulation(dut, bench(), clocks=CLOCKS)

    def test_reset_cancels_pending_read_and_revalidates(self):
        for start, age in ((0, 3), (28, 3), (40, 1), (40, 2), (40, 4), (40, 8)):
            with self.subTest(start=start, age=age):
                dut, identity, boot = make_dut()
                dut = ResetInserter(["sys"])(dut)

                def bench():
                    for _ in range(start):
                        yield
                    yield from drive(boot.bus, BUILD_BASE + 12)
                    for _ in range(age):
                        yield
                    yield dut.reset_sys.eq(1)
                    yield from release(boot.bus)
                    self.assertEqual((yield identity.ready), 0)
                    self.assertEqual((yield identity.valid), 0)
                    self.assertEqual((yield identity.image_slot), 0)
                    self.assertEqual((yield boot.pending), 0)
                    self.assertEqual((yield boot.bus.ack), 0)
                    self.assertEqual((yield boot.bus.err), 0)
                    yield dut.reset_sys.eq(0)
                    yield
                    value, error, _ = yield from transfer(boot.bus, BUILD_BASE + 8)
                    self.assertEqual((value, error), (BUILD_WORDS[2], 0))
                    self.assertEqual((yield identity.valid), 1)

                run_simulation(dut, bench(), clocks=CLOCKS)

    def test_optional_interface_keeps_patchable_table_and_latched_default(self):
        default = ImageIdentity(build_id=BUILD_ID)
        sequential = ImageIdentity(build_id=BUILD_ID, sequential_build_id=True)
        self.assertTrue(hasattr(default, "build_id_words"))
        self.assertTrue(hasattr(default, "build_id"))
        self.assertFalse(hasattr(default, "build_id_index"))
        self.assertFalse(hasattr(sequential, "build_id_words"))
        self.assertFalse(hasattr(sequential, "build_id"))
        self.assertTrue(hasattr(sequential, "build_id_index"))
        self.assertEqual(sequential.get_memories(), [])
        self.assertEqual(sequential.memory.name_override, "ota_identity_rom")
        self.assertEqual(sequential.memory.init, default.memory.init)
        self.assertEqual(identity_words(BUILD_ID)[3:7], BUILD_WORDS)
        a, b = identity_words(BUILD_ID, "A"), identity_words(BUILD_ID, "B")
        self.assertEqual([left ^ right for left, right in zip(a, b)],
                         [0, 0, 1] + [0] * 13)


if __name__ == "__main__":
    unittest.main()
