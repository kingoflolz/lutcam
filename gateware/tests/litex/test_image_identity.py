"""The post-route identity ROM fails closed and latches an unambiguous slot."""

import unittest

from migen.sim import run_simulation

from fpga.litex.lutcam.image_identity import (
    IDENTITY_DEPTH, IDENTITY_MAGIC, ImageIdentity, identity_words,
)


BUILD_ID = "0123456789abcdeffedcba9876543210"
BUILD_WORDS = [0x01234567, 0x89ABCDEF, 0xFEDCBA98, 0x76543210]


class ImageIdentityTests(unittest.TestCase):
    def test_a_and_b_tables_differ_in_exactly_one_bit(self):
        a = identity_words(BUILD_ID, "A")
        b = identity_words(BUILD_ID, "B")
        self.assertEqual(len(a), IDENTITY_DEPTH)
        self.assertEqual(a[:3], [IDENTITY_MAGIC, 0, 0])
        self.assertEqual(b[:3], [IDENTITY_MAGIC, 0, 1])
        self.assertEqual(a[3:7], BUILD_WORDS)
        self.assertEqual([left ^ right for left, right in zip(a, b)],
                         [0, 0, 1] + [0] * 13)
        self.assertEqual(identity_words(bytes.fromhex(BUILD_ID)), a)
        self.assertEqual(identity_words(BUILD_ID.upper()), a)
        # Slot is protected by the configuration CRC, while the identity check
        # deliberately covers all other words so that only one ROM bit differs.
        checksum = 0
        for index, word in enumerate(a):
            if index != 2:
                checksum ^= word
        self.assertEqual(checksum, 0)

    def test_startup_validates_both_slots_and_big_endian_build_id(self):
        for slot, expected in (("A", 1), ("B", 2)):
            with self.subTest(slot=slot):
                dut = ImageIdentity(build_id=BUILD_ID,
                                    init_words=identity_words(BUILD_ID, slot))

                def bench():
                    # A synchronous ROM read takes two cycles for each of the
                    # sixteen words. Nothing is advertised as valid early.
                    for cycle in range(32):
                        self.assertEqual((yield dut.ready), 0, cycle)
                        self.assertEqual((yield dut.valid), 0, cycle)
                        self.assertEqual((yield dut.image_slot), 0, cycle)
                        yield
                    self.assertEqual((yield dut.ready), 1)
                    self.assertEqual((yield dut.valid), 1)
                    self.assertEqual((yield dut.image_slot), expected)
                    words = []
                    for word in dut.build_id_words:
                        words.append((yield word))
                    self.assertEqual(words, BUILD_WORDS)
                    self.assertEqual((yield dut.build_id), int(BUILD_ID, 16))

                run_simulation(dut, bench())

    def test_corrupt_tables_finish_but_do_not_claim_a_slot(self):
        cases = {
            "magic": (0, IDENTITY_MAGIC ^ 1),
            "reserved_checksum": (1, 1),
            "slot_high_bit": (2, 2),
            "slot_all_bits": (2, 0xFFFFFFFF),
            "build_id": (4, BUILD_WORDS[1] ^ 0x80000000),
            "checksum": (15, identity_words(BUILD_ID)[15] ^ 1),
        }
        for name, (index, replacement) in cases.items():
            with self.subTest(corruption=name):
                words = identity_words(BUILD_ID)
                words[index] = replacement
                # Magic must be rejected independently of the XOR
                # check: recompute that check so malformed metadata cannot pass.
                if name == "magic":
                    words[-1] = 0
                    for i, word in enumerate(words[:-1]):
                        if i != 2:
                            words[-1] ^= word
                dut = ImageIdentity(build_id=BUILD_ID, init_words=words)

                def bench():
                    for _ in range(64):
                        self.assertEqual((yield dut.valid), 0)
                        self.assertEqual((yield dut.image_slot), 0)
                        yield
                    self.assertEqual((yield dut.ready), 1)

                run_simulation(dut, bench())


    def test_valid_identity_is_latched_after_rom_is_overwritten(self):
        dut = ImageIdentity(build_id=BUILD_ID)
        # This extra simulation-only port lets the test prove that status is a
        # latch, not a later read of a changed ROM location.
        write_port = dut.memory.get_port(write_capable=True)
        dut.specials += write_port

        def bench():
            for _ in range(32):
                yield
            self.assertEqual((yield dut.ready), 1)
            for cycle in range(64):
                self.assertEqual((yield dut.ready), 1)
                self.assertEqual((yield dut.valid), 1)
                self.assertEqual((yield dut.image_slot), 1)
                self.assertEqual((yield dut.build_id), int(BUILD_ID, 16))
                yield write_port.we.eq(cycle < IDENTITY_DEPTH)
                yield write_port.adr.eq(cycle % IDENTITY_DEPTH)
                yield write_port.dat_w.eq(0)
                yield

        run_simulation(dut, bench())

    def test_invalid_build_ids_are_rejected(self):
        for value in (None, 1, "", "0" * 31, "0" * 33, "g" * 32,
                      b"\x00" * 15, b"\x00" * 17,
                      " " + "0" * 31, "0" * 31 + " ",
                      "+" + "0" * 31, "-" + "0" * 31, "0x" + "0" * 30):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    identity_words(value)

    def test_invalid_slots_are_rejected(self):
        for slot in (None, "", "a", "b", "golden", 0, 1, 2):
            with self.subTest(slot=slot):
                with self.assertRaises(ValueError):
                    identity_words(BUILD_ID, slot)

    def test_invalid_rom_tables_are_rejected(self):
        for words in ([], [0] * 15, [0] * 17, [-1] + [0] * 15,
                      [0] * 15 + [1 << 32], [0] * 15 + [0.5],
                      [0] * 15 + ["0"], [0] * 15 + [None]):
            with self.subTest(words=words):
                with self.assertRaises(ValueError):
                    ImageIdentity(build_id=BUILD_ID, init_words=words)


if __name__ == "__main__":
    unittest.main()
