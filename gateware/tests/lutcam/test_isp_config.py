"""Canonical production MonoIsp programming; no hardware connections required."""

import random
import unittest

from tools.lutcam.isp import (
    DEFECT_CONTINUATION, DEFECT_COUNT, DEFECT_DELTA_MAX, DEFECT_TABLE,
    FRAME_PAIRS, STATUS, IspConfigError, pack_defect_table, program_defect_table,
    validate_defect_table,
)


BASE = 0x30000000


class OrderedBus:
    """Apply queued writes only when a scalar read completes their ordering."""

    def __init__(self, status=0):
        self.memory = {BASE + STATUS: status, BASE + DEFECT_COUNT: 123}
        self.pending = []
        self.operations = []
        self.bad_read_address = None
        self.bad_read_number = None
        self.fail_operation = None
        self.read_counts = {}

    def write(self, address, value):
        assert isinstance(address, int) and isinstance(value, int)
        self.operations.append(("write", address, value))
        if self.fail_operation == ("write", address):
            raise OSError("injected write transport failure")
        self.pending.append((address, value))

    def read(self, address):
        assert isinstance(address, int)
        self.operations.append(("read", address))
        if self.fail_operation == ("read", address):
            raise OSError("injected read transport failure")
        for destination, value in self.pending:
            self.memory[destination] = value
        self.pending.clear()
        number = self.read_counts.get(address, 0) + 1
        self.read_counts[address] = number
        value = self.memory.get(address, 0)
        if address == self.bad_read_address and self.bad_read_number in (None, number):
            return value ^ 1
        return value

    @property
    def writes(self):
        return [operation for operation in self.operations if operation[0] == "write"]


class CanonicalDefectTests(unittest.TestCase):
    def test_empty_first_last_and_delta_boundaries(self):
        self.assertEqual(pack_defect_table([]), [])
        self.assertEqual(validate_defect_table([]), [])
        self.assertEqual(pack_defect_table([(0, 1)]), [1])
        self.assertEqual(pack_defect_table([(DEFECT_DELTA_MAX, 2)]), [0xFFFFE])
        self.assertEqual(pack_defect_table([(DEFECT_DELTA_MAX + 1, 3)]), [0xFFFFC, 7])
        words = pack_defect_table([(FRAME_PAIRS - 1, 1)])
        self.assertEqual(words, [0xFFFFC, ((FRAME_PAIRS - 1 - DEFECT_DELTA_MAX) << 2) | 1])
        self.assertEqual(validate_defect_table(words), [(FRAME_PAIRS - 1, 1)])

    def test_full_table_with_and_without_continuation(self):
        for entries in (
            [(index, index % 3 + 1) for index in range(512)],
            [(FRAME_PAIRS - 511 + index, index % 3 + 1) for index in range(511)],
        ):
            with self.subTest(first=entries[0]):
                words = pack_defect_table(entries)
                self.assertEqual(len(words), 512)
                self.assertEqual(validate_defect_table(words), entries)
        with self.assertRaisesRegex(ValueError, "512 encoded"):
            pack_defect_table([(FRAME_PAIRS - 512 + index, 1) for index in range(512)])

    def test_every_fixed_frame_encoding_has_at_most_one_continuation(self):
        randomizer = random.Random(74)
        cases = [[(0, 1), (FRAME_PAIRS - 1, 3)], [(FRAME_PAIRS - 1, 2)]]
        for count in (1, 2, 3, 32, 256, 511):
            for _ in range(20):
                addresses = sorted(randomizer.sample(range(FRAME_PAIRS), count))
                cases.append([(address, randomizer.randrange(1, 4)) for address in addresses])
        for entries in cases:
            words = pack_defect_table(entries)
            self.assertLessEqual(words.count(DEFECT_CONTINUATION), 1)
            self.assertEqual(validate_defect_table(words), entries)

    def test_reject_invalid_semantic_entries(self):
        cases = [
            [(-1, 1)], [(FRAME_PAIRS, 1)], [(4, 0)], [(4, 4)],
            [(4, 1), (4, 2)], [(4, 1), (3, 2)], [(True, 1)],
            [(0, True)], [(0.0, 1)], [(0, 1.0)], [(0,)], [(0, 1, 2)], [0],
            [(index, 1) for index in range(513)],
        ]
        for entries in cases:
            with self.subTest(entries=entries[:3]), self.assertRaises(ValueError):
                pack_defect_table(entries)

    def test_reject_noncanonical_encoded_words(self):
        cases = [
            [0], [DEFECT_CONTINUATION], [1, 0], [0, 1], [4, 1],
            [DEFECT_CONTINUATION, 1],  # Exact max gap needs no continuation.
            [4, DEFECT_CONTINUATION, 1],  # Split continuation.
            [DEFECT_CONTINUATION, DEFECT_CONTINUATION, 1],
            [1, 2], [-1], [1 << 20], [True], [1.0],
            [DEFECT_CONTINUATION, ((FRAME_PAIRS - DEFECT_DELTA_MAX) << 2) | 1],
            [1] + [0] * 510 + [5],
            [1] * 513,
        ]
        for words in cases:
            with self.subTest(words=words[:3]), self.assertRaises(ValueError):
                validate_defect_table(words)


class DefectProgrammingTests(unittest.TestCase):
    def test_program_full_table_verifies_words_before_committing_count(self):
        words = pack_defect_table([(FRAME_PAIRS - 511 + index, 1) for index in range(511)])
        bus = OrderedBus(status=0x7C)  # Unrelated status bits do not imply busy.
        self.assertEqual(program_defect_table(bus, BASE, iter(words)), 512)
        expected = [("read", BASE + STATUS),
                    ("write", BASE + DEFECT_COUNT, 0), ("read", BASE + DEFECT_COUNT)]
        for index, word in enumerate(words):
            address = BASE + DEFECT_TABLE + 4 * index
            expected.extend([("write", address, word), ("read", address)])
        expected.extend([("read", BASE + STATUS),
                         ("write", BASE + DEFECT_COUNT, 512), ("read", BASE + DEFECT_COUNT)])
        self.assertEqual(bus.operations, expected)
        self.assertEqual(bus.memory[BASE + DEFECT_COUNT], 512)
        self.assertEqual(bus.pending, [])

    def test_empty_table_clears_count_without_touching_storage(self):
        bus = OrderedBus()
        self.assertEqual(program_defect_table(bus, BASE, []), 0)
        self.assertEqual(bus.writes, [("write", BASE + DEFECT_COUNT, 0)] * 2)
        self.assertEqual(bus.memory[BASE + DEFECT_COUNT], 0)

    def test_invalid_input_has_no_bus_access(self):
        for base, words in [(BASE, [1] + [0] * 510 + [5]), (BASE, [1 << 20]),
                            (BASE + 1, []), (-4, []), (True, []), (0xFFFFF000, [])]:
            bus = OrderedBus()
            with self.subTest(base=base, words=words[:3]), self.assertRaises(ValueError):
                program_defect_table(bus, base, words)
            self.assertEqual(bus.operations, [])

    def test_stop_and_drain_both_required_before_any_write(self):
        for status in (1, 2, 3, 0x7F):
            bus = OrderedBus(status)
            with self.subTest(status=status), self.assertRaisesRegex(IspConfigError, "stopped and drained"):
                program_defect_table(bus, BASE, [1])
            self.assertEqual(bus.operations, [("read", BASE + STATUS)])

    def test_silent_timeout_client_rejected_before_any_bus_access(self):
        bus = OrderedBus()
        bus.raise_on_timeout = False
        with self.assertRaisesRegex(ValueError, "raise_on_timeout=True"):
            program_defect_table(bus, BASE, [])
        self.assertEqual(bus.operations, [])
        bus.raise_on_timeout = True
        self.assertEqual(program_defect_table(bus, BASE, []), 0)

    def test_failed_clear_readback_prevents_table_writes(self):
        bus = OrderedBus()
        bus.bad_read_address = BASE + DEFECT_COUNT
        with self.assertRaisesRegex(IspConfigError, "cleared defect count"):
            program_defect_table(bus, BASE, [1])
        self.assertEqual(bus.writes, [("write", BASE + DEFECT_COUNT, 0)])

    def test_failed_word_readback_leaves_count_zero(self):
        words = pack_defect_table([(0, 1), (2, 3), (FRAME_PAIRS - 1, 2)])
        for index in range(len(words)):
            bus = OrderedBus()
            bus.bad_read_address = BASE + DEFECT_TABLE + 4 * index
            with self.subTest(index=index), self.assertRaisesRegex(IspConfigError, f"defect word {index}"):
                program_defect_table(bus, BASE, words)
            self.assertEqual(bus.memory[BASE + DEFECT_COUNT], 0)
            self.assertEqual(len(bus.writes), index + 2)

    def test_transport_failure_during_table_update_never_commits_count(self):
        for action in ("read", "write"):
            bus = OrderedBus()
            bus.fail_operation = (action, BASE + DEFECT_TABLE)
            with self.subTest(action=action), self.assertRaises(OSError):
                program_defect_table(bus, BASE, [1])
            self.assertEqual(bus.memory[BASE + DEFECT_COUNT], 0)
            self.assertNotIn(("write", BASE + DEFECT_COUNT, 1), bus.operations)

    def test_restart_during_programming_prevents_count_commit(self):
        bus = OrderedBus()
        bus.bad_read_address = BASE + STATUS
        bus.bad_read_number = 2
        with self.assertRaisesRegex(IspConfigError, "stopped and drained"):
            program_defect_table(bus, BASE, [1])
        self.assertEqual(bus.memory[BASE + DEFECT_COUNT], 0)

    def test_final_commit_readback_failure_is_reported(self):
        bus = OrderedBus()
        bus.bad_read_address = BASE + DEFECT_COUNT
        bus.bad_read_number = 2
        with self.assertRaisesRegex(IspConfigError, "committed defect count"):
            program_defect_table(bus, BASE, [1])
        # The fully verified table may already be committed when this fails.
        self.assertEqual(bus.memory[BASE + DEFECT_COUNT], 1)


if __name__ == "__main__":
    unittest.main()
