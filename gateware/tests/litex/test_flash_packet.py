"""ABI 3 command records checked against an independent pin-level NOR model."""

from pathlib import Path
import sys
import unittest

from migen import Signal
from migen.sim import run_simulation

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from lutcam.flash_packet import PacketFlashController
from tests.litex.spi_nor_model import SpiNor, transfer


STATUS, ACCEPTED, DONE, ERROR, RESULT = 0x08, 0x1C, 0x20, 0x24, 0x28
BUFFER, COMMAND_WINDOW = 0x100, 0x400
READ, PROGRAM, ERASE4K, ERASE64K, JEDEC, READ_STATUS = range(1, 7)


def command_words(command_id, opcode, address=0, length=0, payload=b""):
    """Build the public wire record without importing controller internals."""
    if opcode == PROGRAM:
        length = len(payload)
    words = [command_id, address, opcode | (length << 8)]
    if opcode == PROGRAM:
        words += [int.from_bytes(payload[index:index + 4].ljust(4, b"\xa5"), "little")
                  for index in range(0, len(payload), 4)]
    return words + [command_id]


class PacketFlashTests(unittest.TestCase):
    def fixture(self, *, boot_slot=1, poll_timeout_cycles=50000,
                golden_maintenance=False, **model_options):
        dut = PacketFlashController(clk_freq=200, spi_clk_freq=50, boot_slot=boot_slot,
                                    poll_timeout_cycles=poll_timeout_cycles,
                                    golden_maintenance=golden_maintenance)
        return dut, SpiNor(dut.pads, **model_options)

    def read(self, dut, offset, **kwargs):
        value, error = yield from transfer(dut.bus, offset, **kwargs)
        self.assertEqual(error, 0, f"read {offset:#x} failed")
        return value

    def word(self, dut, words, index, *, source=0, first=None, last=None,
             count=None, offset=None, data=None, sel=15, idle=True, error=0):
        yield dut.record_first.eq(index == 0 if first is None else first)
        yield dut.record_last.eq(index == len(words) - 1 if last is None else last)
        yield dut.record_count.eq(len(words) if count is None else count)
        yield dut.record_source.eq(source)
        _, observed = yield from transfer(
            dut.bus, COMMAND_WINDOW + index * 4 if offset is None else offset,
            words[index] if data is None else data, sel=sel, idle=idle, limit=16)
        self.assertEqual(observed, error, f"record word {index} returned unexpected ERR")

    def packet(self, dut, words, *, source=0, continuous=False):
        for index in range(len(words)):
            yield from self.word(dut, words, index, source=source,
                                 idle=not continuous or index == len(words) - 1)

    def start(self, dut, command_id, opcode, address=0, length=0, payload=b"", **kwargs):
        yield from self.packet(dut, command_words(command_id, opcode, address, length, payload), **kwargs)
        self.assertEqual((yield from self.read(dut, ACCEPTED)), command_id)

    def complete(self, dut, command_id, *, error=0, limit=5000):
        for _ in range(limit):
            status = yield from self.read(dut, STATUS)
            if not status & 1:
                self.assertEqual((yield from self.read(dut, DONE)), command_id)
                self.assertEqual((yield from self.read(dut, ERROR)), error)
                self.assertEqual(bool(status & 2), bool(error))
                return
            for _ in range(8):
                yield
        self.fail(f"command {command_id} did not complete")

    def read_buffer(self, dut, length=256, **kwargs):
        data = bytearray()
        for offset in range(0, length, 4):
            data.extend((yield from self.read(dut, BUFFER + offset, **kwargs)).to_bytes(4, "little"))
        return bytes(data[:length])

    def assert_uncommitted(self, dut, flash, command_id=0, error=0):
        self.assertEqual((yield from self.read(dut, ACCEPTED)), command_id)
        self.assertEqual((yield from self.read(dut, DONE)), command_id)
        self.assertEqual((yield from self.read(dut, ERROR)), error)
        self.assertFalse((yield dut.busy))
        self.assertEqual(flash.transactions, [])
        self.assertEqual((yield dut.pads.cs_n), 1)

    def test_manifest_reserved_registers_and_readonly_receive_buffer(self):
        dut, flash = self.fixture(boot_slot=2)

        def bench():
            values = ((0x00, 0x4C465331), (0x04, 0), (STATUS, 0), (0x0C, 0x1000000),
                      (0x10, 0), (0x14, 0), (0x18, 0), (ACCEPTED, 0), (DONE, 0),
                      (ERROR, 0), (RESULT, 0), (0x2C, 2), (0x30, 50), (0x34, 0),
                      (0x38, 256), (0x3C, 128), (0x40, BUFFER))
            for offset, expected in values:
                self.assertEqual((yield from self.read(dut, offset)), expected)
                self.assertEqual((yield from transfer(dut.bus, offset, 0x12345678))[1], 1)
            for offset in (BUFFER, BUFFER + 252):
                yield from self.read(dut, offset)
                self.assertEqual((yield from transfer(dut.bus, offset, 0))[1], 1)
            for offset in (0xFC, 0x200, COMMAND_WINDOW, 0x510, 0xFFC):
                self.assertEqual((yield from transfer(dut.bus, offset))[1], 1)
            yield from self.assert_uncommitted(dut, flash)

        run_simulation(dut, [bench(), flash.process()])

    def test_jedec_status_and_full_page_read_have_exact_spi_payloads(self):
        dut, flash = self.fixture()
        address = 0x1FFF00  # Golden protection never forbids a read.
        payload = bytes((index * 43 + 0x81) & 255 for index in range(256))
        flash.memory.update({address + index: value for index, value in enumerate(payload)})

        def bench():
            yield from self.start(dut, 1, JEDEC, address=0xFEDCBA98, continuous=True)
            yield from self.complete(dut, 1)
            self.assertEqual((yield from self.read(dut, RESULT)), 0xC86018)
            self.assertEqual((yield from self.read_buffer(dut, 3)), b"\xc8\x60\x18")
            yield from self.start(dut, 2, READ, address, 256, source=2)
            yield from self.complete(dut, 2)
            self.assertEqual((yield from self.read_buffer(dut, idle=False)), payload)
            flash.wel = True
            yield from self.start(dut, 3, READ_STATUS, address=0xF1E2D3C4, source=1)
            yield from self.complete(dut, 3)
            self.assertEqual((yield from self.read(dut, RESULT)), 2)
            self.assertEqual((yield from self.read_buffer(dut)), b"\x02" + payload[1:])
            self.assertEqual([t[0] for t in flash.transactions], [0x05, 0x9F, 0x05, 0x03, 0x05, 0x05])
            self.assertEqual([t for t in flash.transactions if t[0] == 0x03],
                             [b"\x03" + address.to_bytes(3, "big") + b"\xff" * 256])

        run_simulation(dut, [bench(), flash.process()])

    def test_program_waits_for_commit_and_ignores_padding_without_crc(self):
        dut, flash = self.fixture()
        address, payload = 0x4000FB, b"\x12\x34\x56\x78\x9a"
        words = command_words(1, PROGRAM, address, payload=payload)

        def bench():
            for index in range(len(words) - 1):
                yield from self.word(dut, words, index, idle=False)
            yield dut.bus.cyc.eq(0)
            yield dut.bus.stb.eq(0)
            for _ in range(100):
                yield
            yield from self.assert_uncommitted(dut, flash)
            yield from self.word(dut, words, len(words) - 1)
            yield from self.complete(dut, 1)
            self.assertEqual((yield from self.read(dut, 0x34)), 0)
            self.assertEqual([t for t in flash.transactions if t[0] == 0x02],
                             [b"\x02" + address.to_bytes(3, "big") + payload])
            self.assertEqual(bytes(flash.memory[address + index] for index in range(len(payload))), payload)
            self.assertNotIn(address - 1, flash.memory)
            self.assertNotIn(address + len(payload), flash.memory)
            yield from self.start(dut, 2, READ, address, len(payload))
            yield from self.complete(dut, 2)
            self.assertEqual((yield from self.read_buffer(dut, len(payload))), payload)

        run_simulation(dut, [bench(), flash.process()])

    def test_sector_and_block_erase_use_implicit_lengths_and_preserve_neighbors(self):
        dut, flash = self.fixture()
        cases = ((0x400000, 4096, ERASE4K, 0x20), (0x410000, 65536, ERASE64K, 0xD8))
        for address, size, _, _ in cases:
            for key in (address - 1, address, address + size - 1, address + size):
                flash.memory[key] = 0x36

        def bench():
            for command_id, (address, size, opcode, wire_opcode) in enumerate(cases, 1):
                yield from self.start(dut, command_id, opcode, address)
                yield from self.complete(dut, command_id)
                self.assertNotIn(address, flash.memory)
                self.assertNotIn(address + size - 1, flash.memory)
                self.assertEqual(flash.memory[address - 1], 0x36)
                self.assertEqual(flash.memory[address + size], 0x36)
                self.assertIn(bytes([wire_opcode]) + address.to_bytes(3, "big"), flash.transactions)
            self.assertEqual(flash.wren_count, 2)

        run_simulation(dut, [bench(), flash.process()])

    def test_busy_duplicate_record_cannot_overwrite_program_or_repeat_spi(self):
        dut, flash = self.fixture()
        address = 0x400100
        payload = bytes((index * 31 ^ 0x96) & 255 for index in range(256))
        poison = bytes(value ^ 255 for value in payload)
        latencies = []

        def bench():
            yield from self.start(dut, 1, PROGRAM, address, payload=payload, continuous=True)
            self.assertTrue((yield dut.busy))
            yield from self.packet(dut, command_words(2, JEDEC))
            self.assertEqual((yield from self.read(dut, ACCEPTED)), 1)
            ignored = command_words(1, PROGRAM, address + 256, payload=poison)
            # Ignored records validate outer framing only. Their bodies
            # must not alter or revalidate the active command descriptor.
            ignored[2] = 0xFFFFFFFF
            yield from self.packet(dut, ignored, source=1, continuous=True)
            self.assertEqual((yield from self.read(dut, ACCEPTED)), 1)
            self.assertTrue((yield dut.busy))
            self.assertEqual((yield from self.read(dut, DONE)), 0)
            program_seen = False
            for attempt in range(5000):
                if not (yield dut.busy):
                    break
                offset = (0, 4, 128, 252)[attempt % 4]
                value = yield from self.read(dut, BUFFER + offset, idle=False,
                                             limit=16, latencies=latencies)
                self.assertEqual(value, int.from_bytes(payload[offset:offset + 4], "little"))
                program_seen |= bool(flash.current and flash.current[0] == 0x02
                                     and not (yield dut.pads.cs_n))
            else:
                self.fail("continuous buffer reads prevented PROGRAM completion")
            self.assertTrue(program_seen)
            self.assertTrue(latencies)
            self.assertLessEqual(max(latencies), 16)
            yield from self.complete(dut, 1)
            self.assertEqual([t for t in flash.transactions if t[0] == 0x02],
                             [b"\x02" + address.to_bytes(3, "big") + payload])
            self.assertEqual(bytes(flash.memory[address + index] for index in range(256)), payload)
            self.assertTrue(all(address + 256 + index not in flash.memory for index in range(256)))
            count = len(flash.transactions)
            yield from self.start(dut, 1, PROGRAM, address + 256, payload=poison)
            for _ in range(100):
                yield
            self.assertEqual(len(flash.transactions), count)
            yield from self.start(dut, 2, JEDEC)
            yield from self.complete(dut, 2)
            for ignored_id in (0, 1, 4, 0xFFFFFFFF):
                yield from self.packet(dut, command_words(ignored_id, JEDEC))
            self.assertEqual((yield from self.read(dut, ACCEPTED)), 2)

        run_simulation(dut, [bench(), flash.process()])

    def test_invalid_descriptors_consume_only_next_id_without_spi(self):
        dut, flash = self.fixture()
        cases = ((READ, 0x1000000, 1, b"", 1),
                 (PROGRAM, 0x01400000, 0, b"\x12", 1),
                 (READ, 0x400000, 0, b"", 3), (READ, 0x400000, 257, b"", 3),
                 (PROGRAM, 0x400000, 0, b"", 3),
                 (READ, 0x4000FF, 2, b"", 4), (PROGRAM, 0x4000FF, 0, b"\x12\x34", 4),
                 (READ, 0x4000FF, 256, b"", 4),
                 (PROGRAM, 0x4000FF, 0, bytes(range(256)), 4),
                 (ERASE4K, 0x400001, 0, b"", 4), (ERASE64K, 0x401000, 0, b"", 4),
                 (PROGRAM, 0x000000, 0, b"\x12", 2),
                 (ERASE4K, 0x1FF000, 0, b"", 2),
                 (PROGRAM, 0x200000, 0, b"\x12", 2),
                 (ERASE64K, 0x3F0000, 0, b"", 2),
                 (JEDEC, 0, 1, b"", 3), (READ_STATUS, 0, 1, b"", 3),
                 (0x82, 0, 0, b"", 5), (99, 0, 0, b"", 5))

        def bench():
            for command_id, (opcode, address, length, payload, error) in enumerate(cases, 1):
                yield from self.start(dut, command_id, opcode, address, length, payload)
                yield from self.complete(dut, command_id, error=error)
                self.assertEqual(flash.transactions, [])
            last_id = len(cases)
            # Transport rejection and replay preserve the last operation's error.
            yield from self.packet(dut, command_words(last_id + 2, JEDEC))
            yield from self.start(dut, last_id, JEDEC)
            yield from self.complete(dut, last_id, error=5)
            yield from self.start(dut, last_id + 1, JEDEC)
            yield from self.complete(dut, last_id + 1)

        run_simulation(dut, [bench(), flash.process()])

    def test_malformed_records_never_consume_id_or_access_spi(self):
        dut, flash = self.fixture()
        words = command_words(1, JEDEC)
        # Each tuple is an otherwise valid prefix and one malformed beat.
        cases = ((0, {"first": False}), (0, {"last": True}),
                 (0, {"count": 3}), (0, {"count": 69}),
                 (0, {"offset": COMMAND_WINDOW + 4}), (0, {"sel": 7}),
                 (1, {"first": True}), (1, {"last": True}),
                 (1, {"count": 5}), (1, {"offset": COMMAND_WINDOW + 8}),
                 (2, {"data": JEDEC | (1 << 31)}),
                 (3, {"last": False}), (3, {"data": 2}), (3, {"sel": 3}))

        def bench():
            for bad_index, overrides in cases:
                for index in range(bad_index):
                    yield from self.word(dut, words, index)
                yield from self.word(dut, words, bad_index, error=1, **overrides)
                yield from self.assert_uncommitted(dut, flash)
            # Header requires two program words, but the record claims one.
            malformed = [1, 0x400000, PROGRAM | (5 << 8), 0x12345678, 1]
            yield from self.word(dut, malformed, 0)
            yield from self.word(dut, malformed, 1)
            yield from self.word(dut, malformed, 2, error=1)
            yield from self.assert_uncommitted(dut, flash)
            yield from self.start(dut, 1, JEDEC)
            yield from self.complete(dut, 1)

        run_simulation(dut, [bench(), flash.process()])

    def test_owner_survives_foreign_writes_aborts_and_interleaved_reads(self):
        dut, flash = self.fixture()
        payload = b"\x12\x34\x56\x78\x9a"
        words = command_words(1, PROGRAM, 0x400000, payload=payload)

        def bench():
            for index in range(4):
                yield from self.word(dut, words, index, source=0)
            yield from self.word(dut, command_words(1, JEDEC), 0, source=1, error=1)
            yield dut.record_abort_sources.eq(1 << 1)
            yield
            yield dut.record_abort_sources.eq(0)
            yield dut.record_abort.eq(1)
            yield from self.word(dut, words, 4, source=1, error=1)
            yield dut.record_abort.eq(0)
            yield from self.read(dut, ACCEPTED)
            yield from self.read_buffer(dut, 8)
            for _ in range(20):
                yield
            for index in range(4, len(words)):
                if index == len(words) - 1:
                    yield dut.record_abort_sources.eq(1 << 1)
                yield from self.word(dut, words, index, source=0)
            yield dut.record_abort_sources.eq(0)
            yield from self.complete(dut, 1)
            self.assertEqual([t for t in flash.transactions if t[0] == 0x02],
                             [b"\x02\x40\x00\x00" + payload])

        run_simulation(dut, [bench(), flash.process()])

    def test_source_qualified_abort_during_payload_byte_assembly(self):
        for foreign in (False, True):
            for abort_cycle in (2, 3, 4):
                with self.subTest(foreign=foreign, abort_cycle=abort_cycle):
                    dut, flash = self.fixture()
                    payload = b"\x12\x34\x56\x78\x9a\xbc\xde\xf0"
                    words = command_words(1, PROGRAM, 0x400000, payload=payload)

                    def bench():
                        for index in range(3):
                            yield from self.word(dut, words, index, source=1)
                        # Hold a payload word stable through response, but
                        # pulse the out-of-band abort during byte assembly.
                        yield dut.record_first.eq(0)
                        yield dut.record_last.eq(0)
                        yield dut.record_count.eq(len(words))
                        yield dut.record_source.eq(1)
                        yield dut.bus.adr.eq((COMMAND_WINDOW + 12) // 4)
                        yield dut.bus.dat_w.eq(words[3])
                        yield dut.bus.sel.eq(15)
                        yield dut.bus.we.eq(1)
                        yield dut.bus.cyc.eq(1)
                        yield dut.bus.stb.eq(1)
                        yield
                        for elapsed in range(1, 17):
                            if elapsed == abort_cycle:
                                yield dut.record_abort_sources.eq(1 << (2 if foreign else 1))
                            elif elapsed == abort_cycle + 1:
                                yield dut.record_abort_sources.eq(0)
                            ack, error = (yield dut.bus.ack), (yield dut.bus.err)
                            self.assertFalse(ack and error)
                            if ack or error:
                                self.assertGreater(elapsed, abort_cycle)
                                self.assertEqual(error, int(not foreign))
                                break
                            yield
                        else:
                            self.fail("payload word did not terminate within 16 cycles")
                        yield dut.record_abort_sources.eq(0)
                        yield dut.bus.cyc.eq(0)
                        yield dut.bus.stb.eq(0)
                        yield
                        self.assertEqual(flash.transactions, [])
                        if foreign:
                            for index in range(4, len(words)):
                                yield from self.word(dut, words, index, source=1)
                            expected = payload
                        else:
                            yield from self.word(dut, words, 4, source=1, error=1)
                            yield from self.assert_uncommitted(dut, flash)
                            expected = b"\xa5\x5a\x81"
                            yield from self.start(dut, 1, PROGRAM, 0x400000,
                                                  payload=expected, source=2)
                        yield from self.complete(dut, 1)
                        self.assertEqual([t for t in flash.transactions if t[0] == 0x02],
                                         [b"\x02\x40\x00\x00" + expected])
                        self.assertNotIn(0x400000 + len(expected), flash.memory)

                    run_simulation(dut, [bench(), flash.process()])

    def test_invalid_descriptor_is_not_consumed_before_matching_commit(self):
        dut, flash = self.fixture()
        words = command_words(1, 0x82)

        def bench():
            for index in range(3):
                yield from self.word(dut, words, index)
            yield from self.assert_uncommitted(dut, flash)
            yield dut.record_abort_sources.eq(1)
            yield
            yield dut.record_abort_sources.eq(0)
            yield from self.word(dut, words, 3, error=1)
            yield from self.assert_uncommitted(dut, flash)
            yield from self.packet(dut, words)
            yield from self.complete(dut, 1, error=5)
            self.assertEqual(flash.transactions, [])

        run_simulation(dut, [bench(), flash.process()])

    def test_unsigned_command_id_boundary_and_maximum_never_wrap(self):
        for previous_id in (0x7FFFFFFF, 0xFFFFFFFE):
            with self.subTest(previous_id=previous_id):
                dut, flash = self.fixture()

                def bench():
                    # Seed the public status registers instead of executing
                    # billions of records; every transition uses the ABI.
                    yield dut.accepted_id.eq(previous_id)
                    yield dut.done_id.eq(previous_id)
                    yield
                    command_id = previous_id + 1
                    yield from self.start(dut, command_id, JEDEC)
                    yield from self.complete(dut, command_id)
                    count = len(flash.transactions)
                    for ignored_id in (previous_id, command_id, 0):
                        yield from self.packet(dut, command_words(ignored_id, JEDEC))
                    self.assertEqual((yield from self.read(dut, ACCEPTED)), command_id)
                    self.assertEqual((yield from self.read(dut, DONE)), command_id)
                    self.assertEqual((yield from self.read(dut, RESULT)), 0xC86018)
                    self.assertEqual(len(flash.transactions), count)
                    if command_id == 0xFFFFFFFF:
                        yield from self.packet(dut, command_words(1, JEDEC))
                        self.assertEqual((yield from self.read(dut, ACCEPTED)), command_id)
                        self.assertEqual(len(flash.transactions), count)
                    else:
                        yield from self.start(dut, command_id + 1, JEDEC)
                        yield from self.complete(dut, command_id + 1)

                run_simulation(dut, [bench(), flash.process()])

    def test_slot_change_before_commit_revalidates_active_protection(self):
        slot = Signal(3, reset=1)
        dut, flash = self.fixture(boot_slot=slot)
        words = command_words(1, PROGRAM, 0x400000, payload=b"\x12\x34\x56\x78")

        def bench():
            for index in range(len(words) - 1):
                yield from self.word(dut, words, index)
            yield slot.eq(2)
            yield
            yield from self.word(dut, words, len(words) - 1)
            yield from self.complete(dut, 1, error=2)
            self.assertEqual(flash.transactions, [])
            yield from self.start(dut, 2, PROGRAM, 0x3FFFFF, payload=b"\xA5")
            yield from self.complete(dut, 2)
            self.assertEqual(flash.memory[0x3FFFFF], 0xA5)

        run_simulation(dut, [bench(), flash.process()])

    def test_maintenance_destructive_scope_for_all_runtime_and_target_slots(self):
        slot = Signal(3)
        dut, flash = self.fixture(boot_slot=slot, golden_maintenance=True)

        def bench():
            command_id = 0
            for running_slot in range(8):
                yield slot.eq(running_slot)
                yield
                for target_slot in range(8):
                    # Exercise each destructive operation at its last valid
                    # address in the slot, immediately before the boundary.
                    for opcode, tail, wire_opcode in ((PROGRAM, 1, 0x02),
                                                       (ERASE4K, 4096, 0x20),
                                                       (ERASE64K, 65536, 0xD8)):
                        with self.subTest(running=running_slot, target=target_slot,
                                          opcode=opcode):
                            command_id += 1
                            address = (target_slot + 1) * 0x200000 - tail
                            allowed = running_slot in (1, 2) and target_slot == 0
                            transaction_count = len(flash.transactions)
                            yield from self.start(dut, command_id, opcode, address,
                                                  payload=b"\x5a" if opcode == PROGRAM else b"")
                            yield from self.complete(dut, command_id, error=0 if allowed else 2)
                            transactions = flash.transactions[transaction_count:]
                            if allowed:
                                self.assertTrue(any(item[0] == wire_opcode for item in transactions))
                                if opcode == PROGRAM:
                                    self.assertEqual(flash.memory[address], 0x5A)
                            else:
                                self.assertEqual(transactions, [])

        run_simulation(dut, [bench(), flash.process()])

    def test_maintenance_rechecks_runtime_slot_at_commit(self):
        slot = Signal(3, reset=2)
        dut, flash = self.fixture(boot_slot=slot, golden_maintenance=True)

        def bench():
            for command_id, rejected_slot in enumerate((0, 3, 4, 5, 6, 7), 1):
                yield slot.eq(2)
                yield
                words = command_words(command_id, PROGRAM, 0, payload=b"\x5a")
                for index in range(len(words) - 1):
                    yield from self.word(dut, words, index)
                yield slot.eq(rejected_slot)
                yield
                yield from self.word(dut, words, len(words) - 1)
                yield from self.complete(dut, command_id, error=2)
                self.assertEqual(flash.transactions, [])
            # A rejected original descriptor cannot be made valid by changing
            # the identity before commit either.
            yield slot.eq(0)
            yield
            words = command_words(7, PROGRAM, 0, payload=b"\x5a")
            for index in range(len(words) - 1):
                yield from self.word(dut, words, index)
            yield slot.eq(2)
            yield
            yield from self.word(dut, words, len(words) - 1)
            yield from self.complete(dut, 7, error=2)
            self.assertEqual(flash.transactions, [])
            yield from self.start(dut, 8, PROGRAM, 0, payload=b"\x5a")
            yield from self.complete(dut, 8)
            self.assertEqual(flash.memory[0], 0x5A)

        run_simulation(dut, [bench(), flash.process()])

    def test_maintenance_reads_all_slots_even_when_writes_are_inhibited(self):
        dut, flash = self.fixture(boot_slot=0, golden_maintenance=True)

        def bench():
            for target_slot in range(8):
                address = target_slot * 0x200000
                flash.memory[address] = 0x70 + target_slot
                yield from self.start(dut, target_slot + 1, READ, address, 1)
                yield from self.complete(dut, target_slot + 1)
                self.assertEqual((yield from self.read_buffer(dut, 1)), bytes([0x70 + target_slot]))

        run_simulation(dut, [bench(), flash.process()])

    def test_normal_destructive_scope_is_unchanged(self):
        slot = Signal(3)
        dut, flash = self.fixture(boot_slot=slot)

        def bench():
            command_id = 0
            for running_slot in range(8):
                yield slot.eq(running_slot)
                yield
                for target_slot in range(8):
                    with self.subTest(running=running_slot, target=target_slot):
                        command_id += 1
                        allowed = target_slot != 0 and target_slot != running_slot
                        count = len(flash.transactions)
                        yield from self.start(dut, command_id, PROGRAM,
                                              target_slot * 0x200000, payload=b"\x5a")
                        yield from self.complete(dut, command_id, error=0 if allowed else 2)
                        if allowed:
                            self.assertEqual(flash.memory[target_slot * 0x200000], 0x5A)
                        else:
                            self.assertEqual(len(flash.transactions), count)

        run_simulation(dut, [bench(), flash.process()])

    def test_truncated_record_releases_owner_only_on_qualified_abort(self):
        dut, flash = self.fixture()
        words = command_words(1, PROGRAM, 0x400000, payload=b"\x12\x34\x56\x78\x9a")

        def bench():
            for index in range(4):
                yield from self.word(dut, words, index, source=1)
            for _ in range(50):
                yield
            yield from self.assert_uncommitted(dut, flash)
            yield dut.record_abort_sources.eq(1 << 1)
            yield
            yield dut.record_abort_sources.eq(0)
            yield
            yield from self.word(dut, words, 4, source=1, error=1)
            yield from self.assert_uncommitted(dut, flash)
            yield from self.start(dut, 1, JEDEC, source=0)
            yield from self.complete(dut, 1)

        run_simulation(dut, [bench(), flash.process()])

    def test_abort_wins_over_commit_but_does_not_cancel_accepted_spi(self):
        dut, flash = self.fixture()
        words = command_words(1, PROGRAM, 0x400000, payload=b"\x5a")

        def bench():
            for use_bitmap in (False, True):
                for index in range(len(words) - 1):
                    yield from self.word(dut, words, index, source=2)
                if use_bitmap:
                    yield dut.record_abort_sources.eq(1 << 2)
                else:
                    yield dut.record_abort.eq(1)
                yield from self.word(dut, words, len(words) - 1, source=2, error=1)
                yield dut.record_abort_sources.eq(0)
                yield dut.record_abort.eq(0)
                yield from self.assert_uncommitted(dut, flash)
            yield from self.packet(dut, words, source=2)
            self.assertTrue((yield dut.busy))
            yield dut.record_abort_sources.eq(1 << 2)
            yield dut.record_abort.eq(1)
            yield
            yield dut.record_abort_sources.eq(0)
            yield dut.record_abort.eq(0)
            yield from self.complete(dut, 1)
            self.assertEqual([t for t in flash.transactions if t[0] == 0x02],
                             [b"\x02\x40\x00\x00\x5a"])

        run_simulation(dut, [bench(), flash.process()])

    def test_short_reads_preserve_tail_under_continuous_cpu_buffer_reads(self):
        dut, flash = self.fixture()
        initial = bytes((index * 17 + 0xB3) & 255 for index in range(256))
        replacement = bytes((index * 37 + 0x28) & 255 for index in range(8))
        flash.memory.update({0x400000 + index: value for index, value in enumerate(initial)})
        flash.memory.update({0x400800 + index: value for index, value in enumerate(replacement)})
        latencies = []

        def bench():
            yield from self.start(dut, 1, READ, 0x400000, 256)
            yield from self.complete(dut, 1)
            previous = initial
            for command_id, length in enumerate((1, 2, 3, 5), 2):
                yield from self.start(dut, command_id, READ, 0x400800, length)
                seen_active = False
                for _ in range(5000):
                    if not (yield dut.busy):
                        break
                    value = yield from self.read(dut, BUFFER + 252, idle=False, limit=16,
                                                 latencies=latencies)
                    self.assertEqual(value, int.from_bytes(initial[-4:], "little"))
                    seen_active |= bool(flash.current and flash.current[0] == 0x03
                                        and not (yield dut.pads.cs_n))
                else:
                    self.fail("CPU reads prevented flash completion")
                self.assertTrue(seen_active)
                yield from self.complete(dut, command_id)
                previous = replacement[:length] + previous[length:]
                self.assertEqual((yield from self.read_buffer(dut)), previous)
            self.assertTrue(latencies)
            self.assertLessEqual(max(latencies), 16)

        run_simulation(dut, [bench(), flash.process()])

    def test_dynamic_slot_identity_and_inhibit_preserve_protection(self):
        slot = Signal(3, reset=1)
        dut, flash = self.fixture(boot_slot=slot)

        def bench():
            yield dut.inhibit.eq(1)
            yield from self.packet(dut, command_words(1, JEDEC))
            yield from self.assert_uncommitted(dut, flash)
            yield dut.inhibit.eq(0)
            yield from self.start(dut, 1, PROGRAM, 0x200000, payload=b"\x12")
            yield from self.complete(dut, 1, error=2)
            yield from self.start(dut, 2, PROGRAM, 0x400000, payload=b"\x34")
            yield dut.inhibit.eq(1)
            yield from self.start(dut, 2, PROGRAM, 0x400000, payload=b"\x99")
            yield from self.complete(dut, 2)
            self.assertEqual(flash.memory[0x400000], 0x34)
            yield slot.eq(2)
            yield
            self.assertEqual((yield from self.read(dut, 0x2C)), 2)
            yield dut.inhibit.eq(0)
            yield from self.start(dut, 3, PROGRAM, 0x400000, payload=b"\x56")
            yield from self.complete(dut, 3, error=2)
            yield from self.start(dut, 4, PROGRAM, 0x200000, payload=b"\x78")
            yield from self.complete(dut, 4)
            self.assertEqual(flash.memory[0x200000], 0x78)

        run_simulation(dut, [bench(), flash.process()])

    def test_wel_failure_and_wip_timeout_preserve_exact_completion_ids(self):
        dut, flash = self.fixture(refuse_wren=True, poll_timeout_cycles=1000)

        def bench():
            yield from self.start(dut, 1, ERASE4K, 0x400000)
            yield from self.complete(dut, 1, error=6)
            self.assertEqual([t[0] for t in flash.transactions], [0x05, 0x06, 0x05])
            flash.refuse_wren = False
            flash.stuck_busy = True
            count = len(flash.transactions)
            yield from self.start(dut, 2, PROGRAM, 0x400000, payload=b"\x12")
            yield from self.complete(dut, 2, error=7)
            self.assertTrue(all(t[0] == 0x05 for t in flash.transactions[count:]))
            self.assertFalse(flash.memory)
            flash.stuck_busy = False
            yield from self.start(dut, 3, PROGRAM, 0x400000, payload=b"\x34")
            yield from self.complete(dut, 3)
            self.assertEqual(flash.memory[0x400000], 0x34)

        run_simulation(dut, [bench(), flash.process()])


if __name__ == "__main__":
    unittest.main()
