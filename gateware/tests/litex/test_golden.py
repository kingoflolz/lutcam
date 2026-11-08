"""Golden selection against an independent flash/manifest software model."""

import struct
import unittest
import zlib

from migen import Module
from migen.sim import passive, run_simulation

from fpga.litex.lutcam.golden import (
    FLASH_CSR_BASE, MANIFEST_COMMIT, MANIFEST_MAGIC, MANIFEST_SECTOR_SIZE,
    SLOT_A, SLOT_B, SLOT_SIZE, GoldenBootScanner,
)


def manifest(slot, payload, *, length=None, checksum=None, commit=MANIFEST_COMMIT,
             reserved=0, recorded_slot=None):
    header = (MANIFEST_MAGIC + struct.pack("<IIII", reserved,
        slot if recorded_slot is None else recorded_slot,
        len(payload) if length is None else length,
        zlib.crc32(payload) if checksum is None else checksum) + bytes(range(16)))
    assert len(header) == 40
    return header + struct.pack("<II", zlib.crc32(header), commit) + b"\xff" * 16


class GoldenBootScannerTests(unittest.TestCase):
    def exercise(self, images, *, records=None, ru_error=False, bus_error=False,
                 bus_hang=False, command_hang=False, flash_error=0,
                 poll_delay=2, accept_request=True, expected_slot=0,
                 payload_error=False, scanner_options=None, cycle_trace=None,
                 record_fault=None, initial_command_id=41):
        options = dict(startup_cycles=3, wb_timeout_cycles=12, flash_timeout_cycles=48)
        options.update(scanner_options or {})
        dut = GoldenBootScanner(**options)
        memory = {}
        for slot, image in images.items():
            base = SLOT_A if slot == 1 else SLOT_B
            memory.update(enumerate(image, base))
            record = (records or {}).get(slot, manifest(slot, image))
            memory.update(enumerate(record, base + SLOT_SIZE - MANIFEST_SECTOR_SIZE))
        # Records can deliberately exist without any associated image.
        for slot, record in (records or {}).items():
            base = SLOT_A if slot == 1 else SLOT_B
            memory.update(enumerate(record, base + SLOT_SIZE - MANIFEST_SECTOR_SIZE))
        registers = {0x1C: initial_command_id, 0x20: initial_command_id, 0x24: 0}
        buffer = bytes(256)
        commands = []
        writes = []
        buffer_reads = []
        completion_polls = 0
        record_words = []
        abort_events = []
        fault_injected = False
        result = {}

        @passive
        def abort_monitor():
            while True:
                if (yield dut.record_abort):
                    abort_events.append(tuple(record_words))
                    record_words.clear()
                yield

        @passive
        def flash():
            nonlocal buffer, completion_polls, fault_injected
            bus = dut.bus
            while True:
                yield bus.ack.eq(0)
                yield bus.err.eq(0)
                if not ((yield bus.cyc) and (yield bus.stb)):
                    yield
                    continue
                address = ((yield bus.adr) << 2) - FLASH_CSR_BASE
                write = (yield bus.we)
                data = (yield bus.dat_w)
                framing = ((yield dut.record_first), (yield dut.record_last),
                    (yield dut.record_count), (yield dut.record_source))
                self.assertEqual((yield bus.sel), 15)
                if bus_hang:
                    yield
                    continue
                word_fault = (record_fault is not None and not fault_injected and
                              write and address == 0x400 + record_fault[0] * 4)
                if word_fault:
                    fault_injected = True
                    if record_fault[1] == "hang":
                        while (yield bus.cyc) and (yield bus.stb):
                            yield
                        continue
                # Real controller responses are registered and buffer reads
                # incur an additional RAM cycle. This model also provides a
                # response gap while CYC can remain asserted across transfers.
                yield
                if address >= 0x100:
                    yield
                self.assertEqual(framing, ((yield dut.record_first),
                    (yield dut.record_last), (yield dut.record_count),
                    (yield dut.record_source)), "record framing changed before ACK")
                value = 0
                response_error = bus_error or (word_fault and record_fault[1] == "error")
                if not response_error:
                    if write:
                        writes.append((address, data))
                        self.assertIn(address, (0x400, 0x404, 0x408, 0x40C))
                        index = (address - 0x400) // 4
                        self.assertEqual(index, len(record_words))
                        self.assertEqual(framing, (index == 0, index == 3, 4, 2))
                        record_words.append(data)
                        if index == 3:
                            command_id, start, descriptor, commit = record_words
                            self.assertEqual(command_id, commit)
                            self.assertGreater(command_id, registers[0x1C])
                            self.assertEqual(descriptor & 255, 1, "golden must only read")
                            length = descriptor >> 8
                            self.assertGreaterEqual(length, 1)
                            self.assertLessEqual(length, 256)
                            self.assertLessEqual((start & 255) + length, 256)
                            commands.append((start, length, command_id))
                            buffer = bytes(memory.get(start + i, 0xFF)
                                           for i in range(length)).ljust(256, b"\xff")
                            completion_polls = poll_delay
                            registers[0x10] = start
                            registers[0x1C] = command_id
                            registers[0x24] = (7 if payload_error and
                                start in (SLOT_A, SLOT_B) else flash_error)
                            record_words.clear()
                    elif address == 0x20:
                        if not command_hang:
                            if completion_polls:
                                completion_polls -= 1
                            else:
                                registers[0x20] = registers[0x1C]
                        value = registers[0x20]
                    elif address >= 0x100:
                        buffer_reads.append((registers[0x10], address))
                        value = int.from_bytes(buffer[address - 0x100:address - 0xFC], "little")
                    else:
                        value = registers.get(address, 0)
                yield bus.dat_r.eq(value)
                yield bus.err.eq(bool(response_error))
                yield bus.ack.eq(not response_error)
                yield
                yield bus.ack.eq(0)
                yield bus.err.eq(0)
                yield

        def bench():
            yield dut.ru_error.eq(ru_error)
            for cycle in range(20000):
                if cycle_trace is not None:
                    signals = (dut.bus.adr, dut.bus.dat_w, dut.bus.cyc, dut.bus.stb,
                        dut.bus.we, dut.bus.sel, dut.busy, dut.done, dut.selected_slot,
                        dut.valid_a, dut.valid_b, dut.error, dut.error_a, dut.error_b,
                        dut.request_valid, dut.request_slot, dut.record_first,
                        dut.record_last, dut.record_count, dut.record_source,
                        dut.record_abort)
                    values = []
                    for signal in signals:
                        values.append((yield signal))
                    cycle_trace.append(tuple(values))
                if (yield dut.done):
                    break
                yield
            else:
                self.fail("golden did not finish within the simulation deadline")
            result.update({"slot": (yield dut.selected_slot),
                "valid_a": (yield dut.valid_a), "valid_b": (yield dut.valid_b),
                "error": (yield dut.error), "error_a": (yield dut.error_a),
                "error_b": (yield dut.error_b)})
            result["cycles"] = cycle
            self.assertEqual(result["slot"], expected_slot, (result, commands))
            self.assertEqual((yield dut.busy), 0)
            self.assertEqual((yield dut.request_valid), expected_slot != 0)
            self.assertEqual((yield dut.request_slot), expected_slot)
            # A valid boot request cannot be a one-cycle pulse that the remote
            # update controller could miss while synchronizing its handshake.
            for _ in range(7):
                self.assertEqual((yield dut.request_valid), expected_slot != 0)
                yield
            if accept_request and expected_slot:
                yield dut.request_ready.eq(1)
                yield
                yield
                self.assertEqual((yield dut.request_valid), 0)
                self.assertEqual((yield dut.done), 1)
                self.assertEqual((yield dut.selected_slot), expected_slot)

        run_simulation(dut, [bench(), flash(), abort_monitor()])
        result["buffer_reads"] = buffer_reads
        result["abort_events"] = abort_events
        return result, commands, writes

    def test_valid_a_wins_without_reading_b(self):
        payload = bytes((index * 19 + 13) & 255 for index in range(777))
        result, commands, _ = self.exercise({1: payload, 2: b"also good"}, expected_slot=1)
        self.assertEqual(result["valid_a"], 1)
        self.assertEqual(result["valid_b"], 0)
        self.assertEqual(result["error"], 0)
        self.assertEqual([(start, length) for start, length, _ in commands], [
            (SLOT_A + SLOT_SIZE - MANIFEST_SECTOR_SIZE, 64),
            (SLOT_A, 256), (SLOT_A + 256, 256), (SLOT_A + 512, 256),
            (SLOT_A + 768, 9)])
        self.assertEqual([command[2] for command in commands], list(range(42, 47)))

    def test_corrupt_a_image_falls_back_to_b(self):
        image = b"new gateware\x01\x00" * 22
        bad_crc = zlib.crc32(image) ^ 0x80000000
        result, commands, _ = self.exercise({1: image, 2: b"123456789"},
            records={1: manifest(1, image, checksum=bad_crc)}, expected_slot=2)
        self.assertEqual(result["valid_a"], 0)
        self.assertEqual(result["valid_b"], 1)
        self.assertEqual(result["error_a"], GoldenBootScanner.ERROR_IMAGE_CRC)
        self.assertEqual(result["error"], 0)
        self.assertEqual(commands[-1][:2], (SLOT_B, 9))

    def test_corrupt_header_does_not_read_payload(self):
        record = bytearray(manifest(1, b"abc"))
        record[28] ^= 0x80
        result, commands, _ = self.exercise({1: b"abc", 2: b"safe"},
            records={1: bytes(record)}, expected_slot=2)
        self.assertEqual(result["error_a"], GoldenBootScanner.ERROR_HEADER_CRC)
        self.assertNotIn(SLOT_A, [start for start, _, _ in commands])

    def test_host_bundle_record_is_accepted_by_gateware(self):
        from tools.lutcam.ota_bundle import make_record

        image = bytes((i * 31) & 255 for i in range(519))
        record = make_record(image, 1, "00112233445566778899aabbccddeeff")
        self.exercise({1: image}, records={1: record}, expected_slot=1)


    def test_build_id_corruption_fails_header_crc(self):
        record = bytearray(manifest(1, b"A"))
        record[24] ^= 1
        result, commands, _ = self.exercise({1: b"A", 2: b"B"},
            records={1: bytes(record)}, expected_slot=2)
        self.assertEqual(result["error_a"], GoldenBootScanner.ERROR_HEADER_CRC)
        self.assertNotIn(SLOT_A, [start for start, _, _ in commands])

    def test_partial_tail_bytes_are_not_included_in_crc(self):
        for length in (1, 2, 3, 4, 255, 256, 257):
            with self.subTest(length=length):
                self.exercise({1: bytes((i + 1) & 255 for i in range(length))}, expected_slot=1)

    def test_invalid_manifest_fields_try_b_without_reading_a_image(self):
        cases = {"empty": {"length": 0}, "too_long": {"length": SLOT_SIZE - 4095},
                 "wrong_slot": {"recorded_slot": 2},
                 "uncommitted": {"commit": 0xFFFFFFFF}}
        for name, kwargs in cases.items():
            with self.subTest(name=name):
                result, commands, _ = self.exercise({1: b"A", 2: b"B"},
                    records={1: manifest(1, b"A", **kwargs)}, expected_slot=2)
                self.assertEqual(result["error_a"], GoldenBootScanner.ERROR_MANIFEST)
                self.assertNotIn(SLOT_A, [start for start, _, _ in commands])

    def test_manifest_errors_take_priority_after_all_record_words_are_read(self):
        for offset in (0, 4, 12, 16, 44):
            with self.subTest(corrupt_field_offset=offset):
                record = bytearray(manifest(1, b"A"))
                record[offset + 3 if offset == 16 else offset] ^= 0x80
                record[40] ^= 1
                result, commands, _ = self.exercise({1: b"A", 2: b"B"},
                    records={1: bytes(record)}, expected_slot=2)
                self.assertEqual(result["error_a"], GoldenBootScanner.ERROR_MANIFEST)
                self.assertNotIn(SLOT_A, [start for start, _, _ in commands])
                address = SLOT_A + SLOT_SIZE - MANIFEST_SECTOR_SIZE
                self.assertEqual([offset for start, offset in result["buffer_reads"]
                                  if start == address], list(range(0x100, 0x140, 4)))

    def test_length_boundaries_validate_every_bit_before_narrowing(self):
        limit = SLOT_SIZE - MANIFEST_SECTOR_SIZE
        for length in (0, 1, limit, limit + 1, 1 << 21, 0x80000001, 0xFFFFFFFF):
            with self.subTest(length=length):
                result, commands, _ = self.exercise({1: b"A"}, payload_error=True,
                    records={1: manifest(1, b"A", length=length)})
                payload_commands = [(start, size) for start, size, _ in commands
                                    if start == SLOT_A]
                if 1 <= length <= limit:
                    # Stop after the first payload command with a simulated
                    # controller error, avoiding a multi-million-cycle test.
                    self.assertEqual(payload_commands, [(SLOT_A, min(length, 256))])
                    self.assertEqual(result["error_a"], GoldenBootScanner.ERROR_FLASH)
                else:
                    self.assertEqual(payload_commands, [])
                    self.assertEqual(result["error_a"], GoldenBootScanner.ERROR_MANIFEST)

    def test_shared_timer_resets_between_startup_and_flash_commands(self):
        timings = []
        # Cover startup both shorter and longer than the flash deadline. The
        # longer startup must not leak its counter value into the first poll.
        for startup in (0, 1, 2, 4096):
            with self.subTest(startup_cycles=startup):
                result, _, _ = self.exercise({1: b"A"}, expected_slot=1,
                    scanner_options={"startup_cycles": startup})
                timings.append(result["cycles"] - startup)
        self.assertEqual(timings, [timings[0]] * len(timings))

    def test_minimum_flash_deadlines_and_bus_timeout_remain_bounded(self):
        for deadline in (1, 2):
            with self.subTest(flash_timeout_cycles=deadline):
                result, _, _ = self.exercise({1: b"A"}, command_hang=True,
                    scanner_options={"startup_cycles": 15,
                                     "flash_timeout_cycles": deadline})
                self.assertEqual(result["error_a"], GoldenBootScanner.ERROR_FLASH_TIMEOUT)
                self.assertEqual(result["error_b"], GoldenBootScanner.ERROR_FLASH_TIMEOUT)
            with self.subTest(wb_timeout_cycles=deadline):
                result, _, _ = self.exercise({1: b"A"}, bus_hang=True,
                    scanner_options={"wb_timeout_cycles": deadline})
                self.assertEqual(result["error_a"], GoldenBootScanner.ERROR_BUS_TIMEOUT)
                self.assertEqual(result["error_b"], GoldenBootScanner.ERROR_BUS_TIMEOUT)

    def test_truncated_record_and_empty_flash_stay_in_recovery(self):
        for record in (b"", MANIFEST_MAGIC, manifest(1, b"A")[:43]):
            with self.subTest(record_length=len(record)):
                result, commands, _ = self.exercise({}, records={1: record})
                self.assertEqual(result["error_a"], GoldenBootScanner.ERROR_MANIFEST)
                self.assertEqual(result["error_b"], GoldenBootScanner.ERROR_MANIFEST)
                self.assertEqual(len(commands), 2)

    def test_remote_update_error_prevents_boot_loop_and_flash_access(self):
        result, commands, writes = self.exercise({1: b"good", 2: b"good"}, ru_error=True)
        self.assertEqual(result["error"], GoldenBootScanner.ERROR_REMOTE_UPDATE)
        self.assertEqual(commands, [])
        self.assertEqual(writes, [])

    def test_bus_error_and_missing_ack_are_bounded(self):
        for key, error in (("bus_error", GoldenBootScanner.ERROR_BUS),
                           ("bus_hang", GoldenBootScanner.ERROR_BUS_TIMEOUT)):
            with self.subTest(key=key):
                result, commands, _ = self.exercise({1: b"good"}, **{key: True})
                self.assertEqual(result["error_a"], error)
                self.assertEqual(result["error_b"], error)
                self.assertEqual(commands, [])

    def test_partial_command_error_or_timeout_aborts_record_before_fallback(self):
        for word in (1, 2, 3):
            for fault, error in (("error", GoldenBootScanner.ERROR_BUS),
                                 ("hang", GoldenBootScanner.ERROR_BUS_TIMEOUT)):
                with self.subTest(word=word, fault=fault):
                    result, commands, _ = self.exercise({1: b"A", 2: b"B"},
                        record_fault=(word, fault), expected_slot=2)
                    self.assertEqual(result["error_a"], error)
                    self.assertEqual(len(result["abort_events"]), 1)
                    self.assertEqual(len(result["abort_events"][0]), word)
                    self.assertEqual(commands[0][0],
                        SLOT_B + SLOT_SIZE - MANIFEST_SECTOR_SIZE)
                    # The incomplete A record consumed no command ID.
                    self.assertEqual(commands[0][2], 42)

    def test_full_width_command_ids_and_exhaustion_do_not_wrap(self):
        for previous in (0, 0x7FFFFFFF, 0xFFFFFFFD):
            with self.subTest(initial_command_id=previous):
                _, commands, _ = self.exercise({1: b"A"}, expected_slot=1,
                    initial_command_id=previous)
                self.assertEqual([command[2] for command in commands],
                                 [previous + 1, previous + 2])
        result, commands, writes = self.exercise({1: b"A"}, initial_command_id=0xFFFFFFFF)
        self.assertEqual(result["error_a"], GoldenBootScanner.ERROR_FLASH)
        self.assertEqual(result["error_b"], GoldenBootScanner.ERROR_FLASH)
        self.assertEqual(commands, [])
        self.assertEqual(writes, [])

    def test_packet_words_are_separate_classic_transfers(self):
        trace = []
        self.exercise({1: b"A"}, expected_slot=1, cycle_trace=trace)
        previous_write = None
        saw_gap = True
        for address, _, cyc, stb, we, *_ in trace:
            if not (cyc and stb):
                saw_gap = True
            elif we:
                if previous_write is not None and address != previous_write:
                    self.assertTrue(saw_gap, "packet word changed without a bus gap")
                previous_write = address
                saw_gap = False

    def test_flash_completion_timeout_and_error_are_bounded(self):
        for kwargs, error in (({"command_hang": True}, GoldenBootScanner.ERROR_FLASH_TIMEOUT),
                              ({"flash_error": 7}, GoldenBootScanner.ERROR_FLASH)):
            with self.subTest(kwargs=kwargs):
                result, _, _ = self.exercise({1: b"good"}, **kwargs)
                self.assertEqual(result["error_a"], error)
                self.assertEqual(result["error_b"], error)

    def test_bad_timeout_parameters_fail_early(self):
        for kwargs in ({"startup_cycles": -1}, {"wb_timeout_cycles": 0},
                       {"flash_timeout_cycles": 0}, {"csr_base": 3}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    GoldenBootScanner(**kwargs)

    def test_real_flash_controller_and_spi_pins_fall_back_to_b(self):
        # Exercise the exact controller handshake, registered RAM reads, and
        # on-wire SPI bytes rather than relying only on the CSR software model.
        from fpga.litex.lutcam.flash_packet import PacketFlashController
        from tests.litex.spi_nor_model import SpiNor

        dut = Module()
        dut.submodules.flash = flash = PacketFlashController(clk_freq=200, spi_clk_freq=25)
        dut.submodules.scanner = scanner = GoldenBootScanner(
            startup_cycles=0, wb_timeout_cycles=16, flash_timeout_cycles=20000)
        dut.comb += scanner.bus.connect(flash.bus)
        dut.comb += [flash.record_first.eq(scanner.record_first),
            flash.record_last.eq(scanner.record_last),
            flash.record_count.eq(scanner.record_count),
            flash.record_source.eq(scanner.record_source),
            flash.record_abort.eq(scanner.record_abort),
            flash.record_abort_sources.eq(scanner.record_abort << 2)]
        nor = SpiNor(flash.pads)
        a, b = b"damaged application", bytes((i * 73) & 255 for i in range(257))
        for slot, base, image, record in (
            (1, SLOT_A, a, manifest(1, a, checksum=zlib.crc32(a) ^ 1)),
            (2, SLOT_B, b, manifest(2, b)),
        ):
            nor.memory.update(enumerate(image, base))
            nor.memory.update(enumerate(record, base + SLOT_SIZE - MANIFEST_SECTOR_SIZE))

        def bench():
            for _ in range(40000):
                if (yield scanner.done):
                    break
                yield
            else:
                self.fail("pin-level golden/flash scan did not complete")
            self.assertEqual((yield scanner.selected_slot), 2)
            self.assertEqual((yield scanner.request_valid), 1)
            self.assertEqual((yield scanner.error_a), GoldenBootScanner.ERROR_IMAGE_CRC)
            self.assertEqual((yield scanner.valid_b), 1)
            self.assertEqual((yield flash.busy), 0)
            self.assertTrue(nor.transactions)
            self.assertEqual({transaction[0] for transaction in nor.transactions}, {0x03, 0x05})

        run_simulation(dut, [bench(), nor.process()])


if __name__ == "__main__":
    unittest.main()
