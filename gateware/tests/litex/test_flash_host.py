"""Page-record flash commands, transport framing and application OTA.

The register/packet model is independent of FlashClient's serialization. No
test opens sockets, serial devices, or physical flash.
"""

import socket
import struct
import zlib
import unittest
from unittest.mock import Mock

from tools.lutcam.flash import (
    FLASH_BASE, FLASH_MAGIC, FLASH_SIZE, SLOT_SIZE, JEDEC_ID,
    READ, PROGRAM, ERASE_4K, ERASE_64K, JEDEC, STATUS,
    BOOT_BASE, BOOT_MAGIC, BootClient,
    EtherboneUDP, UARTBoneUSB, FlashClient, FlashError,
)
from tools.lutcam.update import Updater
from tests.litex.test_ota import Board, Clock, FakeBoot, FakeFlash


PACKET_ADDRESS = FLASH_BASE + 0x400


class PacketFlashBus:
    """Atomic packet ingress with strict IDs and independently decoded fields."""

    def __init__(self, *, memory=None, image_slot=1, negotiated_limit=None):
        self.memory = bytearray(b"\xff") * FLASH_SIZE if memory is None else memory
        self.registers = {
            0: FLASH_MAGIC, 4: 0, 8: 0, 0x0c: FLASH_SIZE,
            0x10: 0, 0x14: 0, 0x18: 0, 0x1c: 0, 0x20: 0, 0x24: 0,
            0x28: 0, 0x2c: image_slot, 0x30: 60_000_000, 0x34: 0,
            0x38: 256, 0x3c: 128, 0x40: 0x100,
        }
        self.registers.update({offset: 0 for offset in range(0x100, 0x200, 4)})
        self.max_words = 8
        self.negotiated_limit = negotiated_limit
        self.negotiated, self.reads, self.writes, self.commands = [], [], [], []
        self.rejected, self.ignored = [], []
        self.drop_packets = 0
        self.drop_replies = 0
        self.raise_after_delivery = False
        self.error = 0
        self.hang = False
        self.inhibit = False

    def configure_burst_limit(self, words):
        self.negotiated.append(words)
        self.max_words = words if self.negotiated_limit is None else self.negotiated_limit
        return self.max_words

    def read(self, address, length=None):
        assert length is None or 1 <= length <= self.max_words
        self.reads.append((address, length))
        if self.commands and self.drop_replies:
            self.drop_replies -= 1
            raise socket.timeout("lost completion response")
        offset = address - FLASH_BASE
        if length is None:
            return self.registers[offset]
        return [self.registers[offset + 4 * index] for index in range(length)]

    def write(self, address, words):
        # Every ABI 3 command must reach the ingress in one complete transport
        # packet. Descriptor/payload staging writes are forbidden by this model.
        assert address == PACKET_ADDRESS, f"non-packet flash write at {address:#x}"
        assert isinstance(words, (list, tuple))
        assert 4 <= len(words) <= min(68, self.max_words)
        words = tuple(words)
        self.writes.append((address, words))
        if self.drop_packets:
            self.drop_packets -= 1
            return
        self.deliver(words)
        if self.raise_after_delivery:
            self.raise_after_delivery = False
            raise socket.timeout("packet delivered but transport completion lost")

    def deliver(self, words):
        """Network delivery can duplicate, delay or reorder a whole packet."""
        words = tuple(words)
        if len(words) < 4 or len(words) > 68:
            self.rejected.append((words, "packet length"))
            return
        command_id, address, control = words[:3]
        previous = self.registers[0x1c]
        if not 0 <= command_id <= 0xffffffff or words[-1] != command_id:
            self.rejected.append((words, "commit framing"))
            return
        opcode, wire_length = control & 0xff, control >> 8
        descriptor_error = 0
        if opcode in (READ, PROGRAM):
            if not 1 <= wire_length <= 256:
                descriptor_error = 3
            elif address // 256 != (address + wire_length - 1) // 256:
                descriptor_error = 4
            length = wire_length
        elif opcode in (ERASE_4K, ERASE_64K, JEDEC, STATUS):
            length = {ERASE_4K: 4096, ERASE_64K: 65536, JEDEC: 3, STATUS: 1}[opcode]
            if wire_length:
                descriptor_error = 3
        else:
            descriptor_error, length = 5, 0
        payload_words = (length + 3) // 4 if opcode == PROGRAM else 0
        if len(words) != 4 + payload_words:
            self.rejected.append((words, "payload count"))
            return
        payload = b"".join(word.to_bytes(4, "little") for word in words[3:-1])
        if opcode == PROGRAM and payload[length:] != b"\xff" * (-length % 4):
            self.rejected.append((words, "payload padding"))
            return
        if command_id == previous and command_id != 0:
            return  # Latest complete replay remains inert, even while busy.
        if command_id == 0 or command_id != previous + 1:
            # Old, future and zero IDs are ACKed and drained without WBERR or
            # changing any command, result, buffer or flash state.
            self.ignored.append((words, "command ID"))
            return
        if self.registers[8] & 1 or self.inhibit:
            self.ignored.append((words, "busy or inhibited"))
            return
        if opcode in (READ, PROGRAM, ERASE_4K, ERASE_64K):
            if address < 0 or address + length > FLASH_SIZE:
                descriptor_error = 1
            if opcode != READ:
                active = self.registers[0x2c] * SLOT_SIZE
                if address < SLOT_SIZE or (address < active + SLOT_SIZE and address + length > active):
                    descriptor_error = 2
            if opcode in (ERASE_4K, ERASE_64K) and address % length:
                descriptor_error = 4
        self.commands.append((command_id, opcode, address, length))
        self.registers[0x1c], self.registers[8] = command_id, 1
        if self.hang:
            return
        error = descriptor_error or self.error
        if not error:
            if opcode == READ:
                data = bytes(self.memory[address:address + length]) + b"\xff" * (-length % 4)
                for offset in range(0, len(data), 4):
                    self.registers[0x100 + offset] = int.from_bytes(data[offset:offset + 4], "little")
            elif opcode == PROGRAM:
                for index, byte in enumerate(payload[:length]):
                    self.memory[address + index] &= byte
            elif opcode in (ERASE_4K, ERASE_64K):
                self.memory[address:address + length] = b"\xff" * length
            elif opcode == JEDEC:
                self.registers[0x28] = JEDEC_ID
            elif opcode == STATUS:
                self.registers[0x28] = 0
        self.registers[0x20], self.registers[0x24] = command_id, error
        self.registers[8] = 2 if error else 0


class PacketFlashTests(unittest.TestCase):
    def client(self, bus=None, **kwargs):
        bus = PacketFlashBus() if bus is None else bus
        clock = Clock()
        client = FlashClient(bus, timeout=.4, clock=clock, sleep=clock.sleep, **kwargs)
        return bus, client, clock

    def test_discovery_negotiates_full_packet_before_any_write(self):
        bus, flash, _ = self.client()
        self.assertEqual((flash.buffer_size, flash.buffer_offset,
                          flash.transfer_words), (256, 0x100, 128))
        self.assertEqual(bus.negotiated, [128])
        self.assertEqual(bus.writes, [])




    def test_every_opcode_is_one_self_contained_packet(self):
        cases = ((READ, 7, None, 7), (PROGRAM, 3, b"abc", 3),
                 (ERASE_4K, 4096, None, 0), (ERASE_64K, 65536, None, 0),
                 (JEDEC, 3, None, 0), (STATUS, 1, None, 0))
        for opcode, length, data, wire_length in cases:
            with self.subTest(opcode=opcode):
                bus, flash, _ = self.client()
                flash.command(opcode, 2 * SLOT_SIZE, length, data)
                expected = [1, 2 * SLOT_SIZE, opcode | (wire_length << 8)]
                if data is not None:
                    expected.append(0xff636261)
                expected.append(1)
                self.assertEqual(bus.writes, [(PACKET_ADDRESS, tuple(expected))])
                self.assertEqual(bus.commands, [(1, opcode, 2 * SLOT_SIZE, length)])
                self.assertEqual(bus.rejected, [])

    def test_full_page_is_exactly_68_words_and_little_endian_payload(self):
        bus, flash, _ = self.client()
        data = bytes(range(256))
        flash.program(2 * SLOT_SIZE, data, check_erased=False, verify=False)
        packet = bus.writes[0][1]
        self.assertEqual(len(packet), 68)
        self.assertEqual(packet[:4], (1, 2 * SLOT_SIZE, PROGRAM | (256 << 8), 0x03020100))
        self.assertEqual(packet[-2:], (0xfffefdfc, 1))
        self.assertEqual(bus.memory[2 * SLOT_SIZE:2 * SLOT_SIZE + 256], data)

    def test_partial_program_words_are_padded_with_ff(self):
        for length in (1, 2, 3, 5, 254, 255):
            with self.subTest(length=length):
                bus, flash, _ = self.client()
                payload = bytes(index & 255 for index in range(length))
                flash.command(PROGRAM, 2 * SLOT_SIZE, length, payload)
                words = bus.writes[0][1]
                self.assertEqual(len(words), 4 + (length + 3) // 4)
                packed = b"".join(value.to_bytes(4, "little") for value in words[3:-1])
                self.assertEqual(packed, payload + b"\xff" * (-length % 4))

    def test_read_and_program_split_at_each_physical_page_boundary(self):
        bus, flash, _ = self.client()
        address = 2 * SLOT_SIZE + 251
        data = bytes((index * 37 + 11) & 255 for index in range(520))
        flash.program(address, data, check_erased=False, verify=False)
        self.assertEqual(flash.read(address, len(data)), data)
        expected = [(address, 5), (address + 5, 256), (address + 261, 256), (address + 517, 3)]
        for opcode in (READ, PROGRAM):
            self.assertEqual([(entry[2], entry[3]) for entry in bus.commands if entry[1] == opcode], expected)
        self.assertTrue(all(len(words) <= 68 for _, words in bus.writes))
        self.assertEqual(bus.rejected, [])

    def test_direct_cross_page_commands_fail_before_any_mutation(self):
        for opcode in (READ, PROGRAM):
            for offset, length in ((255, 2), (1, 256), (0, 257), (0, 0)):
                with self.subTest(opcode=opcode, offset=offset, length=length):
                    bus, flash, _ = self.client()
                    with self.assertRaises((FlashError, ValueError)):
                        flash.command(opcode, 2 * SLOT_SIZE + offset, length,
                                      bytes(length) if opcode == PROGRAM else None)
                    self.assertEqual(bus.writes, [])

    def test_lost_whole_packet_retries_identical_id_descriptor_and_payload(self):
        bus, flash, _ = self.client()
        bus.drop_packets = 1
        data = bytes(range(251))
        flash.command(PROGRAM, 2 * SLOT_SIZE + 5, len(data), data)
        self.assertEqual(len(bus.writes), 2)
        self.assertEqual(bus.writes[0], bus.writes[1])
        self.assertEqual(len(bus.commands), 1)
        self.assertEqual(bus.memory[2 * SLOT_SIZE + 5:2 * SLOT_SIZE + 256], data)

    def test_lost_completion_reply_and_send_result_do_not_reexecute(self):
        for failure in ("drop_replies", "raise_after_delivery"):
            with self.subTest(failure=failure):
                bus, flash, _ = self.client()
                setattr(bus, failure, 1)
                self.assertEqual(flash.jedec(), JEDEC_ID)
                self.assertEqual(len(bus.writes), 1)
                self.assertEqual(len(bus.commands), 1)
                self.assertFalse(flash.uncertain)

    def test_strict_next_id_and_last_id_replay_are_safe_under_reordering(self):
        bus, flash, _ = self.client()
        flash.erase(2 * SLOT_SIZE, 4096)
        erase_packet = bus.writes[-1][1]
        flash.program(2 * SLOT_SIZE, b"new", check_erased=False, verify=False)
        program_packet = bus.writes[-1][1]
        # Nonzero sentinels make accidental clobbering of result/error/read
        # buffer visible, independently of the protected flash contents.
        bus.registers.update({0x24: 6, 0x28: 0x12345678, 0x100: 0x89abcdef})
        before = bus.registers.copy()
        bus.deliver(program_packet)  # Exact last command replay: no-op.
        bus.deliver(erase_packet)  # Late stale erase must never execute.
        future = (4, 2 * SLOT_SIZE, JEDEC, 4)
        bus.deliver(future)  # Missing ID 3: a future packet cannot skip it.
        bus.deliver((0, 2 * SLOT_SIZE, ERASE_4K, 0))
        self.assertEqual(len(bus.commands), 2)
        self.assertEqual(bus.memory[2 * SLOT_SIZE:2 * SLOT_SIZE + 3], b"new")
        self.assertEqual(bus.registers, before)
        self.assertEqual(bus.rejected, [])
        self.assertEqual(len(bus.ignored), 3)
        self.assertEqual(flash.jedec(), JEDEC_ID)
        self.assertEqual(bus.commands[-1][0], 3)

    def test_truncated_or_mismatched_commit_packets_do_not_execute(self):
        bus = PacketFlashBus()
        for packet in ((1, 2 * SLOT_SIZE, PROGRAM | (4 << 8), 0x11223344),
                       (1, 2 * SLOT_SIZE, JEDEC, 2),
                       (1, 2 * SLOT_SIZE, JEDEC, 0x11223344, 1)):
            bus.deliver(packet)
        self.assertEqual(bus.commands, [])
        self.assertEqual(bus.registers[0x1c], 0)
        self.assertEqual(len(bus.rejected), 3)

    def test_uses_current_accepted_id_and_refuses_exhaustion(self):
        bus, flash, _ = self.client()
        bus.registers[0x1c] = bus.registers[0x20] = 41
        flash.jedec()
        self.assertEqual(bus.writes[0][1][0], 42)
        self.assertEqual(bus.writes[0][1][-1], 42)
        bus.registers[0x1c] = 0xffffffff
        with self.assertRaisesRegex(FlashError, "exhausted"):
            flash.jedec()
        self.assertEqual(len(bus.writes), 1)

    def test_hung_command_locks_client_without_issuing_new_id(self):
        bus, flash, _ = self.client()
        bus.hang = True
        with self.assertRaisesRegex(FlashError, "uncertain"):
            flash.jedec()
        self.assertTrue(flash.uncertain)
        with self.assertRaisesRegex(FlashError, "previous command outcome"):
            flash.jedec()
        self.assertEqual(len(bus.writes), 1)

    def test_initial_busy_controller_never_receives_a_packet(self):
        bus, flash, _ = self.client()
        bus.registers[8] = 1
        with self.assertRaisesRegex(FlashError, "busy"):
            flash.jedec()
        self.assertEqual(bus.writes, [])

    def test_last_id_replay_and_new_id_are_inert_while_busy_or_inhibited(self):
        for state in ("busy", "inhibited"):
            with self.subTest(state=state):
                bus = PacketFlashBus()
                bus.hang = state == "busy"
                packet = (1, 0, JEDEC, 1)
                bus.deliver(packet)
                bus.inhibit = state == "inhibited"
                bus.registers.update({0x24: 6, 0x28: 0x12345678, 0x100: 0x89abcdef})
                before = bus.registers.copy()
                before_flash = bytes(bus.memory[2 * SLOT_SIZE:2 * SLOT_SIZE + 4])
                bus.deliver(packet)
                bus.deliver((2, 2 * SLOT_SIZE, PROGRAM | (4 << 8), 0x11223344, 2))
                self.assertEqual(bus.commands, [(1, JEDEC, 0, 3)])
                self.assertEqual(bus.registers, before)
                self.assertEqual(bytes(bus.memory[2 * SLOT_SIZE:2 * SLOT_SIZE + 4]), before_flash)
                self.assertEqual(bus.rejected, [])
                self.assertEqual(len(bus.ignored), 1)
                self.assertEqual(bus.ignored[-1][1], "busy or inhibited")

    def test_valid_next_packet_with_bad_descriptor_consumes_id_and_reports_error(self):
        bus = PacketFlashBus()
        bus.deliver((1, 2 * SLOT_SIZE + 255, READ | (2 << 8), 1))
        self.assertEqual((bus.registers[0x1c], bus.registers[0x20], bus.registers[0x24]), (1, 1, 4))
        bus.deliver((2, 0, JEDEC, 2))
        self.assertEqual((bus.registers[0x1c], bus.registers[0x20], bus.registers[0x28]), (2, 2, JEDEC_ID))

    def test_torn_completion_waits_for_matching_done_before_using_result(self):
        bus, flash, clock = self.client()
        original_read, polls = bus.read, []
        def read(address, length=None):
            result = original_read(address, length)
            if (address, length) == (FLASH_BASE + 8, 9):
                polls.append(result.copy())
                if len(polls) == 1:
                    result[6], result[8] = 0, 0xdeadbeef
            return result
        bus.read = read
        self.assertEqual(flash.jedec(), JEDEC_ID)
        self.assertEqual(len(polls), 2)
        self.assertTrue(clock.sleeps)
        self.assertEqual(len(bus.writes), 1)

    def test_concurrent_acceptance_or_final_owner_change_is_uncertain(self):
        for stage in ("completion", "final"):
            with self.subTest(stage=stage):
                bus, flash, _ = self.client()
                original_read, completion_seen = bus.read, []
                def read(address, length=None):
                    result = original_read(address, length)
                    if (address, length) == (FLASH_BASE + 8, 9):
                        completion_seen.append(True)
                        if stage == "completion":
                            result[5] += 1
                    elif completion_seen and (address, length) == (FLASH_BASE + 0x1c, None):
                        return result + 1
                    return result
                bus.read = read
                with self.assertRaisesRegex(FlashError, "changed"):
                    flash.jedec()
                self.assertTrue(flash.uncertain)
                self.assertEqual(len(bus.writes), 1)

    def test_nonprogram_payloads_are_rejected_before_submission(self):
        for opcode, length in ((READ, 1), (ERASE_4K, 4096), (ERASE_64K, 65536),
                               (JEDEC, 3), (STATUS, 1)):
            with self.subTest(opcode=opcode):
                bus, flash, _ = self.client()
                with self.assertRaises(ValueError):
                    flash.command(opcode, 2 * SLOT_SIZE, length, b"x")
                self.assertEqual(bus.writes, [])

    def test_completion_error_is_reported_without_payload_retry(self):
        bus, flash, _ = self.client()
        bus.error = 6
        with self.assertRaisesRegex(FlashError, "write enable failed"):
            flash.command(PROGRAM, 2 * SLOT_SIZE, 3, b"abc")
        self.assertEqual(len(bus.writes), 1)
        self.assertEqual(bus.memory[2 * SLOT_SIZE:2 * SLOT_SIZE + 3], b"\xff" * 3)

    def test_invalid_packet_capabilities_fail_without_flash_writes(self):
        for offset, value in ((0x38, 252), (0x38, 512), (0x40, 0x400),
                              (0x40, 0x102), (0x3c, 0), (0x3c, 67), (0x3c, 129)):
            with self.subTest(offset=offset, value=value):
                bus = PacketFlashBus()
                bus.registers[offset] = value
                with self.assertRaises(FlashError):
                    FlashClient(bus)
                self.assertEqual(bus.writes, [])

    def test_transport_must_negotiate_at_least_68_words(self):
        for limit in (1, 8, 64, 67):
            with self.subTest(limit=limit):
                bus = PacketFlashBus(negotiated_limit=limit)
                with self.assertRaises(FlashError):
                    FlashClient(bus)
                self.assertEqual(bus.writes, [])
        bus = PacketFlashBus(negotiated_limit=68)
        _, flash, _ = self.client(bus)
        flash.command(PROGRAM, 2 * SLOT_SIZE, 256, bytes(range(256)))
        self.assertEqual(len(bus.writes[0][1]), 68)

    def test_nonnegotiating_custom_transport_cannot_issue_packet_commands(self):
        bus = PacketFlashBus()
        class Wrapper:
            read = bus.read
            write = bus.write
        with self.assertRaises(FlashError):
            FlashClient(Wrapper())
        self.assertEqual(bus.writes, [])

    def test_golden_and_running_slot_remain_protected(self):
        bus, flash, _ = self.client()
        for address in (0, SLOT_SIZE):
            with self.subTest(address=address):
                with self.assertRaises(FlashError):
                    flash.command(PROGRAM, address, 1, b"x")
                with self.assertRaises(FlashError):
                    flash.erase(address, 4096)
        self.assertEqual(bus.writes, [])


class MixedPacketFlashBus(PacketFlashBus):
    """Expose the optional atomic submission/completion transport capability."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.combined = []
        self.separate_reads = []

    def read(self, address, length=None):
        self.separate_reads.append((address, length))
        return super().read(address, length)

    def write_read(self, write_address, values, read_address, length=None):
        self.combined.append((write_address, tuple(values), read_address, length))
        self.write(write_address, values)
        return super().read(read_address, length)


class CombinedFlashCommandTests(unittest.TestCase):
    def client(self):
        bus, clock = MixedPacketFlashBus(), Clock()
        flash = FlashClient(bus, timeout=.4, clock=clock, sleep=clock.sleep)
        bus.reads.clear()
        bus.separate_reads.clear()
        return bus, flash

    def test_packet_first_submission_and_completion_share_one_transaction(self):
        bus, flash = self.client()
        data = bytes(range(256))
        flash.command(PROGRAM, 2 * SLOT_SIZE, len(data), data)
        self.assertEqual(len(bus.combined), 1)
        write_address, words, read_address, count = bus.combined[0]
        self.assertEqual((write_address, read_address, count), (PACKET_ADDRESS, FLASH_BASE + 8, 9))
        self.assertEqual(len(words), 68)
        self.assertEqual(bus.writes, [(write_address, words)])
        self.assertEqual(bus.separate_reads, [(FLASH_BASE + 8, 6), (FLASH_BASE + 0x1c, None)])
        self.assertEqual(bus.memory[2 * SLOT_SIZE:2 * SLOT_SIZE + 256], data)

    def test_lost_submission_resends_identical_whole_packet_with_combined_read(self):
        bus, flash = self.client()
        bus.drop_packets = 1
        flash.command(PROGRAM, 2 * SLOT_SIZE, 5, b"abcde")
        self.assertEqual(len(bus.combined), 2)
        self.assertEqual(bus.combined[0], bus.combined[1])
        self.assertEqual(len(bus.commands), 1)
        self.assertEqual(bus.memory[2 * SLOT_SIZE:2 * SLOT_SIZE + 5], b"abcde")

    def test_lost_combined_reply_uses_read_only_poll_without_reexecution(self):
        bus, flash = self.client()
        bus.drop_replies = 1
        self.assertEqual(flash.jedec(), JEDEC_ID)
        self.assertEqual(len(bus.combined), 1)
        self.assertEqual(len(bus.commands), 1)
        self.assertEqual(len(bus.writes), 1)
        self.assertIn((FLASH_BASE + 8, 9), bus.separate_reads)
        self.assertFalse(flash.uncertain)


    def test_custom_and_usb_transports_keep_separate_fallback(self):
        bus = PacketFlashBus()
        self.assertFalse(hasattr(bus, "write_read"))
        self.assertFalse(hasattr(UARTBoneUSB("test-only"), "write_read"))
        flash = FlashClient(bus)
        self.assertEqual(flash.jedec(), JEDEC_ID)
        self.assertEqual(len(bus.writes), 1)
        self.assertIn((FLASH_BASE + 8, 9), bus.reads)
        self.assertEqual(bus.reads[-1], (FLASH_BASE + 0x1c, None))


class MixedReplySocket:
    """Queue real encoded Etherbone replies, without any system socket."""

    def __init__(self, values, *, noise=False, drop=False):
        self.values, self.noise, self.drop = list(values), noise, drop
        self.sent, self.requests, self.queue = [], [], []
        self.received = 0

    def settimeout(self, value):
        self.timeout = value

    @staticmethod
    def response(tag, values):
        from litex.tools.remote.etherbone import EtherbonePacket, EtherboneRecord, EtherboneWrites
        record = EtherboneRecord(addr_size=4)
        record.writes = EtherboneWrites(addr_size=4, base_addr=tag, datas=iter(values))
        record.wcount = len(values)
        packet = EtherbonePacket(32)
        packet.records = [record]
        packet.encode()
        return packet.bytes

    def sendto(self, raw, peer):
        from litex.tools.remote.etherbone import EtherbonePacket
        self.sent.append((bytes(raw), peer))
        packet = EtherbonePacket(32, raw)
        packet.decode()
        self.requests.append(packet)
        if self.drop:
            return
        tag = packet.records[0].reads.base_ret_addr
        if self.noise:
            self.queue.extend([
                (self.response(tag, self.values), ("192.168.1.99", peer[1])),
                (self.response((tag - 1) & 0xffffffff, self.values), peer),
                (self.response(tag, self.values[:-1]), peer),
            ])
        self.queue.append((self.response(tag, self.values), peer))

    def recvfrom(self, maximum):
        self.received += 1
        if not self.queue:
            raise socket.timeout("combined reply lost")
        return self.queue.pop(0)


class PacketTransportTests(unittest.TestCase):
    def packet(self):
        return [17, 2 * SLOT_SIZE, PROGRAM | (256 << 8),
                *[int.from_bytes(bytes(range(index, index + 4)), "little")
                  for index in range(0, 256, 4)], 17]

    def test_usb_encodes_one_complete_68_word_uartbone_frame(self):
        bus = UARTBoneUSB("test-only", max_words=128)
        bus.port = Mock()
        bus.port.write.side_effect = len
        packet = self.packet()
        bus.write(PACKET_ADDRESS, packet)
        bus.port.write.assert_called_once()
        raw = bus.port.write.call_args.args[0]
        self.assertEqual(raw[:6], bytes([1, 68]) + (PACKET_ADDRESS // 4).to_bytes(4, "big"))
        self.assertEqual(len(raw), 278)
        self.assertEqual([int.from_bytes(raw[index:index + 4], "big")
                          for index in range(6, len(raw), 4)], packet)

    def test_etherbone_encodes_one_complete_record_in_one_datagram(self):
        from litex.tools.remote.etherbone import EtherbonePacket
        bus = EtherboneUDP()
        bus.configure_burst_limit(128)
        bus.socket = Mock()
        packet = self.packet()
        bus.write(PACKET_ADDRESS, packet)
        bus.socket.sendto.assert_called_once()
        raw, peer = bus.socket.sendto.call_args.args
        self.assertEqual(peer, ("192.168.1.50", 1234))
        decoded = EtherbonePacket(32, raw)
        decoded.decode()
        self.assertEqual(len(decoded.records), 1)
        record = decoded.records[0]
        self.assertEqual(record.wcount, 68)
        self.assertEqual(record.writes.base_addr, PACKET_ADDRESS)
        self.assertEqual(record.writes.get_datas(), packet)

    def test_mixed_68_write_9_read_record_preserves_return_tag_and_layout(self):
        bus = EtherboneUDP(attempts=3)
        bus.configure_burst_limit(128)
        bus.sequence = 0x1234
        values = list(range(9))
        sock = bus.socket = MixedReplySocket(values)
        packet = self.packet()
        self.assertEqual(bus.write_read(PACKET_ADDRESS, packet, FLASH_BASE + 8, length=9), values)
        self.assertEqual(len(sock.sent), 1)
        self.assertEqual(len(sock.requests[0].records), 1)
        record = sock.requests[0].records[0]
        self.assertEqual((record.wcount, record.rcount), (68, 9))
        self.assertEqual(record.writes.base_addr, PACKET_ADDRESS)
        self.assertEqual(record.writes.get_datas(), packet)
        self.assertEqual(record.reads.base_ret_addr, 0x1235)
        self.assertNotEqual(record.reads.base_ret_addr, record.writes.base_addr)
        self.assertEqual(record.reads.get_addrs(), [FLASH_BASE + 8 + 4 * index for index in range(9)])
        self.assertEqual(bus.read_counter, 1)

    def test_mixed_reply_filters_peer_tag_and_count_without_resending(self):
        bus = EtherboneUDP(attempts=3)
        bus.configure_burst_limit(128)
        values = [0x87654321 + index for index in range(9)]
        sock = bus.socket = MixedReplySocket(values, noise=True)
        self.assertEqual(bus.write_read(PACKET_ADDRESS, self.packet(), FLASH_BASE + 8, length=9), values)
        self.assertEqual((len(sock.sent), sock.received), (1, 4))

    def test_mixed_timeout_never_replays_mutation_despite_read_attempt_setting(self):
        bus = EtherboneUDP(attempts=3)
        bus.configure_burst_limit(128)
        sock = bus.socket = MixedReplySocket([], drop=True)
        with self.assertRaises(socket.timeout):
            bus.write_read(PACKET_ADDRESS, self.packet(), FLASH_BASE + 8, length=9)
        self.assertEqual(len(sock.sent), 1)

    def test_mixed_scalar_read_returns_scalar_and_fresh_sequence_per_call(self):
        bus = EtherboneUDP()
        bus.configure_burst_limit(128)
        sock = bus.socket = MixedReplySocket([0x89abcdef])
        bus.sequence = 0xfffffffe
        self.assertEqual(bus.write_read(PACKET_ADDRESS, [1, 0, JEDEC, 1], FLASH_BASE + 8), 0x89abcdef)
        self.assertEqual(bus.write_read(PACKET_ADDRESS, [1, 0, JEDEC, 1], FLASH_BASE + 8), 0x89abcdef)
        self.assertEqual([packet.records[0].reads.base_ret_addr for packet in sock.requests], [0xffffffff, 0])

    def test_mixed_record_limits_include_both_fifo_address_words(self):
        for writes, reads in ((129, 1), (1, 129), (128, 127), (128, 128), (0, 1), (1, 0)):
            with self.subTest(writes=writes, reads=reads):
                bus = EtherboneUDP()
                bus.configure_burst_limit(128)
                sock = bus.socket = MixedReplySocket([0] * reads)
                with self.assertRaises(ValueError):
                    bus.write_read(PACKET_ADDRESS, [0] * writes, FLASH_BASE + 8, length=reads)
                self.assertEqual(sock.sent, [])
        bus = EtherboneUDP()
        bus.configure_burst_limit(128)
        sock = bus.socket = MixedReplySocket([0] * 126)
        self.assertEqual(bus.write_read(PACKET_ADDRESS, [0] * 128, FLASH_BASE + 8, length=126), [0] * 126)
        self.assertEqual(len(sock.sent), 1)  # 128 + 126 + 2 = 256 FIFO words.

    def test_mixed_invalid_addresses_and_payload_words_fail_before_send(self):
        for write_address, words, read_address in (
            (PACKET_ADDRESS + 1, [1], FLASH_BASE),
            (PACKET_ADDRESS, [1], FLASH_BASE + 1),
            (PACKET_ADDRESS, [-1], FLASH_BASE),
            (PACKET_ADDRESS, [0x100000000], FLASH_BASE),
        ):
            with self.subTest(write_address=write_address, words=words, read_address=read_address):
                bus = EtherboneUDP()
                sock = bus.socket = MixedReplySocket([0])
                with self.assertRaises(ValueError):
                    bus.write_read(write_address, words, read_address)
                self.assertEqual(sock.sent, [])



class UnversionedControlDiscoveryTests(unittest.TestCase):
    def test_ordered_probe_validates_real_capabilities_and_caches_success(self):
        bus = EtherboneUDP()
        bus.read = Mock(side_effect=[[FLASH_MAGIC, 0], [256, 128, 0x100]])
        self.assertTrue(bus.supports_ordered_write_read())
        self.assertTrue(bus.supports_ordered_write_read())
        self.assertEqual(bus.read.call_args_list[0].args, (FLASH_BASE,))
        self.assertEqual(bus.read.call_args_list[1].args, (FLASH_BASE + 0x38,))
        self.assertEqual(bus.read.call_count, 2)

    def test_capability_probe_validates_transfer_limits(self):
        for capabilities in ([256, 67, 0x100],
                             [256, 129, 0x100], [256, 128, 0x400], [0, 0, 0]):
            with self.subTest(capabilities=capabilities):
                bus = EtherboneUDP()
                bus.read = Mock(side_effect=[[FLASH_MAGIC, 0], capabilities])
                self.assertFalse(bus.supports_ordered_write_read())

    def test_lost_zero_capability_response_does_not_enable_or_cache(self):
        bus = EtherboneUDP()
        bus.read = Mock(side_effect=[[FLASH_MAGIC, 0], TimeoutError("lost"),
                                     [FLASH_MAGIC, 0], [256, 68, 0x100]])
        with self.assertRaises(TimeoutError):
            bus.supports_ordered_write_read()
        self.assertTrue(bus.supports_ordered_write_read())
        self.assertEqual(bus.read.call_count, 4)

    def test_boot_checks_magic(self):
        bus = Mock()
        bus.read.return_value = BOOT_MAGIC
        BootClient(bus)
        bus.read.assert_called_once_with(BOOT_BASE)
        bus.write.assert_not_called()
        bus.read.return_value = 0
        with self.assertRaisesRegex(FlashError, "magic"):
            BootClient(bus)


class PacketUpdateTests(unittest.TestCase):


    def test_existing_a_installs_b_then_mirrors_and_boots_a(self):
        board, clock = Board(), Clock()
        connect = board.connect
        models, clients = [], []

        def connect_model():
            connection = connect()
            model = PacketFlashBus(memory=board.memory, image_slot=board.slot)
            model.memory = board.memory
            model.registers[0x2c] = board.slot
            connection.read, connection.write = model.read, model.write
            connection.configure_burst_limit = model.configure_burst_limit
            connection.model = model
            models.append((board.slot, model))
            return connection

        def flash_factory(connection):
            flash = FlashClient(connection, clock=clock, sleep=clock.sleep)
            clients.append(flash.image_slot)
            return flash

        report = Updater(board.manifest, board.images, connect_model, board.health,
                         reboot_timeout=1, clock=clock, sleep=clock.sleep,
                         flash_factory=flash_factory, boot_factory=FakeBoot).run()
        self.assertTrue(report["ok"])
        self.assertEqual(clients, [1, 2, 1])
        self.assertEqual(report["final_identity"]["image_slot"], 1)
        golden = board.images["golden"]
        self.assertEqual(board.memory[:len(golden)], golden)
        self.assertEqual(board.memory[len(golden):SLOT_SIZE], b"\xff" * (SLOT_SIZE - len(golden)))
        self.assertTrue(all(connection.closed for connection in board.connections))
        for name in ("A", "B"):
            entry = board.entries[name]
            self.assertEqual(bytes(board.memory[entry["address"]:entry["address"] + entry["length"]]), board.images[name])
            self.assertEqual(bytes(board.memory[entry["metadata_address"]:entry["metadata_address"] + 64]),
                             bytes.fromhex(entry["metadata_hex"]))
        golden_reads = [command for _, model in models for command in model.commands
                        if command[1] == READ and command[2] == 0]
        self.assertEqual(len(golden_reads), 1)
        self.assertEqual(report["golden_verification"]["sha256"], board.entries["golden"]["sha256"])
        for active_slot, model in models:
            mutations = [command for command in model.commands if command[1] in (PROGRAM, ERASE_4K, ERASE_64K)]
            if mutations:
                expected_slot = 2 if active_slot == 1 else 1
                self.assertTrue(all(command[2] // SLOT_SIZE == expected_slot for command in mutations))
            self.assertEqual(model.rejected, [])


if __name__ == "__main__":
    unittest.main()
