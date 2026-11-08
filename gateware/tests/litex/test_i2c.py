"""Host I2C command protocol tests using an independent register-map model."""

import unittest
from unittest.mock import patch

from tools.lutcam.control import BitBangI2C, I2CError, I2CNack, I2CTimeout
from tools.lutcam.i2c import CameraI2C, HardwareI2C


class EngineBus:
    """One shared command sequencer, mirrored through two camera apertures."""

    def __init__(self, *, ordered=True, feature=True):
        self.ordered, self.feature = ordered, feature
        self.events = []
        self.descriptor = 0
        self.tx = [0] * 4
        self.rx = [0] * 4
        self.status = 0
        self.current_id = 0
        self.done_id = 0
        self.next_id = 1
        self.result = 0
        self.capacity = 16
        self.magic = 0x49324301
        self.registers = {}
        self.commands = []
        self.pending_polls = 0
        self.commit_response_lost = False
        self.drop_commit = False
        self.wrong_current_id = False
        self.result_error = 0
        self.completed_override = None
        self.gpio_control = [0, 0]

    def supports_ordered_write_read(self):
        return self.ordered

    @staticmethod
    def decode(address):
        camera = (address - 0x30020000) // 0x1000
        if camera not in (0, 1):
            raise AssertionError(f"unexpected access {address:#x}")
        return camera, address & 0xfff

    def _value(self, camera, offset):
        if offset == 0:
            return self.gpio_control[camera]
        if offset == 4:
            return 3 | (0x100 if self.feature else 0)
        if not self.feature:
            raise AssertionError("unavailable engine received an engine-register access")
        values = {0x10: self.magic, 0x14: self.status, 0x18: self.descriptor,
                  0x1c: self.current_id ^ int(self.wrong_current_id),
                  0x20: self.done_id, 0x24: self.next_id,
                  0x28: self.result, 0x2c: self.capacity}
        if offset in values:
            return values[offset]
        if 0x40 <= offset < 0x50:
            return self.tx[(offset - 0x40) // 4]
        if 0x60 <= offset < 0x70:
            return self.rx[(offset - 0x60) // 4]
        raise AssertionError(f"unknown register {offset:#x}")

    def read(self, address, length=None):
        self.events.append(("read", address, length))
        camera, offset = self.decode(address)
        if offset == 0x14 and self.status & 1 and self.pending_polls >= 0:
            if self.pending_polls:
                self.pending_polls -= 1
            else:
                self.status = (self.status & ~1) | 2
                self.done_id = self.current_id
        values = [self._value(camera, offset + index * 4)
                  for index in range(1 if length is None else length)]
        return values[0] if length is None else values

    def write(self, address, values):
        values = [values] if type(values) is int else list(values)
        self.events.append(("write", address, values))
        camera, offset = self.decode(address)
        if offset == 0:
            if len(values) != 1:
                raise AssertionError("unexpected burst of GPIO writes")
            self.gpio_control[camera] = values[0]
        elif offset == 0x80:
            if self.drop_commit:
                return
            if self.status & 1:
                raise AssertionError("command submitted while engine busy")
            command_id, descriptor = values[:2]
            if command_id != self.next_id:
                raise AssertionError("unexpected command ID replay")
            count = ((descriptor >> 24) & 15) + 1
            register = (descriptor >> 8) & 0xffff
            device = descriptor & 0x7f
            read = bool(descriptor & 0x80)
            if (len(values) != 3 + (0 if read else (count + 3) // 4)
                    or values[-1] != command_id or descriptor >> 29
                    or register + count > 0x10000):
                raise AssertionError("incomplete or malformed command record")
            self.descriptor = descriptor
            self.tx = values[2:-1] + [0] * (4 - len(values[2:-1]))
            self.commands.append((camera, command_id, device, register, count, read,
                                  bool(descriptor & (1 << 28))))
            self.current_id = command_id
            self.next_id = (command_id + 1) & 0xffffffff
            self.status = 1 | (camera << 2) | (int(read) << 3)
            count_done = count if self.completed_override is None else self.completed_override
            self.result = self.result_error | (2 << 8) | (count_done << 16)
            if read:
                payload = bytes(self.registers.get((camera, device, register + index), 0)
                                for index in range(count)) + bytes(16 - count)
                self.rx = [int.from_bytes(payload[index:index + 4], "little")
                           for index in range(0, 16, 4)]
            elif not self.result_error:
                payload = b"".join(word.to_bytes(4, "little") for word in self.tx)
                for index, byte in enumerate(payload[:count]):
                    self.registers[(camera, device, register + index)] = byte
        else:
            raise AssertionError(f"unexpected write {offset:#x}")

    def write_read(self, address, values, read_address, length=None):
        self.events.append(("ordered", address, read_address, length))
        self.write(address, values)
        if address & 0xfff == 0x80 and self.commit_response_lost:
            self.commit_response_lost = False
            raise TimeoutError("injected lost commit acknowledgment")
        return self.read(read_address, length)


class HardwareI2CTests(unittest.TestCase):

    def test_engine_discovery_and_camera_selection(self):
        bus = EngineBus()
        bus.registers[(1, 0x10, 0x300a)] = 0x92
        bus.registers[(1, 0x10, 0x300b)] = 0x81
        sensor = CameraI2C(bus, 0x30021000)
        self.assertEqual(sensor.read_register(0x10, 0x300a, 2), b"\x92\x81")
        self.assertEqual(sensor.backend_name, "hardware")
        self.assertEqual(bus.commands, [(1, 1, 0x10, 0x300a, 2, True, False)])
        self.assertEqual(bus.events[:3], [("read", 0x30021004, None),
                                          ("read", 0x30021010, None),
                                          ("read", 0x3002102c, None)])
        records = [event[2] for event in bus.events if event[0] == "write"]
        self.assertEqual(records, [[1, 0x01300a90, 1]])

    def test_bad_engine_identity_never_falls_back_to_gpio(self):
        for attribute, value in (("magic", 0), ("capacity", 8)):
            with self.subTest(attribute=attribute):
                bus = EngineBus()
                setattr(bus, attribute, value)
                with self.assertRaises(I2CError):
                    CameraI2C(bus, 0x30020000).write_register(0x60, 0x3501, 20)
                self.assertFalse(any(event[0] == "write" for event in bus.events))

    def test_all_inputs_are_validated_before_discovery(self):
        bus = EngineBus()
        for constructor in (CameraI2C, HardwareI2C):
            with self.assertRaises(ValueError):
                constructor(bus, float(0x30020000))
        sensor = CameraI2C(bus, 0x30020000)
        for arguments in ((True, 0, 1), (0x80, 0, 1), (0x60, -1, 1),
                          (0x60, 0, 257), (0x60, 0xffff, 2), (0x60, 0, True)):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                sensor.read_register(*arguments)
        for arguments in ((0x80, 0, b"x"), (0x60, 0xffff, b"xx"),
                          (0x60, 0, b""), (0x60, 0, bytes(257)),
                          (0x60, 0, True), (0x60, 0, -1), (0x60, 0, 256)):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                sensor.write_register(*arguments)
        self.assertEqual(bus.events, [])

    def test_shared_sequential_ids_and_little_endian_payload(self):
        bus = EngineBus()
        left, right = (HardwareI2C(bus, 0x30020000 + camera * 0x1000)
                       for camera in (0, 1))
        left.write_register(0x60, 0x3500, b"\x01\x23\x45\x67\x89")
        right.write_register(0x10, 0x3500, 0xab)
        self.assertEqual(left.read_register(0x60, 0x3500, 5), b"\x01\x23\x45\x67\x89")
        self.assertEqual(right.read_register(0x10, 0x3500), b"\xab")
        self.assertEqual([command[1] for command in bus.commands], [1, 2, 3, 4])
        self.assertIn(("write", 0x30020080,
                       [1, 0x04350060, 0x67452301, 0x89, 1]), bus.events)
        writes = [event[1] & 0xfff for event in bus.events if event[0] == "write"]
        self.assertEqual(writes, [0x80] * 4)

    def test_large_range_is_split_at_sixteen_bytes_without_wrapping(self):
        bus = EngineBus()
        sensor = HardwareI2C(bus, 0x30020000)
        sensor.write_register(0x60, 0x4000, bytes(range(256)))
        self.assertEqual(sensor.read_register(0x60, 0x4000, 256), bytes(range(256)))
        self.assertEqual(len(bus.commands), 32)
        self.assertEqual([command[3] for command in bus.commands[:16]],
                         list(range(0x4000, 0x4100, 16)))
        self.assertTrue(all(command[4] == 16 for command in bus.commands))

    def test_every_payload_length_is_one_complete_bounded_record(self):
        bus = EngineBus()
        sensor = HardwareI2C(bus, 0x30020000)
        for count in range(1, 17):
            expected = bytes(range(count))
            sensor.write_register(0x60, 0x4000, expected)
            self.assertEqual(sensor.read_register(0x60, 0x4000, count), expected)
        records = [event[2] for event in bus.events if event[0] == "write"]
        for count, (write, read) in enumerate(zip(records[::2], records[1::2]), 1):
            self.assertEqual(len(write), 3 + (count + 3) // 4)
            self.assertEqual(len(read), 3)
            self.assertEqual(write[0], write[-1])
            self.assertEqual(read[0], read[-1])
            self.assertEqual(b"".join(word.to_bytes(4, "little") for word in write[2:-1])[:count],
                             bytes(range(count)))

    def test_real_etherbone_encoding_keeps_command_in_one_mixed_record(self):
        from litex.tools.remote.etherbone import EtherbonePacket
        from tools.lutcam.flash import EtherboneUDP
        from tools.lutcam.live import OrderedManagementBus

        model = EngineBus()
        records = []

        class EncodedLoopback(EtherboneUDP):
            def _send(self, packet):
                packet.encode()
                decoded = EtherbonePacket(32, packet.bytes)
                decoded.decode()
                if len(decoded.records) != 1:
                    raise AssertionError("camera command needs exactly one Etherbone record")
                record = decoded.records[0]
                records.append(record)
                if record.writes is not None:
                    model.write(record.writes.base_addr, record.writes.get_datas())
                addresses = record.reads.get_addrs()
                if addresses != list(range(addresses[0], addresses[0] + 4 * len(addresses), 4)):
                    raise AssertionError("unexpected noncontiguous camera status request")
                self.response = model.read(addresses[0], len(addresses))

            def _receive_words(self, address, count):
                if len(self.response) != count:
                    raise AssertionError("Etherbone reply has the wrong length")
                return self.response

        raw = EncodedLoopback()
        raw._ordered_write_read = True
        sensor = CameraI2C(OrderedManagementBus(raw, require_ordered=True), 0x30021000)
        sensor.write_register(0x10, 0x4000, bytes(range(16)))
        self.assertEqual(sensor.read_register(0x10, 0x4000, 16), bytes(range(16)))
        commands = [record for record in records if record.writes is not None]
        self.assertEqual([record.wcount for record in commands], [7, 3])
        self.assertTrue(all(record.rcount == 6 for record in commands))
        self.assertTrue(all(record.writes.base_addr == 0x30021080 for record in commands))

    def test_busy_engine_is_not_modified_or_recovered(self):
        for operation in (lambda sensor: sensor.write_register(0x60, 0x3501, 40),
                          lambda sensor: sensor.release(), lambda sensor: sensor.recover()):
            with self.subTest(operation=operation):
                bus = EngineBus()
                bus.status, bus.pending_polls = 1, -1
                with self.assertRaisesRegex(I2CError, "busy"):
                    operation(HardwareI2C(bus, 0x30020000))
                self.assertFalse(any(event[0] == "write" for event in bus.events))

    def test_wrong_current_id_cannot_confirm_another_command(self):
        bus = EngineBus()
        bus.wrong_current_id = True
        with self.assertRaisesRegex(I2CError, "not accepted"):
            HardwareI2C(bus, 0x30020000).write_register(0x60, 0x3501, 42)
        self.assertEqual(len(bus.commands), 1)

    def test_lost_commit_ack_resolves_result_without_replaying_mutation(self):
        bus = EngineBus()
        bus.commit_response_lost = True
        HardwareI2C(bus, 0x30020000).write_register(0x60, 0x3208, 0xa0)
        commits = [event for event in bus.events
                   if event[0] == "write" and event[1] & 0xfff == 0x80]
        self.assertEqual(len(commits), 1)
        self.assertEqual(len(bus.commands), 1)

    def test_unaccepted_commit_is_not_replayed(self):
        bus = EngineBus()
        bus.commit_response_lost = bus.drop_commit = True
        with self.assertRaisesRegex(I2CError, "not accepted"):
            HardwareI2C(bus, 0x30020000).write_register(0x60, 0x3208, 0xa0)
        self.assertEqual(bus.commands, [])
        self.assertEqual(sum(event[0] == "write" and event[1] & 0xfff == 0x80
                             for event in bus.events), 1)

    def test_full_transport_timeout_still_gets_one_readonly_resolution(self):
        class ElapsedBus(EngineBus):
            now = 0.0

            def write_read(self, address, values, read_address, length=None):
                try:
                    return super().write_read(address, values, read_address, length)
                except TimeoutError:
                    self.now += 0.2
                    raise

        bus = ElapsedBus()
        bus.commit_response_lost = True
        HardwareI2C(bus, 0x30020000, clock=lambda: bus.now).write_register(0x60, 0x3208, 0xa0)
        self.assertEqual(len(bus.commands), 1)
        commit_index = next(index for index, event in enumerate(bus.events)
                            if event[0] == "write" and event[1] & 0xfff == 0x80)
        self.assertEqual(bus.events[commit_index + 1:], [("read", 0x30020014, 6)])

    def test_bounded_busy_polling_times_out_without_retry_or_recovery(self):
        bus = EngineBus()
        bus.pending_polls = -1
        ticks = iter(index / 100 for index in range(100))
        sensor = HardwareI2C(bus, 0x30020000, timeout=0.03, clock=lambda: next(ticks))
        with self.assertRaisesRegex(I2CTimeout, "outcome is uncertain"):
            sensor.write_register(0x60, 0x3208, 0xa0)
        self.assertEqual(len(bus.commands), 1)
        self.assertFalse(any(event[0] == "write" and event[1] & 0xfff == 0
                             for event in bus.events))

    def test_hardware_errors_have_existing_exception_types_and_no_retry(self):
        for code, exception in ((1, I2CNack), (2, I2CTimeout), (3, I2CError), (255, I2CError)):
            with self.subTest(code=code):
                bus = EngineBus()
                bus.result_error = code
                bus.completed_override = 0
                with self.assertRaises(exception):
                    HardwareI2C(bus, 0x30020000).read_register(0x60, 0x300a, 2)
                self.assertEqual(len(bus.commands), 1)

    def test_incomplete_success_is_an_error(self):
        bus = EngineBus()
        bus.completed_override = 1
        with self.assertRaisesRegex(I2CError, "invalid I2C completion"):
            HardwareI2C(bus, 0x30020000).read_register(0x60, 0x300a, 2)

    def test_usb_uses_write_then_read_fences_and_explicit_slow_speed(self):
        bus = EngineBus(ordered=False)
        HardwareI2C(bus, 0x30020000, slow=True).write_register(0x60, 0x3501, 42)
        self.assertFalse(any(event[0] == "ordered" for event in bus.events))
        self.assertTrue(bus.commands[0][-1])
        for index, event in enumerate(bus.events):
            if event[0] == "write":
                self.assertEqual(bus.events[index + 1][0], "read")

    def test_command_id_wrap_is_valid(self):
        bus = EngineBus()
        bus.next_id = 0xffffffff
        sensor = HardwareI2C(bus, 0x30020000)
        sensor.read_register(0x60, 0x300a)
        sensor.read_register(0x60, 0x300a)
        self.assertEqual([command[1] for command in bus.commands], [0xffffffff, 0])


if __name__ == "__main__":
    unittest.main()
