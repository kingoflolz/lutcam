"""OTA host protocol and interrupted-update tests; no hardware or sockets."""

from contextlib import contextmanager, redirect_stdout
import errno
import hashlib
import io
import json
import socket
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from tools.lutcam import flash as flash_module
from tools.lutcam.flash import (BOOT_BASE, BOOT_MAGIC, FLASH_BASE, FLASH_MAGIC, FLASH_SIZE,
    IMAGE_CAPACITY, SLOT_SIZE, JEDEC_ID, READ, PROGRAM, ERASE_4K, ERASE_64K, JEDEC, STATUS,
    BootClient, EtherboneUDP, FlashClient, FlashError, UARTBoneUSB)
from tools.lutcam.ota_bundle import COMMIT_OFFSET, make_record
from tools.lutcam.update import Updater, check_health


class Clock:
    def __init__(self):
        self.now = 0
        self.sleeps = []

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += max(seconds, 0.001)






class BootProtocolTests(unittest.TestCase):
    def test_unacknowledged_trigger_performs_no_later_bus_io_or_replay(self):
        class Bus:
            def __init__(self):
                self.values = {0: BOOT_MAGIC, 4: 1, 0x0C: 0, 0x20: 0, 0x24: 4, 0x28: 0}
                self.triggers = []

            def read(self, address):
                if self.triggers:
                    raise AssertionError("USB I/O after trigger would close the retained descriptor")
                return self.values[address-BOOT_BASE]

            def write(self, address, value):
                if self.triggers:
                    raise AssertionError("trigger was replayed")
                self.values[address-BOOT_BASE] = value
                if address == BOOT_BASE+0x24:
                    self.triggers.append(value)

        bus = Bus()
        result = BootClient(bus).request(2, acknowledge=False)
        self.assertEqual(result, {"request_id": 10, "acknowledged": False})
        self.assertEqual(bus.triggers, [10])

    def test_trigger_is_not_retried_after_timeout(self):
        class Bus:
            def __init__(self):
                self.values = {0: BOOT_MAGIC, 4: 1, 0x0C: 0, 0x20: 0, 0x24: 4, 0x28: 0}
                self.triggers = []

            def read(self, address):
                if self.triggers:
                    raise socket.timeout()
                return self.values[address - BOOT_BASE]

            def write(self, address, value):
                self.values[address - BOOT_BASE] = value
                if address == BOOT_BASE + 0x24:
                    self.triggers.append(value)
        bus = Bus()
        result = BootClient(bus).request(2)
        self.assertFalse(result["acknowledged"])
        self.assertEqual(bus.triggers, [10])

    def test_read_crossing_reboot_is_ambiguous_and_never_replayed(self):
        class Bus:
            def __init__(self):
                self.values = {0: BOOT_MAGIC, 4: 1, 0x0C: 0, 0x20: 0, 0x24: 0, 0x28: 0}
                self.triggers = []

            def read(self, address):
                return self.values[address - BOOT_BASE]

            def write(self, address, value):
                if address == BOOT_BASE + 0x24:
                    self.triggers.append(value)
                    self.values[0x24] = 0  # Already reset in the next image.
                else:
                    self.values[address - BOOT_BASE] = value
        bus = Bus()
        result = BootClient(bus).request(2)
        self.assertFalse(result["acknowledged"])
        self.assertEqual(bus.triggers, [6])


class TransportTests(unittest.TestCase):
    def usb_reboot_fixture(self):
        old = {"node": "/dev/ttyACM0", "node_identity": (1, 2, 3),
               "usb_path": "/sys/devices/usb5/5-2", "generation": (40, 5, 9)}
        # Even the same tty path/inode is insufficient to establish continuity:
        # the physical USB generation distinguishes the new enumeration.
        new = {**old, "generation": (41, 5, 10)}
        bus = UARTBoneUSB("/dev/serial/by-id/luna")
        port = bus.port = Mock()
        port.fileno.return_value = 17
        stat = SimpleNamespace(st_dev=1, st_ino=2, st_rdev=3)
        return bus, port, old, new, stat, Clock()

    def test_usb_reboot_retains_fd_until_physical_removal_and_new_generation(self):
        bus, port, old, new, stat, clock = self.usb_reboot_fixture()
        captured = False

        def generation(device):
            nonlocal captured
            if not captured:
                captured = True
                return old
            port.close.assert_called_once()
            # A late by-id alias still identifying the old generation cannot
            # authorize a new open, even after physical removal was observed.
            return old if clock.now < .02 else new

        def physical(path):
            port.close.assert_not_called()
            return old["generation"] if clock.now < .01 else None

        with patch.object(flash_module, "_usb_device_generation", side_effect=generation), \
                patch.object(flash_module, "_usb_generation_at", side_effect=physical), \
                patch.object(flash_module.os, "fstat", return_value=stat):
            transition = bus.prepare_reboot()
            self.assertEqual(transition(.1, clock=clock, sleep=clock.sleep),
                             {"old_generation": old, "new_generation": new})
        self.assertGreaterEqual(clock.now, .02)
        self.assertIsNone(bus.port)
        self.assertEqual([call[0] for call in port.method_calls], ["fileno", "close"])

    def test_usb_reboot_detects_generation_change_without_observing_absence(self):
        bus, port, old, new, stat, clock = self.usb_reboot_fixture()
        with patch.object(flash_module, "_usb_device_generation", side_effect=[old, new]), \
                patch.object(flash_module, "_usb_generation_at", return_value=new["generation"]), \
                patch.object(flash_module.os, "fstat", return_value=stat):
            transition = bus.prepare_reboot()
            self.assertEqual(transition(.1, clock=clock, sleep=clock.sleep)["new_generation"], new)
        port.close.assert_called_once()

    def test_uncertain_usb_reboot_write_retains_fd_and_never_replays(self):
        for partial in (False, True):
            with self.subTest(partial=partial):
                bus, port, old, new, stat, clock = self.usb_reboot_fixture()
                values = {0: BOOT_MAGIC, 4: 1, 0x0C: 0, 0x20: 0, 0x24: 0, 0x28: 0}
                triggers = []
                def read(address):
                    self.assertFalse(triggers, "no serial reads after the trigger")
                    return values[address-BOOT_BASE]
                def write(payload):
                    address = int.from_bytes(payload[2:6], "big")*4
                    value = int.from_bytes(payload[6:10], "big")
                    values[address-BOOT_BASE] = value
                    if address == BOOT_BASE+0x24:
                        triggers.append(value)
                        if partial:
                            return len(payload)-1
                        raise TimeoutError("all bytes queued before write timeout")
                    return len(payload)
                bus.read = read
                port.write.side_effect = write
                with patch.object(flash_module, "_usb_device_generation", side_effect=[old, new]), \
                        patch.object(flash_module, "_usb_generation_at", return_value=None), \
                        patch.object(flash_module.os, "fstat", return_value=stat):
                    transition = bus.prepare_reboot()
                    self.assertEqual(BootClient(bus).request(2, acknowledge=False),
                                     {"request_id": 6, "acknowledged": False})
                    port.close.assert_not_called()
                    self.assertEqual(triggers, [6])
                    transition(.1, clock=clock, sleep=clock.sleep)
                port.close.assert_called_once()
                self.assertEqual(triggers, [6])

    def test_usb_reboot_timeout_never_treats_alias_loss_as_physical_removal(self):
        bus, port, old, new, stat, clock = self.usb_reboot_fixture()
        with patch.object(flash_module, "_usb_device_generation", side_effect=[old, None]) as alias, \
                patch.object(flash_module, "_usb_generation_at", return_value=old["generation"]), \
                patch.object(flash_module.os, "fstat", return_value=stat):
            transition = bus.prepare_reboot()
            with self.assertRaisesRegex(FlashError, "physical device removal"):
                transition(.03, clock=clock, sleep=clock.sleep)
            self.assertEqual(alias.call_count, 1)  # Alias is not consulted until physical detach.
        port.close.assert_not_called()
        self.assertAlmostEqual(clock.now, .03)
        bus.close()  # The updater's final cleanup owns timeout cleanup.

    def test_usb_reboot_replacement_must_arrive_within_same_deadline(self):
        bus, port, old, new, stat, clock = self.usb_reboot_fixture()
        with patch.object(flash_module, "_usb_device_generation", side_effect=[old] + [None]*20), \
                patch.object(flash_module, "_usb_generation_at",
                             side_effect=lambda path: old["generation"] if clock.now < .02 else None), \
                patch.object(flash_module.os, "fstat", return_value=stat):
            transition = bus.prepare_reboot()
            with self.assertRaisesRegex(FlashError, "did not re-enumerate"):
                transition(.03, clock=clock, sleep=clock.sleep)
        port.close.assert_called_once()
        self.assertAlmostEqual(clock.now, .03)

    def test_usb_reboot_requires_generation_matching_the_open_fd(self):
        bus, port, old, new, stat, clock = self.usb_reboot_fixture()
        for snapshot in (None, {**old, "node_identity": (1, 99, 3)}):
            with self.subTest(snapshot=snapshot), \
                    patch.object(flash_module, "_usb_device_generation", return_value=snapshot), \
                    patch.object(flash_module.os, "fstat", return_value=stat):
                with self.assertRaises(FlashError):
                    bus.prepare_reboot()
        port.write.assert_not_called()
        bus.close()

    def test_usb_sysfs_removal_race_is_absence_but_permission_errors_propagate(self):
        for error_number in (errno.ENOENT, errno.ENODEV, errno.EACCES):
            with self.subTest(errno=error_number), \
                    patch.object(flash_module.Path, "stat", return_value=SimpleNamespace(st_ino=40)), \
                    patch.object(flash_module.Path, "read_text", side_effect=OSError(error_number, "sysfs read")):
                if error_number == errno.EACCES:
                    with self.assertRaises(PermissionError):
                        flash_module._usb_generation_at("/sys/fake-usb")
                else:
                    self.assertIsNone(flash_module._usb_generation_at("/sys/fake-usb"))

    def test_standalone_usb_boot_uses_transition_without_claiming_boot_validation(self):
        events, output = [], io.StringIO()
        bus, boot = Mock(), Mock()
        def prepare():
            events.append("prepare")
            def transition(deadline):
                events.append("transition")
                return {"generation_changed": True}
            return transition
        bus.prepare_reboot.side_effect = prepare
        def request(slot, *, acknowledge):
            self.assertEqual(slot, 2)
            self.assertFalse(acknowledge)
            events.append("request")
            return {"request_id": 6, "acknowledged": False}
        boot.request.side_effect = request
        @contextmanager
        def transport():
            try:
                yield bus
            finally:
                events.append("close")
        with patch.object(flash_module, "create_transport", return_value=transport()), \
                patch.object(flash_module, "BootClient", return_value=boot), redirect_stdout(output):
            self.assertEqual(flash_module.main(["--transport", "usb", "--device", "/dev/explicit",
                                                "boot", "--slot", "2"]), 0)
        self.assertEqual(events, ["prepare", "request", "transition", "close"])
        self.assertIn("does not verify", json.loads(output.getvalue())["note"])

    def test_discovery_probe_is_single_attempt_and_restores_transport_on_failure(self):
        for configured_timeout in (.2, .01):
            with self.subTest(timeout=configured_timeout):
                bus = EtherboneUDP(timeout=configured_timeout, attempts=3)
                bus.socket = Mock()
                bus.socket.recvfrom.side_effect = socket.timeout("no response")
                with patch.object(bus, "_send") as send:
                    with self.assertRaises(socket.timeout):
                        with bus.discovery_probe():
                            self.assertEqual((bus.timeout, bus.attempts), (min(configured_timeout, .025), 1))
                            bus.read(BOOT_BASE)
                    self.assertEqual(send.call_count, 1)
                    self.assertEqual((bus.timeout, bus.attempts), (configured_timeout, 3))
                    bus.socket.settimeout.assert_called_with(configured_timeout)
                    with self.assertRaises(socket.timeout):
                        bus.read(BOOT_BASE)
                    self.assertEqual(send.call_count, 4)

    def test_transport_packet_limits_require_explicit_configuration(self):
        bus = EtherboneUDP()
        with self.assertRaises(ValueError):
            bus.write(FLASH_BASE+0x400, [0]*9)
        self.assertEqual(bus.configure_burst_limit(128), 128)
        with patch.object(bus, "_send") as send:
            bus.write(FLASH_BASE+0x400, [0]*128)
            self.assertEqual(send.call_args.args[0].records[0].wcount, 128)
        with self.assertRaises(ValueError):
            bus.configure_burst_limit(129)
        usb = UARTBoneUSB("/dev/explicit-luna")
        with self.assertRaises(ValueError):
            usb._header(2, FLASH_BASE, 9)
        usb.configure_burst_limit(255)
        self.assertEqual(usb._header(2, FLASH_BASE, 255)[:2], b"\x02\xff")
        with self.assertRaises(ValueError):
            usb.configure_burst_limit(256)

    def test_etherbone_filters_other_peer_and_stale_response(self):
        from litex.tools.remote.etherbone import EtherbonePacket, EtherboneRecord, EtherboneWrites
        class Socket:
            def __init__(self):
                self.queue, self.sent = [], []
                self.closed = False

            def bind(self, address):
                self.bound = address

            def settimeout(self, value):
                self.timeout = value

            def sendto(self, raw, peer):
                self.sent.append(raw)
                packet = EtherbonePacket(32, raw)
                packet.decode()
                request = packet.records[0]
                for base, source in ((request.reads.base_ret_addr, ("192.168.1.99", 1234)),
                                     (request.reads.base_ret_addr-1, peer),
                                     (request.reads.base_ret_addr, peer)):
                    record = EtherboneRecord(addr_size=4)
                    record.writes = EtherboneWrites(addr_size=4, base_addr=base,
                                                    datas=iter([0x12345678, 0x90ABCDEF]))
                    record.wcount = 2
                    response = EtherbonePacket(32)
                    response.records = [record]
                    response.encode()
                    self.queue.append((response.bytes, source))

            def recvfrom(self, maximum):
                return self.queue.pop(0)

            def close(self):
                self.closed = True
        sock = Socket()
        with patch("tools.lutcam.flash.socket.socket", return_value=sock):
            with EtherboneUDP() as bus:
                self.assertEqual(bus.read(FLASH_BASE, length=2), [0x12345678, 0x90ABCDEF])
                self.assertEqual(bus.read_counter, 1)
        self.assertEqual(sock.bound, ("", 0))
        self.assertTrue(sock.closed)

    def test_usb_binary_framing_and_partial_response(self):
        class Port:
            def __init__(self):
                self.writes = []
                self.fragments = [b"\x12\x34", b"\x56\x78\x90\xab\xcd\xef"]
                self.closed = False

            def reset_input_buffer(self):
                pass

            def reset_output_buffer(self):
                pass

            def write(self, data):
                self.writes.append(data)
                return len(data)

            def read(self, count):
                return self.fragments.pop(0) if self.fragments else b""

            def close(self):
                self.closed = True
        port, clock = Port(), Clock()
        bus = UARTBoneUSB("/dev/explicit-luna", serial_factory=lambda *a, **k: port,
                          clock=clock, sleep=clock.sleep).open()
        self.assertGreaterEqual(clock.now, 0.1)
        self.assertEqual(bus.read(FLASH_BASE, length=2), [0x12345678, 0x90ABCDEF])
        self.assertEqual(port.writes[0], b"\x02\x02" + (FLASH_BASE//4).to_bytes(4, "big"))
        bus.write(FLASH_BASE+0x1C, 0xDEADBEEF)
        self.assertEqual(port.writes[1], b"\x01\x01" + ((FLASH_BASE+0x1C)//4).to_bytes(4, "big") + b"\xde\xad\xbe\xef")
        with self.assertRaisesRegex(TimeoutError, "framing"):
            bus.read(FLASH_BASE)
        self.assertTrue(port.closed)
        with self.assertRaises(ConnectionError):
            bus.read(FLASH_BASE)
        self.assertEqual(len(port.writes), 3)  # The failed read is never replayed.

    def test_usb_disconnect_during_input_reset_closes_and_normalizes_termios_error(self):
        try:
            import termios
        except ImportError:
            self.skipTest("POSIX termios is not available")
        error = termios.error(5, "Input/output error")
        port, clock = Mock(), Clock()
        port.reset_input_buffer.side_effect = error
        bus = UARTBoneUSB("/dev/explicit-luna", serial_factory=lambda *a, **k: port,
                          clock=clock, sleep=clock.sleep)
        with self.assertRaises(OSError) as caught:
            bus.open()
        self.assertEqual(caught.exception.errno, 5)
        self.assertIs(caught.exception.__cause__, error)
        port.close.assert_called_once()
        self.assertIsNone(bus.port)
        self.assertEqual(clock.sleeps, [.15])

    def test_usb_open_preserves_programmer_errors_after_closing(self):
        port, clock = Mock(), Clock()
        error = RuntimeError("programmer error")
        port.reset_input_buffer.side_effect = error
        bus = UARTBoneUSB("/dev/explicit-luna", serial_factory=lambda *a, **k: port,
                          clock=clock, sleep=clock.sleep)
        with self.assertRaises(RuntimeError) as caught:
            bus.open()
        self.assertIs(caught.exception, error)
        port.close.assert_called_once()
        self.assertIsNone(bus.port)


def bundle():
    build_id = "1234567890abcdef" * 2
    images = {"golden": b"golden", "A": b"application-A"*31, "B": b"application-B"*31}
    slots = []
    for name, slot in (("golden", 0), ("A", 1), ("B", 2)):
        entry = {"name": name, "selector": slot, "image_slot": slot, "address": slot*SLOT_SIZE,
                 "length": len(images[name]), "sha256": hashlib.sha256(images[name]).hexdigest(),
                 "build_id": build_id}
        if slot:
            entry.update(metadata_address=(slot+1)*SLOT_SIZE-4096,
                         metadata_hex=make_record(images[name], slot, build_id).hex())
        slots.append(entry)
    return {"slots": slots}, images


class Board:
    def __init__(self):
        self.manifest, self.images = bundle()
        self.entries = {entry["name"]: entry for entry in self.manifest["slots"]}
        self.memory = bytearray(b"\xff") * (3*SLOT_SIZE)
        self.memory[:len(self.images["golden"])] = self.images["golden"]
        self.slot = 1
        self.build_id = "0"*32  # The existing release is different from the update.
        self.calls = []
        self.connections = []
        self.next_slot = None
        self.outage = 0
        self.wrong_build = False
        self.fallback = False
        self.health_fail_b = False
        self.abi_fail_b = False
        self.corrupt_b_after_boot = False
        self.no_reboot = False
        self.jedec_fail_b = False
        self.fail_erase_address = None
        self.fail_program_address = None

    def connect(self):
        if self.outage:
            self.outage -= 1
            raise socket.timeout("reboot outage")
        if self.next_slot is not None:
            self.slot, self.next_slot = self.next_slot, None
            self.build_id = "f"*32 if self.wrong_build else self.entries[{0: "golden", 1: "A", 2: "B"}[self.slot]]["build_id"]
            if self.corrupt_b_after_boot and self.slot == 2:
                self.memory[2*SLOT_SIZE] ^= 1
        board = self
        class Connection:
            closed = False
            def close(self):
                self.closed = True
        connection = Connection()
        connection.board = board
        self.connections.append(connection)
        return connection

    def health(self, bus, slot):
        self.calls.append(("health", slot))
        if slot == 2 and self.health_fail_b:
            raise FlashError("B network/diagnostics health failed")
        return {"healthy": True}

    def seed_b(self):
        self.slot = 2
        self.build_id = self.entries["B"]["build_id"]
        entry = self.entries["B"]
        self.memory[entry["address"]:entry["address"]+entry["length"]] = self.images["B"]
        self.memory[entry["metadata_address"]:entry["metadata_address"]+64] = bytes.fromhex(entry["metadata_hex"])


class FakeFlash:
    def __init__(self, bus):
        self.board, self.image_slot = bus.board, bus.board.slot
        if self.image_slot == 2 and self.board.abi_fail_b:
            raise FlashError("B missing OTA flash interface")

    def jedec(self):
        return 0 if self.image_slot == 2 and self.board.jedec_fail_b else JEDEC_ID

    def erase(self, address, length):
        if address < SLOT_SIZE or (address < (self.image_slot+1)*SLOT_SIZE and address+length > self.image_slot*SLOT_SIZE):
            raise AssertionError("protected slot erase")
        self.board.calls.append(("erase", address, length))
        if address == self.board.fail_erase_address:
            self.board.memory[address:address+length//2] = b"\xff"*(length//2)
            raise FlashError("power interrupted during erase")
        self.board.memory[address:address+length] = b"\xff"*length

    def program(self, address, data, **unused):
        self.board.calls.append(("program", address, bytes(data)))
        if address == self.board.fail_program_address:
            # Model a torn page operation followed by loss of transport/power.
            count = max(1, len(data)//2)
            self.board.memory[address:address+count] = data[:count]
            raise FlashError("power interrupted during page program")
        self.board.memory[address:address+len(data)] = data
        self.verify(address, data)

    def verify(self, address, data, **unused):
        self.board.calls.append(("verify", address, len(data)))
        if bytes(self.board.memory[address:address+len(data)]) != data:
            raise FlashError("flash verification mismatch")


class FakeBoot:
    def __init__(self, bus):
        self.board = bus.board

    def identity(self):
        return {"image_slot": self.board.slot, "build_id": self.board.build_id,
                "status": 8 if self.board.fallback and self.board.slot == 0 else 0}

    def request(self, slot, *, acknowledge=True):
        self.board.calls.append(("boot", slot))
        if not self.board.no_reboot:
            self.board.next_slot = 0 if self.board.fallback else slot
            self.board.outage = 1
        return {"request_id": 1, "acknowledged": acknowledge}


class UpdateTests(unittest.TestCase):
    def setUp(self):
        self.board = Board()
        self.clock = Clock()

    def updater(self, **kwargs):
        return Updater(self.board.manifest, self.board.images, self.board.connect, self.board.health,
                       reboot_timeout=1, clock=self.clock, sleep=self.clock.sleep,
                       flash_factory=FakeFlash, boot_factory=FakeBoot, **kwargs)

    def assert_a_intact(self):
        self.assertFalse(any(call[0] == "erase" and SLOT_SIZE <= call[1] < 2*SLOT_SIZE for call in self.board.calls))

    def replace_image(self, name, data):
        self.board.images[name] = data
        slot = self.board.entries[name]
        slot.update(length=len(data), sha256=hashlib.sha256(data).hexdigest(),
                    metadata_hex=make_record(data, slot["image_slot"], slot["build_id"]).hex())

    def seed_old_b(self, length):
        slot = self.board.entries["B"]
        address, metadata = slot["address"], slot["metadata_address"]
        data = b"\x33" * length
        self.board.memory[address:metadata] = b"\x5a" * IMAGE_CAPACITY
        self.board.memory[address:address+length] = data
        self.board.memory[metadata:metadata+64] = make_record(data, 2, "0"*32)

    def test_maintenance_bundle_is_rejected_before_connection(self):
        self.board.manifest["golden_maintenance"] = True
        with self.assertRaisesRegex(ValueError, "maintenance-only bundle"):
            self.updater()
        self.assertEqual(self.board.connections, [])
        self.assertEqual(self.board.calls, [])

    def test_empty_expected_golden_is_rejected_before_connection(self):
        self.board.images["golden"] = b""
        self.board.entries["golden"].update(length=0, sha256=hashlib.sha256(b"").hexdigest())
        with self.assertRaisesRegex(ValueError, "golden image must be nonempty"):
            self.updater()
        self.assertEqual(self.board.connections, [])

    def test_dormant_old_or_corrupt_golden_stops_before_any_mutation(self):
        for offset in (0, len(self.board.images["golden"]) - 1):
            with self.subTest(offset=offset):
                self.board = Board()
                self.board.memory[offset] ^= 1
                updater = self.updater()
                with self.assertRaisesRegex(FlashError, "installed golden image does not match"):
                    updater.run()
                self.assertFalse(any(call[0] in ("erase", "program", "boot")
                                     for call in self.board.calls))
                self.assertFalse(updater._golden_verified)
                self.assertFalse(updater.report["golden_modified"])
                self.assertTrue(all(connection.closed for connection in self.board.connections))

    def test_matching_golden_is_verified_once_before_first_erase(self):
        updater = self.updater()
        report = updater.run()
        check = ("verify", 0, len(self.board.images["golden"]))
        self.assertEqual(self.board.calls.count(check), 1)
        first_erase = next(i for i, call in enumerate(self.board.calls) if call[0] == "erase")
        self.assertLess(self.board.calls.index(check), first_erase)
        self.assertEqual(report["golden_verification"]["sha256"],
                         self.board.entries["golden"]["sha256"])
        self.assertEqual(report["golden_verification"]["bytes"], len(self.board.images["golden"]))
        self.assertGreaterEqual(report["golden_verification"]["seconds"], 0)
        self.assertEqual([event["phase"] for event in report["events"]].count("golden_verified"), 1)

    def test_new_run_rechecks_golden_instead_of_reusing_previous_verification(self):
        updater = self.updater()
        updater.run()
        self.board.memory[0] ^= 1
        self.board.calls.clear()
        with self.assertRaisesRegex(FlashError, "installed golden image does not match"):
            updater.run()
        self.assertFalse(updater.report["ok"])
        self.assertNotIn("golden_verification", updater.report)
        self.assertFalse(any(call[0] in ("erase", "program", "boot") for call in self.board.calls))

    def test_golden_read_timeout_stops_before_any_mutation(self):
        updater = self.updater()
        class LostGoldenReply(FakeFlash):
            def verify(self, address, data, **kwargs):
                if address == 0:
                    raise TimeoutError("lost golden reply")
                return super().verify(address, data, **kwargs)
        updater.flash_factory = LostGoldenReply
        with self.assertRaisesRegex(TimeoutError, "lost golden reply"):
            updater.run()
        self.assertFalse(any(call[0] in ("erase", "program", "boot") for call in self.board.calls))
        self.assertFalse(updater._golden_verified)

    def test_complete_update_boots_b_before_a_erasure_and_finishes_a(self):
        report = self.updater().run()
        self.assertTrue(report["ok"])
        self.assertEqual(report["final_identity"]["image_slot"], 1)
        b_boot = self.board.calls.index(("boot", 2))
        a_erase = next(i for i, call in enumerate(self.board.calls) if call[0] == "erase" and call[1] < 2*SLOT_SIZE)
        b_verify = ("verify", 2*SLOT_SIZE, len(self.board.images["B"]))
        self.assertIn(b_verify, self.board.calls[b_boot:a_erase])
        self.assertIn(("health", 2), self.board.calls[b_boot:a_erase])
        self.assertTrue(all(connection.closed for connection in self.board.connections))
        for name in ("A", "B"):
            slot = self.board.entries[name]
            self.assertEqual(self.board.memory[slot["address"]:slot["address"]+slot["length"]], self.board.images[name])
            # Validity sector is erased first and commit word programmed last.
            erased = [call for call in self.board.calls if call[0] == "erase" and slot["address"] <= call[1] < slot["address"]+SLOT_SIZE]
            self.assertEqual(erased[0], ("erase", slot["metadata_address"], 4096))
            programs = [call for call in self.board.calls if call[0] == "program" and slot["address"] <= call[1] < slot["address"]+SLOT_SIZE]
            self.assertEqual(programs[-1][1], slot["metadata_address"] + COMMIT_OFFSET)

    def test_discovery_policy_ends_before_preflight_and_failed_candidates_close(self):
        updater = self.updater()
        connect, health = self.board.connect, self.board.health
        failures = {1, 2}
        probes, preflights = [], []
        test = self

        def normal(bus):
            test.assertEqual((bus.timeout, bus.attempts), (.2, 3))

        def connect_with_probe():
            bus = connect()
            bus.timeout, bus.attempts, bus.socket = .2, 3, None
            bus.discovery_probe = lambda: EtherboneUDP.discovery_probe(bus)
            close = bus.close
            def restored_close():
                normal(bus)
                close()
            bus.close = restored_close
            return bus

        class ProbedBoot(FakeBoot):
            def __init__(self, bus):
                super().__init__(bus)
                self.bus = bus

            def identity(self):
                if self.bus.attempts == 1:
                    test.assertEqual(self.bus.timeout, .025)
                    probes.append(self.board.slot)
                    if self.board.slot in failures:
                        failures.remove(self.board.slot)
                        raise socket.timeout("candidate disappeared during identity read")
                else:
                    normal(self.bus)
                return super().identity()

            def request(self, slot):
                normal(self.bus)
                return super().request(slot)

        def checked_flash(bus):
            normal(bus)
            preflights.append(bus.board.slot)
            return FakeFlash(bus)

        def checked_health(bus, slot):
            normal(bus)
            return health(bus, slot)

        updater.connect, updater.health = connect_with_probe, checked_health
        updater.boot_factory, updater.flash_factory = ProbedBoot, checked_flash
        self.assertTrue(updater.run()["ok"])
        self.assertEqual(probes, [1, 1, 2, 2, 1])
        self.assertEqual(preflights, [1, 2, 1])
        self.assertTrue(all(bus.closed for bus in self.board.connections))
        self.assertTrue(self.clock.sleeps)
        self.assertTrue(all(delay <= .01 for delay in self.clock.sleeps))

    def test_reboot_transition_prepares_before_trigger_and_closes_before_connect(self):
        updater = self.updater(final_a=False)
        connect, events = self.board.connect, []
        waiting = False

        def connect_with_transition():
            self.assertFalse(waiting, "reopened USB before transition finished")
            bus = connect()
            def prepare():
                nonlocal waiting
                self.assertFalse(bus.closed)
                waiting = True
                events.append("prepare")
                def transition(deadline, *, clock, sleep):
                    nonlocal waiting
                    self.assertIn(("boot", 2), self.board.calls)
                    self.assertFalse(bus.closed)
                    sleep(.02)
                    bus.close()
                    waiting = False
                    events.append("transition")
                    return {"generation_changed": True}
                return transition
            bus.prepare_reboot = prepare
            return bus

        updater.connect = connect_with_transition
        report = updater.run()
        self.assertTrue(report["ok"])
        self.assertEqual(events, ["prepare", "transition"])
        connected = next(event for event in report["events"] if event["phase"] == "reconnected")
        self.assertTrue(connected["observed_disconnect"])
        self.assertFalse(connected["request"]["acknowledged"])
        self.assertTrue(all(bus.closed for bus in self.board.connections))

    def test_transition_and_reconnect_share_deadline_and_late_identity_cannot_erase_a(self):
        updater = self.updater(final_a=False)
        connect = self.board.connect
        def connect_with_transition():
            if self.board.next_slot is not None:
                self.clock.sleep(.15)  # New USB open/parser-reset wait.
            bus = connect()
            def transition(deadline, *, clock, sleep):
                sleep(.9)
                bus.close()
                self.board.outage = 0  # Next connection returns the correct B identity.
                return {"generation_changed": True}
            bus.prepare_reboot = lambda: transition
            return bus
        updater.connect = connect_with_transition
        with self.assertRaisesRegex(FlashError, "did not reconnect before timeout"):
            updater.run()
        self.assertEqual(self.board.slot, 2)
        self.assertAlmostEqual(self.clock.now, 1.05)
        self.assert_a_intact()
        self.assertFalse(updater.report["a_erase_started"])
        self.assertTrue(all(bus.closed for bus in self.board.connections))

    def test_usb_transition_failure_preserves_a_and_closes_retained_connection(self):
        for stage in ("prepare", "detach", "replacement"):
            with self.subTest(stage=stage):
                self.board, self.clock = Board(), Clock()
                updater = self.updater(final_a=False)
                connect = self.board.connect
                def connect_with_failure():
                    bus = connect()
                    def prepare():
                        if stage == "prepare":
                            raise FlashError("missing USB generation")
                        def transition(deadline, *, clock, sleep):
                            if stage == "replacement":
                                bus.close()
                            sleep(max(0, deadline-clock()))
                            raise FlashError("USB transition timed out")
                        return transition
                    bus.prepare_reboot = prepare
                    return bus
                updater.connect = connect_with_failure
                with self.assertRaises(FlashError):
                    updater.run()
                self.assert_a_intact()
                self.assertFalse(updater.report["a_erase_started"])
                self.assertTrue(all(bus.closed for bus in self.board.connections))
                if stage == "prepare":
                    self.assertNotIn(("boot", 2), self.board.calls)
                else:
                    self.assertEqual(self.board.calls.count(("boot", 2)), 1)

    def test_usb_disconnect_during_reconnect_retries_and_remains_bounded(self):
        try:
            import termios
        except ImportError:
            self.skipTest("POSIX termios is not available")
        for persistent in (False, True):
            with self.subTest(persistent=persistent):
                self.board, self.clock = Board(), Clock()
                updater = self.updater(final_a=False)
                connect, ports = self.board.connect, []

                def reconnect():
                    if self.board.next_slot is not None and (persistent or not ports):
                        port = Mock()
                        port.reset_input_buffer.side_effect = termios.error(5, "USB disconnected")
                        ports.append(port)
                        return UARTBoneUSB("/dev/explicit-luna", serial_factory=lambda *a, **k: port,
                                           clock=self.clock, sleep=self.clock.sleep).open()
                    return connect()

                updater.connect = reconnect
                if persistent:
                    with self.assertRaisesRegex(FlashError, "did not reconnect before timeout"):
                        updater.run()
                    self.assert_a_intact()
                    self.assertFalse(updater.report["a_erase_started"])
                    self.assertGreaterEqual(self.clock.now, 1)
                    self.assertLess(self.clock.now, 1.16)
                    self.assertGreater(len(ports), 1)
                else:
                    self.assertTrue(updater.run()["ok"])
                    self.assertEqual(len(ports), 1)
                for port in ports:
                    port.close.assert_called_once()
                self.assertTrue(all(bus.closed for bus in self.board.connections))

    def test_growth_and_shrink_erase_only_new_image_blocks(self):
        for old_length, new_length in ((1, 403), (403, 65537), (65537, 131089),
                                       (131089, 65536), (196609, 403),
                                       (1072592, 17), (1, 1072592)):
            with self.subTest(old_length=old_length, new_length=new_length):
                self.board = Board()
                self.clock = Clock()
                self.seed_old_b(old_length)
                slot = self.board.entries["B"]
                address, metadata = slot["address"], slot["metadata_address"]
                erase_length = ((new_length + 65535) // 65536) * 65536
                previous_tail = bytes(self.board.memory[address+erase_length:metadata])
                self.replace_image("B", b"\xa5" * new_length)
                report = self.updater(final_a=False).run()
                erased = [call for call in self.board.calls if call[0] == "erase" and call[1] >= address]
                self.assertEqual(erased, [("erase", metadata, 4096), ("erase", address, erase_length)])
                self.assertEqual(self.board.memory[address:address+new_length], self.board.images["B"])
                self.assertEqual(self.board.memory[address+new_length:address+erase_length], b"\xff" * (erase_length-new_length))
                self.assertEqual(self.board.memory[address+erase_length:metadata], previous_tail)
                self.assertEqual(self.board.memory[metadata:metadata+64], bytes.fromhex(slot["metadata_hex"]))
                events = [event for event in report["events"] if event["phase"] == "erase" and event["slot"] == "B"]
                self.assertEqual([(event["region"], event["address"], event["bytes"]) for event in events],
                                 [("metadata", metadata, 4096), ("payload", address, erase_length)])

    def test_near_capacity_payload_erase_can_overlap_metadata(self):
        for length in (IMAGE_CAPACITY-1, IMAGE_CAPACITY):
            with self.subTest(length=length):
                self.board = Board()
                self.clock = Clock()
                self.seed_old_b(403)
                self.replace_image("B", b"\xa5" * length)
                self.assertTrue(self.updater(final_a=False).run()["ok"])
                slot = self.board.entries["B"]
                address, metadata = slot["address"], slot["metadata_address"]
                erased = [call for call in self.board.calls if call[0] == "erase" and call[1] >= address]
                self.assertEqual(erased, [("erase", metadata, 4096), ("erase", address, SLOT_SIZE)])
                self.assertEqual(self.board.memory[address:address+length], self.board.images["B"])
                self.assertEqual(self.board.memory[metadata:metadata+64], bytes.fromhex(slot["metadata_hex"]))
                self.assertEqual(self.board.memory[metadata+64:address+SLOT_SIZE], b"\xff"*(4096-64))

    def test_interrupted_payload_erase_invalidates_previous_b_first(self):
        self.seed_old_b(65537)
        self.replace_image("B", b"\xa5" * 131089)
        self.board.fail_erase_address = 2*SLOT_SIZE
        updater = self.updater()
        with self.assertRaisesRegex(FlashError, "power interrupted during erase"):
            updater.run()
        self.assert_a_intact()
        self.assertFalse(updater.report["a_erase_started"])
        metadata = self.board.entries["B"]["metadata_address"]
        self.assertEqual(self.board.memory[metadata:metadata+4096], b"\xff"*4096)
        self.assertEqual([call for call in self.board.calls if call[0] == "erase"],
                         [("erase", metadata, 4096), ("erase", 2*SLOT_SIZE, 3*65536)])
        self.assertFalse(any(call[0] in ("program", "boot") for call in self.board.calls))

    def test_a_erase_started_is_set_before_metadata_erase_can_fail(self):
        self.board.seed_b()
        metadata = self.board.entries["A"]["metadata_address"]
        self.board.fail_erase_address = metadata
        updater = self.updater()
        with self.assertRaisesRegex(FlashError, "power interrupted during erase"):
            updater.run()
        self.assertTrue(updater.report["a_erase_started"])
        self.assertEqual([call for call in self.board.calls if call[0] == "erase"], [("erase", metadata, 4096)])
        self.assertEqual(self.board.slot, 2)

    def test_golden_start_skips_application_state_and_still_updates(self):
        self.board.slot = 0
        report = self.updater(final_a=False).run()
        self.assertTrue(report["ok"])
        self.assertEqual(self.board.slot, 2)
        self.assertFalse(any(call[0] == "erase" and call[1] < SLOT_SIZE for call in self.board.calls))

    def test_resume_matching_b_does_not_erase_b(self):
        self.board.seed_b()
        self.assertTrue(self.updater().run()["ok"])
        self.assertFalse(any(call[0] == "erase" and call[1] >= 2*SLOT_SIZE for call in self.board.calls))

    def test_resume_wrong_b_build_stops_before_any_erase(self):
        self.board.seed_b()
        self.board.build_id = "f"*32
        with self.assertRaisesRegex(FlashError, "identity"):
            self.updater().run()
        self.assertFalse(any(call[0] == "erase" for call in self.board.calls))

    def test_b_failures_never_erase_a(self):
        for flag in ("wrong_build", "fallback", "health_fail_b", "abi_fail_b",
                     "corrupt_b_after_boot", "no_reboot", "jedec_fail_b"):
            with self.subTest(failure=flag):
                self.board = Board()
                self.clock = Clock()
                setattr(self.board, flag, True)
                updater = self.updater()
                with self.assertRaises(FlashError):
                    updater.run()
                self.assert_a_intact()
                self.assertFalse(updater.report["ok"])
                self.assertFalse(updater.report["a_erase_started"])
                self.assertTrue(all(connection.closed for connection in self.board.connections))

    def test_invalid_hash_prevents_any_connection(self):
        self.board.images["B"] = b"corrupted"
        with self.assertRaises(ValueError):
            self.updater()
        self.assertEqual(self.board.connections, [])
        self.assertEqual(self.board.calls, [])

    def test_interrupted_b_image_never_publishes_validity_or_erases_a(self):
        self.board.fail_program_address = 2*SLOT_SIZE
        with self.assertRaisesRegex(FlashError, "power interrupted"):
            self.updater().run()
        self.assert_a_intact()
        commit = self.board.entries["B"]["metadata_address"] + COMMIT_OFFSET
        self.assertEqual(self.board.memory[commit:commit+4], b"\xff"*4)
        self.assertNotIn(("boot", 2), self.board.calls)

    def test_interrupted_a_image_leaves_verified_b_and_a_uncommitted(self):
        self.board.fail_program_address = SLOT_SIZE
        updater = self.updater()
        with self.assertRaisesRegex(FlashError, "power interrupted"):
            updater.run()
        self.assertTrue(updater.report["a_erase_started"])
        self.assertEqual(self.board.slot, 2)
        b = self.board.entries["B"]
        self.assertEqual(self.board.memory[b["address"]:b["address"]+b["length"]], self.board.images["B"])
        self.assertEqual(self.board.memory[b["metadata_address"]:b["metadata_address"]+64], bytes.fromhex(b["metadata_hex"]))
        commit = self.board.entries["A"]["metadata_address"] + COMMIT_OFFSET
        self.assertEqual(self.board.memory[commit:commit+4], b"\xff"*4)

    def test_torn_commit_is_detected_and_b_is_never_booted(self):
        from tools.lutcam.ota_bundle import parse_record
        record_address = self.board.entries["B"]["metadata_address"]
        self.board.fail_program_address = record_address + COMMIT_OFFSET
        with self.assertRaisesRegex(FlashError, "power interrupted"):
            self.updater().run()
        self.assert_a_intact()
        with self.assertRaisesRegex(ValueError, "uncommitted"):
            parse_record(bytes(self.board.memory[record_address:record_address+64]), expected_slot=2)
        self.assertNotIn(("boot", 2), self.board.calls)

    def test_different_a_b_release_ids_rejected(self):
        self.board.manifest["slots"][1]["build_id"] = "f"*32
        with self.assertRaisesRegex(ValueError, "same release"):
            self.updater()

    def test_health_check_requires_idle_and_preserves_faults(self):
        names = ("board_status", "camera0_run", "camera0_status", "camera1_run", "camera1_status",
                 "etherbone_diagnostics_status", "etherbone_diagnostics_count")
        registers = {name: i*4 for i, name in enumerate(names)}
        values = {0: 3, 4: 0, 8: 2, 12: 0, 16: 2, 20: 0, 24: 0}
        class Bus:
            def read(self, address):
                return values[address]
        self.assertEqual(check_health(Bus(), 0, {}), {"recovery": True})
        self.assertEqual(check_health(Bus(), 1, registers)["board_status"], 3)
        values[4] = 1
        with self.assertRaisesRegex(FlashError, "stopped and idle"):
            check_health(Bus(), 1, registers)
        values[4], values[24] = 0, 1
        with self.assertRaisesRegex(FlashError, "diagnostics_count"):
            check_health(Bus(), 1, registers)


if __name__ == "__main__":
    unittest.main()
