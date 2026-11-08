"""Shared camera I2C commands checked against an independent wire model.

The target observes only resolved open-drain SCL/SDA. Its register pointer,
ACK decisions, read data, and clock holds do not inspect controller state.
"""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from migen import Module, Signal
from migen.sim import passive, run_simulation

from lutcam.i2c import SharedCameraI2C


CONTROL, PINS = 0x00, 0x04
MAGIC, STATUS, DESC, COMMAND = 0x10, 0x14, 0x18, 0x1c
DONE, NEXT, RESULT, CAPACITY = 0x20, 0x24, 0x28, 0x2c
TX, RX, RECORD = 0x40, 0x60, 0x80
LOW, HIGH, TIMEOUT = 6, 4, 40


def descriptor(device, register, length, *, read=False, slow=False):
    assert 1 <= length <= 16
    return (device | (int(read) << 7) | (register << 8)
            | ((length - 1) << 24) | (int(slow) << 28))


def transfer(bus, address, data=None, *, sel=15, limit=16):
    """A classic Wishbone word transfer with a bounded termination check."""
    yield bus.adr.eq(address // 4)
    yield bus.dat_w.eq(0 if data is None else data)
    yield bus.sel.eq(sel)
    yield bus.we.eq(data is not None)
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)
    yield
    for _ in range(limit):
        ack, error = (yield bus.ack), (yield bus.err)
        assert not (ack and error), "Wishbone ACK and ERR asserted together"
        if ack or error:
            value = (yield bus.dat_r)
            yield bus.cyc.eq(0)
            yield bus.stb.eq(0)
            yield
            return value, bool(error)
        yield
    raise AssertionError("I2C register transaction did not terminate")


def read_word(bus, address):
    value, error = yield from transfer(bus, address)
    assert not error, f"read of {address:#x} failed"
    return value


def continuous_transfers(bus, operations):
    """Change words at each response while retaining CYC/STB throughout."""
    responses = []
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)
    yield bus.sel.eq(15)
    for address, data in operations:
        yield bus.adr.eq(address // 4)
        yield bus.dat_w.eq(0 if data is None else data)
        yield bus.we.eq(data is not None)
        yield
        for _ in range(16):
            ack, error = (yield bus.ack), (yield bus.err)
            assert not (ack and error), "Wishbone ACK and ERR asserted together"
            if ack or error:
                responses.append(((yield bus.dat_r), bool(error)))
                break
            yield
        else:
            raise AssertionError("continuous I2C register request did not terminate")
    yield bus.cyc.eq(0)
    yield bus.stb.eq(0)
    yield
    return responses


def write_word(bus, address, value, *, error=False, sel=15):
    _, observed_error = yield from transfer(bus, address, value, sel=sel)
    assert observed_error == error, (address, value, observed_error, error)


def stage(bus, device, register, data=None, *, length=None, read=False, slow=False):
    """Prepare a host record in Python; this performs no hardware staging."""
    if data is not None:
        data = bytes(data)
        length = len(data)
    words = [descriptor(device, register, length, read=read, slow=slow)]
    if not read:
        for index in range(0, len(data), 4):
            word = int.from_bytes(data[index:index + 4].ljust(4, b"\0"), "little")
            words.append(word)
    bus._prepared_i2c_words = words
    yield from ()


def record_word(bus, index, value, *, first=False, last=False, count=4,
                source=0, sel=15, address=None, abort_sources=0):
    engine = bus._i2c_engine
    yield engine.record_first.eq(first)
    yield engine.record_last.eq(last)
    yield engine.record_count.eq(count)
    yield engine.record_source.eq(source)
    yield engine.record_abort_sources.eq(abort_sources)
    result = yield from transfer(bus, RECORD + 4 * index if address is None else address,
                                 value, sel=sel)
    yield engine.record_first.eq(0)
    yield engine.record_last.eq(0)
    yield engine.record_abort_sources.eq(0)
    return result


def record(bus, words, *, source=0, count=None, error=False, continuous=False):
    if continuous:
        return (yield from continuous_record(bus, words, source=source))
    errors = []
    for index, word in enumerate(words):
        _, failed = yield from record_word(bus, index, word, first=index == 0,
                                           last=index == len(words) - 1,
                                           count=len(words) if count is None else count,
                                           source=source)
        errors.append(failed)
    if error is not None:
        assert any(errors) == error, (words, errors, error)
    return errors


def submit(bus, command_id, *, error=False, source=0, continuous=False):
    words = [command_id, *bus._prepared_i2c_words, command_id]
    return (yield from record(bus, words, source=source, error=error, continuous=continuous))


def continuous_record(bus, words, *, source=0):
    """Retain CYC/STB and all source metadata until each word is accepted."""
    engine = bus._i2c_engine
    yield engine.record_source.eq(source)
    yield engine.record_count.eq(len(words))
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)
    yield bus.sel.eq(15)
    yield bus.we.eq(1)
    errors = []
    for index, word in enumerate(words):
        yield engine.record_first.eq(index == 0)
        yield engine.record_last.eq(index == len(words) - 1)
        yield bus.adr.eq((RECORD // 4) + index)
        yield bus.dat_w.eq(word)
        yield
        for _ in range(16):
            ack, failed = (yield bus.ack), (yield bus.err)
            assert not (ack and failed)
            if ack or failed:
                errors.append(bool(failed))
                break
            yield
        else:
            raise AssertionError("atomic record word did not terminate")
    yield bus.cyc.eq(0)
    yield bus.stb.eq(0)
    yield engine.record_first.eq(0)
    yield engine.record_last.eq(0)
    yield
    return errors


def wait_done(bus, command_id, *, limit=4000):
    for _ in range(limit):
        status = yield from read_word(bus, STATUS)
        if not status & 1:
            assert status & 2, "completed command lost its valid result"
            assert (yield from read_word(bus, DONE)) == command_id
            return (yield from read_word(bus, RESULT))
        yield
    raise AssertionError("I2C engine did not finish in bounded time")


class OpenDrainHarness(Module):
    def __init__(self, **kwargs):
        options = dict(fast_low_cycles=LOW, fast_high_cycles=HIGH,
                       stretch_timeout_cycles=TIMEOUT)
        options.update(kwargs)
        self.submodules.engine = SharedCameraI2C(**options)
        self.target_scl_low = [Signal(), Signal()]
        self.target_sda_low = [Signal(), Signal()]
        for bus in self.engine.buses:
            bus._i2c_engine = self.engine
        for camera in range(2):
            scl, sda = self.engine.scl[camera], self.engine.sda[camera]
            self.comb += [
                scl.i.eq(~((scl.oe & ~scl.o) | self.target_scl_low[camera])),
                sda.i.eq(~((sda.oe & ~sda.o) | self.target_sda_low[camera])),
            ]


class RegisterTarget:
    """OV9281-style 16-bit pointer, auto-incrementing byte register target."""

    def __init__(self, dut, camera, address, *, nack_at=None,
                 stretch_after_rise=None, stretch_cycles=0, stuck_sda=False,
                 hold_sda_after_rise=None):
        self.dut, self.camera, self.address = dut, camera, address
        self.nack_at = nack_at
        self.stretch_after_rise = stretch_after_rise
        self.stretch_cycles = stretch_cycles
        self.stuck_sda = stuck_sda
        self.hold_sda_after_rise = hold_sda_after_rise
        self.memory = {}
        self.transactions = []
        self.starts = []
        self.stops = []
        self.master_acks = []
        self.read_data = []
        self.high_widths = []
        self.low_widths = []
        self.stretched_high_widths = []
        self.stretched_rises = []
        self.transitions = []
        self.pointer = 0
        self.wire_index = 0
        self.current = None
        self.segments = []
        self.mode = "idle"
        self.phase = 0
        self.shift = 0
        self.ack = False
        self.target_low = False
        self.read_mode = False

    def received(self, byte):
        self.current.append(byte)
        self.ack = (self.wire_index != self.nack_at
                    and (len(self.current) != 1 or byte >> 1 == self.address))
        self.wire_index += 1
        if len(self.current) == 1:
            self.read_mode = bool(byte & 1)
        elif len(self.current) == 3:
            self.pointer = (self.current[1] << 8) | byte
        elif len(self.current) > 3 and self.ack:
            self.memory[self.pointer] = byte
            self.pointer = (self.pointer + 1) & 0xffff

    def next_read(self):
        self.shift = self.memory.get(self.pointer, 0xff)
        self.pointer = (self.pointer + 1) & 0xffff
        self.phase = 0
        self.mode = "tx"
        self.target_low = not bool(self.shift & 0x80)

    @passive
    def process(self):
        scl_pin = self.dut.engine.scl[self.camera]
        sda_pin = self.dut.engine.sda[self.camera]
        scl_drive = self.dut.target_scl_low[self.camera]
        sda_drive = self.dut.target_sda_low[self.camera]
        previous_scl = previous_sda = 1
        rise_cycle = fall_cycle = None
        cycle = rises = 0
        stretch_active = stretch_done = False
        stretch_remaining = None
        measure_stretched_high = False
        yield sda_drive.eq(self.stuck_sda)
        while True:
            scl, sda = (yield scl_pin.i), (yield sda_pin.i)
            master_scl_low = (yield scl_pin.oe)
            assert not (master_scl_low and (yield scl_pin.o)), "SCL driven high"
            assert not ((yield sda_pin.oe) and (yield sda_pin.o)), "SDA driven high"
            if (scl, sda) != (previous_scl, previous_sda):
                self.transitions.append((cycle, scl, sda))
            # START and STOP are recognized from the physical wired-AND bus.
            if scl and previous_scl and previous_sda and not sda:
                if self.current is None:
                    self.segments = []
                    self.wire_index = 0
                else:
                    self.segments.append(bytes(self.current))
                self.current = []
                self.starts.append(cycle)
                self.mode, self.phase, self.shift = "rx", 0, 0
                self.read_mode = self.target_low = False
            elif scl and previous_scl and not previous_sda and sda:
                if self.current is not None:
                    self.segments.append(bytes(self.current))
                    self.transactions.append(tuple(self.segments))
                    self.current = None
                self.stops.append(cycle)
                self.mode = "idle"
                self.target_low = False
            if not previous_scl and scl:
                rises += 1
                rise_cycle = cycle
                if measure_stretched_high:
                    self.stretched_rises.append(cycle)
                if fall_cycle is not None:
                    self.low_widths.append(cycle - fall_cycle)
                if self.mode == "rx":
                    if self.phase < 8:
                        self.shift = (self.shift << 1) | sda
                        self.phase += 1
                        if self.phase == 8:
                            self.received(self.shift)
                    elif self.phase == 8:
                        self.phase = 9
                elif self.mode == "tx":
                    self.phase += 1
                    if self.phase == 8:
                        self.read_data.append(self.shift)
                elif self.mode == "master_ack":
                    self.master_acks.append(sda)
                    self.ack = not sda
                    self.mode = "master_ack_seen"
            if previous_scl and not scl:
                fall_cycle = cycle
                if rise_cycle is not None:
                    width = cycle - rise_cycle
                    self.high_widths.append(width)
                    if measure_stretched_high:
                        self.stretched_high_widths.append(width)
                        measure_stretched_high = False
                if self.mode == "rx":
                    if self.phase == 8:
                        self.target_low = self.ack
                    elif self.phase == 9:
                        self.target_low = False
                        if self.read_mode and len(self.current) == 1 and self.ack:
                            self.next_read()
                        else:
                            self.phase = self.shift = 0
                elif self.mode == "tx":
                    if self.phase == 8:
                        self.target_low = False
                        self.mode = "master_ack"
                    else:
                        self.target_low = not bool(self.shift & (1 << (7 - self.phase)))
                elif self.mode == "master_ack_seen":
                    if self.ack:
                        self.next_read()
                    else:
                        self.target_low = False
                        self.mode = "wait_stop"
                if (not stretch_done and self.stretch_after_rise is not None
                        and rises == self.stretch_after_rise):
                    stretch_active = stretch_done = True
                    yield scl_drive.eq(1)
                if self.hold_sda_after_rise is not None and rises == self.hold_sda_after_rise:
                    self.stuck_sda = True
            if stretch_active and not master_scl_low:
                if stretch_remaining is None:
                    stretch_remaining = self.stretch_cycles
                if stretch_remaining == 0:
                    yield scl_drive.eq(0)
                    stretch_active = False
                    measure_stretched_high = True
                elif stretch_remaining > 0:
                    stretch_remaining -= 1
                # A negative duration deliberately keeps SCL low forever.
            yield sda_drive.eq(self.target_low or self.stuck_sda)
            previous_scl, previous_sda = scl, sda
            cycle += 1
            yield


class SharedCameraI2CTests(unittest.TestCase):
    def simulate(self, bench, *, target_options=None, engine_options=None):
        dut = OpenDrainHarness(**(engine_options or {}))
        options = target_options or ({}, {})
        targets = [RegisterTarget(dut, camera, address, **options[camera])
                   for camera, address in enumerate((0x60, 0x10))]
        run_simulation(dut, [bench(dut, targets)] + [t.process() for t in targets])
        return dut, targets

    def test_discovery_gpio_pin_controls_and_read_only_registers(self):
        def bench(dut, targets):
            for camera, bus in enumerate(dut.engine.buses):
                self.assertEqual((yield from read_word(bus, MAGIC)), 0x49324301)
                self.assertEqual((yield from read_word(bus, CAPACITY)), 16)
                self.assertEqual((yield from read_word(bus, NEXT)), 1)
                self.assertEqual((yield from read_word(bus, PINS)) & 0x103, 0x103)
                for value in (1, 3, 2, 0):
                    yield from write_word(bus, CONTROL, value)
                    for _ in range(5):
                        yield
                    self.assertEqual((yield from read_word(bus, CONTROL)), value)
                    self.assertEqual((yield from read_word(bus, PINS)) & 3,
                                     (~value) & 3)
                    other = 1 - camera
                    self.assertEqual((yield dut.engine.scl[other].oe), 0)
                    self.assertEqual((yield dut.engine.sda[other].oe), 0)
                for address in (PINS, MAGIC, STATUS, DESC, COMMAND, DONE,
                                NEXT, RESULT, CAPACITY, TX, RX):
                    yield from write_word(bus, address, 0, error=True)
        self.simulate(bench)

    def test_write_word_boundaries_use_exact_wire_order_and_one_camera(self):
        for camera, payload in ((0, b"\xa5"), (0, bytes(range(4))),
                                (1, bytes(range(5))), (1, bytes(range(16)))):
            with self.subTest(camera=camera, length=len(payload)):
                def bench(dut, targets):
                    bus = dut.engine.buses[camera]
                    address = targets[camera].address
                    yield from stage(bus, address, 0x3500, payload)
                    yield from submit(bus, 1)
                    result = yield from wait_done(bus, 1)
                    self.assertEqual(result & 0xff, 0)
                    self.assertEqual((result >> 16) & 0xff, len(payload))
                    self.assertEqual((yield from read_word(bus, NEXT)), 2)
                    status = yield from read_word(bus, STATUS)
                    self.assertEqual(status & 0xf, 2 | (camera << 2))
                    for pin in (*dut.engine.scl, *dut.engine.sda):
                        self.assertEqual((yield pin.oe), 0)
                _, targets = self.simulate(bench)
                target = targets[camera]
                self.assertEqual(target.transactions,
                                 [(bytes([target.address << 1, 0x35, 0]) + payload,)])
                self.assertEqual(bytes(target.memory[0x3500 + i]
                                       for i in range(len(payload))), payload)
                self.assertEqual(len(target.starts), 1)
                self.assertEqual(len(target.stops), 1)
                self.assertEqual(targets[1 - camera].transitions, [])

    def test_read_repeated_start_last_nack_and_retained_little_endian_buffer(self):
        for length in (1, 3, 16):
            with self.subTest(length=length):
                expected = bytes((0x92 + 37 * i) & 255 for i in range(length))
                def bench(dut, targets):
                    bus, target = dut.engine.buses[1], targets[1]
                    target.memory.update({0x300a + i: value
                                          for i, value in enumerate(expected)})
                    yield from stage(bus, 0x10, 0x300a, length=length, read=True)
                    yield from submit(bus, 1)
                    result = yield from wait_done(bus, 1)
                    self.assertEqual(result & 0xff, 0)
                    self.assertEqual((result >> 16) & 0xff, length)
                    self.assertEqual((yield from read_word(bus, STATUS)) & 15, 14)
                    for _ in range(2):
                        received = bytearray()
                        for index in range(0, length, 4):
                            value = yield from read_word(bus, RX + index)
                            received.extend(value.to_bytes(4, "little"))
                        self.assertEqual(received[:length], expected)
                        self.assertEqual((yield from read_word(bus, RESULT)), result)
                    yield from submit(bus, 1)
                    for _ in range(30):
                        yield
                    self.assertEqual((yield from read_word(bus, DONE)), 1)
                _, targets = self.simulate(bench)
                target = targets[1]
                self.assertEqual(target.transactions, [(b"\x20\x30\x0a", b"\x21")])
                self.assertEqual(bytes(target.read_data), expected)
                self.assertEqual(target.master_acks, [0] * (length - 1) + [1])
                self.assertEqual((len(target.starts), len(target.stops)), (2, 1))
                self.assertEqual(targets[0].transitions, [])

    def test_old_busy_and_duplicate_records_never_modify_active_or_fresh_commands(self):
        payload = bytes(range(16))
        old = [1, descriptor(0x60, 0x4000, 1), 0xee, 1]
        def bench(dut, targets):
            bus, other = dut.engine.buses
            yield from stage(bus, 0x60, 0x3500, payload)
            yield from submit(bus, 1)
            self.assertEqual((yield dut.engine.busy), 1)
            yield from write_word(bus, CONTROL, 3, error=True)
            yield from write_word(other, CONTROL, 3)
            self.assertEqual((yield from read_word(other, CONTROL)), 3)
            yield from record(bus, old, source=0)
            yield from record(other, old, source=1)
            yield from record(bus, [2, descriptor(0x60, 0x4000, 1), 0xdd, 2], source=2)
            yield from write_word(other, CONTROL, 0)
            for port in (bus, other):
                for address in (DESC, COMMAND, TX, TX + 4, TX + 8, TX + 12):
                    yield from write_word(port, address, 0, error=True)
            result = yield from wait_done(bus, 1)
            yield from record(bus, old)
            self.assertEqual((yield from read_word(bus, RESULT)), result)
            self.assertEqual((yield from read_word(bus, TX)), 0x03020100)
            yield from stage(other, 0x10, 0x3509, b"\x10")
            yield from submit(other, 2, source=1)
            yield from record(bus, old, source=0)
            self.assertEqual((yield from wait_done(other, 2)), 1 << 16)
            yield from record(bus, old, source=0)
            self.assertEqual((yield from read_word(other, COMMAND)), 2)
            self.assertEqual((yield from read_word(other, RX)) & 255, 0x10)
            self.assertEqual((yield from read_word(other, NEXT)), 3)
        _, targets = self.simulate(bench)
        self.assertEqual(targets[0].transactions, [(b"\xc0\x35\x00" + payload,)])
        self.assertEqual(targets[1].transactions, [(b"\x20\x35\x09\x10",)])

    def test_malformed_records_drain_without_id_consumption_or_sensor_traffic(self):
        names = ("missing_first", "early_last", "missing_last", "wrong_address",
                 "wrong_count", "partial_header", "partial_descriptor", "partial_payload",
                 "bad_commit", "reserved_descriptor", "short_shape", "long_shape",
                 "wrong_camera", "first_wrong_address", "small_header_count",
                 "large_header_count", "partial_commit", "unexpected_first")
        for name in names:
            with self.subTest(malformed=name):
                def bench(dut, targets):
                    bus, other = dut.engine.buses
                    words = [1, descriptor(0x60, 0x4000, 1), 0xa5, 1]
                    if name == "reserved_descriptor":
                        words[1] |= 1 << 29
                    if name == "short_shape":
                        words[1] = descriptor(0x60, 0x4000, 5)
                    if name == "long_shape":
                        words[1] = descriptor(0x60, 0x4000, 1, read=True)
                    for index, value in enumerate(words):
                        options = dict(first=index == 0, last=index == 3, count=4)
                        port = bus
                        if name == "missing_first" and index == 0:
                            options["first"] = False
                        if name == "early_last" and index == 1:
                            options["last"] = True
                        if name == "missing_last" and index == 3:
                            options["last"] = False
                        if name == "wrong_address" and index == 2:
                            options["address"] = RECORD + 4
                        if name == "wrong_count" and index == 1:
                            options["count"] = 5
                        if name == "partial_header" and index == 0:
                            options["sel"] = 7
                        if name == "partial_descriptor" and index == 1:
                            options["sel"] = 7
                        if name == "partial_payload" and index == 2:
                            options["sel"] = 7
                        if name == "partial_commit" and index == 3:
                            options["sel"] = 7
                        if name == "small_header_count":
                            options["count"] = 2
                        if name == "large_header_count":
                            options["count"] = 8
                        if name == "unexpected_first" and index == 2:
                            options["first"] = True
                        if name == "bad_commit" and index == 3:
                            value = 2
                        if name == "wrong_camera" and index == 1:
                            port = other
                        if name == "first_wrong_address" and index == 0:
                            options["address"] = RECORD + 4
                        _, error = yield from record_word(port, index, value, **options)
                        self.assertFalse(error)
                    self.assertEqual((yield from read_word(bus, NEXT)), 1)
                    self.assertEqual((yield from read_word(bus, COMMAND)), 0)
                    self.assertEqual(targets[0].starts, [])
                    # A new first word replaces any abandoned same-source lease.
                    yield from stage(bus, 0x60, 0x3509, b"\x28")
                    yield from submit(bus, 1)
                    self.assertEqual((yield from wait_done(bus, 1)), 1 << 16)
                _, targets = self.simulate(bench)
                self.assertEqual(targets[0].transactions, [(b"\xc0\x35\x09\x28",)])
                self.assertEqual(targets[1].transactions, [])

    def test_nack_reports_wire_byte_and_acknowledged_data_count_then_stops(self):
        for failing_byte in (0, 1, 2, 3, 5):
            with self.subTest(failing_byte=failing_byte):
                def bench(dut, targets):
                    bus = dut.engine.buses[0]
                    yield from stage(bus, 0x60, 0x3500, b"\x01\x23\x45")
                    yield from submit(bus, 1)
                    result = yield from wait_done(bus, 1)
                    self.assertEqual(result & 255, 1)
                    self.assertEqual((result >> 8) & 255, failing_byte)
                    self.assertEqual((result >> 16) & 255, max(0, failing_byte - 3))
                    self.assertEqual((yield dut.engine.scl[0].oe), 0)
                    self.assertEqual((yield dut.engine.sda[0].oe), 0)
                    yield from submit(bus, 1)
                    self.assertEqual((yield from read_word(bus, RESULT)), result)
                _, targets = self.simulate(bench, target_options=({"nack_at": failing_byte}, {}))
                self.assertEqual(len(targets[0].transactions), 1)
                expected = b"\xc0\x35\x00\x01\x23\x45"[:failing_byte + 1]
                self.assertEqual(targets[0].transactions[0], (expected,))
                self.assertEqual(len(targets[0].stops), 1)

    def test_read_address_nack_can_be_retried_with_a_fresh_command(self):
        expected = b"\x92\x81"
        def bench(dut, targets):
            bus, target = dut.engine.buses[0], targets[0]
            target.memory.update({0x300a: expected[0], 0x300b: expected[1]})
            yield from stage(bus, 0x60, 0x300a, length=2, read=True)
            yield from submit(bus, 1)
            result = yield from wait_done(bus, 1)
            self.assertEqual(result, 1 | (3 << 8))
            yield from submit(bus, 1)
            self.assertEqual((yield from read_word(bus, RESULT)), result)
            target.nack_at = None
            yield from stage(bus, 0x60, 0x300a, length=2, read=True)
            yield from submit(bus, 2)
            self.assertEqual((yield from wait_done(bus, 2)), 2 << 16)
            self.assertEqual((yield from read_word(bus, RX)) & 0xffff, 0x8192)
        _, targets = self.simulate(bench, target_options=({"nack_at": 3}, {}))
        target = targets[0]
        self.assertEqual(target.transactions,
                         [(b"\xc0\x30\x0a", b"\xc1")] * 2)
        self.assertEqual(target.read_data, list(expected))
        self.assertEqual(target.master_acks, [0, 1])

    def test_truncated_record_new_first_and_delayed_tail_cannot_replay(self):
        def bench(dut, targets):
            bus = dut.engine.buses[0]
            abandoned = [1, descriptor(0x60, 0x4000, 16), 0xeeeeeeee,
                         0xdddddddd, 0xcccccccc, 0xbbbbbbbb, 1]
            for index in range(3):
                _, error = yield from record_word(bus, index, abandoned[index],
                    first=index == 0, count=7, source=1)
                self.assertFalse(error)
            self.assertEqual((yield from read_word(bus, STATUS)) & 1, 1)
            self.assertEqual((yield from read_word(bus, COMMAND)), 0)
            yield from stage(bus, 0x60, 0x3500, b"\x12\x34\x56\x78\x9a")
            yield from submit(bus, 1, source=1)
            for index in range(3, 7):
                _, error = yield from record_word(bus, index, abandoned[index],
                    last=index == 6, count=7, source=1)
                self.assertFalse(error)
            self.assertEqual((yield from wait_done(bus, 1)), 5 << 16)
            self.assertEqual((yield from read_word(bus, NEXT)), 2)
        _, targets = self.simulate(bench)
        self.assertEqual(targets[0].transactions,
                         [(b"\xc0\x35\x00\x12\x34\x56\x78\x9a",)])

    def test_foreign_source_records_and_abort_cannot_modify_ingress_owner(self):
        def bench(dut, targets):
            bus, other = dut.engine.buses
            owner = [1, descriptor(0x60, 0x3500, 5), 0x78563412, 0x9a, 1]
            for index in range(2):
                yield from record_word(bus, index, owner[index], first=index == 0,
                                       count=5, source=1)
            # A matching-looking continuation from another source is ignored.
            yield from record_word(bus, 2, 0xffffffff, count=5, source=2)
            foreign = [1, descriptor(0x10, 0x4000, 1), 0xee, 1]
            yield from record(other, foreign, source=2)
            yield dut.engine.record_abort_sources.eq(1 << 2)
            yield
            yield dut.engine.record_abort_sources.eq(0)
            self.assertEqual((yield from read_word(bus, STATUS)) & 1, 1)
            for index in range(2, 5):
                _, error = yield from record_word(bus, index, owner[index],
                    last=index == 4, count=5, source=1)
                self.assertFalse(error)
            self.assertEqual((yield from wait_done(bus, 1)), 5 << 16)
        _, targets = self.simulate(bench, engine_options={"stretch_timeout_cycles": 240})
        self.assertEqual(targets[0].transactions,
                         [(b"\xc0\x35\x00\x12\x34\x56\x78\x9a",)])
        self.assertEqual(targets[1].transitions, [])

    def test_source_abort_between_words_or_on_commit_wins_without_id_consumption(self):
        for simultaneous in (False, True):
            with self.subTest(simultaneous=simultaneous):
                def bench(dut, targets):
                    bus = dut.engine.buses[1]
                    words = [1, descriptor(0x10, 0x4000, 1), 0xee, 1]
                    for index in range(3):
                        yield from record_word(bus, index, words[index],
                            first=index == 0, count=4, source=3)
                    if not simultaneous:
                        yield dut.engine.record_abort_sources.eq(1 << 3)
                        yield
                        yield dut.engine.record_abort_sources.eq(0)
                        yield
                    yield from record_word(bus, 3, 1, last=True, count=4, source=3,
                                           abort_sources=(1 << 3) if simultaneous else 0)
                    self.assertEqual((yield from read_word(bus, COMMAND)), 0)
                    self.assertEqual((yield from read_word(bus, NEXT)), 1)
                    self.assertEqual(targets[1].starts, [])
                    yield from stage(bus, 0x10, 0x3509, b"\x10")
                    yield from submit(bus, 1, source=3)
                    self.assertEqual((yield from wait_done(bus, 1)), 1 << 16)
                _, targets = self.simulate(bench)
                self.assertEqual(targets[1].transactions, [(b"\x20\x35\x09\x10",)])

    def test_busy_header_cannot_become_a_command_when_its_tail_arrives_after_completion(self):
        def bench(dut, targets):
            bus = dut.engine.buses[0]
            yield from stage(bus, 0x60, 0x3509, b"\x10")
            yield from submit(bus, 1, source=0)
            words = [2, descriptor(0x60, 0x4000, 1), 0xee, 2]
            yield from record_word(bus, 0, words[0], first=True, count=4, source=1)
            self.assertEqual((yield from wait_done(bus, 1)), 1 << 16)
            for index in range(1, 4):
                yield from record_word(bus, index, words[index], last=index == 3,
                                       count=4, source=1)
            for _ in range(30):
                yield
            self.assertEqual((yield from read_word(bus, COMMAND)), 1)
            self.assertEqual((yield from read_word(bus, NEXT)), 2)
            yield from stage(bus, 0x60, 0x3509, b"\x28")
            yield from submit(bus, 2, source=1)
            self.assertEqual((yield from wait_done(bus, 2)), 1 << 16)
        _, targets = self.simulate(bench)
        self.assertEqual(targets[0].transactions,
                         [(b"\xc0\x35\x09\x10",), (b"\xc0\x35\x09\x28",)])

    def test_ingress_lease_timeout_releases_busy_and_allows_the_same_next_id(self):
        def bench(dut, targets):
            bus = dut.engine.buses[0]
            yield from record_word(bus, 0, 1, first=True, count=4, source=2)
            # Diagnostic traffic from another source cannot itself discard
            # the partial owner's lease; expiration must be the cause.
            yield dut.engine.record_source.eq(3)
            self.assertEqual((yield from read_word(bus, STATUS)) & 1, 1)
            for _ in range(TIMEOUT // 4):
                yield
            self.assertEqual((yield from read_word(bus, STATUS)) & 1, 1)
            for _ in range(2 * TIMEOUT):
                yield
            self.assertEqual((yield from read_word(bus, STATUS)) & 1, 0)
            self.assertEqual((yield from read_word(bus, NEXT)), 1)
            self.assertEqual(targets[0].starts, [])
            tail = [descriptor(0x60, 0x4000, 1), 0xee, 1]
            for index, value in enumerate(tail, start=1):
                yield from record_word(bus, index, value, last=index == 3,
                                       count=4, source=2)
            self.assertEqual((yield from read_word(bus, COMMAND)), 0)
            self.assertEqual(targets[0].starts, [])
            yield from stage(bus, 0x60, 0x3509, b"\x28")
            yield from submit(bus, 1, source=2)
            self.assertEqual((yield from wait_done(bus, 1)), 1 << 16)
        _, targets = self.simulate(bench)
        self.assertEqual(targets[0].transactions, [(b"\xc0\x35\x09\x28",)])

    def test_new_first_can_replace_camera_and_later_abort_does_not_cancel_committed_wire_work(self):
        def bench(dut, targets):
            bus, other = dut.engine.buses
            yield from record_word(bus, 0, 1, first=True, count=4, source=1)
            yield from record_word(bus, 1, descriptor(0x60, 0x4000, 1), count=4, source=1)
            # A new complete record from the same source replaces its partial
            # record even when it selects the other physical camera aperture.
            yield from stage(other, 0x10, 0x3509, b"\x28")
            yield from submit(other, 1, source=1)
            self.assertEqual((yield dut.engine.busy), 1)
            yield dut.engine.record_abort_sources.eq(1 << 1)
            yield
            yield dut.engine.record_abort_sources.eq(0)
            self.assertEqual((yield from wait_done(other, 1)), 1 << 16)
            self.assertEqual((yield from read_word(other, NEXT)), 2)
        _, targets = self.simulate(bench)
        self.assertEqual(targets[0].transactions, [])
        self.assertEqual(targets[1].transactions, [(b"\x20\x35\x09\x28",)])

    def test_fresh_ingress_invalidates_shared_result_and_foreign_old_record_cannot_overwrite_it(self):
        def bench(dut, targets):
            bus = dut.engine.buses[0]
            targets[0].memory.update({0x300a: 0x92, 0x300b: 0x81})
            yield from stage(bus, 0x60, 0x300a, length=2, read=True)
            yield from submit(bus, 1, source=0)
            self.assertEqual((yield from wait_done(bus, 1)), 2 << 16)
            self.assertEqual((yield from read_word(bus, RX)) & 0xffff, 0x8192)
            old = [1, descriptor(0x60, 0x4000, 1), 0xdeadbeef, 1]
            yield from record(bus, old, source=0)
            self.assertEqual((yield from read_word(bus, STATUS)) & 3, 2)
            yield from record_word(bus, 0, 2, first=True, count=4, source=1)
            yield dut.engine.record_source.eq(2)
            self.assertEqual((yield from read_word(bus, STATUS)) & 3, 1)
            self.assertEqual((yield from read_word(bus, RX)) & 0xffff, 0x8192)
            yield from record(bus, old, source=0)
            self.assertEqual((yield from read_word(bus, STATUS)) & 3, 1)
            self.assertEqual((yield from read_word(bus, RX)) & 0xffff, 0x8192)
            yield from record_word(bus, 1, descriptor(0x60, 0x3509, 1), count=4, source=1)
            yield from record_word(bus, 2, 0x28, count=4, source=1)
            yield dut.engine.record_abort_sources.eq(1 << 1)
            yield
            yield dut.engine.record_abort_sources.eq(0)
            yield dut.engine.record_source.eq(2)
            self.assertEqual((yield from read_word(bus, STATUS)) & 3, 0)
            self.assertEqual((yield from read_word(bus, COMMAND)), 1)
            self.assertEqual((yield from read_word(bus, NEXT)), 2)
            self.assertEqual((yield from read_word(bus, RX)) & 255, 0x28)
            # An old completion must not become valid again after its shared
            # data buffer was used by a subsequently aborted ingress record.
            yield from record(bus, old, source=0)
            self.assertEqual((yield from read_word(bus, STATUS)) & 3, 0)
            yield from stage(bus, 0x60, 0x3509, b"\x28")
            yield from submit(bus, 2, source=1)
            self.assertEqual((yield from wait_done(bus, 2)), 1 << 16)
        _, targets = self.simulate(bench, engine_options={"stretch_timeout_cycles": 240})
        self.assertEqual(targets[0].transactions,
                         [(b"\xc0\x30\x0a", b"\xc1"), (b"\xc0\x35\x09\x28",)])

    def test_register_wrap_and_manual_pin_ownership_reject_without_consuming_id(self):
        def bench(dut, targets):
            bus = dut.engine.buses[0]
            yield from stage(bus, 0x60, 0xffff, b"\x12\x34")
            yield from submit(bus, 1)
            self.assertEqual((yield from read_word(bus, NEXT)), 1)
            yield from stage(bus, 0x60, 0xffff, b"\x56")
            yield from write_word(bus, CONTROL, 1)
            yield from submit(bus, 1)
            self.assertEqual((yield from read_word(bus, NEXT)), 1)
            self.assertEqual(targets[0].starts, [])
            yield from write_word(bus, CONTROL, 0)
            yield from submit(bus, 1)
            self.assertEqual((yield from wait_done(bus, 1)), 1 << 16)
        _, targets = self.simulate(bench)
        self.assertEqual(targets[0].transactions, [(b"\xc0\xff\xff\x56",)])

    def test_slow_mode_extends_both_serial_phases(self):
        widths = []
        for slow in (False, True):
            def bench(dut, targets):
                bus = dut.engine.buses[0]
                yield from stage(bus, 0x60, 0x3509, b"\x10", slow=slow)
                yield from submit(bus, 1)
                self.assertEqual((yield from wait_done(bus, 1)), 1 << 16)
            _, targets = self.simulate(bench)
            target = targets[0]
            self.assertEqual(target.transactions, [(b"\xc0\x35\x09\x10",)])
            # Ignore the initial START low/high intervals, whose FSM overhead
            # differs from an ordinary clock pulse; compare full pulse minima.
            widths.append((min(target.low_widths), min(target.high_widths)))
        self.assertGreaterEqual(widths[1][0], 4 * LOW)
        self.assertGreaterEqual(widths[1][1], 4 * HIGH)
        self.assertGreater(widths[1][0], widths[0][0])
        self.assertGreater(widths[1][1], widths[0][1])

    def test_stretched_clock_gets_full_high_period_after_actual_pin_rises(self):
        def bench(dut, targets):
            bus = dut.engine.buses[0]
            yield from stage(bus, 0x60, 0x3509, b"\x28")
            yield from submit(bus, 1)
            self.assertEqual((yield from wait_done(bus, 1)) & 255, 0)
        _, targets = self.simulate(bench, target_options=(
            {"stretch_after_rise": 3, "stretch_cycles": 17}, {}))
        target = targets[0]
        self.assertEqual(target.transactions, [(b"\xc0\x35\x09\x28",)])
        self.assertEqual(len(target.stretched_high_widths), 1)
        self.assertGreaterEqual(target.stretched_high_widths[0], HIGH)
        self.assertGreaterEqual(min(target.high_widths), HIGH)
        self.assertGreaterEqual(min(target.low_widths), LOW)

    def test_stretch_timeout_is_bounded_releases_pins_and_retains_error(self):
        def bench(dut, targets):
            bus = dut.engine.buses[0]
            yield from stage(bus, 0x60, 0x3509, b"\x28")
            yield from submit(bus, 1)
            result = yield from wait_done(bus, 1, limit=100)
            self.assertEqual(result & 255, 2)
            self.assertEqual((result >> 8) & 255, 0)
            self.assertEqual((result >> 16) & 255, 0)
            for pin in (*dut.engine.scl, *dut.engine.sda):
                self.assertEqual((yield pin.oe), 0)
            yield from submit(bus, 1)
            self.assertEqual((yield from read_word(bus, RESULT)), result)
            self.assertEqual((yield from read_word(bus, NEXT)), 2)
        _, targets = self.simulate(bench, target_options=(
            {"stretch_after_rise": 3, "stretch_cycles": -1}, {}))
        self.assertEqual(len(targets[0].starts), 1)
        self.assertEqual(targets[1].transitions, [])

    def test_stretch_at_ack_repeated_start_and_stop_waits(self):
        for name, read, after_rise in (("ack", False, 8),
                                      ("repeated_start", True, 27),
                                      ("stop", False, 36)):
            with self.subTest(phase=name):
                def bench(dut, targets):
                    bus = dut.engine.buses[0]
                    targets[0].memory[0x3509] = 0x28
                    if read:
                        yield from stage(bus, 0x60, 0x3509, length=1, read=True)
                    else:
                        yield from stage(bus, 0x60, 0x3509, b"\x28")
                    yield from submit(bus, 1)
                    self.assertEqual((yield from wait_done(bus, 1)), 1 << 16)
                    if read:
                        self.assertEqual((yield from read_word(bus, RX)) & 255, 0x28)
                _, targets = self.simulate(bench, target_options=(
                    {"stretch_after_rise": after_rise, "stretch_cycles": 17}, {}))
                target = targets[0]
                expected = (b"\xc0\x35\x09", b"\xc1") if read else (b"\xc0\x35\x09\x28",)
                self.assertEqual(target.transactions, [expected])
                self.assertEqual(len(target.stretched_rises), 1)
                if name == "stop":
                    self.assertGreaterEqual(target.stops[-1] - target.stretched_rises[0], HIGH)
                else:
                    self.assertGreaterEqual(target.stretched_high_widths[0], HIGH)

    def test_timeout_phase_and_completed_prefix_accounting(self):
        # Includes waits before the first ACK, repeated START, STOP after a
        # completed payload, and the middle of the second write/read byte.
        cases = (("ack", False, 2, 8, 0, 0),
                 ("repeated_start", True, 2, 27, 3, 0),
                 ("stop", False, 1, 36, 4, 1),
                 ("partial_write", False, 2, 39, 4, 1),
                 ("read_ack", True, 2, 45, 4, 0),
                 ("partial_read", True, 2, 49, 5, 1))
        for name, read, length, after_rise, fail_index, completed in cases:
            with self.subTest(phase=name):
                def bench(dut, targets):
                    bus = dut.engine.buses[0]
                    targets[0].memory.update({0x300a: 0x92, 0x300b: 0x81})
                    if read:
                        yield from stage(bus, 0x60, 0x300a, length=length, read=True)
                    else:
                        yield from stage(bus, 0x60, 0x300a, b"\x92\x81"[:length])
                    yield from submit(bus, 1)
                    result = yield from wait_done(bus, 1, limit=400)
                    self.assertEqual(result, 2 | (fail_index << 8) | (completed << 16))
                    if read and completed:
                        self.assertEqual((yield from read_word(bus, RX)) & 255, 0x92)
                    for pin in (*dut.engine.scl, *dut.engine.sda):
                        self.assertEqual((yield pin.oe), 0)
                    yield from submit(bus, 1)
                    self.assertEqual((yield from read_word(bus, RESULT)), result)
                    self.assertEqual((yield from read_word(bus, NEXT)), 2)
                _, targets = self.simulate(bench, target_options=(
                    {"stretch_after_rise": after_rise, "stretch_cycles": -1}, {}))
                self.assertEqual(targets[0].stops, [])
                self.assertEqual(targets[1].transitions, [])

    def test_continuous_wishbone_record_commits_once_and_diagnostics_do_not_consume_it(self):
        payload = bytes(range(16))
        def bench(dut, targets):
            bus = dut.engine.buses[0]
            yield from stage(bus, 0x60, 0x4000, payload)
            self.assertEqual((yield from submit(bus, 1, continuous=True)), [False] * 7)
            self.assertEqual((yield from submit(bus, 1, continuous=True)), [False] * 7)
            reads = yield from continuous_transfers(bus, [(STATUS, None), (COMMAND, None),
                                                           (NEXT, None), (TX, 0)])
            self.assertEqual([error for _, error in reads], [False] * 3 + [True])
            self.assertTrue(reads[0][0] & 1)
            self.assertEqual((reads[1][0], reads[2][0]), (1, 2))
            self.assertEqual((yield from wait_done(bus, 1)), 16 << 16)
            reads = yield from continuous_transfers(bus, [(DONE, None), (RESULT, None),
                                                           (RX, None), (NEXT, None)])
            self.assertTrue(all(not error for _, error in reads))
            self.assertEqual([value for value, _ in reads], [1, 16 << 16, 0x03020100, 2])
        _, targets = self.simulate(bench)
        self.assertEqual(targets[0].transactions, [(b"\xc0\x40\x00" + payload,)])

    def test_busy_bus_is_reported_without_clocking_target_or_holding_pins(self):
        def bench(dut, targets):
            bus = dut.engine.buses[1]
            yield from stage(bus, 0x10, 0x3509, b"\x10")
            yield from submit(bus, 1)
            result = yield from wait_done(bus, 1, limit=100)
            self.assertEqual(result & 255, 3)
            self.assertEqual((result >> 16) & 255, 0)
            self.assertEqual((yield dut.engine.scl[1].oe), 0)
            self.assertEqual((yield dut.engine.sda[1].oe), 0)
            self.assertEqual(targets[1].transactions, [])
        _, targets = self.simulate(bench, target_options=({}, {"stuck_sda": True}))
        self.assertTrue(all(scl for _, scl, _ in targets[1].transitions))

    def test_sda_held_after_payload_does_not_report_successful_stop(self):
        for nack_at, expected in ((None, 3 | (4 << 8) | (1 << 16)),
                                  (3, 1 | (3 << 8))):
            with self.subTest(nack_at=nack_at):
                def bench(dut, targets):
                    bus = dut.engine.buses[0]
                    yield from stage(bus, 0x60, 0x3509, b"\x28")
                    yield from submit(bus, 1)
                    result = yield from wait_done(bus, 1, limit=200)
                    self.assertEqual(result, expected)
                    for pin in (*dut.engine.scl, *dut.engine.sda):
                        self.assertEqual((yield pin.oe), 0)
                    self.assertEqual((yield from read_word(bus, PINS)) & 3, 1)
                    yield from submit(bus, 1)
                    self.assertEqual((yield from read_word(bus, RESULT)), result)
                _, targets = self.simulate(bench, target_options=(
                    {"nack_at": nack_at, "hold_sda_after_rise": 36}, {}))
                self.assertEqual(targets[0].stops, [])
                self.assertEqual(targets[1].transitions, [])


if __name__ == "__main__":
    unittest.main()
