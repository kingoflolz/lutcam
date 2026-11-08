"""Bounded sensor transactions using the shared FPGA I2C engine.

One management client must own both camera buses: the hardware engine and its
command ID sequence are shared by the two camera apertures. GPIO recovery is
an explicit operation performed while the engine is idle.
"""

from __future__ import annotations

import math
import time

from .control import BitBangI2C, CAMERA_I2C_BASES, I2CError, I2CNack, I2CTimeout


ENGINE_FEATURE = 1 << 8
ENGINE_MAGIC = 0x49324301
MAGIC = 0x10
STATUS = 0x14
DESCRIPTOR = 0x18
COMMAND_ID = 0x1c
DONE_ID = 0x20
NEXT_ID = 0x24
RESULT = 0x28
CAPACITY = 0x2c
RX = 0x60
COMMAND_RECORD = 0x80
BUSY = 1
DONE = 2
CAPACITY_BYTES = 16
MAX_TRANSFER_BYTES = 256


def _validate_range(device, register, length):
    if type(device) is not int or not 0 <= device <= 0x7f:
        raise ValueError("device must be an unshifted seven-bit I2C address")
    if (type(register) is not int or not 0 <= register <= 0xffff
            or type(length) is not int or not 1 <= length <= MAX_TRANSFER_BYTES
            or register + length > 0x10000):
        raise ValueError("I2C transfer needs a sixteen-bit register range and 1..256 bytes")


def _write_payload(device, register, data):
    if type(data) is int:
        if not 0 <= data <= 255:
            raise ValueError("I2C data byte must be in 0..255")
        payload = bytes((data,))
    elif isinstance(data, bool):
        raise ValueError("I2C data must contain integer bytes")
    else:
        try:
            payload = bytes(data)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError("I2C data must contain integer bytes") from error
    _validate_range(device, register, len(payload))
    return payload


class HardwareI2C:
    """Stop-and-wait transactions, with no automatic mutation replay.

    Each request contains its ID, descriptor, payload and matching final ID in
    one incrementing write record. The FPGA validates the complete record
    before execution; delayed copies cannot overwrite another request's data.
    A lost response can be resolved by reading its retained result; the command
    itself is never retried. Recovery is always an explicit separate operation.
    """

    def __init__(self, bus, address, *, slow=False, timeout=0.2,
                 poll_interval=0.0, half_period=5e-6, stretch_timeout=0.1,
                 clock=time.monotonic, sleep=time.sleep):
        if type(address) is not int or address not in CAMERA_I2C_BASES:
            raise ValueError("hardware I2C requires an explicit camera aperture")
        if type(slow) is not bool:
            raise ValueError("slow must be a boolean")
        if (not math.isfinite(timeout) or not 0 < timeout <= 60
                or not math.isfinite(poll_interval) or not 0 <= poll_interval <= timeout):
            raise ValueError("I2C timeout and poll interval must be finite and bounded")
        self.pins = BitBangI2C(bus, address, half_period=half_period,
                                 stretch_timeout=stretch_timeout)
        self.bus, self.address = bus, address
        self.camera = (address - 0x30020000) // 0x1000
        self.slow, self.timeout, self.poll_interval = slow, timeout, poll_interval
        self.clock, self.sleep = clock, sleep
        self._ordered = None

    def _read(self, offset, count=None):
        if count is None:
            value = int(self.bus.read(self.address + offset))
            if not 0 <= value <= 0xffffffff:
                raise I2CError("I2C engine returned a non-uint32 register value")
            return value
        values = list(self.bus.read(self.address + offset, length=count))
        if len(values) != count or any(type(value) is not int or not 0 <= value <= 0xffffffff
                                       for value in values):
            raise I2CError("I2C engine returned an invalid register response")
        return values

    def _write_read(self, offset, values, read_offset, count=None):
        if self._ordered is None:
            supported = getattr(self.bus, "supports_ordered_write_read", None)
            self._ordered = (callable(getattr(self.bus, "write_read", None))
                             and callable(supported) and supported())
        if self._ordered:
            arguments = {} if count is None else {"length": count}
            return self.bus.write_read(self.address + offset, values,
                                       self.address + read_offset, **arguments)
        self.bus.write(self.address + offset, values)
        return self._read(read_offset, count)

    @staticmethod
    def _check_status(status):
        if status & ~0xf:
            raise I2CError(f"invalid I2C engine status 0x{status:08x}")

    def _idle(self):
        # Include NEXT_ID in the same response as the busy check. This is a
        # sole-owner protocol, not a lock against another management client.
        state = self._read(STATUS, 5)
        self._check_status(state[0])
        if state[0] & BUSY:
            raise I2CError("shared I2C engine is busy; no command was submitted")
        return state[4]

    def _transaction(self, device, register, count, payload=None):
        read = payload is None
        command_id = self._idle()
        descriptor = (device | (int(read) << 7) | (register << 8)
                      | ((count - 1) << 24) | (int(self.slow) << 28))
        record = [command_id, descriptor]
        if payload is not None:
            padded = payload + bytes((-len(payload)) % 4)
            record.extend(int.from_bytes(padded[index:index + 4], "little")
                          for index in range(0, len(padded), 4))
        record.append(command_id)

        deadline = self.clock() + self.timeout
        commit_timeout = None
        try:
            state = list(self._write_read(COMMAND_RECORD, record, STATUS, 6))
        except TimeoutError as error:
            # Reading is safe; a new commit is not. In particular, do not
            # replace this ID by NEXT_ID after an uncertain acknowledgment.
            commit_timeout = error
            # The transport's reply timeout may consume this whole budget.
            # Make one bounded read-only attempt to resolve the retained ID
            # even then. Its transport timeout is additional to the engine
            # deadline; a failed USB stream can still raise ConnectionError.
            try:
                state = self._read(STATUS, 6)
            except TimeoutError:
                state = None
        while True:
            if state is not None:
                if len(state) != 6:
                    raise I2CError("I2C command returned a partial status response")
                status, _, current, done_id, _, result = state
                self._check_status(status)
                if current != command_id:
                    raise I2CError(
                        f"I2C command {command_id:#x} was not accepted; not replaying it") from commit_timeout
                if bool(status & 4) != bool(self.camera) or bool(status & 8) != read:
                    raise I2CError("I2C command ownership changed; another client may own the engine")
                if not status & BUSY:
                    if not status & DONE or done_id != command_id:
                        raise I2CError("I2C command lacks its matching completion; not replaying it")
                    self._check_result(result, device, register, count)
                    break
            if self.clock() >= deadline:
                raise I2CTimeout(
                    f"I2C command {command_id:#x} completion timed out; outcome is uncertain, not replayed"
                ) from commit_timeout
            if self.poll_interval:
                self.sleep(min(self.poll_interval, max(0, deadline - self.clock())))
            try:
                state = self._read(STATUS, 6)
            except TimeoutError as error:
                commit_timeout = error
                state = None
        if read:
            words = self._read(RX, (count + 3) // 4)
            return b"".join(word.to_bytes(4, "little") for word in words)[:count]
        return None

    def _check_result(self, result, device, register, count):
        error, byte_index, completed = result & 255, (result >> 8) & 255, (result >> 16) & 255
        context = (f"camera {self.camera}, device 0x{device:02x}, register 0x{register:04x}, "
                   f"wire byte {byte_index}, completed {completed}/{count} payload bytes")
        if error == 1:
            raise I2CNack(f"sensor did not acknowledge ({context})")
        if error == 2:
            raise I2CTimeout(f"sensor held SCL low past the hardware timeout ({context})")
        if error == 3:
            raise I2CError(f"SDA held low while the bus should be idle; explicit recovery may be needed ({context})")
        if error or result >> 24 or completed != count:
            raise I2CError(f"invalid I2C completion 0x{result:08x} ({context})")

    def read_register(self, device, register, length=1):
        _validate_range(device, register, length)
        result = bytearray()
        for offset in range(0, length, CAPACITY_BYTES):
            result.extend(self._transaction(device, register + offset,
                                             min(CAPACITY_BYTES, length - offset)))
        return bytes(result)

    def write_register(self, device, register, data):
        payload = _write_payload(device, register, data)
        for offset in range(0, len(payload), CAPACITY_BYTES):
            chunk = payload[offset:offset + CAPACITY_BYTES]
            self._transaction(device, register + offset, len(chunk), chunk)

    def release(self):
        self._idle()
        self.pins.release()

    def recover(self):
        """Explicit nine-clock recovery, only while the engine is idle."""
        self._idle()
        self.pins.recover()


class CameraI2C:
    """Lazily select a backend without I/O during sensor construction."""

    def __init__(self, bus, address, *, half_period=5e-6, stretch_timeout=0.1,
                 slow=False, timeout=0.2):
        if type(address) is not int or address not in CAMERA_I2C_BASES:
            raise ValueError("camera I2C requires an explicit camera aperture")
        # Validate both backends now, before any status access.
        self._hardware = HardwareI2C(bus, address, half_period=half_period,
                                      stretch_timeout=stretch_timeout, slow=slow,
                                      timeout=timeout)
        self.bus, self.address = bus, address
        self._backend = None

    def _select(self):
        if self._backend is None:
            status = int(self.bus.read(self.address + 4))
            if status & ~(ENGINE_FEATURE | 3):
                raise I2CError(f"invalid camera I2C pin status 0x{status:08x}")
            if status & ENGINE_FEATURE:
                magic = self._hardware._read(MAGIC)
                capacity = self._hardware._read(CAPACITY)
                if magic != ENGINE_MAGIC or capacity != CAPACITY_BYTES:
                    raise I2CError(
                        f"unsupported I2C engine identity/capacity {magic:#x}/{capacity}; no fallback attempted")
                self._backend = self._hardware
            else:
                raise I2CError("camera aperture does not expose the I2C engine")
        return self._backend

    @property
    def backend_name(self):
        if self._backend is None:
            return "undetected"
        return "hardware"

    def read_register(self, device, register, length=1):
        _validate_range(device, register, length)
        return self._select().read_register(device, register, length)

    def write_register(self, device, register, data):
        payload = _write_payload(device, register, data)
        self._select().write_register(device, register, payload)

    def release(self):
        self._select().release()

    def recover(self):
        self._select().recover()


def create_camera_i2c(bus, address, **options):
    """Return a lazy, backwards-compatible camera I2C backend."""
    return CameraI2C(bus, address, **options)
