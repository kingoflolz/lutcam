"""Software I2C/SCCB over the LUTCam two-register Wishbone pin interface.

The bus argument implements the scalar ``read(address)`` / ``write(address,
value)`` API of LiteX RemoteClient. Each write is verified by a control-register
readback before its settling delay. Capable Etherbone bridges carry both in one
record to avoid dropping an immediate second packet. Software must be the sole
owner of this bus while a transaction runs. The interface is open drain: one pulls low, zero
releases. Only call this helper against hardware after explicitly selecting
the camera and device address; it never guesses a sensor initialization table.
The CLI leaves streaming, reset and synchronization registers to the paired
capture coordinator. Other camera writes and bus recovery require a stopped,
drained pair.
"""

from __future__ import annotations

import argparse
import math
import time


CAMERA_I2C_BASES = frozenset((0x30020000, 0x30021000))
PAIRED_SENSOR_REGISTERS = frozenset((
    0x0100, 0x0103, 0x0106, 0x3006, 0x3027, 0x3030, 0x3666, 0x3823, 0x4f00,
    *range(0x4311, 0x4317),
))


class I2CError(RuntimeError):
    """An I2C transaction did not complete."""


class I2CNack(I2CError):
    """The addressed device did not acknowledge a byte."""


class I2CTimeout(I2CError):
    """SCL did not rise before the clock-stretch timeout."""


class BitBangI2C:
    """Drive a camera's minimal Wishbone I2C interface from software.

    Offset 0: bit 0 pulls SCL low; bit 1 pulls SDA low.
    Offset 4: bits 0 and 1 return the observed SCL and SDA levels.

    ``half_period`` is a minimum settling delay, not a promised bus frequency;
    Etherbone request latency normally dominates it. Do not have concurrent
    USB/Etherbone clients operate the same camera's pins.
    """

    def __init__(self, bus, address: int, *, half_period: float = 5e-6,
                 stretch_timeout: float = 0.1):
        if address < 0 or address % 4:
            raise ValueError("I2C base must be a nonnegative word-aligned address")
        if (not math.isfinite(half_period) or not 0 <= half_period <= 1
                or not math.isfinite(stretch_timeout) or not 0 < stretch_timeout <= 60):
            raise ValueError("I2C timings must be finite: half-period 0..1 s, stretch timeout (0,60] s")
        self.bus = bus
        self.address = address
        self.half_period = half_period
        self.stretch_timeout = stretch_timeout
        self._control = 0

    def _delay(self):
        time.sleep(self.half_period)

    def _lines(self, *, scl_low: bool, sda_low: bool):
        self._control = int(scl_low) | (int(sda_low) << 1)
        combined = getattr(self.bus, "write_read", None)
        supported = getattr(self.bus, "supports_ordered_write_read", None)
        if callable(combined) and callable(supported) and supported():
            # The checked-frame bridge can drop a separate immediate poll.
            # One mixed record orders the pin write before its readback. Under
            # the sole-owner, stop-and-wait contract, a timeout may retry only
            # this same level, never advance to the next edge without proof.
            # This does not protect against arbitrarily delayed UDP duplicates.
            first_timeout = None
            for attempt in range(3):
                try:
                    observed = int(combined(self.address, self._control, self.address))
                    break
                except TimeoutError as timeout:
                    if first_timeout is None:
                        first_timeout = timeout
                    try:
                        observed = int(self.bus.read(self.address))
                    except Exception as verification_error:
                        raise first_timeout from verification_error
                    if observed == self._control:
                        break
                    if observed not in range(4) or attempt == 2:
                        raise first_timeout from I2CError(
                            f"I2C control remains 0x{observed:x}; "
                            f"wanted 0x{self._control:x} after {attempt + 1} attempts"
                        )
        else:
            self.bus.write(self.address, self._control)
            # RemoteClient.write queues a write without waiting for a reply.
            # Complete a read through the same ordered connection before timing
            # this pin state; otherwise queued edges can collapse on the device.
            observed = self.bus.read(self.address)
        # A Wishbone error can return 0xffffffff. Masking it to two bits would
        # falsely confirm the state that pulls both pins low.
        observed = int(observed)
        if observed != self._control:
            raise I2CError("I2C control readback mismatch; another host may own the bus")
        self._delay()

    def _status(self) -> int:
        return int(self.bus.read(self.address + 4)) & 3

    def _clock_high(self):
        self._lines(scl_low=False, sda_low=bool(self._control & 2))
        deadline = time.monotonic() + self.stretch_timeout
        while not self._status() & 1:
            if time.monotonic() >= deadline:
                raise I2CTimeout("SCL held low after release")
            self._delay()
        # Measure the high period after the actual pin rises, including when
        # the device stretches the clock.
        self._delay()

    def release(self):
        """Release both pins (also the hardware reset state)."""
        self._lines(scl_low=False, sda_low=False)

    def start(self, *, repeated: bool = False):
        if repeated:
            self._lines(scl_low=True, sda_low=False)
        else:
            self.release()
        self._clock_high()
        if not self._status() & 2:
            raise I2CError("SDA held low before START; bus recovery may be needed")
        self._lines(scl_low=False, sda_low=True)
        self._lines(scl_low=True, sda_low=True)

    def stop(self):
        try:
            self._lines(scl_low=True, sda_low=True)
            self._clock_high()
            self.release()
        except (Exception, KeyboardInterrupt):
            self._release_after_error()
            raise

    def _release_after_error(self):
        # A second transport failure or interrupt must not hide the original
        # transaction error. Releasing is still attempted on both paths.
        try:
            self.release()
        except (Exception, KeyboardInterrupt):
            pass

    def write_bit(self, value: int):
        self._lines(scl_low=True, sda_low=not bool(value))
        self._clock_high()
        self._lines(scl_low=True, sda_low=not bool(value))

    def read_bit(self) -> int:
        self._lines(scl_low=True, sda_low=False)
        self._clock_high()
        value = (self._status() >> 1) & 1
        self._lines(scl_low=True, sda_low=False)
        return value

    def write_byte(self, value: int):
        if not 0 <= value <= 255:
            raise ValueError("I2C byte must fit in eight bits")
        for shift in range(7, -1, -1):
            self.write_bit((value >> shift) & 1)
        if self.read_bit():
            raise I2CNack(f"no ACK for byte 0x{value:02x}")

    def read_byte(self, *, ack: bool) -> int:
        value = 0
        for _ in range(8):
            value = (value << 1) | self.read_bit()
        self.write_bit(0 if ack else 1)
        return value

    @staticmethod
    def _validate_address(device: int, register: int):
        if not 0 <= device <= 0x7f:
            raise ValueError("device must be an unshifted seven-bit I2C address")
        if not 0 <= register <= 0xffff:
            raise ValueError("sensor register must fit in sixteen bits")

    def _register_address(self, device: int, register: int):
        self.write_byte(device << 1)
        self.write_byte(register >> 8)
        self.write_byte(register & 255)

    def _abort(self):
        # Preserve the original NACK/timeout even if an attempted STOP fails.
        try:
            self.stop()
        except (Exception, KeyboardInterrupt):
            # stop() already attempted to release both pins. A failed
            # transport must not replace the original error or interrupt.
            pass

    def write_register(self, device: int, register: int, data):
        """Write bytes after a big-endian 16-bit sensor register address."""
        self._validate_address(device, register)
        if isinstance(data, int):
            data = bytes([data])
        else:
            data = bytes(data)
        if not data:
            raise ValueError("at least one data byte is required")
        try:
            self.start()
            self._register_address(device, register)
            for value in data:
                self.write_byte(value)
        except (Exception, KeyboardInterrupt):
            self._abort()
            raise
        self.stop()

    def read_register(self, device: int, register: int, length: int = 1) -> bytes:
        """Read bytes using a register-address write and repeated START."""
        self._validate_address(device, register)
        if length < 1:
            raise ValueError("read length must be positive")
        try:
            self.start()
            self._register_address(device, register)
            self.start(repeated=True)
            self.write_byte((device << 1) | 1)
            data = bytes(self.read_byte(ack=index + 1 < length)
                         for index in range(length))
        except (Exception, KeyboardInterrupt):
            self._abort()
            raise
        self.stop()
        return data

    def recover(self):
        """Clock nine times with SDA released, then issue STOP.

        Recovery is explicit: a failed ordinary transaction never silently
        clocks another device or retries a write with possible side effects.
        """
        try:
            self.release()
            for _ in range(9):
                self._lines(scl_low=True, sda_low=False)
                self._clock_high()
                # A high SDA may be a data bit in an interrupted sensor read.
                # Finish the byte and its released-SDA NACK before STOP.
            self.stop()
            if self._status() != 3:
                raise I2CError("I2C bus remains stuck after recovery")
            self.release()
        except (Exception, KeyboardInterrupt):
            self._release_after_error()
            raise


def _integer(value):
    return int(value, 0)


def _validate_raw_write(device, register, data):
    """Validate the entire CLI write before opening any transport.

    This restriction deliberately lives outside BitBangI2C: the approved
    sensor configurator and paired coordinator need these register writes.
    """
    BitBangI2C._validate_address(device, register)
    data = bytes(data)
    if not data:
        raise ValueError("at least one data byte is required")
    end = register + len(data)
    if end > 0x10000:
        raise ValueError("sensor register write must not wrap the sixteen-bit address range")
    protected = sorted(value for value in PAIRED_SENSOR_REGISTERS
                       if register <= value < end)
    if protected:
        raise ValueError(
            f"Raw writes to sensor register 0x{protected[0]:04x} are disabled; "
            "use tools.lutcam.ov9281 configure for sensor configuration and "
            "tools.lutcam.capture_dual for paired start/stop."
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="localhost", help="LiteX server host")
    parser.add_argument("--port", type=int, default=1234, help="LiteX server TCP port")
    parser.add_argument("--csr-csv", help="optional generated LiteX CSR CSV")
    parser.add_argument("--i2c-base", type=_integer, choices=sorted(CAMERA_I2C_BASES), required=True,
                        help="camera Wishbone base: 0x30020000 or 0x30021000")
    parser.add_argument("--half-period", type=float, default=5e-6,
                        help="minimum I2C half-period in seconds")
    parser.add_argument("--stretch-timeout", type=float, default=0.1)
    actions = parser.add_subparsers(dest="action", required=True)
    for name in ("read", "write"):
        action = actions.add_parser(name)
        action.add_argument("device", type=_integer, help="unshifted seven-bit address")
        action.add_argument("register", type=_integer, help="sixteen-bit register address")
        if name == "read":
            action.add_argument("--length", type=int, default=1)
        else:
            action.add_argument("data", nargs="+", type=_integer, help="byte values")
    actions.add_parser("recover", help="explicit nine-clock bus recovery")
    args = parser.parse_args(argv)
    try:
        # Construction has no bus side effects; reject unsafe timing before
        # constructing the network transport, including for read and recovery.
        BitBangI2C(None, args.i2c_base, half_period=args.half_period,
                   stretch_timeout=args.stretch_timeout)
        if args.action == "write":
            _validate_raw_write(args.device, args.register, args.data)
    except ValueError as error:
        parser.error(str(error))
    from litex import RemoteClient
    bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv,
                       raise_on_timeout=True)
    bus.open()
    try:
        if args.i2c_base in CAMERA_I2C_BASES and args.action in ("write", "recover"):
            # Keep the low-level library usable by the paired coordinator,
            # but do not let the raw CLI alter either member while running.
            from .ov9281 import require_pair_idle
            require_pair_idle(bus)
        i2c = BitBangI2C(bus, args.i2c_base, half_period=args.half_period,
                        stretch_timeout=args.stretch_timeout)
        if args.action == "read":
            print(i2c.read_register(args.device, args.register, args.length).hex(" "))
        elif args.action == "write":
            i2c.write_register(args.device, args.register, args.data)
        else:
            i2c.recover()
    finally:
        bus.close()


if __name__ == "__main__":
    main()
