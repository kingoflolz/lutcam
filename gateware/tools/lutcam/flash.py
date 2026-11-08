"""Bounded Etherbone access to the LUTCAM flash and remote-update blocks.

Only one management client may operate the board at a time. Flash command IDs
make a repeated commit harmless. Each complete page command is submitted
in one ordered Wishbone burst.
Normal write/update flows separately verify the programmed flash
contents. Golden and the running image are protected by both client and hardware.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import errno
import ipaddress
import json
import os
from pathlib import Path
import secrets
import socket
import struct
import time
import zlib

try:
    import termios
except ImportError:  # USB serial is also usable on non-POSIX hosts.
    termios = None

FLASH_BASE = 0x30025000
BOOT_BASE = 0x30026000
FLASH_MAGIC = 0x4C465331
BOOT_MAGIC = 0x4C425331
FLASH_SIZE = 0x1000000
SLOT_SIZE = 0x200000
IMAGE_CAPACITY = SLOT_SIZE - 0x1000
JEDEC_ID = 0xC86018
READ, PROGRAM, ERASE_4K, ERASE_64K, JEDEC, STATUS = range(1, 7)
ERRORS = {0: "none", 1: "range", 2: "protected", 3: "length", 4: "alignment",
          5: "opcode", 6: "write enable failed", 7: "flash busy timeout", 8: "command CRC mismatch"}




class FlashError(RuntimeError):
    """The operation failed or its outcome cannot be established."""


def _usb_generation_at(path):
    """Read the Linux USB device generation without opening its serial port."""
    path = Path(path)
    try:
        before = path.stat()
        generation = (before.st_ino, int((path / "busnum").read_text()),
                      int((path / "devnum").read_text()))
        return generation if path.stat().st_ino == before.st_ino else None
    except OSError as error:
        if error.errno in (errno.ENOENT, errno.ENODEV):
            return None
        raise


def _usb_device_generation(device):
    """Resolve a stable device alias to its tty inode and physical USB node."""
    try:
        node = Path(device).resolve(strict=True)
        before = node.stat()
        interface = (Path("/sys/class/tty") / node.name / "device").resolve(strict=True)
        for parent in (interface, *interface.parents):
            if (parent / "idVendor").exists():
                generation = _usb_generation_at(parent)
                after = node.stat()
                if generation is None or (before.st_dev, before.st_ino, before.st_rdev) != (after.st_dev, after.st_ino, after.st_rdev):
                    return None
                return {"node": str(node), "node_identity": (before.st_dev, before.st_ino, before.st_rdev),
                        "usb_path": str(parent), "generation": generation}
    except OSError as error:
        if error.errno in (errno.ENOENT, errno.ENODEV):
            return None
        raise
    return None


class EtherboneUDP:
    """Small direct transport with bounded reads, fresh sockets and peer checks.

    A new ephemeral source port and random reply sequence distinguish a new
    FPGA boot from delayed replies to an earlier connection. Writes are never
    retried here: only the register protocol can decide whether that is safe.
    """

    def __init__(self, target="192.168.1.50", port=1234, *, timeout=0.2, attempts=3):
        self.server = str(ipaddress.IPv4Address(target))
        if not 1 <= port <= 65535 or not 0 < timeout <= 60 or not 1 <= attempts <= 10:
            raise ValueError("invalid Etherbone port, timeout or attempt count")
        self.port, self.timeout, self.attempts = port, timeout, attempts
        self.socket = None
        self.sequence = secrets.randbelow(0x7FFFFFFF) + 1
        self.read_counter = 0
        self.max_words = 8
        self._ordered_write_read = None

    def configure_burst_limit(self, words):
        """Called only after the flash ABI's transport capability is checked."""
        if not 1 <= words <= 128:
            raise ValueError("Etherbone burst limit must be in 1..128 words")
        self.max_words = words
        return words

    @contextmanager
    def discovery_probe(self):
        """Temporarily use short, single-attempt identity reads during reboot."""
        timeout, attempts = self.timeout, self.attempts
        self.timeout, self.attempts = min(timeout, 0.025), 1
        try:
            if self.socket is not None:
                self.socket.settimeout(self.timeout)
            yield self
        finally:
            self.timeout, self.attempts = timeout, attempts
            if self.socket is not None:
                self.socket.settimeout(timeout)

    def open(self):
        if self.socket is None:
            self._ordered_write_read = None
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(("", 0))
            self.socket.settimeout(self.timeout)
        return self

    def close(self):
        self._ordered_write_read = None
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *unused):
        self.close()

    def _send(self, packet):
        if self.socket is None:
            raise FlashError("Etherbone socket is closed")
        packet.encode()
        self.socket.sendto(packet.bytes, (self.server, self.port))

    def _receive_words(self, address, count):
        from litex.tools.remote.etherbone import EtherbonePacket
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            self.socket.settimeout(max(0.001, deadline - time.monotonic()))
            try:
                raw, peer = self.socket.recvfrom(8192)
            except socket.timeout:
                break
            if peer != (self.server, self.port):
                continue
            try:
                response = EtherbonePacket(32, raw)
                response.decode()
                if len(response.records) != 1:
                    continue
                reply = response.records[0]
                if reply.writes is None or reply.writes.base_addr != self.sequence:
                    continue
                values = reply.writes.get_datas()
                if len(values) != count:
                    continue
            except (ValueError, AssertionError, IndexError, TypeError, AttributeError):
                continue
            return values
        raise socket.timeout(f"no matching Etherbone response at 0x{address:08x}")

    def read(self, address, length=None):
        from litex.tools.remote.etherbone import EtherbonePacket, EtherboneRecord, EtherboneReads
        count = 1 if length is None else length
        if address % 4 or not 1 <= count <= self.max_words:
            raise ValueError(f"Etherbone reads need an aligned address and one through {self.max_words} words")
        for _ in range(self.attempts):
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            self.read_counter += 1
            record = EtherboneRecord(addr_size=4)
            record.reads = EtherboneReads(addr_size=4, addrs=[address + 4*i for i in range(count)])
            record.reads.base_ret_addr = self.sequence
            record.rcount = count
            packet = EtherbonePacket(addr_width=32)
            packet.records = [record]
            self._send(packet)
            try:
                values = self._receive_words(address, count)
            except socket.timeout:
                continue
            return values[0] if length is None else values
        raise socket.timeout(f"no matching Etherbone response at 0x{address:08x}")

    def supports_ordered_write_read(self):
        """Recognize the LUTCam bridge that orders mixed records correctly.

        Camera clients do not otherwise discover the flash interface. Probe
        only its read-only identity words and cache the result on this socket;
        unrecognized endpoints keep separate write/read transactions.
        """
        if self._ordered_write_read is None:
            magic, version = self.read(FLASH_BASE, length=2)
            supported = False
            if magic == FLASH_MAGIC and version == 0:
                # The interface reserves its identity word as zero.
                # Check its real packet capabilities before using mixed records;
                # zero on a missing/different block must never enable mutation.
                buffer_size, max_words, buffer_offset = self.read(FLASH_BASE + 0x38, length=3)
                supported = buffer_size == 256 and buffer_offset == 0x100 and 68 <= max_words <= 128
            self._ordered_write_read = supported
        return self._ordered_write_read

    def write_read(self, write_address, values, read_address, length=None):
        """Send one standard mixed record, then await its correlated reply.

        This mutating operation never retries internally. The caller must use
        an idempotent command protocol and decide whether a replay is safe.
        The bridge supports ordered write/read sections in one record.
        """
        from litex.tools.remote.etherbone import EtherbonePacket, EtherboneRecord, EtherboneReads, EtherboneWrites
        words = [values] if isinstance(values, int) else list(values)
        count = 1 if length is None else length
        if (any(not 0 <= address <= 0xffffffff or address % 4 for address in (write_address, read_address))
                or not 1 <= len(words) <= self.max_words or not 1 <= count <= self.max_words
                or len(words) + count + 2 > 256
                or any(not 0 <= word <= 0xffffffff for word in words)
                or write_address + 4 * (len(words) - 1) > 0xffffffff
                or read_address + 4 * (count - 1) > 0xffffffff):
            raise ValueError("Etherbone mixed record exceeds its aligned uint32 or bounded word limits")
        self.sequence = (self.sequence + 1) & 0xffffffff
        self.read_counter += 1
        record = EtherboneRecord(addr_size=4)
        record.writes = EtherboneWrites(addr_size=4, base_addr=write_address, datas=iter(words))
        record.wcount = len(words)
        record.reads = EtherboneReads(addr_size=4, addrs=[read_address + 4 * index for index in range(count)])
        record.reads.base_ret_addr = self.sequence
        record.rcount = count
        packet = EtherbonePacket(addr_width=32)
        packet.records = [record]
        self._send(packet)
        result = self._receive_words(read_address, count)
        return result[0] if length is None else result

    def write(self, address, values):
        from litex.tools.remote.etherbone import EtherbonePacket, EtherboneRecord, EtherboneWrites
        words = [values] if isinstance(values, int) else list(values)
        if address % 4 or not 1 <= len(words) <= self.max_words or any(not 0 <= v <= 0xFFFFFFFF for v in words):
            raise ValueError(f"Etherbone writes need an aligned address and one through {self.max_words} uint32 words")
        record = EtherboneRecord(addr_size=4)
        record.writes = EtherboneWrites(addr_size=4, base_addr=address, datas=iter(words))
        record.wcount = len(words)
        packet = EtherbonePacket(addr_width=32)
        packet.records = [record]
        self._send(packet)


class UARTBoneUSB:
    """Bounded UARTBone framing over LUNA's explicitly selected CDC device.

    A partial response destroys framing. That connection is closed, rather
    than replaying an untagged request and confusing old and new replies.
    Reopening waits for UARTBone's 100 ms partial-command timeout.
    """

    def __init__(self, device, *, timeout=0.2, baudrate=115200, max_words=8, serial_factory=None,
                 clock=time.monotonic, sleep=time.sleep):
        if not device or not 0 < timeout <= 60 or baudrate <= 0:
            raise ValueError("USB needs an explicit CDC device and positive timeout/baudrate")
        self.device, self.timeout, self.baudrate = device, timeout, baudrate
        self.serial_factory = serial_factory
        self.clock, self.sleep = clock, sleep
        self.port = None
        self.read_counter = 0
        self.configure_burst_limit(max_words)

    def configure_burst_limit(self, words):
        # UARTBone's count field is eight bits. CDC delivers a packet without
        # UART line-rate serialization; the serial baud setting is not a USB
        # throughput limit. FlashClient still caps negotiated bursts at 128.
        if not 1 <= words <= 255:
            raise ValueError("UARTBone burst limit must be in 1..255 words")
        self.max_words = words
        return words

    def open(self):
        if self.port is None:
            factory = self.serial_factory
            if factory is None:
                import serial
                factory = serial.serial_for_url
            try:
                self.port = factory(self.device, baudrate=self.baudrate, timeout=self.timeout,
                                    write_timeout=self.timeout, exclusive=True)
                self.port.reset_output_buffer()
                self.sleep(0.15)
                self.port.reset_input_buffer()
            except Exception as error:
                self.close()
                # Some pyserial POSIX operations expose termios.error, which
                # is not an OSError. Re-enumeration is a retryable disconnect;
                # other exception types must retain their original meaning.
                if termios is not None and isinstance(error, termios.error):
                    raise OSError(*error.args) from error
                raise
        return self

    def close(self):
        if self.port is not None:
            self.port.close()
            self.port = None

    def prepare_reboot(self):
        """Capture this USB instance before committing a reboot request.

        Linux can fail to enumerate LUNA after an early CDC close. The returned
        waiter holds the original descriptor without I/O until physical removal,
        then waits for a replacement generation before any new open is allowed.
        """
        if self.port is None:
            raise FlashError("USB stream is closed before reboot")
        old = _usb_device_generation(self.device)
        if old is None:
            raise FlashError("cannot establish Linux USB generation before reboot")
        opened = os.fstat(self.port.fileno())
        if (opened.st_dev, opened.st_ino, opened.st_rdev) != old["node_identity"]:
            raise FlashError("USB device alias no longer identifies the opened descriptor")

        def wait(deadline, *, clock=time.monotonic, sleep=time.sleep):
            def pause():
                remaining = deadline - clock()
                if remaining > 0:
                    sleep(min(0.005, remaining))

            while clock() < deadline:
                # A disappearing by-id symlink alone is insufficient: retain
                # the fd until the pinned physical USB generation is gone.
                if _usb_generation_at(old["usb_path"]) != old["generation"]:
                    self.close()
                    break
                pause()
            else:
                raise FlashError("USB reboot timed out waiting for physical device removal")
            while clock() < deadline:
                current = _usb_device_generation(self.device)
                if (current is not None and current["usb_path"] == old["usb_path"] and
                        current["generation"] != old["generation"] and clock() < deadline):
                    return {"old_generation": old, "new_generation": current}
                pause()
            raise FlashError("USB detached but did not re-enumerate before the reboot timeout")

        return wait

    def __enter__(self):
        return self.open()

    def __exit__(self, *unused):
        self.close()

    def _send(self, payload, *, retain_on_error=False):
        if self.port is None:
            raise ConnectionError("USB stream is closed; reconnect after a framing error")
        try:
            # One complete packet, bounded write time, and no blind resend.
            if self.port.write(payload) != len(payload):
                raise TimeoutError("partial UARTBone packet write; outcome is uncertain")
        except Exception:
            if not retain_on_error:
                self.close()
            raise

    def _header(self, command, address, count):
        if not 0 <= address <= 0xFFFFFFFF or address % 4 or not 1 <= count <= self.max_words:
            raise ValueError(f"UARTBone needs an aligned uint32 address and one through {self.max_words} words")
        return bytes([command, count]) + (address // 4).to_bytes(4, "big")

    def read(self, address, length=None):
        count = 1 if length is None else length
        self._send(self._header(2, address, count))
        self.read_counter += 1
        deadline = self.clock() + self.timeout
        raw = bytearray()
        try:
            while len(raw) < 4 * count:
                remaining = deadline - self.clock()
                if remaining <= 0:
                    raise TimeoutError("UARTBone read timed out; framing is no longer trusted")
                self.port.timeout = remaining
                fragment = self.port.read(4 * count - len(raw))
                if not fragment:
                    raise TimeoutError("UARTBone read timed out; framing is no longer trusted")
                raw.extend(fragment)
        except Exception:
            self.close()
            raise
        words = [int.from_bytes(raw[i:i+4], "big") for i in range(0, len(raw), 4)]
        return words[0] if length is None else words

    def write(self, address, values):
        words = [values] if isinstance(values, int) else list(values)
        header = self._header(1, address, len(words))
        if any(not 0 <= value <= 0xFFFFFFFF for value in words):
            raise ValueError("UARTBone write data must fit uint32")
        self._send(header + b"".join(value.to_bytes(4, "big") for value in words))

    def write_reboot_command(self, address, command_id):
        """Send once, retaining the fd even if completion of this send is unknown.

        Pyserial can raise after queuing every byte. The prepared reboot waiter
        must resolve physical removal before closing; this command is never
        replayed and no subsequent serial I/O is permitted on this connection.
        """
        self._send(self._header(1, address, 1) + command_id.to_bytes(4, "big"), retain_on_error=True)


def verified_write(bus, address, values, *, attempts=3, max_words=8):
    """Retry only non-triggering register/buffer writes and confirm every word."""
    scalar = isinstance(values, int)
    expected = [values] if scalar else list(values)
    if not 1 <= max_words <= 128 or not 1 <= len(expected) <= max_words:
        raise ValueError("write verification exceeds the negotiated word limit")
    for _ in range(attempts):
        try:
            bus.write(address, values)
            observed = bus.read(address) if scalar else bus.read(address, length=len(expected))
            if ([observed] if scalar else observed) == expected:
                return
        except (OSError, TimeoutError):
            pass
    raise FlashError(f"register/buffer readback failed at 0x{address:08x}; command not committed")


class FlashClient:
    def __init__(self, bus, *, base=FLASH_BASE, timeout=30.0, poll_interval=0.0001,
                 max_poll_interval=0.005,
                 clock=time.monotonic, sleep=time.sleep):
        if timeout <= 0 or not 0 < poll_interval <= max_poll_interval:
            raise ValueError("invalid flash timeout")
        self.bus, self.base = bus, base
        self.timeout, self.poll_interval = timeout, poll_interval
        self.max_poll_interval = max_poll_interval
        self.clock, self.sleep = clock, sleep
        self.uncertain = False
        self.transfer_words, self.buffer_size, self.buffer_offset = 8, 256, 0x100
        if self._read(0) != FLASH_MAGIC:
            raise FlashError("unsupported flash controller magic")
        if self._read(4) != 0:
            raise FlashError("nonzero reserved flash identity word")
        if self._read(0x0C) != FLASH_SIZE:
            raise FlashError("unexpected flash capacity")
        self.image_slot = self._read(0x2C)
        if self.image_slot not in range(4):
            raise FlashError("invalid running image slot identity")
        buffer_size, advertised_words, buffer_offset = (self._read(offset) for offset in (0x38, 0x3C, 0x40))
        if buffer_size != 256 or buffer_offset != 0x100:
            raise FlashError("invalid page-record flash buffer capacity or offset")
        if not 68 <= advertised_words <= 128:
            raise FlashError("page-record flash requires a transport burst of 68 through 128 words")
        self.buffer_size, self.buffer_offset = buffer_size, buffer_offset
        configure = getattr(bus, "configure_burst_limit", None)
        if configure is None:
            raise FlashError("page-record flash requires transport burst negotiation")
        negotiated = configure(advertised_words)
        if not isinstance(negotiated, int) or not 68 <= negotiated <= advertised_words:
            raise FlashError("page-record flash requires a negotiated transport burst of at least 68 words")
        self.transfer_words = negotiated

    def _read(self, offset):
        return int(self.bus.read(self.base + offset))

    def _read_words(self, offset, count):
        result = []
        for start in range(0, count, self.transfer_words):
            size = min(self.transfer_words, count - start)
            words = list(self.bus.read(self.base + offset + 4*start, length=size))
            if len(words) != size:
                raise FlashError(f"short register burst at 0x{self.base + offset + 4*start:08x}")
            result.extend(int(word) for word in words)
        return result

    def _range(self, address, length, *, mutate=False):
        if not isinstance(address, int) or not isinstance(length, int) or length <= 0 or address < 0 or address + length > FLASH_SIZE:
            raise FlashError("flash address/length is outside the 16 MiB device")
        if mutate:
            active = self.image_slot * SLOT_SIZE
            if address < SLOT_SIZE or (address < active + SLOT_SIZE and address + length > active):
                raise FlashError("golden and the running image slot cannot be erased or written")

    def command(self, opcode, address=0, length=0, data=None):
        """Commit once, then poll/resend that same ID until a bounded outcome.

        A timeout leaves this client unusable for further commands: reconnect
        and inspect the board rather than guessing whether a write completed.
        """
        if opcode in (READ, PROGRAM):
            self._range(address, length, mutate=opcode == PROGRAM)
            if length > self.buffer_size or address // 256 != (address + length - 1) // 256:
                raise FlashError("read/program length exceeds a buffer or crosses a restricted page boundary")
            if opcode == PROGRAM and data is None:
                raise ValueError("program needs data")
        elif opcode in (ERASE_4K, ERASE_64K):
            size = 4096 if opcode == ERASE_4K else 65536
            self._range(address, length, mutate=True)
            if length != size or address % size:
                raise FlashError("erase descriptor has the wrong length or alignment")
        elif opcode in (JEDEC, STATUS):
            if length != (3 if opcode == JEDEC else 1):
                raise ValueError("invalid JEDEC/status descriptor length")
        else:
            raise ValueError("unsupported flash opcode")
        if data is not None and opcode != PROGRAM:
            raise ValueError("page-record command payload is permitted only for program")
        if self.uncertain:
            raise FlashError("previous command outcome is uncertain; reconnect and inspect before continuing")
        initial = self._read_words(8, 6)
        status, previous = initial[0], initial[5]
        if initial[1] != FLASH_SIZE:
            raise FlashError("flash capacity changed before preparing the command")
        if status & 1:
            raise FlashError("flash controller is busy; another client or operation owns it")
        if previous == 0xFFFFFFFF:
            raise FlashError("flash command IDs exhausted; reboot before another operation")
        command_id = previous + 1
        words = []
        if data is not None:
            data = bytes(data)
            if not 1 <= len(data) <= self.buffer_size or len(data) != length:
                raise ValueError("program data must fit the declared negotiated buffer length")
            padded = data + b"\xff" * (-len(data) % 4)
            words = [int.from_bytes(padded[i:i+4], "little") for i in range(0, len(padded), 4)]
        wire_length = length if opcode in (READ, PROGRAM) else 0
        packet = tuple([command_id, address, opcode | (wire_length << 8), *words, command_id])
        submit_address, submit_value = self.base + 0x400, packet
        deadline = self.clock() + self.timeout
        next_send = self.clock()
        sends = 0
        poll_delay = self.poll_interval
        poll_cap = self.max_poll_interval
        operation_cap = {PROGRAM: 0.0001, ERASE_4K: 0.001, ERASE_64K: 0.001}.get(opcode, poll_cap)
        poll_cap = min(poll_cap, max(self.poll_interval, operation_cap))
        combined = getattr(self.bus, "write_read", None)

        def submit():
            if callable(combined):
                # Keep submission and its first status query in the same frame:
                # the validated-frame gate is occupied while writes replay.
                values = list(combined(submit_address, submit_value, self.base + 8, length=9))
                if len(values) != 9:
                    raise FlashError("short combined flash completion response")
                return list(map(int, values))
            self.bus.write(submit_address, submit_value)
            return None

        pending_completion = None
        self.uncertain = True
        while self.clock() < deadline:
            try:
                completion, pending_completion = pending_completion, None
                if sends == 0:
                    # The write can reach hardware even if the transport raises.
                    sends += 1
                    next_send = self.clock() + 0.1
                    completion = submit()
                if completion is None:
                    completion = self._read_words(8, 9)
                status = completion[0]
                accepted, done, error, result = completion[5:9]
                if accepted > command_id or accepted < previous or done > command_id:
                    raise FlashError("flash command ID changed unexpectedly; concurrent client or reboot")
                if accepted == command_id and done == command_id and not status & 1:
                    # Completion can occur partway through the burst; a
                    # previous done ID simply causes another poll. Recheck
                    # ownership so another command cannot replace error/result
                    # after the matching done ID was read.
                    if self._read(0x1C) != command_id:
                        raise FlashError("flash command ownership changed while reading completion")
                    self.uncertain = False
                    # The status word precedes done/error in the completion burst and
                    # may still contain a previous command's error flag.
                    # Once IDs match, the later error word is authoritative.
                    if error:
                        raise FlashError(f"flash command {command_id} failed: {ERRORS.get(error, str(error))}")
                    return result
                if accepted == previous and not status & 1 and sends < 3 and self.clock() >= next_send:
                    sends += 1
                    next_send = self.clock() + 0.1
                    # Retry the identical complete packet, never a suffix
                    # or a separate commit that could publish partial staging.
                    pending_completion = submit()
            except (OSError, TimeoutError):
                pass
            remaining = deadline - self.clock()
            if remaining > 0:
                self.sleep(min(poll_delay, remaining))
            poll_delay = min(poll_cap, poll_delay * 2)
        raise FlashError(f"flash command {command_id} timed out; outcome is uncertain (no new ID was issued)")

    def jedec(self):
        return self.command(JEDEC, length=3)

    def status(self):
        return {"jedec": f"0x{self.jedec():06x}", "flash_status": self.command(STATUS, length=1),
                "image_slot": self.image_slot, "spi_hz": self._read(0x30), "flash_size": FLASH_SIZE,
                "interface": "page-record",
                "buffer_bytes": self.buffer_size,
                "buffer_offset": self.buffer_offset, "transfer_words": self.transfer_words}

    def read(self, address, length, *, progress=None):
        self._range(address, length)
        result = bytearray()
        while len(result) < length:
            count = min(self.buffer_size, length - len(result))
            count = min(count, 256 - ((address + len(result)) % 256))
            self.command(READ, address + len(result), count)
            words = self._read_words(self.buffer_offset, (count + 3)//4)
            result.extend(b"".join(int(w).to_bytes(4, "little") for w in words)[:count])
            if progress:
                progress(len(result), length)
        return bytes(result)

    def erase(self, address, length, *, progress=None):
        self._range(address, length, mutate=True)
        if address % 4096 or length % 4096:
            raise FlashError("erase address and length must be multiples of 4096")
        offset = 0
        while offset < length:
            size = 65536 if (address + offset) % 65536 == 0 and length - offset >= 65536 else 4096
            self.command(ERASE_64K if size == 65536 else ERASE_4K, address + offset, size)
            offset += size
            if progress:
                progress(offset, length)

    def program(self, address, data, *, check_erased=True, verify=True, progress=None):
        data = bytes(data)
        self._range(address, len(data), mutate=True)
        if check_erased:
            existing = self.read(address, len(data))
            if any((old & new) != new for old, new in zip(existing, data)):
                raise FlashError("program would change a zero bit to one; erase the affected sectors first")
        offset = 0
        while offset < len(data):
            capacity = 256 - ((address + offset) % 256)
            count = min(capacity, len(data) - offset)
            self.command(PROGRAM, address + offset, count, data[offset:offset+count])
            offset += count
            if progress:
                progress(offset, len(data))
        if verify:
            self.verify(address, data)

    def verify(self, address, expected, *, progress=None):
        expected = bytes(expected)
        observed = self.read(address, len(expected), progress=progress)
        if observed != expected:
            offset = next(i for i, pair in enumerate(zip(observed, expected)) if pair[0] != pair[1])
            raise FlashError(f"flash verification mismatch at 0x{address + offset:06x}")


class BootClient:
    """Read image identity and request a vendor remote-update selector.

    image_slot is a build-time identity tag, not a physical boot-address latch.
    The updater also checks the independent build ID and flash contents.
    """

    def __init__(self, bus, *, base=BOOT_BASE):
        self.bus, self.base = bus, base
        if self._read(0) != BOOT_MAGIC:
            raise FlashError("unsupported remote-update block magic")

    def _read(self, offset):
        return int(self.bus.read(self.base + offset))

    def identity(self):
        slot = self._read(8)
        if slot not in range(4):
            raise FlashError("invalid image slot identity")
        build_id = "".join(f"{self._read(offset):08x}" for offset in range(0x10, 0x20, 4))
        return {"image_slot": slot, "build_id": build_id, "status": self._read(0x0C)}

    def request(self, slot, *, acknowledge=True):
        if slot not in range(4):
            raise ValueError("remote-update selector must be 0 through 3")
        if self._read(0x0C) & 1:
            raise FlashError("a reboot is already pending")
        previous = self._read(0x24)
        if previous >= 0xFFFFFFFC:
            raise FlashError("boot command IDs exhausted")
        command_id = (((previous >> 2) + 1) << 2) | slot
        verified_write(self.bus, self.base + 0x20, slot)
        if self._read(0x24) != previous:
            raise FlashError("reboot command ownership changed")
        # Exactly one trigger send. A timeout may mean the requested reboot
        # already happened; never replay this write into a different boot.
        try:
            # The low two bits atomically select the target. A delayed old
            # target_slot staging write cannot redirect this request.
            trigger_write = getattr(self.bus, "write_reboot_command", None) if not acknowledge else None
            (trigger_write or self.bus.write)(self.base + 0x24, command_id)
            if not acknowledge:
                # Prepared USB reboots must keep the descriptor open without
                # I/O until removal; even a failed acknowledgement read would
                # close it early. Subsequent generation/identity checks decide
                # whether this single, deliberately unacknowledged send worked.
                return {"request_id": command_id, "acknowledged": False}
            accepted = self._read(0x24)
            error = self._read(0x28)
        except (OSError, TimeoutError):
            return {"request_id": command_id, "acknowledged": False}
        if error:
            raise FlashError(f"remote-update request rejected with error {error}")
        if accepted != command_id:
            if accepted > command_id:
                raise FlashError("another client committed a later reboot request")
            # A retrying read may already have reached the newly configured
            # block, whose accepted ID is reset. Subsequent fresh identity
            # checks distinguish that from a dropped trigger. Never resend.
            return {"request_id": command_id, "acknowledged": False}
        return {"request_id": accepted, "acknowledged": True}


def add_connection_arguments(parser):
    parser.add_argument("--transport", choices=("etherbone", "usb"), default="etherbone")
    parser.add_argument("--target", default="192.168.1.50", type=ipaddress.IPv4Address)
    parser.add_argument("--port", type=int, default=1234)
    parser.add_argument("--device", help="explicit LUNA CDC device for USB; prefer /dev/serial/by-id/...")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.2, help="seconds per bounded transport read attempt")


def create_transport(args):
    if args.transport == "usb":
        if not args.device:
            raise ValueError("--transport usb requires --device; serial devices are never guessed")
        return UARTBoneUSB(args.device, timeout=args.timeout, baudrate=args.baudrate)
    return EtherboneUDP(str(args.target), args.port, timeout=args.timeout)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_arguments(parser)
    sub = parser.add_subparsers(dest="operation", required=True)
    sub.add_parser("status")
    read = sub.add_parser("read")
    read.add_argument("--address", required=True, type=lambda x: int(x, 0))
    read.add_argument("--length", required=True, type=lambda x: int(x, 0))
    read.add_argument("--output", required=True, type=Path)
    erase = sub.add_parser("erase")
    erase.add_argument("--address", required=True, type=lambda x: int(x, 0))
    erase.add_argument("--length", required=True, type=lambda x: int(x, 0))
    write = sub.add_parser("write")
    write.add_argument("--address", required=True, type=lambda x: int(x, 0))
    write.add_argument("--input", required=True, type=Path, help="raw SPI bytes; no implicit erase or .bit conversion")
    boot = sub.add_parser("boot")
    boot.add_argument("--slot", required=True, type=int, choices=range(4))
    boot.add_argument("--reboot-timeout", type=float, default=90.0,
                      help="maximum seconds to wait for a USB device generation change")
    args = parser.parse_args(argv)
    try:
        payload = args.input.read_bytes() if args.operation == "write" else None
        with create_transport(args) as bus:
            if args.operation == "boot":
                if args.reboot_timeout <= 0:
                    raise ValueError("reboot timeout must be positive")
                deadline = time.monotonic() + args.reboot_timeout
                prepare = getattr(bus, "prepare_reboot", None)
                transition = prepare() if prepare is not None else None
                if time.monotonic() >= deadline:
                    raise FlashError("reboot deadline expired before the request")
                result = BootClient(bus).request(args.slot, acknowledge=transition is None)
                if transition is not None:
                    result["usb_transition"] = transition(deadline)
                result["note"] = "reconfiguration requested; this command does not verify the subsequent boot"
            else:
                flash = FlashClient(bus)
                if flash.jedec() != JEDEC_ID:
                    raise FlashError("unexpected JEDEC ID; expected GD25LQ128 (0xc86018)")
                if args.operation == "status":
                    result = {"flash": flash.status(), "boot": BootClient(bus).identity()}
                elif args.operation == "read":
                    args.output.write_bytes(flash.read(args.address, args.length))
                    result = {"read_bytes": args.length, "output": str(args.output)}
                elif args.operation == "erase":
                    flash.erase(args.address, args.length)
                    result = {"erased_bytes": args.length, "address": args.address}
                else:
                    flash.program(args.address, payload)
                    result = {"written_bytes": len(payload), "verified": True, "address": args.address}
        print(json.dumps(result, indent=2))
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(json.dumps({"ok": False, "error": f"{type(error).__name__}: {error}"}, indent=2))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
