"""Validate and program the production LUTCam MonoIsp defect table.

The production raster is fixed at 1280 x 800 pixels, or 512,000 pixel pairs.
Semantic entries are ``(pair_address, mask)`` with masks 1 (lane 0), 2 (lane 1),
or 3 (both). The canonical encoding uses a maximal zero-mask continuation only
when a gap exceeds the 18-bit delta. At most one such continuation fits in this
raster; redundant continuations can stall an unbackpressured camera stream.

Use ``program_defect_table(bus, base, pack_defect_table(entries))`` with the
scalar ``read(address)`` / ``write(address, value)`` API of LiteX RemoteClient.
Construct RemoteClient with ``raise_on_timeout=True``: transport failures must
raise instead of returning zero, which could falsely verify an empty table.
Disarm camera admission, wait for the entire camera to drain, and stop MonoIsp
before calling. Keep exclusive configuration ownership until programming is
complete: a status read cannot prevent another USB/Etherbone host restarting
the camera. The helper does not stop a running camera or modify its controls.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable


FRAME_PAIRS = 1280 * 800 // 2
DEFECT_DELTA_MAX = (1 << 18) - 1
DEFECT_TABLE_CAPACITY = 512
DEFECT_WORD_MAX = (1 << 20) - 1
DEFECT_CONTINUATION = DEFECT_DELTA_MAX << 2
STATUS = 0x0008
DEFECT_COUNT = 0x000C
DEFECT_TABLE = 0x2000


class IspConfigError(RuntimeError):
    """The stopped-ISP prerequisite or an ordered readback check failed."""


def _integer(value, description: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"{description} must be an integer")
    return value


def pack_defect_table(entries: Iterable[tuple[int, int]]) -> list[int]:
    """Encode sorted semantic defects without redundant continuation words.

    Addresses must be strictly increasing and inside the production frame.
    Capacity counts encoded words, including the possible continuation. Every
    semantic list representable by the existing 512-word format has a fitting
    canonical encoding; this function never adds padding or a terminator.
    """
    words = []
    previous = -1
    for index, entry in enumerate(entries):
        try:
            address, mask = entry
        except (TypeError, ValueError) as error:
            raise ValueError(f"defect {index} must be an (address, mask) pair") from error
        address = _integer(address, f"defect {index} address")
        mask = _integer(mask, f"defect {index} mask")
        if not 0 <= address < FRAME_PAIRS:
            raise ValueError(f"defect {index} address must be in 0..{FRAME_PAIRS - 1}")
        if address <= previous:
            raise ValueError("defect addresses must be strictly increasing")
        if mask not in (1, 2, 3):
            raise ValueError(f"defect {index} mask must be 1, 2, or 3")
        delta = address - max(previous, 0)
        while delta > DEFECT_DELTA_MAX:
            words.append(DEFECT_CONTINUATION)
            delta -= DEFECT_DELTA_MAX
        words.append((delta << 2) | mask)
        if len(words) > DEFECT_TABLE_CAPACITY:
            raise ValueError("defect table exceeds 512 encoded words")
        previous = address
    return words


def validate_defect_table(words: Iterable[int]) -> list[tuple[int, int]]:
    """Decode a canonical table or raise ``ValueError`` before bus access.

    Decoding followed by exact repacking rejects redundant, short, split, and
    trailing zero-mask words as well as invalid addresses, ordering, and width.
    The empty table is valid and disables defect correction.
    """
    words = list(words)
    if len(words) > DEFECT_TABLE_CAPACITY:
        raise ValueError("defect table exceeds 512 encoded words")
    entries = []
    address = 0
    for index, word in enumerate(words):
        word = _integer(word, f"defect word {index}")
        if not 0 <= word <= DEFECT_WORD_MAX:
            raise ValueError(f"defect word {index} must fit in 20 bits")
        address += word >> 2
        if address >= FRAME_PAIRS:
            raise ValueError(f"defect word {index} advances beyond the frame")
        mask = word & 3
        if mask:
            entries.append((address, mask))
    if pack_defect_table(entries) != words:
        raise ValueError("defect table is not canonical; repack semantic entries")
    return entries


def _require_stopped(bus, base: int):
    if int(bus.read(base + STATUS)) & 3:
        raise IspConfigError("MonoIsp must be stopped and drained (STATUS bits 0 and 1 clear)")


def _write_verified(bus, address: int, value: int, description: str):
    bus.write(address, value)
    # RemoteClient writes are queued. A scalar read on the same ordered
    # connection both completes the write and verifies its actual value.
    actual = int(bus.read(address))
    if actual != value:
        raise IspConfigError(
            f"{description} readback mismatch: expected 0x{value:x}, got 0x{actual:x}"
        )


def program_defect_table(bus, base: int, words: Iterable[int]) -> int:
    """Verify and install canonical encoded words; return the committed count.

    All input validation precedes any bus access. After checking stopped/drained
    status, clear and verify DEFECT_COUNT, write and verify each table word, then
    commit and verify the count last. A table-write/readback failure leaves the
    count cleared. A transport failure or mismatch during the final count
    commit leaves the commit state uncertain: read it back before restarting.
    Unused table storage is ignored by the count and need not be overwritten.

    The caller must first disarm and drain the camera, stop the ISP, and prevent
    any competing configuration or capture activity throughout this operation.
    The bus must raise on transport failure; RemoteClient requires explicit
    ``raise_on_timeout=True``. A client exposing a disabled timeout flag is
    rejected before any bus access.
    """
    base = _integer(base, "ISP base")
    if base < 0 or base % 4 or base + DEFECT_TABLE + 4 * (DEFECT_TABLE_CAPACITY - 1) > 0xFFFFFFFF:
        raise ValueError("ISP base must be a word-aligned address in the 32-bit address space")
    words = list(words)
    validate_defect_table(words)
    if not getattr(bus, "raise_on_timeout", True):
        raise ValueError("bus must raise on transport failure; set raise_on_timeout=True")
    _require_stopped(bus, base)
    _write_verified(bus, base + DEFECT_COUNT, 0, "cleared defect count")
    for index, word in enumerate(words):
        _write_verified(bus, base + DEFECT_TABLE + 4 * index, word, f"defect word {index}")
    _require_stopped(bus, base)
    _write_verified(bus, base + DEFECT_COUNT, len(words), "committed defect count")
    return len(words)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("entries", type=Path,
                        help="JSON array of [pair_address, mask] semantic defect entries")
    parser.add_argument("--isp-base", type=lambda value: int(value, 0), required=True,
                        help="camera ISP base: 0x30000000 or 0x30010000")
    parser.add_argument("--host", default="localhost", help="LiteX server host")
    parser.add_argument("--port", type=int, default=1234, help="LiteX server TCP port")
    parser.add_argument("--csr-csv", help="optional generated LiteX CSR CSV")
    args = parser.parse_args(argv)
    try:
        entries = json.loads(args.entries.read_text())
        if not isinstance(entries, list):
            raise ValueError("defect JSON must contain an array of semantic entries")
        words = pack_defect_table(entries)
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))
    from litex import RemoteClient
    bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv,
                       raise_on_timeout=True)
    bus.open()
    try:
        count = program_defect_table(bus, args.isp_base, words)
        print(f"Verified {len(entries)} defective pairs in {count} encoded words.")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
