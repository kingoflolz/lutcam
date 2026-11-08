"""Validate LUTCam UDP datagrams and deliver image fragments or records.

Example: ``python -m tools.lutcam.receive --port 6000 --output-dir capture``.
Image fragments go directly to the caller's decoder, which owns frame
completion. This transport does not inspect the codec bitstream. The CLI saves
raw fragments as .lwcpart files with camera, frame, offset, and sequence metadata.
Each child group and accepted pose prints as a separate JSON record.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import socket
import struct
import sys

MAGIC = 0x4C555443
HEADER_SIZE = 32
MAX_DATAGRAM_SIZE = 1440
# Board image-fragment payload size; the parser accepts word-aligned ranges
# within the image budget and UDP payload ceiling.
FRAGMENT_SIZE = 1024
IMAGE_SIZE = 128000
IMAGE, CHILD, POSE = 1, 2, 3
_HEADER = struct.Struct(">8I")


class PacketError(ValueError):
    """An invalid or unsupported LUTCam datagram."""


@dataclass(frozen=True)
class Packet:
    camera: int
    stream: int
    frame_id: int
    sequence: int
    offset: int
    flags: int
    payload: bytes
    # Reserved header byte, always zero.
    version: int = 0

    def child_groups(self) -> tuple[tuple[int, ...], ...]:
        """Return all retained child groups, preserving their boundaries."""
        if self.stream != CHILD:
            raise ValueError("not a child-group packet")
        return _child_groups(self.payload)

    def child_tuples(self) -> tuple[int, ...]:
        """Return one group's tuples; use child_groups() for batched packets."""
        groups = self.child_groups()
        if len(groups) != 1:
            raise ValueError("multiple child groups; use child_groups()")
        return groups[0]

    def poses(self) -> tuple[tuple[int, ...], ...]:
        """Return every accepted 22-word pose record in this packet."""
        if self.stream != POSE:
            raise ValueError("not a pose packet")
        return _poses(self.payload)

    def pose_words(self) -> tuple[int, ...]:
        """Return one pose's words; use poses() for batched packets."""
        poses = self.poses()
        if len(poses) != 1:
            raise ValueError("multiple poses; use poses()")
        return poses[0]


def _child_groups(payload: bytes) -> tuple[tuple[int, ...], ...]:
    if not 72 <= len(payload) <= 4 * 72 or len(payload) % 72:
        raise PacketError("child batch must contain one through four complete nine-tuple records")
    groups = tuple(struct.unpack_from(">9Q", payload, offset)
                   for offset in range(0, len(payload), 72))
    if any(value >> 33 for group in groups for value in group):
        raise PacketError("child tuple is not a zero-extended 33-bit value")
    return groups


def _poses(payload: bytes) -> tuple[tuple[int, ...], ...]:
    if not payload or len(payload) % 88:
        raise PacketError("pose packet must contain complete 22-word records; rejections are forbidden")
    if len(payload) > 2 * 88:
        raise PacketError("pose batch must contain at most two records")
    poses = tuple(struct.unpack_from(">22I", payload, offset)
                  for offset in range(0, len(payload), 88))
    for words in poses:
        status = words[8]
        if not status & 1 or status & 0xFFFFFFF0:
            raise PacketError("invalid accepted pose status")
    return poses


def parse_datagram(data: bytes) -> Packet:
    """Validate header, exact length, framing, and native record encoding."""
    if not HEADER_SIZE <= len(data) <= MAX_DATAGRAM_SIZE:
        raise PacketError("truncated or oversized UDP datagram")
    magic, info, frame, sequence, offset, length, flags, reserved = _HEADER.unpack_from(data)
    version, camera, stream, header_size = info.to_bytes(4, "big")
    if (magic != MAGIC or version != 0
            or header_size != HEADER_SIZE):
        raise PacketError("unsupported LUTCam header")
    if camera not in (0, 1) or stream not in (IMAGE, CHILD, POSE):
        raise PacketError("unknown camera or stream")
    if reserved or flags:
        raise PacketError("nonzero reserved header bits")
    if length != len(data) - HEADER_SIZE:
        raise PacketError("payload length does not match datagram")
    payload = bytes(data[HEADER_SIZE:])
    if stream == IMAGE:
        if length % 2 or offset % 2 or not length:
            raise PacketError("unaligned or empty image fragment")
        end = offset + length
        if end > IMAGE_SIZE:
            raise PacketError("invalid image end boundary")
    elif stream == CHILD:
        if offset:
            raise PacketError("child packet offset must be zero")
        _child_groups(payload)
    else:
        if offset:
            raise PacketError("pose packet offset must be zero")
        _poses(payload)
    return Packet(camera, stream, frame, sequence, offset, flags, payload, version)


class SequenceTracker:
    """Account for gaps, reordering, wraparound and duplicate datagrams.

    Missing counts are provisional until packets age out of the reorder
    window. Reordered arrivals inside that window repair earlier gaps. The
    first observed sequence establishes the baseline; device resets require
    a new receiver session because the wire header has no boot identifier.
    """

    def __init__(self, window: int = 4096):
        if window < 1:
            raise ValueError("sequence window must be positive")
        self.window = window
        self.highest = None
        self.seen = set()
        self.holes = set()
        self.missing = 0
        self.duplicates = 0
        self.reordered = 0
        self.too_old = 0

    def add(self, sequence: int) -> bool:
        if self.highest is None:
            self.highest = sequence
            self.seen.add(sequence)
            return True
        forward = (sequence - self.highest) & 0xFFFFFFFF
        if forward == 0 or sequence in self.seen:
            self.duplicates += 1
            return False
        if forward < 0x80000000:
            self.missing += forward - 1
            # Advance the sliding window in amortized constant time for the
            # normal contiguous case (30,000 image datagrams/second).
            # Rebuilding the complete seen set per packet is too expensive.
            if forward >= self.window:
                self.seen.clear()
                self.holes.clear()
            else:
                for distance in range(forward):
                    expired = (self.highest - self.window + 1 + distance) & 0xFFFFFFFF
                    self.seen.discard(expired)
                    self.holes.discard(expired)
            for distance in range(1, min(forward, self.window)):
                self.holes.add((sequence - distance) & 0xFFFFFFFF)
            self.highest = sequence
        else:
            if (self.highest - sequence) & 0xFFFFFFFF >= self.window:
                self.too_old += 1
                return False
            self.reordered += 1
            if sequence in self.holes:
                self.holes.remove(sequence)
                self.missing -= 1
        self.seen.add(sequence)
        return True


class StreamReceiver:
    """Validate transport and return fresh packets for the caller to consume.

    Image offsets and frame completion are interpreted by the image decoder.
    Duplicate and stale packet sequences are discarded independently per camera
    and product stream.
    """

    def __init__(self):
        self.sequences: dict[tuple[int, int], SequenceTracker] = {}
        self.invalid_packets = 0

    def feed(self, data: bytes) -> Packet | None:
        try:
            packet = parse_datagram(data)
            sequence = self.sequences.setdefault((packet.camera, packet.stream), SequenceTracker())
            return packet if sequence.add(packet.sequence) else None
        except PacketError:
            self.invalid_packets += 1
            raise

    def statistics(self):
        return {
            "invalid_packets": self.invalid_packets,
            "streams": {
                f"{camera}:{stream}": {
                    "missing_packets": state.missing,
                    "duplicate_packets": state.duplicates,
                    "reordered_packets": state.reordered,
                    "too_old_packets": state.too_old,
                }
                for (camera, stream), state in self.sequences.items()
            },
        }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--source", help="accept only datagrams from this IPv4 address")
    parser.add_argument("--output-dir", type=Path,
                        help="save .lwcpart image fragments; otherwise print metadata")
    args = parser.parse_args(argv)
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
    receiver = StreamReceiver()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        sock.bind((args.bind, args.port))
        try:
            while True:
                data, source = sock.recvfrom(65535)
                if args.source and source[0] != args.source:
                    continue
                try:
                    result = receiver.feed(data)
                except PacketError as error:
                    print(f"invalid packet from {source[0]}: {error}", file=sys.stderr)
                    continue
                if result is None:
                    continue
                record = {"camera": result.camera, "frame_id": result.frame_id}
                if result.stream == IMAGE:
                    record.update(stream="image_fragment", offset=result.offset,
                                  sequence=result.sequence, bytes=len(result.payload))
                    if args.output_dir:
                        path = args.output_dir / (
                            f"camera{result.camera}-frame{result.frame_id:08x}"
                            f"-offset{result.offset:08x}-sequence{result.sequence:08x}.lwcpart")
                        with path.open("xb") as output:
                            output.write(result.payload)
                        record["file"] = str(path)
                    records = [record]
                elif result.stream == CHILD:
                    records = [dict(record, stream="child", tuples=group)
                               for group in result.child_groups()]
                else:
                    records = [dict(record, stream="pose", words=words)
                               for words in result.poses()]
                for record in records:
                    print(json.dumps(record), flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            print(json.dumps(receiver.statistics()), file=sys.stderr)


if __name__ == "__main__":
    main()
