#!/usr/bin/env python3
"""Fetch a seeded, size-eligible DIV2K validation sample using ZIP HTTP ranges."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import random
import struct
import urllib.request
import zipfile
import zlib

from PIL import Image

URL = "https://data.vision.ee.ethz.ch/cvl/DIV2K/DIV2K_valid_HR.zip"


def get_range(start, end):
    req = urllib.request.Request(URL, headers={"Range": f"bytes={start}-{end}"})
    with urllib.request.urlopen(req, timeout=120) as response:
        if response.status != 206:
            raise RuntimeError(f"Server did not honor byte range: {response.status}")
        data = response.read()
        total = int(response.headers["Content-Range"].split("/")[-1])
    if len(data) != end - start + 1:
        raise RuntimeError("Truncated range response")
    return data, total


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--count", type=int, default=12)
    parser.add_argument("--seed", type=int, default=20261107)
    args = parser.parse_args()
    original = args.output / "original"
    original.mkdir(parents=True, exist_ok=True)
    _, total = get_range(0, 0)
    tail, _ = get_range(total - 65536, total - 1)
    offset = tail.rfind(b"PK\x05\x06")
    if offset < 0:
        raise RuntimeError("ZIP directory not found")
    eocd = struct.unpack_from("<4s4H2IH", tail, offset)
    directory_size, directory_offset = eocd[5:7]
    directory, _ = get_range(directory_offset, directory_offset + directory_size - 1)
    # A tiny synthetic ZIP lets the standard parser validate the directory.
    end = bytearray(tail[offset:])
    struct.pack_into("<I", end, 16, 0)
    with zipfile.ZipFile(io.BytesIO(directory + end)) as archive:
        entries = sorted((i for i in archive.infolist() if i.filename.endswith(".png")), key=lambda i: i.filename)
    random.Random(args.seed).shuffle(entries)
    accepted, skipped = [], []
    for entry in entries:
        header, _ = get_range(entry.header_offset, entry.header_offset + 29)
        values = struct.unpack("<4s5H3I2H", header)
        if values[0] != b"PK\x03\x04":
            raise RuntimeError("Bad local ZIP header")
        data_start = entry.header_offset + 30 + values[-2] + values[-1]
        # The PNG dimensions are at the start, including for deflated entries.
        prefix, _ = get_range(data_start, data_start + min(entry.compress_size, 4096) - 1)
        raw_prefix = zlib.decompressobj(-15).decompress(prefix, 64) if entry.compress_type == 8 else prefix
        if raw_prefix[:8] != b"\x89PNG\r\n\x1a\n":
            raise RuntimeError("Expected PNG")
        width, height = struct.unpack_from(">II", raw_prefix, 16)
        if width < 1280 or height < 800:
            skipped.append({"file": entry.filename, "width": width, "height": height})
            continue
        filename = Path(entry.filename).name
        path = original / filename
        if path.exists():
            raw = path.read_bytes()
        else:
            packed, _ = get_range(data_start, data_start + entry.compress_size - 1)
            raw = zlib.decompress(packed, -15) if entry.compress_type == 8 else packed
            path.write_bytes(raw)
        assert len(raw) == entry.file_size and zlib.crc32(raw) == entry.CRC
        with Image.open(io.BytesIO(raw)) as image:
            image.verify()
        accepted.append({"id": path.stem, "archive_member": entry.filename,
                         "width": width, "height": height, "sha256": hashlib.sha256(raw).hexdigest()})
        print(f"{len(accepted):2}/{args.count}: {filename} {width}x{height}", flush=True)
        if len(accepted) == args.count:
            break
    if len(accepted) != args.count:
        raise RuntimeError("Insufficient eligible images")
    metadata = {"dataset": "DIV2K", "split": "validation HR", "url": URL,
                "seed": args.seed, "selection": f"Shuffle sorted PNG archive members; accept first {args.count} with width >= 1280 and height >= 800, before encoding.",
                "skipped_for_size": skipped, "images": sorted(accepted, key=lambda x: x["id"])}
    (args.output / "selection.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    main()
