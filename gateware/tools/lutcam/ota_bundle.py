"""Build and verify Ti60 remote-update images using Efinity's image merger.

The input is Efinity's SPI .hex format (one hexadecimal byte per line), not
Intel HEX, a JTAG .bit, or a raw binary. The installed vendor merger performs
all header rewriting. This module only reads the documented device database
fields to check its result; it never patches configuration bits.

The fixed layout leaves the golden image at zero and maps application A/B to
selectors 1/2. Selector 3 is reserved, and is deliberately not populated.
Power-on still starts the golden image; its application-selection policy is
gateware functionality, not something the image merger supplies.

References:
  https://www.efinixinc.com/docs/an010-using-internal-reconfiguration-feature-v2.5.pdf
  Efinity 2025.2 pgm/bin/device_db.xml, die Ti60_MP, option_register
  Efinity 2025.2 pgm/bin/efx_pgm/image/image_builder.py, generate_flash_image
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import struct
import tempfile
import xml.etree.ElementTree as ET
import zlib


SCHEMA = "lutcam-ota-v1"
DEVICE = "Ti60F225"
FLASH_SIZE = 16 * 1024 * 1024
SLOT_SIZE = 2 * 1024 * 1024
METADATA_SIZE = 4096
APPLICATION_CAPACITY = SLOT_SIZE - METADATA_SIZE
RECORD_SIZE = 64
RECORD_MAGIC = b"LUTCOTA1"
COMMIT_WORD = 0x434f4d54
COMMIT_OFFSET = 44
SLOT_LAYOUT = {"golden": (0, 0), "A": (1, SLOT_SIZE), "B": (2, 2 * SLOT_SIZE)}
RESERVED_ADDRESS = 3 * SLOT_SIZE
FIRMWARE_OFFSET = 0x180000


def validate_firmware(data):
    if len(data) < 16:
        raise ValueError("truncated camera firmware")
    magic, length, crc, ram = struct.unpack_from("<4I", data)
    if (magic != 0x3256524c or ram != 4096 or not 0 < length <= ram-128
            or length % 4 or len(data) != length+16 or zlib.crc32(data[16:]) != crc):
        raise ValueError("invalid camera firmware header, length or CRC")
    return {"offset": FIRMWARE_OFFSET, "length": len(data), "sha256": sha256(data)}

SYNC = bytes.fromhex("168a2236")
OPTION_BYTES = 144
# Factual field locations from the installed Ti60_MP device database. The
# option register is serialized most-significant byte first; addresses count
# from its least-significant bit. These are checks, not a bitstream encoder.
OPTION_FIELDS = {"EN_CB": (290, 1), "EN_RUPD": (939, 1),
                 "CB_ADDR_1": (291, 24), "CB_ADDR_2": (315, 24),
                 "CB_ADDR_3": (339, 24), "EN_CASCADE": (393, 1)}
REMOTE_VALUES = {"EN_CB": 0, "EN_RUPD": 1, "CB_ADDR_1": SLOT_SIZE,
                 "CB_ADDR_2": 2 * SLOT_SIZE, "CB_ADDR_3": RESERVED_ADDRESS,
                 "EN_CASCADE": 0}
SOURCE_REFERENCES = [
    "https://www.efinixinc.com/docs/an010-using-internal-reconfiguration-feature-v2.5.pdf",
    "https://www.efinixinc.com/docs/an033-configuring-titanium-fpgas-v3.1.pdf",
    "Efinity 2025.2 pgm/bin/device_db.xml: Ti60_MP option_register",
    "Efinity 2025.2 pgm/bin/efx_pgm/image/image_builder.py: generate_flash_image",
]


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def make_record(image, slot, build_id):
    """Create application metadata; the host programs the commit word last."""
    if type(slot) is not int or slot not in (1, 2):
        raise ValueError("application metadata requires slot 1 or 2")
    if not isinstance(image, bytes) or not 0 < len(image) <= APPLICATION_CAPACITY:
        raise ValueError("application image does not fit before its metadata sector")
    prefix = (RECORD_MAGIC + struct.pack("<IIII", 0, slot, len(image), zlib.crc32(image))
              + bytes.fromhex(_build_id(build_id)))
    return prefix + struct.pack("<II", zlib.crc32(prefix), COMMIT_WORD) + b"\xff" * 16


def parse_record(record, expected_slot, image=None):
    """Check committed metadata and, when supplied, the exact image payload.

    CRC32 is the reflected IEEE polynomial used by zlib.crc32, initial and
    final XOR 0xffffffff. Fields are little-endian except the build ID bytes,
    which are in canonical hexadecimal string order. The reserved word at
    offset 8 must be zero and remains protected by the header CRC.
    """
    if type(expected_slot) is not int or expected_slot not in (1, 2):
        raise ValueError("application metadata requires expected slot 1 or 2")
    if not isinstance(record, bytes) or len(record) != RECORD_SIZE:
        raise ValueError("application record must be exactly 64 bytes")
    if record[:8] != RECORD_MAGIC or record[48:] != b"\xff" * 16:
        raise ValueError("invalid application record magic or reserved bytes")
    reserved, slot, length, crc32 = struct.unpack_from("<IIII", record, 8)
    if reserved != 0:
        raise ValueError("nonzero reserved application record word")
    header_crc, commit = struct.unpack_from("<II", record, 40)
    if slot != expected_slot or not 0 < length <= APPLICATION_CAPACITY:
        raise ValueError("invalid application record slot or length")
    if header_crc != zlib.crc32(record[:40]) or commit != COMMIT_WORD:
        raise ValueError("uncommitted or damaged application record")
    if image is not None and (not isinstance(image, bytes) or len(image) != length or zlib.crc32(image) != crc32):
        raise ValueError("application record does not match image length and CRC32")
    return {"slot": slot, "length": length, "crc32": crc32,
            "build_id": record[24:40].hex()}


def decode_efinity_hex(text):
    """Decode strict vendor byte-per-line text, rejecting other hex formats."""
    if not isinstance(text, str):
        raise ValueError("Efinity .hex input must be ASCII text")
    # Avoid allocating millions of tiny strings for a 6 MiB installer.
    canonical = text.replace("\r\n", "\n").removesuffix("\n")
    pairs = canonical[0::3] + canonical[1::3]
    if (len(canonical) % 3 != 2 or not pairs
            or canonical[2::3] != "\n" * (len(canonical) // 3)
            or re.fullmatch(r"[0-9a-fA-F]+", pairs) is None):
        raise ValueError("expected Efinity SPI .hex: exactly one hexadecimal byte per line")
    return bytes.fromhex(canonical)


def _read_hex(path, maximum=RESERVED_ADDRESS):
    path = Path(path)
    if path.stat().st_size > 4 * maximum:
        raise ValueError(f"hex file exceeds the permitted size: {path}")
    try:
        data = decode_efinity_hex(path.read_text(encoding="ascii"))
    except UnicodeError as error:
        raise ValueError("expected ASCII Efinity SPI .hex") from error
    if len(data) > maximum:
        raise ValueError("decoded image exceeds the permitted size")
    return data


def parse_image(data, *, require_remote=False):
    """Check the Ti60 SPI header and, optionally, the fixed remote slot map.

    This verifies packaging metadata and boot selection fields, not the
    configuration payload's CRC or the behavior of the application logic.
    """
    if not isinstance(data, bytes) or not 0 < len(data) <= SLOT_SIZE:
        raise ValueError("image must be nonempty bytes fitting one 2 MiB slot")
    sync_offset = data.find(SYNC, 0, 1024)
    if sync_offset < 0 or len(data) <= sync_offset + len(SYNC) + OPTION_BYTES:
        raise ValueError("missing or truncated Ti60 option register")
    try:
        text = data[:sync_offset].decode("ascii")
    except UnicodeError as error:
        raise ValueError("invalid ASCII image preamble") from error
    fields = {}
    for line in text.splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            if key in fields:
                raise ValueError(f"duplicate image header field: {key}")
            fields[key] = value.strip()
    expected = {"Family": "Titanium", "Device": DEVICE, "Die Name": "Ti60_MP",
                "Width": "4", "Mode": "active", "4Byte": "0"}
    for key, value in expected.items():
        if fields.get(key) != value:
            raise ValueError(f"image {key} must be {value!r}, got {fields.get(key)!r}")
    start = sync_offset + len(SYNC)
    options = int.from_bytes(data[start:start + OPTION_BYTES], "big")
    values = {name: (options >> bit) & ((1 << width) - 1)
              for name, (bit, width) in OPTION_FIELDS.items()}
    if values["EN_CASCADE"]:
        raise ValueError("daisy-chain bitstreams are not supported")
    if require_remote and values != REMOTE_VALUES:
        raise ValueError(f"remote-update header does not match fixed slot layout: {values}")
    return {"fields": fields, "option_offset": start, "option_values": values}


def validate_combined(combined, source_images):
    """Check all vendor modifications, exact slot segments and erased gaps."""
    if set(source_images) != set(SLOT_LAYOUT):
        raise ValueError("exactly golden, A and B source images are required")
    if len(combined) != RESERVED_ADDRESS:
        raise ValueError("combined installer must end at reserved selector 3, address 0x600000")
    result = {}
    allowed_mask = 0
    for name, (bit, width) in OPTION_FIELDS.items():
        if name != "EN_CASCADE":
            allowed_mask |= ((1 << width) - 1) << bit
    for name, (_, address) in SLOT_LAYOUT.items():
        source = source_images[name]
        parsed = parse_image(source)
        if name != "golden" and len(source) > APPLICATION_CAPACITY:
            raise ValueError(f"{name} image overlaps its metadata sector")
        segment = combined[address:address + len(source)]
        parsed_result = parse_image(segment, require_remote=True)
        offset = parsed["option_offset"]
        if parsed_result["option_offset"] != offset:
            raise ValueError(f"vendor merger changed {name} header position")
        end = offset + OPTION_BYTES
        if source[:offset] != segment[:offset] or source[end:] != segment[end:]:
            raise ValueError(f"vendor merger changed {name} data outside option register")
        changed = int.from_bytes(source[offset:end], "big") ^ int.from_bytes(segment[offset:end], "big")
        if changed & ~allowed_mask:
            raise ValueError(f"vendor merger changed unapproved {name} option fields")
        gap = combined[address + len(source):address + SLOT_SIZE]
        if any(byte != 0xff for byte in gap):
            raise ValueError(f"non-erased padding or image overlap after {name}")
        result[name] = segment
    return result


def _build_id(value):
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{32}", value) is None:
        raise ValueError("build ID must be exactly 32 lowercase hexadecimal characters")
    return value


def _check_build_ids(build_ids):
    if set(build_ids) != set(SLOT_LAYOUT):
        raise ValueError("one build ID is required for each source image")
    for value in build_ids.values():
        _build_id(value)
    if build_ids["A"] != build_ids["B"]:
        raise ValueError("A and B must be slot variants of the same application build ID")


def write_bundle(output_dir, combined, sources, build_ids, provenance, *, firmware=None):
    """Write validated artifacts to a new directory, publishing manifest last.

    Explicit build IDs attest which --build-id was used for each source build;
    this packager cannot infer fabric CSR constants from a compiled bitstream.
    The updater must check both the running slot tag and build ID after boot.
    """
    images = validate_combined(combined, sources)
    firmware_info = validate_firmware(firmware) if firmware is not None else None
    if firmware is not None:
        for name in ("A", "B"):
            if len(images[name]) > FIRMWARE_OFFSET:
                raise ValueError("FPGA image overlaps the camera firmware region")
            images[name] += b"\xff" * (FIRMWARE_OFFSET-len(images[name])) + firmware
    _check_build_ids(build_ids)
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=False)
    slots = []
    installer_data = bytearray(combined)
    for name, (selector, address) in SLOT_LAYOUT.items():
        data = images[name]
        filename = f"slot-{name}.bin"
        (directory / filename).write_bytes(data)
        entry = {"name": name, "selector": selector, "image_slot": selector,
                 "address": address, "capacity": SLOT_SIZE if selector == 0 else APPLICATION_CAPACITY,
                 "length": len(data), "sha256": sha256(data), "file": filename,
                 "build_id": build_ids[name], "source_sha256": sha256(sources[name])}
        if selector:
            installer_data[address:address+len(data)] = data
            if firmware_info is not None:
                entry["firmware"] = firmware_info
            record = make_record(data, selector, build_ids[name])
            metadata_address = address + APPLICATION_CAPACITY
            installer_data[metadata_address:metadata_address + RECORD_SIZE] = record
            entry.update(metadata_address=metadata_address, metadata_hex=record.hex(), crc32=zlib.crc32(data))
        slots.append(entry)
    combined = bytes(installer_data)
    installer_name = "installer.hex"
    (directory / installer_name).write_text("".join(f"{byte:02X}\n" for byte in combined), encoding="ascii")
    manifest = {"schema": SCHEMA, "device": DEVICE, "flash_size": FLASH_SIZE,
                "slot_size": SLOT_SIZE, "reserved_selector": 3,
                "reserved_address": RESERVED_ADDRESS,
                "slots": slots,
                "installer": {"file": installer_name, "format": "efinity-hex",
                              "length": len(combined), "sha256": sha256(combined)},
                "boot_policy": "Power-on boots golden; golden validates committed metadata and full image CRC32, then tries A before B.",
                "identity_source": "One application build ID with per-slot identity BRAM variants; checked after boot.",
                "references": SOURCE_REFERENCES, "provenance": provenance}
    path = directory / "manifest.json"
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    load_bundle(path)
    return path


def _local_file(directory, filename):
    # Bundles are self-contained. Reject traversal, absolute paths and symlinks
    # rather than reading arbitrary host files named by an untrusted manifest.
    if not isinstance(filename, str) or not filename or Path(filename).name != filename or filename in (".", ".."):
        raise ValueError("bundle filenames must be plain local filenames")
    path = directory / filename
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"missing or symlinked bundle file: {filename}")
    return path


def _check_artifact(entry, data):
    if type(entry.get("length")) is not int or entry["length"] != len(data):
        raise ValueError("artifact length differs from manifest")
    if entry.get("sha256") != sha256(data):
        raise ValueError("artifact SHA256 differs from manifest")


def load_bundle(manifest_path):
    """Validate the complete bundle before returning (manifest, slot bytes).

    SHA256 detects damage and mixing releases. It does not authenticate the
    publisher; the bundle itself must come from a trusted build process.
    """
    path = Path(manifest_path)
    if path.stat().st_size > 65536:
        raise ValueError("manifest is too large")
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError("manifest must be an object")
    expected = {"schema": SCHEMA, "device": DEVICE, "flash_size": FLASH_SIZE,
                "slot_size": SLOT_SIZE, "reserved_selector": 3,
                "reserved_address": RESERVED_ADDRESS}
    for key, value in expected.items():
        if manifest.get(key) != value or type(manifest.get(key)) is not type(value):
            raise ValueError(f"unsupported manifest {key}")
    entries = manifest.get("slots")
    if not isinstance(entries, list) or len(entries) != len(SLOT_LAYOUT):
        raise ValueError("manifest must describe golden, A and B")
    images, names, filenames = {}, set(), set()
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("slot entry must be an object")
        name = entry.get("name")
        if name not in SLOT_LAYOUT or name in names:
            raise ValueError("unknown or duplicate slot name")
        names.add(name)
        selector, address = SLOT_LAYOUT[name]
        for key, value in {"selector": selector, "image_slot": selector,
                           "address": address, "capacity": SLOT_SIZE if selector == 0 else APPLICATION_CAPACITY}.items():
            if type(entry.get(key)) is not int or entry[key] != value:
                raise ValueError(f"incorrect {name} {key}")
        _build_id(entry.get("build_id"))
        filename = entry.get("file")
        image_path = _local_file(path.parent, filename)
        if filename in filenames or image_path.stat().st_size > SLOT_SIZE:
            raise ValueError("duplicate or oversized slot file")
        filenames.add(filename)
        data = image_path.read_bytes()
        _check_artifact(entry, data)
        parse_image(data, require_remote=True)
        if selector:
            if "firmware" in entry:
                if validate_firmware(data[FIRMWARE_OFFSET:]) != entry["firmware"]:
                    raise ValueError("camera firmware differs from manifest")
            if len(data) > APPLICATION_CAPACITY or type(entry.get("metadata_address")) is not int or entry["metadata_address"] != address + APPLICATION_CAPACITY:
                raise ValueError("invalid application metadata address or capacity")
            metadata_hex = entry.get("metadata_hex")
            if not isinstance(metadata_hex, str) or re.fullmatch(r"[0-9a-f]{128}", metadata_hex) is None:
                raise ValueError("application metadata must be 64 canonical hexadecimal bytes")
            parsed = parse_record(bytes.fromhex(metadata_hex), selector, data)
            if parsed["build_id"] != entry["build_id"] or type(entry.get("crc32")) is not int or entry["crc32"] != parsed["crc32"]:
                raise ValueError("application record identity differs from manifest")
        elif any(key in entry for key in ("metadata_address", "metadata_hex", "crc32")):
            raise ValueError("golden image does not have application metadata")
        images[name] = data
    _check_build_ids({entry["name"]: entry["build_id"] for entry in entries})
    installer = manifest.get("installer")
    if not isinstance(installer, dict) or installer.get("format") != "efinity-hex":
        raise ValueError("installer must use Efinity hex format")
    installer_path = _local_file(path.parent, installer.get("file"))
    if installer.get("file") in filenames:
        raise ValueError("installer cannot share a slot filename")
    combined = _read_hex(installer_path)
    _check_artifact(installer, combined)
    pure_installer = bytearray(combined)
    for entry in entries:
        if entry["selector"]:
            start = entry["metadata_address"]
            record = bytes.fromhex(entry["metadata_hex"])
            if combined[start:start + RECORD_SIZE] != record:
                raise ValueError("installer application metadata differs from manifest")
            pure_installer[start:start + RECORD_SIZE] = b"\xff" * RECORD_SIZE
    if validate_combined(bytes(pure_installer), images) != images:
        raise ValueError("installer does not contain exact slot binaries")
    return manifest, images


def _validate_vendor_database(root):
    """Fail rather than silently apply assumptions to an incompatible toolset."""
    path = root / "pgm/bin/device_db.xml"
    database = ET.parse(path).getroot()
    matches = [die for die in database.findall("die") if die.get("name") == "Ti60_MP"]
    if len(matches) != 1 or matches[0].find("idcode").get("value").lower() != "0x10660a79":
        raise ValueError("installed Efinity database does not identify production Ti60")
    options = matches[0].find("option_register")
    if options.get("sync").lower() != SYNC.hex() or int(options.get("length")) != 8 * OPTION_BYTES:
        raise ValueError("unsupported Ti60 option register layout")
    fields = {field.get("id"): (int(field.get("address")), int(field.get("length")))
              for field in options.findall("feature")}
    if any(fields.get(name) != value for name, value in OPTION_FIELDS.items()):
        raise ValueError("installed Ti60 remote-update field layout changed")
    return sha256(path.read_bytes())


def build_bundle(efinity, image_paths, build_ids, output_dir, *, firmware=None):
    """Run the unmodified, installed Efinity merger; no hardware is accessed."""
    root = Path(efinity).resolve()
    output = Path(output_dir).resolve()
    if output.exists():
        raise FileExistsError(f"output directory already exists: {output}")
    if set(image_paths) != set(SLOT_LAYOUT):
        raise ValueError("golden, A and B input paths are required")
    sources = {}
    for name, path in image_paths.items():
        if Path(path).suffix.lower() != ".hex":
            raise ValueError("source must be an Efinity SPI .hex, not .bit or raw binary")
        sources[name] = _read_hex(path, SLOT_SIZE)
        parse_image(sources[name])
    _check_build_ids(build_ids)
    database_sha = _validate_vendor_database(root)
    merger = root / "pgm/bin/multi_image_merger.py"
    python = root / "bin/python3"
    if not merger.is_file() or not python.is_file():
        raise ValueError("Efinity installation is missing its merger or Python runtime")
    from litex.build.efinix.toolchain import load_efinity_env

    env = load_efinity_env(str(root))
    env["PYTHONPATH"] = str(root / "pgm/bin") + ":" + str(root / "debugger/bin") + ":" + env.get("PYTHONPATH", "")
    # Canonical temporary source copies avoid input TOCTOU and normalize CRLF
    # because the vendor merger requires exactly three characters per line.
    with tempfile.TemporaryDirectory(prefix="lutcam-ota-bundle-") as temporary:
        directory = Path(temporary)
        command = [str(python), str(merger), "-m", "selectable_flash_image", "-t", "internal"]
        for name, (_, address) in SLOT_LAYOUT.items():
            source_path = directory / f"source-{name}.hex"
            source_path.write_text("".join(f"{byte:02X}\n" for byte in sources[name]), encoding="ascii")
            command += ["-ifile", str(source_path), "-iaddr", f"0x{address:x}"]
        combined_path = directory / "installer.hex"
        command += ["-iaddr", f"0x{RESERVED_ADDRESS:x}", "-o", str(combined_path)]
        process = subprocess.run(command, env=env, cwd=directory, capture_output=True, text=True, timeout=120)
        if process.returncode or not combined_path.is_file() or re.search(r"\bERROR:", process.stdout + process.stderr):
            raise ValueError("Efinity merger failed:\n" + process.stdout + process.stderr)
        combined = _read_hex(combined_path)
        provenance = {"efinity": str(root), "merger_sha256": sha256(merger.read_bytes()),
                      "device_database_sha256": database_sha, "merger_mode": "selectable_flash_image/internal"}
        result = write_bundle(output, combined, sources, build_ids, provenance,
                              firmware=Path(firmware).read_bytes() if firmware else None)
        (output / "merger.log").write_text(process.stdout + process.stderr, encoding="utf-8")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    build = commands.add_parser("build", help="create a new vendor-merged installer and OTA bundle")
    build.add_argument("--efinity", required=True, type=Path)
    for name in ("golden", "slot-a", "slot-b"):
        build.add_argument("--" + name, required=True, type=Path)
        build.add_argument("--" + name + "-build-id", required=True)
    build.add_argument("--output-dir", required=True, type=Path)
    build.add_argument("--firmware", type=Path, help="flash-loaded camera firmware.flash.bin for both A/B slots")
    verify = commands.add_parser("verify", help="validate all bundle artifacts without hardware access")
    verify.add_argument("manifest", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "build":
            path = build_bundle(args.efinity, {"golden": args.golden, "A": args.slot_a, "B": args.slot_b},
                                {"golden": args.golden_build_id, "A": args.slot_a_build_id, "B": args.slot_b_build_id},
                                args.output_dir, firmware=args.firmware)
            print(f"Created and verified {path}")
        else:
            manifest, _ = load_bundle(args.manifest)
            print(f"Verified {manifest['device']} bundle with golden, A and B images")
    except (OSError, ValueError, ET.ParseError, subprocess.TimeoutExpired) as error:
        parser.exit(1, f"OTA bundle error: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
