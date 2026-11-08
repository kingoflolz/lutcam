"""Reject malformed OTA artifacts before any attached flash can be modified."""

import hashlib
import json
from pathlib import Path
import shutil
import struct
import tempfile
import unittest
import zlib

from tools.lutcam.ota_bundle import (
    APPLICATION_CAPACITY, FLASH_SIZE, SLOT_LAYOUT, SLOT_SIZE, decode_efinity_hex,
    load_bundle, make_record, parse_image, parse_record, validate_combined,
    write_bundle,
)


SYNC = bytes.fromhex("168a2236")
OPTION_START = 260
OPTION_LENGTH = 144
INSTALLER_LENGTH = 0x600000
BUILD_IDS = {"golden": "0" * 32, "A": "a" * 32, "B": "a" * 32}


def expected_record(image, selector, build_id):
    header = (b"LUTCOTA1" + struct.pack("<IIII", 0, selector, len(image),
                                      zlib.crc32(image)) + bytes.fromhex(build_id))
    return header + struct.pack("<II", zlib.crc32(header), 0x434f4d54) + b"\xff" * 16


def option_field(image, position, width, value):
    """Use Ti60 option-register bit numbers, independent of bundle helpers."""
    result = bytearray(image)
    options = int.from_bytes(result[OPTION_START:OPTION_START + OPTION_LENGTH], "big")
    mask = ((1 << width) - 1) << position
    options = (options & ~mask) | (value << position)
    result[OPTION_START:OPTION_START + OPTION_LENGTH] = options.to_bytes(OPTION_LENGTH, "big")
    return bytes(result)


def synthetic_image(tag=0, **metadata):
    """Small structural fixture, never a bitstream suitable for programming."""
    fields = {"Family": "Titanium", "Device": "Ti60F225", "Width": "4",
              "Mode": "active", "Die Name": "Ti60_MP", "4Byte": "0"}
    fields.update(metadata)
    header = "Version: test\n" + "".join(f"{key}: {value}\n" for key, value in fields.items())
    preamble = header.encode("ascii").ljust(256, b"\n")
    return preamble + SYNC + bytes(OPTION_LENGTH) + bytes([tag]) * 264


def remote_image(source):
    result = option_field(source, 939, 1, 1)
    for position, address in ((291, 0x200000), (315, 0x400000), (339, 0x600000)):
        result = option_field(result, position, 24, address)
    return result


SOURCES = {name: synthetic_image(tag) for tag, name in enumerate(SLOT_LAYOUT, 1)}
IMAGES = {name: remote_image(source) for name, source in SOURCES.items()}
_combined = bytearray(b"\xff" * INSTALLER_LENGTH)
for _name, (_, _address) in SLOT_LAYOUT.items():
    _combined[_address:_address + len(IMAGES[_name])] = IMAGES[_name]
COMBINED = bytes(_combined)
del _combined, _name, _address


def expected_installer():
    result = bytearray(COMBINED)
    for name in ("A", "B"):
        selector, address = SLOT_LAYOUT[name]
        offset = address + 0x1ff000
        result[offset:offset + 64] = expected_record(IMAGES[name], selector, BUILD_IDS[name])
    return bytes(result)


class HexAndImageTests(unittest.TestCase):
    def test_fixed_board_layout(self):
        self.assertEqual(FLASH_SIZE, 16 * 1024 * 1024)
        self.assertEqual(SLOT_SIZE, 2 * 1024 * 1024)
        self.assertEqual(APPLICATION_CAPACITY, 0x1ff000)
        self.assertEqual(SLOT_LAYOUT, {"golden": (0, 0), "A": (1, 0x200000),
                                     "B": (2, 0x400000)})

    def test_hex_decodes_one_byte_per_line_with_crlf(self):
        self.assertEqual(decode_efinity_hex("00\r\naB\r\nFF\r\n"), b"\x00\xab\xff")
        self.assertEqual(decode_efinity_hex("00\nAB\nff"), b"\x00\xab\xff")

    def test_hex_rejects_intel_hex_and_loose_hex_notation(self):
        for text in ("", ":020000000102FB\n:00000001FF\n", "0\n", "0001\n",
                     "GG\n", "0x00\n", "00 01\n", "00\n\n01\n"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                decode_efinity_hex(text)

    def test_image_accepts_source_and_remote_structure(self):
        parse_image(SOURCES["A"])
        parse_image(IMAGES["A"], require_remote=True)

    def test_image_rejects_wrong_target_or_flash_mode(self):
        for key, value in (("Family", "Trion"), ("Device", "Ti180J484"),
                           ("Die Name", "Ti60_ES"), ("Width", "1"),
                           ("Mode", "passive"), ("4Byte", "1")):
            with self.subTest(field=key), self.assertRaises(ValueError):
                parse_image(synthetic_image(**{key: value}))

    def test_image_rejects_absent_or_nonascii_metadata_and_truncation(self):
        original = SOURCES["A"]
        malformed = (
            original.replace(b"Device: Ti60F225", b"Unused: Ti60F225", 1),
            b"\x80" + original[1:],
            original[:256], original[:OPTION_START + OPTION_LENGTH - 1],
            original[:256] + b"\x00\x00\x00\x00" + original[260:],
        )
        for image in malformed:
            with self.subTest(length=len(image)), self.assertRaises(ValueError):
                parse_image(image)

    def test_remote_configuration_requires_internal_selector_and_fixed_addresses(self):
        for image in (SOURCES["A"], option_field(IMAGES["A"], 290, 1, 1),
                      option_field(IMAGES["A"], 393, 1, 1),
                      option_field(IMAGES["A"], 291, 24, 0),
                      option_field(IMAGES["A"], 315, 24, 0x200000),
                      option_field(IMAGES["A"], 339, 24, 0x500000)):
            with self.subTest(image=image[OPTION_START:OPTION_START + OPTION_LENGTH].hex()), \
                    self.assertRaises(ValueError):
                parse_image(image, require_remote=True)


class CombinedImageTests(unittest.TestCase):
    def test_extracts_patched_images_at_exact_source_lengths(self):
        self.assertEqual(validate_combined(COMBINED, SOURCES), IMAGES)

    def test_rejects_payload_or_preamble_mutation(self):
        for offset in (0x200000 + 10, 0x400000 + OPTION_START + OPTION_LENGTH + 10):
            corrupted = bytearray(COMBINED)
            corrupted[offset] ^= 1
            with self.subTest(offset=offset), self.assertRaises(ValueError):
                validate_combined(bytes(corrupted), SOURCES)

    def test_rejects_nonblank_gaps_and_reserved_selector(self):
        for offset in (len(IMAGES["golden"]), 0x200000 - 1,
                       0x200000 + len(IMAGES["A"]), INSTALLER_LENGTH - 1):
            corrupted = bytearray(COMBINED)
            corrupted[offset] = 0
            with self.subTest(offset=offset), self.assertRaises(ValueError):
                validate_combined(bytes(corrupted), SOURCES)

    def test_rejects_wrong_installer_size_and_extra_image_data(self):
        for combined in (COMBINED[:-1], COMBINED + b"\xff", COMBINED + b"\x00"):
            with self.subTest(length=len(combined)), self.assertRaises(ValueError):
                validate_combined(combined, SOURCES)

    def test_rejects_oversize_source_and_missing_or_extra_slots(self):
        variants = [dict(SOURCES), dict(SOURCES), dict(SOURCES)]
        variants[0]["A"] = SOURCES["A"].ljust(APPLICATION_CAPACITY + 1, b"\0")
        del variants[1]["B"]
        variants[2]["other"] = SOURCES["B"]
        for sources in variants:
            with self.subTest(slots=list(sources)), self.assertRaises(ValueError):
                validate_combined(COMBINED, sources)


class CommitRecordTests(unittest.TestCase):
    def test_record_has_independent_byte_exact_encoding(self):
        for name in ("A", "B"):
            selector = SLOT_LAYOUT[name][0]
            image, build_id = IMAGES[name], BUILD_IDS[name]
            record = make_record(image, selector, build_id)
            with self.subTest(slot=name):
                self.assertEqual(len(record), 64)
                self.assertEqual(record, expected_record(image, selector, build_id))
                self.assertIsInstance(parse_record(record, selector, image), dict)
                self.assertIsInstance(parse_record(record, selector), dict)

    def test_rejects_uncommitted_and_partially_written_records(self):
        record = expected_record(IMAGES["A"], 1, BUILD_IDS["A"])
        malformed = (b"\xff" * 64, record[:44] + b"\xff" * 20,
                     record[:44] + b"TMO\xff" + record[48:], record[:63],
                     record + b"\xff")
        for candidate in malformed:
            with self.subTest(length=len(candidate)), self.assertRaises(ValueError):
                parse_record(candidate, 1)

    def test_rejects_header_payload_and_padding_corruption(self):
        record = expected_record(IMAGES["A"], 1, BUILD_IDS["A"])
        for offset in (0, 8, 12, 16, 20, 24, 40, 44, 48, 63):
            corrupt = bytearray(record)
            corrupt[offset] ^= 1
            with self.subTest(offset=offset), self.assertRaises(ValueError):
                parse_record(bytes(corrupt), 1, IMAGES["A"])
        image = IMAGES["A"][:-1] + b"\x99"
        with self.assertRaises(ValueError):
            parse_record(record, 1, image)
        with self.assertRaises(ValueError):
            parse_record(record, 1, IMAGES["A"][:-1])

    def test_rejects_invalid_fields_even_with_valid_header_crc(self):
        record = expected_record(IMAGES["A"], 1, BUILD_IDS["A"])
        for offset, value in ((12, 0), (12, 2), (12, 3),
                              (16, 0), (16, APPLICATION_CAPACITY + 1)):
            corrupt = bytearray(record)
            struct.pack_into("<I", corrupt, offset, value)
            struct.pack_into("<I", corrupt, 40, zlib.crc32(corrupt[:40]))
            with self.subTest(offset=offset, value=value), self.assertRaises(ValueError):
                parse_record(bytes(corrupt), 1)


    def test_record_builder_rejects_non_application_slots_invalid_ids_and_capacity(self):
        for selector in (0, 3, -1):
            with self.subTest(selector=selector), self.assertRaises(ValueError):
                make_record(IMAGES["A"], selector, BUILD_IDS["A"])
        for build_id in ("a" * 31, "A" * 32, "g" * 32):
            with self.subTest(build_id=build_id), self.assertRaises(ValueError):
                make_record(IMAGES["A"], 1, build_id)
        for image in (b"", b"\0" * (APPLICATION_CAPACITY + 1)):
            with self.subTest(length=len(image)), self.assertRaises(ValueError):
                make_record(image, 1, BUILD_IDS["A"])


class BundleManifestTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.template_dir = tempfile.TemporaryDirectory()
        cls.template = Path(cls.template_dir.name) / "bundle"
        cls.manifest_path = write_bundle(cls.template, COMBINED, SOURCES, BUILD_IDS,
                                         provenance={})

    @classmethod
    def tearDownClass(cls):
        cls.template_dir.cleanup()

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.bundle = Path(self.temp_dir.name) / "bundle"
        shutil.copytree(self.template, self.bundle)
        self.path = self.bundle / Path(self.manifest_path).name
        self.manifest = json.loads(self.path.read_text())

    def save_manifest(self):
        self.path.write_text(json.dumps(self.manifest))

    def test_rejects_different_application_releases_even_with_valid_metadata(self):
        build_ids = dict(BUILD_IDS, B="b" * 32)
        with self.assertRaisesRegex(ValueError, "same application build ID"):
            write_bundle(Path(self.temp_dir.name) / "mixed", COMBINED, SOURCES,
                         build_ids, provenance={})
        entry = next(slot for slot in self.manifest["slots"] if slot["name"] == "B")
        entry["build_id"] = build_ids["B"]
        record = expected_record(IMAGES["B"], 2, entry["build_id"])
        entry["metadata_hex"] = record.hex()
        combined = bytearray(expected_installer())
        offset = entry["metadata_address"]
        combined[offset:offset + 64] = record
        installer = self.manifest["installer"]
        installer["sha256"] = hashlib.sha256(combined).hexdigest()
        (self.bundle / installer["file"]).write_text("".join(f"{byte:02X}\n" for byte in combined))
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "same application build ID"):
            load_bundle(self.path)

    def assert_invalid(self):
        self.save_manifest()
        with self.assertRaises(ValueError):
            load_bundle(self.path)

    def test_bundle_round_trip_and_declared_hashes(self):
        manifest, images = load_bundle(self.path)
        self.assertEqual(images, IMAGES)
        self.assertEqual(manifest["schema"], "lutcam-ota-v1")
        self.assertEqual(manifest["device"], "Ti60F225")
        self.assertEqual(manifest["flash_size"], FLASH_SIZE)
        self.assertEqual(manifest["slot_size"], SLOT_SIZE)
        for slot in manifest["slots"]:
            name = slot["name"]
            self.assertEqual((slot["selector"], slot["address"]), SLOT_LAYOUT[name])
            self.assertEqual(slot["length"], len(IMAGES[name]))
            capacity = SLOT_SIZE if name == "golden" else APPLICATION_CAPACITY
            self.assertEqual(slot["capacity"], capacity)
            self.assertEqual(slot["sha256"], hashlib.sha256(IMAGES[name]).hexdigest())
            self.assertEqual(slot["build_id"], BUILD_IDS[name])
            self.assertEqual(slot["image_slot"], slot["selector"])
            if name != "golden":
                self.assertEqual(slot["metadata_address"], slot["address"] + APPLICATION_CAPACITY)
                self.assertEqual(slot["metadata_hex"],
                                 expected_record(IMAGES[name], slot["selector"], BUILD_IDS[name]).hex())
                self.assertEqual(slot["crc32"], zlib.crc32(IMAGES[name]))
        self.assertEqual(manifest["installer"]["format"], "efinity-hex")
        self.assertEqual(manifest["installer"]["length"], len(COMBINED))
        self.assertEqual(manifest["installer"]["sha256"],
                         hashlib.sha256(expected_installer()).hexdigest())

    def test_writer_refuses_existing_output_directory(self):
        with self.assertRaises((ValueError, FileExistsError)):
            write_bundle(self.bundle, COMBINED, SOURCES, BUILD_IDS, provenance={})

    def test_writer_rejects_invalid_build_ids(self):
        for index, build_id in enumerate(("a" * 31, "a" * 33, "A" * 32, "g" * 32)):
            ids = dict(BUILD_IDS, A=build_id)
            with self.subTest(build_id=build_id), self.assertRaises(ValueError):
                write_bundle(Path(self.temp_dir.name) / f"new-bundle-{index}", COMBINED,
                             SOURCES, ids, provenance={})

    def test_manifest_rejects_wrong_schema_and_device(self):
        for key, value in (("device", "Ti180J484"),
                           ("flash_size", FLASH_SIZE * 2), ("slot_size", SLOT_SIZE // 2)):
            original = self.manifest[key]
            self.manifest[key] = value
            with self.subTest(field=key):
                self.assert_invalid()
            self.manifest[key] = original

    def test_manifest_rejects_missing_duplicate_and_overlapping_slots(self):
        original = self.manifest["slots"]
        variants = [original[:-1], original + [original[0]],
                    [dict(slot, address=0) if slot["name"] == "A" else slot for slot in original]]
        for slots in variants:
            self.manifest["slots"] = slots
            with self.subTest(slots=[slot["name"] for slot in slots]):
                self.assert_invalid()

    def test_manifest_rejects_slot_metadata_mismatch(self):
        slot = self.manifest["slots"][1]
        for key, value in (("selector", 0), ("capacity", SLOT_SIZE * 2),
                           ("length", slot["length"] - 1), ("sha256", "0" * 64),
                           ("image_slot", 2), ("build_id", "A" * 32)):
            original = slot[key]
            slot[key] = value
            with self.subTest(field=key):
                self.assert_invalid()
            slot[key] = original

    def test_manifest_rejects_traversal_and_absolute_paths(self):
        for entry in (self.manifest["slots"][0], self.manifest["installer"]):
            original = entry["file"]
            for filename in ("../outside.bin", "/tmp/outside.bin"):
                entry["file"] = filename
                with self.subTest(filename=filename):
                    self.assert_invalid()
            entry["file"] = original

    def test_manifest_rejects_symlink_escaping_bundle(self):
        entry = self.manifest["slots"][0]
        image_path = self.bundle / entry["file"]
        outside = Path(self.temp_dir.name) / "outside.bin"
        image_path.rename(outside)
        image_path.symlink_to(outside)
        self.assert_invalid()

    def test_manifest_rejects_altered_payload_and_installer_hash_or_length(self):
        image_path = self.bundle / self.manifest["slots"][1]["file"]
        image = image_path.read_bytes()
        image_path.write_bytes(image[:-1] + bytes([image[-1] ^ 1]))
        self.assert_invalid()
        image_path.write_bytes(image)
        for key, value in (("length", len(COMBINED) - 1), ("sha256", "0" * 64)):
            original = self.manifest["installer"][key]
            self.manifest["installer"][key] = value
            with self.subTest(field=key):
                self.assert_invalid()
            self.manifest["installer"][key] = original

    def test_manifest_rejects_image_and_installer_disagreement_despite_matching_hashes(self):
        entry = self.manifest["slots"][1]
        image_path = self.bundle / entry["file"]
        image = bytearray(image_path.read_bytes())
        image[-1] ^= 1
        image_path.write_bytes(image)
        entry["sha256"] = hashlib.sha256(image).hexdigest()
        self.assert_invalid()

    def test_manifest_rejects_nonblank_installer_gap_despite_matching_hash(self):
        entry = self.manifest["installer"]
        installer_path = self.bundle / entry["file"]
        offset = len(IMAGES["golden"])
        with installer_path.open("r+b") as handle:
            line_size = len(handle.readline())
            handle.seek(offset * line_size)
            handle.write(b"00")
        installer = bytearray(expected_installer())
        installer[offset] = 0
        entry["sha256"] = hashlib.sha256(installer).hexdigest()
        self.assert_invalid()

    def test_manifest_rejects_metadata_record_mismatch(self):
        slot = self.manifest["slots"][1]
        for key, value in (("metadata_address", 0x3fe000), ("metadata_hex", "ff" * 64),
                           ("crc32", slot["crc32"] ^ 1)):
            original = slot[key]
            slot[key] = value
            with self.subTest(field=key):
                self.assert_invalid()
            slot[key] = original

    def test_installer_rejects_uncommitted_record_with_matching_installer_hash(self):
        entry = self.manifest["installer"]
        installer_path = self.bundle / entry["file"]
        offset = 0x3ff000 + 44
        with installer_path.open("r+b") as handle:
            line_size = len(handle.readline())
            for index in range(4):
                handle.seek((offset + index) * line_size)
                handle.write(b"FF")
        installer = bytearray(expected_installer())
        installer[offset:offset + 4] = b"\xff" * 4
        entry["sha256"] = hashlib.sha256(installer).hexdigest()
        self.assert_invalid()


if __name__ == "__main__":
    unittest.main()
