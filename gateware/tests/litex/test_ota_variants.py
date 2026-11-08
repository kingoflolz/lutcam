"""Variant generation may change ROM contents, but never a routed design."""

import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from fpga.litex.lutcam.image_identity import IDENTITY_MEMORY, identity_words
from tools.lutcam.ota_variants import generate_variants, read_memory_dump, sha256


BUILD_ID = "0123456789abcdeffedcba9876543210"


def vendor_dump(words, memory=IDENTITY_MEMORY):
    bits = "".join("".join(str((word >> bit) & 1) for bit in range(32))
                   for word in words)
    return f'BRAM Initial Content Updater\n***** Memory "{memory}" *****\n{bits}\n'


class OtaVariantTests(unittest.TestCase):
    def make_project(self, root, *, crc="on"):
        directory = root / "gateware"
        (directory / "outflow").mkdir(parents=True)
        (directory / "work_pnr").mkdir()
        project = directory / "lutcam.xml"
        project.write_text(f'''<efx:project xmlns:efx="http://www.efinixinc.com/enf_proj" name="lutcam">
  <efx:device_info>
    <efx:family name="Titanium"/><efx:device name="Ti60F225"/>
  </efx:device_info>
  <efx:bitstream_generation>
    <efx:param name="enable_roms" value="on"/>
    <efx:param name="enable_crc_check" value="{crc}"/>
    <efx:param name="spi_width" value="4"/>
    <efx:param name="generate_bit" value="on"/>
    <efx:param name="generate_hex" value="on"/>
  </efx:bitstream_generation>
</efx:project>''')
        for name in ("lutcam.raminfo.pb", "lutcam.place", "lutcam.lpf",
                     "lutcam_or.ini", "lutcam.timing.rpt", "lutcam.timing-check.json"):
            (directory / "outflow" / name).write_text(f"protected {name}\n")
        for name in ("lutcam.lbf", "lutcam.route"):
            (directory / "work_pnr" / name).write_text(f"protected {name}\n")
        return project

    def fake_vendor(self, *, wrong_readback=False, mutate=None):
        def run(command, **kwargs):
            executable = Path(command[0]).name
            self.assertEqual(kwargs["cwd"].name, "variants")
            if executable == "efx_bram_update":
                mode = command[command.index("--mode") + 1]
                init = Path(command[command.index("--memory") + 1].split(",", 1)[1])
                if mode == "update":
                    Path(command[command.index("--output") + 1]).write_text(init.read_text())
                    if mutate is not None:
                        mutate.write_text("unexpected vendor mutation\n")
                    output = "BRAM update completed\n"
                else:
                    self.assertEqual(mode, "read")
                    words = [int(word, 16) for word in init.read_text().splitlines()]
                    if wrong_readback:
                        words[2] ^= 1
                    output = vendor_dump(words)
            elif executable == "efx_pgm":
                source = Path(command[command.index("--source") + 1]).read_text()
                destination = Path(command[command.index("--dest") + 1])
                destination.write_text(f"hex {source}")
                destination.with_suffix(".bit").write_text(f"bit {source}")
                output = "Bitstream generation completed\n"
            else:
                self.fail(f"variant generation invoked forbidden tool {executable}")
            return subprocess.CompletedProcess(command, 0, stdout=output)
        return run

    def run_generator(self, project, output, **vendor_options):
        with patch("litex.build.efinix.toolchain.load_efinity_env", return_value={}), \
                patch("tools.lutcam.ota_variants.subprocess.run",
                      side_effect=self.fake_vendor(**vendor_options)) as vendor:
            result = generate_variants(project, BUILD_ID, output, efinity="/fake/efinity")
        return result, vendor

    def test_readback_decodes_lsb_first_and_rejects_malformed_tables(self):
        words = [0x80000001, 0x01234567, 0x89ABCDEF, 0xFEDCBA98] * 4
        dump = vendor_dump(words)
        self.assertEqual(read_memory_dump(dump), words)
        header, bits = dump.rsplit("\n", 2)[:2]
        for broken in ("", vendor_dump(words, "different_memory"),
                       header + "\n" + bits[:-1] + "\n",
                       header + "\n" + bits + "0\n",
                       header + "\n" + bits + "2\n",
                       header + "\n" + bits[:250] + "x" + bits[251:] + "\n"):
            with self.subTest(dump=broken):
                with self.assertRaises(ValueError):
                    read_memory_dump(broken)

    def test_both_images_are_verified_without_synthesis_or_routing(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project = self.make_project(root)
            protected = {path: sha256(path) for path in project.parent.rglob("*")
                         if path.is_file()}
            result, vendor = self.run_generator(project, root / "variants")
            self.assertEqual(result["pnr_runs_for_variants"], 0)
            self.assertTrue(result["routed_artifacts_unchanged"])
            self.assertEqual(protected, {path: sha256(path) for path in protected})
            commands = [call.args[0] for call in vendor.call_args_list]
            stages = [(Path(command[0]).name,
                       command[command.index("--mode") + 1] if "--mode" in command else None)
                      for command in commands]
            self.assertEqual(stages, [("efx_bram_update", "update"),
                                      ("efx_bram_update", "read"),
                                      ("efx_pgm", None)] * 2)
            for command in commands:
                if Path(command[0]).name == "efx_pgm":
                    self.assertIn("--enable_crc_check=on", command)
                    self.assertIn("--spi_width=4", command)
            for slot, slot_id in (("A", 1), ("B", 2)):
                image = result["images"][slot]
                self.assertEqual(image["slot"], slot_id)
                self.assertTrue(image["identity_readback_verified"])
                self.assertEqual(read_memory_dump((root / "variants" /
                                 f"{slot}.bram-readback.log").read_text()),
                                 identity_words(BUILD_ID, slot))
                for extension in ("hex", "bit"):
                    self.assertEqual(image[extension]["sha256"],
                                     sha256(image[extension]["path"]))
            self.assertNotEqual(result["images"]["A"]["hex"]["sha256"],
                                result["images"]["B"]["hex"]["sha256"])
            self.assertEqual(json.loads((root / "variants" / "variants.json").read_text()),
                             result)

    def test_wrong_readback_stops_before_bitstream_generation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project = self.make_project(root)
            with patch("litex.build.efinix.toolchain.load_efinity_env", return_value={}), \
                    patch("tools.lutcam.ota_variants.subprocess.run",
                          side_effect=self.fake_vendor(wrong_readback=True)) as vendor:
                with self.assertRaisesRegex(RuntimeError, "readback mismatch"):
                    generate_variants(project, BUILD_ID, root / "variants",
                                      efinity="/fake/efinity")
            self.assertEqual(vendor.call_count, 2)
            self.assertFalse((root / "variants" / "A.hex").exists())
            self.assertFalse((root / "variants" / "variants.json").exists())

    def test_routing_artifact_mutation_invalidates_the_result(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project = self.make_project(root)
            mutation = project.parent / "work_pnr" / "lutcam.route"
            with self.assertRaisesRegex(RuntimeError, "protected synthesis/routing artifact"):
                self.run_generator(project, root / "variants", mutate=mutation)
            self.assertFalse((root / "variants" / "variants.json").exists())

    def test_unsafe_output_and_crc_disabled_projects_never_run_vendor_tools(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project = self.make_project(root)
            occupied = root / "occupied"
            occupied.mkdir()
            existing = occupied / "keep.txt"
            existing.write_text("existing output must survive\n")
            with patch("tools.lutcam.ota_variants.subprocess.run") as vendor:
                for output in (project.parent, project.parent / "nested", occupied):
                    with self.subTest(output=output):
                        with self.assertRaises(ValueError):
                            generate_variants(project, BUILD_ID, output, efinity="/fake/efinity")
                project.write_text(project.read_text().replace(
                    'name="enable_crc_check" value="on"',
                    'name="enable_crc_check" value="off"'))
                with self.assertRaisesRegex(ValueError, "configuration CRC"):
                    generate_variants(project, BUILD_ID, root / "variants",
                                      efinity="/fake/efinity")
                vendor.assert_not_called()
            self.assertEqual(existing.read_text(), "existing output must survive\n")


if __name__ == "__main__":
    unittest.main()
