"""Generated camera HDL must find its ROMs after the build directory is moved."""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

from migen import Instance, Module
from litex.build.generic_platform import GenericPlatform
from litex.gen import LiteXContext

from fpga.litex.lutcam.camera import CAMERA_ROM_FILES, add_camera_sources
from fpga.litex.lutcam.platform import Platform
from tests.litex.test_camera_calibration import camera_fixture


ROOT = Path(__file__).resolve().parents[2]


class CameraRomFilesTests(unittest.TestCase):
    def setUp(self):
        for name in ("platform", "device", "toolchain"):
            self.addCleanup(setattr, LiteXContext, name, getattr(LiteXContext, name))
        # Exercise the board's actual HDL/data-file exporter without requiring
        # Efinity's physical-device database or constructing clock peripherals.
        self.platform = Platform.__new__(Platform)
        GenericPlatform.__init__(self.platform, "Ti60F225C3", [])
        self.platform._rom_files = {}

    @unittest.skipUnless(shutil.which("iverilog") and shutil.which("vvp"),
                         "Icarus Verilog is required")
    def test_relative_rom_filenames_work_after_moving_generated_hdl(self):
        add_camera_sources(self.platform, ROOT)
        camera = camera_fixture(direct=True)
        pipeline = next(item for item in camera.get_fragment().specials
                        if isinstance(item, Instance) and item.of == "CameraPipeline")
        parameters = {item.name: item.value for item in pipeline.items
                      if isinstance(item, Instance.Parameter)}
        design = Module()
        expected_files = {}
        for parameter, relative in CAMERA_ROM_FILES.items():
            source = ROOT / relative
            filename = parameters[parameter]
            self.assertEqual(filename, source.name)
            contents = source.read_text()
            expected_files[filename] = contents
            words = contents.splitlines()
            design.specials += Instance("RomProbe", p_FILENAME=filename,
                p_WIDTH=len(words[0]), p_DEPTH=len(words),
                p_EXPECTED_SUM=sum(int(word, 2) for word in words))
        output = self.platform.get_verilog(design, name="rom_check")
        self.assertNotIn(str(ROOT), output.main_source)
        self.assertEqual(output.data_files, expected_files)
        with tempfile.TemporaryDirectory(prefix="lutcam-rom-relocation-") as directory:
            original = Path(directory) / "original"
            original.mkdir()
            # ConvOutput.write uses the build cwd for its sidecar data files,
            # exactly as LiteX's GenericToolchain does.
            previous = Path.cwd()
            try:
                os.chdir(original)
                output.write("rom_check.v")
            finally:
                os.chdir(previous)
            (original / "rom_probe.sv").write_text('''
module RomProbe #(parameter FILENAME = "", WIDTH = 1, DEPTH = 1,
                  parameter [63:0] EXPECTED_SUM = 0) ();
    reg [WIDTH-1:0] words [0:DEPTH-1];
    reg [63:0] total;
    initial begin
        $readmemb(FILENAME, words);
        total = 0;
        for (integer i = 0; i < DEPTH; i = i + 1)
            total = total + words[i];
        if (total !== EXPECTED_SUM) $fatal(1, "ROM contents differ");
        #1 $finish;
    end
endmodule
''')
            moved = Path(directory) / "moved build"
            original.rename(moved)
            self.assertFalse(original.exists())
            for filename, contents in expected_files.items():
                self.assertEqual((moved / filename).read_text(), contents)
            for command in (["iverilog", "-g2012", "-s", "rom_check", "-o", "rom_check",
                             "rom_check.v", "rom_probe.sv"], ["vvp", "rom_check"]):
                result = subprocess.run(command, cwd=moved, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_conflicting_rom_basenames_are_rejected(self):
        with tempfile.TemporaryDirectory(prefix="lutcam-rom-collision-") as directory:
            for subdirectory, contents in (("first", "0\n"), ("second", "1\n")):
                path = Path(directory) / subdirectory / "table.bin"
                path.parent.mkdir()
                path.write_text(contents)
            first = Path(directory) / "first/table.bin"
            self.platform.add_rom_file(first)
            self.platform.add_rom_file(first)
            with self.assertRaisesRegex(ValueError, "Conflicting ROM contents"):
                self.platform.add_rom_file(Path(directory) / "second/table.bin")


if __name__ == "__main__":
    unittest.main()
