"""Shared projection arithmetic, output stalls, and abort boundaries."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
DSP_MODEL = Path(os.environ.get("LITEX_ENV_EFINITY", "/nonexistent")) / "sim_models/verilog/efx_dsp48.v"

@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class SharedProjectionTests(unittest.TestCase):
    def run_mode(self, native):
        with tempfile.TemporaryDirectory(prefix="lutcam-shared-projection-") as tmp:
            command = ["verilator", "--binary", "--timing", "--assert", "-Wno-fatal",
                       "-j", "2", "--top-module", "shared_projection_tb", "--Mdir", tmp,
                       str(ROOT / "rtl/lftag/LfTagHomogeneousGeometryScorer.sv"),
                       str(Path(__file__).with_name("shared_projection_tb.sv"))]
            if native:
                command += ["-DSYNTHESIS", str(DSP_MODEL)]
            built = subprocess.run(command, capture_output=True, text=True, timeout=120)
            self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
            result = subprocess.run([str(Path(tmp) / "Vshared_projection_tb")],
                                    capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS shared projection jobs=19 clear=9 reset=9", result.stdout)

    def test_portable_arithmetic_and_abort_boundaries(self):
        self.run_mode(False)

    @unittest.skipUnless(DSP_MODEL.is_file(), "LITEX_ENV_EFINITY vendor model is required")
    def test_native_arithmetic_and_abort_boundaries(self):
        self.run_mode(True)

if __name__ == "__main__":
    unittest.main()
