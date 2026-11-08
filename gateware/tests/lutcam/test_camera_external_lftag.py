"""External LFTag mode through the real production camera frontend RTL."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class CameraExternalLfTagTests(unittest.TestCase):
    def test_native_candidates_external_results_and_frame_retirement(self):
        with tempfile.TemporaryDirectory(prefix="lutcam-external-lftag-") as build_dir:
            sources = [
                line.strip()
                for line in (ROOT / "rtl/tagdetect_lwc/sources.f").read_text().splitlines()
                if line.strip() and not line.lstrip().startswith("#")
            ]
            sources += [
                "rtl/isp/MonoIsp.sv", "rtl/lutcam/CameraPipeline.sv",
                "tests/lutcam/camera_external_lftag_tb.sv",
            ]
            result = subprocess.run([
                "verilator", "--binary", "--timing", "--assert", "-j", "2",
                "-Wno-fatal", "--top-module", "camera_external_lftag_tb",
                "--Mdir", build_dir, f"-I{ROOT / 'rtl/lftag'}",
                *(str(ROOT / source) for source in sources),
            ], cwd=ROOT, capture_output=True, text=True, timeout=180)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run(
                [str(Path(build_dir) / "Vcamera_external_lftag_tb")], cwd=ROOT,
                capture_output=True, text=True, timeout=120,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS external LFTag frontend", result.stdout)
            print(result.stdout, end="")


if __name__ == "__main__":
    unittest.main()
