"""Three 120 fps frames through the complete production camera RTL closure.

Unlike test_camera_control, this test uses the real TagDetectLwc, including
the compressor, detection hierarchy, and pose solver. It checks production
pipeline drain timing and framing, not compressed-image reconstruction or
pose accuracy (covered by the component reference-model suites).
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class CameraFullChainTests(unittest.TestCase):
    def test_three_full_frames_at_120_fps_with_real_processing_pipeline(self):
        with tempfile.TemporaryDirectory(prefix="lutcam-fullchain-") as build_dir:
            closure = ROOT / "rtl/tagdetect_lwc/sources.f"
            sources = [
                line.strip() for line in closure.read_text().splitlines()
                if line.strip() and not line.lstrip().startswith("#")
            ]
            sources += [
                "rtl/isp/MonoIsp.sv",
                "rtl/lutcam/CameraPipeline.sv",
                "tests/lutcam/camera_fullchain_tb.sv",
            ]
            command = [
                "verilator", "--binary", "--timing", "--assert", "-j", "2",
                "-Wno-fatal", "--top-module", "camera_fullchain_tb",
                "--Mdir", build_dir, f"-I{ROOT / 'rtl/lftag'}",
                *(str(ROOT / source) for source in sources),
            ]
            result = subprocess.run(
                command, cwd=ROOT, capture_output=True, text=True, timeout=180,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            # Exercise history warmup, changed previous-frame thresholds, and
            # repeated nonempty hierarchy/solver use across retirement resets.
            for args in ([], ["+ALL_FIXTURE=1"]):
                with self.subTest(args=args):
                    result = subprocess.run(
                        [str(Path(build_dir) / "Vcamera_fullchain_tb"), *args], cwd=ROOT,
                        capture_output=True, text=True, timeout=180,
                    )
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertIn("PASS camera fullchain: three 120 fps frames", result.stdout)
                    print(result.stdout, end="")


if __name__ == "__main__":
    unittest.main()
