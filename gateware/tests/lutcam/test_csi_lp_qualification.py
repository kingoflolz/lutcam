"""Check the receiver's narrow ACTIVE-LP10 diagnostic qualification.

The DUT is the production receiver. A second copy has only that diagnostic
guard removed, reproducing the original single-sample LP10 behavior while
keeping every other input, parameter, and control path the same. Its unused
history flops are harmless. No archived receiver sources are needed.

The bench includes 42 directed/phase-sweep cases covering transient versus
persistent LP10, independent lane history, asynchronous reset, clock-session
restart, SETTLE-to-ACTIVE boundaries, status clearing during a persistent
fault, unchanged entry/exit timing, and clock-lane diagnostics. Packet/pixel
coverage remains in ``cargo test -p csi2-rx-rtl-tests``.

Run with ``python -m unittest discover -s tests/lutcam -p test_csi_lp_qualification.py``.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
RECEIVER = ROOT / "rtl/csi2_rx/csi2_rx.sv"


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class CsiLpQualificationTests(unittest.TestCase):
    def test_transient_diagnosis_and_reset_session_clear_boundaries(self):
        source = RECEIVER.read_text()
        declaration = "module csi2_rx "
        qualified = "data_lane_lp[lane] == 2'b10 && active_lp10_previous[lane]"
        self.assertEqual(source.count(declaration), 1)
        self.assertEqual(source.count(qualified), 1,
                         "Update the raw diagnostic-model transform if its expression changes")
        raw = source.replace(qualified, "data_lane_lp[lane] == 2'b10")
        raw = raw.replace(declaration, "module csi2_rx_raw ")
        narrow = source.replace(declaration, "module csi2_rx_narrow ")

        with tempfile.TemporaryDirectory(prefix="lutcam-csi-lp-") as temporary:
            build = Path(temporary)
            (build / "receiver_raw.sv").write_text(raw)
            (build / "receiver_narrow.sv").write_text(narrow)
            command = [
                "verilator", "--binary", "--timing", "-Wno-fatal", "-j", "2",
                "--top-module", "csi_lp_qualification_tb", "--Mdir", str(build / "obj"),
                str(build / "receiver_raw.sv"), str(build / "receiver_narrow.sv"),
                str(ROOT / "rtl/csi2_rx/Ov9281LaneAligner.sv"),
                str(ROOT / "rtl/csi2_rx/Ov9281PixelAsyncFifo.sv"),
                str(ROOT / "rtl/csi2_rx/Ov9281Counter.sv"),
                str(ROOT / "tests/lutcam/csi_lp_qualification_tb.sv"),
            ]
            compiled = subprocess.run(command, cwd=ROOT, capture_output=True,
                                      text=True, timeout=120)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            result = subprocess.run([str(build / "obj/Vcsi_lp_qualification_tb")],
                                    cwd=build, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS narrow LP qualification cases=42 ", result.stdout)
            self.assertIn("control timing identical", result.stdout)


if __name__ == "__main__":
    unittest.main()
