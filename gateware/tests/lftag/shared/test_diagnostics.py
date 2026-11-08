"""Retained first-error FIFO diagnostics, including shared-engine context."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class SharedDiagnosticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix="lutcam-lftag-diagnostics-")
        cls.addClassCleanup(cls.build.cleanup)
        sources = (ROOT / "rtl/lftag/sources.f").read_text().splitlines()
        sources.append("tests/lftag/shared/diagnostics_tb.sv")
        result = subprocess.run(
            ["verilator", "--binary", "--timing", "--assert", "-Wno-fatal",
             "-j", "2", "--top-module", "diagnostics_tb", "--Mdir", cls.build.name,
             f"-I{ROOT / 'rtl/lftag'}",
             *(str(ROOT / source) for source in sources if source.strip())],
            cwd=ROOT, capture_output=True, text=True, timeout=240,
        )
        if result.returncode:
            raise RuntimeError(f"Verilator compilation failed:\n{result.stdout}\n{result.stderr}")
        cls.binary = Path(cls.build.name) / "Vdiagnostics_tb"

    def run_case(self, case):
        result = subprocess.run(
            [str(self.binary), f"+CASE={case}"], cwd=ROOT,
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(f"PASS retained diagnostics case {case}", result.stdout)

    def test_protocol_first_context_survives_processing_reset_and_clears_explicitly(self):
        self.run_case(0)

    def test_protocol_causes_and_first_offending_tuple_are_distinct(self):
        self.run_case(1)

    def test_full_queue_admission_snapshot_survives_drain_and_reset(self):
        self.run_case(2)

    def test_clear_and_new_error_have_defined_precedence(self):
        self.run_case(3)

    def test_shared_output_stall_context_survives_camera_reset(self):
        self.run_case(4)

    def test_simultaneous_pop_at_full_admits_without_false_drop(self):
        self.run_case(5)


if __name__ == "__main__":
    unittest.main()
