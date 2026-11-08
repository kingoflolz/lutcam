"""Exercise portable/native MonoIsp at an uninterrupted two-pixel cadence."""

from pathlib import Path
import tempfile
import unittest

from isp_sim import compile_isp, profiles, run_isp


class IspNonblockingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build_dir = tempfile.TemporaryDirectory(prefix="lutcam-isp-nonblocking-")
        cls.addClassCleanup(cls.build_dir.cleanup)
        cls.binaries = []
        for profile in profiles():
            binary, hashes = compile_isp(
                cls.build_dir.name, "isp_nonblocking_tb",
                Path(__file__).with_name("isp_nonblocking_tb.sv"), profile,
            )
            cls.binaries.append((profile[0], binary, hashes))

    def run_case(self, case):
        for name, binary, hashes in self.binaries:
            with self.subTest(profile=name):
                output = run_isp(binary, hashes, f"+CASE={case}")
                self.assertIn(f"PASS MonoIsp nonblocking case {case}", output)

    def test_512_consecutive_defects_have_no_active_stalls(self):
        self.run_case(0)

    def test_dense_start_then_long_gap_and_dense_tail_have_no_active_stalls(self):
        self.run_case(1)

    def test_full_encoded_table_with_two_far_dense_clusters_has_no_active_stalls(self):
        self.run_case(2)

    def test_far_final_defect_has_no_active_stalls(self):
        self.run_case(3)

    def test_empty_table_has_no_active_stalls(self):
        self.run_case(4)

    def test_noncanonical_continuation_at_current_address_waits_for_mask(self):
        self.run_case(5)


if __name__ == "__main__":
    unittest.main()
