"""Check portable and native ISP defect-address boundaries through public ports."""

from pathlib import Path
import tempfile
import unittest

from isp_sim import compile_isp, profiles, run_isp


class IspAddressBoundaryTests(unittest.TestCase):
    def test_cumulative_deltas_and_frame_boundaries(self):
        for profile in profiles():
            with self.subTest(profile=profile[0]):
                with tempfile.TemporaryDirectory(prefix="lutcam-isp-boundaries-") as build:
                    binary, hashes = compile_isp(
                        build, "isp_address_boundary_tb",
                        Path(__file__).with_name("isp_address_boundary_tb.sv"), profile,
                    )
                    output = run_isp(binary, hashes)
                    for pairs in (1, 4, 21, 262144, 512000, 67108864, 134217216, 134217727):
                        self.assertIn(f"PASS ISP address boundaries frame_pairs={pairs} ", output)
                    self.assertIn("PASS all ISP public address boundary cases", output)


if __name__ == "__main__":
    unittest.main()
