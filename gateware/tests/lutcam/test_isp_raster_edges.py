"""Independent public-port CSR, raster-wrap, histogram and dirty-reset checks."""

from pathlib import Path
import tempfile
import unittest

from isp_sim import compile_isp, profiles, run_isp


class IspRasterEdgeTests(unittest.TestCase):
    def test_public_raster_edges_and_restart(self):
        for profile in profiles():
            with self.subTest(profile=profile[0]):
                with tempfile.TemporaryDirectory(prefix="lutcam-isp-raster-") as build:
                    binary, hashes = compile_isp(
                        build, "isp_raster_edges_tb",
                        Path(__file__).with_name("isp_raster_edges_tb.sv"), profile,
                    )
                    output = run_isp(binary, hashes)
                    for width, height in ((2, 1), (14, 3), (16, 4), (1280, 800)):
                        self.assertIn(f"PASS ISP raster edges {width}x{height}:", output)
                    self.assertIn("PASS all ISP raster edge cases", output)


if __name__ == "__main__":
    unittest.main()
