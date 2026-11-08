from pathlib import Path
import tempfile
import unittest
from isp_sim import compile_isp, profiles, run_isp


class IspSnapshotTests(unittest.TestCase):
    def test_streaming_snapshot_is_frame_local_and_does_not_block_pixels(self):
        with tempfile.TemporaryDirectory(prefix="lutcam-isp-snapshot-") as directory:
            for profile in profiles():
                with self.subTest(profile=profile[0]):
                    binary, hashes = compile_isp(directory, "isp_snapshot_tb",
                        Path(__file__).with_name("isp_snapshot_tb.sv"), profile)
                    self.assertIn("PASS streaming histogram", run_isp(binary, hashes))


if __name__ == "__main__": unittest.main()
