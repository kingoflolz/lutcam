"""Verify bounded production output and check the frame-only budget's burst counterexample.

Fixed one-bit-per-pixel frames do not constrain instantaneous output rate.
Legal binary pixels produce an early semantic burst requiring more than
128,000 bytes of aggregate buffering for synchronized cameras, even with an
ideal link. This regression documents the capacity gap; passing does not
claim that the integrated packet FIFOs can absorb it.

The golden SHA-256 digests were generated with the current Rust scalar codec,
using the identical xorshift sequence or an all-128 image. Compare the whole
128,000-byte RTL frame so the burst cannot be attributed to malformed output.
Pure zero-padding words are excluded from the bandwidth calculation. Production
mode must additionally obey a 5696-byte + 2-bit/core-clock semantic envelope.

Run: python -m unittest discover -s tests/lutcam -p test_lwc_buffer_bound.py
"""

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
class LwcBufferBoundTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build_dir = tempfile.TemporaryDirectory(prefix="lutcam-lwc-burst-")
        cls.addClassCleanup(cls.build_dir.cleanup)
        build_path = Path(cls.build_dir.name)
        cls.binary = build_path / "obj" / "Vlwc_burst_tb"
        command = [
            "verilator", "--cc", "--exe", "--build", "--assert", "-j", "2",
            "-Wno-fatal", "--top-module", "lwc_burst_tb",
            "--Mdir", str(cls.binary.parent),
            "-f", "rtl/lwc/codec_encoder_sources.f",
            "tests/lutcam/lwc_burst_tb.sv",
            str(ROOT / "tests/lutcam/lwc_burst_probe.cpp"),
        ]
        result = subprocess.run(
            command, cwd=ROOT, capture_output=True, text=True, timeout=180,
        )
        if result.returncode:
            raise RuntimeError(f"Verilator compilation failed:\n{result.stdout}\n{result.stderr}")
        cls.bounded_binary = build_path / "bounded_obj" / "Vlwc_burst_tb"
        command[command.index(str(cls.binary.parent))] = str(cls.bounded_binary.parent)
        command.append("-GBURST_LIMIT_ENABLE=1")
        result = subprocess.run(
            command, cwd=ROOT, capture_output=True, text=True, timeout=180,
        )
        if result.returncode:
            raise RuntimeError(f"Bounded compilation failed:\n{result.stdout}\n{result.stderr}")

    def run_case(self, case, expected_digest, *, bounded=False):
        output_path = Path(self.build_dir.name) / f"{case}-{bounded}.bin"
        result = subprocess.run(
            [str(self.bounded_binary if bounded else self.binary), case, str(output_path)], cwd=ROOT,
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        stats = json.loads(result.stdout)
        data = output_path.read_bytes()
        self.assertEqual(len(data), 128000)
        self.assertEqual(hashlib.sha256(data).hexdigest(), expected_digest)
        self.assertEqual(stats["words"], 64000)
        self.assertEqual(stats["input_pairs"], 512000)
        self.assertEqual(stats["sof_count"], 1)
        self.assertGreater(stats["packets_done_cycle"], 0)
        self.assertLess(stats["packets_done_cycle"], 1000000)
        if bounded:
            self.assertLessEqual(stats["peak_semantic_queue_bits"], 5696 * 8)
        return stats

    def test_legal_binary_pixels_exceed_small_packet_fifo_capacity(self):
        stats = self.run_case(
            "binary", "603927f8250f3f94dc459dc9969252eec127350e89d52dbc821b8e4330eb227b",
        )
        self.assertEqual(stats["packet_bits"], 1023946)
        self.assertEqual(stats["semantic_words"], 63997)
        # Credit the link for every clock since frame start, even before the
        # first output. This still leaves over 128 kB of unavoidable backlog.
        self.assertGreater(stats["peak_cumulative24"], 128000 * 24)
        self.assertGreaterEqual(stats["peak_queue24"], stats["peak_cumulative24"])
        self.assertLess(stats["peak_cycle"], 120000)  # Within the first millisecond.
        print(
            "LWC binary-image burst: "
            f"minimum aggregate backlog {stats['peak_cumulative24'] / 24:.3f} bytes; "
            f"ideal queue peak {stats['peak_queue24'] / 24:.3f} bytes "
            f"at cycle {stats['peak_cycle']}.",
        )

    def test_flat_image_separates_semantic_zero_packets_from_padding(self):
        stats = self.run_case(
            "flat", "a80a95dfd4b1a42ba717acf3f441dc08962c411f7f73f659743788e8a8fd096b",
        )
        # Once negative rate debt reaches cutoff zero, the encoder selects
        # fine zero packets; these remain semantic data, not removable padding.
        self.assertEqual(stats["packet_bits"], 60010)
        self.assertEqual(stats["semantic_words"], 3751)
        self.assertEqual(stats["words"] - stats["semantic_words"], 60249)

    def test_clock_bounded_quality_matches_scalar_and_all_interval_rate_limit(self):
        cases = {
            "binary": "f055fb3c1641aa9a9b59e13f7e7fca08ee699e83d04f9cdf257bdae61521b930",
            "flat": "a80a95dfd4b1a42ba717acf3f441dc08962c411f7f73f659743788e8a8fd096b",
            "late": "808d26f9d7d4207d9211c999494e8798114f770481ced389741a22986f8c7c15",
        }
        for case, digest in cases.items():
            with self.subTest(case=case):
                stats = self.run_case(case, digest, bounded=True)
                self.assertGreater(stats["peak_semantic_queue_bits"], 0)

    def test_flat_then_binary_then_late_detail_is_bounded_across_frames(self):
        output_path = Path(self.build_dir.name) / "three-frames.bin"
        trace_path = Path(self.build_dir.name) / "three-frames.csv"
        result = subprocess.run(
            [str(self.bounded_binary), "mixed", str(output_path), str(trace_path), "3"],
            cwd=ROOT, capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        stats = json.loads(result.stdout)
        self.assertEqual(stats["words"], 192000)
        self.assertEqual(stats["input_pairs"], 1536000)
        self.assertEqual(stats["sof_count"], 3)
        self.assertLessEqual(stats["peak_semantic_queue_bits"], 5696 * 8)
        data = output_path.read_bytes()
        self.assertEqual(len(data), 384000)
        expected = [
            "a80a95dfd4b1a42ba717acf3f441dc08962c411f7f73f659743788e8a8fd096b",
            "f055fb3c1641aa9a9b59e13f7e7fca08ee699e83d04f9cdf257bdae61521b930",
            "808d26f9d7d4207d9211c999494e8798114f770481ced389741a22986f8c7c15",
        ]
        for frame, digest in enumerate(expected):
            self.assertEqual(hashlib.sha256(data[frame * 128000:(frame + 1) * 128000]).hexdigest(), digest)


if __name__ == "__main__":
    unittest.main()
