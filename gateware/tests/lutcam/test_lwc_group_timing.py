"""Verify scalar-reference decision timestamps against production RTL.

Input edge zero is the first valid pixel pair. Each line has 640 valid pair
edges followed by 256 blank clocks. Both highly detailed and constant inputs
must produce the same 64,000 decision edges with output ready continuously.
This table defines only the default reference input cadence; hardware uses
actual elapsed clocks, including gaps, to replenish its transport bucket.

Run: python -m unittest discover -s tests/lutcam -p test_lwc_group_timing.py
"""

import base64
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
RESOURCE = ROOT.parent / "software/crates/lwc/src/canonical_group_cycle_deltas"


class LwcGroupTimingTests(unittest.TestCase):
    def test_embedded_timing_metadata(self):
        raw = gzip.decompress(base64.b64decode(RESOURCE.with_suffix(".b64").read_text()))
        metadata = json.loads(RESOURCE.with_suffix(".json").read_text())
        self.assertEqual(hashlib.sha256(raw).hexdigest(), metadata["uncompressed_sha256"])
        self.assertEqual(len(raw), metadata["uncompressed_bytes"])
        deltas = struct.unpack("<64000I", raw)
        self.assertEqual(deltas[0], metadata["first_commit_cycle"])
        self.assertEqual(sum(deltas), metadata["last_commit_cycle"])
        self.assertEqual(min(deltas[1:]), 11)
        self.assertEqual(max(deltas[1:]), metadata["maximum_subsequent_delta"])

    @unittest.skipUnless(shutil.which("verilator"), "Verilator is required for RTL tests")
    def test_actual_binary_and_flat_decision_edges_match_reference(self):
        expected = gzip.decompress(base64.b64decode(RESOURCE.with_suffix(".b64").read_text()))
        with tempfile.TemporaryDirectory(prefix="lutcam-lwc-group-timing-") as directory:
            build = Path(directory)
            binary = build / "obj" / "Vlwc_group_timing_tb"
            result = subprocess.run(
                ["verilator", "--cc", "--exe", "--build", "--assert", "-j", "2",
                 "-Wno-fatal", "--top-module", "lwc_group_timing_tb",
                 "--Mdir", str(binary.parent), "-f", "rtl/lwc/codec_encoder_sources.f",
                 "tests/lutcam/lwc_group_timing_tb.sv",
                 str(ROOT / "tests/lutcam/lwc_group_timing_probe.cpp")],
                cwd=ROOT, capture_output=True, text=True, timeout=180,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            output = build / "deltas.bin"
            result = subprocess.run(
                [str(binary), str(output)], cwd=ROOT,
                capture_output=True, text=True, timeout=45,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(output.read_bytes(), expected)


if __name__ == "__main__":
    unittest.main()
