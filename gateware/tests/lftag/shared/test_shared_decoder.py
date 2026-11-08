"""Shared decoder integration: independent nine-point oracle and ownership."""
from pathlib import Path
import sys
import unittest
import shutil
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"refine9"))
from test_shared import test_shared as run_refined_shared

@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class SharedDecoderTests(unittest.TestCase):
    def test_nine_point_fit_pipeline_banks_resets_backpressure_and_full_queues(self):
        run_refined_shared()

if __name__ == "__main__":unittest.main()
