"""The maintenance write policy is explicit build metadata, never a runtime unlock."""

import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

import lutcam_soc


class GoldenMaintenanceBuildTests(unittest.TestCase):
    def test_build_flag_is_bound_to_soc_and_packaging_metadata(self):
        for maintenance in (False, True):
            with self.subTest(maintenance=maintenance), tempfile.TemporaryDirectory() as output:
                arguments = ["lutcam_soc.py", "--output-dir", output,
                             "--build-id", "12" * 16]
                if maintenance:
                    arguments.append("--golden-maintenance")
                with patch.object(sys, "argv", arguments), \
                        patch.object(lutcam_soc, "LutcamSoC") as soc, \
                        patch.object(lutcam_soc, "Builder") as builder:
                    lutcam_soc.main()
                self.assertEqual(soc.call_args.kwargs["golden_maintenance"], maintenance)
                self.assertFalse(soc.call_args.kwargs["golden"])
                metadata = json.loads((Path(output) / "ota-build.json").read_text())
                self.assertEqual(metadata["golden_maintenance"], maintenance)
                self.assertFalse(metadata["golden"])
                self.assertEqual(metadata["build_id"], "12" * 16)
                self.assertFalse(builder.return_value.build.call_args.kwargs["run"])

    def test_conflicting_boot_scanner_mode_is_rejected_before_build(self):
        arguments = ["lutcam_soc.py", "--golden", "--golden-maintenance"]
        with patch.object(sys, "argv", arguments), \
                patch.object(lutcam_soc, "LutcamSoC") as soc, \
                contextlib.redirect_stderr(io.StringIO()), \
                self.assertRaises(SystemExit) as error:
            lutcam_soc.main()
        self.assertEqual(error.exception.code, 2)
        soc.assert_not_called()
