"""Application builds always use the canonical adaptive threshold frontend."""

import json
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))
import lutcam_soc


@pytest.mark.parametrize("golden", [False, True])
def test_build_metadata_identifies_application_frontend(golden):
    with tempfile.TemporaryDirectory() as output:
        arguments = ["lutcam_soc.py", "--output-dir", output]
        if golden:
            arguments.append("--golden")
        with patch.object(sys, "argv", arguments), \
                patch.object(lutcam_soc, "LutcamSoC") as soc, \
                patch.object(lutcam_soc, "Builder"):
            lutcam_soc.main()
        metadata = json.loads((Path(output) / "ota-build.json").read_text())
        assert metadata["threshold_frontend"] == (None if golden else "adaptive")
        assert metadata["build_id"] == soc.call_args.kwargs["build_id"]
        assert soc.call_args.kwargs["golden"] is golden
