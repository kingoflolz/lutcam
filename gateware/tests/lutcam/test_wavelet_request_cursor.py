"""Check a verbatim current-source V2 control slice against a counter oracle.

The complete wavelet regressions qualify arithmetic and integration. This
focused test extracts control statements from current production RTL, avoiding
archived RTL copies and force-induced multiple-driver warnings.
"""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

from isp_sim import assert_unchanged, profiles


ROOT = Path(__file__).resolve().parents[2]


def extract_control_slice(source):
    """Preserve statements verbatim; fail on missing/ambiguous control sites."""
    def between(start, end):
        if source.count(start) != 1 or source.count(end) != 1:
            raise AssertionError(f"Wavelet control extraction anchor changed: {start!r}")
        return source.split(start, 1)[1].split(end, 1)[0]

    declarations = "    logic v2_high_requests_done;" + between(
        "    logic v2_high_requests_done;", "    logic [V1_ROW_WIDTH-1:0] v2_high_buffer_row;")
    if "wire [$clog2(H2_WIDTH)-1:0] v2_high_request_index =" not in declarations:
        raise AssertionError("Expected the current derived request-index declaration")
    body_start = "            if (v2_high_read_request) begin"
    body = body_start + between(body_start, "            case ({h2_out_valid, v2_high_consume})")
    sequential = source[source.index("    always_ff @(posedge clk or negedge rst_n) begin\n"
                                     "        if (!rst_n) begin\n"
                                     "            frame_active <= 1'b0;"):]
    reset = sequential.split("        end else begin", 1)[0]
    clear = sequential.split("            if (frame_clear) begin", 1)[1].split(
        "            end else if (in_valid) begin", 1)[0]
    state_names = ("v2_high_request_started", "v2_high_requests_done",
                   "v2_high_read_valid_q", "v2_high_read_column_q")

    def state_assignments(section):
        return "\n".join(line for line in section.splitlines()
                         if any(re.match(r"\s*" + name + r"\s*<=", line)
                                for name in state_names)) + "\n"

    reset = state_assignments(reset)
    clear = state_assignments(clear)
    # Any additional writer must be brought into this model explicitly.
    for name in state_names:
        pattern = r"\b" + name + r"\s*<="
        if len(re.findall(pattern, source)) != len(re.findall(pattern, reset + clear + body)):
            raise AssertionError(f"Unmodeled wavelet state assignment: {name}")
    guard = between("        v2_high_read_request = ", "    vertical_lift_scheduler #(")
    if "&& !v2_high_requests_done" not in guard:
        raise AssertionError("Read-request exclusion after requests_done changed")
    address_start = "        regular_local_row_read_enable = v2_high_read_request;"
    address = address_start + between(address_start,
                                     "        if (!v2_high_read_request && v3_drain_phase")
    return (
        "module wavelet_request_cursor_dut(input wire clk, rst_n, frame_clear, "
        "inject_request, inject_consume);\n"
        "localparam integer H2_WIDTH = 320;\n"
        "localparam integer LOCAL_ROW_ADDRESS_WIDTH = 9;\n"
        + declarations +
        "logic v2_high_buffer_ready;\nlogic v2_high_consume;\n"
        "logic regular_local_row_read_enable, local_row_read_enable;\n"
        "logic [LOCAL_ROW_ADDRESS_WIDTH-1:0] local_row_read_address;\n"
        "assign v2_high_read_request = inject_request;\n"
        "assign v2_high_consume = inject_consume;\n"
        "always_comb begin\n" + address + "end\n"
        "always_ff @(posedge clk or negedge rst_n) begin\n"
        "if (!rst_n) begin\n" + reset + "end else begin\n"
        "if (frame_clear) begin\n" + clear + "end\n"
        + body + "end\nend\nendmodule\n"
    )


class WaveletRequestCursorTests(unittest.TestCase):
    def test_current_rtl_slice_request_clear_consume(self):
        for name, model in profiles():
            with self.subTest(profile=name):
                with tempfile.TemporaryDirectory(prefix="lutcam-wavelet-cursor-") as directory:
                    build = Path(directory)
                    source = ROOT / "rtl/lwc/forward_wavelet_transform.sv"
                    extracted = build / "wavelet_request_cursor_dut.sv"
                    extracted.write_text(extract_control_slice(source.read_text()))
                    bench = Path(__file__).with_name("wavelet_request_cursor_tb.sv")
                    inputs = [source, extracted, bench, Path(__file__),
                              Path(__file__).with_name("isp_sim.py")]
                    command = ["verilator", "--binary", "--timing", "--assert", "-j", "2",
                               "-Wno-TIMESCALEMOD", "--top-module", "wavelet_request_cursor_tb",
                               "--Mdir", str(build)]
                    if model:
                        # The maintained waiver file names only the external
                        # vendor model. Production and bench warnings are fatal.
                        lint = Path(__file__).with_name("efinity_dsp_model.vlt")
                        command += ["-DSYNTHESIS", str(lint), str(model)]
                        inputs += [lint, model]
                    command += [str(extracted), str(bench)]
                    hashes = {path: hashlib.sha256(path.read_bytes()).hexdigest()
                              for path in inputs}
                    result = subprocess.run(command, cwd=ROOT, capture_output=True,
                                            text=True, timeout=120)
                    (build / "compile.log").write_text(result.stdout + result.stderr)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    assert_unchanged(hashes)
                    manifest = {"profile": name, "scope": "current-source control slice; no arithmetic",
                                "command": command,
                                "source_sha256": {str(path): digest
                                                  for path, digest in hashes.items()}}
                    print(json.dumps(manifest, sort_keys=True))
                    executable = build / "Vwavelet_request_cursor_tb"
                    result = subprocess.run([str(executable)], cwd=build,
                                            capture_output=True, text=True, timeout=60)
                    assert_unchanged(hashes)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertIn("PASS wavelet request cursor", result.stdout)
                    self.assertIn("clear_request=", result.stdout)
                    self.assertIn("terminal_request=", result.stdout)
                    print(result.stdout.strip())


if __name__ == "__main__":
    unittest.main()
