"""Exhaustive binary check of the actual remaining-index output network.

The DUT is extracted unchanged from LfTagCornerSelect9.sv, including the
function and sorting declarations when present. No DUT expression is duplicated
in this test. Icarus simulates all 64 exclusion pairs and all five output slots.
The Python references are the sorted two-skip contract and, for distinct
indices below seven, an independently constructed ascending remaining list.

LUTCAM_CORNER_SOURCE selects a source override; production is the
default. LUTCAM_CORNER_REMAINING_BUILD retains generated sources and reports.
LUTCAM_CORNER_EXPECTED_SHA256 optionally pins the source used by qualification.
LUTCAM_REQUIRE_EFINIX_NATIVE=1 makes absent simulation dependencies fail, even
though this combinational fabric network needs no Efinity primitive model.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock


ROOT = next(p for p in Path(__file__).resolve().parents
            if (p / "rtl/lftag/LfTagCornerSelect9.sv").is_file())
HERE = Path(__file__).resolve().parent
SOURCE = Path(os.environ.get("LUTCAM_CORNER_SOURCE",
                             ROOT / "rtl/lftag/LfTagCornerSelect9.sv"))


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require_dependencies():
    for executable in ("iverilog", "vvp"):
        if shutil.which(executable) is None:
            message = f"{executable} executable is missing"
            if os.environ.get("LUTCAM_REQUIRE_EFINIX_NATIVE") == "1":
                raise AssertionError("Required " + message)
            raise unittest.SkipTest(message)


def exactly_one(pattern, text, label):
    matches = list(re.finditer(pattern, text, re.MULTILINE | re.DOTALL))
    assert len(matches) == 1, f"Expected one {label}; found {len(matches)}"
    return matches[0]


def extract_network(source):
    """Copy source text, failing closed if its documented boundary changes."""
    module = exactly_one(r"^module LfTagCornerSelect9\b.*?^endmodule\b",
                         source, "corner module")
    body = module.group()
    exactly_one(r"^\s*logic \[2:0\] bottom_left;\s*$", body,
                "three-bit bottom_left declaration")
    exactly_one(r"^\s*logic \[2:0\] bottom_right;\s*$", body,
                "three-bit bottom_right declaration")
    exactly_one(r"^\s*output logic \[19:0\]\s+out_remaining_index_flat,\s*$",
                body, "twenty-bit output declaration")
    region = exactly_one(
        r"(?<=    assign out_cycles = cycle_counter;\n).*?"
        r"(?=    always_ff @\(posedge clk\) begin)", body,
        "remaining-index combinational region")
    network = region.group()
    assert len(re.findall(r"\bassign\s+out_remaining_index_flat\b", network)) == 1
    assert len(re.findall(r"\bassign\s+out_remaining_index_flat\b", body)) == 1
    exactly_one(r"for \(genvar slot = 0; slot < 5; slot = slot \+ 1\) "
                r"begin : g_remaining_order", network, "five-slot generate loop")
    assert re.search(r"\b(always|always_comb|always_ff|always_latch)\b", network) is None

    pieces = []
    if re.search(r"\bremaining_index\s*\(", network):
        for name in ("excluded_index_low", "excluded_index_high"):
            declaration = exactly_one(r"^    logic \[3:0\] " + name + r";\n",
                                      body, name + " declaration")
            pieces.append(declaration)
        function = exactly_one(
            r"^    function automatic logic \[3:0\] remaining_index\(.*?"
            r"^    endfunction\n", body, "remaining-index function")
        pieces.append(function)
    pieces.append(region)
    extracted = "".join(part.group() for part in pieces)
    # These lines are only a port shell. Every declaration, helper and expression
    # implementing the output network above is copied verbatim from the source.
    generated = (
        "`timescale 1ns/1ps\n`default_nettype none\n"
        "module CornerRemainingSubject(\n"
        "    input wire [2:0] bottom_left, bottom_right,\n"
        "    output wire [19:0] out_remaining_index_flat\n);\n"
        + extracted + "endmodule\n`default_nettype wire\n"
    )
    spans = []
    for part in pieces:
        start = module.start() + part.start()
        end = module.start() + part.end()
        assert source[start:end] == part.group()
        spans.append({"first_line": source.count("\n", 0, start) + 1,
                      "last_line": source.count("\n", 0, end - 1) + 1,
                      "sha256": hashlib.sha256(part.group().encode()).hexdigest()})
    return generated, spans


def sorted_reference(a, b, slot):
    low, high = sorted((a, b))
    result = slot
    if result >= low:
        result += 1
    if result >= high:
        result += 1
    return result


class CornerRemainingTests(unittest.TestCase):
    def test_actual_rtl_all_binary_exclusions_and_slots(self):
        require_dependencies()
        self.assertTrue(SOURCE.is_file(), str(SOURCE))
        expected_hash = os.environ.get("LUTCAM_CORNER_EXPECTED_SHA256")
        if expected_hash is not None:
            self.assertEqual(sha256(SOURCE), expected_hash,
                             "Source differs from the qualification input")
        generated, spans = extract_network(SOURCE.read_text())
        with tempfile.TemporaryDirectory(prefix="corner-remaining-") as temporary:
            build = Path(os.environ.get("LUTCAM_CORNER_REMAINING_BUILD",
                                        temporary)).resolve()
            build.mkdir(parents=True, exist_ok=True)
            (build / "report.json").unlink(missing_ok=True)
            frozen = build / "LfTagCornerSelect9.sv"
            frozen.write_bytes(SOURCE.read_bytes())
            subject = build / "CornerRemainingSubject.sv"
            subject.write_text(generated)
            bench = build / "corner_remaining_tb.sv"
            bench.write_bytes((HERE / "corner_remaining_tb.sv").read_bytes())
            executable = build / "corner_remaining.vvp"
            command = ["iverilog", "-g2012", "-Wall", "-s", "corner_remaining_tb",
                       "-o", str(executable), str(subject), str(bench)]
            compiled = subprocess.run(command, capture_output=True, text=True,
                                      timeout=60)
            (build / "compile.log").write_text(compiled.stdout + compiled.stderr)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            self.assertNotRegex(compiled.stderr, r"(?i)warning", compiled.stderr)
            result = subprocess.run(["vvp", str(executable)], capture_output=True,
                                    text=True, timeout=60)
            (build / "run.log").write_text(result.stdout + result.stderr)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            rows = [tuple(map(int, match)) for match in re.findall(
                r"^ROW (\d+) (\d+) (\d+) (\d+)$", result.stdout, re.MULTILINE)]
            domain = [(a, b, slot) for a in range(8) for b in range(8)
                      for slot in range(5)]
            self.assertEqual([row[:3] for row in rows], domain)
            valid_rows = []
            for a, b, slot, actual in rows:
                self.assertEqual(actual, sorted_reference(a, b, slot),
                                 f"Reference mismatch at exclusions={a,b}, slot={slot}")
                if a < 7 and b < 7 and a != b:
                    remaining = [index for index in range(7) if index not in (a, b)]
                    self.assertEqual(actual, remaining[slot],
                                     f"List mismatch at exclusions={a,b}, slot={slot}")
                    valid_rows.append((a, b, slot, actual))
            self.assertEqual(len(rows), 320)
            self.assertEqual(len(valid_rows), 210)
            self.assertIn("PASS pairs=64 slots=320 clock_edges=0", result.stdout)
            hashed = [SOURCE, HERE / "test_corner_remaining.py",
                      HERE / "corner_remaining_tb.sv", frozen, subject, bench]
            report = {
                "status": "PASS",
                "scope": "Actual source-extracted combinational output network, "
                         "compiled without a vendor model; no DUT arithmetic "
                         "reimplementation, internal state forcing or clock edges.",
                "source_path": str(SOURCE.resolve()),
                "source_sha256": sha256(SOURCE),
                "source_spans": spans,
                "command": command,
                "simulator_version": subprocess.run(
                    ["iverilog", "-V"], capture_output=True, text=True,
                    timeout=10).stdout.splitlines()[0],
                "sha256": {str(path.resolve()): sha256(path) for path in hashed},
                "coverage": {
                    "binary_input_pairs": 64, "sorted_reference_checks": len(rows),
                    "independent_ascending_list_checks": len(valid_rows),
                    "equal_exclusion_checks": sum(a == b for a, b, *_ in rows),
                    "index7_exclusion_checks": sum(a == 7 or b == 7
                                                   for a, b, *_ in rows),
                    "clock_edges": 0,
                    "rows_sha256": hashlib.sha256(json.dumps(
                        rows, separators=(",", ":")).encode()).hexdigest(),
                },
                "limits": "Exhaustive binary combinational verification only; "
                          "no claim about arbitrary X propagation, pipeline "
                          "handshakes, reset behavior or physical timing.",
            }
            (build / "report.json").write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps(report, sort_keys=True))


class DependencyGuardTests(unittest.TestCase):
    def test_required_missing_simulator_fails(self):
        for missing in ("iverilog", "vvp"):
            with self.subTest(missing=missing), \
                 mock.patch.dict(os.environ, {"LUTCAM_REQUIRE_EFINIX_NATIVE": "1"}), \
                 mock.patch.object(shutil, "which", side_effect=lambda name:
                                   None if name == missing else "/tool/" + name):
                with self.assertRaisesRegex(AssertionError, "Required " + missing):
                    require_dependencies()

    def test_optional_missing_simulator_skips(self):
        for missing in ("iverilog", "vvp"):
            with self.subTest(missing=missing), \
                 mock.patch.dict(os.environ, {"LUTCAM_REQUIRE_EFINIX_NATIVE": "0"}), \
                 mock.patch.object(shutil, "which", side_effect=lambda name:
                                   None if name == missing else "/tool/" + name):
                with self.assertRaisesRegex(unittest.SkipTest, missing):
                    require_dependencies()

    def test_present_simulator_needs_no_vendor_model(self):
        with mock.patch.dict(os.environ, {"LUTCAM_REQUIRE_EFINIX_NATIVE": "1",
                                          "EFINITY_HOME": "/nonexistent",
                                          "LITEX_ENV_EFINITY": "/nonexistent"}), \
             mock.patch.object(shutil, "which", side_effect=lambda name: "/tool/" + name):
            require_dependencies()


if __name__ == "__main__":
    unittest.main()
