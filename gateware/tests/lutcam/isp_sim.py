"""Compile current ISP sources with strict warnings and optional vendor DSPs."""

from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[2]


def profiles():
    """Native validation is mandatory when explicitly configured or required."""
    required = os.environ.get("LUTCAM_REQUIRE_EFINIX_NATIVE") == "1"
    if not shutil.which("verilator"):
        if required:
            raise RuntimeError("Required Verilator executable is missing")
        raise unittest.SkipTest("Verilator is required for ISP RTL tests")
    result = [("portable", None)]
    efinity = os.environ.get("LITEX_ENV_EFINITY")
    if efinity:
        model = Path(efinity) / "sim_models/verilog/efx_dsp48.v"
        if not model.is_file():
            raise RuntimeError(f"Configured Efinity DSP48 model is missing: {model}")
        result.append(("native", model.resolve()))
    elif required:
        raise RuntimeError("LUTCAM_REQUIRE_EFINIX_NATIVE=1 requires LITEX_ENV_EFINITY")
    return result


def compile_isp(build, top, testbench, profile):
    """Return the executable and immutable compilation-input fingerprints."""
    name, model = profile
    build = Path(build) / name
    build.mkdir()
    source = ROOT / "rtl/isp/MonoIsp.sv"
    command = ["verilator", "--binary", "--timing", "--assert", "-j", "2",
               "-Wno-TIMESCALEMOD", "--top-module", top, "--Mdir", str(build)]
    inputs = [source, Path(testbench), Path(__file__)]
    if model:
        # Waivers apply only to the unmodified external vendor model; source
        # and testbench warnings remain fatal in both profiles.
        lint = Path(__file__).with_name("efinity_dsp_model.vlt")
        command += ["-DSYNTHESIS", str(lint), str(model)]
        inputs += [lint, model]
    command += [str(source), str(testbench)]
    hashes = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}
    compiled = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120)
    (build / "compile.log").write_text(compiled.stdout + compiled.stderr)
    if compiled.returncode:
        raise RuntimeError(f"{name} ISP compilation failed:\n{compiled.stdout}{compiled.stderr}")
    assert_unchanged(hashes)
    report = {"profile": name, "command": command,
              "source_sha256": {str(path): digest for path, digest in hashes.items()}}
    (build / "source-manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, sort_keys=True))
    return build / ("V" + top), hashes


def assert_unchanged(hashes):
    for path, expected in hashes.items():
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise AssertionError(f"ISP validation input changed: {path}")


def run_isp(binary, hashes, *arguments):
    assert_unchanged(hashes)
    result = subprocess.run([str(binary), *arguments], cwd=binary.parent,
                            capture_output=True, text=True, timeout=60)
    assert_unchanged(hashes)
    if result.returncode:
        raise AssertionError(result.stdout + result.stderr)
    return result.stdout
