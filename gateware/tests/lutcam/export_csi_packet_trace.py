#!/usr/bin/env python3
"""Export actual CSI -> ISP -> camera output timestamps for packet replay.

The explicit ignored Rust analysis test sends either flat/binary/late scenes or
three distinct consecutive binary-noise or full-range grayscale-noise frames.
Random scenes use a continuous xorshift64 sequence from the selected seed.
Ordinary camera regressions always retain their golden checks. This export
preserves source hashes, receiver/core fault checks, fixed frame deadlines,
and every native output word; replay_packet_trace.py checks the exact
reconstructed network payload against these captured words.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
ADAPTER = ROOT / "tests/lftag/tests/rtl_adapters/pipeline_ov9281_camera.sv"
RUST_TEST = ROOT / "tests/lftag/tests/rtl_tagdetect_lwc.rs"


def source_hashes():
    paths = set((ROOT / "rtl").rglob("*.sv"))
    paths.update((ROOT / "rtl").rglob("*.svh"))
    paths.update((ROOT / "rtl").rglob("*.vh"))
    paths.update((ROOT / "rtl").rglob("*.f"))
    paths.update([ADAPTER, RUST_TEST, Path(__file__).resolve()])
    paths.update((ROOT / "tests/lftag/tests/support").glob("*.rs"))
    paths.add(ROOT / "tests/support/src/lib.rs")
    return {str(path.relative_to(ROOT.parent)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(paths)}


def trace_adapter(trace):
    source = ADAPTER.read_text().replace(
        "module pipeline_ov9281_camera (", "module pipeline_ov9281_trace_camera (", 1)
    logger = r'''
  integer trace_file;
  integer trace_cycle = 0;
  initial begin
    trace_file = $fopen(TRACE_PATH, "w");
    if (trace_file == 0) $fatal(1, "Cannot open actual CSI trace");
  end
  always @(posedge core_clk) begin
    if (!reset_n) trace_cycle = 0;
    else begin
      trace_cycle++;
      if (lwc_out_valid) begin
        if (dut.lwc_out_padding && lwc_out_data != 0)
          $fatal(1, "Nonzero data marked as reconstructible zero padding");
        $fwrite(trace_file, "%0d I %0d %04x %0d %0d\n", trace_cycle,
          current_frame_id, lwc_out_data, lwc_out_sof, dut.lwc_out_padding);
      end
      if (inner_fv_valid)
        $fwrite(trace_file, "%0d C %0d %09x %0d\n", trace_cycle,
          current_frame_id, inner_fv_data, hier_inner_end);
      if (lftag_result_valid)
        $fwrite(trace_file, "%0d P %0d %08x %0d\n", trace_cycle,
          current_frame_id, lftag_result_data, lftag_result_last);
      if (camera_idle) $fflush(trace_file);
    end
  end
  final $fclose(trace_file);
'''.replace("TRACE_PATH", json.dumps(str(trace)))
    assert source.count("endmodule") == 1
    return source.replace("endmodule", logger + "\nendmodule")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path,
                        help="directory for trace, adapter, log, and manifest")
    parser.add_argument("--profile", choices=("120", "120.76"), default="120")
    parser.add_argument("--scene", choices=("flat-binary-late", "binary", "uniform"), default="flat-binary-late")
    parser.add_argument("--seed", type=lambda value: int(value, 0), default=0x243f6a8885a308d3)
    parser.add_argument("--frames", type=int, default=3, help="frame count, 1..256 (default: 3)")
    args = parser.parse_args()
    if not 1 <= args.frames <= 256:
        parser.error("frames must be between 1 and 256")
    if not 0 < args.seed < 2**64:
        parser.error("seed must be a nonzero unsigned64-bit integer")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    trace = output / "camera.trace"
    adapter = output / "camera_trace_adapter.sv"
    adapter.write_text(trace_adapter(trace))
    before = source_hashes()
    period = 833_334 if args.profile == "120" else 828_100
    command = ["cargo", "test", "-p", "lftag-rtl-tests", "--test", "rtl_tagdetect_lwc",
               "ov9281_monoisp_export_packet_trace", "--", "--ignored", "--nocapture",
               "--test-threads=1"]
    env = dict(os.environ, LUTCAM_CSI_TRACE_ADAPTER=str(adapter),
               LUTCAM_CSI_TRACE_PERIOD=str(period), LUTCAM_CSI_TRACE_SCENE=args.scene,
               LUTCAM_CSI_TRACE_SEED=str(args.seed), LUTCAM_CSI_TRACE_FRAMES=str(args.frames))
    with (output / "test.log").open("w") as log:
        result = subprocess.run(command, cwd=ROOT.parent, env=env, stdout=log,
                                stderr=subprocess.STDOUT, check=False)
    after = source_hashes()
    drift = {path: {"before": before.get(path), "after": after.get(path)}
             for path in before.keys() | after.keys() if before.get(path) != after.get(path)}
    manifest = {"command": command, "profile": args.profile, "frame_byte_cycles": period,
                "scene": args.scene, "seed": args.seed, "frames": args.frames,
                "returncode": result.returncode, "source_hashes": before,
                "source_drift": drift, "trace": str(trace),
                "adapter_sha256": hashlib.sha256(adapter.read_bytes()).hexdigest()}
    if trace.exists():
        manifest["trace_sha256"] = hashlib.sha256(trace.read_bytes()).hexdigest()
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"{'PASS' if result.returncode == 0 and not drift else 'FAIL'}: {output}")
    return result.returncode or bool(drift)


if __name__ == "__main__":
    raise SystemExit(main())
