"""Record timed packetizer inputs from the three-frame production fixture.

This is an explicit analysis command, not an automatically repeated unit test:
    python tests/lutcam/record_packet_trace.py --output build/camera.trace

The companion packet_service_model.py duplicates the real output trace for two
concurrent cameras and estimates slot occupancy under an explicit link service
model. The trace itself makes no Ethernet capacity or no-overflow assumption.
"""

import argparse
import csv
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]


def record_adversarial_encoder(directory, output):
    """Record the bounded production encoder with flat/binary/late detail."""
    command = ["verilator", "--cc", "--exe", "--build", "--assert", "-j", "2",
               "-Wno-fatal", "--top-module", "lwc_burst_tb", "--Mdir", str(directory),
               "-GBURST_LIMIT_ENABLE=1", "-f", "rtl/lwc/codec_encoder_sources.f",
               "tests/lutcam/lwc_burst_tb.sv", str(ROOT / "tests/lutcam/lwc_burst_probe.cpp")]
    compiled = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=240)
    (directory / "compile.log").write_text(compiled.stdout + compiled.stderr)
    if compiled.returncode:
        raise SystemExit(f"encoder trace build failed; see {directory / 'compile.log'}")
    csv_path = directory / "encoder.csv"
    run = subprocess.run([str(directory / "Vlwc_burst_tb"), "mixed",
                          str(directory / "encoder.bin"), str(csv_path), "3"],
                         cwd=ROOT, capture_output=True, text=True, timeout=240)
    (directory / "simulation.log").write_text(run.stdout + run.stderr)
    print(run.stdout, end="")
    if run.returncode:
        raise SystemExit(f"encoder trace simulation failed; see {directory / 'simulation.log'}")
    with csv_path.open() as source, output.open("w") as destination:
        for row in csv.DictReader(source):
            destination.write(f"{row['cycle']} I {row['frame_index']} "
                              f"{int(row['data']):04x} {row['sof']} {row['padding']}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--build-directory", type=Path)
    parser.add_argument("--fixture-first", action="store_true",
                        help="run the textured fixture first, then flat and ramp")
    parser.add_argument("--all-fixture", action="store_true",
                        help="run the textured fixture in all three frames")
    parser.add_argument("--encoder-adversarial", action="store_true",
                        help="record the bounded encoder alone: flat, binary noise, then late detail")
    args = parser.parse_args()
    if args.encoder_adversarial and (args.fixture_first or args.all_fixture):
        parser.error("encoder-adversarial does not use camera fixture selection")
    directory = args.build_directory or Path(tempfile.mkdtemp(prefix="lutcam-packet-trace-"))
    directory = directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if args.encoder_adversarial:
        record_adversarial_encoder(directory, output)
        print(f"Timed encoder output trace: {output}")
        print(f"Build and logs: {directory}")
        return
    closure = ROOT / "rtl/tagdetect_lwc/sources.f"
    sources = [line.strip() for line in closure.read_text().splitlines()
               if line.strip() and not line.lstrip().startswith("#")]
    sources += ["rtl/isp/MonoIsp.sv", "rtl/lutcam/CameraPipeline.sv",
                "tests/lutcam/camera_fullchain_tb.sv",
                "tests/lutcam/packet_production_trace_tb.sv"]
    command = ["verilator", "--binary", "--timing", "--assert", "-j", "2",
               "-Wno-fatal", "--top-module", "packet_production_trace_tb",
               "--Mdir", str(directory), f"-I{ROOT / 'rtl/lftag'}",
               *(str(ROOT / source) for source in sources)]
    compiled = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=240)
    (directory / "compile.log").write_text(compiled.stdout + compiled.stderr)
    if compiled.returncode:
        raise SystemExit(f"trace build failed; see {directory / 'compile.log'}")
    run = subprocess.run([str(directory / "Vpacket_production_trace_tb"),
                          f"+TRACE={output}", f"+FIXTURE_FIRST={int(args.fixture_first)}",
                          f"+ALL_FIXTURE={int(args.all_fixture)}"],
                         cwd=ROOT, capture_output=True, text=True, timeout=240)
    (directory / "simulation.log").write_text(run.stdout + run.stderr)
    print(run.stdout, end="")
    if run.returncode:
        raise SystemExit(f"trace simulation failed; see {directory / 'simulation.log'}")
    print(f"Timed production output trace: {output}")
    print(f"Build and logs: {directory}")


if __name__ == "__main__":
    main()
