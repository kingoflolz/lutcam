#!/usr/bin/env python3
"""Replay actual camera outputs through the compiled production packet transport.

The trace format matches packet_service_model.py. One camera trace is driven
onto both cameras at identical timestamps. Image and child events cannot stall;
pose events retain valid/data until each camera's sink accepts them. The UDP
consumer accepts one 32-bit word every four 120 MHz clocks with a 128-clock
post-datagram gap. Six maximally batched data packets occupy 3,936 clocks.
The service contract allows at most one 80-byte Etherbone reply between
consecutive image visits. LiteEth regressions check wire formatting and costs.

This verifies the supplied trace and stated service schedule. It does not
establish an arbitrary-image arrival bound or a complete LiteEth timing bound.
"""

import argparse
from collections import defaultdict
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import time

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY))

from migen import Array, If, Module, Signal
from migen.fhdl import verilog

from fpga.litex.lutcam.packet import IMAGE_FRAGMENT_WORDS, SharedPacketTransport
from packet_service_model import assemble, read_trace


KINDS = (("I", "image"), ("C", "children"), ("P", "pose"))


def application_header(camera, stream, frame, sequence, offset, length):
    return struct.pack(
        ">8I", 0x4C555443, 0x20 | camera << 16 | stream << 8,
        frame, sequence, offset, length, 0, 0)


def expected_packets(words, fragment_words=IMAGE_FRAGMENT_WORDS):
    """Build compact images and canonical auxiliary records for batch comparison."""
    products = {"I": [], "C": [], "P": []}
    image = bytearray()
    children = bytearray()
    pose = []
    image_frame = child_frame = pose_frame = None
    for word in words:
        if word.kind == "I":
            if word.sof:
                if image_frame is not None:
                    products["I"].append((image_frame, bytes(image)))
                image.clear()
                image_frame = word.frame
            if not word.padding:
                image.extend(word.data.to_bytes(2, "big"))
        elif word.kind == "C":
            if not children:
                child_frame = word.frame
            children.extend(word.data.to_bytes(8, "big"))
            if word.last:
                products["C"].append((child_frame, bytes(children)))
                children.clear()
        else:
            if not pose:
                pose_frame = word.frame
            pose.append(word.data)
            if word.last:
                if len(pose) == 22 and pose[1] & 0x80000000:
                    products["P"].append((pose_frame, b"".join(data.to_bytes(4, "big") for data in pose)))
                pose.clear()
    if image_frame is not None:
        products["I"].append((image_frame, bytes(image)))
    result = {}
    for camera in range(2):
        for stream, (kind, _) in enumerate(KINDS, 1):
            packets = []
            for frame, payload in products[kind]:
                if kind == "I":
                    fragment_bytes = fragment_words * 2
                    for offset in range(0, len(payload), fragment_bytes):
                        fragment = payload[offset:offset + fragment_bytes]
                        packets.append(application_header(camera, stream, frame, len(packets),
                                                          offset, len(fragment)) + fragment)
                else:
                    if kind == "C" and len(payload) != 72:
                        raise ValueError("child groups require exactly nine tuples")
                    packets.append(application_header(camera, stream, frame, len(packets),
                                                      0, len(payload)) + payload)
            result[6000 + camera * 3 + stream - 1] = packets
    return result


def generate_fixture(directory, words, slots, frame_words=64000, fragment_words=IMAGE_FRAGMENT_WORDS,
                     sequence_latency=10, packet_gap=128):
    """Generate the actual packet RTL, timestamp memories, and its testbench."""
    dut = Module()
    ports = {}
    controllers = []
    dut.submodules.transport = transport = SharedPacketTransport(
        image_slots=slots[0], child_slots=slots[1], pose_slots=slots[2],
        frame_words=frame_words, fragment_words=fragment_words, external_destination=True,
        external_context_held=True, compact_padding=True,
        batched_packets=True)
    sent = Array(Signal(32) for _ in transport.channels)
    if sequence_latency < 0:
        raise ValueError("sequence latency cannot be negative")
    lookup_age = Signal(max=max(2, sequence_latency + 1))
    dut.sync += If(~transport.sequence_request, lookup_age.eq(0)).Elif(
        lookup_age != sequence_latency, lookup_age.eq(lookup_age + 1))
    dut.comb += [
        transport.sequence_valid.eq(transport.sequence_request & (lookup_age == sequence_latency)),
        transport.sequence_value.eq(sent[transport.sequence_stream]),
        transport.sequence_destination_ip.eq(0xC0A80102),
        transport.sequence_destination_port.eq(6000 + transport.sequence_stream),
    ]
    for index, queue in enumerate(transport.channels):
        dut.comb += queue.destination_update.eq(1)
        dut.sync += If(queue.sent_event, sent[index].eq(sent[index] + 1))
    for camera in range(2):
        camera_controllers = transport.channels[camera * 3:camera * 3 + 3]
        for stream, ((kind, name), controller) in enumerate(zip(KINDS, camera_controllers)):
            label = f"camera{camera}_{name}"
            controllers.append(controller)
            for field, value in (("enable", 1), ("destination_ip", 0xC0A80102),
                                 ("destination_port", 6000 + camera * 3 + stream)):
                target = getattr(controller, field)
                dut.comb += target.eq(value)
            fields = ["valid", "data", "frame_id"]
            fields += ["sof", "padding"] if kind == "I" else ["end"] if kind == "C" else ["last", "ready"]
            for field in fields:
                ports[f"{label}_{field}"] = getattr(controller.sink, field)
            for field in ("faults", "high_water", "fifo_level", "packets_sent"):
                ports[f"{label}_{field}"] = (sent[camera * 3 + stream] if field == "packets_sent"
                                              else getattr(controller, field))
    udp_source = transport.source
    for field in ("valid", "ready", "data", "last", "last_be", "length", "src_port", "dst_port", "ip_address"):
        ports[f"udp_{field}"] = getattr(udp_source, field)
    for name, signal in ports.items():
        signal.name_override = name
    rtl = verilog.convert(dut, ios=set(ports.values()), name="PacketTraceFixture")
    (directory / "packet.v").write_text(rtl.main_source)
    for name, content in rtl.data_files.items():
        (directory / name).write_text(content)
    events = {kind: [word for word in words if word.kind == kind] for kind, _ in KINDS}
    for kind, name in KINDS:
        previous = -1
        encoded = []
        for word in events[kind]:
            if word.cycle <= previous:
                raise ValueError(f"{name}: multiple events in one source clock")
            previous = word.cycle
            marker = word.sof if kind == "I" else word.last
            packed = (word.cycle << 96) | (word.frame << 64) | (int(word.padding) << 34) | (int(marker) << 33) | word.data
            encoded.append(f"{packed:032x}\n")
        (directory / f"{name}.hex").write_text("".join(encoded) if encoded else "0\n")

    declarations = ["reg camera_image_valid = 0, camera_image_sof = 0, camera_image_padding = 0;",
                    "reg [15:0] camera_image_data = 0;", "reg [31:0] camera_image_frame_id = 0;",
                    "reg camera_children_valid = 0, camera_children_end = 0;",
                    "reg [32:0] camera_children_data = 0;", "reg [31:0] camera_children_frame_id = 0;"]
    for kind, name in KINDS:
        declarations.append(f"localparam integer {name.upper()}_COUNT = {len(events[kind])};")
        declarations.append(f"reg [127:0] {name}_events [0:{max(0, len(events[kind]) - 1)}];")
    connections = [".sys_clk(sys_clk)", ".sys_rst(sys_rst)"]
    for camera in range(2):
        declarations += [f"reg camera{camera}_pose_valid = 0, camera{camera}_pose_last = 0;",
                         f"reg [31:0] camera{camera}_pose_data = 0, camera{camera}_pose_frame_id = 0;",
                         f"wire camera{camera}_pose_ready;", f"integer pose{camera}_index = 0;"]
        for _, name in KINDS:
            label = f"camera{camera}_{name}"
            fields = ["valid", "data", "frame_id"]
            fields += ["sof", "padding"] if name == "image" else ["end"] if name == "children" else ["last", "ready"]
            for field in fields:
                connected = f"camera_{name}_{field}" if name != "pose" else f"{label}_{field}"
                connections.append(f".{label}_{field}({connected})")
            declarations += [f"wire [1:0] {label}_faults;",
                             f"wire [31:0] {label}_high_water, {label}_fifo_level, {label}_packets_sent;"]
            for field in ("faults", "high_water", "fifo_level", "packets_sent"):
                connections.append(f".{label}_{field}({label}_{field})")
    for field in ("valid", "ready", "data", "last", "last_be", "length", "src_port", "dst_port", "ip_address"):
        connections.append(f".udp_{field}(udp_{field})")
    input_driver = []
    for kind, name in KINDS[:2]:
        index = f"{name}_index"
        marker = "sof" if kind == "I" else "end"
        data_width = 16 if kind == "I" else 33
        input_driver += [f"camera_{name}_valid = 0;",
                         f"if ({index} < {name.upper()}_COUNT) begin",
                         f"  if ({name}_events[{index}][127:96] < cycle) $fatal(1, \"missed {name} event\");",
                         f"  if ({name}_events[{index}][127:96] == cycle) begin",
                         f"    camera_{name}_valid = 1;",
                         f"    camera_{name}_frame_id = {name}_events[{index}][95:64];",
                         f"    camera_{name}_data = {name}_events[{index}][{data_width - 1}:0];",
                         f"    camera_{name}_{marker} = {name}_events[{index}][33];"]
        if kind == "I":
            input_driver.append(f"    camera_image_padding = image_events[{index}][34];")
        input_driver += [f"    {index} = {index} + 1;", "  end", "end"]
    for camera in range(2):
        input_driver += [f"camera{camera}_pose_valid = 0;",
                         f"if (pose{camera}_index < POSE_COUNT && pose_events[pose{camera}_index][127:96] <= cycle) begin",
                         f"  camera{camera}_pose_valid = 1;",
                         f"  camera{camera}_pose_frame_id = pose_events[pose{camera}_index][95:64];",
                         f"  camera{camera}_pose_data = pose_events[pose{camera}_index][31:0];",
                         f"  camera{camera}_pose_last = pose_events[pose{camera}_index][33];", "end"]
    monitors = []
    empty_conditions = []
    for camera in range(2):
        monitors.append(f"if (camera{camera}_pose_valid && camera{camera}_pose_ready) pose{camera}_index = pose{camera}_index + 1;")
        for _, name in KINDS:
            label = f"camera{camera}_{name}"
            monitors.append(f'if ({label}_faults !== 0) $fatal(1, "FAULT cycle=%0d stream={label} faults=%0d level=%0d", cycle, {label}_faults, {label}_fifo_level);')
            empty_conditions.append(f"({label}_fifo_level == 0)")
    summary = []
    for camera in range(2):
        for _, name in KINDS:
            label = f"camera{camera}_{name}"
            summary.append(f'$fwrite(stats_file, "{label} %0d %0d %0d %0d\\n", {label}_faults, {label}_high_water, {label}_fifo_level, {label}_packets_sent);')
    last_cycle = max((word.cycle for word in words), default=0)
    testbench = f"""
module PacketTraceTest;
  reg sys_clk = 0, sys_rst = 1;
  always #5 sys_clk = ~sys_clk;
  integer cycle = -1, gap = 0, quiet = 0, packets = 0;
  integer image_index = 0, children_index = 0;
  integer output_file, stats_file;
  reg udp_ready = 0;
  wire udp_valid, udp_last;
  wire [31:0] udp_data, udp_ip_address;
  wire [3:0] udp_last_be;
  wire [15:0] udp_length, udp_src_port, udp_dst_port;
  {chr(10).join(declarations)}
  PacketTraceFixture dut ({', '.join(connections)});
  initial begin
    $readmemh("image.hex", image_events);
    $readmemh("children.hex", children_events);
    $readmemh("pose.hex", pose_events);
    output_file = $fopen("udp_beats.txt", "w");
    stats_file = $fopen("statistics.txt", "w");
    repeat (4) @(posedge sys_clk);
    #1 sys_rst = 0;
  end
  always @(negedge sys_clk) begin
    if (!sys_rst) begin
      cycle = cycle + 1;
      {chr(10).join(input_driver)}
      udp_ready = (gap == 0) && ((cycle % 4) == 0);
      if (gap > 0) gap = gap - 1;
    end
  end
  always @(posedge sys_clk) begin
    if (!sys_rst) begin
      {chr(10).join(monitors)}
      if (udp_valid && udp_ready) begin
        $fwrite(output_file, "%08x %01x %01x %0d %0d %0d %08x\\n", udp_data, udp_last, udp_last_be, udp_length, udp_src_port, udp_dst_port, udp_ip_address);
        if (udp_last) begin
          packets = packets + 1;
          gap = {packet_gap};
        end
      end
      if ((image_index == IMAGE_COUNT) && (children_index == CHILDREN_COUNT) &&
          (pose0_index == POSE_COUNT) && (pose1_index == POSE_COUNT) &&
          !camera_image_valid && !camera_children_valid && !camera0_pose_valid && !camera1_pose_valid &&
          !udp_valid && {' && '.join(empty_conditions)}) quiet = quiet + 1;
      else quiet = 0;
      if (quiet == 64) begin
        {chr(10).join(summary)}
        $display("PASS cycle=%0d packets=%0d", cycle, packets);
        $fclose(output_file);
        $fclose(stats_file);
        $finish;
      end
      if (cycle > {last_cycle + 2000000}) $fatal(1, "trace replay timed out");
    end
  end
endmodule
"""
    (directory / "testbench.v").write_text(testbench)


def verify_output(directory, expected, verify_host=False):
    received = defaultdict(list)
    pending = bytearray()
    metadata = None
    for line_number, line in enumerate((directory / "udp_beats.txt").read_text().splitlines(), 1):
        data, last, mask, length, source, destination, ip = line.split()
        current = (int(length), int(source), int(destination), int(ip, 16))
        if metadata is None:
            metadata = current
        if current != metadata:
            raise AssertionError(f"UDP metadata changed inside datagram at output line {line_number}")
        if int(last, 16):
            valid_bytes = current[0] - len(pending)
            if not 1 <= valid_bytes <= 4 or int(mask, 16) != 1 << (valid_bytes - 1):
                raise AssertionError(f"invalid UDP final beat at output line {line_number}")
            pending.extend(int(data, 16).to_bytes(4, "little")[:valid_bytes])
            if current[1] not in expected or current[2] != current[1] or current[3] != 0xC0A80102:
                raise AssertionError(f"unexpected UDP address/ports at output line {line_number}: {current}")
            received[current[1]].append(bytes(pending))
            pending.clear()
            metadata = None
        elif int(mask, 16) != 0:
            raise AssertionError(f"final-byte mask on non-final beat at output line {line_number}")
        else:
            pending.extend(int(data, 16).to_bytes(4, "little"))
    if pending:
        raise AssertionError("incomplete UDP datagram at end of replay")
    for port, expected_stream in expected.items():
        actual_stream = received[port]
        product = (port - 6000) % 3
        if product:
            record_bytes, maximum_records = (72, 4) if product == 1 else (88, 2)
            records = []
            for sequence, actual in enumerate(actual_stream):
                if len(actual) < 32:
                    raise AssertionError(f"port {port}: truncated batch header")
                fields = struct.unpack(">8I", actual[:32])
                payload = actual[32:]
                count, remainder = divmod(len(payload), record_bytes)
                wanted_header = application_header(
                    (port - 6000) // 3, product + 1, fields[2], sequence,
                    0, len(payload))
                if actual[:32] != wanted_header or remainder or not 1 <= count <= maximum_records:
                    raise AssertionError(f"port {port}: malformed or oversized batch {sequence}")
                records.extend((fields[2], payload[offset:offset + record_bytes])
                               for offset in range(0, len(payload), record_bytes))
            wanted_records = [(struct.unpack_from(">I", data, 8)[0], data[32:])
                              for data in expected_stream]
            if records != wanted_records:
                raise AssertionError(f"port {port}: batched records differ from source trace")
            continue
        if len(actual_stream) != len(expected_stream):
            raise AssertionError(f"port {port}: expected {len(expected_stream)} datagrams, received {len(actual_stream)}")
        for sequence, (actual, wanted) in enumerate(zip(actual_stream, expected_stream)):
            if actual != wanted:
                difference = next((index for index, pair in enumerate(zip(actual, wanted)) if pair[0] != pair[1]),
                                  min(len(actual), len(wanted)))
                raise AssertionError(f"port {port} packet {sequence}: first differing byte {difference}; "
                                     f"actual length {len(actual)}, expected {len(wanted)}")
    if verify_host:
        from tools.lutcam.receive import IMAGE, StreamReceiver, parse_datagram
        receiver = StreamReceiver()
        delivered_images = defaultdict(bytearray)
        wanted_images = defaultdict(bytearray)
        for port in (6000, 6003):
            for data in expected[port]:
                fields = struct.unpack(">8I", data[:32])
                wanted_images[((fields[1] >> 16) & 0xff, fields[2])].extend(data[32:])
        image_directory = directory / "images"
        image_directory.mkdir(exist_ok=True)
        for stream_packets in received.values():
            for data in stream_packets:
                packet = parse_datagram(data)
                if packet.stream == IMAGE:
                    # The offline oracle knows the frame contents; transport
                    # forwards fragments without interpreting codec completion.
                    fragment = receiver.feed(data)
                    if fragment != packet:
                        raise AssertionError("host did not deliver the image fragment unchanged")
                    key = fragment.camera, fragment.frame_id
                    if fragment.offset != len(delivered_images[key]):
                        raise AssertionError(f"host image fragment gap for {key}")
                    delivered_images[key].extend(fragment.payload)
        if delivered_images != wanted_images:
            raise AssertionError("host image fragments differ from recorded semantic words")
        for (camera, frame_id), payload in delivered_images.items():
            (image_directory / f"camera{camera}-frame{frame_id}.lwc").write_bytes(payload)
    statistics = {}
    for line in (directory / "statistics.txt").read_text().splitlines():
        name, faults, high_water, level, packets = line.split()
        statistics[name] = {"faults": int(faults), "high_water": int(high_water),
                            "fifo_level": int(level), "packets_sent": int(packets)}
        camera = int(name[len("camera")])
        stream = [name for _, name in KINDS].index(name.split("_", 1)[1])
        if int(packets) != len(received[6000 + camera * 3 + stream]):
            raise AssertionError(f"{name}: hardware sent-packet counter differs from observed traffic")
        if int(faults) or int(level):
            raise AssertionError(f"{name}: controller failed to drain cleanly")
    if len(statistics) != 6:
        raise AssertionError("missing final statistics")
    return statistics, sum(len(packets) for packets in received.values())


def run(trace, slots, directory, timeout=240, frame_words=64000, fragment_words=IMAGE_FRAGMENT_WORDS,
        sequence_latency=10):
    words = read_trace(trace)
    _, frames = assemble(words, frame_words=frame_words, fragment_words=fragment_words)
    expected = expected_packets(words, fragment_words)
    packet_gap = 128
    directory.mkdir(parents=True, exist_ok=True)
    generate_fixture(directory, words, slots, frame_words, fragment_words, sequence_latency, packet_gap)
    compilation = subprocess.run(["iverilog", "-g2012", "-s", "PacketTraceTest", "-o", "simulate", "packet.v", "testbench.v"],
                                 cwd=directory, capture_output=True, text=True, timeout=60)
    (directory / "compile.log").write_text(compilation.stdout + compilation.stderr)
    if compilation.returncode:
        raise RuntimeError("RTL replay compilation failed: " + compilation.stdout + compilation.stderr)
    started = time.monotonic()
    simulation = subprocess.run(["vvp", "simulate"], cwd=directory, capture_output=True, text=True, timeout=timeout)
    elapsed = time.monotonic() - started
    (directory / "simulation.log").write_text(simulation.stdout + simulation.stderr)
    if simulation.returncode or "PASS" not in simulation.stdout:
        raise AssertionError("RTL trace replay failed: " + simulation.stdout + simulation.stderr)
    statistics, packets = verify_output(directory, expected, verify_host=frame_words == 64000)
    for frame, stats in frames.items():
        stats["image_datagrams"] = sum(struct.unpack_from(">I", data, 8)[0] == int(frame)
                                        for data in expected[6000])
        # Auxiliary datagram counts depend on each camera's queue timing;
        # the source trace defines records, not predetermined batches.
        stats.pop("children_datagrams")
        stats.pop("pose_datagrams")
    result = {
        "trace": str(trace), "slots": dict(zip((name for _, name in KINDS), slots)),
        "implementation": "shared serializer",
        "reserved_header_byte": 0,
        "simulation_seconds": round(elapsed, 3), "statistics": statistics,
        "total_datagrams": packets, "per_frame_single_camera": frames,
        "service": {"udp_word_bits": 32, "clocks_per_word": 4,
                    "post_datagram_stall_clocks": packet_gap, "clock_hz": 120000000,
                    "sequence_lookup_clocks": sequence_latency,
                    "maximum_payload_bits_per_second_before_packet_gaps": 960000000,
                    "packet_gap_budget": "six maximum data batches: 3168 payload/header clocks + 6*128 gap clocks = 3936; at most one 80-byte control reply between image visits",
                    "assumptions": ["timestamps describe image and child arrivals without backpressure",
                                    "pose words hold valid and data until accepted independently per camera",
                                    "no additional unbounded UDP-ready stalls",
                                    "established link and pre-resolved destinations; ARP/ICMP traffic is outside this service schedule"]},
        "verification": "all datagram headers, frame IDs, sequences, offsets and ordered record bytes match the trace",
        "host_image_fragment_delivery_verified": frame_words == 64000,
        "scope": "two simultaneous copies of this trace under the stated service schedule",
    }
    (directory / "report.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--slots", type=int, nargs=3, default=(8, 8, 2), metavar=("IMAGE", "CHILD", "POSE"))
    parser.add_argument("--outdir", type=Path)
    parser.add_argument("--timeout", type=int, default=240)
    parser.add_argument("--frame-words", type=int, default=64000)
    parser.add_argument("--fragment-words", type=int, default=IMAGE_FRAGMENT_WORDS)
    parser.add_argument("--sequence-latency", type=int, default=10,
                        help="shared sequence/destination lookup delay in clocks (default: 10)")
    args = parser.parse_args()
    if not shutil.which("iverilog") or not shutil.which("vvp"):
        parser.error("Icarus Verilog (iverilog and vvp) is required")
    if args.outdir:
        result = run(args.trace, args.slots, args.outdir.resolve(), args.timeout, args.frame_words, args.fragment_words, args.sequence_latency)
    else:
        with tempfile.TemporaryDirectory(prefix="lutcam-trace-replay-") as temporary:
            result = run(args.trace, args.slots, Path(temporary), args.timeout, args.frame_words, args.fragment_words, args.sequence_latency)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
