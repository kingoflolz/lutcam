"""Batched camera queues through the real 120/125 MHz UDP/MAC datapath.

The control competitor is an explicitly bounded model: one 80-byte UDP request
may start every 2,000 system clocks, with no accumulated request backlog. This
checks serializer planning/record transitions and native MAC continuity; it is
not a claim about unbounded Etherbone traffic or arbitrary camera production.
"""

from collections import defaultdict
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
from types import SimpleNamespace
import zlib

import pytest
from migen import Array, If, Signal
from migen.fhdl import verilog

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))
from lutcam.packet import SharedPacketTransport
from test_udp_wire import _UdpLoopback


class _BatchedUdpLoopback(_UdpLoopback):
    def __init__(self):
        super().__init__(16)
        # The inherited data port supplies a fixed broadcast destination and
        # local UDP port. All six logical streams are identified by LUTC header.
        self.submodules.transport = transport = SharedPacketTransport(
            compact_padding=True, batched_packets=True,
            external_destination=True, external_context_held=True)
        self.comb += transport.source.connect(
            self.mac.sink, omit={"src_port", "dst_port", "ip_address"})
        self.service_enable = Signal(name_override="service_enable")
        delay = Signal(max=11)
        sequences = Array(Signal(32) for _ in transport.channels)
        self.comb += [
            transport.sequence_valid.eq(self.service_enable & (delay == 10)),
            transport.sequence_value.eq(sequences[transport.sequence_stream]),
            transport.sequence_destination_ip.eq(0xc0a801ff),
            transport.sequence_destination_port.eq(6000),
        ]
        self.sync += If(~self.service_enable | ~transport.sequence_request,
            delay.eq(0),
        ).Elif(delay != 10, delay.eq(delay + 1))
        for index, channel in enumerate(transport.channels):
            self.comb += [channel.enable.eq(1), channel.capture_enable.eq(1)]
            self.sync += If(channel.sent_event, sequences[index].eq(sequences[index] + 1))
        control = self.core.udp.crossbar.get_port(6006, dw=32)
        self.comb += [control.sink.src_port.eq(6006), control.sink.dst_port.eq(6000),
                      control.sink.ip_address.eq(0xc0a801ff), control.source.ready.eq(1)]
        self.control = SimpleNamespace(sink=control.sink)
        self.ios.add(self.service_enable)
        for camera in range(2):
            for product, fields in (
                (0, ("valid", "data", "sof", "padding", "frame_id")),
                (1, ("valid", "data", "end", "frame_id")),
                (2, ("valid", "ready", "data", "last", "frame_id")),
            ):
                endpoint = transport.channels[camera * 3 + product].sink
                for field in fields:
                    signal = getattr(endpoint, field)
                    signal.name_override = f"{'image child pose'.split()[product]}{camera}_{field}"
                    self.ios.add(signal)
            for product in range(3):
                queue = transport.channels[camera * 3 + product]
                for field in ("faults", "fifo_level", "committed_level"):
                    signal = getattr(queue, field)
                    signal.name_override = f"queue{camera * 3 + product}_{field}"
                    self.ios.add(signal)
        for field in ("valid", "ready", "data", "last", "last_be", "length"):
            signal = getattr(control.sink, field)
            signal.name_override = f"control_{field}"
            self.ios.add(signal)


def _image(camera):
    return b"".join(((index * 37 + camera * 0x2222 + 0x1234) & 0xffff).to_bytes(2, "big")
                    for index in range(7 * 512 + 1))


def _children(camera, group):
    return b"".join((((group & 1) << 32) | (camera << 24) | (group << 12)
                     | (index * 17 + 1)).to_bytes(8, "big") for index in range(9))


def _pose(camera, record):
    words = [0x83000000 + camera * 0x100000 + record * 0x10000 + index for index in range(22)]
    words[1] |= 0x80000000
    # Actual unversioned status on camera0, opaque upper-nibble bits on
    # camera1: neither changes batching or forwarding through the UDP/MAC.
    words[8] = (15 * camera) << 28 | 1
    return struct.pack(">22I", *words)


def _testbench():
    declarations, image_drives, image_stops, child_drives, child_stops, pose_tasks = [], [], [], [], [], []
    for camera in range(2):
        declarations.append(f'''
reg image{camera}_valid=0,image{camera}_sof=0,image{camera}_padding=0;
reg [15:0] image{camera}_data=0;reg [31:0] image{camera}_frame_id=19;
reg child{camera}_valid=0,child{camera}_end=0;reg [32:0] child{camera}_data=0;
reg [31:0] child{camera}_frame_id=19;
reg pose{camera}_valid=0,pose{camera}_last=0;wire pose{camera}_ready;
reg [31:0] pose{camera}_data=0,pose{camera}_frame_id=19;
''')
        image_drives.append(f'''image{camera}_valid=1; image{camera}_sof=(i==0);
 image{camera}_padding=(i>=3585);image{camera}_data=(i>=3585)?0:((i*37+{camera}*8738+4660)&65535);''')
        image_stops.append(f"image{camera}_valid=0;image{camera}_sof=0;")
        child_drives.append(f'''child{camera}_valid=1;child{camera}_end=(i==8);
 child{camera}_frame_id=19+(g>=4);
 child{camera}_data=((g&1)*33'h100000000)|({camera}<<24)|(g<<12)|(i*17+1);''')
        child_stops.append(f"child{camera}_valid=0;child{camera}_end=0;")
        pose_tasks.append(f'''
task send_pose{camera};
 input integer record_index;
 integer word_index;
 begin
  for(word_index=0;word_index<22;word_index=word_index+1) begin
   @(negedge sys_clk);pose{camera}_valid=1;pose{camera}_last=(word_index==21);
   pose{camera}_data=32'h83000000+{camera}*32'h100000+record_index*32'h10000+word_index;
   if(word_index==8) pose{camera}_data=32'h{(15 * camera) << 28 | 1:08x};
   @(posedge sys_clk);while(!pose{camera}_ready) @(posedge sys_clk);
  end
  @(negedge sys_clk);pose{camera}_valid=0;pose{camera}_last=0;
 end
endtask
''')
    for queue in range(6):
        slots = 2 if queue % 3 == 2 else 8
        declarations.append(f"wire [1:0] queue{queue}_faults;wire [{(slots+1).bit_length()-1}:0] queue{queue}_fifo_level;wire [{slots.bit_length()-1}:0] queue{queue}_committed_level;")
    checks = "\n".join(f'if(queue{queue}_faults) $fatal(1,"Queue {queue} fault: %0d",queue{queue}_faults);' for queue in range(6))
    prefill = "\n".join(f'if(queue{queue}_committed_level!={2 if queue%3==2 else 8}) $fatal(1,"Queue {queue} did not prefill");' for queue in range(6))
    drain = "\n".join(f'if(queue{queue}_fifo_level!=0) $fatal(1,"Queue {queue} did not drain");' for queue in range(6))
    return '''`timescale 1ps/1ps
module tb;
reg sys_clk=0,eth_tx_clk=0,eth_rx_clk=0;
always #4167 sys_clk=~sys_clk;
always #4000 eth_tx_clk=~eth_tx_clk;
initial begin #1000;forever #4000 eth_rx_clk=~eth_rx_clk;end
reg sys_rst=1,eth_tx_rst=1,eth_rx_rst=1,service_enable=0;
wire tx_valid,tx_ready,tx_last;wire [15:0]tx_length;wire [31:0]tx_data;wire [3:0]tx_last_be;
reg rx_ready=0;wire rx_valid,rx_last;wire [31:0]rx_data;wire [3:0]rx_last_be,rx_error;
wire wire_valid,wire_last;wire [7:0]wire_data;
reg control_valid=0,control_last=0;wire control_ready;reg [31:0]control_data=0;
reg [3:0]control_last_be=0;reg [15:0]control_length=80;
DECLARATIONS
mac_probe dut(.*);
integer txf,rxf,sf,pf,i,g,cycle=0,wire_cycle=0,gap=0,seen=0,tx_stalls=0,rx_stalls=0;
integer control_word,control_packet,control_start;
reg in_packet=0,app_packet=0,app_offered=0;
integer first_offer=0;
always @(negedge sys_clk) begin cycle=cycle+1;rx_ready=(cycle%13)>=3;end
always @(posedge sys_clk) if(!sys_rst) begin
 CHECKS
 if(tx_valid && !tx_ready) tx_stalls=tx_stalls+1;
 if(rx_valid && !rx_ready) rx_stalls=rx_stalls+1;
 if(tx_valid && !app_offered) begin first_offer=cycle;app_offered=1;end
 if(tx_valid && tx_ready) begin
  if(!app_packet) $fwrite(pf,"%0d %0d %0d ",first_offer,cycle,tx_length);
  if(tx_last) begin $fwrite(pf,"%0d\\n",cycle);app_offered=0;end
  app_packet=!tx_last;
 end
 if(rx_valid && rx_ready) begin
  if(rx_error) $fatal(1,"RX CRC/error flagged");
  $fwrite(rxf,"%08x %01x %01x\\n",rx_data,rx_last,rx_last_be);
 end
end
always @(posedge eth_tx_clk) if(!eth_tx_rst) begin
 wire_cycle=wire_cycle+1;
 if(wire_valid) begin
  if(!in_packet) $fwrite(sf,"%0d\\n",wire_cycle);
  if(!in_packet && seen && gap<12) $fatal(1,"Short IFG");
  $fwrite(txf,"%02x",wire_data);in_packet=1;gap=0;
  if(wire_last) begin $fwrite(txf,"\\n");in_packet=0;seen=1;end
 end else begin
  if(in_packet) $fatal(1,"Wire underflow at %0d",wire_cycle);
  gap=gap+1;
 end
end
POSE_TASKS
initial begin
 wait(service_enable);
 // A bounded control model, not the real Etherbone parser. Starting a new
 // request requires 2,000 clocks since the preceding request; no catch-up.
 for(control_packet=0;control_packet<12;control_packet=control_packet+1) begin
  control_start=cycle;
  for(control_word=0;control_word<20;control_word=control_word+1) begin
   @(negedge sys_clk);control_valid=1;control_last=(control_word==19);
   control_last_be=(control_word==19)?8:0;
   control_data=32'h4c525443+control_word+control_packet*256;
   @(posedge sys_clk);while(!control_ready) @(posedge sys_clk);
  end
  @(negedge sys_clk);control_valid=0;
  while(cycle-control_start<2000) @(negedge sys_clk);
 end
end
initial begin
 txf=$fopen("wire.txt","w");rxf=$fopen("rx.txt","w");sf=$fopen("starts.txt","w");pf=$fopen("packet_starts.txt","w");
 repeat(8) @(negedge sys_clk);sys_rst=0;eth_tx_rst=0;eth_rx_rst=0;
 // Image is selected first, while the 10-cycle context provider is gated.
 // Its existing payload slots and every auxiliary queue are filled before go.
 for(i=0;i<64000;i=i+1) begin
  @(negedge sys_clk);
  IMAGE_DRIVES
 end
 @(negedge sys_clk);IMAGE_STOPS
 for(g=0;g<8;g=g+1) begin
  for(i=0;i<9;i=i+1) begin
   @(negedge sys_clk);CHILD_DRIVES
  end
  @(negedge sys_clk);CHILD_STOPS
 end
 send_pose0(0);send_pose0(1);send_pose1(0);send_pose1(1);
 repeat(8) @(negedge sys_clk);
 PREFILL
 service_enable=1;
 repeat(35000) @(negedge sys_clk);
 DRAIN
 if(tx_stalls==0 || rx_stalls==0) $fatal(1,"Backpressure was not exercised");
 $fclose(txf);$fclose(rxf);$fclose(sf);$fclose(pf);$finish;
end
initial begin #1000000000;$fatal(1,"Timeout");end
endmodule
'''.replace("DECLARATIONS", "\n".join(declarations)).replace("POSE_TASKS", "\n".join(pose_tasks)).replace("CHECKS", checks).replace("PREFILL", prefill).replace("DRAIN", drain).replace("IMAGE_DRIVES", "\n".join(image_drives)).replace("IMAGE_STOPS", "\n".join(image_stops)).replace("CHILD_DRIVES", "\n".join(child_drives)).replace("CHILD_STOPS", "\n".join(child_stops))


@pytest.mark.skipif(shutil.which("verilator") is None, reason="Verilator required")
def test_batched_transport_udp_wire_continuity_and_service(tmp_path):
    top = _BatchedUdpLoopback()
    (tmp_path / "dut.v").write_text(str(verilog.convert(top, ios=top.ios, name="mac_probe")))
    (tmp_path / "tb.v").write_text(_testbench())
    for command in (
        ["verilator", "--binary", "--timing", "-Wno-fatal", "--top-module", "tb",
         "-j", "4", "dut.v", "tb.v"],
        [str(tmp_path / "obj_dir/Vtb")],
    ):
        result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=180)
        assert result.returncode == 0, result.stdout + result.stderr
    wire = [bytes.fromhex(line) for line in (tmp_path / "wire.txt").read_text().splitlines()]
    starts = [int(line) for line in (tmp_path / "starts.txt").read_text().splitlines()]
    received, current = [], bytearray()
    for line in (tmp_path / "rx.txt").read_text().splitlines():
        data, last, last_be = [int(field, 16) for field in line.split()]
        word = data.to_bytes(4, "little")
        if last:
            assert last_be in (1, 2, 4, 8)
            word = word[:last_be.bit_length()]
        current.extend(word)
        if last:
            received.append(bytes(current))
            current.clear()
    assert not current
    payloads = []
    camera_packets = defaultdict(list)
    image_starts = defaultdict(list)
    controls = []
    for frame, start in zip(wire, starts, strict=True):
        assert frame[:8] == b"\x55" * 7 + b"\xd5"
        body = frame[8:-4]
        assert frame[-4:] == zlib.crc32(body).to_bytes(4, "little")
        assert body[:14] == bytes.fromhex("ffffffffffff024c555400010800")
        ipv4 = body[14:34]
        assert ipv4[:1] == b"\x45" and ipv4[9] == 17
        checksum = sum(int.from_bytes(ipv4[i:i + 2], "big") for i in range(0, 20, 2))
        while checksum >> 16:
            checksum = (checksum & 0xffff) + (checksum >> 16)
        assert checksum == 0xffff
        ip_length = int.from_bytes(ipv4[2:4], "big")
        source_port, destination_port, udp_length, udp_checksum = struct.unpack(">4H", body[34:42])
        assert destination_port == 6000 and udp_checksum == 0
        assert ip_length == 20 + udp_length
        payload = body[42:42 + udp_length - 8]
        assert body[42 + len(payload):] == b"\0" * max(0, 60 - 42 - len(payload))
        payloads.append(payload)
        if source_port == 6006:
            controls.append(payload)
            continue
        assert source_port == 6000
        magic, info, frame_id, sequence, offset, length, flags, reserved = struct.unpack(">8I", payload[:32])
        assert magic == 0x4c555443 and info >> 24 == 0 and info & 255 == 32
        camera, product = (info >> 16) & 255, (info >> 8) & 255
        assert camera in (0, 1) and product in (1, 2, 3)
        assert flags == reserved == 0 and length == len(payload) - 32
        camera_packets[camera, product].append((frame_id, sequence, offset, payload[32:]))
        if product == 1:
            image_starts[camera].append(start)
    assert received == payloads
    assert controls == [b"".join((0x4c525443 + word + packet * 256).to_bytes(4, "little")
                                 for word in range(20)) for packet in range(12)]
    for camera in range(2):
        images = camera_packets[camera, 1]
        assert len(images) == 8
        assert [(frame, sequence, offset) for frame, sequence, offset, _ in images] == [
            (19, sequence, sequence * 1024) for sequence in range(8)]
        assert [len(body) for _, _, _, body in images] == [1024] * 7 + [2]
        assert b"".join(body for _, _, _, body in images) == _image(camera)
        children = camera_packets[camera, 2]
        assert [(frame, sequence, offset, len(body)) for frame, sequence, offset, body in children] == [
            (19, 0, 0, 288), (20, 1, 0, 288)]
        assert [body for _, _, _, body in children] == [
            b"".join(_children(camera, group) for group in range(start, start + 4))
            for start in (0, 4)]
        poses = camera_packets[camera, 3]
        assert poses == [(19, 0, 0, _pose(camera, 0) + _pose(camera, 1))]
    service_cycles = {str(camera): [(second - first) * 120 / 125
                                   for first, second in zip(times, times[1:])]
                      for camera, times in image_starts.items()}
    assert all(max(intervals) < 4000 for intervals in service_cycles.values()), service_cycles
    application = [tuple(map(int, line.split()))
                   for line in (tmp_path / "packet_starts.txt").read_text().splitlines()]
    # First-valid is recorded separately from first-accepted, so this measures
    # the planner/lookup gap without folding network backpressure into it.
    planner_gaps = [following[0] - previous[3] - 1
                    for previous, following in zip(application, application[1:])]
    assert len(application) == 22
    assert all(0 < gap <= 20 for gap in planner_gaps), planner_gaps
    report = {"wire_packets": len(wire), "camera_packets": sum(map(len, camera_packets.values())),
              "control_packets": len(controls), "image_service_sys_cycles": service_cycles,
              "sequence_lookup_cycles": 10, "planner_nonoverlap_sys_cycles": planner_gaps,
              "max_child_records": 4, "max_pose_records": 2,
              "image_slots_per_camera": 8, "child_slots_per_camera": 8, "pose_slots_per_camera": 2}
    (tmp_path / "report.json").write_text(json.dumps(report, indent=2) + "\n")
