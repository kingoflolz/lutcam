from efinix_test_models import SPECIAL_OVERRIDES
"""Full UDP/IP and MAC wire correctness at 120 MHz sys / 125 MHz Ethernet.

Both supported network widths retain 32-bit application and Etherbone ports.
This supplied-stream test measures network service; camera production timing and
competition from actual control requests are checked separately.
"""

from pathlib import Path
import shutil
import subprocess
import struct
import sys
from types import SimpleNamespace
import zlib

import pytest
from migen import ClockDomain, Module
from migen.fhdl import verilog
from litex.soc.interconnect import stream
from liteeth.common import eth_phy_description
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))


class _Phy(Module):
    dw = 8

    def __init__(self):
        self.sink = stream.Endpoint(eth_phy_description(8))
        self.source = stream.Endpoint(eth_phy_description(8))


class _UdpLoopback(Module):
    def __init__(self, core_dw):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.clock_domains.cd_eth_tx = ClockDomain("eth_tx")
        self.clock_domains.cd_eth_rx = ClockDomain("eth_rx")
        self.submodules.phy = _Phy()
        self.submodules.core = core = LiteEthUDPIPCore(
            self.phy, 0x024c55540001, 0xc0a80132, 120_000_000, dw=core_dw,
            with_sys_datapath=False, with_icmp=False, arp_entries=2,
            tx_cdc_depth=64, rx_cdc_depth=64)
        self.submodules.etherbone = etherbone = LiteEthEtherbone(core.udp, 1234, buffer_depth=16)
        # Etherbone remains connected to the real UDP crossbar; the supplied
        # traffic below uses the data port, so its Wishbone host stays idle.
        self.comb += [etherbone.wishbone.bus.ack.eq(1), etherbone.wishbone.bus.dat_r.eq(0)]
        port = core.udp.crossbar.get_port(6000, dw=32)
        assert len(port.sink.data) == len(port.source.data) == 32
        assert len(etherbone.packet.tx.source.data) == len(etherbone.packet.rx.sink.data) == 32
        assert len(etherbone.wishbone.bus.dat_w) == len(etherbone.wishbone.bus.dat_r) == 32
        assert core.udp.crossbar.dw == core_dw
        assert self.phy.dw == 8
        self.mac = SimpleNamespace(sink=port.sink, source=port.source)
        self.comb += [port.sink.src_port.eq(6000), port.sink.dst_port.eq(6000),
                      port.sink.ip_address.eq(0xc0a801ff)]
        # Physical Ethernet cannot be backpressured. Consumer stalls are
        # absorbed by the receive CDC, while the transmit CDC throttles sys.
        self.comb += [
            self.phy.sink.ready.eq(1),
            self.phy.sink.connect(self.phy.source, omit={"ready"}),
        ]
        self.ios = {
            self.cd_sys.clk, self.cd_sys.rst,
            self.cd_eth_tx.clk, self.cd_eth_tx.rst,
            self.cd_eth_rx.clk, self.cd_eth_rx.rst,
        }
        for prefix, endpoint, fields in (
            ("tx", self.mac.sink, ("valid", "ready", "data", "last", "last_be", "length")),
            ("rx", self.mac.source, ("valid", "ready", "data", "last", "last_be", "error")),
            ("wire", self.phy.sink, ("valid", "data", "last")),
        ):
            for name in fields:
                signal = getattr(endpoint, name)
                signal.name_override = prefix + "_" + name
                self.ios.add(signal)


@pytest.mark.skipif(shutil.which("verilator") is None, reason="Verilator required")
@pytest.mark.parametrize("core_dw", [32, 16])
def test_udp_wire_crc_checksum_and_cdc_backpressure(tmp_path, core_dw):
    d = tmp_path
    # Exercise datapath throughput with loopback; complete-frame admission and
    # its busy-slot drops are qualified by test_frame_gate_network.
    from unittest.mock import patch
    with patch("liteeth.mac.frame_gate.ValidatedEthernetFrameGate",
               lambda: stream.Buffer(eth_phy_description(16))):
        top = _UdpLoopback(core_dw)
    rtl = verilog.convert(top, ios=top.ios, name="mac_probe", special_overrides=SPECIAL_OVERRIDES)
    (d / "dut.v").write_text(rtl.main_source)
    for name, data in rtl.data_files.items():
        (d / name).write_text(data)
    # Short traffic exercises padding and every final byte lane. The last
    # two seven-packet rounds model two images/children/poses plus a large
    # control-sized datagram, with every source available immediately.
    frames = [bytes((j * 17 + n) & 255 for j in range(n))
              for n in (14, 60, 61, 62, 63, 64, 1024, 1056, 1450,
                        1056, 104, 120, 1056, 104, 120, 1040,
                        1056, 104, 120, 1056, 104, 120, 1040, 1056)]
    # Compact image tails include a header-only FINAL and a final half-word.
    # Append them after the service rounds so their measured indices stay fixed.
    for frame_id, (offset, body_length) in enumerate(((1024, 0), (0, 2), (0, 1022))):
        header = struct.pack(
            ">8I", 0x4c555443, 0x02000120, frame_id, 0,
            offset, body_length, 1, 0)
        frames.append(header + bytes((j * 17 + body_length) & 255
                                     for j in range(body_length)))
    # Two worst-size batched data rounds, with one bounded 80-byte control reply
    # between image visits. Child groups are fixed nine-tuples, without a
    # record prefix; the two accepted pose records retain their native format.
    batch_start = len(frames)
    child_body = b"".join((0x100000001 + j * 2).to_bytes(8, "big") for j in range(36))
    pose_words = list(range(22))
    pose_words[1], pose_words[8] = 0x80000000, 0x00000001
    pose_body = struct.pack(">22I", *pose_words) * 2
    for sequence in range(2):
        for camera in range(2):
            for product, body in enumerate((bytes(range(256)) * 4, child_body, pose_body), 1):
                frames.append(struct.pack(
                    ">8I", 0x4c555443, 0x03000020 | camera << 16 | product << 8,
                    42, sequence, sequence * 1024 if product == 1 else 0,
                    len(body), 0, 0) + body)
        frames.append(bytes(range(80)))
    frames.append(struct.pack(">8I", 0x4c555443, 0x03000120, 42, 2, 2048, 2, 0, 0) + b"\x12\x34")
    beats = []
    for frame in frames:
        for i in range(0, len(frame), 4):
            word = frame[i:i + 4]
            last = i + 4 >= len(frame)
            last_be = (1 << (len(word) - 1)) if last else 0
            beats.append((len(frame) << 37) | (last << 36) | (last_be << 32) | int.from_bytes(word, "little"))
    (d / "beats.hex").write_text("".join(f"{beat:014x}\n" for beat in beats))
    tb = '''`timescale 1ps/1ps
module tb;
reg sys_clk=0,eth_tx_clk=0,eth_rx_clk=0;
always #4167 sys_clk=~sys_clk; // 120 MHz, rounded to simulator precision.
always #4000 eth_tx_clk=~eth_tx_clk; // 125 MHz.
initial begin #1000;forever #4000 eth_rx_clk=~eth_rx_clk;end
reg sys_rst=1,eth_tx_rst=1,eth_rx_rst=1;
reg tx_valid=0,tx_last=0;reg [15:0]tx_length=0; reg [31:0]tx_data=0;reg [3:0]tx_last_be=0;
wire tx_ready;reg rx_ready=0;wire rx_valid,rx_last;wire [31:0]rx_data;wire [3:0]rx_last_be,rx_error;
wire wire_valid,wire_last;wire [7:0]wire_data;
mac_probe dut(.*);
reg [52:0]beats[0:BEAT_COUNT-1];integer txf,rxf,sf,i,cycle=0,wire_cycle=0,gap=0,seen=0,tx_stalls=0,rx_stalls=0;
reg in_packet=0;
always @(negedge sys_clk) begin
 cycle=cycle+1;
 rx_ready=(cycle%13)>=3;
end
always @(posedge sys_clk) if(!sys_rst) begin
 if(tx_valid && !tx_ready) tx_stalls=tx_stalls+1;
 if(rx_valid && !rx_ready) rx_stalls=rx_stalls+1;
end
always @(posedge sys_clk) if(!sys_rst && rx_valid && rx_ready) begin
 if(rx_error) $fatal(1,"RX CRC/error flagged");
 $fwrite(rxf,"%08x %01x %01x\\n",rx_data,rx_last,rx_last_be);
end
always @(posedge eth_tx_clk) if(!eth_tx_rst) begin
 wire_cycle=wire_cycle+1;
 if(wire_valid) begin
  if(!in_packet) $fwrite(sf,"%0d\\n",wire_cycle);
  if(!in_packet && seen && gap<12) $fatal(1,"Short IFG");
  $fwrite(txf,"%02x",wire_data);
  in_packet=1;gap=0;
  if(wire_last) begin $fwrite(txf,"\\n");in_packet=0;seen=1;end
 end else begin
  if(in_packet) $fatal(1,"Wire underflow");
  gap=gap+1;
 end
end
initial begin
 txf=$fopen("wire.txt","w");rxf=$fopen("rx.txt","w");sf=$fopen("starts.txt","w");
 $readmemh("beats.hex",beats);
 repeat(8) @(negedge sys_clk);
 sys_rst=0;eth_tx_rst=0;eth_rx_rst=0;
 for(i=0;i<BEAT_COUNT;i=i+1) begin
  @(negedge sys_clk);tx_valid=1;tx_data=beats[i][31:0];tx_last_be=beats[i][35:32];tx_last=beats[i][36];tx_length=beats[i][52:37];
  @(posedge sys_clk);while(!tx_ready) @(posedge sys_clk);
 end
 @(negedge sys_clk);tx_valid=0;
 repeat(10000) @(posedge sys_clk);
 if(tx_stalls==0 || rx_stalls==0) $fatal(1,"Backpressure was not exercised");
 $fclose(txf);$fclose(rxf);$fclose(sf);$finish;
end
initial begin #400000000;$fatal(1,"Timeout");end
endmodule
'''.replace("BEAT_COUNT", str(len(beats)))
    (d / "tb.v").write_text(tb)
    for command in (
        ["verilator", "--binary", "--timing", "-Wno-fatal", "--top-module", "tb",
         "-j", "4", "dut.v", "tb.v"],
        [str(d / "obj_dir/Vtb")],
    ):
        result = subprocess.run(command, cwd=d, capture_output=True, text=True, timeout=180)
        assert result.returncode == 0, result.stdout + result.stderr
    expected = frames
    expected_wire = []
    for payload in frames:
        ethernet = bytes.fromhex("ffffffffffff024c555400010800")
        ipv4 = bytearray.fromhex("450000000000000080110000c0a80132c0a801ff")
        ipv4[2:4] = (28 + len(payload)).to_bytes(2, "big")
        checksum = sum(int.from_bytes(ipv4[i:i + 2], "big") for i in range(0, 20, 2))
        while checksum >> 16:
            checksum = (checksum & 0xffff) + (checksum >> 16)
        ipv4[10:12] = (checksum ^ 0xffff).to_bytes(2, "big")
        udp = ((6000).to_bytes(2, "big") * 2
               + (len(payload) + 8).to_bytes(2, "big") + b"\0\0")
        body = ethernet + ipv4 + udp + payload
        body += b"\0" * max(0, 60 - len(body))
        expected_wire.append(b"\x55" * 7 + b"\xd5" + body
                             + zlib.crc32(body).to_bytes(4, "little"))
    wire = [bytes.fromhex(packet) for packet in (d / "wire.txt").read_text().splitlines()]
    assert wire == expected_wire

    received = []
    current = bytearray()
    for line in (d / "rx.txt").read_text().splitlines():
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
    assert received == expected

    starts = [int(cycle) for cycle in (d / "starts.txt").read_text().splitlines()]
    service_cycles = [(starts[i + 7] - starts[i]) * 120 / 125 for i in (9, 16)]
    assert max(service_cycles) < 4000, service_cycles
    batch_service_cycles = [(starts[i + 7] - starts[i]) * 120 / 125
                            for i in (batch_start, batch_start + 7)]
    assert max(batch_service_cycles) < 4000, batch_service_cycles
