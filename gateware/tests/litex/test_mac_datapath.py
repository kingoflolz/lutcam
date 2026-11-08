from efinix_test_models import SPECIAL_OVERRIDES
"""Verify native-byte MAC framing at the 120/125 MHz domain boundary.

The generated LiteEth core is compiled because its CRC network is expensive
for Migen's interpreted simulator. The system-side interface remains 32 bits.
"""

import shutil
import subprocess
import zlib

import pytest
from migen import ClockDomain, Module
from migen.fhdl import verilog
from litex.soc.interconnect import stream
from liteeth.common import eth_phy_description
from liteeth.mac.core import LiteEthMACCore


class _Phy(Module):
    dw = 8

    def __init__(self):
        self.sink = stream.Endpoint(eth_phy_description(8))
        self.source = stream.Endpoint(eth_phy_description(8))


class _MacLoopback(Module):
    def __init__(self, sys_datapath):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.clock_domains.cd_eth_tx = ClockDomain("eth_tx")
        self.clock_domains.cd_eth_rx = ClockDomain("eth_rx")
        self.submodules.phy = _Phy()
        self.submodules.mac = LiteEthMACCore(
            self.phy, 32, with_sys_datapath=sys_datapath,
            tx_cdc_depth=64, rx_cdc_depth=64,
        )
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
            ("tx", self.mac.sink, ("valid", "ready", "data", "last", "last_be")),
            ("rx", self.mac.source, ("valid", "ready", "data", "last", "last_be", "error")),
            ("wire", self.phy.sink, ("valid", "data", "last")),
        ):
            for name in fields:
                signal = getattr(endpoint, name)
                signal.name_override = prefix + "_" + name
                self.ios.add(signal)


@pytest.mark.skipif(shutil.which("verilator") is None, reason="Verilator required")
@pytest.mark.parametrize("sys_datapath", [False, True])
def test_mac_wire_crc_padding_and_cdc_backpressure(tmp_path, sys_datapath):
    d = tmp_path
    top = _MacLoopback(sys_datapath)
    rtl = verilog.convert(top, ios=top.ios, name="mac_probe", special_overrides=SPECIAL_OVERRIDES)
    (d / "dut.v").write_text(rtl.main_source)
    for name, data in rtl.data_files.items():
        (d / name).write_text(data)
    # Short control traffic exercises padding; 61..64 covers every final
    # byte lane. 1098 is the complete MAC body of a 1 KiB image datagram.
    frames = [bytes((j * 17 + n) & 255 for j in range(n))
              for n in (14, 60, 61, 62, 63, 64, 1024, 1098, 1450)]
    beats = []
    for frame in frames:
        for i in range(0, len(frame), 4):
            word = frame[i:i + 4]
            last = i + 4 >= len(frame)
            last_be = (1 << (len(word) - 1)) if last else 0
            beats.append((last << 36) | (last_be << 32) | int.from_bytes(word, "little"))
    (d / "beats.hex").write_text("".join(f"{beat:010x}\n" for beat in beats))
    tb = '''`timescale 1ps/1ps
module tb;
reg sys_clk=0,eth_tx_clk=0,eth_rx_clk=0;
always #4167 sys_clk=~sys_clk; // 120 MHz, rounded to simulator precision.
always #4000 eth_tx_clk=~eth_tx_clk; // 125 MHz.
initial begin #1000;forever #4000 eth_rx_clk=~eth_rx_clk;end
reg sys_rst=1,eth_tx_rst=1,eth_rx_rst=1;
reg tx_valid=0,tx_last=0; reg [31:0]tx_data=0;reg [3:0]tx_last_be=0;
wire tx_ready;reg rx_ready=0;wire rx_valid,rx_last;wire [31:0]rx_data;wire [3:0]rx_last_be,rx_error;
wire wire_valid,wire_last;wire [7:0]wire_data;
mac_probe dut(.*);
reg [36:0]beats[0:BEAT_COUNT-1];integer txf,rxf,i,cycle=0,wire_cycle=0,gap=0,seen=0,tx_stalls=0,rx_stalls=0;
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
 txf=$fopen("wire.txt","w");rxf=$fopen("rx.txt","w");
 $readmemh("beats.hex",beats);
 repeat(8) @(negedge sys_clk);
 sys_rst=0;eth_tx_rst=0;eth_rx_rst=0;
 for(i=0;i<BEAT_COUNT;i=i+1) begin
  @(negedge sys_clk);tx_valid=1;tx_data=beats[i][31:0];tx_last_be=beats[i][35:32];tx_last=beats[i][36];
  @(posedge sys_clk);while(!tx_ready) @(posedge sys_clk);
 end
 @(negedge sys_clk);tx_valid=0;
 repeat(10000) @(posedge sys_clk);
 if(tx_stalls==0 || rx_stalls==0) $fatal(1,"Backpressure was not exercised");
 $fclose(txf);$fclose(rxf);$finish;
end
initial begin #200000000;$fatal(1,"Timeout");end
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
    expected = [payload + b"\0" * max(0, 60 - len(payload)) for payload in frames]
    expected_wire = [b"\x55" * 7 + b"\xd5" + payload + zlib.crc32(payload).to_bytes(4, "little")
                     for payload in expected]
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
