"""Generated RTL must preserve the largest compact image tail's word count."""

import shutil
import struct
import subprocess

import pytest
from migen import ClockDomain, Module
from migen.fhdl import verilog

from fpga.litex.lutcam.packet import SharedPacketTransport


class _TailHarness(Module):
    def __init__(self):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.submodules.transport = transport = SharedPacketTransport(
            compact_padding=True, batched_packets=True,
            external_destination=True, external_context_held=True)
        self.comb += [
            transport.sequence_valid.eq(transport.sequence_request),
            transport.sequence_value.eq(0),
            transport.sequence_destination_ip.eq(0xc0a801fb),
            transport.sequence_destination_port.eq(6003),
        ]
        channel = transport.channels[3]
        self.comb += channel.enable.eq(1)
        self.ios = {self.cd_sys.clk, self.cd_sys.rst}
        for prefix, interface, fields in (
            ("image", channel.sink, ("valid", "data", "sof", "padding", "frame_id")),
            ("udp", transport.source, ("valid", "ready", "data", "last", "last_be", "length")),
            ("queue", channel, ("faults", "fifo_level", "head_words", "head_bytes", "present")),
        ):
            for field in fields:
                signal = getattr(interface, field)
                signal.name_override = prefix + "_" + field
                self.ios.add(signal)


_TESTBENCH = r"""`timescale 1ns/1ps
module tb;
reg sys_clk=0,sys_rst=1;
always #5 sys_clk=~sys_clk;
reg image_valid=0,image_sof=0,image_padding=0;
reg[15:0]image_data=0;reg[31:0]image_frame_id=0;
wire udp_valid,udp_last;reg udp_ready=0;
wire[31:0]udp_data;wire[3:0]udp_last_be;wire[15:0]udp_length;
wire[1:0]queue_faults;wire[3:0]queue_fifo_level;
wire[8:0]queue_head_words;wire[10:0]queue_head_bytes;wire queue_present;
tail_probe dut(.*);
integer cycle=0,packets=0,fd,n,i,prefix;
always @(negedge sys_clk)begin
 cycle=cycle+1;udp_ready=(cycle%7!=1)&&(cycle%7!=4);
end
always @(posedge sys_clk)if(!sys_rst)begin
 if(queue_faults)$fatal(1,"Queue fault");
 if(queue_present && queue_head_bytes==1022 && queue_head_words!=256)
  $fatal(1,"1022-byte tail requires 256 words, got %0d",queue_head_words);
 if(udp_valid&&udp_ready)begin
  $fwrite(fd,"%08x %01x %01x %0d\n",udp_data,udp_last,udp_last_be,udp_length);
  if(udp_last)packets=packets+1;
 end
end
initial begin
 fd=$fopen("beats.txt","w");repeat(8)@(negedge sys_clk);sys_rst=0;
 // Exercise the carry boundary and a subsequent full fragment. Physical
 // encoder padding continues through the board's complete 64000-word frame.
 for(n=0;n<3;n=n+1)begin
  prefix=510+n;
  for(i=0;i<64000;i=i+1)begin
   @(negedge sys_clk);image_valid=1;image_sof=(i==0);image_frame_id=n;
   image_padding=(i>=prefix);image_data=image_padding?0:(16'h1234+i+n);
  end
  @(negedge sys_clk);image_valid=0;image_sof=0;
  repeat(16)@(negedge sys_clk);
  if(packets!=n+1||queue_fifo_level!=0)$fatal(1,"Image tail did not complete");
 end
 $fclose(fd);$display("PASS");$finish;
end
initial begin repeat(200000)@(posedge sys_clk);$fatal(1,"Timeout");end
endmodule
"""


@pytest.mark.skipif(shutil.which("iverilog") is None or shutil.which("vvp") is None,
                    reason="Icarus Verilog required")
def test_compact_tail_rounding_in_generated_rtl(tmp_path):
    top = _TailHarness()
    rtl = verilog.convert(top, ios=top.ios, name="tail_probe")
    (tmp_path / "dut.v").write_text(rtl.main_source)
    for name, content in rtl.data_files.items():
        (tmp_path / name).write_text(content)
    (tmp_path / "tb.v").write_text(_TESTBENCH)
    for command in (
        ["iverilog", "-g2012", "-s", "tb", "-o", "simulate", "dut.v", "tb.v"],
        ["vvp", "simulate"],
    ):
        result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stdout + result.stderr
    packets, pending = [], bytearray()
    for line in (tmp_path / "beats.txt").read_text().splitlines():
        data, last, mask, length = line.split()
        word = int(data, 16).to_bytes(4, "little")
        if int(last, 16):
            mask = int(mask, 16)
            assert mask in (2, 8)
            word = word[:mask.bit_length()]
        pending.extend(word)
        if int(last, 16):
            assert len(pending) == int(length)
            packets.append(bytes(pending))
            pending.clear()
    assert not pending
    expected = []
    for frame, prefix in enumerate((510, 511, 512)):
        header = struct.pack(">8I", 0x4c555443, 0x03010120, frame, 0, 0, prefix * 2, 0, 0)
        payload = b"".join((0x1234 + index + frame).to_bytes(2, "big") for index in range(prefix))
        expected.append(header + payload)
    assert packets == expected
