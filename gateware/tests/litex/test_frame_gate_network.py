"""Real MAC/IPv4/UDP/Etherbone traffic must wait for complete frame validation."""

from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import zlib

import pytest
from migen import ClockDomain, Module
from migen.fhdl import verilog
from migen.sim import passive, run_simulation
from litex.soc.interconnect import stream
from litex.tools.remote.etherbone import EtherbonePacket, EtherboneReads, EtherboneRecord, EtherboneWrites
from liteeth.common import eth_phy_description
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))
from lutcam.control import WishboneGuard
from liteeth.mac.frame_gate import ValidatedEthernetFrameGate
from liteeth.core.arp import LiteEthARPCache
from liteeth.core.efinix import DspWaitTimer


BOARD_MAC = 0x024C55540001
BOARD_IP = 0xC0A80132
PEER_MAC = 0x021122334455
PEER_IP = 0xC0A801FB
UDP_PORT = 1234
PEER_PORT = 40000
BASE = 0x30025100
RETURN_ADDRESS = 0x12345670
PREAMBLE = b"\x55" * 7 + b"\xd5"
BROADCAST_MAC = 0xFFFFFFFFFFFF


def checksum16(data):
    """Internet checksum, including end-around carries and odd byte padding."""
    data = bytes(data)
    data += bytes(len(data) & 1)
    total = sum(struct.unpack(f">{len(data)//2}H", data))
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def ethernet_frame(payload, ethertype, *, source_mac=PEER_MAC, destination_mac=BOARD_MAC):
    body = destination_mac.to_bytes(6, "big") + source_mac.to_bytes(6, "big")
    body += struct.pack(">H", ethertype) + bytes(payload)
    body += bytes(max(0, 60 - len(body)))
    return PREAMBLE + body + struct.pack("<I", zlib.crc32(body))


def strip_wire(frame, *, check_fcs=True):
    frame = bytes(frame)
    assert frame.startswith(PREAMBLE), "Missing exact Ethernet preamble/SFD"
    assert len(frame) >= 72, "Runt Ethernet wire frame"
    body, fcs = frame[8:-4], frame[-4:]
    if check_fcs:
        assert fcs == struct.pack("<I", zlib.crc32(body)), "Bad Ethernet FCS"
    return body


def udp_frame(payload, *, source_port=PEER_PORT, destination_port=UDP_PORT,
              source_ip=PEER_IP, destination_ip=BOARD_IP,
              source_mac=PEER_MAC, destination_mac=BOARD_MAC,
              identification=0x1001, udp_checksum=False):
    payload = bytes(payload)
    udp_length = 8 + len(payload)
    assert udp_length + 20 <= 0xFFFF
    udp = struct.pack(">HHHH", source_port, destination_port, udp_length, 0) + payload
    if udp_checksum:
        pseudo = struct.pack(">IIBBH", source_ip, destination_ip, 0, 17, udp_length)
        value = checksum16(pseudo + udp) or 0xFFFF
        udp = udp[:6] + struct.pack(">H", value) + udp[8:]
    ip = struct.pack(">BBHHHBBHII", 0x45, 0, 20 + udp_length,
                     identification, 0x4000, 64, 17, 0, source_ip, destination_ip)
    ip = ip[:10] + struct.pack(">H", checksum16(ip)) + ip[12:]
    return ethernet_frame(ip + udp, 0x0800, source_mac=source_mac, destination_mac=destination_mac)


def etherbone_payload(*, writes=None, write_base=BASE, reads=None,
                      return_address=RETURN_ADDRESS, byte_enable=15,
                      probe=False, probe_payload=b"\0\0\0\0"):
    packet = EtherbonePacket()
    if probe:
        assert writes is None and reads is None
        assert len(probe_payload) == 4, "Pinned LiteEth probe FIFO holds one 32-bit payload word"
        packet.pf = 1
        packet.encode()
        return bytes(packet.bytes) + bytes(probe_payload)
    assert writes is not None or reads is not None
    record = EtherboneRecord()
    record.byte_enable = byte_enable
    if writes is not None:
        writes = list(writes)
        assert 1 <= len(writes) <= 255 and write_base % 4 == 0
        record.writes = EtherboneWrites(base_addr=write_base, datas=writes)
    if reads is not None:
        reads = list(reads)
        assert 1 <= len(reads) <= 255 and all(address % 4 == 0 for address in reads)
        record.reads = EtherboneReads(base_ret_addr=return_address, addrs=reads)
    packet.records = [record]
    packet.encode()
    return bytes(packet.bytes)


def etherbone_request(*, writes=None, write_base=BASE, reads=None,
                      return_address=RETURN_ADDRESS, byte_enable=15, probe=False,
                      probe_payload=b"\0\0\0\0", source_port=PEER_PORT,
                      identification=0x1001, udp_checksum=False):
    payload = etherbone_payload(writes=writes, write_base=write_base, reads=reads,
        return_address=return_address, byte_enable=byte_enable,
        probe=probe, probe_payload=probe_payload)
    return udp_frame(payload, source_port=source_port, identification=identification,
                     udp_checksum=udp_checksum)


def read_request(addresses, *, return_address=RETURN_ADDRESS, **kwargs):
    return etherbone_request(reads=list(addresses), return_address=return_address, **kwargs)


def write_request(base, values, **kwargs):
    return etherbone_request(write_base=base, writes=list(values), **kwargs)


def probe_request(**kwargs):
    return etherbone_request(probe=True, **kwargs)


def fake_read_data(address_word):
    return (address_word ^ 0x5A5A0000) & 0xFFFFFFFF


def expected_read_values(byte_addresses):
    assert all(address % 4 == 0 for address in byte_addresses)
    return [fake_read_data(address // 4) for address in byte_addresses]


def expected_write_operations(base, values, *, byte_enable=15):
    """Expected bus tuples: (we, word_address, dat_w, sel)."""
    assert base % 4 == 0
    return [(1, base // 4 + index, value, byte_enable) for index, value in enumerate(values)]


def arp_frame(opcode, *, source_mac=PEER_MAC, source_ip=PEER_IP,
              target_mac=None, target_ip=BOARD_IP, destination_mac=None):
    assert opcode in (1, 2)
    if target_mac is None:
        target_mac = 0 if opcode == 1 else BOARD_MAC
    if destination_mac is None:
        destination_mac = BROADCAST_MAC if opcode == 1 else target_mac
    arp = struct.pack(">HHBBH", 1, 0x0800, 6, 4, opcode)
    arp += source_mac.to_bytes(6, "big") + struct.pack(">I", source_ip)
    arp += target_mac.to_bytes(6, "big") + struct.pack(">I", target_ip)
    return ethernet_frame(arp, 0x0806, source_mac=source_mac, destination_mac=destination_mac)


def arp_request(**kwargs):
    return arp_frame(1, **kwargs)


def arp_reply(**kwargs):
    return arp_frame(2, **kwargs)


def parse_frame(frame):
    """Validate wire checksums, strip padding using IP/UDP lengths, decode EB.

    Returns a dictionary with kind='arp', 'etherbone', or 'udp'. Probe payload
    is deliberately kept separate: it is opaque, not an Etherbone record.
    """
    body = strip_wire(frame)
    result = {"destination_mac": int.from_bytes(body[:6], "big"),
              "source_mac": int.from_bytes(body[6:12], "big"),
              "ethertype": int.from_bytes(body[12:14], "big"), "body": body}
    payload = body[14:]
    if result["ethertype"] == 0x0806:
        assert len(payload) >= 28
        hardware, protocol, hlen, plen, opcode = struct.unpack(">HHBBH", payload[:8])
        assert (hardware, protocol, hlen, plen) == (1, 0x0800, 6, 4)
        result.update(kind="arp", opcode=opcode,
            sender_mac=int.from_bytes(payload[8:14], "big"),
            sender_ip=int.from_bytes(payload[14:18], "big"),
            target_mac=int.from_bytes(payload[18:24], "big"),
            target_ip=int.from_bytes(payload[24:28], "big"))
        return result
    assert result["ethertype"] == 0x0800, "Unexpected Ethernet type"
    assert len(payload) >= 20 and payload[0] >> 4 == 4
    ihl = (payload[0] & 15) * 4
    total_length = int.from_bytes(payload[2:4], "big")
    assert 20 <= ihl <= total_length <= len(payload)
    assert checksum16(payload[:ihl]) == 0, "Bad IPv4 header checksum"
    assert int.from_bytes(payload[6:8], "big") & 0x3FFF == 0, "Fragmented IPv4 packet"
    assert payload[9] == 17, "Expected UDP"
    source_ip, destination_ip = struct.unpack(">II", payload[12:20])
    udp = payload[ihl:total_length]
    assert len(udp) >= 8
    source_port, destination_port, length, udp_checksum = struct.unpack(">HHHH", udp[:8])
    assert 8 <= length == len(udp), "IPv4/UDP length mismatch"
    if udp_checksum:
        pseudo = struct.pack(">IIBBH", source_ip, destination_ip, 0, 17, length)
        assert checksum16(pseudo + udp) == 0, "Bad UDP checksum"
    result.update(kind="udp", source_ip=source_ip, destination_ip=destination_ip,
                  source_port=source_port, destination_port=destination_port,
                  udp_payload=udp[8:])
    if source_port != UDP_PORT and destination_port != UDP_PORT:
        return result
    eb = udp[8:]
    assert len(eb) >= 8
    # Use the installed independent host decoder for the packet and records.
    header = EtherbonePacket(init=eb[:8]); header.decode()
    assert (header.magic, header.version, header.addr_size, header.port_size) == (0x4E6F, 1, 4, 4)
    if header.pf or header.pr:
        packet = header
        result["probe_payload"] = eb[8:]
    else:
        packet = EtherbonePacket(init=eb); packet.decode()
    result.update(kind="etherbone", etherbone=packet)
    return result


def reply_to_arp(request):
    packet = parse_frame(request)
    assert packet["kind"] == "arp" and packet["opcode"] == 1
    assert packet["target_ip"] == PEER_IP
    return arp_reply(target_mac=packet["sender_mac"], target_ip=packet["sender_ip"])


def _check_reply_route(packet, source_port):
    assert packet["kind"] == "etherbone"
    assert (packet["source_mac"], packet["destination_mac"]) == (BOARD_MAC, PEER_MAC)
    assert (packet["source_ip"], packet["destination_ip"]) == (BOARD_IP, PEER_IP)
    assert (packet["source_port"], packet["destination_port"]) == (UDP_PORT, source_port)


def assert_read_response(frame, addresses, *, return_address=RETURN_ADDRESS, source_port=PEER_PORT):
    result = parse_frame(frame)
    _check_reply_route(result, source_port)
    packet = result["etherbone"]
    assert (packet.pf, packet.pr) == (0, 0) and len(packet.records) == 1
    record = packet.records[0]
    assert (record.wcount, record.rcount, record.byte_enable) == (len(addresses), 0, 15)
    assert record.writes.base_addr == return_address
    assert record.writes.get_datas() == expected_read_values(addresses)
    return result


def assert_probe_response(frame, *, source_port=PEER_PORT, probe_payload=b"\0\0\0\0"):
    result = parse_frame(frame)
    _check_reply_route(result, source_port)
    assert (result["etherbone"].pf, result["etherbone"].pr) == (0, 1)
    assert result["probe_payload"] == probe_payload
    return result


def write_hex(path, frame):
    """Write one byte per line for an 8-bit Verilog $readmemh array."""
    Path(path).write_text("".join(f"{byte:02x}\n" for byte in frame))


class _WirePHY(Module):
    dw = 8

    def __init__(self):
        self.sink = stream.Endpoint(eth_phy_description(8))
        self.source = stream.Endpoint(eth_phy_description(8))


class _ValidatedNetwork(Module):
    def __init__(self):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.clock_domains.cd_eth_tx = ClockDomain("eth_tx")
        self.clock_domains.cd_eth_rx = ClockDomain("eth_rx")
        self.submodules.phy = phy = _WirePHY()
        self.submodules.core = core = LiteEthUDPIPCore(
            phy, BOARD_MAC, BOARD_IP, 120_000_000, dw=16,
            with_sys_datapath=False, with_icmp=False, arp_entries=2,
            tx_cdc_depth=64, rx_cdc_depth=64)
        self.comb += core.arp.table.cache.clear_enable.eq(0)
        self.submodules.etherbone = eb = LiteEthEtherbone(core.udp, UDP_PORT, buffer_depth=256)
        observed_rx = core.mac.checked_frame_gate.source
        self.submodules.guard = guard = WishboneGuard()
        self.comb += [eb.wishbone.bus.connect(guard.bus), phy.sink.ready.eq(1)]
        self.ios = {self.cd_sys.clk, self.cd_sys.rst,
                    self.cd_eth_rx.clk, self.cd_eth_rx.rst,
                    self.cd_eth_tx.clk, self.cd_eth_tx.rst}
        for prefix, interface, fields in (
            ("wire_rx", phy.source, ("valid", "ready", "data", "first", "last", "last_be", "error")),
            ("wire_tx", phy.sink, ("valid", "data", "last")),
            ("wb", guard.master, ("cyc", "stb", "we", "adr", "dat_w", "sel", "ack", "dat_r")),
            ("gate", observed_rx, ("valid", "ready", "data", "first", "last", "last_be", "error")),
        ):
            for name in fields:
                signal = getattr(interface, name)
                signal.name_override = prefix + "_" + name
                self.ios.add(signal)
        guard.fault.name_override = "guard_fault"
        self.ios.add(guard.fault)


def _corrupt(frame, offset):
    result = bytearray(frame)
    result[offset] ^= 0x40
    return bytes(result)


def _oversize_valid_fcs(frame):
    body = strip_wire(frame)
    body += b"\x9d" * (2200 - len(body))
    return PREAMBLE + body + struct.pack("<I", zlib.crc32(body))


def _network_cases():
    values = [(0xa5f00000 + index * 0x10203) & 0xffffffff for index in range(128)]
    addresses = [BASE + 4 * index for index in range(128)]
    big_write = write_request(BASE, values)
    probe = probe_request(probe_payload=b"test")
    expected = []
    cases = []

    def add(name, wire, operations=(), reply=None):
        expected.extend(operations)
        cases.append({"name": name, "wire": wire, "count": len(expected), "reply": reply})

    # The first probe causes an ARP lookup. The following solicited reply
    # supplies the real cache, so every subsequent response uses the MAC TX.
    add("initial probe and ARP request", probe)
    add("solicited ARP reply", arp_reply(), reply=("probe", b"test"))
    add("128-word write", big_write, expected_write_operations(BASE, values))
    add("128-word read", read_request(addresses),
        [(0, address // 4, fake_read_data(address // 4), 15) for address in addresses],
        reply=("read", addresses))
    mixed_writes, mixed_reads = [0x11111111, 0x22222222, 0x33333333], addresses[13:18]
    add("combined write and read", etherbone_request(writes=mixed_writes, reads=mixed_reads),
        expected_write_operations(BASE, mixed_writes) +
        [(0, address // 4, fake_read_data(address // 4), 15) for address in mixed_reads],
        reply=("read", mixed_reads))
    add("bad final FCS", _corrupt(big_write, -1))
    add("bad payload and old FCS", _corrupt(big_write, 8 + 14 + 20 + 8 + 24))
    # Probe UDP data ends at Ethernet byte 54: the final six bytes are pad.
    # The bad pad/FCS is therefore downstream of the declared UDP payload.
    assert len(strip_wire(probe)) == 60
    add("bad short-frame padding", _corrupt(probe, 8 + 58))
    add("bad short-frame FCS", _corrupt(probe, -2))
    runt = strip_wire(probe)[:54]
    add("runt with valid FCS", PREAMBLE + runt + struct.pack("<I", zlib.crc32(runt)))
    odd_write = write_request(BASE + 0x300, [0x81828384])
    odd_body = strip_wire(odd_write) + b"\x91"
    odd_write = PREAMBLE + odd_body + struct.pack("<I", zlib.crc32(odd_body))
    add("odd final lane valid write", odd_write, expected_write_operations(BASE + 0x300, [0x81828384]))
    add("odd final lane invalid FCS", _corrupt(odd_write, -1))
    add("truncated long write", big_write[:-37])
    add("truncated FCS", big_write[:-2])
    add("oversize with valid FCS", _oversize_valid_fcs(big_write))
    for writes, reads, extra in (([1, 2, 3], None, 1), ([1, 2, 3], None, -1),
                                 (None, addresses[:3], 1), ([1, 2], addresses[:2], 1)):
        payload = bytearray(etherbone_payload(writes=writes, reads=reads))
        count_byte = 10 if writes is not None else 11
        payload[count_byte] += extra
        add(f"record count/UDP length mismatch {len(cases)}", udp_frame(payload))
    recovery_values = [0x01234567, 0x89abcdef, 0x11223344]
    add("recovery write", write_request(BASE + 0x400, recovery_values),
        expected_write_operations(BASE + 0x400, recovery_values))
    add("recovery read", read_request(addresses[:3]),
        [(0, address // 4, fake_read_data(address // 4), 15) for address in addresses[:3]],
        reply=("read", addresses[:3]))
    add("recovery probe", probe_request(), reply=("probe", bytes(4)))
    add("slow replay before overlapping frame", big_write, expected_write_operations(BASE, values))
    cases[-1].update(wait=20, check=False)
    # The physical receiver cannot pause. An incoming frame while the sole
    # replay slot is owned must be discarded completely, even if its bytes
    # themselves have a valid FCS.
    add("valid frame arriving during replay is dropped", write_request(BASE + 0x700, [0xbad0bad0]))
    add("recovery after occupied slot", write_request(BASE + 0x704, [0xc001cafe]),
        expected_write_operations(BASE + 0x704, [0xc001cafe]))
    return cases, expected


_WIRE_BENCH = r"""`timescale 1ps/1ps
module tb;
reg sys_clk=0,eth_tx_clk=0,eth_rx_clk=0;
always #4167 sys_clk=~sys_clk;
always #4000 eth_tx_clk=~eth_tx_clk;
initial begin #1000;forever #4000 eth_rx_clk=~eth_rx_clk;end
reg sys_rst=1,eth_tx_rst=1,eth_rx_rst=1;
reg wire_rx_valid=0,wire_rx_first=0,wire_rx_last=0,wire_rx_last_be=0,wire_rx_error=0;
reg[7:0]wire_rx_data=0;wire wire_rx_ready;
wire wire_tx_valid,wire_tx_last;wire[7:0]wire_tx_data;
wire wb_cyc,wb_stb,wb_we;wire[29:0]wb_adr;wire[31:0]wb_dat_w;wire[3:0]wb_sel;
reg wb_ack=0;reg[31:0]wb_dat_r=0;
wire gate_valid,gate_ready,gate_first,gate_last;wire[15:0]gate_data;
wire[1:0]gate_last_be,gate_error;wire guard_fault;
network_probe dut(.*);
reg[7:0]input_bytes[0:BYTE_COUNT-1];
integer offsets[0:CASE_COUNT],expected_count[0:CASE_COUNT-1],wait_cycles[0:CASE_COUNT-1];
integer input_case=0,i,wb_count=0,wb_delay=0,wb_stalls=0,gate_stalls=0;
integer wb_file,wire_file,case_file;reg tx_packet=0;
reg gate_held=0;reg[21:0]held_gate;
wire[21:0]gate_beat={gate_error,gate_last_be,gate_last,gate_first,gate_data};
always @(posedge sys_clk)if(!sys_rst)begin
 if(guard_fault)$fatal(1,"Wishbone guard timed out");
 if(gate_held&&(!gate_valid||gate_beat!==held_gate))$fatal(1,"Gate changed a stalled beat");
 gate_held<=gate_valid&&!gate_ready;held_gate<=gate_beat;
 if(gate_valid&&!gate_ready)gate_stalls=gate_stalls+1;
 wb_ack<=0;
 if(wb_cyc&&wb_stb&&!wb_ack)begin
  if(wb_delay==((wb_count%11==0)?37:7))begin
   wb_ack<=1;wb_dat_r<=wb_adr^32'h5a5a0000;wb_delay=0;
   $fwrite(wb_file,"%0d %0d %08x %08x %x\n",input_case,wb_we,wb_adr,
           wb_we?wb_dat_w:(wb_adr^32'h5a5a0000),wb_sel);
   wb_count=wb_count+1;
  end else begin wb_delay=wb_delay+1;wb_stalls=wb_stalls+1;end
 end
end
always @(posedge eth_tx_clk)if(!eth_tx_rst&&wire_tx_valid)begin
 if(!tx_packet)$fwrite(wire_file,"%0d ",input_case);
 tx_packet=1;$fwrite(wire_file,"%02x",wire_tx_data);
 if(wire_tx_last)begin $fwrite(wire_file,"\n");tx_packet=0;end
end
initial begin
 wb_file=$fopen("wishbone.txt","w");wire_file=$fopen("wire.txt","w");case_file=$fopen("cases.txt","w");
 $readmemh("input.hex",input_bytes);
 CASE_INITIALIZERS
 repeat(8)@(negedge sys_clk);sys_rst=0;eth_tx_rst=0;eth_rx_rst=0;
 for(input_case=0;input_case<CASE_COUNT;input_case=input_case+1)begin
  for(i=offsets[input_case];i<offsets[input_case+1];i=i+1)begin
   @(negedge eth_rx_clk);
   wire_rx_valid=1;wire_rx_data=input_bytes[i];wire_rx_first=(i==offsets[input_case]);
   wire_rx_last=(i==offsets[input_case+1]-1);wire_rx_last_be=wire_rx_last;
  end
  @(negedge eth_rx_clk);wire_rx_valid=0;wire_rx_first=0;wire_rx_last=0;wire_rx_last_be=0;
  repeat(wait_cycles[input_case])@(posedge sys_clk);
  $fwrite(case_file,"%0d %0d\n",input_case,wb_count);
  if(expected_count[input_case]>=0&&wb_count!=expected_count[input_case])
   $fatal(1,"Case %0d issued %0d bus transactions, expected %0d",input_case,wb_count,expected_count[input_case]);
 end
 if(wb_stalls==0||gate_stalls==0)$fatal(1,"Required backpressure did not occur");
 $fclose(wb_file);$fclose(wire_file);$fclose(case_file);
 $display("PASS validated network frames with %0d bus stalls and %0d gate stalls",wb_stalls,gate_stalls);$finish;
end
initial begin #20000000000;$fatal(1,"Simulation timeout");end
endmodule
"""


def test_wire_fcs_validation_precedes_streaming_etherbone_side_effects(tmp_path):
    checked = srl_arp_cache = True
    # Release qualification can require this exact native primitive test,
    # while portable development environments retain an explicit skip.
    require_native = os.environ.get("LUTCAM_REQUIRE_EFINIX_NATIVE") == "1"
    if shutil.which("verilator") is None:
        if srl_arp_cache and require_native:
            pytest.fail("Native network qualification requires Verilator")
        pytest.skip("Verilator required")
    native_sources = []
    native_input_hashes = {}
    if srl_arp_cache:
        efinity = os.environ.get("LITEX_ENV_EFINITY") or os.environ.get("EFINITY_HOME")
        if not efinity:
            message = "Set EFINITY_HOME or LITEX_ENV_EFINITY for native network models"
            if require_native:
                pytest.fail(message)
            pytest.skip(message)
        efinity = Path(efinity).resolve()
        for name in ("efx_srl8.v", "efx_dsp48.v"):
            vendor = efinity / "sim_models/verilog" / name
            if not vendor.is_file():
                message = f"Set LITEX_ENV_EFINITY for the exact {name} vendor model"
                if require_native:
                    pytest.fail(message)
                pytest.skip(message)
            native_sources.append(str(vendor))
        implementation_sources = [Path(__file__).resolve(),
            Path(sys.modules[LiteEthARPCache.__module__].__file__).resolve(),
            Path(sys.modules[DspWaitTimer.__module__].__file__).resolve(),
            Path(sys.modules["liteeth.frontend.etherbone"].__file__).resolve()]
        native_input_hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in [*implementation_sources,
                                            *(Path(name) for name in native_sources)]}
    top = _ValidatedNetwork()
    rtl = verilog.convert(top, ios=top.ios, name="network_probe")
    dsp48_instances = len(re.findall(r"\bEFX_DSP48\s*#\(", rtl.main_source))
    srl8_instances = len(re.findall(r"\bEFX_SRL8\s*#\(", rtl.main_source))
    assert dsp48_instances == (2 if srl_arp_cache else 0)
    assert srl8_instances == (81 if srl_arp_cache else 0)
    (tmp_path / "dut.v").write_text(rtl.main_source)
    for name, contents in rtl.data_files.items():
        (tmp_path / name).write_text(contents)
    cases, expected_operations = _network_cases()
    data = b"".join(case["wire"] for case in cases)
    write_hex(tmp_path / "input.hex", data)
    initializers = ["offsets[0]=0;"]
    offset = 0
    for index, case in enumerate(cases):
        offset += len(case["wire"])
        initializers.extend((f"offsets[{index + 1}]={offset};",
                             f"expected_count[{index}]={case['count'] if case.get('check', True) else -1};",
                             f"wait_cycles[{index}]={case.get('wait', 7000)};"))
    bench = _WIRE_BENCH.replace("BYTE_COUNT", str(len(data))).replace("CASE_COUNT", str(len(cases)))
    bench = bench.replace("CASE_INITIALIZERS", "\n ".join(initializers))
    (tmp_path / "tb.v").write_text(bench)
    for command in (
        ["verilator", "--binary", "--timing", "-Wno-fatal", "--top-module", "tb",
         "-j", "4", "dut.v", "tb.v", *native_sources],
        [str(tmp_path / "obj_dir" / "Vtb")],
    ):
        result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True, timeout=180)
        assert result.returncode == 0, result.stdout + result.stderr + "\n" + repr([c["name"] for c in cases])
    observed = []
    for line in (tmp_path / "wishbone.txt").read_text().splitlines():
        _case, write, address, value, select = line.split()
        observed.append((int(write), int(address, 16), int(value, 16), int(select, 16)))
    assert observed == expected_operations
    responses = {}
    arp_requests = []
    for line in (tmp_path / "wire.txt").read_text().splitlines():
        case, frame = line.split()
        frame = bytes.fromhex(frame)
        parsed = parse_frame(frame)
        if parsed["kind"] == "arp":
            arp_requests.append(parsed)
            assert parsed["opcode"] == 1 and parsed["target_ip"] == PEER_IP
        else:
            responses.setdefault(int(case), []).append(frame)
    assert len(arp_requests) == int(checked)
    for index, case in enumerate(cases):
        returned = responses.get(index, [])
        if case["reply"] is None:
            assert not returned, f"Unexpected response to {case['name']}"
        else:
            assert len(returned) == 1, f"Missing/duplicate response to {case['name']}"
            kind, expected = case["reply"]
            if kind == "probe":
                assert_probe_response(returned[0], probe_payload=expected)
            else:
                assert_read_response(returned[0], expected)

    if srl_arp_cache:
        assert native_input_hashes == {
            path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
            for path in native_input_hashes}
        manifest = {"passed": True, "dsp48_instances": dsp48_instances,
                    "srl8_instances": srl8_instances,
                    "native_models": native_sources,
                    "source_sha256": native_input_hashes,
                    "rtl_sha256": hashlib.sha256(rtl.main_source.encode()).hexdigest(),
                    "cases": len(cases), "bus_operations": len(observed),
                    "wire_frames": len((tmp_path / "wire.txt").read_text().splitlines())}
        (tmp_path / "native-arp-manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n")


def test_frame_gate_capacity_errors_and_stalled_output_recovery():
    dut = ValidatedEthernetFrameGate(dw=16, depth=8)
    received, pending = [], []
    stats = {"stalls": 0, "held": None}

    @passive
    def consume():
        cycle = 0
        while True:
            yield dut.source.ready.eq(cycle % 11 >= 4)
            cycle += 1
            yield
            if not (yield dut.source.valid):
                assert stats["held"] is None
                continue
            beat = ((yield dut.source.data), (yield dut.source.last),
                    (yield dut.source.last_be), (yield dut.source.error))
            if stats["held"] is not None:
                assert beat == stats["held"]
            if not (yield dut.source.ready):
                stats["held"] = beat
                stats["stalls"] += 1
                continue
            stats["held"] = None
            pending.append(beat)
            if beat[1]:
                received.append(tuple(pending))
                pending.clear()

    def send(words, *, error_at=None, final_bytes=2):
        before = len(received)
        for index, word in enumerate(words):
            yield dut.sink.valid.eq(1)
            yield dut.sink.first.eq(index == 0)
            yield dut.sink.last.eq(index == len(words) - 1)
            yield dut.sink.last_be.eq((1 << (final_bytes - 1)) if index == len(words) - 1 else 0)
            yield dut.sink.data.eq(word)
            yield dut.sink.error.eq(1 if index == error_at else 0)
            yield
            for _ in range(200):
                if (yield dut.sink.ready):
                    break
                yield
            else:
                raise AssertionError("Gate did not finish receiving a frame")
            if index < len(words) - 1:
                assert len(received) == before, "Gate emitted an unvalidated frame"
        yield dut.sink.valid.eq(0)
        yield dut.sink.last.eq(0)
        yield dut.sink.error.eq(0)
        for _ in range(70):
            yield

    def bench():
        good = list(range(0x1100, 0x1108))
        yield from send(good)
        assert len(received) == 1
        assert [beat[0] for beat in received[0]] == good
        assert received[0][-1][1:] == (1, 2, 0)
        for words, error_at in ((good, 0), (good, 7), (list(range(9)), None),
                                (list(range(17)), None)):
            before = len(received)
            yield from send(words, error_at=error_at)
            assert len(received) == before, "Bad or oversize frame escaped"
        yield from send([0xbeef], final_bytes=1)
        assert received[-1] == ((0xbeef, 1, 1, 0),)
        yield from send([0x1234, 0x5678, 0x9abc])
        assert [beat[0] for beat in received[-1]] == [0x1234, 0x5678, 0x9abc]
        assert len(received) == 3
        assert not pending
        assert stats["stalls"] > 0

    run_simulation(dut, [bench(), consume()])
