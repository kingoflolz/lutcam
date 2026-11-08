"""Execute the actual flash-loaded RV32EC firmware with bounded peripheral models."""
from pathlib import Path
import json
import shutil
import subprocess
import sys
import tempfile
import struct
import zlib
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "fpga/litex"))
sys.path.insert(0, str(ROOT))
from migen import ClockDomain, Module, Mux
from migen.fhdl import verilog
from litex.soc.interconnect import wishbone
from lutcam.supervisor import CameraSupervisor
from lutcam.control import WishboneGuard
from lutcam.network import NetworkControl
from lutcam.supervisor import RAM_BYTES
from liteeth.core.udp import LiteEthUDPUserPort
from firmware.build import build
from tools.lutcam.ov9281 import board_mode_writes


class Fixture(Module):
    def __init__(self):
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.submodules.cpu = cpu = CameraSupervisor()
        self.submodules.guard = guard = WishboneGuard()
        self.comb += [cpu.master.connect(guard.bus), cpu.guard_fault.eq(guard.fault_event)]
        udp = LiteEthUDPUserPort(32)
        self.submodules.network = network = NetworkControl(udp, cpu.packet_dma,
            ram_bytes=RAM_BYTES, mac_address=cpu.loader.mac_address, preferred_ip=0xc0a80132)
        host = wishbone.Interface()
        peripheral = wishbone.Interface()
        self.submodules.fabric = fabric = wishbone.InterconnectShared([guard.master, host], [
            (lambda a: a[10:] == 0x30028000 >> 12, cpu.bus),
            (lambda a: a[10:] == 0x30029000 >> 12, network.bus),
            (lambda a: (a[10:] != 0x30028000 >> 12) & (a[10:] != 0x30029000 >> 12), peripheral)], register=True)
        self.comb += [cpu.granted.eq(fabric.arbiter.rr.grant == 0),
                     guard.granted.eq(fabric.arbiter.rr.grant == 0),
                     network.granted.eq(fabric.arbiter.rr.grant == 0),
                     network.inhibit.eq(~cpu.loader.done | cpu.trap | cpu.bus_fault),
                     udp.source.src_port.eq(67), udp.source.dst_port.eq(68),
                     udp.source.ip_address.eq(0xc0a801fe), udp.source.error.eq(0),
                     udp.source.last_be.eq(Mux(udp.source.last,8,0))]
        self.sync += cpu.timer.eq(cpu.timer + 1200)
        self.ports = dict(clk=self.cd_sys.clk, rst=self.cd_sys.rst,
                          slot=cpu.loader.slot, identity_ready=cpu.loader.identity_ready,
                          boot_done=cpu.loader.done, boot_error=cpu.loader.error,
                          trap=cpu.trap, bus_fault=cpu.bus_fault,
                          record_first=cpu.record_first, record_last=cpu.record_last,
                          record_count=cpu.record_count, board_mac=cpu.loader.mac_address)
        for prefix, endpoint, fields in (
                ("net_rx",udp.source,("valid","ready","data","last","length")),
                ("net_tx",udp.sink,("valid","ready","data","last","length","ip_address"))):
            for field in fields:
                self.ports[prefix+"_"+field]=getattr(endpoint,field)
        self.ports.update(net_ip=network.ip_address, net_mask=network.netmask,
                          net_router=network.gateway, net_state=network.state,
                          net_revoke=network.revoke_capture)
        for field in ("start", "done", "tx", "rx", "cs_n", "busy"):
            self.ports["flash_" + field] = getattr(cpu.loader.spi, field)
        for name, bus in (("host", host), ("p", peripheral)):
            for field in ("adr", "dat_w", "sel", "cyc", "stb", "we", "ack", "err", "dat_r"):
                self.ports[name+"_"+field] = getattr(bus, field)
        for name, signal in self.ports.items():
            signal.name_override=name


class SupervisorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which("verilator"):
            raise unittest.SkipTest("Verilator required")
        cls.temp = tempfile.TemporaryDirectory(prefix="lutcam-supervisor-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.directory = d = Path(cls.temp.name)
        cls.metadata = build(d / "firmware")
        mode=board_mode_writes(0,pattern="off",fps=120)
        (d / "mode.hex").write_text("".join(f"{reg<<8|value:08x}\n" for reg,value in mode))
        code=(d / "firmware/firmware.flash.bin").read_bytes()
        (d / "flash.hex").write_text("".join(f"{int.from_bytes(code[i:i+4], 'little'):08x}\n" for i in range(0,len(code),4)))
        invalid = struct.pack("<3I", 0x40000537, 0x00052583, 0x00100073)
        invalid = struct.pack("<4I", int.from_bytes(code[:4], "little"), len(invalid),
                              zlib.crc32(invalid), RAM_BYTES) + invalid
        (d / "invalid-pointer.hex").write_text("".join(
            f"{int.from_bytes(invalid[i:i+4], 'little'):08x}\n" for i in range(0,len(invalid),4)))
        dut=Fixture()
        conversion=verilog.convert(dut, ios=set(dut.ports.values()), name="fixture")
        (d / "fixture.v").write_text(conversion.main_source)
        for name, data in conversion.data_files.items():
            (d / name).write_text(data)
        command=["verilator", "--binary", "--timing", "-j", "2", "-Wno-fatal", "--top-module", "tb",
                 "--Mdir", str(d / "obj"), str(d / "fixture.v"),
                 str(ROOT / "vendor/picorv32/picorv32.v"), str(Path(__file__).with_name("supervisor_tb.sv"))]
        result=subprocess.run(command, cwd=d, text=True, capture_output=True, timeout=120)
        (d / "compile.log").write_text(result.stdout+result.stderr)
        if result.returncode:
            raise RuntimeError(result.stdout+result.stderr)

    def run_case(self, case, slot=1):
        result=subprocess.run([str(self.directory / "obj/Vtb"), f"+CASE={case}", f"+SLOT={slot}",
                               f"+STACK_BOUND={self.metadata['stack']['worst_case_bytes']}"],
                              cwd=self.directory, text=True, capture_output=True, timeout=60)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertIn("PASS supervisor", result.stdout)

    def test_invalid_data_pointer_halts_before_system_bus(self): self.run_case(10)
    def test_slot_a_boot_sensor_init_and_histogram_exposure(self): self.run_case(0)
    def test_slot_b_boot_sensor_init_and_histogram_exposure(self): self.run_case(0,2)
    def test_corrupt_code_never_executes(self): self.run_case(1)
    def test_bad_header_never_executes(self): self.run_case(2)
    def test_i2c_failure_does_not_launch_partial_group(self): self.run_case(3)
    def test_invalid_length_never_executes(self): self.run_case(4)
    def test_flash_byte_timeout_never_executes(self): self.run_case(5)
    def test_ae_recovers_from_one_line_in_a_black_scene(self): self.run_case(6)
    def test_failed_group_end_restarts_without_launch(self): self.run_case(7)
    def test_invalid_first_histogram_reports_failure_before_any_exposure_update(self): self.run_case(8)
    def test_dhcp_dma_acquisition_renewal_and_expiry_on_actual_cpu(self): self.run_case(9)


if __name__ == "__main__": unittest.main()
