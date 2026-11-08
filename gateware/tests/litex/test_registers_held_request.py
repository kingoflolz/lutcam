"""Main-host held-request specialization preserves captured-mode bus behavior."""
from functools import partial
import importlib.util
import inspect
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
from migen import Cat, ClockDomain, Module, Mux, Signal
from migen.fhdl import verilog as migen_verilog
from migen.sim import passive, run_simulation
from litex.gen.fhdl import verilog as litex_verilog
from litex.soc.interconnect import wishbone

ROOT = Path(os.environ.get('LUTCAM_TEST_ROOT', Path(__file__).resolve().parents[2]))
sys.path[:0] = [str(ROOT/'fpga/litex'), str(ROOT/'tests/litex')]
from lutcam.calibration import CalibrationBank
from lutcam.control import WishboneGuard
from lutcam.registers import IntegrationRegisters
import test_integration_registers_staging as staging

# Artifact qualification may bind the candidate without editing production.
if os.environ.get('LUTCAM_HELD_REGISTERS_SOURCE'):
    spec = importlib.util.spec_from_file_location('lutcam.held_candidate',
        os.environ['LUTCAM_HELD_REGISTERS_SOURCE'])
    candidate = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(candidate)
    IntegrationRegisters = candidate.IntegrationRegisters


class Fabric(Module):
    """Real two-host guards, cycle arbiter, registered decoder and calibration RAM."""
    def __init__(self, held):
        self.submodules.bank = bank = CalibrationBank()
        original = staging.IntegrationRegisters
        staging.IntegrationRegisters = partial(IntegrationRegisters, held_request=held)
        try:
            self.submodules.engine = engine = staging.StagedRequestHarness(bank)
        finally:
            staging.IntegrationRegisters = original
        self.guards = []
        for index in range(2):
            guard = WishboneGuard(external_count=True)
            setattr(self.submodules, 'guard'+str(index), guard)
            self.guards.append(guard)
        other = wishbone.Interface()
        self.comb += other.dat_r.eq(0x5a69c387)
        # Registered data selection requires a clocked peripheral response.
        self.sync += other.ack.eq(other.cyc & other.stb & ~other.ack)
        self.submodules.fabric = fabric = wishbone.InterconnectShared(
            [g.master for g in self.guards], [
                (lambda address: address[10:] == 0, engine.regs.bus),
                (lambda address: address[10:] == 1, other),
            ], register=True, timeout_cycles=1_000_000, arbiter='cycle')
        for index, guard in enumerate(self.guards):
            self.comb += guard.granted.eq(fabric.arbiter.rr.grant == index)
        self.inputs = {}
        for index, guard in enumerate(self.guards):
            for name in ('cyc','stb','adr','we','sel','dat_w'):
                self.inputs[f'h{index}_{name}'] = getattr(guard.bus,name)
        counter = engine.counters
        for name in ('query_ready','query_response_valid','query_data','config_write_ready'):
            self.inputs[name] = getattr(counter,name)
        self.inputs.update(camera0_idle=engine.cameras[0].idle,
            camera1_idle=engine.cameras[1].idle, pair_busy=engine.pair.busy,
            pair_drained=engine.pair.drained)
        outputs = []
        for guard in self.guards:
            b = guard.bus
            outputs += [b.ack,b.err,Mux(b.ack,b.dat_r,0),guard.fault,
                guard.fault_address,guard.fault_status,guard.fault_event]
        pair = engine.pair
        outputs += [getattr(pair,n) for n in ('start','stop','reset','clear','snapshot',
            'watchdog_cycles','max_skew_cycles','sync_period','sync_width','sync_enable',
            'min_latency_cycles','max_latency_cycles','phase0_cycles','phase1_cycles')]
        for camera in engine.cameras:
            outputs += [camera.configuration_error.status,camera.calibration_valid.status,
                camera.run.storage,camera.sensor_control.storage]
            for name in ('clear','soft_reset','calibration_commit'):
                command = getattr(camera,name)
                outputs += [command.re,Mux(command.re,command.r,0)]
        outputs += [channel.clear for channel in engine.transport.channels]
        outputs += [counter.query_valid,Mux(counter.query_valid,counter.query_address,0),
            counter.config_write_valid,
            Mux(counter.config_write_valid,counter.config_write_address,0),
            Mux(counter.config_write_valid,counter.config_write_data,0),
            bank.host_read,bank.host_write,
            Mux(bank.host_write,bank.host_write_data,0)]
        self.signature = Signal(len(Cat(*outputs)))
        self.comb += self.signature.eq(Cat(*outputs))


class Miter(Module):
    def __init__(self):
        self.clock_domains.cd_sys = ClockDomain('sys')
        self.submodules.reference = reference = Fabric(False)
        self.submodules.candidate = candidate = Fabric(True)
        self.ports = {'sys_clk':self.cd_sys.clk,'sys_rst':self.cd_sys.rst}
        self.inputs = {}
        for name, port in reference.inputs.items():
            value = Signal(len(port), name=name)
            self.inputs[name] = value
            self.comb += [port.eq(value),candidate.inputs[name].eq(value)]
        self.ports.update(self.inputs)
        mismatch = Signal(name='mismatch')
        self.comb += mismatch.eq(reference.signature != candidate.signature)
        self.ports['mismatch'] = mismatch
        for index,guard in enumerate(reference.guards):
            for field in ('ack','dat_r'):
                self.ports[f'h{index}_{field}'] = getattr(guard.bus,field)
            self.ports[f'h{index}_fault_event'] = guard.fault_event
        for name in ('query_valid','query_address','config_write_valid',
                     'config_write_address','config_write_data'):
            self.ports[name] = getattr(reference.engine.counters,name)
        for name,port in self.ports.items():
            port.name_override = name


def test_held_request_is_explicitly_optional():
    assert inspect.signature(IntegrationRegisters).parameters['held_request'].default is False
    doc = IntegrationRegisters.__doc__
    for term in ('RecordUARTBone','WishboneGuard','cycle arbiter','ACK','CYC','reset'):
        assert term in doc


def test_portable_held_request_matches_capture_through_timeout_and_reselection():
    dut = Miter()
    p = dut.inputs
    @passive
    def compare():
        while True:
            assert (yield dut.ports['mismatch']) == 0
            yield
    def drive(index,address,data=None,sel=15):
        for name,value in [('adr',address//4),('we',data is not None),
                           ('dat_w',data or 0),('sel',sel),('cyc',1),('stb',1)]:
            yield p[f'h{index}_{name}'].eq(value)
    def transfer(index,address,data=None,sel=15):
        yield from drive(index,address,data,sel)
        yield
        for _ in range(4200):
            if (yield dut.ports[f'h{index}_ack']):
                value = yield dut.ports[f'h{index}_dat_r']
                yield
                yield p[f'h{index}_cyc'].eq(0)
                yield p[f'h{index}_stb'].eq(0)
                yield
                return value
            yield
        raise AssertionError('host did not complete')
    def bench():
        for name in ('camera0_idle','camera1_idle','pair_drained','query_ready','config_write_ready'):
            yield p[name].eq(1)
        for _ in range(20): yield
        for index in range(2):
            for mask in range(16):
                assert (yield from transfer(index,0x80+index*256,0x12345678,mask)) == 0
                yield from transfer(index,0x80+index*256)
        yield p['config_write_ready'].eq(0)
        yield from drive(0,0x404,0x12345678)
        for _ in range(8): yield
        yield from drive(1,0x180)
        for _ in range(4100):
            if (yield dut.ports['h0_ack']):
                assert (yield dut.ports['h0_dat_r']) == 0xffffffff
                assert (yield dut.ports['h0_fault_event'])
                break
            yield
        else: raise AssertionError('guard did not time out deferred write')
        yield
        yield p['h0_cyc'].eq(0)
        yield p['h0_stb'].eq(0)
        yield p['h0_adr'].eq(0x3fffffff)
        yield p['h0_dat_w'].eq(0xfedcba98)
        yield p['config_write_ready'].eq(1)
        for _ in range(30):
            if (yield dut.ports['h1_ack']): break
            yield
        else: raise AssertionError('queued peer did not acquire bus')
        yield
        yield p['h1_cyc'].eq(0)
        yield p['h1_stb'].eq(0)
        yield
        assert (yield from transfer(0,0x1000)) == 0x5a69c387
    run_simulation(dut,[bench(),compare()])


@pytest.mark.parametrize('emitter',[migen_verilog,litex_verilog],ids=['migen','litex'])
def test_generated_held_request_matches_capture(tmp_path,emitter):
    if not shutil.which('iverilog') or not shutil.which('vvp'):
        if os.environ.get('LUTCAM_REQUIRE_NATIVE_RTL') == '1':
            pytest.fail('required generated-RTL test needs iverilog and vvp')
        pytest.skip('iverilog and vvp are required for generated-RTL parity')
    dut = Miter()
    result = emitter.convert(dut,ios=set(dut.ports.values()),name='miter')
    (tmp_path/'miter.v').write_text(result.main_source)
    for name,data in result.data_files.items(): (tmp_path/name).write_text(data)
    tb = Path(__file__).with_name('held_request_tb.sv')
    executable = tmp_path/'run'
    compile_result = subprocess.run(['iverilog','-g2012','-s','tb','-o',str(executable),
        str(tb),str(tmp_path/'miter.v')],cwd=tmp_path,text=True,capture_output=True)
    assert compile_result.returncode == 0, compile_result.stdout+compile_result.stderr
    run = subprocess.run(['vvp',str(executable)],cwd=tmp_path,text=True,capture_output=True)
    assert run.returncode == 0, run.stdout+run.stderr
    assert 'PASS held request' in run.stdout
