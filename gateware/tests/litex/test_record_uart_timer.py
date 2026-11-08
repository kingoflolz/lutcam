"""Native elapsed UART timer versus actual LiteX WaitTimer and integer oracle."""
import hashlib
import importlib.util
import inspect
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

import pytest
from migen import ClockDomain, Instance, Module, Signal
from migen.fhdl import verilog as migen_verilog
from litex.gen.fhdl import verilog as litex_verilog
from litex.gen.genlib.misc import WaitTimer
from litex.build.efinix.toolchain import find_efinity_path

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'fpga/litex/lutcam/record_bus.py').is_file())
sys.path.insert(0,str(ROOT/'fpga/litex'))
SOURCE=Path(os.environ.get('LUTCAM_RECORD_BUS_SOURCE',ROOT/'fpga/litex/lutcam/record_bus.py'))


def subject():
    spec=importlib.util.spec_from_file_location('record_timer_subject',SOURCE)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_optional_profiles_and_native_timer_limit():
    implementation=subject()
    for option in ('compact_datapath','use_dsp_timeout'):
        assert inspect.signature(implementation.RecordUARTBone).parameters[option].default is False
    for compact in (False,True):
        for native in (False,True):
            fragment=implementation.RecordUARTBone(clk_freq=120000000,
                compact_datapath=compact,use_dsp_timeout=native).get_fragment()
            assert sum(isinstance(item,Instance) and item.of=='EFX_DSP48'
                for item in fragment.specials)==int(native)
    for timeout in (1,2,3,12000000,(1<<48)-1):
        implementation.RecordUARTBone(timeout_cycles=timeout,use_dsp_timeout=True)
    # Generic timers remain unconstrained by the width of Titanium's DSP.
    implementation.RecordUARTBone(timeout_cycles=1<<48)
    with pytest.raises(ValueError,match='48 bits'):
        implementation.RecordUARTBone(timeout_cycles=1<<48,use_dsp_timeout=True)
    for timeout in (0,-1):
        with pytest.raises(ValueError,match='positive'):
            implementation.RecordUARTBone(timeout_cycles=timeout,use_dsp_timeout=True)


def emit(path,index,timeout,emitter):
    top=Module();top.wait=Signal(name='wait_input')
    top.submodules.reference=reference=WaitTimer(timeout)
    top.submodules.native=native=subject()._DspWaitTimer(timeout)
    # Pin the tested WaitTimer storage rule; fail if the upstream implementation
    # changes instead of silently substituting a locally invented reference.
    count=reference._fragment.sync['sys'][0].t[0].t[0].l
    assert isinstance(count,Signal) and count.reset.value==timeout
    elapsed=native._fragment.comb[0].l
    top.comb += [reference.wait.eq(top.wait),native.wait.eq(top.wait)]
    fields=dict(wait_input=top.wait,countdown=count,elapsed=elapsed,
        reference_done=reference.done,native_done=native.done)
    top.clock_domains.cd_sys=ClockDomain('sys')
    fields.update(sys_clk=top.cd_sys.clk,sys_rst=top.cd_sys.rst)
    for name,signal in fields.items():signal.name_override=name
    converter=migen_verilog if emitter=='migen' else litex_verilog
    output=converter.convert(top,ios=set(fields.values()),name=f'timer_{index}')
    text=str(output) if emitter=='migen' else output.main_source
    path.write_text(text)
    instance=re.search(r'EFX_DSP48 #\(.*?\)\s+(\w+)\s*\(',text,re.S)
    assert instance and text.count('EFX_DSP48 #(')==1
    done_registered = bool(native._fragment.sync.get('sys'))
    if done_registered:
        assert re.search(r'output\s+reg\s+native_done',text)
    return instance[1],done_registered


@pytest.mark.skipif(shutil.which('verilator') is None,reason='Verilator required')
@pytest.mark.parametrize('emitter',['migen','litex'])
def test_native_uart_timer_exact_wait_reset_and_full_production_ramp(tmp_path,emitter):
    try:model=Path(find_efinity_path())/'sim_models/verilog/efx_dsp48.v'
    except OSError:pytest.skip('Efinity DSP model required')
    assert model.is_file()
    timeouts=(1,2,3,255,256,12000000,(1<<48)-1)
    sources=[];instances=[]
    for index,timeout in enumerate(timeouts):
        path=tmp_path/f'timer_{index}.v';sources.append(path)
        instances.append(emit(path,index,timeout,emitter))
    lines=['module tb;reg clk=0,rst=1,wait_input=0;',
        'always #5 clk=~clk;',
        "reg [63:0] random_bits=64'h71e4c93ab58d206f;",
        'integer cycles=0,seeds=0,k,j;reg[47:0]old_elapsed;']
    for i,t in enumerate(timeouts):
        w=t.bit_length()
        lines += [f'wire [{w-1}:0] countdown_{i},elapsed_{i};wire reference_done_{i},native_done_{i};',
            f'reg [{w-1}:0] expected_{i}={w}\'d{t};',
            f'timer_{i} wd_{i}(.sys_clk(clk),.sys_rst(rst),.wait_input,.countdown(countdown_{i}),.elapsed(elapsed_{i}),.reference_done(reference_done_{i}),.native_done(native_done_{i}));',
            f'always @(posedge clk) if(rst || !wait_input) expected_{i}<={w}\'d{t};else if(expected_{i}!=0) expected_{i}<=expected_{i}-1\'b1;']
    lines += ['task tick;begin @(posedge clk);#1;cycles=cycles+1;']
    for i,t in enumerate(timeouts):
        lines += [f'if(countdown_{i}!==expected_{i} || elapsed_{i}!=={t.bit_length()}\'d{t}-expected_{i} || reference_done_{i}!==(expected_{i}==0) || native_done_{i}!==reference_done_{i}) $fatal(1,"timer {t} cycle%0d down%h expected%h up%h",cycles,countdown_{i},expected_{i},elapsed_{i});']
    lines += ['end endtask','task next_input;begin @(negedge clk);random_bits=random_bits^(random_bits<<13);random_bits=random_bits^(random_bits>>7);random_bits=random_bits^(random_bits<<17);end endtask',
        'task seed(input integer which,input reg[47:0]value);begin next_input();case(which)']
    for i,t in enumerate(timeouts):
        w=t.bit_length()
        # A registered lookahead must be seeded with its reachable invariant,
        # just as the generic countdown is seeded to timeout-elapsed. Ordinary
        # unseeded runs still independently check every terminal/reset edge.
        seed_done = f'wd_{i}.native_done=(value==48\'d{t});' if instances[i][1] else ''
        lines += [f'{i}:begin wd_{i}.{instances[i][0]}.genblk1.i_DSP48.W_pipe.O_r=value;wd_{i}.countdown={w}\'d{t}-value[{w-1}:0];expected_{i}={w}\'d{t}-value[{w-1}:0];{seed_done}end']
    lines += ['endcase seeds=seeds+1;end endtask','initial begin tick();next_input();rst=0;',
        'for(k=0;k<20000;k=k+1)begin next_input();rst=(k%337==0);wait_input=random_bits[0]|random_bits[1];tick();end']
    for i,t in enumerate(timeouts):
        values=sorted({0,1,max(0,t-1),t,
            *[1<<bit for bit in range(t.bit_length()) if (1<<bit)<=t],
            *[(1<<bit)-1 for bit in range(t.bit_length()) if (1<<bit)<=t]})
        for value in values:
            lines += [f'seed({i},48\'d{value});rst=0;wait_input=1;tick();tick();tick();']
        for mask in range(4):
            lines += [f'seed({i},48\'d{max(0,t-1)});rst={mask>>1};wait_input={mask&1};tick();']
    lines += ["seed(5,48'd1234567);rst=0;wait_input=1;tick();next_input();old_elapsed=elapsed_5;rst=1;#1;if(elapsed_5!==old_elapsed[23:0])$fatal(1,\"unexpected asynchronous reset\");tick();",
        'next_input();rst=0;wait_input=1;repeat(12000003)tick();',
        'if(!native_done_5 || countdown_5!=0 || elapsed_5!=12000000)$fatal(1,"production timeout");',
        'next_input();wait_input=0;tick();if(native_done_5 || elapsed_5!=0)$fatal(1,"idle recharge");',
        'next_input();wait_input=1;tick();if(native_done_5 || elapsed_5!=1)$fatal(1,"subsequent command");',
        '$display("PASS UART native timers cycles=%0d seeds=%0d",cycles,seeds);$finish;end endmodule']
    (tmp_path/'tb.sv').write_text('\n'.join(lines)+'\n')
    command=['verilator','--binary','--timing','-Wno-fatal','--top-module','tb','--Mdir',str(tmp_path/'obj'),'-j','2',str(tmp_path/'tb.sv'),*(str(p) for p in sources),str(model)]
    result=subprocess.run(command,capture_output=True,text=True)
    (tmp_path/'compile.log').write_text(result.stdout+result.stderr)
    assert result.returncode==0,result.stdout+result.stderr
    result=subprocess.run([str(tmp_path/'obj/Vtb')],capture_output=True,text=True)
    (tmp_path/'run.log').write_text(result.stdout+result.stderr)
    assert result.returncode==0,result.stdout+result.stderr
    assert 'PASS UART native timers' in result.stdout
    report=dict(status='PASS',emitter=emitter,timeouts=timeouts,output=result.stdout,
        source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        vendor_sha256=hashlib.sha256(model.read_bytes()).hexdigest())
    (tmp_path/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
