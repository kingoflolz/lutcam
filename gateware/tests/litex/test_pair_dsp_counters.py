"""Compare production DSP accumulators with fabric control, including wrap."""
from pathlib import Path
import re
import shutil
import subprocess

import pytest
from litex.build.efinix.toolchain import find_efinity_path
from migen import Signal
from migen.fhdl import verilog

from fpga.litex.lutcam.pair import PairedCaptureControl


def _generate(path, name, use_dsp):
    dut = PairedCaptureControl(use_dsp_counters=use_dsp)
    ports = set()
    for field, value in vars(dut).items():
        if not field.startswith('_') and isinstance(value, Signal):
            value.name_override = field
            ports.add(value)
    for index, value in enumerate(dut.pending_frame_id):
        value.name_override = f'pending_frame_id_{index}'
        ports.add(value)
    text = str(verilog.convert(dut.get_fragment(), ios=ports, name=name))
    path.write_text(text)
    return text


@pytest.mark.skipif(shutil.which('verilator') is None, reason='Verilator required')
def test_production_dsp_counters_match_fabric_through_wrap_and_reset(tmp_path):
    try:
        model = Path(find_efinity_path()) / 'sim_models/verilog/efx_dsp48.v'
    except OSError:
        pytest.skip('Set LITEX_ENV_EFINITY to validate the native DSP model')
    assert model.is_file()
    reference = _generate(tmp_path / 'reference.v', 'pair_ref', False)
    candidate = _generate(tmp_path / 'candidate.v', 'pair_dsp', True)
    ports = re.findall(
        r'^\s*(input|output)\s+(?:reg\s+)?(?:signed\s+)?(\[\d+:\d+\])?\s*(\w+)\s*[,\n]',
        reference.split(');', 1)[0], re.M)
    inputs = {name: width for direction, width, name in ports if direction == 'input'}
    outputs = {name: width for direction, width, name in ports if direction == 'output'}
    assert len(outputs) == 34
    # These are initialized architectural states, not forced outputs. The test
    # then resumes normal RTL updates and compares every visible output. Seed
    # the native W feedback registers so wrap coverage needs thousands, rather
    # than billions, of clocks. Match actual generated instance names first.
    # DSPs must only implement the nine arithmetic counters. Snapshot and
    # metadata payloads remain fabric registers.
    assert candidate.count('EFX_DSP48 #(') == 9
    dsp_instances = [name for parameters, name in re.findall(
        r'EFX_DSP48 #\((.*?)\) (EFX_DSP48(?:_\d+)?) \(', candidate, re.S)
        if '.N_SEL("W")' in parameters]
    assert len(dsp_instances) == 9
    assert 'pairedcapturecontrol_now' in reference
    count_names = re.findall(r'reg \[31:0\] (pairedcapturecontrol_counts\d+) =', reference)
    assert len(count_names) == 5

    def dsp_register(index):
        return f'instance_dut.{dsp_instances[index]}.genblk1.i_DSP48.W_pipe.O_r'

    lines = ['module tb;']
    for name, width in inputs.items():
        lines.append(f'reg {width} {name}=0;')
    for name, width in outputs.items():
        lines += [f'wire {width} ref_{name};', f'wire {width} dut_{name};']
    for module, prefix in [('pair_ref', 'ref'), ('pair_dsp', 'dut')]:
        connections = [f'.{name}({name})' for name in inputs]
        connections += [f'.{name}({prefix}_{name})' for name in outputs]
        lines.append(f'{module} instance_{prefix}({", ".join(connections)});')
    lines += [
        'always #5 sys_clk=~sys_clk;',
        "integer cycle, low_wraps=0, full_wraps=0; reg [63:0] previous_now;",
        "reg [63:0] rng=64'h0123456789abcdef;",
        'initial begin',
        'sys_rst=1; repeat(3) @(negedge sys_clk); sys_rst=0;',
        'sync_period=100; sync_width=5; watchdog_cycles=200; max_skew_cycles=10;',
        'min_latency_cycles=2; max_latency_cycles=80; pipeline_ready=3; health_good=1; transport_drained=1;',
        'sync_enable=1;',
        'for(cycle=0;cycle<300000;cycle=cycle+1) begin',
        '@(negedge sys_clk); rng = {rng[62:0],rng[63]^rng[62]^rng[60]^rng[59]};',
        'sof=0; admitted=0; retired=0; start=0; stop=0; clear=0; reset=0; snapshot=0;',
        'if(cycle==0 || cycle==10000) begin',
        "instance_ref.pairedcapturecontrol_now=(cycle==0)?64'hfffffffffffffff0:64'h12345678fffffff0;",
        f"{dsp_register(0)}=48'hfffffffffff0;",
        f"{dsp_register(1)}=(cycle==0)?48'hffffffffffff:48'hffff12345678;",
    ]
    for index, name in enumerate(count_names):
        lines += [f"instance_ref.{name}=32'hfffffff0;",
                  f"{dsp_register(index + 2)}=48'hfffffffffff0;"]
    for index, name in enumerate(('pairs_started', 'pairs_completed')):
        lines += [f"instance_ref.{name}=32'hfffffff0;",
                  f"{dsp_register(index + 7)}=48'hfffffffffff0;"]
    lines += [
        'end',
        'previous_now=instance_ref.pairedcapturecontrol_now;',
        'if (cycle < 100000) begin',
        'sof[0]=(cycle%100==11); sof[1]=(cycle%100==13);',
        'admitted[0]=ref_grants[0] && sof[0]; admitted[1]=ref_grants[1] && sof[1];',
        'retired=(cycle%100==60)?3:0; start=(cycle%2000==400); stop=(cycle%2000==1400);',
        'snapshot=(cycle%37==0); clear=(cycle%5000==3000); reset=(cycle%11000==10000);',
        'end else begin',
        'sof=rng[1:0]; admitted=rng[3:2]; retired=rng[5:4]; active=rng[7:6];',
        'start=rng[8]; stop=rng[9]; snapshot=rng[10];',
        'clear=(rng[19:11]==0); reset=(rng[28:20]==0); sys_rst=(rng[37:29]==0);',
        'local_fault=(rng[42:38]==0)?rng[44:43]:0; receiver_fault=(rng[49:45]==0)?rng[51:50]:0;',
        'packet_fault=(rng[56:52]==0)?rng[58:57]:0; health_good=~(rng[63:59]==0);',
        'transport_drained=rng[12]; pipeline_ready=rng[14:13]; sync_enable=(rng[22:15]!=0);',
        'if(!sync_enable) begin phase0_cycles=rng[6:0]%100; phase1_cycles=rng[13:7]%100; end',
        'end',
        '@(posedge sys_clk); #1;',
        'if(!sys_rst && instance_ref.pairedcapturecontrol_now[31:0] < previous_now[31:0]) low_wraps=low_wraps+1;',
        'if(!sys_rst && instance_ref.pairedcapturecontrol_now < previous_now) full_wraps=full_wraps+1;',
        'if(instance_ref.pairedcapturecontrol_now !== instance_dut.pairedcapturecontrol_now) $fatal(1,"timestamp mismatch cycle=%0d",cycle);',
    ]
    for index, name in enumerate(count_names):
        lines.append(f'if(instance_ref.{name} !== {dsp_register(index + 2)}[31:0]) '
                     f'$fatal(1,"event counter {name} mismatch cycle=%0d",cycle);')
    for name in outputs:
        lines.append(f'if(ref_{name} !== dut_{name}) '
                     f'$fatal(1,"mismatch {name} cycle=%0d ref=%h dut=%h",cycle,ref_{name},dut_{name});')
    lines += ['end',
              'if(low_wraps<2 || full_wraps<1) $fatal(1,"missing wrap coverage");',
              '$display("PASS 300000 cycles, all34 outputs, timestamp carry/wrap and reset priority");',
              '$finish; end', 'endmodule']
    (tmp_path / 'tb.sv').write_text('\n'.join(lines) + '\n')
    command = ['verilator', '--binary', '--timing', '-Wno-fatal', '--top-module', 'tb',
               '--Mdir', str(tmp_path / 'obj'), str(tmp_path / 'tb.sv'),
               str(tmp_path / 'reference.v'), str(tmp_path / 'candidate.v'), str(model), '-j', '4']
    result = subprocess.run(command, capture_output=True, text=True)
    (tmp_path / 'compile.log').write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    result = subprocess.run([str(tmp_path / 'obj/Vtb')], capture_output=True, text=True)
    (tmp_path / 'run.log').write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'PASS 300000 cycles' in result.stdout
