"""Native USB counters preserve electrical link diagnostics."""

from pathlib import Path
import os
import re
import shutil
import subprocess
from types import SimpleNamespace

import pytest
from litex.build.efinix.toolchain import find_efinity_path
from migen import Instance, Signal
from migen.fhdl import verilog
from migen.fhdl.simplify import FullMemoryWE

from fpga.litex.lutcam.usb_diagnostics import USBLinkDiagnostics


@pytest.mark.parametrize('use_dsp', [False, True])
def test_luna_factory_preserves_bindings_and_selects_counter_implementation(use_dsp):
    names = ('i__bus_d_p_i', 'i__bus_d_n_i', 'o__bus_d_p_o', 'o__bus_d_n_o',
             'o__bus_d_p_oe', 'o__bus_d_n_oe', 'o__bus_pullup_o', 'i_connect',
             'i_usb_rst', 'i_usb_io_rst')
    parameters = {name: Signal() for name in names}
    usb = SimpleNamespace(core_params=dict(parameters))
    link = USBLinkDiagnostics.from_luna(usb, pll_locked=Signal(), vbus_valid=Signal(),
                                        use_dsp_counters=use_dsp)
    native = [item for item in link.get_fragment().specials
              if isinstance(item, Instance) and item.of == 'EFX_DSP48']
    assert len(native) == (5 if use_dsp else 0)
    assert usb.core_params.keys() == parameters.keys()
    assert all(usb.core_params[name] is value for name, value in parameters.items())


def _generate(path, name, use_dsp):
    names = ['dp_i', 'dm_i', 'dp_oe', 'dm_oe', 'pullup', 'connect',
             'vbus_valid', 'usb_12_reset', 'dp_o', 'dm_o', 'usb_48_reset', 'pll_locked']
    inputs = {name: Signal(name=name) for name in names}
    dut = USBLinkDiagnostics(**inputs, use_dsp_counters=use_dsp)
    ios = set(inputs.values())
    for field, _, _ in dut.bus.layout:
        signal = getattr(dut.bus, field)
        signal.name_override = 'bus_' + field
        ios.add(signal)
    for field in ('live', 'seen', 'rx_edges', 'tx_bursts', 'se0_events',
                  'se0_cycles', 'usb_12_heartbeat', 'usb_48_heartbeat'):
        signal = getattr(dut, field)
        signal.name_override = field
        ios.add(signal)
    FullMemoryWE()(dut)
    # Migen writes RAM init files relative to cwd, so keep generated artifacts
    # in this test's private directory rather than in the repository.
    previous = Path.cwd()
    os.chdir(path.parent)
    try:
        result = verilog.convert(dut, ios=ios, name=name)
        result.write(path.name)
        return str(result)
    finally:
        os.chdir(previous)


@pytest.mark.skipif(shutil.which('verilator') is None, reason='Verilator required')
def test_usb_dsp_counters_match_fabric_wrap_saturation_and_domain_resets(tmp_path):
    try:
        model = Path(find_efinity_path()) / 'sim_models/verilog/efx_dsp48.v'
    except OSError:
        pytest.skip('Set LITEX_ENV_EFINITY to validate the native DSP model')
    assert model.is_file()
    ref = _generate(tmp_path / 'reference.v', 'probe_ref', False)
    dut = _generate(tmp_path / 'candidate.v', 'probe_dsp', True)
    ports = re.findall(
        r'^\s*(input|output)\s+(?:reg\s+)?(?:signed\s+)?(\[\d+:\d+\])?\s*(\w+)\s*[,\n]',
        ref.split(');', 1)[0], re.M)
    inputs = {name: width for direction, width, name in ports if direction == 'input'}
    outputs = {name: width for direction, width, name in ports if direction == 'output'}
    instances = [name for parameters, name in re.findall(
        r'EFX_DSP48 #\((.*?)\) (EFX_DSP48(?:_\d+)?) \(', dut, re.S)
        if '.N_SEL("W")' in parameters]
    assert len(instances) == 5

    def native(index):
        return f'instance_dut.{instances[index]}.genblk1.i_DSP48.W_pipe.O_r'

    lines = ['module tb;']
    for name, width in inputs.items():
        lines.append(f'reg {width} {name}=0;')
    for name, width in outputs.items():
        lines += [f'wire {width} ref_{name};', f'wire {width} dut_{name};']
    for module, prefix in [('probe_ref', 'ref'), ('probe_dsp', 'dut')]:
        connections = [f'.{name}({name})' for name in inputs]
        connections += [f'.{name}({prefix}_{name})' for name in outputs]
        lines.append(f'{module} instance_{prefix}({", ".join(connections)});')
    lines += [
        'always #5 sys_clk=~sys_clk;', 'always #13 usb_48_clk=~usb_48_clk;',
        'integer cycle, sys_resets=0, usb48_resets=0, usb12_resets=0;',
        'integer wraps=0, saturation=0;',
        "reg [63:0] rng=64'h8bc973df1571a37e; reg [31:0] previous_count=0;",
        'initial begin', 'sys_rst=1; usb_48_rst=1;',
    ]
    lines.insert(1, 'always #52 usb_12_clk=~usb_12_clk;')
    lines += ['usb_12_rst=1;']
    lines += ['repeat(20) @(negedge sys_clk); sys_rst=0; usb_48_rst=0;']
    lines += ['usb_12_rst=0;']
    lines += [
        'for(cycle=0;cycle<300000;cycle=cycle+1) begin',
        '@(negedge sys_clk); rng={rng[62:0],rng[63]^rng[62]^rng[60]^rng[59]};',
        'pullup_1=1; connect=1; vbus_valid=1; usb_12_reset=0;',
        'bus_cyc=1; bus_stb=1; bus_sel=15; bus_cti=0; bus_bte=0; bus_we=0; bus_dat_w=0;',
        # Directed, independent resets last long enough for each source clock.
        # Random pulses alone can miss all mid-run reset cases with a fixed seed.
        'sys_rst=(cycle>=151000 && cycle<151040)||(cycle>=200500 && cycle<200540);',
        'usb_48_rst=(cycle>=151100 && cycle<151140);',
        'if(sys_rst) sys_resets=sys_resets+1; if(usb_48_rst) usb48_resets=usb48_resets+1;',
        'dp_i=rng[0]; dm_i=~rng[0]; dp_oe=(rng[5:1]==0); dm_oe=dp_oe;',
        'if(cycle%10000<2000) begin dp_i=0; dm_i=0; dp_oe=0; dm_oe=0; end',
    ]
    lines += [
        'usb_12_rst=(cycle>=151200 && cycle<151240);',
        'if(usb_12_rst) usb12_resets=usb12_resets+1;',
        'usb_48_reset=0; pll_locked=1; dp_o=rng[7]; dm_o=~rng[7]; bus_adr=cycle%14;',
        'if(cycle==0 || cycle==100000) begin',
    ]
    for index, name in enumerate(('rx_edges', 'tx_bursts', 'se0_events',
                                  'usb_12_heartbeat', 'usb_48_heartbeat')):
        value = "16'hfffc" if index < 3 else "32'hfffffffc"
        lines += [f'instance_ref.{name}={value};', f"{native(index)}=48'hfffffffffffC;"]
    lines += ['end', 'previous_count=ref_usb_48_heartbeat;']
    lines += ['@(posedge sys_clk); #1;']
    for name in outputs:
        lines.append(f'if(ref_{name} !== dut_{name}) $fatal(1,"diagnostics {name} cycle=%0d ref=%h dut=%h",cycle,ref_{name},dut_{name});')
    lines += [
        'if(!sys_rst && ref_usb_48_heartbeat<previous_count) wraps=wraps+1;',
        "if(ref_rx_edges==16'hffff && ref_tx_bursts==16'hffff && ref_se0_events==16'hffff) saturation=saturation+1;",
    ]
    lines += ['end', 'if(sys_resets==0 || usb48_resets==0 || wraps<2) $fatal(1,"missing reset/wrap coverage");']
    lines += ['if(usb12_resets==0 || saturation==0) $fatal(1,"missing USB12 reset/saturation coverage");']
    lines += ['$display("PASS diagnostics 300000 cycles all outputs; wraps=%0d saturation=%0d",wraps,saturation);',
              '$finish; end', 'endmodule']
    testbench = '\n'.join(lines) + '\n'
    (tmp_path / 'tb.sv').write_text(testbench)
    command = ['verilator', '--binary', '--timing', '-Wno-fatal', '--top-module', 'tb',
               '--Mdir', str(tmp_path / 'obj'), str(tmp_path / 'tb.sv'),
               str(tmp_path / 'reference.v'), str(tmp_path / 'candidate.v'), str(model), '-j', '2']
    result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True)
    (tmp_path / 'compile.log').write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    result = subprocess.run([str(tmp_path / 'obj/Vtb')], cwd=tmp_path, capture_output=True, text=True)
    (tmp_path / 'run.log').write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'PASS diagnostics 300000 cycles' in result.stdout
