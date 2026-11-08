"""Fabric snapshots preserve every bit, signed extension and reset priority."""
from pathlib import Path
import shutil
import subprocess

import pytest
from migen import Instance, Module, Signal
from migen.fhdl import verilog

from fpga.litex.lutcam.pair import PairedCaptureControl


def _generate(path, name):
    dut = Module()
    capture, clear = Signal(name='capture'), Signal(name='clear')
    wide_input, wide_output = Signal(519, name='wide_input'), Signal(519, name='wide_output')
    signed_a, signed_b = Signal((23, True), name='signed_a'), Signal((22, True), name='signed_b')
    latency, count = Signal(21, name='latency'), Signal(32, name='count')
    out_a, out_b = Signal((32, True), name='out_a'), Signal((32, True), name='out_b')
    out_latency, out_count = Signal(32, name='out_latency'), Signal(32, name='out_count')
    PairedCaptureControl._snapshot_registers(dut, [(wide_output, wide_input)], capture, clear)
    PairedCaptureControl._snapshot_registers(dut,
        [(out_a, signed_a), (out_b, signed_b), (out_latency, latency), (out_count, count)],
        capture, clear)
    ports = {capture, clear, wide_input, wide_output, signed_a, signed_b, latency, count,
             out_a, out_b, out_latency, out_count}
    fragment = dut.get_fragment()
    assert not any(isinstance(item, Instance) and item.of.startswith('EFX_DSP')
                   for item in fragment.specials)
    verilog.convert(fragment, ios=ports, name=name).write(str(path))


@pytest.mark.skipif(shutil.which('verilator') is None, reason='Verilator required')
def test_fabric_snapshots_walk_every_bit_and_preserve_sign_and_clear(tmp_path):
    (tmp_path / 'reference.v').write_text("""
module register_ref(input sys_clk,sys_rst,clear,capture,
 input [518:0] wide_input,input signed [22:0] signed_a,
 input signed [21:0] signed_b,input [20:0] latency,input [31:0] count,
 output reg [518:0] wide_output=0,
 output reg signed [31:0] out_a=0,out_b=0,
 output reg [31:0] out_latency=0,out_count=0);
always @(posedge sys_clk) begin
 if(sys_rst || clear) begin
  wide_output<=0;out_a<=0;out_b<=0;out_latency<=0;out_count<=0;
 end else if(capture) begin
  wide_output<=wide_input;out_a<=signed_a;out_b<=signed_b;
  out_latency<=latency;out_count<=count;
 end
end
endmodule
""")
    _generate(tmp_path / 'candidate.v', 'register_fabric')
    lines = [
        'module tb;',
        'reg sys_clk=0, sys_rst=0, clear=0, capture=0;',
        'reg [518:0] wide_input=0; reg signed [22:0] signed_a=0; reg signed [21:0] signed_b=0;',
        'reg [20:0] latency=0; reg [31:0] count=0;',
        'wire [518:0] ref_wide_output, dut_wide_output;',
        'wire [31:0] ref_out_a, ref_out_b, ref_out_latency, ref_out_count;',
        'wire [31:0] dut_out_a, dut_out_b, dut_out_latency, dut_out_count;',
        'reg [63:0] rng=64\'h974752c434120abc; reg [575:0] random_wide;',
        'integer cycle, chunk;',
        'always #5 sys_clk=~sys_clk;',
    ]
    inputs = ['sys_clk', 'sys_rst', 'clear', 'capture', 'wide_input', 'signed_a', 'signed_b', 'latency', 'count']
    outputs = ['wide_output', 'out_a', 'out_b', 'out_latency', 'out_count']
    for module, prefix in [('register_ref', 'ref'), ('register_fabric', 'dut')]:
        ports = [f'.{name}({name})' for name in inputs]
        ports += [f'.{name}({prefix}_{name})' for name in outputs]
        lines.append(f'{module} {prefix}_instance({", ".join(ports)});')
    lines += [
        'initial begin',
        'sys_rst=1; repeat(3) @(negedge sys_clk); sys_rst=0;',
        'for(cycle=0;cycle<20000;cycle=cycle+1) begin',
        '@(negedge sys_clk);',
        'for(chunk=0;chunk<9;chunk=chunk+1) begin',
        'rng={rng[62:0],rng[63]^rng[62]^rng[60]^rng[59]}; random_wide[chunk*64+:64]=rng;',
        'end',
        'wide_input=random_wide[518:0];',
        "if(cycle<519) wide_input=519'd1 << cycle;",
        "else if(cycle<1038) wide_input=~(519'd1 << (cycle-519));",
        'signed_a=(cycle%4==0)?23\'h400000:(cycle%4==1)?23\'h3fffff:rng[22:0];',
        'signed_b=(cycle%4==0)?22\'h200000:(cycle%4==1)?22\'h1fffff:rng[44:23];',
        'latency=(cycle%4==0)?21\'h1fffff:rng[63:43]; count=(cycle%4==0)?32\'hffffffff:rng[31:0];',
        'capture=(cycle<1038)?1:rng[4]; clear=(cycle>=1038)&&(cycle%257==0);',
        'sys_rst=(cycle>=1038)&&(cycle%509==0);',
        'if(cycle==1100 || cycle==1101) begin clear=1; capture=cycle[0]; end',
        'if(cycle==1102 || cycle==1103) begin sys_rst=1; capture=cycle[0]; end',
        '@(posedge sys_clk); #1;',
    ]
    for name in outputs:
        lines.append(f'if(ref_{name} !== dut_{name}) $fatal(1,"{name} mismatch cycle=%0d",cycle);')
    lines += ['end', '$display("PASS519 walking bits and inverses; signed extrema, holds and reset");',
              '$finish; end', 'endmodule']
    (tmp_path / 'tb.sv').write_text('\n'.join(lines) + '\n')
    command = ['verilator', '--binary', '--timing', '-Wno-fatal', '--top-module', 'tb',
               '--Mdir', str(tmp_path / 'obj'), str(tmp_path / 'tb.sv'),
               str(tmp_path / 'reference.v'), str(tmp_path / 'candidate.v'), '-j', '4']
    result = subprocess.run(command, capture_output=True, text=True)
    (tmp_path / 'compile.log').write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    result = subprocess.run([str(tmp_path / 'obj/Vtb')], capture_output=True, text=True)
    (tmp_path / 'run.log').write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'PASS519 walking bits' in result.stdout
