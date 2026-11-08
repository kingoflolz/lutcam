"""Cycle-exact emitted RTL replay, with independent Migen/pin oracle inputs."""
from pathlib import Path
import hashlib
import json
import subprocess
from i2c_transaction_harness import generate, pack_storage32


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run_replay(implementations, rows, output, *, emitter='migen', vendor=None):
    output=Path(output).resolve(); output.mkdir(parents=True,exist_ok=True)
    metas=[]
    for name,implementation,options in implementations:
        metas.append(generate(output/(name+'.v'),name,implementation,options,emitter))
    ports=metas[0]
    for meta in metas:
        assert {k:v['width'] for k,v in meta['inputs'].items()}=={k:v['width'] for k,v in ports['inputs'].items()}
        assert {k:v['width'] for k,v in meta['outputs'].items()}=={k:v['width'] for k,v in ports['outputs'].items()}
    def packing(values, layout):
        result=0; shift=0
        for key,item in layout.items():
            value=values[key]; assert 0<=value<1<<item['width'],(key,value)
            result|=value<<shift; shift+=item['width']
        return result
    input_width=sum(p['width'] for p in ports['inputs'].values())
    output_width=sum(p['width'] for p in ports['outputs'].values())
    # Inputs sampled at t=3+10n, sys edge at t=5+10n, then the
    # pin-only target changes its pull signals. Outputs sampled at t=7+10n.
    inputs=[]; post_inputs=[]; expected=[]; seeds=[]
    for i,row in enumerate(rows):
        inputs.append(packing(row['inputs'],ports['inputs']))
        post_inputs.append(packing(row['post_inputs'],ports['inputs']))
        expected.append(packing(row['outputs'],ports['outputs']))
        seeds.append(0 if row['seed'] is None else (1<<32)|row['seed'])
    for name,values,width in [('inputs',inputs,input_width),('post_inputs',post_inputs,input_width),('expected',expected,output_width),('seeds',seeds,33)]:
        (output/(name+'.hex')).write_text(''.join(f'{value:0{(width+3)//4}x}\n' for value in values))
    lines=['module tb;','reg sys_clk=0;',f'reg [{input_width-1}:0] stimulus[0:{len(inputs)-1}];',
           f'reg [{input_width-1}:0] post_stimulus[0:{len(inputs)-1}];',
           f'reg [{output_width-1}:0] oracle[0:{len(inputs)-1}];',f'reg [32:0] seeds[0:{len(inputs)-1}];',
           f'reg [{input_width-1}:0] current_input;', 'integer cycle; integer acks=0; integer completions=0; integer pin_edges=0; reg[3:0] previous_pins=0;']
    offset=0
    for key,p in ports['inputs'].items():
        w=p['width'];lines.append(f'wire [{w-1}:0] {key}=current_input[{offset} +: {w}];');offset+=w
    for n,meta in enumerate(metas):
        for key,p in meta['outputs'].items(): lines.append(f'wire [{p["width"]-1}:0] d{n}_{key};')
        connections=['.'+meta['clock']['name']+'(sys_clk)']
        connections+=['.'+p['name']+'('+key+')' for key,p in meta['inputs'].items()]
        connections+=['.'+p['name']+f'(d{n}_{key})' for key,p in meta['outputs'].items()]
        lines.append(meta['module']+f' d{n}('+','.join(connections)+');')
        outputs=','.join(f'd{n}_{key}' for key in reversed(meta['outputs']))
        lines.append(f'wire [{output_width-1}:0] packed{n}={{{outputs}}};')
    lines+=['initial begin', '$readmemh("inputs.hex",stimulus); $readmemh("post_inputs.hex",post_stimulus); $readmemh("expected.hex",oracle); $readmemh("seeds.hex",seeds);',
            'current_input=0; #1;',f'for(cycle=0;cycle<{len(inputs)};cycle=cycle+1) begin',
            'sys_clk=0; current_input=stimulus[cycle]; #2;',
            'if(seeds[cycle][32]) begin']
    for n,meta in enumerate(metas):
        seed=meta['accepted_id_seed'];expr='seeds[cycle][31:0]'
        if seed['kind']=='dsp': expr="{5'b0,seeds[cycle][31:19],11'b0,seeds[cycle][18:0]}"
        lines += [f'if(d{n}_obs_ingress_active || d{n}_busy || sys_rst || wb_cyc || wb_stb || d{n}_obs_host_state) $fatal(1,"illegal seed precondition cycle=%0d",cycle);',
                  f'd{n}.{seed["lvalue"]}={expr};']
    lines+=['#1;', 'end', 'sys_clk=1; #1; current_input=post_stimulus[cycle]; #1;']
    for n in range(1,len(metas)):
        for key in ports['outputs']:
            lines.append(f'if(d{n}_{key} !== d0_{key}) $fatal(1,"parity {metas[n]["module"]} {key} cycle=%0d ref=%h got=%h",cycle,d0_{key},d{n}_{key});')
    # Check independent Migen outputs too. The first clock starts from initialized
    # signals, just as Migen does; no blanket reset/cancellation grace interval.
    offset=0
    for key,p in ports['outputs'].items():
        w=p['width']
        lines.append(f'if(d0_{key} !== oracle[cycle][{offset} +: {w}]) $fatal(1,"oracle {key} cycle=%0d expected=%h got=%h",cycle,oracle[cycle][{offset} +: {w}],d0_{key});')
        offset+=w
    lines+=['acks=acks+d0_wb_ack; completions=completions+d0_engine_done;',
            'if({d0_scl_oe0,d0_scl_oe1,d0_sda_oe0,d0_sda_oe1} != previous_pins) pin_edges=pin_edges+1;',
            'previous_pins={d0_scl_oe0,d0_scl_oe1,d0_sda_oe0,d0_sda_oe1};',
            'sys_clk=0; #2;', 'end',
            'if(acks<100 || completions<10 || pin_edges<500) $fatal(1,"vacuous coverage");',
            f'$display("PASS cycles=%0d acks=%0d completions=%0d pin_edges=%0d",{len(inputs)},acks,completions,pin_edges); $finish; end endmodule']
    (output/'tb.sv').write_text('\n'.join(lines)+'\n')
    command=['verilator','--binary','--timing','--top-module','tb','-Wno-fatal','--build-jobs','4','--Mdir','obj','-o','sim','tb.sv']+[Path(m['path']).name for m in metas]
    if vendor: command.append(str(Path(vendor).resolve()))
    with (output/'compile.log').open('w') as log:
        compiled=subprocess.run(command,cwd=output,stdout=log,stderr=subprocess.STDOUT)
    if compiled.returncode: raise RuntimeError(f'Compile failed: {output}/compile.log')
    with (output/'run.log').open('w') as log:
        simulated=subprocess.run([str(output/'obj/sim')],cwd=output,stdout=log,stderr=subprocess.STDOUT)
    result={'status':'PASS' if simulated.returncode==0 else 'FAIL','emitter':emitter,'cycles':len(inputs),
            'command':command,'returncode':simulated.returncode,'stdout':(output/'run.log').read_text(),
            'metadata':metas,'generated_sha256':{Path(m['path']).name:sha(m['path']) for m in metas},
            'vendor':{'path':str(vendor),'sha256':sha(vendor)} if vendor else None}
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    if simulated.returncode: raise RuntimeError(result['stdout'])
    return result
