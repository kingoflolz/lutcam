"""Whole UART protocol proof for the optional compact datapath/native timer.

Use the real Efinity DSP model, not a replacement accumulator. The generic core
is a differential oracle, supplemented with independent command/data scoreboards.
Only inactive data and read-side Wishbone dat_w are intentionally unconstrained.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
from migen import Cat, ClockDomain, Module, Signal
from migen.fhdl import verilog as migen_verilog
from litex.gen.fhdl import verilog as litex_verilog
from litex.soc.interconnect import stream
from litex.build.efinix.toolchain import find_efinity_path

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'fpga/litex/lutcam/record_bus.py').is_file())
sys.path.insert(0, str(ROOT/'fpga/litex'))
SOURCE = Path(os.environ.get('LUTCAM_RECORD_BUS_SOURCE', ROOT/'fpga/litex/lutcam/record_bus.py'))
REFERENCE = Path(os.environ.get('LUTCAM_RECORD_BUS_REFERENCE', SOURCE))


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def emit_core(path, name, emitter, width, options, source):
    subject = load(source, name+'_source')
    phy = Module()
    phy.source = stream.Endpoint([('data', 8)])
    phy.sink = stream.Endpoint([('data', 8), ('length', 11)])
    core = subject.RecordUARTBone(phy=phy, clk_freq=120000000,
        timeout_cycles=8192, address_width=width, **options)
    fields = dict(rx_data=core.sink.data, rx_valid=core.sink.valid,
        rx_ready=core.sink.ready, tx_data=core.source.data,
        tx_valid=core.source.valid, tx_ready=core.source.ready,
        tx_last=core.source.last, tx_length=core.source.length,
        guard_fault=core.guard_fault, command_reset=core.command_reset)
    for field in ('adr','dat_w','dat_r','we','sel','cyc','stb','ack','err'):
        fields['wb_'+field] = getattr(core.wishbone,field)
    for field in ('record_first','record_last','record_count','record_source','record_abort'):
        fields[field] = getattr(core,field)
    phase = Signal(7)
    core.comb += phase.eq(Cat(*(core.fsm.ongoing(state) for state in
        ('RECEIVE-CMD','RECEIVE-LENGTH','RECEIVE-ADDRESS','RECEIVE-DATA',
         'WRITE-DATA','READ-DATA','SEND-DATA'))))
    fields['phase'] = phase
    core.clock_domains.cd_sys=ClockDomain('sys')
    fields.update(sys_clk=core.cd_sys.clk,sys_rst=core.cd_sys.rst)
    for key, signal in fields.items():signal.name_override=key
    converter = migen_verilog if emitter == 'migen' else litex_verilog
    result = converter.convert(core, ios=set(fields.values()), name=name)
    path.write_text(str(result) if emitter=='migen' else result.main_source)


def bench(width):
    instances=[]
    outputs = [('rx_ready',1),('tx_data',8),('tx_valid',1),('tx_last',1),('tx_length',11),
        ('wb_adr',width-2),('wb_dat_w',32),('wb_we',1),('wb_sel',4),('wb_cyc',1),('wb_stb',1),
        ('record_first',1),('record_last',1),('record_count',9),('record_source',2),('record_abort',1),('phase',7)]
    for prefix in ('reference','portable','native'):
        wires='\n'.join('wire '+(f'[{bits-1}:0] ' if bits>1 else '')+prefix+'_'+name+';' for name,bits in outputs)
        common='.sys_clk(clk),.sys_rst(rst),.rx_data(rx_data),.rx_valid(rx_valid),.tx_ready(tx_ready),.wb_ack(wb_ack),.wb_err(wb_err),.wb_dat_r(wb_dat_r),.command_reset(command_reset),.guard_fault(guard_fault)'
        instances.append(wires+'\n'+prefix+' dut_'+prefix+'('+common+','+','.join('.'+name+'('+prefix+'_'+name+')' for name,bits in outputs)+');')
    strict=[name for name,_ in outputs if name not in ('tx_data','wb_dat_w')]
    compare=''
    for prefix in ('portable','native'):
        for name in strict:compare += f'if ({prefix}_{name} !== reference_{name}) $fatal(1,"{prefix} {name} cycle=%0d phase=%0h", cycles,reference_phase);\n'
        compare += f'if(reference_tx_valid && {prefix}_tx_data !== reference_tx_data) $fatal(1,"{prefix} valid byte cycle=%0d",cycles);\n'
        compare += f'if(reference_wb_cyc && reference_wb_stb && reference_wb_we && {prefix}_wb_dat_w !== reference_wb_dat_w) $fatal(1,"{prefix} write data cycle=%0d",cycles);\n'
    template = r'''
module tb;
localparam AW=@WIDTH@, TIMEOUT=8192;
reg clk=0,rst=1,rx_valid=0,command_reset=0,guard_fault=0;
reg [7:0] rx_data=0;
reg [31:0] random_bits=32'h91a3b74d;
reg automatic_io=1, manual_bus_ready=0, manual_tx_ready=0, force_ack=0, wb_err=0;
wire tx_ready = automatic_io ? (random_bits[0] | random_bits[1]) : manual_tx_ready;
wire wb_ack = force_ack | (reference_wb_cyc & reference_wb_stb &
    (automatic_io ? (random_bits[3] | random_bits[4]) : manual_bus_ready));
wire [31:0] wb_dat_r = read_word(reference_wb_adr);
integer cycles=0,checked_commands=0,checked_words=0,checked_bytes=0,reset_cases=0,timeout_cases=0;
integer expected_count=0,words_seen=0,bytes_seen=0,word_index,byte_index;
reg [7:0] expected_opcode;
reg [63:0] expected_address;
reg [31:0] expected_seed;
reg checking=0;
reg [31:0] expected_value;
reg [63:0] expected_word_address;
@INSTANCES@
function automatic [31:0] write_word(input [31:0] seed,input integer index);
    write_word = seed ^ (32'h9e3779b9 * index) ^ (index << 19);
endfunction
function automatic [31:0] read_word(input [63:0] address);
    read_word = address[31:0] ^ address[63:32] ^ 32'hbad1cafe;
endfunction
function automatic [63:0] bus_address(input [63:0] address);
    bus_address = address & ((64'h1 << (AW-2))-1);
endfunction
always #5 clk=~clk;
always @(posedge clk) begin
    random_bits <= {random_bits[30:0],random_bits[31]^random_bits[21]^random_bits[1]^random_bits[0]};
    cycles=cycles+1;
    if(!rst) begin
        @COMPARE@
        if(checking) begin
            if(reference_record_abort && reference_phase!=1) $fatal(1,"unexpected abort during ordinary command");
            if(reference_wb_cyc && reference_wb_stb && wb_ack) begin
                if(words_seen >= expected_count) $fatal(1,"extra word");
                expected_word_address=bus_address(expected_address+
                    ((expected_opcode==1 || expected_opcode==2) ? words_seen : 0));
                if(reference_wb_adr !== expected_word_address[AW-3:0] ||
                   reference_wb_we !== (expected_opcode==1 || expected_opcode==3) ||
                   reference_record_count != expected_count || reference_record_source != 1 ||
                   reference_record_first != (words_seen==0) ||
                   reference_record_last != (words_seen==expected_count-1))
                    $fatal(1,"independent word scoreboard opcode=%0d index=%0d",expected_opcode,words_seen);
                if(reference_wb_we && reference_wb_dat_w !== write_word(expected_seed,words_seen))
                    $fatal(1,"independent write word");
                words_seen=words_seen+1;checked_words=checked_words+1;
            end
            if(reference_tx_valid && tx_ready) begin
                if(expected_opcode!=2 && expected_opcode!=4) $fatal(1,"write emitted response");
                word_index=bytes_seen/4;byte_index=bytes_seen%4;
                expected_word_address=bus_address(expected_address+(expected_opcode==2 ? word_index : 0));
                expected_value=read_word(expected_word_address);
                if(reference_tx_data !== ((expected_value >> (24-8*byte_index)) & 8'hff) ||
                   reference_tx_last != (bytes_seen==4*expected_count-1) ||
                   reference_tx_length != 4*expected_count) $fatal(1,"independent response byte %0d",bytes_seen);
                bytes_seen=bytes_seen+1;checked_bytes=checked_bytes+1;
            end
        end
    end
end
task automatic step(input integer count); repeat(count) @(negedge clk); endtask
task automatic send_byte(input [7:0] value);
    integer guard;
    begin
        @(negedge clk);rx_data=value;rx_valid=1;guard=0;
        @(posedge clk);
        while(!reference_rx_ready) begin
            @(posedge clk);guard=guard+1;if(guard>TIMEOUT+10) $fatal(1,"byte input stalled");
        end
        @(negedge clk);rx_valid=0;
    end
endtask
task automatic header(input [7:0] opcode,input [7:0] count,input [63:0] address);
    integer i;
    begin send_byte(opcode);send_byte(count);
        for(i=AW/8-1;i>=0;i=i-1) send_byte(address>>(8*i));
    end
endtask
task automatic wait_idle;
    integer guard;
    begin guard=0;
        while(reference_phase != 1) begin
            @(negedge clk);guard=guard+1;if(guard>TIMEOUT+10) $fatal(1,"not idle");
        end
        step(3);
    end
endtask
task automatic record(input [7:0] opcode,input integer count,input [63:0] address,input [31:0] seed);
    integer i,j;reg [31:0] value;
    begin
        automatic_io=1;checking=1;expected_opcode=opcode;expected_count=count;
        expected_address=address;expected_seed=seed;words_seen=0;bytes_seen=0;
        header(opcode,count==256 ? 0 : count,address);
        if(opcode==1 || opcode==3) for(i=0;i<count;i=i+1) begin
            value=write_word(seed,i);
            for(j=3;j>=0;j=j-1) send_byte(value>>(8*j));
        end
        wait_idle();
        if(words_seen!=count || bytes_seen!=((opcode==2 || opcode==4)?4*count:0))
            $fatal(1,"wrong record totals %0d %0d count=%0d",words_seen,bytes_seen,count);
        checking=0;checked_commands=checked_commands+1;
    end
endtask
task automatic arm_phase(input integer phase);
    begin
        automatic_io=0;manual_bus_ready=0;manual_tx_ready=0;
        if(phase!=0) begin
            send_byte((phase==5 || phase==6)?2:1);
            if(phase!=1) begin
                send_byte(2);
                if(phase!=2) begin
                    for(integer i=AW/8-1;i>=0;i=i-1) send_byte(64'h123456789abcdef0>>(8*i));
                    if(phase==4) begin send_byte(8'hff);send_byte(8'h55);send_byte(8'haa);send_byte(8'h1);end
                    if(phase==6) begin
                        manual_bus_ready=1;
                        while(!reference_tx_valid) @(negedge clk);
                        manual_bus_ready=0;
                    end
                end
            end
        end
        step(1);
        if(reference_phase !== (7'b1<<phase)) $fatal(1,"arm phase %0d got %0h",phase,reference_phase);
    end
endtask
integer op,n,p,k,seen_abort;
initial begin
    step(4);rst=0;step(4);
    // Every opcode, preserving the full address collection for invalid bytes.
    for(op=0;op<256;op=op+1) begin
        if(op>=1 && op<=4) record(op,1,64'hfedcba9876543210,op*32'h1234567);
        else begin
            automatic_io=1;header(op,8'hff,64'hff10203040506070);wait_idle();
            if(reference_wb_cyc || reference_tx_valid) $fatal(1,"invalid opcode executed");
        end
    end
    // Full 256-word range for the deployed width; boundary lengths elsewhere.
    for(op=1;op<=4;op=op+1) for(n=1;n<=256;n=n+1)
        if(AW==32 || n==1 || n==2 || n==4 || n==68 || n==255 || n==256)
            record(op,n,64'hfffffffffffffffd,n*32'ha5366b4d+op);
    // Reset and timeout in every parser/bus/response phase, including held data.
    for(p=0;p<7;p=p+1) for(k=0;k<2;k=k+1) begin
        arm_phase(p);
        @(negedge clk);rx_valid=1;rx_data=8'hff;guard_fault=1;force_ack=1;
        if(k==0) command_reset=1;else rst=1;
        step(3);
        rx_valid=0;guard_fault=0;force_ack=0;command_reset=0;rst=0;
        wait_idle();reset_cases=reset_cases+1;record(2,2,64'h33,32'hdcba0123);
    end
    for(p=1;p<7;p=p+1) begin
        arm_phase(p);seen_abort=0;
        for(k=0;k<TIMEOUT+4;k=k+1) begin
            @(negedge clk);if(reference_record_abort) seen_abort=1;
        end
        if(!seen_abort || reference_phase!=1 || reference_wb_cyc || reference_tx_valid)
            $fatal(1,"timeout did not abort phase %0d",p);
        timeout_cases=timeout_cases+1;record(1,2,64'h3f,32'h84c7a926);
    end
    // A guard fault poisons the record across byte/bus gaps, then clears on CMD.
    arm_phase(6);guard_fault=1;step(1);guard_fault=0;step(2);
    if(!reference_record_abort) $fatal(1,"guard poison not sticky");
    automatic_io=1;wait_idle();
    record(4,2,64'h12,32'h87f6d5e4);
    $display("PASS UART width=%0d cycles=%0d commands=%0d words=%0d bytes=%0d resets=%0d timeouts=%0d",
        AW,cycles,checked_commands,checked_words,checked_bytes,reset_cases,timeout_cases);
    $finish;
end
initial #100000000 $fatal(1,"test timeout");
endmodule
'''
    return template.replace('@WIDTH@',str(width)).replace('@INSTANCES@','\n'.join(instances)).replace('@COMPARE@',compare)


@pytest.mark.skipif(shutil.which('verilator') is None, reason='Verilator required')
@pytest.mark.parametrize('emitter',['migen','litex'])
@pytest.mark.parametrize('width',[8,16,32,64])
def test_compact_and_native_uart_preserve_complete_protocol(tmp_path,emitter,width):
    try:model=Path(find_efinity_path())/'sim_models/verilog/efx_dsp48.v'
    except OSError:pytest.skip('Efinity native DSP model required')
    assert model.is_file()
    for name,options,source in [('reference',{},REFERENCE),
        ('portable',dict(compact_datapath=True),SOURCE),
        ('native',dict(compact_datapath=True,use_dsp_timeout=True),SOURCE)]:
        emit_core(tmp_path/(name+'.v'),name,emitter,width,options,source)
    (tmp_path/'tb.sv').write_text(bench(width))
    command=['verilator','--binary','--timing','-Wno-fatal','--top-module','tb',
        '--Mdir',str(tmp_path/'obj'),'-j','2',str(tmp_path/'tb.sv'),
        *(str(tmp_path/(name+'.v')) for name in ('reference','portable','native')),str(model)]
    compiled=subprocess.run(command,capture_output=True,text=True)
    (tmp_path/'compile.log').write_text(compiled.stdout+compiled.stderr)
    assert compiled.returncode==0,compiled.stdout+compiled.stderr
    result=subprocess.run([str(tmp_path/'obj/Vtb')],capture_output=True,text=True)
    (tmp_path/'run.log').write_text(result.stdout+result.stderr)
    assert result.returncode==0,result.stdout+result.stderr
    assert 'PASS UART' in result.stdout
    report=dict(emitter=emitter,width=width,status='PASS',output=result.stdout,
        source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        reference_sha256=hashlib.sha256(REFERENCE.read_bytes()).hexdigest(),
        vendor_sha256=hashlib.sha256(model.read_bytes()).hexdigest())
    (tmp_path/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
