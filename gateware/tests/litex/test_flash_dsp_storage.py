"""Native flash command identity storage preserves all bits and capture priority."""
from pathlib import Path
import re
import shutil
import subprocess

import pytest
from litex.build.efinix.toolchain import find_efinity_path
from tests.litex.test_flash_dsp_poll import _generate


@pytest.mark.skipif(shutil.which("verilator") is None, reason="Verilator required")
def test_native_flash_command_ids_preserve_width_replay_and_reset(tmp_path, monkeypatch):
    try:
        MODEL = Path(find_efinity_path()) / "sim_models/verilog/efx_dsp48.v"
    except OSError:
        pytest.skip("Set LITEX_ENV_EFINITY to validate the native DSP model")
    assert MODEL.is_file()
    monkeypatch.chdir(tmp_path)
    P = tmp_path
    base = _generate(P / "reference.v", "flash_ref", True)
    _generate(P / "candidate.v", "flash_dsp", True, use_dsp_storage=True)
    ports=re.findall(r'^\s*(input|output)\s+(?:reg\s+)?(?:signed\s+)?(\[\d+:\d+\])?\s*(\w+)\s*[,\n]',base.split(');',1)[0],re.M)
    ins={n:w for d,w,n in ports if d=='input'};outs={n:w for d,w,n in ports if d=='output'}
    assert len(outs)==9
    lines=['module tb;']
    for n,w in ins.items():lines.append(f'reg {w} {n}=0;')
    for n,w in outs.items():lines.append(f'wire {w} ref_{n}, dut_{n};')
    for module,prefix in [('flash_ref','ref'),('flash_dsp','dut')]:
     connections=[f'.{n}({n})' for n in ins]+[f'.{n}({prefix}_{n})' for n in outs]
     lines.append(f'{module} instance_{prefix}({", ".join(connections)});')
    lines+=['always #5 sys_clk=~sys_clk;',
    'integer cycles=0, transfers=0, idcases=0, priority_cases=0, resets=0;',
    'wire[31:0] ref_accepted=instance_ref.packetflashcontroller13;',
    'wire[31:0] dut_accepted=instance_dut.packetflashcontroller13;',
    'wire[31:0] ref_done=instance_ref.packetflashcontroller14;',
    'wire[31:0] dut_done=instance_dut.packetflashcontroller14;',
    'wire[31:0] ref_pending=instance_ref.packetflashcontroller_pending_id;',
    'wire[31:0] dut_pending=instance_dut.packetflashcontroller_pending_id;',
    'always @(posedge sys_clk) begin #1; cycles=cycles+1;',
    'if(ref_accepted!==dut_accepted || ref_done!==dut_done || ref_pending!==dut_pending) $fatal(1,"ID mismatch cycle=%0d accepted=%h/%h done=%h/%h pending=%h/%h", cycles,ref_accepted,dut_accepted,ref_done,dut_done,ref_pending,dut_pending);',
    'if(instance_ref.packetflashcontroller_poll_count!==instance_dut.packetflashcontroller_poll_count) $fatal(1,"poll counter mismatch");']
    for n in outs:lines.append(f'if(ref_{n}!==dut_{n}) $fatal(1,"{n} mismatch cycle=%0d ref=%h dut=%h",cycles,ref_{n},dut_{n});')
    lines+=['end',r'''
    function automatic [47:0] packed_id(input [31:0] value);
    begin packed_id={5'b0,value[31:19],11'b0,value[18:0]};end
    endfunction

    task seed_ids(input [31:0] accepted_value, done_value, pending_value);
    begin
    @(negedge sys_clk);
    instance_ref.packetflashcontroller13=accepted_value;
    instance_ref.packetflashcontroller14=done_value;
    instance_ref.packetflashcontroller_pending_id=pending_value;
    instance_dut.EFX_DSP48_1.genblk1.i_DSP48.W_pipe.O_r=packed_id(pending_value);
    instance_dut.EFX_DSP48_2.genblk1.i_DSP48.W_pipe.O_r=packed_id(accepted_value);
    instance_dut.EFX_DSP48_3.genblk1.i_DSP48.W_pipe.O_r=packed_id(done_value);
    end endtask

    task transfer(input bit write, input [29:0] address, input [31:0] value,
                  input bit first,last, input [1:0] source, input bit expected_error);
    integer waited;
    begin
    @(negedge sys_clk); bus_bus_adr=address;bus_bus_dat_w=value;bus_bus_cyc=1;bus_bus_stb=1;bus_bus_we=write;
    record_first=first;record_last=last;record_source=source;record_count=4;
    waited=0;
    do begin @(posedge sys_clk);#2;waited=waited+1;if(waited>30)$fatal(1,"transfer timeout");end
    while(!ref_bus_bus_ack && !ref_bus_bus_err);
    if(ref_bus_bus_err!==expected_error)$fatal(1,"unexpected error addr=%h data=%h actual=%b expected=%b",address,value,ref_bus_bus_err,expected_error);
    transfers=transfers+1;
    @(negedge sys_clk);bus_bus_cyc=0;bus_bus_stb=0;bus_bus_we=0;
    end endtask

    task word(input integer index,input [31:0] value);
    begin transfer(1,30'd256+30'(index),value,index==0,index==3,0,0);end endtask

    task command(input [31:0] id,input [31:0] descriptor);
    begin word(0,id);word(1,32'h400000);word(2,descriptor);word(3,id);end endtask

    task complete;
    integer waited;
    begin
    waited=0;while(ref_busy)begin @(negedge sys_clk);waited=waited+1;if(waited>15000)$fatal(1,"command completion timeout");end
    repeat(3)@(negedge sys_clk);
    end endtask

    task public_ids(input [31:0] accepted_value,done_value);
    begin
    transfer(0,7,0,0,0,0,0);
    if(ref_bus_bus_dat_r!==accepted_value)$fatal(1,"accepted CSR got=%h expected=%h",ref_bus_bus_dat_r,accepted_value);
    transfer(0,8,0,0,0,0,0);
    if(ref_bus_bus_dat_r!==done_value)$fatal(1,"done CSR got=%h expected=%h",ref_bus_bus_dat_r,done_value);
    end endtask

    task reset_all;
    begin @(negedge sys_clk);sys_rst=1;repeat(3)@(negedge sys_clk);sys_rst=0;resets=resets+1;end endtask

    task id_case(input [31:0] value);
    begin
    seed_ids(value-1,~value,32'hfedcba98);
    command(value,6);complete();public_ids(value,value);
    // Same full ID replay must not replace accepted/done or start a transaction.
    command(value,7);if(ref_busy)$fatal(1,"replay restarted command");public_ids(value,value);
    idcases=idcases+1;
    end endtask

    integer i;reg[31:0] target;reg[63:0] rng=64'h123456789abcdef1;
    initial begin
    slot=1;bus_bus_sel=15;record_count=4;flash_miso=0;reset_all();
    for(i=0;i<32;i=i+1)begin id_case(32'h1<<i);id_case(~(32'h1<<i));end
    id_case(32'h0007ffff);id_case(32'h00080000);id_case(32'h000fffff);id_case(32'h00100000);
    id_case(32'h7fffffff);id_case(32'h80000000);id_case(32'hfffffffe);id_case(32'hffffffff);
    // accepted==FFFFFFFF forbids wrapping to0; pending still captures the replay ID.
    command(0,6);if(ref_busy)$fatal(1,"wrapped ID accepted");public_ids(32'hffffffff,32'hffffffff);
    if(ref_pending!==0)$fatal(1,"replay pending not captured");
    // Invalid descriptor commits accepted/done atomically without starting SPI.
    seed_ids(32'h7fffffff,32'hdeadbeef,32'hfffff123);
    command(32'h80000000,7);if(ref_busy)$fatal(1,"invalid descriptor started SPI");public_ids(32'h80000000,32'h80000000);
    // Foreign first word cannot replace an owner; abort releases owner only.
    word(0,32'h80000001);
    transfer(1,256,32'h12345678,1,0,1,1);
    if(ref_pending!==32'h80000001)$fatal(1,"foreign word changed pending ID");
    @(negedge sys_clk);record_abort_sources=1;repeat(2)@(negedge sys_clk);record_abort_sources=0;
    public_ids(32'h80000000,32'h80000000);
    if(ref_pending!==32'h80000001)$fatal(1,"abort unexpectedly changed pending ID");
    command(32'h80000001,6);complete();public_ids(32'h80000001,32'h80000001);
    // Mismatched commit rejects without capturing accepted/done.
    word(0,32'h80000002);word(1,32'h400000);word(2,6);
    transfer(1,259,32'h80080002,0,1,0,1);public_ids(32'h80000001,32'h80000001);
    // Replay while executing captures full pending ID but leaves the in-flight ID.
    flash_miso=1;command(32'h80000002,6);
    if(!ref_busy)$fatal(1,"missing busy status poll");
    command(32'hdeadbeef,6);public_ids(32'h80000002,32'h80000001);
    if(ref_pending!==32'hdeadbeef)$fatal(1,"busy replay pending not captured");
    flash_miso=0;complete();public_ids(32'h80000002,32'h80000002);
    // Reset clears full IDs while idle and while a partial record is owned.
    seed_ids(32'hffffffff,32'h80080000,32'hf007ffff);reset_all();public_ids(0,0);
    word(0,1);reset_all();public_ids(0,0);if(ref_pending!==0)$fatal(1,"reset failed pending clear");
    // Force all meaningful complete/new-command/invalid combinations to verify
    // old-value capture and complete priority independently of normal FSM reachability.
    for(i=0;i<8;i=i+1)begin
    seed_ids(32'hfedcba98,32'h87654321,32'h8007ffff);
    force instance_ref.packetflashcontroller_complete=(i&1)!=0;
    force instance_dut.packetflashcontroller_complete=(i&1)!=0;
    force instance_ref.packetflashcontroller26=(i&2)!=0;
    force instance_dut.packetflashcontroller26=(i&2)!=0;
    force instance_ref.packetflashcontroller_invalid_descriptor=(i&4)!=0;
    force instance_dut.packetflashcontroller_invalid_descriptor=(i&4)!=0;
    @(posedge sys_clk);#2;
    if(ref_accepted!==((i&2)?32'h8007ffff:32'hfedcba98))$fatal(1,"accepted capture priority");
    if(ref_done!==((i&1)?32'hfedcba98:(((i&6)==6)?32'h8007ffff:32'h87654321)))$fatal(1,"done capture priority");
    @(negedge sys_clk);
    release instance_ref.packetflashcontroller_complete;release instance_dut.packetflashcontroller_complete;
    release instance_ref.packetflashcontroller26;release instance_dut.packetflashcontroller26;
    release instance_ref.packetflashcontroller_invalid_descriptor;release instance_dut.packetflashcontroller_invalid_descriptor;
    priority_cases=priority_cases+1;
    reset_all();
    end
    // Global reset has priority over a simultaneous pending capture.
    seed_ids(32'ha0000000,32'hffffffff,32'hf007ffff);
    bus_bus_cyc=1;bus_bus_stb=1;bus_bus_we=1;bus_bus_adr=256;bus_bus_dat_w=32'ha0000001;
    record_first=1;record_last=0;record_count=4;record_source=0;sys_rst=1;
    @(posedge sys_clk);#2;
    if(ref_accepted!==0 || ref_done!==0 || ref_pending!==0)$fatal(1,"reset lost to pending capture");
    @(negedge sys_clk);bus_bus_cyc=0;bus_bus_stb=0;sys_rst=0;resets=resets+1;
    // Global reset also wins over accepted/done capture enables.
    seed_ids(32'hfedcba98,32'h87654321,32'h8007ffff);
    force instance_ref.packetflashcontroller_complete=1;
    force instance_dut.packetflashcontroller_complete=1;
    force instance_ref.packetflashcontroller26=1;
    force instance_dut.packetflashcontroller26=1;
    force instance_ref.packetflashcontroller_invalid_descriptor=1;
    force instance_dut.packetflashcontroller_invalid_descriptor=1;
    sys_rst=1;
    @(posedge sys_clk);#2;
    if(ref_accepted!==0 || ref_done!==0 || ref_pending!==0)$fatal(1,"reset lost to ID capture");
    @(negedge sys_clk);sys_rst=0;
    release instance_ref.packetflashcontroller_complete;release instance_dut.packetflashcontroller_complete;
    release instance_ref.packetflashcontroller26;release instance_dut.packetflashcontroller26;
    release instance_ref.packetflashcontroller_invalid_descriptor;release instance_dut.packetflashcontroller_invalid_descriptor;
    resets=resets+1;
    // Random low-level traffic and resets compares all9publicoutputs plus IDs.
    for(i=0;i<10000;i=i+1)begin
    @(negedge sys_clk);rng={rng[62:0],rng[63]^rng[62]^rng[60]^rng[59]};
    sys_rst=(rng[39:32]==0);inhibit=rng[12];record_abort_sources=rng[16:13];record_abort=rng[17];
    record_first=rng[18];record_last=rng[19];record_count=rng[28:20];record_source=rng[30:29];
    bus_bus_cyc=rng[0];bus_bus_stb=rng[1];bus_bus_we=rng[2];bus_bus_adr=rng[32:3];bus_bus_dat_w=rng[63:32];bus_bus_sel=rng[6:3];flash_miso=rng[31];
    end
    if(idcases!=72 || priority_cases!=8 || transfers<900)$fatal(1,"insufficient coverage ids=%0d transfers=%0d priorities=%0d",idcases,transfers,priority_cases);
    $display("PASS cycles=%0d full_id_cases=%0d transfers=%0d priority_cases=%0d directed_resets=%0d all9outputs+IDs+poll",cycles,idcases,transfers,priority_cases,resets);
    $finish;end
    endmodule
    ''']
    (P/'tb.sv').write_text('\n'.join(lines)+'\n')
    cmd=['verilator','--binary','--timing','-Wno-fatal','--top-module','tb','--Mdir',str(P/'obj'),str(P/'tb.sv'),str(P/'reference.v'),str(P/'candidate.v'),str(MODEL),'-j','4']
    c=subprocess.run(cmd,cwd=P,capture_output=True,text=True);(P/'compile.log').write_text(c.stdout+c.stderr);assert c.returncode==0,(c.stdout+c.stderr)[-6000:]
    c=subprocess.run([str(P/'obj/Vtb')],cwd=P,capture_output=True,text=True);(P/'run.log').write_text(c.stdout+c.stderr);print(c.stdout+c.stderr);assert c.returncode==0
