"""Native flash timeout counter parity at the production 600M-cycle limit."""
from pathlib import Path
import re
import shutil
import subprocess

import pytest
from litex.build.efinix.toolchain import find_efinity_path
from migen import Signal
from migen.fhdl import verilog
from migen.fhdl.simplify import FullMemoryWE

from fpga.litex.lutcam.flash_packet import PacketFlashController


def _generate(path, name, use_dsp_poll, *, use_dsp_storage=False):
    slot = Signal(3, name_override="slot")
    dut = PacketFlashController(boot_slot=slot, use_dsp_poll=use_dsp_poll,
                               use_dsp_storage=use_dsp_storage)
    ports = {slot}
    for signal in dut.bus.flatten():
        signal.name_override = "bus_" + signal.backtrace[-1][0]
        ports.add(signal)
    for field in ("inhibit", "busy", "record_first", "record_last", "record_abort",
                  "record_count", "record_source", "record_abort_sources"):
        signal = getattr(dut, field)
        signal.name_override = field
        ports.add(signal)
    for field in ("clk", "cs_n", "mosi", "miso", "wp", "hold"):
        signal = getattr(dut.pads, field)
        signal.name_override = "flash_" + field
        ports.add(signal)
    FullMemoryWE()(dut)
    verilog.convert(dut.get_fragment(), ios=ports, name=name).write(str(path))
    return path.read_text()


@pytest.mark.skipif(shutil.which("verilator") is None, reason="Verilator required")
def test_native_flash_poll_counter_preserves_timeout_and_reset(tmp_path, monkeypatch):
    try:
        model = Path(find_efinity_path()) / "sim_models/verilog/efx_dsp48.v"
    except OSError:
        pytest.skip("Set LITEX_ENV_EFINITY to validate the native DSP model")
    assert model.is_file()
    # Both models use identically initialized page RAM; keep generated init
    # files and compilation private to this test.
    monkeypatch.chdir(tmp_path)
    P = tmp_path
    base = _generate(P / "reference.v", "flash_ref", False)
    _generate(P / "candidate.v", "flash_dsp", True)
    ports=re.findall(r'^\s*(input|output)\s+(?:reg\s+)?(?:signed\s+)?(\[\d+:\d+\])?\s*(\w+)\s*[,\n]',base.split(');',1)[0],re.M)
    ins={n:w for d,w,n in ports if d=='input'};outs={n:w for d,w,n in ports if d=='output'}
    assert len(outs)==9
    lines=['module tb;']
    for n,w in ins.items():lines.append(f'reg {w} {n}=0;')
    for n,w in outs.items():lines.append(f'wire {w} ref_{n}, dut_{n};')
    for module,prefix in [('flash_ref','ref'),('flash_dsp','dut')]:
     connections=[f'.{n}({n})' for n in ins]+[f'.{n}({prefix}_{n})' for n in outs]
     lines.append(f'{module} instance_{prefix}({", ".join(connections)});')
    lines += ['always #5 sys_clk=~sys_clk;','integer cycles=0, saturations=0, cleared=0;',
    'wire [29:0] ref_count=instance_ref.packetflashcontroller_poll_count;',
    'wire [29:0] dut_count=instance_dut.packetflashcontroller_poll_count;',
    'always @(posedge sys_clk) begin #1; cycles=cycles+1;',
    'if(ref_count!==dut_count) $fatal(1,"counter mismatch cycle=%0d ref=%h dut=%h",cycles,ref_count,dut_count);',
    'if(ref_count==599999999) saturations=saturations+1;',
    'if(ref_count==0) cleared=cleared+1;']
    for n in outs:lines.append(f'if(ref_{n}!==dut_{n}) $fatal(1,"{n} mismatch cycle=%0d ref=%h dut=%h",cycles,ref_{n},dut_{n});')
    lines += ['end',
    '''task word(input integer index, input [31:0] value);
    integer waited;
    begin
    @(negedge sys_clk); bus_bus_adr=256+30'(index); bus_bus_dat_w=value;
    bus_bus_cyc=1; bus_bus_stb=1; bus_bus_we=1; record_first=(index==0); record_last=(index==3);
    waited=0;
    do begin @(posedge sys_clk); #2; waited=waited+1; if(waited>20) $fatal(1,"word timeout"); end while(!ref_bus_bus_ack && !ref_bus_bus_err);
    if(ref_bus_bus_err) $fatal(1,"word unexpected err");
    @(negedge sys_clk); bus_bus_cyc=0;bus_bus_stb=0;bus_bus_we=0;
    end endtask

    task command(input [31:0] id, input [31:0] descriptor);
    begin word(0,id);word(1,32'h400000);word(2,descriptor);word(3,id);end endtask

    task seed(input [29:0] value);
    begin
    @(negedge sys_clk);
    instance_ref.packetflashcontroller_poll_count=value;
    instance_dut.EFX_DSP48.genblk1.i_DSP48.W_pipe.O_r={18'h3c57a,value};
    end endtask

    task complete;
    integer waited;
    begin
    waited=0;
    while(ref_busy)begin @(negedge sys_clk);waited=waited+1;if(waited>15000)$fatal(1,"command completion timeout");end
    repeat(3)@(negedge sys_clk);
    end endtask

    integer i;reg [63:0] rng=64'h8123456789abcdef;
    initial begin
    slot=1;bus_bus_sel=15;record_count=4;
    sys_rst=1;repeat(3)@(negedge sys_clk);sys_rst=0;
    // Reset wins over disabled CE at the saturation value, including while idle.
    seed(599999999);repeat(3)@(negedge sys_clk);
    flash_miso=1;command(1,6);
    if(!ref_busy || !instance_ref.packetflashcontroller_polling)$fatal(1,"missing initial poll");
    seed(599999994);complete();
    // Exercise successful initial poll and a full page, followed by a new poll.
    flash_miso=0;command(2,32'h10001);complete();command(3,5);complete();
    flash_miso=1;command(4,6);
    for(i=0;i<30;i=i+1)begin seed(30'd1<<i);repeat(4)@(negedge sys_clk);end
    seed(599999999);complete();
    // Reset with a saturated polling counter then repeat real commands.
    command(5,6);seed(599999999);sys_rst=1;repeat(3)@(negedge sys_clk);sys_rst=0;
    flash_miso=0;command(1,6);complete();
    for(i=0;i<40000;i=i+1)begin
    @(negedge sys_clk);rng={rng[62:0],rng[63]^rng[62]^rng[60]^rng[59]};
    sys_rst=(rng[39:32]==0);inhibit=rng[12];record_abort_sources=rng[16:13];record_abort=rng[17];
    record_first=rng[18];record_last=rng[19];record_count=rng[28:20];record_source=rng[30:29];
    bus_bus_cyc=rng[0];bus_bus_stb=rng[1];bus_bus_we=rng[2];bus_bus_adr=rng[32:3];bus_bus_dat_w=rng[63:32];bus_bus_sel=rng[6:3];flash_miso=rng[31];
    end
    if(cycles<40000 || saturations<10 || cleared<1000)$fatal(1,"insufficient coverage cycles=%0d saturation=%0d clear=%0d",cycles,saturations,cleared);
    $display("PASS cycles=%0d saturation=%0d clear=%0d all9outputs+pollcounter",cycles,saturations,cleared);
    $finish;end
    endmodule''']
    (P/'tb.sv').write_text('\n'.join(lines)+'\n')
    cmd=['verilator','--binary','--timing','-Wno-fatal','--top-module','tb','--Mdir',str(P/'obj'),str(P/'tb.sv'),str(P/'reference.v'),str(P/'candidate.v'),str(model),'-j','4']
    c=subprocess.run(cmd,cwd=P,capture_output=True,text=True);(P/'compile.log').write_text(c.stdout+c.stderr);assert c.returncode==0,(c.stdout+c.stderr)[-4000:]
    c=subprocess.run([str(P/'obj/Vtb')],cwd=P,capture_output=True,text=True);(P/'run.log').write_text(c.stdout+c.stderr);print(c.stdout+c.stderr);assert c.returncode==0
