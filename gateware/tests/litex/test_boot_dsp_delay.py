"""The optional boot DSP delay preserves reboot timing and sequential identity."""

import inspect
from pathlib import Path
import re
import shutil
import subprocess

import pytest
from litex.build.efinix.toolchain import find_efinity_path
from litex.gen.fhdl import verilog
from migen import ClockDomain, Instance, Module

from fpga.litex.lutcam.boot import BootControl
from fpga.litex.lutcam.image_identity import ImageIdentity, identity_words


BUILD_ID = "0123456789abcdeffedcba9876543210"


def test_boot_dsp_delay_is_explicitly_optional():
    assert inspect.signature(BootControl).parameters["use_dsp_delay"].default is False
    for options, expected in (({}, 0), ({"use_dsp_delay": True}, 1)):
        fragment = BootControl(**options).get_fragment()
        native = [item for item in fragment.specials
                  if isinstance(item, Instance) and item.of == "EFX_DSP48"]
        assert len(native) == expected


def _generate(directory, name, delay, use_dsp):
    dut = Module()
    dut.submodules.identity = identity = ImageIdentity(
        build_id=BUILD_ID, init_words=identity_words(BUILD_ID, "B"),
        sequential_build_id=True)
    dut.submodules.boot = boot = BootControl(
        identity=identity, delay_cycles=delay, use_dsp_delay=use_dsp)
    dut.clock_domains.cd_sys = ClockDomain("sys")
    dut.clock_domains.cd_cam_ref = ClockDomain("cam_ref")
    ports = {f"bus_{field}": getattr(boot.bus, field)
             for field in ("adr", "dat_w", "dat_r", "we", "sel", "cyc", "stb", "ack", "err")}
    ports.update({name: getattr(boot, name) for name in
                  ("flash_busy", "pending", "ru_error", "golden_status",
                   "auto_request", "auto_slot", "auto_ready")})
    ports.update({f"pins_{name}": signal for name, signal in vars(boot.pins).items()})
    ports.update(identity_ready=identity.ready, identity_valid=identity.valid,
                 identity_slot=identity.image_slot, identity_data=identity.build_id_data)
    for domain in ("sys", "cam_ref"):
        cd = getattr(dut, "cd_" + domain)
        ports.update({domain + "_clk": cd.clk, domain + "_rst": cd.rst})
    for port, signal in ports.items():
        signal.name_override = port
    generated = verilog.convert(dut, ios=set(ports.values()), name=name)
    (directory / (name + ".v")).write_text(generated.main_source)
    for filename, data in generated.data_files.items():
        (directory / filename).write_text(data)
    return generated.main_source


@pytest.mark.parametrize("delay", [1, 2, 37])
@pytest.mark.skipif(shutil.which("verilator") is None, reason="Verilator required")
def test_native_boot_delay_matches_fabric_with_sequential_identity(tmp_path, delay):
    try:
        model = Path(find_efinity_path()) / "sim_models/verilog/efx_dsp48.v"
    except OSError:
        pytest.skip("Set LITEX_ENV_EFINITY to validate the native DSP model")
    assert model.is_file()
    reference = _generate(tmp_path, "reference", delay, False)
    candidate = _generate(tmp_path, "candidate", delay, True)
    assert "EFX_DSP48" not in reference
    assert candidate.count("EFX_DSP48 #(") == 1
    ports = re.findall(
        r"^\s*(input|output)\s+(?:(?:reg|wire)\s+)?(?:signed\s+)?(\[\d+:\d+\])?\s*(\w+)\s*[,\n]",
        reference.split(");", 1)[0], re.M)
    inputs = {name: width for direction, width, name in ports if direction == "input"}
    outputs = {name: width for direction, width, name in ports if direction == "output"}
    lines = ["`timescale 1ns/1ps", "module tb;", f"localparam integer DELAY={delay};"]
    lines += [f"reg {width} {name}=0;" for name, width in inputs.items()]
    lines += [f"wire {width} {prefix}_{name};"
              for name, width in outputs.items() for prefix in ("ref", "dut")]
    for module, prefix in (("reference", "ref"), ("candidate", "dut")):
        connections = [f".{name}({name})" for name in inputs]
        connections += [f".{name}({prefix}_{name})" for name in outputs]
        lines.append(f"{module} instance_{prefix}({', '.join(connections)});")
    lines += [r"""
always #5 sys_clk=~sys_clk;
always #24 cam_ref_clk=~cam_ref_clk;
integer checks=0,acks=0,errors=0,configs=0,age=0,normal_age=-1;
reg checking=0,previous_pending=0,previous_config=0,block_on_ack=0,normal_boot=0;
reg[31:0] last_read;
task transfer(input[29:0]address,input wr,input[31:0]value,input expected_error,input integer hold_cycles);
integer waited;
begin
 @(negedge sys_clk);bus_adr=address;bus_we=wr;bus_dat_w=value;
 bus_sel=15;bus_cyc=1;bus_stb=1;waited=0;
 begin: response_wait
  forever begin
   @(posedge sys_clk);#2;waited=waited+1;
   if(ref_bus_ack||ref_bus_err)begin
    if(ref_bus_err!==expected_error || (ref_bus_ack&&ref_bus_err))
     $fatal(1,"unexpected bus response address=%h",address);
    last_read=ref_bus_dat_r;
    if(block_on_ack)begin flash_busy=1;block_on_ack=0;end
    disable response_wait;
   end
   if(waited>100)$fatal(1,"bus timeout");
  end
 end
 repeat(hold_cycles)@(negedge sys_clk);
 @(negedge sys_clk);bus_cyc=0;bus_stb=0;
 repeat(3)@(negedge sys_clk);
end
endtask
task reset_sys;
begin
 @(negedge sys_clk);sys_rst=1;bus_cyc=0;bus_stb=0;auto_request=0;
 repeat(8)@(negedge sys_clk);sys_rst=0;
 repeat(40)@(negedge sys_clk);
 if(ref_pending||ref_pins_CONFIG||!ref_identity_valid)$fatal(1,"reset did not restore idle identity");
end
endtask
task wait_config;
integer waited;
begin
 waited=0;
 while(!ref_pins_CONFIG)begin
  @(negedge sys_clk);waited=waited+1;
  if(waited>DELAY+100)$fatal(1,"configuration timeout");
 end
end
endtask
always @(posedge sys_clk)begin
 #1;
 if(ref_bus_ack)acks=acks+1;if(ref_bus_err)errors=errors+1;
 if(sys_rst||!ref_pending||!previous_pending)age=0;else age=age+1;
 previous_pending=ref_pending;
 if(checking&&ref_pins_CONFIG&&!previous_config)begin
  configs=configs+1;
  if(age<DELAY)$fatal(1,"configuration before delay expired");
  if(normal_boot)normal_age=age;
 end
 previous_config=ref_pins_CONFIG;
end
initial begin
 sys_rst=1;cam_ref_rst=1;bus_sel=15;
 repeat(30)@(negedge sys_clk);sys_rst=0;cam_ref_rst=0;checking=1;
 if(ref_identity_ready)$fatal(1,"early sequential identity read was not exercised");
 transfer(4,0,0,0,0);if(last_read!==32'h01234567)$fatal(1,"build word 0");
 transfer(5,0,0,0,0);if(last_read!==32'h89abcdef)$fatal(1,"build word 1");
 transfer(6,0,0,0,0);if(last_read!==32'hfedcba98)$fatal(1,"build word 2");
 transfer(7,0,0,0,0);if(last_read!==32'h76543210)$fatal(1,"build word 3");
 if(!ref_identity_valid||ref_identity_slot!==2)$fatal(1,"identity not validated");
 transfer(2,0,0,0,0);if(last_read!==2)$fatal(1,"identity slot readback");
 flash_busy=1;transfer(9,1,6,1,0);flash_busy=0;

 // Busy is asserted on acceptance, before even delay=1 can trigger.
 block_on_ack=1;transfer(9,1,6,0,8);
 transfer(9,1,6,0,0);transfer(9,1,10,1,0);
 repeat(DELAY+10)@(negedge sys_clk);
 if(ref_pins_CONFIG||ref_pins_ENA)$fatal(1,"terminal flash-busy hold failed");
 flash_busy=0;wait_config();
 if(ref_pins_CBSEL!==2)$fatal(1,"host reboot target changed");
 cam_ref_rst=1;repeat(30)@(negedge sys_clk);
 if(ref_pins_CONFIG||ref_pins_ENA)$fatal(1,"RU domain reset failed");
 cam_ref_rst=0;wait_config();reset_sys();

 // Reset with the DSP's CE disabled at its saturated terminal value.
 block_on_ack=1;transfer(9,1,5,0,0);
 repeat(DELAY+10)@(negedge sys_clk);reset_sys();
 block_on_ack=1;flash_busy=0;transfer(9,1,5,0,0);
 repeat(DELAY/2)@(negedge sys_clk);reset_sys();flash_busy=0;

 // A fresh automatic request must traverse the complete delay from zero.
 normal_boot=1;auto_slot=1;auto_request=1;
 repeat(2)@(negedge sys_clk);auto_request=0;
 wait_config();repeat(2)@(negedge sys_clk);
 if(normal_age<DELAY||normal_age>DELAY+100||ref_pins_CBSEL!==1)
  $fatal(1,"automatic reboot latency or target changed");
 transfer(9,1,6,1,0);
 transfer(4,0,0,0,0);if(last_read!==32'h01234567)$fatal(1,"identity after resets");
 if(checks<200||acks<10||errors<3||configs<3)$fatal(1,"insufficient coverage");
 $display("PASS boot DSP delay=%0d checks=%0d normal_age=%0d",DELAY,checks,normal_age);
 $finish;
end
"""]
    lines += ["always @(posedge sys_clk or posedge cam_ref_clk)begin #1;if(checking)begin checks=checks+1;"]
    for name in outputs:
        lines.append(f'if(ref_{name}!==dut_{name})$fatal(1,"{name} differs delay=%0d",DELAY);')
    lines += ["end end endmodule"]
    (tmp_path / "tb.sv").write_text("\n".join(lines) + "\n")
    command = ["verilator", "--binary", "--timing", "-Wno-fatal", "--top-module", "tb",
               "--Mdir", str(tmp_path / "obj"), str(tmp_path / "tb.sv"),
               str(tmp_path / "reference.v"), str(tmp_path / "candidate.v"), str(model), "-j", "2"]
    result = subprocess.run(command, cwd=tmp_path, capture_output=True, text=True)
    (tmp_path / "compile.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    result = subprocess.run([str(tmp_path / "obj/Vtb")], cwd=tmp_path, capture_output=True, text=True)
    (tmp_path / "run.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"PASS boot DSP delay={delay}" in result.stdout
