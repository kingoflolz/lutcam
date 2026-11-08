"""Real-vendor proof of I2C DSP capture and watchdog reset/enable semantics.

The actual helper and watchdog branch are loaded from the tested I2C source;
the SV scoreboard independently expresses the old synchronous register rules.
LUTCAM_I2C_SOURCE can select a frozen artifact before production promotion.
"""
import ast
import hashlib
import importlib.util
import inspect
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

import pytest
from migen import Cat, Constant, If, Instance, Module, Signal
from migen.fhdl import verilog
from litex.build.efinix.toolchain import find_efinity_path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "fpga/litex/lutcam/i2c.py").is_file())
SOURCE = Path(os.environ.get("LUTCAM_I2C_SOURCE", ROOT / "fpga/litex/lutcam/i2c.py"))


def load_source():
    spec = importlib.util.spec_from_file_location("i2c_primitive_subject", SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def watchdog_slice(subject, timeout, path, name):
    """Extract actual constructor branch, preserving all DSP parameters/guards."""
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "I2CTransactionEngine")
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
    branch = next(n for n in init.body if isinstance(n, ast.If)
                  and isinstance(n.test, ast.Name) and n.test.id == "use_dsp_watchdog")
    dut = Module()
    dut.start, dut.lease_reset, dut.lease_wait, dut.scl_i = [Signal(name=n) for n in
        ("start", "lease_reset", "lease_wait", "scl_i")]
    wait_high, stretch = Signal(name="wait_high"), Signal(max=timeout, name="count")
    namespace = dict(vars(subject), self=dut, wait_high=wait_high, stretch=stretch,
                     stretch_timeout_cycles=timeout, use_dsp_watchdog=True)
    exec(compile(ast.Module(body=[branch], type_ignores=[]), str(SOURCE), "exec"), namespace)
    generated = str(verilog.convert(dut, ios={dut.start, dut.lease_reset, dut.lease_wait,
        dut.scl_i, wait_high, stretch}, name=name))
    path.write_text(generated)
    match = re.search(r"EFX_DSP48 #\(.*?\)\s+(\w+)\s*\(", generated, re.S)
    assert match and generated.count("EFX_DSP48 #(") == 1
    return match[1]


def capture_slice(subject, path):
    dut = Module()
    data, enable = Signal(37, name="data"), Signal(name="enable")
    widths = (1, 18, 19, 20, 22, 32, 37)
    outputs = [Signal(width, name=f"value_{width}") for width in widths]
    for target in outputs:
        subject._capture_dsp(dut, target, data, enable)
    path.write_text(str(verilog.convert(dut, ios={data, enable, *outputs}, name="captures")))


def composite_slice(subject, path):
    dut = Module()
    data = Signal(32, name="data")
    descriptor_ce, start_ce, commit_ce, done_ce = [Signal(name=n) for n in
        ("descriptor_ce", "start_ce", "commit_ce", "done_ce")]
    descriptor, candidate, accepted, done = [Signal(32, name=n) for n in
        ("descriptor", "candidate", "accepted", "done")]
    subject._capture_dsp(dut, descriptor, data, descriptor_ce)
    subject._capture_dsp(dut, candidate, data, start_ce)
    subject._capture_dsp(dut, accepted, candidate, commit_ce)
    subject._capture_dsp(dut, done, accepted, done_ce)
    path.write_text(str(verilog.convert(dut, ios={data, descriptor_ce, start_ce, commit_ce,
        done_ce, descriptor, candidate, accepted, done}, name="capture_chain")))


def test_options_default_to_generic_and_native_bounds_are_explicit():
    subject = load_source()
    for name in ("compact_record", "use_dsp_state", "use_dsp_watchdog"):
        assert inspect.signature(subject.SharedCameraI2C).parameters[name].default is False
    assert inspect.signature(subject.I2CTransactionEngine).parameters["use_dsp_watchdog"].default is False
    # 48-bit maximum is supported; arbitrary larger generic timers stay legal.
    subject.I2CTransactionEngine(stretch_timeout_cycles=1 << 48, use_dsp_watchdog=True)
    subject.I2CTransactionEngine(stretch_timeout_cycles=(1 << 48) + 1)
    with pytest.raises(ValueError, match="at most 48 bits"):
        subject.I2CTransactionEngine(stretch_timeout_cycles=(1 << 48) + 1, use_dsp_watchdog=True)
    with pytest.raises(ValueError, match="1 through 37"):
        subject._capture_dsp(Module(), Signal(38), Signal(38), Signal())
    # Pin the resource contract for generic callers and the board's compact
    # three-capture plus one-watchdog profile. No native option is implicit.
    counts = {}
    for compact in (False, True):
        for state in (False, True):
            for watchdog in (False, True):
                fragment = subject.SharedCameraI2C(single_aperture=True,
                    compact_record=compact, use_dsp_state=state,
                    use_dsp_watchdog=watchdog).get_fragment()
                count = sum(isinstance(item, Instance) and item.of == "EFX_DSP48"
                            for item in fragment.specials)
                assert count == (3 if compact else 4) * int(state) + int(watchdog)
                counts[f"compact={compact},state={state},watchdog={watchdog}"] = count
    print(json.dumps({"elaborated_profile_dsp_counts": counts}, sort_keys=True))


@pytest.mark.skipif(shutil.which("verilator") is None, reason="Verilator required")
def test_real_vendor_storage_and_saturating_watchdog_match_register_contract(tmp_path):
    try:
        model = Path(find_efinity_path()) / "sim_models/verilog/efx_dsp48.v"
    except OSError:
        pytest.skip("Efinity DSP model required")
    assert model.is_file()
    subject = load_source()
    timeouts = (2, 37, 3_000_000, 1 << 48)
    sources = []
    dsp_instances = []
    for index, timeout in enumerate(timeouts):
        path = tmp_path / f"watchdog_{index}.v"
        dsp_instances.append(watchdog_slice(subject, timeout, path, f"watchdog_{index}"))
        sources.append(path)
    capture_slice(subject, tmp_path / "captures.v")
    composite_slice(subject, tmp_path / "capture_chain.v")
    sources += [tmp_path / "captures.v", tmp_path / "capture_chain.v"]
    lines = ["`timescale 1ns/1ps", "module tb;",
        "reg sys_clk=0,sys_rst=1,start=0,lease_reset=0,lease_wait=0,scl_i=1,wait_high=0;",
        "always #5 sys_clk=~sys_clk;",
        "reg [36:0] data=0; reg enable=0;",
        "reg descriptor_ce=0,start_ce=0,commit_ce=0,done_ce=0;",
        "wire [31:0] descriptor,candidate,accepted,done;",
        "reg [31:0] ref_descriptor=0,ref_candidate=0,ref_accepted=0,ref_done=0;",
        "reg [36:0] expected_data=0;",
        "reg [63:0] rng=64'h7ab302e69f54d811;",
        "integer cycles=0,seed_cases=0,guard_cases=0;",
        "wire waiting=(wait_high && !scl_i)||lease_wait;",
        "capture_chain chain(.sys_clk,.sys_rst,.data(data[31:0]),.descriptor_ce,.start_ce,.commit_ce,.done_ce,.descriptor,.candidate,.accepted,.done);",
    ]
    widths = (1, 18, 19, 20, 22, 32, 37)
    for width in widths:
        lines.append(f"wire [{width-1}:0] value_{width};")
    lines.append("captures captures_dut(.sys_clk,.sys_rst,.data,.enable,"+
        ",".join(f".value_{width}" for width in widths)+");")
    for i, timeout in enumerate(timeouts):
        width = (timeout-1).bit_length()
        lines += [f"wire [{width-1}:0] count_{i}; reg [{width-1}:0] expected_{i}=0;",
            f"watchdog_{i} wd_{i}(.sys_clk,.sys_rst,.start,.lease_reset,.lease_wait,.scl_i,.wait_high,.count(count_{i}));"]
    lines += ["always @(posedge sys_clk) begin",
        "if(sys_rst) expected_data<=0; else if(enable) expected_data<=data;",
        "if(sys_rst) begin ref_descriptor<=0;ref_candidate<=0;ref_accepted<=0;ref_done<=0;end",
        "else begin",
        "if(descriptor_ce)ref_descriptor<=data[31:0];",
        "if(start_ce)ref_candidate<=data[31:0];",
        "if(commit_ce)ref_accepted<=ref_candidate;",
        "if(done_ce)ref_done<=ref_accepted;end"]
    for i, timeout in enumerate(timeouts):
        width = (timeout-1).bit_length()
        lines += [f"if(sys_rst || start || lease_reset) expected_{i}<=0;",
            f"else if(waiting) begin if(expected_{i}!={width}'d{timeout-1}) expected_{i}<=expected_{i}+1'b1;end",
            f"else expected_{i}<=0;"]
    lines += ["end", "task tick;begin @(posedge sys_clk);#1;cycles=cycles+1;"]
    for width in widths:
        lines.append(f'if(value_{width}!==expected_data[{width-1}:0])$fatal(1,"capture width{width} cycle%0d got%h expected%h",cycles,value_{width},expected_data[{width-1}:0]);')
    lines += ['if({descriptor,candidate,accepted,done}!=={ref_descriptor,ref_candidate,ref_accepted,ref_done})$fatal(1,"old-value capture chain cycle%0d",cycles);']
    for i, timeout in enumerate(timeouts):
        lines.append(f'if(count_{i}!==expected_{i})$fatal(1,"watchdog T={timeout} cycle%0d got%h expected%h",cycles,count_{i},expected_{i});')
    lines += ["end endtask", "task next_input;begin @(negedge sys_clk);",
        "rng=rng^(rng<<13);rng=rng^(rng>>7);rng=rng^(rng<<17);data=rng[36:0];end endtask",
        "task seed(input integer which,input reg[47:0]value);begin next_input();case(which)"]
    for i, instance in enumerate(dsp_instances):
        width = (timeouts[i]-1).bit_length()
        lines.append(f"{i}:begin wd_{i}.{instance}.genblk1.i_DSP48.W_pipe.O_r=48'(value[{width-1}:0]);expected_{i}=value[{width-1}:0];end")
    lines += ["endcase seed_cases=seed_cases+1;end endtask", "integer k,j,mask; reg[47:0] old_count;",
        "initial begin tick(); next_input();sys_rst=0;enable=1;",
        "for(k=0;k<37;k=k+1)begin data=37'd1<<k;tick();next_input();data=~(37'd1<<k);tick();next_input();end",
        # All event combinations test pre-edge capture semantics and reset priority.
        "for(k=0;k<64;k=k+1)begin next_input();{sys_rst,enable,done_ce,commit_ce,start_ce,descriptor_ce}=6'(k);tick();end",
        "for(k=0;k<20000;k=k+1)begin next_input();",
        "sys_rst=(k%331==0);enable=rng[0];descriptor_ce=rng[1];start_ce=rng[2];commit_ce=rng[3];done_ce=rng[4];",
        "start=rng[5];lease_reset=rng[6];lease_wait=rng[7];wait_high=rng[8];scl_i=rng[9];tick();end",
    ]
    # Seed legal boundary and arbitrary low-width values so large carries and
    # saturation are exercised without waiting2**48 cycles. Full production
    # 3,000,000-cycle walk below still checks the reachable no-seed path.
    for i, timeout in enumerate(timeouts):
        width = (timeout-1).bit_length()
        values = sorted({0,1,max(0,timeout-2),timeout-1,(1<<width)-1,
                         *[1<<bit for bit in range(width)]})
        for value in values:
            lines += [f"seed({i},48'd{value});",
                "sys_rst=0;start=0;lease_reset=0;lease_wait=1;wait_high=0;scl_i=1;",
                "tick();tick();tick();"]
    lines += ["for(mask=0;mask<64;mask=mask+1)begin",
        "seed(2,48'd2999998);{sys_rst,start,lease_reset,lease_wait,wait_high,scl_i}=6'(mask);tick();guard_cases=guard_cases+1;end",
        # Synchronous reset must not change state before its sampling edge.
        "seed(2,48'd1234567);sys_rst=0;start=0;lease_reset=0;lease_wait=1;tick();",
        "next_input();old_count=48'(count_2);sys_rst=1;#1;if(count_2!==old_count[21:0])$fatal(1,\"asynchronous reset\");tick();",
        # Production timeout bound reached by ordinary increments, no seed.
        "next_input();sys_rst=0;start=0;lease_reset=0;lease_wait=1;wait_high=0;scl_i=1;enable=0;",
        "descriptor_ce=0;start_ce=0;commit_ce=0;done_ce=0;",
        "repeat(3000010)tick();",
        "if(count_2!==22'd2999999)$fatal(1,\"production saturation\");",
        "next_input();lease_wait=0;tick();if(count_2!==0)$fatal(1,\"idle clear\");",
        '$display("PASS native I2C primitives cycles=%0d seed_cases=%0d guard_cases=%0d",cycles,seed_cases,guard_cases);',
        "$finish;end endmodule"]
    tb = tmp_path / "tb.sv"
    tb.write_text("\n".join(lines)+"\n")
    command = ["verilator", "--binary", "--timing", "--assert", "-Wno-fatal", "-j", "2",
               "--top-module", "tb", "--Mdir", str(tmp_path / "obj"), str(tb),
               *(str(p) for p in sources), str(model)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout+result.stderr
    result = subprocess.run([str(tmp_path / "obj/Vtb")], capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout+result.stderr
    assert "PASS native I2C primitives" in result.stdout
    print(result.stdout)
    print(json.dumps({"source_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        "vendor_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
        "timeouts": timeouts, "capture_widths": widths}, sort_keys=True))
