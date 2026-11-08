"""Exact-cycle qualification of the optional production held-frame contract.

The main CameraPipeline holds its frame ID from admission until the complete
image and shared solver pose/centroid transaction retire. Faults retain the ID;
paired clear/reset resets transport ownership on the ID-changing edge. This is
stronger than ready/valid payload stability: an accepted pose writes its saved
frame during prefix replay even when the input is invalid. Generic/default mode
must still detect arbitrary frame-ID changes. This test compares complete
transports, including queued old-frame descriptors, under both contracts.

Verilator executes generated RTL, not a vendor primitive model. Both Migen and
LiteX emitters are checked. No FPGA, network, synthesis or build-tree fixtures
are required. The tests compare the two production modes.
"""
from pathlib import Path
import inspect
import re
import shutil
import subprocess
import tempfile
import unittest

from migen import ClockDomain, Module
from migen.sim import run_simulation
from migen.fhdl import verilog as migen_verilog
from migen.fhdl.simplify import FullMemoryWE
from litex.gen.fhdl import verilog as litex_verilog
from fpga.litex.lutcam.packet import SharedPacketTransport


OPTIONS = dict(child_slots=8, frame_words=64000,
    external_destination=True, external_context_held=True, compact_padding=True,
    batched_packets=True, accepted_centroids=True, shared_solver_ingress=True)


def emit(transport_class, held, directory, name, emitter="migen"):
    options = dict(OPTIONS)
    if held is not None:
        options["held_frame_id"] = held
    dut = transport_class(**options)
    ios = set()
    def port(label, signal):
        signal.name_override = label
        ios.add(signal)
    if emitter == "litex":
        dut.clock_domains.cd_sys = ClockDomain("sys")
        port("sys_clk", dut.cd_sys.clk)
        port("sys_rst", dut.cd_sys.rst)
    def endpoint(label, record):
        for field in ("valid", "ready", "first", "last", "data", "sof", "padding",
                      "frame_id", "end", "last_be", "error", "src_port", "dst_port",
                      "ip_address", "length"):
            if hasattr(record, field):
                port(label + "_" + field, getattr(record, field))
    for camera, reset in enumerate(dut.camera_reset):
        port(f"camera_reset{camera}", reset)
    endpoint("source", dut.source)
    for field in ("drained", "sequence_request", "sequence_stream", "sequence_valid",
                  "sequence_value", "sequence_destination_ip", "sequence_destination_port"):
        port(field, getattr(dut, field))
    for index, channel in enumerate(dut.channels):
        endpoint(f"sink{index}", channel.sink)
        for field in ("enable", "capture_enable", "clear", "faults", "fifo_level",
                      "counter_committed_level", "high_water", "commit_event", "sent_event",
                      "bytes_event", "overflow_event", "protocol_event", "reject_event",
                      "configuration_idle", "release", "_write_enable", "_write_index",
                      "_write_data", "_commit", "_frame", "_offset", "_length"):
            port(f"ch{index}_{field.lstrip('_')}", getattr(channel, field))
    FullMemoryWE()(dut)
    converter = migen_verilog if emitter == "migen" else litex_verilog
    result = converter.convert(dut.get_fragment(), ios=ios, name=name)
    path = directory / (name + ".v")
    result.write(str(path))
    for filename, contents in result.data_files.items():
        (directory / filename).write_text(contents)
    return path


def make_bench(reference, malformed_ids=False):
    ports = re.findall(r"^\s*(input|output)\s+(?:(?:reg|wire)\s+)?(\[[^]]+\])?\s*(\w+)\s*[,\n]",
                       reference.read_text().split(");", 1)[0], re.M)
    assert ports and any(name == "source_valid" for _, _, name in ports)
    lines = ["module tb; logic sys_clk=0;"]
    for direction, width, name in ports:
        if name != "sys_clk":
            lines.append(f"logic {width} {name}" + ("=0;" if direction == "input" else f", candidate_{name};"))
    for module, prefix in (("reference", ""), ("candidate", "candidate_")):
        connections = ",".join(f".{name}({prefix + name if direction == 'output' else name})"
                               for direction, _, name in ports)
        lines.append(f"{module} dut_{module}({connections});")
    lines += ["integer cycles=0, comparisons=0, packets=0, frame_headers=0, resets=0;",
              "integer pose_stalls=0, child_stalls=0, output_stalls=0, protocol_events=0;",
              "integer full_child[2], full_pose[2], image_completed[2], image_index[2], image_prefix[2];",
              "integer pose_jobs[2], child_jobs[2], aborts=0, gap_cycles=0, wrapped=0, queued_id_changes=0, queued_old_headers=0;",
              "logic [31:0] frame[2], seqs[6], rng=32'h83a12f97;",
              "logic image_active[2], take_pose[2], take_child[2], take_image[2], pose_protocol[2], child_protocol[2];",
              "integer output_mode=1, header_position=0, output_stream=0;",
              "integer expected_read[6], expected_write[6]; logic [31:0] expected_frame[6][0:8191];",
              "function automatic [31:0] be(input [31:0] x); be={x[7:0],x[15:8],x[23:16],x[31:24]}; endfunction",
              "task automatic compare_outputs; begin"]
    for direction, _, name in ports:
        if direction != "output":
            continue
        condition = "1"
        if name.startswith("source_") and name != "source_valid":
            condition = "source_valid"
        elif name.endswith("bytes_event"):
            condition = name.replace("bytes_event", "sent_event")
        elif name.endswith(("write_index", "write_data")):
            condition = name.rsplit("_", 2)[0] + "_write_enable"
        elif name.endswith(("_frame", "_offset", "_length")) and name.startswith("ch"):
            condition = name.rsplit("_", 1)[0] + "_commit"
        lines.append(f'if (({condition}) && ({name} !== candidate_{name})) '
                     f'$fatal(1,"mismatch {name} cycle%0d base%h candidate%h",cycles,{name},candidate_{name});')
    lines += ["comparisons=comparisons+1; end endtask", "task automatic step; begin",
              "rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};",
              "source_ready=(output_mode==1) || ((output_mode==2) && rng[2:0]!=0);",
              "sequence_destination_ip=32'hc0a80102; sequence_destination_port=7100;",
              "for(integer c=0;c<2;c=c+1) begin"]
    for camera in range(2):
        image, child, pose = 3 * camera, 3 * camera + 1, 3 * camera + 2
        lines += [f"if(c=={camera}) begin",
                  f"sink{image}_frame_id=frame[c]; sink{child}_frame_id=frame[c]; sink{pose}_frame_id=frame[c];",
                  f"sink{image}_valid=image_active[c] && rng[c];",
                  f"sink{image}_sof=(image_index[c]==0); sink{image}_padding=(image_index[c]>=image_prefix[c]);",
                  f"sink{image}_data=sink{image}_padding ? 0 : (16'h12a5 ^ image_index[c]); end"]
    lines += ["end"]
    if malformed_ids:
        lines += ["if (inject_pose_collision) begin if(active_camera==0)sink2_frame_id=frame[0]+1;else sink5_frame_id=frame[1]+1;end",
                  "if (inject_child_collision) begin if(active_camera==0)sink1_frame_id=frame[0]+1;else sink4_frame_id=frame[1]+1;end"]
    lines += ["#1; sequence_valid=sequence_request && (rng[4:3]!=0); sequence_value=seqs[sequence_stream];",
              "#1; compare_outputs();",
              "take_pose[0]=sink2_valid && sink2_ready;take_pose[1]=sink5_valid && sink5_ready;",
              "take_child[0]=sink1_valid && sink1_ready;take_child[1]=sink4_valid && sink4_ready;",
              "pose_protocol[0]=ch2_protocol_event;pose_protocol[1]=ch5_protocol_event;child_protocol[0]=ch1_protocol_event;child_protocol[1]=ch4_protocol_event;",
              "take_image[0]=sink0_valid;take_image[1]=sink3_valid;",
              "if(source_valid&&!source_ready)output_stalls=output_stalls+1;"]
    for channel in range(6):
        lines += [f"if(ch{channel}_protocol_event)protocol_events=protocol_events+1;",
                  f"if(ch{channel}_commit)begin expected_frame[{channel}][expected_write[{channel}]]=frame[{channel//3}]; expected_write[{channel}]=expected_write[{channel}]+1;end",
                  f"if(ch{channel}_sent_event)seqs[{channel}]=seqs[{channel}]+1;"]
    for camera in range(2):
        child, pose = camera * 3 + 1, camera * 3 + 2
        lines += [f"if(ch{child}_fifo_level==8)full_child[{camera}]=full_child[{camera}]+1;",
                  f"if(ch{pose}_fifo_level==3)full_pose[{camera}]=full_pose[{camera}]+1;",
                  f"if(sink{child}_valid&&!sink{child}_ready)child_stalls=child_stalls+1;",
                  f"if(sink{pose}_valid&&!sink{pose}_ready)pose_stalls=pose_stalls+1;"]
    lines += ["if(source_valid&&source_ready)begin",
              "if(source_first)begin header_position=0;output_stream=source_src_port-6000;if(source_data!==be(32'h4c555443))$fatal(1,\"bad magic\");end",
              "if(header_position==2)begin if(source_data!==be(expected_frame[output_stream][expected_read[output_stream]]))$fatal(1,\"queued frame mismatch cycle%0d stream%0d\",cycles,output_stream);if(output_stream%3!=0 && expected_frame[output_stream][expected_read[output_stream]]!=frame[output_stream/3])queued_old_headers=queued_old_headers+1;frame_headers=frame_headers+1;end",
              "header_position=header_position+1;if(source_last)packets=packets+1;end"]
    for channel in range(6):
        lines += [f"if(ch{channel}_release)expected_read[{channel}]=expected_read[{channel}]+1;"]
    lines += ["sys_clk=1;#1;compare_outputs();sys_clk=0;#1;cycles=cycles+1;",
              "for(integer c=0;c<2;c=c+1)begin if(take_image[c])begin image_index[c]=image_index[c]+1; if(image_index[c]==64000)begin image_active[c]=0;image_completed[c]=image_completed[c]+1;end end end",
              "end endtask",
              "task automatic idle(input integer clocks);begin repeat(clocks)begin step();gap_cycles=gap_cycles+1;end end endtask",
              "task automatic reset_pair;begin",
              "sink1_valid=0;sink2_valid=0;sink4_valid=0;sink5_valid=0;image_active[0]=0;image_active[1]=0;",
              "camera_reset0=1;camera_reset1=1;step();camera_reset0=0;camera_reset1=0;",
              "for(integer c=0;c<2;c=c+1)begin frame[c]=0;image_index[c]=0;end",
              "for(integer c=0;c<6;c=c+1)begin expected_read[c]=0;expected_write[c]=0;end",
              "resets=resets+1;idle(2);end endtask",
              "logic inject_pose_collision=0, inject_child_collision=0;integer active_camera=0;",
              "task automatic pose_word(input integer cam,input integer idx,input integer count,input integer accepted,input integer bad_status,input integer collide);",
              "logic [31:0] data;integer waits;begin active_camera=cam;inject_pose_collision=collide;",
              "data=32'h50700000+idx;if(idx==1)data=accepted?32'h813579bd:32'h013579bd;if(idx==8)data=bad_status?32'h40000000:32'h40000001;",
              "if(cam==0)begin sink2_valid=1;sink2_data=data;sink2_last=(idx==count-1);end else begin sink5_valid=1;sink5_data=data;sink5_last=(idx==count-1);end",
              "waits=0;do begin step();waits=waits+1;if(waits>10000)$fatal(1,\"pose watchdog\");end while(!take_pose[cam]);",
              "if(collide&&!pose_protocol[cam])$fatal(1,\"missing generic pose frame fault\");sink2_valid=0;sink5_valid=0;inject_pose_collision=0;end endtask",
              "task automatic pose(input integer cam,input integer count,input integer accepted,input integer bad_status,input integer collision);begin",
              "for(integer i=0;i<count;i=i+1)begin pose_word(cam,i,count,accepted,bad_status,collision&&(i==5));if((i%3)==0)idle(2);end pose_jobs[cam]=pose_jobs[cam]+1;end endtask",
              "task automatic child_word(input integer cam,input integer idx,input integer count,input integer collision);integer waits;begin",
              "active_camera=cam;inject_child_collision=collision;",
              "if(cam==0)begin sink1_valid=1;sink1_data=33'h123456000+idx;sink1_end=(idx==count-1);end else begin sink4_valid=1;sink4_data=33'h145678000+idx;sink4_end=(idx==count-1);end",
              "waits=0;do begin step();waits=waits+1;if(waits>10000)$fatal(1,\"child watchdog\");end while(!take_child[cam]);",
              "if(collision&&!child_protocol[cam])$fatal(1,\"missing generic child frame fault\");sink1_valid=0;sink4_valid=0;inject_child_collision=0;end endtask",
              "task automatic children(input integer cam,input integer count,input integer collision);begin for(integer i=0;i<count;i=i+1)begin child_word(cam,i,count,collision&&(i==4));if(i==3)idle(3);end child_jobs[cam]=child_jobs[cam]+1;end endtask",
              "task automatic enable_all;begin"]
    for channel in range(6):
        lines += [f"ch{channel}_enable=1;ch{channel}_capture_enable=1;ch{channel}_clear=0;"]
    lines += ["end endtask", "initial begin"]
    for direction, _, name in ports:
        if direction == "input" and name != "sys_clk":
            lines.append(name + "=0;")
    lines += ["for(integer c=0;c<6;c=c+1)begin seqs[c]=32'hfffffffe;expected_read[c]=0;expected_write[c]=0;end",
              "for(integer c=0;c<2;c=c+1)begin frame[c]=0;image_active[c]=0;image_index[c]=0;image_prefix[c]=511;full_child[c]=0;full_pose[c]=0;image_completed[c]=0;pose_jobs[c]=0;child_jobs[c]=0;end",
              "sys_rst=1;step();sys_rst=0;enable_all();idle(3);",
              "// Abort every pose word, every child word, and each prefix replay phase.",
              "for(integer cam=0;cam<2;cam=cam+1)begin",
              "for(integer stop=0;stop<22;stop=stop+1)begin reset_pair();frame[cam]=32'h87654320+stop;for(integer i=0;i<=stop;i=i+1)pose_word(cam,i,22,1,0,0);idle(7);aborts=aborts+1;end",
              "for(integer phase=0;phase<3;phase=phase+1)begin reset_pair();frame[cam]=32'h93574286;pose_word(cam,0,22,1,0,0);pose_word(cam,1,22,1,0,0);idle(phase);reset_pair();aborts=aborts+1;end",
              "for(integer stop=0;stop<9;stop=stop+1)begin reset_pair();frame[cam]=32'h1234abcd;for(integer i=0;i<=stop;i=i+1)child_word(cam,i,9,0);idle(7);aborts=aborts+1;end end",
              "// Full child queues, then stalled ninth group; alternate owners only between groups.",
              "reset_pair();output_mode=0;ch2_enable=0;ch5_enable=0;",
              "for(integer cam=0;cam<2;cam=cam+1)begin frame[cam]=32'h50000000+cam;for(integer j=0;j<8;j=j+1)begin pose(cam,22,1,0,0);children(cam,9,0);end end",
              "sink1_valid=1;sink1_end=0;sink1_data=33'h100000001;idle(30);sink1_valid=0;output_mode=2;children(0,9,0);idle(2000);",
              "// Full pose queues retain the two-word prefix through long invalid gaps.",
              "reset_pair();enable_all();output_mode=0;ch1_enable=0;ch4_enable=0;",
              "for(integer cam=0;cam<2;cam=cam+1)begin frame[cam]=32'h60000000+cam;pose(cam,22,1,0,0);children(cam,9,0);pose(cam,22,1,0,0);children(cam,9,0);pose_word(cam,0,22,1,0,0);pose_word(cam,1,22,1,0,0);idle(40);",
              "if(cam==0)sink2_valid=1;else sink5_valid=1;idle(20);sink2_valid=0;sink5_valid=0;output_mode=2;for(integer i=2;i<22;i=i+1)pose_word(cam,i,22,1,0,0);children(cam,9,0);idle(1000);output_mode=0;end",
              "// Force committed auxiliary records across a legal producer-ID advance.",
              "reset_pair();enable_all();output_mode=0;",
              "for(integer cam=0;cam<2;cam=cam+1)begin frame[cam]=32'habcdef01+cam;pose(cam,22,1,0,0);children(cam,9,0);",
              "if(cam==0)begin if(ch1_fifo_level!=1||ch2_fifo_level!=1)$fatal(1,\"missing queued camera0\");end else begin if(ch4_fifo_level!=1||ch5_fifo_level!=1)$fatal(1,\"missing queued camera1\");end",
              "frame[cam]=frame[cam]+1;queued_id_changes=queued_id_changes+1;pose(cam,22,1,0,0);children(cam,9,0);end",
              "output_mode=2;idle(2000);if(ch1_fifo_level!=0||ch2_fifo_level!=0||ch4_fifo_level!=0||ch5_fifo_level!=0||queued_old_headers!=4)$fatal(1,\"queued old-frame coverage%0d\",queued_old_headers);",
              "reset_pair();enable_all();output_mode=2;"]
    if malformed_ids:
        lines += ["for(integer cam=0;cam<2;cam=cam+1)begin frame[cam]=32'h70000000+cam;pose(cam,22,1,0,1);reset_pair();children(cam,9,1);reset_pair();end"]
    else:
        lines += ["// Three full image epochs include 0xffffffff -> 0 while old packets may remain queued.",
                  "frame[0]=32'hfffffffe;frame[1]=32'hffffffff;",
                  "for(integer epoch=0;epoch<3;epoch=epoch+1)begin",
                  "for(integer c=0;c<2;c=c+1)begin image_active[c]=1;image_index[c]=0;image_prefix[c]=(epoch==0)?511:((epoch==1)?512:64000);end",
                  "for(integer job=0;job<80;job=job+1)begin pose(job%2,(job%5==0)?8:22,(job%5!=0),0,0);if(job%5!=0)children(job%2,9,0);end",
                  "while(image_active[0]||image_active[1])step();",
                  "for(integer c=0;c<2;c=c+1)begin if(frame[c]==32'hffffffff)wrapped=wrapped+1;frame[c]=frame[c]+1;end end",
                  "idle(6000);"]
    lines += ["// Non-ID malformed packets retain faults and early/late-last behavior in both modes.",
              "for(integer cam=0;cam<2;cam=cam+1)begin reset_pair();pose(cam,6,1,0,0);reset_pair();pose(cam,24,1,0,0);reset_pair();pose(cam,22,1,1,0);reset_pair();children(cam,4,0);reset_pair();children(cam,12,0);end",
              "reset_pair();enable_all();pose(1,22,1,0,0);children(1,9,0);idle(1000);",
              "if(full_child[0]==0||full_child[1]==0||full_pose[0]==0||full_pose[1]==0||child_stalls==0||pose_stalls==0||output_stalls==0||protocol_events==0||aborts!=68||frame_headers<10)$fatal(1,\"coverage incomplete children%0d/%0d poses%0d/%0d stalls%0d/%0d protocol%0d aborts%0d headers%0d\",full_child[0],full_child[1],full_pose[0],full_pose[1],child_stalls,pose_stalls,protocol_events,aborts,frame_headers);"]
    if not malformed_ids:
        lines += ["if(image_completed[0]!=3||image_completed[1]!=3||wrapped!=2)$fatal(1,\"missing image/wrap coverage\");"]
    lines += ['$display("PASS cycles=%0d comparisons=%0d packets=%0d frame_headers=%0d resets=%0d aborts=%0d pose_stalls=%0d child_stalls=%0d output_stalls=%0d protocol_events=%0d images=%0d/%0d wraps=%0d queued_id_changes=%0d queued_old_headers=%0d",cycles,comparisons,packets,frame_headers,resets,aborts,pose_stalls,child_stalls,output_stalls,protocol_events,image_completed[0],image_completed[1],wrapped,queued_id_changes,queued_old_headers);$finish;end endmodule']
    return "\n".join(lines) + "\n"


def run_comparison(reference_class, candidate_class, directory, *, held=True,
                   malformed_ids=False, emitter="migen", reference_flag=None):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    reference = emit(reference_class, reference_flag, directory, "reference", emitter)
    candidate = emit(candidate_class, held, directory, "candidate", emitter)
    bench = directory / "tb.sv"
    bench.write_text(make_bench(reference, malformed_ids))
    command = ["verilator", "--binary", "--timing", "--assert", "-Wno-fatal", "-j", "4",
               "--top-module", "tb", "--Mdir", str(directory / "obj"),
               str(reference), str(candidate), str(bench)]
    with (directory / "compile.log").open("w") as log:
        result = subprocess.run(command, cwd=directory, stdout=log, stderr=subprocess.STDOUT, timeout=120)
    if result.returncode:
        raise AssertionError((directory / "compile.log").read_text()[-7000:])
    result = subprocess.run([str(directory / "obj/Vtb")], cwd=directory,
                            capture_output=True, text=True, timeout=60)
    output = result.stdout + result.stderr
    (directory / "parity.log").write_text(output)
    if result.returncode or "PASS cycles=" not in output:
        raise AssertionError(output)
    return output


def run_portable(reference_class, candidate_class):
    """Exercise invalid-input phase3 metadata writes and one-edge dirty resets."""
    top = Module()
    top.submodules.reference = reference = reference_class(**OPTIONS)
    top.submodules.candidate = candidate = candidate_class(**OPTIONS, held_frame_id=True)
    inputs = [(reference.source.ready, candidate.source.ready)]
    for field in ("sequence_valid", "sequence_value", "sequence_destination_ip", "sequence_destination_port"):
        inputs.append((getattr(reference, field), getattr(candidate, field)))
    inputs += list(zip(reference.camera_reset, candidate.camera_reset))
    for old, new in zip(reference.channels, candidate.channels):
        for field in ("enable", "capture_enable", "clear"):
            inputs.append((getattr(old, field), getattr(new, field)))
        for field in ("valid", "first", "last", "data", "sof", "padding", "frame_id", "end"):
            if hasattr(old.sink, field):
                inputs.append((getattr(old.sink, field), getattr(new.sink, field)))
    top.comb += [new.eq(old) for old, new in inputs]
    coverage = {"cycles": 0, "invalid_frame_writes": 0, "reset_edges": 0}
    def process():
        for cycle in range(128):
            phase, owner = cycle % 32, (cycle // 32) % 2
            resetting = phase == 20
            for reset in reference.camera_reset:
                yield reset.eq(resetting)
            for index, channel in enumerate(reference.channels):
                yield channel.enable.eq(1)
                yield channel.capture_enable.eq(1)
                yield channel.clear.eq(resetting)
                yield channel.sink.valid.eq(index == owner * 3 + 2 and phase in (2, 3))
                yield channel.sink.frame_id.eq(0 if phase >= 21 else 0x81234567 + index // 3)
                yield channel.sink.data.eq(0x81234567 if phase == 3 else 0x11223344)
            yield reference.sequence_valid.eq(1)
            yield reference.source.ready.eq(0)
            yield
            for field in ("drained", "sequence_request", "sequence_stream"):
                assert (yield getattr(reference, field)) == (yield getattr(candidate, field)), field
            for old, new in zip(reference.channels, candidate.channels):
                for field in ("faults", "fifo_level", "counter_committed_level", "high_water",
                              "commit_event", "sent_event", "overflow_event", "protocol_event",
                              "reject_event", "configuration_idle", "_write_enable", "_commit"):
                    assert (yield getattr(old, field)) == (yield getattr(new, field)), field
                assert (yield old.sink.ready) == (yield new.sink.ready)
                if (yield old._write_enable):
                    assert (yield old._write_index) == (yield new._write_index)
                    assert (yield old._write_data) == (yield new._write_data)
                    if old.stream_type == 3 and (yield old._write_index) == 22:
                        assert not (yield old.sink.valid)
                        coverage["invalid_frame_writes"] += 1
            assert (yield reference.source.valid) == (yield candidate.source.valid)
            if (yield reference.source.valid):
                for field in ("first", "last", "data", "last_be", "error", "src_port",
                              "dst_port", "ip_address", "length"):
                    assert (yield getattr(reference.source, field)) == (yield getattr(candidate.source, field)), field
            coverage["cycles"] += 1
            coverage["reset_edges"] += resetting
    run_simulation(top, process())
    assert coverage["invalid_frame_writes"] == 4, coverage
    assert coverage["reset_edges"] == 4, coverage
    return coverage


class HeldFrameTests(unittest.TestCase):
    def test_requires_complete_topology_and_defaults_off(self):
        self.assertIs(inspect.signature(SharedPacketTransport).parameters["held_frame_id"].default, False)
        for options in ({}, {"shared_solver_ingress": True},
                        {"accepted_centroids": True, "shared_solver_ingress": True},
                        {"accepted_centroids": True, "batched_packets": True, "compact_padding": True}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                SharedPacketTransport(held_frame_id=True, **options)

    def test_portable_dirty_prefix_reset(self):
        self.assertEqual(run_portable(SharedPacketTransport, SharedPacketTransport)["cycles"], 128)

    @unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
    def test_held_frame_exact_cycle(self):
        for emitter in ("migen", "litex"):
            with self.subTest(emitter=emitter), tempfile.TemporaryDirectory(prefix="lutcam-held-frame-") as work:
                output = run_comparison(SharedPacketTransport, SharedPacketTransport,
                    work, emitter=emitter, reference_flag=False)
                self.assertIn("images=3/3 wraps=2", output)

    @unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
    def test_default_retains_malformed_frame_ids(self):
        with tempfile.TemporaryDirectory(prefix="lutcam-held-frame-default-") as work:
            output = run_comparison(SharedPacketTransport, SharedPacketTransport,
                work, held=False, malformed_ids=True)
            self.assertIn("PASS cycles=", output)


if __name__ == "__main__":
    unittest.main()
