"""Native transport equivalence for the single-owner shared LFTag ingress.

Four million clocks exercise simultaneous images, accepted/rejected/malformed
poses, full eight-slot centroid queues, output stalls, camera aborts, and resets.
The reference keeps independent camera parsers; DSP storage is optional.
"""
from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest

from migen.fhdl import verilog
from migen.fhdl.simplify import FullMemoryWE
from fpga.litex.lutcam.packet import SharedPacketTransport


def _rtl(shared, dsp=False):
    dut = SharedPacketTransport(child_slots=8, frame_words=64000,
        external_destination=True, external_context_held=True,
        compact_padding=True, batched_packets=True, accepted_centroids=True,
        shared_solver_ingress=shared,
        **({"use_dsp_metadata": True} if dsp else {}))
    ios = set()
    def port(name, signal):
        signal.name_override = name
        ios.add(signal)
    def record(name, endpoint):
        for signal in endpoint.flatten():
            port(name + "_" + signal.backtrace[-1][0], signal)
    for index, signal in enumerate(dut.camera_reset):
        port("camera_reset" + str(index), signal)
    record("source", dut.source)
    for name in ("drained", "sequence_request", "sequence_stream", "sequence_valid",
                 "sequence_value", "sequence_destination_ip", "sequence_destination_port"):
        port(name, getattr(dut, name))
    for index, channel in enumerate(dut.channels):
        record("sink" + str(index), channel.sink)
        for name in ("enable", "capture_enable", "clear", "faults", "fifo_level",
                     "counter_committed_level", "high_water", "commit_event", "sent_event",
                     "bytes_event", "overflow_event", "protocol_event", "reject_event",
                     "configuration_idle"):
            port("ch" + str(index) + "_" + name, getattr(channel, name))
    FullMemoryWE()(dut)
    return str(verilog.convert(dut.get_fragment(), ios=ios, name="area_probe"))


def _run_parity(old, new, p, dsp=False, model=None):
    old = old.replace("module area_probe(", "module packet_baseline(", 1)
    new = new.replace("module area_probe(", "module packet_candidate(", 1)
    (p / "packet-baseline.v").write_text(old)
    (p / "packet-candidate.v").write_text(new)
    ports=[]
    for d,w,n in re.findall(r'^\s*(input|output)(?: reg)?\s*(\[[^]]+\])?\s*(\w+)[,\n]',old.split(');',1)[0],re.M):ports.append((d,w,n))
    lines=['module tb; logic sys_clk=0; always #5 sys_clk=~sys_clk;']
    for d,w,n in ports:
     if n=='sys_clk':continue
     lines.append(f'logic {w} {n}'+ (' = 0;' if d=='input' else ', cand_'+n+';'))
    for module,prefix in [('packet_baseline',''),('packet_candidate','cand_')]:
     lines.append(module+' dut_'+module+'('+','.join(f'.{n}({prefix+n if d=="output" else n})' for d,w,n in ports)+');')
    lines += ['integer cyc, camera=0, phase=0, index=0, jobs=0, packet_words=22; integer accepted=1, malformed=0; integer full_child0=0, full_child1=0, invalid_owner_resets=0; integer pose_stalls=0, child_stalls=0, emitted=0, resets=0, abort_pause=0, image_completed[2], image_position[2], padding_at[2]; logic [1:0] image_active=0; logic [31:0] image_frame[2]; logic [31:0] rng=32\'h81b905a3, frame=32\'h12340000; logic take_pose,take_child; logic [31:0] seqs[0:5];', 'initial begin']
    for d,w,n in ports:
     if d=='input' and n!='sys_clk':lines.append(n+"=0;")
    lines+=['for(integer j=0;j<6;j=j+1) seqs[j]=0;for(integer c=0;c<2;c=c+1) begin image_completed[c]=0;image_position[c]=0;image_frame[c]=0;padding_at[c]=64000;end;',"sequence_destination_ip=32'hc0a801fb; sequence_destination_port=6000;",'for(cyc=0;cyc<4000000;cyc=cyc+1) begin','@(negedge sys_clk);',"rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};",'sys_rst=(cyc<5); camera_reset0=0;camera_reset1=0;','if(cyc>250000 && cyc%1237==0) begin if(rng[19]) camera_reset1=1; else camera_reset0=1; resets=resets+1; end','source_source_ready=(cyc>2000 && (cyc%16000<12000) && (rng[4:2]!=0));','sequence_valid=sequence_request && (rng[6:5]!=0); sequence_value=seqs[sequence_stream];']
    for j in range(6):
     lines += [f'ch{j}_enable=1;ch{j}_capture_enable=1;ch{j}_clear=(phase==0);sink{j}_sink_valid=0;sink{j}_sink_first=0;sink{j}_sink_last=0;']
    lines += ['if(cyc>6 && !sys_rst && !(camera ? camera_reset1:camera_reset0) && abort_pause==0) begin','case(phase)','0: begin end','1: begin','if(camera==0) begin', 'sink2_sink_valid=(rng[1:0]!=0); sink2_sink_payload_data=frame+index; sink2_sink_payload_frame_id=frame; sink2_sink_last=(index==packet_words-1);',"if(index==1) sink2_sink_payload_data={accepted[0],31'h13579b}; if(index==8) sink2_sink_payload_data=malformed==2 ? 32'h40000000:32'h40000001; if(malformed==1&&index==5) sink2_sink_payload_frame_id=frame+1;",'end else begin','sink5_sink_valid=(rng[1:0]!=0); sink5_sink_payload_data=frame+index; sink5_sink_payload_frame_id=frame; sink5_sink_last=(index==packet_words-1);',"if(index==1) sink5_sink_payload_data={accepted[0],31'h13579b}; if(index==8) sink5_sink_payload_data=malformed==2 ? 32'h40000000:32'h40000001; if(malformed==1&&index==5) sink5_sink_payload_frame_id=frame+1;",'end end','2: begin','if(camera==0) begin sink1_sink_valid=(rng[1:0]!=0);sink1_sink_payload_data={1\'b1,frame+index};sink1_sink_payload_frame_id=frame;sink1_sink_payload_end=(index==8);end','else begin sink4_sink_valid=(rng[1:0]!=0);sink4_sink_payload_data={1\'b1,frame+index};sink4_sink_payload_frame_id=frame;sink4_sink_payload_end=(index==8);end','end endcase end',
    'if(cyc>250000 && cyc%20000==15000) abort_pause=64;',
    'if(abort_pause>0) begin sink1_sink_valid=0;sink2_sink_valid=0;sink4_sink_valid=0;sink5_sink_valid=0; if(abort_pause==1) begin camera_reset0=1;camera_reset1=1;end end',
    'sink0_sink_valid=image_active[0] && rng[0];sink0_sink_payload_sof=(image_position[0]==0);sink0_sink_payload_padding=(image_position[0]>=padding_at[0]);sink0_sink_payload_data=sink0_sink_payload_padding?0:rng[31:16];sink0_sink_payload_frame_id=image_frame[0];',
    'sink3_sink_valid=image_active[1] && rng[1];sink3_sink_payload_sof=(image_position[1]==0);sink3_sink_payload_padding=(image_position[1]>=padding_at[1]);sink3_sink_payload_data=sink3_sink_payload_padding?0:rng[15:0];sink3_sink_payload_frame_id=image_frame[1];',
    '#1;','if(!sys_rst) begin']
    for d,w,n in ports:
     if d!='output':continue
     if n.startswith('sink'):
      idx=n[4]
      cond=f'sink{idx}_sink_valid'
     elif n.startswith('source_') and n!='source_source_valid':cond='source_source_valid'
     elif n.endswith('bytes_event'):cond=n.replace('bytes_event','sent_event')
     else:cond='1'
     lines.append(f'if(({cond}) && {n} !== cand_{n}) $fatal(1,"mismatch {n} cycle %0d base %h candidate %h phase%0d index%0d",cyc,{n},cand_{n},phase,index);')
    if dsp:
     for name in ('imagepacketizer0_frame_id','imagepacketizer1_frame_id','childpacketizer2_frame_id','parser_prefix0','parser_prefix1','parser_frame_id'):
      lines.append(f'if(dut_packet_baseline.{name} !== dut_packet_candidate.{name}) $fatal(1,"metadata {name} mismatch cycle%0d",cyc);')
    lines += ['end','if(ch1_fifo_level==8)full_child0=full_child0+1;if(ch4_fifo_level==8)full_child1=full_child1+1;if((camera?camera_reset1:camera_reset0) && !(sink2_sink_valid|sink5_sink_valid) && dut_packet_candidate.parser_active)invalid_owner_resets=invalid_owner_resets+1;','take_pose=camera ? (sink5_sink_valid && sink5_sink_ready):(sink2_sink_valid&&sink2_sink_ready);take_child=camera ? (sink4_sink_valid&&sink4_sink_ready):(sink1_sink_valid&&sink1_sink_ready);','if(phase==1 && !take_pose) pose_stalls=pose_stalls+1;if(phase==2&&!take_child)child_stalls=child_stalls+1;','for(integer j=0;j<6;j=j+1) begin case(j)']
    for j in range(6):lines.append(f'{j}: if(ch{j}_sent_event) begin seqs[j]=seqs[j]+1;emitted=emitted+1;end')
    lines+=['endcase end','@(posedge sys_clk);#1;','if(sys_rst || (camera ? camera_reset1:camera_reset0)) begin phase=0;index=0;end','else if(phase==0 && abort_pause==0) begin camera=rng[7];frame=frame+1;accepted=(rng[9:8]!=0);malformed=rng[14:12]==0 ? (rng[16:15]%3)+1:0;packet_words=accepted?22:8;if(malformed==3)packet_words=6;phase=1;if(cyc<3000)begin camera=jobs%2;phase=2;accepted=1;malformed=0;end index=0;jobs=jobs+1;end','else if(phase==1&&take_pose)begin if(index==packet_words-1)begin index=0;phase=accepted?2:0;end else index=index+1;end','else if(phase==2&&take_child)begin if(index==8)begin index=0;phase=0;end else index=index+1;end','if(abort_pause>0)abort_pause=abort_pause-1;',
    'for(integer c=0;c<2;c=c+1) begin if(sys_rst || (c==0?camera_reset0:camera_reset1))begin image_active[c]=0;image_position[c]=0;end else if(!image_active[c])begin if(rng[11:2]==0)begin image_active[c]=1;image_frame[c]=rng;image_position[c]=0;padding_at[c]=rng[27:16]==0?64000:((rng[27:16]%125)*512+(rng[31]?0:127));end end else if(c==0?sink0_sink_valid:sink3_sink_valid) begin if(image_position[c]==63999)begin image_completed[c]=image_completed[c]+1;image_active[c]=0;end else image_position[c]=image_position[c]+1;end end',
    'end','if(full_child0==0||full_child1==0||invalid_owner_resets==0)$fatal(1,"coverage missing full0%0d full1%0d reset%0d",full_child0,full_child1,invalid_owner_resets);','$display("PASS 4000000 cycle shared-ingress parity, jobs=%0d emitted=%0d pose_stalls=%0d child_stalls=%0d resets=%0d images=%0d,%0d",jobs,emitted,pose_stalls,child_stalls,resets,image_completed[0],image_completed[1]);$finish;end endmodule']
    (p/'tb.sv').write_text('\n'.join(lines)+'\n')
    cmd=['verilator','--binary','--timing','--assert','-Wno-fatal','-j','4','--top-module','tb','--Mdir',str(p/'sim'),str(p/'packet-baseline.v'),str(p/'packet-candidate.v'),str(p/'tb.sv'),*([str(model)] if model is not None else [])]
    with (p/'compile.log').open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
    if r.returncode: raise AssertionError((p/'compile.log').read_text()[-5000:])
    r=subprocess.run([str(p/'sim/Vtb')],capture_output=True,text=True);(p/'parity.log').write_text(r.stdout+r.stderr);
    if r.returncode: raise AssertionError(r.stdout+r.stderr)
    return r.stdout+r.stderr


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class SharedIngressTests(unittest.TestCase):
    def test_shared_ingress_preserves_all_queues_and_wire_values(self):
        with tempfile.TemporaryDirectory(prefix="lutcam-shared-ingress-") as work:
            output = _run_parity(_rtl(False), _rtl(True), Path(work))
        self.assertIn("PASS 4000000 cycle", output)

    @unittest.skipUnless(os.environ.get("LITEX_ENV_EFINITY"), "Efinity DSP model is required")
    def test_metadata_dsp_preserves_all_capture_and_reset_edges(self):
        model = Path(os.environ["LITEX_ENV_EFINITY"]) / "sim_models/verilog/efx_dsp48.v"
        with tempfile.TemporaryDirectory(prefix="lutcam-packet-metadata-") as work:
            output = _run_parity(_rtl(True), _rtl(True, dsp=True), Path(work), dsp=True, model=model)
        self.assertIn("PASS 4000000 cycle", output)

    def test_shared_ingress_requires_single_accepted_centroid_producer(self):
        with self.assertRaises(ValueError):
            SharedPacketTransport(shared_solver_ingress=True)


if __name__ == "__main__":
    unittest.main()
