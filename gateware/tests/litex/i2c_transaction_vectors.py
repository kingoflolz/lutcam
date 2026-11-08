"""Bounded I2C transaction stimulus and independent open-drain target oracle."""
from pathlib import Path
import json
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'fpga/litex/lutcam/i2c.py').exists())
sys.path[:0] = [str(ROOT / 'fpga/litex'), str(ROOT / 'tests/litex')]
from migen.sim import passive, run_simulation
import test_i2c_engine as wire
from i2c_transaction_harness import Harness


def make_trace(implementation, destination):
    destination = Path(destination)
    dut = Harness(implementation)
    core, master = dut.core, dut.master
    targets = [wire.RegisterTarget(dut, c, address) for c,address in enumerate((0x60,0x10))]
    rows, cases = [], []
    metadata = {'seed': None, 'case': 'initial'}
    pre_inputs = {}
    @passive
    def before_edge():
        while True:
            for name, signal in dut.inputs.items(): pre_inputs[name] = (yield signal)
            yield
    def mark(name):
        metadata['case'] = name
        cases.append(name)
    @passive
    def monitor():
        while True:
            ins = {}
            for name,signal in dut.inputs.items(): ins[name] = (yield signal)
            outs = {}
            for name,signal in dut.outputs.items(): outs[name] = (yield signal)
            rows.append({'inputs':dict(pre_inputs),'post_inputs':ins,'outputs':outs,'seed':metadata['seed'],'case':metadata['case']})
            metadata['seed'] = None
            yield
    def idle(n=1):
        for _ in range(n): yield
    def reset():
        yield master.cyc.eq(0); yield master.stb.eq(0); yield master.we.eq(0)
        yield core.record_first.eq(0); yield core.record_last.eq(0); yield core.record_abort_sources.eq(0)
        yield dut.reset.eq(1)
        yield from idle(3)
        yield dut.reset.eq(0)
        yield from idle(5)
    def public_ids(bus, accepted, done=None):
        yield core.record_source.eq(3)
        assert (yield from wire.read_word(bus,wire.COMMAND)) == accepted
        assert (yield from wire.read_word(bus,wire.NEXT)) == ((accepted+1)&0xffffffff)
        if done is not None: assert (yield from wire.read_word(bus,wire.DONE)) == done
    def finish(bus, command, length):
        assert (yield from wire.wait_done(bus,command)) == length<<16
    def real_write(camera, command, data, register=0x4200, source=1, slow=False):
        bus=core.buses[camera]
        yield from wire.stage(bus,targets[camera].address,register,data,slow=slow)
        yield from wire.submit(bus,command,source=source,continuous=True)
        yield from finish(bus,command,len(data))
        assert bytes(targets[camera].memory[register+i] for i in range(len(data))) == bytes(data)
    def bench():
        mark('reset_and_physical_windows')
        yield from reset()
        for camera,bus in enumerate(core.buses):
            yield from wire.write_word(bus,wire.CONTROL,camera+1)
            assert (yield from wire.read_word(bus,wire.CONTROL)) == camera+1
            yield from wire.write_word(bus,wire.CONTROL,0)
        yield from idle(10)
        command=0
        for camera,bus in enumerate(core.buses):
            payload=bytes((0x91+camera*23+31*i)&255 for i in range(16))
            mark(f'camera{camera}_write16_'+('slow' if camera else 'fast'))
            command+=1
            yield from real_write(camera,command,payload,slow=bool(camera))
            mark(f'camera{camera}_read16_and_duplicate')
            yield from wire.stage(bus,targets[camera].address,0x4200,length=16,read=True)
            command+=1
            yield from wire.submit(bus,command,source=2)
            yield from finish(bus,command,16)
            for i in range(4):
                assert (yield from wire.read_word(bus,wire.RX+4*i)) == int.from_bytes(payload[4*i:4*i+4],'little')
            starts=len(targets[camera].starts)
            yield from wire.submit(bus,command,source=2)
            yield from public_ids(bus,command,command)
            assert len(targets[camera].starts)==starts
        bus=core.buses[0]
        mark('all32_descriptor_bits_retained_after_abort')
        for descriptor in [1<<bit for bit in range(32)]+[0xffffffff,0x80080000,0x7ff7ffff]:
            yield from wire.record_word(bus,0,command+1,first=True,count=4,source=1)
            yield from wire.record_word(bus,1,descriptor,count=4,source=1)
            yield core.record_source.eq(3)
            assert (yield from wire.read_word(bus,wire.DESC)) == descriptor
            yield core.record_abort_sources.eq(2); yield; yield core.record_abort_sources.eq(0)
            assert (yield from wire.read_word(bus,wire.DESC)) == descriptor
            yield from public_ids(bus,command,command)
        mark('invalid_descriptor_commit_does_not_consume_id')
        yield from wire.record(bus,[command+1,0xffffffff,0xdeadbeef,command+1],source=1)
        yield from public_ids(bus,command,command)
        assert not (yield core.engine.busy)
        mark('foreign_record_cannot_replace_owner')
        yield from wire.record_word(bus,0,command+1,first=True,count=4,source=1)
        yield from wire.record_word(bus,1,wire.descriptor(0x60,0x4500,4),count=4,source=1)
        yield from wire.record(core.buses[1],[command+1,0xffffffff,0xdeadbeef,command+1],source=2)
        yield from wire.record_word(bus,2,0x76543210,count=4,source=1)
        yield from wire.record_word(bus,3,command+1,last=True,count=4,source=1)
        command+=1; yield from finish(bus,command,4)
        assert bytes(targets[0].memory[0x4500+i] for i in range(4))==b'\x10\x32\x54\x76'
        mark('same_source_replacement_changes_camera')
        yield from wire.record_word(bus,0,command+1,first=True,count=4,source=1)
        yield from wire.record_word(bus,1,0xffffffff,count=4,source=1)
        command+=1; yield from real_write(1,command,b'\xa5',register=0x4504,source=1)
        mark('abort_wins_commit')
        yield from wire.record_word(bus,0,command+1,first=True,count=4,source=1)
        yield from wire.record_word(bus,1,wire.descriptor(0x60,0x4600,4),count=4,source=1)
        yield from wire.record_word(bus,2,0x12345678,count=4,source=1)
        yield from wire.record_word(bus,3,command+1,last=True,count=4,source=1,abort_sources=2)
        yield from public_ids(bus,command,command)
        mark('lease_expiration_retains_descriptor_and_next_id')
        yield from wire.record_word(bus,0,command+1,first=True,count=4,source=1)
        yield from wire.record_word(bus,1,0xfedcba98,count=4,source=1)
        yield core.record_source.eq(3); yield from idle(260)
        assert not (yield core.busy)
        assert (yield from wire.read_word(bus,wire.DESC))==0xfedcba98
        yield from public_ids(bus,command,command)
        yield from wire.record_word(bus,2,0xabcdef01,count=4,source=1)
        yield from wire.record_word(bus,3,command+1,last=True,count=4,source=1)
        yield from public_ids(bus,command,command)
        # Cover all three cancellation signals and early/late serialized
        # phases without replaying the old exhaustive RAM-conversion matrix.
        phases=[(1,1,'cyc',0,False),(1,2,'stb',1,True),(1,3,'both',1,False),
                (2,0,'cyc',0,True),(3,1,'stb',1,True),(2,3,'both',1,True)]
        for camera in (0,1):
            for state,lane,withdraw,opposite,abort in phases:
                mark(f'cancel_camera{camera}_state{state}_lane{lane}_{withdraw}_other{opposite}_abort{int(abort)}')
                yield from reset(); bus=core.buses[camera]
                yield from wire.record_word(bus,0,1,first=True,count=4,source=2)
                yield from wire.record_word(bus,1,wire.descriptor(targets[camera].address,0x4700,4),count=4,source=2)
                if state!=1:
                    yield from wire.record_word(bus,2,0x44332211,count=4,source=2)
                    yield core.record_abort_sources.eq(4); yield; yield core.record_abort_sources.eq(0)
                yield bus.adr.eq((wire.RECORD+8 if state==1 else wire.RX)//4)
                yield bus.dat_w.eq(0x88776655); yield bus.sel.eq(15)
                yield bus.we.eq(state==1); yield bus.cyc.eq(1); yield bus.stb.eq(1)
                for _ in range(24):
                    yield
                    if (yield dut.private['host_state'])==state and (yield dut.private['host_lane'])==lane: break
                else: raise AssertionError('serialized cancellation phase not reached')
                assert not (yield bus.ack)
                follow=core.buses[camera^opposite]
                yield bus.cyc.eq(withdraw=='stb'); yield bus.stb.eq(withdraw=='cyc')
                yield follow.adr.eq(wire.MAGIC//4); yield bus.dat_w.eq(0xdecafbad); yield bus.we.eq(0)
                yield core.record_first.eq(0); yield core.record_last.eq(0); yield core.record_source.eq(3)
                if abort: yield core.record_abort_sources.eq(4)
                yield
                assert not (yield bus.ack)
                yield core.record_abort_sources.eq(0)
                assert (yield from wire.read_word(follow,wire.MAGIC))==0x49324301
                assert (yield from wire.read_word(follow,wire.RX))==(0x88776655 if state==1 else 0x44332211)
                yield from public_ids(follow,0,0)
                if state==1:
                    yield from wire.record_word(bus,3,1,last=True,count=4,source=2)
                    if abort: yield from public_ids(bus,0,0)
                    else:
                        yield from finish(bus,1,4)
                        assert bytes(targets[camera].memory[0x4700+i] for i in range(4))==b'\x55\x66\x77\x88'
        for state,lane in [(1,1),(1,3),(2,0),(3,3)]:
            mark(f'reset_serialized_state{state}_lane{lane}')
            yield from reset(); bus=core.buses[0]
            yield from wire.record_word(bus,0,1,first=True,count=4,source=2)
            yield from wire.record_word(bus,1,wire.descriptor(0x60,0x4800,4),count=4,source=2)
            if state!=1:
                yield from wire.record_word(bus,2,0xffeeddcc,count=4,source=2)
                yield core.record_abort_sources.eq(4); yield; yield core.record_abort_sources.eq(0)
            yield bus.adr.eq((wire.RECORD+8 if state==1 else wire.RX)//4)
            yield bus.dat_w.eq(0x88776655); yield bus.sel.eq(15); yield bus.we.eq(state==1)
            yield bus.cyc.eq(1); yield bus.stb.eq(1)
            for _ in range(24):
                yield
                if (yield dut.private['host_state'])==state and (yield dut.private['host_lane'])==lane: break
            else: raise AssertionError('serialized reset phase not reached')
            yield from reset()
            yield from public_ids(bus,0,0)
            assert (yield from wire.read_word(bus,wire.DESC))==0
            for i in range(4): assert (yield from wire.read_word(bus,wire.RX+4*i))==0
        mark('target_nack_retains_error_then_retry')
        yield from reset(); bus=core.buses[0]
        targets[0].nack_at=3
        yield from wire.stage(bus,0x60,0x4880,b'\x15\x27')
        yield from wire.submit(bus,1,source=1)
        assert (yield from wire.wait_done(bus,1)) == (1 | (3<<8))
        yield from wire.submit(bus,1,source=1)
        assert (yield from wire.read_word(bus,wire.RESULT)) == (1 | (3<<8))
        targets[0].nack_at=None
        yield from real_write(0,2,b'\x15\x27',register=0x4880)
        mark('wire_scl_timeout_retains_error_then_recovery')
        yield from reset(); bus=core.buses[0]
        yield from wire.stage(bus,0x60,0x4884,b'\x39')
        yield from wire.submit(bus,1,source=1)
        for _ in range(100):
            if (yield core.scl[0].oe): break
            yield
        else: raise AssertionError('wire low phase not reached')
        yield dut.target_scl_low[0].eq(1)
        timeout_result=yield from wire.wait_done(bus,1)
        assert (timeout_result & 255)==2
        assert (timeout_result >> 16)==0
        for pin in (*core.scl,*core.sda): assert (yield pin.oe)==0
        yield from wire.submit(bus,1,source=1)
        assert (yield from wire.read_word(bus,wire.RESULT))==timeout_result
        yield dut.target_scl_low[0].eq(0)
        yield from idle(10)
        yield from real_write(0,2,b'\x39',register=0x4884)
        # Precondition only accepted storage while ingress is inactive. This
        # avoids2**32 real transactions and does not break the live invariant.
        for seed in (0x7ffffffe,0xfffffffe):
            mark(f'full_id_seed_{seed:08x}_then_two_real_commands')
            yield from reset(); bus=core.buses[0]
            assert not (yield dut.private['ingress_active'])
            yield dut.private['accepted_id'].eq(seed)
            metadata['seed']=seed
            yield from idle(2)
            yield from public_ids(bus,seed,0)
            for command in ((seed+1)&0xffffffff,(seed+2)&0xffffffff):
                yield from wire.stage(bus,0x60,0x4900,bytes([command&255]))
                yield from wire.submit(bus,command,source=1)
                # Read DONE on exactly the completion-capture edge. The read
                # response must contain the previous ID, then the next read new.
                old_done=(yield dut.private['done_id'])
                for _ in range(4000):
                    if (yield core.engine.done): break
                    yield
                else: raise AssertionError('completion edge not reached')
                value=yield from wire.read_word(bus,wire.DONE)
                assert value==old_done, ('read-at-capture',value,old_done)
                assert (yield from wire.read_word(bus,wire.DONE))==command
                yield from public_ids(bus,command,command)
                assert (yield from wire.read_word(bus,wire.RESULT))==1<<16
        mark('reset_active_wire_and_clean_recovery')
        yield from reset(); bus=core.buses[0]
        yield from wire.stage(bus,0x60,0x4a00,bytes(range(16)))
        yield from wire.submit(bus,1,source=1)
        yield from idle(70)
        assert (yield core.engine.busy)
        yield from reset()
        yield from public_ids(bus,0,0)
        yield from real_write(0,1,b'\x35\xa9',register=0x4a20)
        yield from public_ids(bus,1,1)
        yield from idle(10)
    run_simulation(dut, {'tb':bench(),'sys':[t.process() for t in targets],'pre':before_edge(),'trace':monitor()},
                   clocks={'sys':10,'tb':(10,5),'pre':(10,2),'trace':(10,8)})
    summary={'cases':cases,'cycles':len(rows),'seed_values':[r['seed'] for r in rows if r['seed'] is not None],
             'target_transactions':[len(t.transactions) for t in targets],
             'target_starts':[len(t.starts) for t in targets],
             'target_stops':[len(t.stops) for t in targets],
             'source':'Existing independent RegisterTarget uses only resolved open-drain pins; no internal controller state for ACK/data.'}
    destination.mkdir(parents=True,exist_ok=True)
    (destination/'trace.json').write_text(json.dumps(rows,separators=(',',':'))+'\n')
    (destination/'trace-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return rows,summary
