"""Production serialized event descriptors preserve exact counter queries."""
import random
import unittest
from migen import Array, Constant, Module, Mux, Signal
from migen.sim import run_simulation
from fpga.litex.lutcam.counters import SharedPacketCounters


class SerializedEventDescriptorTests(unittest.TestCase):
    def test_descriptors_match_generic_at_wrap_and_every_clear_phase(self):
        kwargs = dict(packet_totals=True, with_destinations=True, fixed_packet_layout=True,
            with_host_counters=True, compact_image_packets=True, batched_packets=True)
        dut = Module()
        dut.submodules.reference = reference = SharedPacketCounters(**kwargs)
        dut.submodules.variant = candidate = SharedPacketCounters(**kwargs, serialized_datagrams=True, serialized_event_descriptor=True)
        for name in ('host_event', 'clear_host', 'clear_camera', 'clear_statistics', 'query_valid',
                     'query_address', 'config_write_valid', 'config_write_address', 'config_write_data'):
            dut.comb += getattr(candidate, name).eq(getattr(reference, name))
        for index in range(6):
            for field, _ in reference.events[index].layout:
                dut.comb += getattr(candidate.events[index], field).eq(getattr(reference.events[index], field))
            dut.comb += candidate.committed_levels[index].eq(reference.committed_levels[index])
        # Force collisions against the candidate's priority updater as well as
        # exercising the old periodic schedule. Looking only at the reference
        # address would not prove the new descriptor forwarding paths ran.
        requested_query = Signal.like(reference.query_address)
        force_candidate_collision = Signal()
        physical_queries = [stream * 6 + field for stream, field in candidate.physical_fields] + [48, 49]
        dut.comb += reference.query_address.eq(Mux(force_candidate_collision,
            Array(physical_queries)[candidate.update_address], requested_query))
        for counter in (reference, candidate):
            counter.memory.init = [0xffffffed - index for index in range(counter.memory.depth)]
            for initialized in counter.initialized: initialized.reset = Constant(1)

        rng = random.Random(0x427344)
        stats = dict(cycles=40000, queries=0, packet_completions=[0]*6,
                     maximum_pending=[0]*candidate.storage_count)
        clear_phases = {name: set() for name in ('clear_host', 'clear_camera', 'clear_statistics')}
        query_collisions = set()
        candidate_query_collisions = set()
        fault_indices = [i for i, (_, field) in enumerate(candidate.physical_fields)
                         if field not in (1, 2)] + [candidate.packet_storage_count,
                                                   candidate.packet_storage_count + 1]
        fault_age = {index: 0 for index in fault_indices}
        maximum_fault_age = 0
        def bench():
            nonlocal maximum_fault_age
            next_completion = 1
            stream = 0
            length = 1056
            for cycle in range(stats['cycles']):
                assert (yield reference.query_ready) == (yield candidate.query_ready)
                assert (yield reference.config_write_ready) == (yield candidate.config_write_ready)
                valid = (yield reference.query_response_valid)
                assert valid == (yield candidate.query_response_valid)
                if valid:
                    actual, expected = (yield candidate.query_data), (yield reference.query_data)
                    assert actual == expected, (cycle, actual, expected)
                    stats['queries'] += 1
                phase = 2 * (yield reference.update_address) + (yield reference.update_write)
                for name, seen in clear_phases.items():
                    value = (yield getattr(reference, name))
                    for camera in range(2):
                        if value & (1 << camera): seen.add((camera, phase))
                if (yield reference.query_valid) and (yield reference.query_ready):
                    address = (yield reference.query_address)
                    physical = None
                    if address < 36:
                        key = address // 6, address % 6
                        if key in reference.physical_fields: physical = reference.physical_fields.index(key)
                    elif address >= 48: physical = reference.packet_storage_count + address - 48
                    if physical == (yield reference.update_address): query_collisions.add(physical)
                    if physical == (yield candidate.update_address): candidate_query_collisions.add(physical)
                for index in fault_indices:
                    fault_age[index] += 1
                    if not (yield candidate.update_write) and (yield candidate.update_address) == index:
                        maximum_fault_age = max(maximum_fault_age, fault_age[index])
                        assert fault_age[index] <= 80, (cycle, index, fault_age[index])
                        fault_age[index] = 0
                    assert fault_age[index] <= 80, (cycle, index, fault_age[index])
                for index, pending in enumerate(candidate.pending):
                    stats['maximum_pending'][index] = max(stats['maximum_pending'][index], (yield pending))
                yield reference.query_valid.eq(1)
                # Alternating sequential walks and random requests cover every physical
                # updater/query collision, invalid addresses and destination requests.
                yield requested_query.eq((cycle//2)%50 if cycle%1024 < 512 else rng.randrange(64))
                yield force_candidate_collision.eq(cycle >= 20000)
                yield reference.config_write_valid.eq(cycle % 3 != 0)
                yield reference.config_write_address.eq(rng.randrange(16))
                yield reference.config_write_data.eq(rng.getrandbits(32))
                yield reference.host_event.eq(3)
                # Keep one clear target for 56 intervals before switching targets. Each
                # odd interval is coprime to the 56-clock sweep, covering all phases for
                # both targets; alternating target every interval would skip parity.
                yield reference.clear_host.eq(1 << ((cycle//(57*56))%2) if cycle%57 == 0 else 0)
                yield reference.clear_camera.eq(1 << ((cycle//(113*56))%2) if cycle%113 == 0 else 0)
                yield reference.clear_statistics.eq(1 << ((cycle//(127*56))%2) if cycle%127 == 0 else 0)
                completed = cycle == next_completion
                if completed:
                    assert not (yield candidate.descriptor_sent_pending), cycle
                    assert not (yield candidate.descriptor_bytes_pending), cycle
                for index, event in enumerate(reference.events):
                    yield reference.committed_levels[index].eq(rng.randrange(16))
                    yield event.committed.eq(0)
                    yield event.sent.eq(completed and index == stream)
                    yield event.bytes.eq(length if index == stream else 0)
                    # Fault and rejection events remain unconstrained at one per cycle.
                    yield event.overflow.eq(1)
                    yield event.protocol.eq(1)
                    yield event.reject.eq(1)
                if completed:
                    stats['packet_completions'][stream] += 1
                    # Deliberately combine a maximum-size first packet with repeated
                    # eight-word packets, reaching the conservative window boundary.
                    if cycle < 4000:
                        stream = 0
                        length = 32 if (stats['packet_completions'][0] % 12) else 1056
                    else:
                        stream = rng.randrange(6)
                        length = rng.choice((32, (1056,320,208)[stream%3], 64, 120))
                    next_completion += (length + 3)//4
                yield
            for _ in range(5): yield
        run_simulation(dut, bench())
        for name, phases in clear_phases.items():
            assert phases == {(camera, phase) for camera in range(2) for phase in range(56)}, (name, phases)
        assert query_collisions == set(range(candidate.storage_count)), query_collisions
        assert candidate_query_collisions == set(range(candidate.storage_count)), candidate_query_collisions
        assert stats['queries'] > 17000
        assert max(stats['maximum_pending']) == 1056
        assert stats['maximum_pending'][0] == 1
        stats['ok'] = True
        stats['clear_phases_per_type'] = {name: len(phases) for name, phases in clear_phases.items()}
        stats['counter_query_collisions'] = len(query_collisions)
        stats['candidate_query_collisions'] = len(candidate_query_collisions)
        stats['maximum_fault_capture_interval'] = maximum_fault_age
        stats['pending_bits_reference'] = sum(len(v) for v in reference.pending)
        stats['pending_bits_candidate'] = sum(len(v) for v in candidate.pending)
        self.coverage = stats
