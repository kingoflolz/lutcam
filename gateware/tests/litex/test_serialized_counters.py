"""Production serialized counter bounds against the unrestricted reference."""
import random
import unittest
from migen import Constant, Module
from migen.sim import run_simulation
from fpga.litex.lutcam.counters import SharedPacketCounters


class SerializedCounterTests(unittest.TestCase):
    def test_bounded_windows_match_generic_counters_at_wrap_and_every_clear_phase(self):
        kwargs = dict(packet_totals=True, with_destinations=True, fixed_packet_layout=True,
            with_host_counters=True, compact_image_packets=True, batched_packets=True)
        dut = Module()
        dut.submodules.reference = reference = SharedPacketCounters(**kwargs)
        dut.submodules.variant = candidate = SharedPacketCounters(**kwargs, serialized_datagrams=True)
        for name in ('host_event', 'clear_host', 'clear_camera', 'clear_statistics', 'query_valid',
                     'query_address', 'config_write_valid', 'config_write_address', 'config_write_data'):
            dut.comb += getattr(candidate, name).eq(getattr(reference, name))
        for index in range(6):
            for field, _ in reference.events[index].layout:
                dut.comb += getattr(candidate.events[index], field).eq(getattr(reference.events[index], field))
            dut.comb += candidate.committed_levels[index].eq(reference.committed_levels[index])
        for counter in (reference, candidate):
            counter.memory.init = [0xffffffed - index for index in range(counter.memory.depth)]
            for initialized in counter.initialized: initialized.reset = Constant(1)

        rng = random.Random(0x427344)
        stats = dict(cycles=40000, queries=0, packet_completions=[0]*6,
                     maximum_pending=[0]*candidate.storage_count)
        clear_phases = {name: set() for name in ('clear_host', 'clear_camera', 'clear_statistics')}
        query_collisions = set()
        def bench():
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
                for index, pending in enumerate(candidate.pending):
                    stats['maximum_pending'][index] = max(stats['maximum_pending'][index], (yield pending))
                yield reference.query_valid.eq(1)
                # Alternating sequential walks and random requests cover every physical
                # updater/query collision, invalid addresses and destination requests.
                yield reference.query_address.eq((cycle//2)%50 if cycle%1024 < 512 else rng.randrange(64))
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
        assert stats['queries'] > 17000
        assert max(stats['maximum_pending']) >= 1248
        assert stats['maximum_pending'][0] == 7
        stats['ok'] = True
        stats['clear_phases_per_type'] = {name: len(phases) for name, phases in clear_phases.items()}
        stats['counter_query_collisions'] = len(query_collisions)
        stats['pending_bits_reference'] = sum(len(v) for v in reference.pending)
        stats['pending_bits_candidate'] = sum(len(v) for v in candidate.pending)
