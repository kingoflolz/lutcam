"""Exact shared packet diagnostics under simultaneous maximum-rate events."""

import random
import unittest

from migen import Constant, If, ResetInserter
from migen.sim import run_simulation

from fpga.litex.lutcam.counters import (
    COUNTER_FIELDS, COUNTER_MAX_INCREMENT, SharedPacketCounters,
)


MASK = (1 << 32) - 1


class SharedCounterTests(unittest.TestCase):
    def exercise(self, steps, *, initial=None, expect_collisions=False, packet_totals=False,
                 expect_clear_phases=False, with_destinations=False, fixed_packet_layout=False,
                 compact_image_packets=False, batched_packets=False):
        dut = ResetInserter()(SharedPacketCounters(packet_totals=packet_totals,
                                                 with_destinations=with_destinations,
                                                 fixed_packet_layout=fixed_packet_layout,
                                                 compact_image_packets=compact_image_packets,
                                                 batched_packets=batched_packets))
        if packet_totals:
            for stream, (event, level) in enumerate(zip(dut.events, dut.committed_levels)):
                dut.sync += If(dut.clear_camera[stream // 3], level.eq(0)).Else(
                    level.eq(level + event.committed - event.sent))
        if initial is not None:
            # Seed the architectural RAM state near wrap without billions of
            # simulation clocks. All normal accesses remain through the DUT.
            dut.memory.init = [initial[stream * 6 + field] for stream, field in dut.physical_fields]
            for valid in dut.initialized:
                valid.reset = Constant(1)
        counters = list(initial or [0] * 36)
        destination_defaults = [0 if field == 0 else 6000 + stream
                                for stream in range(6) for field in range(2)]
        destinations = list(destination_defaults)
        collisions = set()
        clear_phases = set()
        maxima = [0] * dut.storage_count

        def bench():
            expected = None
            for cycle, step in enumerate([*steps, {}, {}, {}]):
                self.assertEqual((yield dut.query_response_valid), expected is not None,
                                 f"response valid at cycle {cycle}")
                if expected is not None:
                    self.assertEqual((yield dut.query_data), expected,
                                     f"query result at cycle {cycle}")

                address = (yield dut.query_address)
                self.assertEqual((yield dut.query_ready),
                                 bool((yield dut.update_write)) and address < dut.query_count)
                accepting = (yield dut.query_valid) and (yield dut.query_ready)
                expected = ((counters[address] if address < 36 else destinations[address - 36])
                            if accepting else None)
                if accepting and address < 36 and (yield dut.update_write):
                    key = (address // 6, address % 6)
                    if key not in dut.physical_fields:
                        key = (address // 6, 1)
                    physical = dut.physical_fields.index(key)
                    if physical == (yield dut.update_address):
                        collisions.add(physical)
                if (yield dut.reset):
                    counters[:] = [0] * 36
                    destinations[:] = destination_defaults
                    expected = None
                else:
                    clear = (yield dut.clear_camera)
                    for camera in range(2):
                        if clear & (1 << camera):
                            phase = 2 * (yield dut.update_address) + (yield dut.update_write)
                            clear_phases.add((camera, phase))
                    for stream, event in enumerate(dut.events):
                        for field_index, field in enumerate(COUNTER_FIELDS):
                            index = stream * 6 + field_index
                            increment = (yield getattr(event, field))
                            if field == "bytes" and not (yield event.sent):
                                increment = 0
                            if fixed_packet_layout and field == "reject" and stream % 3 != 2:
                                increment = 0
                            if clear & (1 << (stream // 3)):
                                counters[index] = 0
                            else:
                                counters[index] = (counters[index] + increment) & MASK
                    if with_destinations:
                        write_address = (yield dut.config_write_address)
                        self.assertEqual((yield dut.config_write_ready),
                                         not (yield dut.update_write) and write_address < 12)
                        if (yield dut.config_write_valid) and (yield dut.config_write_ready):
                            value = (yield dut.config_write_data)
                            destinations[write_address] = value & (0xffff if write_address % 2 else MASK)
                        for index in range(12):
                            if clear & (1 << (index // 6)):
                                destinations[index] = destination_defaults[index]
                    for index, ((stream, field_index), pending) in enumerate(
                            zip(dut.physical_fields, dut.pending)):
                        value = (yield pending)
                        maxima[index] = max(maxima[index], value)
                        maximum = COUNTER_MAX_INCREMENT[field_index]
                        if field_index == 2:
                            if batched_packets:
                                maximum = (1056, 320, 208)[stream % 3]
                            elif compact_image_packets and stream % 3 == 0:
                                maximum = 1056
                            elif fixed_packet_layout:
                                maximum = 104
                        self.assertLessEqual(value, dut.update_period * maximum)

                yield dut.reset.eq(step.get("reset", 0))
                yield dut.clear_camera.eq(step.get("clear", 0))
                query = step.get("query")
                yield dut.query_valid.eq(query is not None)
                yield dut.query_address.eq(query if query is not None else 0)
                if with_destinations:
                    config = step.get("config")
                    yield dut.config_write_valid.eq(config is not None)
                    yield dut.config_write_address.eq(config[0] if config else 0)
                    yield dut.config_write_data.eq(config[1] if config else 0)
                values = step.get("events", [[0] * 6 for _ in range(6)])
                for event, increments in zip(dut.events, values):
                    for field, increment in zip(COUNTER_FIELDS, increments):
                        yield getattr(event, field).eq(increment)
                yield
            if expect_collisions:
                self.assertEqual(collisions, set(range(dut.storage_count)))
            if expect_clear_phases:
                self.assertEqual(clear_phases, {(camera, phase) for camera in range(2)
                                               for phase in range(dut.update_period)})

        run_simulation(dut, bench())
        return maxima

    def test_maximum_events_every_clock_and_all_query_write_collisions(self):
        # Two consecutive queries per counter deliberately encounter every
        # updater write phase, and exercise maximum input rates continuously.
        steps = [{"events": [list(COUNTER_MAX_INCREMENT)] * 6,
                  "query": (cycle // 2) % 36}
                 for cycle in range(720)]
        maxima = self.exercise(steps, expect_collisions=True)
        self.assertEqual(maxima, [72 * maximum
                                 for _ in range(6)
                                 for maximum in COUNTER_MAX_INCREMENT])

    def test_random_queries_events_and_camera_clears(self):
        rng = random.Random(0x32109)
        steps = []
        for cycle in range(1200):
            steps.append({
                "events": [[rng.randrange(maximum + 1)
                            for maximum in COUNTER_MAX_INCREMENT] for _ in range(6)],
                "query": rng.randrange(64) if cycle % 7 else None,
                "clear": (1 << rng.randrange(2)) if cycle % 43 == 0 else 0,
            })
        steps.extend({"query": index} for index in range(36) for _ in range(2))
        self.exercise(steps)

    def test_modulo_wrap_for_all_fields_and_query_collisions(self):
        initial = [MASK - index for index in range(36)]
        steps = [{"events": [list(COUNTER_MAX_INCREMENT)] * 6,
                  "query": (cycle // 2) % 36}
                 for cycle in range(216)]
        self.exercise(steps, initial=initial, expect_collisions=True)

    def test_clear_on_every_updater_phase_retains_next_edge_events(self):
        steps = []
        for cycle in range(5328):
            steps.append({
                "events": [list(COUNTER_MAX_INCREMENT)] * 6,
                "query": (cycle // 2) % 36,
                # Coprime interval covers every update address and phase.
                "clear": 1 << (cycle // 2664) if cycle % 37 == 0 else 0,
            })
        steps.extend({"query": index} for index in range(36) for _ in range(2))
        self.exercise(steps, expect_clear_phases=True)

    def test_global_reset_discards_old_ram_and_held_clear_is_immediate(self):
        steps = []
        for cycle in range(400):
            steps.append({
                "events": [list(COUNTER_MAX_INCREMENT)] * 6,
                "query": (cycle // 2) % 36,
                "reset": cycle in (180, 181, 255),
                "clear": 1 if 90 <= cycle < 100 else 0,
            })
        steps.extend({"query": index} for index in range(36) for _ in range(2))
        self.exercise(steps)

    def test_byte_input_requires_sent_and_invalid_queries_do_not_respond(self):
        events = [[2, 0, 1440, 1, 1, 1]] * 6
        steps = [{"events": events, "query": cycle % 64} for cycle in range(144)]
        steps.extend({"query": index} for index in range(36) for _ in range(2))
        self.exercise(steps)

    def test_sequence_queries_include_every_prior_sent_pulse(self):
        steps = []
        for packet in range(24):
            stream = packet % 6
            values = [[0] * 6 for _ in range(6)]
            values[stream] = [0, 1, 1056, 0, 0, 0]
            steps.append({"events": values})
            # The serializer may query before the updater reaches this stream;
            # exact snapshots must include the as-yet-unflushed sent event.
            steps.extend({"query": stream * 6 + 1} for _ in range(2))
            # Move across both updater phases and all stream service offsets.
            steps.extend({} for _ in range(packet % 5))
        self.exercise(steps)

    def test_packet_totals_maximum_events_and_all_logical_queries(self):
        events = [[1, 1, (1056, 1440, 120)[stream % 3], 1, 1, 1]
                  for stream in range(6)]
        physical = [stream * 6 + field for stream, field in
                    SharedPacketCounters(packet_totals=True).physical_fields]
        steps = [{"events": events, "query": physical[(cycle // 2) % 26]}
                 for cycle in range(104)]
        steps += [{"events": events, "query": (cycle // 2) % 36}
                  for cycle in range(936)]
        self.exercise(steps, packet_totals=True, expect_collisions=True)

    def test_packet_totals_queue_conservation_and_resets(self):
        rng = random.Random(0x1ee7)
        levels = [0] * 6
        steps = []
        for cycle in range(1500):
            reset = cycle in (777, 778)
            clear = 3 if reset else (1 << rng.randrange(2)) if cycle % 29 == 0 else 0
            events = []
            for stream in range(6):
                committed = rng.randrange(3) if levels[stream] <= 6 else 0
                sent = rng.randrange(2) if levels[stream] else 0
                size = (1056, rng.randrange(1, 1441), 120)[stream % 3]
                events.append([committed, sent, size, rng.randrange(2),
                               rng.randrange(2), rng.randrange(2)])
                levels[stream] = (0 if clear & (1 << (stream // 3)) else
                                  levels[stream] + committed - sent)
            steps.append({"events": events, "clear": clear, "reset": reset,
                          "query": rng.randrange(64)})
        steps.extend({"query": index} for index in range(36) for _ in range(2))
        self.exercise(steps, packet_totals=True)

    def test_packet_totals_derivations_wrap_modulo_32_bits(self):
        initial = []
        for stream in range(6):
            sent = MASK - stream
            size = (1056, 1440, 120)[stream % 3]
            initial.extend([sent, sent, (sent * size) & MASK, MASK - 1, MASK - 2, MASK - 3])
        events = [[1, 1, (1056, 1440, 120)[stream % 3], 1, 1, 1]
                  for stream in range(6)]
        self.exercise([{"events": events, "query": (cycle // 2) % 36}
                       for cycle in range(216)], initial=initial, packet_totals=True)

    def test_packet_totals_pending_widths_follow_shorter_service_bound(self):
        dut = SharedPacketCounters(packet_totals=True)
        self.assertEqual(dut.update_period, 52)
        self.assertEqual(sum(len(value) for value in dut.pending), 178)
        self.assertEqual((dut.memory.width, dut.memory.depth), (32, 26))

    def test_destination_defaults_writes_and_camera_isolation(self):
        steps = [{"query": address} for address in range(36, 48) for _ in range(2)]
        for index in range(12):
            # Every query overlaps a write request; phase separation guarantees
            # the reference sees either the value before or after acceptance.
            steps += [{"query": 36 + index, "config": (index, 0x12345600 + index)}
                      for _ in range(4)]
        steps += [{"clear": 1}, {}]
        steps += [{"query": address} for address in range(36, 48) for _ in range(2)]
        steps += [{"clear": 2}, {}]
        steps += [{"query": address} for address in range(36, 48) for _ in range(2)]
        self.exercise(steps, packet_totals=True, with_destinations=True)

    def test_destination_clear_wins_write_and_global_reset_restores_defaults(self):
        steps = []
        for index in range(12):
            steps += [{"config": (index, MASK), "clear": 1 << (index // 6)}
                      for _ in range(2)]
            steps += [{"query": 36 + index} for _ in range(2)]
            steps += [{"config": (index, MASK)} for _ in range(2)]
        steps += [{"reset": 1}, {"reset": 1}, {}]
        steps += [{"query": address} for address in range(36, 48) for _ in range(2)]
        self.exercise(steps, packet_totals=True, with_destinations=True)

    def test_destination_writes_never_delay_maximum_counter_events(self):
        rng = random.Random(0xd357)
        events = [[1, 1, (1056, 1440, 120)[stream % 3], 1, 1, 1]
                  for stream in range(6)]
        steps = [{"events": events, "query": rng.randrange(64),
                  "config": (rng.randrange(16), rng.getrandbits(32)),
                  "clear": rng.randrange(4) if cycle % 37 == 0 else 0}
                 for cycle in range(1800)]
        steps += [{"query": address} for address in range(48) for _ in range(2)]
        self.exercise(steps, packet_totals=True, with_destinations=True)

    def test_destination_rows_do_not_extend_counter_sweep(self):
        dut = SharedPacketCounters(packet_totals=True, with_destinations=True)
        self.assertEqual(dut.update_period, 52)
        self.assertEqual(sum(len(value) for value in dut.pending), 178)
        self.assertEqual((dut.memory.width, dut.memory.depth), (32, 38))
        self.assertEqual(len(dut.destination_valid), 12)
        with self.assertRaises(ValueError):
            SharedPacketCounters(with_destinations=True)

    def test_fixed_layout_counter_bounds_and_all_query_collisions(self):
        variant = SharedPacketCounters(packet_totals=True, with_destinations=True,
                                       fixed_packet_layout=True)
        self.assertEqual(variant.storage_count, 22)
        self.assertEqual(variant.update_period, 44)
        self.assertEqual(sum(len(value) for value in variant.pending), 146)
        self.assertEqual((variant.memory.width, variant.memory.depth), (32, 34))
        events = [[1, 1, (1056, 104, 120)[stream % 3], 1, 1, 1] for stream in range(6)]
        addresses = [stream * 6 + field for stream, field in variant.physical_fields]
        steps = [{"events": events, "query": addresses[(cycle // 2) % 22]}
                 for cycle in range(88)]
        steps += [{"events": events, "query": (cycle // 2) % 36} for cycle in range(792)]
        self.exercise(steps, packet_totals=True, with_destinations=True,
                      fixed_packet_layout=True, expect_collisions=True)
        with self.assertRaises(ValueError):
            SharedPacketCounters(fixed_packet_layout=True)

    def test_fixed_layout_continuous_events_configuration_and_all_clear_phases(self):
        rng = random.Random(0xf17ed)
        events = [[1, 1, (1056, 104, 120)[stream % 3], 1, 1, stream % 3 == 2]
                  for stream in range(6)]
        steps = [{"events": events, "query": rng.randrange(64),
                  "config": (rng.randrange(16), rng.getrandbits(32)),
                  "clear": 1 << (cycle // 1980) if cycle % 45 == 0 else 0}
                 for cycle in range(3960)]
        steps += [{"reset": 1}, {"reset": 1}, {}]
        steps += [{"query": address} for address in range(48) for _ in range(2)]
        self.exercise(steps, packet_totals=True, with_destinations=True,
                      fixed_packet_layout=True, expect_clear_phases=True)

    def test_fixed_layout_modulo_wrap_and_constant_zero_rejections(self):
        initial = []
        for stream in range(6):
            sent = MASK - stream
            size = (1056, 104, 120)[stream % 3]
            initial.extend([sent, sent, (sent * size) & MASK, MASK - 1, MASK - 2,
                            MASK - 3 if stream % 3 == 2 else 0])
        events = [[1, 1, (1056, 104, 120)[stream % 3], 1, 1, 1] for stream in range(6)]
        steps = [{"events": events, "query": (cycle // 2) % 36} for cycle in range(288)]
        self.exercise(steps, initial=initial, packet_totals=True, with_destinations=True,
                      fixed_packet_layout=True)

    def test_pending_widths_follow_proven_service_bound(self):
        dut = SharedPacketCounters()
        self.assertEqual(dut.update_period, 72)
        self.assertEqual([len(value) for value in dut.pending], [8, 7, 17, 7, 7, 7] * 6)
        self.assertEqual((dut.memory.width, dut.memory.depth), (32, 36))

    def test_compact_image_generic_mode_bounds_and_all_collisions(self):
        dut = SharedPacketCounters(packet_totals=True, compact_image_packets=True)
        self.assertEqual(dut.storage_count, 28)
        self.assertEqual(dut.update_period, 56)
        self.assertEqual((dut.memory.width, dut.memory.depth), (32, 28))
        addresses = [stream * 6 + field for stream, field in dut.physical_fields]
        events = [[1, 1, (1056, 1440, 120)[stream % 3], 1, 1, 1]
                  for stream in range(6)]
        maxima = self.exercise([
            {"events": events, "query": addresses[(cycle // 2) % 28]}
            for cycle in range(560)], packet_totals=True,
            compact_image_packets=True, expect_collisions=True)
        for index, (stream, field) in enumerate(dut.physical_fields):
            maximum = (1056 if stream % 3 == 0 else 1440) if field == 2 else 1
            self.assertEqual(maxima[index], 56 * maximum)
        with self.assertRaises(ValueError):
            SharedPacketCounters(compact_image_packets=True)

    def test_batched_committed_counts_pending_datagrams_not_raw_records(self):
        rng = random.Random(0xba7c)
        levels = [0] * 6
        steps = []
        for cycle in range(1200):
            clear = (1 << rng.randrange(2)) if cycle % 53 == 0 else 0
            events = []
            for stream in range(6):
                # A batch plan commits one datagram regardless of the number
                # of raw child/pose queue records that its body contains.
                committed = int(levels[stream] == 0 and rng.randrange(3) == 0)
                sent = int(levels[stream] != 0 and rng.randrange(4) == 0)
                length = (rng.randrange(1, 513) * 2 + 32,
                          rng.randrange(1, 5) * 72 + 32,
                          rng.randrange(1, 3) * 88 + 32)[stream % 3]
                events.append([committed, sent, length, 0, 0, 0])
                levels[stream] = (0 if clear & (1 << (stream // 3)) else
                                  levels[stream] + committed - sent)
            steps.append({"events": events, "clear": clear, "query": (cycle // 2) % 36})
        steps += [{"query": address} for address in range(36) for _ in range(2)]
        self.exercise(steps, packet_totals=True, fixed_packet_layout=True,
                      compact_image_packets=True, batched_packets=True)
        for kwargs in ({}, {"compact_image_packets": True}, {"fixed_packet_layout": True}):
            with self.assertRaises(ValueError):
                SharedPacketCounters(packet_totals=True, batched_packets=True, **kwargs)


class HostCounterTests(unittest.TestCase):
    def exercise(self, steps, initial_hosts=(0, 0), all_clear_phases=False,
                 compact_image_packets=False, initial_packets=None,
                 all_counter_collisions=False, all_camera_clear_phases=False,
                 batched_packets=False):
        dut = ResetInserter()(SharedPacketCounters(packet_totals=True,
            with_destinations=True, fixed_packet_layout=True, with_host_counters=True,
            compact_image_packets=compact_image_packets, batched_packets=batched_packets))
        expected_rows = 24 + 2 * int(compact_image_packets) + 2 * int(batched_packets)
        self.assertEqual(dut.storage_count, expected_rows)
        self.assertEqual(dut.update_period, 2 * expected_rows)
        self.assertEqual(dut.memory.depth, expected_rows + 12)
        packets = list(initial_packets or [0] * 36)
        if initial_hosts != (0, 0) or initial_packets is not None:
            dut.memory.init = [packets[stream * 6 + field]
                               for stream, field in dut.physical_fields] + list(initial_hosts)
            for valid in dut.initialized:
                valid.reset = Constant(1)
        hosts = list(initial_hosts)
        defaults = [0 if field == 0 else 6000 + stream for stream in range(6) for field in range(2)]
        destinations = list(defaults)
        clear_phases = set()
        camera_clear_phases = set()
        collisions = set()
        counter_collisions = set()
        maxima = [0] * dut.storage_count
        def bench():
            expected = None
            for cycle, step in enumerate([*steps, {}, {}, {}]):
                self.assertEqual((yield dut.query_response_valid), expected is not None)
                if expected is not None:
                    self.assertEqual((yield dut.query_data), expected, f'cycle {cycle}')
                address = (yield dut.query_address)
                self.assertEqual((yield dut.query_ready), bool((yield dut.update_write)) and address < 50)
                accepting = (yield dut.query_valid) and (yield dut.query_ready)
                expected = None
                if accepting:
                    expected = packets[address] if address < 36 else destinations[address - 36] if address < 48 else hosts[address - 48]
                    if address >= 48 and (yield dut.update_address) == dut.packet_storage_count + address - 48:
                        collisions.add(address - 48)
                    if address < 36:
                        key = (address // 6, address % 6)
                        if key in dut.physical_fields:
                            physical = dut.physical_fields.index(key)
                            if physical == (yield dut.update_address):
                                counter_collisions.add(physical)
                if (yield dut.reset):
                    packets[:] = [0] * 36
                    hosts[:] = [0, 0]
                    destinations[:] = defaults
                    expected = None
                else:
                    clear = (yield dut.clear_camera)
                    for camera in range(2):
                        if clear & (1 << camera):
                            camera_clear_phases.add((camera,
                                2 * (yield dut.update_address) + (yield dut.update_write)))
                    for stream, event in enumerate(dut.events):
                        for field, name in enumerate(COUNTER_FIELDS):
                            value = (yield getattr(event, name))
                            if name == 'bytes' and not (yield event.sent): value = 0
                            if name == 'reject' and stream % 3 != 2: value = 0
                            index = stream * 6 + field
                            packets[index] = 0 if clear & (1 << (stream // 3)) else (packets[index] + value) & MASK
                    write = (yield dut.config_write_address)
                    if (yield dut.config_write_ready) and (yield dut.config_write_valid):
                        destinations[write] = (yield dut.config_write_data) & (0xffff if write % 2 else MASK)
                    for index in range(12):
                        if clear & (1 << (index // 6)): destinations[index] = defaults[index]
                    host_clear, events = (yield dut.clear_host), (yield dut.host_event)
                    for host in range(2):
                        if host_clear & (1 << host):
                            clear_phases.add((host, 2 * (yield dut.update_address) + (yield dut.update_write)))
                        hosts[host] = min(MASK, (0 if host_clear & (1 << host) else hosts[host]) + ((events >> host) & 1))
                    for index, value in enumerate(dut.pending):
                        increment = 1
                        if index < dut.packet_storage_count:
                            stream, field = dut.physical_fields[index]
                            if field == 2:
                                increment = ((1056, 320, 208)[stream % 3] if batched_packets
                                             else 1056 if stream % 3 == 0 else 104)
                        maximum = dut.update_period * increment
                        maxima[index] = max(maxima[index], (yield value))
                        self.assertLessEqual((yield value), maximum)
                yield dut.reset.eq(step.get('reset', 0))
                yield dut.host_event.eq(step.get('faults', 0))
                yield dut.clear_host.eq(step.get('clear_hosts', 0))
                yield dut.clear_camera.eq(step.get('clear_camera', 0))
                query = step.get('query')
                yield dut.query_valid.eq(query is not None)
                yield dut.query_address.eq(query or 0)
                config = step.get('config')
                yield dut.config_write_valid.eq(config is not None)
                yield dut.config_write_address.eq(config[0] if config else 0)
                yield dut.config_write_data.eq(config[1] if config else 0)
                for stream, event in enumerate(dut.events):
                    active = step.get('packets', 0)
                    image_bytes = step.get('image_bytes', (1056, 1056))[stream // 3]
                    child_bytes = step.get('child_bytes', (320, 320) if batched_packets
                                           else (104, 104))[stream // 3]
                    pose_bytes = step.get('pose_bytes', (208, 208) if batched_packets
                                          else (120, 120))[stream // 3]
                    values = [active, active, (image_bytes, child_bytes, pose_bytes)[stream % 3],
                              active, active, active if stream % 3 == 2 else 0]
                    for field, value in zip(COUNTER_FIELDS, values):
                        yield getattr(event, field).eq(value)
                yield
            if all_clear_phases:
                self.assertEqual(clear_phases, {(host, phase) for host in range(2)
                                                for phase in range(dut.update_period)})
            if all_camera_clear_phases:
                self.assertEqual(camera_clear_phases, {(camera, phase) for camera in range(2)
                                                       for phase in range(dut.update_period)})
            if all_counter_collisions:
                self.assertEqual(counter_collisions, set(range(dut.packet_storage_count)))
            self.assertEqual(collisions, {0, 1})
        run_simulation(dut, bench())
        return maxima

    def test_saturation_before_during_and_after_ram_flush(self):
        steps = [{'faults': 3, 'packets': 1, 'query': 48 + ((cycle // 2) % 2)} for cycle in range(288)]
        steps += [{'faults': 3, 'clear_camera': 3, 'query': 48 + ((cycle // 2) % 2)} for cycle in range(96)]
        steps += [{'query': 48 + ((cycle // 2) % 2)} for cycle in range(48)]
        self.exercise(steps, initial_hosts=(MASK - 1, MASK - 70))

    def test_every_host_clear_phase_new_fault_precedence_and_camera_isolation(self):
        rng = random.Random(0x481a)
        steps = [{'faults': 3, 'packets': 1,
                  'query': 48 + ((cycle // 2) % 2) if cycle % 7 else rng.randrange(64),
                  'config': (rng.randrange(12), rng.getrandbits(32)),
                  'clear_hosts': 1 << (cycle // 2352) if cycle % 49 == 0 else 0,
                  'clear_camera': rng.randrange(4) if cycle % 41 == 0 else 0}
                 for cycle in range(4704)]
        steps += [{'reset': 1}, {'reset': 1}, {}]
        steps += [{'query': address} for address in range(50) for _ in range(2)]
        self.exercise(steps, all_clear_phases=True)

    def test_held_clear_returns_current_edge_fault_not_stale_saturated_count(self):
        steps = [{'faults': 3, 'query': 48 + ((cycle // 2) % 2)} for cycle in range(192)]
        steps += [{'faults': (cycle % 4), 'clear_hosts': 3, 'query': 48 + ((cycle // 2) % 2)} for cycle in range(192)]
        steps += [{'faults': 3, 'query': 48 + ((cycle // 2) % 2)} for cycle in range(192)]
        self.exercise(steps, initial_hosts=(MASK, MASK))

    def test_compact_image_maximum_rate_pending_and_all_counter_collisions(self):
        dut = SharedPacketCounters(packet_totals=True, with_destinations=True,
            fixed_packet_layout=True, with_host_counters=True, compact_image_packets=True)
        self.assertEqual(dut.query_count, 50)
        self.assertEqual((dut.memory.width, dut.memory.depth), (32, 38))
        self.assertEqual(sum(len(value) for value in dut.pending), 190)
        addresses = [stream * 6 + field for stream, field in dut.physical_fields] + [48, 49]
        steps = [{'faults': 3, 'packets': 1,
                  'query': addresses[(cycle // 2) % 26],
                  'config': ((cycle // 2) % 12, 0x87650000 + cycle)}
                 for cycle in range(1040)]
        steps += [{'query': address} for address in range(50) for _ in range(2)]
        maxima = self.exercise(steps, compact_image_packets=True,
                              all_counter_collisions=True)
        for index, (stream, field) in enumerate(dut.physical_fields):
            increment = (1056 if stream % 3 == 0 else 104) if field == 2 else 1
            self.assertEqual(maxima[index], 52 * increment)
        self.assertEqual(maxima[-2:], [52, 52])

    def test_compact_image_lengths_modulo_wrap_and_host_saturation(self):
        initial = []
        for stream in range(6):
            sent = MASK - stream
            # Image bytes have no relation to packet count in compact mode.
            byte_count = ((sent * 120) & MASK) if stream % 3 == 2 else MASK - 17 - stream
            initial.extend([sent, sent, byte_count, MASK - 1, MASK - 2,
                            MASK - 3 if stream % 3 == 2 else 0])
        lengths = (32, 34, 1054, 1056)
        steps = [{'faults': 3, 'packets': int(cycle % 5 != 0),
                  'image_bytes': (lengths[cycle % 4], lengths[(cycle + 1) % 4]),
                  'query': (cycle // 2) % 50,
                  'config': ((cycle // 2) % 12, 0x12340000 + cycle)}
                 for cycle in range(1300)]
        # Leave each variable byte row idle for several sweeps before reading
        # it again, verifying the RAM value after every pending total drains.
        steps += [{} for _ in range(156)]
        steps += [{'query': address} for address in range(50) for _ in range(2)]
        self.exercise(steps, initial_hosts=(MASK - 1, MASK - 70),
                      initial_packets=initial, compact_image_packets=True)

    def test_compact_image_all_clear_phases_and_contended_queries(self):
        rng = random.Random(0xc052)
        # 53 is coprime with the 52-clock sweep, so each clear visits every
        # updater read/write phase. Camera and host clears share those edges
        # while destination writes and maximal image byte events continue.
        steps = [{'faults': 3, 'packets': 1,
                  'query': 48 + ((cycle // 2) % 2) if cycle % 7 else rng.randrange(64),
                  'config': (rng.randrange(12), rng.getrandbits(32)),
                  'clear_hosts': 1 << (cycle // 2756) if cycle % 53 == 0 else 0,
                  'clear_camera': 1 << (cycle // 2756) if cycle % 53 == 0 else 0}
                 for cycle in range(5512)]
        steps += [{'query': address} for address in range(50) for _ in range(2)]
        steps += [{'reset': 1}, {'reset': 1}, {}]
        steps += [{'query': address} for address in range(50) for _ in range(2)]
        self.exercise(steps, compact_image_packets=True,
                      all_clear_phases=True, all_camera_clear_phases=True)

    def test_batched_maximum_byte_events_every_clock_and_all_counter_collisions(self):
        dut = SharedPacketCounters(packet_totals=True, with_destinations=True,
            fixed_packet_layout=True, with_host_counters=True,
            compact_image_packets=True, batched_packets=True)
        self.assertEqual(dut.query_count, 50)
        self.assertEqual((dut.memory.width, dut.memory.depth), (32, 40))
        self.assertEqual(sum(len(value) for value in dut.pending), 222)
        addresses = [stream * 6 + field for stream, field in dut.physical_fields] + [48, 49]
        steps = [{'faults': 3, 'packets': 1,
                  'query': addresses[(cycle // 2) % 28],
                  'config': ((cycle // 2) % 12, 0x87650000 + cycle)}
                 for cycle in range(1120)]
        steps += [{'query': address} for address in range(50) for _ in range(2)]
        maxima = self.exercise(steps, compact_image_packets=True, batched_packets=True,
                              all_counter_collisions=True)
        for index, (stream, field) in enumerate(dut.physical_fields):
            increment = (1056, 320, 208)[stream % 3] if field == 2 else 1
            self.assertEqual(maxima[index], 56 * increment)
        self.assertEqual(maxima[-2:], [56, 56])

    def test_batched_variable_lengths_modulo_wrap_and_host_saturation(self):
        initial = []
        for stream in range(6):
            sent = MASK - stream
            initial.extend([sent, sent, MASK - 17 - stream, MASK - 1, MASK - 2,
                            MASK - 3 if stream % 3 == 2 else 0])
        images = (34, 36, 1054, 1056)
        # One through four groups, each containing exactly nine tuples.
        children = (104, 176, 248, 320)
        poses = (120, 208)
        steps = [{'faults': 3, 'packets': int(cycle % 5 != 0),
                  'image_bytes': (images[cycle % 4], images[(cycle + 1) % 4]),
                  'child_bytes': (children[cycle % 4], children[(cycle + 1) % 4]),
                  'pose_bytes': (poses[cycle % 2], poses[(cycle + 1) % 2]),
                  'query': (cycle // 2) % 50,
                  'config': ((cycle // 2) % 12, 0x12340000 + cycle)}
                 for cycle in range(1400)]
        steps += [{} for _ in range(168)]
        steps += [{'query': address} for address in range(50) for _ in range(2)]
        self.exercise(steps, initial_hosts=(MASK - 1, MASK - 70),
                      initial_packets=initial, compact_image_packets=True, batched_packets=True)

    def test_batched_all_clear_phases_and_contended_queries(self):
        rng = random.Random(0xba756)
        # One clear each 57 clocks covers all 56 update phases for each host
        # and camera, while every stream sends maximum-size datagrams each edge.
        steps = [{'faults': 3, 'packets': 1,
                  'query': 48 + ((cycle // 2) % 2) if cycle % 7 else rng.randrange(64),
                  'config': (rng.randrange(12), rng.getrandbits(32)),
                  'clear_hosts': 1 << (cycle // 3192) if cycle % 57 == 0 else 0,
                  'clear_camera': 1 << (cycle // 3192) if cycle % 57 == 0 else 0}
                 for cycle in range(6384)]
        steps += [{'query': address} for address in range(50) for _ in range(2)]
        steps += [{'reset': 1}, {'reset': 1}, {}]
        steps += [{'query': address} for address in range(50) for _ in range(2)]
        self.exercise(steps, compact_image_packets=True, batched_packets=True,
                      all_clear_phases=True, all_camera_clear_phases=True)


if __name__ == "__main__":
    unittest.main()
