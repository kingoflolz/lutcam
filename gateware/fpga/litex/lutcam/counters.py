"""Shared RAM-backed packet diagnostics with bounded exact event accumulation."""

from functools import reduce
from operator import or_
from migen import Array, Cat, Case, Constant, If, Memory, Module, Mux, READ_FIRST, Record, Signal


COUNTER_FIELDS = ("committed", "sent", "bytes", "overflow", "protocol", "reject")
COUNTER_MAX_INCREMENT = (2, 1, 1440, 1, 1, 1)


class SharedPacketCounters(Module):
    """Six streams' six modulo-2**32 counters in shared RAM.

    ``events[stream]`` is sampled on each rising edge. ``committed`` accepts
    0..2; all flags accept 0..1. ``bytes`` accepts 0..1440 and is counted only
    when ``sent`` is asserted. This is an event interface, with no backpressure.

    ``clear_camera[0/1]`` clears streams 0..2 / 3..5 on that rising edge.
    Clear wins over that edge's events; events from the next edge onward are
    retained. Clear has no sweep or busy interval. Global synchronous reset
    likewise invalidates the RAM through resettable one-bit validity tags.

    Address = stream * 6 + index in COUNTER_FIELDS. A query is accepted on an
    edge with ``query_valid & query_ready``. ``query_response_valid`` and the
    exact 32-bit ``query_data`` appear during the following cycle. The snapshot
    includes every event BEFORE the acceptance edge, excluding events or clear
    on that edge. There is no response backpressure. Query acceptance is every
    other clock: the RAM read port serves the updater on one phase and a query
    on the other. Hold valid/address until ready; the maximum request-to-response
    latency is three clocks. Invalid addresses are not accepted. A caller can
    arbitrate software and packet sequence reads, giving packet requests priority.

    The updater reads then writes one counter every two clocks, never yielding
    to queries. Each counter is therefore flushed every 72 clocks. Pending
    registers need only ceil(log2(72 * maximum_increment + 1)) bits: 8 for
    committed, 17 for bytes, and 7 for each one-bit event. This bounds storage
    even if EVERY stream asserts EVERY maximum event on EVERY clock forever.
    Queries combine the RAM value with a snapshot of pending events. Pending
    is captured/cleared on the RAM read edge, allowing one shared pending mux.
    Same-edge query/write collisions forward the in-flight written sum before
    adding events since its capture, making semantics independent of the FPGA
    RAM's read-during-write mode.

    ``packet_totals=True`` specializes the board's six fixed packet streams.
    Only 26 values need storage: committed is derived from sent plus the live
    ``committed_levels[stream]`` (0..15); image/pose byte totals are sent times
    1056/120. Callers must maintain the queue conservation invariant and fixed
    datagram lengths; child byte totals still accumulate exact event lengths.
    Levels are sampled on the same query edge as sent, so simultaneous commit,
    transmit and clear remain coherent. The sweep is then 52 clocks, requiring
    6 pending bits for flags and 17 for the two variable byte counters.

    ``with_destinations=True`` adds twelve active destination words to this
    same RAM, without changing the updater period. Queries 36+2*stream and
    37+2*stream return IP and UDP port. Reset defaults are IP=0 and port=6000+
    stream. ``config_write_valid/ready/address/data`` writes a complete word
    (address 0..11); port writes retain only bits 15:0. Writes are accepted on
    the updater read phase, so they neither stall counters nor collide with
    accepted queries. Camera clear wins over a simultaneous config write.
    Partial software writes must read/merge/write through the caller; this
    module deliberately has no second configuration copy or byte-enable port.

    ``fixed_packet_layout=True`` additionally specializes the production child
    streams (at most nine 8-byte records plus the 32-byte header) and pose-only
    rejections. Non-pose rejection queries return zero; only pose rejection
    events are counted. Child byte increments must be at most 104 per clock.
    This removes four unused values, giving 22 counters, a 44-clock sweep and
    13-bit child byte pending totals (44 * 104 = 4576). Generic modes retain
    their 1440-byte contract and all six rejection counters.

    ``with_host_counters=True`` adds two saturating host fault counters queried
    at addresses 48/49. This requires the fixed packet layout and destination
    storage. ``host_event[host]`` increments once per clock; ``clear_host[host]``
    clears independently of camera reset. A new fault on the clear edge leaves
    a count of one. Host queries include every pending event and saturate at
    0xffffffff. Two sticky saturation bits allow the stored host values to
    wrap internally while retaining exact visible saturation. The 24-counter
    sweep takes 48 clocks; all flag pending totals still fit six bits and the
    child byte totals still fit 13 bits (48 * 104 = 4992).

    ``compact_image_packets=True`` retains actual image byte totals instead
    of deriving them from sent packet counts. This requires packet totals and
    accepts 32..1056 bytes per image datagram, including header-only packets.
    Pose byte totals remain derived. Two extra counter rows share the existing
    RAM; logical query addresses, including destinations and hosts, do not
    change. With fixed layout and host counters the sweep is 52 clocks: image
    pending totals need 16 bits (52 * 1056 = 54912), child totals need 13 bits
    (52 * 104 = 5408), and flag/host totals need six bits. These bounds assume
    maximum events every clock, independent of actual packet transmission rate.

    ``batched_packets=True`` counts actual bytes for every stream, including
    poses. It requires compact image packets and fixed layout. Each sent event
    denotes one datagram: image/child/pose lengths are at most 1056/320/208
    bytes, respectively. ``committed_levels`` must count pending datagrams,
    not records: auxiliary streams expose one once their batch is planned,
    until that batch is sent. Two pose byte rows give 28 counters with hosts,
    a 56-clock sweep, and unchanged logical addresses. Pending byte maxima
    are 59136/17920/11648, requiring 16/15/14 bits; flags and hosts need six
    bits. All rows still share the same RAM, now 40 words with destinations.


    ``serialized_datagrams=True`` specializes the batched production transport:
    all six streams complete packets through one 32-bit ready/valid serializer,
    and ``sent`` denotes its accepted final word, never an enqueue event. Every
    datagram has at least eight header words and no more than 1056/320/208 bytes
    for image/children/pose. Over a T-clock updater window, each stream can
    complete at most ceil(T/8) datagrams. Its first completion can contribute a
    full packet; later packets contribute at most four bytes per intervening
    clock, bounding its byte total by max_packet_bytes + 4*(T-1). For the
    56-clock production sweep this needs three sent bits and 11/10/9 byte bits.
    Fault/rejection events retain their unrestricted one-per-clock bounds.
    Query/config traffic never stalls the updater, and clears shorten windows.
    The default keeps the generic simultaneous maximum-event-per-clock API.

    ``serialized_event_descriptor=True`` replaces the production sent/byte
    pending accumulators with one datagram descriptor. It requires the complete
    production layout and the same single-serializer contract. The descriptor
    is drained through two priority RAM updates within at most five clocks;
    consecutive completions are at least eight clocks apart. Fault/host rows
    retain independent seven-bit pending counters and a conservative 80-clock
    service bound. Query acceptance, snapshot semantics, clear priority,
    destination writes and host saturation are unchanged. Generic configurations
    keep the original periodic updater unless this specialization is requested.
    """

    streams = 6
    counters_per_stream = len(COUNTER_FIELDS)
    counter_count = streams * counters_per_stream
    update_period = counter_count * 2

    def __init__(self, packet_totals=False, with_destinations=False, fixed_packet_layout=False,
                 with_host_counters=False, compact_image_packets=False,
                 batched_packets=False, serialized_datagrams=False, serialized_event_descriptor=False):
        if with_destinations and not packet_totals:
            raise ValueError("destination storage requires packet_totals=True")
        if fixed_packet_layout and not packet_totals:
            raise ValueError("fixed packet layout requires packet_totals=True")
        if with_host_counters and not (fixed_packet_layout and with_destinations):
            raise ValueError("host faults require fixed packet layout and destinations")
        if compact_image_packets and not packet_totals:
            raise ValueError("compact image packets require packet_totals=True")
        if batched_packets and not (compact_image_packets and fixed_packet_layout):
            raise ValueError("batched packets require compact images and fixed packet layout")
        if serialized_datagrams and not batched_packets:
            raise ValueError("serialized datagram bounds require batched packets")
        if serialized_event_descriptor and not (packet_totals and with_destinations and fixed_packet_layout and
                with_host_counters and compact_image_packets and batched_packets and serialized_datagrams):
            raise ValueError("serialized descriptor requires the complete production packet layout")
        self.batched_packets = batched_packets
        self.compact_image_packets = compact_image_packets
        self.with_host_counters = with_host_counters
        self.host_event = Signal(2)
        self.clear_host = Signal(2)
        self.fixed_packet_layout = fixed_packet_layout
        self.with_destinations = with_destinations
        self.query_count = self.counter_count + (12 if with_destinations else 0) + (2 if with_host_counters else 0)
        self.events = [Record([
            ("committed", 2), ("sent", 1), ("bytes", 11),
            ("overflow", 1), ("protocol", 1), ("reject", 1),
        ], name=f"events{stream}") for stream in range(self.streams)]
        self.clear_camera = Signal(2)
        # Clear capture statistics without discarding configured destinations.
        self.clear_statistics = Signal(2)
        self.query_valid = Signal()
        self.query_ready = Signal()
        self.query_address = Signal(max=self.query_count)
        self.query_response_valid = Signal()
        self.query_data = Signal(32)

        self.committed_levels = [Signal(4, name=f"committed_level{stream}")
                                 for stream in range(self.streams)]
        self.packet_totals = packet_totals
        self.physical_fields = [(stream, field)
            for stream in range(self.streams)
            for field in range(self.counters_per_stream)
            if not (packet_totals and (field == 0 or
                    (field == 2 and ((stream % 3 == 2 and not batched_packets) or
                                    (stream % 3 == 0 and not compact_image_packets)))))
            and not (fixed_packet_layout and field == 5 and stream % 3 != 2)]
        self.packet_storage_count = len(self.physical_fields)
        self.storage_count = self.packet_storage_count + (2 if with_host_counters else 0)
        self.update_period = 80 if serialized_event_descriptor else 2 * self.storage_count
        physical_index = {key: index for index, key in enumerate(self.physical_fields)}
        query_indices = [physical_index.get((stream, field), physical_index[(stream, 1)])
            for stream in range(self.streams) for field in range(self.counters_per_stream)]

        # Exposed for simulation/diagnostics, not additional interfaces.
        self.pending = pending = []
        self.initialized = initialized = [Signal(name=f"initialized{i}")
                                          for i in range(self.storage_count)]
        self.update_address = update_address = Signal(max=self.storage_count)
        self.update_write = update_write = Signal()
        increments = []
        for stream, field_index in self.physical_fields:
            field = COUNTER_FIELDS[field_index]
            maximum = COUNTER_MAX_INCREMENT[field_index]
            if field == "bytes":
                if batched_packets:
                    maximum = (1056, 320, 208)[stream % 3]
                elif compact_image_packets and stream % 3 == 0:
                    maximum = 1056
                elif fixed_packet_layout:
                    maximum = 104
            width = (self.update_period * maximum).bit_length()
            if serialized_datagrams and field == "bytes":
                # All six streams share one 32-bit output. Within a window of
                # T clocks, the first completed packet may contribute its full
                # length; later packets contribute at most four bytes/clock.
                width = (maximum + 4 * (self.update_period - 1)).bit_length()
            elif serialized_datagrams and field == "sent":
                # Every datagram contains the eight-word LUTC header, even if
                # its payload is empty. This bound deliberately ignores all
                # planning, protocol-header and inter-packet gaps.
                width = ((self.update_period + 7) // 8).bit_length()
            if serialized_event_descriptor:
                width = 11 if field == "bytes" else 1 if field == "sent" else 7
            pending.append(Signal(width, name=f"pending_{stream}_{field}"))
            increment = getattr(self.events[stream], field)
            if field == "bytes":
                increment = Mux(self.events[stream].sent, increment, 0)
            increments.append(increment)

        counter_clears = [(self.clear_camera | self.clear_statistics)[stream // 3]
                          for stream, _ in self.physical_fields]
        if with_host_counters:
            for host in range(2):
                pending.append(Signal(7 if serialized_event_descriptor else self.update_period.bit_length(), name=f"host{host}_pending"))
                increments.append(self.host_event[host])
                counter_clears.append(self.clear_host[host])

        if serialized_event_descriptor:
            descriptor_stream = Signal(max=6)
            descriptor_bytes = Signal(11)
            descriptor_sent_pending = Signal()
            descriptor_bytes_pending = Signal()
            self.descriptor_sent_pending = descriptor_sent_pending
            self.descriptor_bytes_pending = descriptor_bytes_pending
            input_stream = Signal(max=6)
            input_bytes = Signal(11)
            input_valid = Signal()
            self.comb += [
                input_valid.eq(Cat(*(event.sent for event in self.events)) != 0),
                input_stream.eq(reduce(or_, (Mux(event.sent, stream, 0) for stream, event in enumerate(self.events)))),
                input_bytes.eq(reduce(or_, (Mux(event.sent, event.bytes, 0) for event in self.events))),
            ]
            descriptor_clear = Signal()
            input_clear = Signal()
            self.comb += [
                descriptor_clear.eq(Array([(self.clear_camera | self.clear_statistics)[0], (self.clear_camera | self.clear_statistics)[1]])[descriptor_stream >= 3]),
                input_clear.eq(Array([(self.clear_camera | self.clear_statistics)[0], (self.clear_camera | self.clear_statistics)[1]])[input_stream >= 3]),
            ]
            fault_indices = [i for i, key in enumerate(self.physical_fields) if key[1] not in (1, 2)]
            fault_indices += list(range(self.packet_storage_count, self.storage_count))
            fault_cursor = Signal(max=len(fault_indices))
            held_update_address = Signal.like(update_address)
            selected_update_address = Signal.like(update_address)
            selected_update_is_fault = Signal()
            held_update_is_fault = Signal()
            self.comb += [
                selected_update_is_fault.eq(~descriptor_sent_pending & ~descriptor_bytes_pending),
                selected_update_address.eq(Mux(descriptor_sent_pending,
                    Array([physical_index[(stream, 1)] for stream in range(6)])[descriptor_stream],
                    Mux(descriptor_bytes_pending,
                        Array([physical_index[(stream, 2)] for stream in range(6)])[descriptor_stream],
                        Array(fault_indices)[fault_cursor]))),
                update_address.eq(Mux(update_write, held_update_address, selected_update_address)),
            ]
            self.sync += [
                If(~update_write,
                    held_update_address.eq(selected_update_address),
                    held_update_is_fault.eq(selected_update_is_fault)),
                If(update_write & held_update_is_fault,
                    fault_cursor.eq(Mux(fault_cursor == len(fault_indices) - 1, 0, fault_cursor + 1))),
                If(descriptor_clear,
                    descriptor_sent_pending.eq(0), descriptor_bytes_pending.eq(0)
                ).Elif(~update_write,
                    If(descriptor_sent_pending, descriptor_sent_pending.eq(0))
                    .Elif(descriptor_bytes_pending, descriptor_bytes_pending.eq(0))),
                If(input_valid & ~input_clear,
                    descriptor_stream.eq(input_stream), descriptor_bytes.eq(input_bytes),
                    descriptor_sent_pending.eq(1), descriptor_bytes_pending.eq(1)),
            ]

        memory = Memory(32, self.storage_count + (12 if with_destinations else 0),
                        name="packet_counter_values")
        self.specials += memory
        self.memory = memory
        # A single physical read port avoids replicating the RAM on Titanium.
        # The writer port's unused read result is removed by synthesis.
        update = memory.get_port(write_capable=True, mode=READ_FIRST)
        query = memory.get_port(has_re=True, mode=READ_FIRST)
        self.specials += update, query

        update_sum = Signal(32)
        update_wide_sum = Signal(33)
        query_is_host = Signal()
        response_is_host = Signal()
        response_saturated = Signal()
        response_wide_value = Signal(33)
        update_pending = Signal(max(len(value) for value in pending))
        selected_pending = Signal.like(update_pending)
        pending_address = Signal.like(update_address)
        query_address = Signal(max=memory.depth)
        accept = Signal()
        collision = Signal()
        response_forward = Signal()
        response_forward_data = Signal(32)
        response_initialized = Signal()
        # Read phase prepares an updater snapshot; write phase prepares a
        # query snapshot. Their consumers occupy opposite clock phases.
        response_pending = update_pending
        response_value = Signal(32)
        # Batched committed queries are sent + live queued datagrams. Fold the
        # four-bit level into pending before the wide RAM-value add, avoiding
        # a second 32-bit carry chain. Host/destination queries add no level.
        response_level_correction = Signal(4)
        response_adjustment = Signal(len(response_pending) + 1)
        self.comb += response_adjustment.eq(response_pending + Mux(update_write, 0, response_level_correction))
        query_zero = Signal()
        response_zero = Signal()
        query_result = Signal(32)
        self.comb += self.query_data.eq(Mux(response_zero, 0,
            Mux(response_is_host & (response_saturated | response_wide_value[32]), 0xffffffff, query_result)))
        if fixed_packet_layout:
            zeros = [Constant(field == 5 and stream % 3 != 2)
                     for stream in range(self.streams) for field in range(self.counters_per_stream)]
            self.comb += query_zero.eq((self.query_address < self.counter_count)
                                      & Array(zeros)[self.query_address])
        else:
            self.comb += query_zero.eq(0)
        response_default = Signal(32)
        query_default = Signal(32)
        query_initialized = Signal()
        query_is_destination = Signal()
        write_address = Signal(max=memory.depth)
        write_value = Signal(32)
        write_enable = Signal()
        self.comb += [
            update.adr.eq(write_address),
            update.we.eq(write_enable),
            pending_address.eq(Mux(update_write, query_address, update_address)),
            selected_pending.eq(
                Mux((pending_address == Array([physical_index[(stream, 1)] for stream in range(6)])[descriptor_stream]) & descriptor_sent_pending, 1, 0)
                | Mux((pending_address == Array([physical_index[(stream, 2)] for stream in range(6)])[descriptor_stream]) & descriptor_bytes_pending, descriptor_bytes, 0)
                | Array([v if i in fault_indices else Constant(0) for i, v in enumerate(pending)])[pending_address] if serialized_event_descriptor else Array(pending)[pending_address]),
            # RAM update/forwarding consumes this adder only in write phase;
            # query responses consume it only in the following read phase.
            update_wide_sum.eq(response_wide_value),
            update_sum.eq(update_wide_sum[:32]),
            update.dat_w.eq(write_value),
            self.query_ready.eq(update_write & (self.query_address < self.query_count)),
            accept.eq(self.query_valid & self.query_ready),
            query.adr.eq(Mux(update_write,
                Mux(self.query_address < self.query_count, query_address, 0),
                update_address)),
            query.re.eq(~update_write | accept),
            collision.eq(update_write & (update_address == query_address)),
            response_wide_value.eq(
                Mux(~update_write & response_forward, response_forward_data,
                    Mux(Mux(update_write, Array(initialized)[update_address], response_initialized),
                        query.dat_r, Mux(update_write, 0, response_default)))
                + (response_adjustment if batched_packets else response_pending)),
            response_value.eq(response_wide_value[:32]),
        ]
        self.sync += [
            update_write.eq(~update_write),
            If(~update_write,
                update_pending.eq(Mux(
                    Array(counter_clears)[update_address],
                    0, selected_pending)),
            ),
            *([] if serialized_event_descriptor else [If(update_write,
                If(update_address == self.storage_count - 1,
                    update_address.eq(0),
                ).Else(update_address.eq(update_address + 1)),
            )]),
            self.query_response_valid.eq(accept),
            If(accept,
                response_forward.eq(collision), response_zero.eq(query_zero),
                response_is_host.eq(query_is_host),
                response_forward_data.eq(update_sum),
                response_initialized.eq(query_initialized),
                response_default.eq(query_default),
                response_pending.eq(Mux(query_is_destination, 0, selected_pending)),
            ),
        ]
        for index, (value, increment, valid) in enumerate(
                zip(pending, increments, initialized)):
            is_host = index >= self.packet_storage_count
            if serialized_event_descriptor and index < self.packet_storage_count and self.physical_fields[index][1] in (1, 2):
                stream, field = self.physical_fields[index]
                self.comb += value.eq(Mux(descriptor_stream == stream,
                    descriptor_sent_pending if field == 1 else Mux(descriptor_bytes_pending, descriptor_bytes, 0), 0))
                self.sync += If(counter_clears[index], valid.eq(0)).Elif(
                    update_write & (update_address == index), valid.eq(1))
                continue
            self.sync += If(counter_clears[index],
                value.eq(increment if is_host else 0), valid.eq(0),
            ).Else(
                If(~update_write & (update_address == index),
                    # Capture the old pending total alongside the RAM read.
                    # This edge's event starts the next accumulation window.
                    value.eq(increment),
                ).Else(value.eq(value + increment)),
                If(update_write & (update_address == index), valid.eq(1)),
            )

        if with_host_counters:
            # Keep saturation outside the RAM so the shared update adder can
            # wrap normally for every row. Queries clamp using this flag or a
            # carry from not-yet-flushed events. A collision must also retain
            # the overflow from the update whose wrapped value it forwards.
            self.host_saturated = host_saturated = [Signal(name=f"host{host}_saturated") for host in range(2)]
            for host, saturated in enumerate(host_saturated):
                self.sync += If(self.clear_host[host], saturated.eq(0)).Elif(
                    update_write & (update_address == self.packet_storage_count + host) & update_wide_sum[32],
                    saturated.eq(1))
            self.sync += If(accept,
                response_saturated.eq(query_is_host &
                    (Array(host_saturated)[self.query_address[0]] | (collision & update_wide_sum[32]))))
            self.comb += [
                query_is_host.eq(self.query_address >= 48),
            ]
        else:
            self.comb += [query_is_host.eq(0), response_saturated.eq(0)]
        counter_query_address = Array([Constant(index, len(query_address))
                                       for index in query_indices])[self.query_address]
        if with_destinations:
            self.config_write_valid = Signal()
            self.config_write_ready = Signal()
            self.config_write_address = Signal(max=12)
            self.config_write_data = Signal(32)
            self.destination_valid = destination_valid = [Signal(name=f"destination_valid{i}")
                                                         for i in range(12)]
            destination_index = Signal(max=12)
            config_accept = Signal()
            defaults = [0 if field == 0 else 6000 + stream
                        for stream in range(self.streams) for field in range(2)]
            self.comb += [
                query_is_destination.eq((self.query_address >= self.counter_count)
                                        & (self.query_address < 48)),
                destination_index.eq(self.query_address - self.counter_count),
                query_address.eq(Mux(query_is_host,
                    self.packet_storage_count + self.query_address[0],
                    Mux(query_is_destination, self.storage_count + destination_index,
                        counter_query_address))),
                query_initialized.eq(Mux(query_is_destination,
                    Array(destination_valid)[destination_index], Array(initialized)[query_address])),
                query_default.eq(Mux(query_is_destination,
                    Array([Constant(value, 32) for value in defaults])[destination_index], 0)),
                self.config_write_ready.eq(~update_write & (self.config_write_address < 12)),
                config_accept.eq(self.config_write_valid & self.config_write_ready),
                write_enable.eq(update_write | config_accept),
                write_address.eq(Mux(update_write, update_address,
                                     self.storage_count + self.config_write_address)),
                write_value.eq(Mux(update_write, update_sum,
                    Mux(self.config_write_address[0], self.config_write_data[:16],
                        self.config_write_data))),
            ]
            for index, valid in enumerate(destination_valid):
                self.sync += If(self.clear_camera[index // 6], valid.eq(0)).Elif(
                    config_accept & (self.config_write_address == index), valid.eq(1))
        else:
            self.comb += [
                query_address.eq(counter_query_address),
                query_default.eq(0), query_is_destination.eq(0),
                query_initialized.eq(Array(initialized)[query_address]),
                write_enable.eq(update_write), write_address.eq(update_address),
                write_value.eq(update_sum),
            ]

        if packet_totals:
            mode = Signal(1 if batched_packets else 2)
            response_mode = Signal.like(mode)
            level = Signal(4)
            response_level = Signal.like(level)
            # The queue invariant is committed = sent + committed_level. Legacy
            # image datagrams are 1056 bytes; accepted-pose datagrams are 120.
            # Compact images and batched auxiliary datagrams use their actual
            # accumulated byte count.
            # Multiplication remains correct across the modulo-2**32 wrap.
            modes = [1 if field == 0 else
                     (3 if stream % 3 == 2 and not batched_packets else
                      2 if stream % 3 == 0 and not compact_image_packets else 0)
                     if field == 2 else 0
                     for stream in range(self.streams)
                     for field in range(self.counters_per_stream)]
            derive_a, derive_b = Signal(32), Signal(32)
            derive_c = Signal()
            derive_cases = {
                0: [derive_a.eq(response_value), derive_b.eq(0), derive_c.eq(0)],
                1: [derive_a.eq(response_value), derive_b.eq(response_level), derive_c.eq(0)],
            }
            if not batched_packets:
                derive_cases[3] = [derive_a.eq(response_value << 7),
                                  derive_b.eq(~(response_value << 3)), derive_c.eq(1)]
            if not compact_image_packets:
                derive_cases[2] = [derive_a.eq(response_value << 10),
                                  derive_b.eq(response_value << 5), derive_c.eq(0)]
            self.comb += [
                mode.eq(Mux(query_is_destination | query_is_host, 0,
                    Array([Constant(value, len(mode)) for value in modes])[self.query_address])),
                level.eq(Array([self.committed_levels[stream] if field == 0 else Constant(0, 4)
                                for stream in range(self.streams)
                                for field in range(self.counters_per_stream)])[self.query_address]),
                Case(response_mode, derive_cases),
                # (2*a + 1 + 2*b + carry) >> 1 is a+b+carry. Encoding
                # carry as a low bit infers one adder without an incrementer;
                # taking bits 1:33 preserves the modulo-2**32 result.
                query_result.eq(response_value if batched_packets else
                    (Cat(Constant(1, 1), derive_a) + Cat(derive_c, derive_b))[1:33]),
                response_level_correction.eq(Mux(response_mode, response_level, 0) if batched_packets else 0),
            ]
            self.sync += If(accept, response_mode.eq(mode), response_level.eq(level))
        else:
            self.comb += query_result.eq(response_value)
