"""Atomic camera packets and a packet-boundary LiteEth UDP arbiter.

All logic is in ``sys``. The source is the native 32-bit LiteEth UDP user
interface, whose first wire byte occupies bits 7:0. Application integers are
big endian. There is exactly one payload memory per stream: an uncommitted
write slot becomes visible only after the complete fragment/group/pose arrives.
Compact output sends only an image's meaningful prefix: the producer's known
zero padding is checked and discarded. The fixed-size test configuration
regenerates the zero tail from one descriptor.

The common 32-byte header consists of eight big-endian u32s: ``LUTC``,
``reserved_zero:8/camera:8/type:8/header_bytes:8``, frame ID, stream sequence,
fragment byte offset, payload byte length, flags, reserved.
Types are image=1, children=2, pose=3; flag bit 0 marks an image's final
fragment in nonbatched test configurations. A compact nonbatched FINAL packet may have an empty payload when the
semantic stream ends exactly at a fragment boundary. Image words,
zero-extended 64-bit child records, and native pose
words follow in big-endian order. Rejected solver candidates have no packet.
Production batched output omits FINAL and empty image packets, and concatenates up to four
fixed nine-tuple child groups or two poses from the same camera and frame.

Configure destination while capture is disabled, then wait for ``fifo_level``
to reach zero before enabling capture. Destination changes take effect only
when the stream is completely drained; queued packets retain their original
destination. Faults stop new packets, preserving already committed packets.
Clear faults only after capture has stopped and the input is at a clean frame
or group boundary. Counter/sequence arithmetic wraps modulo 2**32.
"""

from types import SimpleNamespace

from migen import Array, Cat, ClockSignal, Constant, If, Instance, Memory, Module, Mux, READ_FIRST, ResetInserter, ResetSignal, Signal
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import AutoCSR, CSR, CSRStatus, CSRStorage
from liteeth.common import eth_udp_user_description


HEADER_BYTES = 32
MAX_UDP_PAYLOAD = 1440
IMAGE_FRAGMENT_WORDS = 512
MAGIC = 0x4C555443


def _be32(value):
    """Convert a numeric big-endian word to LiteEth's little-lane layout."""
    if isinstance(value, int):
        value = Constant(value, 32)
    return Cat(value[24:32], value[16:24], value[8:16], value[:8])


def _power_of_two(value):
    return value > 0 and value & (value - 1) == 0



def _store_metadata(owner, target, source, enable, use_dsp):
    """Keep exact metadata capture edges, optionally using one native DSP.

    Unsigned A19*1 and C18 aligned left occupy disjoint parts of the W
    register. ResetInserter handles generic storage; the native instance also
    receives the owning queue/parser's explicit camera reset.
    """
    if not use_dsp:
        owner.sync += If(enable, target.eq(source))
        return
    result = Signal(48)
    padded = Signal(37)
    owner.comb += padded.eq(source)
    owner.specials += Instance("EFX_DSP48",
        p_MODE="NORMAL", p_SIGNED=0, p_M_SEL="P", p_N_SEL="C", p_C_EXT="ALIGN_LEFT",
        p_W_SEL="X", p_W_REG=1, p_RST_SYNC=1,
        i_A=padded[:19], i_B=Constant(1, 18), i_C=padded[19:37],
        i_CASCIN=Constant(0, 48), i_OP=Constant(0, 2), i_SHIFT_ENA=0,
        i_CLK=ClockSignal(), i_CE=enable,
        i_RST=ResetSignal() | owner._storage_reset, o_O=result)
    owner.comb += target.eq(Cat(result[:19], result[30:48])[:len(target)])


def _initialize_parser_interface(owner, slot_words, width):
    """Only the capture writer interface; committed queues remain external."""
    owner._storage_reset = Signal()
    owner._writer_busy = Signal()
    owner._overflow = Signal()
    owner._protocol_error = Signal()
    owner._enabled = Signal()
    owner._available = Signal()
    owner._commit = Signal()
    owner._frame = Signal(32)
    owner._offset = Signal(32)
    owner._length = Signal(11)
    owner._flags = Signal(32)
    owner._run_length = Signal(32)
    owner._stored_length = Signal(11)
    owner._write_ready = Signal(reset=1)
    owner._write_enable = Signal()
    owner._write_index = Signal(max=max(2, slot_words))
    owner._write_data = Signal(width)
    owner.reject_event = Signal()


class _PacketSlots(Module, AutoCSR):
    """Single-writer committed-slot FIFO with a synchronous payload RAM.

    32-bit payload words are already in wire-lane order; native 33-bit child
    records are expanded into two UDP words while reading. Header words use
    small descriptors, so memories do not duplicate space for headers. With
    ``zero_fragment_bytes``, a descriptor can cover many datagrams: its stored
    prefix is read once, and the remaining payload is synthesized as zeros.
    """

    def __init__(self, camera_id, stream_type, slots, slot_words, width=32,
                 zero_fragment_bytes=None, max_frame_bytes=None, max_payload_bytes=None,
                 shared=False, external_destination=False, compact_padding=False, batched_packets=False):
        self._storage_reset = Signal()
        if shared:
            _initialize_shared_queue(self, camera_id, stream_type, slots, slot_words,
                                     width, zero_fragment_bytes, max_frame_bytes, external_destination,
                                     compact_padding, batched_packets)
            return
        if not _power_of_two(slots) or slots < 2:
            raise ValueError("slots must be a power of two and at least two")
        if slot_words < 1 or width not in (32, 33):
            raise ValueError("slot_words must be positive; storage width is 32/33")
        if not 0 <= camera_id <= 255:
            raise ValueError("camera ID must fit in one byte")
        # A child record occupies 33 RAM bits and expands to eight wire bytes.
        # The previous 64-bit representation unnecessarily allocated RAM for
        # the 31 zero-extension bits. All other payloads use four-byte words.
        element_bytes = 8 if width == 33 else 4
        payload_bytes = max_payload_bytes or slot_words * element_bytes
        payload_bits = payload_bytes.bit_length()
        frame_bits = max_frame_bytes.bit_length() if max_frame_bytes is not None else 32
        # A fixed frame split into equal power-of-two fragments needs only
        # a fragment number and a virtual-tail marker in each descriptor.
        # Retain the byte-granular representation for other configurations.
        compact_image = (zero_fragment_bytes is not None
                         and max_frame_bytes is not None
                         and _power_of_two(zero_fragment_bytes)
                         and max_frame_bytes % zero_fragment_bytes == 0)
        fragment_count = max_frame_bytes // zero_fragment_bytes if compact_image else 0
        fragment_shift = zero_fragment_bytes.bit_length() - 1 if compact_image else 0
        offset_bits = max(1, (fragment_count - 1).bit_length()) if compact_image else frame_bits

        port = 6000 + camera_id * 3 + stream_type - 1
        self.enable = CSRStorage(1, name="enable")
        self.destination_ip = CSRStorage(32, name="destination_ip")
        self.destination_port = CSRStorage(16, reset=port, name="destination_port")
        self.active_destination_ip = CSRStatus(32, name="active_destination_ip")
        self.active_destination_port = CSRStatus(16, name="active_destination_port")
        self.fifo_level = CSRStatus(32, name="fifo_level")
        self.high_water = CSRStatus(32, name="high_water")
        self.faults = CSRStatus(2, name="faults")
        self.clear = CSR(name="clear")
        self.packets_committed = CSRStatus(32, name="packets_committed")
        self.packets_sent = CSRStatus(32, name="packets_sent")
        self.bytes_sent = CSRStatus(32, name="bytes_sent")
        self.overflows = CSRStatus(32, name="overflows")
        self.protocol_errors = CSRStatus(32, name="protocol_errors")
        self.capture_enable = Signal(reset=1)
        self.fault_sticky = Signal()
        self.source = source = stream.Endpoint(eth_udp_user_description(32))

        self._writer_busy = Signal()
        self._overflow = Signal()
        self._protocol_error = Signal()
        self._enabled = Signal()
        self._available = Signal()
        self._commit = Signal()
        self._frame = Signal(32)
        self._offset = Signal(frame_bits)
        self._length = Signal(payload_bits)
        self._flags = Signal(32)
        self._run_length = Signal(frame_bits)
        self._stored_length = Signal(payload_bits)
        self._write_enable = Signal()
        self._write_index = Signal(max=max(2, slot_words))
        self._write_data = Signal(width)

        level = Signal(max=slots + 1)
        # Pose has a two-word classification prefix outside the payload RAM;
        # it can hold one pending native packet while all slots are committed.
        used = Signal(max=slots + 2)
        write_slot = Signal(max=slots)
        read_slot = Signal(max=slots)
        read_index = Signal(max=max(2, slot_words))
        read_half = Signal()
        header_index = Signal(3)
        reading = Signal()
        in_payload = Signal()
        payload_index = Signal(payload_bits)
        release = Signal()
        dequeue = Signal()
        continue_run = Signal()
        accept = Signal()
        sequence = Signal(32)
        active_ip = Signal(32)
        active_port = Signal(16, reset=port)

        # Unlike payload RAMs these tiny descriptor arrays map to registers.
        frames = Array(Signal(32, name=f"frame_{i}") for i in range(slots))
        offsets = Array(Signal(offset_bits, name=f"offset_{i}") for i in range(slots))
        lengths = Array(Constant(zero_fragment_bytes, payload_bits) if compact_image
                        else Signal(payload_bits, name=f"length_{i}") for i in range(slots))
        flags = Array(Signal(32, name=f"flags_{i}") for i in range(slots))
        if zero_fragment_bytes is not None:
            run_lengths = Array(Signal(1 if compact_image else frame_bits,
                                      name=f"run_length_{i}") for i in range(slots))
            stored_lengths = Array(Signal(payload_bits, name=f"stored_length_{i}") for i in range(slots))
            if compact_image:
                self.comb += continue_run.eq(run_lengths[read_slot] & (offsets[read_slot] != fragment_count - 1))
            else:
                self.comb += continue_run.eq(run_lengths[read_slot] > lengths[read_slot])
        else:
            self.comb += continue_run.eq(0)

        memory = Memory(width, slots * slot_words, name="packet_payload")
        write = memory.get_port(write_capable=True)
        read = memory.get_port()
        self.specials += memory, write, read
        self.comb += [
            write.adr.eq(write_slot * slot_words + self._write_index),
            write.dat_w.eq(self._write_data),
            write.we.eq(self._write_enable),
            self._available.eq(level < slots),
            used.eq(level + self._writer_busy),
            self.fifo_level.status.eq(used),
            self.fault_sticky.eq(self.faults.status != 0),
            self._enabled.eq(self.enable.storage & self.capture_enable & ~self.fault_sticky),
            self.active_destination_ip.status.eq(active_ip),
            self.active_destination_port.status.eq(active_port),
            accept.eq(source.valid & source.ready),
            release.eq(accept & source.last),
            dequeue.eq(release & ~continue_run),
        ]
        self.sync += [
            If((level == 0) & ~self._writer_busy,
                active_ip.eq(self.destination_ip.storage),
                active_port.eq(self.destination_port.storage),
            ),
            If(used > self.high_water.status, self.high_water.status.eq(used)),
            If(self.clear.re, self.faults.status.eq(0)),
            If(self._overflow, self.faults.status[0].eq(1)),
            If(self._protocol_error, self.faults.status[1].eq(1)),
            If(self._overflow, self.overflows.status.eq(self.overflows.status + 1)),
            If(self._protocol_error, self.protocol_errors.status.eq(self.protocol_errors.status + 1)),
            If(self._commit & ~dequeue, level.eq(level + 1)),
            If(dequeue & ~self._commit, level.eq(level - 1)),
            If(self._commit,
                frames[write_slot].eq(self._frame),
                offsets[write_slot].eq(self._offset >> fragment_shift),
                *([] if compact_image else [lengths[write_slot].eq(self._length),
                                           flags[write_slot].eq(self._flags)]),
                write_slot.eq(write_slot + 1),
            ),
            If(self._commit | (release & continue_run),
                self.packets_committed.status.eq(self.packets_committed.status + self._commit + (release & continue_run))),
            If(dequeue, read_slot.eq(read_slot + 1)),
            If(release,
                sequence.eq(sequence + 1),
                self.packets_sent.status.eq(self.packets_sent.status + 1),
                self.bytes_sent.status.eq(self.bytes_sent.status + source.length),
            ),
        ]
        if zero_fragment_bytes is not None:
            remaining = run_lengths[read_slot] - lengths[read_slot]
            self.sync += [
                If(self._commit,
                    run_lengths[write_slot].eq((self._run_length != 0) if compact_image else self._run_length),
                    stored_lengths[write_slot].eq(self._stored_length),
                ),
                If(release & continue_run,
                    stored_lengths[read_slot].eq(0),
                    offsets[read_slot].eq(offsets[read_slot] + (1 if compact_image else lengths[read_slot])),
                    *([] if compact_image else [
                        run_lengths[read_slot].eq(remaining),
                        lengths[read_slot].eq(Mux(remaining > zero_fragment_bytes, zero_fragment_bytes, remaining)),
                        flags[read_slot].eq(remaining <= zero_fragment_bytes)]),
                ),
            ]

        header_offset = Cat(Constant(0, fragment_shift), offsets[read_slot]) if compact_image else offsets[read_slot]
        header_flags = (Cat(offsets[read_slot] == fragment_count - 1, Constant(0, 31))
                        if compact_image else flags[read_slot])
        header = Array([
            _be32(MAGIC),
            _be32(camera_id << 16 | stream_type << 8 | HEADER_BYTES),
            _be32(frames[read_slot]),
            _be32(sequence),
            _be32(Cat(header_offset, Constant(0, 32 - len(header_offset)))) if len(header_offset) < 32 else _be32(header_offset),
            _be32(Cat(lengths[read_slot], Constant(0, 32 - payload_bits))),
            _be32(header_flags),
            Constant(0, 32),
        ])
        payload_data = read.dat_r if width == 32 else Mux(read_half,
            _be32(read.dat_r[:32]), _be32(Cat(read.dat_r[32], Constant(0, 31))))
        if zero_fragment_bytes is not None:
            payload_data = Mux(payload_index >= stored_lengths[read_slot], 0, payload_data)
        last = Signal()
        advance_read = accept & in_payload
        if width == 33:
            advance_read = advance_read & read_half
        self.comb += [
            # Prefetch first word throughout header; on an accepted payload
            # word prefetch its successor. Stalled payload words remain stable.
            read.adr.eq(read_slot * slot_words + Mux(in_payload,
                Mux(advance_read, Mux(read_index == slot_words - 1, 0, read_index + 1), read_index), 0)),
            source.valid.eq(reading),
            source.first.eq(reading & ~in_payload & (header_index == 0)),
            last.eq(in_payload & (payload_index + 4 >= lengths[read_slot])),
            source.last.eq(reading & last),
            source.data.eq(Mux(in_payload, payload_data, header[header_index])),
            source.last_be.eq(Mux(last, 8, 0)),
            source.error.eq(0),
            source.src_port.eq(port),
            source.dst_port.eq(active_port),
            source.ip_address.eq(active_ip),
            source.length.eq(HEADER_BYTES + lengths[read_slot]),
        ]
        self.sync += [
            If(~reading,
                If(level != 0, reading.eq(1)),
                header_index.eq(0),
                in_payload.eq(0),
                read_index.eq(0),
                read_half.eq(0),
                payload_index.eq(0),
            ).Elif(accept,
                If(~in_payload,
                    If(header_index == 7,
                        in_payload.eq(1),
                    ).Else(header_index.eq(header_index + 1)),
                ).Else(
                    payload_index.eq(payload_index + 4),
                    If(width == 33,
                        read_half.eq(~read_half),
                        If(read_half, read_index.eq(Mux(read_index == slot_words - 1, 0, read_index + 1))),
                    ).Else(read_index.eq(Mux(read_index == slot_words - 1, 0, read_index + 1))),
                ),
                If(source.last, reading.eq(0)),
            ),
        ]


class ImagePacketizer(_PacketSlots):
    """Capture fixed-size LWC frames, in complete even-word fragments.

    ``frame_words`` and ``fragment_words`` count 16-bit LWC words. ``sof``
    accompanies the first valid word. ``frame_id`` is captured at SOF and held
    across every fragment. There is deliberately no input backpressure.

    ``padding`` marks known pure-zero encoder padding, never a partial last
    semantic word. Once asserted, all remaining words in that frame must be
    zero padding. A single descriptor includes the already-stored semantic
    prefix in its first fragment. The fixed-size test reader regenerates the full tail;
    shared compact mode sends that prefix once with FINAL, omitting all pure
    padding words. An odd number of semantic 16-bit words stays byte-exact.

    The first padding marker is a producer promise that defines the complete
    remaining frame. It commits the descriptor immediately, so transmission
    overlaps the producer's physical padding words. Those words are still
    counted and checked through the fixed frame boundary; a later violation
    raises a protocol fault but cannot retract the already published frame.

    ``packed_slots=True`` allocates exactly ``fragment_words // 2`` RAM words
    per slot. Set it false only to compare synthesis against power-of-two
    address strides; it does not change packet size or logical queue depth.
    """

    def __init__(self, camera_id, slots=8, frame_words=64000, fragment_words=IMAGE_FRAGMENT_WORDS,
                 packed_slots=True, _shared=False, _external_destination=False, _compact_padding=False,
                 _batched_packets=False, use_dsp_metadata=False, _held_frame_id=False):
        if frame_words < 2 or frame_words % 2:
            raise ValueError("frame_words must be positive and even")
        if fragment_words < 2 or fragment_words % 2 or fragment_words * 2 > MAX_UDP_PAYLOAD - HEADER_BYTES:
            raise ValueError("fragment_words must be even and fit the UDP payload")
        slot_words = fragment_words // 2
        fixed_fragments = frame_words % fragment_words == 0 and _power_of_two(fragment_words * 2)
        if not packed_slots:
            slot_words = 1 << ((slot_words - 1).bit_length())
        super().__init__(camera_id, 1, slots, slot_words, zero_fragment_bytes=fragment_words * 2,
                         max_frame_bytes=frame_words * 2, max_payload_bytes=fragment_words * 2,
                         shared=_shared, external_destination=_external_destination,
                         compact_padding=_compact_padding, batched_packets=_batched_packets)
        self.sink = sink = stream.Endpoint([("data", 16), ("sof", 1), ("padding", 1), ("frame_id", 32)])

        active = Signal()
        packet_active = Signal()
        half = Signal()
        first_word = Signal(16)
        zero_tail = Signal()
        frame_index = Signal(max=frame_words)
        fragment_index = Signal(max=fragment_words)
        frame_id = Signal(32)
        offset = Signal((frame_words * 2).bit_length())
        start_frame = Signal()
        take = Signal()
        frame_last = Signal()
        packet_last = Signal()
        can_start = Signal()
        padding_start = Signal()
        padding_mode = Signal()
        tail_length = Signal((frame_words * 2).bit_length())
        stored_length = Signal((fragment_words * 2).bit_length())
        invalid_padding = Signal()
        empty_padding = Signal()
        self.comb += [
            empty_padding.eq(sink.padding & (sink.sof | (fragment_index == 0))),
            sink.ready.eq(1),
            can_start.eq(self._enabled & (self._available | (empty_padding if _batched_packets else 0))),
            start_frame.eq(sink.valid & sink.sof & can_start & ~active),
            take.eq(sink.valid & (start_frame | (active & ~sink.sof &
                (zero_tail | packet_active | can_start)))),
            frame_last.eq(~start_frame & (frame_index == frame_words - 1)),
            padding_start.eq(take & sink.padding & (start_frame | ~zero_tail)),
            padding_mode.eq(Mux(start_frame, sink.padding, zero_tail | padding_start)),
            invalid_padding.eq(take & ((padding_mode & (sink.data != 0)) |
                (~start_frame & zero_tail & ~sink.padding))),
            packet_last.eq(~start_frame & (frame_last | (~padding_mode & (fragment_index == fragment_words - 1)))),
            tail_length.eq(frame_words * 2 - Mux(start_frame, 0, offset)),
            stored_length.eq(Mux(start_frame, 0,
                Mux(padding_start, fragment_index * 2 + (0 if _compact_padding else Mux(half, 2, 0)),
                    (fragment_index + 1) * 2))),
            self._writer_busy.eq(packet_active),
            self._write_enable.eq(take & ~start_frame & half & ~zero_tail & ~invalid_padding),
            self._write_index.eq(fragment_index >> 1),
            self._write_data.eq(Cat(first_word[8:16], first_word[:8], sink.data[8:16], sink.data[:8])),
            self._commit.eq(take & ((padding_start & (~empty_padding if _batched_packets else 1)) |
                                   (packet_last & ~padding_mode)) & ~invalid_padding),
            self._frame.eq(Mux(start_frame, sink.frame_id, frame_id)),
            self._offset.eq(Mux(start_frame, 0, offset)),
            self._length.eq(Mux(padding_mode,
                Mux(tail_length > fragment_words * 2, fragment_words * 2, tail_length), (fragment_index + 1) * 2)),
            self._run_length.eq(Mux(padding_mode, tail_length, 0)),
            self._stored_length.eq(stored_length),
            self._flags.eq(Mux(padding_mode, tail_length <= fragment_words * 2, frame_last)),
            self._overflow.eq(sink.valid & self._enabled & ~self._available &
                (sink.sof | (active & ~packet_active & ~zero_tail)) &
                (~empty_padding if _batched_packets else 1)),
            self._protocol_error.eq((sink.valid & sink.sof & active) |
                (sink.valid & ~sink.sof & ~active & self._enabled) | invalid_padding),
        ]
        # Equal power-of-two fragments share the full-frame word position.
        # Once padding starts, its complete descriptor is already committed;
        # advancing through the physical zeros requires no further payload write.
        if _shared:
            fragment_shift = (fragment_words - 1).bit_length()
            self.comb += [fragment_index.eq(frame_index[:fragment_shift]),
                          offset.eq((frame_index >> fragment_shift) << (fragment_shift + 1)),
                          half.eq(frame_index[0])]

        def position_update(*updates):
            return [] if _shared else updates

        if _held_frame_id:
            self.comb += frame_id.eq(sink.frame_id)
        else:
            _store_metadata(self, frame_id, sink.frame_id, sink.valid & sink.sof, use_dsp_metadata)
        self.sync += [
            If(sink.valid,
                If(sink.sof,
                    active.eq(can_start & ~active),
                    packet_active.eq(can_start & ~active & ~sink.padding),
                    *position_update(half.eq(1)),
                    first_word.eq(sink.data),
                    zero_tail.eq(sink.padding),
                    frame_index.eq(1),
                    *position_update(fragment_index.eq(1)),
                    *position_update(offset.eq(0)),
                ).Elif(take,
                    frame_index.eq(frame_index + 1),
                    If(~padding_mode,
                        *position_update(fragment_index.eq(fragment_index + 1)),
                        *position_update(half.eq(~half)),
                        If(~half, first_word.eq(sink.data)),
                    ),
                    If(padding_start,
                        zero_tail.eq(1),
                        packet_active.eq(0),
                        *position_update(half.eq(0)),
                    ),
                    If(~padding_mode,
                        packet_active.eq(1),
                        If(packet_last,
                            packet_active.eq(0),
                            *position_update(fragment_index.eq(0)),
                            *position_update(offset.eq(offset + (fragment_words * 2 if fixed_fragments else (fragment_index + 1) * 2))),
                            *position_update(half.eq(0)),
                        ),
                    ),
                    If(frame_last, active.eq(0), zero_tail.eq(0)),
                ).Elif(active,
                    # A service-bound violation stops this frame; the next SOF
                    # can recover after software clears the sticky fault.
                    active.eq(0), packet_active.eq(0), *position_update(half.eq(0)),
                ),
                If(invalid_padding,
                    active.eq(0), packet_active.eq(0), *position_update(half.eq(0)), zero_tail.eq(0),
                ),
            ),
        ]


class ChildPacketizer(_PacketSlots):
    """One complete compatibility centroid group per UDP datagram.

    Tuples are zero-extended to 64 bits, preserving all 33 native bits. The
    final valid tuple asserts ``end``. Consecutive tuples need no bubbles.
    Standalone packed slots store ``max_records`` native 33-bit words each.
    Shared transport uses aligned regions inside the combined child/pose RAM;
    the accepted record count per slot remains ``max_records``.
    Batched output requires exactly nine tuples and faults on shorter groups too.
    ``backpressured=True`` waits for room before starting a group. A reserved
    group then completes without stalls. The default retains the unstallable
    hierarchy input contract and reports whole-group overflow instead.
    """

    def __init__(self, camera_id, slots=8, max_records=9, packed_slots=True, _shared=False,
                 _external_destination=False, _batched_packets=False, backpressured=False, _queue_only=False,
                 _parser_only=False, use_dsp_metadata=False, _held_frame_id=False):
        if not 1 <= max_records <= (MAX_UDP_PAYLOAD - HEADER_BYTES) // 8:
            raise ValueError("child group does not fit in one UDP payload")
        if _batched_packets and max_records != 9:
            raise ValueError("batched child groups contain exactly nine tuples")
        slot_words = max_records if packed_slots and not _shared else 1 << ((max_records - 1).bit_length())
        if (_queue_only or _parser_only) and not _shared:
            raise ValueError("split parser/queue mode requires shared transport")
        if _parser_only:
            _initialize_parser_interface(self, slot_words, 33)
        else:
            super().__init__(camera_id, 2, slots, slot_words, width=33, max_payload_bytes=max_records * 8,
                             shared=_shared, external_destination=_external_destination, batched_packets=_batched_packets)
        self.sink = sink = stream.Endpoint([("data", 33), ("end", 1), ("frame_id", 32)])
        if _queue_only:
            return
        active = Signal()
        dropping = Signal()
        count = Signal(max=max_records + 1)
        frame_id = Signal(32)
        take = Signal()
        malformed = Signal()
        self.comb += [
            sink.ready.eq((active | dropping | ~self._enabled | self._available)
                          if backpressured else 1),
            take.eq(sink.valid & sink.ready & ~dropping & (active | (self._enabled & self._available))),
            malformed.eq(take & (((count == max_records - 1) & ~sink.end) |
                (active & (sink.frame_id != frame_id)) |
                ((sink.end & (count != 8)) if _batched_packets else 0))),
            self._writer_busy.eq(active),
            self._write_enable.eq(take & ~malformed),
            self._write_index.eq(count),
            self._write_data.eq(Cat(_be32(sink.data[:32]), sink.data[32]) if _shared else sink.data),
            self._commit.eq(take & sink.end & ~malformed),
            self._frame.eq(Mux(active, frame_id, sink.frame_id)),
            self._offset.eq(0),
            self._length.eq((count + 1) * 8),
            self._flags.eq(0),
            self._overflow.eq(0 if backpressured else
                (sink.valid & ~active & ~dropping & self._enabled & ~self._available)),
            self._protocol_error.eq(malformed),
        ]
        if _held_frame_id:
            self.comb += frame_id.eq(sink.frame_id)
        else:
            _store_metadata(self, frame_id, sink.frame_id, sink.valid & sink.ready & take & ~malformed & ~active, use_dsp_metadata)
        self.sync += If(sink.valid & sink.ready,
            If(take & ~malformed,
                count.eq(count + 1), active.eq(1),
            ).Else(active.eq(0), dropping.eq(1)),
            If(sink.end, count.eq(0), active.eq(0), dropping.eq(0)),
        )


class PosePacketizer(_PacketSlots):
    """Consume the solver's ready/valid packets and export accepted poses.

    Words 0 and 1 are staged in registers to inspect the accepted marker at
    word 1 bit 31 before reserving RAM. Eight-word rejections therefore retire
    even when every output slot is full. Accepted packets wait for a free slot,
    replay the prefix, then receive the rest. A 22-word packet is committed
    only if word 8 has valid bit 0 and no reserved status bits 4..31.
    The original common header is retained. ``fifo_level`` includes a pending
    classification prefix, so it can reach ``slots + 1`` for this controller.
    Packed slots allocate exactly the 22 accepted-result words; the two-word
    classification prefix is held in registers and rejections use no RAM slot.
    Shared transport places the frame ID in a private 23rd RAM word, written
    during one additional prefix clock. Only the 22 native words reach UDP.
    Shared RAM writes yield to simultaneous child records; ready holds native
    pose words until they can be stored, and rejects do not require RAM access.
    """

    def __init__(self, camera_id, slots=2, packed_slots=True, _shared=False,
                 _external_destination=False, _batched_packets=False, _queue_only=False,
                 _parser_only=False, use_dsp_metadata=False, _held_frame_id=False):
        if (_queue_only or _parser_only) and not _shared:
            raise ValueError("split parser/queue mode requires shared transport")
        if _parser_only:
            _initialize_parser_interface(self, 32, 32)
        else:
            super().__init__(camera_id, 3, slots, 32 if _shared else 22 if packed_slots else 32, max_payload_bytes=88,
                             shared=_shared, external_destination=_external_destination, batched_packets=_batched_packets)
        self.sink = sink = stream.Endpoint([("data", 32), ("frame_id", 32)])
        if _queue_only:
            return
        if not _shared:
            self.rejections = CSRStatus(32, name="rejections")
        active = Signal()
        forwarding = Signal()
        marker = Signal()
        invalid = Signal()
        reserved = Signal()
        prefix_phase = Signal(2)
        prefix0 = Signal(32)
        prefix1 = Signal(32)
        index = Signal(5)
        frame_id = Signal(32)
        status_valid = Signal()
        take = Signal()
        prefix_write = Signal()
        accepted = Signal()
        rejected = Signal()
        malformed = Signal()
        native_word = _be32(Mux(prefix_phase == 1, prefix0,
                                Mux(prefix_phase == 2, prefix1, sink.data)))
        self.comb += [
            sink.ready.eq((prefix_phase == 0) &
                (self._write_ready | ~active | ~forwarding | ~reserved if _shared else 1)),
            take.eq(sink.valid & sink.ready),
            prefix_write.eq(((prefix_phase == 2) | ((prefix_phase == 1) & self._available) |
                            ((prefix_phase == 3) if _shared else 0)) &
                            (self._write_ready if _shared else 1)),
            accepted.eq(marker & (index == 21) & status_valid),
            rejected.eq(~marker & (index == 7)),
            malformed.eq(take & ((sink.last & ~accepted & ~rejected) |
                (~sink.last & (index == 21)) | (active & (sink.frame_id != frame_id)))),
            self._writer_busy.eq(active & forwarding),
            self._write_enable.eq(prefix_write | (take & forwarding & reserved & ~malformed)),
            self._write_index.eq(Mux(prefix_phase == 1, 0, Mux(prefix_phase == 2, 1,
                Mux(prefix_phase == 3, 22, index) if _shared else index))),
            self._write_data.eq(Mux(prefix_phase == 3, frame_id, native_word)
                                if _shared else native_word),
            self._commit.eq(take & sink.last & accepted & forwarding & reserved & ~invalid & ~malformed),
            self._frame.eq(frame_id),
            self._offset.eq(0),
            self._length.eq(88),
            self._flags.eq(0),
            self._overflow.eq(0),
            self._protocol_error.eq(malformed),
        ]
        _store_metadata(self, prefix0, sink.data, take & ~active, use_dsp_metadata)
        if _held_frame_id:
            self.comb += frame_id.eq(sink.frame_id)
        else:
            _store_metadata(self, frame_id, sink.frame_id, take & ~active, use_dsp_metadata)
        _store_metadata(self, prefix1, sink.data, take & (index == 1), use_dsp_metadata)
        self.sync += [
            If(prefix_write,
                If(prefix_phase == 1,
                    prefix_phase.eq(2), reserved.eq(1),
                ).Elif((prefix_phase == 2) if _shared else 0, prefix_phase.eq(3)).Else(prefix_phase.eq(0)),
            ),
            If(take,
                If(~active,
                    forwarding.eq(self._enabled),
                    status_valid.eq(0), marker.eq(0), invalid.eq(0), reserved.eq(0),
                ),
                If(index == 1,
                    marker.eq(sink.data[31]),
                    If(sink.data[31] & forwarding & ~invalid & ~malformed,
                        prefix_phase.eq(1),
                    ).Else(forwarding.eq(0)),
                ),
                If(index == 8,
                    status_valid.eq(sink.data[0] & (sink.data[4:32] == 0)),
                ),
                active.eq(1),
                If(index != 31, index.eq(index + 1)),
                If(malformed, forwarding.eq(0), invalid.eq(1)),
                If(sink.last,
                    active.eq(0), forwarding.eq(0), index.eq(0), prefix_phase.eq(0), reserved.eq(0),
                    *([] if _shared else [If(rejected & ~invalid & ~malformed,
                        self.rejections.status.eq(self.rejections.status + 1))]),
                ),
            ),
        ]
        if _shared:
            self.comb += self.reject_event.eq(take & sink.last & rejected & ~invalid & ~malformed)


class PacketArbiter(Module):
    """Round-robin arbitration; ownership changes only after an accepted last."""

    def __init__(self, sources):
        if not sources:
            raise ValueError("at least one packet source is required")
        self.source = output = stream.Endpoint(eth_udp_user_description(32))
        count = len(sources)
        selected = Signal(max=max(2, count))
        cursor = Signal(max=max(2, count))
        locked = Signal()
        found = Signal()
        choice = Signal(max=max(2, count))
        choices = []
        for origin in range(count):
            chain = None
            for step in range(count):
                candidate = (origin + step) % count
                action = [found.eq(1), choice.eq(candidate)]
                if chain is None:
                    chain = If(sources[candidate].valid, *action)
                else:
                    chain = chain.Elif(sources[candidate].valid, *action)
            choices.append(If(cursor == origin, chain))
        self.comb += [found.eq(0), choice.eq(cursor), *choices]
        for i, source in enumerate(sources):
            self.comb += [source.ready.eq(0), If(locked & (selected == i), *source.connect(output))]
        self.sync += If(~locked,
            If(found, selected.eq(choice), locked.eq(1)),
        ).Elif(output.valid & output.ready & output.last,
            locked.eq(0),
            cursor.eq(Mux(selected == count - 1, 0, selected + 1)),
        )


class _FrameMemory(Memory):
    """Keep the eight-entry frame-ID banks in BRAM on Efinity.

    The generic memory remains simulatable; the synthesis attribute applies
    only to this declaration, without lowering the whole design's RAM limit.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # LiteX uses its own memory emitter and reads the public attributes.
        self.attr = {("syn_ramstyle", "block_ram")}

    @staticmethod
    def emit_verilog(memory, namespace, add_data_file):
        return '(* syn_ramstyle = "block_ram" *) ' + Memory.emit_verilog(memory, namespace, add_data_file)


def _initialize_shared_queue(owner, camera_id, stream_type, slots, slot_words,
                             width, fragment_bytes, frame_bytes, external_destination=False,
                             compact_padding=False, batched_packets=False):
    """Payload storage and capture-side state; no UDP formatter or counters."""
    if not _power_of_two(slots) or slots < 2:
        raise ValueError("slots must be a power of two and at least two")
    image = stream_type == 1
    child = stream_type == 2
    if image and (not _power_of_two(fragment_bytes) or frame_bytes % fragment_bytes):
        raise ValueError("shared image transport requires equal power-of-two fragments")
    fragment_count = frame_bytes // fragment_bytes if image else 1
    fragment_shift = fragment_bytes.bit_length() - 1 if image else 0
    fragment_bits = max(1, (fragment_count - 1).bit_length())
    frame_bits = frame_bytes.bit_length() if image else 32
    port = 6000 + camera_id * 3 + stream_type - 1
    owner.camera_id = camera_id
    owner.stream_type = stream_type
    owner.port = port
    owner.slots = slots

    # Raw controls/status are served by the shared software register block.
    owner.enable = Signal()
    owner.destination_ip = Signal(32)
    owner.destination_port = Signal(16, reset=port)
    owner.destination_update = Signal()
    owner.configuration_idle = Signal()
    owner.active_destination_ip = Signal(32)
    owner.active_destination_port = Signal(16, reset=port)
    owner.capture_enable = Signal(reset=1)
    owner.clear = Signal()
    owner.faults = Signal(2)
    owner.fault_sticky = Signal()
    owner.fifo_level = Signal(max=slots + 2)
    owner.committed_level = level = Signal(max=slots + 1)
    owner.counter_committed_level = Signal(max=slots + 1)
    owner.high_water = Signal(max=slots + 2)
    owner.commit_event = Signal(2)
    owner.sent_event = Signal()
    owner.bytes_event = Signal(16)
    owner.overflow_event = Signal()
    owner.protocol_event = Signal()
    owner.reject_event = Signal()

    # Capture writers use the same atomic-slot interface as the standalone
    # controllers, so framing/rejection/fault behavior has one implementation.
    owner._writer_busy = Signal()
    owner._overflow = Signal()
    owner._protocol_error = Signal()
    owner._enabled = Signal()
    owner._available = Signal()
    owner._commit = Signal()
    owner._frame = Signal(32)
    owner._offset = Signal(frame_bits)
    owner._length = Signal(11)
    owner._flags = Signal(32)
    owner._run_length = Signal(frame_bits)
    owner._stored_length = Signal(11)
    owner._write_ready = Signal(reset=1)
    owner._write_enable = Signal()
    owner._write_index = Signal(max=max(2, slot_words))
    owner._write_data = Signal(width)

    owner.present = Signal()
    owner.release = Signal()
    owner.peek_offset = Signal(max=slots)
    if not batched_packets:
        owner.comb += owner.peek_offset.eq(0)
    if batched_packets and not image:
        owner.packet_commit = Signal()
        owner.datagram_sent = Signal()
        owner.datagram_bytes = Signal(11)
        owner.packet_pending = Signal()
    owner.read_index = Signal(max=max(2, slot_words))
    owner.read_data = Signal(width)
    owner.head_frame = Signal(32)
    owner.head_offset = Signal(32)
    owner.head_words = Signal(9)
    owner.head_bytes = Signal(11)
    owner.head_final = Signal()
    owner.head_stored_words = Signal(9)
    write_slot = Signal(max=slots)
    read_slot = Signal(max=slots)
    peek_slot = Signal(max=slots)
    continue_run = Signal()
    dequeue = Signal()

    if image:
        payload = Memory(width, slots * slot_words, name="packet_payload")
        write = payload.get_port(write_capable=True)
        read = payload.get_port(mode=READ_FIRST)
        owner.specials += payload, write, read
    else:
        write = owner._payload_write = SimpleNamespace(
            adr=Signal(max=slots * slot_words), dat_w=Signal(width), we=Signal())
        read = owner._payload_read = SimpleNamespace(
            adr=Signal(max=slots * slot_words), dat_r=Signal(width))
        owner._payload_words = slots * slot_words
    owner.comb += [
        write.adr.eq(write_slot * slot_words + owner._write_index),
        write.dat_w.eq(owner._write_data), write.we.eq(owner._write_enable),
        peek_slot.eq(read_slot + owner.peek_offset),
        read.adr.eq(peek_slot * slot_words + owner.read_index),
        owner.read_data.eq(read.dat_r),
        owner.present.eq(level != 0),
        owner.configuration_idle.eq((level == 0) & ~owner._writer_busy),
        owner._available.eq(level < slots),
        owner.fifo_level.eq(level + owner._writer_busy),
        owner.fault_sticky.eq(owner.faults != 0),
        owner._enabled.eq(owner.enable & owner.capture_enable & ~owner.fault_sticky),
        owner.overflow_event.eq(owner._overflow),
        owner.protocol_event.eq(owner._protocol_error),
        dequeue.eq(owner.release & ~continue_run),
    ]
    if batched_packets and not image:
        owner.comb += [owner.commit_event.eq(owner.packet_commit),
                       owner.sent_event.eq(owner.datagram_sent),
                       owner.bytes_event.eq(owner.datagram_bytes),
                       owner.counter_committed_level.eq(owner.packet_pending)]
    else:
        owner.comb += [owner.commit_event.eq(owner._commit + (owner.release & continue_run)),
                       owner.sent_event.eq(owner.release),
                       owner.bytes_event.eq(HEADER_BYTES + owner.head_bytes),
                       owner.counter_committed_level.eq(level)]
    if stream_type != 3:
        owner.comb += owner.reject_event.eq(0)
    if not external_destination:
        owner.sync += If(owner.configuration_idle & owner.destination_update,
            owner.active_destination_ip.eq(owner.destination_ip),
            owner.active_destination_port.eq(owner.destination_port))
    owner.sync += [
        If(owner.fifo_level > owner.high_water, owner.high_water.eq(owner.fifo_level)),
        If(owner.clear, owner.faults.eq(0)),
        If(owner._overflow, owner.faults[0].eq(1)),
        If(owner._protocol_error, owner.faults[1].eq(1)),
        If(owner._commit ^ dequeue,
            level.eq(level + Mux(dequeue, -1, 1))),
        If(owner._commit, write_slot.eq(write_slot + 1)),
        If(dequeue, read_slot.eq(read_slot + 1)),
    ]

    # Commit immutable metadata once. The current virtual image run keeps
    # its fragment displacement outside RAM, so the bank needs one writer.
    metadata_width = 32 + (fragment_bits + 10 if image else 4 if child and not batched_packets else 0)
    if image or child:
        metadata_class = _FrameMemory if slots >= 8 else Memory
        metadata = metadata_class(metadata_width, slots, name="packet_metadata")
        metadata_write = metadata.get_port(write_capable=True)
        metadata_read = metadata.get_port(mode=READ_FIRST)
        owner.specials += metadata, metadata_write, metadata_read
        if image:
            initial_fragment = Signal(fragment_bits)
            owner.comb += initial_fragment.eq(owner._offset >> fragment_shift)
            metadata_input = Cat(owner._frame, initial_fragment,
                                 owner._stored_length[1:10] if compact_padding else owner._stored_length[2:11],
                                 owner._run_length != 0)
        elif child:
            metadata_input = owner._frame if batched_packets else Cat(owner._frame, owner._length[3:7])
        else:
            metadata_input = owner._frame
        owner.comb += [
            metadata_write.adr.eq(write_slot), metadata_write.dat_w.eq(metadata_input),
            metadata_write.we.eq(owner._commit), metadata_read.adr.eq(peek_slot),
            owner.head_frame.eq(metadata_read.dat_r[:32]),
        ]
        metadata_output = metadata_read.dat_r
    else:
        # Pose slots retain the frame ID in private payload word22. The shared
        # reader visits it during selection/header, before prefetching word0.
        owner.comb += owner.head_frame.eq(owner.read_data)

    if image and compact_padding:
        # A tail prefix contains 0..511 semantic 16-bit words. Full fragments
        # imply 512 words and do not need a length field. This retains the
        # existing 49-bit descriptor/RAM geometry while preserving odd-word
        # prefixes exactly. Physical producer padding remains checked by the
        # writer but is neither stored nor transmitted.
        head_fragment = metadata_output[32:32 + fragment_bits]
        prefix_words = metadata_output[32 + fragment_bits:41 + fragment_bits]
        tail = metadata_output[41 + fragment_bits]
        owner.comb += [
            owner.head_offset.eq(head_fragment << fragment_shift),
            owner.head_bytes.eq(Mux(tail, prefix_words << 1, fragment_bytes)),
            # Shift before rounding up: Verilog evaluates the original
            # (prefix_words + 1) >> 1 in the 9-bit destination context,
            # wrapping 511 + 1 to zero and preventing the final beat.
            owner.head_words.eq(Mux(tail, (prefix_words >> 1) + prefix_words[0], fragment_bytes // 4)),
            owner.head_final.eq(tail | (head_fragment == fragment_count - 1)),
            owner.head_stored_words.eq(owner.head_words),
            continue_run.eq(0),
        ]
    elif image:
        advance = Signal(fragment_bits)
        head_fragment = Signal(fragment_bits)
        initial_fragment = metadata_output[32:32 + fragment_bits]
        stored_words = metadata_output[32 + fragment_bits:41 + fragment_bits]
        tail = metadata_output[41 + fragment_bits]
        owner.comb += [
            head_fragment.eq(initial_fragment + advance),
            owner.head_offset.eq(head_fragment << fragment_shift),
            owner.head_words.eq(fragment_bytes // 4),
            owner.head_bytes.eq(fragment_bytes),
            owner.head_final.eq(head_fragment == fragment_count - 1),
            owner.head_stored_words.eq(Mux(advance == 0, stored_words, 0)),
            continue_run.eq(tail & ~owner.head_final),
        ]
        owner.sync += If(dequeue, advance.eq(0)).Elif(owner.release & continue_run, advance.eq(advance + 1))
    else:
        owner.comb += [continue_run.eq(0), owner.head_offset.eq(0), owner.head_final.eq(0),
                      owner.head_stored_words.eq(0)]
        if child:
            owner.comb += [owner.head_words.eq(18 if batched_packets else metadata_output[32:36] << 1),
                           owner.head_bytes.eq(72 if batched_packets else metadata_output[32:36] << 3)]
        else:
            owner.comb += [owner.head_words.eq(22), owner.head_bytes.eq(88)]


class SharedPacketTransport(Module):
    """Six atomic capture queues with one UDP/header serializer.

    ``channels`` are ordered camera 0 image/children/pose, then camera 1.
    Controls and counters live in the external software register block. Each
    channel exposes raw event increments and narrow live queue/fault status.
    ``bytes_event`` is valid only with ``sent_event``. ``commit_event`` can be
    two when capture commits while a virtual image descriptor advances.

    A sequence lookup begins at packet selection. Hold ``sequence_valid`` and
    its value until the request is accepted; the formatter emits nothing until
    that response, then retains the value through arbitrary downstream stalls.
    The provider must include all preceding ``sent_event`` increments for the
    selected stream. Bound lookup latency as part of the packet service budget.
    With ``external_destination=True``, the same response supplies destination
    IP/port and all three values are held for the complete datagram. Destination
    configuration then belongs to the external register bank; changes are only
    allowed while capture is paused and that channel is configuration-idle.
    ``external_context_held=True`` omits the duplicate destination registers: the
    provider must hold both destination inputs from lookup acceptance through
    the accepted final UDP word, including arbitrary downstream stalls.

    ``compact_padding=True`` selects compact output for all six streams. Images
    end at their semantic EOF; every descriptor produces exactly one datagram,
    including a header-only FINAL when padding starts at a fragment boundary.
    Pure producer padding is still validated through the physical frame end.
    The default preserves padded output for baseline comparisons.

    ``batched_packets=True`` additionally batches records. Images have no
    FINAL flag or empty termination packet. Child groups contain exactly nine
    tuples; up to four already-committed groups or two accepted poses of one
    frame share a datagram. Queues retain records in their existing RAM slots.
    ``committed_level`` continues to count records; ``counter_committed_level``
    counts image packets or the one fully planned auxiliary datagram. Auxiliary
    ``commit_event`` marks that plan, and sent/byte events mark its final word.

    ``accepted_centroids=True`` connects the child queues to the shared
    solver's accepted-group replay, which can wait for a free output slot.

    ``shared_solver_ingress=True`` shares each auxiliary parser across cameras.
    This requires the SharedLfTagDecoder contract: one camera owns an entire
    native pose and its following centroid group, and no other camera sends
    either stream until that transaction completes. After a partial-transfer
    abort, no new owner may send until the owning transport ``camera_reset``
    has cleared the parser. On the paired board, ordinary frame retirement
    waits for the solver to become idle; a fault holds both producers reset
    until pair CLEAR/RESET also resets the transport. A decoder-only reset
    does not by itself meet the transport reset contract.
    The two committed queues and their capacities remain independent. Generic
    callers retain separate ingress parsers by default. Only a valid input's
    ready signal is observable under this single-owner contract.

    ``held_frame_id=True`` is an additional producer-lifetime contract, used
    only by the paired board. All three input frame IDs of each camera must
    be tied to the same retained camera frame ID. Hold it through every image
    word (including physical padding), pose classification/prefix replay, and
    accepted centroid group, including invalid gaps and backpressure. A new
    ID is allowed after those producers retire; queued records retain their
    own IDs in RAM. Changing the ID earlier requires the owning camera_reset
    on the same edge, clearing all unfinished parser and queue state. A fault
    must prevent either camera from replacing an abandoned shared-parser owner
    until that reset. In particular, drained alone is insufficient: a faulted
    partial pose prefix can remain uncommitted. The paired CLEAR/RESET wiring
    resets transport and camera IDs together. Generic callers keep this option
    false and retain the mid-packet frame-ID collision checks.

    ``use_dsp_metadata=True`` stores frame IDs and classification prefixes in
    native DSP W registers; all capture, clear, and reset edges are unchanged.
    """

    def __init__(self, cameras=2, image_slots=8, child_slots=8, pose_slots=2,
                 frame_words=64000, fragment_words=IMAGE_FRAGMENT_WORDS,
                 external_destination=False, external_context_held=False,
                 compact_padding=False, batched_packets=False, accepted_centroids=False,
                 shared_solver_ingress=False, use_dsp_metadata=False, held_frame_id=False):
        if held_frame_id and not (shared_solver_ingress and accepted_centroids and batched_packets):
            raise ValueError("held frame IDs require the complete production shared-solver topology")
        if batched_packets and not compact_padding:
            raise ValueError("batched packets require compact image padding")
        if external_context_held and not external_destination:
            raise ValueError("held destination context requires external destinations")
        if shared_solver_ingress and not accepted_centroids:
            raise ValueError("shared ingress requires the single shared solver")
        if cameras != 2:
            raise ValueError("the shared transport contains two camera pipelines")
        self.camera_reset = [Signal() for _ in range(cameras)]
        self.channels = channels = []
        for camera in range(cameras):
            products = [ImagePacketizer(camera, image_slots, frame_words, fragment_words, _shared=True,
                                       _external_destination=external_destination, _compact_padding=compact_padding,
                                       _batched_packets=batched_packets, use_dsp_metadata=use_dsp_metadata, _held_frame_id=held_frame_id),
                        ChildPacketizer(camera, child_slots, _shared=True, _queue_only=shared_solver_ingress, _external_destination=external_destination,
                                        _batched_packets=batched_packets, backpressured=accepted_centroids, use_dsp_metadata=use_dsp_metadata, _held_frame_id=held_frame_id),
                        PosePacketizer(camera, pose_slots, _shared=True, _queue_only=shared_solver_ingress, _external_destination=external_destination,
                                       _batched_packets=batched_packets, use_dsp_metadata=use_dsp_metadata, _held_frame_id=held_frame_id)]
            for product, queue in enumerate(products):
                queue = ResetInserter()(queue)
                setattr(self.submodules, f"camera{camera}_queue{product}", queue)
                self.comb += [queue.reset.eq(self.camera_reset[camera]),
                              queue._storage_reset.eq(self.camera_reset[camera])]
                channels.append(queue)

        if shared_solver_ingress:
            for kind in (1, 2):
                queues = [channels[kind], channels[3 + kind]]
                parser = (ChildPacketizer(0, child_slots, _shared=True, _parser_only=True, use_dsp_metadata=use_dsp_metadata, _held_frame_id=held_frame_id,
                            _batched_packets=batched_packets, backpressured=True) if kind == 1 else
                          PosePacketizer(0, pose_slots, _shared=True, _parser_only=True, use_dsp_metadata=use_dsp_metadata, _held_frame_id=held_frame_id,
                            _batched_packets=batched_packets))
                parser = ResetInserter()(parser)
                setattr(self.submodules, "shared_child_ingress" if kind == 1 else "shared_pose_ingress", parser)
                # Retain the most recent owner across prefix replay and
                # invalid gaps, so an abort reaches its parser even when no
                # producer beat is currently valid. A new valid owner can
                # appear only after completion or the previous owner's reset.
                owner = Signal()
                chosen = Signal()
                self.comb += chosen.eq(Mux(queues[1].sink.valid, 1,
                                          Mux(queues[0].sink.valid, 0, owner)))
                self.sync += If(queues[0].sink.valid | queues[1].sink.valid, owner.eq(chosen))
                self.comb += [parser.reset.eq(Array(self.camera_reset)[chosen]),
                              parser._storage_reset.eq(Array(self.camera_reset)[chosen]),
                              parser._enabled.eq(Array(q._enabled for q in queues)[chosen]),
                              parser._available.eq(Array(q._available for q in queues)[chosen]),
                              parser._write_ready.eq(Array(q._write_ready for q in queues)[chosen]),
                              parser.sink.valid.eq(Array(q.sink.valid for q in queues)[chosen])]
                for name, _width in parser.sink.description.payload_layout:
                    self.comb += getattr(parser.sink, name).eq(Array(getattr(q.sink, name) for q in queues)[chosen])
                self.comb += [parser.sink.first.eq(Array(q.sink.first for q in queues)[chosen]),
                              parser.sink.last.eq(Array(q.sink.last for q in queues)[chosen])]
                for camera, queue in enumerate(queues):
                    self.comb += queue.sink.ready.eq(parser.sink.ready)
                    for field in ("_writer_busy", "_overflow", "_protocol_error", "_commit", "_write_enable"):
                        self.comb += getattr(queue, field).eq(getattr(parser, field) & (chosen == camera))
                    for field in ("_frame", "_offset", "_length", "_flags", "_run_length", "_stored_length", "_write_index", "_write_data"):
                        self.comb += getattr(queue, field).eq(getattr(parser, field))
                    if kind == 2:
                        self.comb += queue.reject_event.eq(parser.reject_event & (chosen == camera))

        self.source = source = stream.Endpoint(eth_udp_user_description(32))
        self.drained = Signal()
        self.sequence_request = Signal()
        self.sequence_stream = Signal(3)
        self.sequence_valid = Signal()
        self.sequence_value = Signal(32)
        self.sequence_destination_ip = Signal(32)
        self.sequence_destination_port = Signal(16)
        if external_context_held:
            destination_ip = self.sequence_destination_ip
            destination_port = self.sequence_destination_port
        elif external_destination:
            destination_ip = Signal(32)
            destination_port = Signal(16)
        else:
            destination_ip = Array(queue.active_destination_ip for queue in channels)
            destination_port = Array(queue.active_destination_port for queue in channels)
        selected = Signal(3)
        # One SDP RAM bank per camera holds both small payload queues. Child
        # input owns conflicting writes; accepted-only replay can stall at a
        # full queue. Accepted pose input and prefix replay wait. Rejection
        # classification needs no RAM.
        # Power-of-two address regions simplify selection without allocating
        # another physical RAM or increasing either queue's logical capacity.
        for camera in range(cameras):
            child, pose = channels[camera * 3 + 1:camera * 3 + 3]
            child_write, pose_write = child._payload_write, pose._payload_write
            child_read, pose_read = child._payload_read, pose._payload_read
            pose_base = 1 << ((child._payload_words - 1).bit_length())
            payload = Memory(33, pose_base + pose._payload_words,
                             name=f"camera{camera}_child_pose_payload")
            write = payload.get_port(write_capable=True)
            read = payload.get_port(mode=READ_FIRST)
            self.specials += payload, write, read
            self.comb += [
                pose._write_ready.eq(~child_write.we),
                write.we.eq(child_write.we | pose_write.we),
                write.adr.eq(Mux(child_write.we, child_write.adr,
                                 pose_base + pose_write.adr)),
                write.dat_w.eq(Mux(child_write.we, child_write.dat_w, pose_write.dat_w)),
                read.adr.eq(Mux(selected == camera * 3 + 2,
                                pose_base + pose_read.adr, child_read.adr)),
                child_read.dat_r.eq(read.dat_r), pose_read.dat_r.eq(read.dat_r[:32]),
            ]
        if batched_packets:
            _initialize_batched_sender(self, channels, selected, destination_ip, destination_port,
                                       external_destination, external_context_held)
            return
        cursor = Signal(3)
        selecting = Signal()
        reading = Signal()
        in_payload = Signal()
        header_index = Signal(3)
        word_index = Signal(8)
        sequence = Signal(32)
        found = Signal()
        choice = Signal(3)
        accept = Signal()
        last = Signal()
        head_words = Array(queue.head_words for queue in channels)[selected]
        head_bytes = Array(queue.head_bytes for queue in channels)[selected]
        camera_reset = Array(self.camera_reset)[selected >= 3]
        after_found = Signal()
        after_choice = Signal(3)
        fallback = None
        preferred = None
        for candidate, queue in enumerate(channels):
            fallback_action = [found.eq(1), choice.eq(candidate)]
            preferred_action = [after_found.eq(1), after_choice.eq(candidate)]
            condition = queue.present & (cursor <= candidate)
            if fallback is None:
                fallback = If(queue.present, *fallback_action)
                preferred = If(condition, *preferred_action)
            else:
                fallback = fallback.Elif(queue.present, *fallback_action)
                preferred = preferred.Elif(condition, *preferred_action)
        self.comb += [
            found.eq(0), choice.eq(0), after_found.eq(0), after_choice.eq(0),
            fallback, preferred,
            If(after_found, choice.eq(after_choice)),
        ]
        def selected_word(values):
            value = Constant(0, 32)
            for index, candidate in enumerate(values):
                value = value | Mux(selected == index, candidate, Constant(0, 32))
            return value

        info_words = Array(Constant(queue.camera_id << 16 |
                                    queue.stream_type << 8 | HEADER_BYTES, 32) for queue in channels)
        header = [
            _be32(MAGIC), _be32(info_words[selected]),
            _be32(selected_word(queue.head_frame for queue in channels)),
            _be32(sequence),
            _be32(Array(queue.head_offset for queue in channels)[selected]),
            _be32(Cat(head_bytes, Constant(0, 21))),
            _be32(Cat(Array(queue.head_final for queue in channels)[selected], Constant(0, 31))),
            Constant(0, 32),
        ]
        header_word = Constant(0, 32)
        for index, value in enumerate(header):
            header_word = header_word | Mux(header_index == index, value, Constant(0, 32))
        payloads = []
        for index, queue in enumerate(channels):
            if queue.stream_type == 2:
                payloads.append(Mux(word_index[0], queue.read_data[:32],
                                    _be32(Cat(queue.read_data[32], Constant(0, 31)))))
            elif queue.stream_type == 1:
                payloads.append(queue.read_data if compact_padding else
                                Mux(word_index >= queue.head_stored_words, 0, queue.read_data))
            else:
                payloads.append(queue.read_data)
            next_word = Mux(accept, Mux(last, 0, word_index + 1), word_index)
            self.comb += [
                queue.release.eq(accept & last & (selected == index)),
                queue.read_index.eq(Mux(reading & in_payload & (selected == index),
                    next_word >> (1 if queue.stream_type == 2 else 0),
                    Mux(reading & (header_index == 7) & (selected == index), 0, 22)
                    if queue.stream_type == 3 else 0)),
            ]
        # Child low words and pose words share wire-lane order in one RAM.
        # Select that bank once; only child high words require zero extension.
        payload_word = Constant(0, 32)
        for camera in range(cameras):
            image_index = camera * 3
            child, pose = channels[image_index + 1:image_index + 3]
            is_pose = selected == image_index + 2
            bank_word = Mux(is_pose | word_index[0], pose.read_data,
                            _be32(Cat(child.read_data[32], Constant(0, 31))))
            payload_word = payload_word | Mux(selected == image_index, payloads[image_index], 0)
            payload_word = payload_word | Mux((selected == image_index + 1) | is_pose, bank_word, 0)
        self.comb += [
            self.sequence_request.eq(selecting & ~camera_reset),
            self.drained.eq(~selecting & ~reading &
                (Cat(*(queue.present for queue in channels)) == 0)),
            self.sequence_stream.eq(selected),
            source.valid.eq(reading & ~camera_reset),
            source.first.eq(reading & ~in_payload & (header_index == 0)),
            last.eq((in_payload & (word_index + 1 == head_words)) |
                    (~in_payload & (header_index == 7) & (head_bytes == 0))),
            source.last.eq(reading & last),
            source.data.eq(Mux(in_payload, payload_word, header_word)),
            source.last_be.eq(Mux(last, Mux(head_bytes[1], 2, 8), 0)), source.error.eq(0),
            source.length.eq(HEADER_BYTES + head_bytes),
            source.src_port.eq(Array(Constant(queue.port, 16) for queue in channels)[selected]),
            source.dst_port.eq(destination_port if external_destination else destination_port[selected]),
            source.ip_address.eq(destination_ip if external_destination else destination_ip[selected]),
            accept.eq(source.valid & source.ready),
        ]
        if external_destination and not external_context_held:
            self.sync += If(selecting & self.sequence_valid & ~camera_reset,
                destination_ip.eq(self.sequence_destination_ip),
                destination_port.eq(self.sequence_destination_port))
        self.sync += [
            If(~selecting & ~reading,
                If(found, selected.eq(choice), selecting.eq(1)),
                header_index.eq(0), word_index.eq(0), in_payload.eq(0),
            ).Elif(selecting,
                If(self.sequence_valid & ~camera_reset,
                    sequence.eq(self.sequence_value), selecting.eq(0), reading.eq(1)),
            ).Elif(accept,
                If(~in_payload,
                    If(header_index == 7, in_payload.eq(1)).Else(header_index.eq(header_index + 1)),
                ).Else(word_index.eq(word_index + 1)),
                If(last,
                    reading.eq(0), cursor.eq(Mux(selected == len(channels) - 1, 0, selected + 1))),
            ),
            If(camera_reset & (selecting | reading), selecting.eq(0), reading.eq(0)),
        ]


def _initialize_batched_sender(owner, channels, selected, destination_ip, destination_port,
                               external_destination, external_context_held):
    """Plan bounded batches by scanning the existing committed queue memories.

    Only the selected frame, aggregate byte count, and record count are held
    outside those memories. The scan never includes records committed after
    selection. Each inspected record takes two clocks for the synchronous RAM;
    the head is restored during the following sequence lookup/header clocks.
    At a record boundary the RAM prefetch visits the next slot on the same
    edge that releases the old slot, so concatenation needs no extra buffer.
    """
    IDLE, SCAN_WAIT, SCAN_EVAL, LOOKUP, HEADER, PAYLOAD = range(6)
    state = Signal(max=6)
    cursor = Signal(3)
    found = Signal()
    choice = Signal(3)
    after_found = Signal()
    after_choice = Signal(3)
    scan_index = Signal(2)
    scan_limit = Signal(3)
    batch_frame = Signal(32)
    batch_bytes = Signal(11)
    batch_remaining = Signal(3)
    sequence = Signal(32, name="packet_sequence")
    header_index = Signal(3)
    word_index = Signal(8)
    source = owner.source
    accept = Signal()
    record_last = Signal()
    last = Signal()
    plan_done = Signal()
    same_frame = Signal()
    camera_reset = Array(owner.camera_reset)[selected >= 3]
    head_words = Array(queue.head_words for queue in channels)[selected]
    head_bytes = Array(queue.head_bytes for queue in channels)[selected]
    head_frame = Array(queue.head_frame for queue in channels)[selected]

    fallback = preferred = None
    for index, queue in enumerate(channels):
        fallback_action = [found.eq(1), choice.eq(index)]
        preferred_action = [after_found.eq(1), after_choice.eq(index)]
        available = queue.present & ~owner.camera_reset[queue.camera_id]
        condition = available & (cursor <= index)
        if fallback is None:
            fallback = If(available, *fallback_action)
            preferred = If(condition, *preferred_action)
        else:
            fallback = fallback.Elif(available, *fallback_action)
            preferred = preferred.Elif(condition, *preferred_action)
    owner.comb += [
        found.eq(0), choice.eq(0), after_found.eq(0), after_choice.eq(0),
        fallback, preferred,
        If(after_found, choice.eq(after_choice)),
        same_frame.eq((scan_index == 0) | (head_frame == batch_frame)),
        plan_done.eq((state == SCAN_EVAL) &
                     (~same_frame | (scan_index + 1 == scan_limit)) & ~camera_reset),
    ]
    limits = Array(Mux(queue.committed_level > (4 if queue.stream_type == 2 else 2),
                       4 if queue.stream_type == 2 else 2, queue.committed_level)
                   if queue.stream_type != 1 else Constant(1, 3) for queue in channels)

    def selected_word(values):
        result = Constant(0, 32)
        for index, value in enumerate(values):
            result = result | Mux(selected == index, value, Constant(0, 32))
        return result

    info = Array(Constant(queue.camera_id << 16 |
                          queue.stream_type << 8 | HEADER_BYTES, 32) for queue in channels)
    header = [
        _be32(MAGIC), _be32(info[selected]), _be32(batch_frame), _be32(sequence),
        _be32(selected_word(queue.head_offset for queue in channels)),
        _be32(Cat(batch_bytes, Constant(0, 21))), Constant(0, 32), Constant(0, 32),
    ]
    header_word = Constant(0, 32)
    for index, value in enumerate(header):
        header_word = header_word | Mux(header_index == index, value, Constant(0, 32))

    for index, queue in enumerate(channels):
        chosen = selected == index
        release = accept & (state == PAYLOAD) & record_last & chosen
        # A release advances read_slot at this edge. Prefetch the successor
        # through the old pointer plus one, then resume offset zero next cycle.
        owner.comb += [
            queue.release.eq(release),
            queue.peek_offset.eq(Mux((state == SCAN_WAIT) | (state == SCAN_EVAL),
                                     scan_index, Mux(release & ~last, 1, 0)) if queue.stream_type != 1 else 0),
        ]
        next_word = Mux(accept, Mux(record_last, 0, word_index + 1), word_index)
        if queue.stream_type == 3:
            index_value = Mux(state == PAYLOAD, next_word,
                              Mux((state == HEADER) & (header_index == 7), 0, 22))
        else:
            index_value = Mux(state == PAYLOAD,
                              next_word >> (1 if queue.stream_type == 2 else 0), 0)
        owner.comb += queue.read_index.eq(index_value)
        if queue.stream_type != 1:
            owner.comb += [
                queue.packet_commit.eq(plan_done & chosen),
                queue.packet_pending.eq(chosen & ((state == LOOKUP) | (state == HEADER) | (state == PAYLOAD))),
                queue.datagram_sent.eq(accept & last & chosen),
                queue.datagram_bytes.eq(HEADER_BYTES + batch_bytes),
            ]

    payload_word = Constant(0, 32)
    for camera in range(2):
        image_index = camera * 3
        image, child, pose = channels[image_index:image_index + 3]
        is_pose = selected == image_index + 2
        bank_word = Mux(is_pose | word_index[0], pose.read_data,
                        _be32(Cat(child.read_data[32], Constant(0, 31))))
        payload_word = payload_word | Mux(selected == image_index, image.read_data, 0)
        payload_word = payload_word | Mux((selected == image_index + 1) | is_pose, bank_word, 0)
    owner.comb += [
        owner.sequence_request.eq((state == LOOKUP) & ~camera_reset),
        owner.drained.eq((state == IDLE) & (Cat(*(queue.present for queue in channels)) == 0)),
        owner.sequence_stream.eq(selected),
        source.valid.eq(((state == HEADER) | (state == PAYLOAD)) & ~camera_reset),
        source.first.eq((state == HEADER) & (header_index == 0)),
        record_last.eq(word_index + 1 == head_words),
        last.eq((state == PAYLOAD) & record_last & (batch_remaining == 1)),
        source.last.eq(last), source.error.eq(0),
        source.data.eq(Mux(state == PAYLOAD, payload_word, header_word)),
        source.last_be.eq(Mux(last, Mux(batch_bytes[1], 2, 8), 0)),
        source.length.eq(HEADER_BYTES + batch_bytes),
        source.src_port.eq(Array(Constant(queue.port, 16) for queue in channels)[selected]),
        source.dst_port.eq(destination_port if external_destination else destination_port[selected]),
        source.ip_address.eq(destination_ip if external_destination else destination_ip[selected]),
        accept.eq(source.valid & source.ready),
    ]
    if external_destination and not external_context_held:
        owner.sync += If((state == LOOKUP) & owner.sequence_valid & ~camera_reset,
            destination_ip.eq(owner.sequence_destination_ip),
            destination_port.eq(owner.sequence_destination_port))
    owner.sync += [
        If(state == IDLE,
            header_index.eq(0), word_index.eq(0), scan_index.eq(0),
            If(found,
                selected.eq(choice), scan_limit.eq(limits[choice]),
                batch_bytes.eq(0), batch_remaining.eq(0), state.eq(SCAN_WAIT)),
        ).Elif(state == SCAN_WAIT,
            state.eq(SCAN_EVAL),
        ).Elif(state == SCAN_EVAL,
            If(same_frame,
                If(scan_index == 0, batch_frame.eq(head_frame)),
                batch_bytes.eq(batch_bytes + head_bytes),
                batch_remaining.eq(batch_remaining + 1)),
            If(plan_done, state.eq(LOOKUP)).Else(scan_index.eq(scan_index + 1), state.eq(SCAN_WAIT)),
        ).Elif(state == LOOKUP,
            If(owner.sequence_valid & ~camera_reset, sequence.eq(owner.sequence_value), state.eq(HEADER)),
        ).Elif(accept,
            If(state == HEADER,
                If(header_index == 7, state.eq(PAYLOAD)).Else(header_index.eq(header_index + 1)),
            ).Else(
                If(record_last, word_index.eq(0), batch_remaining.eq(batch_remaining - 1)).Else(
                    word_index.eq(word_index + 1)),
                If(last, state.eq(IDLE), cursor.eq(Mux(selected == len(channels) - 1, 0, selected + 1))),
            ),
        ),
        If(camera_reset & (state != IDLE), state.eq(IDLE)),
    ]
