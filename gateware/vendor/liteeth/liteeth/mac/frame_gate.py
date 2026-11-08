"""Publish complete validated frames before any host bus effects."""
from migen import Cat, Constant, If, Memory, Module, Mux, Signal
from litex.soc.interconnect import stream
from liteeth.common import eth_phy_description


class ValidatedEthernetFrameGate(Module):
    """One 2048-byte checked-frame slot, with an always-ready physical input.

    ``frame_dropped``, ``frame_error`` and ``frame_overflow`` pulse at the
    actual last beat of a discarded frame. ``busy`` means a committed frame
    owns the RAM. Missing last never fabricates a boundary. Final byte-enable
    must be one-hot; non-final byte-enable is ignored.

    Fixed IPv4/UDP length checks precede publication. The board's IP receiver
    already accepts only IPv4/IHL5. Extra Ethernet padding remains allowed,
    but the complete advertised IP datagram must exist and UDP must consume
    exactly the IP payload. ARP and other EtherTypes retain normal handling.
    """

    def __init__(self, dw=16, depth=1024):
        if dw != 16 or depth < 2 or depth & (depth - 1):
            raise ValueError("Frame gate requires 16-bit words and power-of-two depth >= 2")
        self.sink = sink = stream.Endpoint(eth_phy_description(dw))
        self.source = source = stream.Endpoint(eth_phy_description(dw))
        self.busy = owned = Signal()
        self.frame_dropped = Signal()
        self.frame_error = Signal()
        self.frame_overflow = Signal()
        self.frame_length = Signal(max=2 * depth + 1)
        self.memory = memory = Memory(dw, depth, name="checked_frame")
        memory.attr = {("syn_ramstyle", "block_ram")}
        port = memory.get_port(write_capable=True, has_re=True, mode=0)
        self.specials += memory, port

        FREE, CAPTURE, DROP = range(3)
        state = Signal(2)
        write_index = Signal(max=depth)
        last_index = Signal(max=depth)
        bad = Signal()
        overflow = Signal()
        ipv4 = Signal()
        fixed_ipv4 = Signal()
        udp = Signal()
        # High advertised-length bits can only reject a bounded physical
        # frame. Preserve that predicate rather than the discarded bits.
        # At least five bits keep +14/+20 exact even for tiny test buffers.
        length_width = min(16, max(5, depth.bit_length()))
        # Short IP/UDP frames reject independently of these values; every
        # potentially accepted frame captures both fields before its last beat.
        ip_length = Signal(length_width, reset_less=True)
        udp_length = Signal(length_width, reset_less=True)
        ip_length_high = Signal(reset_less=True)
        udp_length_high = Signal(reset_less=True)
        received_length = Cat(sink.data[8:16], sink.data[:8])
        byte_length = Signal(max=2 * depth + 1)
        ip_frame_length = Signal(length_width + 1)
        length_bad = Signal()
        final_be_bad = (sink.last_be != 1) & (sink.last_be != 2)
        ipv4_now = Mux(write_index == 6, sink.data == 0x0008, ipv4)
        self.comb += [
            sink.ready.eq(1),
            byte_length.eq((write_index << 1) + Mux(sink.last_be == 1, 1, 2)),
            ip_frame_length.eq(Cat(ip_length, Constant(0, 1)) + 14),
            length_bad.eq(ipv4_now & ((byte_length < 34) | ~fixed_ipv4 | ip_length_high | (ip_length < 20) |
                (byte_length < ip_frame_length) |
                (udp & ((ip_length < 28) | udp_length_high | (udp_length < 8) |
                        (Cat(udp_length, Constant(0, 1)) + 20 != ip_length))))),
        ]
        capture = (state != DROP) & ~owned
        start = (state == FREE)
        active_bad = Mux(start, 0, bad) | (sink.error != 0)
        # A short new frame must not consult header state from its predecessor.
        complete_bad = active_bad | final_be_bad | ((write_index >= 6) & length_bad)
        self.comb += [
            port.we.eq(sink.valid & capture),
            port.dat_w.eq(sink.data),
        ]
        read_index = Signal(max=depth)
        read_valid = Signal()
        read_last = Signal()
        read_all_issued = Signal()
        final_be = Signal(2)
        issue_read = owned & ~read_all_issued & (~read_valid | source.ready)
        self.comb += [
            port.adr.eq(Mux(owned, read_index, write_index)),
            port.re.eq(issue_read),
            source.valid.eq(read_valid), source.data.eq(port.dat_r), source.first.eq(0),
            source.last.eq(read_last), source.last_be.eq(Mux(read_last, final_be, 0)),
            source.error.eq(0),
        ]
        self.sync += [
            self.frame_dropped.eq(0), self.frame_error.eq(0), self.frame_overflow.eq(0),
            If(issue_read,
                read_valid.eq(1), read_last.eq(read_index == last_index),
                If(read_index == last_index, read_all_issued.eq(1)).Else(
                    read_index.eq(read_index + 1)),
            ).Elif(source.valid & source.ready, read_valid.eq(0)),
            If(source.valid & source.ready & source.last, owned.eq(0)),
            If(sink.valid,
                If(start,
                    bad.eq(sink.error != 0), overflow.eq(0),
                    ipv4.eq(0), fixed_ipv4.eq(0), udp.eq(0),
                ).Else(bad.eq(bad | (sink.error != 0))),
                If(capture,
                    If(write_index == 6, ipv4.eq(sink.data == 0x0008)),
                    If(write_index == 7, fixed_ipv4.eq(sink.data[:8] == 0x45)),
                    If(write_index == 8, ip_length.eq(received_length[:length_width]),
                       ip_length_high.eq(received_length[length_width:] != 0 if length_width < 16 else 0)),
                    If(write_index == 11, udp.eq(sink.data[8:16] == 17)),
                    If(write_index == 19, udp_length.eq(received_length[:length_width]),
                       udp_length_high.eq(received_length[length_width:] != 0 if length_width < 16 else 0)),
                    If(sink.last,
                        state.eq(FREE), write_index.eq(0),
                        If(complete_bad,
                            self.frame_dropped.eq(1), self.frame_error.eq(1),
                        ).Else(
                            owned.eq(1), last_index.eq(write_index), final_be.eq(sink.last_be),
                            self.frame_length.eq(byte_length), read_index.eq(0), read_all_issued.eq(0),
                        ),
                    ).Elif(write_index == depth - 1,
                        overflow.eq(1), state.eq(DROP),
                    ).Else(write_index.eq(write_index + 1), state.eq(CAPTURE)),
                ).Else(
                    If(sink.last,
                        state.eq(FREE), write_index.eq(0),
                        self.frame_dropped.eq(1),
                        self.frame_error.eq(active_bad | final_be_bad),
                        self.frame_overflow.eq(~start & overflow),
                    ).Else(state.eq(DROP)),
                ),
            ),
        ]

