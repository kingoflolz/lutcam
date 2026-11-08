"""CPU-owned UDP/68 aperture and dynamic network configuration.

RX remains in the already checked MAC frame slot until consumed or discarded.
TX reads a complete payload from CPU SRAM through one held output word.
DHCP parsing, transactions and lease policy are entirely firmware functions.
"""
from migen import Cat, Case, Constant, If, Module, Mux, Signal
from litex.soc.interconnect import stream, wishbone

NETWORK_BASE = 0x30029000
NETWORK_MAGIC = 0x4c4e4554
DHCP_TX_BYTES = 268  # The resident client always sends its 67-word BOOTP buffer.


class NetworkControl(Module):
    def __init__(self, udp_port, dma, *, ram_bytes, mac_address, preferred_ip, shared_readback=None):
        self.bus = bus = wishbone.Interface(data_width=32, addressing="word")
        response = Signal(32) if shared_readback is None else shared_readback.word
        self.comb += bus.dat_r.eq(response)
        self.granted, self.inhibit = Signal(), Signal()
        self.revoke_capture = Signal()
        self.ip_address, self.netmask, self.gateway = Signal(32), Signal(32), Signal(32)
        self.state, self.lease, self.server = Signal(3), Signal(32), Signal(32)
        rx, tx = udp_port.source, udp_port.sink
        receive_enable, owned, complete, draining = [Signal() for _ in range(4)]
        seen, request, pop, release, start = [Signal() for _ in range(5)]
        busy = Signal()
        tx_address = Signal(len(dma.address))
        tx_destination = Signal(32)
        position = Signal.like(dma.address)
        end = Signal(len(dma.address)+1) # Includes the exclusive end at RAM_BYTES.
        all_issued, read_last = Signal(), Signal()
        pending, valid, last = Signal(), Signal(), Signal()
        payload = Signal(32)
        eligible = (rx.src_port == 67) & (rx.length >= 244) & (rx.length <= 1472) & (rx.error == 0)
        available = rx.valid & eligible & receive_enable & ~self.inhibit & ~draining & ~complete
        self.comb += [
            request.eq(bus.cyc & bus.stb & ~seen),
            pop.eq(request & ~bus.we & self.granted & (bus.adr[:10] == 0x1c//4) & available),
            release.eq(request & bus.we & self.granted & (bus.sel == 15)
                       & (bus.adr[:10] == 0x20//4) & bus.dat_w[0]),
            self.revoke_capture.eq(request & bus.we & self.granted & (bus.sel == 15)
                                   & (bus.adr[:10] == 0x4c//4) & bus.dat_w[0]),
            rx.ready.eq(~receive_enable | self.inhibit | draining | (~eligible & ~complete) | pop),
            dma.request.eq(busy & ~all_issued & ~pending & ~valid),
            dma.address.eq(position),
            tx.valid.eq(valid), tx.data.eq(payload), tx.last.eq(last),
            tx.last_be.eq(Mux(last, 8, 0)), tx.src_port.eq(68), tx.dst_port.eq(67),
            tx.length.eq(DHCP_TX_BYTES), tx.ip_address.eq(tx_destination), tx.error.eq(0),
        ]
        self.sync += [
            If(dma.request & dma.ready, pending.eq(1)),
            If(tx.valid & tx.ready, valid.eq(0)),
            If(dma.ack, pending.eq(0), valid.eq(1), payload.eq(dma.data), last.eq(read_last)),
            If(pop, owned.eq(1), If(rx.last, complete.eq(1))),
            If(release,
                If(owned & ~complete, draining.eq(1)).Else(owned.eq(0), complete.eq(0))),
            If(rx.valid & rx.ready & rx.last & draining,
                draining.eq(0), owned.eq(0), complete.eq(0)),
            If(~receive_enable | self.inhibit, draining.eq(0), owned.eq(0), complete.eq(0)),
            If(start, busy.eq(1), position.eq(tx_address),
               end.eq(tx_address + DHCP_TX_BYTES//4), all_issued.eq(0)),
            If(dma.request & dma.ready,
                position.eq(position + 1), read_last.eq(position + 1 == end),
                If(position + 1 == end, all_issued.eq(1))),
            If(tx.valid & tx.ready & tx.last, busy.eq(0)),
        ]
        mac = Constant(mac_address, 48) if isinstance(mac_address, int) else mac_address
        reads = {0:Constant(NETWORK_MAGIC,32), 4:Cat(available,busy),
                 8:self.ip_address, 12:self.netmask, 16:self.gateway,
                 0x14:rx.length, 0x18:rx.ip_address,
                 0x24:Cat(Constant(0,2),tx_address), 0x28:Constant(DHCP_TX_BYTES,32), 0x2c:tx_destination,
                 0x34:Cat(mac[40:48],mac[32:40],mac[24:32],mac[16:24]),
                 0x38:Cat(mac[8:16],mac[:8]),
                 0x3c:Constant(preferred_ip,32), 0x40:self.state, 0x44:self.lease,
                 0x48:self.server, 0x50:receive_enable}
        read_cases = {offset//4:[response.eq(value),bus.ack.eq(1)] for offset,value in reads.items()}
        read_cases[0x1c//4] = If(pop,response.eq(rx.data),bus.ack.eq(1)).Else(bus.err.eq(1))
        read_cases["default"] = bus.err.eq(1)
        writes = {8:self.ip_address,12:self.netmask,16:self.gateway,
                  0x40:self.state,0x44:self.lease,0x48:self.server,0x50:receive_enable}
        write_cases = {offset//4:[value.eq(bus.dat_w),bus.ack.eq(1)] for offset,value in writes.items()}
        for offset,value in ((0x2c,tx_destination),):
            write_cases[offset//4] = If(~busy, value.eq(bus.dat_w),bus.ack.eq(1)).Else(bus.err.eq(1))
        write_cases[0x24//4] = If(~busy & (bus.dat_w <= ram_bytes - DHCP_TX_BYTES) & (bus.dat_w[:2] == 0),
                                 tx_address.eq(bus.dat_w[2:]),bus.ack.eq(1)).Else(bus.err.eq(1))
        # The fixed firmware packet size removes a length register and
        # variable-length range arithmetic. Retain the existing CSR write.
        write_cases[0x28//4] = If(~busy & (bus.dat_w == DHCP_TX_BYTES),
                                 bus.ack.eq(1)).Else(bus.err.eq(1))
        write_cases[0x20//4] = bus.ack.eq(1)
        write_cases[0x4c//4] = bus.ack.eq(1)
        # All requests are complete, word-aligned DHCP payloads in local SRAM.
        # Every accepted start address already fits one complete payload.
        valid_tx = ~busy
        self.comb += start.eq(request & bus.we & self.granted & (bus.sel == 15)
                             & (bus.adr[:10] == 0x30//4) & bus.dat_w[0] & valid_tx)
        write_cases[0x30//4] = If(start,bus.ack.eq(1)).Else(bus.err.eq(1))
        write_cases["default"] = bus.err.eq(1)
        self.sync += [bus.ack.eq(0),bus.err.eq(0),
            If(~(bus.cyc & bus.stb),seen.eq(0)).Elif(request,
                seen.eq(1),response.eq(0),
                If(bus.we,
                    If(self.granted & (bus.sel == 15),Case(bus.adr[:10],write_cases))
                    .Else(bus.err.eq(1))).Else(Case(bus.adr[:10],read_cases)))]

        if shared_readback is not None:
            shared_readback.attach(self)
