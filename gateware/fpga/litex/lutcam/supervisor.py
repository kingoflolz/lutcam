"""Minimal flash-loaded PicoRV32 and bounded camera-supervisor mailbox."""
from pathlib import Path

from migen import (Cat, Case, ClockSignal, Constant, FSM, If, Instance, Memory,
                   Module, Mux, NextState, NextValue, Replicate, ResetSignal, Signal)
from litex.soc.interconnect import wishbone

SUPERVISOR_BASE = 0x30028000
SUPERVISOR_MAGIC = 0x4c435256
from .boot_reader import (FirmwareLoader, FIRMWARE_OFFSET, FIRMWARE_MAGIC,
                          RAM_BYTES, STACK_BYTES, MAILBOX_BASE)


class SupervisorRam(Module):
    """One word-wide SRAM shared by the core, boot loader and packet reader.

    Native byte writes avoid read/modify/write state and data merging. Packet
    reads and host mailbox accesses briefly arbitrate the same port; a stalled
    UDP transmitter never reserves the SRAM.
    """
    def __init__(self, size=RAM_BYTES):
        from migen import Record
        self.valid, self.ready = Signal(), Signal()
        self.address, self.wdata, self.rdata = Signal(32), Signal(32), Signal(32)
        self.strobes = Signal(4)
        self.load_busy, self.load_write = Signal(), Signal()
        self.load_address = Signal(max=size//4)
        self.load_data = Signal(32)
        self.dma = dma = Record([("request",1), ("ready",1), ("ack",1),
                                ("address",len(self.load_address)), ("data",32)])
        self.host = host = Record([("request",1), ("we",1), ("ack",1),
                                  ("address",len(self.load_address)), ("wdata",32), ("rdata",32)])
        self.memory = memory = Memory(32, size//4, name="supervisor_sram")
        port = memory.get_port(write_capable=True, we_granularity=8, has_re=True)
        self.specials += memory, port
        IDLE, ACK = range(2)
        phase = Signal()
        accept = self.valid & (phase == IDLE) & ~dma.request & ~host.request & ~self.load_busy
        fetch = dma.request & dma.ready
        host_grant = host.request & ~host.ack & (phase == IDLE) & ~self.load_busy
        self.comb += [
            dma.ready.eq((phase == IDLE) & ~self.load_busy & ~host.request), dma.data.eq(port.dat_r),
            host.rdata.eq(port.dat_r),
            self.ready.eq(phase == ACK), self.rdata.eq(port.dat_r),
            port.adr.eq(Mux(self.load_busy, self.load_address,
                           Mux(host_grant, host.address, Mux(fetch, dma.address, self.address[2:])))),
            port.re.eq(fetch | (host_grant & ~host.we) | (accept & (self.strobes == 0))),
            port.we.eq(Mux(self.load_busy, Replicate(self.load_write, 4),
                           Mux(host_grant, Replicate(host.we, 4), self.strobes & Replicate(accept, 4)))),
            port.dat_w.eq(Mux(self.load_busy, self.load_data, Mux(host_grant, host.wdata, self.wdata))),
        ]
        self.sync += [dma.ack.eq(fetch), host.ack.eq(host_grant),
            If(accept, phase.eq(ACK)).Elif(phase == ACK, phase.eq(IDLE))]


class CameraSupervisor(Module):
    """4 KiB local SRAM, native core bus, one guarded system-bus master.

    Mailbox command arguments/opcode precede the sequence commit. Firmware
    publishes status before acknowledging that sequence. Only the CPU may
    write result registers or its trusted I2C-record length.
    """
    def __init__(self, platform=None, *, shared_readback=None):
        self.submodules.loader = loader = FirmwareLoader()
        self.master = master = wishbone.Interface(data_width=32, addressing="word")
        self.bus = bus = wishbone.Interface(data_width=32, addressing="word")
        response = Signal(32) if shared_readback is None else shared_readback.word
        self.granted, self.guard_fault = Signal(), Signal()
        self.timer = Signal(32)
        self.trap, self.bus_fault = Signal(), Signal()
        self.record_first, self.record_last, self.record_abort = Signal(), Signal(), Signal()
        self.record_count, self.record_source = Signal(9), Signal(2)
        if platform is not None:
            platform.add_source(str(Path(__file__).resolve().parents[3] / "vendor/picorv32/picorv32.v"))
        valid, ready, instr = Signal(), Signal(), Signal()
        address, wdata, rdata = Signal(32), Signal(32), Signal(32)
        strobes = Signal(4)
        enabled = Signal()
        local, peripheral = Signal(), Signal()
        self.submodules.ram = ram = SupervisorRam()
        self.packet_dma = ram.dma
        self.comb += [enabled.eq(loader.done & ~self.bus_fault), local.eq(address < RAM_BYTES),
            peripheral.eq(address[18:] == (0x30000000 >> 18)),
            ram.valid.eq(enabled & valid & local), ram.address.eq(address),
            ram.wdata.eq(wdata), ram.strobes.eq(strobes),
            ram.load_busy.eq(loader.busy), ram.load_write.eq(loader.write),
            ram.load_address.eq(loader.address), ram.load_data.eq(loader.data),
            # The supervisor accesses only the 256 KiB peripheral aperture.
            # Check the prefix once, then keep it constant through the guard.
            master.adr.eq(Cat(address[2:18], Constant(0x30000000 >> 18, 14))), master.dat_w.eq(wdata), master.sel.eq(Mux(strobes != 0, strobes, 15)),
            master.we.eq(strobes != 0),
            master.cyc.eq(enabled & valid & peripheral), master.stb.eq(master.cyc),
            ready.eq(ram.ready | master.ack), rdata.eq(Mux(local, ram.rdata, master.dat_r)),
            self.record_source.eq(3), self.record_abort.eq(self.guard_fault),
            self.record_first.eq(address[:12] == 0x80),
            self.record_last.eq(address[:12] == Mux(self.record_count == 3, 0x88, 0x8c)),
        ]
        self.sync += If(self.guard_fault | (enabled & valid & ~local & ~peripheral), self.bus_fault.eq(1))
        self.specials += Instance("picorv32",
            p_ENABLE_COUNTERS=0, p_ENABLE_COUNTERS64=0, p_ENABLE_REGS_16_31=0, p_ENABLE_REGS_DUALPORT=0,
            p_COMPRESSED_ISA=1, p_ENABLE_MUL=0, p_ENABLE_DIV=0,
            p_ENABLE_IRQ=0, p_ENABLE_PCPI=0, p_BARREL_SHIFTER=0,
            p_PROGADDR_WIDTH=12, p_STACKADDR_WIDTH=13, p_FIXED_GP=2032, p_ZERO_REGS=0xd1, p_PROGADDR_RESET=0, p_STACKADDR=RAM_BYTES,
            p_TWO_STAGE_SHIFT=0,
            i_clk=ClockSignal(), i_resetn=enabled & ~ResetSignal(), o_trap=self.trap,
            o_mem_valid=valid, o_mem_instr=instr, i_mem_ready=ready,
            o_mem_addr=address, o_mem_wdata=wdata, o_mem_wstrb=strobes, i_mem_rdata=rdata,
            i_irq=0, i_pcpi_wr=0, i_pcpi_rd=0, i_pcpi_wait=0, i_pcpi_ready=0)
        # Firmware and hosts share 14 words in the existing SRAM. Only
        # command sequence/opcode/arguments are writable by external hosts.
        count = Signal(3, reset=4)
        self.comb += self.record_count.eq(count)
        seen, request, mailbox, allowed = Signal(), Signal(), Signal(), Signal()
        ack, error = Signal(), Signal()
        self.comb += [
            request.eq(bus.cyc & bus.stb & ~seen),
            mailbox.eq((bus.adr[:10] >= 2) & (bus.adr[:10] < 16)),
            allowed.eq(~loader.busy & (~bus.we | ((bus.sel == 15) &
                (self.granted | ((bus.adr[:10] >= 4) & (bus.adr[:10] <= 7)))))),
            ram.host.request.eq(request & mailbox & allowed), ram.host.we.eq(bus.we),
            ram.host.address.eq((MAILBOX_BASE//4 - 2) | bus.adr[:4]),
            ram.host.wdata.eq(bus.dat_w),
            bus.ack.eq(ack | ram.host.ack), bus.err.eq(error),
            bus.dat_r.eq(Mux(mailbox & ~loader.busy, ram.host.rdata, response)),
        ]
        values = {0: Constant(SUPERVISOR_MAGIC, 32),
                  4: Cat(loader.done, loader.busy, self.trap, self.bus_fault, Constant(0, 4), loader.error),
                  0x40:self.timer, 0x44:count, 0x48:loader.length, 0x4c:loader.expected_crc}
        read_cases = {offset//4: [response.eq(value), ack.eq(1)] for offset, value in values.items()}
        read_cases["default"] = error.eq(1)
        self.sync += [ack.eq(0), error.eq(0),
            If(~(bus.cyc & bus.stb), seen.eq(0)).Elif(request,
                If(mailbox,
                    If(loader.busy & ~bus.we, response.eq(0), ack.eq(1), seen.eq(1))
                    .Elif(~allowed, error.eq(1), seen.eq(1)).Elif(ram.host.ack, seen.eq(1)))
                .Else(seen.eq(1),
                    If(bus.we,
                        If(self.granted & (bus.sel == 15) & (bus.adr[:10] == 0x44//4)
                           & ((bus.dat_w == 3) | (bus.dat_w == 4)),
                           count.eq(bus.dat_w), ack.eq(1)).Else(error.eq(1)))
                    .Else(Case(bus.adr[:10], read_cases))))]
        if shared_readback is not None:
            shared_readback.attach(self)
