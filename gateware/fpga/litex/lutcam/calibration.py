"""Sequential calibration storage shared by the host and LFTag solver."""

from migen import If, Memory, Module, Mux, Signal
from migen.fhdl.specials import READ_FIRST


CONFIG_DEFAULTS = [0] * 16
CONFIG_DEFAULTS[1] = 1024
CONFIG_DEFAULTS[7] = (16 << 8) | 19


class _CalibrationMemory(Memory):
    def __init__(self):
        super().__init__(32, 16, init=CONFIG_DEFAULTS, name="calibration_values")
        self.attr = {("syn_ramstyle", "block_ram")}

    @staticmethod
    def emit_verilog(memory, namespace, add_data_file):
        return '(* syn_ramstyle = "block_ram" *) ' + Memory.emit_verilog(
            memory, namespace, add_data_file
        )


class CalibrationBank(Module):
    """One physical reader arbitrates host and sequential numeric requests.

    Each accepted read produces valid/data on the following edge. Host reads
    win simultaneous requests; writes are already guarded by paused-state CSR
    decode. Reset restores defaults in sixteen clocks, during which ready is
    low. The solver must honor ready and never backpressure a read response.
    """

    def get_memories(self):
        # This RAM is accessed through the guarded camera CSR words only.
        return []

    def __init__(self):
        self.host_address = Signal(4)
        self.host_read = Signal()
        self.host_write = Signal()
        self.host_write_data = Signal(32)
        self.host_ready = Signal()
        self.host_response_valid = Signal()
        self.host_read_data = Signal(32)
        self.cfg_req_addr = Signal(4)
        self.cfg_req_valid = Signal()
        self.cfg_req_ready = Signal()
        self.cfg_rsp_valid = Signal()
        self.cfg_rsp_data = Signal(32)

        memory = _CalibrationMemory()
        write = memory.get_port(write_capable=True, mode=READ_FIRST)
        read = memory.get_port(has_re=True, mode=READ_FIRST)
        self.specials += memory, write, read
        self.memory = memory
        initializing = Signal(reset=1)
        init_address = Signal(4)
        host_accept = self.host_read & self.host_ready
        solver_accept = self.cfg_req_valid & self.cfg_req_ready
        self.comb += [
            self.host_ready.eq(~initializing),
            self.cfg_req_ready.eq(~initializing & ~self.host_read),
            read.adr.eq(Mux(self.host_read, self.host_address, self.cfg_req_addr)),
            read.re.eq(host_accept | solver_accept),
            self.host_read_data.eq(read.dat_r),
            self.cfg_rsp_data.eq(read.dat_r),
            write.adr.eq(Mux(initializing, init_address, self.host_address)),
            write.we.eq(initializing | (self.host_write & self.host_ready)),
            write.dat_w.eq(
                Mux(
                    initializing,
                    Mux(init_address == 7, CONFIG_DEFAULTS[7],
                        Mux(init_address == 1, CONFIG_DEFAULTS[1], 0)),
                    self.host_write_data,
                )
            ),
        ]
        self.sync += [
            self.host_response_valid.eq(host_accept),
            self.cfg_rsp_valid.eq(solver_accept),
            If(
                initializing,
                init_address.eq(init_address + 1),
                If(init_address == 15, initializing.eq(0)),
            ),
        ]
