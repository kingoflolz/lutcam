"""Pin-level SPI NOR model and bounded Wishbone transfers for simulations."""

from migen.sim import passive


def transfer(bus, address, data=None, *, sel=15, idle=True, limit=16, latencies=None):
    """A bounded classic transfer, optionally keeping CYC/STB asserted."""
    yield bus.adr.eq(address // 4)
    yield bus.dat_w.eq(0 if data is None else data)
    yield bus.sel.eq(sel)
    yield bus.we.eq(data is not None)
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)
    yield
    for elapsed in range(1, limit + 1):
        ack, error = (yield bus.ack), (yield bus.err)
        assert not (ack and error), "Wishbone ACK and ERR asserted together"
        if ack or error:
            result = (yield bus.dat_r), error
            if latencies is not None:
                latencies.append(elapsed)
            if idle:
                yield bus.cyc.eq(0)
                yield bus.stb.eq(0)
                yield
            return result
        yield
    raise AssertionError("flash register transaction did not terminate promptly")


class SpiNor:
    """Sparse, mode-0, three-byte-address GD25LQ128 behavioral model."""

    def __init__(self, pads, *, refuse_wren=False, refuse_wren_number=None,
                 stuck_busy=False, busy_polls=2, miso_delay_model_ticks=0):
        self.pads = pads
        self.memory = {}
        self.uid = bytes(range(16))
        self.transactions = []
        self.wel = False
        self.refuse_wren = refuse_wren
        self.refuse_wren_number = refuse_wren_number
        self.wren_count = 0
        self.stuck_busy = stuck_busy
        self.busy_polls = busy_polls
        self.remaining_busy_polls = 0
        self.miso_delay_model_ticks = miso_delay_model_ticks
        self.current = []
        self.response = 0

    def status(self):
        return (int(self.wel) << 1) | int(self.stuck_busy or self.remaining_busy_polls > 0)

    def response_byte(self):
        if not self.current:
            return 0
        command = self.current[0]
        if command == 0x05:
            return self.status()
        if command == 0x9F:
            index = len(self.current) - 1
            return (0xC8, 0x60, 0x18)[index] if index < 3 else 0xFF
        if command == 0x4B and len(self.current) >= 5:
            return self.uid[(len(self.current) - 5) % 16]
        if command == 0x03 and len(self.current) >= 4:
            address = int.from_bytes(bytes(self.current[1:4]), "big")
            return self.memory.get(address + len(self.current) - 4, 0xFF)
        return 0

    def finish(self):
        transaction = bytes(self.current)
        assert transaction, "empty SPI transaction"
        self.transactions.append(transaction)
        command = transaction[0]
        assert command in (0x03, 0x05, 0x06, 0x02, 0x20, 0xD8, 0x9F, 0x4B), (
            f"unexpected/destructive SPI opcode {command:#04x}"
        )
        if command == 0x05:
            assert len(transaction) >= 2, "RDSR did not clock a status byte"
            if self.remaining_busy_polls:
                self.remaining_busy_polls -= 1
        elif command == 0x06:
            assert len(transaction) == 1, "WREN must contain only its opcode"
            assert not self.status() & 1, "WREN issued while flash busy"
            self.wren_count += 1
            self.wel = not self.refuse_wren and self.wren_count != self.refuse_wren_number
        elif command in (0x02, 0x20, 0xD8):
            assert self.wel, "mutation issued without WEL"
            assert not self.status() & 1, "mutation issued while flash busy"
            assert len(transaction) >= 4, "missing mutation address"
            address = int.from_bytes(transaction[1:4], "big")
            if command == 0x02:
                data = transaction[4:]
                assert 1 <= len(data) <= 256
                assert (address & 255) + len(data) <= 256, "page program wraps"
                for index, value in enumerate(data):
                    self.memory[address + index] = self.memory.get(address + index, 0xFF) & value
            else:
                assert len(transaction) == 4, "erase unexpectedly carried data"
                size = 4096 if command == 0x20 else 65536
                assert address % size == 0, "unaligned erase issued"
                for key in list(self.memory):
                    if address <= key < address + size:
                        del self.memory[key]
            self.wel = False
            self.remaining_busy_polls = self.busy_polls

    @passive
    def process(self):
        previous_cs, previous_clk = 1, 0
        bits, received = 0, 0
        cycle = 0
        pending_miso = []
        while True:
            cs, clk, mosi = (yield self.pads.cs_n), (yield self.pads.clk), (yield self.pads.mosi)
            if previous_cs and not cs:
                assert not clk, "mode-0 CS asserted while SCK high"
                self.current = []
                bits = received = 0
                self.response = 0
                pending_miso.clear()
                yield self.pads.miso.eq(0)
            if not cs:
                assert (yield self.pads.wp) == 1, "WP# was asserted"
                assert (yield self.pads.hold) == 1, "HOLD# was asserted"
                if not previous_clk and clk:
                    received = (received << 1) | mosi
                    bits += 1
                    if bits == 8:
                        self.current.append(received)
                        bits = received = 0
                if previous_clk and not clk:
                    if bits == 0:
                        self.response = self.response_byte()
                    # Delay the flash's physical output transition after
                    # SCK falls, independently of the controller's logic.
                    pending_miso.append((cycle + self.miso_delay_model_ticks,
                                         (self.response >> (7 - bits)) & 1))
            if not previous_cs and cs:
                assert not clk, "mode-0 CS released while SCK high"
                assert bits == 0, "CS released during a byte"
                self.finish()
                pending_miso.clear()
                yield self.pads.miso.eq(0)
            while pending_miso and pending_miso[0][0] <= cycle:
                _, value = pending_miso.pop(0)
                yield self.pads.miso.eq(value)
            previous_cs, previous_clk = cs, clk
            cycle += 1
            yield
