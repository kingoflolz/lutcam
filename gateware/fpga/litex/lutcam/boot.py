"""Ti60 remote reconfiguration and truthful, per-image identity registers.

The configuration block has a two-bit selector, not a runtime flash-address
register or a current-slot output. image_slot identifies this compiled image;
it is meaningful only with the validated bundle's fixed placement.
"""

from functools import reduce
from operator import or_
from types import SimpleNamespace

from migen import Cat, Case, Constant, If, Replicate, Signal
from migen.genlib.cdc import MultiReg
from litex.gen import LiteXModule
from litex.build.efinix.ifacewriter import InterfaceWriterBlock
from litex.soc.integration.export import MockCSR
from litex.soc.integration.soc import SoCCSRRegion
from litex.soc.interconnect import wishbone

from .dsp import add_dsp_counter


BOOT_BASE = 0x30026000
BOOT_MAGIC = 0x4c425331
BOOT_REGISTERS = (
    ("magic", 0x00, True), ("reserved0", 0x04, True),
    ("image_slot", 0x08, True), ("status", 0x0c, True),
    ("build_id0", 0x10, True), ("build_id1", 0x14, True),
    ("build_id2", 0x18, True), ("build_id3", 0x1c, True),
    ("target_slot", 0x20, False), ("command_id", 0x24, False),
    ("error", 0x28, True), ("pending_target", 0x2c, True),
    ("golden_status", 0x30, True),
)


class _RemoteUpdateBlock(InterfaceWriterBlock):
    def generate(self):
        # The pinned LiteX helper uses the signal's Python variable backtrace,
        # which collapses dynamically constructed pins to the same name.
        # Interface ports have explicit, stable Verilog names instead.
        properties = {"RECONFIG_EN": "1", "CLK_PIN": self["clock"],
                      "INVERT_CLK_EN": "0"}
        properties.update({name + "_PIN": pin.name_override
                           for name, pin in vars(self["pins"]).items()})
        return "\n".join(f'design.set_device_property("ru", "{key}", "{value}", "RU")'
                         for key, value in properties.items()) + "\n"


def request_remote_update(platform, clock):
    pins = SimpleNamespace(**{name: platform.add_iface_io("ota_cfg_" + name, width)
                             for name, width in
                             (("CBSEL", 2), ("CONFIG", 1), ("ENA", 1), ("ERROR", 1))})
    platform.toolchain.ifacewriter.blocks.append(_RemoteUpdateBlock({
        "type": "REMOTE_UPDATE", "name": "ota_cfg", "pins": pins,
        "clock": clock.name_override, "invert_clock": 0, "enable": 1,
    }))
    return pins


class BootControl(LiteXModule):
    """Nonblocking, duplicate-safe reboot request; serial flash must be idle.

    A request waits 100 ms before signalling the 25 MHz configuration domain.
    This permits host acknowledgement and keeps flash pins quiescent. New flash
    commands must be inhibited for the entire pending interval.
    A sequential ImageIdentity serves build-ID reads through its existing ROM;
    those reads wait for validation and take two extra clocks after acceptance.
    When shared_readback is provided, acknowledged responses use its common
    storage; inactive response data is unspecified. The default stays private.
    """

    def __init__(self, pins=None, *, image_slot=0, build_id="0" * 32,
                 identity=None, delay_cycles=12_000_000, ru_domain="cam_ref",
                 use_dsp_delay=False, shared_readback=None):
        if identity is None and image_slot not in range(4):
            raise ValueError("Ti60 supports image selectors 0..3")
        if len(build_id) != 32:
            raise ValueError("build_id must contain 32 hexadecimal digits")
        identity_bytes = bytes.fromhex(build_id)
        if len(identity_bytes) != 16 or delay_cycles < 1:
            raise ValueError("invalid identity or reboot delay")
        self.pins = pins or SimpleNamespace(CBSEL=Signal(2), CONFIG=Signal(),
                                            ENA=Signal(), ERROR=Signal())
        pins = self.pins
        self.bus = bus = wishbone.Interface(data_width=32, addressing="word")
        self.flash_busy = Signal()
        self.pending = Signal()
        self.ru_error = Signal()
        self.golden_status = Signal(32)
        self.auto_request = Signal()
        self.auto_slot = Signal(2)
        self.auto_ready = Signal()
        identity_valid = identity.valid if identity is not None else Constant(1, 1)
        if identity is not None:
            image_slot = identity.image_slot
        target = Signal(2)
        accepted = Signal(32)
        pending_target = Signal(2)
        error = Signal(2)
        delay = Signal(max=max(2, delay_cycles))
        trigger = Signal()
        self.specials += MultiReg(pins.ERROR, self.ru_error)
        self.comb += self.auto_ready.eq(~self.pending & ~self.flash_busy & identity_valid)

        # The selector is held stable throughout the delay and reconfiguration.
        ru_trigger = Signal()
        ru_target = Signal(2)
        self.specials += [MultiReg(trigger, ru_trigger, odomain=ru_domain),
                          MultiReg(pending_target, ru_target, odomain=ru_domain)]
        ru_count = Signal(2)
        getattr(self.sync, ru_domain).__iadd__([
            If(ru_trigger,
                pins.CBSEL.eq(ru_target), pins.ENA.eq(1),
                If(ru_count != 3, ru_count.eq(ru_count + 1)),
                If(ru_count == 3, pins.CONFIG.eq(1)),
            ).Else(pins.ENA.eq(0), pins.CONFIG.eq(0), ru_count.eq(0))
        ])
        if use_dsp_delay:
            # Preserve the exact saturating delay and reset behavior while
            # storing the counter in the native DSP feedback register.
            add_dsp_counter(self, delay,
                self.pending & ~trigger & (delay != delay_cycles - 1))
            self.sync += If(self.pending & ~trigger & (delay == delay_cycles - 1)
                           & ~self.flash_busy, trigger.eq(1))
        else:
            self.sync += If(self.pending & ~trigger,
                If(delay == delay_cycles - 1,
                    If(~self.flash_busy, trigger.eq(1))
                ).Else(delay.eq(delay + 1)))

        sequential_identity = (identity is not None
                               and getattr(identity, "sequential_build_id", False))
        state = Signal(3 if sequential_identity else 2)
        read_value = Signal(32) if shared_readback is None else shared_readback.word
        response_error = Signal()
        request = bus.cyc & bus.stb
        # Wait for withdrawal after ACK: a held classic request cannot commit
        # repeatedly. Independent Etherbone requests still complete promptly.
        self.comb += [bus.dat_r.eq(read_value),
                      bus.ack.eq(request & (state == 1) & ~response_error),
                      bus.err.eq(request & (state == 1) & response_error)]
        read_values = {
            0x00 // 4: BOOT_MAGIC,
            0x04 // 4: 0,
            0x08 // 4: image_slot,
            0x0c // 4: Cat(self.pending, self.ru_error, self.golden_status[:2]),
            0x20 // 4: target,
            0x24 // 4: accepted,
            0x28 // 4: error,
            0x2c // 4: pending_target,
            0x30 // 4: self.golden_status,
        }
        if not sequential_identity:
            for index in range(4):
                read_values[(0x10 + index * 4) // 4] = (
                    identity.build_id_words[index] if identity is not None else
                    int.from_bytes(identity_bytes[index * 4:index * 4 + 4], "big"))
        # Addresses are mutually exclusive. A masked OR shares decode terms
        # within four-word banks without changing the registered response.
        read_banks = {}
        for word, value in read_values.items():
            read_banks.setdefault(word >> 2, []).append(
                value & Replicate(bus.adr[:2] == (word & 3), 32))
        selected_read = Signal(32)
        self.comb += selected_read.eq(reduce(or_, (
            reduce(or_, values) & Replicate(bus.adr[2:10] == bank, 32)
            for bank, values in read_banks.items())))
        reads = {word: [] for word in read_values}
        reads["default"] = response_error.eq(1)
        writes = {
            0x20 // 4: If(self.pending, response_error.eq(1))
                .Elif(bus.dat_w > 3, response_error.eq(1), error.eq(1))
                .Else(target.eq(bus.dat_w)),
            0x24 // 4: If((bus.dat_w == accepted) & (bus.dat_w != 0),
                # A duplicated UDP request is an acknowledgement, never a new boot.
            ).Elif((bus.dat_w == 0) | self.pending,
                # accepted changes only when pending becomes permanently set.
                # Therefore pending rejects every non-replay later command,
                # including stale IDs, without another 32-bit comparison.
                response_error.eq(1), error.eq(3)
            ).Elif(self.flash_busy | ~identity_valid | self.auto_request,
                response_error.eq(1), error.eq(2)
            ).Else(accepted.eq(bus.dat_w), pending_target.eq(bus.dat_w[:2]),
                   self.pending.eq(1), error.eq(0)),
            "default": response_error.eq(1),
        }
        read_action = [read_value.eq(selected_read), Case(bus.adr[:10], reads)]
        if sequential_identity:
            # The accepted address owns this response until ACK or cancellation.
            # WAIT_READY permits early reads without disturbing validation;
            # WAIT_DATA consumes the ROM result only after its read edge.
            read_action = If(bus.adr[2:10] == 1,
                identity.build_id_index.eq(bus.adr[:2]), state.eq(3),
            ).Else(*read_action)
        transaction = If(state == 0,
            If(request,
                state.eq(1), response_error.eq(0), read_value.eq(0),
                If(bus.we,
                    If(bus.sel != 0xf, response_error.eq(1))
                    .Else(Case(bus.adr[:10], writes))
                ).Else(read_action)
            )
        ).Elif(state == 1, state.eq(2))
        if sequential_identity:
            transaction = transaction.Elif(state == 3,
                If(~request, state.eq(0)).Elif(identity.ready, state.eq(4))
            ).Elif(state == 4,
                If(~request, state.eq(0))
                .Elif(~identity.ready, state.eq(3))
                .Else(read_value.eq(identity.build_id_data), state.eq(1))
            )
        self.sync += [transaction.Elif(~request, state.eq(0)),
            If(self.auto_request & self.auto_ready,
                # delay is still at its reset value: it runs only after the
                # one request that can be accepted before reconfiguration.
                self.pending.eq(1), pending_target.eq(self.auto_slot), error.eq(0)),
        ]
        self.regions = {"boot": SoCCSRRegion(BOOT_BASE, 32, [
            MockCSR(name, 1, "ro" if ro else "rw", origin=BOOT_BASE + offset)
            for name, offset, ro in BOOT_REGISTERS])}

        if shared_readback is not None:
            shared_readback.attach(self)


class FlashAccessMux(LiteXModule):
    """Give the golden scanner exclusive flash access until its scan ends."""
    def __init__(self, flash_bus, scanner):
        self.bus = host = wishbone.Interface(data_width=32, addressing="word")
        self.comb += If(scanner.busy,
            *scanner.bus.connect(flash_bus),
            host.dat_r.eq(0), host.ack.eq(0), host.err.eq(host.cyc & host.stb)
        ).Else(
            *host.connect(flash_bus),
            scanner.bus.dat_r.eq(0), scanner.bus.ack.eq(0), scanner.bus.err.eq(0))
