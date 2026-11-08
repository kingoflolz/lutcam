"""Post-route patchable application identity, read from a dedicated BRAM ROM.

Efinity's BRAM Initial Content Updater changes this elaborated memory and then
regenerates the configuration checksums. A and B share one routed design; their
initialization tables differ in exactly one bit (word 2, bit 0).
"""

from functools import reduce
from operator import xor

from migen import Cat, If, Memory, Mux, Signal
from litex.gen import LiteXModule


IDENTITY_MEMORY = "ota_identity_rom"
IDENTITY_MAGIC = 0x4C434944
IDENTITY_DEPTH = 16
IDENTITY_FILL = (
    0xA536C97E, 0x5AC93681, 0x01234567, 0x89ABCDEF,
    0xFEDCBA98, 0x76543210, 0xC3D2E1F0, 0x3C2D1E0F,
)


def identity_words(build_id, slot="A"):
    """Return the 16-word table; only the slot bit changes between A and B.

    The XOR check excludes the slot word, so A/B differ by one initialization
    bit. Efinity's configuration CRC covers both the slot bit and this table.
    This small check detects a wrong identity table, not malicious firmware.
    """
    if isinstance(build_id, bytes):
        build_id = build_id.hex()
    if (not isinstance(build_id, str) or len(build_id) != 32
            or any(char not in "0123456789abcdefABCDEF" for char in build_id)):
        raise ValueError("build_id must contain exactly 128 bits as 32 hex digits")
    try:
        value = int(build_id, 16)
    except ValueError as error:
        raise ValueError("build_id must contain exactly 128 bits as 32 hex digits") from error
    if slot not in ("A", "B"):
        raise ValueError("application identity slot must be A or B")
    # Word 1 is reserved; retain its position and checksum coverage.
    words = [IDENTITY_MAGIC, 0, int(slot == "B")]
    words += [(value >> shift) & 0xFFFFFFFF for shift in (96, 64, 32, 0)]
    words += list(IDENTITY_FILL)
    words.append(reduce(xor, words[:2] + words[3:], 0))
    return words


class _IdentityMemory(Memory):
    def __init__(self, words):
        super().__init__(32, IDENTITY_DEPTH, init=words, name=IDENTITY_MEMORY)
        self.attr = {("syn_romstyle", "block_rom"), ("syn_ramstyle", "block_ram")}

    @staticmethod
    def emit_verilog(memory, namespace, add_data_file):
        return ('(* syn_romstyle = "block_rom", syn_ramstyle = "block_ram" *) '
                + Memory.emit_verilog(memory, namespace, add_data_file))


class ImageIdentity(LiteXModule):
    """Latch the immutable table after reset; invalid identity fails closed.

    ``image_slot`` is 1 for A, 2 for B, or 0 while invalid. ``build_id_words``
    exposes the four words in most-significant-first order. ``ready`` reports
    that the read finished; ``valid`` additionally requires a valid table.
    With sequential_build_id=True, build_id_index selects one word and
    build_id_data exposes the synchronous ROM response instead of retaining
    four words. Its consumer must wait for a ROM read edge with ready asserted
    and the selected index stable, then capture the response on the next edge.
    Slot/ready/valid remain latched. The default keeps the original build_id
    and build_id_words interface.
    The platform argument is accepted to match other board-level modules.
    """

    def get_memories(self):
        # LiteX otherwise exports every public Memory as a writable CSR RAM.
        # This ROM is a build artifact, never a host-writable peripheral.
        return []

    def __init__(self, platform=None, *, build_id, init_words=None,
                 sequential_build_id=False):
        words = identity_words(build_id) if init_words is None else list(init_words)
        if len(words) != IDENTITY_DEPTH or any(
                not isinstance(x, int) or not 0 <= x <= 0xFFFFFFFF for x in words):
            raise ValueError("identity table must contain sixteen 32-bit words")
        self.image_slot = Signal(2)
        self.sequential_build_id = bool(sequential_build_id)
        if sequential_build_id:
            self.build_id_index = Signal(2)
            self.build_id_data = Signal(32)
        else:
            self.build_id = Signal(128)
            self.build_id_words = [Signal(32, name=f"image_build_id_{i}") for i in range(4)]
        self.ready = Signal()
        self.valid = Signal()
        self.memory = _IdentityMemory(words)
        port = self.memory.get_port()
        self.specials += self.memory, port
        index = Signal(4)
        capture = Signal()
        bad = Signal()
        slot_bit = Signal()
        checksum = Signal(32)
        self.comb += [
            self.image_slot.eq(0),
            If(self.valid, self.image_slot.eq(1 + slot_bit)),
        ]
        if sequential_build_id:
            # After validation, the same synchronous ROM port serves host
            # reads. Consumers must wait one read edge after ready/address.
            self.comb += [port.adr.eq(Mux(self.ready, 3 + self.build_id_index, index)),
                          self.build_id_data.eq(port.dat_r)]
        else:
            self.comb += [port.adr.eq(index),
                          self.build_id.eq(Cat(*reversed(self.build_id_words)))]
        checks = [
            If(index == 0, If(port.dat_r != IDENTITY_MAGIC, bad.eq(1))),
            If(index == 1, If(port.dat_r != 0, bad.eq(1))),
            If(index == 2,
                slot_bit.eq(port.dat_r[0]),
                If(port.dat_r[1:] != 0, bad.eq(1)),
            ).Else(checksum.eq(checksum ^ port.dat_r)),
        ]
        if not sequential_build_id:
            checks += [If(index == i + 3, word.eq(port.dat_r))
                       for i, word in enumerate(self.build_id_words)]
        self.sync += If(~self.ready,
            capture.eq(~capture),
            If(capture,
                *checks,
                If(index == IDENTITY_DEPTH - 1,
                    self.ready.eq(1),
                    self.valid.eq(~bad & ((checksum ^ port.dat_r) == 0)),
                ).Else(index.eq(index + 1)),
            ),
        )
