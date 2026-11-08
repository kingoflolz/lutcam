"""Shared response storage for the qualified guarded main-image bus.

Only acknowledged read data is part of this interface. Peripheral response
histories are not retained while idle. Every attached leaf must capture its
response solely while its own decoded CYC/STB request remains asserted.
"""

from functools import reduce
from operator import or_

from migen import Case, If, Module, Signal
from migen.fhdl.decorators import ModuleTransformer
from migen.fhdl.structure import _Assign
from migen.fhdl.tools import list_targets
from migen.genlib.roundrobin import SP_WITHDRAW
from litex.soc.interconnect import wishbone

from .control import WishboneGuard


class _ResponseCapture(ModuleTransformer):
    """Move one response word's next-value logic to a combinational port.

    Keep the original conditional assignment priority within each leaf. The
    parent can OR these ports because decoded requests are mutually exclusive.
    Other sequential assignments, including side effects and ACK, stay local.
    """

    def __init__(self, word):
        self.word = word
        self.data = Signal(32)
        self.enable = Signal()
        self.assignments = 0

    def _split(self, statements, capture):
        result = []
        for statement in statements:
            if isinstance(statement, (list, tuple)):
                result.extend(self._split(statement, capture))
            elif isinstance(statement, _Assign):
                if statement.l is self.word:
                    if capture:
                        result += [self.data.eq(statement.r), self.enable.eq(1)]
                        self.assignments += 1
                elif self.word in list_targets([statement]):
                    raise ValueError("shared readback requires full-word assignments")
                elif not capture:
                    result.append(statement)
            elif isinstance(statement, If):
                yes = self._split(statement.t, capture)
                no = self._split(statement.f, capture)
                if yes or no:
                    result.append(If(statement.cond, *yes).Else(*no))
            elif isinstance(statement, Case):
                cases = {key: self._split(body, capture)
                         for key, body in statement.cases.items()}
                if any(cases.values()):
                    result.append(Case(statement.test, cases))
            else:
                raise TypeError("unsupported shared response statement: "
                                + type(statement).__name__)
        return result

    def transform_fragment(self, module, fragment):
        for domain, statements in fragment.sync.items():
            if domain != "sys" and self.word in list_targets(statements):
                raise ValueError("shared response captures must use the sys clock")
        statements = fragment.sync.get("sys", [])
        fragment.sync["sys"] = self._split(statements, False)
        fragment.comb += [self.data.eq(0), self.enable.eq(0)]
        fragment.comb += self._split(statements, True)
        if not self.assignments:
            raise ValueError("attached peripheral does not capture the shared word")


class SharedReadback(Module):
    """One response register for the guarded main-image register peripherals.

    Attach only IntegrationRegisters, BootControl, USBLinkDiagnostics,
    the two HistogramReaders, CameraSupervisor and NetworkControl. I2C uses its response as
    canceled-write scratch; flash can finish reads after withdrawal. Neither
    may join this group without a separate ownership redesign.

    Validate the finalized bus topology before elaboration. In particular,
    LiteX's registered CSR bridge can acknowledge a canceled request after a
    peer gains the bus. Its retained-data behavior is incompatible with this
    sharing, so builds with that bridge (including SDRAM mode) fail explicitly.
    The ordinary unregistered CSR bridge and cycle arbiter are qualified only
    with a 4096-cycle guard on every master and the 1,000,000-cycle global fallback.
    """

    def __init__(self):
        self.word = Signal(32, reset_less=True)
        self._captures = []
        self._members = set()
        self._validated = False

    def attach(self, module):
        if self.finalized or module.finalized:
            raise ValueError("shared readback must attach before finalization")
        if module in self._members:
            raise ValueError("peripheral already attached to shared readback")
        self._members.add(module)
        capture = _ResponseCapture(self.word)
        capture(module)
        self._captures.append(capture)

    def validate_topology(self, csr_bridge, interconnect, guards, *, decoder_registered,
                          interconnect_timeout_cycles):
        guards = tuple(guards)
        if (not isinstance(csr_bridge, wishbone.Wishbone2CSR)
                or set(csr_bridge.fsm.actions) != {"WRITE-READ", "ACK"}
                or csr_bridge.wishbone.data_width != 32
                or csr_bridge.csr.data_width != 32
                or csr_bridge.csr.alignment != 32):
            raise ValueError("shared readback requires the unregistered 32-bit CSR bridge")
        if (not isinstance(interconnect, wishbone.InterconnectShared)
                or interconnect.arbiter.rr.switch_policy != SP_WITHDRAW
                or len(interconnect.arbiter.rr.request) != len(guards)
                or not decoder_registered
                or not interconnect.decoder._fragment.sync.get("sys")):
            raise ValueError("shared readback requires the registered decoder and guarded cycle arbiter")
        if (len(guards) not in (2, 3)
                or any(not isinstance(guard, WishboneGuard)
                       or guard.timeout_cycles != 4096 for guard in guards)):
            raise ValueError("shared readback requires two or three 4096-cycle Wishbone hosts")
        if (interconnect_timeout_cycles != 1_000_000
                or not isinstance(getattr(interconnect, "timeout", None), wishbone.Timeout)):
            raise ValueError("shared readback requires the 1,000,000-cycle global timeout")
        self._validated = True

    def do_finalize(self):
        if not self._validated:
            raise ValueError("validate shared readback bus topology before elaboration")
        if not self._captures:
            raise ValueError("shared readback requires attached peripherals")
        # Inactive ports already drive zero. A priority mux or a second enable
        # mask here costs substantially more LUTs than the exclusive OR.
        self.sync += If(reduce(or_, (port.enable for port in self._captures)),
            self.word.eq(reduce(or_, (port.data for port in self._captures))))
