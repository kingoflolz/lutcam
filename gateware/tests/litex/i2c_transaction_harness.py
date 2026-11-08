"""Physical Wishbone/open-drain harness for current SharedCameraI2C classes.

This helper depends only on Migen/LiteX and the implementation supplied by its
caller. It neither imports frozen build artifacts nor changes production RTL.
Clock/reset ports are the actual sys domain, including native DSP instances.
"""

from pathlib import Path
import sys

from migen import Cat, ClockDomain, Constant, Module, Signal
from migen.fhdl.specials import Instance
from migen.fhdl.structure import _Slice
from migen.fhdl.tools import list_signals
from migen.fhdl.visit import NodeVisitor
from litex.soc.interconnect import wishbone


PRIVATE_NAMES = ("accepted_id", "done_id", "descriptor", "ingress_active",
                 "host_state", "host_lane")


class _PhysicalAddress:
    def __init__(self, signal, camera):
        self.signal = signal
        self.base = (0x30020000 + camera * 0x1000) // 4

    def eq(self, value):
        return self.signal.eq(self.base + value)


class _Aperture:
    """Python-only adapter accepted by the maintained I2C wire helpers."""

    def __init__(self, master, camera, core):
        self.master = master
        self._i2c_engine = core
        self.adr = _PhysicalAddress(master.adr, camera)

    def __getattr__(self, name):
        return getattr(self.master, name)


def _construct_core(implementation, options):
    cls = getattr(implementation, "SharedCameraI2C", implementation)
    constructor = cls.__init__.__code__
    captured = {}
    previous = sys.getprofile()

    def capture(frame, event, arg):
        if previous is not None:
            previous(frame, event, arg)
        if event == "return" and frame.f_code is constructor:
            # Tuple-created Signals need not receive Python local-name
            # backtraces. Capture their identities without rewriting source.
            captured.update({name: frame.f_locals[name] for name in PRIVATE_NAMES
                             if name in frame.f_locals})

    sys.setprofile(capture)
    try:
        core = cls(**options)
    finally:
        sys.setprofile(previous)
    signals = list_signals(core._fragment)
    private = {}
    for name in PRIVATE_NAMES:
        matches = [signal for signal in signals
                   if signal.backtrace and signal.backtrace[-1][0] == name]
        signal = matches[0] if len(matches) == 1 else captured.get(name)
        if not isinstance(signal, Signal) or signal not in signals:
            raise ValueError(f"Cannot identify unique I2C local signal {name!r}")
        if name in captured and signal is not captured[name]:
            raise ValueError(f"Ambiguous I2C local signal {name!r}")
        private[name] = signal
    return core, private


class _Assignments(NodeVisitor):
    def __init__(self):
        self.items = []

    def visit_Assign(self, node):
        self.items.append(node)


def _assignments(statements):
    visitor = _Assignments()
    visitor.visit(statements)
    return visitor.items


def _accepted_storage(core, accepted):
    """Resolve a real storage object before conversion, not a numeric name."""
    drivers = [assignment for assignment in _assignments(core._fragment.comb)
               if assignment.l is accepted]
    if not drivers:
        writes = [assignment for statements in core._fragment.sync.values()
                  for assignment in _assignments(statements)
                  if assignment.l is accepted]
        if not writes:
            raise ValueError("accepted_id has neither a register nor DSP driver")
        return {"kind": "fabric", "signal": accepted}
    if len(drivers) != 1:
        raise ValueError("accepted_id has multiple combinational drivers")
    dependencies = list_signals(drivers[0].r)
    matches = [special for special in core._fragment.specials
               if isinstance(special, Instance) and special.of == "EFX_DSP48"
               and special.get_io("O") in dependencies]
    if len(matches) != 1:
        raise ValueError("accepted_id does not resolve to one native DSP output")
    instance = matches[0]
    parameters = {item.name: getattr(item.value, "value", item.value)
                  for item in instance.items if isinstance(item, Instance.Parameter)}
    required = {"MODE": "NORMAL", "SIGNED": 0, "M_SEL": "P", "N_SEL": "C",
                "C_EXT": "ALIGN_LEFT", "W_SEL": "X", "W_REG": 1, "RST_SYNC": 1}
    if any(parameters.get(key) != value for key, value in required.items()):
        raise ValueError(f"Unexpected accepted_id DSP configuration: {parameters}")
    for key in ("A_REG", "B_REG", "C_REG", "P_REG", "OP_REG", "O_REG"):
        if parameters.get(key, 0) != 0:
            raise ValueError(f"accepted_id DSP has an unexpected {key} stage")
    for name, expected in (("B", 1), ("OP", 0), ("SHIFT_ENA", 0)):
        value = instance.get_io(name)
        if not isinstance(value, Constant) or value.value != expected:
            raise ValueError(f"accepted_id DSP requires constant {name}={expected}")
    if len(accepted) != 32 or len(instance.get_io("O")) != 48:
        raise ValueError("Expected 32-bit accepted ID in a 48-bit DSP W register")
    result = instance.get_io("O")

    def source_bits(value):
        if isinstance(value, Signal):
            return [(value, bit) for bit in range(len(value))]
        if isinstance(value, _Slice):
            return source_bits(value.value)[value.start:value.stop]
        if isinstance(value, Cat):
            return [bit for part in value.l for bit in source_bits(part)]
        raise ValueError("accepted_id DSP has an unexpected reconstruction expression")

    actual_bits = source_bits(drivers[0].r)
    expected_bits = list(range(19)) + list(range(30, 43))
    if len(actual_bits) != 32 or any(signal is not result or bit != expected
                                    for (signal, bit), expected
                                    in zip(actual_bits, expected_bits)):
        raise ValueError("accepted_id DSP does not use the packed 32-bit W layout")
    return {"kind": "dsp", "signal": accepted, "instance": instance,
            "result": result}


def pack_storage32(value):
    """Encode a 32-bit ID for the qualified A19/C18-aligned-left W register."""
    if not 0 <= value < (1 << 32):
        raise ValueError("ID must be an unsigned 32-bit integer")
    return (value & 0x7ffff) | (((value >> 19) & 0x1fff) << 30)


class Harness(Module):
    def __init__(self, implementation, **kwargs):
        options = dict(single_aperture=True, fast_low_cycles=6,
                       fast_high_cycles=4, stretch_timeout_cycles=240)
        options.update(kwargs)
        if options["single_aperture"] is not True:
            raise ValueError("This harness requires the physical single aperture")
        self.clock_domains.cd_sys = ClockDomain("sys")
        self.clock, self.reset = self.cd_sys.clk, self.cd_sys.rst
        core, self.private = _construct_core(implementation, options)
        self.submodules.core = core
        self.engine = core  # RegisterTarget and existing wire helper interface.
        core.reset = self.reset  # Alias only: never a ResetInserter reset net.
        self.master = self.bus = master = wishbone.Interface(
            data_width=32, addressing="word")
        self.submodules.decoder = wishbone.Decoder(master, [
            (lambda address: address[11:] == (0x30020000 >> 13), core.bus),
        ], register=True)
        core.buses = tuple(_Aperture(master, camera, core) for camera in range(2))
        self.target_scl_low = [Signal(name=f"target_scl_low{i}") for i in range(2)]
        self.target_sda_low = [Signal(name=f"target_sda_low{i}") for i in range(2)]
        for camera in range(2):
            scl, sda = core.scl[camera], core.sda[camera]
            self.comb += [
                scl.i.eq(~((scl.oe & ~scl.o) | self.target_scl_low[camera])),
                sda.i.eq(~((sda.oe & ~sda.o) | self.target_sda_low[camera])),
            ]
        self.inputs = {"sys_rst": self.reset}
        self.inputs.update(("wb_" + name, getattr(master, name))
                           for name in ("adr", "dat_w", "sel", "we", "cyc", "stb", "cti", "bte"))
        self.inputs.update((name, getattr(core, name)) for name in (
            "record_first", "record_last", "record_count", "record_source", "record_abort_sources"))
        for kind in ("scl", "sda"):
            self.inputs.update((f"target_{kind}_low{i}", getattr(self, f"target_{kind}_low")[i])
                               for i in range(2))
        self.outputs = {}

        def observe(name, source):
            signal = Signal(len(source), name=name)
            self.comb += signal.eq(source)
            self.outputs[name] = signal

        for name in ("ack", "err", "dat_r"):
            observe("wb_" + name, getattr(master, name))
        observe("busy", core.busy)
        for kind in ("scl", "sda"):
            for camera in range(2):
                observe(f"{kind}_oe{camera}", getattr(core, kind)[camera].oe)
        for name in ("done", "error", "completed"):
            observe("engine_" + name, getattr(core.engine, name))
        for name, signal in self.private.items():
            observe("obs_" + name, signal)
        for name, signal in self.inputs.items():
            signal.name_override = name
        for name, signal in self.outputs.items():
            signal.name_override = name
        self.clock.name_override = "sys_clk"
        self.storage = _accepted_storage(core, self.private["accepted_id"])
        self.options = options

    @property
    def ios(self):
        return {self.clock, *self.inputs.values(), *self.outputs.values()}


def generate(path, name, implementation, kwargs=None, emitter="migen"):
    """Emit one harness and return JSON-compatible ports and seed metadata.

    Seed lvalues are relative to the testbench's DUT instance. Deposits are
    permitted only during explicit idle preconditioning, with reset deasserted,
    ingress inactive, wire engine idle and capture disabled. Never force a live
    transaction. Read obs_accepted_id after a deposit to confirm reconstruction.
    """
    if emitter == "migen":
        from migen.fhdl import verilog
    elif emitter == "litex":
        from litex.gen.fhdl import verilog
    else:
        raise ValueError("emitter must be 'migen' or 'litex'")
    dut = Harness(implementation, **(kwargs or {}))
    converted = verilog.convert(dut, ios=dut.ios, name=name)
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(converted.main_source)
    for filename, content in converted.data_files.items():
        (destination.parent / filename).write_text(content)
    ns = converted.ns
    storage = dut.storage
    seed = {"kind": storage["kind"], "signal": ns.get_name(storage["signal"]),
            "guard_output": "obs_ingress_active"}
    if storage["kind"] == "dsp":
        instance_name = ns.get_name(storage["instance"])
        seed.update(lvalue=instance_name + ".genblk1.i_DSP48.W_pipe.O_r",
                    width=48, encoding="storage37_packed32", instance=instance_name,
                    result=ns.get_name(storage["result"]),
                    pack_expression="(id & 0x7ffff) | (((id >> 19) & 0x1fff) << 30)")
    else:
        seed.update(lvalue=seed["signal"], width=32, encoding="unsigned32")
    ports = lambda signals: {key: {"name": ns.get_name(signal), "width": len(signal)}
                             for key, signal in signals.items()}
    return {"path": str(destination.resolve()), "module": name, "emitter": emitter,
            "clock": {"name": ns.get_name(dut.clock), "width": 1},
            "inputs": ports(dut.inputs), "outputs": ports(dut.outputs),
            "accepted_id_seed": seed, "options": dut.options,
            "data_files": sorted(converted.data_files)}
