#!/usr/bin/env python3
"""Find event-constrained redundant LUT inputs in a Yosys JSON netlist."""

from __future__ import annotations

import argparse
import dataclasses
import json
import re
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

from z3 import And, Bool, BoolRef, BoolVal, Not, Or, Solver, is_false, is_true, sat, unknown, unsat


EVENT_STAGES = ("s1", "s2", "s3", "s4", "s4b", "s5")
CONST_BITS = {"0": False, "1": True, "x": None, "z": None}


@dataclasses.dataclass(frozen=True)
class LutCell:
    name: str
    width: int
    table: str
    inputs: tuple[Any, ...]
    output: Any
    src: str


@dataclasses.dataclass
class RedundantPin:
    cell: LutCell
    pin: int
    raw_dependency_rows: int
    raw_dependency_guards: list[str]
    invariants: list[str]
    status: str
    elapsed_ms: float


def bit_is_const(bit: Any) -> bool:
    return isinstance(bit, str) and bit in CONST_BITS


def const_value(bit: Any) -> bool | None:
    return CONST_BITS[bit] if bit_is_const(bit) else None


def safe_name(value: Any) -> str:
    return re.sub(r"[^a-zA-Z0-9_]+", "_", str(value))


def lut_eval(table: str, values: list[bool]) -> bool:
    idx = 0
    for i, value in enumerate(values):
        if value:
            idx |= 1 << i
    return table[idx] == "1"


def raw_dependency(cell: LutCell, pin: int) -> tuple[bool, int, list[str]]:
    other_pins = [i for i in range(cell.width) if i != pin]
    rows: list[dict[int, bool]] = []
    for mask in range(1 << len(other_pins)):
        base = [False] * cell.width
        row: dict[int, bool] = {}
        for j, other in enumerate(other_pins):
            value = bool(mask & (1 << j))
            base[other] = value
            row[other] = value
        lo = list(base)
        hi = list(base)
        lo[pin] = False
        hi[pin] = True
        if lut_eval(cell.table, lo) != lut_eval(cell.table, hi):
            rows.append(row)

    if not rows:
        return False, 0, []

    guards: list[str] = []
    for other in other_pins:
        vals = {row[other] for row in rows}
        if len(vals) == 1:
            value = next(iter(vals))
            guards.append(f"raw-dep only when pin{other}={int(value)}")
    return True, len(rows), guards


class Netlist:
    def __init__(self, json_path: Path, top: str):
        data = json.loads(json_path.read_text())
        self.json_path = json_path
        self.module = data["modules"][top]
        self.cells = self.module["cells"]
        self.netnames = self.module.get("netnames", {})
        self.bit_to_names: dict[Any, list[str]] = defaultdict(list)
        self.bit_to_srcs: dict[Any, list[str]] = defaultdict(list)
        for name, net in self.netnames.items():
            for bit in net.get("bits", []):
                self.bit_to_names[bit].append(name)
                src = net.get("attributes", {}).get("src")
                if src:
                    self.bit_to_srcs[bit].append(src)

        self.luts: dict[str, LutCell] = {}
        self.driver_by_bit: dict[Any, LutCell] = {}
        for name, cell in self.cells.items():
            if cell.get("type") != "$lut":
                continue
            params = cell.get("parameters", {})
            width = int(params["WIDTH"], 2)
            table = params["LUT"]
            conns = cell.get("connections", {})
            output = conns["Y"][0]
            lut = LutCell(
                name=name,
                width=width,
                table=table,
                inputs=tuple(conns["A"]),
                output=output,
                src=cell.get("attributes", {}).get("src", ""),
            )
            self.luts[name] = lut
            self.driver_by_bit[output] = lut

    def bit_label(self, bit: Any, max_names: int = 3) -> str:
        if bit_is_const(bit):
            return str(bit)
        names = self.bit_to_names.get(bit, [])
        if not names:
            return f"bit{bit}"
        public = [name for name in names if "$abc" not in name and "$auto" not in name]
        chosen = public or names
        label = ", ".join(chosen[:max_names])
        if len(chosen) > max_names:
            label += ", ..."
        return label

    def bit_src(self, bit: Any) -> str:
        srcs = self.bit_to_srcs.get(bit, [])
        return srcs[0] if srcs else ""

    def bit_matches(self, bit: Any, patterns: list[re.Pattern[str]]) -> bool:
        if not patterns or bit_is_const(bit):
            return False
        names = self.bit_to_names.get(bit, [])
        return any(p.search(name) for p in patterns for name in names)

    def cell_matches(self, cell: LutCell, patterns: list[re.Pattern[str]]) -> bool:
        if not patterns:
            return False
        haystack = [cell.name, self.bit_label(cell.output, max_names=20)]
        haystack.extend(self.bit_label(bit, max_names=20) for bit in cell.inputs)
        return any(p.search(text) for p in patterns for text in haystack)

    def endpoint_bits(self, patterns: list[re.Pattern[str]], include_ff_di: bool) -> set[Any]:
        bits: set[Any] = set()
        for name, net in self.netnames.items():
            if any(p.search(name) for p in patterns):
                bits.update(net.get("bits", []))

        if include_ff_di:
            for cell in self.cells.values():
                if cell.get("type") != "TRELLIS_FF":
                    continue
                conns = cell.get("connections", {})
                q_bits = conns.get("Q", [])
                if any(self.bit_matches(bit, patterns) for bit in q_bits):
                    bits.update(conns.get("DI", []))
        return bits

    def lut_cone(self, endpoints: set[Any]) -> set[str]:
        seen: set[str] = set()

        def walk_bit(bit: Any) -> None:
            lut = self.driver_by_bit.get(bit)
            if lut is None or lut.name in seen:
                return
            seen.add(lut.name)
            for inp in lut.inputs:
                walk_bit(inp)

        for bit in endpoints:
            walk_bit(bit)
        return seen


class Z3Checker:
    def __init__(self, netlist: Netlist, event_stages: tuple[str, ...], timeout_ms: int):
        self.nl = netlist
        self.event_stages = event_stages
        self.timeout_ms = timeout_ms
        self.expr_cache: dict[tuple[int, Any], BoolRef] = {}
        self.root_cache: dict[tuple[int, Any], BoolRef] = {}
        self.lut_cache: dict[tuple[int, str], BoolRef] = {}

    def root(self, copy: int, bit: Any) -> BoolRef:
        key = (copy, bit)
        if key not in self.root_cache:
            self.root_cache[key] = Bool(f"r{copy}_{safe_name(bit)}")
        return self.root_cache[key]

    def expr(self, copy: int, bit: Any) -> BoolRef:
        if bit_is_const(bit):
            value = const_value(bit)
            return BoolVal(False if value is None else value)
        key = (copy, bit)
        if key in self.expr_cache:
            return self.expr_cache[key]
        lut = self.nl.driver_by_bit.get(bit)
        if lut is None:
            expr = self.root(copy, bit)
        else:
            expr = self.lut_expr(copy, lut)
        self.expr_cache[key] = expr
        return expr

    def lut_expr(self, copy: int, cell: LutCell) -> BoolRef:
        key = (copy, cell.name)
        if key in self.lut_cache:
            return self.lut_cache[key]

        inputs = [self.expr(copy, bit) for bit in cell.inputs]
        terms: list[BoolRef] = []
        for idx, char in enumerate(cell.table):
            if char != "1":
                continue
            factors = []
            for i, inp in enumerate(inputs):
                factors.append(inp if idx & (1 << i) else Not(inp))
            terms.append(And(*factors) if factors else BoolVal(True))
        expr = Or(*terms) if terms else BoolVal(False)
        self.lut_cache[key] = expr
        return expr

    def named_bit(self, name: str) -> Any | None:
        net = self.nl.netnames.get(name)
        if not net:
            return None
        bits = net.get("bits", [])
        return bits[0] if bits else None

    def named_expr(self, copy: int, name: str) -> BoolRef | None:
        bit = self.named_bit(name)
        return None if bit is None else self.expr(copy, bit)

    def add_eq_if_present(self, solver: Solver, copy: int, name: str, value: BoolRef) -> None:
        expr = self.named_expr(copy, name)
        if expr is not None:
            solver.add(expr == value)

    def add_event_constraints(self, solver: Solver, copy: int) -> None:
        for stage in self.event_stages:
            prefix = f"core.{stage}_"
            v = Bool(f"c{copy}_{stage}_V")
            a = Bool(f"c{copy}_{stage}_A")
            b0 = Bool(f"c{copy}_{stage}_B0")
            b1 = Bool(f"c{copy}_{stage}_B1")
            c = Bool(f"c{copy}_{stage}_C")
            d0 = Bool(f"c{copy}_{stage}_D0")
            d1 = Bool(f"c{copy}_{stage}_D1")
            self.add_eq_if_present(solver, copy, prefix + "valid", v)
            self.add_eq_if_present(solver, copy, prefix + "d0", d0)
            self.add_eq_if_present(solver, copy, prefix + "d1", d1)
            self.add_eq_if_present(solver, copy, prefix + "Ec0", And(v, c, Not(d0)))
            self.add_eq_if_present(solver, copy, prefix + "Ec1", And(v, d0, Not(d1)))
            self.add_eq_if_present(solver, copy, prefix + "Ep0", And(v, a, Not(b0)))
            self.add_eq_if_present(solver, copy, prefix + "Ep1", And(v, b0, Not(b1)))
            self.add_eq_if_present(solver, copy, prefix + "O0", And(v, b0, d0, Or(Not(a), Not(c))))
            self.add_eq_if_present(solver, copy, prefix + "O1", And(v, b1, d1, Or(Not(b0), Not(d0))))
            self.add_eq_if_present(solver, copy, prefix + "Sp0", And(v, Not(a), b0))
            self.add_eq_if_present(solver, copy, prefix + "Sp1", And(v, Not(b0), b1))
            self.add_eq_if_present(solver, copy, prefix + "Sc0", And(v, Not(c), d0))
            self.add_eq_if_present(solver, copy, prefix + "Sc1", And(v, Not(d0), d1))
            self.add_eq_if_present(solver, copy, prefix + "pf_valid", And(v, Or(And(Not(a), b0), And(Not(b0), b1))))

    def base_solver(self) -> Solver:
        solver = Solver()
        solver.set(timeout=self.timeout_ms)
        self.add_event_constraints(solver, 0)
        self.add_event_constraints(solver, 1)
        return solver

    def constrained_pin_status(self, cell: LutCell, pin: int) -> tuple[Any, float]:
        solver = self.base_solver()
        inputs0 = [self.expr(0, bit) for bit in cell.inputs]
        inputs1 = [self.expr(1, bit) for bit in cell.inputs]
        for idx, (left, right) in enumerate(zip(inputs0, inputs1)):
            solver.add(left != right if idx == pin else left == right)
        solver.add(self.expr(0, cell.output) != self.expr(1, cell.output))
        started = time.monotonic()
        result = solver.check()
        return result, (time.monotonic() - started) * 1000.0

    def condition_satisfiable(self, constraints: list[BoolRef]) -> bool:
        solver = self.base_solver()
        solver.add(*constraints)
        return solver.check() == sat

    def invariant_unsat(self, constraints: list[BoolRef]) -> bool:
        solver = self.base_solver()
        solver.add(*constraints)
        return solver.check() == unsat

    def explain_pin(self, cell: LutCell, pin: int, raw_guards: list[str]) -> list[str]:
        inputs = [self.expr(0, bit) for bit in cell.inputs]
        cand = inputs[pin]
        explanations: list[str] = []

        for other, other_expr in enumerate(inputs):
            if other == pin:
                continue
            if self.condition_satisfiable([cand != other_expr]) is False:
                explanations.append(f"pin{pin} == pin{other}")

        for value in (False, True):
            if self.condition_satisfiable([cand != BoolVal(value)]) is False:
                explanations.append(f"pin{pin} == {int(value)}")

        guard_candidates: list[tuple[int, bool]] = []
        for guard in raw_guards:
            match = re.search(r"pin(\d+)=(0|1)", guard)
            if match:
                guard_candidates.append((int(match.group(1)), match.group(2) == "1"))

        for guard_pin, guard_value in guard_candidates:
            guard_expr = inputs[guard_pin] if guard_value else Not(inputs[guard_pin])
            if not self.condition_satisfiable([guard_expr]):
                explanations.append(f"pin{guard_pin}={int(guard_value)} unreachable")
                continue
            for other, other_expr in enumerate(inputs):
                if other == pin:
                    continue
                if self.invariant_unsat([guard_expr, cand != other_expr]):
                    explanations.append(f"if pin{guard_pin}={int(guard_value)} then pin{pin} == pin{other}")
            for value in (False, True):
                if self.invariant_unsat([guard_expr, cand != BoolVal(value)]):
                    explanations.append(f"if pin{guard_pin}={int(guard_value)} then pin{pin} == {int(value)}")

        return sorted(set(explanations))


def analyze(netlist: Netlist, lut_names: list[str], args: argparse.Namespace) -> list[RedundantPin]:
    checker = Z3Checker(netlist, tuple(args.event_stage), args.timeout_ms)
    results: list[RedundantPin] = []
    for index, name in enumerate(lut_names):
        if args.max_luts and index >= args.max_luts:
            break
        cell = netlist.luts[name]
        if cell.width < args.min_width:
            continue
        for pin, bit in enumerate(cell.inputs):
            if bit_is_const(bit):
                continue
            raw_essential, raw_rows, raw_guards = raw_dependency(cell, pin)
            if not raw_essential:
                results.append(
                    RedundantPin(cell, pin, raw_rows, raw_guards, ["raw truth-table redundant"], "raw-redundant", 0.0)
                )
                continue
            status, elapsed_ms = checker.constrained_pin_status(cell, pin)
            if status == unsat:
                invariants = checker.explain_pin(cell, pin, raw_guards) if args.explain else []
                results.append(
                    RedundantPin(cell, pin, raw_rows, raw_guards, invariants, "event-constrained", elapsed_ms)
                )
            elif status == unknown:
                results.append(RedundantPin(cell, pin, raw_rows, raw_guards, [], "unknown", elapsed_ms))
    return results


def markdown_report(netlist: Netlist, lut_names: list[str], results: list[RedundantPin], args: argparse.Namespace) -> str:
    constrained = [r for r in results if r.status == "event-constrained"]
    raw = [r for r in results if r.status == "raw-redundant"]
    unknowns = [r for r in results if r.status == "unknown"]
    lines: list[str] = []
    lines.append(f"# {args.stage} Event-Constrained LUT SAT Report")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append(f"- Netlist: `{netlist.json_path}`")
    lines.append(f"- LUTs scanned: {len(lut_names)}")
    lines.append(f"- Minimum LUT width: {args.min_width}")
    lines.append(f"- Event stages constrained: {', '.join(args.event_stage)}")
    lines.append(f"- Event-constrained redundant pins: {len(constrained)}")
    lines.append(f"- Raw truth-table redundant pins: {len(raw)}")
    lines.append(f"- Unknown solver results: {len(unknowns)}")
    lines.append("")
    lines.append("## Procedure")
    lines.append("")
    lines.append("For each direct LUT input, the checker holds all other direct LUT inputs equal")
    lines.append("between two copies of the post-ABC netlist, forces the candidate input to differ,")
    lines.append("and asks Z3 whether the LUT output can differ while all recognized S1-S5 event")
    lines.append("signals satisfy the pixel-connectivity event equations. `unsat` means the raw LUT")
    lines.append("input is geometrically unreachable as an independent degree of freedom.")
    lines.append("")
    lines.append("## Endpoint Selection")
    lines.append("")
    if args.endpoint_regex:
        for regex in args.endpoint_regex:
            lines.append(f"- Endpoint regex: `{regex}`")
    if args.cell_regex:
        for regex in args.cell_regex:
            lines.append(f"- Direct cell/net regex: `{regex}`")
    if args.all_luts:
        lines.append("- Scope: all `$lut` cells in the module")
    lines.append("")
    lines.append("## Key Redundancies")
    lines.append("")
    if not constrained:
        lines.append("No event-constrained redundant pins found in this scope.")
    else:
        for r in constrained[:25]:
            cell = r.cell
            lines.append(
                f"- `{cell.name}` W{cell.width} output `{netlist.bit_label(cell.output)}` pin{r.pin} "
                f"input `{netlist.bit_label(cell.inputs[r.pin])}`"
            )
            if r.raw_dependency_guards:
                lines.append(f"  - Raw dependency: {', '.join(r.raw_dependency_guards)}")
            if r.invariants:
                lines.append(f"  - Invariants: {', '.join(r.invariants[:6])}")
            src = cell.src or netlist.bit_src(cell.output)
            if src:
                lines.append(f"  - Source: `{src}`")
    lines.append("")
    lines.append("## Full Redundant Pin List")
    lines.append("")
    lines.append("| Status | Cell | W | Output | Pin | Input | Raw dependency rows | Guards | Invariants | Source |")
    lines.append("| --- | --- | ---: | --- | ---: | --- | ---: | --- | --- | --- |")
    for r in results:
        if r.status not in {"event-constrained", "raw-redundant", "unknown"}:
            continue
        cell = r.cell
        guards = "<br>".join(r.raw_dependency_guards)
        invariants = "<br>".join(r.invariants)
        src = cell.src or netlist.bit_src(cell.output)
        lines.append(
            f"| {r.status} | `{cell.name}` | {cell.width} | `{netlist.bit_label(cell.output)}` | "
            f"{r.pin} | `{netlist.bit_label(cell.inputs[r.pin])}` | {r.raw_dependency_rows} | "
            f"{guards} | {invariants} | `{src}` |"
        )
    lines.append("")
    return "\n".join(lines)


def compile_patterns(values: list[str]) -> list[re.Pattern[str]]:
    return [re.compile(value) for value in values]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", required=True, type=Path)
    parser.add_argument("--top", required=True, help="top module in the supplied netlist")
    parser.add_argument("--stage", required=True)
    parser.add_argument("--endpoint-regex", action="append", default=[])
    parser.add_argument("--cell-regex", action="append", default=[])
    parser.add_argument("--event-stage", action="append", default=list(EVENT_STAGES))
    parser.add_argument("--include-ff-di", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--all-luts", action="store_true")
    parser.add_argument("--min-width", "--min-lut-width", dest="min_width", type=int, default=1)
    parser.add_argument("--max-luts", type=int, default=0)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--explain", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    netlist = Netlist(args.json, args.top)
    endpoint_patterns = compile_patterns(args.endpoint_regex)
    cell_patterns = compile_patterns(args.cell_regex)

    lut_set: set[str] = set(netlist.luts) if args.all_luts else set()
    if endpoint_patterns:
        endpoints = netlist.endpoint_bits(endpoint_patterns, args.include_ff_di)
        lut_set.update(netlist.lut_cone(endpoints))
    if cell_patterns:
        lut_set.update(name for name, cell in netlist.luts.items() if netlist.cell_matches(cell, cell_patterns))

    lut_names = sorted(name for name in lut_set if netlist.luts[name].width >= args.min_width)
    results = analyze(netlist, lut_names, args)
    report = markdown_report(netlist, lut_names, results, args)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report)
    else:
        print(report)


if __name__ == "__main__":
    main()
