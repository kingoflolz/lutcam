#!/usr/bin/env python3
"""Analyze Efinity post-map area-cell attribution from ``*.map.core.v``.

Efinity's mapped core netlist preserves useful but incomplete attribution:
many primitives retain hierarchical net names and some have generated-Verilog
source comments, while optimized intermediate cells often use anonymous nets.
By default this reports all EFX/XLR primitive cells. Pass ``--cell-type`` when
you want a narrower breakdown, for example ``--cell-type EFX_LUT4`` for a
LUT-only view. The attribution pass propagates anonymous cells through
input/output hierarchy and fanout.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
from functools import cache
from itertools import combinations
from pathlib import Path
import re
import sys


DEFAULT_TOP_MODULES = (
    "child_arena",
    "fv_combiner",
    "fv_fifo",
    "inner_centroid_compressor",
    "inner_hier_2ppc",
    "outer_hier_2ppc",
)

DEFAULT_OUTPUT_PORTS = frozenset(("O", "Q", "CO", "RDATA", "RDATAA", "RDATAB"))
OUTPUT_PORTS_BY_CELL_TYPE = {
    "EFX_LUT4": frozenset(("O",)),
    "EFX_ADD": frozenset(("O", "CO")),
    "EFX_FF": frozenset(("Q",)),
    "EFX_SRL8": frozenset(("Q",)),
    "EFX_RAM10": frozenset(("RDATA",)),
    "EFX_DPRAM10": frozenset(("RDATAA", "RDATAB")),
    "EFX_DSP48": frozenset(("O", "CASCOUT")),
    "EFX_DSP12": frozenset(("O", "CASCOUT")),
}

REPORT_RESOURCE_FOR_CELL_TYPE = {
    "EFX_FF": "FFs",
    "EFX_SRL8": "SRLs",
    "EFX_ADD": "ADDs",
    "EFX_LUT4": "LUTs",
    "EFX_RAM10": "RAMs",
    "EFX_DPRAM10": "RAMs",
    "EFX_DSP48": "DSP/MULTs",
    "EFX_DSP12": "DSP/MULTs",
}

CELL_INSTANCE_RE = re.compile(r"^\s*(EFX_[A-Z0-9]+)\s+")
INSTANCE_RE = re.compile(r"^\s*(?P<cell_type>EFX_[A-Z0-9]+)\s+(?P<name>\S+)\s*\(")
PORT_RE = re.compile(
    r"\.(?P<port>[A-Za-z_][A-Za-z0-9_$]*)\((?P<expr>.*?)\)"
    r"(?=\s*(?:,|\)|/\*))"
)
SOURCE_COMMENT_RE = re.compile(r"//\s+(.+?)\((\d+)\)\s*$")
PY_SRC_ATTR_RE = re.compile(r'\(\*\s*src\s*=\s*"([^"]+):(\d+)"\s*\*\)')
ESCAPED_NET_RE = re.compile(r"\\[^\s,{}()]+(?:\s+\[[^\]]+\])?")
SIMPLE_NET_RE = re.compile(r"\b[A-Za-z_][A-Za-z0-9_$]*(?:\[[^\]]+\])?")
COUNT_PAIR_RE = re.compile(r"^(?P<total>\d+)\((?P<self>\d+)\)$")
RESOURCE_SUMMARY_RE = re.compile(r"^(EFX_[A-Z0-9]+)\s*:\s*(\d+)\s*$")
VERIFIC_ATTR_COMMENT_RE = re.compile(r"/\*\s*verific\s+(.*?)\s*\*/")
VERIFIC_ATTR_PAIR_RE = re.compile(
    r"(?P<key>[A-Za-z_][A-Za-z0-9_$]*)\s*=\s*(?P<value>[^,*/]+)"
)


@dataclass(frozen=True)
class PortConnection:
    port: str
    expr: str
    nets: tuple[str, ...]


@dataclass(frozen=True)
class SourceLoc:
    path: Path
    line: int

    @property
    def short(self) -> str:
        return f"{self.path.name}:{self.line}"


@dataclass(frozen=True)
class CellRecord:
    line: int
    cell_type: str
    name: str
    outputs: tuple[str, ...]
    inputs: tuple[str, ...]
    source: SourceLoc | None
    ports: tuple[PortConnection, ...] = ()
    attributes: tuple[tuple[str, str], ...] = ()
    statement: str = ""


@dataclass(frozen=True)
class ReportModule:
    name: str
    resources: dict[str, tuple[int, int]]


@dataclass(frozen=True)
class SourceAnchor:
    generated_line: int
    source: SourceLoc


def resolve_map_core(path: Path) -> Path:
    if path.is_file():
        return path

    candidates = sorted((path / "outflow").glob("*.map.core.v"))
    candidates.extend(sorted(path.glob("*.map.core.v")))
    if len(candidates) == 1:
        return candidates[0]
    if candidates:
        names = "\n  ".join(str(candidate) for candidate in candidates)
        raise SystemExit(f"Multiple map core files found; pass one explicitly:\n  {names}")
    raise SystemExit(f"No *.map.core.v found under {path}")


def infer_map_report(map_core: Path) -> Path | None:
    stem = map_core.name.removesuffix(".map.core.v")
    candidates = [
        map_core.with_name(f"{stem}.map.rpt"),
        map_core.parent / f"{stem}.map.rpt",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    matches = sorted(map_core.parent.glob("*.map.rpt"))
    return matches[0] if len(matches) == 1 else None


def infer_generated_verilog(records: list[CellRecord]) -> Path | None:
    wrapper_names = {"helloworld.v", "lrcca_ti60_1xhierarchical.v"}
    counts: Counter[Path] = Counter()
    for record in records:
        if record.source and record.source.path.name.endswith(".v"):
            counts[record.source.path] += 1
    for path, _count in counts.most_common():
        if path.is_file() and path.name not in wrapper_names:
            return path
    for path, _count in counts.most_common():
        if path.is_file():
            return path
    return None


def extract_nets(expr: str) -> tuple[str, ...]:
    nets: list[str] = []
    for match in ESCAPED_NET_RE.finditer(expr):
        nets.append(match.group(0).strip())

    cleaned = ESCAPED_NET_RE.sub(" ", expr)
    for match in SIMPLE_NET_RE.finditer(cleaned):
        token = match.group(0)
        if match.start() > 0 and cleaned[match.start() - 1] == "'":
            continue
        nets.append(token)
    return tuple(nets)


def parse_verific_attributes(statement: str) -> tuple[tuple[str, str], ...]:
    match = VERIFIC_ATTR_COMMENT_RE.search(statement)
    if not match:
        return ()
    return tuple(
        (attr_match.group("key"), attr_match.group("value").strip())
        for attr_match in VERIFIC_ATTR_PAIR_RE.finditer(match.group(1))
    )


def output_ports_for(cell_type: str) -> frozenset[str]:
    return OUTPUT_PORTS_BY_CELL_TYPE.get(cell_type, DEFAULT_OUTPUT_PORTS)


def parse_cell_records(path: Path) -> list[CellRecord]:
    lines = path.read_text().splitlines()
    records: list[CellRecord] = []
    index = 0

    while index < len(lines):
        line = lines[index]
        if not CELL_INSTANCE_RE.search(line):
            index += 1
            continue

        start_line = index + 1
        statement = line.rstrip()
        while ";" not in statement and index + 1 < len(lines):
            index += 1
            statement += " " + lines[index].strip()

        instance_match = INSTANCE_RE.search(statement)
        if not instance_match:
            index += 1
            continue

        cell_type = instance_match.group("cell_type")
        output_ports = output_ports_for(cell_type)
        port_connections = tuple(
            PortConnection(
                port=match.group("port"),
                expr=match.group("expr").strip(),
                nets=extract_nets(match.group("expr").strip()),
            )
            for match in PORT_RE.finditer(statement)
        )
        ports: dict[str, str] = {
            connection.port: connection.expr for connection in port_connections
        }
        outputs: list[str] = []
        inputs: list[str] = []
        for port, expr in ports.items():
            if port in output_ports:
                outputs.extend(extract_nets(expr))
            else:
                inputs.extend(extract_nets(expr))

        source_match = SOURCE_COMMENT_RE.search(statement)
        source = None
        if source_match:
            source = SourceLoc(Path(source_match.group(1)), int(source_match.group(2)))

        records.append(
            CellRecord(
                line=start_line,
                cell_type=cell_type,
                name=instance_match.group("name"),
                outputs=tuple(outputs),
                inputs=tuple(inputs),
                source=source,
                ports=port_connections,
                attributes=parse_verific_attributes(statement),
                statement=statement,
            )
        )
        index += 1

    return records


def normalized_net(net: str) -> str:
    net = net.strip()
    if not net:
        return ""
    if net.startswith("\\"):
        return net[1:].split()[0]
    return net.split("[")[0].strip()


def direct_module(net: str, top_modules: set[str]) -> str | None:
    normalized = normalized_net(net)
    if not normalized:
        return None
    if normalized.startswith("core/"):
        parts = normalized.split("/")
        if len(parts) >= 2 and parts[1] in top_modules:
            return parts[1]
        return "core_top"
    if re.fullmatch(r"n\d+", normalized):
        return None
    if re.fullmatch(r"Open_\d+", normalized):
        return None
    if re.fullmatch(r"LUT__\d+", normalized):
        return None
    return "top_io_or_wrapper"


def direct_module_from_nets(nets: tuple[str, ...], top_modules: set[str]) -> str | None:
    modules = {
        module
        for module in (direct_module(net, top_modules) for net in nets)
        if module is not None
    }
    known_modules = {module for module in modules if module in top_modules}
    if len(known_modules) == 1:
        return next(iter(known_modules))
    if len(modules) == 1:
        return next(iter(modules))
    return None


def direct_output_prefix(record: CellRecord, top_modules: set[str], depth: int = 2) -> str:
    candidates = record.outputs or (record.name,)
    for net in candidates:
        normalized = normalized_net(net)
        if normalized.startswith("core/"):
            parts = normalized.split("/")
            return "/".join(parts[:depth]) if len(parts) >= depth else normalized
    for net in candidates:
        normalized = normalized_net(net)
        if re.fullmatch(r"n\d+", normalized):
            continue
        if re.fullmatch(r"Open_\d+", normalized):
            continue
        if re.fullmatch(r"LUT__\d+", normalized):
            continue
        if normalized:
            return "<top-level/other>"
    return "<anonymous/open>"


def build_consumers(records: list[CellRecord]) -> dict[str, list[int]]:
    consumers: dict[str, list[int]] = defaultdict(list)
    for index, record in enumerate(records):
        for net in record.inputs:
            consumers[normalized_net(net)].append(index)
    return consumers


def assign_cells(records: list[CellRecord], top_modules: set[str]) -> list[str]:
    consumers = build_consumers(records)
    direct_outputs = [
        direct_module_from_nets(record.outputs, top_modules)
        for record in records
    ]
    direct_instances = [
        direct_module(record.name, top_modules)
        for record in records
    ]

    @cache
    def downstream_modules(index: int) -> frozenset[str]:
        direct = direct_outputs[index] or direct_instances[index]
        if direct:
            return frozenset((direct,))

        modules: set[str] = set()
        for output in records[index].outputs:
            for consumer_index in consumers.get(normalized_net(output), []):
                if consumer_index != index:
                    modules.update(downstream_modules(consumer_index))
        return frozenset(modules)

    assignments: list[str] = []
    for index, record in enumerate(records):
        direct = direct_outputs[index] or direct_instances[index]
        if direct:
            assignments.append(direct)
            continue

        input_module = direct_module_from_nets(record.inputs, top_modules)
        if input_module in top_modules:
            assignments.append(input_module)
            continue

        downstream = downstream_modules(index)
        known_downstream = {module for module in downstream if module in top_modules}
        if len(known_downstream) == 1:
            assignments.append(next(iter(known_downstream)))
        elif len(downstream) == 1:
            assignments.append(next(iter(downstream)))
        elif not downstream:
            assignments.append("unresolved")
        else:
            assignments.append("mixed_downstream")

    return assignments


def parse_count_pair(token: str) -> tuple[int, int] | None:
    match = COUNT_PAIR_RE.match(token)
    if not match:
        return None
    return int(match.group("total")), int(match.group("self"))


def parse_map_report(path: Path | None) -> dict[str, ReportModule]:
    if path is None or not path.is_file():
        return {}

    modules: dict[str, ReportModule] = {}
    columns = ("FFs", "SRLs", "ADDs", "LUTs", "COMB4s", "RAMs", "DSP/MULTs")
    for line in path.read_text().splitlines():
        parts = line.strip().split()
        if len(parts) < 1 + len(columns):
            continue
        if ":" not in parts[0]:
            continue
        pairs = [parse_count_pair(token) for token in parts[1 : 1 + len(columns)]]
        if any(pair is None for pair in pairs):
            continue
        name = parts[0].split(":", 1)[0].lstrip("+")
        modules[name] = ReportModule(
            name=name,
            resources={
                column: pair
                for column, pair in zip(columns, pairs, strict=True)
                if pair is not None
            },
        )
    return modules


def parse_resource_summary(path: Path | None) -> dict[str, int]:
    if path is None or not path.is_file():
        return {}

    resources: dict[str, int] = {}
    for line in path.read_text().splitlines():
        match = RESOURCE_SUMMARY_RE.match(line.strip())
        if match:
            resources[match.group(1)] = int(match.group(2))
    return resources


def parse_source_anchors(path: Path | None) -> list[SourceAnchor]:
    if path is None or not path.is_file():
        return []
    anchors: list[SourceAnchor] = []
    for line_number, line in enumerate(path.read_text().splitlines(), start=1):
        match = PY_SRC_ATTR_RE.search(line)
        if match:
            anchors.append(
                SourceAnchor(
                    generated_line=line_number,
                    source=SourceLoc(Path(match.group(1)), int(match.group(2))),
                )
            )
    return anchors


def nearest_anchor(
    anchors_by_file: dict[Path, list[SourceAnchor]], source: SourceLoc
) -> SourceAnchor | None:
    anchors = anchors_by_file.get(source.path)
    if not anchors:
        return None

    best: SourceAnchor | None = None
    for anchor in anchors:
        if anchor.generated_line > source.line:
            break
        best = anchor
    return best


def print_table(headers: tuple[str, ...], rows: list[tuple[object, ...]]) -> None:
    widths = [len(header) for header in headers]
    rendered = [[str(cell) for cell in row] for row in rows]
    for row in rendered:
        for index, cell in enumerate(row):
            widths[index] = max(widths[index], len(cell))

    print("  ".join(header.ljust(widths[index]) for index, header in enumerate(headers)))
    print("  ".join("-" * width for width in widths))
    for row in rendered:
        print("  ".join(cell.ljust(widths[index]) for index, cell in enumerate(row)))


def selected_indices(
    records: list[CellRecord], cell_types: set[str] | None
) -> list[int]:
    if not cell_types:
        return list(range(len(records)))
    return [
        index
        for index, record in enumerate(records)
        if record.cell_type in cell_types
    ]


def format_type_counts(counter: Counter[str], limit: int = 4) -> str:
    most_common = counter.most_common(limit)
    parts = [f"{cell_type}={count}" for cell_type, count in most_common]
    shown = sum(count for _cell_type, count in most_common)
    total = sum(counter.values())
    if total > shown:
        parts.append(f"other={total - shown}")
    return ", ".join(parts)


def print_cell_type_summary(
    records: list[CellRecord],
    indices: list[int],
    resource_summary: dict[str, int],
) -> None:
    total = Counter(records[index].cell_type for index in indices)
    with_source = Counter(
        records[index].cell_type
        for index in indices
        if records[index].source
    )
    print("\nCell type summary")
    rows: list[tuple[object, ...]] = []
    for cell_type, count in total.most_common():
        report_count = resource_summary.get(cell_type, "")
        delta = count - report_count if isinstance(report_count, int) else ""
        rows.append(
            (
                cell_type,
                count,
                with_source[cell_type],
                count - with_source[cell_type],
                report_count,
                delta,
            )
        )
    print_table(("cell type", "cells", "with src", "no src", "map.rpt", "delta"), rows)


def assignment_rows(
    records: list[CellRecord],
    assignments: list[str],
    indices: list[int],
    report_modules: dict[str, ReportModule],
    max_type_columns: int = 8,
) -> tuple[tuple[str, ...], list[tuple[object, ...]]]:
    by_assignment: dict[str, Counter[str]] = defaultdict(Counter)
    by_assignment_src: Counter[str] = Counter()
    for index in indices:
        assignment = assignments[index]
        by_assignment[assignment][records[index].cell_type] += 1
        if records[index].source:
            by_assignment_src[assignment] += 1

    type_totals = Counter()
    for counter in by_assignment.values():
        type_totals.update(counter)
    type_columns = [
        cell_type
        for cell_type, _count in type_totals.most_common(max_type_columns)
    ]

    selected_report_buckets = {
        REPORT_RESOURCE_FOR_CELL_TYPE[cell_type]
        for cell_type in type_columns
        if cell_type in REPORT_RESOURCE_FOR_CELL_TYPE
    }
    report_bucket = next(iter(selected_report_buckets)) if len(selected_report_buckets) == 1 else None

    headers = ["assignment", "cells", "with src", "no src", *type_columns]
    if report_bucket:
        headers.extend((f"map.rpt {report_bucket}", "delta"))

    rows: list[tuple[object, ...]] = []
    for assignment, counter in sorted(
        by_assignment.items(), key=lambda item: sum(item[1].values()), reverse=True
    ):
        total = sum(counter.values())
        row: list[object] = [
            assignment,
            total,
            by_assignment_src[assignment],
            total - by_assignment_src[assignment],
            *(counter[cell_type] or "" for cell_type in type_columns),
        ]
        if report_bucket:
            report = report_modules.get(assignment)
            report_count = report.resources[report_bucket][0] if report and report_bucket in report.resources else ""
            delta = total - report_count if isinstance(report_count, int) else ""
            row.extend((report_count, delta))
        rows.append(tuple(row))

    return tuple(headers), rows


def print_assignment_summary(
    records: list[CellRecord],
    assignments: list[str],
    indices: list[int],
    report_modules: dict[str, ReportModule],
) -> None:
    print("\nHeuristic cell attribution")
    headers, rows = assignment_rows(records, assignments, indices, report_modules)
    print_table(headers, rows)


def format_anchor(anchor: SourceAnchor | None, generated_line: int) -> str:
    if anchor is None:
        return ""
    delta = generated_line - anchor.generated_line
    return f"{anchor.source.short} (+{delta})"


def print_source_summary(
    records: list[CellRecord],
    assignments: list[str],
    indices: list[int],
    anchors_by_file: dict[Path, list[SourceAnchor]],
    limit: int,
) -> None:
    source_counts: Counter[SourceLoc] = Counter(
        records[index].source
        for index in indices
        if records[index].source
    )
    if not source_counts:
        print("\nNo source comments found for selected cells.")
        return

    print(f"\nTop generated-Verilog source comments for selected cells (top {limit})")
    rows: list[tuple[object, ...]] = []
    assignment_by_source: dict[SourceLoc, Counter[str]] = defaultdict(Counter)
    types_by_source: dict[SourceLoc, Counter[str]] = defaultdict(Counter)
    for index in indices:
        source = records[index].source
        if source:
            assignment_by_source[source][assignments[index]] += 1
            types_by_source[source][records[index].cell_type] += 1

    for source, count in source_counts.most_common(limit):
        module_counts = assignment_by_source[source]
        module = module_counts.most_common(1)[0][0] if module_counts else ""
        anchor = nearest_anchor(anchors_by_file, source)
        rows.append(
            (
                count,
                module,
                format_type_counts(types_by_source[source]),
                source.short,
                format_anchor(anchor, source.line),
            )
        )
    print_table(("cells", "assignment", "cell types", "source line", "nearest source anchor"), rows)


def print_prefix_summary(
    records: list[CellRecord],
    indices: list[int],
    top_modules: set[str],
    limit: int,
) -> None:
    all_prefixes: Counter[str] = Counter()
    with_source: Counter[str] = Counter()
    type_by_prefix: dict[str, Counter[str]] = defaultdict(Counter)
    for index in indices:
        record = records[index]
        prefix = direct_output_prefix(record, top_modules)
        all_prefixes[prefix] += 1
        type_by_prefix[prefix][record.cell_type] += 1
        if record.source:
            with_source[prefix] += 1

    print(f"\nDirect output/instance prefixes before heuristic propagation (top {limit})")
    rows: list[tuple[object, ...]] = []
    for prefix, count in all_prefixes.most_common(limit):
        rows.append(
            (
                prefix,
                count,
                with_source[prefix],
                count - with_source[prefix],
                format_type_counts(type_by_prefix[prefix]),
            )
        )
    print_table(("prefix", "cells", "with src", "no src", "cell types"), rows)


def attribute_dict(record: CellRecord) -> dict[str, str]:
    return dict(record.attributes)


def lutmask(record: CellRecord) -> int | None:
    value = attribute_dict(record).get("LUTMASK")
    if value is None:
        return None
    match = re.search(r"16'h([0-9a-fA-F]+)", value)
    if match:
        return int(match.group(1), 16)
    try:
        return int(value, 0)
    except ValueError:
        return None


def port_expr(record: CellRecord, port: str) -> str:
    for connection in record.ports:
        if connection.port == port:
            return connection.expr
    return ""


def port_nets(record: CellRecord, port: str) -> tuple[str, ...]:
    for connection in record.ports:
        if connection.port == port:
            return tuple(normalized_net(net) for net in connection.nets)
    return ()


@cache
def lut_support(mask: int) -> tuple[str, ...]:
    support: list[str] = []
    for bit in range(4):
        for index in range(16):
            if ((mask >> index) & 1) != ((mask >> (index ^ (1 << bit))) & 1):
                support.append(f"I{bit}")
                break
    return tuple(support)


def combine_implicants(
    left: tuple[int | None, ...],
    right: tuple[int | None, ...],
) -> tuple[int | None, ...] | None:
    different_index = -1
    combined: list[int | None] = []
    for index, (left_bit, right_bit) in enumerate(zip(left, right, strict=True)):
        if left_bit == right_bit:
            combined.append(left_bit)
        elif left_bit is None or right_bit is None:
            return None
        else:
            if different_index >= 0:
                return None
            different_index = index
            combined.append(None)
    return tuple(combined) if different_index >= 0 else None


def implicant_covers(implicant: tuple[int | None, ...], minterm: int) -> bool:
    bits = tuple((minterm >> bit) & 1 for bit in range(4))
    return all(
        implicant_bit is None or implicant_bit == minterm_bit
        for implicant_bit, minterm_bit in zip(implicant, bits, strict=True)
    )


def implicant_cost(implicant: tuple[int | None, ...]) -> int:
    return sum(1 for bit in implicant if bit is not None)


def implicant_sort_key(implicant: tuple[int | None, ...]) -> tuple[int, tuple[int, ...]]:
    return (
        implicant_cost(implicant),
        tuple(-1 if bit is None else bit for bit in implicant),
    )


@cache
def minimized_sop_terms(mask: int) -> tuple[tuple[int | None, ...], ...]:
    minterms = tuple(index for index in range(16) if (mask >> index) & 1)
    if not minterms:
        return ()
    if len(minterms) == 16:
        return ((None, None, None, None),)

    current: set[tuple[int | None, ...]] = {
        tuple((minterm >> bit) & 1 for bit in range(4))
        for minterm in minterms
    }
    prime_implicants: set[tuple[int | None, ...]] = set()
    while current:
        used: set[tuple[int | None, ...]] = set()
        next_terms: set[tuple[int | None, ...]] = set()
        ordered = sorted(current, key=implicant_sort_key)
        for left_index in range(len(ordered)):
            for right_index in range(left_index + 1, len(ordered)):
                combined = combine_implicants(
                    ordered[left_index],
                    ordered[right_index],
                )
                if combined is not None:
                    used.add(ordered[left_index])
                    used.add(ordered[right_index])
                    next_terms.add(combined)
        prime_implicants.update(term for term in current if term not in used)
        if not next_terms or next_terms == current:
            break
        current = next_terms

    primes = sorted(prime_implicants, key=implicant_sort_key)
    covers = {
        prime: {minterm for minterm in minterms if implicant_covers(prime, minterm)}
        for prime in primes
    }

    uncovered = set(minterms)
    selected: list[tuple[int | None, ...]] = []
    while True:
        essential: list[tuple[int | None, ...]] = []
        for minterm in list(uncovered):
            covering_primes = [prime for prime in primes if minterm in covers[prime]]
            if len(covering_primes) == 1 and covering_primes[0] not in selected:
                essential.append(covering_primes[0])
        if not essential:
            break
        for prime in essential:
            if prime not in selected:
                selected.append(prime)
                uncovered -= covers[prime]

    remaining = [
        prime
        for prime in primes
        if prime not in selected and covers[prime] & uncovered
    ]
    best: tuple[tuple[int, int, tuple[tuple[int, ...], ...]], tuple[tuple[int | None, ...], ...]] | None = None
    for count in range(len(remaining) + 1):
        for combo in combinations(remaining, count):
            combo_cover: set[int] = set()
            for prime in combo:
                combo_cover |= covers[prime]
            if not uncovered <= combo_cover:
                continue
            candidate = tuple(selected + list(combo))
            score = (
                len(candidate),
                sum(implicant_cost(term) for term in candidate),
                tuple(tuple(-1 if bit is None else bit for bit in term) for term in candidate),
            )
            if best is None or score < best[0]:
                best = (score, candidate)
        if best is not None:
            break

    if best is None:
        return tuple(sorted(selected, key=implicant_sort_key))
    return tuple(sorted(best[1], key=implicant_sort_key))


@cache
def lut_formula(mask: int) -> str:
    terms = minimized_sop_terms(mask)
    if not terms:
        return "0"

    rendered_terms: list[str] = []
    for term in terms:
        literals: list[str] = []
        for bit_index, bit in enumerate(term):
            if bit is None:
                continue
            literals.append(f"I{bit_index}" if bit else f"~I{bit_index}")
        rendered_terms.append(" & ".join(literals) if literals else "1")
    return " | ".join(rendered_terms)


@cache
def lut_one_minterms(mask: int) -> str:
    return ",".join(f"{index:x}" for index in range(16) if (mask >> index) & 1)


def is_anonymous_net(net: str) -> bool:
    normalized = normalized_net(net)
    if not normalized:
        return True
    return bool(re.fullmatch(r"(?:n\d+|Open_\d+|LUT__\d+)", normalized))


def named_downstream_sinks(
    records: list[CellRecord],
    consumers: dict[str, list[int]],
    index: int,
    *,
    max_depth: int,
    max_items: int,
) -> tuple[str, ...]:
    seen_nets: set[str] = set()
    seen_records: set[int] = set()
    queue: deque[tuple[str, int]] = deque(
        (normalized_net(output), 0)
        for output in records[index].outputs
        if normalized_net(output)
    )
    sinks: list[str] = []

    while queue and len(sinks) < max_items:
        net, depth = queue.popleft()
        if net in seen_nets:
            continue
        seen_nets.add(net)

        for consumer_index in consumers.get(net, []):
            if consumer_index == index or consumer_index in seen_records:
                continue
            seen_records.add(consumer_index)
            consumer = records[consumer_index]
            named_outputs = [
                normalized_net(output)
                for output in consumer.outputs
                if not is_anonymous_net(output)
            ]
            if named_outputs:
                for output in named_outputs:
                    if output not in sinks:
                        sinks.append(output)
                        if len(sinks) >= max_items:
                            break
            elif depth < max_depth:
                for output in consumer.outputs:
                    normalized_output = normalized_net(output)
                    if normalized_output and normalized_output not in seen_nets:
                        queue.append((normalized_output, depth + 1))
    return tuple(sinks)


def immediate_consumer_text(
    records: list[CellRecord],
    consumers: dict[str, list[int]],
    index: int,
    *,
    max_items: int,
) -> tuple[str, ...]:
    items: list[str] = []
    for output in records[index].outputs:
        for consumer_index in consumers.get(normalized_net(output), []):
            if consumer_index == index:
                continue
            consumer = records[consumer_index]
            consumer_outputs = [
                normalized_net(consumer_output)
                for consumer_output in consumer.outputs
                if normalized_net(consumer_output)
            ]
            text = f"{consumer.cell_type}:{consumer.name}"
            if consumer_outputs:
                text += "->" + "|".join(consumer_outputs[:3])
            items.append(text)
            if len(items) >= max_items:
                return tuple(items)
    return tuple(items)


def assignment_reason(
    record: CellRecord,
    top_modules: set[str],
) -> str:
    direct_output = direct_module_from_nets(record.outputs, top_modules)
    if direct_output:
        return "direct_output"
    direct_instance = direct_module(record.name, top_modules)
    if direct_instance:
        return "direct_instance"
    input_module = direct_module_from_nets(record.inputs, top_modules)
    if input_module in top_modules:
        return "input_cone"
    return "single_owner_downstream"


def signal_family(record: CellRecord, downstream_sinks: tuple[str, ...]) -> str:
    candidates = (*record.outputs, *downstream_sinks, record.name)
    for candidate in candidates:
        normalized = normalized_net(candidate)
        if not normalized or is_anonymous_net(normalized):
            continue
        return re.sub(r"\[\d+\]$", "", normalized)
    return "<anonymous>"


def classify_lut_role(record: CellRecord, downstream_sinks: tuple[str, ...]) -> str:
    context = " ".join(
        (
            record.name,
            " ".join(record.inputs),
            " ".join(record.outputs),
            " ".join(downstream_sinks),
            " ".join(connection.expr for connection in record.ports),
        )
    ).lower()

    if "core/child_arena/cmd_fifo" in context:
        if any(
            token in context
            for token in ("r_port__addr", "w_port__addr", "w_rdy", "r_rdy", "level")
        ):
            return "cmd_fifo_pointer_status"
        return "cmd_fifo_storage_bypass_payload"
    if "core/child_arena/child_arena_returned_ids" in context:
        if any(
            token in context
            for token in ("r_port__addr", "w_port__addr", "w_rdy", "r_rdy", "level")
        ):
            return "returned_id_fifo_pointer_status"
        return "returned_id_fifo_payload_bypass"
    if "r_data$49_tmp" in context or "r_data$49_bp" in context:
        return "cmd_fifo_write_first_bypass_mux"
    if any(
        token in context
        for token in (
            "port_a_w__data",
            "port_a_fv_w__data",
            "port_a_next_w__data",
            "port_a_meta_w__data",
            "port_a_fv_w_data",
            "port_a_next_w_data",
            "port_a_meta_w_data",
        )
    ):
        return "port_a_init_write_data_mux"
    if any(
        token in context
        for token in (
            "port_b_w__data",
            "port_b_fv_w__data",
            "port_b_next_w__data",
            "port_b_meta_w__data",
            "port_b_fv_w_data",
            "port_b_next_w_data",
            "port_b_meta_w_data",
        )
    ):
        return "port_b_service_write_data_mux"
    if any(
        token in context
        for token in ("port_a_write_addr", "port_a_addr", "read_port_addr")
    ):
        return "port_a_read_write_address_mux"
    if any(
        token in context
        for token in ("port_b_write_addr", "port_b_addr", "service_read_addr")
    ):
        return "port_b_service_address_mux"
    if any(
        token in context
        for token in (
            "alloc_",
            "linear_remaining",
            "linear_next_id",
            "return_push",
            "returned_id",
            "free_count",
            "new_ready",
            "new_rsp",
            "rsp_full",
            "svc_alloc",
        )
    ):
        return "allocation_free_list_control"
    if any(
        token in context
        for token in ("read_", "out_valid", "out_first", "out_last", "out_overflow")
    ):
        return "read_drain_fsm_output"
    if (
        "cmd_fv" in context and "port_b_r__data" in context
    ) or (
        "add_fv" in context
        and any(token in context for token in ("port_b_r__data", "add_head_fv", "add_prev_fv"))
    ):
        return "add_key_compare_saturation"
    if "add_" in context:
        return "add_state_datapath_control"
    if any(
        token in context
        for token in (
            "choose_c",
            "selected_",
            "drop_selected",
            "free_selected",
            "first_output",
            "use_carry",
            "use_spare",
            "output_node",
            "output_fv",
            "pending_needs_write",
            "can_drop",
        )
    ):
        return "merge_select_drop_output_control"
    if "merge_" in context:
        return "merge_state_datapath_control"
    if any(
        token in context
        for token in ("cmd_head", "cmd_pop", "cmd_fire", "cmd_ready", "fsm_state", "idle")
    ):
        return "service_fsm_dispatch_control"
    if "rst" in context or "reset" in context:
        return "reset_clock_enable_glue"
    return "unclassified_child_arena_logic"


def source_line_context(
    record: CellRecord,
    generated_verilog: Path | None,
    generated_lines: list[str],
) -> tuple[str, str]:
    if record.source is None:
        return "", ""
    source = record.source
    if (
        generated_verilog
        and source.path.name == generated_verilog.name
        and 1 <= source.line <= len(generated_lines)
    ):
        return source.short, generated_lines[source.line - 1].strip()
    return source.short, ""


def write_lut_detail_report(
    *,
    records: list[CellRecord],
    assignments: list[str],
    module: str,
    top_modules: set[str],
    map_core: Path,
    generated_verilog: Path | None,
    csv_path: Path | None,
    markdown_path: Path | None,
    max_sinks: int,
) -> dict[str, object]:
    indices = [
        index
        for index, record in enumerate(records)
        if record.cell_type == "EFX_LUT4" and assignments[index] == module
    ]
    if not indices:
        raise SystemExit(f"No EFX_LUT4 cells attributed to {module!r}")

    generated_lines = (
        generated_verilog.read_text().splitlines()
        if generated_verilog and generated_verilog.is_file()
        else []
    )
    consumers = build_consumers(records)
    rows: list[dict[str, object]] = []
    role_counts: Counter[str] = Counter()
    reason_counts: Counter[str] = Counter()
    prefix_counts: Counter[str] = Counter()
    formula_counts: Counter[str] = Counter()
    family_counts: Counter[str] = Counter()

    for row_index, record_index in enumerate(indices, start=1):
        record = records[record_index]
        mask = lutmask(record)
        if mask is None:
            mask_text = ""
            active_pins = ""
            formula = ""
            minterms = ""
        else:
            mask_text = f"16'h{mask:04x}"
            active_pins = ",".join(lut_support(mask))
            formula = lut_formula(mask)
            minterms = lut_one_minterms(mask)

        downstream_sinks = named_downstream_sinks(
            records,
            consumers,
            record_index,
            max_depth=7,
            max_items=max_sinks,
        )
        consumers_text = immediate_consumer_text(
            records,
            consumers,
            record_index,
            max_items=max_sinks,
        )
        role = classify_lut_role(record, downstream_sinks)
        reason = assignment_reason(record, top_modules)
        prefix = direct_output_prefix(record, top_modules)
        family = signal_family(record, downstream_sinks)
        generated_source, generated_line_text = source_line_context(
            record,
            generated_verilog,
            generated_lines,
        )

        role_counts[role] += 1
        reason_counts[reason] += 1
        prefix_counts[prefix] += 1
        formula_counts[formula] += 1
        family_counts[family] += 1

        rows.append(
            {
                "row": row_index,
                "map_line": record.line,
                "instance": record.name,
                "semantic_role": role,
                "attribution_reason": reason,
                "direct_prefix_before_propagation": prefix,
                "signal_family": family,
                "output_nets": ";".join(normalized_net(net) for net in record.outputs),
                "output_expr": port_expr(record, "O"),
                "lutmask": mask_text,
                "active_pins": active_pins,
                "pin_formula_sop": formula,
                "one_minterms_hex": minterms,
                "I0_expr": port_expr(record, "I0"),
                "I0_nets": ";".join(port_nets(record, "I0")),
                "I1_expr": port_expr(record, "I1"),
                "I1_nets": ";".join(port_nets(record, "I1")),
                "I2_expr": port_expr(record, "I2"),
                "I2_nets": ";".join(port_nets(record, "I2")),
                "I3_expr": port_expr(record, "I3"),
                "I3_nets": ";".join(port_nets(record, "I3")),
                "nearest_named_downstream_sinks": ";".join(downstream_sinks),
                "immediate_consumers": ";".join(consumers_text),
                "generated_source": generated_source,
                "generated_line_text": generated_line_text,
                "map_statement": record.statement,
            }
        )

    if csv_path:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=tuple(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    def counter_table(counter: Counter[str], first_header: str, limit: int | None = None) -> str:
        lines = [
            f"| {first_header} | LUTs |",
            "|---|---:|",
        ]
        for key, count in counter.most_common(limit):
            lines.append(f"| `{key}` | {count} |")
        return "\n".join(lines)

    if markdown_path:
        markdown_path.parent.mkdir(parents=True, exist_ok=True)
        markdown = [
            f"# {module} EFX_LUT4 Detail",
            "",
            f"Map core: `{map_core}`",
            f"Generated Verilog: `{generated_verilog or 'not found'}`",
            f"Attributed `EFX_LUT4` cells: `{len(rows)}`",
            "",
            "The CSV is the exhaustive per-LUT table. It records the exact LUT mask,",
            "the exact pin expressions, the active pins, the exact 1-minterms, and",
            "a minimized SOP expression over `I0..I3` for every LUT attributed to",
            f"`{module}`. The role column is a grouping aid; the mask/pin/minterm",
            "columns are the authoritative function.",
            "",
            "The Efinix simulation model indexes `LUTMASK` with `{I3,I2,I1,I0}`,",
            "so minterm index `n` is `I0 + 2*I1 + 4*I2 + 8*I3`.",
            "",
            "## Semantic Role Counts",
            "",
            counter_table(role_counts, "role"),
            "",
            "## Attribution Reason Counts",
            "",
            counter_table(reason_counts, "reason"),
            "",
            "## Direct Prefix Counts Before Propagation",
            "",
            counter_table(prefix_counts, "prefix"),
            "",
            "## Most Common LUT Functions",
            "",
            counter_table(formula_counts, "pin formula", 40),
            "",
            "## Most Common Signal Families",
            "",
            counter_table(family_counts, "signal family", 60),
            "",
        ]
        if csv_path:
            markdown.extend(("## CSV", "", f"`{csv_path}`", ""))
        markdown_path.write_text("\n".join(markdown))

    return {
        "rows": len(rows),
        "csv_path": csv_path,
        "markdown_path": markdown_path,
        "role_counts": role_counts,
        "reason_counts": reason_counts,
        "prefix_counts": prefix_counts,
        "formula_counts": formula_counts,
        "family_counts": family_counts,
    }


def parse_cell_type_filter(cell_types: list[str] | None) -> set[str] | None:
    if not cell_types:
        return None
    return {cell_type.upper() for cell_type in cell_types}


def main(
    argv: list[str] | None = None,
    *,
    default_cell_types: tuple[str, ...] | None = None,
) -> int:
    parser = argparse.ArgumentParser(
        description="Analyze Efinity EFX/XLR primitive-cell attribution from a *.map.core.v netlist."
    )
    parser.add_argument(
        "map_core",
        type=Path,
        help="Path to *.map.core.v, or an Efinity project/output directory containing one.",
    )
    parser.add_argument(
        "--map-report",
        type=Path,
        help="Optional *.map.rpt for comparison against Efinity resource summaries.",
    )
    parser.add_argument(
        "--generated-verilog",
        type=Path,
        help="Optional Verilog file used for nearest source anchors.",
    )
    parser.add_argument(
        "--cell-type",
        action="append",
        dest="cell_types",
        help="Primitive cell type to include, such as EFX_LUT4 or EFX_FF. May be repeated.",
    )
    parser.add_argument(
        "--top",
        type=int,
        default=20,
        help="Rows to show in top-N source/prefix tables.",
    )
    parser.add_argument(
        "--module",
        action="append",
        dest="modules",
        help=(
            "Immediate core child module to treat as a top-level attribution bucket. "
            "May be passed more than once."
        ),
    )
    parser.add_argument(
        "--no-prefix-summary",
        action="store_true",
        help="Hide the direct output/instance-prefix table.",
    )
    parser.add_argument(
        "--no-source-summary",
        action="store_true",
        help="Hide the generated-Verilog source-comment table.",
    )
    parser.add_argument(
        "--lut-detail-module",
        help=(
            "Write an exhaustive EFX_LUT4 detail report for one attribution bucket, "
            "for example child_arena."
        ),
    )
    parser.add_argument(
        "--lut-detail-csv",
        type=Path,
        help=(
            "CSV path for --lut-detail-module. Defaults to "
            "<map-core-dir>/<module>_lut_detail.csv."
        ),
    )
    parser.add_argument(
        "--lut-detail-md",
        type=Path,
        help=(
            "Markdown summary path for --lut-detail-module. Defaults to "
            "<map-core-dir>/<module>_lut_detail.md."
        ),
    )
    parser.add_argument(
        "--lut-detail-max-sinks",
        type=int,
        default=12,
        help="Maximum downstream sink/consumer entries to include per detailed LUT row.",
    )
    args = parser.parse_args(argv)
    if (args.lut_detail_csv or args.lut_detail_md) and not args.lut_detail_module:
        parser.error("--lut-detail-csv/--lut-detail-md require --lut-detail-module")

    map_core = resolve_map_core(args.map_core)
    records = parse_cell_records(map_core)
    if not records:
        raise SystemExit(f"No EFX_* primitive cell instances found in {map_core}")

    explicit_filter = parse_cell_type_filter(args.cell_types)
    default_filter = set(default_cell_types) if default_cell_types and not explicit_filter else None
    cell_type_filter = explicit_filter or default_filter
    indices = selected_indices(records, cell_type_filter)
    if not indices:
        selected = ", ".join(sorted(cell_type_filter or ()))
        raise SystemExit(f"No selected primitive cell instances found in {map_core}: {selected}")

    map_report = args.map_report or infer_map_report(map_core)
    generated_verilog = args.generated_verilog or infer_generated_verilog(records)
    report_modules = parse_map_report(map_report)
    resource_summary = parse_resource_summary(map_report)
    top_modules = set(args.modules or DEFAULT_TOP_MODULES)
    assignments = assign_cells(records, top_modules)

    anchors_by_file: dict[Path, list[SourceAnchor]] = {}
    if generated_verilog:
        anchors_by_file[generated_verilog] = parse_source_anchors(generated_verilog)

    direct_source_count = sum(1 for index in indices if records[index].source)
    selected_types_text = (
        ", ".join(sorted(cell_type_filter))
        if cell_type_filter
        else "all EFX_*"
    )
    print(f"map core:          {map_core}")
    print(f"map report:        {map_report or 'not found'}")
    print(f"generated verilog: {generated_verilog or 'not found'}")
    print(f"selected cells:    {selected_types_text}")
    print(f"cell instances:    {len(indices)} selected / {len(records)} total")
    print(f"with src comment:  {direct_source_count}")
    print(f"without src:       {len(indices) - direct_source_count}")
    if generated_verilog:
        print(f"source anchors:    {len(anchors_by_file.get(generated_verilog, []))}")

    print_cell_type_summary(records, indices, resource_summary)
    print_assignment_summary(records, assignments, indices, report_modules)
    if not args.no_source_summary:
        print_source_summary(records, assignments, indices, anchors_by_file, args.top)
    if not args.no_prefix_summary:
        print_prefix_summary(records, indices, top_modules, args.top)
    if args.lut_detail_module:
        detail_stem = f"{args.lut_detail_module}_lut_detail"
        csv_path = args.lut_detail_csv or map_core.parent / f"{detail_stem}.csv"
        markdown_path = args.lut_detail_md or map_core.parent / f"{detail_stem}.md"
        detail = write_lut_detail_report(
            records=records,
            assignments=assignments,
            module=args.lut_detail_module,
            top_modules=top_modules,
            map_core=map_core,
            generated_verilog=generated_verilog,
            csv_path=csv_path,
            markdown_path=markdown_path,
            max_sinks=args.lut_detail_max_sinks,
        )
        print("\nDetailed LUT report")
        print(f"module:           {args.lut_detail_module}")
        print(f"EFX_LUT4 cells:   {detail['rows']}")
        print(f"CSV:              {detail['csv_path']}")
        print(f"Markdown summary: {detail['markdown_path']}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
