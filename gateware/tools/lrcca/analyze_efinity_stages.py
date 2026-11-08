#!/usr/bin/env python3
"""Summarize Efinity timing paths by LRCCA pipeline stage.

The Efinity text timing reports do not contain an exhaustive per-net endpoint
database like nextpnr's detailed JSON report. This script classifies paths
emitted by ``report_path`` or ``report_timing`` and reports the worst observed
path by sink stage and by source->sink stage pair. Coverage follows the path
caps used when the timing report was generated.
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


StageRules = tuple[tuple[str, tuple[str, ...]], ...]

IO_RULES = (
    r"(^|/)(clk|rst|d0|d1|datavalid|sof|eol)(\b|\[|$)",
    r"(^|/)(fv_valid|fv_data|inner_fv_valid|inner_fv_data|inner_end|in_ready)(\b|\[|$)",
    r"\bGPIO",
)


def hier_2ppc_rules(prefix: str, label: str) -> StageRules:
    root = rf"\b{re.escape(prefix)}/"
    return (
        (
            f"{label} S0 ingress/rowbuf",
            (
                root + r"(accept|first_row|last_row|row_start|row_end|d[01]_eff)(\b|_|\[)",
                root + r"(ingress_|drain|s0_|rowbuf_)",
                root + r"rowbuf_2ppc",
            ),
        ),
        (f"{label} S1 events/PF issue", (root + r"s1(\b|_|\[)", root + r"p_issue")),
        (
            f"{label} S2 PF0 reads",
            (
                root + r"s2(\b|_|\[)",
                root + r"pf0_",
                root + r"(next|head|data)_2ppc_.*_unused_read_sink",
            ),
        ),
        (f"{label} S3 PF1 resolve", (root + r"s3(\b|_|\[)", root + r"pf1_", root + r"token_raw(?!_repaired)")),
        (
            f"{label} S4 token assembly",
            (
                root + r"s4(\b|_|\[)",
                root + r"s4_token",
                root + r"token_raw_repaired",
            ),
        ),
        (
            f"{label} S4b token repair",
            (
                root + r"s4b(\b|_|\[)",
                root + r"s4b_token",
                root + r"s4_token_repaired",
            ),
        ),
        (
            f"{label} S5 macro transition",
            (
                root + r"s5(\b|_|\[)",
                root + r"s5_token",
                root + r"s5_fv_",
                root + r"s4b_token_for_s5",
                root + r"(state_|state_reg|p_active)",
                root + r"(l0_|l1_|lane0_|token_for_lane1|active_for_lane1)",
                root + r"(close_|touch_|tail_|alias_|head_prev_|data_prev_|output_)",
            ),
        ),
        (f"{label} S6a logical repair", (root + r"s6a(\b|_|\[)", root + r"s6_repair")),
        (
            f"{label} S6b physical writeback",
            (
                root + r"s6b(\b|_|\[)",
                root + r"skid_",
                root + r"curr_commit",
                root + r"next_2ppc",
                root + r"head_2ppc",
                root + r"data_2ppc",
                root + r"tail_2ppc",
                root + r"\$signature__(addr|data)",
            ),
        ),
    )


HIERARCHICAL_STAGE_RULES: StageRules = (
    *hier_2ppc_rules("core/outer_hier_2ppc", "Outer 2PPC"),
    *hier_2ppc_rules("core/inner_hier_2ppc", "Inner 2PPC"),
    (
        "Child arena merge/compact",
        (
            r"\bcore/child_arena/(merge_|child_arena_merge_|merge)",
        ),
    ),
    (
        "Child arena add/free/memory",
        (
            r"\bcore/child_arena/(add_|free_|free_count|free_next|free_linear)",
            r"\bcore/child_arena/(child_arena_mem|port_[ab]_|r_data_|w_data_)",
        ),
    ),
    (
        "Child arena read/output",
        (
            r"\bcore/child_arena/(read_|out_|out_valid|out_fv|out_last|out_overflow)",
        ),
    ),
    (
        "Child arena command/FIFO",
        (
            r"\bcore/child_arena/(cmd_|new_|id_[ab]|fsm_state|cmd_fifo)",
        ),
    ),
    (
        "Top hierarchical glue/IO",
        (
            r"\bcore/(col|row|inner_|outer_|fv_|d0|d1|sof|eol|datavalid|rst|in_ready)",
            r"\bcore/(child_id[01]?|output_)",
            *IO_RULES,
        ),
    ),
)

STAGE_RULES_BY_CORE: dict[str, StageRules] = {
    "hierarchical": HIERARCHICAL_STAGE_RULES,
}

PATH_ROW_RE = re.compile(
    r"^(?P<name>.+?)\s+(?P<model>\S+)\s+(?P<delay>-?\d+(?:\.\d+)?)\s+"
    r"(?P<cumulative>-?\d+(?:\.\d+)?)\s+(?P<pins>\d+)"
    r"(?:\s+(?P<location>\([^)]*\)))?\s*$"
)


@dataclass
class ClockSummary:
    target_period_ns: float | None = None
    target_frequency_mhz: float | None = None
    achieved_period_ns: float | None = None
    achieved_frequency_mhz: float | None = None


@dataclass
class TimingRow:
    name: str
    model: str
    delay_ns: float
    cumulative_ns: float
    pins: int
    location: str
    stage: str = "unclassified"


@dataclass
class TimingPath:
    section: str
    index: int
    begin: str = ""
    end: str = ""
    slack_ns: float | None = None
    delay_ns: float | None = None
    logic_level: int | None = None
    non_global_nets: int | None = None
    global_nets: int | None = None
    rows: list[TimingRow] = field(default_factory=list)

    @property
    def data_path_delay_ns(self) -> float:
        if not self.rows:
            return 0.0
        return self.rows[-1].cumulative_ns


@dataclass
class EfinityReport:
    path: Path
    clocks: dict[str, ClockSummary]
    max_paths: list[TimingPath]
    min_paths: list[TimingPath]
    path_source: str = "report_timing"


def classify(name: str, stage_rules: StageRules) -> str:
    normalized = name.replace("\\", "")
    for stage, patterns in stage_rules:
        if any(re.search(pattern, normalized) for pattern in patterns):
            return stage
    return "unclassified"


def classify_any(stage_rules: StageRules, *names: str) -> str:
    for name in names:
        stage = classify(name, stage_rules)
        if stage != "unclassified":
            return stage
    return "unclassified"


def fmt_name(name: str, limit: int) -> str:
    if len(name) <= limit:
        return name
    return "..." + name[-(limit - 3) :]


def fmt_ns(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.3f}"


def fmt_mhz(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.3f}"


def parse_float(text: str) -> float | None:
    match = re.search(r"-?\d+(?:\.\d+)?", text)
    return float(match.group(0)) if match else None


def parse_int(text: str) -> int | None:
    match = re.search(r"-?\d+", text)
    return int(match.group(0)) if match else None


def resolve_report_path(path: Path) -> Path:
    if path.is_file():
        return path
    def report_sort_key(report_path: Path) -> tuple[bool, str]:
        return (report_path.name.startswith("helloworld."), report_path.name)

    candidates = []
    candidates.extend(
        sorted((path / "outflow").glob("*.sta.out"), key=report_sort_key)
        if (path / "outflow").is_dir()
        else []
    )
    candidates.extend(
        sorted((path / "outflow").glob("*.timing.rpt"), key=report_sort_key)
        if (path / "outflow").is_dir()
        else []
    )
    candidates.extend(sorted(path.glob("*.sta.out"), key=report_sort_key))
    candidates.extend(sorted(path.glob("*.timing.rpt"), key=report_sort_key))
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"no Efinity *.sta.out or *.timing.rpt found under {path}")


def parse_clock_line(line: str) -> tuple[str, float, float] | None:
    parts = line.split()
    if len(parts) < 3:
        return None
    try:
        return parts[0], float(parts[1]), float(parts[2])
    except ValueError:
        return None


def parse_row(line: str) -> TimingRow | None:
    match = PATH_ROW_RE.match(line.rstrip())
    if not match:
        return None
    return TimingRow(
        name=match.group("name").strip(),
        model=match.group("model"),
        delay_ns=float(match.group("delay")),
        cumulative_ns=float(match.group("cumulative")),
        pins=int(match.group("pins")),
        location=match.group("location") or "",
    )


def parse_report(report_path: Path) -> EfinityReport:
    clocks: dict[str, ClockSummary] = {}
    max_paths: list[TimingPath] = []
    min_paths: list[TimingPath] = []

    clock_mode: str | None = None
    section: str | None = None
    current: TimingPath | None = None
    in_data_path = False
    path_source = "report_timing"

    def finish_current() -> None:
        nonlocal current
        if current is None:
            return
        if current.section == "max":
            max_paths.append(current)
        elif current.section == "min":
            min_paths.append(current)
        current = None

    for raw_line in report_path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.rstrip("\n")
        stripped = line.strip()

        if stripped.startswith("User target constrained clocks"):
            clock_mode = "target"
            continue
        if stripped.startswith("Maximum possible analyzed clocks frequency"):
            clock_mode = "achieved"
            continue
        if stripped.startswith("---------- Clock Frequency Summary (end)"):
            clock_mode = None
            continue
        if clock_mode and stripped and not stripped.startswith("Clock Name"):
            parsed_clock = parse_clock_line(stripped)
            if parsed_clock:
                clock, period, frequency = parsed_clock
                summary = clocks.setdefault(clock, ClockSummary())
                if clock_mode == "target":
                    summary.target_period_ns = period
                    summary.target_frequency_mhz = frequency
                else:
                    summary.achieved_period_ns = period
                    summary.achieved_frequency_mhz = frequency

        if "Path Details for Max Critical Paths (begin)" in stripped:
            finish_current()
            section = "max"
            continue
        if "Path Details for Max Critical Paths (end)" in stripped:
            finish_current()
            section = None
            continue
        if "Path Details for Min Critical Paths (begin)" in stripped:
            finish_current()
            section = "min"
            continue
        if "Path Details for Min Critical Paths (end)" in stripped:
            finish_current()
            section = None
            continue
        if stripped.startswith("INFO") and "Print delay paths report" in stripped:
            finish_current()
            path_source = "report_path"
            section = "max"
            in_data_path = False
            continue
        if stripped.startswith("INFO") and "Done printing reported path result" in stripped:
            finish_current()
            section = None
            in_data_path = False
            continue

        path_match = re.match(r"\+{4}\s+Path\s+(\d+)\s+\+{4,}", stripped)
        if path_match and section:
            finish_current()
            current = TimingPath(section=section, index=int(path_match.group(1)))
            in_data_path = False
            continue

        if current is None:
            continue

        if stripped.startswith("Path Begin"):
            current.begin = stripped.split(":", 1)[1].strip()
            continue
        if stripped.startswith("Path End"):
            current.end = stripped.split(":", 1)[1].strip()
            continue
        if stripped.startswith("Slack"):
            current.slack_ns = parse_float(stripped.split(":", 1)[1])
            continue
        if stripped.startswith("Delay"):
            current.delay_ns = parse_float(stripped.split(":", 1)[1])
            continue
        if stripped.startswith("Logic Level"):
            current.logic_level = parse_int(stripped.split(":", 1)[1])
            continue
        if stripped.startswith("Non-global nets on path"):
            current.non_global_nets = parse_int(stripped.split(":", 1)[1])
            continue
        if stripped.startswith("Global nets on path"):
            current.global_nets = parse_int(stripped.split(":", 1)[1])
            continue
        if stripped == "Data Path":
            in_data_path = True
            continue
        if stripped == "Capture Clock Path":
            in_data_path = False
            continue
        if not in_data_path:
            continue
        row = parse_row(line)
        if row is not None:
            current.rows.append(row)

    finish_current()
    return EfinityReport(
        path=report_path,
        clocks=clocks,
        max_paths=max_paths,
        min_paths=min_paths,
        path_source=path_source,
    )


def infer_row_stages(path: TimingPath, stage_rules: StageRules) -> None:
    current_stage = classify(path.begin, stage_rules)
    for row in path.rows:
        row_stage = classify(row.name, stage_rules)
        if row_stage != "unclassified":
            current_stage = row_stage
            row.stage = row_stage
        else:
            row.stage = current_stage


def path_delay(path: TimingPath) -> float:
    if path.delay_ns is not None:
        return path.delay_ns
    return path.data_path_delay_ns


def stage_pairs(paths: Iterable[TimingPath], stage_rules: StageRules) -> dict[tuple[str, str], list[TimingPath]]:
    result: dict[tuple[str, str], list[TimingPath]] = {}
    for path in paths:
        source = classify(path.begin, stage_rules)
        sink = classify(path.end, stage_rules)
        result.setdefault((source, sink), []).append(path)
    return result


def sink_stages(paths: Iterable[TimingPath], stage_rules: StageRules) -> dict[str, list[TimingPath]]:
    result: dict[str, list[TimingPath]] = {}
    for path in paths:
        result.setdefault(classify(path.end, stage_rules), []).append(path)
    return result


def summarize_grouped_paths(
    title: str,
    grouped: dict[str | tuple[str, str], list[TimingPath]],
    limit: int,
) -> None:
    print(title)
    items = sorted(
        grouped.items(),
        key=lambda item: max(path_delay(path) for path in item[1]),
        reverse=True,
    )
    for key, paths in items[:limit]:
        sorted_paths = sorted(paths, key=path_delay, reverse=True)
        worst = sorted_paths[0]
        if isinstance(key, tuple):
            label = f"{key[0]} -> {key[1]}"
        else:
            label = key
        print(
            f"{label}: worst {path_delay(worst):.3f} ns, "
            f"slack {fmt_ns(worst.slack_ns)} ns ({len(paths)} paths)"
        )
    print()


def summarize_path(path: TimingPath, stage_rules: StageRules, name_width: int) -> None:
    infer_row_stages(path, stage_rules)
    source = classify(path.begin, stage_rules)
    sink = classify(path.end, stage_rules)
    print(
        f"{path.index}. {source} -> {sink}: {path_delay(path):.3f} ns "
        f"slack {fmt_ns(path.slack_ns)} ns, logic {path.logic_level if path.logic_level is not None else 'n/a'}"
    )
    print(f"   {fmt_name(path.begin, name_width)} -> {fmt_name(path.end, name_width)}")
    print(f"   data-path cumulative: {path.data_path_delay_ns:.3f} ns")

    by_model: dict[str, float] = {}
    by_stage: dict[str, float] = {}
    routing_rows: list[TimingRow] = []
    for row in path.rows:
        by_model[row.model] = by_model.get(row.model, 0.0) + row.delay_ns
        by_stage[row.stage] = by_stage.get(row.stage, 0.0) + row.delay_ns
        if row.model == "net":
            routing_rows.append(row)

    if by_model:
        print(
            "   by model: "
            + ", ".join(f"{model} {delay:.3f} ns" for model, delay in sorted(by_model.items()))
        )
    if by_stage:
        print(
            "   by inferred stage: "
            + ", ".join(
                f"{stage} {delay:.3f} ns"
                for stage, delay in sorted(by_stage.items(), key=lambda item: item[1], reverse=True)[:5]
            )
        )
    for row in sorted(routing_rows, key=lambda item: item.delay_ns, reverse=True)[:3]:
        print(
            f"   route {row.delay_ns:.3f} ns {row.stage}: "
            f"{fmt_name(row.name, name_width)} pins={row.pins} loc={row.location}"
        )


def summarize_report(
    report: EfinityReport,
    stage_rules: StageRules,
    *,
    limit: int,
    critical_paths: int,
    name_width: int,
    include_min: bool,
) -> None:
    print("Classifier: efinity-hierarchical")
    print(f"Report: {report.path}")
    if report.clocks:
        print()
        print("Clock summary:")
        for clock, summary in sorted(report.clocks.items()):
            print(
                f"  {clock}: target {fmt_mhz(summary.target_frequency_mhz)} MHz "
                f"({fmt_ns(summary.target_period_ns)} ns), achieved "
                f"{fmt_mhz(summary.achieved_frequency_mhz)} MHz "
                f"({fmt_ns(summary.achieved_period_ns)} ns)"
            )
    print()
    if report.path_source == "report_path":
        main_label = "delay paths"
        source_name = "report_path"
    else:
        main_label = "max/setup paths"
        source_name = "report_timing"
    print(
        f"Classifying {len(report.max_paths)} {main_label} from Efinity {source_name}. "
        "Coverage follows the path caps used when this report was generated."
    )
    print()

    summarize_grouped_paths("Sink stages:", sink_stages(report.max_paths, stage_rules), limit)
    summarize_grouped_paths("Stage pairs:", stage_pairs(report.max_paths, stage_rules), limit)

    print(f"{main_label.capitalize()}:")
    for path in sorted(report.max_paths, key=path_delay, reverse=True)[:critical_paths]:
        summarize_path(path, stage_rules, name_width)
    print()

    if include_min:
        print(f"Classifying {len(report.min_paths)} min/hold paths from Efinity {source_name}.")
        print()
        summarize_grouped_paths("Hold sink stages:", sink_stages(report.min_paths, stage_rules), limit)
        summarize_grouped_paths("Hold stage pairs:", stage_pairs(report.min_paths, stage_rules), limit)
        print("Min/hold paths:")
        for path in sorted(report.min_paths, key=path_delay, reverse=True)[:critical_paths]:
            summarize_path(path, stage_rules, name_width)
        print()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "path",
        help="Efinity project directory, *.sta.out path, or *.timing.rpt path",
    )
    parser.add_argument("--core", choices=["auto", "hierarchical"], default="auto")
    parser.add_argument("--limit", type=int, default=8, help="stage groups to print")
    parser.add_argument("--critical-paths", type=int, default=5, help="critical paths to print")
    parser.add_argument("--name-width", type=int, default=82, help="truncate long names")
    parser.add_argument("--include-min", action="store_true", help="also classify hold/min paths")
    args = parser.parse_args()

    report_path = resolve_report_path(Path(args.path))
    report = parse_report(report_path)
    core = "hierarchical" if args.core == "auto" else args.core
    summarize_report(
        report,
        STAGE_RULES_BY_CORE[core],
        limit=args.limit,
        critical_paths=args.critical_paths,
        name_width=args.name_width,
        include_min=args.include_min,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
