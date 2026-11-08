#!/usr/bin/env python3
"""Summarize nextpnr timing by 2ppc pipeline stage.

For best results, generate the nextpnr report with --detailed-timing-report.
Without that field, nextpnr only reports the global critical clock path and IO
paths, so this script can only classify those few paths.
"""

from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any


StageRules = tuple[tuple[str, tuple[str, ...]], ...]

IO_RULES = (
    r"\bgpio_",
    r"\bfv_",
    r"\bdatavalid\b",
    r"\bd[01]\b",
    r"\bsof\b",
    r"\beol\b",
)

TWOPPC_STAGE_RULES: StageRules = (
    (
        "S0 ingress/rowbuf",
        (
            r"\bcore\.(accept|first_row|last_row|row_start|row_end|d[01]_eff)(?:\b|_)",
            r"\bcore\.(ingress_|drain|s0_|rowbuf_)",
            r"\bcore\.rowbuf_2ppc",
        ),
    ),
    ("S1 events/PF issue", (r"\bcore\.s1_", r"\bcore\.p_issue")),
    (
        "S2 PF0 reads",
        (
            r"\bcore\.s2_",
            r"\bcore\.pf0_",
            r"\bcore\.(next|head|data)_2ppc_.*_unused_read_sink",
        ),
    ),
    (
        "S3 PF1 resolve",
        (
            r"\bcore\.s3_",
            r"\bcore\.pf1_",
            r"\bcore\.token_raw(?!_repaired)",
        ),
    ),
    (
        "S4 token assembly",
        (
            r"\bcore\.s4_",
            r"\bcore\.s4_token",
            r"\bcore\.token_raw_repaired",
        ),
    ),
    (
        "S4b token repair",
        (
            r"\bcore\.s4b_",
            r"\bcore\.s4b_token",
            r"\bcore\.s4_token_repaired",
        ),
    ),
    (
        "S4b->S5 handoff",
        (
            r"\bcore\.s5_",
            r"\bcore\.s5_token",
            r"\bcore\.s4b_token_for_s5",
        ),
    ),
    (
        "S5 macro transition",
        (
            r"\bcore\.(state_|state_reg|p_active)",
            r"\bcore\.(l0_|l1_|lane0_|token_for_lane1|active_for_lane1)",
            r"\bcore\.(close_|touch_|tail_|alias_|head_prev_|data_prev_|output_)",
        ),
    ),
    (
        "S6a logical repair",
        (
            r"\bcore\.s6a_",
            r"\bcore\.s6_repair",
        ),
    ),
    (
        "S6b physical writeback",
        (
            r"\bcore\.s6b_",
            r"\bcore\.skid_",
            r"\bcore\.curr_commit",
            r"\bcore\.next_2ppc",
            r"\bcore\.head_2ppc",
            r"\bcore\.data_2ppc",
            r"\bcore\.tail_2ppc",
        ),
    ),
    ("IO", IO_RULES),
)

STAGE_RULES_BY_CORE: dict[str, StageRules] = {
    "2ppc": TWOPPC_STAGE_RULES,
}


@dataclass(frozen=True)
class TimingEndpoint:
    stage: str
    driver_stage: str
    delay_ns: float
    net_delay_ns: float
    driver: str
    net: str
    endpoint: str
    port: str
    event: str
    source: str


def classify(name: str, stage_rules: StageRules) -> str:
    for stage, patterns in stage_rules:
        if any(re.search(pattern, name) for pattern in patterns):
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


def to_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def fmt_number(value: Any) -> str:
    try:
        return f"{float(value):.2f}"
    except (TypeError, ValueError):
        return "n/a"


def endpoint_delay(endpoint: dict[str, Any]) -> tuple[float, float]:
    delay = endpoint.get("delay", 0.0)
    if isinstance(delay, list):
        if len(delay) >= 2:
            return to_float(delay[1]), to_float(delay[0])
        if len(delay) == 1:
            return to_float(delay[0]), 0.0
        return 0.0, 0.0
    return to_float(delay), 0.0


def iter_detailed_endpoints(
    report: dict[str, Any], stage_rules: StageRules
) -> list[TimingEndpoint]:
    endpoints: list[TimingEndpoint] = []
    for net_timing in report.get("detailed_net_timings", []):
        driver = str(net_timing.get("driver", ""))
        net = str(net_timing.get("net", ""))
        event = str(net_timing.get("event", ""))
        sources = net_timing.get("sources") or []
        source = str(sources[0]) if sources else ""
        for endpoint in net_timing.get("endpoints", []):
            cell = str(endpoint.get("cell", ""))
            port = str(endpoint.get("port", ""))
            delay_ns, net_delay_ns = endpoint_delay(endpoint)
            endpoints.append(
                TimingEndpoint(
                    stage=classify_any(stage_rules, f"{cell}.{port}", cell, net, driver),
                    driver_stage=classify_any(stage_rules, driver, net),
                    delay_ns=delay_ns,
                    net_delay_ns=net_delay_ns,
                    driver=driver,
                    net=net,
                    endpoint=cell,
                    port=port,
                    event=str(endpoint.get("event", event)),
                    source=source,
                )
            )
    return endpoints


def summarize_detailed(
    report: dict[str, Any], stage_rules: StageRules, limit: int, name_width: int
) -> None:
    endpoints = iter_detailed_endpoints(report, stage_rules)
    if not endpoints:
        print("No detailed timing endpoints found.")
        return

    by_stage: dict[str, list[TimingEndpoint]] = {}
    for endpoint in endpoints:
        by_stage.setdefault(endpoint.stage, []).append(endpoint)

    fmax = report.get("fmax", {})
    for clock_name, clock_report in fmax.items():
        if not isinstance(clock_report, dict):
            continue
        achieved = fmt_number(clock_report.get("achieved"))
        constraint = fmt_number(clock_report.get("constraint"))
        print(f"Clock {clock_name}: achieved {achieved} MHz, constraint {constraint} MHz")
    print()

    stages = [stage for stage, _ in stage_rules] + ["unclassified"]
    for stage in stages:
        stage_endpoints = sorted(
            by_stage.get(stage, ()),
            key=lambda item: item.delay_ns,
            reverse=True,
        )
        if not stage_endpoints:
            continue

        worst = stage_endpoints[0]
        print(f"{stage}: worst {worst.delay_ns:.2f} ns ({len(stage_endpoints)} endpoints)")
        for item in stage_endpoints[:limit]:
            source = f" source={item.source}" if item.source else ""
            print(
                "  "
                f"{item.delay_ns:6.2f} ns "
                f"net={item.net_delay_ns:5.2f} ns "
                f"{item.driver_stage} -> {item.stage} "
                f"{fmt_name(item.driver, name_width)} -> "
                f"{fmt_name(item.endpoint + '.' + item.port, name_width)}"
                f" net={fmt_name(item.net, name_width)}"
                f"{source}"
            )
        print()


def step_endpoint_name(endpoint: Any) -> str:
    if isinstance(endpoint, dict):
        cell = str(endpoint.get("cell", ""))
        port = str(endpoint.get("port", ""))
        return f"{cell}.{port}" if port else cell
    return str(endpoint)


def path_stage(path: dict[str, Any], stage_rules: StageRules) -> str:
    steps = path.get("path", [])
    if not steps:
        return classify(str(path.get("to", "")), stage_rules)
    for step in reversed(steps):
        stage = classify_any(
            stage_rules,
            step_endpoint_name(step.get("to", {})),
            str(step.get("net", "")),
            step_endpoint_name(step.get("from", {})),
        )
        if stage != "unclassified":
            return stage
    return "unclassified"


def path_total_delay(path: dict[str, Any]) -> float:
    return sum(to_float(step.get("delay", 0.0)) for step in path.get("path", []))


def summarize_critical_paths(
    report: dict[str, Any],
    stage_rules: StageRules,
    name_width: int,
    limit: int,
    *,
    detailed_report_available: bool,
) -> None:
    if not detailed_report_available:
        print(
            "Report does not include detailed_net_timings; rerun nextpnr with "
            "--detailed-timing-report for per-stage endpoint coverage."
        )
        print("Classifying the limited critical_paths list instead.\n")
    else:
        print("Critical paths:")

    for index, path in enumerate(report.get("critical_paths", [])[:limit], start=1):
        steps = path.get("path", [])
        first = steps[0].get("from", {}) if steps else {}
        last = steps[-1].get("to", {}) if steps else {}
        first_cell = step_endpoint_name(first) if steps else str(path.get("from", ""))
        last_cell = step_endpoint_name(last) if steps else str(path.get("to", ""))
        source_stage = classify_any(stage_rules, first_cell)
        sink_stage = path_stage(path, stage_rules)
        delay_by_type: dict[str, float] = {}
        delay_by_stage: dict[str, float] = {}
        routing_steps: list[tuple[float, str, str]] = []
        for step in steps:
            delay = to_float(step.get("delay", 0.0))
            step_type = str(step.get("type", "unknown"))
            delay_by_type[step_type] = delay_by_type.get(step_type, 0.0) + delay
            net = str(step.get("net", ""))
            step_stage = classify_any(
                stage_rules,
                net,
                step_endpoint_name(step.get("to", {})),
                step_endpoint_name(step.get("from", {})),
            )
            delay_by_stage[step_stage] = delay_by_stage.get(step_stage, 0.0) + delay
            if step_type == "routing":
                routing_steps.append((delay, net, step_stage))

        type_bits = ", ".join(
            f"{step_type} {delay:.2f} ns"
            for step_type, delay in sorted(delay_by_type.items(), key=lambda item: item[0])
        )
        stage_bits = ", ".join(
            f"{stage} {delay:.2f} ns"
            for stage, delay in sorted(delay_by_stage.items(), key=lambda item: item[1], reverse=True)[:4]
        )
        print(
            f"{index}. {source_stage} -> {sink_stage}: {path_total_delay(path):.2f} ns "
            f"{fmt_name(first_cell, name_width)} -> {fmt_name(last_cell, name_width)}"
        )
        if type_bits:
            print(f"   by type: {type_bits}")
        if stage_bits:
            print(f"   by stage: {stage_bits}")
        print(
            "   domain: "
            f"{fmt_name(str(path.get('from', '')), name_width)} -> "
            f"{fmt_name(str(path.get('to', '')), name_width)}"
        )
        for delay, net, stage in sorted(routing_steps, reverse=True)[:3]:
            print(f"   route {delay:.2f} ns {stage}: net={fmt_name(net, name_width)}")
    if detailed_report_available:
        print()


def detect_core(report_path: Path, report: dict[str, Any]) -> str:
    del report_path, report
    return "2ppc"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "report",
        help="nextpnr report JSON",
    )
    parser.add_argument("--core", choices=["auto", "2ppc"], default="auto")
    parser.add_argument("--limit", type=int, default=3, help="endpoints to print per stage")
    parser.add_argument(
        "--critical-paths",
        type=int,
        default=1,
        help="critical paths to print after the stage summary",
    )
    parser.add_argument("--name-width", type=int, default=72, help="truncate long cell/net names")
    args = parser.parse_args()

    report_path = Path(args.report)
    with report_path.open("r", encoding="utf-8") as handle:
        report = json.load(handle)

    core = detect_core(report_path, report) if args.core == "auto" else args.core
    stage_rules = STAGE_RULES_BY_CORE[core]
    print(f"Classifier: {core}\n")

    if "detailed_net_timings" in report:
        summarize_detailed(report, stage_rules, args.limit, args.name_width)
        if args.critical_paths:
            summarize_critical_paths(
                report,
                stage_rules,
                args.name_width,
                args.critical_paths,
                detailed_report_available=True,
            )
    else:
        summarize_critical_paths(
            report,
            stage_rules,
            args.name_width,
            args.critical_paths,
            detailed_report_available=False,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
