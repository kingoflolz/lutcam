#!/usr/bin/env python3
"""Compare actual and ideal-mask dot masses in diagnostic cases.

Ground truth is used only for attribution. Pixel sums are checked against the
unchanged production benchmark's Q5.3 mass descriptors before interpreting them.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import label
from scipy.optimize import linear_sum_assignment


def log_mass(value):
    value = int(value)
    if value == 0:
        return 0
    exponent = value.bit_length() - 1
    normalized = value >> (exponent - 21) if exponent >= 21 else value << (21 - exponent)
    fraction = normalized - (1 << 21)
    code = int(np.searchsorted([189812, 396812, 622506, 868669, 1137124, 1429837, 1749040],
                               fraction, side="right"))
    if fraction == 1749041:
        code = 6
    return min(255, exponent * 8 + code)


def closest_attempt(frame, tag, plane):
    expected = np.array(tag["centroids"])
    best = None
    for attempt in frame["attempts"]:
        if attempt["plane"] != plane:
            continue
        observed = np.array(attempt["points"])
        distance = ((observed[:, None] - expected[None]) ** 2).sum(2)
        rows, columns = linear_sum_assignment(distance)
        rms = np.sqrt(distance[rows, columns].mean())
        if best is None or rms < best[0]:
            keys = np.zeros(9, dtype=int)
            keys[columns] = np.array(attempt["log_masses"])[rows]
            best = (rms, keys.tolist(), attempt)
    if best is None or best[0] > max(2, .04 * max(tag["edge_lengths"])):
        return None
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--masks", type=Path, required=True)
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--results", type=Path, nargs="+", required=True)
    parser.add_argument("--oracle-result", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--all-cases", action="store_true",
                        help="Inspect all listed failures, including ones not recovered by ideal masks")
    args = parser.parse_args()
    manifest = json.loads((args.dataset / "manifest.json").read_text())
    actual = {f["index"]: f for path in args.results
              for f in json.loads(path.read_text())["frames"]}
    ideal = ({f["index"]: f for f in json.loads(args.oracle_result.read_text())["frames"]}
             if args.oracle_result else {})
    cases = [r for r in json.loads(args.cases.read_text())["failures"]
             if args.all_cases or not args.oracle_result or r["oracle_recovers"]]
    report = []
    for case in cases:
        index, tag_id = case["frame"], case["id"]
        tag = next(t for t in manifest["frames"][index]["tags"] if t["id"] == tag_id)
        recovered_plane = next((a["plane"] for a in ideal.get(index, {}).get("attempts", [])
                                if a["accepted"] and a["id"] == tag_id), None)
        plane = recovered_plane or case["nearest_attempt"]["plane"]
        level, scale, phase = {"Raw": (0, 1, 0), "Ll2": (1, 4, 0), "Ll4": (2, 16, 4)}[plane]
        prefix = args.masks / f"adaptive_{index:03}"
        black = np.array(Image.open(str(prefix) + f"_{level}.png")) == 0
        labels = np.array(Image.open(args.dataset / "oracle" / f"frame_{index:03}_{plane.lower()}.png"))
        ideal_black = np.where(labels > 0, labels == 2, black)
        pixels = np.array(Image.open(args.masks / f"frame_{index:03}_tap{level}_i16.png"), dtype=int) - 32768
        threshold = np.array(Image.open(str(prefix) + "_reference.png")) if level == 0 else None
        if level:
            pixels = np.clip(pixels + 128, 0, 255)
            curve = np.floor(255 * ((255 - np.arange(256)) / 255) ** 2.5 + .5)
        else:
            dark = np.clip(279 - np.arange(256), 0, 255)
            curve = np.floor(.85 * np.floor(255 * (dark / 255) ** 5 + .5) + .5)
        curve = np.minimum(31, (curve.astype(int) + 4) >> 3)
        weights = curve[pixels].astype(int)
        base_black = (np.array(Image.open(str(prefix) + "_base.png")) == 0
                      if level == 0 else None)
        actual_labels, _ = label(black)
        ideal_labels, _ = label(ideal_black)
        dots = []
        for dot, center in enumerate(tag["centroids"]):
            center = (np.array(center) - [0, phase]) / scale
            x, y = np.floor(center + .5).astype(int)
            ac, ic = int(actual_labels[y, x]), int(ideal_labels[y, x])
            if not ac or not ic:
                raise ValueError(f"Dot center outside black component: {index}/{tag_id}/{dot}")
            a = actual_labels == ac
            b = ideal_labels == ic
            if level == 0:
                a &= base_black
            mass_a, mass_b = int(weights[a].sum()), int(weights[b].sum())
            changes = []
            for py, px in zip(*np.nonzero(a != b)):
                changes.append({"x": int(px), "y": int(py), "pixel": int(pixels[py, px]),
                                "threshold": int(threshold[py, px]) if threshold is not None else None,
                                "weight": int(weights[py, px]), "actual": bool(a[py, px]),
                                "ideal": bool(b[py, px])})
            dots.append({"dot": dot, "anchor": dot < 2, "actual_pixels": int(a.sum()),
                         "ideal_pixels": int(b.sum()), "actual_mass": mass_a, "ideal_mass": mass_b,
                         "actual_component": ac, "ideal_component": ic,
                         "actual_log_mass": log_mass(mass_a), "ideal_log_mass": log_mass(mass_b),
                         "changed_pixels": changes})
        observed = closest_attempt(actual[index], tag, plane)
        oracle = closest_attempt(ideal.get(index, {"attempts": []}), tag, plane)
        actual_keys = [d["actual_log_mass"] for d in dots]
        ideal_keys = [d["ideal_log_mass"] for d in dots]
        if observed is None or actual_keys != observed[1]:
            raise ValueError(f"Actual pixel-sum descriptors do not match production for {index}/{tag_id}: "
                             f"{actual_keys} vs {None if observed is None else observed[1]}")
        if oracle is not None and ideal_keys != oracle[1]:
            raise ValueError(f"Ideal pixel-sum descriptors do not match diagnostic for {index}/{tag_id}: "
                             f"{ideal_keys} vs {oracle[1]}")
        report.append({"frame": index, "id": tag_id, "plane": plane,
                       "size": max(tag["edge_lengths"]), "dots": dots,
                       "actual_anchor_gap": min(actual_keys[:2]) - max(actual_keys[2:]),
                       "ideal_anchor_gap": min(ideal_keys[:2]) - max(ideal_keys[2:]),
                       "actual_distinct_dot_components": len({d["actual_component"] for d in dots}),
                       "ideal_distinct_dot_components": len({d["ideal_component"] for d in dots}),
                       "actual_mass_keys_match_production": True,
                       "ideal_mass_keys_match_production": True if oracle is not None else None,
                       "mass_keys_match_production": oracle is not None,
                       "ideal_nearby_candidate": oracle is not None})
        print(index, tag_id, plane, actual_keys, "->", ideal_keys,
              "" if oracle is not None else "(no nearby oracle candidate)", flush=True)
    args.output.write_text(json.dumps({"frontend": "adaptive",
        "note": "Actual pixel sums independently match production mass descriptors. "
                "Ideal sums are checked when a nearby nine-dot oracle candidate exists; otherwise "
                "the report retains the pixel sums without claiming a production match. Ideal masks are diagnostic only.",
        "cases": report}, indent=2) + "\n")


if __name__ == "__main__":
    main()
