#!/usr/bin/env python3
"""Describe missed tags without treating an ideal-mask diagnostic as an upper bound."""

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--oracle", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    dataset = json.loads(args.dataset.read_text())
    result = json.loads(args.result.read_text())
    truth = {frame["index"]: frame for frame in dataset["frames"]}
    oracle = {}
    if args.oracle:
        oracle = {
            frame["index"]: set(frame["matched_ids"])
            for frame in json.loads(args.oracle.read_text())["frames"]
        }
    failures = []
    for frame in result["frames"]:
        for tag in truth[frame["index"]]["tags"]:
            if tag["id"] in frame["matched_ids"]:
                continue
            expected = np.array(tag["centroids"])
            closest = None
            for attempt in frame["attempts"]:
                observed = np.array(attempt["points"])
                distances = ((observed[:, None, :] - expected[None, :, :]) ** 2).sum(2)
                rows, columns = linear_sum_assignment(distances)
                rms = float(np.sqrt(distances[rows, columns].mean()))
                if closest is None or rms < closest[0]:
                    closest = (rms, attempt, columns.tolist())
            size = max(tag["edge_lengths"])
            rms, attempt, assignment = closest or (None, None, None)
            if rms is None or rms > max(2, size * 0.04):
                cause = "no_nearby_nine_dot_candidate"
            elif attempt["accepted"]:
                cause = "wrong_id"
            elif attempt["rejection"] == "Some(CornerSelection)":
                masses = attempt.get("log_masses", [])
                cause = (
                    "corner_mass_tie"
                    if len(masses) == 9 and masses[6] == masses[7]
                    else "corner_selection"
                )
            else:
                cause = attempt["rejection"]
            failures.append({
                "frame": frame["index"], "id": tag["id"], "max_edge_px": size,
                "observed_failure": cause, "nearest_centroid_rms_px": rms,
                "nearest_attempt": attempt, "matched_canonical_dot_indices": assignment,
                "oracle_recovers": tag["id"] in oracle[frame["index"]]
                if frame["index"] in oracle else None,
            })
    counts = dict(Counter(row["observed_failure"] for row in failures))
    output = {
        "result": str(args.result), "config": result["config"],
        "note": "Failure labels describe observations, not definitive root causes. "
        "Oracle recovery shows that changing segmentation can help; absence of "
        "oracle recovery does not prove a threshold-independent failure.",
        "counts": counts, "failures": failures,
    }
    args.output.write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps(counts, sort_keys=True))


if __name__ == "__main__":
    main()
