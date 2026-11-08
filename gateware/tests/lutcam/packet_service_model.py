#!/usr/bin/env python3
"""Replay timestamped production outputs through an explicit packet service model.

Input lines describe ONE camera; the model duplicates them onto two simultaneous
cameras. Format: ``cycle I frame data_hex sof padding``,
``cycle C frame data_hex end``, or ``cycle P frame data_hex last``.

The model keeps assembly slots allocated until their final datagram is sent.
Pure image padding uses one descriptor for all remaining frame fragments. Pose
rejections allocate no RAM; accepted poses reserve early at their first word
and commit two clocks after their last word to account for prefix replay.

This is an arrival-trace replay under a conservative, stated service schedule,
not a universal sensor-traffic proof or a measured LiteEth latency bound.
"""

import argparse
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
import heapq
import json
from pathlib import Path


KINDS = {"I": 0, "C": 1, "P": 2}
NAMES = ("image", "children", "pose")


@dataclass(frozen=True)
class Word:
    cycle: int
    kind: str
    frame: int
    data: int
    sof: bool = False
    padding: bool = False
    last: bool = False


@dataclass(frozen=True)
class Descriptor:
    kind: str
    frame: int
    allocate: int
    commit: int
    payload_bytes: int
    virtual: bool = False


@dataclass
class Pending:
    descriptor: Descriptor
    remaining: int
    committed: bool = False


def read_trace(path):
    words = []
    previous_cycle = -1
    for line_number, line in enumerate(Path(path).read_text().splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split()
        try:
            cycle, kind, frame = int(fields[0]), fields[1], int(fields[2], 0)
            data = int(fields[3], 16)
            if cycle < previous_cycle or cycle < 0 or kind not in KINDS:
                raise ValueError("invalid timestamp or stream type")
            if kind == "I" and len(fields) == 6:
                word = Word(cycle, kind, frame, data, bool(int(fields[4])), bool(int(fields[5])))
            elif kind in ("C", "P") and len(fields) == 5:
                word = Word(cycle, kind, frame, data, last=bool(int(fields[4])))
            else:
                raise ValueError("wrong field count")
            limit = {"I": 16, "C": 33, "P": 32}[kind]
            if data < 0 or data >= 1 << limit:
                raise ValueError("data exceeds stream width")
        except (ValueError, IndexError) as error:
            raise ValueError(f"{path}:{line_number}: {error}") from error
        words.append(word)
        previous_cycle = cycle
    return words


def assemble(words, frame_words=64000, fragment_words=512, early_padding=False):
    """Build one-camera allocation/commit events and per-frame measurements."""
    if frame_words < 2 or frame_words % 2 or fragment_words < 2 or fragment_words % 2:
        raise ValueError("frame and fragment sizes must be positive even word counts")
    descriptors = []
    measurements = defaultdict(lambda: Counter())
    image = None
    child = []
    pose = []
    for word in words:
        stats = measurements[word.frame]
        stats[f"{NAMES[KINDS[word.kind]]}_words"] += 1
        if word.kind == "I":
            if word.sof:
                if image is not None:
                    raise ValueError(f"cycle {word.cycle}: new SOF before prior image completed")
                image = {"frame": word.frame, "count": 0, "start": None,
                         "fragment_start": 0, "padding": False, "padding_cycle": None}
            if image is None or word.frame != image["frame"]:
                raise ValueError(f"cycle {word.cycle}: image without matching active frame")
            if image["padding"] and not word.padding:
                raise ValueError(f"cycle {word.cycle}: padding marker withdrawn")
            if word.padding and word.data:
                raise ValueError(f"cycle {word.cycle}: nonzero pure-padding word")
            if image["start"] is None:
                image["start"] = word.cycle
                image["fragment_start"] = image["count"]
            if word.padding and not image["padding"]:
                image["padding_cycle"] = word.cycle
            image["padding"] |= word.padding
            stats["lwc_padding_words" if word.padding else "lwc_semantic_words"] += 1
            image["count"] += 1
            final = image["count"] == frame_words
            ordinary_fragment_done = (image["count"] - image["fragment_start"] == fragment_words)
            if final or (ordinary_fragment_done and not image["padding"]):
                payload_words = image["count"] - image["fragment_start"]
                commit_cycle = (image["padding_cycle"] if early_padding and image["padding"] else word.cycle)
                descriptors.append(Descriptor("I", word.frame, image["start"], commit_cycle,
                                              payload_words * 2, image["padding"]))
                image["start"] = None
            if final:
                stats["complete_image_frames"] += 1
                image = None
        elif word.kind == "C":
            if child and child[0].frame != word.frame:
                raise ValueError(f"cycle {word.cycle}: child frame changed inside a group")
            child.append(word)
            if len(child) > 9:
                raise ValueError(f"cycle {word.cycle}: child group exceeds nine records")
            if word.last:
                descriptors.append(Descriptor("C", word.frame, child[0].cycle,
                                              word.cycle, len(child) * 8))
                stats["child_groups"] += 1
                child = []
        else:
            if pose and pose[0].frame != word.frame:
                raise ValueError(f"cycle {word.cycle}: pose frame changed inside a native packet")
            pose.append(word)
            if len(pose) > 22:
                raise ValueError(f"cycle {word.cycle}: native pose packet exceeds 22 words")
            if word.last:
                marker = len(pose) >= 2 and bool(pose[1].data & 0x80000000)
                if len(pose) == 8 and not marker:
                    stats["solver_rejections"] += 1
                elif (len(pose) == 22 and marker and pose[8].data & 1
                      and pose[8].data & 0xFFFFFFF0 == 0):
                    descriptors.append(Descriptor("P", word.frame, pose[0].cycle,
                                                  word.cycle + 2, 88))
                    stats["accepted_poses"] += 1
                else:
                    raise ValueError(f"cycle {word.cycle}: malformed native solver result")
                pose = []
    if image is not None or child or pose:
        raise ValueError("trace ends inside an image, child group, or native solver packet")
    for descriptor in descriptors:
        datagrams = ((descriptor.payload_bytes + fragment_words * 2 - 1) // (fragment_words * 2)
                     if descriptor.virtual else 1)
        measurements[descriptor.frame][f"{NAMES[KINDS[descriptor.kind]]}_datagrams"] += datagrams
    for stats in measurements.values():
        for field in ("image_words", "children_words", "pose_words", "lwc_semantic_words",
                      "lwc_padding_words", "complete_image_frames", "child_groups",
                      "accepted_poses", "solver_rejections", "image_datagrams",
                      "children_datagrams", "pose_datagrams"):
            stats.setdefault(field, 0)
    return descriptors, {str(frame): dict(stats) for frame, stats in sorted(measurements.items())}


def replay(descriptors, fragment_words=512, etherbone_words=16, implementation_gap=8,
           capacities=(2, 4, 16)):
    """Return queue peaks for two identically timed copies of a descriptor trace."""
    if not 0 <= etherbone_words <= 256 or implementation_gap < 0:
        raise ValueError("invalid Etherbone size or implementation gap")
    if any(capacity < 1 for capacity in capacities):
        raise ValueError("hypothetical slot capacities must be positive")
    events = []
    pending = {}
    serial = 0
    for camera in range(2):
        for descriptor in descriptors:
            if descriptor.allocate < 0 or descriptor.commit < descriptor.allocate or descriptor.payload_bytes <= 0:
                raise ValueError("invalid descriptor timing or length")
            stream = camera * 3 + KINDS[descriptor.kind]
            item = Pending(descriptor, descriptor.payload_bytes)
            pending[serial] = item
            # ALLOC precedes COMMIT; both precede a same-cycle dequeue.
            heapq.heappush(events, (descriptor.allocate, 0, stream, serial))
            heapq.heappush(events, (descriptor.commit, 1, stream, serial))
            serial += 1
    queues = [deque() for _ in range(6)]
    report = [{"peak_slots": 0, "peak_cycle": None,
               "first_overflow_cycles": {str(capacity): None for capacity in capacities},
               "datagrams": 0, "udp_bytes": 0, "committed_descriptors": 0}
              for _ in range(6)]
    cursor = 0
    job = None
    now = 0
    total_datagrams = total_udp = total_wire = 0
    eb_udp_bytes = 8 + 4 + 4 + 4 * etherbone_words
    eb_cycles = ((eb_udp_bytes + 66) * 24 + 24) // 25 + implementation_gap
    first_allocation = min((descriptor.allocate for descriptor in descriptors), default=0)

    def snapshot():
        return [{"stream": f"camera{stream // 3}.{NAMES[stream % 3]}",
                 "allocated": len(queue),
                 "committed": sum(item.committed for item in queue),
                 "head_frame": queue[0].descriptor.frame if queue else None,
                 "head_payload_remaining": queue[0].remaining if queue else 0}
                for stream, queue in enumerate(queues)]

    while events or job is not None:
        now = min(events[0][0] if events else float("inf"),
                  job[0] if job is not None else float("inf"))
        allocated_streams = set()
        while events and events[0][0] == now:
            _, event_kind, stream, identity = heapq.heappop(events)
            item = pending[identity]
            if event_kind == 0:
                queues[stream].append(item)
                allocated_streams.add(stream)
            else:
                item.committed = True
                report[stream]["committed_descriptors"] += 1
        for stream in allocated_streams:
            count = len(queues[stream])
            stats = report[stream]
            if count > stats["peak_slots"]:
                stats["peak_slots"] = count
                stats["peak_cycle"] = now
                stats["peak_context"] = snapshot()
            for capacity in capacities:
                if count > capacity and stats["first_overflow_cycles"][str(capacity)] is None:
                    stats["first_overflow_cycles"][str(capacity)] = now
        if job is not None and job[0] == now:
            _, stream, payload, udp_bytes = job
            item = queues[stream][0]
            item.remaining -= payload
            if item.remaining == 0:
                queues[stream].popleft()
            report[stream]["datagrams"] += 1
            report[stream]["udp_bytes"] += udp_bytes
            total_datagrams += 1
            total_udp += udp_bytes
            total_wire += udp_bytes + 66 + eb_udp_bytes + 66
            cursor = (stream + 1) % 6
            job = None
        if job is None:
            for step in range(6):
                stream = (cursor + step) % 6
                if not queues[stream] or not queues[stream][0].committed:
                    continue
                item = queues[stream][0]
                payload = min(item.remaining, fragment_words * 2) if item.descriptor.virtual else item.remaining
                udp_bytes = 32 + payload
                data_cycles = ((udp_bytes + 66) * 24 + 24) // 25 + implementation_gap
                job = (now + eb_cycles + data_cycles, stream, payload, udp_bytes)
                break
    if any(queues):
        raise ValueError("uncommitted descriptor remained after trace replay")
    for stats in report:
        stats["smallest_power_of_two_depth"] = max(2, 1 << (max(1, stats["peak_slots"]) - 1).bit_length())
    return {
        "model": {
            "clock_hz": 120000000, "link_bps": 1000000000,
            "wire_overhead_bytes_per_datagram": 66,
            "etherbone_response_words": etherbone_words,
            "etherbone_udp_bytes": eb_udp_bytes,
            "etherbone_replies_before_each_data_datagram": 1,
            "implementation_gap_cycles_per_datagram": implementation_gap,
            "implementation_gap_applies_to_etherbone": True,
            "alignment": "one camera trace duplicated onto two simultaneous cameras",
            "pose_reservation": "first native word; commit delayed two cycles for prefix replay",
            "assumptions": ["established full-duplex 1 Gb/s link", "pre-resolved destinations",
                            "no ARP or ICMP traffic competing for transmit service",
                            "stated implementation gap is a model assumption, not a proved LiteEth bound"],
        },
        "streams": {f"camera{stream // 3}.{NAMES[stream % 3]}": stats
                    for stream, stats in enumerate(report)},
        "totals": {"data_datagrams": total_datagrams, "udp_bytes": total_udp,
                   "wire_bytes_including_etherbone": total_wire,
                   "first_allocation_cycle": first_allocation,
                   "last_service_cycle": now, "elapsed_cycles": now - first_allocation},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", required=True, type=Path)
    parser.add_argument("--etherbone-words", type=int, default=16)
    parser.add_argument("--implementation-gap", type=int, default=8,
                        help="assumed extra clocks for EACH data and Etherbone datagram")
    parser.add_argument("--slots", type=int, nargs="+", default=[2, 4, 16],
                        help="report first over-capacity arrival for each hypothetical uniform depth")
    parser.add_argument("--early-padding", action="store_true",
                        help="model committing a complete virtual tail on its first pure-padding marker")
    parser.add_argument("--frame-words", type=int, default=64000)
    parser.add_argument("--fragment-words", type=int, default=512)
    args = parser.parse_args()
    words = read_trace(args.trace)
    descriptors, frames = assemble(words, args.frame_words, args.fragment_words, args.early_padding)
    result = replay(descriptors, args.fragment_words, args.etherbone_words,
                    args.implementation_gap, args.slots)
    result["model"]["virtual_padding_commit"] = ("first pure-padding marker" if args.early_padding
                                                    else "last physical frame word")
    result["trace"] = {"path": str(args.trace), "word_events": len(words),
                       "single_camera_descriptors": len(descriptors),
                       "first_input_cycle": words[0].cycle if words else None,
                       "last_input_cycle": words[-1].cycle if words else None,
                       "per_frame_single_camera": frames}
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
