"""Check LUTCAM Etherbone directly, including loss and restored scalar writes.

Use the CSR CSV from the programmed build. Keep other configuration clients
idle during this check. No camera is started and calibration is never committed.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import ipaddress
import json
from pathlib import Path
import socket
import time


BURST_SIZES = (1, 2, 4, 8)
PATTERNS = (0x12345678, 0x87654321, 0xffffffff, 0)


class CheckFailure(RuntimeError):
    """The connection or register contents failed a check."""


def load_map(path):
    """Read addresses and the exact expected identifier without opening a link."""
    registers, bases, constants = {}, {}, {}
    with Path(path).open(encoding="utf-8", newline="") as source:
        for row in csv.reader(line for line in source if line.strip() and not line.startswith("#")):
            kind, name, value, size, mode = row
            if kind == "csr_register":
                registers[name] = (int(value, 0), int(size), mode)
            elif kind == "csr_base":
                bases[name] = int(value, 0)
            elif kind == "constant":
                constants[name] = value
    for name in ("config_csr_data_width", "config_csr_alignment", "config_bus_address_width"):
        if constants.get(name) != "32":
            raise ValueError(f"{name} must be 32 in the selected CSV")
    identifier = constants["config_identifier"].encode("ascii") + b"\0"
    if not 8 <= len(identifier) <= 256:
        raise ValueError("unexpected identifier length in the selected CSV")
    return {"registers": registers, "identifier_base": bases["identifier_mem"],
            "identifier": identifier}


def bounded_probe(comm, attempts, timeout):
    """Probe with an explicit attempt limit; do not hide failed probe attempts."""
    from litex.tools.remote.etherbone import EtherbonePacket

    request = EtherbonePacket(comm.addr_width)
    request.pf = 1
    request.encode()
    payload = request.bytes + bytes(4)
    result = {"attempts": 0, "timeouts": 0, "unexpected_packets": 0, "ok": False}
    try:
        for attempt in range(attempts):
            result["attempts"] = attempt + 1
            comm.socket.sendto(payload, (comm.server, comm.port))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                comm.socket.settimeout(max(0.001, deadline - time.monotonic()))
                try:
                    data, peer = comm.socket.recvfrom(8192)
                except socket.timeout:
                    break
                if peer != (comm.server, comm.port):
                    result["unexpected_packets"] += 1
                    continue
                try:
                    response = EtherbonePacket(comm.addr_width, data)
                    response.decode()
                    valid = response.pr == 1
                except (ValueError, AssertionError, IndexError):
                    valid = False
                if valid:
                    result["ok"] = True
                    return result
                result["unexpected_packets"] += 1
            result["timeouts"] += 1
        return result
    finally:
        comm.socket.settimeout(timeout)


def check_connection(comm, mapping, *, iterations=100, read_only=False):
    """Run checks on an already open CommUDP-compatible transport.

    CommUDP retries a timed-out read internally up to ten times. The difference
    in its read_counter exposes every attempt, including exhausted reads.
    """
    if iterations < 1:
        raise ValueError("iterations must be positive")
    report = {"ok": False, "errors": [], "iterations_requested": iterations,
              "iterations_completed": 0, "read_calls": 0, "read_attempts": 0,
              "read_retries": 0, "read_failures": 0, "burst_sizes": list(BURST_SIZES),
              "write_test": {"enabled": not read_only, "patterns_verified": 0,
                             "restore_attempted": False, "restored": None}}
    registers = mapping["registers"]
    durations = []
    original = None
    write_attempted = False

    def read(address, length=None):
        if length is not None and not 1 <= length <= 8:
            raise ValueError("read bursts must contain one through eight words")
        before = comm.read_counter
        start = time.monotonic()
        report["read_calls"] += 1
        try:
            value = comm.read(address, length=length)
            if length is not None and len(value) != length:
                raise CheckFailure(f"read at 0x{address:08x} returned the wrong word count")
            return value
        except Exception:
            report["read_failures"] += 1
            raise
        finally:
            attempts = comm.read_counter - before
            report["read_attempts"] += attempts
            report["read_retries"] += max(0, attempts - 1)
            durations.append(time.monotonic() - start)

    def register(name):
        address, words, mode = registers[name]
        if words != 1 or mode not in ("ro", "rw"):
            raise ValueError(f"{name} must be a readable scalar register")
        return read(address)

    def camera_state(index):
        run = register(f"camera{index}_run")
        status = register(f"camera{index}_status")
        return {"run": run, "status": status, "idle": bool(status & 2)}

    def require_stopped():
        state = camera_state(0)
        if state["run"] != 0 or not state["idle"]:
            raise CheckFailure("camera 0 must be stopped and idle for the restored write test")

    def diagnostics():
        result = {}
        for host in ("etherbone", "usb"):
            prefix = f"{host}_diagnostics_"
            if prefix + "status" in registers:
                result[host] = {field: register(prefix + field)
                                for field in ("status", "address", "count")}
        return result

    try:
        report["board_status"] = register("board_status")
        if report["board_status"] & 3 != 3:
            raise CheckFailure("board_status does not report both PLL lock and PHY reset release")
        report["cameras"] = {str(i): camera_state(i) for i in (0, 1)
                              if f"camera{i}_run" in registers}
        report["diagnostics_before"] = diagnostics()
        expected = mapping["identifier"]
        observed = []
        for offset in range(0, len(expected), 8):
            observed.extend(read(mapping["identifier_base"] + 4 * offset,
                                 min(8, len(expected) - offset)))
        report["identifier_expected"] = expected[:-1].decode("ascii")
        report["identifier_observed"] = "".join(chr(word) if 0 <= word <= 255 else "?"
                                                  for word in observed).rstrip("\0")
        if observed != list(expected):
            raise CheckFailure("identifier differs from the selected CSR CSV")
        if not read_only:
            require_stopped()
            scratch_address, words, mode = registers["camera0_cfg_cx_q16_16"]
            if words != 1 or mode != "rw":
                raise ValueError("camera0_cfg_cx_q16_16 must be a read/write scalar register")
            original = read(scratch_address)
            report["write_test"].update(address=scratch_address, original=original)
        for iteration in range(iterations):
            size = BURST_SIZES[iteration % len(BURST_SIZES)]
            offset = (iteration * 7) % (len(expected) - size + 1)
            values = read(mapping["identifier_base"] + 4 * offset, size)
            if values != list(expected[offset:offset + size]):
                raise CheckFailure(f"identifier burst mismatch on iteration {iteration + 1}")
            if not read_only:
                require_stopped()
                pattern = PATTERNS[iteration % len(PATTERNS)]
                write_attempted = True  # A send error can occur after delivery.
                comm.write(scratch_address, pattern)
                if read(scratch_address) != pattern:
                    raise CheckFailure(f"scalar write/readback mismatch on iteration {iteration + 1}")
                report["write_test"]["patterns_verified"] += 1
            report["iterations_completed"] += 1
    except (Exception, KeyboardInterrupt) as error:
        report["errors"].append(f"{type(error).__name__}: {error}")
    finally:
        if write_attempted:
            report["write_test"]["restore_attempted"] = True
            try:
                comm.write(scratch_address, original)
                if read(scratch_address) != original:
                    raise CheckFailure("original calibration value did not read back after restoration")
                report["write_test"]["restored"] = True
            except Exception as error:
                report["write_test"]["restored"] = False
                report["errors"].append(f"RESTORATION FAILED: {type(error).__name__}: {error}")
        try:
            report["diagnostics_after"] = diagnostics()
        except Exception as error:
            report["errors"].append(f"final diagnostics: {type(error).__name__}: {error}")
    for when in ("before", "after"):
        for host, values in report.get(f"diagnostics_{when}", {}).items():
            if values["status"] or values["count"]:
                report["errors"].append(f"{host} diagnostics {when}: {values}")
    if report["read_retries"]:
        report["errors"].append(f"CommUDP required {report['read_retries']} read retries")
    if durations:
        report["read_latency_ms"] = {"minimum": min(durations) * 1000,
            "mean": sum(durations) * 1000 / len(durations), "maximum": max(durations) * 1000}
    report["ok"] = not report["errors"]
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csr-csv", required=True, type=Path,
                        help="CSR CSV from the exact programmed build")
    parser.add_argument("--target", default="192.168.1.50", type=ipaddress.IPv4Address)
    parser.add_argument("--port", default=1234, type=int)
    parser.add_argument("--timeout", default=0.2, type=float,
                        help="seconds per attempt; pinned CommUDP tries each read at most ten times")
    parser.add_argument("--probe-attempts", default=3, type=int)
    parser.add_argument("--iterations", default=100, type=int)
    parser.add_argument("--read-only", action="store_true")
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args(argv)
    if not 0 < args.timeout <= 60 or args.iterations < 1 or not 1 <= args.probe_attempts <= 100:
        parser.error("timeout must be in (0, 60], iterations positive, and probe-attempts in [1, 100]")
    if not 1 <= args.port <= 65535:
        parser.error("port must be in [1, 65535]")
    report = {"ok": False, "started_utc": datetime.now(timezone.utc).isoformat(),
              "target": str(args.target), "port": args.port,
              "csr_csv": str(args.csr_csv.resolve()), "errors": []}
    comm = None
    try:
        mapping = load_map(args.csr_csv)
        from litex.tools.remote.comm_udp import CommUDP
        comm = CommUDP(server=str(args.target), port=args.port,
                       csr_csv=str(args.csr_csv), timeout=args.timeout)
        comm.open(probe=False)
        report["probe"] = bounded_probe(comm, args.probe_attempts, args.timeout)
        if not report["probe"]["ok"]:
            raise CheckFailure(f"no Etherbone probe response after {args.probe_attempts} attempts")
        report.update(check_connection(comm, mapping, iterations=args.iterations, read_only=args.read_only))
        if report["probe"]["timeouts"]:
            report["errors"].append(f"probe required {report['probe']['timeouts']} retries")
            report["ok"] = False
    except (Exception, KeyboardInterrupt) as error:
        report["errors"].append(f"{type(error).__name__}: {error}")
    finally:
        if comm is not None:
            comm.close()
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.json_output:
        args.json_output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
