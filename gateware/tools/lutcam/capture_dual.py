"""Capture both configured, stopped OV9281 sensors through one management bus.

Each camera writes a separate report.json and compact .lwc files, with the same
strict validation as tools.lutcam.capture. Image UDP uses --port0/--port1;
--port/--control-port selects the Etherbone management port. Both cameras must
be configured for 1280x800 and in standby. This installs TEST-ONLY intrinsics,
uses ISP bypass, disables child/pose output, and leaves both sensors stopped.
The FPGA drives shared-period FSYNC with independent phases into both slaves. Startup frames are
discarded before raw SOF lock and paired admission. Only one management client
may operate the board. SOF alignment does not establish exposure alignment.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import ipaddress
import json
import math
from pathlib import Path
import select
import socket
import time

from .capture import CaptureError, DiagnosticCapture, FrameWriter, validate_capture
from .flash import add_connection_arguments, create_transport
from .ov9281 import OV9281, board_mode_writes
from .paired import PairedCaptureControl, DRAINED, FAULT, LOCKED, quantize_phases
from .receive import PacketError


class DualDiagnosticCapture:
    """One owner for paired admission, phased FSYNC and both image receivers."""

    def __init__(self, bus, captures, *, seconds, fps=120, patterns=("off", "off"),
                 solids=(0x200, 0x200), phases_us=(0, 0), clock=time.monotonic, sleep=time.sleep, wait=select.select):
        if not math.isfinite(seconds) or not 0 < seconds <= 60:
            raise ValueError("seconds must be finite and in (0, 60]")
        if fps not in (60, 120) or len(patterns) != 2 or len(solids) != 2:
            raise ValueError("explicit mode needs FPS60/120 and two pattern/solid selections")
        self.captures = sorted(captures, key=lambda capture: capture.camera)
        if ([capture.camera for capture in self.captures] != [0, 1]
                or any(capture.bus is not bus for capture in self.captures)):
            raise ValueError("paired capture requires cameras0/1 sharing exactly one bus")
        self.bus, self.seconds, self.fps = bus, seconds, fps
        self.patterns, self.solids = patterns, solids
        self.phases_us = phases_us
        quantize_phases(fps, phases_us)
        for camera in (0, 1):
            board_mode_writes(camera, patterns[camera], solids[camera], fps=fps)
        self.clock, self.sleep, self.wait = clock, sleep, wait
        self.pair = PairedCaptureControl(bus, clock=clock, sleep=sleep)
        self.sensors = [OV9281(bus, camera=c.camera, address=c.address, i2c=c.sensor,
                              clock=clock, sleep=sleep) for c in self.captures]
        self.report = {"ok": False, "seconds_requested": seconds, "fps_preset": fps,
                       "test_calibration_only": True, "cleanup_errors": [], "pair_snapshots": [],
                       "synchronization": "FPGA FSYNC outputs share a period with independent phases; both sensors are slaves. Coherent CSI SOFs identify a shared epoch; exposure alignment is not measured.",
                       "camera_reports": {str(camera): f"camera{camera}/report.json" for camera in (0, 1)}}

    def _cleanup(self, name, action, capture=None):
        try:
            return action()
        except (Exception, KeyboardInterrupt) as error:
            message = f"{name}: {type(error).__name__}: {error}"
            if capture is not None:
                message = f"camera{capture.camera} {message}"
                capture.report["cleanup_errors"].append(message)
            self.report["cleanup_errors"].append(message)

    def _receive(self, *, preserve_failure=False):
        empty = True
        for capture in self.captures:
            try:
                empty = capture.receive_pending() and empty
            except (PacketError, OSError) as error:
                if not preserve_failure:
                    raise
                empty = False
                evidence = self.report.setdefault("failure_drain_packet_errors", {}).setdefault(
                    str(capture.camera), {"count": 0, "examples": []})
                evidence["count"] += 1
                if len(evidence["examples"]) < 8:
                    evidence["examples"].append(f"{type(error).__name__}: {error}")
        return empty

    def _warmup_failure_evidence(self):
        """Retain raw monitor diagnostics before cleanup stops the pulse source."""
        evidence = self.report["warmup_failure"] = {"errors": [], "cameras": {}}
        try:
            evidence["pair_snapshot"] = self.pair.snapshot(timeout=0.025)
        except (Exception, KeyboardInterrupt) as error:
            evidence["errors"].append(f"snapshot: {type(error).__name__}: {error}")
        try:
            live = self.bus.read(0x30024208, length=4)
            evidence["pair_live"] = {"run": int(live[0]), "status": int(live[1]), "fault": int(live[3])}
            # SNAPSHOT strobes also latch counters when no valid timestamp
            # epoch exists. These diagnostics do not claim SOF coherence.
            counters = self.bus.read(0x3002424c, length=5)
            evidence["latched_counts"] = dict(zip(
                ("raw_sof0", "raw_sof1", "sync", "matched", "missing"), map(int, counters)))
        except (Exception, KeyboardInterrupt) as error:
            evidence["errors"].append(f"registers: {type(error).__name__}: {error}")
        for capture in self.captures:
            try:
                evidence["cameras"][str(capture.camera)] = capture.snapshot()
            except (Exception, KeyboardInterrupt) as error:
                evidence["errors"].append(f"camera{capture.camera}: {type(error).__name__}: {error}")

    def _initialize_receivers(self):
        start = self.clock()
        deadline = start + 1
        while True:
            clean = True
            for capture in self.captures:
                status = int(self.bus.read(capture.base + 0x14))
                receiver = int(self.bus.read(capture.base + 0x18))
                capture.report["receiver_initialization"] = {
                    "status": status, "receiver_status": receiver, "elapsed": self.clock() - start}
                clean &= bool(status & 8) and receiver == 0
            if clean:
                return
            if self.clock() >= deadline:
                raise TimeoutError("both CSI receivers must initialize cleanly in sensor standby")
            self.sleep(0.001)

    def _warmup(self):
        start = self.clock()
        deadline = start + 2
        while True:
            self._receive()
            if any(c.writer.packets for c in self.captures):
                raise CaptureError("image packets appeared while paired admission was disarmed")
            status = self.pair.read(0x0c)
            if self.clock() - start >= 0.1 and status & LOCKED:
                state = self.pair.snapshot()
                if state["matched"] >= 2 and state["raw_sof0"] >= 2 and state["raw_sof1"] >= 2:
                    state["warmup_seconds"] = self.clock() - start
                    self.report["warmup"] = state
                    return
            if self.clock() >= deadline:
                raise TimeoutError("both slave sensors did not establish raw SOF lock during warmup")
            self.sleep(0.005)

    def _drain(self, *, evidence_key="capture_complete"):
        deadline = self.clock() + 1
        preserve_failure = evidence_key == "failure_drain"
        quiet_since = None
        while True:
            empty = self._receive(preserve_failure=preserve_failure)
            states = [capture.snapshot(include_isp=not preserve_failure) for capture in self.captures]
            status = self.pair.read(0x0c)
            run = self.pair.read(0x08)
            clean = empty and not run and (status & 15) == 0 and bool(status & DRAINED)
            for capture, state in zip(self.captures, states):
                capture.report[evidence_key] = state
                if preserve_failure:
                    clean &= state["image"][9] == state["image"][10]
                    # DRAINED proves that all committed transport packets have
                    # left. A fault may leave one unfinished writer fragment;
                    # its producer is reset, so only paired RESET can discard it.
                    if not status & FAULT:
                        clean &= not state["image"][5]
                    else:
                        clean &= state["image"][5] in (0, 1)
                else:
                    clean &= not state["image"][5]
                    clean &= (bool(state["camera"][5] & 2)
                              and state["image"][10] == capture.writer.packets)
            if preserve_failure:
                self.report[evidence_key] = {
                    "run": run, "status": status, "physical_drained": bool(status & DRAINED),
                    "partial_writer_cameras": [capture.camera for capture, state in zip(self.captures, states)
                        if status & FAULT and state["image"][5] and state["image"][9] == state["image"][10]],
                    "basis": "Hardware DRAINED covers committed packets; aborted partial writers are retained until paired reset.",
                }
            if clean:
                if not preserve_failure:
                    return True
                # Let datagrams already handed to the MAC reach both sockets.
                if quiet_since is None:
                    quiet_since = self.clock()
                if self.clock() - quiet_since >= 0.01:
                    return True
            else:
                quiet_since = None
            if self.clock() >= deadline:
                raise TimeoutError("paired camera/UDP drain timeout")
            self.sleep(0.002)

    def _stop_sensor(self, camera):
        self.sensors[camera]._write_register(0x0100, 0)
        return True

    def _pair_live(self):
        return {name: self.pair.read(offset) for name, offset in (
            ("run", 0x08), ("status", 0x0c), ("fault", 0x14),
            ("pairs_started", 0x18), ("pairs_completed", 0x1c), ("sync_enable", 0x6c))}

    def _recover_fault(self, stopped, physical_drained):
        """Recover a failed capture only after preserving its stopped evidence."""
        evidence = self.report["fault_recovery"] = {"attempted": False, "ok": False}
        try:
            evidence["before"] = self._pair_live()
            if (not all(stopped) or not self.report.get("generator_disabled_after_standby")
                    or not physical_drained or evidence["before"]["sync_enable"]):
                raise CaptureError("fault reset requires verified sensor standby, FSYNC off and physical transport drain")
            self.pair.disarmed_drained()
            if any(capture.report.get("post_stop") is None for capture in self.captures):
                raise CaptureError("fault reset requires both post-stop camera snapshots")
            try:
                evidence["pair_snapshot"] = self.pair.snapshot(timeout=0.025)
            except (Exception, KeyboardInterrupt) as error:
                evidence["snapshot_error"] = f"{type(error).__name__}: {error}"
            evidence["evidence_files"] = []
            for capture in self.captures:
                capture.writer.close()
                path = capture.writer.output / "before-fault-reset.json"
                with path.open("x") as output:
                    json.dump({"failed_capture": True, "pair": self.report,
                               "camera": capture.report, "received": capture.writer.summary()}, output, indent=2)
                    output.write("\n")
                evidence["evidence_files"].append(str(path))
            # Recheck immediately before the evidence-clearing operation.
            self.pair.disarmed_drained()
            if self.pair.read(0x6c):
                raise CaptureError("FSYNC changed before fault reset")
            evidence["attempted"] = True
            self.pair.reset()
            deadline = self.clock() + 1
            while True:
                pair = self._pair_live()
                states = [capture.snapshot(include_isp=False) for capture in self.captures]
                levels = [[int(self.bus.read(capture.image + channel * 0x80 + 0x14))
                           for channel in range(3)] for capture in self.captures]
                evidence["after"] = {"pair": pair, "cameras": states, "product_fifo_levels": levels}
                clean = (not pair["run"] and not pair["fault"] and not pair["sync_enable"]
                         and (pair["status"] & (15 | FAULT)) == 0 and bool(pair["status"] & DRAINED))
                clean &= all((state["camera"][5] & 15) == 10 and not state["camera"][6]
                             for state in states)
                clean &= all(not level for camera_levels in levels for level in camera_levels)
                if clean:
                    states = [capture.snapshot() for capture in self.captures]
                    evidence["after"]["cameras"] = states
                    if any(state["isp"] is None or state["isp"][0] != 0x4d495350
                           or state["isp"][1] for state in states):
                        raise CaptureError("ISP did not recover stopped register access after paired reset")
                    evidence["ok"] = True
                    return True
                if self.clock() >= deadline:
                    raise TimeoutError("paired fault reset did not restore clean initialized idle receivers")
                self.sleep(0.002)
        except (Exception, KeyboardInterrupt) as error:
            evidence["error"] = f"{type(error).__name__}: {error}"
            raise

    def _validate_pair(self):
        errors = []
        final = self.report.get("pair_capture_complete")
        if final is None:
            return ["paired completion snapshot is missing"]
        ids = [list(capture.writer.frames) for capture in self.captures]
        if not ids[0] or ids[0] != ids[1]:
            errors.append("both cameras must save the same contiguous paired frame IDs")
        count = len(ids[0])
        if final["pairs_started"] != count or final["pairs_completed"] != count:
            errors.append("hardware pair counts do not match saved frame pairs")
        if min(final["matched"], final["raw_sof0"], final["raw_sof1"]) < count:
            errors.append("coherent raw SOF evidence does not cover all completed pairs")
        initial = self.report.get("pair_before_arm")
        if initial is None or not count <= ((final["epoch"] - initial["epoch"]) & 0xffffffff) < 0x80000000:
            errors.append("coherent SOF epoch did not advance across all completed pairs")
        for state in self.report["pair_snapshots"] + [final]:
            if state["fault"] or state["status"] & FAULT or not state["status"] & LOCKED:
                errors.append("paired capture lost lock or reported a fault")
                break
            if state["missing"]:
                errors.append("raw SOF synchronization monitor recorded missing/duplicate events")
                break
            if max(abs(state["min_residual_skew_cycles"]), abs(state["max_residual_skew_cycles"])) > state["max_skew_limit_cycles"]:
                errors.append("phase-corrected SOF skew exceeded the configured hardware bound")
                break
            if not state["min_latency_limit_cycles"] <= state["min_latency_cycles"] <= state["max_latency_cycles"] <= state["max_latency_limit_cycles"]:
                errors.append("SOF latency fell outside the configured sync-anchor window")
                break
        return errors

    def run(self):
        owned = False
        drained = False
        original_network_ready = None
        current = None
        phase = "preflight"
        try:
            # Verify paired-control capabilities before clocking SCCB.
            self.pair.require_idle()
            for current, sensor in zip(self.captures, self.sensors):
                current.preflight()
                sensor.verify_mode(pattern=self.patterns[current.camera], solid=self.solids[current.camera], fps=self.fps)
            original_network_ready = self.captures[0].report["initial"]["board"][0]
            if self.captures[1].report["initial"]["board"][0] != original_network_ready:
                raise CaptureError("board network-ready changed during preflight")
            owned = True
            phase = "configure"
            current = None
            # Both standby readbacks passed: changing the pulse generator is safe.
            self.pair.generator(False)
            self.report["generator"] = self.pair.configure_generator(self.fps, self.phases_us)
            self.pair.reset()
            self._initialize_receivers()
            for current in self.captures:
                current.configure()
            phase = "start"
            current = None
            self.pair.generator(True)
            for sensor in self.sensors:
                sensor._write_register(0x0100, 1)
            phase = "warmup"
            self._warmup()
            # This clears diagnostics/counters without resetting the active D-PHY.
            self.pair.clear()
            self.report["pair_before_arm"] = self.pair.snapshot(after_epoch=self.report["warmup"]["epoch"])
            for current in self.captures:
                state = current.snapshot()
                current.report["armed"] = state
                if state["camera"][6] or state["camera"][5] & 4:
                    raise CaptureError("CSI diagnostic clear did not leave both receivers clean")
                if state.get("lftag") and state["lftag"]["raw_words"][:3] != [0, 0, 0]:
                    raise CaptureError("LFTag diagnostic clear did not leave both retained contexts clean")
            phase = "arm"
            current = None
            self.pair.run(True)
            start = self.clock()
            self.report["capture_window"] = {"basis": "Host interval after paired RUN readback until paired stop request; hardware pair/CSI timestamps establish synchronization.", "duration": None}
            next_poll = start
            phase = "capture"
            sockets = [capture.receiver for capture in self.captures]
            while self.clock() - start < self.seconds:
                readable = self.wait(sockets, [], [], 0.002)[0]
                for current in self.captures:
                    if current.receiver in readable:
                        current.receive_pending()
                now = self.clock()
                if now >= next_poll:
                    current = None
                    pair_state = self.pair.snapshot()
                    pair_state["elapsed"] = now - start
                    self.report["pair_snapshots"].append(pair_state)
                    if pair_state["fault"] or pair_state["status"] & FAULT or not pair_state["status"] & LOCKED:
                        raise CaptureError("paired capture fault or synchronization loss")
                    for current in self.captures:
                        state = current.snapshot()
                        state["elapsed"] = now - start
                        current.report["snapshots"].append(state)
                        if state["camera"][6] or state["camera"][5] & 4:
                            raise CaptureError("receiver/capture fault during streaming")
                    next_poll = now + 0.02
            self.report["capture_window"]["duration"] = self.clock() - start
            phase = "drain"
            current = None
            self.pair.run(False)
            self._drain()
            last_epoch = (self.report["pair_snapshots"][-1] if self.report["pair_snapshots"]
                          else self.report["pair_before_arm"])["epoch"]
            self.report["pair_capture_complete"] = self.pair.snapshot(after_epoch=last_epoch)
            for current in self.captures:
                current.report["capture_elapsed"] = self.clock() - start
            drained = True
        except (Exception, KeyboardInterrupt) as error:
            message = f"{type(error).__name__}: {error}"
            self.report.update(error=message, error_phase=phase,
                               error_camera=current.camera if current is not None else None)
            if current is not None:
                current.report["error"] = message
            if phase == "warmup":
                self._warmup_failure_evidence()
        finally:
            if owned:
                physical_drained = drained
                if not drained:
                    self._cleanup("paired disarm", lambda: self.pair.run(False))
                    physical_drained = bool(self._cleanup(
                        "failure drain", lambda: self._drain(evidence_key="failure_drain")))
                # Retain FSYNC until both sensor standby requests are verified.
                stopped = [False, False]
                for camera in (1, 0):
                    stopped[camera] = bool(self._cleanup("sensor standby", lambda index=camera: self._stop_sensor(index), self.captures[camera]))
                self.report["sensor_standby_verified"] = stopped
                # 0x0106=0 requests graceful standby; register readback may
                # precede the final unadmitted frame's physical completion.
                self._cleanup("standby settling", lambda: self.sleep(0.05))
                if all(stopped):
                    disabled = self._cleanup("disable FSYNC generator", lambda: self.pair.generator(False))
                    self.report["generator_disabled_after_standby"] = bool(disabled)
                else:
                    self.report["generator_disabled_after_standby"] = False
                for capture in self.captures:
                    state = self._cleanup("post-stop snapshot", capture.snapshot, capture)
                    if state is not None:
                        capture.report["post_stop"] = state
                pair_state = self._cleanup("post-stop paired state", self._pair_live)
                self.report["pair_post_stop"] = pair_state
                faulted = bool(pair_state and (pair_state["fault"] or pair_state["status"] & FAULT))
                faulted |= any(capture.report.get("post_stop", {}).get("camera", [0] * 6)[5] & 4
                               for capture in self.captures)
                recovered = False
                if faulted:
                    self.report.setdefault("error", "CaptureError: hardware capture fault retained after sensor standby")
                    recovered = bool(self._cleanup("paired fault recovery",
                        lambda: self._recover_fault(stopped, physical_drained)))
                outputs_safe = recovered or (physical_drained and not faulted
                    and all(capture.report.get("post_stop") is not None for capture in self.captures))
                for capture in self.captures:
                    if outputs_safe:
                        self._cleanup("disable image", lambda c=capture: c.write(c.image, 0), capture)
                        self._cleanup("stop ISP", lambda c=capture: c.write(c.isp + 4, 0), capture)
                    else:
                        capture.report["output_cleanup_skipped"] = "pipeline was not proven drained and accessible"
                self._cleanup("restore network-ready", lambda: self.captures[0].write(0, original_network_ready))
                for capture in self.captures:
                    capture.report["after_cleanup"] = self._cleanup("cleanup snapshot", capture.snapshot, capture)
            summaries = {}
            for capture in self.captures:
                self._cleanup("close frame file", capture.writer.close, capture)
                capture.report.update(capture.writer.summary())
                capture.report["validation_errors"] = validate_capture(capture.report)
                capture.report["ok"] = not (capture.report.get("error") or capture.report["cleanup_errors"] or capture.report["validation_errors"])
                summaries[str(capture.camera)] = {
                    "ok": capture.report["ok"], "frames": len(capture.report["frames"]),
                    "packets": capture.report["packets"], "host_frame_rate_hz": capture.report["host_frame_rate_hz"],
                    "payload_length_histogram": capture.report["payload_length_histogram"],
                }
            self.report["cameras"] = summaries
            self.report["validation_errors"] = self._validate_pair()
            self.report["ok"] = not (self.report.get("error") or self.report["cleanup_errors"]
                or self.report["validation_errors"] or any(not report["ok"] for report in summaries.values()))
        return self.report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, conflict_handler="resolve")
    add_connection_arguments(parser)
    parser.add_argument("--control-port", "--port", dest="port", type=int, default=1234,
                        help="Etherbone management port (default 1234)")
    for camera in (0, 1):
        parser.add_argument(f"--address{camera}", type=lambda value: int(value, 0), required=True,
                            help=f"explicit unshifted seven-bit SCCB address for camera{camera}")
        parser.add_argument(f"--pattern{camera}", choices=("off", "bars", "solid"), default="off")
        parser.add_argument(f"--solid{camera}", type=lambda value: int(value, 0), default=0x200)
        parser.add_argument(f"--phase{camera}-us", type=float, default=0,
                            help=f"camera {camera} FSYNC phase in microseconds; rounded to one 120 MHz clock")
        parser.add_argument(f"--port{camera}", type=int, default=6000 + 3 * camera,
                            help=f"image UDP port for camera{camera}")
    parser.add_argument("--destination", type=ipaddress.IPv4Address, required=True)
    parser.add_argument("--fps", type=int, choices=(60, 120), default=120)
    parser.add_argument("--seconds", type=float, default=1)
    parser.add_argument("--output", type=Path, required=True, help="new exclusive evidence directory")
    args = parser.parse_args(argv)
    try:
        quantize_phases(args.fps, (args.phase0_us, args.phase1_us))
        for camera in (0, 1):
            board_mode_writes(camera, getattr(args, f"pattern{camera}"), getattr(args, f"solid{camera}"), fps=args.fps)
    except (ValueError, TypeError) as error:
        parser.error(str(error))
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
        parser.error("seconds must be finite and in (0,60]")
    if any(not 0 <= getattr(args, f"address{camera}") <= 0x7f for camera in (0, 1)):
        parser.error("sensor addresses must be unshifted seven-bit values")
    if any(not 1 <= port <= 65535 for port in (args.port, args.port0, args.port1)) or args.port0 == args.port1:
        parser.error("UDP ports must be in 1..65535, with distinct image ports")
    args.output.mkdir(parents=True, exist_ok=False)
    directories = [args.output / f"camera{camera}" for camera in (0, 1)]
    for directory in directories:
        directory.mkdir()
    captures = []
    runner = None
    report = {"ok": False}
    try:
        with ExitStack() as stack:
            receivers = []
            for camera in (0, 1):
                receiver = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
                receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
                receiver.bind((str(args.destination), getattr(args, f"port{camera}")))
                receiver.setblocking(False)
                receivers.append(receiver)
            bus = stack.enter_context(create_transport(args))
            for camera in (0, 1):
                captures.append(DiagnosticCapture(
                    bus, receivers[camera], FrameWriter(directories[camera], camera), camera=camera,
                    address=getattr(args, f"address{camera}"), destination=args.destination,
                    port=getattr(args, f"port{camera}"), board_ip=args.target,
                    seconds=args.seconds, transport=args.transport))
            runner = DualDiagnosticCapture(bus, captures, seconds=args.seconds, fps=args.fps,
                patterns=(args.pattern0, args.pattern1), solids=(args.solid0, args.solid1),
                phases_us=(args.phase0_us, args.phase1_us))
            report = runner.run()
    except (Exception, KeyboardInterrupt) as error:
        if runner is not None:
            report = runner.report
        report["ok"] = False
        report.setdefault("error", f"{type(error).__name__}: {error}")
    finally:
        for camera, directory in enumerate(directories):
            camera_report = captures[camera].report if camera < len(captures) else {
                "ok": False, "camera": camera, "error": report.get("error", "capture did not start")}
            with (directory / "report.json").open("x") as output:
                json.dump(camera_report, output, indent=2)
                output.write("\n")
        with (args.output / "report.json").open("x") as output:
            json.dump(report, output, indent=2)
            output.write("\n")
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
