"""Management-only paired-camera session for the live viewer.

Image UDP goes directly to the viewer; this process never binds its image ports.
One process must exclusively own camera management. Send ``stop`` on stdin (or
close stdin / send SIGINT or SIGTERM) to stop both cameras and release ownership.
Stdout is a stream of JSON objects with state, message and details fields.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack, contextmanager
import ipaddress
import json
from pathlib import Path
import signal
import socket
import sys
import threading
import time

from .capture import CaptureError, DiagnosticCapture
from .capture_dual import DualDiagnosticCapture
from .calibration_config import configure_camera, load_calibration
from .flash import create_transport
from .exposure import apply_linked_exposure, exposure_state, validate_settings
from .ov9281 import PAIR_BASE, PAIR_MAGIC
from .paired import DRAINED, FAULT, LOCKED
from .supervisor import CameraSupervisor


PACKET_REGISTER_NAMES = (
    "enable", "destination_ip", "destination_port", "active_destination_ip",
    "active_destination_port", "fifo_level", "high_water", "faults", "clear",
    "packets_committed", "packets_sent", "bytes_sent", "overflows",
    "protocol_errors", "rejections",
)


class StopRequested(Exception):
    """Cooperative cancellation, including partially completed configuration."""


class OrderedManagementBus:
    """Fence each management write before sending the next Ethernet packet.

    A plain Etherbone write has no reply. Consecutive clear/configuration
    packets can overrun the checked-frame receiver, even when a later level
    readback happens to match its reset value. The ABI3 mixed record executes
    the write and then reads the always-accessible paired-control identity.
    Writes are sent once: uncertain reset/clear/commit commands are not replayed.
    """

    def __init__(self, bus, *, require_ordered=False):
        self.inner = bus
        capability = getattr(bus, "supports_ordered_write_read", None)
        self.ordered = (callable(getattr(bus, "write_read", None))
                        and callable(capability) and capability())
        if require_ordered and not self.ordered:
            raise CaptureError("live Etherbone management requires the ABI3 ordered-record bridge")

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def write(self, address, values):
        if self.ordered:
            observed = int(self.inner.write_read(address, values, PAIR_BASE))
            if observed != PAIR_MAGIC:
                raise CaptureError("management write completion returned invalid paired-control identity")
        else:
            self.inner.write(address, values)  # USB is an ordered byte stream.


class CounterWriter:
    """Warmup-only packet accounting; no image files or viewer-port socket."""

    def __init__(self):
        self.packets = 0

    def add(self, packet):
        self.packets += 1


class CooperativeSensorIO:
    """Check cancellation between SCCB transactions, never between pin edges."""

    def __init__(self, inner, check):
        self.inner, self.check = inner, check
        self.suspended = 0

    @contextmanager
    def uncancellable(self):
        """Finish group END during exposure cleanup even after STOP arrives."""
        self.suspended += 1
        try:
            yield
        finally:
            self.suspended -= 1

    def _check(self):
        if not self.suspended:
            self.check()

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def read_register(self, *args, **kwargs):
        self._check()
        result = self.inner.read_register(*args, **kwargs)
        self._check()
        return result

    def write_register(self, *args, **kwargs):
        self._check()
        result = self.inner.write_register(*args, **kwargs)
        self._check()
        return result


def json_event(state, message, details=None):
    print(json.dumps({"state": state, "message": message, "details": details or {}},
                     separators=(",", ":")), flush=True)


def route_destination(host):
    """Ask the OS which local IPv4 interface routes to the FPGA; send nothing."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.connect((host, 1234))
        return probe.getsockname()[0]


class ExposureCommands:
    """One pending setting and one input error; stdin never accesses hardware."""

    MAX_LINE = 1024

    def __init__(self):
        self.lock = threading.Lock()
        self.pending = None
        self.error = None

    def reject(self, message, command_id=None):
        with self.lock:
            self.error = {"id": command_id, "message": message}

    def submit(self, line):
        command_id = None
        try:
            if len(line) > self.MAX_LINE:
                raise ValueError("exposure command exceeds1024 characters")
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError("exposure command must be a JSON object")
            possible_id = value.get("id")
            if type(possible_id) is int and 0 <= possible_id <= 0xffffffff:
                command_id = possible_id
            if value.get("command") == "auto_exposure":
                if (set(value) != {"command", "id", "enabled", "ev_quarters"}
                        or command_id is None or type(value["enabled"]) is not bool
                        or type(value["ev_quarters"]) is not int or not -8 <= value["ev_quarters"] <= 8):
                    raise ValueError("expected auto_exposure with uint32 id, boolean enabled and ev_quarters -8..8")
            elif (set(value) != {"command", "id", "exposure_lines", "gain_code"}
                    or value["command"] != "exposure" or command_id is None):
                raise ValueError("expected exposure command with uint32 id, exposure_lines and gain_code")
            elif value["command"] == "exposure":
                validate_settings(value["exposure_lines"], value["gain_code"])
        except (ValueError, TypeError, RecursionError) as error:
            self.reject(str(error), command_id)
            return
        with self.lock:
            self.pending = value

    def take(self):
        with self.lock:
            result = self.pending, self.error
            self.pending = self.error = None
            return result

    def clear(self):
        with self.lock:
            self.pending = self.error = None


def stdin_commands(stream, stop, commands=None):
    try:
        while not stop.is_set():
            line = stream.readline(ExposureCommands.MAX_LINE + 1)
            if not line:
                break
            oversized = len(line) > ExposureCommands.MAX_LINE
            if oversized:
                while not line.endswith("\n"):
                    line = stream.readline(ExposureCommands.MAX_LINE + 1)
                    if not line:
                        break
                if commands is not None:
                    commands.reject("input line exceeds1024 characters")
                continue
            if line.strip().lower() == "stop":
                break
            if commands is not None:
                commands.submit(line)
    except (OSError, ValueError):
        pass
    finally:
        stop.set()  # STOP, EOF, or a closed viewer pipe always wins over AE.
        if commands is not None:
            commands.clear()


class LiveCapture:
    """Own startup, paired capture and bounded cleanup, without receiving images."""

    def __init__(self, bus, captures, *, stop=None, emit=json_event,
                 clock=time.monotonic, sleep=time.sleep, dual=None, commands=None,
                 calibration=None):
        self.bus, self.captures = bus, captures
        self.stop = stop if stop is not None else threading.Event()
        self._output, self.clock, self.sleep = emit, clock, sleep
        self.cleaning = False
        self.commands = commands if commands is not None else ExposureCommands()
        self.exposure = exposure_state()
        self.exposure_error = None
        self.exposure_disabled = False
        self.supervisor = None
        self.calibration = calibration if calibration is not None else load_calibration()
        self.dual = dual if dual is not None else DualDiagnosticCapture(
            bus, captures, seconds=60, fps=120, patterns=("off", "off"),
            phases_us=(0, 0), clock=clock, sleep=self._sleep)
        self.pair, self.sensors = self.dual.pair, self.dual.sensors
        for capture, sensor in zip(self.captures, self.sensors):
            sensor.i2c = CooperativeSensorIO(sensor.i2c, self._check_stop)
            capture.sensor = sensor.i2c
            sensor.sleep = self._sleep
            capture.sleep = self._sleep
        self.owned = False
        self.original_network_ready = None
        self.last_state = {}

    def emit(self, state, message, details=None):
        if state == "running":
            details = dict(details or {})
            if self.supervisor is not None:
                firmware = self.supervisor.snapshot()
                self.exposure = exposure_state(self.exposure["id"], firmware["exposure_lines"], firmware["gain_code"])
                details["firmware_exposure"] = firmware
            details["exposure"] = dict(self.exposure)
            details["calibration"] = self.calibration
            if self.exposure_error is not None:
                details["exposure_error"] = dict(self.exposure_error)
        try:
            self._output(state, message, details)
        except OSError:
            # The viewer may close stdout while exiting. Ownership still
            # requires cleanup even when it cannot receive status messages.
            self.stop.set()

    def _process_exposure(self):
        self._check_stop()
        command, input_error = self.commands.take()
        if input_error is not None:
            self.exposure_error = dict(input_error, disabled=self.exposure_disabled,
                                       uncertain=self.exposure_disabled)
            self.emit("running", "Exposure command rejected", self.last_state)
        if command is None:
            return
        self._check_stop()
        command_id = command["id"]
        if self.exposure_disabled:
            self.exposure_error = {"id": command_id, "message": "Exposure control disabled after an uncertain sensor update",
                                   "disabled": True, "uncertain": True}
            self.emit("running", "Exposure control is disabled", self.last_state)
            return
        try:
            if command["command"] == "auto_exposure":
                if self.supervisor is None:
                    raise CaptureError("firmware exposure control is unavailable")
                self.supervisor.set_auto_exposure(command["enabled"], command["ev_quarters"])
            elif self.supervisor is not None:
                self.supervisor.command(5, command["exposure_lines"], command["gain_code"])
            else:
                apply_linked_exposure(self.sensors, command["exposure_lines"], command["gain_code"], sleep=self._sleep)
        except (Exception, KeyboardInterrupt) as error:
            self.exposure_disabled = True
            self.exposure_error = {"id": command_id, "message": str(error), "disabled": True,
                                   "uncertain": True, "cleanup_errors": list(getattr(error, "cleanup_errors", ()))}
            if self.stop.is_set() or isinstance(error, KeyboardInterrupt):
                raise StopRequested() from error
            self.emit("running", "Exposure update failed; further adjustments disabled", self.last_state)
            return
        if command["command"] == "exposure":
            self.exposure = exposure_state(command_id, command["exposure_lines"], command["gain_code"])
        self.exposure_error = None
        self.emit("running", "Exposure updated on both cameras", self.last_state)

    def _check_stop(self):
        if not self.cleaning and self.stop.is_set():
            raise StopRequested()

    def _sleep(self, seconds):
        self._check_stop()
        self.sleep(seconds)
        self._check_stop()

    def _progress(self, camera, done, total):
        self._check_stop()
        if done == 1 or done == total or done % 16 == 0:
            self.emit("starting", f"Configuring camera {camera}",
                      {"camera": camera, "registers_done": done, "registers_total": total})

    def _start(self):
        self._check_stop()
        self.supervisor = CameraSupervisor.discover(self.bus, clock=self.clock, sleep=self._sleep)
        if self.supervisor is not None:
            self.supervisor.wait()
        self.pair.require_idle()
        # Prove no sensor is already streaming before claiming anything. In
        # particular, a viewer launch must not stop another owner's warmup.
        powered = []
        for sensor in self.sensors:
            self._check_stop()
            power = sensor._read_control(0x0c)
            powered.append(power == 5)
            if power == 5 and sensor.read_register(0x0100) != b"\0":
                raise CaptureError("both sensors must be in standby before live capture")
        self.original_network_ready = int(self.bus.read(0))
        if self.original_network_ready not in (0, 1):
            raise CaptureError("invalid network-ready register")
        self._check_stop()
        self.owned = True
        self.pair.generator(False)
        if self.supervisor is not None:
            self.supervisor.command(1)
        for camera, sensor in enumerate(self.sensors):
            self._check_stop()
            if self.supervisor is not None:
                continue
            if not powered[camera]:
                sensor.power_on()
            sensor.configure(pattern="off", fps=120,
                             progress=lambda done, total, c=camera: self._progress(c, done, total))
        for capture in self.captures:
            self._check_stop()
            capture.preflight()
        self.pair.configure_generator(120, (0, 0))
        self.pair.reset()
        self.dual._initialize_receivers()
        if self.supervisor is not None:
            self.supervisor.command(2)
        for capture in self.captures:
            self._check_stop()
            if self.supervisor is not None:
                capture.configure(firmware_isp=True)
            else:
                capture.configure()
        # Replace the diagnostic placeholder before admitting any frame. Both
        # product types retain the same camera/port ownership as their image.
        calibrated = [configure_camera(capture, camera, check_stop=self._check_stop)
                      for capture, camera in zip(self.captures, self.calibration["cameras"])]
        self.calibration = dict(self.calibration, cameras=calibrated, readback_verified=True)
        self._check_stop()
        self.pair.generator(True)
        if self.supervisor is not None:
            self.supervisor.command(3)
        for sensor in self.sensors:
            self._check_stop()
            if self.supervisor is None:
                sensor._write_register(0x0100, 1)
        self.dual._warmup()
        self._check_stop()
        # Our ephemeral sockets cannot observe packets addressed to the viewer.
        # Check hardware output counters before CLEAR discards warmup evidence.
        for capture in self.captures:
            state = capture.snapshot(include_isp=False)
            if state["image"][9] or state["image"][10]:
                raise CaptureError("image packets committed while paired capture was disarmed")
        self.pair.clear()  # Preserve the warmed-up D-PHY phase.
        self.pair.snapshot(after_epoch=self.dual.report["warmup"]["epoch"])
        for capture in self.captures:
            state = capture.snapshot(include_isp=False)
            if state["camera"][6] or state["camera"][5] & 4:
                raise CaptureError("receiver diagnostics did not clear before paired arm")
        self._check_stop()
        self.pair.run(True)

    def _snapshot(self):
        result = {"pair": self.dual._pair_live(), "cameras": []}
        for capture in self.captures:
            state = dict(capture.snapshot(include_isp=False))
            products = {}
            for name, channel, count in (("centroids", 1, 14), ("pose", 2, 15)):
                # These shared transport CSRs remain accessible during a
                # camera fault; no reset-sensitive ISP responder is touched.
                words = capture.read_words(capture.image + channel * 0x80, count)
                if len(words) != count:
                    raise CaptureError(f"camera{capture.camera} {name} diagnostics were truncated")
                products[name] = {key: int(value) for key, value in
                                  zip(PACKET_REGISTER_NAMES, words) if key != "clear"}
            state["products"] = products
            result["cameras"].append(state)
        result["warnings"] = [f"camera{camera}: LFTag candidate protocol error retained"
                              for camera, state in enumerate(result["cameras"])
                              if state["camera"][5] & 64]
        return result

    @staticmethod
    def _faulted(state):
        pair = state["pair"]
        return bool(pair["fault"] or pair["status"] & FAULT
                    or not pair["status"] & LOCKED or not pair["run"]
                    or any(camera["camera"][6] or camera["camera"][5] & 4
                           for camera in state["cameras"]))

    def _cleanup(self):
        if not self.owned:
            return {"owned": False, "errors": []}
        self.cleaning = True
        result = {"owned": True, "errors": [], "sensor_standby_verified": [False, False]}

        def attempt(name, action):
            try:
                return action()
            except (Exception, KeyboardInterrupt) as error:
                result["errors"].append(f"{name}: {type(error).__name__}: {error}")
                return None

        self.emit("stopping", "Stopping the paired capture")
        def request_stop():
            try:
                status, run = self.pair.read(0x0c), self.pair.read(0x08)
                already_stopped = not run and not status & 15 and bool(status & DRAINED)
            except Exception as error:
                result["stop_state_error"] = f"{type(error).__name__}: {error}"
                already_stopped = False
            # STOP transiently asserts busy. An unnecessary STOP during a
            # failed startup (network-ready still0) manufactures a health fault.
            if not already_stopped:
                self.pair.run(False)
            result["stop_already_disarmed_drained"] = already_stopped

        attempt("paired stop", request_stop)

        def drain():
            deadline = self.clock() + 1
            while True:
                status = self.pair.read(0x0c)
                if not self.pair.read(0x08) and not status & 15 and status & DRAINED:
                    return True
                if self.clock() >= deadline:
                    raise TimeoutError("paired transport did not drain")
                self.sleep(0.005)

        drained = bool(attempt("drain", drain))
        result["physical_drained"] = drained
        # FSYNC must remain enabled until BOTH standby requests are verified.
        firmware_stopped = None
        if self.supervisor is not None:
            firmware_stopped = bool(attempt("firmware standby", lambda: (self.supervisor.command(4), True)[1]))
        for camera in (1, 0):
            def standby(index=camera):
                if self.supervisor is not None:
                    return firmware_stopped
                sensor = self.sensors[index]
                if sensor._read_control(0x0c) == 5:
                    sensor._write_register(0x0100, 0)
                return True  # A powered-down sensor is also stopped.
            result["sensor_standby_verified"][camera] = bool(attempt(f"camera{camera} standby", standby))
        self.sleep(0.05)
        stopped = all(result["sensor_standby_verified"])
        disabled = stopped and bool(attempt("FSYNC off", lambda: self.pair.generator(False)))
        result["generator_disabled"] = disabled
        result["before_reset"] = attempt("stopped diagnostics", self._snapshot)
        state = result["before_reset"]
        result["hardware_fault"] = bool(state and (
            state["pair"]["fault"] or state["pair"]["status"] & FAULT
            or any(c["camera"][5] & 4 or c["camera"][6] & ~0x400 for c in state["cameras"])))
        sticky = bool(state and (state["pair"]["fault"] or state["pair"]["status"] & FAULT
                                or any(c["camera"][6] or c["camera"][5] & 4 for c in state["cameras"])))
        safe = drained and stopped and disabled and state is not None
        if safe and sticky:
            # Publish evidence before the operation that clears retained faults.
            self.emit("stopping", "Retained diagnostics before paired reset", state)
            def reset():
                self.pair.disarmed_drained()
                self.pair.reset()
                self.dual._initialize_receivers()
                self.pair.require_idle()
                return True
            safe = bool(attempt("paired recovery reset", reset))
            result["reset_recovered"] = safe
        if safe:
            for camera, capture in enumerate(self.captures):
                for channel in range(3):
                    attempt(f"camera{camera} product{channel} off",
                            lambda c=capture, ch=channel: c.write(c.image + ch * 0x80, 0))
                attempt(f"camera{camera} ISP off", lambda c=capture: c.write(c.isp + 4, 0))
        else:
            result["output_cleanup_skipped"] = "drain, sensor standby or safe ISP access not established"
        if drained and self.original_network_ready is not None:
            attempt("network-ready restore", lambda: self.captures[0].write(0, self.original_network_ready))
        return result

    def run(self):
        terminal, message = "stopped", "Both cameras stopped"
        self.emit("starting", "Preparing both cameras for live capture")
        try:
            self._start()
            self.emit("running", "Both cameras are streaming", {
                "fps_preset": 120, "phases_us": [0, 0],
                "destination": self.captures[0].report["destination"],
                "ports": [c.port for c in self.captures]})
            next_update = self.clock() + 1
            while not self.stop.is_set():
                self.last_state = self._snapshot()
                if self._faulted(self.last_state):
                    terminal, message = "fault", "Camera or paired-capture fault"
                    self.emit(terminal, message, self.last_state)
                    break
                self._process_exposure()
                if self.clock() >= next_update:
                    update = ("Streaming; LFTag candidates rejected" if self.last_state["warnings"]
                              else "Both cameras are streaming")
                    self.emit("running", update, self.last_state)
                    next_update = self.clock() + 1
                self._sleep(0.1)
        except StopRequested:
            pass
        except (Exception, KeyboardInterrupt) as error:
            terminal, message = "error", f"{type(error).__name__}: {error}"
            if self.owned:
                try:
                    self.last_state = self._snapshot()
                except (Exception, KeyboardInterrupt) as diagnostic_error:
                    self.last_state["diagnostic_error"] = f"{type(diagnostic_error).__name__}: {diagnostic_error}"
            self.emit(terminal, message, self.last_state)
        finally:
            cleanup = self._cleanup()
        if cleanup.get("hardware_fault") and terminal == "stopped":
            terminal, message = "fault", "Hardware fault retained while stopping the cameras"
        if cleanup["errors"] and terminal == "stopped":
            terminal, message = "error", "Camera cleanup was not fully verified"
        self.emit(terminal, message, {"cleanup": cleanup, "last_state": self.last_state})
        return 0 if terminal == "stopped" else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", type=ipaddress.IPv4Address, default=ipaddress.IPv4Address("192.168.1.50"))
    parser.add_argument("--destination", type=ipaddress.IPv4Address)
    parser.add_argument("--port0", type=int, default=6000)
    parser.add_argument("--port1", type=int, default=6003)
    parser.add_argument("--transport", choices=("etherbone", "usb"), default="etherbone")
    parser.add_argument("--device", help="explicit USB CDC path when --transport usb is selected")
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--calibration", type=Path,
                        help="schema1 JSON pinhole calibration; default: nominal2.8mm lenses, normalized tag width")
    args = parser.parse_args(argv)
    if any(not 1 <= p <= 65535 for p in (args.port0, args.port1)) or args.port0 == args.port1:
        json_event("error", "Camera ports must be distinct and in 1..65535")
        return 1
    try:
        calibration = load_calibration(args.calibration)
    except (OSError, ValueError) as error:
        json_event("error", f"Calibration: {error}")
        return 1
    stop = threading.Event()
    commands = ExposureCommands()
    handlers = {signum: signal.signal(signum, lambda *_: stop.set())
                for signum in (signal.SIGINT, signal.SIGTERM)}
    threading.Thread(target=stdin_commands, args=(sys.stdin, stop, commands), daemon=True).start()
    try:
        destination = str(args.destination) if args.destination else route_destination(str(args.host))
        args.target, args.port, args.baudrate = args.host, 1234, 115200
        with ExitStack() as stack:
            raw_bus = stack.enter_context(create_transport(args))
            bus = OrderedManagementBus(raw_bus, require_ordered=args.transport == "etherbone")
            captures = []
            for camera, port in enumerate((args.port0, args.port1)):
                # These sockets only satisfy the existing disarmed warmup check;
                # image destinations remain the viewer ports, never these ports.
                receiver = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
                receiver.bind(("0.0.0.0", 0))
                receiver.setblocking(False)
                captures.append(DiagnosticCapture(bus, receiver, CounterWriter(), camera=camera,
                    address=(0x60, 0x10)[camera], destination=destination, port=port, board_ip=str(args.host),
                    seconds=60, transport=args.transport))
            return LiveCapture(bus, captures, stop=stop, commands=commands,
                               calibration=calibration).run()
    except (Exception, KeyboardInterrupt) as error:
        json_event("error", f"{type(error).__name__}: {error}")
        return 1
    finally:
        for signum, handler in handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
