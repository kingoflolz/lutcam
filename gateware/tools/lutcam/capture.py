"""Per-camera evidence helpers used by the paired-only capture command.

Configure both sensors in standby with tools.lutcam.ov9281, then use
tools.lutcam.capture_dual. The paired command installs TEST-ONLY placeholder
intrinsics, disables child/pose output, and uses ISP bypass. It leaves both
cameras and ISPs stopped and their image channels disabled.
Only one management client may operate the board at a time. Management may
use USB, but image packets always use Ethernet. A successful report establishes
transport/counter consistency; decode each .lwc with the Rust compact decoder
to validate the self-terminating codec stream and its image dimensions.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
from pathlib import Path
import select
import time

from .i2c import create_camera_i2c
from .flash import BootClient
from .receive import IMAGE, PacketError, parse_datagram
from .ov9281 import require_pair_idle


class CaptureError(RuntimeError):
    """Capture could not establish a complete, fault-free transfer."""


LFTAG_DIAGNOSTICS_MAGIC = 0x4c464400


def decode_lftag_diagnostics(words):
    """Decode retained first-error contexts, or return None when unsupported.

    A context can be captured before the group ends; context validity does not
    imply the separate completed-group error flag. Contexts survive processing
    resets, but explicit paired CLEAR and global reset clear them.
    """
    words = list(words)
    if len(words) != 4 or any(type(word) is not int or not 0 <= word <= 0xffffffff for word in words):
        raise ValueError("LFTag diagnostics require four uint32 words")
    status, protocol, dropped, magic = words
    if magic != LFTAG_DIAGNOSTICS_MAGIC:
        return None
    level_bits = 6
    level_mask = (1 << level_bits) - 1
    pop_bit = 2 * level_bits
    result = {
        "magic": magic, "raw_words": words, "high_water": status & level_mask,
        "protocol_context_valid": bool(status & (1 << level_bits)),
        "denied_admission_context_valid": bool(status & (1 << (level_bits + 1))),
        "protocol_group_seen": bool(status & (1 << (level_bits + 2))),
        "dropped_group_seen": bool(status & (1 << (level_bits + 3))),
        "protocol": None, "denied_admission": None,
    }
    if result["protocol_context_valid"]:
        # Retired metadata bits retain their wire positions but have no
        # meaning in the current diagnostic schema. Raw words remain intact.
        reason_names = ("bad_count", "scale_3", "scale_changed", "frame_changed")
        result["protocol"] = {
            "reason_bits": protocol & 15,
            "reasons": [name for bit, name in enumerate(reason_names) if protocol & (1 << bit)],
            "stage_count_before_tuple": (protocol >> 5) & 15,
            "group_end": bool(protocol & (1 << 9)),
            "first_scale": (protocol >> 10) & 3, "current_scale": (protocol >> 12) & 3,
            "first_frame": (protocol >> 14) & 255, "current_frame": (protocol >> 22) & 255,
        }
    if result["denied_admission_context_valid"]:
        result["denied_admission"] = {
            "own_level": dropped & level_mask, "peer_level": (dropped >> level_bits) & level_mask,
            "pop_fire": bool(dropped & (1 << pop_bit)), "owner_active": bool(dropped & (1 << (pop_bit + 1))),
            "owner_camera": (dropped >> (pop_bit + 2)) & 1,
            "core_output_valid": bool(dropped & (1 << (pop_bit + 3))),
            "core_output_ready": bool(dropped & (1 << (pop_bit + 4))), "core_idle": bool(dropped & (1 << (pop_bit + 5))),
            "scale": (dropped >> (pop_bit + 6)) & 3, "frame": (dropped >> (pop_bit + 8)) & 255,
            "group_end": bool(dropped & (1 << (pop_bit + 17))),
        }
    return result


class FrameWriter:
    """Stream strictly ordered fragments to exclusive files, retaining metadata.

    UDP reordering, duplication and loss deliberately fail this diagnostic.
    Payload memory is bounded by one datagram; metadata grows with frame count.
    The current format has no transport FINAL flag, so the hardware counters and an
    external codec decoder must establish completion of the final frame.
    Frame-rate estimates omit the first 100 ms of host arrivals: the blocking
    sensor start/readback can leave initial frames buffered in the UDP socket.
    """

    def __init__(self, output, camera, *, clock=time.monotonic):
        self.output, self.camera, self.clock = Path(output), camera, clock
        self.frames = {}
        self.packets = 0
        self.payload_length_histogram = {}
        self.last_sequence = self.frame_id = None
        self.first_time = self.last_time = None
        self.file = None

    def add(self, packet):
        if (packet.version != 0 or packet.camera != self.camera
                or packet.stream != IMAGE):
            raise PacketError("capture requires current compact image packets for the selected camera")
        if self.last_sequence is not None and packet.sequence != ((self.last_sequence + 1) & 0xffffffff):
            raise PacketError("missing, duplicate or reordered packet sequence")
        if packet.frame_id != self.frame_id:
            if self.frame_id is not None and packet.frame_id != ((self.frame_id + 1) & 0xffffffff):
                raise PacketError("missing, duplicate or reordered frame ID")
            if packet.offset:
                raise PacketError("frame does not start at offset zero")
            self.close()
            name = f"frame-{packet.frame_id:08d}.lwc"
            self.file = (self.output / name).open("xb")
            self.frame_id = packet.frame_id
            self.last_time = self.clock()
            if self.first_time is None:
                self.first_time = self.last_time
            self.frames[str(packet.frame_id)] = {
                "file": name, "bytes": 0, "packets": 0, "last_packet_bytes": 0,
                "first_packet_elapsed": self.last_time - self.first_time}
        frame = self.frames[str(packet.frame_id)]
        if packet.offset != frame["bytes"]:
            raise PacketError("missing, duplicate, overlapping or reordered frame offset")
        self.file.write(packet.payload)
        frame["bytes"] += len(packet.payload)
        frame["packets"] += 1
        frame["last_packet_bytes"] = len(packet.payload)
        length = str(len(packet.payload))
        self.payload_length_histogram[length] = self.payload_length_histogram.get(length, 0) + 1
        self.packets += 1
        self.last_sequence = packet.sequence

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None

    def summary(self):
        warmup = 0.1
        cadence = [(index, frame["first_packet_elapsed"])
                   for index, frame in enumerate(self.frames.values())
                   if frame["first_packet_elapsed"] >= warmup]
        duration = cadence[-1][1] - cadence[0][1] if len(cadence) >= 2 else 0
        window = {
            "basis": "First image-fragment host arrivals after startup warmup; not sensor timestamps.",
            "warmup_seconds": warmup,
            "start_elapsed": cadence[0][1] if cadence else None,
            "end_elapsed": cadence[-1][1] if cadence else None,
            "frames": len(cadence),
        }
        return {"frames": self.frames, "packets": self.packets,
                "payload_length_histogram": self.payload_length_histogram,
                "host_frame_rate_hz": ((cadence[-1][0] - cadence[0][0]) / duration
                                       if duration > 0 else None),
                "host_frame_rate_window": window}


def validate_capture(report):
    """Check evidence before cleanup can clear hardware counters or sticky bits."""
    errors = []
    complete = report.get("capture_complete")
    final = report.get("post_stop")
    if not complete or not final:
        return ["capture or post-stop snapshot is missing"]
    frames, packets = len(report["frames"]), report["packets"]
    if not frames:
        errors.append("no image frames received")
    if complete["camera"][8] != frames or complete["camera"][9] != frames:
        errors.append("hardware started/completed frame counts do not equal received frames")
    if complete["image"][9] != packets or complete["image"][10] != packets:
        errors.append("hardware committed/sent packet counts do not equal received packets")
    if complete["image"][5]:
        errors.append("image packet FIFO did not drain")
    for state in report["snapshots"] + [complete]:
        if state["camera"][6]:
            errors.append("receiver reported an error during capture")
            break
    for state in report["snapshots"] + [complete, final]:
        # Bit 64 retains a malformed LFTag candidate that the solver rejects.
        # It does not invalidate image capture; actual capture, queue-drop and
        # codec faults remain fatal.
        if state["camera"][5] & (4 | 32 | 128) or state["camera"][15]:
            errors.append("camera capture/configuration fault")
            break
        if state["image"][7] or state["image"][12] or state["image"][13]:
            errors.append("image packet channel fault")
            break
        if state.get("isp") is None:
            errors.append("ISP snapshot is unavailable")
            break
        if state["isp"][2] & 64:
            errors.append("ISP protocol error")
            break
        if state["host_faults"][0] or state["host_faults"][2]:
            errors.append("management bus error")
            break
        if state["board"][2]:
            errors.append("Ethernet destination unreachable")
            break
    if not complete["camera"][5] & 2:
        errors.append("camera did not become idle after disarming")
    if final["camera"][6] & ~0x400:
        errors.append("unexpected receiver status after sensor standby")
    if final["camera"][8:10] != complete["camera"][8:10]:
        errors.append("frame counters changed after capture completion")
    if final["image"][9:11] != complete["image"][9:11]:
        errors.append("packet counters changed after capture completion")
    return errors


class DiagnosticCapture:
    def __init__(self, bus, receiver, writer, *, camera, address, destination,
                 port, board_ip, seconds, transport="etherbone", sensor=None,
                 clock=time.monotonic, sleep=time.sleep, wait=select.select):
        if type(camera) is not int or camera not in (0, 1):
            raise ValueError("camera must explicitly select 0 or 1")
        if type(address) is not int or not 0 <= address <= 0x7f:
            raise ValueError("address must be an unshifted seven-bit sensor address")
        if not math.isfinite(seconds) or not 0 < seconds <= 60:
            raise ValueError("seconds must be finite and in (0, 60]")
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("image port must be in 1..65535")
        self.bus, self.receiver, self.writer = bus, receiver, writer
        self.camera, self.address = camera, address
        self.destination = int(ipaddress.IPv4Address(destination))
        self.board_ip = str(ipaddress.IPv4Address(board_ip))
        self.port, self.seconds = port, seconds
        self.base, self.isp = 0x30024000 + camera * 0x100, 0x30000000 + camera * 0x10000
        self.image = 0x30024400 + camera * 0x180
        self.fault_base = 0x30024880 if transport == "usb" else 0x30024800
        self.sensor = sensor if sensor is not None else create_camera_i2c(bus, 0x30020000 + camera * 0x1000)
        self.clock, self.sleep, self.wait = clock, sleep, wait
        self.report = {"ok": False, "camera": camera, "address": address,
                       "destination": str(ipaddress.IPv4Address(destination)), "port": port,
                       "board_ip": self.board_ip, "seconds_requested": seconds,
                       "test_calibration_only": True, "snapshots": [],
                       "ignored_datagrams": 0, "cleanup_errors": [],
                       "codec_validation": "Not performed: run the Rust decoder with --compact on saved frames.",
                       "stream_off_note": "Only a line-count sticky bit (0x400) arising after clean capture completion is allowed after standby: stopping an unadmitted sensor frame may truncate it."}

    def write(self, address, value):
        self.bus.write(address, value)
        if int(self.bus.read(address)) != value:
            raise CaptureError(f"register readback mismatch at 0x{address:08x}")

    def read_words(self, address, count):
        # Conservative limit shared by Etherbone and USB; no negotiated burst ABI needed.
        result = []
        for offset in range(0, count, 8):
            amount = min(8, count - offset)
            words = self.bus.read(address + offset * 4, length=amount)
            result.extend(map(int, words))
        return result

    def snapshot(self, *, include_isp=True):
        camera = self.read_words(self.base, 16)
        # A sticky capture fault holds MonoIsp in reset, including its bus
        # responder. Reading it would create new host timeouts and overwrite
        # the diagnostics we are trying to preserve.
        result = {"camera": camera, "host_faults": self.read_words(self.fault_base, 3),
                  "board": self.read_words(0, 4), "image": self.read_words(self.image, 14)}
        # Require the diagnostic magic before interpreting these fields.
        lftag = decode_lftag_diagnostics(self.read_words(self.base + 0x40, 4))
        if lftag is not None:
            result["lftag"] = lftag
        if include_isp and not camera[5] & 4:
            # A fault can arrive during those diagnostic bursts. Recheck just
            # before accessing the reset-sensitive ISP responder.
            camera[5] = int(self.bus.read(self.base + 0x14))
        if camera[5] & 4 or not include_isp:
            result.update(isp=None, isp_read_skipped=("camera fault holds ISP in reset"
                if camera[5] & 4 else "omitted during failure diagnostics"))
        else:
            result["isp"] = self.read_words(self.isp, 5)
        return result

    def receive_pending(self):
        """Consume a bounded batch; return whether the socket became empty."""
        # A hostile/unrelated sender cannot make this loop unbounded.
        for _ in range(256):
            try:
                data, source = self.receiver.recvfrom(2048)
            except BlockingIOError:
                return True
            if source[0] != self.board_ip:
                self.report["ignored_datagrams"] += 1
                continue
            packet = parse_datagram(data)
            if packet.camera != self.camera:
                self.report["ignored_datagrams"] += 1
                continue
            self.writer.add(packet)
        return False

    def configure(self, *, firmware_isp=False):
        """Configure this stopped pipeline; pair RESET/initialization is external."""
        require_pair_idle(self.bus)
        # TEST ONLY: enough to admit image frames, not measured camera calibration.
        for offset, value in ((0x80, round((1 << 30) / 800)), (0x84, round((1 << 30) / 800)),
                              (0x88, 640 << 16), (0x8c, 400 << 16), (0x90, 1 << 16)):
            self.write(self.base + offset, value)
        self.bus.write(self.base + 0x34, 1)
        if self.bus.read(self.base + 0x38) != 1:
            raise CaptureError("test calibration commit failed")
        if self.bus.read(self.isp) != 0x4d495350:
            raise CaptureError("unsupported ISP register block")
        if not firmware_isp:
            self.write(self.isp + 4, 0)
            self.write(self.isp + 12, 0)
            self.write(self.isp + 4, 1)
        deadline = self.clock() + 1
        while not int(self.bus.read(self.isp + 8)) & 4:
            if self.clock() >= deadline:
                raise TimeoutError("ISP defect-ready timeout")
            self.sleep(0.001)
        for channel in range(3):
            base = self.image + channel * 0x80
            self.write(base, 0)
            self.bus.write(base + 0x20, 1)
        self.write(self.image + 4, self.destination)
        self.write(self.image + 8, self.port)
        self.write(self.image, 1)
        self.write(0, 1)

    def preflight(self):
        """Check this camera without writes; dual capture preflights both first."""
        self.report["identity"] = BootClient(self.bus).identity()
        initial = self.report["initial"] = self.snapshot()
        if initial["camera"][0] or not initial["camera"][5] & 2 or initial["isp"][2] & 2:
            raise CaptureError("selected camera must be disarmed and idle, with its ISP idle")
        product_levels = [initial["image"][5]] + [
            int(self.bus.read(self.image + channel * 0x80 + 0x14))
            for channel in (1, 2)]
        self.report["initial_product_fifo_levels"] = dict(zip(
            ("image", "children", "pose"), product_levels))
        if any(product_levels):
            raise CaptureError(
                "selected camera still has queued product packets; wait for them to drain "
                "or reload a stalled design before capture (soft reset would abort queued packets)")
        if initial["camera"][3] != 5:
            raise CaptureError("selected sensor must have XVCLK enabled and shutdown released")
        if self.sensor.read_register(self.address, 0x300a, 2) != bytes.fromhex("9281"):
            raise CaptureError("selected sensor is not an OV9281")
        if self.sensor.read_register(self.address, 0x0100) != b"\0":
            raise CaptureError("selected sensor must already be stopped")
        if self.sensor.read_register(self.address, 0x3808, 4) != bytes.fromhex("05000320"):
            raise CaptureError("diagnostic requires configured 1280x800 geometry")
        return initial

    def run(self):
        raise CaptureError("single-camera capture is disabled; use tools.lutcam.capture_dual")


def main(argv=None):
    parser = argparse.ArgumentParser(description="Single-camera capture is disabled. Use tools.lutcam.capture_dual for paired acquisition.")
    parser.parse_known_args(argv)
    print(json.dumps({"ok": False, "error": "Single-camera capture is disabled; use tools.lutcam.capture_dual."}))
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
