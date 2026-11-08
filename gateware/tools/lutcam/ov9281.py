"""Configure one explicitly selected member of the synchronized OV9281 pair.

Example: python -m tools.lutcam.ov9281 --camera 1 --address 0x10 power-on
Then use configure; streaming and capture start/stop belong to paired capture.
The FPGA is the synchronization master; both cameras are FSIN slaves. Mutating
operations require paired-control firmware and both capture paths stopped.
Only one management client may operate camera pins or configuration at a time.
The mode's clock rates are calculated targets, not measurements of the link.
No action scans addresses, initializes the other camera, or touches flash.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from .i2c import create_camera_i2c
from .flash import add_connection_arguments, create_transport


MODE_NAME = "raw10-1280x800-25mhz-60fps-bars"
MODE_NAMES = {60: MODE_NAME, 120: "raw10-1280x800-25mhz-120fps-bars"}
FRAME_LENGTHS = {60: 1822, 120: 910}
CHIP_ID = 0x9281
CONTROL_BASE = 0x30024000
I2C_BASE = 0x30020000
PAIR_BASE = 0x30024200
PAIR_MAGIC = 0x4c435031
PAIR_RUN = 0x08
PAIR_STATUS = 0x0c
PAIR_FAULT = 0x14
PAIR_STATUS_BUSY = 0x0f
PAIR_STATUS_DRAINED = 1 << 4
PAIR_STATUS_FAULT = 1 << 5
SYNC_SETTLE_SECONDS = 0.066
SYNC_REFERENCE_CLOCK_HZ = 120_000_000
SYNC_PULSE_WIDTH_CYCLES = 1536
# Counter-inclusive line total observed during board bring-up: at 120 MHz,
# 1.5 * HTS * (VTS + 1) cycles matches the sensor's 80 MHz timing domain.
SYNC_PERIOD_CYCLES = {fps: 1092 * (vts + 1) for fps, vts in FRAME_LENGTHS.items()}
PAIRED_CAPTURE_GUIDANCE = (
    "Single-camera start/stop is disabled; use tools.lutcam.capture_dual "
    "to start and stop the synchronized camera pair."
)
SYNC_SOURCE = (
    "https://github.com/antmicro/tx2-deep-learning-kit-bsp/blob/master/"
    "kernel/kernel-4.4/drivers/media/i2c/ov9281_mode_tbls.h#L49-L60"
)

# Register facts from Linux's ov9282 common_regs/mode_1280x800_regs:
# https://raw.githubusercontent.com/torvalds/linux/master/drivers/media/i2c/ov9282.c
# PLLs are adapted using OV9281_CSP5_DS_1.01.pdf, tables 2-9/2-10.
# Each tuple is (16-bit register, byte value), in programming order.
# Keep 0x3022=0x01 from the working driver: the preliminary datasheet's
# inferred 0x51 differs. RAW10 also requires 0x3662=0x05 and the PLL2 settings.
# Omit the driver's 0x0101 and 0x1000 writes: the OV9281 datasheet marks
# 0x0101 reserved, does not define 0x1000, and both read zero on this sensor.
MODE_WRITES = (
    (0x0100, 0x00),
    (0x0302, 0x30), (0x030e, 0x02), (0x3001, 0x00), (0x3004, 0x00),
    (0x3005, 0x00), (0x3006, 0x04), (0x3011, 0x0a), (0x3013, 0x18),
    (0x301c, 0xf0), (0x3022, 0x01), (0x3030, 0x10), (0x3039, 0x32),
    (0x303a, 0x00), (0x3503, 0x08), (0x3505, 0x8c), (0x3507, 0x03),
    (0x3508, 0x00), (0x3610, 0x80), (0x3611, 0xa0), (0x3620, 0x6e),
    (0x3632, 0x56), (0x3633, 0x78), (0x3666, 0x00), (0x366f, 0x5a),
    (0x3680, 0x84), (0x3712, 0x80), (0x372d, 0x22), (0x3731, 0x80),
    (0x3732, 0x30), (0x377d, 0x22), (0x3788, 0x02), (0x3789, 0xa4),
    (0x378a, 0x00), (0x378b, 0x4a), (0x3799, 0x20), (0x3881, 0x42),
    (0x38a8, 0x02), (0x38a9, 0x80), (0x38b1, 0x00), (0x38c4, 0x00),
    (0x38c5, 0xc0), (0x38c6, 0x04), (0x38c7, 0x80), (0x3920, 0xff),
    (0x4010, 0x40), (0x4043, 0x40), (0x4307, 0x30), (0x4317, 0x00),
    (0x4501, 0x00), (0x450a, 0x08), (0x4601, 0x04), (0x470f, 0x00),
    (0x4f07, 0x00), (0x5000, 0x9f), (0x5001, 0x00), (0x5e00, 0x00),
    (0x5d00, 0x07), (0x5d01, 0x00), (0x5a08, 0x84),
    # Full 1280 x 800 active geometry, with eight border pixels skipped.
    (0x3778, 0x00), (0x3800, 0x00), (0x3801, 0x00), (0x3802, 0x00),
    (0x3803, 0x00), (0x3804, 0x05), (0x3805, 0x0f), (0x3806, 0x03),
    (0x3807, 0x2f), (0x3808, 0x05), (0x3809, 0x00), (0x380a, 0x03),
    (0x380b, 0x20), (0x3810, 0x00), (0x3811, 0x08), (0x3812, 0x00),
    (0x3813, 0x08), (0x3814, 0x11), (0x3815, 0x11), (0x3820, 0x40),
    (0x3821, 0x00), (0x4003, 0x40), (0x4008, 0x04), (0x4009, 0x0b),
    (0x400c, 0x00), (0x400d, 0x07), (0x4507, 0x00), (0x4509, 0x00),
    # 25 / 1.5 * 48 = 800 MHz VCO; /8 byte clock; /5 /2 SYS clock.
    (0x030a, 0x00), (0x0300, 0x01), (0x0301, 0x00), (0x0302, 0x30),
    (0x0303, 0x00), (0x0304, 0x03), (0x0305, 0x02), (0x0306, 0x01),
    # 25 / 2.5 * 64 = 640 MHz VCO; /4 /2 SYS; /8 SA1; /2 DAC.
    (0x0314, 0x00), (0x030b, 0x03), (0x030c, 0x00), (0x030d, 0x40),
    (0x030f, 0x03), (0x030e, 0x02), (0x0312, 0x07), (0x0313, 0x01),
    # RAW10, HTS728, VTS1822, exposure400 lines, unity gain.
    (0x3662, 0x05), (0x380c, 0x02), (0x380d, 0xd8), (0x380e, 0x07),
    (0x380f, 0x1e), (0x3500, 0x00), (0x3501, 0x19), (0x3502, 0x00),
    (0x3509, 0x10),
    # Continuous clock, no LS/LE packets, automatic PHY timings, no ULPS.
    (0x4800, 0x00), (0x4802, 0x00), (0x4806, 0x00),
    (0x4320, 0x80), (0x5e00, 0x80),
)
MODE_FINAL = dict(MODE_WRITES)
RECEIVER_STATUS_NAMES = (
    "header_ecc_corrected", "header_ecc_uncorrectable", "payload_crc_error",
    "sot_sync_error", "lane_alignment_error", "lp_sequence_error",
    "unexpected_packet", "word_count_error", "packet_length_error",
    "frame_sequence_error", "line_count_error", "pixel_fifo_overflow",
    "clock_stop_error", "initialization_error",
)


def mode_writes(pattern="bars", solid=0x200, *, fps=60):
    """Select a frame-length preset; CSI lane rate and active geometry stay fixed.

    The names 60/120 denote nominal rates: the divider-derived rates are about
    60.313/120.758 Hz. They are not measurements of the physical sensor clock.
    """
    if isinstance(fps, bool) or not isinstance(fps, int) or fps not in FRAME_LENGTHS:
        raise ValueError("fps must be the integer preset 60 or 120")
    if pattern not in ("bars", "off", "solid"):
        raise ValueError("pattern must be bars, off or solid")
    if isinstance(solid, bool) or not isinstance(solid, int) or not 0 <= solid <= 1023:
        raise ValueError("solid pixel value must be a ten-bit integer")
    if pattern == "bars" and fps == 60:
        return MODE_WRITES
    vts = FRAME_LENGTHS[fps]
    replacements = {0x380e: vts >> 8, 0x380f: vts & 255}
    if pattern != "bars":
        replacements.update({0x5e00: 0, 0x4320: 0x82 if pattern == "solid" else 0x80})
    writes = tuple((reg, replacements.get(reg, value)) for reg, value in MODE_WRITES)
    if pattern == "solid":
        # P1/P2/P4/P3 all receive the same value: no Bayer/pixel-order ambiguity.
        writes += tuple((0x4322 + index, (solid >> 8) if index % 2 == 0 else solid & 255)
                        for index in range(8))
    return writes


def mode_manifest(pattern="bars", solid=0x200, *, fps=60):
    """Return a serializable, hardware-independent description of this mode."""
    writes = mode_writes(pattern, solid, fps=fps)
    vts = FRAME_LENGTHS[fps]
    return {
        "name": MODE_NAMES[fps], "fps_preset": fps,
        "pattern": pattern, "solid_pixel": solid if pattern == "solid" else None,
        "input_clock_hz": 25_000_000,
        "width": 1280, "height": 800, "lanes": 2, "bits_per_pixel": 10,
        "expected_byte_clock_hz": 100_000_000,
        "expected_lane_rate_bps": 800_000_000,
        "hts": 728, "vts": vts,
        "derived_frame_rate_hz": 80_000_000 / (728 * vts),
        "hardware_validated": False,
        "note": "Register readback does not measure clocks or establish CSI packet/frame correctness.",
        "provenance": {
            "mode_table": "https://raw.githubusercontent.com/torvalds/linux/master/drivers/media/i2c/ov9282.c",
            "datasheet": "docs/OV9281_CSP5_DS_1.01.pdf, tables 2-9/2-10, PDF pages 32-34",
            "omitted_driver_registers": {
                "0x0101": "Reserved in OV9281 table 7-1; observed readback zero",
                "0x1000": "Undocumented in OV9281 datasheet; observed readback zero",
            },
        },
        "pll_derivation": {
            "pll1": "25 MHz /1.5 *48 =800 MHz VCO; /8 =100 MHz byte; /5 /2 =80 MHz SYS",
            "pll2": "25 MHz /2.5 *64 =640 MHz VCO; /4 /2 =80 MHz SYS; /8 =80 MHz SA1; /2 =320 MHz DAC",
        },
        "writes": [list(pair) for pair in writes],
        "stream_on_included": False,
    }


def board_mode_writes(camera, pattern="bars", solid=0x200, *, fps=60):
    """Describe the fixed board role, with FSYNC enabled only after all timing.

    First disable an earlier slave role while already in standby. NVIDIA's
    OV9281 driver suppresses VTS changes under FSYNC because they can destabilize
    streaming. The role-selection tail follows every PLL/HTS/VTS write. Pin
    direction safety is guaranteed by the required paired-control firmware.
    """
    if isinstance(camera, bool) or not isinstance(camera, int) or camera not in (0, 1):
        raise ValueError("camera must explicitly select slave camera 0 or 1")
    base = mode_writes(pattern, solid, fps=fps)
    return ((0x3823, 0x00),) + base + (
        (0x0106, 0x00),  # Complete the current frame before software standby.
        (0x4f00, 0x00),  # Normal SYS-clock timing, not low-power snapshot mode.
        (0x3027, 0x00), (0x3666, 0x00),  # Normal VSYNC/FSIN signal paths.
        # Datasheet reset defaults: SOF-derived VSYNC, 1024 SYS clocks wide,
        # delayed by 256 SYS clocks. These do not establish exposure alignment.
        (0x4311, 0x04), (0x4312, 0x00), (0x4313, 0x00),
        (0x4314, 0x00), (0x4315, 0x01), (0x4316, 0x00),
        (0x3006, 0x00),  # Both sensors receive FPGA-driven FSIN pulses.
        (0x3823, 0x30),  # Continuous external frame sync, not snapshot sleep.
    )


def board_mode_manifest(camera, pattern="bars", solid=0x200, *, fps=60):
    """Export the actual board sequence, including its fixed synchronization role."""
    writes = board_mode_writes(camera, pattern, solid, fps=fps)
    result = mode_manifest(pattern, solid, fps=fps)
    result["camera"] = camera
    result["writes"] = [list(pair) for pair in writes]
    result["synchronization"] = {
        "role": "slave", "master": "fpga", "slave_cameras": [0, 1],
        "routing": "FPGA outputs -> camera0 FSIN and camera1 FSIN inputs, with a shared period and independent phases",
        "phase_configuration_owner": "paired capture client",
        "fpga_fsin_direction": "output", "sensor_fsin_direction": "input",
        "generator_reference_clock_hz": SYNC_REFERENCE_CLOCK_HZ,
        "generator_period_cycles": SYNC_PERIOD_CYCLES[fps],
        "generator_pulse_width_cycles": SYNC_PULSE_WIDTH_CYCLES,
        "derived_trigger_rate_hz": SYNC_REFERENCE_CLOCK_HZ / SYNC_PERIOD_CYCLES[fps],
        "generator_period_derivation": "120 MHz / 80 MHz * HTS * (VTS + 1)",
        "generator_period_note": (
            "VTS+1 matches the measured 120 fps board mode; the 60 fps pulse "
            "preset is derived from that model and needs physical validation."
        ),
        "settle_after_writes_seconds": SYNC_SETTLE_SECONDS,
        "sensor_vsync_output_enabled": False,
        "graceful_standby": True,
        "required_pair_control": {
            "base": PAIR_BASE, "magic": PAIR_MAGIC,
        },
        "note": (
            "Continuous frame-sync slave mode; external pulses set synchronized "
            "frame timing. Frame synchronization does not by itself prove exposure alignment."
        ),
    }
    result["provenance"]["synchronization_table"] = SYNC_SOURCE
    result["provenance"]["synchronization_datasheet"] = (
        "docs/OV9281_CSP5_DS_1.01.pdf, PDF pages 23-24, 69, 83, 85, 98"
    )
    result["provenance"]["fsync_vts_workaround"] = (
        "https://github.com/antmicro/tx2-deep-learning-kit-bsp/blob/master/"
        "kernel/kernel-4.4/drivers/media/i2c/ov9281.c#L399-L430"
    )
    return result


class CameraError(RuntimeError):
    """The selected sensor or a required readback did not match expectations."""


def require_pair_idle(bus):
    """Require the fixed-role firmware and a healthy, stopped, drained pair.

    This guard performs only CSR reads. A single management client must own
    camera control throughout configuration; these reads do not acquire a lock.
    """
    magic = int(bus.read(PAIR_BASE))
    if magic != PAIR_MAGIC:
        raise CameraError(
            "Sensor mutation requires paired-control firmware "
            f"magic 0x{PAIR_MAGIC:08x}; observed 0x{magic:08x}. "
            "Both sensors require the shared FPGA FSIN generator and paired capture control."
        )
    for camera in (0, 1):
        base = CONTROL_BASE + camera * 0x100
        run = int(bus.read(base))
        status = int(bus.read(base + 0x14))
        if (run != 0 or status == 0xffffffff or not status & 2 or status & 5):
            raise CameraError(
                f"Both cameras must be disarmed and idle before sensor mutation; "
                f"camera {camera} has RUN={run:#x}, status={status:#x}. "
                "Stop or reset the paired capture first."
            )
    run = int(bus.read(PAIR_BASE + PAIR_RUN))
    status = int(bus.read(PAIR_BASE + PAIR_STATUS))
    fault = int(bus.read(PAIR_BASE + PAIR_FAULT))
    if (run != 0 or status == 0xffffffff or fault != 0
            or status & (PAIR_STATUS_BUSY | PAIR_STATUS_FAULT)
            or not status & PAIR_STATUS_DRAINED):
        raise CameraError(
            "The camera pair must be stopped, drained and fault-free before "
            f"sensor mutation (RUN={run:#x}, status={status:#x}, fault={fault:#x}); "
            "stop or reset the paired capture first."
        )
    return status


class OV9281:
    """Bounded operations on exactly one camera, with no automatic retries.

    A failed or interrupted configure leaves the sensor in standby with a
    partially applied table. Re-run configure after diagnosing the error.
    Public individual stream transitions are refused; the paired coordinator
    verifies both sensors before starting the synchronized capture operation.
    Capture calibration, ISP and transport setup remain the caller's policy.
    """

    def __init__(self, bus, *, camera, address, i2c=None, half_period=5e-6,
                 stretch_timeout=0.1, sleep=time.sleep, clock=time.monotonic):
        if isinstance(camera, bool) or not isinstance(camera, int) or camera not in (0, 1):
            raise ValueError("camera must explicitly select 0 or 1")
        if (isinstance(address, bool) or not isinstance(address, int)
                or not 0 <= address <= 0x7f):
            raise ValueError("address must be an explicit unshifted seven-bit sensor address")
        if (not math.isfinite(half_period) or not 0 <= half_period <= 1
                or not math.isfinite(stretch_timeout) or not 0 < stretch_timeout <= 60):
            raise ValueError("I2C timings must be finite: half-period 0..1 s, stretch timeout (0,60] s")
        self.bus, self.camera, self.address = bus, camera, address
        self.base = CONTROL_BASE + camera * 0x100
        self.i2c_base = I2C_BASE + camera * 0x1000
        self.i2c = i2c if i2c is not None else create_camera_i2c(
            bus, self.i2c_base, half_period=half_period, stretch_timeout=stretch_timeout)
        self.sleep, self.clock = sleep, clock

    def _read_control(self, offset):
        value = int(self.bus.read(self.base + offset))
        if value == 0xffffffff:
            raise CameraError(f"camera control read failed at 0x{self.base + offset:08x}")
        return value

    def _write_control(self, offset, value):
        self.bus.write(self.base + offset, value)
        actual = self._read_control(offset)
        if actual != value:
            raise CameraError(f"camera control readback mismatch at +0x{offset:x}: {actual:#x} != {value:#x}")

    def _require_idle(self):
        require_pair_idle(self.bus)

    def _require_powered(self):
        if self._read_control(0x0c) != 5:
            raise CameraError("sensor requires shutdown released and 25 MHz XVCLK enabled (control=5)")

    @staticmethod
    def _validate_read(register, length):
        if (isinstance(register, bool) or not isinstance(register, int)
                or not 0 <= register <= 0xffff or isinstance(length, bool)
                or not isinstance(length, int) or not 1 <= length <= 256
                or register + length > 0x10000):
            raise ValueError("read needs a sixteen-bit register range and 1..256 bytes")

    def read_register(self, register, length=1):
        self._validate_read(register, length)
        result = bytes(self.i2c.read_register(self.address, register, length))
        if len(result) != length:
            raise CameraError("sensor register read returned a partial response")
        return result

    def _verify_register(self, register, expected):
        observed = self.read_register(register)[0]
        if observed != expected:
            raise CameraError(f"sensor register 0x{register:04x}: expected 0x{expected:02x}, got 0x{observed:02x}")

    def _write_register(self, register, value):
        self.i2c.write_register(self.address, register, value)
        self._verify_register(register, value)

    def identify(self):
        identity = int.from_bytes(self.read_register(0x300a, 2), "big")
        if identity != CHIP_ID:
            raise CameraError(f"unexpected sensor ID 0x{identity:04x}; expected 0x{CHIP_ID:04x}")
        return identity

    def power_on(self):
        """Hardware-reset the selected idle sensor and verify its identity."""
        self._require_idle()
        self._write_control(0x0c, 4)
        self.sleep(0.001)
        self._write_control(0x0c, 5)
        # >=8192 XVCLK cycles after shutdown release, per datasheet table2-5.
        self.sleep(0.001)
        self.i2c.release()
        return {"camera": self.camera, "address": self.address,
                "chip_id": self.identify(), "sensor_control": 5}

    def configure(self, mode=None, *, pattern="bars", solid=0x200, fps=60, progress=None):
        writes = board_mode_writes(self.camera, pattern, solid, fps=fps)
        if mode is not None and mode != MODE_NAMES[fps]:
            raise ValueError(f"mode {mode!r} does not match the selected {fps} fps preset")
        self._require_idle()
        self._require_powered()
        self.identify()
        if self.read_register(0x0100)[0] != 0:
            raise CameraError("sensor is streaming; stop it before configuring PLLs or timing")
        for index, (register, value) in enumerate(writes):
            self._write_register(register, value)
            if progress is not None:
                progress(index + 1, len(writes))
        self.sleep(SYNC_SETTLE_SECONDS)
        verified = self.verify_mode(pattern=pattern, solid=solid, fps=fps)
        return {"mode": MODE_NAMES[fps], "fps_preset": fps,
                "camera": self.camera, "role": "slave", "synchronization_master": "fpga",
                "derived_frame_rate_hz": 80_000_000 / (728 * FRAME_LENGTHS[fps]),
                "pattern": pattern, "writes_verified": len(writes),
                "final_registers_verified": verified, "streaming": False,
                "note": "Mode register readbacks passed; physical CSI validation is separate."}

    def verify_mode(self, *, pattern="bars", solid=0x200, fps=60):
        """Verify every final mode byte, with no sensor writes."""
        final = dict(board_mode_writes(self.camera, pattern, solid, fps=fps))
        for register, expected in final.items():
            self._verify_register(register, expected)
        return len(final)

    def start(self, *, pattern="bars", solid=0x200, fps=60):
        """Refuse individual stream-on before any hardware access."""
        raise CameraError(PAIRED_CAPTURE_GUIDANCE)

    def stop(self):
        """Refuse individual stream-off before any hardware access."""
        raise CameraError(PAIRED_CAPTURE_GUIDANCE)

    def status(self):
        """Read controls and a small sensor snapshot without any writes."""
        control = self._read_control(0x0c)
        status = self._read_control(0x14)
        receiver = self._read_control(0x18)
        result = {
            "camera": self.camera, "address": self.address,
            "expected_role": "slave", "synchronization_master": "fpga",
            "run": self._read_control(0), "sensor_control": control,
            "camera_status": status, "receiver_status": receiver,
            "receiver_errors": [name for bit, name in enumerate(RECEIVER_STATUS_NAMES)
                                if receiver & (1 << bit)],
            "receiver_initialized": bool(status & 8),
            "capture_fault": bool(status & 4),
            "frames_started": self._read_control(0x20),
            "frames_completed": self._read_control(0x24),
        }
        # A powered-down sensor cannot answer SCCB; retain useful FPGA status.
        if control & 5 == 5:
            result["chip_id"] = self.identify()
            registers = (0x0100, 0x0106, 0x0300, 0x0302, 0x030b, 0x030d,
                         0x3006, 0x3022, 0x3027, 0x3039, 0x3662, 0x3666,
                         0x3823, 0x4800, 0x5e00)
            result["sensor_registers"] = {f"0x{reg:04x}": self.read_register(reg)[0]
                                          for reg in registers}
            result["width"] = int.from_bytes(self.read_register(0x3808, 2), "big")
            result["height"] = int.from_bytes(self.read_register(0x380a, 2), "big")
            result["hts"] = int.from_bytes(self.read_register(0x380c, 2), "big")
            result["vts"] = int.from_bytes(self.read_register(0x380e, 2), "big")
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_arguments(parser)
    parser.add_argument("--camera", type=int, choices=(0, 1), required=True)
    parser.add_argument("--address", type=lambda value: int(value, 0), required=True,
                        help="explicit unshifted seven-bit SCCB address; never scanned")
    parser.add_argument("--half-period", type=float, default=5e-6)
    parser.add_argument("--stretch-timeout", type=float, default=0.1)
    operations = parser.add_subparsers(dest="operation", required=True)
    for name in ("power-on", "stop", "status"):
        operations.add_parser(name)
    start = operations.add_parser("start")
    configure = operations.add_parser("configure")
    configure.add_argument("--mode", choices=tuple(MODE_NAMES.values()),
                           help="optional explicit mode name; must match --fps")
    read = operations.add_parser("read")
    read.add_argument("register", type=lambda value: int(value, 0))
    read.add_argument("--length", type=int, default=1)
    export = operations.add_parser("export-mode", help="write mode/provenance JSON without opening hardware")
    export.add_argument("output", type=Path)
    for operation in (configure, start, export):
        operation.add_argument("--fps", type=int, choices=(60, 120), default=60,
                               help="nominal frame-rate preset; actual calculated rate is slightly higher")
        operation.add_argument("--pattern", choices=("bars", "off", "solid"), default="bars")
        operation.add_argument("--solid", type=lambda value: int(value, 0), default=0x200,
                               help="ten-bit constant when --pattern solid (default0x200)")
    args = parser.parse_args(argv)
    try:
        if args.operation in ("start", "stop"):
            raise CameraError(PAIRED_CAPTURE_GUIDANCE)
        # Validate selection and timings before opening a network/serial device.
        OV9281(None, camera=args.camera, address=args.address,
               half_period=args.half_period, stretch_timeout=args.stretch_timeout)
        if args.operation == "read":
            OV9281._validate_read(args.register, args.length)
        if args.operation in ("configure", "start", "export-mode"):
            mode_writes(args.pattern, args.solid, fps=args.fps)
        if args.operation == "configure" and args.mode not in (None, MODE_NAMES[args.fps]):
            raise ValueError("--mode must match --fps")
        if args.operation == "export-mode":
            manifest = board_mode_manifest(args.camera, args.pattern, args.solid, fps=args.fps)
            manifest.update(camera=args.camera, device_address=args.address)
            args.output.write_text(json.dumps(manifest, indent=2) + "\n")
            result = {"output": str(args.output), "mode": MODE_NAMES[args.fps], "fps_preset": args.fps}
        else:
            with create_transport(args) as bus:
                sensor = OV9281(bus, camera=args.camera, address=args.address,
                                half_period=args.half_period, stretch_timeout=args.stretch_timeout)
                if args.operation == "configure":
                    result = sensor.configure(args.mode, pattern=args.pattern, solid=args.solid, fps=args.fps)
                elif args.operation == "start":
                    result = sensor.start(pattern=args.pattern, solid=args.solid, fps=args.fps)
                elif args.operation == "read":
                    result = {"register": args.register,
                              "data": sensor.read_register(args.register, args.length).hex(" ")}
                else:
                    result = getattr(sensor, args.operation.replace("-", "_"))()
        print(json.dumps(result, indent=2))
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(json.dumps({"ok": False, "error": f"{type(error).__name__}: {error}"}, indent=2))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
