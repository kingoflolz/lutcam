"""Verified access to the fixed paired-camera control and timing ABI."""

from __future__ import annotations

import math
import time

from .ov9281 import PAIR_BASE, PAIR_MAGIC, require_pair_idle


REQUESTED, ARMED, ACTIVE, STOPPING = 1, 2, 4, 8
DRAINED, FAULT, LOCKED, READY, SNAPSHOT_VALID = 16, 32, 64, 128, 512
PERIOD_CYCLES = {60: 1_990_716, 120: 994_812}
WIDTH_CYCLES = 1536
SYS_HZ = 120_000_000


class PairError(RuntimeError):
    """Paired capture hardware or its coherent evidence failed validation."""


def signed32(value):
    return value - (1 << 32) if value & (1 << 31) else value


def quantize_phases(fps, phases_us=(0, 0)):
    if fps not in PERIOD_CYCLES or len(phases_us) != 2:
        raise ValueError("phase configuration requires FPS60/120 and two offsets")
    period = PERIOD_CYCLES[fps]
    result = []
    for value in phases_us:
        if isinstance(value, bool) or not math.isfinite(value) or not 0 <= value < period * 1e6 / SYS_HZ:
            raise ValueError("phase offsets must be finite, nonnegative and shorter than one FSYNC period")
        cycles = round(value * SYS_HZ / 1e6)
        if cycles >= period:
            raise ValueError("phase offset rounds outside the FSYNC period")
        result.append({"requested_us": value, "cycles": cycles, "actual_us": cycles * 1e6 / SYS_HZ})
    return result


class PairedCaptureControl:
    def __init__(self, bus, *, clock=time.monotonic, sleep=time.sleep):
        self.bus, self.clock, self.sleep = bus, clock, sleep

    def read(self, offset):
        value = int(self.bus.read(PAIR_BASE + offset))
        if value == 0xffffffff:
            raise PairError(f"paired control read failed at +0x{offset:02x}")
        return value

    def identity(self):
        if self.read(0) != PAIR_MAGIC:
            raise PairError("paired-control magic mismatch; paired firmware is required")

    def require_idle(self):
        return require_pair_idle(self.bus)

    def disarmed_drained(self):
        self.identity()
        status = self.read(0x0c)
        if self.read(0x08) or status & 15 or not status & DRAINED:
            raise PairError("paired operation requires disarmed, drained acquisition")
        return status

    def write_verified(self, offset, value):
        self.identity()
        self.bus.write(PAIR_BASE + offset, value)
        if self.read(offset) != value:
            raise PairError(f"paired register readback mismatch at +0x{offset:02x}")

    def reset(self):
        # Recovery must remain possible when a stopped pipeline has sticky
        # faults. The caller must first verify both sensors are in standby.
        self.disarmed_drained()
        self.bus.write(PAIR_BASE + 0x10, 1)
        self.read(0x08)  # Ordered completion barrier before checking CSI initialization.

    def clear(self):
        # Warmup may have diagnostic faults; CLEAR preserves the active D-PHY phase.
        self.disarmed_drained()
        self.bus.write(PAIR_BASE + 0x60, 1)
        self.read(0x08)

    def configure_generator(self, fps, phases_us=(0, 0)):
        if fps not in PERIOD_CYCLES:
            raise ValueError("paired FPS preset must be 60 or 120")
        phases = quantize_phases(fps, phases_us)
        self.disarmed_drained()
        if self.read(0x6c):
            raise PairError("FSYNC generator must be disabled before configuring timing")
        period = PERIOD_CYCLES[fps]
        minimum, maximum = self.read(0x88), self.read(0x8c)
        if not 1 <= minimum < maximum < period:
            raise PairError("SOF latency limits must satisfy 1 <= minimum < maximum < FSYNC period")
        if not 0 < self.read(0x68) < period:
            raise PairError("existing FSYNC width must be shorter than the requested period before reconfiguration")
        # Old phases may exceed a shorter newly selected period. Park both at
        # zero while the generator is disabled, then commit the requested timing.
        self.write_verified(0x90, 0)
        self.write_verified(0x94, 0)
        self.write_verified(0x64, period)
        self.write_verified(0x68, WIDTH_CYCLES)
        for offset, phase in zip((0x90, 0x94), phases):
            self.write_verified(offset, phase["cycles"])
        return {"fps_preset": fps, "sys_hz": SYS_HZ, "period_cycles": period,
                "width_cycles": WIDTH_CYCLES, "derived_trigger_rate_hz": SYS_HZ / period,
                "phases": phases,
                "min_latency_limit_cycles": minimum,
                "max_latency_limit_cycles": maximum,
                "note": "Period derives from the observed VTS+1 cadence; SOF timestamps verify physical operation."}

    def generator(self, enabled):
        self.disarmed_drained()
        self.write_verified(0x6c, int(bool(enabled)))
        return True

    def run(self, enabled):
        self.identity()
        if enabled:
            status = self.disarmed_drained()
            if ((status & (LOCKED | READY | FAULT)) != (LOCKED | READY)
                    or self.read(0x14) or self.read(0x6c) != 1):
                raise PairError("both cameras must be ready, locked and fault-free before paired arm")
        self.write_verified(0x08, int(bool(enabled)))

    def snapshot(self, *, timeout=0.25, after_epoch=None):
        """Latch one coherent raw-SOF epoch; live admission counts are identified separately."""
        if after_epoch is not None and (type(after_epoch) is not int or not 0 <= after_epoch <= 0xffffffff):
            raise ValueError("snapshot freshness requires a uint32 epoch")
        self.identity()
        deadline = self.clock() + timeout
        while True:
            self.bus.write(PAIR_BASE + 0x28, 1)
            status = self.read(0x0c)
            if status & SNAPSHOT_VALID:
                epoch = int(self.bus.read(PAIR_BASE + 0x2c))
                if after_epoch is None or 0 < ((epoch - after_epoch) & 0xffffffff) < 0x80000000:
                    break
            if self.clock() >= deadline:
                raise TimeoutError("no fresh paired SOF snapshot" if after_epoch is not None else "no valid paired SOF snapshot")
            self.sleep(0.002)
        words = []
        for offset, count in ((0x2c, 8), (0x4c, 5)):
            words.extend(int(value) for value in self.bus.read(PAIR_BASE + offset, length=count))
        result = {
            "epoch": words[0], "sof0_cycles": words[1] | words[2] << 32,
            "sof1_cycles": words[3] | words[4] << 32,
            "last_skew_cycles": signed32(words[5]), "min_skew_cycles": signed32(words[6]),
            "max_skew_cycles": signed32(words[7]), "raw_sof0": words[8], "raw_sof1": words[9],
            "vsync": words[10], "matched": words[11], "missing": words[12],
            "status": self.read(0x0c), "fault": self.read(0x14),
            "pairs_started": self.read(0x18), "pairs_completed": self.read(0x1c),
            "max_skew_limit_cycles": self.read(0x24), "sys_hz": SYS_HZ,
            "basis": "Coherent FPGA CSI SOF arrival timestamps, not exposure or UDP timestamps.",
        }
        latency = [int(value) for value in self.bus.read(PAIR_BASE + 0x70, length=6)]
        result.update(anchor_cycles=latency[0] | latency[1] << 32,
                      sof0_latency_cycles=signed32(latency[2]),
                      sof1_latency_cycles=signed32(latency[3]),
                      min_latency_cycles=signed32(latency[4]),
                      max_latency_cycles=signed32(latency[5]),
                      min_latency_limit_cycles=self.read(0x88),
                      max_latency_limit_cycles=self.read(0x8c))
        residual = [signed32(int(value)) for value in self.bus.read(PAIR_BASE + 0x98, length=3)]
        result.update(phase0_cycles=self.read(0x90), phase1_cycles=self.read(0x94),
                      residual_skew_cycles=residual[0], min_residual_skew_cycles=residual[1],
                      max_residual_skew_cycles=residual[2])
        if result["sof1_cycles"] - result["sof0_cycles"] != result["last_skew_cycles"]:
            raise PairError("paired snapshot timestamps and signed skew disagree")
        if any(result[f"sof{camera}_cycles"] - result["anchor_cycles"] - result[f"phase{camera}_cycles"] != result[f"sof{camera}_latency_cycles"]
               for camera in (0, 1)):
            raise PairError("paired snapshot anchor and SOF latencies disagree")
        if result["last_skew_cycles"] - (result["phase1_cycles"] - result["phase0_cycles"]) != result["residual_skew_cycles"]:
            raise PairError("paired snapshot phase-corrected skew disagrees with timestamps")
        return result
