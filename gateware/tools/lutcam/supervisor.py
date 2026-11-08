"""Host commands for the flash-loaded camera supervisor (single host owner)."""
import time

BASE = 0x30028000
MAGIC = 0x4c435256


class SupervisorError(RuntimeError):
    pass


class CameraSupervisor:
    def __init__(self, bus, *, clock=time.monotonic, sleep=time.sleep):
        self.bus, self.clock, self.sleep = bus, clock, sleep

    @classmethod
    def discover(cls, bus, **kwargs):
        # Check the supervisor capability before probing its registers.
        if not int(bus.read(4)) & 16:
            return None
        if int(bus.read(BASE)) != MAGIC:
            raise SupervisorError("camera supervisor identity mismatch")
        return cls(bus, **kwargs)

    def wait(self, *, sequence=None, timeout=5):
        deadline = self.clock() + timeout
        while True:
            hardware = int(self.bus.read(BASE+4))
            if hardware & ~3:
                raise SupervisorError(f"camera supervisor hardware fault 0x{hardware:08x}")
            state = int(self.bus.read(BASE+8))
            completed = int(self.bus.read(BASE+0x20))
            if hardware == 1 and state & 5 and (sequence is None or completed == sequence):
                if state & 4:
                    raise SupervisorError(f"camera firmware error {self.bus.read(BASE+12)}")
                return
            if self.clock() >= deadline:
                raise TimeoutError("camera supervisor did not complete its operation")
            self.sleep(0.002)

    def command(self, opcode, arg0=0, arg1=0):
        # Do not re-submit uncertain operations; arguments precede commit.
        sequence = int(self.bus.read(BASE+0x10))
        if int(self.bus.read(BASE+0x20)) != sequence:
            self.wait(sequence=sequence)
        sequence = (sequence+1) & 0xffffffff
        self.bus.write(BASE+0x14, [opcode, arg0, arg1])
        self.bus.write(BASE+0x10, sequence)
        self.wait(sequence=sequence)

    def snapshot(self):
        hardware = int(self.bus.read(BASE+4))
        status = int(self.bus.read(BASE+8))
        if hardware != 1 or status & 4:
            raise SupervisorError(f"camera firmware unhealthy: hardware={hardware:#x}, status={status:#x}")
        values = list(self.bus.read(BASE+0x24, length=7))
        result = dict(exposure_lines=values[0], gain_code=values[1],
                      enabled=bool(values[5]), heartbeat=values[6])
        if not 0xb8 <= values[2] <= 0xc8:
            raise SupervisorError("invalid exposure compensation encoding")
        result.update(metering="percentile_ev", ev_compensation=(values[2]-0xc0)/4,
                      p90=values[3:5])
        return result

    def set_auto_exposure(self, enabled, ev_quarters):
        if type(enabled) is not bool or type(ev_quarters) is not int or not -8 <= ev_quarters <= 8:
            raise ValueError("exposure compensation must be -8..8 quarter stops")
        self.snapshot()
        self.command(6, int(enabled), 0xc0+ev_quarters)
