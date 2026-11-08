"""Runtime linked exposure for the verified OV9281 120-fps slave mode.

The OV9281 group commands at 0x3208 are write-only. Stage both sensors, end
both groups, then launch each at vertical blank. Sequential SCCB launches do
not promise an atomic transition on the same physical frame across cameras.
HTS, VTS, FSYNC, streaming and the configured manual/gain modes stay unchanged.
"""

from contextlib import nullcontext
import time


LINE_TIME_US = 9.1  # HTS728 / sensor timing clock80 MHz.
MIN_LINES, MAX_LINES = 1, 885  # VTS910 minus the conservative25-line margin.
MIN_GAIN_CODE, MAX_GAIN_CODE = 16, 255  # Gain units are1/16;16 is unity.
DEFAULT_LINES, DEFAULT_GAIN_CODE = 400, 16


class ExposureUpdateError(RuntimeError):
    """A mutating group transaction or its final verification failed."""

    def __init__(self, message, *, cleanup_errors=()):
        super().__init__(message)
        self.uncertain = True
        self.cleanup_errors = tuple(cleanup_errors)


def validate_settings(exposure_lines, gain_code):
    if type(exposure_lines) is not int or not MIN_LINES <= exposure_lines <= MAX_LINES:
        raise ValueError(f"exposure_lines must be an integer in {MIN_LINES}..{MAX_LINES}")
    if type(gain_code) is not int or not MIN_GAIN_CODE <= gain_code <= MAX_GAIN_CODE:
        raise ValueError(f"gain_code must be an integer in {MIN_GAIN_CODE}..{MAX_GAIN_CODE}")


def exposure_state(command_id=0, exposure_lines=DEFAULT_LINES, gain_code=DEFAULT_GAIN_CODE):
    return {"id": command_id, "exposure_lines": exposure_lines, "gain_code": gain_code,
            "line_time_us": LINE_TIME_US, "min_lines": MIN_LINES, "max_lines": MAX_LINES,
            "min_gain_code": MIN_GAIN_CODE, "max_gain_code": MAX_GAIN_CODE}


def apply_linked_exposure(sensors, exposure_lines, gain_code, *, sleep=time.sleep):
    """Apply and verify both sensors, closing any uncertain open group on error.

    Use only from the paired capture owner while its verified mode is running.
    Cancellation may occur between complete SCCB transactions. Cleanup END is
    uncancellable when the adapter provides that context, and never launches an
    incomplete group. There is no documented group-abort command to substitute.
    """
    validate_settings(exposure_lines, gain_code)
    sensors = tuple(sensors)
    if len(sensors) != 2 or [sensor.camera for sensor in sensors] != [0, 1]:
        raise ValueError("linked exposure requires cameras0 and1 in order")
    exposure = (exposure_lines << 4).to_bytes(3, "big")
    gain = bytes((0, gain_code))
    opened = set()
    error = None
    cleanup_errors = []
    try:
        for camera, sensor in enumerate(sensors):
            opened.add(camera)  # A lost reply may follow an applied group open.
            sensor.i2c.write_register(sensor.address, 0x3208, 0x00)
        for sensor in sensors:
            sensor.i2c.write_register(sensor.address, 0x3500, exposure)
            sensor.i2c.write_register(sensor.address, 0x3508, gain)
        for camera, sensor in enumerate(sensors):
            sensor.i2c.write_register(sensor.address, 0x3208, 0x10)
            opened.remove(camera)
        for sensor in sensors:
            sensor.i2c.write_register(sensor.address, 0x3208, 0xa0)
        # 3503 retains a frame of exposure/gain delay; allow at least two of
        # the120.625-fps periods before inspecting the launched values.
        sleep(0.025)
        for sensor in sensors:
            actual_exposure = sensor.read_register(0x3500, 3)
            actual_gain = sensor.read_register(0x3508, 2)
            if actual_exposure != exposure or actual_gain != gain:
                raise RuntimeError(f"camera{sensor.camera} exposure/gain readback mismatch")
    except (Exception, KeyboardInterrupt) as failure:
        error = failure
    finally:
        for camera in sorted(opened):
            sensor = sensors[camera]
            try:
                with getattr(sensor.i2c, "uncancellable", nullcontext)():
                    sensor.i2c.write_register(sensor.address, 0x3208, 0x10)
            except (Exception, KeyboardInterrupt) as failure:
                cleanup_errors.append(f"camera{camera} group END: {type(failure).__name__}: {failure}")
    if error is not None:
        raise ExposureUpdateError(f"{type(error).__name__}: {error}",
                                  cleanup_errors=cleanup_errors) from error
