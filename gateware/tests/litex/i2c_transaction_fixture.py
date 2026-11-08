"""Shared finite wire trace generated from the current fabric implementation."""
from functools import lru_cache
from pathlib import Path
import shutil
import sys
import tempfile

ROOT = next(p for p in Path(__file__).resolve().parents
            if (p / "fpga/litex/lutcam/i2c.py").exists())
sys.path.insert(0, str(ROOT / "fpga/litex"))
from lutcam.i2c import SharedCameraI2C
from i2c_transaction_vectors import make_trace

# Keep the trace once per process. It contains independent pin-target/CSR
# assertions and no historical source or expected-output fixture.
@lru_cache(maxsize=1)
def transaction_trace():
    if not shutil.which("verilator"):
        import pytest
        pytest.skip("Verilator is required for emitted I2C transaction replay")
    with tempfile.TemporaryDirectory(prefix="i2c-current-trace-") as directory:
        rows, summary = make_trace(SharedCameraI2C, Path(directory))
    assert summary["seed_values"] == [0x7ffffffe, 0xfffffffe]
    return rows
