"""Current fabric versus native state/watchdog DSPs through complete I2C."""
import os
from pathlib import Path
import pytest
from i2c_transaction_fixture import SharedCameraI2C, transaction_trace
from i2c_transaction_replay import run_replay

@pytest.mark.parametrize("emitter", ["migen", "litex"])
def test_native_i2c_state_and_watchdog_preserve_transactions(tmp_path, emitter):
    root = os.environ.get("LITEX_ENV_EFINITY")
    if not root:
        pytest.skip("Set LITEX_ENV_EFINITY to validate the native DSP model")
    model = Path(root) / "sim_models/verilog/efx_dsp48.v"
    assert model.is_file(), model
    report = run_replay([
        ("reference", SharedCameraI2C, {}),
        ("native", SharedCameraI2C, {"compact_record": True,
         "use_dsp_state": True, "use_dsp_watchdog": True}),
    ], transaction_trace(), tmp_path, emitter=emitter, vendor=model)
    assert report["status"] == "PASS"
