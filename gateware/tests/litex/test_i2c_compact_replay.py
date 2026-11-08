"""Current default versus compact record state, physical registered decoder."""
import pytest
from i2c_transaction_fixture import SharedCameraI2C, transaction_trace
from i2c_transaction_replay import run_replay

@pytest.mark.parametrize("emitter", ["migen", "litex"])
def test_compact_record_preserves_transactions(tmp_path, emitter):
    report = run_replay([
        ("reference", SharedCameraI2C, {}),
        ("compact", SharedCameraI2C, {"compact_record": True}),
    ], transaction_trace(), tmp_path, emitter=emitter)
    assert report["status"] == "PASS"
