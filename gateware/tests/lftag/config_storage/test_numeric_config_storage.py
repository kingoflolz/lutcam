"""Check fabric configuration storage against the original register contract.

This deliberately extracts the storage block from the RTL being tested: changing
its reset or enable changes this test's DUT. The reference is the prior
nested controller behavior, not a copy of the new combined CE expression.

CONFIG_STORAGE_SOURCE overrides rtl/lftag for artifact qualification. Both
normal and synthesis-mode compilation must work without a vendor DSP model.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = next(p for p in Path(__file__).resolve().parents if (p / "rtl/lftag").is_dir())
SOURCE = Path(os.environ.get("CONFIG_STORAGE_SOURCE", ROOT / "rtl/lftag"))
HERE = Path(__file__).resolve().parent


def storage_slice(filename, register, module):
    source = (SOURCE / filename).read_text()
    enum = re.search(r"    typedef enum logic \[3:0\] \{.*?\} state_t;", source, re.S)
    assert enum, "controller enum was not found"
    names = re.findall(r"\bS_[A-Z_]+\b", enum.group())
    assert names[0] == "S_IDLE"
    if register == "config_work":
        assert names[11:16] == ["S_PACKET", "S_WAIT_PACKET", "S_CENTROIDS",
                                "S_CONFIG_REQUEST", "S_CONFIG_RESPONSE"]
    else:
        assert names[15] == "S_CONFIG_FETCH"
    block = re.search(
        rf"    always_ff @\(posedge clk\) begin : {register}_storage\n.*?\n    end",
        source, re.S,
    )
    assert block, "actual fabric storage block was not found"
    return f"""module {module} #(parameter bit EXTERNAL_CONFIG=1) (
        input logic clk, rst, config_response_valid, config_wait, config_word,
        input logic [3:0] state_bits,
        input logic [2:0] config_index,
        input logic [9:0] cycle_counter,
        input logic [31:0] config_read_data,
        output wire [31:0] value);
{enum.group()}
    state_t state;
    assign state=state_t'(state_bits);
    logic [31:0] {register};
{block.group()}
    assign value={register};
endmodule
"""


@unittest.skipUnless(shutil.which("verilator"), "Verilator required")
class NumericConfigStorageTests(unittest.TestCase):
    def run_variant(self, synthesis):
        with tempfile.TemporaryDirectory(prefix="lftag-config-storage-") as temporary:
            build = Path(temporary)
            slices = build / "storage_slices.sv"
            slices.write_text(
                storage_slice("LfTagCompact3x3DecoderLocalizer.sv", "config_work", "ControllerStorage")
                + storage_slice("LfTagInitialHomographyFused.sv", "current_inverse_focal", "FocalStorage")
            )
            command = [
                "verilator", "--binary", "--timing", "--assert", "-Wno-fatal", "-j", "2",
                "--top-module", "numeric_config_storage_tb", "--Mdir", str(build / "obj"),
                *(["-DSYNTHESIS"] if synthesis else []),
                str(slices), str(HERE / "numeric_config_storage_tb.sv"),
            ]
            result = subprocess.run(command, capture_output=True, text=True, timeout=120)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run(
                [str(build / "obj/Vnumeric_config_storage_tb")],
                capture_output=True, text=True, timeout=60,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS numeric config storage", result.stdout)
            print(("SYNTHESIS " if synthesis else "PORTABLE ") + result.stdout, end="")
            print(json.dumps({
                "synthesis": synthesis,
                "sources": {name: hashlib.sha256((SOURCE / name).read_bytes()).hexdigest()
                            for name in ("LfTagCompact3x3DecoderLocalizer.sv", "LfTagInitialHomographyFused.sv")},
            }, sort_keys=True))

    def test_portable_storage_reset_priority_and_all_control_values(self):
        self.run_variant(False)

    def test_synthesis_storage_every_bit_reset_priority_and_all_control_values(self):
        self.run_variant(True)


if __name__ == "__main__":
    unittest.main()
