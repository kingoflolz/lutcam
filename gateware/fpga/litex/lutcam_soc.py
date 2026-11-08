#!/usr/bin/env python3
"""Generate or build the Ti60F225 LUTCAM board target; never program hardware."""

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

from litex.soc.integration.builder import Builder
from lutcam.soc import LutcamSoC
from lutcam.identifier import IDENTIFIER_ROM_LOGIC_BITS


# Preserve the synthesis policy used to establish the existing Ti60 camera
# resource budget. LiteX defaults to speed optimization, whose narrower RAM
# decomposition alone costs seven additional RAM10 blocks per camera.
EFINITY_MAP_PARAMS = {
    "mode": ["area", "e_option"],
    "allow-const-ram-index": ["0", "e_option"],
    "blackbox-error": ["1", "e_option"],
    "blast_const_operand_adders": ["1", "e_option"],
    "bram_output_regs_packing": ["1", "e_option"],
    "bram-push-tco-outreg": ["0", "e_option"],
    "create-onehot-fsms": ["0", "e_option"],
    "fanout-limit": ["0", "e_integer"],
    "hdl-compile-unit": ["1", "e_option"],
    "hdl-loop-limit": ["20000", "e_integer"],
    "infer-clk-enable": ["3", "e_option"],
    "infer-sync-set-reset": ["1", "e_option"],
    "max_ram": ["-1", "e_integer"],
    "max_mult": ["-1", "e_integer"],
    # Permit the explicitly logic-mapped 49-byte identifier ROM. This is a
    # bounded allowance, not the automatic small-memory mapping threshold.
    "max-bit-blast-mem-size": [str(IDENTIFIER_ROM_LOGIC_BITS), "e_integer"],
    "min-sr-fanout": ["0", "e_integer"],
    "min-ce-fanout": ["0", "e_integer"],
    "mult-auto-pipeline": ["0", "e_integer"],
    "mult-decomp-retime": ["1", "e_option"],
    "operator-sharing": ["0", "e_option"],
    "optimize-adder-tree": ["1", "e_option"],
    "optimize-zero-init-rom": ["1", "e_option"],
    "ram-decomp-mode": ["1", "e_option"],
    "retiming": ["0", "e_option"],
    "seq_opt": ["1", "e_option"],
    "seq-opt-sync-only": ["0", "e_option"],
    # The complete SoC also has tiny protocol descriptor/crossing FIFOs. A
    # zero threshold wastes full RAM10 blocks on these 8--228-bit memories.
    "use-logic-for-small-mem": ["256", "e_integer"],
    "use-logic-for-small-rom": ["64", "e_integer"],
    "dsp-input-regs-packing": ["1", "e_option"],
    "dsp-output-regs-packing": ["1", "e_option"],
    "dsp-mac-packing": ["1", "e_option"],
    "insert-carry-skip": ["0", "e_option"],
    # Hardware A/B testing with Efinity 2025.2 found COMB4 packing stalls
    # JTAGBone and drives continuous malformed Ethernet traffic.
    "pack-luts-to-comb4": ["0", "e_option"],
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true", help="Run Efinity synthesis/place/route/bitstream.")
    parser.add_argument("--output-dir", default="build/lutcam")
    parser.add_argument("--ip-address", default="192.168.1.50")
    parser.add_argument("--mac-address", type=lambda value: int(value.replace(":", ""), 16),
                        default=None, help="Override the default MAC derived from the flash UID.")
    parser.add_argument("--without-usb", action="store_true", help="Diagnostic build without USB host.")
    parser.add_argument("--golden", action="store_true",
                        help="Build minimal Ethernet/USB recovery with automatic A/B CRC selection.")
    parser.add_argument("--golden-maintenance", action="store_true",
                        help="Build a temporary application that can write only golden flash; A/B remain read-only.")
    parser.add_argument("--build-id", help="128-bit release identity (32 hex digits); defaults to source hash.")
    parser.add_argument("--efinity", help="Efinity installation directory.")
    parser.add_argument("--timing-model", choices=("C3",), default="C3",
                        help="Required Efinity timing model (always C3).")
    args = parser.parse_args()
    if args.golden and args.golden_maintenance:
        parser.error("--golden-maintenance requires an application build, without --golden")
    if args.efinity:
        os.environ["LITEX_ENV_EFINITY"] = str(Path(args.efinity).resolve())
    output = Path(args.output_dir).resolve()
    firmware = None
    if not args.golden:
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
        from firmware.build import build as build_firmware
        firmware = build_firmware(output / "firmware")
    build_id = args.build_id
    if build_id is None:
        root = Path(__file__).resolve().parents[2]
        digest = hashlib.sha256()
        for directory in (root / "fpga", root / "rtl", root / "firmware", root / "vendor/picorv32", root / "vendor/luna/luna",
                          root / "vendor/liteeth/liteeth", root / "vendor/litex_luna/litex_luna"):
            for path in sorted(directory.rglob("*")):
                if path.is_file() and path.suffix in (".py", ".v", ".sv", ".svh", ".hex", ".mem", ".c", ".h", ".S", ".ld"):
                    digest.update(str(path.relative_to(root)).encode() + b"\0" + path.read_bytes())
        for path in (root / "pyproject.toml", root / "uv.lock", root / "vendor/luna/pyproject.toml",
                     root / "vendor/liteeth/pyproject.toml", root / "vendor/liteeth/setup.py",
                     root / "vendor/litex_luna/pyproject.toml"):
            digest.update(str(path.relative_to(root)).encode() + b"\0" + path.read_bytes())
        if firmware:
            digest.update((output / "firmware/firmware.flash.bin").read_bytes())
        build_id = digest.hexdigest()[:32]
    if len(build_id) != 32 or len(bytes.fromhex(build_id)) != 16:
        parser.error("--build-id must contain exactly 32 hexadecimal digits")
    soc = LutcamSoC(ip_address=args.ip_address, mac_address=args.mac_address,
                    with_usb=not args.without_usb, timing_model=args.timing_model,
                    golden=args.golden, build_id=build_id,
                    golden_maintenance=args.golden_maintenance)
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    (output / "ota-build.json").write_text(json.dumps({
        "build_id": build_id.lower(), "golden": args.golden,
        "usb": not args.without_usb, "device": "Ti60F225",
        "flash_interface": "page-record",
        "mac_address": args.mac_address,
        "mac_source": "flash-uid" if args.mac_address is None else "override",
        "golden_maintenance": args.golden_maintenance,
        "threshold_frontend": None if args.golden else "adaptive",
        "firmware": firmware,
    }, indent=2) + "\n")
    builder = Builder(soc, output_dir=str(output), compile_software=False,
        csr_csv=str(output / "csr.csv"), csr_json=str(output / "csr.json"))
    map_params = dict(EFINITY_MAP_PARAMS)
    if not args.golden:
        # Whole-board C3 qualification saves area without decomposed-multiply
        # retiming. Keep the recovery image's established synthesis policy.
        map_params["mult-decomp-retime"] = ["0", "e_option"]
    builder.build(run=args.build, build_name="lutcam",
        efx_map_params=map_params,
        # AN033 Table 11 recommends DIV2 / negative-edge capture for internal
        # active-SPI configuration. Keep compression and configuration CRC on.
        efx_pgm_params={"oscillator_clock_divider": ["DIV2", "e_option"],
                        "active_capture_clk_edge": ["negedge", "e_option"]},
        efx_pnr_params={"seed": ["1", "e_integer"],
                        "placer_effort_level": ["3", "e_option"],
                        "beneficial_skew": ["on", "e_option"],
                        "optimization_level": ["TIMING_3", "e_option"]})


if __name__ == "__main__":
    main()
