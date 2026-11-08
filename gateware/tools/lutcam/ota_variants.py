"""Generate A/B identities from one routed Efinity application design.

Only the vendor BRAM updater and bitstream generator run. Synthesis, placement,
and routing are never invoked. The input project and routed artifacts remain
unchanged, and both patched memories are read back with the vendor tool.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

from fpga.litex.lutcam.image_identity import IDENTITY_MEMORY, identity_words


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_memory_dump(text, memory=IDENTITY_MEMORY):
    match = re.search(r'\*{5} Memory "' + re.escape(memory)
                      + r'" \*{5}[^\S\n]*\n([01]+)[^\S\n]*$', text, re.MULTILINE)
    if match is None or len(match[1]) != 512:
        raise ValueError("vendor BRAM readback did not contain the 512-bit identity table")
    # Efinity prints each logical word starting with its least significant bit.
    return [int(match[1][i:i + 32][::-1], 2) for i in range(0, 512, 32)]


def _project_inputs(project):
    root = ET.parse(project).getroot()
    namespace = {"e": "http://www.efinixinc.com/enf_proj"}
    design = root.attrib["name"]
    directory = project.parent
    outflow = directory / "outflow"
    device_info = root.find("e:device_info", namespace)
    family = device_info.find("e:family", namespace).attrib["name"]
    device = device_info.find("e:device", namespace).attrib["name"]
    bitstream = root.find("e:bitstream_generation", namespace)
    options = {element.attrib["name"]: element.attrib["value"] for element in bitstream}
    # This command emits ordinary configuration files. Secured/encrypted builds
    # require the vendor security stage and are deliberately not silently downgraded.
    security = root.find("e:security", namespace)
    if security is not None and len(security):
        raise ValueError("OTA variants currently require an unsecured Efinity project")
    if options.get("enable_roms") != "on" or options.get("enable_crc_check") != "on":
        raise ValueError("project must enable ROM initialization and configuration CRC checking")
    inputs = {
        "project": project,
        "memory_info": outflow / f"{design}.raminfo.pb",
        "placement": outflow / f"{design}.place",
        "logical_bitstream": directory / "work_pnr" / f"{design}.lbf",
        "periphery": outflow / f"{design}.lpf",
        "interface_settings": outflow / f"{design}_or.ini",
    }
    for path in inputs.values():
        if not path.is_file():
            raise ValueError(f"missing compiled project artifact: {path}")
    return design, family, device, options, inputs


def generate_variants(project, build_id, output_dir, *, efinity=None):
    """Patch two separate LBFs; generate and verify both A/B configuration files."""
    from litex.build.efinix.toolchain import find_efinity_path, load_efinity_env

    project = Path(project).resolve()
    output_dir = Path(output_dir).resolve()
    words = {slot: identity_words(build_id, slot) for slot in ("A", "B")}
    build_id = build_id.hex() if isinstance(build_id, bytes) else build_id.lower()
    if output_dir == project.parent or project.parent in output_dir.parents:
        raise ValueError("variant output must be outside the compiled gateware directory")
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError("variant output directory must be new or empty")
    _, family, device, options, inputs = _project_inputs(project)
    efinity = Path(efinity or find_efinity_path()).resolve()
    env = load_efinity_env(str(efinity))
    output_dir.mkdir(parents=True, exist_ok=True)
    # These include the original routed LBF and every routing database file.
    # No tool receives any original artifact as an output destination.
    protected = set(inputs.values()) | set((project.parent / "work_pnr").glob("*"))
    protected |= {path for suffix in ("vdb", "map.v", "timing.rpt", "timing-check.json")
                  if (path := project.parent / "outflow" / f"{project.stem}.{suffix}").is_file()}
    before = {str(path): sha256(path) for path in sorted(protected) if path.is_file()}
    commands = []

    def run(command, logfile):
        commands.append([str(part) for part in command])
        result = subprocess.run(commands[-1], cwd=output_dir, env=env, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        (output_dir / logfile).write_text(result.stdout)
        if result.returncode or re.search(r"(?m)^ERROR\s*:", result.stdout):
            raise RuntimeError(f"vendor command failed; see {output_dir / logfile}")
        return result.stdout

    update_base = [efinity / "bin" / "efx_bram_update",
        "--mem_info", inputs["memory_info"], "--place", inputs["placement"],
        "--family", family]
    report = {
        "schema": 1, "build_id": build_id, "memory": IDENTITY_MEMORY,
        "pnr_runs_for_variants": 0, "identity_bit_differences": 1,
        "project": str(project), "efinity": str(efinity), "images": {},
    }
    try:
        for slot in ("A", "B"):
            init = output_dir / f"{slot}.identity.init"
            init.write_text("".join(f"{word:08x}\n" for word in words[slot]))
            lbf = output_dir / f"{slot}.lbf"
            memory_arg = f"{IDENTITY_MEMORY},{init}"
            run(update_base + ["--lbf", inputs["logical_bitstream"],
                "--output", lbf, "--memory", memory_arg, "--mode", "update"],
                f"{slot}.bram-update.log")
            readback = run(update_base + ["--lbf", lbf, "--memory", memory_arg,
                "--mode", "read"], f"{slot}.bram-readback.log")
            if read_memory_dump(readback) != words[slot]:
                raise RuntimeError(f"slot {slot} logical identity readback mismatch")
            hex_file = output_dir / f"{slot}.hex"
            bit_file = output_dir / f"{slot}.bit"
            # Match Efinity's own pgm flow, retaining every project bitstream
            # option except post-generation binary exports handled separately.
            bitgen = [efinity / "bin" / "efx_pgm", "--interface_designer_settings",
                inputs["interface_settings"], "--periph", inputs["periphery"],
                "--family", family, "--device", device, "--source", lbf,
                "--dest", hex_file]
            exports = {"generate_bit", "generate_hex", "generate_bitbin", "generate_hexbin"}
            bitgen += [f"--{name}={value}" for name, value in options.items()
                       if name not in exports]
            run(bitgen, f"{slot}.bitgen.log")
            if not hex_file.is_file() or not bit_file.is_file():
                raise RuntimeError(f"vendor bitstream generator did not emit slot {slot} files")
            report["images"][slot] = {
                "slot": 1 if slot == "A" else 2, "identity_readback_verified": True,
                "hex": {"path": str(hex_file), "sha256": sha256(hex_file)},
                "bit": {"path": str(bit_file), "sha256": sha256(bit_file)},
                "lbf_sha256": sha256(lbf), "identity_init_sha256": sha256(init),
            }
        if report["images"]["A"]["hex"]["sha256"] == report["images"]["B"]["hex"]["sha256"]:
            raise RuntimeError("A/B bitstreams are unexpectedly identical")
    finally:
        after = {path: sha256(path) for path in before}
        if before != after:
            raise RuntimeError("a protected synthesis/routing artifact changed during variant generation")
    report["routed_artifacts_unchanged"] = True
    report["routed_artifact_sha256"] = before
    report["commands"] = commands
    (output_dir / "variants.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", required=True, type=Path)
    parser.add_argument("--build-id", required=True, help="128-bit build ID as 32 hex digits")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--efinity", type=Path)
    args = parser.parse_args()
    generate_variants(args.project, args.build_id, args.output_dir, efinity=args.efinity)
    print(f"Generated A/B from one routed design: {args.output_dir / 'variants.json'}")


if __name__ == "__main__":
    main()
