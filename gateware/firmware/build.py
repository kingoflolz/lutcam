"""Build the flash-loaded RV32EC supervisor using LLVM (no RISC-V SDK needed)."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import zlib

ROOT = Path(__file__).resolve().parents[1]
RAM_SIZE = 4096
STACK_SIZE = 128
FLASH_OFFSET = 0x180000
MAGIC = 0x3256524c  # LRV2
HEADER_SIZE = 16


def stack_usage(elf, usage_file):
    """Conservative call-path bound; firmware has no interrupts or recursion.

    LLVM includes outlined prologue sizes in each caller's .su frame. Direct
    tail calls are conservatively added as if the caller frame remained live.
    Unknown/indirect calls fail the build rather than escaping the budget.
    """
    frames = {"_start": 0}
    for line in Path(usage_file).read_text().splitlines():
        location, size, kind = line.split("\t")
        if kind != "static":
            raise ValueError(f"unbounded firmware stack: {line}")
        frames[location.rsplit(":", 1)[-1]] = int(size)
    disassembly = subprocess.check_output(
        ["llvm-objdump", "-d", "--no-show-raw-insn", str(elf)], text=True)
    graph, current = {}, None
    for line in disassembly.splitlines():
        label = re.match(r"^[0-9a-f]+ <([^>]+)>:", line)
        if label:
            current = label[1] if label[1] in frames else None
            if current:
                graph[current] = set()
            continue
        if current is None:
            continue
        instruction = re.match(r"^\s*[0-9a-f]+:\s+(\S+)\s*(.*)", line)
        if not instruction:
            continue
        opcode, operands = instruction.groups()
        if re.search(r"\b(t1|t2)\b", operands):
            raise ValueError(f"firmware uses a hardware-reserved register: {line}")
        if (re.match(r"sp(?:,|$)", operands) and opcode not in ("sb", "sh", "sw")
                and not (opcode == "addi" and operands.startswith("sp, sp,"))):
            raise ValueError(f"firmware uses SP outside the local stack ABI: {line}")
        if (re.match(r"(?:gp|tp)(?:,|$)", operands)
                and opcode not in ("sb", "sh", "sw", "beq", "bne", "blt", "bge", "bltu", "bgeu")):
            raise ValueError(f"firmware changes a fixed ABI register: {line}")
        if opcode in ("jal", "j", "call", "tail"):
            target = re.search(r"<([^>+]+)(?:\+[^>]*)?>", operands)
            if not target or target[1] not in frames:
                raise ValueError(f"unknown firmware call: {line}")
            if target[1] != current:
                graph[current].add(target[1])
            elif opcode in ("jal", "call"):
                raise ValueError(f"recursive firmware call: {line}")
        elif opcode in ("jalr", "jr"):
            if not (opcode == "jr" and operands == "t0"
                    and current.startswith("OUTLINED_FUNCTION_")):
                raise ValueError(f"indirect firmware call: {line}")
    def depth(name, ancestors=()):
        if name in ancestors:
            raise ValueError(f"recursive firmware call path: {ancestors + (name,)}")
        child_depth, path = max((depth(child, ancestors + (name,))
                                 for child in graph.get(name, ())), default=(0, []))
        return frames[name] + child_depth, [name, *path]
    peak, path = depth("_start")
    if peak + 24 > STACK_SIZE:
        raise ValueError(f"firmware needs {peak} stack bytes plus 24-byte margin; reserved {STACK_SIZE}")
    return {"reserved_bytes": STACK_SIZE, "worst_case_bytes": peak, "path": path}


def build(output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(ROOT))
    from tools.lutcam.ov9281 import board_mode_writes
    mode = board_mode_writes(0, pattern="off", fps=120)
    assert mode == board_mode_writes(1, pattern="off", fps=120)
    # Reuse the current register page. A high bit introduces a full address;
    # low offsets in the same page need only an offset and a value.
    packed, page = [], 0
    for reg, value in mode:
        assert reg < 0x8000
        if reg >> 8 != page or reg & 255 >= 128:
            page = reg >> 8
            packed.append(0x80 | page)
        packed.extend((reg & 255, value))
    (output / "sensor_mode.h").write_text("static const uint8_t sensor_mode[] = {\n" +
        ",".join(f"0x{byte:02x}" for byte in packed) + "\n};\n")
    clang = os.environ.get("CLANG", "clang")
    linker = os.environ.get("LD_LLD") or shutil.which("ld.lld")
    if not linker:
        candidates = sorted((Path.home() / ".rustup/toolchains").glob(
            "stable-*/lib/rustlib/*/bin/rust-lld"))
        if not candidates:
            raise RuntimeError("Install LLVM lld or set LD_LLD to ld.lld / rust-lld")
        linker = str(candidates[0])
    flags = ["--target=riscv32-unknown-elf", "-march=rv32ec", "-mabi=ilp32e", "-ffixed-x6", "-ffixed-x7", "-Oz",
             "-ffreestanding", "-fno-builtin", "-fno-stack-protector", "-fno-unwind-tables",
             "-ffunction-sections", "-fdata-sections", "-fstack-usage",
             "-mllvm", "-apply-ext-tsp-for-size", "-Wall", "-Wextra", "-Werror",
             "-I", str(output)]
    for source in ("start.S", "control.c"):
        subprocess.run([clang, *flags, "-c", str(ROOT / "firmware" / source),
                        "-o", str(output / (source + ".o"))], check=True)
    flavor = ["-flavor", "gnu"] if Path(linker).name == "rust-lld" else []
    subprocess.run([linker, *flavor, "-m", "elf32lriscv", "--gc-sections",
                    "--relax-gp", "-T", str(ROOT / "firmware/link.ld"),
                    "-Map=" + str(output / "firmware.map"),
                    str(output / "start.S.o"), str(output / "control.c.o"),
                    "-o", str(output / "firmware.elf")], check=True)
    subprocess.run(["llvm-objcopy", "-O", "binary", str(output / "firmware.elf"),
                    str(output / "firmware.bin")], check=True)
    code = (output / "firmware.bin").read_bytes()
    code += b"\0" * (-len(code) % 4)
    if len(code) > RAM_SIZE - STACK_SIZE:
        raise ValueError("firmware does not leave the required stack space")
    stack = stack_usage(output / "firmware.elf", output / "control.c.su")
    symbols = {name: int(address, 16) for address, _, name in
               (line.split() for line in subprocess.check_output(
                   ["llvm-nm", str(output / "firmware.elf")], text=True).splitlines())}
    if len(code) != symbols["_bss_end"]:
        raise ValueError("flash image must initialize all data, including BSS")
    (output / "firmware.bin").write_bytes(code)
    crc = zlib.crc32(code)
    image = struct.pack("<4I", MAGIC, len(code), crc, RAM_SIZE) + code
    (output / "firmware.flash.bin").write_bytes(image)
    metadata = dict(isa="rv32ec", ram_bytes=RAM_SIZE, image_bytes=len(code),
                    flash_offset=FLASH_OFFSET, crc32=crc,
                    sha256=hashlib.sha256(image).hexdigest(), stack=stack,
                    zero_data_bytes=symbols["_bss_end"]-symbols["_bss_start"],
                    free_bytes=RAM_SIZE-len(code)-STACK_SIZE)
    (output / "firmware.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "build/firmware")
    print(json.dumps(build(parser.parse_args().output_dir), indent=2))
