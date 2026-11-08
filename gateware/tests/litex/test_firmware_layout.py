"""The final relaxed RISC-V layout must retain the checked stack reservation."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
from firmware.build import RAM_SIZE, STACK_SIZE


class FirmwareLayoutTests(unittest.TestCase):
    def link(self, bss_bytes, mailbox_bytes=56):
        linker=os.environ.get('LD_LLD') or shutil.which('ld.lld')
        if not linker:
            linker=str(next((Path.home()/'.rustup/toolchains').glob('stable-*/lib/rustlib/*/bin/rust-lld')))
        with tempfile.TemporaryDirectory(prefix='lutcam-stack-') as tmp:
            root=Path(tmp)
            source=root/'start.S'
            source.write_text(f'''.section .text.start,"ax",@progbits
.global _start
_start:
    la a0, reserved
    ret
.section .bss,"aw",@nobits
reserved:
    .zero {bss_bytes}
.section .mailbox,"aw",@progbits
    .zero {mailbox_bytes}
''')
            subprocess.run([os.environ.get('CLANG','clang'),'--target=riscv32-unknown-elf',
                            '-march=rv32ec','-mabi=ilp32e','-c',str(source),'-o',str(root/'start.o')],check=True)
            flavor=['-flavor','gnu'] if Path(linker).name=='rust-lld' else []
            return subprocess.run([linker,*flavor,'-m','elf32lriscv','--gc-sections',
                                   '-T',str(ROOT/'firmware/link.ld'),str(root/'start.o'),
                                   '-o',str(root/'firmware.elf')],capture_output=True,text=True)

    def test_layout_with_enough_stack_links(self):
        result=self.link(RAM_SIZE-STACK_SIZE-84)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_bss_cannot_overlap_mailbox_or_stack(self):
        result=self.link(RAM_SIZE-STACK_SIZE)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('overlap',result.stderr)
        self.assertIn('.mailbox',result.stderr)

    def test_mailbox_cannot_overlap_reserved_stack(self):
        result=self.link(4,60)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('overlap',result.stderr)
        self.assertIn('.stack',result.stderr)


if __name__=='__main__': unittest.main()
