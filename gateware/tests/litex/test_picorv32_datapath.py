"""Exercise the specialized CPU with independent arithmetic/branch results."""
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class PicoDatapathTests(unittest.TestCase):
    def test_arithmetic_comparisons_shifts_and_stalled_memory(self):
        if not shutil.which("iverilog") or not shutil.which("vvp"):
            self.skipTest("Icarus Verilog required")
        code, expected, expected_strobes, expected_offsets = [], [], [], []

        def immediate(rd, rs, value):
            return ((value & 4095) << 20) | (rs << 15) | (rd << 7) | 0x13

        def load(rd, value):
            code.extend([(((value + 0x800) & 0xfffff000) | rd << 7 | 0x37),
                         immediate(rd, rd, value)])

        def store(value, size=2, offset=0):
            code.append((12 << 20) | (5 << 15) | (size << 12) | (offset << 7) | 0x23)
            expected.append(value & 0xffffffff)
            expected_strobes.append(((1 << (1 << size))-1) << offset)
            expected_offsets.append(offset & ~3)

        def branch(funct, rs1, rs2, delta):
            imm = delta & 8191
            return (((imm >> 12) << 31) | (((imm >> 5) & 63) << 25) |
                    (rs2 << 20) | (rs1 << 15) | (funct << 12) |
                    (((imm >> 1) & 15) << 8) | (((imm >> 11) & 1) << 7) | 0x63)

        def signed(n):
            return n if n < 0x80000000 else n - 0x100000000

        load(5, 0x20000000)
        code.append(immediate(12, 3, 0)); store(2032)
        code.append(immediate(12, 4, 0)); store(0)
        # LLVM may use ra as a full-width temporary; only SP is narrowed.
        for register in (1, 8, 9, 13, 14, 15):
            load(register, 0xdeadbeef)
            code.append(immediate(12, register, 0)); store(0xdeadbeef)
        code.append(immediate(2, 2, -100))
        code.append(immediate(12, 2, 0)); store(3996)
        code.append(immediate(2, 2, 100))
        code.append(immediate(12, 2, 0)); store(4096)
        load(12, 0x12345678)
        for offset in range(4): store(0x78787878, size=0, offset=offset)
        for offset in (0, 2): store(0x56785678, size=1, offset=offset)
        pairs = [(0, 0), (0xffffffff, 1), (0x80000000, 0x7fffffff),
                 (0x7fffffff, 0x80000000), (0xffffffff, 0xffffffff),
                 (1, 31), (0x80000000, 0), (0x80000000, 31)]
        rng = random.Random(0x4c555443)
        pairs += [(rng.getrandbits(32), rng.getrandbits(32)) for _ in range(10)]
        for a, b in pairs:
            load(10, a); load(11, b)
            results = ((0, 0, a+b), (0, 32, a-b), (2, 0, signed(a)<signed(b)),
                       (3, 0, a<b), (4, 0, a^b), (6, 0, a|b), (7, 0, a&b),
                       (1, 0, a << (b & 31)), (5, 0, a >> (b & 31)),
                       (5, 32, signed(a) >> (b & 31)))
            for funct, high, result in results:
                code.append(high << 25 | 11 << 20 | 10 << 15 | funct << 12 | 12 << 7 | 0x33)
                store(result)
            for funct, result in ((0, a==b), (1, a!=b), (4, signed(a)<signed(b)),
                                  (5, signed(a)>=signed(b)), (6, a<b), (7, a>=b)):
                code.extend([immediate(12, 0, 1), branch(funct, 10, 11, 8), immediate(12, 0, 0)])
                store(result)
        code.extend([immediate(12, 0, 3), immediate(12, 12, -1), branch(1, 12, 0, -4)])
        store(0)
        code.append(0x00100073)  # EBREAK: all prior writes must be complete.
        self.assertLessEqual(len(code), 1024)
        with tempfile.TemporaryDirectory(prefix="lutcam-cpu-datapath-") as directory:
            d = Path(directory)
            (d / "code.hex").write_text("".join(f"{v:08x}\n" for v in code))
            (d / "expected.hex").write_text("".join(f"{v:08x}\n" for v in expected))
            (d / "strobes.hex").write_text("".join(f"{v:x}\n" for v in expected_strobes))
            (d / "offsets.hex").write_text("".join(f"{v:08x}\n" for v in expected_offsets))
            (d / "tb.v").write_text(f'''module tb;
reg clk=0, resetn=0;
always #5 clk=~clk;
wire valid, instr, trap;
wire [31:0] address, wdata;
wire [3:0] strobes;
reg ready=0;
reg [31:0] rdata=0;
reg [31:0] code[0:{len(code)-1}], expected[0:{len(expected)-1}];
reg [3:0] expected_strobes[0:{len(expected)-1}];
reg [31:0] expected_offsets[0:{len(expected)-1}];
reg stalled=0;
reg [31:0] held_address, held_wdata;
reg [3:0] held_strobes;
integer count=0, cycles=0, delay=0;
picorv32 #(.ENABLE_COUNTERS(0), .ENABLE_COUNTERS64(0), .ENABLE_REGS_16_31(0),
 .ENABLE_REGS_DUALPORT(0), .COMPRESSED_ISA(1), .TWO_STAGE_SHIFT(0),
 .PROGADDR_WIDTH(12), .STACKADDR_WIDTH(13), .FIXED_GP(2032), .ZERO_REGS(32'hd1), .STACKADDR(4096)) cpu (
 .clk(clk), .resetn(resetn), .trap(trap), .mem_valid(valid), .mem_instr(instr),
 .mem_ready(ready), .mem_addr(address), .mem_wdata(wdata), .mem_wstrb(strobes), .mem_rdata(rdata),
 .irq(0), .pcpi_wr(1'b0), .pcpi_rd(0), .pcpi_wait(1'b0), .pcpi_ready(1'b0));
always @(posedge clk) begin
 cycles<=cycles+1;
 ready<=0;
 if(stalled && (!valid || address!==held_address || strobes!==held_strobes ||
                (held_strobes!=0 && wdata!==held_wdata)))
  $fatal(1,"memory request changed under backpressure");
 stalled<=valid && !ready;
 held_address<=address; held_wdata<=wdata; held_strobes<=strobes;
 if(valid && !ready) begin
  if(delay==3) begin
   ready<=1; delay<=0;
   if(strobes==0) begin
    if(address>=4*{len(code)}) $fatal(1,"invalid instruction address %h",address);
    rdata<=code[address>>2];
   end
  end else delay<=delay+1;
 end
 if(valid && ready && strobes!=0) begin
  if(count>={len(expected)} || strobes!==expected_strobes[count] || address!=32'h20000000+expected_offsets[count] || wdata!==expected[count])
   $fatal(1,"result %0d: got %h expected %h",count,wdata,expected[count]);
  count<=count+1;
 end
 if(trap) begin
  if(count!={len(expected)}) $fatal(1,"truncated instruction run");
  $display("PASS %0d arithmetic and branch results",count); $finish;
 end
 if(cycles>200000) $fatal(1,"CPU timeout");
end
initial begin
 $readmemh("code.hex",code); $readmemh("expected.hex",expected);
 $readmemh("strobes.hex",expected_strobes); $readmemh("offsets.hex",expected_offsets);
 repeat(5) @(negedge clk); resetn=1;
end
endmodule
''')
            result = subprocess.run(["iverilog", "-g2012", "-s", "tb", "-o", "run", "tb.v",
                                     str(ROOT / "vendor/picorv32/picorv32.v")], cwd=d,
                                    capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            result = subprocess.run(["vvp", "run"], cwd=d, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            self.assertIn("PASS", result.stdout)
