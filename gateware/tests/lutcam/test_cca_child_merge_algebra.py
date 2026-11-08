"""Exhaustive binary child-count/ID algebra using actual stage source.

The oracle models count addition or an ordered pair of optional child IDs.
The generated subject extracts the real RTL functions and output expressions;
it does not duplicate their optimized implementation in a Python surrogate.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

import pytest


ROOT = next(p for p in Path(__file__).resolve().parents
            if (p / "rtl/lrcca/TwoPPCS5Stage.sv").is_file())
SOURCE = Path(os.environ.get("LUTCAM_CCA_S5_SOURCE",
                             ROOT / "rtl/lrcca/TwoPPCS5Stage.sv"))
REFERENCE = os.environ.get("LUTCAM_CCA_S5_REFERENCE")


def extract(source, module):
    text = source.read_text()
    functions = []
    for name in ("feature_count_of", "merged_child_count_from_counts"):
        match = re.search(r"  function automatic logic \[CHILD_WIDTH-1:0\] "
                          + name + r"\(.*?  endfunction", text, re.S)
        assert match, name
        functions.append(match.group())
    touch = re.search(r"      touch_count = merge_prev \?.*?;", text, re.S)
    pair = re.search(r"      child_merge_id_a_local = touch_count;\s*"
                     r"child_merge_id_b_local =.*?;", text, re.S)
    valid = re.search(r"      if \(d\) begin\s*if \(o\) begin\s*"
                      r"if \((.*?)\) begin\s*"
                      r"result.child_merge_valid = 1'b1;", text, re.S)
    assert touch and pair and valid
    # Keep the real null-code and storage-width definitions bound to the test.
    assert re.search(r"CHILD_WIDTH = \(CHILD_COUNT_BITS > 0\) \? "
                     r"CHILD_COUNT_BITS : 1", text)
    assert re.search(r"CHILD_ID_NULL =\s*CHILD_ID_MODE \? "
                     r"\{CHILD_WIDTH\{1'b1\}\} : '0", text)
    return f"""
module {module} #(
    parameter integer CHILD_COUNT_BITS=1,
    parameter bit CHILD_ID_MODE=1,
    localparam integer CHILD_WIDTH=(CHILD_COUNT_BITS>0)?CHILD_COUNT_BITS:1,
    localparam integer DATA_WIDTH=CHILD_WIDTH,
    localparam integer OFF_CHILD=0,
    localparam logic [CHILD_WIDTH-1:0] CHILD_ID_NULL=
        CHILD_ID_MODE?{{CHILD_WIDTH{{1'b1}}}}:'0
) (
    input logic [CHILD_WIDTH-1:0] left_feature,right_feature,
    input logic merge_prev,d,o,
    output logic [CHILD_WIDTH-1:0] merged,touch_count,
    output logic [CHILD_WIDTH-1:0] child_merge_id_a_local,child_merge_id_b_local,
    output logic valid
);
logic [CHILD_WIDTH-1:0] add_pix_count,prev_count;
{chr(10).join(functions)}
always_comb begin
    add_pix_count=feature_count_of(left_feature);
    prev_count=feature_count_of(right_feature);
    merged=merged_child_count_from_counts(add_pix_count,prev_count);
{touch.group()}
{pair.group()}
    valid=1'b0;
    if(d) if(o) if({valid.group(1)}) valid=1'b1;
end
endmodule
"""


def bench(include_reference):
    compare_reference = ""
    reference = ""
    if include_reference:
        reference = """
wire [W-1:0] ref_merged,ref_touch,ref_a,ref_b;wire ref_valid;
reference #(.CHILD_COUNT_BITS(COUNT),.CHILD_ID_MODE(ID_MODE)) ref_dut(
    .left_feature,.right_feature,.merge_prev,.d,.o,
    .merged(ref_merged),.touch_count(ref_touch),
    .child_merge_id_a_local(ref_a),.child_merge_id_b_local(ref_b),.valid(ref_valid));
"""
        compare_reference = """
if ({merged,touch,a,b,valid} !== {ref_merged,ref_touch,ref_a,ref_b,ref_valid})
    $fatal(1,"historical parity COUNT%0d ID%0d L%0d R%0d merge%0d d%0d o%0d",
        COUNT,ID_MODE,left,right,merge_prev,d,o);
"""
    template = r"""
module check_case #(
    parameter integer COUNT=1,
    parameter bit ID_MODE=1,
    localparam integer W=(COUNT>0)?COUNT:1,
    localparam integer MASK=(1<<W)-1
)(output logic done=0,output longint unsigned checked=0);
logic [W-1:0] left_feature=0,right_feature=0;
logic merge_prev=0,d=0,o=0;
wire [W-1:0] merged,touch,a,b;wire valid;
subject #(.CHILD_COUNT_BITS(COUNT),.CHILD_ID_MODE(ID_MODE)) dut(
    .left_feature,.right_feature,.merge_prev,.d,.o,
    .merged,.touch_count(touch),
    .child_merge_id_a_local(a),.child_merge_id_b_local(b),.valid);
@REFERENCE@
integer left,right,controls,l,r,null_id,representative,other,distinct;
integer expected_merged,expected_touch,expected_b;
logic expected_valid;
initial begin
    for(left=0;left<=MASK;left=left+1)
      for(right=0;right<=MASK;right=right+1)
        for(controls=0;controls<8;controls=controls+1) begin
            left_feature=W'(left);right_feature=W'(right);
            {merge_prev,d,o}=3'(controls);
            // The feature extractor has no child field when COUNT is zero.
            l=(COUNT==0)?0:left;r=(COUNT==0)?0:right;
            null_id=ID_MODE?MASK:0;
            // Ordered optional-ID model, independent of the RTL mux identity.
            if(l==null_id) begin representative=r;other=l;end
            else if(r==null_id) begin representative=l;other=r;end
            else if(l<=r) begin representative=l;other=r;end
            else begin representative=r;other=l;end
            distinct=0;
            if(l!=null_id) distinct=distinct+1;
            if(r!=null_id && r!=l) distinct=distinct+1;
            expected_merged=(COUNT==0)?0:
                (ID_MODE?representative:((l+r)&MASK));
            expected_touch=merge_prev?expected_merged:l;
            if(ID_MODE && COUNT>0 && merge_prev) expected_b=other;
            else expected_b=(expected_touch==l)?r:l;
            expected_valid=ID_MODE && merge_prev && d && o && (distinct==2);
            #1;
            if(merged!==W'(expected_merged) || touch!==W'(expected_touch) ||
               a!==W'(expected_touch) || b!==W'(expected_b) || valid!==expected_valid)
                $fatal(1,"independent oracle COUNT%0d ID%0d L%0d R%0d merge%0d d%0d o%0d got%0d,%0d,%0d,%0d,%0d expected%0d,%0d,%0d,%0d",
                    COUNT,ID_MODE,left,right,merge_prev,d,o,merged,touch,a,b,valid,
                    expected_merged,expected_touch,expected_b,expected_valid);
            @COMPARE_REFERENCE@
            checked=checked+1;
        end
    done=1;
end
endmodule
module tb;
wire [21:0] done;
wire [63:0] checked[0:21];
for(genvar width=0;width<=10;width=width+1) begin:g_width
    for(genvar mode=0;mode<2;mode=mode+1) begin:g_mode
        check_case #(.COUNT(width),.ID_MODE(mode!=0)) test_case(
            .done(done[width*2+mode]),.checked(checked[width*2+mode]));
    end
end
longint unsigned total;
initial begin
    wait(&done);#1;total=0;
    for(integer i=0;i<22;i=i+1)total=total+checked[i];
    $display("PASS source-extracted child algebra cases=22 vectors=%0d",total);
    $finish;
end
initial #9000000 $fatal(1,"exhaustive proof timeout");
endmodule
"""
    return template.replace("@REFERENCE@", reference).replace(
        "@COMPARE_REFERENCE@", compare_reference)


@pytest.mark.skipif(shutil.which("verilator") is None, reason="Verilator required")
def test_child_merge_exact_algebra_all_binary_widths(tmp_path):
    sources = [tmp_path / "subject.sv"]
    sources[0].write_text(extract(SOURCE, "subject"))
    if REFERENCE:
        reference = tmp_path / "reference.sv"
        reference.write_text(extract(Path(REFERENCE), "reference"))
        sources.append(reference)
    tb = tmp_path / "tb.sv"
    tb.write_text(bench(bool(REFERENCE)))
    command = ["verilator", "--binary", "--timing", "-Wno-fatal",
               "--top-module", "tb", "--Mdir", str(tmp_path / "obj"),
               "-j", "2", str(tb), *(str(p) for p in sources)]
    compiled = subprocess.run(command, capture_output=True, text=True)
    (tmp_path / "compile.log").write_text(compiled.stdout + compiled.stderr)
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr
    result = subprocess.run([str(tmp_path / "obj/Vtb")], capture_output=True, text=True)
    (tmp_path / "run.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS source-extracted child algebra" in result.stdout
    report = dict(status="PASS", source=str(SOURCE),
                  source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                  reference=REFERENCE, output=result.stdout,
                  generated_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in [*sources, tb]})
    if REFERENCE:
        report["reference_sha256"] = hashlib.sha256(Path(REFERENCE).read_bytes()).hexdigest()
    (tmp_path / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)
