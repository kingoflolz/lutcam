"""Generate a source-bound S5 miter; inputs come from the real S0..S4 raster path.

The maintained profile instantiates the same production S5 twice with only
USE_DSP_HIGH_FOM different. No pipeline payload masking occurs.
"""
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import time

HELPERS = ["lrcca_util_pkg.sv", "TwoPPC.sv", *[
    f"TwoPPCS{stage}Stage.sv" for stage in [0, 1, 2, 3, 4, "6B"]]]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wrapper(source):
    header = source[:source.index("\n);") + 3]
    params = re.findall(r"^\s*parameter\s+(?:int|bit)\s+(\w+)\s*=", header, re.M)
    ports = re.findall(r"^\s*(input|output)\s+logic\s*(\[[^\]]+\])?\s*(\w+)\s*[,\n]", header, re.M)
    outputs = [(width, name) for direction, width, name in ports if direction == "output"]
    assert len(outputs) == 19, outputs
    assert "USE_DSP_HIGH_ZOM" in params
    assert "USE_DSP_HIGH_FOM" in params
    def parameter_map(fom):
        values = {x: x for x in params}
        values["USE_DSP_HIGH_ZOM"] = "1'b1"
        values["USE_DSP_HIGH_FOM"] = "1'b1" if fom else "1'b0"
        return ", ".join(f".{name}({value})" for name, value in values.items())
    base_name = "TwoPPCS5StageSubject"
    opt_name = "TwoPPCS5StageSubject"
    result = header + "\n"
    result += "\n".join(f"  logic {width or ''} opt_{name};" for width, name in outputs) + "\n"
    result += f"  {base_name} #({parameter_map(False)}) base (.*);\n"
    result += f"  {opt_name} #({parameter_map(True)}) opt (" + ", ".join(
        f".{name}(opt_{name})" for _, name in outputs) + ", .*);\n"
    result += """
  integer compared_cycles = 0;
  integer valid_s5_cycles = 0;
  integer compared_data_writes = 0;
  integer compared_tail_writes = 0;
  integer close_events = 0;
  integer state_updates = 0;
  integer lane0_close_events = 0;
  integer lane1_close_events = 0;
  integer reset_cycles = 0;
  logic [7:0] observed_predicates = '0;
  logic [3:0] observed_shapes = '0;
  logic [1:0] observed_previous_lanes = '0;
  logic [2:0] observed_close_refs0 = '0;
  logic [2:0] observed_close_refs1 = '0;
  logic [2:0] observed_return_refs0 = '0;
  logic [2:0] observed_return_refs1 = '0;
  logic [1:0] observed_base_to_pair = '0;
  localparam integer X_LOW = MOMENT_X_BITS - 17;
  localparam integer Y_LOW = MOMENT_Y_BITS - 17;
  wire [MOMENT_X_BITS-1:0] x_base = base.feature_x_fom_of(base.fused_pair_base_data);
  wire [MOMENT_Y_BITS-1:0] y_base = base.feature_y_fom_of(base.fused_pair_base_data);
  wire [MOMENT_X_BITS:0] x_sum = {1'b0,x_base} + {1'b0,base.fused_prev_x_fom};
  wire [MOMENT_Y_BITS:0] y_sum = {1'b0,y_base} + {1'b0,base.fused_prev_y_fom};
  wire [X_LOW:0] x_low_sum = {1'b0,x_base[X_LOW-1:0]} + {1'b0,base.fused_prev_x_fom[X_LOW-1:0]};
  wire [Y_LOW:0] y_low_sum = {1'b0,y_base[Y_LOW-1:0]} + {1'b0,base.fused_prev_y_fom[Y_LOW-1:0]};
  logic [MOMENT_X_BITS-1:0] x_ones = '0, x_zeroes = '0;
  logic [MOMENT_Y_BITS-1:0] y_ones = '0, y_zeroes = '0;
  logic [1:0] x_delta_sign = '0, y_delta_sign = '0;
  logic [1:0] x_carry_values = '0, y_carry_values = '0;
  integer x_low_carries = 0, y_low_carries = 0;
  integer x_modular_wraps = 0, y_modular_wraps = 0;
  integer x_negative_merges = 0, y_negative_merges = 0;
  initial begin
    assert (MOMENT_X_BITS == 38 && MOMENT_Y_BITS == 37)
      else $fatal(1, "Fixture must exercise production 38/37-bit quotients");
    assert (base.USE_DSP_HIGH_ZOM && opt.USE_DSP_HIGH_ZOM && opt.USE_DSP_HIGH_FOM)
      else $fatal(1, "Incorrect arithmetic flags");
`ifdef SYNTHESIS
    assert (opt.dsp_x_merge_add.USE_NATIVE_HIGH && opt.dsp_y_merge_add.USE_NATIVE_HIGH)
      else $fatal(1, "Native profile failed to use vendor arithmetic");
`else
    assert (!opt.dsp_x_merge_add.USE_NATIVE_HIGH && !opt.dsp_y_merge_add.USE_NATIVE_HIGH)
      else $fatal(1, "Portable profile incorrectly selected vendor arithmetic");
`endif
  end
  always_ff @(posedge clk) begin
    if (rst) begin
      reset_cycles <= reset_cycles + 1;
    end else begin
      compared_cycles <= compared_cycles + 1;
"""
    # The arithmetic candidate promises identical final outputs even when
    # invalid; compare every output bit, without weakening payload qualification.
    for _, name in outputs:
        result += f'      assert ({name} === opt_{name}) else $fatal(1, "S5 public {name} differs at cycle %0d", compared_cycles);\n'
    for name in ["s5", "s5_token", "p_active", "c_reg", "f_reg", "h_reg", "t_reg", "d_reg",
                 "s5_token_data_bypass_valid", "s5_token_data_bypass_source", "s5_token_data_bypass_data",
                 "data_r_en0", "data_r_en1", "tail_r_en", "data_r_addr0", "data_r_addr1", "tail_r_addr",
                 "data_r_data0", "data_r_data1", "tail_r_data", "data_w_en0", "data_w_en1", "tail_w_en"]:
        result += f'      assert (base.{name} === opt.{name}) else $fatal(1, "S5 state/control {name} differs at cycle %0d", compared_cycles);\n'
    result += """
      assert (base.fused_zom_final === opt.fused_zom_final)
        else $fatal(1, "Fused ZOM differs at cycle %0d", compared_cycles);
      assert (base.fused_x_final === opt.fused_x_final)
        else $fatal(1, "Fused X differs at cycle %0d", compared_cycles);
      assert (base.fused_y_final === opt.fused_y_final)
        else $fatal(1, "Fused Y differs at cycle %0d", compared_cycles);
      if (base.s5.valid) begin
        x_ones <= x_ones | x_base | base.fused_prev_x_fom | base.fused_x_delta_q | base.fused_x_final;
        x_zeroes <= x_zeroes | ~x_base | ~base.fused_prev_x_fom | ~base.fused_x_delta_q | ~base.fused_x_final;
        y_ones <= y_ones | y_base | base.fused_prev_y_fom | base.fused_y_delta_q | base.fused_y_final;
        y_zeroes <= y_zeroes | ~y_base | ~base.fused_prev_y_fom | ~base.fused_y_delta_q | ~base.fused_y_final;
        x_delta_sign[base.fused_x_delta_q[MOMENT_X_BITS-1]] <= 1'b1;
        y_delta_sign[base.fused_y_delta_q[MOMENT_Y_BITS-1]] <= 1'b1;
        x_carry_values[base.fused_x_three_mass_carry] <= 1'b1;
        y_carry_values[base.fused_y_three_mass_carry] <= 1'b1;
        if (x_low_sum[X_LOW]) x_low_carries <= x_low_carries + 1;
        if (y_low_sum[Y_LOW]) y_low_carries <= y_low_carries + 1;
        if (x_sum[MOMENT_X_BITS]) x_modular_wraps <= x_modular_wraps + 1;
        if (y_sum[MOMENT_Y_BITS]) y_modular_wraps <= y_modular_wraps + 1;
        if (x_base[MOMENT_X_BITS-1] && base.fused_prev_x_fom[MOMENT_X_BITS-1])
          x_negative_merges <= x_negative_merges + 1;
        if (y_base[MOMENT_Y_BITS-1] && base.fused_prev_y_fom[MOMENT_Y_BITS-1])
          y_negative_merges <= y_negative_merges + 1;
        valid_s5_cycles <= valid_s5_cycles + 1;
        state_updates <= state_updates + 1;
        observed_shapes[{base.s5.d0, base.s5.d1}] <= 1'b1;
        observed_predicates <= observed_predicates | {
          base.s5_sp0, base.s5_sp1, base.s5_o0, base.s5_o1,
          base.s5_ec0, base.s5_ec1, base.s5_ep0, base.s5_ep1};
        observed_previous_lanes <= observed_previous_lanes |
          {base.fused_use_prev1, base.fused_use_prev0};
        observed_close_refs0[base.lane0_data_refs.close_data] <= 1'b1;
        observed_close_refs1[base.lane1_data_refs.close_data] <= 1'b1;
        observed_return_refs0[base.lane0_data_refs.ret_data] <= 1'b1;
        observed_return_refs1[base.lane1_data_refs.ret_data] <= 1'b1;
        observed_base_to_pair[base.fused_base_to_pair] <= 1'b1;
        assert (base.lane0.state_ret === opt.lane0.state_ret)
          else $fatal(1, "S5 lane0 next state differs at cycle %0d", compared_cycles);
        assert (base.lane1.state_ret === opt.lane1.state_ret)
          else $fatal(1, "S5 lane1 next state differs at cycle %0d", compared_cycles);
      end
      if (base.data_w_en0) begin
        assert ({base.data_w_addr0, base.data_w_data0} === {opt.data_w_addr0, opt.data_w_data0})
          else $fatal(1, "S5 RAM0 write differs at cycle %0d", compared_cycles);
      end
      if (base.data_w_en1) begin
        assert ({base.data_w_addr1, base.data_w_data1} === {opt.data_w_addr1, opt.data_w_data1})
          else $fatal(1, "S5 RAM1 write differs at cycle %0d", compared_cycles);
      end
      compared_data_writes <= compared_data_writes + int'(base.data_w_en0) + int'(base.data_w_en1);
      if (base.tail_w_en) begin
        compared_tail_writes <= compared_tail_writes + 1;
        assert ({base.tail_w_addr, base.tail_w_data} === {opt.tail_w_addr, opt.tail_w_data})
          else $fatal(1, "S5 tail RAM write differs at cycle %0d", compared_cycles);
      end
      if (s5_close_moment_valid) begin
        close_events <= close_events + 1;
        if (s5_close_moment_lane) lane1_close_events <= lane1_close_events + 1;
        else lane0_close_events <= lane0_close_events + 1;
      end
    end
  end
  final begin
    assert (&x_ones && &x_zeroes && &y_ones && &y_zeroes)
      else $fatal(1, "Missing full-width moment bit coverage");
    assert (&x_delta_sign && &y_delta_sign && &x_carry_values && &y_carry_values)
      else $fatal(1, "Missing signed delta or three-mass carry coverage");
    assert (x_low_carries > 0 && y_low_carries > 0 && x_modular_wraps > 0 && y_modular_wraps > 0
        && x_negative_merges > 0 && y_negative_merges > 0)
      else $fatal(1, "Missing legal carry/modular-wrap/negative-merge coverage");
    $display("MOMENTS widths=%0d/%0d low_carry=%0d/%0d wrap=%0d/%0d negative_merge=%0d/%0d delta_sign=%b/%b parity_carry=%b/%b ones=%h/%h zeroes=%h/%h", MOMENT_X_BITS, MOMENT_Y_BITS, x_low_carries, y_low_carries, x_modular_wraps, y_modular_wraps, x_negative_merges, y_negative_merges, x_delta_sign, y_delta_sign, x_carry_values, y_carry_values, x_ones, y_ones, x_zeroes, y_zeroes);
    $display("S5 comparison coverage cycles=%0d valid=%0d writes=%0d tail_writes=%0d closes=%0d lane0_closes=%0d lane1_closes=%0d resets=%0d shapes=%b predicates=%b previous=%b close_refs0=%b close_refs1=%b return_refs0=%b return_refs1=%b base_to_pair=%b", compared_cycles, valid_s5_cycles, compared_data_writes, compared_tail_writes, close_events, lane0_close_events, lane1_close_events, reset_cycles, observed_shapes, observed_predicates, observed_previous_lanes, observed_close_refs0, observed_close_refs1, observed_return_refs0, observed_return_refs1, observed_base_to_pair);
  end
endmodule
"""
    return result


def run_profile(source_dir, build, *, native, vendor):
    """Compare the production FOM0 and FOM1 implementations."""
    source_dir, build = Path(source_dir).resolve(), Path(build).resolve()
    build.mkdir(parents=True, exist_ok=True)
    here = Path(__file__).resolve().parent
    originals = [source_dir / name for name in HELPERS]
    originals += [source_dir / 'TwoPPCS5Stage.sv', source_dir / 'S5HighChunkAdd.sv',
                  here / 's5_moments_raster_tb.sv', Path(__file__)]
    if native:
        originals += [vendor]
    original_hashes = {str(p): sha(p) for p in originals}
    sources = []
    for name in HELPERS:
        dest = build / name
        shutil.copyfile(source_dir / name, dest)
        sources.append(dest)
    cases = [(source_dir, 'TwoPPCS5StageSubject')]
    for directory, module in cases:
        source = (directory / 'TwoPPCS5Stage.sv').read_text()
        assert source.count('module TwoPPCS5Stage #(') == 1
        dest = build / (module + '.sv')
        dest.write_text(source.replace('module TwoPPCS5Stage #(', f'module {module} #(', 1))
        sources.append(dest)
    generated_wrapper = build / 'TwoPPCS5Stage.sv'
    generated_wrapper.write_text(wrapper((source_dir / 'TwoPPCS5Stage.sv').read_text()))
    sources.append(generated_wrapper)
    for original in [source_dir / 'S5HighChunkAdd.sv', here / 's5_moments_raster_tb.sv']:
        dest = build / original.name
        shutil.copyfile(original, dest)
        sources.append(dest)
    vendor_copy = build / 'efx_dsp48.v'
    if native:
        shutil.copyfile(vendor, vendor_copy)
        sources.append(vendor_copy)
    # Keep default RTL diagnostics fatal. Only width warnings in the exact
    # unmodified vendor simulation library are waived; timescale is explicit.
    lint = build / 'lint.vlt'
    lint.write_text('`verilator_config\n' + (
        f'lint_off -rule WIDTHEXPAND -file "{vendor_copy}"\n' if native else ''))
    sources_and_config = [*sources, lint]
    generated_hashes = {str(p): sha(p) for p in sources_and_config}
    command = ['verilator', '--binary', '--timing', '--assert', '--timescale', '1ns/1ps',
               '-j', '2', '--top-module', 'tb', '--Mdir', str(build / 'obj'),
               *(['-DSYNTHESIS'] if native else []), str(lint), *map(str, sources)]
    report = {
        'status': 'RUNNING', 'profile': 'native' if native else 'portable',
        'comparison_mode': 'same-source FOM0 versus FOM1',
        'source_directory': str(source_dir), 'source_directory': str(source_dir),
        'original_sha256': original_hashes, 'generated_sha256': generated_hashes,
        'native_vendor': str(vendor) if native else None, 'compile_command': command,
        'masking': 'All 19 public output bits (including invalid payloads), full pipeline/state/read controls/data, write enables and fused X/Y/ZOM compared each non-reset cycle. Data/tail write address+payload compare when enabled; lane next-state when s5.valid. Reset-high cycles excluded; post-reset recovery has no grace period.',
        'limits': 'Binary full-S5 parity under legal production-geometry raster traffic; no arbitrary-state forcing, no X-propagation claim, no exhaustive legal camera histories or full-board timing claim.'}
    start = time.monotonic()
    try:
        compiled = subprocess.run(command, cwd=build, capture_output=True, text=True, timeout=300)
        (build / 'compile.log').write_text(compiled.stdout + compiled.stderr)
        report['compile_returncode'] = compiled.returncode
        if compiled.returncode == 0:
            result = subprocess.run([str(build / 'obj/Vtb')], cwd=build, capture_output=True, text=True, timeout=180)
            (build / 'run.log').write_text(result.stdout + result.stderr)
            report.update(returncode=result.returncode, stdout=result.stdout, stderr=result.stderr)
    finally:
        report['seconds'] = time.monotonic() - start
        report['original_sources_unchanged'] = all(sha(Path(p)) == h for p, h in original_hashes.items())
        report['generated_sources_unchanged'] = all(sha(Path(p)) == h for p, h in generated_hashes.items())
        required_markers = ['PASS legal rasters=6 WIDTH=1696 HEIGHT=807',
                            'MOMENTS widths=38/37', 'S5 comparison coverage cycles=']
        report['required_markers_present'] = all(marker in report.get('stdout', '') for marker in required_markers)
        report['status'] = 'PASS' if report.get('compile_returncode') == 0 and report.get('returncode') == 0 and report['original_sources_unchanged'] and report['generated_sources_unchanged'] and report['required_markers_present'] else 'FAIL'
        (build / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    return report
