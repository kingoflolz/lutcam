"""Actual S5 high-chunk helper versus an independent widened sum reference.

The real Efinity primitive model is required for native verification. The same
source is also instantiated with SYNTHESIS undefined, checking the portable
fallback simultaneously. S5_HIGH_ADD_SOURCE selects a frozen artifact before
promotion; otherwise this tests production rtl/lrcca/S5HighChunkAdd.sv.
S5_HIGH_ADD_BUILD retains generated sources, logs and a hash-bound report.
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
from unittest import mock

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'rtl/lrcca').is_dir())
HERE=Path(__file__).resolve().parent
SOURCE=Path(os.environ.get('S5_HIGH_ADD_SOURCE',ROOT/'rtl/lrcca/S5HighChunkAdd.sv'))
MODEL=Path(os.environ.get('EFINITY_HOME') or os.environ.get('LITEX_ENV_EFINITY') or '/nonexistent')/'sim_models/verilog/efx_dsp48.v'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def require_dependencies(fabric):
    required=os.environ.get('LUTCAM_REQUIRE_EFINIX_NATIVE')=='1'
    problem=None
    if not shutil.which('verilator'):
        problem='Verilator executable is missing'
    elif not fabric and not MODEL.is_file():
        problem='Efinity DSP model is missing; set EFINITY_HOME or LITEX_ENV_EFINITY'
    if problem:
        if required:
            raise AssertionError('Required '+problem)
        raise unittest.SkipTest(problem)


class S5HighChunkAddTests(unittest.TestCase):
    def run_profile(self,fabric):
        require_dependencies(fabric)
        with tempfile.TemporaryDirectory(prefix='s5-high-add-') as temporary:
            profile='fabric' if fabric else 'native'
            build=Path(os.environ.get('S5_HIGH_ADD_BUILD',temporary)).resolve()/profile
            build.mkdir(parents=True,exist_ok=True)
            self.assertTrue(SOURCE.is_file(),str(SOURCE))
            source=SOURCE.read_text()
            self.assertEqual(source.count('module S5HighChunkAdd #('),1)
            frozen=build/'S5HighChunkAdd.sv'
            frozen.write_bytes(SOURCE.read_bytes())
            vendor=build/'efx_dsp48.v'
            if not fabric:
                vendor.write_bytes(MODEL.read_bytes())
            testbench=build/'s5_high_add_tb.sv'
            testbench.write_bytes((HERE/'s5_high_add_tb.sv').read_bytes())
            # Only the duplicate module name changes. Actual preprocessor
            # guards choose both implementations; the bench asserts each mode.
            combined=build/'implementations.sv'
            combined.write_text('`define SYNTHESIS\n`include "S5HighChunkAdd.sv"\n'
                '`undef SYNTHESIS\n'+source.replace('module S5HighChunkAdd #(',
                    'module S5HighChunkAddPortable #(',1)+'\n`define SYNTHESIS\n')
            top='s5_high_add_fabric_tb' if fabric else 's5_high_add_tb'
            # Waive only naming on the generated multi-module harness and
            # diagnostics owned by the unmodified vendor model. RTL warnings
            # remain fatal; there are no global warning waivers.
            lint_config=build/'lint.vlt'
            waivers=[('DECLFILENAME',combined),('DECLFILENAME',testbench)]
            if not fabric:
                waivers += [(rule,vendor) for rule in [
                    'DECLFILENAME','GENUNNAMED','VARHIDDEN','DEFPARAM',
                    'WIDTHEXPAND','UNUSEDPARAM','PROCASSINIT','UNUSEDSIGNAL']]
            lint_config.write_text('`verilator_config\n'+''.join(
                f'lint_off -rule {rule} -file "{path}"\n' for rule,path in waivers))
            command=['verilator','--binary','--timing','--assert','-Wall','-Wpedantic',
                '--timescale','1ns/1ps',str(lint_config),
                '-j','2','--top-module',top,'--Mdir',str(build/'obj'),
                *(['-DS5_HIGH_ADD_NO_VENDOR'] if fabric else []),
                '-I'+str(build),str(combined),str(testbench),
                *([] if fabric else [str(vendor)])]
            compiled=subprocess.run(command,cwd=build,capture_output=True,text=True,timeout=240)
            (build/'compile.log').write_text(compiled.stdout+compiled.stderr)
            self.assertEqual(compiled.returncode,0,compiled.stdout+compiled.stderr)
            result=subprocess.run([str(build/('obj/V'+top))],cwd=build,
                capture_output=True,text=True,timeout=240)
            (build/'run.log').write_text(result.stdout+result.stderr)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            rows=[dict(zip(['width','native','checks','directed','boundary','random','overflow','low_carry'],
                map(int,m))) for m in re.findall(r'PASS width=(\d+) native=(\d+) checks=(\d+) directed=(\d+) boundary=(\d+) random=(\d+) overflow=(\d+) low_carry=(\d+)',result.stdout)]
            rows.sort(key=lambda r:r['width'])
            widths=[29,97] if fabric else [1,17,18,26,28,29,32,33,35,37,38,39,40,64,97]
            self.assertEqual([r['width'] for r in rows],widths)
            self.assertEqual(sum(r['random'] for r in rows),400000*len(widths))
            self.assertTrue(all(r['native']==int(r['width']>17 and not fabric) for r in rows))
            marker='PASS all explicit fabric widths; no vendor model or clock transitions' if fabric else 'PASS all native/portable widths; no clock transitions'
            self.assertIn(marker,result.stdout)
            hashed=[SOURCE,HERE/'test_s5_high_add.py',HERE/'s5_high_add_tb.sv',frozen,combined,testbench,lint_config]
            if not fabric: hashed += [MODEL,vendor]
            report={'status':'PASS','profile':profile,'scope':'Standalone actual-source helper, simultaneous ordinary-simulation portable branch, independent widened unsigned addition oracle; no clocks or resets driven. Native profile includes real vendor model; explicit USE_DSP=0 profile compiles without any vendor primitive definition.',
                'cases':rows,'total_checks':sum(r['checks'] for r in rows),
                'random_checks':sum(r['random'] for r in rows),
                'clock_transitions':0,'source_path':str(SOURCE.resolve()),
                'vendor_path':None if fabric else str(MODEL.resolve()),'command':command,
                'sha256':{str(p):sha(p) for p in hashed},
                'limits':'Binary arithmetic verification. Does not claim arbitrary X-propagation equivalence, pipeline reset/valid behavior, or physical timing.'}
            (build/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(result.stdout,end='')
            print(json.dumps({'profile':profile,'total_checks':report['total_checks'],'random_checks':report['random_checks'],'source_sha256':sha(SOURCE),'vendor_sha256':None if fabric else sha(MODEL)},sort_keys=True))

    def test_native_and_portable_full_width_modular_arithmetic(self):
        self.run_profile(False)

    def test_explicit_fabric_synthesis_without_vendor_primitives(self):
        self.run_profile(True)



class DependencyGuardTests(unittest.TestCase):
    def test_required_missing_verilator_fails(self):
        with mock.patch.dict(os.environ,{'LUTCAM_REQUIRE_EFINIX_NATIVE':'1'}), \
             mock.patch.object(shutil,'which',return_value=None):
            with self.assertRaisesRegex(AssertionError,'Required Verilator'):
                require_dependencies(False)

    def test_optional_missing_verilator_skips(self):
        with mock.patch.dict(os.environ,{'LUTCAM_REQUIRE_EFINIX_NATIVE':'0'}), \
             mock.patch.object(shutil,'which',return_value=None):
            with self.assertRaisesRegex(unittest.SkipTest,'Verilator'):
                require_dependencies(False)

    def test_required_missing_model_fails(self):
        with mock.patch.dict(os.environ,{'LUTCAM_REQUIRE_EFINIX_NATIVE':'1'}), \
             mock.patch.object(shutil,'which',return_value='/tool/verilator'), \
             mock.patch.object(Path,'is_file',return_value=False):
            with self.assertRaisesRegex(AssertionError,'Required Efinity DSP'):
                require_dependencies(False)

    def test_optional_missing_model_skips(self):
        with mock.patch.dict(os.environ,{'LUTCAM_REQUIRE_EFINIX_NATIVE':'0'}), \
             mock.patch.object(shutil,'which',return_value='/tool/verilator'), \
             mock.patch.object(Path,'is_file',return_value=False):
            with self.assertRaisesRegex(unittest.SkipTest,'Efinity DSP'):
                require_dependencies(False)

    def test_fabric_needs_no_model_even_when_native_required(self):
        with mock.patch.dict(os.environ,{'LUTCAM_REQUIRE_EFINIX_NATIVE':'1'}), \
             mock.patch.object(shutil,'which',return_value='/tool/verilator'), \
             mock.patch.object(Path,'is_file',return_value=False):
            require_dependencies(True)

    def test_present_native_dependencies_do_not_skip(self):
        with mock.patch.dict(os.environ,{'LUTCAM_REQUIRE_EFINIX_NATIVE':'1'}), \
             mock.patch.object(shutil,'which',return_value='/tool/verilator'), \
             mock.patch.object(Path,'is_file',return_value=True):
            require_dependencies(False)


if __name__=='__main__':
    unittest.main()
