"""Exact S5 FOM0/1 parity with legal, full-width camera raster inputs.

Defaults to the current production lrcca closure. S5_MOMENTS_SOURCE may select
an artifact rtl/lrcca directory before promotion; S5_MOMENTS_BUILD retains the
source-bound reports. Native uses the unmodified vendor DSP model; portable
requires no vendor library. LUTCAM_REQUIRE_EFINIX_NATIVE=1 forbids skipping a
missing simulator or vendor model. The neighboring helper regression proves
the complete binary arithmetic domain independently of this pipeline miter.
"""
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest import mock

from s5_moments_support import run_profile

ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'rtl/lrcca').is_dir())
SOURCE = Path(os.environ.get('S5_MOMENTS_SOURCE', ROOT / 'rtl/lrcca'))
MODEL = Path(os.environ.get('EFINITY_HOME') or os.environ.get('LITEX_ENV_EFINITY') or '/nonexistent') / 'sim_models/verilog/efx_dsp48.v'


def require_dependencies(native):
    problem = None
    if not shutil.which('verilator'):
        problem = 'Verilator executable is missing'
    elif native and not MODEL.is_file():
        problem = 'Efinity DSP model is missing; set EFINITY_HOME or LITEX_ENV_EFINITY'
    if problem:
        if os.environ.get('LUTCAM_REQUIRE_EFINIX_NATIVE') == '1':
            raise AssertionError('Required ' + problem)
        raise unittest.SkipTest(problem)


class S5MomentsTests(unittest.TestCase):
    def check_profile(self, native):
        require_dependencies(native)
        self.assertTrue((SOURCE / 'TwoPPCS5Stage.sv').is_file(), str(SOURCE))
        with tempfile.TemporaryDirectory(prefix='s5-moments-') as temporary:
            build = Path(os.environ.get('S5_MOMENTS_BUILD', temporary)).resolve() / ('native' if native else 'portable')
            report = run_profile(SOURCE, build, native=native, vendor=MODEL)
            self.assertEqual(report['status'], 'PASS', f'See {build}/report.json and compile.log/run.log')
            print(report['stdout'], end='')

    def test_native_full_pipeline_exact_moments_and_cycles(self):
        self.check_profile(True)

    def test_portable_full_pipeline_exact_moments_and_cycles(self):
        self.check_profile(False)


class MomentsDependencyGuardTests(unittest.TestCase):
    def test_missing_simulator_fails_when_required_and_otherwise_skips(self):
        for required, exception in [('1', AssertionError), ('0', unittest.SkipTest)]:
            with self.subTest(required=required), mock.patch.dict(os.environ, {'LUTCAM_REQUIRE_EFINIX_NATIVE': required}), mock.patch.object(shutil, 'which', return_value=None):
                with self.assertRaisesRegex(exception, 'Verilator'):
                    require_dependencies(True)

    def test_missing_vendor_fails_when_required_and_otherwise_skips(self):
        for required, exception in [('1', AssertionError), ('0', unittest.SkipTest)]:
            with self.subTest(required=required), mock.patch.dict(os.environ, {'LUTCAM_REQUIRE_EFINIX_NATIVE': required}), mock.patch.object(shutil, 'which', return_value='/tool/verilator'), mock.patch.object(Path, 'is_file', return_value=False):
                with self.assertRaisesRegex(exception, 'Efinity DSP'):
                    require_dependencies(True)

    def test_portable_needs_no_vendor_even_when_native_required(self):
        with mock.patch.dict(os.environ, {'LUTCAM_REQUIRE_EFINIX_NATIVE': '1'}), mock.patch.object(shutil, 'which', return_value='/tool/verilator'), mock.patch.object(Path, 'is_file', return_value=False):
            require_dependencies(False)

    def test_present_native_dependencies_do_not_skip(self):
        with mock.patch.dict(os.environ, {'LUTCAM_REQUIRE_EFINIX_NATIVE': '1'}), mock.patch.object(shutil, 'which', return_value='/tool/verilator'), mock.patch.object(Path, 'is_file', return_value=True):
            require_dependencies(True)


if __name__ == '__main__':
    unittest.main()
