"""Exercise the actual firmware AE math against independent synthetic scenes."""
import ctypes
import math
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
PAIRS=512000


class Histogram(ctypes.Structure):
    _fields_=[(name,ctypes.c_uint32) for name in ("count","p50","p90","p99","clipped")]


class History(ctypes.Structure):
    _fields_=[(name,ctypes.c_int32) for name in ("p50","p90","p99","exposure")]+[("valid",ctypes.c_uint32)]


class HistogramExposureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory(prefix="lutcam-ae-")
        cls.addClassCleanup(cls.temp.cleanup)
        root=Path(cls.temp.name)
        source=root/'ae.c'
        source.write_text('''#include "ae.h"
int32_t log_value(uint32_t x) { return ae_log2(x); }
AeHistogram histogram(const uint32_t *bins) {
    AeHistogram h={0};
    for (uint32_t i=0;i<512;i++) ae_bin(&h,i,bins[i]);
    return h;
}
int32_t correction(AeHistory *h,AeHistogram *a,AeHistogram *b,uint32_t e,int32_t q) {
    int32_t scratch[6]; return ae_error(h,a,b,e,q,scratch);
}
uint32_t settings(uint32_t l,uint32_t g,int32_t e) { return ae_settings(l,g,e,ae_log2(l*g)); }
''')
        subprocess.run(['clang','-shared','-fPIC','-O2','-Wall','-Wextra','-Werror',
                        '-I',str(ROOT/'firmware'),str(source),'-o',str(root/'ae.so')],check=True)
        cls.lib=ctypes.CDLL(str(root/'ae.so'))
        cls.lib.log_value.argtypes=[ctypes.c_uint32];cls.lib.log_value.restype=ctypes.c_int32
        cls.lib.histogram.argtypes=[ctypes.POINTER(ctypes.c_uint32)];cls.lib.histogram.restype=Histogram
        cls.lib.correction.argtypes=[ctypes.POINTER(History),ctypes.POINTER(Histogram),ctypes.POINTER(Histogram),ctypes.c_uint32,ctypes.c_int32]
        cls.lib.correction.restype=ctypes.c_int32
        cls.lib.settings.argtypes=[ctypes.c_uint32,ctypes.c_uint32,ctypes.c_int32]
        cls.lib.settings.restype=ctypes.c_uint32

    def histogram(self, scene, energy=6400, black_level=0):
        # Scene intensities are unbounded linear RAW-pair bins at 400 lines,
        # unity gain; rendering clips them to the actual 9-bit ISP bins.
        bins=[0]*512
        for intensity, count in scene:
            value=min(511,max(0,black_level+int(intensity*energy/6400)))
            bins[value]+=count
        self.assertEqual(sum(bins),PAIRS)
        h=self.lib.histogram((ctypes.c_uint32*512)(*bins))
        self.assertEqual(h.count,PAIRS)
        cumulative=0
        expected={}
        for i,n in enumerate(bins):
            cumulative+=n
            for q in (50,90,99):
                if q not in expected and cumulative>=PAIRS*q//100: expected[q]=i
        self.assertEqual((h.p50,h.p90,h.p99,h.clipped),(expected[50],expected[90],expected[99],sum(bins[508:])))
        return h

    def next(self, lines, gain, error):
        packed=self.lib.settings(lines,gain,error)
        result=packed&1023,packed>>10
        self.assertTrue(1<=result[0]<=885 and 16<=result[1]<=255)
        return result

    def converge(self, scene, *, second=None, initial=(400,16), quarters=0, history=None, black_level=0):
        state=initial;trace=[];keys=set()
        history=history if history is not None else History()
        for _ in range(60):
            a=self.histogram(scene,state[0]*state[1],black_level)
            b=self.histogram(second or scene,state[0]*state[1],black_level)
            error=self.lib.correction(ctypes.byref(history),ctypes.byref(a),ctypes.byref(b),state[0]*state[1],quarters)
            following=self.next(*state,error)
            trace.append((state,error,a.p90,b.p90,a.clipped,b.clipped))
            if following==state: return state,trace
            key=(*state,history.p50,history.p90,history.p99,history.valid)
            self.assertNotIn(key,keys,f"exposure limit cycle: {trace}")
            keys.add(key)
            state=following
        self.fail(f"exposure did not settle: {trace}")

    def test_log_is_monotonic_and_accurate_in_stop_units(self):
        previous=-1
        for value in range(1,885*255+1):
            actual=self.lib.log_value(value)
            self.assertGreaterEqual(actual,previous)
            self.assertLessEqual(abs(actual/16-math.log2(value)),0.10)
            previous=actual

    def test_dark_bright_and_mixed_scenes_converge_without_limit_cycles(self):
        shapes=[[(100,PAIRS)],[(10,PAIRS//2),(200,PAIRS//2)],
                [(25,PAIRS*7//10),(160,PAIRS*29//100),(10000,PAIRS//100)],
                [(40,PAIRS*4//5),(250,PAIRS//5)]]
        for scene in shapes:
            for stops in (-4,-2,0,2,4):
                for initial in ((1,16),(400,16),(885,255)):
                    with self.subTest(scene=scene,stops=stops,initial=initial):
                        _,trace=self.converge([(v*2**stops,n) for v,n in scene],initial=initial)
                        self.assertLessEqual(trace[-1][4],PAIRS//100)

    def test_quarter_stop_bias_changes_exposure_by_the_requested_ratio(self):
        scene=[(100,PAIRS)]
        neutral,_=self.converge(scene)
        for quarters in (-8,-6,-4,-2,0,1,2,4,6,8):
            with self.subTest(quarters=quarters):
                state,_=self.converge(scene,quarters=quarters)
                ratio=(state[0]*state[1])/(neutral[0]*neutral[1])
                self.assertAlmostEqual(math.log2(ratio),quarters/4,delta=0.18)

    def test_small_highlights_are_ignored_but_clipping_is_bounded_in_each_camera(self):
        diffuse=[(100,PAIRS)]
        specular=[(100,PAIRS-512),(100000,512)]
        self.assertEqual(self.converge(diffuse)[0],self.converge(specular)[0])
        bright=[(100,PAIRS*4//5),(1000,PAIRS//5)]
        state,trace=self.converge(diffuse,second=bright)
        self.assertLessEqual(trace[-1][5],5120)
        self.assertLess(state[0]*state[1],6400)

    def test_positive_ev_scales_the_highlight_limited_baseline_and_returns_to_zero(self):
        scene=[(40,PAIRS*4//5),(800,PAIRS//5)]
        history=History()
        neutral,_=self.converge(scene,history=history)
        state=neutral
        for quarters in (4,8,0,-4,0):
            state,trace=self.converge(scene,initial=state,quarters=quarters,history=history)
            ratio=state[0]*state[1]/(neutral[0]*neutral[1])
            self.assertAlmostEqual(math.log2(ratio),quarters/4,delta=0.18)
            if quarters>0: self.assertGreater(trace[-1][4],5120)

    def test_positive_ev_tracks_lighting_changes_when_highlights_are_clipped(self):
        scene=[(40,PAIRS*4//5),(800,PAIRS//5)]
        history=History()
        state,_=self.converge(scene,quarters=4,history=history)
        first=state[0]*state[1]
        for stops in (1,-1,0):
            state,_=self.converge([(v*2**stops,n) for v,n in scene],initial=state,quarters=4,history=history)
            self.assertAlmostEqual(math.log2(state[0]*state[1]/first),-stops,delta=0.20)

    def test_black_offset_does_not_amplify_positive_ev_through_the_median(self):
        scene=[(40,PAIRS*4//5),(800,PAIRS//5)]
        for black in (16,32,64):
            with self.subTest(black_level=black):
                history=History()
                neutral,_=self.converge(scene,history=history,black_level=black)
                state=neutral
                for quarters in (4,8,0):
                    state,_=self.converge(scene,initial=state,quarters=quarters,history=history,black_level=black)
                    self.assertAlmostEqual(math.log2(state[0]*state[1]/(neutral[0]*neutral[1])),quarters/4,delta=.18)

    def test_clipped_highlight_estimate_does_not_drift_under_percentile_noise(self):
        scenes=[[(40,PAIRS*4//5),(800,PAIRS//5)],
                [(100,PAIRS*95//100),(800,PAIRS*5//100)]]
        for scene in scenes:
            for black in (0,32,64):
                for noise in (1,2,4):
                    with self.subTest(scene=scene,black=black,noise=noise):
                        history=History()
                        neutral,_=self.converge(scene,history=history,black_level=black)
                        state=neutral
                        for quarters in (4,8,0):
                            # Continue metering after convergence: noise can
                            # otherwise turn a clipped estimate into a ratchet.
                            for frame in range(150):
                                h=self.histogram(scene,state[0]*state[1],black)
                                h.p50+=noise*(frame%3-1)
                                if h.p90<508: h.p90+=noise*(frame%3-1)
                                error=self.lib.correction(ctypes.byref(history),ctypes.byref(h),ctypes.byref(h),state[0]*state[1],quarters)
                                state=self.next(*state,error)
                            self.assertAlmostEqual(math.log2(state[0]*state[1]/(neutral[0]*neutral[1])),quarters/4,delta=.18)

    def test_clipped_highlight_anchor_accumulates_gradual_lighting_changes(self):
        scene=[(40,PAIRS*4//5),(800,PAIRS//5)]
        history=History()
        state,_=self.converge(scene,quarters=4,history=history)
        first=state[0]*state[1]
        for step in range(1,65):
            stops=step/64
            state,_=self.converge([(v*2**stops,n) for v,n in scene],initial=state,quarters=4,history=history)
            self.assertAlmostEqual(math.log2(state[0]*state[1]/first),-stops,delta=.35)

    def test_step_limits_and_sensor_limits(self):
        for lines,gain in ((400,16),(885,32),(885,128)):
            for error in (-128,-16,0,16,128):
                new=self.next(lines,gain,error)
                stops=math.log2(new[0]*new[1]/(lines*gain))
                self.assertGreaterEqual(stops,-1.03)
                self.assertLessEqual(stops,0.54)
                if abs(error)<=2: self.assertEqual(new,(lines,gain))
        self.assertEqual(self.next(1,16,-128),(1,16))
        self.assertEqual(self.next(885,255,128),(885,255))
        self.assertGreater(self.next(1,16,128)[0],1)

    def test_integer_line_quantization_does_not_toggle_between_one_and_two(self):
        for optimum in (1,1.25,1.4,1.6,1.8,2,3,5,10):
            with self.subTest(optimum=optimum):
                scene=[(384*400/optimum,PAIRS)]
                self.converge(scene,initial=(1,16))

    def test_unexposable_scenes_settle_at_the_sensor_limits(self):
        self.assertEqual(self.converge([(0,PAIRS)],initial=(1,16))[0],(885,255))
        self.assertEqual(self.converge([(1e9,PAIRS)],initial=(885,255))[0],(1,16))


if __name__=='__main__': unittest.main()
