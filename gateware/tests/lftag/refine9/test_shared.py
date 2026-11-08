from pathlib import Path
import pytest
import subprocess,tempfile,os,sys
import numpy as np
from model import world_points
from program import reference
ROOT=Path(__file__).resolve().parents[3]
def test_shared(use_native=False):
    rows=[]
    for f in range(12):
        tag_id=(0x1234+f*311)&16383;scale=(f//2)%3;c=f%2
        w=world_points(tag_id);theta=(-2.8,-2.2,-1.6,-1.0,-.4,0.,.6,1.1,1.7,2.1,2.7,3.1)[f];a=np.cos(theta);b=np.sin(theta);pitch=60+10*f
        h=np.array([[pitch*a,-pitch*b,0],[pitch*b,pitch*a,0],[.025*(-1)**f,-.018,1.]])
        h=np.array([[1,0,640+f*4],[0,1,400+f*2],[0,0,1]])@h
        q=np.c_[w,np.ones(9)]@h.T;points=np.rint(16*q[:,:2]/q[:,2,None]).astype(np.int64)
        native=points.copy()
        if scale==2:native[:,1]-=64
        native=native>>(2*scale)
        source=native<<(2*scale)
        if scale==2:source[:,1]+=64
        out=reference(tag_id,native,scale=scale,invf=(1533917,1470879) if c==0 else (1851279,1731842),principal=(640,400) if c==0 else (620,390))
        rows += [tag_id,scale]+[int(v) for pair in native for v in pair]+out
    with tempfile.TemporaryDirectory(prefix='lftag-refined-shared-') as td:
        d=Path(td);(d/'shared_vectors.hex').write_text(''.join(f'{v&0xffffffff:08x}\n' for v in rows))
        native_options=[]
        if use_native:
            efinity = os.environ.get("LITEX_ENV_EFINITY") or os.environ.get("EFINITY_HOME")
            vendor = Path(efinity) / "sim_models/verilog/efx_dsp48.v" if efinity else None
            if vendor is None or not vendor.is_file():
                message = "Set EFINITY_HOME or LITEX_ENV_EFINITY for the native DSP48 model"
                if os.environ.get("LUTCAM_REQUIRE_EFINIX_NATIVE") == "1":
                    pytest.fail(message)
                pytest.skip(message)
            native_options=['-DSYNTHESIS',str(vendor)]
        sources=[str(ROOT/f) for f in (ROOT/'rtl/lftag/sources.f').read_text().splitlines() if f.strip()]
        r=subprocess.run(['verilator','--binary','--timing','--assert','-Wno-fatal','-j','4','--top-module','shared_refine_tb','--Mdir',str(d/'obj'),'-I'+str(ROOT/'rtl/lftag'),*native_options,*sources,str(Path(__file__).with_name('shared_tb.sv'))],cwd=d,capture_output=True,text=True,timeout=240)
        assert r.returncode==0,(r.stdout+r.stderr)[-6000:]
        for case in range(7):
            r=subprocess.run([str(d/'obj/Vshared_refine_tb'),f'+CASE={case}'],cwd=d,capture_output=True,text=True,timeout=120);print(r.stdout);assert r.returncode==0,r.stdout+r.stderr
if __name__=='__main__':test_shared('--native' in sys.argv)
