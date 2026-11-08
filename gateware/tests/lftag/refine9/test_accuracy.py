"""Compare fitted centroids with independent floating geometry and ground truth."""
from pathlib import Path
import json
import numpy as np
from model import fixtures,world_points,fit_float
from program import reference

def project(h,w):
    p=np.c_[w,np.ones(len(w))]@h.T
    return p[:,:2]/p[:,2,None]

def test_accuracy():
    errors=[];refined=[];four=[]
    # Use the exact configured inverse focal length for the independent
    # de-calibration, so this measures the RTL's fit and Q16 narrowing.
    focal=(1<<30)/1533917;k=np.array([[focal,0,640],[0,focal,400],[0,0,1.]])
    for tag_id,points,truth in fixtures(500):
        w=world_points(tag_id);actual=k@(np.asarray(reference(tag_id,points)).reshape(3,3)/65536)
        lo=points.min(0);hi=points.max(0);center=(lo+hi)//2
        scale=np.array([2**int((h-l)//2+1).bit_length() for l,h in zip(lo,hi)])
        observed=(points-center)/scale
        t=np.array([[scale[0]/16,0,center[0]/16],[0,scale[1]/16,center[1]/16],[0,0,1.]])
        expected=t@fit_float(w,observed)
        errors.append(np.max(np.abs(project(actual,w)-project(expected,w))))
        exact=project(truth,w)
        refined.append(float(np.sqrt(np.mean((project(actual,w)-exact)**2))))
        corners=[0,1,8,6]
        h4=fit_float(w[corners],points[corners]/16)
        four.append(float(np.sqrt(np.mean((project(h4,w)-exact)**2))))
    report=dict(fixtures=len(errors),max_coordinate_error_vs_float_pixels=float(max(errors)),
        mean_centroid_rms_9_point_pixels=float(np.mean(refined)),
        mean_centroid_rms_4_point_pixels=float(np.mean(four)),
        nine_point_wins=sum(a<b for a,b in zip(refined,four)))
    print(json.dumps(report,indent=2))
    # The 20-bit scratch format must stay within two Q4 input-coordinate bins.
    assert max(errors)<2/16,report
    assert np.mean(refined)<np.mean(four),report
    return report

if __name__=='__main__':test_accuracy()
