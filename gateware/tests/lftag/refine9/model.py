"""Independent floating least-squares reference and synthetic nine-point fixtures."""
from __future__ import annotations
import numpy as np

DATA_CENTERS = [(0,-1),(-1,0),(0,0),(1,0),(-1,1),(0,1),(1,1)]


def world_points(tag_id):
    return np.array([(-1.,-1.),(1.,-1.)] + [
        (x + (1 if tag_id & (2 << (2*i)) else -1)/12,
         y + (1 if tag_id & (1 << (2*i)) else -1)/12)
        for i,(x,y) in enumerate(DATA_CENTERS)])


def fit_float(world, observed):
    p = np.column_stack((world,np.ones(len(world))))
    a = np.zeros((2*len(world),8)); b = observed.ravel()
    a[::2,:3]=p; a[1::2,3:6]=p
    a[::2,6:]=-observed[:,0,None]*world
    a[1::2,6:]=-observed[:,1,None]*world
    h=np.r_[np.linalg.lstsq(a,b,rcond=None)[0],1.].reshape(3,3)
    return h


def fixtures(count=500):
    rng=np.random.default_rng(0x1f7a9)
    for i in range(count):
        tag_id=int(rng.integers(16384));world=world_points(tag_id)
        theta=rng.uniform(-np.pi,np.pi);c,s=np.cos(theta),np.sin(theta)
        scale=rng.uniform(15,140)
        h=np.array([[scale*c,-scale*s,0],[scale*s,scale*c,0],
                    [rng.uniform(-.15,.15),rng.uniform(-.15,.15),1.]])
        h=np.array([[1,0,640],[0,1,400],[0,0,1]])@h
        p=np.column_stack((world,np.ones(len(world))))@h.T
        observed=p[:,:2]/p[:,2,None]+rng.normal(0,.25,size=(9,2))
        yield tag_id,np.rint(observed*16).astype(np.int64),h
