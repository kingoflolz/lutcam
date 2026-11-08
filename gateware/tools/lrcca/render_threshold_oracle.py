"""Diagnostic ideal tag segmentation; never an input to a deployable algorithm.

Labels: 0=outside known paper, 1=white paper, 2=black ink. Area coverage is
sampled at 8x8 subpixels in each detector plane's physical sampling grid.
The diagnostic overrides threshold decisions only on known tag paper.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import numpy as np
from PIL import Image
from render_threshold_dataset import canonical_points


def render(frame, root):
    for name, scale, phase in [('raw',1,0),('ll2',4,0),('ll4',16,4)]:
        width,height=1280//scale,800//scale
        labels=np.zeros((height,width),dtype=np.uint8)
        for tag in frame['tags']:
            inverse=np.linalg.inv(np.array(tag['homography']))
            paper=(np.array(tag['paper_corners'])-[0,phase])/scale
            x0,y0=np.maximum(0,np.floor(paper.min(axis=0)-1)).astype(int)
            x1,y1=np.minimum([width,height],np.ceil(paper.max(axis=0)+1)).astype(int)
            xx=((x0-.5+(np.arange((x1-x0)*8,dtype=np.float32)+.5)/8)*scale)[None,:]
            yy=((y0-.5+(np.arange((y1-y0)*8,dtype=np.float32)+.5)/8)*scale+phase)[:,None]
            den=inverse[2,0]*xx+inverse[2,1]*yy+inverse[2,2]
            u=(inverse[0,0]*xx+inverse[0,1]*yy+inverse[0,2])/den
            v=(inverse[1,0]*xx+inverse[1,1]*yy+inverse[1,2])/den
            inside=(np.abs(u)<=2.2)&(np.abs(v)<=2.2)
            black=(np.abs(u)<=2)&(np.abs(v)<=2)&((np.abs(u)>=5/3)|(np.abs(v)>=5/3))
            for i,(cx,cy) in enumerate(canonical_points(tag['id'])):
                half=1/3 if i<2 else 1/4
                black|=(np.abs(u-cx)<=half)&(np.abs(v-cy)<=half)
            def reduce(mask):return mask.reshape(y1-y0,8,x1-x0,8).mean(axis=(1,3))>=.5
            roi=labels[y0:y1,x0:x1]
            roi[reduce(inside)]=1;roi[reduce(black)]=2
        Image.fromarray(labels).save(root/f'frame_{frame["index"]:03}_{name}.png')
    return frame['index']


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True)
    args=p.parse_args();root=args.dataset/'oracle';root.mkdir(exist_ok=True)
    data=json.loads((args.dataset/'manifest.json').read_text())
    with ThreadPoolExecutor(4) as pool:
        for index in pool.map(lambda f:render(f,root),data['frames']):print(index,flush=True)
