#!/usr/bin/env python3
"""Build measured comparison reports; never re-encode the benchmark results."""
import argparse
import base64
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import findfont
import numpy as np
from PIL import Image, ImageDraw, ImageFont, features

from compare_codecs import JXS_CONFIG

LABELS = {"0802":"Fruit", "0803":"Plant", "0811":"Rocky coast", "0816":"Carved stone",
          "0826":"Ornate gate", "0839":"Mushroom", "0841":"Library", "0861":"Bridge",
          "0877":"White flowers", "0884":"Grand hall", "0893":"Hat and leaves", "0898":"Blossoms"}
CODECS = ["LWC","JPEG","JPEG_XS","WebP"]
NAMES = {"source":"Original", "LWC":"LWC", "JPEG":"JPEG", "JPEG_XS":"JPEG XS", "WebP":"WebP"}
EXCLUDED = {"0826","0861"}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def data_url(image):
    buf=io.BytesIO()
    image.save(buf,format="WEBP",lossless=True,method=6)
    raw=buf.getvalue()
    assert np.array_equal(np.asarray(Image.open(io.BytesIO(raw)).convert("L")),np.asarray(image.convert("L")))
    return "data:image/webp;base64,"+base64.b64encode(raw).decode()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("output",type=Path)
    p.add_argument("--inline",type=Path,required=True)
    p.add_argument("--jpeg-xs-source", type=Path, help="reference source archive for provenance")
    args=p.parse_args()
    root=args.output.resolve();tools=Path(__file__).resolve().parent
    selection=json.loads((root/"selection.json").read_text())
    raw={c:json.loads((root/c/"metrics.json").read_text()) for c in CODECS}
    assert all(len(rows)==12 for rows in raw.values())
    indices={c:{r["image"]:r for r in rows} for c,rows in raw.items()}
    # Audit source identity, decoded shape and all encoded-file budgets.
    for c,rows in raw.items():
        ext={"LWC":"lwc","JPEG":"jpg","JPEG_XS":"jxs","WebP":"webp"}[c]
        for r in rows:
            stem=r['image'];encoded=root/c/f'{stem}.{ext}';decoded=root/c/f'{stem}.png';source=root/'source'/f'{stem}.png'
            assert 0<encoded.stat().st_size==r['bytes']<=128000
            assert sha(encoded)==r['encoded_sha256'] and sha(decoded)==r['reconstruction_sha256'] and sha(source)==r['source_sha256']
            with Image.open(decoded) as im: assert im.size==(1280,800) and im.mode=='L'
            a=np.asarray(Image.open(source)).astype(np.int64);b=np.asarray(Image.open(decoded)).astype(np.int64)
            assert int(np.sum((a-b)**2))==r['sse']
    included=[i for i in selection['images'] if i['id'] not in EXCLUDED]
    summary={}
    for c in CODECS:
        rows=[indices[c][im['id']] for im in included]
        sse=sum(r['sse'] for r in rows);pixels=1280*800*len(rows)
        summary[c]={"images":len(rows),"pooled_psnr_db":10*math.log10(65025*pixels/sse),
                    "mean_psnr_db":sum(r['psnr_db'] for r in rows)/len(rows),
                    "mean_ssim":sum(r['ssim'] for r in rows)/len(rows),
                    "bpp":sum(r['bytes'] for r in rows)*8/pixels,"total_sse":sse,
                    "mean_bytes":sum(r['bytes'] for r in rows)/len(rows)}
    images=[{**im,"label":LABELS[im['id']],"metrics":{c:indices[c][im['id']] for c in CODECS}} for im in included]
    wins={c:sum(min(CODECS,key=lambda c:im['metrics'][c]['sse'])==c for im in images) for c in CODECS}
    finding=(f"On this filtered subset, WebP has the highest PSNR on {wins['WebP']} of 10 images. "
             f"Pooled PSNR: WebP {summary['WebP']['pooled_psnr_db']:.2f} dB; JPEG {summary['JPEG']['pooled_psnr_db']:.2f} dB; "
             f"LWC {summary['LWC']['pooled_psnr_db']:.2f} dB; JPEG XS {summary['JPEG_XS']['pooled_psnr_db']:.2f} dB. "
             f"LWC is {summary['JPEG']['pooled_psnr_db']-summary['LWC']['pooled_psnr_db']:.2f} dB below JPEG "
             f"and {summary['LWC']['pooled_psnr_db']-summary['JPEG_XS']['pooled_psnr_db']:.2f} dB above this JPEG XS reference configuration.")
    data={"summary":summary,"images":images,"excluded":sorted(EXCLUDED),"finding":finding,"jxs_config":JXS_CONFIG,
          "selection_note":"User-requested exclusion of 0826 and 0861 for LWC budget exhaustion; every codec uses the same ten-image subset."}
    (root/'metrics.json').write_text(json.dumps(data,indent=2)+'\n')
    (root/'metrics-all-12.json').write_text(json.dumps(raw,indent=2)+'\n')
    with (root/'metrics.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=['image','codec','bytes','bpp','sse','mse','psnr_db','ssim','mae'],extrasaction='ignore');writer.writeheader()
        writer.writerows(im['metrics'][c] for im in images for c in CODECS)
    (root/'comparison.html').write_text((tools/'comparison_report.template.html').read_text().replace('__REPORT_DATA__',json.dumps(data,separators=(',',':'))))
    figures=root/'figures';figures.mkdir(exist_ok=True)
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    fig,ax=plt.subplots(figsize=(12,4.2),layout='constrained')
    for c,color,marker in zip(CODECS,['#166b8a','#bb6333','#7257a1','#278c59'],['o','s','^','D']):
        ax.plot([im['id'] for im in images],[im['metrics'][c]['psnr_db'] for im in images],label=NAMES[c],color=color,marker=marker,linewidth=1.7,markersize=5)
    ax.set(xlabel='DIV2K validation image',ylabel='PSNR (dB)',title='Same ten images · 1280 × 800 grayscale · ≤ 1 bpp')
    ax.grid(axis='y',alpha=.2);ax.legend(ncol=4,loc='upper left');fig.savefig(figures/'psnr.png',dpi=160);plt.close(fig)
    # A full-frame contact sheet, with no extra lossy compression.
    font=ImageFont.truetype(findfont('DejaVu Sans'),15)
    small=ImageFont.truetype(findfont('DejaVu Sans'),12)
    sheet=Image.new('RGB',(5*272+16,len(images)*210+48),'#fafafa');draw=ImageDraw.Draw(sheet)
    draw.text((16,12),'DIV2K · 10 retained images · 1280 × 800 grayscale · ≤ 1 bpp',fill='#17212a',font=font)
    for row,im in enumerate(images):
        for col,c in enumerate(['source',*CODECS]):
            x=16+col*272;y=48+row*210
            draw.text((x,y),f"{im['id']} · {NAMES[c]}",fill='#17212a',font=font)
            original=Image.open(root/c/f"{im['id']}.png")
            sheet.paste(original.resize((256,160),Image.Resampling.LANCZOS),(x,y+22))
            caption=LABELS[im['id']] if c=='source' else f"{im['metrics'][c]['psnr_db']:.2f} dB / {im['metrics'][c]['bpp']:.4f} bpp"
            draw.text((x,y+185),caption,fill='#48545e',font=small)
    sheet.save(figures/'contact-sheet.png')
    # Inline views carry exact lossless crops, plus a downsampled locator image.
    for size in [112,96,80]:
        locations={'center':[(1280-size)//2,(800-size)//2], 'bottom':[(1280-size)//2,800-size-24]}
        inline={'summary':summary,'crop_size':size,'locations':locations,'images':[]}
        for im in images:
            item={k:im[k] for k in ['id','label','width','height']}
            item['metrics']={c:{'psnr_db':im['metrics'][c]['psnr_db'],'ssim':im['metrics'][c]['ssim']} for c in CODECS}
            item['overview']=data_url(Image.open(root/'source'/f"{im['id']}.png").resize((256,160),Image.Resampling.LANCZOS))
            item['crops']={loc:{} for loc in locations}
            for c in ['source',*CODECS]:
                image=Image.open(root/c/f"{im['id']}.png")
                for loc,(x,y) in locations.items():item['crops'][loc][c]=data_url(image.crop((x,y,x+size,y+size)))
            inline['images'].append(item)
        fragment=(tools/'comparison_inline.template.html').read_text().replace('__INLINE_DATA__',json.dumps(inline,separators=(',',':')))
        if len(fragment.encode())<980000:break
    assert len(fragment.encode())<1000000
    args.inline.parent.mkdir(parents=True,exist_ok=True);args.inline.write_text(fragment)
    provenance={"platform":platform.platform(),"python":platform.python_version(),"pillow":Image.__version__,
                "webp":features.version('webp'),"pillow_libjpeg_turbo":features.version('libjpeg_turbo'),
                "cjpeg":subprocess.run(['cjpeg','-version'],capture_output=True,text=True).stderr.strip(),
                "jpeg_xs":"ISO/IEC 21122-5 ED3, XSM 3.0.2-ED3, source unmodified",
                "jpeg_xs_source_url":"https://standards.iso.org/iso-iec/21122/-5/ed-3/en/ISO_IEC_21122-5_ED-3.zip",
                "independent_decoder":"SVT-JPEG-XS 4d7e116c3ef259529a0917060e5b113834ed4894; all 12 decoded frames bit-exact with XSM",
                "lwc_source_sha256":{p.name:sha(p) for p in (tools.parent/'src').glob('*.rs')},
                "included_images":[im['id'] for im in images],"excluded_images":sorted(EXCLUDED)}
    archive=args.jpeg_xs_source
    if archive is not None and archive.is_file():provenance['jpeg_xs_source_sha256']=sha(archive)
    (root/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(json.dumps(summary,indent=2));print(finding)
    print('Inline bytes:',len(fragment.encode()),'crop:',size,'px. All encoded sizes, hashes, dimensions and SSE validated.')


if __name__=='__main__':main()
