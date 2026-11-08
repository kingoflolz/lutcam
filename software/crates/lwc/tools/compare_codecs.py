#!/usr/bin/env python3
"""Reproduce the 1280x800 grayscale, <=1 bpp codec comparison."""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess
import tempfile

import numpy as np
from PIL import Image, features
from scipy.ndimage import gaussian_filter

WIDTH, HEIGHT, BUDGET = 1280, 800, 128000
JPEG_TABLE = np.array([
    16,11,10,16,24,40,51,61,12,12,14,19,26,58,60,55,
    14,13,16,24,40,57,69,56,14,17,22,29,51,87,80,62,
    18,22,37,56,68,109,103,77,24,35,55,64,81,104,113,92,
    49,64,78,87,103,121,120,101,72,92,95,98,112,100,103,99,
], dtype=float)
# Component-zero entries of XSM's built-in 5H/2V, three-component, no-color-
# transform PSNR table. Priority values are rank-compressed to one component.
# Chosen once from the reference implementation, never fitted to these images.
JXS_CONFIG = (
    "p=high444.12;size=128000;nlx=5;nly=2;slh=16;"
    "gains=3,2,2,1,1,1,0,0,0,0;priorities=4,3,8,0,6,7,5,2,1,9"
)


def run(args):
    result = subprocess.run([str(x) for x in args], capture_output=True, check=True)
    return result.stdout, result.stderr


def pixels(image):
    with Image.open(image) as im:
        assert im.size == (WIDTH, HEIGHT)
        return np.array(im.convert("L"))


def metrics(source, reconstructed):
    assert source.shape == reconstructed.shape == (HEIGHT, WIDTH)
    x, y = source.astype(np.float64), reconstructed.astype(np.float64)
    error = x - y
    sse = int(np.sum(error * error))
    mse = sse / source.size
    # Wang SSIM: Gaussian 11x11, sigma=1.5, population covariance, valid region.
    def blur(a):
        return gaussian_filter(a, sigma=1.5, truncate=3.5, mode="reflect")
    ux, uy = blur(x), blur(y)
    vx, vy, cov = blur(x*x)-ux*ux, blur(y*y)-uy*uy, blur(x*y)-ux*uy
    c1, c2 = (0.01*255)**2, (0.03*255)**2
    ssim = ((2*ux*uy+c1)*(2*cov+c2))/((ux*ux+uy*uy+c1)*(vx+vy+c2))
    return {"sse": sse, "mse": mse, "psnr_db": 10*math.log10(255**2/mse),
            "ssim": float(ssim[5:-5,5:-5].mean()), "mae": float(np.abs(error).mean())}


def jpeg_encode(source, destination):
    """Conventional integer quality, then refine just its scalar table scale."""
    fixed = ["cjpeg", "-strict", "-noicc", "-grayscale", "-baseline", "-dct", "int", "-optimize"]
    trials = []
    best = None
    for q in range(100, 0, -1):
        data, _ = run(fixed + ["-quality", str(q), source])
        trials.append({"quality": q, "bytes": len(data)})
        if len(data) <= BUDGET:
            best = data
            break
    if best is None:
        raise RuntimeError("JPEG cannot meet budget")
    integer_quality, integer_bytes = q, len(best)
    scale = (5000 // q) if q < 50 else (200 - 2*q)
    best_scale = scale
    table = np.array(Image.open(io.BytesIO(best)).quantization[0])
    # Enumerate every distinct standard-table scaling state between the highest
    # fitting integer quality and the next integer quality. Do not assume size
    # monotonicity within this interval. This removes most integer-Q underfill.
    if q < 100:
        lower = (5000 // (q+1)) if q+1 < 50 else (200 - 2*(q+1))
        breaks = {float(lower), float(scale)}
        for base in JPEG_TABLE:
            for n in range(1, 256):
                threshold = (n - 0.5) * 100 / base
                if lower < threshold < scale:
                    breaks.add(threshold)
        breaks = sorted(breaks)
        scales = [(a+b)/2 for a,b in zip(breaks,breaks[1:])]
        best_error = math.inf
        source_pixels = pixels(source).astype(np.int32)
        seen = {tuple(table)}
        with tempfile.TemporaryDirectory(prefix="jpeg-qt-") as temp:
            qfile = Path(temp)/"qtable.txt"
            for s in scales:
                candidate_table = np.clip(np.floor(JPEG_TABLE*s/100+0.5), 1, 255).astype(int)
                if tuple(candidate_table) in seen:
                    continue
                seen.add(tuple(candidate_table))
                qfile.write_text("\n".join(" ".join(map(str,row)) for row in candidate_table.reshape(8,8))+"\n")
                data, _ = run(fixed + ["-quality", "50", "-qtables", qfile, source])
                trials.append({"table_scale_percent": s, "bytes": len(data)})
                if len(data) <= BUDGET:
                    decoded = np.array(Image.open(io.BytesIO(data))).astype(np.int32)
                    error = int(np.sum((decoded-source_pixels).astype(np.int64)**2))
                    if error < best_error:
                        best_error, best, best_scale, table = error, data, s, candidate_table
        # Include the unrefined integer-quality candidate in quality selection.
        integer, _ = run(fixed + ["-quality", str(q), source])
        decoded = np.array(Image.open(io.BytesIO(integer))).astype(np.int32)
        integer_error = int(np.sum((decoded-source_pixels).astype(np.int64)**2))
        if integer_error <= best_error:
            best, best_scale = integer, scale
            table = np.array(Image.open(io.BytesIO(best)).quantization[0])
    destination.write_bytes(best)
    qtable_path = destination.with_suffix(".qtable.txt")
    qtable_path.write_text("\n".join(" ".join(map(str,row)) for row in table.reshape(8,8))+"\n")
    reconstruction, _ = run(["djpeg", "-strict", "-grayscale", "-dct", "int", "-pnm", destination])
    image = Image.open(io.BytesIO(reconstruction))
    image.save(destination.with_suffix(".png"))
    return {"integer_quality": integer_quality, "integer_quality_bytes": integer_bytes,
            "table_scale_percent": float(best_scale), "search_trials": trials}


def webp_encode(source, destination):
    # Pillow's libwebp path imports grayscale as RGB with equal channels, then
    # uses WebP's normal color conversion. All RIFF/chroma overhead is counted.
    image = Image.open(source).convert("L")
    trials, candidates = [], []
    def candidate(q):
        buf = io.BytesIO()
        image.save(buf, format="WEBP", quality=float(q), method=6, lossless=False)
        data = buf.getvalue()
        trials.append({"quality": q, "bytes": len(data)})
        if len(data) <= BUDGET:
            candidates.append((q, data))
        return len(data)
    low, high = 0., 100.
    candidate(low)
    if candidate(high) > BUDGET:
        for _ in range(16):
            mid = (low+high)/2
            if candidate(mid) <= BUDGET:
                low = mid
            else:
                high = mid
        # Check nearby points, since entropy coding does not ensure monotonic
        # file size or pixel error as quality varies.
        for q in np.linspace(max(0, low-.2), min(100, high+.2), 9):
            candidate(float(q))
    if not candidates:
        raise RuntimeError("WebP cannot meet budget")
    original = np.array(image).astype(np.int32)
    def sse(item):
        decoded = np.array(Image.open(io.BytesIO(item[1])).convert("L")).astype(np.int32)
        return int(np.sum((original-decoded).astype(np.int64)**2))
    q, data = min(candidates, key=sse)
    destination.write_bytes(data)
    Image.open(io.BytesIO(data)).convert("L").save(destination.with_suffix(".png"))
    return {"quality": q, "method": 6, "search_trials": trials}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--codecs", nargs="+", default=["LWC","JPEG","JPEG_XS","WebP"])
    parser.add_argument("--jxs-bin", type=Path)
    parser.add_argument("--svt-decoder", type=Path)
    parser.add_argument("--lwc-bin", type=Path)
    parser.add_argument("--resume", action="store_true", help="Reuse completed, hash-verified measurements")
    args = parser.parse_args()
    root = args.output.resolve()
    selection = json.loads((root/"selection.json").read_text())
    for folder in ["source", "logs", "LWC"]:
        (root/folder).mkdir(exist_ok=True)
    for item in selection["images"]:
        with Image.open(root/"original"/f"{item['id']}.png") as image:
            w,h = image.size
            assert w >= WIDTH and h >= HEIGHT
            x,y = (w-WIDTH)//2, (h-HEIGHT)//2
            gray = image.convert("L").crop((x,y,x+WIDTH,y+HEIGHT))
            gray.save(root/"source"/f"{item['id']}.png")
            gray.save(root/"source"/f"{item['id']}.pgm")
            item["crop_xywh"] = [x,y,WIDTH,HEIGHT]
    (root/"selection.json").write_text(json.dumps(selection,indent=2)+"\n")
    if args.lwc_bin and "LWC" in args.codecs:
        stdout, _ = run([args.lwc_bin, root/"source", root/"LWC"])
        (root/"logs/lwc.csv").write_bytes(stdout)
    lwc_data = {r["image"]: r for r in csv.DictReader((root/"logs/lwc.csv").open())}
    for codec in args.codecs:
        rows = []
        old_path = root/codec/"metrics.json"
        saved = {r["image"]:r for r in json.loads(old_path.read_text())} if args.resume and old_path.exists() else {}
        for item in selection["images"]:
            stem = item["id"]
            source = root/"source"/f"{stem}.png"
            directory = root/codec
            directory.mkdir(exist_ok=True)
            if stem in saved:
                old = saved[stem]
                extension = {"LWC":"lwc", "JPEG":"jpg", "JPEG_XS":"jxs", "WebP":"webp"}[codec]
                for path,key in [(source,"source_sha256"), (directory/f"{stem}.{extension}","encoded_sha256"), (directory/f"{stem}.png","reconstruction_sha256")]:
                    assert hashlib.sha256(path.read_bytes()).hexdigest() == old[key], path
                rows.append(old)
                continue
            extra = {}
            if codec == "JPEG":
                encoded = directory/f"{stem}.jpg"
                extra = jpeg_encode(source, encoded)
            elif codec == "WebP":
                encoded = directory/f"{stem}.webp"
                extra = webp_encode(source, encoded)
            elif codec == "JPEG_XS":
                encoded = directory/f"{stem}.jxs"
                stdout, stderr = run([args.jxs_bin/"jxs_encoder", "-q", "-c", JXS_CONFIG,
                                      source.with_suffix(".pgm"), encoded])
                (root/"logs"/f"{stem}-jxs-encode.txt").write_bytes(stdout+stderr)
                assert encoded.stat().st_size == BUDGET
                pgm = directory/f"{stem}.pgm"
                stdout, stderr = run([args.jxs_bin/"jxs_decoder", "-q", encoded, pgm])
                (root/"logs"/f"{stem}-jxs-decode.txt").write_bytes(stdout+stderr)
                Image.open(pgm).save(directory/f"{stem}.png")
                extra = {"config": JXS_CONFIG, "components": 1, "bit_depth": 8}
                if args.svt_decoder:
                    raw = directory/f"{stem}.gray"
                    stdout, stderr = run([args.svt_decoder, "-i", encoded, "-o", raw, "-n", "1", "-v", "1"])
                    (root/"logs"/f"{stem}-jxs-independent-decode.txt").write_bytes(stdout+stderr)
                    assert raw.read_bytes() == Image.open(pgm).tobytes()
                    extra["independent_svt_decode_identical"] = True
            elif codec == "LWC":
                encoded = directory/f"{stem}.lwc"
                extra = {k:int(v) for k,v in lwc_data[stem].items() if k != "image"}
                assert encoded.stat().st_size == BUDGET
            else:
                raise ValueError(codec)
            size = encoded.stat().st_size
            assert 0 < size <= BUDGET
            row = {"image":stem, "codec":codec, "bytes":size, "bpp":size*8/(WIDTH*HEIGHT),
                   **metrics(pixels(source), pixels(directory/f"{stem}.png")), **extra,
                   "encoded_sha256":hashlib.sha256(encoded.read_bytes()).hexdigest(),
                   "source_sha256":hashlib.sha256(source.read_bytes()).hexdigest(),
                   "reconstruction_sha256":hashlib.sha256((directory/f"{stem}.png").read_bytes()).hexdigest()}
            rows.append(row)
            (directory/"metrics.json").write_text(json.dumps(rows,indent=2)+"\n")
            print(f"{codec:8} {stem} {size:6} bytes {row['bpp']:.6f} bpp {row['psnr_db']:.3f} dB SSIM {row['ssim']:.5f}", flush=True)
    print("Pillow/libwebp/libjpeg-turbo:", Image.__version__, features.version("webp"), features.version("libjpeg_turbo"))


if __name__ == "__main__":
    main()
