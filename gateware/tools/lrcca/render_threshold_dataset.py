"""Deterministic natural-background LFTag3x3 threshold benchmark generator.

Pixel coordinates denote pixel centers. Tag size is the projected outer black
border edge length, not its rotated axis-aligned bounding box. Eight-by-eight
area sampling integrates the analytic tag after inverse homography projection.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageOps
from scipy.ndimage import gaussian_filter

WIDTH, HEIGHT, SEED = 1280, 800, 20261106
CORNERS = np.array([[-2, -2], [2, -2], [2, 2], [-2, 2]], dtype=float)
DATA = np.array([[0, -1], [-1, 0], [0, 0], [1, 0], [-1, 1], [0, 1], [1, 1]])


def project(h, points):
    p = np.c_[points, np.ones(len(points))] @ h.T
    return p[:, :2] / p[:, 2:]


def canonical_points(tag_id):
    offsets = np.array([[(1 if tag_id >> (2*i+1) & 1 else -1) / 12,
                         (1 if tag_id >> (2*i) & 1 else -1) / 12] for i in range(7)])
    return np.concatenate(([[-1, -1], [1, -1]], DATA + offsets))


def overlaps(a, b):
    # Separating axis theorem for convex quadrilaterals, including paper margins.
    for polygon in (a, b):
        for edge in np.roll(polygon, -1, axis=0) - polygon:
            axis = np.array([-edge[1], edge[0]])
            pa, pb = a @ axis, b @ axis
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def layout(rng):
    sizes = np.exp(np.linspace(np.log(20), np.log(250), 20))
    ids = rng.choice(16384, 20, replace=False)
    placed = []
    for index in range(19, -1, -1):
        size = sizes[index]
        for attempt in range(10000):
            roll = rng.uniform(-np.pi, np.pi)
            # The 20px endpoint is fronto-parallel so no projected edge is <20.
            pitch, yaw = rng.uniform(-0.85, 0.85, 2) if index else (0., 0.)
            c, s = np.cos, np.sin
            rx = np.array([[1, 0, 0], [0, c(pitch), -s(pitch)], [0, s(pitch), c(pitch)]])
            ry = np.array([[c(yaw), 0, s(yaw)], [0, 1, 0], [-s(yaw), 0, c(yaw)]])
            rz = np.array([[c(roll), -s(roll), 0], [s(roll), c(roll), 0], [0, 0, 1]])
            rot = rz @ ry @ rx
            distance = rng.uniform(10, 24)
            h = np.column_stack((rot[:, 0], rot[:, 1], [0, 0, distance]))
            quad = project(h, CORNERS)
            edges = np.linalg.norm(np.roll(quad, -1, axis=0) - quad, axis=1)
            scale = size / edges.max()
            if edges.min() * scale < 20 - 1e-7:
                continue
            h[:2] *= scale
            paper = project(h, CORNERS * 1.1)
            lo, hi = paper.min(axis=0), paper.max(axis=0)
            center = rng.uniform([6, 6] - lo, [WIDTH-7, HEIGHT-7] - hi)
            h[:2] += center[:, None] * h[2]
            paper = project(h, CORNERS * 1.1)
            # Reserve an extra two pixels around paper for optical blur/noise.
            guard = paper + (paper - paper.mean(axis=0)) * (4 / size)
            if any(overlaps(guard, np.array(tag['guard'])) for tag in placed):
                continue
            quad = project(h, CORNERS)
            placed.append({'id': int(ids[index]), 'size_index': index,
                           'homography': (h / h[2, 2]).tolist(),
                           'corners': quad.tolist(), 'paper_corners': paper.tolist(),
                           'guard': guard.tolist(),
                           'edge_lengths': np.linalg.norm(np.roll(quad, -1, axis=0)-quad, axis=1).tolist(),
                           'centroids': project(h, canonical_points(int(ids[index]))).tolist(),
                           'pose_radians': [float(roll), float(pitch), float(yaw)]})
            break
        else:
            raise RuntimeError(f'Could not place tag {index}')
    return sorted(placed, key=lambda item: item['size_index'])


def render_tag(frame, illumination, tag, rng):
    h = np.array(tag['homography'])
    inverse = np.linalg.inv(h)
    paper = np.array(tag['paper_corners'])
    x0, y0 = np.floor(paper.min(axis=0) - 1).astype(int)
    x1, y1 = np.ceil(paper.max(axis=0) + 1).astype(int)
    x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, WIDTH), min(y1, HEIGHT)
    # Center each subpixel in [-0.5,+0.5) about an integer pixel center.
    xx = (x0 - .5 + (np.arange((x1-x0)*8, dtype=np.float32)+.5)/8)[None, :]
    yy = (y0 - .5 + (np.arange((y1-y0)*8, dtype=np.float32)+.5)/8)[:, None]
    den = inverse[2, 0]*xx + inverse[2, 1]*yy + inverse[2, 2]
    u = (inverse[0, 0]*xx + inverse[0, 1]*yy + inverse[0, 2]) / den
    v = (inverse[1, 0]*xx + inverse[1, 1]*yy + inverse[1, 2]) / den
    inside = (np.abs(u) <= 2.2) & (np.abs(v) <= 2.2)
    black = (np.abs(u) <= 2) & (np.abs(v) <= 2) & ((np.abs(u) >= 5/3) | (np.abs(v) >= 5/3))
    for i, (cx, cy) in enumerate(canonical_points(tag['id'])):
        half = 1/3 if i < 2 else 1/4
        black |= (np.abs(u-cx) <= half) & (np.abs(v-cy) <= half)
    def reduce(values):
        return values.reshape(y1-y0, 8, x1-x0, 8).mean(axis=(1, 3))
    coverage, ink = reduce(inside), reduce(black)
    local = illumination[y0:y1, x0:x1]
    brightness = rng.uniform(.85, 1.15)
    white = np.clip((.72*local + 62)*brightness, 35, 248)
    black_ratio = rng.uniform(.035, .22)
    gradient = rng.uniform(-.16, .16, 2)
    py, px = np.mgrid[y0:y1, x0:x1]
    center = paper.mean(axis=0)
    length = max(tag['edge_lengths'])
    shading = 1 + gradient[0]*(px-center[0])/length + gradient[1]*(py-center[1])/length
    paper_value = np.clip(white*shading, 0, 255)
    original = frame[y0:y1, x0:x1]
    original[:] = original*(1-coverage) + paper_value*(coverage-ink + black_ratio*ink)
    tag['lighting'] = {'white_mean': float(paper_value[coverage > .99].mean()),
                       'black_ratio': float(black_ratio), 'gradient': gradient.tolist(),
                       'brightness_factor': float(brightness)}


def generate(backgrounds, output, workers, seed=SEED, split=None):
    source = json.loads((backgrounds/'manifest.json').read_text())
    assert len(source['images']) == 100
    output.mkdir(parents=True, exist_ok=True)
    def one(pair):
        index, background = pair
        rng = np.random.default_rng(np.random.SeedSequence([seed, index]))
        with Image.open(backgrounds/background['file']) as source_image:
            natural = ImageOps.fit(source_image.convert('L'), (WIDTH, HEIGHT), method=Image.Resampling.LANCZOS)
        frame = np.asarray(natural, dtype=np.float32).copy()
        illumination = gaussian_filter(frame, 18)
        tags = layout(rng)
        for tag in tags:
            render_tag(frame, illumination, tag, rng)
        # Mild optical PSF after area sampling, then camera-like noise globally.
        blur_sigma = float(rng.uniform(.25, .55))
        frame = gaussian_filter(frame, blur_sigma)
        read_sigma, shot_factor = float(rng.uniform(.4, 2)), float(rng.uniform(.005, .055))
        frame += rng.standard_normal(frame.shape).astype(np.float32) * np.sqrt(read_sigma**2 + shot_factor*frame.clip(0))
        pixels = np.clip(np.rint(frame), 0, 255).astype(np.uint8)
        name = f'frame_{index:03}.png'
        Image.fromarray(pixels).save(output/name)
        record = {'index': index, 'file': name, 'split': split or ('train' if index < 70 else 'validation' if index < 85 else 'test'),
                  'background': background, 'tags': tags,
                  'noise': {'read_sigma': read_sigma, 'shot_variance_per_dn': shot_factor},
                  'optical_blur_sigma': blur_sigma,
                  'sha256': hashlib.sha256((output/name).read_bytes()).hexdigest()}
        (output/f'frame_{index:03}.json').write_text(json.dumps(record, indent=2)+'\n')
        print(f'rendered {index:03}', flush=True)
        return record
    with ThreadPoolExecutor(max_workers=workers) as executor:
        frames = list(executor.map(one, enumerate(source['images'])))
    audit = {'frames': len(frames), 'tags': sum(len(f['tags']) for f in frames),
             'distinct_backgrounds': len({f['background']['coco_id'] for f in frames}),
             'overlaps': 0, 'out_of_bounds': 0,
             'minimum_edge_px': min(min(t['edge_lengths']) for f in frames for t in f['tags']),
             'maximum_edge_px': max(max(t['edge_lengths']) for f in frames for t in f['tags'])}
    for frame in frames:
        assert len(frame['tags']) == len({t['id'] for t in frame['tags']}) == 20
        for i, tag in enumerate(frame['tags']):
            p = np.array(tag['guard'])
            audit['out_of_bounds'] += int((p < 0).any() or (p >= [WIDTH, HEIGHT]).any())
            for other in frame['tags'][:i]:
                audit['overlaps'] += int(overlaps(p, np.array(other['guard'])))
    assert audit['frames'] == 100 and audit['tags'] == 2000 and audit['distinct_backgrounds'] == 100
    assert audit['overlaps'] == audit['out_of_bounds'] == 0
    assert audit['minimum_edge_px'] >= 20-1e-7 and audit['maximum_edge_px'] <= 250+1e-7
    manifest = {'version': 1, 'seed': seed, 'width': WIDTH, 'height': HEIGHT,
                'background_resize': 'aspect-preserving center crop, Lanczos to 1280x800',
                'tag_geometry': 'LFTag3x3 24-unit original geometry; 8x8 analytic projective area sampling',
                'size_definition': 'outer black border projected edge lengths, all in [20,250] pixels',
                'generator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'audit': audit, 'frames': frames}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (output/'audit.json').write_text(json.dumps(audit, indent=2)+'\n')
    sheet = Image.new('RGB', (4*320, 3*220), '#202020')
    draw = ImageDraw.Draw(sheet)
    for j, index in enumerate([0, 1, 2, 3, 20, 40, 60, 70, 75, 85, 90, 99]):
        with Image.open(output/f'frame_{index:03}.png') as im:
            sheet.paste(im.resize((320, 200)), ((j % 4)*320, (j//4)*220))
        draw.text(((j % 4)*320+6, (j//4)*220+202), f'{index:03}  {frames[index]["split"]}', fill='white')
    sheet.save(output/'contact-sheet.jpg')
    print(json.dumps(audit), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backgrounds', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--workers', default=4, type=int)
    parser.add_argument('--seed', default=SEED, type=int)
    parser.add_argument('--split', choices=['train', 'validation', 'test', 'holdout'])
    args = parser.parse_args()
    generate(args.backgrounds, args.output, args.workers, args.seed, args.split)
