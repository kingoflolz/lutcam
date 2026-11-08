"""Add per-level coverage and centroid localization metrics to bench results.

Theoretical centers are the nine canonical LFTag dot centers transformed by
the generating homography. An optimal bijection pairs detected and theoretical
centers. All coordinates/errors use source-image pixels; normalized errors use
one tag grid pitch = maximum projected outer edge / 4. Ground truth never
enters detection, threshold selection within an image, or the fixed decoder.
"""

import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment

LEVELS = ['Raw', 'Ll2', 'Ll4']


def summarize(records):
    if not records:
        return {'dots': 0, 'tags': 0, 'rms_px': None, 'p95_px': None,
                'median_px': None, 'rms_grid_pitch': None, 'tag_center_rms_px': None}
    errors = np.concatenate([r['errors_px'] for r in records])
    normalized = np.concatenate([r['errors_grid_pitch'] for r in records])
    return {'dots': len(errors), 'tags': len(records),
            'rms_px': float(np.sqrt(np.mean(errors**2))),
            'median_px': float(np.median(errors)), 'p95_px': float(np.quantile(errors, .95)),
            'rms_grid_pitch': float(np.sqrt(np.mean(normalized**2))),
            'tag_center_rms_px': float(np.sqrt(np.mean([r['tag_center_error_px']**2 for r in records])))}


def annotate(result_path, dataset):
    result = json.loads(result_path.read_text())
    all_errors = {key: {level: [] for level in LEVELS} for key in result['totals']}
    for frame in result['frames']:
        gt = {t['id']: t for t in dataset[frame['index']]['tags']}
        sets = [set() for _ in LEVELS]
        for detection in frame['detections']:
            if detection['matched_id'] is None:
                continue
            level = detection['plane']
            sets[LEVELS.index(level)].add(detection['matched_id'])
            tag = gt[detection['matched_id']]
            points, theoretical = np.array(detection['points']), np.array(tag['centroids'])
            cost = ((points[:, None]-theoretical[None, :])**2).sum(axis=2)
            rows, cols = linear_sum_assignment(cost)
            errors = np.sqrt(cost[rows, cols])
            error_record = {'errors_px': errors.tolist(),
                            'errors_grid_pitch': (errors/(max(tag['edge_lengths'])/4)).tolist(),
                            'theoretical_indices': cols.tolist(),
                            'rms_px': float(np.sqrt(np.mean(errors**2))),
                            'tag_center_error_px': float(np.linalg.norm(points.mean(axis=0)-theoretical.mean(axis=0)))}
            detection['centroid_error'] = error_record
            for key in ['all', frame['split']]:
                all_errors[key][level].append(error_record)
        frame['level_tp'] = list(map(len, sets))
        frame['agreement'] = [0]*3
        for tag in set.union(*sets):
            frame['agreement'][sum(tag in level for level in sets)-1] += 1
        frame['multi_level_tags'] = sum(frame['agreement'][1:])
    for split, totals in result['totals'].items():
        frames = [f for f in result['frames'] if split == 'all' or f['split'] == split]
        for key in ['level_tp', 'agreement']:
            totals[key] = [sum(f[key][i] for f in frames) for i in range(3)]
        totals['multi_level_tags'] = sum(totals['agreement'][1:])
        totals['centroid_error'] = {level: summarize(all_errors[split][level]) for level in LEVELS}
        totals['centroid_error']['all'] = summarize(sum(all_errors[split].values(), []))
    result['levels'] = LEVELS
    result['centroid_ground_truth'] = 'homography-projected canonical dot centers; optimal bijection; pixel-center coordinates'
    result_path.write_text(json.dumps(result, indent=2)+'\n')
    t = result['totals']['all']
    return {'name': result['config']['name'], 'tp':t['tp'], 'fp':t['fp'], 'gt':t['gt'],
            'levels':t['level_tp'], 'multi_level':t['multi_level_tags'],
            'centroid_rms_px':t['centroid_error']['all']['rms_px'],
            'centroid_rms_grid_pitch':t['centroid_error']['all']['rms_grid_pitch']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True, type=Path)
    parser.add_argument('results', nargs='+', type=Path)
    args = parser.parse_args()
    dataset = {f['index']:f for f in json.loads(args.dataset.read_text())['frames']}
    summaries = [annotate(path, dataset) for path in args.results]
    for row in sorted(summaries, key=lambda r:(-r['tp'], -r['multi_level'], r['centroid_rms_grid_pitch'] or 0)):
        print(json.dumps(row))
