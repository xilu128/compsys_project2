"""Fit provisional XY bias from a near-horizontal rotation (not 3D calibration).

No hardware writes. Use a separate recording to validate the fitted centre.
Only read the explicitly supplied input. In the October 5 fix, use trial 2,
first 33 seconds; trial 3 is reserved for validation, trial 1 is excluded.
"""
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import statistics


def solve3(matrix, values):
    a = [list(row) + [value] for row, value in zip(matrix, values)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda i: abs(a[i][col]))
        a[col], a[pivot] = a[pivot], a[col]
        if abs(a[col][col]) < 1e-8:
            raise ValueError('Degenerate rotation: cannot fit XY circle')
        scale = a[col][col]
        a[col] = [v / scale for v in a[col]]
        for row in range(3):
            if row != col:
                scale = a[row][col]
                a[row] = [x - scale*y for x, y in zip(a[row], a[col])]
    return [a[i][3] for i in range(3)]


def fit(path, seconds):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('Empty recording')
    start = datetime.fromisoformat(rows[0]['host_time_iso'])
    points, tilts = [], []
    for row in rows:
        elapsed = (datetime.fromisoformat(row['host_time_iso']) - start).total_seconds()
        if not 0 <= elapsed < seconds:
            continue
        if row.get('acc_fresh') == '0' or row.get('mag_fresh') == '0':
            continue
        ax, ay, az = [float(row[f'acc_{a}_mg']) for a in 'xyz']
        x, y = [float(row[f'mag_{a}_mgauss']) for a in 'xy']
        if not all(math.isfinite(v) for v in (ax, ay, az, x, y)):
            raise ValueError('Non-finite input')
        tilts.append(math.degrees(math.atan2(math.hypot(ax, ay), az)))
        points.append((x, y))
    if len(points) < 100 or max(tilts) > 15:
        raise ValueError('Need >=100 fresh samples with +Z up and tilt <=15 degrees')
    m = [[0.0]*3 for _ in range(3)]
    v = [0.0]*3
    for x, y in points:
        a, b = (2*x, 2*y, 1), x*x + y*y
        for i in range(3):
            v[i] += a[i]*b
            for j in range(3):
                m[i][j] += a[i]*a[j]
    bx, by, c = solve3(m, v)
    radius = math.sqrt(c + bx*bx + by*by)
    radii = [math.hypot(x-bx, y-by) for x,y in points]
    angles = sorted(math.degrees(math.atan2(y-by, x-bx)) % 360 for x,y in points)
    max_gap = max(b-a for a,b in zip(angles, angles[1:]+[angles[0]+360]))
    if max_gap > 90 or radius < 100 or statistics.pstdev(radii)/radius > .15:
        raise ValueError('Insufficient XY coverage or inconsistent magnetic field')
    return dict(file=path.name, seconds=seconds, samples=len(points),
                bias_x_mgauss=bx, bias_y_mgauss=by, radius_mgauss=radius,
                radial_std_mgauss=statistics.pstdev(radii),
                coverage_deg=360-max_gap, max_tilt_deg=max(tilts),
                calibration='XY_PROVISIONAL',
                limitation='Z bias, full soft-iron distortion and absolute north are not established.')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv', type=Path)
    p.add_argument('--seconds', type=float, default=33)
    p.add_argument('--output', type=Path)
    args = p.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        p.error('seconds must be positive and finite')
    result = fit(args.csv, args.seconds)
    text = json.dumps(result, indent=2)
    print(text)
    if args.output:
        with args.output.open('x', encoding='utf-8') as stream:
            stream.write(text + '\n')
