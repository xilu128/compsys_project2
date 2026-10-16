#!/usr/bin/env python3
"""Estimate hard-iron bias and diagonal soft-iron scale from UART rotation log.
Collect full, slow 3D rotations away from steel/electronics; not ordinary walking.
No hardware writes. Review results before copying macros into p2_config.h.
"""
import argparse
import csv
import math
import re
from pathlib import Path
from capture_log import parse_line


def load_samples(path):
    """Accept capture CSV, structured UART or legacy mag=x,y,z logs."""
    samples = []
    with Path(path).open(encoding='utf-8-sig', errors='replace', newline='') as source:
        if Path(path).suffix.lower() == '.csv':
            reader = csv.DictReader(source)
            columns = ['mag_x_mgauss', 'mag_y_mgauss', 'mag_z_mgauss']
            if not set(columns) <= set(reader.fieldnames or []):
                raise ValueError('CSV needs mag_x_mgauss, mag_y_mgauss, mag_z_mgauss')
            for row in reader:
                if row.get('mag_fresh') == '0':
                    continue
                samples.append(tuple(float(row[c]) for c in columns))
        else:
            for line in source:
                if line.startswith('P2DATA,'):
                    row = parse_line(line)
                    if row.get('mag_fresh') != 0:
                        samples.append(tuple(row['mag_'+a+'_mgauss'] for a in 'xyz'))
                else:
                    match = re.search(r'mag=(-?\d+),(-?\d+),(-?\d+)', line)
                    if match:
                        samples.append(tuple(map(float, match.groups())))
    return samples


def solve(matrix, values):
    """Solve the small, normalized least-squares system with pivot checks."""
    n = len(values)
    rows = [list(row) + [value] for row, value in zip(matrix, values)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda i: abs(rows[i][col]))
        rows[col], rows[pivot] = rows[pivot], rows[col]
        if abs(rows[col][col]) < 1e-7:
            raise ValueError("Degenerate 3D coverage; rotate around all three axes")
        divisor = rows[col][col]
        rows[col] = [v/divisor for v in rows[col]]
        for i in range(n):
            if i != col:
                factor = rows[i][col]
                rows[i] = [a-factor*b for a, b in zip(rows[i], rows[col])]
    return [row[-1] for row in rows]


def estimate(samples):
    """Fit an axis-aligned ellipsoid; never infer calibration from two poses.

    Equal-weight spatial cells prevent long stationary holds from dominating.
    The existing firmware supports diagonal scales, not cross-axis correction.
    """
    if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in samples):
        raise ValueError("Calibration samples must contain three finite values")
    if len(samples) < 100:
        raise ValueError("Need at least 100 samples from full 3D rotations")
    lo = [min(v[i] for v in samples) for i in range(3)]
    hi = [max(v[i] for v in samples) for i in range(3)]
    origin = [(h+l)/2 for l, h in zip(lo, hi)]
    span = [(h-l)/2 for l, h in zip(lo, hi)]
    if min(span) < 150 or max(span) > 1000 or max(span)/min(span) > 3:
        raise ValueError("Insufficient axis coverage or magnetic disturbance; repeat away from metal")
    cells = {}
    for sample in samples:
        point = tuple((sample[i]-origin[i])/span[i] for i in range(3))
        key = tuple(math.floor(v*10) for v in point)
        cells.setdefault(key, []).append(point)
    points = [tuple(sum(p[i] for p in cell)/len(cell) for i in range(3))
              for cell in cells.values()]
    if len(points) < 60:
        raise ValueError("Insufficient distinct orientations; stationary holds do not add coverage")
    matrix = [[0.0]*6 for _ in range(6)]
    values = [0.0]*6
    for x, y, z in points:
        basis = (x*x, y*y, z*z, x, y, z)
        for i in range(6):
            values[i] += basis[i]/len(points)
            for j in range(6):
                matrix[i][j] += basis[i]*basis[j]/len(points)
    coefficients = solve(matrix, values)
    quadratic, linear = coefficients[:3], coefficients[3:]
    if min(quadratic) <= 0:
        raise ValueError("Fit is not an ellipsoid; reject this recording")
    centre = [-linear[i]/(2*quadratic[i]) for i in range(3)]
    level = 1 + sum(quadratic[i]*centre[i]**2 for i in range(3))
    radius = [span[i]*math.sqrt(level/quadratic[i]) for i in range(3)]
    bias = [origin[i]+span[i]*centre[i] for i in range(3)]
    if min(radius) < 150 or max(radius) > 1000 or max(radius)/min(radius) > 3:
        raise ValueError("Implausible fitted radius; collect full 3D rotations")
    directions = [tuple((p[i]-centre[i])*span[i]/radius[i] for i in range(3))
                  for p in points]

    octants = {tuple(v >= 0 for v in p) for p in directions}
    if len(octants) < 8 or any(sum(sign*p[i] > .8 for p in directions) < 3
                             for i in range(3) for sign in (-1, 1)):
        raise ValueError("Incomplete 3D coverage; include all six faces and intermediate angles")
    errors = [abs(math.sqrt(sum(v*v for v in p))-1) for p in directions]
    if math.sqrt(sum(e*e for e in errors)/len(errors)) > .08 or max(errors) > .25:
        raise ValueError("Poor diagonal fit; magnetic disturbance or cross-axis distortion")
    average = sum(radius)/3
    scale = [average/r for r in radius]
    lengths = [math.sqrt(sum(((v[i]-bias[i])*scale[i])**2 for i in range(3))) for v in samples]
    mean = sum(lengths)/len(lengths)
    variation = math.sqrt(sum((v-mean)**2 for v in lengths)/len(lengths))/mean
    if variation > .08 or max(abs(v/average-1) for v in lengths) > .25:
        raise ValueError("Inconsistent raw samples; repeat calibration away from interference")
    return bias, scale, variation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    args = parser.parse_args()
    try:
        samples = load_samples(args.log)
        bias, scale, variation = estimate(samples)
    except (ValueError, OSError, TypeError) as error:
        parser.error(str(error))
    for prefix, values in [("BIAS", bias), ("SCALE", scale)]:
        for axis, value in zip("XYZ", values):
            print(f"#define P2_MAG_{prefix}_{axis} {value:.6f}f")
    print(f"// {len(samples)} samples; corrected magnitude CV={variation:.1%}")
    print("// Diagonal fit only; validate against independent known directions.")

if __name__ == "__main__":
    main()
