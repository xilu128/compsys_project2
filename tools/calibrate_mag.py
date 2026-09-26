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


def estimate(samples):
    if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in samples):
        raise ValueError("Calibration samples must contain three finite values")
    if len(samples) < 100:
        raise ValueError("Need at least 100 samples from full 3D rotations")
    lo = [min(v[i] for v in samples) for i in range(3)]
    hi = [max(v[i] for v in samples) for i in range(3)]
    radius = [(h-l)/2 for l, h in zip(lo, hi)]
    if min(radius) < 150 or max(radius) > 1000 or max(radius)/min(radius) > 3:
        raise ValueError("Insufficient axis coverage or magnetic disturbance; repeat away from metal")
    bias = [(h+l)/2 for l, h in zip(lo, hi)]
    average = sum(radius)/3
    scale = [average/r for r in radius]
    lengths = [math.sqrt(sum(((v[i]-bias[i])*scale[i])**2 for i in range(3))) for v in samples]
    mean = sum(lengths)/len(lengths)
    variation = math.sqrt(sum((v-mean)**2 for v in lengths)/len(lengths))/mean
    if variation > 0.2:
        raise ValueError("Corrected magnitude varies >20%; need better coverage or a full ellipsoid fit")
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
