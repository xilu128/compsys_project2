#!/usr/bin/env python3
"""Estimate hard-iron bias and diagonal soft-iron scale from UART rotation log.
Collect full, slow 3D rotations away from steel/electronics; not ordinary walking.
No hardware writes. Review results before copying macros into p2_config.h.
"""
import argparse
import math
import re
from pathlib import Path


def estimate(samples):
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
    samples = [tuple(map(float, m)) for m in re.findall(r"mag=(-?\d+),(-?\d+),(-?\d+)", args.log.read_text())]
    try:
        bias, scale, variation = estimate(samples)
    except ValueError as error:
        parser.error(str(error))
    for prefix, values in [("BIAS", bias), ("SCALE", scale)]:
        for axis, value in zip("XYZ", values):
            print(f"#define P2_MAG_{prefix}_{axis} {value:.6f}f")
    print(f"// {len(samples)} samples; corrected magnitude CV={variation:.1%}")
    print("// Diagonal fit only; validate against independent known directions.")

if __name__ == "__main__":
    main()
