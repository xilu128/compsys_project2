#!/usr/bin/env python3
"""Fit a personal cadence/step-length model from measured walking trials.
CSV: distance_m,steps,duration_s,split (train or validation).
Use manual step counts and walking-only duration, consistent placement/person.
Outputs suggested settings only; never writes firmware or programs hardware.
"""
import argparse
import csv
import math
from pathlib import Path


def load(path):
    trials = []
    with Path(path).open(encoding='utf-8-sig', newline='') as stream:
        for row in csv.DictReader(stream):
            d, n, t = (float(row[k]) for k in ('distance_m', 'steps', 'duration_s'))
            split = row['split'].strip()
            if not all(math.isfinite(v) and v > 0 for v in (d, n, t)) or n != int(n):
                raise ValueError('Distance/duration must be positive; steps must be a positive integer')
            if split not in ('train', 'validation'):
                raise ValueError('split must be train or validation')
            hz, length = n/t, d/n
            if not 1000/1500 <= hz <= 1000/280 or not .25 <= length <= 1.5:
                raise ValueError('Trial outside supported walking cadence/step-length range')
            trials.append((hz, length, n, d, split))
    return trials


def fit(trials):
    train = [r for r in trials if r[4] == 'train']
    validation = [r for r in trials if r[4] == 'validation']
    if len(train) < 6 or len(validation) < 3:
        raise ValueError('Need at least 6 training and 3 independent validation trials')
    x = [r[0] for r in train]
    y = [r[1] for r in train]
    low, high = min(x), max(x)
    if high-low < .5:
        raise ValueError('Insufficient cadence coverage: include slow, normal and fast walking')
    xm, ym = sum(x)/len(x), sum(y)/len(y)
    slope = sum((a-xm)*(b-ym) for a,b in zip(x,y))/sum((a-xm)**2 for a in x)
    intercept = ym-slope*xm
    if slope < 0 or intercept+slope*low < .25 or intercept+slope*high > 1.5:
        raise ValueError('Implausible fit; review measurements rather than forcing coefficients')
    fixed = sum(r[3] for r in train)/sum(r[2] for r in train)
    def error(mode):
        errors = []
        for hz, _, n, distance, _ in validation:
            length = .7 if mode == 'baseline' else fixed
            if mode == 'adaptive' and low <= hz <= high:
                length = intercept+slope*hz
            errors.append(abs(n*length-distance)/distance)
        return sum(errors)/len(errors)
    errors = {m:error(m) for m in ('baseline','fixed','adaptive')}
    return fixed, intercept, slope, low, high, errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    args = parser.parse_args()
    try:
        fixed, intercept, slope, low, high, errors = fit(load(args.csv))
    except (ValueError, KeyError, OSError) as exc:
        parser.error(str(exc))
    for name, error in errors.items():
        print(f'Validation mean absolute relative distance error ({name}): {error:.2%}')
    enable = errors['adaptive'] < min(errors['baseline'], errors['fixed'])
    print('// Suggested values only. Validate firmware end-to-end before use.')
    for name, value in [('STEP_LENGTH_M',fixed), ('STRIDE_INTERCEPT_M',intercept),
                        ('STRIDE_SLOPE_M_PER_HZ',slope), ('STRIDE_MIN_HZ',low),
                        ('STRIDE_MAX_HZ',high)]:
        print(f'#define P2_{name} {value:.6f}f')
    print(f'#define P2_STRIDE_ADAPTIVE {int(enable)}')
    print('// Adaptive recommended only if it beats BOTH baselines on held-out trials.')
    print('// These scores use manual counts and trial-average cadence, not firmware replay.')


if __name__ == '__main__':
    main()
