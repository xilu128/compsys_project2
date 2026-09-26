"""Synthetic calibration correctness, not a hardware accuracy result."""
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from calibrate_mag import estimate
samples = []
for i in range(400):
    z = 1 - 2*(i+0.5)/400
    phi = i*2.3999632297
    r = math.sqrt(1-z*z)
    samples.append((80+400*r*math.cos(phi), -50+500*r*math.sin(phi), 30+600*z))
bias, scale, cv = estimate(samples)
assert max(abs(x-y) for x, y in zip(bias, (80, -50, 30))) < 5
assert max(abs(x-y) for x, y in zip(scale, (1.25, 1, 5/6))) < 0.02
assert cv < 0.02
for invalid in [samples[:10], [(1, 2, 3)]*100, [(float('nan'),2,3)]*100,
                [(1,float('inf'),3)]*100]:
    try:
        estimate(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError('Insufficient data/coverage must be rejected')
print('PASS: known ellipsoid, bias/scale recovery, insufficient data and coverage rejection')
