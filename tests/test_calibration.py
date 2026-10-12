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

# Long endpoint holds must not bias the fit toward two orientations.
held = samples + [samples[0]]*5000 + [samples[200]]*5000
bias, scale, cv = estimate(held)
assert max(abs(x-y) for x, y in zip(bias, (80, -50, 30))) < 1
# Deterministic measurement noise, with unequal axis scales and offsets.
noisy = [tuple(v[j]+2*math.sin(i*1.7+j) for j in range(3))
         for i, v in enumerate(samples)]
bias, scale, cv = estimate(noisy)
assert max(abs(x-y) for x, y in zip(bias, (80, -50, 30))) < 3
assert cv < .02
# A tilted circle can span all three axes while still being planar.
planar = [(400*math.cos(i*.02), 400*math.sin(i*.02),
           300*math.cos(i*.02)+300*math.sin(i*.02)) for i in range(400)]
for invalid in [planar, [samples[0], samples[200]]*500,
                samples+[(2000, -2000, 2000)]]:
    try:
        estimate(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError('Planar/two-pose/interference calibration must be rejected')
print('PASS: uneven dwell, noisy fit, planar rotation and interference rejection')
