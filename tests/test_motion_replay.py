"""Regression for a user-confirmed no-walking handling/flip recording.

Fixture contains only device milliseconds and acceleration, without host time
or BLE identifiers. Source: walk_100_normal_02_20260926_105821.csv, 530 rows.
The recording name says 'walk', but the user confirmed only flipping the board.
Actual original firmware counted 3; replaying its 10 Hz snapshots counted 2.
This test must not be reported as a full-rate or on-device validation.
"""
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "tools"))
from replay_motion import replay

app = root / "Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1"
result = replay(root / "tests/data/flip_10hz.csv", app / "Src/p2_motion.c", app / "Inc")
assert result["rows"] == 530
assert result["snapshot_hz"] == 10.0
assert result["replayed_steps"] == 0, result
print("PASS: user flip recording, 10 Hz snapshot replay: 0 steps (hardware retest still required)")

# Motivating hand-held recording, manual total 50. This is a development
# regression at 20 Hz, not a held-out accuracy test or the original 50 Hz input.
result = replay(root / 'tests/data/handheld50_20hz.csv', app / 'Src/p2_motion.c', app / 'Inc')
assert result['snapshot_hz'] == 20.0, result
assert 48 <= result['replayed_steps'] <= 53, result
print('PASS: hand-held development replay:', result['replayed_steps'],
      '(full-rate hardware count not established)')

# User-confirmed no-walking pitch rocking, including smaller and near-vertical
# excursions. Source: tilt_retry_751d00d1a259434abd61736f345f456b.log, last
# reset-separated segment only (device_ms 100..105600), Oct 13 2026.
# The board's original 50 Hz code published 21 false steps. These UART snapshots
# are only 10 Hz, so this regression cannot claim the same full-rate behavior.
# In fact, the old code already counted zero on this downsampled UART stream:
# keep this as a no-regression fixture, not evidence the fix removes the bug.
result = replay(root / 'tests/data/tilt_rocking_10hz.csv', app / 'Src/p2_motion.c', app / 'Inc')
assert result['rows'] == 1056 and result['snapshot_hz'] == 10.0, result
assert result['replayed_steps'] == 0, result
print('PASS: no-walking tilt/rocking development replay: 0 steps',
      '(10 Hz snapshots; full-rate hardware retest required)')

# Same trial, BLE 20 Hz snapshots from ble_20261013_041055_667613.csv;
# take the first session through device_ms 105600, before subsequent activity.
# Device ticks are quantized to 8 ms. Old algorithm replay counted 20 (board21),
# so unlike UART this fixture retains the false-positive triggering waveform.
result = replay(root / 'tests/data/tilt_rocking_ble20hz.csv', app / 'Src/p2_motion.c', app / 'Inc')
assert result['rows'] == 1843 and 19.9 < result['snapshot_hz'] < 20.1, result
assert result['recorded_step_delta'] == 21, result
assert result['replayed_steps'] == 0, result
print('PASS: no-walking tilt BLE replay: old 20 -> fixed 0 steps',
      '(20 Hz snapshots; independent full-rate hardware retest required)')
