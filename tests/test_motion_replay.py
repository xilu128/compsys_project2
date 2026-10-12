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
