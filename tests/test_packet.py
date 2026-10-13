"""Desktop validation of the documented course BLE packet's host decoding."""
import struct
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from check_hardware import decode
p=decode(struct.pack('<H9h',65535,1000,-1000,0,120,-10,84,150,-150,480))
assert p['tick']==65535 and p['acc']==(1000,-1000,0)
assert p['steps']==12 and p['heading']==-1 and p['distance']==8.4
assert p['mag']==(150,-150,480)
for invalid in (b'',bytes(19),bytes(21)):
    try:
        decode(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError('Malformed packet accepted')
print('PASS: BLE packet length, signed units, timestamp, steps, distance, invalid heading')

from ble_monitor import packet
for status in range(-32768, -32763):
    p = packet(struct.pack('<H9h', 1, 0, 0, 1000, 0, status, 0, 300, 0, 250))
    assert p['heading'] == status/10 and not p['heading_valid']
for degrees in (-3200, -720, -360, -180, -4, -3, -2, -1, -0.1, 0, 360, 720, 3200):
    p = packet(struct.pack('<H9h', 1, 0, 0, 1000, 0, round(degrees*10), 0, 300, 0, 250))
    assert p['heading'] == degrees and p['heading_valid']
try:
    packet(struct.pack('<H9h', 1, 0, 0, 1000, 0, -32763, 0, 300, 0, 250))
except ValueError:
    pass
else:
    raise AssertionError('Unknown heading status accepted')
print('PASS: signed cumulative headings, reserved status codes and overflow')
