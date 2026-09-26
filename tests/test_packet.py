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
