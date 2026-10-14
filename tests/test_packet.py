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
print('PASS: BLE packet length, signed raw fields, timestamp, steps and distance scaling')

from ble_monitor import Monitor, packet, csv_row, heading_label
for status in range(-32768, -32763):
    p = packet(struct.pack('<H9h', 1, 0, 0, 1000, 0, status, 0, 300, 0, 250))
    assert p['heading'] == status/10 and not p['heading_valid']
    assert '°' not in heading_label(p)
for degrees in range(360):
    p = packet(struct.pack('<H9h', 1, 0, 0, 1000, 0, degrees*10, 0, 300, 0, 250))
    assert p['heading'] == degrees and p['heading_valid']
    assert heading_label(p) == f'{degrees}°'

# Exact course examples, including both directions through startup zero and a
# complete revolution. No host-side origin is learned on the first packet.
for session in (1, 2):
    for degrees in (270, 0, 90, 180, 270, 359, 0, 1, 0, 359, 270, 180, 90, 0):
        p = packet(struct.pack('<H9h', 65535, 100, -200, 980, 120, degrees*10,
                               84, 300, 0, 250), session=session)
        assert p['session'] == session and p['heading'] == degrees
        assert p['steps'] == 12 and p['distance'] == 8.4
        assert csv_row(p)[10:13] == [degrees, 1, 8.4]

# Fail visibly for old signed-cycle firmware or malformed integer-degree data;
# never silently turn -90 into 270 or 360 into 0 in the Windows viewer.
for wire_heading in (-32763, -32000, -3600, -900, -10, -1, 1, 899, 3591, 3599,
                     3600, 7200, 32000, 32767):
    try:
        packet(struct.pack('<H9h', 1, 0, 0, 1000, 0, wire_heading, 0, 300, 0, 250))
    except ValueError as exc:
        assert 'Heading protocol mismatch' in str(exc)
    else:
        raise AssertionError(f'Invalid/legacy heading accepted: {wire_heading}')
for steps, distance in ((-1, 0), (0, -1)):
    try:
        packet(struct.pack('<H9h', 1, 0, 0, 1000, steps, 0, distance, 300, 0, 250))
    except ValueError:
        pass
    else:
        raise AssertionError('Negative step count or distance accepted')
print('PASS: 0–359 integer headings, clockwise/CCW examples, reconnect origin, CSV and reserved statuses')

# Exercise compass rendering without opening a GUI or connecting to hardware.
class FakeCanvas:
    def __init__(self):
        self.text = []
        self.arrows = []

    def delete(self, *_):
        self.text.clear()
        self.arrows.clear()

    def create_oval(self, *_, **__):
        pass

    def create_line(self, *args, **kwargs):
        if kwargs.get('arrow') == 'last':
            self.arrows.append(args)

    def create_text(self, *_, **kwargs):
        self.text.append(kwargs['text'])


from types import SimpleNamespace
canvas = FakeCanvas()
monitor = SimpleNamespace(compass=canvas, muted='gray', ink='black', font='Arial',
                          state='connected')
for degrees, expected_direction in ((0, (0, -1)), (90, (1, 0)),
                                    (180, (0, 1)), (270, (-1, 0))):
    monitor.latest = packet(struct.pack('<H9h', 1, 0, 0, 1000, 0, degrees*10,
                                        0, 300, 0, 250))
    Monitor.draw_compass(monitor, True)
    assert canvas.text == ['0°', '90°', '180°', '270°', f'{degrees}° · From startup']
    x1, y1, x2, y2 = canvas.arrows[0]
    dx, dy = expected_direction
    assert abs((x2-x1)/80-dx) < 1e-9 and abs((y2-y1)/80-dy) < 1e-9
monitor.latest = packet(struct.pack('<H9h', 1, 0, 0, 1000, 0, -32767,
                                    0, 300, 0, 250))
Monitor.draw_compass(monitor, False)
assert canvas.text[-1] == 'Calibrating' and not canvas.arrows
print('PASS: compass cardinal labels, clockwise arrow geometry and unavailable-state rendering')
