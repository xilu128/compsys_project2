"""Local SensorTile BLE viewer. No firmware writes and no cloud connection."""
import argparse
import asyncio
import csv
import math
import sys
from collections import deque
from datetime import datetime
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

from check_hardware import MOTION_UUID, decode

ROOT = Path(__file__).resolve().parents[2]
LOGS = ROOT / 'logs' / 'ble_monitor'
ADDRESS = 'F9:B0:57:92:6B:52'
NAME = 'P202S01'
HEADING_STATUS = {-3276.8: 'Unavailable', -3276.7: 'Calibrating',
                  -3276.6: 'Disturbance', -3276.5: 'Recovering',
                  -3276.4: 'Invalid numerical angle'}
HEADING_HELP = ('Requires interview-guideline firmware: integer 0–359°. Startup direction = 0°; '
                'clockwise 90° = 90°, counter-clockwise 90° = 270°. '
                'Reconnect does not reset zero; this is not geographic north.')
FIELDS = ['host_time_iso', 'session', 'device_tick_8ms', 'acc_x_mg', 'acc_y_mg',
          'acc_z_mg', 'mag_x_mgauss', 'mag_y_mgauss', 'mag_z_mgauss',
          'step_count', 'heading_deg', 'heading_valid', 'distance_m',
          'acc_fresh', 'mag_fresh', 'raw_hex']


def packet(data, session=1):
    p = decode(data)
    if p['steps'] < 0 or p['distance'] < 0:
        raise ValueError('Invalid summary fields in motion packet')
    # The firmware owns the startup reference and integer-degree conversion.
    # Do not wrap/re-zero legacy signed packets here: that would hide a firmware
    # mismatch from the instructor's app, which receives the same BLE fields.
    if p['heading'] not in HEADING_STATUS and not (
            0 <= p['heading'] <= 359 and p['heading'].is_integer()):
        raise ValueError('Heading protocol mismatch: expected integer 0–359°; '
                         'install interview-guideline firmware (legacy signed angles are not converted)')
    p.update(received=time.monotonic(), host_time_iso=datetime.now().astimezone().isoformat(),
             session=session, heading_valid=p['heading'] not in HEADING_STATUS,
             acc_fresh=-32768 not in p['acc'], mag_fresh=-32768 not in p['mag'],
             raw_hex=bytes(data).hex())
    return p


def heading_label(p):
    return f"{p['heading']:.0f}°" if p['heading_valid'] else HEADING_STATUS.get(p['heading'], 'Invalid')


def csv_row(p):
    return [p['host_time_iso'], p['session'], p['tick'], *p['acc'], *p['mag'],
            p['steps'], p['heading'], int(p['heading_valid']), p['distance'],
            int(p['acc_fresh']), int(p['mag_fresh']), p['raw_hex']]


class Receiver:
    def __init__(self):
        self.events = queue.Queue(maxsize=2000)
        self.stop = threading.Event()
        self.record = threading.Event()
        self.thread = None
        self.loop = self.task = None
        self.display_dropped = 0
        self.file = self.writer = None
        self.last_flush = 0

    def emit(self, kind, value):
        try:
            self.events.put_nowait((kind, value))
        except queue.Full:
            self.display_dropped += 1

    def start(self, target):
        if self.thread and self.thread.is_alive():
            return
        self.stop.clear()
        self.thread = threading.Thread(target=self.run, args=(target,), daemon=True)
        self.thread.start()

    def disconnect(self):
        self.stop.set()
        if self.loop and self.task:
            try:
                self.loop.call_soon_threadsafe(self.task.cancel)
            except RuntimeError:
                pass

    def close_log(self):
        if self.file:
            self.file.close()
            self.file = self.writer = None
            self.emit('log', 'CSV saved')

    def log_packet(self, p):
        if not self.record.is_set():
            self.close_log()
            return
        try:
            if not self.file:
                LOGS.mkdir(parents=True, exist_ok=True)
                path = LOGS / (datetime.now().strftime('ble_%Y%m%d_%H%M%S_%f') + '.csv')
                self.file = path.open('x', newline='', encoding='utf-8-sig')
                self.writer = csv.writer(self.file)
                self.writer.writerow(FIELDS)
                self.emit('log', str(path))
            self.writer.writerow(csv_row(p))
            if time.monotonic() - self.last_flush >= 1:
                self.file.flush()
                self.last_flush = time.monotonic()
        except OSError as exc:
            self.record.clear()
            self.close_log()
            self.emit('log', f'CSV save failed: {exc}')

    def run(self, target):
        try:
            asyncio.run(self.receive(target))
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            self.emit('status', f'Bluetooth error: {exc}')
        finally:
            self.close_log()
            self.loop = self.task = None
            self.emit('state', 'idle')

    async def receive(self, target):
        from bleak import BleakClient, BleakScanner
        self.loop = asyncio.get_running_loop()
        self.task = asyncio.current_task()
        session = 0
        while not self.stop.is_set():
            self.emit('state', 'scanning')
            self.emit('status', f'Scanning {target} … Power on the board and disconnect the iPad app')
            try:
                device = await BleakScanner.find_device_by_filter(
                    lambda d, a: d.address.upper() == target.upper()
                    or a.local_name == target or d.name == target
                    or (target == NAME and d.address.upper() == ADDRESS), timeout=12)
                if self.stop.is_set():
                    break
                if device is None:
                    raise RuntimeError('Device not found; check power, Bluetooth and other app connections')
                self.emit('status', f'Connecting to {device.name or target} ({device.address}) …')
                async with BleakClient(device, timeout=25) as client:
                    char = client.services.get_characteristic(MOTION_UUID)
                    if char is None:
                        raise RuntimeError('Required motion characteristic is missing')
                    session += 1
                    last_packet = time.monotonic()

                    def notified(_, data):
                        nonlocal last_packet
                        try:
                            p = packet(data, session)
                            last_packet = p['received']
                            self.log_packet(p)
                            self.emit('packet', p)
                        except ValueError as exc:
                            self.emit('bad', str(exc))

                    await client.start_notify(char, notified)
                    self.emit('state', 'connected')
                    self.emit('status', f'Connected to {device.name or target} · Waiting for live data')
                    while client.is_connected and not self.stop.is_set():
                        if not self.record.is_set():
                            self.close_log()
                        if time.monotonic() - last_packet > 8:
                            raise RuntimeError('No notifications for 8 seconds; reconnecting')
                        await asyncio.sleep(.2)
                    if client.is_connected:
                        await client.stop_notify(char)
                self.emit('status', 'Connection lost; reconnecting …')
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.emit('status', f'{exc}; retrying in 5 seconds')
            self.emit('state', 'retry')
            for _ in range(25):
                if self.stop.is_set():
                    return
                await asyncio.sleep(.2)


class Monitor:
    def __init__(self, root, auto=True):
        self.root, self.receiver = root, Receiver()
        self.history = deque(maxlen=600)
        self.latest = None
        self.total = self.bad = 0
        self.state = 'idle'
        self.closing = False
        root.title('SensorTile · Group 02')
        root.geometry('1240x900')
        root.minsize(1060, 800)
        self.bg, self.ink, self.muted = '#f5f5f7', '#1d1d1f', '#6e6e73'
        self.font = 'Helvetica Neue' if sys.platform == 'darwin' else 'Segoe UI'
        root.configure(bg=self.bg)
        style = ttk.Style(root)
        style.theme_use('clam')
        style.configure('.', font=(self.font, 11), foreground=self.ink)
        style.configure('TFrame', background=self.bg)
        style.configure('TLabel', background=self.bg)
        style.configure('TButton', background='#e8e8ed', borderwidth=0, padding=(16, 9))
        style.map('TButton', background=[('active', '#dcdce2')])
        style.configure('Primary.TButton', background='#0071e3', foreground='white')
        style.map('Primary.TButton', background=[('disabled', '#d2d2d7'), ('active', '#0077ed')])
        style.configure('TCheckbutton', background=self.bg)
        style.configure('TEntry', padding=8, fieldbackground='white', bordercolor='#d2d2d7')
        wrap = ttk.Frame(root, padding=24)
        wrap.pack(fill='both', expand=True)
        header = ttk.Frame(wrap)
        header.pack(fill='x', pady=(0, 18))
        ttk.Label(header, text='SensorTile', font=(self.font, 27, 'bold')).pack(side='left')
        ttk.Label(header, text='GROUP 02  /  LIVE MOTION MONITOR', foreground=self.muted).pack(side='left', padx=18)
        self.connection = tk.StringVar(value='● Disconnected')
        self.connection_label = ttk.Label(header, textvariable=self.connection, foreground=self.muted)
        self.connection_label.pack(side='right')
        controls = ttk.Frame(wrap)
        controls.pack(fill='x')
        ttk.Label(controls, text='Device', foreground=self.muted).pack(side='left', padx=(0, 8))
        self.target = tk.StringVar(value=NAME)
        self.entry = ttk.Entry(controls, textvariable=self.target, width=22)
        self.entry.pack(side='left')
        self.connect = ttk.Button(controls, text='Connect', command=self.start, style='Primary.TButton')
        self.connect.pack(side='left', padx=8)
        ttk.Button(controls, text='Disconnect', command=self.stop).pack(side='left')
        self.recording = tk.BooleanVar(value=False)
        ttk.Checkbutton(controls, text='Record CSV', variable=self.recording, command=self.toggle_record).pack(side='right')
        self.status = tk.StringVar(value='Power on the board and connect to view live data.')
        ttk.Label(wrap, textvariable=self.status, wraplength=980, foreground=self.muted).pack(anchor='w', pady=(10, 16))
        overview = ttk.Frame(wrap)
        overview.pack(fill='x')
        self.compass = tk.Canvas(overview, width=236, height=230, bg='white', highlightthickness=0)
        self.compass.pack(side='left', padx=(0, 12))
        cards = ttk.Frame(overview)
        cards.pack(side='left', fill='both', expand=True)
        self.values = {}
        descriptions = [('steps', 'Step count', 'Steps since board startup'),
                        ('heading', 'Relative heading', 'Board +X · Startup 0° · Range 0–359°'),
                        ('distance', 'Estimated distance', 'Firmware estimate · Model not reported'),
                        ('rate', 'BLE notification rate', 'Receive rate · Sensor rate differs')]
        for i, (key, label, note) in enumerate(descriptions):
            box = tk.Frame(cards, bg='white', padx=18, pady=12)
            box.grid(row=i//2, column=i%2, padx=(0, 10 if i%2 == 0 else 0),
                     pady=(0 if i<2 else 10, 0), sticky='nsew')
            cards.columnconfigure(i%2, weight=1, uniform='cards')
            cards.rowconfigure(i//2, weight=1)
            tk.Label(box, text=label, bg='white', fg=self.muted, font=(self.font, 11)).pack(anchor='w')
            self.values[key] = tk.StringVar(value='—')
            tk.Label(box, textvariable=self.values[key], bg='white', fg=self.ink,
                     font=(self.font, 22 if key == 'heading' else 27, 'bold')).pack(anchor='w')
            tk.Label(box, text=note, bg='white', fg=self.muted, font=(self.font, 10)).pack(anchor='w')
        sensors = ttk.Frame(wrap)
        sensors.pack(fill='x', pady=12)
        self.sensor_values, self.sensor_states = {}, {}
        for col, (key, title, unit) in enumerate([('acc', 'Acceleration', 'mg'), ('mag', 'Magnetic field', 'mGauss')]):
            box = tk.Frame(sensors, bg='white', padx=16, pady=12)
            box.grid(row=0, column=col, sticky='nsew', padx=(0, 12 if col==0 else 0))
            sensors.columnconfigure(col, weight=1, uniform='sensors')
            self.sensor_states[key] = tk.StringVar(value=f'{title} · {unit}  /  Waiting for data')
            tk.Label(box, textvariable=self.sensor_states[key], bg='white', fg=self.muted,
                     font=(self.font, 11)).pack(anchor='w')
            row = tk.Frame(box, bg='white'); row.pack(fill='x', pady=(8, 0))
            for axis, color in enumerate(['#ff3b30', '#248a3d', '#007aff']):
                self.sensor_values[key, axis] = tk.StringVar(value='XYZ'[axis]+'  —')
                tk.Label(row, textvariable=self.sensor_values[key, axis], bg='white', fg=color,
                         font=(self.font, 15, 'bold'), anchor='w').pack(side='left', expand=True, fill='x')
        charts_title = ttk.Frame(wrap); charts_title.pack(fill='x', pady=(0, 8))
        ttk.Label(charts_title, text='Sensor trends', font=(self.font, 14, 'bold')).pack(side='left')
        ttk.Label(charts_title, text='Last 15 seconds    X red  ·  Y green  ·  Z blue', foreground=self.muted).pack(side='right')
        self.canvas = tk.Canvas(wrap, bg='white', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.stats = tk.StringVar(value='No data received')
        ttk.Label(wrap, textvariable=self.stats, foreground=self.muted).pack(anchor='w', pady=(10, 4))
        self.log_path = tk.StringVar(value=f'CSV folder: {LOGS}')
        ttk.Label(wrap, textvariable=self.log_path, wraplength=980, foreground=self.muted,
                  font=(self.font, 9)).pack(anchor='w')
        ttk.Label(wrap, text=HEADING_HELP,
                  foreground=self.muted, font=(self.font, 9), wraplength=980).pack(anchor='w', pady=(4, 0))
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(100, self.poll)
        if auto:
            root.after(500, self.start)

    def start(self):
        target = self.target.get().strip()
        if not target or (self.receiver.thread and self.receiver.thread.is_alive()):
            return
        self.history.clear()
        self.latest = None
        self.total = self.bad = 0
        self.connect.configure(state='disabled')
        self.entry.configure(state='disabled')
        self.receiver.start(target)

    def stop(self):
        self.status.set('Disconnecting …')
        self.receiver.disconnect()

    def toggle_record(self):
        if self.recording.get():
            self.receiver.record.set()
            self.log_path.set('Recording starts with the next live packet …')
        else:
            self.receiver.record.clear()

    def close(self):
        self.closing = True
        self.stop()
        self.finish_close()

    def finish_close(self):
        if self.receiver.thread and self.receiver.thread.is_alive():
            self.root.after(100, self.finish_close)
        else:
            self.root.destroy()

    def poll(self):
        for _ in range(2000):
            try:
                kind, value = self.receiver.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'status': self.status.set(value)
            elif kind == 'log': self.log_path.set(value)
            elif kind == 'bad':
                self.bad += 1
                self.status.set(value)
            elif kind == 'state':
                self.state = value
                if value == 'idle':
                    self.connect.configure(state='normal')
                    self.entry.configure(state='normal')
                    if self.receiver.stop.is_set(): self.status.set('Disconnected. Reconnect or close the window.')
            elif kind == 'packet':
                if self.latest and self.latest['session'] != value['session']:
                    self.history.clear()
                self.latest = value
                self.history.append(value)
                self.total += 1
        now = time.monotonic()
        fresh = self.latest is not None and self.state == 'connected' and now - self.latest['received'] <= 1
        if fresh:
            p = self.latest
            self.values['steps'].set(f"{p['steps']:g}" + (' (limit)' if p['steps'] >= 3276 else ''))
            self.values['heading'].set(heading_label(p))
            self.values['distance'].set(f"{p['distance']:.1f} m")
            recent = [x for x in self.history if now - x['received'] < 5]
            hz = (len(recent)-1)/(recent[-1]['received']-recent[0]['received']) if len(recent)>1 else 0
            self.values['rate'].set(f'{hz:.1f} Hz')

        else:
            for value in self.values.values(): value.set('—')

        labels = {'idle': 'Disconnected', 'scanning': 'Scanning', 'retry': 'Reconnecting', 'connected': 'Waiting for data'}
        live_label = 'Live' if fresh else ('Stale data' if self.state == 'connected' and self.latest else labels.get(self.state, self.state))
        self.connection.set('● ' + live_label)
        self.connection_label.configure(foreground='#248a3d' if fresh else '#b36b00' if self.state != 'idle' else self.muted)
        for key, title, unit in [('acc', 'Acceleration', 'mg'), ('mag', 'Magnetic field', 'mGauss')]:
            valid = fresh and self.latest[key+'_fresh']
            self.sensor_states[key].set(f'{title} · {unit}  /  ' + ('Live' if valid else 'Invalid or stale' if self.latest else 'Waiting for data'))
            for axis in range(3):
                value = f'{self.latest[key][axis]:g}' if valid else '—'
                self.sensor_values[key, axis].set('XYZ'[axis]+'  '+value)
        age = f"{now-self.latest['received']:.1f} s" if self.latest else '—'
        self.stats.set(f'Packets {self.total:,}    ·    Format errors {self.bad}    ·    Display drops {self.receiver.display_dropped}    ·    Data age {age}')
        self.draw_compass(fresh and self.latest['heading_valid'])
        self.draw(now)
        if not self.closing: self.root.after(100, self.poll)

    def draw_compass(self, valid):
        c = self.compass
        c.delete('all')
        cx, cy, r = 118, 104, 66
        c.create_oval(cx-r, cy-r, cx+r, cy+r, outline='#e5e5ea', width=2)
        for angle in range(0, 360, 30):
            a = math.radians(angle)
            c.create_line(cx+math.sin(a)*(r-5), cy-math.cos(a)*(r-5),
                          cx+math.sin(a)*r, cy-math.cos(a)*r, fill='#d2d2d7')
        for x, y, label in [(cx, 20, '0°'), (cx+88, cy, '90°'), (cx, 186, '180°'), (cx-88, cy, '270°')]:
            c.create_text(x, y, text=label, fill=self.muted, font=(self.font, 10))
        if valid:
            heading = self.latest['heading']
            a = math.radians(heading)
            dx, dy = math.sin(a), -math.cos(a)
            c.create_line(cx-dx*28, cy-dy*28, cx+dx*52, cy+dy*52,
                          fill='#007aff', width=4, arrow='last', arrowshape=(14, 16, 6))
            c.create_oval(cx-5, cy-5, cx+5, cy+5, fill='#007aff', outline='white', width=2)
            caption = f'{heading_label(self.latest)} · From startup'
        else:
            c.create_text(cx, cy, text='—', fill='#aeaeb2', font=(self.font, 30))
            caption = ('Stale data' if self.state != 'connected' or time.monotonic()-self.latest['received'] > 1 else
                       HEADING_STATUS.get(self.latest['heading'], 'Heading unavailable')) if self.latest else 'Waiting for heading'
        c.create_text(cx, 215, text=caption, fill=self.ink, font=(self.font, 11))

    def draw(self, now):
        c = self.canvas
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        if w < 100 or h < 100: return
        left, right = 65, w-18
        colors = ['#ff3b30', '#248a3d', '#007aff']
        for index, (key, label, limit) in enumerate([('acc','Acceleration / mg',2000),('mag','Magnetic field / mGauss',1000)]):
            top, bottom = index*h/2+30, (index+1)*h/2-20
            points = [p for p in self.history if now-p['received'] <= 15]
            valid = [abs(v) for p in points if p[key+'_fresh'] for v in p[key]]
            limit = max(limit, max(valid, default=0)*1.1)
            c.create_text(8, top-17, text=label, anchor='w', fill='#243449')
            for val in [-limit, 0, limit]:
                y = bottom-(val+limit)/(2*limit)*(bottom-top)
                c.create_line(left,y,right,y,fill='#e2e8ef')
                c.create_text(left-7,y,text=f'{val:.0f}',anchor='e',fill='#6b7280')
            for axis, color in enumerate(colors):
                segment = []
                previous = None
                for p in points:
                    if not p[key+'_fresh'] or (previous is not None and p['received']-previous > .5):
                        if len(segment)>=4: c.create_line(*segment,fill=color,width=2)
                        segment=[]
                    if p[key+'_fresh']:
                        segment.extend([right-(now-p['received'])/15*(right-left),
                                        bottom-(p[key][axis]+limit)/(2*limit)*(bottom-top)])
                    previous = p['received']
                if len(segment)>=4: c.create_line(*segment,fill=color,width=2)
            if not valid:
                c.create_text((left+right)/2, (top+bottom)/2, text='Waiting for valid sensor data', fill='#aeaeb2')
            c.create_text(left,bottom+12,text='−15 s',anchor='w',fill='#6b7280')
            c.create_text(right,bottom+12,text='Now',anchor='e',fill='#6b7280')


async def probe():
    from bleak import BleakScanner
    d = await BleakScanner.find_device_by_filter(
        lambda d,a: d.address.upper()==ADDRESS or a.local_name==NAME or d.name==NAME, timeout=12)
    print(f'TARGET={d}' if d else 'TARGET_NOT_FOUND', flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', action='store_true')
    parser.add_argument('--smoke', action='store_true')
    parser.add_argument('--no-auto', action='store_true')
    args=parser.parse_args()
    if args.probe:
        asyncio.run(probe())
        return
    root=tk.Tk()
    Monitor(root, auto=not(args.smoke or args.no_auto))
    if args.smoke: root.after(1500, root.destroy)
    root.mainloop()


if __name__ == '__main__':
    main()
