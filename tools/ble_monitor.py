"""Local SensorTile BLE viewer. No firmware writes and no cloud connection."""
import argparse
import asyncio
import csv
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
FIELDS = ['host_time_iso', 'session', 'device_tick_8ms', 'acc_x_mg', 'acc_y_mg',
          'acc_z_mg', 'mag_x_mgauss', 'mag_y_mgauss', 'mag_z_mgauss',
          'step_count', 'heading_deg', 'heading_valid', 'distance_m',
          'acc_fresh', 'mag_fresh', 'raw_hex']


def packet(data, session=1):
    p = decode(data)
    if p['steps'] < 0 or p['distance'] < 0 or not (p['heading'] == -1 or 0 <= p['heading'] < 360):
        raise ValueError('Invalid summary fields in motion packet')
    p.update(received=time.monotonic(), host_time_iso=datetime.now().astimezone().isoformat(),
             session=session, heading_valid=p['heading'] != -1,
             acc_fresh=-32768 not in p['acc'], mag_fresh=-32768 not in p['mag'],
             raw_hex=bytes(data).hex())
    return p


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
            self.emit('log', 'CSV 已保存')

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
            self.emit('log', f'CSV 保存失败：{exc}')

    def run(self, target):
        try:
            asyncio.run(self.receive(target))
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            self.emit('status', f'蓝牙错误：{exc}')
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
            self.emit('status', f'扫描 {target} … 请给板子供电，并断开 iPad App')
            try:
                device = await BleakScanner.find_device_by_filter(
                    lambda d, a: d.address.upper() == target.upper()
                    or a.local_name == target or d.name == target
                    or (target == NAME and d.address.upper() == ADDRESS), timeout=12)
                if self.stop.is_set():
                    break
                if device is None:
                    raise RuntimeError('未找到设备；检查供电、Windows 蓝牙和 iPad 占用')
                self.emit('status', f'连接 {device.name or target} ({device.address}) …')
                async with BleakClient(device, timeout=25) as client:
                    char = client.services.get_characteristic(MOTION_UUID)
                    if char is None:
                        raise RuntimeError('设备没有课程所需的运动数据特征值')
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
                    self.emit('status', f'已连接 {device.name or target} · 等待真实数据')
                    while client.is_connected and not self.stop.is_set():
                        if not self.record.is_set():
                            self.close_log()
                        if time.monotonic() - last_packet > 8:
                            raise RuntimeError('8 秒未收到通知，正在重新连接')
                        await asyncio.sleep(.2)
                    if client.is_connected:
                        await client.stop_notify(char)
                self.emit('status', '连接已断开，准备重连 …')
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.emit('status', f'{exc}；5 秒后重试')
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
        root.title('SensorTile BLE Monitor — Group 02')
        root.geometry('1050x760')
        root.minsize(850, 650)
        root.configure(bg='#edf2f7')
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TLabel', font=('Microsoft YaHei UI', 10))
        style.configure('TButton', font=('Microsoft YaHei UI', 10), padding=6)
        wrap = ttk.Frame(root, padding=18)
        wrap.pack(fill='both', expand=True)
        ttk.Label(wrap, text='SensorTile · 实时蓝牙数据', font=('Microsoft YaHei UI', 21, 'bold')).pack(anchor='w')
        ttk.Label(wrap, text='Group 02  |  Acc / Mag / 步数 / 磁航向 / 距离').pack(anchor='w', pady=(0,12))
        controls = ttk.Frame(wrap)
        controls.pack(fill='x')
        ttk.Label(controls, text='设备名称或地址：').pack(side='left')
        self.target = tk.StringVar(value=NAME)
        self.entry = ttk.Entry(controls, textvariable=self.target, width=24)
        self.entry.pack(side='left', padx=4)
        self.connect = ttk.Button(controls, text='连接', command=self.start)
        self.connect.pack(side='left', padx=5)
        ttk.Button(controls, text='断开', command=self.stop).pack(side='left', padx=5)
        self.recording = tk.BooleanVar(value=False)
        ttk.Checkbutton(controls, text='保存 CSV', variable=self.recording, command=self.toggle_record).pack(side='left', padx=12)
        self.status = tk.StringVar(value='就绪')
        ttk.Label(wrap, textvariable=self.status, wraplength=970, foreground='#245279').pack(anchor='w', pady=10)
        cards = ttk.Frame(wrap)
        cards.pack(fill='x')
        self.values = {}
        for col, (key, label) in enumerate([('steps','累计步数'),('heading','磁航向'),('distance','估算距离'),('rate','通知速率')]):
            box = ttk.LabelFrame(cards, text=label, padding=12)
            box.grid(row=0, column=col, padx=4, sticky='nsew')
            cards.columnconfigure(col, weight=1)
            self.values[key] = tk.StringVar(value='—')
            ttk.Label(box, textvariable=self.values[key], font=('Segoe UI', 25, 'bold')).pack()
        self.axes = tk.StringVar(value='Acc XYZ：— mg       Mag XYZ：— mGauss')
        ttk.Label(wrap, textvariable=self.axes, font=('Consolas', 12)).pack(anchor='w', pady=12)
        self.canvas = tk.Canvas(wrap, bg='white', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.stats = tk.StringVar(value='尚未收到数据')
        ttk.Label(wrap, textvariable=self.stats).pack(anchor='w', pady=(8,2))
        self.log_path = tk.StringVar(value=f'CSV 目录：{LOGS}')
        ttk.Label(wrap, textvariable=self.log_path, wraplength=970).pack(anchor='w')
        ttk.Label(wrap, text='无效航向显示“无效”；断线或超过 1 秒无通知显示“数据过期”。磁航向未代表已校准。\n步数/距离累计自板子启动；BLE 显示上限约 3276 步、3276.7 m。', foreground='#626b78').pack(anchor='w', pady=(8,0))
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
        self.status.set('正在断开 …')
        self.receiver.disconnect()

    def toggle_record(self):
        if self.recording.get():
            self.receiver.record.set()
            self.log_path.set('等待下一条真实数据开始保存 CSV …')
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
            elif kind == 'bad': self.bad += 1
            elif kind == 'state':
                self.state = value
                if value == 'idle':
                    self.connect.configure(state='normal')
                    self.entry.configure(state='normal')
                    if self.receiver.stop.is_set(): self.status.set('已断开，可重新连接或关闭窗口')
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
            self.values['steps'].set(f"{p['steps']:g}" + (' ≥上限' if p['steps'] >= 3276 else ''))
            self.values['heading'].set(f"{p['heading']:.1f}°" if p['heading_valid'] else '无效')
            self.values['distance'].set(f"{p['distance']:.1f} m")
            recent = [x for x in self.history if now - x['received'] < 5]
            hz = (len(recent)-1)/(recent[-1]['received']-recent[0]['received']) if len(recent)>1 else 0
            self.values['rate'].set(f'{hz:.1f} Hz')
            acc = str(p['acc']) if p['acc_fresh'] else '无效/过期'
            mag = str(p['mag']) if p['mag_fresh'] else '无效/过期'
            self.axes.set(f'Acc XYZ: {acc} mg     Mag XYZ: {mag} mGauss')
        else:
            for value in self.values.values(): value.set('—')
            self.axes.set('尚无实时数据' if not self.latest else '数据过期 · 请检查连接和设备供电')
        self.stats.set(f'接收 {self.total} 包  |  格式错误 {self.bad}  |  显示队列丢弃 {self.receiver.display_dropped}  |  最近 15 秒曲线，X 红 / Y 绿 / Z 蓝')
        self.draw(now)
        if not self.closing: self.root.after(100, self.poll)

    def draw(self, now):
        c = self.canvas
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        if w < 100 or h < 100: return
        left, right = 65, w-18
        colors = ['#cc4353', '#16845d', '#306bc0']
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
            c.create_text(right,bottom+12,text='现在',anchor='e',fill='#6b7280')


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
