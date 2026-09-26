#!/usr/bin/env python3
"""SensorTile UART -> loss-visible raw bytes and validated CSV; no device writes."""
import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import re
import signal
import sys
import threading
import time

FIELDS = {
    'ms': 'device_ms', 'acc_x': 'acc_x_mg', 'acc_y': 'acc_y_mg',
    'acc_z': 'acc_z_mg', 'mag_x': 'mag_x_mgauss', 'mag_y': 'mag_y_mgauss',
    'mag_z': 'mag_z_mgauss', 'steps': 'step_count', 'heading': 'heading_deg',
    'heading_valid': 'heading_valid', 'distance': 'distance_m',
    'sensor_errors': 'sensor_error_count', 'ble_connected': 'ble_connected',
    'ble_subscribed': 'ble_subscribed', 'ble_errors': 'ble_error_count',
}
OPTIONAL = {'sample_count': 'sample_count', 'mag_sample_count': 'mag_sample_count',
            'acc_fresh': 'acc_fresh', 'mag_fresh': 'mag_fresh', 'log_dropped': 'log_dropped'}
COLUMNS = ['host_time_iso', *FIELDS.values(), *OPTIONAL.values()]
FLOATS = {'heading', 'distance'}
FLAGS = {'heading_valid', 'ble_connected', 'ble_subscribed', 'acc_fresh', 'mag_fresh'}
COUNTERS = {'ms', 'steps', 'sensor_errors', 'ble_errors', 'sample_count',
            'mag_sample_count', 'log_dropped'}


def parse_line(line, host_time=None):
    """None for INFO/ERROR/legacy text; ValueError for malformed P2DATA.

    Optional fields stay blank on older senders. Unknown fields are permitted
    for forward compatibility; duplicates, nonfinite values and partial rows
    are not accepted. No stale or invalid values are silently made valid.
    """
    if not line.startswith('P2DATA,'):
        return None
    values = {}
    for item in line.rstrip('\r\n').split(',')[1:]:
        key, sep, value = item.partition('=')
        if not sep or not key or not value or key in values:
            raise ValueError('Malformed or duplicate field')
        values[key] = value
    if not FIELDS.keys() <= values.keys():
        raise ValueError('Missing required telemetry field')
    row = {'host_time_iso': host_time or datetime.now().astimezone().isoformat(timespec='milliseconds')}
    for key, column in {**FIELDS, **OPTIONAL}.items():
        if key not in values:
            row[column] = ''
            continue
        value = values[key]
        if key in FLOATS:
            number = float(value)
            if not math.isfinite(number):
                raise ValueError('Nonfinite telemetry')
        else:
            if not re.fullmatch(r'-?\d+', value):
                raise ValueError('Invalid integer')
            number = int(value)
        if key in FLAGS and number not in (0, 1):
            raise ValueError('Invalid boolean')
        if key in COUNTERS and not 0 <= number <= 0xffffffff:
            raise ValueError('Invalid uint32 counter')
        if key == 'distance' and number < 0:
            raise ValueError('Negative distance')
        row[column] = number
    if row['heading_valid']:
        if not 0 <= row['heading_deg'] < 360:
            raise ValueError('Invalid valid-heading range')
    elif row['heading_deg'] != -1:
        raise ValueError('Invalid heading must use -1 sentinel')
    return row


class Recorder:
    """Incremental framing handles serial timeouts mid-line and limits RAM use."""
    def __init__(self, raw, csv_file):
        self.raw, self.csv_file = raw, csv_file
        self.writer = csv.DictWriter(csv_file, fieldnames=COLUMNS)
        self.writer.writeheader()
        csv_file.flush()
        self.pending = bytearray()
        self.discarding = False
        self.samples = self.malformed = 0

    def feed(self, chunk):
        self.raw.write(chunk)  # All bytes, including startup/error/malformed/partial lines.
        self.raw.flush()
        for byte in chunk:
            if self.discarding:
                if byte == 10:
                    self.discarding = False
                continue
            self.pending.append(byte)
            if len(self.pending) > 4096:
                self.pending.clear()
                self.discarding = byte != 10
                self.malformed += 1
                continue
            if byte != 10:
                continue
            line = bytes(self.pending)
            self.pending.clear()
            try:
                row = parse_line(line.decode('ascii'))
                if row is not None:
                    self.writer.writerow(row)
                    self.samples += 1
            except (ValueError, UnicodeError):
                self.malformed += 1
        self.csv_file.flush()


def choose_port(ports, requested=None, ask=input):
    ports = sorted(ports, key=lambda p: p.device)
    print('Available serial ports:')
    for index, port in enumerate(ports, 1):
        print(f'  {index}. {port.device}: {port.description}')
    if requested:
        return requested
    candidates = [p for p in ports if getattr(p, 'vid', None) == 0x0483
                  and ('STLINK' in p.description.upper().replace('-', '')
                       or getattr(p, 'pid', None) in (0x374b, 0x374e, 0x374f, 0x3753))]
    if len(candidates) == 1:
        print(f'Automatically selected ST-LINK UART: {candidates[0].device}')
        return candidates[0].device
    if not ports:
        raise ValueError('No serial ports found. Connect ST-LINK and its UART wiring.')
    # Never silently open an unrelated Bluetooth/USB serial device.
    while True:
        answer = ask('Select port number (or q to cancel): ').strip()
        if answer.lower() == 'q':
            raise ValueError('Port selection cancelled.')
        if answer.isdigit() and 1 <= int(answer) <= len(ports):
            return ports[int(answer)-1].device
        print('Enter one of the listed numbers.')


def create_outputs(directory, name):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', name):
        raise ValueError('Name must use 1–80 letters/digits/underscore/hyphen, starting with a letter/digit.')
    directory.mkdir(parents=True, exist_ok=True)
    stem = name + '_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    for suffix in range(1000):
        base = stem + (f'_{suffix:03d}' if suffix else '')
        raw_path, csv_path = directory/(base+'.log'), directory/(base+'.csv')
        if csv_path.exists():
            continue
        try:
            raw = raw_path.open('xb')
        except FileExistsError:
            continue
        try:
            csv_file = csv_path.open('x', newline='', encoding='utf-8')
        except BaseException:
            raw.close()
            raise
        return raw_path, csv_path, raw, csv_file
    raise ValueError('Too many filename collisions.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='capture')
    parser.add_argument('--port', help='Explicit COM port; otherwise list/select automatically')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).resolve().parents[2]/'logs',
                        help='Default: logs beside, not inside, the source project')
    parser.add_argument('--seconds', type=float, help='Optional timed capture; default runs until Ctrl+C')
    args = parser.parse_args(argv)
    if args.baud <= 0 or (args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0)):
        parser.error('baud and seconds must be positive and finite')
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print('Missing dependency. Install with: pip install pyserial', file=sys.stderr)
        return 1
    stop = threading.Event()
    previous_handler = None
    recorder = None
    raw_path = csv_path = None
    status = 0
    try:
        port_name = choose_port(list_ports.comports(), args.port)
        # No firmware reset command or input-buffer purge.
        with serial.Serial(port_name, args.baud, bytesize=8, parity='N', stopbits=1,
                           timeout=0.2, rtscts=False, dsrdtr=False) as port:
            raw_path, csv_path, raw, csv_file = create_outputs(args.output_dir.resolve(), args.name)
            with raw, csv_file:
                recorder = Recorder(raw, csv_file)
                previous_handler = signal.signal(signal.SIGINT, lambda *_: stop.set())
                print(f'Capturing {port_name} at {args.baud} 8N1. Ctrl+C to stop.\nCSV: {csv_path}\nRaw log: {raw_path}', flush=True)
                deadline = time.monotonic()+args.seconds if args.seconds else math.inf
                try:
                    while not stop.is_set() and time.monotonic() < deadline:
                        chunk = port.read(min(max(port.in_waiting, 1), 4096))
                        if chunk:
                            recorder.feed(chunk)
                    # Drain the bytes already buffered at stop, not an unbounded
                    # stream. A trailing incomplete record stays in the raw log.
                    remaining = min(port.in_waiting, 65536)
                    while remaining:
                        chunk = port.read(min(remaining, 4096))
                        if not chunk:
                            break
                        recorder.feed(chunk)
                        remaining -= len(chunk)
                    # Finish a line already in flight, with a bounded grace period.
                    # On unplug/timeout its partial bytes are still preserved raw.
                    finish_by = time.monotonic() + 0.25
                    while recorder.pending and time.monotonic() < finish_by:
                        chunk = port.read(1)
                        if chunk:
                            recorder.feed(chunk)
                finally:
                    raw.flush()
                    csv_file.flush()
    except serial.SerialException as error:
        print(f'Serial connection lost.' if recorder else f'Cannot open serial port: {error}', file=sys.stderr)
        status = 1
    except (OSError, ValueError, EOFError) as error:
        print(f'Capture error: {error}', file=sys.stderr)
        status = 1
    except KeyboardInterrupt:
        pass
    finally:
        if previous_handler is not None:
            signal.signal(signal.SIGINT, previous_handler)
        if recorder is not None:
            print(f'Capture stopped.\nSamples recorded: {recorder.samples}\nMalformed lines: {recorder.malformed}\n'
                  f'Unterminated bytes preserved in raw log: {len(recorder.pending)}\nCSV: {csv_path}\nRaw log: {raw_path}')
            if not recorder.samples:
                print('No P2DATA records. Check firmware version, UART wiring and baud rate.', file=sys.stderr)
                status = 1
    return status


if __name__ == '__main__':
    raise SystemExit(main())
