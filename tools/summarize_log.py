#!/usr/bin/env python3
"""Summarize captured CSV; reject resets/gaps ambiguous across uint32 wrap."""
import argparse
import csv
from datetime import datetime
from pathlib import Path


def summarize(rows):
    if not rows:
        raise ValueError('No samples in CSV')
    duration_ms = 0
    for previous, current in zip(rows, rows[1:]):
        delta = (int(current['device_ms']) - int(previous['device_ms'])) & 0xffffffff
        if delta > 0x7fffffff or int(current['step_count']) < int(previous['step_count']):
            raise ValueError('Device reset/counter decrease detected; summarize separate runs')
        duration_ms += delta
    first, last = rows[0], rows[-1]
    errors = int(last['sensor_error_count'])-int(first['sensor_error_count'])
    if errors < 0:
        raise ValueError('Sensor error counter reset; split the capture')
    return {'Duration (device s)': duration_ms/1000,
            'Duration (host s)': (datetime.fromisoformat(last['host_time_iso'])-datetime.fromisoformat(first['host_time_iso'])).total_seconds(),
            'Samples': len(rows), 'Start steps': int(first['step_count']),
            'End steps': int(last['step_count']),
            'Detected steps': int(last['step_count'])-int(first['step_count']),
            'Distance increase (m)': float(last['distance_m'])-float(first['distance_m']),
            'Heading valid (%)': sum(int(r['heading_valid']) == 1 for r in rows)*100/len(rows),
            'Sensor error increase': errors}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv', type=Path)
    args = p.parse_args()
    try:
        with args.csv.open(newline='', encoding='utf-8-sig') as source:
            result = summarize(list(csv.DictReader(source)))
    except (OSError, ValueError, KeyError, TypeError) as error:
        p.error(str(error))
    for name, value in result.items():
        print(f'{name}: {value}')


if __name__ == '__main__':
    main()
