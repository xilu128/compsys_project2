#!/usr/bin/env python3
"""Capture real UART/BLE data without programming firmware.

Install: python -m pip install pyserial bleak
UART: python tools/check_hardware.py serial --port COM9 --seconds 120
BLE:  python tools/check_hardware.py ble --address F9:B0:57:92:6B:52 --seconds 15 --cycles 2
The BLE mode subscribes, records all 20-byte course packets, disconnects and
reconnects. A passing result verifies transport, not human gait/heading accuracy.
"""
import argparse
import asyncio
import json
import math
import struct
import time
from pathlib import Path

MOTION_UUID = "00e00000-0001-11e1-ac36-0002a5d5c51b"


def decode(data):
    if len(data) != 20:
        raise ValueError(f"Expected 20-byte motion packet, got {len(data)}")
    fields = struct.unpack("<H9h", data)
    return dict(tick=fields[0], acc=fields[1:4], steps=fields[4]/10,
                heading=fields[5]/10, distance=fields[6]/10, mag=fields[7:10])


async def ble_capture(args):
    from bleak import BleakClient, BleakScanner
    cycles = []
    for cycle in range(args.cycles):
        device = await BleakScanner.find_device_by_address(args.address, timeout=15)
        if device is None:
            raise RuntimeError(f"Target {args.address} not advertising (disconnect phone first)")
        records, invalid = [], []
        async with BleakClient(device, timeout=20) as client:
            if client.services.get_characteristic(MOTION_UUID) is None:
                raise RuntimeError("Course motion characteristic not found")

            def received(_, data):
                try:
                    records.append(dict(received_s=time.monotonic(), **decode(data)))
                except ValueError as error:
                    invalid.append(str(error))

            await client.start_notify(MOTION_UUID, received)
            await asyncio.sleep(args.seconds)
            await client.stop_notify(MOTION_UUID)
        duration = records[-1]["received_s"]-records[0]["received_s"] if len(records)>1 else 0
        rate = (len(records)-1)/duration if duration else 0
        advancing = any(a["tick"] != b["tick"] for a,b in zip(records,records[1:]))
        summary = dict(cycle=cycle+1, packets=len(records), malformed=invalid,
                       rate_hz=round(rate,2), timestamp_advancing=advancing,
                       first=records[0] if records else None, last=records[-1] if records else None)
        print(json.dumps(summary), flush=True)
        cycles.append(dict(summary=summary, records=records))
        args.output.write_text(json.dumps(cycles,indent=2), encoding="utf-8")
        if invalid or not advancing or not 15 <= rate <= 25:
            raise RuntimeError("BLE transport test failed; inspect captured evidence")
        if cycle+1 < args.cycles:
            await asyncio.sleep(2)


def serial_capture(args):
    import serial
    deadline = time.monotonic()+args.seconds
    count = 0
    with serial.Serial(args.port,115200,timeout=0.25) as port, args.output.open("w",encoding="utf-8") as log:
        while time.monotonic()<deadline:
            line = port.readline().decode("ascii",errors="replace").strip()
            if line:
                print(line,flush=True)
                log.write(line+"\n")
                log.flush()
                if line.startswith(("steps=", "P2DATA,")):
                    count += 1
    if count == 0:
        raise RuntimeError("No motion telemetry; check bootloader, power and UART wiring")
    print(f"Captured {count} telemetry lines to {args.output}")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode",choices=["serial","ble"])
    p.add_argument("--port",default="COM9")
    p.add_argument("--address",default="F9:B0:57:92:6B:52")
    p.add_argument("--seconds",type=float,default=15)
    p.add_argument("--cycles",type=int,default=2)
    p.add_argument("--output",type=Path)
    args=p.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=0 or args.cycles<1:
        p.error("seconds and cycles must be positive")
    args.output = args.output or Path("uart.log" if args.mode=="serial" else "ble.json")
    if args.mode=="serial":
        serial_capture(args)
    else:
        asyncio.run(ble_capture(args))


if __name__=="__main__":
    main()
