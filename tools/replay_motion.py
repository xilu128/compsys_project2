#!/usr/bin/env python3
"""Compare portable motion code against recorded acceleration snapshots.

This is a reduced-rate replay, NOT an exact replay of the firmware's 50 Hz
input. Do not use its count to claim hardware accuracy or tune to a target.
Requires a native C compiler; uses only Python's standard library.
"""
import argparse
import csv
import json
from pathlib import Path
import shlex
import subprocess
import tempfile

RUNNER = r'''
#include "p2_motion.h"
#include <inttypes.h>
#include <stdio.h>
int main(void) {
    P2_Motion s;
    P2_MotionInit(&s);
    uint32_t ms;
    P2_Vector a;
    while (scanf("%" SCNu32 " %f %f %f", &ms, &a.x, &a.y, &a.z) == 4)
        P2_MotionAcceleration(&s, a, ms);
    printf("%" PRIu32 "\n", s.steps);
    return 0;
}
'''


def replay(path, source, include, cc="cc"):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError("At least two samples are required")
    values = [(int(r["device_ms"]), *(float(r[f"acc_{axis}_mg"]) for axis in "xyz"))
              for r in rows]
    duration = ((values[-1][0] - values[0][0]) & 0xffffffff) / 1000
    if duration <= 0:
        raise ValueError("Non-positive recording duration")
    for before, after in zip(values, values[1:]):
        if not 0 < ((after[0]-before[0]) & 0xffffffff) < 0x80000000:
            raise ValueError("Duplicate timestamps or device reset; split recording first")
    with tempfile.TemporaryDirectory(prefix="p2-replay-") as directory:
        directory = Path(directory)
        runner = directory / "replay.c"
        runner.write_text(RUNNER, encoding="utf-8")
        executable = directory / "replay"
        subprocess.run([*shlex.split(cc), "-std=c99", "-Wall", "-Wextra", "-Werror",
                        "-I", str(include), str(runner), str(source), "-lm", "-o", str(executable)], check=True)
        data = "".join(" ".join(map(str, row)) + "\n" for row in values)
        count = int(subprocess.run([str(executable)], input=data, text=True,
                                  capture_output=True, check=True).stdout)
    result = {"file": path.name, "rows": len(rows), "duration_s": duration,
              "snapshot_hz": round((len(rows)-1)/duration, 3), "replayed_steps": count,
              "limitation": "Downsampled snapshots; not the original full-rate sensor stream."}
    if "step_count" in rows[0]:
        result["recorded_step_delta"] = int(rows[-1]["step_count"]) - int(rows[0]["step_count"])
    return result


def main():
    app = Path(__file__).resolve().parents[1] / "Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--source", type=Path, default=app / "Src/p2_motion.c")
    parser.add_argument("--include", type=Path, default=app / "Inc")
    parser.add_argument("--cc", default="cc")
    args = parser.parse_args()
    print(json.dumps(replay(args.csv, args.source, args.include, args.cc), indent=2))


if __name__ == "__main__":
    main()
