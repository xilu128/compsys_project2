#!/usr/bin/env python3
"""Replay BLE acceleration/magnetic snapshots through the real C heading code.

The BLE stream is approximately 20 Hz, not the original approximately 50 Hz
sensor input. This is a diagnostic comparison, not a heading-accuracy test.
Requires a native C compiler and only Python's standard library.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


REQUIRED = ("session", "device_tick_8ms", "acc_fresh", "mag_fresh",
            "acc_x_mg", "acc_y_mg", "acc_z_mg",
            "mag_x_mgauss", "mag_y_mgauss", "mag_z_mgauss")
EXTRA = ("replay_device_ms", "replay_heading", "replay_valid", "replay_steps")


def compiler_command(cc):
    """Accept a compiler command, a quoted path, or a bare Zig executable."""
    if Path(cc).is_file():
        command = [cc]
    else:
        command = shlex.split(cc, posix=os.name != "nt")
        if os.name == "nt":
            command = [part[1:-1] if len(part) >= 2 and part[0] == part[-1]
                       and part[0] in "\"'" else part for part in command]
    if not command:
        raise ValueError("The compiler command must not be empty")
    if len(command) == 1 and Path(command[0]).name.lower() in ("zig", "zig.exe"):
        command.append("cc")
    return command


def flag(row, key):
    value = row[key].strip()
    if value not in ("0", "1"):
        raise ValueError(f"{key} must be 0 or 1, got {value!r}")
    return int(value)


def prepare_rows(rows):
    """Unwrap the 16-bit device clock separately within each connection."""
    samples = []
    previous_session = None
    previous_tick = 0
    unwrapped_ms = 0
    epoch = 0
    duration_ms = 0
    wraps = 0
    duplicates = 0
    for number, row in enumerate(rows, start=2):
        try:
            session = row["session"].strip()
            if not session:
                raise ValueError("session must not be empty")
            tick = int(row["device_tick_8ms"])
            if not 0 <= tick <= 65535:
                raise ValueError("device_tick_8ms must be an unsigned 16-bit value")
            if session != previous_session:
                epoch += 1
                unwrapped_ms = tick * 8
            else:
                delta = (tick - previous_tick) & 0xffff
                if delta >= 0x8000:
                    raise ValueError("device clock moved backwards or has an ambiguous "
                                     "gap >= 262.144 s; split/reset the session first")
                wraps += int(tick < previous_tick)
                duplicates += int(delta == 0)
                unwrapped_ms += delta * 8
                duration_ms += delta * 8
            previous_session = session
            previous_tick = tick
            values = [float(row[f"acc_{axis}_mg"]) for axis in "xyz"]
            values += [float(row[f"mag_{axis}_mgauss"]) for axis in "xyz"]
            samples.append((epoch, unwrapped_ms, flag(row, "acc_fresh"),
                            flag(row, "mag_fresh"), *values))
        except (ValueError, TypeError, AttributeError) as error:
            raise ValueError(f"CSV row {number}: {error}") from error
    return samples, {"sessions": epoch, "duration_s": duration_ms / 1000.0,
                     "device_tick_wraps": wraps, "duplicate_ticks": duplicates}


def replay(path, source, include, cc="cc", output=None):
    path, source, include = Path(path), Path(source), Path(include)
    if output is not None and Path(output).resolve() == path.resolve():
        raise ValueError("Output must not overwrite the original input CSV")
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames or []
        missing = [key for key in REQUIRED if key not in fieldnames]
        if missing:
            raise ValueError("CSV is missing required columns: " + ", ".join(missing))
        rows = list(reader)
    if not rows:
        raise ValueError("The recording has no samples")
    samples, timing = prepare_rows(rows)
    runner = Path(__file__).resolve().parents[1] / "tests/heading_replay_runner.c"
    with tempfile.TemporaryDirectory(prefix="p2-heading-replay-") as directory:
        executable = Path(directory) / ("heading_replay.exe" if os.name == "nt" else "heading_replay")
        subprocess.run([*compiler_command(cc), "-std=c99", "-Wall", "-Wextra",
                        "-Werror", "-pedantic", "-I", str(include), str(runner),
                        str(source), "-lm", "-o", str(executable)], check=True)
        # The portable implementation's unsigned milliseconds naturally wraps
        # at 2**32; retain full unwrapped time in the exported CSV.
        data = "".join(" ".join(map(str, (sample[0], sample[1] & 0xffffffff,
                                        *sample[2:]))) + "\n" for sample in samples)
        completed = subprocess.run([str(executable)], input=data, text=True,
                                   capture_output=True, check=True)
    lines = completed.stdout.splitlines()
    if len(lines) != len(rows):
        raise RuntimeError(f"C runner returned {len(lines)} rows for {len(rows)} samples")
    valid_count = 0
    for row, sample, line in zip(rows, samples, lines):
        valid, heading, steps = line.split()
        valid_count += int(valid)
        row.update(replay_device_ms=sample[1], replay_heading=heading,
                   replay_valid=valid, replay_steps=steps)
    if output is not None:
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames +
                                    [key for key in EXTRA if key not in fieldnames])
            writer.writeheader()
            writer.writerows(rows)
    duration = timing["duration_s"]
    result = {"file": str(path), "source": str(source), "include": str(include),
              "rows": len(rows), **timing,
              "snapshot_hz": round((len(rows) - timing["sessions"]) / duration, 3)
              if duration > 0 else None,
              "replay_valid_samples": valid_count,
              "replay_invalid_samples": len(rows) - valid_count,
              "replay_valid_percent": round(valid_count * 100.0 / len(rows), 3),
              "limitation": "Approximately 20 Hz BLE snapshots, not original 50 Hz "
                            "sensor input. Startup state is reconstructed per session; "
                            "results are not physical heading accuracy."}
    if "heading_valid" in fieldnames:
        result["recorded_valid_samples"] = sum(flag(row, "heading_valid") for row in rows)
    if output is not None:
        result["output"] = str(output)
    return result


def main():
    app = Path(__file__).resolve().parents[1] / "Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--source", type=Path, default=app / "Src/p2_motion.c")
    parser.add_argument("--include", type=Path, default=app / "Inc")
    parser.add_argument("--cc", default="cc", help="Compiler command or path to zig.exe")
    parser.add_argument("--output", type=Path, help="Save original columns and replay results")
    args = parser.parse_args()
    try:
        result = replay(args.csv, args.source, args.include, args.cc, args.output)
    except (ValueError, OSError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Heading replay failed: {error}\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
