# CS704 Project 2 — Group 02 — SensorTile motion tracking

This implementation extends the supplied COMSYS704 starter. It acquires real
LSM303AGR acceleration/magnetic field, detects steps and estimates magnetic
heading with tilt compensation, and sends measurements through the existing
ST BLE Sensor application interface. Distance is an additional fixed-stride
estimate, not an independently measured position.

**Verification boundary (Windows, 2026-09-23):** Debug/Release clean builds,
desktop tests, STLINK programming/readback, debugger entry to `main`, UART,
real sensors and Windows BLE notifications/reconnect have passed. See
`VALIDATION.md` and `docs/evidence/`. Phone-app compatibility, mounting axes,
human-motion accuracy and measured calibration remain NOT TESTED. Default
magnetic calibration is identity (UNCALIBRATED), not a measured calibration.

## Scope from the local course brief

The supplied `CS704_Project2_Brief_2026.pdf` is authoritative. The interim
demonstration (6 October) requires raw accelerometer/magnetometer data over
Bluetooth to the course mobile app and interpretation of units. The final
demonstration (20 October) adds steps/orientation and measured accuracy.
Distance is an optional fixed-stride estimate. The existing final-stage motion
module is retained, but no GPS, INS, EKF, custom BLE stack or RTOS is added.
Literature review (6 October) and group report (19 October) are separate
deliverables; real performance measurements still need to be collected.

## Build in STM32CubeIDE

Validated toolchain: STM32CubeIDE 2.2.0 on Windows, bundled GNU Tools for STM32
14.3.rel1 / GCC 14.3.1. Install CubeIDE with its bundled compiler, make, GDB,
ST-LINK GDB server and STM32CubeProgrammer integration, plus the ST-LINK USB
debug and Virtual COM Port drivers. A separate ARM GCC, Keil, IAR, CubeMX or
new STM32 project is not needed. Standalone CubeProgrammer is optional.
The linker uses `READONLY`, requiring a modern linker; do not force the
starter's old GCC 7 setting. Python 3 plus `pyserial`/`bleak` is needed only
for the optional automatic hardware tests; desktop C tests additionally need
a host C compiler (clang, GCC or Zig), not the ARM compiler.

1. Extract the **whole** COMSYS704 folder. Keep Drivers, Middlewares, Utilities
   and Projects together; source and library paths are relative.
2. File → Import → General → Existing Projects into Workspace. Select:
   `Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/STM32CubeIDE`.
3. Import `COMSYS704`; leave **Copy projects into workspace** unchecked.
   For an already imported project, refresh it after external edits.
4. Project → Build Configurations → Set Active → Debug (or Release).
   Project → Clean, then Build Project. Both configurations are supported.
5. Output is `<import-directory>/Debug/STM32L476JG-SensorTile_ALLMEMS1.elf`
   and `.bin` (or the corresponding Release folder).

On Windows, from this source root:

```powershell
.\tools\build_windows.ps1 -Configuration Debug
.\tools\build_windows.ps1 -Configuration Release
```

Discovery uses `STM32CUBEIDE_HOME`, PATH, uninstall registry and typical install
directories. If several installations exist, use `-IdeDirectory <folder>`.
The script uses a separate workspace beside the source root and verifies a
successful build summary plus a freshly generated ELF; it never flashes.
Existing macOS helper `sh tools/build_macos.sh` is retained but not retested on
this Windows host. Preserve the complete Drivers/Middlewares/Utilities tree.
After this validation, generated Debug/Release trees are archived outside the
source root under `../build_artifacts/20260923/`; the next build regenerates them.

## Module layout

Application sources are in `Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1`.

| File | Responsibility |
|---|---|
| `Src/main.c` | Existing platform and SPI transport; bounded waits, polling, BLE scheduling and diagnostics |
| `Src/p2_sensor.c` / `Inc/p2_sensor.h` | Identity/configuration readback, data-ready checks and signed physical-unit conversion; mockable register interface |
| `Src/p2_motion.c` / `Inc/p2_motion.h` | HAL-independent gravity filtering, step detection, distance and tilt-compensated heading |
| `Inc/p2_config.h` | Thresholds, stride, logging interval and measured calibration constants |
| `Inc/p2_packet.h` | Saturating signed 16-bit packet conversion |
| `Src/sensor_service.c` | Existing BLE service; visible as `ble_interface.c` in the IDE |
| `tests/` | Desktop tests with mock sensors and synthetic motion |
| `tools/calibrate_mag.py` | Offline hard-iron/diagonal soft-iron calibration from serial logs |

## Acquisition and algorithms

- Accelerometer: WHO_AM_I `0x33`; CTRL_REG4_A `0x99` (BDU, ±4 g,
  high resolution, three-wire SPI), CTRL_REG1_A `0x47` (50 Hz, XYZ).
  Signed, left-justified 12-bit values use 2 mg/count. Raw physical-unit values
  reach BLE before algorithmic filtering or magnetic calibration.
- Magnetometer: WHO_AM_I `0x40`; CFG_REG_C_M `0x30` (BDU, I2C disabled),
  CFG_REG_B_M `0x03` (internal offset cancellation and LPF), CFG_REG_A_M `0x88`
  (temperature compensation, continuous high-resolution mode, 50 Hz).
  Signed 16-bit sensitivity is 1.5 mGauss/count. Internal offset cancellation
  does not replace external hard-iron calibration.
- TIM4 polls at 100 Hz (10 kHz / 100), but only fresh data-ready samples are
  processed. Algorithms use actual elapsed milliseconds, not the poll count.
  BLE retains the starter's 20 Hz notification schedule when subscribed.
- Steps: acceleration magnitude removes dependence on fixed orientation. A
  0.6 s low-pass baseline estimates gravity magnitude; a 0.06 s low-pass smooths
  the residual. A negative trough arms detection; a subsequent falling peak
  above 100 mg counts a step, with a 280 ms refractory interval. Startup and
  sampling gaps over 250 ms trigger a one-second settling interval. Counts
  reset on reboot. This is a tunable baseline, not a universal gait classifier.
- Heading: low-pass acceleration estimates up. Subtract magnetic hard-iron bias,
  apply diagonal scale, project magnetic north into the plane normal to up,
  then use `atan2` to find the heading of body +X. Heading is clockwise from
  **magnetic** north in `[0,360)` degrees. No local declination is guessed.
  Sensor register axes are used for both sensors (datasheet Fig. 2). Mount
  **+X forward and +Z up**; confirm board/chip orientation experimentally.
- Reject weak/strong magnetic field, invalid gravity, nearly vertical +X and
  stale data. An invalid heading is sent as -1 degree. Magnitude checks cannot
  detect all magnetic disturbances, particularly uniform field distortions.
- Distance = total steps × configured stride (default 0.70 m). Measure a known
  distance and divide by actual steps to set `P2_STEP_LENGTH_M`.

## Phone and serial data

Advertising name is `ABCDEFG`, retained from the starter; the GAP device-name
characteristic/boot log says `CSys704`. This board's address is
`F9:B0:57:92:6B:52`. Connect using the
course-provided ST BLE Sensor app and enable/subscribe to motion notifications.
The Gyroscope display is **repurposed**, not a measurement of angular velocity.
All multibyte fields below are little endian; values other than timestamp are
signed 16-bit. The timestamp is `(HAL_GetTick() >> 3) & 0xffff` (8 ms ticks).

| Byte offsets | Meaning | Raw packet scale |
|---|---|---|
| 0–1 | Timestamp | 8 ms/tick, wraps |
| 2–7 | Uncalibrated acceleration XYZ | mg |
| 8–9 | Step count in Gyroscope X | steps × 10 |
| 10–11 | Magnetic heading in Gyroscope Y | degrees × 10; -10 means invalid |
| 12–13 | Estimated distance in Gyroscope Z | metres × 10 |
| 14–19 | Uncalibrated magnetic field XYZ | mGauss |

The existing app is expected to divide gyroscope fields by 10; verify this on
its actual version. The internal 32-bit count continues beyond the phone's
3276-step display ceiling; phone steps saturate at 3276 instead of wrapping.
Distance saturates at 3276.7 m and raw fields saturate at int16 limits.
Stale/unavailable acceleration or magnetic XYZ fields use raw `-32768`
(invalid sentinel; the old phone app may display it as a number). Characteristic
UUID is `00e00000-0001-11e1-ac36-0002a5d5c51b`; packet length is 20 bytes.
UART diagnostics report the full count, heading validity and bus error count.
BLE mapping/scaling is unchanged by the 2026-09-26 logging update.

UART5 is 115200 baud, 8 data bits, no parity, 1 stop bit. Use the course hardware
wiring; this build does not enable USB CDC. Structured `P2DATA` logs appear at
10 Hz by default, independently of the 50 Hz sensor ODR, with elapsed
milliseconds, raw axes, counters, heading/validity, distance and BLE/freshness state.
Close any other serial monitor before capturing COM9 (port can vary by PC).
Sensor initialization retries every second without waiting a whole second in
the event loop. Bus failures increment `errors`, invalidate the affected data
and heading; a one-second acquisition stall triggers reinitialization. UART
retains last raw values for diagnostics but marks freshness false; BLE uses
the invalid sentinel. BLE update failures increment `ble_errors`. LED is on
when connected and blinks while disconnected. Counts reset on reboot.

## Calibration and tuning before evaluation

1. Keep `P2_LOG_PERIOD_MS` at its default `100U`, and capture UART output while
   rotating the **assembled wearable** slowly through all 3D orientations for
   at least 30 seconds, away from metal, magnets and power equipment.
2. Save the capture as `rotation.log`, then run:

   ```sh
   python tools/calibrate_mag.py ../logs/magnetometer_calibration_YYYYMMDD_HHMMSS.csv
   ```

3. The tool prints `P2_MAG_BIAS_*` and `P2_MAG_SCALE_*` definitions. Copy them
   into `Inc/p2_config.h`, set `P2_MAG_CALIBRATED` to `1`, rebuild and flash.
   Parameters persist in firmware, not in runtime flash writes.
4. Validate at independent known directions and ±30° tilt. Repeat collection
   if coverage is inadequate. The min/max diagonal fit cannot correct arbitrary
   cross-axis soft-iron distortion. Recalibrate after changing mounting hardware.
5. Test a measured 100-step walk at slow/normal/fast speed and fixed mounting.
   Tune high/low thresholds only against recorded trials, then validate with
   separate trials. Measure stride for distance. Do not tune on the final test.

## Flash layout and first hardware run

The course starter's layout is retained: application/vector table at
**0x08004000**, with a compatible bootloader expected at **0x08000000**.
The image is not standalone at address zero. On this board the bootloader was
missing. A full 1 MB backup was taken before restoring the **supplied**
`Utilities/BootLoader/STM32L476RG/BootLoaderL4.bin`, whose supplied readme also
covers SensorTile. This is not a new/custom bootloader. Application Debug ELF
was then programmed and verified; reset now boots it without a manual jump.
Do not repeat bootloader programming during normal development. Do not mass
erase or load application `.bin` at `0x08000000`. ELF carries its own addresses.
In CubeProgrammer: connect SWD, select the fresh ELF, enable verify, download,
then reset/run. For BIN explicitly use **0x08004000**. Preserve option bytes.

Connect SensorTile and STLINK-V3 following the actual board's lab wiring.
The old `COMSYS704-2022 Debug.launch` names a different project; use
`STM32L476JG-SensorTile_ALLMEMS1.launch`, verify project `COMSYS704` and the
current Debug ELF, and select the connected programmer. The starter retains
its STM32L476RG configuration/memory limits; confirm the actual device before
changing these settings. Check both debugger start and independent power reset.
If normal/hotplug SWD reconnect reports no core ID, select **Connect Under
Reset**, hardware reset, and **1000 kHz**; this recovered the connection here.
Do not use mass erase to solve this connection issue.

Remaining physical acceptance checklist (record real results):

- Reach `main`, see `LSM303AGR ready`; verify no accumulating bus errors.
- Six stationary orientations: the upward acceleration axis should be roughly
  ±1000 mg and magnitude near 1000 mg. Rotation must change magnetic axes.
- Phone raw values and UART agree after unit conversion. Disconnect/reconnect
  and verify notifications resume.
- Calibrate; test north/east/south/west, full 360° turn and tilts. Evaluate
  circular error: `min(abs(a-b), 360-abs(a-b))`.
- Log counts for at least three 100-step trials per speed and mounting position,
  plus two minutes stationary and non-walking handling. Record false positives,
  misses, step error percentage, mean/max heading error and magnetic conditions.
- Verify invalid-heading handling near magnetic interference or vertical +X.
- Reboot without the debugger, reconnect the app and repeat a short walk.

## Desktop verification

Logging regressions run with `python tests/test_logging.py` (pyserial needed for
mocked serial lifecycle tests; no physical board needed). Both platform test
runners include them. Parsing itself uses only the Python standard library.

On Windows:

```powershell
.\tests\run_tests_windows.ps1 -Compiler <host-clang-gcc-or-zig.exe> -Python python
python -m pip install pyserial bleak
python tools/check_hardware.py serial --port COM9 --seconds 125 --output uart.txt
python tools/check_hardware.py ble --seconds 15 --cycles 2 --output ble.json
```

Enable Windows Bluetooth and disconnect the phone before the BLE test. The
scripts only observe/subscribe, never flash. On another board pass `--address`.
On macOS with command-line C tools (or Linux with GCC/Clang):

```sh
sh tests/run_tests.sh
python3 tests/test_calibration.py
python3 tests/test_packet.py
```

The C tests compile only the portable project modules, use `-Wall -Wextra
-Werror -pedantic` (the Unix script also enables sanitizers), and exercise
stationary noise, synthetic 1/2/3 Hz motion, timestamp wrap, distance,
cardinal/tilted headings, invalid inputs, sensor identity/conversion/error
paths and BLE saturation. Synthetic tests establish implementation consistency;
they do **not** establish real gait accuracy. See `VALIDATION.md` for recorded
build/test evidence.

## Limits and submission

Acceleration alone cannot reliably distinguish walking from all repeated
hand movements. Very weak steps, running impacts, loose mounting, movement
while settling and irregular gait can be missed or overcounted. Acceleration
based tilt compensation is less accurate during rapid motion. Magnetic north
is unreliable near steel/electronics; no gyroscope, GPS or absolute indoor
position estimate is implemented. Defaults need hardware calibration/tuning.

The Group 02 submission archive is named `project2_group02.zip`. Keep this README at the extracted project root and keep
all linked dependencies. Exclude generated Debug/Release directories and IDE
workspace metadata. The separate literature review and report require the
specified templates, your own comparison and **measured** performance results;
this code does not fabricate those deliverables.

Create a source-only archive with `python tools/package_submission.py --group 02`.
The archive contains a complete `COMSYS704/` root and verifies ZIP CRCs. It does
not overwrite an existing archive. Generated outputs and caches are excluded;
vendor libraries and supplied bootloader binaries are intentionally retained.
The actual working folder remains `compsys_project2`; renaming is unnecessary.

Register definitions: supplied LSM303AGR datasheet (DocID027765 Rev 10), sections
8.8, 8.11, 8.39–8.47; supplied AN4825 for sensor operation and compensation.
The BLE and hardware platform originate from the teacher's ST ALLMEMS1 starter;
existing third-party notices are retained.

## Data Logging

From the source root (`compsys_project2`), install the one capture dependency:

```powershell
python -m pip install pyserial
python tools/capture_log.py --name walk_100_normal_01
```

On this Windows machine, the default `python` resolves to Anaconda and currently
does **not** have pyserial; the install command above is therefore required for
that interpreter. No package was silently installed. The existing validation
environment already has it and can be used immediately instead:

```powershell
..\.p2-validation\Scripts\python.exe tools/capture_log.py --name walk_100_normal_01
```

The script lists COM ports and automatically chooses a unique ST-LINK VCP.
With multiple/no identified candidates, select a listed port; override with
`--port COM9`. Close other serial monitors first. UART is **115200 8N1**.
No dependency is installed automatically. Run without `--name` for `capture`.
Names use letters/digits/underscore/hyphen. Optional `--seconds 30` stops a test
automatically. The capture tool never flashes, resets or changes the device.

Default output is **../logs/** beside the source directory, independent of
the terminal working directory, so experiment data is outside source:

```text
COMPSYS704_Project2/
  compsys_project2/tools/capture_log.py
  logs/walk_100_normal_01_YYYYMMDD_HHMMSS.log
  logs/walk_100_normal_01_YYYYMMDD_HHMMSS.csv
```

Use `--output-dir <folder>` if needed. Existing files are never overwritten;
same-second collisions receive a numeric suffix. Local `logs/*.csv`/`*.log`
are ignored, and the submission packager excludes logs directories entirely.
The previous 23 September ZIP is a historical baseline, not this logging update.

Every telemetry record starts `P2DATA,` and ends CRLF; information starts
`P2INFO,` and errors start `P2ERROR,`. Example:

```text
P2DATA,ms=1000,acc_x=1,acc_y=2,acc_z=999,mag_x=100,mag_y=-50,mag_z=300,steps=10,heading=90.0,heading_valid=1,distance=7.0,sensor_errors=0,ble_connected=1,ble_subscribed=1,ble_errors=0,sample_count=50,mag_sample_count=50,acc_fresh=1,mag_fresh=1,log_dropped=0
```

CSV columns, in order:

```text
host_time_iso,device_ms,acc_x_mg,acc_y_mg,acc_z_mg,mag_x_mgauss,mag_y_mgauss,mag_z_mgauss,step_count,heading_deg,heading_valid,distance_m,sensor_error_count,ble_connected,ble_subscribed,ble_error_count,sample_count,mag_sample_count,acc_fresh,mag_fresh,log_dropped
```

- `host_time_iso`: PC arrival time, ISO 8601 milliseconds and timezone offset;
  not an exact sensor-sampling timestamp. `device_ms`: uint32 MCU milliseconds,
  wraps after about 49.7 days and resets on boot. Split rebooted experiments.
- Acceleration is **mg**, magnetic field **mGauss**, magnetic heading **degrees**,
  distance **metres**. `step_count` is cumulative since boot, not per-log steps.
- `heading_valid=0` means `heading_deg=-1.0`: exclude it from angle analysis.
  Freshness flags distinguish last-held sensor values from current data.
- `sample_count` counts accepted acceleration reads; `mag_sample_count` counts
  magnetic reads. They are not CSV row counts. Counter differences measure ODR.
- Error counts are cumulative; BLE connection/subscription are 0/1.
  `log_dropped` counts whole UART records dropped on queue overflow/format error.
  Missing optional counters from older P2DATA senders produce blank CSV cells.

The bounded 8 x 512-byte RAM queue uses the existing UART interrupt transmitter;
it never waits for the wire and never writes MCU flash or enables SD logging.
10 Hz logging is decimated telemetry, **not** a complete 50 Hz sensor waveform.
Do not use it to claim that every sensor sample was recorded. The measured
sensor/BLE rates remained about 51/20 Hz in the hardware regression.

Raw logs preserve all received bytes, including startup/errors/malformed lines.
Only complete validated data rows enter CSV. A malformed row increments the
reported rejection count without stopping capture. Ctrl+C requests graceful
stop: drain a bounded receive buffer, allow up to 250 ms to complete an in-flight
line, flush/close both files and the port, then print counts and paths. If a
line remains incomplete, it stays raw and is not fabricated into a CSV row.
Unplugging reports connection loss without a traceback; captured rows remain.
Files are flushed throughout capture; forced termination/power loss is not
guaranteed to preserve OS/device-buffered data.

Useful experiment names: `stationary_2min`, `walk_20_normal_01`,
`walk_100_slow_01`, `walk_100_normal_01`, `walk_100_fast_01`,
`heading_cardinal`, `magnetometer_calibration`.

```powershell
python tools/summarize_log.py ../logs/walk_100_normal_01_YYYYMMDD_HHMMSS.csv
python tools/calibrate_mag.py ../logs/magnetometer_calibration_YYYYMMDD_HHMMSS.csv
```

The summary reports duration, start/end/delta steps, distance increase, heading
validity and error increase; it rejects detected resets rather than giving a
misleading negative count. Calibration accepts new CSV/P2DATA raw logs and old
`mag=x,y,z` logs. CSV stale magnetic rows are excluded; calibration still needs
full 3D rotations, not a stationary/walking log. No calibration constants or
motion algorithm parameters were changed by the logging work.
