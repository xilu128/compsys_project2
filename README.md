# CS704 Project 2 — Group 02 — SensorTile motion tracking

This implementation extends the supplied COMSYS704 starter. It acquires real
LSM303AGR acceleration/magnetic field, detects steps and estimates magnetic
heading with tilt compensation, and sends measurements through the existing
ST BLE Sensor application interface. Distance is an additional fixed-stride
estimate, not an independently measured position.

**Verification boundary:** compiled firmware and synthetic desktop tests are
provided. No SensorTile was connected during development. SPI timing, radio
operation, mounting axes, calibration and human-motion accuracy still need the
hardware acceptance tests below. The default magnetic calibration is identity;
it is not a measured calibration and accuracy must not be claimed from it.

## Build in STM32CubeIDE

Validated toolchain: STM32CubeIDE 2.2.0 on macOS Apple Silicon, bundled GNU Tools
for STM32 14.3.rel1 / GCC 14.3.1. The linker uses `READONLY` sections supported by
this toolchain. Use this version rather than the starter's old GCC 7 setting.

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

On this Mac the optional command-line equivalent is:

```sh
sh tools/build_macos.sh
```

The script uses a separate workspace beside this folder, so the IDE workspace
can stay open. Override `STM32CUBEIDE_EXECUTABLE` or `P2_BUILD_WORKSPACE` if needed.
No build script flashes hardware.

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

Default BLE name is `ABCDEFG`, retained from the starter. Connect using the
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
UART diagnostics report the full count, heading validity and bus error count.

UART5 is 115200 baud, 8 data bits, no parity, 1 stop bit. Use the course hardware
wiring; this build does not enable USB CDC. Logs appear once per second by
default, including `mag=x,y,z`. Initialization failure produces a fast LED
blink; normal disconnected advertising uses a slower blink. A bus error holds
previous displayed raw values, increments `errors`, and eventually invalidates
heading; lack of data-ready also invalidates heading after the freshness limit.

## Calibration and tuning before evaluation

1. Set `P2_LOG_PERIOD_MS` to `100U`, rebuild, and capture UART output while
   rotating the **assembled wearable** slowly through all 3D orientations for
   at least 30 seconds, away from metal, magnets and power equipment.
2. Save the capture as `rotation.log`, then run:

   ```sh
   python3 tools/calibrate_mag.py rotation.log
   ```

3. The tool prints `P2_MAG_BIAS_*` and `P2_MAG_SCALE_*` definitions. Copy them
   into `Inc/p2_config.h`, restore the normal logging period, rebuild and flash.
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
The image is not standalone at address zero. Confirm the bootloader with the
lab instructions before flashing. Do not mass erase it or load the application
`.bin` at `0x08000000`. An ELF carries its own application addresses.

Connect SensorTile and STLINK-V3 following the actual board's lab wiring.
The old `COMSYS704-2022 Debug.launch` names a different project; use
`STM32L476JG-SensorTile_ALLMEMS1.launch`, verify project `COMSYS704` and the
current Debug ELF, and select the connected programmer. The starter retains
its STM32L476RG configuration/memory limits; confirm the actual device before
changing these settings. Check both debugger start and independent power reset.

Acceptance checklist (record real results; none are claimed here):

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

On macOS with command-line C tools (or Linux with GCC/Clang):

```sh
sh tests/run_tests.sh
python3 tests/test_calibration.py
```

The C tests compile only the portable project modules, use `-Wall -Wextra
-Werror -pedantic` plus AddressSanitizer/UndefinedBehaviorSanitizer, and exercise
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

Register definitions: supplied LSM303AGR datasheet (DocID027765 Rev 10), sections
8.8, 8.11, 8.39–8.47; supplied AN4825 for sensor operation and compensation.
The BLE and hardware platform originate from the teacher's ST ALLMEMS1 starter;
existing third-party notices are retained.
