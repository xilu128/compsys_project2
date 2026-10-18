# COMPSYS704 Project 2 — Group 02

This SensorTile project measures step count, estimated distance and heading
relative to the starting direction, and sends the results over Bluetooth Low
Energy (BLE). Firmware version: `heading-handheld-10`.

## Requirements

- SensorTile with an ST-LINK programmer and target power for programming.
- STM32CubeIDE 2.2.0 with its bundled GNU Tools for STM32 14.3.rel1
  (GCC 14.3.1). No separate compiler or CubeMX project generation is required.
- On Windows, install the ST-LINK USB and Virtual COM Port drivers.
- Python is optional: it is needed for desktop tools and host tests, not for
  compiling the firmware in CubeIDE.

## Import and compile

1. Extract the complete submission. Keep `Projects`, `Drivers`, `Middlewares`
   and `Utilities` together; the IDE project links to these directories.
2. Open STM32CubeIDE and select **File > Import > General > Existing Projects
   into Workspace**.
3. Select this directory within the extracted source:
   `Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/STM32CubeIDE`.
4. Import the project named **COMSYS704**. Leave **Copy projects into workspace**
   unchecked so the relative source links remain valid.
5. If prompted, select the bundled STM32 toolchain. Set the active build
   configuration to **Debug**, then select **Project > Clean** followed by
   **Project > Build Project**.
6. Confirm that the build finishes without errors. The firmware files are
   `STM32L476JG-SensorTile_ALLMEMS1.elf` and
   `STM32L476JG-SensorTile_ALLMEMS1.bin` in the IDE project's `Debug` directory.

For a Windows command-line build, run from the source root:

```powershell
.\tools\build_windows.ps1 -Configuration Debug -IdeDirectory 'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE'
```

Replace the example path with your installation directory. On macOS, run
`sh tools/build_macos.sh`; this helper builds Debug and Release using the
installed STM32CubeIDE. The helpers build firmware but do not program the board.

## Program and run

1. Connect ST-LINK to the SensorTile debug interface and power the target.
2. In STM32CubeProgrammer, connect through **ST-LINK / SWD**, program the built
   ELF and verify it. If programming the BIN instead, use address `0x08000000`.
3. Reset the board. Hold it steady, away from metal and magnetic interference,
   while initialization completes. In suitable conditions this takes about
   five seconds; movement or inconsistent readings can extend it.
4. Connect to BLE device **P202S01** and enable the motion-data notifications.
   In the ST BLE Sensor app's Gyroscope view, **X shows steps**, **Y shows
   relative heading**, and **Z carries estimated distance**. These fields are
   reused for project outputs, not raw gyro measurements.

The accepted starting direction is 0°. Heading is displayed from 0 to 359°:
clockwise 90° gives 90°, counterclockwise 90° gives 270°, and a full turn returns
to zero. Reset establishes a new origin and clears the count. BLE reconnection
and heading recovery preserve the origin and accumulated steps.

For the interview, charge the battery and verify operation with cables removed
and with the course marking app before the demonstration.

## Code organization

Paths below are relative to
`Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1`.

| File | Responsibility |
|---|---|
| `Src/main.c` | Initialization, sensor scheduling, output variables and BLE/UART integration |
| `Src/p2_sensor.c`, `Inc/p2_sensor.h` | Register configuration, data-ready checks and conversion to physical units |
| `Src/p2_motion.c`, `Inc/p2_motion.h` | Acceleration filtering, step confirmation, handling rejection and heading estimation |
| `Inc/p2_config.h` | Firmware version, thresholds and calibration parameters |
| `Inc/p2_packet.h`, `Src/sensor_service.c` | Output encoding, BLE characteristic updates and notification handling |

The `tests` directory contains host tests and recorded-data replays. The
`tools` directory contains build, capture, visualization, calibration and
packaging scripts. Vendor dependencies are in `Drivers`, `Middlewares` and
`Utilities`.

## Algorithms and limits

Step detection uses filtered acceleration magnitude, peak/trough amplitude
checks and timing checks. Initial candidates are added together after walking
is confirmed. Repeated rocking can suppress detection; isolated very short
walks may remain unconfirmed. Distance is estimated using a fixed 0.70 m per
step, so distance accuracy also depends on the user's actual step length.

Heading uses gyro prediction with selective accelerometer and magnetic
corrections. It is relative to startup, not geographic north. Magnetic
interference and residual gyro bias can affect it, and heading is unavailable
when the board's forward axis (+X) is too close to vertical. Recovery retains
the original zero and step count.

See [VALIDATION.md](VALIDATION.md) for measured results, replay results and
remaining validation work. Host tests and replays do not replace physical
walking and turn-angle tests.

## Optional tools and tests

Install the Python dependencies from the source root:

```powershell
python -m pip install -r tools/requirements.txt
python tools/ble_monitor.py
```

The viewer also needs Tkinter. For serial recording:

```powershell
python tools/capture_log.py --name walk_100_normal_02
```

UART uses **115200 baud, 8N1**. Select the ST-LINK Virtual COM Port, not a
Bluetooth serial port. Run only one capture process per port. Stop with Ctrl+C;
the tool prints the CSV and raw-log locations when recording starts.

Host tests need a desktop C compiler in addition to Python and the dependencies
above. On Windows, for example:

```powershell
.\tests\run_tests_windows.ps1 -Compiler 'C:\tools\zig\zig.exe' -Python python
python tests/test_submission.py
```

Replace the compiler path with your installed compiler. On macOS or Linux,
run `sh tests/run_tests.sh` and `python3 tests/test_submission.py`.

## Submission and licensing

See [SUBMISSION.md](SUBMISSION.md) for archive creation and additional setup
instructions. The source packager uses a committed Git revision: uncommitted
changes are not included. Submit `project2_group02.zip` for code and the final
report separately as `project2_group02.pdf`.

Keep vendor copyright and license notices with the source. Earlier development
notes are retained in [docs/DEVELOPMENT_HISTORY.md](docs/DEVELOPMENT_HISTORY.md);
the current instructions in this README and SUBMISSION.md take precedence.
