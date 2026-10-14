# Historical development notes

Archived before submission preparation. Some sections describe superseded
firmware behaviour; use the root README and SUBMISSION.md for current operation.

> Flash layout update: current builds run standalone at **0x08000000**.
> Follow **Flash layout and first hardware run** below. Historical 0x08004000
> programming results elsewhere in this document apply only to earlier builds.

# CS704 Project 2 — Group 02 — SensorTile motion tracking

## Current demo: interview heading (`heading-handheld-10`)

The teacher's **CS704 final interview guideline 2026** supersedes the former
signed-cycle display: startup is 0, clockwise 90 is 90, counterclockwise 90 is
270, and output is a whole number **0 through 359 degrees**. This revision
changes angle presentation/encoding and shortens stationary startup confirmation;
step detection, post-startup gyro fusion, magnetic aiding and tilt compensation
retain the version 7 logic.

This revision supersedes the absolute/reference-aligned heading descriptions
below. Hold the board gently, away from metal, **until calibration finishes**;
small hand tremor is allowed, but deliberate turns are not calibration motion.
The LSM6DSM gyro first needs a continuous 2-second bias window. Then heading
requires a continuous 3-second bounded-motion magnetic/acceleration window: three
1-second circular magnetic-heading means must span no more than 1 degree, with
at least 150 distinct magnetic samples. Two consecutive same-direction block
changes both above 0.125 degree reject persistent drift/turning; small reversing
noise no longer fails solely for exceeding the former 0.25-degree span. The
gyro calibration remains 2 seconds with at least 150 fresh gyro samples; step
settling and post-startup correction thresholds are unchanged.
Ideal startup takes about 5 seconds rather than 8; excessive movement or a still-changing magnetic field
extends it. There is no timeout that forces an unstable zero. Do not slowly rotate
during startup. Shorter averages can be more sensitive to noise; actual cold-start
duration requires hardware measurement. The same window
averages magnetic XYZ and gravity XYZ, and refines
gyro bias; that averaged tilt-compensated direction defines **0 degrees**.
After zero is established, neither the origin nor gyro bias is continuously
relearned. Real slow turns remain observable; the display is not held at zero.

Version 10 widens only the acquisition/recovery motion envelope: provisional
gyro calibration allows 10 dps magnitude/spread and 3-degree gravity-direction
variation; magnetic startup allows 6 dps and 5 degrees. Recovery allows 15 dps
and 5 degrees, still requiring 2 seconds and 80 fresh magnetic samples. Existing
acceleration, magnetic-strength/dip, freshness and heading-consistency guards
remain active. Recovery preserves the original zero, gyro bias and step count.
Large swings, walking and changing magnetic interference may still delay it.
The Windows regression suite passes, including 48 synthetic handheld cases
(startup 5.02 s, recovery 2 s, maximum subsequent 120-second drift 0.7324 degrees).
Four recorded motion replays also pass; these are downsampled development logs,
not a replacement for full-rate physical walking/handheld acceptance testing.
The separate extreme-boundary diagnostic still exceeds the strict 5-degree
target in 4 of 6 cases (maximum 5.1999 degrees), unchanged from version 9.

Clockwise rotation displays 90, 180, 270, then 0 at a full turn.
Counterclockwise rotation displays 270, 180, 90, then 0.
The internal angle is `(fused_heading - startup_origin) modulo 360`, with no
signed winding or route-dependent branch. Internal sub-degree precision is
retained; presentation rounds to the nearest degree and applies modulo 360
again so that 359.5 rounds to 0, never 360. Crossing startup displays
357 -> 359 -> 0 -> 3. UART, BLE and the dial use the same whole-degree value.
Rounding is not a claim of exact physical return. This is horizontal heading
of board +X, not rotation
around X and not a full 3D attitude. No roll/pitch display is implemented.
The updated desktop viewer uses 0/90/180/270 labels; restart it after updating.
It does not re-zero on connection or silently wrap older signed firmware values.
Old negative/fractional/out-of-range headings produce a protocol mismatch warning.

`COMP_Value.Steps` holds the true step count; `COMP_Value.Heading` holds the true
whole-degree heading 0..359 when valid. BLE bytes 10-11 are this heading times ten:
e.g. 270 degrees is raw 2700, so the ST app's gyro_y displays 270. Steps at
bytes 8-9 are likewise scaled once for gyro_x. No BLE layout/UUID changes.
Raw int16 codes -32768/-32767/-32766/-32765 mean unavailable/calibrating/
disturbed/recovering; -32764 means invalid numerical angle. These sentinels are
not multiplied by ten. Do not present unavailable data as a valid zero.
UART keeps its explicit heading_valid flag and generic -1 invalid marker.
Log tools still accept historical signed recordings; new valid output is 0..359.

### Interview preparation and compatibility checks

1. Charge the board, disconnect every USB/debug/UART cable, then start it from
   battery while held still until its first valid zero. The guideline requires
   a cable-free device; USB-powered success alone is not this acceptance test.
2. Connect the supplied marking app via BLE and subscribe to motion data.
   Check actual steps on ST gyro_x and actual heading on ST gyro_y, not scaled
   values such as 900/2700. The marking app itself was not supplied in either PDF.
3. At one fixed location, pause at clockwise 90/180/270/0 and counterclockwise
   270/180/90/0. Compare stopped angles with known directions, then walk the
   requested number of steps. The guideline allows two attempts, best retained.
4. The **CS704 Flash Location Fix** is a startup/linker correction, not marking
   software. Current sources already have `VECT_TAB_OFFSET=0x00` and linker
   `ORIGIN=0x08000000`; these are retained. The PDF's old-address prose has a
   typo; its requested final address and screenshot agree on `0x08000000`.

Software mapping/regression tests do not certify real marking-app operation,
cable-free battery startup, or physical heading accuracy. Repeat those on hardware.

The origin remains fixed through invalid readings, sampling gaps and BLE
reconnections. Only motion-state initialization (normally board restart) clears
it. Invalid startup readings delay zero acquisition. After zero, an inertial
sample loss that discards orientation (gyro gap/error or acceleration reset)
temporarily pauses valid output, then automatically reacquires a stable endpoint.
Keep still for at least two seconds of fresh gyro/acceleration/magnetic data;
at least 80 distinct magnetic samples are needed. A gyro driver restart also
requires its initial bias calibration. Recovery preserves the original zero,
field reference and published step totals; the recovery pose is NOT a new zero.
The recovery window checks acceleration, angular rate, magnetic strength/dip,
direction stability and sample continuity. Invalid data cannot force recovery.
BLE loss alone does not stop firmware acquisition. Complete turns during missing
inertial data cannot be reconstructed, but the output represents current direction
modulo 360 rather than a turn count. Reacquisition can produce an endpoint correction.
Calibration, tilt compensation,
magnetic checks and circular filtering still apply. Relative zero does not cure
magnetic interference or calibration error. Physical demo accuracy still needs
a level clockwise/counterclockwise rotation test after flashing.

Current tilt processing uses LSM6DSM gyro (104 Hz ODR, polled at 100 Hz,
2000 dps full scale, 0.07 dps/LSB), while the required LSM303AGR raw channels remain unchanged.
Gyro transport rotates both gravity and horizontal north. Acceleration provides
slow, gated tilt correction; suspect linear acceleration is not treated as gravity.
Magnetic yaw correction is limited to 0.5 deg/s and rejected for large field
strength, dip or innovation changes. Rejected magnetics disable magnetic aiding,
but **fresh gyro continues to transport and publish the changing angle**: there
is no five-second display cutoff or permanent disturbance latch. The estimate
can be gyro-only, NOT a certified magnetically corrected heading; `P2IMU.mag_aiding`
and `coast_ms` expose that distinction. Longer gyro-only intervals can drift.
Actual missing/invalid gyro still pauses output and triggers automatic recovery.
Range rejection is per axis above 2000 dps, not a 490 dps vector-norm threshold.
Initial magnetic contamination, same-strength direction-only interference after a
data gap, residual calibration errors and slow bias-learning motion are not fully
observable. This is not an interference-proof compass.

`heading-demo-7` removes the lifetime magnetic-correction budget and independently
integrated boot-direction rejection. Short-window confidence now uses innovation
against the running attitude, allowing magnetic correction of accumulated gyro
bias. Two-second circular means above 6 degrees suspend magnetic aiding; a clean
mean below 2.5 degrees clears that condition. Hard magnetic rejection also requires
a complete clean block before aiding resumes. These checks affect the correction,
not the fresh inertial angle. Large persistent innovations may continue to prevent
magnetic aiding; data availability does not imply bounded long-term angle error.
The core gyro/tilt transport and pedometer detector are unchanged.

These are conservative engineering thresholds, not measured accuracy limits.
The user-selected acceptance target is **120 continuous seconds after the first
valid startup zero, all heading samples valid and within +/-5 degrees while the
board stays still**, measured by shortest circular error (359 is -1 degree
from zero, not 359 degrees of drift). Version 9 changes presentation and startup
confirmation, not that acceptance target or post-startup version 7 fusion. It neither clamps
the display to +/-5 degrees nor resets the origin to hide drift. Actual turns
beyond 5 degrees must remain observable. Passing analytic fixtures is not a
hardware acceptance result.
Residual gyro bias and slowly varying magnetic fields can still change the estimate;
there is no guarantee of indefinite drift-free or interference-proof operation.
Hardware acceptance must report both valid-angle error AND valid-output fraction,
not count invalid samples as zero-error readings. Recheck slow turns and tilts.

Known boundary, explicitly NOT passed by `heading-tolerance-6`: a magnetic target
ramping by +/-5 degrees in 80 seconds while post-startup gyro bias is +/-0.04 deg/s.
Over 120 seconds, same-sign combinations stay valid but peak at about 5.198 deg;
opposite-sign combinations become invalid at 65.29 s (6528/12000 valid samples).
Those are historical results for version 6, not version 7 test results. The
combined magnetic-drift/bias fixture remains a separate `--boundary` characterization
with the same +/-5-degree and 100%-valid target. Do not infer robustness to changing
fields from a single successful stationary recording.

Version 9 software verification (2026-10-15): the default Windows regression suite
passes, including new asynchronous 104 Hz gyro / 50 Hz acceleration+magnetic cold
starts across five sample phases, quiet/noisy magnetic input and timestamp wrap.
Those ideal synthetic cold starts first publish exactly zero at 5021-5039 ms.
A slow 0.2 deg/s moving startup stays uninitialized for the whole 12-second test,
then settles after stopping; its following 120-second maximum drift is 0.2911
degrees. This is not a claim that arbitrarily slow motion can always be detected.
The 12 standard 120-second stationary fixtures remain fully valid with maximum
internal circular error 1.6521 degrees. Strict `--boundary` characterization still
fails four of six cases, maximum 5.1999 degrees with 100% valid output. No drift
target was relaxed. Software results do not establish physical initialization
time, physical tilt accuracy or marking-app compatibility. This revision has not
been flashed or tested cable-free in this update.

Version 7 software verification (2026-10-13): the default Windows regression suite
and all four recorded walking/handling replays pass. Twelve 120-second synthetic
stationary cases (gyro residual +/-0.025 or +/-0.04 deg/s, stable field or modest
direction variation/noise) remain 100% valid, with maximum displayed error 1.6521 deg.
The separate strict `--boundary` run intentionally remains a known failure:
four of six extreme cases exceed 5 degrees (maximum 5.1980 deg), although all
12000 samples per case remain valid. This is NOT a claim that every accuracy test
passes. The new regressions also cover signed full turns at 600/1000 deg/s,
magnetic-strength variation, per-axis gyro range, and automatic recovery without
losing the original zero or step count. Synthetic tests do not establish physical
turn/tilt accuracy.

Version 7 on-board check (2026-10-13): Debug build completed with 0 errors and
0 warnings; application write/readback verification passed at 0x08004000.
The existing bootloader was retained, and the previous full 1 MiB flash is in
`../environment_backup/before_heading_demo7_20261013_d93a6.bin`.
Firmware BIN SHA256:
`90CAD9375BE79D9352FF6EA60C31A91A90A90FA3E2242C46339AB769F831F4B7`.
UART capture began before reset/programming and is saved as
`../logs/heading_demo7_boot_20261013_d93a6.log`. Analyze only the new
`firmware=heading-demo-7` boot, not the preceding version 6 data. Its startup
reports gyro ID 0x6A, 104 Hz and 2000 dps. First valid zero is at 22301 ms.
The inclusive 22301..142301 ms window has 1201 records at exactly 100 ms intervals,
all heading-valid, with displayed range -2.4..+0.1 deg. Origin stays 11866
centidegrees; steps, sensor/BLE errors, UART drops and reference-loss flags stay
zero. Magnetic aiding remains active in every logged sample. Thus this recorded
window passes the chosen availability and +/-5-degree target. Sensor traces are
consistent with near rest; they cannot independently prove complete physical
immobility. The desktop viewer was connected at about 20 Hz with CSV recording
enabled. Full physical clockwise/counterclockwise and tilted-turn accuracy still
require the user's follow-up test; do not infer those results from this static run.

Native gyro XYZ is mapped to (-Y,-X,-Z) for the existing LSM303AGR register frame.
This uses the original ST MotionFX `wsu`/`swu` orientations and the determinant
sign for an axial vector under a reflected coordinate transform. **Physical axis
correlation and tilted-yaw accuracy are still acceptance tests**, not established
by synthetic geometry. See [ST's clarification of the inconsistent axis drawing](https://community.st.com/mems-sensors-48/coordinate-system-of-lsm303c-lsm303agr-lsm303ar-is-right-hand-54054).
Forward +X near vertical remains deliberately invalid (|up.x| > 0.95, about
72 degrees pitch). Calibration remains `DIAGONAL_3D_PENDING_VALIDATION`.

Pedometer processing now rejects repeated large direction reversals, discarding
unconfirmed peaks during handling. Release requires a quiet direction cone as
well as a minimum hold time. The magnitude baseline, step sensitivity, distance
model and existing published totals are retained. Exaggerated back-and-forth arm
motion while walking may be rejected too: carrying-mode accuracy needs retesting.

Development checks for this revision:

- Same user tilt trial, 20 Hz BLE snapshots: old algorithm 20 false steps,
  new algorithm 0. The old board actually reported 21 at full rate. The 10 Hz
  UART snapshots yield 0 in both versions and are NOT improvement evidence.
- Existing 50-step walking development recording still replays as 51; flip
  recording remains 0. Eighteen analytic pitch/roll rocking cases produce 0.
- Independent combined yaw/pitch/roll plus translation fixtures: maximum heading
  error 0.0051 deg in ideal synthetic data. This is NOT physical accuracy.
- Boot-only regressions cover a first magnetic outlier, convergence before zero,
  bias refinement, 60-second stationary simulation and real 0.1 deg/s turns.
  Sensor interruptions, duplicate timestamps and inertial movement between
  magnetic updates cannot satisfy the quiet window. Pedometer state remains
  identical sample-for-sample in the paired startup regression.
- Tests cover sensor I/O faults, bias startup, orthogonality, magnetic rejection,
  recovery, near-vertical invalidity, full-turn zero and tick wrap.
- The old real-world trial has no gyro samples; do not invent gyro values to
  claim this heading implementation has passed that recording. After flashing,
  verify level turns, +/-30-degree pitch/roll, near-vertical rejection, return to
  rest, no-step rocking and independent counted walking trials.

First on-board check (2026-10-13, `motion_fusion3_trial_20261013_1a947.log`):
Debug compiled with 0 errors/warnings and SWD write verification passed at
0x08004000; bootloader retained and full flash backed up. Gyro samples advance
at 100 Hz and desktop BLE at about 20.2 Hz. In the user's slower tilt trial,
acceleration-derived pitch reached about -38 to +51 degrees. Heading baseline
(device 45-52 s) was +4.88 deg; motion (53-75 s) ranged +2.7 to +8.8 deg,
maximum deviation 3.92 deg. Returned rest (85-110 s) averaged +2.477 deg,
an offset of -2.403 deg from baseline, with 2.4-2.6 deg range. Steps stayed 0;
no sensor/BLE errors, invalid headings or dropped UART records in that interval.
Gyro-propagated up and acceleration direction agreed within 3.9 deg while moving
(translation means acceleration is not an independent ground-truth attitude).
This supports pitch tracking, NOT full three-axis/yaw mapping certification.
The slower motion is not a controlled reproduction of the earlier fast rocking:
repeat fast rocking and independent counted walking before final acceptance.

UART adds `P2IMU` diagnostics alongside unchanged `P2DATA`: native gyro in mdps,
fresh/calibrated flags, sample counter, magnetic-aiding flag, inertial coast age,
handling flag and estimated unit-up vector scaled by 1000. The raw `.log` keeps
these records; the existing CSV logger intentionally keeps its original columns.
`P2HEAD` (introduced in `startup-zero-4`) reports `bx/by/bz` as learned BODY-frame gyro bias
in mdps; `rx/ry/rz` are residual rates in that frame. `ns/nb/stable_ms/span_cd`
describe the boot window (samples/blocks/milliseconds/block-mean span in
centidegrees). `ready/zero/status` describe state; `mag_cd/innovation_cd`,
`correction_cdps` and `origin_cd` expose magnetic target, innovation, applied yaw
correction rate and fixed zero. Correction is zero when not applied; target and
innovation must be interpreted with fresh/valid/aiding flags. Start UART recording
before reset to capture the full boot and learned reference, then keep still for
at least 120 seconds after zero. A changing external field or later thermal bias
can still change heading; software fixtures alone are not hardware acceptance.
`mag_budget_cd` retains its legacy key but in version 7 is only a wrapped diagnostic
total of magnetic correction, NOT a budget. `mag_delta_cd` is the last complete
current-innovation mean; `mag_blocked` describes magnetic aiding, not heading validity.
These angles are centidegrees. `reference_lost` now means automatic reacquisition
pending. `recovery_ms` and `recovery_samples` show that window's progress. Loss logs
`P2WARN,inertial_reference_lost,reset_required=0,recovery=keep_still,steps_preserved=1`;
successful reacquisition logs `P2INFO,heading_recovered,origin_preserved=1,steps_preserved=1`.
Tests cover tolerated small
magnetic changes, excessive drift, repeated block interruptions, clean recovery,
real +/-0.1 deg/s turns, combined tilt/yaw/translation, and independent steps.
The 20-byte BLE packet format is unchanged, so the signed-cycle desktop viewer
remains compatible. Historical sections below describe older revisions.

Previous startup-reference hardware check (2026-10-13; user subsequently confirmed
the board and nearby objects stayed untouched):
`logs/startup_zero4_boot_20261013_c9f84.log` in the parent project directory
was opened BEFORE backup/reset/flash. Analyze only its final
`firmware=startup-zero-4` boot segment; earlier segments are the prior firmware.
Debug built with 0 errors/warnings; application write at 0x08004000 verified.
First valid output was at 12.201 s, after 307 samples / 6030 ms with a 0.41-degree
block-mean span. Origin remained 86.96 degrees. Initial approximately 40 seconds
after zero ranged -0.2 to +0.9 degrees, but the full first minute reached +4.2
degrees. Device 60-70 s displayed +3.323 degrees on average while the magnetic
target averaged +4.363 degrees relative to zero; mean residual gyro yaw rate was
only about +0.0038 deg/s. By 80-90 s the output mean returned to +0.043 degrees.
Steps, sensor errors, BLE errors and dropped logs remained zero in those windows.
This did NOT pass stationary-heading acceptance: startup seeding was improved,
but a changing magnetic target still influenced running yaw correction. It motivated
the current consistency safeguard. The source of the fluctuating magnetic readings
has not been physically isolated; this is not proof that nearby objects moved.
The original complete 1 MiB flash image is preserved in the parent
`environment_backup/before_startup_zero4_20261013_c9f84.bin`.

Previous `mag-consistency-5` guard hardware check (2026-10-13): Debug built with 0 errors/warnings;
the 51,816-byte application verified at 0x08004000. Binary SHA256:
`8F52F31A2755198769EC9F8EE863642D4331DF651BF9A101010D9F46C7574241`.
Prior complete flash is in the parent
`environment_backup/before_mag_consistency5_20261013_b6719.bin` (1,048,576 bytes).
UART `logs/mag_consistency5_boot_20261013_b6719.log` began before reset/flash;
use only its final `firmware=mag-consistency-5` segment. First valid was at
8.201 s (307 samples / 6030 ms / 0.12-degree startup mean span). The first
60 seconds AFTER zero had 601/601 valid samples, ranging -0.3 to +0.3 degrees.
Through device 103.501 s, validity was 882/954 = 92.45%, with valid angles
-0.8 to +0.3 degrees. Disagreement latched around 91.4 s; invalid output began
at 96.401 s and was still ongoing at the analysis cutoff. `reference_lost`
remained 0, and steps/sensor/BLE/log-drop counters remained zero. Mean residual
gyro yaw was approximately -0.013 to -0.017 deg/s in device 50-100 s: the
independent inertial reference can itself slowly drift. Thus the safeguard is
working, but sustained stationary availability is NOT accepted yet. Do not
present the first-minute result alone as full success. Physical slow-turn and
tilt checks, plus continued residual-bias diagnosis, remain pending.

This implementation extends the supplied COMSYS704 starter. It acquires real
LSM303AGR acceleration/magnetic field, detects steps and estimates magnetic
heading with tilt compensation, and sends measurements through the existing
ST BLE Sensor application interface. Distance is an additional fixed-stride
estimate, not an independently measured position.

**Historical revision (Windows, 2026-10-05): `heading-fix-1`.** Debug clean build,
desktop regression tests, STLINK programming/verification, UART and Windows
BLE runtime checks passed. Heading now uses provisional measured XY bias,
an empirical horizontal-turn direction correction and circular smoothing.
Only rotation trials 2 and 3 were used; trial 1 was excluded. Persistent
magnetic-input drift is NOT resolved. Full 3D calibration, mounting-axis/tilt
validation and independent absolute-bearing accuracy remain outstanding.
See `VALIDATION.md` and `docs/heading-fix-20261005.md` for evidence and limits.
Historical Debug/Release, debugger and transport results below do not replace
validation of the current revision. Release was not rebuilt for this update.

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
  above 100 mg is a candidate, with a 280 ms refractory interval. Three candidates
  with adjacent intervals no greater than 1.5 s and an interval ratio no greater
  than 1.6 confirm a walking bout. The first three are then published together,
  so confirmation delays the display but retains the starting candidates.
  Subsequent candidates count immediately; a pause longer than 1.5 s restarts
  confirmation. One/two-step bouts may be discarded, and rhythmic shaking may
  still resemble walking. This is not a general activity classifier. Startup and
  sampling gaps over 250 ms trigger a one-second settling interval. Counts
  reset on reboot. This is a tunable baseline, not a universal gait classifier.
- Heading: low-pass acceleration estimates up. Subtract magnetic hard-iron bias,
  apply diagonal scale, project magnetic north into the plane normal to up,
  then use `atan2` to find the heading of body +X. A configurable sign is
  applied before the heading offset; `-1` matches the user's labelled clockwise
  and counterclockwise horizontal turns. A 0.15 s sin/cos low-pass handles the
  359/0 degree boundary without averaging through 180 degrees. Invalid samples
  immediately invalidate heading; reacquisition resets the filter.
  Output is `[0,360)` degrees with clockwise increase intended. This is a
  **magnetic**, not true-north, estimate; absolute north and the full 3D axis
  convention are not independently verified. No local declination is guessed.
  Raw sensor register axes and the original tilt projection are retained.
  Intended mounting is **+X forward and +Z up**; verify it experimentally.
  Current bias is `(12.637410, -161.102725, 0)` mGauss with unit scales,
  labelled `XY_PROVISIONAL`, not a full 3D calibration.
- Reject weak/strong magnetic field, invalid gravity, nearly vertical +X and
  stale data. An invalid heading is sent as -1 degree. Magnitude checks cannot
  detect all magnetic disturbances, particularly uniform field distortions.
- Distance = total steps × configured stride (default 0.70 m). Measure a known
  distance and divide by actual steps to set `P2_STEP_LENGTH_M`.

## Phone and serial data

For the local Windows BLE viewer, install Python with Tkinter and `bleak`,
then run `python tools/ble_monitor.py` from this source root. Enable Windows
Bluetooth and disconnect the iPad/phone first. The viewer connects to `P202S01`
by default; use `--no-auto` to edit the target name/address before connecting.
It shows live raw XYZ, steps, heading, distance and notification rate, marks
stale/invalid data, and retries a lost connection. Check **Save CSV** to record
under `../logs/ble_monitor/`; uncheck it or close normally to finish the file.
The viewer is local-only and does not flash firmware. BLE logs contain about
20 Hz snapshots, not the complete approximately 50 Hz sensor stream.

Advertising and GAP device names are now both `P202S01` (Project 2, Group 02,
step-confirm revision 01), configured by `P2_BLE_NAME`. UART boot output includes
`firmware=heading-fix-1,magnetic_calibration=XY_PROVISIONAL`. The unchanged
`P202S01` advertising name is not a revision identifier. The app's
`FP-SNS-ALLMEMS1 v4.2.0` label describes the
inherited ST profile, not this project's firmware revision. After flashing,
disconnect and scan again; a cached name alone is not proof of firmware version.
This board's address is
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
| 10–11 | Startup-relative heading in Gyroscope Y | whole degrees 0..359 × 10; reserved invalid codes above |
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

The current XY-only fit is provisional for the recorded mounting/environment.
It cannot determine Z bias or general soft-iron distortion. Keep
`P2_MAG_CALIBRATED=0` until a measured 3D calibration is performed. A laptop
case or nearby magnet may invalidate heading; do not relax the field-validity
limits merely to make the display show a number. The limits also cannot detect
every plausible-magnitude field distortion. Circular smoothing reduces fast
jitter, not sustained external-field changes, and introduces response lag.

1. Keep `P2_LOG_PERIOD_MS` at its default `100U`, and capture UART output while
   rotating the **assembled wearable** slowly through all 3D orientations for
   at least 30 seconds, away from metal, magnets and power equipment.
2. Save the capture as `rotation.log`, then run:

   ```sh
   python tools/calibrate_mag.py ../logs/magnetometer_calibration_YYYYMMDD_HHMMSS.csv
   ```

3. The tool prints candidate `P2_MAG_BIAS_*` and `P2_MAG_SCALE_*` definitions.
   Copy them into `Inc/p2_config.h`, rebuild and flash for validation. Keep
   `P2_MAG_CALIBRATED=0` until independent direction/tilt tests pass; then enable
   the calibrated strength guard. Parameters persist in firmware, not runtime flash writes.
4. Validate at independent known directions and ±30° tilt. Repeat collection
   if coverage is inadequate. The least-squares diagonal fit cannot correct arbitrary
   cross-axis soft-iron distortion. Recalibrate after changing mounting hardware.
5. Test a measured 100-step walk at slow/normal/fast speed and fixed mounting.
   Tune high/low thresholds only against recorded trials, then validate with
   separate trials. Measure stride for distance. Do not tune on the final test.

## Flash layout and first hardware run

The October 12 course flash-location fix is applied: the application and vector
table both start at **0x08000000**, with `VECT_TAB_OFFSET=0x00`. The linker
keeps the instructor's **496K** Flash limit. No separate bootloader is required
for this layout. Earlier hardware notes in this README refer to the old
0x08004000 application layout and do not describe the current build.

Clean-build Debug or Release after this change. In CubeProgrammer, select the
fresh **ELF** (it carries its addresses), download with verification, then reset.
For a freshly rebuilt **BIN**, explicitly use **0x08000000**. Do not relocate an
old 0x08004000-linked BIN: it must be rebuilt. Programming this standalone image
replaces the former bootloader region; do not install BootLoaderL4.bin afterward.
Preserve option bytes. Check startup after removing the debugger and power cycling.

The two legacy `CleanALLMEMS1_STM32CubeIDE_ST` scripts are disabled with an early
exit because they mass-erase, install a bootloader and program the old offset.
Use the freshly built ELF and CubeProgrammer instead. No hardware is flashed
by the build helpers.

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


### Heading guard and English dashboard (heading-guard-2)

The dashboard now uses English labels, separate magnetic-heading/step/distance/rate
cards, a compass, XYZ values and sensor freshness indicators. The compass describes
board +X relative to magnetic north, not true north or pedestrian travel direction.

Firmware `heading-guard-3` establishes its field-strength reference from a continuous
one-second window whose magnitude range stays within 10% of the current magnitude.
This follows the existing acceleration settling period. Gaps over 200 ms or invalid
samples restart reference confirmation. Repeated calls at one timestamp do not add
samples. A strength change over 35% of the reference rejects the heading; recovery
requires 500 ms of consecutive acceptable observations. The reference never adapts
to sustained interference. An acceleration reset clears it.

The 20-byte packet layout is unchanged, but negative heading values now distinguish
-1 unavailable/stale sensor, -2 establishing reference, -3 magnetic disturbance,
and -4 confirming recovery. They are encoded using the existing x10 wire scale.
Use the updated English viewer; older clients must treat ALL negative headings as
invalid, and the ST app may display the sentinel as a negative gyro value. CSV
heading_valid is false for every negative status. UART retains its generic -1 invalid
heading for compatibility. Calibrating means reference acquisition, not full 3D calibration.

This is a configurable heuristic, not trained classification or gyro fusion.
Startup must be away from interference. Direction-only disturbances and polluted
startup are not detected. The thresholds require hardware validation and provisional
XY calibration may cause magnitude changes during tilt.

Before claiming improved accuracy: record a clean stationary baseline, test known
cardinal directions and slow turns, introduce/remove a magnetic disturbance, and
check recovery. Repeat with the intended mounting. Complete independent 3D calibration
before assessing tilted heading accuracy. Step detection and fixed 0.70 m stride are
unchanged. No new hardware accuracy percentage is claimed by software tests.


### Per-step distance and personal calibration (stride-ready-1)

Distance is accumulated when steps are confirmed, including startup backfill.
Unconfirmed bouts are discarded. Each confirmed step retains its historical distance;
a later cadence change never rescales earlier distance. The first step in a confirmed
bout uses the first observed interval. An inconsistent candidate sequence starts a
new pending pair, with its distance recomputed from that pair's interval.

Default behaviour remains 0.70 m per step: no personal distance measurements exist
in this update. Optional calibrated mode uses `length = intercept + slope * cadence`
(cadence in steps/second). Outside the calibrated cadence range it falls back to the
personal fixed length. Model parameters are checked for finite, plausible values.
`P2_MotionSetStride` rejects mid-bout updates; config macros are applied at startup.
Invalid compile-time models fall back to the safe uncalibrated 0.70 m default.

Create a CSV with `distance_m,steps,duration_s,split`. Use measured ground-truth
route length, manually counted steps, and walking-only duration. Keep the same person
and pocket placement. Collect at least six training and three separate validation
walks covering slow, normal and fast speeds; never duplicate a walk across splits.
Run `python tools/calibrate_stride.py measurements.csv`. It prints candidate config
macros and validation errors versus 0.70 m and a personal fixed step length. It does
not edit firmware. Enable adaptive mode only after improvement is reproduced on
additional independent walks. If even the personal fixed model loses to 0.70 m,
retain the baseline and collect better data rather than applying its suggested value.

The calibration tool uses trial-average cadence and manual counts. Its errors do
not include firmware miscounts or within-walk speed changes. End-to-end firmware
validation is still required. The viewer labels distance as a firmware estimate;
the current packet does not identify the active model. Existing packet range limits
still apply. This model is empirical, not the Weinberg or Ho paper implementation.


### Provisional-calibration heading availability (heading-availability-1)

The reference-strength rejection/startup/recovery windows now run ONLY when
P2_MAG_CALIBRATED is 1. The shipping configuration is 0 because only provisional
XY correction is available. Rotating a partially calibrated magnetometer can change
its corrected strength enough to trigger the guard, so that guard must not block
heading in this configuration. Finite-value, absolute magnetic/gravity bounds,
horizontal projection, forward-axis and stale-data checks remain active.
A displayed heading is still uncalibrated magnetic heading, not verified true north.
Do not set the calibration flag merely to enable the guard. This fixes an identified
availability risk; the user's particular missing-heading cause needs hardware retest.


### Carrying-amplitude robustness (adaptive-gait-1)

The step detector still uses acceleration magnitude and does not classify hand-held,
arm-swing or pocket modes. A decaying amplitude envelope now adjusts the positive
peak threshold between 40 and 100 mg and trough threshold between -20 and -40 mg.
A candidate also requires at least 90 mg peak-to-trough prominence. These bounds
prevent the detector from chasing tiny stationary noise. Strong-signal startup still
needs three consistent candidates. Any weak startup candidate raises confirmation
to five and tightens adjacent interval agreement to a factor of 1.25. Confirmed
candidates and their distances are backfilled together. Fewer than five weak steps
may be deliberately missed. Abrupt rhythm changes during walking trigger fresh
confirmation without counting the already-published previous step again.

Synthetic tests cover 110/180/500 mg amplitudes, 50/75/120 steps per minute, continuously
rotating sensor axes, low noise, and amplitude/speed transitions. The existing
user-confirmed flip-only 10 Hz recording still replays to zero steps. Neither test
establishes real hand/pocket accuracy: the replay is downsampled, and rhythmic hand
shaking can resemble gait. Arm-swing signals may have a different period from foot
steps; do not apply an unvalidated x2 correction. No running, stair or pathological
gait accuracy claim is made.

Hardware acceptance: for each of steady hand-held, natural arm swing and trouser
pocket, record three 100-step walks at slow/normal/fast speeds. Keep separate tests
for stationary handling, transfers between hand and pocket, short 1–5 step bouts,
and stop/restart. Record manual counts and compare deltas, not lifetime counts.
Validate distance separately for each placement/person before enabling stride fitting.

### Magnetic calibration fitting (2026-10-12)

`tools/calibrate_mag.py` now fits an axis-aligned ellipsoid to the complete
rotation dataset instead of estimating each bias from two extrema. Normalized
coordinates and spatial-cell weighting reduce numerical imbalance and the effect
of long stationary holds. The tool requires all eight octants, both ends of each
axis, and at least 60 occupied spatial cells; it rejects degenerate fits, large
residuals and implausible radii. These are engineering acceptance checks, not
proof of absolute heading accuracy. Cross-axis soft-iron distortion needs a more
general model than the current firmware's three scales.

Use fresh UART or BLE CSV from the assembled board, slowly covering all six faces
and intermediate orientations. A half-turn on a tabletop is insufficient for
3D calibration. Validate on a separate recording and known physical directions;
never select an offset merely to force two measured endpoints to match.

The October 12 half-turn recording changed from about 175.5 to 262.3 degrees
while the user reported a physical 180-degree turn. No invalid headings or extra
steps occurred in the analyzed 173-second snapshot. The new fitter correctly
rejects this recording for insufficient 3D coverage. Existing firmware calibration
constants are unchanged until suitable measurements are available.

### October 12 measured calibration candidate (`gait-magfit-1`)

The current config uses a diagonal 3D fit from 5342 BLE snapshots collected
14:46:10–14:50:37 NZDT on October 12. Bias in mGauss: (-298.170662,
74.582182, 258.322494); scale: (1.000105, 1.046709, 0.957186). The
corrected magnitude coefficient of variation is 3.16%. All eight magnetic
octants and both ends of each axis passed the fitter's coverage checks.

The preceding half-turn recording was excluded from fitting. Replaying it
through the C heading implementation with these parameters gives stable
endpoints of about 129.4 and 311.0 degrees: a 181.6-degree positive change,
compared with the recorded 86.8-degree change under the earlier calibration.
This supports improved relative rotation for this trial, not verified north.
The user labelled the endpoints south/north, so an approximately 50-degree
absolute discrepancy remains. No heading offset was introduced. Check board
+X alignment and independently referenced directions before enabling
`P2_MAG_CALIBRATED`. The fit cannot correct arbitrary cross-axis distortion.

These are offline results from 20 Hz BLE snapshots. Reconstructed startup
invalidates the first second of replay; it is not a live dropout. Rebuild and
flash, then validate level cardinal headings and tilted poses independently.
The live viewer and connected board keep using their existing firmware until
reflashed. The earlier XY constants remain documented above as history.

### Approximate user-reference alignment (`gait-aligned-1`)

At the user's request, treat the earlier stationary holds as approximately level
and facing south then north. With the new diagonal calibration, the unaligned C
replay means were 129.368727 and 310.956849 degrees. Equal-weight circular averaging
of the two reference corrections gives `P2_HEADING_OFFSET_DEG=49.837212`.
This is a constant zero adjustment: it preserves the measured rotation and does
not force a 181.6-degree turn to equal 180. The magnetometer bias/scale stay based
on the measured 3D fit; raw acceleration and magnetic readings are unchanged.

The reference recording is now alignment training data, not an independent
absolute-accuracy test. Approximate handling and the user's south/north reference
limit its accuracy. This offset is not a measured declination and does not prove
true north. The board +X axis remains the heading axis. Near-vertical +X remains
undefined and is rejected; no artificial direction is substituted. Independent
level east/west and tilted tests are still required. The viewer therefore labels
the value Board heading / Firmware reference instead of promising magnetic north.

### Hand-held walking continuity (`gait-continuity-1`)

The October 12 manual 50-step hand-held trial reported 36 steps on the board.
Its 20 Hz BLE snapshots replayed as 21 steps with `gait-aligned-1`. Requiring
adjacent intervals to remain similar after walking was already confirmed caused
repeated bout resets and discarded candidates. Cadence consistency now applies
to startup only. Established walking still requires each trough/peak, prominence
and refractory check, and pauses longer than 1500 ms or invalid/gapped sensor
input still reset confirmation. No peaks or missing steps are invented.

The same snapshot replay produces 51 with the continuity fix. This is development
evidence on the motivating recording, not an independent test or a prediction
of the full-rate hardware count. More permissive continuity may admit handling
that begins immediately after walking; static/flip regressions cannot rule out
every such case. Re-test hand-held walking and stop-then-handle behavior on-device.
A synthetic changing-cadence regression failed at 10/50 before this fix and
checks continuity plus pause/restart protection after it.
