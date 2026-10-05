# Heading correction and validation, 5 October 2026

Firmware: `heading-fix-1`. Scope: reduce short-term jitter, correct the observed
horizontal-turn direction and remove the XY bias responsible for a weak
horizontal magnetic component at some angles. **Persistent magnetic-input
drift is not resolved.** No claim of complete 3D calibration or absolute
heading accuracy is made. No GitHub operation was performed.

## Source observations and data boundary

Only these original recordings under `../../logs/ble_monitor/` were used:

- Trial 2: `ble_20261005_130753_309873.csv`. The user states that the board was
  stationary at the beginning. Do not relabel that opening as intentional yaw.
- Trial 3: `ble_20261005_131056_903890.csv`. User-labelled clockwise turn then
  counterclockwise turn, with stationary intervals.

Trial 1 was explicitly excluded from fitting, comparison and conclusions.
Original recordings are unchanged. Fit and C-replay outputs are saved under
`../../build_logs/heading_fix_20261005/`.

Trial 2's raw magnetic field changes from about `(1.67, -374.50, 228.83)`
mGauss in the first 0.5 s to `(-70.73, -354.45, 232.45)` in 2.5-3 s.
This is input variation, not an angle-wrap artefact. With no yaw-rate input,
the current algorithm cannot distinguish a changing external magnetic field
from level yaw using acceleration alone. Holding the last heading whenever
acceleration appears still would also suppress real level turns, so no such
lock was added. The user's computer-case invalid-heading observation is
consistent with interference, but the exact rejection gate for that event
was not captured and cannot be identified retrospectively.

## Implemented design

1. **Provisional XY bias.** Fit `(x-bx)^2 + (y-by)^2 = r^2` to 663 fresh samples
   from trial 2's first 33 s. Results: `bx=12.637410`, `by=-161.102725`,
   `r=216.770040` mGauss. Radial population SD is 6.638993 mGauss; angular
   coverage is 301.766978 degrees and maximum measured tilt 9.7795 degrees.
   Trial 3 independently gives approximately `(13.12, -162.78)` for CW and
   `(16.79, -162.28)` for CCW fits, supporting repeatability in this setup.
   Z bias stays zero and scales stay one. Horizontal rotations cannot identify
   Z bias or full soft-iron coupling. `P2_MAG_CALIBRATED=0` remains explicit;
   the boot label is `XY_PROVISIONAL`. A changed environment/mount may invalidate
   this fit.
2. **Direction convention.** Keep raw XYZ and the existing gravity projection.
   Apply `P2_HEADING_SIGN=-1` before `P2_HEADING_OFFSET_DEG`. This matches the
   labelled horizontal rotation direction, not a proven 3D sensor-axis remap.
   True north, installation offset and tilted direction require independent
   physical reference tests. Do not copy starter MotionFX axis strings from
   a different accelerometer into this LSM303AGR path without verification.
3. **Circular low-pass.** Filter sine and cosine of the measured heading with
   `alpha=dt/(0.15+dt)` and reconstruct with `atan2`. This handles wrap at
   0/360 degrees and reduces fast jitter. The 0.15 s time constant introduces
   response lag. Invalid data immediately clears validity; reacquisition or a
   long gap seeds the filter from the new observation. A near-zero filtered
   vector is also reseeded. The filter does not integrate gyro rate and does
   not remove persistent input-field changes.

Weak/strong-field, horizontal-component, gravity, forward-axis and freshness
checks are retained. There is no relaxation to accommodate the computer case,
no stale-heading latch, and no fabricated valid bearing. A plausible field
magnitude alone does not guarantee an undistorted direction.

Steps, stride, sensor ODR, raw physical-unit XYZ, BLE packet layout and device
name `P202S01` are unchanged. The BLE heading retains its existing whole-degree
rounding before encoding; UART gives 0.1-degree diagnostic values.

## Offline evidence: real C replay, not an exact sensor-stream reproduction

`tools/replay_heading.py` compiles the actual motion C implementation using
`tests/heading_replay_runner.c`. Old sources were saved before editing in
`../../build_logs/heading_fix_20261005/original/`. BLE data are approximately
20 Hz snapshots, while the board processes fresh sensor samples at about
50 Hz. Replay resets per session and has an initial one-second settling period.
Its results must not be presented as measured board accuracy.

| Trial 3 interval, local host time | Original C replay | Corrected C replay |
|---|---:|---:|
| CW 13:11:13-13:11:37, net unwrapped change | -359.604 degrees | +359.041 degrees |
| CW interval invalid snapshots | 11 | 0 |
| CCW 13:11:50-13:12:10, net unwrapped change | +358.138 degrees | -359.013 degrees |
| CCW interval invalid snapshots | 5 | 0 |
| After settling, all trial 3 invalid snapshots | 16 | 0 |
| Quiet 13:10:58-13:11:13, population SD | 0.544 degrees | 0.289 degrees |
| Quiet 13:11:39-13:11:50, population SD | 0.483 degrees | 0.295 degrees |
| Quiet 13:12:20-13:12:30, population SD | 0.548 degrees | 0.306 degrees |

Corrected quiet means are 23.770, 22.481 and 24.497 degrees respectively.
Final-minus-initial mean is +0.727 degrees. This closure is not an absolute
heading-error result, nor evidence that intermediate 90-degree angles are exact.
The minimum horizontal component increases from about 41.47 to 193.84 mGauss
in the trial 3 replay, eliminating its weak-horizontal region after settling.

**Counterexample retained:** trial 2's first 1-8 s has original range 12.360
degrees / population SD 2.197 degrees, versus corrected range 17.640 degrees /
SD 3.682 degrees. The 35-90 s interval also increases from range 4.676 / SD
0.724 degrees to range 6.239 / SD 1.101 degrees. Bias correction removes angular
compression but can expose or amplify the angle effect of field changes.
It would be incorrect to describe this revision as eliminating all drift.

## Build, backup and on-device checks

- Windows desktop suite passes: motion, mock sensor, packet, calibration,
  seven logging tests, walking-bout tests and new heading tests. The nine
  synthetic 100-cycle walking cases still count 100 each. Synthetic tests do
  not establish real walking accuracy. No Windows sanitizer run is claimed.
- CubeIDE 2.2.0 / bundled GCC 14.3.1 Debug clean build: zero errors/warnings,
  text 38888, data 112, bss 18840 bytes. Log:
  `../../STM32CubeIDE_Workspace_HeadingFix/build-Debug.log`.
  Release was not rebuilt in this update.
- Full 1 MB old flash backup:
  `../../environment_backup/before_heading_fix_20261005.bin`, SHA256
  `0D502F8985A8266DA4AF782F786D41FBDF384A940DCF0237F3BF8D8988560E19`.
- STLINK-V3MODS serial `0026004D3233510639363634`, STM32L476, 3.26 V.
  Under reset / hardware reset / SWD 1000 kHz was required. Wrote BIN at
  `0x08004000`, verified successfully, then reset. Only sectors 8-27 were
  erased. Bootloader and option bytes were preserved; no mass erase.
- Current Debug ELF SHA256:
  `ADF407AB0CE3F74CE369A21989A44DE30CCA1A2FDEB47B06B95BCCFDE4026F97`.
  BIN SHA256:
  `5660150F742A5F1A17B605D7DF87B02EC6DB9FD1B94DC7D03843509D200CBA0B`.
- `../../build_logs/heading_fix_20261005/flash.log` records verification.
  `flash_uart.log` contains the old/new reset transition, the new boot version,
  sensor identity and BLE startup. Partial lines at the reset boundary are
  retained as captured. This file is not a wholly new-firmware steady-state run.
- Fresh post-flash capture:
  `../../logs/heading_fix1_bench_20261005_133244.log` and matching `.csv`.
  300 complete records, device ms 209200-239100 (29.9 s), all valid/fresh,
  no malformed complete rows, zero sensor/BLE errors and log drops, zero steps.
  Acceleration/magnetic rates inferred from sample counters: 51.24/50.87 Hz.
  Every row indicates BLE connection and subscription. UI check shows about
  20 Hz and live heading near 145 degrees, zero malformed packets/display drops.

The post-flash heading range is 140.3-149.4 degrees, population SD 1.9334
degrees, mean 145.904 degrees. First/last five-second means are
143.268/146.292 degrees. Physical stillness and whether the board had been
moved away from the computer case are awaiting user confirmation. These data
establish runtime/transport health, not a successful stationary-drift test.
The variation is not monotonic: the first/last five individual samples average
144.14/143.54 degrees. Five-second middle intervals have SD 0.319-0.468 degrees,
while the final interval has SD 2.512 degrees. Tilt is near level (mean 0.420,
maximum 0.954 degrees). Correlation between the provisional-bias-corrected
instantaneous XY magnetic angle and reported heading is 0.912. These observations
are consistent with heading following magnetic-input changes; they neither
prove physical stillness nor separate level yaw from environmental interference.

## Reproduction and required physical acceptance

From the source root, using the existing Python environment and a host compiler:

```powershell
..\.p2-validation\Scripts\python.exe tools\fit_heading_xy.py ..\logs\ble_monitor\ble_20261005_130753_309873.csv --seconds 33
..\.p2-validation\Scripts\python.exe tools\replay_heading.py ..\logs\ble_monitor\ble_20261005_131056_903890.csv --cc ..\.p2-validation\Lib\site-packages\ziglang\zig.exe
.\tests\run_tests_windows.ps1 -Compiler ..\.p2-validation\Lib\site-packages\ziglang\zig.exe -Python ..\.p2-validation\Scripts\python.exe
```

The fitting tool prints results only unless an explicit new `--output` file is
provided. It does not modify firmware or write the board. Replay can select
saved original `--source` and `--include` paths to reproduce the baseline.

Next, on wood/cardboard away from computer/phone/metal, explicitly confirm a
30-60 s untouched stationary recording. Then keep the same position and height
while rotating level CW and CCW with marked start/90/180/270/360-degree stops.
Changing location as well as yaw confounds magnetic disturbance with rotation.
Finally perform full 3D assembled-device calibration and independent known
bearing/tilt tests. Do not claim final accuracy from the two horizontal logs.
