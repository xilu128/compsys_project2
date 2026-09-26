# Validation record — latest logging update 2026-09-26

## UART-to-PC logging — PASS

Scope: logging infrastructure only. No step/heading/distance algorithm changes,
no BLE mapping changes, no sensor ODR changes, no SD or internal-flash logging.
The older 23 September results below are historical, not the current ELF hashes.
The old source ZIP was not silently replaced; use the current source for logging.

### Build and host tests

| Test | Result |
|---|---|
| Debug clean build, 10:02:40 | PASS: 0 errors / 0 warnings; text 34144, data 112, bss 18816 bytes |
| Release clean build, 10:04:19 | PASS: 0 errors / 0 warnings; text 30272, data 112, bss 18808 bytes |
| Existing motion/sensor/packet/calibration tests | PASS; synthetic results unchanged |
| Seven logging regression tests | PASS; parser/CSV fields, invalid heading, malformed values, byte fragmentation/oversize recovery, COM selection, collision-safe files, calibration formats, wrap/reset summary, mocked Ctrl+C/disconnect and file closure |
| Actual disconnect by physically unplugging | NOT TESTED; exception/closure path tested with mock serial |
| Actual Ctrl+C during hardware experiment | NOT TESTED; signal handler and complete-row preservation tested with mock serial; hardware timed shutdown tested |
| UART queue overflow / forced UART fault injection | NOT TESTED; normal dual-output capture reports zero drops |

Fresh current ELF SHA256:

- Debug: BF482DACCD76C03DB79606151C9B9B5DF0ACDE508F0796897758A4F76A6F7946
- Release: 7D58EE9F05B31800F7B1B824197E72C7E374CC73790E552D30B204C9F68A793B

Build outputs are in the existing STM32CubeIDE Debug/Release directories.
Logs: ../STM32CubeIDE_Workspace_Build/build-Debug.log and build-Release.log.
An initial clean on a source-only tree reported missing makefile; build helper
now bootstraps makefile generation before clean and rejects any clean failure.
Both final clean builds passed. No compiler/toolchain installation was changed.

Protected module SHA256 values before/after are identical:

- p2_motion.c: DFB62506F965F97C05B70BCF1797208C33EEBB9855521FDB72FE7450AA58A198
- p2_sensor.c: 0641749D7710945CB166A9D37AAE8C985049FB1FA5DD0A810E262F012B41E867
- p2_packet.h: 7E0B89998F4CF401E81C9C9828D1227C3A073200FFC8BD9E562F734EC5E106B1

### Actual board capture

Full flash saved before programming: ../environment_backup/before_logging_20260926.bin,
1 MB, SHA256 8938B1B458ABCEA207667973899731B63E8126A91FF5010FEB2ADFDD4140CECB.
STLINK-V3MODS, target ID 0x415. Debug ELF programmed/verified at **0x08004000**,
application pages 8–24 only; bootloader untouched, no mass erase. Reset/run PASS.

Final command: python tools/capture_log.py --name logging_final --seconds 30.
Auto-selection picked ST-LINK COM9 from seven listed COM ports. Python test
environment had pyserial 3.5; BLE regression used bleak 3.0.2.
Default PATH python is E:/Anaconda/python.exe and lacks pyserial. The missing-
dependency path was actually exercised: clear install instruction, no traceback
or automatic installation. Use the existing ../.p2-validation/Scripts/python.exe
or explicitly install pyserial into the default interpreter as README describes.

Evidence is outside source:

- ../logs/logging_final_20260926_100534.log (raw bytes)
- ../logs/logging_final_20260926_100534.csv (300 structured rows)
- ../logs/ble_logging_final_20260926.json (two BLE cycles)

Results: 300 rows over 29.9 seconds of device timestamps, all adjacent increments
exactly 100 ms (10 Hz); no malformed rows, no unfinished final line. Both sensor
axes change; sample counters give acceleration 51.24 Hz / magnetic 50.77 Hz.
Freshness flags remain 1, sensor/BLE errors and log_dropped remain 0.
Steps=0 and distance=0 throughout this bench capture; no claim of walking
accuracy. Heading varies 224.9–280.7 degrees with heading_valid=1; calibration
has not been changed or independently validated.

191 CSV rows show BLE subscription. Concurrent BLE connection/subscription/
disconnect/reconnect PASS: 201 packets in each cycle at 19.94/19.97 Hz, no
malformed packets. Therefore retain **100 ms / 10 Hz** rather than fall back
to 200 ms. These are short-run rate/continuity tests, not long-term reliability
or a complete 50 Hz waveform recording.

An earlier 30-second test produced 299 rows plus a partial final line retained
raw. Added a bounded line-completion grace period and repeated the test above:
final 300 rows, zero partial bytes. No data was fabricated to complete a row.

### Changed files in this logging task

APP = Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/.

- APP/Inc/p2_config.h: 100 ms logging, queue capacity and error-log interval.
- APP/Inc/ALLMEMS1_config.h: 512-byte bounded formatting, reject truncation.
- APP/Src/main.c: one-line P2DATA, P2INFO/P2ERROR labels, full internal distance
  rendering, sample/freshness/drop counters, interrupt-driven bounded UART queue.
- APP/Src/sensor_service.c: log labels and rate-limit notification error messages
  only; packet construction/scales unchanged.
- APP/Src/hci_tl_interface.c: startup log label only.
- tools/capture_log.py (new): serial selection, raw/CSV capture, parser and cleanup.
- tools/summarize_log.py (new): duration/steps/distance/validity/error summary.
- tools/calibrate_mag.py: CSV/P2DATA input plus preserved legacy log support.
- tools/check_hardware.py: recognize P2DATA in serial validation.
- tests/test_logging.py (new), tests/run_tests_windows.ps1, tests/run_tests.sh:
  logging regressions and integration into normal host test runners.
- tools/build_windows.ps1: safe first-build bootstrap and clean-failure check.
- tools/package_submission.py, .gitignore: exclude experiment logs.
- README.md and VALIDATION.md: operation, units, limitations and real evidence.

## Historical baseline — 2026-09-23, Windows

This supersedes the earlier Mac-only/no-hardware record. PASS is scoped to
the stated test; synthetic motion and PC BLE do not establish human accuracy
or course-phone compatibility. No GitHub operations were used.

## Requirements and milestone boundary

Authority: local CS704_Project2_Brief_2026.pdf, page 1.

| Classification | Requirement | Status |
|---|---|---|
| Required, interim 6 October | Real accelerometer/magnetometer over BLE; interpret units | Sensor/PC BLE PASS; course phone NOT TESTED |
| Required, final 20 October | Human steps/orientation and measured accuracy | Implementation/synthetic tests PASS; physical accuracy NOT TESTED |
| Required, code 20 October | Modular documented source, CubeIDE, top-level build README, group ZIP | PASS; source package provided |
| Optional | Fixed-stride distance | Mathematics PASS; physical stride NOT TESTED |
| Future/separate | Literature review 6 October; group report 19 October | Not produced by this firmware task |
| Not required | GPS/INS/EKF/ML, new BLE stack, RTOS, cloud, new bootloader | Not added |

## Build and structure — PASS

Existing project COMSYS704 imported headlessly; no new STM32 project created.
All 56 non-virtual .project linked paths resolve, including Drivers/Middlewares.
Original directory layout is preserved. CubeIDE 2.2.0 was automatically found;
scripts do not hardcode its path. Windows GNU Tools for STM32 14.3.rel1 /
GCC 14.3.1 and bundled make 4.4.1.

| Fresh clean build | Errors/warnings | text | data | bss | BIN bytes |
|---|---|---:|---:|---:|---:|
| Debug, 11:23 | 0 / 0 | 31,816 | 112 | 14,680 | 31,936 |
| Release, 11:24 | 0 / 0 | 28,000 | 112 | 14,672 | 28,120 |

ELF SHA256:

- Debug: A55297746AD76A7A19968D7E6EAE689C1DFBFC1E0B9BE64EF34E3CE9FCC9EA81
- Release: 4188F929E9355DDED2C8B2A4BB7D9AFBAD3B7A0DC7DA66F88B7E6925BCBA8E6C

Build logs: ../STM32CubeIDE_Workspace_Build/build-Debug.log and build-Release.log.
Fresh ELF/BIN and map/list/make outputs are archived under
../build_artifacts/20260923/Debug/ and Release/ to keep generated files outside
source. The next build regenerates these directories normally.

Older-starter caveats: stale refresh-scope names were corrected; selected launch
no longer hardcodes a machine log path. Historical launch files remain references,
not recommended entrypoints. Existing modern-linker READONLY sections and removed
stale Release MotionPM reference are retained; GCC 7 is not validated.
RG/NUCLEO-named metadata/linker limits are inherited, not proof of a different
physical board. Do not regenerate this imported project with CubeMX.

## Desktop tests — PASS

Windows Zig 0.16.0 (zig cc), C99, -Wall -Wextra -Werror -pedantic; Python 3.
No Windows sanitizer run is claimed. Evidence: docs/evidence/desktop_tests.txt.

- 60 seconds synthetic stationary noise: zero steps.
- 20-second 1/2/3 Hz synthetic gait: 19/39/59 counts for 20/40/60 cycles.
  Initial incomplete trough/peak is not counted; this is not measured accuracy.
- Adversarial 6 Hz motion: every step respects 280 ms refractory time.
- uint32 timestamp wrap, distance, cardinal and analytic tilted headings PASS.
- Invalid acceleration cannot revive stale gravity; settling after gaps,
  nonfinite gravity/field, weak/strong field, vertical forward axis, invalid
  calibration scale and expired trough are rejected.
- Mock WHO/configuration, data-ready, signed conversions and bus failures PASS.
- C saturation/invalid-heading encoding and Python 20-byte packet decoding PASS.
- Synthetic calibration recovery, inadequate coverage, too few points and
  nonfinite values PASS. These are not board calibration values.

## Hardware and boot — PASS with connection caveat

STLINK-V3MODS serial 0026004D3233510639363634, firmware V3J16M9B5S1;
target ID 0x415, revision 4, Cortex-M4 STM32L476 family, 1 MB flash,
observed 3.24–3.30 V. CubeProgrammer 2.23.0.

Initial reset vector at 0x08000000 was blank. Before repair, full 1 MB backup:
../environment_backup/before_baseline_20260923.bin
SHA256 E33436DD33362ED707C06D45BF10BB9175EF52EA5AD19B0D85391F9B66F1898B.

Restored supplied 16,152-byte Utilities/BootLoader/STM32L476RG/BootLoaderL4.bin
at 0x08000000, supported for SensorTile by its supplied readme; verification PASS.
SHA256 EBA40B1080627F92462EDCECCF3B5CAA5911E9E3F46C43FBD2416852C185588A.
Only bootloader pages 0–7 were erased. No new bootloader or mass erase.

Fresh Debug ELF programmed at its embedded address **0x08004000**, only
application pages 8–23 erased, verification PASS. Later exact 31,936-byte
readback matches Debug BIN SHA256:
E0866A948211DBC7764E178990F11CA8DA0C901731A3FF1948D99CCB944F4C5A.
Transcript: docs/evidence/flash_readback.txt; dump outside source:
../environment_backup/baseline_app_readback_20260923.bin.

GDB reset, hardware breakpoint at main PC 0x08005d78, SP 0x20018000,
backtrace and detach/continued runtime PASS (docs/evidence/gdb_main.txt).
Hardware/system reset boots the application without a manual jump.
No fault or permanent init stall observed in captured runs.
Physical unplug/replug cold-power test NOT TESTED.

After debugger use, normal/hotplug SWD at 4/8 MHz sometimes failed with
"Unable to get core ID". **Under Reset + hardware reset + 1000 kHz** succeeded
and gave the matching readback. Root cause (signal quality/debug state) is
not proven. Use this setting if reconnect fails; do not mass erase.
The first sandboxed dump could not save a file; approved execution outside
the sandbox saved it successfully.

## Sensors and UART — PASS for bench operation

COM9, UART5, 115200 8N1. Startup confirms WHO IDs 0x33/0x40, configuration
and BLE initialization. docs/evidence/uart_runtime.txt includes reset/startup;
uart_initial.txt, uart_stationary.txt and uart_final.txt capture real telemetry.

Stationary observation: 125 one-second records, tick 167006–291016
(124.01 s span). All show steps=0, sensor/BLE errors=0 and fresh samples;
30 records show BLE subscribed. Acceleration magnitude min/mean/max
952.5/963.8/986.3 mg. Both sensors varied with sample noise, not zero/frozen/
saturated. Sample-count deltas imply 51.2 Hz acceleration and 50.9 Hz magnetic.
Heading about 227 degrees is a live **uncalibrated magnetic** result, not an
independently verified bearing.

Six-face axis check, 3D rotation, deliberate physical sensor-fault injection
and calibrated heading/stride/real walking remain NOT TESTED.

## BLE transport — PASS on Windows, phone NOT TESTED

Advertising found at F9:B0:57:92:6B:52; existing 20-byte characteristic
00e00000-0001-11e1-ac36-0002a5d5c51b discovered and subscribed.
Final two-cycle run in docs/evidence/ble.json:

| Cycle | Packets | Rate | Malformed | Timestamp |
|---|---:|---:|---:|---|
| 1 | 301 | 19.96 Hz | 0 | advancing |
| 2, after disconnect/reconnect | 302 | 19.96 Hz | 0 | advancing |

Real vectors vary; decoded steps=0, distance=0 m, heading=227 degrees agree
with contemporaneous UART scale/range. UART shows connected/subscribed then
disconnected with zero send errors. Earlier two-cycle run also passed
(302 packets each, 20.03/19.98 Hz). Windows Bluetooth was initially off; user
enabled it. WinRT connections required approved execution outside the sandbox.

Course phone connection, exact UI scaling/sentinel presentation and phone
reconnect remain NOT TESTED. PC BLE results do not substitute for that test.

## Remaining acceptance

1. Course phone: connect ABCDEFG (GAP name CSys704), subscribe, check axes and
   repurposed gyroscope fields; disconnect/reconnect once.
2. Later per user preference: six-face check, assembled-device 3D calibration,
   known-direction/tilt tests and independent known-step walks.
3. Record actual errors/stride, tune with separate training trials, complete
   the separate literature review/report with real evidence.
4. Cold power-cycle and physical fault-recovery test when convenient.

This is a validated runnable baseline, not a claim of final human-motion accuracy.
README covers operation; docs/CHANGES.md lists each changed file.
