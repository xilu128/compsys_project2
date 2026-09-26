# Local baseline completion — file inventory

Only the current local project was used. No GitHub comparison/sync/commit/push.
Existing Project 2 modules were repaired/reused, not rewritten.
APP means Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/.

| File | Purpose and changes in this completion pass |
|---|---|
| APP/Inc/p2_config.h | Centralized filters, validity, freshness, retry, stall, settling and trough limits; explicit UNCALIBRATED flag; retained stride/threshold/calibration defaults |
| APP/Inc/p2_motion.h | Added trough timestamp to portable motion state |
| APP/Src/p2_motion.c | Reject stale/invalid gravity, settle after gaps, validate scales, expire old troughs; use configuration constants |
| APP/Src/main.c | Retry sensor init, detect stalls, preserve counts during recovery, track freshness/errors, rate-limit telemetry, send invalid sentinels and gate BLE |
| APP/Src/sensor_service.c | Retain ST stack/service; accept handle zero, clear disconnect state, validate CCCD, log connections without event spam; correct LED header |
| APP/STM32CubeIDE/.cproject | Correct stale refresh-scope names; no flash-layout change |
| APP/STM32CubeIDE/STM32L476JG-SensorTile_ALLMEMS1.launch | Workspace-relative debugger log path |
| tests/test_p2.c | Regression cases for stale gravity, settling, invalid scales/gravity, trough expiry and refractory behavior |
| tests/test_calibration.py | Nonfinite calibration rejection tests |
| tests/test_packet.py | New decoder/unit/length/invalid-heading tests |
| tests/run_tests_windows.ps1 | New host-test runner with compiler selection and scoped temporary cleanup |
| tests/run_tests.sh | Normalize only this shell script to LF; run both Python suites after C tests (Unix sanitizer runner not executed on Windows) |
| tools/calibrate_mag.py | Reject nonfinite/malformed vectors |
| tools/find_cubeide.ps1 | New environment/PATH/registry/folder discovery |
| tools/build_windows.ps1 | Existing-project import/clean builds; verify summary and fresh ELF |
| tools/check_hardware.py | New UART capture and BLE subscription/packet/reconnect verification; never flashes |
| tools/package_submission.py | New source-layout ZIP, excludes generated files, no overwrite, CRC check |
| README.md | Windows setup/import/build/flash, boot recovery, protocols, modules, calibration and limits |
| VALIDATION.md | Scoped PASS/NOT TESTED results, hashes, actual hardware evidence and remaining work |
| docs/CHANGES.md | This inventory |
| docs/evidence/desktop_tests.txt | Actual host-test output |
| docs/evidence/gdb_main.txt | Actual hardware breakpoint/backtrace/detach |
| docs/evidence/flash_readback.txt | Actual target/readback/reset transcript |
| docs/evidence/uart_initial.txt | Initial post-flash telemetry |
| docs/evidence/uart_runtime.txt | Runtime plus reset/startup log |
| docs/evidence/uart_stationary.txt | 125 records including BLE tests |
| docs/evidence/uart_final.txt | Final runtime after reset |
| docs/evidence/ble.json | Full final two-cycle live BLE capture |

p2_sensor.c/.h and p2_packet.h are existing Project 2 modules reused and tested,
not changed in this pass. .project, startup and linker layout are retained.
.settings/language.settings.xml is IDE-managed environment state, not algorithm
code. Drivers, Middlewares and Utilities remain teacher/ST dependencies; their
source and notices were not edited. The original sibling starter is intact.

Generated Debug/Release and Python caches are moved recoverably to
../build_artifacts/20260923/, outside source. Toolchains, validation environment,
full flash backups, build workspaces and previous logs remain outside the
deliverable. No broad workspace cleanup was performed.
