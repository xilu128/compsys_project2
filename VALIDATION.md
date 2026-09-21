# Validation record — 2026-09-21

## Toolchain builds

STM32CubeIDE 2.2.0 headless managed build on macOS arm64,
GNU Tools for STM32 14.3.rel1 (GCC 14.3.1), Cortex-M4, softfp ABI.

| Configuration | Compiler/linker result | text | data | bss (includes reserved heap/stack) |
|---|---|---:|---:|---:|
| Debug | 0 errors, 0 warnings | 30,808 B | 112 B | 14,656 B |
| Release | 0 errors, 0 warnings | 27,128 B | 112 B | 14,656 B |

The Release build needed removal of a legacy MotionPM library reference to a
nonexistent old workspace project. Both configurations exclude unused
HWAdvanceFeatures.c. Initialization arrays are explicitly read-only to avoid
RWX flash load segments with the modern linker. UART logging has a proper
prototype and bounded snprintf formatting. All new modules are linked into
the normal CubeIDE project, not only a standalone desktop test build.

## Desktop tests

C99, -Wall -Wextra -Werror -pedantic, AddressSanitizer and
UndefinedBehaviorSanitizer: passed. No sanitizer findings.

- 60 seconds stationary with small synthetic noise: zero counted steps.
- 20-second sinusoidal gait traces: 19 / 39 / 59 detected at 1 / 2 / 3 Hz,
  respectively, versus 20 / 40 / 60 cycles. The first incomplete trough-to-peak
  cycle is intentionally not counted. These are synthetic signals, not people.
- 2 Hz trace crossing uint32 millisecond wrap: 39 counts, same as non-wrap.
- Fixed-stride distance, cardinal magnetic headings and analytic 30-degree
  tilt cases: assertions passed.
- Weak/strong/NaN magnetic field and vertical forward axis: rejected.
- Sampling gap resets transient state without adding a step.
- Mock identity/configuration, data-ready, signed acceleration/magnetic unit
  conversion and bus failures: assertions passed.
- BLE int16 saturation and negative invalid-heading encoding: passed.
- Python calibration test: recovers known synthetic ellipsoid offsets/scales,
  rejects too few points and insufficient 3D coverage.

Run the checked-in tests using the commands in README.md. Build logs are saved
outside the deliverable tree under project2/build-logs on the development Mac.

## Not validated

The user reported that hardware is not connected. No flash/programming operation
was performed. Real sensor timing, Bluetooth/mobile decoding, bootloader
compatibility, mounting, measured calibration, stationary false positives and
human gait/heading accuracy require the README hardware acceptance sequence.
Do not use synthetic test outcomes as measured results in the course report.
