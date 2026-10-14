# Group 02 source submission guide

## Contents and source identity

The archive is project2_group02.zip, with one complete COMSYS704 directory.
It contains the existing project, application code, vendor dependencies,
licenses, test fixtures and helper scripts. The final report is submitted
separately. SUBMISSION_MANIFEST.json records the exact Git commit and SHA-256
of every packaged source file. Keep this file when sharing the archive.

The packager reads a committed Git revision, not local working files. Untracked
files and local edits are not included. Generated Debug/Release output, caches,
logs and IDE workspace state are excluded. Supplied Utilities/BootLoader
binaries remain as vendor assets; they are not the firmware to flash.

## Windows prerequisites

- STM32CubeIDE 2.2.0 was validated, with bundled GNU Tools for STM32 14.3.rel1
  (GCC 14.3.1), make, debugger and CubeProgrammer integration.
- Install ST-LINK USB debug and Virtual COM Port drivers. Use a USB data cable
  and an ST-LINK connected to the SensorTile debug interface and target power.
- No separate Keil, IAR, CubeMX or old GCC 7 installation is needed. The linker
  script uses READONLY and requires a modern linker.
- Optional desktop tools require Python 3, Tkinter, bleak and pyserial; install
  with `python -m pip install -r tools/requirements.txt`.
- Host tests additionally require desktop clang, GCC or Zig, not ARM GCC.
  Git is required only to produce a new commit-based archive, not to build
  an extracted submission.

## Import and build the existing project

1. Extract the whole COMSYS704 directory into a short writable path. Keep
   Drivers, Middlewares, Utilities and Projects together.
2. In CubeIDE select File > Import > General > Existing Projects into Workspace.
3. Select COMSYS704/Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/STM32CubeIDE.
4. Import the project named COMSYS704. Leave Copy projects into workspace
   unchecked, because sources and libraries are linked by relative paths.
5. Select the bundled STM32 toolchain if prompted to migrate the old project.
   Do not generate a replacement project. Choose Debug, then Clean and Build.
6. The ELF and BIN appear in the project's Debug directory. The current
   submission is validated using Debug; Release is available but is not the
   current revision's acceptance configuration.

Alternatively, from the extracted COMSYS704 root:

```powershell
.\tools\build_windows.ps1 -Configuration Debug -IdeDirectory 'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE'
```

Replace the example IDE path with the installed location. The build helper
creates a separate workspace beside the source root and does not flash.

## Test and program

Run host tests from the root, with an installed desktop compiler:

```powershell
.\tests\run_tests_windows.ps1 -Compiler 'C:\tools\zig\zig.exe' -Python python
python tests/test_submission.py
```

The existing Unix runner is tests/run_tests.sh. Recorded replay fixtures are
10/20 Hz snapshots, not complete sensor streams; passing replay is not a new
physical walking accuracy measurement. See VALIDATION.md for known limits.

Build before programming. In STM32CubeProgrammer select ST-LINK/SWD, connect,
download the built ELF, verify and reset. For a raw BIN use address 0x08000000.
The vector offset and linker origin are both zero-offset standalone settings;
do not use the historical 0x08004000 address or flash a supplied bootloader.
Back up any firmware that must be retained before replacing it.

Hold the board gently away from metal while startup completes; small tremor
is allowed but deliberate rotation may restart calibration. The first accepted
direction becomes zero. Reconnection/recovery does not re-zero; reset does.
Before the interview, separately verify charged battery-only operation with
all cables disconnected and the actual course marking app.

## Recreate a source archive

Commit the intended changes first, then run from a Git checkout:

```powershell
python tools/package_submission.py --group 02 --revision HEAD --output ..\project2_group02.zip
```

An existing ZIP is never overwritten. The script validates required project
files and linked resources, checks ZIP CRCs, and prints the archive SHA-256.
Check the manifest commit against the intended merged main before submission.
No source deletion or removal of vendor licenses is required to make the ZIP.
