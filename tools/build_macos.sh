#!/bin/sh
# Run from any directory; separate headless workspace avoids locking the GUI workspace.
set -eu
root=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
ide=${STM32CUBEIDE_EXECUTABLE:-/Applications/STM32CubeIDE.app/Contents/MacOS/STM32CubeIDE}
workspace=${P2_BUILD_WORKSPACE:-"$root/../p2-headless-workspace"}
"$ide" --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data "$workspace" -import "$root/Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1/STM32CubeIDE" -cleanBuild COMSYS704/Debug -cleanBuild COMSYS704/Release
