#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
app=Projects/STM32L476JG-SensorTile/Applications/ALLMEMS1
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
${CC:-cc} -std=c99 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -I"$app/Inc" tests/test_p2.c "$app/Src/p2_motion.c" "$app/Src/p2_sensor.c" -lm -o "$out/test_p2"
"$out/test_p2"
${CC:-cc} -std=c99 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -I"$app/Inc" tests/test_step_bouts.c "$app/Src/p2_motion.c" -lm -o "$out/test_step_bouts"
"$out/test_step_bouts"
${PYTHON:-python3} tests/test_calibration.py
${PYTHON:-python3} tests/test_packet.py
${PYTHON:-python3} tests/test_motion_replay.py
${PYTHON:-python3} tests/test_logging.py
