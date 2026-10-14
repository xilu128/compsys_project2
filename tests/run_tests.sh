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
${CC:-cc} -std=c99 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -I"$app/Inc" tests/test_heading.c "$app/Src/p2_motion.c" -lm -o "$out/test_heading"
"$out/test_heading"
${PYTHON:-python3} tests/test_calibration.py
${PYTHON:-python3} tests/test_packet.py
${PYTHON:-python3} tests/test_motion_replay.py
${PYTHON:-python3} tests/test_logging.py

${PYTHON:-python3} tests/test_stride_calibration.py

# Exercise the optional fully calibrated guard as well as shipping defaults.
${CC:-cc} -std=c99 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -DP2_MAG_CALIBRATED=1 -I"$app/Inc" tests/test_heading.c "$app/Src/p2_motion.c" -lm -o "$out/test_heading_calibrated"
"$out/test_heading_calibrated"

${CC:-cc} -std=c99 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -I"$app/Inc" tests/test_gait.c "$app/Src/p2_motion.c" -lm -o "$out/test_gait"
"$out/test_gait"

${CC:-cc} -std=c99 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -I"$app/Inc" tests/test_handling.c "$app/Src/p2_motion.c" -lm -o "$out/test_handling"
"$out/test_handling"

${CC:-cc} -std=c99 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -I"$app/Inc" tests/test_fusion.c "$app/Src/p2_motion.c" -lm -o "$out/test_fusion"
"$out/test_fusion"
${CC:-cc} -std=c99 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -I"$app/Inc" tests/test_startup.c "$app/Src/p2_motion.c" -lm -o "$out/test_startup"
"$out/test_startup"
${CC:-cc} -std=c99 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -I"$app/Inc" tests/test_handheld_init.c "$app/Src/p2_motion.c" -lm -o "$out/test_handheld_init"
"$out/test_handheld_init"
${CC:-cc} -std=c99 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -I"$app/Inc" tests/test_mag_consistency.c "$app/Src/p2_motion.c" -lm -o "$out/test_mag_consistency"
"$out/test_mag_consistency"
${CC:-cc} -std=c99 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -I"$app/Inc" tests/test_gyro_sensor.c "$app/Src/p2_sensor.c" -lm -o "$out/test_gyro_sensor"
"$out/test_gyro_sensor"
