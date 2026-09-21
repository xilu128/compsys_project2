#ifndef P2_CONFIG_H
#define P2_CONFIG_H
/* Sensor register axes are the body axes. Mount +X forward, +Z up.
 * Heading is clockwise from magnetic north; verify axes with the physical board. */
#define P2_LOG_PERIOD_MS 1000U /* Use 100U temporarily when collecting calibration data. */
#define P2_STEP_HIGH_MG 100.0f
#define P2_STEP_LOW_MG (-40.0f)
#define P2_STEP_MIN_MS 280U
#define P2_STEP_LENGTH_M 0.70f
#define P2_HEADING_OFFSET_DEG 0.0f
/* Measured hard-iron offsets and diagonal soft-iron scale, in sensor axes.
 * Replace these after a full 3D calibration; defaults are uncalibrated. */
#define P2_MAG_BIAS_X 0.0f
#define P2_MAG_BIAS_Y 0.0f
#define P2_MAG_BIAS_Z 0.0f
#define P2_MAG_SCALE_X 1.0f
#define P2_MAG_SCALE_Y 1.0f
#define P2_MAG_SCALE_Z 1.0f
#endif
