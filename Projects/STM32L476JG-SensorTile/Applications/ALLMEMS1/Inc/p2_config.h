#ifndef P2_CONFIG_H
#define P2_CONFIG_H
/* Sensor register axes are the body axes. Mount +X forward, +Z up.
 * Heading is clockwise from magnetic north; verify axes with the physical board. */
#define P2_LOG_PERIOD_MS 100U /* UART telemetry only; sensor ODR remains 50 Hz. */
#define P2_LOG_LINE_BYTES 512U
#define P2_LOG_QUEUE_DEPTH 8U /* Bounded RAM queue; never wait for UART wire time. */
#define P2_ERROR_LOG_PERIOD_MS 1000U
#define P2_STEP_HIGH_MG 100.0f
#define P2_STEP_LOW_MG (-40.0f)
#define P2_STEP_MIN_MS 280U
#define P2_STEP_LENGTH_M 0.70f
#define P2_HEADING_OFFSET_DEG 0.0f
/* Seconds, milliseconds, mg and mGauss: keep tunable physical limits here. */
#define P2_GRAVITY_FILTER_S 0.6f
#define P2_MOTION_FILTER_S 0.06f
#define P2_SETTLE_MS 1000U
#define P2_SAMPLE_GAP_MS 250U
#define P2_TROUGH_TIMEOUT_MS 2000U
#define P2_ACC_MIN_MG 200.0f
#define P2_ACC_MAX_MG 3900.0f
#define P2_GRAVITY_MIN_MG 700.0f
#define P2_GRAVITY_MAX_MG 1300.0f
#define P2_MAG_MIN_MGAUSS 100.0f
#define P2_MAG_MAX_MGAUSS 1000.0f
#define P2_HORIZONTAL_MIN_MGAUSS 50.0f
#define P2_FORWARD_VERTICAL_LIMIT 0.95f
#define P2_ACC_FRESH_MS 100U
#define P2_MAG_FRESH_MS 200U
#define P2_SENSOR_RETRY_MS 1000U
#define P2_SENSOR_STALL_MS 1000U
#define P2_MAG_CALIBRATED 0 /* Set to 1 only after measuring calibration. */
/* Measured hard-iron offsets and diagonal soft-iron scale, in sensor axes.
 * Replace these after a full 3D calibration; defaults are uncalibrated. */
#define P2_MAG_BIAS_X 0.0f
#define P2_MAG_BIAS_Y 0.0f
#define P2_MAG_BIAS_Z 0.0f
#define P2_MAG_SCALE_X 1.0f
#define P2_MAG_SCALE_Y 1.0f
#define P2_MAG_SCALE_Z 1.0f
#endif
