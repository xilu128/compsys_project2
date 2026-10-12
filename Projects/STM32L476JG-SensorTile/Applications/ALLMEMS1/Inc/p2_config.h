#ifndef P2_CONFIG_H
#define P2_CONFIG_H
#define P2_FIRMWARE_VERSION "heading-availability-1"
/* Seven bytes fit the starter's advertising layout: project/group/revision. */
#define P2_BLE_NAME "P202S01"
typedef char P2_BleNameMustBeSevenBytes[(sizeof(P2_BLE_NAME) == 8U) ? 1 : -1];
/* Sensor register axes are the body axes. Mount +X forward, +Z up.
 * Heading is clockwise from magnetic north; verify axes with the physical board. */
#define P2_LOG_PERIOD_MS 100U /* UART telemetry only; sensor ODR remains 50 Hz. */
#define P2_LOG_LINE_BYTES 512U
#define P2_LOG_QUEUE_DEPTH 8U /* Bounded RAM queue; never wait for UART wire time. */
#define P2_ERROR_LOG_PERIOD_MS 1000U
#define P2_STEP_HIGH_MG 100.0f
#define P2_STEP_LOW_MG (-40.0f)
#define P2_STEP_MIN_MS 280U
/* Confirm a walking bout before publishing its first steps. A long pause
 * starts a new bout. Applies to candidate peaks, not the UART log interval. */
#define P2_STEP_MAX_MS 1500U
#define P2_STEP_CONFIRM_COUNT 3U
#define P2_STEP_CADENCE_RATIO 1.6f
/* Calibrate with measured distance, manual steps and walking-only duration.
 * Adaptive mode stays off until independently validated coefficients exist. */
#define P2_STEP_LENGTH_M 0.70f
#define P2_STRIDE_ADAPTIVE 0
#define P2_STRIDE_INTERCEPT_M 0.70f
#define P2_STRIDE_SLOPE_M_PER_HZ 0.0f
#define P2_STRIDE_MIN_HZ 0.8f
#define P2_STRIDE_MAX_HZ 2.5f
#define P2_STRIDE_MIN_M 0.25f
#define P2_STRIDE_MAX_M 1.50f
#define P2_HEADING_OFFSET_DEG 0.0f
/* Empirical board yaw convention: trial 3's labelled clockwise turn produced
 * -360 degrees with the original sign. Do not infer a 3D axis remapping from
 * two horizontal turns; raw XYZ and the tilt projection are kept unchanged. */
#define P2_HEADING_SIGN (-1.0f)
/* Heuristic strength guard, not a trained interference classifier.
 * Startup must be away from magnetic interference. */
#define P2_MAG_REFERENCE_MS 1000U
#define P2_MAG_STABILITY_RATIO 0.10f
#define P2_MAG_RECOVERY_MS 500U
#define P2_MAG_RELATIVE_CHANGE_MAX 0.35f
#define P2_HEADING_FILTER_S 0.15f /* Circular low-pass; never latch a stale heading. */
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
#ifndef P2_MAG_CALIBRATED
#define P2_MAG_CALIBRATED 0 /* Full 3D calibration is still outstanding. */
#endif
#define P2_MAG_CALIBRATION_LABEL "XY_PROVISIONAL"
/* Raw magnetometer-register axes, BEFORE mapping to accelerometer body axes.
 * XY circle fitted ONLY to trial 2 (20261005_130753_309873, first 33 s).
 * Independently checked on trial 3 (131056_903890). Trial 1 is excluded.
 * Near-horizontal, current mounting/environment only: Z bias and soft-iron
 * coupling are unobservable in these data. Do not claim a full 3D calibration. */
#define P2_MAG_BIAS_X 12.637410f
#define P2_MAG_BIAS_Y (-161.102725f)
#define P2_MAG_BIAS_Z 0.0f
#define P2_MAG_SCALE_X 1.0f
#define P2_MAG_SCALE_Y 1.0f
#define P2_MAG_SCALE_Z 1.0f
#endif
