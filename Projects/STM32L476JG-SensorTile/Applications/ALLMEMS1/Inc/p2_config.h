#ifndef P2_CONFIG_H
#define P2_CONFIG_H
#define P2_FIRMWARE_VERSION "heading-handheld-10"
/* Seven bytes fit the starter's advertising layout: project/group/revision. */
#define P2_BLE_NAME "P202S01"
typedef char P2_BleNameMustBeSevenBytes[(sizeof(P2_BLE_NAME) == 8U) ? 1 : -1];
/* Sensor register axes are the body axes. Mount +X forward, +Z up.
 * Heading is clockwise from startup; course display uses whole degrees 0..359. */
#define P2_LOG_PERIOD_MS 100U /* UART telemetry only; sensor ODR remains 50 Hz. */
#define P2_LOG_LINE_BYTES 512U
#define P2_LOG_QUEUE_DEPTH 8U /* Bounded RAM queue; never wait for UART wire time. */
#define P2_ERROR_LOG_PERIOD_MS 1000U
#define P2_STEP_HIGH_MG 100.0f
#define P2_STEP_LOW_MG (-40.0f)
/* Bounded sensitivity for low-amplitude hand-held and stronger pocket motion.
 * These are initial engineering thresholds, not population-validated values. */
#define P2_STEP_PEAK_FLOOR_MG 40.0f
#define P2_STEP_TROUGH_FLOOR_MG 20.0f
#define P2_STEP_PROMINENCE_MG 90.0f
#define P2_STEP_ENVELOPE_S 1.0f
#define P2_STEP_MIN_MS 280U
/* Confirm a walking bout before publishing its first steps. A long pause
 * starts a new bout. Applies to candidate peaks, not the UART log interval. */
#define P2_STEP_MAX_MS 1500U
#define P2_STEP_CONFIRM_COUNT 3U
#define P2_STEP_WEAK_CONFIRM_COUNT 5U
#define P2_STEP_WEAK_CADENCE_RATIO 1.25f
#define P2_STEP_CADENCE_RATIO 1.6f
/* Deliberate large back-and-forth handling is not a walking bout. Work on
 * normalized acceleration direction, independently of heading/gyro state.
 * These conservative heuristics may reject walking with exaggerated arm
 * rotations; human carrying-mode accuracy still requires measurement. */
#define P2_STEP_DIRECTION_FILTER_S 0.10f
#define P2_STEP_ROTATION_MIN_RAD 0.002f
#define P2_STEP_ROTATION_AXIS_COS 0.70f
#define P2_STEP_ROTATION_SWEEP_RAD 0.52359878f /* 30 degrees */
#define P2_STEP_ROTATION_REVERSE_RAD 0.26179939f /* 15 degrees hysteresis */
#define P2_STEP_ROTATION_REPEAT_MS 3000U /* Require two reversals, not one reposition. */
#define P2_STEP_HANDLING_HOLD_MS 2000U
#define P2_STEP_HANDLING_QUIET_MS 1000U
#define P2_STEP_HANDLING_QUIET_COS 0.96592583f /* Within a 15-degree direction cone. */
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
/* Absolute north alignment is unnecessary for boot-relative heading. */
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
/* Dedicated heading tilt vector. Never change the pedometer baseline above. */
#define P2_TILT_FILTER_S 0.03f
/* Gyro-aided tilt. Engineering safeguards, not validated accuracy limits. */
#define P2_GYRO_FRESH_MS 50U
#define P2_GYRO_MAX_DPS 2000.0f /* Per axis, matching CTRL2_G and sensitivity. */
#define P2_GYRO_CAL_MS 2000U
#define P2_GYRO_CAL_MIN_SAMPLES 150U
#define P2_GYRO_CAL_MAX_DPS 10.0f
#define P2_GYRO_CAL_SPREAD_DPS 10.0f
#define P2_GYRO_CAL_UP_COS_MIN 0.9986295f /* 3 degrees */
/* Initial zero is a stable multi-sample reference, not the first mag sample.
 * Three consecutive one-second circular block means must agree, after the
 * unchanged two-second gyro calibration. At 50 Hz this needs >=150 distinct
 * magnetic observations. Bounded reversing hand tremor is allowed; persistent
 * same-direction mean changes still reject a moving/drifting startup.
 * Unstable data restart the window; five seconds is an ideal, not a timeout. */
#define P2_STARTUP_BLOCK_MS 1000U
#define P2_STARTUP_BLOCKS 3U
#define P2_STARTUP_MIN_SAMPLES 150U
#define P2_STARTUP_RATE_MAX_DPS 6.0f
#define P2_STARTUP_ACC_NORM_TOL_MG 40.0f
#define P2_STARTUP_UP_COS_MIN 0.9961947f /* 5 degrees, bounded handheld sway */
#define P2_STARTUP_MAG_MAX_DEG 8.0f
#define P2_STARTUP_MEAN_SPAN_DEG 1.0f
#define P2_STARTUP_MEAN_TREND_DEG 0.125f /* Two consecutive signed block increments */
#define P2_STARTUP_FIELD_SPAN_RATIO 0.10f
#define P2_FUSION_ACC_FILTER_S 2.0f
#define P2_FUSION_ACC_NORM_TOL_MG 80.0f
#define P2_FUSION_ACC_COS_MIN 0.9781476f /* 12 degrees */
#define P2_FUSION_MAG_STRENGTH_RATIO 0.20f
#define P2_FUSION_MAG_DIP_MAX_DEG 15.0f
#define P2_FUSION_MAG_INNOVATION_DEG 20.0f
#define P2_FUSION_MAG_FILTER_S 5.0f
#define P2_FUSION_MAG_CORRECTION_DPS 0.5f
/* Magnetic checks gate aiding, not fresh gyro-transported heading output.
 * There is no lifetime correction budget or five-second display cutoff.
 * +/-5 deg over 120 s at rest remains a measured demo target, not a guarantee
 * during persistent interference or unlimited gyro-only operation. */
#define P2_MAG_GUARD_BLOCK_MS 2000U
#define P2_MAG_GUARD_MIN_SAMPLES 50U
#define P2_MAG_GUARD_ENTRY_DEG 6.0f
#define P2_MAG_GUARD_EXIT_DEG 2.5f
/* After actual inertial loss, reacquire a stable magnetic/gravity endpoint.
 * Preserve the original zero and steps; do not reconstruct unobserved turns. */
#define P2_HEADING_RECOVERY_MS 2000U
#define P2_HEADING_RECOVERY_MIN_SAMPLES 80U
#define P2_HEADING_RECOVERY_RATE_DPS 15.0f
#define P2_HEADING_RECOVERY_STRENGTH_RATIO 0.50f
#define P2_HEADING_RECOVERY_DIP_DEG 30.0f
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
#define P2_MAG_CALIBRATED 0 /* Enable only after independent direction/tilt validation. */
#endif
#define P2_MAG_CALIBRATION_LABEL "DIAGONAL_3D_PENDING_VALIDATION"
/* Raw magnetometer-register axes, before heading projection.
 * Diagonal 3D fit: Oct 12 2026, 14:46:10--14:50:37 NZDT, 5342 BLE rows.
 * Complete calibration CSV is retained in logs/ble_monitor/analysis locally.
 * Corrected magnitude CV=3.16%; not proof of absolute north or tilt accuracy.
 * Earlier independent half-turn replay: 181.6 deg, absolute offset unresolved.
 * Specific to this assembled board. Recalibrate after mounting changes.
 */
#define P2_MAG_BIAS_X (-298.170662f)
#define P2_MAG_BIAS_Y (74.582182f)
#define P2_MAG_BIAS_Z (258.322494f)
#define P2_MAG_SCALE_X (1.000105f)
#define P2_MAG_SCALE_Y (1.046709f)
#define P2_MAG_SCALE_Z (0.957186f)
#endif
