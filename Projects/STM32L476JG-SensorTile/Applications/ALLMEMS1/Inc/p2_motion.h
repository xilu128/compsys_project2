
#ifndef P2_MOTION_H
#define P2_MOTION_H
#include <stdint.h>
#include <stdbool.h>
typedef struct { float x, y, z; } P2_Vector;

typedef enum { P2_HEADING_UNAVAILABLE=1, P2_HEADING_CALIBRATING=2,
    P2_HEADING_DISTURBED=3, P2_HEADING_RECOVERING=4 } P2_HeadingStatus;
typedef struct {
    float fixed_m, intercept_m, slope_m_per_hz, min_hz, max_hz;
    bool adaptive;
} P2_StrideModel;

typedef struct {
    P2_Vector mag_sum, acc_sum, gyro_sum, anchor_up;
    uint32_t start_ms, last_ms, block_ms, samples, block_samples, blocks, elapsed_ms;
    float anchor_angle, block_sin, block_cos, mean_min, mean_max;
    float field_min, field_max, mean_span, previous_mean, previous_delta;
} P2_HeadingStartup;

typedef struct {
    float correction_deg, sin_sum, cos_sum, mean_deg;
    uint32_t start_ms, last_ms, samples, verified_ms;
    bool blocked, reference_lost, seen;
} P2_MagneticGuard;

typedef struct {
    P2_Vector up, north, bias, sum, low, high, calibration_acc, acc;
    P2_Vector raw_rate, residual_rate;
    P2_HeadingStartup startup, recovery;
    P2_MagneticGuard mag_guard;
    uint32_t last_ms, start_ms, count, trusted_ms, recovery_ms;
    float acc_norm, rest_norm, rate, field_norm, dip;
    float magnetic_heading_deg, innovation_deg, correction_dps;
    bool required, seen, calibrated, ready, magnetic_accepted, recovering;
} P2_Attitude;
typedef struct {
    uint32_t steps, last_ms, peak_ms, start_ms, trough_ms;
    uint32_t pending_steps, candidate_interval_ms;
    P2_StrideModel stride;
    float pending_distance_m, distance_compensation;
    float step_envelope, step_trough;
    P2_Vector step_direction, step_rotation_axis, step_quiet_direction;
    float step_rotation_forward, step_rotation_reverse;
    uint32_t step_handling_ms, step_rotation_reversal_ms, step_quiet_ms;
    bool step_rotation_axis_set, step_rotation_reversal_seen, step_handling;
    float distance_m, heading_deg, baseline, filtered, previous;
    uint32_t heading_last_ms;
    float heading_sin, heading_cos;

    float heading_origin_deg;
    bool heading_origin_set;
    float magnetic_reference, magnetic_sum, magnetic_min, magnetic_max;
    uint32_t magnetic_count, magnetic_start_ms, magnetic_last_ms, recovery_start_ms;
    bool magnetic_seen, magnetic_blocked, recovery_active;
    P2_HeadingStatus heading_status;
    P2_Attitude attitude;
    P2_Vector gravity, mag_bias, mag_scale;
    bool initialized, armed, heading_valid;
    bool have_peak, walking, weak_bout;
} P2_Motion;
void P2_MotionInit(P2_Motion *state);

bool P2_MotionSetStride(P2_Motion *state, P2_StrideModel model);
void P2_MotionAcceleration(P2_Motion *state, P2_Vector acc, uint32_t now_ms);
bool P2_MotionHeading(P2_Motion *state, P2_Vector mag);

void P2_MotionRequireGyro(P2_Motion *state);
P2_Vector P2_GyroToBody(P2_Vector native_lsm6dsm_dps);

void P2_MotionGyro(P2_Motion *state, P2_Vector rate_dps, uint32_t now_ms);
#endif
