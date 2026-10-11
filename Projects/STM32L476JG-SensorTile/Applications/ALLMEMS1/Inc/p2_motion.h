/** Portable motion processing; units: mg, mGauss, degrees, metres. */
#ifndef P2_MOTION_H
#define P2_MOTION_H
#include <stdint.h>
#include <stdbool.h>
typedef struct { float x, y, z; } P2_Vector;
/* Positive status values are sent as negative heading sentinels over BLE. */
typedef enum { P2_HEADING_UNAVAILABLE=1, P2_HEADING_CALIBRATING=2,
    P2_HEADING_DISTURBED=3, P2_HEADING_RECOVERING=4 } P2_HeadingStatus;
typedef struct {
    float fixed_m, intercept_m, slope_m_per_hz, min_hz, max_hz;
    bool adaptive;
} P2_StrideModel;
typedef struct {
    uint32_t steps, last_ms, peak_ms, start_ms, trough_ms;
    uint32_t pending_steps, candidate_interval_ms;
    P2_StrideModel stride;
    float pending_distance_m, distance_compensation;
    float distance_m, heading_deg, baseline, filtered, previous;
    uint32_t heading_last_ms;
    float heading_sin, heading_cos;
    float magnetic_reference, magnetic_sum, magnetic_min, magnetic_max;
    uint32_t magnetic_count, magnetic_start_ms, magnetic_last_ms, recovery_start_ms;
    bool magnetic_seen, magnetic_blocked, recovery_active;
    P2_HeadingStatus heading_status;
    P2_Vector gravity, mag_bias, mag_scale;
    bool initialized, armed, heading_valid;
    bool have_peak, walking;
} P2_Motion;
void P2_MotionInit(P2_Motion *state);
/* Changes apply to future bouts only; rejects updates during pending/walking. */
bool P2_MotionSetStride(P2_Motion *state, P2_StrideModel model);
void P2_MotionAcceleration(P2_Motion *state, P2_Vector acc, uint32_t now_ms);
bool P2_MotionHeading(P2_Motion *state, P2_Vector mag);
#endif
