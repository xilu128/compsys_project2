/** Portable motion processing; units: mg, mGauss, degrees, metres. */
#ifndef P2_MOTION_H
#define P2_MOTION_H
#include <stdint.h>
#include <stdbool.h>
typedef struct { float x, y, z; } P2_Vector;
typedef struct {
    uint32_t steps, last_ms, peak_ms, start_ms, trough_ms;
    uint32_t pending_steps, candidate_interval_ms;
    float distance_m, heading_deg, baseline, filtered, previous;
    P2_Vector gravity, mag_bias, mag_scale;
    bool initialized, armed, heading_valid;
    bool have_peak, walking;
} P2_Motion;
void P2_MotionInit(P2_Motion *state);
void P2_MotionAcceleration(P2_Motion *state, P2_Vector acc, uint32_t now_ms);
bool P2_MotionHeading(P2_Motion *state, P2_Vector mag);
#endif
