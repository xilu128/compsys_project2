#include "p2_motion.h"
#include "p2_config.h"
#include <math.h>
#include <string.h>

static float norm(P2_Vector v) { return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z); }
static float wrap(float x) {
    x = fmodf(x, 360.0f);
    return x < 0.0f ? x + 360.0f : x;
}
void P2_MotionInit(P2_Motion *s) {
    memset(s, 0, sizeof(*s));
    s->mag_bias = (P2_Vector){P2_MAG_BIAS_X, P2_MAG_BIAS_Y, P2_MAG_BIAS_Z};
    s->mag_scale = (P2_Vector){P2_MAG_SCALE_X, P2_MAG_SCALE_Y, P2_MAG_SCALE_Z};
}
void P2_MotionAcceleration(P2_Motion *s, P2_Vector a, uint32_t now) {
    float magnitude = norm(a);
    if (!isfinite(magnitude) || magnitude < 200.0f || magnitude > 3900.0f) {
        s->armed = false;
        s->heading_valid = false;
        return;
    }
    uint32_t elapsed = now - s->last_ms; /* unsigned arithmetic handles tick wrap */
    if (!s->initialized || elapsed > 250U) {
        s->gravity = a;
        s->baseline = magnitude;
        s->filtered = s->previous = 0.0f;
        s->armed = false;
        s->start_ms = now;
        s->last_ms = now;
        s->initialized = true;
        return;
    }
    if (elapsed == 0U) return;
    float dt = elapsed * 0.001f;
    s->last_ms = now;
    float slow = dt / (0.6f + dt);
    s->gravity.x += slow * (a.x - s->gravity.x);
    s->gravity.y += slow * (a.y - s->gravity.y);
    s->gravity.z += slow * (a.z - s->gravity.z);
    s->baseline += slow * (magnitude - s->baseline);
    float fast = dt / (0.06f + dt);
    s->previous = s->filtered;
    s->filtered += fast * (magnitude - s->baseline - s->filtered);
    if (now - s->start_ms < 1000U) return;
    /* Require a trough followed by a positive falling peak, plus refractory time.
     * Magnitude makes counting independent of the fixed mounting orientation. */
    if (s->filtered < P2_STEP_LOW_MG) s->armed = true;
    if (s->armed && s->previous > P2_STEP_HIGH_MG && s->filtered < s->previous &&
        (s->steps == 0U || now - s->peak_ms >= P2_STEP_MIN_MS)) {
        if (s->steps != UINT32_MAX) ++s->steps;
        s->distance_m = s->steps * P2_STEP_LENGTH_M;
        s->peak_ms = now;
        s->armed = false;
    }
}
bool P2_MotionHeading(P2_Motion *s, P2_Vector m) {
    s->heading_valid = false;
    if (!s->initialized) return false;
    m.x = (m.x - s->mag_bias.x) * s->mag_scale.x;
    m.y = (m.y - s->mag_bias.y) * s->mag_scale.y;
    m.z = (m.z - s->mag_bias.z) * s->mag_scale.z;
    float mn = norm(m), gn = norm(s->gravity);
    if (!isfinite(mn) || mn < 100.0f || mn > 1000.0f || gn < 700.0f || gn > 1300.0f) return false;
    P2_Vector up = {s->gravity.x/gn, s->gravity.y/gn, s->gravity.z/gn};
    /* Project body +X and magnetic field onto the horizontal plane. */
    float dot = m.x*up.x + m.y*up.y + m.z*up.z;
    P2_Vector north = {m.x-dot*up.x, m.y-dot*up.y, m.z-dot*up.z};
    float horizontal = norm(north);
    if (horizontal < 50.0f || fabsf(up.x) > 0.95f) return false;
    /* north x up points east. atan2 is tilt compensated and wrap safe. */
    float east_x = north.y*up.z - north.z*up.y;
    float heading = wrap(atan2f(east_x, north.x)*57.295779513f + P2_HEADING_OFFSET_DEG);
    s->heading_deg = heading;
    s->heading_valid = true;
    return true;
}
