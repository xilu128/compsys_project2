#include "p2_motion.h"
#include "p2_config.h"
#include <math.h>
#include <string.h>

static float norm(P2_Vector v) { return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z); }
static float wrap(float x) {
    x = fmodf(x, 360.0f);
    return x < 0.0f ? x + 360.0f : x;
}

/* Discard unconfirmed motion, but preserve already published steps. */
static void reset_bout(P2_Motion *s) {
    s->pending_steps = 0U;
    s->pending_distance_m = 0.0f;
    s->candidate_interval_ms = 0U;
    s->have_peak = false;
    s->walking = false;
    s->armed = false;
}

bool P2_MotionSetStride(P2_Motion *s, P2_StrideModel m) {
    if (s->have_peak || s->walking || !isfinite(m.fixed_m) ||
        m.fixed_m < P2_STRIDE_MIN_M || m.fixed_m > P2_STRIDE_MAX_M) return false;
    if (m.adaptive) {
        if (!isfinite(m.intercept_m) || !isfinite(m.slope_m_per_hz) ||
            !isfinite(m.min_hz) || !isfinite(m.max_hz) ||
            m.min_hz < 1000.0f/P2_STEP_MAX_MS ||
            m.max_hz > 1000.0f/P2_STEP_MIN_MS || m.min_hz >= m.max_hz ||
            m.slope_m_per_hz < 0.0f) return false;
        float lo = m.intercept_m + m.slope_m_per_hz*m.min_hz;
        float hi = m.intercept_m + m.slope_m_per_hz*m.max_hz;
        if (!isfinite(lo) || !isfinite(hi) || lo < P2_STRIDE_MIN_M ||
            hi > P2_STRIDE_MAX_M) return false;
    }
    s->stride = m;
    return true;
}

static float step_length(const P2_Motion *s, uint32_t interval) {
    if (!s->stride.adaptive || interval == 0U) return s->stride.fixed_m;
    float hz = 1000.0f/interval;
    /* Do not extrapolate outside the measured calibration range. */
    if (hz < s->stride.min_hz || hz > s->stride.max_hz) return s->stride.fixed_m;
    return s->stride.intercept_m + s->stride.slope_m_per_hz*hz;
}

static void add_steps(P2_Motion *s, uint32_t count, float distance) {
    uint32_t accepted = count > UINT32_MAX-s->steps ? UINT32_MAX-s->steps : count;
    if (accepted == 0U) return;
    s->steps += accepted;
    /* Compensated sum avoids repeated float rounding at ordinary walk lengths.
     * At count saturation, accept only the corresponding fraction of a bout. */
    float increment = distance*((float)accepted/count) - s->distance_compensation;
    float total = s->distance_m + increment;
    s->distance_compensation = (total-s->distance_m)-increment;
    s->distance_m = total;
}

static void accept_candidate(P2_Motion *s, uint32_t now) {
    uint32_t interval = now - s->peak_ms;
    if (!s->have_peak) {
        s->pending_steps = 1U;
        s->pending_distance_m = 0.0f; /* First interval not known yet. */
    } else if (s->walking) {
        /* Allow cadence changes once walking is established. */
        add_steps(s, 1U, step_length(s, interval));
    } else {
        /* Two adjacent intervals must agree before the first three candidates
         * become steps. Irregular handling starts a fresh confirmation window. */
        bool consistent = s->candidate_interval_ms == 0U ||
            ((float)interval <= s->candidate_interval_ms * P2_STEP_CADENCE_RATIO &&
             (float)s->candidate_interval_ms <= interval * P2_STEP_CADENCE_RATIO);
        float length = step_length(s, interval);
        /* Infer the first step from the first observed interval. After an
         * inconsistent cadence, retain only the latest pair and its distance. */
        if (!consistent || s->pending_steps == 1U) s->pending_distance_m = 2.0f*length;
        else s->pending_distance_m += length;
        s->pending_steps = consistent ? s->pending_steps + 1U : 2U;
        if (s->pending_steps >= P2_STEP_CONFIRM_COUNT) {
            add_steps(s, s->pending_steps, s->pending_distance_m); /* Backfill, do not lose startup steps. */
            s->pending_steps = 0U;
            s->pending_distance_m = 0.0f;
            s->walking = true;
        }
    }
    s->candidate_interval_ms = s->have_peak ? interval : 0U;
    s->peak_ms = now;
    s->have_peak = true;
    s->armed = false;
}

void P2_MotionInit(P2_Motion *s) {
    memset(s, 0, sizeof(*s));
    s->stride = (P2_StrideModel){0.70f,0.70f,0.0f,0.8f,2.5f,false};
    (void)P2_MotionSetStride(s, (P2_StrideModel){P2_STEP_LENGTH_M,
        P2_STRIDE_INTERCEPT_M,P2_STRIDE_SLOPE_M_PER_HZ,
        P2_STRIDE_MIN_HZ,P2_STRIDE_MAX_HZ,P2_STRIDE_ADAPTIVE != 0});
    s->heading_status = P2_HEADING_UNAVAILABLE;
    s->mag_bias = (P2_Vector){P2_MAG_BIAS_X, P2_MAG_BIAS_Y, P2_MAG_BIAS_Z};
    s->mag_scale = (P2_Vector){P2_MAG_SCALE_X, P2_MAG_SCALE_Y, P2_MAG_SCALE_Z};
}
void P2_MotionAcceleration(P2_Motion *s, P2_Vector a, uint32_t now) {
    float magnitude = norm(a);
    if (!isfinite(magnitude) || magnitude < P2_ACC_MIN_MG || magnitude > P2_ACC_MAX_MG) {
        reset_bout(s);
        s->heading_valid = false;
        s->heading_status = P2_HEADING_UNAVAILABLE;
        /* Discard the old gravity estimate after a drop/impact/invalid sample.
         * A new magnetic sample must not make that old estimate valid again. */
        s->initialized = false;
        return;
    }
    uint32_t elapsed = now - s->last_ms; /* unsigned arithmetic handles tick wrap */
    if (!s->initialized || elapsed > P2_SAMPLE_GAP_MS) {
        s->magnetic_reference = 0.0f;
        s->magnetic_count = 0U;
        s->magnetic_seen = s->magnetic_blocked = s->recovery_active = false;
        s->gravity = a;
        s->baseline = magnitude;
        s->filtered = s->previous = 0.0f;
        reset_bout(s);
        s->start_ms = now;
        s->last_ms = now;
        s->initialized = true;
        s->heading_valid = false;
        s->heading_status = P2_HEADING_UNAVAILABLE;
        return;
    }
    if (elapsed == 0U) return;
    float dt = elapsed * 0.001f;
    s->last_ms = now;
    if (s->have_peak && now - s->peak_ms > P2_STEP_MAX_MS) reset_bout(s);
    float slow = dt / (P2_GRAVITY_FILTER_S + dt);
    s->gravity.x += slow * (a.x - s->gravity.x);
    s->gravity.y += slow * (a.y - s->gravity.y);
    s->gravity.z += slow * (a.z - s->gravity.z);
    s->baseline += slow * (magnitude - s->baseline);
    float fast = dt / (P2_MOTION_FILTER_S + dt);
    s->previous = s->filtered;
    s->filtered += fast * (magnitude - s->baseline - s->filtered);
    if (now - s->start_ms < P2_SETTLE_MS) return;
    /* Require a trough followed by a positive falling peak, plus refractory time.
     * Magnitude makes counting independent of the fixed mounting orientation. */
    if (s->filtered < P2_STEP_LOW_MG) {
        s->armed = true;
        s->trough_ms = now;
    }
    /* Do not pair an isolated trough with an unrelated motion much later. */
    if (s->armed && now - s->trough_ms > P2_TROUGH_TIMEOUT_MS) s->armed = false;
    if (s->armed && s->previous > P2_STEP_HIGH_MG && s->filtered < s->previous &&
        (!s->have_peak || now - s->peak_ms >= P2_STEP_MIN_MS)) {
        accept_candidate(s, now);
    }
}
bool P2_MotionHeading(P2_Motion *s, P2_Vector m) {
    bool was_valid = s->heading_valid;
    s->heading_valid = false;
    s->heading_status = P2_HEADING_UNAVAILABLE;
    /* Any invalid observation interrupts reference/recovery confirmation. */
    uint32_t prior_count = s->magnetic_count;
    bool prior_recovery = s->recovery_active;
    s->magnetic_count = 0U;
    s->recovery_active = false;
    if (!s->initialized || s->last_ms - s->start_ms < P2_SETTLE_MS) return false;
    if (!isfinite(s->mag_scale.x) || !isfinite(s->mag_scale.y) ||
        !isfinite(s->mag_scale.z) || s->mag_scale.x <= 0.0f ||
        s->mag_scale.y <= 0.0f || s->mag_scale.z <= 0.0f) return false;
    m.x = (m.x - s->mag_bias.x) * s->mag_scale.x;
    m.y = (m.y - s->mag_bias.y) * s->mag_scale.y;
    m.z = (m.z - s->mag_bias.z) * s->mag_scale.z;
    float mn = norm(m), gn = norm(s->gravity);
    if (!isfinite(mn) || !isfinite(gn) || mn < P2_MAG_MIN_MGAUSS || mn > P2_MAG_MAX_MGAUSS ||
        gn < P2_GRAVITY_MIN_MG || gn > P2_GRAVITY_MAX_MG) return false;
    P2_Vector up = {s->gravity.x/gn, s->gravity.y/gn, s->gravity.z/gn};
    /* Project body +X and magnetic field onto the horizontal plane. */
    float dot = m.x*up.x + m.y*up.y + m.z*up.z;
    P2_Vector north = {m.x-dot*up.x, m.y-dot*up.y, m.z-dot*up.z};
    float horizontal = norm(north);
    if (horizontal < P2_HORIZONTAL_MIN_MGAUSS || fabsf(up.x) > P2_FORWARD_VERTICAL_LIMIT) return false;
    uint32_t now = s->last_ms;
    bool gap = !s->magnetic_seen || now - s->magnetic_last_ms > P2_MAG_FRESH_MS;
    bool duplicate = s->magnetic_seen && now == s->magnetic_last_ms;
    s->magnetic_last_ms = now;
    s->magnetic_seen = true;
    s->magnetic_count = gap ? 0U : prior_count;
    s->recovery_active = !gap && prior_recovery;
    if (s->magnetic_reference == 0.0f) {
        s->heading_status = P2_HEADING_CALIBRATING;
        if (s->magnetic_count == 0U ||
            fmaxf(s->magnetic_max, mn) - fminf(s->magnetic_min, mn) >
                P2_MAG_STABILITY_RATIO * mn) {
            s->magnetic_start_ms = now;
            s->magnetic_sum = s->magnetic_min = s->magnetic_max = mn;
            s->magnetic_count = 1U;
            return false;
        }
        if (duplicate) return false;
        s->magnetic_sum += mn;
        s->magnetic_count++;
        s->magnetic_min = fminf(s->magnetic_min, mn);
        s->magnetic_max = fmaxf(s->magnetic_max, mn);
        if (now - s->magnetic_start_ms < P2_MAG_REFERENCE_MS) return false;
        s->magnetic_reference = s->magnetic_sum / s->magnetic_count;
    }
    /* Never learn a sustained disturbance as the new normal. Strength alone
     * cannot detect direction-only interference or contaminated startup. */
    if (fabsf(mn - s->magnetic_reference) >
            P2_MAG_RELATIVE_CHANGE_MAX * s->magnetic_reference) {
        s->magnetic_blocked = true;
        s->recovery_active = false;
        s->heading_status = P2_HEADING_DISTURBED;
        return false;
    }
    if (s->magnetic_blocked) {
        s->heading_status = P2_HEADING_RECOVERING;
        if (!s->recovery_active) {
            s->recovery_start_ms = now;
            s->recovery_active = true;
        }
        if (now - s->recovery_start_ms < P2_MAG_RECOVERY_MS) return false;
        s->magnetic_blocked = s->recovery_active = false;
    }
    /* Signed horizontal component in the register-axis basis. The explicit
     * board yaw convention is verified separately from this projection. */
    float east_x = north.y*up.z - north.z*up.y;
    float heading = wrap(P2_HEADING_SIGN * atan2f(east_x, north.x)*57.295779513f + P2_HEADING_OFFSET_DEG);
    /* Average unit vectors, not degree values: 359/1 degrees must average at
     * north, never south. Invalid input above is immediately invalid; after
     * invalidity/gaps seed from the new observation, not an old output. */
    float radians = heading * 0.01745329252f;
    uint32_t elapsed = s->last_ms - s->heading_last_ms;
    float sn = sinf(radians), cs = cosf(radians);
    if (!was_valid || elapsed > P2_SAMPLE_GAP_MS) {
        s->heading_sin = sn;
        s->heading_cos = cs;
    } else if (elapsed != 0U) {
        float dt = elapsed * 0.001f;
        float alpha = dt / (P2_HEADING_FILTER_S + dt);
        s->heading_sin += alpha * (sn - s->heading_sin);
        s->heading_cos += alpha * (cs - s->heading_cos);
        /* An abrupt ~180-degree change can cancel the vectors. Re-seed rather
         * than publish atan2(0,0). Normal smooth rotations never need this. */
        if (s->heading_sin*s->heading_sin + s->heading_cos*s->heading_cos < 0.0001f) {
            s->heading_sin = sn;
            s->heading_cos = cs;
        }
    }
    s->heading_last_ms = s->last_ms;
    s->heading_deg = wrap(atan2f(s->heading_sin, s->heading_cos)*57.295779513f);
    s->heading_valid = true;
    return true;
}
