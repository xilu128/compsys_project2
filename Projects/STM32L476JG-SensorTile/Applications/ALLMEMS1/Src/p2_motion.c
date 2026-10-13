#include "p2_motion.h"
#include "p2_config.h"
#include <math.h>
#include <string.h>

static float norm(P2_Vector v) { return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z); }
static float wrap(float x) {
    x = fmodf(x, 360.0f);
    if (x < 0.0f) x += 360.0f;
    /* Tiny negative differences can round to exactly 360 in float. */
    return x >= 360.0f ? 0.0f : x;
}

static float dot3(P2_Vector a, P2_Vector b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
static P2_Vector cross3(P2_Vector a, P2_Vector b) {
    return (P2_Vector){a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
static P2_Vector times(P2_Vector v, float k) { return (P2_Vector){v.x*k,v.y*k,v.z*k}; }
static P2_Vector unit(P2_Vector v) { return times(v,1.0f/fmaxf(norm(v),0.000001f)); }
static float clamp1(float x) { return fmaxf(-1.0f,fminf(1.0f,x)); }
/* Rodrigues rotation. Same rigid rotation MUST act on up and north. */
static P2_Vector rotate(P2_Vector v, P2_Vector axis, float angle) {
    float c=cosf(angle), sn=sinf(angle), d=dot3(axis,v)*(1.0f-c);
    P2_Vector cv=cross3(axis,v);
    return (P2_Vector){v.x*c+cv.x*sn+axis.x*d,
                      v.y*c+cv.y*sn+axis.y*d,
                      v.z*c+cv.z*sn+axis.z*d};
}
static void orthogonalize(P2_Attitude *f) {
    f->up=unit(f->up);
    float d=dot3(f->north,f->up);
    f->north=unit((P2_Vector){f->north.x-d*f->up.x,
        f->north.y-d*f->up.y,f->north.z-d*f->up.z});
}
static void reset_startup(P2_Attitude *f) {
    if (f->field_norm <= 0.0f) memset(&f->startup,0,sizeof(f->startup));
    memset(&f->recovery,0,sizeof(f->recovery));
}
static float vector_heading(P2_Vector north, P2_Vector up) {
    return wrap(P2_HEADING_SIGN*atan2f(cross3(north,up).x,north.x)*57.295779513f+
                P2_HEADING_OFFSET_DEG);
}
void P2_MotionRequireGyro(P2_Motion *s) {
    s->attitude.required=true;
    s->heading_valid=false;
}
P2_Vector P2_GyroToBody(P2_Vector native) {
    /* ST's SensorTile MotionFX_Manager: LSM6DSM gyro wsu, LSM303AGR mag swu.
     * Polar coordinates swap X/Y. Angular velocity is AXIAL: det(P)*P*w,
     * hence negate all three after swapping. A plain swap reverses transport.
     * Physical single-axis correlation remains a hardware acceptance test. */
    return (P2_Vector){-native.y,-native.x,-native.z};
}
void P2_MotionGyro(P2_Motion *s, P2_Vector rate, uint32_t now) {
    P2_Attitude *f=&s->attitude;
    float speed=norm(rate);
    uint32_t elapsed=now-f->last_ms;
    bool gap=!f->seen || elapsed>P2_GYRO_FRESH_MS;
    /* Full scale is specified independently for each sensor axis. */
    if (!isfinite(speed) || fabsf(rate.x)>P2_GYRO_MAX_DPS ||
        fabsf(rate.y)>P2_GYRO_MAX_DPS || fabsf(rate.z)>P2_GYRO_MAX_DPS) {
        if (s->heading_origin_set) f->mag_guard.reference_lost=true;
        f->seen=f->ready=f->recovering=false;
        f->count=0U;
        reset_startup(f);
        s->heading_valid=false;
        return;
    }
    if (f->seen && elapsed==0U) return;
    f->raw_rate=rate;
    f->last_ms=now;
    f->seen=true;
    if (gap) {
        if (s->heading_origin_set) f->mag_guard.reference_lost=true;
        f->ready=f->recovering=false; f->count=0U; s->heading_valid=false;
        reset_startup(f);
    }
    if (!f->calibrated) {
        /* Two continuous stationary seconds. Never estimate bias from a
         * moving startup or let duplicate calls satisfy this window. */
        if (!s->initialized || now-s->last_ms>P2_ACC_FRESH_MS ||
            f->acc_norm<850.0f || f->acc_norm>1150.0f ||
            speed>P2_GYRO_CAL_MAX_DPS) { f->count=0U; return; }
        if (f->count && (dot3(unit(f->acc),unit(f->calibration_acc))<0.9993908f ||
            fabsf(f->acc_norm-norm(f->calibration_acc))>40.0f ||
            fmaxf(f->high.x,rate.x)-fminf(f->low.x,rate.x)>P2_GYRO_CAL_SPREAD_DPS ||
            fmaxf(f->high.y,rate.y)-fminf(f->low.y,rate.y)>P2_GYRO_CAL_SPREAD_DPS ||
            fmaxf(f->high.z,rate.z)-fminf(f->low.z,rate.z)>P2_GYRO_CAL_SPREAD_DPS)) f->count=0U;
        if (!f->count) {
            f->start_ms=now; f->sum=(P2_Vector){0,0,0};
            f->low=f->high=rate; f->calibration_acc=f->acc;
        }
        f->sum.x+=rate.x; f->sum.y+=rate.y; f->sum.z+=rate.z;
        f->low=(P2_Vector){fminf(f->low.x,rate.x),fminf(f->low.y,rate.y),fminf(f->low.z,rate.z)};
        f->high=(P2_Vector){fmaxf(f->high.x,rate.x),fmaxf(f->high.y,rate.y),fmaxf(f->high.z,rate.z)};
        ++f->count;
        if (now-f->start_ms<P2_GYRO_CAL_MS || f->count<P2_GYRO_CAL_MIN_SAMPLES) return;
        f->bias=times(f->sum,1.0f/f->count);
        f->rest_norm=f->acc_norm;
        f->up=unit(f->acc);
        f->calibrated=true;
    }
    rate=(P2_Vector){rate.x-f->bias.x,rate.y-f->bias.y,rate.z-f->bias.z};
    f->residual_rate=rate;
    f->rate=norm(rate);
    /* Gyro is faster than the magnetometer: an intervening motion sample
     * must interrupt the quiet window even if the next magnetic call is calm. */
    if (!f->ready && f->rate>(f->field_norm<=0.0f ? P2_STARTUP_RATE_MAX_DPS :
                              P2_HEADING_RECOVERY_RATE_DPS)) reset_startup(f);
    if (!f->ready || gap) return;
    if (f->rate>0.000001f) {
        /* A world-fixed vector expressed in the moving body follows
         * dv/dt = v x omega, hence the NEGATIVE body rotation. */
        P2_Vector axis=times(rate,1.0f/f->rate);
        float angle=-f->rate*elapsed*0.00001745329252f;
        f->up=rotate(f->up,axis,angle);
        f->north=rotate(f->north,axis,angle);
        orthogonalize(f);
    }
}

/* Establish the FIRST reference from the same quiet window for magnetic field,
 * gravity and gyro bias. Block means detect warm-up convergence without treating
 * ordinary magnetometer sample noise as rotation. Never run after acquiring zero. */
static bool startup_reference(P2_Attitude *f, P2_Vector m, float mn, uint32_t now) {
    P2_HeadingStartup *q=&f->startup;
    if (q->samples && now==q->last_ms) return false;
    P2_Vector up=unit(f->acc);
    float d=dot3(m,up);
    P2_Vector h={m.x-d*up.x,m.y-d*up.y,m.z-d*up.z};
    if (f->rate>P2_STARTUP_RATE_MAX_DPS ||
        fabsf(f->acc_norm-f->rest_norm)>P2_STARTUP_ACC_NORM_TOL_MG ||
        !isfinite(mn) || mn<P2_MAG_MIN_MGAUSS || mn>P2_MAG_MAX_MGAUSS ||
        norm(h)<P2_HORIZONTAL_MIN_MGAUSS || fabsf(up.x)>P2_FORWARD_VERTICAL_LIMIT) {
        reset_startup(f); return false;
    }
    float angle=vector_heading(h,up);
    f->magnetic_heading_deg=angle;
    if (q->samples && (now-q->last_ms>P2_MAG_FRESH_MS ||
        dot3(up,q->anchor_up)<P2_STARTUP_UP_COS_MIN ||
        fabsf(remainderf(angle-q->anchor_angle,360.0f))>P2_STARTUP_MAG_MAX_DEG ||
        fmaxf(q->field_max,mn)-fminf(q->field_min,mn)>P2_STARTUP_FIELD_SPAN_RATIO*mn))
        reset_startup(f);
    if (!q->samples) {
        q->start_ms=q->block_ms=now;
        q->anchor_up=up; q->anchor_angle=angle;
        q->field_min=q->field_max=mn;
    }
    q->last_ms=now; q->elapsed_ms=now-q->start_ms;
    q->field_min=fminf(q->field_min,mn); q->field_max=fmaxf(q->field_max,mn);
    q->mag_sum.x+=m.x; q->mag_sum.y+=m.y; q->mag_sum.z+=m.z;
    q->acc_sum.x+=f->acc.x; q->acc_sum.y+=f->acc.y; q->acc_sum.z+=f->acc.z;
    q->gyro_sum.x+=f->raw_rate.x; q->gyro_sum.y+=f->raw_rate.y; q->gyro_sum.z+=f->raw_rate.z;
    ++q->samples; ++q->block_samples;
    float radians=remainderf(angle-q->anchor_angle,360.0f)*.01745329252f;
    q->block_sin+=sinf(radians); q->block_cos+=cosf(radians);
    if (now-q->block_ms<P2_STARTUP_BLOCK_MS) return false;
    float mean=atan2f(q->block_sin,q->block_cos)*57.295779513f;
    if (!q->blocks) q->mean_min=q->mean_max=mean;
    q->mean_min=fminf(q->mean_min,mean); q->mean_max=fmaxf(q->mean_max,mean);
    q->mean_span=q->mean_max-q->mean_min;
    ++q->blocks;
    q->block_ms=now; q->block_samples=0U; q->block_sin=q->block_cos=0.0f;
    if (q->mean_span>P2_STARTUP_MEAN_SPAN_DEG) { reset_startup(f); return false; }
    if (q->blocks<P2_STARTUP_BLOCKS || q->samples<P2_STARTUP_MIN_SAMPLES) return false;
    float k=1.0f/q->samples;
    P2_Vector averaged=times(q->mag_sum,k), gravity=times(q->acc_sum,k);
    f->up=unit(gravity); f->rest_norm=norm(gravity);
    f->field_norm=norm(averaged);
    d=dot3(averaged,f->up);
    f->dip=asinf(clamp1(d/f->field_norm));
    f->north=unit((P2_Vector){averaged.x-d*f->up.x,averaged.y-d*f->up.y,averaged.z-d*f->up.z});
    f->bias=times(q->gyro_sum,k);
    f->residual_rate=(P2_Vector){f->raw_rate.x-f->bias.x,f->raw_rate.y-f->bias.y,f->raw_rate.z-f->bias.z};
    f->rate=norm(f->residual_rate);
    f->trusted_ms=now; f->ready=true; f->recovering=false;
    f->mag_guard.verified_ms=now; /* The complete boot window is trusted. */
    return true;
}

static void correct_tilt(P2_Motion *s, P2_Vector a, float magnitude, float dt) {
    P2_Attitude *f=&s->attitude;
    if (!f->ready) return;
    P2_Vector observed=times(a,1.0f/magnitude);
    float cosine=dot3(f->up,observed);
    /* Linear acceleration is NOT gravity. Reject large innovations/norm
     * changes; let gyro carry tilt instead of chasing each swing impulse. */
    if (fabsf(magnitude-f->rest_norm)>P2_FUSION_ACC_NORM_TOL_MG ||
        cosine<P2_FUSION_ACC_COS_MIN) return;
    P2_Vector axis=cross3(f->up,observed);
    float sn=norm(axis);
    if (sn>0.000001f) {
        axis=times(axis,1.0f/sn);
        float angle=atan2f(sn,cosine)*dt/(P2_FUSION_ACC_FILTER_S+dt);
        f->up=rotate(f->up,axis,angle);
        f->north=rotate(f->north,axis,angle);
        orthogonalize(f);
    }
}

static void guard_interrupt(P2_MagneticGuard *g) {
    g->samples=0U; g->sin_sum=g->cos_sum=0.0f;
}
static void guard_reject(P2_MagneticGuard *g) {
    guard_interrupt(g);
    g->blocked=true;
}
static bool magnetic_consistent(P2_MagneticGuard *g, float innovation, uint32_t now) {
    if (g->seen && now==g->last_ms)
        return !g->blocked && now-g->verified_ms<=P2_MAG_GUARD_BLOCK_MS+P2_MAG_FRESH_MS;
    if (g->samples && now-g->last_ms>P2_MAG_FRESH_MS) guard_interrupt(g);
    if (!g->samples) g->start_ms=now;
    g->last_ms=now;
    g->seen=true;
    /* Compare with the running attitude, not an indefinitely integrated boot
     * witness. Residual gyro bias must remain correctable throughout a run. */
    float residual=remainderf(innovation,360.0f)*.01745329252f;
    g->sin_sum+=sinf(residual); g->cos_sum+=cosf(residual); ++g->samples;
    if (now-g->start_ms>=P2_MAG_GUARD_BLOCK_MS && g->samples>=P2_MAG_GUARD_MIN_SAMPLES) {
        g->mean_deg=atan2f(g->sin_sum,g->cos_sum)*57.295779513f;
        float length=sqrtf(g->sin_sum*g->sin_sum+g->cos_sum*g->cos_sum)/g->samples;
        if (length<0.95f || fabsf(g->mean_deg)>P2_MAG_GUARD_ENTRY_DEG) g->blocked=true;
        else if (fabsf(g->mean_deg)<P2_MAG_GUARD_EXIT_DEG) g->blocked=false;
        if (!g->blocked) g->verified_ms=now;
        guard_interrupt(g);
    }
    /* Fragmented windows must not grant unlimited trust. A full clean block,
     * not occasional individually plausible samples, renews this evidence. */
    return !g->blocked && now-g->verified_ms<=P2_MAG_GUARD_BLOCK_MS+P2_MAG_FRESH_MS;
}

/* A lost interval cannot be integrated retroactively. Reacquire a stable
 * magnetic endpoint automatically, preserving the user's original boot zero.
 * This does not recover full revolutions made while sensor data were missing. */
static bool recover_reference(P2_Attitude *f, P2_Vector m, float mn, uint32_t now) {
    P2_HeadingStartup *q=&f->recovery;
    if (q->samples && now==q->last_ms) return false;
    P2_Vector up=unit(f->acc);
    float d=dot3(m,up);
    P2_Vector h={m.x-d*up.x,m.y-d*up.y,m.z-d*up.z};
    if (f->rate>P2_HEADING_RECOVERY_RATE_DPS ||
        fabsf(f->acc_norm-f->rest_norm)>P2_FUSION_ACC_NORM_TOL_MG ||
        !isfinite(mn) || mn<P2_MAG_MIN_MGAUSS || mn>P2_MAG_MAX_MGAUSS ||
        norm(h)<P2_HORIZONTAL_MIN_MGAUSS || fabsf(up.x)>P2_FORWARD_VERTICAL_LIMIT ||
        fabsf(mn-f->field_norm)>P2_HEADING_RECOVERY_STRENGTH_RATIO*f->field_norm ||
        fabsf(asinf(clamp1(d/mn))-f->dip)*57.295779513f>P2_HEADING_RECOVERY_DIP_DEG) {
        memset(q,0,sizeof(*q)); return false;
    }
    float angle=vector_heading(h,up);
    if (q->samples && (now-q->last_ms>P2_MAG_FRESH_MS ||
        dot3(up,q->anchor_up)<P2_STARTUP_UP_COS_MIN ||
        fabsf(remainderf(angle-q->anchor_angle,360.0f))>P2_STARTUP_MAG_MAX_DEG ||
        fmaxf(q->field_max,mn)-fminf(q->field_min,mn)>P2_STARTUP_FIELD_SPAN_RATIO*mn))
        memset(q,0,sizeof(*q));
    if (!q->samples) {
        q->start_ms=now; q->anchor_up=up; q->anchor_angle=angle;
        q->field_min=q->field_max=mn;
    }
    q->last_ms=now; q->elapsed_ms=now-q->start_ms;
    q->field_min=fminf(q->field_min,mn); q->field_max=fmaxf(q->field_max,mn);
    q->mag_sum.x+=m.x; q->mag_sum.y+=m.y; q->mag_sum.z+=m.z;
    q->acc_sum.x+=f->acc.x; q->acc_sum.y+=f->acc.y; q->acc_sum.z+=f->acc.z;
    ++q->samples;
    if (q->elapsed_ms<P2_HEADING_RECOVERY_MS || q->samples<P2_HEADING_RECOVERY_MIN_SAMPLES)
        return false;
    P2_Vector averaged=times(q->mag_sum,1.0f/q->samples);
    f->up=unit(q->acc_sum);
    d=dot3(averaged,f->up);
    f->north=unit((P2_Vector){averaged.x-d*f->up.x,averaged.y-d*f->up.y,averaged.z-d*f->up.z});
    f->ready=true; f->recovering=false; f->trusted_ms=now;
    memset(&f->mag_guard,0,sizeof(f->mag_guard));
    f->mag_guard.verified_ms=now;
    return true;
}

/* Magnetic yaw correction is bounded and cannot overwrite inertial tilt.
 * A rejected field disables magnetic aiding, not the live gyro estimate.
 * True inertial loss requires fresh, stable reacquisition; never freeze data. */
static bool fused_heading(P2_Motion *s, P2_Vector m, float *degrees) {
    P2_Attitude *f=&s->attitude;
    uint32_t now=f->last_ms;
    f->magnetic_accepted=false;
    f->correction_dps=0.0f;
    if (!f->ready) s->heading_status=f->field_norm<=0.0f ?
        P2_HEADING_CALIBRATING : P2_HEADING_RECOVERING;
    int32_t skew=(int32_t)(s->last_ms-now);
    if (!f->seen || !f->calibrated || skew>(int32_t)P2_GYRO_FRESH_MS ||
        skew<-(int32_t)P2_GYRO_FRESH_MS) { reset_startup(f); return false; }
    float mn=norm(m);
    bool just_seeded=false;
    if (!f->ready && f->field_norm<=0.0f) {
        if (!startup_reference(f,m,mn,now)) return false;
        just_seeded=true;
    }
    if (!f->ready) {
        f->recovering=true;
        if (!recover_reference(f,m,mn,now)) return false;
        just_seeded=true;
    }
    float d=dot3(m,f->up);
    P2_Vector horizontal={m.x-d*f->up.x,m.y-d*f->up.y,m.z-d*f->up.z};
    float hn=norm(horizontal);
    bool plausible=isfinite(mn) && mn>=P2_MAG_MIN_MGAUSS && mn<=P2_MAG_MAX_MGAUSS &&
        hn>=P2_HORIZONTAL_MIN_MGAUSS &&
        fabsf(mn-f->field_norm)<=P2_FUSION_MAG_STRENGTH_RATIO*f->field_norm &&
        fabsf(asinf(clamp1(d/mn))-f->dip)*57.295779513f<=P2_FUSION_MAG_DIP_MAX_DEG;
    if (plausible) {
        P2_Vector target=times(horizontal,1.0f/hn);
        f->magnetic_heading_deg=vector_heading(target,f->up);
        float innovation=atan2f(dot3(f->up,cross3(f->north,target)),dot3(f->north,target));
        f->innovation_deg=innovation*57.295779513f;
        plausible=fabsf(innovation)*57.295779513f<=P2_FUSION_MAG_INNOVATION_DEG;
        if (plausible)
            plausible=magnetic_consistent(&f->mag_guard,f->innovation_deg,now);
        else guard_reject(&f->mag_guard);
        /* Gyro-transported tilt remains available during linear acceleration.
         * Its correction veto must not erase valid magnetic-consistency evidence. */
        plausible=plausible && fabsf(f->acc_norm-f->rest_norm)<=P2_FUSION_ACC_NORM_TOL_MG;
        if (plausible) {
            /* Timestamp freshness, not call count. First seed needs no correction. */
            uint32_t ms=now-s->heading_last_ms;
            float dt=just_seeded || ms>P2_SAMPLE_GAP_MS ? 0.0f : ms*.001f;
            float limit=P2_FUSION_MAG_CORRECTION_DPS*.01745329252f*dt;
            float correction=fmaxf(-limit,fminf(limit,innovation*dt/(P2_FUSION_MAG_FILTER_S+dt)));
            f->mag_guard.correction_deg=remainderf(f->mag_guard.correction_deg+
                                                   correction*57.295779513f,360.0f);
            if (dt>0.0f) f->correction_dps=correction*57.295779513f/dt;
            f->north=rotate(f->north,f->up,correction);
            orthogonalize(f);
            f->trusted_ms=now; f->magnetic_accepted=true;
        }
    } else guard_reject(&f->mag_guard);
    if (fabsf(f->up.x)>P2_FORWARD_VERTICAL_LIMIT) return false;
    *degrees=vector_heading(f->north,f->up);
    s->gravity=times(f->up,f->rest_norm);
    return true;
}

/* Discard unconfirmed motion, but preserve already published steps. */
static void reset_bout(P2_Motion *s) {
    s->pending_steps = 0U;
    s->pending_distance_m = 0.0f;
    s->candidate_interval_ms = 0U;
    s->have_peak = false;
    s->walking = s->weak_bout = false;
    s->armed = false;
    s->step_trough = 0.0f;
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
    bool weak = s->previous < P2_STEP_HIGH_MG;
    float cadence_ratio = (weak || s->weak_bout) ? P2_STEP_WEAK_CADENCE_RATIO : P2_STEP_CADENCE_RATIO;
    /* Cadence consistency confirms startup only. During established walking,
     * hand motion can produce unequal intervals without ending the bout.
     * Stop/gap handling and amplitude/refractory checks remain in Acceleration. */
    if (!s->have_peak) {
        s->weak_bout = weak;
        s->pending_steps = 1U;
        s->pending_distance_m = 0.0f; /* First interval not known yet. */
    } else if (s->walking) {
        /* Allow cadence changes once walking is established. */
        add_steps(s, 1U, step_length(s, interval));
    } else {
        /* Two adjacent intervals must agree before the first three candidates
         * become steps. Irregular handling starts a fresh confirmation window. */
        bool consistent = s->candidate_interval_ms == 0U ||
            ((float)interval <= s->candidate_interval_ms * cadence_ratio &&
             (float)s->candidate_interval_ms <= interval * cadence_ratio);
        float length = step_length(s, interval);
        /* Infer the first step from the first observed interval. After an
         * inconsistent cadence, retain only the latest pair and its distance. */
        if (!consistent || s->pending_steps == 1U) s->pending_distance_m = 2.0f*length;
        else s->pending_distance_m += length;
        s->pending_steps = consistent ? s->pending_steps + 1U : 2U;
        s->weak_bout = consistent ? (s->weak_bout || weak) : weak;
        if (s->pending_steps >= (s->weak_bout ? P2_STEP_WEAK_CONFIRM_COUNT : P2_STEP_CONFIRM_COUNT)) {
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
/* Detect a large direction sweep followed by a real reversal. Mere tilt or
 * continuous rotation does not veto magnitude-based steps. Normalizing before
 * filtering avoids conflating ordinary vertical walking acceleration with
 * orientation change. This is handling rejection, not an activity classifier. */
static bool step_handling(P2_Motion *s, P2_Vector a, float magnitude,
                          float dt, uint32_t now) {
    float alpha=dt/(P2_STEP_DIRECTION_FILTER_S+dt);
    P2_Vector previous=s->step_direction;
    P2_Vector direction={previous.x+alpha*(a.x/magnitude-previous.x),
                         previous.y+alpha*(a.y/magnitude-previous.y),
                         previous.z+alpha*(a.z/magnitude-previous.z)};
    float length=norm(direction);
    if (length < .001f) {
        s->step_rotation_axis_set=false;
        s->step_direction=(P2_Vector){a.x/magnitude,a.y/magnitude,a.z/magnitude};
    } else {
        direction.x/=length; direction.y/=length; direction.z/=length;
        s->step_direction=direction;
        P2_Vector turn={previous.y*direction.z-previous.z*direction.y,
                        previous.z*direction.x-previous.x*direction.z,
                        previous.x*direction.y-previous.y*direction.x};
        float sine=norm(turn);
        float cosine=previous.x*direction.x+previous.y*direction.y+previous.z*direction.z;
        float angle=atan2f(sine,cosine);
        if (angle >= P2_STEP_ROTATION_MIN_RAD && sine > .0001f) {
            turn.x/=sine; turn.y/=sine; turn.z/=sine;
            float aligned=turn.x*s->step_rotation_axis.x+
                          turn.y*s->step_rotation_axis.y+turn.z*s->step_rotation_axis.z;
            if (!s->step_rotation_axis_set || fabsf(aligned)<P2_STEP_ROTATION_AXIS_COS) {
                s->step_rotation_axis=turn;
                s->step_rotation_axis_set=true;
                s->step_rotation_forward=angle;
                s->step_rotation_reverse=0.0f;
            } else if (aligned >= 0.0f) {
                s->step_rotation_forward=fminf(6.283185307f,s->step_rotation_forward+angle);
                s->step_rotation_reverse=0.0f;
            } else {
                s->step_rotation_reverse+=angle;
                if (s->step_rotation_reverse >= P2_STEP_ROTATION_REVERSE_RAD) {
                    if (s->step_rotation_forward >= P2_STEP_ROTATION_SWEEP_RAD) {
                        if (s->step_rotation_reversal_seen &&
                            now-s->step_rotation_reversal_ms<=P2_STEP_ROTATION_REPEAT_MS) {
                            if (!s->step_handling) {
                                s->step_quiet_direction=direction;
                                s->step_quiet_ms=now;
                            }
                            s->step_handling=true;
                            s->step_handling_ms=now;
                        }
                        s->step_rotation_reversal_ms=now;
                        s->step_rotation_reversal_seen=true;
                    }
                    s->step_rotation_axis.x=-s->step_rotation_axis.x;
                    s->step_rotation_axis.y=-s->step_rotation_axis.y;
                    s->step_rotation_axis.z=-s->step_rotation_axis.z;
                    s->step_rotation_forward=s->step_rotation_reverse;
                    s->step_rotation_reverse=0.0f;
                }
            }
        }
    }
    if (s->step_handling) {
        /* Missing a reversal in a compound 3D swing is not proof it stopped.
         * Release only after direction stays in a small cone. This still
         * permits modest walking sway; no perfectly motionless wait is needed. */
        P2_Vector q=s->step_quiet_direction, d=s->step_direction;
        if (q.x*d.x+q.y*d.y+q.z*d.z<P2_STEP_HANDLING_QUIET_COS) {
            s->step_quiet_direction=d;
            s->step_quiet_ms=now;
        }
        if (now-s->step_handling_ms>=P2_STEP_HANDLING_HOLD_MS &&
            now-s->step_quiet_ms>=P2_STEP_HANDLING_QUIET_MS)
            s->step_handling=false;
    }
    return s->step_handling;
}

void P2_MotionAcceleration(P2_Motion *s, P2_Vector a, uint32_t now) {
    float magnitude = norm(a);
    s->attitude.acc=a;
    s->attitude.acc_norm=magnitude;
    if (!isfinite(magnitude) || magnitude < P2_ACC_MIN_MG || magnitude > P2_ACC_MAX_MG) {
        if (s->heading_origin_set) s->attitude.mag_guard.reference_lost=true;
        reset_bout(s);
        s->heading_valid = false;
        s->heading_status = P2_HEADING_UNAVAILABLE;
        /* Discard the old gravity estimate after a drop/impact/invalid sample.
         * A new magnetic sample must not make that old estimate valid again. */
        s->initialized = false;
        s->attitude.ready=s->attitude.recovering=false;
        reset_startup(&s->attitude);
        return;
    }
    uint32_t elapsed = now - s->last_ms; /* unsigned arithmetic handles tick wrap */
    if (!s->attitude.ready && s->attitude.field_norm<=0.0f && s->attitude.startup.samples &&
        (fabsf(magnitude-s->attitude.rest_norm)>P2_STARTUP_ACC_NORM_TOL_MG ||
         dot3(times(a,1.0f/magnitude),s->attitude.startup.anchor_up)<P2_STARTUP_UP_COS_MIN))
        reset_startup(&s->attitude);
    if (!s->attitude.ready && s->attitude.field_norm>0.0f && s->attitude.recovery.samples &&
        (fabsf(magnitude-s->attitude.rest_norm)>P2_FUSION_ACC_NORM_TOL_MG ||
         dot3(times(a,1.0f/magnitude),s->attitude.recovery.anchor_up)<P2_STARTUP_UP_COS_MIN))
        reset_startup(&s->attitude);
    if (!s->initialized || elapsed > P2_SAMPLE_GAP_MS) {
        if (s->heading_origin_set) s->attitude.mag_guard.reference_lost=true;
        s->attitude.ready=s->attitude.recovering=false;
        reset_startup(&s->attitude);
        s->magnetic_reference = 0.0f;
        s->magnetic_count = 0U;
        s->magnetic_seen = s->magnetic_blocked = s->recovery_active = false;
        s->gravity = a;
        s->baseline = magnitude;
        s->filtered = s->previous = s->step_envelope = 0.0f;
        s->step_direction=(P2_Vector){a.x/magnitude,a.y/magnitude,a.z/magnitude};
        s->step_rotation_axis_set=s->step_rotation_reversal_seen=s->step_handling=false;
        s->step_rotation_forward=s->step_rotation_reverse=0.0f;
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
    /* Heading needs current tilt, not the slow magnitude baseline used by
     * the pedometer. These vector updates never feed step detection. */
    float tilt = dt / (P2_TILT_FILTER_S + dt);
    if (s->attitude.required && s->attitude.ready) {
        correct_tilt(s,a,magnitude,dt);
        s->gravity=times(s->attitude.up,s->attitude.rest_norm);
    } else {
        s->gravity.x += tilt * (a.x - s->gravity.x);
        s->gravity.y += tilt * (a.y - s->gravity.y);
        s->gravity.z += tilt * (a.z - s->gravity.z);
    }
    s->baseline += slow * (magnitude - s->baseline);
    float fast = dt / (P2_MOTION_FILTER_S + dt);
    s->previous = s->filtered;
    s->filtered += fast * (magnitude - s->baseline - s->filtered);
    if (step_handling(s,a,magnitude,dt,now)) {
        reset_bout(s);
        s->step_envelope=0.0f;
        return;
    }
    if (now - s->start_ms < P2_SETTLE_MS) return;
    float envelope_decay = dt/(P2_STEP_ENVELOPE_S+dt);
    s->step_envelope = fmaxf(fabsf(s->filtered),
                            s->step_envelope*(1.0f-envelope_decay));
    float high = fmaxf(P2_STEP_PEAK_FLOOR_MG,
                      fminf(P2_STEP_HIGH_MG,0.45f*s->step_envelope));
    float low = -fmaxf(P2_STEP_TROUGH_FLOOR_MG,
                      fminf(-P2_STEP_LOW_MG,0.20f*s->step_envelope));
    /* Require a trough followed by a positive falling peak, plus refractory time.
     * Magnitude makes counting independent of the fixed mounting orientation. */
    if (s->filtered < low) {
        if (!s->armed || s->filtered < s->step_trough) s->step_trough = s->filtered;
        s->armed = true;
        s->trough_ms = now;
    }
    /* Do not pair an isolated trough with an unrelated motion much later. */
    if (s->armed && now - s->trough_ms > P2_TROUGH_TIMEOUT_MS) s->armed = false;
    if (s->armed && s->previous > high && s->filtered < s->previous &&
        s->previous-s->step_trough >= P2_STEP_PROMINENCE_MG &&
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
    float heading;
    if (s->attitude.required) {
        if (!fused_heading(s,m,&heading)) return false;
        goto filter_heading;
    }
    float mn = norm(m), gn = norm(s->gravity);
    if (!isfinite(mn) || !isfinite(gn) || mn < P2_MAG_MIN_MGAUSS || mn > P2_MAG_MAX_MGAUSS ||
        gn < P2_GRAVITY_MIN_MG || gn > P2_GRAVITY_MAX_MG) return false;
    P2_Vector up = {s->gravity.x/gn, s->gravity.y/gn, s->gravity.z/gn};
    /* Project body +X and magnetic field onto the horizontal plane. */
    float dot = m.x*up.x + m.y*up.y + m.z*up.z;
    P2_Vector north = {m.x-dot*up.x, m.y-dot*up.y, m.z-dot*up.z};
    float horizontal = norm(north);
    if (horizontal < P2_HORIZONTAL_MIN_MGAUSS || fabsf(up.x) > P2_FORWARD_VERTICAL_LIMIT) return false;
    /* Provisional XY correction cannot establish rotation-invariant strength.
     * Only use reference/recovery rejection with validated full calibration.
     * Raw sanity, gravity and projection checks above always remain active. */
    if (P2_MAG_CALIBRATED) {
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
    }
    /* Signed horizontal component in the register-axis basis. The explicit
     * board yaw convention is verified separately from this projection. */
    float east_x = north.y*up.z - north.z*up.y;
    heading = wrap(P2_HEADING_SIGN * atan2f(east_x, north.x)*57.295779513f + P2_HEADING_OFFSET_DEG);
filter_heading:;
    /* Average unit vectors, not degree values: 359/1 degrees must average at
     * north, never south. Invalid input above is immediately invalid; after
     * invalidity/gaps seed from the new observation, not an old output. */
    float radians = heading * 0.01745329252f;
    uint32_t elapsed = s->last_ms - s->heading_last_ms;
    float sn = sinf(radians), cs = cosf(radians);
    if (s->attitude.required || !was_valid || elapsed > P2_SAMPLE_GAP_MS) {
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
    float absolute_heading = wrap(atan2f(s->heading_sin, s->heading_cos)*57.295779513f);
    /* Only the first valid, settled observation sets zero. Recovery must not
     * silently redefine the user's original startup direction. */
    if (!s->heading_origin_set) {
        s->heading_origin_deg = absolute_heading;
        s->heading_origin_set = true;
        s->heading_previous_deg = absolute_heading;
        s->heading_deg = 0.0f;
        s->heading_turns_deg = 0.0f;
    }
    /* Unwrap successive angles. Sampling must observe less than 180 degrees
     * per interval. After missing data only the shortest endpoint change is
     * recoverable; complete turns during the outage cannot be reconstructed. */
    float delta = wrap(absolute_heading - s->heading_previous_deg + 180.0f) - 180.0f;
    /* Keep only the current signed cycle, not lifetime winding. Otherwise
     * -363 -> -357 would display -3 -> -357 instead of -3 -> +3. */
    s->heading_turns_deg = fmodf(s->heading_turns_deg + delta, 360.0f);
    float relative = wrap(absolute_heading - s->heading_origin_deg + 180.0f) - 180.0f;
    float previous_relative = wrap(s->heading_previous_deg - s->heading_origin_deg + 180.0f) - 180.0f;
    /* Detect actual reference crossings, even when no sample equals zero.
     * Restrict to the zero half-plane so crossing 180 cannot reset the cycle. */
    if ((fabsf(relative) < 90.0f && fabsf(previous_relative) < 90.0f &&
         ((relative >= 0.0f && previous_relative <= 0.0f) ||
          (relative <= 0.0f && previous_relative >= 0.0f))) ||
        fabsf(relative) < 0.05f)
        s->heading_turns_deg = relative;
    s->heading_deg = s->heading_turns_deg;
    /* Match 0.1-degree telemetry resolution at the full-turn boundary.
     * This is display rounding, not a claim that physical error is zero. */
    if (fabsf(s->heading_deg) >= 359.95f || fabsf(s->heading_deg) < 0.05f)
        s->heading_deg = 0.0f;
    s->heading_previous_deg = absolute_heading;
    s->heading_valid = true;
    return true;
}
