

#include "p2_motion.h"
#include "p2_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t now;
static void sample(P2_Motion *s, float magnitude, unsigned axis) {
    P2_Vector v = {0, 0, 0};
    if (axis == 0U) v.x = magnitude;
    else if (axis == 1U) v.y = -magnitude;
    else v.z = magnitude;
    P2_MotionAcceleration(s, v, now);
    now += 20U;
}
static void rest(P2_Motion *s, unsigned ms, unsigned axis) {
    for (unsigned t = 0; t < ms; t += 20U) sample(s, 1000.0f, axis);
}
static void init(P2_Motion *s, uint32_t origin, unsigned axis) {
    now = origin;
    P2_MotionInit(s);
    rest(s, 2000U, axis);
}
static void cycle(P2_Motion *s, unsigned period, unsigned axis) {

    for (unsigned t = 0; t < period; t += 20U)
        sample(s, 1000.0f - 300.0f * sinf(6.283185307f * t / period), axis);
}
static void gait(P2_Motion *s, unsigned cycles, unsigned period, unsigned axis) {
    for (unsigned i = 0; i < cycles; ++i) cycle(s, period, axis);
}
int main(void) {
    P2_Motion s;

    init(&s, 0U, 2U);
    cycle(&s, 600U, 2U); assert(s.steps == 0U);
    cycle(&s, 600U, 2U); assert(s.steps == 0U);
    cycle(&s, 600U, 2U); assert(s.steps == 3U);
    cycle(&s, 600U, 2U); assert(s.steps == 4U);
    rest(&s, 2500U, 2U);
    cycle(&s, 600U, 2U); assert(s.steps == 4U);
    cycle(&s, 600U, 2U); assert(s.steps == 4U);
    cycle(&s, 600U, 2U); assert(s.steps == 7U);

    init(&s, 0U, 2U);
    for (unsigned i = 0; i < 6U; ++i) {
        gait(&s, i % 2U + 1U, 600U, 2U);
        rest(&s, 2500U, 2U);
        assert(s.steps == 0U);
    }

    init(&s, 0U, 2U);
    cycle(&s, 600U, 2U);
    cycle(&s, 600U, 2U);
    rest(&s, 750U, 2U);
    cycle(&s, 600U, 2U);
    assert(s.steps == 0U);

    init(&s, 0U, 2U); gait(&s, 2U, 600U, 2U);
    now += 500U; rest(&s, 1500U, 2U);
    cycle(&s, 600U, 2U); assert(s.steps == 0U);
    init(&s, 0U, 2U); gait(&s, 2U, 600U, 2U);
    sample(&s, NAN, 2U); rest(&s, 1500U, 2U);
    cycle(&s, 600U, 2U); assert(s.steps == 0U);

    const unsigned periods[] = {1240U, 600U, 400U};
    for (unsigned axis = 0; axis < 3U; ++axis) {
        for (unsigned j = 0; j < 3U; ++j) {
            init(&s, UINT32_MAX - 3000U, axis);
            gait(&s, 100U, periods[j], axis);
            printf("axis=%u period=%u ms: %lu / 100 synthetic steps\n",
                   axis, periods[j], (unsigned long)s.steps);
            assert(s.steps == 100U);
        }
    }

    init(&s, 0U, 2U);
    gait(&s, 10U, 1000U, 2U); gait(&s, 10U, 800U, 2U);
    gait(&s, 10U, 600U, 2U); gait(&s, 10U, 400U, 2U);
    assert(s.steps == 40U);

    init(&s, 0U, 2U);
    P2_StrideModel model = {.7f,.3f,.2f,.8f,2.5f,true};
    assert(P2_MotionSetStride(&s,model));
    gait(&s,10U,1000U,2U);
    assert(s.steps==10U && fabsf(s.distance_m-5.0f)<.05f);
    assert(!P2_MotionSetStride(&s,model));
    rest(&s,2500U,2U);
    float before=s.distance_m;
    cycle(&s,400U,2U); cycle(&s,400U,2U);
    assert(s.distance_m==before);
    gait(&s,8U,400U,2U);
    assert(s.steps==20U && fabsf((s.distance_m-before)-8.0f)<.08f);
    assert(fabsf(s.distance_m-16.0f)>2.0f);
    rest(&s,2500U,2U);
    model.slope_m_per_hz=NAN;
    assert(!P2_MotionSetStride(&s,model));
    init(&s,UINT32_MAX-3000U,2U);
    model=(P2_StrideModel){.7f,.3f,.2f,.8f,2.5f,true};
    assert(P2_MotionSetStride(&s,model));
    gait(&s,2U,600U,2U); rest(&s,2500U,2U);
    assert(s.distance_m==0.0f);
    gait(&s,10U,1000U,2U);
    assert(fabsf(s.distance_m-5.0f)<.05f);

    init(&s,0U,2U);
    model=(P2_StrideModel){.7f,.3f,.2f,1.2f,2.5f,true};
    assert(P2_MotionSetStride(&s,model));
    gait(&s,10U,1000U,2U);
    assert(s.steps==10U && fabsf(s.distance_m-7.0f)<.01f);

    init(&s, 0U, 2U);
    for (unsigned t = 0; t < 30000U; t += 20U) {
        float angle = 6.283185307f * t / 4000.0f;
        P2_MotionAcceleration(&s, (P2_Vector){1000*sinf(angle), 0, 1000*cosf(angle)}, now);
        now += 20U;
    }
    assert(s.steps == 0U);

    init(&s, 0U, 2U); s.steps = UINT32_MAX - 1U;
    gait(&s, 4U, 600U, 2U); assert(s.steps == UINT32_MAX);
    puts("PASS: confirmation/backfill, isolated motion, pause/restart, cadence, gaps, invalid samples, rotation, saturation.");
    return 0;
}
