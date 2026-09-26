/* Exercise motion waveforms through the public API, rather than manipulating
 * the candidate state. These synthetic cases do not establish human accuracy. */
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
    /* Begin with a trough, then produce exactly one positive peak. */
    for (unsigned t = 0; t < period; t += 20U)
        sample(s, 1000.0f - 300.0f * sinf(6.283185307f * t / period), axis);
}
static void gait(P2_Motion *s, unsigned cycles, unsigned period, unsigned axis) {
    for (unsigned i = 0; i < cycles; ++i) cycle(s, period, axis);
}
int main(void) {
    P2_Motion s;
    /* Startup confirmation backfills real candidate steps, without double count. */
    init(&s, 0U, 2U);
    cycle(&s, 600U, 2U); assert(s.steps == 0U);
    cycle(&s, 600U, 2U); assert(s.steps == 0U);
    cycle(&s, 600U, 2U); assert(s.steps == 3U);
    cycle(&s, 600U, 2U); assert(s.steps == 4U);
    rest(&s, 2500U, 2U);
    cycle(&s, 600U, 2U); assert(s.steps == 4U);
    cycle(&s, 600U, 2U); assert(s.steps == 4U);
    cycle(&s, 600U, 2U); assert(s.steps == 7U);

    /* Separated handling pulses and short pairs must not accumulate over time. */
    init(&s, 0U, 2U);
    for (unsigned i = 0; i < 6U; ++i) {
        gait(&s, i % 2U + 1U, 600U, 2U);
        rest(&s, 2500U, 2U);
        assert(s.steps == 0U);
    }
    /* Uneven cadence cannot confirm a bout even inside the absolute time limit. */
    init(&s, 0U, 2U);
    cycle(&s, 600U, 2U);
    cycle(&s, 600U, 2U);
    rest(&s, 750U, 2U);
    cycle(&s, 600U, 2U);
    assert(s.steps == 0U);

    /* Sampling gaps and invalid acceleration must discard pending confirmation. */
    init(&s, 0U, 2U); gait(&s, 2U, 600U, 2U);
    now += 500U; rest(&s, 1500U, 2U);
    cycle(&s, 600U, 2U); assert(s.steps == 0U);
    init(&s, 0U, 2U); gait(&s, 2U, 600U, 2U);
    sample(&s, NAN, 2U); rest(&s, 1500U, 2U);
    cycle(&s, 600U, 2U); assert(s.steps == 0U);

    /* 48, 100 and 150 steps/min across different mount axes and tick wrap. */
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
    /* Established walking accommodates gradual speed changes. */
    init(&s, 0U, 2U);
    gait(&s, 10U, 1000U, 2U); gait(&s, 10U, 800U, 2U);
    gait(&s, 10U, 600U, 2U); gait(&s, 10U, 400U, 2U);
    assert(s.steps == 40U);

    /* An ideal slow rotation has constant gravity magnitude and no steps. */
    init(&s, 0U, 2U);
    for (unsigned t = 0; t < 30000U; t += 20U) {
        float angle = 6.283185307f * t / 4000.0f;
        P2_MotionAcceleration(&s, (P2_Vector){1000*sinf(angle), 0, 1000*cosf(angle)}, now);
        now += 20U;
    }
    assert(s.steps == 0U);

    /* Backfill must saturate safely at the uint32 count limit. */
    init(&s, 0U, 2U); s.steps = UINT32_MAX - 1U;
    gait(&s, 4U, 600U, 2U); assert(s.steps == UINT32_MAX);
    puts("PASS: confirmation/backfill, isolated motion, pause/restart, cadence, gaps, invalid samples, rotation, saturation.");
    return 0;
}
