/* Regression of direction convention, invalid handling and circular filtering.
 * Analytic signals are not a claim of real-world heading accuracy. */
#include "p2_motion.h"
#include "p2_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const P2_Vector level = {0,0,1000};
static float error(float a, float b) {
    float e = fmodf(a-b+540.0f,360.0f)-180.0f;
    return fabsf(e);
}
static void settle(P2_Motion *s) {
    P2_MotionInit(s);
    s->mag_bias=(P2_Vector){0,0,0};
    for (uint32_t t=0;t<=1200;t+=20) P2_MotionAcceleration(s,level,t);
    s->magnetic_reference=sqrtf(300*300+250*250); /* Geometry fixture. */
}
static P2_Vector observed(float clockwise) {
    /* Raw observed horizontal field rotates oppositely in the labelled CW
     * fixture; cardinal cases below explicitly exercise this board convention. */
    float r=clockwise*0.01745329252f;
    return (P2_Vector){300*cosf(r),-300*sinf(r),250};
}
/* Wait for the explicit startup/recovery window when testing geometry. */
static bool ready_heading(P2_Motion *s, P2_Vector m) {
    for (unsigned i=0;i<60;i++) {
        if (P2_MotionHeading(s,m)) return true;
        if (s->heading_status != P2_HEADING_CALIBRATING &&
            s->heading_status != P2_HEADING_RECOVERING) return false;
        P2_MotionAcceleration(s,s->gravity,s->last_ms+20U);
    }
    return false;
}
int main(void) {
    P2_Motion s;
    const P2_Vector cardinal[]={{300,0,250},{0,-300,250},{-300,0,250},{0,300,250}};
    for (unsigned i=0;i<4;i++) {
        settle(&s);
        assert(ready_heading(&s,cardinal[i]));
        assert(error(s.heading_deg,90.0f*i)<.01f);
    }
    /* Crossing north must not pass through south. */
    settle(&s); assert(ready_heading(&s,observed(359)));
    for(uint32_t t=1220;t<2220;t+=20) {
        P2_MotionAcceleration(&s,level,t);
        assert(ready_heading(&s,observed(1)));
        assert(error(s.heading_deg,0)<1.01f);
    }
    /* Unit-circle filtering rejects alternating jitter without freezing yaw. */
    settle(&s); assert(ready_heading(&s,observed(90)));
    for(uint32_t t=1220;t<3220;t+=20) {
        P2_MotionAcceleration(&s,level,t);
        assert(ready_heading(&s,observed((t/20)%2 ? 93 : 87)));
        if(t>2220) assert(error(s.heading_deg,90)<.3f);
    }
    for(uint32_t t=3220;t<4220;t+=20) {
        P2_MotionAcceleration(&s,level,t);
        assert(ready_heading(&s,observed(180)));
    }
    assert(error(s.heading_deg,180)<.3f);
    /* Never turn bad measurements into held-valid output; reacquire fresh. */
    assert(!P2_MotionHeading(&s,(P2_Vector){0,0,250}));
    assert(!s.heading_valid);
    P2_MotionAcceleration(&s,level,4220);
    assert(ready_heading(&s,observed(270)));
    assert(error(s.heading_deg,270)<.01f);
    assert(!P2_MotionHeading(&s,(P2_Vector){NAN,0,250}));
    assert(!P2_MotionHeading(&s,(P2_Vector){9000,0,250}));
    P2_MotionAcceleration(&s,level,8000);
    assert(!P2_MotionHeading(&s,observed(45)));
    for(uint32_t t=8020;t<=9020;t+=20) P2_MotionAcceleration(&s,level,t);
    assert(ready_heading(&s,observed(45)));
    assert(error(s.heading_deg,45)<.01f);
    /* Measured XY bias is subtracted before projection/filtering. */
    P2_MotionInit(&s);
    for(uint32_t t=0;t<=1200;t+=20) P2_MotionAcceleration(&s,level,t);
    P2_Vector with_bias={P2_MAG_BIAS_X, -300+P2_MAG_BIAS_Y, 250};
    assert(ready_heading(&s,with_bias));
    assert(error(s.heading_deg,90)<.01f);
    /* Plausible absolute strength may still be a local disturbance.
     * Sustained interference must not become the new reference. */
    settle(&s); assert(ready_heading(&s,observed(0)));
    for (uint32_t t=1220;t<3220;t+=20) {
        P2_MotionAcceleration(&s,level,t);
        assert(!P2_MotionHeading(&s,(P2_Vector){0,-600,500}));
        assert(!s.heading_valid);
    }
    assert(ready_heading(&s,observed(90)));
    assert(error(s.heading_deg,90)<.01f);
    /* Normal rotation with constant field strength remains accepted. */
    for (unsigned i=0;i<360;i++) {
        P2_MotionAcceleration(&s,level,s.last_ms+20U);
        assert(ready_heading(&s,observed((float)i)));
    }
    /* Reference requires continuous time, not repeated calls at one tick. */
    settle(&s); s.magnetic_reference=0;
    for (unsigned i=0;i<100;i++) {
        assert(!P2_MotionHeading(&s,observed(0)));
        assert(s.heading_status==P2_HEADING_CALIBRATING);
    }
    for (uint32_t t=1220;t<2200;t+=20) {
        P2_MotionAcceleration(&s,level,t);
        assert(!P2_MotionHeading(&s,observed(0)));
    }
    P2_MotionAcceleration(&s,level,2200);
    assert(P2_MotionHeading(&s,observed(0)));
    assert(!P2_MotionHeading(&s,(P2_Vector){0,-600,500}));
    assert(s.heading_status==P2_HEADING_DISTURBED);
    for (uint32_t t=2220;t<2720;t+=20) {
        P2_MotionAcceleration(&s,level,t);
        assert(!P2_MotionHeading(&s,observed(90)));
        assert(s.heading_status==P2_HEADING_RECOVERING);
    }
    P2_MotionAcceleration(&s,level,2720);
    assert(P2_MotionHeading(&s,observed(90)));
    assert(error(s.heading_deg,90)<.01f);
    /* 32-bit tick rollover must retain the correct small filter interval. */
    settle(&s); s.start_ms=UINT32_MAX-3000U; s.last_ms=UINT32_MAX-10U;
    assert(ready_heading(&s,observed(359)));
    s.last_ms=9U; assert(ready_heading(&s,observed(1)));
    assert(error(s.heading_deg,0)<1.01f);
    puts("PASS: board yaw convention, circular wrap/filter, motion response, invalidity, gaps, XY bias, tick wrap");
    return 0;
}
