/* Regression of direction convention, invalid handling and circular filtering.
 * Analytic signals are not a claim of real-world heading accuracy. */
#include "p2_motion.h"
#include "p2_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const P2_Vector level = {0,0,1000};
static float error(float a, float b) {
    /* Compare analytic geometry after the configured reference rotation. */
    assert(isfinite(a));
    float e = remainderf(a-b-P2_HEADING_OFFSET_DEG,360.0f);
    return fabsf(e);
}
static void settle(P2_Motion *s) {
    P2_MotionInit(s);
    /* Geometry/filter fixtures use a fixed zero; boot-relative tests below
     * explicitly clear this flag to exercise real startup acquisition. */
    s->heading_origin_set = true;
    s->mag_bias=(P2_Vector){0,0,0};
    s->mag_scale=(P2_Vector){1,1,1};
    for (uint32_t t=0;t<=1200;t+=20) P2_MotionAcceleration(s,level,t);
    s->magnetic_reference=sqrtf(300*300+250*250); /* Geometry fixture. */
}
static P2_Vector observed(float clockwise) {
    /* Raw observed horizontal field rotates oppositely in the labelled CW
     * fixture; cardinal cases below explicitly exercise this board convention. */
    float r=clockwise*0.01745329252f;
    return (P2_Vector){300*cosf(r),-300*sinf(r),250};
}
/* Independent fixture: body-to-world Rz(yaw)*Ry(pitch)*Rx(roll).
 * Transform a fixed world vector back into the sensor frame with R transpose. */
static P2_Vector in_body(P2_Vector v, float yaw, float pitch, float roll) {
    float d=0.01745329252f;
    float cy=cosf(yaw*d), sy=sinf(yaw*d);
    float cp=cosf(pitch*d), sp=sinf(pitch*d);
    float cr=cosf(roll*d), sr=sinf(roll*d);
    float x=cy*v.x+sy*v.y, y=-sy*v.x+cy*v.y, z=v.z;
    float xx=cp*x-sp*z, zz=sp*x+cp*z;
    return (P2_Vector){xx,cr*y+sr*zz,-sr*y+cr*zz};
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
    /* Regression: a complete negative/positive cycle must not poison the
     * branch chosen when subsequently oscillating through startup zero. */
    for(int direction=-1;direction<=1;direction+=2) {
        settle(&s); s.heading_origin_set=false;
        assert(ready_heading(&s,observed(137)));
        for(int angle=1;angle<=363;angle++) {
            s.heading_valid=false; /* Isolate branch semantics from filter lag. */
            P2_MotionAcceleration(&s,level,s.last_ms+20U);
            assert(ready_heading(&s,observed(137+direction*(float)angle)));
        }
        const float crossing[]={-3,-1,0,3,1,0,-3,3,-3,0.01f,-0.01f,3};
        for(unsigned j=0;j<sizeof(crossing)/sizeof(crossing[0]);j++) {
            s.heading_valid=false;
            P2_MotionAcceleration(&s,level,s.last_ms+20U);
            assert(ready_heading(&s,observed(137+crossing[j])));
            assert(fabsf(s.heading_deg-crossing[j])<.06f);
        }
    }
    puts("PASS: full cycle then -3 -> 0 -> +3; reverse crossings and sub-resolution zero");
    /* Actual moving-tilt fixture: yaw is constant while pitch/roll oscillate
     * at 0.5 Hz. Validate while moving, not only after a long settling hold. */
    float max_dynamic_error=0.0f;
    for(int yaw=0;yaw<360;yaw+=30) {
        settle(&s); s.heading_origin_set=false;
        assert(ready_heading(&s,observed((float)yaw)));
        for(unsigned i=1;i<=1000;i++) {
            float phase=6.283185307f*.5f*i*.02f;
            float pitch=40.0f*sinf(phase), roll=25.0f*sinf(phase*.5f);
            P2_MotionAcceleration(&s,in_body(level,(float)yaw,pitch,roll),s.last_ms+20U);
            assert(ready_heading(&s,in_body((P2_Vector){300,0,250},(float)yaw,pitch,roll)));
            max_dynamic_error=fmaxf(max_dynamic_error,error(s.heading_deg,0));
        }
    }
    printf("Dynamic tilt max analytic heading error: %.3f deg\n",max_dynamic_error);
    assert(max_dynamic_error<5.0f);
    /* All heading quadrants, combined pitch/roll, with shipping calibration
     * inverted to generate raw readings. This proves geometry, not hardware
     * axis alignment or the physical calibration coefficients. */
    float max_tilt_error=0.0f;
    unsigned tilt_cases=0;
    for(int yaw=0;yaw<360;yaw+=15)
        for(int pitch=-60;pitch<=60;pitch+=20)
            for(int roll=-60;roll<=60;roll+=20) {
                settle(&s);
                s.gravity=in_body(level,(float)yaw,(float)pitch,(float)roll);
                s.mag_bias=(P2_Vector){P2_MAG_BIAS_X,P2_MAG_BIAS_Y,P2_MAG_BIAS_Z};
                s.mag_scale=(P2_Vector){P2_MAG_SCALE_X,P2_MAG_SCALE_Y,P2_MAG_SCALE_Z};
                P2_Vector m=in_body((P2_Vector){300,0,250},(float)yaw,(float)pitch,(float)roll);
                m=(P2_Vector){m.x/s.mag_scale.x+s.mag_bias.x,
                             m.y/s.mag_scale.y+s.mag_bias.y,
                             m.z/s.mag_scale.z+s.mag_bias.z};
                assert(ready_heading(&s,m));
                float e=error(s.heading_deg,(float)yaw);
                max_tilt_error=fmaxf(max_tilt_error,e);
                assert(e<.06f);
                ++tilt_cases;
            }
    /* Gravity low-pass converges after a real change of tilt while the
     * world yaw stays fixed; do not bypass the accelerometer update here. */
    settle(&s); s.heading_origin_set=false;
    assert(ready_heading(&s,observed(43)));
    P2_Vector tilted_g=in_body(level,43,45,-35);
    P2_Vector tilted_m=in_body((P2_Vector){300,0,250},43,45,-35);
    for(unsigned i=0;i<300;i++) {
        P2_MotionAcceleration(&s,tilted_g,s.last_ms+20U);
        (void)P2_MotionHeading(&s,tilted_m);
    }
    assert(s.heading_valid && error(s.heading_deg,0)<.2f);
    /* +X too near vertical has no reliable horizontal heading. */
    settle(&s); s.gravity=in_body(level,0,85,0);
    assert(!P2_MotionHeading(&s,in_body((P2_Vector){300,0,250},0,85,0)));
    printf("PASS: %u combined tilt cases, max analytic error %.5f deg; settled tilt and near-vertical rejection\n",
           tilt_cases,max_tilt_error);
    /* Smooth two-turn traces cross +/-180 and +/-360 without wrapping.
     * Returning along the same path returns accumulated angle to zero. */
    for(int sign=-1;sign<=1;sign+=2) {
        settle(&s); s.heading_origin_set=false;
        assert(ready_heading(&s,observed(137)));
        for(unsigned i=1;i<=1440;i++) {
            float turn=sign*(i<=720 ? (float)i : 1440.0f-i);
            P2_MotionAcceleration(&s,level,s.last_ms+20U);
            assert(ready_heading(&s,observed(137+turn)));
            assert(error(s.heading_turns_deg,turn)<9.0f); /* intentional filter lag */
            assert(s.heading_deg > -360.0f && s.heading_deg < 360.0f);
            assert(error(s.heading_deg,turn)<9.0f);
            if(i==360 || i==720) {
                for(unsigned j=0;j<100;j++) {
                    P2_MotionAcceleration(&s,level,s.last_ms+20U);
                    assert(ready_heading(&s,observed(137+turn)));
                }
                assert(fabsf(s.heading_turns_deg)<.05f);
                assert(s.heading_deg==0.0f);
            }
        }
        for(unsigned j=0;j<100;j++) {
            P2_MotionAcceleration(&s,level,s.last_ms+20U);
            assert(ready_heading(&s,observed(137)));
        }
        assert(fabsf(s.heading_deg)<.05f);
    }
    puts("PASS: signed full-turn display wrap and reverse return to zero");
    /* Arbitrary startup directions become zero; recovery retains that origin. */
    for (unsigned origin=0; origin<360; origin+=37) {
        settle(&s); s.heading_origin_set=false;
        assert(!P2_MotionHeading(&s,(P2_Vector){NAN,0,0}));
        assert(!s.heading_origin_set);
        assert(ready_heading(&s,observed((float)origin)));
        assert(error(s.heading_deg,0)<.01f);
        float saved=s.heading_origin_deg;
        const float turns[]={90,180,270,360,-90,-180,-270,-360};
        for (unsigned j=0;j<sizeof(turns)/sizeof(turns[0]);j++) {
            /* Fresh seed isolates geometry from intentional filter lag. */
            assert(!P2_MotionHeading(&s,(P2_Vector){0,0,0}));
            P2_MotionAcceleration(&s,level,s.last_ms+20U);
            assert(ready_heading(&s,observed(origin+turns[j])));
            assert(error(s.heading_deg,turns[j])<.02f);
            assert(s.heading_origin_deg==saved);
        }
        P2_MotionAcceleration(&s,(P2_Vector){NAN,0,0},s.last_ms+20U);
        uint32_t restart=s.last_ms+2000U;
        for(uint32_t t=0;t<=1200;t+=20) P2_MotionAcceleration(&s,level,restart+t);
        assert(ready_heading(&s,observed(origin+90.0f)));
        assert(error(s.heading_deg,90)<.02f);
        assert(s.heading_origin_deg==saved);
        P2_MotionInit(&s);
        assert(!s.heading_origin_set);
        assert(!P2_MotionHeading(&s,observed(123)));
    }
    settle(&s); s.heading_origin_set=false;
    assert(ready_heading(&s,observed(123)));
    for(unsigned i=0;i<100;i++) {
        P2_MotionAcceleration(&s,level,s.last_ms+20U);
        assert(ready_heading(&s,observed(i%2 ? 124 : 122)));
        assert(error(s.heading_deg,0)<1.01f);
    }
    puts("PASS: startup zero, CW/CCW turns, recovery retains origin, reset, relative wrap");
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
    /* Reference offset moves north; filtering must wrap at displayed zero too. */
    settle(&s); assert(ready_heading(&s,observed(359-P2_HEADING_OFFSET_DEG)));
    for(uint32_t t=1220;t<2220;t+=20) {
        P2_MotionAcceleration(&s,level,t);
        assert(ready_heading(&s,observed(1-P2_HEADING_OFFSET_DEG)));
        assert(fabsf(remainderf(s.heading_deg,360.0f)) < 1.01f);
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
    /* Shipping bias and scale are applied before projection/filtering. */
    P2_MotionInit(&s);
    s.heading_origin_set = true; /* Isolate calibration geometry. */
    for(uint32_t t=0;t<=1200;t+=20) P2_MotionAcceleration(&s,level,t);
    P2_Vector with_bias={P2_MAG_BIAS_X,
        -300/P2_MAG_SCALE_Y+P2_MAG_BIAS_Y,
        250/P2_MAG_SCALE_Z+P2_MAG_BIAS_Z};
    assert(ready_heading(&s,with_bias));
    assert(error(s.heading_deg,90)<.01f);
#if P2_MAG_CALIBRATED
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
#else
    /* Uncalibrated startup must not wait for an unreliable strength reference.
     * Field variation inside the sanity limits must not latch the display off. */
    settle(&s); s.magnetic_reference=0.0f;
    assert(P2_MotionHeading(&s,observed(0)));
    for (uint32_t t=1220;t<3220;t+=20) {
        P2_MotionAcceleration(&s,level,t);
        assert(P2_MotionHeading(&s,(P2_Vector){0,-600,500}));
    }
    assert(!P2_MotionHeading(&s,(P2_Vector){9000,0,250}));
    assert(!P2_MotionHeading(&s,(P2_Vector){0,0,250}));
    assert(P2_MotionHeading(&s,observed(90)));
#endif
    /* 32-bit tick rollover must retain the correct small filter interval. */
    settle(&s); s.start_ms=UINT32_MAX-3000U; s.last_ms=UINT32_MAX-10U;
    assert(ready_heading(&s,observed(359)));
    s.last_ms=9U; assert(ready_heading(&s,observed(1)));
    assert(error(s.heading_deg,0)<1.01f);
    puts("PASS: board yaw convention, circular wrap/filter, motion response, invalidity, gaps, calibration, tick wrap");
    return 0;
}
