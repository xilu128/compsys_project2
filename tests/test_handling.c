

#include "p2_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t now;
static void sample(P2_Motion *s, float magnitude, float angle, unsigned axis) {
    P2_Vector a = {0,0,magnitude*cosf(angle)};
    if (axis == 0U) a.x = magnitude*sinf(angle);
    else a.y = magnitude*sinf(angle);
    P2_MotionAcceleration(s,a,now);
    now += 20U;
}
static void rest(P2_Motion *s, unsigned duration) {
    for (unsigned t=0; t<duration; t+=20U) sample(s,1000,0,0);
}
static void init(P2_Motion *s) {
    now=0; P2_MotionInit(s); rest(s,2000);
}
static void motion(P2_Motion *s, unsigned duration, float hz,
                   float angle_deg, float amplitude, unsigned axis) {
    for (unsigned t=0; t<duration; t+=20U) {
        float phase=6.283185307f*hz*t*.001f;
        float angle=angle_deg*.01745329252f*sinf(phase);
        sample(s,1000-amplitude*sinf(phase),angle,axis);
    }
}
int main(void) {
    P2_Motion s;
    const float angles[]={30,60,100};
    const float frequencies[]={.4f,.8f,1.2f};
    for (unsigned axis=0; axis<2; ++axis)
        for (unsigned a=0; a<3; ++a)
            for (unsigned f=0; f<3; ++f) {
                init(&s);
                motion(&s,20000,frequencies[f],angles[a],250,axis);
                rest(&s,3000);
                printf("rock axis=%u amplitude=%.0f deg hz=%.1f: %u steps\n",
                       axis,angles[a],frequencies[f],s.steps);
                assert(s.steps==0U);
                assert(s.distance_m==0.0f);
            }

    init(&s);
    motion(&s,10000,2,5,250,0);
    assert(s.steps>=19 && s.steps<=20);
    uint32_t before=s.steps;
    rest(&s,2500);
    motion(&s,10000,.8f,60,250,0);
    rest(&s,3000);
    assert(s.steps==before);
    motion(&s,10000,2,5,250,1);
    assert(s.steps>=before+19 && s.steps<=before+20);

    init(&s);
    motion(&s,10000,.8f,60,250,0);
    assert(s.steps==0);
    motion(&s,15000,2,5,250,0);
    assert(s.steps>=24 && s.steps<=30);

    now=UINT32_MAX-7000U; P2_MotionInit(&s); rest(&s,2000);
    motion(&s,10000,.8f,60,250,0);
    assert(s.steps==0);
    rest(&s,3000);
    motion(&s,10000,2,5,250,1);
    assert(s.steps>=19 && s.steps<=20);
    puts("PASS: large reversing handling, modest walking sway, recovery and distance");
    return 0;
}
