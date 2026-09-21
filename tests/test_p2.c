#include "p2_motion.h"
#include "p2_sensor.h"
#include "p2_packet.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void near(float actual, float expected, float eps) { assert(fabsf(actual-expected) < eps); }
static void settle(P2_Motion *s) {
    P2_MotionInit(s);
    for (unsigned t=0; t<2000; t+=20) P2_MotionAcceleration(s,(P2_Vector){0,0,1000},t);
}
static void walking(float hz, uint32_t origin) {
    P2_Motion s;
    P2_MotionInit(&s);
    for (unsigned t=0;t<2000;t+=20) P2_MotionAcceleration(&s,(P2_Vector){0,0,1000},origin+t);
    for (unsigned t=0;t<20000;t+=20) {
        float z=1000+250*sinf(6.283185307f*hz*t/1000);
        P2_MotionAcceleration(&s,(P2_Vector){0,0,z},origin+2000+t);
    }
    printf("synthetic %.1f Hz: %u steps (expected approx %.0f)\n",hz,s.steps,hz*20);
    assert(fabsf((float)s.steps-hz*20) <= 2);
    near(s.distance_m,s.steps*0.70f,0.01f);
}
static uint8_t ar[256], mr[256];
static int fault;
static int32_t ra(uint16_t r,uint8_t *d,uint16_t n) { if(fault) return -1; memcpy(d,ar+r,n);return 0; }
static int32_t rm(uint16_t r,uint8_t *d,uint16_t n) { if(fault) return -1; memcpy(d,mr+r,n);return 0; }
static int32_t wa(uint16_t r,uint8_t *d,uint16_t n) { memcpy(ar+r,d,n);return 0; }
static int32_t wm(uint16_t r,uint8_t *d,uint16_t n) { memcpy(mr+r,d,n);return 0; }
int main(void) {
    P2_Motion s;
    settle(&s);
    for (unsigned t=2000;t<62000;t+=20)
        P2_MotionAcceleration(&s,(P2_Vector){0,0,1000+8*sinf(t)},t);
    assert(s.steps==0);
    walking(1.0f,0); walking(2.0f,0); walking(3.0f,0); walking(2.0f,UINT32_MAX-5000U);
    const P2_Vector mag[]={{300,0,-400},{0,300,-400},{-300,0,-400},{0,-300,-400}};
    for (unsigned i=0;i<4;++i) {
        settle(&s); assert(P2_MotionHeading(&s,mag[i])); near(s.heading_deg,90.0f*i,0.01f);
    }
    /* Analytic 30-degree pitch: north and gravity rotate together. */
    settle(&s); s.gravity=(P2_Vector){-500,0,866.0254f};
    assert(P2_MotionHeading(&s,(P2_Vector){459.8076f,0,-196.4102f})); near(s.heading_deg,0,0.01f);
    assert(P2_MotionHeading(&s,(P2_Vector){200,300,-346.4102f})); near(s.heading_deg,90,0.01f);
    assert(!P2_MotionHeading(&s,(P2_Vector){0,0,0}));
    assert(!P2_MotionHeading(&s,(P2_Vector){9000,0,0}));
    assert(!P2_MotionHeading(&s,(P2_Vector){NAN,0,0}));
    s.gravity=(P2_Vector){1000,0,0}; assert(!P2_MotionHeading(&s,mag[0]));
    settle(&s); P2_MotionAcceleration(&s,(P2_Vector){0,0,1200},10000); assert(s.steps==0);
    P2_SensorBus b={ra,wa,rm,wm}; P2_Vector v;
    assert(!P2_SensorInit(&b)); ar[15]=0x33; mr[79]=0x40;
    assert(P2_SensorInit(&b)); assert(ar[0x23]==0x99 && mr[0x62]==0x30);
    assert(P2_SensorReadAcc(&b,&v)==0);
    ar[0x27]=8; ar[0x28]=0x40; ar[0x29]=0x1f; /* +1000 mg */
    ar[0x2a]=0xc0; ar[0x2b]=0xe0; /* -1000 mg */
    assert(P2_SensorReadAcc(&b,&v)==1); near(v.x,1000,0.01f);near(v.y,-1000,0.01f);near(v.z,0,0.01f);
    mr[0x67]=8; mr[0x68]=0x64; mr[0x6a]=0x9c;mr[0x6b]=0xff;
    assert(P2_SensorReadMag(&b,&v)==1);near(v.x,150,0.01f);near(v.y,-150,0.01f);
    fault=1; assert(!P2_SensorInit(&b)); assert(P2_SensorReadAcc(&b,&v)==-1);assert(P2_SensorReadMag(&b,&v)==-1);
    uint8_t packed[2]; P2_PutI16(packed,50000); assert(packed[0]==255 && packed[1]==127);
    P2_PutI16(packed,-50000); assert(packed[0]==0 && packed[1]==128);
    P2_PutI16(packed,-10); assert(packed[0]==246 && packed[1]==255);
    puts("PASS: stationary/noise, cadence, tick wrap, distance, cardinal/tilted headings, invalid data, driver conversions/errors, packet saturation.");
    return 0;
}
