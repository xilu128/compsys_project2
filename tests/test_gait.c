/* Signal robustness tests, not evidence of human carrying-mode accuracy. */
#include "p2_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static uint32_t tick;
static void feed(P2_Motion *s, float magnitude, bool rotating) {
    float a=rotating ? (float)tick*.0017f : 0.0f;
    P2_MotionAcceleration(s,(P2_Vector){magnitude*sinf(a),0,magnitude*cosf(a)},tick);
    tick+=20U;
}
static void init(P2_Motion *s) {
    tick=0; P2_MotionInit(s);
    for(unsigned i=0;i<100;i++) feed(s,1000,false);
}
static void walk(P2_Motion *s,unsigned count,unsigned period,float amplitude,bool rotating) {
    for(unsigned i=0;i<count*period;i+=20) {
        float phase=6.283185307f*i/period;
        feed(s,1000-amplitude*sinf(phase)+8*sinf(phase*7),rotating);
    }
}
int main(void) {
    P2_Motion s;
    const float amplitudes[]={110,180,500};
    const unsigned periods[]={1200,800,500};
    for(unsigned a=0;a<3;a++) for(unsigned p=0;p<3;p++) {
        init(&s); walk(&s,50,periods[p],amplitudes[a],true);
        printf("amplitude %.0f mg, period %u ms: %u/50\n",amplitudes[a],periods[p],s.steps);
        assert(s.steps>=49 && s.steps<=51);
    }
    init(&s); walk(&s,50,600,20,true); assert(s.steps==0);
    init(&s); walk(&s,20,800,500,false); walk(&s,20,800,110,true);
    assert(s.steps>=38 && s.steps<=41);
    init(&s); walk(&s,20,1200,300,false); walk(&s,20,500,300,false);
    assert(s.steps>=38 && s.steps<=41); /* Continue through a speed change. */
    /* Hand-held asymmetry: adjacent weak steps need not have equal periods.
     * Once walking is confirmed, valid trough/peak cycles remain countable. */
    init(&s); walk(&s,10,600,110,false);
    for(unsigned i=0;i<40;i++) walk(&s,1,(i/2)%2 ? 1000 : 400,110,true);
    printf("alternating hand-held cadence: %u/50\n",s.steps);
    assert(s.steps>=49 && s.steps<=51);
    /* A real pause still requires a new bout, even after established walking. */
    for(unsigned i=0;i<150;i++) feed(&s,1000,false);
    uint32_t before=s.steps;
    walk(&s,1,600,110,false);
    assert(s.steps==before);
    puts("PASS: weak/strong signals, cadence, rotating frame, noise, amplitude/speed transitions");
}
