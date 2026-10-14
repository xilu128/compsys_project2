/* Bounded handheld motion, not walking/rotation during calibration.
 * Synthetic envelopes do not establish real-board accuracy. */
#include "p2_motion.h"
#include "p2_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const double rad=.017453292519943295,pi=3.141592653589793;
static const P2_Vector bias={.4f,-2.4f,.45f};
static P2_Vector body(P2_Vector v, unsigned axis, double deg) {
    if(axis==3) return body(body(body(v,2,deg),1,.7*deg),0,.5*deg);
    double c=cos(deg*rad),s=sin(deg*rad);
    if(axis==0) return (P2_Vector){v.x,(float)(c*v.y+s*v.z),(float)(-s*v.y+c*v.z)};
    if(axis==1) return (P2_Vector){(float)(c*v.x-s*v.z),v.y,(float)(s*v.x+c*v.z)};
    return (P2_Vector){(float)(c*v.x+s*v.y),(float)(-s*v.x+c*v.y),v.z};
}
static bool tick(P2_Motion *s,unsigned now,unsigned axis,double angle,double rate,double noise) {
    P2_Vector g=bias;
    if(axis==0)g.x+=(float)rate;
    if(axis==1)g.y+=(float)rate;
    if(axis==2)g.z+=(float)rate;
    if(axis==3) {
        double p=.7*angle*rad,r=.5*angle*rad;
        g.x+=(float)(.5*rate-rate*sin(p));
        g.y+=(float)(.7*rate*cos(r)+rate*sin(r)*cos(p));
        g.z+=(float)(-.7*rate*sin(r)+rate*cos(r)*cos(p));
    }
    P2_MotionGyro(s,g,now);
    if(now%20U) return s->heading_valid;
    P2_Vector a=body((P2_Vector){0,0,1000},axis,angle);
    P2_Vector m=body((P2_Vector){150,0,300},2,31+noise);
    m=body(m,axis,angle);
    P2_MotionAcceleration(s,a,now);
    return P2_MotionHeading(s,m);
}
static void handheld_boot_and_recovery(void) {
    const double hz[]={.7,1.1,1.7};
    unsigned latest=0, recovery_latest=0;
    double largest_drift=0;
    for(unsigned axis=0;axis<4;axis++) for(unsigned f=0;f<3;f++) for(unsigned phase=0;phase<4;phase++) {
        P2_Motion s; P2_MotionInit(&s); P2_MotionRequireGyro(&s);
        s.mag_bias=(P2_Vector){0,0,0};s.mag_scale=(P2_Vector){1,1,1};
        unsigned now,first=0; double stop_angle=0;
        for(now=0;now<20000;now+=10) {
            double t=now*.001, w=2*pi*hz[f], a=.35*sin(w*t+phase*.9);
            double rate=.35*w*cos(w*t+phase*.9);
            double noise=.6*sin(2*pi*13*t)+.2*sin(2*pi*3.3*t);
            if(tick(&s,now,axis,a,rate,noise)) { first=now;stop_angle=a;break; }
        }
        if(!first)fprintf(stderr,"handheld boot timed out axis%u hz%.1f phase%u\n",axis,hz[f],phase);
        assert(first && first>=5000 && first<15000 && s.steps==0);
        assert(s.heading_deg==0.0f);
        if(first>latest)latest=first;
        float origin=s.heading_origin_deg;
        /* Stop in the acquired pose. Retain the original +/-5 degree target,
         * including residual bias accumulated by a non-integer tremor cycle. */
        double peak=0;
        for(unsigned i=0;i<12000;i++) {
            now+=10;
            assert(tick(&s,now,axis,stop_angle,0,0));
            double e=fabs(remainder(s.heading_deg,360));
            if(e>peak)peak=e;
            assert(e<=5 && s.steps==0 && s.heading_origin_deg==origin);
        }
        if(peak>largest_drift)largest_drift=peak;
        P2_Vector saved=s.attitude.bias;
        now+=80; /* real inertial loss */
        unsigned begin=now; bool recovered=false;
        for(unsigned i=0;i<800;i++,now+=10) {
            double t=i*.01,w=2*pi*hz[f];
            /* 1-degree reversing motion: rate exceeds old3 dps; total
             * endpoint displacement stays inside the new5 degree envelope. */
            double a=stop_angle+sin(w*t+phase*.9);
            bool valid=tick(&s,now,axis,a,w*cos(w*t+phase*.9),.3*sin(2*pi*13*t));
            assert(s.heading_origin_deg==origin && s.steps==0);
            assert(s.attitude.bias.x==saved.x && s.attitude.bias.y==saved.y && s.attitude.bias.z==saved.z);
            if(valid) { assert(now-begin>=P2_HEADING_RECOVERY_MS);recovered=true;break; }
        }
        assert(recovered);
        if(now-begin>recovery_latest)recovery_latest=now-begin;
        assert(fabs(remainder(s.heading_deg,360))<5);
        for(unsigned i=0;i<12000;i++) {
            now+=10;
            assert(tick(&s,now,axis,stop_angle,0,0));
            double e=fabs(remainder(s.heading_deg,360));
            if(e>largest_drift)largest_drift=e;
            assert(e<=5 && s.steps==0 && s.heading_origin_deg==origin);
        }
    }
    printf("PASS48 handheld boot/recovery trajectories including combined axes: latest boot%ums recovery%ums, stationary120s peak%.4fdeg; zero/count/bias preservation\n",latest,recovery_latest,largest_drift);
}
static void reversing_block_noise(void) {
    P2_Motion s;P2_MotionInit(&s);P2_MotionRequireGyro(&s);
    s.mag_bias=(P2_Vector){0,0,0};s.mag_scale=(P2_Vector){1,1,1};
    unsigned first=0;
    for(unsigned now=0;now<12000;now+=10) {
        double noise=(now/1000U)%2U ? .35 : -.35;
        if(tick(&s,now,2,0,0,noise)) {first=now;break;}
    }
    assert(first>=5000 && first<7000);
    printf("PASS reversing magnetic block noise acquires at%ums without weakening sustained-trend rejection\n",first);
}
int main(void) { handheld_boot_and_recovery();reversing_block_noise();return 0; }
