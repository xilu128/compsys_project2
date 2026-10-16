

#include "p2_motion.h"
#include "p2_config.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const double rad = 0.017453292519943295;
static const P2_Vector field = {150,0,300};
static const P2_Vector rest = {0,0,1000};
static const P2_Vector bias = {.4f,-.25f,.8f};
enum { TICK_MS = 10 };

static P2_Vector body(P2_Vector v, double yaw, double pitch, double roll) {
    double cy=cos(yaw*rad),sy=sin(yaw*rad),cp=cos(pitch*rad),sp=sin(pitch*rad);
    double cr=cos(roll*rad),sr=sin(roll*rad);
    double x=cy*v.x+sy*v.y,y=-sy*v.x+cy*v.y,z=v.z;
    double xx=cp*x-sp*z,zz=sp*x+cp*z;
    return (P2_Vector){(float)xx,(float)(cr*y+sr*zz),(float)(-sr*y+cr*zz)};
}
static P2_Vector omega(double pitch,double roll,double yd,double pd,double rd) {
    double cp=cos(pitch*rad),sp=sin(pitch*rad),cr=cos(roll*rad),sr=sin(roll*rad);
    return (P2_Vector){(float)(rd-yd*sp),(float)(pd*cr+yd*sr*cp),
                       (float)(-pd*sr+yd*cr*cp)};
}
static P2_Vector add(P2_Vector a,P2_Vector b) {
    return (P2_Vector){a.x+b.x,a.y+b.y,a.z+b.z};
}
static P2_Vector magnetic(double yaw) { return body(field,yaw,0,0); }

static double angular_error(double a,double b) { return fabs(remainder(a-b,360)); }
static bool feed(P2_Motion *s,uint32_t now,P2_Vector a,P2_Vector rate,P2_Vector m) {
    P2_MotionGyro(s,add(rate,bias),now);
    P2_MotionAcceleration(s,a,now);
    bool valid=P2_MotionHeading(s,m);
    if(valid) assert(isfinite(s->heading_deg) && s->heading_deg>=0.0f && s->heading_deg<360.0f);
    return valid;
}
static uint32_t settle(P2_Motion *s,uint32_t start,double yaw) {
    P2_MotionInit(s); P2_MotionRequireGyro(s);
    s->mag_bias=(P2_Vector){0,0,0}; s->mag_scale=(P2_Vector){1,1,1};
    for (unsigned i=0;i<2000;i++) {
        uint32_t now=start+i*TICK_MS;
        if(feed(s,now,rest,(P2_Vector){0,0,0},magnetic(yaw))) {
            assert(s->heading_origin_set && s->heading_deg==0.0f);
            return now;
        }
    }
    assert(!"quiet startup timed out");
    return start;
}
static uint32_t reject_magnetic_field(P2_Motion *s,uint32_t now,double yaw) {
    for(unsigned i=1;i<=1000;i++) {
        bool valid=feed(s,now+i*TICK_MS,rest,(P2_Vector){0,0,0},magnetic(yaw+10));
        assert(valid);
        if(i>=900) assert(!s->attitude.magnetic_accepted);
    }
    return now+10000U;
}
static void finish_clean_recovery(P2_Motion *s,uint32_t now,double yaw) {
    float origin=s->heading_origin_deg;

    for(unsigned i=0;i<4000;i++) {
        now+=TICK_MS;
        assert(feed(s,now,rest,(P2_Vector){0,0,0},magnetic(yaw)));
        assert(s->heading_origin_deg==origin);
        assert(angular_error(s->heading_deg,0)<=5);
    }
    assert(angular_error(s->heading_deg,0)<.1);
}

static unsigned magnetic_drift_boundary_120_seconds(void) {
    unsigned failures=0;
    for(int direction=-1;direction<=1;direction+=2) {
        P2_Motion s;
        uint32_t now=settle(&s,0,359.5);
        float origin=s.heading_origin_deg;
        double maximum=0,first_excess=-1;
        unsigned valid_count=0;
        for(unsigned i=1;i<=12000;i++) {
            now+=TICK_MS;
            double t=i*.01;
            double observed=359.5+direction*fmin(5,.0625*t)+(i%2 ? 1 : -1);
            bool valid=feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(observed));
            assert(s.heading_origin_deg==origin);
            if(valid) {
                ++valid_count;
                double error=angular_error(s.heading_deg,0);
                if(error>maximum) maximum=error;
                if(error>5 && first_excess<0) first_excess=t;
            }
        }
        bool passed=valid_count==12000 && maximum<=5;
        if(!passed) ++failures;
        printf("%s 120 s boundary magnetic %+d*5 deg with +/-1 noise: %u/12000 valid, peak %.4f deg, first >5 %.2f s\n",
               passed?"PASS":"FAIL",direction,valid_count,maximum,first_excess);
    }
    fflush(stdout);
    return failures;
}

static void persistent_magnetic_offset(void) {
    P2_Motion s;
    uint32_t now=settle(&s,0,41);
    float origin=s.heading_origin_deg;
    double maximum=0;
    unsigned rejected=0;
    for(unsigned i=1;i<=16000;i++) {
        now+=TICK_MS;
        double t=i*.01;
        double observed=51+(i%2 ? 1 : -1);

        if(t>110 && i%17==0) observed=41;
        bool valid=feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(observed));
        assert(s.heading_origin_deg==origin);
        assert(valid);
        double e=angular_error(s.heading_deg,0);
        if(e>maximum) maximum=e;
        if(!s.attitude.magnetic_accepted) ++rejected;
        if(t>=10) assert(!s.attitude.magnetic_accepted);
    }
    assert(rejected>15000 && maximum<=5);
    bool recovered=false;
    for(unsigned i=1;i<=600;i++) {
        now+=TICK_MS;
        bool valid=feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(41+(i%2 ? .2 : -.2)));
        assert(s.heading_origin_deg==origin);
        assert(valid);
        if(s.attitude.magnetic_accepted) {
            recovered=true;
            assert(angular_error(s.heading_deg,0)<=5);
        }
    }
    assert(recovered && s.heading_valid && s.attitude.magnetic_accepted);
    assert(angular_error(s.heading_deg,0)<2.5);
    finish_clean_recovery(&s,now,41);
    printf("PASS persistent noisy +10 deg magnetic offset: live output peak %.4f deg, %u rejected magnetic samples; clean recovery\n",
           maximum,rejected);
}

static void residual_bias_120_seconds(void) {
    const double offsets[]={-.04,-.025,.025,.04};
    for(unsigned mode=0;mode<3;mode++) for(unsigned j=0;j<4;j++) {
        P2_Motion s;
        uint32_t now=settle(&s,0,41);
        float origin=s.heading_origin_deg;
        P2_Vector stored_bias=s.attitude.bias;
        double maximum=0;
        for(unsigned i=1;i<=12000;i++) {
            now+=TICK_MS;
            double t=i*.01;
            double residual=offsets[j]*(mode==1 ? fmin(1,t/30) : 1);
            double magnetic_delta=mode==2 ? 1.5*sin(2*3.141592653589793*t/120) : 0;
            double noise=mode==2 ? (i%2 ? 1 : -1) : 0;
            assert(feed(&s,now,rest,(P2_Vector){0,0,(float)residual},magnetic(41+magnetic_delta+noise)));
            assert(s.heading_origin_deg==origin);
            assert(s.attitude.bias.x==stored_bias.x && s.attitude.bias.y==stored_bias.y &&
                   s.attitude.bias.z==stored_bias.z);
            double error=angular_error(s.heading_deg,0);
            if(error>maximum) maximum=error;
            assert(error<=5);
        }
        printf("PASS 120 s gyro residual %+.3f dps, scenario %u: 100%% valid, maximum %.4f deg\n",
               offsets[j],mode,maximum);
    }
}

static void abrupt_large_magnetic_fault(void) {
    for(int direction=-1;direction<=1;direction+=2) {
        P2_Motion s;
        uint32_t now=settle(&s,0,31);
        float origin=s.heading_origin_deg;
        for(unsigned i=1;i<=1200;i++) {
            now+=TICK_MS;
            bool valid=feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(31+direction*25));
            assert(!s.attitude.magnetic_accepted);
            assert(valid && angular_error(s.heading_deg,0)<.1);
            assert(s.heading_origin_deg==origin);
        }
        bool recovered=false;
        for(unsigned i=1;i<=500;i++) {
            now+=TICK_MS;
            bool valid=feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(31));
            assert(valid);
            if(i<200) assert(!s.attitude.magnetic_accepted);
            if(s.attitude.magnetic_accepted) {
                recovered=true;
                assert(angular_error(s.heading_deg,0)<.1);
            }
        }
        assert(recovered && s.heading_valid);
    }
    puts("PASS abrupt +/-25 deg magnetic aiding rejection preserves live gyro heading");
}

static unsigned combined_boundary_120_seconds(void) {
    unsigned failures=0;
    for(int magnetic_sign=-1;magnetic_sign<=1;magnetic_sign+=2)
        for(int gyro_sign=-1;gyro_sign<=1;gyro_sign+=2) {
            P2_Motion s;
            uint32_t now=settle(&s,0,41);
            float origin=s.heading_origin_deg;
            unsigned valid_count=0;
            double maximum=0,first_invalid=-1;
            for(unsigned i=1;i<=12000;i++) {
                now+=TICK_MS;
                double t=i*.01;
                bool valid=feed(&s,now,rest,(P2_Vector){0,0,gyro_sign*.04f},
                    magnetic(41+magnetic_sign*fmin(5,.0625*t)));
                assert(s.heading_origin_deg==origin);
                if(valid) {
                    ++valid_count;
                    double error=angular_error(s.heading_deg,0);
                    if(error>maximum) maximum=error;
                } else if(first_invalid<0) first_invalid=t;
            }
            bool passed=valid_count==12000 && maximum<=5;
            if(!passed) ++failures;
            printf("%s 120 s boundary magnetic %+d*5 deg + gyro %+.2f dps: %u/12000 valid, peak %.4f deg, first invalid %.2f s\n",
                   passed?"PASS":"FAIL",magnetic_sign,gyro_sign*.04,valid_count,maximum,first_invalid);
        }
    fflush(stdout);
    return failures;
}

static void slow_real_yaw(void) {
    for(int direction=-1;direction<=1;direction+=2) {
        P2_Motion s;
        uint32_t now=settle(&s,100,359.5);
        double maximum=0;
        for(unsigned i=1;i<=12000;i++) {
            now+=TICK_MS;
            double turn=direction*.001*i;
            assert(feed(&s,now,rest,(P2_Vector){0,0,direction*.1f},magnetic(359.5+turn)));
            double error=angular_error(s.heading_deg,turn);
            if(error>maximum) maximum=error;
        }
        assert(maximum<.15 && angular_error(s.heading_deg,direction*12)<.1);
    }
    puts("PASS genuine slow +/-0.1 deg/s yaw is live and crosses the 359/0 boundary");
}

static void combined_tilt_and_yaw(void) {
    double maximum=0;
    for(int initial=0;initial<360;initial+=90) {
        P2_Motion s;
        uint32_t now=settle(&s,0,initial);
        for(unsigned i=1;i<=4800;i++) {
            now+=TICK_MS;
            double t=i*.01,mid=t-.005,yaw=initial+20*t;
            double pitch=45*sin(1.3*t),roll=30*sin(.7*t);
            P2_Vector a=body(rest,yaw,pitch,roll);
            if(i%160>=80 && i%160<100)
                a=add(a,body((P2_Vector){350,-150,140},yaw,pitch,roll));
            P2_Vector g=omega(45*sin(1.3*mid),30*sin(.7*mid),20,
                58.5*cos(1.3*mid),21*cos(.7*mid));
            assert(feed(&s,now,a,g,body(field,yaw,pitch,roll)));
            double e=angular_error(s.heading_deg,yaw-initial);
            if(e>maximum) maximum=e;
        }
    }
    assert(maximum<1);
    printf("PASS combined tilt/translation/yaw remains live: heading error %.4f deg\n",maximum);
}

static void recovery_blocks_are_real_samples(void) {
    P2_Motion s;
    uint32_t now=settle(&s,0,23);
    now=reject_magnetic_field(&s,now,23);
    uint32_t start=now;

    for(unsigned i=1;i<=210;i++) {
        now+=TICK_MS;
        P2_MotionGyro(&s,bias,now);
        P2_MotionAcceleration(&s,rest,now);
        if(i%10==0) {
            assert(P2_MotionHeading(&s,magnetic(23)) && !s.attitude.magnetic_accepted);
            assert(isfinite(s.heading_deg) && s.heading_deg>=0.0f && s.heading_deg<360.0f);
            float before=s.heading_deg;
            uint32_t trusted=s.attitude.trusted_ms;
            for(unsigned repeat=0;repeat<100;repeat++) {
                assert(P2_MotionHeading(&s,magnetic(23)) && !s.attitude.magnetic_accepted);
                assert(isfinite(s.heading_deg) && s.heading_deg>=0.0f && s.heading_deg<360.0f);
                assert(s.heading_deg==before && s.attitude.trusted_ms==trusted);
            }
        }
    }
    assert(now-start>P2_MAG_GUARD_BLOCK_MS && s.heading_valid && !s.attitude.magnetic_accepted);
    bool recovered=false;
    for(unsigned i=0;i<500;i++) {
        now+=TICK_MS;
        assert(feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(23)));
        if(s.attitude.magnetic_accepted) recovered=true;
    }
    assert(recovered && s.heading_valid);
    finish_clean_recovery(&s,now,23);
    puts("PASS duplicate calls cannot create a full clean recovery block");
}

static void magnetic_gap_interrupts_recovery(void) {
    for(unsigned wrapping=0;wrapping<2;wrapping++) {
        P2_Motion s;
        uint32_t now=settle(&s,wrapping ? UINT32_MAX-19000U : 0U,359.8);
        now=reject_magnetic_field(&s,now,359.8);
        float origin=s.heading_origin_deg;
        for(unsigned i=0;i<150;i++) {
            now+=TICK_MS;
            assert(feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(359.8)));
            assert(!s.attitude.magnetic_accepted);
        }

        for(unsigned i=0;i<P2_MAG_FRESH_MS/TICK_MS+10U;i++) {
            now+=TICK_MS;
            P2_MotionGyro(&s,bias,now);
            P2_MotionAcceleration(&s,rest,now);
        }
        for(unsigned i=0;i<150;i++) {
            now+=TICK_MS;
            assert(feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(359.8)));
            assert(!s.attitude.magnetic_accepted);
        }
        bool recovered=false;
        for(unsigned i=0;i<300;i++) {
            now+=TICK_MS;
            assert(feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(359.8)));
            if(s.attitude.magnetic_accepted) recovered=true;
            assert(s.heading_origin_deg==origin);
        }
        assert(recovered && s.heading_valid && angular_error(s.heading_deg,0)<=5);
        finish_clean_recovery(&s,now,359.8);
    }
    puts("PASS mag-only gap interrupts clean recovery; circular/tick boundaries remain safe");
}

static void interrupted_blocks_cannot_refresh_trust(void) {
    for(unsigned mode=0;mode<2;mode++) {
        P2_Motion s;
        uint32_t now=settle(&s,0,53);
        float origin=s.heading_origin_deg;
        for(unsigned i=1;i<=1500;i++) {
            now+=TICK_MS;
            P2_Vector a=rest;
            double observed=63+(i%2 ? 1 : -1);
            if(i%100==0) {
                if(mode==0) observed=143;
                else a.z=1200;
            }
            bool valid=feed(&s,now,a,(P2_Vector){0,0,0},magnetic(observed));
            assert(valid && angular_error(s.heading_deg,0)<=5);
            if(i>=1000) assert(!s.attitude.magnetic_accepted);
            assert(s.heading_origin_deg==origin);
        }

        if(mode==1) assert(s.attitude.mag_guard.blocked);
        bool recovered=false;
        for(unsigned i=1;i<=450;i++) {
            now+=TICK_MS;
            bool valid=feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(53));
            assert(valid);
            if(i<100) assert(!s.attitude.magnetic_accepted);
            if(s.attitude.magnetic_accepted) recovered=true;
            assert(s.heading_origin_deg==origin);
        }
        assert(recovered && s.heading_valid && angular_error(s.heading_deg,0)<=5);
        finish_clean_recovery(&s,now,53);
    }
    puts("PASS repeated magnetic/acceleration spikes cannot keep unverified field trusted");
}

static void lost_inertial_reference_recovers(void) {
    for(unsigned mode=0;mode<6;mode++) {
        bool disturbed=(mode&1U)!=0,invalid_gyro=mode>=2;
        P2_Motion s;
        uint32_t now=settle(&s,0,64);
        if(disturbed) now=reject_magnetic_field(&s,now,64);
        float origin=s.heading_origin_deg;
        P2_Vector saved_bias=s.attitude.bias;
        s.steps=17; s.distance_m=11.9f;
        now+=invalid_gyro ? TICK_MS : P2_GYRO_FRESH_MS+20U;
        P2_Vector bad=mode>=4 ? (P2_Vector){P2_GYRO_MAX_DPS+1,0,0} : (P2_Vector){NAN,0,0};
        P2_MotionGyro(&s,invalid_gyro ? bad : bias,now);
        P2_MotionAcceleration(&s,rest,now);
        assert(!P2_MotionHeading(&s,magnetic(84)));
        P2_Motion plain=s;
        plain.attitude.required=false;
        for(unsigned i=1;i<=2200;i++) {
            now+=TICK_MS;
            double t=i*.01;
            P2_Vector a={0,0,(float)(1000+(t<2.5 ? 0 : 180*sin(4*3.141592653589793*t)))};
            bool valid=feed(&s,now,a,(P2_Vector){0,0,0},magnetic(84));
            if(i*TICK_MS<P2_HEADING_RECOVERY_MS) assert(!valid);
            if(i*TICK_MS>P2_HEADING_RECOVERY_MS+TICK_MS) {
                assert(valid && !s.attitude.mag_guard.reference_lost);
                assert(fabsf(s.heading_deg-20)<.1f);
            }
            assert(s.heading_origin_set && s.heading_origin_deg==origin);
            assert(s.attitude.bias.x==saved_bias.x && s.attitude.bias.y==saved_bias.y &&
                   s.attitude.bias.z==saved_bias.z);
            P2_MotionAcceleration(&plain,a,now);
            assert(s.steps==plain.steps && s.distance_m==plain.distance_m);
            assert(s.baseline==plain.baseline && s.filtered==plain.filtered);
        }
        assert(s.steps>42 && s.heading_valid);
    }
    puts("PASS normal/disturbed gyro gap/NaN/over-range recover automatically without losing zero/steps");
}

static void recovery_requires_continuous_fresh_sensors(void) {

    for(unsigned mode=0;mode<11;mode++) {
        P2_Motion s;
        uint32_t boot_ms=P2_GYRO_CAL_MS+P2_STARTUP_BLOCK_MS*P2_STARTUP_BLOCKS;
        uint32_t now=settle(&s,mode==7 ? UINT32_MAX-boot_ms-1500U : 0U,359.5);
        float origin=s.heading_origin_deg;
        now+=P2_GYRO_FRESH_MS+20U;
        assert(!feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(19.5)));
        for(unsigned i=0;i<120;i++) {
            now+=TICK_MS;
            assert(!feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(19.5)));
        }
        if(mode==0) {

            for(unsigned i=0;i<300;i++) {
                now+=TICK_MS;
                P2_MotionAcceleration(&s,rest,now);
                assert(!P2_MotionHeading(&s,magnetic(19.5)));
            }
        } else if(mode==1 || mode==7) {
            for(unsigned i=0;i<P2_MAG_FRESH_MS/TICK_MS+1U;i++) {
                now+=TICK_MS;
                P2_MotionGyro(&s,bias,now);
                P2_MotionAcceleration(&s,rest,now);
            }
        } else if(mode==8) {

            for(unsigned i=0;i<15;i++) {
                now+=TICK_MS;
                P2_MotionGyro(&s,bias,now);
                assert(!P2_MotionHeading(&s,magnetic(19.5)));
            }
        } else {
            now+=TICK_MS;
            P2_Vector m=magnetic(19.5),a=rest,g={0,0,0};
            if(mode==2) m.x=NAN;
            if(mode==3) m=(P2_Vector){m.x*1.8f,m.y*1.8f,m.z*1.8f};
            if(mode==4) a.z=1200;
            if(mode==5) g.z=P2_HEADING_RECOVERY_RATE_DPS+1;
            if(mode==6) a=body(rest,19.5,8,0);
            if(mode==9) m=body((P2_Vector){335,0,0},19.5,0,0);
            if(mode==10) m=(P2_Vector){0,0,335};
            P2_MotionGyro(&s,add(g,bias),now);
            P2_MotionAcceleration(&s,a,now);

            if(mode<4 || mode>=9) assert(!P2_MotionHeading(&s,m));
        }
        uint32_t clean_start=now+TICK_MS;
        for(unsigned i=0;i<=220;i++) {
            now=clean_start+i*TICK_MS;
            bool valid=feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(19.5));
            if(i*TICK_MS<P2_HEADING_RECOVERY_MS) assert(!valid);
            else assert(valid && !s.attitude.mag_guard.reference_lost);
            assert(s.heading_origin_deg==origin);
        }
        assert(fabsf(s.heading_deg-20)<.1f);
    }
    P2_Motion s;
    uint32_t now=settle(&s,0,71);
    now+=P2_GYRO_FRESH_MS+20U;
    assert(!feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(91)));
    for(unsigned i=1;i<=210;i++) {
        now+=TICK_MS;
        P2_MotionGyro(&s,bias,now);
        P2_MotionAcceleration(&s,rest,now);
        if(i%10==0) {
            for(unsigned repeat=0;repeat<100;repeat++)
                assert(!P2_MotionHeading(&s,magnetic(91)));
        }
    }
    assert(!s.heading_valid && s.attitude.recovery.samples<P2_HEADING_RECOVERY_MIN_SAMPLES);
    for(unsigned i=0;i<250;i++) {
        now+=TICK_MS;
        feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(91));
    }
    assert(s.heading_valid && fabsf(s.heading_deg-20)<.1f);
    puts("PASS controlled reacquisition rejects stale sensors, duplicates, gaps, bad field/dip/projection and between-mag motion/accel impulses");
}

static void recovery_window_crosses_tick_wrap(void) {
    P2_Motion s;
    uint32_t boot_ms=P2_GYRO_CAL_MS+P2_STARTUP_BLOCK_MS*P2_STARTUP_BLOCKS;
    uint32_t now=settle(&s,UINT32_MAX-boot_ms-1100U,359.5);
    float origin=s.heading_origin_deg;
    P2_Vector stored_bias=s.attitude.bias;
    s.steps=31; s.distance_m=21.7f;
    now+=P2_GYRO_FRESH_MS+20U;
    uint32_t recovery_start=now;
    assert(recovery_start>UINT32_MAX-P2_HEADING_RECOVERY_MS);
    unsigned first_ready=0;
    for(unsigned i=0;i<=220;i++) {
        now=recovery_start+i*TICK_MS;
        bool valid=feed(&s,now,rest,(P2_Vector){0,0,0},magnetic(19.5));
        if(i*TICK_MS<P2_HEADING_RECOVERY_MS) {
            assert(!valid && !s.attitude.ready && s.attitude.mag_guard.reference_lost);
        } else {
            assert(valid && s.attitude.ready && !s.attitude.mag_guard.reference_lost);
            assert(s.attitude.recovery.samples>=P2_HEADING_RECOVERY_MIN_SAMPLES);
            assert(fabsf(s.heading_deg-20)<.1f);
            if(!first_ready) first_ready=i*TICK_MS;
        }
        assert(s.heading_origin_deg==origin && s.heading_origin_set);
        assert(s.attitude.bias.x==stored_bias.x && s.attitude.bias.y==stored_bias.y &&
               s.attitude.bias.z==stored_bias.z);
        assert(s.steps==31 && s.distance_m==21.7f);
    }
    assert(now<recovery_start && first_ready==P2_HEADING_RECOVERY_MS);
    printf("PASS recovery window crosses uint32 tick wrap: first valid at %u ms, original zero/bias/steps preserved\n",
           first_ready);
}

static void asynchronous_104hz_recovery(void) {

    const uint32_t phases_us[]={1000U,3500U,9200U,13500U,19900U};
    unsigned earliest=UINT32_MAX,latest=0;
    for(unsigned wrapping=0;wrapping<2;wrapping++)
        for(unsigned phase=0;phase<sizeof(phases_us)/sizeof(phases_us[0]);phase++) {
            P2_Motion s;
            double yaw0=phase%2 ? .5 : 359.5,turn=phase%2 ? -20 : 20;
            uint32_t boot_ms=P2_GYRO_CAL_MS+P2_STARTUP_BLOCK_MS*P2_STARTUP_BLOCKS;
            uint32_t now=settle(&s,wrapping ? UINT32_MAX-boot_ms-1100U : 0U,yaw0);
            float origin=s.heading_origin_deg;
            P2_Vector stored_bias=s.attitude.bias;
            s.steps=31; s.distance_m=21.7f;
            now+=TICK_MS;
            P2_MotionGyro(&s,(P2_Vector){NAN,0,0},now);
            P2_MotionAcceleration(&s,rest,now);
            assert(!P2_MotionHeading(&s,magnetic(yaw0+turn)));
            uint32_t base=now+TICK_MS,first_mag=base+phases_us[phase]/1000U;
            uint32_t previous_gyro=base,first_ready=UINT32_MAX;
            unsigned gyro_count=0,mag_count=0,nine_ms=0,ten_ms=0;
            uint64_t gyro_us=0,mag_us=phases_us[phase];
            uint64_t end_us=phases_us[phase]+2200000U;
            while(gyro_us<=end_us || mag_us<=end_us) {
                if(gyro_us<=mag_us) {
                    now=base+(uint32_t)(gyro_us/1000U);
                    if(gyro_count) {
                        uint32_t elapsed=now-previous_gyro;
                        assert(elapsed==9U || elapsed==10U);
                        if(elapsed==9U) ++nine_ms;
                        else ++ten_ms;
                    }
                    previous_gyro=now;
                    P2_MotionGyro(&s,bias,now);
                    ++gyro_count;
                    gyro_us=(uint64_t)gyro_count*1000000U/104U;
                } else {
                    now=base+(uint32_t)(mag_us/1000U);
                    uint32_t elapsed=now-first_mag;
                    ++mag_count;
                    P2_MotionAcceleration(&s,rest,now);
                    bool valid=P2_MotionHeading(&s,magnetic(yaw0+turn));
                    if(elapsed<P2_HEADING_RECOVERY_MS ||
                       mag_count<P2_HEADING_RECOVERY_MIN_SAMPLES) {
                        assert(!valid && s.attitude.mag_guard.reference_lost);
                    }
                    if(valid) {
                        assert(elapsed>=P2_HEADING_RECOVERY_MS);
                        assert(s.attitude.recovery.samples>=P2_HEADING_RECOVERY_MIN_SAMPLES);
                        assert(!s.attitude.mag_guard.reference_lost && s.attitude.ready);
                        assert(isfinite(s.heading_deg) && s.heading_deg>=0.0f && s.heading_deg<360.0f);
                        assert(angular_error(s.heading_deg,turn)<.1f);
                        if(first_ready==UINT32_MAX) first_ready=elapsed;
                    }
                    assert(s.heading_origin_set && s.heading_origin_deg==origin);
                    assert(s.attitude.bias.x==stored_bias.x && s.attitude.bias.y==stored_bias.y &&
                           s.attitude.bias.z==stored_bias.z);
                    assert(s.steps==31 && s.distance_m==21.7f);
                    mag_us+=20000U;
                }
            }
            assert(nine_ms>0 && ten_ms>0 && gyro_count>2U*mag_count);
            assert(s.heading_valid && first_ready>=P2_HEADING_RECOVERY_MS &&
                   first_ready<=P2_HEADING_RECOVERY_MS+20U);
            if(wrapping) assert(now<base);
            if(first_ready<earliest) earliest=first_ready;
            if(first_ready>latest) latest=first_ready;
        }
    printf("PASS 104 Hz gyro / 50 Hz accel-mag recovery: five phases, both tick domains, first valid %u..%u ms, zero/bias/steps preserved\n",
           earliest,latest);
}

int main(int argc,char **argv) {
    if(argc==2 && strcmp(argv[1],"--boundary")==0) {
        unsigned failures=magnetic_drift_boundary_120_seconds();
        failures+=combined_boundary_120_seconds();
        if(failures) {
            fprintf(stderr,"BOUNDARY CHARACTERIZATION FAILED: %u cases did not meet +/-5 deg AND 100%% validity.\n",failures);
            return 1;
        }
        puts("PASS boundary characterization: every case met +/-5 deg and 100% validity");
        return 0;
    }
    if(argc!=1) {
        fprintf(stderr,"Usage: %s [--boundary]\n",argv[0]);
        return 2;
    }
    residual_bias_120_seconds();
    persistent_magnetic_offset();
    abrupt_large_magnetic_fault();
    slow_real_yaw();
    combined_tilt_and_yaw();
    recovery_blocks_are_real_samples();
    magnetic_gap_interrupts_recovery();
    interrupted_blocks_cannot_refresh_trust();
    lost_inertial_reference_recovers();
    recovery_requires_continuous_fresh_sensors();
    recovery_window_crosses_tick_wrap();
    asynchronous_104hz_recovery();
    puts("PASS default regression envelope; combined extreme boundary is separate (--boundary), not claimed passed here");
    return 0;
}
