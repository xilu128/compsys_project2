/* Independent deterministic fixtures for the boot-only heading reference.
 * Synthetic acceptance is not proof of hardware accuracy or field immunity. */
#include "p2_motion.h"
#include "p2_config.h"
#include "p2_packet.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const double rad = 0.017453292519943295;
static const P2_Vector rest = {0, 0, 1000};
static const P2_Vector bias = {.4f, -.25f, .8f};
enum { TICK_MS = 10 };

static P2_Vector magnetic(double yaw) {
    return (P2_Vector){(float)(150*cos(yaw*rad)),
                       (float)(-150*sin(yaw*rad)), 300};
}
static P2_Vector add(P2_Vector a, P2_Vector b) {
    return (P2_Vector){a.x+b.x, a.y+b.y, a.z+b.z};
}
static double angular_error(double a, double b) {
    return fabs(remainder(a-b, 360));
}
static bool feed(P2_Motion *s, uint32_t now, P2_Vector a,
                 P2_Vector raw_rate, P2_Vector m) {
    P2_MotionGyro(s, raw_rate, now);
    P2_MotionAcceleration(s, a, now);
    bool valid=P2_MotionHeading(s, m);
    if(valid) {
        assert(isfinite(s->heading_deg) && s->heading_deg>=0.0f && s->heading_deg<360.0f);
        int display=P2_HeadingDisplay(s->heading_deg,1,s->heading_status);
        assert(display>=0 && display<=359);
    }
    return valid;
}
static uint32_t window_ms(void) {
    return P2_STARTUP_BLOCK_MS*P2_STARTUP_BLOCKS;
}
static void pending(const P2_Motion *s) {
    assert(!s->heading_valid && !s->heading_origin_set);
    assert(!s->attitude.ready);
    assert(s->heading_status == P2_HEADING_CALIBRATING);
}
/* Isolate magnetic startup from the first gyro-bias calibration: no magnetic
 * heading is supplied until the gyro calibration has actually completed. */
static uint32_t warm_gyro(P2_Motion *s, uint32_t start) {
    P2_MotionInit(s);
    P2_MotionRequireGyro(s);
    s->mag_bias = (P2_Vector){0, 0, 0};
    s->mag_scale = (P2_Vector){1, 1, 1};
    for (unsigned i=0; i<1000; ++i) {
        uint32_t now = start+i*TICK_MS;
        P2_MotionGyro(s, bias, now);
        P2_MotionAcceleration(s, rest, now);
        if (s->attitude.calibrated) {
            assert(!s->heading_origin_set && !s->heading_valid);
            return now;
        }
    }
    assert(!"gyro calibration timed out");
    return start;
}
static uint32_t await_ready(P2_Motion *s, uint32_t now, double yaw,
                            P2_Vector raw_bias) {
    uint32_t start = now;
    while (now-start < window_ms()+3000U) {
        now += TICK_MS;
        if (feed(s, now, rest, raw_bias, magnetic(yaw))) {
            assert(s->attitude.ready && s->heading_origin_set);
            assert(s->heading_deg == 0.0f);
            assert(P2_HeadingDisplay(s->heading_deg,1,s->heading_status)==0);
            return now;
        }
        pending(s);
    }
    assert(!"stable magnetic startup timed out");
    return now;
}

static void cold_boot_asynchronous_clocks(void) {
    /* Match the independent sensor ODRs, not a synchronous 100 Hz shortcut.
     * This exercises the entire cold path: two-second gyro bias acquisition,
     * then three fresh magnetic blocks; no pre-calibrated state is injected. */
    const uint32_t phases_us[]={1000U,3500U,9200U,13500U,19900U};
    unsigned earliest=UINT32_MAX,latest=0;
    for(unsigned wrapping=0;wrapping<2;wrapping++)
        for(unsigned noisy=0;noisy<2;noisy++)
            for(unsigned phase=0;phase<sizeof(phases_us)/sizeof(phases_us[0]);phase++) {
                P2_Motion s;
                P2_MotionInit(&s); P2_MotionRequireGyro(&s);
                s.mag_bias=(P2_Vector){0,0,0}; s.mag_scale=(P2_Vector){1,1,1};
                uint32_t base=wrapping ? UINT32_MAX-2500U : 0U;
                uint32_t calibrated_at=UINT32_MAX,first_ready=UINT32_MAX;
                uint64_t gyro_us=0,mag_us=phases_us[phase];
                unsigned gyro_count=0,mag_count=0;
                while(first_ready==UINT32_MAX && gyro_us<8000000U && mag_us<8000000U) {
                    if(gyro_us<=mag_us) {
                        uint32_t elapsed=(uint32_t)(gyro_us/1000U);
                        P2_MotionGyro(&s,bias,base+elapsed);
                        if(s.attitude.calibrated && calibrated_at==UINT32_MAX) {
                            calibrated_at=elapsed;
                            assert(elapsed>=P2_GYRO_CAL_MS);
                            assert(s.attitude.count>=P2_GYRO_CAL_MIN_SAMPLES);
                        }
                        gyro_us=(uint64_t)++gyro_count*1000000U/104U;
                    } else {
                        uint32_t elapsed=(uint32_t)(mag_us/1000U);
                        P2_MotionAcceleration(&s,rest,base+elapsed);
                        double yaw=359.9+(noisy ? (mag_count%2 ? .8 : -.8) : 0);
                        bool valid=P2_MotionHeading(&s,magnetic(yaw));
                        if(valid) {
                            assert(calibrated_at!=UINT32_MAX);
                            assert(elapsed>=calibrated_at+window_ms());
                            assert(s.attitude.startup.blocks>=P2_STARTUP_BLOCKS);
                            assert(s.attitude.startup.samples>=P2_STARTUP_MIN_SAMPLES);
                            assert(s.heading_origin_set && s.heading_deg==0.0f);
                            assert(P2_HeadingDisplay(s.heading_deg,1,s.heading_status)==0);
                            assert(angular_error(s.heading_origin_deg,359.9)<.05);
                            assert(fabsf(s.attitude.bias.x-bias.x)<.0001f);
                            assert(fabsf(s.attitude.bias.y-bias.y)<.0001f);
                            assert(fabsf(s.attitude.bias.z-bias.z)<.0001f);
                            first_ready=elapsed;
                            if(wrapping) assert(base+elapsed<base);
                        } else assert(!s.heading_origin_set && !s.attitude.ready);
                        assert(s.steps==0 && s.distance_m==0.0f);
                        ++mag_count;
                        mag_us+=20000U;
                    }
                }
                unsigned minimum=P2_GYRO_CAL_MS+window_ms();
                assert(first_ready>=minimum && first_ready<=minimum+200U);
                assert(gyro_count>2U*mag_count);
                if(first_ready<earliest)earliest=first_ready;
                if(first_ready>latest)latest=first_ready;
            }
    printf("PASS cold boot 104/50 Hz, five phases, quiet/+/-0.8 deg noise and timer wrap: first valid %u..%u ms; first display exactly zero\n",
           earliest,latest);
}

static void sustained_moving_boot_is_not_forced(void) {
    P2_Motion s;
    P2_MotionInit(&s); P2_MotionRequireGyro(&s);
    s.mag_bias=(P2_Vector){0,0,0}; s.mag_scale=(P2_Vector){1,1,1};
    uint32_t now=0;
    /* A slow constant turn may look like bias to a gyro alone. Its coherent
     * magnetic trend must prevent zero acquisition well beyond ideal latency. */
    for(unsigned i=0;i<=1200;i++) {
        now=i*TICK_MS;
        double yaw=20+.2*i*.01;
        assert(!feed(&s,now,rest,add(bias,(P2_Vector){0,0,.2f}),magnetic(yaw)));
        assert(!s.heading_origin_set && !s.attitude.ready);
    }
    now=await_ready(&s,now,22.4,bias);
    assert(angular_error(s.heading_origin_deg,22.4)<.05);
    /* Stopping a below-threshold slow turn need not reset every partial block.
     * Report the retained bias rather than claiming exact stationary bias;
     * the published two-minute drift target still applies after acquisition. */
    double residual=s.attitude.bias.z-bias.z,peak=0;
    float origin=s.heading_origin_deg;
    for(unsigned i=0;i<12000;i++) {
        now+=TICK_MS;
        assert(feed(&s,now,rest,bias,magnetic(22.4)));
        double error=angular_error(s.heading_deg,0);
        if(error>peak)peak=error;
        assert(error<=5 && s.heading_origin_deg==origin && s.steps==0);
    }
    printf("PASS 12 s slow moving boot cannot force ready; after stopping bias residual %.5f dps, 120 s peak %.4f deg\n",
           residual,peak);
}

static void first_sample_outlier(void) {
    P2_Motion s;
    uint32_t now = warm_gyro(&s, 0);
    assert(!P2_MotionHeading(&s, magnetic(48)));
    pending(&s);
    now = await_ready(&s, now, 42, bias);
    assert(angular_error(s.heading_origin_deg, 42) < .05);
    double peak = 0;
    for (unsigned i=0; i<6000; ++i) {
        now += TICK_MS;
        assert(feed(&s, now, rest, bias, magnetic(42)));
        double error = angular_error(s.heading_deg, 0);
        if (error > peak) peak = error;
    }
    assert(peak < .1);
    printf("PASS first magnetic sample +6 deg outlier: 60 s post-zero drift %.4f deg\n", peak);
}

static void magnetic_ramp(void) {
    P2_Motion s;
    uint32_t now = warm_gyro(&s, 100);
    /* Magnetic direction changes without actual rotation at 0.5 degree/s.
     * Its trend exceeds the configured circular block-mean span limit. */
    for (unsigned i=0; i<=2000; ++i) {
        now += TICK_MS;
        assert(!feed(&s, now, rest, bias, magnetic(37+.005*i)));
        pending(&s);
    }
    now = await_ready(&s, now, 47, bias);
    assert(angular_error(s.heading_origin_deg, 47) < .3);
    for (unsigned i=0; i<3000; ++i) {
        now += TICK_MS;
        assert(feed(&s, now, rest, bias, magnetic(47)));
        assert(angular_error(s.heading_deg, 0) < .3);
    }
    puts("PASS magnetic startup direction ramp stays CALIBRATING until stable");
}

static void circular_mean_and_tick_wrap(void) {
    for (unsigned wrapping=0; wrapping<2; ++wrapping) {
        P2_Motion s;
        uint32_t now = warm_gyro(&s, wrapping ? UINT32_MAX-4500U : 100U);
        uint32_t start = now;
        bool ready = false;
        for (unsigned i=0; now-start<window_ms()+1000U; ++i) {
            now += TICK_MS;
            ready = feed(&s, now, rest, bias, magnetic(359.99+(i%2 ? .2 : -.2)));
            if (ready) break;
            pending(&s);
        }
        assert(ready);
        assert(now-start >= window_ms());
        assert(angular_error(s.heading_origin_deg, 359.99) < .02);
        for (unsigned i=0; i<1000; ++i) {
            now += TICK_MS;
            assert(feed(&s, now, rest, bias, magnetic(359.99)));
            assert(angular_error(s.heading_deg, 0) < .05);
        }
    }
    puts("PASS circular 359/0 averaging and uint32_t startup timer wrap");
}

static void duplicates_and_interrupted_windows(void) {
    /* Independent interruptions: rotation, translation, tilt with unchanged
     * norm, gyro gap, field strength, invalid field, and magnetometer outage. */
    for (unsigned mode=0; mode<7; ++mode) {
        P2_Motion s;
        uint32_t now = warm_gyro(&s, 100);
        for (unsigned i=0; i<2U*window_ms()/(3U*TICK_MS); ++i) {
            now += TICK_MS;
            assert(!feed(&s, now, rest, bias, magnetic(31)));
            pending(&s);
        }
        P2_Attitude before = s.attitude;
        for (unsigned i=0; i<2000; ++i)
            assert(!feed(&s, now, rest, bias, magnetic(31)));
        assert(memcmp(&before, &s.attitude, sizeof(before)) == 0);
        pending(&s);
        if (mode==6) {
            /* Fresh inertial data must not conceal a missing magnetometer. */
            for (unsigned i=0; i<P2_MAG_FRESH_MS/TICK_MS+10U; ++i) {
                now += TICK_MS;
                P2_MotionGyro(&s, bias, now);
                P2_MotionAcceleration(&s, rest, now);
            }
        }
        now += mode==3 ? P2_GYRO_FRESH_MS+20U : TICK_MS;
        P2_Vector a = rest, g = bias, m = magnetic(31);
        if (mode==0) g.z += 10;
        if (mode==1) a.z += 150;
        if (mode==2) a = (P2_Vector){(float)(1000*sin(8*rad)), 0,
                                                (float)(1000*cos(8*rad))};
        if (mode==4) m = (P2_Vector){m.x*1.3f, m.y*1.3f, m.z*1.3f};
        if (mode==5) m.x = NAN;
        assert(!feed(&s, now, a, g, m));
        pending(&s);
        uint32_t interrupted = now;
        while (now-interrupted < window_ms()-200U) {
            now += TICK_MS;
            assert(!feed(&s, now, rest, bias, magnetic(31)));
            pending(&s);
        }
        now = await_ready(&s, now, 31, bias);
        assert(angular_error(s.heading_origin_deg, 31) < .05);
    }
    puts("PASS duplicate samples do not accumulate; motion/tilt/gap/field restart startup");
}

static void bias_refinement_and_slow_rotation(void) {
    P2_Motion s;
    uint32_t now = warm_gyro(&s, 0);
    /* Gyro settles a little more after its first two-second calibration.
     * Below the stationary-rate limit, but enough to drift several degrees. */
    P2_Vector settled_bias = {.55f, -.33f, 1.02f};
    now = await_ready(&s, now, 170, settled_bias);
    assert(fabsf(s.attitude.bias.x-settled_bias.x) < .003f);
    assert(fabsf(s.attitude.bias.y-settled_bias.y) < .003f);
    assert(fabsf(s.attitude.bias.z-settled_bias.z) < .003f);
    for (unsigned i=0; i<6000; ++i) {
        now += TICK_MS;
        assert(feed(&s, now, rest, settled_bias, magnetic(170)));
        assert(angular_error(s.heading_deg, 0) < .1);
    }
    float origin = s.heading_origin_deg;
    P2_Vector saved_bias = s.attitude.bias;
    double peak = 0;
    for (unsigned i=1; i<=6000; ++i) {
        now += TICK_MS;
        double yaw = 170+.001*i; /* Deliberate real yaw: only 0.1 deg/s. */
        assert(feed(&s, now, rest, add(settled_bias, (P2_Vector){0,0,.1f}), magnetic(yaw)));
        double error = angular_error(s.heading_deg, yaw-170);
        if (error > peak) peak = error;
        assert(s.heading_origin_deg == origin);
        assert(memcmp(&saved_bias, &s.attitude.bias, sizeof(saved_bias)) == 0);
    }
    assert(peak < .12 && angular_error(s.heading_deg, 6) < .1);
    printf("PASS startup bias refinement, stationary drift, and real slow yaw (error %.4f deg)\n", peak);
}

static void inertial_interruption_between_magnetic_samples(void) {
    for (unsigned mode=0; mode<2; ++mode) {
        P2_Motion s;
        uint32_t now = warm_gyro(&s, 0);
        for (unsigned i=0; i<2U*window_ms()/(3U*TICK_MS); ++i) {
            now += TICK_MS;
            assert(!feed(&s, now, rest, bias, magnetic(31)));
        }
        now += TICK_MS;
        P2_Vector a = rest, g = bias;
        if (mode==0) g.z += 10;
        else a.z += 150;
        /* The inertial devices can publish faster than the magnetometer.
         * A detected disturbance cannot be forgotten before its next update. */
        P2_MotionGyro(&s, g, now);
        P2_MotionAcceleration(&s, a, now);
        uint32_t disturbed = now;
        while (now-disturbed < window_ms()-200U) {
            now += TICK_MS;
            assert(!feed(&s, now, rest, bias, magnetic(31)));
            pending(&s);
        }
        now = await_ready(&s, now, 31, bias);
        assert(angular_error(s.heading_origin_deg, 31) < .05);
    }
    puts("PASS inertial disturbances between magnetic samples restart the quiet window");
}

static void recovery_preserves_origin(void) {
    P2_Motion s;
    uint32_t now = warm_gyro(&s, 0);
    now = await_ready(&s, now, 64, bias);
    float origin = s.heading_origin_deg;
    P2_Vector saved_bias = s.attitude.bias;
    s.steps=17; s.distance_m=11.9f;
    /* A missing gyro makes heading temporarily unavailable. A complete fresh,
     * quiet window recovers the moved endpoint, never a new startup zero. */
    now += P2_GYRO_FRESH_MS+20U;
    uint32_t recovery_start = now;
    assert(!feed(&s, now, rest, bias, magnetic(84)));
    bool ready = false;
    while (now-recovery_start < P2_HEADING_RECOVERY_MS+200U) {
        now += TICK_MS;
        ready = feed(&s, now, rest, bias, magnetic(84));
        assert(s.heading_origin_deg == origin && s.heading_origin_set);
        assert(memcmp(&saved_bias, &s.attitude.bias, sizeof(saved_bias)) == 0);
        if(now-recovery_start<P2_HEADING_RECOVERY_MS)
            assert(!ready && s.attitude.mag_guard.reference_lost);
        else assert(ready && !s.attitude.mag_guard.reference_lost);
        assert(s.steps==17 && s.distance_m==11.9f);
    }
    assert(ready && fabsf(s.heading_deg-20)<.1f);
    assert(!s.attitude.mag_guard.reference_lost);
    puts("PASS post-boot inertial loss recovers automatically, preserving zero/bias/steps");
}

static void pedometer_independence(void) {
    P2_Motion aided, plain;
    P2_MotionInit(&aided); P2_MotionInit(&plain);
    P2_MotionRequireGyro(&aided);
    aided.mag_bias = (P2_Vector){0,0,0};
    aided.mag_scale = (P2_Vector){1,1,1};
    for (unsigned i=0; i<4000; ++i) {
        double t=.01*i;
        P2_Vector a = {0,0,(float)(1000+(t<10 ? 0 : 180*sin(4*3.141592653589793*t)))};
        (void)feed(&aided, i*TICK_MS, a, bias, magnetic(25));
        P2_MotionAcceleration(&plain, a, i*TICK_MS);
        assert(aided.steps == plain.steps);
        assert(aided.distance_m == plain.distance_m);
        assert(aided.baseline == plain.baseline && aided.filtered == plain.filtered);
        assert(aided.previous == plain.previous && aided.step_envelope == plain.step_envelope);
        assert(aided.pending_steps == plain.pending_steps && aided.armed == plain.armed);
    }
    assert(aided.steps > 40);
    puts("PASS startup changes leave deterministic pedometer state and counts identical");
}

int main(void) {
    cold_boot_asynchronous_clocks();
    sustained_moving_boot_is_not_forced();
    first_sample_outlier();
    magnetic_ramp();
    circular_mean_and_tick_wrap();
    duplicates_and_interrupted_windows();
    inertial_interruption_between_magnetic_samples();
    bias_refinement_and_slow_rotation();
    recovery_preserves_origin();
    pedometer_independence();
    puts("PASS: independent startup reference regression tests");
    return 0;
}
