

#include "p2_motion.h"
#include "p2_config.h"
#include "p2_packet.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static const double rad = 0.017453292519943295;
static const P2_Vector field = {150,0,300};
static const P2_Vector bias = {.4f,-.25f,.8f};

static P2_Vector body(P2_Vector v, double yaw, double pitch, double roll) {
    double cy=cos(yaw*rad),sy=sin(yaw*rad),cp=cos(pitch*rad),sp=sin(pitch*rad);
    double cr=cos(roll*rad),sr=sin(roll*rad);
    double x=cy*v.x+sy*v.y,y=-sy*v.x+cy*v.y,z=v.z;
    double xx=cp*x-sp*z,zz=sp*x+cp*z;
    return (P2_Vector){(float)xx,(float)(cr*y+sr*zz),(float)(-sr*y+cr*zz)};
}
static P2_Vector omega(double pitch,double roll,double yd,double pd,double rd) {
    double cp=cos(pitch*rad),sp=sin(pitch*rad),cr=cos(roll*rad),sr=sin(roll*rad);
    return (P2_Vector){(float)(rd-yd*sp),(float)(pd*cr+yd*sr*cp),(float)(-pd*sr+yd*cr*cp)};
}
static P2_Vector plus(P2_Vector a,P2_Vector b) {
    return (P2_Vector){a.x+b.x,a.y+b.y,a.z+b.z};
}
static double angular_error(double a,double b) { return fabs(remainder(a-b,360)); }
static void basis_ok(const P2_Motion *s) {
    P2_Vector u=s->attitude.up,n=s->attitude.north;
    assert(fabs(u.x*u.x+u.y*u.y+u.z*u.z-1)<.00001);
    assert(fabs(n.x*n.x+n.y*n.y+n.z*n.z-1)<.00001);
    assert(fabs(u.x*n.x+u.y*n.y+u.z*n.z)<.00001);
}
static bool feed(P2_Motion *s,uint32_t now,P2_Vector a,P2_Vector g,P2_Vector m) {
    P2_MotionGyro(s,plus(g,bias),now);
    P2_MotionAcceleration(s,a,now);
    bool valid=P2_MotionHeading(s,m);
    if(valid) {
        assert(isfinite(s->heading_deg) && s->heading_deg>=0.0f && s->heading_deg<360.0f);
        int display=P2_HeadingDisplay(s->heading_deg,1,s->heading_status);
        assert(display>=0 && display<=359);
    }
    return valid;
}
static uint32_t settle(P2_Motion *s,uint32_t start,double yaw,double pitch,double roll) {
    P2_MotionInit(s);P2_MotionRequireGyro(s);
    s->mag_bias=(P2_Vector){0,0,0};s->mag_scale=(P2_Vector){1,1,1};
    for(unsigned i=0;i<=1000;i++) {
        bool valid=feed(s,start+10U*i,body((P2_Vector){0,0,1000},yaw,pitch,roll),
            (P2_Vector){0,0,0},body(field,yaw,pitch,roll));
        if(i*10U<P2_GYRO_CAL_MS+P2_STARTUP_BLOCK_MS*P2_STARTUP_BLOCKS) assert(!valid);
    }
    assert(s->attitude.calibrated && s->attitude.ready && s->heading_valid);
    assert(fabsf(s->attitude.bias.x-bias.x)<.00001f);
    assert(fabsf(s->attitude.bias.y-bias.y)<.00001f);
    assert(fabsf(s->attitude.bias.z-bias.z)<.00001f);
    assert(s->heading_origin_set && angular_error(s->heading_deg,0)<.05f);
    assert(P2_HeadingDisplay(s->heading_deg,1,s->heading_status)==0);
    basis_ok(s);
    return start+10000U;
}

static void dynamic_pose(void) {
    double max_error=0,max_up_error=0;
    for(int yaw0=0;yaw0<360;yaw0+=45) for(int moving=0;moving<2;moving++) {
        P2_Motion s;uint32_t start=settle(&s,0,yaw0,0,0);
        for(unsigned i=1;i<=2400;i++) {
            double t=i*.01,mid=t-.005,yaw=yaw0+(moving?20*t:0);
            double pitch=45*sin(1.3*t),roll=30*sin(.7*t);
            P2_Vector acc=body((P2_Vector){0,0,1000},yaw,pitch,roll);

            if(i%160>=80 && i%160<100)
                acc=plus(acc,body((P2_Vector){350,-150,140},yaw,pitch,roll));
            P2_Vector g=omega(45*sin(1.3*mid),30*sin(.7*mid),moving?20:0,
                58.5*cos(1.3*mid),21*cos(.7*mid));
            assert(feed(&s,start+i*10U,acc,g,body(field,yaw,pitch,roll)));
            double e=angular_error(s.heading_deg,yaw-yaw0);
            if(e>max_error)max_error=e;
            P2_Vector u=body((P2_Vector){0,0,1},yaw,pitch,roll);
            double d=u.x*s.attitude.up.x+u.y*s.attitude.up.y+u.z*s.attitude.up.z;
            double ue=acos(fmax(-1,fmin(1,d)))/rad;
            if(ue>max_up_error)max_up_error=ue;
            basis_ok(&s);
        }
        if(moving)assert(angular_error(s.heading_deg,120)<1.0);
    }
    printf("Fusion combined tilt/translation/yaw: max heading %.4f deg, up %.4f deg\n",max_error,max_up_error);
    assert(max_error<1.0 && max_up_error<1.0);
}

static void interference(void) {
    for(int mode=0;mode<2;mode++) {
        P2_Motion s;uint32_t start=settle(&s,100,47,0,0);
        float origin=s.heading_origin_deg;
        unsigned rejected=0;double max_error=0;
        for(unsigned i=1;i<=900;i++) {
            double t=i*.01,yaw=47+15*t;
            P2_Vector m=body(field,yaw,0,0);
            bool bad=i<=650;
            if(bad) {
                if(mode==0)m=(P2_Vector){m.x*1.8f,m.y*1.8f,m.z*1.8f};
                else m=body((P2_Vector){0,150,300},yaw,0,0);
            }
            bool valid=feed(&s,start+i*10U,(P2_Vector){0,0,1000},(P2_Vector){0,0,15},m);
            assert(valid);
            if(bad) { assert(!s.attitude.magnetic_accepted); ++rejected; }

            if(i>650 && i<=850) assert(!s.attitude.magnetic_accepted);
            if(i>850) assert(s.attitude.magnetic_accepted);
            if(valid) {
                double e=angular_error(s.heading_deg,yaw-47);
                if(e>max_error)max_error=e;
            }
            assert(s.heading_origin_deg==origin);
        }
        printf("Fusion magnetic %s rejection: %u rejected magnetic samples, continuous heading error %.4f\n",
            mode?"direction-only":"strength",rejected,max_error);
        assert(max_error<.08);
    }
}

static void vertical_and_turns(void) {
    P2_Motion s;uint32_t now=settle(&s,0,13,0,0);
    unsigned invalid=0;
    for(unsigned i=1;i<=400;i++) {
        double pitch=i<=200?i*.45:180-i*.45;
        double rate=i<=200?45:-45;
        bool valid=feed(&s,now+i*10U,body((P2_Vector){0,0,1000},13,pitch,0),
            (P2_Vector){0,(float)rate,0},body(field,13,pitch,0));
        if(pitch>75){assert(!valid);++invalid;}
        if(pitch<70){assert(valid);assert(angular_error(s.heading_deg,0)<.3);}
    }
    assert(invalid>40);now+=4000;
    for(int sign=-1;sign<=1;sign+=2) {
        now=settle(&s,now+100,137,0,0);
        for(unsigned i=1;i<=800;i++) {
            double yaw=137+sign*.9*i;
            assert(feed(&s,now+i*10U,(P2_Vector){0,0,1000},
                (P2_Vector){0,0,sign*90.0f},body(field,yaw,0,0)));
            assert(angular_error(s.heading_deg,yaw-137)<.06);
            if(i%100==0) {
                int expected=(sign*(int)(i/100U)*90)%360;
                if(expected<0)expected+=360;
                assert(P2_HeadingDisplay(s.heading_deg,1,s.heading_status)==expected);
                assert(P2_HeadingWire(s.heading_deg,1,s.heading_status)==expected*10);
            }
            if(i%400==0)assert(angular_error(s.heading_deg,0)<.05f);
        }
        now+=8000;
    }
    puts("PASS fusion near-vertical invalidity/return; CW 90=90, CCW 90=270, full-turn zero");
}

static void time_and_startup(void) {
    P2_Motion s;P2_MotionInit(&s);P2_MotionRequireGyro(&s);
    s.mag_bias=(P2_Vector){0,0,0};s.mag_scale=(P2_Vector){1,1,1};

    for(unsigned i=0;i<300;i++)
        assert(!feed(&s,i*10U,(P2_Vector){0,0,1000},(P2_Vector){0,0,20},body(field,i*.2,0,0)));
    assert(!s.attitude.calibrated);
    uint32_t start=settle(&s,UINT32_MAX-1500U,83,0,0);
    uint32_t count=s.attitude.count;P2_Vector up=s.attitude.up,north=s.attitude.north;
    for(unsigned i=0;i<100;i++)P2_MotionGyro(&s,(P2_Vector){100,100,100},start);
    assert(s.attitude.count==count);
    assert(s.attitude.up.x==up.x && s.attitude.up.y==up.y && s.attitude.up.z==up.z);
    assert(s.attitude.north.x==north.x && s.attitude.north.y==north.y && s.attitude.north.z==north.z);
    for(unsigned i=1;i<=200;i++)
        assert(feed(&s,start+i*10U,(P2_Vector){0,0,1000},(P2_Vector){0,0,20},body(field,83+i*.2,0,0)));
    assert(angular_error(s.heading_deg,40)<.06);

    P2_MotionAcceleration(&s,(P2_Vector){0,0,1000},start+2200U);
    assert(!P2_MotionHeading(&s,body(field,123,0,0)));

    P2_MotionGyro(&s,(P2_Vector){NAN,0,0},start+2200U);
    assert(!s.heading_valid);
    assert(!P2_MotionHeading(&s,body(field,123,0,0)));
    puts("PASS fusion moving startup, bias, duplicate ticks, tick wrap and stale/invalid gyro");
}

static P2_Vector cross(P2_Vector a,P2_Vector b) {
    return (P2_Vector){a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
static P2_Vector polar_swap(P2_Vector v) { return (P2_Vector){v.y,v.x,v.z}; }
static void frame_mapping(void) {

    const P2_Vector vectors[]={{1,2,3},{-.2f,.5f,.9f},{0,0,1}};
    const P2_Vector rates[]={{90,0,0},{0,90,0},{0,0,90},{3,7,-11}};
    for(unsigned i=0;i<sizeof(vectors)/sizeof(vectors[0]);i++)
        for(unsigned j=0;j<sizeof(rates)/sizeof(rates[0]);j++) {
            P2_Vector lhs=polar_swap(cross(vectors[i],rates[j]));
            P2_Vector rhs=cross(polar_swap(vectors[i]),P2_GyroToBody(rates[j]));
            assert(fabsf(lhs.x-rhs.x)<.00001f);
            assert(fabsf(lhs.y-rhs.y)<.00001f);
            assert(fabsf(lhs.z-rhs.z)<.00001f);
        }
    puts("PASS gyro axial mapping preserves polar-vector transport under reflected frame");
}

static void gap_recovery(void) {
    P2_Motion s;uint32_t now=settle(&s,0,47,0,0);
    for(unsigned i=1;i<=100;i++)
        assert(feed(&s,now+i*10U,(P2_Vector){0,0,1000},(P2_Vector){0,0,30},body(field,47+i*.3,0,0)));
    now+=1000;
    float origin=s.heading_origin_deg,reference=s.attitude.field_norm,dip=s.attitude.dip;
    P2_Vector clean=body(field,77,0,0),bad={clean.x*1.8f,clean.y*1.8f,clean.z*1.8f};
    s.steps=17; s.distance_m=11.9f;

    now+=100;
    for(unsigned i=0;i<100;i++) {
        assert(!feed(&s,now+i*10U,(P2_Vector){0,0,1000},(P2_Vector){0,0,0},bad));
        assert(!s.attitude.ready && !s.attitude.magnetic_accepted);
        assert(s.attitude.field_norm==reference && s.attitude.dip==dip);
        assert(s.heading_origin_deg==origin);
    }
    now+=1000;
    for(unsigned i=0;i<30;i++)
        assert(!feed(&s,now+i*10U,(P2_Vector){0,0,1000},(P2_Vector){0,0,0},clean));

    now+=300;
    assert(!feed(&s,now,(P2_Vector){0,0,1300},(P2_Vector){0,0,0},clean));
    now+=10;
    for(unsigned i=0;i<=220;i++) {
        bool valid=feed(&s,now+i*10U,(P2_Vector){0,0,1000},(P2_Vector){0,0,0},clean);
        if(i*10U<P2_HEADING_RECOVERY_MS) assert(!valid);
        else assert(valid && !s.attitude.mag_guard.reference_lost);
        assert(s.heading_origin_deg==origin);
        assert(s.steps==17 && s.distance_m==11.9f);
    }
    assert(angular_error(s.heading_deg,30)<.06);
    assert(s.attitude.field_norm==reference && s.attitude.dip==dip);
    assert(s.heading_valid);
    puts("PASS inertial gap reacquires after uninterrupted quiet data, preserving zero/steps/field");
}

static P2_Vector rotate_truth(P2_Vector v,P2_Vector w,double dt) {
    double speed=sqrt((double)w.x*w.x+(double)w.y*w.y+(double)w.z*w.z);
    double ax=w.x/speed,ay=w.y/speed,az=w.z/speed;
    double angle=-speed*dt*rad,c=cos(angle),sn=sin(angle);
    double d=(ax*v.x+ay*v.y+az*v.z)*(1-c);
    return (P2_Vector){(float)(v.x*c+(ay*v.z-az*v.y)*sn+ax*d),
        (float)(v.y*c+(az*v.x-ax*v.z)*sn+ay*d),
        (float)(v.z*c+(ax*v.y-ay*v.x)*sn+az*d)};
}
static void high_rate_turns_and_variable_field(void) {
    const int rates[]={600,1000};
    for(unsigned r=0;r<2;r++) for(int sign=-1;sign<=1;sign+=2) {
        P2_Motion s; uint32_t now=settle(&s,0,13,0,0);
        unsigned samples=72000U/(unsigned)rates[r],rejected=0;
        for(unsigned i=1;i<=samples;i++) {
            now+=10;
            double turn=sign*rates[r]*.01*i,yaw=13+turn;
            P2_Vector m=body(field,yaw,0,0);
            float scale=(float)(1+.25*sin(turn*rad));
            m=(P2_Vector){m.x*scale,m.y*scale,m.z*scale};
            assert(feed(&s,now,(P2_Vector){0,0,1000},(P2_Vector){0,0,(float)(sign*rates[r])},m));
            assert(!s.attitude.mag_guard.reference_lost);
            assert(angular_error(s.heading_deg,turn)<.1);
            if(!s.attitude.magnetic_accepted) ++rejected;
            if(i%(samples/2)==0) {
                assert(angular_error(s.heading_deg,0)<.05f);
                assert(P2_HeadingDisplay(s.heading_deg,1,s.heading_status)==0);
            }
        }
        assert(rejected>0 && angular_error(s.heading_deg,0)<.05f);
    }
    P2_Motion s; uint32_t now=settle(&s,0,0,0,0);
    P2_Vector rate={1500,1500,0};
    P2_Vector a=rotate_truth((P2_Vector){0,0,1000},rate,.01);
    assert(feed(&s,now+10,a,rate,rotate_truth(field,rate,.01)));
    assert(!s.attitude.mag_guard.reference_lost && s.attitude.ready);
    assert(fabsf(s.attitude.up.x-a.x/1000)<.0001f);
    assert(fabsf(s.attitude.up.y-a.y/1000)<.0001f);
    assert(fabsf(s.attitude.up.z-a.z/1000)<.0001f);

    P2_MotionGyro(&s,(P2_Vector){P2_GYRO_MAX_DPS+1,0,0},now+20);
    assert(!s.heading_valid && s.attitude.mag_guard.reference_lost);
    now=settle(&s,now+100,0,0,0);
    for(unsigned axis=0;axis<3;axis++) for(int sign=-1;sign<=1;sign+=2) {
        P2_Motion endpoint=s;
        P2_Vector raw={0,0,0};
        if(axis==0) raw.x=sign*P2_GYRO_MAX_DPS;
        if(axis==1) raw.y=sign*P2_GYRO_MAX_DPS;
        if(axis==2) raw.z=sign*P2_GYRO_MAX_DPS;
        P2_MotionGyro(&endpoint,raw,now+10);
        assert(endpoint.attitude.ready && !endpoint.attitude.mag_guard.reference_lost);
        if(axis==0) raw.x+=sign;
        if(axis==1) raw.y+=sign;
        if(axis==2) raw.z+=sign;
        P2_MotionGyro(&endpoint,raw,now+20);
        assert(!endpoint.heading_valid && endpoint.attitude.mag_guard.reference_lost);
    }
    puts("PASS 600/1000 dps unsigned full turns, +/-25% field variation and per-axis gyro range");
}

int main(void) {
    dynamic_pose();interference();vertical_and_turns();time_and_startup();
    frame_mapping();gap_recovery();high_rate_turns_and_variable_field();
    puts("PASS: independent gyro-aided fusion tests (hardware alignment still requires measurement)");
    return 0;
}
