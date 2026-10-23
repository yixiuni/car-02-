#include "car_control.h"
#include "car_config.h"
#include "imu_filter.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static CarGeometry Geometry(void)
{
    return (CarGeometry){65,120,1560,1560,125,125,75,70,true};
}
static CarControl car;
static CarInputs in;
static void Init(uint32_t time,bool held)
{
    CarGeometry g=Geometry();
    in=(CarInputs){.now_ms=time,.pressed=held,.enabled=true,.gray_online=true,
        .imu_ready=true,.encoder_valid=true};
    CarControl_Init(&car,&g,time,held);
}
static void Tick(unsigned ms)
{
    for (unsigned t=0;t<ms;t+=5) {
        in.now_ms+=5; ++in.gray_samples;
        in.left_mm+=in.left_speed*0.005f; in.right_mm+=in.right_speed*0.005f;
        CarControl_Update(&car,&in);
    }
}
static void Start(void)
{
    in.pressed=true; Tick(50); in.pressed=false; Tick(50);
}
int main(void)
{
    CarGeometry g=CarGeometry_Default();
    assert(!CarGeometry_Valid(&g)); /* delivered build must be parked */
    g=Geometry(); assert(CarGeometry_Valid(&g));
    g.counts_a_rev=NAN; assert(!CarGeometry_Valid(&g));
    assert(Encoder_CounterDelta(3,65530)==9);
    assert(Encoder_CounterDelta(65530,3)==-9);
    assert(Encoder_CounterDelta(0,32768)==-32768);
    MotionControl m={0}; uint16_t l,r;
    for(unsigned i=0;i<1000;i++) Motion_Update(&m,1000,1000,0,0,.005f,&l,&r);
    assert(l<=CAR_PWM_LIMIT && r<=CAR_PWM_LIMIT);
    Motion_Update(&m,0,0,200,200,.005f,&l,&r); assert(l==0 && r==0);
    Motion_Update(&m,NAN,100,0,0,.005f,&l,&r); assert(l==0 && r==0);

    Init(0,false); Start(); assert(car.state==CAR_STRAIGHT && car.point_events==1);
    in.left_speed=in.right_speed=200; Tick(1000);
    assert(car.state==CAR_STRAIGHT); /* planned white is not lost line */
    in.gray_online=false; Tick(5);
    assert(car.state==CAR_FAULT && car.fault==CAR_FAULT_GRAY && !car.left_pwm);
    in.gray_online=true; Tick(100); assert(car.state==CAR_FAULT);
    in.left_speed=in.right_speed=0; Start(); assert(car.state==CAR_STRAIGHT);
    in.pressed=true; Tick(35); assert(car.state==CAR_STOPPED);
    in.pressed=false; Tick(50); assert(car.state==CAR_STOPPED);

    Init(0,true); Tick(1000); assert(!CarControl_Active(&car));
    in.pressed=false; Tick(50); assert(!CarControl_Active(&car));
    Start(); assert(car.state==CAR_STRAIGHT);
    Init(0,false); in.pressed=true; Tick(1000);
    assert(car.task==2 && car.mode_events==1 && !CarControl_Active(&car));
    in.pressed=false; Tick(50); assert(!CarControl_Active(&car));
    Init(0,false); car.geometry.confirmed=false; Start();
    assert(car.fault==CAR_FAULT_CONFIG && !car.left_pwm);
    Init(0,false); in.enabled=false; Start(); in.enabled=true; Tick(100);
    assert(!CarControl_Active(&car));
    Init(0,false); in.enabled=false; in.pressed=true; Tick(30);
    in.enabled=true; in.pressed=false; Tick(50); assert(!CarControl_Active(&car));
    Init(0,false); Start(); in.enabled=false; Tick(5); in.enabled=true; Tick(100);
    assert(car.state==CAR_STOPPED);
    Init(0,false); Start(); in.now_ms+=100; CarControl_Update(&car,&in);
    assert(car.fault==CAR_FAULT_TIMING);
    Init(0,false); Start(); Tick(1500); assert(car.fault==CAR_FAULT_STALL);
    Init(UINT32_MAX-200,false); Start();
    in.left_speed=in.right_speed=200; Tick(1000); assert(car.state==CAR_STRAIGHT);

    /* Black away from the destination, including illegal FF, cannot advance. */
    Init(0,false); car.task=2; Start(); in.black_mask=0xFF;
    in.left_speed=in.right_speed=150; Tick(500); assert(car.segment==0);
    in.black_mask=0x18; Tick(500); assert(car.segment==0);
    /* Endpoint confirmations require new frames; repeated main-loop reads don't count. */
    car.x_mm=930; car.y_mm=0; car.black_frames=0;
    in.left_speed=in.right_speed=0; in.black_mask=0x18;
    for(unsigned i=0;i<10;i++) { in.now_ms+=5; CarControl_Update(&car,&in); }
    assert(car.segment==0);
    in.imu_ready=false; Tick(5); assert(car.fault==CAR_FAULT_IMU);

    ImuFilter f; ImuFilter_Init(&f);
    for(unsigned i=0;i<500;i++) ImuFilter_Update(&f,i*5,1.25f,true,true);
    assert(f.ready && fabsf(f.bias_dps-1.25f)<.001f);
    for(unsigned i=500;i<700;i++) ImuFilter_Update(&f,i*5,91.25f,true,false);
    assert(fabsf(f.yaw_rad-CAR_PI*.5f)<.01f);
    ImuFilter_Update(&f,3500,0,false,false); assert(!f.ready && !f.online);
    ImuFilter_Init(&f);
    for(unsigned i=0;i<1000;i++) ImuFilter_Update(&f,i*5,i%2 ? 4 : -4,true,true);
    assert(!f.ready); /* unstable bias rejected */
    puts("PASS: startup interlocks, buttons, white navigation, freshness, faults, wrap, PI, encoder wrap, gyro calibration");
}
