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
/* A centered exit edge may correct modest accumulated odometry error,
 * but must not snap an unrelated near-endpoint pose onto the course. */
static void ArcExit(float backward_error, float sideways_error)
{
    Init(0,false); car.geometry=CarGeometry_Default();
    car.geometry.counts_a_rev=car.geometry.counts_b_rev=1560;
    car.geometry.confirmed=true;
    car.task=2; Start();
    car.state=CAR_ARC; car.segment=3;
    float heading=asinf(car.geometry.sensor_forward_mm/400.0f);
    car.heading_rad=in.yaw_rad=heading; car.yaw_offset=0;
    car.arc_heading_start=CAR_PI;
    car.x_mm=-(car.geometry.sensor_forward_mm+backward_error)*cosf(heading)
             -sideways_error*sinf(heading);
    car.y_mm=-(car.geometry.sensor_forward_mm+backward_error)*sinf(heading)
             +sideways_error*cosf(heading);
    car.distance_mm=1100; car.segment_distance=0;
    car.have_line=true; car.white_frames=2; car.last_line_at=in.now_ms;
    car.line_error=0; in.black_mask=0;
    Tick(5);
}
/* Reconstructed early B white-edge pose from cross run 02 (sparse trace),
 * with fresh confirmation frames. Checks the bounded correction, not a
 * claim that logged odometry is ground truth. */
static void CrossBEdge(float extra_error, bool wrong_heading, bool all_black)
{
    Init(0,false); car.geometry=CarGeometry_Default();
    car.geometry.confirmed=true;car.task=3;Start();
    car.state=CAR_ARC;car.segment=1;
    car.heading_rad=in.yaw_rad=wrong_heading ? 0 : 2.6851158f;car.yaw_offset=0;
    car.arc_heading_start=-0.6222511f;
    car.x_mm=1131.4813f;car.y_mm=-152.2937f-extra_error;
    car.distance_mm=1036;car.segment_distance=0;
    car.have_line=true;car.white_frames=2;car.last_line_at=in.now_ms;
    car.line_error=7.5f;in.black_mask=all_black ? 0xFF : 0;
    Tick(5);
}
static float LateArcSpeed(unsigned task,unsigned laps)
{
    Init(0,false);car.task=task;Start();
    car.state=CAR_ARC;car.segment=3;car.completed_laps=laps;
    car.heading_rad=in.yaw_rad=.4f;car.yaw_offset=0;car.arc_heading_start=CAR_PI;
    car.distance_mm=1100;car.segment_distance=0;car.have_line=true;
    car.line_error=0;in.black_mask=0x18;Tick(5);
    assert(car.state==CAR_ARC);
    return (car.left_target+car.right_target)*.5f;
}
static void RecentArcEdge(unsigned task,uint32_t time,bool observed)
{
    Init(time,false);car.task=task;Start();
    car.state=CAR_ARC;car.segment=1;car.x_mm=1200;car.y_mm=-600;
    car.heading_rad=in.yaw_rad=1.2f;car.yaw_offset=0;car.arc_heading_start=0;
    car.distance_mm=500;car.segment_distance=0;car.have_line=observed;
    car.line_error=-45;car.line_rate=100000; /* stale derivative must be ignored */
    car.line_missing=false;car.last_line_at=in.now_ms;in.black_mask=0;
    float speed=task==4 ? CAR_FOUR_CRUISE_SPEED : CAR_ARC_SPEED;
    float base=speed/400;
    float correction=task==4 && observed ? -(speed/180)*CAR_LINE_KP*car.line_error : 0;
    Tick(5);
    assert(car.state==CAR_ARC);
    assert(fabsf((car.right_target-car.left_target)/car.geometry.track_mm-base-correction)<.001f);
    Tick(75); /* last known edge is valid at 80 ms, including tick wrap */
    assert(fabsf((car.right_target-car.left_target)/car.geometry.track_mm-base-correction)<.001f);
    Tick(5); /* stale observation expires; no extra loss-time allowance */
    assert(fabsf((car.right_target-car.left_target)/car.geometry.track_mm-base)<.001f);
    Tick(100);
    assert(car.state==CAR_FAULT && car.fault==CAR_FAULT_LINE && !car.left_pwm && !car.right_pwm);
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
    Motion_UpdateWithAccel(&m,240,240,0,0,.005f,1000,&l,&r);
    assert(fabsf(m.left.setpoint-5.0f)<.001f && fabsf(m.right.setpoint-5.0f)<.001f);
    Motion_UpdateWithAccel(&m,240,240,0,0,.005f,NAN,&l,&r);
    assert(l==0 && r==0 && m.left.setpoint==0 && m.right.setpoint==0);
    Motion_UpdateWithAccel(&m,240,240,0,0,.005f,0,&l,&r);
    assert(l==0 && r==0);

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
    Init(0,false); Start(); in.right_speed=150; Tick(1500);
    assert(car.fault==CAR_FAULT_STALL && !car.left_pwm && !car.right_pwm);
    Init(0,false); Start(); in.left_speed=150; Tick(1500);
    assert(car.fault==CAR_FAULT_STALL && !car.left_pwm && !car.right_pwm);
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

    ArcExit(25,0);
    assert(car.state==CAR_BRAKING && car.completed_laps==1);
    ArcExit(0,80);
    assert(car.state==CAR_ARC && car.completed_laps==0 && car.point_events==1);
    ArcExit(25,0); /* corrected pose cannot signal until the wheels settle */
    Tick(130); assert(car.state==CAR_DONE && car.point_events==2);

    CrossBEdge(0,false,false);
    assert(car.state==CAR_STRAIGHT && car.segment==2 && car.last_point==CAR_POINT_B);
    /* beyond the CROSS correction limit -> the qualified edge must be rejected */
    CrossBEdge(CAR_CROSS_ENDPOINT_CORRECTION_MM+20,false,false);
    assert(car.state==CAR_ARC && car.segment==1 && car.point_events==1);
    Tick(200);assert(car.state==CAR_FAULT && car.fault==CAR_FAULT_LINE);
    CrossBEdge(0,true,false);
    assert(car.state==CAR_ARC && car.segment==1);
    CrossBEdge(0,false,true);
    assert(car.state==CAR_ARC && car.segment==1);

    Init(0,false);car.task=3;Start();
    assert(fabsf(car.heading_rad)<.001f && car.left_target>car.right_target &&
           car.right_target>=0); /* face B, automatic forward-only right turn */

    assert(fabsf(LateArcSpeed(3,0)-CAR_ARC_SPEED)<.01f);
    assert(fabsf(LateArcSpeed(4,2)-CAR_FOUR_CRUISE_SPEED)<.01f);
    assert(fabsf(LateArcSpeed(4,3)-CAR_FOUR_CRUISE_SPEED)<.01f);
    RecentArcEdge(4,0,true);
    RecentArcEdge(4,UINT32_MAX-160U,true);
    RecentArcEdge(4,0,false);
    RecentArcEdge(3,0,true);
    Init(0,false);Start();car.x_mm=880;in.black_mask=0;Tick(5);
    assert(fabsf((car.left_target+car.right_target)*.5f-CAR_STRAIGHT_SPEED)<.01f);
    for(unsigned wrap=0;wrap<2;wrap++) {
        uint32_t at=wrap ? UINT32_MAX-50U : 100U;
        CarControl stopped={.state=CAR_BRAKING,.fault=CAR_OK,.brake_at=at};
        assert(CarControl_BrakeActive(&stopped,at));
        assert(CarControl_BrakeActive(&stopped,at+CAR_BRAKE_HOLD_MS-1U));
        assert(!CarControl_BrakeActive(&stopped,at+CAR_BRAKE_HOLD_MS));
        stopped.state=CAR_DONE;assert(!CarControl_BrakeActive(&stopped,at));
        stopped.state=CAR_STOPPED;assert(!CarControl_BrakeActive(&stopped,at));
        stopped.state=CAR_FAULT;assert(!CarControl_BrakeActive(&stopped,at));
        stopped.state=CAR_BRAKING;stopped.fault=CAR_FAULT_GRAY;
        assert(!CarControl_BrakeActive(&stopped,at));
    }
    ArcExit(25,0);assert(CarControl_BrakeActive(&car,in.now_ms));
    in.enabled=false;Tick(5);
    assert(!CarControl_BrakeActive(&car,in.now_ms) && !car.left_pwm && !car.right_pwm);

    const float reference=CAR_GYRO_Z_BIAS_PRIOR_DPS;
    const float stationary=reference+1.25f; /* bias must be measured, not forced to the prior */
    ImuFilter f; ImuFilter_Init(&f);
    for(unsigned i=0;i<500;i++) ImuFilter_Update(&f,i*5,stationary,true,true);
    assert(f.ready && fabsf(f.bias_dps-stationary)<.001f);
    for(unsigned i=500;i<700;i++) ImuFilter_Update(&f,i*5,stationary+90,true,false);
    assert(fabsf(f.yaw_rad-CAR_PI*.5f)<.01f);
    ImuFilter_Update(&f,3500,0,false,false); assert(!f.ready && !f.online);
    ImuFilter_Init(&f);
    for(unsigned i=0;i<1000;i++) ImuFilter_Update(&f,i*5,reference+(i%2 ? 4 : -4),true,true);
    assert(!f.ready); /* unstable bias rejected even when its mean equals the reference */
    ImuFilter_Init(&f);
    for(unsigned i=0;i<500;i++) ImuFilter_Update(&f,i*5,reference+15,true,true);
    assert(!f.ready && f.count==0); /* constant turn outside reference window rejected */
    ImuFilter_Init(&f);
    for(unsigned i=0;i<500;i++) ImuFilter_Update(&f,i*5,stationary,true,false);
    assert(!f.ready && f.count==0); /* wheel motion blocks zero calibration */
    ImuFilter_Init(&f);
    for(unsigned i=0;i<200;i++) ImuFilter_Update(&f,i*5,stationary,true,true);
    assert(f.count>0 && !f.ready);
    ImuFilter_Update(&f,1000,reference+15,true,true);
    assert(f.count==0 && !f.ready);
    for(unsigned i=201;i<500;i++) ImuFilter_Update(&f,i*5,stationary,true,true);
    assert(!f.ready); /* interrupted windows cannot combine */
    for(unsigned i=500;i<700;i++) ImuFilter_Update(&f,i*5,stationary,true,true);
    assert(f.ready);
    ImuFilter_Update(&f,3600,stationary,true,true);
    assert(!f.ready && !f.online); /* stale heading still invalidates calibration */
    ImuFilter_Update(&f,3605,NAN,true,true); assert(!f.ready && !f.online);
    puts("PASS: startup interlocks, buttons, white navigation, freshness, faults, wrap, PI, encoder wrap, gyro calibration");
}
