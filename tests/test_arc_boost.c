#include "car_control.h"
#include "car_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static CarControl c;
static CarInputs in;
static void InitD(float x,float y)
{
    CarGeometry g=CarGeometry_Default();g.confirmed=true;
    CarControl_Init(&c,&g,0,false);
    c.task=4;c.target_laps=4;c.state=CAR_STRAIGHT;c.segment=2;
    c.x_mm=x;c.y_mm=y;c.heading_rad=atan2f(-800,-1000);
    in=(CarInputs){.enabled=true,.gray_online=true,.imu_ready=true,
        .encoder_valid=true,.yaw_rad=c.heading_rad,.left_speed=800,.right_speed=800};
}
static float Step(uint8_t mask,bool fresh)
{
    in.now_ms+=5;in.black_mask=mask;if(fresh)++in.gray_samples;
    CarControl_Update(&c,&in);assert(c.fault==CAR_OK);
    return (c.left_target+c.right_target)*.5f;
}
static void InitArc(float progress)
{
    InitD(1200,-400);c.state=CAR_ARC;c.segment=1;
    c.distance_mm=progress;c.segment_distance=0;
    c.arc_heading_start=c.heading_rad;c.have_line=true;
}
int main(void)
{
    assert(CAR_TIME_TRIAL_ARC_BOOST && CAR_CROSS_D_ENTRY_PREVIEW);
    assert(CAR_TT_STRAIGHT_SPEED==850 && CAR_TT_ARC_SPEED==550);
    assert(CAR_TT_ARC_ACCEL_START_MM==165 && CAR_TT_ARC_PWM_RESERVE==900);
    /* D preview outside the footprint steers, but cannot report D early. */
    InitD(150,-680);Step(0x80,true);assert(c.right_target<c.left_target);
    Step(0x80,true);Step(0x80,true);
    assert(c.black_frames==3 && c.point_events==0 && c.state==CAR_STRAIGHT);
    Step(0x80,false);assert(c.black_frames==3 && c.point_events==0);
    Step(0,true);float white=c.right_target-c.left_target;
    InitD(150,-680);Step(0,true);assert(fabsf(c.right_target-c.left_target-white)<.01f);
    InitD(150,-680);Step(0xff,true);assert(c.point_events==0 && c.black_frames==0);
    InitD(500,-400);Step(0,true);float far=c.right_target-c.left_target;
    InitD(500,-400);Step(0x80,true);assert(fabsf(c.right_target-c.left_target-far)<.01f);
    InitD(120,-704);Step(0x18,true);Step(0x18,false);assert(c.point_events==0);
    Step(0x18,true);assert(c.point_events==0);Step(0x18,true);
    assert(c.state==CAR_ARC && c.segment==3 && c.last_point==CAR_POINT_D && c.point_events==1);
    /* An aligned line must not accelerate before the front/axle offset clears. */
    InitArc(100);assert(fabsf(Step(0x18,true)-370)<.01f);
    InitArc(165);assert(fabsf(Step(0x18,true)-370)<.01f);
    InitArc(200);float ramp=Step(0x18,true);assert(ramp>370 && ramp<550);
    InitArc(300);assert(fabsf(Step(0x18,true)-550)<.01f);
    /* Arc output allocation limits demand while retaining intended curvature. */
    InitArc(300);in.left_speed=250;in.right_speed=300;
    c.motion.left.integral=300;c.motion.right.integral=350;
    float v=Step(0x18,true);assert(v>0 && v<550);
    float curvature=(c.right_target-c.left_target)/c.geometry.track_mm/v;
    float radius=sqrtf(400*400-c.geometry.sensor_forward_mm*c.geometry.sensor_forward_mm);
    assert(fabsf(curvature-1/radius)<.00001f);
    float left=(CAR_SPEED_FF+CAR_SPEED_KP)*c.left_target-CAR_SPEED_KP*250+300;
    float right=(CAR_SPEED_FF+CAR_SPEED_KP)*c.right_target-CAR_SPEED_KP*300+350;
    assert(left<=900.01f && right<=900.01f);
    in.gray_online=false;in.now_ms+=5;CarControl_Update(&c,&in);
    assert(c.state==CAR_FAULT && c.fault==CAR_FAULT_GRAY && !c.left_pwm && !c.right_pwm);
    puts("PASS: D preview fresh footprint gates, 165mm arc acquisition, cruise release, arc PI budget and curvature, fault stop");
}
