#include "car_control.h"
#include "car_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static CarControl c;
static CarInputs in;
static void Init(float x,float y)
{
    CarGeometry g=CarGeometry_Default();g.confirmed=true;
    CarControl_Init(&c,&g,0,false);
    c.task=4;c.target_laps=4;c.state=CAR_STRAIGHT;
    c.x_mm=x;c.y_mm=y;c.heading_rad=atan2f(-800,1000);
    in=(CarInputs){.enabled=true,.gray_online=true,.imu_ready=true,
        .encoder_valid=true,.yaw_rad=c.heading_rad};
}
static float Step(uint8_t mask,bool fresh)
{
    in.now_ms+=5;in.black_mask=mask;if(fresh)++in.gray_samples;
    CarControl_Update(&c,&in);
    assert(c.fault==CAR_OK);
    return c.right_target-c.left_target;
}
int main(void)
{
    assert(CAR_TIME_TRIAL_C_ENTRY);
    assert(CAR_TT_STRAIGHT_SPEED==(CAR_TIME_TRIAL_FAST ? 650 : CAR_TIME_TRIAL_STRAIGHT_BOOST ? 520 : 500));
    assert(CAR_CROSS_C_ENTRY_AIM_MM==(CAR_TIME_TRIAL_FAST ? 25 : 20));
    if(CAR_TIME_TRIAL_FAST) assert(CAR_FOUR_PWM_LIMIT==1000 && CAR_TT_WHEEL_SPEED==850);
    assert(CAR_TT_ARC_SPEED==430 && CAR_TT_ENTRY_SPEED==370);
    assert(CAR_CROSS_D_ENTRY_AIM_MM==20 && !CAR_TIME_TRIAL_STAGE3);
    /* C is 192 mm ahead of the axle: within approach gate, beyond the
     * 170 mm front bumper. Preview may steer but must not signal C. */
    Init(850,-680);assert(Step(1,true)>0);
    Step(1,true);Step(1,true);
    assert(c.black_frames==3 && c.point_events==0 && c.state==CAR_STRAIGHT);
    for(unsigned i=0;i<5;i++)Step(1,false);
    assert(c.point_events==0 && c.black_frames==3);
    /* White/all-black cancel preview even after an outer-edge observation. */
    float cancelled=Step(0,true);
    assert(c.black_frames==0 && c.point_events==0);
    Init(850,-680);float white=Step(0,true);
    assert(fabsf(cancelled-white)<.01f);
    Init(850,-680);assert(fabsf(Step(0xff,true)-white)<.01f);
    assert(c.point_events==0);
    /* Black outside the approach window cannot cause preview or a point. */
    Init(500,-400);float planned=Step(0,true);
    assert(planned>0); /* white path aims along the arc beyond C */
    Init(500,-400);assert(fabsf(Step(1,true)-planned)<.01f);
    assert(c.point_events==0);
    /* Once the body covers actual C, still require three fresh frames. */
    Init(880,-704);Step(0x18,true);
    for(unsigned i=0;i<5;i++)Step(0x18,false);
    assert(c.point_events==0 && c.black_frames==1);
    Step(0x18,true);assert(c.point_events==0);
    Step(0x18,true);
    assert(c.point_events==1 && c.last_point==CAR_POINT_C);
    assert(c.segment==1 && c.state==CAR_ARC);
    /* Passing C on white remains a fault, never an odometry-only success. */
    Init(1100,-880);in.now_ms=5;++in.gray_samples;
    CarControl_Update(&c,&in);
    assert(c.state==CAR_FAULT && c.fault==CAR_FAULT_ENDPOINT);
    assert(c.point_events==0 && !c.left_pwm && !c.right_pwm);
    puts("PASS: C preview before footprint, no early event, white/ambiguous cancellation, range and fresh-frame gates, fixed baseline speeds");
}
