#include "car_control.h"
#include "car_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static CarControl c;
static CarInputs in;
static void Setup(void)
{
    CarGeometry g=CarGeometry_Default();g.confirmed=true;
    CarControl_Init(&c,&g,0,false);
    c.task=4;c.target_laps=4;c.state=CAR_STRAIGHT;
    c.x_mm=880;c.y_mm=-704;c.heading_rad=atan2f(-800,1000);
    in=(CarInputs){.enabled=true,.gray_online=true,.imu_ready=true,
        .encoder_valid=true,.yaw_rad=c.heading_rad,.left_speed=900,.right_speed=900};
}
static void Step(unsigned mask,bool fresh)
{
    in.now_ms+=5;in.black_mask=mask;if(fresh)++in.gray_samples;
    CarControl_Update(&c,&in);
    assert(c.fault==CAR_OK);
    assert(c.left_target>=0 && c.right_target>=0);
    assert(c.left_target<=CAR_TT_WHEEL_SPEED+.01f && c.right_target<=CAR_TT_WHEEL_SPEED+.01f);
}
int main(void)
{
    assert(CAR_TIME_TRIAL_CROSS_SPRINT);
    /* A rightmost-only hit must still steer inward, without claiming C. */
    Setup();for(unsigned i=0;i<3;i++)Step(0x80,true);
    assert(c.state==CAR_STRAIGHT && c.point_events==0 && c.black_frames==3);
    assert(c.right_target<c.left_target);
    for(unsigned i=0;i<10;i++)Step(0x80,false);
    assert(c.point_events==0 && c.black_frames==3);
    Step(0,true);assert(c.point_events==0 && c.black_frames==0);
    Step(0xff,true);assert(c.point_events==0 && c.black_frames==0);
    Step(0x18,true);Step(0x18,true);assert(c.point_events==0);
    Step(0x18,true);assert(c.point_events==1 && c.last_point==CAR_POINT_C && c.state==CAR_ARC);
    /* The other side is a valid different entry geometry, not a symmetric gate. */
    Setup();for(unsigned i=0;i<3;i++)Step(1,true);
    assert(c.point_events==1 && c.state==CAR_ARC);
    /* D mirrors the unsafe edge, and previews before axle footprint coverage. */
    Setup();c.segment=2;c.x_mm=150;c.y_mm=-680;
    c.heading_rad=in.yaw_rad=atan2f(-800,-1000);
    Step(1,true);assert(c.point_events==0 && c.right_target>c.left_target);
    c.x_mm=120;c.y_mm=-704;Step(1,true);Step(1,true);
    assert(c.black_frames==3 && c.point_events==0 && c.state==CAR_STRAIGHT);
    Step(0x18,true);
    assert(c.point_events==1 && c.last_point==CAR_POINT_D && c.state==CAR_ARC);
    /* Black away from C or on white beyond C cannot manufacture a point. */
    Setup();c.x_mm=500;c.y_mm=-400;
    for(unsigned i=0;i<4;i++)Step(0x18,true);
    assert(c.point_events==0 && c.state==CAR_STRAIGHT);
    Setup();c.x_mm=1100;c.y_mm=-880;in.now_ms=5;
    CarControl_Update(&c,&in);
    assert(c.state==CAR_FAULT && c.fault==CAR_FAULT_ENDPOINT && !c.left_pwm && !c.right_pwm);
    /* Cruise tuning applies only after acquisition and only to CROSS. */
    for(unsigned normal=0;normal<2;normal++) for(unsigned early=0;early<2;early++) {
        Setup();c.task=normal ? 2 : 4;c.state=CAR_ARC;c.segment=1;
        c.x_mm=1200;c.y_mm=-400;c.heading_rad=0;in.yaw_rad=0;
        c.distance_mm=early ? 100 : 250;c.have_line=true;
        c.line_error=15;c.line_rate=400;c.last_line_at=0;
        Step(0x30,false);
        float v=(c.left_target+c.right_target)*.5f;
        float ps=!normal && !early ? CAR_CROSS_SPRINT_P_SCALE : 1;
        float ds=!normal && !early ? CAR_CROSS_SPRINT_D_SCALE : 1;
        float radius=sqrtf(400*400-CAR_SENSOR_FORWARD_MM*CAR_SENSOR_FORWARD_MM);
        float omega=(normal ? -1 : 1)*v/radius-
            ps*(v/180)*CAR_LINE_KP*15-ds*(CAR_FOUR_CRUISE_SPEED/180)*CAR_LINE_KD*400;
        assert(fabsf((c.right_target-c.left_target)/CAR_WHEEL_TRACK_MM-omega)<.001f);
        if(normal)assert(fabsf(v-460)<.01f);
        else if(!early)assert(fabsf(v-650)<.01f);
    }
    Setup();c.state=CAR_ARC;c.segment=1;c.x_mm=1200;c.y_mm=-400;
    c.distance_mm=250;c.have_line=true;c.line_error=0;
    c.motion.left.integral=400;c.motion.right.integral=400;
    in.left_speed=300;in.right_speed=300;Step(0x18,false);
    assert((c.left_target+c.right_target)*.5f<650);
    float pl=(CAR_SPEED_FF+CAR_SPEED_KP)*c.left_target-CAR_SPEED_KP*300+400;
    float pr=(CAR_SPEED_FF+CAR_SPEED_KP)*c.right_target-CAR_SPEED_KP*300+400;
    assert(pl<=CAR_CROSS_SPRINT_ARC_PWM+.01f && pr<=CAR_CROSS_SPRINT_ARC_PWM+.01f);
    in.enabled=false;Step(0,false);
    assert(c.state==CAR_STOPPED && !c.left_pwm && !c.right_pwm);
    puts("PASS: CROSS sprint C observation gate, fresh frames, preview, asymmetric acquisition, stage-specific feedback, NORMAL cruise, arc headroom and disable");
}
