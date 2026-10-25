#include "car_control.h"
#include "car_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static CarControl c;
static CarInputs in;
static void Init(float x,float heading)
{
    CarGeometry g=CarGeometry_Default();g.confirmed=true;
    CarControl_Init(&c,&g,0,false);
    c.task=2;c.target_laps=4;c.state=CAR_STRAIGHT;
    c.x_mm=x;c.heading_rad=heading;
    in=(CarInputs){.enabled=true,.gray_online=true,.imu_ready=true,
        .encoder_valid=true,.yaw_rad=heading};
}
static float Step(uint8_t mask,bool fresh)
{
    in.now_ms+=5;in.black_mask=mask;if(fresh) ++in.gray_samples;
    CarControl_Update(&c,&in);
    assert(c.fault==CAR_OK);
    assert(c.left_target>=0 && c.right_target>=0);
    assert(c.left_target<=CAR_TT_WHEEL_SPEED+.01f && c.right_target<=CAR_TT_WHEEL_SPEED+.01f);
    return (c.left_target+c.right_target)*.5f;
}
int main(void)
{
    Init(400,0);float straight=Step(0,true);
    assert(straight>CAR_TT_ENTRY_SPEED+20); /* actually uses straight acceleration */
    Init(800,0);float entry=Step(0,true);
    assert(fabsf(entry-CAR_TT_ENTRY_SPEED)<.01f);
    Init(400,0.8f);assert(Step(0,true)<straight); /* align before sprint */

    Init(840,0);Step(0x18,true);
    assert(c.state==CAR_STRAIGHT && c.point_events==0 && c.right_target<c.left_target);
    for(unsigned i=0;i<4;i++) Step(0x18,false);
    assert(c.point_events==0 && c.black_frames==1);
    Step(0x18,true);Step(0x18,true);
    assert(c.state==CAR_ARC && c.point_events==1); /* real three-frame gate */

    Init(400,0);c.state=CAR_ARC;c.segment=1;
    c.x_mm=1200;c.y_mm=-400;c.have_line=true;
    c.line_error=-52.5f;c.line_rate=-2000;c.last_line_at=0;
    Step(1,false); /* strong steering: common scale must retain curvature */
    float radius=sqrtf(400*400-c.geometry.sensor_forward_mm*c.geometry.sensor_forward_mm);
    float omega=-CAR_TT_ENTRY_SPEED/radius-
        (CAR_TT_ENTRY_SPEED/180)*CAR_LINE_KP*c.line_error-
        (CAR_FOUR_CRUISE_SPEED/180)*CAR_LINE_KD*c.line_rate;
    float expected=omega/CAR_TT_ENTRY_SPEED;
    float actual=(c.right_target-c.left_target)/c.geometry.track_mm/
        ((c.left_target+c.right_target)*.5f);
    assert(fabsf(expected-actual)<.00001f);
    assert(fabsf(fmaxf(c.left_target,c.right_target)-CAR_TT_WHEEL_SPEED)<.01f);
    in.gray_online=false;in.now_ms+=5;CarControl_Update(&c,&in);
    assert(c.state==CAR_FAULT && c.fault==CAR_FAULT_GRAY && !c.left_pwm && !c.right_pwm);
    /* Reproduce an outer-probe loss during D entry. Hold P, not D, while
     * recovering, but still stop at the original 180 ms loss deadline. */
    Init(120,3.8f);c.task=4;c.segment=3;c.state=CAR_ARC;
    c.y_mm=-680;c.arc_heading_start=c.heading_rad;
    c.have_line=true;c.line_error=52.5f;c.line_rate=2000;
    for(unsigned i=0;i<36;i++) {
        Step(0,true);
        float omega_actual=(c.right_target-c.left_target)/c.geometry.track_mm;
        float omega_expected=-CAR_TT_ENTRY_SPEED/radius;
        if(CAR_TIME_TRIAL_STAGE3 || in.now_ms<=80)
            omega_expected-=(CAR_TT_ENTRY_SPEED/180)*CAR_LINE_KP*c.line_error;
        assert(fabsf(omega_actual-omega_expected)<.0001f);
        assert(c.point_events==0 && c.black_frames==0);
    }
    in.now_ms+=5;++in.gray_samples;CarControl_Update(&c,&in);
    assert(c.state==CAR_FAULT && c.fault==CAR_FAULT_LINE);
    assert(!c.left_pwm && !c.right_pwm && c.point_events==0);
    puts("PASS: time-trial straight acceleration, alignment/entry envelope, fresh endpoint confirmation, curvature-preserving wheel limit and fault stop");
}
