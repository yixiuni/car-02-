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
    assert(fabsf(straight-CAR_TT_STRAIGHT_SPEED)<.01f); /* actually uses straight acceleration */
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
    c.x_mm=1200;c.y_mm=-400;c.have_line=true;c.distance_mm=200;
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
    float requested_peak=CAR_TT_ENTRY_SPEED+fabsf(omega)*c.geometry.track_mm*.5f;
    assert(fabsf(fmaxf(c.left_target,c.right_target)-fminf(requested_peak,CAR_TT_WHEEL_SPEED))<.01f);
    in.gray_online=false;in.now_ms+=5;CarControl_Update(&c,&in);
    assert(c.state==CAR_FAULT && c.fault==CAR_FAULT_GRAY && !c.left_pwm && !c.right_pwm);
    /* Replay both outer-edge directions from C/D entry. During lost samples
     * steering must fade, never pivot a target wheel, or manufacture a point. */
    for(unsigned side=0;side<2;side++) {
        float sign=side==0 ? 1.0f : -1.0f;
        Init(side==0 ? 870 : 120,side==0 ? -0.67f : 3.8f);
        c.task=4;c.segment=side==0 ? 1 : 3;c.state=CAR_ARC;
        c.y_mm=-690;c.arc_heading_start=c.heading_rad;
        c.have_line=true;c.line_error=-sign*52.5f;c.line_rate=-sign*2000;
        float previous_turn=100;
        for(unsigned i=0;i<36;i++) {
            Step(0,true);
            float turn=sign*(c.right_target-c.left_target)/c.geometry.track_mm;
            if(CAR_TIME_TRIAL_STAGE3) {
                assert(fminf(c.left_target,c.right_target)>=CAR_TT_ENTRY_SPEED*.5f-.01f);
                assert(turn<=previous_turn+.0001f);
                if(in.now_ms==180) assert(fabsf(turn-CAR_TT_ENTRY_SPEED/radius)<.0001f);
            } else {
                float expected_turn=CAR_TT_ENTRY_SPEED/radius;
                if(in.now_ms<=80) expected_turn+=(CAR_TT_ENTRY_SPEED/180)*CAR_LINE_KP*52.5f;
                assert(fabsf(turn-expected_turn)<.0001f);
            }
            previous_turn=turn;
            assert(c.point_events==0 && c.black_frames==0);
        }
        in.now_ms+=5;++in.gray_samples;CarControl_Update(&c,&in);
        assert(c.state==CAR_FAULT && c.fault==CAR_FAULT_LINE);
        assert(!c.left_pwm && !c.right_pwm && c.point_events==0);
    }
    if(CAR_TIME_TRIAL_STAGE3) {
        Init(870,-.67f);c.task=4;c.segment=1;c.state=CAR_ARC;
        c.y_mm=-690;c.arc_heading_start=c.heading_rad;c.have_line=true;
        c.line_error=-52.5f;c.line_rate=-2000;
        Step(1,false); /* fresh-state extreme correction also keeps inner target */
        assert(fminf(c.left_target,c.right_target)>=CAR_TT_ENTRY_SPEED*.5f-.01f);
        for(unsigned i=0;i<20;i++) Step(0,true);
        Step(0x18,true); /* fresh centered observation cancels loss */
        assert(!c.line_missing && c.black_frames==1 && c.point_events==0);
        for(unsigned i=0;i<40;i++) Step(0x18,true);
        assert(c.state==CAR_ARC && c.fault==CAR_OK && c.line_error==0);
    }
    puts("PASS: time-trial straight acceleration, alignment/entry envelope, fresh endpoint confirmation, curvature-preserving wheel limit and fault stop");
}
