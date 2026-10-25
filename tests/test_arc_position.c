#include "car_control.h"
#include "car_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static CarControl c;
static CarInputs in;
static void Setup(bool left)
{
    CarGeometry g=CarGeometry_Default();g.confirmed=true;
    CarControl_Init(&c,&g,0,false);c.task=4;c.target_laps=4;c.state=CAR_ARC;
    c.segment=left ? 3 : 1;c.x_mm=left ? -450 : 1450;
    c.y_mm=left ? -235 : -565;c.heading_rad=left ? -CAR_PI*.5f : CAR_PI*.5f;
    c.arc_heading_start=0;c.distance_mm=500;c.have_line=true;c.black_frames=2;
    in=(CarInputs){.enabled=true,.gray_online=true,.imu_ready=true,
        .encoder_valid=true,.yaw_rad=c.heading_rad,.left_speed=650,.right_speed=650};
}
static void Step(uint8_t mask,bool fresh)
{
    in.now_ms+=5;in.black_mask=mask;if(fresh)++in.gray_samples;
    CarControl_Update(&c,&in);
}
static void Unchanged(uint8_t mask,bool fresh)
{
    float x=c.x_mm,y=c.y_mm,h=c.heading_rad,d=c.distance_mm;
    Step(mask,fresh);
    assert(c.x_mm==x && c.y_mm==y && c.heading_rad==h && c.distance_mm==d);
    assert(!c.point_events);
}
int main(void)
{
    assert(CAR_TIME_TRIAL_ARC_LOCALIZE);
    assert(CAR_ARC_POSITION_GAIN==.5f && CAR_ARC_POSITION_STEP_MM==8.0f);
    for(unsigned left=0;left<2;left++) {
        Setup(left);float x=c.x_mm,y=c.y_mm,h=c.heading_rad;
        Step(0x18,true);
        assert(c.state==CAR_ARC && c.fault==CAR_OK && !c.point_events);
        assert(fabsf(c.x_mm-x-(left ? 8 : -8))<.001f && fabsf(c.y_mm-y)<.001f);
        assert(c.heading_rad==h && c.distance_mm==500 && c.yaw_offset==0);
        Unchanged(0x18,false); /* one observation cannot be applied twice */
        for(unsigned i=0;i<30;i++) {
            float px=c.x_mm,py=c.y_mm;Step(0x18,true);
            assert(hypotf(c.x_mm-px,c.y_mm-py)<=8.001f && !c.point_events);
        }
        Setup(left);Unchanged(0,true);
        Setup(left);Unchanged(0xff,true);
        Setup(left);Unchanged(0x24,true); /* split black, zero mean */
        Setup(left);Unchanged(0x3c,true); /* wide band, zero mean */
        Setup(left);Unchanged(0x60,true); /* edge observation */
        Setup(left);c.distance_mm=100;Unchanged(0x18,true);
        Setup(left);c.task=2;Unchanged(0x18,true);
        Setup(left);c.black_frames=0;Unchanged(0x18,true);Unchanged(0x18,true);
        Setup(left);c.x_mm=left ? 450 : 550;Unchanged(0x18,true); /* wrong half */
        Setup(left);c.x_mm=left ? -600 : 1600;Unchanged(0x18,true); /* innovation too large */
        Setup(left);in.enabled=false;Unchanged(0x18,true);
        assert(c.state==CAR_STOPPED && !c.left_pwm && !c.right_pwm);
        Setup(left);in.now_ms=200;c.last_update=200;c.line_missing=true;c.missing_at=20;c.last_line_at=20;
        Unchanged(0,true);
        assert(c.state==CAR_FAULT && c.fault==CAR_FAULT_LINE && !c.left_pwm && !c.right_pwm);
    }
    puts("PASS: arc position observation bounds, freshness, band shape, acquisition/half/innovation exclusions, unchanged yaw/distance/points, disable and loss stop");
}
