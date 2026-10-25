#include "car_control.h"
#include "car_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static CarControl c;
static CarInputs in;
static void Setup(uint32_t now,uint32_t age,float error,float progress,bool observed)
{
    CarGeometry g=CarGeometry_Default();g.confirmed=true;
    CarControl_Init(&c,&g,now-5U,false);
    c.task=4;c.target_laps=4;c.state=CAR_ARC;c.segment=1;
    c.x_mm=1200;c.y_mm=-600;c.heading_rad=1.0f;c.arc_heading_start=.5f;
    c.distance_mm=progress;c.have_line=observed;c.line_missing=true;
    c.line_error=error;c.line_rate=100000; /* stale D must not be used */
    c.last_line_at=c.line_at=c.missing_at=now-age;
    c.white_frames=10;c.last_gray_samples=20;c.run_at=now-500U;
    in=(CarInputs){.now_ms=now,.gray_samples=20,.enabled=true,
        .gray_online=true,.imu_ready=true,.encoder_valid=true,.yaw_rad=1.0f,
        .left_speed=370,.right_speed=370};
}
static float Omega(void)
{
    CarControl_Update(&c,&in);
    assert(c.state==CAR_ARC && c.fault==CAR_OK && c.point_events==0);
    return (c.right_target-c.left_target)/c.geometry.track_mm;
}
int main(void)
{
    assert(CAR_TIME_TRIAL_ENTRY_HOLD);
    float radius=sqrtf(400.0f*400.0f-CAR_SENSOR_FORWARD_MM*CAR_SENSOR_FORWARD_MM);
    float base=370.0f/radius;
    for(unsigned wrap=0;wrap<2;wrap++) {
        uint32_t now=wrap ? 5U : 2000U;
        for(unsigned side=0;side<2;side++) {
            float error=side ? 52.5f : -52.5f;
            Setup(now,85,error,100,true);
            assert(fabsf(Omega()-(base-(370.0f/180.0f)*CAR_LINE_KP*error))<.001f);
            Setup(now,175,error,100,true);
            assert(fabsf(Omega()-(base-(370.0f/180.0f)*CAR_LINE_KP*error))<.001f);
            Setup(now,180,error,100,true);CarControl_Update(&c,&in);
            assert(c.state==CAR_FAULT && c.fault==CAR_FAULT_LINE);
            assert(!c.left_pwm && !c.right_pwm && !c.point_events);
        }
        Setup(now,100,52.5f,200,true);assert(fabsf(Omega()-base)<.001f);
        Setup(now,100,15,100,true);assert(fabsf(Omega()-base)<.001f);
        Setup(now,100,52.5f,100,false);assert(fabsf(Omega()-base)<.001f);
        Setup(now,100,52.5f,100,true);
        ++in.gray_samples;in.black_mask=0x18;Omega();
        assert(!c.line_missing && c.line_error==0 && c.white_frames==0);
        Setup(now,100,52.5f,100,true);in.pressed=true;in.enabled=false;
        CarControl_Update(&c,&in);
        assert(c.state==CAR_STOPPED && !c.left_pwm && !c.right_pwm);
    }
    puts("PASS: entry P-only hold, both sides, expiry180, mature/center/no-observation exclusions, reacquisition, disable and tick wrap");
}
