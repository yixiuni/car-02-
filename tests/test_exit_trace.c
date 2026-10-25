#include "car_control.h"
#include "car_config.h"
#include "car_arc_debug.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static CarControl c;static CarInputs in;
static void Init(float x,float y,float heading)
{
    CarGeometry g=CarGeometry_Default();g.confirmed=true;CarControl_Init(&c,&g,0,false);
    c.task=4;c.target_laps=4;c.state=CAR_ARC;c.segment=3;c.point_events=4;
    c.x_mm=x;c.y_mm=y;c.heading_rad=heading;c.arc_heading_start=3.5f;
    c.distance_mm=1000;c.have_line=true;c.line_error=0;c.last_line_at=10;
    c.line_missing=true;c.missing_at=20;c.white_frames=3;
    in=(CarInputs){.now_ms=50,.enabled=true,.gray_online=true,.imu_ready=true,
        .encoder_valid=true,.yaw_rad=heading};
    for(unsigned i=0;i<CAR_ARC_DEBUG_CAPACITY;i++) car_arc_debug[i]=(CarArcDebug){0};
}
int main(void)
{
    assert(CAR_TIME_TRIAL_EXIT_TRACE && !CAR_TIME_TRIAL_ARC_BOOST);
    assert(CAR_TT_STRAIGHT_SPEED==800 && CAR_TT_ARC_SPEED==(CAR_TIME_TRIAL_ENTRY_HOLD ? 460 : CAR_TIME_TRIAL_ARC_STEP ? 450 : 430));
    /* Correction rejection is recorded but does not turn into a point event. */
    float h=asinf(165.0f/400);
    Init(0,-150,h);CarControl_Update(&c,&in);
    assert(c.state==CAR_ARC && c.point_events==4);
    assert(car_arc_debug[1].exit_attempts==1);
    assert(car_arc_debug[1].first.rejected&ARC_EXIT_CORRECTION);
    uint32_t first_tick=car_arc_debug[1].first.tick;
    for(in.now_ms=55;in.now_ms<=120;in.now_ms+=5) CarControl_Update(&c,&in);
    assert(car_arc_debug[1].last.rejected&ARC_EXIT_STALE);
    assert(car_arc_debug[1].first.tick==first_tick && car_arc_debug[1].last.tick==120);
    /* A valid edge remains accepted; the zero rejection mask records success. */
    Init(-165*cosf(h),-165*sinf(h),h);CarControl_Update(&c,&in);
    assert(c.state==CAR_STRAIGHT && c.point_events==5);
    assert(car_arc_debug[1].exit_attempts==1 && car_arc_debug[1].last.rejected==0);
    /* Black-line radius logging is observational: no position correction. */
    Init(-390,-400,1.5707963268f);c.white_frames=0;c.line_missing=false;c.last_line_at=0;
    in.now_ms=5;in.gray_samples=1;in.black_mask=0x18;CarControl_Update(&c,&in);
    assert(c.x_mm==-390 && c.y_mm==-400 && c.point_events==4);
    assert(car_arc_debug[1].black_samples==1 && car_arc_debug[1].max_abs_radial_error_mm>0);
    in.now_ms=10;CarControl_Update(&c,&in);assert(car_arc_debug[1].black_samples==1);
    /* New physical-button start clears observations from the previous run. */
    uint32_t previous_events=c.point_events;
    c.state=CAR_DONE;in.pressed=true;in.now_ms=100;CarControl_Update(&c,&in);
    in.now_ms=150;CarControl_Update(&c,&in);in.pressed=false;in.now_ms=200;CarControl_Update(&c,&in);
    in.now_ms=250;CarControl_Update(&c,&in);
    assert(c.state==CAR_STRAIGHT && c.point_events==previous_events+1);
    for(unsigned i=0;i<CAR_ARC_DEBUG_CAPACITY;i++) assert(car_arc_debug[i].exit_attempts==0 && car_arc_debug[i].black_samples==0);
    puts("PASS: exit rejection observations, first/last preservation, accepted edge, fresh radial logging without pose changes, run reset");
}
