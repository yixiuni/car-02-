#include "car_control.h"
#include "car_arc_debug.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static CarControl c;
static CarInputs in;
static void Init(void)
{
    CarGeometry g=CarGeometry_Default();g.confirmed=true;
    CarControl_Init(&c,&g,0,false);c.task=4;c.target_laps=4;c.state=CAR_STRAIGHT;
    c.x_mm=100;c.y_mm=-80;c.heading_rad=atan2f(-800,1000);
    in=(CarInputs){.enabled=true,.gray_online=true,.imu_ready=true,.encoder_valid=true,
        .yaw_rad=c.heading_rad,.left_speed=650,.right_speed=650};
    for(unsigned i=0;i<8;i++)car_arc_debug[i]=(CarArcDebug){0};
}
static void Step(unsigned mask,bool fresh)
{
    in.now_ms+=15;in.black_mask=mask;if(fresh)++in.gray_samples;
    CarControl_Update(&c,&in);
}
static void ExitFixture(void)
{
    Init();float h=asinf(165.0f/400);
    c.state=CAR_ARC;c.segment=3;c.point_events=4;c.distance_mm=1150;
    c.heading_rad=in.yaw_rad=h;c.arc_heading_start=h+3.6f;
    c.edge_x=-165*cosf(h);c.edge_y=-165*sinf(h);c.edge_heading=h;
    c.edge_valid=c.edge_ready=true;
    c.x_mm=c.edge_x+20*cosf(h);c.y_mm=c.edge_y+20*sinf(h);
    c.have_line=true;c.line_missing=true;c.last_line_at=10;c.missing_at=25;
    c.white_frames=3;c.last_update=45;in.now_ms=50;
}
int main(void)
{
    assert(CAR_TIME_TRIAL_EDGE_POSE);
    Init();Step(0x18,true);Step(0x18,true);assert(!c.edge_valid);
    Step(0x18,true);assert(c.edge_valid && !c.edge_ready);
    float x=c.x_mm,y=c.y_mm,h=c.heading_rad;
    c.x_mm+=10;c.y_mm+=2;c.heading_rad+=.02f;in.yaw_rad=c.heading_rad;
    Step(0,true);
    assert(c.edge_ready && c.edge_valid);
    assert(fabsf(c.edge_x-x-5)<.001f && fabsf(c.edge_y-y-1)<.001f);
    assert(fabsf(c.edge_heading-h-.01f)<.001f);
    x=c.edge_x;y=c.edge_y;h=c.edge_heading;
    c.x_mm+=10;Step(0,true);Step(0,false);
    assert(c.edge_x==x && c.edge_y==y && c.edge_heading==h);
    Step(0xff,true);assert(!c.edge_valid && !c.edge_ready);
    Step(0,true);assert(!c.edge_ready);
    Init();for(unsigned i=0;i<3;i++)Step(0x24,true);
    Step(0,true);assert(!c.edge_ready); /* split bands are not an edge */
    Init();for(unsigned i=0;i<3;i++)Step(0x18,true);
    Step(0,false);Step(0,false);Step(0,true);assert(!c.edge_ready); /* >40ms bracket */
    ExitFixture();x=c.x_mm;y=c.y_mm;CarControl_Update(&c,&in);
    assert(c.state==CAR_STRAIGHT && c.point_events==5);
    assert(fabsf(c.x_mm-x)<.001f && fabsf(c.y_mm-y)<.001f); /* no 20mm backwards snap */
    assert(!c.edge_ready && !c.edge_valid); /* next segment clears snapshot */
    ExitFixture();c.edge_valid=false;CarControl_Update(&c,&in);
    assert(c.point_events==4 && c.state==CAR_ARC);
    assert(car_arc_debug[1].first.rejected&ARC_EXIT_EDGE_POSE);
    /* offset must exceed CAR_CROSS_ENDPOINT_CORRECTION_MM; the test checks the rule, not the number */
    ExitFixture();c.x_mm+=CAR_CROSS_ENDPOINT_CORRECTION_MM+20;
    c.edge_x+=CAR_CROSS_ENDPOINT_CORRECTION_MM+20;CarControl_Update(&c,&in);
    assert(c.point_events==4 && (car_arc_debug[1].first.rejected&ARC_EXIT_CORRECTION));
    ExitFixture();c.last_line_at=0;in.now_ms=105;c.last_update=100;CarControl_Update(&c,&in);
    assert(c.point_events==4 && (car_arc_debug[1].first.rejected&ARC_EXIT_STALE));
    ExitFixture();c.edge_heading+=.6f;CarControl_Update(&c,&in);
    assert(c.point_events==4 && (car_arc_debug[1].first.rejected&ARC_EXIT_HEADING));
    ExitFixture();in.enabled=false;CarControl_Update(&c,&in);
    assert(c.state==CAR_STOPPED && !c.left_pwm && !c.right_pwm);
    puts("PASS: timestamped edge midpoint, freeze, ambiguity/gap rejection, propagation, reset, correction/heading/age gates and disable");
}
