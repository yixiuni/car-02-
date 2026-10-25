#include "car_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static CarControl c;
static CarInputs in;
static float radius;
static void Setup(unsigned left)
{
    CarGeometry g=CarGeometry_Default();g.confirmed=true;
    CarControl_Init(&c,&g,0,false);c.task=4;c.target_laps=4;c.state=CAR_ARC;
    c.segment=left ? 3 : 1;c.heading_rad=CAR_PI*.5f;
    radius=sqrtf(400*400-g.sensor_forward_mm*g.sensor_forward_mm);
    c.x_mm=left ? -radius : 1000+radius;c.y_mm=-320;
    c.distance_mm=500;c.black_frames=4;c.have_line=true;
    c.arc_heading_start=c.heading_rad;
    in=(CarInputs){.enabled=true,.gray_online=true,.imu_ready=true,.encoder_valid=true,
        .yaw_rad=c.heading_rad,.left_speed=650,.right_speed=650};
}
static void Step(bool fresh,unsigned mask)
{
    in.now_ms+=5;if(fresh)++in.gray_samples;in.black_mask=mask;
    CarControl_Update(&c,&in);
}
static void RadialOnly(void)
{
    float x=c.x_mm,y=c.y_mm,cs=cosf(c.heading_rad),sn=sinf(c.heading_rad);
    float cx=c.segment==1 ? 1000 : 0;
    float dx=x+c.geometry.sensor_forward_mm*cs-cx,dy=y+c.geometry.sensor_forward_mm*sn+400;
    float r=hypotf(dx,dy),step=fmaxf(-8,fminf(8,.5f*(400-r)));
    Step(true,0x18);
    assert(fabsf(c.x_mm-x-step*dx/r)<.001f && fabsf(c.y_mm-y-step*dy/r)<.001f);
}
int main(void)
{
    assert(CAR_TIME_TRIAL_ARC_PHASE);
    for(unsigned left=0;left<2;left++) {
        Setup(left);
        for(unsigned i=0;i<180;i++) {
            float x=c.x_mm,y=c.y_mm,h=c.heading_rad,d=c.distance_mm;
            float cx=left ? 0 : 1000,dx=x-cx,dy=y+165+400,r=hypotf(dx,dy);
            float radial=fmaxf(-8,fminf(8,.5f*(400-r)));
            Step(true,0x18);
            float tx=c.x_mm-x-radial*dx/r,ty=c.y_mm-y-radial*dy/r;
            assert(hypotf(tx,ty)<=2.001f && fabsf(tx*dx/r+ty*dy/r)<.001f);
            assert(c.state==CAR_ARC && c.heading_rad==h && c.distance_mm==d && !c.point_events);
        }
        assert(fabsf(c.y_mm+400)<.02f && fabsf(c.x_mm-(left ? -radius : 1000+radius))<.02f);
        float x=c.x_mm,y=c.y_mm;Step(false,0x18);assert(c.x_mm==x && c.y_mm==y);
        Setup(left);c.distance_mm=250;RadialOnly(); /* acquisition exclusion */
        Setup(left);c.black_frames=2;RadialOnly(); /* only three after update */
        Setup(left);c.line_rate=1000;RadialOnly(); /* unsettled line */
        Setup(left);c.x_mm=left ? -200 : 1200;c.y_mm=-170;RadialOnly(); /* innovation >160 */
        Setup(left);x=c.x_mm;y=c.y_mm;Step(true,0);assert(c.x_mm==x && c.y_mm==y);
        Setup(left);x=c.x_mm;y=c.y_mm;Step(true,0xff);assert(c.x_mm==x && c.y_mm==y);
        Setup(left);x=c.x_mm;y=c.y_mm;Step(true,0x24);assert(c.x_mm==x && c.y_mm==y);
        Setup(left);x=c.x_mm;y=c.y_mm;Step(true,0x80);assert(c.x_mm==x && c.y_mm==y);
        Setup(left);c.task=2;x=c.x_mm;y=c.y_mm;Step(true,0x18);assert(c.x_mm==x && c.y_mm==y);
        Setup(left);in.enabled=false;x=c.x_mm;y=c.y_mm;Step(true,0x18);
        assert(c.state==CAR_STOPPED && !c.left_pwm && !c.right_pwm && c.x_mm==x && c.y_mm==y);
    }
    puts("PASS: bounded tangential phase correction, no radial/yaw/distance/event mutation, both arcs, convergence, freshness and exclusions");
}
