/* Independent kinematic plant and geometric sensor model. This checks closed
 * loop behavior, not just a scripted sequence of expected state transitions.
 * It cannot validate tire slip, electrical wiring or real sensor calibration. */
#include "car_control.h"
#include "car_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct { float x,y,theta,vl,vr,left,right; } Plant;
static uint8_t Sense(const Plant *p,float forward)
{
    uint8_t mask=0;
    for (unsigned i=0;i<8;i++) {
        float side=(3.5f-(float)i)*CAR_GRAY_PITCH_MM;
        float x=p->x+forward*cosf(p->theta)-side*sinf(p->theta);
        float y=p->y+forward*sinf(p->theta)+side*cosf(p->theta);
        bool left=x<=0 && fabsf(hypotf(x,y+400)-400)<=9;
        bool right=x>=1000 && fabsf(hypotf(x-1000,y+400)-400)<=9;
        if(left || right) mask|=(uint8_t)(1U<<i);
    }
    return mask;
}
static bool PointCovered(const Plant *p,CarPoint point,const CarGeometry *g)
{
    const float xy[4][2]={{0,0},{1000,0},{1000,-800},{0,-800}};
    float dx=xy[point][0]-p->x,dy=xy[point][1]-p->y;
    float f=cosf(p->theta)*dx+sinf(p->theta)*dy;
    float l=-sinf(p->theta)*dx+cosf(p->theta)*dy;
    return f<=g->front_mm && f>=-g->rear_mm && fabsf(l)<=g->half_width_mm;
}
static bool ArcCovered(const Plant *p, const CarGeometry *g, unsigned segment)
{
    /* Sample the prescribed black semicircle, not the controller's estimate. */
    for(unsigned i=0;i<=180;i++) {
        float angle=((float)i-90)*CAR_PI/180.0f;
        if(segment==3) angle+=CAR_PI;
        float x=(segment==1 ? 1000.0f : 0.0f)+400*cosf(angle);
        float y=-400+400*sinf(angle);
        float dx=x-p->x,dy=y-p->y;
        float f=cosf(p->theta)*dx+sinf(p->theta)*dy;
        float l=-sinf(p->theta)*dx+cosf(p->theta)*dy;
        if(f<=g->front_mm && f>=-g->rear_mm && fabsf(l)<=g->half_width_mm) return true;
    }
    return false;
}
static int Run(unsigned task,float gain_l,float gain_r,float bias,unsigned injection)
{
    CarGeometry g={65,120,1560,1560,125,125,75,70,true};
    CarControl c; CarControl_Init(&c,&g,0,false); c.task=(uint8_t)task;
    float initial=task<=2 ? 0 : atan2f(-800,1000);
    Plant p={.theta=initial};
    CarInputs in={.enabled=true,.gray_online=true,.imu_ready=true,.encoder_valid=true};
    unsigned events=0; CarState prev=CAR_STOPPED; unsigned seg=0;
    uint32_t arc_at=0; bool injected=false;
    float frozen_left=0;
    const CarFault expected_fault[]={CAR_OK,CAR_FAULT_GRAY,CAR_FAULT_LINE,CAR_FAULT_STALL,CAR_FAULT_TIMING};
    for(unsigned ms=5;ms<170000;ms+=5) {
        float tl=(float)c.left_pwm/CAR_SPEED_FF*gain_l;
        float tr=(float)c.right_pwm/CAR_SPEED_FF*gain_r;
        p.vl+=(tl-p.vl)*(.005f/.065f); p.vr+=(tr-p.vr)*(.005f/.065f);
        float dl=p.vl*.005f,dr=p.vr*.005f,dtheta=(dr-dl)/g.track_mm;
        p.x+=(dl+dr)*.5f*cosf(p.theta+dtheta*.5f);
        p.y+=(dl+dr)*.5f*sinf(p.theta+dtheta*.5f); p.theta+=dtheta;
        p.left+=dl; p.right+=dr;
        in.now_ms=ms; in.pressed=ms>=100 && ms<160;
        in.left_mm=p.left; in.right_mm=p.right; in.left_speed=p.vl; in.right_speed=p.vr;
        in.yaw_rad=p.theta-initial+bias*(float)ms*.001f;
        if(ms%15==0) { ++in.gray_samples; in.black_mask=Sense(&p,g.sensor_forward_mm); }
        if(c.state==CAR_ARC && !arc_at) arc_at=ms;
        if(injection && arc_at && ms-arc_at>1000) {
            if(!injected) frozen_left=in.left_mm;
            injected=true;
            if(injection==1) in.gray_online=false;
            if(injection==2) in.black_mask=0;
            if(injection==3) { in.left_mm=frozen_left; in.left_speed=0; }
            if(injection==4) in.now_ms+=100;
        }
        if(!injection && (c.state==CAR_CAPTURE || c.state==CAR_ARC) &&
            !ArcCovered(&p,&g,c.segment)) {
            fprintf(stderr,"FAIL task%u: body completely off prescribed arc\n",task); return 1;
        }
        CarControl_Update(&c,&in);
        if(c.state!=prev || c.segment!=seg) {
            if(getenv("SIM_TRACE")) printf("task%u t%.2f state%d fault%d seg%u laps%u true(%.0f,%.0f,%.1f) map(%.0f,%.0f) mask%02x\n",
                task,ms*.001,c.state,c.fault,c.segment,c.completed_laps,p.x,p.y,p.theta*180/CAR_PI,c.x_mm,c.y_mm,in.black_mask);
            prev=c.state; seg=c.segment;
        }
        if(c.point_events!=events) {
            events=c.point_events;
            if(!PointCovered(&p,c.last_point,&g)) {
                fprintf(stderr,"FAIL task%u: event point%d not physically covered at %.0f,%.0f\n",task,c.last_point,p.x,p.y);return 1;
            }
        }
        if(c.state==CAR_FAULT) {
            if(injected && c.fault==expected_fault[injection] && !c.left_pwm && !c.right_pwm) {
                in.gray_online=in.imu_ready=in.encoder_valid=true; in.pressed=false;
                for(unsigned k=0;k<30;k++) { in.now_ms+=5; CarControl_Update(&c,&in); }
                if(c.state!=CAR_FAULT) return 1;
                printf("PASS: injected fault %u stops and latches during arc\n",injection);
                return 0;
            }
            fprintf(stderr,"FAIL task%u t%.2f fault%d seg%u true(%.1f,%.1f,%.1f)\n",
                task,ms*.001,c.fault,c.segment,p.x,p.y,p.theta*180/CAR_PI); return 1;
        }
        if(c.state==CAR_DONE) {
            if(injection) return 1;
            unsigned expected=task==1 ? 2 : task==4 ? 17 : 5;
            if(events!=expected || !PointCovered(&p,c.last_point,&g) || c.left_pwm || c.right_pwm) return 1;
            for(unsigned k=0;k<100;k++) { in.now_ms+=5; ++in.gray_samples; CarControl_Update(&c,&in); }
            if(c.state!=CAR_DONE || c.point_events!=events) return 1;
            printf("PASS: task%u %.2fs, %u point events, %u laps (motor gains %.2f/%.2f, yaw bias %.3f rad/s)\n",
                task,ms*.001-.185,events,c.completed_laps,gain_l,gain_r,bias); return 0;
        }
    }
    return 1;
}
int main(void)
{
    for(unsigned task=1;task<=4;task++) if(Run(task,1,1,0,0)) return 1;
    for(unsigned task=1;task<=4;task++) if(Run(task,.95f,1.05f,.001f,0)) return 1;
    for(unsigned fault=1;fault<=4;fault++) if(Run(2,1,1,0,fault)) return 1;
    return 0;
}
