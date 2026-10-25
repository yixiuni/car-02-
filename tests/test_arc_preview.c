#include "car_control.h"
#include "car_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static CarControl c;
static CarInputs in;
static void Init(unsigned task,unsigned segment)
{
    CarGeometry g=CarGeometry_Default();g.confirmed=true;
    CarControl_Init(&c,&g,0,false);
    c.task=(uint8_t)task;c.target_laps=4;c.segment=(uint8_t)segment;
    c.state=CAR_STRAIGHT;c.point_events=1;
    c.x_mm=task==2 ? 840 : segment==0 ? 880 : 120;
    c.y_mm=task==2 ? 0 : -704;
    c.heading_rad=task==2 ? 0 : segment==0 ? atan2f(-800,1000) : atan2f(-800,-1000);
    in=(CarInputs){.enabled=true,.gray_online=true,.imu_ready=true,
        .encoder_valid=true,.yaw_rad=c.heading_rad};
}
static void Step(uint8_t mask,bool fresh)
{
    in.now_ms+=5;if(fresh) ++in.gray_samples;in.black_mask=mask;
    CarControl_Update(&c,&in);
    assert(c.fault==CAR_OK);
    assert(fabsf((c.left_target+c.right_target)*.5f-CAR_FOUR_CRUISE_SPEED)<.01f);
}
int main(void)
{
    for(unsigned route=0;route<3;route++) {
        unsigned task=route==0 ? 2 : 4,seg=route==2 ? 2 : 0;
        Init(task,seg);Step(0x18,true);
        assert(c.state==CAR_STRAIGHT && c.segment==seg && c.point_events==1);
        float turn=c.right_target-c.left_target;
        assert(route==1 ? turn>0 : turn<0);
        for(unsigned i=0;i<4;i++) Step(0x18,false);
        assert(c.black_frames==1 && c.point_events==1); /* no repeated stale confirmations */
        Step(0x18,true);assert(c.black_frames==2 && c.state==CAR_STRAIGHT);
        Step(0x18,true);assert(c.state==CAR_ARC && c.segment==seg+1 && c.point_events==2);
    }
    Init(2,0);Step(0x18,true);Step(0,true);
    assert(c.state==CAR_STRAIGHT && c.point_events==1 && c.black_frames==0);
    assert(fabsf(c.left_target-c.right_target)<.01f); /* provisional steering cancelled */
    Init(2,0);Step(0x18,true);Step(0xff,true);
    assert(c.point_events==1 && fabsf(c.left_target-c.right_target)<.01f);
    Init(2,0);c.x_mm=600;Step(0x18,true);
    assert(c.point_events==1 && fabsf(c.left_target-c.right_target)<.01f);
    Init(2,0);c.x_mm=800;Step(0x18,true); /* distance gate yes, footprint no */
    assert(c.point_events==1 && fabsf(c.left_target-c.right_target)<.01f);
    puts("PASS: arc preview steers before confirmation, no early event, fresh-frame gate, white/all-black cancellation and footprint/distance bounds");
}
