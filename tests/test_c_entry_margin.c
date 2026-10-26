#include "car_control.h"
#include "car_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static CarControl car;
static CarInputs input;
static void Init(unsigned task,unsigned segment,float forward)
{
    CarGeometry g=CarGeometry_Default();g.confirmed=true;
    CarControl_Init(&car,&g,0,false);
    car.task=task;car.target_laps=4;car.segment=segment;car.state=CAR_STRAIGHT;
    float x=task==2 || segment==0 ? 1000 : 0;
    float y=task==2 ? 0 : -800;
    car.heading_rad=task==2 ? 0 : atan2f(-800,segment==0 ? 1000 : -1000);
    car.x_mm=x-forward*cosf(car.heading_rad);
    car.y_mm=y-forward*sinf(car.heading_rad);
    input=(CarInputs){.enabled=true,.gray_online=true,.imu_ready=true,
        .encoder_valid=true,.yaw_rad=car.heading_rad};
}
static void Step(uint8_t mask,bool fresh)
{
    input.now_ms+=5;input.black_mask=mask;if(fresh)++input.gray_samples;
    CarControl_Update(&car,&input);assert(car.fault==CAR_OK);
}
int main(void)
{
    assert(CAR_TIME_TRIAL_ARC_PHASE);
    /* Covered by the old bounding box, but not yet by the inset front. */
    Init(4,0,160);
    for(unsigned i=0;i<8;++i)Step(0x18,true);
    assert(car.state==CAR_STRAIGHT && car.point_events==0);
    car.x_mm+=10*cosf(car.heading_rad);car.y_mm+=10*sinf(car.heading_rad);
    Step(0x18,true);assert(car.state==CAR_ARC && car.last_point==CAR_POINT_C);
    /* Old +22.5 mm right-side acceptance must keep previewing now. */
    Init(4,0,150);
    for(unsigned i=0;i<8;++i)Step(0x20,true);
    assert(car.state==CAR_STRAIGHT && car.point_events==0);
    for(unsigned i=0;i<8;++i)Step(0x10,false);
    assert(car.state==CAR_STRAIGHT && car.point_events==0);
    Step(0x10,true);assert(car.state==CAR_ARC && car.point_events==1);
    /* White/all-black cannot confirm C; left-side acquisition is unchanged. */
    for(unsigned mask=0;mask<=255;mask+=255){
        Init(4,0,150);for(unsigned i=0;i<8;++i)Step(mask,true);
        assert(car.state==CAR_STRAIGHT && car.point_events==0);
    }
    Init(4,0,150);for(unsigned i=0;i<3;++i)Step(0x04,true);
    assert(car.state==CAR_ARC && car.last_point==CAR_POINT_C);
    Init(4,0,180);for(unsigned i=0;i<8;++i)Step(0x18,true);
    assert(car.state==CAR_STRAIGHT && car.point_events==0);
    /* D and NORMAL retain their existing footprint gates. */
    Init(4,2,160);for(unsigned i=0;i<3;++i)Step(0x18,true);
    assert(car.state==CAR_ARC && car.last_point==CAR_POINT_D);
    Init(2,0,160);for(unsigned i=0;i<3;++i)Step(0x18,true);
    assert(car.state==CAR_ARC && car.last_point==CAR_POINT_B);
    puts("PASS: C front inset and right-line gate, freshness/white exclusion, D/NORMAL unchanged");
}
