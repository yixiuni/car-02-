#include "course_mode.h"
#include "car_config.h"
#include <assert.h>
#include <stdio.h>

static CourseMode mode;
static Task1Test gate;
static CarControl car;
static uint32_t now;
static unsigned calibrations;
static void Step(bool pressed,bool enabled,unsigned duration)
{
    for(unsigned i=0;i<duration;i+=5) {
        now+=5;
        CarInputs in={.now_ms=now,.gray_samples=now/5,.pressed=pressed,.enabled=enabled,
            .gray_online=true,.imu_ready=true,.encoder_valid=true};
        CourseMode_Apply(&mode,&gate,&in,&car);
        if(gate.recalibrate) ++calibrations;
        CarControl_Update(&car,&in);
    }
}
static void Init(uint32_t time,bool held)
{
    CarGeometry geometry=CarGeometry_Default();geometry.confirmed=true;
    now=time;calibrations=0;gate=(Task1Test){0};
    CarControl_Init(&car,&geometry,now,held);CourseMode_Init(&mode,&car);
    Step(held,true,50);
}
static void Double(void)
{
    Step(true,true,60);Step(false,true,80);
    Step(true,true,60);Step(false,true,450);
    assert(gate.phase==TASK1_IDLE && car.state==CAR_STOPPED && !calibrations);
    assert(!car.left_pwm && !car.right_pwm);
}
static void Hold(void) { Step(true,true,1500);Step(false,true,450); }
static void Click(void) { Step(true,true,60);Step(false,true,60); }
int main(void)
{
    for(unsigned wrap=0;wrap<2;wrap++) {
        Init(wrap ? UINT32_MAX-250U : 0,false);
        assert(car.task==2 && car.target_laps==1 && mode.feedback_pulses==1);
        for(unsigned n=0;n<4;n++) { Double();assert(car.target_laps==(n%2==0 ? 4 : 1)); }
        Hold();assert(car.task==4 && car.target_laps==4 && mode.feedback_pulses==2);
        assert(car.mode_events==6 && !calibrations); /* no repeat while held */
        Double();assert(car.target_laps==1);
        Double();assert(car.target_laps==4 && mode.feedback_pulses==4);
        Hold();assert(car.task==2 && car.target_laps==1);
        Hold();assert(car.task==4 && car.target_laps==4);
        Click();assert(gate.phase==TASK1_IDLE && !car.left_pwm && !car.right_pwm);
        Step(false,true,300);assert(!calibrations); /* wait for double click */
        Step(false,true,150);
        assert(calibrations==1 && CarControl_Active(&car) && car.task==4 && car.target_laps==4);
        unsigned events=car.mode_events;
        Step(true,true,1000); /* motion press aborts, never switches route */
        assert(gate.phase==TASK1_ABORTED && car.state==CAR_STOPPED);
        assert(car.task==4 && car.mode_events==events && !car.left_pwm && !car.right_pwm);
        Step(false,true,500);assert(!CarControl_Active(&car) && calibrations==1);
        Click();Step(false,true,450);assert(calibrations==2 && CarControl_Active(&car));
    }
    Init(0,true);Hold();assert(car.task==2 && !calibrations); /* boot-held */
    Init(0,false);Step(true,true,10);Step(false,true,500);
    assert(!calibrations && car.task==2); /* bounce */
    Init(0,false);Click();Step(false,false,100);Step(false,true,500);
    assert(!calibrations && !CarControl_Active(&car)); /* pending click cancelled */
    Init(0,false);Step(true,false,1000);Step(false,true,500);
    assert(car.task==2 && !calibrations); /* disabled hold */
    for(unsigned state=CAR_DONE;state<=CAR_FAULT;state++) {
        Init(0,false);car.state=(CarState)state;gate.phase=TASK1_FINISHED;
        Click();Step(false,true,450);
        assert(calibrations==1 && car.state==CAR_STRAIGHT && car.point_events==1);
    }
    puts("PASS: dual route/lap gestures, 1/4 toggle, per-route memory, delayed start, rearm, abort, boot hold, bounce, disable and tick wrap");
}
