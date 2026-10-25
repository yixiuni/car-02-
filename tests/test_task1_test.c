#include "task1_test.h"
#include "car_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static CarInputs ready(uint32_t now)
{
    return (CarInputs){.now_ms=now,.enabled=true,.gray_online=true,
        .imu_ready=true,.encoder_valid=true};
}
static void button_step(Task1Test *t,uint32_t now,bool pressed,bool enabled)
{
    CarInputs in=ready(now); in.pressed=pressed; in.enabled=enabled;
    Task1Test_Apply(t,&in,CAR_STOPPED,0);
    assert(!in.enabled && !in.pressed);
}
int main(void)
{
    /* Boot-held button cannot start or carry a pending command. */
    for (unsigned request=0;request<=2;++request) {
        Task1Test t={0}; CarInputs in=ready(1); in.pressed=true;
        Task1Test_Apply(&t,&in,CAR_STOPPED,request);
        assert(!in.enabled && !in.pressed && !t.recalibrate);
        assert(t.phase==(request==0 ? TASK1_IDLE : TASK1_ABORTED));
    }
    for (unsigned wrap=0;wrap<2;++wrap) {
        uint32_t at=wrap ? UINT32_MAX-150U : 0;
        Task1Test t={0};
        button_step(&t,at,true,true);
        button_step(&t,at+100,true,true);
        button_step(&t,at+105,false,true);
        button_step(&t,at+130,false,true);
        assert(t.phase==TASK1_IDLE); /* Releasing a boot-held key is harmless. */
        button_step(&t,at+135,true,true);
        button_step(&t,at+140,false,true);
        button_step(&t,at+170,false,true);
        assert(t.phase==TASK1_IDLE); /* Bounce is not a short press. */
        button_step(&t,at+200,true,true);
        button_step(&t,at+225,true,true);
        button_step(&t,at+300,false,true);
        button_step(&t,at+324,false,true);
        assert(t.phase==TASK1_IDLE);
        button_step(&t,at+325,false,true);
        assert(t.phase==TASK1_CALIBRATING && t.recalibrate);
    }
    for (unsigned disabled=0;disabled<2;++disabled) {
        Task1Test t={0};
        button_step(&t,0,false,true);
        button_step(&t,25,false,true);
        button_step(&t,100,true,true);
        button_step(&t,125,true,true);
        button_step(&t,150,true,!disabled);
        button_step(&t,1000,false,true);
        button_step(&t,1025,false,true);
        assert(t.phase==TASK1_IDLE); /* Long hold or disabled hold never queues a run. */
    }
    for (unsigned wrap=0;wrap<2;++wrap) {
        Task1Test t={0}; uint32_t start=wrap ? UINT32_MAX-100 : 0;
        CarInputs in=ready(start);
        Task1Test_Apply(&t,&in,CAR_STOPPED,1);
        assert(t.recalibrate && !in.enabled && t.phase==TASK1_CALIBRATING);
        in=ready(start+7999);in.imu_ready=false;
        Task1Test_Apply(&t,&in,CAR_STOPPED,0);
        assert(!t.recalibrate && !in.enabled);
        in=ready(start+8000);
        Task1Test_Apply(&t,&in,CAR_STOPPED,0);
        assert(t.phase==TASK1_ABORTED && !in.enabled);
        in=ready(start+8001);
        Task1Test_Apply(&t,&in,CAR_STOPPED,1);
        assert(t.phase==TASK1_ABORTED && !in.enabled);
    }
    for (unsigned reason=0;reason<2;++reason) {
        Task1Test t={.phase=TASK1_RUNNING,.seen_running=true};
        CarInputs in=ready(10);in.enabled=reason!=0;in.pressed=reason==1;
        Task1Test_Apply(&t,&in,CAR_STRAIGHT,0);
        assert(t.phase==TASK1_ABORTED && !in.enabled);
    }
    {
        Task1Test t={.phase=TASK1_RUNNING,.phase_at=UINT32_MAX-10,.seen_running=true};
        CarInputs in=ready(t.phase_at+CAR_TEST_RUN_LIMIT_MS);
        Task1Test_Apply(&t,&in,CAR_STRAIGHT,0);
        assert(t.phase==TASK1_ABORTED && !in.enabled);
    }
    /* Real production controller, realistic button debounce and independent
       first-order wheel plant. No synthetic endpoint or readiness bypass. */
    for (unsigned trigger=0;trigger<2;++trigger) for (unsigned scenario=0;scenario<4;++scenario) {
        Task1Test gate={0}; CarGeometry geometry=CarGeometry_Default();
        geometry.confirmed=true;
        CarControl car;CarControl_Init(&car,&geometry,0,false);
        float left=0,right=0,ls=0,rs=0;
        bool calibrated=false;uint32_t calibrate_at=0;
        for (uint32_t now=0;now<=22000;now+=5) {
            if (now-calibrate_at>=3000 && gate.phase==TASK1_CALIBRATING) calibrated=true;
            CarInputs in=ready(now);in.imu_ready=calibrated;
            in.left_mm=left;in.right_mm=right;in.left_speed=ls;in.right_speed=rs;
            in.gray_samples=now/5;
            float sensor=(left+right)*0.5f+geometry.sensor_forward_mm;
            in.black_mask=(scenario!=1 && fabsf(sensor-1000)<=9) ? 0x18 : 0;
            if (scenario==3 && CarControl_Active(&car) && left>100) in.pressed=true;
            if (trigger && now>=100 && now<200) in.pressed=true;
            Task1Test_Apply(&gate,&in,car.state,!trigger && now==100 ? 1 : 0);
            if (gate.recalibrate) { calibrated=false;calibrate_at=now;in.imu_ready=false; }
            CarControl_Update(&car,&in);
            assert(car.task==1 && car.left_pwm<=CAR_PWM_LIMIT && car.right_pwm<=CAR_PWM_LIMIT);
            if (now<3100) assert(!car.left_pwm && !car.right_pwm && car.point_events==0);
            ls+=(car.left_pwm*0.8f-ls)*0.05f;
            rs+=(car.right_pwm-rs)*0.05f;
            if (scenario==2) ls=rs=0;
            left+=ls*0.005f;right+=rs*0.005f;
            if (gate.phase==TASK1_FINISHED || gate.phase==TASK1_ABORTED) break;
        }
        if (scenario==0) {
            assert(car.state==CAR_DONE && car.fault==CAR_OK);
            assert(car.point_events==2 && car.last_point==CAR_POINT_B);
            assert(car.x_mm>830 && car.x_mm<900);
        } else if (scenario==1) {
            assert(car.state==CAR_FAULT && car.fault==CAR_FAULT_ENDPOINT);
            assert(car.point_events==1);
        } else if (scenario==2) {
            assert(car.state==CAR_FAULT && car.fault==CAR_FAULT_STALL);
        } else {
            assert(gate.phase==TASK1_ABORTED && car.state==CAR_STOPPED);
        }
        assert(!car.left_pwm && !car.right_pwm);
        CarInputs in=ready(23000);
        Task1Test_Apply(&gate,&in,car.state,1);
        assert(!in.enabled);
    }
    puts("PASS: task1 SWD/button one-shot, debounce/boot hold/wrap, calibration, inhibits, timeouts, real endpoint and missing-line/stall stops");
}
