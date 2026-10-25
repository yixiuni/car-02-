#include "ground_test.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void off(const MotorTest *t) { assert(!t->left_pwm && !t->right_pwm); }
static void step(GroundTest *g, MotorTest *t, uint32_t now, uint32_t request,
                 bool enabled, bool button, bool valid, float l, float r,
                 float ls, float rs)
{
    MotorTest_Update(t,now,request,enabled,button);
    GroundTest_Feedback(g,t,now,valid,l,r,ls,rs,0.005f);
    assert(t->left_pwm<=350 && t->right_pwm<=350);
    if (t->state!=MOTOR_TEST_RUNNING) off(t);
}
int main(void)
{
    for (unsigned request=1;request<=20;++request) {
        MotorTest t={0}; GroundTest g={0};
        step(&g,&t,0,request,true,false,true,100,-20,0,0);
        assert(t.state==((request>=17 && request<=19) ? MOTOR_TEST_RUNNING : MOTOR_TEST_ABORTED));
        if (request>=17 && request<=19) {
            assert(t.distance_mm==(request==17 ? 200U : request==18 ? 500U : 1000U));
            assert(t.duration_ms==(request==17 ? 4000U : request==18 ? 8000U : 15000U));
            assert(MotorTest_TracePeriodMs(&t)==(request==17 ? 20U : request==18 ? 40U : 80U));
            assert((t.duration_ms+300U)/MotorTest_TracePeriodMs(&t)+1<=256);
        }
    }
    /* A complete run at an independent prescribed speed, including tick wrap.
       Reaching either side's bound stops both; requests cannot restart it. */
    for (unsigned request=17;request<=19;++request)
    for (unsigned wrap=0;wrap<2;++wrap) {
        MotorTest t={0}; GroundTest g={0};
        uint32_t start=wrap ? UINT32_MAX-100 : 0;
        step(&g,&t,start,request,true,false,true,300,400,0,0);
        unsigned steps=t.distance_mm*5/2;
        for (unsigned i=1;i<=steps;++i) {
            step(&g,&t,start+i*5,0,true,false,true,300+i*0.4f,400+i*0.4f,80,80);
            if (i<steps) assert(t.state==MOTOR_TEST_RUNNING);
        }
        assert(t.state==MOTOR_TEST_DONE && g.fault==GROUND_OK); off(&t);
        step(&g,&t,start+steps*5+5,request,true,false,true,300+t.distance_mm,400+t.distance_mm,0,0);
        assert(t.state==MOTOR_TEST_DONE);
    }
    for (unsigned request=17;request<=19;++request)
    for (unsigned reason=0;reason<9;++reason) {
        MotorTest t={0}; GroundTest g={0};
        step(&g,&t,0,request,true,false,true,0,0,0,0);
        float l=0,r=0,ls=0,rs=0;
        if (reason==3) l=-4;
        if (reason==4) r=13;
        if (reason==5) rs=181;
        if (reason==6) l=NAN;
        if (reason==7) rs=INFINITY;
        step(&g,&t,reason==8 ? 51 : 5,0,reason!=0,reason==1,reason!=2,l,r,ls,rs);
        assert(t.state==MOTOR_TEST_ABORTED); off(&t);
        step(&g,&t,56,request,true,false,true,0,0,0,0);
        assert(t.state==MOTOR_TEST_ABORTED);
    }
    /* Each wheel must make progress; tiny backwards/forwards jitter cannot
       continually postpone the progress deadline. */
    for (unsigned request=17;request<=19;++request)
    for (unsigned stalled=0;stalled<2;++stalled) {
        MotorTest t={0}; GroundTest g={0};
        uint32_t start=UINT32_MAX-100;
        step(&g,&t,start,request,true,false,true,0,0,0,0);
        for (unsigned i=1;i<=140;++i) {
            float moving=i/20.0f, jitter=(i%2)*0.1f;
            step(&g,&t,start+i*5,0,true,false,true,
                 stalled==0 ? jitter : moving,stalled==1 ? jitter : moving,1,1);
        }
        assert(t.state==MOTOR_TEST_ABORTED && g.fault==GROUND_STALL);
    }
    /* Both wheels progressing slowly still hit their independent deadline. */
    for (unsigned request=17;request<=19;++request) {
        MotorTest t={0}; GroundTest g={0};
        step(&g,&t,0,request,true,false,true,0,0,0,0);
        for (unsigned i=1;i<=t.duration_ms/5;++i)
            step(&g,&t,i*5,0,true,false,true,i*0.02f,i*0.02f,4,4);
        assert(t.state==MOTOR_TEST_ABORTED); off(&t);
    }
    {
        MotorTest t={0}; GroundTest g={0};
        step(&g,&t,0,17,true,false,true,0,0,6,0);
        assert(t.state==MOTOR_TEST_ABORTED && g.fault==GROUND_MOVING_START);
    }
    /* Closed-loop exercise with a fixed first-order plant independent of FF. */
    for (unsigned request=17;request<=19;++request)
    for (unsigned scenario=0;scenario<3;++scenario) {
        MotorTest t={0}; GroundTest g={0};
        float l=0,r=0,ls=0,rs=0;
        for (unsigned i=0;i<=3000;++i) {
            step(&g,&t,i*5,i==0 ? request : 0,true,false,true,l,r,ls,rs);
            if (t.state!=MOTOR_TEST_RUNNING) break;
            float gain_l=scenario==1 ? 0.7f : 1.0f;
            float gain_r=scenario==2 ? 0.8f : 1.0f;
            ls+=(t.left_pwm*gain_l-ls)*0.05f;
            rs+=(t.right_pwm*gain_r-rs)*0.05f;
            l+=ls*0.005f; r+=rs*0.005f;
        }
        if (t.state!=MOTOR_TEST_DONE)
            fprintf(stderr,"plant scenario %u: state=%u fault=%u distances=%.3f/%.3f\n",
                    scenario,t.state,g.fault,l,r);
        assert(t.state==MOTOR_TEST_DONE); off(&t);
    }
    puts("PASS: ground request isolation, position/deadline stop, stall, feedback, inhibit, wrap and bounded PI");
}
