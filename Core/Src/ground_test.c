#include "ground_test.h"
#include <math.h>

static void Stop(GroundTest *g, MotorTest *t, uint32_t state, uint32_t fault)
{
    t->state=state; t->left_pwm=t->right_pwm=0;
    g->pwm[0]=g->pwm[1]=0; g->fault=fault;
    Motion_Reset(&g->motion);
}
void GroundTest_Feedback(GroundTest *g, MotorTest *t, uint32_t now,
                        bool valid, float left_mm, float right_mm,
                        float left_speed, float right_speed, float dt)
{
    if (t->state!=MOTOR_TEST_RUNNING) {
        Stop(g,t,t->state,g->fault); return;
    }
    if ((t->distance_mm!=200 && t->distance_mm!=500 && t->distance_mm!=1000) ||
        !valid || !isfinite(left_mm) || !isfinite(right_mm) ||
        !isfinite(left_speed) || !isfinite(right_speed) ||
        !isfinite(dt) || dt<0 || dt>0.05f) {
        Stop(g,t,MOTOR_TEST_ABORTED,GROUND_BAD_FEEDBACK); return;
    }
    if (!g->started) {
        if (fabsf(left_speed)>5 || fabsf(right_speed)>5) {
            Stop(g,t,MOTOR_TEST_ABORTED,GROUND_MOVING_START); return;
        }
        g->started=true; g->origin[0]=left_mm; g->origin[1]=right_mm;
        g->progressed_at[0]=g->progressed_at[1]=now;
        Motion_Reset(&g->motion);
    }
    float d[2]={left_mm-g->origin[0],right_mm-g->origin[1]};
    if (d[0]<-3 || d[1]<-3) {
        Stop(g,t,MOTOR_TEST_ABORTED,GROUND_REVERSE); return;
    }
    if (fabsf(d[0]-d[1])>12) {
        Stop(g,t,MOTOR_TEST_ABORTED,GROUND_DIVERGENCE); return;
    }
    if (fabsf(left_speed)>180 || fabsf(right_speed)>180) {
        Stop(g,t,MOTOR_TEST_ABORTED,GROUND_OVERSPEED); return;
    }
    if (d[0]>=t->distance_mm || d[1]>=t->distance_mm) {
        Stop(g,t,MOTOR_TEST_DONE,GROUND_OK); return;
    }
    for (unsigned i=0;i<2;++i) {
        if (d[i]-g->progress[i]>=1) {
            g->progress[i]=d[i]; g->progressed_at[i]=now;
        } else if (now-g->progressed_at[i]>=700) {
            Stop(g,t,MOTOR_TEST_ABORTED,GROUND_STALL); return;
        }
    }
    if (dt>0) {
        /* Slow the final 30 mm; end distance is still measured after coasting. */
        float target=fmaxf(d[0],d[1])>=t->distance_mm-30 ? 50 : 85;
        /* Small distance-balancing trim, bounded to keep both wheels forward.
         * The independent 12 mm divergence stop remains active. */
        float trim=fmaxf(-15,fminf(15,2*(d[0]-d[1])));
        Motion_Update(&g->motion,target-trim,target+trim,left_speed,right_speed,dt,
                      &g->pwm[0],&g->pwm[1]);
    }
    t->left_pwm=g->pwm[0]; t->right_pwm=g->pwm[1];
}
