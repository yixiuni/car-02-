/* Independent kinematic plant and geometric sensor model. This checks closed
 * loop behavior, not just a scripted sequence of expected state transitions.
 * It cannot validate tire slip, electrical wiring or real sensor calibration. */
#include "car_control.h"
#include "car_config.h"
#if CAR_ARC_TEST_BUILD || CAR_LAP_TEST_BUILD || CAR_CROSS_TEST_BUILD || CAR_FOUR_TEST_BUILD || CAR_DUAL_MODE_BUILD
#include "task1_test.h"
#if CAR_DUAL_MODE_BUILD
#include "course_mode.h"
#endif
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct { float x,y,theta,vl,vr,left,right; } Plant;
static float placement_error_rad;
static unsigned fault_lap;
static unsigned selected_laps;
static float b_pose_error_mm;
static unsigned b_pose_error_lap;
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
    /* Optional sensitivity experiment only. Keep the historical default fixed;
     * a faster assumed plant is never evidence that the actual car is faster. */
    float plant_pwm_per_mm_s=getenv("SIM_PWM_PER_MM_S") ?
        strtof(getenv("SIM_PWM_PER_MM_S"),NULL) : 1.5f;
    if (!isfinite(plant_pwm_per_mm_s) || plant_pwm_per_mm_s<0.5f || plant_pwm_per_mm_s>3.0f) {
        fputs("FAIL: invalid assumed motor model\n",stderr); return 1;
    }
    unsigned gray_phase=getenv("SIM_GRAY_PHASE_MS") ?
        (unsigned)strtoul(getenv("SIM_GRAY_PHASE_MS"),NULL,10)%15U : 0;
    /* Use measured chassis dimensions; counts are a test fixture only. */
    CarGeometry g=CarGeometry_Default();
    g.counts_a_rev=g.counts_b_rev=1560;
    g.confirmed=true;
    CarControl c; CarControl_Init(&c,&g,0,false); c.task=(uint8_t)task; c.target_laps=(uint8_t)selected_laps;
    unsigned laps=selected_laps ? selected_laps : task==4 ? 4 : 1;
    float initial=placement_error_rad; /* All tasks physically face B at A. */
    Plant p={.theta=initial};
    CarInputs in={.enabled=true,.gray_online=true,.imu_ready=true,.encoder_valid=true};
    #if CAR_ARC_TEST_BUILD || CAR_LAP_TEST_BUILD || CAR_CROSS_TEST_BUILD || CAR_FOUR_TEST_BUILD || CAR_DUAL_MODE_BUILD
    Task1Test gate={0};
#if CAR_DUAL_MODE_BUILD
    CourseMode mode;CourseMode_Init(&mode,&c);
    c.task=(uint8_t)task;c.target_laps=(uint8_t)laps;
    if(task==2) mode.oval_laps=(uint8_t)laps;
    else mode.cross_laps=(uint8_t)laps;
#endif
#endif
    unsigned events=0; CarState prev=CAR_STOPPED; unsigned seg=0;
    uint32_t arc_at=0,injected_at=0; bool injected=false,pose_displaced=false;
    float frozen_left=0;
    unsigned active_updates=0,saturated_updates=0,peak_pwm=0;
    const CarFault expected_fault[]={CAR_OK,CAR_FAULT_GRAY,CAR_FAULT_LINE,CAR_FAULT_STALL,CAR_FAULT_TIMING,CAR_OK};
    for(unsigned ms=5;ms<170000;ms+=5) {
        /* Fixed historical plant model: changing the controller feedforward
         * must not silently change the simulated motor being controlled. */
        float tl=(float)c.left_pwm/plant_pwm_per_mm_s*gain_l;
        float tr=(float)c.right_pwm/plant_pwm_per_mm_s*gain_r;
        /* Brake response is an independent plant assumption, not an actual
         * measured stopping distance. Also test a weak-brake/coast case. */
        float tau=CarControl_BrakeActive(&c,ms) && !getenv("SIM_WEAK_BRAKE") ? .020f : .065f;
        p.vl+=(tl-p.vl)*(.005f/tau); p.vr+=(tr-p.vr)*(.005f/tau);
        float dl=p.vl*.005f,dr=p.vr*.005f,dtheta=(dr-dl)/g.track_mm;
        p.x+=(dl+dr)*.5f*cosf(p.theta+dtheta*.5f);
        p.y+=(dl+dr)*.5f*sinf(p.theta+dtheta*.5f); p.theta+=dtheta;
        p.left+=dl; p.right+=dr;
        in.now_ms=ms; in.enabled=true; in.pressed=ms>=100 && ms<160;
        in.left_mm=p.left; in.right_mm=p.right; in.left_speed=p.vl; in.right_speed=p.vr;
        in.yaw_rad=p.theta-initial+bias*(float)ms*.001f;
        if((ms+gray_phase)%15==0) { ++in.gray_samples; in.black_mask=Sense(&p,g.sensor_forward_mm); }
        if(c.state==CAR_ARC && !arc_at) arc_at=ms;
        if(injection && arc_at && ms-arc_at>1000 && c.completed_laps>=fault_lap && (injected || c.state==CAR_ARC)) {
            if(!injected) { frozen_left=in.left_mm; injected_at=ms; }
            injected=true;
            if(injection==1) in.gray_online=false;
            if(injection==2) in.black_mask=0;
            if(injection==3) { in.left_mm=frozen_left; in.left_speed=0; }
            if(injection==4) in.now_ms+=100;
            if(injection==5) in.pressed=true;
        }
        if(!injection && (c.state==CAR_CAPTURE || c.state==CAR_ARC) &&
            !ArcCovered(&p,&g,c.segment)) {
            fprintf(stderr,"FAIL task%u: body completely off prescribed arc\n",task); return 1;
        }
#if CAR_ARC_TEST_BUILD || CAR_LAP_TEST_BUILD || CAR_CROSS_TEST_BUILD || CAR_FOUR_TEST_BUILD || CAR_DUAL_MODE_BUILD
#if CAR_DUAL_MODE_BUILD
        CourseMode_Apply(&mode,&gate,&in,&c);
#else
        Task1Test_Apply(&gate,&in,c.state,0);
#endif
#endif
        CarControl_Update(&c,&in);
        if(c.state==CAR_STRAIGHT || c.state==CAR_CAPTURE || c.state==CAR_ARC) {
            ++active_updates;
            unsigned peak=c.left_pwm>c.right_pwm ? c.left_pwm : c.right_pwm;
            if(peak>peak_pwm) peak_pwm=peak;
            if(peak>=CAR_PWM_LIMIT) ++saturated_updates;
        }
        if(c.left_target<0 || c.right_target<0) {
            fputs("FAIL: automatic start requested reverse motion\n",stderr); return 1;
        }
        if(!CAR_TIME_TRIAL_BUILD && (task==4 || (CAR_DUAL_MODE_BUILD && task==2)) && ms!=c.run_at && (c.state==CAR_STRAIGHT || c.state==CAR_CAPTURE || c.state==CAR_ARC) &&
            fabsf((c.left_target+c.right_target)*.5f-CAR_FOUR_CRUISE_SPEED)>.01f) {
            fputs("FAIL: task4 cruise slowed at an approach or segment boundary\n",stderr); return 1;
        }
        if(CAR_TIME_TRIAL_BUILD && ms!=c.run_at &&
           (c.state==CAR_STRAIGHT || c.state==CAR_CAPTURE || c.state==CAR_ARC) &&
           (c.left_target>CAR_TT_WHEEL_SPEED+.01f || c.right_target>CAR_TT_WHEEL_SPEED+.01f ||
            (c.left_target+c.right_target)*.5f<1 ||
            (c.left_target+c.right_target)*.5f>fmaxf(CAR_TT_STRAIGHT_SPEED,CAR_TT_ARC_SPEED)+.01f)) {
            fputs("FAIL: time-trial target outside planned envelope\n",stderr); return 1;
        }
        if(task>=3 && ms==2000 && p.theta-initial>-0.35f) {
            fputs("FAIL: no automatic right turn toward C\n",stderr); return 1;
        }
#if CAR_ARC_TEST_BUILD || CAR_LAP_TEST_BUILD || CAR_CROSS_TEST_BUILD || CAR_FOUR_TEST_BUILD || CAR_DUAL_MODE_BUILD
        if (injection==5 && injected) {
            if(gate.phase!=TASK1_ABORTED || c.state!=CAR_STOPPED || c.left_pwm || c.right_pwm) return 1;
            puts("PASS: physical button aborts arc immediately"); return 0;
        }
        if (CAR_ARC_TEST_BUILD && c.segment>1) { fputs("FAIL: arc test advanced beyond C\n",stderr); return 1; }
#endif
        if(c.state!=prev || c.segment!=seg) {
            if(getenv("SIM_TRACE")) printf("task%u t%.2f state%d fault%d seg%u laps%u true(%.0f,%.0f,%.1f) map(%.0f,%.0f) mask%02x\n",
                task,ms*.001,c.state,c.fault,c.segment,c.completed_laps,p.x,p.y,p.theta*180/CAR_PI,c.x_mm,c.y_mm,in.black_mask);
            prev=c.state; seg=c.segment;
        }
        if(c.point_events!=events) {
            events=c.point_events;
            if(task==4 || task==2) {
                const CarPoint cross_order[]={CAR_POINT_A,CAR_POINT_C,CAR_POINT_B,CAR_POINT_D};
                const CarPoint oval_order[]={CAR_POINT_A,CAR_POINT_B,CAR_POINT_C,CAR_POINT_D};
                const CarPoint *order=task==4 ? cross_order : oval_order;
                if(c.last_point!=order[(events-1)%4]) return 1;
            }
            if(!PointCovered(&p,c.last_point,&g)) {
                fprintf(stderr,"FAIL task%u: event point%d not physically covered at %.0f,%.0f\n",task,c.last_point,p.x,p.y);return 1;
            }
        }
        if(task==4 && !pose_displaced && b_pose_error_mm!=0 && c.segment==2 &&
           c.state==CAR_STRAIGHT && c.completed_laps==b_pose_error_lap) {
            /* Model an unobserved B-exit localization error independently of
             * the controller. Positive error misses D on its outside, leaving
             * the black arc to the right of the approaching car. This is a
             * robustness fixture, not a measurement of this run's error. */
            p.x+=b_pose_error_mm*0.624695f;p.y-=b_pose_error_mm*0.780869f;
            pose_displaced=true;
        }
        if(c.state==CAR_FAULT) {
            bool approach_stop=c.segment==2 && c.fault==CAR_FAULT_ENDPOINT &&
                c.point_events==4*c.completed_laps+3;
            bool entry_stop=c.segment==3 && c.fault==CAR_FAULT_LINE &&
                in.now_ms-c.missing_at<=CAR_LINE_LOST_TIMEOUT_MS+5U;
            if(pose_displaced && b_pose_error_mm>40 && (approach_stop || entry_stop) &&
               !c.left_pwm && !c.right_pwm) {
                puts("PASS: excessive D approach pose error stops with existing endpoint/line guards");
                return 0;
            }
            /* Frozen feedback can change steering until the line is lost.
             * The 700 ms stall clock requires continuous target >60 mm/s;
             * the inner-wheel target on the crossing arc drops below that.
             * Bound this coupled injected fault by 2 s, and separately keep
             * the direct wheel-stall detector tests. Other routes retain
             * their previously observed tighter bound. */
            bool expected=c.fault==expected_fault[injection] ||
                (injection==3 && c.fault==CAR_FAULT_LINE);
            uint32_t freeze_limit=task>=3 ? 2000U : CAR_STALL_TIMEOUT_MS+5U;
            bool bounded=injection!=3 || ms-injected_at<=freeze_limit;
            if(injected && expected && bounded && !c.left_pwm && !c.right_pwm) {
                in.gray_online=in.imu_ready=in.encoder_valid=true; in.pressed=false;
                for(unsigned k=0;k<30;k++) { in.now_ms+=5; CarControl_Update(&c,&in); }
                if(c.state!=CAR_FAULT) return 1;
                printf("PASS: injected fault %u stops and latches during arc\n",injection);
                return 0;
            }
            fprintf(stderr,"FAIL task%u t%.2f fault%d seg%u true(%.1f,%.1f,%.1f)\n",
                task,ms*.001,c.fault,c.segment,p.x,p.y,p.theta*180/CAR_PI); return 1;
        }
        if(!CAR_ARC_TEST_BUILD && task>=2 && c.completed_laps<laps && (c.state==CAR_BRAKING || CarControl_BrakeActive(&c,ms))) {
            fputs("FAIL: four-lap run braked before lap4\n",stderr); return 1;
        }
        if(c.state==CAR_DONE) {
            if(injection) return 1;
            unsigned expected=CAR_ARC_TEST_BUILD ? 3 : task==1 ? 2 : 4*laps+1;
#if CAR_ARC_TEST_BUILD || CAR_LAP_TEST_BUILD || CAR_CROSS_TEST_BUILD || CAR_FOUR_TEST_BUILD || CAR_DUAL_MODE_BUILD
            if(c.last_point!=(CAR_ARC_TEST_BUILD ? CAR_POINT_C : CAR_POINT_A) ||
               c.completed_laps!=(CAR_ARC_TEST_BUILD ? 0 : laps) || ms>=CAR_TEST_RUN_LIMIT_MS) return 1;
            for(unsigned k=0;k<100;k++) {
                in.now_ms+=5;in.enabled=true;in.pressed=false;
#if CAR_DUAL_MODE_BUILD
                CourseMode_Apply(&mode,&gate,&in,&c);
#else
                Task1Test_Apply(&gate,&in,c.state,1);
#endif
                CarControl_Update(&c,&in);
                if(in.enabled || c.left_pwm || c.right_pwm || gate.phase!=TASK1_FINISHED) return 1;
            }
#endif
            if(events!=expected || !PointCovered(&p,c.last_point,&g) || c.left_pwm || c.right_pwm) return 1;
            for(unsigned k=0;k<100;k++) { in.now_ms+=5; ++in.gray_samples; CarControl_Update(&c,&in); }
            if(c.state!=CAR_DONE || c.point_events!=events) return 1;
            printf("PASS: task%u %.2fs, %u point events, %u laps (motor gains %.2f/%.2f, yaw bias %.3f rad/s)\n",
                task,(ms-c.run_at)*.001,events,c.completed_laps,gain_l,gain_r,bias);
            if(getenv("SIM_METRICS")) printf("MODEL: pwm_per_mm_s=%.3f peak_pwm=%u saturation=%u/%u travel_mm=%.1f\n",
                plant_pwm_per_mm_s,peak_pwm,saturated_updates,active_updates,(p.left+p.right)*.5f);
            return 0;
        }
    }
    return 1;
}
int main(void)
{
#if CAR_DUAL_MODE_BUILD
    if(getenv("SIM_NOMINAL_ONLY")) {
        selected_laps=4;
        return Run(2,1,1,0,0) || Run(4,1,1,0,0);
    }
    if(getenv("SIM_D_ENTRY_SWEEP")) {
        const float offsets[]={-40,0,40,60};
        for(selected_laps=1;selected_laps<=4;selected_laps+=3)
            for(b_pose_error_lap=0;b_pose_error_lap<selected_laps;b_pose_error_lap++)
                for(unsigned i=0;i<sizeof(offsets)/sizeof(offsets[0]);i++) {
                    b_pose_error_mm=offsets[i];
                    if(Run(4,1,1,0,0) || Run(4,.95f,1.05f,.001f,0)) return 1;
                }
        puts("PASS: D entry pose-error sweep, 1/4 laps, each B exit, full coverage and speed envelope");
        return 0;
    }
    for(unsigned task=2;task<=4;task+=2) {
        for(selected_laps=1;selected_laps<=4;selected_laps++) {
            if(Run(task,1,1,0,0) || Run(task,.95f,1.05f,.001f,0)) return 1;
        }
        selected_laps=4;
        for(int degrees=-2;degrees<=2;degrees++) {
            placement_error_rad=(float)degrees*CAR_PI/180;
            if(Run(task,.95f,1.05f,.001f,0) || Run(task,1.05f,.95f,-.001f,0)) return 1;
        }
        placement_error_rad=0;
        for(fault_lap=0;fault_lap<=3;fault_lap+=3) {
            for(unsigned fault=1;fault<=5;fault++) if(Run(task,1,1,0,fault)) return 1;
        }
        fault_lap=0;
    }
#elif CAR_ARC_TEST_BUILD || CAR_LAP_TEST_BUILD || CAR_CROSS_TEST_BUILD || CAR_FOUR_TEST_BUILD
    unsigned task=CAR_FOUR_TEST_BUILD ? 4 : CAR_CROSS_TEST_BUILD ? 3 : 2;
    if(Run(task,1,1,0,0) || Run(task,.95f,1.05f,.001f,0)) return 1;
    if(task>=3) {
        placement_error_rad=2*CAR_PI/180;
        if(Run(task,1,1,0,0)) return 1;
        placement_error_rad=-2*CAR_PI/180;
        if(Run(task,1,1,0,0)) return 1;
        placement_error_rad=0;
    }
    if(task==4 && getenv("SIM_SWEEP")) {
        /* Combine placement and unequal motor response instead of checking
         * each disturbance only in isolation when selecting a faster tune. */
        for(int degrees=-2;degrees<=2;degrees++) {
            placement_error_rad=(float)degrees*CAR_PI/180;
            if(Run(task,.95f,1.05f,.001f,0) || Run(task,1.05f,.95f,-.001f,0)) return 1;
        }
        placement_error_rad=0;
    }
    for(unsigned fault=1;fault<=5;fault++) if(Run(task,1,1,0,fault)) return 1;
    if(task==4) {
        fault_lap=3;
        for(unsigned fault=1;fault<=5;fault++) if(Run(task,1,1,0,fault)) return 1;
        fault_lap=0;
    }
#else
    for(unsigned task=1;task<=4;task++) if(Run(task,1,1,0,0)) return 1;
    for(unsigned task=1;task<=4;task++) if(Run(task,.95f,1.05f,.001f,0)) return 1;
    for(unsigned fault=1;fault<=4;fault++) if(Run(2,1,1,0,fault)) return 1;
#endif
    return 0;
}
