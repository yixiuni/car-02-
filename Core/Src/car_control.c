#include "car_control.h"
#include "car_config.h"
#include <math.h>
#if CAR_TIME_TRIAL_EXIT_TRACE
#include "car_arc_debug.h"
volatile CarArcDebug car_arc_debug[CAR_ARC_DEBUG_CAPACITY];
_Static_assert(sizeof(CarArcDebug)==116, "Exit trace layout");
static volatile CarArcDebug *ArcDebugSlot(const CarControl *c)
{
    if (c->point_events<2 || (c->point_events&1U)) return 0;
    unsigned slot=c->point_events/2-1;
    if (slot>=CAR_ARC_DEBUG_CAPACITY) return 0;
    volatile CarArcDebug *d=&car_arc_debug[slot];
    d->point_events=c->point_events;return d;
}
#endif

/* Map: A=(0,0), B=(1000,0), C=(1000,-800), D=(0,-800).
 * Heading positive CCW. Start axle at A facing B for every task. */
typedef struct { float x, y; } Point;
static const Point points[] = {{0,0},{1000,0},{1000,-800},{0,-800}};
static const CarPoint oval[] = {CAR_POINT_A,CAR_POINT_B,CAR_POINT_C,CAR_POINT_D,CAR_POINT_A};
static const CarPoint cross[] = {CAR_POINT_A,CAR_POINT_C,CAR_POINT_B,CAR_POINT_D,CAR_POINT_A};
static const CarPoint *Route(const CarControl *c) { return c->task <= 2 ? oval : cross; }
static bool FastCourse(const CarControl *c)
{
    return c->task==4 || (CAR_DUAL_MODE_BUILD && c->task==2);
}
static bool CrossSprint(const CarControl *c)
{
    return CAR_TIME_TRIAL_CROSS_SPRINT && c->task==4;
}
static unsigned TargetLaps(const CarControl *c)
{
    return c->target_laps>=1 && c->target_laps<=4 ? c->target_laps : c->task==4 ? 4U : 1U;
}
static float Clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo,x)); }
bool CarControl_Active(const CarControl *c)
{
    return c->state >= CAR_STRAIGHT && c->state <= CAR_BRAKING;
}
bool CarControl_BrakeActive(const CarControl *c, uint32_t now)
{
    return c->state==CAR_BRAKING && c->fault==CAR_OK &&
           (uint32_t)(now-c->brake_at)<CAR_BRAKE_HOLD_MS;
}
static bool FinalSegment(const CarControl *c)
{
#if CAR_ARC_TEST_BUILD
    if(c->task==2 && c->segment==1) return true;
#endif
    return c->task==1 || (c->segment==3 && (c->completed_laps+1U>=TargetLaps(c)));
}
static void Stop(CarControl *c, CarState state, CarFault fault)
{
    c->state = state; c->fault = fault;
    c->left_pwm = c->right_pwm = 0;
    c->left_target = c->right_target = 0;
    c->left_stall = c->right_stall = false;
    Motion_Reset(&c->motion);
}
static void Signal(CarControl *c, CarPoint point)
{
    c->last_point = point; ++c->point_events;
}
static bool CoversPose(const CarControl *c, Point p, float x, float y, float heading)
{
    float dx=p.x-x, dy=p.y-y;
    float forward=cosf(heading)*dx+sinf(heading)*dy;
    float lateral=-sinf(heading)*dx+cosf(heading)*dy;
    return forward <= c->geometry.front_mm && forward >= -c->geometry.rear_mm &&
           fabsf(lateral) <= c->geometry.half_width_mm;
}
static bool Covers(const CarControl *c, Point p)
{
    return CoversPose(c,p,c->x_mm,c->y_mm,c->heading_rad);
}
static CarFault InputFault(const CarControl *c, const CarInputs *in)
{
    if (!CarGeometry_Valid(&c->geometry)) return CAR_FAULT_CONFIG;
    if (CAR_TIME_TRIAL_BUILD &&
        !(CAR_TT_ENTRY_SPEED>0 && CAR_TT_ENTRY_SPEED<=CAR_TT_ARC_SPEED &&
          CAR_TT_ENTRY_SPEED<=CAR_TT_STRAIGHT_SPEED && CAR_TT_ARC_SPEED<=1000 &&
          CAR_TT_STRAIGHT_SPEED<=1000 && CAR_TT_WHEEL_SPEED>=CAR_TT_ENTRY_SPEED &&
          CAR_TT_WHEEL_SPEED<=1600 && CAR_TT_STRAIGHT_DECEL>0 && CAR_TT_STRAIGHT_DECEL<=5000 &&
          CAR_TT_STRAIGHT_PWM_RESERVE>=0 && CAR_TT_STRAIGHT_PWM_RESERVE<=CAR_PWM_LIMIT &&
          CAR_TT_ARC_PWM_RESERVE>=0 && CAR_TT_ARC_PWM_RESERVE<=CAR_PWM_LIMIT &&
          CAR_TT_ARC_ACCEL_START_MM>=0 && CAR_TT_ARC_ACCEL_START_MM<=400 &&
          CAR_TT_D_FILTER_MS>=0 && CAR_TT_D_FILTER_MS<=100))
        return CAR_FAULT_CONFIG;
    if (CrossSprint(c) &&
        !(CAR_CROSS_SPRINT_STRAIGHT_SPEED>=CAR_TT_ENTRY_SPEED &&
          CAR_CROSS_SPRINT_STRAIGHT_SPEED<=CAR_TT_WHEEL_SPEED &&
          CAR_CROSS_SPRINT_ARC_SPEED>=CAR_TT_ENTRY_SPEED &&
          CAR_CROSS_SPRINT_ARC_SPEED<=CAR_TT_WHEEL_SPEED &&
          CAR_CROSS_SPRINT_ARC_PWM>0 && CAR_CROSS_SPRINT_ARC_PWM<=CAR_PWM_LIMIT))
        return CAR_FAULT_CONFIG;
    if (CAR_TIME_TRIAL_ARC_LOCALIZE &&
        !(CAR_ARC_POSITION_GAIN>0 && CAR_ARC_POSITION_GAIN<=0.5f &&
          CAR_ARC_POSITION_STEP_MM>0 && CAR_ARC_POSITION_STEP_MM<=8.0f &&
          CAR_ARC_POSITION_INNOVATION_MM>0 && CAR_ARC_POSITION_INNOVATION_MM<=120.0f))
        return CAR_FAULT_CONFIG;
    if (!in->gray_online) return CAR_FAULT_GRAY;
    if (!in->imu_ready || !isfinite(in->yaw_rad)) return CAR_FAULT_IMU;
    if (!in->encoder_valid || !isfinite(in->left_mm) || !isfinite(in->right_mm) ||
        !isfinite(in->left_speed) || !isfinite(in->right_speed)) return CAR_FAULT_ENCODER;
    return CAR_OK;
}
static void BeginSegment(CarControl *c, uint32_t now)
{
    c->segment_at=now; c->segment_distance=c->distance_mm;
#if CAR_TIME_TRIAL_EDGE_POSE
    c->edge_valid=c->edge_ready=false;
#endif
    c->have_line=false; c->line_missing=false;
    c->black_frames=c->white_frames=0;
    c->line_error=c->line_rate=0;
}
static void Start(CarControl *c, const CarInputs *in)
{
    CarFault f=InputFault(c,in);
    if (f!=CAR_OK) { Stop(c,CAR_FAULT,f); return; }
#if CAR_TIME_TRIAL_EXIT_TRACE
    for(unsigned i=0;i<CAR_ARC_DEBUG_CAPACITY;i++) car_arc_debug[i]=(CarArcDebug){0};
#endif
    c->segment=0; c->completed_laps=0; c->distance_mm=0;
    c->x_mm=c->y_mm=0;
    /* Common physical reference: A facing B. Cross tasks turn forward toward
     * C; integrate that motion rather than relabeling the initial heading. */
    c->heading_rad=0.0f;
    c->yaw_offset=c->heading_rad-in->yaw_rad;
    c->last_left_mm=in->left_mm; c->last_right_mm=in->right_mm;
    c->last_gray_samples=in->gray_samples;
    c->run_at=in->now_ms; c->last_update=in->now_ms;
    c->state=CAR_STRAIGHT; c->fault=CAR_OK;
    Motion_Reset(&c->motion); BeginSegment(c,in->now_ms);
    Signal(c,CAR_POINT_A);
}
/* Short release starts; long press selects task only when stopped. During
 * motion the debounced press stops immediately, and its release cannot restart.
 * A button held at boot must first be released. */
static bool Button(CarControl *c, const CarInputs *in)
{
    uint32_t now=in->now_ms;
    if (in->pressed!=c->button_raw) { c->button_raw=in->pressed; c->button_changed_at=now; }
    if (now-c->button_changed_at>=CAR_BUTTON_DEBOUNCE_MS && c->button_stable!=c->button_raw) {
        c->button_stable=c->button_raw;
        if (c->button_stable) {
            c->pressed_at=now;
            c->hold_consumed=!c->button_armed || !in->enabled;
            if (CarControl_Active(c)) {
                Stop(c,CAR_STOPPED,CAR_OK); c->hold_consumed=true;
            }
        } else {
            bool start=c->button_armed && !c->hold_consumed;
            c->button_armed=true;
            if (start && in->enabled) { Start(c,in); return true; }
        }
    }
    if (c->button_stable && c->button_armed && !c->hold_consumed &&
        !CarControl_Active(c) && now-c->pressed_at>=CAR_BUTTON_HOLD_MS) {
        c->task=(uint8_t)(c->task%4U+1U); ++c->mode_events;
        c->hold_consumed=true; Stop(c,CAR_STOPPED,CAR_OK);
    }
    return false;
}
void CarControl_Init(CarControl *c,const CarGeometry *g,uint32_t now,bool pressed)
{
    *c=(CarControl){0}; c->geometry=*g; c->task=CAR_DEFAULT_TASK;
    c->button_raw=c->button_stable=pressed; c->button_armed=!pressed;
    c->hold_consumed=pressed; c->button_changed_at=c->last_update=now;
}
static void Gray(CarControl *c,const CarInputs *in)
{
    if (in->gray_samples==c->last_gray_samples) return;
    c->last_gray_samples=in->gray_samples;
    /* All-black is ambiguous (not a legal course marker); never use as a point. */
    bool black=in->black_mask!=0 && in->black_mask!=0xFF;
    if (black) {
        float sum=0; unsigned n=0;
        for (unsigned i=0;i<8;i++) if (in->black_mask&(1U<<i)) { sum+=(float)i-3.5f; ++n; }
        float error=sum*CAR_GRAY_PITCH_MM/(float)n;
        uint32_t elapsed=in->now_ms-c->line_at;
        float rate=c->have_line && elapsed>0 && elapsed<100 ?
            (error-c->line_error)*1000.0f/(float)elapsed : 0;
        if (CAR_TIME_TRIAL_BUILD && FastCourse(c) && c->have_line && elapsed>0 && elapsed<100)
            c->line_rate+=(float)elapsed/((float)elapsed+CAR_TT_D_FILTER_MS)*(rate-c->line_rate);
        else c->line_rate=rate;
        c->line_error=error; c->line_at=c->last_line_at=in->now_ms;
        c->have_line=true; c->line_missing=false; c->white_frames=0;
#if CAR_TIME_TRIAL_EXIT_TRACE
        if (c->state==CAR_ARC && c->distance_mm-c->segment_distance>=c->geometry.sensor_forward_mm &&
            fabsf(c->line_error)<=1.5f*CAR_GRAY_PITCH_MM) {
            volatile CarArcDebug *d=ArcDebugSlot(c);
            if (d) {
                float cs=cosf(c->heading_rad),sn=sinf(c->heading_rad);
                float px=c->x_mm+c->geometry.sensor_forward_mm*cs+c->line_error*sn;
                float py=c->y_mm+c->geometry.sensor_forward_mm*sn-c->line_error*cs;
                float cx=c->segment==1 ? 1000.0f : 0.0f;
                float residual=hypotf(px-cx,py+400.0f)-400.0f;
                d->last_radial_error_mm=residual;
                d->max_abs_radial_error_mm=fmaxf(d->max_abs_radial_error_mm,fabsf(residual));
                ++d->black_samples;
            }
        }
#endif
        if (c->black_frames<255) ++c->black_frames;
    } else {
        c->black_frames=0;
        /* Wide black is invalid, not an arc-end white edge. */
        if (in->black_mask==0) { if (c->white_frames<255) ++c->white_frames; }
        else c->white_frames=0;
        if (!c->line_missing) { c->line_missing=true; c->missing_at=in->now_ms; }
    }
}
static void ObserveArcPosition(CarControl *c,const CarInputs *in,bool fresh)
{
    if (!CAR_TIME_TRIAL_ARC_LOCALIZE || !CrossSprint(c) || c->state!=CAR_ARC ||
        !fresh || c->black_frames<3 ||
        c->distance_mm-c->segment_distance<c->geometry.sensor_forward_mm ||
        fabsf(c->line_error)>1.5f*CAR_GRAY_PITCH_MM) return;
    /* One narrow contiguous band, not split/all-black or repeated old data. */
    uint8_t band=in->black_mask;
    if (!band) return;
    while (!(band&1U)) band>>=1;
    if (band!=1U && band!=3U && band!=7U) return;
    float cs=cosf(c->heading_rad),sn=sinf(c->heading_rad);
    float cx=c->segment==1 ? 1000.0f : 0.0f;
    float sx=c->x_mm+c->geometry.sensor_forward_mm*cs+c->line_error*sn;
    float sy=c->y_mm+c->geometry.sensor_forward_mm*sn-c->line_error*cs;
    float dx=sx-cx,dy=sy+400.0f,r=hypotf(dx,dy);
    bool half=c->segment==1 ? dx>=0 : dx<=0;
    if (!half || r<400.0f-CAR_ARC_POSITION_INNOVATION_MM ||
        r>400.0f+CAR_ARC_POSITION_INNOVATION_MM) return;
    float step=Clamp(CAR_ARC_POSITION_GAIN*(400.0f-r),
                     -CAR_ARC_POSITION_STEP_MM,CAR_ARC_POSITION_STEP_MM);
    /* Radial position observation only. Yaw, traveled distance, endpoint
     * gates and loss timeouts are unchanged; white cannot update position. */
    c->x_mm+=step*dx/r;c->y_mm+=step*dy/r;
#if CAR_TIME_TRIAL_ARC_PHASE

    /* In settled centered tracking, the axle lies on the smaller concentric
     * circle and its heading is tangent. This supplies the missing along-circle
     * constraint; never apply this prior during acquisition, white or transients. */
    if (c->distance_mm-c->segment_distance>=300.0f && c->black_frames>=4 &&
        fabsf(c->line_error)<=0.5f*CAR_GRAY_PITCH_MM && fabsf(c->line_rate)<=200.0f) {
        float sign=c->segment==1 ? 1.0f : -1.0f;
        float radius=sqrtf(400.0f*400.0f-c->geometry.sensor_forward_mm*c->geometry.sensor_forward_mm);
        float axle_radius=radius-sign*c->line_error;
        float tx=cx+sign*axle_radius*sn,ty=-400.0f-sign*axle_radius*cs;
        float sensor_x=tx+c->geometry.sensor_forward_mm*cs+c->line_error*sn;
        bool target_half=c->segment==1 ? sensor_x>=cx : sensor_x<=cx;
        float tangent_x=-dy/r,tangent_y=dx/r;
        float innovation=(tx-c->x_mm)*tangent_x+(ty-c->y_mm)*tangent_y;
        if (target_half && fabsf(innovation)<=160.0f) {
            float shift=Clamp(0.1f*innovation,-2.0f,2.0f);
            c->x_mm+=shift*tangent_x;c->y_mm+=shift*tangent_y;
        }
    }
#endif

}
#if CAR_TIME_TRIAL_EDGE_POSE

static void TrackEdgePose(CarControl *c,const CarInputs *in,bool fresh,bool was_missing)
{
    if (!CrossSprint(c) || !fresh) return;
    uint8_t band=in->black_mask;
    if (band && band!=255) {
        while (!(band&1U)) band>>=1;
        c->edge_valid=(band==1 || band==3 || band==7) && c->black_frames>=3 &&
            fabsf(c->line_error)<=1.5f*CAR_GRAY_PITCH_MM;
        c->edge_ready=false;
        c->edge_x=c->x_mm;c->edge_y=c->y_mm;c->edge_heading=c->heading_rad;
    } else if (!band && !was_missing) {
        c->edge_valid=c->edge_valid && in->now_ms-c->last_line_at<=40;
        c->edge_ready=c->edge_valid;
        c->edge_x=(c->edge_x+c->x_mm)*0.5f;
        c->edge_y=(c->edge_y+c->y_mm)*0.5f;
        c->edge_heading+=(c->heading_rad-c->edge_heading)*0.5f;
    } else if (band) {
        c->edge_valid=c->edge_ready=false;
    }
}
#endif

static void Drive(CarControl *c,float speed,float omega,float dt,const CarInputs *in)
{
    /* Differential drive forward only; curvature is limited before mixing. */
    omega=Clamp(omega,-2*speed/c->geometry.track_mm,2*speed/c->geometry.track_mm);
    c->left_target=fmaxf(0,speed-omega*c->geometry.track_mm*0.5f);
    c->right_target=fmaxf(0,speed+omega*c->geometry.track_mm*0.5f);
    if (CAR_TIME_TRIAL_BUILD && FastCourse(c)) {
        /* Scale both wheels together to retain requested curvature. Independent
         * clipping would straighten the turn when the outside wheel saturates.
         * This speed ceiling is an experiment, not a measured motor capability. */
        float peak=fmaxf(c->left_target,c->right_target);
        if (peak>CAR_TT_WHEEL_SPEED) {
            c->left_target*=CAR_TT_WHEEL_SPEED/peak;
            c->right_target*=CAR_TT_WHEEL_SPEED/peak;
        }
    }
    float pwm_budget=c->state==CAR_STRAIGHT ? CAR_TT_STRAIGHT_PWM_RESERVE :
        c->state==CAR_ARC ? (CrossSprint(c) ? CAR_CROSS_SPRINT_ARC_PWM : CAR_TT_ARC_PWM_RESERVE) : 0;
    if (CAR_TIME_TRIAL_BUILD && FastCourse(c) && pwm_budget>0) {
        /* Invert the current wheel PI demand to reserve steering authority.
         * Common scaling keeps path curvature when a weak/battery-limited
         * wheel cannot deliver the requested straight or arc speed. No plant gain
         * is assumed; measured speed and each PI integral determine the cap. */
        float left_available=fmaxf(0,(pwm_budget+
            CAR_SPEED_KP*in->left_speed-c->motion.left.integral)/(CAR_SPEED_FF+CAR_SPEED_KP));
        float right_available=fmaxf(0,(pwm_budget+
            CAR_SPEED_KP*in->right_speed-c->motion.right.integral)/(CAR_SPEED_FF+CAR_SPEED_KP));
        float scale=1.0f;
        if (c->left_target>0) scale=fminf(scale,left_available/c->left_target);
        if (c->right_target>0) scale=fminf(scale,right_available/c->right_target);
        c->left_target*=scale;c->right_target*=scale;
    }
    Motion_UpdateWithAccel(&c->motion,c->left_target,c->right_target,in->left_speed,in->right_speed,
                  dt,FastCourse(c) ? CAR_FOUR_ACCEL_MM_S2 : CAR_ACCEL_MM_S2,&c->left_pwm,&c->right_pwm);
}
static bool StallWheel(bool *pending,uint32_t *at,float target,float measured,uint32_t now)
{
    bool bad=target>60 && measured<5;
    if (!bad) { *pending=false; return false; }
    if (!*pending) { *pending=true; *at=now; }
    return now-*at>=CAR_STALL_TIMEOUT_MS;
}
static void Brake(CarControl *c,uint32_t now)
{
    Stop(c,CAR_BRAKING,CAR_OK); c->brake_at=now; c->settled=false;
}
static float ArcStart(const CarControl *c) { return c->segment==1 ? 0.0f : CAR_PI; }
static float ArcEnd(const CarControl *c) { return c->segment==1 ? CAR_PI : 0.0f; }
static float ArcSign(const CarControl *c) { return c->task>=3 && c->segment==1 ? 1.0f : -1.0f; }
static float FastArcSpeed(const CarControl *c)
{
    if (CrossSprint(c)) return CAR_CROSS_SPRINT_ARC_SPEED;
    return CAR_TIME_TRIAL_BUILD ? CAR_TT_ARC_SPEED : CAR_FOUR_CRUISE_SPEED;
}
static float FastEntrySpeed(void)
{
    return CAR_TIME_TRIAL_BUILD ? CAR_TT_ENTRY_SPEED : CAR_FOUR_CRUISE_SPEED;
}
static float ArcRadius(const CarControl *c)
{
    /* The front sensor follows the 400 mm circle; its axle follows a smaller
     * circle. Use that axle radius for feedforward, with gray feedback intact. */
    return CAR_TIME_TRIAL_BUILD ?
        sqrtf(400.0f*400.0f-c->geometry.sensor_forward_mm*c->geometry.sensor_forward_mm) : 400.0f;
}
static float ArcFeedback(const CarControl *c,float speed,bool derivative)
{
    float gain=FastCourse(c) ? speed/180.0f : 1.0f;
    if (!CAR_TIME_TRIAL_BUILD)
        return gain*(CAR_LINE_KP*c->line_error+(derivative ? CAR_LINE_KD*c->line_rate : 0));
    /* d(error)/dt already grows with forward speed. Holding its angular gain
     * at the reference speed avoids amplifying the same spatial disturbance
     * twice when cruise is raised. P still scales with speed. */
    float d_gain=CAR_TIME_TRIAL_BUILD && FastCourse(c) ? CAR_FOUR_CRUISE_SPEED/180.0f : gain;
    /* Keep acquisition feedback intact. During settled CROSS tracking, reduce
     * binary-probe steering steps and derivative kicks at the higher cruise. */
    bool cruise=CrossSprint(c) && c->state==CAR_ARC &&
        c->distance_mm-c->segment_distance>=c->geometry.sensor_forward_mm;
    float pscale=cruise ? CAR_CROSS_SPRINT_P_SCALE : 1.0f;
    float dscale=cruise ? CAR_CROSS_SPRINT_D_SCALE : 1.0f;
    return pscale*gain*CAR_LINE_KP*c->line_error+
        (derivative ? dscale*d_gain*CAR_LINE_KD*c->line_rate : 0);
}

/* During fast acquisition keep both target wheels moving. A saturated turn
 * on an outer-probe observation can pivot past the line before a fresh frame. */
static float EntryOmega(const CarControl *c,float speed,float omega)
{
    if (!CAR_TIME_TRIAL_STAGE3) return omega;
    float limit=speed/c->geometry.track_mm; /* inner target >= half center speed */
    return Clamp(omega,-limit,limit);
}

void CarControl_Update(CarControl *c,const CarInputs *in)
{
    if (!in->enabled) {
        if (CarControl_Active(c)) Stop(c,CAR_STOPPED,CAR_OK);
        /* Pressing while disabled cannot queue a start on re-enable. */
        if (in->pressed || c->button_stable) c->hold_consumed=true;
    }
    bool started=Button(c,in);
    if (!CarControl_Active(c)) { c->last_update=in->now_ms; return; }
    if (!in->enabled) { Stop(c,CAR_STOPPED,CAR_OK); return; }
    CarFault fault=InputFault(c,in);
    if (fault!=CAR_OK) { Stop(c,CAR_FAULT,fault); return; }
    uint32_t elapsed=in->now_ms-c->last_update;
    if (started || elapsed<CAR_CONTROL_PERIOD_MS) return;
    if (elapsed>CAR_CONTROL_MAX_GAP_MS) { Stop(c,CAR_FAULT,CAR_FAULT_TIMING); return; }
    c->last_update=in->now_ms;
    float dt=(float)elapsed*0.001f;
    float dl=in->left_mm-c->last_left_mm, dr=in->right_mm-c->last_right_mm;
    /* Impossible jumps / wrong encoder sign are not allowed to advance route. */
    if (fabsf(dl)>2000*dt || fabsf(dr)>2000*dt || dl < -10 || dr < -10) {
        Stop(c,CAR_FAULT,CAR_FAULT_ENCODER); return;
    }
    c->last_left_mm=in->left_mm; c->last_right_mm=in->right_mm;
    float heading=in->yaw_rad+c->yaw_offset;
    float turn=Motion_Wrap(heading-c->heading_rad);
    if (fabsf(turn)>12.0f*dt) { Stop(c,CAR_FAULT,CAR_FAULT_IMU); return; }
    float ds=(dl+dr)*0.5f, mid=c->heading_rad+turn*0.5f;
    c->x_mm+=ds*cosf(mid); c->y_mm+=ds*sinf(mid);
    c->heading_rad+=turn; c->distance_mm+=ds;
    bool fresh_gray=in->gray_samples!=c->last_gray_samples;
#if CAR_TIME_TRIAL_EDGE_POSE
    bool was_missing=c->line_missing;
#endif
    Gray(c,in);
    ObserveArcPosition(c,in,fresh_gray);
#if CAR_TIME_TRIAL_EDGE_POSE
    TrackEdgePose(c,in,fresh_gray,was_missing);
#endif
    /* Task 4 has no stated fixed deadline; 160 s is our recoverable watchdog,
     * not a claim about the competition rules. */
    uint32_t limit=c->task==1 ? 15000U : c->task==2 ? (TargetLaps(c)>1 ? 160000U : 30000U) : c->task==3 ? 40000U : 160000U;
    if (in->now_ms-c->run_at>=limit) { Stop(c,CAR_FAULT,CAR_FAULT_TIMEOUT); return; }
    CarPoint dest=Route(c)[c->segment+1]; Point end=points[dest];
    float end_distance=hypotf(end.x-c->x_mm,end.y-c->y_mm);
    if (c->state==CAR_BRAKING) {
        if (fabsf(in->left_speed)<10 && fabsf(in->right_speed)<10) {
            if (!c->settled) { c->settled=true; c->settled_at=in->now_ms; }
            if (in->now_ms-c->settled_at>=120) {
                if (!Covers(c,end)) Stop(c,CAR_FAULT,CAR_FAULT_ENDPOINT);
                else { Signal(c,dest); Stop(c,CAR_DONE,CAR_OK); }
            }
        } else c->settled=false;
        if (c->state==CAR_BRAKING && in->now_ms-c->brake_at>2000)
            Stop(c,CAR_FAULT,CAR_FAULT_ENDPOINT);
        return;
    }
    float speed=0,omega=0;
    if (c->state==CAR_STRAIGHT) {
        Point begin=points[Route(c)[c->segment]];
        float dx=end.x-begin.x, dy=end.y-begin.y, length=hypotf(dx,dy);
        float ux=dx/length, uy=dy/length;
        float along=(c->x_mm-begin.x)*ux+(c->y_mm-begin.y)*uy;
        float lateral=-uy*(c->x_mm-begin.x)+ux*(c->y_mm-begin.y);
        float remaining=length-along;
        /* Position + fresh black + footprint gate. Black elsewhere cannot switch. */
        /* At C a right-edge-only hit can be the arc beside the body while C
         * remains outside its footprint. Continue preview steering, but wait
         * for the observation to move inward before confirming this point.
         * Left-side acquisition is different geometry and retains its gate. */
        bool entry_line_confirmed=!CrossSprint(c) ||
            (c->segment==0 ? c->line_error<=CAR_CROSS_SPRINT_EDGE_LIMIT :
                            c->line_error>=-CAR_CROSS_SPRINT_EDGE_LIMIT);
        bool reached=remaining<CAR_ENDPOINT_GATE_MM && remaining>-80 &&
            end_distance<CAR_ENDPOINT_GATE_MM && Covers(c,end) &&
            c->black_frames>=CAR_LINE_CONFIRM_FRAMES && entry_line_confirmed;
        if (reached) {
            if (c->task==1) { Brake(c,in->now_ms); return; }
            Signal(c,dest); ++c->segment;
            if (FastCourse(c)) {
                /* The approach already confirmed three fresh black frames.
                 * Keep that observation and steer on the arc immediately;
                 * waiting to align at cruise would carry the probes past it. */
                c->segment_at=in->now_ms; c->segment_distance=c->distance_mm;
                c->arc_heading_start=c->heading_rad; c->state=CAR_ARC;
            } else {
                BeginSegment(c,in->now_ms); c->state=CAR_CAPTURE;
            }
        } else {
            if (remaining < -80 || fabsf(lateral)>180) {
                Stop(c,CAR_FAULT,CAR_FAULT_ENDPOINT); return;
            }
            float lookahead=CrossSprint(c) ? CAR_CROSS_SPRINT_LOOKAHEAD_MM : 250.0f;
            float desired=atan2f(dy,dx)-atan2f(lateral,lookahead);
            if(c->task==4 && (c->segment==2 ||
                (c->segment==0 && CAR_CROSS_C_ENTRY_AIM_MM>0))) {
                /* Aim along the incoming black arc, not only its exposed tip.
                 * All point gates remain referenced to the actual C/D endpoint:
                 * fresh black and physical footprint coverage are still required. */
                float advance=Clamp(c->segment==2 ? CAR_CROSS_D_ENTRY_AIM_MM :
                                    CAR_CROSS_C_ENTRY_AIM_MM,0.0f,60.0f);
                if (CrossSprint(c)) advance=c->segment==2 ? CAR_CROSS_SPRINT_D_AIM_MM : CAR_CROSS_SPRINT_C_AIM_MM;
                if (CAR_TIME_TRIAL_EDGE_POSE && CrossSprint(c)) advance=CAR_EDGE_POSE_ENTRY_AIM_MM;
                float ax=end.x+(c->segment==2 ? -advance : advance)-begin.x;
                float ay=end.y+400.0f-sqrtf(400.0f*400.0f-advance*advance)-begin.y;
                float aim_length=hypotf(ax,ay);
                float aim_lateral=-(ay/aim_length)*(c->x_mm-begin.x)+
                                   (ax/aim_length)*(c->y_mm-begin.y);
                desired=atan2f(ay,ax)-atan2f(aim_lateral,lookahead);
            }
            omega=CAR_HEADING_KP*Motion_Wrap(desired-c->heading_rad);
            speed=FastCourse(c) ? CAR_FOUR_CRUISE_SPEED : c->task<=2 ? CAR_STRAIGHT_SPEED : CAR_DIAGONAL_SPEED;
            if (CAR_TIME_TRIAL_BUILD && FastCourse(c)) {
                float heading_error=Motion_Wrap(desired-c->heading_rad);
                /* Accelerate only after aligning. The distance envelope reaches
                 * arc speed before the front sensor can see the endpoint. */
                float aligned=Clamp((0.55f-fabsf(heading_error))/0.35f,0,1);
                float cruise=CrossSprint(c) ? CAR_CROSS_SPRINT_STRAIGHT_SPEED : CAR_TT_STRAIGHT_SPEED;
                float requested=CAR_TT_ENTRY_SPEED+
                    aligned*(cruise-CAR_TT_ENTRY_SPEED);
                float available=fmaxf(0,remaining-c->geometry.sensor_forward_mm-60.0f);
                speed=fminf(requested,sqrtf(CAR_TT_ENTRY_SPEED*CAR_TT_ENTRY_SPEED+
                                            2*CAR_TT_STRAIGHT_DECEL*available));
                /* Speed-scaled heading feedback preserves geometric turning
                 * response when accelerating, unlike a fixed angular gain. */
                omega=CAR_HEADING_KP*(speed/CAR_FOUR_CRUISE_SPEED)*heading_error;
            }
            if(CAR_DUAL_MODE_BUILD && FastCourse(c) && c->black_frames>0 &&
               remaining<CAR_ENDPOINT_GATE_MM && remaining>-80 &&
               end_distance<CAR_ENDPOINT_GATE_MM &&
               (Covers(c,end) || (CAR_CROSS_C_ENTRY_PREVIEW && c->task==4 && c->segment==0) ||
                ((CAR_CROSS_D_ENTRY_PREVIEW || CrossSprint(c)) && c->task==4 && c->segment==2))) {
                /* Near an endpoint, start tracking the observed arc on
                 * its first valid black frame. Do not signal/advance the route
                 * until the usual three consecutive frames confirm it above.
                 * A white/ambiguous frame cancels this provisional steering. */
                float sign=c->task>=3 && c->segment==0 ? 1.0f : -1.0f;
                if (CAR_TIME_TRIAL_BUILD) speed=FastEntrySpeed();
                omega=EntryOmega(c,speed,sign*speed/ArcRadius(c)-ArcFeedback(c,speed,true));
            }
            /* Gentle initial turn from A-B to the crossing line. Both wheels
             * stay forward; odometry includes the curved departure from A. */
            if (c->task==3 && c->segment==0 && c->distance_mm<250 &&
                fabsf(Motion_Wrap(desired-c->heading_rad))>0.20f)
                speed=fminf(speed,CAR_CAPTURE_SPEED);
            if (!FastCourse(c) && remaining<250 && !FinalSegment(c)) speed=fminf(speed,fmaxf(CAR_CAPTURE_SPEED,
                sqrtf(2*CAR_ACCEL_MM_S2*fmaxf(0,remaining-c->geometry.sensor_forward_mm))));
        }
    }
    if (c->state==CAR_CAPTURE) {
        float offset=asinf(c->geometry.sensor_forward_mm/400.0f);
        float relative=ArcSign(c)*Motion_Wrap(c->heading_rad-ArcStart(c));
        float desired=ArcStart(c)-ArcSign(c)*offset;
        speed=FastCourse(c) ? FastEntrySpeed() : CAR_CAPTURE_SPEED;
        omega=CAR_HEADING_KP*Motion_Wrap(desired-c->heading_rad);
        /* A forward sensor reaches the arc before the axle. Accept the
         * interval between the entry tangent and the settled body heading,
         * avoiding a forced tangent alignment that sweeps the sensor off-line. */
        if (c->black_frames>=CAR_LINE_CONFIRM_FRAMES &&
            relative>=-offset-0.20f && relative<=0.20f) {
            c->state=CAR_ARC; c->arc_heading_start=c->heading_rad;
            /* Keep capture travel in the minimum-length gate. */
        }
        if (in->now_ms-c->segment_at>CAR_CAPTURE_TIMEOUT_MS) {
            Stop(c,CAR_FAULT,CAR_FAULT_LINE); return;
        }
    }
    if (c->state==CAR_ARC) {
        end=points[Route(c)[c->segment+1]];
        end_distance=hypotf(end.x-c->x_mm,end.y-c->y_mm);
        float progress=c->distance_mm-c->segment_distance;
        float turned=ArcSign(c)*(c->heading_rad-c->arc_heading_start);
        /* Compare body heading with body heading, including the forward sensor
         * offset; the endpoint tangent can differ by over 24 degrees. */
        float observed=ArcEnd(c)-ArcSign(c)*asinf(c->geometry.sensor_forward_mm/400.0f);
        float edge_heading_for_gate=c->heading_rad;
#if CAR_TIME_TRIAL_EDGE_POSE
        if (CrossSprint(c) && c->edge_ready) edge_heading_for_gate=c->edge_heading;
#endif
        float heading_error=fabsf(Motion_Wrap(edge_heading_for_gate-observed));
        bool exit=c->have_line && c->white_frames>=CAR_LINE_CONFIRM_FRAMES &&
            in->now_ms-c->last_line_at<100 && fabsf(c->line_error)<=1.5f*CAR_GRAY_PITCH_MM &&
            progress>CAR_ARC_MIN_PROGRESS*400*CAR_PI && turned>CAR_ARC_MIN_TURN_RAD &&
            heading_error<CAR_ENDPOINT_HEADING_RAD && end_distance<CAR_ENDPOINT_GATE_MM;
        /* A qualified sensor edge is a position observation. Bound its
         * correction before using the corrected footprint: accumulated odometry
         * error must not reject a valid edge just 5 mm behind the front bumper. */
        float correction=Clamp(Motion_Wrap(observed-c->heading_rad),-0.15f,0.15f);
        float observed_heading=c->heading_rad+CAR_ENDPOINT_YAW_BLEND*correction;
        float lateral=-c->line_error;
        float observed_x=end.x-c->geometry.sensor_forward_mm*cosf(observed_heading)
                             +lateral*sinf(observed_heading);
        float observed_y=end.y-c->geometry.sensor_forward_mm*sinf(observed_heading)
                             -lateral*cosf(observed_heading);
#if CAR_TIME_TRIAL_EDGE_POSE

        if (CrossSprint(c) && c->edge_valid && c->edge_ready) {
            float adjust=CAR_ENDPOINT_YAW_BLEND*Clamp(Motion_Wrap(observed-c->edge_heading),-0.15f,0.15f);
            float edge_heading=c->edge_heading+adjust;
            float dx=c->x_mm-c->edge_x,dy=c->y_mm-c->edge_y;
            observed_x=end.x-c->geometry.sensor_forward_mm*cosf(edge_heading)+lateral*sinf(edge_heading)
                +cosf(adjust)*dx-sinf(adjust)*dy;
            observed_y=end.y-c->geometry.sensor_forward_mm*sinf(edge_heading)-lateral*cosf(edge_heading)
                +sinf(adjust)*dx+cosf(adjust)*dy;
            observed_heading=c->heading_rad+adjust;
        }
        if (CrossSprint(c) && (!c->edge_valid || !c->edge_ready)) exit=false;
#endif
        float correction_limit=(c->task>=3 || FastCourse(c)) ?
            CAR_CROSS_ENDPOINT_CORRECTION_MM : CAR_ENDPOINT_CORRECTION_MM;
        exit=exit && hypotf(observed_x-c->x_mm,observed_y-c->y_mm)<=correction_limit &&
            CoversPose(c,end,observed_x,observed_y,observed_heading);
#if CAR_TIME_TRIAL_EXIT_TRACE
        if (c->white_frames>=CAR_LINE_CONFIRM_FRAMES && progress>CAR_ARC_MIN_PROGRESS*400*CAR_PI) {
            volatile CarArcDebug *d=ArcDebugSlot(c);
            if (d) {
                float offset=hypotf(observed_x-c->x_mm,observed_y-c->y_mm);
                uint32_t rejected=(!c->have_line ? ARC_EXIT_NO_LINE : 0U) |
                    (in->now_ms-c->last_line_at>=100 ? ARC_EXIT_STALE : 0U) |
                    (fabsf(c->line_error)>1.5f*CAR_GRAY_PITCH_MM ? ARC_EXIT_LATERAL : 0U) |
                    (turned<=CAR_ARC_MIN_TURN_RAD ? ARC_EXIT_TURN : 0U) |
                    (heading_error>=CAR_ENDPOINT_HEADING_RAD ? ARC_EXIT_HEADING : 0U) |
                    (end_distance>=CAR_ENDPOINT_GATE_MM ? ARC_EXIT_DISTANCE : 0U) |
                    (offset>correction_limit ? ARC_EXIT_CORRECTION : 0U) |
                    (!CoversPose(c,end,observed_x,observed_y,observed_heading) ? ARC_EXIT_FOOTPRINT : 0U);
#if CAR_TIME_TRIAL_EDGE_POSE
                if (CrossSprint(c) && (!c->edge_valid || !c->edge_ready)) rejected|=ARC_EXIT_EDGE_POSE;
#endif
                CarArcExitSample sample={in->now_ms,rejected,c->x_mm,c->y_mm,c->heading_rad,
                    progress,turned,heading_error,offset,c->line_error,c->white_frames,
                    in->now_ms-c->last_line_at};
                if (!d->exit_attempts) d->first=sample;
                d->last=sample;++d->exit_attempts;
            }
        }
#endif
        if (exit) {
            /* Geometric yaw observation at a centered, qualified arc exit.
             * For a forward sensor on a circle, body heading lags the sensor
             * tangent by asin(sensor_forward / radius), with the bend sign.
             * Blend rather than hard-snap: binary sensing has finite uncertainty.
             * This assumes settled circular tracking; verify on the real car. */
            c->heading_rad=observed_heading;
            c->yaw_offset=c->heading_rad-in->yaw_rad;
            c->x_mm=observed_x;
            c->y_mm=observed_y;
            dest=Route(c)[c->segment+1];
#if CAR_ARC_TEST_BUILD
            /* Isolated A-B-C experiment: keep the qualified arc exit and
             * settled footprint checks, then stop at C without entering C-D. */
            if (c->task==2 && c->segment==1) { Brake(c,in->now_ms); return; }
#endif
            if (c->segment==3) {
                ++c->completed_laps;
                if (c->completed_laps>=TargetLaps(c)) { Brake(c,in->now_ms); return; }
                Signal(c,dest); c->segment=0;
            } else { Signal(c,dest); ++c->segment; }
            BeginSegment(c,in->now_ms); c->state=CAR_STRAIGHT;
            /* Task 4 keeps cruise through the segment boundary as well. */
            speed=FastCourse(c) ? FastEntrySpeed() : CAR_CAPTURE_SPEED; omega=0;
        } else {
            if ((c->line_missing && in->now_ms-c->missing_at>=CAR_LINE_LOST_TIMEOUT_MS) ||
                progress>400*CAR_PI+350 || turned>CAR_PI+1.0f) {
                Stop(c,CAR_FAULT,CAR_FAULT_LINE); return;
            }
            speed=FastCourse(c) ? FastArcSpeed(c) :
                (turned>2.55f && !FinalSegment(c)) ? CAR_CAPTURE_SPEED : CAR_ARC_SPEED;
            if (CAR_TIME_TRIAL_BUILD && FastCourse(c)) {
                /* Finish front-sensor acquisition before raising arc cruise. */
                speed=fminf(speed,sqrtf(CAR_TT_ENTRY_SPEED*CAR_TT_ENTRY_SPEED+
                    2*CAR_TT_APPROACH_ACCEL*fmaxf(0,progress-CAR_TT_ARC_ACCEL_START_MM)));
                if (c->line_missing || fabsf(c->line_error)>1.5f*CAR_GRAY_PITCH_MM)
                    speed=fminf(speed,CAR_TT_ENTRY_SPEED);
            }
            omega=ArcSign(c)*speed/ArcRadius(c);
            /* An outer-probe observation becomes uncertain during loss.
             * Retain P initially, then fade it to zero by the existing 180 ms
             * loss deadline. Never extrapolate D or confirm an unseen point. */
            bool entry_edge=(CAR_TIME_TRIAL_STAGE3 || (CAR_TIME_TRIAL_ENTRY_HOLD && FastCourse(c))) &&
                progress<c->geometry.sensor_forward_mm &&
                fabsf(c->line_error)>=2.5f*CAR_GRAY_PITCH_MM;
            uint32_t edge_age=in->now_ms-c->last_line_at;
            uint32_t edge_hold_ms=entry_edge ? CAR_LINE_LOST_TIMEOUT_MS : 80U;
            bool recent_edge=FastCourse(c) && c->have_line && edge_age<=edge_hold_ms;
            if (!c->line_missing || recent_edge) {
                float correction=ArcFeedback(c,speed,!c->line_missing);
                /* T317 retains only the last outer-probe P correction during
                 * entry. Stale D remains disabled, the existing loss timeout
                 * still stops at 180 ms, and white cannot confirm a point. */
                if (c->line_missing && entry_edge && edge_age>80U && !CAR_TIME_TRIAL_ENTRY_HOLD)
                    correction*=Clamp((float)(CAR_LINE_LOST_TIMEOUT_MS-edge_age)/
                                      (CAR_LINE_LOST_TIMEOUT_MS-80U),0,1);
                omega-=correction;
            }
            if (progress<c->geometry.sensor_forward_mm)
                omega=EntryOmega(c,speed,omega);
        }
    }
    Drive(c,speed,omega,dt,in);
    bool left_bad=StallWheel(&c->left_stall,&c->left_stall_at,c->motion.left.setpoint,in->left_speed,in->now_ms);
    bool right_bad=StallWheel(&c->right_stall,&c->right_stall_at,c->motion.right.setpoint,in->right_speed,in->now_ms);
    if (left_bad || right_bad) Stop(c,CAR_FAULT,CAR_FAULT_STALL);
}
