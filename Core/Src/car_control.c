#include "car_control.h"
#include "car_config.h"
#include <math.h>

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
          CAR_TT_WHEEL_SPEED<=1600 && CAR_TT_D_FILTER_MS>=0 && CAR_TT_D_FILTER_MS<=100))
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
    c->have_line=false; c->line_missing=false;
    c->black_frames=c->white_frames=0;
    c->line_error=c->line_rate=0;
}
static void Start(CarControl *c, const CarInputs *in)
{
    CarFault f=InputFault(c,in);
    if (f!=CAR_OK) { Stop(c,CAR_FAULT,f); return; }
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
        if (c->black_frames<255) ++c->black_frames;
    } else {
        c->black_frames=0;
        /* Wide black is invalid, not an arc-end white edge. */
        if (in->black_mask==0) { if (c->white_frames<255) ++c->white_frames; }
        else c->white_frames=0;
        if (!c->line_missing) { c->line_missing=true; c->missing_at=in->now_ms; }
    }
}
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
static float FastArcSpeed(void)
{
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
    return gain*CAR_LINE_KP*c->line_error+(derivative ? d_gain*CAR_LINE_KD*c->line_rate : 0);
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
    Gray(c,in);
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
        bool reached=remaining<CAR_ENDPOINT_GATE_MM && remaining>-80 &&
            end_distance<CAR_ENDPOINT_GATE_MM && Covers(c,end) &&
            c->black_frames>=CAR_LINE_CONFIRM_FRAMES;
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
            float desired=atan2f(dy,dx)-atan2f(lateral,250.0f);
            if(c->task==4 && (c->segment==2 ||
                (c->segment==0 && CAR_CROSS_C_ENTRY_AIM_MM>0))) {
                /* Aim along the incoming black arc, not only its exposed tip.
                 * All point gates remain referenced to the actual C/D endpoint:
                 * fresh black and physical footprint coverage are still required. */
                float advance=Clamp(c->segment==2 ? CAR_CROSS_D_ENTRY_AIM_MM :
                                    CAR_CROSS_C_ENTRY_AIM_MM,0.0f,60.0f);
                float ax=end.x+(c->segment==2 ? -advance : advance)-begin.x;
                float ay=end.y+400.0f-sqrtf(400.0f*400.0f-advance*advance)-begin.y;
                float aim_length=hypotf(ax,ay);
                float aim_lateral=-(ay/aim_length)*(c->x_mm-begin.x)+
                                   (ax/aim_length)*(c->y_mm-begin.y);
                desired=atan2f(ay,ax)-atan2f(aim_lateral,250.0f);
            }
            omega=CAR_HEADING_KP*Motion_Wrap(desired-c->heading_rad);
            speed=FastCourse(c) ? CAR_FOUR_CRUISE_SPEED : c->task<=2 ? CAR_STRAIGHT_SPEED : CAR_DIAGONAL_SPEED;
            if (CAR_TIME_TRIAL_BUILD && FastCourse(c)) {
                float heading_error=Motion_Wrap(desired-c->heading_rad);
                /* Accelerate only after aligning. The distance envelope reaches
                 * arc speed before the front sensor can see the endpoint. */
                float aligned=Clamp((0.55f-fabsf(heading_error))/0.35f,0,1);
                float requested=CAR_TT_ENTRY_SPEED+
                    aligned*(CAR_TT_STRAIGHT_SPEED-CAR_TT_ENTRY_SPEED);
                float available=fmaxf(0,remaining-c->geometry.sensor_forward_mm-60.0f);
                speed=fminf(requested,sqrtf(CAR_TT_ENTRY_SPEED*CAR_TT_ENTRY_SPEED+
                                            2*CAR_TT_APPROACH_ACCEL*available));
                /* Speed-scaled heading feedback preserves geometric turning
                 * response when accelerating, unlike a fixed angular gain. */
                omega=CAR_HEADING_KP*(speed/CAR_FOUR_CRUISE_SPEED)*heading_error;
            }
            if(CAR_DUAL_MODE_BUILD && FastCourse(c) && c->black_frames>0 &&
               remaining<CAR_ENDPOINT_GATE_MM && remaining>-80 &&
               end_distance<CAR_ENDPOINT_GATE_MM &&
               (Covers(c,end) || (CAR_CROSS_C_ENTRY_PREVIEW && c->task==4 && c->segment==0))) {
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
        float heading_error=fabsf(Motion_Wrap(c->heading_rad-observed));
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
        float correction_limit=(c->task>=3 || FastCourse(c)) ?
            CAR_CROSS_ENDPOINT_CORRECTION_MM : CAR_ENDPOINT_CORRECTION_MM;
        exit=exit && hypotf(observed_x-c->x_mm,observed_y-c->y_mm)<=correction_limit &&
            CoversPose(c,end,observed_x,observed_y,observed_heading);
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
                progress>400*CAR_PI+350 || turned>CAR_PI+0.7f) {
                Stop(c,CAR_FAULT,CAR_FAULT_LINE); return;
            }
            speed=FastCourse(c) ? FastArcSpeed() :
                (turned>2.55f && !FinalSegment(c)) ? CAR_CAPTURE_SPEED : CAR_ARC_SPEED;
            if (CAR_TIME_TRIAL_BUILD && FastCourse(c)) {
                speed=fminf(speed,sqrtf(CAR_TT_ENTRY_SPEED*CAR_TT_ENTRY_SPEED+
                    2*CAR_TT_APPROACH_ACCEL*fmaxf(0,progress)));
                if (c->line_missing || fabsf(c->line_error)>1.5f*CAR_GRAY_PITCH_MM)
                    speed=fminf(speed,CAR_TT_ENTRY_SPEED);
            }
            omega=ArcSign(c)*speed/ArcRadius(c);
            /* An outer-probe observation becomes uncertain during loss.
             * Retain P initially, then fade it to zero by the existing 180 ms
             * loss deadline. Never extrapolate D or confirm an unseen point. */
            bool entry_edge=CAR_TIME_TRIAL_STAGE3 &&
                progress<c->geometry.sensor_forward_mm &&
                fabsf(c->line_error)>=2.5f*CAR_GRAY_PITCH_MM;
            uint32_t edge_age=in->now_ms-c->last_line_at;
            uint32_t edge_hold_ms=entry_edge ? CAR_LINE_LOST_TIMEOUT_MS : 80U;
            bool recent_edge=FastCourse(c) && c->have_line && edge_age<=edge_hold_ms;
            if (!c->line_missing || recent_edge) {
                float correction=ArcFeedback(c,speed,!c->line_missing);
                if (c->line_missing && entry_edge && edge_age>80U)
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
