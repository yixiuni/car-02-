#include "car_control.h"
#include "car_config.h"
#include <math.h>

/* Map: A=(0,0), B=(1000,0), C=(1000,-800), D=(0,-800).
 * Heading positive CCW. Start axle at A: face B for tasks 1/2, C for 3/4. */
typedef struct { float x, y; } Point;
static const Point points[] = {{0,0},{1000,0},{1000,-800},{0,-800}};
static const CarPoint oval[] = {CAR_POINT_A,CAR_POINT_B,CAR_POINT_C,CAR_POINT_D,CAR_POINT_A};
static const CarPoint cross[] = {CAR_POINT_A,CAR_POINT_C,CAR_POINT_B,CAR_POINT_D,CAR_POINT_A};
static const CarPoint *Route(const CarControl *c) { return c->task <= 2 ? oval : cross; }
static float Clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo,x)); }
bool CarControl_Active(const CarControl *c)
{
    return c->state >= CAR_STRAIGHT && c->state <= CAR_BRAKING;
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
static bool Covers(const CarControl *c, Point p)
{
    float dx=p.x-c->x_mm, dy=p.y-c->y_mm;
    float forward=cosf(c->heading_rad)*dx+sinf(c->heading_rad)*dy;
    float lateral=-sinf(c->heading_rad)*dx+cosf(c->heading_rad)*dy;
    return forward <= c->geometry.front_mm && forward >= -c->geometry.rear_mm &&
           fabsf(lateral) <= c->geometry.half_width_mm;
}
static CarFault InputFault(const CarControl *c, const CarInputs *in)
{
    if (!CarGeometry_Valid(&c->geometry)) return CAR_FAULT_CONFIG;
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
    c->heading_rad=c->task<=2 ? 0.0f : atan2f(-800.0f,1000.0f);
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
        c->line_rate=c->have_line && elapsed>0 && elapsed<100 ?
            (error-c->line_error)*1000.0f/(float)elapsed : 0;
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
    Motion_Update(&c->motion,c->left_target,c->right_target,in->left_speed,in->right_speed,
                  dt,&c->left_pwm,&c->right_pwm);
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
    uint32_t limit=c->task==1 ? 15000U : c->task==2 ? 30000U : c->task==3 ? 40000U : 160000U;
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
            Signal(c,dest); ++c->segment; BeginSegment(c,in->now_ms);
            c->state=CAR_CAPTURE;
        } else {
            if (remaining < -80 || fabsf(lateral)>180) {
                Stop(c,CAR_FAULT,CAR_FAULT_ENDPOINT); return;
            }
            float desired=atan2f(dy,dx)-atan2f(lateral,250.0f);
            omega=CAR_HEADING_KP*Motion_Wrap(desired-c->heading_rad);
            speed=c->task<=2 ? CAR_STRAIGHT_SPEED : CAR_DIAGONAL_SPEED;
            if (remaining<250) speed=fminf(speed,fmaxf(CAR_CAPTURE_SPEED,
                sqrtf(2*CAR_ACCEL_MM_S2*fmaxf(0,remaining-c->geometry.sensor_forward_mm))));
        }
    }
    if (c->state==CAR_CAPTURE) {
        float error=Motion_Wrap(ArcStart(c)-c->heading_rad);
        speed=CAR_CAPTURE_SPEED;
        omega=CAR_HEADING_KP*error;
        if (c->black_frames>=CAR_LINE_CONFIRM_FRAMES && fabsf(error)<0.20f) {
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
        float heading_error=fabsf(Motion_Wrap(c->heading_rad-ArcEnd(c)));
        bool exit=c->have_line && c->white_frames>=CAR_LINE_CONFIRM_FRAMES &&
            in->now_ms-c->last_line_at<100 && fabsf(c->line_error)<=1.5f*CAR_GRAY_PITCH_MM &&
            progress>CAR_ARC_MIN_PROGRESS*400*CAR_PI && turned>CAR_ARC_MIN_TURN_RAD &&
            heading_error<CAR_ENDPOINT_HEADING_RAD && end_distance<CAR_ENDPOINT_GATE_MM && Covers(c,end);
        if (exit) {
            /* Geometric yaw observation at a centered, qualified arc exit.
             * For a forward sensor on a circle, body heading lags the sensor
             * tangent by asin(sensor_forward / radius), with the bend sign.
             * Blend rather than hard-snap: binary sensing has finite uncertainty.
             * This assumes settled circular tracking; verify on the real car. */
            float observed=ArcEnd(c)-ArcSign(c)*asinf(c->geometry.sensor_forward_mm/400.0f);
            float correction=Clamp(Motion_Wrap(observed-c->heading_rad),-0.15f,0.15f);
            c->heading_rad+=CAR_ENDPOINT_YAW_BLEND*correction;
            c->yaw_offset=c->heading_rad-in->yaw_rad;
            float lateral=-c->line_error; /* positive to the vehicle's left */
            c->x_mm=end.x-c->geometry.sensor_forward_mm*cosf(c->heading_rad)
                        +lateral*sinf(c->heading_rad);
            c->y_mm=end.y-c->geometry.sensor_forward_mm*sinf(c->heading_rad)
                        -lateral*cosf(c->heading_rad);
            dest=Route(c)[c->segment+1];
            if (c->segment==3) {
                ++c->completed_laps;
                if (c->task!=4 || c->completed_laps>=4) { Brake(c,in->now_ms); return; }
                Signal(c,dest); c->segment=0;
            } else { Signal(c,dest); ++c->segment; }
            BeginSegment(c,in->now_ms); c->state=CAR_STRAIGHT;
            /* Retain low forward speed for this single boundary tick. */
            speed=CAR_CAPTURE_SPEED; omega=0;
        } else {
            if ((c->line_missing && in->now_ms-c->missing_at>=CAR_LINE_LOST_TIMEOUT_MS) ||
                progress>400*CAR_PI+350 || turned>CAR_PI+0.7f) {
                Stop(c,CAR_FAULT,CAR_FAULT_LINE); return;
            }
            speed=(turned>2.55f) ? CAR_CAPTURE_SPEED : CAR_ARC_SPEED;
            omega=ArcSign(c)*speed/400.0f;
            if (!c->line_missing) omega-=CAR_LINE_KP*c->line_error+CAR_LINE_KD*c->line_rate;
        }
    }
    Drive(c,speed,omega,dt,in);
    bool left_bad=StallWheel(&c->left_stall,&c->left_stall_at,c->motion.left.setpoint,in->left_speed,in->now_ms);
    bool right_bad=StallWheel(&c->right_stall,&c->right_stall_at,c->motion.right.setpoint,in->right_speed,in->now_ms);
    if (left_bad || right_bad) Stop(c,CAR_FAULT,CAR_FAULT_STALL);
}
