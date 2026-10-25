#include "task1_test.h"
#include "car_config.h"
#include <math.h>

static bool ButtonRequest(Task1Test *t, uint32_t now, bool pressed, bool enabled)
{
    if (!enabled) { t->button_armed=false; t->button_candidate=false; }
    if (!t->button_initialized) {
        t->button_initialized=true;
        t->button_raw=t->button_stable=pressed;
        t->button_changed_at=now;
        return false;
    }
    if (pressed!=t->button_raw) {
        t->button_raw=pressed; t->button_changed_at=now;
    }
    if (now-t->button_changed_at<25U) return false;
    if (t->button_stable!=pressed) {
        t->button_stable=pressed;
        if (pressed) {
            t->button_candidate=t->button_armed && enabled;
            t->button_armed=false; t->button_pressed_at=now;
        } else {
            bool start=t->button_candidate && enabled && now-t->button_pressed_at<800U;
            t->button_candidate=false; t->button_armed=enabled;
            return start;
        }
    }
    if (!pressed && enabled) t->button_armed=true;
    return false;
}

void Task1Test_Apply(Task1Test *t, CarInputs *in, CarState state, uint32_t request)
{
    bool enabled=in->enabled, pressed=in->pressed;
    uint32_t now=in->now_ms;
    in->enabled=false; in->pressed=false; t->recalibrate=false;
    if (t->phase==TASK1_IDLE) {
        bool button_request=ButtonRequest(t,now,pressed,enabled && state==CAR_STOPPED);
        if (!request && button_request) request=1;
        if (!request) return;
        if (request!=1 || !enabled || pressed || state!=CAR_STOPPED) {
            t->phase=TASK1_ABORTED; return;
        }
        t->phase=TASK1_CALIBRATING; t->phase_at=now;
        t->recalibrate=true; return;
    }
    if (t->phase==TASK1_FINISHED || t->phase==TASK1_ABORTED) return;
    if (!enabled || pressed) { t->phase=TASK1_ABORTED; return; }
    if (t->phase==TASK1_CALIBRATING) {
        if (now-t->phase_at>=8000) { t->phase=TASK1_ABORTED; return; }
        if (!in->gray_online || !in->imu_ready || !in->encoder_valid ||
            !isfinite(in->left_speed) || !isfinite(in->right_speed) ||
            fabsf(in->left_speed)>5 || fabsf(in->right_speed)>5) return;
        t->phase=TASK1_PRESS; t->phase_at=now;
    }
    if (t->phase==TASK1_PRESS) {
        if (now-t->phase_at<80) { in->enabled=true; in->pressed=true; return; }
        t->phase=TASK1_RUNNING; t->phase_at=now;
    }
    if (t->phase==TASK1_RUNNING) {
        if (state>=CAR_STRAIGHT && state<=CAR_BRAKING) t->seen_running=true;
        if (state==CAR_DONE || state==CAR_FAULT || (t->seen_running && state==CAR_STOPPED)) {
            t->phase=TASK1_FINISHED; return;
        }
        if (now-t->phase_at>=CAR_TEST_RUN_LIMIT_MS || (!t->seen_running && now-t->phase_at>500)) {
            t->phase=TASK1_ABORTED; return;
        }
        in->enabled=true;
    }
}
