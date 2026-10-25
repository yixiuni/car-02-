#ifndef TASK1_TEST_H
#define TASK1_TEST_H
#include "car_control.h"
enum { TASK1_IDLE, TASK1_CALIBRATING, TASK1_PRESS, TASK1_RUNNING,
       TASK1_FINISHED, TASK1_ABORTED };
typedef struct {
    uint32_t phase, phase_at;
    bool seen_running, recalibrate;
    uint32_t button_changed_at, button_pressed_at;
    bool button_initialized, button_raw, button_stable, button_armed, button_candidate;
} Task1Test;
/* One debounced short press/release or SWD request (1) per boot. After
 * starting calibration, any physical press aborts. The actual
 * course controller still performs all sensor, route and endpoint checks. */
void Task1Test_Apply(Task1Test *t, CarInputs *in, CarState state, uint32_t request);
#endif
