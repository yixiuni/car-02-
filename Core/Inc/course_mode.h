#ifndef COURSE_MODE_H
#define COURSE_MODE_H
#include "task1_test.h"

typedef struct {
    uint32_t changed_at, pressed_at, click_at;
    uint8_t oval_laps, cross_laps, feedback_pulses;
    bool initialized, raw, stable, armed, candidate, click_pending;
} CourseMode;
void CourseMode_Init(CourseMode *mode, CarControl *car);
/* Parked: short click starts after the double-click window; double click
 * cycles 1..4 laps; hold 800 ms toggles route once. Moving: press aborts. */
void CourseMode_Apply(CourseMode *mode, Task1Test *gate, CarInputs *input, CarControl *car);
#endif
