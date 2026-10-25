#ifndef GROUND_TEST_H
#define GROUND_TEST_H
#include "motor_test.h"
#include "motion_control.h"
enum { GROUND_OK, GROUND_BAD_FEEDBACK, GROUND_MOVING_START,
       GROUND_REVERSE, GROUND_DIVERGENCE, GROUND_OVERSPEED, GROUND_STALL };
typedef struct {
    bool started;
    float origin[2], progress[2];
    uint32_t progressed_at[2], fault;
    MotionControl motion;
    uint16_t pwm[2];
} GroundTest;
void GroundTest_Feedback(GroundTest *g, MotorTest *t, uint32_t now,
                        bool valid, float left_mm, float right_mm,
                        float left_speed, float right_speed, float dt);
#endif
