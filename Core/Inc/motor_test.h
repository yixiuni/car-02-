#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H
#include <stdbool.h>
#include <stdint.h>

#define MOTOR_TEST_DUTY 350U
#define MOTOR_TEST_DURATION_MS 300U
enum { MOTOR_TEST_IDLE, MOTOR_TEST_RUNNING, MOTOR_TEST_DONE, MOTOR_TEST_ABORTED };
typedef struct {
    uint32_t state, wheel, started_at, last_at;
    uint32_t duty, duration_ms;
    uint32_t target_mm_s;
    uint32_t distance_mm;
    uint16_t left_pwm, right_pwm;
} MotorTest;
/* One request per reset. 1/2: left/right 35%, 300 ms.
 * 3/4: 15%, 1000 ms; 5/6: 25%, 1000 ms; 7/8: 35%, 1000 ms.
 * 9/10: closed-loop 150 mm/s; 11/12: 180 mm/s; 13/14: 85 mm/s;
 * 15/16: 250 mm/s,
 * each 2000 ms with a 35% ceiling. State machine leaves PWM at zero for
 * closed-loop commands; the port uses Motion_Update with encoder feedback.
 * Fixed profiles only: external writes cannot request arbitrary power/time. */
void MotorTest_Update(MotorTest *test, uint32_t now, uint32_t request,
                      bool enabled, bool button_pressed);
void MotorTest_Run(void);
/* Ground-only image accepts fixed requests: 17 = 200 mm / 4 s,
 * 18 = 500 mm / 8 s, 19 = 1000 mm / 15 s. Both wheels, 85 mm/s nominal target.
 * GroundTest_Feedback additionally enforces position/feedback stop guards.
 * Bench images reject 17/18/19; ground images reject every bench request. */
uint32_t MotorTest_TracePeriodMs(const MotorTest *test);
#endif
