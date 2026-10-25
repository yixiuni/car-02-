#include "motor_test.h"
#include "car_config.h"

void MotorTest_Update(MotorTest *t, uint32_t now, uint32_t request,
                      bool enabled, bool button_pressed)
{
    t->left_pwm=t->right_pwm=0;
    if (t->state==MOTOR_TEST_IDLE && request) {
        if (
#if CAR_GROUND_TEST_BUILD
            (request!=17 && request!=18 && request!=19) ||
#else
            request>16 ||
#endif
            !enabled || button_pressed) {
            t->state=MOTOR_TEST_ABORTED;
            return;
        }
        t->wheel=(request&1U) ? 1U : 2U;
        t->duty=request<=2 ? MOTOR_TEST_DUTY : request<=4 ? 150U : request<=6 ? 250U : 350U;
        t->duration_ms=request<=2 ? MOTOR_TEST_DURATION_MS : 1000U;
        if (request>=9) {
            t->target_mm_s=request<=10 ? 150U : request<=12 ? 180U : request<=14 ? 85U : 250U;
            t->duration_ms=2000U;
        }
#if CAR_GROUND_TEST_BUILD
        t->wheel=3U; t->target_mm_s=85U;
        t->distance_mm=request==17 ? 200U : request==18 ? 500U : 1000U;
        t->duration_ms=request==17 ? 4000U : request==18 ? 8000U : 15000U;
#endif
        t->started_at=t->last_at=now;
        t->state=MOTOR_TEST_RUNNING;
    }
    if (t->state!=MOTOR_TEST_RUNNING) return;
    if (!enabled || button_pressed || now-t->last_at>CAR_CONTROL_MAX_GAP_MS) {
        t->state=MOTOR_TEST_ABORTED;
        return;
    }
    t->last_at=now;
    if (now-t->started_at>=t->duration_ms) {
        t->state=CAR_GROUND_TEST_BUILD ? MOTOR_TEST_ABORTED : MOTOR_TEST_DONE;
        return;
    }
    if (!t->target_mm_s) {
        if (t->wheel==1) t->left_pwm=(uint16_t)t->duty;
        else t->right_pwm=(uint16_t)t->duty;
    }
}

uint32_t MotorTest_TracePeriodMs(const MotorTest *t)
{
#if CAR_GROUND_TEST_BUILD
    /* Include the entire fixed profile plus 300 ms coast in 256 entries. */
    return t->duration_ms>8000U ? 80U : t->duration_ms>4000U ? 40U : 20U;
#else
    (void)t;
    return 10U;
#endif
}
