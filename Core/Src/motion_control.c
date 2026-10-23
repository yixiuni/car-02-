#include "motion_control.h"
#include "car_config.h"
#include <math.h>

CarGeometry CarGeometry_Default(void)
{
    return (CarGeometry){CAR_WHEEL_DIAMETER_MM, CAR_WHEEL_TRACK_MM,
        CAR_ENCODER_A_COUNTS_REV, CAR_ENCODER_B_COUNTS_REV,
        CAR_FRONT_MM, CAR_REAR_MM, CAR_HALF_WIDTH_MM, CAR_SENSOR_FORWARD_MM,
        CAR_CALIBRATION_CONFIRMED != 0};
}
bool CarGeometry_Valid(const CarGeometry *g)
{
    return g->confirmed && isfinite(g->diameter_mm) && g->diameter_mm > 10 &&
        g->diameter_mm < 200 && isfinite(g->track_mm) && g->track_mm > 30 &&
        g->track_mm < 300 && isfinite(g->counts_a_rev) && g->counts_a_rev >= 4 &&
        isfinite(g->counts_b_rev) && g->counts_b_rev >= 4 &&
        g->counts_a_rev < 1000000 && g->counts_b_rev < 1000000 &&
        isfinite(g->front_mm) && g->front_mm > 0 &&
        isfinite(g->rear_mm) && g->rear_mm > 0 &&
        g->front_mm + g->rear_mm <= 250 &&
        isfinite(g->half_width_mm) && g->half_width_mm > 0 && g->half_width_mm <= 75 &&
        isfinite(g->sensor_forward_mm) && g->sensor_forward_mm >= 0 &&
        g->sensor_forward_mm <= g->front_mm;
}
float Motion_Wrap(float angle)
{
    return remainderf(angle, 2.0f * CAR_PI);
}
void Motion_Reset(MotionControl *m) { *m = (MotionControl){0}; }
int32_t Encoder_CounterDelta(uint16_t current, uint16_t previous)
{
    uint32_t delta = (uint16_t)(current - previous);
    return delta <= 32767U ? (int32_t)delta : (int32_t)delta - 65536;
}
static float Clamp(float x, float low, float high)
{
    return fminf(high, fmaxf(low, x));
}
static uint16_t WheelUpdate(WheelController *w, float target, float measured, float dt)
{
    if (target <= 0.0f) {
        *w = (WheelController){0};
        return 0U; /* coast; never request reverse torque */
    }
    w->setpoint += Clamp(target - w->setpoint, -CAR_ACCEL_MM_S2 * dt, CAR_ACCEL_MM_S2 * dt);
    float error = w->setpoint - measured;
    float proposed = w->integral + CAR_SPEED_KI * error * dt;
    float base = CAR_SPEED_FF * w->setpoint + CAR_SPEED_KP * error;
    /* Conditional integration prevents windup at both actuator limits. */
    if ((base + proposed >= 0 && base + proposed <= CAR_PWM_LIMIT) ||
        (base + proposed > CAR_PWM_LIMIT && error < 0) ||
        (base + proposed < 0 && error > 0)) w->integral = proposed;
    return (uint16_t)Clamp(base + w->integral, 0, CAR_PWM_LIMIT);
}
void Motion_Update(MotionControl *m, float left_target, float right_target,
                   float left_measured, float right_measured, float dt,
                   uint16_t *left_pwm, uint16_t *right_pwm)
{
    if (!isfinite(left_target) || !isfinite(right_target) ||
        !isfinite(left_measured) || !isfinite(right_measured) ||
        !isfinite(dt) || dt <= 0 || dt > 0.05f) {
        Motion_Reset(m); *left_pwm = *right_pwm = 0; return;
    }
    *left_pwm = WheelUpdate(&m->left, left_target, left_measured, dt);
    *right_pwm = WheelUpdate(&m->right, right_target, right_measured, dt);
}
