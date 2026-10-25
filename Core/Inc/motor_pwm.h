#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H
#include <stdbool.h>
#include <stdint.h>

typedef struct { uint32_t in1, in2; } MotorPwm;
/* AT8236 1/1 shorts both motor terminals low; never requests reverse drive.
 * Caller owns the time limit and motion-enable/button inhibits. */
static inline MotorPwm MotorPwm_Brake(uint32_t period, bool allowed)
{
    return allowed ? (MotorPwm){period,period} : (MotorPwm){0,0};
}
/* PWM mode 1: CCR=period is constant high. In slow decay the other
 * input is high during BRAKE, hence period-drive, not drive. Zero command
 * must always produce 0/0 (coast), never a permanent holding brake. */
static inline MotorPwm MotorPwm_Encode(uint32_t drive, uint32_t period,
                                      bool forward_in1, bool slow_decay)
{
    if (!drive || !period) return (MotorPwm){0,0};
    if (drive>period) drive=period;
    uint32_t active=slow_decay ? period : drive;
    uint32_t other=slow_decay ? period-drive : 0;
    return forward_in1 ? (MotorPwm){active,other} : (MotorPwm){other,active};
}
#endif
