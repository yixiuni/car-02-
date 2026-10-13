#ifndef APP_H
#define APP_H
#include "car_control.h"
typedef struct
{
    CarState state;
    bool enabled, gray_online, mpu_online;
    uint8_t mpu_identity, gray_raw_status, black_mask, completed_laps;
    uint16_t left_pwm, right_pwm;
    uint32_t gray_samples, gray_timeouts, gray_errors, gray_unexpected_bytes;
} AppDiagnostics;
extern volatile AppDiagnostics app_diagnostics;
void App_Run(void);
#endif
