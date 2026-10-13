#include "app.h"
#include "board_c10b.h"
#include "hiwonder_gray.h"
#include "mpu6050.h"

volatile AppDiagnostics app_diagnostics;
static CarControl car;

void App_Run(void)
{
    uint8_t identity = 0U;
    Board_Init();
    HiwonderGray_Init(HAL_GetTick());
    app_diagnostics.mpu_online = MPU6050_CheckIdentity(&identity);
    app_diagnostics.mpu_identity = identity;
    CarControl_Init(&car, HAL_GetTick(), Board_ButtonPressed());
    while (1)
    {
        uint32_t now = HAL_GetTick();
        GrayProtocol sample;
        bool enabled = Board_MotionEnabled();
        HiwonderGray_Poll(now);
        HiwonderGray_Snapshot(&sample);
        bool online = GrayProtocol_Online(&sample, now);
        uint8_t mask = GrayProtocol_BlackMask(sample.raw_status);
        CarControl_Update(&car, now, mask, Board_ButtonPressed(), online, enabled);
        Board_MotorApply(car.left_pwm, car.right_pwm);
        Board_SetLeds(car.state == CAR_RUNNING,
                       !online || car.state == CAR_FAULT_SENSOR ||
                       car.state == CAR_FAULT_LINE_LOST || car.state == CAR_START_REQUIRED);
        app_diagnostics.state = car.state;
        app_diagnostics.enabled = enabled;
        app_diagnostics.gray_online = online;
        app_diagnostics.gray_raw_status = sample.raw_status;
        app_diagnostics.black_mask = mask;
        app_diagnostics.completed_laps = car.completed_laps;
        app_diagnostics.left_pwm = car.left_pwm;
        app_diagnostics.right_pwm = car.right_pwm;
        app_diagnostics.gray_samples = sample.samples;
        app_diagnostics.gray_timeouts = sample.timeouts;
        app_diagnostics.gray_errors = sample.errors;
        app_diagnostics.gray_unexpected_bytes = sample.unexpected_bytes;
        HAL_Delay(1U);
    }
}
