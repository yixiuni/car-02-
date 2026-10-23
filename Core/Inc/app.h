#ifndef APP_H
#define APP_H
#include "car_control.h"
typedef struct {
    CarState state;
    CarFault fault;
    bool enabled, calibration_valid, gray_online, mpu_online, imu_ready, encoder_valid;
    uint8_t task, segment, mpu_identity, gray_raw_status, black_mask, completed_laps;
    uint16_t left_pwm, right_pwm;
    uint32_t gray_samples, gray_timeouts, gray_errors, gray_unexpected_bytes;
    uint32_t point_events, gyro_calibration_samples, control_interval_ms;
    CarPoint last_point;
    int64_t encoder_a_counts, encoder_b_counts;
    float line_error_mm, gyro_z_dps, gyro_bias_dps, yaw_rad, x_mm, y_mm, heading_rad;
    float left_mm, right_mm, left_speed, right_speed, left_target, right_target;
} AppDiagnostics;
extern volatile AppDiagnostics app_diagnostics;
void App_Run(void);
#endif
