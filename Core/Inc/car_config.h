#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* Physical calibration. 65 mm wheels supplied by the user. The other values
 * MUST be measured on this car. Counts mean quadrature x4 at the WHEEL shaft,
 * including the gearbox, not encoder pulses at the motor shaft. */
#ifndef CAR_CALIBRATION_CONFIRMED
#define CAR_CALIBRATION_CONFIRMED    0
#endif
#define CAR_WHEEL_DIAMETER_MM       65.0f
#define CAR_WHEEL_TRACK_MM           0.0f
#define CAR_ENCODER_A_COUNTS_REV     0.0f
#define CAR_ENCODER_B_COUNTS_REV     0.0f
#define CAR_ENCODER_A_SIGN           1
#define CAR_ENCODER_B_SIGN           1
/* Axle midpoint is the navigation origin. Measure front/rear/half-width of
 * the complete car footprint and the sensor bar's forward offset from it. */
#define CAR_FRONT_MM                 0.0f
#define CAR_REAR_MM                  0.0f
#define CAR_HALF_WIDTH_MM            0.0f
#define CAR_SENSOR_FORWARD_MM        0.0f
#define CAR_GRAY_PITCH_MM           12.0f /* initial tuning value; measure */
#define CAR_GYRO_Z_SIGN              1    /* positive = counterclockwise */
#define CAR_DEFAULT_TASK            1U

#define CAR_PWM_MAX              1000U
#define CAR_PWM_LIMIT             700U
#define CAR_PWM_FREQUENCY_HZ     20000U
#define CAR_CONTROL_PERIOD_MS       5U
#define CAR_CONTROL_MAX_GAP_MS     50U
#define CAR_BUTTON_DEBOUNCE_MS     25U
#define CAR_BUTTON_HOLD_MS        800U
#define CAR_SENSOR_COUNT            8U
#define CAR_LINE_LOST_TIMEOUT_MS  180U
#define CAR_CAPTURE_TIMEOUT_MS  1800U
#define CAR_STALL_TIMEOUT_MS     700U
#define CAR_SIGNAL_MS            120U
#define CAR_GYRO_CALIBRATION_SAMPLES 400U
#define CAR_GYRO_CALIBRATION_STD_DPS 0.8f

/* Initial gains/limits, all require track tuning. Units: mm, s, rad, permille. */
#define CAR_STRAIGHT_SPEED        250.0f
#define CAR_DIAGONAL_SPEED        220.0f
#define CAR_ARC_SPEED             180.0f
#define CAR_CAPTURE_SPEED          85.0f
#define CAR_ACCEL_MM_S2           350.0f
#define CAR_SPEED_KP                0.8f
#define CAR_SPEED_KI                2.0f
#define CAR_SPEED_FF                1.5f
#define CAR_HEADING_KP              3.0f
#define CAR_LINE_KP                 0.025f
#define CAR_LINE_KD                 0.001f
#define CAR_ENDPOINT_GATE_MM      220.0f
#define CAR_LINE_CONFIRM_FRAMES      3U
#define CAR_ARC_MIN_PROGRESS        0.65f
#define CAR_ARC_MIN_TURN_RAD        2.45f
#define CAR_ENDPOINT_HEADING_RAD    0.44f
#define CAR_ENDPOINT_YAW_BLEND      0.5f

#ifndef CAR_GRAY_TARGET_BIT_IS_ONE
#define CAR_GRAY_TARGET_BIT_IS_ONE  1
#endif
#ifndef CAR_GRAY_REVERSE_ORDER
#define CAR_GRAY_REVERSE_ORDER      0
#endif
#define CAR_GRAY_BAUD               115200U
#define CAR_GRAY_BOOT_DELAY_MS      1000U
#define CAR_GRAY_MODE_DELAY_MS      50U
#define CAR_GRAY_QUERY_PERIOD_MS    10U
#define CAR_GRAY_RESPONSE_TIMEOUT_MS 20U
#define CAR_GRAY_SETTLE_MS          2U
#define CAR_GRAY_STALE_TIMEOUT_MS   100U

#define CAR_LEFT_IS_MOTOR_A          1
#define CAR_MOTOR_A_FORWARD_IN1      1
#define CAR_MOTOR_B_FORWARD_IN1      1
#define CAR_LEFT_SCALE_PERCENT    100U
#define CAR_RIGHT_SCALE_PERCENT   100U
/* Confirm actual buzzer polarity before enabling sound. PA15 is free after
 * disabling JTAG while retaining SWD in Board_Init. */
#define CAR_BUZZER_ACTIVE_HIGH       1

#if CAR_DEFAULT_TASK < 1 || CAR_DEFAULT_TASK > 4
#error "Task must be 1..4"
#endif
#if CAR_CONTROL_PERIOD_MS == 0 || CAR_CONTROL_MAX_GAP_MS <= CAR_CONTROL_PERIOD_MS
#error "Invalid control timing"
#endif
#if (CAR_ENCODER_A_SIGN != 1 && CAR_ENCODER_A_SIGN != -1) || \
    (CAR_ENCODER_B_SIGN != 1 && CAR_ENCODER_B_SIGN != -1) || \
    (CAR_GYRO_Z_SIGN != 1 && CAR_GYRO_Z_SIGN != -1)
#error "Sensor signs must be +1 or -1"
#endif
#if CAR_PWM_FREQUENCY_HZ == 0 || CAR_PWM_LIMIT > CAR_PWM_MAX
#error "Invalid PWM configuration"
#endif
#if (CAR_GRAY_TARGET_BIT_IS_ONE != 0 && CAR_GRAY_TARGET_BIT_IS_ONE != 1) || \
    (CAR_GRAY_REVERSE_ORDER != 0 && CAR_GRAY_REVERSE_ORDER != 1) || \
    (CAR_LEFT_IS_MOTOR_A != 0 && CAR_LEFT_IS_MOTOR_A != 1) || \
    (CAR_MOTOR_A_FORWARD_IN1 != 0 && CAR_MOTOR_A_FORWARD_IN1 != 1) || \
    (CAR_MOTOR_B_FORWARD_IN1 != 0 && CAR_MOTOR_B_FORWARD_IN1 != 1) || \
    (CAR_CALIBRATION_CONFIRMED != 0 && CAR_CALIBRATION_CONFIRMED != 1)
#error "Boolean board configuration must be 0 or 1"
#endif
#if CAR_GRAY_SETTLE_MS == 0 || CAR_GRAY_RESPONSE_TIMEOUT_MS <= CAR_GRAY_SETTLE_MS || \
    CAR_GRAY_STALE_TIMEOUT_MS <= CAR_GRAY_RESPONSE_TIMEOUT_MS
#error "Invalid grayscale timing"
#endif
#endif
