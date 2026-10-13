#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* PWM permille is a motor command, not measured speed. */
#define CAR_PWM_MAX                 1000U
#define CAR_PWM_FREQUENCY_HZ        20000U
#define CAR_BASE_SPEED              360
#define CAR_MAX_SPEED               700
#define CAR_SEARCH_SPEED            220
#define CAR_WIDE_LINE_SPEED         250
#define CAR_KP                      2
#define CAR_KD                      1
#define CAR_TURN_SLOWDOWN_DIVISOR    3
#define CAR_CONTROL_PERIOD_MS       5U
#define CAR_BUTTON_DEBOUNCE_MS      25U
#define CAR_LINE_LOST_TIMEOUT_MS    600U

/* Preserve existing lap mode. 0 selects continuous/manual-stop operation. */
#ifndef CAR_TARGET_LAPS
#define CAR_TARGET_LAPS             1U
#endif
#define CAR_MARKER_DEBOUNCE_MS      15U
#define CAR_MARKER_CLEAR_MS         100U
#define CAR_MIN_LAP_TIME_MS         1000U
#define CAR_SENSOR_COUNT            8U
#define CAR_MARKER_MASK             0xFFU

/* UART status is opposite to the module's GPIO levels: 1 = learned target.
   Calibrate black as the target. Reverse order if S1 is on the right. */
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

/* Physical left/right and forward direction require a wheels-raised check.
   A = CN3/PA6-PA7, B = CN2/PB0-PB1. These do not change the board pin map. */
#define CAR_LEFT_IS_MOTOR_A         1
#define CAR_MOTOR_A_FORWARD_IN1     1
#define CAR_MOTOR_B_FORWARD_IN1     1
#define CAR_LEFT_SCALE_PERCENT      100U
#define CAR_RIGHT_SCALE_PERCENT     100U

#if (CAR_PWM_FREQUENCY_HZ == 0U)
#error "PWM frequency must be positive"
#endif
#if (CAR_BASE_SPEED <= 0) || (CAR_BASE_SPEED > CAR_MAX_SPEED) || \
    (CAR_MAX_SPEED > CAR_PWM_MAX) || (CAR_SEARCH_SPEED <= 0) || \
    (CAR_SEARCH_SPEED > CAR_MAX_SPEED) || (CAR_WIDE_LINE_SPEED <= 0) || \
    (CAR_WIDE_LINE_SPEED > CAR_MAX_SPEED) || (CAR_KP < 0) || (CAR_KD < 0) || \
    (CAR_TURN_SLOWDOWN_DIVISOR <= 0)
#error "Invalid car speed or steering configuration"
#endif
#if (CAR_CONTROL_PERIOD_MS == 0U) || (CAR_BUTTON_DEBOUNCE_MS == 0U) || \
    (CAR_LINE_LOST_TIMEOUT_MS <= CAR_CONTROL_PERIOD_MS)
#error "Invalid car timing configuration"
#endif
#if (CAR_TARGET_LAPS > 255U) || (CAR_MARKER_DEBOUNCE_MS == 0U) || \
    (CAR_MARKER_CLEAR_MS < CAR_MARKER_DEBOUNCE_MS) || \
    (CAR_MIN_LAP_TIME_MS < CAR_MARKER_CLEAR_MS)
#error "Invalid finish-line or lap configuration"
#endif
#if (CAR_LEFT_SCALE_PERCENT == 0U) || (CAR_LEFT_SCALE_PERCENT > 200U) || \
    (CAR_RIGHT_SCALE_PERCENT == 0U) || (CAR_RIGHT_SCALE_PERCENT > 200U)
#error "Motor scaling must be between 1 and 200 percent"
#endif
#if (CAR_GRAY_SETTLE_MS == 0U) || \
    (CAR_GRAY_RESPONSE_TIMEOUT_MS <= CAR_GRAY_SETTLE_MS) || \
    (CAR_GRAY_STALE_TIMEOUT_MS <= CAR_GRAY_RESPONSE_TIMEOUT_MS)
#error "Invalid grayscale communication timing"
#endif
#if (CAR_LEFT_IS_MOTOR_A != 0 && CAR_LEFT_IS_MOTOR_A != 1) || \
    (CAR_MOTOR_A_FORWARD_IN1 != 0 && CAR_MOTOR_A_FORWARD_IN1 != 1) || \
    (CAR_MOTOR_B_FORWARD_IN1 != 0 && CAR_MOTOR_B_FORWARD_IN1 != 1) || \
    (CAR_GRAY_TARGET_BIT_IS_ONE != 0 && CAR_GRAY_TARGET_BIT_IS_ONE != 1) || \
    (CAR_GRAY_REVERSE_ORDER != 0 && CAR_GRAY_REVERSE_ORDER != 1)
#error "Board orientation and polarity settings must be 0 or 1"
#endif
#endif
