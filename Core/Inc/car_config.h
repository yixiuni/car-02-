#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* PWM 使用千分比：360 表示 36%，不是闭环测得的车轮转速。 */
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

/* 模块在黑线上输出低电平时为 1，高电平时改为 0。 */
#define CAR_SENSOR_BLACK_IS_LOW     1
/* 某个车轮方向反了，就把对应的 1 改为 0。 */
#define CAR_LEFT_FORWARD_IN1_HIGH   1
#define CAR_RIGHT_FORWARD_IN1_HIGH  1
/* 用于补偿左右电机差异，100 表示保持原占空比。 */
#define CAR_LEFT_SCALE_PERCENT      100U
#define CAR_RIGHT_SCALE_PERCENT     100U

/* 从车头朝前看，传感器按最左到最右排列；安装于后轮前方。 */
#define CAR_SENSOR_PORT             GPIOA
#define CAR_SENSOR_LEFT_OUTER_PIN   GPIO_PIN_0
#define CAR_SENSOR_LEFT_PIN         GPIO_PIN_1
#define CAR_SENSOR_CENTER_PIN       GPIO_PIN_2
#define CAR_SENSOR_RIGHT_PIN        GPIO_PIN_3
#define CAR_SENSOR_RIGHT_OUTER_PIN  GPIO_PIN_4
#define CAR_SENSOR_PINS             (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | \
                                     GPIO_PIN_3 | GPIO_PIN_4)

/* PA6/PA7 固定对应未重映射的 TIM3_CH1/CH2，修改 PWM 接线须同步改定时器。 */
#define CAR_PWM_PORT                GPIOA
#define CAR_PWM_LEFT_PIN            GPIO_PIN_6
#define CAR_PWM_RIGHT_PIN           GPIO_PIN_7
#define CAR_MOTOR_PORT              GPIOB
#define CAR_LEFT_IN1_PIN            GPIO_PIN_0
#define CAR_LEFT_IN2_PIN            GPIO_PIN_1
#define CAR_RIGHT_IN1_PIN           GPIO_PIN_10
#define CAR_RIGHT_IN2_PIN           GPIO_PIN_11
#define CAR_STANDBY_PIN             GPIO_PIN_12
#define CAR_BUTTON_PORT             GPIOB
#define CAR_BUTTON_PIN              GPIO_PIN_5
#define CAR_LED_PORT                GPIOC
#define CAR_LED_PIN                 GPIO_PIN_13

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
#if (CAR_LEFT_SCALE_PERCENT == 0U) || (CAR_LEFT_SCALE_PERCENT > 200U) || \
    (CAR_RIGHT_SCALE_PERCENT == 0U) || (CAR_RIGHT_SCALE_PERCENT > 200U)
#error "Motor scaling must be between 1 and 200 percent"
#endif

#endif /* CAR_CONFIG_H */
