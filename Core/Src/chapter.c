#include "main.h"
#include "chapter.h"
#include "car_config.h"
#include "car_control.h"

/* 可在调试器中查看 state、completed_laps、left_pwm、right_pwm。 */
static CarControl car;

static bool ReadButton(void)
{
    return HAL_GPIO_ReadPin(CAR_BUTTON_PORT, CAR_BUTTON_PIN) == GPIO_PIN_RESET;
}

static uint8_t ReadBlackMask(void)
{
    static const uint16_t sensor_pins[5] = {
        CAR_SENSOR_LEFT_OUTER_PIN, CAR_SENSOR_LEFT_PIN, CAR_SENSOR_CENTER_PIN,
        CAR_SENSOR_RIGHT_PIN, CAR_SENSOR_RIGHT_OUTER_PIN
    };
    /* 一次读取端口，五路传感器使用同一时刻的快照。 */
    uint32_t levels = CAR_SENSOR_PORT->IDR;
    uint8_t mask = 0U;

    for (uint8_t i = 0U; i < 5U; ++i)
    {
        bool high = (levels & sensor_pins[i]) != 0U;
        bool black = CAR_SENSOR_BLACK_IS_LOW ? !high : high;
        if (black)
        {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

static uint32_t ScaledDuty(uint16_t duty, uint32_t scale_percent)
{
    uint32_t scaled = (uint32_t)duty * scale_percent / 100U;
    if (scaled > CAR_PWM_MAX)
    {
        scaled = CAR_PWM_MAX;
    }
    return (TIM3->ARR + 1U) * scaled / CAR_PWM_MAX;
}

static void MotorApply(uint16_t left, uint16_t right)
{
    /* TB6612FNG：IN1=IN2=1 是短路制动；停车时持续保持制动。 */
    HAL_GPIO_WritePin(CAR_MOTOR_PORT, CAR_LEFT_IN1_PIN,
                     (left == 0U || CAR_LEFT_FORWARD_IN1_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAR_MOTOR_PORT, CAR_LEFT_IN2_PIN,
                     (left == 0U || !CAR_LEFT_FORWARD_IN1_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAR_MOTOR_PORT, CAR_RIGHT_IN1_PIN,
                     (right == 0U || CAR_RIGHT_FORWARD_IN1_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAR_MOTOR_PORT, CAR_RIGHT_IN2_PIN,
                     (right == 0U || !CAR_RIGHT_FORWARD_IN1_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    TIM3->CCR1 = ScaledDuty(left, CAR_LEFT_SCALE_PERCENT);
    TIM3->CCR2 = ScaledDuty(right, CAR_RIGHT_SCALE_PERCENT);
    HAL_GPIO_WritePin(CAR_MOTOR_PORT, CAR_STANDBY_PIN, GPIO_PIN_SET);
}

static void HardwareInit(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t timer_clock;
    uint32_t period;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* TIM3 使用默认 PA6/PA7；外设重映射后最后明确保留 SWD。 */
    __HAL_AFIO_REMAP_TIM3_DISABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    HAL_GPIO_WritePin(CAR_MOTOR_PORT,
                     CAR_LEFT_IN1_PIN | CAR_LEFT_IN2_PIN | CAR_RIGHT_IN1_PIN |
                     CAR_RIGHT_IN2_PIN | CAR_STANDBY_PIN, GPIO_PIN_RESET);
    gpio.Pin = CAR_LEFT_IN1_PIN | CAR_LEFT_IN2_PIN | CAR_RIGHT_IN1_PIN |
               CAR_RIGHT_IN2_PIN | CAR_STANDBY_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAR_MOTOR_PORT, &gpio);

    gpio.Pin = CAR_SENSOR_PINS;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(CAR_SENSOR_PORT, &gpio);
    gpio.Pin = CAR_BUTTON_PIN;
    HAL_GPIO_Init(CAR_BUTTON_PORT, &gpio);

    HAL_GPIO_WritePin(CAR_LED_PORT, CAR_LED_PIN, GPIO_PIN_SET);
    gpio.Pin = CAR_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAR_LED_PORT, &gpio);

    /* 不依赖项目中尚未引入的 HAL_TIM 驱动，使用现有 CMSIS 寄存器定义。 */
    timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        timer_clock *= 2U;
    }
    period = timer_clock / CAR_PWM_FREQUENCY_HZ;
    if (period < 2U || period > 65536U)
    {
        /* 此时 STBY 仍为低，电机尚未使能。 */
        Error_Handler();
    }
    __HAL_RCC_TIM3_FORCE_RESET();
    __HAL_RCC_TIM3_RELEASE_RESET();
    TIM3->PSC = 0U;
    TIM3->ARR = period - 1U;   /* 72 MHz / 3600 = 20 kHz。 */
    TIM3->CCR1 = 0U;
    TIM3->CCR2 = 0U;
    TIM3->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;

    gpio.Pin = CAR_PWM_LEFT_PIN | CAR_PWM_RIGHT_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAR_PWM_PORT, &gpio);
    TIM3->CR1 |= TIM_CR1_CEN;
    MotorApply(0U, 0U);
}

void chapter_main(void)
{
    HardwareInit();
    CarControl_Init(&car, HAL_GetTick(), ReadButton());

    while (1)
    {
        uint32_t now = HAL_GetTick();
        CarControl_Update(&car, now, ReadBlackMask(), ReadButton());
        MotorApply(car.left_pwm, car.right_pwm);

        /* LED：停止/完成时熄灭，运行时常亮，丢线或不在起点时闪烁。 */
        bool led_on = (car.state == CAR_RUNNING) ||
                      ((car.state == CAR_FAULT_LINE_LOST || car.state == CAR_START_REQUIRED) &&
                       (now / 100U) % 2U == 0U);
        HAL_GPIO_WritePin(CAR_LED_PORT, CAR_LED_PIN,
                         led_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_Delay(1U);
    }
}
