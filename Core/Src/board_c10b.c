#include "board_c10b.h"
#include "car_config.h"

void Board_EmergencyStop(void)
{
    /* Works before HAL/Board_Init too. Force both AT8236 inputs low: coast/sleep. */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB2ENR;
    (void)RCC->APB1ENR;
    TIM3->CCER = 0U;
    TIM3->CR1 = 0U;
    GPIOA->BRR = BOARD_MOTOR_A_IN1_PIN | BOARD_MOTOR_A_IN2_PIN;
    GPIOB->BRR = BOARD_MOTOR_B_IN1_PIN | BOARD_MOTOR_B_IN2_PIN;
    GPIOA->CRL = (GPIOA->CRL & ~0xFF000000UL) | 0x22000000UL;
    GPIOB->CRL = (GPIOB->CRL & ~0x000000FFUL) | 0x00000022UL;
}

void Board_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t timer_clock, period;
    Board_EmergencyStop();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_TIM3_DISABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); /* retain PA13/PA14 SWD */

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Pin = BOARD_BUTTON_PIN;
    HAL_GPIO_Init(BOARD_BUTTON_PORT, &gpio);
    gpio.Pull = GPIO_PULLUP;
    gpio.Pin = BOARD_ENABLE_PIN;
    HAL_GPIO_Init(BOARD_ENABLE_PORT, &gpio);

    HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_RUN_PIN | BOARD_LED_FAULT_PIN,
                      GPIO_PIN_SET);
    gpio.Pin = BOARD_LED_RUN_PIN | BOARD_LED_FAULT_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_LED_PORT, &gpio);

    gpio.Pin = BOARD_BUZZER_PIN;
    HAL_GPIO_WritePin(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN,
                     CAR_BUZZER_ACTIVE_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_Init(BOARD_BUZZER_PORT, &gpio);

    timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        timer_clock *= 2U;
    }
    period = timer_clock / CAR_PWM_FREQUENCY_HZ;
    if (period < 2U || period > 65536U) { Error_Handler(); }
    __HAL_RCC_TIM3_FORCE_RESET();
    __HAL_RCC_TIM3_RELEASE_RESET();
    TIM3->PSC = 0U;
    TIM3->ARR = period - 1U;
    TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 0U;
    TIM3->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM3->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 |
                  TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = BOARD_MOTOR_A_IN1_PIN | BOARD_MOTOR_A_IN2_PIN;
    HAL_GPIO_Init(BOARD_MOTOR_A_PORT, &gpio);
    gpio.Pin = BOARD_MOTOR_B_IN1_PIN | BOARD_MOTOR_B_IN2_PIN;
    HAL_GPIO_Init(BOARD_MOTOR_B_PORT, &gpio);
    TIM3->CR1 |= TIM_CR1_CEN;
}

bool Board_ButtonPressed(void)
{
    return HAL_GPIO_ReadPin(BOARD_BUTTON_PORT, BOARD_BUTTON_PIN) == GPIO_PIN_SET;
}

bool Board_MotionEnabled(void)
{
    return HAL_GPIO_ReadPin(BOARD_ENABLE_PORT, BOARD_ENABLE_PIN) == GPIO_PIN_RESET;
}

static uint32_t ScaledDuty(uint16_t duty, uint32_t scale)
{
    uint32_t scaled = (uint32_t)duty * scale / 100U;
    if (scaled > CAR_PWM_MAX) { scaled = CAR_PWM_MAX; }
    return (TIM3->ARR + 1U) * scaled / CAR_PWM_MAX;
}

void Board_MotorApply(uint16_t left, uint16_t right)
{
    uint32_t a, b;
    if (!Board_MotionEnabled()) { left = right = 0U; }
    a = ScaledDuty(CAR_LEFT_IS_MOTOR_A ? left : right,
                   CAR_LEFT_IS_MOTOR_A ? CAR_LEFT_SCALE_PERCENT : CAR_RIGHT_SCALE_PERCENT);
    b = ScaledDuty(CAR_LEFT_IS_MOTOR_A ? right : left,
                   CAR_LEFT_IS_MOTOR_A ? CAR_RIGHT_SCALE_PERCENT : CAR_LEFT_SCALE_PERCENT);
    /* AT8236 has two PWM inputs per motor; no TB6612 STBY/direction GPIOs. */
    TIM3->CCR1 = CAR_MOTOR_A_FORWARD_IN1 ? a : 0U;
    TIM3->CCR2 = CAR_MOTOR_A_FORWARD_IN1 ? 0U : a;
    TIM3->CCR3 = CAR_MOTOR_B_FORWARD_IN1 ? b : 0U;
    TIM3->CCR4 = CAR_MOTOR_B_FORWARD_IN1 ? 0U : b;
    /* Commit all four preloaded channels on the same update, including stop. */
    TIM3->EGR = TIM_EGR_UG;
}

void Board_SetLeds(bool running, bool fault)
{
    HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_RUN_PIN,
                      running ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_FAULT_PIN,
                      fault ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Board_SetBuzzer(bool on)
{
    HAL_GPIO_WritePin(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN,
        (on == (CAR_BUZZER_ACTIVE_HIGH != 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
