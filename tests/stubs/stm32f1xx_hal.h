#ifndef TEST_STM32F1XX_HAL_H
#define TEST_STM32F1XX_HAL_H
/* Host-only register storage for the real encoder.c. No GPIO/edge simulation. */
#include <stdint.h>
typedef struct {
    volatile uint32_t CR1, CCER, PSC, ARR, CCMR1, SMCR, EGR, CNT, SR;
} TIM_TypeDef;
extern TIM_TypeDef test_tim4, test_tim8;
#define TIM4 (&test_tim4)
#define TIM8 (&test_tim8)
typedef struct { uint32_t Pin, Mode, Pull; } GPIO_InitTypeDef;
#define GPIOB ((void *)0)
#define GPIOC ((void *)0)
#define GPIO_PIN_6 (1U<<6)
#define GPIO_PIN_7 (1U<<7)
#define GPIO_MODE_INPUT 0U
#define GPIO_PULLUP 1U
#define TIM_CCMR1_CC1S_0 1U
#define TIM_CCMR1_CC2S_0 (1U<<8)
#define TIM_SMCR_SMS_0 1U
#define TIM_SMCR_SMS_1 2U
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC2E (1U<<4)
#define TIM_EGR_UG 1U
#define TIM_CR1_CEN 1U
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_AFIO_CLK_ENABLE() ((void)0)
#define __HAL_AFIO_REMAP_TIM4_DISABLE() ((void)0)
#define __HAL_RCC_TIM4_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM8_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM4_FORCE_RESET() ((void)0)
#define __HAL_RCC_TIM4_RELEASE_RESET() ((void)0)
#define __HAL_RCC_TIM8_FORCE_RESET() ((void)0)
#define __HAL_RCC_TIM8_RELEASE_RESET() ((void)0)
static inline void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config)
{ (void)port; (void)config; }
#endif
