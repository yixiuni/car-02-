#include "encoder.h"
#include "board_c10b.h"
#include "car_config.h"
#include <math.h>

static void TimerInit(TIM_TypeDef *timer)
{
    timer->CR1=0; timer->CCER=0; timer->PSC=0; timer->ARR=65535;
    /* Encoder mode 3: both inputs, TI1/TI2 direct, digital input filter fDTS/4 N=6. */
    timer->CCMR1=TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | (6U<<4) | (6U<<12);
    timer->SMCR=TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    timer->CCER=TIM_CCER_CC1E | TIM_CCER_CC2E;
    timer->EGR=TIM_EGR_UG; timer->CNT=0; timer->SR=0;
    timer->CR1=TIM_CR1_CEN;
}
void Encoder_Init(Encoder *e,uint32_t now)
{
    *e=(Encoder){0}; e->last_at=now;
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_AFIO_REMAP_TIM4_DISABLE();
    __HAL_RCC_TIM4_CLK_ENABLE(); __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_TIM4_FORCE_RESET(); __HAL_RCC_TIM4_RELEASE_RESET();
    __HAL_RCC_TIM8_FORCE_RESET(); __HAL_RCC_TIM8_RELEASE_RESET();
    GPIO_InitTypeDef g={0}; g.Mode=GPIO_MODE_INPUT; g.Pull=GPIO_PULLUP;
    g.Pin=BOARD_ENCODER_A_CH1_PIN | BOARD_ENCODER_A_CH2_PIN;
    HAL_GPIO_Init(BOARD_ENCODER_A_PORT,&g);
    g.Pin=BOARD_ENCODER_B_CH1_PIN | BOARD_ENCODER_B_CH2_PIN;
    HAL_GPIO_Init(BOARD_ENCODER_B_PORT,&g);
    TimerInit(TIM4); TimerInit(TIM8);
}
void Encoder_Update(Encoder *e,const CarGeometry *g,uint32_t now)
{
    uint32_t elapsed=now-e->last_at;
    if (elapsed<CAR_CONTROL_PERIOD_MS) return;
    uint16_t a=(uint16_t)TIM4->CNT, b=(uint16_t)TIM8->CNT;
    int32_t da=Encoder_CounterDelta(a,e->previous_a)*CAR_ENCODER_A_SIGN;
    int32_t db=Encoder_CounterDelta(b,e->previous_b)*CAR_ENCODER_B_SIGN;
    e->previous_a=a; e->previous_b=b; e->last_at=now;
    e->counts_a+=da; e->counts_b+=db;
    e->valid=CarGeometry_Valid(g) && elapsed<=CAR_CONTROL_MAX_GAP_MS;
    if (!e->valid) { e->left_speed=e->right_speed=0; return; }
    float ka=CAR_PI*g->diameter_mm/g->counts_a_rev, kb=CAR_PI*g->diameter_mm/g->counts_b_rev;
    float sa=(float)da*ka*1000/(float)elapsed, sb=(float)db*kb*1000/(float)elapsed;
    if (fabsf(sa)>2000 || fabsf(sb)>2000) { e->valid=false; return; }
    float alpha=(float)elapsed/(30.0f+(float)elapsed);
    float left=CAR_LEFT_IS_MOTOR_A ? sa : sb, right=CAR_LEFT_IS_MOTOR_A ? sb : sa;
    e->left_speed+=alpha*(left-e->left_speed); e->right_speed+=alpha*(right-e->right_speed);
    float am=(float)e->counts_a*ka, bm=(float)e->counts_b*kb;
    e->left_mm=CAR_LEFT_IS_MOTOR_A ? am : bm;
    e->right_mm=CAR_LEFT_IS_MOTOR_A ? bm : am;
}
