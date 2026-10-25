#include "hiwonder_gray.h"
#include "board_c10b.h"
#include "car_config.h"
#include "gray_learn.h"
static volatile GrayProtocol gray;
#if CAR_GRAY_USE_ANALOG && !CAR_GRAY_LEARN_BUILD
#include "gray_analog.h"
#include "gray_calibration.h"
#if GRAY_CALIBRATION_VALID_MASK != 0xFFU
#error "All eight grayscale channels must be calibrated"
#endif
static volatile GrayAnalogReceiver analog;
volatile uint16_t hiwonder_gray_analog_values[8];
static uint8_t analog_mask;
static void SyncAnalog(uint32_t now)
{
    GrayAnalog_Poll(&analog,now);
    if(gray.samples!=analog.samples){
        uint16_t values[8];
        for(unsigned i=0;i<8;++i) values[i]=hiwonder_gray_analog_values[i]=analog.values[i];
        analog_mask=GrayAnalog_Classify(values,gray_calibration_threshold,
            gray_calibration_hysteresis,GRAY_CALIBRATION_BLACK_HIGH_MASK,analog_mask);
        /* Keep the existing snapshot convention; this is a synthesized digital
         * value, not the sensor's internal threshold result in analog mode. */
        gray.raw_status=CAR_GRAY_TARGET_BIT_IS_ONE?analog_mask:(uint8_t)~analog_mask;
    }
    gray.valid=analog.valid;gray.last_valid_at=analog.last_valid_at;
    gray.samples=analog.samples;gray.timeouts=analog.timeouts;
    gray.errors=analog.errors;gray.unexpected_bytes=analog.unexpected_bytes;
}
#endif

static uint32_t boot_at, mode_at;
static bool mode_sent;

void HiwonderGray_Init(uint32_t now)
{
    GPIO_InitTypeDef gpio = {0};
    HAL_NVIC_DisableIRQ(UART4_IRQn);
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_UART4_FORCE_RESET();
    __HAL_RCC_UART4_RELEASE_RESET();
    gpio.Pin = BOARD_GRAY_TX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BOARD_GRAY_PORT, &gpio);
    gpio.Pin = BOARD_GRAY_RX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_GRAY_PORT, &gpio);
    /* F103 USART oversampling by 16: BRR encodes round(PCLK / baud). */
    UART4->BRR = (HAL_RCC_GetPCLK1Freq() + CAR_GRAY_BAUD / 2U) / CAR_GRAY_BAUD;
    UART4->CR2 = 0U; /* one stop bit */
    UART4->CR3 = USART_CR3_EIE;
    GrayProtocol_Init(&gray, now);
#if CAR_GRAY_USE_ANALOG && !CAR_GRAY_LEARN_BUILD
    GrayAnalog_Init(&analog,now);analog_mask=0;
#endif
    boot_at = now;
    mode_sent = false;
    UART4->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(UART4_IRQn, 2U, 0U);
    HAL_NVIC_ClearPendingIRQ(UART4_IRQn);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
}

void HiwonderGray_Poll(uint32_t now)
{
    uint32_t primask;
    if ((uint32_t)(now - boot_at) < CAR_GRAY_BOOT_DELAY_MS) { return; }
    primask = __get_PRIMASK();
    __disable_irq();
    if (!mode_sent)
    {
        if ((UART4->SR & USART_SR_TXE) != 0U)
        {
            UART4->DR = 0x00U; /* manual mode; accepted once per sensor power cycle */
            mode_at = now;
            mode_sent = true;
        }
    }
    else if ((uint32_t)(now - mode_at) >= CAR_GRAY_MODE_DELAY_MS)
    {
#if CAR_GRAY_USE_ANALOG && !CAR_GRAY_LEARN_BUILD
        SyncAnalog(now);
        if (GrayAnalog_CanRequest(&analog,now) && (UART4->SR&USART_SR_TXE)) {
            GrayAnalog_BeginRequest(&analog,now);
            UART4->DR=0x02U;
        }
#else
        GrayProtocol_Poll(&gray, now);
        if (GrayProtocol_CanRequest(&gray, now) &&
            (UART4->SR & USART_SR_TXE) != 0U)
        {
            GrayProtocol_BeginRequest(&gray, now);
            UART4->DR = 0x01U; /* request S1..S8 digital state */
        }
#endif
    }
    __set_PRIMASK(primask);
}

void HiwonderGray_IRQHandler(void)
{
    uint32_t status = UART4->SR;
    uint32_t errors = USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE;
    if ((status & (USART_SR_RXNE | errors)) != 0U)
    {
        /* Reading SR followed by DR clears RXNE and UART error flags on F1. */
        uint8_t byte = (uint8_t)UART4->DR;
#if CAR_GRAY_LEARN_BUILD
        GrayLearn_OnByte(byte,(status & errors)!=0U);
#elif CAR_GRAY_USE_ANALOG
        GrayAnalog_OnByte(&analog,HAL_GetTick(),byte,(status & errors)!=0U);
#else
        GrayProtocol_OnByte(&gray, HAL_GetTick(), byte, (status & errors) != 0U);
#endif
    }
}

void HiwonderGray_Snapshot(GrayProtocol *snapshot)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = gray;
    __set_PRIMASK(primask);
}
