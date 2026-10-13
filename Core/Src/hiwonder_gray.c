#include "hiwonder_gray.h"
#include "board_c10b.h"
#include "car_config.h"

static volatile GrayProtocol gray;
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
        GrayProtocol_Poll(&gray, now);
        if (GrayProtocol_CanRequest(&gray, now) &&
            (UART4->SR & USART_SR_TXE) != 0U)
        {
            GrayProtocol_BeginRequest(&gray, now);
            UART4->DR = 0x01U; /* request S1..S8 digital state */
        }
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
        GrayProtocol_OnByte(&gray, HAL_GetTick(), byte, (status & errors) != 0U);
    }
}

void HiwonderGray_Snapshot(GrayProtocol *snapshot)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = gray;
    __set_PRIMASK(primask);
}
