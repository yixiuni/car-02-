#include "oled.h"
#include "board_c10b.h"

volatile OledDiagnostics oled_diagnostics;
static uint8_t frame[1024];
static unsigned offset=1024;
static uint32_t rendered_at;
static bool first=true;
static void HalfClock(void)
{
    uint32_t start=DWT->CYCCNT;
    while((uint32_t)(DWT->CYCCNT-start)<SystemCoreClock/500000U) {}
}
static void Byte(uint8_t value,bool data)
{
    BOARD_OLED_PORT->BSRR=data ? BOARD_OLED_DC_PIN : (uint32_t)BOARD_OLED_DC_PIN<<16;
    for(unsigned bit=0;bit<8;bit++) {
        BOARD_OLED_CLOCK_PORT->BSRR=(uint32_t)BOARD_OLED_CLOCK_PIN<<16;
        BOARD_OLED_PORT->BSRR=(value&0x80) ? BOARD_OLED_DATA_PIN : (uint32_t)BOARD_OLED_DATA_PIN<<16;
        HalfClock();
        BOARD_OLED_CLOCK_PORT->BSRR=BOARD_OLED_CLOCK_PIN;
        HalfClock();value<<=1;
    }
}
void Oled_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE();
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    GPIO_InitTypeDef gpio={.Pin=BOARD_OLED_DC_PIN|BOARD_OLED_RST_PIN|BOARD_OLED_DATA_PIN,
        .Mode=GPIO_MODE_OUTPUT_PP,.Pull=GPIO_NOPULL,.Speed=GPIO_SPEED_FREQ_LOW};
    HAL_GPIO_WritePin(BOARD_OLED_PORT,gpio.Pin,GPIO_PIN_RESET);
    HAL_GPIO_Init(BOARD_OLED_PORT,&gpio);
    gpio.Pin=BOARD_OLED_CLOCK_PIN;
    HAL_GPIO_WritePin(BOARD_OLED_CLOCK_PORT,gpio.Pin,GPIO_PIN_RESET);
    HAL_GPIO_Init(BOARD_OLED_CLOCK_PORT,&gpio);
    HAL_Delay(20);
    HAL_GPIO_WritePin(BOARD_OLED_PORT,BOARD_OLED_RST_PIN,GPIO_PIN_SET);
    HAL_Delay(20);
    static const uint8_t init[]={0xae,0xd5,0x80,0xa8,0x3f,0xd3,0,0x40,
        0x8d,0x14,0x20,0x02,0xa1,0xc8,0xda,0x12,0x81,0x8f,
        0xd9,0xf1,0xdb,0x30,0xa4,0xa6};
    for(unsigned i=0;i<sizeof(init);i++) Byte(init[i],false);
    offset=1024;first=true;rendered_at=0;
    oled_diagnostics=(OledDiagnostics){0};
    /* Display stays off until the first complete frame replaces power-on RAM. */
}
void Oled_Poll(uint32_t now,const OledView *view)
{
    uint32_t start=DWT->CYCCNT;
    if(offset==1024) {
        if(!first && now-rendered_at<200U) return;
        OledUI_Render(view,frame);offset=0;rendered_at=now;first=false;
        /* Rendering and SPI each have a separate bounded loop iteration. */
    } else {
        if(offset%128==0) {
            Byte((uint8_t)(0xb0+offset/128),false);Byte(0,false);Byte(0x10,false);
        }
        for(unsigned i=0;i<16;i++) Byte(frame[offset++],true);
        if(offset==1024) {
            if(!oled_diagnostics.frames) Byte(0xaf,false);
            ++oled_diagnostics.frames;
        }
    }
    uint32_t cycles=DWT->CYCCNT-start;
    if(cycles>oled_diagnostics.max_poll_cycles) oled_diagnostics.max_poll_cycles=cycles;
}
