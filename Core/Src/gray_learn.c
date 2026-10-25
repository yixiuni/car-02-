#include "car_config.h"
#if CAR_GRAY_LEARN_BUILD
#include "gray_learn.h"
#include "gray_analog.h"
#include "board_c10b.h"
#include "hiwonder_gray.h"

/* Dedicated no-motion image. Only sensor READ commands 0/1/2/3 are sent.
 * USER/enable have no start path. Capture requests only reset sample storage. */
typedef struct {uint32_t now_ms; uint16_t value[8];} GrayLearnSample;
volatile uint32_t gray_learn_request;
volatile struct {
    uint32_t magic,now_ms,analog_frames,threshold_frames,digital_frames;
    uint32_t timeouts,errors,uart_errors,unexpected_bytes,capture_count;
    uint32_t command,last_size,raw_status,capture_generation,sequence;
    uint16_t analog[8],threshold[8];
    uint8_t last_reply[64];
} gray_learn_diagnostics;
volatile GrayLearnSample gray_learn_samples[128];
static volatile uint8_t rx[64];
static volatile uint32_t rx_count,rx_at,rx_errors;
static volatile bool waiting;

void GrayLearn_OnByte(uint8_t byte,bool error)
{
    if (error) {++rx_errors; ++gray_learn_diagnostics.uart_errors;}
    if (!waiting) {++gray_learn_diagnostics.unexpected_bytes; return;}
    if (rx_count<sizeof(rx)) rx[rx_count++]=byte;
    else ++rx_errors;
    rx_at=HAL_GetTick();
}

static void WatchdogStart(void)
{
    DBGMCU->CR &= ~DBGMCU_CR_DBG_IWDG_STOP;
    IWDG->KR=0xCCCCU; IWDG->KR=0x5555U;
    IWDG->PR=3U; IWDG->RLR=124U;
    while (IWDG->SR!=0U) {}
    IWDG->KR=0xAAAAU;
}

void GrayLearn_Run(void)
{
    Board_Init(); Board_EmergencyStop(); Board_SetBuzzer(false);
    uint32_t boot=HAL_GetTick(),fed_at=boot;
    HiwonderGray_Init(boot); /* UART only: never call digital-mode Poll here. */
    WatchdogStart();
    gray_learn_diagnostics.magic=0x474C524EU;
    uint32_t mode_at=0,request_at=0,last_end=0,query=0;
    uint8_t command=0;
    bool mode_sent=false;
    while (1) {
        Board_EmergencyStop(); /* TIM3 disabled and driver pins forced LOW. */
        DBGMCU->CR &= ~DBGMCU_CR_DBG_IWDG_STOP;
        uint32_t now=HAL_GetTick();
        gray_learn_diagnostics.now_ms=now;
        if (gray_learn_request) {
            uint32_t r=gray_learn_request; gray_learn_request=0;
            if (r==1) {
                gray_learn_diagnostics.capture_count=0;
                ++gray_learn_diagnostics.capture_generation;
                /* Do not include a reply requested before the new window. */
                uint32_t pm=__get_PRIMASK();__disable_irq();
                waiting=false;rx_count=0;rx_errors=0;
                __set_PRIMASK(pm);last_end=now;
            }
        }
        if (!mode_sent && now-boot>=1000 && (UART4->SR&USART_SR_TXE)) {
            UART4->DR=0;mode_sent=true;mode_at=last_end=now;
        }
        if (waiting && ((rx_count && now-rx_at>=3) || now-request_at>=40)) {
            uint8_t bytes[64]; uint32_t n,e;
            uint32_t pm=__get_PRIMASK();__disable_irq();
            waiting=false;n=rx_count;e=rx_errors;
            for (uint32_t i=0;i<n;++i) bytes[i]=rx[i];
            __set_PRIMASK(pm);
            gray_learn_diagnostics.last_size=n;
            for(uint32_t i=0;i<64;++i) gray_learn_diagnostics.last_reply[i]=i<n?bytes[i]:0;
            uint16_t values[8];
            ++gray_learn_diagnostics.sequence;
            if (!n) ++gray_learn_diagnostics.timeouts;
            else if (!e && command==1 && n==1) {
                gray_learn_diagnostics.raw_status=bytes[0];
                ++gray_learn_diagnostics.digital_frames;
            } else if (!e && GrayAnalog_Decode(bytes,n,command,values)) {
                if (command==2) {
                    for(unsigned i=0;i<8;++i) gray_learn_diagnostics.analog[i]=values[i];
                    ++gray_learn_diagnostics.analog_frames;
                    uint32_t at=gray_learn_diagnostics.capture_count;
                    if (at<128) {
                        gray_learn_samples[at].now_ms=now;
                        for(unsigned i=0;i<8;++i) gray_learn_samples[at].value[i]=values[i];
                        gray_learn_diagnostics.capture_count=at+1;
                    }
                } else {
                    for(unsigned i=0;i<8;++i) gray_learn_diagnostics.threshold[i]=values[i];
                    ++gray_learn_diagnostics.threshold_frames;
                }
            } else ++gray_learn_diagnostics.errors;
            ++gray_learn_diagnostics.sequence;last_end=now;
        }
        if (mode_sent && now-mode_at>=50 && !waiting && now-last_end>=20 &&
            (UART4->SR&USART_SR_TXE)) {
            /* Mostly analog, with threshold and digital cross-checks. */
            command=(query%16==0)?3:(query%16==1)?1:2;++query;
            uint32_t pm=__get_PRIMASK();__disable_irq();
            rx_count=rx_errors=0;waiting=true;request_at=now;
            gray_learn_diagnostics.command=command;
            UART4->DR=command;__set_PRIMASK(pm);
        }
        Board_SetLeds(gray_learn_diagnostics.capture_count>=128,
                      gray_learn_diagnostics.errors || gray_learn_diagnostics.timeouts);
        if(now!=fed_at){IWDG->KR=0xAAAAU;fed_at=now;}
        HAL_Delay(1);
    }
}
#endif
